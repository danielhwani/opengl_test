#!/usr/bin/env python3
"""mujoco_ball_sim
------------------
MuJoCo로 공을 하나 떨어뜨려서, 지금 걷고 있는 humanoid와 순찰 중인
4족 로봇의 몸통과 실제로 충돌(접촉)하게 만들고, **맞으면 로봇도
반응하게** 만드는 노드.

humanoid_proxy / quad_proxy는 질량이 있는 진짜 물리 바디다(자유
6DOF). 처음엔 "mocap" 바디(물리 영향을 안 받고 자세만 외부에서
주입받는)로 만들었는데, 그러면 공이 부딪혀도 로봇이 전혀 반응하지
않아서 이렇게 바꿨다. 대신 매 틱, humanoid_state_publisher /
quadruped_state_publisher가 발행하는 "world"->"base_link_target" /
"world"->"quad_base_link_target" TF(걷기/순찰 애니메이션이 "원하는"
순수 목표 자세)를 향해 가상의 스프링-댐퍼 힘/토크를 걸어준다
(data.xfrc_applied) - 그래서:
  - 평소엔 스프링이 그 목표 자세를 그대로 따라가게 만들어서 걷는
    것처럼 보이고,
  - 공에 맞는 순간엔 충돌 충격으로 스프링이 늘어나(목표에서
    벗어나) 흔들린 뒤, 스프링 힘이 다시 목표로 당겨서 돌아온다.

이 노드가 그 물리 시뮬레이션 결과(흔들림 포함)를 실제
"world"->"base_link" / "world"->"quad_base_link" TF로 발행한다 -
robot_state_publisher가 이걸로 전체 로봇(Body/Head/팔다리 등)의
TF를 계산하므로, RViz2에 보이는 로봇 메쉬 자체가 흔들린다. 그래서
humanoid_state_publisher/quadruped_state_publisher는
publish_base_link 파라미터를 False로 받아서(physics_demo.launch.py가
그렇게 띄움) "base_link"/"quad_base_link"를 직접 발행하지 않는다 -
이 노드가 그 프레임의 유일한 발행자가 되게 해서 TF 충돌을 막는다.

스프링 상수(SPRING_KP_POS/HUMANOID_KP_ROT/QUAD_KP_ROT)의 댐핑은
각각 질량/관성을 기준으로 임계댐핑(critical damping)을 계산해서
자동으로 정하지만, "얼마나 세게 흔들렸으면 좋겠는지"를 정하는
비례 계수(kp) 자체와 공/로봇 질량은 결국 실제로 보면서 조절해야
하는 값이다 - 회전 kp/댐핑은 humanoid/quad가 서로 다르다(걷는
경로가 얼마나 급한지가 달라서, 아래 상수 선언부 설명 참고). 쓰러진
뒤 다시 일어서는 건 훨씬 어려운 문제라 그 상황 자체를 피하고, 공에
맞아도 거의 안 쓰러질 만큼 일부러 세게 잡아뒀다. 이 세션에서는
MuJoCo를 직접 돌려볼 수 없어서 수치만 합리적으로 골랐을 뿐 튜닝은
못 했다.

공은 RESPAWN_INTERVAL_SEC마다 주기적으로 다시 떨어뜨린다(둘 중
한쪽 몸통 바로 위에서).

결과는 세 가지로 RViz2에 보여준다:
  - TF: "world" -> "ball" (공의 실제 물리 위치/자세)
  - TF: "world" -> "base_link" / "world" -> "quad_base_link" (로봇의
    물리 반응이 섞인 실제 자세 - robot_state_publisher가 이걸로
    전체 메쉬를 그린다)
  - visualization_msgs/MarkerArray: 공(빨간 구) + 두 충돌 프록시
    (반투명 상자, "왜 거기서 부딪혔는지" 눈으로 보려고)

주의: 이 노드는 humanoid_state_publisher/quadruped_state_publisher가
TF를 발행하고 있어야 제대로 동작한다 (launch/physics_demo.launch.py가
그것까지 같이 띄운다). 그리고 이 노드를 실행하는 Python 환경에
"mujoco" 패키지가 설치돼 있어야 한다.
"""

import os
import random

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory

import mujoco

TIMER_PERIOD_SEC = 0.01     # 100 Hz - 스프링 제어 주기이기도 하다
PHYSICS_SUBSTEPS = 2        # MJCF의 timestep(0.005)과 곱해서 0.01s(=TIMER_
                            # PERIOD_SEC)를 채운다 - 바꾸려면 둘을 같이
                            # 맞춰야 실시간 속도가 안 틀어진다.

FALL_RESET_Z = -2.0         # 이 높이보다 낮아지면 다시 떨어뜨림 (예외 상황 대비)
RESPAWN_INTERVAL_SEC = 3.0  # 이 주기마다 항상 다시 떨어뜨림 (주된 트리거)
RESPAWN_HEIGHT_OFFSET = 2.0  # 고른 몸통 위 이만큼 높은 곳에서 다시 떨어뜨림
RESPAWN_JITTER = 0.3        # 몸통 중심에서 좌우로 흔드는 정도(m)

# "base_link_target"/"quad_base_link_target"은 발 높이(z=0)를
# 가리키는데, 충돌 프록시는 몸통(torso) 자리에 있어야 한다 - 그
# 차이를 더해주는 상수. humanoid.urdf/quadruped.urdf의
# base_to_body/base_to_quadbody 관절 오프셋과 정확히 같은 값이다
# (gen_humanoid_assets.c의 LEG_LEN+BODY_H/2, QUAD_LEG_LEN+QUAD_BODY_H/2).
HUMANOID_BODY_HEIGHT_OFFSET = 1.9   # LEG_LEN(1.2) + BODY_H/2(0.7)
QUAD_BODY_HEIGHT_OFFSET = 0.75      # QUAD_LEG_LEN(0.55) + QUAD_BODY_H/2(0.2)

# 로봇 프록시를 목표 자세로 당기는 가상 스프링-댐퍼 상수. 위치
# 스프링(SPRING_KP_POS)과 회전 스프링(SPRING_KP_ROT) 둘 다, 각각
# 질량/관성으로부터 임계댐핑(critical damping, kd = 2*sqrt(kp*m)
# 또는 kd = 2*sqrt(kp*I))을 계산해서 "진동 없이 가장 빨리 되돌아오는"
# 감쇠 계수를 자동으로 맞춘다(__init__ 참고).
#
# SPRING_KP_ROT을 처음엔 150으로 뒀었는데, 공에 맞아 쓰러진 뒤로
# 다시 못 일어나고 위치 스프링에 끌려 바닥을 기어가는 문제가 있었다
# - 쓰러진 자세에서는 중력+바닥 접촉 마찰이 "다시 세우는" 토크를
# 꽤 세게 버티기 때문에, 150 정도로는 역부족이었다. 그래서 올려서
# 웬만한 충격으로는 아예 안 쓰러지게 하려 했는데, 4000까지 올렸다가
# MuJoCo의 기본(explicit) 적분 방식이 그 뻣뻣함을 못 버티고 값이
# 발산해버렸다(로봇이 "날아가는" 버그) - ball_collision.xml에
# integrator="implicitfast"를 추가해서 더 안정적으로 만들고, kp도
# 800으로 낮췄었다.
#
# 그런데 800은 4족 로봇이 순찰하며 방향을 빠르게(최대 ±90도) 트는
# "정상적인" 애니메이션 목표조차 못 따라가서, 공과 무관하게 계속
# 지연(lag)-흔들림이 생겼다 - 그래서 quad만 1500으로 올렸다. 지금은
# MAX_TORQUE/MAX_ANG_VEL 같은 속도/힘 상한이 안전망으로 있어서,
# 예전처럼 끝까지 발산하지는 않을 것으로 본다.
#
# humanoid는 걷는 경로가 완만해서(Catmull-Rom 스플라인, quad의
# ±90도 급선회 같은 게 없음) 원래 1500까지 올릴 필요가 없었는데,
# 두 로봇이 같은 값을 공유하고 있어서 quad 때문에 올린 값이
# humanoid에는 오히려 과해서(더 높은 고유진동수가 우리의 "평균
# 관성" 댐핑 근사와 덜 맞아서) 공에 맞은 뒤 안 가라앉고 계속
# 떨리는 문제가 생겼다 - 그래서 humanoid/quad를 따로 분리했다.
HUMANOID_KP_ROT = 800.0   # humanoid 회전 스프링 - 경로가 완만해서 이 정도로 충분
QUAD_KP_ROT = 1500.0      # quad 회전 스프링 - 급선회를 따라가려면 더 뻣뻣해야 함
SPRING_KP_POS = 600.0     # N/m (위치 스프링은 공용 - 아직 분리 필요성 못 느낌)
POS_DAMPING_MULTIPLIER = 1.8  # kd_pos(임계댐핑 계산값)에 곱하는 안전 계수.
                        # 회전 스프링과 똑같이, 질량 기준 "순수"
                        # 임계댐핑만으로는 부족해서 과댐핑시켰다
                        # (ros2 bag 분석으로 확인 - 아래 POS_QUAD_DRAG
                        # 주석 참고).
POS_QUAD_DRAG = 20.0    # 선속도 제곱에 비례하는 추가 감쇠(공기저항
                        # 느낌) - ROT_QUAD_DRAG와 같은 이유로 추가.
                        # 공에 맞은 뒤 수평 위치가 끝까지 안 가라앉고
                        # 초속 18m/s급으로 계속 진동하는 게 ros2 bag
                        # 녹화 분석으로 확인됐다(회전/높이는 정상이고
                        # 수평 위치만 문제였음) - 빠르게 움직일 때만
                        # 세게 제동을 걸어 이 진동을 죽이려는 것.

# 실제로 돌려보니 quad_proxy가 방향을 틀기 시작하는 순간부터 각속도
# |w|가 몇 틱 안에 기하급수적으로 폭주했다(0 -> 27 -> 180 -> 103920,
# 틱당 수백 배씩) - 길쭉한 상자(관성이 축마다 꽤 다름, intermediate
# axis 문제)에 평균 관성 하나로만 댐핑을 계산한 선형 PD 제어라, 축
# 간 자이로스코픽 결합을 못 잡아내서 생기는 것으로 보인다. 정확한
# 원인을 더 파기보다, 실제 액추에이터에도 있는 "최대 토크/힘"
# 한계를 그대로 두는 쪽이 더 견고하다 - 어떤 이유로 피드백이
# 불안정해지더라도 이 한계가 폭주를 막아준다.
MAX_FORCE = 20000.0     # N - 5000이었는데, 새로 추가한 위치 과댐핑/
                        # POS_QUAD_DRAG가 관측된 진동 속도(18m/s급)
                        # 에서 내야 하는 제동력(계산상 1만N대)보다
                        # 낮아서 그 효과를 깎아먹고 있었다 - 올려서
                        # 댐핑이 실제로 걸리게 했다.
MAX_TORQUE = 3000.0     # N*m
MAX_LIN_VEL = 15.0      # m/s - 걷기/순찰 속도보다 훨씬 넉넉한 상한
MAX_ANG_VEL = 6.0       # rad/s - "팽이처럼 돈다"는 피드백을 받고 15->6으로 낮춤

# ros2 bag 녹화를 직접 분석해보니(network 접속 없이 파일로만), 공에
# 맞은 뒤 수평 속도가 MAX_LIN_VEL 근처(초속 16~20m/s대)에 계속
# 붙박인 채 녹화가 끝날 때까지(13~17초 내내) 전혀 안 가라앉는
# 경우가 있었다 - 공이 멀리 떨어져 있을 때도(8m+) 똑같이 계속돼서
# 공이 계속 때려서 그런 게 아니라 스프링 피드백 자체의 리밋
# 사이클(limit cycle)로 보인다. 댐핑을 과하게(1.8배) 올려도 안
# 죽어서, 근본 원인을 더 파기보다 "속도가 상한 근처에 너무 오래
# 붙어있으면 그냥 목표 자세로 강제 스냅시켜 끊어주는" 안전장치를
# 뒀다(check_stuck_and_recover 참고).
STUCK_VEL_FRAC = 0.9        # 상한의 이 비율 이상이면 "붙었다"고 봄
STUCK_RESET_SEC = 0.5       # 이 시간 이상 계속 붙어있으면 강제 스냅
ROT_QUAD_DRAG = 15.0    # 각속도 제곱에 비례하는 추가 감쇠(공기저항 느낌) -
                        # 빠르게 돌 때만 세게 제동 걸어서 "살짝 흔들렸다가
                        # 바로 정신 차리는" 느낌을 내려고 추가함
# kd_rot(임계댐핑 계산값)에 곱하는 안전 계수 - humanoid/quad가
# kp_rot도 다르고(위에서 분리함) 체감 반응도 달라서 따로 둔다.
# "계속 떨린다"는 피드백에 2.5까지 올렸었는데, 그러니 공에 맞아도
# 거의 반응이 안 보일 만큼 과도하게 눌렀다 - humanoid는 1.8로
# 낮춰서 "맞은 직후엔 살짝 흔들리다가 시간이 지나면 가라앉는"
# 느낌을 내려고 했다(여전히 critically damped(1.0)보다는 세다).
# quad는 1500처럼 더 뻣뻣한 kp_rot을 쓰니 그와 어울리게 1.3을 유지.
HUMANOID_ROT_DAMPING_MULTIPLIER = 1.8
QUAD_ROT_DAMPING_MULTIPLIER = 1.3


def clamp_vec(v, max_norm):
    """3차원 벡터 v의 크기(norm)가 max_norm을 넘으면 방향은 유지한
    채로 줄인다."""
    norm = np.linalg.norm(v)
    if norm > max_norm:
        return v * (max_norm / norm)
    return v


def default_mjcf_path():
    """설치된 패키지의 share 디렉터리에서 MJCF를 찾는다 (colcon install이
    mujoco/ball_collision.xml을 거기로 복사해둔다 - setup.py의
    data_files 참고). ament 인덱스를 못 찾는 경우(소스 트리에서 직접
    스크립트를 실행하는 등)만 __file__ 기준 상대경로로 fallback한다."""
    try:
        share_dir = get_package_share_directory("humanoid_sim")
        return os.path.join(share_dir, "mujoco", "ball_collision.xml")
    except Exception:
        return os.path.normpath(
            os.path.join(os.path.dirname(__file__), "..", "mujoco", "ball_collision.xml")
        )


def ros_quat_to_mujoco(q):
    """geometry_msgs/Quaternion(x,y,z,w) -> MuJoCo 쿼터니언(w,x,y,z)."""
    return np.array([q.w, q.x, q.y, q.z], dtype=float)


def quat_conj(q):
    """MuJoCo 쿼터니언(w,x,y,z)의 켤레(역회전, 단위 쿼터니언 가정)."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_mul(a, b):
    """두 MuJoCo 쿼터니언(w,x,y,z)의 곱 a*b (먼저 b, 그다음 a를 적용)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_rotate_vec(q, v):
    """MuJoCo 쿼터니언(w,x,y,z) q로 3차원 벡터 v를 회전시킨다 - v를
    순허수 쿼터니언(0,v)으로 보고 q*v*conj(q)를 계산해서 벡터부만
    꺼낸다."""
    v_quat = np.array([0.0, v[0], v[1], v[2]])
    result = quat_mul(quat_mul(q, v_quat), quat_conj(q))
    return result[1:4]


def free_joint_adr(model, body_id):
    """body_id가 가진(유일한) freejoint의 qpos/qvel 시작 주소."""
    jnt_id = model.body_jntadr[body_id]
    return model.jnt_qposadr[jnt_id], model.jnt_dofadr[jnt_id]


class MujocoBallSim(Node):
    def __init__(self):
        super().__init__("mujoco_ball_sim")

        self.declare_parameter("mjcf_path", default_mjcf_path())
        mjcf_path = self.get_parameter("mjcf_path").get_parameter_value().string_value

        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)

        self.ball_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "ball")
        self.humanoid_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "humanoid_proxy")
        self.quad_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "quad_proxy")

        self.ball_qpos_adr, self.ball_qvel_adr = free_joint_adr(self.model, self.ball_body_id)
        self.humanoid_qpos_adr, self.humanoid_qvel_adr = free_joint_adr(self.model, self.humanoid_body_id)
        self.quad_qpos_adr, self.quad_qvel_adr = free_joint_adr(self.model, self.quad_body_id)

        self.humanoid_mass = float(self.model.body_mass[self.humanoid_body_id])
        self.quad_mass = float(self.model.body_mass[self.quad_body_id])
        gravity_z = -float(self.model.opt.gravity[2])
        # 회전 스프링과 똑같은 문제가 위치 스프링에도 있었다 - 질량
        # 기준 "순수" 임계댐핑(mult=1.0)만으로는 실제로 부족해서,
        # ros2 bag으로 world->base_link를 직접 녹화해 분석해보니
        # 공에 맞은 뒤(약 1.5~2초 지점) 수평 위치가 끝까지(녹화 13초
        # 내내) 초속 18m/s급으로 계속 진동하며 전혀 안 가라앉는 게
        # 확인됐다(회전/높이는 멀쩡했다 - 수평 위치만의 문제). 회전
        # 때와 동일하게 과댐핑으로 처방한다.
        self.humanoid_kd_pos = POS_DAMPING_MULTIPLIER * 2.0 * (SPRING_KP_POS * self.humanoid_mass) ** 0.5
        self.quad_kd_pos = POS_DAMPING_MULTIPLIER * 2.0 * (SPRING_KP_POS * self.quad_mass) ** 0.5
        self.gravity_z = gravity_z

        # 회전 스프링의 임계댐핑도 질량이 아니라 관성(moment of
        # inertia)으로 계산해야 한다. body_inertia는 (x,y,z) 축별
        # 대각 관성 3개짜리 벡터인데, 대표값으로 평균을 쓴다(상자가
        # 축마다 관성이 다르긴 하지만, 간단한 근사로 충분하다).
        #
        # 실제로 돌려보니 이 "평균 관성 기준 임계댐핑"이 충분하지
        # 않았다 - 공에 맞은 뒤 흔들림이 깨끗이 안 죽고 계속
        # 떨렸다(underdamped). 상자가 축마다 관성이 달라서(실제
        # 축별 관성과 평균의 차이, + 회전축이 계속 바뀌는 자이로
        # 효과) 단순 평균 공식이 과소추정하는 것으로 보여서,
        # HUMANOID_ROT_DAMPING_MULTIPLIER/QUAD_ROT_DAMPING_MULTIPLIER로
        # 일부러 과댐핑(critically damped보다 세게) 시켰다 - 복원이
        # 조금 느려지는 대신 진동 없이 확실히 가라앉는 쪽을 택했다.
        humanoid_inertia = float(np.mean(self.model.body_inertia[self.humanoid_body_id]))
        quad_inertia = float(np.mean(self.model.body_inertia[self.quad_body_id]))
        self.humanoid_kd_rot = (HUMANOID_ROT_DAMPING_MULTIPLIER * 2.0
                                * (HUMANOID_KP_ROT * humanoid_inertia) ** 0.5)
        self.quad_kd_rot = (QUAD_ROT_DAMPING_MULTIPLIER * 2.0
                            * (QUAD_KP_ROT * quad_inertia) ** 0.5)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, "physics_markers", 10)

        self.humanoid_pose_known = False
        self.quad_pose_known = False
        self.humanoid_prev_target_pos = None
        self.quad_prev_target_pos = None
        self.humanoid_prev_target_time = 0.0
        self.quad_prev_target_time = 0.0
        self.humanoid_prev_target_vel = np.zeros(3)
        self.quad_prev_target_vel = np.zeros(3)
        self.humanoid_prev_target_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.quad_prev_target_quat = np.array([1.0, 0.0, 0.0, 0.0])
        # "리밋 사이클에 갇힘" 감지용 - ros2 bag으로 확인해보니, 공에
        # 한 번 맞은 뒤 수평 속도가 MAX_LIN_VEL 근처에 계속 붙박인 채
        # (댐핑을 올려도) 전혀 안 가라앉는 경우가 있었다. 근본 원인을
        # 못 찾아서, 속도가 상한 근처에 STUCK_RESET_SEC 이상 계속
        # 붙어있으면 그냥 목표 자세로 강제 스냅시켜 끊어주는 안전장치.
        self.humanoid_stuck_ticks = 0
        self.quad_stuck_ticks = 0
        self.last_respawn_time = self.get_clock().now()
        self.next_respawn_is_humanoid = True  # 다음 낙하는 humanoid부터 시작 (번갈아)

        self.timer = self.create_timer(TIMER_PERIOD_SEC, self.on_timer)
        self.get_logger().info(f"MuJoCo 모델 로드: {mjcf_path}")

    def lookup_target(self, target_frame, height_offset):
        """"world" -> target_frame TF를 읽는다. 아직 없으면 None.

        "base_link_target"/"quad_base_link_target"은 발이 닿는
        높이(z=0, humanoid_state_publisher/quadruped_state_publisher
        참고)를 가리키는데, 충돌 프록시 박스는 몸통(torso) 크기로
        만들어서 몸통 높이에 있어야 한다 - 그 차이가 바로 URDF의
        base_to_body류 fixed 관절 오프셋이다(humanoid.urdf는
        LEG_LEN+BODY_H/2=1.9, quadruped.urdf는
        QUAD_LEG_LEN+QUAD_BODY_H/2=0.75). 그래서 읽은 z에 그 오프셋을
        더해줘야 박스가 몸통 자리에 온다 - 안 그러면 발밑(땅)에
        박스가 생겨버린다(처음에 겪은 버그).

        (pos, quat, stamp) 중 stamp(그 TF 메시지 자체의 타임스탬프,
        float 초)를 같이 돌려준다 - update_proxy()가 target_vel을
        "우리 틱 주기(TIMER_PERIOD_SEC)"가 아니라 "그 TF가 실제로
        얼마나 오래된 값인지"로 나눠야 한다. humanoid/quadruped_state_
        publisher는 50Hz로 발행하는데 이 노드는 100Hz로 읽어서,
        고정된 TIMER_PERIOD_SEC으로 나누면 한 틱은 "안 변함"(속도 0),
        다음 틱은 그 두 배로 튀는 에일리어싱이 생겨 - 공과 무관하게
        "애초부터 계속 떨리는" 원인이었다."""
        try:
            tf = self.tf_buffer.lookup_transform("world", target_frame, Time())
        except Exception:
            return None
        t = tf.transform.translation
        pos = np.array([t.x, t.y, t.z + height_offset])
        quat = ros_quat_to_mujoco(tf.transform.rotation)
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        return pos, quat, stamp

    def snap_body(self, qpos_adr, qvel_adr, pos, quat):
        """프록시를 목표 자세로 즉시 "스냅"시키고 속도를 0으로 만든다 -
        처음 TF를 받았을 때, 장면 밖(100,100,2)에서 중력으로 떨어지고
        있던 상태를 그대로 스프링으로 서서히 당기면 부자연스러워서,
        첫 순간만 하드 리셋한다."""
        self.data.qpos[qpos_adr:qpos_adr + 3] = pos
        self.data.qpos[qpos_adr + 3:qpos_adr + 7] = quat
        self.data.qvel[qvel_adr:qvel_adr + 6] = 0.0

    def apply_spring(self, body_id, qvel_adr, mass, kd_pos, target_pos, target_vel):
        """body_id를 target_pos 목표로 당기는 위치 스프링-댐퍼 힘을
        data.xfrc_applied에 쌓는다(이번 틱 동안 유지됨 - MuJoCo는
        xfrc_applied를 자동으로 0으로 안 돌려놔서, 매 틱 새로
        덮어써야 한다). 중력을 상쇄하는 힘을 같이 더해서, "목표보다
        약간 아래로 처지는" 정적 오차가 안 생기게 한다(순수 스프링
        만 쓰면 평형점이 중력 때문에 목표보다 낮아진다). 회전
        토크는 apply_rotation_spring()이 따로 건다."""
        cur_pos = self.data.xpos[body_id]
        cur_vel = self.data.qvel[qvel_adr:qvel_adr + 3]

        pos_err = target_pos - cur_pos
        vel_err = target_vel - cur_vel
        force = SPRING_KP_POS * pos_err + kd_pos * vel_err
        force[2] += mass * self.gravity_z  # 중력 상쇄 (feedforward)

        # apply_rotation_spring()의 ROT_QUAD_DRAG와 같은 이유 - 빠르게
        # 움직일 때만 속도 제곱에 비례해서 세게 제동을 걸어, 공에 맞은
        # 뒤 수평 위치가 안 가라앉고 계속 진동하는 걸 막는다.
        vel_norm = np.linalg.norm(cur_vel)
        force += -POS_QUAD_DRAG * vel_norm * cur_vel

        self.data.xfrc_applied[body_id, 0:3] = clamp_vec(force, MAX_FORCE)

    def clamp_body_velocity(self, qvel_adr):
        """물리 스텝 이후 선속도/각속도를 MAX_LIN_VEL/MAX_ANG_VEL로
        직접 누른다 - 접촉 솔버가 만든 속도까지 포함해서 눌러야
        터널링(깊은 침투 -> 과도한 충돌 임펄스) 폭주를 막을 수
        있다(MAX_FORCE/MAX_TORQUE로 우리가 거는 힘만 제한해서는
        안 잡혔다)."""
        lin = self.data.qvel[qvel_adr:qvel_adr + 3]
        ang = self.data.qvel[qvel_adr + 3:qvel_adr + 6]
        lin_norm = np.linalg.norm(lin)
        if lin_norm > MAX_LIN_VEL:
            self.data.qvel[qvel_adr:qvel_adr + 3] = lin * (MAX_LIN_VEL / lin_norm)
        ang_norm = np.linalg.norm(ang)
        if ang_norm > MAX_ANG_VEL:
            self.data.qvel[qvel_adr + 3:qvel_adr + 6] = ang * (MAX_ANG_VEL / ang_norm)

    def check_stuck_and_recover(self, body_id, qpos_adr, qvel_adr, stuck_ticks,
                                 target_pos, target_quat):
        """속도가 MAX_LIN_VEL/MAX_ANG_VEL 근처에 STUCK_RESET_SEC 이상
        계속 붙어있으면(리밋 사이클에 갇힌 것으로 판단) 그냥 목표
        자세로 강제 스냅시킨다. 새 stuck_ticks 값을 돌려준다."""
        lin_norm = np.linalg.norm(self.data.qvel[qvel_adr:qvel_adr + 3])
        ang_norm = np.linalg.norm(self.data.qvel[qvel_adr + 3:qvel_adr + 6])
        saturated = (lin_norm > STUCK_VEL_FRAC * MAX_LIN_VEL
                     or ang_norm > STUCK_VEL_FRAC * MAX_ANG_VEL)
        if saturated:
            stuck_ticks += 1
        else:
            stuck_ticks = 0

        reset_ticks = int(round(STUCK_RESET_SEC / TIMER_PERIOD_SEC))
        if stuck_ticks >= reset_ticks:
            self.snap_body(qpos_adr, qvel_adr, target_pos, target_quat)
            stuck_ticks = 0
        return stuck_ticks

    def apply_rotation_spring(self, body_id, qvel_adr, kp_rot, kd_rot, target_quat):
        """body_id를 target_quat(목표 방향) 쪽으로 되돌리는 회전
        스프링-댐퍼 토크를 data.xfrc_applied에 쌓는다. 쿼터니언
        오차의 벡터부(허수부)가 작은 오차에서는 회전축*sin(각도/2)에
        가까우므로, 표준 PD 자세 제어 근사로 그 값에 비례하는 복원
        토크(kp_rot)를 쓰고, 현재 각속도에 비례하는 감쇠 토크(kd_rot,
        관성 기준 임계댐핑)를 같이 더한다. humanoid/quad는 kp_rot/
        kd_rot이 서로 다르다(걷기 경로가 얼마나 급한지가 달라서)."""
        cur_quat = self.data.xquat[body_id]
        cur_angvel = self.data.qvel[qvel_adr + 3:qvel_adr + 6]

        q_err = quat_mul(target_quat, quat_conj(cur_quat))
        if q_err[0] < 0.0:
            q_err = -q_err  # 쿼터니언은 q와 -q가 같은 회전이라, 더 짧은 쪽으로

        # 공에 맞아 각속도가 커지면, 회전축이 계속 바뀌면서 복원 토크가
        # 자이로스코프처럼 계속 "옆으로" 밀려서 - kd_rot(선형 댐핑)만으로는
        # 잘 안 죽고 MAX_ANG_VEL 상한 근처에서 한동안 "팽이처럼" 계속
        # 돌았다. 속도의 제곱에 비례하는 추가 감쇠(공기저항과 비슷한
        # 느낌)를 더해서, 빠르게 돌 때만 훨씬 세게 제동이 걸리게 한다 -
        # 평소 걷는 중의 작은 흔들림(각속도 작음)에는 거의 영향이 없다.
        angvel_norm = np.linalg.norm(cur_angvel)
        quad_drag = -ROT_QUAD_DRAG * angvel_norm * cur_angvel
        torque = kp_rot * 2.0 * q_err[1:4] - kd_rot * cur_angvel + quad_drag

        self.data.xfrc_applied[body_id, 3:6] = clamp_vec(torque, MAX_TORQUE)

    def update_proxy(self, name, body_id, qpos_adr, qvel_adr, mass, kd_pos, kp_rot, kd_rot,
                      height_offset, pose_known, prev_target_pos, prev_target_time, prev_target_vel,
                      prev_target_quat):
        """target TF를 읽어서 스프링을 걸거나(이미 알고 있던 경우) 처음
        보는 거면 스냅한다. (pose_known, prev_target_pos, prev_target_time,
        prev_target_vel)을 돌려준다 - 호출하는 쪽에서 self.xxx_*에
        다시 저장해야 한다(이 메서드는 상태를 안 들고 있음).

        target_vel은 TIMER_PERIOD_SEC(우리 틱 주기)이 아니라 TF
        자체의 타임스탬프 차이로 나눈다 - humanoid/quadruped_state_
        publisher가 50Hz로 발행하는데 우리는 100Hz로 읽어서, 고정
        주기로 나누면 한 틱은 "안 변함"(속도 0), 다음 틱은 그 두 배로
        튀는 에일리어싱이 생겨 공과 무관하게 계속 떨렸다. 같은
        타임스탬프를 또 읽은 틱(소스가 아직 새 값을 안 보낸 경우)은
        dt가 0이라 나눌 수 없으니, 그냥 이전 target_vel을 그대로
        재사용한다(이번 틱에 새 정보가 없다는 뜻이므로)."""
        target = self.lookup_target(name, height_offset)
        if target is None:
            return pose_known, prev_target_pos, prev_target_time, prev_target_vel, prev_target_quat

        target_pos, target_quat, stamp = target

        if not pose_known:
            self.snap_body(qpos_adr, qvel_adr, target_pos, target_quat)
            return True, target_pos, stamp, np.zeros(3), target_quat

        dt = stamp - prev_target_time
        if dt > 1e-6:
            target_vel = (target_pos - prev_target_pos) / dt
        else:
            target_vel = prev_target_vel

        self.apply_spring(body_id, qvel_adr, mass, kd_pos, target_pos, target_vel)
        self.apply_rotation_spring(body_id, qvel_adr, kp_rot, kd_rot, target_quat)
        return True, target_pos, stamp, target_vel, target_quat

    def respawn_ball(self):
        """둘 중 한쪽 몸통 바로 위에서 공을 다시 떨어뜨린다 - 매번
        실제로 부딪히는 모습을 보여주려는 의도. on_timer()가
        RESPAWN_INTERVAL_SEC마다 주기적으로 부르거나(주된 트리거),
        공이 FALL_RESET_Z보다 낮게 떨어진 예외 상황에서 부른다.

        처음엔 매번 무작위로 둘 중 하나를 골랐는데(random.choice),
        50:50이라 운이 나쁘면 한쪽이 여러 번 연속으로 안 맞아서
        "반응이 없어졌다"는 느낌을 줄 수 있었다 - 번갈아(humanoid,
        quad, humanoid, ...) 가게 바꿔서 둘 다 꾸준히 맞는 걸 보여준다."""
        humanoid_ok = self.humanoid_pose_known
        quad_ok = self.quad_pose_known

        if humanoid_ok and quad_ok:
            use_humanoid = self.next_respawn_is_humanoid
            self.next_respawn_is_humanoid = not self.next_respawn_is_humanoid
        else:
            use_humanoid = humanoid_ok

        if humanoid_ok and (use_humanoid or not quad_ok):
            target = self.data.xpos[self.humanoid_body_id]
        elif quad_ok:
            target = self.data.xpos[self.quad_body_id]
        else:
            target = None

        if target is not None:
            x = target[0] + random.uniform(-RESPAWN_JITTER, RESPAWN_JITTER)
            y = target[1] + random.uniform(-RESPAWN_JITTER, RESPAWN_JITTER)
            z = target[2] + RESPAWN_HEIGHT_OFFSET
        else:
            # 아직 TF를 하나도 못 받았으면(다른 노드들이 아직 안 떴으면)
            # 원점 위에서 그냥 떨어뜨린다.
            x, y, z = 0.0, 0.0, 3.0

        adr = self.ball_qpos_adr
        self.data.qpos[adr:adr + 3] = [x, y, z]
        self.data.qpos[adr + 3:adr + 7] = [1.0, 0.0, 0.0, 0.0]  # 단위 쿼터니언
        vadr = self.ball_qvel_adr
        self.data.qvel[vadr:vadr + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def on_timer(self):
        (self.humanoid_pose_known, self.humanoid_prev_target_pos,
         self.humanoid_prev_target_time, self.humanoid_prev_target_vel,
         self.humanoid_prev_target_quat) = self.update_proxy(
            "base_link_target", self.humanoid_body_id,
            self.humanoid_qpos_adr, self.humanoid_qvel_adr,
            self.humanoid_mass, self.humanoid_kd_pos, HUMANOID_KP_ROT, self.humanoid_kd_rot,
            HUMANOID_BODY_HEIGHT_OFFSET,
            self.humanoid_pose_known, self.humanoid_prev_target_pos,
            self.humanoid_prev_target_time, self.humanoid_prev_target_vel,
            self.humanoid_prev_target_quat,
        )
        (self.quad_pose_known, self.quad_prev_target_pos,
         self.quad_prev_target_time, self.quad_prev_target_vel,
         self.quad_prev_target_quat) = self.update_proxy(
            "quad_base_link_target", self.quad_body_id,
            self.quad_qpos_adr, self.quad_qvel_adr,
            self.quad_mass, self.quad_kd_pos, QUAD_KP_ROT, self.quad_kd_rot,
            QUAD_BODY_HEIGHT_OFFSET,
            self.quad_pose_known, self.quad_prev_target_pos,
            self.quad_prev_target_time, self.quad_prev_target_vel,
            self.quad_prev_target_quat,
        )

        # 토크/힘을 막아도(MAX_FORCE/MAX_TORQUE), 그 결과로 생긴 빠른
        # 회전이 접촉(contact) 솔버에서 "터널링"(한 스텝 사이에 너무
        # 깊이 파고들어 생기는 비현실적으로 큰 충돌 임펄스)을 일으켜
        # 각속도가 수백만 rad/s까지 치솟는 걸 실제로 확인했다 - 이건
        # 우리가 거는 힘이 아니라 MuJoCo 자체의 접촉 반응이라
        # xfrc_applied 쪽 제한으로는 못 막는다. substep "안에서" 한
        # 번이라도 극단적인 속도가 나오면 다음 substep에서 더 깊이
        # 파고들어 되돌릴 수 없게 되므로, 틱이 끝난 뒤가 아니라
        # 매 substep마다 바로 눌러준다.
        for _ in range(PHYSICS_SUBSTEPS):
            mujoco.mj_step(self.model, self.data)
            self.clamp_body_velocity(self.humanoid_qvel_adr)
            self.clamp_body_velocity(self.quad_qvel_adr)

        if self.humanoid_pose_known:
            self.humanoid_stuck_ticks = self.check_stuck_and_recover(
                self.humanoid_body_id, self.humanoid_qpos_adr, self.humanoid_qvel_adr,
                self.humanoid_stuck_ticks, self.humanoid_prev_target_pos,
                self.humanoid_prev_target_quat,
            )
        if self.quad_pose_known:
            self.quad_stuck_ticks = self.check_stuck_and_recover(
                self.quad_body_id, self.quad_qpos_adr, self.quad_qvel_adr,
                self.quad_stuck_ticks, self.quad_prev_target_pos,
                self.quad_prev_target_quat,
            )

        now = self.get_clock().now()
        elapsed_since_respawn = (now - self.last_respawn_time).nanoseconds / 1e9

        ball_pos = self.data.xpos[self.ball_body_id]
        if ball_pos[2] < FALL_RESET_Z or elapsed_since_respawn > RESPAWN_INTERVAL_SEC:
            self.respawn_ball()
            self.last_respawn_time = now
            ball_pos = self.data.xpos[self.ball_body_id]

        ball_quat = self.data.xquat[self.ball_body_id]  # mujoco: w,x,y,z

        self.broadcast_tf(now, "ball", ball_pos, ball_quat)
        if self.humanoid_pose_known:
            base_pos, base_quat = self.body_pose_to_root(
                self.humanoid_body_id, HUMANOID_BODY_HEIGHT_OFFSET)
            self.broadcast_tf(now, "base_link", base_pos, base_quat)
        if self.quad_pose_known:
            base_pos, base_quat = self.body_pose_to_root(
                self.quad_body_id, QUAD_BODY_HEIGHT_OFFSET)
            self.broadcast_tf(now, "quad_base_link", base_pos, base_quat)

        self.publish_markers(now, ball_pos, ball_quat)

    def body_pose_to_root(self, body_id, height_offset):
        """물리 시뮬레이션은 몸통(torso) 위치를 직접 다루지만(박스
        프록시가 몸통 크기), robot_state_publisher에 발행해야 하는
        "base_link"는 그 몸통에서 base_to_body류 fixed 관절만큼
        떨어진 발 높이(루트) 프레임이다. fixed 관절은 회전 없이
        평행이동만 하므로(rpy=0), base_link의 방향은 몸통과 같고
        위치만 몸통의 "로컬 -Z축 방향"으로 height_offset만큼 뺀
        지점이다 - 몸통이 기울어 있으면 그 방향도 같이 기울여야
        해서 quat_rotate_vec()으로 오프셋 벡터를 회전시킨다."""
        body_pos = self.data.xpos[body_id]
        body_quat = self.data.xquat[body_id]
        offset_world = quat_rotate_vec(body_quat, np.array([0.0, 0.0, height_offset]))
        return body_pos - offset_world, body_quat

    def broadcast_tf(self, stamp, child_frame, pos, quat):
        tf = TransformStamped()
        tf.header.stamp = stamp.to_msg()
        tf.header.frame_id = "world"
        tf.child_frame_id = child_frame
        tf.transform.translation.x = float(pos[0])
        tf.transform.translation.y = float(pos[1])
        tf.transform.translation.z = float(pos[2])
        tf.transform.rotation.w = float(quat[0])
        tf.transform.rotation.x = float(quat[1])
        tf.transform.rotation.y = float(quat[2])
        tf.transform.rotation.z = float(quat[3])
        self.tf_broadcaster.sendTransform(tf)

    def publish_markers(self, stamp, ball_pos, ball_quat):
        arr = MarkerArray()

        ball_marker = Marker()
        ball_marker.header.frame_id = "world"
        ball_marker.header.stamp = stamp.to_msg()
        ball_marker.ns = "physics"
        ball_marker.id = 0
        ball_marker.type = Marker.SPHERE
        ball_marker.action = Marker.ADD
        ball_marker.pose.position.x = float(ball_pos[0])
        ball_marker.pose.position.y = float(ball_pos[1])
        ball_marker.pose.position.z = float(ball_pos[2])
        ball_marker.pose.orientation.w = float(ball_quat[0])
        ball_marker.pose.orientation.x = float(ball_quat[1])
        ball_marker.pose.orientation.y = float(ball_quat[2])
        ball_marker.pose.orientation.z = float(ball_quat[3])
        ball_marker.scale.x = ball_marker.scale.y = ball_marker.scale.z = 0.30  # 지름 (반지름 0.15*2)
        ball_marker.color.r = 0.85
        ball_marker.color.g = 0.25
        ball_marker.color.b = 0.25
        ball_marker.color.a = 1.0
        arr.markers.append(ball_marker)

        proxy_specs = [
            (1, self.humanoid_body_id, (0.5, 0.8, 1.4), (0.55, 0.55, 0.65), self.humanoid_pose_known),
            (2, self.quad_body_id, (1.2, 0.5, 0.4), (0.50, 0.55, 0.60), self.quad_pose_known),
        ]
        for marker_id, body_id, full_size, color, known in proxy_specs:
            if not known:
                continue
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = stamp.to_msg()
            m.ns = "physics"
            m.id = marker_id
            m.type = Marker.CUBE
            m.action = Marker.ADD
            pos = self.data.xpos[body_id]
            quat = self.data.xquat[body_id]
            m.pose.position.x = float(pos[0])
            m.pose.position.y = float(pos[1])
            m.pose.position.z = float(pos[2])
            m.pose.orientation.w = float(quat[0])
            m.pose.orientation.x = float(quat[1])
            m.pose.orientation.y = float(quat[2])
            m.pose.orientation.z = float(quat[3])
            m.scale.x, m.scale.y, m.scale.z = full_size
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 0.35  # 반투명 - 실제 로봇 메쉬를 가리지 않게
            arr.markers.append(m)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoBallSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
