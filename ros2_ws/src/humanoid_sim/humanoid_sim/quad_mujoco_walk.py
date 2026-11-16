#!/usr/bin/env python3
"""quad_mujoco_walk
------------------
4족 로봇을 "실제 관절 구동 물리 시뮬레이션"으로 평지에서 걷게 하는
노드. mujoco_ball_sim.py(공과 충돌하는 데모)와는 접근이 다르다:

- mujoco_ball_sim.py는 humanoid_state_publisher/quadruped_state_publisher
  가 발행하는 "목표 자세" TF를 향해 가상의 스프링-댐퍼 힘을
  xfrc_applied로 직접 미는 방식이었다(몸통 크기 박스 프록시 하나,
  팔다리/머리는 물리적으로 없음 - 공이 그대로 통과함).
- 이 노드는 그 반대다: mujoco/quad_walk.xml에 몸통+머리+다리 4개가
  각각 실제 충돌 가능한 지오메트리로 정의돼 있고, 다리 관절은
  MuJoCo 네이티브 <position> 액추에이터(PD 위치 서보)로 구동된다.
  이 노드가 하는 일은 quadruped_state_publisher.py와 똑같은 트롯
  걸음걸이 각도(30*sin(2t), 대각선 다리 쌍 반대 위상)를 그 액추에이터
  "목표각"으로 넣어주는 것뿐 - 몸통이 실제로 앞으로 나가는지는
  순전히 물리(다리가 바닥을 밀어내는 반작용 + 마찰)로 결정된다.
  외부에서 몸통 위치를 끌어당기지 않는다.

독립 MuJoCo 스크립트(ROS 없이, mj_step만 반복)로 먼저 검증했다.
처음엔 quadruped_state_publisher.py의 걸음걸이 시간 공식을 그대로
재사용했는데(한 걸음 주기 약 15.7초), MuJoCo 자체 뷰어(mujoco.viewer,
이 역시 ROS/DDS와 무관해서 안전하게 띄워볼 수 있었다)로 직접 보니
사용자가 "걸어가려고 애쓰는데 앞으로 잘 안 나간다"고 확인 - 그래서
다리가 직접 추진력을 내는 이 구조에 맞는 완전히 새로운 걸음
주파수를 0.5~3.0Hz 범위로 쓸어보며 찾았다: 1.1Hz/진폭 30도가
가장 좋았다 - 10초 동안 쓰러지지 않고(최대 기울기 2도대) 약 0.88m
순전진. 그래도 사용자가 "미끄러워서 안 나가는 것 같다"고 다시
확인 - 바닥/다리 마찰 계수를 0.5~5.0까지 쓸어봤지만 그게 원인이
아니었다(오히려 기본값 1.0이 최적, 더 끈적하게 하면 발이 바닥에
박혀서 더 불안정해지거나 넘어짐). 진짜 원인은 **무릎이 없어서
스윙 중에 발을 제대로 못 드는 것**이었다 - 그래서 다리마다 무릎
관절을 추가했다(허벅지+종아리 2 세그먼트, 각 0.275m = 기존
QUAD_LEG_LEN(0.55)을 반으로 나눈 것 - 다리가 펴졌을 때 총 길이가
그대로라 서있는 높이 계산은 안 바뀐다). 무릎을 "엉덩이가 스윙
(발을 앞으로 가져가는 구간)일 때 굽히고, 스탠스(바닥을 미는
구간)일 때 펴지는" 형태의 신호(`max(0,-cos(phase))`)로 굽히는
타이밍/진폭도 같은 방식(ROS 없이 mj_step 반복)으로 쓸어서 찾았다
(무릎 진폭 5~60도 쓸어봄) - **15도**에서 10초 동안 **5.26m** 순전진
(무릎 없을 때의 약 6배!)하면서도 기울기는 7도 이내로 안정적이었다.
16도부터 급격히 불안정해지고(19도대 기울기) 17도부터는 넘어져서,
15도는 "거의 한계까지 밀어붙이되 안전 마진은 있는" 값이다.

**그런데 이 10초짜리 검증이 함정이었다.** 사용자가 MuJoCo 뷰어로
오래(1분 이상) 지켜보더니 "가장자리에 가서 뒤집어지네?"라고
확인 - 180초까지 쓸어보니 **1.1Hz는 18~20초쯤 롤(기울기)이 서서히
커지다가 반드시 뒤집혀서 옆으로 누운 채 멈췄다**(한 번의 사고가
아니라 누적되는 드리프트였다 - 그래서 10초 안에는 안 보였다).
0.8~0.95Hz는 전부 180초까지 안정적이었는데, 그중 **0.85Hz**가
옆으로 쏠리는 양(lateral drift)이 가장 작아서(180초에 1.8m,
1.0Hz는 60초에 8.5m나 쏠림, 0.95Hz는 120초에 17.8m) 가장 똑바로
걷는다 - 그래서 최종적으로 1.1Hz 대신 이 값을 쓴다(아래
GAIT_FREQ_HZ). **교훈: 걸음걸이를 검증할 땐 짧은 구간만 보지 말고
충분히 길게(최소 1~2분) 돌려봐야 한다** - 리밋 사이클/누적 드리프트
같은 문제는 보통 바로 안 드러난다.

시각적 모델(RViz에 보이는 메쉬)은 기존 quadruped.urdf를 그대로
쓴다 - 이 노드는 그 URDF와 똑같은 관절 이름(quadbody_to_QuadLegFL
등)으로 /quadruped/joint_states를 발행하고, 물리 시뮬레이션이 계산한
실제 몸통 자세를 "world"->"quad_base_link" TF로 발행한다(기존
quadruped_state_publisher.py가 발행하던 자리를 대신하므로, 이
노드를 쓸 때는 quadruped_state_publisher를 띄우지 않는다 - 둘 다
같은 TF 프레임을 발행하면 충돌한다).

좌표계: quad_walk.xml은 quadruped.urdf와 똑같이 ROS 좌표계(X=앞,
Y=왼쪽, Z=위)를 그대로 쓴다 - 별도 축 변환이 필요 없다.

**2026-11-16 업데이트: /cmd_vel(geometry_msgs/Twist)로 방향 조정.**
linear.x(전진 속도)와 angular.z(회전 속도)를 받아서 걸음걸이를
실시간으로 바꾼다 - "엉덩이 보폭을 좌우 다리에 다르게 주면 도는지"를
ROS 없이 mj_step만 반복하는 스크립트로 먼저 검증했다(README의
"cmd_vel 방향 조정" 섹션에 전체 과정 기록). 핵심 발견: 이 매핑은
**선형이 아니라 꽤 예민/비선형(카오스적)**이다 - 기존 직진 보폭
32도에 회전 차이 3도만 더해도 역주행(뒤로 감)이 나올 정도로
불안정했다. 그래서 회전을 많이 줄수록 전진 보폭을 32도에서 25도
쪽으로 같이 낮추는 보정을 넣었다(아래 BASE_AMP_* 상수) - 이 조합
(32/0, 28.5/1.5, 25/3도)만 90초씩 따로 검증했다. 또한 다리 엉덩이
관절에 요(yaw)/벌림(ab-duction) 축이 없어서(피치 1축뿐) 회전
속도 자체가 매우 느리다(최대 설정으로도 초당 1도 안팎) - 더 빠른
회전을 원하면 엉덩이에 두 번째(요) 관절을 추가하는 게 다음 단계일
것이다(무릎을 추가했던 것과 같은 패턴).
"""

import math
import os

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

import mujoco

TIMER_PERIOD_SEC = 0.01   # 100 Hz
PHYSICS_SUBSTEPS = 2      # MJCF의 timestep(0.005)과 곱해서 0.01s를 채움

# 처음엔 quadruped_state_publisher.py의 걸음걸이 "시간" 공식(T_INCREMENT/
# TIMER_PERIOD_SEC 기반, 한 걸음 주기가 약 15.7초)을 그대로 재사용했는데
# - 그건 몸통을 따로 빠르게(patrol 함수로) 움직이고 다리는 "보기용"
# 으로만 느리게 흔들던 애니메이션용 숫자였다. 여기서는 다리가 직접
# 추진력을 내야 해서 그렇게 느리면 거의 안 걸어간다(실제로 사용자가
# MuJoCo 뷰어로 보고 "걸어가려고 애쓰는데 앞으로 잘 안 나간다"고
# 확인함). ROS 없이 순수 MuJoCo로 주파수를 쓸어보며(0.5~3.0Hz,
# 진폭 20~40도) 검증한 결과 1.1Hz/진폭 30도가 가장 좋았다 - 10초
# 동안 쓰러지지 않고(최대 기울기 2도대) 약 0.88m 전진(1.0Hz 대비
# 1.1Hz에서 더 멀리 감, 1.5Hz부터는 옆으로 밀리기 시작, 2Hz 이상은
# 넘어짐) - 그래서 완전히 새로운, 독립적인 걸음 주파수를 쓴다.
GAIT_FREQ_HZ = 0.85         # 걸음 주기(다리 1개 기준) - 초당 사이클 수.
                             # 원래 10초 검증에서 1.1Hz가 가장 멀리
                             # 갔었는데(5.26m), 사용자가 오래 보니
                             # "가장자리에서 뒤집어지네" - 180초까지
                             # 쓸어보니 1.1Hz는 18~20초쯤 반드시 뒤로
                             # 쓰러져서 옆으로 누운 채 멈췄다(순간적
                             # 사고가 아니라 롤이 서서히 쌓이다 넘어가는
                             # 패턴). 0.8~0.95Hz는 전부 180초까지
                             # 안정적이었는데, 그중 0.85Hz가 옆으로
                             # 쏠리는 양(lateral drift)이 가장 작아서
                             # (180초에 1.8m, 1.0Hz는 60초에 8.5m,
                             # 0.95Hz는 120초에 17.8m) 제일 똑바로
                             # 걷는다 - 그래서 1.1Hz 대신 이 값을 쓴다.
SWING_AMPLITUDE_DEG = 32.0  # 엉덩이가 앞뒤로 흔들리는 최대 각도.
                             # "너무 느리다"는 피드백을 받고, 0.85Hz
                             # 고정한 채 30~37도를 150~300초씩 쓸어서
                             # 찾았다 - 32/33도가 유독 옆쏠림이 거의
                             # 없었다(300초에 1.6m). 34~37도는 기울기는
                             # 안정적이어도 옆쏠림이 요동쳐서(어떤
                             # 값은 240초에 27m나 쏠림) 똑바로 안
                             # 걷는다 - 민감한(카오스적) 구간으로
                             # 보여서 32도로 안전하게 잡았다. 300초
                             # 동안 48m 전진(기존 30도보다 꽤 빠름),
                             # 기울기 6도 이내.
KNEE_AMPLITUDE_DEG = 15.0   # 무릎이 굽혀지는 최대 각도(0=쭉 뻗음) -
                             # 16도부터 급격히 불안정, 17도부터 넘어짐
                             # (위 독립 검증 결과 참고) - 15도가 한계
                             # 바로 아래의 안전한 값.

# ---- /cmd_vel 매핑 상수 (2026-11-16, 독립 MuJoCo 스크립트로 검증) ----
# cmd_linear_x(m/s)가 이 값 이상이면 "최고 속도"(직진 시 보폭 32도)로
# 간주 - SWING_AMPLITUDE_DEG=32도일 때 300초 동안 48.3m 간 실측값
# (48.3/300)에서 가져옴.
CMD_MAX_SPEED_MPS = 0.16
# cmd_angular_z(rad/s)가 이 값 이상이면 "최대 회전"으로 간주 - 아래
# MAX_TURN_DIFF_DEG(좌우 보폭 차이 3도)를 90초 동안 줬을 때 실측 회전
# 약 0.8도/초(75.2도/90초) 기준.
CMD_MAX_TURN_RATE_RAD_S = 0.015
# 좌우 다리 보폭 차이의 최대값(도) - 회전을 만드는 방법. 90초 검증으로
# 0~3도는 깨끗하게(회전량이 차이값에 거의 비례, 안 넘어짐) 늘었지만
# 4.5도부터 회전력을 잃고 6도에서는 넘어졌다 - 3도를 한계로 잡았다.
CMD_MAX_TURN_DIFF_DEG = 3.0
# 회전이 전혀 없을 때(직진) 쓰는 최대 보폭 - 기존 SWING_AMPLITUDE_DEG와
# 같음.
CMD_BASE_AMP_AT_ZERO_TURN_DEG = SWING_AMPLITUDE_DEG
# 최대 회전일 때 쓰는 보폭 - **32도(직진용 최대 보폭) 그대로에 회전
# 차이 3도를 더하면 역주행까지 나올 만큼 불안정했다**(90초 검증,
# fwd=-1.4m). 25도로 낮춘 상태에서 3도 차이를 주니 안정적이고
# 방향도 한쪽으로 일관됐다(90초에 75.2도 회전, 안 넘어짐) - 그래서
# 회전이 커질수록 보폭을 32->25도 쪽으로 같이 낮춘다(선형 보간).
CMD_BASE_AMP_AT_MAX_TURN_DEG = 25.0
# /cmd_vel을 이 시간(초) 이상 못 받으면 멈춘 것으로 간주 - teleop
# 연결이 끊겼는데 로봇이 계속 걷는 걸 막는 안전장치.
CMD_VEL_TIMEOUT_SEC = 1.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def compute_leg_targets(t, cmd_linear_x, cmd_angular_z):
    """현재 시뮬레이션 시간 t와 명령 속도로 다리마다 (엉덩이각, 무릎각)
    라디안을 계산한다. quad_mujoco_walk.py/view_quad_walk.py가
    공통으로 쓰는 매핑(둘 다 같은 값을 써야 동작이 똑같이 보인다)."""
    speed_frac = clamp(cmd_linear_x / CMD_MAX_SPEED_MPS, 0.0, 1.0)
    turn_frac = clamp(cmd_angular_z / CMD_MAX_TURN_RATE_RAD_S, -1.0, 1.0)
    turn_diff_deg = turn_frac * CMD_MAX_TURN_DIFF_DEG

    amp_span = CMD_BASE_AMP_AT_ZERO_TURN_DEG - CMD_BASE_AMP_AT_MAX_TURN_DEG
    base_amp_deg = (CMD_BASE_AMP_AT_ZERO_TURN_DEG - amp_span * abs(turn_frac)) * speed_frac
    knee_amp_deg = KNEE_AMPLITUDE_DEG * speed_frac

    base_phase = 2.0 * math.pi * GAIT_FREQ_HZ * t
    hip_rad = {}
    knee_rad = {}
    for leg in LEGS:
        # 오른쪽(FR,BR)은 +turn_diff, 왼쪽(FL,BL)은 -turn_diff - 오른쪽
        # 보폭이 커지면 로봇이 왼쪽(반시계, ROS에서 angular.z>0과
        # 같은 방향)으로 돈다(독립 검증으로 부호 확인함).
        side_sign = 1.0 if leg in ("FR", "BR") else -1.0
        amp_deg = base_amp_deg + side_sign * turn_diff_deg
        hip_rad[leg] = HIP_SIGN[leg] * math.radians(amp_deg) * math.sin(base_phase)

        knee_phase = base_phase + KNEE_PHASE_OFFSET[leg]
        knee_rad[leg] = max(0.0, -math.cos(knee_phase)) * math.radians(knee_amp_deg)
    return hip_rad, knee_rad

# QUAD_LEG_LEN(0.55) + QUAD_BODY_H/2(0.2) - quad_walk.xml의 몸통 시작
# 높이, quadruped.urdf의 base_to_body_z와 동일.
QUAD_BODY_HEIGHT_OFFSET = 0.75

LEGS = ["FL", "FR", "BL", "BR"]
# 대각선 쌍(FL,BR)이 한 위상, (FR,BL)이 반대 위상 - quadruped_state_
# publisher.py의 트롯 패턴과 동일. 부호는 엉덩이 스윙에 곱하는 값.
HIP_SIGN = {"FL": 1.0, "BR": 1.0, "FR": -1.0, "BL": -1.0}
# 무릎 위상 오프셋(라디안) - 무릎은 "자기 다리의 엉덩이가 스윙하는
# 동안" 굽혀야 하므로, (FR,BL)처럼 엉덩이가 반대 위상이면 무릎도
# 반대 위상(phase+pi)으로 계산해야 각 다리 입장에서 "스윙=굽힘"이
# 맞게 유지된다.
KNEE_PHASE_OFFSET = {"FL": 0.0, "BR": 0.0, "FR": math.pi, "BL": math.pi}

JOINT_NAMES = (
    [f"quadbody_to_QuadLeg{leg}" for leg in LEGS]
    + [f"QuadLeg{leg}_to_QuadShin{leg}" for leg in LEGS]
)
HIP_ACTUATOR_NAMES = {leg: f"act_{leg}" for leg in LEGS}
KNEE_ACTUATOR_NAMES = {leg: f"act_knee_{leg}" for leg in LEGS}


def default_mjcf_path():
    """설치된 패키지의 share 디렉터리에서 MJCF를 찾는다(mujoco_ball_sim.py의
    default_mjcf_path()와 같은 패턴)."""
    try:
        share_dir = get_package_share_directory("humanoid_sim")
        return os.path.join(share_dir, "mujoco", "quad_walk.xml")
    except Exception:
        return os.path.normpath(
            os.path.join(os.path.dirname(__file__), "..", "mujoco", "quad_walk.xml")
        )


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_rotate_vec(q, v):
    """쿼터니언(w,x,y,z) q로 3차원 벡터 v를 회전시킨다."""
    v_quat = np.array([0.0, v[0], v[1], v[2]])
    result = quat_mul(quat_mul(q, v_quat), quat_conj(q))
    return result[1:4]


class QuadMujocoWalk(Node):
    def __init__(self):
        super().__init__("quad_mujoco_walk")

        self.declare_parameter("mjcf_path", default_mjcf_path())
        mjcf_path = self.get_parameter("mjcf_path").get_parameter_value().string_value

        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)

        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "quad_body")
        self.joint_qpos_adr = {
            name: self.model.jnt_qposadr[
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ]
            for name in JOINT_NAMES
        }
        self.hip_actuator_id = {
            leg: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for leg, name in HIP_ACTUATOR_NAMES.items()
        }
        self.knee_actuator_id = {
            leg: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for leg, name in KNEE_ACTUATOR_NAMES.items()
        }

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)

        self.timer = self.create_timer(TIMER_PERIOD_SEC, self.on_timer)
        self.get_logger().info(f"MuJoCo 모델 로드: {mjcf_path}")

    def on_cmd_vel(self, msg):
        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def on_timer(self):
        # ---- 트롯 걸음걸이. 엉덩이/무릎 각도는 compute_leg_targets()가
        # /cmd_vel 명령(linear.x=전진 속도, angular.z=회전 속도)으로
        # 계산한다 - CMD_VEL_TIMEOUT_SEC 넘게 명령이 없으면 정지(0,0)로
        # 간주해서, teleop 연결이 끊겨도 로봇이 계속 걷지 않게 한다. ----
        age_sec = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age_sec > CMD_VEL_TIMEOUT_SEC:
            linear_x, angular_z = 0.0, 0.0
        else:
            linear_x, angular_z = self.cmd_linear_x, self.cmd_angular_z

        hip_rad, knee_rad = compute_leg_targets(self.data.time, linear_x, angular_z)
        for leg in LEGS:
            self.data.ctrl[self.hip_actuator_id[leg]] = hip_rad[leg]
            self.data.ctrl[self.knee_actuator_id[leg]] = knee_rad[leg]

        for _ in range(PHYSICS_SUBSTEPS):
            mujoco.mj_step(self.model, self.data)

        now = self.get_clock().now()

        # ---- 관절각(실제 물리 결과) 발행 ----
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = list(JOINT_NAMES)
        js.position = [float(self.data.qpos[self.joint_qpos_adr[n]]) for n in JOINT_NAMES]
        self.joint_pub.publish(js)

        # ---- 몸통(QuadBody) 실제 물리 자세를 "world"->"quad_base_link"
        #      TF로 발행. QuadBody는 base_link보다 QUAD_BODY_HEIGHT_OFFSET
        #      만큼(URDF의 base_to_quadbody fixed 관절) 위에 있으므로,
        #      그 오프셋을 몸통의 현재 방향으로 회전시켜 다시 빼서
        #      base_link 자세를 역산한다(mujoco_ball_sim.py의
        #      body_pose_to_root()와 동일한 방법) - 안 그러면
        #      robot_state_publisher가 URDF에서 그 오프셋을 또 적용해서
        #      로봇이 공중에 떠 보인다. -----
        body_pos = self.data.xpos[self.body_id]
        body_quat = self.data.xquat[self.body_id]
        offset_world = quat_rotate_vec(body_quat, np.array([0.0, 0.0, QUAD_BODY_HEIGHT_OFFSET]))
        base_pos = body_pos - offset_world

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "world"
        tf.child_frame_id = "quad_base_link"
        tf.transform.translation.x = float(base_pos[0])
        tf.transform.translation.y = float(base_pos[1])
        tf.transform.translation.z = float(base_pos[2])
        tf.transform.rotation.w = float(body_quat[0])
        tf.transform.rotation.x = float(body_quat[1])
        tf.transform.rotation.y = float(body_quat[2])
        tf.transform.rotation.z = float(body_quat[3])
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = QuadMujocoWalk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
