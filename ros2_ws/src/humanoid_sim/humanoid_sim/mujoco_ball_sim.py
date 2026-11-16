#!/usr/bin/env python3
"""mujoco_ball_sim
------------------
MuJoCo로 공을 하나 떨어뜨려서, 지금 걷고 있는 humanoid와 순찰 중인
4족 로봇의 몸통과 실제로 충돌(접촉)하게 만들고, **맞으면 로봇도
반응하게** 만드는 노드.

humanoid_proxy / quad_proxy는 질량이 있는 진짜 물리 바디다(자유
6DOF). 처음엔 순수 "mocap" 바디(물리 영향을 안 받고 자세만 외부에서
주입받는)로 만들었는데, 그러면 공이 부딪혀도 로봇이 전혀 반응하지
않아서 자유 바디로 바꿨다 - 자세를 어떻게 "따라가게" 만드는지가
이 노드의 핵심이다.

**2026-11-16 재설계 (방식 2): MuJoCo 네이티브 `mocap`+`weld` 제약.**
처음엔(방식 1) 매 틱 "world"->"base_link_target"/"quad_base_link_
target" TF(걷기/순찰 애니메이션의 "원하는" 목표 자세)를 향해 직접
만든 스프링-댐퍼 힘/토크를 data.xfrc_applied로 걸었다. 그런데 공에
맞은 뒤 수평 위치가 리밋 사이클에 빠져서 16~20m/s대로 영원히 안
가라앉는 버그가 있었다(ros2 bag 녹화 분석으로 발견, 댐핑을 아무리
올려도 안 죽었음 - 자세한 경위는 아래 "아직 안 풀린 문제"에 그대로
남겨뒀다). [[feedback_prefer_library_primitives]] 교훈대로 "직접
만들기 전에 라이브러리 기본 기능부터 찾아보자"는 원칙을 적용해서,
`ball_collision.xml`에 `mocap="true"` 목표 바디(humanoid_target/
quad_target)와 `<equality><weld>` 제약을 추가했다 - 이제 이 노드는
그 mocap 바디의 `data.mocap_pos`/`data.mocap_quat`에 TF로 받은
목표 자세를 매 틱 그대로 써넣기만 하면 되고, 스프링 힘 계산은
MuJoCo의 제약 솔버가 대신 한다(접촉과 같은 솔버 스텝에서 함께
풀려서 훨씬 안정적 - 독립 MuJoCo 스크립트로 solref 시간상수
0.02~2.0을 쓸어본 결과 단 한 번도 리밋 사이클/발산이 재현되지
않았다). apply_spring()/apply_rotation_spring()/check_stuck_and_
recover() 등 방식 1의 수동 스프링 코드는 전부 제거했다.

이 노드가 그 물리 시뮬레이션 결과(흔들림 포함)를 실제
"world"->"base_link" / "world"->"quad_base_link" TF로 발행한다 -
robot_state_publisher가 이걸로 전체 로봇(Body/Head/팔다리 등)의
TF를 계산하므로, RViz2에 보이는 로봇 메쉬 자체가 흔들린다. 그래서
humanoid_state_publisher/quadruped_state_publisher는
publish_base_link 파라미터를 False로 받아서(physics_demo.launch.py가
그렇게 띄움) "base_link"/"quad_base_link"를 직접 발행하지 않는다 -
이 노드가 그 프레임의 유일한 발행자가 되게 해서 TF 충돌을 막는다.

weld 제약의 "부드러움"(ball_collision.xml의 solref 시간상수)이
"얼마나 세게 흔들렸으면 좋겠는지"를 정하는 값이다 - 0.15로 일단
잡았는데(과하게 멀리 안 밀려나면서도 눈에 보일 만큼), 실제 데모를
보면서 더 조정해야 할 수 있다.

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

TIMER_PERIOD_SEC = 0.01     # 100 Hz - mocap 목표 갱신 주기이기도 하다
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

# 로봇 프록시가 목표 자세를 따라가는 "부드러움"은 이제 전부
# ball_collision.xml의 weld 제약(solref)이 결정한다 - 방식 1(수동
# xfrc_applied 스프링, kp/댐핑 상수 뭉치)은 리밋 사이클 버그로 전부
# 제거했다. 자세한 경위는 README의 "아직 안 풀린 문제" +
# "네이티브 제약으로 재설계" 섹션, 과거 버전은 git 히스토리 참고.
#
# 그래도 남겨두는 것: 접촉(contact) 솔버가 "터널링"(한 스텝 사이에
# 너무 깊이 파고들어 생기는 비현실적으로 큰 충돌 임펄스)을 일으켜
# 각속도가 수백만 rad/s까지 치솟는 걸 실제로 확인했다 - 이건 weld
# 제약과 무관하게 MuJoCo 접촉 자체의 특성이라, 속도를 직접 눌러주는
# 안전망은 계속 필요하다.
MAX_LIN_VEL = 15.0      # m/s - 걷기/순찰 속도보다 훨씬 넉넉한 상한
MAX_ANG_VEL = 6.0       # rad/s - "팽이처럼 돈다"는 피드백을 받고 15->6으로 낮춤


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
        # mocap 목표 바디 - weld 제약이 humanoid_proxy/quad_proxy를
        # 여기 붙여준다(ball_collision.xml 참고). body_mocapid는
        # mocap_pos/mocap_quat 배열 안에서의 인덱스(일반 body_id와는
        # 다른 별도 번호 체계)다.
        humanoid_target_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "humanoid_target")
        quad_target_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "quad_target")
        self.humanoid_mocap_idx = self.model.body_mocapid[humanoid_target_body_id]
        self.quad_mocap_idx = self.model.body_mocapid[quad_target_body_id]

        self.ball_qpos_adr, self.ball_qvel_adr = free_joint_adr(self.model, self.ball_body_id)
        self.humanoid_qpos_adr, self.humanoid_qvel_adr = free_joint_adr(self.model, self.humanoid_body_id)
        self.quad_qpos_adr, self.quad_qvel_adr = free_joint_adr(self.model, self.quad_body_id)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, "physics_markers", 10)

        self.humanoid_pose_known = False
        self.quad_pose_known = False
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
        박스가 생겨버린다(처음에 겪은 버그)."""
        try:
            tf = self.tf_buffer.lookup_transform("world", target_frame, Time())
        except Exception:
            return None
        t = tf.transform.translation
        pos = np.array([t.x, t.y, t.z + height_offset])
        quat = ros_quat_to_mujoco(tf.transform.rotation)
        return pos, quat

    def snap_body(self, qpos_adr, qvel_adr, pos, quat):
        """프록시를 목표 자세로 즉시 "스냅"시키고 속도를 0으로 만든다 -
        처음 TF를 받았을 때, 장면 밖(100,100,2)에서 중력으로 떨어지고
        있던 상태를 그대로 weld로 서서히 당기면 부자연스러워서, 첫
        순간만 하드 리셋한다."""
        self.data.qpos[qpos_adr:qpos_adr + 3] = pos
        self.data.qpos[qpos_adr + 3:qpos_adr + 7] = quat
        self.data.qvel[qvel_adr:qvel_adr + 6] = 0.0

    def clamp_body_velocity(self, qvel_adr):
        """물리 스텝 이후 선속도/각속도를 MAX_LIN_VEL/MAX_ANG_VEL로
        직접 누른다 - 접촉 솔버가 만든 속도까지 포함해서 눌러야
        터널링(깊은 침투 -> 과도한 충돌 임펄스) 폭주를 막을 수
        있다."""
        lin = self.data.qvel[qvel_adr:qvel_adr + 3]
        ang = self.data.qvel[qvel_adr + 3:qvel_adr + 6]
        lin_norm = np.linalg.norm(lin)
        if lin_norm > MAX_LIN_VEL:
            self.data.qvel[qvel_adr:qvel_adr + 3] = lin * (MAX_LIN_VEL / lin_norm)
        ang_norm = np.linalg.norm(ang)
        if ang_norm > MAX_ANG_VEL:
            self.data.qvel[qvel_adr + 3:qvel_adr + 6] = ang * (MAX_ANG_VEL / ang_norm)

    def update_proxy(self, name, qpos_adr, qvel_adr, mocap_idx, height_offset, pose_known):
        """target TF를 읽어서 mocap 목표의 pos/quat를 갱신한다(이미
        알고 있던 경우) - 처음 보는 거면 프록시 자체를 그 자리로
        스냅한다. 방식 1(xfrc_applied 스프링)과 달리 목표 속도를 따로
        계산할 필요가 없다 - mocap은 속도가 없는 순수 kinematic
        위치/방향이고, weld 제약 자체의 solref가 추종 속도를 결정
        한다(ball_collision.xml 참고). 새 pose_known 값을 돌려준다."""
        target = self.lookup_target(name, height_offset)
        if target is None:
            return pose_known

        target_pos, target_quat = target

        if not pose_known:
            self.snap_body(qpos_adr, qvel_adr, target_pos, target_quat)

        self.data.mocap_pos[mocap_idx] = target_pos
        self.data.mocap_quat[mocap_idx] = target_quat
        return True

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
        self.humanoid_pose_known = self.update_proxy(
            "base_link_target", self.humanoid_qpos_adr, self.humanoid_qvel_adr,
            self.humanoid_mocap_idx, HUMANOID_BODY_HEIGHT_OFFSET, self.humanoid_pose_known,
        )
        self.quad_pose_known = self.update_proxy(
            "quad_base_link_target", self.quad_qpos_adr, self.quad_qvel_adr,
            self.quad_mocap_idx, QUAD_BODY_HEIGHT_OFFSET, self.quad_pose_known,
        )

        # weld 제약과 무관하게, 접촉(contact) 솔버가 "터널링"(한 스텝
        # 사이에 너무 깊이 파고들어 생기는 비현실적으로 큰 충돌
        # 임펄스)을 일으켜 각속도가 수백만 rad/s까지 치솟는 걸 실제로
        # 확인했다 - substep "안에서" 한 번이라도 극단적인 속도가
        # 나오면 다음 substep에서 더 깊이 파고들어 되돌릴 수 없게
        # 되므로, 틱이 끝난 뒤가 아니라 매 substep마다 바로 눌러준다.
        for _ in range(PHYSICS_SUBSTEPS):
            mujoco.mj_step(self.model, self.data)
            self.clamp_body_velocity(self.humanoid_qvel_adr)
            self.clamp_body_velocity(self.quad_qvel_adr)

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

        # MARKER_SCALE_UP: proxy_specs의 치수는 ball_collision.xml의
        # 실제 충돌 박스(=몸통 메쉬와 완전히 같은 크기)와 똑같이
        # 맞춰놨는데, 그러면 평소(안 맞았을 때)엔 반투명 박스가
        # 불투명한 로봇 메쉬 안에 정확히 겹쳐 들어가서 전혀 안
        # 보인다 - 디버그용으로 늘 식별 가능하게, 시각화 전용으로만
        # 살짝(8%) 키워서 메쉬 밖으로 테두리처럼 삐져나오게 한다
        # (물리 충돌 크기 자체는 ball_collision.xml 쪽만 쓰고 이
        # 마커 스케일과는 무관 - 안 바뀜).
        MARKER_SCALE_UP = 1.08
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
            m.scale.x, m.scale.y, m.scale.z = [s * MARKER_SCALE_UP for s in full_size]
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
