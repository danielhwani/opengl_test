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

스프링 상수(SPRING_KP_POS 등)는 질량을 기준으로 임계댐핑(critical
damping)을 계산해서 정하지만, 공/로봇 질량이나 "얼마나 세게
흔들렸으면 좋겠는지"는 결국 실제로 보면서 조절해야 하는 값이다 -
이 세션에서는 MuJoCo를 직접 돌려볼 수 없어서 수치만 합리적으로
골랐을 뿐 튜닝은 못 했다.

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
PHYSICS_SUBSTEPS = 2        # MJCF의 timestep(0.005)과 곱해서 0.01s를 채움

FALL_RESET_Z = -2.0         # 이 높이보다 낮아지면 다시 떨어뜨림 (예외 상황 대비)
RESPAWN_INTERVAL_SEC = 3.0  # 이 주기마다 항상 다시 떨어뜨림 (주된 트리거)
RESPAWN_HEIGHT_OFFSET = 2.0  # 고른 몸통 위 이만큼 높은 곳에서 다시 떨어뜨림
RESPAWN_JITTER = 0.3        # 몸통 중심에서 좌우로 흔드는 정도(m)

# 로봇 프록시를 목표 자세로 당기는 가상 스프링-댐퍼 상수. 위치
# 스프링(SPRING_KP_POS)에 대한 댐핑은 질량으로부터 임계댐핑
# (critical damping, kd = 2*sqrt(kp*m))을 계산해서 "진동 없이 가장
# 빨리 되돌아오는" 값으로 맞춘다. 회전 스프링은 질량 대신 관성을
# 써야 더 정확하지만, 간단하게 고정값을 쓰고 튜닝 대상으로 남긴다.
SPRING_KP_POS = 600.0   # N/m
SPRING_KP_ROT = 150.0   # 쿼터니언 오차에 곱하는 토크 계수
SPRING_KD_ROT = 30.0    # 각속도에 곱하는 감쇠 토크 계수


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
        self.humanoid_kd_pos = 2.0 * (SPRING_KP_POS * self.humanoid_mass) ** 0.5
        self.quad_kd_pos = 2.0 * (SPRING_KP_POS * self.quad_mass) ** 0.5
        self.gravity_z = gravity_z

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, "physics_markers", 10)

        self.humanoid_pose_known = False
        self.quad_pose_known = False
        self.humanoid_prev_target_pos = None
        self.quad_prev_target_pos = None
        self.last_respawn_time = self.get_clock().now()

        self.timer = self.create_timer(TIMER_PERIOD_SEC, self.on_timer)
        self.get_logger().info(f"MuJoCo 모델 로드: {mjcf_path}")

    def lookup_target(self, target_frame):
        """"world" -> target_frame TF를 읽는다. 아직 없으면 None."""
        try:
            tf = self.tf_buffer.lookup_transform("world", target_frame, Time())
        except Exception:
            return None
        t = tf.transform.translation
        pos = np.array([t.x, t.y, t.z])
        quat = ros_quat_to_mujoco(tf.transform.rotation)
        return pos, quat

    def snap_body(self, qpos_adr, qvel_adr, pos, quat):
        """프록시를 목표 자세로 즉시 "스냅"시키고 속도를 0으로 만든다 -
        처음 TF를 받았을 때, 장면 밖(100,100,2)에서 중력으로 떨어지고
        있던 상태를 그대로 스프링으로 서서히 당기면 부자연스러워서,
        첫 순간만 하드 리셋한다."""
        self.data.qpos[qpos_adr:qpos_adr + 3] = pos
        self.data.qpos[qpos_adr + 3:qpos_adr + 7] = quat
        self.data.qvel[qvel_adr:qvel_adr + 6] = 0.0

    def apply_spring(self, body_id, qvel_adr, mass, kd_pos, target_pos, target_vel):
        """body_id를 target_pos/현재 쿼터니언 목표로 당기는 스프링-댐퍼
        힘/토크를 data.xfrc_applied에 쌓는다(이번 틱 동안 유지됨 -
        MuJoCo는 xfrc_applied를 자동으로 0으로 안 돌려놔서, 매 틱
        새로 덮어써야 한다). 위치 스프링은 중력을 상쇄하는 힘을
        더해서, "목표보다 약간 아래로 처지는" 정적 오차가 안 생기게
        한다(순수 스프링만 쓰면 평형점이 중력 때문에 목표보다
        낮아진다)."""
        cur_pos = self.data.xpos[body_id]
        cur_vel = self.data.qvel[qvel_adr:qvel_adr + 3]
        cur_angvel = self.data.qvel[qvel_adr + 3:qvel_adr + 6]

        pos_err = target_pos - cur_pos
        force = SPRING_KP_POS * pos_err + kd_pos * (target_vel - cur_vel)
        force[2] += mass * self.gravity_z  # 중력 상쇄 (feedforward)

        # 회전 토크(목표 방향 복원)는 apply_rotation_spring()이 따로
        # 더한다 - 여기서는 각속도 댐핑만 먼저 걸어둔다.
        self.data.xfrc_applied[body_id, 0:3] = force
        self.data.xfrc_applied[body_id, 3:6] = -SPRING_KD_ROT * cur_angvel

    def apply_rotation_spring(self, body_id, target_quat):
        """apply_spring()이 건 토크(각속도 댐핑)에, "원래 목표 방향을
        보고 있어야 한다"는 복원 토크를 더한다. 쿼터니언 오차의
        벡터부(허수부)가 작은 오차에서는 회전축*sin(각도/2)에
        가까우므로, 표준 PD 자세 제어 근사로 그 값에 비례하는 토크를
        쓴다."""
        cur_quat = self.data.xquat[body_id]
        q_err = quat_mul(target_quat, quat_conj(cur_quat))
        if q_err[0] < 0.0:
            q_err = -q_err  # 쿼터니언은 q와 -q가 같은 회전이라, 더 짧은 쪽으로
        self.data.xfrc_applied[body_id, 3:6] += SPRING_KP_ROT * 2.0 * q_err[1:4]

    def update_proxy(self, name, body_id, qpos_adr, qvel_adr, mass, kd_pos,
                      pose_known, prev_target_pos):
        """target TF를 읽어서 스프링을 걸거나(이미 알고 있던 경우) 처음
        보는 거면 스냅한다. (pose_known, prev_target_pos, target_pos)를
        돌려준다 - 호출하는 쪽에서 self.xxx_pose_known/prev_target_pos에
        다시 저장해야 한다(이 메서드는 상태를 안 들고 있음)."""
        target = self.lookup_target(name)
        if target is None:
            return pose_known, prev_target_pos

        target_pos, target_quat = target

        if not pose_known:
            self.snap_body(qpos_adr, qvel_adr, target_pos, target_quat)
            return True, target_pos

        target_vel = (target_pos - prev_target_pos) / TIMER_PERIOD_SEC
        self.apply_spring(body_id, qvel_adr, mass, kd_pos, target_pos, target_vel)
        self.apply_rotation_spring(body_id, target_quat)
        return True, target_pos

    def respawn_ball(self):
        """둘 중 한쪽 몸통 바로 위에서 공을 다시 떨어뜨린다 - 매번
        실제로 부딪히는 모습을 보여주려는 의도. on_timer()가
        RESPAWN_INTERVAL_SEC마다 주기적으로 부르거나(주된 트리거),
        공이 FALL_RESET_Z보다 낮게 떨어진 예외 상황에서 부른다."""
        candidates = []
        if self.humanoid_pose_known:
            candidates.append(self.data.xpos[self.humanoid_body_id])
        if self.quad_pose_known:
            candidates.append(self.data.xpos[self.quad_body_id])

        if candidates:
            target = random.choice(candidates)
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
        self.humanoid_pose_known, self.humanoid_prev_target_pos = self.update_proxy(
            "base_link_target", self.humanoid_body_id,
            self.humanoid_qpos_adr, self.humanoid_qvel_adr,
            self.humanoid_mass, self.humanoid_kd_pos,
            self.humanoid_pose_known, self.humanoid_prev_target_pos,
        )
        self.quad_pose_known, self.quad_prev_target_pos = self.update_proxy(
            "quad_base_link_target", self.quad_body_id,
            self.quad_qpos_adr, self.quad_qvel_adr,
            self.quad_mass, self.quad_kd_pos,
            self.quad_pose_known, self.quad_prev_target_pos,
        )

        for _ in range(PHYSICS_SUBSTEPS):
            mujoco.mj_step(self.model, self.data)

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
            self.broadcast_tf(now, "base_link", self.data.xpos[self.humanoid_body_id],
                               self.data.xquat[self.humanoid_body_id])
        if self.quad_pose_known:
            self.broadcast_tf(now, "quad_base_link", self.data.xpos[self.quad_body_id],
                               self.data.xquat[self.quad_body_id])

        self.publish_markers(now, ball_pos, ball_quat)

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
