#!/usr/bin/env python3
"""mujoco_ball_sim
------------------
MuJoCo로 공을 하나 떨어뜨려서, 지금 걷고 있는 humanoid와 순찰 중인
4족 로봇의 몸통과 실제로 충돌(접촉)하게 만드는 노드.

humanoid_state_publisher / quadruped_state_publisher가 이미 발행하고
있는 "world" -> "Body" / "world" -> "QuadBody" TF를 매 틱 읽어서,
MuJoCo 장면(ball_collision.xml) 안의 두 "mocap" 바디(humanoid_proxy/
quad_proxy)의 자세로 그대로 복사해 넣는다. mocap 바디는 물리
시뮬레이션의 영향을 받지 않고(질량/관성이 적용되지 않음) 자세를
외부에서 그대로 주입받지만, 충돌에는 정상적으로 참여한다 - 그래서
"지금 이 순간 걷고 있는 humanoid/순찰 중인 4족 로봇의 몸통" 노릇을
한다. 공(ball)만 진짜 물리 바디(자유낙하 + 중력 + 충돌 반응)다.

공이 바닥 밑으로 너무 떨어지면(둘 중 하나와 충돌하지 않고 빗나간
경우) 둘 중 한쪽 몸통 위로 다시 떨어뜨린다 - 매번 실제로 부딪히는
모습을 보려는 의도.

결과는 두 가지로 RViz2에 보여준다:
  - TF: "world" -> "ball" (공의 실제 물리 위치/자세)
  - visualization_msgs/MarkerArray: 공(빨간 구) + 두 충돌 프록시
    (반투명 상자, "왜 거기서 부딪혔는지" 눈으로 보려고)

주의: 이 노드는 humanoid_state_publisher/quadruped_state_publisher가
TF를 발행하고 있어야 제대로 동작한다 (launch/physics_demo.launch.py가
그것까지 같이 띄운다). 그리고 이 노드를 실행하는 Python 환경에
"mujoco" 패키지가 설치돼 있어야 한다 (conda 환경 mujoco_ros_38에서
확인됨).
"""

import os
import random

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

import mujoco

TIMER_PERIOD_SEC = 0.01     # 100 Hz
PHYSICS_SUBSTEPS = 2        # MJCF의 timestep(0.005)과 곱해서 0.01s를 채움

FALL_RESET_Z = -2.0         # 이 높이보다 낮아지면 다시 떨어뜨림
RESPAWN_HEIGHT_OFFSET = 2.0  # 고른 몸통 위 이만큼 높은 곳에서 다시 떨어뜨림
RESPAWN_JITTER = 0.3        # 몸통 중심에서 좌우로 흔드는 정도(m)

DEFAULT_MJCF = os.path.join(os.path.dirname(__file__), "..", "mujoco", "ball_collision.xml")


def ros_quat_to_mujoco(q):
    """geometry_msgs/Quaternion(x,y,z,w) -> MuJoCo 쿼터니언(w,x,y,z)."""
    return [q.w, q.x, q.y, q.z]


class MujocoBallSim(Node):
    def __init__(self):
        super().__init__("mujoco_ball_sim")

        self.declare_parameter("mjcf_path", os.path.normpath(DEFAULT_MJCF))
        mjcf_path = self.get_parameter("mjcf_path").get_parameter_value().string_value

        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)

        self.ball_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "ball")
        self.humanoid_proxy_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "humanoid_proxy")
        self.quad_proxy_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "quad_proxy")

        self.humanoid_mocap_idx = self.model.body_mocapid[self.humanoid_proxy_id]
        self.quad_mocap_idx = self.model.body_mocapid[self.quad_proxy_id]

        # free joint는 이름을 안 줬으므로(MJCF의 <freejoint/>), ball 바디의
        # 첫 번째(유일한) 관절 주소를 직접 쓴다.
        self.ball_qpos_adr = self.model.jnt_qposadr[self.model.body_jntadr[self.ball_body_id]]
        self.ball_qvel_adr = self.model.jnt_dofadr[self.model.body_jntadr[self.ball_body_id]]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, "physics_markers", 10)

        self.humanoid_pose_known = False
        self.quad_pose_known = False

        self.timer = self.create_timer(TIMER_PERIOD_SEC, self.on_timer)
        self.get_logger().info(f"MuJoCo 모델 로드: {mjcf_path}")

    def update_proxy_from_tf(self, target_frame, mocap_idx):
        """"world" -> target_frame TF를 읽어서 mocap 바디 자세로 복사한다.
        아직 그 TF가 없으면(다른 노드가 아직 시작 전) 조용히 넘어간다."""
        try:
            tf = self.tf_buffer.lookup_transform("world", target_frame, Time())
        except Exception:
            return False

        t = tf.transform.translation
        self.data.mocap_pos[mocap_idx] = [t.x, t.y, t.z]
        self.data.mocap_quat[mocap_idx] = ros_quat_to_mujoco(tf.transform.rotation)
        return True

    def respawn_ball(self):
        """공이 바닥 아래로 너무 떨어지면, 둘 중 한쪽 몸통 바로 위에서
        다시 떨어뜨린다 - 매번 실제로 부딪히는 모습을 보여주려는
        의도."""
        candidates = []
        if self.humanoid_pose_known:
            candidates.append(self.data.mocap_pos[self.humanoid_mocap_idx])
        if self.quad_pose_known:
            candidates.append(self.data.mocap_pos[self.quad_mocap_idx])

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
        self.humanoid_pose_known = self.update_proxy_from_tf("Body", self.humanoid_mocap_idx) or self.humanoid_pose_known
        self.quad_pose_known = self.update_proxy_from_tf("QuadBody", self.quad_mocap_idx) or self.quad_pose_known

        for _ in range(PHYSICS_SUBSTEPS):
            mujoco.mj_step(self.model, self.data)

        ball_pos = self.data.xpos[self.ball_body_id]
        if ball_pos[2] < FALL_RESET_Z:
            self.respawn_ball()
            ball_pos = self.data.xpos[self.ball_body_id]

        ball_quat = self.data.xquat[self.ball_body_id]  # mujoco: w,x,y,z
        now = self.get_clock().now()

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "world"
        tf.child_frame_id = "ball"
        tf.transform.translation.x = float(ball_pos[0])
        tf.transform.translation.y = float(ball_pos[1])
        tf.transform.translation.z = float(ball_pos[2])
        tf.transform.rotation.w = float(ball_quat[0])
        tf.transform.rotation.x = float(ball_quat[1])
        tf.transform.rotation.y = float(ball_quat[2])
        tf.transform.rotation.z = float(ball_quat[3])
        self.tf_broadcaster.sendTransform(tf)

        self.publish_markers(now, ball_pos, ball_quat)

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
            (1, self.humanoid_proxy_id, self.humanoid_mocap_idx, (0.5, 0.8, 1.4), (0.55, 0.55, 0.65), self.humanoid_pose_known),
            (2, self.quad_proxy_id, self.quad_mocap_idx, (1.2, 0.5, 0.4), (0.50, 0.55, 0.60), self.quad_pose_known),
        ]
        for marker_id, body_id, mocap_idx, full_size, color, known in proxy_specs:
            if not known:
                continue
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = stamp.to_msg()
            m.ns = "physics"
            m.id = marker_id
            m.type = Marker.CUBE
            m.action = Marker.ADD
            pos = self.data.mocap_pos[mocap_idx]
            quat = self.data.mocap_quat[mocap_idx]
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
