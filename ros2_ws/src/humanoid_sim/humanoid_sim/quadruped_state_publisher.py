#!/usr/bin/env python3
"""quadruped_state_publisher
---------------------------
humanoid_state_publisher.py의 4족 로봇 버전. computer_graphics의
humanoid_test.c / humanoid_loader_test.c에서 4족 로봇이 하던
x축 왕복(patrol) + 트롯(trot) 걸음걸이를 ROS2 joint_states + TF로
재현한다.

- 트롯 걸음걸이(대각선 다리 쌍이 반대 위상으로 30도 진폭 스윙)를
  /joint_states로 발행한다 - robot_state_publisher가 quadruped.urdf를
  읽어서 TF를 계산해준다.
- x축 왕복 위치 + tanh로 완만하게 도는 방향 전환은 URDF가 표현하지
  못하는 "세계 속 로봇의 위치"라서, 이 노드가 "world" ->
  "quad_base_link" TF를 직접 발행한다 (quadruped.urdf의 루트가
  humanoid.urdf의 "base_link"와 안 겹치게 "quad_base_link"인 이유는
  quadruped.urdf 안의 주석 참고).

좌표 변환은 humanoid_state_publisher.py와 동일:
ros_xyz = (ours_z, ours_x, ours_y).
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

TIMER_PERIOD_SEC = 0.02   # 50 Hz
T_INCREMENT = 0.004       # humanoid_test.c의 idle()과 동일

TURN_SHARPNESS = 6.0      # QUAD_TURN_SHARPNESS와 동일

# quadruped_patrol.txt 기본값과 동일 ("z x_min x_max speed")
DEFAULT_PATROL = (-9.0, -4.0, 4.0, 0.3)

DEFAULT_PATROL_FILE = "D:/claude_test/computer_graphics/quadruped_patrol.txt"


def load_patrol(filename):
    """humanoid_test.c의 load_quad_patrol()과 동일: 첫 번째 유효한
    "z x_min x_max speed" 줄만 쓴다. 파일이 없으면 기본값."""
    try:
        with open(filename, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) == 4:
                    return tuple(float(p) for p in parts)
    except OSError:
        pass
    return DEFAULT_PATROL


class QuadrupedStatePublisher(Node):
    def __init__(self):
        super().__init__("quadruped_state_publisher")

        self.declare_parameter("patrol_file", DEFAULT_PATROL_FILE)
        patrol_file = self.get_parameter("patrol_file").get_parameter_value().string_value
        self.patrol_z, self.x_min, self.x_max, self.speed = load_patrol(patrol_file)

        self.t = 0.0
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(TIMER_PERIOD_SEC, self.on_timer)

    def on_timer(self):
        self.t += T_INCREMENT
        now = self.get_clock().now()

        # ---- 트롯 걸음걸이 ----
        swing_deg = 30.0 * math.sin(2.0 * self.t)
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [
            "quadbody_to_QuadLegFL",
            "quadbody_to_QuadLegBR",
            "quadbody_to_QuadLegFR",
            "quadbody_to_QuadLegBL",
        ]
        js.position = [
            math.radians(swing_deg),
            math.radians(swing_deg),
            math.radians(-swing_deg),
            math.radians(-swing_deg),
        ]
        self.joint_pub.publish(js)

        # ---- x축 왕복(patrol) 위치 + 완만한 방향 전환 ----
        center = (self.x_min + self.x_max) / 2.0
        half_range = (self.x_max - self.x_min) / 2.0
        phase = self.speed * self.t
        quad_x = center + half_range * math.sin(phase)
        heading_deg = 90.0 * math.tanh(TURN_SHARPNESS * math.cos(phase))

        # Y-up(ours) -> Z-up(ROS): ros_xyz = (ours_z, ours_x, ours_y).
        # ours 위치는 (quad_x, 0, patrol_z)라 ros는 (patrol_z, quad_x, 0).
        ros_x = self.patrol_z
        ros_y = quad_x
        yaw = math.radians(heading_deg)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "world"
        tf.child_frame_id = "quad_base_link"
        tf.transform.translation.x = ros_x
        tf.transform.translation.y = ros_y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = math.sin(yaw / 2.0)
        tf.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
