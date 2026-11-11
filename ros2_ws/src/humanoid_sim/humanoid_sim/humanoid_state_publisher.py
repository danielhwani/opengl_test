#!/usr/bin/env python3
"""humanoid_state_publisher
--------------------------
computer_graphics/humanoid_test.c (와 humanoid_loader_test.c)의
idle()이 하던 걷기 애니메이션을 ROS2로 그대로 옮겨온 노드.

- 팔다리 스윙(35/25도 진폭의 sin(t))을 sensor_msgs/JointState로
  /joint_states에 발행한다 - robot_state_publisher가 humanoid.urdf를
  읽어서 이 각도들로부터 모든 링크의 TF를 계산해 발행해준다.
- 몸통이 humanoid_path.txt의 웨이포인트를 따라 걷는 것(path_sample,
  Catmull-Rom 스플라인)은 URDF가 표현하지 못하는 "세계 속 로봇의
  위치"라서, 이 노드가 "world" -> "base_link_target" TF를 발행한다.
  publish_base_link 파라미터(기본 True)가 켜져 있으면 같은 값을
  "world" -> "base_link"로도 발행한다 - mujoco_ball_sim이 그 프레임을
  물리 시뮬레이션 결과로 대신 발행할 때는(physics_demo.launch.py)
  이 파라미터를 False로 꺼서 두 발행자가 충돌하지 않게 한다.

좌표 변환: C 데모는 Y-up(X=왼쪽, Y=위, Z=앞)을 쓰고 ROS는 Z-up
(X=앞, Y=왼쪽, Z=위)을 쓴다. humanoid.urdf를 만들 때와 똑같이
ros_xyz = (ours_z, ours_x, ours_y)로 바꾼다 - 이건 두 우수(right-
handed) 좌표계 사이의 순수한 회전이라 회전 각도의 부호는 그대로
유지된다(heading을 그대로 yaw로 써도 된다).

t는 idle()과 똑같은 방식으로 흐른다: 타이머가 고정된 주기
(TIMER_PERIOD_SEC)로 불릴 때마다 T_INCREMENT만큼씩 누적한다 - 이
값들이 C 코드의 "t += 0.004"와 같아서, 실제 걷기/스윙 속도 비율이
원본 OpenGL 데모와 동일하다.

주의: 이번 포팅에서는 humanoid_test.c의 PATH/MANUAL/STOPPED 모드
구분이나 클론 캐릭터, 옷 색 틴트는 다루지 않는다 - "주인공 1명이
경로를 따라 걷는 것"만 그대로 옮겼다 (RViz는 보기용이라 키보드
조작까지는 포팅하지 않음). 자세한 손실/생략 정보는
computer_graphics/humanoid_extra.yaml 참고.
"""

import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

TIMER_PERIOD_SEC = 0.02   # 50 Hz
T_INCREMENT = 0.004       # humanoid_test.c의 idle()과 동일

# humanoid_test.c의 load_default_path()와 동일한 기본 사각형 경로
# (humanoid_path.txt를 못 읽을 때만 쓴다)
DEFAULT_WAYPOINTS = [(-2.0, -1.5), (2.0, -1.5), (2.0, 1.5), (-2.0, 1.5)]

DEFAULT_PATH_FILE = "D:/claude_test/computer_graphics/humanoid_path.txt"


def load_path(filename):
    """humanoid_test.c의 load_path()와 동일: "x z" 한 줄씩, #/빈 줄은
    건너뛴다. 파일이 없거나 점이 2개 미만이면 기본 사각형 경로."""
    waypoints = []
    try:
        with open(filename, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) == 2:
                    waypoints.append((float(parts[0]), float(parts[1])))
    except OSError:
        waypoints = []

    if len(waypoints) < 2:
        waypoints = list(DEFAULT_WAYPOINTS)

    n = len(waypoints)
    seg_len = [0.0] * n
    total_len = 0.0
    for i in range(n):
        j = (i + 1) % n
        dx = waypoints[j][0] - waypoints[i][0]
        dz = waypoints[j][1] - waypoints[i][1]
        seg_len[i] = math.hypot(dx, dz)
        total_len += seg_len[i]

    return waypoints, seg_len, total_len


def catmull_rom(p0, p1, p2, p3, frac):
    """humanoid_test.c의 catmull_rom()과 동일한 식. p0..p3는 (x,z) 튜플.
    위치와 그 지점의 접선(진행 방향)을 같이 돌려준다."""
    t2 = frac * frac
    t3 = t2 * frac

    def axis(a0, a1, a2, a3):
        pos = 0.5 * ((2.0 * a1) + (-a0 + a2) * frac
                      + (2.0 * a0 - 5.0 * a1 + 4.0 * a2 - a3) * t2
                      + (-a0 + 3.0 * a1 - 3.0 * a2 + a3) * t3)
        tan = 0.5 * ((-a0 + a2) + 2.0 * (2.0 * a0 - 5.0 * a1 + 4.0 * a2 - a3) * frac
                      + 3.0 * (-a0 + 3.0 * a1 - 3.0 * a2 + a3) * t2)
        return pos, tan

    x, tx = axis(p0[0], p1[0], p2[0], p3[0])
    z, tz = axis(p0[1], p1[1], p2[1], p3[1])
    return x, z, tx, tz


def path_sample(waypoints, seg_len, total_len, s):
    """humanoid_test.c의 path_sample()과 동일: 누적 거리 s에 해당하는
    지점의 (x, z, heading_deg)를 돌려준다."""
    n = len(waypoints)
    if n < 2 or total_len <= 0.0:
        return 0.0, 0.0, 0.0

    s = s % total_len
    if s < 0.0:
        s += total_len

    i = 0
    while i < n - 1 and s > seg_len[i]:
        s -= seg_len[i]
        i += 1

    im1 = (i - 1) % n
    j = (i + 1) % n
    jp1 = (i + 2) % n
    seg = seg_len[i]
    frac = (s / seg) if seg > 1e-9 else 0.0

    x, z, tx, tz = catmull_rom(waypoints[im1], waypoints[i], waypoints[j], waypoints[jp1], frac)

    if abs(tx) > 1e-9 or abs(tz) > 1e-9:
        heading_deg = math.degrees(math.atan2(tx, tz))
    else:
        dx = waypoints[j][0] - waypoints[i][0]
        dz = waypoints[j][1] - waypoints[i][1]
        heading_deg = math.degrees(math.atan2(dx, dz))

    return x, z, heading_deg


class HumanoidStatePublisher(Node):
    def __init__(self):
        super().__init__("humanoid_state_publisher")

        self.declare_parameter("path_file", DEFAULT_PATH_FILE)
        path_file = self.get_parameter("path_file").get_parameter_value().string_value
        self.waypoints, self.seg_len, self.total_len = load_path(path_file)
        if not os.path.isfile(path_file):
            self.get_logger().warn(
                f"경로 파일을 찾을 수 없어 기본 사각형 경로를 씁니다: {path_file}"
            )

        self.declare_parameter("publish_base_link", True)
        self.publish_base_link = self.get_parameter("publish_base_link").value

        self.t = 0.0
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(TIMER_PERIOD_SEC, self.on_timer)

    def on_timer(self):
        self.t += T_INCREMENT
        now = self.get_clock().now()

        # ---- 팔다리 스윙 (humanoid_test.c의 idle()과 동일한 식) ----
        left_arm_deg = 35.0 * math.sin(self.t)
        right_arm_deg = -35.0 * math.sin(self.t)
        left_leg_deg = -25.0 * math.sin(self.t)
        right_leg_deg = 25.0 * math.sin(self.t)

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [
            "body_to_leftarm",
            "body_to_rightarm",
            "body_to_leftleg",
            "body_to_rightleg",
        ]
        js.position = [
            math.radians(left_arm_deg),
            math.radians(right_arm_deg),
            math.radians(left_leg_deg),
            math.radians(right_leg_deg),
        ]
        self.joint_pub.publish(js)

        # ---- 몸통이 경로를 따라 걷는 위치 (path_sample) ----
        body_x, body_z, heading_deg = path_sample(
            self.waypoints, self.seg_len, self.total_len, 0.6 * self.t
        )

        # Y-up(ours) -> Z-up(ROS): ros_xyz = (ours_z, ours_x, ours_y).
        # 높이(ours_y)는 0이라 ros_z도 0 - base_link 위에서의 높이는
        # humanoid.urdf의 base_to_body fixed 관절이 이미 담당한다.
        ros_x = body_z
        ros_y = body_x
        yaw = math.radians(heading_deg)  # 회전 부호는 그대로 유지됨 (주석 참고)

        # "base_link_target"은 항상 발행한다 - "걷기 애니메이션이
        # 원하는" 순수 운동학적 목표 자세다. physics_demo.launch.py에서
        # mujoco_ball_sim이 이 target을 스프링으로 따라가는 물리
        # 시뮬레이션 결과를 "world"->"base_link"로 대신 발행하기 때문에
        # (공에 맞으면 순간적으로 target에서 벗어났다가 다시 돌아오게),
        # 그 경우엔 이 노드가 "base_link"를 발행하면 같은 프레임에
        # 두 발행자가 충돌한다 - 그래서 publish_base_link 파라미터로
        # 끌 수 있게 했다 (physics_demo.launch.py가 False로 띄움).
        # display.launch.py(물리 데모 없음)는 기본값 True라 이 노드가
        # 직접 "base_link"까지 발행해서 그대로 잘 보인다.
        tf_target = TransformStamped()
        tf_target.header.stamp = now.to_msg()
        tf_target.header.frame_id = "world"
        tf_target.child_frame_id = "base_link_target"
        tf_target.transform.translation.x = ros_x
        tf_target.transform.translation.y = ros_y
        tf_target.transform.translation.z = 0.0
        tf_target.transform.rotation.z = math.sin(yaw / 2.0)
        tf_target.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(tf_target)

        if self.publish_base_link:
            tf_base = TransformStamped()
            tf_base.header.stamp = now.to_msg()
            tf_base.header.frame_id = "world"
            tf_base.child_frame_id = "base_link"
            tf_base.transform = tf_target.transform
            self.tf_broadcaster.sendTransform(tf_base)


def main(args=None):
    rclpy.init(args=args)
    node = HumanoidStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
