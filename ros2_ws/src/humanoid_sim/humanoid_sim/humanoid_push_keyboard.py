#!/usr/bin/env python3
"""humanoid_push_keyboard
--------------------------
`view_humanoid_stand.py`의 화살표 키 push("가만히 서있으니 아무것도
안 하는 것 같다"는 사용자 코멘트로 추가했던 기능)를 RViz2/ROS2
쪽에서도 쓸 수 있게 - teleop_keyboard.py와 똑같은 방식(Windows
`msvcrt`로 터미널에서 직접 키 읽기)으로 만든 작은 노드.

cmd_vel과 달리 push는 "순간 충격"이라 **한 번 누를 때마다 딱 한 번만
발행**한다(teleop_keyboard처럼 타임아웃 안전장치용 10Hz 재발행이
필요 없음 - 안 눌려 있는 동안 아무것도 안 보내는 게 맞음).
humanoid_mujoco_walk.py의 PUSH_SPEED_MPS(0.8 m/s)를 그대로 재사용
(import로 - 숫자를 또 베끼지 않음).

실행:
  ros2 run humanoid_sim humanoid_push_keyboard
  (다른 토픽을 쓰려면: ros2 run humanoid_sim humanoid_push_keyboard --ros-args -p topic:=/humanoid/push)
기본 토픽은 /humanoid/push다(humanoid_mujoco_walk.py가
namespace="humanoid"로 떠 있으므로).

Up/Down/Left/Right: 그 방향으로 툭 밀기(여러 번 누르면 세게 누적 -
humanoid_mujoco_walk.py의 on_push()가 매번 더함), Ctrl+C: 종료.
"""

import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

from humanoid_sim.humanoid_mujoco_walk import PUSH_SPEED_MPS

if sys.platform != "win32":
    raise RuntimeError("humanoid_push_keyboard는 Windows 전용(msvcrt)입니다 - 이 프로젝트의 ROS2 설치가 Windows라 문제없음")

import msvcrt  # noqa: E402  (win32 전용 - 위 플랫폼 체크 다음에 import)

TIMER_PERIOD_SEC = 0.05  # 키 입력 폴링 주기 - push 자체는 눌릴 때만 1회 발행


class HumanoidPushKeyboard(Node):
    def __init__(self):
        super().__init__("humanoid_push_keyboard")
        self.declare_parameter("topic", "/humanoid/push")
        topic = self.get_parameter("topic").get_parameter_value().string_value

        self.pub = self.create_publisher(Vector3, topic, 10)
        self.timer = self.create_timer(TIMER_PERIOD_SEC, self.on_timer)
        print(f"humanoid_push_keyboard: '{topic}'로 발행합니다.")
        print("Up/Down/Left/Right: 그 방향으로 밀기(여러 번 누르면 세게 누적), Ctrl+C: 종료")

    def publish_push(self, dx, dy):
        msg = Vector3()
        msg.x = dx
        msg.y = dy
        msg.z = 0.0
        self.pub.publish(msg)
        print(f"push: x={dx:+.2f} y={dy:+.2f} m/s")

    def on_timer(self):
        while msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch in (b"\xe0", b"\x00"):  # 화살표 키 - 다음 바이트가 스캔코드
                ch2 = msvcrt.getch()
                if ch2 == b"H":  # Up -> 앞으로(+X)
                    self.publish_push(PUSH_SPEED_MPS, 0.0)
                elif ch2 == b"P":  # Down -> 뒤로(-X)
                    self.publish_push(-PUSH_SPEED_MPS, 0.0)
                elif ch2 == b"K":  # Left -> 왼쪽으로(+Y)
                    self.publish_push(0.0, PUSH_SPEED_MPS)
                elif ch2 == b"M":  # Right -> 오른쪽으로(-Y)
                    self.publish_push(0.0, -PUSH_SPEED_MPS)


def main(args=None):
    rclpy.init(args=args)
    node = HumanoidPushKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
