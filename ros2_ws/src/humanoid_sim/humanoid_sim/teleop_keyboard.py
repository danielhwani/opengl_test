#!/usr/bin/env python3
"""teleop_keyboard
-----------------
`ros2 topic pub ... -r 10`로 매번 cmd_vel을 손으로 치는 게 귀찮다는
요청으로 추가 - 공식 `teleop_twist_keyboard` 패키지는 이 Windows
ROS2 Humble 설치엔 안 깔려 있고(`ros2 pkg list`로 확인, apt 바이너리가
없어 소스 빌드가 필요함), git 클론+colcon build까지 가는 것보다
이 프로젝트 안에 이미 있는 MuJoCo 뷰어 조작(view_quad_walk.py 등)과
똑같은 키 배치로 작은 노드를 직접 만드는 게 더 간단하고 일관적이라
이렇게 했다.

터미널에서 바로 키를 읽어야 해서(별도 GUI 창 없음) Windows 전용
`msvcrt` 모듈을 쓴다(이 프로젝트의 ROS2 설치 자체가 Windows
전용이라 문제없음) - 화살표 키는 msvcrt에서 2바이트로 온다(첫
바이트 b'\\xe0', 둘째 바이트가 스캔코드: Up=72, Down=80, Left=75,
Right=77).

조작은 view_quad_walk.py/view_quad_walk_ball_throw.py와 동일하게
맞췄다(한 프로젝트 안에서 조작 체계를 통일) - Up/Down: 전진/후진
속도 증가/감소, Left/Right: 좌/우 회전 속도 증가/감소, Enter: 정지,
Ctrl+C: 종료. 증가 폭도 quad_mujoco_walk.py의 CMD_MAX_SPEED_MPS/
CMD_MAX_TURN_RATE_RAD_S 기준 1/8씩으로 그 파일들과 통일했다(import로
재사용 - 숫자를 또 베끼지 않음).

cmd_vel은 CMD_VEL_TIMEOUT_SEC(1초) 안에 안 오면 로봇이 자동으로
멈추는 안전장치가 있어서(quad_mujoco_walk.py 참고), 이 노드는 현재
속도를 10Hz로 계속 재발행한다(값이 안 바뀌어도) - 안 그러면 가만히
서서 키를 안 누르는 동안 타임아웃으로 멈춰버린다.

실행:
  ros2 run humanoid_sim teleop_keyboard
  (다른 토픽을 쓰려면: ros2 run humanoid_sim teleop_keyboard --ros-args -p topic:=/quadruped/cmd_vel)
기본 토픽은 /quadruped/cmd_vel이다(quad_mujoco_walk.py/quad_mujoco_
walk_ball.py가 namespace="quadruped"로 떠 있으므로).
"""

import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from humanoid_sim.quad_mujoco_walk import CMD_MAX_SPEED_MPS, CMD_MAX_TURN_RATE_RAD_S

if sys.platform != "win32":
    raise RuntimeError("teleop_keyboard는 Windows 전용(msvcrt)입니다 - 이 프로젝트의 ROS2 설치가 Windows라 문제없음")

import msvcrt  # noqa: E402  (win32 전용 - 위 플랫폼 체크 다음에 import)

TIMER_PERIOD_SEC = 0.1  # 10Hz 재발행 - CMD_VEL_TIMEOUT_SEC(1초)보다 훨씬 촘촘해야 함
LINEAR_STEP_MPS = CMD_MAX_SPEED_MPS / 8.0
ANGULAR_STEP_RAD_S = CMD_MAX_TURN_RATE_RAD_S / 8.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.declare_parameter("topic", "/quadruped/cmd_vel")
        topic = self.get_parameter("topic").get_parameter_value().string_value

        self.pub = self.create_publisher(Twist, topic, 10)
        self.linear_x = 0.0
        self.angular_z = 0.0

        self.timer = self.create_timer(TIMER_PERIOD_SEC, self.on_timer)
        print(f"teleop_keyboard: '{topic}'로 발행합니다.")
        print("Up/Down: 전진/후진 속도 증가/감소, Left/Right: 좌/우 회전 증가/감소, "
              "Enter: 정지, Ctrl+C: 종료")

    def on_timer(self):
        while msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch in (b"\xe0", b"\x00"):  # 화살표 키 - 다음 바이트가 스캔코드
                ch2 = msvcrt.getch()
                if ch2 == b"H":  # Up
                    self.linear_x = clamp(self.linear_x + LINEAR_STEP_MPS, -CMD_MAX_SPEED_MPS, CMD_MAX_SPEED_MPS)
                elif ch2 == b"P":  # Down
                    self.linear_x = clamp(self.linear_x - LINEAR_STEP_MPS, -CMD_MAX_SPEED_MPS, CMD_MAX_SPEED_MPS)
                elif ch2 == b"K":  # Left
                    self.angular_z = clamp(self.angular_z + ANGULAR_STEP_RAD_S,
                                            -CMD_MAX_TURN_RATE_RAD_S, CMD_MAX_TURN_RATE_RAD_S)
                elif ch2 == b"M":  # Right
                    self.angular_z = clamp(self.angular_z - ANGULAR_STEP_RAD_S,
                                            -CMD_MAX_TURN_RATE_RAD_S, CMD_MAX_TURN_RATE_RAD_S)
                else:
                    continue
            elif ch in (b"\r", b"\n"):  # Enter
                self.linear_x = 0.0
                self.angular_z = 0.0
            else:
                continue
            print(f"cmd_vel: linear_x={self.linear_x:.3f} m/s  angular_z={self.angular_z:.4f} rad/s")

        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
