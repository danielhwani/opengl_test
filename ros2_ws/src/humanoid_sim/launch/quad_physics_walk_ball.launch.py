#!/usr/bin/env python3
"""quad_physics_walk_ball.launch.py
-----------------------------------
quad_physics_walk.launch.py와 동일하지만 quad_mujoco_walk 대신
quad_mujoco_walk_ball을 띄운다 - 실제 관절 구동 물리로 걷는(mocap/
weld 트릭 없는) 4족 로봇에 공을 던져서 반응을 본다(README의 "진짜
물리 4족 로봇에 공 던지기" 섹션 참고).

실행 (colcon build 후):
  ros2 launch humanoid_sim quad_physics_walk_ball.launch.py

RViz2가 뜬 뒤, 다른 터미널에서 공을 던지려면(약->강 순서로 순환) -
서비스 호출이라 호출한 터미널에 던진 공의 질량/속도가 바로 찍힌다:
  ros2 service call /quadruped/throw_ball std_srvs/srv/Trigger "{}"

걷게 하려면(quad_physics_walk.launch.py와 동일):
  ros2 topic pub /quadruped/cmd_vel geometry_msgs/msg/Twist \\
    "{linear: {x: 0.16}, angular: {z: 0.0}}" -r 10

mujoco_ball_sim/quad_mujoco_walk과 마찬가지로 pip 패키지 "mujoco"가
필요하다(이 ROS2 Humble(Windows) 설치는 C:\\Python38을 쓰므로 거기
설치해야 한다 - humanoid_sim/README.md 참고).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    assets_dir = LaunchConfiguration("assets_dir").perform(context)

    quadruped_urdf_path = os.path.join(assets_dir, "quadruped.urdf")
    with open(quadruped_urdf_path, "r", encoding="utf-8") as f:
        quadruped_desc = f.read()

    try:
        rviz_config = os.path.join(
            get_package_share_directory("humanoid_sim"), "rviz", "humanoid_sim.rviz"
        )
        rviz_args = ["-d", rviz_config]
    except Exception:
        rviz_args = []

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="quadruped",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": quadruped_desc}],
        ),
        Node(
            package="humanoid_sim",
            executable="quad_mujoco_walk_ball",
            namespace="quadruped",
            name="quad_mujoco_walk_ball",
            output="screen",
            # Windows에서 ros2 launch가 자식 프로세스 stdout을 파이프로
            # 받다 보니(진짜 터미널이 아님) C 표준입출력이 풀 버퍼링
            # 모드로 전환돼서, 로그가 바로 안 보이고 쌓였다가(또는
            # 프로세스가 끝날 때야) 몰아서 나오는 문제가 있었다(사용자가
            # "공 투척 로그가 터미널에 안 찍힌다"고 확인 - 콜백 자체는
            # 정상 실행 중이었다, RViz 마커는 바로바로 갱신됐음).
            # emulate_tty+PYTHONUNBUFFERED를 줘봤지만(Windows엔 POSIX
            # pty가 없어서 emulate_tty 효과가 제한적인 듯) 여전히 안
            # 보여서, 결국 투척 트리거 자체를 토픽 대신 서비스(아래
            # quad_mujoco_walk_ball.py의 /quadruped/throw_ball,
            # std_srvs/Trigger)로 바꿔서 launch 프로세스의 출력 버퍼링과
            # 아예 무관하게(서비스 응답은 호출한 터미널에 직접 찍힘)
            # 만들었다 - 이 두 옵션은 혹시 몰라 남겨둔다(해는 없음).
            emulate_tty=True,
            additional_env={"PYTHONUNBUFFERED": "1"},
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=rviz_args,
        ),
    ]


def generate_launch_description():
    assets_dir_arg = DeclareLaunchArgument(
        "assets_dir",
        default_value="D:/claude_test/computer_graphics",
        description="gen_humanoid_assets.exe가 생성한 quadruped.urdf가 있는 디렉터리",
    )
    return LaunchDescription([assets_dir_arg, OpaqueFunction(function=launch_setup)])
