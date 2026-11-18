#!/usr/bin/env python3
"""humanoid_physics_walk.launch.py
----------------------------------
quad_physics_walk.launch.py의 휴머노이드 버전 - humanoid_state_publisher
(순수 애니메이션) 대신 humanoid_mujoco_walk(실제 관절 구동 물리
시뮬레이션)를 띄운다. 시각적 메쉬는 기존 humanoid.urdf를 그대로
쓴다 - 바뀌는 건 "몸통 위치를 누가/어떻게 정하는가"뿐이다.

**정직한 결과: 넘어진다** - humanoid_mujoco_walk.py/humanoid_walk.xml의
모듈 docstring과 README의 "휴머노이드 실제 관절 물리 보행 시도"
섹션 참고. 좌우 밸런스 메커니즘이 없어서 1~3초 안에 쓰러지는 게
정상 동작이다.

실행 (colcon build 후):
  ros2 launch humanoid_sim humanoid_physics_walk.launch.py
  ros2 launch humanoid_sim humanoid_physics_walk.launch.py assets_dir:=D:/some/path

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

    humanoid_urdf_path = os.path.join(assets_dir, "humanoid.urdf")
    with open(humanoid_urdf_path, "r", encoding="utf-8") as f:
        humanoid_desc = f.read()

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
            namespace="humanoid",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": humanoid_desc}],
        ),
        Node(
            package="humanoid_sim",
            executable="humanoid_mujoco_walk",
            namespace="humanoid",
            name="humanoid_mujoco_walk",
            output="screen",
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
        description="gen_humanoid_assets.exe가 생성한 humanoid.urdf가 있는 디렉터리",
    )
    return LaunchDescription([assets_dir_arg, OpaqueFunction(function=launch_setup)])
