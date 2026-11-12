#!/usr/bin/env python3
"""quad_physics_walk.launch.py
-----------------------------
display.launch.py의 4족 로봇 부분과 비슷하지만, quadruped_state_publisher
(순수 애니메이션 - 경로를 따라 위치를 직접 지정) 대신 quad_mujoco_walk
(실제 관절 구동 물리 시뮬레이션 - MuJoCo가 다리를 움직여서 몸통이
물리적으로 걸어가게 함)를 띄운다. 시각적 메쉬는 기존 quadruped.urdf를
그대로 쓴다 - 바뀌는 건 "몸통 위치를 누가/어떻게 정하는가"뿐이다.

실행 (colcon build 후):
  ros2 launch humanoid_sim quad_physics_walk.launch.py
  ros2 launch humanoid_sim quad_physics_walk.launch.py assets_dir:=D:/some/path

mujoco_ball_sim과 마찬가지로 pip 패키지 "mujoco"가 필요하다(이 ROS2
Humble(Windows) 설치는 C:\\Python38을 쓰므로 거기 설치해야 한다 -
humanoid_sim/README.md 참고).
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
            executable="quad_mujoco_walk",
            namespace="quadruped",
            name="quad_mujoco_walk",
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
        description="gen_humanoid_assets.exe가 생성한 quadruped.urdf가 있는 디렉터리",
    )
    return LaunchDescription([assets_dir_arg, OpaqueFunction(function=launch_setup)])
