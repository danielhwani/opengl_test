#!/usr/bin/env python3
"""display.launch.py
--------------------
humanoid.urdf / quadruped.urdf(gen_humanoid_assets.exe가 생성)를
robot_state_publisher에 각각 넘기고, humanoid_state_publisher /
quadruped_state_publisher가 joint_states + world->base_link(또는
world->quad_base_link) TF를 발행하게 한 다음 rviz2를 띄운다.

두 로봇을 /humanoid, /quadruped 네임스페이스로 분리했다 - 그래야
각자의 robot_state_publisher가 자기 joint_states 토픽만 받고
(두 노드가 같은 토픽에 서로 다른 조인트 이름을 발행해서 섞이는 일이
없게), TF 프레임(Body/QuadBody 등)은 어차피 이름이 겹치지 않으니
그대로 전역 /tf에 합쳐진다.

실행 (colcon build 후):
  ros2 launch humanoid_sim display.launch.py
  ros2 launch humanoid_sim display.launch.py assets_dir:=D:/some/other/path

RViz2에서 수동으로 설정해야 할 때 (저장된 .rviz 설정이 안 맞으면):
  - Global Options -> Fixed Frame: world
  - Add -> By display type -> RobotModel (x2)
      Description Topic: /humanoid/robot_description
      Description Topic: /quadruped/robot_description
  - Add -> By display type -> TF
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
    quadruped_urdf_path = os.path.join(assets_dir, "quadruped.urdf")
    humanoid_path_file = os.path.join(assets_dir, "humanoid_path.txt")
    quadruped_patrol_file = os.path.join(assets_dir, "quadruped_patrol.txt")

    with open(humanoid_urdf_path, "r", encoding="utf-8") as f:
        humanoid_desc = f.read()
    with open(quadruped_urdf_path, "r", encoding="utf-8") as f:
        quadruped_desc = f.read()

    try:
        rviz_config = os.path.join(
            get_package_share_directory("humanoid_sim"), "rviz", "humanoid_sim.rviz"
        )
        rviz_args = ["-d", rviz_config]
    except Exception:
        # rviz 설정 파일을 못 찾아도 rviz2는 그냥 빈 화면으로 띄운다 -
        # 그 다음 위 docstring대로 Displays를 수동으로 추가하면 된다.
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
            executable="humanoid_state_publisher",
            namespace="humanoid",
            name="humanoid_state_publisher",
            output="screen",
            parameters=[{"path_file": humanoid_path_file}],
        ),
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
            executable="quadruped_state_publisher",
            namespace="quadruped",
            name="quadruped_state_publisher",
            output="screen",
            parameters=[{"patrol_file": quadruped_patrol_file}],
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
        description=(
            "gen_humanoid_assets.exe가 생성한 humanoid.urdf / quadruped.urdf / "
            "humanoid_path.txt / quadruped_patrol.txt가 있는 디렉터리"
        ),
    )
    return LaunchDescription([assets_dir_arg, OpaqueFunction(function=launch_setup)])
