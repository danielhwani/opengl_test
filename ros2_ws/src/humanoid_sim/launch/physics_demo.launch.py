#!/usr/bin/env python3
"""physics_demo.launch.py
-------------------------
display.launch.py와 똑같이 humanoid/quadruped robot_state_publisher +
state publisher + rviz2를 띄우고, 거기에 더해 mujoco_ball_sim
노드까지 띄운다. 그 노드가 MuJoCo로 공 하나를 자유낙하시켜서, 지금
걷고 있는 humanoid와 순찰 중인 4족 로봇의 몸통(실시간 TF로 읽은
위치)과 실제로 충돌시킨다.

**중요 - 실행 전에 꼭 확인할 것**: mujoco_ball_sim은 pip 패키지
"mujoco"가 필요하다. 이 패키지의 다른 두 노드(humanoid_state_publisher/
quadruped_state_publisher)는 mujoco가 없어도 되지만, 전체 colcon
빌드는 "어느 Python 환경이 활성화된 상태로 `colcon build`를
돌렸는지"에 따라 세 노드 모두의 실행 파일(entry point 스크립트)이
그 환경의 Python 인터프리터를 가리키게 된다. 즉 mujoco와 rclpy가
둘 다 있는 환경(예: conda 환경 mujoco_ros_38)을 활성화한 채로
`colcon build`를 다시 돌려야 이 launch가 제대로 동작한다:

  conda activate mujoco_ros_38
  cd D:\\claude_test\\computer_graphics\\ros2_ws
  colcon build --packages-select humanoid_sim
  call install\\setup.bat
  ros2 launch humanoid_sim physics_demo.launch.py

RViz2에서 수동으로 설정해야 할 때 (저장된 .rviz 설정이 안 맞으면):
  - display.launch.py와 동일 + Add -> By display type -> MarkerArray
      Marker Topic: /physics_markers
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
        # mujoco_ball_sim은 네임스페이스를 안 준다 - tf2_ros의
        # TransformBroadcaster/Buffer는 항상 전역 "/tf"를 쓰므로(노드
        # 네임스페이스와 무관), humanoid/quadruped 네임스페이스 아래
        # 있는 두 state publisher가 발행하는 "world"->"Body"/
        # "world"->"QuadBody" TF를 네임스페이스 걱정 없이 그대로
        # 구독할 수 있다.
        Node(
            package="humanoid_sim",
            executable="mujoco_ball_sim",
            name="mujoco_ball_sim",
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
        description=(
            "gen_humanoid_assets.exe가 생성한 humanoid.urdf / quadruped.urdf / "
            "humanoid_path.txt / quadruped_patrol.txt가 있는 디렉터리"
        ),
    )
    return LaunchDescription([assets_dir_arg, OpaqueFunction(function=launch_setup)])
