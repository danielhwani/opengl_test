#!/usr/bin/env python3
"""physics_demo.launch.py
-------------------------
display.launch.py와 똑같이 humanoid/quadruped robot_state_publisher +
state publisher + rviz2를 띄우고, 거기에 더해 mujoco_ball_sim
노드까지 띄운다. 그 노드가 MuJoCo로 공 하나를 자유낙하시켜서, 지금
걷고 있는 humanoid와 순찰 중인 4족 로봇의 몸통(실시간 TF로 읽은
목표 자세를 스프링으로 따라가는 물리 바디)과 실제로 충돌시킨다 -
맞으면 로봇도 순간적으로 흔들렸다가 다시 애니메이션 자세로
돌아온다.

**중요 - 실행 전에 꼭 확인할 것**: mujoco_ball_sim은 pip 패키지
"mujoco"가 필요하다. 처음엔 conda 환경(mujoco_ros_38)에 설치하면
될 줄 알았는데, 이 ROS2 Humble(Windows) 설치는 colcon/ros2가 항상
`C:\\Python38`을 쓰도록 고정돼 있어서(conda activate를 해도 colcon
자체가 그 env에 없으면 PATH에서 C:\\Python38의 colcon이 호출됨),
결국 `mujoco`를 그 Python에 직접 설치해야 했다:

  C:\\Python38\\python.exe -m pip install mujoco
  # PyPI 인증서 문제가 나면: --trusted-host pypi.org
  #   --trusted-host files.pythonhosted.org 를 덧붙인다

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
            # publish_base_link=False: mujoco_ball_sim이 물리 반응이
            # 섞인 "world"->"base_link"를 대신 발행하므로, 이 노드는
            # "world"->"base_link_target"(순수 목표 자세)만 내보낸다.
            parameters=[{"path_file": humanoid_path_file, "publish_base_link": False}],
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
            # publish_base_link=False: 위 humanoid_state_publisher와
            # 같은 이유 - mujoco_ball_sim이 "world"->"quad_base_link"를
            # 대신 발행한다.
            parameters=[{"patrol_file": quadruped_patrol_file, "publish_base_link": False}],
        ),
        # mujoco_ball_sim은 네임스페이스를 안 준다 - tf2_ros의
        # TransformBroadcaster/Buffer는 항상 전역 "/tf"를 쓰므로(노드
        # 네임스페이스와 무관), humanoid/quadruped 네임스페이스 아래
        # 있는 두 state publisher가 발행하는 "world"->"base_link_target"/
        # "world"->"quad_base_link_target" TF를 네임스페이스 걱정
        # 없이 그대로 구독할 수 있다. 이 노드가 그 target을 스프링으로
        # 따라가는 물리 시뮬레이션 결과를 "world"->"base_link"/
        # "world"->"quad_base_link"로 대신 발행한다(위에서 두 state
        # publisher의 publish_base_link를 False로 끈 이유).
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
