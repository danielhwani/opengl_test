#!/usr/bin/env python3
"""quad_mujoco_walk_ball
-----------------------
quad_mujoco_walk.py(실제 관절 액추에이터로 걷는, mocap/weld 트릭이
전혀 없는 "진짜" 물리 4족 로봇)에 공 던지기를 더한 버전. 독립
MuJoCo 스크립트(quad_walk_ball_throw.py)와 MuJoCo 자체 뷰어
(view_quad_walk_ball_throw.py)로 먼저 ROS 없이 확인한 결과를 그대로
ROS2/RViz2로 가져온다 - mocap+weld로 "항상 제자리로 끌어당기는"
mujoco_ball_sim.py의 공 데모와 달리, 여기는 그런 안전장치가 전혀
없다: 약하게 맞으면 걸으면서 우연히 균형을 되찾지만(실측:
0.43kg/5m/s는 순간 기울기 18도까지 튀었다가 1초 만에 원래 걸음
수준으로 회복), 세게 맞으면 그냥 쓰러지고(실측: 2kg/6m/s부터 0.9초
만에 완전히 옆으로 누움) 쓰러진 채로는 "일어나기" 동작이 없어서
다리만 허우적댄다 - 진짜 물리의 정직한 결과를 그대로 보여준다.

quad_mujoco_walk.py의 compute_leg_targets()/상수/quat_rotate_vec()을
그대로 import해서 재사용한다(걸음걸이 공식을 또 베끼지 않음) - 이
노드는 거기에 "공 바디 추가 + 공 투척 트리거"만 더한다.

**공 투척 트리거**: `throw_ball`(std_srvs/Trigger) **서비스**를
호출할 때마다 THROW_PRESETS를 순서대로 하나씩(끝까지 가면 처음으로
순환) 로봇 바로 옆에서 던진다 - MuJoCo 자체 뷰어 버전(view_quad_
walk_ball_throw.py)에서 `/` 키를 눌러 던지던 것과 똑같은 동작/세기
목록을 ROS로 흉내낸다(RViz만 보는 환경이라 키보드 콜백을 쓸 수
없음). 사용자가 직접(Claude는 ROS2/DDS 그래프에 참여하지 않음 -
README의 "Claude가 직접 노드를 띄워 테스트하는 것의 위험성" 참고)
아래처럼 호출한다:

  ros2 service call /quadruped/throw_ball std_srvs/srv/Trigger "{}"

**2026-11-16: 토픽(std_msgs/Empty)에서 서비스로 바꿈** - 처음엔
`ros2 topic pub`으로 Empty를 보내는 방식이었는데, 투척 결과(질량/
속도)를 launch 터미널의 `get_logger().info()` 로그로만 보여줬더니
두 가지 문제가 있었다: (1) RViz를 보는 동안엔 그 터미널이 안 보여서
놓치기 쉬웠고(그래서 아래처럼 공 위에 TEXT_VIEW_FACING 마커도
추가함), (2) 그보다 더 큰 문제로 Windows에서 `ros2 launch`가 자식
프로세스 stdout을 파이프로 받다 보니(진짜 터미널이 아님) C
표준입출력이 풀 버퍼링으로 전환돼서 로그가 바로 안 뜨고 쌓였다가
프로세스 종료 시점에야 몰아서 나왔다(`emulate_tty`+
`PYTHONUNBUFFERED=1`을 noda에 줘봤는데도 여전히 안 보임 - 아마
Windows에 pty가 없어서 emulate_tty 자체가 POSIX만큼 효과가 없는
듯). 사용자 제안대로 **"공을 던지는 그 터미널"에 바로 보이게**
토픽 대신 서비스로 바꿨다 - `std_srvs/Trigger`의 응답
(`success`/`message`)에 질량/속도/설명을 담으면, `ros2 service
call`이 호출한 터미널에 응답을 자동으로 찍어주므로(서비스 호출은
요청-응답이 한 프로세스 안에서 바로 왕복해서, launch가 띄운 별도
프로세스의 출력 버퍼링 문제 자체를 피해간다) 버퍼링 문제와 완전히
무관해진다.

(quad_mujoco_walk과 같은 이유로 namespace="quadruped" - 그래서 실제
서비스는 /quadruped/throw_ball, 토픽은 /quadruped/cmd_vel이다.)

공 자체는 quad_walk.xml 파일을 안 건드리고(다른 데모가 그 파일을
그대로 쓰므로), 그 내용을 읽어서 공 바디를 메모리상에서만 끼워
넣은 뒤 문자열로 컴파일한다(view_quad_walk_ball_throw.py와 같은
방법 - MuJoCo는 컴파일된 모델에 바디를 런타임에 추가 못 함).

가시화: 로봇 메쉬/관절은 기존 quadruped.urdf + /quadruped/joint_states
+ "world"->"quad_base_link" TF로(quad_mujoco_walk.py와 동일). 공은
URDF가 없으므로 mujoco_ball_sim.py와 같은 방식(visualization_msgs/
MarkerArray, 빨간 구)으로 "physics_markers" 토픽에 발행한다 - 이
토픽 이름을 mujoco_ball_sim.py와 똑같이 맞춰서(네임스페이스 없이)
기존 rviz/humanoid_sim.rviz의 PhysicsMarkers 디스플레이를 그대로
재사용한다(RViz 설정을 새로 안 건드려도 됨).
"""

import math
import os

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Twist
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

import mujoco

from humanoid_sim.quad_mujoco_walk import (
    JOINT_NAMES,
    LEGS,
    QUAD_BODY_HEIGHT_OFFSET,
    CMD_VEL_TIMEOUT_SEC,
    HIP_ACTUATOR_NAMES,
    KNEE_ACTUATOR_NAMES,
    compute_leg_targets,
    quat_rotate_vec,
)

TIMER_PERIOD_SEC = 0.01
PHYSICS_SUBSTEPS = 2

# 독립 스크립트(quad_walk_ball_throw.py)로 먼저 확인한 스펙트럼 -
# view_quad_walk_ball_throw.py와 동일(약 -> 강, 설명 문자열도 그대로).
THROW_PRESETS = [
    (0.5, 2.0, "가벼운 공(0.5kg/2m/s) - 거의 안 느낌"),
    (0.43, 5.0, "축구공(0.43kg/5m/s) - 휘청하다 스스로 회복"),
    (2.0, 6.0, "무거운 공(2.0kg/6m/s) - 쓰러짐, 못 일어남"),
    (3.0, 10.0, "아주 세게(3.0kg/10m/s) - 더 빨리 쓰러짐"),
]
SPAWN_SIDE_DIST = 0.7  # 몸통 옆 이만큼에서 던짐(비행시간 짧게 유지 - 중력으로 처져서 빗나가는 일 방지)

BALL_BODY_XML = """
    <body name="throw_ball" pos="5 0 1">
      <freejoint/>
      <geom name="throw_ball_geom" type="sphere" size="0.15" mass="0.5" rgba="0.85 0.25 0.25 1"/>
    </body>
"""


def default_mjcf_path():
    """quad_mujoco_walk.py의 같은 이름 함수와 동일한 탐색 순서(설치된
    공유 디렉터리 우선, 안 되면 소스 상대경로) - quad_walk.xml 자체를
    가리킨다(공 바디가 없는 원본, 아래에서 메모리상으로 합친다)."""
    try:
        share_dir = get_package_share_directory("humanoid_sim")
        return os.path.join(share_dir, "mujoco", "quad_walk.xml")
    except Exception:
        return os.path.normpath(
            os.path.join(os.path.dirname(__file__), "..", "mujoco", "quad_walk.xml")
        )


def load_model_with_ball(mjcf_path):
    with open(mjcf_path, "r", encoding="utf-8") as f:
        xml = f.read()
    xml = xml.replace("</worldbody>", BALL_BODY_XML + "  </worldbody>")
    return mujoco.MjModel.from_xml_string(xml)


def free_joint_adr(model, body_id):
    jnt_id = model.body_jntadr[body_id]
    return model.jnt_qposadr[jnt_id], model.jnt_dofadr[jnt_id]


class QuadMujocoWalkBall(Node):
    def __init__(self):
        super().__init__("quad_mujoco_walk_ball")

        self.declare_parameter("mjcf_path", default_mjcf_path())
        mjcf_path = self.get_parameter("mjcf_path").get_parameter_value().string_value

        self.model = load_model_with_ball(mjcf_path)
        self.data = mujoco.MjData(self.model)

        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "quad_body")
        self.ball_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "throw_ball")
        self.ball_qpos_adr, self.ball_qvel_adr = free_joint_adr(self.model, self.ball_body_id)
        # 첫 투척 전까지 로봇과 무관하게 장면 한쪽에 치워둔다.
        self.data.qpos[self.ball_qpos_adr:self.ball_qpos_adr + 3] = [5.0, 0.0, 1.0]
        mujoco.mj_forward(self.model, self.data)

        self.joint_qpos_adr = {
            name: self.model.jnt_qposadr[
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ]
            for name in JOINT_NAMES
        }
        self.hip_actuator_id = {
            leg: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for leg, name in HIP_ACTUATOR_NAMES.items()
        }
        self.knee_actuator_id = {
            leg: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for leg, name in KNEE_ACTUATOR_NAMES.items()
        }

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        # mujoco_ball_sim.py와 같은 이유로 네임스페이스 없이(전역
        # "physics_markers") 발행해서 기존 rviz 설정의 PhysicsMarkers
        # 디스플레이를 그대로 재사용한다.
        self.marker_pub = self.create_publisher(MarkerArray, "/physics_markers", 10)

        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)

        self.throw_idx = 0
        self.last_throw_label = None  # (mass, speed) - 공 위 텍스트 마커용
        self.throw_srv = self.create_service(Trigger, "throw_ball", self.on_throw_ball)

        self.timer = self.create_timer(TIMER_PERIOD_SEC, self.on_timer)
        self.get_logger().info(f"MuJoCo 모델 로드(공 포함): {mjcf_path}")
        self.get_logger().info(
            "공 투척: ros2 service call /quadruped/throw_ball std_srvs/srv/Trigger \"{}\""
        )

    def on_cmd_vel(self, msg):
        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def on_throw_ball(self, _request, response):
        mass, speed, desc = THROW_PRESETS[self.throw_idx % len(THROW_PRESETS)]
        self.throw_idx += 1
        self.model.body_mass[self.ball_body_id] = mass
        body_pos = self.data.xpos[self.body_id].copy()
        spawn = body_pos + np.array([0.3, -SPAWN_SIDE_DIST, 0.0])
        flight_t = SPAWN_SIDE_DIST / max(speed, 0.1)
        vz_compensate = 0.5 * 9.81 * flight_t
        adr = self.ball_qpos_adr
        vadr = self.ball_qvel_adr
        self.data.qpos[adr:adr + 3] = spawn
        self.data.qpos[adr + 3:adr + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[vadr:vadr + 3] = [0.0, speed, vz_compensate]
        self.data.qvel[vadr + 3:vadr + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.last_throw_label = (mass, speed)
        # 서비스 응답(success/message)에 담아서 반환 - ros2 service call을
        # 호출한 바로 그 터미널에 이 메시지가 자동으로 찍힌다(launch가
        # 띄운 별도 프로세스의 stdout 버퍼링 문제와 무관 - 위 모듈
        # docstring의 2026-11-16 항목 참고).
        response.success = True
        response.message = f"질량={mass}kg 속도={speed}m/s - {desc}"
        self.get_logger().info(f"투척: {response.message}")
        return response

    def on_timer(self):
        age_sec = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age_sec > CMD_VEL_TIMEOUT_SEC:
            linear_x, angular_z = 0.0, 0.0
        else:
            linear_x, angular_z = self.cmd_linear_x, self.cmd_angular_z

        hip_rad, knee_rad = compute_leg_targets(self.data.time, linear_x, angular_z)
        for leg in LEGS:
            self.data.ctrl[self.hip_actuator_id[leg]] = hip_rad[leg]
            self.data.ctrl[self.knee_actuator_id[leg]] = knee_rad[leg]

        for _ in range(PHYSICS_SUBSTEPS):
            mujoco.mj_step(self.model, self.data)

        now = self.get_clock().now()

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = list(JOINT_NAMES)
        js.position = [float(self.data.qpos[self.joint_qpos_adr[n]]) for n in JOINT_NAMES]
        self.joint_pub.publish(js)

        body_pos = self.data.xpos[self.body_id]
        body_quat = self.data.xquat[self.body_id]
        offset_world = quat_rotate_vec(body_quat, np.array([0.0, 0.0, QUAD_BODY_HEIGHT_OFFSET]))
        base_pos = body_pos - offset_world

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "world"
        tf.child_frame_id = "quad_base_link"
        tf.transform.translation.x = float(base_pos[0])
        tf.transform.translation.y = float(base_pos[1])
        tf.transform.translation.z = float(base_pos[2])
        tf.transform.rotation.w = float(body_quat[0])
        tf.transform.rotation.x = float(body_quat[1])
        tf.transform.rotation.y = float(body_quat[2])
        tf.transform.rotation.z = float(body_quat[3])
        self.tf_broadcaster.sendTransform(tf)

        self.publish_ball_marker(now)

    def publish_ball_marker(self, stamp):
        ball_pos = self.data.xpos[self.ball_body_id]
        ball_quat = self.data.xquat[self.ball_body_id]

        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = stamp.to_msg()
        m.ns = "physics"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(ball_pos[0])
        m.pose.position.y = float(ball_pos[1])
        m.pose.position.z = float(ball_pos[2])
        m.pose.orientation.w = float(ball_quat[0])
        m.pose.orientation.x = float(ball_quat[1])
        m.pose.orientation.y = float(ball_quat[2])
        m.pose.orientation.z = float(ball_quat[3])
        m.scale.x = m.scale.y = m.scale.z = 0.30  # 지름 (반지름 0.15*2)
        m.color.r, m.color.g, m.color.b, m.color.a = 0.85, 0.25, 0.25, 1.0

        arr = MarkerArray()
        arr.markers.append(m)

        # 질량/속도 정보 - 터미널 로그로만 보여줬더니 RViz를 보고 있는
        # 동안엔 안 보인다는 피드백을 받아서, 공 바로 위에 떠다니는
        # 텍스트로도 같이 보여준다(TEXT_VIEW_FACING - 카메라를 항상
        # 바라봐서 어느 각도에서도 읽을 수 있다). 아직 한 번도 안
        # 던졌으면(last_throw_label이 None) 표시 안 함.
        if self.last_throw_label is not None:
            mass, speed = self.last_throw_label
            t = Marker()
            t.header.frame_id = "world"
            t.header.stamp = stamp.to_msg()
            t.ns = "physics"
            t.id = 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(ball_pos[0])
            t.pose.position.y = float(ball_pos[1])
            t.pose.position.z = float(ball_pos[2]) + 0.35  # 공 바로 위
            t.pose.orientation.w = 1.0
            t.scale.z = 0.25  # 글자 높이(m)
            t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 1.0, 1.0, 1.0
            t.text = f"{mass:g}kg / {speed:g}m/s"
            arr.markers.append(t)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = QuadMujocoWalkBall()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
