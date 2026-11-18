#!/usr/bin/env python3
"""humanoid_mujoco_walk
----------------------
휴머노이드(2족)를 quad_mujoco_walk.py와 같은 방식(순수 애니메이션이
아니라 실제 MuJoCo 관절 액추에이터로 구동)으로 세워본다.

**경과 요약(humanoid_sim/README.md의 해당 섹션에 전체 기록)**:
1) 4족의 오픈루프 사인파 걸음걸이를 그대로 포팅 -> 넘어짐(1.15~2.54초).
2) 엉덩이 ab/duction(좌우) 축 추가, 오픈루프+PD -> 여전히 거의
   그대로 넘어짐(최선 2.4초 정도) - 앞/뒤(피치) 방향 실패가 상당수라
   좌우 축만으로는 구조적으로 못 고침. 다리 끝이 둥근 캡슐 "점
   접촉"이라 지지 다각형이 사실상 없었던 게 근본 원인.
3) 발목(롤+피치) + 평평한 발 추가 -> 패시브로 조금 나아졌지만 발목만
   으로는 복원력이 너무 약함. 엉덩이/허리/무릎까지 다 동원한 그리드
   서치+랜덤서치(최대 1000개 게인 조합)도 전부 6초 안에 실패 -
   단순 PD로는 한계.
4) 허리(롤+피치) 추가 + **LQR**(mujoco.mjd_transitionFD로 평형점
   수치 선형화 -> discrete Riccati를 numpy 반복법으로 직접 풀어
   최적 게인 계산, scipy 불필요) -> **300초 완전 안정, 최대 기울기
   0.06도**. PD로 찾은 최선(~5.8초) 대비 50배 이상 개선.

**그래서 이번엔 "서있기"만 안정화한다 - 걷기(전진)는 다음 단계.**
cmd_vel 없음, compute_leg_targets() 같은 오픈루프 걸음걸이 공식도
없음 - LQR 컨트롤러가 평형 자세(모든 관절 0, 똑바로 서기)를 그대로
유지하도록만 한다.

좌표/높이 변환은 quad_mujoco_walk.py와 동일한 방법(body_pose_to_root
스타일 - 몸통 중심에서 base_to_body 오프셋만큼 로컬 -Z 방향으로 뺀
지점이 base_link) - 이번에 몸통을 골반(torso)/가슴(chest)으로
나눴지만, 다리가 붙는 "torso"(골반) 링크의 원점 자체는 안 바뀌어서
이 변환 공식도 그대로 재사용 가능하다.
"""

import math
import os

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Vector3
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

import mujoco

TIMER_PERIOD_SEC = 0.01
PHYSICS_SUBSTEPS = 2

# view_humanoid_stand.py의 화살표 키 push와 같은 값/방식 - "/push"
# 토픽(Vector3, m/s 순간 속도 충격)을 받으면 그대로 토르소(자유관절)
# 선속도에 더한다. humanoid_push_keyboard.py가 이 토픽에 발행한다.
PUSH_SPEED_MPS = 0.8

CTRL_LIMIT_DEG = 30.0

# humanoid_stand_lqr.py에서 300초 완전 안정(max_tilt=0.06도)으로
# 검증한 비용함수 가중치. 자유관절(처음 6 tangent dof)의 0-2=위치
# 오차, 3-5=기울어짐(롤/피치/요), nv+0-2=선속도, nv+3-5=각속도.
Q_POS_WEIGHT = 5.0
Q_HEIGHT_WEIGHT = 2.0
Q_TILT_WEIGHT = 100.0
Q_LINVEL_WEIGHT = 0.5
Q_ANGVEL_WEIGHT = 10.0
R_WEIGHT = 0.2
# 어깨 ab/duction(2026-11-18 추가)은 균형에 대한 실제 기여가 약한데도
# LQR이 다른 축과 똑같은 R로 취급하면 평형점의 작은 수치 오차만으로
# 거의 바로 포화돼서 계속 그 상태를 유지하며 넘어뜨린다(어깨 제어만
# 꺼보면 300초 멀쩡히 서있는 것으로 확인 - 어깨 채널 자체가
# 문제였음). R을 다른 관절보다 비싸게 매겨서(LQR이 자연히 덜 쓰게)
# 스윕: 100배=112초, 500배부터 300초 완전 안정 - 여유를 둬서 500.
SHOULDER_R_MULTIPLIER = 500.0

EQUILIBRIUM_SETTLE_SEC = 0.3

# LEG_LEN(1.2) + BODY_H/2(0.7) - humanoid.urdf의 base_to_body_z와 동일
# (torso=골반 링크 원점은 허리를 나누기 전과 동일하게 안 바뀜).
HUMANOID_BODY_HEIGHT_OFFSET = 1.9

# MJCF 관절 이름 -> URDF(humanoid.urdf) 관절 이름. mjd_transitionFD로
# 선형화한 상태/제어 벡터는 MJCF의 model.nu/nv 순서를 그대로 쓰므로,
# JointState 발행 시에만 URDF 쪽 이름으로 바꿔준다.
MJCF_TO_URDF_JOINT = {
    "hip_abduct_L": "Body_to_LeftHip",
    "hip_abduct_R": "Body_to_RightHip",
    "hip_L": "body_to_leftleg",
    "hip_R": "body_to_rightleg",
    "knee_L": "LeftLeg_to_LeftShin",
    "knee_R": "RightLeg_to_RightShin",
    "ankle_roll_L": "LeftShin_to_LeftAnkleRoll",
    "ankle_roll_R": "RightShin_to_RightAnkleRoll",
    "ankle_pitch_L": "LeftAnkleRoll_to_LeftFoot",
    "ankle_pitch_R": "RightAnkleRoll_to_RightFoot",
    "waist_roll": "Body_to_ChestRoll",
    "waist_pitch": "ChestRoll_to_Chest",
    # 2026-11-17: 팔도 실제 물리에 추가(무게중심을 앞뒤로 움직여
    # 균형에 쓸 수 있는지 보려고) - humanoid_walk.xml의 arm_L/R.
    # URDF 쪽 이름(body_to_leftarm/rightarm)은 이전부터 있던 것
    # 그대로 재사용(관절 자체는 바뀐 게 없고, 전엔 고정 0만 받다가
    # 이제 실제 LQR 제어값을 받는 것만 다름).
    "arm_L": "body_to_leftarm",
    "arm_R": "body_to_rightarm",
    # 2026-11-18: 어깨 ab/duction 추가(사용자 요청 - "옆으로도 벌릴
    # 수 있게") - 엉덩이/발목/허리와 같은 패턴.
    "shoulder_abduct_L": "Chest_to_LeftShoulderAbduct",
    "shoulder_abduct_R": "Chest_to_RightShoulderAbduct",
    # 2026-11-18: 팔꿈치 추가(사용자 요청) - 무릎과 같은 패턴.
    "elbow_L": "LeftArm_to_LeftForearm",
    "elbow_R": "RightArm_to_RightForearm",
    # 2026-11-18: 전완 회전(회내/회외) 추가(사용자 요청 - "사람과
    # 유사하게") - 팔꿈치 굽힘 다음(안쪽)에 적용.
    "elbow_twist_L": "LeftForearmTwist_to_LeftForearm",
    "elbow_twist_R": "RightForearmTwist_to_RightForearm",
    # 2026-11-18: 목 추가(사용자 요청 - "허리 관절과 비슷하면 될 것
    # 같아") - 허리와 같은 패턴(롤 바깥 + 피치 안쪽). 피치 쪽 URDF
    # 이름(body_to_head)은 예전 fixed 관절 때부터 있던 것 재사용.
    "neck_roll": "Chest_to_HeadRoll",
    "neck_pitch": "body_to_head",
    # 2026-11-18: 손목 추가 - 엉덩이/발목/허리/목과 같은 패턴
    # (편위=롤, 바깥 먼저 / 굽힘=피치, 안쪽 나중).
    "wrist_deviate_L": "LeftForearm_to_LeftWristDeviate",
    "wrist_deviate_R": "RightForearm_to_RightWristDeviate",
    "wrist_flex_L": "LeftWristDeviate_to_LeftHand",
    "wrist_flex_R": "RightWristDeviate_to_RightHand",
}


def default_mjcf_path():
    """quad_mujoco_walk.py의 같은 이름 함수와 동일한 탐색 순서."""
    try:
        share_dir = get_package_share_directory("humanoid_sim")
        return os.path.join(share_dir, "mujoco", "humanoid_walk.xml")
    except Exception:
        return os.path.normpath(
            os.path.join(os.path.dirname(__file__), "..", "mujoco", "humanoid_walk.xml")
        )


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_rotate_vec(q, v):
    v_quat = np.array([0.0, v[0], v[1], v[2]])
    result = quat_mul(quat_mul(q, v_quat), quat_conj(q))
    return result[1:4]


def find_equilibrium(model):
    """ctrl=0(똑바로 서기 목표)으로 짧게 settle시켜 접촉 과도응답을
    가라앉힌 뒤의 상태를 LQR 선형화 기준점(평형점)으로 쓴다."""
    data = mujoco.MjData(model)
    for _ in range(int(EQUILIBRIUM_SETTLE_SEC / model.opt.timestep)):
        mujoco.mj_step(model, data)
    return data.qpos.copy(), data.qvel.copy()


def linearize(model, qpos_eq, qvel_eq):
    """mujoco.mjd_transitionFD로 평형점 주변을 수치 선형화 - A, B를
    손으로 유도할 필요 없이(부호/커플링 전부 자동) 시뮬레이터가
    직접 계산해준다. 상태는 MuJoCo 표준 방식대로 [qpos를 접공간으로
    투영한 오차(nv), qvel(nv)] - 자유관절 쿼터니언도 자동 처리됨."""
    data = mujoco.MjData(model)
    data.qpos[:] = qpos_eq
    data.qvel[:] = qvel_eq
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)

    nv = model.nv
    nu = model.nu
    A = np.zeros((2 * nv, 2 * nv))
    B = np.zeros((2 * nv, nu))
    mujoco.mjd_transitionFD(model, data, 1e-6, True, A, B, None, None)
    return A, B


def solve_dare(A, B, Q, R, n_iter=2000, tol=1e-10):
    """discrete-time algebraic Riccati 방정식을 반복법으로 직접 푼다
    (scipy.linalg.solve_discrete_are 없이 numpy만으로)."""
    P = Q.copy()
    for _ in range(n_iter):
        BtP = B.T @ P
        K = np.linalg.solve(R + BtP @ B, BtP @ A)
        P_new = Q + A.T @ P @ A - A.T @ P @ B @ K
        if np.max(np.abs(P_new - P)) < tol:
            P = P_new
            break
        P = P_new
    BtP = B.T @ P
    K = np.linalg.solve(R + BtP @ B, BtP @ A)
    return K


def build_cost(model):
    nv, nu = model.nv, model.nu
    Q = np.eye(2 * nv) * 0.01
    Q[0:3, 0:3] = np.eye(3) * Q_POS_WEIGHT
    Q[2, 2] = Q_HEIGHT_WEIGHT
    Q[3:6, 3:6] = np.eye(3) * Q_TILT_WEIGHT
    Q[nv:nv + 3, nv:nv + 3] = np.eye(3) * Q_LINVEL_WEIGHT
    Q[nv + 3:nv + 6, nv + 3:nv + 6] = np.eye(3) * Q_ANGVEL_WEIGHT
    R = np.eye(nu) * R_WEIGHT
    for i in range(nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if name is not None and "shoulder" in name:
            R[i, i] = R_WEIGHT * SHOULDER_R_MULTIPLIER
    return Q, R


class HumanoidMujocoWalk(Node):
    def __init__(self):
        super().__init__("humanoid_mujoco_walk")

        self.declare_parameter("mjcf_path", default_mjcf_path())
        mjcf_path = self.get_parameter("mjcf_path").get_parameter_value().string_value

        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)

        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")

        self.get_logger().info(f"MuJoCo 모델 로드: {mjcf_path}")
        self.get_logger().info("LQR 게인 계산 중 (평형점 선형화 + Riccati)...")

        self.qpos_eq, self.qvel_eq = find_equilibrium(self.model)
        A, B = linearize(self.model, self.qpos_eq, self.qvel_eq)
        Q, R = build_cost(self.model)
        self.K = solve_dare(A, B, Q, R)
        self.ctrl_limits = np.full(self.model.nu, math.radians(CTRL_LIMIT_DEG))

        self.get_logger().info(
            "LQR 게인 계산 완료 - 독립 스크립트 검증으로는 300초 완전 안정"
            "(최대 기울기 0.06도). 걷기(전진)는 아직 없음 - 서있기만 안정화."
        )

        self.mjcf_joint_names = list(MJCF_TO_URDF_JOINT.keys())
        self.joint_qpos_adr = {
            name: self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)]
            for name in self.mjcf_joint_names
        }

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # view_humanoid_stand.py의 화살표 키 push를 ROS에서도 쓸 수
        # 있게(humanoid_push_keyboard.py가 여기 발행) - 콜백은
        # rclpy 기본 단일 스레드 spin에서 타이머 콜백과 번갈아
        # 실행되므로 self.data를 직접 건드려도 안전하다(MuJoCo 뷰어의
        # GLFW 콜백과 달리 별도 스레드가 아님).
        self.push_sub = self.create_subscription(Vector3, "push", self.on_push, 10)

        self.timer = self.create_timer(TIMER_PERIOD_SEC, self.on_timer)

    def on_push(self, msg):
        self.data.qvel[0] += msg.x
        self.data.qvel[1] += msg.y
        self.data.qvel[2] += msg.z
        self.get_logger().info(f"push: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}) m/s")

    def on_timer(self):
        nv = self.model.nv
        # LQR 게인(K)은 model.opt.timestep(0.005s) 간격으로 매 스텝
        # 제어를 다시 계산한다는 전제로 풀었다 - 예전엔 제어를 한 번만
        # 계산하고 PHYSICS_SUBSTEPS(2)번 그대로 유지했는데, 그러면
        # 실질 제어 주기가 절반으로 느려져서(0.01s) 불안정해진다
        # (2026-11-17, RViz2에서 3초 만에 넘어지는 걸로 발견 -
        # 독립 검증 스크립트는 전부 매 스텝 재계산이라 못 잡았었음).
        # 그래서 매 substep마다 x/u를 다시 계산한다.
        for _ in range(PHYSICS_SUBSTEPS):
            dq = np.zeros(nv)
            mujoco.mj_differentiatePos(self.model, dq, 1.0, self.qpos_eq, self.data.qpos)
            dv = self.data.qvel - self.qvel_eq
            x = np.concatenate([dq, dv])

            u = -self.K @ x
            u = np.clip(u, -self.ctrl_limits, self.ctrl_limits)
            self.data.ctrl[:] = u

            mujoco.mj_step(self.model, self.data)

        now = self.get_clock().now()

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [MJCF_TO_URDF_JOINT[n] for n in self.mjcf_joint_names]
        js.position = [float(self.data.qpos[self.joint_qpos_adr[n]]) for n in self.mjcf_joint_names]
        self.joint_pub.publish(js)

        # ---- 몸통(골반="torso") 실제 물리 자세를 "world"->"base_link"
        # TF로 발행. quad_mujoco_walk.py의 body_pose_to_root()와 동일한
        # 방법 - torso는 base_link보다 HUMANOID_BODY_HEIGHT_OFFSET만큼
        # (URDF의 base_to_body fixed 관절) 위에 있으므로, 그 오프셋을
        # 몸통의 현재 방향으로 회전시켜 다시 빼서 base_link 자세를
        # 역산한다. ----
        body_pos = self.data.xpos[self.body_id]
        body_quat = self.data.xquat[self.body_id]
        offset_world = quat_rotate_vec(body_quat, np.array([0.0, 0.0, HUMANOID_BODY_HEIGHT_OFFSET]))
        base_pos = body_pos - offset_world

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "world"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = float(base_pos[0])
        tf.transform.translation.y = float(base_pos[1])
        tf.transform.translation.z = float(base_pos[2])
        tf.transform.rotation.w = float(body_quat[0])
        tf.transform.rotation.x = float(body_quat[1])
        tf.transform.rotation.y = float(body_quat[2])
        tf.transform.rotation.z = float(body_quat[3])
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = HumanoidMujocoWalk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
