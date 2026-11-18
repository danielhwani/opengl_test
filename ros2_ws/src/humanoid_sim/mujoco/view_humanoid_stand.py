#!/usr/bin/env python3
"""view_humanoid_stand.py
--------------------------
MuJoCo 자체 뷰어(mujoco.viewer)로 humanoid_walk.xml의 LQR 서있기
안정화를 ROS2/RViz2와 전혀 무관하게 바로 보여준다 - view_quad_walk.py
와 같은 이유(디스커버리 문제 없이 빠르게 확인용).

**왜 만들었나**: RViz2에서 발이 땅에서 살짝 떠 보인다는 보고가
있었는데, 접촉력/순방향기구학을 전부 손으로/코드로 검산해도
"발이 거의 정확히 z≈0에 닿는다"는 결과만 나왔다 - URDF/
robot_state_publisher/RViz2 파이프라인을 다 거치지 않고, MuJoCo
물리 결과를 직접 눈으로 보고 "진짜로 발이 땅에 닿아있는지"를
확인하기 위한 가장 직접적인 방법이다.

humanoid_mujoco_walk.py와 똑같은 LQR 로직(mujoco.mjd_transitionFD로
평형점 선형화 -> discrete Riccati를 numpy 반복법으로 직접 풀기,
scipy 불필요)을 그대로 복제했다 - ROS 패키지 import 경로 밖이라
공유 모듈로 뽑기엔 과함(view_quad_walk.py의 같은 주석 참고).
**둘 중 하나를 고치면 반드시 둘 다 고칠 것.**

실행:
  python view_humanoid_stand.py

창을 닫으면 종료된다. cmd_vel/걷기 없음 - 순수 "서있기" 안정화만
본다(README의 "좌우/전후 밸런스 메커니즘 추가 + LQR로 '서있기'
안정화" 섹션 참고).
"""
import math
import os
import time

import numpy as np
import mujoco
import mujoco.viewer

MJCF_PATH = os.path.join(os.path.dirname(__file__), "humanoid_walk.xml")

CTRL_LIMIT_DEG = 30.0
Q_POS_WEIGHT = 5.0
Q_HEIGHT_WEIGHT = 2.0
Q_TILT_WEIGHT = 100.0
Q_LINVEL_WEIGHT = 0.5
Q_ANGVEL_WEIGHT = 10.0
R_WEIGHT = 0.2
# 2026-11-18: 어깨 ab/duction 추가 후 다른 관절과 같은 R이면 평형점의
# 작은 수치 오차만으로 거의 바로 포화돼서 넘어뜨렸다(humanoid_mujoco_walk.py
# 쪽과 동일 - 둘 중 하나 고치면 반드시 둘 다 고칠 것). 500배부터
# 300초 완전 안정.
SHOULDER_R_MULTIPLIER = 500.0
EQUILIBRIUM_SETTLE_SEC = 0.3


def find_equilibrium(model):
    data = mujoco.MjData(model)
    for _ in range(int(EQUILIBRIUM_SETTLE_SEC / model.opt.timestep)):
        mujoco.mj_step(model, data)
    return data.qpos.copy(), data.qvel.copy()


def linearize(model, qpos_eq, qvel_eq):
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


PUSH_SPEED_MPS = 0.8  # 화살표 키 한 번당 더해지는 순간 속도(수평 충격).


def main():
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)

    print("평형점 선형화 + Riccati 계산 중...")
    qpos_eq, qvel_eq = find_equilibrium(model)
    A, B = linearize(model, qpos_eq, qvel_eq)
    Q, R = build_cost(model)
    K = solve_dare(A, B, Q, R)
    ctrl_limits = np.full(model.nu, math.radians(CTRL_LIMIT_DEG))
    print(f"완료. 평형 높이(qpos[2])={qpos_eq[2]:.4f}")

    data = mujoco.MjData(model)

    # 화살표 키로 로봇을 툭 밀어서 LQR이 실제로 버티는 모습을 눈으로
    # 보려고 추가(2026-11-17, 사용자 요청 - "가만히 서있으니 아무것도
    # 안 하는 것 같다"). view_quad_walk.py와 같은 이유로 알파벳/Space
    # 키는 피함(MuJoCo 뷰어 자체 토글과 겹침) - glfw 키코드도 거기서
    # 확인한 값 재사용(UP=265, DOWN=264, LEFT=263, RIGHT=262).
    # 콜백은 별도 스레드에서 올 수 있어서 data를 직접 건드리지 않고
    # "미는 속도"만 누적해뒀다가 메인 루프에서 한 번에 반영한다.
    KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT = 265, 264, 263, 262
    pending_push = np.zeros(3)

    def key_callback(keycode):
        if keycode == KEY_UP:
            pending_push[0] += PUSH_SPEED_MPS
            print(f"앞으로 밀기(+X {PUSH_SPEED_MPS} m/s)")
        elif keycode == KEY_DOWN:
            pending_push[0] -= PUSH_SPEED_MPS
            print(f"뒤로 밀기(-X {PUSH_SPEED_MPS} m/s)")
        elif keycode == KEY_LEFT:
            pending_push[1] += PUSH_SPEED_MPS
            print(f"왼쪽으로 밀기(+Y {PUSH_SPEED_MPS} m/s)")
        elif keycode == KEY_RIGHT:
            pending_push[1] -= PUSH_SPEED_MPS
            print(f"오른쪽으로 밀기(-Y {PUSH_SPEED_MPS} m/s)")

    print("화살표 키: 로봇을 그 방향으로 툭 밀기(여러 번 누르면 세게 누적)")

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running():
            if np.any(pending_push):
                data.qvel[0:3] += pending_push
                pending_push[:] = 0.0

            nv = model.nv
            dq = np.zeros(nv)
            mujoco.mj_differentiatePos(model, dq, 1.0, qpos_eq, data.qpos)
            dv = data.qvel - qvel_eq
            x = np.concatenate([dq, dv])
            u = np.clip(-K @ x, -ctrl_limits, ctrl_limits)
            data.ctrl[:] = u

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - 0.0005))


if __name__ == "__main__":
    main()
