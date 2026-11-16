#!/usr/bin/env python3
"""view_quad_walk.py
-------------------
MuJoCo 자체 뷰어(mujoco.viewer)로 quad_walk.xml의 트롯 걸음걸이를
실시간으로 보여준다. ROS2/DDS와 전혀 무관한 순수 로컬 MuJoCo 창이라
(RViz2처럼) 디스커버리 문제를 겪을 일이 없다 - quad_mujoco_walk.py
노드/launch 파일을 빌드/실행하지 않고도 걸음걸이가 어떻게 생겼는지
빠르게 확인할 때 쓴다.

실행 (이 ROS2 Humble(Windows) 설치는 colcon/노드 실행을 C:\\Python38
에 고정해서 쓰므로, 이 스크립트도 같은 Python으로 돌리는 게
일관적이다 - 꼭 그래야만 하는 건 아니고 mujoco 패키지가 깔린
아무 Python이면 된다):

  C:\\Python38\\python.exe view_quad_walk.py

창을 닫으면 종료된다. 걸음 주파수/진폭/무릎 진폭은
quad_mujoco_walk.py와 똑같은 값(독립 MuJoCo 스크립트로 쓸어보며
찾은 값 - README의 "4족 로봇 실제 관절 구동 보행" 섹션 참고)을
쓴다. 바꿔서 직접 실험해보고 싶으면 아래 상수들을 고치면 된다.

**2026-11-16 업데이트: 화살표 키로 /cmd_vel 없이도 방향 조정 가능.**
이 창은 ROS 노드가 아니라서 실제 /cmd_vel 토픽을 구독할 수 없으니,
MuJoCo 뷰어 자체의 키보드 콜백으로 똑같은 역할(전진 속도/회전
속도 올리고 내리기)을 흉내낸다 - **Up/Down**=전진속도 증가/감소,
**Left/Right**=좌회전/우회전 속도 증가/감소, **Enter**=정지.
처음엔 W/S/A/D + Space로 만들었는데, 사용자가 "W 눌렀더니 색이
바뀐다"고 확인 - MuJoCo 뷰어 자체가 이미 알파벳 키 대부분을
렌더링 토글(W=와이어프레임 등 `mjvOption`/`mjvScene` 표시 플래그들,
Space=실행/일시정지)에 쓰고 있어서(`glfw` 콜백은 우리 콜백에
*추가로* 전달될 뿐 막아주지 않음), 그 키들과 겹쳐서 의도치 않게
렌더링이 토글됐던 것이었다. 화살표 키/Enter는 시뮬레이션이 멈춰있지
않은 동안은 MuJoCo 자체 바인딩이 없어서 안전하다(`glfw.KEY_UP`
등 정확한 코드는 로컬에 설치된 `glfw` 패키지에서 직접 확인함:
UP=265, DOWN=264, LEFT=263, RIGHT=262, ENTER=257). 매핑 공식(속도/
회전->보폭)은 quad_mujoco_walk.py의 compute_leg_targets()와 완전히
같은 상수/식을 여기 복제했다(ROS 패키지 import 경로 밖이라 공유
모듈로 뽑기엔 과함 - 둘 중 하나를 고치면 반드시 둘 다 고칠 것).
"""
import math
import os
import time

import mujoco
import mujoco.viewer

MJCF_PATH = os.path.join(os.path.dirname(__file__), "quad_walk.xml")
GAIT_FREQ_HZ = 0.85  # 1.1Hz는 짧게는 좋아 보여도 18~20초쯤 반드시
                      # 뒤집혔다(180초까지 쓸어서 확인) - 0.8~0.95Hz는
                      # 다 안정적인데 그중 옆으로 덜 쏠리는 0.85Hz를
                      # 골랐다. README의 "무릎 관절 추가" 섹션 참고.
SWING_AMPLITUDE_DEG = 32.0  # "너무 느리다"는 피드백으로 30->32도로
                             # 올림 - 300초 검증으로 옆쏠림이 거의
                             # 없는(1.6m) 값을 찾았다. README 참고.
KNEE_AMPLITUDE_DEG = 15.0  # 16도부터 급격히 불안정, 17도부터 넘어짐 - README 참고

LEGS = ["FL", "FR", "BL", "BR"]
HIP_SIGN = {"FL": 1.0, "BR": 1.0, "FR": -1.0, "BL": -1.0}
KNEE_PHASE_OFFSET = {"FL": 0.0, "BR": 0.0, "FR": math.pi, "BL": math.pi}

# ---- /cmd_vel 매핑 상수 - quad_mujoco_walk.py와 동일 (거기 주석에
# 검증 과정 전체 기록돼 있음). ----
CMD_MAX_SPEED_MPS = 0.16
CMD_MAX_TURN_RATE_RAD_S = 0.015
CMD_MAX_TURN_DIFF_DEG = 3.0
CMD_BASE_AMP_AT_ZERO_TURN_DEG = SWING_AMPLITUDE_DEG
CMD_BASE_AMP_AT_MAX_TURN_DEG = 25.0

# 키보드로 조금씩 올리고 내릴 때 한 번 누름당 증가/감소량.
LINEAR_STEP_MPS = CMD_MAX_SPEED_MPS / 8.0
ANGULAR_STEP_RAD_S = CMD_MAX_TURN_RATE_RAD_S / 8.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def compute_leg_targets(t, cmd_linear_x, cmd_angular_z):
    """quad_mujoco_walk.py의 같은 이름 함수와 동일한 매핑 - 그쪽 주석에
    설계 이유가 다 적혀 있다. 여기서 바꾸면 거기도 똑같이 바꿀 것."""
    speed_frac = clamp(cmd_linear_x / CMD_MAX_SPEED_MPS, 0.0, 1.0)
    turn_frac = clamp(cmd_angular_z / CMD_MAX_TURN_RATE_RAD_S, -1.0, 1.0)
    turn_diff_deg = turn_frac * CMD_MAX_TURN_DIFF_DEG

    amp_span = CMD_BASE_AMP_AT_ZERO_TURN_DEG - CMD_BASE_AMP_AT_MAX_TURN_DEG
    base_amp_deg = (CMD_BASE_AMP_AT_ZERO_TURN_DEG - amp_span * abs(turn_frac)) * speed_frac
    knee_amp_deg = KNEE_AMPLITUDE_DEG * speed_frac

    base_phase = 2.0 * math.pi * GAIT_FREQ_HZ * t
    hip_rad = {}
    knee_rad = {}
    for leg in LEGS:
        side_sign = 1.0 if leg in ("FR", "BR") else -1.0
        amp_deg = base_amp_deg + side_sign * turn_diff_deg
        hip_rad[leg] = HIP_SIGN[leg] * math.radians(amp_deg) * math.sin(base_phase)

        knee_phase = base_phase + KNEE_PHASE_OFFSET[leg]
        knee_rad[leg] = max(0.0, -math.cos(knee_phase)) * math.radians(knee_amp_deg)
    return hip_rad, knee_rad


def main():
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    data = mujoco.MjData(model)

    hip_act = {leg: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{leg}")
               for leg in LEGS}
    knee_act = {leg: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_knee_{leg}")
                for leg in LEGS}

    # GLFW 키코드 - 로컬 glfw 패키지에서 직접 확인한 값(W/A/S/D/Space는
    # MuJoCo 뷰어 자체의 렌더링 토글/실행-정지와 겹쳐서 피함. 위 모듈
    # docstring의 2026-11-16 업데이트 참고).
    KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_ENTER = 265, 264, 263, 262, 257

    cmd = {"linear_x": 0.0, "angular_z": 0.0}

    def key_callback(keycode):
        if keycode == KEY_UP:
            cmd["linear_x"] = clamp(cmd["linear_x"] + LINEAR_STEP_MPS, 0.0, CMD_MAX_SPEED_MPS)
        elif keycode == KEY_DOWN:
            cmd["linear_x"] = clamp(cmd["linear_x"] - LINEAR_STEP_MPS, 0.0, CMD_MAX_SPEED_MPS)
        elif keycode == KEY_LEFT:
            cmd["angular_z"] = clamp(cmd["angular_z"] + ANGULAR_STEP_RAD_S,
                                      -CMD_MAX_TURN_RATE_RAD_S, CMD_MAX_TURN_RATE_RAD_S)
        elif keycode == KEY_RIGHT:
            cmd["angular_z"] = clamp(cmd["angular_z"] - ANGULAR_STEP_RAD_S,
                                      -CMD_MAX_TURN_RATE_RAD_S, CMD_MAX_TURN_RATE_RAD_S)
        elif keycode == KEY_ENTER:
            cmd["linear_x"] = 0.0
            cmd["angular_z"] = 0.0
        else:
            return
        print(f"cmd_vel: linear_x={cmd['linear_x']:.3f} m/s  angular_z={cmd['angular_z']:.4f} rad/s")

    print("Up/Down: 전진 속도 증가/감소, Left/Right: 좌/우 회전 증가/감소, Enter: 정지")

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running():
            hip_rad, knee_rad = compute_leg_targets(data.time, cmd["linear_x"], cmd["angular_z"])
            for leg in LEGS:
                data.ctrl[hip_act[leg]] = hip_rad[leg]
                data.ctrl[knee_act[leg]] = knee_rad[leg]

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - 0.0005))


if __name__ == "__main__":
    main()
