#!/usr/bin/env python3
"""view_quad_walk_ball_throw.py
------------------------------
view_quad_walk.py에 공 던지기 실험을 더한 버전 - "트릭(mocap+weld)
없는 진짜 물리 4족 로봇"(quad_walk.xml, quad_mujoco_walk.py와 완전히
같은 걸음걸이 공식)에 공을 던지면 어떻게 되는지 직접 눈으로 보려고
만들었다. ball_collision.xml 쪽 데모(mujoco_ball_sim.py)와 달리 여기는
mocap/weld로 "항상 제자리로 끌어당기는" 장치가 전혀 없다 - 다리
액추에이터가 정해진 걸음 패턴을 쫓아가는 것뿐이라, 약하게 맞으면
걸으면서 우연히 균형을 되찾지만 세게 맞으면 그냥 쓰러지고(쓰러진
채로 일어나는 동작이 없어서) 계속 쓰러진 채로 다리만 허우적댄다 -
ROS 없이 mj_step만 반복하는 독립 스크립트로 먼저 확인한 결과
(0.43kg/5m/s는 휘청하다 회복, 2kg/6m/s부터는 못 일어남)를 그대로
반영해서, THROW_PRESETS에 그 스펙트럼(약->강)을 순서대로 담아뒀다.

실행 (이 ROS2 Humble(Windows) 설치는 colcon/노드 실행을 C:\\Python38
에 고정해서 쓰므로, 이 스크립트도 같은 Python으로 돌리는 게
일관적이다 - mujoco 패키지가 깔린 아무 Python이면 된다):

  C:\\Python38\\python.exe view_quad_walk_ball_throw.py

창을 닫으면 종료된다. 걷기 방향 조작은 view_quad_walk.py와 동일하게
화살표 키(Up/Down/Left/Right)+Enter를 쓴다.

**공 투척은 `/` 키**(사용자 요청: "스페이스바를 누를 때마다 공이
투척되면 재밌겠다") - 원래 Space로 하려다가, Space는 MuJoCo 뷰어
자체의 일시정지 토글과 겹친다는 걸 알고 있어서(key_callback은
*추가로* 전달될 뿐 뷰어 자체 바인딩을 막지 못함 - 화살표 키를
쓰는 이유와 동일, view_quad_walk.py 주석 참고) 누를 때마다
재생/정지도 같이 토글돼버리는 문제를 피하려고, 글자/숫자가 아닌
`/`(GLFW_KEY_SLASH=47, 로컬 glfw 패키지로 직접 확인)로 바꿨다.
누를 때마다 THROW_PRESETS를 순서대로 하나씩 던진다(끝까지 가면
처음으로 순환).
"""
import math
import os
import time

import numpy as np
import mujoco
import mujoco.viewer

MJCF_PATH = os.path.join(os.path.dirname(__file__), "quad_walk.xml")
GAIT_FREQ_HZ = 0.85
SWING_AMPLITUDE_DEG = 32.0
KNEE_AMPLITUDE_DEG = 15.0

LEGS = ["FL", "FR", "BL", "BR"]
HIP_SIGN = {"FL": 1.0, "BR": 1.0, "FR": -1.0, "BL": -1.0}
KNEE_PHASE_OFFSET = {"FL": 0.0, "BR": 0.0, "FR": math.pi, "BL": math.pi}

CMD_MAX_SPEED_MPS = 0.16
CMD_MAX_TURN_RATE_RAD_S = 0.015
CMD_MAX_TURN_DIFF_DEG = 3.0
CMD_BASE_AMP_AT_ZERO_TURN_DEG = SWING_AMPLITUDE_DEG
CMD_BASE_AMP_AT_MAX_TURN_DEG = 25.0
LINEAR_STEP_MPS = CMD_MAX_SPEED_MPS / 8.0
ANGULAR_STEP_RAD_S = CMD_MAX_TURN_RATE_RAD_S / 8.0

# 독립 오프라인 스크립트(quad_walk_ball_throw.py)로 먼저 확인한
# 스펙트럼 그대로 - (질량kg, 속도m/s, 설명). 순서대로 돌아가며 던진다.
THROW_PRESETS = [
    (0.5, 2.0, "가벼운 공(0.5kg/2m/s) - 거의 안 느낌"),
    (0.43, 5.0, "축구공(0.43kg/5m/s) - 휘청하다 스스로 회복"),
    (2.0, 6.0, "무거운 공(2.0kg/6m/s) - 쓰러짐, 못 일어남"),
    (3.0, 10.0, "아주 세게(3.0kg/10m/s) - 더 빨리 쓰러짐"),
]
SPAWN_SIDE_DIST = 0.7  # 몸통 옆 이만큼에서 던짐(비행시간 짧게 유지)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def compute_leg_targets(t, cmd_linear_x, cmd_angular_z):
    """quad_mujoco_walk.py의 같은 이름 함수와 동일한 매핑 - 바꾸면 거기도 같이 바꿀 것."""
    going_backward = cmd_linear_x < 0.0
    speed_frac = clamp(abs(cmd_linear_x) / CMD_MAX_SPEED_MPS, 0.0, 1.0)

    if going_backward:
        turn_diff_deg = 0.0
        base_amp_deg = CMD_BASE_AMP_AT_ZERO_TURN_DEG * speed_frac
    else:
        turn_frac = clamp(cmd_angular_z / CMD_MAX_TURN_RATE_RAD_S, -1.0, 1.0)
        turn_diff_deg = turn_frac * CMD_MAX_TURN_DIFF_DEG
        amp_span = CMD_BASE_AMP_AT_ZERO_TURN_DEG - CMD_BASE_AMP_AT_MAX_TURN_DEG
        base_amp_deg = (CMD_BASE_AMP_AT_ZERO_TURN_DEG - amp_span * abs(turn_frac)) * speed_frac
    knee_amp_deg = KNEE_AMPLITUDE_DEG * speed_frac

    direction_sign = -1.0 if going_backward else 1.0
    knee_shift = (math.pi / 2.0) if going_backward else 0.0

    base_phase = 2.0 * math.pi * GAIT_FREQ_HZ * t
    hip_rad = {}
    knee_rad = {}
    for leg in LEGS:
        side_sign = 1.0 if leg in ("FR", "BR") else -1.0
        amp_deg = base_amp_deg + side_sign * turn_diff_deg
        hip_rad[leg] = direction_sign * HIP_SIGN[leg] * math.radians(amp_deg) * math.sin(base_phase)

        knee_phase = base_phase + KNEE_PHASE_OFFSET[leg] + knee_shift
        knee_rad[leg] = max(0.0, -math.cos(knee_phase)) * math.radians(knee_amp_deg)
    return hip_rad, knee_rad


def free_joint_adr(model, body_id):
    jnt_id = model.body_jntadr[body_id]
    return model.jnt_qposadr[jnt_id], model.jnt_dofadr[jnt_id]


BALL_BODY_XML = """
    <body name="throw_ball" pos="5 0 1">
      <freejoint/>
      <geom name="throw_ball_geom" type="sphere" size="0.15" mass="0.5" rgba="0.85 0.25 0.25 1"/>
    </body>
"""


def load_model_with_ball():
    """quad_walk.xml 파일 자체는 안 건드리고, 그 내용을 읽어서 공
    바디 하나를 </worldbody> 바로 앞에 메모리상에서 끼워 넣은 뒤
    문자열로 컴파일한다(MuJoCo는 컴파일된 모델에 바디를 런타임에
    추가할 수 없어서, 처음부터 XML에 포함시켜야 한다)."""
    with open(MJCF_PATH, "r", encoding="utf-8") as f:
        xml = f.read()
    xml = xml.replace("</worldbody>", BALL_BODY_XML + "  </worldbody>")
    return mujoco.MjModel.from_xml_string(xml)


def main():
    model = load_model_with_ball()
    data = mujoco.MjData(model)

    hip_act = {leg: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{leg}")
               for leg in LEGS}
    knee_act = {leg: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_knee_{leg}")
                for leg in LEGS}

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quad_body")
    ball_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "throw_ball")
    ball_qpos_adr, ball_qvel_adr = free_joint_adr(model, ball_body_id)
    # 공을 장면 밖(5,0,1)에서 시작시켜서, 첫 투척 전까지 로봇과 무관하게
    # 멀리 떨어져 있게 한다(quad_walk_ball_throw.py 오프라인 검증과
    # 같은 이유).
    data.qpos[ball_qpos_adr:ball_qpos_adr + 3] = [5.0, 0.0, 1.0]
    mujoco.mj_forward(model, data)

    KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_ENTER = 265, 264, 263, 262, 257
    KEY_SLASH = 47  # 공 투척 - Space는 MuJoCo 뷰어 자체의 일시정지와
                     # 겹쳐서(위 모듈 docstring 참고) 대신 이 키를 쓴다.
    cmd = {"linear_x": 0.0, "angular_z": 0.0}
    throw_state = {"pending": False, "idx": 0}

    def key_callback(keycode):
        if keycode == KEY_UP:
            cmd["linear_x"] = clamp(cmd["linear_x"] + LINEAR_STEP_MPS, -CMD_MAX_SPEED_MPS, CMD_MAX_SPEED_MPS)
        elif keycode == KEY_DOWN:
            cmd["linear_x"] = clamp(cmd["linear_x"] - LINEAR_STEP_MPS, -CMD_MAX_SPEED_MPS, CMD_MAX_SPEED_MPS)
        elif keycode == KEY_LEFT:
            cmd["angular_z"] = clamp(cmd["angular_z"] + ANGULAR_STEP_RAD_S,
                                      -CMD_MAX_TURN_RATE_RAD_S, CMD_MAX_TURN_RATE_RAD_S)
        elif keycode == KEY_RIGHT:
            cmd["angular_z"] = clamp(cmd["angular_z"] - ANGULAR_STEP_RAD_S,
                                      -CMD_MAX_TURN_RATE_RAD_S, CMD_MAX_TURN_RATE_RAD_S)
        elif keycode == KEY_ENTER:
            cmd["linear_x"] = 0.0
            cmd["angular_z"] = 0.0
        elif keycode == KEY_SLASH:
            throw_state["pending"] = True
            return
        else:
            return
        print(f"cmd_vel: linear_x={cmd['linear_x']:.3f} m/s  angular_z={cmd['angular_z']:.4f} rad/s")

    print("Up/Down/Left/Right/Enter: 걷기 방향 조작(view_quad_walk.py와 동일)")
    print("/ (슬래시): 공 투척 - 누를 때마다 아래 순서대로 하나씩(끝까지 가면 처음으로 순환):")
    for mass, speed, desc in THROW_PRESETS:
        print(f"  - {desc}")

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running():
            hip_rad, knee_rad = compute_leg_targets(data.time, cmd["linear_x"], cmd["angular_z"])
            for leg in LEGS:
                data.ctrl[hip_act[leg]] = hip_rad[leg]
                data.ctrl[knee_act[leg]] = knee_rad[leg]

            if throw_state["pending"]:
                throw_state["pending"] = False
                mass, speed, desc = THROW_PRESETS[throw_state["idx"] % len(THROW_PRESETS)]
                throw_state["idx"] += 1
                model.body_mass[ball_body_id] = mass
                body_pos = data.xpos[body_id].copy()
                spawn = body_pos + np.array([0.3, -SPAWN_SIDE_DIST, 0.0])
                flight_t = SPAWN_SIDE_DIST / max(speed, 0.1)
                vz_compensate = 0.5 * 9.81 * flight_t
                data.qpos[ball_qpos_adr:ball_qpos_adr + 3] = spawn
                data.qpos[ball_qpos_adr + 3:ball_qpos_adr + 7] = [1, 0, 0, 0]
                data.qvel[ball_qvel_adr:ball_qvel_adr + 3] = [0.0, speed, vz_compensate]
                data.qvel[ball_qvel_adr + 3:ball_qvel_adr + 6] = 0.0
                mujoco.mj_forward(model, data)
                print(f"[t={data.time:.1f}s] 투척: 질량={mass}kg 속도={speed}m/s - {desc}")

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - 0.0005))


if __name__ == "__main__":
    main()
