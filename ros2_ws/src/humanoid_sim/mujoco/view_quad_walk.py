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


def main():
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    data = mujoco.MjData(model)

    hip_act = {leg: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{leg}")
               for leg in LEGS}
    knee_act = {leg: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_knee_{leg}")
                for leg in LEGS}

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            base_phase = 2.0 * math.pi * GAIT_FREQ_HZ * data.time
            for leg in LEGS:
                hip_rad = HIP_SIGN[leg] * math.radians(SWING_AMPLITUDE_DEG) * math.sin(base_phase)
                data.ctrl[hip_act[leg]] = hip_rad
                knee_phase = base_phase + KNEE_PHASE_OFFSET[leg]
                knee_rad = max(0.0, -math.cos(knee_phase)) * math.radians(KNEE_AMPLITUDE_DEG)
                data.ctrl[knee_act[leg]] = knee_rad

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - 0.0005))


if __name__ == "__main__":
    main()
