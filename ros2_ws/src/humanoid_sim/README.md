# humanoid_sim

`D:\claude_test\computer_graphics`의 OpenGL 데모(`humanoid_test.c` /
`humanoid_loader_test.c`)가 하던 걷기/트롯 애니메이션을 ROS2
`joint_states` + TF로 재현해서 RViz2로 보기 위한 패키지. 모델 자체는
`gen_humanoid_assets.exe`가 생성하는 `humanoid.urdf` / `quadruped.urdf`
(+ `*_extra.yaml`)를 그대로 쓴다.

**Windows용 ROS2 Humble에서 `colcon build` → `ros2 launch
humanoid_sim display.launch.py`로 실제로 빌드/실행해서 RViz2에
humanoid와 4족 로봇이 둘 다 움직이는 것까지 확인했다.**

### 빌드 중 겪었던 문제: 한글 인코딩

처음 빌드했을 때 `colcon.colcon_core.package_augmentation ERROR
... 'cp949' codec can't decode byte ...`가 났다 - colcon의
`ros_ament_python` 패키지 분석 단계가 `setup.py`를 Windows
한국어 로캘의 기본 코드페이지(cp949)로 읽으려다, 그 안에 있던
UTF-8 한글 설명문을 디코딩하지 못해서 난 에러였다. 빌드는 그
단계를 건너뛰고도 일단 "Finished"로 끝났지만, package.xml 설치나
마커 파일 설치가 암묵적 fallback에 의존하게 돼서 찜찜했다 -
`setup.py`/`package.xml`의 설명문(description)만 영어로 바꿔서
해결했다 (코드 안의 한글 주석은 `.py` 파일을 UTF-8로 그냥 읽으므로
문제없다). 고친 뒤로는 그 에러 없이 깨끗하게 빌드된다.

빌드가 끝난 뒤 `WNDPROC return value cannot be converted to
LRESULT` / `TypeError: WPARAM is simple, so must be an int object
(got NoneType)`가 뜨는 것도 봤는데, 이건 colcon이 빌드 완료 후
**Windows 데스크톱 알림 팝업**을 띄우려다 나는 것으로, 빌드 결과와는
무관한 사소한 문제다 (이미 빌드가 성공적으로 끝난 뒤에 발생).

## 사전 준비

1. `D:\claude_test\computer_graphics`에서 `gen_humanoid_assets.exe`를
   한 번 실행해서 `humanoid.urdf` / `quadruped.urdf` /
   `*_ros.obj` / `humanoid_path.txt` / `quadruped_patrol.txt`가
   그 폴더에 있는지 확인한다 (`humanoid_path.txt`/
   `quadruped_patrol.txt`는 손으로 쓴 파일이라 이미 있을 것이고,
   나머지는 `gen_humanoid_assets.exe`가 만든다).
2. 이 패키지(`humanoid_sim`)를 ROS2 작업공간
   (`D:\claude_test\computer_graphics\ros2_ws`) 아래 `src/`에 둔
   상태 그대로, ROS2 Humble 셸(환경 설정 배치 파일을 소스한 셸)에서:

   ```powershell
   cd D:\claude_test\computer_graphics\ros2_ws
   colcon build --packages-select humanoid_sim
   # (Windows에서 colcon은 보통 call install\setup.bat 로 환경을 적용한다)
   call install\setup.bat
   ```

## 실행

```powershell
ros2 launch humanoid_sim display.launch.py
```

이 런치 파일이 하는 일:

- `robot_state_publisher`를 `/humanoid`, `/quadruped` 네임스페이스로
  각각 띄워서 `humanoid.urdf`/`quadruped.urdf`를 로딩한다 (조인트
  이름이 두 로봇 사이에서 안 섞이게 네임스페이스로 분리했다 - 자세한
  이유는 `launch/display.launch.py`의 docstring 참고).
- `humanoid_state_publisher`/`quadruped_state_publisher`(이 패키지의
  Python 노드)가 걷기/트롯 각도를 `/humanoid/joint_states` /
  `/quadruped/joint_states`로 발행하고, 몸통의 세계 속 위치를
  `world -> base_link` / `world -> quad_base_link` TF로 직접
  발행한다.
- `rviz2`를 띄운다 (저장된 `rviz/humanoid_sim.rviz`를 시도한다).
  실제로 띄워봐서 humanoid와 4족 로봇이 둘 다 RViz2 안에서 걷는
  것까지 확인했다. 혹시 디스플레이가 안 보이면 수동으로 Fixed
  Frame을 `world`로 두고 RobotModel 디스플레이 2개(Description
  Topic을 각각 `/humanoid/robot_description`,
  `/quadruped/robot_description`로)와 TF 디스플레이를 추가하면 된다.

프로젝트 폴더가 `D:\claude_test\computer_graphics`가 아닌 다른
곳이면:

```powershell
ros2 launch humanoid_sim display.launch.py assets_dir:=D:/다른/경로/computer_graphics
```

## 물리 데모: 공을 떨어뜨려 로봇과 충돌시키기 (로봇도 반응함)

`physics_demo.launch.py`는 `display.launch.py`와 똑같이 두 로봇을
띄우고, 거기에 더해 `mujoco_ball_sim` 노드로 MuJoCo 물리 엔진을
돌린다. 공 하나(실제 물리 바디, 자유낙하 + 중력)가 humanoid/4족
로봇의 몸통 충돌 프록시와 실제로 부딪힌다 - **그리고 맞으면 로봇도
반응한다.**

- 처음엔 프록시를 "mocap" 바디(물리 영향은 안 받고 자세만 외부
  TF로 그대로 주입받는)로 만들어서, 공은 튕기는데 로봇은 전혀
  반응이 없었다.
- 그래서 프록시를 질량이 있는 진짜 물리 바디로 바꾸고, 대신 매 틱
  "지금 걷고 있는/순찰 중인 목표 자세"(`humanoid_state_publisher`/
  `quadruped_state_publisher`가 발행하는 `"world"->"base_link_target"`/
  `"world"->"quad_base_link_target"` TF)를 향해 가상의 스프링-댐퍼
  힘/토크를 걸었다. 평소엔 그 목표를 그대로 따라가며 걷지만, 공에
  맞으면 스프링이 늘어나(목표에서 벗어나) 순간적으로 흔들리다가
  다시 걷기 자세로 돌아온다.
- `mujoco_ball_sim`이 그 물리 시뮬레이션 결과(흔들림 포함)를 실제
  `"world"->"base_link"`/`"world"->"quad_base_link"` TF로 발행해서,
  `robot_state_publisher`가 그걸로 전체 로봇 메쉬의 TF를 계산한다 -
  그래서 RViz2에 보이는 로봇 자체가 흔들린다. 두 state publisher는
  `publish_base_link` 파라미터를 `False`로 받아서(이 launch 파일이
  그렇게 띄움) 그 프레임을 직접 발행하지 않는다 - 안 그러면 두
  발행자가 같은 TF 프레임을 두고 충돌한다.

**3초(`RESPAWN_INTERVAL_SEC`)마다 둘 중 한쪽 바로 위에서 공을 다시
떨어뜨린다** - 처음엔 "바닥 아래로 떨어졌을 때만" 다시 떨어뜨리게
해놨는데, 빈 바닥에 착지해서 가만히 멈추면 그 조건을 다시는 만족
못 해 공이 영원히 멈춰버리는 버그가 있었다.

결과는 `/physics_markers`(공 + 반투명 충돌 프록시 2개 - 로봇이
흔들리는 동안 이 프록시도 같이 흔들린다)와 `world -> ball` /
`world -> base_link` / `world -> quad_base_link` TF로 RViz2에
보여준다. 자세한 설계는 `mujoco_ball_sim.py`와
`mujoco/ball_collision.xml`의 docstring/주석 참고.

**스프링 상수는 안 튜닝했다.** `SPRING_KP_POS`(위치 스프링 세기)는
질량으로 임계댐핑을 계산해서 "진동 없이 가장 빨리 돌아오는" 값을
자동으로 맞추지만, 회전 스프링(`SPRING_KP_ROT`/`SPRING_KD_ROT`)과
공/로봇 질량, "얼마나 세게 흔들렸으면 좋겠는지" 자체는 실제로 보면서
조절해야 하는 값이다 - 이 세션에서는 MuJoCo를 직접 돌려볼 수 없어서
숫자만 합리적으로 골랐고 튜닝은 못 했다.

**중요 - MuJoCo는 rosdep으로 안 받아진다.** `mujoco_ball_sim`은 pip
패키지 `mujoco`가 필요하다. 처음엔 conda 환경(`mujoco_ros_38`)에
설치하면 될 줄 알았는데, 이 ROS2 Humble(Windows) 설치는 colcon/ros2
전체가 **항상 `C:\Python38`을 쓰도록 고정**돼 있었다 -
`conda activate`를 해도 `colcon` 명령 자체가 그 conda 환경에
설치돼 있지 않으면(`colcon-common-extensions`를 거기 따로 깐 적이
없으면) PATH에서 결국 `C:\Python38`의 `colcon.exe`가 호출되기
때문이다. 그래서 `mujoco`를 그 Python에 직접 설치해야 했다:

```powershell
C:\Python38\python.exe -m pip install mujoco
```

(PyPI 인증서 문제 - `NO_CERTIFICATE_OR_CRL_FOUND` 류 SSL 에러가
나면, `--trusted-host pypi.org --trusted-host files.pythonhosted.org`
를 덧붙이면 우회된다. 이 환경에서 실제로 이 방법으로 설치됐다.)

```powershell
cd D:\claude_test\computer_graphics\ros2_ws
colcon build --packages-select humanoid_sim
call install\setup.bat
ros2 launch humanoid_sim physics_demo.launch.py
```

**검증 상태**: 공이 로봇과 충돌하고(이전 버전, mocap 방식) RViz2에
정상 표시되는 것까지는 실제로 빌드/실행해서 확인했다. 로봇이
질량을 갖고 스프링으로 반응하게 만든 이번 버전(이 섹션에서 설명한
전체)은 아직 이 세션에서 실제로 돌려보지 못했다 - 문법 검사
(`py_compile`), MJCF XML 유효성, 쿼터니언 곱/켤레 함수의 수치
검증만 했다.

## 원본 OpenGL 데모와 다른 점 (의도적으로 생략한 것)

- 키보드로 주인공을 수동 조작/정지시키는 기능(`MAIN_MODE_MANUAL`/
  `MAIN_MODE_STOPPED`)은 포팅하지 않았다 - 주인공이 늘 경로를 자동
  으로 걷는 것만 재현한다 (RViz는 보기용이라 키보드 조작까지
  옮기는 건 범위 밖으로 뒀다).
- 여러 캐릭터(클론)와 옷 색 틴트, 땅/하늘/그림자/나무 같은 배경
  소품은 로봇 모델이 아니라서 아예 포함하지 않았다. 자세한 내용은
  `computer_graphics/humanoid_extra.yaml` / `quadruped_extra.yaml`
  참고.
- `t`(애니메이션 시간)는 두 노드가 각자 독립적으로 흘린다 - 원본
  C 코드는 humanoid와 4족 로봇이 같은 `idle()`의 `t`를 공유했지만,
  두 로봇의 위상이 서로 맞든 안 맞든 기능적으로 의미가 없어서
  단순하게 분리했다.
