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

**실제로 돌려보니, 공에 맞아 쓰러진 뒤로 다시 못 일어나고 바닥에
쓰러진 채로 걷기 목표 쪽으로 계속 끌려가는(기어가는) 문제가
있었다.** 회전 스프링(`SPRING_KP_ROT`, 처음엔 150)이 쓰러진 자세의
중력+바닥 접촉 마찰을 이기고 다시 세울 만큼 세지 않아서였다 -
위치 스프링(강함)만 이기고 로봇을 계속 끌고 다닌 것. 쓰러진 뒤
스스로 균형을 잡고 일어서는 건 실제 다리로 버티는 제어가 필요한
훨씬 어려운 문제라, 그 상황 자체를 피하는 쪽으로 고쳤다 -
`SPRING_KP_ROT`을 4000까지 크게 올려서(그리고 감쇠도 `SPRING_KP_POS`
처럼 관성 기준 임계댐핑으로 자동 계산하도록 바꿔서), 웬만한 충격
으로는 애초에 안 쓰러지고 "잠깐 휘청였다가 바로 돌아오는" 정도로만
보이게 했다.

**그래도 스프링 상수는 완전히 튜닝된 게 아니다.** `SPRING_KP_POS`/
`SPRING_KP_ROT`의 감쇠(kd)는 각각 질량/관성으로 임계댐핑을 계산해서
자동으로 맞추지만, 비례 계수(kp) 자체와 공/로봇 질량, "얼마나 세게
흔들렸으면 좋겠는지"는 실제로 보면서 조절해야 하는 값이다.

**박스 프록시가 로봇 발밑(땅)에 생기는 버그도 있었다.** 스프링
목표로 쓰는 `base_link_target`/`quad_base_link_target`은 발 높이
(z=0) 기준인데, 충돌 박스는 몸통(torso) 크기라 그 사이 높이
차이(URDF의 `base_to_body`류 fixed 관절 오프셋, humanoid 1.9m /
quad 0.75m)를 안 더해줘서 생겼다 - `HUMANOID_BODY_HEIGHT_OFFSET`/
`QUAD_BODY_HEIGHT_OFFSET`을 목표 위치에 더하고, 반대로
"world"->"base_link"를 발행할 때는(robot_state_publisher가 그
오프셋을 URDF에서 또 적용하므로) 그 오프셋을 몸통의 현재 방향으로
회전시켜 다시 빼주도록 고쳤다(`body_pose_to_root()`).

**회전 스프링을 세게 올리자(쓰러짐 방지), 이번엔 로봇이 통째로
날아가 버리는 더 심한 버그가 생겼다.** ROS2/RViz2 GUI 없이도
`ros2 run`으로 노드를 직접 띄우고 `ros2 run tf2_ros tf2_echo`로 TF
값을 숫자로 찍어보면서(+ MuJoCo를 독립적으로 돌리는 진단 스크립트)
원인을 찾았다:
- quad_proxy가 방향을 틀기 시작하는 순간부터 각속도(`|w|`)가 몇
  틱 안에 기하급수적으로 폭주했다(0 -> 27 -> 180 -> 103920 ...).
  길쭉한 상자라 축마다 관성이 꽤 다른데(intermediate axis 문제),
  댐핑을 평균 관성 하나로만 계산하는 선형 PD 제어가 그 자이로스코픽
  결합을 못 잡아낸 것으로 보인다.
- `MAX_FORCE`/`MAX_TORQUE`로 우리가 거는 힘/토크를 제한해도 안
  잡혔다 - 그 결과로 생긴 빠른 회전이 MuJoCo의 접촉(contact) 솔버
  에서 "터널링"(한 스텝 사이 과도한 침투 -> 비현실적으로 큰 충돌
  임펄스)을 일으켜, 우리가 안 건 속도까지 치솟았다(최대 9백만
  rad/s 관측).
- 결국 물리 스텝 "이후"(매 substep마다)에 선속도/각속도 자체를
  `MAX_LIN_VEL`/`MAX_ANG_VEL`(15 m/s, 15 rad/s)로 직접 눌러주는
  방식으로 해결했다 - 걷거나 순찰하는 로봇이 이보다 빨리 움직일
  이유가 없으니 안전한 상한이다. 25초짜리 헤드리스 테스트로 더 이상
  폭주하지 않음을 확인했다(위치가 한 번도 합리적 범위를 벗어나지
  않았다).
- 다만 완전히 매끄럽진 않다 - 클램프가 걸리는 동안은 로봇이 다소
  휘청인다(RPY로 수십 도 정도 기울었다가 가라앉는 모습을 TF로
  확인함). RViz2로 실제로 보면서 더 다듬어야 한다.

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

## 반응형 물리 튜닝 (진행 중)

위 "날아가는 버그"를 고친 뒤에도, 실제로 공을 맞혀보면서 계속
문제가 나왔고 하나씩 고쳤다 - 순서대로:

1. **공이 로봇 발밑(땅)에 박스로 나타남** - 이미 위에 적은
   `HUMANOID_BODY_HEIGHT_OFFSET`/`body_pose_to_root()` 버그.
2. **공을 맞아도 거의 반응이 없거나(과댐핑) 반대로 계속 떨림
   (언더댐핑)을 왔다갔다** - `MAX_ANG_VEL`을 15→6으로 낮추고, 각속도
   제곱에 비례하는 추가 감쇠(`ROT_QUAD_DRAG`, 공기저항 느낌 - 빠르게
   돌 때만 세게 제동)를 추가했다. 그래도 damping multiplier 하나로
   둘 다 만족시키기 어려워서 계속 왔다갔다했다.
3. **4족 로봇이 공과 무관하게 계속 흔들림** - 알고 보니 patrol
   방향 전환(최대 ±90도)이 원래 꽤 급한데, 회전 스프링이 그 "정상
   애니메이션 목표"조차 못 따라가서 생긴 지연(lag)-흔들림이었다
   (공 충돌 반응 문제가 아니었음).
4. **결국 humanoid/quad의 회전 스프링을 완전히 분리**했다 -
   `HUMANOID_KP_ROT=800`(경로가 완만해서 안 뻣뻣해도 됨) +
   `HUMANOID_ROT_DAMPING_MULTIPLIER=1.8`, `QUAD_KP_ROT=1500`(급선회를
   따라가려면 더 뻣뻣해야 함) + `QUAD_ROT_DAMPING_MULTIPLIER=1.3`.
5. **공이 둘 중 아무 로봇에나 무작위(50:50)로 떨어져서, 운이 나쁘면
   한쪽이 한동안 안 맞아 "반응이 없어졌다"는 느낌을 줄 수 있었다** -
   `respawn_ball()`을 번갈아(humanoid, quad, humanoid, ...) 가게
   바꿨다(`next_respawn_is_humanoid`).
6. **target_vel 계산에서 에일리어싱 버그 발견 및 수정** -
   `humanoid_state_publisher`/`quadruped_state_publisher`는 50Hz로
   TF를 발행하는데 `mujoco_ball_sim`은 100Hz로 읽어서, 고정된 틱
   주기(`TIMER_PERIOD_SEC`)로 나누면 한 틱은 "값이 안 변함"(속도
   0으로 계산됨), 다음 틱은 그 두 배로 튀는 문제가 있었다 -
   `lookup_target()`이 TF 메시지 자체의 타임스탬프를 같이 돌려주고,
   `update_proxy()`가 그 실제 시간 차이로 속도를 계산하도록 고쳤다
   (같은 타임스탬프를 또 읽은 틱은 이전 속도를 그대로 재사용).

**아직 안 풀린 문제 (업데이트, 다음 세션)**: 6번을 고친 뒤에도
"휴머노이드가 공을 맞기 전부터(처음부터) 계속 흔들린다"는 보고가
계속됐다. `ros2 bag record -o <name> /tf /tf_static`로 실제 TF를
파일로 녹화해서(네트워크/DDS에 전혀 안 끼어들고 `rclpy.serialization.
deserialize_message`로 sqlite3 `.db3` 파일을 오프라인으로 직접
파싱 - 이 방법이 이 환경에서 가장 안전하게 데이터를 확인하는 방법
이었다) 분석해보니:

- **회전(yaw)과 높이(z)는 정상**이다 - 쓰러지거나 팽이처럼 돌지
  않는다.
- **수평 위치(x,y)가 첫 공 충돌 시점부터 녹화가 끝날 때까지
  (10초 이상) 초속 7~19m/s급으로 계속 진동하며 거의 안 가라앉는다.**
  공이 로봇에서 8m 이상 떨어져 있을 때도 똑같이 계속돼서, 공이
  계속 때려서 그런 게 아니라 **스프링 피드백 자체가 자기유지하는
  리밋 사이클(limit cycle)**로 보인다.
- 이번 세션 내내 **회전** 스프링만 과댐핑/쿼드래틱 드래그로 손봤고
  **위치** 스프링(`SPRING_KP_POS`)은 맨 처음 값(순수 임계댐핑)
  그대로였다 - 위치에도 똑같이 과댐핑(`POS_DAMPING_MULTIPLIER=1.8`)
  + 속도 제곱 감쇠(`POS_QUAD_DRAG=20`)를 추가하고, 그 감쇠력이
  깎이지 않도록 `MAX_FORCE`도 5000→20000으로 올렸다 - **그런데도
  거의 효과가 없었다** (여전히 같은 패턴).
- 마지막으로, 근본 원인을 더 파기보다 "속도가 `MAX_LIN_VEL`/
  `MAX_ANG_VEL`의 90% 이상으로 0.5초 넘게 계속 붙어있으면 그냥
  목표 자세로 강제 스냅시키는" 안전장치(`check_stuck_and_recover()`)
  를 추가했다 - 처음 3초(공 맞기 전)는 확실히 조용해졌지만, 맞은
  뒤로는 속도가 7~19m/s 사이를 오르내리며 **여전히 깨끗하게
  원래 상태로 안 돌아온다.**

세 번의 서로 다른 처방(위치 과댐핑, 힘 제한 상향, 강제 스냅
안전장치)이 다 부분적으로만 효과가 있어서, **"가상 스프링을
`xfrc_applied`로 직접 미는" 지금 설계 자체의 구조적 한계**일 가능성이
있다고 보고 이번 세션은 여기서 마무리했다. 다음에 이어서 볼
방향: (a) MuJoCo 네이티브 조인트 기반 액추에이터/`equality`
제약으로 바꿔서 암묵적(implicit) 적분의 이득을 위치에도 받게
하거나, (b) 왜 댐핑을 올려도 안 죽는지(에너지가 정확히 어디서
재생산되는지) 더 깊이 분석하거나, (c) 지금 수준에서 받아들이고
다른 작업으로 넘어가는 것 중 선택.

## 4족 로봇 실제 관절 구동 보행 (`quad_physics_walk.launch.py`)

위 두 데모(`display.launch.py`, `physics_demo.launch.py`)는 로봇의
몸통 위치를 외부에서 정해주는 방식(애니메이션 경로, 또는 그 경로를
향하는 가상 스프링)이었다 - 팔다리/머리가 물리적으로 존재하지
않아서 공이 몸을 그냥 통과했다. 이 launch는 그것과 다르다: 4족
로봇의 몸통+머리+다리 4개를 **실제 충돌 가능한 물리 모델**로 만들고,
다리 관절을 MuJoCo 액추에이터로 직접 구동해서 **몸통이 다리의
물리적 반작용만으로 걸어가게** 한다(휴머노이드는 더 복잡해서 4족
로봇부터 시작).

```powershell
cd D:\claude_test\computer_graphics\ros2_ws
colcon build --packages-select humanoid_sim
call install\setup.bat
ros2 launch humanoid_sim quad_physics_walk.launch.py
```

- **`mujoco/quad_walk.xml`** — 이번에도 `ball_collision.xml`처럼
  손으로 직접 쓴 MJCF다(`gen_humanoid_assets.c`가 생성하는 게
  아님). 치수는 그 C 파일의 `QUAD_*` 상수(줄 60-77)에서 그대로
  가져왔다 - 몸통(박스), 머리(박스, 장식), 다리 4개(캡슐 - 박스보다
  바닥 접촉이 안정적이라 흔한 MuJoCo 레그드 로봇 관행을 따름).
  관절 이름(`quadbody_to_QuadLegFL` 등)은 `quadruped.urdf`와 똑같이
  맞췄다.
- 다리 관절은 MuJoCo 네이티브 `<position>` 액추에이터(PD 위치
  서보)로 구동한다 - `mujoco_ball_sim.py`가 `xfrc_applied`로 가상
  힘을 직접 밀어넣던 방식과 다르다(그 방식은 지난 세션 내내
  리밋 사이클 등으로 계속 불안정했다 - 네이티브 액추에이터는
  `integrator="implicitfast"`의 암묵적 적분 이득을 받아서 더
  안정적일 것으로 기대했고, 실제로 검증해보니 그랬다).
- **`quad_mujoco_walk.py`**는 `quadruped_state_publisher.py`와
  똑같은 트롯 걸음걸이 공식(`30*sin(2t)`, 대각선 다리 쌍 반대
  위상)을 액추에이터 목표각으로 넣어줄 뿐이다 - 몸통이 실제로
  앞으로 나가는지는 순전히 물리(다리가 바닥을 밀어내는 반작용 +
  마찰)로 결정된다. 시각적 메쉬는 기존 `quadruped.urdf`를 그대로
  쓴다(관절 이름이 같아서 `joint_states`만 맞게 발행하면 그대로
  그려짐).
- **ROS 없이 순수 MuJoCo로 먼저 검증했다**(`mj_step`만 반복하는
  독립 스크립트 - 이 환경은 ROS2 노드를 직접 띄우면 위험하지만
  ROS를 아예 안 쓰는 MuJoCo 스크립트는 안전하다): kp=80→150으로
  올리니 전진이 조금 나아졌고, 트롯 진폭을 45도로 올려봤더니
  오히려 더 주저앉고 느려져서(다리가 수직에서 벗어난 시간이
  길어져 평균 지지 높이가 낮아짐) 30도로 되돌렸다. **10초 동안
  쓰러지지 않고(최대 기울기 1도 미만) 약 0.2m 순전진했다** - 느리고
  완벽하지 않지만(처음 높이 0.75m에서 0.53~0.6m로 주저앉는다 -
  무릎 없는 1자유도 다리라 자연스러운 현상), 핵심("실제로 관절을
  움직여 걷는다")은 확인됐다. 더 빠르게/안정적으로 걷게 하려면
  다리에 무릎 자유도를 추가하는 쪽이 가장 효과적일 것 같다(다음에
  시도해볼 만함).
- **아직 실제 RViz2로는 확인 못 했다** - 독립 MuJoCo 스크립트로만
  검증했고, ROS2 노드로 빌드까지는 했지만 사용자가 직접 띄워서
  봐야 한다.

### 업데이트: 무릎 관절 추가 (결과: 약 6배 빨라짐)

사용자가 `view_quad_walk.py`(아래)로 직접 보고 "미끄러워서 안
나가는 것 같다"고 확인 - 마찰 계수를 0.5~5.0까지 쓸어봤지만
원인이 아니었다(기본값 1.0이 이미 최적, 더 끈적하게 하면 오히려
불안정/넘어짐). 진짜 원인은 **무릎이 없어서 스윙 중에 발을 제대로
못 드는 것**이었다.

다리를 허벅지(`quadbody_to_QuadLegFL` 등, 기존과 동일한 엉덩이
관절)+종아리(`QuadLegFL_to_QuadShinFL` 등, 새 무릎 관절, 한쪽으로만
0~90도) 2 세그먼트로 나눴다(`mujoco/quad_walk.xml`). 길이는 각각
0.275m(합치면 기존 다리 길이 0.55m와 같아서 서있는 높이는 그대로).
무릎을 "엉덩이가 스윙하는 동안 굽히고 스탠스일 때 펴지는" 신호
(`max(0, -cos(phase))`)로 움직이는데, 독립 MuJoCo 스크립트로 무릎
진폭을 5~60도까지 쓸어서 찾았다:

- **15도**: 10초에 **5.26m** 순전진(무릎 없을 때의 약 6배), 최대
  기울기 7도 이내 - 채택한 값.
- 16도부터 급격히 불안정(19도대 기울기), 17도부터 넘어짐 - 15도는
  한계 바로 아래의 안전한 값이다.

이 결과는 `gen_humanoid_assets.c`/`humanoid_test.c`/
`humanoid_loader_test.c`(원본 OpenGL 데모)에도 그대로 반영했다 -
URDF/OBJ/scene.txt에 무릎 링크/관절을 추가하고, 두 OpenGL 데모의
`idle()`에도 같은 공식을 넣었다(자세한 내용은 프로젝트 루트
`README.md`의 "4족 보행 로봇" 섹션 참고). 다만 OpenGL 쪽은 물리가
아니라 순수 애니메이션이라 안정성 제약이 없어서, 진폭을 45도로
더 크게 키웠다(15도로는 "다리가 아직 뻣뻣하다"는 피드백을 받을
만큼 거의 안 보였다).

**`mujoco/view_quad_walk.py`**: `quad_mujoco_walk.py` 노드/launch
파일을 안 띄우고도(ROS2/DDS와 전혀 무관) MuJoCo 자체 뷰어로
걸음걸이를 빠르게 확인할 수 있는 독립 스크립트. 재부팅 걱정 없이
자유롭게 돌려볼 수 있어서, 이번 무릎 튜닝 전체를 이 스크립트 +
ROS 없는 `mj_step` 반복 스크립트만으로 검증했다.

```powershell
cd D:\claude_test\computer_graphics\ros2_ws\src\humanoid_sim\mujoco
C:\Python38\python.exe view_quad_walk.py
```

### 추가 업데이트: 10초 검증의 함정 - 장시간 드리프트로 결국 뒤집힘

사용자가 `view_quad_walk.py`로 1분 넘게 지켜보더니 "가장자리에 가서
뒤집어지네?"라고 확인 - 위에서 찾은 1.1Hz를 180초까지 돌려보니,
**18~20초쯤 롤(좌우 기울기)이 서서히 커지다가 반드시 뒤집혀서
옆으로 누운 채 멈췄다.** 한 번의 사고가 아니라 **누적되는 드리프트**
였다(0~18초: 기울기가 4.5도→9.9도로 서서히 증가, 18~20초: 급격히
108~117도로 뒤집힘, 그 뒤: 90도로 누운 채 안정). 10초짜리 검증
으로는 절대 안 보이는 문제였다.

0.5Hz 간격으로 0.8~1.2Hz를 180초까지 다시 쓸어봤다:

| 주파수 | 180초 결과 |
|---|---|
| 0.8~0.95Hz | 전부 안정(기울기 5~6도대), 안 넘어짐 |
| 1.0Hz | 안정은 한데 60초 만에 옆으로 8.5m나 쏠림 |
| 1.1Hz | **18~20초쯤 반드시 뒤집힘** |
| 1.2Hz | 더 빨리 넘어짐 |

0.8~0.95Hz 중에서는 **0.85Hz**가 옆으로 쏠리는 양(lateral drift)이
가장 작아서(180초에 1.8m, 0.95Hz는 120초에 17.8m나 쏠림) 가장
똑바로 걷는다 - 최종적으로 `GAIT_FREQ_HZ`를 1.1 → 0.85로 바꿨다.

**교훈**: 걸음걸이/제어 게인을 검증할 땐 짧은 구간(10초)만 보고
"됐다"고 판단하면 안 된다 - 리밋 사이클이나 서서히 쌓이는 드리프트
같은 문제는 보통 한참 지나야 드러난다. 최소 1~2분은 돌려봐야
안다. (조명도 이때 같이 손봤다 - `<headlight>`로 전역 밝기를
더하고 `light`를 `directional`로 바꿔서, 로봇이 원래 조명의
원뿔/도달 범위를 벗어나도 가장자리가 어두워지지 않게 했다. 그 뒤
"전역 조명이 너무 밝아서 스폿라이트가 안 보인다"는 피드백으로
`headlight`/`light`의 ambient/diffuse를 다시 낮췄다.)

### 추가 업데이트: 속도 개선 (보폭 32도)

0.85Hz로는 "너무 느려서 답답하다"는 피드백을 받았다. 주파수를
다시 올리면 장시간 불안정(위 섹션)이 재발할 수 있어서, 대신 **보폭
(엉덩이 스윙 진폭)**을 0.85Hz로 고정한 채 150~300초씩 쓸어서
찾았다:

- 30→35도까지는 빨라지다가(35도: 150초에 29.8m), 35도부터 롤
  최대값이 눈에 띄게 커지기 시작(7.7도) - 안전 마진을 더 보려고
  32~37도 구간을 240초로 다시 쓸어봤다.
- 32~37도는 전부 안 넘어지긴 했지만(최대 롤 5.9~9.8도), **옆으로
  쏠리는 양이 값마다 들쭉날쭉했다**(32도: 0.7m, 34도: 18.9m, 36도:
  27.6m - 매끄러운 경향이 아니라 민감한/카오스적인 구간으로
  보인다).
- **32도**가 전진(240초에 38.6m)도 빠르고 옆쏠림도 제일 적어서
  (0.7m) 채택했다. 300초(5분)까지 다시 확인해도 48m 전진, 옆쏠림
  1.6m, 최대 롤 5.9도로 안정적이었다.

`SWING_AMPLITUDE_DEG`를 30 → 32로 바꿨다. 체감 속도가 확실히
빨라졌을 것이다(기존 대비 300초 기준 약 20~25% 더 빠름).

## 중요 - Claude가 직접 노드를 띄워 테스트하는 것의 위험성

이 세션에서 디버깅용으로 `ros2 run`/`ros2 launch`를 수십 번
직접 실행하고 강제종료(Force-Kill)하면서, **이 Windows 머신의
ROS2 CLI 데몬 / Fast-DDS 공유메모리 디스커버리 상태가 두 번이나
망가졌다** - 증상은 실제 노드가 멀쩍하게 잘 돌고 있어도 RViz2가
"Fixed Frame: world does not exist"만 띄우거나, `ros2 node list`/
`ros2 topic info` 같은 기본 CLI 명령조차 멈춰버리는 것이었다.
스크립트로는 못 고쳤고, **재부팅이 유일하게 확인된 해결책**이었다.

그래서 이후로는: Claude가 테스트를 위해 직접 `ros2 run`/`launch`로
노드를 띄우는 걸 최소화하고, 코드 리뷰/정적 검증(문법 검사, MJCF
유효성, 수치 계산 직접 검증) 위주로 하고, 실제 RViz2 확인은 사용자가
직접 하는 쪽으로 방향을 바꿨다. `ros2 run` 같은 명령이 **wrapper
프로세스를 띄우고 실제 실행 파일은 그 자식 프로세스**라서, wrapper
PID만 죽이면 실제 노드는 안 죽고 좀비로 남는 것도 이번에 알게 된
함정이다(`Get-CimInstance Win32_Process`로 실제 자식 PID를 찾아야
한다).

**업데이트 (다음 세션): 읽기 전용 구독조차 위험하다고 확인됐다.**
사용자 세션이 돌고 있는 동안, Claude가 `/tf`를 구독만 하는(아무것도
발행 안 하는) 스크립트를 딱 하나 띄웠는데, 그 자체가 사용자의 이미
멀쩍하게 돌고 있던 RViz2의 TF를 깨버렸다. 즉 "노드를 띄웠다 죽이는
것"만이 아니라, **이 환경에서는 어떤 새 DDS 참가자가 끼어드는 것
자체가 위험하다** - 읽기 전용이어도, 금방 끝나도 마찬가지다.

**그래서 찾은 안전한 대안: `ros2 bag record`를 Claude가 아니라
사용자가 직접 띄우게 하고, 녹화 끝난 뒤 생긴 `.db3` 파일을 Claude가
오프라인으로(네트워크/DDS 전혀 없이) 분석한다.** `rclpy.serialization.
deserialize_message()`로 CDR 직렬화를 풀면 되는데, 이건 순수 로컬
CPU 연산이라 `rclpy.init()`도, 노드 생성도, 네트워크 소켓도 전혀
필요 없다 - `sqlite3`로 `.db3`를 열어서 `/tf` 토픽의 raw 메시지를
읽고, `tf2_msgs.msg.TFMessage`로 역직렬화해서 원하는 `child_frame_id`
만 뽑아 시계열로 분석하면 된다(`C:\Python38\python.exe`에 이미
`rclpy`/`tf2_msgs`가 있으므로, ROS2 환경만 `call local_setup.bat`로
sourcing해서 import 경로만 맞추면 됨 - 노드를 안 띄우니 안전하다).
이 방법으로 실제로 위치/회전/각속도 이상(위 "아직 안 풀린 문제"
섹션)을 직접 찾아냈다 - 화면 녹화/스크린샷보다 훨씬 정확하고,
사용자에게 아무 추가 작업도 안 시켜도 된다(녹화는 어차피 사용자가
직접 트리거해야 하지만).

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
