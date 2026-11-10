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
