# OpenGL Test

두 가지 테스트를 담고 있다.

1. **Teapot test** — `glBegin(GL_TRIANGLES)` / `glVertex3d` / `glNormal3d` /
   `glTexCoord2d` 패턴을 `glutSolidTeapot()`을 통해 확인 (Python/C 양쪽 구현)
2. **Humanoid matrix stack test** — `glPushMatrix`/`glPopMatrix`로 몸통-머리-
   양팔-양다리를 **하나로 이어진 계층 구조**로 다루는 예제 (C, 하드코딩 버전 + OBJ 로더 버전)

## 파일

| 파일 | 설명 |
|---|---|
| `teapot_test.py` | Teapot test, PyOpenGL 버전 |
| `teapot_test.c` | Teapot test, C/freeglut 포팅 버전 |
| `humanoid_test.c` | Humanoid 예제, 하드코딩 버전 (도형/치수/계층이 전부 코드 안에 있음) |
| `gen_humanoid_assets.c` | `humanoid_test.c`의 모델을 실제 정점 데이터로 계산해 OBJ + 씬 파일로 저장 |
| `humanoid_loader_test.c` | 그 OBJ + 씬 파일을 런타임에 읽어 렌더링하는 로더 버전 |
| `humanoid_path.txt` | 몸통이 따라 걷는 웨이포인트 경로 (`x z` 한 줄씩, 폐곡선) |
| `quadruped_patrol.txt` | 배경의 4족 로봇이 왕복하는 경로 설정 (`z x_min x_max speed`) |
| `humanoid.urdf` / `quadruped.urdf` | `gen_humanoid_assets.exe`가 같이 만드는 ROS URDF (파일 생성까지만, 아직 ROS 노드는 없음) |
| `humanoid_extra.yaml` / `quadruped_extra.yaml` | URDF가 담지 못하는 정보(텍스처/옷 색 틴트/애니메이션 파라미터 등)를 담은 보조 파일 |

## 조작

| 키 | 기능 | 대상 |
|---|---|---|
| Space | 와이어프레임/솔리드 토글 (teapot) · 주인공을 그 자리에 정지 (humanoid) | 공통 |
| P | 주인공을 다시 경로 모드로 복귀 (humanoid) | humanoid |
| T | 텍스처 on/off | 공통 |
| L | 조명 on/off | 공통 |
| Q / A | 왼팔 각도 수동 증가/감소 | humanoid |
| E / D | 오른팔 각도 수동 증가/감소 | humanoid |
| Z / X | 왼다리 각도 수동 증가/감소 | humanoid |
| C / V | 오른다리 각도 수동 증가/감소 | humanoid |
| 방향키 위/아래 | 주인공 전진/후진 (수동 조작 모드로 전환) | humanoid |
| 방향키 좌/우 | 주인공 좌/우 회전 (수동 조작 모드로 전환) | humanoid |
| R | 각도/위치 초기화, 경로 모드로 복귀 | humanoid |
| 마우스 왼쪽 드래그 | 카메라 궤도 회전 (yaw/pitch) | 공통 |
| 마우스 휠 | 줌 인/아웃 | 공통 |
| ESC | 종료 | 공통 |

## Python 버전

conda `base` 환경(`D:\miniconda3`)에 PyOpenGL이 설치되어 있음.

```bash
"/d/miniconda3/python.exe" teapot_test.py
```

없는 환경이라면:
```bash
pip install PyOpenGL PyOpenGL_accelerate
python teapot_test.py
```

## C 버전

### 사용 툴체인

이 프로젝트에서는 별도 설치 없이 `D:\OpenModelica1.26.3-64bit`에 번들된
MSYS2 ucrt64 gcc 툴체인을 사용한다. gcc, GL/GLU/freeglut 헤더와 라이브러리가
모두 포함되어 있음.

```
D:\OpenModelica1.26.3-64bit\tools\msys\ucrt64\bin
```

(참고: VS2019 Community의 MSVC(`cl.exe`)도 시스템에 설치되어 있지만,
freeglut 헤더/라이브러리가 없어 이 프로젝트에서는 사용하지 않음.)

### 빌드 & 실행

**Git Bash**
```bash
export PATH="/d/OpenModelica1.26.3-64bit/tools/msys/ucrt64/bin:$PATH"
cd /d/claude_test/computer_graphics
gcc teapot_test.c -o teapot_test.exe -lfreeglut -lopengl32 -lglu32 -lwinmm
./teapot_test.exe
```

**cmd.exe**
```cmd
set PATH=D:\OpenModelica1.26.3-64bit\tools\msys\ucrt64\bin;%PATH%
cd D:\claude_test\computer_graphics
gcc teapot_test.c -o teapot_test.exe -lfreeglut -lopengl32 -lglu32 -lwinmm
teapot_test.exe
```

**PowerShell**
```powershell
$env:PATH = "D:\OpenModelica1.26.3-64bit\tools\msys\ucrt64\bin;" + $env:PATH
cd D:\claude_test\computer_graphics
gcc teapot_test.c -o teapot_test.exe -lfreeglut -lopengl32 -lglu32 -lwinmm
.\teapot_test.exe
```

위 PATH 설정은 세션 한정이다. 매번 설정하기 번거로우면 Windows 시스템
환경변수 PATH에 해당 경로를 영구로 추가해두면 된다.

`teapot_test.exe`를 그 bin 폴더 밖에서 단독으로 배포/실행하려면
`libfreeglut.dll`, `libgcc_s_seh-1.dll`, `libwinpthread-1.dll`을
exe와 같은 폴더로 복사해야 "DLL을 찾을 수 없습니다" 오류가 나지 않는다.

## Humanoid matrix stack test

이 예제의 핵심은 몸통(Body)·머리(Head)·왼팔(LeftArm)·오른팔(RightArm)이
**따로따로 존재하는 도형들이 아니라, 하나의 변환 트리로 수직 연결되어 있다**는
점이다. 각 부위는 자기 자신의 위치가 아니라 **"부모 기준 상대 위치"**만
가지고 있고, 실제 화면에 그려질 위치는 그 부위부터 루트까지 이어지는
변환들을 전부 곱한 결과다.

```
ROOT
 └─ Body       (T1)              → 최종 변환 = T1
     ├─ Head    (T2)             → 최종 변환 = T1 × T2
     ├─ LeftArm (T2 × R1)        → 최종 변환 = T1 × T2 × R1
     ├─ RightArm(T2 × R2)        → 최종 변환 = T1 × T2 × R2
     ├─ LeftLeg (T3 × R3)        → 최종 변환 = T1 × T3 × R3
     └─ RightLeg(T3 × R4)        → 최종 변환 = T1 × T3 × R4
```

이게 왜 "수직적 연계"인지: `Body`(T1)를 옮기면 그 아래 매달린 `Head`,
`LeftArm`, `RightArm`도 전부 같이 따라 움직인다 — 셋 다 자기 변환 앞에
`T1`을 곱하고 있기 때문이다. 즉 부모의 변화가 자식 전체에 전파된다.
반대로 `LeftArm`의 회전(`R1`)은 `RightArm`(`R2`)에는 전혀 영향을 주지
않는데, 이는 각 팔을 그리기 직전에 `glPushMatrix()`로 "부모까지의
변환 상태(T1×T2)"를 스냅샷 떠두고, 다 그린 뒤 `glPopMatrix()`로 그
스냅샷으로 되돌리기 때문이다. 두 팔은 서로 독립이 아니라 **같은
부모(T1×T2)를 공유하는 형제(sibling)** 관계이고, push/pop은 그
형제끼리 서로의 변환이 새지 않게 막아주는 장치일 뿐, 부모와의 연결
자체는 끊지 않는다.

- `humanoid_test.c`는 이 트리를 코드의 중첩 구조(`glPushMatrix`/
  `glTranslated`/`glRotated`의 들여쓰기)로 직접 표현한다.
- `gen_humanoid_assets.c` → `humanoid_loader_test.c`는 같은 트리를
  **데이터**(`humanoid_scene.txt`의 `parent` 컬럼)로 표현하고, 로더가
  그 부모-자식 관계를 재귀적으로 따라가며 push/pop 트리를 그때그때
  구성한다. 표현 방식만 다를 뿐 "부모 변환이 자식에 누적된다"는
  본질은 동일하다.

### 빌드 & 실행 — 하드코딩 버전

```bash
export PATH="/d/OpenModelica1.26.3-64bit/tools/msys/ucrt64/bin:$PATH"
cd /d/claude_test/computer_graphics
gcc humanoid_test.c -o humanoid_test.exe -lfreeglut -lopengl32 -lglu32 -lwinmm
./humanoid_test.exe
```

### 빌드 & 실행 — OBJ 로더 버전

먼저 자산(OBJ + 씬 파일)을 생성하고, 그 다음 로더를 실행한다.

```bash
export PATH="/d/OpenModelica1.26.3-64bit/tools/msys/ucrt64/bin:$PATH"
cd /d/claude_test/computer_graphics

gcc gen_humanoid_assets.c -o gen_humanoid_assets.exe -lm
./gen_humanoid_assets.exe
# → body.obj, head.obj, arm.obj, leg.obj,
#    quad_body.obj, quad_head.obj, quad_leg.obj, humanoid_scene.txt 생성

gcc humanoid_loader_test.c -o humanoid_loader_test.exe -lfreeglut -lopengl32 -lglu32 -lwinmm
./humanoid_loader_test.exe
```

`humanoid_scene.txt`는 이런 형태다 (컬럼: 이름, 메쉬 파일, **부모**, 위치,
관절 종류, 색):

```
# name     mesh       parent  tx       ty       tz  joint  r     g     b
Body       body.obj   ROOT    0.0      0.7000    0.0  NONE   0.55  0.55  0.65
Head       head.obj   Body    0.0      0.7000    0.0  NONE   0.85  0.75  0.65
LeftArm    arm.obj    Body    -0.5250  0.7000    0.0  ROTX   0.45  0.45  0.55
RightArm   arm.obj    Body    0.5250   0.7000    0.0  ROTX   0.45  0.45  0.55
LeftLeg    leg.obj    Body    -0.2500  -0.7000   0.0  ROTX   0.35  0.35  0.45
RightLeg   leg.obj    Body    0.2500   -0.7000   0.0  ROTX   0.35  0.35  0.45
```

`Head`/`LeftArm`/`RightArm`/`LeftLeg`/`RightLeg`의 `parent`가 전부 `Body`로
적혀 있는 것이 바로 이 수직 연계를 데이터로 표현한 부분이다.

## 하늘 / 땅 / 그림자

humanoid 예제(하드코딩 버전 + OBJ 로더 버전) 둘 다 `display()`에서
매 프레임 다음 순서로 그린다.

```
1. draw_sky()        화면 전체를 덮는 하늘-지평선 그라디언트
2. apply_camera()     3D 카메라 설정 (이후부터 "월드 공간")
3. draw_axes()        원점 좌표축
4. draw_ground()       y=-0.01의 큰 흙색 사각형
5. (그림자 패스) 전단 변환을 곱한 채로 몸 전체를 반투명 검은색으로
6. (실제 캐릭터 패스) 몸 전체를 평소대로
```

### 1. 하늘 — `draw_sky()`

3D 장면과 완전히 분리된 2D 오버레이다. `glClear()` 직후, 아직 3D
카메라를 설정하기 전에:

1. `GL_DEPTH_TEST`/`GL_LIGHTING`/`GL_TEXTURE_2D`를 끈다 (배경이라
   깊이/조명과 무관해야 함).
2. `GL_PROJECTION`을 `gluOrtho2D(0,1,0,1)`로, `GL_MODELVIEW`를
   단위행렬로 **임시로** 바꾼다 (둘 다 `glPushMatrix()`로 저장해둔
   원래 값을 나중에 `glPopMatrix()`로 복원한다 — 그래야 뒤따르는
   `apply_camera()`의 perspective 투영에 영향이 없다).
3. 화면 전체(0,0)~(1,1)를 덮는 `GL_QUADS` 하나를 그리는데, 위쪽 두
   꼭짓점은 하늘색(`0.35,0.55,0.85`), 아래쪽 두 꼭짓점은 밝은
   지평선색(`0.80,0.88,0.95`)을 준다. OpenGL 기본값인
   `GL_SMOOTH` 셰이딩이 그 사이를 자동으로 보간해 그라디언트가
   생긴다.
4. 행렬을 복원하고 `GL_DEPTH_TEST`를 다시 켠다.

이후 3D 장면은 깊이 테스트가 있는 평소 방식으로 그려지므로, 사물이
있는 자리는 하늘을 덮어 그리고 사물이 없는 자리(배경)에는 이
그라디언트가 그대로 남는다 — "그림을 그려놓은 배경판" 기법이다.

### 2. 땅 — `draw_ground()`

원점 주변 `[-20, 20] × [-20, 20]` 범위를 덮는 단일 사각형을
`y = GROUND_Y = -0.01`에 그린다. 그림자가 투영되는 평면이 정확히
`y = 0`이기 때문에, 땅을 그 평면보다 살짝(`-0.01`) 아래에 둬서 같은
높이에서 생기는 z-fighting(깜빡거림)을 피한다. 법선은 `(0,1,0)`
고정이라 조명을 받으면 위를 향한 평평한 면으로 보인다. 색은 흙색
(`0.42, 0.30, 0.18`)이고, `texturing`이 켜져 있으면 다른 부위와
똑같은 체커 텍스처가 `GL_TEXTURE_GEN`으로 자동 입혀져 격자 바닥처럼
보인다 (텍스처 좌표를 안 만들어주는 사각형이라도 object-linear
텍스처 생성이 그대로 적용되기 때문).

### 3. 그림자 — 평행광 전단(shear) 투영

캐릭터를 그리는 함수(`draw_humanoid_body()` / `draw_humanoid_root()`)
를 **두 번** 호출해서 그림자를 만든다. 한 번은 평소대로, 한 번은
"광원 기준으로 y=0 평면에 투영하는 행렬"을 미리 곱해둔 채로.

처음 구현은 정석적인 점광원 투영 행렬(레드북에 나오는, 광원 위치
L과 평면으로 정의되는 4×4 투영 행렬)을 썼다. 이 행렬은 동차좌표의
w값이 `Ly - y`(광원 높이에서 그 점의 높이를 뺀 값)가 되는데, 캐릭터의
머리처럼 `y`가 광원 높이(`LIGHT_Y = 4.0`)에 가까워질수록 `w`가
0에 가까워지면서 투영된 그림자가 기하급수적으로 멀리 늘어난다.
그렇게 늘어난 삼각형이 카메라의 절두체(view frustum) 경계와 만나면
클리핑되면서 "그림자가 화면 위쪽에서 잘리는" 것처럼 보이는 문제가
있었다.

그래서 점광원 대신 **평행광(광원이 무한히 멀리 있다고 가정)**
전단 변환으로 바꿨다. 광원 위치 `L=(Lx,Ly,Lz)`를 위치가 아니라
방향으로만 쓰고, 한 점 `P=(x,y,z)`에서 그 방향을 따라 `y=0`
평면까지 내려가는 변위를 구한다:

```
P' = P - (y / Ly) * L
  x' = x - (Lx/Ly) * y
  y' = 0
  z' = z - (Lz/Ly) * y
```

이건 순수한 **선형(affine) 변환**이라 동차좌표의 `w`가 항상 `1`로
고정된다 — 점광원 투영과 달리 나누기(perspective divide)가 전혀
없으므로 `y`가 얼마나 크든 그림자가 무한히 늘어나거나 잘리는 일이
없다. 대신 "광원이 아주 멀리 있다"고 가정하는 근사라서, 광원이
실제로 가까운 점광원이라면 그림자 모양이 약간 부정확해질 수 있는데,
이 데모에서는 방향감만 맞으면 충분하다. `apply_shadow_matrix(lx,ly,lz)`
가 이 행렬을 `glMultMatrixd()`로 곱해준다.

그림자를 그리는 동안 지켜야 하는 상태:

- `GL_LIGHTING`/`GL_TEXTURE_2D`를 끈다 — 그림자는 단색이어야 하므로
  조명 계산도, 텍스처도 필요 없다 (조명이 꺼진 상태에서는 `glColor`가
  바로 화면색이 되므로, 부위별 `glColor3f` 호출들을 그대로 두면
  그림자가 몸 색 그대로 투영돼버린다 — 그래서 각 부위를 그리는
  함수 안에서 `rendering_shadow` 플래그가 켜져 있으면 색을 바꾸지
  않도록 막아둔다).
- `GL_BLEND` + `glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)`로
  반투명하게(`glColor4f(0,0,0,0.35)`) 그린다 — 완전히 검은 단색보다
  바닥 질감이 비쳐 보이는 반투명 그림자가 더 자연스럽다.
- `glDepthMask(GL_FALSE)`로 깊이 버퍼에 쓰지 않는다 — 그림자는
  이미 그려진 땅(또는 다른 물체) 위에 "겹쳐 보이기만" 하면 되고,
  깊이값까지 갱신하면 그 뒤에 그려지는 실제 캐릭터와 애매하게
  z-fighting을 일으킬 수 있다.

### 4. ROOT 높이 보정

캐릭터의 ROOT 변환은 원래 "엉덩이 높이"를 기준으로 삼고 있었다 —
몸통(Body)이 `ROOT`에서 위로, 다리(Leg)가 `ROOT`에서 아래로
`LEG_LEN`만큼 매달리는 구조라서다. 땅이 없을 때는 다리가 `y<0`까지
내려가도 보이는 문제가 없었는데, `y=0`에 실제 땅을 깔고 나니 다리가
그 아래로 파고드는(땅에 "묻히는") 것처럼 보였다. 고치는 방법은
ROOT 자체를 다리 길이만큼 들어올리는 것: `glTranslated(body_x,
LEG_LEN, body_z, ...)`. 그러면 발(다리 끝)이 정확히 `y=0`, 즉 땅
표면에 닿는다.

### 5. 나무 — `draw_tree()` / `draw_trees()`

배경이 너무 밋밋해서 걷는 경로(사각형) 바깥쪽에 정적인 나무 5그루를
심었다. 각 나무는:

- **줄기**: 갈색으로 스케일한 상자 하나.
- **잎**: 크리스마스 트리처럼 아래로 갈수록 커지는 원뿔 3단을 겹쳐
  쌓는다. `glutSolidCone()`은 기본적으로 로컬 +Z축 방향으로
  뾰족해지는 모양이라, X축으로 -90도 돌려서 +Y(위) 방향을 향하게
  한다. 아래쪽 원뿔일수록 짙은 초록, 위쪽일수록 밝은 초록을 줘서
  입체감을 낸다.
- **장식구**: 각 원뿔 표면 위에 작은 빨간 구를 몇 개씩 얹었다.
  원뿔은 선형으로 가늘어지므로, 밑면에서 높이 비율 `f`만큼 올라간
  지점의 반지름은 `tier_radius * (1 - f)`다. 장식구를 그 반지름의
  1.08배 되는 위치(표면보다 살짝 바깥)에 두면 잎 속에 파묻히지
  않고 표면 위에 올라앉은 것처럼 보인다.

나무도 캐릭터와 똑같은 `rendering_shadow` 플래그 + 전단 변환
그림자를 적용받는다. 처음 구현에서는 그림자 패스 안에서
`draw_humanoid_body()`(또는 `draw_humanoid_root()`)를 부른 다음 바로
`draw_trees()`를 불렀는데, 그 몸 그리기 함수가 자기 몫의 ROOT
이동/회전을 **자체 push/pop 없이** 현재 행렬에 바로 곱해버리는
함수라서 — 뒤따라오는 나무 그림자까지 그 변환의 영향을 받아
캐릭터를 따라 같이 움직이고 돌아버리는 버그가 있었다. 고친 방법은
몸 그리기 호출만 별도의 `glPushMatrix()`/`glPopMatrix()`로 감싸서,
ROOT 변환이 나무를 그리기 전에 확실히 걷히게 한 것.

## 사람 텍스처 (피부 / 상의 / 하의)

처음에는 모든 부위(머리/몸통/팔/다리)가 teapot·땅과 똑같은 체커
텍스처를 공유했다. 사람을 좀 더 그럴듯하게 보이려고, 부위별로
서로 다른 단색 텍스처 3장을 새로 만들어 입혔다:

| 텍스처 | 색 | 입히는 부위 |
|---|---|---|
| `skin_texture_id` | 살색 단색 | Head |
| `shirt_texture_id` | 남색/흰색 가로 줄무늬 | Body, LeftArm, RightArm (긴팔 소매로 취급) |
| `pants_texture_id` | 데님 청색 단색 | LeftLeg, RightLeg |

- `humanoid_test.c`는 `draw_body()`/`draw_head()`/`draw_arm()`/
  `draw_leg()`가 부위별 함수로 따로 있어서 그 안에서 바로
  `glBindTexture()`로 자기 몫의 텍스처를 바인딩한다.
- `humanoid_loader_test.c`는 모든 부위를 `render_children()` 하나가
  씬 데이터를 순회하며 그리므로, `texture_for_part(part->name)`으로
  이름(`"Head"`, `"Body"`, ...)에 따라 어떤 텍스처를 바인딩할지
  분기한다.
- `GL_MODULATE` 모드라 최종 색 = 텍스처 색 × vertex 색이다. 텍스처를
  입힐 때는 vertex 색을 흰색(`1,1,1`)으로 둬야 텍스처 색이 그대로
  나온다 — 기존에 쓰던 칙칙한 회색/남색 vertex 색을 그대로 두면
  텍스처 위에 그 색이 덧씌워져 탁해진다. 텍스처가 꺼져 있을 때는
  원래 쓰던 단색(씬 파일의 `color` 컬럼 / 하드코딩된 `glColor3f`)로
  돌아간다.
- 머리의 빨간 코는 피부 텍스처 위에 순색 빨강이 또렷이 보이도록,
  그릴 때만 `GL_TEXTURE_2D`를 잠깐 꺼서 텍스처 없이 순수 `glColor`
  값으로 칠한다.
- 처음에는 skin/pants 텍스처에 `(i*7+j*13)%17` 같은 좌표 기반
  모듈러 연산으로 잡티(noise)를 섞어봤는데, 그 모듈러 패턴이 일정한
  간격으로 반복되면서 체커 패턴과 다를 바 없는 격자 무늬처럼 보여
  오히려 더 보기 나빴다. 그래서 잡티 없는 깔끔한 단색으로
  되돌렸다 (상의의 가로 줄무늬는 의도된 패턴이라 그대로 유지).
- 땅(`draw_ground()`)과 나무(`draw_trees()`)는 체커 텍스처
  (`texture_id`)와의 연결을 완전히 끊었다. `display()`에서 그 둘을
  그리기 직전에 항상 `glDisable(GL_TEXTURE_2D)`를 호출해서,
  `texturing` 토글이나 어떤 텍스처가 마지막에 바인딩돼 있었는지와
  무관하게 그 함수 안의 `glColor3f` 값(흙색 / 나무껍질·잎·장식구
  색)만으로 그려지게 한다. 예전에는 사람에게 셔츠/바지 텍스처를
  바인딩한 다음 나무를 그리면, 나무가 그 마지막으로 바인딩된
  텍스처(바지 등)를 그대로 입은 것처럼 보이는 문제가 있었다.

## 4족 보행 로봇 (배경 소품)

humanoid와 똑같은 matrix stack 기법(몸통 → 머리/다리로 이어지는
push/pop 트리)을 4족 로봇에도 적용해, 사람이 걷는 장면의 배경
소품으로 넣었다. 처음에는 독립 실행 데모(`quadruped_test.c`)로
따로 만들었지만, 목적은 어디까지나 기존 humanoid 장면에 같이
집어넣는 것이었어서 그 파일은 지우고 `humanoid_test.c` /
`humanoid_loader_test.c` 안으로 코드를 옮겼다.

- **구조**: 몸통(Body, z축으로 긴 상자) → 머리(Head, 몸통 앞쪽 끝) +
  다리 4개(FrontLeft/FrontRight/BackLeft/BackRight, 몸통 네
  귀퉁이에서 아래로 매달림). `draw_quadruped()`가 `draw_trees()`와
  똑같이 자기 몫의 ROOT 변환을 **내부에서** push/pop으로 감싸서,
  호출하는 쪽(그림자 패스 등)에서 따로 감싸지 않아도 그 변환이
  다음에 그려지는 다른 소품으로 새지 않는다.
- **걸음걸이**: 트롯(trot) - 대각선으로 마주보는 두 다리(앞왼쪽+
  뒤오른쪽, 앞오른쪽+뒤왼쪽)가 쌍을 이뤄 서로 반대 위상으로
  흔들린다. humanoid의 팔다리 스윙과 같은 원리로 공유 `t`에서
  `30*sin(2t)`로 직접 계산한다(프레임 속도 무관).
- **왕복(patrol) 경로**: 처음엔 제자리걸음만 했는데, 사람과 동선이
  겹치지 않게 왔다 갔다 하면 좋겠다는 요청으로 x축 방향 왕복을
  추가했다. z는 `quadruped_patrol.txt`에서 읽은 고정값(기본 -9)이라
  humanoid의 걷는 경로(z: -3.5..3.5)와 절대 교차하지 않는다. 위치는
  `x = center + half_range * sin(speed*t)`로, 이 역시 humanoid_path.txt
  처럼 하드코딩 대신 `quadruped_patrol.txt`("z x_min x_max speed"
  한 줄)에서 읽어 수정하기 쉽게 했다 (파일이 없으면 기본값 사용).
- **방향 전환**: 처음에는 이동 방향(`cos(phase)`의 부호)이 바뀌는
  순간 heading을 90도에서 -90도로 그냥 뚝 끊어 바꿨는데, 몸이
  순간이동한 것처럼 보여 부자연스러웠다. `heading = 90 *
  tanh(k*cos(phase))`로 바꿔서, 왕복 양 끝(마침 위치도 거의 멈춰있는
  지점)에서 heading이 90 → 0(로컬 +Z, x축에 수직으로 선 모습) → -90으로
  부드럽게 돌아가는 "제자리 방향 전환" 애니메이션이 자연스럽게
  생기게 했다. `k`(`QUAD_TURN_SHARPNESS`)가 클수록 전환 구간이 좁아져
  더 급하게 꺾인다.
- 몸 색도 땅/나무와 같은 이유로 texturing 토글/체커 텍스처와
  분리된 단색이다.
- **하드코딩 vs OBJ 로더**: 처음엔 두 버전 모두 `glutSolidCube()`를
  직접 호출하는 하드코딩 방식으로 넣었는데, "로더 버전은 사람처럼
  OBJ 로드해서 그리고 싶다"는 요청에 따라 `humanoid_loader_test.c`
  쪽만 데이터 기반으로 바꿨다 (`humanoid_test.c`는 원래 취지대로
  하드코딩 버전 그대로 유지). `gen_humanoid_assets.c`가
  `quad_body.obj`/`quad_head.obj`/`quad_leg.obj`를 만들고,
  `humanoid_scene.txt`에 parent가 `QUAD_ROOT`인 부분들
  (`QuadBody`/`QuadHead`/`QuadLegFL`/`FR`/`BL`/`BR`)로 그 계층을
  기술한다. `QUAD_ROOT`는 humanoid의 `ROOT`와는 별개의 가상 부모라서,
  `draw_quadruped()`가 `render_children("QUAD_ROOT")`를 직접 호출하며
  순찰 위치/방향을 미리 곱해둬도 humanoid의 걷기 변환과 섞이지
  않는다 - `render_children("ROOT")`와 완전히 같은 함수를 이름만
  다르게 불러 재사용하는 것뿐이다.

## 여러 캐릭터 동시 렌더링

humanoid 몸을 그리는 함수(`draw_humanoid_body()` /
`draw_humanoid_root()`)가 원래는 전역 상태(`body_x`/`body_z`/
`body_heading`)를 직접 읽어서 캐릭터 1명만 그릴 수 있었다. 여러
캐릭터를 동시에 그리기 위해 그 함수를 "ROOT의 월드 위치/방향을
인자로 받는" 형태로 바꾸고, 그 위에 `draw_all_humanoids()`를
얹었다:

- **위치**: 주인공(`body_x`/`body_z`/`body_heading`, 방향키로 수동
  조작 가능)과 똑같은 `humanoid_path.txt` 경로를 걷는 "클론"
  (`HUMANOID_CLONE_COUNT`명, 기본 2명)을 추가했다. 클론의 위치는
  매 틱 독립적으로 계산하지 않고, `path_sample()`에 주는 이동
  거리(s)에 `path_total_len / (클론 수+1)`만큼씩 다른 오프셋을
  더해서 구한다 - s가 경로의 "호의 길이"이므로, 이렇게 等分하면
  경로 모양과 무관하게 항상 고리 위에 고르게 퍼진 채로 같은
  속도로 따라 걷는다.
- **애니메이션**: 팔다리 스윙(`left_angle` 등)은 전부 공유 전역
  상태라 모든 캐릭터가 같은 박자로 움직인다 - "같은 동작으로 경로를
  도는 여러 캐릭터"만 보여주는 게 목적이라 단순하게 위치만
  instance별로 다르게 뒀다.
- **옷 색 구별**: 다 똑같이 생기면 구별이 안 되길래, 셔츠/바지
  텍스처에 캐릭터별 틴트(tint)를 곱했다. `GL_MODULATE`라 최종 색 =
  텍스처 색 × vertex 색인데, 기존엔 항상 흰색(1,1,1)을 곱해서
  텍스처 색이 그대로 나오게 했던 것을, `draw_all_humanoids()`가
  캐릭터를 바꿔 그리기 전에 `outfit_tint`에 그 캐릭터의 색을
  넣어두는 방식으로 바꿨다 (주인공은 흰색=원래 색 그대로, 클론
  1은 붉은 톤, 클론 2는 푸른 톤). 얼굴(피부) 텍스처는 그대로
  흰색을 써서 모든 캐릭터의 얼굴은 같게 두고 옷만 구별되게 했다.
- 그림자/나무/4족 로봇과 마찬가지로, `draw_all_humanoids()`도
  각 캐릭터의 ROOT 변환을 내부에서 push/pop으로 감싸서 호출하는
  쪽에서 따로 감싸지 않아도 변환이 새지 않는다.

## 주인공 걷기 모드 (경로 / 수동 / 정지) 분리

처음엔 `animate` 하나로 "팔다리 스윙 + 주인공 경로 이동 + 클론들 +
4족 로봇"을 전부 묶어서 켜고 껐다. 그래서 방향키로 주인공을 수동
조작하면(=`animate = 0`) 클론과 4족 로봇도 같이 멈춰버렸는데,
"동선이 겹치지 않는 클론/로봇은 주인공 상태와 무관하게 항상
움직여야 한다"는 요청으로 아래처럼 분리했다.

- **`t`는 항상 전진한다.** `idle()` 맨 앞에서 조건 없이
  `t += 0.004`를 한다. 클론들의 경로 위치, 4족 로봇의 순찰/트롯은
  전부 이 `t`로부터 직접 계산되므로, 주인공이 뭘 하든 항상
  계속 움직인다.
- **팔다리 스윙은 `limb_animate`로만 따로 켜고 끈다.** Q/A/E/D/Z/X/C/V로
  각도를 수동으로 조절할 때 이 플래그를 꺼서, 다음 틱에
  `t`로부터 다시 계산된 값이 그 수동 조정을 덮어쓰지 않게 한다.
  (처음엔 이걸 깜빡해서, 수동으로 각도를 만진 다음 방향키로
  이동하면 다리가 고정된 채로 미끄러지듯 움직이는 버그가 있었다 -
  고친 방법은 방향키를 누르는 순간 `limb_animate`를 다시 켜는
  것. "이동을 시작하면 걷는 동작도 같이 살아나야 한다"는 게
  더 자연스러운 동작이기 때문이다.)
- **주인공의 위치/방향은 `main_mode` 세 가지 중 하나로 결정된다:**
  - `MAIN_MODE_PATH` (기본값): 매 틱 `path_sample(0.6*t, ...)`로
    위치를 다시 계산해 경로를 자동으로 걷는다.
  - `MAIN_MODE_MANUAL`: 방향키를 누르는 순간 이 모드로 전환되고,
    이후 `idle()`은 `body_x`/`body_z`/`body_heading`을 건드리지
    않는다 - `special_keys()`가 방향키 입력으로 직접 바꾼다.
  - `MAIN_MODE_STOPPED`: 스페이스바를 누르면 이 모드가 되고, 아무도
    위치를 건드리지 않아 그 자리에 가만히 선다.
  - `P`를 누르면 다시 `MAIN_MODE_PATH`로 돌아가 경로를 계속
    걷는다(현재 `t`에 해당하는 경로 위 지점으로 즉시 이어붙기
    때문에, 경로에서 멀리 벗어나 있었다면 그 지점으로 순간
    이동하듯 복귀한다 - 부드러운 복귀 보간은 하지 않는 단순한
    구현이다).
- 방향키 좌/우는 `body_heading += 5` / `-= 5`다 (처음엔 반대로
  둬서 좌우가 뒤집혀 보이는 버그가 있었다).
- `MAIN_MODE_MANUAL`로 한참 걸어나가면 땅과 카메라 시야 바깥으로
  사라질 수 있었다. 이를 막기 위해 `special_keys()`에서 방향키로
  이동한 뒤 `body_x`/`body_z`를 `PLAY_HALF`(=`GROUND_HALF - 1.0`)
  범위 안으로 고정(clamp)한다. 같이 `GROUND_HALF`도 20 → 30으로
  넓히고, 더 넓어진 땅이 멀리서도 다 보이도록 카메라 줌아웃 최대
  거리(`cam_dist` 상한)를 20 → 40, 원근 투영의 far 클리핑 평면을
  50 → 100으로 같이 늘렸다 (안 늘리면 땅 모서리 쪽이 시야 밖으로
  잘려 보일 수 있다).

## URDF / YAML 내보내기 (ROS2)

`gen_humanoid_assets.c`는 OpenGL 데모용 OBJ + 씬 파일 외에, 같은
모델을 ROS2(URDF)로도 내보낸다 - `humanoid.urdf` / `quadruped.urdf`와
그 옆의 `humanoid_extra.yaml` / `quadruped_extra.yaml`. **이번 작업
범위는 파일 생성까지다** - TF/`joint_states`를 발행해서 RViz2에
실제로 띄우는 ROS2 노드는 아직 없다(환경은 Windows용 ROS2 Humble
바이너리 설치본으로 확인했음).

### 왜 YAML이 따로 필요한가

URDF는 로봇의 정적인 뼈대(링크-관절 계층)와 겉모습(메쉬+단색)만
표현한다. 그런데 이 프로젝트의 OpenGL 데모는 그보다 훨씬 많은
정보를 담고 있다 - 절차적 텍스처, 캐릭터별 옷 색, 시간에 따른
걷기/트롯 애니메이션, 경로 파일 등. 이런 건 URDF 스키마 자체에
담을 자리가 없어서, 변환 과정에서 그냥 버리면 "URDF만 보고는 원래
모습으로 되돌릴 수 없는" 정보 손실이 생긴다. 그래서 URDF 옆에
같은 이름의 `*_extra.yaml`을 같이 둬서, 사람이 다시 읽고 OpenGL
쪽 구현에 참고하거나 나중에 더 풍부한 표현(예: Gazebo 플러그인,
커스텀 RViz 디스플레이)으로 복원할 수 있게 적어뒀다. 구체적으로:

- **텍스처**: URDF `<material><color rgba=.../></material>`는 단색
  하나만 표현할 수 있다. 실제로는 상의가 가로 줄무늬(절차적으로
  생성한 64×64 텍스처)인데, URDF에는 그 "대표색"만 넣고, 줄무늬
  색/주기 같은 진짜 생성 파라미터는 `humanoid_extra.yaml`의
  `materials` 아래에 적었다.
- **캐릭터별 옷 색 틴트**: 여러 캐릭터(주인공+클론)를 동시에 그릴 때
  셔츠/바지에 곱하는 `outfit_tint`는 애초에 "로봇 1개의 구조"만
  표현하는 URDF의 범위를 벗어난다 (인스턴스가 여러 개라는 개념 자체가
  URDF엔 없음). `instance_outfit_tints`에 적어뒀다.
- **애니메이션/경로**: URDF는 "지금 이 모양"만 표현하고 "시간에 따라
  어떻게 움직이는가"는 담지 않는다. 팔다리 스윙 공식
  (`35*sin(t)` 등), 걷기 경로 파일(`humanoid_path.txt`)과 보간 방식
  (Catmull-Rom), 4족 로봇의 트롯/순찰 공식과 설정 파일
  (`quadruped_patrol.txt`)을 `animation` 아래에 적었다.
- **코 장식**: 유일하게 "안 잃은" 예외. `draw_head()`에서 OBJ 메쉬가
  아니라 매 프레임 따로 그리던 작은 구였는데, URDF의 `<visual>`은
  링크 하나에 여러 개를 둘 수 있고 `<sphere>` 프리미티브도 있어서,
  Head 링크에 두 번째 `<visual>`(빨간 구)로 손실 없이 그대로
  옮겼다 - `decorations`에 그 사실만 메모해뒀다.
- **관절 각도 한계**: 원래 애니메이션은 순수 기구학적 `sin()` 함수라
  동역학적 한계가 없다. URDF는 `<limit>`(effort/velocity 포함)가
  문법적으로 필요해서, 실제 스윙 진폭(팔 35도·다리 25도·4족 로봇
  다리 30도)에 여유를 더한 자리값(팔 45도·다리 35도·4족 로봇
  45도)을 넣고, YAML에 "이건 진짜 한계가 아니라 placeholder"라고
  적어뒀다.
- **장면/배경**: 하늘·땅·그림자·나무는 로봇 자체가 아니라 장면
  요소라 URDF/YAML 어디에도 넣지 않았다 - `scene_not_included`에
  "이건 의도적으로 안 넣었다"는 목록만 남겨서, 나중에 world/Gazebo
  파일로 옮길 때 빠뜨리지 않게 해뒀다.

### Y-up → Z-up 축 변환

OpenGL 데모는 Y가 위인 좌표계를 쓴다 (X=왼쪽, Y=위, Z=앞 - 방향키
좌/우를 고칠 때 확인한 바로 그 축). ROS는 REP-103 규약에 따라 Z가
위다 (X=앞, Y=왼쪽, Z=위). 그래서 모든 위치를
`ros_xyz = (ours_z, ours_x, ours_y)`로 바꿔서 내보낸다. 이건 두
우수(right-handed) 좌표계 사이의 순수한 회전(행렬식 +1인 치환
행렬)이라 회전 "방향"(각도의 부호)은 그대로 유지되고 좌표축
이름만 재배치된다 - 그래서 팔/다리의 ROTX(어깨·엉덩이 pitch) 회전은
ours의 X축이 ros의 Y축이 되어 URDF에서 `axis="0 1 0"`이 된다
(ROS의 pitch 축과도 자연스럽게 일치).

메쉬 파일도 같은 변환을 적용한 `*_ros.obj`를 새로 만들어 URDF가
참조한다 (`body_ros.obj`, `head_ros.obj`, ... 기존 `body.obj` 등은
Y-up 그대로 남겨 OpenGL 데모에는 영향이 없다). 상자 메쉬는
`write_box_obj()`에 min/max 인자 순서만 `(z,x,y)`로 바꿔 넣으면
축이 재배치된 메쉬가 그대로 나오고(상자는 축끼리 쌍을 이루는
min/max 구간일 뿐이라 재배치로 충분함), 구(머리)는 극축이 코드에
박혀 있어서 `write_sphere_obj_zup()`을 따로 만들었다.

### 다음 단계로 남겨둔 것

- URDF의 `<mesh filename="...">`는 지금 상대 경로(예:
  `body_ros.obj`)만 적혀 있다. 실제로 `robot_state_publisher`/
  RViz2에서 띄우려면 ROS 패키지를 만들어 `package://<pkg>/...`로
  바꾸거나, 절대경로 `file:///...`로 바꿔야 한다.
- TF/`joint_states`를 발행하는 ROS2 노드(`rclpy`)는 아직 없다 -
  이번엔 "URDF/YAML 파일 생성까지"로 범위를 정했고, 다음 단계는
  이 노드를 작성해서 지금 이 세션에서 바로 쓸 수 있는 Windows용
  ROS2 Humble + RViz2로 실제 시각화해보는 것.

## 구현 메모

- `glutSolidTeapot()`은 정점(Vertex)/법선(Normal)만 생성하고 텍스처 좌표는
  만들어주지 않는다. 원본 메시 데이터에 접근할 수 없어 텍스처 좌표는
  `GL_TEXTURE_GEN_S/T` (object-linear)로 자동 생성한다.
- 회전 속도는 `idle()`에서 프레임마다 `angle`에 더해지는 값으로 조절한다
  (C 버전 현재 `0.01`, Python 버전 `0.05`).
- humanoid 예제의 팔 스윙 속도도 같은 방식으로 `idle()`의 `t += 0.004`
  값으로 조절한다.
- OBJ 로더는 최소 구현이라 `v`/`vn`/`f`(포맷 `f v//vn v//vn v//vn`)만
  지원하고 텍스처 좌표(`vt`)는 읽지 않는다. `gen_humanoid_assets.c`가
  항상 이 포맷으로만 저장하므로 두 프로그램 사이에서는 문제없다.
- humanoid 예제(하드코딩 버전 + OBJ 로더 버전) 둘 다 teapot과 같은
  체커 텍스처 + `GL_TEXTURE_GEN_S/T`를 그대로 재사용한다. `glutSolidCube`/
  `glutSolidSphere`도, OBJ 로더가 읽은 메시도 텍스처 좌표가 없는 건
  동일하므로 같은 해법이 그대로 통한다.
- 몸통의 이동 경로는 더 이상 고정된 원이 아니라 `humanoid_path.txt`에
  적힌 웨이포인트(`x z`)를 순서대로 잇는 폐곡선이다 (파일이 없으면
  기본 사각형 경로). `body_heading`/`body_x`/`body_z`에 매 틱 고정값을
  누적시키지 않고, 팔다리 스윙과 같은 `t`로부터 "이동 거리 s = 0.6*t"를
  직접 계산해(`path_sample`) 그 거리에 해당하는 위치/방향을 구하므로
  `idle()` 호출 빈도(프레임 속도)와 무관하게 항상 같은 속도로 걷는다.
- 웨이포인트를 직선으로 잇지 않고 Catmull-Rom 스플라인으로 보간한다
  (`path_sample` 내부). 직선 보간은 꼭짓점마다 진행 방향이 뚝뚝 꺾여
  경로가 들쭉날쭉해 보이는데, 스플라인은 앞뒤 점까지 참고해 접선이
  연속적으로 이어지므로 꼭짓점에서도 부드럽게 방향이 바뀐다.
- humanoid 머리의 +Z쪽에 작은 빨간 공("코")을 붙여 앞/뒤를 구별할 수
  있게 했다. 구(sphere)만으로는 몸통이 회전해도 앞/뒤가 똑같이 보여서
  실제로 방향을 바꾸며 걷는지 확인하기 어려웠기 때문.
