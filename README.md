# OpenGL Test

두 가지 테스트를 담고 있다.

1. **Teapot test** — `glBegin(GL_TRIANGLES)` / `glVertex3d` / `glNormal3d` /
   `glTexCoord2d` 패턴을 `glutSolidTeapot()`을 통해 확인 (Python/C 양쪽 구현)
2. **Humanoid matrix stack test** — `glPushMatrix`/`glPopMatrix`로 몸통-머리-
   양팔을 **하나로 이어진 계층 구조**로 다루는 예제 (C, 하드코딩 버전 + OBJ 로더 버전)

## 파일

| 파일 | 설명 |
|---|---|
| `teapot_test.py` | Teapot test, PyOpenGL 버전 |
| `teapot_test.c` | Teapot test, C/freeglut 포팅 버전 |
| `humanoid_test.c` | Humanoid 예제, 하드코딩 버전 (도형/치수/계층이 전부 코드 안에 있음) |
| `gen_humanoid_assets.c` | `humanoid_test.c`의 모델을 실제 정점 데이터로 계산해 OBJ + 씬 파일로 저장 |
| `humanoid_loader_test.c` | 그 OBJ + 씬 파일을 런타임에 읽어 렌더링하는 로더 버전 |

## 조작

| 키 | 기능 | 대상 |
|---|---|---|
| Space | 와이어프레임/솔리드 토글 (teapot) · 자동 애니메이션 on/off (humanoid) | 공통 |
| T | 텍스처 on/off | teapot |
| L | 조명 on/off | teapot |
| Q / A | 왼팔 각도 수동 증가/감소 | humanoid |
| E / D | 오른팔 각도 수동 증가/감소 | humanoid |
| Z / X | 왼다리 각도 수동 증가/감소 | humanoid |
| C / V | 오른다리 각도 수동 증가/감소 | humanoid |
| R | 각도 초기화 | humanoid |
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

(`LeftLeg`/`RightLeg`는 현재 `humanoid_test.c`(하드코딩 버전)에만 반영되어
있고, `gen_humanoid_assets.c`/`humanoid_loader_test.c`(OBJ 로더 버전)는
아직 Body/Head/양팔까지만 생성한다.)

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
# → body.obj, head.obj, arm.obj, humanoid_scene.txt 생성

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
```

`Head`/`LeftArm`/`RightArm`의 `parent`가 전부 `Body`로 적혀 있는 것이
바로 이 수직 연계를 데이터로 표현한 부분이다.

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
