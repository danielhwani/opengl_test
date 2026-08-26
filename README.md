# Teapot OpenGL Test

`glBegin(GL_TRIANGLES)` / `glVertex3d` / `glNormal3d` / `glTexCoord2d` 패턴을
`glutSolidTeapot()`을 통해 확인해보는 테스트. 동일한 기능을 Python(PyOpenGL)과
C(freeglut) 양쪽으로 구현해뒀다.

## 파일

- `teapot_test.py` — PyOpenGL 버전
- `teapot_test.c` — C/freeglut 포팅 버전

## 조작

| 키 | 기능 |
|---|---|
| Space | 와이어프레임 / 솔리드 토글 |
| T | 텍스처 on/off |
| L | 조명 on/off |
| ESC | 종료 |

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
cd /c/claude_test/computer_graphics
gcc teapot_test.c -o teapot_test.exe -lfreeglut -lopengl32 -lglu32 -lwinmm
./teapot_test.exe
```

**cmd.exe**
```cmd
set PATH=D:\OpenModelica1.26.3-64bit\tools\msys\ucrt64\bin;%PATH%
cd C:\claude_test\computer_graphics
gcc teapot_test.c -o teapot_test.exe -lfreeglut -lopengl32 -lglu32 -lwinmm
teapot_test.exe
```

**PowerShell**
```powershell
$env:PATH = "D:\OpenModelica1.26.3-64bit\tools\msys\ucrt64\bin;" + $env:PATH
cd C:\claude_test\computer_graphics
gcc teapot_test.c -o teapot_test.exe -lfreeglut -lopengl32 -lglu32 -lwinmm
.\teapot_test.exe
```

위 PATH 설정은 세션 한정이다. 매번 설정하기 번거로우면 Windows 시스템
환경변수 PATH에 해당 경로를 영구로 추가해두면 된다.

`teapot_test.exe`를 그 bin 폴더 밖에서 단독으로 배포/실행하려면
`libfreeglut.dll`, `libgcc_s_seh-1.dll`, `libwinpthread-1.dll`을
exe와 같은 폴더로 복사해야 "DLL을 찾을 수 없습니다" 오류가 나지 않는다.

## 구현 메모

- `glutSolidTeapot()`은 정점(Vertex)/법선(Normal)만 생성하고 텍스처 좌표는
  만들어주지 않는다. 원본 메시 데이터에 접근할 수 없어 텍스처 좌표는
  `GL_TEXTURE_GEN_S/T` (object-linear)로 자동 생성한다.
- 회전 속도는 `idle()`에서 프레임마다 `angle`에 더해지는 값으로 조절한다
  (C 버전 현재 `0.01`, Python 버전 `0.05`).
