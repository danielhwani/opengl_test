"""
teapot_test.py
--------------
glBegin(GL_TRIANGLES) / glVertex3d / glNormal3d / glTexCoord2d 패턴을 테스트하기 위한
간단한 PyOpenGL 예제입니다.

실행 방법 (Windows PowerShell):
    pip install PyOpenGL PyOpenGL_accelerate
    python teapot_test.py

주전자(Utah teapot) 자체는 수백 개의 베지어 패치로 이루어져 있어서
정점을 직접 손으로 나열하기는 비현실적입니다. 그래서 GLUT이 내장 제공하는
glutSolidTeapot()을 사용합니다 - 이 함수 내부가 바로

    glBegin(GL_TRIANGLES)
        glNormal3d(nx, ny, nz)
        glTexCoord2d(u, v)
        glVertex3d(x, y, z)
    glEnd()

패턴으로 구현되어 있습니다. 조명(Normal 확인용)과 와이어프레임 토글로
Normal / TexCoord가 실제로 적용되는지 눈으로 확인할 수 있게 했습니다.

[텍스처 매핑 관련 참고]
glutSolidTeapot()은 정점(Vertex)과 법선(Normal)만 내부에서 생성하고
텍스처 좌표(glTexCoord2d)는 만들어주지 않습니다. 원래대로라면
glBegin(GL_TRIANGLES) 루프 안에서 각 정점마다 glTexCoord2d(u, v)를
직접 지정해야 하지만, 주전자의 원본 메시 데이터에는 접근할 수 없으므로
여기서는 OpenGL의 "텍스처 좌표 자동 생성(GL_TEXTURE_GEN_S/T)" 기능을
사용합니다. 이 기능은 GPU가 각 정점을 그릴 때 내부적으로
glTexCoord2d를 자동으로 호출해주는 것과 동일한 효과를 냅니다.

조작:
    스페이스바 : 와이어프레임 <-> 솔리드 토글
    T         : 텍스처 on/off
    L         : 조명 on/off
    ESC       : 종료
"""

import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

# ---- 상태값 ----
angle = 0.0
wireframe = False
lighting = True
texturing = True
texture_id = None


def init_gl():
    glEnable(GL_DEPTH_TEST)
    glClearColor(0.08, 0.08, 0.10, 1.0)

    # 조명 설정 (Normal이 올바르게 계산되는지 확인하기 위함)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glLightfv(GL_LIGHT0, GL_POSITION, [2.0, 3.0, 4.0, 1.0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE, [1.0, 1.0, 1.0, 1.0])
    glLightfv(GL_LIGHT0, GL_AMBIENT, [0.15, 0.15, 0.15, 1.0])

    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
    glColor3f(0.7, 0.75, 0.85)

    glEnable(GL_NORMALIZE)  # 스케일 변환 시 Normal 길이 보정

    create_checker_texture()

    # 텍스처 좌표 자동 생성 설정: 오브젝트 좌표(x, y)를 그대로 (u, v)로 매핑
    glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR)
    glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR)
    glTexGenfv(GL_S, GL_OBJECT_PLANE, [1.0, 0.0, 0.0, 0.0])
    glTexGenfv(GL_T, GL_OBJECT_PLANE, [0.0, 0.0, 1.0, 0.0])
    glEnable(GL_TEXTURE_GEN_S)
    glEnable(GL_TEXTURE_GEN_T)

    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE)


def create_checker_texture(size=64, squares=8):
    """8x8 체커보드 텍스처를 절차적으로 생성해서 GPU에 업로드"""
    global texture_id
    checker = np.zeros((size, size, 3), dtype=np.uint8)
    step = size // squares
    for i in range(size):
        for j in range(size):
            if (i // step + j // step) % 2 == 0:
                checker[i, j] = (230, 230, 230)
            else:
                checker[i, j] = (40, 60, 120)

    texture_id = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, texture_id)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, size, size, 0,
                 GL_RGB, GL_UNSIGNED_BYTE, checker)


def draw_axes(length=1.5):
    """xyz 축을 직접 glBegin(GL_LINES)로 그려서 좌표계 확인"""
    glDisable(GL_LIGHTING)
    glBegin(GL_LINES)
    glColor3f(1, 0, 0); glVertex3d(0, 0, 0); glVertex3d(length, 0, 0)
    glColor3f(0, 1, 0); glVertex3d(0, 0, 0); glVertex3d(0, length, 0)
    glColor3f(0, 0, 1); glVertex3d(0, 0, 0); glVertex3d(0, 0, length)
    glEnd()
    if lighting:
        glEnable(GL_LIGHTING)
    glColor3f(0.7, 0.75, 0.85)


def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    gluLookAt(3.5, 2.5, 3.5, 0, 0, 0, 0, 1, 0)

    draw_axes()

    glPushMatrix()
    glRotated(angle, 0, 1, 0)

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE if wireframe else GL_FILL)

    if texturing and not wireframe:
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, texture_id)
    else:
        glDisable(GL_TEXTURE_2D)

    # ------------------------------------------------------------------
    # 여기가 실질적으로 glBegin(GL_TRIANGLES) / glVertex3d / glNormal3d /
    # glTexCoord2d 패턴이 내부적으로 실행되는 지점입니다.
    # (glutSolidTeapot이 정점마다 Normal을 함께 넘겨주고,
    #  glTexCoord2d는 위에서 설정한 GL_TEXTURE_GEN이 자동으로 대신 호출합니다)
    # ------------------------------------------------------------------
    glutSolidTeapot(1.0)

    glDisable(GL_TEXTURE_2D)

    glPopMatrix()
    glutSwapBuffers()


def idle():
    global angle
    angle += 0.05
    glutPostRedisplay()


def keyboard(key, x, y):
    global wireframe, lighting, texturing
    if key == b' ':
        wireframe = not wireframe
    elif key in (b'l', b'L'):
        lighting = not lighting
        if lighting:
            glEnable(GL_LIGHTING)
        else:
            glDisable(GL_LIGHTING)
    elif key in (b't', b'T'):
        texturing = not texturing
    elif key == b'\x1b':  # ESC
        glutLeaveMainLoop()
    glutPostRedisplay()


def reshape(w, h):
    h = max(h, 1)
    glViewport(0, 0, w, h)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, w / h, 0.1, 50.0)
    glMatrixMode(GL_MODELVIEW)


def main():
    glutInit()
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(800, 600)
    glutCreateWindow(b"Teapot Test - glBegin/glVertex/glNormal/glTexCoord")

    init_gl()

    glutDisplayFunc(display)
    glutReshapeFunc(reshape)
    glutKeyboardFunc(keyboard)
    glutIdleFunc(idle)

    glutMainLoop()


if __name__ == "__main__":
    main()
