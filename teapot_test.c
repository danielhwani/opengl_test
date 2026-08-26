/*
 * teapot_test.c
 * -------------
 * teapot_test.py(PyOpenGL)를 그대로 C/freeglut로 포팅한 버전.
 * glBegin(GL_TRIANGLES)/glVertex3d/glNormal3d/glTexCoord2d 패턴은
 * glutSolidTeapot() 내부에서 동일하게 실행된다.
 *
 * 빌드 (MSYS2 ucrt64 gcc 사용):
 *   gcc teapot_test.c -o teapot_test.exe -lfreeglut -lopengl32 -lglu32 -lwinmm
 *
 * 조작:
 *   스페이스바 : 와이어프레임 <-> 솔리드 토글
 *   T          : 텍스처 on/off
 *   L          : 조명 on/off
 *   ESC        : 종료
 */

#include <GL/freeglut.h>
#include <stdlib.h>

static double angle = 0.0;
static int wireframe = 0;
static int lighting = 1;
static int texturing = 1;
static GLuint texture_id = 0;

#define TEX_SIZE 64
#define TEX_SQUARES 8

static void create_checker_texture(void) {
    static unsigned char checker[TEX_SIZE][TEX_SIZE][3];
    int step = TEX_SIZE / TEX_SQUARES;
    for (int i = 0; i < TEX_SIZE; i++) {
        for (int j = 0; j < TEX_SIZE; j++) {
            if (((i / step) + (j / step)) % 2 == 0) {
                checker[i][j][0] = 230; checker[i][j][1] = 230; checker[i][j][2] = 230;
            } else {
                checker[i][j][0] = 40; checker[i][j][1] = 60; checker[i][j][2] = 120;
            }
        }
    }

    glGenTextures(1, &texture_id);
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, TEX_SIZE, TEX_SIZE, 0,
                 GL_RGB, GL_UNSIGNED_BYTE, checker);
}

static void init_gl(void) {
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.08f, 0.08f, 0.10f, 1.0f);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat light_pos[] = {2.0f, 3.0f, 4.0f, 1.0f};
    GLfloat light_diffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat light_ambient[] = {0.15f, 0.15f, 0.15f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);

    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glColor3f(0.7f, 0.75f, 0.85f);

    glEnable(GL_NORMALIZE);

    create_checker_texture();

    glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
    glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
    GLfloat plane_s[] = {1.0f, 0.0f, 0.0f, 0.0f};
    GLfloat plane_t[] = {0.0f, 0.0f, 1.0f, 0.0f};
    glTexGenfv(GL_S, GL_OBJECT_PLANE, plane_s);
    glTexGenfv(GL_T, GL_OBJECT_PLANE, plane_t);
    glEnable(GL_TEXTURE_GEN_S);
    glEnable(GL_TEXTURE_GEN_T);

    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
}

static void draw_axes(double length) {
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
        glColor3f(1, 0, 0); glVertex3d(0, 0, 0); glVertex3d(length, 0, 0);
        glColor3f(0, 1, 0); glVertex3d(0, 0, 0); glVertex3d(0, length, 0);
        glColor3f(0, 0, 1); glVertex3d(0, 0, 0); glVertex3d(0, 0, length);
    glEnd();
    if (lighting) glEnable(GL_LIGHTING);
    glColor3f(0.7f, 0.75f, 0.85f);
}

static void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(3.5, 2.5, 3.5, 0, 0, 0, 0, 1, 0);

    draw_axes(1.5);

    glPushMatrix();
    glRotated(angle, 0, 1, 0);

    glPolygonMode(GL_FRONT_AND_BACK, wireframe ? GL_LINE : GL_FILL);

    if (texturing && !wireframe) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, texture_id);
    } else {
        glDisable(GL_TEXTURE_2D);
    }

    /* glutSolidTeapot 내부에서 glBegin(GL_TRIANGLES)/glNormal3d/
       glTexCoord2d(GL_TEXTURE_GEN으로 자동)/glVertex3d 패턴이 실행됨 */
    glutSolidTeapot(1.0);

    glDisable(GL_TEXTURE_2D);

    glPopMatrix();
    glutSwapBuffers();
}

static void idle(void) {
    angle += 0.01;
    glutPostRedisplay();
}

static void keyboard(unsigned char key, int x, int y) {
    (void)x; (void)y;
    switch (key) {
        case ' ':
            wireframe = !wireframe;
            break;
        case 'l': case 'L':
            lighting = !lighting;
            if (lighting) glEnable(GL_LIGHTING); else glDisable(GL_LIGHTING);
            break;
        case 't': case 'T':
            texturing = !texturing;
            break;
        case 27: /* ESC */
            glutLeaveMainLoop();
            break;
    }
    glutPostRedisplay();
}

static void reshape(int w, int h) {
    if (h < 1) h = 1;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)w / (double)h, 0.1, 50.0);
    glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Teapot Test (C) - glBegin/glVertex/glNormal/glTexCoord");

    init_gl();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutIdleFunc(idle);

    glutMainLoop();
    return 0;
}
