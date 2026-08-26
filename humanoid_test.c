/*
 * humanoid_test.c
 * ---------------
 * glPushMatrix/glPopMatrix로 계층적 모델링(hierarchical modeling)을
 * 테스트하는 예제. 몸통(Body) 위에 머리(Head)와 양팔(Left/Right arm)을
 * 붙이고, 스택을 이용해 각 부위의 변환을 독립적으로 유지한다.
 *
 * 행렬 스택 구조:
 *   glPushMatrix();
 *     glTranslated(...);         // body: T1
 *     draw(Body);
 *
 *     glPushMatrix();
 *       glTranslated(...);       // head: T1*T2
 *       draw(Head);
 *
 *       glPushMatrix();
 *         glRotated(...);        // left_arm: T1*T2*R1
 *         draw(Left arm);
 *       glPopMatrix();           // <- T1*T2로 복귀 (R1 제거)
 *
 *       glPushMatrix();
 *         glRotated(...);        // right_arm: T1*T2*R2 (R1 영향 없음)
 *         draw(Right arm);
 *       glPopMatrix();           // <- T1*T2로 복귀 (R2 제거)
 *     glPopMatrix();             // <- T1로 복귀 (head 변환 제거)
 *   glPopMatrix();               // <- 원점으로 복귀 (body 변환 제거)
 *
 * 빌드 (MSYS2 ucrt64 gcc 사용):
 *   gcc humanoid_test.c -o humanoid_test.exe -lfreeglut -lopengl32 -lglu32 -lwinmm
 *
 * 조작:
 *   스페이스바 : 자동 애니메이션 on/off
 *   Q / A      : 왼팔 각도 직접 조정 (자동 애니메이션은 자동으로 꺼짐)
 *   E / D      : 오른팔 각도 직접 조정
 *   R          : 각도 초기화
 *   ESC        : 종료
 */

#include <GL/freeglut.h>
#include <math.h>

/* ---- 사람 형태 치수 ---- */
#define BODY_W 0.8
#define BODY_H 1.4
#define BODY_D 0.5

#define HEAD_R 0.35

#define ARM_W 0.25
#define ARM_LEN 1.0
#define ARM_D 0.25
#define SHOULDER_X (BODY_W / 2.0 + ARM_W / 2.0)

/* ---- 상태값 ---- */
static double t = 0.0;
static double left_angle = 0.0;
static double right_angle = 0.0;
static int animate = 1;

static void init_gl(void) {
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.10f, 0.10f, 0.12f, 1.0f);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat light_pos[] = {3.0f, 4.0f, 5.0f, 1.0f};
    GLfloat light_diffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat light_ambient[] = {0.2f, 0.2f, 0.2f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);

    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_NORMALIZE);
}

static void draw_axes(double length) {
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
        glColor3f(1, 0, 0); glVertex3d(0, 0, 0); glVertex3d(length, 0, 0);
        glColor3f(0, 1, 0); glVertex3d(0, 0, 0); glVertex3d(0, length, 0);
        glColor3f(0, 0, 1); glVertex3d(0, 0, 0); glVertex3d(0, 0, length);
    glEnd();
    glEnable(GL_LIGHTING);
}

/* 몸통: 현재 원점을 중심으로 한 상자 */
static void draw_body(void) {
    glPushMatrix();
    glColor3f(0.55f, 0.55f, 0.65f);
    glScaled(BODY_W, BODY_H, BODY_D);
    glutSolidCube(1.0);
    glPopMatrix();
}

/* 머리: 목 위치(현재 원점)에서 반지름만큼 위로 띄운 구 */
static void draw_head(void) {
    glPushMatrix();
    glColor3f(0.85f, 0.75f, 0.65f);
    glTranslated(0.0, HEAD_R, 0.0);
    glutSolidSphere(HEAD_R, 24, 24);
    glPopMatrix();
}

/* 팔: 어깨(현재 원점)에서 옆으로 벌리고 아래로 매달리는 상자.
   여기서 하는 이동(옆으로 SHOULDER_X)은 x축이고, 회전(R1/R2)도
   x축 기준이라 서로 간섭하지 않는다 - 그래서 바깥에서 회전을
   먼저 적용해도 어깨 지점이 정확히 회전축이 된다. */
static void draw_arm(double side_x) {
    glPushMatrix();
    glColor3f(0.45f, 0.45f, 0.55f);
    glTranslated(side_x, -ARM_LEN / 2.0, 0.0);
    glScaled(ARM_W, ARM_LEN, ARM_D);
    glutSolidCube(1.0);
    glPopMatrix();
}

static void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(4.0, 2.2, 4.0, 0.0, 0.9, 0.0, 0.0, 1.0, 0.0);

    draw_axes(1.0);

    glPushMatrix();
        glTranslated(0.0, BODY_H / 2.0, 0.0);      /* body: T1 */
        draw_body();

        glPushMatrix();
            glTranslated(0.0, BODY_H / 2.0, 0.0);  /* head: T1*T2 */
            draw_head();

            glPushMatrix();
                glRotated(left_angle, 1, 0, 0);    /* left_arm: T1*T2*R1 */
                draw_arm(-SHOULDER_X);
            glPopMatrix();                          /* -> T1*T2 복귀 (R1 제거) */

            glPushMatrix();
                glRotated(right_angle, 1, 0, 0);   /* right_arm: T1*T2*R2 (R1 영향 없음) */
                draw_arm(SHOULDER_X);
            glPopMatrix();                          /* -> T1*T2 복귀 (R2 제거) */
        glPopMatrix();                              /* -> T1 복귀 (head 변환 제거) */
    glPopMatrix();                                   /* -> 원점 복귀 (body 변환 제거) */

    glutSwapBuffers();
}

static void idle(void) {
    if (animate) {
        t += 0.004;
        left_angle = 35.0 * sin(t);
        right_angle = -35.0 * sin(t);   /* 왼팔과 반대 위상 - 걷는 팔처럼 */
    }
    glutPostRedisplay();
}

static void keyboard(unsigned char key, int x, int y) {
    (void)x; (void)y;
    switch (key) {
        case ' ':
            animate = !animate;
            break;
        case 'q': case 'Q':
            animate = 0; left_angle += 5.0;
            break;
        case 'a': case 'A':
            animate = 0; left_angle -= 5.0;
            break;
        case 'e': case 'E':
            animate = 0; right_angle += 5.0;
            break;
        case 'd': case 'D':
            animate = 0; right_angle -= 5.0;
            break;
        case 'r': case 'R':
            left_angle = 0.0; right_angle = 0.0; t = 0.0;
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
    glutCreateWindow("Humanoid Matrix Stack Test - Body/Head/Left arm/Right arm");

    init_gl();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutIdleFunc(idle);

    glutMainLoop();
    return 0;
}
