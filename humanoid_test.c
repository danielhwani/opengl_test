/*
 * humanoid_test.c
 * ---------------
 * glPushMatrix/glPopMatrix로 계층적 모델링(hierarchical modeling)을
 * 테스트하는 예제. 몸통(Body) 위에 머리(Head)와 양팔(Left/Right arm),
 * 아래에 양다리(Left/Right leg)를 붙이고, 스택을 이용해 각 부위의
 * 변환을 독립적으로 유지한다.
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
 *   Z / X      : 왼다리 각도 직접 조정
 *   C / V      : 오른다리 각도 직접 조정
 *   T          : 텍스처 on/off
 *   L          : 조명 on/off
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

#define LEG_W 0.3
#define LEG_LEN 1.2
#define LEG_D 0.3
#define HIP_X ((BODY_W - LEG_W) / 2.0)

/* ---- 상태값 ---- */
static double t = 0.0;
static double left_angle = 0.0;
static double right_angle = 0.0;
static double left_leg_angle = 0.0;
static double right_leg_angle = 0.0;
static int animate = 1;
static int lighting = 1;
static int texturing = 1;
static GLuint texture_id = 0;

/* teapot_test.c와 동일한 체커 텍스처 - object-linear GL_TEXTURE_GEN을
   쓰므로 glutSolidCube/glutSolidSphere처럼 텍스처 좌표를 안 만들어주는
   도형에도 텍스처 좌표가 자동으로 생긴다. */
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

/* 다리: 엉덩이(현재 원점)에서 아래로 매달리는 상자. 팔과 같은 원리로
   x축 회전이 그대로 엉덩이 관절 회전이 된다. */
static void draw_leg(double side_x) {
    glPushMatrix();
    glColor3f(0.35f, 0.35f, 0.45f);
    glTranslated(side_x, -LEG_LEN / 2.0, 0.0);
    glScaled(LEG_W, LEG_LEN, LEG_D);
    glutSolidCube(1.0);
    glPopMatrix();
}

static void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(4.0, 2.2, 4.0, 0.0, 0.9, 0.0, 0.0, 1.0, 0.0);

    draw_axes(1.0);

    if (texturing) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, texture_id);
    } else {
        glDisable(GL_TEXTURE_2D);
    }

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

        glPushMatrix();
            glTranslated(0.0, -BODY_H / 2.0, 0.0); /* legs: T1*T3 (T3 = 몸통 바닥) */

            glPushMatrix();
                glRotated(left_leg_angle, 1, 0, 0);  /* left_leg: T1*T3*R3 */
                draw_leg(-HIP_X);
            glPopMatrix();                            /* -> T1*T3 복귀 (R3 제거) */

            glPushMatrix();
                glRotated(right_leg_angle, 1, 0, 0); /* right_leg: T1*T3*R4 (R3 영향 없음) */
                draw_leg(HIP_X);
            glPopMatrix();                            /* -> T1*T3 복귀 (R4 제거) */
        glPopMatrix();                                /* -> T1 복귀 (legs 변환 제거) */
    glPopMatrix();                                   /* -> 원점 복귀 (body 변환 제거) */

    glDisable(GL_TEXTURE_2D);

    glutSwapBuffers();
}

static void idle(void) {
    if (animate) {
        t += 0.004;
        left_angle = 35.0 * sin(t);
        right_angle = -35.0 * sin(t);   /* 왼팔과 반대 위상 - 걷는 팔처럼 */
        /* 걷는 동작: 왼팔과 같은 쪽 다리가 아니라 반대쪽 팔과 같은 위상으로
           움직여야 자연스럽다 (오른팔이 앞으로 나올 때 왼다리가 앞으로). */
        left_leg_angle = -25.0 * sin(t);
        right_leg_angle = 25.0 * sin(t);
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
        case 'z': case 'Z':
            animate = 0; left_leg_angle += 5.0;
            break;
        case 'x': case 'X':
            animate = 0; left_leg_angle -= 5.0;
            break;
        case 'c': case 'C':
            animate = 0; right_leg_angle += 5.0;
            break;
        case 'v': case 'V':
            animate = 0; right_leg_angle -= 5.0;
            break;
        case 'r': case 'R':
            left_angle = 0.0; right_angle = 0.0;
            left_leg_angle = 0.0; right_leg_angle = 0.0;
            t = 0.0;
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
    glutCreateWindow("Humanoid Matrix Stack Test - Body/Head/Arms/Legs");

    init_gl();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutIdleFunc(idle);

    glutMainLoop();
    return 0;
}
