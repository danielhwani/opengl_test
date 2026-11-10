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
 *   방향키 위/아래 : 몸통 전진/후진 (자동 애니메이션은 자동으로 꺼짐)
 *   방향키 좌/우  : 몸통 좌/우 회전
 *
 * 자동 애니메이션일 때 몸통은 humanoid_path.txt에 적힌 웨이포인트
 * ("x z" 한 줄씩, 마지막 점에서 첫 점으로 돌아와 폐곡선을 이룸)를
 * 순서대로 따라 걷는다. 파일이 없으면 기본 사각형 경로를 쓴다.
 *   마우스 왼쪽 드래그 : 카메라 궤도 회전
 *   마우스 휠        : 줌 인/아웃
 *   R          : 각도 초기화
 *   ESC        : 종료
 */

#include <GL/freeglut.h>
#include <math.h>
#include <stdio.h>

#define CAM_PI 3.14159265358979323846

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
/* 몸통 전체의 월드 위치/방향 - 팔다리와 달리 "관절"이 아니라 트리
   전체(ROOT)에 적용되는 변환이라 다른 모든 부위를 다 끌고 다닌다. */
static double body_x = 0.0;
static double body_z = 0.0;
static double body_heading = 0.0; /* degrees, y축 둘레 회전 */
static int animate = 1;

/* ---- 경로(path): humanoid_path.txt에서 읽은 웨이포인트(x z)를 순서대로
   잇는 폐곡선을 따라 걷는다. 파일이 없으면 기본 사각형 경로를 쓴다. ---- */
#define MAX_WAYPOINTS 64
static double path_x[MAX_WAYPOINTS];
static double path_z[MAX_WAYPOINTS];
static double path_seg_len[MAX_WAYPOINTS]; /* i번 구간: path[i] -> path[(i+1)%path_count] */
static int path_count = 0;
static double path_total_len = 0.0;

static void load_default_path(void) {
    static const double defx[] = {-2.0, 2.0, 2.0, -2.0};
    static const double defz[] = {-1.5, -1.5, 1.5, 1.5};
    path_count = 4;
    for (int i = 0; i < path_count; i++) { path_x[i] = defx[i]; path_z[i] = defz[i]; }
}

static void load_path(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (!f) {
        fprintf(stderr, "경로 파일을 열 수 없어 기본 사각형 경로를 사용합니다: %s\n", filename);
        load_default_path();
    } else {
        path_count = 0;
        char line[128];
        while (fgets(line, sizeof(line), f) && path_count < MAX_WAYPOINTS) {
            if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
            double x, z;
            if (sscanf(line, "%lf %lf", &x, &z) == 2) {
                path_x[path_count] = x;
                path_z[path_count] = z;
                path_count++;
            }
        }
        fclose(f);
        if (path_count < 2) {
            fprintf(stderr, "경로 파일에 점이 2개 미만입니다. 기본 사각형 경로를 사용합니다.\n");
            load_default_path();
        }
    }

    /* 마지막 점 -> 첫 점으로 돌아오는 구간까지 포함해 폐곡선으로 닫는다 */
    path_total_len = 0.0;
    for (int i = 0; i < path_count; i++) {
        int j = (i + 1) % path_count;
        double dx = path_x[j] - path_x[i];
        double dz = path_z[j] - path_z[i];
        path_seg_len[i] = sqrt(dx * dx + dz * dz);
        path_total_len += path_seg_len[i];
    }
}

/* Catmull-Rom 스플라인: 직선 보간(꼭짓점마다 방향이 뚝뚝 꺾임) 대신
   앞뒤 두 점(p0,p3)까지 참고해 부드러운 곡선을 그린다. frac=0에서 p1,
   frac=1에서 p2를 지나며, 접선(out_tx/out_tz)이 곧 그 지점에서의
   진행 방향(heading)이 되어 꼭짓점에서도 방향이 급격히 꺾이지 않는다. */
static void catmull_rom(double p0x, double p0z, double p1x, double p1z,
                         double p2x, double p2z, double p3x, double p3z,
                         double frac, double *out_x, double *out_z,
                         double *out_tx, double *out_tz) {
    double t2 = frac * frac, t3 = t2 * frac;
    *out_x = 0.5 * ((2.0 * p1x) + (-p0x + p2x) * frac
             + (2.0 * p0x - 5.0 * p1x + 4.0 * p2x - p3x) * t2
             + (-p0x + 3.0 * p1x - 3.0 * p2x + p3x) * t3);
    *out_z = 0.5 * ((2.0 * p1z) + (-p0z + p2z) * frac
             + (2.0 * p0z - 5.0 * p1z + 4.0 * p2z - p3z) * t2
             + (-p0z + 3.0 * p1z - 3.0 * p2z + p3z) * t3);
    *out_tx = 0.5 * ((-p0x + p2x) + 2.0 * (2.0 * p0x - 5.0 * p1x + 4.0 * p2x - p3x) * frac
              + 3.0 * (-p0x + 3.0 * p1x - 3.0 * p2x + p3x) * t2);
    *out_tz = 0.5 * ((-p0z + p2z) + 2.0 * (2.0 * p0z - 5.0 * p1z + 4.0 * p2z - p3z) * frac
              + 3.0 * (-p0z + 3.0 * p1z - 3.0 * p2z + p3z) * t2);
}

/* 경로를 따라 s만큼 이동한 지점의 위치/방향을 계산한다 (s는 경로
   시작점부터의 누적 거리, path_total_len을 넘으면 처음부터 다시 돈다).
   어느 구간(i)에 있는지/그 구간 안에서 얼마나 왔는지(frac)는 기존처럼
   직선 길이표로 찾고, 실제 위치/방향은 그 구간의 앞뒤 점까지 포함한
   Catmull-Rom 스플라인으로 계산해 꼭짓점에서 경로가 들쭉날쭉 꺾이지
   않고 부드럽게 이어지게 한다. */
static void path_sample(double s, double *out_x, double *out_z, double *out_heading_deg) {
    if (path_count < 2 || path_total_len <= 0.0) {
        *out_x = 0.0; *out_z = 0.0; *out_heading_deg = 0.0;
        return;
    }
    s = fmod(s, path_total_len);
    if (s < 0.0) s += path_total_len;

    int i = 0;
    while (i < path_count - 1 && s > path_seg_len[i]) {
        s -= path_seg_len[i];
        i++;
    }

    int im1 = (i - 1 + path_count) % path_count;
    int j = (i + 1) % path_count;
    int jp1 = (i + 2) % path_count;
    double seg = path_seg_len[i];
    double frac = (seg > 1e-9) ? (s / seg) : 0.0;

    double px, pz, tx, tz;
    catmull_rom(path_x[im1], path_z[im1], path_x[i], path_z[i],
                path_x[j], path_z[j], path_x[jp1], path_z[jp1],
                frac, &px, &pz, &tx, &tz);

    *out_x = px;
    *out_z = pz;
    if (fabs(tx) > 1e-9 || fabs(tz) > 1e-9) {
        *out_heading_deg = atan2(tx, tz) * 180.0 / CAM_PI;
    } else {
        double dx = path_x[j] - path_x[i];
        double dz = path_z[j] - path_z[i];
        *out_heading_deg = atan2(dx, dz) * 180.0 / CAM_PI;
    }
}
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

/* ---- 사람 몸에 입힐 좀 더 그럴듯한 텍스처 3종 (피부/상의/하의) ----
   체커 패턴 하나로 모든 부위를 덮던 것 대신, 부위별로 다른 텍스처를
   입혀 사람처럼 보이게 한다. 진짜 이미지 파일 없이, 픽셀 배열을
   직접 채워서 절차적으로(procedurally) 만든다. */
#define BODY_TEX_SIZE 64

static GLuint upload_body_texture(unsigned char *pixels) {
    GLuint id;
    glGenTextures(1, &id);
    glBindTexture(GL_TEXTURE_2D, id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, BODY_TEX_SIZE, BODY_TEX_SIZE, 0,
                 GL_RGB, GL_UNSIGNED_BYTE, pixels);
    return id;
}

static GLuint skin_texture_id = 0;
static GLuint shirt_texture_id = 0;
static GLuint pants_texture_id = 0;

/* 피부: 살색 단색. 처음엔 (i*7+j*13)%17 같은 좌표 기반 잡티를 섞어
   봤는데, 그 모듈러 연산 패턴이 일정한 간격으로 반복되면서(특히
   작은 텍스처 좌표 범위에 겹쳐 보일 때) 체커 패턴과 다를 바 없는
   격자 무늬처럼 보여 오히려 더 보기 나빴다. 그래서 잡티 없는
   깔끔한 단색으로 되돌렸다. */
static void create_skin_texture(void) {
    static unsigned char pix[BODY_TEX_SIZE][BODY_TEX_SIZE][3];
    for (int i = 0; i < BODY_TEX_SIZE; i++) {
        for (int j = 0; j < BODY_TEX_SIZE; j++) {
            pix[i][j][0] = 222; pix[i][j][1] = 180; pix[i][j][2] = 150;
        }
    }
    skin_texture_id = upload_body_texture(&pix[0][0][0]);
}

/* 상의: 가로 줄무늬 (셔츠처럼). i(세로 방향)를 8픽셀 단위로 묶어
   짙은/밝은 색을 번갈아 준다. */
static void create_shirt_texture(void) {
    static unsigned char pix[BODY_TEX_SIZE][BODY_TEX_SIZE][3];
    for (int i = 0; i < BODY_TEX_SIZE; i++) {
        int stripe = (i / 8) % 2;
        for (int j = 0; j < BODY_TEX_SIZE; j++) {
            if (stripe == 0) {
                pix[i][j][0] = 50; pix[i][j][1] = 80; pix[i][j][2] = 150;
            } else {
                pix[i][j][0] = 215; pix[i][j][1] = 220; pix[i][j][2] = 235;
            }
        }
    }
    shirt_texture_id = upload_body_texture(&pix[0][0][0]);
}

/* 하의: 데님 느낌의 짙은 청색 단색. skin과 같은 이유로 잡티 노이즈를
   빼고 깔끔한 단색만 쓴다. */
static void create_pants_texture(void) {
    static unsigned char pix[BODY_TEX_SIZE][BODY_TEX_SIZE][3];
    for (int i = 0; i < BODY_TEX_SIZE; i++) {
        for (int j = 0; j < BODY_TEX_SIZE; j++) {
            pix[i][j][0] = 60; pix[i][j][1] = 80; pix[i][j][2] = 120;
        }
    }
    pants_texture_id = upload_body_texture(&pix[0][0][0]);
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
    create_skin_texture();
    create_shirt_texture();
    create_pants_texture();

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

/* ---- 카메라: (0, CAM_TARGET_Y, 0)을 바라보는 궤도(orbit) 카메라.
   타겟을 몸통 위치에 맞춰 따라가게 하면 몸통이 실제로 이동해도 화면상
   으로는 항상 중앙에 고정돼 보여서(이동이 카메라로 상쇄됨) 제자리
   회전처럼 보인다. 걷는 원의 반지름이 크지 않아 화면 밖으로 나갈
   일이 없으므로, 타겟은 원점에 고정해 실제 이동이 보이게 한다.
   yaw/pitch/dist(구면 좌표)를 마우스로 바꾸고 매 프레임 직교 좌표로
   환산해 gluLookAt에 넘긴다. ---- */
static double cam_yaw = 45.0;    /* degrees, y축 둘레 회전 */
static double cam_pitch = 21.0;  /* degrees, -89..89로 클램프 (gimbal lock 방지) */
static double cam_dist = 10.0;
#define CAM_TARGET_Y 0.9
static int mouse_dragging = 0;
static int last_mouse_x = 0, last_mouse_y = 0;

static void apply_camera(void) {
    double yaw_r = cam_yaw * CAM_PI / 180.0;
    double pitch_r = cam_pitch * CAM_PI / 180.0;
    double ex = cam_dist * cos(pitch_r) * sin(yaw_r);
    double ey = CAM_TARGET_Y + cam_dist * sin(pitch_r);
    double ez = cam_dist * cos(pitch_r) * cos(yaw_r);
    gluLookAt(ex, ey, ez, 0.0, CAM_TARGET_Y, 0.0, 0.0, 1.0, 0.0);
}

static void mouse_button(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        mouse_dragging = (state == GLUT_DOWN);
        last_mouse_x = x; last_mouse_y = y;
    }
}

static void mouse_motion(int x, int y) {
    if (!mouse_dragging) return;
    int dx = x - last_mouse_x;
    int dy = y - last_mouse_y;
    last_mouse_x = x; last_mouse_y = y;

    cam_yaw += dx * 0.4;
    cam_pitch += dy * 0.4;
    if (cam_pitch > 89.0) cam_pitch = 89.0;
    if (cam_pitch < -89.0) cam_pitch = -89.0;

    glutPostRedisplay();
}

static void mouse_wheel(int wheel, int direction, int x, int y) {
    (void)wheel; (void)x; (void)y;
    cam_dist -= direction * 0.4;
    if (cam_dist < 1.5) cam_dist = 1.5;
    if (cam_dist > 20.0) cam_dist = 20.0;
    glutPostRedisplay();
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

/* 그림자를 그리는 동안(rendering_shadow)은 각 부위가 자기 색을 칠하면
   안 된다 - 그림자는 display()에서 미리 glColor4f로 설정해둔 반투명
   검은색을 그대로 유지해야 하기 때문. 조명이 꺼진 상태라 glColor는
   lighting 여부와 무관하게 바로 화면색이 되므로, 부위별 glColor3f를
   그냥 두면 그림자가 몸 색 그대로 투영돼버린다. */
static int rendering_shadow = 0;

/* 부위별 텍스처(피부/상의/하의)로 바꿔 칠하는 동안은 glColor를
   흰색으로 둬야 한다 - GL_MODULATE 모드라 최종 색 = 텍스처 색 *
   vertex 색인데, 기존의 칙칙한 회색/남색 vertex 색을 그대로 두면
   텍스처 색 위에 그 색이 덧씌워져 탁해진다. 텍스처가 꺼져 있을
   때는(tex_on == 0) 원래 쓰던 단색으로 그대로 돌아간다. */
static int tex_on(void) {
    return texturing && !rendering_shadow;
}

/* 여러 캐릭터를 동시에 그릴 때 옷(셔츠/바지) 색을 서로 다르게 보이게
   하는 틴트(tint). GL_MODULATE라 최종 색 = 텍스처 색 * vertex 색이므로,
   흰색(1,1,1) 대신 이 값을 곱하면 같은 셔츠/바지 텍스처가 물든 것처럼
   색만 바뀐다. draw_all_humanoids()가 캐릭터를 바꿔 그리기 전에 이
   값을 설정해둔다. 얼굴(피부)은 전부 흰색 그대로 둬서 옷만 구별되게
   한다. */
static float outfit_tint[3] = {1.0f, 1.0f, 1.0f};

/* 몸통: 현재 원점을 중심으로 한 상자. 상의(셔츠) 텍스처를 입힌다. */
static void draw_body(void) {
    glPushMatrix();
    if (tex_on()) {
        glBindTexture(GL_TEXTURE_2D, shirt_texture_id);
        if (!rendering_shadow) glColor3f(outfit_tint[0], outfit_tint[1], outfit_tint[2]);
    } else if (!rendering_shadow) {
        glColor3f(0.55f, 0.55f, 0.65f);
    }
    glScaled(BODY_W, BODY_H, BODY_D);
    glutSolidCube(1.0);
    glPopMatrix();
}

/* 머리: 목 위치(현재 원점)에서 반지름만큼 위로 띄운 구. 피부 텍스처를
   입힌다. +Z쪽에 작은 코를 붙여 앞/뒤를 구별한다 - heading=0일 때
   forward 벡터가 (sin(heading), cos(heading)) = (0, 1), 즉 로컬 +Z축
   이라 코도 그 축 위에 놓아야 실제로 "앞"을 가리킨다. 코는 피부
   텍스처 위에 순색 빨강이 또렷이 보이도록 텍스처를 잠깐 끈다. */
static void draw_head(void) {
    int t = tex_on();
    glPushMatrix();
    if (t) {
        glBindTexture(GL_TEXTURE_2D, skin_texture_id);
        if (!rendering_shadow) glColor3f(1.0f, 1.0f, 1.0f);
    } else if (!rendering_shadow) {
        glColor3f(0.85f, 0.75f, 0.65f);
    }
    glTranslated(0.0, HEAD_R, 0.0);
    glutSolidSphere(HEAD_R, 24, 24);

    if (t) glDisable(GL_TEXTURE_2D);
    if (!rendering_shadow) glColor3f(0.85f, 0.25f, 0.25f);
    glTranslated(0.0, 0.0, HEAD_R * 0.85);
    glutSolidSphere(HEAD_R * 0.28, 12, 12);
    if (t) glEnable(GL_TEXTURE_2D);

    glPopMatrix();
}

/* 팔: 어깨(현재 원점)에서 옆으로 벌리고 아래로 매달리는 상자. 긴팔
   소매라고 보고 상의와 같은 텍스처를 입힌다. 여기서 하는 이동(옆으로
   SHOULDER_X)은 x축이고, 회전(R1/R2)도 x축 기준이라 서로 간섭하지
   않는다 - 그래서 바깥에서 회전을 먼저 적용해도 어깨 지점이 정확히
   회전축이 된다. */
static void draw_arm(double side_x) {
    glPushMatrix();
    if (tex_on()) {
        glBindTexture(GL_TEXTURE_2D, shirt_texture_id);
        if (!rendering_shadow) glColor3f(outfit_tint[0], outfit_tint[1], outfit_tint[2]);
    } else if (!rendering_shadow) {
        glColor3f(0.45f, 0.45f, 0.55f);
    }
    glTranslated(side_x, -ARM_LEN / 2.0, 0.0);
    glScaled(ARM_W, ARM_LEN, ARM_D);
    glutSolidCube(1.0);
    glPopMatrix();
}

/* 다리: 엉덩이(현재 원점)에서 아래로 매달리는 상자. 바지(하의)
   텍스처를 입힌다. 팔과 같은 원리로 x축 회전이 그대로 엉덩이 관절
   회전이 된다. */
static void draw_leg(double side_x) {
    glPushMatrix();
    if (tex_on()) {
        glBindTexture(GL_TEXTURE_2D, pants_texture_id);
        if (!rendering_shadow) glColor3f(outfit_tint[0], outfit_tint[1], outfit_tint[2]);
    } else if (!rendering_shadow) {
        glColor3f(0.35f, 0.35f, 0.45f);
    }
    glTranslated(side_x, -LEG_LEN / 2.0, 0.0);
    glScaled(LEG_W, LEG_LEN, LEG_D);
    glutSolidCube(1.0);
    glPopMatrix();
}

/* 트리 전체(ROOT 변환 + body/head/arms/legs)를 그린다. display()에서
   분리해둔 이유는 그림자 렌더링에도 똑같이 재사용하기 위해서다 -
   그림자는 이 함수를 "광원 기준 y=0 투영 행렬"을 미리 곱해둔 상태로
   한 번 더 호출해서 만든다(아래 apply_shadow_matrix).
   ROOT의 월드 위치/방향을 인자로 받는다 - 여러 캐릭터를 같은 경로
   위 다른 지점에 동시에 그리기 위해서다 (팔다리 스윙은 전부 같은
   left_angle/right_angle 등 전역 상태를 공유해서 같은 박자로 움직
   인다). */
static void draw_humanoid_body(double x, double z, double heading) {
    /* ROOT 자체가 "엉덩이" 높이라서, y를 0으로 두면 다리(LEG_LEN)가
       그대로 땅(y=0) 밑으로 파고든다. 발이 땅에 닿도록 ROOT를
       LEG_LEN만큼 들어올린다. */
    glTranslated(x, LEG_LEN, z);   /* 몸통의 월드 위치 (ROOT) */
    glRotated(heading, 0, 1, 0);    /* 몸통의 방향 (ROOT) */

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
}

/* ---- 여러 캐릭터 동시 렌더링 ----
   주인공(body_x/body_z/body_heading, 방향키로 수동 조작 가능)과 똑같은
   humanoid_path.txt 경로를 걷는 "클론" 캐릭터를 몇 명 더 세워둔다.
   클론의 위치는 매 틱 독립적으로 계산하지 않고, path_sample()에 주는
   이동 거리(s)에 서로 다른 고정 오프셋을 더해서 구한다 - s가 경로
   "호의 길이"이므로, 오프셋을 path_total_len/인원수로 等分하면 경로
   모양과 무관하게 항상 고리 위에 고르게 퍼진 채로 같은 속도로
   따라 걷는다. 팔다리 스윙(left_angle 등)은 전부 공유 전역 상태라
   모든 캐릭터가 같은 박자로 움직인다 - 각자 다른 위상으로 걷게
   하려면 그 각도들도 인스턴스별로 따로 둬야 하지만, 지금은 "같은
   동작으로 경로를 도는 여러 캐릭터"만 보여주는 게 목적이라 단순하게
   위치만 instance별로 다르게 둔다. */
#define HUMANOID_CLONE_COUNT 2
static double clone_x[HUMANOID_CLONE_COUNT];
static double clone_z[HUMANOID_CLONE_COUNT];
static double clone_heading[HUMANOID_CLONE_COUNT];

/* 캐릭터별 옷 색 틴트 - [0]은 주인공, [1..]은 클론들. outfit_tint에
   대입해두면 draw_body/draw_arm/draw_leg가 셔츠/바지 텍스처에 이 색을
   곱해서 그린다 (위 outfit_tint 선언부 주석 참고). */
static const float outfit_tints[HUMANOID_CLONE_COUNT + 1][3] = {
    {1.00f, 1.00f, 1.00f}, /* 주인공: 원래 셔츠/바지 색 그대로 */
    {1.00f, 0.55f, 0.55f}, /* 클론 1: 붉은 톤 */
    {0.55f, 0.80f, 1.00f}, /* 클론 2: 푸른 톤 */
};

/* draw_trees()처럼 각 캐릭터의 ROOT 변환을 내부에서 push/pop으로
   감싸서, 호출하는 쪽에서 따로 감싸지 않아도 변환이 새지 않는다. */
static void draw_all_humanoids(void) {
    outfit_tint[0] = outfit_tints[0][0];
    outfit_tint[1] = outfit_tints[0][1];
    outfit_tint[2] = outfit_tints[0][2];
    glPushMatrix();
        draw_humanoid_body(body_x, body_z, body_heading);
    glPopMatrix();

    for (int i = 0; i < HUMANOID_CLONE_COUNT; i++) {
        outfit_tint[0] = outfit_tints[i + 1][0];
        outfit_tint[1] = outfit_tints[i + 1][1];
        outfit_tint[2] = outfit_tints[i + 1][2];
        glPushMatrix();
            draw_humanoid_body(clone_x[i], clone_z[i], clone_heading[i]);
        glPopMatrix();
    }
}

/* ---- 그림자: 평행광(directional light) 가정의 전단(shear) 투영 ----
   점광원 위치를 기준으로 진짜 투영(perspective divide, w가 0에 가까워짐)
   행렬을 쓰면 머리처럼 광원 높이(LIGHT_Y)에 가까워지는 부위의 그림자가
   무한히 늘어나면서 카메라 절두체 경계에서 잘려 보이는 문제가 있었다
   (화면 위쪽이 잘리는 현상). 대신 광원을 "방향"으로만 취급해 높이(y)에
   비례해 x/z를 미리 어긋나게 하는 전단 변환만 쓰면 w가 항상 1로
   유지되어 그런 클리핑이 생기지 않는다 - 점광원처럼 정확하진 않지만
   이 데모에서는 방향만 맞으면 충분하다. */
#define LIGHT_X 3.0
#define LIGHT_Y 4.0
#define LIGHT_Z 5.0

static void apply_shadow_matrix(double lx, double ly, double lz) {
    GLdouble m[16] = {
        1.0,     0.0, 0.0, 0.0,
        -lx/ly,  0.0, -lz/ly, 0.0,
        0.0,     0.0, 1.0, 0.0,
        0.0,     0.0, 0.0, 1.0
    };
    glMultMatrixd(m);
}

/* ---- 하늘: 화면 전체를 덮는 2D 사각형에 위/아래 색을 다르게 줘서
   생기는 그라디언트. 3D 장면을 그리기 전에, 깊이 테스트/조명을 끈
   채 별도의 직교(ortho) 투영으로 그리고 나서 원래 투영/모델뷰
   행렬로 복귀한다 - 그래야 뒤에 이어지는 3D 카메라 설정에 영향을
   주지 않는다. glClearColor 한 가지 색보다 하늘-지평선 그라디언트가
   훨씬 "배경"처럼 보인다. ---- */
static void draw_sky(void) {
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0, 0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glBegin(GL_QUADS);
        glColor3f(0.35f, 0.55f, 0.85f); glVertex2d(0.0, 1.0); /* 위: 하늘색 */
        glColor3f(0.35f, 0.55f, 0.85f); glVertex2d(1.0, 1.0);
        glColor3f(0.80f, 0.88f, 0.95f); glVertex2d(1.0, 0.0); /* 아래: 지평선 근처, 더 밝게 */
        glColor3f(0.80f, 0.88f, 0.95f); glVertex2d(0.0, 0.0);
    glEnd();

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    glEnable(GL_DEPTH_TEST);
    if (lighting) glEnable(GL_LIGHTING);
}

/* ---- 땅: 원점 근처를 덮는 큰 사각형(y=GROUND_Y, 거의 0이지만 살짝
   아래로 둬서 그림자가 투영되는 y=0 평면과 z-fighting이 안 나게
   한다). texturing이 켜져 있으면 체커 텍스처가 그대로 타일링돼서
   바닥 격자처럼 보인다(GL_TEXTURE_GEN이 물체 좌표 기준이라 큰
   사각형 위에서는 자동으로 여러 번 반복된다). ---- */
#define GROUND_HALF 20.0
#define GROUND_Y -0.01

static void draw_ground(void) {
    glColor3f(0.42f, 0.30f, 0.18f); /* 흙색 */
    glNormal3d(0.0, 1.0, 0.0);
    glBegin(GL_QUADS);
        glVertex3d(-GROUND_HALF, GROUND_Y, -GROUND_HALF);
        glVertex3d( GROUND_HALF, GROUND_Y, -GROUND_HALF);
        glVertex3d( GROUND_HALF, GROUND_Y,  GROUND_HALF);
        glVertex3d(-GROUND_HALF, GROUND_Y,  GROUND_HALF);
    glEnd();
}

/* ---- 나무: 배경을 덜 밋밋하게 만드는 정적(static) 소품.
   줄기(상자) + 크리스마스 트리처럼 아래로 갈수록 커지는 원뿔 3단을
   겹쳐 쌓은 잎. draw_body/draw_arm 등과 똑같이 rendering_shadow
   플래그로 그림자 패스에서는 색을 바꾸지 않는다. glutSolidCone은
   기본적으로 +Z 방향으로 뾰족해지므로, X축으로 -90도 돌려 +Y(위)
   방향을 향하게 한다. ---- */
static void draw_tree(double x, double z) {
    static const double tier_y[3]      = {1.00, 1.45, 1.85}; /* 아래->위 */
    static const double tier_radius[3] = {0.70, 0.52, 0.36};
    static const double tier_height[3] = {0.85, 0.75, 0.65};
    static const float tier_color[3][3] = {
        {0.12f, 0.36f, 0.15f}, /* 아래: 짙은 초록 */
        {0.15f, 0.44f, 0.18f},
        {0.20f, 0.52f, 0.22f}, /* 위: 밝은 초록 - 입체감 */
    };

    glPushMatrix();
    glTranslated(x, 0.0, z);

    glPushMatrix();
        if (!rendering_shadow) glColor3f(0.40f, 0.26f, 0.13f); /* 줄기: 나무껍질색 */
        glTranslated(0.0, 0.55, 0.0);
        glScaled(0.22, 1.1, 0.22);
        glutSolidCube(1.0);
    glPopMatrix();

    for (int i = 0; i < 3; i++) {
        glPushMatrix();
            if (!rendering_shadow) {
                glColor3f(tier_color[i][0], tier_color[i][1], tier_color[i][2]);
            }
            glTranslated(0.0, tier_y[i], 0.0);
            glRotated(-90.0, 1, 0, 0); /* 원뿔의 +Z축 -> +Y축(위) */
            glutSolidCone(tier_radius[i], tier_height[i], 16, 4);
        glPopMatrix();
    }

    /* 장식구: 각 원뿔 표면의 (높이 비율, 둘레 각도) 지점에 올려둔다.
       그 높이에서의 원뿔 반지름 r(h) = tier_radius*(1 - h/tier_height)
       (원뿔이라 선형으로 가늘어짐)에 약간의 여유(1.08배)를 둬서 잎
       속에 파묻히지 않고 표면 위로 살짝 튀어나오게 한다. */
    {
        static const int    orn_tier[]      = {0, 0, 0, 1, 1, 2, 2};
        static const double orn_frac[]      = {0.25, 0.55, 0.80, 0.30, 0.70, 0.25, 0.65};
        static const double orn_angle_deg[] = {20.0, 160.0, 280.0, 70.0, 250.0, 10.0, 190.0};
        const int orn_count = (int)(sizeof(orn_tier) / sizeof(orn_tier[0]));

        if (!rendering_shadow) glColor3f(0.85f, 0.08f, 0.08f); /* 빨간 장식구 */
        for (int k = 0; k < orn_count; k++) {
            int ti = orn_tier[k];
            double h = orn_frac[k] * tier_height[ti];
            double r = tier_radius[ti] * (1.0 - orn_frac[k]) * 1.08;
            double ang = orn_angle_deg[k] * CAM_PI / 180.0;

            glPushMatrix();
                glTranslated(r * cos(ang), tier_y[ti] + h, r * sin(ang));
                glutSolidSphere(0.09, 10, 10);
            glPopMatrix();
        }
    }

    glPopMatrix();
}

#define TREE_COUNT 5
static const double tree_x[TREE_COUNT] = {-7.0,  7.0, 7.0, -7.0, 0.0};
static const double tree_z[TREE_COUNT] = {-5.0, -5.0, 5.0,  5.0, 6.5};

static void draw_trees(void) {
    for (int i = 0; i < TREE_COUNT; i++) {
        draw_tree(tree_x[i], tree_z[i]);
    }
}

/* ---- 4족 보행 로봇: quadruped_test.c에서 독립 실행하던 데모를 이
   장면의 배경 소품으로 옮겨왔다. 몸통 + 머리 + 다리 4개로, humanoid와
   똑같은 matrix stack 기법을 쓴다. humanoid의 걷는 경로(사각형, z는
   -3.5..3.5)와 겹치지 않는 z=QUAD_POS_Z(=-9, 훨씬 바깥쪽)에 고정해두고,
   그 위에서 x축 방향으로만 왕복(patrol)한다 - 동선이 사람과 절대
   교차하지 않는다. 걸을 때는 트롯(trot) 걸음걸이로: 대각선으로
   마주보는 두 다리(앞왼쪽+뒤오른쪽, 앞오른쪽+뒤왼쪽)가 쌍을 이뤄
   서로 반대 위상으로 흔들린다. 몸 색은 땅/나무처럼 texturing 토글과
   분리된 단색이다. ---- */
#define QUAD_BODY_W 0.5
#define QUAD_BODY_L 1.2
#define QUAD_BODY_H 0.4
#define QUAD_HEAD_SIZE 0.3
#define QUAD_LEG_W 0.12
#define QUAD_LEG_LEN 0.55
#define QUAD_LEG_D 0.12
#define QUAD_HIP_X (QUAD_BODY_W / 2.0)
#define QUAD_HIP_Z_FRONT (QUAD_BODY_L / 2.0 - QUAD_LEG_D / 2.0)
#define QUAD_HIP_Z_BACK (-(QUAD_BODY_L / 2.0 - QUAD_LEG_D / 2.0))

/* 왕복 경로는 humanoid_path.txt와 같은 원칙으로 하드코딩하지 않고
   quadruped_patrol.txt에서 읽는다 ("z x_min x_max speed" 한 줄).
   파일이 없으면 기본값을 쓴다. */
static double quad_patrol_z = -9.0;
static double quad_patrol_x_min = -4.0;
static double quad_patrol_x_max = 4.0;
static double quad_patrol_speed = 0.3;

static void load_quad_patrol(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (!f) {
        fprintf(stderr, "4족 로봇 경로 파일을 열 수 없어 기본값을 사용합니다: %s\n", filename);
        return;
    }
    char line[128];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        double z, xmin, xmax, speed;
        if (sscanf(line, "%lf %lf %lf %lf", &z, &xmin, &xmax, &speed) == 4) {
            quad_patrol_z = z;
            quad_patrol_x_min = xmin;
            quad_patrol_x_max = xmax;
            quad_patrol_speed = speed;
            break; /* 첫 번째 유효한 줄 하나만 쓴다 */
        }
    }
    fclose(f);
}

#define QUAD_TURN_SHARPNESS 6.0 /* 클수록 방향 전환 구간이 좁고 급함 */

static double quad_x = 0.0;
static double quad_heading = 90.0; /* degrees, y축 둘레 회전 */
static double quad_fl_angle = 0.0; /* front-left */
static double quad_fr_angle = 0.0; /* front-right */
static double quad_bl_angle = 0.0; /* back-left */
static double quad_br_angle = 0.0; /* back-right */

static void draw_quad_body(void) {
    glPushMatrix();
    if (!rendering_shadow) glColor3f(0.50f, 0.55f, 0.60f);
    glScaled(QUAD_BODY_W, QUAD_BODY_H, QUAD_BODY_L);
    glutSolidCube(1.0);
    glPopMatrix();
}

static void draw_quad_head(void) {
    glPushMatrix();
    if (!rendering_shadow) glColor3f(0.60f, 0.62f, 0.65f);
    glScaled(QUAD_HEAD_SIZE, QUAD_HEAD_SIZE, QUAD_HEAD_SIZE);
    glutSolidCube(1.0);
    glPopMatrix();
}

static void draw_quad_leg(void) {
    glPushMatrix();
    if (!rendering_shadow) glColor3f(0.35f, 0.35f, 0.40f);
    glTranslated(0.0, -QUAD_LEG_LEN / 2.0, 0.0);
    glScaled(QUAD_LEG_W, QUAD_LEG_LEN, QUAD_LEG_D);
    glutSolidCube(1.0);
    glPopMatrix();
}

/* draw_trees()처럼 자기 몫의 ROOT 변환을 내부에서 직접 push/pop으로
   감싸서, 호출하는 쪽에서 따로 감싸지 않아도 변환이 새지 않는다
   (draw_humanoid_body()와 반대 방식 - humanoid_test.c의 하늘/땅/그림자
   절 참고). */
static void draw_quadruped(void) {
    glPushMatrix();
        glTranslated(quad_x, QUAD_LEG_LEN, quad_patrol_z); /* ROOT: 발이 땅에 닿도록 들어올림 */
        glRotated(quad_heading, 0, 1, 0);               /* ROOT: 이동 방향을 바라봄 */

        glPushMatrix();
            glTranslated(0.0, QUAD_BODY_H / 2.0, 0.0); /* body: T1 */
            draw_quad_body();

            glPushMatrix();
                glTranslated(0.0, QUAD_BODY_H * 0.15, QUAD_BODY_L / 2.0 + QUAD_HEAD_SIZE * 0.45); /* head: T1*T2 */
                draw_quad_head();
            glPopMatrix();
        glPopMatrix();

        glPushMatrix();
            glTranslated(-QUAD_HIP_X, 0.0, QUAD_HIP_Z_FRONT);
            glRotated(quad_fl_angle, 1, 0, 0);
            draw_quad_leg();
        glPopMatrix();

        glPushMatrix();
            glTranslated(QUAD_HIP_X, 0.0, QUAD_HIP_Z_FRONT);
            glRotated(quad_fr_angle, 1, 0, 0);
            draw_quad_leg();
        glPopMatrix();

        glPushMatrix();
            glTranslated(-QUAD_HIP_X, 0.0, QUAD_HIP_Z_BACK);
            glRotated(quad_bl_angle, 1, 0, 0);
            draw_quad_leg();
        glPopMatrix();

        glPushMatrix();
            glTranslated(QUAD_HIP_X, 0.0, QUAD_HIP_Z_BACK);
            glRotated(quad_br_angle, 1, 0, 0);
            draw_quad_leg();
        glPopMatrix();
    glPopMatrix();
}

static void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    draw_sky();

    glLoadIdentity();
    apply_camera();

    draw_axes(1.0);

    /* 땅: 체커 텍스처와는 완전히 분리해서, texturing 토글과 무관하게
       항상 draw_ground() 안의 glColor3f(흙색)만으로 그린다. */
    glDisable(GL_TEXTURE_2D);
    draw_ground();

    /* 그림자 패스: 조명/텍스처를 끄고 반투명 검은색으로, 광원 기준
       y=0 투영 행렬을 곱한 채로 몸과 나무를 한 번 더 그린다. 깊이값은
       쓰지 않아서(glDepthMask(GL_FALSE)) 평면 위에 눌린 모양으로만
       남고 실제 몸이나 바닥과 z-fighting을 일으키지 않는다. */
    rendering_shadow = 1;
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glColor4f(0.0f, 0.0f, 0.0f, 0.35f);
    glPushMatrix();
        apply_shadow_matrix(LIGHT_X, LIGHT_Y, LIGHT_Z);
        /* draw_all_humanoids()/draw_trees()/draw_quadruped() 모두 각자
           자기 몫의 ROOT 변환을 내부에서 push/pop으로 감싸므로, 여기서
           따로 감싸지 않아도 한 캐릭터의 변환이 다음에 그려지는 다른
           소품으로 새지 않는다 (예전에 이걸 안 지켜서 나무 그림자가
           캐릭터를 따라 빙글빙글 도는 버그가 있었다). */
        draw_all_humanoids();
        draw_trees();
        draw_quadruped();
    glPopMatrix();
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    rendering_shadow = 0;
    if (lighting) glEnable(GL_LIGHTING);

    /* 실제 몸: texturing이 켜져 있으면 draw_body/draw_head/draw_arm/
       draw_leg가 각자 알아서 자기 몫의 텍스처(셔츠/피부/바지)를
       바인딩한다. 여기서는 그냥 켜줄지 말지만 결정한다. */
    if (texturing) {
        glEnable(GL_TEXTURE_2D);
    } else {
        glDisable(GL_TEXTURE_2D);
    }

    draw_all_humanoids();

    /* 나무/4족 로봇: 땅과 마찬가지로 체커 텍스처와 분리해서, 항상
       각자의 glColor3f만으로 그린다. */
    glDisable(GL_TEXTURE_2D);
    draw_trees();
    draw_quadruped();

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

        /* 몸통도 같이 걸어간다: humanoid_path.txt의 웨이포인트를 잇는
           폐곡선을 따라, 이동 거리(s)를 t에 비례시켜 구한다. 매 틱
           고정값을 누적시키지 않고 t로부터 직접 계산하므로(path_sample),
           idle() 호출 빈도(프레임 속도)와 무관하게 항상 같은 속도로 걷는다. */
        path_sample(0.6 * t, &body_x, &body_z, &body_heading);

        /* 클론들은 같은 경로를 같은 속도로 걷되, 호의 길이(s) 기준으로
           고르게 떨어진 지점에서 걷는다 - path_total_len을
           (클론 수+1)로 나눈 간격만큼씩 밀어주면 경로 모양과 무관하게
           항상 고리 위에 균등하게 퍼진다. */
        for (int i = 0; i < HUMANOID_CLONE_COUNT; i++) {
            double offset = path_total_len * (double)(i + 1) / (double)(HUMANOID_CLONE_COUNT + 1);
            path_sample(0.6 * t + offset, &clone_x[i], &clone_z[i], &clone_heading[i]);
        }

        /* 4족 로봇은 x축 방향으로만 왕복(patrol)한다 - z는
           quadruped_patrol.txt에서 읽은 quad_patrol_z로 고정이라
           humanoid의 걷는 경로(z: -3.5..3.5)와 절대 겹치지 않는다.
           위치는 sin(), 이동 방향은 그 미분인 cos()의 부호를 따른다:
           cos > 0이면 +x로 가는 중이라 몸의 forward 벡터(로컬 +Z,
           heading=90도일 때 세계 +X)가 +x를 향하고, cos < 0이면
           반대로 -90도를 향해야 한다.
           cos의 부호가 바뀌는 순간(=왕복 양 끝, 마침 sin이 최대/최소라
           위치는 거의 멈춰있는 지점) heading을 90 <-> -90으로 그냥
           뚝 끊어 바꾸면 몸이 순간이동한 것처럼 보인다. 그래서
           tanh(k*cos(phase))로 완만하게 이어서, cos가 0 근처를 지나는
           짧은 구간 동안 heading이 90 -> 0(로컬 +Z, 즉 x축에 수직으로
           선 모습) -> -90으로 부드럽게 돌아가는 "제자리 방향 전환"
           애니메이션이 저절로 생기게 한다. k가 클수록 전환 구간이
           좁아져 더 급하게 꺾인다. */
        double quad_center = (quad_patrol_x_min + quad_patrol_x_max) / 2.0;
        double quad_half_range = (quad_patrol_x_max - quad_patrol_x_min) / 2.0;
        double quad_phase = quad_patrol_speed * t;
        quad_x = quad_center + quad_half_range * sin(quad_phase);
        quad_heading = 90.0 * tanh(QUAD_TURN_SHARPNESS * cos(quad_phase));

        /* 트롯(trot) 걸음걸이도 같은 t로부터 직접 계산한다 (같은 이유 -
           idle() 호출 빈도와 무관하게 항상 같은 속도로 흔들리게 하려고).
           대각선 다리 쌍이 반대 위상. */
        double quad_swing = 30.0 * sin(2.0 * t);
        quad_fl_angle = quad_swing;
        quad_br_angle = quad_swing;
        quad_fr_angle = -quad_swing;
        quad_bl_angle = -quad_swing;
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
            body_x = 0.0; body_z = 0.0; body_heading = 0.0;
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

static void special_keys(int key, int x, int y) {
    (void)x; (void)y;
    double heading_r = body_heading * CAM_PI / 180.0;
    switch (key) {
        case GLUT_KEY_UP:
            animate = 0;
            body_x += 0.1 * sin(heading_r);
            body_z += 0.1 * cos(heading_r);
            break;
        case GLUT_KEY_DOWN:
            animate = 0;
            body_x -= 0.1 * sin(heading_r);
            body_z -= 0.1 * cos(heading_r);
            break;
        case GLUT_KEY_LEFT:
            animate = 0; body_heading -= 5.0;
            break;
        case GLUT_KEY_RIGHT:
            animate = 0; body_heading += 5.0;
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
    load_path("humanoid_path.txt");
    load_quad_patrol("quadruped_patrol.txt");

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Humanoid Matrix Stack Test - Body/Head/Arms/Legs");

    init_gl();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(special_keys);
    glutIdleFunc(idle);
    glutMouseFunc(mouse_button);
    glutMotionFunc(mouse_motion);
    glutMouseWheelFunc(mouse_wheel);

    glutMainLoop();
    return 0;
}
