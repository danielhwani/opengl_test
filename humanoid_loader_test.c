/*
 * humanoid_loader_test.c
 * -----------------------
 * gen_humanoid_assets.c가 만들어낸 body.obj / head.obj / arm.obj / leg.obj
 * (표준 Wavefront OBJ)와 humanoid_scene.txt(계층/관절 정보)를
 * 런타임에 읽어서 렌더링하는 프로그램.
 *
 * humanoid_test.c가 glutSolidCube/glutSolidSphere + 하드코딩된
 * push/pop 트리였다면, 이 파일은:
 *   - 정점/법선 데이터는 OBJ 파일에서 읽어 glBegin(GL_TRIANGLES) /
 *     glVertex3d / glNormal3d로 직접 그리고
 *   - 부위 간 계층 구조(부모-자식)와 관절 정보는 humanoid_scene.txt에서
 *     읽어 그 데이터로 push/pop 트리를 "동적으로" 구성한다.
 * 즉 모델 데이터와 렌더링 코드가 분리된 형태.
 *
 * 빌드:
 *   gcc humanoid_loader_test.c -o humanoid_loader_test.exe -lfreeglut -lopengl32 -lglu32 -lwinmm
 *
 * 실행 전에 gen_humanoid_assets.exe를 먼저 실행해서
 * body.obj / head.obj / arm.obj / leg.obj / humanoid_scene.txt를 만들어둬야 한다.
 *
 * 조작: humanoid_test.c와 동일 (Space, Q/A, E/D, Z/X, C/V, T, L, R, ESC,
 *       방향키 = 몸통 전진/후진/회전,
 *       마우스 왼쪽 드래그 = 카메라 궤도 회전, 마우스 휠 = 줌)
 *
 * 자동 애니메이션일 때 몸통은 humanoid_path.txt에 적힌 웨이포인트
 * ("x z" 한 줄씩, 마지막 점에서 첫 점으로 돌아와 폐곡선을 이룸)를
 * Catmull-Rom 스플라인으로 부드럽게 이어 따라 걷는다. 파일이 없으면
 * 기본 사각형 경로를 쓴다.
 */

#include <GL/freeglut.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CAM_PI 3.14159265358979323846

/* ---------------- OBJ 메쉬 ---------------- */

#define MAX_V 8192
#define MAX_F 8192
#define MAX_MESH 8

typedef struct {
    char name[64];
    double v[MAX_V][3];
    double vn[MAX_V][3];
    int f[MAX_F][6]; /* v1,vn1, v2,vn2, v3,vn3 (0-based) */
    int nv, nvn, nf;
} Mesh;

static Mesh g_meshes[MAX_MESH];
static int g_mesh_count = 0;

static int load_obj(const char *filename) {
    for (int i = 0; i < g_mesh_count; i++) {
        if (strcmp(g_meshes[i].name, filename) == 0) return i; /* 이미 로드됨 */
    }

    FILE *f = fopen(filename, "r");
    if (!f) {
        fprintf(stderr, "OBJ 파일을 열 수 없습니다: %s\n", filename);
        return -1;
    }

    Mesh *m = &g_meshes[g_mesh_count];
    strncpy(m->name, filename, sizeof(m->name) - 1);
    m->nv = m->nvn = m->nf = 0;

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == 'v' && line[1] == ' ') {
            sscanf(line + 2, "%lf %lf %lf",
                   &m->v[m->nv][0], &m->v[m->nv][1], &m->v[m->nv][2]);
            m->nv++;
        } else if (line[0] == 'v' && line[1] == 'n') {
            sscanf(line + 3, "%lf %lf %lf",
                   &m->vn[m->nvn][0], &m->vn[m->nvn][1], &m->vn[m->nvn][2]);
            m->nvn++;
        } else if (line[0] == 'f' && line[1] == ' ') {
            int v1, n1, v2, n2, v3, n3;
            sscanf(line + 2, "%d//%d %d//%d %d//%d", &v1, &n1, &v2, &n2, &v3, &n3);
            m->f[m->nf][0] = v1 - 1; m->f[m->nf][1] = n1 - 1;
            m->f[m->nf][2] = v2 - 1; m->f[m->nf][3] = n2 - 1;
            m->f[m->nf][4] = v3 - 1; m->f[m->nf][5] = n3 - 1;
            m->nf++;
        }
    }
    fclose(f);

    printf("loaded %s: %d verts, %d normals, %d faces\n", filename, m->nv, m->nvn, m->nf);
    return g_mesh_count++;
}

static void draw_mesh(int mesh_index) {
    if (mesh_index < 0) return;
    Mesh *m = &g_meshes[mesh_index];
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < m->nf; i++) {
        for (int k = 0; k < 3; k++) {
            int vi = m->f[i][k * 2 + 0];
            int ni = m->f[i][k * 2 + 1];
            glNormal3d(m->vn[ni][0], m->vn[ni][1], m->vn[ni][2]);
            glVertex3d(m->v[vi][0], m->v[vi][1], m->v[vi][2]);
        }
    }
    glEnd();
}

/* ---------------- 씬(계층/관절) ---------------- */

#define MAX_PARTS 16
#define JOINT_NONE 0
#define JOINT_ROTX 1

typedef struct {
    char name[32];
    char parent[32];
    double t[3];
    int joint;
    float color[3];
    int mesh_index;
} Part;

static Part g_parts[MAX_PARTS];
static int g_part_count = 0;

static int joint_from_string(const char *s) {
    if (strcmp(s, "ROTX") == 0) return JOINT_ROTX;
    return JOINT_NONE;
}

static void load_scene(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (!f) {
        fprintf(stderr, "씬 파일을 열 수 없습니다: %s\n", filename);
        return;
    }
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;

        char name[32], mesh_file[64], parent[32], joint_str[16];
        double tx, ty, tz;
        float r, g, b;
        int n = sscanf(line, "%31s %63s %31s %lf %lf %lf %15s %f %f %f",
                        name, mesh_file, parent, &tx, &ty, &tz, joint_str, &r, &g, &b);
        if (n != 10) continue;

        Part *p = &g_parts[g_part_count];
        strncpy(p->name, name, sizeof(p->name) - 1);
        strncpy(p->parent, parent, sizeof(p->parent) - 1);
        p->t[0] = tx; p->t[1] = ty; p->t[2] = tz;
        p->joint = joint_from_string(joint_str);
        p->color[0] = r; p->color[1] = g; p->color[2] = b;
        p->mesh_index = load_obj(mesh_file);
        g_part_count++;
    }
    fclose(f);
    printf("loaded %s: %d parts\n", filename, g_part_count);
}

/* ---------------- 애니메이션 상태 ---------------- */

static double t = 0.0;
static double left_angle = 0.0;
static double right_angle = 0.0;
static double left_leg_angle = 0.0;
static double right_leg_angle = 0.0;
/* 몸통 전체의 월드 위치/방향 - humanoid_test.c와 동일하게, 트리 전체
   (ROOT)에 적용되는 변환이라 다른 모든 부위를 다 끌고 다닌다. */
static double body_x = 0.0;
static double body_z = 0.0;
static double body_heading = 0.0; /* degrees, y축 둘레 회전 */
static int animate = 1;

/* 4족 로봇(QUAD_ROOT 트리) 상태 - angle_for_part()가 참조하므로 그보다
   앞에 선언해야 한다. */
static double quad_x = 0.0;
static double quad_heading = 90.0; /* degrees, y축 둘레 회전 */
static double quad_fl_angle = 0.0; /* front-left */
static double quad_fr_angle = 0.0; /* front-right */
static double quad_bl_angle = 0.0; /* back-left */
static double quad_br_angle = 0.0; /* back-right */
static int lighting = 1;
static int texturing = 1;
static GLuint texture_id = 0;

/* 얼굴 코 표시용 - head.obj 생성 시 쓴 반지름(gen_humanoid_assets.c의
   HEAD_R)과 맞춰야 코가 얼굴 표면에 딱 붙는다. */
#define HEAD_R 0.35
/* ROOT가 "엉덩이" 높이라서 다리 길이(gen_humanoid_assets.c의 LEG_LEN과
   동일)만큼 들어올려야 발이 땅(y=0)에 닿는다. */
#define LEG_LEN 1.2

/* ---- 경로(path): humanoid_test.c와 동일. humanoid_path.txt에서 읽은
   웨이포인트(x z)를 순서대로 잇는 폐곡선을 Catmull-Rom 스플라인으로
   부드럽게 따라 걷는다. 파일이 없으면 기본 사각형 경로를 쓴다. ---- */
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

    path_total_len = 0.0;
    for (int i = 0; i < path_count; i++) {
        int j = (i + 1) % path_count;
        double dx = path_x[j] - path_x[i];
        double dz = path_z[j] - path_z[i];
        path_seg_len[i] = sqrt(dx * dx + dz * dz);
        path_total_len += path_seg_len[i];
    }
}

/* Catmull-Rom 스플라인: 직선 보간 대신 앞뒤 두 점(p0,p3)까지 참고해
   부드러운 곡선을 그린다. 접선(out_tx/out_tz)이 그 지점의 진행 방향. */
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
   시작점부터의 누적 거리, path_total_len을 넘으면 처음부터 다시 돈다) */
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

/* teapot_test.c/humanoid_test.c와 동일한 체커 텍스처 - object-linear
   GL_TEXTURE_GEN을 쓰므로, OBJ 로더가 텍스처 좌표(vt)를 읽지 않아도
   텍스처가 자동으로 입혀진다. */
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
   humanoid_test.c와 동일. 체커 패턴 하나로 모든 부위를 덮던 것 대신
   부위별로 다른 (단색) 텍스처를 입힌다. */
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

/* 피부: 살색 단색. */
static void create_skin_texture(void) {
    static unsigned char pix[BODY_TEX_SIZE][BODY_TEX_SIZE][3];
    for (int i = 0; i < BODY_TEX_SIZE; i++) {
        for (int j = 0; j < BODY_TEX_SIZE; j++) {
            pix[i][j][0] = 222; pix[i][j][1] = 180; pix[i][j][2] = 150;
        }
    }
    skin_texture_id = upload_body_texture(&pix[0][0][0]);
}

/* 상의: 가로 줄무늬 (셔츠처럼). */
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

/* 하의: 데님 느낌의 짙은 청색 단색. */
static void create_pants_texture(void) {
    static unsigned char pix[BODY_TEX_SIZE][BODY_TEX_SIZE][3];
    for (int i = 0; i < BODY_TEX_SIZE; i++) {
        for (int j = 0; j < BODY_TEX_SIZE; j++) {
            pix[i][j][0] = 60; pix[i][j][1] = 80; pix[i][j][2] = 120;
        }
    }
    pants_texture_id = upload_body_texture(&pix[0][0][0]);
}

/* 부위 이름(씬 파일의 Body/Head/LeftArm/...)으로 어떤 텍스처를 입힐지
   고른다. humanoid_test.c는 draw_body/draw_head처럼 부위별 함수가
   따로 있어서 그 안에서 직접 바인딩하지만, 로더 버전은 모든 부위를
   render_children() 하나가 데이터로 순회하며 그리므로 이름으로
   분기한다. */
static GLuint texture_for_part(const char *name) {
    if (strcmp(name, "Head") == 0) return skin_texture_id;
    if (strcmp(name, "Body") == 0) return shirt_texture_id;
    if (strcmp(name, "LeftArm") == 0 || strcmp(name, "RightArm") == 0) return shirt_texture_id;
    if (strcmp(name, "LeftLeg") == 0 || strcmp(name, "RightLeg") == 0) return pants_texture_id;
    return 0;
}

static double angle_for_part(const char *name) {
    if (strcmp(name, "LeftArm") == 0) return left_angle;
    if (strcmp(name, "RightArm") == 0) return right_angle;
    if (strcmp(name, "LeftLeg") == 0) return left_leg_angle;
    if (strcmp(name, "RightLeg") == 0) return right_leg_angle;
    if (strcmp(name, "QuadLegFL") == 0) return quad_fl_angle;
    if (strcmp(name, "QuadLegFR") == 0) return quad_fr_angle;
    if (strcmp(name, "QuadLegBL") == 0) return quad_bl_angle;
    if (strcmp(name, "QuadLegBR") == 0) return quad_br_angle;
    return 0.0;
}

/* 그림자를 그리는 동안(rendering_shadow)은 부위별 색을 칠하지 않고
   display()에서 미리 설정해둔 반투명 검은색을 유지한다.
   humanoid_test.c와 동일한 이유. */
static int rendering_shadow = 0;

/* parent_name의 자식들을 재귀적으로 push/pop하며 그린다 (씬 파일의
   계층 정보를 그대로 따라가는, "데이터로부터 만들어진" 트리 렌더링) */
static void render_children(const char *parent_name) {
    for (int i = 0; i < g_part_count; i++) {
        Part *p = &g_parts[i];
        if (strcmp(p->parent, parent_name) != 0) continue;

        int tex_on = texturing && !rendering_shadow;
        GLuint part_tex = texture_for_part(p->name);

        glPushMatrix();
        glTranslated(p->t[0], p->t[1], p->t[2]);
        if (p->joint == JOINT_ROTX) {
            glRotated(angle_for_part(p->name), 1, 0, 0);
        }
        if (tex_on && part_tex != 0) {
            /* 부위별 텍스처(피부/상의/하의)를 입히는 동안은 glColor를
               흰색으로 둬야 한다 - GL_MODULATE라 최종 색 = 텍스처 색 *
               vertex 색인데, 씬 파일에 적힌 칙칙한 색을 그대로 두면
               텍스처 색 위에 덧씌워져 탁해진다. */
            glBindTexture(GL_TEXTURE_2D, part_tex);
            if (!rendering_shadow) glColor3f(1.0f, 1.0f, 1.0f);
        } else if (!rendering_shadow) {
            glColor3f(p->color[0], p->color[1], p->color[2]);
        }
        draw_mesh(p->mesh_index);

        if (strcmp(p->name, "Head") == 0) {
            /* 코: 머리 앞쪽(+Z)에 작은 돌출부를 붙여 앞/뒤를 구별한다.
               humanoid_test.c의 draw_head()와 동일한 원리. 피부 텍스처
               위에 순색 빨강이 또렷이 보이도록 텍스처를 잠깐 끈다. */
            if (tex_on) glDisable(GL_TEXTURE_2D);
            glPushMatrix();
            if (!rendering_shadow) glColor3f(0.85f, 0.25f, 0.25f);
            glTranslated(0.0, HEAD_R, HEAD_R * 0.85);
            glutSolidSphere(HEAD_R * 0.28, 12, 12);
            glPopMatrix();
            if (tex_on) glEnable(GL_TEXTURE_2D);
        }

        render_children(p->name); /* 이 부위를 부모로 둔 자식들 */

        glPopMatrix();
    }
}

/* 광원 위치(LIGHT_X/Y/Z, init_gl()의 GL_LIGHT0과 동일)와 같은 좌표를
   평행광 방향으로 취급해, y에 비례해 x/z를 미리 어긋나게 하는 전단
   (shear) 변환만으로 y=0 평면에 투영한다. 점광원 투영(perspective
   divide)을 쓰면 광원 높이에 가까운 부위(머리)의 그림자가 무한히
   늘어나 카메라 절두체에서 잘려 보이는 문제가 있어 humanoid_test.c와
   동일하게 전단 변환으로 바꿨다. */
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

/* 하늘: humanoid_test.c와 동일한 화면 전체 그라디언트. */
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
        glColor3f(0.35f, 0.55f, 0.85f); glVertex2d(0.0, 1.0);
        glColor3f(0.35f, 0.55f, 0.85f); glVertex2d(1.0, 1.0);
        glColor3f(0.80f, 0.88f, 0.95f); glVertex2d(1.0, 0.0);
        glColor3f(0.80f, 0.88f, 0.95f); glVertex2d(0.0, 0.0);
    glEnd();

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    glEnable(GL_DEPTH_TEST);
    if (lighting) glEnable(GL_LIGHTING);
}

/* 땅: humanoid_test.c와 동일, 흙색 큰 사각형. */
#define GROUND_HALF 20.0
#define GROUND_Y -0.01

static void draw_ground(void) {
    glColor3f(0.42f, 0.30f, 0.18f);
    glNormal3d(0.0, 1.0, 0.0);
    glBegin(GL_QUADS);
        glVertex3d(-GROUND_HALF, GROUND_Y, -GROUND_HALF);
        glVertex3d( GROUND_HALF, GROUND_Y, -GROUND_HALF);
        glVertex3d( GROUND_HALF, GROUND_Y,  GROUND_HALF);
        glVertex3d(-GROUND_HALF, GROUND_Y,  GROUND_HALF);
    glEnd();
}

/* 나무: humanoid_test.c와 동일 - 줄기(상자) + 크리스마스 트리처럼
   아래로 갈수록 커지는 원뿔 3단 + 빨간 장식구. */
static void draw_tree(double x, double z) {
    static const double tier_y[3]      = {1.00, 1.45, 1.85};
    static const double tier_radius[3] = {0.70, 0.52, 0.36};
    static const double tier_height[3] = {0.85, 0.75, 0.65};
    static const float tier_color[3][3] = {
        {0.12f, 0.36f, 0.15f},
        {0.15f, 0.44f, 0.18f},
        {0.20f, 0.52f, 0.22f},
    };

    glPushMatrix();
    glTranslated(x, 0.0, z);

    glPushMatrix();
        if (!rendering_shadow) glColor3f(0.40f, 0.26f, 0.13f); /* 줄기 */
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

/* ---- 4족 보행 로봇: humanoid과 마찬가지로 OBJ + 씬 데이터 기반이다.
   gen_humanoid_assets.c가 만든 quad_body.obj/quad_head.obj/quad_leg.obj와
   humanoid_scene.txt에 parent가 "QUAD_ROOT"인 부분(QuadBody/QuadHead/
   QuadLegFL/FR/BL/BR)으로 기술된 계층을 render_children("QUAD_ROOT")가
   그대로 재귀 렌더링한다 - humanoid 쪽 render_children("ROOT")와 완전히
   같은 함수를 이름만 다르게 불러 재사용하는 것뿐이다. 하드코딩된
   glutSolidCube 호출은 더 이상 없다.

   humanoid의 걷는 경로(사각형, z는 -3.5..3.5)와 겹치지 않는
   z=quad_patrol_z(기본 -9, 훨씬 바깥쪽)에 고정해두고, 그 위에서 x축
   방향으로만 왕복(patrol)한다 - 동선이 사람과 절대 교차하지 않는다.
   걸을 때는 트롯(trot) 걸음걸이로: 대각선으로 마주보는 두 다리가
   쌍을 이뤄 서로 반대 위상으로 흔들린다. ---- */
#define QUAD_LEG_LEN 0.55 /* gen_humanoid_assets.c의 QUAD_LEG_LEN과 동일 -
                             ROOT를 이만큼 들어올려야 발이 땅에 닿는다 */
#define QUAD_TURN_SHARPNESS 6.0 /* 클수록 방향 전환 구간이 좁고 급함 */

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

/* render_children()처럼 자기 몫의 ROOT 변환을 내부에서 직접 push/pop
   으로 감싸서, 호출하는 쪽에서 따로 감싸지 않아도 변환이 새지 않는다
   (draw_trees()와 같은 방식). */
static void draw_quadruped(void) {
    glPushMatrix();
        glTranslated(quad_x, QUAD_LEG_LEN, quad_patrol_z); /* ROOT: 발이 땅에 닿도록 들어올림 */
        glRotated(quad_heading, 0, 1, 0);                   /* ROOT: 이동 방향을 바라봄 */
        render_children("QUAD_ROOT");
    glPopMatrix();
}

/* ---------------- OpenGL 보일러플레이트 ---------------- */

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

/* ROOT 변환 + 전체 트리를 그린다. 그림자 패스에서도 재사용하기 위해
   분리해뒀다. ROOT는 "엉덩이" 높이라 LEG_LEN만큼 들어올려야 발이
   땅(y=0)에 닿는다 (humanoid_test.c의 draw_humanoid_body()와 동일). */
static void draw_humanoid_root(void) {
    glTranslated(body_x, LEG_LEN, body_z); /* 몸통의 월드 위치 (ROOT) */
    glRotated(body_heading, 0, 1, 0);      /* 몸통의 방향 (ROOT) */
    render_children("ROOT");
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

    /* 그림자 패스: humanoid_test.c와 동일 - 조명/텍스처를 끄고 반투명
       검은색으로, 전단 변환을 곱한 채로 몸과 나무를 한 번 더 그린다.
       draw_humanoid_root()는 자기 몫의 ROOT 변환을 push/pop 없이 바로
       곱하므로, 그 변환이 나무까지 새어나가지 않도록 별도로
       glPushMatrix로 감싼다 (안 그러면 나무 그림자가 캐릭터를 따라
       움직이고 도는 버그가 생긴다). */
    rendering_shadow = 1;
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glColor4f(0.0f, 0.0f, 0.0f, 0.35f);
    glPushMatrix();
        apply_shadow_matrix(LIGHT_X, LIGHT_Y, LIGHT_Z);
        glPushMatrix();
            draw_humanoid_root();
        glPopMatrix();
        draw_trees();
        draw_quadruped();
    glPopMatrix();
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    rendering_shadow = 0;
    if (lighting) glEnable(GL_LIGHTING);

    /* 실제 몸: texturing이 켜져 있으면 render_children()이 부위별로
       알아서 자기 몫의 텍스처(셔츠/피부/바지)를 바인딩한다. 여기서는
       그냥 켜줄지 말지만 결정한다. */
    if (texturing) {
        glEnable(GL_TEXTURE_2D);
    } else {
        glDisable(GL_TEXTURE_2D);
    }

    glPushMatrix();
        draw_humanoid_root();
    glPopMatrix();

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
        right_angle = -35.0 * sin(t);
        left_leg_angle = -25.0 * sin(t);
        right_leg_angle = 25.0 * sin(t);

        /* 몸통도 같이 걸어간다: humanoid_path.txt의 웨이포인트를 잇는
           폐곡선을 따라, 이동 거리(s)를 t에 비례시켜 구한다. 매 틱
           고정값을 누적시키지 않고 t로부터 직접 계산하므로(path_sample),
           idle() 호출 빈도(프레임 속도)와 무관하게 항상 같은 속도로 걷는다. */
        path_sample(0.6 * t, &body_x, &body_z, &body_heading);

        /* 4족 로봇은 x축 방향으로만 왕복(patrol)한다 - humanoid_test.c와
           동일한 이유/계산. 방향 전환은 tanh로 완만하게 이어서 왕복 양
           끝에서 순간이동하듯 꺾이지 않고 제자리에서 도는 것처럼
           보이게 한다. */
        double quad_center = (quad_patrol_x_min + quad_patrol_x_max) / 2.0;
        double quad_half_range = (quad_patrol_x_max - quad_patrol_x_min) / 2.0;
        double quad_phase = quad_patrol_speed * t;
        quad_x = quad_center + quad_half_range * sin(quad_phase);
        quad_heading = 90.0 * tanh(QUAD_TURN_SHARPNESS * cos(quad_phase));

        /* 트롯(trot) 걸음걸이도 같은 t로부터 직접 계산한다. 대각선
           다리 쌍이 반대 위상. */
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

int main(int argc, char **argv) {
    load_path("humanoid_path.txt");
    load_quad_patrol("quadruped_patrol.txt");
    load_scene("humanoid_scene.txt");

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Humanoid Loader Test - OBJ + scene file -> push/pop tree");

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
