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
static int lighting = 1;
static int texturing = 1;
static GLuint texture_id = 0;

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

static double angle_for_part(const char *name) {
    if (strcmp(name, "LeftArm") == 0) return left_angle;
    if (strcmp(name, "RightArm") == 0) return right_angle;
    if (strcmp(name, "LeftLeg") == 0) return left_leg_angle;
    if (strcmp(name, "RightLeg") == 0) return right_leg_angle;
    return 0.0;
}

/* parent_name의 자식들을 재귀적으로 push/pop하며 그린다 (씬 파일의
   계층 정보를 그대로 따라가는, "데이터로부터 만들어진" 트리 렌더링) */
static void render_children(const char *parent_name) {
    for (int i = 0; i < g_part_count; i++) {
        Part *p = &g_parts[i];
        if (strcmp(p->parent, parent_name) != 0) continue;

        glPushMatrix();
        glTranslated(p->t[0], p->t[1], p->t[2]);
        if (p->joint == JOINT_ROTX) {
            glRotated(angle_for_part(p->name), 1, 0, 0);
        }
        glColor3f(p->color[0], p->color[1], p->color[2]);
        draw_mesh(p->mesh_index);

        render_children(p->name); /* 이 부위를 부모로 둔 자식들 */

        glPopMatrix();
    }
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
static double cam_dist = 6.0;
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

static void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    apply_camera();

    draw_axes(1.0);

    if (texturing) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, texture_id);
    } else {
        glDisable(GL_TEXTURE_2D);
    }

    glPushMatrix();
        glTranslated(body_x, 0.0, body_z);   /* 몸통의 월드 위치 (ROOT) */
        glRotated(body_heading, 0, 1, 0);    /* 몸통의 방향 (ROOT) */
        render_children("ROOT");
    glPopMatrix();

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

        /* 몸통도 같이 걸어간다. heading을 매 틱 += 상수로 누적시키면
           idle()이 불리는 빈도(프레임 속도)에 따라 회전 속도가 달라져서
           - 빠른 PC에서는 눈이 어지러울 정도로 빨리 돈다. 그 대신 팔다리
           스윙과 똑같은 t를 써서 원 궤적을 닫힌 형태로 직접 계산하면
           프레임 속도와 무관하게 항상 같은 속도로 걷는다. */
        double heading_r = 0.15 * t;            /* 각속도: t 단위당 라디안 */
        const double walk_radius = 2.5;
        body_heading = heading_r * 180.0 / CAM_PI;
        body_x = walk_radius * (1.0 - cos(heading_r));
        body_z = walk_radius * sin(heading_r);
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
