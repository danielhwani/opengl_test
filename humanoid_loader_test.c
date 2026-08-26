/*
 * humanoid_loader_test.c
 * -----------------------
 * gen_humanoid_assets.c가 만들어낸 body.obj / head.obj / arm.obj
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
 * body.obj / head.obj / arm.obj / humanoid_scene.txt를 만들어둬야 한다.
 *
 * 조작: humanoid_test.c와 동일 (Space, Q/A, E/D, R, ESC)
 */

#include <GL/freeglut.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

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
static int animate = 1;

static double angle_for_part(const char *name) {
    if (strcmp(name, "LeftArm") == 0) return left_angle;
    if (strcmp(name, "RightArm") == 0) return right_angle;
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

static void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(4.0, 2.2, 4.0, 0.0, 0.9, 0.0, 0.0, 1.0, 0.0);

    draw_axes(1.0);

    render_children("ROOT");

    glutSwapBuffers();
}

static void idle(void) {
    if (animate) {
        t += 0.004;
        left_angle = 35.0 * sin(t);
        right_angle = -35.0 * sin(t);
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
    glutIdleFunc(idle);

    glutMainLoop();
    return 0;
}
