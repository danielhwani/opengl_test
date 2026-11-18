/*
 * gen_humanoid_assets.c
 * ----------------------
 * humanoid_test.c에 하드코딩되어 있던 모델(큐브로 만든 몸통/팔,
 * 구로 만든 머리)의 실제 정점/법선 데이터를 계산해서 표준 OBJ 파일로
 * 저장하고, 부위 간 계층/관절 정보는 별도의 간단한 텍스트 파일
 * (humanoid_scene.txt)로 저장한다.
 *
 * 추가로, 같은 모델을 ROS/URDF로도 내보낸다 (humanoid.urdf /
 * quadruped.urdf) - Y-up(OpenGL)에서 Z-up(ROS REP-103)으로 축을
 * 바꾼 *_ros.obj 메쉬를 참조하고, URDF가 표현할 수 없는 정보(절차적
 * 텍스처, 캐릭터별 옷 색 틴트, 걷기/트롯 애니메이션 파라미터 등)는
 * *_extra.yaml에 같이 적어둔다. 아직 TF/joint_state를 발행하는 ROS
 * 노드는 없고, 파일 생성까지만 한다.
 *
 * 출력:
 *   body.obj, head.obj, arm.obj, leg.obj,
 *   quad_body.obj, quad_head.obj, quad_leg.obj,
 *   quad_shin.obj                                 - OpenGL 데모용 Y-up OBJ 메쉬
 *   *_ros.obj (위와 같은 이름들)                  - URDF용 Z-up OBJ 메쉬
 *   humanoid_scene.txt                            - 부위 이름/메쉬/부모/위치/관절/색 정보
 *   humanoid.urdf / humanoid_extra.yaml           - humanoid의 URDF + 보조 정보
 *   quadruped.urdf / quadruped_extra.yaml         - 4족 로봇의 URDF + 보조 정보
 *
 * humanoid_loader_test.c가 Y-up OBJ + humanoid_scene.txt를 읽어서
 * 렌더링한다 (URDF는 별도 산출물이고 이 OpenGL 데모는 그대로 쓴다).
 *
 * 빌드:
 *   gcc gen_humanoid_assets.c -o gen_humanoid_assets.exe -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define PI 3.14159265358979323846

/* URDF의 <mesh filename="...">에 쓸 기본 경로. 이 프로젝트가 아직
   ROS 패키지가 아니라서(package://를 쓸 수 없어서) 이 머신에서의
   절대 경로를 file:// URI로 그냥 박아 넣는다 - robot_state_publisher/
   RViz2가 바로 찾을 수 있게 하기 위해서다. 프로젝트 폴더를 옮기면
   이 값을 바꾸고 gen_humanoid_assets.exe를 다시 돌려야 한다. */
#define ASSET_URI_PREFIX "file:///D:/claude_test/computer_graphics/"

/* humanoid_test.c와 동일한 치수 */
#define BODY_W 0.8
#define BODY_H 1.4
#define BODY_D 0.5
#define HEAD_R 0.35
#define ARM_W 0.25
#define ARM_LEN 1.0
#define ARM_D 0.25
/* 팔꿈치(2026-11-18 추가, 사용자 요청) - 다리의 THIGH_LEN/SHIN_LEN과
   같은 트릭(위팔+아래팔 길이 합이 기존 ARM_LEN과 같게 반으로 나눔).
   OpenGL 데모(arm.obj, write_scene())는 그대로 ARM_LEN 통짜라 손
   안 댐 - URDF(arm_ros.obj를 이 길이로 줄이고 forearm_ros.obj 추가)
   쪽에만 쓴다. */
#define UPPER_ARM_LEN (ARM_LEN / 2.0)
#define FOREARM_LEN (ARM_LEN / 2.0)
/* 손(2026-11-18 추가, humanoid_walk.xml의 hand_L/R 캡슐(fromto
   0~-0.15)과 길이 맞춤) - OpenGL 데모는 손 없이 아래팔까지만이라
   이 상수도 URDF 전용. */
#define HAND_LEN 0.15
/* SHOULDER_X: OpenGL 데모(write_scene() -> humanoid_scene.txt ->
   humanoid_loader_test.c)의 어깨 오프셋 - 몸통이 박스라는 전제의
   공식이라 URDF가 원기둥으로 바뀐 지금은 URDF 쪽엔 안 쓴다(아래
   URDF_SHOULDER_X 참고) - OpenGL 쪽 모양은 그대로 유지하려고 이
   값/용도는 안 건드림. */
#define SHOULDER_X (BODY_W / 2.0 + ARM_W / 2.0)
/* 몸통(골반/가슴) URDF 메쉬 반지름 - 2026-11-17: 처음엔 타원(rx=BODY_D/2,
   ry=BODY_W/2)이었다가, MJCF 쪽(humanoid_walk.xml)을 MuJoCo에 타원
   원기둥이 없어서 원형(반지름 0.32)으로 맞췄더니 사용자가 "통일성을
   위해 RViz 쪽도 원형으로" 요청 - 그래서 여기도 같은 0.32로 통일. */
#define TORSO_RADIUS 0.32
/* ARM_ROS_RY(팔 원기둥의 어깨축 방향 반지름)만큼 몸통 표면에서
   떨어뜨려 붙인다 - URDF 전용(OpenGL의 SHOULDER_X와 별개) -
   몸통이 원기둥(TORSO_RADIUS)으로 바뀌면서 기존 박스 기준 공식이
   안 맞게 됐다(팔이 몸통에서 떨어져 보임 - MJCF에서 먼저 겪은
   것과 같은 문제). */
#define ARM_ROS_RY 0.06
#define URDF_SHOULDER_X (TORSO_RADIUS + ARM_ROS_RY)
#define LEG_W 0.3
/* 무릎 관절 추가(2026-11-17) - 4족 로봇과 똑같은 트릭: 뻣뻣한 막대
   1개(LEG_LEN)였던 다리를 허벅지+종아리 2 세그먼트로 나누되, 다리가
   펴진(무릎각 0) 상태의 총 길이는 그대로 보존해서 base_to_body_z 등
   기존 "서있는 높이" 계산을 안 건드린다. 독립 MuJoCo 스크립트로
   먼저 확인한 결과: 4족과 똑같은 엉덩이 sin + 무릎 max(0,-cos)
   걸음걸이를 그대로 적용하면(좌우 균형 축이 없어서) 진폭/주파수를
   뭘 쓰든 1~3초 안에 넘어진다(기준선인 "가만히 서있기"는 완벽하게
   안정적이라 MJCF 자체의 문제는 아님) - 4족은 항상 다리 4개가
   땅에 닿아있어 정역학적으로 안정적이었지만, 2족은 한쪽 다리를
   드는 순간 무게중심이 지지 발 바깥쪽으로 나가버려서(좌우로 무게를
   옮기는 메커니즘이 없음) 구조적으로 넘어질 수밖에 없다. 그래도
   "진짜 관절 물리가 걷기를 시도한다"는 결과 자체(그리고 넘어지는
   모습)를 보여주는 데모로 만들기로 하고 무릎은 그대로 추가한다. */
#define THIGH_LEN 0.6
#define SHIN_LEN 0.6
#define LEG_LEN (THIGH_LEN + SHIN_LEN)
#define LEG_D 0.3
#define HIP_X ((BODY_W - LEG_W) / 2.0)

/* humanoid_test.c / humanoid_loader_test.c의 4족 로봇과 동일한 치수 */
#define QUAD_BODY_W 0.5
#define QUAD_BODY_L 1.2
#define QUAD_BODY_H 0.4
#define QUAD_HEAD_SIZE 0.3
#define QUAD_LEG_W 0.12
/* 무릎 관절 추가 - 뻣뻣한 막대 1개(QUAD_LEG_LEN)였던 다리를 허벅지+
   종아리 2 세그먼트로 나눴다. 다리가 펴진(무릎각 0) 상태의 총
   길이가 그대로 보존되도록 QUAD_LEG_LEN을 두 세그먼트 길이의 합으로
   다시 정의했다 - 그래서 base_to_body_z 등 QUAD_LEG_LEN을 참조하는
   기존 "서있는 높이" 계산은 전혀 안 건드려도 된다. 물리 보행
   데모(ros2_ws/.../mujoco/quad_walk.xml)에서 ROS 없이 mj_step만
   반복하는 독립 스크립트로 검증했다: 무릎 없을 때 10초에 0.88m
   전진하던 게, 무릎을 "엉덩이가 스윙하는 동안 굽히고 스탠스일 때
   펴지는" 신호(진폭 15도)로 움직이자 10초에 5.26m로(약 6배) 늘고
   기울기도 더 안정적이었다(16도부터 급격히 불안정, 17도부터
   넘어짐 - 15도가 한계 바로 아래 안전한 값). */
#define QUAD_THIGH_LEN 0.275
#define QUAD_SHIN_LEN 0.275
#define QUAD_LEG_LEN (QUAD_THIGH_LEN + QUAD_SHIN_LEN)
#define QUAD_LEG_D 0.12
#define QUAD_HIP_X (QUAD_BODY_W / 2.0)
#define QUAD_HIP_Z_FRONT (QUAD_BODY_L / 2.0 - QUAD_LEG_D / 2.0)
#define QUAD_HIP_Z_BACK (-(QUAD_BODY_L / 2.0 - QUAD_LEG_D / 2.0))

/* URDF 관절 각도 한계 - 실제 동역학 한계가 아니라(원래 애니메이션은
   순수 기구학적 sin() 함수라 그런 게 없다), humanoid_test.c의 swing
   진폭(팔 35도, 다리 25도, 4족 로봇 다리 30도)에 여유를 더한 placeholder
   값이다. humanoid_extra.yaml/quadruped_extra.yaml에도 같이 적어둔다. */
#define ARM_JOINT_LIMIT_RAD (45.0 * PI / 180.0)
#define LEG_JOINT_LIMIT_RAD (35.0 * PI / 180.0)
#define QUAD_LEG_JOINT_LIMIT_RAD (45.0 * PI / 180.0)
/* 무릎은 실제 무릎처럼 한쪽으로만(0=쭉 뻗음 ~ 90도=최대로 굽힘)
   굽혀진다 - 엉덩이처럼 좌우 대칭 범위가 아니다. */
#define QUAD_KNEE_JOINT_LIMIT_RAD (90.0 * PI / 180.0)
#define HUMANOID_KNEE_JOINT_LIMIT_RAD (90.0 * PI / 180.0)
/* 팔꿈치 - 무릎과 같은 값(한쪽으로만 0~90도, 실제 팔꿈치는 더 많이
   굽혀지지만 무릎 때처럼 단순화해서 일관성 유지). */
#define HUMANOID_ELBOW_JOINT_LIMIT_RAD (90.0 * PI / 180.0)
/* 전완 회전(회내/회외, 2026-11-18 추가) - humanoid_walk.xml의
   elbow_twist_L/R range와 동일. */
#define HUMANOID_ELBOW_TWIST_LIMIT_RAD (90.0 * PI / 180.0)
/* 손목(2026-11-18 추가) - humanoid_walk.xml의 wrist_deviate_L/R
   (편위, 20도) / wrist_flex_L/R(굽힘, 60도) range와 동일. */
#define HUMANOID_WRIST_DEVIATE_LIMIT_RAD (20.0 * PI / 180.0)
#define HUMANOID_WRIST_FLEX_LIMIT_RAD (60.0 * PI / 180.0)
/* 좌우/전후 밸런스 메커니즘(2026-11-17) - 엉덩이 ab/duction, 발목
   (롤+피치), 허리(롤+피치) - 전부 독립 스크립트로 LQR 검증 완료
   (humanoid_stand_lqr.py 참고). 값은 그 검증에 쓴 스크래치 MJCF와
   동일. */
#define HUMANOID_HIP_ABDUCT_LIMIT_RAD (25.0 * PI / 180.0)
#define HUMANOID_ANKLE_ROLL_LIMIT_RAD (20.0 * PI / 180.0)
#define HUMANOID_ANKLE_PITCH_LIMIT_RAD (25.0 * PI / 180.0)
#define HUMANOID_WAIST_ROLL_LIMIT_RAD (25.0 * PI / 180.0)
#define HUMANOID_WAIST_PITCH_LIMIT_RAD (30.0 * PI / 180.0)
/* 목(2026-11-18 추가, 사용자 요청 - "허리 관절과 비슷하면 될 것
   같아") - humanoid_walk.xml의 neck_roll/neck_pitch range와 동일
   (허리와 같은 값을 그대로 재사용). */
#define HUMANOID_NECK_ROLL_LIMIT_RAD (25.0 * PI / 180.0)
#define HUMANOID_NECK_PITCH_LIMIT_RAD (30.0 * PI / 180.0)
/* 어깨 ab/duction(2026-11-18 추가, 사용자 요청 - "옆으로도 벌릴
   수 있게") - humanoid_walk.xml의 shoulder_abduct_L/R range와 동일. */
#define HUMANOID_SHOULDER_ABDUCT_LIMIT_RAD (60.0 * PI / 180.0)
/* 발 두께(humanoid_walk.xml의 foot geom size z*2 = 0.02*2) - URDF
   쪽 발목 피벗을 땅바닥(z=0)이 아니라 그보다 이만큼 위에 둬야,
   종아리 메쉬(shin_ros.obj, 발목 피벗까지 꽉 차게 내려옴)와 발
   메쉬(foot_ros.obj, 피벗에서 위로 0~0.04 범위)가 안 겹친다 -
   안 그러면 발이 종아리 속에 파묻혀서 RViz에 전혀 안 보인다. */
#define HUMANOID_ANKLE_HEIGHT 0.05

/* 부위별 색 - 한 곳에서만 정의한다. write_scene()(humanoid_scene.txt,
   OpenGL 로더 버전이 읽음), write_humanoid_urdf()/write_quadruped_urdf()
   (URDF <material>), main()의 bake_material_into_obj() 호출(*_ros.obj에
   박아 넣는 .mtl)이 전부 이 상수들만 참조한다. 예전에는 이 세 군데에
   같은 색을 리터럴로 따로따로 적어서, 색을 바꾸려면 세 곳을 전부
   손으로 맞춰야 했다 - 손으로 맞추다 보면 어긋나기 쉬우므로
   한 곳으로 모았다. */
typedef struct { double r, g, b; } Color;

static const Color COLOR_BODY      = {0.55, 0.55, 0.65};
static const Color COLOR_HEAD      = {0.85, 0.75, 0.65};
static const Color COLOR_ARM       = {0.45, 0.45, 0.55};
static const Color COLOR_LEG       = {0.35, 0.35, 0.45};
static const Color COLOR_FOOT      = {0.20, 0.20, 0.25};
static const Color COLOR_NOSE      = {0.85, 0.25, 0.25};
static const Color COLOR_QUAD_BODY = {0.50, 0.55, 0.60};
static const Color COLOR_QUAD_HEAD = {0.60, 0.62, 0.65};
static const Color COLOR_QUAD_LEG  = {0.35, 0.35, 0.40};

static void write_quad(FILE *f, int *vcount,
                        double p1[3], double p2[3], double p3[3], double p4[3],
                        double n[3]) {
    fprintf(f, "v %f %f %f\n", p1[0], p1[1], p1[2]);
    fprintf(f, "v %f %f %f\n", p2[0], p2[1], p2[2]);
    fprintf(f, "v %f %f %f\n", p3[0], p3[1], p3[2]);
    fprintf(f, "v %f %f %f\n", p4[0], p4[1], p4[2]);
    fprintf(f, "vn %f %f %f\n", n[0], n[1], n[2]);
    int v1 = *vcount + 1, v2 = *vcount + 2, v3 = *vcount + 3, v4 = *vcount + 4;
    int vn = v1; /* 4개 정점이 이 vn 하나를 공유 (per-face normal) */
    /* vn 인덱스는 v와 별도로 세어야 하지만, 여기서는 quad마다 정확히
       1개의 vn만 추가되므로 (*vcount/4 + 1)번째 vn이 된다 */
    int vn_index = (*vcount) / 4 + 1;
    (void)vn;
    fprintf(f, "f %d//%d %d//%d %d//%d\n", v1, vn_index, v2, vn_index, v3, vn_index);
    fprintf(f, "f %d//%d %d//%d %d//%d\n", v1, vn_index, v3, vn_index, v4, vn_index);
    *vcount += 4;
}

/* 상자(box) 메쉬를 OBJ로 저장. xmin..zmax로 범위를 지정 (원점 기준 상대 좌표) */
static void write_box_obj(const char *filename,
                           double xmin, double xmax,
                           double ymin, double ymax,
                           double zmin, double zmax) {
    FILE *f = fopen(filename, "w");
    if (!f) { perror(filename); return; }
    fprintf(f, "# box mesh generated by gen_humanoid_assets.c\n");

    int vcount = 0;
    double x0 = xmin, x1 = xmax, y0 = ymin, y1 = ymax, z0 = zmin, z1 = zmax;

    double A[3] = {x0, y0, z0}, B[3] = {x1, y0, z0};
    double C[3] = {x1, y1, z0}, D[3] = {x0, y1, z0};
    double E[3] = {x0, y0, z1}, G[3] = {x1, y0, z1};
    double Hh[3] = {x1, y1, z1}, I[3] = {x0, y1, z1};

    double nZm[3] = {0, 0, -1}, nZp[3] = {0, 0, 1};
    double nXm[3] = {-1, 0, 0}, nXp[3] = {1, 0, 0};
    double nYm[3] = {0, -1, 0}, nYp[3] = {0, 1, 0};

    write_quad(f, &vcount, A, D, C, B, nZm);   /* -Z */
    write_quad(f, &vcount, E, G, Hh, I, nZp);  /* +Z */
    write_quad(f, &vcount, A, E, I, D, nXm);   /* -X */
    write_quad(f, &vcount, B, C, Hh, G, nXp);  /* +X */
    write_quad(f, &vcount, A, B, G, E, nYm);   /* -Y */
    write_quad(f, &vcount, D, I, Hh, C, nYp);  /* +Y */

    fclose(f);
    printf("wrote %s (%d vertices)\n", filename, vcount);
}

/* 타원기둥(elliptic cylinder) 메쉬를 Z-up으로 저장(ROS 전용,
   write_sphere_obj_zup과 같은 이유 - *_ros.obj만 쓴다. OpenGL
   데모는 그대로 box 유지). z0..z1 구간에 서있는 형태(z0<z1이면
   위로, 다리/팔처럼 원점에서 매달리게 하려면 z0=-length, z1=0으로
   호출) - write_box_obj의 zmin/zmax 관례와 동일. 원래는 원형
   (rx=ry)이었는데, 사용자 요청("가슴과 원통도 타원형이면 이쁠
   텐데")으로 rx!=ry를 받아 타원 단면으로 만들 수 있게 일반화했다.
   옆면 법선은 타원의 실제 바깥쪽 방향(음함수 x²/rx²+y²/ry²=1의
   그래디언트 (cosθ/rx, sinθ/ry)를 정규화) - rx=ry일 때는 기존
   원기둥과 동일한 결과. 옆면은 box와 같은 스타일로 슬라이스마다
   평평한 사각형(저폴리곤), 위/아래 뚜껑은 slices각형 하나씩. */
static void write_elliptic_cylinder_obj_zup(const char *filename, double rx, double ry,
                                             double z0, double z1, int slices) {
    FILE *f = fopen(filename, "w");
    if (!f) { perror(filename); return; }
    fprintf(f, "# elliptic cylinder mesh generated by gen_humanoid_assets.c\n");

    int vcount = 0;
    int vn_count = 0;

    for (int i = 0; i < slices; i++) {
        double a0 = 2.0 * PI * i / slices;
        double a1 = 2.0 * PI * (i + 1) / slices;
        double x0 = rx * cos(a0), y0s = ry * sin(a0);
        double x1 = rx * cos(a1), y1s = ry * sin(a1);
        double amid = 0.5 * (a0 + a1);
        double gnx = cos(amid) / rx, gny = sin(amid) / ry;
        double glen = sqrt(gnx * gnx + gny * gny);
        double nx = gnx / glen, ny = gny / glen;

        fprintf(f, "v %f %f %f\n", x0, y0s, z0);
        fprintf(f, "v %f %f %f\n", x1, y1s, z0);
        fprintf(f, "v %f %f %f\n", x1, y1s, z1);
        fprintf(f, "v %f %f %f\n", x0, y0s, z1);
        fprintf(f, "vn %f %f 0.0\n", nx, ny);
        vn_count++;
        int v1 = vcount + 1, v2 = vcount + 2, v3 = vcount + 3, v4 = vcount + 4;
        fprintf(f, "f %d//%d %d//%d %d//%d\n", v1, vn_count, v2, vn_count, v3, vn_count);
        fprintf(f, "f %d//%d %d//%d %d//%d\n", v1, vn_count, v3, vn_count, v4, vn_count);
        vcount += 4;
    }

    /* z1이 항상 "위"(호출 쪽 관례: 매달린 형태는 z0=-length,z1=0 /
       서있는 형태는 z0=0,z1=height - 둘 다 z1>z0) - 위쪽 뚜껑(z1,
       법선 +Z)은 증가 순서(바깥/위에서 봤을 때 반시계), 아래쪽
       뚜껑(z0, 법선 -Z)은 감소 순서로 감아야 바깥쪽 법선과
       일치한다. */
    int top_start = vcount + 1;
    for (int i = 0; i < slices; i++) {
        double a = 2.0 * PI * i / slices;
        fprintf(f, "v %f %f %f\n", rx * cos(a), ry * sin(a), z1);
    }
    fprintf(f, "vn 0 0 1\n");
    vn_count++;
    fprintf(f, "f");
    for (int i = 0; i < slices; i++) fprintf(f, " %d//%d", top_start + i, vn_count);
    fprintf(f, "\n");
    vcount += slices;

    int bot_start = vcount + 1;
    for (int i = slices - 1; i >= 0; i--) {
        double a = 2.0 * PI * i / slices;
        fprintf(f, "v %f %f %f\n", rx * cos(a), ry * sin(a), z0);
    }
    fprintf(f, "vn 0 0 -1\n");
    vn_count++;
    fprintf(f, "f");
    for (int i = 0; i < slices; i++) fprintf(f, " %d//%d", bot_start + i, vn_count);
    fprintf(f, "\n");
    vcount += slices;

    fclose(f);
    printf("wrote %s (%d vertices)\n", filename, vcount);
}

/* 타원기둥을 Y-up(ours, OpenGL 데모용)으로 저장 - write_elliptic_cylinder_obj_zup
   과 완전히 같은 로직인데 축만 Z<->Y로 바꿨다(단면이 XZ 평면, Y축
   방향으로 뻗음) - write_box_obj의 Y-up 관례(다리/팔은 Y로 매달림,
   몸통은 Y로 센터링)와 맞추려고. 2026-11-17: 사용자 요청("OpenGL
   쪽도 URDF/MJCF처럼 원기둥으로 통일") - 관절 구조는 안 건드리고
   leg.obj/shin.obj/arm.obj/body.obj의 모양만 box에서 바꾼다. */
static void write_elliptic_cylinder_obj(const char *filename, double rx, double rz,
                                         double y0, double y1, int slices) {
    FILE *f = fopen(filename, "w");
    if (!f) { perror(filename); return; }
    fprintf(f, "# elliptic cylinder mesh generated by gen_humanoid_assets.c\n");

    int vcount = 0;
    int vn_count = 0;

    for (int i = 0; i < slices; i++) {
        double a0 = 2.0 * PI * i / slices;
        double a1 = 2.0 * PI * (i + 1) / slices;
        double x0 = rx * cos(a0), z0s = rz * sin(a0);
        double x1 = rx * cos(a1), z1s = rz * sin(a1);
        double amid = 0.5 * (a0 + a1);
        double gnx = cos(amid) / rx, gnz = sin(amid) / rz;
        double glen = sqrt(gnx * gnx + gnz * gnz);
        double nx = gnx / glen, nz = gnz / glen;

        fprintf(f, "v %f %f %f\n", x0, y0, z0s);
        fprintf(f, "v %f %f %f\n", x1, y0, z1s);
        fprintf(f, "v %f %f %f\n", x1, y1, z1s);
        fprintf(f, "v %f %f %f\n", x0, y1, z0s);
        fprintf(f, "vn %f 0.0 %f\n", nx, nz);
        vn_count++;
        int v1 = vcount + 1, v2 = vcount + 2, v3 = vcount + 3, v4 = vcount + 4;
        fprintf(f, "f %d//%d %d//%d %d//%d\n", v1, vn_count, v2, vn_count, v3, vn_count);
        fprintf(f, "f %d//%d %d//%d %d//%d\n", v1, vn_count, v3, vn_count, v4, vn_count);
        vcount += 4;
    }

    /* y1이 "위"(bot/top이 아니라 box 관례를 따라 그냥 ymin/ymax) -
       y1쪽 뚜껑(법선 +Y)은 증가 순서, y0쪽 뚜껑(법선 -Y)은 감소
       순서로 감아야 바깥쪽 법선과 일치한다(write_elliptic_cylinder_obj_zup
       의 z0/z1 규칙과 동일, Y로 축만 바뀜). **이 Y-up 버전은
       humanoid_loader_test.c의 손으로 짠 OBJ 파서가 소비하는데, 그
       파서는 삼각형(f a//n b//n c//n) 3개짜리 줄만 읽는다(n각형
       줄은 앞 3개만 읽고 나머지 무시 - 캡이 깨져 보임) - 그래서
       여기(및 옆면)는 n각형 face 한 줄 대신 전부 삼각형 fan으로
       쪼갠다. write_elliptic_cylinder_obj_zup(URDF용)은 Assimp가
       n각형을 알아서 삼각분할해주는 걸 RViz2에서 이미 확인해서
       그대로 둠. */
    int top_start = vcount + 1;
    for (int i = 0; i < slices; i++) {
        double a = 2.0 * PI * i / slices;
        fprintf(f, "v %f %f %f\n", rx * cos(a), y1, rz * sin(a));
    }
    fprintf(f, "vn 0 1 0\n");
    vn_count++;
    for (int i = 1; i < slices - 1; i++) {
        fprintf(f, "f %d//%d %d//%d %d//%d\n",
                top_start, vn_count, top_start + i, vn_count, top_start + i + 1, vn_count);
    }
    vcount += slices;

    int bot_start = vcount + 1;
    for (int i = slices - 1; i >= 0; i--) {
        double a = 2.0 * PI * i / slices;
        fprintf(f, "v %f %f %f\n", rx * cos(a), y0, rz * sin(a));
    }
    fprintf(f, "vn 0 -1 0\n");
    vn_count++;
    for (int i = 1; i < slices - 1; i++) {
        fprintf(f, "f %d//%d %d//%d %d//%d\n",
                bot_start, vn_count, bot_start + i, vn_count, bot_start + i + 1, vn_count);
    }
    vcount += slices;

    fclose(f);
    printf("wrote %s (%d vertices)\n", filename, vcount);
}

/* 구(sphere) 메쉬를 OBJ로 저장. 중심이 (0, center_y, 0)에 오도록 오프셋 적용 */
static void write_sphere_obj(const char *filename, double radius, double center_y,
                              int slices, int stacks) {
    FILE *f = fopen(filename, "w");
    if (!f) { perror(filename); return; }
    fprintf(f, "# sphere mesh generated by gen_humanoid_assets.c\n");

    int rows = stacks + 1;
    int cols = slices + 1;

    for (int i = 0; i < rows; i++) {
        double phi = -PI / 2.0 + PI * i / stacks;
        for (int j = 0; j < cols; j++) {
            double theta = 2.0 * PI * j / slices;
            double nx = cos(phi) * cos(theta);
            double ny = sin(phi);
            double nz = cos(phi) * sin(theta);
            double x = radius * nx;
            double y = center_y + radius * ny;
            double z = radius * nz;
            fprintf(f, "v %f %f %f\n", x, y, z);
            fprintf(f, "vn %f %f %f\n", nx, ny, nz);
        }
    }

    for (int i = 0; i < stacks; i++) {
        for (int j = 0; j < slices; j++) {
            int a = i * cols + j + 1;       /* 1-based */
            int b = i * cols + (j + 1) + 1;
            int c = (i + 1) * cols + (j + 1) + 1;
            int d = (i + 1) * cols + j + 1;
            fprintf(f, "f %d//%d %d//%d %d//%d\n", a, a, b, b, c, c);
            fprintf(f, "f %d//%d %d//%d %d//%d\n", a, a, c, c, d, d);
        }
    }

    fclose(f);
    printf("wrote %s (%d vertices)\n", filename, rows * cols);
}

/* write_sphere_obj()와 똑같지만 극축이 Y가 아니라 Z다 - URDF/ROS의
   Z-up 좌표계(ros_z = ours_y)에 맞춰 head_ros.obj를 만들 때 쓴다.
   write_box_obj()는 상자라서 그냥 min/max 인자 순서만 바꿔서 재사용할
   수 있지만(아래 main() 참고), 구는 극축이 코드에 박혀 있어서 별도
   함수가 필요하다. */
static void write_sphere_obj_zup(const char *filename, double radius, double center_z,
                                   int slices, int stacks) {
    FILE *f = fopen(filename, "w");
    if (!f) { perror(filename); return; }
    fprintf(f, "# sphere mesh (Z-up, for URDF/ROS) generated by gen_humanoid_assets.c\n");

    int rows = stacks + 1;
    int cols = slices + 1;

    for (int i = 0; i < rows; i++) {
        double phi = -PI / 2.0 + PI * i / stacks;
        for (int j = 0; j < cols; j++) {
            double theta = 2.0 * PI * j / slices;
            double nx = cos(phi) * cos(theta);
            double ny = cos(phi) * sin(theta);
            double nz = sin(phi);
            double x = radius * nx;
            double y = radius * ny;
            double z = center_z + radius * nz;
            fprintf(f, "v %f %f %f\n", x, y, z);
            fprintf(f, "vn %f %f %f\n", nx, ny, nz);
        }
    }

    for (int i = 0; i < stacks; i++) {
        for (int j = 0; j < slices; j++) {
            int a = i * cols + j + 1;
            int b = i * cols + (j + 1) + 1;
            int c = (i + 1) * cols + (j + 1) + 1;
            int d = (i + 1) * cols + j + 1;
            fprintf(f, "f %d//%d %d//%d %d//%d\n", a, a, b, b, c, c);
            fprintf(f, "f %d//%d %d//%d %d//%d\n", a, a, c, c, d, d);
        }
    }

    fclose(f);
    printf("wrote %s (%d vertices)\n", filename, rows * cols);
}

/* ---- *_ros.obj에 색을 박아 넣기 ----
   *_ros.obj는 v/vn/f만 있고 재질(material) 정보가 전혀 없어서,
   RViz2(Ogre/assimp 로더)가 URDF의 <material> 색을 무시하고 OBJ를
   기본 회/백색 민무늬로 그려버리는 경우가 있었다 ("스킨이 안
   입혀졌다"). 그래서 메쉬 파일 자체에도 똑같은 색을 가진 .mtl을
   붙여서, URDF 쪽 재질 처리가 어떻게 되든 메쉬 자체가 항상 맞는
   색을 들고 있게 한다. */
static void write_mtl(const char *filename, double r, double g, double b) {
    FILE *f = fopen(filename, "w");
    if (!f) { perror(filename); return; }
    fprintf(f, "# material generated by gen_humanoid_assets.c\n");
    fprintf(f, "newmtl mat0\n");
    fprintf(f, "Ka 0.0 0.0 0.0\n");
    fprintf(f, "Kd %f %f %f\n", r, g, b);
    fprintf(f, "Ks 0.0 0.0 0.0\n");
    fprintf(f, "d 1.0\n");
    fprintf(f, "illum 1\n");
    fclose(f);
    printf("wrote %s\n", filename);
}

/* mtl_filename(.mtl)을 쓰고, obj_filename 맨 위(첫 주석 줄 다음)에
   "mtllib .../usemtl mat0"을 끼워 넣는다. */
static void bake_material_into_obj(const char *obj_filename, const char *mtl_filename,
                                     double r, double g, double b) {
    const char *slash1 = strrchr(mtl_filename, '/');
    const char *slash2 = strrchr(mtl_filename, '\\');
    const char *base = mtl_filename;
    if (slash1 && slash1 + 1 > base) base = slash1 + 1;
    if (slash2 && slash2 + 1 > base) base = slash2 + 1;

    write_mtl(mtl_filename, r, g, b);

    FILE *in = fopen(obj_filename, "r");
    if (!in) { perror(obj_filename); return; }

    char *buf = NULL;
    size_t cap = 0, len = 0;
    char line[512];
    while (fgets(line, sizeof(line), in)) {
        size_t l = strlen(line);
        if (len + l + 1 > cap) {
            size_t newcap = (cap == 0) ? 4096 : cap * 2;
            while (newcap < len + l + 1) newcap *= 2;
            buf = realloc(buf, newcap);
            cap = newcap;
        }
        memcpy(buf + len, line, l);
        len += l;
        buf[len] = '\0';
    }
    fclose(in);

    FILE *out = fopen(obj_filename, "w");
    if (!out) { perror(obj_filename); free(buf); return; }

    char *first_newline = memchr(buf, '\n', len);
    if (first_newline) {
        size_t first_line_len = (size_t)(first_newline - buf) + 1;
        fwrite(buf, 1, first_line_len, out);
        fprintf(out, "mtllib %s\n", base);
        fprintf(out, "usemtl mat0\n");
        fwrite(buf + first_line_len, 1, len - first_line_len, out);
    } else {
        fprintf(out, "mtllib %s\n", base);
        fprintf(out, "usemtl mat0\n");
        fwrite(buf, 1, len, out);
    }
    fclose(out);
    free(buf);
    printf("baked material into %s (mtllib %s)\n", obj_filename, base);
}

static void write_scene(const char *filename) {
    FILE *f = fopen(filename, "w");
    if (!f) { perror(filename); return; }
    fprintf(f, "# name     mesh       parent  tx       ty       tz  joint  r     g     b\n");
    fprintf(f, "Body       body.obj   ROOT    0.0      %.4f    0.0  NONE   %.2f  %.2f  %.2f\n",
            BODY_H / 2.0, COLOR_BODY.r, COLOR_BODY.g, COLOR_BODY.b);
    fprintf(f, "Head       head.obj   Body    0.0      %.4f    0.0  NONE   %.2f  %.2f  %.2f\n",
            BODY_H / 2.0, COLOR_HEAD.r, COLOR_HEAD.g, COLOR_HEAD.b);
    fprintf(f, "LeftArm    arm.obj    Body    %.4f     %.4f    0.0  ROTX   %.2f  %.2f  %.2f\n",
            -SHOULDER_X, BODY_H / 2.0, COLOR_ARM.r, COLOR_ARM.g, COLOR_ARM.b);
    fprintf(f, "RightArm   arm.obj    Body    %.4f     %.4f    0.0  ROTX   %.2f  %.2f  %.2f\n",
            SHOULDER_X, BODY_H / 2.0, COLOR_ARM.r, COLOR_ARM.g, COLOR_ARM.b);
    fprintf(f, "LeftLeg    leg.obj    Body    %.4f     %.4f    0.0  ROTX   %.2f  %.2f  %.2f\n",
            -HIP_X, -BODY_H / 2.0, COLOR_LEG.r, COLOR_LEG.g, COLOR_LEG.b);
    fprintf(f, "RightLeg   leg.obj    Body    %.4f     %.4f    0.0  ROTX   %.2f  %.2f  %.2f\n",
            HIP_X, -BODY_H / 2.0, COLOR_LEG.r, COLOR_LEG.g, COLOR_LEG.b);

    /* 종아리(무릎 아래) - 부모가 Body가 아니라 각자의 LeftLeg/RightLeg
       (허벅지)다. 4족의 QuadShinFL과 같은 패턴 - 자식의 오프셋은
       부모(허벅지) 로컬 프레임 기준이라, 엉덩이가 돌아가도 종아리
       부착점이 자동으로 같이 따라간다. */
    fprintf(f, "LeftShin   shin.obj   LeftLeg  0.0     %.4f    0.0  ROTX   %.2f  %.2f  %.2f\n",
            -THIGH_LEN, COLOR_LEG.r, COLOR_LEG.g, COLOR_LEG.b);
    fprintf(f, "RightShin  shin.obj   RightLeg 0.0     %.4f    0.0  ROTX   %.2f  %.2f  %.2f\n",
            -THIGH_LEN, COLOR_LEG.r, COLOR_LEG.g, COLOR_LEG.b);

    /* 4족 로봇: QUAD_ROOT는 humanoid의 ROOT와는 별개의 가상 부모다.
       humanoid_loader_test.c가 draw_quadruped()에서
       render_children("QUAD_ROOT")를 직접 호출하면서 자기만의
       이동/회전(순찰 위치)을 미리 곱해두기 때문에, humanoid의 걷기
       변환과는 전혀 섞이지 않는다. */
    fprintf(f, "QuadBody   quad_body.obj  QUAD_ROOT  0.0      %.4f    0.0  NONE   %.2f  %.2f  %.2f\n",
            QUAD_BODY_H / 2.0, COLOR_QUAD_BODY.r, COLOR_QUAD_BODY.g, COLOR_QUAD_BODY.b);
    fprintf(f, "QuadHead   quad_head.obj  QuadBody   0.0      %.4f    %.4f  NONE   %.2f  %.2f  %.2f\n",
            QUAD_BODY_H * 0.15, QUAD_BODY_L / 2.0 + QUAD_HEAD_SIZE * 0.45,
            COLOR_QUAD_HEAD.r, COLOR_QUAD_HEAD.g, COLOR_QUAD_HEAD.b);
    /* ty(두번째 열, Y-up이라 수직축)는 원래 리터럴 0.0이었는데, 그러면
       다리가 QuadBody의 "중심"(상자 메쉬 원점)에 매달려서 몸통 높이의
       절반(QUAD_BODY_H/2)만큼 로봇 전체가 땅 위로 떠 보이는 버그가
       있었다 - humanoid의 LeftLeg/RightLeg는 이미 -BODY_H/2를 쓰고
       있어서(몸통 "바닥"에 다리를 매닮) 비교해서 발견했다. 같은
       방식으로 -QUAD_BODY_H/2를 줘서 다리가 몸통 바닥에서 매달리게
       고쳤다. */
    fprintf(f, "QuadLegFL  quad_leg.obj   QuadBody   %.4f     %.4f     %.4f  ROTX   %.2f  %.2f  %.2f\n",
            -QUAD_HIP_X, -QUAD_BODY_H / 2.0, QUAD_HIP_Z_FRONT, COLOR_QUAD_LEG.r, COLOR_QUAD_LEG.g, COLOR_QUAD_LEG.b);
    fprintf(f, "QuadLegFR  quad_leg.obj   QuadBody   %.4f     %.4f     %.4f  ROTX   %.2f  %.2f  %.2f\n",
            QUAD_HIP_X, -QUAD_BODY_H / 2.0, QUAD_HIP_Z_FRONT, COLOR_QUAD_LEG.r, COLOR_QUAD_LEG.g, COLOR_QUAD_LEG.b);
    fprintf(f, "QuadLegBL  quad_leg.obj   QuadBody   %.4f     %.4f     %.4f  ROTX   %.2f  %.2f  %.2f\n",
            -QUAD_HIP_X, -QUAD_BODY_H / 2.0, QUAD_HIP_Z_BACK, COLOR_QUAD_LEG.r, COLOR_QUAD_LEG.g, COLOR_QUAD_LEG.b);
    fprintf(f, "QuadLegBR  quad_leg.obj   QuadBody   %.4f     %.4f     %.4f  ROTX   %.2f  %.2f  %.2f\n",
            QUAD_HIP_X, -QUAD_BODY_H / 2.0, QUAD_HIP_Z_BACK, COLOR_QUAD_LEG.r, COLOR_QUAD_LEG.g, COLOR_QUAD_LEG.b);

    /* 종아리(무릎 아래) - 부모가 QuadBody가 아니라 각자의 QuadLegXX
       (허벅지)다. 자식의 (tx,ty,tz) 오프셋은 부모의 회전(엉덩이
       각도)까지 적용된 뒤의 부모 로컬 프레임 기준이라 - 허벅지 끝
       (로컬 -y 방향으로 QUAD_THIGH_LEN만큼)에 그대로 넣으면, 엉덩이가
       돌아가도 무릎 부착점이 자동으로 같이 따라간다(순방향 기구학). */
    fprintf(f, "QuadShinFL quad_shin.obj  QuadLegFL  0.0      %.4f    0.0  ROTX   %.2f  %.2f  %.2f\n",
            -QUAD_THIGH_LEN, COLOR_QUAD_LEG.r, COLOR_QUAD_LEG.g, COLOR_QUAD_LEG.b);
    fprintf(f, "QuadShinFR quad_shin.obj  QuadLegFR  0.0      %.4f    0.0  ROTX   %.2f  %.2f  %.2f\n",
            -QUAD_THIGH_LEN, COLOR_QUAD_LEG.r, COLOR_QUAD_LEG.g, COLOR_QUAD_LEG.b);
    fprintf(f, "QuadShinBL quad_shin.obj  QuadLegBL  0.0      %.4f    0.0  ROTX   %.2f  %.2f  %.2f\n",
            -QUAD_THIGH_LEN, COLOR_QUAD_LEG.r, COLOR_QUAD_LEG.g, COLOR_QUAD_LEG.b);
    fprintf(f, "QuadShinBR quad_shin.obj  QuadLegBR  0.0      %.4f    0.0  ROTX   %.2f  %.2f  %.2f\n",
            -QUAD_THIGH_LEN, COLOR_QUAD_LEG.r, COLOR_QUAD_LEG.g, COLOR_QUAD_LEG.b);

    fclose(f);
    printf("wrote %s\n", filename);
}

/* ============================================================
 * URDF + YAML 내보내기
 * ------------------------------------------------------------
 * humanoid_test.c / humanoid_loader_test.c는 Y-up 좌표계를 쓴다
 * (X=왼쪽, Y=위, Z=앞 - 방향키 좌/우 수정 때 확인한 바로 그 축).
 * ROS/URDF는 REP-103 규약대로 Z-up이다 (X=앞, Y=왼쪽, Z=위).
 * 그래서 모든 위치를 ros_xyz = (ours_z, ours_x, ours_y)로 축을
 * 바꿔서 내보낸다 - 이건 두 우수(right-handed) 좌표계 사이의
 * 순수한 회전(행렬식 +1)이라 회전 방향(각도의 부호)은 그대로
 * 유지되고, 좌표축 이름만 재배치된다. 우리 몸의 ROTX(어깨/엉덩이
 * pitch) 회전은 ours의 X축이 ros의 Y축이 되므로 URDF에서는
 * axis="0 1 0"이 된다.
 *
 * 메쉬 파일도 같은 변환을 적용한 *_ros.obj를 새로 만든다. 상자는
 * write_box_obj()에 min/max 인자 순서만 바꿔서 넣으면 축이 재배치된
 * 메쉬가 그대로 나오고(아래 main() 참고), 구는 극축이 코드에 박혀
 * 있어서 write_sphere_obj_zup()을 따로 썼다. 원래 쓰던 body.obj 등은
 * 그대로 Y-up으로 둬서 OpenGL 데모에 영향이 없다.
 *
 * URDF가 표현하지 못하는 정보(절차적 텍스처, 캐릭터별 옷 색 틴트,
 * 걷기/트롯 애니메이션 파라미터, 경로 파일 등)는 URDF 옆에 같이
 * 적어주는 *_extra.yaml에 담는다.
 * ============================================================ */

static void write_humanoid_urdf(const char *filename) {
    FILE *f = fopen(filename, "w");
    if (!f) { perror(filename); return; }

    double base_to_body_z = LEG_LEN + BODY_H / 2.0;
    /* 머리는 가슴(Chest) 원점(가슴 상자/원기둥 바닥)에서 BODY_H/2만큼
       위 = 가슴 꼭대기. head_ros.obj(write_sphere_obj_zup 호출의
       center_z=HEAD_R)는 "바닥이 원점" 관례로 이미 구워져 있어서
       (write_box_obj 계열의 leg_ros.obj 등과 같은 방식) 추가로
       HEAD_R을 더하면 안 된다 - 한 번 더하면 머리가 몸통 위로
       HEAD_R만큼 붕 뜬다(2026-11-17, 사용자 보고로 발견).
       **반면 MJCF(humanoid_walk.xml)의 head_geom은 MuJoCo 기본
       sphere(중심 기준 pos)라 관례가 다르다 - 거기는 BODY_H/2+HEAD_R
       (=1.05)이 맞다, 혼동해서 여기까지 똑같이 고치면 안 됨.** */
    double body_to_head_z = BODY_H / 2.0;
    double nose_x = HEAD_R * 0.85;
    /* 이 코 <visual>은 "Head" 링크 안에 있어서, nose_z는 Head
       자신의 원점 기준이다 - 그런데 Head 원점은 머리 구(sphere)의
       "중심"이 아니라 "바닥"(목 붙는 지점, body_to_head_z 주석
       참고 - head_ros.obj가 바닥 기준으로 구워져 있음)이다. 한때
       "이마에 붙었다"를 고친다고 0으로 낮췄는데, 그러면 오히려
       목 위치(머리 바닥)로 가버린다 - 0은 "Head 원점과 같은 높이"
       라는 뜻이기 때문. 얼굴 중앙(머리 바닥에서 반지름만큼 위)에
       오려면 HEAD_R이 맞다 - 원래 값으로 되돌림(2026-11-18, RViz2
       에서 "코가 목에 있다"는 사용자 보고로 발견). **MJCF
       (humanoid_walk.xml)의 nose_geom은 head_geom과 같은 "중심
       기준" pos를 쓰므로 거기는 머리 중심과 같은 높이(오프셋 0)가
       맞다 - 혼동하지 말 것(머리-가슴 간격 버그 때와 같은 함정).** */
    double nose_z = HEAD_R;
    double arm_z = BODY_H / 2.0;
    double leg_z = -BODY_H / 2.0;

    fprintf(f, "<?xml version=\"1.0\"?>\n");
    fprintf(f, "<!--\n");
    fprintf(f, "  gen_humanoid_assets.c가 생성. humanoid_test.c / humanoid_loader_test.c가\n");
    fprintf(f, "  쓰는 Y-up 좌표계(X=왼쪽, Y=위, Z=앞)를 ROS REP-103 Z-up 규약\n");
    fprintf(f, "  (X=앞, Y=왼쪽, Z=위)으로 축을 바꿔(ros_xyz = (ours_z, ours_x, ours_y))\n");
    fprintf(f, "  내보낸 것이다. 메쉬도 전부 *_ros.obj로 같은 축 변환을 적용한 별도\n");
    fprintf(f, "  버전을 쓴다 (원래 OpenGL 데모가 쓰는 body.obj 등은 Y-up 그대로 둔다).\n");
    fprintf(f, "  URDF가 담지 못하는 정보(텍스처 종류, 캐릭터별 옷 색 틴트, 걷기\n");
    fprintf(f, "  경로/애니메이션 파라미터 등)는 humanoid_extra.yaml을 같이 참고할 것.\n");
    fprintf(f, "-->\n");
    fprintf(f, "<robot name=\"humanoid\">\n\n");

    fprintf(f, "  <link name=\"base_link\"/>\n\n");

    fprintf(f, "  <joint name=\"base_to_body\" type=\"fixed\">\n");
    fprintf(f, "    <parent link=\"base_link\"/>\n");
    fprintf(f, "    <child link=\"Body\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 %.4f\" rpy=\"0 0 0\"/>\n", base_to_body_z);
    fprintf(f, "  </joint>\n\n");

    /* Body = 골반(하체) - 예전엔 몸통 전체(허리 위까지)를 한 상자로
       그렸지만, 좌우/전후 밸런스 실험(humanoid_stand_lqr.py)에서
       허리를 따로 굽혀야 한다는 게 확인돼서 하체(Body)/상체(Chest)로
       나눴다. Body의 원점 자체는 그대로(다리 부착 기준점 leg_z=
       -BODY_H/2 등 전부 안 바뀜) - 시각적으로 상자를 반으로 나눈
       것뿐. */
    fprintf(f, "  <link name=\"Body\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "pelvis_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"Body_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_BODY.r, COLOR_BODY.g, COLOR_BODY.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    /* 허리(waist) - 엉덩이/발목과 같은 패턴: 롤(바깥쪽 관절, 좌우
       굽힘)이 먼저, 피치(안쪽 관절, 앞뒤 굽힘)가 나중 - ChestRoll은
       base_link처럼 visual 없는 순수 연결용 링크. 원점은 Body
       원점과 동일(옛 몸통 상자의 정중앙 높이) - 그래서 body_to_head/
       body_to_leftarm/rightarm이 쓰던 arm_z=BODY_H/2, body_to_head_z=
       BODY_H/2 공식을 그대로(parent만 Chest로) 재사용할 수 있다. */
    fprintf(f, "  <joint name=\"Body_to_ChestRoll\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"Body\"/>\n");
    fprintf(f, "    <child link=\"ChestRoll\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n");
    fprintf(f, "    <axis xyz=\"1 0 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"20\" velocity=\"2\"/>\n",
            HUMANOID_WAIST_ROLL_LIMIT_RAD, HUMANOID_WAIST_ROLL_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"ChestRoll\"/>\n\n");

    fprintf(f, "  <joint name=\"ChestRoll_to_Chest\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"ChestRoll\"/>\n");
    fprintf(f, "    <child link=\"Chest\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n");
    fprintf(f, "    <axis xyz=\"0 1 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"20\" velocity=\"2\"/>\n",
            HUMANOID_WAIST_PITCH_LIMIT_RAD, HUMANOID_WAIST_PITCH_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"Chest\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "chest_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"Chest_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_BODY.r, COLOR_BODY.g, COLOR_BODY.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    /* 목 - 허리와 같은 패턴(롤 바깥 먼저, 피치 안쪽 나중, 기존
       body_to_head 이름은 안쪽 피치 관절에 유지) - HeadRoll은
       ChestRoll처럼 visual 없는 순수 연결용 링크. 2026-11-18 추가
       (이전엔 fixed 관절이었음), 사용자 요청 - "허리 관절과 비슷하면
       될 것 같아". */
    fprintf(f, "  <joint name=\"Chest_to_HeadRoll\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"Chest\"/>\n");
    fprintf(f, "    <child link=\"HeadRoll\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 %.4f\" rpy=\"0 0 0\"/>\n", body_to_head_z);
    fprintf(f, "    <axis xyz=\"1 0 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_NECK_ROLL_LIMIT_RAD, HUMANOID_NECK_ROLL_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"HeadRoll\"/>\n\n");

    fprintf(f, "  <joint name=\"body_to_head\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"HeadRoll\"/>\n");
    fprintf(f, "    <child link=\"Head\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n");
    fprintf(f, "    <axis xyz=\"0 1 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_NECK_PITCH_LIMIT_RAD, HUMANOID_NECK_PITCH_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");

    fprintf(f, "  <link name=\"Head\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "head_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"Head_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_HEAD.r, COLOR_HEAD.g, COLOR_HEAD.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "    <!-- 코: humanoid_test.c의 draw_head()에서는 OBJ 메쉬가 아니라 매\n");
    fprintf(f, "         프레임 따로 그리던 작은 빨간 구였다. 처음엔 URDF <sphere>\n");
    fprintf(f, "         프리미티브로 넣었는데, 같은 링크(Head)에 메쉬 visual과 프리미티브\n");
    fprintf(f, "         visual이 같이 있을 때 RViz2가 프리미티브 쪽 <material> 색을\n");
    fprintf(f, "         무시하는 문제가 있어서(Head 메쉬 색으로 덮임), 다른 부위와\n");
    fprintf(f, "         똑같이 작은 메쉬(nose_ros.obj) + 거기에 색을 박아 넣은\n");
    fprintf(f, "         nose_ros.mtl 조합으로 바꿨다. -->\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <origin xyz=\"%.4f 0 %.4f\" rpy=\"0 0 0\"/>\n", nose_x, nose_z);
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "nose_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"Nose_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_NOSE.r, COLOR_NOSE.g, COLOR_NOSE.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    /* 어깨 ab/duction - 엉덩이/발목/허리와 같은 패턴: 벌림(바깥) 먼저,
       피치(안쪽, 기존 body_to_leftarm 이름 유지) 나중. */
    fprintf(f, "  <joint name=\"Chest_to_LeftShoulderAbduct\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"Chest\"/>\n");
    fprintf(f, "    <child link=\"LeftShoulderAbduct\"/>\n");
    fprintf(f, "    <origin xyz=\"0 %.4f %.4f\" rpy=\"0 0 0\"/>\n", -URDF_SHOULDER_X, arm_z);
    fprintf(f, "    <axis xyz=\"1 0 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_SHOULDER_ABDUCT_LIMIT_RAD, HUMANOID_SHOULDER_ABDUCT_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"LeftShoulderAbduct\"/>\n\n");

    fprintf(f, "  <joint name=\"body_to_leftarm\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"LeftShoulderAbduct\"/>\n");
    fprintf(f, "    <child link=\"LeftArm\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n");
    fprintf(f, "    <axis xyz=\"0 1 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n", ARM_JOINT_LIMIT_RAD, ARM_JOINT_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"LeftArm\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "arm_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"LeftArm_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_ARM.r, COLOR_ARM.g, COLOR_ARM.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    /* 팔꿈치(한쪽으로만 0~90도 굽혀짐, 부호는 무릎과 반대 - 무릎은
       양수 방향이 자연스러운 굽힘이었지만 팔꿈치는 사람처럼 앞으로
       굽으려면 음수 방향이어야 했다(2026-11-18, "팔꿈치가 뒤로
       꺾여"라는 사용자 보고로 발견 - humanoid_walk.xml에서 먼저
       확인/수정). 원점은 위팔 메쉬가 매달린 끝(-UPPER_ARM_LEN). */
    fprintf(f, "  <joint name=\"LeftArm_to_LeftForearm\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"LeftArm\"/>\n");
    fprintf(f, "    <child link=\"LeftForearmTwist\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 %.4f\" rpy=\"0 0 0\"/>\n", -UPPER_ARM_LEN);
    fprintf(f, "    <axis xyz=\"0 1 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"0\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_ELBOW_JOINT_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"LeftForearmTwist\"/>\n\n");

    /* 전완 회전 - 팔꿈치 굽힘(바깥) 다음, 아래팔 자신의 긴 축(로컬 Z)
       기준 비틀기(안쪽) - humanoid_walk.xml의 joint 순서와 동일. */
    fprintf(f, "  <joint name=\"LeftForearmTwist_to_LeftForearm\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"LeftForearmTwist\"/>\n");
    fprintf(f, "    <child link=\"LeftForearm\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n");
    fprintf(f, "    <axis xyz=\"0 0 1\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_ELBOW_TWIST_LIMIT_RAD, HUMANOID_ELBOW_TWIST_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"LeftForearm\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "forearm_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"LeftForearm_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_ARM.r, COLOR_ARM.g, COLOR_ARM.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    /* 손목 - 엉덩이/발목/허리/목과 같은 패턴(편위=롤, 바깥 먼저 /
       굽힘=피치, 안쪽 나중). 전완 회전은 이미 elbow_twist에서
       처리했으니 여기엔 비틀기 축 없음. */
    fprintf(f, "  <joint name=\"LeftForearm_to_LeftWristDeviate\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"LeftForearm\"/>\n");
    fprintf(f, "    <child link=\"LeftWristDeviate\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 %.4f\" rpy=\"0 0 0\"/>\n", -FOREARM_LEN);
    fprintf(f, "    <axis xyz=\"1 0 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_WRIST_DEVIATE_LIMIT_RAD, HUMANOID_WRIST_DEVIATE_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"LeftWristDeviate\"/>\n\n");

    fprintf(f, "  <joint name=\"LeftWristDeviate_to_LeftHand\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"LeftWristDeviate\"/>\n");
    fprintf(f, "    <child link=\"LeftHand\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n");
    fprintf(f, "    <axis xyz=\"0 1 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_WRIST_FLEX_LIMIT_RAD, HUMANOID_WRIST_FLEX_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"LeftHand\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "hand_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"LeftHand_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_HEAD.r, COLOR_HEAD.g, COLOR_HEAD.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    fprintf(f, "  <joint name=\"Chest_to_RightShoulderAbduct\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"Chest\"/>\n");
    fprintf(f, "    <child link=\"RightShoulderAbduct\"/>\n");
    fprintf(f, "    <origin xyz=\"0 %.4f %.4f\" rpy=\"0 0 0\"/>\n", URDF_SHOULDER_X, arm_z);
    fprintf(f, "    <axis xyz=\"1 0 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_SHOULDER_ABDUCT_LIMIT_RAD, HUMANOID_SHOULDER_ABDUCT_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"RightShoulderAbduct\"/>\n\n");

    fprintf(f, "  <joint name=\"body_to_rightarm\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"RightShoulderAbduct\"/>\n");
    fprintf(f, "    <child link=\"RightArm\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n");
    fprintf(f, "    <axis xyz=\"0 1 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n", ARM_JOINT_LIMIT_RAD, ARM_JOINT_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"RightArm\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "arm_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"RightArm_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_ARM.r, COLOR_ARM.g, COLOR_ARM.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    fprintf(f, "  <joint name=\"RightArm_to_RightForearm\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"RightArm\"/>\n");
    fprintf(f, "    <child link=\"RightForearmTwist\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 %.4f\" rpy=\"0 0 0\"/>\n", -UPPER_ARM_LEN);
    fprintf(f, "    <axis xyz=\"0 1 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"0\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_ELBOW_JOINT_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"RightForearmTwist\"/>\n\n");

    fprintf(f, "  <joint name=\"RightForearmTwist_to_RightForearm\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"RightForearmTwist\"/>\n");
    fprintf(f, "    <child link=\"RightForearm\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n");
    fprintf(f, "    <axis xyz=\"0 0 1\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_ELBOW_TWIST_LIMIT_RAD, HUMANOID_ELBOW_TWIST_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"RightForearm\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "forearm_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"RightForearm_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_ARM.r, COLOR_ARM.g, COLOR_ARM.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    fprintf(f, "  <joint name=\"RightForearm_to_RightWristDeviate\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"RightForearm\"/>\n");
    fprintf(f, "    <child link=\"RightWristDeviate\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 %.4f\" rpy=\"0 0 0\"/>\n", -FOREARM_LEN);
    fprintf(f, "    <axis xyz=\"1 0 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_WRIST_DEVIATE_LIMIT_RAD, HUMANOID_WRIST_DEVIATE_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"RightWristDeviate\"/>\n\n");

    fprintf(f, "  <joint name=\"RightWristDeviate_to_RightHand\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"RightWristDeviate\"/>\n");
    fprintf(f, "    <child link=\"RightHand\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n");
    fprintf(f, "    <axis xyz=\"0 1 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_WRIST_FLEX_LIMIT_RAD, HUMANOID_WRIST_FLEX_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"RightHand\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "hand_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"RightHand_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_HEAD.r, COLOR_HEAD.g, COLOR_HEAD.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    /* 엉덩이 ab/duction(좌우 벌림/모음) - 피치(body_to_leftleg)보다
       바깥쪽(몸통에 더 가까운) 관절. 기존 body_to_leftleg 이름/축/
       limit은 그대로 유지(parent만 Body->LeftHip, origin만 "0 0 0"
       으로) - humanoid_state_publisher.py 등 다른 소비자가 그
       이름을 그대로 쓸 수 있게. */
    fprintf(f, "  <joint name=\"Body_to_LeftHip\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"Body\"/>\n");
    fprintf(f, "    <child link=\"LeftHip\"/>\n");
    fprintf(f, "    <origin xyz=\"0 %.4f %.4f\" rpy=\"0 0 0\"/>\n", -HIP_X, leg_z);
    fprintf(f, "    <axis xyz=\"1 0 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_HIP_ABDUCT_LIMIT_RAD, HUMANOID_HIP_ABDUCT_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"LeftHip\"/>\n\n");

    fprintf(f, "  <joint name=\"body_to_leftleg\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"LeftHip\"/>\n");
    fprintf(f, "    <child link=\"LeftLeg\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n");
    fprintf(f, "    <axis xyz=\"0 1 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n", LEG_JOINT_LIMIT_RAD, LEG_JOINT_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"LeftLeg\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "leg_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"LeftLeg_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_LEG.r, COLOR_LEG.g, COLOR_LEG.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    /* 종아리(무릎 - 한쪽으로만 0~90도 굽혀짐) - 부모가 Body가 아니라
       방금 쓴 허벅지 링크고, 원점은 허벅지 메쉬가 매달린 끝
       (-THIGH_LEN) - 4족의 QuadLegFL_to_QuadShinFL과 동일한 패턴. */
    fprintf(f, "  <joint name=\"LeftLeg_to_LeftShin\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"LeftLeg\"/>\n");
    fprintf(f, "    <child link=\"LeftShin\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 %.4f\" rpy=\"0 0 0\"/>\n", -THIGH_LEN);
    fprintf(f, "    <axis xyz=\"0 1 0\"/>\n");
    fprintf(f, "    <limit lower=\"0\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_KNEE_JOINT_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"LeftShin\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "shin_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"LeftShin_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_LEG.r, COLOR_LEG.g, COLOR_LEG.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    /* 발목(ankle) - 엉덩이/허리와 같은 패턴: 롤(바깥) 먼저, 피치(안쪽)
       나중. 원점은 종아리 끝(-SHIN_LEN, 발목 축) - LeftLeg_to_LeftShin
       과 같은 원리. LeftFoot은 (ChestRoll/LeftHip과 달리) 실제로
       땅에 닿는 부위라 visual을 넣는다(foot_ros.obj). */
    fprintf(f, "  <joint name=\"LeftShin_to_LeftAnkleRoll\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"LeftShin\"/>\n");
    fprintf(f, "    <child link=\"LeftAnkleRoll\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 %.4f\" rpy=\"0 0 0\"/>\n", -(SHIN_LEN - HUMANOID_ANKLE_HEIGHT));
    fprintf(f, "    <axis xyz=\"1 0 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_ANKLE_ROLL_LIMIT_RAD, HUMANOID_ANKLE_ROLL_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"LeftAnkleRoll\"/>\n\n");

    fprintf(f, "  <joint name=\"LeftAnkleRoll_to_LeftFoot\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"LeftAnkleRoll\"/>\n");
    fprintf(f, "    <child link=\"LeftFoot\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n");
    fprintf(f, "    <axis xyz=\"0 1 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_ANKLE_PITCH_LIMIT_RAD, HUMANOID_ANKLE_PITCH_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"LeftFoot\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "foot_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"LeftFoot_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_FOOT.r, COLOR_FOOT.g, COLOR_FOOT.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    fprintf(f, "  <joint name=\"Body_to_RightHip\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"Body\"/>\n");
    fprintf(f, "    <child link=\"RightHip\"/>\n");
    fprintf(f, "    <origin xyz=\"0 %.4f %.4f\" rpy=\"0 0 0\"/>\n", HIP_X, leg_z);
    fprintf(f, "    <axis xyz=\"1 0 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_HIP_ABDUCT_LIMIT_RAD, HUMANOID_HIP_ABDUCT_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"RightHip\"/>\n\n");

    fprintf(f, "  <joint name=\"body_to_rightleg\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"RightHip\"/>\n");
    fprintf(f, "    <child link=\"RightLeg\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n");
    fprintf(f, "    <axis xyz=\"0 1 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n", LEG_JOINT_LIMIT_RAD, LEG_JOINT_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"RightLeg\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "leg_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"RightLeg_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_LEG.r, COLOR_LEG.g, COLOR_LEG.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    fprintf(f, "  <joint name=\"RightLeg_to_RightShin\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"RightLeg\"/>\n");
    fprintf(f, "    <child link=\"RightShin\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 %.4f\" rpy=\"0 0 0\"/>\n", -THIGH_LEN);
    fprintf(f, "    <axis xyz=\"0 1 0\"/>\n");
    fprintf(f, "    <limit lower=\"0\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_KNEE_JOINT_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"RightShin\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "shin_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"RightShin_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_LEG.r, COLOR_LEG.g, COLOR_LEG.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    fprintf(f, "  <joint name=\"RightShin_to_RightAnkleRoll\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"RightShin\"/>\n");
    fprintf(f, "    <child link=\"RightAnkleRoll\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 %.4f\" rpy=\"0 0 0\"/>\n", -(SHIN_LEN - HUMANOID_ANKLE_HEIGHT));
    fprintf(f, "    <axis xyz=\"1 0 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_ANKLE_ROLL_LIMIT_RAD, HUMANOID_ANKLE_ROLL_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"RightAnkleRoll\"/>\n\n");

    fprintf(f, "  <joint name=\"RightAnkleRoll_to_RightFoot\" type=\"revolute\">\n");
    fprintf(f, "    <parent link=\"RightAnkleRoll\"/>\n");
    fprintf(f, "    <child link=\"RightFoot\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n");
    fprintf(f, "    <axis xyz=\"0 1 0\"/>\n");
    fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
            HUMANOID_ANKLE_PITCH_LIMIT_RAD, HUMANOID_ANKLE_PITCH_LIMIT_RAD);
    fprintf(f, "  </joint>\n\n");
    fprintf(f, "  <link name=\"RightFoot\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "foot_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"RightFoot_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_FOOT.r, COLOR_FOOT.g, COLOR_FOOT.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    fprintf(f, "</robot>\n");

    fclose(f);
    printf("wrote %s\n", filename);
}

static void write_quadruped_urdf(const char *filename) {
    FILE *f = fopen(filename, "w");
    if (!f) { perror(filename); return; }

    double base_to_body_z = QUAD_LEG_LEN + QUAD_BODY_H / 2.0;
    double head_x = QUAD_BODY_L / 2.0 + QUAD_HEAD_SIZE * 0.45;
    double head_z = QUAD_BODY_H * 0.15;

    fprintf(f, "<?xml version=\"1.0\"?>\n");
    fprintf(f, "<!--\n");
    fprintf(f, "  gen_humanoid_assets.c가 생성. humanoid.urdf와 같은 좌표 변환\n");
    fprintf(f, "  (ros_xyz = (ours_z, ours_x, ours_y), Y-up -> ROS Z-up)을 적용했다.\n");
    fprintf(f, "  URDF가 담지 못하는 정보(순찰 경로, 트롯 걸음걸이 파라미터 등)는\n");
    fprintf(f, "  quadruped_extra.yaml을 같이 참고할 것. 루트 링크 이름이\n");
    fprintf(f, "  (humanoid.urdf처럼 \"base_link\"가 아니라) \"quad_base_link\"인 이유:\n");
    fprintf(f, "  humanoid와 4족 로봇을 같은 RViz 장면에 동시에 띄울 때 두 URDF의\n");
    fprintf(f, "  루트 프레임 이름이 겹치면 TF가 충돌한다 - 다른 링크 이름\n");
    fprintf(f, "  (Body/Head/... vs QuadBody/QuadHead/...)은 원래부터 겹치지 않으니\n");
    fprintf(f, "  루트만 구분해주면 충분하다.\n");
    fprintf(f, "-->\n");
    fprintf(f, "<robot name=\"quadruped\">\n\n");

    fprintf(f, "  <link name=\"quad_base_link\"/>\n\n");

    fprintf(f, "  <joint name=\"base_to_quadbody\" type=\"fixed\">\n");
    fprintf(f, "    <parent link=\"quad_base_link\"/>\n");
    fprintf(f, "    <child link=\"QuadBody\"/>\n");
    fprintf(f, "    <origin xyz=\"0 0 %.4f\" rpy=\"0 0 0\"/>\n", base_to_body_z);
    fprintf(f, "  </joint>\n\n");

    fprintf(f, "  <link name=\"QuadBody\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "quad_body_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"QuadBody_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_QUAD_BODY.r, COLOR_QUAD_BODY.g, COLOR_QUAD_BODY.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    fprintf(f, "  <joint name=\"quadbody_to_quadhead\" type=\"fixed\">\n");
    fprintf(f, "    <parent link=\"QuadBody\"/>\n");
    fprintf(f, "    <child link=\"QuadHead\"/>\n");
    fprintf(f, "    <origin xyz=\"%.4f 0 %.4f\" rpy=\"0 0 0\"/>\n", head_x, head_z);
    fprintf(f, "  </joint>\n\n");

    fprintf(f, "  <link name=\"QuadHead\">\n");
    fprintf(f, "    <visual>\n");
    fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "quad_head_ros.obj\"/></geometry>\n");
    fprintf(f, "      <material name=\"QuadHead_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
            COLOR_QUAD_HEAD.r, COLOR_QUAD_HEAD.g, COLOR_QUAD_HEAD.b);
    fprintf(f, "    </visual>\n");
    fprintf(f, "  </link>\n\n");

    {
        static const char *leg_names[4] = {"QuadLegFL", "QuadLegFR", "QuadLegBL", "QuadLegBR"};
        static const char *shin_names[4] = {"QuadShinFL", "QuadShinFR", "QuadShinBL", "QuadShinBR"};
        double leg_x[4] = {QUAD_HIP_Z_FRONT, QUAD_HIP_Z_FRONT, QUAD_HIP_Z_BACK, QUAD_HIP_Z_BACK};
        double leg_y[4] = {-QUAD_HIP_X, QUAD_HIP_X, -QUAD_HIP_X, QUAD_HIP_X};
        for (int i = 0; i < 4; i++) {
            /* 허벅지(엉덩이 관절 - 기존과 동일, 좌우 대칭 범위) */
            fprintf(f, "  <joint name=\"quadbody_to_%s\" type=\"revolute\">\n", leg_names[i]);
            fprintf(f, "    <parent link=\"QuadBody\"/>\n");
            fprintf(f, "    <child link=\"%s\"/>\n", leg_names[i]);
            /* z(세번째 값)는 원래 리터럴 0이었는데, humanoid의
               body_to_leftleg/rightleg가 쓰는 leg_z=-BODY_H/2와
               달리 QuadBody의 "중심"(상자 메쉬 원점)에 다리를
               매달아서, base_to_quadbody_z(LEG_LEN+BODY_H/2)와
               어긋나 로봇 전체가 QUAD_BODY_H/2(0.2m)만큼 땅 위로
               떠 보이는 버그였다 - humanoid와 똑같이 -QUAD_BODY_H/2를
               써서 몸통 "바닥"에 다리가 매달리게 고쳤다. */
            fprintf(f, "    <origin xyz=\"%.4f %.4f %.4f\" rpy=\"0 0 0\"/>\n",
                    leg_x[i], leg_y[i], -QUAD_BODY_H / 2.0);
            fprintf(f, "    <axis xyz=\"0 1 0\"/>\n");
            fprintf(f, "    <limit lower=\"-%.4f\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
                    QUAD_LEG_JOINT_LIMIT_RAD, QUAD_LEG_JOINT_LIMIT_RAD);
            fprintf(f, "  </joint>\n\n");
            fprintf(f, "  <link name=\"%s\">\n", leg_names[i]);
            fprintf(f, "    <visual>\n");
            fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "quad_leg_ros.obj\"/></geometry>\n");
            fprintf(f, "      <material name=\"%s_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
                    leg_names[i], COLOR_QUAD_LEG.r, COLOR_QUAD_LEG.g, COLOR_QUAD_LEG.b);
            fprintf(f, "    </visual>\n");
            fprintf(f, "  </link>\n\n");

            /* 종아리(무릎 관절 - 한쪽으로만 0~90도 굽혀짐). 부모가
               QuadBody가 아니라 방금 쓴 허벅지 링크고, 원점은
               허벅지 메쉬가 매달린 끝(-QUAD_THIGH_LEN)이다. */
            fprintf(f, "  <joint name=\"%s_to_%s\" type=\"revolute\">\n", leg_names[i], shin_names[i]);
            fprintf(f, "    <parent link=\"%s\"/>\n", leg_names[i]);
            fprintf(f, "    <child link=\"%s\"/>\n", shin_names[i]);
            fprintf(f, "    <origin xyz=\"0 0 %.4f\" rpy=\"0 0 0\"/>\n", -QUAD_THIGH_LEN);
            fprintf(f, "    <axis xyz=\"0 1 0\"/>\n");
            fprintf(f, "    <limit lower=\"0\" upper=\"%.4f\" effort=\"10\" velocity=\"2\"/>\n",
                    QUAD_KNEE_JOINT_LIMIT_RAD);
            fprintf(f, "  </joint>\n\n");
            fprintf(f, "  <link name=\"%s\">\n", shin_names[i]);
            fprintf(f, "    <visual>\n");
            fprintf(f, "      <geometry><mesh filename=\"" ASSET_URI_PREFIX "quad_shin_ros.obj\"/></geometry>\n");
            fprintf(f, "      <material name=\"%s_material\"><color rgba=\"%.2f %.2f %.2f 1.0\"/></material>\n",
                    shin_names[i], COLOR_QUAD_LEG.r, COLOR_QUAD_LEG.g, COLOR_QUAD_LEG.b);
            fprintf(f, "    </visual>\n");
            fprintf(f, "  </link>\n\n");
        }
    }

    fprintf(f, "</robot>\n");

    fclose(f);
    printf("wrote %s\n", filename);
}

static void write_humanoid_extra_yaml(const char *filename) {
    FILE *f = fopen(filename, "w");
    if (!f) { perror(filename); return; }

    fprintf(f, "# humanoid_extra.yaml\n");
    fprintf(f, "# gen_humanoid_assets.c가 생성. humanoid.urdf로는 표현할 수 없거나\n");
    fprintf(f, "# URDF의 범위를 벗어나는 정보를 모아둔 파일 - 항상 humanoid.urdf와\n");
    fprintf(f, "# 같이 봐야 한다.\n\n");

    fprintf(f, "coordinate_convention:\n");
    fprintf(f, "  source: \"humanoid_test.c / humanoid_loader_test.c (Y-up: X=left, Y=up, Z=forward)\"\n");
    fprintf(f, "  urdf: \"ROS REP-103 (Z-up: X=forward, Y=left, Z=up)\"\n");
    fprintf(f, "  mapping: \"ros_xyz = (ours_z, ours_x, ours_y)\"\n");
    fprintf(f, "  note: >\n");
    fprintf(f, "    메쉬 파일도 이 변환을 똑같이 적용한 *_ros.obj 버전을 새로 만들어\n");
    fprintf(f, "    urdf가 참조한다. 원래 body.obj 등은 Y-up 그대로 OpenGL 데모용으로\n");
    fprintf(f, "    남겨둔다.\n\n");

    fprintf(f, "materials:\n");
    fprintf(f, "  # URDF <material><color rgba=.../></material>는 단색 1개만 표현할 수\n");
    fprintf(f, "  # 있어서, 실제 OpenGL 데모가 쓰는 절차적 텍스처(체커 생성 패턴,\n");
    fprintf(f, "  # GL_TEXTURE_GEN 매핑, 가로 줄무늬 상의 등)는 그대로 못 옮긴다.\n");
    fprintf(f, "  # urdf에는 대표색만 넣었고, 실제 텍스처 생성 파라미터는 여기 적는다.\n");
    fprintf(f, "  skin:\n");
    fprintf(f, "    links: [Head]\n");
    fprintf(f, "    texture_type: flat_color\n");
    fprintf(f, "    rgb: [0.871, 0.706, 0.588]   # 222,180,150 /255, create_skin_texture()\n");
    fprintf(f, "  shirt:\n");
    fprintf(f, "    links: [Body, LeftArm, RightArm]\n");
    fprintf(f, "    texture_type: horizontal_stripe\n");
    fprintf(f, "    stripe_colors_rgb: [[0.196, 0.314, 0.588], [0.843, 0.863, 0.922]]\n");
    fprintf(f, "    stripe_period_px: 8\n");
    fprintf(f, "    texture_size_px: 64\n");
    fprintf(f, "    source_function: create_shirt_texture()\n");
    fprintf(f, "  pants:\n");
    fprintf(f, "    links: [LeftLeg, RightLeg]\n");
    fprintf(f, "    texture_type: flat_color\n");
    fprintf(f, "    rgb: [0.235, 0.314, 0.471]   # 60,80,120 /255, create_pants_texture()\n\n");

    fprintf(f, "instance_outfit_tints:\n");
    fprintf(f, "  # 여러 캐릭터(주인공 + 클론)를 동시에 그릴 때 shirt/pants 텍스처에\n");
    fprintf(f, "  # GL_MODULATE로 곱해지는 캐릭터별 색(draw_all_humanoids()의\n");
    fprintf(f, "  # outfit_tints[]). URDF는 로봇 1개의 구조만 표현하므로 인스턴스별\n");
    fprintf(f, "  # 변형은 담을 수 없다 - 여기에만 있다.\n");
    fprintf(f, "  - name: main\n");
    fprintf(f, "    tint_rgb: [1.00, 1.00, 1.00]\n");
    fprintf(f, "  - name: clone_1\n");
    fprintf(f, "    tint_rgb: [1.00, 0.55, 0.55]\n");
    fprintf(f, "  - name: clone_2\n");
    fprintf(f, "    tint_rgb: [0.55, 0.80, 1.00]\n\n");

    fprintf(f, "decorations:\n");
    fprintf(f, "  # draw_head()에서 OBJ 메쉬가 아니라 매 프레임 따로 그리던 장식.\n");
    fprintf(f, "  # humanoid.urdf의 Head 링크에 손실 없이 옮겨뒀다 (참고용 메모).\n");
    fprintf(f, "  # 처음엔 <sphere> 프리미티브로 넣었는데, 같은 링크(Head)에 메쉬\n");
    fprintf(f, "  # visual과 프리미티브 visual이 같이 있으면 RViz2가 프리미티브의\n");
    fprintf(f, "  # <material> 색을 무시하는 문제가 있어서(Head 메쉬 색으로 덮임),\n");
    fprintf(f, "  # 다른 부위와 똑같이 작은 메쉬(nose_ros.obj + nose_ros.mtl)로\n");
    fprintf(f, "  # 바꿨다.\n");
    fprintf(f, "  nose:\n");
    fprintf(f, "    represented_in_urdf: true\n");
    fprintf(f, "    link: Head\n");
    fprintf(f, "    shape: mesh (nose_ros.obj, small sphere)\n\n");

    fprintf(f, "animation:\n");
    fprintf(f, "  # URDF는 정적인 뼈대/외형만 담고, 시간에 따른 움직임(걷기 애니메이션)\n");
    fprintf(f, "  # 자체는 담지 않는다 - idle()의 파라미터를 여기 적어둔다.\n");
    fprintf(f, "  idle_tick_delta_t: 0.004   # t += 0.004 per idle() tick (프레임 속도 무관, t 기준 계산)\n");
    fprintf(f, "  limb_swing:\n");
    fprintf(f, "    left_arm_deg: \"35 * sin(t)\"\n");
    fprintf(f, "    right_arm_deg: \"-35 * sin(t)\"\n");
    fprintf(f, "    left_leg_deg: \"-25 * sin(t)\"\n");
    fprintf(f, "    right_leg_deg: \"25 * sin(t)\"\n");
    fprintf(f, "    joint_axis_note: \"위 각도는 ours의 ROTX(=urdf의 axis 0 1 0) 회전각이다\"\n");
    fprintf(f, "    gated_by: \"limb_animate (Q/A/E/D/Z/X/C/V로 수동 포즈 조절 시 꺼짐)\"\n");
    fprintf(f, "  walk_path:\n");
    fprintf(f, "    speed_s_per_t: 0.6   # path_sample(0.6*t, ...)\n");
    fprintf(f, "    waypoint_file: humanoid_path.txt\n");
    fprintf(f, "    interpolation: Catmull-Rom spline (arc-length parameterized)\n");
    fprintf(f, "  main_character_modes: [PATH, MANUAL, STOPPED]\n");
    fprintf(f, "  clone_offsets:\n");
    fprintf(f, "    count: 2\n");
    fprintf(f, "    spacing: path_total_len / (clone_count + 1), arc-length 기준 균등 배치\n");
    fprintf(f, "  movement_clamp:\n");
    fprintf(f, "    play_half: GROUND_HALF - 1.0\n");
    fprintf(f, "    ground_half: 30.0\n\n");

    fprintf(f, "joint_limits_note: >\n");
    fprintf(f, "  humanoid.urdf의 revolute 관절 limit(effort/velocity 포함)은 원래\n");
    fprintf(f, "  애니메이션에 실제 동역학 한계가 없어서(순수 기구학적 sin() 애니메이션)\n");
    fprintf(f, "  의미 있는 값이 아니라 임의로 넣은 자리값(placeholder)이다. 각도\n");
    fprintf(f, "  범위(lower/upper)는 위 limb_swing 진폭(팔 35도, 다리 25도)에 여유를\n");
    fprintf(f, "  더해(팔 45도, 다리 35도) 정했다.\n\n");

    fprintf(f, "scene_not_included:\n");
    fprintf(f, "  # 이 로봇의 URDF에는 당연히 포함되지 않는, 장면(scene)/배경 요소.\n");
    fprintf(f, "  # 참고용 - 나중에 world/Gazebo 파일로 옮길 때 참고.\n");
    fprintf(f, "  - 하늘 그라디언트 배경 (draw_sky)\n");
    fprintf(f, "  - 땅 바닥 (draw_ground, GROUND_HALF=30)\n");
    fprintf(f, "  - 그림자 (평행광 전단 투영, LIGHT_X/Y/Z)\n");
    fprintf(f, "  - 나무 5그루 (draw_trees, 크리스마스 트리 모양)\n");
    fprintf(f, "  - 4족 로봇 (별도 quadruped.urdf + quadruped_extra.yaml)\n\n");

    fprintf(f, "mesh_filename_note: >\n");
    fprintf(f, "  humanoid.urdf의 <mesh filename=\"...\">들은 이 머신에서의 절대경로를\n");
    fprintf(f, "  file:// URI로 박아 넣었다 (ASSET_URI_PREFIX,\n");
    fprintf(f, "  file:///D:/claude_test/computer_graphics/). 이 프로젝트가 아직\n");
    fprintf(f, "  ROS 패키지가 아니라 package://를 쓸 수 없어서 고른 방법이다.\n");
    fprintf(f, "  프로젝트 폴더를 다른 경로로 옮기면 gen_humanoid_assets.c의\n");
    fprintf(f, "  ASSET_URI_PREFIX를 고치고 다시 생성해야 한다.\n");

    fclose(f);
    printf("wrote %s\n", filename);
}

static void write_quadruped_extra_yaml(const char *filename) {
    FILE *f = fopen(filename, "w");
    if (!f) { perror(filename); return; }

    fprintf(f, "# quadruped_extra.yaml\n");
    fprintf(f, "# gen_humanoid_assets.c가 생성. quadruped.urdf로는 표현할 수 없거나\n");
    fprintf(f, "# URDF의 범위를 벗어나는 정보를 모아둔 파일 - 항상 quadruped.urdf와\n");
    fprintf(f, "# 같이 봐야 한다.\n\n");

    fprintf(f, "coordinate_convention:\n");
    fprintf(f, "  source: \"humanoid_test.c / humanoid_loader_test.c (Y-up: X=left, Y=up, Z=forward)\"\n");
    fprintf(f, "  urdf: \"ROS REP-103 (Z-up: X=forward, Y=left, Z=up)\"\n");
    fprintf(f, "  mapping: \"ros_xyz = (ours_z, ours_x, ours_y)\"\n\n");

    fprintf(f, "root_link_name: quad_base_link\n");
    fprintf(f, "root_link_name_note: >\n");
    fprintf(f, "  humanoid.urdf의 루트는 \"base_link\"인데 quadruped.urdf의 루트는\n");
    fprintf(f, "  \"quad_base_link\"다 - 두 로봇을 같은 RViz 장면/TF 트리에 동시에\n");
    fprintf(f, "  띄울 때 루트 프레임 이름이 겹치면 TF가 충돌하기 때문에 다르게 뒀다.\n\n");

    fprintf(f, "materials:\n");
    fprintf(f, "  # 4족 로봇은 땅/나무와 마찬가지로 체커 텍스처와 분리된 단색만 쓴다\n");
    fprintf(f, "  # (draw_quad_body/head/leg, 또는 로더 버전의 texture_for_part()가\n");
    fprintf(f, "  # Quad* 부위에 대해 0을 반환해 텍스처를 안 입히는 것과 동일).\n");
    fprintf(f, "  # urdf의 material color와 이미 동일하므로 손실은 없다 - 참고용.\n");
    fprintf(f, "  body_rgb: [%.2f, %.2f, %.2f]\n", COLOR_QUAD_BODY.r, COLOR_QUAD_BODY.g, COLOR_QUAD_BODY.b);
    fprintf(f, "  head_rgb: [%.2f, %.2f, %.2f]\n", COLOR_QUAD_HEAD.r, COLOR_QUAD_HEAD.g, COLOR_QUAD_HEAD.b);
    fprintf(f, "  leg_rgb: [%.2f, %.2f, %.2f]\n\n", COLOR_QUAD_LEG.r, COLOR_QUAD_LEG.g, COLOR_QUAD_LEG.b);

    fprintf(f, "animation:\n");
    fprintf(f, "  idle_tick_delta_t: 0.004   # t += 0.004 per idle() tick\n");
    fprintf(f, "  gait: trot\n");
    fprintf(f, "  trot_swing_deg: \"30 * sin(2*t)\"\n");
    fprintf(f, "  trot_leg_phase:\n");
    fprintf(f, "    QuadLegFL: \"+swing\"\n");
    fprintf(f, "    QuadLegBR: \"+swing\"\n");
    fprintf(f, "    QuadLegFR: \"-swing\"\n");
    fprintf(f, "    QuadLegBL: \"-swing\"\n");
    fprintf(f, "  # 무릎(QuadShinFL 등) - 엉덩이가 스윙(발을 앞으로 가져가는\n");
    fprintf(f, "  # 구간)일 때 굽히고 스탠스일 때 펴진다. 진폭 15도는 ROS2/MuJoCo\n");
    fprintf(f, "  # 물리 보행 데모(ros2_ws/.../quad_mujoco_walk.py)에서 ROS 없이\n");
    fprintf(f, "  # mj_step만 반복하는 독립 스크립트로 쓸어서 찾은 값이다(16도부터\n");
    fprintf(f, "  # 급격히 불안정, 17도부터 넘어짐 - 15도가 한계 바로 아래 안전한 값).\n");
    fprintf(f, "  knee_bend_deg: \"15 * max(0, -cos(hip_phase))\"\n");
    fprintf(f, "  knee_phase:\n");
    fprintf(f, "    QuadShinFL: \"same as QuadLegFL hip phase\"\n");
    fprintf(f, "    QuadShinBR: \"same as QuadLegBR hip phase\"\n");
    fprintf(f, "    QuadShinFR: \"same as QuadLegFR hip phase (opposite of FL/BR)\"\n");
    fprintf(f, "    QuadShinBL: \"same as QuadLegBL hip phase (opposite of FL/BR)\"\n");
    fprintf(f, "  joint_axis_note: \"위 각도는 ours의 ROTX(=urdf의 axis 0 1 0) 회전각이다\"\n");
    fprintf(f, "  patrol:\n");
    fprintf(f, "    axis: world X (ros) / ours X (고정된 z=quad_patrol_z 위에서 x만 왕복)\n");
    fprintf(f, "    config_file: quadruped_patrol.txt   # \"z x_min x_max speed\" 한 줄\n");
    fprintf(f, "    position_formula: \"x = center + half_range * sin(speed*t)\"\n");
    fprintf(f, "    heading_formula: \"heading = 90 * tanh(QUAD_TURN_SHARPNESS * cos(speed*t))\"\n");
    fprintf(f, "    turn_sharpness: 6.0   # QUAD_TURN_SHARPNESS, 클수록 방향 전환 구간이 좁고 급함\n");
    fprintf(f, "    note: >\n");
    fprintf(f, "      heading이 tanh로 완만하게 90 -> 0 -> -90도를 오가며 왕복 양 끝에서\n");
    fprintf(f, "      순간이동하듯 꺾이지 않고 제자리에서 도는 것처럼 보이게 한다.\n\n");

    fprintf(f, "joint_limits_note: >\n");
    fprintf(f, "  quadruped.urdf의 revolute 관절 limit은 실제 동역학 한계가 아니라\n");
    fprintf(f, "  trot_swing_deg 진폭(30도)에 여유를 더한(45도) placeholder 값이다.\n\n");

    fprintf(f, "scene_not_included:\n");
    fprintf(f, "  - 땅 바닥, 하늘, 그림자, 나무 (humanoid.urdf와 같은 장면을 공유함)\n");
    fprintf(f, "  - humanoid 캐릭터 (별도 humanoid.urdf + humanoid_extra.yaml)\n\n");

    fprintf(f, "mesh_filename_note: >\n");
    fprintf(f, "  humanoid_extra.yaml의 mesh_filename_note와 동일 - 이 머신 절대경로의\n");
    fprintf(f, "  file:// URI를 그대로 박아 넣었다 (ASSET_URI_PREFIX).\n");

    fclose(f);
    printf("wrote %s\n", filename);
}

int main(void) {
    /* 몸통/팔/다리를 원기둥으로(2026-11-17, 사용자 요청: "OpenGL
       쪽도 URDF/MJCF처럼 일관성 있게" - 관절 구조는 안 바꾸고
       모양만). 몸통은 URDF/MJCF와 똑같이 원형(TORSO_RADIUS=0.32,
       둘 다 그 값으로 통일했던 것과 맞춤) - box 시절처럼 원점
       중심(-BODY_H/2..BODY_H/2). 팔/다리는 URDF의 타원 비율
       (write_elliptic_cylinder_obj_zup 호출 값)과 맞췄다 - 거기서
       rx=ROS_X(=ours_Z, 앞뒤), ry=ROS_Y(=ours_X, 좌우)였으니 여기선
       반대로 넣는다. */
    write_elliptic_cylinder_obj("body.obj", TORSO_RADIUS, TORSO_RADIUS,
                                 -BODY_H / 2.0, BODY_H / 2.0, 16);
    write_sphere_obj("head.obj", HEAD_R, HEAD_R, 24, 16);
    /* 팔: 어깨(원점)에서 아래로 매달린 형태 (y: -ARM_LEN..0) */
    write_elliptic_cylinder_obj("arm.obj", 0.06, 0.10, -ARM_LEN, 0.0, 12);
    /* 허벅지(엉덩이 관절에서 아래로 매달림, 길이 THIGH_LEN)와
       종아리(무릎 관절에서 아래로 매달림, 길이 SHIN_LEN) - 둘을
       합치면 예전 한 덩어리 다리(LEG_LEN)와 총 길이가 같다(4족과
       동일한 패턴). */
    write_elliptic_cylinder_obj("leg.obj", 0.075, 0.11, -THIGH_LEN, 0.0, 12);
    write_elliptic_cylinder_obj("shin.obj", 0.06, 0.085, -SHIN_LEN, 0.0, 12);

    /* 4족 로봇: 몸통은 중심이 원점인 상자(humanoid_scene.txt의
       QuadBody 위치가 이미 들어올려주므로), 머리도 마찬가지, 다리는
       엉덩이(원점)에서 아래로 매달린 형태 (humanoid 다리와 동일). */
    write_box_obj("quad_body.obj", -QUAD_BODY_W / 2.0, QUAD_BODY_W / 2.0,
                   -QUAD_BODY_H / 2.0, QUAD_BODY_H / 2.0, -QUAD_BODY_L / 2.0, QUAD_BODY_L / 2.0);
    write_box_obj("quad_head.obj", -QUAD_HEAD_SIZE / 2.0, QUAD_HEAD_SIZE / 2.0,
                   -QUAD_HEAD_SIZE / 2.0, QUAD_HEAD_SIZE / 2.0, -QUAD_HEAD_SIZE / 2.0, QUAD_HEAD_SIZE / 2.0);
    /* 허벅지(엉덩이 관절에서 아래로 매달림, 길이 QUAD_THIGH_LEN)와
       종아리(무릎 관절에서 아래로 매달림, 길이 QUAD_SHIN_LEN) -
       둘을 합치면 예전 한 덩어리 다리(QUAD_LEG_LEN)와 총 길이가
       같다. */
    write_box_obj("quad_leg.obj", -QUAD_LEG_W / 2.0, QUAD_LEG_W / 2.0,
                   -QUAD_THIGH_LEN, 0.0, -QUAD_LEG_D / 2.0, QUAD_LEG_D / 2.0);
    write_box_obj("quad_shin.obj", -QUAD_LEG_W / 2.0, QUAD_LEG_W / 2.0,
                   -QUAD_SHIN_LEN, 0.0, -QUAD_LEG_D / 2.0, QUAD_LEG_D / 2.0);

    write_scene("humanoid_scene.txt");

    /* ---- URDF/ROS용 Z-up 메쉬(*_ros.obj) ----
       write_box_obj()는 min/max 인자 순서만 (z,x,y)로 바꿔 넣으면
       그대로 축이 재배치된 메쉬가 나온다 - 상자는 축끼리 쌍을 이루는
       min/max 구간일 뿐이라 재배치로 충분하다. 구(머리)는 극축이
       코드에 박혀 있어서 write_sphere_obj_zup()을 따로 썼다. */
    write_box_obj("body_ros.obj", -BODY_D / 2.0, BODY_D / 2.0,
                   -BODY_W / 2.0, BODY_W / 2.0, -BODY_H / 2.0, BODY_H / 2.0);
    write_sphere_obj_zup("head_ros.obj", HEAD_R, HEAD_R, 24, 16);
    /* 코: humanoid.urdf에서 <sphere> 프리미티브로 뒀더니 같은 링크의
       메쉬 visual에 색이 덮여버려서(아래 write_humanoid_urdf() 주석
       참고), 다른 부위와 똑같이 작은 메쉬로 만든다. center_z=0 -
       위치는 URDF의 <origin>이 맡고, 이 메쉬는 반지름만 책임진다. */
    write_sphere_obj_zup("nose_ros.obj", HEAD_R * 0.28, 0.0, 12, 12);
    /* 팔/다리/가슴/골반 URDF 메쉬를 타원기둥으로(사용자 요청 2번:
       처음엔 "mjcf는 원기둥인데 urdf는 직육면체라 통일해줘"로 원형
       원기둥 썼다가, "가슴과 원통도 타원형이면 이쁠 텐데"로 전부
       타원 단면으로 바꿨다). 팔다리는 MJCF capsule 반지름 근처에서
       약간 납작하게(rx>ry), 가슴/골반은 기존 box의 depth/width
       (BODY_D/2, BODY_W/2)를 그대로 타원 장단축으로 썼다(앞뒤로
       얇고 좌우로 넓은 몸통 단면 - 기존 박스와 같은 비율).
       OpenGL용 arm.obj/leg.obj/shin.obj는 그대로 box 유지(그쪽은
       물리와 무관한 별개 데모라 안 건드림). */
    /* 팔꿈치 추가로 URDF용 arm_ros.obj는 위팔 길이(UPPER_ARM_LEN)로
       줄이고, forearm_ros.obj(아래팔, 살짝 더 가늘게)를 새로
       추가한다 - OpenGL용 arm.obj(관절 없음)는 그대로 ARM_LEN 전체. */
    write_elliptic_cylinder_obj_zup("arm_ros.obj", 0.10, 0.06, -UPPER_ARM_LEN, 0.0, 12);
    write_elliptic_cylinder_obj_zup("forearm_ros.obj", 0.09, 0.05, -FOREARM_LEN, 0.0, 12);
    /* 손(2026-11-18 추가) - 아래팔보다 살짝 더 가늚, humanoid_walk.xml의
       hand_L/R 캡슐 크기(size 0.05)와 맞춤. */
    write_elliptic_cylinder_obj_zup("hand_ros.obj", 0.06, 0.035, -HAND_LEN, 0.0, 12);
    write_elliptic_cylinder_obj_zup("leg_ros.obj", 0.11, 0.075, -THIGH_LEN, 0.0, 12);
    /* 종아리 메쉬를 HUMANOID_ANKLE_HEIGHT만큼 짧게 - 발목 피벗이
       땅(z=0)이 아니라 그 위(ANKLE_HEIGHT)에 있어서, 종아리가 거기
       까지만 내려오고 그 아래는 발(foot_ros.obj) 몫이다(안 그러면
       발이 종아리 속에 파묻혀 안 보임 - write_humanoid_urdf() 주석
       참고). OpenGL용 shin.obj(발목 없음)는 그대로 SHIN_LEN 전체. */
    write_elliptic_cylinder_obj_zup("shin_ros.obj", 0.085, 0.06, -(SHIN_LEN - HUMANOID_ANKLE_HEIGHT), 0.0, 12);
    /* 골반/가슴 - body_ros.obj(몸통 전체)를 반으로 쪼갠 것(좌우/전후
       밸런스용 허리 관절 추가, write_humanoid_urdf() 주석 참고).
       OpenGL 데모는 안 건드리므로 Y-up(ours) 버전은 필요 없음 -
       URDF 전용 _ros만 생성. */
    write_elliptic_cylinder_obj_zup("pelvis_ros.obj", TORSO_RADIUS, TORSO_RADIUS, -BODY_H / 2.0, 0.0, 16);
    write_elliptic_cylinder_obj_zup("chest_ros.obj", TORSO_RADIUS, TORSO_RADIUS, 0.0, BODY_H / 2.0, 16);
    /* 발 - 발목 관절 위치(종아리 끝)보다 살짝 앞+위로 오프셋된
       평평한 박스(지지 다각형 확보용, humanoid_stand_lqr.py에서
       검증한 치수와 동일). 역시 URDF 전용. */
    /* 발목 피벗(이 메쉬의 원점)은 이제 땅이 아니라 HUMANOID_ANKLE_HEIGHT
       만큼 위에 있다(write_humanoid_urdf() 주석 참고) - 그래서 발은
       피벗에서 "위로"가 아니라 "아래로"(z: -ANKLE_HEIGHT~0) 내려가야
       발바닥이 땅에 닿는다. */
    write_box_obj("foot_ros.obj", -0.11, 0.23, -0.10, 0.10, -HUMANOID_ANKLE_HEIGHT, 0.0);
    write_box_obj("quad_body_ros.obj", -QUAD_BODY_L / 2.0, QUAD_BODY_L / 2.0,
                   -QUAD_BODY_W / 2.0, QUAD_BODY_W / 2.0, -QUAD_BODY_H / 2.0, QUAD_BODY_H / 2.0);
    write_box_obj("quad_head_ros.obj", -QUAD_HEAD_SIZE / 2.0, QUAD_HEAD_SIZE / 2.0,
                   -QUAD_HEAD_SIZE / 2.0, QUAD_HEAD_SIZE / 2.0, -QUAD_HEAD_SIZE / 2.0, QUAD_HEAD_SIZE / 2.0);
    write_box_obj("quad_leg_ros.obj", -QUAD_LEG_D / 2.0, QUAD_LEG_D / 2.0,
                   -QUAD_LEG_W / 2.0, QUAD_LEG_W / 2.0, -QUAD_THIGH_LEN, 0.0);
    write_box_obj("quad_shin_ros.obj", -QUAD_LEG_D / 2.0, QUAD_LEG_D / 2.0,
                   -QUAD_LEG_W / 2.0, QUAD_LEG_W / 2.0, -QUAD_SHIN_LEN, 0.0);

    /* *_ros.obj 각각에 URDF의 <material> 색과 똑같은 색을 담은 .mtl을
       박아 넣는다 - RViz2가 URDF 재질을 무시하고 민무늬로 그리는
       경우가 있어서, 메쉬 자체에도 색을 들려보낸다. */
    bake_material_into_obj("body_ros.obj", "body_ros.mtl", COLOR_BODY.r, COLOR_BODY.g, COLOR_BODY.b);
    bake_material_into_obj("head_ros.obj", "head_ros.mtl", COLOR_HEAD.r, COLOR_HEAD.g, COLOR_HEAD.b);
    bake_material_into_obj("nose_ros.obj", "nose_ros.mtl", COLOR_NOSE.r, COLOR_NOSE.g, COLOR_NOSE.b);
    bake_material_into_obj("arm_ros.obj", "arm_ros.mtl", COLOR_ARM.r, COLOR_ARM.g, COLOR_ARM.b);
    bake_material_into_obj("forearm_ros.obj", "forearm_ros.mtl", COLOR_ARM.r, COLOR_ARM.g, COLOR_ARM.b);
    bake_material_into_obj("hand_ros.obj", "hand_ros.mtl", COLOR_HEAD.r, COLOR_HEAD.g, COLOR_HEAD.b);
    bake_material_into_obj("leg_ros.obj", "leg_ros.mtl", COLOR_LEG.r, COLOR_LEG.g, COLOR_LEG.b);
    bake_material_into_obj("shin_ros.obj", "shin_ros.mtl", COLOR_LEG.r, COLOR_LEG.g, COLOR_LEG.b);
    bake_material_into_obj("pelvis_ros.obj", "pelvis_ros.mtl", COLOR_BODY.r, COLOR_BODY.g, COLOR_BODY.b);
    bake_material_into_obj("chest_ros.obj", "chest_ros.mtl", COLOR_BODY.r, COLOR_BODY.g, COLOR_BODY.b);
    bake_material_into_obj("foot_ros.obj", "foot_ros.mtl", COLOR_FOOT.r, COLOR_FOOT.g, COLOR_FOOT.b);
    bake_material_into_obj("quad_body_ros.obj", "quad_body_ros.mtl",
                            COLOR_QUAD_BODY.r, COLOR_QUAD_BODY.g, COLOR_QUAD_BODY.b);
    bake_material_into_obj("quad_head_ros.obj", "quad_head_ros.mtl",
                            COLOR_QUAD_HEAD.r, COLOR_QUAD_HEAD.g, COLOR_QUAD_HEAD.b);
    bake_material_into_obj("quad_leg_ros.obj", "quad_leg_ros.mtl",
                            COLOR_QUAD_LEG.r, COLOR_QUAD_LEG.g, COLOR_QUAD_LEG.b);
    bake_material_into_obj("quad_shin_ros.obj", "quad_shin_ros.mtl",
                            COLOR_QUAD_LEG.r, COLOR_QUAD_LEG.g, COLOR_QUAD_LEG.b);

    write_humanoid_urdf("humanoid.urdf");
    write_humanoid_extra_yaml("humanoid_extra.yaml");
    write_quadruped_urdf("quadruped.urdf");
    write_quadruped_extra_yaml("quadruped_extra.yaml");

    return 0;
}
