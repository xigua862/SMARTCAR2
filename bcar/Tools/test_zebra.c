/**
  ******************************************************************************
  * @file    test_zebra.c
  * @brief   斑马线（S/G）检测的 PC 端单元测试
  *
  * 用法（在 bcar 目录下，或用 Tools\run_pc_tests.py）：
  *     gcc -I src/middleware -o build_test_zebra.exe Tools/test_zebra.c ^
  *         src/middleware/zebra.c src/middleware/line_pos.c -lm
  *
  * ★用例里的 p 值全是**2026-10-15 在真赛道上实测的**（不是编的、也不是按图估的）：
  *   白线正中  p = 0  0  0.66 0.94 0.96 0.14 0.61 0    sum_p=3.33
  *   斑马线上  p = 0.99 0.40 0.69 0.94 0.95 0.76 0.66 0.95 sum_p=6.40
  *   ⚠ 教训：按赛道图估算的"白区跨度 40mm"阈值，在真白线上（弱白跨度 44mm）会误报。
  *     所以判据只认**强白**（p≥0.85）的跨度 + 总白量，两条都要过。
  ******************************************************************************
  */
#include <stdio.h>
#include "zebra.h"
#include "line_pos.h"

static int g_pass = 0;
static int g_fail = 0;

static void check(int cond, const char *what)
{
    if (cond) { g_pass++; }
    else { g_fail++; printf("  FAIL: %s\n", what); }
}

/* 造一个 line_pos_t：p 数组 + 自动求和（zebra 用 sum_p，不能忘）。
 * 返回静态缓冲的指针 —— 单测单线程，够用（不要在同一表达式里调两次）。*/
static const line_pos_t *mk(const float *p)
{
    static line_pos_t lp;
    uint8_t i;
    lp.sum_p = 0.0f;
    for (i = 0u; i < LINE_CH; i++) {
        lp.p[i] = p[i];
        lp.sum_p += p[i];
    }
    lp.err_mm  = 0.0f;
    lp.visible = (lp.sum_p >= 0.4f) ? 1u : 0u;
    return &lp;
}

int main(void)
{
    line_pos_cfg_t cfg;
    uint8_t i, k;

    /* ---- 实测用例（放在 main 外面，方便对照）---- */
    /* 白线正中：ch2~ch6 有弱白，强白只有 ch3/ch4 */
    const float LINE_P[8]   = {0.00f, 0.00f, 0.66f, 0.94f, 0.96f, 0.14f, 0.61f, 0.00f};
    /* 白线压偏一点（最坏情况：白线正好盖住 4 路强白）*/
    const float LINE2_P[8]  = {0.00f, 0.90f, 1.00f, 1.00f, 0.90f, 0.00f, 0.00f, 0.00f};
    /* 斑马线上（实测） */
    const float ZEBRA_P[8]  = {0.99f, 0.40f, 0.69f, 0.94f, 0.95f, 0.76f, 0.66f, 0.95f};
    /* 全黑（丢线） */
    const float DARK_P[8]   = {0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f};

    printf("=== 斑马线检测单测（判据用实测数据）===\n");

    /* 位置表 = 实测：±5.5/±16.5/±27.5/±38.5mm */
    for (i = 0u; i < LINE_CH; i++) {
        cfg.pos_mm[i] = ((float)i - 3.5f) * 11.0f;
        cfg.black[i]  = 1000.0f;
        cfg.white[i]  = 30.0f;
    }
    cfg.lost_sum_p = 0.4f;
    line_pos_init(&cfg);

    /* ---- 1. ★真白线（实测值）：绝不能报 ---- */
    {
        zebra_t z;
        uint8_t ev = 0u;
        zebra_init(&z);
        for (k = 0u; k < 40u; k++) { ev |= zebra_step(&z, mk(LINE_P)); }
        check(ev == 0u, "★真白线 40 拍：一次都不许报（实测 sum_p=3.33 / 强白跨度 11mm）");
    }

    /* ---- 2. 白线压偏（4 路强白，最坏情况）：也不许报 ---- */
    {
        zebra_t z;
        uint8_t ev = 0u;
        zebra_init(&z);
        for (k = 0u; k < 40u; k++) { ev |= zebra_step(&z, mk(LINE2_P)); }
        check(ev == 0u, "白线压偏（4 路强白，跨度 33mm）：不报");
    }

    /* ---- 3. ★真斑马线（实测值）：3 拍内报一次 ---- */
    {
        zebra_t z;
        uint8_t e1, e2;
        zebra_init(&z);
        e1 = zebra_step(&z, mk(ZEBRA_P));
        e2 = zebra_step(&z, mk(ZEBRA_P));
        check(e1 == 0u && e2 == 0u, "斑马线前 2 拍：去抖，先不报");
        check(zebra_step(&z, mk(ZEBRA_P)) == 1u, "★斑马线第 3 拍：报事件");
    }

    /* ---- 4. 一直压着只报一次；开过去再压才报第二次 ---- */
    {
        zebra_t z;
        uint8_t ev = 0u, ev2 = 0u;
        zebra_init(&z);
        for (k = 0u; k < 60u; k++) { ev += zebra_step(&z, mk(ZEBRA_P)); }
        check(ev == 1u, "★一直压着斑马线 60 拍：只报 1 次");
        check(zebra_inside(&z) == 1u, "压着时 inside=1");
        for (k = 0u; k < 30u; k++) { (void)zebra_step(&z, mk(DARK_P)); }
        check(zebra_inside(&z) == 0u, "离开 30 拍：inside 归 0");
        for (k = 0u; k < 10u; k++) { ev2 += zebra_step(&z, mk(ZEBRA_P)); }
        check(ev2 == 1u, "★第二条斑马线：能再报一次（先 S 后 G）");
    }

    /* ---- 5. 刚离开又碰一下（毛刺）：算同一条，不重复报 ---- */
    {
        zebra_t z;
        uint8_t ev = 0u;
        zebra_init(&z);
        for (k = 0u; k < 10u; k++) { (void)zebra_step(&z, mk(ZEBRA_P)); }
        for (k = 0u; k < 5u; k++)  { (void)zebra_step(&z, mk(DARK_P)); }
        for (k = 0u; k < 10u; k++) { ev += zebra_step(&z, mk(ZEBRA_P)); }
        check(ev == 0u, "离开不足 ZEBRA_OFF_TICKS 又压上：不重复报");
    }

    /* ---- 6. 全黑：不报 ---- */
    {
        zebra_t z;
        uint8_t ev = 0u;
        zebra_init(&z);
        for (k = 0u; k < 40u; k++) { ev |= zebra_step(&z, mk(DARK_P)); }
        check(ev == 0u, "全黑：不报");
    }

    /* ---- 7. 空指针别崩 ---- */
    {
        zebra_t z;
        zebra_init(&z);
        check(zebra_step(0, 0) == 0u, "空指针：返回 0");
        check(zebra_inside(0) == 0u, "空指针：inside 返回 0");
    }

    printf("  通过 %d / 失败 %d\n", g_pass, g_fail);
    return g_fail;
}
