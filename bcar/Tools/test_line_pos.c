/**
  ******************************************************************************
  * @file    test_line_pos.c
  * @brief   8 路灰度 → 线偏差 的 PC 端单元测试
  *
  * 用法（在 bcar 目录下）：
  *     gcc -I src/middleware -o build_test_line_pos.exe Tools/test_line_pos.c src/middleware/line_pos.c -lm
  *     ./build_test_line_pos.exe
  ******************************************************************************
  */
#include <stdio.h>
#include <math.h>
#include "line_pos.h"

static int g_pass = 0;
static int g_fail = 0;

static void check(int cond, const char *what)
{
    if (cond) {
        g_pass++;
    } else {
        g_fail++;
        printf("  FAIL: %s\n", what);
    }
}

static void check_near(float got, float want, float tol, const char *what)
{
    if (fabsf(got - want) <= tol) {
        g_pass++;
    } else {
        g_fail++;
        printf("  FAIL: %s（得到 %.4f，期望 %.4f ±%.4f）\n", what, got, want, tol);
    }
}

/* 造一套"标准"配置：位置 -21..+21（6mm 间距），每路黑 200 / 白 3000 */
static line_pos_cfg_t std_cfg(void)
{
    line_pos_cfg_t c;
    uint8_t i;
    for (i = 0; i < LINE_CH; i++) {
        c.pos_mm[i] = ((float)i - 3.5f) * 6.0f;
        c.black[i]  = 200.0f;
        c.white[i]  = 3000.0f;
    }
    c.lost_sum_p = 0.4f;
    return c;
}

/* 全黑（哪路都看不见线）*/
static void fill_black(uint16_t *raw)
{
    uint8_t i;
    for (i = 0; i < LINE_CH; i++) {
        raw[i] = 200;   /* = black ⇒ p=0 */
    }
}

/* 让第 idx 路完全看见白线 */
static void light_one(uint16_t *raw, int idx)
{
    fill_black(raw);
    raw[idx] = 3000;   /* = white ⇒ p=1 */
}

int main(void)
{
    line_pos_cfg_t cfg = std_cfg();
    line_pos_t     r;
    uint16_t       raw[LINE_CH];

    printf("=== line_pos 单测 (位置 -21..+21mm, 黑200/白3000) ===\n");

    /* ---- 1. 归一化：黑→0、白→1、中间→0.5 ---- */
    {
        line_pos_init(&cfg);
        fill_black(raw);
        raw[0] = 200;    /* 黑 */
        raw[1] = 3000;   /* 白 */
        raw[2] = 1600;   /* 正中间 → 0.5 */
        r = line_pos_step(raw);
        check_near(r.p[0], 0.0f, 1e-4f, "全黑 → p=0");
        check_near(r.p[1], 1.0f, 1e-4f, "全白 → p=1");
        check_near(r.p[2], 0.5f, 0.01f, "中间值 → p≈0.5");
    }

    /* ---- 2. 只有一路看见线 → err = 那一路的位置 ---- */
    {
        line_pos_init(&cfg);
        light_one(raw, 0);
        r = line_pos_step(raw);
        check(r.visible == 1u, "单路可见 → visible");
        check_near(r.err_mm, -21.0f, 0.01f, "最左一路 → err=-21mm");

        light_one(raw, 7);
        r = line_pos_step(raw);
        check_near(r.err_mm, 21.0f, 0.01f, "最右一路 → err=+21mm");

        light_one(raw, 3);
        r = line_pos_step(raw);
        check_near(r.err_mm, -3.0f, 0.01f, "中心左侧一路 → err=-3mm");
    }

    /* ---- 3. 相邻两路各半 → 取中点 ---- */
    {
        line_pos_init(&cfg);
        fill_black(raw);
        raw[3] = 3000;   /* p=1，位置 -3 */
        raw[4] = 3000;   /* p=1，位置 +3 */
        r = line_pos_step(raw);
        check_near(r.err_mm, 0.0f, 0.01f, "3/4 路都亮 → err=0（线在正中间）");

        /* 一边强一边弱 → 偏向强的那边 */
        raw[4] = 1600;   /* p=0.5 */
        r = line_pos_step(raw);
        /* (1×(-3) + 0.5×3) / 1.5 = -1 → 偏向左边 */
        check_near(r.err_mm, -1.0f, 0.02f, "强弱不等 → 偏向强侧");
    }

    /* ---- 4. 丢线判据 ---- */
    {
        line_pos_init(&cfg);
        fill_black(raw);
        r = line_pos_step(raw);
        check(r.visible == 0u, "全黑 → 丢线");
        check_near(r.err_mm, 0.0f, 1e-4f, "丢线时 err 不许给随机值（=0）");
        check_near(r.sum_p, 0.0f, 1e-4f, "丢线时 Σp=0");
    }

    /* ---- 5. 钳位：被强光晃到（读数超出白基准）不许把质心拽飞 ---- */
    {
        line_pos_init(&cfg);
        light_one(raw, 4);
        raw[0] = 4000;   /* 远超白基准 → 应被钳成 p=1，而不是 p=1.36 */
        r = line_pos_step(raw);
        /* 两路都是 p=1：位置 -21 和 +3 → 中点 -9 */
        check_near(r.err_mm, -9.0f, 0.02f, "超量程读数被钳到 p=1");
        check_near(r.sum_p, 2.0f, 0.01f, "钳位后 Σp 不超过路数");
    }

    /* ---- 6. 读数低于黑基准 → 钳成 0 ---- */
    {
        line_pos_init(&cfg);
        light_one(raw, 0);
        raw[1] = 50;     /* 比黑还低 → 应为 0 */
        r = line_pos_step(raw);
        check_near(r.p[1], 0.0f, 1e-4f, "低于黑基准 → p=0");
        check_near(r.err_mm, -21.0f, 0.01f, "负向超量程不干扰结果");
    }

    /* ---- 7. 每路各自标定：8 路基准不同也能正确归一化 ---- */
    {
        line_pos_cfg_t c2 = std_cfg();
        uint8_t i;
        /* 第 0 路灵敏度天生差：黑 500 / 白 1500 */
        c2.black[0] = 500.0f;
        c2.white[0] = 1500.0f;
        for (i = 1; i < LINE_CH; i++) {
            c2.black[i] = 200.0f;
            c2.white[i] = 3000.0f;
        }
        line_pos_init(&c2);
        fill_black(raw);
        raw[0] = 500;    /* 对这路来说就是"黑" */
        raw[1] = 200;    /* 对那路来说也是"黑" */
        raw[2] = 200;
        raw[3] = 200;
        raw[4] = 200;
        raw[5] = 200;
        raw[6] = 200;
        raw[7] = 200;
        r = line_pos_step(raw);
        check(r.visible == 0u, "各自标定后全黑 → 仍然判丢线");

        raw[0] = 1500;   /* 第 0 路看到白 */
        r = line_pos_step(raw);
        check_near(r.p[0], 1.0f, 1e-3f, "灵敏度差的通道也能到 p=1");
        check_near(r.err_mm, -21.0f, 0.02f, "各自标定后位置仍正确");
    }

    /* ---- 8. 白比黑还暗（极性反）也能算 ---- */
    {
        line_pos_cfg_t c3 = std_cfg();
        uint8_t i;
        for (i = 0; i < LINE_CH; i++) {
            c3.black[i] = 3000.0f;   /* 反极性 */
            c3.white[i] = 200.0f;
        }
        line_pos_init(&c3);
        {
            uint8_t k;
            for (k = 0; k < LINE_CH; k++) {
                raw[k] = 200;   /* 反极性下 200 = "白" ⇒ 8 路都看见线 */
            }
        }
        r = line_pos_step(raw);
        /* 语义说明：8 路全"看见白"= 线铺满视野，质心必然落在正中间。
         * "视野里全白是不是合理" 属于上层策略（比如再判一句饱和路数），
         * 不归这一层管 —— 这一层只负责"几何上算得对"。 */
        check(r.visible == 1u, "反极性全白 → 8 路都算看见线");
        check_near(r.err_mm, 0.0f, 0.01f, "反极性全白 → 质心在正中间");

        for (i = 0; i < LINE_CH; i++) {
            raw[i] = 3000;   /* 全是"黑" */
        }
        raw[4] = 200;        /* 第 4 路是"白" */
        r = line_pos_step(raw);
        check_near(r.err_mm, 3.0f, 0.02f, "反极性下也能定位");
    }

    /* ---- 9. 黑白基准相同（没标定）→ 不该贡献，也不该除以 0 ---- */
    {
        line_pos_cfg_t c4 = std_cfg();
        uint8_t i;
        for (i = 0; i < LINE_CH; i++) {
            c4.black[i] = 1000.0f;
            c4.white[i] = 1000.0f;
        }
        line_pos_init(&c4);
        { uint8_t k; for (k = 0; k < LINE_CH; k++) { raw[k] = 2000; } }
        r = line_pos_step(raw);
        check(r.visible == 0u, "未标定 → 丢线");
        check(r.err_mm == r.err_mm, "未标定 → 不是 NaN");
    }

    /* ---- 10. line_pos_set_ref 现场修正某一路 ---- */
    {
        line_pos_init(&cfg);
        line_pos_set_ref(2, 400.0f, 2000.0f);
        check_near(line_pos_cfg()->black[2], 400.0f, 1e-4f, "set_ref 改黑基准");
        check_near(line_pos_cfg()->white[2], 2000.0f, 1e-4f, "set_ref 改白基准");
        fill_black(raw);
        raw[2] = 1200.0f;   /* 对这路正好是中间 */
        r = line_pos_step(raw);
        check_near(r.p[2], 0.5f, 0.01f, "set_ref 后归一化生效");
    }

    printf("\n结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
