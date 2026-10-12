/**
  ******************************************************************************
  * @file    telemetry.c
  * @brief   遥测实现
  ******************************************************************************
  */
#include "telemetry.h"
#include "bcar_config.h"
#include "uart_dbg.h"

#define TLM_LINE_MAX    160u    /* 必须 <= BCAR_UART_TX_BUF */
#define TLM_RT_DIV      20u     /* 实时行节奏：20 × 5ms = 100ms */

static lapstat_t    s_acc;
static lap_record_t s_last;
static uint32_t     s_lap_index = 0u;
static uint8_t      s_rt_on     = 0u;
static uint32_t     s_rt_div    = 0u;
static char         s_line[TLM_LINE_MAX];

/* 把带小数的量按"整数.十分位"分两半打（迷你 printf 没有 %f）*/
#define TENTHS(v)   ((int)((v) * 10.0f))
#define IPART(v)    (TENTHS(v) / 10)
#define FPART(v)    (((TENTHS(v) % 10) + 10) % 10)

void telemetry_init(void)
{
    lapstat_reset(&s_acc);
    s_lap_index = 1u;
    s_rt_on     = 0u;
    s_rt_div    = 0u;
}

void telemetry_tick(float err_mm, uint8_t visible, float v_mm_s,
                    float ds_mm, float yaw_deg)
{
    uint16_t n;

    lapstat_push(&s_acc, err_mm, visible, v_mm_s, ds_mm, BCAR_TICK_MS);

    if (!s_rt_on) {
        return;
    }
    s_rt_div++;
    if (s_rt_div < TLM_RT_DIV) {
        return;
    }
    s_rt_div = 0u;

    /* 实时短行：手推车时盯这几个数就够 —— 偏差、线在不在、速度、航向、丢线累计 */
    n = uart_dbg_snprintf(s_line, TLM_LINE_MAX,
                          "TLM t=%u err=%d.%d mm vis=%u v=%u mm/s yaw=%d.%d deg lost=%u\r\n",
                          (unsigned)(s_acc.t_ms),
                          IPART(err_mm), FPART(err_mm),
                          (unsigned)visible,
                          (unsigned)((uint32_t)(v_mm_s < 0.0f ? 0.0f : v_mm_s)),
                          IPART(yaw_deg), FPART(yaw_deg),
                          (unsigned)s_acc.lost_ticks);
    (void)uart_dbg_write_dma((const uint8_t *)s_line, n);
}

void telemetry_start_lap(void)
{
    lapstat_reset(&s_acc);
    s_rt_div = 0u;
}

uint8_t telemetry_finish_lap(void)
{
    uint16_t n;

    s_last.lap_index = s_lap_index;
    if (!lapstat_finish(&s_acc, &s_last)) {
        return 0u;      /* 太短，不算一圈；累加器保持不动，让调用方自己决定重开 */
    }

    /* LAP 行：调参迭代器直接吃这一行。
     * 字段：圈号 / 用时 / 里程 / 平均速度 / 最高速度 / |偏差|均值 / |偏差|P95 / 丢线拍数 */
    n = uart_dbg_snprintf(s_line, TLM_LINE_MAX,
                          "LAP %u t=%u ms s=%u mm vavg=%u mm/s vmax=%u mm/s "
                          "errMean=%d.%d mm errP95=%d.%d mm lost=%u\r\n",
                          (unsigned)s_last.lap_index,
                          (unsigned)s_last.t_ms,
                          (unsigned)((uint32_t)(s_last.s_mm < 0.0f ? 0.0f : s_last.s_mm)),
                          (unsigned)((uint32_t)s_last.v_avg_mm_s),
                          (unsigned)((uint32_t)s_last.v_max_mm_s),
                          IPART(s_last.abs_err_mean_mm), FPART(s_last.abs_err_mean_mm),
                          IPART(s_last.abs_err_p95_mm), FPART(s_last.abs_err_p95_mm),
                          (unsigned)s_last.lost_ticks);
    (void)uart_dbg_write_dma((const uint8_t *)s_line, n);

    s_lap_index++;      /* 结算完自动进下一圈 */
    telemetry_start_lap();
    return 1u;
}

void telemetry_set_realtime(uint8_t on)
{
    s_rt_on  = (on != 0u) ? 1u : 0u;
    s_rt_div = 0u;
}

uint8_t telemetry_realtime(void)
{
    return s_rt_on;
}

const lap_record_t *telemetry_last_lap(void)
{
    return &s_last;
}

uint32_t telemetry_lap_ticks(void)
{
    return s_acc.ticks;
}

uint32_t telemetry_lap_index(void)
{
    return s_lap_index;
}
