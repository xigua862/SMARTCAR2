/**
  ******************************************************************************
  * @file    cmdline.c
  * @brief   串口命令行实现（解析逻辑在 str_util.c，本文件只做"解析→动作"的映射）
  ******************************************************************************
  */
#include "cmdline.h"
#include "bcar_config.h"
#include "stm32f1xx_hal.h"
#include "bsp_afio.h"
#include "bsp_systick.h"
#include "buzzer.h"
#include "fsm.h"
#include "key.h"
#include "led.h"
#include "motor.h"
#include "fan.h"
#include "adc_ir.h"
#include "control.h"
#include "encoder.h"
#include "imu660.h"
#include "line_pos.h"
#include "heading.h"
#include "odom.h"
#include "str_util.h"
#include "telemetry.h"
#include "uart_dbg.h"

volatile uint8_t g_hello_en = 1u;

/* 由 main.c 维护的最新灰度结果 */
extern line_pos_t s_line_out;

#define LINE_MAX 64u

static char    s_line[LINE_MAX];
static uint8_t s_len = 0u;

/* 陀螺零偏标定：跨多拍采样，所以状态放在文件作用域 */
static heading_cal_t s_cal;
static uint8_t       s_cal_active = 0u;
static uint8_t       s_cal_auto   = 0u;    /* 1 = 这次是发车读秒里自动采的 */
static float         s_cal_gz_min = 0.0f;  /* 采集期间 gz 的极值 → 判"车有没有在动" */
static float         s_cal_gz_max = 0.0f;

/** @brief 起一次零偏采集（manual = 0：手敲 GYROZERO；1：发车读秒里自动）*/
static void gyrocal_begin(uint8_t auto_mode)
{
    heading_cal_begin(&s_cal);
    s_cal_auto   = auto_mode;
    s_cal_gz_min = 1.0e9f;
    s_cal_gz_max = -1.0e9f;
    s_cal_active = 1u;
}

/* ---------------- 帮助 ---------------- */

static void print_help(void)
{
    uart_dbg_puts("\r\n命令（大小写不敏感，参数用空格分隔）:\r\n");
    uart_dbg_puts("发车流程: 上电待机(1 声) → PB0=测试模式(2 声,等串口指令)\r\n");
    uart_dbg_puts("          / PC13 长按 200ms=发车(3 秒读秒:短短长,期间自动标陀螺零偏) → RUN\r\n");
    uart_dbg_puts("  HELP                  这页帮助\r\n");
    uart_dbg_puts("  PING                  应答 PONG（测链路）\r\n");
    uart_dbg_puts("  STATUS                当前状态/节拍/电机/按键/MOE\r\n");
    uart_dbg_puts("  MAPR                  打印 AFIO->MAPR（查重映射是否生效）\r\n");
    uart_dbg_puts("  HELLO <0|1>           每秒心跳打印开关\r\n");
    uart_dbg_puts("  BEEP [ms]             蜂鸣（默认 100ms）\r\n");
    uart_dbg_puts("  MODE <IDLE|TEST|COUNTDOWN|RUN|ESTOP>  切模式（COUNTDOWN=3 秒读秒）\r\n");
    uart_dbg_puts("  MOTOR <dir> <mag>     两侧同设；dir=FWD|REV|BRAKE|COAST，mag=0~1000\r\n");
    uart_dbg_puts("  MOTOR L|R <dir> <mag> 只设一侧（L=PA8/PA9，R=PA10/PA11）\r\n");
    uart_dbg_puts("  KEY                   打印两个按键当前电平\r\n");
    uart_dbg_puts("  ENC                   编码器累计计数/本拍增量（★标定用）\r\n");
    uart_dbg_puts("  ODOM                  里程与位姿（x/y/yaw/s/v）\r\n");
    uart_dbg_puts("  ZERO                  编码器与位姿清零（标定起点）\r\n");
    uart_dbg_puts("  IMU                   陀螺/加速度原始值与 dps（查 SPI 接线用）\r\n");
    uart_dbg_puts("  GYROZERO              陀螺零偏标定（车静止约 1 秒）\r\n");
    uart_dbg_puts("  HEAD                  当前航向、零偏、死区\r\n");
    uart_dbg_puts("  IR                    8 路灰度原始值 + 归一化 + 偏差（查探头用）\r\n");
    uart_dbg_puts("  IRBLACK               把当前读数教成全黑基准（车放黑底上执行）\r\n");
    uart_dbg_puts("  IRWHITE               把当前读数教成白线基准（车放白线上执行）\r\n");
    uart_dbg_puts("  IRPOS <间距mm*100>    改探头间距（如 IRPOS 600 = 6.00mm）\r\n");
    uart_dbg_puts("  LAP                   结算当前圈并发一行 LAP（走 DMA），然后自动进下一圈\r\n");
    uart_dbg_puts("  LAPSTART              丢弃当前累积，重开一圈\r\n");
    uart_dbg_puts("  TLM <0|1>             实时行开关（每 100ms 一行 err/vis/v/yaw/lost）\r\n");
    uart_dbg_puts("  PD <kp*100> <kd*100>  现场改转向增益（PD 2500 300 → kp=25.00 kd=3.00）\r\n");
    uart_dbg_puts("  BASE <mag>            现场改循迹速度基准（0~1000）\r\n");
    uart_dbg_puts("  GAIN                  看当前增益/模式/丢线时长\r\n");
    uart_dbg_puts("  SGAIN <kp*100> <kd*100>  现场改\"陀螺直行\"增益（SGAIN 1200 150 → 12.0 / 1.5）\r\n");
    uart_dbg_puts("  SDIR <1|-1>           翻陀螺直行方向（直行时车越偏越远就试另一个）\r\n");
    uart_dbg_puts("                        —— 看不到线时车靠陀螺直行找线，2.5 秒还没找到才刹\r\n");
    uart_dbg_puts("  LOOPBACK [ms]         TX-RX 短接时的回环自检（默认 200ms）\r\n");
}

static void cmd_status(void)
{
    motor_dir_t dl = MOTOR_BRAKE, dr = MOTOR_BRAKE;
    uint16_t    ml = 0u, mr = 0u;

    motor_get_last(MOTOR_L, &dl, &ml);
    motor_get_last(MOTOR_R, &dr, &mr);

    /* 版本 + 编译时间戳：对着 hex 的文件时间一比，就知道板上是不是最新那版 */
    uart_dbg_printf("FW %s built %s\r\n", BCAR_FW_VERSION, BCAR_FW_BUILD);
    uart_dbg_printf("STATE = %s  (进入于 %u ms)\r\n",
                    fsm_state_name(fsm_state()), bsp_millis() - fsm_state_enter_ms());
    uart_dbg_printf("t = %u ms, tick = %u ms\r\n", bsp_millis(), BCAR_TICK_MS);
    uart_dbg_printf("MOTOR L: dir=%u mag=%u | R: dir=%u mag=%u  (dir: 0=FWD 1=REV 2=BRAKE 3=COAST)\r\n",
                    (unsigned)dl, (unsigned)ml, (unsigned)dr, (unsigned)mr);
    uart_dbg_printf("KEY  START(PC13)=%u held=%u ms  TRIG(PB0)=%u  (1=按住)\r\n",
                    (unsigned)key_is_down(KEY_START), (unsigned)key_hold_ms(KEY_START),
                    (unsigned)key_is_down(KEY_TRIG));
    uart_dbg_printf("BUZZER = %u, HELLO = %u\r\n",
                    (unsigned)buzzer_is_on(), (unsigned)g_hello_en);
    uart_dbg_printf("AFIO MAPR = 0x%08X\r\n", bsp_afio_mapr());
}

/* ---------------- MOTOR ---------------- */

static void cmd_motor(char *line)
{
    su_motor_t m;

    if (!su_parse_motor(line, &m)) {
        uart_dbg_puts("ERR 用法: MOTOR <FWD|REV|BRAKE|COAST> <0..1000>\r\n");
        uart_dbg_puts("         或: MOTOR L|R <FWD|REV|BRAKE|COAST> <0..1000>\r\n");
        return;
    }

    /* 硬约束：运行中不许滑行（AT8236 会进休眠，再想动要等唤醒）*/
    if (m.dir == SU_DIR_COAST && !fsm_coast_allowed()) {
        uart_dbg_puts("ERR 运行中不许 COAST（会让驱动休眠）；要停请用 BRAKE\r\n");
        return;
    }

    if (m.has_side) {
        motor_set((motor_side_t)m.side, (motor_dir_t)m.dir, (uint16_t)m.mag);
        uart_dbg_printf("OK MOTOR %s dir=%u mag=%u\r\n",
                        m.side ? "R" : "L", (unsigned)m.dir, (unsigned)m.mag);
    } else {
        motor_set_all((motor_dir_t)m.dir, (uint16_t)m.mag);
        uart_dbg_printf("OK MOTOR BOTH dir=%u mag=%u\r\n",
                        (unsigned)m.dir, (unsigned)m.mag);
    }
}

/* ---------------- 单个数值参数（HELLO / BEEP / LOOPBACK 用） ---------------- */

static uint8_t arg_u32(char *line, uint32_t *out)
{
    char *a1 = su_arg(line, 1u);
    if (a1 == 0) {
        return 0u;
    }
    su_tok(a1);
    return su_parse_u32(a1, out);
}

/* 打印定点小数（一位）—— 我们的 mini printf 不支持 %f（软浮点格式化很占空间），
 * 所以在整数域里自己拆"整数.小数"，精度够看里程用。 */
static void print_fixed1(const char *label, float v)
{
    int32_t  scaled;
    uint32_t ip, fp;

    if (v < 0.0f) {
        uart_dbg_printf("%s=-", label);
        v = -v;
    } else {
        uart_dbg_printf("%s=", label);
    }
    scaled = (int32_t)(v * 10.0f + 0.5f);
    ip = (uint32_t)(scaled / 10);
    fp = (uint32_t)(scaled % 10);
    uart_dbg_printf("%u.%u", ip, fp);
}
#define PRINT_FIXED(label, v)  print_fixed1(label, v)

/* ---------------- 分发 ---------------- */

static void execute(char *line)
{
    char  w[12];
    char *cmd;

    su_upper(line);
    cmd = su_arg(line, 0u);
    if (cmd == 0) {
        return;
    }
    /* ★取命令词到局部缓冲，**不截断原文** ——
     * 一旦截断，"MOTOR FWD 500" 后面的参数就取不到了（这是修过的一个真 bug）。 */
    if (su_word(cmd, w, (uint8_t)sizeof(w)) == 0u) {
        uart_dbg_puts("ERR 命令词太长\r\n");
        return;
    }

    if (su_streq(w, "HELP") || su_streq(w, "?")) {
        print_help();
    } else if (su_streq(w, "PING")) {
        uart_dbg_puts("PONG\r\n");
    } else if (su_streq(w, "STATUS")) {
        cmd_status();
    } else if (su_streq(w, "MAPR")) {
        uart_dbg_printf("MAPR = 0x%08X (%s)\r\n",
                        bsp_afio_mapr(), bsp_afio_mapr_explain(bsp_afio_mapr()));
    } else if (su_streq(w, "HELLO")) {
        uint32_t v = 1u;
        if (arg_u32(line, &v)) {
            g_hello_en = (v != 0u) ? 1u : 0u;
        }
        uart_dbg_printf("OK HELLO=%u\r\n", (unsigned)g_hello_en);
    } else if (su_streq(w, "BEEP")) {
        uint32_t ms = 100u;
        if (arg_u32(line, &ms) && ms > 5000u) {
            ms = 5000u;
        }
        buzzer_beep(ms);
        uart_dbg_printf("OK BEEP %u ms\r\n", (unsigned)ms);
    } else if (su_streq(w, "MODE")) {
        char *a1 = su_arg(line, 1u);
        fsm_state_t s;
        if (a1 == 0) {
            uart_dbg_puts("ERR 用法: MODE <IDLE|TEST|RUN|ESTOP>\r\n");
            return;
        }
        su_tok(a1);
        if (su_streq(a1, "IDLE"))           { s = FSM_IDLE; }
        else if (su_streq(a1, "TEST"))      { s = FSM_TEST; }
        else if (su_streq(a1, "COUNTDOWN")) { s = FSM_COUNTDOWN; }   /* 台上试读秒用 */
        else if (su_streq(a1, "RUN"))       { s = FSM_RUN; }
        else if (su_streq(a1, "ESTOP"))     { s = FSM_ESTOP; }
        else {
            uart_dbg_puts("ERR 模式只能是 IDLE/TEST/COUNTDOWN/RUN/ESTOP\r\n");
            return;
        }
        (void)fsm_set(s);
        uart_dbg_printf("OK MODE %s\r\n", fsm_state_name(fsm_state()));
        if (fsm_state() == FSM_RUN) {
            uart_dbg_puts("   （RUN：循迹环接管电机；正式发车请用 PC13 长按走读秒流程）\r\n");
        } else if (fsm_state() == FSM_COUNTDOWN) {
            uart_dbg_puts("   （3 秒读秒开始，短短长 3 声后自动进 RUN；PB0 可取消）\r\n");
        } else {
            uart_dbg_puts("   （电机已刹车）\r\n");
        }
    } else if (su_streq(w, "MOTOR") || su_streq(w, "M")) {
        cmd_motor(line);
    } else if (su_streq(w, "IMU")) {
        imu660_read();
        uart_dbg_printf("IMU raw G(%d,%d,%d) A(%d,%d,%d) reads=%u\r\n",
                        (int)imu660_raw_gyro(IMU_AXIS_X), (int)imu660_raw_gyro(IMU_AXIS_Y),
                        (int)imu660_raw_gyro(IMU_AXIS_Z),
                        (int)imu660_raw_acc(IMU_AXIS_X), (int)imu660_raw_acc(IMU_AXIS_Y),
                        (int)imu660_raw_acc(IMU_AXIS_Z), (unsigned)imu660_read_count());
        /* dps 用定点打印（迷你 printf 没有 %f）*/
        uart_dbg_puts("    gyro(dps*10) ");
        uart_dbg_printf("x=%d y=%d z=%d\r\n",
                        (int)(imu660_gyro_dps(IMU_AXIS_X) * 10.0f),
                        (int)(imu660_gyro_dps(IMU_AXIS_Y) * 10.0f),
                        (int)(imu660_gyro_dps(IMU_AXIS_Z) * 10.0f));
    } else if (su_streq(w, "GYROZERO")) {
        gyrocal_begin(0u);
        uart_dbg_printf("GYROZERO 采集中（%u 拍 ≈ %u ms）—— 现在开始别碰车\r\n",
                        (unsigned)BCAR_GYRO_CAL_SAMPLES, (unsigned)BCAR_GYRO_CAL_MS);
    } else if (su_streq(w, "HEAD")) {
        PRINT_FIXED("yaw_deg", heading_yaw_deg());
        PRINT_FIXED("  bias_z", heading_bias(2));
        PRINT_FIXED("  dead_zone", heading_dead_zone());
        uart_dbg_printf("  samples=%u\r\n", (unsigned)heading_samples());
    } else if (su_streq(w, "IR")) {
        const uint16_t *raw = adc_ir_raw_all();
        const line_pos_t *lp = &s_line_out;
        uint8_t k;
        uart_dbg_puts("IR raw  ");
        for (k = 0u; k < LINE_CH; k++) {
            uart_dbg_printf("%4u ", (unsigned)raw[k]);
        }
        uart_dbg_puts("\r\nIR p*100 ");
        for (k = 0u; k < LINE_CH; k++) {
            uart_dbg_printf("%4d ", (int)(lp->p[k] * 100.0f));
        }
        uart_dbg_printf("\r\nIR err_mm_x10=%d sum_p_x100=%d visible=%u seq=%u\r\n",
                        (int)(lp->err_mm * 10.0f), (int)(lp->sum_p * 100.0f),
                        (unsigned)lp->visible, (unsigned)adc_ir_seq_count());
    } else if (su_streq(w, "IRBLACK") || su_streq(w, "IRWHITE")) {
        uint32_t sum[LINE_CH];
        uint8_t  k, n;
        uint8_t  as_black = su_streq(w, "IRBLACK") ? 1u : 0u;
        for (k = 0u; k < LINE_CH; k++) {
            sum[k] = 0u;
        }
        for (n = 0u; n < (uint8_t)BCAR_IR_CAL_SAMPLES; n++) {
            const uint16_t *r = adc_ir_raw_all();
            for (k = 0u; k < LINE_CH; k++) {
                sum[k] += (uint32_t)r[k];
            }
            HAL_Delay(2);
        }
        for (k = 0u; k < LINE_CH; k++) {
            float avg = (float)sum[k] / (float)BCAR_IR_CAL_SAMPLES;
            const line_pos_cfg_t *c = line_pos_cfg();
            if (as_black) {
                line_pos_set_ref(k, avg, c->white[k]);
            } else {
                line_pos_set_ref(k, c->black[k], avg);
            }
        }
        uart_dbg_puts(as_black ? "OK 已教成\"全黑\"基准："
                               : "OK 已教成\"白线\"基准：");
        {
            const line_pos_cfg_t *c = line_pos_cfg();
            for (k = 0u; k < LINE_CH; k++) {
                uart_dbg_printf("%d ", (int)(as_black ? c->black[k] : c->white[k]));
            }
        }
        uart_dbg_puts("\r\n（黑白基准是 RAM 里的，掉电就没了；标定完请把数抄进 bcar_config.h）\r\n");
    } else if (su_streq(w, "IRPOS")) {
        uint32_t pitch_x100 = 600u;
        if (arg_u32(line, &pitch_x100) && pitch_x100 > 0u && pitch_x100 < 10000u) {
            line_pos_cfg_t *c = line_pos_cfg_mut();
            float pitch = (float)pitch_x100 / 100.0f;
            uint8_t k;
            for (k = 0u; k < LINE_CH; k++) {
                c->pos_mm[k] = ((float)k - 3.5f) * pitch;
            }
            uart_dbg_printf("OK 探头间距设为 %u.%02u mm（位置已按中心对称重排）\r\n",
                            (unsigned)(pitch_x100 / 100u), (unsigned)(pitch_x100 % 100u));
        } else {
            uart_dbg_puts("ERR 用法: IRPOS <间距mm*100>，例如 IRPOS 600 = 6.00mm\r\n");
        }
    } else if (su_streq(w, "LAP")) {
        if (telemetry_finish_lap()) {
            /* 具体那行由 telemetry 自己用 DMA 发出去，这里只回一句确认 */
            uart_dbg_printf("OK LAP 已结算（第 %u 圈），下一圈已开始\r\n",
                            (unsigned)(telemetry_lap_index() - 1u));
        } else {
            uart_dbg_printf("ERR 这一圈太短（%u 拍 < %u），没结算；要重开就用 LAPSTART\r\n",
                            (unsigned)telemetry_lap_ticks(), (unsigned)LAPSTAT_MIN_TICKS);
        }
    } else if (su_streq(w, "LAPSTART")) {
        telemetry_start_lap();
        uart_dbg_printf("OK 第 %u 圈开始计（累加器已清零）\r\n",
                        (unsigned)telemetry_lap_index());
    } else if (su_streq(w, "TLM")) {
        uint32_t v = 1u;
        if (arg_u32(line, &v)) {
            telemetry_set_realtime((v != 0u) ? 1u : 0u);
        }
        uart_dbg_printf("OK TLM=%u（%s）\r\n",
                        (unsigned)telemetry_realtime(),
                        telemetry_realtime() ? "每 100ms 一行" : "已关");
    } else if (su_streq(w, "PD")) {
        char *a1 = su_arg(line, 1u);
        char *a2 = su_arg(line, 2u);
        if (a1 == 0 || a2 == 0) {
            uart_dbg_puts("ERR 用法: PD <kp*100> <kd*100>，例如 PD 2500 300 → kp=25.0 kd=3.0\r\n");
        } else {
            uint32_t kp100 = 0u, kd100 = 0u;
            su_tok(a1); su_tok(a2);
            if (su_parse_u32(a1, &kp100) && su_parse_u32(a2, &kd100)) {
                control_set_pd((float)kp100 / 100.0f, (float)kd100 / 100.0f);
                {
                    float kp = 0.0f, kd = 0.0f;
                    control_get_pd(&kp, &kd);
                    uart_dbg_printf("OK PD kp=%u.%02u kd=%u.%02u\r\n",
                                    (unsigned)(kp100 / 100u), (unsigned)(kp100 % 100u),
                                    (unsigned)(kd100 / 100u), (unsigned)(kd100 % 100u));
                }
            } else {
                uart_dbg_puts("ERR 增益必须是整数（按 1/100 记）\r\n");
            }
        }
    } else if (su_streq(w, "BASE")) {
        uint32_t v = 0u;
        if (arg_u32(line, &v) && v <= 1000u) {
            control_set_base((uint16_t)v);
            uart_dbg_printf("OK BASE=%u\r\n", (unsigned)control_base());
        } else {
            uart_dbg_puts("ERR 用法: BASE <0..1000>\r\n");
        }
    } else if (su_streq(w, "GAIN")) {
        float kp = 0.0f, kd = 0.0f;
        float skp = 0.0f, skd = 0.0f, stgt = 0.0f;
        int8_t ssign = 1;
        control_get_pd(&kp, &kd);
        control_get_straight(&skp, &skd, &ssign, &stgt);
        uart_dbg_printf("BASE=%u kp_x100=%d kd_x100=%d mode=%s lost=%u ms holding=%u\r\n",
                        (unsigned)control_base(),
                        (int)(kp * 100.0f), (int)(kd * 100.0f),
                        control_mode_name(),
                        (unsigned)control_lost_ms(), (unsigned)control_lost_holding());
        uart_dbg_printf("STRAIGHT skp_x100=%d skd_x100=%d sign=%d target_x10=%d deg\r\n",
                        (int)(skp * 100.0f), (int)(skd * 100.0f), (int)ssign, (int)(stgt * 10.0f));
    } else if (su_streq(w, "SGAIN")) {
        char *a1 = su_arg(line, 1u);
        char *a2 = su_arg(line, 2u);
        if (a1 == 0 || a2 == 0) {
            uart_dbg_puts("ERR 用法: SGAIN <kp*100> <kd*100>，例如 SGAIN 1200 150 → kp=12.0 kd=1.5\r\n");
        } else {
            uint32_t kp100 = 0u, kd100 = 0u;
            su_tok(a1); su_tok(a2);
            if (su_parse_u32(a1, &kp100) && su_parse_u32(a2, &kd100)) {
                control_set_straight((float)kp100 / 100.0f, (float)kd100 / 100.0f);
                uart_dbg_printf("OK SGAIN skp=%u.%02u skd=%u.%02u\r\n",
                                (unsigned)(kp100 / 100u), (unsigned)(kp100 % 100u),
                                (unsigned)(kd100 / 100u), (unsigned)(kd100 % 100u));
            } else {
                uart_dbg_puts("ERR 增益必须是整数（按 1/100 记）\r\n");
            }
        }
    } else if (su_streq(w, "LSDIR")) {
        uint32_t v = 1u;
        if (arg_u32(line, &v) && (v == 1u || v == 2u)) {
            int8_t sign = (v == 1u) ? (int8_t)1 : (int8_t)-1;
            control_set_line_sign(sign);
            uart_dbg_printf("OK LSDIR=%d（循迹环方向；1=默认 2=反过来）\r\n", (int)sign);
        } else {
            uart_dbg_puts("ERR 用法: LSDIR <1|2>  （车朝远离线的方向拐 ⇒ 试 2）\r\n");
        }
    } else if (su_streq(w, "SDIR")) {
        uint32_t v = 1u;
        if (arg_u32(line, &v) && (v == 1u || v == 2u)) {
            /* 收 1 或 2 都行：2 当 -1 用（免得纠结负号怎么打）*/
            int8_t sign = (v == 1u) ? (int8_t)1 : (int8_t)-1;
            control_set_straight_sign(sign);
            uart_dbg_printf("OK SDIR=%d（陀螺直行方向；1=默认 2=反过来）\r\n", (int)sign);
        } else {
            uart_dbg_puts("ERR 用法: SDIR <1|2>  （1 = 默认方向；2 = 反过来）\r\n");
        }
    } else if (su_streq(w, "FAN")) {
        /* 风机（负压）：0 = 关，1 / 2 = 两个吹风方向 —— 试出"往下压"的那个，
         * 它就同时被记成"发车时用的方向"（读秒会自动开它）。 */
        uint32_t v = 0u;
        if (arg_u32(line, &v) && v <= 2u) {
            if (v == 0u) {
                fan_off();
                uart_dbg_puts("OK FAN 关（发车方向不变）\r\n");
            } else {
                fan_set_dir((uint8_t)v);
                uart_dbg_printf("OK FAN %u（已记作发车方向；试一下风往哪吹，选往下压的那个）\r\n",
                                (unsigned)v);
            }
        } else {
            uart_dbg_puts("ERR 用法: FAN <0|1|2>  （0=关 1=正吹 2=反吹）\r\n");
        }
    } else if (su_streq(w, "KEY")) {
        /* held = 这一下按住多久了 —— 台上验"长按 200ms 发车"就看它 */
        uart_dbg_printf("KEY START(PC13)=%u held=%u ms TRIG(PB0)=%u (1=按住)\r\n",
                        (unsigned)key_is_down(KEY_START), (unsigned)key_hold_ms(KEY_START),
                        (unsigned)key_is_down(KEY_TRIG));
    } else if (su_streq(w, "ENC")) {
        /* ★标定就看这行：推车 1 米，看 total 涨了多少 → mm_per_count = 1000/计数 */
        uart_dbg_printf("ENC L total=%d delta=%d | R total=%d delta=%d\r\n",
                        (int)encoder_total(ENC_L), (int)encoder_delta(ENC_L),
                        (int)encoder_total(ENC_R), (int)encoder_delta(ENC_R));
    } else if (su_streq(w, "ODOM")) {
        odom_state_t o = odom_get();
        PRINT_FIXED("x_mm", o.x_mm);
        PRINT_FIXED(" y_mm", o.y_mm);
        uart_dbg_puts("\r\n");
        PRINT_FIXED("yaw_deg", o.yaw_deg);
        PRINT_FIXED(" s_mm", o.s_mm);
        PRINT_FIXED(" v_mm/s", o.v_mm_s);
        uart_dbg_puts("\r\n");
    } else if (su_streq(w, "ZERO")) {
        encoder_reset();
        odom_reset();
        uart_dbg_puts("OK ZERO（编码器与位姿已清零）\r\n");
    } else if (su_streq(w, "LOOPBACK")) {
        uint32_t ms = 200u;
        uint8_t  ok;
        (void)arg_u32(line, &ms);
        ok = uart_dbg_loopback_test(ms);
        if (ok) {
            uart_dbg_puts("LOOPBACK PASS（能发也能收）\r\n");
        } else {
            uart_dbg_puts("LOOPBACK FAIL（只发不收：查 TX-RX 有没有短接）\r\n");
        }
    } else {
        uart_dbg_printf("ERR 不认识的命令: %s（打 HELP 看列表）\r\n", w);
    }
}

/* ---------------- 陀螺零偏标定（跨多拍，所以要有状态） ---------------- */

void cmdline_gyrozero_auto_begin(void)
{
    gyrocal_begin(1u);
    uart_dbg_puts("[GYRO] 读秒期间自动标零偏：采 1 秒静止样本，车别动\r\n");
}

void cmdline_tick_5ms(void)
{
    /* 标定期间：每拍采一个静止样本，够了就用均值当零偏 */
    if (s_cal_active)
    {
        float bias[3];
        float gz;
        imu660_read();
        gz = imu660_gyro_dps(IMU_AXIS_Z);
        heading_cal_push(&s_cal, imu660_gyro_dps(IMU_AXIS_X),
                                 imu660_gyro_dps(IMU_AXIS_Y),
                                 gz);
        if (gz < s_cal_gz_min) { s_cal_gz_min = gz; }
        if (gz > s_cal_gz_max) { s_cal_gz_max = gz; }

        if (s_cal.n >= (uint32_t)BCAR_GYRO_CAL_SAMPLES)
        {
            float spread = s_cal_gz_max - s_cal_gz_min;
            if (heading_cal_finish(&s_cal, bias, (uint32_t)(BCAR_GYRO_CAL_SAMPLES / 2u)))
            {
                /* 自动采集时多一道门：车在动就别标 —— 那时采到的"均值"是转速不是零偏，
                 * 标进去比不标更糟（宁可不标）。手敲 GYROZERO 不加这道门（人在看着）。 */
                if (s_cal_auto && spread > BCAR_GYRO_AUTOCAL_MAX_SPREAD_DPS)
                {
                    uart_dbg_puts("\r\n[GYRO] 放弃自动标定：采集期间车在动（极差");
                    PRINT_FIXED(" ", spread);
                    PRINT_FIXED(" dps，限 ", BCAR_GYRO_AUTOCAL_MAX_SPREAD_DPS);
                    uart_dbg_puts("），沿用原零偏\r\n");
                }
                else
                {
                    heading_set_bias(bias[0], bias[1], bias[2]);
                    uart_dbg_puts(s_cal_auto ? "\r\n[GYRO] 自动标定完成，零偏(dps) x="
                                             : "\r\nGYROZERO 完成，零偏(dps) x=");
                    PRINT_FIXED("", bias[0]);
                    PRINT_FIXED(" y", bias[1]);
                    PRINT_FIXED(" z", bias[2]);
                    if (s_cal_auto)
                    {
                        PRINT_FIXED("（采集极差 ", spread);
                        uart_dbg_puts(" dps，越小越静）");
                    }
                    uart_dbg_puts("\r\n");
                }
            }
            else
            {
                uart_dbg_puts("\r\nGYROZERO 失败：样本不足（别动它，重来一次）\r\n");
            }
            s_cal_active = 0u;
        }
    }
}

/* ---------------- 收字节 → 组行（带回显，方便手打） ---------------- */

void cmdline_init(void)
{
    s_len = 0u;
    g_hello_en = 1u;
}

void cmdline_poll(void)
{
    uint8_t ch;

    while (uart_dbg_read(&ch, 1u) > 0u) {
        if (ch == '\r' || ch == '\n') {
            uart_dbg_puts("\r\n");
            if (s_len > 0u) {
                s_line[s_len] = '\0';
                execute(s_line);
                s_len = 0u;
            }
        } else if (ch == '\b' || ch == 0x7Fu) {
            if (s_len > 0u) {
                s_len--;
                uart_dbg_puts("\b \b");
            }
        } else if (ch >= 0x20u && ch < 0x7Fu) {
            if (s_len < (LINE_MAX - 1u)) {
                s_line[s_len] = (char)ch;
                s_len++;
                uart_dbg_write(&ch, 1u);
            }
        }
    }
}
