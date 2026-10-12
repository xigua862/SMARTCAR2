/**
  ******************************************************************************
  * @file    cmdline.c
  * @brief   串口命令行实现（解析逻辑在 str_util.c，本文件只做"解析→动作"的映射）
  ******************************************************************************
  */
#include "cmdline.h"
#include "bcar_config.h"
#include "bsp_afio.h"
#include "bsp_systick.h"
#include "buzzer.h"
#include "fsm.h"
#include "key.h"
#include "led.h"
#include "motor.h"
#include "encoder.h"
#include "odom.h"
#include "str_util.h"
#include "uart_dbg.h"

volatile uint8_t g_hello_en = 1u;

#define LINE_MAX 64u

static char    s_line[LINE_MAX];
static uint8_t s_len = 0u;

/* ---------------- 帮助 ---------------- */

static void print_help(void)
{
    uart_dbg_puts("\r\n命令（大小写不敏感，参数用空格分隔）:\r\n");
    uart_dbg_puts("  HELP                  这页帮助\r\n");
    uart_dbg_puts("  PING                  应答 PONG（测链路）\r\n");
    uart_dbg_puts("  STATUS                当前状态/节拍/电机/按键/MOE\r\n");
    uart_dbg_puts("  MAPR                  打印 AFIO->MAPR（查重映射是否生效）\r\n");
    uart_dbg_puts("  HELLO <0|1>           每秒心跳打印开关\r\n");
    uart_dbg_puts("  BEEP [ms]             蜂鸣（默认 100ms）\r\n");
    uart_dbg_puts("  MODE <IDLE|TEST|RUN>  切模式\r\n");
    uart_dbg_puts("  MOTOR <dir> <mag>     两侧同设；dir=FWD|REV|BRAKE|COAST，mag=0~1000\r\n");
    uart_dbg_puts("  MOTOR L|R <dir> <mag> 只设一侧（L=PA8/PA9，R=PA10/PA11）\r\n");
    uart_dbg_puts("  KEY                   打印两个按键当前电平\r\n");
    uart_dbg_puts("  ENC                   编码器累计计数/本拍增量（★标定用）\r\n");
    uart_dbg_puts("  ODOM                  里程与位姿（x/y/yaw/s/v）\r\n");
    uart_dbg_puts("  ZERO                  编码器与位姿清零（标定起点）\r\n");
    uart_dbg_puts("  LOOPBACK [ms]         TX-RX 短接时的回环自检（默认 200ms）\r\n");
}

static void cmd_status(void)
{
    motor_dir_t dl = MOTOR_BRAKE, dr = MOTOR_BRAKE;
    uint16_t    ml = 0u, mr = 0u;

    motor_get_last(MOTOR_L, &dl, &ml);
    motor_get_last(MOTOR_R, &dr, &mr);

    uart_dbg_printf("STATE = %s  (进入于 %u ms)\r\n",
                    fsm_state_name(fsm_state()), bsp_millis() - fsm_state_enter_ms());
    uart_dbg_printf("t = %u ms, tick = %u ms\r\n", bsp_millis(), BCAR_TICK_MS);
    uart_dbg_printf("MOTOR L: dir=%u mag=%u | R: dir=%u mag=%u  (dir: 0=FWD 1=REV 2=BRAKE 3=COAST)\r\n",
                    (unsigned)dl, (unsigned)ml, (unsigned)dr, (unsigned)mr);
    uart_dbg_printf("KEY  START(PC13)=%u  TRIG(PB0)=%u  (1=按住)\r\n",
                    (unsigned)key_is_down(KEY_START), (unsigned)key_is_down(KEY_TRIG));
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
        if (su_streq(a1, "IDLE"))       { s = FSM_IDLE; }
        else if (su_streq(a1, "TEST"))  { s = FSM_TEST; }
        else if (su_streq(a1, "RUN"))   { s = FSM_RUN; }
        else if (su_streq(a1, "ESTOP")) { s = FSM_ESTOP; }
        else {
            uart_dbg_puts("ERR 模式只能是 IDLE/TEST/RUN/ESTOP\r\n");
            return;
        }
        (void)fsm_set(s);
        uart_dbg_printf("OK MODE %s（电机已刹车）\r\n", fsm_state_name(fsm_state()));
    } else if (su_streq(w, "MOTOR") || su_streq(w, "M")) {
        cmd_motor(line);
    } else if (su_streq(w, "KEY")) {
        uart_dbg_printf("KEY START(PC13)=%u TRIG(PB0)=%u (1=按住)\r\n",
                        (unsigned)key_is_down(KEY_START), (unsigned)key_is_down(KEY_TRIG));
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
