/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bcar_config.h"
#include "bsp_afio.h"
#include "bsp_gpio.h"
#include "bsp_systick.h"
#include "uart_dbg.h"
#include "led.h"
#include "buzzer.h"
#include "key.h"
#include "motor.h"
#include "fan.h"
#include "adc_ir.h"
#include "encoder.h"
#include "imu660.h"
#include "line_pos.h"
#include "heading.h"
#include "odom.h"
#include "fsm.h"
#include "launch.h"
#include "zebra.h"
#include "cmdline.h"
#include "telemetry.h"
#include "control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 阶段 1 的软定时器与计数器（放文件作用域，别塞进 while 里当局部变量）*/
static uint32_t s_t_led   = 0u;   /* 500ms 走马灯 */
static uint32_t s_t_print = 0u;   /* 1000ms 串口心跳 */
static uint32_t s_hello   = 0u;   /* 心跳序号，用来验证"每秒一条不丢" */
static uint8_t  s_chase   = 0u;   /* 走马灯位置 0/1/2=亮某只，3=全灭 */
/* ★上一拍的状态机状态：用来抓"进/出 RUN"的边沿。
 * 控制环的接管(control_enter_run)/交还(control_exit_run)必须挂在状态边沿上，
 * 否则 s_running 永远是 0，control_step() 直接 return —— 看着像在闭环，其实电机收不到任何指令。*/
static fsm_state_t s_fsm_prev = FSM_BOOT;

/* 发车读秒（PC13 长按 → 3 秒 / 短短长 3 声 → RUN）。
 * 时序逻辑在 app/launch.c（纯逻辑、PC 可单测），这里只管"事件 → 动作"。 */
static launch_t s_launch;

/* 斑马线（S/G）判定与"过 S / 到 G 停车"的计数（zebra.c 是纯逻辑，PC 可单测）*/
static zebra_t s_zebra;
static uint8_t s_mark_count = 0u;    /* 已经过了几条斑马线：0=还没，1=过 S，2=过 G 完事 */
/* 本拍弧长（毫米）—— 单编码器模式只要一个轮子（见 bcar_config.h 的说明）*/
static float   s_ds_mm = 0.0f;

/* 最近一次的灰度结果（8 路归一化值 + 偏差）。
 * 放全局是为了让 cmdline 能直接打印 —— 阶段 6 起转向环也读它。 */
line_pos_t s_line_out;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* ⚠ 这里**不要**加 MX_USART1_UART_Init()：
   * 它会把 PA9/PA10 配成串口，而这两块脚在 B 车上是**电机 PWM**（TIM1_CH2/CH3）。
   * 本板串口走的是重映射后的 PB6/PB7，初始化在 uart_dbg_init() 里手写（见下）。*/
  /* USER CODE BEGIN 2 */
  /* 初始化顺序有讲究，别调换：
   *   1) AFIO  —— 先关 JTAG 保 SWD、把 USART1 重映射到 PB6/PB7
   *   2) 时基  —— DWT 微秒计时（后面 IMU 积分要用）
   *   3) 串口  —— 必须晚于 AFIO，否则重映射没生效，串口是哑巴
   *   4) 其它外设（灯/蜂鸣器/按键/电机）*/
  bsp_afio_init();
  bsp_systick_init();
  uart_dbg_init();
  led_init();
  buzzer_init();
  fan_init();          /* 风机（负压）：上电先关着，发车读秒才开 */
  /* 风机自检：上电转 350ms —— 这是"这条路通不通"的唯一可靠判据
   * （串口命令时通时断，靠敲 FAN 测不准）。听得到/看得到就说明 U6 + 风机是好的。 */
  fan_launch_on();
  HAL_Delay(350);
  fan_off();  key_init();
  cmdline_init();
  fsm_init();

  /* 上电横幅：把关键事实打出来自证（MAPR 是"重映射有没有生效"的判据；
   * built 后面是**编译时间戳**，跟 hex 的文件时间一比就知道板上是不是最新那版）*/
  uart_dbg_printf("\r\n\r\n=== %s === built %s\r\n", BCAR_FW_VERSION, BCAR_FW_BUILD);
  uart_dbg_printf("MAPR = 0x%08X  (%s)\r\n",
                  bsp_afio_mapr(), bsp_afio_mapr_explain(bsp_afio_mapr()));
  uart_dbg_printf("SYSCLK = %u Hz, PCLK2 = %u Hz, tick = %u ms\r\n",
                  SystemCoreClock, HAL_RCC_GetPCLK2Freq(), BCAR_TICK_MS);

  /* 电机 PWM：MOE 是"程序在跑、电机不转"的经典坑，所以把结果打出来 */
  if (motor_init())
  {
    uart_dbg_printf("TIM1 PWM %u Hz, MOE = ON\r\n", BCAR_PWM_FREQ_HZ);
  }
  else
  {
    uart_dbg_printf("TIM1 MOE = OFF  ← 电机不会转！查 BDTR 写入\r\n");
    led_on(LED3);
  }

  /* 编码器（TIM2/TIM3 硬件正交）+ 里程解算 */
  (void)encoder_init();
  {
    odom_cfg_t oc;
    oc.mm_per_count = BCAR_MM_PER_COUNT;   /* ★待标定 */
    oc.track_mm     = BCAR_TRACK_MM;
    oc.dt_s         = (float)BCAR_TICK_MS / 1000.0f;
    odom_init(&oc);
  }
  uart_dbg_printf("ENC TIM2/TIM3 quadrature OK (mm_per_count x1000 = %u, 待标定)\r\n",
                  (unsigned)(BCAR_MM_PER_COUNT * 1000.0f));

  /* 灰度 8 路：ADC 扫描 + DMA 一开就一直在后台滚，CPU 不用管 */
  adc_ir_init();
  {
    /* 逐路黑白基准：2026-10-15 在**真赛道材料**上实测（见 bcar_config.h 里的说明）。
     * 不再是"猜的 200/3000"，所以上电就是标定好的状态 —— 现场不用每次教。 */
    static const float k_ir_black[LINE_CH] = BCAR_IR_BLACK_ARR;
    static const float k_ir_white[LINE_CH] = BCAR_IR_WHITE_ARR;
    line_pos_cfg_t lc;
    uint8_t k;
    /* 数组长度和通道数对不上就编译不过（换模块/换路数时防呆）*/
    typedef char ir_ref_len_chk[(sizeof(k_ir_black) / sizeof(k_ir_black[0]) == (size_t)LINE_CH) ? 1 : -1];

    for (k = 0u; k < LINE_CH; k++) {
      lc.pos_mm[k] = ((float)k - 3.5f) * BCAR_IR_PITCH_MM;
      lc.black[k]  = k_ir_black[k];
      lc.white[k]  = k_ir_white[k];
    }
    lc.lost_sum_p = BCAR_IR_LOST_SUM_P;
    line_pos_init(&lc);
  }
  uart_dbg_printf("IR 8ch ADC+DMA ready (间距 x100 = %u, 黑白基准 = 实测值)\r\n",
                  (unsigned)(BCAR_IR_PITCH_MM * 100.0f));

  /* 遥测：每圈结算 + 可选实时行（都走 TX DMA）*/
  telemetry_init();

  /* 循迹控制环（转向 PD + 差速混合）：初始化但还没启动，等进 RUN 才动 */
  control_init();

  /* ★IMU（SPI2 + DRDY 外部中断）
   * 2026-10-15 补：之前**漏了 imu660_init() 这一句** ⇒ SPI2 从没被使能（SPE=0），
   * 一敲 IMU 命令就死在 spi2_xfer 里等 RXNE（整机假死、连 hello 心跳都停）。
   * 上电自证：读得到 WHO_AM_I 才算过；读不到要吼一声，别静默带病跑。 */
  if (imu660_init())
  {
    uart_dbg_printf("IMU660RC OK (WHO_AM_I 已读到)\r\n");
  }
  else
  {
    uart_dbg_printf("ERR IMU660RC 无应答 —— 查 SPI2 接线 / CS / 模块 3.3V\r\n");
    buzzer_beep(300u);
  }
  heading_init(BCAR_GYRO_DEAD_ZONE_DPS);

  /* 进入待机：电机刹车，等发车键或串口命令 */
  (void)fsm_set(FSM_IDLE);

  /* 上电自检：响一声 + 三灯全亮一下，证明"人机通路"是活的。
   * ★这"1 声"就是发车流程的第一步：上电 1 声 = 待机中。 */
  buzzer_beep(120u);
  led_on(LED1); led_on(LED2); led_on(LED3);
  HAL_Delay(150);
  led_all_off();

  /* 发车流程（2026-10-15 用户定的口径，en 手边看一眼就知道怎么发车）*/
  uart_dbg_puts("\r\n[待机] 上电 1 声 = 已待机。发车流程：\r\n");
  uart_dbg_puts("       PB0 → 测试模式（蜂鸣 2 声，之后等串口指令，调参器在这个模式跑）\r\n");
  uart_dbg_printf("       PC13 长按 %u ms → 发车：3 秒读秒（短短长 3 声）→ 循迹\r\n",
                  (unsigned)BCAR_KEY_LONG_MS);
  uart_dbg_puts("stage2 ready: LED chaser 500ms, hello 1/s, 打 HELP 看命令\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 5ms 控制节拍：阶段 2 起这里挂速度环/角度环；现在先挂"慢活"
     * （按键消抖、蜂鸣器计时）—— 它们不需要进中断。 */
    if (bsp_tick5ms_take())
    {
      key_poll();
      buzzer_poll();

      /* 陀螺 → 航向：有 DRDY 才读（采样时刻由芯片给，抖动小）。
       * 拿不到新数据就退回"差速推算"，所以 IMU 挂了车还能勉强跑。 */
      {
        float    dth = 0.0f;
        uint8_t  has_gyro = 0u;
        if (imu660_drdy_take())
        {
          imu660_read();
          dth = heading_step(imu660_gyro_dps(IMU_AXIS_Z),
                             (float)BCAR_TICK_MS / 1000.0f);
          has_gyro = 1u;
        }
        /* 编码器 → 里程：弧长 + 航向增量 → 位姿。
         * ⚠ 右编码器读不到数（硬件待查）⇒ 单编码器模式下直接把左轮增量当整车位移，
         *   别喂 0（喂 0 等于把里程砍半，"跑完一圈停车"的里程兜底就永不触发）。 */
        encoder_poll();
#if (BCAR_ODOM_SINGLE_ENC)
        {
          int16_t d = encoder_delta(ENC_L);
          s_ds_mm = (float)d * BCAR_MM_PER_COUNT;
          odom_step(d, d, has_gyro, dth);
        }
#else
        {
          int16_t dl = encoder_delta(ENC_L);
          int16_t dr = encoder_delta(ENC_R);
          s_ds_mm = (float)(dl + dr) * 0.5f * BCAR_MM_PER_COUNT;
          odom_step(dl, dr, has_gyro, dth);
        }
#endif
      }

      /* 灰度 → 线偏差（err/visible 都在 s_line_out 里，阶段 7 起给转向环用）*/
      s_line_out = line_pos_step(adc_ir_raw_all());

      /* ★过 S / 到 G：斑马线判定（用户口径："过 S 触发一次、跑完一圈到 G 再触发 → 停车"）。
       * 判据全部来自实测（见 zebra.h）：强白跨度 ≥55mm 且 sum_p ≥4.5 —— 普通白线（11mm/3.3）够不着。
       * ★这里**每拍都跑、不分状态**：这样台上把车往斑马线上一放，串口就会打 [MARK]，好验证。 */
      if (zebra_step(&s_zebra, &s_line_out))
      {
        float s_now = odom_get().s_mm;
        if (fsm_state() != FSM_RUN)
        {
          uart_dbg_printf("\r\n[MARK] 斑马线（当前 %s，不计入）s=%d mm\r\n",
                          fsm_state_name(fsm_state()), (int)s_now);
        }
        else if (s_mark_count == 0u && s_now < BCAR_MARK_S_WINDOW_MM)
        {
          s_mark_count = 1u;
          uart_dbg_printf("\r\n[MARK] 过 S（第 1 条斑马线）s=%d mm\r\n", (int)s_now);
        }
        else if (s_now > BCAR_FINISH_MIN_MM)
        {
          s_mark_count = 2u;
          uart_dbg_puts("\r\n[FINISH] 过 G（第 2 条斑马线）→ 停车\r\n");
          buzzer_beeps(3u, (uint16_t)BCAR_BEEP_SHORT_MS, (uint16_t)BCAR_BEEP_GAP_MS);
          (void)fsm_set(FSM_IDLE);       /* 离开 RUN 会自动刹车（见边沿处理）*/
        }
        else
        {
          uart_dbg_printf("\r\n[MARK] 斑马线（s=%d mm 不在判定窗口，按误判忽略）\r\n", (int)s_now);
        }
      }

      /* 里程兜底：斑马线万一漏认（白条只有 18mm，位置不巧就可能漏），跑够一圈也停 */
      if (fsm_state() == FSM_RUN && s_mark_count == 1u
          && odom_get().s_mm > BCAR_FINISH_ODOM_MM)
      {
        s_mark_count = 2u;
        uart_dbg_printf("\r\n[FINISH] 里程到一圈（s=%d mm，斑马线没认出来）→ 停车\r\n",
                        (int)odom_get().s_mm);
        buzzer_beeps(3u, (uint16_t)BCAR_BEEP_SHORT_MS, (uint16_t)BCAR_BEEP_GAP_MS);
        (void)fsm_set(FSM_IDLE);
      }

      /* --- 按键事件（先处理；后面的状态边沿同拍就能看到新状态）--- */

      /* PB0（KEY1）→ 测试模式：蜂鸣 2 声，之后等串口指令（调参器在这里跑）。
       * ★读秒中按它 = 取消发车（下面边沿块会把读秒掐掉）；RUN 中按它 = 停车交还。 */
      if (key_take_press(KEY_TRIG))
      {
        (void)fsm_set(FSM_TEST);
        buzzer_beeps(2u, (uint16_t)BCAR_BEEP_SHORT_MS, (uint16_t)BCAR_BEEP_GAP_MS);
        uart_dbg_puts("\r\n[KEY] PB0 → 测试模式（蜂鸣 2 声）：等串口指令\r\n");
      }

      /* PC13 短按 → 不发车（用户口径：短按不响应）。提示一句，免得以为板子死了 */
      if (key_take_short(KEY_START))
      {
        uart_dbg_printf("\r\n[KEY] PC13 短按忽略（发车要长按 %u ms）\r\n",
                        (unsigned)BCAR_KEY_LONG_MS);
      }

      /* PC13（KEY2）长按 → 正式发车：进读秒 */
      if (key_take_long(KEY_START))
      {
        fsm_state_t st = fsm_state();
        if (st == FSM_IDLE || st == FSM_TEST)
        {
          (void)fsm_set(FSM_COUNTDOWN);
        }
        else
        {
          uart_dbg_printf("\r\n[KEY] PC13 长按：当前是 %s，不发车（要停在待机/测试模式才能发车）\r\n",
                          fsm_state_name(st));
        }
      }

      /* --- 发车读秒：3 秒内短短长 3 声，到点自动起跑 --- */
      if (fsm_state() == FSM_COUNTDOWN)
      {
        launch_ev_t lev = launch_step(&s_launch, bsp_millis());
        if (lev == LAUNCH_EV_BEEP_SHORT)
        {
          buzzer_beep(BCAR_BEEP_SHORT_MS);
        }
        else if (lev == LAUNCH_EV_BEEP_LONG)
        {
          buzzer_beep(BCAR_BEEP_LONG_MS);
        }
        else if (lev == LAUNCH_EV_DONE)
        {
          (void)fsm_set(FSM_RUN);
          uart_dbg_puts("[LAUNCH] 读秒结束 → RUN：循迹开始\r\n");
        }
      }

      /* ★循迹控制：挂在状态边沿上接管/交还电机，且只在 RUN 里真正驱动。
       * ★2026-10-15 补：原来只调了 control_step()，而 control_enter_run()/exit_run()
       * 全项目没人调用 ⇒ s_running 恒为 0 ⇒ 这个"闭环"其实是死的（按发车键车不动）。
       * 进 RUN → 接管（清微分历史、清丢线计数）；离开 RUN → 交还（强制刹车）。 */
      {
        fsm_state_t st = fsm_state();
        if (st != s_fsm_prev)
        {
          /* 读秒没走完就切走了（多半是按 PB0 取消）⇒ 把时序器掐掉，别留着半截读秒 */
          if (s_fsm_prev == FSM_COUNTDOWN && st != FSM_COUNTDOWN)
          {
            if (launch_active(&s_launch))
            {
              launch_abort(&s_launch);
              uart_dbg_puts("[LAUNCH] 读秒被取消，没起跑\r\n");
            }
          }

          if (st == FSM_COUNTDOWN)
          {
            /* ★风机先转起来（要几百毫秒才能上劲），这样起步那一脚压得住车头 */
            fan_launch_on();
            /* 读秒这 3 秒：车摆在发车点（赛道图上那个"小蓝框" = nav_map 的 s=0，车头朝 S），
             * 电机已刹车、纹丝不动 —— 正好拿来标陀螺零偏。 */
            launch_begin(&s_launch, bsp_millis());
#if (BCAR_GYRO_AUTOCAL_ON_LAUNCH)
            /* 不标的话实测有 ≈0.4 dps 的原始零偏，跑一圈航向能飘十几度，而且不报错 */
            cmdline_gyrozero_auto_begin();
#endif
            uart_dbg_puts("[LAUNCH] 开始读秒（车别动：正在标陀螺零偏）\r\n");
          }

          if (st == FSM_RUN)
          {
            /* ★发车锚定：把里程/位姿/航向清到发车点这一点 ⇒ 之后 odom 的 s 就是
             * "从发车点跑出多远"、航向 0 = 发车方向（车头朝 S）。
             * ★放在"进 RUN 这一刻"而不是读秒开始，是为了让零偏标定先落地 ——
             *   否则读秒头 1 秒还在用旧零偏，会把航向带偏零点几度。
             * ⚠ 地图坐标 (2772, 203) 与航向 180° 的换算等地图进固件时统一做
             *   （含"地图 yaw 顺时针为正"与陀螺逆时针为正的符号），见交接文档待办。 */
            heading_reset();
            encoder_reset();
            odom_reset();
            telemetry_start_lap();   /* 本圈统计从起跑算，别把站着读秒的 3 秒算进去 */
            /* 起步模式 = 陀螺直行（目标航向 = 0 = 发车方向），看到线自动交回循迹环 */
            zebra_init(&s_zebra);
            s_mark_count = 0u;
            control_enter_run(heading_yaw_deg());
            uart_dbg_printf("[CTRL] 进入 RUN：先按陀螺直行（目标 %d deg），看到线交给循迹环；BASE=%u\r\n",
                            (int)heading_yaw_deg(), (unsigned)control_base());
          }
          else if (s_fsm_prev == FSM_RUN)
          {
            fan_off();                     /* 离开 RUN 就关风机（2.5A，别常开）*/
            control_exit_run();
            uart_dbg_puts("[CTRL] 离开 RUN：转向环交还，电机已刹车\r\n");
          }
          s_fsm_prev = st;
        }
        if (st == FSM_RUN)
        {
          /* 循迹看 err/visible，看不到线就切陀螺直行 —— 两种模式在 control.c 里切 */
          control_step(s_line_out.err_mm, s_line_out.visible, heading_yaw_deg());

          /* ★过 S / 到 G：斑马线判定（用户口径："过 S 触发一次、跑完一圈到 G 再触发 → 停车"）。
           * 判据是实测出来的：斑马线上 7 路白、白区跨度 77mm；普通白线最多 4 路、≤33mm。 */
          if (zebra_step(&s_zebra, &s_line_out))
          {
            float s_now = odom_get().s_mm;
            if (s_mark_count == 0u && s_now < BCAR_MARK_S_WINDOW_MM)
            {
              s_mark_count = 1u;
              uart_dbg_printf("\r\n[MARK] 过 S（第 1 条斑马线）s=%d mm\r\n", (int)s_now);
            }
            else if (s_now > BCAR_FINISH_MIN_MM)
            {
              s_mark_count = 2u;
              uart_dbg_puts("\r\n[FINISH] 过 G（第 2 条斑马线）→ 停车\r\n");
              buzzer_beeps(3u, (uint16_t)BCAR_BEEP_SHORT_MS, (uint16_t)BCAR_BEEP_GAP_MS);
              (void)fsm_set(FSM_IDLE);       /* 离开 RUN 会自动刹车（见上面的边沿处理）*/
            }
            else
            {
              uart_dbg_printf("\r\n[MARK] 斑马线（s=%d mm 不在判定窗口，按误判忽略）\r\n", (int)s_now);
            }
          }
          else if (s_mark_count == 1u && odom_get().s_mm > BCAR_FINISH_ODOM_MM)
          {
            /* 兜底：斑马线万一没认出来（白条只有 18mm，位置不巧就可能漏），
             * 里程到一圈也停车 —— 宁可停早点，也别一直冲。 */
            s_mark_count = 2u;
            uart_dbg_printf("\r\n[FINISH] 里程到一圈（s=%d mm，斑马线没认出来）→ 停车\r\n",
                            (int)odom_get().s_mm);
            buzzer_beeps(3u, (uint16_t)BCAR_BEEP_SHORT_MS, (uint16_t)BCAR_BEEP_GAP_MS);
            (void)fsm_set(FSM_IDLE);
          }
        }
      }

      /* 遥测：把这拍的数据喂进本圈统计（顺带按需发实时行）*/
      telemetry_tick(s_line_out.err_mm, s_line_out.visible,
                     odom_get().v_mm_s,
                     s_ds_mm,                 /* 本拍弧长（上面算好的）*/
                     heading_yaw_deg());

      /* 跨多拍的命令（陀螺零偏标定）在这里推进 */
      cmdline_tick_5ms();
    }

    /* 串口命令（行式，收到换行才执行）—— 每圈都查，别等 5ms 拍 */
    cmdline_poll();

    /* 500ms：三色灯走马（亮一只 → 亮下一只 → ... → 全灭 → 循环） */
    if (bsp_timer_expired(&s_t_led, 500u))
    {
      led_all_off();
      if (s_chase < (uint8_t)LED_COUNT)
      {
        led_on((led_id_t)s_chase);
      }
      s_chase = (uint8_t)((s_chase + 1u) % ((uint8_t)LED_COUNT + 1u));
    }

    /* 1000ms：串口心跳。序号必须连续递增 —— 丢一条就说明发送侧有问题。
     * 调参时用命令 `HELLO 0` 关掉，免得刷屏。 */
    if (g_hello_en && bsp_timer_expired(&s_t_print, 1000u))
    {
      s_hello++;
      uart_dbg_printf("hello %u  t=%u ms  state=%s\r\n",
                      s_hello, bsp_millis(), fsm_state_name(fsm_state()));
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 走到这里说明 HAL 初始化失败（最常见：晶振没起振 / 时钟配置不受支持）。
   * 场上没有调试器时，用 LED3 常亮 + 串口一行字告诉你"我挂在哪一步"。 */
  __disable_irq();
  led_on(LED3);
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
