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
#include "usart.h"
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
#include "adc_ir.h"
#include "encoder.h"
#include "imu660.h"
#include "line_pos.h"
#include "heading.h"
#include "odom.h"
#include "fsm.h"
#include "cmdline.h"
#include "telemetry.h"
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
  MX_USART1_UART_Init();
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
  key_init();
  cmdline_init();
  fsm_init();

  /* 上电横幅：把关键事实打出来自证（MAPR 是"重映射有没有生效"的判据）*/
  uart_dbg_printf("\r\n\r\n=== %s ===\r\n", BCAR_FW_VERSION);
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
    line_pos_cfg_t lc;
    uint8_t k;
    for (k = 0u; k < LINE_CH; k++) {
      lc.pos_mm[k] = ((float)k - 3.5f) * BCAR_IR_PITCH_MM;
      lc.black[k]  = 200.0f;    /* ★默认值只是占位，现场用 IRBLACK / IRWHITE 教一遍 */
      lc.white[k]  = 3000.0f;
    }
    lc.lost_sum_p = BCAR_IR_LOST_SUM_P;
    line_pos_init(&lc);
  }
  uart_dbg_printf("IR 8ch ADC+DMA ready (间距 x100 = %u, 黑白基准待现场标定)\r\n",
                  (unsigned)(BCAR_IR_PITCH_MM * 100.0f));

  /* 遥测：每圈结算 + 可选实时行（都走 TX DMA）*/
  telemetry_init();

  /* 进入待机：电机刹车，等发车键或串口命令 */
  (void)fsm_set(FSM_IDLE);

  /* 上电自检：响一声 + 三灯全亮一下，证明"人机通路"是活的 */
  buzzer_beep(120u);
  led_on(LED1); led_on(LED2); led_on(LED3);
  HAL_Delay(150);
  led_all_off();

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
        /* 编码器 → 里程：弧长 + 航向增量 → 位姿 */
        encoder_poll();
        odom_step(encoder_delta(ENC_L), encoder_delta(ENC_R), has_gyro, dth);
      }

      /* 灰度 → 线偏差（err/visible 都在 s_line_out 里，阶段 7 起给转向环用）*/
      s_line_out = line_pos_step(adc_ir_raw_all());

      /* 遥测：把这拍的数据喂进本圈统计（顺带按需发实时行）*/
      telemetry_tick(s_line_out.err_mm, s_line_out.visible,
                     odom_get().v_mm_s,
                     /* 本拍弧长用编码器差分直接算，避免再读一次 odom 内部量 */
                     (float)(encoder_delta(ENC_L) + encoder_delta(ENC_R)) * 0.5f
                       * BCAR_MM_PER_COUNT,
                     heading_yaw_deg());

      /* 跨多拍的命令（陀螺零偏标定）在这里推进 */
      cmdline_tick_5ms();

      /* --- 按键事件 --- */
      if (key_take_press(KEY_TRIG))          /* PB0：进测试模式 */
      {
        (void)fsm_set(FSM_TEST);
        uart_dbg_printf("\r\n[KEY] PB0 → MODE TEST（电机滑行可用）\r\n");
        buzzer_beep(60u);
      }
      if (key_take_press(KEY_START))         /* PC13：发车 */
      {
        /* 运动学还没写（阶段 3），先只切状态 + 报一声，别真跑 */
        (void)fsm_set(FSM_RUN);
        uart_dbg_printf("\r\n[KEY] PC13 → MODE RUN（注意：循迹还没接，车不会自己走）\r\n");
        buzzer_beep(200u);
      }
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
