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
#include "tim.h"
#include "gpio.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>   /* atof: 串口在线调参用 */

#define BASE_SPEED      40    /* 基础速度默认值(0~99)：2026-08-2x 实测最优=40 */
#define KP              8     /* 循迹比例系数默认值：实测最优=8（直线蛇形时调小） */
#define KD              30    /* 微分系数默认值(放大10倍存: 30 = 3.0)，压蛇形用，配合20ms周期 */
#define PULSES_PER_REV  1     /* 霍尔每圈脉冲数：每轮1块磁铁(3片叠成一块)，每圈1脉冲 */

/* 丢线旋转找线参数 */
#define LOST_SPIN_DELAY 2     /* 丢线后前2个控制周期沿原方向修正(应对瞬间丢线) */
#define LOST_SPIN_SPEED 35    /* 持续丢线时原地旋转找线的轮速 */
#define LOST_MAX_CYCLES 150   /* 旋转上限(约1.5s@10ms)，找不到就停车，防跑飞 */

/* 循迹模块触发电平: 2026-08-19 实测本模块为"白板触发"型——
   白板: 信号灯亮、DO输出低；黑线: 灯灭、DO输出高。
   即黑线输出【高】电平才算检测到线 → 用 GPIO_PIN_SET。
   （若换模块变成黑线输出低电平，改回 GPIO_PIN_RESET） */
#define LINE_ACTIVE_LEVEL GPIO_PIN_SET

volatile uint32_t hall_count1 = 0;   /* 霍尔1脉冲计数(EXTI中断里累加) */
volatile uint32_t hall_count2 = 0;   /* 霍尔2脉冲计数 */
uint32_t rpm1 = 0, rpm2 = 0;         /* 最近一次计算的转速(RPM) */

/* ---- 可在线修改的参数（串口发 P 12.5 / S 80 / D 30 直接改，不用重烧）---- */
int16_t base_speed = BASE_SPEED;     /* 基础速度(0~99) */
int16_t kp_x10 = KP * 10;            /* KP 放大10倍保存(支持小数)，如 12.5 -> 125 */
int16_t kd_x10 = KD * 10;            /* KD 放大10倍保存，如 3.0 -> 30 */

/* ---- 串口在线调参：寄存器级单字节接收 → 攒一行 → 主循环解析 ---- */
static          char     cmd_line[32];
static          uint8_t  cmd_len   = 0;
static volatile uint8_t  cmd_ready = 0;
static          uint8_t  test_mode = 0;   /* T命令单电机测试模式: 1=暂停循迹，只响应T */

static int8_t last_error = 0;
static int8_t last_e    = 0;        /* PD控制: 上一次误差，算微分用 */
static uint8_t lost_cycles = 0;     /* 连续丢线的控制周期计数 */
static uint8_t line_lost   = 0;     /* 1=持续丢线(需要原地旋转找线) */

void change1(int16_t SPEED);
void change2(int16_t SPEED);
static void process_command(void);

/* 读取5路循迹传感器，返回加权误差: 偏左为负，偏右为正，居中为0 */
static int8_t read_sensors(void)
{
  GPIO_TypeDef* ports[5]  = {IR_L_GPIO_Port, IR_LM_GPIO_Port, IR_C_GPIO_Port, IR_RM_GPIO_Port, IR_R_GPIO_Port};
  const uint16_t     pins[5]   = {IR_L_Pin, IR_LM_Pin, IR_C_Pin, IR_RM_Pin, IR_R_Pin};
  const int8_t       weights[5] = {-2, -1, 0, 1, 2};   /* 左->右加权 */
  int8_t error = 0;
  uint8_t active = 0;

  for (uint8_t i = 0; i < 5; i++)
  {
    if (HAL_GPIO_ReadPin(ports[i], pins[i]) == LINE_ACTIVE_LEVEL)
    {
      error += weights[i];
      active++;
    }
  }
  if (active == 0)
  {
    lost_cycles++;
    if (lost_cycles <= LOST_SPIN_DELAY)
    {
      line_lost = 0;
      error = last_error * 2;        /* 短暂丢线: 沿上次偏差方向加强修正(应对传感器瞬断) */
    }
    else
    {
      line_lost = 1;                 /* 持续丢线: 交给 line_follow 原地旋转找线 */
      error = 0;
    }
  }
  else
  {
    last_error = error;
    lost_cycles = 0;
    line_lost = 0;
  }
  return error;
}

/* 循迹差速控制: 每10ms调用一次 (PD控制 + 丢线旋转找线)
   2026-08-20 实测确认映射：电机1(change1/TIM2_CH3/A通道/AO)=【左】轮，
   电机2(change2/TIM2_CH4/B通道/BO)=【右】轮
   差速修正：线偏右(error>0)→corr>0→左轮加速、右轮减速→车右转追线
   PD控制: corr = P项 + D项；D项抑制直线蛇形(甩头)
   丢线旋转(2026-08-2x): 持续丢线时原地旋转找线，对付直角弯/锐角弯 */
static void line_follow(int16_t base_speed)
{
  int8_t e = read_sensors();

  /* 持续丢线: 原地旋转找线（先朝上次偏差方向转） */
  if (line_lost)
  {
    if (lost_cycles > LOST_MAX_CYCLES)
    {
      change1(0); change2(0);        /* 找线超时: 停车防跑飞 */
    }
    else if (last_error >= 0)
    {
      change1(LOST_SPIN_SPEED);      /* 线之前在右 → 右轮后退左轮前进 = 右转找线 */
      change2(-LOST_SPIN_SPEED);
    }
    else
    {
      change1(-LOST_SPIN_SPEED);     /* 线之前在左 → 左转找线 */
      change2(LOST_SPIN_SPEED);
    }
    return;
  }

  int16_t d_term = (int16_t)(e - last_e);          /* 误差变化率(微分) */
  int16_t corr = (kp_x10 * e) / 10 + (kd_x10 * d_term) / 10;  /* P项 + D项 */
  last_e = e;
  if (corr > 99)  corr = 99;
  if (corr < -99) corr = -99;
  int16_t m1   = base_speed + corr;  /* 电机1 = 左轮: 线偏右时加速 */
  int16_t m2   = base_speed - corr;  /* 电机2 = 右轮: 线偏右时减速 */
  if (m1 > 99)  m1 = 99;
  if (m1 < -99) m1 = -99;
  if (m2 > 99)  m2 = 99;
  if (m2 < -99) m2 = -99;
  change1(m1);
  change2(m2);
}

/* 测速+串口打印: 每500ms调用一次 */
static void speed_report(void)
{
  uint32_t c1 = hall_count1, c2 = hall_count2;
  hall_count1 = 0;
  hall_count2 = 0;
  /* 500ms内计数 * 120 = 每分钟脉冲数(60s/0.5s)，再除以每转脉冲数得RPM */
  rpm1 = c1 * 120 / PULSES_PER_REV;
  rpm2 = c2 * 120 / PULSES_PER_REV;

  char buf[96];
  int n = snprintf(buf, sizeof(buf),
    "IR:%d%d%d%d%d RPM1=%lu RPM2=%lu KP=%d.%d KD=%d.%d SP=%d\r\n",
    (HAL_GPIO_ReadPin(IR_L_GPIO_Port,  IR_L_Pin)  == LINE_ACTIVE_LEVEL),
    (HAL_GPIO_ReadPin(IR_LM_GPIO_Port, IR_LM_Pin) == LINE_ACTIVE_LEVEL),
    (HAL_GPIO_ReadPin(IR_C_GPIO_Port,  IR_C_Pin)  == LINE_ACTIVE_LEVEL),
    (HAL_GPIO_ReadPin(IR_RM_GPIO_Port, IR_RM_Pin) == LINE_ACTIVE_LEVEL),
    (HAL_GPIO_ReadPin(IR_R_GPIO_Port,  IR_R_Pin)  == LINE_ACTIVE_LEVEL),
    (unsigned long)rpm1, (unsigned long)rpm2,
    kp_x10 / 10, kp_x10 % 10, kd_x10 / 10, kd_x10 % 10, base_speed);
  if (n > 0)
  {
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)n, 100);
  }
}
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);

  HAL_NVIC_EnableIRQ(USART1_IRQn);                    /* 开串口接收中断(在线调参) */
  USART1->CR1 |= USART_CR1_RXNEIE;                    /* 直接使能RXNE中断(绕开HAL状态机，防卡死) */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t t_ctl = 0, t_spd = 0;   /* 控制周期计时 */
  while (1)
  {
		uint32_t now = HAL_GetTick();
		if (test_mode == 0)                                            /* T单电机测试时暂停循迹 */
		{
			if (now - t_ctl >= 20)  { t_ctl = now; line_follow(base_speed); }  /* 50Hz 循迹控制(20ms, 实测最优; 10ms会让D项噪声放大导致蛇形) */
		}
		if (now - t_spd >= 500) { t_spd = now; speed_report(); }           /* 每0.5s 测速+串口打印 */
		process_command();                                                /* 串口在线调参 */
		
		
		
		
		
		
		
		
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
void change1(int16_t SPEED)
{
	if(SPEED>0)
	{
		HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,SPEED);
	}else if(SPEED<0)
	{
		HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,-SPEED);
	}else
	{
		HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
	}
}

void change2(int16_t SPEED)
{
	if(SPEED>0)
	{
		HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_SET);   /* 2026-08-19: 电机2实测反转，方向电平对调 */
		HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,SPEED);
	}else if(SPEED<0)
	{
		HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,-SPEED);
	}else
	{
		HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);
	}
}






/* 霍尔脉冲中断回调: 上升沿触发(PA11/PA12)
   ⚠️ 必须用按位与(&)判断！EXTI15_10_IRQHandler 传进来的是合并掩码
   (GPIO_PIN_11|GPIO_PIN_12=0x1800)，用 == 永远不相等（2026-08-19 修复） */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin & GPIO_PIN_11)
  {
    hall_count1++;
  }
  if (GPIO_Pin & GPIO_PIN_12)
  {
    hall_count2++;
  }
}

/* ---- 串口在线调参 ---- */
/* 处理一条完整命令（主循环调用）: P <KP 0.5~50> / S <SPEED 0~99> */
static void process_command(void)
{
  if (!cmd_ready) { return; }
  cmd_ready = 0;
  char  c = (char)(cmd_line[0] & 0xDF);   /* 首字母转大写 */
  float v = atof(cmd_line + 1);           /* 兼容 "P 15" 和 "P15" */
  char buf[48];
  int  n = 0;
  if (c == 'P')
  {
    if (v >= 0.5f && v <= 50.0f) { kp_x10 = (int16_t)(v * 10.0f + 0.5f); }
    n = snprintf(buf, sizeof(buf), "OK KP=%d.%d\r\n", kp_x10 / 10, kp_x10 % 10);
  }
  else if (c == 'S')
  {
    test_mode = 0;                          /* S 命令同时退出单电机测试模式 */
    if (v >= 0.0f && v <= 99.0f) { base_speed = (int16_t)(v + 0.5f); }
    n = snprintf(buf, sizeof(buf), "OK SP=%d\r\n", base_speed);
  }
  else if (c == 'D')                        /* D <系数0~20>: 在线调微分项(压蛇形) */
  {
    if (v >= 0.0f && v <= 20.0f) { kd_x10 = (int16_t)(v * 10.0f + 0.5f); }
    n = snprintf(buf, sizeof(buf), "OK KD=%d.%d\r\n", kd_x10 / 10, kd_x10 % 10);
  }
  else if (c == 'H')                        /* H: 读 PA11/PA12 实时电平，查霍尔接线 */
  {
    n = snprintf(buf, sizeof(buf), "HALL1=%d HALL2=%d\r\n",
      (HAL_GPIO_ReadPin(HALL1_GPIO_Port, HALL1_Pin) != GPIO_PIN_RESET),
      (HAL_GPIO_ReadPin(HALL2_GPIO_Port, HALL2_Pin) != GPIO_PIN_RESET));
  }
  else if (c == 'T')                        /* T <1|2> <速度>: 单电机转向测试 */
  {
    int mot = 0, spd = 0;
    if (sscanf(cmd_line + 1, "%d %d", &mot, &spd) == 2)
    {
      if (spd > 99) { spd = 99; } else if (spd < -99) { spd = -99; }
      if (mot == 1) { change1((int16_t)spd); test_mode = 1; }
      else if (mot == 2) { change2((int16_t)spd); test_mode = 1; }
      else { change1(0); change2(0); test_mode = 0; }   /* T 0 0 = 停止测试 */
      n = snprintf(buf, sizeof(buf), "OK T%d=%d\r\n", mot, spd);
    }
    else
    {
      n = snprintf(buf, sizeof(buf), "USE: T <1|2> <speed 0~99>\r\n");
    }
  }
  else
  {
    n = snprintf(buf, sizeof(buf), "CMD: P <KP 0.5~50> / S <SPEED 0~99>\r\n");
  }
  if (n > 0) { HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)n, 100); }
}

/* USART1 中断入口: 寄存器级接收（直接读 SR/DR，绕开 HAL 状态机）
   即使发生溢出(ORE)也只清标志继续收，不会像 HAL Receive_IT 那样卡死 */
void USART1_IRQHandler(void)
{
  uint32_t sr = USART1->SR;
  if (sr & USART_SR_RXNE)                  /* 收到一字节 */
  {
    uint8_t b = (uint8_t)(USART1->DR & 0xFF);   /* 读 DR 自动清 RXNE */
    if (b == '\n' || b == '\r')            /* 回车/换行 = 一条命令结束 */
    {
      if (cmd_len > 0) 
{ 
cmd_line[cmd_len] = '\0'; cmd_len = 0; cmd_ready = 1; 
}
    }
    else if (cmd_len < (uint8_t)(sizeof(cmd_line) - 1))
    {
      cmd_line[cmd_len++] = (char)b;
    }
  }
  else if (sr & USART_SR_ORE)              /* 溢出: 读 DR 清标志，防止 RXNE 中断失效 */
  {
    (void)USART1->DR;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
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
