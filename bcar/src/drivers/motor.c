/**
  ******************************************************************************
  * @file    motor.c
  * @brief   TIM1 四路 PWM 实现（寄存器级）
  *
  * 为什么不用 CubeMX 的 MX_TIM1_Init：
  *   本工程把"外设初始化"分两半 —— CubeMX 管时钟/SWD/GPIO，
  *   电机这种"和硬件一对一"的部分写在驱动里，意图明面、且 CubeMX
  *   重新生成工程时不会把它改回去。参数只在 bcar_config.h 里。
  ******************************************************************************
  */
#include "motor.h"
#include "bcar_config.h"
#include "bsp_gpio.h"
#include "stm32f1xx.h"

/* 20kHz @ 72MHz：72 000 000 / 20 000 = 3600，所以 ARR = 3600-1（向上计数 0..ARR）*/
#define PWM_ARR   ((uint32_t)(SystemCoreClock / BCAR_PWM_FREQ_HZ) - 1u)

static motor_dir_t s_dir[MOTOR_SIDE_COUNT];
static uint16_t    s_mag[MOTOR_SIDE_COUNT];
static uint8_t     s_moe_ok = 0u;

/* 把 mag(0~1000) 换算成 CCR。含下限保护：低于 BCAR_MOTION_MIN_MAG 直接给 0 —— 
 * 因为低于 4.4% 占空比时 AT8236 因消隐根本不输出，"给一点点"只会让人误判"驱动坏了"。 */
static uint32_t mag_to_ccr(uint16_t mag)
{
    if (mag < BCAR_MOTION_MIN_MAG) {
        return 0u;
    }
    if (mag > BCAR_MAG_MAX) {
        mag = BCAR_MAG_MAX;
    }
    return ((uint32_t)mag * (PWM_ARR + 1u)) / BCAR_MAG_MAX;
}

static void gpio_af_setup(void)
{
    /* PA8/PA9/PA10/PA11 → 复用推挽 50MHz（PWM 要快边沿）*/
    bsp_gpio_set_af_pp(GPIOA, 8u,  BSP_GPIO_SPEED_50MHZ);
    bsp_gpio_set_af_pp(GPIOA, 9u,  BSP_GPIO_SPEED_50MHZ);
    bsp_gpio_set_af_pp(GPIOA, 10u, BSP_GPIO_SPEED_50MHZ);
    bsp_gpio_set_af_pp(GPIOA, 11u, BSP_GPIO_SPEED_50MHZ);
}

uint8_t motor_init(void)
{
    /* 1) 时钟：GPIOA（APB2）+ TIM1（APB2）*/
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;

    gpio_af_setup();

    /* 2) 时基：PSC=0（APB2 预分频=1，TIM1 时钟就是 72MHz）、向上计数、ARR 定 20kHz */
    TIM1->PSC  = 0u;
    TIM1->ARR  = PWM_ARR;
    TIM1->CR1  = TIM_CR1_ARPE;      /* 自动重装预装载：改 ARR 不会撕裂当前周期 */

    /* 3) 四路都配成 PWM 模式 1（计数值 < CCR 时输出有效电平），并开预装载 */
    TIM1->CCMR1 = (6u << 4)  | TIM_CCMR1_OC1PE    /* CH1: OC1M=110 */
                | (6u << 12) | TIM_CCMR1_OC2PE;   /* CH2: OC2M=110 */
    TIM1->CCMR2 = (6u << 4)  | TIM_CCMR2_OC3PE    /* CH3 */
                | (6u << 12) | TIM_CCMR2_OC4PE;   /* CH4 */

    /* 4) 先清零比较值，再使能输出（上电瞬间不能有意外占空比）*/
    TIM1->CCR1 = 0u; TIM1->CCR2 = 0u; TIM1->CCR3 = 0u; TIM1->CCR4 = 0u;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    /* 5) ★MOE：高级定时器的主输出总开关。不开这里，PWM 波形永远出不来 ——
     *    表现就是"程序在跑、电机不转"，是新手最常白耗半天的一个坑。*/
    TIM1->BDTR |= TIM_BDTR_MOE;

    /* 6) 启动计数器 */
    TIM1->CR1 |= TIM_CR1_CEN;

    s_dir[0] = s_dir[1] = MOTOR_BRAKE;
    s_mag[0] = s_mag[1] = 0u;

    /* 自检：把 MOE 位读回来确认真的置上了 */
    s_moe_ok = (TIM1->BDTR & TIM_BDTR_MOE) ? 1u : 0u;
    return s_moe_ok;
}

void motor_set(motor_side_t side, motor_dir_t dir, uint16_t mag)
{
    uint32_t duty;
    motor_dir_t d = dir;

    if ((uint8_t)side >= (uint8_t)MOTOR_SIDE_COUNT) {
        return;
    }

#if (BCAR_MOTOR_DIR_FLIP == 1)
    /* 整车方向翻一次：只翻"前进/后退"，刹车和滑行不受影响 */
    if (d == MOTOR_FWD)      d = MOTOR_REV;
    else if (d == MOTOR_REV) d = MOTOR_FWD;
#endif

    duty = mag_to_ccr(mag);

    /* 每侧两个通道：CH(2n+1) = IN1、CH(2n+2) = IN2 */
    if (side == MOTOR_L) {
        switch (d) {
        case MOTOR_FWD:   TIM1->CCR1 = duty; TIM1->CCR2 = 0u;    break;
        case MOTOR_REV:   TIM1->CCR1 = 0u;   TIM1->CCR2 = duty;  break;
        case MOTOR_BRAKE: TIM1->CCR1 = PWM_ARR + 1u; TIM1->CCR2 = PWM_ARR + 1u; break;
        case MOTOR_COAST: TIM1->CCR1 = 0u;   TIM1->CCR2 = 0u;    break;
        default: break;
        }
    } else {
        switch (d) {
        case MOTOR_FWD:   TIM1->CCR3 = duty; TIM1->CCR4 = 0u;    break;
        case MOTOR_REV:   TIM1->CCR3 = 0u;   TIM1->CCR4 = duty;  break;
        case MOTOR_BRAKE: TIM1->CCR3 = PWM_ARR + 1u; TIM1->CCR4 = PWM_ARR + 1u; break;
        case MOTOR_COAST: TIM1->CCR3 = 0u;   TIM1->CCR4 = 0u;    break;
        default: break;
        }
    }

    s_dir[side] = dir;   /* 记原始方向，方便回读时看到"你要求的是什么" */
    s_mag[side] = mag;
}

void motor_set_all(motor_dir_t dir, uint16_t mag)
{
    motor_set(MOTOR_L, dir, mag);
    motor_set(MOTOR_R, dir, mag);
}

void motor_brake_all(void)
{
    motor_set_all(MOTOR_BRAKE, 0u);
}

void motor_get_last(motor_side_t side, motor_dir_t *dir, uint16_t *mag)
{
    if ((uint8_t)side >= (uint8_t)MOTOR_SIDE_COUNT) {
        return;
    }
    if (dir != 0) { *dir = s_dir[side]; }
    if (mag != 0) { *mag = s_mag[side]; }
}
