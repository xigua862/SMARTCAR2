/**
  ******************************************************************************
  * @file    fsm.c
  * @brief   最小状态机实现
  ******************************************************************************
  */
#include "fsm.h"
#include "bcar_config.h"
#include "motor.h"
#include "bsp_systick.h"

static fsm_state_t s_state = FSM_BOOT;
static uint32_t    s_enter_ms = 0u;

void fsm_init(void)
{
    s_state    = FSM_BOOT;
    s_enter_ms = bsp_millis();
}

fsm_state_t fsm_state(void)
{
    return s_state;
}

const char *fsm_state_name(fsm_state_t s)
{
    switch (s) {
    case FSM_BOOT:  return "BOOT";
    case FSM_IDLE:  return "IDLE";
    case FSM_TEST:  return "TEST";
    case FSM_RUN:   return "RUN";
    case FSM_ESTOP: return "ESTOP";
    default:        return "?";
    }
}

uint8_t fsm_set(fsm_state_t s)
{
    if ((uint8_t)s >= (uint8_t)FSM_STATE_COUNT) {
        return 0u;
    }

    /* 进 TEST 前先把电机刹住：避免带着上一个状态的速度进测试模式 */
    if (s == FSM_TEST || s == FSM_IDLE) {
        motor_brake_all();
    }
    /* 任何状态切出去都先刹车（RUN 里由控制环接管，届时改这里）*/
    if (s_state == FSM_RUN && s != FSM_RUN) {
        motor_brake_all();
    }

    s_state    = s;
    s_enter_ms = bsp_millis();
    return 1u;
}

uint8_t fsm_coast_allowed(void)
{
    return (s_state == FSM_TEST || s_state == FSM_IDLE || s_state == FSM_BOOT) ? 1u : 0u;
}

uint32_t fsm_state_enter_ms(void)
{
    return s_enter_ms;
}
