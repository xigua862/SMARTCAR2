/**
  ******************************************************************************
  * @file    fsm.h
  * @brief   最小状态机（阶段 1~2 只需要"区分模式"，完整 9 态留到阶段 7）
  ******************************************************************************
  */
#ifndef __APP_FSM_H
#define __APP_FSM_H

#include <stdint.h>

typedef enum {
    FSM_BOOT = 0,   /* 上电初始化中 */
    FSM_IDLE,       /* 待机：电机刹车，等发车键 */
    FSM_TEST,       /* 测试模式：电机允许滑行/单独点动，等串口指令 */
    FSM_RUN,        /* 运行中：循迹跑（阶段 3 以后才真正跑起来）*/
    FSM_ESTOP,      /* 急停 */
    FSM_STATE_COUNT
} fsm_state_t;

void fsm_init(void);

fsm_state_t fsm_state(void);

const char *fsm_state_name(fsm_state_t s);

/**
  * @brief  切状态
  * @retval 1 = 成功；0 = 拒绝（非法迁移或条件不满足）
  */
uint8_t fsm_set(fsm_state_t s);

/**
  * @brief  当前是否允许使用"滑行（COAST）"
  * @note   按框架硬约束：COAST 会让 AT8236 进休眠，**只允许在测试/待机下用**。
  *         运行中"停"一律用 BRAKE。
  */
uint8_t fsm_coast_allowed(void);

/** @brief 当前状态是什么时候进来的（毫秒时间戳）—— 超时判据要用 */
uint32_t fsm_state_enter_ms(void);

#endif /* __APP_FSM_H */
