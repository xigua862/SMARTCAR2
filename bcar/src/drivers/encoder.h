/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   双路编码器（TIM2 = 左、TIM3 = 右，硬件正交计数，不开中断）
  *
  * 接线（见 文档\引脚图-B车C8T6-20261002.html）：
  *   左编码器 A相 PA15 (TIM2_CH1)、B相 PB3 (TIM2_CH2)
  *   右编码器 A相 PB4  (TIM3_CH1)、B相 PB5 (TIM3_CH2)
  * ⚠ PA15/PB3/PB4 默认是 JTAG 脚 —— 必须先禁 JTAG 保 SWD（bsp_afio.c 已做），
  *   否则这几个脚根本不听 GPIO/定时器的话，表现是"计数永远是 0"。
  *
  * ★标定口径（v1.8 更正）：编码器是**齿轮驱动**（不是装在轮轴上），
  *   所以老的 mm/count = πD/4096 不成立。标定方法 = **推车 1 米，看累计计数**：
  *       mm_per_count = 1000 / 推 1m 得到的计数
  ******************************************************************************
  */
#ifndef __DRV_ENCODER_H
#define __DRV_ENCODER_H

#include <stdint.h>

typedef enum {
    ENC_L = 0,        /* TIM2：左 */
    ENC_R = 1,        /* TIM3：右 */
    ENC_COUNT = 2
} enc_id_t;

/**
  * @brief  初始化 TIM2/TIM3 为正交编码器模式（4 倍频：1024 线 → 每转 4096 计数）
  * @retval 1 = 成功
  */
uint8_t encoder_init(void);

/**
  * @brief  主循环 5ms 拍调用：读回 CNT，算出这一拍的增量并累加
  * @note   用"读 CNT 求差"而不是中断计数 —— 硬件正交模式下 CNT 自己会加减，
  *         中断反而没必要；且 5ms 内的增量换算成里程足够用。
  */
void encoder_poll(void);

/** @brief 上一拍的增量（原始计数，已带方向符号修正）*/
int16_t encoder_delta(enc_id_t id);

/** @brief 上电至今的累计计数（已带方向符号修正）*/
int32_t encoder_total(enc_id_t id);

/** @brief 把累计计数清零（标定起点用）*/
void encoder_reset(void);

#endif /* __DRV_ENCODER_H */
