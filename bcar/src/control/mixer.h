/**
  ******************************************************************************
  * @file    mixer.h
  * @brief   差速混合（**纯逻辑，无硬件依赖，可在 PC 上单测**）
  *
  *   左轮 = base + steer
  *   右轮 = base − steer
  * 符号约定：steer > 0 表示"该往右转" ⇒ 左轮快、右轮慢 ✓
  *
  * 三个必须处理的情况：
  *   ① 结果超过 ±1000 → 限幅，同时把另一侧按同样比例缩，**保住差速比例**
  *      （只砍一侧会让车突然变直，是"进弯甩出去"的常见原因）
  *   ② 结果低于 AT8236 的消隐下限 → 直接给 0，避免"给了一点点却不转"的误导
  *   ③ 结果为负 → 该轮反转（坦克转向），不是简单夹到 0
  ******************************************************************************
  */
#ifndef __CTRL_MIXER_H
#define __CTRL_MIXER_H

#include <stdint.h>

typedef enum {
    MIX_DIR_FWD = 0,
    MIX_DIR_REV = 1
} mix_dir_t;

typedef struct {
    uint16_t mag;    /* 0~1000 */
    mix_dir_t dir;   /* 该轮是正转还是反转 */
} mix_wheel_t;

typedef struct {
    mix_wheel_t l;
    mix_wheel_t r;
} mix_out_t;

/**
  * @brief  混合：base（共同速度）+ steer（转向量）→ 左右轮
  * @param  base   共同速度分量（0~1000），通常由速度环给；先用手动设定
  * @param  steer  转向量（可正可负）
  * @param  min_mag 电机消隐下限（低于它的非零值一律当 0），传 50 即可
  * @return 左右轮的方向与力度
  */
mix_out_t mixer_step(float base, float steer, uint16_t min_mag);

#endif /* __CTRL_MIXER_H */
