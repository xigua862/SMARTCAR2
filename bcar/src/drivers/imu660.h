/**
  ******************************************************************************
  * @file    imu660.h
  * @brief   逐飞 IMU660RC 陀螺仪/加速度计驱动（SPI2 + DRDY 中断）
  *
  * 芯片：LSM6DSV16X 系列（WHO_AM_I = 0x70）
  * 接线（见 文档\引脚图-B车C8T6-20261002.html）：
  *   PB13 = SCL/SPC（SPI2_SCK）
  *   PB14 = SA0/SDO（SPI2_MISO）  ⚠ 别接地/接 3.3V，SPI 模式下它就是 MISO
  *   PB15 = SDA/SDI（SPI2_MOSI）
  *   PB12 = CS（软件片选，GPIO 输出，低有效）
  *   PB8  = INT1 → EXTI9_5（DRDY：数据好了芯片拉一下，采样时刻由芯片给）
  *   PB9  = INT2（备用）
  *
  * ★寄存器值不是猜的：取自本工程 `开源工程\飞檐走壁-STC32G源码\Libraries\zf_device\
  *   zf_device_imu660rc.c`（逐飞官方对这颗模块的驱动）。搬运的是**寄存器值**，
  *   不是它的代码 —— 平台是 RT1064/STC32G，仓不能混用。
  ******************************************************************************
  */
#ifndef __DRV_IMU660_H
#define __DRV_IMU660_H

#include <stdint.h>

typedef enum {
    IMU_AXIS_X = 0,
    IMU_AXIS_Y = 1,
    IMU_AXIS_Z = 2
} imu_axis_t;

/**
  * @brief  初始化 SPI2 + IMU660RC，并做 WHO_AM_I 自检
  * @retval 1 = 成功（读到 0x70）；0 = 失败（接线/模式问题）
  * @note   SPI 模式会**自动试**：先试 mode 3，读不到 0x70 再试 mode 0。
  *         这颗芯片两种模式都支持，自动试掉一整类"接对了但读不出来"的坑。
  */
uint8_t imu660_init(void);

/** @brief 读一次三轴（陀螺 + 加速度），存在驱动内部 */
void imu660_read(void);

/** @brief 原始值（有符号 16 位）*/
int16_t imu660_raw_gyro(imu_axis_t a);
int16_t imu660_raw_acc(imu_axis_t a);

/**
  * @brief  换算成物理量
  * @note   陀螺：±2000dps ⇒ **0.07 dps/LSB**（= 1/14.2857，逐飞官方换算系数）
  *         加速度：±8g ⇒ 4096 LSB/g
  */
float imu660_gyro_dps(imu_axis_t a);
float imu660_acc_g(imu_axis_t a);

/** @brief DRDY 中断入口调用（PB8 上升沿）*/
void imu660_drdy_isr(void);

/** @brief 取"有新数据"标志（取走清零），主循环用 */
uint8_t imu660_drdy_take(void);

/** @brief 上电至今成功读数次数（诊断用：不动车它也应该在涨）*/
uint32_t imu660_read_count(void);

#endif /* __DRV_IMU660_H */
