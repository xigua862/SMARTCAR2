/**
 * @file gyroscope.h
 * @brief IMU 采样、航向积分、零偏校准和姿态保护数据。
 * @author 高冷马铃薯
 *
 * @par 所属项目
 * 2026 智能车区赛（摄像头）
 *
 * @par WegooAI 中转站
 * 本项目用户代码 99% 通过作者自建的 WegooAI 中转站，
 * 使用 Claude Code、Codex 等 AI 编程工具完成。
 * WegooAI 由作者高冷马铃薯搭建并维护，
 * 方便大家接入 AI 编程工具编写和修改代码，欢迎使用 :)
 * 官网：https://ai.wegoo.site/
 *
 * @par 联系作者
 * Bilibili：https://space.bilibili.com/513729639
 * QQ：1153626206
 * Email：moonserver@foxmail.com
 */

/*
 * gyroscope.h
 *
 * Migrated from the 264 project.
 */

#ifndef CODE_GYROSCOPE_H_
#define CODE_GYROSCOPE_H_

#include "zf_common_headfile.h"

typedef struct
{
    /* 加速度，单位由底层 transition 换算后给出。 */
    float acc_x;
    float acc_y;
    float acc_z;

    /* 角速度，单位 rad/s，零飘校准完成后会扣除偏置。 */
    float gyro_x;
    float gyro_y;
    float gyro_z;

    /* Mahony 姿态滤波（软件）解算的欧拉角，单位 degree。
     * yaw 无磁力计参考仅供显示；航向判据用 yaw_total。 */
    float roll;
    float pitch;
    float yaw;
    /* Mahony 软件四元数。 */
    float quaternion[4];
} IMU_param_t;

extern volatile IMU_param_t IMU_Data;

/* 运行控制使用的连续航向（度，右转为正）。
 * 具体来源由 gyroscope.c 当前航向模式决定，可被软件归零；
 * 转弯退出判据使用它，不可拿它作为独立传感器标定真值。 */
extern volatile float yaw_total;

/* 独立纯陀螺连续航向（度，右转为正）。
 * 仅随每次有效 IMU 航向增量累加，不响应 yaw_total 软件归零，
 * 也不受编码器航向源和融合模式影响；用于相机/编码器标定参考。 */
float gyro_get_pure_yaw_deg(void);

/* 采集一帧 IMU 数据并完成 yaw 积分（中断内调用，dt=实测采样间隔秒）。 */
void IMU_GetValues(float dt);
/* 请求把连续航向 yaw_total 归零；真正清零在 IMU 中断完成。 */
void gyro_request_yaw_total_reset(void);
/* 读取编码器差速航向（度，右转为正，与 yaw_total 同步归零）。
 * 融合的主导源：轮子滚动时 yaw_total 以短时间常数跟随它；
 * 轮子没滚时它跟随 yaw_total（手转调试不受拉扯）。调试页 E 值即它。 */
float gyro_get_yaw_enc_total(void);
/* 发车瞬间调用：归零航向 + 以当前姿态重锚 roll/pitch 保护零点。
 * 零偏沿用上电静置校准，发车流程不再重新校准。 */
void gyro_request_launch_rezero(void);
/* 初始化 IMU（四元数/SFLP 禁用，INT2 配为 480Hz 陀螺数据就绪中断）。 */
void gyro_init(void);
/* IMU 数据就绪外部中断的采样入口。 */
void gyro_external_interrupt_update(void);
/* 获取相对 roll。四元数关闭后无数据源，恒为 0，仅保留接口。 */
float gyro_get_roll_delta(void);
/* 获取相对 pitch。同 roll，恒为 0。 */
float gyro_get_pitch_delta(void);
/* 请求开始 500ms 零飘校准，真正采样在 IMU 外部中断中完成。 */
void gyro_zero_calibration_start(void);
/* 查询零飘是否完成。 */
uint8 gyro_zero_calibration_done(void);
/* 查询零飘是否正在等待开始或正在采样。 */
uint8 gyro_zero_calibration_running(void);

#endif /* CODE_GYROSCOPE_H_ */
