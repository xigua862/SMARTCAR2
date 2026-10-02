/*****************************************************************************
* 文件名称: eeprom_data.c
* 作者:     get_lucky()
* 日期:     2026.8.31
* 功能:     EEPROM(IAP)数据存取实现：菜单参数块保存/恢复/校验、通用类型读写接口
*****************************************************************************/

/*********************************************************************************************************************
* 文件名:      eeprom_data.c
* 说明:        EEPROM 数据存取模块实现
*              基于逐飞库 zf_driver_eeprom(IAP) 底层驱动(iap_init/iap_read_buff/iap_write_buff/
*              iap_erase_page/extern_iap_write_buff)
*              通过联合体 eeprom_val_t 以"内存视图"方式直接读写, 无手动字节拼接
* 适用平台:    STC32G (AI32G12K128)
* 修改记录:
* 日期          作者         备注
* 2026.8.31     get_lucky()  更新文件头注释
*********************************************************************************************************************/

#include "my_headfile.h"
#include "eeprom_data.h"
#include "zf_driver_eeprom.h"

//-------------------------------------------------------------------------------------------------------------------
// 参数块定义 (存放在扇区0, 0x0000 起, 共 51 字节, 不跨扇区)
// 布局: 魔数(4) + 版本号(1) + 保留(3) + 14 个参数(42字节) + 校验和(1)
// 说明: 校验和必须是最后一个成员, 计算时对其余所有字节求和
//-------------------------------------------------------------------------------------------------------------------
#define PARAM_MAGIC                 (0xA55A5AA5u)   // 魔数, 用于识别参数数据是否有效
#define PARAM_VERSION               (0x02u)         // 参数块版本号, 修改结构体布局后需递增

typedef struct
{
    uint32  magic;                  // 魔数
    uint8   version;                // 版本号
    uint8   reserve[3];             // 保留字段
    int16   car_speed;              // 目标车速
    uint16  fly_dis;                // 飞坡距离
    uint16  fly_tim;                // 飞坡时间
    uint16  in_ring_dis_d;          // 大环入环距离
    uint16  in_ring_dis_x;          // 小环入环距离
    uint16  in_ring_threshold;      // 入环阈值
    uint16  out_ring_tim;           // 退环时间
    float   Kb;                     // 竖电感比例
    float   r_offset_d;             // 大环入环偏差
    float   r_offset_x;             // 小环入环偏差
    float   angle_kp;               // 角度环比例
    float   angle_kd;               // 角度环微分
    float   angle_rg1_kp;           // 圆环1角度环比例
    float   angle_rg1_kd;           // 圆环1角度环微分
    uint8   checksum;               // 校验和(必须是最后一个成员)
} sys_param_t;

static sys_param_t sys_param;                           // 当前参数块(内存镜像)

// 出厂默认参数(与 pid.c / adc.c 中的初始值保持一致)
static const sys_param_t sys_param_default =
{
    PARAM_MAGIC,                    // magic
    PARAM_VERSION,                  // version
    {0, 0, 0},                      // reserve
    500,                            // car_speed
    2600,                           // fly_dis
    60,                             // fly_tim
    6000,                           // in_ring_dis_d
    5000,                           // in_ring_dis_x
    1300,                           // in_ring_threshold
    500,                            // out_ring_tim
    1.2f,                           // Kb
    35.0f,                          // r_offset_d
    50.0f,                          // r_offset_x
    55.0f,                          // angle_kp
    50.0f,                          // angle_kd
    70.0f,                          // angle_rg1_kp
    50.0f,                          // angle_rg1_kd
    0,                              // checksum, 写入时重新计算
};

//-------------------------------------------------------------------------------------------------------------------
// @brief      计算校验和: 对除最后一个字节外的所有字节求和
// @param      p   参数块指针
// @return     校验和
//-------------------------------------------------------------------------------------------------------------------
static uint8 param_checksum(const sys_param_t *p)
{
    uint8   sum = 0;
    uint16  i;
    const uint8 *b = (const uint8 *)p;

    for(i = 0; i < (sizeof(sys_param_t) - 1); i++)
    {
        sum += b[i];
    }
    return sum;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief      EEPROM 初始化(使能 IAP), 开机调用一次
//-------------------------------------------------------------------------------------------------------------------
void eeprom_init(void)
{
    iap_init();
}

//-------------------------------------------------------------------------------------------------------------------
// @brief      上电加载参数, 校验失败(首次上电/数据损坏)自动恢复出厂默认值
//-------------------------------------------------------------------------------------------------------------------
void sys_param_load(void)
{
    iap_read_buff(EEPROM_PARAM_BASE, (uint8 *)&sys_param, sizeof(sys_param_t));

    // 校验: 魔数 + 版本号 + 校验和
    if((sys_param.magic    != PARAM_MAGIC)   ||
       (sys_param.version  != PARAM_VERSION) ||
       (sys_param.checksum != param_checksum(&sys_param)))
    {
        sys_param_reset();          // 数据无效, 恢复出厂默认值并写回
        return;
    }

    // 参数生效到全局变量
    car_speed           = sys_param.car_speed;
    fly_dis             = sys_param.fly_dis;
    fly_tim             = sys_param.fly_tim;
    in_ring_dis_d       = sys_param.in_ring_dis_d;
    in_ring_dis_x       = sys_param.in_ring_dis_x;
    in_ring_threshold   = sys_param.in_ring_threshold;
    out_ring_tim        = sys_param.out_ring_tim;
    Kb                  = sys_param.Kb;
    r_offset_d          = sys_param.r_offset_d;
    r_offset_x          = sys_param.r_offset_x;
    angle_kp            = sys_param.angle_kp;
    angle_kd            = sys_param.angle_kd;
    angle_rg1_kp        = sys_param.angle_rg1_kp;
    angle_rg1_kd        = sys_param.angle_rg1_kd;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief      保存当前全局参数到 EEPROM
//-------------------------------------------------------------------------------------------------------------------
void sys_param_save(void)
{
    // 同步全局变量到参数块
    sys_param.magic             = PARAM_MAGIC;
    sys_param.version           = PARAM_VERSION;
    sys_param.car_speed         = car_speed;
    sys_param.fly_dis           = fly_dis;
    sys_param.fly_tim           = fly_tim;
    sys_param.in_ring_dis_d     = in_ring_dis_d;
    sys_param.in_ring_dis_x     = in_ring_dis_x;
    sys_param.in_ring_threshold = in_ring_threshold;
    sys_param.out_ring_tim      = out_ring_tim;
    sys_param.Kb                = Kb;
    sys_param.r_offset_d        = r_offset_d;
    sys_param.r_offset_x        = r_offset_x;
    sys_param.angle_kp          = angle_kp;
    sys_param.angle_kd          = angle_kd;
    sys_param.angle_rg1_kp      = angle_rg1_kp;
    sys_param.angle_rg1_kd      = angle_rg1_kd;

    sys_param.checksum = param_checksum(&sys_param);

    // 读扇区->改->擦除->写回(参数块在扇区0内, 不跨扇区)
    eeprom_write_buff(EEPROM_PARAM_BASE, (uint8 *)&sys_param, sizeof(sys_param_t));
}

//-------------------------------------------------------------------------------------------------------------------
// @brief      恢复出厂默认值(写回 EEPROM 并立即生效)
//-------------------------------------------------------------------------------------------------------------------
void sys_param_reset(void)
{
    uint16      i;
    const uint8 *src = (const uint8 *)&sys_param_default;
    uint8       *dst = (uint8 *)&sys_param;

    for(i = 0; i < sizeof(sys_param_t); i++)
    {
        dst[i] = src[i];
    }

    sys_param.checksum = param_checksum(&sys_param);
    eeprom_write_buff(EEPROM_PARAM_BASE, (uint8 *)&sys_param, sizeof(sys_param_t));

    sys_param_load();               // 重新加载并生效
}

//-------------------------------------------------------------------------------------------------------------------
// 通用接口: 联合体内存视图直接读写
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// @brief      检查 [addr, addr+len) 是否全部落在 EEPROM 容量内
// @return     1=合法 0=越界
//-------------------------------------------------------------------------------------------------------------------
static uint8 eeprom_range_ok(uint16 addr, uint16 len)
{
    if(addr >= EEPROM_SIZE_BYTES)   return 0;
    if(len  >  EEPROM_SIZE_BYTES)   return 0;
    if((addr + len) > EEPROM_SIZE_BYTES) return 0;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief      读多个字节(内存视图, 直接拷贝到 buf)
//-------------------------------------------------------------------------------------------------------------------
void eeprom_read_buff(uint16 addr, uint8 *buf, uint16 len)
{
    if(!eeprom_range_ok(addr, len)) return;
    iap_read_buff(addr, buf, len);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief      写多个字节(读扇区->改->擦除->写回), 越界或跨扇区直接返回
//-------------------------------------------------------------------------------------------------------------------
void eeprom_write_buff(uint16 addr, uint8 *buf, uint16 len)
{
    if(!eeprom_range_ok(addr, len)) return;
    if(((addr & (EEPROM_SECTOR_SIZE - 1)) + len) > EEPROM_SECTOR_SIZE) return;   // 禁止跨扇区
    extern_iap_write_buff(addr, buf, len);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief      读一个 8 位数据
//-------------------------------------------------------------------------------------------------------------------
uint8 eeprom_read_u8(uint16 addr)
{
    eeprom_val_t v;
    if(!eeprom_range_ok(addr, 1)) return 0;
    iap_read_buff(addr, v.bytes, 1);
    return v.u8;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief      写一个 8 位数据
//-------------------------------------------------------------------------------------------------------------------
void eeprom_write_u8(uint16 addr, uint8 val)
{
    eeprom_val_t v;
    v.u8 = val;
    eeprom_write_buff(addr, v.bytes, 1);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief      读一个 16 位数据
//-------------------------------------------------------------------------------------------------------------------
uint16 eeprom_read_u16(uint16 addr)
{
    eeprom_val_t v;
    if(!eeprom_range_ok(addr, 2)) return 0;
    iap_read_buff(addr, v.bytes, 2);
    return v.u16;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief      写一个 16 位数据
//-------------------------------------------------------------------------------------------------------------------
void eeprom_write_u16(uint16 addr, uint16 val)
{
    eeprom_val_t v;
    v.u16 = val;
    eeprom_write_buff(addr, v.bytes, 2);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief      读一个 32 位数据
//-------------------------------------------------------------------------------------------------------------------
uint32 eeprom_read_u32(uint16 addr)
{
    eeprom_val_t v;
    if(!eeprom_range_ok(addr, 4)) return 0;
    iap_read_buff(addr, v.bytes, 4);
    return v.u32;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief      写一个 32 位数据
//-------------------------------------------------------------------------------------------------------------------
void eeprom_write_u32(uint16 addr, uint32 val)
{
    eeprom_val_t v;
    v.u32 = val;
    eeprom_write_buff(addr, v.bytes, 4);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief      读一个 32 位浮点
//-------------------------------------------------------------------------------------------------------------------
float eeprom_read_f32(uint16 addr)
{
    eeprom_val_t v;
    if(!eeprom_range_ok(addr, 4)) return 0.0f;
    iap_read_buff(addr, v.bytes, 4);
    return v.f32;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief      写一个 32 位浮点
//-------------------------------------------------------------------------------------------------------------------
void eeprom_write_f32(uint16 addr, float val)
{
    eeprom_val_t v;
    v.f32 = val;
    eeprom_write_buff(addr, v.bytes, 4);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief      擦除 addr 所在扇区(512 字节)
//-------------------------------------------------------------------------------------------------------------------
void eeprom_erase_sector(uint16 addr)
{
    if(addr >= EEPROM_SIZE_BYTES)
    {
        return;
    }
    iap_erase_page(addr);
}
