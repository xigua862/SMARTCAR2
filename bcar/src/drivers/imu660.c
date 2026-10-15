/**
  ******************************************************************************
  * @file    imu660.c
  * @brief   IMU660RC（LSM6DSV16X）驱动实现 —— SPI2 寄存器级
  *
  * 寄存器值与初始化序列来源：逐飞官方 zf_device_imu660rc.c（见 imu660.h 说明）
  ******************************************************************************
  */
#include "imu660.h"
#include "bcar_config.h"
#include "bsp_gpio.h"
#include "bsp_systick.h"
#include "stm32f1xx.h"

/* ---------------- 寄存器地址（LSM6DSV16X，取自逐飞头文件） ---------------- */
#define IMU_FUNC_CFG_ACCESS  0x01u
#define IMU_INT1_CTRL        0x0Du
#define IMU_CHIP_ID          0x0Fu
#define IMU_CTRL1            0x10u
#define IMU_CTRL2            0x11u
#define IMU_CTRL3            0x12u
#define IMU_CTRL6            0x15u
#define IMU_CTRL7            0x16u
#define IMU_CTRL9            0x18u
#define IMU_OUTX_L_G         0x22u
#define IMU_OUTX_L_A         0x28u

#define IMU_WHO_AM_I_EXPECT  0x70u   /* 读到这个值才算认出芯片 */

/* 初始化值（照抄逐飞；括号里是它对应的物理含义）*/
#define IMU_CTRL1_VAL        0x15u   /* 加速度 ODR 120Hz + 高精度 */
#define IMU_CTRL2_VAL        0x18u   /* 陀螺 ODR 960Hz（逐飞默认）*/
#define IMU_CTRL3_VAL        0x44u   /* ★BDU=1（读的时候不许更新）+ IF_INC=1（多字节读自增）*/
#define IMU_CTRL6_VAL        0x04u   /* ★陀螺量程 ±2000dps ⇒ 0.07 dps/LSB */
#define IMU_CTRL7_VAL        0x01u
#define IMU_CTRL9_VAL        0x08u
#define IMU_INT1_DRDY_G      0x02u   /* INT1_CTRL 的 bit1 = 陀螺 DRDY 打到 INT1 */

/* 物理量换算 */
#define IMU_GYRO_DPS_PER_LSB 0.07f   /* ±2000dps（1/14.2857，逐飞官方系数）*/
#define IMU_ACC_G_PER_LSB    (1.0f / 4096.0f)   /* ±8g */

#define IMU_CS_LOW()   (GPIOB->BSRR = ((uint32_t)GPIO_PIN_12 << 16))
#define IMU_CS_HIGH()  (GPIOB->BSRR = GPIO_PIN_12)

static int16_t  s_gyro[3];
static int16_t  s_acc[3];
static volatile uint8_t  s_drdy = 0u;
static volatile uint32_t s_read_cnt = 0u;

/* ---------------- SPI2 底层 ---------------- */

static void spi2_cs_setup(void)
{
    /* PB12 = 软件片选：通用推挽输出 50MHz，空闲拉高 */
    GPIOB->CRH = (GPIOB->CRH & ~(0xFu << 16)) | (0x3u << 16);   /* pin12 → 位 16~19 */
    IMU_CS_HIGH();
}

static void spi2_config(uint8_t cpol, uint8_t cpha)
{
    /* 关掉再配（F1 的 SPI 配置位要求 SPE=0）*/
    SPI2->CR1 = 0u;

    /* 主模式 + 软件 NSS（我们自己拉 PB12）+ 8 位 + 分频 /4
     * 分频位 BR[5:3]：001 = /4 → APB1 36MHz /4 = **9MHz**（芯片上限 10MHz，别用 /2）*/
    SPI2->CR1 = (uint32_t)(SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (0x1u << 3));
    if (cpol) {
        SPI2->CR1 |= SPI_CR1_CPOL;
    }
    if (cpha) {
        SPI2->CR1 |= SPI_CR1_CPHA;
    }
    /* MSB 先行（F1 默认就是）、CRC 关 */
    SPI2->CR2 = 0u;
    SPI2->CR1 |= SPI_CR1_SPE;
}

static uint8_t spi2_xfer(uint8_t out)
{
    while ((SPI2->SR & SPI_SR_TXE) == 0u) {
        /* 等发送寄存器空 */
    }
    *(volatile uint8_t *)&SPI2->DR = out;
    while ((SPI2->SR & SPI_SR_RXNE) == 0u) {
        /* 等接收寄存器有货 */
    }
    return *(volatile uint8_t *)&SPI2->DR;
}

/* ---------------- 寄存器读写 ---------------- */

static void imu_write_reg(uint8_t reg, uint8_t val)
{
    IMU_CS_LOW();
    (void)spi2_xfer((uint8_t)(reg & 0x7Fu));   /* 写：bit7 = 0 */
    (void)spi2_xfer(val);
    IMU_CS_HIGH();
}

static uint8_t imu_read_reg(uint8_t reg)
{
    uint8_t v;
    IMU_CS_LOW();
    (void)spi2_xfer((uint8_t)(reg | 0x80u));   /* 读：bit7 = 1 */
    v = spi2_xfer(0x00u);
    IMU_CS_HIGH();
    return v;
}

static void imu_read_regs(uint8_t reg, uint8_t *dst, uint8_t len)
{
    uint8_t i;
    IMU_CS_LOW();
    (void)spi2_xfer((uint8_t)(reg | 0x80u));
    for (i = 0u; i < len; i++) {
        dst[i] = spi2_xfer(0x00u);
    }
    IMU_CS_HIGH();
}

/* ---------------- DRDY：PB8 → EXTI9_5 ---------------- */

static void imu_drdy_exti_setup(void)
{
    /* PB8 → EXTI8：AFIO->EXTICR[2] 的 bit[3:0] = 0x1（端口 B）*/
    AFIO->EXTICR[2] = (AFIO->EXTICR[2] & ~0x0000000Fu) | 0x00000001u;

    EXTI->IMR  |= (1u << 8);        /* 放开 EXTI8 中断 */
    EXTI->RTSR |= (1u << 8);        /* 上升沿触发（DRDY 是"数据好了"的脉冲）*/
    EXTI->EMR  &= ~(1u << 8);       /* 不要事件模式 */

    NVIC_SetPriority(EXTI9_5_IRQn, 6u);   /* 比控制节拍略低，别抢 SysTick */
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void imu660_drdy_isr(void)
{
    if (EXTI->PR & (1u << 8)) {
        EXTI->PR = (1u << 8);       /* 写 1 清标志 */
        s_drdy = 1u;
    }
}

uint8_t imu660_drdy_take(void)
{
    if (s_drdy) {
        s_drdy = 0u;
        return 1u;
    }
    return 0u;
}

uint32_t imu660_read_count(void)
{
    return s_read_cnt;
}

/* ---------------- 初始化 ---------------- */

static uint8_t imu_try_mode(uint8_t cpol, uint8_t cpha)
{
    uint8_t id;

    spi2_config(cpol, cpha);

    /* 芯片上电后需要一点时间启动；连续读到 0x70 才算认出来 */
    {
        uint16_t retry = 20u;
        while (retry > 0u) {
            id = imu_read_reg(IMU_CHIP_ID);
            if (id == IMU_WHO_AM_I_EXPECT) {
                return 1u;
            }
            retry--;
        }
    }
    return 0u;
}

uint8_t imu660_init(void)
{
    uint8_t ok;

    /* 1) 时钟：SPI2 在 APB1；GPIOB 在 APB2 */
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    (void)RCC->APB1ENR;
    (void)RCC->APB2ENR;

    /* 2) 三个 SPI 脚：复用推挽 50MHz（SCK 要快边沿）*/
    bsp_gpio_set_af_pp(GPIOB, 13u, BSP_GPIO_SPEED_50MHZ);   /* SCK */
    bsp_gpio_set_af_pp(GPIOB, 15u, BSP_GPIO_SPEED_50MHZ);   /* MOSI */
    /* MISO 在上电时先按浮空输入配，SPI 使能后由外设接管 */
    bsp_gpio_set_input_floating(GPIOB, 14u);

    spi2_cs_setup();

    /* 3) 自动试 SPI 模式：先 mode 3（CPOL=1,CPHA=1），不行再 mode 0 */
    ok = imu_try_mode(1u, 1u);
    if (!ok) {
        ok = imu_try_mode(0u, 0u);
    }
    if (!ok) {
        return 0u;
    }

    /* 4) 初始化序列（照抄逐飞官方）*/
    imu_write_reg(IMU_FUNC_CFG_ACCESS, 0x04u);   /* 解锁 embedded functions 页 */
    /* ★2026-10-15：官方初始化序列要求这里等 30ms（切换寄存器页需要时间）。
     * 原来没等，实测后果 = WHO_AM_I 读得到、但后面 CTRL 寄存器全没写进去，
     * 芯片留在掉电态 ⇒ IMU 原始值恒为 0（G(0,0,0) A(0,0,0)）。
     * 这里用 millis 忙等，不引 HAL_Delay。 */
    {
        uint32_t t0 = bsp_millis();
        while ((uint32_t)(bsp_millis() - t0) < 30u) {
            /* 空转 30ms */
        }
    }
    imu_write_reg(IMU_CTRL3, IMU_CTRL3_VAL);     /* BDU + IF_INC */
    imu_write_reg(IMU_CTRL6, IMU_CTRL6_VAL);     /* 陀螺 ±2000dps */
    imu_write_reg(IMU_CTRL1, IMU_CTRL1_VAL);     /* 加速度 120Hz */
    imu_write_reg(IMU_CTRL2, IMU_CTRL2_VAL);     /* 陀螺 960Hz */
    imu_write_reg(IMU_CTRL7, IMU_CTRL7_VAL);
    imu_write_reg(IMU_CTRL9, IMU_CTRL9_VAL);

    /* 5) 把"陀螺数据好了"打到 INT1（PB8 用）*/
    imu_write_reg(IMU_INT1_CTRL, IMU_INT1_DRDY_G);

    imu_drdy_exti_setup();

    s_gyro[0] = s_gyro[1] = s_gyro[2] = 0;
    s_acc[0]  = s_acc[1]  = s_acc[2]  = 0;
    s_read_cnt = 0u;
    s_drdy = 0u;
    return 1u;
}

/* ---------------- 读数 ---------------- */

void imu660_read(void)
{
    uint8_t g[6];
    uint8_t a[6];

    imu_read_regs(IMU_OUTX_L_G, g, 6u);
    imu_read_regs(IMU_OUTX_L_A, a, 6u);

    s_gyro[0] = (int16_t)(((uint16_t)g[1] << 8) | g[0]);
    s_gyro[1] = (int16_t)(((uint16_t)g[3] << 8) | g[2]);
    s_gyro[2] = (int16_t)(((uint16_t)g[5] << 8) | g[4]);

    s_acc[0]  = (int16_t)(((uint16_t)a[1] << 8) | a[0]);
    s_acc[1]  = (int16_t)(((uint16_t)a[3] << 8) | a[2]);
    s_acc[2]  = (int16_t)(((uint16_t)a[5] << 8) | a[4]);

    s_read_cnt++;
}

int16_t imu660_raw_gyro(imu_axis_t a)
{
    return ((uint8_t)a < 3u) ? s_gyro[a] : 0;
}

int16_t imu660_raw_acc(imu_axis_t a)
{
    return ((uint8_t)a < 3u) ? s_acc[a] : 0;
}

float imu660_gyro_dps(imu_axis_t a)
{
    return (float)imu660_raw_gyro(a) * IMU_GYRO_DPS_PER_LSB;
}

float imu660_acc_g(imu_axis_t a)
{
    return (float)imu660_raw_acc(a) * IMU_ACC_G_PER_LSB;
}
