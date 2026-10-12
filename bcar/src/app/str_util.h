/**
  ******************************************************************************
  * @file    str_util.h
  * @brief   命令解析用的小工具（**纯逻辑，无硬件依赖，可在 PC 上单测**）
  *
  * 为什么单独拆出来：命令行解析是最容易写错、又最值得在电脑上测的一层。
  * 本文件不许 #include 任何 stm32/HAL/驱动头文件 —— 这是硬纪律，
  * 保证 Tools\test_parse.c 能用普通 gcc 直接编译运行。
  ******************************************************************************
  */
#ifndef __APP_STR_UTIL_H
#define __APP_STR_UTIL_H

#include <stdint.h>

/** @brief 全串相等（自带终止，不用 libc 的 strcmp）*/
uint8_t su_streq(const char *a, const char *b);

/** @brief 原地转大写（命令大小写不敏感）*/
void su_upper(char *s);

/**
  * @brief  取第 n 个空格分隔参数（n 从 0 开始），**不修改缓冲区**
  * @return 指向该参数的指针；没有则返回 0
  * @note   注意：它会一直走到字符串结尾，所以"先把所有参数指针都取到，
  *         再逐个 su_tok()"才是正确用法 —— 反过来会截断掉后面的参数。
  */
char *su_arg(char *line, uint8_t n);

/** @brief 就地截断：把 p 这个单词结尾的空格改成 '\0' */
void su_tok(char *p);

/** @brief 把 p 处第一个单词拷进 dst（含终止符），返回单词长度；放不下返回 0 */
uint8_t su_word(const char *p, char *dst, uint8_t dst_size);

/** @brief 解析十进制无符号数；返回 1 = 成功 */
uint8_t su_parse_u32(const char *s, uint32_t *out);

/* ---------------- MOTOR 命令的解析结果 ---------------- */

/* dir 取值与 drivers\motor.h 的 motor_dir_t 一一对应，故意保持一致：
 * 这样调用方可以直接强转，少一层映射就少一处能写错的地方。 */
#define SU_DIR_FWD   0u
#define SU_DIR_REV   1u
#define SU_DIR_BRAKE 2u
#define SU_DIR_COAST 3u

typedef struct {
    uint8_t  valid;      /* 1 = 解析成功 */
    uint8_t  has_side;   /* 1 = 指定了 L / R */
    uint8_t  side;       /* 0 = L，1 = R */
    uint8_t  dir;        /* SU_DIR_* */
    uint32_t mag;        /* 0~1000 */
} su_motor_t;

/**
  * @brief  解析 `MOTOR <dir> <mag>` 或 `MOTOR L|R <dir> <mag>`
  * @param  line 命令原文（"MOTOR" 之后的部分也会被读到；函数内部只读第 1 个参数起）
  * @param  out  解析结果
  * @retval 1 = 成功；0 = 失败（用法错/方向词非法/力度越界）
  * @note   会就地修改 line（截断各参数尾空格），故参数类型是 char*。
  */
uint8_t su_parse_motor(char *line, su_motor_t *out);

#endif /* __APP_STR_UTIL_H */
