/**
  ******************************************************************************
  * @file    str_util.c
  * @brief   命令解析小工具实现（纯逻辑，可在 PC 上单测）
  ******************************************************************************
  */
#include "str_util.h"

uint8_t su_streq(const char *a, const char *b)
{
    if (a == 0 || b == 0) {
        return 0u;
    }
    while (*a != '\0' && *b != '\0') {
        if (*a != *b) {
            return 0u;
        }
        a++;
        b++;
    }
    return (*a == '\0' && *b == '\0') ? 1u : 0u;
}

void su_upper(char *s)
{
    if (s == 0) {
        return;
    }
    while (*s != '\0') {
        if (*s >= 'a' && *s <= 'z') {
            *s = (char)(*s - 'a' + 'A');
        }
        s++;
    }
}

char *su_arg(char *line, uint8_t n)
{
    char *p = line;
    uint8_t idx = 0u;

    if (line == 0) {
        return 0;
    }
    while (*p != '\0') {
        while (*p == ' ') {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        if (idx == n) {
            return p;
        }
        while (*p != '\0' && *p != ' ') {
            p++;
        }
        idx++;
    }
    return 0;
}

void su_tok(char *p)
{
    char *s = p;
    if (p == 0) {
        return;
    }
    while (*s != '\0' && *s != ' ') {
        s++;
    }
    if (*s == ' ') {
        *s = '\0';
    }
}

uint8_t su_word(const char *p, char *dst, uint8_t dst_size)
{
    uint8_t n = 0u;

    if (p == 0 || dst == 0 || dst_size == 0u) {
        return 0u;
    }
    while (p[n] != '\0' && p[n] != ' ') {
        if (n >= (uint8_t)(dst_size - 1u)) {
            dst[0] = '\0';
            return 0u;      /* 放不下：宁可报失败，也不要截断出一个"看起来对"的词 */
        }
        dst[n] = p[n];
        n++;
    }
    dst[n] = '\0';
    return n;
}

uint8_t su_parse_u32(const char *s, uint32_t *out)
{
    uint32_t v = 0u;
    uint8_t  any = 0u;

    if (s == 0 || out == 0) {
        return 0u;
    }
    while (*s == ' ') {
        s++;
    }
    while (*s >= '0' && *s <= '9') {
        uint32_t digit = (uint32_t)(*s - '0');
        /* 越界保护：> 429496729 时再乘 10 必溢出 */
        if (v > 429496729u) {
            return 0u;
        }
        v = v * 10u + digit;
        any = 1u;
        s++;
    }
    if (!any || *s != '\0') {
        return 0u;      /* 后面还挂着非数字字符 → 整串非法（例如 "500x"）*/
    }
    *out = v;
    return 1u;
}

static uint8_t parse_dir_word(const char *w, uint8_t *dir_out)
{
    if (su_streq(w, "FWD"))   { *dir_out = SU_DIR_FWD;   return 1u; }
    if (su_streq(w, "REV"))   { *dir_out = SU_DIR_REV;   return 1u; }
    if (su_streq(w, "BRAKE")) { *dir_out = SU_DIR_BRAKE; return 1u; }
    if (su_streq(w, "COAST")) { *dir_out = SU_DIR_COAST; return 1u; }
    return 0u;
}

uint8_t su_parse_motor(char *line, su_motor_t *out)
{
    char *a1, *a2, *a3;

    if (line == 0 || out == 0) {
        return 0u;
    }
    out->valid    = 0u;
    out->has_side = 0u;
    out->side     = 0u;
    out->dir      = SU_DIR_BRAKE;
    out->mag      = 0u;

    /* ★先把三个参数指针全部取到，再逐个截断 ——
     * 反过来的话，截断 a1 会在 a1 后面写 '\0'，su_arg 就再也走不到 a2/a3 了。 */
    a1 = su_arg(line, 1u);
    a2 = su_arg(line, 2u);
    a3 = su_arg(line, 3u);

    if (a1 == 0) {
        return 0u;
    }

    /* 指针都拿到之后才能截断：a1 必须先截断，否则拿 "L FWD 500" 去比 "L" 永远不等
     *（这个 bug 就是被 Tools\test_parse.c 单测抓出来的）。 */
    su_tok(a1);
    if (a2 != 0) { su_tok(a2); }
    if (a3 != 0) { su_tok(a3); }

    /* 三参数形式：MOTOR L|R <dir> <mag> */
    if (a3 != 0 && (su_streq(a1, "L") || su_streq(a1, "R"))) {
        out->has_side = 1u;
        out->side     = su_streq(a1, "L") ? 0u : 1u;
        if (!parse_dir_word(a2, &out->dir)) {
            return 0u;
        }
        if (!su_parse_u32(a3, &out->mag) || out->mag > 1000u) {
            return 0u;
        }
        out->valid = 1u;
        return 1u;
    }

    /* 两参数形式：MOTOR <dir> <mag> */
    if (!parse_dir_word(a1, &out->dir)) {
        return 0u;
    }
    if (a2 == 0 || !su_parse_u32(a2, &out->mag) || out->mag > 1000u) {
        return 0u;
    }
    out->valid = 1u;
    return 1u;
}
