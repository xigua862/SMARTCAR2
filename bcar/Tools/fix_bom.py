#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
给 bcar 的自写源码统一补 UTF-8 BOM。

为什么必须：
  ARMCC(Keil) 在中文 Windows 上默认按 GBK 解析源码；我们的 .c 是 UTF-8，
  字符串里的中文就会被判成 "invalid multibyte character sequence"（#870-D），
  编译出一堆 warning。而阶段验收要求 0 warning。加 BOM 后 ARMCC 就知道是 UTF-8。
  （二进制里存的仍是 UTF-8 字节，串口终端按 UTF-8 显示中文正常。）

用法（在 bcar 目录下）：
    python Tools\\fix_bom.py
幂等：已有 BOM 的文件不动。建议每次用编辑器大步改写 .c/.h 之后跑一次。
"""
import glob
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BCAR = os.path.dirname(HERE)

PATTERNS = [
    "src/**/*.c", "src/**/*.h",
    "Core/Inc/*.h", "Core/Src/main.c", "Core/Src/stm32f1xx_it.c",
]
BOM = b"\xef\xbb\xbf"


def main():
    changed = []
    for pat in PATTERNS:
        for f in sorted(glob.glob(os.path.join(BCAR, pat), recursive=True)):
            raw = open(f, "rb").read()
            if raw.startswith(BOM):
                continue
            if not any(b > 127 for b in raw):
                continue          # 纯 ASCII 不必加
            open(f, "wb").write(BOM + raw)
            changed.append(os.path.relpath(f, BCAR))
    if changed:
        print("已补 BOM：")
        for c in changed:
            print("  " + c)
    else:
        print("所有自写源码都已有 BOM（或无需）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
