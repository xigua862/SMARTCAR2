#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
编译 bcar（不开 Keil 界面），并且**保证不会被日志骗**。

为什么要这个脚本：
  1) UV4 的 -o build.log 是**追加**的 —— 不清空的话，上一次成功的
     `0 Error(s), 0 Warning(s)` 会一直留在日志里，而这次的报错行格式不一样
     （`error:  #8: ...`），结果 grep "Error(s)" 会得到**假的"编译通过"**。
     这个坑真踩过：改完源码其实编译报错，没发现，烧进去的还是旧 hex。
  2) 所以本脚本先删 build.log，再编，最后同时认 `error:` 和 `0 Error(s)`。

用法（在 bcar 目录下）：
    python Tools\\build.py
返回码 0 = 真的编过了；非 0 = 失败（把错误行打出来，最多 20 行）。
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BCAR = os.path.dirname(HERE)
MDK = os.path.join(BCAR, "MDK-ARM")
LOG = os.path.join(MDK, "build.log")
UV4 = r"D:\Keil_v5\UV4\UV4.exe"


def main():
    if not os.path.isfile(UV4):
        print("找不到 UV4：%s" % UV4)
        return 2
    if os.path.exists(LOG):
        os.remove(LOG)                      # ★先清空，杜绝"旧的成功行"骗人
    subprocess.run([UV4, "-b", "bcar.uvprojx", "-j0", "-o", "build.log"],
                   cwd=MDK, capture_output=True, text=True)
    if not os.path.exists(LOG):
        print("UV4 没产出 build.log —— 工程没打开？")
        return 2

    text = open(LOG, encoding="utf-8", errors="replace").read()
    # 报错行：Keil 是 "xxx.c(12): error:  #8: ..."；同时兜住别的格式
    errs = [l for l in text.splitlines() if re.search(r"error(\s|:)", l, re.I)]
    warn = re.search(r"(\d+) Warning\(s\)", text)
    size = re.search(r"Program Size: (.+)", text)
    ok = ("0 Error(s), 0 Warning(s)" in text) and not errs

    print("编译结果：%s" % ("通过（0 error 0 warning）" if ok else "★失败"))
    if size:
        print("  " + size.group(1).strip())
    if warn:
        print("  警告数：%s" % warn.group(1))
    for l in errs[:20]:
        print("  " + l.strip())
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
