#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
在电脑上跑全部单元测试（不需要板子、不需要 Keil）。

用法（在 bcar 目录下）：
    python Tools\\run_pc_tests.py
返回码 0 = 全过；非 0 = 有失败（方便以后接 CI 或 pre-commit）。

为什么要留在工程里：命令行解析和里程/航向解算是最容易改坏又最难上板查的两层，
每次动它们先跑一遍，比烧进去试快一个数量级。
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BCAR = os.path.dirname(HERE)

# 需要 gcc 在 PATH 里；没有就用 w64devkit 的默认路径
GCC_CANDIDATES = [
    "gcc",
    r"D:\VScode\w64devkit\bin\gcc.exe",
    r"C:\w64devkit\bin\gcc.exe",
]

TESTS = [
    # (名字, 源码, 额外源码列表, 头文件目录, 是否需要 -lm)
    ("str_util 命令行解析", "Tools/test_parse.c",   ["src/app/str_util.c"],      "src/app",        False),
    ("odom 里程与位姿",     "Tools/test_odom.c",    ["src/middleware/odom.c"],   "src/middleware", True),
    ("heading 陀螺航向",    "Tools/test_heading.c", ["src/middleware/heading.c"], "src/middleware", True),
    ("line_pos 灰度质心",   "Tools/test_line_pos.c", ["src/middleware/line_pos.c"], "src/middleware", True),
    ("p2quant P²分位",      "Tools/test_p2quant.c",  ["src/middleware/p2quant.c"],  "src/middleware", True),
    ("control 控制环(循迹+直行)",  "Tools/test_control.c", ["src/control/angle_pd.c",
                                                      "src/control/straight.c",
                                                      "src/control/speed_pi.c",
                                                      "src/control/mixer.c"],   "src/control",   True),
    ("lapstat 单圈统计",    "Tools/test_lapstat.c",  ["src/middleware/lapstat.c",
                                                      "src/middleware/p2quant.c"],  "src/middleware", True),
    ("launch 发车读秒",     "Tools/test_launch.c",   ["src/app/launch.c"],        "src/app",        False),
    ("speed_pi 速度环",      "Tools/test_speed_pi.c", ["src/control/speed_pi.c"],  "src/control",    True),
    ("zebra 斑马线检测",    "Tools/test_zebra.c",    ["src/middleware/zebra.c",
                                                      "src/middleware/line_pos.c"], "src/middleware", True),
]


def find_gcc():
    for c in GCC_CANDIDATES:
        try:
            subprocess.run([c, "--version"], stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, check=True)
            return c
        except Exception:
            continue
    return None


def main():
    gcc = find_gcc()
    if gcc is None:
        print("找不到 gcc，请先装一个 PC 版 gcc（或改本脚本里的路径）")
        return 2

    failed = 0
    for name, test_src, extra, inc, need_math in TESTS:
        exe = os.path.join(BCAR, "build_" + os.path.basename(test_src).replace(".c", ".exe"))
        cmd = [gcc, "-Wall", "-Wextra", "-I", inc, "-o", exe,
               os.path.join(BCAR, test_src)] + [os.path.join(BCAR, e) for e in extra]
        if need_math:
            cmd.append("-lm")

        print("=" * 60)
        print("[编译] " + name)
        r = subprocess.run(cmd, cwd=BCAR, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout + r.stderr)
            failed += 1
            continue
        if r.stderr.strip():
            print("  (编译警告)\n" + r.stderr.strip())

        print("[运行] " + name)
        r2 = subprocess.run([exe], cwd=BCAR, capture_output=True, text=True)
        print(r2.stdout.rstrip())
        if r2.returncode != 0:
            failed += 1

    print("=" * 60)
    if failed:
        print("结果：%d 组失败" % failed)
        return 1
    print("结果：全部通过 ✓")
    return 0


if __name__ == "__main__":
    sys.exit(main())
