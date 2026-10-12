#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
把 B 车自建的 bsp/ drivers/ （以后再加 middleware/ control/ app/）挂进 Keil 工程。

为什么需要这个脚本：
  CubeMX 重新生成工程时，只会把它自己生成的文件列进 bcar.uvprojx，
  我们手写的 bsp/drivers/... 会被挤掉。所以每次重新生成后跑一遍本脚本。

用法（在 bcar/MDK-ARM 目录下）：
    python ..\\Tools\\patch_uvprojx.py
幂等：已经在工程里的组不会重复添加。
"""
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.join(HERE, "..", "MDK-ARM", "bcar.uvprojx")

# 要挂进去的目录（相对 bcar/）：每个目录 = 一个组，组名取目录名
SRC_DIRS = ["bsp", "drivers", "middleware", "control", "app"]

# 额外的头文件搜索路径（相对 MDK-ARM/）
EXTRA_INC = ["../bsp", "../drivers", "../middleware", "../control", "../app",
             "../Core/Inc"]


def collect(base_real, rel_dir):
    d = os.path.join(base_real, rel_dir)
    if not os.path.isdir(d):
        return []
    out = []
    for name in sorted(os.listdir(d)):
        if name.lower().endswith((".c", ".s")):
            out.append((name, "../%s/%s" % (rel_dir, name)))
    return out


def main():
    proj = os.path.abspath(PROJ)
    bcar = os.path.dirname(os.path.dirname(proj))
    if not os.path.isfile(proj):
        sys.exit("找不到工程文件：%s" % proj)

    t = io.open(proj, encoding="utf-8", errors="replace").read()

    # 1) 头文件搜索路径
    m = re.search(r"<IncludePath>([^<]*)</IncludePath>", t)
    if not m:
        sys.exit("工程里没有 <IncludePath>，格式可能变了")
    cur = [p for p in m.group(1).split(";") if p]
    added_inc = [p for p in EXTRA_INC if p not in cur]
    if added_inc:
        new = ";".join(cur + added_inc)
        t = t[:m.start()] + "<IncludePath>%s</IncludePath>" % new + t[m.end():]

    # 2) 文件组
    groups_xml = ""
    added_groups = []
    for rel in SRC_DIRS:
        files = collect(bcar, rel)
        if not files:
            continue
        if ("<GroupName>%s</GroupName>" % rel) in t:
            continue  # 已经有了，跳过
        body = ""
        for name, relpath in files:
            body += (
                "        <File>\n"
                "          <FileName>%s</FileName>\n"
                "          <FileType>1</FileType>\n"
                "          <FilePath>%s</FilePath>\n"
                "        </File>\n" % (name, relpath)
            )
        groups_xml += (
            "      <Group>\n"
            "        <GroupName>%s</GroupName>\n"
            "        <Files>\n%s        </Files>\n"
            "      </Group>\n" % (rel, body)
        )
        added_groups.append("%s(%d)" % (rel, len(files)))

    if groups_xml:
        t = t.replace("    </Groups>", groups_xml + "    </Groups>", 1)

    if added_inc or groups_xml:
        io.open(proj, "w", encoding="utf-8", newline="\n").write(t)

    print("IncludePath 新增：%s" % (added_inc if added_inc else "无"))
    print("新增文件组：%s" % (", ".join(added_groups) if added_groups else "无"))


if __name__ == "__main__":
    main()
