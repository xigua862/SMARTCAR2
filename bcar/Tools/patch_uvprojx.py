#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
把 B 车自建的 src/ 下各层（bsp, drivers, 以后的 middleware/control/app）
挂进 Keil 工程 bcar.uvprojx。

为什么需要这个脚本：
  CubeMX 重新生成工程时只列它自己生成的文件，我们手写的会被挤掉。
  所以每次重新生成工程后跑一遍本脚本。

★为什么源码放在 src/ 而不是工程根目录的 bsp/ drivers/：
  Windows 不区分大小写，`drivers/` 会和 CubeMX 的 `Drivers/`（HAL 库）
  撞成同一个目录 —— 一来文件混进 HAL 里，二来 CubeMX 重新生成时
  `DeletePrevious=true` 会清空 Drivers/，把我们写的驱动一起删掉。
  放进 src/ 就彻底隔离。

用法（在 bcar/MDK-ARM 目录下）：
    python ..\\Tools\\patch_uvprojx.py
幂等：本脚本管理的组每次都会先删后加，不会重复、也不会留下指向已删文件的死组。
"""
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.join(HERE, "..", "MDK-ARM", "bcar.uvprojx")

# 本脚本管理的源码目录（相对 bcar/），组名就用这个相对路径
SRC_DIRS = ["src/bsp", "src/drivers", "src/middleware", "src/control", "src/app"]

# 额外头文件搜索路径（相对 MDK-ARM/）
EXTRA_INC = ["../src/bsp", "../src/drivers", "../src/middleware",
             "../src/control", "../src/app", "../Core/Inc"]

# 历史遗留：曾经挂在根目录的组名，见到就删（避免指向不存在的文件）
LEGACY_GROUPS = ["bsp", "drivers", "middleware", "control", "app"]


def collect(bcar, rel_dir):
    d = os.path.join(bcar, rel_dir)
    if not os.path.isdir(d):
        return []
    out = []
    for name in sorted(os.listdir(d)):
        if name.lower().endswith((".c", ".s")):
            out.append((name, "../%s/%s" % (rel_dir, name)))
    return out


def drop_group(text, name):
    """删掉名为 name 的整个 <Group> 块（含其 Files）。"""
    pattern = re.compile(
        r"[ \t]*<Group>\s*<GroupName>%s</GroupName>.*?</Group>\s*\n" % re.escape(name),
        re.S,
    )
    return pattern.sub("", text, count=1)


def main():
    proj = os.path.abspath(PROJ)
    bcar = os.path.dirname(os.path.dirname(proj))
    if not os.path.isfile(proj):
        sys.exit("找不到工程文件：%s" % proj)

    t = io.open(proj, encoding="utf-8", errors="replace").read()
    report = []

    # 1) 头文件搜索路径
    m = re.search(r"<IncludePath>([^<]*)</IncludePath>", t)
    if not m:
        sys.exit("工程里没有 <IncludePath>，Keil 工程格式可能变了")
    cur = [p for p in m.group(1).split(";") if p]
    added_inc = [p for p in EXTRA_INC if p not in cur]
    if added_inc:
        t = t[:m.start()] + "<IncludePath>%s</IncludePath>" % ";".join(cur + added_inc) + t[m.end():]

    # 2) 先删旧组（含历史遗留名），保证幂等且不留死引用
    removed = []
    for name in LEGACY_GROUPS + SRC_DIRS:
        if ("<GroupName>%s</GroupName>" % name) in t:
            t = drop_group(t, name)
            removed.append(name)

    # 3) 再按磁盘现状重建
    groups_xml = ""
    added = []
    for rel in SRC_DIRS:
        files = collect(bcar, rel)
        if not files:
            continue
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
        added.append("%s(%d)" % (rel, len(files)))

    if groups_xml:
        if "    </Groups>" not in t:
            sys.exit("工程里找不到 </Groups>，格式可能变了")
        t = t.replace("    </Groups>", groups_xml + "    </Groups>", 1)

    io.open(proj, "w", encoding="utf-8", newline="\n").write(t)

    print("IncludePath 新增：%s" % (added_inc if added_inc else "无"))
    print("删除旧组：%s" % (removed if removed else "无"))
    print("重建文件组：%s" % (", ".join(added) if added else "无"))


if __name__ == "__main__":
    main()
