"""Parse 地图.DXF and draw wall lines."""

from pathlib import Path

import matplotlib.pyplot as plt

# Windows 中文字体，避免标题/图例缺字
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "SimSun", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

DXF_PATH = Path(__file__).with_name("地图.DXF")
OUT_PATH = Path(__file__).with_name("地图预览.png")


def parse_dxf_lines(path: Path):
    with path.open("r", encoding="utf-8") as f:
        raw = [ln.rstrip("\n\r") for ln in f]

    pairs = []
    i = 0
    while i + 1 < len(raw):
        pairs.append((raw[i].strip(), raw[i + 1].strip()))
        i += 2

    section = None
    waiting = False
    in_entities = False
    current_type = None
    current = {}
    lines = []

    def flush():
        nonlocal current_type, current
        if current_type == "LINE":
            lines.append(
                (
                    float(current.get("10", 0)),
                    float(current.get("20", 0)),
                    float(current.get("11", 0)),
                    float(current.get("21", 0)),
                )
            )
        current_type = None
        current = {}

    for code, val in pairs:
        if code == "0" and val == "SECTION":
            flush()
            waiting = True
            continue
        if waiting and code == "2":
            section = val
            waiting = False
            in_entities = section == "ENTITIES"
            continue
        if code == "0" and val == "ENDSEC":
            flush()
            section = None
            in_entities = False
            continue
        if not in_entities:
            continue
        if code == "0":
            flush()
            current_type = val
            current = {}
            continue
        if current_type and code not in current:
            current[code] = val
    flush()
    return lines


def main():
    segs = parse_dxf_lines(DXF_PATH)
    xs = [v for s in segs for v in (s[0], s[2])]
    ys = [v for s in segs for v in (s[1], s[3])]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)

    fig, ax = plt.subplots(figsize=(14, 10), dpi=140)
    fig.patch.set_facecolor("#0f1419")
    ax.set_facecolor("#151b22")

    for x1, y1, x2, y2 in segs:
        ax.plot(
            [x1, x2],
            [y1, y2],
            color="#7ec8e3",
            linewidth=1.8,
            solid_capstyle="round",
            zorder=3,
        )

    ax.set_aspect("equal", adjustable="box")
    pad_x = (xmax - xmin) * 0.05
    pad_y = (ymax - ymin) * 0.05
    ax.set_xlim(xmin - pad_x, xmax + pad_x)
    ax.set_ylim(ymin - pad_y, ymax + pad_y)

    ax.set_xlabel("X (mm)", color="#c8d0d8", fontsize=11)
    ax.set_ylabel("Y (mm)", color="#c8d0d8", fontsize=11)
    ax.set_title(
        f"地图.DXF  —  {len(segs)} 条线段   "
        f"范围 X[{xmin:.0f}, {xmax:.0f}]  Y[{ymin:.0f}, {ymax:.0f}] mm",
        color="#e8eef4",
        fontsize=13,
        pad=12,
    )

    ax.grid(True, color="#2a3440", linestyle="--", linewidth=0.6, zorder=0)
    ax.tick_params(colors="#9aa7b5")
    for spine in ax.spines.values():
        spine.set_color("#3a4654")

    info = (
        f"图元: LINE × {len(segs)}\n"
        f"图层: 0\n"
        f"单位: mm\n"
        f"尺寸: {xmax - xmin:.0f} × {ymax - ymin:.0f}"
    )
    ax.text(
        0.02,
        0.98,
        info,
        transform=ax.transAxes,
        va="top",
        ha="left",
        color="#d5dee6",
        fontsize=9,
        bbox=dict(
            boxstyle="round,pad=0.45",
            facecolor="#1c242e",
            edgecolor="#3a4654",
            alpha=0.92,
        ),
        zorder=4,
    )

    fig.tight_layout()
    fig.savefig(OUT_PATH, dpi=140, facecolor=fig.get_facecolor())
    print(f"saved: {OUT_PATH}")
    print(f"segments: {len(segs)}")
    plt.show()


if __name__ == "__main__":
    main()
