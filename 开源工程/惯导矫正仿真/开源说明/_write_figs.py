# -*- coding: utf-8 -*-
"""重画开源说明里的 SVG。"""
from pathlib import Path

OUT = Path(__file__).resolve().parent / "图示"
OUT.mkdir(parents=True, exist_ok=True)

HEAD = (
    '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" '
    'font-family="Microsoft YaHei, SimHei, PingFang SC, sans-serif">\n'
    '<rect width="{w}" height="{h}" fill="#fafbfc"/>\n'
)
FOOT = "</svg>\n"
ARROWS = """<defs>
  <marker id="arr" markerWidth="8" markerHeight="8" refX="6" refY="4" orient="auto">
    <path d="M0,0 L8,4 L0,8 Z" fill="#1a2330"/>
  </marker>
  <marker id="arrG" markerWidth="8" markerHeight="8" refX="6" refY="4" orient="auto">
    <path d="M0,0 L8,4 L0,8 Z" fill="#2a6f4e"/>
  </marker>
  <marker id="arrB" markerWidth="8" markerHeight="8" refX="6" refY="4" orient="auto">
    <path d="M0,0 L8,4 L0,8 Z" fill="#2a6aa8"/>
  </marker>
  <marker id="arrO" markerWidth="8" markerHeight="8" refX="6" refY="4" orient="auto">
    <path d="M0,0 L8,4 L0,8 Z" fill="#b45a18"/>
  </marker>
</defs>
"""


def title(text, w, y=32):
    return (
        f'<text x="{w/2:.0f}" y="{y}" text-anchor="middle" '
        f'font-size="18" font-weight="700" fill="#1a2330">{text}</text>\n'
    )


figs = {}

figs["01-两圈架构.svg"] = HEAD.format(w=920, h=340) + title("第一圈把走过的点记下来，后面按这张表跑", 920) + r"""
<rect x="40" y="58" width="390" height="250" rx="10" fill="#fff" stroke="#d7dee6"/>
<text x="235" y="86" text-anchor="middle" font-size="16" font-weight="700" fill="#2a6f4e">建图圈 · 贴线 + 记账</text>
<path d="M90 240 L90 140 L200 140 L200 200 L310 200 L310 120 L390 120" fill="none" stroke="#c9a227" stroke-width="4"/>
<g fill="#1f6f78">
  <circle cx="90" cy="240" r="5"/><circle cx="90" cy="190" r="5"/>
  <circle cx="90" cy="140" r="5"/><circle cx="145" cy="140" r="5"/>
  <circle cx="200" cy="140" r="5"/><circle cx="200" cy="170" r="5"/>
  <circle cx="200" cy="200" r="5"/><circle cx="255" cy="200" r="5"/>
  <circle cx="310" cy="200" r="5"/><circle cx="310" cy="160" r="5"/>
  <circle cx="310" cy="120" r="5"/><circle cx="350" cy="120" r="5"/>
</g>
<text x="102" y="252" font-size="12" fill="#556">(0, 0) 起点</text>
<text x="235" y="278" text-anchor="middle" font-size="13" fill="#334">每走一小段，记下当前惯导坐标</text>
<text x="235" y="298" text-anchor="middle" font-size="12" fill="#667">得到 pts[0], pts[1], pts[2] …</text>
<polygon points="450,180 478,168 478,192" fill="#3d8fd1"/>
<rect x="500" y="58" width="390" height="250" rx="10" fill="#fff" stroke="#d7dee6"/>
<text x="695" y="86" text-anchor="middle" font-size="16" font-weight="700" fill="#1f6f78">正式跑 · 跟表</text>
<path d="M550 240 L550 140 L660 140 L660 200 L770 200 L770 120 L850 120" fill="none" stroke="#2aa89a" stroke-width="4" stroke-dasharray="7 4"/>
<path d="M562 248 L555 155 L668 128 L675 208 L778 188 L790 128 L848 128" fill="none" stroke="#e07a3d" stroke-width="2.4"/>
<circle cx="660" cy="140" r="6" fill="#2aa89a"/>
<rect x="644" y="154" width="24" height="14" rx="2" fill="#ffb347" stroke="#8a5a20" transform="rotate(-14 656 161)"/>
<text x="695" y="278" text-anchor="middle" font-size="13" fill="#334">转向看惯导点在路表左边还是右边</text>
<text x="695" y="298" text-anchor="middle" font-size="12" fill="#667">惯导歪了，真车就会被打下线</text>
""" + FOOT

figs["02-坐标系与车体.svg"] = HEAD.format(w=920, h=400) + title("起点 (0,0)，朝上 yaw=90°，左边为正", 920) + ARROWS + r"""
<g transform="translate(90,330)">
  <line x1="0" y1="0" x2="240" y2="0" stroke="#1a2330" stroke-width="2" marker-end="url(#arr)"/>
  <line x1="0" y1="0" x2="0" y2="-240" stroke="#1a2330" stroke-width="2" marker-end="url(#arr)"/>
  <text x="248" y="6" font-size="14" fill="#1a2330">X</text>
  <text x="-10" y="-248" text-anchor="end" font-size="14" fill="#1a2330">Y</text>
  <circle cx="0" cy="0" r="6" fill="#1a2330"/>
  <text x="12" y="22" font-size="14" fill="#334">起点 (0, 0)</text>
  <path d="M0 0 L 0 -180" stroke="#c9a227" stroke-width="5"/>
  <polygon points="-6,-176 0,-192 6,-176" fill="#c9a227"/>
  <text x="14" y="-160" font-size="13" fill="#8a7010">出发朝 +Y</text>
  <text x="14" y="-140" font-size="13" fill="#8a7010">yaw = 90°</text>
</g>
<g transform="translate(620,210)">
  <rect x="-36" y="-20" width="72" height="40" rx="6" fill="#ffb347" stroke="#8a5a20" stroke-width="2"/>
  <polygon points="36,-9 52,0 36,9" fill="#fff3d6" stroke="#8a5a20"/>
  <line x1="0" y1="0" x2="110" y2="0" stroke="#2a6aa8" stroke-width="2.2" marker-end="url(#arrB)"/>
  <text x="116" y="-8" font-size="14" fill="#2a6aa8">车头 = yaw</text>
  <line x1="0" y1="0" x2="0" y2="-80" stroke="#2aa89a" stroke-width="2"/>
  <text x="10" y="-84" font-size="14" fill="#1f6f78">车体左，误差为正</text>
  <line x1="110" y1="-36" x2="110" y2="36" stroke="#4a90d9" stroke-width="5" stroke-linecap="round"/>
  <text x="122" y="6" font-size="13" fill="#1a4f8a">车头前一排传感器</text>
</g>
<text x="460" y="375" text-anchor="middle" font-size="13" fill="#445">线在左边为正，路点在车左边也为正。两套环同号。</text>
""" + FOOT

# 03：用数字走出前几个点
figs["03-航迹推算.svg"] = HEAD.format(w=960, h=500) + title("举例：从 (0,0) 朝上走出前几个点", 960) + ARROWS + r"""
<g transform="translate(70,430)">
  <line x1="0" y1="0" x2="280" y2="0" stroke="#889" stroke-width="1.4" marker-end="url(#arr)"/>
  <line x1="0" y1="0" x2="0" y2="-360" stroke="#889" stroke-width="1.4" marker-end="url(#arr)"/>
  <text x="286" y="6" font-size="13" fill="#556">X (mm)</text>
  <text x="-8" y="-368" text-anchor="end" font-size="13" fill="#556">Y (mm)</text>
  <path d="M0 0 L0 -80 L0 -160 L0 -240 L80 -240 L160 -240" fill="none" stroke="#c9a227" stroke-width="3"/>
  <g fill="#1f6f78" font-size="12">
    <circle cx="0" cy="0" r="6"/><text x="10" y="16" fill="#334">(0, 0) 起点</text>
    <circle cx="0" cy="-80" r="6"/><text x="12" y="-74" fill="#1f6f78">(0, 2)</text>
    <circle cx="0" cy="-160" r="6"/><text x="12" y="-154" fill="#1f6f78">(0, 4)</text>
    <circle cx="0" cy="-240" r="6"/><text x="12" y="-248" fill="#1f6f78">(0, 6)</text>
    <circle cx="80" cy="-240" r="6"/><text x="70" y="-252" text-anchor="end" fill="#1f6f78">(2, 6)</text>
    <circle cx="160" cy="-240" r="6"/><text x="150" y="-252" text-anchor="end" fill="#1f6f78">(4, 6)</text>
  </g>
  <text x="16" y="-40" font-size="12" fill="#b45a18">yaw=90°</text>
  <text x="90" y="-220" font-size="12" fill="#b45a18">yaw=0° 右转</text>
</g>
<rect x="430" y="70" width="500" height="390" rx="10" fill="#fff" stroke="#d7dee6"/>
<text x="450" y="102" font-size="15" font-weight="700" fill="#1a2330">这一小段编码器报：走了 2 mm</text>
<text x="450" y="128" font-size="15" font-weight="700" fill="#1a2330">陀螺报：航向 90°（朝上）</text>
<text x="450" y="168" font-size="14" fill="#334">x = 0 + 2 × cos(90°) = 0</text>
<text x="450" y="194" font-size="14" fill="#334">y = 0 + 2 × sin(90°) = 2</text>
<text x="450" y="230" font-size="14" fill="#1f6f78">得到第一个点 (0, 2)</text>
<text x="450" y="266" font-size="14" fill="#334">再走 2 mm，还是朝上 → (0, 4)</text>
<text x="450" y="292" font-size="14" fill="#334">再走 2 mm → (0, 6)</text>
<text x="450" y="328" font-size="14" fill="#334">右转后 yaw=0°，cos(0°)=1</text>
<text x="450" y="354" font-size="14" fill="#334">x 开始增加：(2, 6)、(4, 6) …</text>
<text x="450" y="400" font-size="13" fill="#b45a18">没有人告诉你坐标。全是自己加出来的。</text>
<text x="450" y="426" font-size="13" fill="#556">yaw 是方向，(x,y) 是位置。先有方向，才有下一步的位置。</text>
""" + FOOT

# 04：数组怎么来的
figs["04-建图记表.svg"] = HEAD.format(w=960, h=480) + title("建图：把积出来的点塞进数组，这就是路表", 960) + r"""
<path d="M70 400 L70 280 L70 160 L190 160 L310 160" fill="none" stroke="#c9a227" stroke-width="4"/>
<g fill="#1f6f78" font-size="12">
  <circle cx="70" cy="400" r="7"/><text x="82" y="416" fill="#334">0</text>
  <circle cx="70" cy="340" r="7"/><text x="82" y="336" fill="#1f6f78">1</text>
  <circle cx="70" cy="280" r="7"/><text x="82" y="276" fill="#1f6f78">2</text>
  <circle cx="70" cy="220" r="7"/><text x="82" y="216" fill="#1f6f78">3</text>
  <circle cx="70" cy="160" r="7"/><text x="82" y="156" fill="#1f6f78">4</text>
  <circle cx="130" cy="160" r="7"/><text x="122" y="146" fill="#1f6f78">5</text>
  <circle cx="190" cy="160" r="7"/><text x="182" y="146" fill="#1f6f78">6</text>
  <circle cx="250" cy="160" r="7"/><text x="242" y="146" fill="#1f6f78">7</text>
  <circle cx="310" cy="160" r="7"/><text x="302" y="146" fill="#1f6f78">8</text>
</g>
<text x="70" y="444" text-anchor="middle" font-size="13" fill="#556">赛道上实际开过的位置</text>
<rect x="400" y="70" width="520" height="360" rx="10" fill="#fff" stroke="#d7dee6"/>
<text x="420" y="102" font-size="15" font-weight="700" fill="#1a2330">MCU 里就是这样一张表</text>
<text x="420" y="140" font-family="Consolas, Microsoft YaHei, monospace" font-size="15" fill="#1a2330">pts[0] = ( 0.0,  0.0)    走过  0 mm</text>
<text x="420" y="168" font-family="Consolas, Microsoft YaHei, monospace" font-size="15" fill="#1a2330">pts[1] = ( 0.0,  2.0)    走过  2 mm</text>
<text x="420" y="196" font-family="Consolas, Microsoft YaHei, monospace" font-size="15" fill="#1a2330">pts[2] = ( 0.0,  4.0)    走过  4 mm</text>
<text x="420" y="224" font-family="Consolas, Microsoft YaHei, monospace" font-size="15" fill="#1a2330">pts[3] = ( 0.0,  6.0)    走过  6 mm</text>
<text x="420" y="252" font-family="Consolas, Microsoft YaHei, monospace" font-size="15" fill="#1a2330">pts[4] = ( 0.0,  8.0)    走过  8 mm</text>
<text x="420" y="280" font-family="Consolas, Microsoft YaHei, monospace" font-size="15" fill="#1a2330">pts[5] = ( 2.0,  8.0)    走过 10 mm</text>
<text x="420" y="308" font-family="Consolas, Microsoft YaHei, monospace" font-size="15" fill="#1a2330">pts[6] = ( 4.0,  8.0)    …</text>
<text x="420" y="350" font-size="14" fill="#1f6f78">序号 i 和路程锁死：走了 10 mm 就用 pts[5]</text>
<text x="420" y="378" font-size="13" fill="#556">正式跑用编码器读数当进度条，不要搜最近点。</text>
<text x="420" y="404" font-size="13" fill="#556">过起点搜最近会跳到别的点上去。</text>
""" + FOOT

# 05：yaw错了点往哪偏
figs["05-角漂成因.svg"] = HEAD.format(w=960, h=460) + title("yaw 错一点，每一步都往错的方向加", 960) + ARROWS + r"""
<line x1="80" y1="400" x2="80" y2="70" stroke="#c9a227" stroke-width="8"/>
<text x="96" y="86" font-size="13" fill="#8a7010">路表 / 该走的直道  x = 0</text>
<g fill="#1f6f78">
  <circle cx="80" cy="400" r="5"/><circle cx="80" cy="340" r="5"/>
  <circle cx="80" cy="280" r="5"/><circle cx="80" cy="220" r="5"/>
  <circle cx="80" cy="160" r="5"/><circle cx="80" cy="100" r="5"/>
</g>
<text x="92" y="404" font-size="12" fill="#334">(0, 0)</text>
<path d="M80 400 L 115 340 L 150 280 L 185 220 L 220 160 L 255 100" fill="none" stroke="#e07a3d" stroke-width="3"/>
<g fill="#e07a3d">
  <circle cx="115" cy="340" r="5"/><circle cx="150" cy="280" r="5"/>
  <circle cx="185" cy="220" r="5"/><circle cx="220" cy="160" r="5"/>
  <circle cx="255" cy="100" r="5"/>
</g>
<text x="268" y="104" font-size="13" fill="#b45a18">惯导点越积越偏</text>
<line x1="80" y1="100" x2="255" y2="100" stroke="#1a2330" stroke-width="1.4" stroke-dasharray="4 3"/>
<text x="120" y="88" font-size="13" fill="#1a2330">走 1 m 后的侧偏</text>
<rect x="430" y="80" width="490" height="330" rx="10" fill="#fff" stroke="#d7dee6"/>
<text x="450" y="116" font-size="15" font-weight="700" fill="#1a2330">该朝 90°，惯导以为 85°</text>
<text x="450" y="156" font-size="14" fill="#334">每一步 2 mm：</text>
<text x="450" y="186" font-size="14" fill="#2a6f4e">该加  Δx=0，    Δy=2.00</text>
<text x="450" y="214" font-size="14" fill="#b45a18">实加  Δx=0.17， Δy=1.99</text>
<text x="450" y="254" font-size="14" fill="#334">现在的 (x,y) 不会跳开。</text>
<text x="450" y="282" font-size="14" fill="#334">是之后每一小步都斜着加，</text>
<text x="450" y="310" font-size="14" fill="#334">走得越远，偏得越多。</text>
<text x="450" y="354" font-size="14" fill="#1a2330">侧偏 ≈ 路程 × 角度误差（弧度）</text>
<text x="450" y="382" font-size="13" fill="#556">0.5° 走 2 m ≈ 17 mm。零偏不除，下一圈接着转。</text>
""" + FOOT

figs["06-编码误差.svg"] = HEAD.format(w=920, h=380) + title("尺子长了：方向可能还对，进度条超前了", 920) + r"""
<line x1="70" y1="150" x2="850" y2="150" stroke="#2aa89a" stroke-width="6"/>
<g fill="#1f6f78" font-size="12">
  <circle cx="70" cy="150" r="6"/><text x="70" y="128" text-anchor="middle">0</text>
  <circle cx="200" cy="150" r="6"/><text x="200" y="128" text-anchor="middle">1</text>
  <circle cx="330" cy="150" r="6"/><text x="330" y="128" text-anchor="middle">2</text>
  <circle cx="460" cy="150" r="6"/><text x="460" y="128" text-anchor="middle">3</text>
  <circle cx="590" cy="150" r="6"/><text x="590" y="128" text-anchor="middle">4</text>
  <circle cx="720" cy="150" r="6"/><text x="720" y="128" text-anchor="middle">5</text>
  <circle cx="850" cy="150" r="6"/><text x="850" y="128" text-anchor="middle">6 弯</text>
</g>
<rect x="444" y="168" width="28" height="16" rx="3" fill="#ffb347" stroke="#8a5a20"/>
<text x="458" y="210" text-anchor="middle" font-size="13" fill="#b45a18">车实际在 pts[3]</text>
<text x="720" y="210" text-anchor="middle" font-size="13" fill="#1a4f8a">编码器却报到 pts[5]</text>
<path d="M458 184 L458 230" stroke="#b45a18" stroke-width="1.6"/>
<path d="M720 156 L720 230" stroke="#1a4f8a" stroke-width="1.6"/>
<path d="M458 240 H720" stroke="#1a2330" stroke-width="1.6"/>
<text x="589" y="262" text-anchor="middle" font-size="14" fill="#1a2330">超前了两格：提前按弯减速、提前打方向</text>
<rect x="70" y="290" width="780" height="64" rx="8" fill="#fff" stroke="#d7dee6"/>
<text x="90" y="318" font-size="14" fill="#1a2330">打滑、内外轮差会把编码器积长或积短。这和 yaw 是两回事。</text>
<text x="90" y="340" font-size="13" fill="#556">yaw 管下一步往哪走；编码器管表上取第几个点。两个都可能错。</text>
""" + FOOT

figs["07-光电测偏.svg"] = HEAD.format(w=920, h=380) + title("线传感器：线在车头前方偏左还是偏右", 920) + r"""
<rect x="360" y="60" width="20" height="250" rx="4" fill="#e8c547"/>
<text x="390" y="80" font-size="13" fill="#8a7010">赛道</text>
<g transform="translate(520,230)">
  <rect x="-28" y="-16" width="56" height="32" rx="5" fill="#ffb347" stroke="#8a5a20"/>
  <polygon points="28,-7 42,0 28,7" fill="#fff3d6" stroke="#8a5a20"/>
  <line x1="0" y1="0" x2="0" y2="-80" stroke="#4a90d9" stroke-width="2" stroke-dasharray="4 3"/>
  <line x1="-40" y1="-80" x2="40" y2="-80" stroke="#4a90d9" stroke-width="5" stroke-linecap="round"/>
  <g fill="#1a4f8a">
    <circle cx="-40" cy="-80" r="3.2"/><circle cx="-20" cy="-80" r="3.2"/>
    <circle cx="0" cy="-80" r="3.2"/><circle cx="20" cy="-80" r="3.2"/>
    <circle cx="40" cy="-80" r="3.2"/>
  </g>
  <circle cx="-20" cy="-80" r="8" fill="none" stroke="#2a6f4e" stroke-width="2"/>
  <text x="52" y="-76" font-size="13" fill="#1a4f8a">线打在左边</text>
</g>
<text x="200" y="150" font-size="14" fill="#2a6f4e">线在车左边</text>
<text x="200" y="174" font-size="14" fill="#2a6f4e">→ 真横向为正</text>
<rect x="60" y="290" width="800" height="64" rx="8" fill="#fff" stroke="#d7dee6"/>
<text x="80" y="318" font-size="14" fill="#1a2330">光电、电感、摄像头都行，只要给出「线在哪一侧、偏多少毫米」。</text>
<text x="80" y="340" font-size="13" fill="#556">测的是车头前面，不是车心。直道上差不多；弯里对不齐，所以弯里不用它修惯导。</text>
""" + FOOT

figs["08-直道观测.svg"] = HEAD.format(w=960, h=440) + title("直道上：线说车在哪，惯导说自己在哪，一减就是该拧的量", 960) + r"""
<line x1="80" y1="220" x2="420" y2="220" stroke="#c9a227" stroke-width="10"/>
<text x="80" y="200" font-size="13" fill="#8a7010">路表 / 赛道</text>
<rect x="168" y="208" width="40" height="24" rx="4" fill="#ffb347" stroke="#8a5a20"/>
<polygon points="208,214 222,220 208,226" fill="#fff3d6" stroke="#8a5a20"/>
<text x="188" y="258" text-anchor="middle" font-size="13" fill="#b45a18">真车在线上</text>
<text x="188" y="278" text-anchor="middle" font-size="12" fill="#b45a18">线传感器：偏了 0 mm</text>
<circle cx="188" cy="150" r="9" fill="#2aa89a"/>
<line x1="188" y1="159" x2="188" y2="208" stroke="#2aa89a" stroke-width="2" stroke-dasharray="3 3"/>
<text x="204" y="146" font-size="13" fill="#1f6f78">惯导点</text>
<text x="204" y="166" font-size="12" fill="#1f6f78">相对路表偏左 15 mm</text>
<rect x="480" y="70" width="440" height="200" rx="10" fill="#fff" stroke="#d7dee6"/>
<text x="500" y="106" font-size="16" font-weight="700" fill="#1a2330">该拧的量 = 15 − 0 = 15 mm</text>
<text x="500" y="142" font-size="14" fill="#334">惯导横向：点在路表左边 15 mm</text>
<text x="500" y="170" font-size="14" fill="#334">真横向：线传感器说 0 mm</text>
<text x="500" y="198" font-size="14" fill="#334">差 15 mm → 惯导点偏左了</text>
<text x="500" y="234" font-size="13" fill="#1f6f78">PID 看到「偏左」，把 yaw 往右拧。</text>
<rect x="80" y="320" width="840" height="90" rx="8" fill="#fff8ee" stroke="#e7c9a4"/>
<text x="100" y="350" font-size="14" fill="#1a2330">真车也偏了也没关系：两个数一减，剩的还是惯导相对真值的差。</text>
<text x="100" y="376" font-size="14" fill="#1a2330">这个差用来拧 yaw，不是用来打舵。舵仍然跟路表。</text>
<text x="100" y="396" font-size="13" fill="#556">只在长直道中间算。弯里两个「左」对不齐。</text>
""" + FOOT

# 09：核心图 yaw 和 xy 的关系
figs["09-航向补偿.svg"] = HEAD.format(w=960, h=520) + title("改 yaw 不改现在的点，只改下一步落在哪", 960) + ARROWS + r"""
<line x1="70" y1="80" x2="70" y2="300" stroke="#c9a227" stroke-width="8"/>
<text x="84" y="96" font-size="13" fill="#8a7010">路表 x = 0</text>
<circle cx="70" cy="280" r="5" fill="#bbb"/>
<circle cx="130" cy="220" r="9" fill="#2aa89a"/>
<text x="144" y="216" font-size="14" fill="#1f6f78">现在的惯导点 (x, y)</text>
<text x="144" y="236" font-size="13" fill="#1f6f78">已经积在这，不会被瞬移走</text>
<line x1="130" y1="220" x2="175" y2="100" stroke="#999" stroke-width="2.4" stroke-dasharray="6 4" marker-end="url(#arr)"/>
<text x="184" y="110" font-size="13" fill="#666">不修 yaw：下一步更偏左</text>
<line x1="130" y1="220" x2="88" y2="100" stroke="#2a6f4e" stroke-width="2.8" marker-end="url(#arrG)"/>
<text x="20" y="118" font-size="13" fill="#2a6f4e">拧回一点：下一步往路表靠</text>
<circle cx="88" cy="100" r="7" fill="#2a6f4e"/>
<circle cx="175" cy="100" r="6" fill="#999"/>
<rect x="430" y="70" width="500" height="240" rx="10" fill="#fff" stroke="#d7dee6"/>
<text x="450" y="104" font-size="15" font-weight="700" fill="#1a2330">下一步永远是这句话</text>
<text x="450" y="140" font-family="Consolas, Microsoft YaHei, monospace" font-size="16" fill="#1a2330">x += ds × cos(yaw)</text>
<text x="450" y="168" font-family="Consolas, Microsoft YaHei, monospace" font-size="16" fill="#1a2330">y += ds × sin(yaw)</text>
<text x="450" y="208" font-size="14" fill="#334">补偿加在 yaw 上 → cos/sin 变了</text>
<text x="450" y="236" font-size="14" fill="#334">→ 这一步的 Δx、Δy 变了</text>
<text x="450" y="264" font-size="14" fill="#334">→ 新的 (x, y) 落点变了</text>
<text x="450" y="292" font-size="13" fill="#b45a18">走一段直道，点才慢慢走回路表。</text>
<rect x="70" y="340" width="860" height="150" rx="10" fill="#fff8ee" stroke="#e7c9a4"/>
<text x="90" y="372" font-size="15" font-weight="700" fill="#1a2330">和打舵是两件事</text>
<text x="90" y="404" font-size="14" fill="#334">跟表环看见惯导点在路表左边 → 打右舵 → 真车也往右。</text>
<text x="90" y="430" font-size="14" fill="#334">若真车其实已经在线上、只是惯导点偏左，这一打，真车就被打下线。</text>
<text x="90" y="456" font-size="14" fill="#334">所以先拧 yaw，让惯导点自己走回真位置。跟表打的舵才是给真车用的。</text>
""" + FOOT

figs["10-弧长靠拢.svg"] = HEAD.format(w=920, h=360) + title("方向对了，还要把「表上走到哪」拨准", 920) + r"""
<line x1="80" y1="140" x2="840" y2="140" stroke="#2aa89a" stroke-width="5"/>
<g fill="#1f6f78">
  <circle cx="80" cy="140" r="5"/><circle cx="200" cy="140" r="5"/>
  <circle cx="320" cy="140" r="5"/><circle cx="440" cy="140" r="5"/>
  <circle cx="560" cy="140" r="5"/><circle cx="680" cy="140" r="5"/>
  <circle cx="800" cy="140" r="5"/>
</g>
<circle cx="440" cy="140" r="9" fill="#2aa89a"/>
<text x="440" y="118" text-anchor="middle" font-size="13" fill="#1f6f78">惯导点投在这 → 该用 pts[3]</text>
<circle cx="560" cy="200" r="8" fill="#4a90d9"/>
<text x="572" y="204" font-size="13" fill="#1a4f8a">编码器还在报 pts[4]</text>
<path d="M560 200 H452" stroke="#1a2330" stroke-width="2"/>
<polygon points="460,194 448,200 460,206" fill="#1a2330"/>
<text x="460" y="240" font-size="14" fill="#1a2330">把编码器读数慢慢拨回 pts[3]</text>
<rect x="80" y="270" width="760" height="64" rx="8" fill="#fff" stroke="#d7dee6"/>
<text x="100" y="298" font-size="14" fill="#1a2330">不要去估一个「尺子比例」再乘以后所有路程。弯里噪声会把比例污染。</text>
<text x="100" y="320" font-size="13" fill="#556">只拨当前读到第几毫米。差太大别拉，可能投到邻段去了。</text>
""" + FOOT

figs["11-直道分段.svg"] = HEAD.format(w=920, h=280) + title("只修长直道中间，弯里带着上次的补偿走", 920) + r"""
<path d="M50 140 H230" fill="none" stroke="#ff8a4c" stroke-width="12" stroke-linecap="round"/>
<path d="M230 140 H340" fill="none" stroke="#e6c35c" stroke-width="12" stroke-linecap="round"/>
<path d="M340 140 H580" fill="none" stroke="#4ee0a8" stroke-width="12" stroke-linecap="round"/>
<path d="M580 140 H690" fill="none" stroke="#e6c35c" stroke-width="12" stroke-linecap="round"/>
<path d="M690 140 C 780 140, 820 80, 880 70" fill="none" stroke="#ff8a4c" stroke-width="12" stroke-linecap="round"/>
<text x="140" y="180" text-anchor="middle" font-size="14" fill="#c45c20">弯 · 不修</text>
<text x="285" y="180" text-anchor="middle" font-size="14" fill="#8a7010">出入弯</text>
<text x="460" y="180" text-anchor="middle" font-size="14" fill="#1f6f78">直道中间 · 在这里拧 yaw</text>
<text x="635" y="180" text-anchor="middle" font-size="14" fill="#8a7010">出入弯</text>
<text x="800" y="108" text-anchor="middle" font-size="14" fill="#c45c20">弯 · 不修</text>
<text x="460" y="224" text-anchor="middle" font-size="13" fill="#334">路表航向几乎不变才算直。太短的直段丢掉，两端再裁一段。</text>
<text x="460" y="248" text-anchor="middle" font-size="13" fill="#334">弯里两个横向对不齐。补偿角过弯保持，下一条直道接着用。</text>
""" + FOOT

for name, xml in figs.items():
    path = OUT / name
    path.write_text(xml, encoding="utf-8")
    print("wrote", path.name, path.stat().st_size)
