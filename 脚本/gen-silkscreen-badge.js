// gen-silkscreen-badge.js
// 重新生成 "丝印-校徽融合机器人工程" 徽章的文字部分。
// 保留原 SVG 中所有非文字 path（齿轮环、双细圆环、电路走线、机器人图标），
// 用系统字体的真实字形轮廓沿上/下弧烘焙（不使用 transform 属性）生成文字 path。
//
// 依赖：opentype.js（读取字体轮廓）、sharp（渲染 PNG 预览）
//   npm install opentype.js sharp
//   （若依赖装在别处，可用 NODE_PATH 指向对应 node_modules 目录运行）
// 字体：
//   中文 -> C:\Windows\Fonts\simhei.ttf (黑体)
//   英文 -> C:\Windows\Fonts\arialbd.ttf (Arial Bold)
//
// 输出（写回 硬件工程/丝印/）：
//   丝印-校徽融合机器人工程.svg  （单色黑、纯 path、无 text/transform/stroke）
//   丝印-校徽融合机器人工程.png  （1000x1000 白底预览）
//   丝印-校徽融合机器人工程.dxf  （R12，嘉立创 EDA 导入备选；1 单位 = 0.03mm）

'use strict';
const fs = require('fs');
const path = require('path');
const opentype = require('opentype.js');
const sharp = require('sharp');

// ------------------------------------------------------------------ 配置
const DIR = 'c:\\Users\\asus\\Desktop\\test16\\硬件工程\\丝印';
const SVG_IN = path.join(DIR, '丝印-校徽融合机器人工程.svg');
const SVG_OUT = SVG_IN;
const PNG_OUT = path.join(DIR, '丝印-校徽融合机器人工程.png');
const DXF_OUT = path.join(DIR, '丝印-校徽融合机器人工程.dxf');

const CN_FONT = 'C:\\Windows\\Fonts\\simhei.ttf';
const EN_FONT = fs.existsSync('C:\\Windows\\Fonts\\arialbd.ttf')
  ? 'C:\\Windows\\Fonts\\arialbd.ttf'
  : 'C:\\Windows\\Fonts\\arial.ttf';

const CX = 500, CY = 500;      // 徽章中心
const R = 369;                 // 文字中心线半径（环带 332~406 正中，上下留余量）

const CN_TEXT = '西南石油大学';
const CN_SIZE = 68;
const CN_STEP = 16;            // 度，从 12 点方向顺时针为正，整组居中于顶部

const EN_TEXT = 'ROBOTICS ENGINEERING';
const EN_SIZE = 46;
const EN_STEP = 5.5;           // 度，从 6 点方向，左负右正，整组居中

const DXF_SCALE = 0.03;        // 1 viewBox 单位 = 0.03mm => 徽章直径 30mm
const FLATTEN_SEG = 24;        // 每段 C/Q/A 曲线采样段数

const f2 = (n) => (Math.round(n * 100) / 100).toString();

// ------------------------------------------------------------------ 几何工具
// 居中偏移：把字形 bbox 中心移到原点
function centerOffset(cmds) {
  let minx = Infinity, miny = Infinity, maxx = -Infinity, maxy = -Infinity;
  const acc = (x, y) => {
    if (x < minx) minx = x; if (x > maxx) maxx = x;
    if (y < miny) miny = y; if (y > maxy) maxy = y;
  };
  for (const c of cmds) {
    if (c.type === 'Z') continue;
    acc(c.x, c.y);
    if (c.type === 'C' || c.type === 'Q') acc(c.x1, c.y1);
    if (c.type === 'C') acc(c.x2, c.y2);
  }
  return { dx: -(minx + maxx) / 2, dy: -(miny + maxy) / 2 };
}

// 烘焙弧排变换：先平移居中，再旋转 alpha（度，SVG 顺时针为正），再平移到 (px,py)
function bake(cmds, dx, dy, alphaDeg, px, py) {
  const a = (alphaDeg * Math.PI) / 180;
  const ca = Math.cos(a), sa = Math.sin(a);
  const T = (x, y) => {
    const xc = x + dx, yc = y + dy;
    return [xc * ca - yc * sa + px, xc * sa + yc * ca + py];
  };
  return cmds.map((c) => {
    const n = { type: c.type };
    if (c.type === 'M' || c.type === 'L') {
      const [x, y] = T(c.x, c.y); n.x = x; n.y = y;
    } else if (c.type === 'C') {
      const [x1, y1] = T(c.x1, c.y1); const [x2, y2] = T(c.x2, c.y2); const [x, y] = T(c.x, c.y);
      n.x1 = x1; n.y1 = y1; n.x2 = x2; n.y2 = y2; n.x = x; n.y = y;
    } else if (c.type === 'Q') {
      const [x1, y1] = T(c.x1, c.y1); const [x, y] = T(c.x, c.y);
      n.x1 = x1; n.y1 = y1; n.x = x; n.y = y;
    }
    return n;
  });
}

function cmdsToData(cmds) {
  let d = '';
  for (const c of cmds) {
    if (c.type === 'M') d += `M${f2(c.x)},${f2(c.y)}`;
    else if (c.type === 'L') d += `L${f2(c.x)},${f2(c.y)}`;
    else if (c.type === 'C') d += `C${f2(c.x1)},${f2(c.y1)} ${f2(c.x2)},${f2(c.y2)} ${f2(c.x)},${f2(c.y)}`;
    else if (c.type === 'Q') d += `Q${f2(c.x1)},${f2(c.y1)} ${f2(c.x)},${f2(c.y)}`;
    else if (c.type === 'Z') d += 'Z';
  }
  return d;
}

// 生成一段弧排文字 -> path data 数组（每字符一条）
const RADIAL = { top: { min: Infinity, max: -Infinity }, bottom: { min: Infinity, max: -Infinity } };
function trackRadial(cmds, mode) {
  const st = RADIAL[mode];
  const acc = (x, y) => {
    const r = Math.hypot(x - CX, y - CY);
    if (r < st.min) st.min = r; if (r > st.max) st.max = r;
  };
  for (const c of cmds) {
    if (c.type === 'Z') continue;
    acc(c.x, c.y);
    if (c.type === 'C' || c.type === 'Q') acc(c.x1, c.y1);
    if (c.type === 'C') acc(c.x2, c.y2);
  }
}
function arcText(font, text, size, stepDeg, mode) {
  const n = text.length;
  const out = [];
  for (let i = 0; i < n; i++) {
    const ch = text[i];
    if (ch === ' ') continue;              // 空格占位但不绘制
    const off = (i - (n - 1) / 2) * stepDeg; // 整组居中
    let alpha, px, py;
    if (mode === 'top') {
      // 上弧：off = s（度，从 12 点顺时针）
      const s = off * Math.PI / 180;
      px = CX + R * Math.sin(s); py = CY - R * Math.cos(s); alpha = off;
    } else {
      // 下弧：off = t（度，从 6 点，左负右正）；alpha = -t 使字顶朝向圆心、正读
      const t = off * Math.PI / 180;
      px = CX + R * Math.sin(t); py = CY + R * Math.cos(t); alpha = -off;
    }
    const glyph = font.charToGlyph(ch);
    const p = glyph.getPath(0, 0, size);
    const { dx, dy } = centerOffset(p.commands);
    const baked = bake(p.commands, dx, dy, alpha, px, py);
    trackRadial(baked, mode);
    out.push(cmdsToData(baked));
  }
  return out;
}

// ------------------------------------------------------------------ SVG path 解析 / flatten（供 DXF 用）
function parsePath(d) {
  // 仅支持大写（绝对）命令：M L H V C Q A Z
  const re = /([MLHVCSQTAZ])([^MLHVCSQTAZ]*)/g;
  const segs = []; let m;
  while ((m = re.exec(d)) !== null) {
    const cmd = m[1];
    const nums = (m[2].match(/-?\d*\.?\d+(?:e[-+]?\d+)?/gi) || []).map(Number);
    segs.push({ cmd, nums });
  }
  return segs;
}

function svgArcToCenter(x1, y1, x2, y2, fa, fs, rx, ry, phiDeg) {
  const phi = phiDeg * Math.PI / 180, cosPhi = Math.cos(phi), sinPhi = Math.sin(phi);
  const dxh = (x1 - x2) / 2, dyh = (y1 - y2) / 2;
  const x1p = cosPhi * dxh + sinPhi * dyh;
  const y1p = -sinPhi * dxh + cosPhi * dyh;
  rx = Math.abs(rx); ry = Math.abs(ry);
  const x1p2 = x1p * x1p, y1p2 = y1p * y1p;
  const lambda = x1p2 / (rx * rx) + y1p2 / (ry * ry);
  if (lambda > 1) { const s = Math.sqrt(lambda); rx *= s; ry *= s; }
  const rx2 = rx * rx, ry2 = ry * ry;
  const num = rx2 * ry2 - rx2 * y1p2 - ry2 * x1p2;
  const den = rx2 * y1p2 + ry2 * x1p2;
  let co = den === 0 ? 0 : Math.sqrt(Math.max(0, num / den));
  if (fa === fs) co = -co;
  const cxp = co * (rx * y1p) / ry;
  const cyp = co * -(ry * x1p) / rx;
  const cx = cosPhi * cxp - sinPhi * cyp + (x1 + x2) / 2;
  const cy = sinPhi * cxp + cosPhi * cyp + (y1 + y2) / 2;
  const ang = (ux, uy, vx, vy) => {
    const nn = Math.sqrt(ux * ux + uy * uy) * Math.sqrt(vx * vx + vy * vy);
    let c = nn === 0 ? 1 : (ux * vx + uy * vy) / nn; c = Math.max(-1, Math.min(1, c));
    let a = Math.acos(c); if (ux * vy - uy * vx < 0) a = -a; return a;
  };
  const theta1 = ang(1, 0, (x1p - cxp) / rx, (y1p - cyp) / ry);
  let dtheta = ang((x1p - cxp) / rx, (y1p - cyp) / ry, (-x1p - cxp) / rx, (-y1p - cyp) / ry);
  if (fs === 0 && dtheta > 0) dtheta -= 2 * Math.PI;
  if (fs === 1 && dtheta < 0) dtheta += 2 * Math.PI;
  return { cx, cy, rx, ry, phi, cosPhi, sinPhi, theta1, dtheta };
}

// 把一条 path 的 d 拆成若干闭合子轮廓（点数组，viewBox 单位）
function flattenPathToContours(d) {
  const segs = parsePath(d);
  const contours = [];
  let cur = [];
  let x = 0, y = 0, sx = 0, sy = 0; // 当前点 / 子路径起点
  const push = (px, py) => { if (cur.length === 0 || cur[cur.length - 1][0] !== px || cur[cur.length - 1][1] !== py) cur.push([px, py]); };
  const close = () => { if (cur.length > 1) contours.push(cur); cur = []; };
  for (const { cmd, nums } of segs) {
    if (cmd === 'M') {
      close(); x = nums[0]; y = nums[1]; sx = x; sy = y; cur = [[x, y]];
    } else if (cmd === 'L') {
      x = nums[0]; y = nums[1]; push(x, y);
    } else if (cmd === 'H') {
      x = nums[0]; push(x, y);
    } else if (cmd === 'V') {
      y = nums[0]; push(x, y);
    } else if (cmd === 'C') {
      const x1 = nums[0], y1 = nums[1], x2 = nums[2], y2 = nums[3], xn = nums[4], yn = nums[5];
      for (let i = 1; i <= FLATTEN_SEG; i++) {
        const t = i / FLATTEN_SEG, mt = 1 - t;
        const bx = mt * mt * mt * x + 3 * mt * mt * t * x1 + 3 * mt * t * t * x2 + t * t * t * xn;
        const by = mt * mt * mt * y + 3 * mt * mt * t * y1 + 3 * mt * t * t * y2 + t * t * t * yn;
        push(bx, by);
      }
      x = xn; y = yn;
    } else if (cmd === 'Q') {
      const x1 = nums[0], y1 = nums[1], xn = nums[2], yn = nums[3];
      for (let i = 1; i <= FLATTEN_SEG; i++) {
        const t = i / FLATTEN_SEG, mt = 1 - t;
        push(mt * mt * x + 2 * mt * t * x1 + t * t * xn, mt * mt * y + 2 * mt * t * y1 + t * t * yn);
      }
      x = xn; y = yn;
    } else if (cmd === 'A') {
      const rx = nums[0], ry = nums[1], phiDeg = nums[2], fa = nums[3], fs = nums[4], xn = nums[5], yn = nums[6];
      const A = svgArcToCenter(x, y, xn, yn, fa, fs, rx, ry, phiDeg);
      for (let i = 1; i <= FLATTEN_SEG; i++) {
        const a = A.theta1 + A.dtheta * (i / FLATTEN_SEG);
        const px = A.cx + A.rx * Math.cos(a) * A.cosPhi - A.ry * Math.sin(a) * A.sinPhi;
        const py = A.cy + A.rx * Math.cos(a) * A.sinPhi + A.ry * Math.sin(a) * A.cosPhi;
        push(px, py);
      }
      x = xn; y = yn;
    } else if (cmd === 'Z') {
      x = sx; y = sy; close();
    }
  }
  close();
  return contours;
}

// ------------------------------------------------------------------ DXF (R12)
function buildDxf(allContours) {
  const L = [];
  const g = (code, val) => { L.push(String(code)); L.push(String(val)); };
  g(0, 'SECTION'); g(2, 'HEADER');
  g(9, '$INSUNITS'); g(70, 4);      // 4 = millimeters
  g(9, '$MEASUREMENT'); g(70, 1);
  g(0, 'ENDSEC');
  g(0, 'SECTION'); g(2, 'ENTITIES');
  for (const contour of allContours) {
    if (contour.length < 2) continue;
    g(0, 'POLYLINE'); g(8, 'SILKSCREEN'); g(66, 1); g(70, 1); // 70=1 闭合
    for (const [ux, uy] of contour) {
      g(0, 'VERTEX'); g(8, 'SILKSCREEN');
      g(10, (ux * DXF_SCALE).toFixed(4));
      g(20, ((1000 - uy) * DXF_SCALE).toFixed(4)); // DXF y 轴向上，翻转
      g(30, '0.0');
    }
    g(0, 'SEQEND'); g(8, 'SILKSCREEN');
  }
  g(0, 'ENDSEC');
  g(0, 'EOF');
  return L.join('\n') + '\n';
}

// ------------------------------------------------------------------ 主流程
(async function main() {
  // 1. 读取原 SVG，移除所有 <g> 文字组，保留非文字 path
  const raw = fs.readFileSync(SVG_IN, 'utf8');
  const stripped = raw.replace(/<g\b[\s\S]*?<\/g>/g, '');
  const kept = [];
  const pathRe = /<path\b[^>]*?\/>/g; let pm;
  while ((pm = pathRe.exec(stripped)) !== null) {
    const tag = pm[0];
    const d = (tag.match(/\sd="([^"]*)"/) || [])[1];
    const fr = (tag.match(/\sfill-rule="([^"]*)"/) || [])[1];
    if (d) kept.push({ d, fillRule: fr || null });
  }
  console.log(`保留非文字 path: ${kept.length} 条`);

  // 2. 加载字体（opentype.parse 读取 ArrayBuffer；load 已弃用），生成弧排文字
  const toArrayBuffer = (p) => {
    const b = fs.readFileSync(p);
    return b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength);
  };
  const cnFont = opentype.parse(toArrayBuffer(CN_FONT));
  const enFont = opentype.parse(toArrayBuffer(EN_FONT));
  const cnPaths = arcText(cnFont, CN_TEXT, CN_SIZE, CN_STEP, 'top');
  const enPaths = arcText(enFont, EN_TEXT, EN_SIZE, EN_STEP, 'bottom');
  console.log(`上弧中文字符 path: ${cnPaths.length}  下弧英文字符 path: ${enPaths.length}`);
  console.log(`上弧径向范围: ${RADIAL.top.min.toFixed(1)} ~ ${RADIAL.top.max.toFixed(1)} (环带 332~406)`);
  console.log(`下弧径向范围: ${RADIAL.bottom.min.toFixed(1)} ~ ${RADIAL.bottom.max.toFixed(1)} (环带 332~406)`);

  // 3. 组装 SVG
  const parts = [];
  parts.push('<?xml version="1.0" encoding="UTF-8"?>');
  parts.push('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1000 1000" width="1000" height="1000">');
  parts.push('<!-- PCB Silkscreen: 西南石油大学 机器人工程 融合徽章 (字体真实轮廓弧排版) -->');
  parts.push('');
  parts.push('<!-- 非文字图形（齿轮环 / 双细圆环 / 电路走线 / 机器人图标），原样保留 -->');
  for (const k of kept) {
    parts.push(`<path d="${k.d}" fill="#000000"${k.fillRule ? ` fill-rule="${k.fillRule}"` : ''}/>`);
  }
  parts.push('');
  parts.push('<!-- 上弧：西南石油大学（simhei.ttf 轮廓，烘焙弧排） -->');
  for (const d of cnPaths) parts.push(`<path d="${d}" fill="#000000" fill-rule="nonzero"/>`);
  parts.push('');
  parts.push('<!-- 下弧：ROBOTICS ENGINEERING（arialbd.ttf 轮廓，烘焙弧排） -->');
  for (const d of enPaths) parts.push(`<path d="${d}" fill="#000000" fill-rule="nonzero"/>`);
  parts.push('');
  parts.push('</svg>');
  const svg = parts.join('\n');
  fs.writeFileSync(SVG_OUT, svg, 'utf8');
  console.log(`写出 SVG: ${SVG_OUT} (${(svg.length / 1024).toFixed(1)} KB)`);

  // 4. 渲染 PNG 预览（白底）
  await sharp(Buffer.from(svg), { density: 300 })
    .flatten({ background: '#ffffff' })
    .resize(1000, 1000)
    .png()
    .toFile(PNG_OUT);
  console.log(`写出 PNG: ${PNG_OUT}`);

  // 5. 导出 DXF（R12）—— 合并所有 path 的子轮廓
  const allContours = [];
  const everyD = [...kept.map((k) => k.d), ...cnPaths, ...enPaths];
  for (const d of everyD) allContours.push(...flattenPathToContours(d));
  fs.writeFileSync(DXF_OUT, buildDxf(allContours), 'utf8');
  console.log(`写出 DXF: ${DXF_OUT} (轮廓 ${allContours.length} 条, 比例 ${DXF_SCALE}mm/单位)`);
})();
