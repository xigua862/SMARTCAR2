"""虚拟小车线上循迹：光电管 + PD 转向 + 增量式速度 PID。"""

from __future__ import annotations

import json
from collections import defaultdict, deque
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon
from matplotlib.widgets import Button, Slider

from plot_map import DXF_PATH, parse_dxf_lines

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "SimSun", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

CAR_LENGTH = 35.0
CAR_WIDTH = 20.0
WHEELBASE = 30.0
SNAP_EPS = 0.05
MIN_CORNER_DEG = 90.0
STRAIGHT_DEG = 15.0
START = (0.0, 0.0)
FIRST = (0.0, 300.0)

SENSOR_AHEAD = 50.0  # 前瞻 mm
SENSOR_WIDTH = 50.0  # 光电排宽度 mm
SENSOR_COUNT = 9
SENSOR_SPACING = SENSOR_WIDTH / (SENSOR_COUNT - 1)
ERROR_RANGE = 100.0  # 误差输出 ±100
DETECT_RADIUS = 12.0
LOST_DIST = SENSOR_WIDTH * 0.55
MAX_STEER = np.deg2rad(55.0)
CTRL_HZ = 10000  # 光电扫描 / 转向 / 速度 并联
CTRL_DT = 1.0 / CTRL_HZ
PID_DT = 0.001  # 速度增量式 PID 仍按 1 ms 含义整定
FRAME_MS = 33
PLAY_SPEED = 2.0  # 画面快进倍数（仿真时间 / 墙钟）
STEPS_PER_FRAME = int(round(FRAME_MS * 0.001 / CTRL_DT * PLAY_SPEED))
ERR_TAU = 0.12  # 光电误差低通时间常数
ENCODER_RES = 1e-5  # 轮速编码器量化 mm（轴距 30 时用 0.00001）
MAP_SAMPLE_DS = 2.0  # 记表间隔 mm（0.002 m；0.002 mm 一圈约七百万点）
MAP_PATH = Path(__file__).resolve().parent / "惯导路点.json"
LOOKAHEAD_MM = 50.0
MAP_PARAM_PATH = Path(__file__).resolve().parent / "控制参数.json"
RUN_PARAM_PATH = Path(__file__).resolve().parent / "控制参数_跑图.json"
_BODY_LOCAL = np.array(
    [
        [CAR_LENGTH / 2 + 5.0, 0.0],
        [CAR_LENGTH / 2 - 3.0, CAR_WIDTH / 2],
        [-CAR_LENGTH / 2, CAR_WIDTH / 2],
        [-CAR_LENGTH / 2, -CAR_WIDTH / 2],
        [CAR_LENGTH / 2 - 3.0, -CAR_WIDTH / 2],
    ]
)
_SENSOR_OFF = (np.arange(SENSOR_COUNT) - (SENSOR_COUNT - 1) / 2.0) * SENSOR_SPACING


def snap_pt(x, y, eps=SNAP_EPS):
    return (round(x / eps) * eps, round(y / eps) * eps)


def heading(p, q):
    return np.arctan2(q[1] - p[1], q[0] - p[0])


def heading_delta_deg(hin, hout):
    d = (hout - hin + np.pi) % (2 * np.pi) - np.pi
    return abs(d) * 180.0 / np.pi


def corner_deg(prev, cur, nxt):
    a = np.array(cur, dtype=float) - np.array(prev, dtype=float)
    b = np.array(nxt, dtype=float) - np.array(cur, dtype=float)
    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)
    if na < 1e-12 or nb < 1e-12:
        return 180.0
    a /= na
    b /= nb
    c = float(np.clip(np.dot(-a, b), -1.0, 1.0))
    return float(np.degrees(np.arccos(c)))


def edge_key(a, b):
    return tuple(sorted((a, b)))


def build_graph(segs):
    adj = defaultdict(list)
    for x1, y1, x2, y2 in segs:
        a, b = snap_pt(x1, y1), snap_pt(x2, y2)
        if a == b:
            continue
        if b not in adj[a]:
            adj[a].append(b)
        if a not in adj[b]:
            adj[b].append(a)
    return adj


def choose_next(prev, cur, cands):
    """未走边里：能转苯环就先转（合法非锐角），否则直行。"""
    if not cands:
        return None
    hin = heading(prev, cur)
    scored = []
    for nxt in cands:
        scored.append(
            (
                heading_delta_deg(hin, heading(cur, nxt)),
                corner_deg(prev, cur, nxt),
                nxt,
            )
        )
    legal = [s for s in scored if s[1] + 1e-6 >= MIN_CORNER_DEG]
    pool = legal if legal else scored
    turns = [s for s in pool if s[0] > STRAIGHT_DEG]
    if turns:
        turns.sort(key=lambda s: s[0])
        return turns[0][2]
    pool.sort(key=lambda s: s[0])
    return pool[0][2]


def build_follow_path(segs):
    """欧拉回路：外圈 + 苯环，每条边走一次。"""
    adj = build_graph(segs)
    start = snap_pt(*START)
    first = snap_pt(*FIRST)
    if first not in adj[start]:
        raise RuntimeError("起点 (0,0) 没有通往 (0,300) 的线段。")

    used = set()
    path = [start, first]
    used.add(edge_key(start, first))
    while True:
        prev, cur = path[-2], path[-1]
        unused = [n for n in adj[cur] if edge_key(cur, n) not in used]
        if not unused:
            break
        nxt = choose_next(prev, cur, unused)
        used.add(edge_key(cur, nxt))
        path.append(nxt)
        if len(path) > 10000:
            break
    if path[-1] != path[0]:
        path.append(path[0])
    return path


def path_to_segments(points):
    segs = []
    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        if np.hypot(x2 - x1, y2 - y1) > 1e-9:
            segs.append((float(x1), float(y1), float(x2), float(y2)))
    return np.asarray(segs, dtype=float)


def points_min_dist(px, py, segs):
    """多个点到折线的最小距离。"""
    if segs is None or len(segs) == 0:
        return np.full_like(np.atleast_1d(px), 1e9, dtype=float)
    x1, y1, x2, y2 = segs[:, 0], segs[:, 1], segs[:, 2], segs[:, 3]
    dx, dy = x2 - x1, y2 - y1
    len2 = np.maximum(dx * dx + dy * dy, 1e-12)
    pxa = np.atleast_1d(px)[:, None]
    pya = np.atleast_1d(py)[:, None]
    t = np.clip(((pxa - x1) * dx + (pya - y1) * dy) / len2, 0.0, 1.0)
    qx = x1 + t * dx
    qy = y1 + t * dy
    return np.hypot(pxa - qx, pya - qy).min(axis=1)


def point_to_segments(px, py, segs):
    """返回到折线的最小距离、最近点、该处切向角、段索引、段内比例 t。"""
    if segs is None or len(segs) == 0:
        return 1e9, px, py, 0.0, -1, 0.0
    x1, y1, x2, y2 = segs[:, 0], segs[:, 1], segs[:, 2], segs[:, 3]
    dx, dy = x2 - x1, y2 - y1
    len2 = dx * dx + dy * dy
    t = np.where(len2 > 1e-12, ((px - x1) * dx + (py - y1) * dy) / np.maximum(len2, 1e-12), 0.0)
    t = np.clip(t, 0.0, 1.0)
    qx = x1 + t * dx
    qy = y1 + t * dy
    dist = np.hypot(px - qx, py - qy)
    i = int(np.argmin(dist))
    tang = np.arctan2(dy[i], dx[i])
    return float(dist[i]), float(qx[i]), float(qy[i]), float(tang), i, float(t[i])


class Track:
    """把循迹折线做成带弧长的局部窗口，避免光电管扫到邻墙。"""

    def __init__(self, points):
        self.pts = np.asarray(points, dtype=float)
        dxy = np.diff(self.pts, axis=0)
        self.seglen = np.hypot(dxy[:, 0], dxy[:, 1])
        self.cum = np.concatenate([[0.0], np.cumsum(self.seglen)])
        self.total = float(self.cum[-1])
        self.segs = path_to_segments(points)

    def local_mask(self, s, back=40.0, fwd=140.0):
        """线段只要和弧长窗口有交集就算（长边不能只用中点）。"""
        a = self.cum[:-1]
        b = self.cum[1:]
        lo = s - back
        hi = s + fwd
        if hi - lo >= self.total - 1e-6:
            return np.ones(len(a), dtype=bool)
        lo_m = lo % self.total
        hi_m = hi % self.total
        if lo_m <= hi_m:
            return (a <= hi_m) & (b >= lo_m)
        return ((a <= self.total) & (b >= lo_m)) | ((a <= hi_m) & (b >= 0.0))

    def local_segs(self, s, back=40.0, fwd=140.0):
        mask = self.local_mask(s, back, fwd)
        if not np.any(mask):
            return self.segs, np.arange(len(self.segs))
        idx = np.nonzero(mask)[0]
        return self.segs[mask], idx

    def pose_at(self, s):
        s = float(s % self.total)
        i = int(np.searchsorted(self.cum, s, side="right") - 1)
        i = min(max(i, 0), len(self.seglen) - 1)
        span = self.seglen[i]
        t = 0.0 if span < 1e-9 else (s - self.cum[i]) / span
        t = float(np.clip(t, 0.0, 1.0))
        x = self.pts[i, 0] + t * (self.pts[i + 1, 0] - self.pts[i, 0])
        y = self.pts[i, 1] + t * (self.pts[i + 1, 1] - self.pts[i, 1])
        yaw = np.arctan2(self.pts[i + 1, 1] - self.pts[i, 1], self.pts[i + 1, 0] - self.pts[i, 0])
        return float(x), float(y), float(yaw)

    def project(self, px, py, s, back=80.0, fwd=160.0):
        segs, idx = self.local_segs(s, back, fwd)
        dist, qx, qy, tang, i, t = point_to_segments(px, py, segs)
        if i < 0:
            return dist, qx, qy, tang, s
        gi = int(idx[i])
        s_hit = float(self.cum[gi] + t * self.seglen[gi]) % self.total
        return dist, qx, qy, tang, s_hit


class IncrementalPID:
    """增量式 PID：u_k = u_{k-1} + Kp(e-e1) + Ki e + Kd(e-2e1+e2)"""

    def __init__(self, kp=0.55, ki=0.0036, kd=1.32, umin=0.0, umax=400.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.umin = umin
        self.umax = umax
        self.reset(0.0)

    def reset(self, u0=0.0):
        self.e1 = 0.0
        self.e2 = 0.0
        self.u = float(u0)

    def update(self, setpoint, meas, dt=CTRL_DT):
        e = float(setpoint - meas)
        # Ki/Kd 按 1 ms 含义整定，10 kHz 时按拍数折算
        n = float(dt) / PID_DT
        du = self.kp * (e - self.e1) + self.ki * e * n + self.kd * (e - 2.0 * self.e1 + self.e2) / max(n, 1e-9)
        self.u = float(np.clip(self.u + du, self.umin, self.umax))
        self.e2 = self.e1
        self.e1 = e
        return self.u


def wrap_pi(a):
    return float((a + np.pi) % (2.0 * np.pi) - np.pi)


class Odometry:
    """陀螺仪 yaw + 轮速编码器。无误差时贴真值；有漂移时只污染估计坐标并跨圈积分。"""

    def reset(self):
        self.x = float(START[0])
        self.y = float(START[1])
        self._yaw_int = float(np.pi / 2.0)
        self.yaw = float(np.pi / 2.0)
        self.enc_s = 0.0
        self._resid = 0.0
        self._enc_bias = 0.0
        self.yaw_corr = 0.0  # 直道矫正叠加在积分航向上，弯道保持

    def __init__(self):
        self._rng = np.random.default_rng()
        self.reset()

    @staticmethod
    def _ins_error_on():
        ang = abs(Params.gyro_mu) > 1e-15 or Params.gyro_sigma > 1e-15
        enc = Params.enc_w > 1e-15 and (abs(Params.enc_mu) > 1e-15 or Params.enc_sigma > 1e-15)
        return ang or enc

    def _quantize_ds(self, v, dt):
        self._resid += float(v) * dt
        n = np.floor(self._resid / ENCODER_RES + 1e-18)
        if n <= 0.0:
            return 0.0
        ds = float(n * ENCODER_RES)
        self._resid -= ds
        return ds

    def update(self, v, omega, dt, noisy=False, truth=None):
        """噪声/偏置只进 (x,y,yaw) 估计，不改电机控制量。第二圈起一直积分。"""
        v = float(v)
        omega = float(omega)
        if noisy and abs(omega) > 1e-8 and Params.enc_w > 1e-15:
            inc = float(self._rng.normal(Params.enc_mu, max(Params.enc_sigma, 0.0)))
            inc *= Params.enc_w * abs(omega) * dt
            self._enc_bias += inc
            v = v * (1.0 + float(np.clip(self._enc_bias, -0.8, 3.0)))
        elif noisy and abs(self._enc_bias) > 1e-18:
            v = v * (1.0 + float(np.clip(self._enc_bias, -0.8, 3.0)))
        ds = self._quantize_ds(v, dt)
        if ds > 0.0:
            self.enc_s += ds

        have_err = self._ins_error_on() or abs(self._enc_bias) > 1e-18
        if (not noisy) or (not have_err):
            if truth is not None:
                self.x = float(truth[0])
                self.y = float(truth[1])
                self.yaw = float(truth[2])
                self._yaw_int = self.yaw
            return ds

        dpsi = np.deg2rad(float(self._rng.normal(Params.gyro_mu, max(Params.gyro_sigma, 0.0)))) * dt
        self._yaw_int = wrap_pi(self._yaw_int + omega * dt + dpsi)
        self.yaw = wrap_pi(self._yaw_int + self.yaw_corr)
        if ds > 0.0:
            self.x += ds * np.cos(self.yaw)
            self.y += ds * np.sin(self.yaw)
        return ds


class RouteTable:
    """第一圈按编码器弧长记 (x,y)，第二圈按表循迹。"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.pts = []
        self.xy = np.zeros((0, 2), dtype=np.float64)
        self.i = 0
        self._acc = 0.0
        self.frozen = False
        self.left_home = False
        self.length = 1.0
        self.v_profile = np.zeros(0, dtype=np.float64)
        self.kind = np.zeros(0, dtype=np.int8)  # 0弯 1出入弯 2可矫正直道
        self.kind_ver = 0

    def maybe_record(self, odom: Odometry, ds):
        if self.frozen or ds <= 0.0:
            return
        self._acc += ds
        while self._acc >= MAP_SAMPLE_DS:
            self._acc -= MAP_SAMPLE_DS
            self.pts.append((float(odom.x), float(odom.y)))

    def should_close(self, odom: Odometry):
        if self.frozen:
            return False
        if not self.left_home:
            if odom.enc_s > 80.0:
                self.left_home = True
            return False
        if odom.enc_s < 5000.0 or len(self.pts) < 80:
            return False
        return float(np.hypot(odom.x - START[0], odom.y - START[1])) < 18.0

    def freeze(self, odom: Odometry | None = None):
        if len(self.pts) >= 2:
            p0 = self.pts[0]
            p1 = self.pts[-1]
            if np.hypot(p1[0] - p0[0], p1[1] - p0[1]) > MAP_SAMPLE_DS * 0.5:
                self.pts.append(p0)
        self.xy = np.asarray(self.pts, dtype=np.float64)
        self.frozen = True
        self.i = 0
        n = len(self.xy)
        self.length = max(MAP_SAMPLE_DS, (max(n, 1) - 1) * MAP_SAMPLE_DS)
        self.rebuild_speed()
        self.classify_straights()
        try:
            self.save_json(MAP_PATH)
        except Exception:
            pass

    def set_index_from_enc(self, enc_s):
        """按编码器弧长在表上取点，整圈循环，不跳跃搜最近。"""
        n = len(self.xy)
        if n < 2:
            return
        s = float(enc_s) % self.length
        self.i = int(np.clip(s / MAP_SAMPLE_DS, 0.0, n - 1.0001))

    def lookahead(self, dist_mm=None):
        n = len(self.xy)
        if n < 2:
            return float(START[0]), float(START[1])
        if dist_mm is None:
            dist_mm = LOOKAHEAD_MM
        step = max(1, int(round(float(dist_mm) / MAP_SAMPLE_DS)))
        p = self.xy[(self.i + step) % n]
        return float(p[0]), float(p[1])

    def window_segs(self, back_mm=80.0, fwd_mm=160.0):
        """编码器下标附近的路表折线，闭环处首尾相接。"""
        n = len(self.xy)
        if n < 3:
            return None
        b = max(1, int(round(back_mm / MAP_SAMPLE_DS)))
        f = max(2, int(round(fwd_mm / MAP_SAMPLE_DS)))
        idx = (self.i + np.arange(-b, f + 1)) % n
        pts = self.xy[idx]
        segs = np.column_stack((pts[:-1, 0], pts[:-1, 1], pts[1:, 0], pts[1:, 1]))
        keep = np.hypot(segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1]) > 1e-6
        if not np.any(keep):
            return None
        return segs[keep]

    def _chord(self, i, ahead_pts):
        n = len(self.xy)
        j = (int(i) + max(1, int(ahead_pts))) % n
        return self.xy[int(i) % n], self.xy[j]

    def path_yaw(self, ahead_mm=30.0):
        n = len(self.xy)
        if n < 2:
            return 0.0
        a, b = self._chord(self.i, max(1, int(round(ahead_mm / MAP_SAMPLE_DS))))
        return float(np.arctan2(b[1] - a[1], b[0] - a[0]))

    def cross_track(self, x, y, ahead_mm=12.0):
        n = len(self.xy)
        if n < 2:
            return 0.0
        a, b = self._chord(self.i, max(1, int(round(ahead_mm / MAP_SAMPLE_DS))))
        vx, vy = b[0] - a[0], b[1] - a[1]
        den = vx * vx + vy * vy
        if den < 1e-18:
            return 0.0
        return float((x - a[0]) * (-vy) + (y - a[1]) * vx) / np.sqrt(den)

    def turn_ahead(self, near_mm=24.0, far_mm=90.0):
        """表上前方航向变化 0..1，90° 为 1。"""
        n = len(self.xy)
        if n < 8:
            return 0.0
        yaw0 = self.path_yaw(near_mm)
        i = self.i
        k = max(2, int(round(far_mm / MAP_SAMPLE_DS)))
        a, b = self._chord(i + k // 2, max(1, k // 2))
        yaw1 = float(np.arctan2(b[1] - a[1], b[0] - a[0]))
        d = abs(wrap_pi(yaw1 - yaw0))
        return min(d / (0.5 * np.pi), 1.0)

    def save_json(self, path=None):
        path = Path(path or MAP_PATH)
        if len(self.xy) < 2:
            raise ValueError("没有可保存的路表")
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "encoder_res": ENCODER_RES,
                        "sample_ds": MAP_SAMPLE_DS,
                        "n": int(len(self.xy)),
                        "pts": self.xy.tolist(),
                    },
                    f,
                    ensure_ascii=False,
                )
        except Exception as e:
            print("保存路点失败:", e)
            raise

    def load_json(self, path=None):
        path = Path(path or MAP_PATH)
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        pts = data.get("pts") if isinstance(data, dict) else data
        if pts is None or len(pts) < 8:
            raise ValueError("路表太短或格式不对")
        self.pts = [(float(p[0]), float(p[1])) for p in pts]
        self.xy = np.asarray(self.pts, dtype=np.float64)
        self.frozen = True
        self.left_home = True
        self.i = 0
        self._acc = 0.0
        self.length = max(MAP_SAMPLE_DS, (len(self.xy) - 1) * MAP_SAMPLE_DS)
        self.rebuild_speed()
        self.classify_straights()
        return len(self.xy)

    def rebuild_speed(self):
        """按路表曲率做前后向速度规划：直道拉满，弯前按刹车距离提前减速。"""
        n = len(self.xy)
        if n < 8:
            self.v_profile = np.full(max(n, 1), Params.vmin, dtype=np.float64)
            return
        nxt = np.roll(self.xy, -1, axis=0)
        yaw = np.arctan2(nxt[:, 1] - self.xy[:, 1], nxt[:, 0] - self.xy[:, 0])
        dyaw = np.abs((np.roll(yaw, -1) - yaw + np.pi) % (2.0 * np.pi) - np.pi)
        win = max(1, int(round(48.0 / MAP_SAMPLE_DS)))
        ker = np.ones(2 * win + 1) / (2 * win + 1)
        ext = np.concatenate([dyaw[-win:], dyaw, dyaw[:win]])
        dyaw_s = np.convolve(ext, ker, mode="valid")[:n]
        kappa = dyaw_s / MAP_SAMPLE_DS
        a_lat = max(float(Params.a_lat), 200.0)
        v_lim = np.sqrt(a_lat / np.maximum(kappa, 1e-8))
        v_lim = np.clip(v_lim, Params.vmin, Params.vmax)
        ds = MAP_SAMPLE_DS
        a_br = max(float(Params.a_brake), 80.0)
        a_ac = max(float(Params.a_acc), 80.0)
        v = v_lim.astype(np.float64).copy()
        for _ in range(2):
            for i in range(n - 2, -1, -1):
                v[i] = min(v[i], np.sqrt(v[i + 1] * v[i + 1] + 2.0 * a_br * ds))
            v[-1] = min(v[-1], np.sqrt(v[0] * v[0] + 2.0 * a_br * ds))
        for _ in range(2):
            for i in range(n - 1):
                v[i + 1] = min(v[i + 1], np.sqrt(v[i] * v[i] + 2.0 * a_ac * ds))
            v[0] = min(v[0], np.sqrt(v[-1] * v[-1] + 2.0 * a_ac * ds))
        self.v_profile = np.clip(v, Params.vmin, Params.vmax)

    def speed_at(self, enc_s):
        n = len(self.v_profile)
        if n < 2:
            return float(Params.vmin)
        s = float(enc_s) % self.length
        i = int(np.clip(s / MAP_SAMPLE_DS, 0.0, n - 1.0001))
        step = max(0, int(round(max(Params.look_mm, 0.0) / MAP_SAMPLE_DS)))
        j = (i + step) % n
        return float(min(self.v_profile[i], self.v_profile[j]))

    def classify_straights(self):
        """按航向变化切出长直道；两端再按出入弯距离裁掉，中间才允许矫正。"""
        n = len(self.xy)
        self.kind = np.zeros(max(n, 1), dtype=np.int8)
        self.kind_ver += 1
        if n < 16:
            return
        nxt = np.roll(self.xy, -1, axis=0)
        yaw = np.arctan2(nxt[:, 1] - self.xy[:, 1], nxt[:, 0] - self.xy[:, 0])
        probe = max(2, int(round(40.0 / MAP_SAMPLE_DS)))
        dpsi = np.abs((np.roll(yaw, -probe) - yaw + np.pi) % (2.0 * np.pi) - np.pi)
        thresh = np.deg2rad(max(Params.straight_deg, 1.0))
        straight = dpsi < thresh
        min_n = max(8, int(round(Params.straight_min / MAP_SAMPLE_DS)))
        trim_n = max(1, int(round(Params.corner_margin / MAP_SAMPLE_DS)))
        runs = []
        if bool(np.all(straight)):
            runs = [(0, n)]
        else:
            ext = np.concatenate([[False], straight, [False]])
            d = np.diff(ext.astype(np.int8))
            st = np.where(d == 1)[0]
            en = np.where(d == -1)[0]
            pairs = list(zip(st.tolist(), en.tolist()))
            if straight[0] and straight[-1] and len(pairs) >= 2:
                a0, b0 = pairs[0]
                a1, b1 = pairs[-1]
                pairs = [(a1, b0 + n)] + pairs[1:-1]
            runs = pairs
        for a, b in runs:
            ln = int(b - a)
            if ln < min_n:
                continue
            for k in range(ln):
                i = (a + k) % n
                if k < trim_n or k >= ln - trim_n:
                    self.kind[i] = 1
                else:
                    self.kind[i] = 2

    def on_core_straight(self):
        n = len(self.kind)
        if n < 2 or self.i < 0:
            return False
        return int(self.kind[int(self.i) % n]) == 2

    def color_segments(self):
        n = len(self.xy)
        if n < 2:
            return [], []
        cols_map = {0: "#ff8a4c", 1: "#e6c35c", 2: "#4ee0a8"}
        segs = []
        cols = []
        kind = self.kind if len(self.kind) == n else np.zeros(n, dtype=np.int8)
        for i in range(n):
            j = (i + 1) % n
            segs.append([(float(self.xy[i, 0]), float(self.xy[i, 1])),
                         (float(self.xy[j, 0]), float(self.xy[j, 1]))])
            cols.append(cols_map[int(kind[i])])
        return segs, cols

    def project_s(self, x, y):
        """惯导点投到当前下标附近路表，得到弧长。"""
        n = len(self.xy)
        if n < 2:
            return 0.0
        a, b = self._chord(self.i, max(2, int(round(16.0 / MAP_SAMPLE_DS))))
        vx, vy = float(b[0] - a[0]), float(b[1] - a[1])
        den = vx * vx + vy * vy
        t = 0.0 if den < 1e-12 else ((x - a[0]) * vx + (y - a[1]) * vy) / den
        s = (self.i * MAP_SAMPLE_DS + t * np.sqrt(max(den, 0.0))) % self.length
        return float(s)


PARAM_DIR = Path(__file__).resolve().parent
PARAM_KEYS = (
    "kp_steer",
    "kd_steer",
    "k_gyro",
    "kp_speed",
    "ki_speed",
    "kd_speed",
    "vmax",
    "vmin",
    "c_long",
    "c_lat",
    "span_th",
    "e_slow",
    "look_mm",
    "a_lat",
    "a_brake",
    "a_acc",
    "gyro_mu",
    "gyro_sigma",
    "enc_mu",
    "enc_sigma",
    "enc_w",
    "corr_on",
    "straight_min",
    "corner_margin",
    "straight_deg",
    "corr_kp",
    "corr_ki",
    "corr_kd",
    "corr_enc_k",
)


class Params:
    kp_steer = 0.02169349424402764
    kd_steer = 0.005600512637774163
    k_gyro = 0.04810452892721513  # 陀螺仪转向抑制：u -= k_gyro * omega
    kp_speed = 1.1099415376109032
    ki_speed = 0.0068565348842504
    kd_speed = 1.8337472250590277
    vmax = 538.4284312105809
    vmin = 135.7749553648659
    c_long = 0.5560642681707801  # 纵向摩擦 1/s
    c_lat = 2.1309316741537865  # 横向/横摆阻尼 1/s
    span_th = 20.0  # 最左-最右跨度阈值 mm，大于则取最外侧
    e_slow = 0.9493471184180695  # 光电误差满偏时减到 vmin 的比例
    look_mm = 50.0  # 跑图转向/速度前瞻 mm
    a_lat = 4500.0  # 弯道向心加速度 mm/s²
    a_brake = 3000.0  # 规划刹车加速度 mm/s²
    a_acc = 2000.0  # 规划加速 mm/s²
    gyro_mu = 0.0  # 航向误差均值 deg/s，第二圈起积分
    gyro_sigma = 0.0  # 航向误差标准差 deg/s，积分成随机游走
    enc_mu = 0.0  # 转弯时编码器误差均值，积分进比例
    enc_sigma = 0.0  # 转弯时编码器误差标准差
    enc_w = 0.0  # 编码器噪声 × |ω| 系数，积分不跨圈清零
    corr_on = 0.0  # 1=开启直道矫正
    straight_min = 220.0  # 长直道最短 mm
    corner_margin = 80.0  # 出入弯裁掉 mm，这段不矫正
    straight_deg = 10.0  # 40mm 内航向变化小于此值算直
    corr_kp = 0.002  # 横向偏差 mm → 航向补偿 rad（增量式 P 跳变）
    corr_ki = 4e-6  # 长直道慢慢吃掉常值偏，太大容易顶满 ±18°
    corr_kd = 0.0002
    corr_enc_k = 1.2  # 直道上编码器弧长向几何投影靠拢 1/s

    @classmethod
    def to_dict(cls):
        return {k: float(getattr(cls, k)) for k in PARAM_KEYS}

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise ValueError("参数文件格式不对")
        for k in PARAM_KEYS:
            if k in data:
                setattr(cls, k, float(data[k]))
        if cls.vmax <= cls.vmin + 5.0:
            cls.vmax = cls.vmin + 5.0
        if cls.look_mm < 20.0:
            cls.look_mm = 20.0


def _default_run_dict(src):
    d = dict(src)
    d["vmax"] = 1500.0
    d["vmin"] = 280.0
    d["look_mm"] = 120.0
    d["a_lat"] = 5200.0
    d["a_brake"] = 3400.0
    d["a_acc"] = 2200.0
    d["e_slow"] = 0.28
    d["k_gyro"] = min(float(d.get("k_gyro", 0.08)), 0.08)
    d.setdefault("corr_on", 0.0)
    d.setdefault("straight_min", 220.0)
    d.setdefault("corner_margin", 80.0)
    d.setdefault("straight_deg", 10.0)
    d.setdefault("corr_kp", 0.002)
    d.setdefault("corr_ki", 4e-6)
    d.setdefault("corr_kd", 0.0002)
    d.setdefault("corr_enc_k", 1.2)
    return d


class ParamBank:
    """建图 / 跑图两套参数，滑条改的是当前模式。"""

    mode = "map"
    data = {"map": {}, "run": {}}
    pending_ui_sync = False

    @classmethod
    def stash(cls):
        cls.data[cls.mode] = Params.to_dict()

    @classmethod
    def use(cls, mode):
        cls.stash()
        cls.mode = "run" if mode == "run" else "map"
        src = cls.data.get(cls.mode) or {}
        if src:
            Params.from_dict(src)
        cls.pending_ui_sync = True

    @classmethod
    def load_files(cls):
        cls.data["map"] = Params.to_dict()
        if MAP_PARAM_PATH.exists():
            try:
                with open(MAP_PARAM_PATH, encoding="utf-8") as f:
                    Params.from_dict(json.load(f))
                cls.data["map"] = Params.to_dict()
            except Exception as e:
                print("加载建图参数失败:", e)
        if RUN_PARAM_PATH.exists():
            try:
                with open(RUN_PARAM_PATH, encoding="utf-8") as f:
                    Params.from_dict(json.load(f))
                cls.data["run"] = Params.to_dict()
            except Exception as e:
                print("加载跑图参数失败:", e)
                cls.data["run"] = _default_run_dict(cls.data["map"])
        else:
            cls.data["run"] = _default_run_dict(cls.data["map"])
        Params.from_dict(cls.data["map"])
        cls.mode = "map"


def _ask_param_path(save=True):
    """弹出文件框；没有 tkinter 时退回到工程目录下的 控制参数.json。"""
    fallback = RUN_PARAM_PATH if ParamBank.mode == "run" else MAP_PARAM_PATH
    try:
        import tkinter as tk
        from tkinter import filedialog
    except ImportError:
        return fallback if save or fallback.exists() else None

    own = tk._default_root is None
    root = tk._default_root or tk.Tk()
    if own:
        root.withdraw()
        try:
            root.wm_attributes("-topmost", True)
        except Exception:
            pass
    kw = dict(
        parent=root,
        initialdir=str(PARAM_DIR),
        filetypes=[("JSON 参数", "*.json"), ("全部文件", "*.*")],
    )
    initial = "控制参数_跑图.json" if ParamBank.mode == "run" else "控制参数.json"
    if save:
        path = filedialog.asksaveasfilename(
            title="保存控制参数",
            defaultextension=".json",
            initialfile=initial,
            **kw,
        )
    else:
        path = filedialog.askopenfilename(title="导入控制参数", **kw)
    if own:
        root.destroy()
    return Path(path) if path else None


def _ask_map_path(save=True):
    fallback = MAP_PATH
    try:
        import tkinter as tk
        from tkinter import filedialog
    except ImportError:
        return fallback if save or fallback.exists() else None

    own = tk._default_root is None
    root = tk._default_root or tk.Tk()
    if own:
        root.withdraw()
        try:
            root.wm_attributes("-topmost", True)
        except Exception:
            pass
    kw = dict(
        parent=root,
        initialdir=str(PARAM_DIR),
        filetypes=[("JSON 路表", "*.json"), ("全部文件", "*.*")],
    )
    if save:
        path = filedialog.asksaveasfilename(
            title="保存路表",
            defaultextension=".json",
            initialfile="惯导路点.json",
            **kw,
        )
    else:
        path = filedialog.askopenfilename(title="载入路表", **kw)
    if own:
        root.destroy()
    return Path(path) if path else None


class Car:
    def __init__(self, track: Track):
        self.track = track
        self.speed_pid = IncrementalPID()
        self.odom = Odometry()
        self.route = RouteTable()
        self.reset()

    def reset(self, keep_map=False):
        saved = None
        if keep_map and self.route.frozen and len(self.route.xy) >= 8:
            saved = (
                self.route.xy.copy(),
                float(self.route.length),
                None if self.route.v_profile is None else self.route.v_profile.copy(),
            )
        self.x, self.y = float(START[0]), float(START[1])
        self.yaw = np.pi / 2.0
        self.v = Params.vmin
        self.v_lat = 0.0
        self.omega = 0.0
        self.s = 0.0
        self.steer = 0.0
        self.v_cmd = Params.vmin
        self.v_target = Params.vmin
        self.error = 0.0
        self.error_prev = 0.0
        self.err_f = 0.0
        self.last_seen_error = 0.0
        self.photo_error = 0.0
        self.photo_lost = True
        self.on_straight = False
        self.corr_e = 0.0
        self.lost = False
        self.lost_time = 0.0
        self.hits = np.zeros(SENSOR_COUNT, dtype=bool)
        self.sensor_xy = np.zeros((SENSOR_COUNT, 2))
        self._local = None
        self._idx = None
        self._s_cached = -1e9
        self.odom.reset()
        self.route.reset()
        if saved is not None:
            xy, length, vprof = saved
            self.route.xy = xy
            self.route.pts = [(float(p[0]), float(p[1])) for p in xy]
            self.route.frozen = True
            self.route.left_home = True
            self.route.length = length
            if vprof is not None and len(vprof) == len(xy):
                self.route.v_profile = vprof
            else:
                self.route.rebuild_speed()
            self.route.classify_straights()
            self.nav = "follow"
        else:
            self.nav = "map"
        self.speed_pid.kp = Params.kp_speed
        self.speed_pid.ki = Params.ki_speed
        self.speed_pid.kd = Params.kd_speed
        self.speed_pid.umin = 0.0
        self.speed_pid.umax = Params.vmax
        self.speed_pid.reset(Params.vmin)
        self.corr_pid = getattr(self, "corr_pid", IncrementalPID())
        self.corr_pid.kp = Params.corr_kp
        self.corr_pid.ki = Params.corr_ki
        self.corr_pid.kd = Params.corr_kd
        self.corr_pid.umin = -np.deg2rad(18.0)
        self.corr_pid.umax = np.deg2rad(18.0)
        self.corr_pid.reset(0.0)

    def read_sensors(self, local):
        """多个误差点：跨度大于阈值取最外侧，否则最左与最右平均。"""
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        fx = self.x + SENSOR_AHEAD * c
        fy = self.y + SENSOR_AHEAD * s
        lx, ly = -s, c
        self.sensor_xy[:, 0] = fx + _SENSOR_OFF * lx
        self.sensor_xy[:, 1] = fy + _SENSOR_OFF * ly

        dists = points_min_dist(self.sensor_xy[:, 0], self.sensor_xy[:, 1], local)
        hits = dists <= DETECT_RADIUS
        self.hits = hits
        if not bool(hits.any()):
            return self.last_seen_error, True

        hit_off = _SENSOR_OFF[hits]
        half = SENSOR_WIDTH * 0.5
        if hit_off.size == 1:
            err_mm = float(hit_off[0])
        else:
            left_m = float(hit_off.max())
            right_m = float(hit_off.min())
            span = abs(left_m - right_m)
            if span > Params.span_th:
                # 跨度大：取离光电中心更远的最外侧（左右一样远则仍平均，十字直行）
                if abs(left_m) > abs(right_m) + 1e-9:
                    err_mm = left_m
                elif abs(right_m) > abs(left_m) + 1e-9:
                    err_mm = right_m
                else:
                    err_mm = 0.5 * (left_m + right_m)
            else:
                err_mm = 0.5 * (left_m + right_m)

        err = float(np.clip(err_mm / half * ERROR_RANGE, -ERROR_RANGE, ERROR_RANGE))
        self.last_seen_error = err
        return err, False

    def _sync_path(self):
        if self._local is None or abs(self.s - self._s_cached) > 40.0:
            self._local, self._idx = self.track.local_segs(self.s, back=80.0, fwd=SENSOR_AHEAD + 180.0)
            self._s_cached = self.s
        dist_body, _qx, _qy, _tang, i, t = point_to_segments(self.x, self.y, self._local)
        if i >= 0 and dist_body < 50.0:
            gi = int(self._idx[i])
            s_hit = float(self.track.cum[gi] + t * self.track.seglen[gi]) % self.track.total
            ds = (s_hit - self.s + 0.5 * self.track.total) % self.track.total - 0.5 * self.track.total
            if ds > -80.0:
                self.s = s_hit

    def control_steer(self, dt):
        """转向环：第一圈光电；其后把惯导当成车，在路表上用同一套 PD（含陀螺抑制）。"""
        if self.nav == "follow":
            self.route.set_index_from_enc(self.odom.enc_s)
            ld = float(max(Params.look_mm, 40.0))
            gx, gy = self.route.lookahead(ld)
            dx = gx - self.odom.x
            dy = gy - self.odom.y
            c, s = np.cos(self.odom.yaw), np.sin(self.odom.yaw)
            # 路表前瞻点在车体坐标系：前 lx、左 ly（与光电同号）
            err_mm = float(-s * dx + c * dy)
            half = SENSOR_WIDTH * 0.5
            self.error = float(np.clip(err_mm / half * ERROR_RANGE, -ERROR_RANGE, ERROR_RANGE))
        err = self.error
        if self.lost and self.nav != "follow":
            self.err_f = err
            u = Params.kp_steer * err
        else:
            a = 1.0 - np.exp(-dt / ERR_TAU)
            self.err_f += a * (err - self.err_f)
            de = (self.err_f - self.error_prev) / max(dt, 1e-6)
            kp = Params.kp_steer
            kd = Params.kd_steer
            u = kp * err + kd * de
        gyro_k = Params.k_gyro
        if self.nav == "follow":
            gyro_k *= 0.35
        u -= gyro_k * self.omega
        self.steer = float(np.clip(u, -MAX_STEER, MAX_STEER))
        self.error_prev = self.err_f

    def control_speed(self, dt):
        """速度环：第一圈看光电；其后按路表转角和惯导横向偏差。"""
        if self.lost and self.nav != "follow":
            v_target = Params.vmin
        elif self.nav == "follow":
            if self.route.frozen and len(self.route.v_profile) > 2:
                v_target = self.route.speed_at(self.odom.enc_s)
            else:
                turn = self.route.turn_ahead(
                    near_mm=max(30.0, Params.look_mm * 0.35),
                    far_mm=max(100.0, Params.look_mm * 1.3),
                )
                v_target = Params.vmax - turn * (Params.vmax - Params.vmin)
            e_n = float(np.clip(abs(self.error) / ERROR_RANGE, 0.0, 1.0))
            v_target = v_target * (1.0 - 0.45 * Params.e_slow * e_n)
            v_target = float(np.clip(v_target, Params.vmin, Params.vmax))
        else:
            e_n = float(np.clip(abs(self.error) / ERROR_RANGE, 0.0, 1.0))
            slow = float(np.clip(Params.e_slow * e_n, 0.0, 1.0))
            v_target = Params.vmax - slow * (Params.vmax - Params.vmin)
            v_target = float(np.clip(v_target, Params.vmin, Params.vmax))
        self.v_target = v_target
        pid = self.speed_pid
        pid.kp, pid.ki, pid.kd = Params.kp_speed, Params.ki_speed, Params.kd_speed
        pid.umax = Params.vmax
        pid.umin = 0.0 if (self.lost and self.nav != "follow") else Params.vmin * 0.5
        self.v_cmd = float(np.clip(pid.update(v_target, self.v, dt), 0.0, Params.vmax))

    def correct_ins(self, dt):
        """长直道：光电横向 − 惯导横向 → 偏差，PID 叠加到惯导航向；编码器弧长向路表投影靠拢。"""
        self.on_straight = False
        if self.nav != "follow" or not self.route.frozen:
            return
        if Params.corr_on < 0.5:
            if abs(self.odom.yaw_corr) > 1e-12:
                self.odom.yaw_corr = 0.0
                self.corr_pid.reset(0.0)
                self.odom.yaw = wrap_pi(self.odom._yaw_int)
            return
        if not self.route.on_core_straight():
            return
        if self.photo_lost:
            return
        self.on_straight = True
        half = SENSOR_WIDTH * 0.5
        photo_mm = float(self.photo_error) / ERROR_RANGE * half
        cte_true = -photo_mm
        cte_ins = self.route.cross_track(self.odom.x, self.odom.y, ahead_mm=20.0)
        e_lat = float(cte_ins - cte_true)
        self.corr_e = e_lat
        pid = self.corr_pid
        pid.kp, pid.ki, pid.kd = Params.corr_kp, Params.corr_ki, Params.corr_kd
        pid.umin, pid.umax = -np.deg2rad(18.0), np.deg2rad(18.0)
        self.odom.yaw_corr = float(pid.update(0.0, e_lat, dt))
        self.odom.yaw = wrap_pi(self.odom._yaw_int + self.odom.yaw_corr)
        s_proj = self.route.project_s(self.odom.x, self.odom.y)
        L = self.route.length
        d = (s_proj - self.odom.enc_s + 0.5 * L) % L - 0.5 * L
        if abs(d) < 120.0:
            step = float(np.clip(Params.corr_enc_k * d * dt, -3.0, 3.0))
            self.odom.enc_s = float(self.odom.enc_s + step)

    def plant(self, dt):
        delta = float(np.clip(self.steer, -MAX_STEER, MAX_STEER))
        tau = 0.10
        a_long = (self.v_cmd - self.v) / tau - Params.c_long * self.v
        self.v += a_long * dt
        self.v = float(np.clip(self.v, 0.0, Params.vmax))

        omega_cmd = self.v / WHEELBASE * np.tan(delta)
        self.omega += (omega_cmd - self.omega) * min(1.0, 8.0 * dt)
        self.omega -= Params.c_lat * self.omega * dt
        self.v_lat += (self.v * np.sin(delta) * 0.25 - Params.c_lat * self.v_lat) * dt

        c, s = np.cos(self.yaw), np.sin(self.yaw)
        self.x += (self.v * c - self.v_lat * s) * dt
        self.y += (self.v * s + self.v_lat * c) * dt
        self.yaw += self.omega * dt
        self.yaw = (self.yaw + np.pi) % (2 * np.pi) - np.pi

    def tick(self):
        """同一拍：采样一次，转向/速度并联计算，再进被控对象。"""
        dt = CTRL_DT
        if ParamBank.mode == "run" and self.route.frozen:
            self.nav = "follow"
        self._sync_path()
        err, lost = self.read_sensors(self._local)
        self.photo_error = err
        self.photo_lost = bool(lost)
        self.error = err
        if lost:
            self.lost = True
            self.lost_time += dt
        else:
            self.lost = False
            self.lost_time = 0.0
        self.control_steer(dt)
        self.control_speed(dt)
        self.plant(dt)
        ds = self.odom.update(
            self.v,
            self.omega,
            dt,
            noisy=(self.nav == "follow"),
            truth=(self.x, self.y, self.yaw),
        )
        if self.nav == "follow":
            self.correct_ins(dt)
        if self.nav == "map":
            self.route.maybe_record(self.odom, ds)
            if self.route.should_close(self.odom):
                self.route.freeze(self.odom)
                self.odom._resid = 0.0
                ParamBank.use("run")
                self.route.rebuild_speed()
                self.nav = "follow"
                print(f"第一圈结束，路点 {len(self.route.xy)}，切跑图参数并按表规划速度。已写 {MAP_PATH.name}")
        return self.v_target


def car_poly(x, y, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return _BODY_LOCAL @ rot.T + np.array([x, y])


def main():
    print("解析 DXF，生成含苯环的循迹路线...")
    ParamBank.load_files()
    print(f"建图参数 {MAP_PARAM_PATH.name}，跑图参数 {RUN_PARAM_PATH.name}")
    wall_segs = parse_dxf_lines(DXF_PATH)
    path_xy = build_follow_path(wall_segs)
    track = Track(path_xy)
    pts = track.pts
    print(f"路线节点 {len(path_xy)}，边 {len(track.segs)}，全长 {track.total:.0f} mm，含苯环: {(np.abs(pts[:,1]-200)<1).any()}")

    car = Car(track)
    paused = {"v": False}
    trail = deque(maxlen=5000)
    v_target_hold = {"v": Params.vmin}
    hud_n = {"i": 0}
    bg = {"map": None}

    xs = [v for s in wall_segs for v in (s[0], s[2])]
    ys = [v for s in wall_segs for v in (s[1], s[3])]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)

    plt.rcParams["path.simplify"] = True
    fig = plt.figure(figsize=(14.5, 8.0), dpi=80)
    fig.patch.set_facecolor("#0f1419")
    if fig.canvas.manager is not None:
        fig.canvas.manager.set_window_title("惯导建图 · 光电循迹")
    ax = fig.add_axes([0.04, 0.05, 0.64, 0.91])
    ax.set_facecolor("#151b22")

    wall_lc = LineCollection(
        [((x1, y1), (x2, y2)) for x1, y1, x2, y2 in wall_segs],
        colors="#3d5a6c",
        linewidths=1.6,
        zorder=2,
    )
    ax.add_collection(wall_lc)
    ax.plot(pts[:, 0], pts[:, 1], color="#f0c14a", linewidth=1.4, solid_capstyle="round", zorder=3, alpha=0.95)

    trail_line, = ax.plot([], [], color="#ff8a4c", linewidth=1.4, alpha=0.85, zorder=4, animated=True)
    map_line, = ax.plot([], [], color="#5ee0c0", linewidth=1.2, alpha=0.9, zorder=3, animated=True)
    route_lc = LineCollection([], colors="#5ee0c0", linewidths=2.0, zorder=3.5)
    ax.add_collection(route_lc)
    ins_dot, = ax.plot([], [], color="#5ee0c0", marker="o", markersize=5.5, linestyle="None", zorder=8, animated=True)
    sensor_bar, = ax.plot([], [], color="#9ad1ff", linewidth=2.6, solid_capstyle="round", zorder=7, animated=True)
    body = Polygon(car_poly(car.x, car.y, car.yaw), closed=True, facecolor="#ffb347",
                   edgecolor="#fff3d6", linewidth=1.0, zorder=6, animated=True)
    ax.add_patch(body)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(xmin - 40, xmax + 40)
    ax.set_ylim(ymin - 40, ymax + 40)
    ax.set_xlabel("X (mm)", color="#c8d0d8")
    ax.set_ylabel("Y (mm)", color="#c8d0d8")
    ax.tick_params(colors="#9aa7b5")
    for spine in ax.spines.values():
        spine.set_color("#3a4654")

    panel = fig.add_axes([0.70, 0.05, 0.28, 0.91])
    panel.set_facecolor("#1a222c")
    panel.set_xticks([])
    panel.set_yticks([])
    for spine in panel.spines.values():
        spine.set_color("#3a4654")
    panel.text(0.5, 0.985, "控制参数", ha="center", va="top", color="#e8eef4", fontsize=13, transform=panel.transAxes)
    panel.text(0.5, 0.948, "青绿=直道可矫正  黄=出入弯  橙=弯道", ha="center", va="top",
               color="#9aa7b5", fontsize=8.5, transform=panel.transAxes)

    panel.text(0.5, 0.90, "跨度大取最外侧，否则左右平均", ha="center", va="top",
               color="#9aa7b5", fontsize=8.5, transform=panel.transAxes)

    hud = ax.text(
        0.015, 0.02, "", transform=ax.transAxes, va="bottom", ha="left",
        color="#e8eef4", fontsize=9, animated=True, zorder=12,
        bbox=dict(boxstyle="round,pad=0.35", facecolor="#1c242e", edgecolor="#3a4654", alpha=0.88),
    )

    def add_slider(y, label, vmin, vmax, val, fmt="%.3f", valstep=None):
        sax = fig.add_axes([0.78, y, 0.175, 0.022])
        sax.set_facecolor("#151b22")
        kw = dict(valinit=val, valfmt=fmt, color="#3d8fd1")
        if valstep is not None:
            kw["valstep"] = valstep
        s = Slider(sax, label, vmin, vmax, **kw)
        s.label.set_color("#d5dee6")
        s.valtext.set_color("#e8eef4")
        return s

    s_span = add_slider(0.818, "跨度阈值", 0.0, 50.0, Params.span_th, "%.0f")
    s_kp = add_slider(0.776, "转向 Kp", 0.001, 0.06, Params.kp_steer)
    s_kd = add_slider(0.734, "转向 Kd", 0.000, 0.012, Params.kd_steer)
    s_gyro = add_slider(0.692, "陀螺抑制", 0.0, 0.25000, Params.k_gyro, "%.5f", valstep=0.00001)
    s_vkp = add_slider(0.650, "速度 Kp", 0.05, 1.50, Params.kp_speed)
    s_vki = add_slider(0.608, "速度 Ki", 0.00, 0.04, Params.ki_speed, "%.4f")
    s_vkd = add_slider(0.566, "速度 Kd", 0.00, 4.00, Params.kd_speed, "%.2f")
    s_vmax = add_slider(0.524, "最大速度", 120.0, 2500.0, Params.vmax, "%.0f")
    s_vmin = add_slider(0.482, "最小速度", 0.0, 800.0, Params.vmin, "%.0f")
    s_look = add_slider(0.452, "前瞻mm", 40.0, 250.0, Params.look_mm, "%.0f")
    s_abrk = add_slider(0.422, "刹车acc", 400.0, 8000.0, Params.a_brake, "%.0f")
    s_clong = add_slider(0.392, "纵向阻尼", 0.0, 8.0, Params.c_long, "%.2f")
    s_clat = add_slider(0.362, "横向阻尼", 0.0, 20.0, Params.c_lat, "%.2f")
    s_gmu = add_slider(0.332, "角漂μ°/s", -2.00, 2.00, Params.gyro_mu, "%.2f")
    s_gsig = add_slider(0.302, "角漂σ°/s", 0.00, 2.00, Params.gyro_sigma, "%.2f")
    s_emu = add_slider(0.272, "编码μ", -1.00, 1.00, Params.enc_mu, "%.2f")
    s_esig = add_slider(0.242, "编码σ", 0.00, 1.00, Params.enc_sigma, "%.2f")
    s_ew = add_slider(0.220, "编码k|ω|", 0.00, 0.50, Params.enc_w, "%.3f")
    s_smin = add_slider(0.200, "最短直道", 80.0, 600.0, Params.straight_min, "%.0f")
    s_cmarg = add_slider(0.180, "出入弯", 20.0, 200.0, Params.corner_margin, "%.0f")
    s_ckp = add_slider(0.160, "矫正Kp", 0.0, 0.020, Params.corr_kp, "%.4f")
    s_cenc = add_slider(0.140, "编码修正", 0.0, 4.00, Params.corr_enc_k, "%.2f")
    sliders = {
        "span_th": s_span,
        "kp_steer": s_kp,
        "kd_steer": s_kd,
        "k_gyro": s_gyro,
        "kp_speed": s_vkp,
        "ki_speed": s_vki,
        "kd_speed": s_vkd,
        "vmax": s_vmax,
        "vmin": s_vmin,
        "look_mm": s_look,
        "a_brake": s_abrk,
        "c_long": s_clong,
        "c_lat": s_clat,
        "gyro_mu": s_gmu,
        "gyro_sigma": s_gsig,
        "enc_mu": s_emu,
        "enc_sigma": s_esig,
        "enc_w": s_ew,
        "straight_min": s_smin,
        "corner_margin": s_cmarg,
        "corr_kp": s_ckp,
        "corr_enc_k": s_cenc,
    }

    def on_span(v):
        Params.span_th = v

    def on_kp(v):
        Params.kp_steer = v

    def on_kd(v):
        Params.kd_steer = v

    def on_gyro(v):
        Params.k_gyro = float(v)

    def on_vkp(v):
        Params.kp_speed = v

    def on_vki(v):
        Params.ki_speed = v

    def on_vkd(v):
        Params.kd_speed = v

    def _rebuild_if_follow():
        if car.route.frozen:
            car.route.rebuild_speed()

    def on_vmax(v):
        Params.vmax = max(v, Params.vmin + 5.0)
        _rebuild_if_follow()

    def on_vmin(v):
        Params.vmin = min(v, Params.vmax - 5.0)
        _rebuild_if_follow()

    def on_look(v):
        Params.look_mm = float(v)
        _rebuild_if_follow()

    def on_abrk(v):
        Params.a_brake = float(v)
        _rebuild_if_follow()

    def on_clong(v):
        Params.c_long = v

    def on_clat(v):
        Params.c_lat = v

    def on_gmu(v):
        Params.gyro_mu = float(v)

    def on_gsig(v):
        Params.gyro_sigma = float(v)

    def on_emu(v):
        Params.enc_mu = float(v)

    def on_esig(v):
        Params.enc_sigma = float(v)

    def on_ew(v):
        Params.enc_w = float(v)

    def _reclass():
        if car.route.frozen:
            car.route.classify_straights()
            bg["map"] = None

    def on_smin(v):
        Params.straight_min = float(v)
        _reclass()

    def on_cmarg(v):
        Params.corner_margin = float(v)
        _reclass()

    def on_ckp(v):
        Params.corr_kp = float(v)

    def on_cenc(v):
        Params.corr_enc_k = float(v)

    s_span.on_changed(on_span)
    s_kp.on_changed(on_kp)
    s_kd.on_changed(on_kd)
    s_gyro.on_changed(on_gyro)
    s_vkp.on_changed(on_vkp)
    s_vki.on_changed(on_vki)
    s_vkd.on_changed(on_vkd)
    s_vmax.on_changed(on_vmax)
    s_vmin.on_changed(on_vmin)
    s_look.on_changed(on_look)
    s_abrk.on_changed(on_abrk)
    s_clong.on_changed(on_clong)
    s_clat.on_changed(on_clat)
    s_gmu.on_changed(on_gmu)
    s_gsig.on_changed(on_gsig)
    s_emu.on_changed(on_emu)
    s_esig.on_changed(on_esig)
    s_ew.on_changed(on_ew)
    s_smin.on_changed(on_smin)
    s_cmarg.on_changed(on_cmarg)
    s_ckp.on_changed(on_ckp)
    s_cenc.on_changed(on_cenc)

    ax_pause = fig.add_axes([0.73, 0.108, 0.10, 0.026])
    ax_reset = fig.add_axes([0.85, 0.108, 0.10, 0.026])
    ax_mapm = fig.add_axes([0.73, 0.080, 0.10, 0.026])
    ax_runm = fig.add_axes([0.85, 0.080, 0.10, 0.026])
    ax_save = fig.add_axes([0.73, 0.052, 0.10, 0.026])
    ax_load = fig.add_axes([0.85, 0.052, 0.10, 0.026])
    ax_smap = fig.add_axes([0.73, 0.024, 0.10, 0.026])
    ax_lmap = fig.add_axes([0.85, 0.024, 0.10, 0.026])
    ax_corr = fig.add_axes([0.73, 0.001, 0.22, 0.022])
    btn_pause = Button(ax_pause, "暂停", color="#2c3a48", hovercolor="#3d5166")
    btn_reset = Button(ax_reset, "重新运行", color="#2c3a48", hovercolor="#3d5166")
    btn_mapm = Button(ax_mapm, "建图模式", color="#3d6a48", hovercolor="#4d8a58")
    btn_runm = Button(ax_runm, "跑图模式", color="#2c3a48", hovercolor="#3d5166")
    btn_save = Button(ax_save, "保存参数", color="#2c3a48", hovercolor="#3d5166")
    btn_load = Button(ax_load, "导入参数", color="#2c3a48", hovercolor="#3d5166")
    btn_smap = Button(ax_smap, "保存路表", color="#2c3a48", hovercolor="#3d5166")
    btn_lmap = Button(ax_lmap, "载入路表", color="#2c3a48", hovercolor="#3d5166")
    btn_corr = Button(ax_corr, "矫正 关", color="#2c3a48", hovercolor="#3d5166")
    for b in (btn_pause, btn_reset, btn_mapm, btn_runm, btn_save, btn_load, btn_smap, btn_lmap, btn_corr):
        b.label.set_color("#e8eef4")

    param_hint = fig.text(0.84, 0.018, "空格暂停  R重置  C开/关矫正", ha="center", va="center", color="#9aa7b5", fontsize=7.5)

    def do_pause(event=None):
        paused["v"] = not paused["v"]
        btn_pause.label.set_text("继续" if paused["v"] else "暂停")

    def do_reset(event=None):
        keep = ParamBank.mode == "run" and car.route.frozen
        car.reset(keep_map=keep)
        trail.clear()
        trail_line.set_data([], [])
        if not keep:
            map_line.set_data([], [])
        ins_dot.set_data([], [])
        paused["v"] = False
        btn_pause.label.set_text("暂停")

    def _paint_mode_btns():
        on, off = "#3d8fd1", "#2c3a48"
        btn_mapm.ax.set_facecolor(on if ParamBank.mode == "map" else off)
        btn_runm.ax.set_facecolor(on if ParamBank.mode == "run" else off)
        btn_mapm.label.set_text("建图 ●" if ParamBank.mode == "map" else "建图")
        btn_runm.label.set_text("跑图 ●" if ParamBank.mode == "run" else "跑图")
        corr = Params.corr_on >= 0.5
        btn_corr.ax.set_facecolor("#3d8fd1" if corr else off)
        btn_corr.label.set_text("矫正 ●开" if corr else "矫正 关")

    def do_corr_toggle(event=None):
        Params.corr_on = 0.0 if Params.corr_on >= 0.5 else 1.0
        ParamBank.stash()
        _paint_mode_btns()
        param_hint.set_text("直道矫正已开" if Params.corr_on >= 0.5 else "直道矫正已关")

    def _sync_sliders_from_params():
        for key, s in sliders.items():
            v = float(np.clip(getattr(Params, key), s.valmin, s.valmax))
            was = s.eventson
            s.eventson = False
            s.set_val(v)
            s.eventson = was
        if car.route.frozen:
            car.route.rebuild_speed()
            car.route.classify_straights()
            bg["map"] = None

    def _enter_map_mode():
        ParamBank.use("map")
        ParamBank.pending_ui_sync = False
        _sync_sliders_from_params()
        car.reset(keep_map=False)
        trail.clear()
        trail_line.set_data([], [])
        map_line.set_data([], [])
        ins_dot.set_data([], [])
        _paint_mode_btns()
        param_hint.set_text("建图模式 · 光电记表")

    def _enter_run_mode(load_if_needed=True):
        if not car.route.frozen:
            if not load_if_needed:
                param_hint.set_text("没有路表，先建图或载入")
                return False
            path = MAP_PATH if MAP_PATH.exists() else _ask_map_path(save=False)
            if path is None or not Path(path).exists():
                param_hint.set_text("没有路表，先建图或载入")
                return False
            try:
                n = car.route.load_json(path)
                param_hint.set_text(f"已载入路表 {n} 点")
            except Exception as e:
                param_hint.set_text("载入路表失败")
                print("载入路表失败:", e)
                return False
        ParamBank.use("run")
        ParamBank.pending_ui_sync = False
        _sync_sliders_from_params()
        car.reset(keep_map=True)
        if not car.route.frozen:
            param_hint.set_text("路表丢失，仍在建图")
            ParamBank.use("map")
            ParamBank.pending_ui_sync = False
            _sync_sliders_from_params()
            _paint_mode_btns()
            return False
        car.nav = "follow"
        trail.clear()
        trail_line.set_data([], [])
        _paint_mode_btns()
        param_hint.set_text("跑图模式 · 沿路表规划速度")
        return True

    def do_map_mode(event=None):
        _enter_map_mode()

    def do_run_mode(event=None):
        _enter_run_mode(True)

    def do_save(event=None):
        was = paused["v"]
        paused["v"] = True
        try:
            path = _ask_param_path(save=True)
            if path is None:
                return
            path = Path(path)
            if path.suffix.lower() != ".json":
                path = path.with_suffix(".json")
            with open(path, "w", encoding="utf-8") as f:
                json.dump(Params.to_dict(), f, ensure_ascii=False, indent=2)
            ParamBank.stash()
            param_hint.set_text(f"已保存 {path.name}")
            fig.canvas.draw_idle()
        except Exception as e:
            param_hint.set_text("保存失败")
            fig.canvas.draw_idle()
            print("保存参数失败:", e)
        finally:
            paused["v"] = was
            btn_pause.label.set_text("继续" if paused["v"] else "暂停")

    def do_load(event=None):
        was = paused["v"]
        paused["v"] = True
        try:
            path = _ask_param_path(save=False)
            if path is None:
                return
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            Params.from_dict(data)
            ParamBank.stash()
            _sync_sliders_from_params()
            param_hint.set_text(f"已导入 {Path(path).name}")
            bg["map"] = None
            fig.canvas.draw_idle()
        except Exception as e:
            param_hint.set_text("导入失败")
            fig.canvas.draw_idle()
            print("导入参数失败:", e)
        finally:
            paused["v"] = was
            btn_pause.label.set_text("继续" if paused["v"] else "暂停")

    def do_save_map(event=None):
        was = paused["v"]
        paused["v"] = True
        try:
            if not car.route.frozen and len(car.route.xy) < 2 and len(car.route.pts) < 2:
                param_hint.set_text("还没有路表")
                fig.canvas.draw_idle()
                return
            if not car.route.frozen and car.route.pts:
                car.route.freeze()
            path = _ask_map_path(save=True)
            if path is None:
                return
            path = Path(path)
            if path.suffix.lower() != ".json":
                path = path.with_suffix(".json")
            car.route.save_json(path)
            param_hint.set_text(f"已保存路表 {path.name}")
            fig.canvas.draw_idle()
        except Exception as e:
            param_hint.set_text("保存路表失败")
            fig.canvas.draw_idle()
            print("保存路表失败:", e)
        finally:
            paused["v"] = was
            btn_pause.label.set_text("继续" if paused["v"] else "暂停")

    def do_load_map(event=None):
        was = paused["v"]
        paused["v"] = True
        try:
            path = _ask_map_path(save=False)
            if path is None:
                return
            n = car.route.load_json(path)
            _enter_run_mode(load_if_needed=False)
            param_hint.set_text(f"已载入路表 {n} 点，进入跑图")
            fig.canvas.draw_idle()
        except Exception as e:
            param_hint.set_text("载入路表失败")
            fig.canvas.draw_idle()
            print("载入路表失败:", e)
        finally:
            paused["v"] = was
            btn_pause.label.set_text("继续" if paused["v"] else "暂停")

    btn_pause.on_clicked(do_pause)
    btn_reset.on_clicked(do_reset)
    btn_mapm.on_clicked(do_map_mode)
    btn_runm.on_clicked(do_run_mode)
    btn_save.on_clicked(do_save)
    btn_load.on_clicked(do_load)
    btn_smap.on_clicked(do_save_map)
    btn_lmap.on_clicked(do_load_map)
    btn_corr.on_clicked(do_corr_toggle)

    def on_key(event):
        if event.key == " ":
            do_pause()
        elif event.key in ("r", "R"):
            do_reset()
        elif event.key in ("c", "C"):
            do_corr_toggle()

    fig.canvas.mpl_connect("key_press_event", on_key)

    def on_gyro_scroll(event):
        if event.inaxes is not s_gyro.ax:
            return
        step = 0.0001
        if event.key == "shift":
            step = 0.001
        elif event.key in ("control", "ctrl"):
            step = 0.00001
        if getattr(event, "button", None) == "down" or getattr(event, "step", 0) < 0:
            step = -step
        s_gyro.set_val(float(np.clip(round(s_gyro.val + step, 5), s_gyro.valmin, s_gyro.valmax)))

    fig.canvas.mpl_connect("scroll_event", on_gyro_scroll)

    moving = (trail_line, map_line, ins_dot, sensor_bar, body, hud)
    lost_color = {"lost": False}
    route_draw = {"ver": -1}

    def capture_bg(_event=None):
        bg["map"] = fig.canvas.copy_from_bbox(ax.bbox)

    def _refresh_route_colors():
        if car.route.frozen and len(car.route.xy) > 2:
            segs, cols = car.route.color_segments()
            route_lc.set_segments(segs)
            route_lc.set_color(cols)
            map_line.set_data([], [])
        else:
            route_lc.set_segments([])
        route_draw["ver"] = int(car.route.kind_ver)
        bg["map"] = None

    def tick():
        try:
            if ParamBank.pending_ui_sync:
                _sync_sliders_from_params()
                _paint_mode_btns()
                ParamBank.pending_ui_sync = False
            if car.route.kind_ver != route_draw["ver"]:
                _refresh_route_colors()
            if not paused["v"]:
                vt = Params.vmin
                for _ in range(STEPS_PER_FRAME):
                    vt = car.tick()
                v_target_hold["v"] = vt
                trail.append((car.x, car.y))

            body.set_xy(car_poly(car.x, car.y, car.yaw))
            sensor_bar.set_data(car.sensor_xy[:, 0], car.sensor_xy[:, 1])
            ins_dot.set_data([car.odom.x], [car.odom.y])
            if not car.route.frozen:
                mp = np.asarray(car.route.pts, dtype=float) if car.route.pts else None
                if mp is not None and len(mp) > 1:
                    stride = max(1, len(mp) // 2500)
                    map_line.set_data(mp[::stride, 0], mp[::stride, 1])
                else:
                    map_line.set_data([], [])
            if car.lost != lost_color["lost"]:
                lost_color["lost"] = car.lost
                sensor_bar.set_color("#ff5c5c" if car.lost else "#9ad1ff")
            if trail:
                trail_line.set_data(*zip(*trail))

            hud_n["i"] += 1
            if hud_n["i"] % 5 == 0:
                st = "丢边" if car.lost else "在线"
                bank = "跑图" if ParamBank.mode == "run" else "建图"
                if car.nav == "map":
                    mode = f"{bank}记表 {len(car.route.pts)}"
                else:
                    n = max(len(car.route.xy), 1)
                    mode = f"{bank} {100.0 * car.route.i / n:.0f}%  规划{car.v_target:.0f}"
                    if Params.corr_on >= 0.5:
                        mode += "  矫正开"
                        if car.on_straight:
                            mode += f" 直道 e={car.corr_e:+.1f}mm yaw{np.degrees(car.odom.yaw_corr):+.2f}°"
                        else:
                            mode += " 弯/出入弯"
                    else:
                        mode += "  矫正关"
                hud.set_text(
                    f"{mode}  {st}{'  暂停' if paused['v'] else ''}   "
                    f"真({car.x:.0f},{car.y:.0f})  INS({car.odom.x:.2f},{car.odom.y:.2f})   "
                    f"编码 {car.odom.enc_s:.5f}  e={car.error:+.0f}  {car.v:.0f}mm/s"
                )

            canvas = fig.canvas
            if bg["map"] is None:
                canvas.draw()
                _paint_mode_btns()
                capture_bg()
            canvas.restore_region(bg["map"])
            for art in moving:
                ax.draw_artist(art)
            canvas.blit(ax.bbox)
            canvas.flush_events()
        except Exception:
            paused["v"] = True
            import traceback
            traceback.print_exc()

    fig.canvas.mpl_connect("resize_event", lambda _e: bg.__setitem__("map", None))
    timer = fig.canvas.new_timer(interval=FRAME_MS)
    timer.add_callback(tick)
    fig._timer = timer
    fig._btns = (btn_pause, btn_reset, btn_mapm, btn_runm, btn_save, btn_load, btn_smap, btn_lmap, btn_corr, s_span, s_kp, s_kd, s_gyro, s_vkp, s_vki, s_vkd, s_vmax, s_vmin, s_look, s_abrk, s_clong, s_clat, s_gmu, s_gsig, s_emu, s_esig, s_ew, s_smin, s_cmarg, s_ckp, s_cenc)
    fig.canvas.draw()
    _paint_mode_btns()
    capture_bg()
    timer.start()
    plt.show()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        input("出错了，按回车退出...")
