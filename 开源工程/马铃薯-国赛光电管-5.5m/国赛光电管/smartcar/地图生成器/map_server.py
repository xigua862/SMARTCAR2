# -*- coding: utf-8 -*-
"""地图编辑器本地服务:双击 启动地图编辑器.bat 使用。

GET  /             静态文件(map_editor.html 等)
GET  /save-info    告诉网页后端在线,返回可写目标列表
POST /save         {name, content} → 写入当前地图 JSON 或主工程 nav_path.h
POST /solve-anytime  路线请求 → 优先返回带最短性证明的精确路线

网页检测到本服务后允许保存 yusai.json/juesai.json，并在用户点击导出时
写入主工程的 code/nav_path.h。其它路径仍不会被服务接受。
"""
import argparse
import copy
import json
import os
import threading
import time
import webbrowser
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

from fast_route_solver import RESULT_SCHEMA as FAST_RESULT_SCHEMA, solve_anytime_route

try:
    from exact_route_solver import (
        RESULT_SCHEMA as EXACT_RESULT_SCHEMA,
        solve_exact_route,
    )
except ImportError:
    EXACT_RESULT_SCHEMA = None
    solve_exact_route = None

# Keep the fast schema name available for the legacy error responses and for
# clients that use the bounded fallback.
RESULT_SCHEMA = FAST_RESULT_SCHEMA

ROOT = Path(__file__).resolve().parent
PORT = 8642
MAX_JSON_BODY_BYTES = 8 * 1024 * 1024
# The browser sends this value in its request.  It is intentionally longer
# than the old 120 s fast-planner budget: exact enumeration must be allowed to
# close the complete shortest topology-length class.
EXACT_DEFAULT_TIMEOUT_S = 900.0
EXACT_MAX_TIMEOUT_S = 3600.0
SOLVER_LOCK = threading.Lock()
PROGRESS_LOCK = threading.Lock()
SOLVER_PROGRESS = {
    "state": "idle",
    "phase": "idle",
    "elapsed_s": 0.0,
}

# 允许写入的目标(白名单):文件名 → 绝对路径。想写别的文件在这里加。
TARGETS = {
    "yusai.json": ROOT / "yusai.json",
    "juesai.json": ROOT / "juesai.json",
    "nav_path.h": (
        ROOT.parent
        / "Crazy_Car"
        / "Seekfree_TC264_Opensource_Library"
        / "code"
        / "nav_path.h"
    ),
}


def _edge_pair(edge):
    """Return an exact-protocol [u, v] pair from either editor edge shape."""
    if isinstance(edge, (list, tuple)) and len(edge) == 2:
        return [str(edge[0]), str(edge[1])]
    if isinstance(edge, dict):
        u = edge.get("u", edge.get("a"))
        v = edge.get("v", edge.get("b"))
        if u is not None and v is not None:
            # Keep virtual connectors explicit.  They are topological
            # shortcuts and must not be mistaken for physical scan edges by
            # the shared graph verifier.
            edge_type = edge.get("type", "physical")
            return {"u": str(u), "v": str(v), "type": str(edge_type)}
    raise ValueError(f"边格式无效: {edge!r}")


def _prepare_exact_request(data):
    """Translate the current fast-editor payload to exact solver protocol v6."""
    if not isinstance(data, dict):
        raise ValueError("请求必须是 JSON 对象")
    payload = copy.deepcopy(data)
    payload["schema"] = "smartcar.exact-route.request.v6"
    payload.pop("routes", None)
    payload.pop("route_candidates", None)
    payload["edges"] = [_edge_pair(edge) for edge in payload.get("edges", [])]

    route_settings = payload.setdefault("route_settings", {})
    speed_plan = route_settings.get("speed_plan") or {}
    straight_cmps = float(speed_plan.get(
        "straight_cmps", speed_plan.get("max_speed_cmps", 700.0)
    ))
    corner_cmps = float(speed_plan.get(
        "corner_cmps", min(straight_cmps, 450.0)
    ))
    if not straight_cmps > 0:
        straight_cmps = 700.0
    if not corner_cmps > 0:
        corner_cmps = min(straight_cmps, 450.0)
    corner_cmps = min(corner_cmps, straight_cmps)

    # The exact solver uses a separate, explicit speed ledger.  These values
    # affect only its secondary travel-time tie break, never the primary path
    # length proof.
    payload["route_settings"] = route_settings
    route_settings.update({
        "geometry_mode": "pure_pursuit_polyline",
        "radius": 0,
        "ease_mm": 0,
        "smoothing_profile": "overlapping_circular_arc_blend.v5",
        "composite_profile_version": "circular_arc_curvature_blend.v9",
        "composite_partition_policy": (
            "overlap_seeded_minimum_expansion_backtracking.v2"
        ),
        "composite_corridor_profile": "single_corner_exact_envelope.v2",
        "composite_collision_profile": "physical_edges_and_smooth_route.v2",
        "max_curvature_per_mm": float(
            route_settings.get("max_curvature_per_mm") or (1.0 / 150.0)
        ),
        "smoothing_ease_mm": 0,
        "composite_short_gap_mm": float(
            route_settings.get("composite_short_gap_mm") or
            route_settings.get("reverse_bend_preview_mm") or 100.0
        ),
        "speed_plan": {
            "straight_cmps": straight_cmps,
            "corner_cmps": corner_cmps,
            "max_accel_cmps2": float(speed_plan.get("max_accel_cmps2", 3000.0)),
            "max_decel_cmps2": float(speed_plan.get("max_decel_cmps2", 5000.0)),
            "safety_margin_mm": float(speed_plan.get("safety_margin_mm", 80.0)),
            "brake_lead_mm": float(speed_plan.get("brake_lead_mm", 100.0)),
            "start_speed_cmps": float(speed_plan.get("start_speed_cmps", 0.0)),
            "time_quantum_us": int(speed_plan.get("time_quantum_us", 1)),
            "time_model": "constant_accel_speed_cap.v1",
        },
    })

    # Preserve a valid fixed departure direction when the current map does
    # not store one.  A stale direction is replaced by an adjacent neighbour.
    start = payload.get("start") or {}
    start_name = start.get("name")
    pairs = payload["edges"]
    neighbours = [
        pair[1] if pair[0] == start_name else pair[0]
        for pair in pairs if start_name in pair
    ]
    requested_dir = route_settings.get("start_dir")
    if requested_dir not in neighbours:
        requested_dir = "P3" if "P3" in neighbours else (neighbours[0] if neighbours else None)
    if requested_dir:
        route_settings["start_dir"] = requested_dir

    solve_options = dict(payload.get("solve_options") or {})
    timeout_s = float(solve_options.get("timeout_s") or EXACT_DEFAULT_TIMEOUT_S)
    solve_options["timeout_s"] = min(EXACT_MAX_TIMEOUT_S, max(1.0, timeout_s))
    solve_options["return_to_start"] = bool(solve_options.get("return_to_start", True))
    solve_options["candidate_count"] = max(1, int(solve_options.get("candidate_count", 1)))
    payload["solve_options"] = solve_options
    return payload


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def end_headers(self):
        # The editor generates a firmware ABI header.  Never let an old page
        # kept in the browser cache overwrite nav_path.h with an older schema.
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def _json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        raw_length = self.headers.get("Content-Length")
        if raw_length is None:
            raise ValueError("缺少 Content-Length")
        length = int(raw_length)
        if length <= 0 or length > MAX_JSON_BODY_BYTES:
            raise ValueError(f"JSON 请求大小必须在 1..{MAX_JSON_BODY_BYTES} 字节内")
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_GET(self):
        if self.path == "/save-info":
            self._json({"targets": {k: str(v) for k, v in TARGETS.items()}})
            return
        if self.path.split("?", 1)[0] == "/solve-progress":
            with PROGRESS_LOCK:
                progress = dict(SOLVER_PROGRESS)
            self._json(progress)
            return
        super().do_GET()

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/solve-anytime":
            try:
                data = self._read_json()
            except Exception as exc:  # noqa: BLE001 本地协议错误直接回给网页
                self._json({
                    "schema": EXACT_RESULT_SCHEMA or FAST_RESULT_SCHEMA,
                    "status": "not_proven_optimal",
                    "proven_optimal": False,
                    "commit_allowed": False,
                    "reason": "invalid_json",
                    "message": str(exc),
                }, 400)
                return
            exact_mode = solve_exact_route is not None
            solver = solve_exact_route if exact_mode else solve_anytime_route
            solver_schema = EXACT_RESULT_SCHEMA if exact_mode else FAST_RESULT_SCHEMA
            solver_data = data
            if exact_mode:
                try:
                    solver_data = _prepare_exact_request(data)
                except Exception as exc:  # noqa: BLE001 协议转换错误
                    self._json({
                        "schema": solver_schema,
                        "status": "not_proven_optimal",
                        "proven_optimal": False,
                        "commit_allowed": False,
                        "reason": "invalid_request",
                        "message": f"精确协议转换失败: {exc}",
                    }, 400)
                    return
            if not SOLVER_LOCK.acquire(blocking=False):
                self._json({
                    "schema": solver_schema,
                    "status": "not_proven_optimal",
                    "proven_optimal": False,
                    "commit_allowed": False,
                    "reason": "solver_busy",
                    "message": "另一个路线求解事务仍在运行",
                }, 409)
                return
            started_at = time.perf_counter()

            def publish_progress(update):
                with PROGRESS_LOCK:
                    SOLVER_PROGRESS.clear()
                    SOLVER_PROGRESS.update({
                        "state": "running",
                        "started_at": started_at,
                        "updated_at": time.time(),
                        **update,
                    })

            publish_progress({"phase": "starting", "elapsed_s": 0.0})
            progress_stop = threading.Event()

            def progress_heartbeat():
                # Refresh wall-clock time while the solver is between progress
                # callbacks; phase and counters remain truthful.
                while not progress_stop.wait(1.0):
                    with PROGRESS_LOCK:
                        SOLVER_PROGRESS["state"] = "running"
                        SOLVER_PROGRESS["updated_at"] = time.time()
                        SOLVER_PROGRESS["elapsed_s"] = time.perf_counter() - started_at

            heartbeat_thread = threading.Thread(
                target=progress_heartbeat,
                name="fast-solver-progress",
                daemon=True,
            )
            heartbeat_thread.start()
            try:
                try:
                    result = solver(solver_data, progress_callback=publish_progress)
                except Exception as exc:  # noqa: BLE001 服务层最后一道拒绝回退保护
                    result = {
                        "schema": solver_schema,
                        "status": "not_proven_optimal",
                        "proven_optimal": False,
                        "commit_allowed": False,
                        "reason": "solver_error",
                        "message": (
                            f"{'精确' if exact_mode else '快速'}求解器异常终止: {exc}"
                        ),
                    }
            finally:
                progress_stop.set()
                heartbeat_thread.join(timeout=0.2)
                with PROGRESS_LOCK:
                    SOLVER_PROGRESS.update({
                        "state": (
                            "complete"
                            if result.get("status") in {"optimal", "feasible"}
                            else "failed"
                        ),
                        "finished_at": time.time(),
                        "elapsed_s": time.perf_counter() - started_at,
                        "reason": result.get("reason"),
                        "message": result.get("message"),
                    })
                SOLVER_LOCK.release()
            status_code = 200
            if result.get("reason") == "solver_unavailable":
                status_code = 503
            elif result.get("reason") in {
                "invalid_request",
                "unsupported_schema",
            }:
                status_code = 400
            self._json(result, status_code)
            return

        if path != "/save":
            self.send_error(404)
            return
        try:
            data = self._read_json()
            target = TARGETS.get(data.get("name", ""))
            if target is None:
                self._json({"ok": False, "error": "文件名不在白名单里"}, 403)
                return
            content = data.get("content")
            if not isinstance(content, str):
                self._json({"ok": False, "error": "content 必须是字符串"}, 400)
                return
            # 所有目标都采用目标目录临时文件 + 原子替换，避免规划器或编辑器
            # 中途异常时把现有路线截断成半个文件。目标路径由白名单固定给出。
            tmp = target.with_name(target.name + ".tmp")
            tmp.write_text(content, encoding="utf-8", newline="\n")
            back = tmp.read_text(encoding="utf-8")
            if back != content:
                try:
                    tmp.unlink()
                except OSError:
                    pass
                self._json({"ok": False, "error": "回读校验不一致,文件可能被占用"}, 500)
                return
            os.replace(tmp, target)
            back = target.read_text(encoding="utf-8")
            if back != content:
                self._json({"ok": False, "error": "原子替换后回读校验不一致"}, 500)
                return
            st = target.stat()
            self._json({
                "ok": True,
                "path": str(target),
                "bytes": st.st_size,
                "mtime": time.strftime("%H:%M:%S", time.localtime(st.st_mtime)),
            })
        except Exception as e:  # noqa: BLE001 局部工具,报错原样回给网页
            self._json({"ok": False, "error": str(e)}, 500)

    def log_message(self, fmt, *args):
        pass  # 不刷终端


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SmartCar map editor server")
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be in 1..65535")

    url = f"http://localhost:{args.port}/map_editor.html"
    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    print(f"地图编辑器已启动: {url}")
    print("仅允许保存地图 JSON: yusai.json / juesai.json")
    print("关闭本窗口即退出。")
    try:
        # 多线程:浏览器会开预连接空占连接,单线程服务器会被堵死
        ThreadingHTTPServer(("127.0.0.1", args.port), Handler).serve_forever()
    except OSError:
        print(f"端口 {args.port} 被占用:大概已经开着一个了,直接用浏览器里那个页面即可。")
        input("按回车退出...")
