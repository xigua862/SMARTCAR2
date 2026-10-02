#!/usr/bin/env python3
"""Audit route hashes and replay protection decisions from frozen telemetry.

The firmware protection loops run at 1 kHz while ``control.csv`` and
``phototube.csv`` are normally sampled at 200 Hz.  This tool therefore reports
sampled lower bounds/approximations; it deliberately does not claim that a
5 ms CSV row is an exact 1 ms firmware decision.

Examples (from the project root)::

    python tools/audit_protection_history.py --root . --output output
    python tools/audit_protection_history.py --root . --output output --json-out output/protection_audit.json

The stall replay evaluates both the active legacy detector and the new V2
ratio detector.  The adaptive black-stop replay is exact only for logs whose
route hash equals the current ``nav_path.h`` hash.  For older route hashes it
also emits a curvature-based proxy so historical behaviour can still be
screened, but that proxy is explicitly labelled as non-exact.

The summary deliberately reports long/terminal blackout intervals separately
from the raw interval count. A 5 ms telemetry sample can briefly report no
line at a corner; counting every such sample as a possible safety failure
would substantially overstate the risk.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import statistics
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
TELEMETRY_PATH = Path(__file__).resolve().with_name("telemetry_capture.py")


def _load_telemetry_module():
    spec = importlib.util.spec_from_file_location("telemetry_capture", TELEMETRY_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {TELEMETRY_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


TC = _load_telemetry_module()


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def _float(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def _int(row: dict[str, str], key: str, default: int = 0) -> int:
    try:
        return int(float(row.get(key, "")))
    except (TypeError, ValueError):
        return default


def _bool(row: dict[str, str], key: str) -> bool:
    return str(row.get(key, "")).strip().lower() in {"1", "true", "yes", "on"}


def _median_sample_ms(rows: list[dict[str, str]], key: str = "tick_ms") -> float:
    ticks = [_float(row, key) for row in rows]
    deltas = [b - a for a, b in zip(ticks, ticks[1:]) if b > a]
    return statistics.median(deltas) if deltas else 5.0


def _route_hash_from_metadata(run_dir: Path) -> str | None:
    metadata = run_dir / "metadata.json"
    if not metadata.is_file():
        return None
    try:
        value = json.loads(metadata.read_text(encoding="utf-8")).get("route_hash")
    except (OSError, ValueError, TypeError):
        return None
    if not value:
        return None
    try:
        # Metadata is emitted with ``0x`` while some hand-edited/older files
        # use ``0X`` or a bare decimal value.  Compare the numeric value, not
        # the spelling; otherwise an actually matching route is misclassified
        # as a mismatch and the exact adaptive replay is never exercised.
        return f"0x{int(str(value), 0) & 0xFFFFFFFF:08X}"
    except (TypeError, ValueError):
        return str(value).strip().lower()


def _stop_reason(run_dir: Path) -> str:
    metadata = run_dir / "metadata.json"
    if metadata.is_file():
        try:
            value = json.loads(metadata.read_text(encoding="utf-8")).get(
                "stop_reason_name"
            )
            if value:
                return str(value)
        except (OSError, ValueError, TypeError):
            pass
    events = run_dir / "events.csv"
    if events.is_file():
        try:
            rows = _read_csv(events)
            stops = [row for row in rows if row.get("type_name") == "run_stop"]
            if stops:
                return str(stops[-1].get("value1") or "unknown")
        except (OSError, csv.Error):
            pass
    return "unknown"


def _run_stop_timestamp(run_dir: Path) -> float | None:
    """Return the recorded stop event time, when the event stream has one."""
    events = run_dir / "events.csv"
    if not events.is_file():
        return None
    try:
        rows = _read_csv(events)
    except (OSError, csv.Error):
        return None
    stops = [row for row in rows if row.get("type_name") == "run_stop"]
    if not stops:
        return None
    value = stops[-1].get("timestamp_ms")
    try:
        timestamp = float(value)
    except (TypeError, ValueError):
        return None
    return timestamp if math.isfinite(timestamp) else None


def _config_value(config_text: str, name: str, integer: bool = False) -> float | int:
    value = TC._parse_c_numeric_macro(config_text, name, integer=integer)
    if value is None:
        raise ValueError(f"cannot parse {name} from competition_config.h")
    return value


@dataclass
class StallSideAudit:
    candidate_rows: int = 0
    max_candidate_streak_ms: float = 0.0
    first_trigger_ms: float | None = None
    min_ratio: float | None = None


@dataclass
class StallAudit:
    mode: str
    left: StallSideAudit
    right: StallSideAudit
    trigger_side: str | None


def _stall_side(
    rows: list[dict[str, str]],
    side: str,
    *,
    mode: str,
    sample_ms: float,
    duty_percent: int,
    target_min_cmps: float,
    ratio_percent: int,
    min_actual_cmps: float,
    confirm_ms: float,
    grace_ms: float,
    count_max: int,
    soft_limit: float,
) -> StallSideAudit:
    result = StallSideAudit()
    run_ms = 0.0
    fault_ms = 0.0
    streak_start: float | None = None
    previous_tick: float | None = None
    motion_seen = False

    for row in rows:
        tick = _float(row, "tick_ms")
        dt = sample_ms if previous_tick is None else max(1.0, min(50.0, tick - previous_tick))
        previous_tick = tick
        raw = _int(row, f"encoder_{side}")
        target = _float(row, f"target_{side}_cmps")
        actual = _float(row, f"actual_{side}_cmps")
        duty = abs(_float(row, f"pwm_{side}"))
        target_abs = abs(target)
        actual_abs = abs(actual)
        if raw != 0:
            motion_seen = True
        run_ms = min(grace_ms, run_ms + dt)
        armed = motion_seen or run_ms >= grace_ms
        near_limit = duty >= soft_limit * duty_percent / 100.0
        target_running = target_abs >= target_min_cmps
        no_encoder_motion = abs(raw) <= count_max

        if mode == "legacy":
            candidate = near_limit and no_encoder_motion
        else:
            low_speed_limit = max(
                min_actual_cmps, target_abs * ratio_percent / 100.0
            )
            wrong_direction = (
                target_running
                and actual_abs >= min_actual_cmps
                and target * actual < 0.0
            )
            candidate = (
                armed
                and near_limit
                and target_running
                and (no_encoder_motion or actual_abs <= low_speed_limit or wrong_direction)
            )

        if candidate:
            result.candidate_rows += 1
            if streak_start is None:
                streak_start = tick
            streak_ms = max(sample_ms, tick - streak_start + sample_ms)
            result.max_candidate_streak_ms = max(result.max_candidate_streak_ms, streak_ms)
            if target_abs > 0.0:
                ratio = actual_abs / target_abs
                result.min_ratio = ratio if result.min_ratio is None else min(result.min_ratio, ratio)
            # Legacy increments one millisecond per firmware tick.  The CSV is
            # decimated, so use elapsed sample time only as a conservative
            # replay estimate for both modes.
            fault_ms += dt
            required_ms = 100.0 if mode == "legacy" else confirm_ms
            if result.first_trigger_ms is None and fault_ms >= required_ms:
                result.first_trigger_ms = tick
        else:
            streak_start = None
            if mode == "legacy":
                fault_ms = max(0.0, fault_ms - 2.0 * dt)
            else:
                fault_ms = max(0.0, fault_ms - max(2.0, 2.0 * dt))

    return result


def _stall_audit(rows: list[dict[str, str]], config: dict[str, float | int], mode: str) -> StallAudit:
    sample_ms = _median_sample_ms(rows)
    kwargs = dict(
        sample_ms=sample_ms,
        duty_percent=int(config["duty_percent"]),
        target_min_cmps=float(config["target_min_cmps"]),
        ratio_percent=int(config["ratio_percent"]),
        min_actual_cmps=float(config["min_actual_cmps"]),
        confirm_ms=float(config["confirm_ms"]),
        grace_ms=float(config["grace_ms"]),
        count_max=int(config["count_max"]),
        soft_limit=float(config["soft_limit"]),
    )
    left = _stall_side(rows, "l", mode=mode, **kwargs)
    right = _stall_side(rows, "r", mode=mode, **kwargs)
    trigger_side = None
    if left.first_trigger_ms is not None and right.first_trigger_ms is not None:
        trigger_side = "L" if left.first_trigger_ms <= right.first_trigger_ms else "R"
    elif left.first_trigger_ms is not None:
        trigger_side = "L"
    elif right.first_trigger_ms is not None:
        trigger_side = "R"
    return StallAudit(mode, left, right, trigger_side)


def _nearest_progress(nav_rows: list[dict[str, str]], tick_ms: float) -> float | None:
    if not nav_rows:
        return None
    row = min(nav_rows, key=lambda item: abs(_float(item, "timestamp_ms") - tick_ms))
    value = row.get("progress_points")
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _progress_span(nav_rows: list[dict[str, str]], start_ms: float, end_ms: float) -> float:
    values = [
        _float(row, "progress_points")
        for row in nav_rows
        if start_ms <= _float(row, "timestamp_ms") <= end_ms
    ]
    if not values:
        return 0.0
    start = _nearest_progress(nav_rows, start_ms)
    return max(0.0, max(values) - (start if start is not None else values[0])) * 5.0


def _adaptive_distance_from_features(
    start_points: float,
    feature_progress: list[tuple[float, int]],
    *,
    path_sample_mm: float,
    min_distance_mm: float,
    max_distance_mm: float,
    turn_margin_mm: float,
    cluster_gap_mm: float,
) -> float:
    target: float | None = None
    for progress, feature_type in feature_progress:
        if feature_type not in (1, 2) or progress <= start_points:
            continue
        target = progress
        previous = progress
        for next_progress, next_type in feature_progress:
            if next_progress <= previous or next_progress - previous > cluster_gap_mm / path_sample_mm:
                if next_progress > previous:
                    break
                continue
            if next_type not in (1, 2):
                break
            target = next_progress
            previous = next_progress
        break
    if target is None:
        target = start_points
    distance = (target - start_points) * path_sample_mm + turn_margin_mm
    return max(min_distance_mm, min(max_distance_mm, distance))


def _proxy_adaptive_distance(
    start_points: float,
    nav_rows: list[dict[str, str]],
    *,
    path_sample_mm: float,
    min_distance_mm: float,
    max_distance_mm: float,
    turn_margin_mm: float,
    cluster_gap_mm: float,
    turn_curvature_threshold: float,
) -> float:
    turns = [
        (_float(row, "progress_points"), abs(_float(row, "curvature_per_mm")))
        for row in nav_rows
        if _float(row, "progress_points") > start_points + 2.0
        and abs(_float(row, "curvature_per_mm")) >= turn_curvature_threshold
    ]
    if not turns:
        target = max((_float(row, "progress_points") for row in nav_rows), default=start_points)
        return max(min_distance_mm, min(max_distance_mm, (target - start_points) * path_sample_mm + turn_margin_mm))
    target = turns[0][0]
    previous = target
    for progress, _ in turns[1:]:
        if progress - previous > cluster_gap_mm / path_sample_mm:
            break
        target = progress
        previous = progress
    distance = (target - start_points) * path_sample_mm + turn_margin_mm
    return max(min_distance_mm, min(max_distance_mm, distance))


def _blackout_audit(
    run_dir: Path,
    *,
    current_route_hash: str,
    route_reference: dict[str, Any],
    config: dict[str, float | int],
    use_proxy_for_mismatch: bool,
    screening_min_interval_ms: float,
) -> dict[str, Any]:
    photo_path = run_dir / "phototube.csv"
    nav_path = run_dir / "navigation.csv"
    result: dict[str, Any] = {
        "intervals": 0,
        "longest_interval_ms": 0.0,
        "adaptive_extensions": 0,
        "line_loss_extensions": 0,
        "fixed_reached_but_adaptive_not_reached": 0,
        "intervals_with_progress": 0,
        "screened_intervals": 0,
        "screened_adaptive_extensions": 0,
        "screened_fixed_reached_but_adaptive_not_reached": 0,
        "terminal": None,
        "route_hash": _route_hash_from_metadata(run_dir),
        "exact_route": _route_hash_from_metadata(run_dir) == current_route_hash,
        "proxy_used": False,
    }
    if not photo_path.is_file() or not nav_path.is_file():
        return result
    photo_rows = _read_csv(photo_path)
    nav_rows = _read_csv(nav_path)
    sample_ms = _median_sample_ms(photo_rows)
    route_hash = _route_hash_from_metadata(run_dir)
    exact_route = route_hash == current_route_hash
    if not exact_route and not use_proxy_for_mismatch:
        result["skipped_reason"] = "route_hash_mismatch"
        return result
    result["proxy_used"] = not exact_route
    result["route_hash"] = route_hash
    feature_progress = []
    if exact_route:
        feature_progress = list(
            zip(
                route_reference["scan_progress"],
                route_reference["scan_type"],
            )
        )

    stop_reason = _stop_reason(run_dir)
    stop_timestamp = _run_stop_timestamp(run_dir)
    terminal_entry: dict[str, Any] | None = None
    index = 0
    while index < len(photo_rows):
        if _bool(photo_rows[index], "line_valid"):
            index += 1
            continue
        start = index
        while index + 1 < len(photo_rows) and not _bool(photo_rows[index + 1], "line_valid"):
            index += 1
        end = index
        start_ms = _float(photo_rows[start], "tick_ms")
        end_ms = _float(photo_rows[end], "tick_ms")
        start_points = _nearest_progress(nav_rows, start_ms)
        interval_ms = end_ms - start_ms + sample_ms
        traveled_mm = 0.0
        adaptive_mm: float | None = None
        if start_points is not None:
            result["intervals_with_progress"] += 1
            if exact_route:
                adaptive_mm = _adaptive_distance_from_features(
                    start_points,
                    feature_progress,
                    path_sample_mm=float(config["path_sample_mm"]),
                    min_distance_mm=float(config["black_min_mm"]),
                    max_distance_mm=float(config["historical_black_max_mm"]),
                    turn_margin_mm=float(config["historical_black_margin_mm"]),
                    cluster_gap_mm=float(config["historical_black_cluster_gap_mm"]),
                )
            else:
                adaptive_mm = _proxy_adaptive_distance(
                    start_points,
                    nav_rows,
                    path_sample_mm=float(config["path_sample_mm"]),
                    min_distance_mm=float(config["black_min_mm"]),
                    max_distance_mm=float(config["historical_black_max_mm"]),
                    turn_margin_mm=float(config["historical_black_margin_mm"]),
                    cluster_gap_mm=float(config["historical_black_cluster_gap_mm"]),
                    turn_curvature_threshold=0.001,
                )
            fixed_mm = float(config["black_min_mm"])
            traveled_mm = _progress_span(nav_rows, start_ms, end_ms)
            if adaptive_mm > fixed_mm + 0.5:
                result["adaptive_extensions"] += 1
            if stop_reason == "line_lost_after_200mm_black_scan" and adaptive_mm > fixed_mm + 0.5:
                result["line_loss_extensions"] += 1
            if traveled_mm >= fixed_mm and traveled_mm < adaptive_mm:
                result["fixed_reached_but_adaptive_not_reached"] += 1
            # Very short sampled gaps are commonly a single bad frame or a
            # corner transition.  Keep the raw count above, but use a
            # duration/progress gate for the safety conclusion below.
            screened = (
                interval_ms >= screening_min_interval_ms or
                traveled_mm >= fixed_mm
            )
            if screened:
                result["screened_intervals"] += 1
                if adaptive_mm > fixed_mm + 0.5:
                    result["screened_adaptive_extensions"] += 1
                if traveled_mm >= fixed_mm and traveled_mm < adaptive_mm:
                    result["screened_fixed_reached_but_adaptive_not_reached"] += 1

            terminal_match = (
                stop_reason == "line_lost_after_200mm_black_scan" and
                (stop_timestamp is None or
                 abs(stop_timestamp - end_ms) <=
                 max(3.0 * sample_ms, 25.0))
            )
            if terminal_match:
                terminal_entry = {
                    "interval_ms": interval_ms,
                    "traveled_mm": traveled_mm,
                    "fixed_mm": fixed_mm,
                    "adaptive_mm": adaptive_mm,
                    "fixed_reached": traveled_mm >= fixed_mm,
                    "adaptive_not_reached": (
                        traveled_mm < adaptive_mm
                        if adaptive_mm is not None else None
                    ),
                    "screened": screened,
                }
        result["intervals"] += 1
        result["longest_interval_ms"] = max(
            result["longest_interval_ms"], interval_ms
        )
        index += 1
    if terminal_entry is None and stop_reason == "line_lost_after_200mm_black_scan":
        # Some older captures omit a run_stop event or end the phototube
        # stream a few samples before it.  The final blackout is the only
        # defensible fallback, and remains labelled as approximate.
        final_false = [
            row for row in photo_rows if not _bool(row, "line_valid")
        ]
        if final_false:
            end_ms = _float(final_false[-1], "tick_ms")
            terminal_entry = {
                "interval_ms": None,
                "traveled_mm": None,
                "fixed_mm": float(config["black_min_mm"]),
                "adaptive_mm": None,
                "fixed_reached": None,
                "adaptive_not_reached": None,
                "screened": False,
                "fallback": True,
            }
    result["terminal"] = terminal_entry
    return result


def _current_route(root: Path) -> tuple[str, dict[str, Any]]:
    path_text = (root / "code" / "nav_path.h").read_text(encoding="utf-8", errors="replace")
    points = TC._parse_c_integer_array(path_text, "nav_path_x_0")
    if not points:
        raise ValueError("cannot read nav_path_x_0")
    reference = TC._load_route_reference(root, 0, len(points))
    if reference is None:
        raise ValueError("cannot load current route reference")
    route_hash = TC._embedded_route_hash(root, reference)
    return f"0x{route_hash:08X}", reference


def _run_dirs(output_root: Path) -> Iterable[Path]:
    for control in sorted(output_root.rglob("control.csv")):
        run_dir = control.parent
        if (run_dir / "metadata.json").is_file() or (run_dir / "events.csv").is_file():
            yield run_dir


def _format_ms(value: float | None) -> str:
    return "-" if value is None else f"{value:.0f} ms"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT, help="project root")
    parser.add_argument("--output", type=Path, default=None, help="telemetry output root")
    parser.add_argument("--json-out", type=Path, default=None, help="write machine-readable report")
    parser.add_argument(
        "--no-proxy-adaptive",
        action="store_true",
        help="skip adaptive black-stop proxy for historical route-hash mismatches",
    )
    parser.add_argument(
        "--min-blackout-ms",
        type=float,
        default=100.0,
        help=(
            "minimum blackout duration used by the safety screening summary "
            "(raw interval counts are always retained; default: 100 ms)"
        ),
    )
    args = parser.parse_args()
    root = args.root.resolve()
    output_root = (args.output or (root / "output")).resolve()
    config_text = (root / "code" / "competition_config.h").read_text(encoding="utf-8")
    config: dict[str, float | int] = {
        "duty_percent": _config_value(config_text, "CONTROL_STALL_DUTY_PERCENT", True),
        "target_min_cmps": _config_value(config_text, "CONTROL_STALL_TARGET_MIN_CMPS"),
        "ratio_percent": _config_value(config_text, "CONTROL_STALL_SPEED_RATIO_PERCENT", True),
        "min_actual_cmps": _config_value(config_text, "CONTROL_STALL_MIN_ACTUAL_CMPS"),
        "confirm_ms": _config_value(config_text, "CONTROL_STALL_CONFIRM_MS", True),
        "grace_ms": _config_value(config_text, "CONTROL_STALL_START_GRACE_MS", True),
        "count_max": _config_value(config_text, "CONTROL_STALL_ENCODER_COUNT_MAX", True),
        "soft_limit": 10000.0,
        "black_min_mm": _config_value(config_text, "PHOTOTUBE_ARRAY_BLACK_STOP_DISTANCE_MM"),
        # Historical adaptive policy values are analysis constants, not live
        # firmware knobs.  Keep them here so old logs can still be compared
        # after the runtime branch was removed.
        "historical_black_max_mm": 450.0,
        "historical_black_margin_mm": 120.0,
        "historical_black_cluster_gap_mm": 350.0,
        "path_sample_mm": _config_value(
            (root / "code" / "nav_path.h").read_text(encoding="utf-8", errors="replace"),
            "NAV_PATH_SAMPLE_MM",
        ),
    }
    current_hash, route_reference = _current_route(root)
    report: dict[str, Any] = {
        "current_route_hash": current_hash,
        "current_route_points": len(route_reference["x"]),
        "config": config,
        "runs": [],
    }
    for run_dir in _run_dirs(output_root):
        try:
            control_rows = _read_csv(run_dir / "control.csv")
            legacy = _stall_audit(control_rows, config, "legacy")
            v2 = _stall_audit(control_rows, config, "v2")
            black = _blackout_audit(
                run_dir,
                current_route_hash=current_hash,
                route_reference=route_reference,
                config=config,
                use_proxy_for_mismatch=not args.no_proxy_adaptive,
                screening_min_interval_ms=max(0.0, args.min_blackout_ms),
            )
            report["runs"].append(
                {
                    "run": str(run_dir.relative_to(root)),
                    "stop_reason": _stop_reason(run_dir),
                    "route_hash": _route_hash_from_metadata(run_dir),
                    "legacy_stall": asdict(legacy),
                    "v2_stall": asdict(v2),
                    "black_stop": black,
                }
            )
        except (OSError, csv.Error, ValueError, KeyError) as exc:
            report["runs"].append({"run": str(run_dir.relative_to(root)), "error": str(exc)})

    valid = [item for item in report["runs"] if "error" not in item]
    motor_faults = [item for item in valid if item["stop_reason"] == "motor_fault"]
    path_done = [item for item in valid if item["stop_reason"] == "path_done"]
    line_lost = [
        item for item in valid if item["stop_reason"] == "line_lost_after_200mm_black_scan"
    ]
    v2_triggered = [item for item in valid if item["v2_stall"]["trigger_side"]]
    path_done_v2_false = [item for item in path_done if item["v2_stall"]["trigger_side"]]
    black_items = [item["black_stop"] for item in valid]
    exact_black = [item for item in black_items if item.get("exact_route")]
    proxy_black = [item for item in black_items if item.get("proxy_used")]
    line_loss_terminal = [
        item for item in black_items
        if item.get("terminal") is not None
    ]
    terminal_screened = [
        item for item in line_loss_terminal
        if item["terminal"].get("screened")
    ]
    terminal_fixed_reached = [
        item for item in terminal_screened
        if item["terminal"].get("fixed_reached")
    ]
    terminal_proxy_longer = [
        item for item in terminal_screened
        if item["terminal"].get("adaptive_mm") is not None
        and item["terminal"].get("adaptive_mm") > item["terminal"].get("fixed_mm", 0.0) + 0.5
    ]
    terminal_proxy_may_delay = [
        item for item in terminal_fixed_reached
        if item["terminal"].get("adaptive_not_reached")
    ]
    route_hash_counts: dict[str, int] = {}
    for item in valid:
        route_hash = item.get("route_hash") or "missing"
        route_hash_counts[route_hash] = route_hash_counts.get(route_hash, 0) + 1
    report["summary"] = {
        "runs": len(valid),
        "parse_errors": len(report["runs"]) - len(valid),
        "motor_fault_runs": len(motor_faults),
        "path_done_runs": len(path_done),
        "line_lost_runs": len(line_lost),
        "v2_trigger_runs": len(v2_triggered),
        "v2_trigger_on_path_done_runs": len(path_done_v2_false),
        "adaptive_exact_route_runs": len(exact_black),
        "adaptive_proxy_route_runs": len(proxy_black),
        "route_hash_match_runs": len(exact_black),
        "route_hash_mismatch_runs": len(valid) - len(exact_black),
        "route_hash_counts": route_hash_counts,
        "adaptive_extensions": sum(item["adaptive_extensions"] for item in black_items),
        "line_loss_extensions": sum(item["line_loss_extensions"] for item in black_items),
        "fixed_reached_but_adaptive_not_reached": sum(
            item["fixed_reached_but_adaptive_not_reached"] for item in black_items
        ),
        "screened_intervals": sum(item["screened_intervals"] for item in black_items),
        "screened_adaptive_extensions": sum(
            item["screened_adaptive_extensions"] for item in black_items
        ),
        "screened_fixed_reached_but_adaptive_not_reached": sum(
            item["screened_fixed_reached_but_adaptive_not_reached"]
            for item in black_items
        ),
        "line_loss_terminal_runs": len(line_loss_terminal),
        "line_loss_terminal_screened_runs": len(terminal_screened),
        "line_loss_terminal_fixed_reached_runs": len(terminal_fixed_reached),
        "line_loss_terminal_proxy_longer_runs": len(terminal_proxy_longer),
        "line_loss_terminal_proxy_may_delay_runs": len(terminal_proxy_may_delay),
        "screening_min_interval_ms": max(0.0, args.min_blackout_ms),
    }
    print(f"current route hash: {current_hash} ({len(route_reference['x'])} points)")
    print(
        "runs={runs} motor_fault={motor_fault_runs} path_done={path_done_runs} "
        "line_lost={line_lost_runs}".format(**report["summary"])
    )
    print(
        "V2 replay: trigger_runs={v2_trigger_runs}, "
        "path_done false triggers={v2_trigger_on_path_done_runs}".format(**report["summary"])
    )
    print(
        "adaptive black-stop: exact_route_runs={adaptive_exact_route_runs}, "
        "proxy_route_runs={adaptive_proxy_route_runs}, "
        "route_hash_mismatch_runs={route_hash_mismatch_runs}".format(
            **report["summary"]
        )
    )
    print(
        "adaptive screening (proxy is non-exact): intervals={screened_intervals}, "
        "terminal_line_loss={line_loss_terminal_screened_runs}, "
        "proxy_longer={line_loss_terminal_proxy_longer_runs}, "
        "may_delay_fixed_stop={line_loss_terminal_proxy_may_delay_runs}".format(
            **report["summary"]
        )
    )
    for item in motor_faults:
        v2 = item["v2_stall"]
        print(
            f"  motor_fault {item['run']}: V2 side={v2['trigger_side'] or '-'} "
            f"L={_format_ms(v2['left']['first_trigger_ms'])} "
            f"R={_format_ms(v2['right']['first_trigger_ms'])}"
        )
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"wrote {args.json_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
