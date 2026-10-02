#!/usr/bin/env python3
"""Analyze longitudinal speed planning and execution from a TC264 run.

The report is intentionally independent of pandas so it can run with the
bundled Python installation.  It separates the planned body acceleration from
the measured acceleration and from wheel acceleration caused by steering.

Examples:
  python tools/analyze_speed_profile.py output/wansai11
  python tools/analyze_speed_profile.py output/wansai11 --json-out speed.json
  python tools/analyze_speed_profile.py output --all --json-out speed_all.json
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from typing import Any, Iterable


def number(value: Any, default: float | None = None) -> float | None:
    if value is None or value == "":
        return default
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def percentile(values: Iterable[float], fraction: float) -> float | None:
    data = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not data:
        return None
    if len(data) == 1:
        return data[0]
    position = (len(data) - 1) * fraction
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return data[low]
    return data[low] + (data[high] - data[low]) * (position - low)


def rms(values: Iterable[float]) -> float | None:
    data = [float(v) for v in values if math.isfinite(float(v))]
    return math.sqrt(sum(v * v for v in data) / len(data)) if data else None


def median_dt_ms(times: list[float]) -> float:
    deltas = [b - a for a, b in zip(times, times[1:]) if 0.0 < b - a <= 1000.0]
    return statistics.median(deltas) if deltas else 5.0


def nearest_value(rows: list[dict[str, str]], timestamp_ms: float, key: str) -> float | None:
    best: tuple[float, float] | None = None
    for row in rows:
        t = number(row.get("timestamp_ms"))
        value = number(row.get(key))
        if t is None or value is None:
            continue
        candidate = (abs(t - timestamp_ms), value)
        if best is None or candidate[0] < best[0]:
            best = candidate
    return None if best is None else best[1]


def phase_name(plan_slope: float | None) -> str:
    if plan_slope is None:
        return "unknown"
    if plan_slope > 100.0:
        return "accelerating"
    if plan_slope < -100.0:
        return "decelerating"
    return "steady"


def local_slope(rows: list[dict[str, Any]], index: int, key: str, half_window: int = 2) -> float | None:
    """Least-squares slope over a short time window, in value/second."""
    lo = max(0, index - half_window)
    hi = min(len(rows), index + half_window + 1)
    points = [(row["time_ms"] / 1000.0, row[key])
              for row in rows[lo:hi]
              if row.get(key) is not None]
    if len(points) < 2:
        return None
    mean_x = sum(point[0] for point in points) / len(points)
    mean_y = sum(point[1] for point in points) / len(points)
    denom = sum((x - mean_x) ** 2 for x, _ in points)
    if denom <= 0.0:
        return None
    return sum((x - mean_x) * (y - mean_y) for x, y in points) / denom


def contiguous_events(rows: list[dict[str, Any]], name: str) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None
    for row in rows:
        if row["phase"] != name:
            if current is not None:
                events.append(current)
                current = None
            continue
        if current is None:
            current = {"start": row, "end": row, "count": 1}
        else:
            current["end"] = row
            current["count"] += 1
    if current is not None:
        events.append(current)
    return [event for event in events if event["count"] >= 2]


def event_report(event: dict[str, Any], nav_rows: list[dict[str, str]], decel_cmps2: float) -> dict[str, Any]:
    start = event["start"]
    end = event["end"]
    start_v = float(start["plan_v_cmps"])
    end_v = float(end["plan_v_cmps"])
    distance_mm = max(0.0, (start_v * start_v - end_v * end_v) /
                      (2.0 * decel_cmps2) * 10.0) if decel_cmps2 > 0 else None
    start_progress = nearest_value(nav_rows, start["time_ms"], "progress_points")
    end_progress = nearest_value(nav_rows, end["time_ms"], "progress_points")
    progress_delta_mm = None
    if start_progress is not None and end_progress is not None:
        progress_delta_mm = (end_progress - start_progress) * 5.0
    return {
        "start_ms": start["time_ms"],
        "end_ms": end["time_ms"],
        "duration_ms": end["time_ms"] - start["time_ms"],
        "samples": event["count"],
        "plan_start_cmps": start_v,
        "plan_end_cmps": end_v,
        "plan_delta_cmps": end_v - start_v,
        "actual_start_cmps": start["actual_avg_cmps"],
        "actual_end_cmps": end["actual_avg_cmps"],
        "actual_delta_cmps": end["actual_avg_cmps"] - start["actual_avg_cmps"],
        "command_body_accel_min_cmps2": min(
            row["command_body_accel_cmps2"] for row in event["rows"]
        ) if "rows" in event else None,
        "theoretical_distance_mm_at_configured_decel": distance_mm,
        "nav_progress_delta_mm": progress_delta_mm,
    }


def analyze_run(run_dir: Path) -> dict[str, Any]:
    summary_path = run_dir / "analysis_summary.json"
    control_path = run_dir / "control.csv"
    if not control_path.exists():
        raise FileNotFoundError(f"missing {control_path}")
    summary = json.loads(summary_path.read_text(encoding="utf-8")) if summary_path.exists() else {}
    parameters = summary.get("parameters", {})
    configured_max_speed = number(parameters.get("speed_profile_max_speed_cmps"))
    configured_lateral_accel = number(
        parameters.get("speed_profile_max_lateral_accel_cmps2")
    )
    configured_accel = number(parameters.get("speed_profile_max_accel_cmps2"), 3500.0) or 3500.0
    configured_decel = number(parameters.get("speed_profile_max_decel_cmps2"), 3500.0) or 3500.0

    control_raw = read_csv(control_path)
    control: list[dict[str, Any]] = []
    for row in control_raw:
        time_ms = number(row.get("tick_ms"), number(row.get("timestamp_ms")))
        actual = number(row.get("actual_avg_cmps"))
        plan = number(row.get("plan_v_cmps"))
        if time_ms is None or actual is None or plan is None:
            continue
        control.append({
            "time_ms": time_ms,
            "actual_avg_cmps": actual,
            "plan_v_cmps": plan,
            "accel_l_cmps2": number(row.get("accel_l_cmps2"), 0.0) or 0.0,
            "accel_r_cmps2": number(row.get("accel_r_cmps2"), 0.0) or 0.0,
            "pwm_l": number(row.get("pwm_l"), 0.0) or 0.0,
            "pwm_r": number(row.get("pwm_r"), 0.0) or 0.0,
            "speed_error_cmps": number(
                row.get("speed_error_actual_minus_plan_cmps"), actual - plan
            ) or 0.0,
            "final_turn_cmps": number(row.get("final_turn_cmps"), 0.0) or 0.0,
        })
    control.sort(key=lambda row: row["time_ms"])
    if not control:
        raise ValueError(f"no usable control rows in {control_path}")

    nav_rows = read_csv(run_dir / "navigation.csv") if (run_dir / "navigation.csv").exists() else []
    for index, row in enumerate(control):
        measured_accel = local_slope(control, index, "actual_avg_cmps")
        plan_slope = local_slope(control, index, "plan_v_cmps")
        row["measured_body_accel_cmps2"] = measured_accel
        row["plan_slope_cmps2"] = plan_slope
        row["command_body_accel_cmps2"] = 0.5 * (
            row["accel_l_cmps2"] + row["accel_r_cmps2"]
        )
        row["phase"] = phase_name(plan_slope)

    acceleration_rows = [r for r in control if r["phase"] == "accelerating"]
    deceleration_rows = [r for r in control if r["phase"] == "decelerating"]
    steady_rows = [r for r in control if r["phase"] == "steady"]

    phase_stats: dict[str, Any] = {}
    for name, rows in (("accelerating", acceleration_rows),
                       ("decelerating", deceleration_rows),
                       ("steady", steady_rows),
                       ("all", control)):
        measured = [r["measured_body_accel_cmps2"] for r in rows
                    if r["measured_body_accel_cmps2"] is not None]
        command = [r["command_body_accel_cmps2"] for r in rows]
        errors = [r["speed_error_cmps"] for r in rows]
        phase_stats[name] = {
            "samples": len(rows),
            "duration_ms": (rows[-1]["time_ms"] - rows[0]["time_ms"]) if rows else 0.0,
            "measured_accel_p05_cmps2": percentile(measured, 0.05),
            "measured_accel_p50_cmps2": percentile(measured, 0.50),
            "measured_accel_p95_cmps2": percentile(measured, 0.95),
            "command_body_accel_min_cmps2": min(command) if command else None,
            "command_body_accel_max_cmps2": max(command) if command else None,
            "speed_error_mean_cmps": (sum(errors) / len(errors)) if errors else None,
            "speed_error_rms_cmps": rms(errors),
            "speed_error_p95_abs_cmps": percentile((abs(v) for v in errors), 0.95),
        }

    braking_groups = contiguous_events(control, "decelerating")
    for event in braking_groups:
        event["rows"] = [row for row in control
                          if event["start"]["time_ms"] <= row["time_ms"] <= event["end"]["time_ms"]]
    braking_events = [event_report(event, nav_rows, configured_decel)
                      for event in braking_groups]

    pwm_values = [max(abs(r["pwm_l"]), abs(r["pwm_r"])) for r in control]
    saturation_rows = [r for r, pwm in zip(control, pwm_values) if pwm >= 9900.0]
    turn_values = [abs(r["final_turn_cmps"]) for r in control]
    report = {
        "run": run_dir.name,
        "source": str(run_dir),
        "duration_ms": control[-1]["time_ms"] - control[0]["time_ms"],
        "samples": len(control),
        "configured": {
            "max_speed_cmps": configured_max_speed,
            "max_lateral_accel_cmps2": configured_lateral_accel,
            "max_accel_cmps2": configured_accel,
            "max_decel_cmps2": configured_decel,
            "plan_max_cmps": max(r["plan_v_cmps"] for r in control),
            "plan_min_cmps": min(r["plan_v_cmps"] for r in control),
        },
        "phase_stats": phase_stats,
        "braking_events": braking_events,
        "actuator": {
            "pwm_abs_max": max(pwm_values) if pwm_values else None,
            "pwm_saturation_rows": len(saturation_rows),
            "pwm_saturation_ratio": len(saturation_rows) / len(control),
            "final_turn_abs_p95_cmps": percentile(turn_values, 0.95),
            "final_turn_abs_max_cmps": max(turn_values) if turn_values else None,
        },
        "quality": {
            "has_measurable_acceleration": bool(acceleration_rows),
            "has_measurable_braking": bool(deceleration_rows),
            "braking_event_count": len(braking_events),
            "telemetry_drops": (summary.get("run", {}).get("drops", {}) if summary else {}),
            "stop_reason": summary.get("run", {}).get("stop_reason_name") if summary else None,
        },
    }
    return report


def discover_runs(path: Path, all_runs: bool) -> list[Path]:
    if not path.is_dir():
        raise FileNotFoundError(path)
    runs = [child for child in path.iterdir()
            if child.is_dir() and (child / "control.csv").exists()]
    # An output root may also contain stale top-level CSV files.  Prefer its
    # named run directories whenever they exist.
    if runs:
        if not all_runs:
            runs.sort(key=lambda child: child.stat().st_mtime, reverse=True)
            return runs[:1]
        return sorted(runs, key=lambda child: child.name)
    if (path / "control.csv").exists():
        return [path]
    if not all_runs:
        raise FileNotFoundError(f"no run directory below {path}")
    return []


def print_report(report: dict[str, Any]) -> None:
    configured = report["configured"]
    all_stats = report["phase_stats"]["all"]
    accel = report["phase_stats"]["accelerating"]
    decel = report["phase_stats"]["decelerating"]
    print(f"[{report['run']}] {report['duration_ms']:.0f} ms, {report['samples']} control rows")
    print(f"  plan: {configured['plan_min_cmps']:.1f}..{configured['plan_max_cmps']:.1f} cm/s; "
          f"configured accel/decel {configured['max_accel_cmps2']:.0f}/"
          f"{configured['max_decel_cmps2']:.0f} cm/s^2")
    if configured["max_speed_cmps"] is not None:
        print(f"  envelope: max {configured['max_speed_cmps']:.1f} cm/s, "
              f"lateral {configured['max_lateral_accel_cmps2']!s} cm/s^2")
    print(f"  measured body accel p05/p50/p95: "
          f"{all_stats['measured_accel_p05_cmps2']!s}/"
          f"{all_stats['measured_accel_p50_cmps2']!s}/"
          f"{all_stats['measured_accel_p95_cmps2']!s} cm/s^2")
    print(f"  accelerating rows={accel['samples']}, decelerating rows={decel['samples']}, "
          f"braking events={len(report['braking_events'])}")
    print(f"  speed error RMS/P95={all_stats['speed_error_rms_cmps']!s}/"
          f"{all_stats['speed_error_p95_abs_cmps']!s} cm/s; "
          f"PWM>=9900 rows={report['actuator']['pwm_saturation_rows']}")
    for event in report["braking_events"]:
        print(f"  brake {event['start_ms']:.0f}..{event['end_ms']:.0f} ms: "
              f"plan {event['plan_start_cmps']:.1f}->{event['plan_end_cmps']:.1f} cm/s, "
              f"theoretical {event['theoretical_distance_mm_at_configured_decel']:.1f} mm")
    if not report["quality"]["has_measurable_braking"]:
        print("  NOTE: no sustained planned deceleration; this run cannot validate braking capability.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path, help="run directory or output root")
    parser.add_argument("--all", action="store_true", help="analyze every run below an output root")
    parser.add_argument("--json-out", type=Path, help="write the report JSON")
    args = parser.parse_args()
    runs = discover_runs(args.path, args.all)
    reports = [analyze_run(run) for run in runs]
    for report in reports:
        print_report(report)
    payload: Any = reports if args.all or len(reports) != 1 else reports[0]
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
                                 encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
