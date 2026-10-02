#!/usr/bin/env python3
"""基于地图生成器完整白线网络的一维光电管位姿可观测性模型。

与早期只把 nav_path.h 中的平滑轨迹当作白线的简化模型不同，本脚本明确区分：

* ``route``：车辆按地图生成器生成的提前切弯平滑轨迹行驶；
* ``edges``：地面真实存在的黑底白线网络，包含 L 弯、三叉和十字；
* ``sensor``：轮轴前 100 mm 的 16 路模拟光电阵列，间距 8 mm；
* ``measurement``：保留 16 路原始模拟量，不在三叉处压成单一质心；
* ``state``：扫描窗口起点的 x、y、偏航角三个自由度。

脚本复刻 map_editor.html 的共线优化、圆弧/缓和曲线和相邻弯半径压缩规则，
随后用数值雅可比判断三个自由度是否可观测，并用阻尼最小二乘演示位姿纠正。
它是离线建模工具，不是固件测试，不修改 tests/，也不编译车上工程。
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MAP = ROOT.parents[1] / "地图生成器" / "juesai_shuangjuesai.json"
DEFAULT_NAV_HEADER = ROOT / "code" / "nav_path.h"
DEFAULT_OUTPUT = ROOT / "docs" / "full_map_phototube_simulation"


@dataclass(frozen=True)
class SensorConfig:
    count: int = 16
    spacing_mm: float = 8.0
    forward_mm: float = 100.0
    lateral_mm: float = 0.0
    mounting_deg: float = 2.0
    line_width_mm: float = 20.0
    edge_softness_mm: float = 1.25
    noise_std: float = 0.015

    @property
    def channel_u_mm(self) -> np.ndarray:
        return (
            np.arange(self.count, dtype=float) - 0.5 * (self.count - 1)
        ) * self.spacing_mm

    @property
    def channel_points_body_mm(self) -> np.ndarray:
        beta = math.radians(self.mounting_deg)
        axis = np.array([-math.sin(beta), math.cos(beta)], dtype=float)
        origin = np.array([self.forward_mm, self.lateral_mm], dtype=float)
        return origin + self.channel_u_mm[:, None] * axis[None, :]


@dataclass
class PhysicalMap:
    source: Path
    title: str
    points: dict[str, np.ndarray]
    edge_names: list[tuple[str, str]]
    edges_a_mm: np.ndarray
    edges_b_mm: np.ndarray
    degrees: dict[str, int]
    routes: list[dict[str, Any]]
    route_settings: dict[str, Any]


@dataclass
class Segment:
    kind: str
    length_mm: float
    start_s_mm: float
    end_s_mm: float
    ax: float = 0.0
    ay: float = 0.0
    bx: float = 0.0
    by: float = 0.0
    points: np.ndarray | None = None
    point_step_mm: float = 0.0


@dataclass
class Turn:
    name: str
    vertex_index: int
    direction: str
    angle_deg: float
    requested_radius_mm: float
    actual_radius_mm: float
    tangent_before_mm: float
    tangent_after_mm: float
    start_s_mm: float
    end_s_mm: float
    outgoing_straight_mm: float
    graph_degree: int


@dataclass
class RouteGeometry:
    name: str
    route_index: int
    segments: list[Segment]
    turns: list[Turn]
    total_length_mm: float
    warnings: list[str]


@dataclass
class WindowAnalysis:
    route_index: int
    route_name: str
    turn_name: str
    graph_degree: int
    window: str
    start_s_mm: float
    end_s_mm: float
    sample_count: int
    visible_count: int
    singular_values: np.ndarray
    rank: int
    null_direction_local: np.ndarray
    outgoing_straight_mm: float
    profiles: np.ndarray
    poses: np.ndarray
    relative_poses: np.ndarray


def rotation(angle_rad: float) -> np.ndarray:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array([[c, -s], [s, c]], dtype=float)


def pose_matrix(x_mm: float, y_mm: float, yaw_rad: float) -> np.ndarray:
    result = np.eye(3, dtype=float)
    result[:2, :2] = rotation(yaw_rad)
    result[:2, 2] = (x_mm, y_mm)
    return result


def inverse_pose(pose: np.ndarray) -> np.ndarray:
    result = np.eye(3, dtype=float)
    result[:2, :2] = pose[:2, :2].T
    result[:2, 2] = -(result[:2, :2] @ pose[:2, 2])
    return result


def transform_points(pose: np.ndarray, points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    return points @ pose[:2, :2].T + pose[:2, 2]


def pose_yaw(pose: np.ndarray) -> float:
    return math.atan2(float(pose[1, 0]), float(pose[0, 0]))


def wrap_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def load_physical_map(path: Path) -> PhysicalMap:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"地图根节点不是对象: {path}")
    points: dict[str, np.ndarray] = {
        str(name): np.asarray(xy, dtype=float)
        for name, xy in data.get("nodes", {}).items()
    }
    start = data.get("start")
    if start and start.get("coord"):
        points[str(start.get("name", "S0"))] = np.asarray(start["coord"], dtype=float)
    for item in data.get("required_points", []):
        if item.get("name") and item.get("coord"):
            points[str(item["name"])] = np.asarray(item["coord"], dtype=float)
    for item in data.get("nocal_points", []) or []:
        if item.get("name") and item.get("coord"):
            points[str(item["name"])] = np.asarray(item["coord"], dtype=float)

    edge_names: list[tuple[str, str]] = []
    edge_a: list[np.ndarray] = []
    edge_b: list[np.ndarray] = []
    degrees: dict[str, int] = {}
    for raw_edge in data.get("edges", []):
        a_name, b_name = (
            (raw_edge[0], raw_edge[1])
            if isinstance(raw_edge, list)
            else (raw_edge.get("u"), raw_edge.get("v"))
        )
        if a_name not in points or b_name not in points:
            raise ValueError(f"白线边端点不存在: {a_name!r} -> {b_name!r}")
        edge_names.append((str(a_name), str(b_name)))
        edge_a.append(points[str(a_name)])
        edge_b.append(points[str(b_name)])
        degrees[str(a_name)] = degrees.get(str(a_name), 0) + 1
        degrees[str(b_name)] = degrees.get(str(b_name), 0) + 1

    return PhysicalMap(
        source=path,
        title=str(data.get("title", path.stem)),
        points=points,
        edge_names=edge_names,
        edges_a_mm=np.asarray(edge_a, dtype=float),
        edges_b_mm=np.asarray(edge_b, dtype=float),
        degrees=degrees,
        routes=list(data.get("routes", [])),
        route_settings=dict(data.get("route_settings", {})),
    )


def build_corner_curve(
    theta_rad: float, radius_mm: float, easing_length_mm: float
) -> tuple[np.ndarray, float, float, float, float]:
    """逐行复刻 map_editor.html/buildCornerCurve 的中点积分。"""
    curvature = 1.0 / radius_mm
    easing = easing_length_mm
    arc_angle = theta_rad - curvature * easing
    if arc_angle < 0.0:
        easing = theta_rad / curvature
        arc_angle = 0.0
    arc_length = arc_angle / curvature
    total = 2.0 * easing + arc_length
    count = max(4, math.ceil(total / 2.0))
    ds = total / count
    points = np.zeros((count + 1, 3), dtype=float)
    x = 0.0
    y = 0.0
    heading = 0.0
    for index in range(count):
        middle_s = (index + 0.5) * ds
        if easing > 1.0e-9 and middle_s < easing:
            local_curvature = curvature * middle_s / easing
        elif middle_s < easing + arc_length or easing <= 1.0e-9:
            local_curvature = curvature
        else:
            local_curvature = curvature * (total - middle_s) / easing
        middle_heading = heading + local_curvature * ds * 0.5
        x += ds * math.cos(middle_heading)
        y += ds * math.sin(middle_heading)
        heading += local_curvature * ds
        points[index + 1] = (x, y, heading)
    sine = math.sin(heading)
    tangent_out = y / sine
    tangent_in = x - y * math.cos(heading) / sine
    return points, ds, total, tangent_in, tangent_out


def build_route_geometry(
    physical_map: PhysicalMap, route: dict[str, Any], route_index: int
) -> RouteGeometry:
    raw: list[dict[str, Any]] = []
    for original_index, name in enumerate(route.get("nodes", [])):
        if name not in physical_map.points:
            raise ValueError(f"路线 {route.get('name')} 找不到点 {name}")
        xy = physical_map.points[name]
        item = {
            "x": float(xy[0]),
            "y": float(xy[1]),
            "name": str(name),
            "original_index": original_index,
        }
        if raw and math.hypot(item["x"] - raw[-1]["x"], item["y"] - raw[-1]["y"]) < 1.0e-6:
            continue
        raw.append(item)
    if len(raw) < 2:
        raise ValueError(f"路线 {route.get('name')} 少于两个有效点")

    vertices = raw
    if physical_map.route_settings.get("skip_collinear", True) and len(vertices) > 2:
        filtered = [vertices[0]]
        for index in range(1, len(vertices) - 1):
            previous = filtered[-1]
            current = vertices[index]
            following = vertices[index + 1]
            first = np.array(
                [current["x"] - previous["x"], current["y"] - previous["y"]],
                dtype=float,
            )
            second = np.array(
                [following["x"] - current["x"], following["y"] - current["y"]],
                dtype=float,
            )
            first_length = float(np.linalg.norm(first))
            second_length = float(np.linalg.norm(second))
            if first_length < 1.0e-6 or second_length < 1.0e-6:
                filtered.append(current)
                continue
            dot = float(first @ second / (first_length * second_length))
            if dot < math.cos(0.01):
                filtered.append(current)
        filtered.append(vertices[-1])
        vertices = filtered

    shifts = route.get("turn_shifts") or []
    if any(value not in (None, 0, 0.0) for value in shifts):
        raise ValueError(
            "当前模型尚未复刻 turn_shifts；所选完整地图实际没有非零提前/延拐表"
        )

    warnings: list[str] = []
    count = len(vertices)
    global_radius = float(physical_map.route_settings.get("radius", 200.0))
    easing = max(0.0, float(physical_map.route_settings.get("ease", 0.0)))
    radii = route.get("radii") or []
    corners: list[dict[str, Any] | None] = [None] * count

    for index in range(1, count - 1):
        previous = vertices[index - 1]
        current = vertices[index]
        following = vertices[index + 1]
        vector_in = np.array(
            [current["x"] - previous["x"], current["y"] - previous["y"]],
            dtype=float,
        )
        vector_out = np.array(
            [following["x"] - current["x"], following["y"] - current["y"]],
            dtype=float,
        )
        length_in = float(np.linalg.norm(vector_in))
        length_out = float(np.linalg.norm(vector_out))
        direction_in = vector_in / length_in
        direction_out = vector_out / length_out
        cross = float(
            direction_in[0] * direction_out[1]
            - direction_in[1] * direction_out[0]
        )
        dot = float(np.clip(direction_in @ direction_out, -1.0, 1.0))
        theta = math.acos(dot)
        if theta < 0.01:
            continue
        if theta > math.pi - 0.05:
            warnings.append(f"{current['name']} 接近 180°，按折点处理")
            corners[index] = {"uturn": True, "tangent": 0.0}
            continue
        original_index = int(current["original_index"])
        requested_radius = (
            float(radii[original_index])
            if original_index < len(radii)
            and radii[original_index] is not None
            and float(radii[original_index]) > 0.0
            else global_radius
        )
        local_points, local_step, curve_length, tangent_in, tangent_out = (
            build_corner_curve(theta, requested_radius, easing)
        )
        corners[index] = {
            "uturn": False,
            "direction_in": direction_in,
            "direction_out": direction_out,
            "theta": theta,
            "sign": 1.0 if cross > 0.0 else -1.0,
            "requested_radius": requested_radius,
            "local_points": local_points,
            "local_step": local_step,
            "curve_length": curve_length,
            "tangent_in_original": tangent_in,
            "tangent_out_original": tangent_out,
            "tangent": max(tangent_in, tangent_out),
            "factor": 1.0,
            "clamped": False,
        }

    segment_factor = np.ones(count - 1, dtype=float)
    for index in range(count - 1):
        length = math.hypot(
            vertices[index + 1]["x"] - vertices[index]["x"],
            vertices[index + 1]["y"] - vertices[index]["y"],
        )
        required = (
            (float(corners[index]["tangent"]) if corners[index] else 0.0)
            + (float(corners[index + 1]["tangent"]) if corners[index + 1] else 0.0)
        )
        if required > length * 0.98:
            segment_factor[index] = length * 0.98 / required
    for index in range(1, count - 1):
        corner = corners[index]
        if not corner or corner["uturn"]:
            continue
        factor = min(1.0, float(segment_factor[index - 1]), float(segment_factor[index]))
        corner["factor"] = factor
        corner["tangent"] *= factor
        corner["clamped"] = factor < 1.0

    raw_segments: list[Segment] = []
    pending_turns: list[tuple[Turn, Segment]] = []
    cursor = np.array([vertices[0]["x"], vertices[0]["y"]], dtype=float)

    def append_line(destination: np.ndarray) -> None:
        nonlocal cursor
        length = float(np.linalg.norm(destination - cursor))
        if length > 1.0e-6:
            raw_segments.append(
                Segment(
                    kind="line",
                    length_mm=length,
                    start_s_mm=0.0,
                    end_s_mm=0.0,
                    ax=float(cursor[0]),
                    ay=float(cursor[1]),
                    bx=float(destination[0]),
                    by=float(destination[1]),
                )
            )
        cursor = destination.copy()

    for index in range(1, count - 1):
        vertex = vertices[index]
        corner = corners[index]
        if not corner:
            continue
        vertex_point = np.array([vertex["x"], vertex["y"]], dtype=float)
        if corner["uturn"]:
            append_line(vertex_point)
            continue
        factor = float(corner["factor"])
        direction_in = np.asarray(corner["direction_in"])
        start = vertex_point - direction_in * (
            float(corner["tangent_in_original"]) * factor
        )
        append_line(start)

        incoming_angle = math.atan2(float(direction_in[1]), float(direction_in[0]))
        cosine = math.cos(incoming_angle)
        sine = math.sin(incoming_angle)
        local = np.asarray(corner["local_points"], dtype=float)
        mirrored_y = float(corner["sign"]) * local[:, 1]
        world = np.empty_like(local)
        world[:, 0] = start[0] + factor * (
            local[:, 0] * cosine - mirrored_y * sine
        )
        world[:, 1] = start[1] + factor * (
            local[:, 0] * sine + mirrored_y * cosine
        )
        world[:, 2] = incoming_angle + float(corner["sign"]) * local[:, 2]
        curve_segment = Segment(
            kind="curve",
            length_mm=float(corner["curve_length"]) * factor,
            start_s_mm=0.0,
            end_s_mm=0.0,
            points=world,
            point_step_mm=float(corner["local_step"]) * factor,
        )
        raw_segments.append(curve_segment)
        cursor = world[-1, :2].copy()
        pending_turns.append(
            (
                Turn(
                    name=str(vertex["name"]),
                    vertex_index=int(vertex["original_index"]),
                    direction="RIGHT" if float(corner["sign"]) > 0.0 else "LEFT",
                    angle_deg=math.degrees(float(corner["theta"])),
                    requested_radius_mm=float(corner["requested_radius"]),
                    actual_radius_mm=float(corner["requested_radius"]) * factor,
                    tangent_before_mm=float(corner["tangent_in_original"]) * factor,
                    tangent_after_mm=float(corner["tangent_out_original"]) * factor,
                    start_s_mm=0.0,
                    end_s_mm=0.0,
                    outgoing_straight_mm=0.0,
                    graph_degree=physical_map.degrees.get(str(vertex["name"]), 0),
                ),
                curve_segment,
            )
        )
        if corner["clamped"]:
            warnings.append(
                f"{vertex['name']} 半径压缩到 {float(corner['requested_radius']) * factor:.1f} mm"
            )

    append_line(
        np.array([vertices[-1]["x"], vertices[-1]["y"]], dtype=float)
    )
    accumulated = 0.0
    for segment in raw_segments:
        segment.start_s_mm = accumulated
        accumulated += segment.length_mm
        segment.end_s_mm = accumulated

    turns: list[Turn] = []
    for turn, curve_segment in pending_turns:
        turn.start_s_mm = curve_segment.start_s_mm
        turn.end_s_mm = curve_segment.end_s_mm
        segment_index = raw_segments.index(curve_segment)
        if (
            segment_index + 1 < len(raw_segments)
            and raw_segments[segment_index + 1].kind == "line"
        ):
            turn.outgoing_straight_mm = raw_segments[segment_index + 1].length_mm
        turns.append(turn)

    return RouteGeometry(
        name=str(route.get("name", f"路线{route_index + 1}")),
        route_index=route_index,
        segments=raw_segments,
        turns=turns,
        total_length_mm=accumulated,
        warnings=sorted(set(warnings)),
    )


def sample_pose(geometry: RouteGeometry, distance_mm: float) -> np.ndarray:
    distance = float(np.clip(distance_mm, 0.0, geometry.total_length_mm))
    for segment in geometry.segments:
        if distance > segment.end_s_mm and segment is not geometry.segments[-1]:
            continue
        local_distance = distance - segment.start_s_mm
        if segment.kind == "line":
            fraction = (
                float(np.clip(local_distance / segment.length_mm, 0.0, 1.0))
                if segment.length_mm > 0.0
                else 0.0
            )
            x = segment.ax + fraction * (segment.bx - segment.ax)
            y = segment.ay + fraction * (segment.by - segment.ay)
            yaw = math.atan2(segment.by - segment.ay, segment.bx - segment.ax)
            return pose_matrix(x, y, yaw)
        assert segment.points is not None
        raw_index = local_distance / segment.point_step_mm
        lower = int(np.clip(math.floor(raw_index), 0, len(segment.points) - 2))
        fraction = float(np.clip(raw_index - lower, 0.0, 1.0))
        first = segment.points[lower]
        second = segment.points[lower + 1]
        point = first + fraction * (second - first)
        return pose_matrix(float(point[0]), float(point[1]), float(point[2]))
    raise RuntimeError("路线采样失败")


def sample_poses(geometry: RouteGeometry, distances_mm: np.ndarray) -> np.ndarray:
    return np.asarray([sample_pose(geometry, float(value)) for value in distances_mm])


def relative_poses(poses: np.ndarray) -> np.ndarray:
    origin_inverse = inverse_pose(poses[0])
    return np.asarray([origin_inverse @ pose for pose in poses])


def distance_to_white_edges(
    points_mm: np.ndarray, physical_map: PhysicalMap
) -> np.ndarray:
    points = np.asarray(points_mm, dtype=float).reshape(-1, 2)
    first = physical_map.edges_a_mm
    vector = physical_map.edges_b_mm - physical_map.edges_a_mm
    length_squared = np.sum(vector * vector, axis=1)
    relative = points[:, None, :] - first[None, :, :]
    fraction = np.sum(relative * vector[None, :, :], axis=2) / length_squared[None, :]
    fraction = np.clip(fraction, 0.0, 1.0)
    closest = first[None, :, :] + fraction[:, :, None] * vector[None, :, :]
    distance = np.linalg.norm(points[:, None, :] - closest, axis=2)
    return np.min(distance, axis=1)


def sensor_profiles(
    poses: np.ndarray,
    physical_map: PhysicalMap,
    sensor: SensorConfig,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    body_points = sensor.channel_points_body_mm
    world_points: list[np.ndarray] = []
    for pose in poses:
        world_points.append(transform_points(pose, body_points))
    stacked = np.asarray(world_points)
    distances = distance_to_white_edges(stacked.reshape(-1, 2), physical_map).reshape(
        len(poses), sensor.count
    )
    argument = np.clip(
        (distances - 0.5 * sensor.line_width_mm) / sensor.edge_softness_mm,
        -40.0,
        40.0,
    )
    profiles = 1.0 / (1.0 + np.exp(argument))
    if rng is not None and sensor.noise_std > 0.0:
        profiles = profiles + rng.normal(0.0, sensor.noise_std, profiles.shape)
    return np.clip(profiles, 0.0, 1.0)


def profiles_from_start_pose(
    start_pose: np.ndarray,
    local_poses: np.ndarray,
    physical_map: PhysicalMap,
    sensor: SensorConfig,
) -> np.ndarray:
    global_poses = np.asarray([start_pose @ local for local in local_poses])
    return sensor_profiles(global_poses, physical_map, sensor)


YAW_PARAMETER_SCALE_MM = 200.0


def pose_to_parameters(pose: np.ndarray) -> np.ndarray:
    return np.array(
        [pose[0, 2], pose[1, 2], pose_yaw(pose) * YAW_PARAMETER_SCALE_MM],
        dtype=float,
    )


def parameters_to_pose(parameters: np.ndarray) -> np.ndarray:
    return pose_matrix(
        float(parameters[0]),
        float(parameters[1]),
        float(parameters[2] / YAW_PARAMETER_SCALE_MM),
    )


def measurement_jacobian(
    start_pose: np.ndarray,
    local_poses: np.ndarray,
    physical_map: PhysicalMap,
    sensor: SensorConfig,
    parameter_step_mm: float = 0.25,
) -> np.ndarray:
    parameters = pose_to_parameters(start_pose)
    columns: list[np.ndarray] = []
    for column in range(3):
        plus = parameters.copy()
        minus = parameters.copy()
        plus[column] += parameter_step_mm
        minus[column] -= parameter_step_mm
        plus_profile = profiles_from_start_pose(
            parameters_to_pose(plus), local_poses, physical_map, sensor
        )
        minus_profile = profiles_from_start_pose(
            parameters_to_pose(minus), local_poses, physical_map, sensor
        )
        columns.append(
            ((plus_profile - minus_profile) / (2.0 * parameter_step_mm)).reshape(-1)
        )
    return np.column_stack(columns)


def analyze_window(
    geometry: RouteGeometry,
    turn: Turn,
    window_name: str,
    start_s_mm: float,
    end_s_mm: float,
    sample_step_mm: float,
    physical_map: PhysicalMap,
    sensor: SensorConfig,
) -> WindowAnalysis:
    start = max(0.0, float(start_s_mm))
    end = min(float(end_s_mm), geometry.total_length_mm)
    distances = np.arange(start, end + 0.25 * sample_step_mm, sample_step_mm)
    if len(distances) < 4:
        distances = np.linspace(start, end, 4)
    poses = sample_poses(geometry, distances)
    local = relative_poses(poses)
    profiles = sensor_profiles(poses, physical_map, sensor)
    visible = np.max(profiles, axis=1) >= 0.36
    visible &= np.ptp(profiles, axis=1) >= 0.25
    jacobian = measurement_jacobian(
        poses[0], local, physical_map, sensor
    )
    _, singular_values, right_vectors = np.linalg.svd(jacobian, full_matrices=False)
    # 这里关心的是能否在真实噪声/安装误差下稳定纠正，而不只是数学上的非零。
    # 最弱方向至少达到最强方向的 2%，并超过绝对噪声灵敏度，才算可靠可观测。
    threshold = max(float(singular_values[0]) * 2.0e-2, 1.0e-2)
    rank = int(np.sum(singular_values > threshold))
    null_global = right_vectors[-1]
    # 将全局 x/y 的最弱方向旋转到扫描起点车体系，第三维仍为 200*dYaw。
    null_local_xy = poses[0, :2, :2].T @ null_global[:2]
    null_local = np.array([null_local_xy[0], null_local_xy[1], null_global[2]])
    if np.linalg.norm(null_local) > 0.0:
        null_local /= np.linalg.norm(null_local)
    return WindowAnalysis(
        route_index=geometry.route_index,
        route_name=geometry.name,
        turn_name=turn.name,
        graph_degree=turn.graph_degree,
        window=window_name,
        start_s_mm=start,
        end_s_mm=end,
        sample_count=len(distances),
        visible_count=int(np.sum(visible)),
        singular_values=singular_values,
        rank=rank,
        null_direction_local=null_local,
        outgoing_straight_mm=turn.outgoing_straight_mm,
        profiles=profiles,
        poses=poses,
        relative_poses=local,
    )


def pose_error(estimate: np.ndarray, truth: np.ndarray) -> dict[str, float]:
    local = inverse_pose(truth) @ estimate
    return {
        "position_mm": float(np.linalg.norm(local[:2, 2])),
        "longitudinal_mm": float(local[0, 2]),
        "lateral_mm": float(local[1, 2]),
        "yaw_deg": math.degrees(wrap_angle(pose_yaw(local))),
    }


def fit_start_pose(
    observations: np.ndarray,
    local_poses: np.ndarray,
    initial_pose: np.ndarray,
    physical_map: PhysicalMap,
    sensor: SensorConfig,
    maximum_iterations: int = 30,
) -> tuple[np.ndarray, list[float]]:
    parameters = pose_to_parameters(initial_pose)
    damping = 1.0e-2
    costs: list[float] = []
    for _ in range(maximum_iterations):
        pose = parameters_to_pose(parameters)
        prediction = profiles_from_start_pose(
            pose, local_poses, physical_map, sensor
        )
        residual = (prediction - observations).reshape(-1)
        cost = float(np.sqrt(np.mean(residual * residual)))
        costs.append(cost)
        jacobian = measurement_jacobian(
            pose, local_poses, physical_map, sensor, parameter_step_mm=0.25
        )
        hessian = jacobian.T @ jacobian
        gradient = jacobian.T @ residual
        try:
            delta = -np.linalg.solve(hessian + damping * np.eye(3), gradient)
        except np.linalg.LinAlgError:
            break
        delta[:2] = np.clip(delta[:2], -20.0, 20.0)
        delta[2] = float(np.clip(delta[2], -YAW_PARAMETER_SCALE_MM * math.radians(2.0), YAW_PARAMETER_SCALE_MM * math.radians(2.0)))
        trial = parameters + delta
        trial_prediction = profiles_from_start_pose(
            parameters_to_pose(trial), local_poses, physical_map, sensor
        )
        trial_residual = (trial_prediction - observations).reshape(-1)
        trial_cost = float(np.sqrt(np.mean(trial_residual * trial_residual)))
        if trial_cost < cost:
            parameters = trial
            damping = max(1.0e-6, damping * 0.35)
            if np.linalg.norm(delta) < 1.0e-3:
                costs.append(trial_cost)
                break
        else:
            damping = min(1.0e6, damping * 8.0)
            if damping >= 1.0e6:
                break
    return parameters_to_pose(parameters), costs


def coarse_to_fine_fit(
    observations: np.ndarray,
    local_poses: np.ndarray,
    initial_pose: np.ndarray,
    physical_map: PhysicalMap,
    sensor: SensorConfig,
) -> tuple[np.ndarray, list[float], dict[str, Any]]:
    """在已知当前路线/路口动作的前提下先做小范围栅格搜索，再局部优化。

    16 路路口波形是非凸的；直接从几十毫米误差做梯度下降可能落到错误支线。
    粗搜索只围绕当前里程计位姿，不做整张地图的全局重定位。
    """
    longitudinal_values = np.arange(-70.0, 70.1, 10.0)
    lateral_values = np.arange(-50.0, 50.1, 10.0)
    yaw_values = np.arange(-6.0, 6.1, 2.0)
    best_cost = math.inf
    best_pose = initial_pose
    best_offset = (0.0, 0.0, 0.0)
    minimum_by_longitudinal = {
        float(value): math.inf for value in longitudinal_values
    }
    for yaw_deg in yaw_values:
        for lateral_mm in lateral_values:
            for longitudinal_mm in longitudinal_values:
                candidate = initial_pose @ pose_matrix(
                    float(longitudinal_mm),
                    float(lateral_mm),
                    math.radians(float(yaw_deg)),
                )
                prediction = profiles_from_start_pose(
                    candidate, local_poses, physical_map, sensor
                )
                residual = prediction - observations
                cost = float(np.sqrt(np.mean(residual * residual)))
                key = float(longitudinal_mm)
                minimum_by_longitudinal[key] = min(
                    minimum_by_longitudinal[key], cost
                )
                if cost < best_cost:
                    best_cost = cost
                    best_pose = candidate
                    best_offset = (
                        float(longitudinal_mm),
                        float(lateral_mm),
                        float(yaw_deg),
                    )
    near_longitudinal = [
        value
        for value, cost in minimum_by_longitudinal.items()
        if cost <= best_cost + max(2.0 * sensor.noise_std, 0.01)
    ]
    ambiguity = (
        max(near_longitudinal) - min(near_longitudinal)
        if near_longitudinal
        else math.inf
    )
    fitted, local_costs = fit_start_pose(
        observations,
        local_poses,
        best_pose,
        physical_map,
        sensor,
    )
    return fitted, local_costs, {
        "best_coarse_profile_rms": best_cost,
        "best_correction_from_initial_long_lat_yaw_deg": list(best_offset),
        "longitudinal_ambiguity_mm": float(ambiguity),
        "grid_candidates": int(
            len(longitudinal_values) * len(lateral_values) * len(yaw_values)
        ),
    }


def summarize_analyses(items: list[WindowAnalysis]) -> dict[str, Any]:
    if not items:
        return {"windows": 0}
    full = [item for item in items if item.rank == 3]
    return {
        "windows": len(items),
        "full_rank_windows": len(full),
        "full_rank_rate": len(full) / len(items),
        "median_visible_rate": float(
            np.median([item.visible_count / item.sample_count for item in items])
        ),
        "median_smallest_singular_value": float(
            np.median([item.singular_values[-1] for item in items])
        ),
        "smallest_to_largest_ratio_median_p10": [
            float(np.median([item.singular_values[-1] / item.singular_values[0] for item in items])),
            float(np.percentile([item.singular_values[-1] / item.singular_values[0] for item in items], 10)),
        ],
    }


def serializable_analysis(item: WindowAnalysis) -> dict[str, Any]:
    return {
        "route": item.route_index,
        "route_name": item.route_name,
        "turn": item.turn_name,
        "graph_degree": item.graph_degree,
        "window": item.window,
        "start_s_mm": item.start_s_mm,
        "end_s_mm": item.end_s_mm,
        "sample_count": item.sample_count,
        "visible_count": item.visible_count,
        "rank": item.rank,
        "singular_values": item.singular_values.tolist(),
        "smallest_to_largest_ratio": float(
            item.singular_values[-1] / item.singular_values[0]
        ),
        "weakest_direction_local_long_lat_yaw200": item.null_direction_local.tolist(),
        "outgoing_straight_mm": item.outgoing_straight_mm,
    }


def route_plot_points(geometry: RouteGeometry, step_mm: float = 20.0) -> np.ndarray:
    distances = np.arange(0.0, geometry.total_length_mm + 0.5 * step_mm, step_mm)
    poses = sample_poses(geometry, distances)
    return poses[:, :2, 2]


def plot_full_map(
    physical_map: PhysicalMap,
    geometries: list[RouteGeometry],
    representative: WindowAnalysis,
    output: Path,
) -> None:
    figure, axis = plt.subplots(figsize=(9.0, 9.0))
    for first, second in zip(physical_map.edges_a_mm, physical_map.edges_b_mm):
        axis.plot(
            [first[0], second[0]],
            [first[1], second[1]],
            color="0.78",
            linewidth=4.0,
            solid_capstyle="round",
        )
        axis.plot(
            [first[0], second[0]],
            [first[1], second[1]],
            color="0.25",
            linewidth=0.8,
        )
    colors = ["tab:blue", "tab:orange", "tab:purple"]
    for geometry, color in zip(geometries, colors):
        points = route_plot_points(geometry)
        axis.plot(
            points[:, 0],
            points[:, 1],
            color=color,
            linewidth=1.0,
            alpha=0.55,
            label=f"Route {geometry.route_index + 1}",
        )
    window_points = representative.poses[:, :2, 2]
    axis.scatter(
        window_points[:, 0],
        window_points[:, 1],
        s=28,
        c=np.arange(len(window_points)),
        cmap="viridis",
        edgecolors="k",
        linewidths=0.3,
        label=f"{representative.turn_name} {representative.window}",
        zorder=5,
    )
    axis.set_aspect("equal", adjustable="box")
    axis.invert_yaxis()
    axis.set_title("Physical white-line graph and generated early-turn routes")
    axis.set_xlabel("map x / mm")
    axis.set_ylabel("map y / mm")
    axis.grid(alpha=0.12)
    axis.legend(fontsize=8, loc="best")
    figure.tight_layout()
    figure.savefig(output, dpi=180)
    plt.close(figure)


def plot_window_comparison(
    passage: WindowAnalysis,
    post_exit: WindowAnalysis,
    before_error: dict[str, float],
    passage_after: dict[str, float],
    post_after: dict[str, float],
    passage_costs: list[float],
    post_costs: list[float],
    output: Path,
) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(11.0, 7.6))
    for axis, item, title in (
        (axes[0, 0], passage, "Junction / early-turn passage: 16-channel profiles"),
        (axes[0, 1], post_exit, "Only after curve exit: 16-channel profiles"),
    ):
        image = axis.imshow(
            item.profiles,
            aspect="auto",
            origin="lower",
            vmin=0.0,
            vmax=1.0,
            cmap="magma",
        )
        axis.set_title(title)
        axis.set_xlabel("phototube channel")
        axis.set_ylabel("distance sample")
        axis.set_xticks(np.arange(0, 16, 2), np.arange(1, 17, 2))
        figure.colorbar(image, ax=axis, fraction=0.046, pad=0.04, label="normalized ADC")

    labels = ["largest", "middle", "smallest"]
    x = np.arange(3)
    width = 0.36
    axes[1, 0].bar(x - width / 2, passage.singular_values, width, label="junction/turn")
    axes[1, 0].bar(x + width / 2, post_exit.singular_values, width, label="post-exit")
    axes[1, 0].set_yscale("log")
    axes[1, 0].set_xticks(x, labels)
    axes[1, 0].set_ylabel("measurement Jacobian singular value")
    axes[1, 0].set_title("Three-pose observability")
    axes[1, 0].grid(alpha=0.2, axis="y")
    axes[1, 0].legend(fontsize=8)

    axes[1, 1].plot(passage_costs, "o-", label="junction/turn fit")
    axes[1, 1].plot(post_costs, "o-", label="post-exit fit")
    axes[1, 1].set_yscale("log")
    axes[1, 1].set_xlabel("damped least-squares iteration")
    axes[1, 1].set_ylabel("16-channel RMS")
    axes[1, 1].set_title(
        "Pose fit\n"
        f"before {before_error['position_mm']:.1f}mm/{abs(before_error['yaw_deg']):.1f}deg; "
        f"turn {passage_after['position_mm']:.1f}mm/{abs(passage_after['yaw_deg']):.2f}deg; "
        f"post {post_after['position_mm']:.1f}mm/{abs(post_after['yaw_deg']):.2f}deg"
    )
    axes[1, 1].grid(alpha=0.2)
    axes[1, 1].legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(output, dpi=180)
    plt.close(figure)


def finite_numbers(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: finite_numbers(item) for key, item in value.items()}
    if isinstance(value, list):
        return [finite_numbers(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--nav-header", type=Path, default=DEFAULT_NAV_HEADER)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--seed", type=int, default=20260811)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    physical_map = load_physical_map(args.map)
    sensor = SensorConfig()
    geometries = [
        build_route_geometry(physical_map, route, index)
        for index, route in enumerate(physical_map.routes)
    ]
    rng = np.random.default_rng(args.seed)

    passage_items: list[WindowAnalysis] = []
    post_items: list[WindowAnalysis] = []
    clear_post_items: list[WindowAnalysis] = []
    for geometry in geometries:
        for turn in geometry.turns:
            if turn.angle_deg <= 45.0:
                continue
            passage = analyze_window(
                geometry,
                turn,
                "junction_and_early_turn",
                turn.start_s_mm - 100.0,
                turn.end_s_mm + 150.0,
                25.0,
                physical_map,
                sensor,
            )
            post = analyze_window(
                geometry,
                turn,
                "post_exit_360mm",
                turn.end_s_mm,
                turn.end_s_mm + 360.0,
                40.0,
                physical_map,
                sensor,
            )
            passage_items.append(passage)
            post_items.append(post)
            # 阵列在轴前 100mm；下一个路口离出弯点超过 360+100 才是纯直线窗口。
            if turn.outgoing_straight_mm >= 460.0:
                clear_post_items.append(post)

    representative_pair: tuple[WindowAnalysis, WindowAnalysis] | None = None
    for passage, matching_post in zip(passage_items, post_items):
        if (
            matching_post is not None
            and passage.rank == 3
            and matching_post.rank < 3
            and passage.graph_degree >= 3
            and passage.visible_count >= 4
            and matching_post.visible_count >= 4
            and passage.outgoing_straight_mm >= 460.0
        ):
            representative_pair = (passage, matching_post)
            break
    if representative_pair is None:
        for passage, post in zip(passage_items, post_items):
            if passage.rank == 3 and post.rank < 3:
                representative_pair = (passage, post)
                break
    if representative_pair is None:
        raise RuntimeError("没有找到可展示的 rank3/rank2 对照弯道")
    passage_case, post_case = representative_pair

    truth_passage = passage_case.poses[0]
    initial_local_error = pose_matrix(-30.0, -15.0, math.radians(3.0))
    initial_passage = truth_passage @ initial_local_error
    observed_passage = np.clip(
        passage_case.profiles
        + rng.normal(0.0, sensor.noise_std, passage_case.profiles.shape),
        0.0,
        1.0,
    )
    fitted_passage, passage_costs, passage_search = coarse_to_fine_fit(
        observed_passage,
        passage_case.relative_poses,
        initial_passage,
        physical_map,
        sensor,
    )

    truth_post = post_case.poses[0]
    initial_post = truth_post @ initial_local_error
    observed_post = np.clip(
        post_case.profiles
        + rng.normal(0.0, sensor.noise_std, post_case.profiles.shape),
        0.0,
        1.0,
    )
    fitted_post, post_costs, post_search = coarse_to_fine_fit(
        observed_post,
        post_case.relative_poses,
        initial_post,
        physical_map,
        sensor,
    )

    header_generated = None
    if args.nav_header.exists():
        header_text = args.nav_header.read_text(encoding="utf-8")
        for line in header_text.splitlines()[:8]:
            if line.startswith("// 生成时间:"):
                header_generated = line.split(":", 1)[1].strip()
                break
    map_modified = datetime.fromtimestamp(args.map.stat().st_mtime).isoformat(timespec="seconds")

    route_summaries: list[dict[str, Any]] = []
    for geometry in geometries:
        major = [turn for turn in geometry.turns if turn.angle_deg > 45.0]
        route_summaries.append(
            {
                "route": geometry.route_index,
                "name": geometry.name,
                "length_mm": geometry.total_length_mm,
                "major_turns": len(major),
                "turns_at_degree3_or_more": sum(turn.graph_degree >= 3 for turn in major),
                "turn_angle_deg_min_median_max": [
                    float(np.min([turn.angle_deg for turn in major])),
                    float(np.median([turn.angle_deg for turn in major])),
                    float(np.max([turn.angle_deg for turn in major])),
                ],
                "turn_curve_length_mm_min_median_max": [
                    float(np.min([turn.end_s_mm - turn.start_s_mm for turn in major])),
                    float(np.median([turn.end_s_mm - turn.start_s_mm for turn in major])),
                    float(np.max([turn.end_s_mm - turn.start_s_mm for turn in major])),
                ],
                "turn_start_before_vertex_mm_min_median_max": [
                    float(np.min([turn.tangent_before_mm for turn in major])),
                    float(np.median([turn.tangent_before_mm for turn in major])),
                    float(np.max([turn.tangent_before_mm for turn in major])),
                ],
                "outgoing_straight_mm_min_median_max": [
                    float(np.min([turn.outgoing_straight_mm for turn in major])),
                    float(np.median([turn.outgoing_straight_mm for turn in major])),
                    float(np.max([turn.outgoing_straight_mm for turn in major])),
                ],
                "warnings": geometry.warnings,
            }
        )

    before_error = pose_error(initial_passage, truth_passage)
    passage_after = pose_error(fitted_passage, truth_passage)
    post_after = pose_error(fitted_post, truth_post)
    report: dict[str, Any] = {
        "physical_map": str(args.map.resolve()),
        "physical_map_modified": map_modified,
        "nav_header": str(args.nav_header.resolve()),
        "nav_header_generated": header_generated,
        "nav_header_is_older_than_map": (
            args.nav_header.exists() and args.nav_header.stat().st_mtime < args.map.stat().st_mtime
        ),
        "sensor": asdict(sensor),
        "map_topology": {
            "named_points": len(physical_map.points),
            "white_line_edges": len(physical_map.edge_names),
            "degree3_or_more_nodes": sum(value >= 3 for value in physical_map.degrees.values()),
            "degree4_or_more_nodes": sum(value >= 4 for value in physical_map.degrees.values()),
        },
        "routes": route_summaries,
        "observability": {
            "junction_and_early_turn": summarize_analyses(passage_items),
            "post_exit_360mm_all": summarize_analyses(post_items),
            "post_exit_360mm_clear_straight_only": summarize_analyses(clear_post_items),
            "meaning": (
                "rank=3 means local x/y/yaw are jointly observable; rank<3 means at least one "
                "pose direction cannot be recovered from that scan window"
            ),
        },
        "representative_case": {
            "passage": serializable_analysis(passage_case),
            "post_exit": serializable_analysis(post_case),
            "initial_error": before_error,
            "passage_fit_after_error": passage_after,
            "post_exit_fit_after_error": post_after,
            "passage_search": passage_search,
            "post_exit_search": post_search,
            "passage_fit_final_profile_rms": passage_costs[-1] if passage_costs else math.inf,
            "post_exit_fit_final_profile_rms": post_costs[-1] if post_costs else math.inf,
        },
        "conclusions": [
            "The smooth generated route is not the physical white line at a corner or branch.",
            "A fixed offset from nav_zone is invalid because turns begin before the vertex and branch topology varies.",
            "Collapsing a T/cross-junction profile to one centroid discards the landmark information.",
            "A clear outgoing straight can stabilize lateral/yaw tracking but cannot observe translation along the line.",
            "Full pose correction needs a junction/endpoint/marker profile or another longitudinal anchor.",
        ],
    }
    report = finite_numbers(report)
    (args.output / "results.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    plot_full_map(
        physical_map,
        geometries,
        passage_case,
        args.output / "full_map_and_routes.png",
    )
    plot_window_comparison(
        passage_case,
        post_case,
        before_error,
        passage_after,
        post_after,
        passage_costs,
        post_costs,
        args.output / "window_observability.png",
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"\nArtifacts: {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
