# -*- coding: utf-8 -*-
"""Fast route solver for the local map editor.

The primary fast path solves a directed-edge MILP, enumerates a bounded set of
Euler route orders, and independently checks each concrete route against the
map topology, smoothing geometry, curvature limit and acceleration-limited
speed ledger.  The direction-aware beam/K-shortest/2-opt/LNS planner remains a
stdlib-compatible fallback; neither path claims global optimality.
"""

from __future__ import annotations

import math
import os
import time
import heapq
from dataclasses import dataclass
from functools import lru_cache
from typing import Any, Iterator, Mapping, Sequence


REQUEST_SCHEMA = "smartcar.fast-route.request.v2"
RESULT_SCHEMA = "smartcar.fast-route.result.v2"
COINCIDENT_TOLERANCE_MM = 1.0e-6
DIRECTION_EPS = 1.0e-12
NANOMETRES_PER_MM = 1_000_000
# The fast planner uses a raw topology polyline before smoothing. A
# corner is a change of graph direction, not an independent circular arc, so
# there is no artificial tangent-fit radius constraint between adjacent bends.
TOPOLOGY_GEOMETRY_MODE = "pure_pursuit_polyline"
TIME_GEOMETRY_MODE = "pure_pursuit_smooth"
TIME_MODEL_VERSION = "curvature_accel_speed_envelope.v2"
TIME_QUANTUM_US = 1
DEFAULT_MAX_SPEED_CMPS = 700.0
# The exported map is a spatial envelope and the firmware follows it with the
# same finite longitudinal limits.  Keeping these defaults finite is important:
# an "unlimited" acceleration cap would turn a short straight into an
# instantaneous speed step and make the solver's time ledger optimistic.
DEFAULT_MAX_ACCEL_CMPS2 = 3000.0
DEFAULT_MAX_DECEL_CMPS2 = 5000.0
DEFAULT_MAX_LATERAL_ACCEL_CMPS2 = 20000.0
DEFAULT_SAFETY_MARGIN_MM = 80.0
DEFAULT_START_SPEED_CMPS = 0.0
SMOOTHING_PROFILE = "full_short_reverse_chain_then_arc_blend.v6"
COMPOSITE_PROFILE_VERSION = "circular_arc_curvature_blend.v12"
COMPOSITE_PREFIX_GAP_MM = 100.0
COMPOSITE_PARTITION_POLICY = "preferred_close_reverse_pair_backtracking.v3"
COMPOSITE_CORRIDOR_PROFILE = "short_chain_span_envelope.v3"
COMPOSITE_COLLISION_PROFILE = "physical_edges_and_smooth_route.v2"
COMPOSITE_CHAIN_CORRIDOR_RATIO = 0.30
COMPOSITE_CURVATURE_SAFETY_RATIO = 1.0
COMPOSITE_EXPANSION_STEPS = 64
COMPOSITE_HANDLE_RATIO_MIN = 0.25
COMPOSITE_HANDLE_RATIO_MAX = 1.25
COMPOSITE_HANDLE_RATIO_STEP = 0.05
# Fast planning compares a bounded set of independently valid routes.
FAST_MAX_FEASIBLE_CANDIDATES = 12
FAST_DIRECTION_WINDOW_MM = 6000.0
FAST_DIRECTION_CHANGE_EQUIVALENT_MM = 1000.0
FAST_BEAM_WIDTH = 192
FAST_K_SHORTEST_PATHS = 8
FAST_BEAM_TASK_BRANCHES = 24
FAST_LNS_MAX_ATTEMPTS = 300
FAST_MAX_SEARCH_ROUNDS = 96
FAST_MAX_GEOMETRY_CHECKS = 16
FAST_LNS_GEOMETRY_CHECKS = 16
# Historical fast planning used a directed-edge MILP to obtain a short
# feasible topology, then bounded the Euler-order expansion before the full
# current v12 geometry/speed verifier.  Keep these limits finite: this is
# still fast/non-proving mode, not the one-hour exact proof protocol.
FAST_MILP_MAX_TOPOLOGY_ATTEMPTS = 64
FAST_MILP_MAX_EULER_ORDERS_PER_TOPOLOGY = 256
FAST_MILP_ORDER_SLICE_S = 3.0
FAST_MILP_BUDGET_S = 120.0
FAST_MILP_FIRST_TOPOLOGY_SLICE_S = 60.0
FAST_MILP_NEXT_TOPOLOGY_SLICE_S = 15.0
# The bounded fast fallback uses a modest turn penalty to obtain a useful
# incumbent quickly.  The exact backend does not use this preference and
# optimizes physical topology length directly.
FAST_MILP_TURN_PENALTY_MM = 500.0
HIGHS_THREADS = 16
# The speed reference is updated by a 1 ms control tick.  Costs are kept in
# microseconds so the continuous bang-bang calculation is not distorted by
# rounding every macro transition to a whole control tick.
CONTROL_PERIOD_US = 1000
DEFAULT_TIMEOUT_S = 120.0
MAX_TIMEOUT_S = 120.0

SOURCE_NODE = -1
SINK_NODE = -2


class RouteRequestError(ValueError):
    """A request cannot be represented by the fast planning protocol."""

    def __init__(self, reason: str, message: str):
        super().__init__(message)
        self.reason = reason
        self.message = message


@dataclass(frozen=True)
class PlanGraph:
    coords: dict[str, tuple[float, float]]
    adjacency: dict[str, tuple[str, ...]]
    edge_lengths: dict[tuple[str, str], float]
    physical_edges: tuple[tuple[str, str], ...]
    required: frozenset[str]
    # Floating required points are services on a straight split edge rather
    # than geometry vertices.  Keeping their host edge/parameter prevents a
    # required marker from shortening the usable tangent span.
    required_edges: dict[str, tuple[tuple[str, str], float]]
    required_input_count: int
    aliases: dict[str, str]
    start: str
    start_output_name: str
    start_direction: str | None
    geometry_mode: str
    timeout_s: float
    avoided_edges: tuple[tuple[str, str], ...]
    # The planning topology remains a raw Pure Pursuit polyline. These values
    # freeze the vehicle-facing smoothing profile and preview-based reverse
    # bend safety policy into the solve request.
    swerve_gap_nm: int
    swerve_radius_mm: float
    swerve_ease_mm: float
    max_curvature_per_mm: float
    smoothing_profile: str
    composite_profile_version: str
    composite_corridor_profile: str
    composite_collision_profile: str
    swerve_min_turn_angle_rad: float
    max_speed_cmps: float
    max_accel_cmps2: float
    max_decel_cmps2: float
    max_lateral_accel_cmps2: float
    safety_margin_mm: float
    start_speed_cmps: float
    time_quantum_us: int
    time_model_version: str


@dataclass(frozen=True)
class CornerProfile:
    tangent_in_mm: float
    tangent_out_mm: float
    adjustment_mm: float
    is_turn: bool
    turn_sign: int = 0
    angle_rad: float = 0.0


@dataclass(frozen=True)
class SmoothTurnProfile:
    tangent_in_nm: int
    tangent_out_nm: int
    curve_length_nm: int
    actual_radius_mm: float
    actual_ease_mm: float
    scale: float


@dataclass(frozen=True)
class DirectedEdgeState:
    index: int
    tail: str
    head: str


@dataclass(frozen=True)
class ModelArc:
    index: int
    source: int
    target: int
    cost_nm: int
    kind: str
    chain: tuple[str, ...]
    swerve_cost: int = 0
    overlap_cost: int = 0
    smooth_cost_nm: int = 0
    time_cost_us: int = 0
    covered_required: tuple[str, ...] = ()


@dataclass
class SparseMilp:
    costs: list[float]
    lower: list[float]
    upper: list[float]
    integer: list[bool]
    rows: list[tuple[float | None, float | None, dict[int, float]]]
    objective_unit_nm: int

    def __init__(self) -> None:
        self.costs = []
        self.lower = []
        self.upper = []
        self.integer = []
        self.rows = []
        self.objective_unit_nm = 1

    def add_variable(
        self,
        *,
        cost: float = 0.0,
        lower: float = 0.0,
        upper: float,
        integer: bool,
    ) -> int:
        index = len(self.costs)
        self.costs.append(float(cost))
        self.lower.append(float(lower))
        self.upper.append(float(upper))
        self.integer.append(bool(integer))
        return index

    def add_row(
        self,
        coefficients: Mapping[int, float],
        *,
        lower: float | None = None,
        upper: float | None = None,
    ) -> None:
        cleaned = {
            index: float(value)
            for index, value in coefficients.items()
            if value != 0.0
        }
        self.rows.append((lower, upper, cleaned))


@dataclass(frozen=True)
class _FastBeamState:
    route: tuple[str, ...]
    covered_vertices: int
    covered_edges: int
    distance_nm: int
    turns: tuple[tuple[int, int], ...]
    direction_change_count: int
    max_direction_changes: int
    max_alternating_turns: int
    max_true_turns: int
    last_direction: tuple[float, float] | None
    current_straight_nm: int
    longest_straight_nm: int
    straight_concentration_nm2: int
    straight_run_count: int
    nominal_curve_nm: int


def _fast_direction_change_metrics(
    graph: PlanGraph,
    route: Sequence[str],
    *,
    window_mm: float = FAST_DIRECTION_WINDOW_MM,
) -> dict[str, Any]:
    """Measure turn density and sign changes in a fixed travelled-distance window.

    The metric uses the raw topology route, not the smoothed curve.  A true
    turn is one whose incoming and outgoing graph directions are not
    collinear.  ``max_direction_changes`` counts sign transitions (left to
    right or right to left), while ``max_true_turns`` prevents a route from
    hiding many same-direction corners behind a zero sign-transition score.
    Windows are evaluated at every true turn, which is sufficient because the
    count only changes when a window boundary crosses a turn position.  A
    closed route is evaluated cyclically, including the start corner and every
    window that crosses the finish/start seam.
    """
    if not math.isfinite(window_mm) or window_mm <= 0.0:
        raise RouteRequestError(
            "invalid_fast_metric", "direction-change window must be positive"
        )
    edge_lengths_nm = [
        _round_positive(_edge_length(graph, left, right) * NANOMETRES_PER_MM)
        for left, right in zip(route, route[1:])
    ]
    prefix_nm = [0]
    for edge_length_nm in edge_lengths_nm:
        prefix_nm.append(prefix_nm[-1] + edge_length_nm)
    turns: list[dict[str, Any]] = []
    closed_route = bool(
        len(route) >= 3 and route[0] == route[-1] and prefix_nm[-1] > 0
    )
    if closed_route:
        corner = _corner_profile(graph, route[-2], route[0], route[1])
        if corner is not None and corner.is_turn:
            turns.append({
                "route_index": 0,
                "vertex": route[0],
                "s_nm": 0,
                "sign": int(corner.turn_sign),
            })
    for index in range(1, len(route) - 1):
        left, at, right = route[index - 1], route[index], route[index + 1]
        corner = _corner_profile(graph, left, at, right)
        if corner is not None and corner.is_turn:
            turns.append({
                "route_index": index,
                "vertex": at,
                "s_nm": prefix_nm[index],
                "sign": int(corner.turn_sign),
            })

    window_nm = _round_positive(window_mm * NANOMETRES_PER_MM)
    straight_metrics = _fast_straight_metrics(graph, route)
    if not turns:
        return {
            "window_mm": window_mm,
            "window_nm": window_nm,
            "turn_count": 0,
            "max_true_turns": 0,
            "max_direction_changes": 0,
            "direction_change_count": 0,
            "max_alternating_turns": 0,
            "longest_same_direction_turns": 0,
            "max_true_turn_window": [],
            "max_direction_change_window": [],
            "max_alternating_window": [],
            "cyclic": closed_route,
            "distance_basis": "topology_route_proxy",
            **straight_metrics,
        }

    max_turns = 0
    max_changes = 0
    max_alternating = 0
    max_turn_window: list[str] = []
    max_change_window: list[str] = []
    max_alternating_window: list[str] = []
    scan_turns = list(turns)
    if closed_route:
        lap_nm = prefix_nm[-1]
        scan_turns.extend(
            {
                **item,
                "s_nm": int(item["s_nm"]) + lap_nm,
            }
            for item in turns
        )
    for start_index, start in enumerate(turns):
        end_index = start_index
        while (
            end_index + 1 < len(scan_turns)
            and end_index + 1 < start_index + len(turns)
            and scan_turns[end_index + 1]["s_nm"] - start["s_nm"] <= window_nm
        ):
            end_index += 1
        window_turns = scan_turns[start_index : end_index + 1]
        change_count = sum(
            int(previous["sign"]) != int(current["sign"])
            for previous, current in zip(window_turns, window_turns[1:])
        )
        names = [str(item["vertex"]) for item in window_turns]
        if len(window_turns) > max_turns:
            max_turns = len(window_turns)
            max_turn_window = names
        if change_count > max_changes:
            max_changes = change_count
            max_change_window = names

        alternating_start = 0
        for offset in range(1, len(window_turns) + 1):
            if (
                offset == len(window_turns)
                or int(window_turns[offset - 1]["sign"])
                == int(window_turns[offset]["sign"])
            ):
                run = window_turns[alternating_start:offset]
                if len(run) > max_alternating:
                    max_alternating = len(run)
                    max_alternating_window = [str(item["vertex"]) for item in run]
                alternating_start = offset

    signs = [int(item["sign"]) for item in turns]
    direction_change_count = sum(
        left != right for left, right in zip(signs, signs[1:])
    )
    if closed_route and len(signs) > 1:
        direction_change_count += int(signs[-1] != signs[0])

    longest_same_direction_run = 1
    if len(signs) > 1:
        if closed_route and direction_change_count == 0:
            longest_same_direction_run = len(signs)
        else:
            run = 1
            scan_signs = signs + (signs if closed_route else [])
            for left, right in zip(scan_signs, scan_signs[1:]):
                if left == right:
                    run += 1
                    longest_same_direction_run = max(
                        longest_same_direction_run, min(run, len(signs))
                    )
                else:
                    run = 1

    return {
        "window_mm": window_mm,
        "window_nm": window_nm,
        "turn_count": len(turns),
        "max_true_turns": max_turns,
        "max_direction_changes": max_changes,
        "direction_change_count": direction_change_count,
        "max_alternating_turns": max_alternating,
        "longest_same_direction_turns": longest_same_direction_run,
        "max_true_turn_window": max_turn_window,
        "max_direction_change_window": max_change_window,
        "max_alternating_window": max_alternating_window,
        "cyclic": closed_route,
        "distance_basis": "topology_route_proxy",
        **straight_metrics,
    }


def _fast_straight_metrics(
    graph: PlanGraph,
    route: Sequence[str],
) -> dict[str, Any]:
    """Measure uninterrupted same-heading runs in a topology route.

    Every graph edge is geometrically straight.  What matters for the car is
    a *continuous* straight: adjacent edges must be collinear and point in
    the same direction.  ``straight_concentration_nm2`` is the sum of squared
    run lengths, so at equal total distance it favours fewer, longer straights
    rather than a collection of short fragments.
    """
    runs_nm: list[int] = []
    current_run_nm = 0
    previous_direction: tuple[float, float] | None = None
    for left, right in zip(route, route[1:]):
        start = graph.coords[left]
        end = graph.coords[right]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        edge_nm = _round_positive(math.hypot(dx, dy) * NANOMETRES_PER_MM)
        if edge_nm <= 0:
            continue
        same_heading = False
        if previous_direction is not None:
            previous_dx, previous_dy = previous_direction
            cross = previous_dx * dy - previous_dy * dx
            dot = previous_dx * dx + previous_dy * dy
            tolerance = DIRECTION_EPS * max(
                1.0,
                abs(previous_dx * dy),
                abs(previous_dy * dx),
            )
            same_heading = abs(cross) <= tolerance and dot > 0.0
        if not same_heading and current_run_nm > 0:
            runs_nm.append(current_run_nm)
            current_run_nm = 0
        current_run_nm += edge_nm
        previous_direction = (dx, dy)
    if current_run_nm > 0:
        runs_nm.append(current_run_nm)
    concentration_nm2 = sum(run * run for run in runs_nm)
    return {
        "straight_run_count": len(runs_nm),
        "longest_straight_nm": max(runs_nm, default=0),
        "longest_straight_mm": max(runs_nm, default=0) / NANOMETRES_PER_MM,
        "straight_concentration_nm2": concentration_nm2,
    }


def _finite_number(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RouteRequestError("invalid_request", f"{context} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise RouteRequestError("invalid_request", f"{context} must be a finite number")
    return result


def _coordinate(value: Any, context: str) -> tuple[float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 2:
        raise RouteRequestError("invalid_request", f"{context} must contain [x, y]")
    x = _finite_number(value[0], f"{context}[0]")
    y = _finite_number(value[1], f"{context}[1]")
    if not x.is_integer() or not y.is_integer():
        raise RouteRequestError("invalid_request", f"{context} must use integer millimetres")
    return (x, y)


def _name(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RouteRequestError("invalid_request", f"{context} must be a non-empty string")
    return value.strip()


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _round_positive(value: float) -> int:
    """Match JavaScript Math.round for the non-negative geometry values."""

    if value < 0.0:
        raise RouteRequestError("internal_rounding_error", "expected a non-negative value")
    return math.floor(value + 0.5)




def _edge_key(a: str, b: str) -> tuple[str, str]:
    return (a, b) if a < b else (b, a)


def _segment_projection(
    point: tuple[float, float],
    a: tuple[float, float],
    b: tuple[float, float],
) -> tuple[float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 0.0:
        return (0.0, _distance(point, a))
    t = ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length_sq
    t = min(1.0, max(0.0, t))
    projection = (a[0] + t * dx, a[1] + t * dy)
    return (t, _distance(point, projection))


def _parse_avoid_edges(raw: Any) -> set[tuple[str, str]]:
    if raw is None:
        return set()
    if not isinstance(raw, list):
        raise RouteRequestError("invalid_request", "route_settings.avoid_edges must be a list")
    result: set[tuple[str, str]] = set()
    for index, item in enumerate(raw):
        if isinstance(item, str):
            parts = [part.strip() for part in item.replace("\u2014", "-").replace("~", "-").split("-")]
        elif isinstance(item, Sequence) and not isinstance(item, (str, bytes)):
            parts = [str(part).strip() for part in item]
        else:
            parts = []
        if len(parts) != 2 or not all(parts) or parts[0] == parts[1]:
            raise RouteRequestError(
                "invalid_request", f"route_settings.avoid_edges[{index}] must identify two endpoints"
            )
        result.add(_edge_key(parts[0], parts[1]))
    return result


def _merge_coordinate(
    coords: dict[str, tuple[float, float]],
    name: str,
    coord: tuple[float, float],
    context: str,
) -> None:
    existing = coords.get(name)
    if existing is not None and _distance(existing, coord) > COINCIDENT_TOLERANCE_MM:
        raise RouteRequestError("conflicting_point", f"{context} conflicts with point {name}")
    coords[name] = coord


def _host_edge(value: Any, context: str) -> tuple[str, str] | None:
    if value is None:
        return None
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 2:
        raise RouteRequestError("invalid_request", f"{context}.host_edge must contain two names")
    return _edge_key(_name(value[0], f"{context}.host_edge[0]"), _name(value[1], f"{context}.host_edge[1]"))


def _build_plan_graph(payload: Mapping[str, Any]) -> PlanGraph:
    if payload.get("schema") != REQUEST_SCHEMA:
        raise RouteRequestError("unsupported_schema", f"schema must be {REQUEST_SCHEMA}")

    nodes_raw = payload.get("nodes")
    edges_raw = payload.get("edges")
    start_raw = payload.get("start")
    required_raw = payload.get("required_points")
    settings = payload.get("route_settings") or {}
    options = payload.get("solve_options") or {}
    if not isinstance(nodes_raw, Mapping):
        raise RouteRequestError("invalid_request", "nodes must be an object")
    if not isinstance(edges_raw, list):
        raise RouteRequestError("invalid_request", "edges must be a list")
    if not isinstance(start_raw, Mapping):
        raise RouteRequestError("invalid_request", "start must be an object")
    if not isinstance(required_raw, list) or not required_raw:
        raise RouteRequestError("invalid_request", "required_points must be a non-empty list")
    if not isinstance(settings, Mapping) or not isinstance(options, Mapping):
        raise RouteRequestError("invalid_request", "route_settings and solve_options must be objects")
    if "seed_route" in options:
        raise RouteRequestError(
            "seed_disabled",
            "快速规划协议不接受预设路线；请让求解器直接从当前地图建立闭环",
        )
    if options.get("return_to_start", True) is not True:
        raise RouteRequestError("unsupported_route_type", "the fast route protocol requires return_to_start=true")

    coords: dict[str, tuple[float, float]] = {}
    for raw_name, raw_coord in nodes_raw.items():
        point_name = _name(raw_name, "nodes key")
        _merge_coordinate(coords, point_name, _coordinate(raw_coord, f"nodes.{point_name}"), "nodes")

    start_name = _name(start_raw.get("name"), "start.name")
    _merge_coordinate(coords, start_name, _coordinate(start_raw.get("coord"), "start.coord"), "start")

    required_names: list[str] = []
    feature_records: list[tuple[str, tuple[float, float], tuple[str, str] | None]] = []
    feature_records.append((start_name, coords[start_name], _host_edge(start_raw.get("host_edge"), "start")))
    for index, raw_point in enumerate(required_raw):
        if not isinstance(raw_point, Mapping):
            raise RouteRequestError("invalid_request", f"required_points[{index}] must be an object")
        point_name = _name(raw_point.get("name"), f"required_points[{index}].name")
        if point_name in required_names:
            raise RouteRequestError("duplicate_required_point", f"required point {point_name} is duplicated")
        coord = _coordinate(raw_point.get("coord"), f"required_points[{index}].coord")
        _merge_coordinate(coords, point_name, coord, f"required_points[{index}]")
        required_names.append(point_name)
        feature_records.append(
            (point_name, coord, _host_edge(raw_point.get("host_edge"), f"required_points[{index}]"))
        )

    original_edges: list[tuple[str, str]] = []
    virtual_edge_keys: set[tuple[str, str]] = set()
    seen_original: set[tuple[str, str]] = set()
    for index, raw_edge in enumerate(edges_raw):
        if isinstance(raw_edge, Mapping):
            raw_endpoints = (raw_edge.get("u"), raw_edge.get("v"))
            edge_type = raw_edge.get("type", "physical")
            if edge_type not in {"physical", "virtual"}:
                raise RouteRequestError("invalid_edge_type", f"edges[{index}].type must be physical or virtual")
        else:
            raw_endpoints = raw_edge
            edge_type = "physical"
        if not isinstance(raw_endpoints, Sequence) or isinstance(raw_endpoints, (str, bytes)) or len(raw_endpoints) != 2:
            raise RouteRequestError("invalid_request", f"edges[{index}] must contain two names")
        a = _name(raw_endpoints[0], f"edges[{index}][0]")
        b = _name(raw_endpoints[1], f"edges[{index}][1]")
        if a == b:
            raise RouteRequestError("invalid_edge", f"edges[{index}] is a self edge")
        if a not in coords or b not in coords:
            raise RouteRequestError("unknown_edge_endpoint", f"edges[{index}] references an unknown point")
        key = _edge_key(a, b)
        if key in seen_original:
            raise RouteRequestError("duplicate_edge", f"edge {a}-{b} is duplicated")
        if _distance(coords[a], coords[b]) <= COINCIDENT_TOLERANCE_MM:
            raise RouteRequestError("zero_length_edge", f"edge {a}-{b} has zero length")
        seen_original.add(key)
        if edge_type == "virtual":
            virtual_edge_keys.add(key)
        original_edges.append((a, b))

    avoid_wanted = _parse_avoid_edges(settings.get("avoid_edges"))
    avoid_missing = avoid_wanted - seen_original
    if avoid_missing:
        missing = ", ".join(f"{a}-{b}" for a, b in sorted(avoid_missing))
        raise RouteRequestError("unknown_avoid_edge", f"avoid_edges not present in the map: {missing}")
    active_edges = [(a, b) for a, b in original_edges if _edge_key(a, b) not in avoid_wanted]
    if not active_edges:
        raise RouteRequestError("empty_graph", "all map edges are excluded")

    referenced = {name for edge in active_edges for name in edge}
    aliases: dict[str, str] = {}
    split_points: list[list[tuple[float, str]]] = [[] for _ in active_edges]
    required_edge_hosts: dict[str, tuple[int, float]] = {}

    # Virtual connectors are topological shortcuts, not physical scan-map
    # lines. If one ends inside a physical edge, split that physical host at
    # the exact endpoint so the planner can enter the connector.
    physical_active_indices = [
        index
        for index, edge in enumerate(active_edges)
        if _edge_key(*edge) not in virtual_edge_keys
    ]
    physical_endpoint_names = {
        name
        for index in physical_active_indices
        for name in active_edges[index]
    }
    for a, b in active_edges:
        if _edge_key(a, b) not in virtual_edge_keys:
            continue
        for endpoint in (a, b):
            if endpoint in physical_endpoint_names:
                continue
            candidates: list[tuple[int, float]] = []
            for edge_index in physical_active_indices:
                host_a, host_b = active_edges[edge_index]
                t, distance_mm = _segment_projection(
                    coords[endpoint], coords[host_a], coords[host_b]
                )
                if (
                    distance_mm <= COINCIDENT_TOLERANCE_MM
                    and t > COINCIDENT_TOLERANCE_MM
                    and t < 1.0 - COINCIDENT_TOLERANCE_MM
                ):
                    candidates.append((edge_index, t))
            if len(candidates) > 1:
                raise RouteRequestError(
                    "ambiguous_virtual_edge_host",
                    f"virtual edge endpoint {endpoint} lies on multiple physical edges",
                )
            if len(candidates) == 1:
                edge_index, t = candidates[0]
                if not any(name == endpoint for _, name in split_points[edge_index]):
                    split_points[edge_index].append((t, endpoint))

    for feature_name, feature_coord, requested_host in feature_records:
        if feature_name in referenced:
            aliases[feature_name] = feature_name
            continue

        coincident = [name for name in sorted(referenced) if _distance(feature_coord, coords[name]) <= COINCIDENT_TOLERANCE_MM]
        if coincident:
            if len(coincident) != 1:
                raise RouteRequestError("ambiguous_feature_host", f"point {feature_name} matches multiple vertices")
            aliases[feature_name] = coincident[0]
            continue

        # A required marker may be exactly coincident with a split start
        # (most importantly P56).  The original vertex set is intentionally
        # captured before splitting, so check the live split lists as well;
        # otherwise the same coordinate lands on both child-edge boundaries
        # and is rejected as an ambiguous service.
        existing_splits = [
            name
            for split_list in split_points
            for _, name in split_list
            if _distance(coords[name], feature_coord) <= COINCIDENT_TOLERANCE_MM
        ]
        if existing_splits:
            if len(set(existing_splits)) != 1:
                raise RouteRequestError(
                    "ambiguous_feature_host",
                    f"point {feature_name} matches multiple split vertices",
                )
            aliases[feature_name] = existing_splits[0]
            continue

        candidates: list[tuple[float, int, float]] = []
        for edge_index, (a, b) in enumerate(active_edges):
            if requested_host is not None and _edge_key(a, b) != requested_host:
                continue
            if _edge_key(a, b) in virtual_edge_keys:
                continue
            t, distance_mm = _segment_projection(feature_coord, coords[a], coords[b])
            if distance_mm <= COINCIDENT_TOLERANCE_MM:
                candidates.append((distance_mm, edge_index, t))
        if not candidates:
            raise RouteRequestError(
                "feature_not_on_edge",
                f"point {feature_name} is not exactly on an active edge",
            )
        candidates.sort()
        best_distance = candidates[0][0]
        equally_best = [candidate for candidate in candidates if candidate[0] <= best_distance + 1.0e-9]
        if len(equally_best) != 1:
            raise RouteRequestError(
                "ambiguous_feature_host",
                f"point {feature_name} lies on multiple edges; persist an explicit host_edge",
            )
        _, edge_index, t = equally_best[0]
        aliases[feature_name] = feature_name
        # Only the start point is a topological split.  A required point in
        # the interior of a straight edge is a service on that edge; making
        # it a vertex would incorrectly constrain adjacent corner tangents.
        if feature_name != start_name:
            required_edge_hosts[feature_name] = (edge_index, t)
            continue
        duplicate = next(
            (
                existing_name
                for existing_t, existing_name in split_points[edge_index]
                if abs(existing_t - t) <= 1.0e-12
                and _distance(coords[existing_name], feature_coord) <= COINCIDENT_TOLERANCE_MM
            ),
            None,
        )
        if duplicate is not None:
            aliases[feature_name] = duplicate
        else:
            split_points[edge_index].append((t, feature_name))

    adjacency_sets: dict[str, set[str]] = {}
    edge_lengths: dict[tuple[str, str], float] = {}

    def add_edge(a: str, b: str) -> None:
        if a == b:
            return
        key = _edge_key(a, b)
        if key in edge_lengths:
            raise RouteRequestError("duplicate_split_edge", f"split graph contains duplicate edge {a}-{b}")
        length_mm = _distance(coords[a], coords[b])
        if length_mm <= COINCIDENT_TOLERANCE_MM:
            raise RouteRequestError("zero_length_split_edge", f"split graph edge {a}-{b} has zero length")
        edge_lengths[key] = length_mm
        adjacency_sets.setdefault(a, set()).add(b)
        adjacency_sets.setdefault(b, set()).add(a)

    split_edge_for_original: list[list[tuple[float, str]]] = []
    for edge_index, (a, b) in enumerate(active_edges):
        ordered = [name for _, name in sorted(split_points[edge_index], key=lambda item: (item[0], item[1]))]
        chain = [a, *ordered, b]
        split_edge_for_original.append([])
        for left, right in zip(chain, chain[1:]):
            add_edge(left, right)
            split_edge_for_original[edge_index].append((left, right))

    start = aliases.get(start_name, start_name)
    if start not in adjacency_sets:
        raise RouteRequestError("start_not_on_graph", "start is not on the active graph")
    required = frozenset(
        aliases.get(name, name) for name in required_names if name not in required_edge_hosts
    )
    required_edges: dict[str, tuple[tuple[str, str], float]] = {}
    for required_name, (edge_index, feature_t) in required_edge_hosts.items():
        candidates = []
        original_a, original_b = active_edges[edge_index]
        original_dx = coords[original_b][0] - coords[original_a][0]
        original_dy = coords[original_b][1] - coords[original_a][1]
        for left, right in split_edge_for_original[edge_index]:
            left_t = ((coords[left][0] - coords[original_a][0]) * original_dx +
                      (coords[left][1] - coords[original_a][1]) * original_dy) / (
                          original_dx * original_dx + original_dy * original_dy)
            right_t = ((coords[right][0] - coords[original_a][0]) * original_dx +
                       (coords[right][1] - coords[original_a][1]) * original_dy) / (
                           original_dx * original_dx + original_dy * original_dy)
            if left_t - 1.0e-12 <= feature_t <= right_t + 1.0e-12:
                candidates.append((left, right, feature_t))
            elif right_t - 1.0e-12 <= feature_t <= left_t + 1.0e-12:
                candidates.append((left, right, feature_t))
        if len(candidates) != 1:
            raise RouteRequestError(
                "required_host_resolution",
                f"required point {required_name} does not resolve to one split edge",
            )
        left, right, _ = candidates[0]
        required_edges[required_name] = (_edge_key(left, right), feature_t)
    missing_required = sorted(name for name in required if name not in adjacency_sets and name != start)
    if missing_required:
        raise RouteRequestError("required_not_on_graph", f"required points not on graph: {', '.join(missing_required)}")

    reachable = {start}
    stack = [start]
    while stack:
        current = stack.pop()
        for neighbour in adjacency_sets.get(current, ()):
            if neighbour not in reachable:
                reachable.add(neighbour)
                stack.append(neighbour)
    unreachable = sorted(name for name in required if name not in reachable)
    if unreachable:
        raise RouteRequestError("required_unreachable", f"required points unreachable: {', '.join(unreachable)}")
    unreachable_edge_services = sorted(
        name for name, (edge, _) in required_edges.items()
        if edge[0] not in reachable or edge[1] not in reachable
    )
    if unreachable_edge_services:
        raise RouteRequestError(
            "required_unreachable",
            f"required edge services unreachable: {', '.join(unreachable_edge_services)}",
        )

    start_direction_raw = settings.get("start_dir")
    start_direction = None
    if start_direction_raw is not None:
        start_direction_name = _name(start_direction_raw, "route_settings.start_dir")
        start_direction = aliases.get(start_direction_name, start_direction_name)
        if start_direction not in adjacency_sets[start]:
            direction_coord = coords.get(start_direction_name)
            direction_candidates: list[str] = []
            if direction_coord is not None:
                for neighbour in adjacency_sets[start]:
                    t, distance_mm = _segment_projection(
                        direction_coord, coords[start], coords[neighbour]
                    )
                    if distance_mm <= COINCIDENT_TOLERANCE_MM and t > 0.0:
                        direction_candidates.append(neighbour)
            if len(direction_candidates) == 1:
                start_direction = direction_candidates[0]
        if start_direction not in adjacency_sets[start]:
            raise RouteRequestError("invalid_start_direction", "start_dir must be adjacent to the split start vertex")

    radius_mm = _finite_number(settings.get("radius", 0.0), "route_settings.radius")
    ease_mm = _finite_number(settings.get("ease_mm", 0.0), "route_settings.ease_mm")
    geometry_mode = settings.get("geometry_mode")
    if geometry_mode != TOPOLOGY_GEOMETRY_MODE:
        raise RouteRequestError(
            "unsupported_geometry_mode",
            f"fast route v1 requires geometry_mode={TOPOLOGY_GEOMETRY_MODE}",
        )
    if radius_mm != 0.0:
        raise RouteRequestError(
            "unsupported_curve_profile",
            "pure_pursuit_polyline requires radius=0; smoothing curvature is validated separately",
        )
    # Pure Pursuit follows the exported polyline directly.  A non-zero easing
    # value would silently introduce a second geometric model, so reject it.
    if ease_mm != 0.0:
        raise RouteRequestError(
            "unsupported_curve_profile",
            "pure_pursuit_polyline requires ease_mm=0",
        )

    # Vehicle geometry is constrained by curvature, not by asking every graph
    # corner to own an independent circle.  Close true turns are one composite
    # chain; isolated turns retain the same curvature-ramp profile but may only
    # shorten the ramp, never shrink below the curvature-derived minimum radius.
    smoothing_profile = settings.get("smoothing_profile")
    if smoothing_profile != SMOOTHING_PROFILE:
        raise RouteRequestError(
            "unsupported_smoothing_profile",
            f"route_settings.smoothing_profile must be {SMOOTHING_PROFILE}",
        )
    composite_profile_version = settings.get("composite_profile_version")
    if composite_profile_version != COMPOSITE_PROFILE_VERSION:
        raise RouteRequestError(
            "unsupported_composite_profile",
            "route_settings.composite_profile_version must be "
            f"{COMPOSITE_PROFILE_VERSION}",
        )
    composite_partition_policy = settings.get("composite_partition_policy")
    if composite_partition_policy != COMPOSITE_PARTITION_POLICY:
        raise RouteRequestError(
            "unsupported_composite_partition_policy",
            "route_settings.composite_partition_policy must be "
            f"{COMPOSITE_PARTITION_POLICY}",
        )
    composite_corridor_profile = settings.get("composite_corridor_profile")
    if composite_corridor_profile != COMPOSITE_CORRIDOR_PROFILE:
        raise RouteRequestError(
            "unsupported_composite_corridor",
            "route_settings.composite_corridor_profile must be "
            f"{COMPOSITE_CORRIDOR_PROFILE}",
        )
    composite_collision_profile = settings.get("composite_collision_profile")
    if composite_collision_profile != COMPOSITE_COLLISION_PROFILE:
        raise RouteRequestError(
            "unsupported_composite_collision",
            "route_settings.composite_collision_profile must be "
            f"{COMPOSITE_COLLISION_PROFILE}",
        )
    max_curvature_per_mm = _finite_number(
        settings.get("max_curvature_per_mm"),
        "route_settings.max_curvature_per_mm",
    )
    smoothing_ease_mm = _finite_number(
        settings.get("smoothing_ease_mm"),
        "route_settings.smoothing_ease_mm",
    )
    composite_short_gap_mm = _finite_number(
        settings.get("composite_short_gap_mm"),
        "route_settings.composite_short_gap_mm",
    )
    if max_curvature_per_mm <= 0.0:
        raise RouteRequestError(
            "invalid_curvature_constraint",
            "route_settings.max_curvature_per_mm must be positive",
        )
    if smoothing_ease_mm < 0.0 or composite_short_gap_mm < 0.0:
        raise RouteRequestError(
            "invalid_smoothing_profile",
            "smoothing_ease_mm and composite_short_gap_mm must be non-negative",
        )
    swerve_radius_mm = 1.0 / max_curvature_per_mm
    swerve_ease_mm = smoothing_ease_mm
    swerve_gap_mm = composite_short_gap_mm
    speed_plan = settings.get("speed_plan")
    if not isinstance(speed_plan, Mapping):
        raise RouteRequestError(
            "invalid_speed_protocol",
            "route_settings.speed_plan must contain the curvature speed envelope",
        )
    max_speed_cmps = _finite_number(
        speed_plan.get("max_speed_cmps", DEFAULT_MAX_SPEED_CMPS),
        "route_settings.speed_plan.max_speed_cmps",
    )
    max_accel_cmps2 = _finite_number(
        speed_plan.get("max_accel_cmps2", DEFAULT_MAX_ACCEL_CMPS2),
        "route_settings.speed_plan.max_accel_cmps2",
    )
    max_decel_cmps2 = _finite_number(
        speed_plan.get("max_decel_cmps2", DEFAULT_MAX_DECEL_CMPS2),
        "route_settings.speed_plan.max_decel_cmps2",
    )
    max_lateral_accel_cmps2 = _finite_number(
        speed_plan.get("max_lateral_accel_cmps2", DEFAULT_MAX_LATERAL_ACCEL_CMPS2),
        "route_settings.speed_plan.max_lateral_accel_cmps2",
    )
    safety_margin_mm = _finite_number(
        speed_plan.get("safety_margin_mm", DEFAULT_SAFETY_MARGIN_MM),
        "route_settings.speed_plan.safety_margin_mm",
    )
    start_speed_cmps = _finite_number(
        speed_plan.get("start_speed_cmps", DEFAULT_START_SPEED_CMPS),
        "route_settings.speed_plan.start_speed_cmps",
    )
    time_quantum_us_raw = speed_plan.get("time_quantum_us", TIME_QUANTUM_US)
    if (
        isinstance(time_quantum_us_raw, bool)
        or not isinstance(time_quantum_us_raw, int)
        or time_quantum_us_raw != TIME_QUANTUM_US
    ):
        raise RouteRequestError(
            "invalid_speed_protocol",
            f"route_settings.speed_plan.time_quantum_us must be {TIME_QUANTUM_US}",
        )
    time_model_version = speed_plan.get("time_model")
    if time_model_version != TIME_MODEL_VERSION:
        raise RouteRequestError(
            "invalid_speed_protocol",
            f"route_settings.speed_plan.time_model must be {TIME_MODEL_VERSION}",
        )
    if (
        max_speed_cmps <= 0.0
        or max_accel_cmps2 <= 0.0
        or max_decel_cmps2 <= 0.0
        or max_lateral_accel_cmps2 <= 0.0
        or safety_margin_mm < 0.0
        or start_speed_cmps != DEFAULT_START_SPEED_CMPS
    ):
        raise RouteRequestError(
            "invalid_speed_protocol",
            "speed profile requires start=0 and positive speed/acceleration limits",
        )
    swerve_gap_nm = (
        _round_positive(swerve_gap_mm * NANOMETRES_PER_MM)
        if swerve_gap_mm > 0.0
        else 0
    )
    if swerve_gap_mm > 0.0 and swerve_gap_nm <= 0:
        raise RouteRequestError(
            "invalid_swerve_gap",
            "reverse_bend_preview_mm is below the one-nanometre protocol resolution",
        )

    timeout_s = _finite_number(options.get("timeout_s", DEFAULT_TIMEOUT_S), "solve_options.timeout_s")
    if timeout_s <= 0.0 or timeout_s > MAX_TIMEOUT_S:
        raise RouteRequestError(
            "invalid_timeout", f"solve_options.timeout_s must be in (0, {MAX_TIMEOUT_S:g}]"
        )
    adjacency = {name: tuple(sorted(neighbours)) for name, neighbours in adjacency_sets.items()}
    return PlanGraph(
        coords=coords,
        adjacency=adjacency,
        edge_lengths=edge_lengths,
        physical_edges=tuple(edge for edge in original_edges if _edge_key(*edge) not in virtual_edge_keys),
        required=required,
        required_edges=required_edges,
        required_input_count=len(required_names),
        aliases=aliases,
        start=start,
        start_output_name=start_name,
        start_direction=start_direction,
        geometry_mode=geometry_mode,
        timeout_s=timeout_s,
        avoided_edges=tuple(sorted(avoid_wanted)),
        swerve_gap_nm=swerve_gap_nm,
        swerve_radius_mm=swerve_radius_mm,
        swerve_ease_mm=swerve_ease_mm,
        max_curvature_per_mm=max_curvature_per_mm,
        smoothing_profile=smoothing_profile,
        composite_profile_version=composite_profile_version,
        composite_corridor_profile=composite_corridor_profile,
        composite_collision_profile=composite_collision_profile,
        swerve_min_turn_angle_rad=0.0,
        max_speed_cmps=max_speed_cmps,
        max_accel_cmps2=max_accel_cmps2,
        max_decel_cmps2=max_decel_cmps2,
        max_lateral_accel_cmps2=max_lateral_accel_cmps2,
        safety_margin_mm=safety_margin_mm,
        start_speed_cmps=start_speed_cmps,
        time_quantum_us=time_quantum_us_raw,
        time_model_version=time_model_version,
    )


def _corner_profile(graph: PlanGraph, previous: str, at: str, next_name: str) -> CornerProfile | None:
    a = graph.coords[previous]
    b = graph.coords[at]
    c = graph.coords[next_name]
    in_x = b[0] - a[0]
    in_y = b[1] - a[1]
    out_x = c[0] - b[0]
    out_y = c[1] - b[1]
    in_length = math.hypot(in_x, in_y)
    out_length = math.hypot(out_x, out_y)
    if in_length <= COINCIDENT_TOLERANCE_MM or out_length <= COINCIDENT_TOLERANCE_MM:
        return None
    scale = in_length * out_length
    dot = (in_x * out_x + in_y * out_y) / scale
    cross = (in_x * out_y - in_y * out_x) / scale
    if abs(cross) <= DIRECTION_EPS and dot > 0.0:
        return CornerProfile(
            tangent_in_mm=0.0,
            tangent_out_mm=0.0,
            adjustment_mm=0.0,
            is_turn=False,
        )
    if abs(cross) <= DIRECTION_EPS and dot < 0.0:
        return None
    if graph.geometry_mode == TOPOLOGY_GEOMETRY_MODE:
        # The planning topology is a raw polyline. The controller's preview bridges
        # the heading discontinuity; no per-corner tangent length is charged.
        clamped_dot = max(-1.0, min(1.0, dot))
        return CornerProfile(
            tangent_in_mm=0.0,
            tangent_out_mm=0.0,
            adjustment_mm=0.0,
            is_turn=True,
            turn_sign=1 if cross > 0.0 else -1,
            angle_rad=math.atan2(abs(cross), clamped_dot),
        )
    raise RouteRequestError(
        "unsupported_geometry_mode",
        f"unsupported graph geometry mode: {graph.geometry_mode}",
    )


def _fixed_corner_tangent_nm(
    angle_rad: float,
    radius_mm: float,
    ease_mm: float,
) -> tuple[int, int]:
    """Return deterministic fixed-profile entry/exit cuts in nanometres.

    This mirrors the editor's ``buildCornerCurve`` profile: a circular arc
    when easing is zero, otherwise linear curvature ramps around an optional
    constant-curvature section.  The editor defines the eased profile by a
    deterministic midpoint integration at approximately 2 mm, so duplicating
    that construction here keeps the browser and backend ledgers identical.
    Rounding the resulting cuts once to integer nanometres makes every later
    fit reproducible.
    """

    if not (0.0 < angle_rad < math.pi):
        raise RouteRequestError("invalid_swerve_geometry", "turn angle is outside (0, pi)")
    if radius_mm <= 0.0 or ease_mm < 0.0:
        raise RouteRequestError("invalid_swerve_profile", "fixed swerve profile is invalid")
    if ease_mm <= 1.0e-9:
        tangent_mm = radius_mm * math.tan(angle_rad * 0.5)
        tangent_nm = _round_positive(tangent_mm * NANOMETRES_PER_MM)
        return tangent_nm, tangent_nm

    curvature = 1.0 / radius_mm
    ramp_length_mm = ease_mm
    arc_angle = angle_rad - curvature * ramp_length_mm
    if arc_angle < 0.0:
        ramp_length_mm = angle_rad / curvature
        arc_angle = 0.0
    arc_length_mm = arc_angle / curvature
    total_length_mm = 2.0 * ramp_length_mm + arc_length_mm
    sample_count = max(4, math.ceil(total_length_mm / 2.0))
    ds_mm = total_length_mm / sample_count
    x_mm = 0.0
    y_mm = 0.0
    heading = 0.0
    for sample_index in range(sample_count):
        midpoint_mm = (sample_index + 0.5) * ds_mm
        if ramp_length_mm > 1.0e-9 and midpoint_mm < ramp_length_mm:
            local_curvature = curvature * midpoint_mm / ramp_length_mm
        elif midpoint_mm < ramp_length_mm + arc_length_mm or ramp_length_mm <= 1.0e-9:
            local_curvature = curvature
        else:
            local_curvature = curvature * (total_length_mm - midpoint_mm) / ramp_length_mm
        midpoint_heading = heading + local_curvature * ds_mm * 0.5
        x_mm += ds_mm * math.cos(midpoint_heading)
        y_mm += ds_mm * math.sin(midpoint_heading)
        heading += local_curvature * ds_mm

    sin_heading = math.sin(heading)
    if abs(sin_heading) <= DIRECTION_EPS:
        raise RouteRequestError(
            "invalid_swerve_geometry",
            "fixed corner profile has an undefined tangent intersection",
        )
    tangent_out_mm = y_mm / sin_heading
    tangent_in_mm = x_mm - y_mm * math.cos(heading) / sin_heading
    if (
        not math.isfinite(tangent_in_mm)
        or not math.isfinite(tangent_out_mm)
        or tangent_in_mm < 0.0
        or tangent_out_mm < 0.0
    ):
        raise RouteRequestError(
            "invalid_swerve_geometry",
            "fixed corner profile produced invalid tangent cuts",
        )
    return (
        _round_positive(tangent_in_mm * NANOMETRES_PER_MM),
        _round_positive(tangent_out_mm * NANOMETRES_PER_MM),
    )


def _corner_curve_metrics_mm(
    angle_rad: float,
    radius_mm: float,
    ease_mm: float,
) -> tuple[float, float, float]:
    """Mirror buildCornerCurve and return (tIn, tOut, curve length)."""

    if not (0.0 < angle_rad < math.pi) or radius_mm <= 0.0 or ease_mm < 0.0:
        raise RouteRequestError("invalid_smooth_geometry", "invalid smooth corner profile")
    if ease_mm <= 1.0e-9:
        tangent_mm = radius_mm * math.tan(angle_rad * 0.5)
        return tangent_mm, tangent_mm, radius_mm * angle_rad

    curvature = 1.0 / radius_mm
    ramp_length_mm = ease_mm
    arc_angle = angle_rad - curvature * ramp_length_mm
    if arc_angle < 0.0:
        ramp_length_mm = angle_rad / curvature
        arc_angle = 0.0
    arc_length_mm = arc_angle / curvature
    total_length_mm = 2.0 * ramp_length_mm + arc_length_mm
    sample_count = max(4, math.ceil(total_length_mm / 2.0))
    ds_mm = total_length_mm / sample_count
    x_mm = 0.0
    y_mm = 0.0
    heading = 0.0
    for sample_index in range(sample_count):
        midpoint_mm = (sample_index + 0.5) * ds_mm
        if ramp_length_mm > 1.0e-9 and midpoint_mm < ramp_length_mm:
            local_curvature = curvature * midpoint_mm / ramp_length_mm
        elif midpoint_mm < ramp_length_mm + arc_length_mm or ramp_length_mm <= 1.0e-9:
            local_curvature = curvature
        else:
            local_curvature = curvature * (total_length_mm - midpoint_mm) / ramp_length_mm
        midpoint_heading = heading + local_curvature * ds_mm * 0.5
        x_mm += ds_mm * math.cos(midpoint_heading)
        y_mm += ds_mm * math.sin(midpoint_heading)
        heading += local_curvature * ds_mm

    sin_heading = math.sin(heading)
    if abs(sin_heading) <= DIRECTION_EPS:
        raise RouteRequestError(
            "invalid_smooth_geometry",
            "smooth corner profile has an undefined tangent intersection",
        )
    tangent_out_mm = y_mm / sin_heading
    tangent_in_mm = x_mm - y_mm * math.cos(heading) / sin_heading
    if min(tangent_in_mm, tangent_out_mm, total_length_mm) < 0.0:
        raise RouteRequestError("invalid_smooth_geometry", "smooth corner metrics are negative")
    return tangent_in_mm, tangent_out_mm, total_length_mm




def _smooth_turn_profile_with_cut_limits(
    graph: PlanGraph,
    angle_rad: float,
    incoming_cut_limit_nm: int,
    outgoing_cut_limit_nm: int,
) -> SmoothTurnProfile:
    """Fit one turn after neighbouring groups have reserved their cuts."""

    incoming_limit_mm = incoming_cut_limit_nm / NANOMETRES_PER_MM
    outgoing_limit_mm = outgoing_cut_limit_nm / NANOMETRES_PER_MM
    if min(incoming_limit_mm, outgoing_limit_mm) <= 0.0:
        raise RouteRequestError("invalid_smooth_geometry", "corner has no tangent span")
    nominal_in, nominal_out, _ = _corner_curve_metrics_mm(
        angle_rad,
        graph.swerve_radius_mm,
        graph.swerve_ease_mm,
    )
    if min(nominal_in, nominal_out) <= 0.0:
        raise RouteRequestError("invalid_smooth_geometry", "corner nominal cut is zero")
    # Preserve the curvature constraint exactly: shorten the easing first, but
    # never reduce radius.  A corner that still cannot fit is either absorbed
    # by the deterministic composite-chain pass or rejected as infeasible.
    actual_radius_mm = graph.swerve_radius_mm
    actual_ease_mm = graph.swerve_ease_mm
    circular_in, circular_out, _ = _corner_curve_metrics_mm(
        angle_rad, actual_radius_mm, 0.0
    )
    nominal_fits = bool(
        nominal_in <= incoming_limit_mm + 1.0e-7
        and nominal_out <= outgoing_limit_mm + 1.0e-7
    )
    circular_fits = bool(
        circular_in <= incoming_limit_mm + 1.0e-7
        and circular_out <= outgoing_limit_mm + 1.0e-7
    )
    if nominal_fits:
        pass
    elif circular_fits:
        lo = 0.0
        hi = graph.swerve_ease_mm
        actual_ease_mm = 0.0
        for _ in range(32):
            mid = 0.5 * (lo + hi)
            candidate_in, candidate_out, _ = _corner_curve_metrics_mm(
                angle_rad, actual_radius_mm, mid
            )
            if (
                candidate_in <= incoming_limit_mm
                and candidate_out <= outgoing_limit_mm
            ):
                lo = mid
                actual_ease_mm = mid
            else:
                hi = mid
    else:
        raise RouteRequestError(
            "curvature_limited_corner_does_not_fit",
            "minimum-radius corner does not fit its available tangent spans",
        )
    tangent_in_mm, tangent_out_mm, curve_length_mm = _corner_curve_metrics_mm(
        angle_rad, actual_radius_mm, actual_ease_mm
    )
    if (
        tangent_in_mm > incoming_limit_mm + 0.05
        or tangent_out_mm > outgoing_limit_mm + 0.05
    ):
        raise RouteRequestError("invalid_smooth_geometry", "fitted corner exceeds tangent limit")
    return SmoothTurnProfile(
        tangent_in_nm=_round_positive(tangent_in_mm * NANOMETRES_PER_MM),
        tangent_out_nm=_round_positive(tangent_out_mm * NANOMETRES_PER_MM),
        curve_length_nm=_round_positive(curve_length_mm * NANOMETRES_PER_MM),
        actual_radius_mm=actual_radius_mm,
        actual_ease_mm=actual_ease_mm,
        scale=actual_radius_mm / graph.swerve_radius_mm,
    )


def _edge_length(graph: PlanGraph, a: str, b: str) -> float:
    try:
        return graph.edge_lengths[_edge_key(a, b)]
    except KeyError as exc:
        raise RouteRequestError("internal_graph_error", f"missing graph edge {a}-{b}") from exc


def _travel_toward_speed(
    distance_mm: float,
    start_mmps: float,
    target_mmps: float,
    accel_mmps2: float,
    decel_mmps2: float,
) -> tuple[float, float]:
    """Traverse one constant-target segment and return (seconds, end speed)."""

    if distance_mm <= 0.0:
        return 0.0, start_mmps
    if start_mmps < target_mmps:
        reach_mm = (target_mmps * target_mmps - start_mmps * start_mmps) / (2.0 * accel_mmps2)
        if distance_mm < reach_mm:
            end_mmps = math.sqrt(start_mmps * start_mmps + 2.0 * accel_mmps2 * distance_mm)
            return (end_mmps - start_mmps) / accel_mmps2, end_mmps
        cruise_seconds = 0.0 if target_mmps <= 0.0 else (distance_mm - reach_mm) / target_mmps
        return ((target_mmps - start_mmps) / accel_mmps2 + cruise_seconds, target_mmps)
    if start_mmps > target_mmps:
        reach_mm = (start_mmps * start_mmps - target_mmps * target_mmps) / (2.0 * decel_mmps2)
        if distance_mm < reach_mm:
            end_sq = max(0.0, start_mmps * start_mmps - 2.0 * decel_mmps2 * distance_mm)
            end_mmps = math.sqrt(end_sq)
            return (start_mmps - end_mmps) / decel_mmps2, end_mmps
        cruise_seconds = 0.0 if target_mmps <= 0.0 else (distance_mm - reach_mm) / target_mmps
        return ((start_mmps - target_mmps) / decel_mmps2 + cruise_seconds, target_mmps)
    return distance_mm / target_mmps, target_mmps


def _high_interval_time(
    distance_mm: float,
    start_mmps: float,
    end_mmps: float,
    cap_mmps: float,
    accel_mmps2: float,
    decel_mmps2: float,
) -> tuple[float, float] | None:
    """Fastest bounded-speed interval with fixed boundary speeds."""

    if start_mmps < -1.0e-9 or end_mmps < -1.0e-9:
        return None
    if start_mmps > cap_mmps + 1.0e-9 or end_mmps > cap_mmps + 1.0e-9:
        return None
    if distance_mm <= 0.0:
        return (0.0, start_mmps) if abs(start_mmps - end_mmps) <= 1.0e-9 else None
    minimum_mm = 0.0
    if start_mmps < end_mmps:
        minimum_mm = (end_mmps * end_mmps - start_mmps * start_mmps) / (2.0 * accel_mmps2)
    elif start_mmps > end_mmps:
        minimum_mm = (start_mmps * start_mmps - end_mmps * end_mmps) / (2.0 * decel_mmps2)
    if distance_mm + 1.0e-9 < minimum_mm:
        return None

    peak_sq = (
        2.0 * accel_mmps2 * decel_mmps2 * distance_mm
        + decel_mmps2 * start_mmps * start_mmps
        + accel_mmps2 * end_mmps * end_mmps
    ) / (accel_mmps2 + decel_mmps2)
    peak_mmps = min(cap_mmps, math.sqrt(max(0.0, peak_sq)))
    peak_mmps = max(peak_mmps, start_mmps, end_mmps)
    accel_distance = max(0.0, (peak_mmps * peak_mmps - start_mmps * start_mmps) / (2.0 * accel_mmps2))
    decel_distance = max(0.0, (peak_mmps * peak_mmps - end_mmps * end_mmps) / (2.0 * decel_mmps2))
    cruise_distance = max(0.0, distance_mm - accel_distance - decel_distance)
    seconds = (
        max(0.0, peak_mmps - start_mmps) / accel_mmps2
        + max(0.0, peak_mmps - end_mmps) / decel_mmps2
        + cruise_distance / peak_mmps
    )
    return seconds, peak_mmps


def _time_us(seconds: float, quantum_us: int) -> int:
    raw_us = _round_positive(seconds * 1_000_000.0)
    return ((raw_us + quantum_us // 2) // quantum_us) * quantum_us


def _speed_limits_mm(graph: PlanGraph) -> tuple[float, float, float, float]:
    maximum_mmps = graph.max_speed_cmps * 10.0
    curve_mmps = maximum_mmps
    if graph.max_curvature_per_mm > 0.0:
        curvature_cap_mmps = math.sqrt(
            graph.max_lateral_accel_cmps2 * 10.0 /
            graph.max_curvature_per_mm
        )
        curve_mmps = min(curve_mmps, curvature_cap_mmps)
    return (
        maximum_mmps,
        curve_mmps,
        graph.max_accel_cmps2 * 10.0,
        graph.max_decel_cmps2 * 10.0,
    )


def _source_time_profile(
    graph: PlanGraph,
    line_length_nm: int,
    curve_length_nm: int,
) -> tuple[int, dict[str, Any]]:
    maximum, curve, accel, decel = _speed_limits_mm(graph)
    line_mm = line_length_nm / NANOMETRES_PER_MM
    curve_mm = curve_length_nm / NANOMETRES_PER_MM
    low_budget_mm = graph.safety_margin_mm
    low_line_mm = min(line_mm, low_budget_mm)
    high_mm = max(0.0, line_mm - low_line_mm)
    start = graph.start_speed_cmps * 10.0
    fixed = _high_interval_time(high_mm, start, curve, maximum, accel, decel)
    if fixed is None:
        # The unrestricted part is too short to reach the curvature cap at
        # its end. Use its reachable terminal speed, then require the safety
        # zone plus the curve to reach that physical boundary exactly.
        high_seconds, boundary = _travel_toward_speed(
            high_mm, start, maximum, accel, decel
        )
        peak = boundary
        if boundary > curve + 1.0e-6:
            raise RouteRequestError(
                "speed_model_boundary_violation",
                "source unrestricted interval ends above the curvature cap",
            )
    else:
        high_seconds, peak = fixed
        boundary = curve
    low_seconds, end_speed = _travel_toward_speed(
        low_line_mm + curve_mm,
        boundary,
        curve,
        accel,
        decel,
    )
    if abs(end_speed - curve) > 1.0e-6:
        raise RouteRequestError(
            "speed_model_unreachable_corner_speed",
            "the first curve cannot reach the curvature-speed boundary",
        )
    total_seconds = high_seconds + low_seconds
    return _time_us(total_seconds, graph.time_quantum_us), {
        "kind": "start_to_first_corner_exit",
        "high_distance_mm": high_mm,
        "low_distance_mm": low_line_mm + curve_mm,
        "peak_speed_cmps": peak / 10.0,
        "seconds": total_seconds,
    }


def _between_turn_time_profile(
    graph: PlanGraph,
    line_length_nm: int,
    next_curve_length_nm: int,
) -> tuple[int, dict[str, Any]]:
    maximum, curve, accel, decel = _speed_limits_mm(graph)
    line_mm = line_length_nm / NANOMETRES_PER_MM
    curve_mm = next_curve_length_nm / NANOMETRES_PER_MM
    low_budget_mm = graph.safety_margin_mm
    low_line_mm = min(line_mm, low_budget_mm)
    high_mm = max(0.0, line_mm - low_line_mm)
    high = _high_interval_time(high_mm, curve, curve, maximum, accel, decel)
    if high is None:
        raise RouteRequestError("invalid_speed_protocol", "internal corner interval is infeasible")
    high_seconds, peak = high
    low_seconds = (low_line_mm + curve_mm) / curve
    total_seconds = high_seconds + low_seconds
    return _time_us(total_seconds, graph.time_quantum_us), {
        "kind": "corner_to_corner_exit",
        "high_distance_mm": high_mm,
        "low_distance_mm": low_line_mm + curve_mm,
        "peak_speed_cmps": peak / 10.0,
        "seconds": total_seconds,
    }


def _finish_time_profile(graph: PlanGraph, line_length_nm: int) -> tuple[int, dict[str, Any]]:
    maximum, curve, accel, decel = _speed_limits_mm(graph)
    line_mm = line_length_nm / NANOMETRES_PER_MM
    seconds, end_speed = _travel_toward_speed(
        line_mm, curve, maximum, accel, decel
    )
    return _time_us(seconds, graph.time_quantum_us), {
        "kind": "last_corner_exit_to_finish",
        "high_distance_mm": line_mm,
        "low_distance_mm": 0.0,
        "peak_speed_cmps": end_speed / 10.0,
        "seconds": seconds,
    }























def _build_directed_edge_model(
    graph: PlanGraph,
) -> tuple[list[DirectedEdgeState], list[ModelArc], SparseMilp, dict[str, Any]]:
    """Build the exact walk MILP on directed physical-edge states.

    State ``(u, v)`` means that the route just traversed physical edge
    ``u-v``.  A transition to ``(v, w)`` is legal exactly when the original
    graph permits the edge and the existing corner predicate permits the
    non-reversing passage.  This is equivalent to the previous maneuver
    model for the polyline objective, while avoiding its turn-to-turn macro
    expansion.
    """

    raw_states: list[DirectedEdgeState] = []
    for tail in sorted(graph.adjacency):
        for head in graph.adjacency[tail]:
            raw_states.append(
                DirectedEdgeState(index=len(raw_states), tail=tail, head=head)
            )
    raw_state_by_edge = {
        (state.tail, state.head): state.index for state in raw_states
    }

    arc_specs: list[tuple[int, int, int, str, tuple[str, ...]]] = []

    def add_arc_spec(
        source: int,
        target: int,
        cost_nm: int,
        kind: str,
        chain: Sequence[str],
    ) -> None:
        arc_specs.append((source, target, cost_nm, kind, tuple(chain)))

    for first in graph.adjacency.get(graph.start, ()):
        if graph.start_direction is not None and first != graph.start_direction:
            continue
        target = raw_state_by_edge[(graph.start, first)]
        add_arc_spec(
            SOURCE_NODE,
            target,
            _round_positive(_edge_length(graph, graph.start, first) * NANOMETRES_PER_MM),
            "start",
            (graph.start, first),
        )

    for state in raw_states:
        if state.head == graph.start:
            add_arc_spec(
                state.index,
                SINK_NODE,
                0,
                "finish",
                (graph.start,),
            )
            continue
        for next_name in graph.adjacency.get(state.head, ()):
            if next_name == state.tail:
                continue
            if _corner_profile(graph, state.tail, state.head, next_name) is None:
                continue
            target = raw_state_by_edge[(state.head, next_name)]
            add_arc_spec(
                state.index,
                target,
                _round_positive(_edge_length(graph, state.head, next_name) * NANOMETRES_PER_MM),
                "continue",
                (state.head, next_name),
            )

    if not any(source == SOURCE_NODE for source, _, _, _, _ in arc_specs):
        raise RouteRequestError("no_legal_start", "no legal directed edge leaves the start")
    if not any(target == SINK_NODE for _, target, _, _, _ in arc_specs):
        raise RouteRequestError("no_legal_finish", "no legal directed edge returns to the start")

    # Keep only state nodes that lie on at least one source-to-sink walk.
    # This is exact reachability pruning; no cost or route-length bound is
    # involved.
    raw_outgoing: dict[int, list[int]] = {}
    raw_incoming: dict[int, list[int]] = {}
    for source, target, _, _, _ in arc_specs:
        raw_outgoing.setdefault(source, []).append(target)
        raw_incoming.setdefault(target, []).append(source)

    def reachable(seed: int, adjacency: Mapping[int, Sequence[int]]) -> set[int]:
        seen = {seed}
        stack = [seed]
        while stack:
            current = stack.pop()
            for neighbour in adjacency.get(current, ()):
                if neighbour not in seen:
                    seen.add(neighbour)
                    stack.append(neighbour)
        return seen

    forward = reachable(SOURCE_NODE, raw_outgoing)
    backward = reachable(SINK_NODE, raw_incoming)
    if SINK_NODE not in forward:
        raise RouteRequestError(
            "no_legal_closed_route", "no non-reversing route can return to the start"
        )
    kept_old_indices = sorted(
        node for node in forward & backward if node >= 0
    )
    old_to_new = {
        old_index: new_index for new_index, old_index in enumerate(kept_old_indices)
    }
    states = [
        DirectedEdgeState(
            index=new_index,
            tail=raw_states[old_index].tail,
            head=raw_states[old_index].head,
        )
        for new_index, old_index in enumerate(kept_old_indices)
    ]
    live_nodes = {SOURCE_NODE, SINK_NODE, *kept_old_indices}
    arcs: list[ModelArc] = []
    for source, target, cost_nm, kind, chain in arc_specs:
        if source not in live_nodes or target not in live_nodes:
            continue
        mapped_source = source if source < 0 else old_to_new[source]
        mapped_target = target if target < 0 else old_to_new[target]
        arcs.append(
            ModelArc(
                index=len(arcs),
                source=mapped_source,
                target=mapped_target,
                cost_nm=cost_nm,
                kind=kind,
                chain=chain,
            )
        )

    # Exact local curvature pruning.  If a transition makes v a true turn,
    # enters state (v,w), and that state has exactly one possible internal
    # successor (w,z), then every Euler realization containing this arc also
    # contains the consecutive turns v,w.  The shared physical edge must then
    # contain both minimum-radius tangent cuts.  Only a real circular-cut
    # overlap is impossible for the independent-arc partition; no arbitrary
    # reserve is applied.  Opposite turns that are eligible for a
    # composite chain remain available to the composite verifier.
    outgoing_arc_indices: dict[int, list[int]] = {}
    for candidate in arcs:
        if candidate.source >= 0:
            outgoing_arc_indices.setdefault(candidate.source, []).append(candidate.index)
    locally_curvature_infeasible: set[int] = set()
    for candidate in arcs:
        if candidate.source < 0 or candidate.target < 0:
            continue
        successors = [
            index
            for index in outgoing_arc_indices.get(candidate.target, [])
            if arcs[index].target >= 0
        ]
        if len(successors) != 1:
            continue
        successor = arcs[successors[0]]
        incoming_state = states[candidate.source]
        current_state = states[candidate.target]
        next_state = states[successor.target]
        first_corner = _corner_profile(
            graph,
            incoming_state.tail,
            incoming_state.head,
            current_state.head,
        )
        second_corner = _corner_profile(
            graph,
            current_state.tail,
            current_state.head,
            next_state.head,
        )
        if (
            first_corner is None
            or not first_corner.is_turn
            or second_corner is None
            or not second_corner.is_turn
        ):
            continue
        edge_span_nm = _round_positive(
            _edge_length(graph, current_state.tail, current_state.head)
            * NANOMETRES_PER_MM
        )
        eligible, _, fit_limited = _pair_expansion_flags(
            graph, first_corner, second_corner, edge_span_nm
        )
        if eligible:
            continue
        if fit_limited:
            locally_curvature_infeasible.add(candidate.index)

    # The internal state graph condenses into a DAG of strongly connected
    # components.  A source-to-sink walk can cross each condensation edge at
    # most once because there is no path back to an earlier component.  Only
    # arcs inside a cyclic SCC can therefore occur more than once.  This is an
    # exact per-arc bound, not a route-length heuristic, and substantially
    # tightens the integer relaxation on the larger final map.
    internal_outgoing: dict[int, list[int]] = {
        state.index: [] for state in states
    }
    internal_incoming: dict[int, list[int]] = {
        state.index: [] for state in states
    }
    for arc in arcs:
        if arc.source < 0 or arc.target < 0:
            continue
        internal_outgoing[arc.source].append(arc.target)
        internal_incoming[arc.target].append(arc.source)

    visited: set[int] = set()
    finish_order: list[int] = []
    for root in sorted(internal_outgoing):
        if root in visited:
            continue
        visited.add(root)
        stack: list[tuple[int, int]] = [(root, 0)]
        while stack:
            node, cursor = stack[-1]
            neighbours = internal_outgoing[node]
            if cursor < len(neighbours):
                neighbour = neighbours[cursor]
                stack[-1] = (node, cursor + 1)
                if neighbour not in visited:
                    visited.add(neighbour)
                    stack.append((neighbour, 0))
                continue
            finish_order.append(node)
            stack.pop()

    component_by_state: dict[int, int] = {}
    component_sizes: list[int] = []
    for root in reversed(finish_order):
        if root in component_by_state:
            continue
        component = len(component_sizes)
        size = 0
        stack = [root]
        component_by_state[root] = component
        while stack:
            node = stack.pop()
            size += 1
            for neighbour in internal_incoming[node]:
                if neighbour in component_by_state:
                    continue
                component_by_state[neighbour] = component
                stack.append(neighbour)
        component_sizes.append(size)

    required_vertices = tuple(
        sorted(name for name in graph.required if name != graph.start)
    )
    required_host_edges = tuple(
        sorted({edge for edge, _ in graph.required_edges.values()})
    )
    # Between consecutive first visits of required targets, a repeated state
    # encloses a positive-cost service-free closed subwalk and can be removed.
    # Thus this is a proof-derived finite bound, not a heuristic length cap.
    occurrence_limit = max(
        2,
        len(required_vertices) + len(required_host_edges) + 1,
    )

    # The MILP objective is physical topology length plus a small steering
    # penalty.  This is only an incumbent-producing fast fallback; its result
    # is never labelled globally shortest.
    turn_penalty_nm = _round_positive(
        FAST_MILP_TURN_PENALTY_MM * NANOMETRES_PER_MM
    )
    objective_costs_nm: list[int] = []
    turn_arc_count = 0
    for arc in arcs:
        objective_cost = int(arc.cost_nm)
        if arc.source >= 0 and arc.target >= 0:
            incoming_state = states[arc.source]
            current_state = states[arc.target]
            corner = _corner_profile(
                graph,
                incoming_state.tail,
                incoming_state.head,
                current_state.head,
            )
            if corner is not None and corner.is_turn:
                objective_cost += turn_penalty_nm
                turn_arc_count += 1
        objective_costs_nm.append(objective_cost)

    objective_unit_nm = 0
    for objective_cost in objective_costs_nm:
        if objective_cost > 0:
            objective_unit_nm = math.gcd(objective_unit_nm, objective_cost)
    if objective_unit_nm <= 0:
        raise RouteRequestError("invalid_objective", "route model has no positive edge cost")

    model = SparseMilp()
    model.objective_unit_nm = objective_unit_nm
    x_variables: list[int] = []
    arc_upper_bounds: list[int] = []
    length_units: list[int] = []
    objective_units: list[int] = []
    repeatable_arc_count = 0
    for arc in arcs:
        repeatable = (
            arc.source >= 0
            and arc.target >= 0
            and component_by_state[arc.source] == component_by_state[arc.target]
            and (
                component_sizes[component_by_state[arc.source]] > 1
                or arc.source == arc.target
            )
        )
        upper = 0 if arc.index in locally_curvature_infeasible else (
            occurrence_limit if repeatable else 1
        )
        if repeatable:
            repeatable_arc_count += 1
        arc_upper_bounds.append(upper)
        objective_cost_nm = objective_costs_nm[arc.index]
        if objective_cost_nm % objective_unit_nm != 0:
            raise RouteRequestError("invalid_objective", "edge cost normalization is not exact")
        length_units.append(arc.cost_nm // objective_unit_nm)
        objective_units.append(objective_cost_nm // objective_unit_nm)
        x_variables.append(
            model.add_variable(
                cost=objective_cost_nm // objective_unit_nm,
                upper=float(upper),
                integer=True,
            )
        )

    all_nodes = [SOURCE_NODE, SINK_NODE, *(state.index for state in states)]
    outgoing: dict[int, list[int]] = {node: [] for node in all_nodes}
    incoming: dict[int, list[int]] = {node: [] for node in all_nodes}
    for arc in arcs:
        outgoing[arc.source].append(arc.index)
        incoming[arc.target].append(arc.index)

    # Count-level Hall cuts for locally impossible consecutive-turn pairs.
    # At a state s, an Euler order pairs every incoming arc occurrence with
    # an outgoing occurrence.  If A enters s and B leaves s, at least
    # max(0, x_A + x_B - total_in(s)) occurrences must be paired A->B.  When
    # that pair is itself curvature-infeasible, every valid order therefore
    # satisfies x_A + x_B <= total_in(s).  This remains exact even when s has
    # several alternative predecessors/successors; it does not assume a
    # particular Euler ordering.
    locally_invalid_adjacency_pairs: set[tuple[int, int]] = set()
    for first_arc in arcs:
        if first_arc.source < 0 or first_arc.target < 0:
            continue
        successors = [
            index
            for index in outgoing.get(first_arc.target, [])
            if arcs[index].target >= 0
        ]
        if not successors:
            continue
        incoming_state = states[first_arc.source]
        current_state = states[first_arc.target]
        first_corner = _corner_profile(
            graph,
            incoming_state.tail,
            incoming_state.head,
            current_state.head,
        )
        if first_corner is None or not first_corner.is_turn:
            continue
        edge_span_nm = _round_positive(
            _edge_length(graph, current_state.tail, current_state.head)
            * NANOMETRES_PER_MM
        )
        for successor_index in successors:
            second_arc = arcs[successor_index]
            next_state = states[second_arc.target]
            second_corner = _corner_profile(
                graph,
                current_state.tail,
                current_state.head,
                next_state.head,
            )
            if second_corner is None or not second_corner.is_turn:
                continue
            composite, _, fit_limited = _pair_expansion_flags(
                graph, first_corner, second_corner, edge_span_nm
            )
            if composite or not fit_limited:
                continue
            locally_invalid_adjacency_pairs.add((first_arc.index, second_arc.index))

    for first_index, second_index in sorted(locally_invalid_adjacency_pairs):
        shared_state = arcs[first_index].target
        coefficients: dict[int, float] = {
            x_variables[first_index]: 1.0,
            x_variables[second_index]: 1.0,
        }
        for incoming_index in incoming.get(shared_state, []):
            variable = x_variables[incoming_index]
            coefficients[variable] = coefficients.get(variable, 0.0) - 1.0
        model.add_row(coefficients, upper=0.0)

    for node in all_nodes:
        coefficients: dict[int, float] = {}
        for arc_index in outgoing[node]:
            variable = x_variables[arc_index]
            coefficients[variable] = coefficients.get(variable, 0.0) + 1.0
        for arc_index in incoming[node]:
            variable = x_variables[arc_index]
            coefficients[variable] = coefficients.get(variable, 0.0) - 1.0
        rhs = 1.0 if node == SOURCE_NODE else (-1.0 if node == SINK_NODE else 0.0)
        model.add_row(coefficients, lower=rhs, upper=rhs)

    for required_name in required_vertices:
        candidates = [
            x_variables[arc.index]
            for arc in arcs
            if required_name in arc.chain
        ]
        if not candidates:
            raise RouteRequestError(
                "required_has_no_legal_passage",
                f"required point {required_name} has no legal directed-edge passage",
            )
        model.add_row({index: 1.0 for index in candidates}, lower=1.0)

    for left, right in required_host_edges:
        candidates = [
            x_variables[arc.index]
            for arc in arcs
            if any(
                _edge_key(chain_left, chain_right) == (left, right)
                for chain_left, chain_right in zip(arc.chain, arc.chain[1:])
            )
        ]
        if not candidates:
            raise RouteRequestError(
                "required_has_no_legal_passage",
                f"required host edge {left}-{right} has no legal traversal",
            )
        model.add_row({index: 1.0 for index in candidates}, lower=1.0)

    # Encode SOURCE reachability with one support-flow network.  The state
    # activation variables are continuous, not integer: for an integer route
    # count x, x <= upper * used forces used > 0 exactly when the state has a
    # selected incident arc, while used <= sum(incident x) prevents activation
    # of an unused state.  A disconnected positive component therefore has a
    # strictly positive total demand but no support-flow boundary, making it
    # infeasible.  This is the same projection as binary activation, with no
    # extra integer branch variables; network-flow integrality is not needed
    # for the reachability implication.
    support_capacity = len(states)
    support_flow_variables: list[int] = []
    for arc, variable, upper in zip(arcs, x_variables, arc_upper_bounds):
        flow = model.add_variable(
            upper=float(support_capacity * upper),
            integer=False,
        )
        support_flow_variables.append(flow)
        model.add_row(
            {flow: 1.0, variable: -float(support_capacity)},
            upper=0.0,
        )

    state_used_variables: list[int] = []
    for state in states:
        used = model.add_variable(upper=1.0, integer=False)
        state_used_variables.append(used)
        incident = sorted(set(outgoing[state.index] + incoming[state.index]))
        for arc_index in incident:
            model.add_row(
                {x_variables[arc_index]: 1.0, used: -float(arc_upper_bounds[arc_index])},
                upper=0.0,
            )
        coefficients = {used: 1.0}
        for arc_index in incident:
            variable = x_variables[arc_index]
            coefficients[variable] = coefficients.get(variable, 0.0) - 1.0
        model.add_row(coefficients, upper=0.0)

    root_balance: dict[int, float] = {}
    for arc_index in outgoing[SOURCE_NODE]:
        variable = support_flow_variables[arc_index]
        root_balance[variable] = root_balance.get(variable, 0.0) + 1.0
    for arc_index in incoming[SOURCE_NODE]:
        variable = support_flow_variables[arc_index]
        root_balance[variable] = root_balance.get(variable, 0.0) - 1.0
    for variable in state_used_variables:
        root_balance[variable] = root_balance.get(variable, 0.0) - 1.0
    model.add_row(root_balance, lower=0.0, upper=0.0)

    for state in states:
        coefficients: dict[int, float] = {
            state_used_variables[state.index]: -1.0
        }
        for arc_index in incoming[state.index]:
            variable = support_flow_variables[arc_index]
            coefficients[variable] = coefficients.get(variable, 0.0) + 1.0
        for arc_index in outgoing[state.index]:
            variable = support_flow_variables[arc_index]
            coefficients[variable] = coefficients.get(variable, 0.0) - 1.0
        model.add_row(coefficients, lower=0.0, upper=0.0)

    sink_balance: dict[int, float] = {}
    for arc_index in incoming[SINK_NODE]:
        variable = support_flow_variables[arc_index]
        sink_balance[variable] = sink_balance.get(variable, 0.0) + 1.0
    for arc_index in outgoing[SINK_NODE]:
        variable = support_flow_variables[arc_index]
        sink_balance[variable] = sink_balance.get(variable, 0.0) - 1.0
    model.add_row(sink_balance, lower=0.0, upper=0.0)

    metadata = {
        "x_variables": x_variables,
        "occurrence_limit": occurrence_limit,
        "objective_unit_nm": objective_unit_nm,
        "length_unit_nm": objective_unit_nm,
        "length_coefficients": length_units,
        "objective_coefficients": objective_units,
        "turn_penalty_mm": FAST_MILP_TURN_PENALTY_MM,
        "turn_arc_count": turn_arc_count,
        "arc_upper_bounds": arc_upper_bounds,
        "raw_state_count": len(raw_states),
        "raw_arc_count": len(arc_specs),
        "required_vertex_targets": len(required_vertices),
        "required_edge_targets": len(required_host_edges),
        "strongly_connected_components": len(component_sizes),
        "repeatable_arc_count": repeatable_arc_count,
        "single_use_arc_count": len(arcs) - repeatable_arc_count,
        "locally_curvature_infeasible_arc_count": len(locally_curvature_infeasible),
        "locally_invalid_adjacency_pair_count": len(locally_invalid_adjacency_pairs),
        "state_used_variables": state_used_variables,
        "support_flow_variables": support_flow_variables,
        "support_flow_capacity": support_capacity,
    }
    return states, arcs, model, metadata

def _insert_required_edge_points(graph: PlanGraph, route: Sequence[str]) -> list[str]:
    """Restore floating required names in traversal order for the editor.

    They remain absent from the optimization topology so they cannot shorten
    the tangent span, but the returned node list explicitly shows every
    covered required point and can be independently split/verified by JS.
    """

    services_by_edge: dict[tuple[str, str], list[str]] = {}
    for required_name, (edge, _) in graph.required_edges.items():
        services_by_edge.setdefault(edge, []).append(required_name)

    output = [route[0]]
    for left, right in zip(route, route[1:]):
        candidates = services_by_edge.get(_edge_key(left, right), [])
        if candidates:
            ax, ay = graph.coords[left]
            bx, by = graph.coords[right]
            dx = bx - ax
            dy = by - ay
            candidates = sorted(
                candidates,
                key=lambda name: (
                    (graph.coords[name][0] - ax) * dx +
                    (graph.coords[name][1] - ay) * dy,
                    name,
                ),
            )
            last_coord: tuple[float, float] | None = None
            for required_name in candidates:
                required_coord = graph.coords[required_name]
                if last_coord is not None and _distance(required_coord, last_coord) <= COINCIDENT_TOLERANCE_MM:
                    continue
                if required_name != output[-1] and required_name != right:
                    output.append(required_name)
                    last_coord = required_coord
        if right != output[-1]:
            output.append(right)
    return output




def _composite_point_on_segment(
    point: tuple[float, float],
    a: tuple[float, float],
    b: tuple[float, float],
    epsilon: float = 1.0e-9,
) -> bool:
    return (
        min(a[0], b[0]) - epsilon <= point[0] <= max(a[0], b[0]) + epsilon
        and min(a[1], b[1]) - epsilon <= point[1] <= max(a[1], b[1]) + epsilon
    )


def _composite_segments_intersect(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
    d: tuple[float, float],
) -> bool:
    if (
        max(a[0], b[0]) + 1.0e-9 < min(c[0], d[0])
        or max(c[0], d[0]) + 1.0e-9 < min(a[0], b[0])
        or max(a[1], b[1]) + 1.0e-9 < min(c[1], d[1])
        or max(c[1], d[1]) + 1.0e-9 < min(a[1], b[1])
    ):
        return False

    def cross(
        p: tuple[float, float],
        q: tuple[float, float],
        r: tuple[float, float],
    ) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    o1 = cross(a, b, c)
    o2 = cross(a, b, d)
    o3 = cross(c, d, a)
    o4 = cross(c, d, b)
    if (
        ((o1 > 1.0e-9 and o2 < -1.0e-9) or (o1 < -1.0e-9 and o2 > 1.0e-9))
        and ((o3 > 1.0e-9 and o4 < -1.0e-9) or (o3 < -1.0e-9 and o4 > 1.0e-9))
    ):
        return True
    if abs(o1) <= 1.0e-9 and _composite_point_on_segment(c, a, b):
        return True
    if abs(o2) <= 1.0e-9 and _composite_point_on_segment(d, a, b):
        return True
    if abs(o3) <= 1.0e-9 and _composite_point_on_segment(a, c, d):
        return True
    if abs(o4) <= 1.0e-9 and _composite_point_on_segment(b, c, d):
        return True
    return False


def _composite_curve_self_intersects(points: Sequence[tuple[float, float]]) -> bool:
    for first_index in range(len(points) - 1):
        a = points[first_index]
        b = points[first_index + 1]
        for second_index in range(first_index + 2, len(points) - 1):
            if _composite_segments_intersect(
                a,
                b,
                points[second_index],
                points[second_index + 1],
            ):
                return True
    return False


def _composite_original_progress_ledger(
    points: Sequence[tuple[float, float]],
    polyline: Sequence[tuple[float, float]],
) -> dict[str, Any] | None:
    """Mirror the editor's projection-order guard for one composite curve."""

    if len(polyline) < 2:
        return None
    cumulative = [0.0]
    for left, right in zip(polyline, polyline[1:]):
        cumulative.append(cumulative[-1] + _distance(left, right))

    previous_progress = -math.inf
    max_residual_mm = 0.0
    projected_progress_mm: list[float] = []
    for point in points:
        best_distance_sq = math.inf
        best_progress = 0.0
        for segment_index, (a, b) in enumerate(zip(polyline, polyline[1:])):
            dx = b[0] - a[0]
            dy = b[1] - a[1]
            length_sq = dx * dx + dy * dy
            if length_sq <= 1.0e-12:
                continue
            projection = ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length_sq
            projection = max(0.0, min(1.0, projection))
            projected_x = a[0] + projection * dx
            projected_y = a[1] + projection * dy
            distance_sq = (
                (point[0] - projected_x) * (point[0] - projected_x)
                + (point[1] - projected_y) * (point[1] - projected_y)
            )
            progress = cumulative[segment_index] + projection * math.sqrt(length_sq)
            if (
                distance_sq < best_distance_sq - 1.0e-9
                or (
                    abs(distance_sq - best_distance_sq) <= 1.0e-9
                    and progress < best_progress
                )
            ):
                best_distance_sq = distance_sq
                best_progress = progress
        if not math.isfinite(best_distance_sq) or best_progress + 1.0e-6 < previous_progress:
            return None
        previous_progress = max(previous_progress, best_progress)
        max_residual_mm = max(max_residual_mm, math.sqrt(best_distance_sq))
        projected_progress_mm.append(best_progress)

    if not projected_progress_mm:
        return None
    return {
        "start_mm": projected_progress_mm[0],
        "end_mm": projected_progress_mm[-1],
        "max_residual_mm": max_residual_mm,
        "monotonic": True,
        "sample_count": len(projected_progress_mm),
    }


def _corner_curve_points_mm(
    angle_rad: float,
    radius_mm: float,
    ease_mm: float,
) -> tuple[float, float, float, tuple[tuple[float, float], ...]]:
    """Reproduce buildCornerCurve, including its deterministic 2 mm grid."""

    if not (0.0 < angle_rad < math.pi) or radius_mm <= 0.0 or ease_mm < 0.0:
        raise RouteRequestError("invalid_smooth_geometry", "invalid corner envelope profile")
    if ease_mm <= 1.0e-9:
        length_mm = radius_mm * angle_rad
        tangent_mm = radius_mm * math.tan(angle_rad * 0.5)
        sample_count = max(4, math.ceil(length_mm / 2.0))
        points = tuple(
            (
                radius_mm * math.sin(angle_rad * sample / sample_count),
                radius_mm * (1.0 - math.cos(angle_rad * sample / sample_count)),
            )
            for sample in range(sample_count + 1)
        )
        return tangent_mm, tangent_mm, length_mm, points

    curvature = 1.0 / radius_mm
    ramp_length_mm = ease_mm
    arc_angle = angle_rad - curvature * ramp_length_mm
    if arc_angle < 0.0:
        ramp_length_mm = angle_rad / curvature
        arc_angle = 0.0
    arc_length_mm = arc_angle / curvature
    length_mm = 2.0 * ramp_length_mm + arc_length_mm
    sample_count = max(4, math.ceil(length_mm / 2.0))
    ds_mm = length_mm / sample_count
    x_mm = 0.0
    y_mm = 0.0
    heading = 0.0
    points_list = [(0.0, 0.0)]
    for sample_index in range(sample_count):
        midpoint_mm = (sample_index + 0.5) * ds_mm
        if ramp_length_mm > 1.0e-9 and midpoint_mm < ramp_length_mm:
            local_curvature = curvature * midpoint_mm / ramp_length_mm
        elif midpoint_mm < ramp_length_mm + arc_length_mm or ramp_length_mm <= 1.0e-9:
            local_curvature = curvature
        else:
            local_curvature = curvature * (length_mm - midpoint_mm) / ramp_length_mm
        midpoint_heading = heading + local_curvature * ds_mm * 0.5
        x_mm += ds_mm * math.cos(midpoint_heading)
        y_mm += ds_mm * math.sin(midpoint_heading)
        heading += local_curvature * ds_mm
        points_list.append((x_mm, y_mm))

    sin_heading = math.sin(heading)
    if abs(sin_heading) <= DIRECTION_EPS:
        raise RouteRequestError(
            "invalid_smooth_geometry", "corner envelope has an undefined tangent intersection"
        )
    tangent_out_mm = y_mm / sin_heading
    tangent_in_mm = x_mm - y_mm * math.cos(heading) / sin_heading
    return tangent_in_mm, tangent_out_mm, length_mm, tuple(points_list)


def _corner_reference_envelope_mm(
    angle_rad: float,
    radius_mm: float,
    ease_mm: float,
) -> float:
    """Maximum departure of one legal isolated corner from its host polyline."""

    tangent_in_mm, _, _, points = _corner_curve_points_mm(
        angle_rad, radius_mm, ease_mm
    )
    start = points[0]
    vertex = (tangent_in_mm, 0.0)
    end = points[-1]
    isolated_corner_envelope_mm = max(
        min(
            _segment_projection(point, start, vertex)[1],
            _segment_projection(point, vertex, end)[1],
        )
        for point in points
    )
    return isolated_corner_envelope_mm


def _point_close(
    left: tuple[float, float],
    right: tuple[float, float],
    epsilon: float = 1.0e-6,
) -> bool:
    return _distance(left, right) <= epsilon


def _cross_value(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _proper_segment_intersection(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
    d: tuple[float, float],
) -> tuple[float, float] | None:
    """Return the strict interior/interior crossing point, excluding touches."""

    epsilon = 1.0e-9
    if (
        max(a[0], b[0]) + epsilon < min(c[0], d[0])
        or max(c[0], d[0]) + epsilon < min(a[0], b[0])
        or max(a[1], b[1]) + epsilon < min(c[1], d[1])
        or max(c[1], d[1]) + epsilon < min(a[1], b[1])
    ):
        return None
    o1 = _cross_value(a, b, c)
    o2 = _cross_value(a, b, d)
    o3 = _cross_value(c, d, a)
    o4 = _cross_value(c, d, b)
    if not (
        ((o1 > epsilon and o2 < -epsilon) or (o1 < -epsilon and o2 > epsilon))
        and ((o3 > epsilon and o4 < -epsilon) or (o3 < -epsilon and o4 > epsilon))
    ):
        return None
    ab_x = b[0] - a[0]
    ab_y = b[1] - a[1]
    cd_x = d[0] - c[0]
    cd_y = d[1] - c[1]
    denominator = ab_x * cd_y - ab_y * cd_x
    if abs(denominator) <= epsilon:
        return None
    ac_x = c[0] - a[0]
    ac_y = c[1] - a[1]
    t = (ac_x * cd_y - ac_y * cd_x) / denominator
    return (a[0] + t * ab_x, a[1] + t * ab_y)


def _pair_expansion_flags(
    graph: PlanGraph,
    first_corner: CornerProfile,
    second_corner: CornerProfile,
    edge_span_nm: int,
) -> tuple[bool, bool, bool]:
    """Return (eligible, mandatory, fit_limited) for a turn pair.

    The preview window is only an eligibility gate.  A pair is fit-limited
    only when the actual minimum-radius circular tangent cuts overlap on the
    shared physical edge.  Close but non-overlapping turns are tried as
    independent arcs first and are merged only if that partition fails.
    """

    _, first_out_nm = _fixed_corner_tangent_nm(
        first_corner.angle_rad, graph.swerve_radius_mm, graph.swerve_ease_mm
    )
    second_in_nm, _ = _fixed_corner_tangent_nm(
        second_corner.angle_rad, graph.swerve_radius_mm, graph.swerve_ease_mm
    )
    nominal_gap_nm = edge_span_nm - first_out_nm - second_in_nm
    _, circular_first_out_nm = _fixed_corner_tangent_nm(
        first_corner.angle_rad, graph.swerve_radius_mm, 0.0
    )
    circular_second_in_nm, _ = _fixed_corner_tangent_nm(
        second_corner.angle_rad, graph.swerve_radius_mm, 0.0
    )
    # Independent circles are governed by their actual tangent occupancy:
    # the two cuts must fit in the shared physical span.  The former arbitrary
    # margin rejected valid pairs and forced unnecessary
    # composite curves.
    fit_limited = (
        circular_first_out_nm + circular_second_in_nm > edge_span_nm + 100.0
    )
    eligible = bool(
        graph.swerve_gap_nm > 0
        and (
            edge_span_nm <= graph.swerve_gap_nm
            or nominal_gap_nm <= 0
            or fit_limited
        )
    )
    # A nominal eased profile may overlap even though its easing can be
    # shortened to the requested minimum-radius circle.  It is not a hard
    # composite boundary; only real circular-cut overlap is mandatory.
    return eligible, bool(fit_limited), fit_limited


def _segments_collinear_overlap(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
    d: tuple[float, float],
) -> bool:
    epsilon = 1.0e-7
    if abs(_cross_value(a, b, c)) > epsilon or abs(_cross_value(a, b, d)) > epsilon:
        return False
    axis = 0 if abs(b[0] - a[0]) >= abs(b[1] - a[1]) else 1
    overlap = min(max(a[axis], b[axis]), max(c[axis], d[axis])) - max(
        min(a[axis], b[axis]), min(c[axis], d[axis])
    )
    return overlap > epsilon


def _curve_physical_edge_ledger(
    graph: PlanGraph,
    points: Sequence[tuple[float, float]],
    reference_polyline: Sequence[tuple[float, float]],
    allow_crossing: bool = False,
) -> dict[str, Any] | None:
    """Reject a curve that crosses a physical edge outside its local host chain."""

    allowed_edges: set[tuple[str, str]] = set()
    for edge in graph.physical_edges:
        edge_start = graph.coords[edge[0]]
        edge_end = graph.coords[edge[1]]
        if any(
            _segments_collinear_overlap(edge_start, edge_end, left, right)
            for left, right in zip(reference_polyline, reference_polyline[1:])
        ):
            allowed_edges.add(_edge_key(*edge))

    checked_pairs = 0
    start = points[0]
    end = points[-1]
    curve_bounds = (
        min(point[0] for point in points),
        min(point[1] for point in points),
        max(point[0] for point in points),
        max(point[1] for point in points),
    )
    for edge in graph.physical_edges:
        if _edge_key(*edge) in allowed_edges:
            continue
        edge_start = graph.coords[edge[0]]
        edge_end = graph.coords[edge[1]]
        if (
            curve_bounds[2] + 1.0e-9 < min(edge_start[0], edge_end[0])
            or max(edge_start[0], edge_end[0]) + 1.0e-9 < curve_bounds[0]
            or curve_bounds[3] + 1.0e-9 < min(edge_start[1], edge_end[1])
            or max(edge_start[1], edge_end[1]) + 1.0e-9 < curve_bounds[1]
        ):
            continue
        for curve_start, curve_end in zip(points, points[1:]):
            checked_pairs += 1
            crossing = _proper_segment_intersection(
                curve_start, curve_end, edge_start, edge_end
            )
            if crossing is not None:
                if allow_crossing:
                    continue
                return None
            if not _composite_segments_intersect(
                curve_start, curve_end, edge_start, edge_end
            ):
                continue
            if allow_crossing:
                continue
            endpoint_touch = any(
                _point_close(anchor, endpoint)
                for anchor in (start, end)
                for endpoint in (curve_start, curve_end, edge_start, edge_end)
            )
            if not endpoint_touch:
                return None
    return {
        "profile": graph.composite_collision_profile,
        "physical_edges": len(graph.physical_edges),
        "allowed_host_edges": len(allowed_edges),
        "checked_segment_pairs": checked_pairs,
        "collision_count": 0,
    }


def _circular_corner_pose(
    graph: PlanGraph,
    route: Sequence[str],
    route_index: int,
    corner: CornerProfile,
    angle_rad: float,
) -> tuple[tuple[float, float], float]:
    vertex = graph.coords[route[route_index]]
    previous = graph.coords[route[route_index - 1]]
    incoming_heading = math.atan2(vertex[1] - previous[1], vertex[0] - previous[0])
    incoming_length = _distance(previous, vertex)
    incoming_unit = (
        (vertex[0] - previous[0]) / incoming_length,
        (vertex[1] - previous[1]) / incoming_length,
    )
    tangent = graph.swerve_radius_mm * math.tan(corner.angle_rad * 0.5)
    start = (
        vertex[0] - incoming_unit[0] * tangent,
        vertex[1] - incoming_unit[1] * tangent,
    )
    local_x = graph.swerve_radius_mm * math.sin(angle_rad)
    local_y = graph.swerve_radius_mm * (1.0 - math.cos(angle_rad))
    cos_heading = math.cos(incoming_heading)
    sin_heading = math.sin(incoming_heading)
    point = (
        start[0] + local_x * cos_heading - corner.turn_sign * local_y * sin_heading,
        start[1] + local_x * sin_heading + corner.turn_sign * local_y * cos_heading,
    )
    return point, incoming_heading + corner.turn_sign * angle_rad


def _sample_circular_corner_arc(
    graph: PlanGraph,
    route: Sequence[str],
    route_index: int,
    corner: CornerProfile,
    from_angle: float,
    to_angle: float,
) -> tuple[tuple[tuple[float, float], ...], float]:
    length_mm = abs(to_angle - from_angle) * graph.swerve_radius_mm
    sample_count = max(2, math.ceil(length_mm / 2.0))
    points = tuple(
        _circular_corner_pose(
            graph,
            route,
            route_index,
            corner,
            from_angle + (to_angle - from_angle) * sample / sample_count,
        )[0]
        for sample in range(sample_count + 1)
    )
    return points, length_mm


def _circular_arc_blend(
    graph: PlanGraph,
    route: Sequence[str],
    first_index: int,
    last_index: int,
    first_corner: CornerProfile,
    last_corner: CornerProfile,
) -> dict[str, Any] | None:
    """Mirror the editor's local circle-body S-curve replacement."""

    radius_mm = graph.swerve_radius_mm
    k1 = first_corner.turn_sign / radius_mm
    k2 = last_corner.turn_sign / radius_mm
    last_vertex = graph.coords[route[last_index]]
    last_previous = graph.coords[route[last_index - 1]]
    last_phi = math.atan2(
        last_vertex[1] - last_previous[1],
        last_vertex[0] - last_previous[0],
    )

    def unwrap_near(angle: float, reference: float) -> float:
        while angle - reference > math.pi:
            angle -= 2.0 * math.pi
        while angle - reference < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def integrate(start: tuple[float, float], heading: float, ramp_mm: float) -> dict[str, Any]:
        total_mm = 2.0 * ramp_mm
        sample_count = max(24, math.ceil(total_mm * 2.0))
        ds_mm = total_mm / sample_count
        x_mm, y_mm = start
        points: list[tuple[float, float]] = [start]
        peak_curvature = 0.0
        for sample in range(sample_count):
            distance_mm = (sample + 0.5) * ds_mm
            curvature = (
                k1 * (1.0 - distance_mm / ramp_mm)
                if distance_mm < ramp_mm
                else k2 * ((distance_mm - ramp_mm) / ramp_mm)
            )
            midpoint_heading = heading + curvature * ds_mm * 0.5
            x_mm += ds_mm * math.cos(midpoint_heading)
            y_mm += ds_mm * math.sin(midpoint_heading)
            heading += curvature * ds_mm
            peak_curvature = max(peak_curvature, abs(curvature))
            points.append((x_mm, y_mm))
        return {
            "points": tuple(points),
            "heading": heading,
            "peak_curvature_per_mm": peak_curvature,
            "length_mm": total_mm,
        }

    def candidate(a: float, b: float, ramp_mm: float) -> dict[str, Any]:
        start, start_heading = _circular_corner_pose(
            graph, route, first_index, first_corner, a
        )
        target, target_heading = _circular_corner_pose(
            graph, route, last_index, last_corner, b
        )
        blend = integrate(start, start_heading, ramp_mm)
        heading_error = abs(unwrap_near(blend["heading"], target_heading) - target_heading)
        position_error = _distance(blend["points"][-1], target)
        return {
            "a": a,
            "b": b,
            "ramp_mm": ramp_mm,
            "start": start,
            "start_heading": start_heading,
            "target": target,
            "target_heading": target_heading,
            "blend": blend,
            "heading_error": heading_error,
            "position_error": position_error,
        }

    best: dict[str, Any] | None = None

    def consider(item: dict[str, Any]) -> None:
        nonlocal best
        if item["heading_error"] > 0.02 or item["position_error"] > 2.0:
            return
        item["score"] = item["position_error"] + item["heading_error"] * 100.0
        if best is None or item["score"] < best["score"]:
            best = item

    # Search both circle bodies symmetrically.  The former 10%-of-turn lower
    # bound made a valid 90-degree reverse blend depend on floating-point
    # rounding at exactly 9 degrees (P86->P43 is the concrete example).  A
    # fixed angular margin is the actual geometric requirement: keep a small
    # non-degenerate circular body at each end, then let the curvature,
    # progress and physical-edge ledgers decide whether the blend is legal.
    first_margin_rad = min(0.1, first_corner.angle_rad * 0.25)
    last_margin_rad = min(0.1, last_corner.angle_rad * 0.25)
    a = first_margin_rad
    while a <= first_corner.angle_rad - first_margin_rad + 1.0e-12:
        ramp_mm = 10.0
        while ramp_mm <= 160.0 + 1.0e-12:
            _, start_heading = _circular_corner_pose(
                graph, route, first_index, first_corner, a
            )
            expected_heading = start_heading + (k1 + k2) * ramp_mm * 0.5
            b = (expected_heading - last_phi) / last_corner.turn_sign
            if (
                b >= last_margin_rad - 1.0e-12
                and b <= last_corner.angle_rad - last_margin_rad + 1.0e-12
            ):
                consider(candidate(a, b, ramp_mm))
            ramp_mm += 2.0
        a += math.pi / 90.0
    if best is None:
        return None

    coarse = best
    a = max(first_margin_rad, coarse["a"] - math.pi / 45.0)
    a_limit = min(
        first_corner.angle_rad - first_margin_rad,
        coarse["a"] + math.pi / 45.0,
    )
    while a <= a_limit + 1.0e-12:
        ramp_mm = max(10.0, coarse["ramp_mm"] - 4.0)
        ramp_limit = min(160.0, coarse["ramp_mm"] + 4.0)
        while ramp_mm <= ramp_limit + 1.0e-12:
            _, start_heading = _circular_corner_pose(
                graph, route, first_index, first_corner, a
            )
            expected_heading = start_heading + (k1 + k2) * ramp_mm * 0.5
            b = (expected_heading - last_phi) / last_corner.turn_sign
            if (
                last_margin_rad - 1.0e-12
                <= b
                <= last_corner.angle_rad - last_margin_rad + 1.0e-12
            ):
                consider(candidate(a, b, ramp_mm))
            ramp_mm += 0.25
        a += math.pi / 360.0
    if best is None or best["blend"]["peak_curvature_per_mm"] > graph.max_curvature_per_mm + 1.0e-12:
        return None

    before, _ = _sample_circular_corner_arc(
        graph, route, first_index, first_corner, 0.0, best["a"]
    )
    after, _ = _sample_circular_corner_arc(
        graph, route, last_index, last_corner, best["b"], last_corner.angle_rad
    )
    points = list(before)
    points.extend(best["blend"]["points"][1:])
    points.extend(after[1:])
    target_index = len(before) + len(best["blend"]["points"]) - 2
    points[target_index] = best["target"]
    if _composite_curve_self_intersects(points):
        return None

    start = points[0]
    end = points[-1]
    reference_polyline = [start]
    reference_polyline.extend(graph.coords[route[index]] for index in range(first_index, last_index + 1))
    reference_polyline.append(end)
    progress_ledger = _composite_original_progress_ledger(points, reference_polyline)
    corridor_limit_mm = max(
        _corner_reference_envelope_mm(first_corner.angle_rad, radius_mm, 0.0),
        _corner_reference_envelope_mm(last_corner.angle_rad, radius_mm, 0.0),
    )
    if (
        progress_ledger is None
        or progress_ledger["max_residual_mm"] > corridor_limit_mm + 1.0e-6
    ):
        return None
    physical_edge_ledger = _curve_physical_edge_ledger(
        graph, points, reference_polyline
    )
    if physical_edge_ledger is None:
        return None
    length_mm = sum(_distance(left, right) for left, right in zip(points, points[1:]))
    tangent_in_mm = radius_mm * math.tan(first_corner.angle_rad * 0.5)
    tangent_out_mm = radius_mm * math.tan(last_corner.angle_rad * 0.5)
    return {
        "start": start,
        "end": end,
        "cut_in_nm": _round_positive(tangent_in_mm * NANOMETRES_PER_MM),
        "cut_out_nm": _round_positive(tangent_out_mm * NANOMETRES_PER_MM),
        "expansion_level": 0,
        "handle_ratio": 0.0,
        "curve_length_nm": _round_positive(length_mm * NANOMETRES_PER_MM),
        "peak_curvature_per_mm": max(
            abs(k1), abs(k2), best["blend"]["peak_curvature_per_mm"]
        ),
        "progress_ledger": progress_ledger,
        "corridor_limit_mm": corridor_limit_mm,
        "sampled_points": tuple(points),
        "reference_polyline": tuple(reference_polyline),
        "physical_edge_ledger": physical_edge_ledger,
    }


def _three_member_circular_chain(
    graph: PlanGraph,
    route: Sequence[str],
    member_indices: Sequence[int],
    corners: Mapping[int, CornerProfile],
) -> dict[str, Any] | None:
    """Join one legal circular turn to a mandatory overlapping pair.

    The first member remains a true minimum-radius arc.  Its outgoing tangent
    is connected to the second member's incoming tangent by the physical
    straight remainder, then the existing two-arc circular blend handles the
    mandatory overlap.  This keeps the curvature cap exact while treating the
    connected short-turn block as one exported composite feature.
    """

    if len(member_indices) != 3:
        return None
    first_index, middle_index, last_index = map(int, member_indices)
    first = corners.get(first_index)
    middle = corners.get(middle_index)
    last = corners.get(last_index)
    if first is None or middle is None or last is None:
        return None
    first_points, _ = _sample_circular_corner_arc(
        graph, route, first_index, first, 0.0, first.angle_rad
    )
    pair = _circular_arc_blend(
        graph, route, middle_index, last_index, middle, last
    )
    if pair is None:
        return None
    first_end = first_points[-1]
    pair_start = pair["sampled_points"][0]
    connector_length_mm = _distance(first_end, pair_start)
    connector_count = max(1, math.ceil(connector_length_mm / 2.0))
    connector = tuple(
        (
            first_end[0] + (pair_start[0] - first_end[0]) * sample / connector_count,
            first_end[1] + (pair_start[1] - first_end[1]) * sample / connector_count,
        )
        for sample in range(1, connector_count + 1)
    )
    points = tuple(first_points) + connector + tuple(pair["sampled_points"][1:])
    if _composite_curve_self_intersects(points):
        return None
    start = points[0]
    end = points[-1]
    reference_polyline = (
        start,
        *(graph.coords[route[index]] for index in range(first_index, last_index + 1)),
        end,
    )
    progress_ledger = _composite_original_progress_ledger(points, reference_polyline)
    if progress_ledger is None:
        return None
    corridor_limit_mm = max(
        _corner_reference_envelope_mm(first.angle_rad, graph.swerve_radius_mm, 0.0),
        _corner_reference_envelope_mm(middle.angle_rad, graph.swerve_radius_mm, 0.0),
        _corner_reference_envelope_mm(last.angle_rad, graph.swerve_radius_mm, 0.0),
    )
    if progress_ledger["max_residual_mm"] > corridor_limit_mm + 1.0e-6:
        return None
    physical_edge_ledger = _curve_physical_edge_ledger(
        graph, points, reference_polyline, allow_crossing=True
    )
    if physical_edge_ledger is None:
        return None
    length_mm = sum(_distance(left, right) for left, right in zip(points, points[1:]))
    return {
        "start": start,
        "end": end,
        "cut_in_nm": _round_positive(
            graph.swerve_radius_mm * math.tan(first.angle_rad * 0.5)
            * NANOMETRES_PER_MM
        ),
        "cut_out_nm": pair["cut_out_nm"],
        "expansion_level": 0,
        "handle_ratio": 0.0,
        "curve_length_nm": _round_positive(length_mm * NANOMETRES_PER_MM),
        "peak_curvature_per_mm": max(
            1.0 / graph.swerve_radius_mm,
            pair["peak_curvature_per_mm"],
        ),
        "progress_ledger": progress_ledger,
        "corridor_limit_mm": corridor_limit_mm,
        "sampled_points": points,
        "reference_polyline": reference_polyline,
        "physical_edge_ledger": physical_edge_ledger,
        "allow_crossing": True,
        "replacement_length_nm": _round_positive(
            (
                sum(
                    _edge_length(graph, route[index], route[index + 1])
                    for index in range(first_index, last_index)
                )
                + graph.swerve_radius_mm * math.tan(first.angle_rad * 0.5)
                + pair["cut_out_nm"] / NANOMETRES_PER_MM
            )
            * NANOMETRES_PER_MM
        ),
    }


def _smooth_curve_crossing_ledger(
    graph: PlanGraph,
    curve_records: Sequence[Mapping[str, Any]],
) -> dict[str, Any] | None:
    checked_pairs = 0
    for left_index, left in enumerate(curve_records):
        left_points = left["_sampled_points"]
        for right in curve_records[left_index + 1:]:
            if not left.get("composite", False) and not right.get("composite", False):
                continue
            if left.get("allow_crossing", False) or right.get("allow_crossing", False):
                continue
            right_points = right["_sampled_points"]
            left_bounds = (
                min(point[0] for point in left_points),
                min(point[1] for point in left_points),
                max(point[0] for point in left_points),
                max(point[1] for point in left_points),
            )
            right_bounds = (
                min(point[0] for point in right_points),
                min(point[1] for point in right_points),
                max(point[0] for point in right_points),
                max(point[1] for point in right_points),
            )
            if (
                left_bounds[2] + 1.0e-9 < right_bounds[0]
                or right_bounds[2] + 1.0e-9 < left_bounds[0]
                or left_bounds[3] + 1.0e-9 < right_bounds[1]
                or right_bounds[3] + 1.0e-9 < left_bounds[1]
            ):
                continue
            for left_start, left_end in zip(left_points, left_points[1:]):
                for right_start, right_end in zip(right_points, right_points[1:]):
                    checked_pairs += 1
                    crossing = _proper_segment_intersection(
                        left_start, left_end, right_start, right_end
                    )
                    if crossing is None:
                        continue
                    if any(_point_close(crossing, coord) for coord in graph.coords.values()):
                        continue
                    return None
    return {
        "profile": graph.composite_collision_profile,
        "curve_count": len(curve_records),
        "checked_segment_pairs": checked_pairs,
        "new_crossing_count": 0,
    }


def _quintic_composite_coarse_corridor_ok(
    start: tuple[float, float],
    end: tuple[float, float],
    start_heading: float,
    end_heading: float,
    handle_ratio: float,
    reference_polyline: Sequence[tuple[float, float]],
    corridor_limit_mm: float,
) -> bool:
    chord = _distance(start, end)
    handle = chord * handle_ratio
    d0 = (handle * math.cos(start_heading), handle * math.sin(start_heading))
    d1 = (handle * math.cos(end_heading), handle * math.sin(end_heading))
    points: list[tuple[float, float]] = []
    for sample in range(33):
        t = sample / 32.0
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        h00 = 1.0 - 10.0 * t3 + 15.0 * t4 - 6.0 * t5
        h10 = t - 6.0 * t3 + 8.0 * t4 - 3.0 * t5
        h01 = 10.0 * t3 - 15.0 * t4 + 6.0 * t5
        h11 = -4.0 * t3 + 7.0 * t4 - 3.0 * t5
        points.append(
            (
                h00 * start[0] + h10 * d0[0] + h01 * end[0] + h11 * d1[0],
                h00 * start[1] + h10 * d0[1] + h01 * end[1] + h11 * d1[1],
            )
        )
    progress = _composite_original_progress_ledger(points, reference_polyline)
    return bool(
        progress is not None
        and progress["max_residual_mm"] <= corridor_limit_mm + 1.0e-6
    )


def _quintic_composite_metrics(
    start: tuple[float, float],
    end: tuple[float, float],
    start_heading: float,
    end_heading: float,
    handle_ratio: float,
    max_curvature_per_mm: float,
) -> tuple[float, float, tuple[tuple[float, float], ...]] | None:
    """Build the editor's accepted Hermite member and its arc-length samples.

    The basis, derivative grid, curvature gate, arc-length resampling, and
    self-intersection test are the protocol shared with the editor's
    legacy endpoint-family implementation retained only for chains longer
    than two arcs; preferred short reverse pairs use the full-chain quintic
    family while non-preferred two-arc overlaps retain the circular blend.
    """

    chord = _distance(start, end)
    if (
        chord <= COINCIDENT_TOLERANCE_MM
        or handle_ratio * chord <= COINCIDENT_TOLERANCE_MM
        or max_curvature_per_mm <= 0.0
    ):
        return None
    handle = chord * handle_ratio
    d0 = (handle * math.cos(start_heading), handle * math.sin(start_heading))
    d1 = (handle * math.cos(end_heading), handle * math.sin(end_heading))
    dense_count = max(128, math.ceil(chord * 2.0))
    dense: list[tuple[tuple[float, float], float]] = []
    cumulative = [0.0]
    length_mm = 0.0
    peak_curvature = 0.0
    for sample in range(dense_count + 1):
        t = sample / dense_count
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        h00 = 1.0 - 10.0 * t3 + 15.0 * t4 - 6.0 * t5
        h10 = t - 6.0 * t3 + 8.0 * t4 - 3.0 * t5
        h01 = 10.0 * t3 - 15.0 * t4 + 6.0 * t5
        h11 = -4.0 * t3 + 7.0 * t4 - 3.0 * t5
        point = (
            h00 * start[0] + h10 * d0[0] + h01 * end[0] + h11 * d1[0],
            h00 * start[1] + h10 * d0[1] + h01 * end[1] + h11 * d1[1],
        )
        dense.append((point, t))
        if sample > 0:
            length_mm += _distance(dense[sample - 1][0], point)
            cumulative.append(length_mm)

        dh00 = -30.0 * t2 + 60.0 * t3 - 30.0 * t4
        dh10 = 1.0 - 18.0 * t2 + 32.0 * t3 - 15.0 * t4
        dh01 = 30.0 * t2 - 60.0 * t3 + 30.0 * t4
        dh11 = -12.0 * t2 + 28.0 * t3 - 15.0 * t4
        ddh00 = -60.0 * t + 180.0 * t2 - 120.0 * t3
        ddh10 = -36.0 * t + 96.0 * t2 - 60.0 * t3
        ddh01 = 60.0 * t - 180.0 * t2 + 120.0 * t3
        ddh11 = -24.0 * t + 84.0 * t2 - 60.0 * t3
        dx = dh00 * start[0] + dh10 * d0[0] + dh01 * end[0] + dh11 * d1[0]
        dy = dh00 * start[1] + dh10 * d0[1] + dh01 * end[1] + dh11 * d1[1]
        ddx = ddh00 * start[0] + ddh10 * d0[0] + ddh01 * end[0] + ddh11 * d1[0]
        ddy = ddh00 * start[1] + ddh10 * d0[1] + ddh01 * end[1] + ddh11 * d1[1]
        speed_sq = dx * dx + dy * dy
        if speed_sq <= 1.0e-12 or not math.isfinite(speed_sq):
            return None
        curvature = abs(dx * ddy - dy * ddx) / (speed_sq ** 1.5)
        if not math.isfinite(curvature):
            return None
        peak_curvature = max(peak_curvature, curvature)
    if (
        length_mm <= COINCIDENT_TOLERANCE_MM
        or not math.isfinite(length_mm)
        or not math.isfinite(peak_curvature)
        or peak_curvature
        > max_curvature_per_mm * COMPOSITE_CURVATURE_SAFETY_RATIO + 1.0e-12
    ):
        return None

    sample_count = max(4, math.ceil(length_mm / 2.0))
    ds_mm = length_mm / sample_count
    points: list[tuple[float, float]] = []
    cursor = 0
    for sample in range(sample_count + 1):
        target = min(length_mm, sample * ds_mm)
        while cursor + 1 < len(cumulative) and cumulative[cursor + 1] < target:
            cursor += 1
        left_point, left_t = dense[cursor]
        right_index = min(len(dense) - 1, cursor + 1)
        right_point, right_t = dense[right_index]
        span = cumulative[right_index] - cumulative[cursor]
        fraction = (target - cumulative[cursor]) / span if span > 1.0e-9 else 0.0
        t = left_t + (right_t - left_t) * fraction
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        h00 = 1.0 - 10.0 * t3 + 15.0 * t4 - 6.0 * t5
        h10 = t - 6.0 * t3 + 8.0 * t4 - 3.0 * t5
        h01 = 10.0 * t3 - 15.0 * t4 + 6.0 * t5
        h11 = -4.0 * t3 + 7.0 * t4 - 3.0 * t5
        points.append(
            (
                h00 * start[0] + h10 * d0[0] + h01 * end[0] + h11 * d1[0],
                h00 * start[1] + h10 * d0[1] + h01 * end[1] + h11 * d1[1],
            )
        )
    points[0] = start
    points[-1] = end
    if _composite_curve_self_intersects(points):
        return None
    return length_mm, peak_curvature, tuple(points)


def _verify_route(
    graph: PlanGraph,
    route: Sequence[str],
    *,
    deadline_s: float | None = None,
    max_partition_attempts: int | None = None,
    require_radius_infeasible_for_composite: bool = False,
) -> dict[str, Any]:
    """Independently verify topology and the curvature-limited smoothing policy."""

    if len(route) < 3 or route[0] != graph.start or route[-1] != graph.start:
        raise RouteRequestError("verification_failed", "route endpoints are invalid")
    if sum(1 for name in route if name == graph.start) != 2:
        raise RouteRequestError("verification_failed", "route crosses the start before finish")
    if graph.start_direction is not None and route[1] != graph.start_direction:
        raise RouteRequestError("verification_failed", "route violates start_dir")

    edge_lengths_nm: list[int] = []
    route_edges: list[tuple[str, str]] = []
    for left, right in zip(route, route[1:]):
        if right not in graph.adjacency.get(left, ()):
            raise RouteRequestError("verification_failed", f"route uses non-edge {left}-{right}")
        route_edges.append(_edge_key(left, right))
        edge_lengths_nm.append(
            _round_positive(_edge_length(graph, left, right) * NANOMETRES_PER_MM)
        )
    topology_length_nm = sum(edge_lengths_nm)

    if not graph.required.issubset(set(route)):
        missing = sorted(graph.required - set(route))
        raise RouteRequestError("verification_failed", f"route misses required points: {', '.join(missing)}")
    traversed_edges = set(route_edges)
    missing_edge_services = sorted(
        name for name, (edge, _) in graph.required_edges.items() if edge not in traversed_edges
    )
    if missing_edge_services:
        raise RouteRequestError(
            "verification_failed",
            f"route misses required edge services: {', '.join(missing_edge_services)}",
        )

    def span_nm(left_index: int, right_index: int) -> int:
        return sum(edge_lengths_nm[left_index:right_index])

    turns: list[dict[str, Any]] = []
    for route_index in range(1, len(route) - 1):
        if route[route_index - 1] == route[route_index + 1]:
            raise RouteRequestError("verification_failed", f"route reverses at {route[route_index]}")
        corner = _corner_profile(
            graph, route[route_index - 1], route[route_index], route[route_index + 1]
        )
        if corner is None:
            raise RouteRequestError(
                "verification_failed", f"route has an invalid corner at {route[route_index]}"
            )
        if not corner.is_turn:
            continue
        nominal_in_mm, nominal_out_mm, _ = _corner_curve_metrics_mm(
            corner.angle_rad, graph.swerve_radius_mm, graph.swerve_ease_mm
        )
        turns.append(
            {
                "route_index": route_index,
                "corner": corner,
                "nominal_in_nm": _round_positive(nominal_in_mm * NANOMETRES_PER_MM),
                "nominal_out_nm": _round_positive(nominal_out_mm * NANOMETRES_PER_MM),
            }
        )
    if not turns:
        raise RouteRequestError("verification_failed", "route has no true turn")

    # A preview-close opposite-turn pair is attempted as one curvature-
    # continuous manoeuvre before two independent arcs.  Geometry, monotone
    # progress and physical-edge collision checks remain hard acceptance
    # gates; if the preferred composite is infeasible, the independent-arc
    # partition remains available.  Same-direction close turns keep the
    # smaller independent partition unless a real tangent overlap requires a
    # composite.
    eligible_after: list[bool] = []
    mandatory_after: list[bool] = []
    preferred_after: list[bool] = []
    pair_ledger: list[dict[str, Any]] = []
    for left, right in zip(turns, turns[1:]):
        left_corner = left["corner"]
        right_corner = right["corner"]
        separation_nm = span_nm(left["route_index"], right["route_index"])
        straight_gap_nm = separation_nm - left["nominal_out_nm"] - right["nominal_in_nm"]
        opposite = left_corner.turn_sign * right_corner.turn_sign < 0
        eligible, mandatory, _ = _pair_expansion_flags(
            graph, left_corner, right_corner, separation_nm
        )
        eligible_after.append(eligible)
        mandatory_after.append(mandatory)
        preferred_after.append(bool(
            eligible and opposite and separation_nm <= graph.swerve_gap_nm
        ))
        pair_ledger.append(
            {
                "from_vertex": route[left["route_index"]],
                "to_vertex": route[right["route_index"]],
                "opposite": opposite,
                "span_nm": separation_nm,
                "nominal_straight_gap_nm": straight_gap_nm,
                "composite": False,
                "chain_horizon_merge": separation_nm <= graph.swerve_gap_nm,
                "eligible_for_expansion": eligible,
                "mandatory_overlap": mandatory,
                "preferred_close_reverse": preferred_after[-1],
            }
        )

    if require_radius_infeasible_for_composite:
        # Fast mode is deliberately stricter than the historical preview
        # policy: an independent 150 mm-radius fit must be attempted first.
        # Once such a pair is genuinely infeasible, allow a bounded forward
        # search through nearby subsequent turns to find the first feasible
        # composite chain.  A distant, independently feasible turn cannot
        # seed or join this search.
        forward_search = False
        for position, pair in enumerate(pair_ledger):
            if mandatory_after[position]:
                forward_search = True
            if forward_search:
                pair_span_nm = int(pair["span_nm"])
                if mandatory_after[position] or pair_span_nm <= graph.swerve_gap_nm:
                    eligible_after[position] = True
                else:
                    forward_search = False
                    eligible_after[position] = False
            else:
                eligible_after[position] = (
                    mandatory_after[position] or preferred_after[position]
                )

    # The historical verifier could absorb a short feasible prefix before a
    # mandatory overlap.  Fast mode keeps the user's rule literal: a
    # composite may start only at the turn whose independent 150 mm fit is
    # actually infeasible, never at an otherwise feasible predecessor.
    if not require_radius_infeasible_for_composite:
        for position in range(len(mandatory_after) - 2, -1, -1):
            if mandatory_after[position + 1] and (
                span_nm(turns[position]["route_index"], turns[position + 1]["route_index"])
                - turns[position]["nominal_out_nm"]
                - turns[position + 1]["nominal_in_nm"]
                <= _round_positive(COMPOSITE_PREFIX_GAP_MM * NANOMETRES_PER_MM)
            ):
                eligible_after[position] = True
                mandatory_after[position] = True

    # Keep the Python verifier bit-for-bit aligned with the editor protocol:
    # once a mandatory overlap owns its immediately preceding short pair, the
    # ownership propagates through the already eligible short-turn window.
    # Without this pass Python could accept a smaller partition which the
    # browser correctly rejects as one incomplete compound manoeuvre.
    if not require_radius_infeasible_for_composite:
        for position in range(len(mandatory_after) - 2, -1, -1):
            if eligible_after[position] and mandatory_after[position + 1]:
                mandatory_after[position] = True

    def cluster_end(first: int) -> int:
        end = first
        while end < len(turns) - 1 and eligible_after[end]:
            end += 1
        return end

    def cluster_groupings(first: int, last: int) -> Iterator[tuple[tuple[int, int], ...]]:
        if first > last:
            yield ()
            return
        # Prefer the shortest close reverse pair, then the independent first
        # turn, then longer composites.  This removes a 25 mm token straight
        # between opposing R150 arcs without allowing a distant turn into the
        # chain.  Every yielded group still passes the full finite verifier.
        group_lasts = list(range(first, last + 1))
        if (
            first < last
            and preferred_after[first]
            and eligible_after[first]
        ):
            group_lasts = [first + 1, first, *range(first + 2, last + 1)]
        for group_last in group_lasts:
            if group_last < last and mandatory_after[group_last]:
                continue
            for suffix in cluster_groupings(group_last + 1, last):
                yield ((first, group_last), *suffix)

    def complete_groupings() -> Iterator[tuple[tuple[int, int], ...]]:
        def combine_from(first: int) -> Iterator[tuple[tuple[int, int], ...]]:
            if first >= len(turns):
                yield ()
                return
            last = cluster_end(first)
            for cluster_groups in cluster_groupings(first, last):
                for suffix_groups in combine_from(last + 1):
                    yield (*cluster_groups, *suffix_groups)

        yield from combine_from(0)

    def minimum_group_in_cut_nm(group: tuple[int, int]) -> int:
        group_first, group_last = group
        first = turns[group_first]
        if group_first != group_last:
            return int(first["nominal_in_nm"])
        circular_in_mm, _, _ = _corner_curve_metrics_mm(
            first["corner"].angle_rad,
            graph.swerve_radius_mm,
            0.0,
        )
        return _round_positive(circular_in_mm * NANOMETRES_PER_MM)

    def evaluate_groups(
        groups: tuple[tuple[int, int], ...],
    ) -> tuple[list[dict[str, Any]], int, int, int, float, dict[str, Any]]:
        smoothing_ledger: list[dict[str, Any]] = []
        replaced_polyline_nm = 0
        curve_length_nm = 0
        composite_count = 0
        peak_curvature = 0.0
        reserved_before_nm = 0

        for group_index, (group_first, group_last) in enumerate(groups):
            if group_index > 0 and groups[group_index - 1][1] + 1 != group_first:
                raise RouteRequestError(
                    "verification_failed", "composite partition is not contiguous"
                )
            first = turns[group_first]
            last = turns[group_last]
            first_index = int(first["route_index"])
            last_index = int(last["route_index"])
            previous_turn_index = (
                0 if group_first == 0 else int(turns[group_first - 1]["route_index"])
            )
            next_turn_index = (
                len(route) - 1
                if group_last + 1 == len(turns)
                else int(turns[group_last + 1]["route_index"])
            )
            incoming_span_nm = span_nm(previous_turn_index, first_index)
            outgoing_span_nm = span_nm(last_index, next_turn_index)
            reserved_after_nm = (
                0
                if group_index + 1 == len(groups)
                else minimum_group_in_cut_nm(groups[group_index + 1])
            )
            available_in_nm = incoming_span_nm - reserved_before_nm
            available_out_nm = outgoing_span_nm - reserved_after_nm
            if min(available_in_nm, available_out_nm) <= 0:
                raise RouteRequestError(
                    "curvature_infeasible",
                    "finite smoothing partition has no remaining boundary span",
                )

            if group_first == group_last:
                smooth = _smooth_turn_profile_with_cut_limits(
                    graph,
                    first["corner"].angle_rad,
                    _round_positive(available_in_nm),
                    _round_positive(available_out_nm),
                )
                local_replaced_nm = smooth.tangent_in_nm + smooth.tangent_out_nm
                local_peak = 1.0 / smooth.actual_radius_mm
                vertex = graph.coords[route[first_index]]
                before = graph.coords[route[first_index - 1]]
                incoming_heading = math.atan2(
                    vertex[1] - before[1], vertex[0] - before[0]
                )
                tangent_in_mm, _, _, local_points = _corner_curve_points_mm(
                    first["corner"].angle_rad,
                    smooth.actual_radius_mm,
                    smooth.actual_ease_mm,
                )
                start = (
                    vertex[0] - tangent_in_mm * math.cos(incoming_heading),
                    vertex[1] - tangent_in_mm * math.sin(incoming_heading),
                )
                turn_sign = first["corner"].turn_sign
                cos_heading = math.cos(incoming_heading)
                sin_heading = math.sin(incoming_heading)
                sampled_points = tuple(
                    (
                        start[0] + point[0] * cos_heading - turn_sign * point[1] * sin_heading,
                        start[1] + point[0] * sin_heading + turn_sign * point[1] * cos_heading,
                    )
                    for point in local_points
                )
                reference_polyline = (start, vertex, sampled_points[-1])
                progress_ledger = _composite_original_progress_ledger(
                    sampled_points, reference_polyline
                )
                physical_edge_ledger = _curve_physical_edge_ledger(
                    graph, sampled_points, reference_polyline
                )
                corridor_limit_mm = _corner_reference_envelope_mm(
                    first["corner"].angle_rad,
                    smooth.actual_radius_mm,
                    smooth.actual_ease_mm,
                )
                if (
                    progress_ledger is None
                    or physical_edge_ledger is None
                    or progress_ledger["max_residual_mm"] > corridor_limit_mm + 1.0e-6
                ):
                    raise RouteRequestError(
                        "smoothing_infeasible",
                        "finite smoothing partition leaves the physical track envelope",
                    )
                smoothing_ledger.append(
                    {
                        "kind": "single_turn",
                        "composite": False,
                        "vertex": route[first_index],
                        "route_index": first_index,
                        "tangent_in_nm": smooth.tangent_in_nm,
                        "tangent_out_nm": smooth.tangent_out_nm,
                        "available_in_nm": available_in_nm,
                        "available_out_nm": available_out_nm,
                        "reserved_before_nm": reserved_before_nm,
                        "reserved_after_nm": reserved_after_nm,
                        "curve_length_nm": smooth.curve_length_nm,
                        "replaced_polyline_nm": local_replaced_nm,
                        "entry_s_nm": span_nm(0, first_index) - smooth.tangent_in_nm,
                        "exit_s_nm": span_nm(0, first_index) + smooth.tangent_out_nm,
                        "radius_mm": smooth.actual_radius_mm,
                        "ease_mm": smooth.actual_ease_mm,
                        "peak_curvature_per_mm": local_peak,
                        "corridor_profile": graph.composite_corridor_profile,
                        "corridor_limit_mm": corridor_limit_mm,
                        "progress_ledger": progress_ledger,
                        "physical_edge_ledger": physical_edge_ledger,
                        "_sampled_points": sampled_points,
                    }
                )
                replaced_polyline_nm += local_replaced_nm
                curve_length_nm += smooth.curve_length_nm
                peak_curvature = max(peak_curvature, local_peak)
                reserved_before_nm = smooth.tangent_out_nm
                continue

            composite_count += 1
            base_in_nm = min(available_in_nm, int(first["nominal_in_nm"]))
            base_out_nm = min(available_out_nm, int(last["nominal_out_nm"]))
            if min(base_in_nm, base_out_nm) <= 0:
                raise RouteRequestError(
                    "curvature_infeasible",
                    "finite composite member has no anchor span",
                )
            max_in_nm = available_in_nm
            max_out_nm = available_out_nm
            first_corner = first["corner"]
            last_corner = last["corner"]
            first_coord = graph.coords[route[first_index]]
            last_coord = graph.coords[route[last_index]]
            before_coord = graph.coords[route[first_index - 1]]
            after_coord = graph.coords[route[last_index + 1]]
            incoming_heading = math.atan2(
                first_coord[1] - before_coord[1], first_coord[0] - before_coord[0]
            )
            outgoing_heading = math.atan2(
                after_coord[1] - last_coord[1], after_coord[0] - last_coord[0]
            )
            fit: dict[str, Any] | None = None
            preferred_pair = bool(
                group_last == group_first + 1
                and pair_ledger[group_first].get("preferred_close_reverse", False)
            )
            if (
                graph.swerve_ease_mm <= 1.0e-9
                and group_last == group_first + 2
            ):
                member_route_indices = [
                    int(turns[offset]["route_index"])
                    for offset in range(group_first, group_last + 1)
                ]
                member_corners = {
                    index: turns[offset]["corner"]
                    for offset, index in zip(
                        range(group_first, group_last + 1), member_route_indices
                    )
                }
                chain_fit = _three_member_circular_chain(
                    graph, route, member_route_indices, member_corners
                )
                if chain_fit is not None:
                    fit = chain_fit
            # A close opposing pair should replace the whole short S-bend.
            # Prefer the endpoint-tangent curve for it; the circular blend is
            # still a fully verified fallback when that curve cannot fit.
            if fit is None and not preferred_pair and (
                graph.swerve_ease_mm <= 1.0e-9
                and group_last == group_first + 1
                and int(first["nominal_in_nm"]) <= available_in_nm
                and int(last["nominal_out_nm"]) <= available_out_nm
            ):
                fit = _circular_arc_blend(
                    graph,
                    route,
                    first_index,
                    last_index,
                    first_corner,
                    last_corner,
                )
            for expansion_level in (
                () if fit is not None else range(COMPOSITE_EXPANSION_STEPS + 1)
            ):
                fraction = expansion_level / COMPOSITE_EXPANSION_STEPS
                cut_in_nm = _round_positive(
                    base_in_nm + (max_in_nm - base_in_nm) * fraction
                )
                cut_out_nm = _round_positive(
                    base_out_nm + (max_out_nm - base_out_nm) * fraction
                )
                cut_in_mm = cut_in_nm / NANOMETRES_PER_MM
                cut_out_mm = cut_out_nm / NANOMETRES_PER_MM
                start = (
                    first_coord[0] - cut_in_mm * math.cos(incoming_heading),
                    first_coord[1] - cut_in_mm * math.sin(incoming_heading),
                )
                end = (
                    last_coord[0] + cut_out_mm * math.cos(outgoing_heading),
                    last_coord[1] + cut_out_mm * math.sin(outgoing_heading),
                )
                reference_polyline = [start]
                reference_polyline.extend(
                    graph.coords[route[route_index]]
                    for route_index in range(first_index, last_index + 1)
                )
                reference_polyline.append(end)
                corridor_limit_mm = max(
                    _corner_reference_envelope_mm(
                        turns[offset]["corner"].angle_rad,
                        graph.swerve_radius_mm,
                        graph.swerve_ease_mm,
                    )
                    for offset in range(group_first, group_last + 1)
                )
                if preferred_pair:
                    corridor_limit_mm = max(
                        corridor_limit_mm,
                        COMPOSITE_CHAIN_CORRIDOR_RATIO
                        * span_nm(first_index, last_index)
                        / NANOMETRES_PER_MM,
                    )
                feasible_at_level: list[dict[str, Any]] = []
                handle_steps = round(
                    (COMPOSITE_HANDLE_RATIO_MAX - COMPOSITE_HANDLE_RATIO_MIN)
                    / COMPOSITE_HANDLE_RATIO_STEP
                )
                for handle_step in range(handle_steps + 1):
                    handle_ratio = (
                        COMPOSITE_HANDLE_RATIO_MIN
                        + handle_step * COMPOSITE_HANDLE_RATIO_STEP
                    )
                    if not _quintic_composite_coarse_corridor_ok(
                        start,
                        end,
                        incoming_heading,
                        outgoing_heading,
                        handle_ratio,
                        reference_polyline,
                        corridor_limit_mm,
                    ):
                        continue
                    metrics = _quintic_composite_metrics(
                        start,
                        end,
                        incoming_heading,
                        outgoing_heading,
                        handle_ratio,
                        graph.max_curvature_per_mm,
                    )
                    if metrics is None:
                        continue
                    length_mm, local_peak, sampled_points = metrics
                    progress_ledger = _composite_original_progress_ledger(
                        sampled_points, reference_polyline
                    )
                    if progress_ledger is None:
                        continue
                    feasible_at_level.append(
                        {
                            "start": start,
                            "end": end,
                            "cut_in_nm": cut_in_nm,
                            "cut_out_nm": cut_out_nm,
                            "expansion_level": expansion_level,
                            "handle_ratio": handle_ratio,
                            "curve_length_nm": _round_positive(
                                length_mm * NANOMETRES_PER_MM
                            ),
                            "peak_curvature_per_mm": local_peak,
                            "progress_ledger": progress_ledger,
                            "corridor_limit_mm": corridor_limit_mm,
                            "sampled_points": sampled_points,
                            "reference_polyline": tuple(reference_polyline),
                        }
                    )
                for candidate_fit in sorted(
                    feasible_at_level,
                    key=lambda item: (
                        item["curve_length_nm"],
                        item["peak_curvature_per_mm"],
                        item["handle_ratio"],
                    ),
                ):
                    physical_edge_ledger = _curve_physical_edge_ledger(
                        graph,
                        candidate_fit["sampled_points"],
                        candidate_fit["reference_polyline"],
                    )
                    if physical_edge_ledger is None:
                        continue
                    candidate_fit["physical_edge_ledger"] = physical_edge_ledger
                    fit = candidate_fit
                    break
                if fit is not None:
                    break
            if fit is None and preferred_pair and (
                graph.swerve_ease_mm <= 1.0e-9
                and int(first["nominal_in_nm"]) <= available_in_nm
                and int(last["nominal_out_nm"]) <= available_out_nm
            ):
                fit = _circular_arc_blend(
                    graph,
                    route,
                    first_index,
                    last_index,
                    first_corner,
                    last_corner,
                )
            if fit is None:
                raise RouteRequestError(
                    "smoothing_infeasible",
                    "finite composite member has no accepted curve",
                )

            internal_nm = span_nm(first_index, last_index)
            local_replaced_nm = fit["cut_in_nm"] + internal_nm + fit["cut_out_nm"]
            smoothing_ledger.append(
                {
                    "kind": "composite_short_chain",
                    "composite": True,
                    "profile_version": graph.composite_profile_version,
                    "members": [
                        route[int(turns[offset]["route_index"])]
                        for offset in range(group_first, group_last + 1)
                    ],
                    "first_route_index": first_index,
                    "last_route_index": last_index,
                    "start_anchor": [fit["start"][0], fit["start"][1]],
                    "end_anchor": [fit["end"][0], fit["end"][1]],
                    "cut_in_nm": fit["cut_in_nm"],
                    "cut_out_nm": fit["cut_out_nm"],
                    "max_cut_in_nm": max_in_nm,
                    "max_cut_out_nm": max_out_nm,
                    "reserved_before_nm": reserved_before_nm,
                    "reserved_after_nm": reserved_after_nm,
                    "expansion_level": fit["expansion_level"],
                    "handle_ratio": fit["handle_ratio"],
                    "curve_length_nm": fit["curve_length_nm"],
                    "replaced_polyline_nm": local_replaced_nm,
                    "entry_s_nm": span_nm(0, first_index) - fit["cut_in_nm"],
                    "exit_s_nm": span_nm(0, last_index) + fit["cut_out_nm"],
                    "peak_curvature_per_mm": fit["peak_curvature_per_mm"],
                    "max_curvature_per_mm": graph.max_curvature_per_mm,
                    "progress_ledger": fit["progress_ledger"],
                    "self_intersection_checked": True,
                    "corridor_profile": graph.composite_corridor_profile,
                    "corridor_limit_mm": fit["corridor_limit_mm"],
                    "physical_edge_ledger": fit["physical_edge_ledger"],
                    "allow_crossing": bool(fit.get("allow_crossing", False)),
                    "composite_reason": (
                        "independent_radius_infeasible"
                        if require_radius_infeasible_for_composite
                        else (
                            "preferred_close_reverse_pair"
                            if group_last == group_first + 1
                            and preferred_after[group_first]
                            else "minimum_radius_tangent_overlap_or_partition"
                        )
                    ),
                    "search_direction": "forward" if require_radius_infeasible_for_composite else "local",
                    "crossed_intermediate_nodes": bool(
                        (group_last - group_first) > 1 or fit.get("allow_crossing", False)
                    ),
                    "_sampled_points": fit["sampled_points"],
                }
            )
            replaced_polyline_nm += local_replaced_nm
            curve_length_nm += fit["curve_length_nm"]
            peak_curvature = max(peak_curvature, fit["peak_curvature_per_mm"])
            reserved_before_nm = fit["cut_out_nm"]

        smooth_crossing_ledger = _smooth_curve_crossing_ledger(graph, smoothing_ledger)
        if smooth_crossing_ledger is None:
            raise RouteRequestError(
                "smoothing_infeasible",
                "finite smoothing partition creates an off-topology crossing",
            )
        return (
            smoothing_ledger,
            replaced_polyline_nm,
            curve_length_nm,
            composite_count,
            peak_curvature,
            smooth_crossing_ledger,
        )

    selected_groups: tuple[tuple[int, int], ...] | None = None
    selected_evaluation: tuple[
        list[dict[str, Any]], int, int, int, float, dict[str, Any]
    ] | None = None
    partition_attempts = 0
    for candidate_groups in complete_groupings():
        if deadline_s is not None and time.perf_counter() >= deadline_s:
            raise RouteRequestError(
                "smoothing_search_timeout",
                "fast smoothing partition search exceeded its per-route deadline",
            )
        if (
            max_partition_attempts is not None
            and partition_attempts >= max_partition_attempts
        ):
            raise RouteRequestError(
                "smoothing_search_limit",
                "fast smoothing partition search reached its attempt limit",
            )
        partition_attempts += 1
        try:
            candidate_evaluation = evaluate_groups(candidate_groups)
        except RouteRequestError as exc:
            if exc.reason in {
                "curvature_limited_corner_does_not_fit",
                "invalid_smooth_geometry",
                "curvature_infeasible",
                "smoothing_infeasible",
            }:
                continue
            raise
        selected_groups = candidate_groups
        selected_evaluation = candidate_evaluation
        break
    if selected_groups is None or selected_evaluation is None:
        raise RouteRequestError(
            "smoothing_infeasible",
            "no complete partition in the finite composite smoothing protocol is feasible",
        )
    (
        smoothing_ledger,
        replaced_polyline_nm,
        curve_length_nm,
        composite_count,
        peak_curvature,
        smooth_crossing_ledger,
    ) = selected_evaluation
    selected_pair_ranges = {
        (group_first, group_last)
        for group_first, group_last in selected_groups
        if group_last > group_first
    }
    for pair_index, item in enumerate(pair_ledger):
        item["composite"] = any(
            group_first <= pair_index < group_last
            for group_first, group_last in selected_pair_ranges
        )

    smooth_length_nm = topology_length_nm - replaced_polyline_nm + curve_length_nm
    straight_length_nm = topology_length_nm - replaced_polyline_nm
    if smooth_length_nm <= 0 or straight_length_nm < 0:
        raise RouteRequestError("verification_failed", "smoothed length ledger is invalid")
    time_ledger: list[dict[str, Any]] = []
    first_curve = smoothing_ledger[0]
    source_line_nm = int(first_curve["entry_s_nm"])
    if source_line_nm < 0:
        raise RouteRequestError("verification_failed", "first curve starts before the route")
    source_time_us, source_detail = _source_time_profile(
        graph, source_line_nm, int(first_curve["curve_length_nm"])
    )
    total_time_us = source_time_us
    time_ledger.append(
        {
            **source_detail,
            "from_s_nm": 0,
            "to_s_nm": int(first_curve["exit_s_nm"]),
            "time_us": source_time_us,
        }
    )
    for previous_curve, current_curve in zip(smoothing_ledger, smoothing_ledger[1:]):
        line_nm = int(current_curve["entry_s_nm"]) - int(previous_curve["exit_s_nm"])
        if line_nm < 0:
            raise RouteRequestError(
                "verification_failed", "neighbouring smoothing groups overlap"
            )
        interval_time_us, detail = _between_turn_time_profile(
            graph, line_nm, int(current_curve["curve_length_nm"])
        )
        total_time_us += interval_time_us
        time_ledger.append(
            {
                **detail,
                "from_s_nm": int(previous_curve["exit_s_nm"]),
                "to_s_nm": int(current_curve["exit_s_nm"]),
                "time_us": interval_time_us,
            }
        )
    finish_line_nm = topology_length_nm - int(smoothing_ledger[-1]["exit_s_nm"])
    if finish_line_nm < 0:
        raise RouteRequestError("verification_failed", "last curve ends after the route")
    finish_time_us, finish_detail = _finish_time_profile(graph, finish_line_nm)
    total_time_us += finish_time_us
    time_ledger.append(
        {
            **finish_detail,
            "from_s_nm": int(smoothing_ledger[-1]["exit_s_nm"]),
            "to_s_nm": topology_length_nm,
            "time_us": finish_time_us,
        }
    )
    ledger_smooth_nm = sum(
        int(item["curve_length_nm"]) for item in smoothing_ledger
    ) + source_line_nm + finish_line_nm + sum(
        int(current["entry_s_nm"]) - int(previous["exit_s_nm"])
        for previous, current in zip(smoothing_ledger, smoothing_ledger[1:])
    )
    if ledger_smooth_nm != smooth_length_nm:
        raise RouteRequestError(
            "verification_failed", "smoothing and time distance ledgers disagree"
        )
    return {
        "objective_time_us": total_time_us,
        "objective_time_ms": total_time_us / 1000.0,
        "objective_time_s": total_time_us / 1_000_000.0,
        "objective_length_mm": topology_length_nm / NANOMETRES_PER_MM,
        "objective_length_um": topology_length_nm // 1000,
        "objective_length_nm": topology_length_nm,
        "topology_length_mm": topology_length_nm / NANOMETRES_PER_MM,
        "topology_length_nm": topology_length_nm,
        "smooth_path_length_mm": smooth_length_nm / NANOMETRES_PER_MM,
        "smooth_path_length_nm": smooth_length_nm,
        "curve_path_length_mm": curve_length_nm / NANOMETRES_PER_MM,
        "curve_path_length_nm": curve_length_nm,
        "straight_path_length_mm": straight_length_nm / NANOMETRES_PER_MM,
        "straight_path_length_nm": straight_length_nm,
        "curve_path_ratio": curve_length_nm / smooth_length_nm,
        "curve_path_ratio_percent": 100.0 * curve_length_nm / smooth_length_nm,
        "time_model": graph.time_model_version,
        "time_geometry_mode": TIME_GEOMETRY_MODE,
        "time_ledger": time_ledger,
        "max_speed_cmps": graph.max_speed_cmps,
        "max_lateral_accel_cmps2": graph.max_lateral_accel_cmps2,
        "max_accel_cmps2": graph.max_accel_cmps2,
        "max_decel_cmps2": graph.max_decel_cmps2,
        "safety_margin_mm": graph.safety_margin_mm,
        "polyline_length_mm": topology_length_nm / NANOMETRES_PER_MM,
        "polyline_length_nm": topology_length_nm,
        "turn_count": len(turns),
        "fast_direction_change_metrics": _fast_direction_change_metrics(graph, route),
        "composite_chain_count": composite_count,
        "smoothing_ledger": [
            {key: value for key, value in item.items() if not key.startswith("_")}
            for item in smoothing_ledger
        ],
        "smooth_route_crossing_ledger": smooth_crossing_ledger,
        "turn_pair_ledger": pair_ledger,
        "smoothing_profile": graph.smoothing_profile,
        "composite_profile_version": graph.composite_profile_version,
        "composite_corridor_profile": graph.composite_corridor_profile,
        "composite_collision_profile": graph.composite_collision_profile,
        "composite_short_gap_mm": graph.swerve_gap_nm / NANOMETRES_PER_MM,
        "smoothing_ease_mm": graph.swerve_ease_mm,
        "composite_partition_policy": COMPOSITE_PARTITION_POLICY,
        "composite_partition_attempts": partition_attempts,
        "composite_partition_member_counts": [
            last - first + 1 for first, last in selected_groups
        ],
        "max_curvature_per_mm": graph.max_curvature_per_mm,
        "minimum_radius_mm": 1.0 / graph.max_curvature_per_mm,
        "peak_curvature_per_mm": peak_curvature,
        "composite_curvature_accept_ratio": COMPOSITE_CURVATURE_SAFETY_RATIO,
        "composite_expansion_steps": COMPOSITE_EXPANSION_STEPS,
        "composite_handle_ratio_min": COMPOSITE_HANDLE_RATIO_MIN,
        "composite_handle_ratio_max": COMPOSITE_HANDLE_RATIO_MAX,
        "composite_handle_ratio_step": COMPOSITE_HANDLE_RATIO_STEP,
        "composite_chain_corridor_ratio": COMPOSITE_CHAIN_CORRIDOR_RATIO,
        "required_count": graph.required_input_count,
        "start_visit_count": 2,
        "no_uturn": True,
        "geometry_mode": graph.geometry_mode,
        "curvature_constraint": "hard_maximum",
    }


def _verify_expanded_route(
    graph: PlanGraph,
    canonical_route: Sequence[str],
    expanded_route: Sequence[str],
) -> None:
    """Check that service insertion is exactly the canonical edge traversal."""

    expected = _insert_required_edge_points(graph, canonical_route)
    if list(expanded_route) != expected:
        raise RouteRequestError("verification_failed", "expanded service route is not deterministic")
    traversed = {_edge_key(left, right) for left, right in zip(canonical_route, canonical_route[1:])}
    for name in expanded_route:
        if name in graph.required_edges:
            host_edge, _ = graph.required_edges[name]
            if host_edge not in traversed:
                raise RouteRequestError("verification_failed", f"expanded route contains an off-route service {name}")

def _safe_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None

def _run_highs(
    model: SparseMilp,
    timeout_s: float,
    warm_start: Sequence[float] | None = None,
    *,
    accept_incumbent: bool = False,
    stop_after_first_incumbent: bool = False,
) -> tuple[str, list[float] | None, dict[str, Any]]:
    try:
        import highspy  # type: ignore[import-not-found]
        import numpy as np
    except ImportError as exc:
        return (
            "solver_unavailable",
            None,
            {
                "name": "HiGHS",
                "message": "highspy is not installed; run: python -m pip install -r requirements.txt",
                "detail": str(exc),
            },
        )

    highs = highspy.Highs()
    highs.setOptionValue("output_flag", False)
    highs.setOptionValue("time_limit", float(timeout_s))
    highs.setOptionValue("mip_rel_gap", 0.0)
    highs.setOptionValue("mip_abs_gap", 0.0)
    if stop_after_first_incumbent:
        highs.setOptionValue("mip_max_improving_sols", 1)
    # The model is a finite integer walk with a strict zero-gap acceptance
    # rule.  Parallel branch-and-bound improves proof throughput without
    # changing the objective or accepting an incumbent at a nonzero gap.
    highs.setOptionValue("parallel", "on")
    highs.setOptionValue("threads", HIGHS_THREADS)

    lp = highspy.HighsLp()
    lp.num_col_ = len(model.costs)
    lp.num_row_ = len(model.rows)
    lp.col_cost_ = np.asarray(model.costs, dtype=np.float64)
    lp.col_lower_ = np.asarray(model.lower, dtype=np.float64)
    lp.col_upper_ = np.asarray(model.upper, dtype=np.float64)
    lp.row_lower_ = np.asarray(
        [-highspy.kHighsInf if lower is None else lower for lower, _, _ in model.rows], dtype=np.float64
    )
    lp.row_upper_ = np.asarray(
        [highspy.kHighsInf if upper is None else upper for _, upper, _ in model.rows], dtype=np.float64
    )
    lp.integrality_ = [
        highspy.HighsVarType.kInteger if integer else highspy.HighsVarType.kContinuous
        for integer in model.integer
    ]
    starts = [0]
    indices: list[int] = []
    values: list[float] = []
    for _, _, coefficients in model.rows:
        for column, value in sorted(coefficients.items()):
            indices.append(column)
            values.append(value)
        starts.append(len(indices))
    lp.a_matrix_.format_ = highspy.MatrixFormat.kRowwise
    lp.a_matrix_.start_ = np.asarray(starts, dtype=np.int32)
    lp.a_matrix_.index_ = np.asarray(indices, dtype=np.int32)
    lp.a_matrix_.value_ = np.asarray(values, dtype=np.float64)

    pass_status = highs.passModel(lp)
    if pass_status != highspy.HighsStatus.kOk:
        return (
            "solver_error",
            None,
            {"name": "HiGHS", "message": f"passModel failed: {pass_status}"},
        )
    warm_start_status = None
    if warm_start is not None:
        if len(warm_start) != len(model.costs):
            return (
                "solver_error",
                None,
                {"name": "HiGHS", "message": "warm start width differs from model"},
            )
        warm_start_status = highs.setSolution(
            len(warm_start),
            np.arange(len(warm_start), dtype=np.int32),
            np.asarray(warm_start, dtype=np.float64),
        )
    run_status = highs.run()
    model_status = highs.getModelStatus()
    info = highs.getInfo()
    try:
        version = highs.version()
    except Exception:  # noqa: BLE001 - version reporting must not affect a proof
        version = None
    objective_units = _safe_float(getattr(info, "objective_function_value", None))
    objective_bound_units = _safe_float(getattr(info, "mip_dual_bound", None))
    objective_unit_nm = int(model.objective_unit_nm)
    solver_info = {
        "name": "HiGHS",
        "version": version,
        "parallel": "on",
        "threads": HIGHS_THREADS,
        "logical_cpu_count": os.cpu_count(),
        "run_status": str(run_status),
        "model_status": highs.modelStatusToString(model_status),
        "mip_gap": _safe_float(getattr(info, "mip_gap", None)),
        "objective_unit_nm": objective_unit_nm,
        "objective_units": objective_units,
        "objective_bound_units": objective_bound_units,
        "objective_nm": None if objective_units is None else objective_units * objective_unit_nm,
        "objective_bound_nm": (
            None if objective_bound_units is None else objective_bound_units * objective_unit_nm
        ),
        "mip_node_count": int(getattr(info, "mip_node_count", 0)),
        "solve_time_s": _safe_float(highs.getRunTime()),
        "warm_start_status": (
            None if warm_start_status is None else str(warm_start_status)
        ),
        "accept_incumbent": bool(accept_incumbent),
        "stop_after_first_incumbent": bool(stop_after_first_incumbent),
    }
    if model_status == highspy.HighsModelStatus.kInfeasible:
        return ("infeasible", None, solver_info)
    nonoptimal_incumbent = model_status != highspy.HighsModelStatus.kOptimal
    if model_status != highspy.HighsModelStatus.kOptimal:
        if not accept_incumbent:
            return ("not_optimal", None, solver_info)
        solver_info["incumbent_accepted"] = True

    solution = highs.getSolution()
    if not getattr(solution, "value_valid", True):
        return ("invalid_solution", None, solver_info)
    solution_values = [float(value) for value in solution.col_value]
    if len(solution_values) != len(model.costs):
        solver_info["objective_certificate_error"] = "solution width differs from model"
        return ("invalid_solution", None, solver_info)

    # The route counts and activation variables are integer-valued.  The
    # embedded support flow is a continuous network-flow certificate with
    # zero objective cost; for fixed integer support it has an integral
    # extreme point, so allowing it to remain continuous does not change the
    # projected route model.  Canonicalize integer variables to their lattice
    # and verify rows containing flow with a tight residual check.
    exact_float_limit = 1 << 53

    def integer_model_value(value: float, context: str) -> int:
        number = float(value)
        if (
            not math.isfinite(number)
            or not number.is_integer()
            or abs(number) > exact_float_limit
        ):
            raise ValueError(f"{context} is not an exactly represented integer")
        return int(number)

    try:
        canonical_values: list[float] = []
        max_integer_error = 0.0
        for index, (value, lower, upper) in enumerate(
            zip(solution_values, model.lower, model.upper)
        ):
            if not math.isfinite(value):
                raise ValueError(f"solution variable {index} is not finite")
            lower_int = integer_model_value(lower, f"variable {index} lower bound")
            upper_int = integer_model_value(upper, f"variable {index} upper bound")
            if model.integer[index]:
                canonical = int(round(value))
                max_integer_error = max(max_integer_error, abs(value - canonical))
                if canonical < lower_int or canonical > upper_int:
                    raise ValueError(f"solution variable {index} violates its exact bounds")
                canonical_values.append(canonical)
            else:
                bound_tolerance = 1.0e-7 * max(
                    1.0,
                    abs(float(lower_int)),
                    abs(float(upper_int)),
                )
                if value < lower_int - bound_tolerance or value > upper_int + bound_tolerance:
                    raise ValueError(
                        f"continuous variable {index} violates its bounds: "
                        f"value={value!r}, lower={lower_int}, upper={upper_int}, "
                        f"tolerance={bound_tolerance!r}"
                    )
                canonical_values.append(value)

        exact_costs = [
            integer_model_value(cost, f"objective coefficient {index}")
            for index, cost in enumerate(model.costs)
        ]
        if any(
            cost != 0 and not model.integer[index]
            for index, cost in enumerate(exact_costs)
        ):
            raise ValueError("continuous variable has a nonzero objective coefficient")
        exact_objective_units = sum(
            cost * int(canonical_values[index])
            for index, cost in enumerate(exact_costs)
            if cost != 0
        )

        for row_index, (lower, upper, coefficients) in enumerate(model.rows):
            if any(not model.integer[column] for column in coefficients):
                lhs_float = 0.0
                for column, coefficient in coefficients.items():
                    if column < 0 or column >= len(canonical_values):
                        raise ValueError(f"row {row_index} contains an invalid column")
                    lhs_float += float(coefficient) * float(canonical_values[column])
                tolerance = 1.0e-7 * max(
                    1.0,
                    abs(lhs_float),
                    0.0 if lower is None else abs(float(lower)),
                    0.0 if upper is None else abs(float(upper)),
                )
                if lower is not None and lhs_float < float(lower) - tolerance:
                    raise ValueError(f"solution violates row {row_index} lower bound")
                if upper is not None and lhs_float > float(upper) + tolerance:
                    raise ValueError(f"solution violates row {row_index} upper bound")
                continue
            lhs = 0
            for column, coefficient in coefficients.items():
                if column < 0 or column >= len(canonical_values):
                    raise ValueError(f"row {row_index} contains an invalid column")
                lhs += integer_model_value(
                    coefficient,
                    f"row {row_index} coefficient {column}",
                ) * int(canonical_values[column])
            if lower is not None and lhs < integer_model_value(lower, f"row {row_index} lower bound"):
                raise ValueError(f"solution violates row {row_index} lower bound")
            if upper is not None and lhs > integer_model_value(upper, f"row {row_index} upper bound"):
                raise ValueError(f"solution violates row {row_index} upper bound")
    except ValueError as exc:
        solver_info["objective_certificate_error"] = str(exc)
        return ("invalid_solution", None, solver_info)

    if nonoptimal_incumbent:
        exact_objective = exact_objective_units * objective_unit_nm
        solver_info["objective_certificate"] = "feasible_incumbent_only"
        solver_info["objective_units"] = exact_objective_units
        solver_info["objective_bound_units"] = objective_bound_units
        solver_info["objective_nm"] = exact_objective
        return ("feasible", [float(value) for value in canonical_values], solver_info)

    objective = solver_info["objective_units"]
    objective_bound = solver_info["objective_bound_units"]
    gap = solver_info["mip_gap"]
    if objective is None or objective_bound is None or gap is None:
        return ("nonzero_gap", None, solver_info)
    objective_lattice_quantum = 0
    for cost in exact_costs:
        if cost != 0:
            objective_lattice_quantum = math.gcd(objective_lattice_quantum, abs(cost))
    if objective_lattice_quantum == 0:
        objective_lattice_quantum = 1
    half_quantum = objective_lattice_quantum / 2.0
    if (
        abs(objective - exact_objective_units) >= half_quantum
        or abs(objective_bound - exact_objective_units) >= half_quantum
    ):
        return ("nonzero_gap", None, solver_info)

    solver_info["reported_objective_units"] = objective
    solver_info["reported_objective_bound_units"] = objective_bound
    solver_info["reported_mip_gap"] = gap
    solver_info["reported_objective_rounding_error_units"] = objective - exact_objective_units
    solver_info["reported_bound_rounding_error_units"] = objective_bound - exact_objective_units
    solver_info["objective_certificate"] = "exact_integer_incumbent_and_zero_gap_lattice_bound"
    solver_info["objective_lattice_quantum_units"] = objective_lattice_quantum
    solver_info["max_integer_error"] = max_integer_error
    solver_info["objective_units"] = exact_objective_units
    solver_info["objective_bound_units"] = exact_objective_units
    solver_info["mip_gap"] = 0.0
    solver_info["objective_nm"] = exact_objective_units * objective_unit_nm
    solver_info["objective_bound_nm"] = exact_objective_units * objective_unit_nm
    return ("optimal", [float(value) for value in canonical_values], solver_info)

def _selected_arc_counts(
    arcs: Sequence[ModelArc],
    x_variables: Sequence[int],
    values: Sequence[float],
) -> list[int]:
    if len(arcs) != len(x_variables):
        raise RouteRequestError(
            "internal_connectivity_error",
            "context arc and decision-variable widths differ",
        )
    counts: list[int] = []
    for arc, variable in zip(arcs, x_variables):
        value = values[variable]
        rounded = round(value)
        if abs(value - rounded) > 1.0e-5 or rounded < 0:
            raise RouteRequestError("invalid_integer_solution", f"arc {arc.index} is not integral")
        counts.append(rounded)
    return counts

def _disconnected_selected_components(
    arcs: Sequence[ModelArc],
    counts: Sequence[int],
) -> tuple[frozenset[int], ...]:
    """Return positive-arc weak components that are detached from SOURCE."""

    if len(arcs) != len(counts):
        raise RouteRequestError(
            "internal_connectivity_error",
            "arc-count vector width differs from the context graph",
        )

    adjacency: dict[int, set[int]] = {}
    selected_nodes: set[int] = set()
    for arc, count in zip(arcs, counts):
        if count <= 0:
            continue
        selected_nodes.add(arc.source)
        selected_nodes.add(arc.target)
        adjacency.setdefault(arc.source, set()).add(arc.target)
        adjacency.setdefault(arc.target, set()).add(arc.source)

    if SOURCE_NODE not in selected_nodes or SINK_NODE not in selected_nodes:
        raise RouteRequestError(
            "invalid_flow_solution",
            "the selected context arcs do not contain both SOURCE and SINK",
        )

    components: list[frozenset[int]] = []
    unseen = set(selected_nodes)
    while unseen:
        seed = min(unseen)
        pending = [seed]
        component: set[int] = set()
        unseen.remove(seed)
        while pending:
            node = pending.pop()
            component.add(node)
            for neighbour in adjacency.get(node, ()):
                if neighbour in unseen:
                    unseen.remove(neighbour)
                    pending.append(neighbour)
        components.append(frozenset(component))

    return tuple(
        sorted(
            (component for component in components if SOURCE_NODE not in component),
            key=lambda component: (min(component), len(component)),
        )
    )

def _add_connectivity_cuts(
    model: SparseMilp,
    arcs: Sequence[ModelArc],
    x_variables: Sequence[int],
    arc_upper_bounds: Sequence[int],
    components: Sequence[frozenset[int]],
) -> int:
    """Cut the current detached supports without forcing an unused component."""

    if not (
        len(arcs) == len(x_variables) == len(arc_upper_bounds)
    ):
        raise RouteRequestError(
            "internal_connectivity_error",
            "context arc metadata widths are inconsistent",
        )

    added = 0
    for component in components:
        internal = [
            arc.index
            for arc in arcs
            if arc.source in component and arc.target in component
        ]
        crossing = [
            arc.index
            for arc in arcs
            if (arc.source in component) != (arc.target in component)
        ]
        activation_limit = sum(arc_upper_bounds[index] for index in internal)
        if not internal or activation_limit <= 0:
            raise RouteRequestError(
                "internal_connectivity_error",
                "a detached selected component has no bounded internal arc",
            )

        # This is an exact activation cut, not an unconditional subtour cut:
        # an unused component remains legal, while any selected internal arc
        # requires at least one selected arc crossing the component boundary.
        coefficients: dict[int, float] = {
            x_variables[index]: 1.0 for index in internal
        }
        for index in crossing:
            variable = x_variables[index]
            coefficients[variable] = (
                coefficients.get(variable, 0.0) - float(activation_limit)
            )
        model.add_row(coefficients, upper=0.0)
        added += 1
    return added

def _run_highs_with_connectivity(
    model: SparseMilp,
    arcs: Sequence[ModelArc],
    x_variables: Sequence[int],
    arc_upper_bounds: Sequence[int],
    deadline_s: float,
    warm_start: Sequence[float] | None = None,
    *,
    accept_incumbent: bool = False,
    stop_after_first_incumbent: bool = False,
) -> tuple[str, list[float] | None, dict[str, Any]]:
    """Solve one MILP stage and separate every detached positive-arc support."""

    started = time.perf_counter()
    cut_rounds = 0
    cuts_added = 0
    solve_attempts = 0
    next_warm_start = warm_start
    connected_warm_start: Sequence[float] | None = None
    if warm_start is not None:
        try:
            warm_counts = _selected_arc_counts(arcs, x_variables, warm_start)
            if not _disconnected_selected_components(arcs, warm_counts):
                connected_warm_start = warm_start
        except RouteRequestError:
            connected_warm_start = None

    def annotate(
        solver_info: Mapping[str, Any],
        *,
        support_connected: bool | None,
    ) -> dict[str, Any]:
        result = dict(solver_info)
        result["last_solve_time_s"] = result.get("solve_time_s")
        result["solve_time_s"] = time.perf_counter() - started
        result["connectivity_solve_attempts"] = solve_attempts
        result["connectivity_cut_rounds"] = cut_rounds
        result["connectivity_cuts_added"] = cuts_added
        result["support_connected"] = support_connected
        return result

    while True:
        remaining = deadline_s - time.perf_counter()
        if remaining <= 0.0:
            return (
                "not_optimal",
                None,
                annotate(
                    {
                        "name": "HiGHS",
                        "model_status": "Time limit reached during exact connectivity separation",
                        "message": "the shared solve deadline expired before a connected zero-gap solution was proved",
                        "mip_gap": None,
                        "objective_unit_nm": int(model.objective_unit_nm),
                    },
                    support_connected=False if cut_rounds else None,
                ),
            )

        outcome, values, solver_info = _run_highs(
            model,
            remaining,
            next_warm_start,
            accept_incumbent=accept_incumbent,
            stop_after_first_incumbent=stop_after_first_incumbent,
        )
        solve_attempts += 1
        if outcome not in {"optimal", "feasible"} or values is None:
            return (
                outcome,
                None,
                annotate(solver_info, support_connected=None),
            )

        counts = _selected_arc_counts(arcs, x_variables, values)
        components = _disconnected_selected_components(arcs, counts)
        if not components:
            return (
                outcome,
                values,
                annotate(solver_info, support_connected=True),
            )

        added = _add_connectivity_cuts(
            model,
            arcs,
            x_variables,
            arc_upper_bounds,
            components,
        )
        if added != len(components):
            raise RouteRequestError(
                "internal_connectivity_error",
                "not every detached support component produced a cut",
            )
        cut_rounds += 1
        cuts_added += added
        # The just-returned support violates the cuts that were added for it.
        # Keep using the caller's known connected incumbent when available;
        # otherwise let HiGHS rebuild an incumbent instead of feeding it a
        # solution that is now explicitly infeasible.
        next_warm_start = connected_warm_start

def _euler_arc_order(arcs: Sequence[ModelArc], original_counts: Sequence[int]) -> list[int]:
    counts = list(original_counts)
    outgoing: dict[int, list[int]] = {}
    for arc in arcs:
        if counts[arc.index] > 0:
            outgoing.setdefault(arc.source, []).append(arc.index)
    for indices in outgoing.values():
        indices.sort(reverse=True)

    stack: list[tuple[int, int | None]] = [(SOURCE_NODE, None)]
    reverse_order: list[int] = []
    while stack:
        node = stack[-1][0]
        candidates = outgoing.get(node, [])
        while candidates and counts[candidates[-1]] <= 0:
            candidates.pop()
        if candidates:
            arc_index = candidates[-1]
            counts[arc_index] -= 1
            stack.append((arcs[arc_index].target, arc_index))
            continue
        _, incoming_arc = stack.pop()
        if incoming_arc is not None:
            reverse_order.append(incoming_arc)
    if any(counts) or len(reverse_order) != sum(original_counts):
        raise RouteRequestError("disconnected_solution", "selected arcs do not form one Euler trail")
    order = list(reversed(reverse_order))
    node = SOURCE_NODE
    for arc_index in order:
        arc = arcs[arc_index]
        if arc.source != node:
            raise RouteRequestError("broken_euler_trail", "selected arc order is discontinuous")
        node = arc.target
    if node != SINK_NODE:
        raise RouteRequestError("broken_euler_trail", "selected arc order does not end at the sink")
    return order

def _all_euler_arc_orders(
    arcs: Sequence[ModelArc],
    original_counts: Sequence[int],
    deadline_s: float,
) -> Iterator[list[int]]:
    """Enumerate every compatible Euler trail for one arc-count vector.

    Arc occurrences with the same index are indistinguishable, so branching is
    only by distinct outgoing arc index.  Completing this generator is the
    certificate needed before an arc-count vector may be excluded for geometry.
    """

    # Always expose one deterministic Hierholzer order before exploring the
    # potentially exponential branching tree.  The previous implementation
    # entered the recursive enumerator first; on the pre-race graph it could
    # spend the whole Euler slice before yielding even one order, making a
    # seedless solve look infeasible.  This first order is only a candidate,
    # not a proof that the topology is exhausted.
    try:
        first_order = _euler_arc_order(arcs, original_counts)
    except RouteRequestError:
        first_order = None
    if first_order is not None:
        yield first_order

    counts = list(original_counts)
    total = sum(counts)
    outgoing: dict[int, list[int]] = {}
    for arc in arcs:
        if counts[arc.index] > 0:
            outgoing.setdefault(arc.source, []).append(arc.index)
    for indices in outgoing.values():
        indices.sort()
    order: list[int] = []
    steps = 0

    def visit(node: int, remaining: int) -> Iterator[list[int]]:
        nonlocal steps
        steps += 1
        if (steps & 1023) == 0 and time.perf_counter() >= deadline_s:
            raise RouteRequestError(
                "euler_enumeration_timeout",
                "timeout while proving all route orders for one optimal arc-count vector",
            )
        if remaining == 0:
            if node == SINK_NODE:
                yield list(order)
            return
        if node == SINK_NODE:
            return
        for arc_index in outgoing.get(node, ()):
            if counts[arc_index] <= 0:
                continue
            counts[arc_index] -= 1
            order.append(arc_index)
            yield from visit(arcs[arc_index].target, remaining - 1)
            order.pop()
            counts[arc_index] += 1

    yield from visit(SOURCE_NODE, total)

def _route_from_arc_order(
    graph: PlanGraph,
    arcs: Sequence[ModelArc],
    order: Sequence[int],
) -> tuple[list[str], list[int]]:
    """Expand selected directed-state arcs into one complete raw route."""

    route: list[str] = []
    state_order: list[int] = []
    for arc_index in order:
        arc = arcs[arc_index]
        if not arc.chain:
            raise RouteRequestError("invalid_solution_chain", "selected state arc has an empty chain")
        if not route:
            route.extend(arc.chain)
        else:
            if route[-1] != arc.chain[0]:
                raise RouteRequestError("invalid_solution_chain", "selected state arcs are discontinuous")
            route.extend(arc.chain[1:])
        if arc.target >= 0:
            state_order.append(arc.target)
    if not route or route[0] != graph.start or route[-1] != graph.start:
        raise RouteRequestError("invalid_solution_finish", "expanded route does not return to P56")
    return route, state_order

def _exclude_arc_count_solution(
    model: SparseMilp,
    x_variables: Sequence[int],
    selected_counts: Sequence[int],
) -> None:
    """Add one exact no-good cut for a previously returned arc-count vector.

    The route MILP uses integer arc occurrence counts rather than binary edge
    variables because a closed route may traverse an edge more than once.  A
    plain binary no-good cut would therefore fail to exclude repeated-edge
    candidates.  Positive counts get ``greater``/``less`` indicators; all
    zero-count arcs share one ``any new arc`` indicator.  This is still an
    exact exclusion of one vector, but its integer-variable cost scales with
    the selected route support instead of the whole context graph.
    """

    if len(x_variables) != len(selected_counts):
        raise RouteRequestError("internal_no_good_error", "arc-count vector length mismatch")
    difference_indicators: list[int] = []
    zero_count_variables: list[int] = []
    binary_selected_variables: list[int] = []
    binary_zero_variables: list[int] = []
    for variable, raw_count in zip(x_variables, selected_counts):
        count = int(raw_count)
        lower = int(round(model.lower[variable]))
        upper = int(round(model.upper[variable]))
        if count < lower or count > upper:
            raise RouteRequestError("internal_no_good_error", "selected count is outside its model bounds")

        if count == 0:
            if lower != 0:
                raise RouteRequestError(
                    "internal_no_good_error",
                    "a zero selected count has a positive variable lower bound",
                )
            if upper > 0:
                if upper == 1:
                    binary_zero_variables.append(variable)
                else:
                    zero_count_variables.append(variable)
            continue

        # For an upper-one variable, a different value is exactly a decrease
        # from the selected support.  All such variables can share one
        # disjunct with the formerly-zero upper-one variables below.
        if count == 1 and lower == 0 and upper == 1:
            binary_selected_variables.append(variable)
            continue

        if count < upper:
            greater = model.add_variable(upper=1.0, integer=True)
            # greater=0 -> x >= lower; greater=1 -> x >= count+1.
            big_m = count + 1 - lower
            model.add_row(
                {variable: 1.0, greater: -float(big_m)},
                lower=float(lower),
            )
            difference_indicators.append(greater)

        if count > lower:
            less = model.add_variable(upper=1.0, integer=True)
            # less=0 -> x <= upper; less=1 -> x <= count-1.
            big_m = upper - count + 1
            model.add_row(
                {variable: 1.0, less: float(big_m)},
                upper=float(upper),
            )
            difference_indicators.append(less)

    if binary_selected_variables or binary_zero_variables:
        selected_drop = model.add_variable(upper=1.0, integer=True)
        zero_addition = model.add_variable(upper=1.0, integer=True)
        selected_total = len(binary_selected_variables)
        big_m = max(1, selected_total, len(binary_zero_variables)) + 1
        # selected_drop=1 requires at least one formerly-selected upper-one
        # arc to drop; zero_addition=1 requires at least one formerly-zero
        # upper-one arc to become selected.  Both indicators are optional so
        # differences in repeatable arcs can satisfy the final disjunction on
        # their own.  If all groups are unchanged, no indicator can be 1.
        model.add_row(
            {
                **{variable: 1.0 for variable in binary_selected_variables},
                selected_drop: float(big_m),
            },
            upper=float(selected_total - 1 + big_m),
        )
        model.add_row(
            {
                **{variable: 1.0 for variable in binary_zero_variables},
                zero_addition: -1.0,
            },
            lower=0.0,
        )
        difference_indicators.extend((selected_drop, zero_addition))

    if zero_count_variables:
        any_new_arc = model.add_variable(upper=1.0, integer=True)
        coefficients = {variable: 1.0 for variable in zero_count_variables}
        coefficients[any_new_arc] = -1.0
        # any_new_arc=1 requires at least one formerly-zero occurrence to be
        # selected.  When it is zero this row adds no restriction.
        model.add_row(coefficients, lower=0.0)
        difference_indicators.append(any_new_arc)

    if not difference_indicators:
        raise RouteRequestError("internal_no_good_error", "arc-count vector has no alternative value")
    model.add_row({variable: 1.0 for variable in difference_indicators}, lower=1.0)

def _not_proven(reason: str, message: str, **extra: Any) -> dict[str, Any]:
    result: dict[str, Any] = {
        "schema": RESULT_SCHEMA,
        "status": "not_proven_optimal",
        "proven_optimal": False,
        "commit_allowed": False,
        "reason": reason,
        "message": message,
    }
    result.update(extra)
    return result


def _merge_fast_speed_zones(
    zones: Sequence[Mapping[str, Any]],
    total_mm: float,
) -> list[dict[str, Any]]:
    """Merge overlapping route-progress zones without rejecting short bends."""

    ordered = sorted(
        (
            {
                "start_mm": max(0.0, min(total_mm, float(zone["start_mm"]))),
                "end_mm": max(0.0, min(total_mm, float(zone["end_mm"]))),
                "vertices": [str(name) for name in zone.get("vertices", ())],
            }
            for zone in zones
        ),
        key=lambda item: (item["start_mm"], item["end_mm"], item["vertices"]),
    )
    merged: list[dict[str, Any]] = []
    for zone in ordered:
        if zone["end_mm"] <= zone["start_mm"] + 1.0e-9:
            continue
        if merged and zone["start_mm"] <= merged[-1]["end_mm"] + 1.0e-9:
            merged[-1]["end_mm"] = max(merged[-1]["end_mm"], zone["end_mm"])
            merged[-1]["vertices"].extend(zone["vertices"])
            merged[-1]["vertices"] = list(dict.fromkeys(merged[-1]["vertices"]))
        else:
            merged.append(zone)
    return merged


def _fast_speed_ledger(
    graph: PlanGraph,
    route: Sequence[str],
    cumulative_mm: Sequence[float],
    turns: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Build the fastest piecewise-cap profile under acceleration limits.

    The topology path remains a polyline. Each true turn creates a physical
    curvature-cap zone from the configured radius/easing profile. Zones may
    overlap and are merged; the look-ahead safety margin shifts the same cap
    forward. There is no independent straight/corner speed setting.
    """

    total_mm = float(cumulative_mm[-1])
    maximum_mmps, curve_mmps, accel_mmps2, decel_mmps2 = _speed_limits_mm(graph)
    core_zones: list[dict[str, Any]] = []
    timed_zones: list[dict[str, Any]] = []
    turn_ledger: list[dict[str, Any]] = []
    timing_lead_mm = graph.safety_margin_mm
    for turn in turns:
        route_index = int(turn["route_index"])
        progress_mm = float(cumulative_mm[route_index])
        angle_rad = float(turn["angle_rad"])
        tangent_in_mm, tangent_out_mm, nominal_curve_mm = _corner_curve_metrics_mm(
            angle_rad,
            graph.swerve_radius_mm,
            graph.swerve_ease_mm,
        )
        core_start_mm = max(0.0, progress_mm - tangent_in_mm)
        core_end_mm = min(total_mm, progress_mm + tangent_out_mm)
        timed_start_mm = max(0.0, core_start_mm - timing_lead_mm)
        vertex = str(turn["vertex"])
        core_zones.append({
            "start_mm": core_start_mm,
            "end_mm": core_end_mm,
            "vertices": [vertex],
        })
        timed_zones.append({
            "start_mm": timed_start_mm,
            "end_mm": core_end_mm,
            "vertices": [vertex],
        })
        turn_ledger.append({
            "route_index": route_index,
            "vertex": vertex,
            "progress_mm": progress_mm,
            "angle_deg": math.degrees(angle_rad),
            "turn_sign": int(turn["turn_sign"]),
            "nominal_tangent_in_mm": tangent_in_mm,
            "nominal_tangent_out_mm": tangent_out_mm,
            "nominal_curve_length_mm": nominal_curve_mm,
            "core_start_mm": core_start_mm,
            "core_end_mm": core_end_mm,
            "timed_start_mm": timed_start_mm,
            "timed_end_mm": core_end_mm,
        })

    merged_core = _merge_fast_speed_zones(core_zones, total_mm)
    merged_timed = _merge_fast_speed_zones(timed_zones, total_mm)
    breakpoints = {0.0, total_mm}
    for zone in merged_timed:
        breakpoints.add(float(zone["start_mm"]))
        breakpoints.add(float(zone["end_mm"]))
    positions = sorted(breakpoints)
    intervals: list[dict[str, Any]] = []
    zone_index = 0
    for start_mm, end_mm in zip(positions, positions[1:]):
        if end_mm <= start_mm + 1.0e-9:
            continue
        midpoint = 0.5 * (start_mm + end_mm)
        while (
            zone_index < len(merged_timed)
            and merged_timed[zone_index]["end_mm"] <= midpoint + 1.0e-9
        ):
            zone_index += 1
        is_corner = (
            zone_index < len(merged_timed)
            and merged_timed[zone_index]["start_mm"] <= midpoint + 1.0e-9
            and midpoint < merged_timed[zone_index]["end_mm"] - 1.0e-9
        )
        intervals.append({
            "start_mm": start_mm,
            "end_mm": end_mm,
            "distance_mm": end_mm - start_mm,
            "constraint": "curvature" if is_corner else "maximum",
            "cap_mmps": curve_mmps if is_corner else maximum_mmps,
        })
    if not intervals:
        raise RouteRequestError("verification_failed", "fast speed ledger has no distance")

    boundary_caps: list[float] = []
    for index in range(len(intervals) + 1):
        adjacent_caps: list[float] = []
        if index > 0:
            adjacent_caps.append(float(intervals[index - 1]["cap_mmps"]))
        if index < len(intervals):
            adjacent_caps.append(float(intervals[index]["cap_mmps"]))
        boundary_caps.append(min(adjacent_caps))
    speeds = list(boundary_caps)
    speeds[0] = graph.start_speed_cmps * 10.0

    # Alternating forward/backward projections produce the maximal feasible
    # boundary-speed envelope.  This is what prevents a short straight from
    # being credited with an instantaneous jump to the 10 m/s cap.
    for _ in range(max(4, 2 * len(intervals) + 2)):
        changed = False
        speeds[0] = graph.start_speed_cmps * 10.0
        for index, interval in enumerate(intervals):
            reachable = math.sqrt(max(
                0.0,
                speeds[index] * speeds[index]
                + 2.0 * accel_mmps2 * float(interval["distance_mm"]),
            ))
            limited = min(speeds[index + 1], boundary_caps[index + 1], reachable)
            if limited + 1.0e-9 < speeds[index + 1]:
                speeds[index + 1] = limited
                changed = True
        for index in range(len(intervals) - 1, -1, -1):
            interval = intervals[index]
            reachable = math.sqrt(max(
                0.0,
                speeds[index + 1] * speeds[index + 1]
                + 2.0 * decel_mmps2 * float(interval["distance_mm"]),
            ))
            limited = min(speeds[index], boundary_caps[index], reachable)
            if limited + 1.0e-9 < speeds[index]:
                speeds[index] = limited
                changed = True
        if not changed:
            break

    total_time_us = 0
    peak_mmps = speeds[0]
    interval_ledger: list[dict[str, Any]] = []
    for index, interval in enumerate(intervals):
        timed = _high_interval_time(
            float(interval["distance_mm"]),
            speeds[index],
            speeds[index + 1],
            float(interval["cap_mmps"]),
            accel_mmps2,
            decel_mmps2,
        )
        if timed is None:
            raise RouteRequestError(
                "verification_failed",
                "fast acceleration envelope produced an infeasible interval",
            )
        seconds, interval_peak_mmps = timed
        interval_time_us = _time_us(seconds, graph.time_quantum_us)
        total_time_us += interval_time_us
        peak_mmps = max(peak_mmps, interval_peak_mmps)
        interval_ledger.append({
            "start_mm": interval["start_mm"],
            "end_mm": interval["end_mm"],
            "distance_mm": interval["distance_mm"],
            "constraint": interval["constraint"],
            "cap_cmps": float(interval["cap_mmps"]) / 10.0,
            "start_speed_cmps": speeds[index] / 10.0,
            "end_speed_cmps": speeds[index + 1] / 10.0,
            "peak_speed_cmps": interval_peak_mmps / 10.0,
            "time_us": interval_time_us,
        })

    core_corner_mm = sum(zone["end_mm"] - zone["start_mm"] for zone in merged_core)
    timed_curvature_mm = sum(
        zone["end_mm"] - zone["start_mm"] for zone in merged_timed
    )
    return {
        "objective_time_us": total_time_us,
        "objective_time_ms": total_time_us / 1000.0,
        "objective_time_s": total_time_us / 1_000_000.0,
        "peak_speed_cmps": peak_mmps / 10.0,
        "reached_max_speed_cap": peak_mmps >= maximum_mmps - 1.0e-6,
        "curve_path_length_mm": core_corner_mm,
        "curve_path_length_nm": _round_positive(core_corner_mm * NANOMETRES_PER_MM),
        "straight_path_length_mm": max(0.0, total_mm - core_corner_mm),
        "straight_path_length_nm": _round_positive(
            max(0.0, total_mm - core_corner_mm) * NANOMETRES_PER_MM
        ),
        "curve_path_ratio": core_corner_mm / total_mm if total_mm > 0.0 else 0.0,
        "curve_path_ratio_percent": (
            100.0 * core_corner_mm / total_mm if total_mm > 0.0 else 0.0
        ),
        "timed_curvature_zone_mm": timed_curvature_mm,
        "turn_speed_ledger": turn_ledger,
        "merged_turn_zones": merged_core,
        "merged_timed_speed_zones": merged_timed,
        "time_ledger": interval_ledger,
    }


def _verify_fast_route(graph: PlanGraph, route: Sequence[str]) -> dict[str, Any]:
    """Verify fast-mode topology, curvature and the complete speed ledger.

    Fast search still proposes topology routes, but the returned route is not
    usable until it passes the finite smoothing protocol. A close opposing
    pair is tried as one curvature-continuous manoeuvre first; independent
    R150 arcs remain the fallback when that curve cannot pass the hard
    curvature, progress and physical-edge checks.
    """

    if len(route) < 3 or route[0] != graph.start or route[-1] != graph.start:
        raise RouteRequestError("verification_failed", "route endpoints are invalid")
    if sum(1 for name in route if name == graph.start) != 2:
        raise RouteRequestError("verification_failed", "route crosses the start before finish")
    if graph.start_direction is not None and route[1] != graph.start_direction:
        raise RouteRequestError("verification_failed", "route violates start_dir")

    edge_lengths_nm: list[int] = []
    traversed_edges: set[tuple[str, str]] = set()
    for left, right in zip(route, route[1:]):
        if right not in graph.adjacency.get(left, ()):
            raise RouteRequestError(
                "verification_failed", f"route uses non-edge {left}-{right}"
            )
        edge = _edge_key(left, right)
        if edge in graph.avoided_edges:
            raise RouteRequestError(
                "verification_failed", f"route uses avoided edge {left}-{right}"
            )
        traversed_edges.add(edge)
        edge_lengths_nm.append(_round_positive(
            _edge_length(graph, left, right) * NANOMETRES_PER_MM
        ))
    topology_length_nm = sum(edge_lengths_nm)
    cumulative_mm = [0.0]
    for edge_nm in edge_lengths_nm:
        cumulative_mm.append(cumulative_mm[-1] + edge_nm / NANOMETRES_PER_MM)

    missing_vertices = sorted(graph.required - set(route))
    if missing_vertices:
        raise RouteRequestError(
            "verification_failed",
            f"route misses required points: {', '.join(missing_vertices)}",
        )
    missing_edge_services = sorted(
        name for name, (edge, _) in graph.required_edges.items()
        if edge not in traversed_edges
    )
    if missing_edge_services:
        raise RouteRequestError(
            "verification_failed",
            f"route misses required edge services: {', '.join(missing_edge_services)}",
        )

    # Reuse the complete geometry verifier. Same-direction ordinary corners
    # remain independent; close opposite turns prefer a verified composite.
    smooth = _verify_route(
        graph,
        route,
        require_radius_infeasible_for_composite=False,
    )
    turns: list[dict[str, Any]] = []
    for index in range(1, len(route) - 1):
        corner = _corner_profile(
            graph, route[index - 1], route[index], route[index + 1]
        )
        if corner is None or not corner.is_turn:
            continue
        turns.append({
            "route_index": index,
            "vertex": route[index],
            "angle_rad": corner.angle_rad,
            "turn_sign": int(corner.turn_sign),
        })
    speed = _fast_speed_ledger(graph, route, cumulative_mm, turns)
    metrics = _fast_direction_change_metrics(graph, route)
    return {
        **smooth,
        **speed,
        "objective_length_mm": topology_length_nm / NANOMETRES_PER_MM,
        "objective_length_um": topology_length_nm // 1000,
        "objective_length_nm": topology_length_nm,
        "topology_length_mm": topology_length_nm / NANOMETRES_PER_MM,
        "topology_length_nm": topology_length_nm,
        "smooth_path_length_mm": smooth["smooth_path_length_mm"],
        "smooth_path_length_nm": smooth["smooth_path_length_nm"],
        "polyline_length_mm": topology_length_nm / NANOMETRES_PER_MM,
        "polyline_length_nm": topology_length_nm,
        "time_model": graph.time_model_version,
        "time_geometry_mode": TIME_GEOMETRY_MODE,
        "max_speed_cmps": graph.max_speed_cmps,
        "max_lateral_accel_cmps2": graph.max_lateral_accel_cmps2,
        "max_accel_cmps2": graph.max_accel_cmps2,
        "max_decel_cmps2": graph.max_decel_cmps2,
        "safety_margin_mm": graph.safety_margin_mm,
        "start_speed_cmps": graph.start_speed_cmps,
        "turn_count": int(metrics["turn_count"]),
        "fast_direction_change_metrics": metrics,
        "required_count": graph.required_input_count,
        "start_visit_count": 2,
        "no_uturn": True,
        "geometry_mode": TOPOLOGY_GEOMETRY_MODE,
        "curvature_constraint": "hard_maximum",
        "fast_validation": "topology_curvature_and_acceleration_limited_speed_ledger",
    }


def _verify_generated_route_candidate(
    graph: PlanGraph,
    raw_route: Any,
    *,
    deadline_s: float,
    model_info: Mapping[str, Any],
    rejection: list[str] | None = None,
) -> dict[str, Any] | None:
    """Verify one route generated by the direct graph search.

    Required points that lie strictly inside a physical edge are display
    services, not optimization vertices. Remove those names for the topology
    verifier, then return the original expanded names so the editor preserves
    its visible service markers. Every remaining route pair must be an actual
    graph adjacency; no silent shortest-path repair is allowed. Fast mode
    verifies topology and the acceleration-limited speed ledger while the
    shared verifier handles smooth-curve fit, curvature and envelope checks.
    """
    def reject(reason: str) -> None:
        if rejection is not None:
            rejection.append(reason)

    if not isinstance(raw_route, list) or len(raw_route) < 3:
        reject("route_format")
        return None
    names: list[str] = []
    for raw_name in raw_route:
        if not isinstance(raw_name, str) or not raw_name:
            reject("route_name")
            return None
        resolved = graph.aliases.get(raw_name, raw_name)
        if resolved in graph.required_edges:
            continue
        if resolved not in graph.coords:
            reject(f"unknown_name:{raw_name}")
            return None
        if not names or names[-1] != resolved:
            names.append(resolved)
    if len(names) < 3 or names[0] != graph.start or names[-1] != graph.start:
        reject("route_endpoint")
        return None
    # A generated route is verified, not repaired.  The previous Dijkstra
    # repair silently replaced a bad
    # segment (and could even choose a virtual connector), making a planning
    # request appear to succeed without solving the route the editor showed.
    # Required service markers were removed above, so every remaining pair
    # must be an actual adjacent graph edge.
    for left, right in zip(names, names[1:]):
        if right not in graph.adjacency.get(left, ()):
            reject(f"route_non_adjacent:{left}->{right}")
            return None
    route_direction_reversed = False
    if graph.start_direction is not None and names[1] != graph.start_direction:
        # A closed undirected track can be traversed in either direction.  If
        # the editor has an explicit start direction and the generated route is
        # the same closed loop in the opposite direction, reverse the loop
        # before rejecting it.  This also repairs a stale direction left over
        # after switching maps; it does not relax any topology or curvature
        # constraint because the reversed loop is verified again below.
        reversed_names = [names[0], *reversed(names[1:-1]), names[-1]]
        if len(reversed_names) >= 3 and reversed_names[1] == graph.start_direction:
            names = reversed_names
            route_direction_reversed = True
        else:
            reject(f"route_start_direction:{names[1]}!={graph.start_direction}")
            return None
    for previous, current, following in zip(names, names[1:], names[2:]):
        if previous == following:
            reject(f"route_immediate_reverse:{current}")
            return None

    traversed_edges: set[tuple[str, str]] = set()
    topology_length_nm = 0
    for left, right in zip(names, names[1:]):
        if left == right:
            continue
        traversed_edges.add(_edge_key(left, right))
        topology_length_nm += _round_positive(
            _edge_length(graph, left, right) * NANOMETRES_PER_MM
        )

    missing_required_vertices = sorted(graph.required - set(names))
    if missing_required_vertices:
        reject(
            "route_missing_required_vertices:" +
            ",".join(missing_required_vertices)
        )
        return None
    missing_required_edges = sorted(
        required_name
        for required_name, (edge, _) in graph.required_edges.items()
        if edge not in traversed_edges
    )
    if missing_required_edges:
        reject(
            "route_missing_required_edges:" +
            ",".join(missing_required_edges)
        )
        return None

    try:
        verification = _verify_fast_route(graph, names)
    except (RouteRequestError, RecursionError) as exc:
        reason = getattr(exc, "reason", "route_recursion_limit")
        message = getattr(exc, "message", str(exc))
        reject(f"route_topology_or_time:{reason}:{message}")
        return None

    # Reinsert interior required markers with the same deterministic helper
    # used by the normal solver; hand-inserting them can create a display-only
    # jump when adjacent required points share one host edge.
    output_route = _insert_required_edge_points(graph, names)
    try:
        _verify_expanded_route(graph, names, output_route)
    except RouteRequestError as exc:
        reject(f"route_expansion:{exc.message}")
        return None
    start_output = graph.start_output_name
    if output_route:
        output_route[0] = start_output
        output_route[-1] = start_output
    solver_info = {
        "name": "direct_search_route",
        "verification_basis": "direct_graph_search_topology_and_speed_ledger_verified",
        "global_optimality_proven": False,
        "incumbent_accepted": True,
        "objective_kind": "feasible_direct_search_route",
        "model_status": "direct_search_route",
        "route_direction_reversed": route_direction_reversed,
    }
    return {
        "schema": RESULT_SCHEMA,
        "status": "feasible",
        "proven_optimal": False,
        "commit_allowed": True,
        "approximate": True,
        "candidate_rank": 1,
        "route": output_route,
        "objective_length_mm": verification["objective_length_mm"],
        "objective_length_um": verification["objective_length_um"],
        "objective_length_nm": verification["objective_length_nm"],
        "objective_time_us": verification["objective_time_us"],
        "objective_time_ms": verification["objective_time_ms"],
        "objective_time_s": verification["objective_time_s"],
        "smooth_path_length_mm": verification["smooth_path_length_mm"],
        "smooth_path_length_nm": verification["smooth_path_length_nm"],
        "objective_model": "direct_graph_search_route",
        "smoothing_profile": graph.smoothing_profile,
        "composite_profile_version": graph.composite_profile_version,
        "composite_corridor_profile": graph.composite_corridor_profile,
        "composite_collision_profile": graph.composite_collision_profile,
        "composite_partition_policy": COMPOSITE_PARTITION_POLICY,
        "max_curvature_per_mm": graph.max_curvature_per_mm,
        "minimum_radius_mm": 1.0 / graph.max_curvature_per_mm,
        "composite_short_gap_mm": graph.swerve_gap_nm / NANOMETRES_PER_MM,
        "smoothing_ease_mm": graph.swerve_ease_mm,
        "time_model": graph.time_model_version,
        "time_geometry_mode": TIME_GEOMETRY_MODE,
        "time_quantum_us": graph.time_quantum_us,
        "control_period_us": CONTROL_PERIOD_US,
        "geometry_mode": graph.geometry_mode,
        "start_direction": names[1],
        "arrival_from": names[-2],
        "solver": solver_info,
        "model": {**model_info, "solver_mode": "verified_direct_search_route"},
        "verification": {
            **verification,
            "direct_search_basic_topology_checked": True,
            "topology_length_nm": topology_length_nm,
            "direct_search_speed_ledger_verified_by_backend": True,
            "route_direction_reversed": route_direction_reversed,
        },
        "candidate_generation": {
            "requested": 1,
            "returned": 1,
            "complete": True,
            "method": "verified_direct_graph_route",
            "global_optimality_proven": False,
            "stop_reason": "direct_route_verified",
            "solve_time_s": 0.0,
        },
        "candidates": [],
    }


def _fast_candidate_sort_key(candidate: Mapping[str, Any]) -> tuple[Any, ...]:
    """Rank equal-length candidates by reversals first, then predicted time."""
    verification = candidate.get("verification") or {}
    metrics = verification.get("fast_direction_change_metrics") or {}
    route = candidate.get("route") or ()
    return (
        int(metrics.get("direction_change_count", 1 << 30)),
        int(candidate.get("objective_time_us", 1 << 62)),
        int(metrics.get("max_direction_changes", 1 << 30)),
        int(metrics.get("max_alternating_turns", 1 << 30)),
        int(metrics.get("max_true_turns", 1 << 30)),
        int(metrics.get("turn_count", 1 << 30)),
        int(candidate.get("objective_length_nm", 1 << 62)),
        -int(metrics.get("longest_straight_nm", 0)),
        tuple(str(name) for name in route),
    )


def _fast_ranked_candidate_pool(
    candidates: Sequence[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Rank every independently verified candidate by the lap-time objective."""
    return sorted(candidates, key=_fast_candidate_sort_key)


def _public_candidate_view(
    candidate: Mapping[str, Any],
    *,
    rank: int,
    selected: bool,
) -> dict[str, Any]:
    """Return one complete-but-compact candidate for the editor.

    The internal candidate contains MILP arrays and solver diagnostics that
    are useful for debugging but would multiply the response size when several
    alternatives are returned.  Keep the route, the full independent
    verification ledger, and the ranking metrics; omit internal model state.
    """
    verification = dict(candidate.get("verification") or {})
    metrics = dict(
        verification.get("fast_direction_change_metrics")
        or candidate.get("fast_direction_change_metrics")
        or {}
    )
    return {
        "schema": RESULT_SCHEMA,
        "status": "feasible",
        "proven_optimal": False,
        "commit_allowed": True,
        "approximate": True,
        "candidate_rank": int(rank),
        "selected": bool(selected),
        "route": list(candidate.get("route") or []),
        "objective_length_mm": candidate.get("objective_length_mm"),
        "objective_length_um": candidate.get("objective_length_um"),
        "objective_length_nm": candidate.get("objective_length_nm"),
        "objective_time_us": candidate.get("objective_time_us"),
        "objective_time_ms": candidate.get("objective_time_ms"),
        "objective_time_s": candidate.get("objective_time_s"),
        "smooth_path_length_mm": candidate.get("smooth_path_length_mm"),
        "smooth_path_length_nm": candidate.get("smooth_path_length_nm"),
        "objective_model": candidate.get("objective_model"),
        "smoothing_profile": candidate.get("smoothing_profile"),
        "composite_profile_version": candidate.get("composite_profile_version"),
        "composite_corridor_profile": candidate.get("composite_corridor_profile"),
        "composite_collision_profile": candidate.get("composite_collision_profile"),
        "composite_partition_policy": candidate.get("composite_partition_policy"),
        "max_curvature_per_mm": candidate.get("max_curvature_per_mm"),
        "minimum_radius_mm": candidate.get("minimum_radius_mm"),
        "composite_short_gap_mm": candidate.get("composite_short_gap_mm"),
        "smoothing_ease_mm": candidate.get("smoothing_ease_mm"),
        "time_model": candidate.get("time_model"),
        "time_geometry_mode": candidate.get("time_geometry_mode"),
        "time_quantum_us": candidate.get("time_quantum_us"),
        "control_period_us": candidate.get("control_period_us"),
        "geometry_mode": candidate.get("geometry_mode"),
        "start_direction": candidate.get("start_direction"),
        "arrival_from": candidate.get("arrival_from"),
        "verification": verification,
        "fast_direction_change_metrics": metrics,
    }


def _solve_directed_edge_milp_fast(
    graph: PlanGraph,
    *,
    deadline_s: float,
    started_s: float,
    report,
) -> tuple[dict[str, Any] | None, str | None]:
    """Enumerate feasible MILP topologies and rank the shortest length class.

    This is deliberately an incumbent-only fast mode.  HiGHS chooses a
    directed-edge occurrence vector, then the finite Euler enumerator expands
    that vector into concrete walks.  A topology is excluded only after every
    Euler order tried for it has failed the current v12 verifier; the no-good
    cut then asks HiGHS for the next topology.  No global optimality claim is
    made and the existing beam planner remains the safe fallback.
    """

    model_info: dict[str, Any] = {
        "solver_mode": "directed_edge_milp_euler_fast",
        "milp_objective": "topology_length_plus_turn_penalty_nm",
        "milp_global_optimality_proven": False,
        "max_topology_attempts": FAST_MILP_MAX_TOPOLOGY_ATTEMPTS,
        "max_euler_orders_per_topology": FAST_MILP_MAX_EULER_ORDERS_PER_TOPOLOGY,
        "euler_order_slice_s": FAST_MILP_ORDER_SLICE_S,
    }
    try:
        report("milp_building_model")
        states, arcs, model, metadata = _build_directed_edge_model(graph)
    except RouteRequestError as exc:
        return None, f"milp_model:{exc.reason}:{exc.message}"
    except Exception as exc:  # noqa: BLE001 - beam fallback remains available
        return None, f"milp_model_error:{exc}"

    model_info.update({
        "vertices": len(graph.adjacency),
        "edge_segments": len(graph.edge_lengths),
        "directed_states": len(states),
        "model_arcs": len(arcs),
        **metadata,
    })
    topology_attempts = 0
    euler_orders_checked = 0
    topology_rejections: list[str] = []
    feasible_candidates: list[dict[str, Any]] = []
    seen_routes: set[tuple[str, ...]] = set()
    last_solver_info: dict[str, Any] = {"name": "HiGHS"}
    last_failure = "milp_no_feasible_topology"

    while (
        topology_attempts < FAST_MILP_MAX_TOPOLOGY_ATTEMPTS
        and time.perf_counter() < deadline_s
    ):
        topology_attempts += 1
        remaining = deadline_s - time.perf_counter()
        if remaining <= 0.0:
            break
        report(
            "milp_solving_topology",
            topology_attempt=topology_attempts,
            euler_orders_checked=euler_orders_checked,
        )
        try:
            solve_slice_s = min(
                remaining,
                FAST_MILP_FIRST_TOPOLOGY_SLICE_S
                if topology_attempts == 1
                else FAST_MILP_NEXT_TOPOLOGY_SLICE_S,
            )
            outcome, values, solver_info = _run_highs_with_connectivity(
                model,
                arcs,
                metadata["x_variables"],
                metadata["arc_upper_bounds"],
                time.perf_counter() + solve_slice_s,
                accept_incumbent=True,
                # Do not stop at the first feasible incumbent.  A first
                # incumbent can be hundreds of metres longer than the true
                # shortest topology; the direct planner must let HiGHS
                # optimize the integer edge-count objective before Euler
                # ordering and geometric verification begin.
                stop_after_first_incumbent=False,
            )
        except RouteRequestError as exc:
            return None, f"milp_connectivity:{exc.reason}:{exc.message}"
        last_solver_info = dict(solver_info)
        if outcome not in {"optimal", "feasible"} or values is None:
            last_failure = f"milp_solver:{outcome}"
            break

        try:
            counts = _selected_arc_counts(arcs, metadata["x_variables"], values)
        except RouteRequestError as exc:
            return None, f"milp_counts:{exc.reason}:{exc.message}"

        order_deadline = min(
            deadline_s,
            time.perf_counter() + FAST_MILP_ORDER_SLICE_S,
        )
        local_seen: set[tuple[str, ...]] = set()
        accepted_candidates: list[dict[str, Any]] = []
        topology_rejection: str | None = None
        topology_complete = True
        report(
            "milp_enumerating_euler",
            topology_attempt=topology_attempts,
            euler_orders_checked=euler_orders_checked,
        )
        try:
            for order in _all_euler_arc_orders(arcs, counts, order_deadline):
                canonical_route, _ = _route_from_arc_order(graph, arcs, order)
                key = tuple(canonical_route)
                if key in local_seen:
                    continue
                if len(local_seen) >= FAST_MILP_MAX_EULER_ORDERS_PER_TOPOLOGY:
                    break
                local_seen.add(key)
                euler_orders_checked += 1
                rejection: list[str] = []
                candidate = _verify_generated_route_candidate(
                    graph,
                    canonical_route,
                    deadline_s=deadline_s,
                    model_info=model_info,
                    rejection=rejection,
                )
                if candidate is not None:
                    if key not in seen_routes:
                        metrics = dict(
                            candidate.get("verification", {}).get(
                                "fast_direction_change_metrics"
                            )
                            or {}
                        )
                        candidate["objective_model"] = (
                            "directed_edge_milp_then_euler"
                        )
                        candidate["fast_direction_change_metrics"] = metrics
                        candidate["solver"] = {
                            "name": "HiGHS_directed_edge_milp",
                            "verification_basis": (
                                "directed_edge_topology_and_bounded_euler_order"
                                "_with_current_v12_verifier"
                            ),
                            "global_optimality_proven": False,
                            "incumbent_accepted": True,
                            "topology_attempts": topology_attempts,
                            "euler_orders_checked": euler_orders_checked,
                            "milp_outcome": outcome,
                            "topology_solver": solver_info,
                        }
                        candidate["model"] = {
                            **model_info,
                            "topology_attempts": topology_attempts,
                            "euler_orders_checked": euler_orders_checked,
                            "solver_info": solver_info,
                        }
                        candidate["verification"] = {
                            **dict(candidate.get("verification") or {}),
                            "directed_edge_milp_checked": True,
                            "bounded_euler_order_checked": True,
                            "milp_topology_attempt": topology_attempts,
                            "milp_euler_order_count": euler_orders_checked,
                        }
                        accepted_candidates.append(candidate)
                        seen_routes.add(key)
                        report(
                            "milp_candidate_accepted",
                            topology_attempt=topology_attempts,
                            euler_orders_checked=euler_orders_checked,
                            feasible_candidates=(
                                len(feasible_candidates) +
                                len(accepted_candidates)
                            ),
                            route_length_mm=candidate.get("objective_length_mm"),
                            objective_time_s=candidate.get("objective_time_s"),
                            direction_change_count=metrics.get(
                                "direction_change_count"
                            ),
                            max_direction_changes=metrics.get(
                                "max_direction_changes"
                            ),
                        )
                    if (
                        len(feasible_candidates) + len(accepted_candidates)
                        >= FAST_MAX_FEASIBLE_CANDIDATES
                    ):
                        break
                if rejection:
                    topology_rejection = rejection[0]
        except RouteRequestError as exc:
            topology_rejection = f"{exc.reason}:{exc.message}"
            topology_complete = False

        feasible_candidates.extend(accepted_candidates)
        if not topology_complete:
            topology_rejections.append(
                topology_rejection or "euler_enumeration_budget_exhausted"
            )
            # The no-good cut is only sound after every Euler order for this
            # arc-count vector has been exhausted.  If the bounded enumerator
            # timed out, preserve the candidates already found and stop the
            # MILP phase instead of excluding an incompletely inspected vector.
            break
        if topology_rejection:
            topology_rejections.append(topology_rejection)

        if len(feasible_candidates) >= FAST_MAX_FEASIBLE_CANDIDATES:
            break
        if time.perf_counter() >= deadline_s:
            break
        try:
            _exclude_arc_count_solution(
                model,
                metadata["x_variables"],
                counts,
            )
        except RouteRequestError as exc:
            return None, f"milp_no_good:{exc.reason}:{exc.message}"
        report(
            "milp_topology_rejected",
            topology_attempt=topology_attempts,
            reason=topology_rejection or "no_verified_euler_order",
            euler_orders_checked=euler_orders_checked,
        )

    if not feasible_candidates:
        if topology_rejections:
            last_failure = "milp_no_feasible_topology:" + topology_rejections[0]
        return None, last_failure

    # Do not trade a materially longer lap for fewer reversals.  First keep
    # only the shortest topology-length class found by the MILP, then apply
    # the requested steering-reversal and predicted-time ranking inside it.
    shortest_length_nm = min(
        int(candidate.get("objective_length_nm", 1 << 62))
        for candidate in feasible_candidates
    )
    shortest_candidates = [
        candidate
        for candidate in feasible_candidates
        if int(candidate.get("objective_length_nm", 1 << 62)) == shortest_length_nm
    ]
    ranked = _fast_ranked_candidate_pool(shortest_candidates)
    # Keep alternatives from other physical-length classes too.  The first
    # item remains the selected shortest-length candidate, while the editor
    # can now inspect and apply the other independently verified routes.
    presented_candidates = sorted(
        feasible_candidates,
        key=lambda candidate: (
            int(candidate.get("objective_length_nm", 1 << 62)),
            _fast_candidate_sort_key(candidate),
        ),
    )
    selected = dict(ranked[0])
    selected_metrics = dict(
        (selected.get("verification") or {}).get(
            "fast_direction_change_metrics"
        )
        or {}
    )
    selected["candidate_rank"] = 1
    selected["objective_model"] = (
        "shortest_topology_length_then_direction_changes_then_predicted_time"
    )
    selected["fast_direction_change_metrics"] = selected_metrics
    selected["solver"] = {
        **dict(selected.get("solver") or {}),
        "name": "directed_edge_milp_euler_candidate_pool",
        "candidate_count_examined": len(feasible_candidates),
        "milp_candidate_count": len(feasible_candidates),
        "shortest_length_class_candidate_count": len(shortest_candidates),
        "global_optimality_proven": False,
    }
    selected["model"] = {
        **dict(selected.get("model") or {}),
        "topology_attempts": topology_attempts,
        "euler_orders_checked": euler_orders_checked,
        "feasible_candidates": len(feasible_candidates),
        "shortest_length_class_candidates": len(shortest_candidates),
    }
    stop_reason = (
        "candidate_budget_reached"
        if len(feasible_candidates) >= FAST_MAX_FEASIBLE_CANDIDATES
        else "time_budget_or_milp_exhausted"
    )
    selected["candidate_generation"] = {
        "requested": FAST_MAX_FEASIBLE_CANDIDATES,
        "returned": len(presented_candidates),
        "examined": len(feasible_candidates),
        "complete": False,
        "method": (
            "directed_edge_milp_bounded_euler_ranked_by_direction_changes_and_time"
        ),
        "global_optimality_proven": False,
        "stop_reason": stop_reason,
        "topology_attempts": topology_attempts,
        "euler_orders_checked": euler_orders_checked,
        "shortest_length_class_candidates": len(shortest_candidates),
        "shortest_topology_length_mm": shortest_length_nm / NANOMETRES_PER_MM,
        "solve_time_s": time.perf_counter() - started_s,
        "direction_change_window_mm": FAST_DIRECTION_WINDOW_MM,
    }
    selected["candidates"] = [
        _public_candidate_view(
            candidate,
            rank=rank,
            selected=candidate is ranked[0],
        )
        for rank, candidate in enumerate(presented_candidates, 1)
    ]
    report(
        "milp_complete_feasible",
        topology_attempts=topology_attempts,
        euler_orders_checked=euler_orders_checked,
        feasible_candidates=len(feasible_candidates),
        shortest_length_class_candidates=len(shortest_candidates),
        route_length_mm=selected.get("objective_length_mm"),
        objective_time_s=selected.get("objective_time_s"),
        direction_change_count=selected_metrics.get("direction_change_count"),
        max_direction_changes=selected_metrics.get("max_direction_changes"),
    )
    return selected, None


def _fast_path_distance_nm(graph: PlanGraph, path: Sequence[str]) -> int:
    return sum(
        _round_positive(_edge_length(graph, left, right) * NANOMETRES_PER_MM)
        for left, right in zip(path, path[1:])
    )


def _fast_nominal_time_us(
    graph: PlanGraph,
    distance_nm: int,
    nominal_curve_nm: int,
) -> int:
    """Cheap beam-search time estimate; final ranking uses the full ledger."""

    low_nm = min(max(0, nominal_curve_nm), max(0, distance_nm))
    high_nm = max(0, distance_nm - low_nm)
    maximum_mmps, curve_mmps, _, _ = _speed_limits_mm(graph)
    seconds = (
        high_nm / NANOMETRES_PER_MM / maximum_mmps
        + low_nm / NANOMETRES_PER_MM / curve_mmps
    )
    return _time_us(seconds, graph.time_quantum_us)


def _fast_nominal_time_key(
    graph: PlanGraph,
    route: Sequence[str],
) -> tuple[Any, ...]:
    """Choose geometry-check order with a finite steering-reversal cost."""
    curve_nm = 0
    for index in range(1, len(route) - 1):
        corner = _corner_profile(
            graph, route[index - 1], route[index], route[index + 1]
        )
        if corner is None or not corner.is_turn:
            continue
        _, _, curve_mm = _corner_curve_metrics_mm(
            corner.angle_rad,
            graph.swerve_radius_mm,
            graph.swerve_ease_mm,
        )
        curve_nm += _round_positive(curve_mm * NANOMETRES_PER_MM)
    distance_nm = _fast_path_distance_nm(graph, route)
    metrics = _fast_direction_change_metrics(graph, route)
    direction_changes = int(metrics["direction_change_count"])
    search_cost_nm = distance_nm + direction_changes * _round_positive(
        FAST_DIRECTION_CHANGE_EQUIVALENT_MM * NANOMETRES_PER_MM
    )
    return (
        search_cost_nm,
        direction_changes,
        int(metrics["max_direction_changes"]),
        int(metrics["max_alternating_turns"]),
        _fast_nominal_time_us(graph, distance_nm, curve_nm),
        distance_nm,
        int(metrics["max_true_turns"]),
        int(metrics["turn_count"]),
        -int(metrics["longest_straight_nm"]),
        route,
    )


def _fast_dijkstra_path(
    graph: PlanGraph,
    source: str,
    target: str,
    *,
    banned_nodes: frozenset[str] = frozenset(),
    banned_arcs: frozenset[tuple[str, str]] = frozenset(),
) -> tuple[str, ...] | None:
    if source == target:
        return (source,)
    distances: dict[str, float] = {source: 0.0}
    previous: dict[str, str] = {}
    queue: list[tuple[float, str]] = [(0.0, source)]
    while queue:
        distance, node = heapq.heappop(queue)
        if distance > distances.get(node, float("inf")) + 1.0e-9:
            continue
        if node == target:
            break
        for neighbour in graph.adjacency.get(node, ()):
            if neighbour in banned_nodes or (node, neighbour) in banned_arcs:
                continue
            next_distance = distance + _edge_length(graph, node, neighbour)
            previous_best = distances.get(neighbour, float("inf"))
            if next_distance + 1.0e-9 < previous_best:
                distances[neighbour] = next_distance
                previous[neighbour] = node
                heapq.heappush(queue, (next_distance, neighbour))
    if target not in distances:
        return None
    path = [target]
    while path[-1] != source:
        path.append(previous[path[-1]])
    path.reverse()
    return tuple(path)


def _fast_k_shortest_paths(
    graph: PlanGraph,
    source: str,
    target: str,
    count: int,
) -> tuple[tuple[str, ...], ...]:
    """Return up to ``count`` loopless paths with Yen's algorithm."""
    first = _fast_dijkstra_path(graph, source, target)
    if first is None:
        return ()
    accepted = [first]
    accepted_set = {first}
    candidates: list[tuple[int, tuple[str, ...]]] = []
    candidate_set: set[tuple[str, ...]] = set()
    while len(accepted) < count:
        previous_path = accepted[-1]
        for spur_index in range(len(previous_path) - 1):
            root = previous_path[: spur_index + 1]
            banned_arcs = {
                (candidate[spur_index], candidate[spur_index + 1])
                for candidate in accepted
                if len(candidate) > spur_index
                and candidate[: spur_index + 1] == root
            }
            spur = _fast_dijkstra_path(
                graph,
                root[-1],
                target,
                banned_nodes=frozenset(root[:-1]),
                banned_arcs=frozenset(banned_arcs),
            )
            if spur is None:
                continue
            candidate = (*root[:-1], *spur)
            if candidate in accepted_set or candidate in candidate_set:
                continue
            candidate_set.add(candidate)
            heapq.heappush(
                candidates,
                (_fast_path_distance_nm(graph, candidate), candidate),
            )
        if not candidates:
            break
        _, selected = heapq.heappop(candidates)
        candidate_set.discard(selected)
        accepted.append(selected)
        accepted_set.add(selected)
    return tuple(accepted)


def _fast_extend_state(
    graph: PlanGraph,
    state: _FastBeamState,
    suffix: Sequence[str],
    vertex_bits: Mapping[str, int],
    edge_bits: Mapping[tuple[str, str], int],
    *,
    allow_final_start: bool,
) -> _FastBeamState | None:
    if not suffix or suffix[0] != state.route[-1]:
        return None
    appended = tuple(suffix[1:])
    if not appended:
        return state
    if (
        len(state.route) == 1
        and graph.start_direction is not None
        and appended[0] != graph.start_direction
    ):
        return None
    if len(state.route) >= 2 and appended[0] == state.route[-2]:
        return None
    if graph.start in appended[:-1] or (
        graph.start in appended and not allow_final_start
    ):
        return None
    combined = (*state.route, *appended)
    if any(
        previous == following
        for previous, _, following in zip(combined, combined[1:], combined[2:])
    ):
        return None

    covered_vertices = state.covered_vertices
    covered_edges = state.covered_edges
    for name in appended:
        bit = vertex_bits.get(name)
        if bit is not None:
            covered_vertices |= 1 << bit
    for left, right in zip(suffix, suffix[1:]):
        bit = edge_bits.get(_edge_key(left, right))
        if bit is not None:
            covered_edges |= 1 << bit

    new_distance_nm = state.distance_nm
    cumulative_by_index = [state.distance_nm]
    last_direction = state.last_direction
    current_straight_nm = state.current_straight_nm
    longest_straight_nm = state.longest_straight_nm
    straight_concentration_nm2 = state.straight_concentration_nm2
    straight_run_count = state.straight_run_count
    for left, right in zip(suffix, suffix[1:]):
        start = graph.coords[left]
        end = graph.coords[right]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        edge_nm = _round_positive(math.hypot(dx, dy) * NANOMETRES_PER_MM)
        new_distance_nm += edge_nm
        cumulative_by_index.append(new_distance_nm)
        same_heading = False
        if last_direction is not None:
            previous_dx, previous_dy = last_direction
            cross = previous_dx * dy - previous_dy * dx
            dot = previous_dx * dx + previous_dy * dy
            tolerance = DIRECTION_EPS * max(
                1.0,
                abs(previous_dx * dy),
                abs(previous_dy * dx),
            )
            same_heading = abs(cross) <= tolerance and dot > 0.0
        if same_heading:
            straight_concentration_nm2 -= current_straight_nm * current_straight_nm
            current_straight_nm += edge_nm
            straight_concentration_nm2 += current_straight_nm * current_straight_nm
        else:
            current_straight_nm = edge_nm
            straight_run_count += 1
            straight_concentration_nm2 += edge_nm * edge_nm
        longest_straight_nm = max(longest_straight_nm, current_straight_nm)
        last_direction = (dx, dy)

    turns = list(state.turns)
    direction_change_count = state.direction_change_count
    max_changes = state.max_direction_changes
    max_alternating = state.max_alternating_turns
    max_true_turns = state.max_true_turns
    nominal_curve_nm = state.nominal_curve_nm
    first_new_corner = len(state.route) - 1
    for route_index in range(first_new_corner, len(combined) - 1):
        corner = _corner_profile(
            graph,
            combined[route_index - 1],
            combined[route_index],
            combined[route_index + 1],
        )
        if corner is None or not corner.is_turn:
            continue
        _, _, nominal_curve_mm = _corner_curve_metrics_mm(
            corner.angle_rad,
            graph.swerve_radius_mm,
            graph.swerve_ease_mm,
        )
        nominal_curve_nm += _round_positive(
            nominal_curve_mm * NANOMETRES_PER_MM
        )
        suffix_offset = route_index - (len(state.route) - 1)
        turn_s_nm = cumulative_by_index[suffix_offset]
        if turns and turns[-1][1] != int(corner.turn_sign):
            direction_change_count += 1
        turns.append((turn_s_nm, int(corner.turn_sign)))
        window_start_nm = turn_s_nm - _round_positive(
            FAST_DIRECTION_WINDOW_MM * NANOMETRES_PER_MM
        )
        first = 0
        while first < len(turns) and turns[first][0] < window_start_nm:
            first += 1
        window_turns = turns[first:]
        max_true_turns = max(max_true_turns, len(window_turns))
        changes = sum(
            left_sign != right_sign
            for (_, left_sign), (_, right_sign) in zip(
                window_turns, window_turns[1:]
            )
        )
        max_changes = max(max_changes, changes)
        alternating = 1
        for previous_turn, current_turn in zip(
            reversed(window_turns[:-1]), reversed(window_turns[1:])
        ):
            if previous_turn[1] == current_turn[1]:
                break
            alternating += 1
        max_alternating = max(max_alternating, alternating)

    return _FastBeamState(
        route=combined,
        covered_vertices=covered_vertices,
        covered_edges=covered_edges,
        distance_nm=new_distance_nm,
        turns=tuple(turns),
        direction_change_count=direction_change_count,
        max_direction_changes=max_changes,
        max_alternating_turns=max_alternating,
        max_true_turns=max_true_turns,
        last_direction=last_direction,
        current_straight_nm=current_straight_nm,
        longest_straight_nm=longest_straight_nm,
        straight_concentration_nm2=straight_concentration_nm2,
        straight_run_count=straight_run_count,
        nominal_curve_nm=nominal_curve_nm,
    )


def _fast_state_rank(
    graph: PlanGraph,
    state: _FastBeamState,
    all_vertex_mask: int,
    all_edge_mask: int,
) -> tuple[Any, ...]:
    remaining = (
        (all_vertex_mask & ~state.covered_vertices).bit_count()
        + (all_edge_mask & ~state.covered_edges).bit_count()
    )
    search_cost_nm = state.distance_nm + state.direction_change_count * _round_positive(
        FAST_DIRECTION_CHANGE_EQUIVALENT_MM * NANOMETRES_PER_MM
    )
    return (
        remaining,
        search_cost_nm,
        state.direction_change_count,
        state.max_direction_changes,
        state.max_alternating_turns,
        state.max_true_turns,
        _fast_nominal_time_us(graph, state.distance_nm, state.nominal_curve_nm),
        state.distance_nm,
        -state.longest_straight_nm,
        state.route,
    )


def _fast_state_direction_rank(
    graph: PlanGraph,
    state: _FastBeamState,
    all_vertex_mask: int,
    all_edge_mask: int,
) -> tuple[Any, ...]:
    """Keep a second Beam frontier that does not discard smooth service order.

    The normal frontier must still prefer states that finish required services
    quickly.  That ordering alone, however, deletes an equal-length branch as
    soon as it temporarily covers one fewer service.  This complementary rank
    keeps the best steering-continuity states alive until both branches can be
    compared as complete routes.
    """
    remaining = (
        (all_vertex_mask & ~state.covered_vertices).bit_count()
        + (all_edge_mask & ~state.covered_edges).bit_count()
    )
    search_cost_nm = state.distance_nm + state.direction_change_count * _round_positive(
        FAST_DIRECTION_CHANGE_EQUIVALENT_MM * NANOMETRES_PER_MM
    )
    return (
        state.direction_change_count,
        state.max_direction_changes,
        state.max_alternating_turns,
        search_cost_nm,
        remaining,
        state.distance_nm,
        -state.longest_straight_nm,
        state.route,
    )


def _fast_select_beam_states(
    graph: PlanGraph,
    states: Sequence[_FastBeamState],
    all_vertex_mask: int,
    all_edge_mask: int,
) -> list[_FastBeamState]:
    """Merge coverage-first and steering-continuity Pareto frontiers."""
    if len(states) <= FAST_BEAM_WIDTH:
        return sorted(
            states,
            key=lambda item: _fast_state_rank(
                graph, item, all_vertex_mask, all_edge_mask
            ),
        )
    coverage_quota = FAST_BEAM_WIDTH // 2
    direction_quota = FAST_BEAM_WIDTH - coverage_quota
    coverage = sorted(
        states,
        key=lambda item: _fast_state_rank(
            graph, item, all_vertex_mask, all_edge_mask
        ),
    )
    direction = sorted(
        states,
        key=lambda item: _fast_state_direction_rank(
            graph, item, all_vertex_mask, all_edge_mask
        ),
    )
    selected: list[_FastBeamState] = []
    selected_routes: set[tuple[str, ...]] = set()

    def append_from(ordered: Sequence[_FastBeamState], quota: int) -> None:
        added = 0
        for item in ordered:
            if item.route in selected_routes:
                continue
            selected.append(item)
            selected_routes.add(item.route)
            added += 1
            if added >= quota or len(selected) >= FAST_BEAM_WIDTH:
                return

    append_from(coverage, coverage_quota)
    append_from(direction, direction_quota)
    if len(selected) < FAST_BEAM_WIDTH:
        append_from(coverage, FAST_BEAM_WIDTH - len(selected))
    return selected


def _fast_route_has_all_tasks(
    route: Sequence[str],
    vertex_bits: Mapping[str, int],
    edge_bits: Mapping[tuple[str, str], int],
    all_vertex_mask: int,
    all_edge_mask: int,
) -> bool:
    vertex_mask = 0
    edge_mask = 0
    for name in route:
        bit = vertex_bits.get(name)
        if bit is not None:
            vertex_mask |= 1 << bit
    for left, right in zip(route, route[1:]):
        bit = edge_bits.get(_edge_key(left, right))
        if bit is not None:
            edge_mask |= 1 << bit
    return vertex_mask == all_vertex_mask and edge_mask == all_edge_mask


def _fast_two_opt_proposals(
    graph: PlanGraph,
    route: Sequence[str],
    vertex_bits: Mapping[str, int],
    edge_bits: Mapping[tuple[str, str], int],
    all_vertex_mask: int,
    all_edge_mask: int,
) -> Iterator[tuple[str, ...]]:
    """Generate graph-valid 2-opt and K-path segment replacements for LNS."""
    route_tuple = tuple(route)
    proposals: dict[tuple[str, ...], tuple[Any, ...]] = {}
    path_cache: dict[tuple[str, str], tuple[tuple[str, ...], ...]] = {}

    def consider(candidate: tuple[str, ...]) -> None:
        if candidate == route_tuple or graph.start in candidate[1:-1]:
            return
        if any(
            previous == following
            for previous, _, following in zip(
                candidate, candidate[1:], candidate[2:]
            )
        ):
            return
        if not _fast_route_has_all_tasks(
            candidate,
            vertex_bits,
            edge_bits,
            all_vertex_mask,
            all_edge_mask,
        ):
            return
        proposals[candidate] = (
            *_fast_nominal_time_key(graph, candidate),
        )

    # Classical 2-opt changes the order in which an existing route segment is
    # traversed while preserving every service already carried by that segment.
    for first in range(1, len(route_tuple) - 2):
        for last in range(first + 1, len(route_tuple) - 1):
            a, b = route_tuple[first - 1], route_tuple[first]
            c, d = route_tuple[last], route_tuple[last + 1]
            if c not in graph.adjacency.get(a, ()) or d not in graph.adjacency.get(b, ()):
                continue
            consider((
                *route_tuple[:first],
                *reversed(route_tuple[first : last + 1]),
                *route_tuple[last + 1 :],
            ))

    # A serviced closed loop can be traversed in either direction with exactly
    # the same topology length.  Reversing only the loop may create an immediate
    # U-turn at its old exit, so reconnect it to nearby future anchors.  The
    # task-mask and full geometry verifier below remain authoritative.
    repeated_positions: dict[str, list[int]] = {}
    for index, name in enumerate(route_tuple[1:-1], start=1):
        repeated_positions.setdefault(name, []).append(index)
    for positions in repeated_positions.values():
        for position_index, first in enumerate(positions):
            for last in positions[position_index + 1 :]:
                if last - first < 3 or last - first > 16:
                    continue
                reversed_loop = tuple(reversed(route_tuple[first : last + 1]))
                for reconnect_index in range(
                    last + 2, min(len(route_tuple) - 1, last + 8) + 1
                ):
                    target = route_tuple[reconnect_index]
                    cache_key = (route_tuple[last], target)
                    replacements = path_cache.get(cache_key)
                    if replacements is None:
                        replacements = _fast_k_shortest_paths(
                            graph,
                            route_tuple[last],
                            target,
                            FAST_K_SHORTEST_PATHS,
                        )
                        path_cache[cache_key] = replacements
                    for replacement in replacements:
                        consider((
                            *route_tuple[:first],
                            *reversed_loop,
                            *replacement[1:],
                            *route_tuple[reconnect_index + 1 :],
                        ))

    # Large-neighbourhood replacement: remove a cluster of two to six true
    # turns and reconnect its boundary with each of the four shortest loopless
    # graph paths.  The task-mask check rejects any replacement that loses a
    # uniquely serviced required vertex or host edge.
    turn_indices = []
    for index in range(1, len(route_tuple) - 1):
        corner = _corner_profile(
            graph,
            route_tuple[index - 1],
            route_tuple[index],
            route_tuple[index + 1],
        )
        if corner is not None and corner.is_turn:
            turn_indices.append(index)
    for first_turn in range(len(turn_indices)):
        for turn_count in range(2, 7):
            last_turn = first_turn + turn_count - 1
            if last_turn >= len(turn_indices):
                break
            left_index = max(0, turn_indices[first_turn] - 1)
            right_index = min(len(route_tuple) - 1, turn_indices[last_turn] + 1)
            if left_index == 0 or right_index == len(route_tuple) - 1:
                continue
            source, target = route_tuple[left_index], route_tuple[right_index]
            cache_key = (source, target)
            replacements = path_cache.get(cache_key)
            if replacements is None:
                replacements = _fast_k_shortest_paths(
                    graph, source, target, FAST_K_SHORTEST_PATHS
                )
                path_cache[cache_key] = replacements
            original_segment = route_tuple[left_index : right_index + 1]
            for replacement in replacements:
                if replacement == original_segment:
                    continue
                consider((
                    *route_tuple[:left_index],
                    *replacement,
                    *route_tuple[right_index + 1 :],
                ))

    for candidate, _ in sorted(
        proposals.items(), key=lambda item: item[1]
    )[:FAST_LNS_MAX_ATTEMPTS]:
        yield candidate


def _fast_beam_routes(
    graph: PlanGraph,
    *,
    deadline_s: float,
    report,
) -> tuple[list[tuple[str, ...]], dict[str, int]]:
    vertex_names = tuple(sorted(name for name in graph.required if name != graph.start))
    required_edge_keys = tuple(sorted({edge for edge, _ in graph.required_edges.values()}))
    vertex_bits = {name: index for index, name in enumerate(vertex_names)}
    edge_bits = {edge: index for index, edge in enumerate(required_edge_keys)}
    all_vertex_mask = (1 << len(vertex_names)) - 1
    all_edge_mask = (1 << len(required_edge_keys)) - 1
    initial_vertices = 0
    start_bit = vertex_bits.get(graph.start)
    if start_bit is not None:
        initial_vertices |= 1 << start_bit
    initial = _FastBeamState(
        route=(graph.start,),
        covered_vertices=initial_vertices,
        covered_edges=0,
        distance_nm=0,
        turns=(),
        direction_change_count=0,
        max_direction_changes=0,
        max_alternating_turns=0,
        max_true_turns=0,
        last_direction=None,
        current_straight_nm=0,
        longest_straight_nm=0,
        straight_concentration_nm2=0,
        straight_run_count=0,
        nominal_curve_nm=0,
    )
    path_cache: dict[tuple[str, str], tuple[tuple[str, ...], ...]] = {}

    def paths(source: str, target: str) -> tuple[tuple[str, ...], ...]:
        key = (source, target)
        cached = path_cache.get(key)
        if cached is None:
            cached = _fast_k_shortest_paths(
                graph, source, target, FAST_K_SHORTEST_PATHS
            )
            path_cache[key] = cached
        return cached

    beam = [initial]
    completed: dict[tuple[str, ...], _FastBeamState] = {}
    expanded_states = 0
    generated_states = 0
    rounds = 0
    while beam and rounds < FAST_MAX_SEARCH_ROUNDS and time.perf_counter() < deadline_s:
        rounds += 1
        next_states: dict[tuple[Any, ...], _FastBeamState] = {}
        for state in beam:
            if time.perf_counter() >= deadline_s:
                break
            expanded_states += 1
            current = state.route[-1]
            previous = state.route[-2] if len(state.route) >= 2 else None

            for return_path in paths(current, graph.start):
                if previous is not None and len(return_path) >= 2 and return_path[1] == previous:
                    continue
                final_state = _fast_extend_state(
                    graph,
                    state,
                    return_path,
                    vertex_bits,
                    edge_bits,
                    allow_final_start=True,
                )
                if final_state is None:
                    continue
                if (
                    final_state.covered_vertices == all_vertex_mask
                    and final_state.covered_edges == all_edge_mask
                ):
                    completed[final_state.route] = final_state

            task_options: list[tuple[int, str, Any]] = []
            for name, bit in vertex_bits.items():
                if state.covered_vertices & (1 << bit):
                    continue
                candidate_paths = paths(current, name)
                if candidate_paths:
                    task_options.append(
                        (_fast_path_distance_nm(graph, candidate_paths[0]), "vertex", name)
                    )
            for edge, bit in edge_bits.items():
                if state.covered_edges & (1 << bit):
                    continue
                for entry, exit_name in (edge, tuple(reversed(edge))):
                    candidate_paths = paths(current, entry)
                    if candidate_paths:
                        distance = _fast_path_distance_nm(graph, candidate_paths[0])
                        distance += _round_positive(
                            _edge_length(graph, entry, exit_name) * NANOMETRES_PER_MM
                        )
                        task_options.append((distance, "edge", (entry, exit_name)))
            task_options.sort(key=lambda item: (item[0], item[1], item[2]))
            for _, task_kind, task in task_options[:FAST_BEAM_TASK_BRANCHES]:
                suffixes: list[tuple[str, ...]] = []
                if task_kind == "vertex":
                    suffixes.extend(paths(current, str(task)))
                else:
                    entry, exit_name = task
                    for prefix in paths(current, entry):
                        if prefix[-1] != exit_name:
                            suffixes.append((*prefix, exit_name))
                for suffix in suffixes:
                    if previous is not None and len(suffix) >= 2 and suffix[1] == previous:
                        continue
                    new_state = _fast_extend_state(
                        graph,
                        state,
                        suffix,
                        vertex_bits,
                        edge_bits,
                        allow_final_start=False,
                    )
                    if new_state is None:
                        continue
                    generated_states += 1
                    recent_cutoff_nm = new_state.distance_nm - _round_positive(
                        FAST_DIRECTION_WINDOW_MM * NANOMETRES_PER_MM
                    )
                    recent_turn_signature = tuple(
                        (new_state.distance_nm - turn_s_nm, turn_sign)
                        for turn_s_nm, turn_sign in new_state.turns
                        if turn_s_nm >= recent_cutoff_nm
                    )
                    key = (
                        new_state.route[-1],
                        new_state.route[-2] if len(new_state.route) >= 2 else None,
                        new_state.covered_vertices,
                        new_state.covered_edges,
                        recent_turn_signature,
                    )
                    existing = next_states.get(key)
                    if existing is None or _fast_state_rank(
                        graph, new_state, all_vertex_mask, all_edge_mask
                    ) < _fast_state_rank(graph, existing, all_vertex_mask, all_edge_mask):
                        next_states[key] = new_state

        beam = _fast_select_beam_states(
            graph,
            tuple(next_states.values()),
            all_vertex_mask,
            all_edge_mask,
        )
        report(
            "fast_beam_search",
            round=rounds,
            beam_size=len(beam),
            completed_routes=len(completed),
            expanded_states=expanded_states,
            generated_states=generated_states,
        )
        if len(completed) >= FAST_MAX_FEASIBLE_CANDIDATES * 4:
            break

    ordered = sorted(
        completed.values(),
        key=lambda item: _fast_nominal_time_key(graph, item.route),
    )
    return [state.route for state in ordered], {
        "rounds": rounds,
        "expanded_states": expanded_states,
        "generated_states": generated_states,
        "completed_routes": len(completed),
        "required_vertex_tasks": len(vertex_names),
        "required_edge_tasks": len(required_edge_keys),
        "path_cache_entries": len(path_cache),
    }


def solve_anytime_route(payload: Any, progress_callback=None) -> dict[str, Any]:
    """Fast non-proving planner: direction-aware beam/K-path search and LNS."""
    started = time.perf_counter()

    def report(phase: str, **fields: Any) -> None:
        if progress_callback is None:
            return
        try:
            progress_callback({
                "phase": phase,
                "elapsed_s": time.perf_counter() - started,
                **fields,
            })
        except Exception:
            pass

    report("building_fast_graph")
    try:
        if not isinstance(payload, Mapping):
            raise RouteRequestError("invalid_request", "request body must be an object")
        graph = _build_plan_graph(payload)
    except RouteRequestError as exc:
        report("rejected", reason=exc.reason, message=exc.message)
        return _not_proven(exc.reason, exc.message)
    except Exception as exc:  # noqa: BLE001
        report("rejected", reason="fast_graph_error", message=str(exc))
        return _not_proven("fast_graph_error", str(exc))

    deadline_s = started + graph.timeout_s
    model_info = {
        "vertices": len(graph.adjacency),
        "edge_segments": len(graph.edge_lengths),
        "solver_mode": "beam_k_shortest_2opt_lns",
        "beam_width": FAST_BEAM_WIDTH,
        "k_shortest_paths": FAST_K_SHORTEST_PATHS,
        "task_branches": FAST_BEAM_TASK_BRANCHES,
        "direction_change_window_mm": FAST_DIRECTION_WINDOW_MM,
        "direction_change_equivalent_mm": FAST_DIRECTION_CHANGE_EQUIVALENT_MM,
    }
    # Restore the historical fast path first.  It optimizes the directed
    # physical-edge occurrence vector, expands only a bounded number of Euler
    # orders, and leaves the existing beam/LNS implementation as a fallback
    # when HiGHS is unavailable or the incumbent cannot pass v12 verification.
    milp_deadline_s = min(
        deadline_s,
        started + min(graph.timeout_s, FAST_MILP_BUDGET_S),
    )
    milp_candidate, milp_failure = _solve_directed_edge_milp_fast(
        graph,
        deadline_s=milp_deadline_s,
        started_s=started,
        report=report,
    )
    if milp_candidate is not None:
        milp_verification = milp_candidate.get("verification") or {}
        milp_metrics = dict(
            milp_verification.get("fast_direction_change_metrics")
            or milp_candidate.get("fast_direction_change_metrics")
            or {}
        )
        milp_candidate["fast_direction_change_metrics"] = milp_metrics
        milp_candidate["fast_candidate_score"] = {
            "max_direction_changes": milp_metrics.get("max_direction_changes"),
            "direction_change_count": milp_metrics.get("direction_change_count"),
            "max_alternating_turns": milp_metrics.get("max_alternating_turns"),
            "longest_same_direction_turns": milp_metrics.get(
                "longest_same_direction_turns"
            ),
            "max_true_turns": milp_metrics.get("max_true_turns"),
            "objective_time_us": milp_candidate.get("objective_time_us"),
            "objective_time_s": milp_candidate.get("objective_time_s"),
            "peak_speed_cmps": milp_verification.get("peak_speed_cmps"),
            "reached_max_speed_cap": milp_verification.get(
                "reached_max_speed_cap"
            ),
        }
        milp_candidate["fast_candidate_bounds"] = {
            "selection_policy": (
                "shortest_topology_length_then_direction_changes_then_predicted_time"
            ),
            "verified_candidates": int(
                (milp_candidate.get("candidate_generation") or {}).get(
                    "examined", 1
                )
            ),
            "shortest_length_class_candidates": int(
                (milp_candidate.get("candidate_generation") or {}).get(
                    "shortest_length_class_candidates", 1
                )
            ),
            "global_optimality_proven": False,
        }
        milp_candidate["fast_candidate_listing_fast_to_slow"] = [
            {
                "rank": item.get("rank"),
                "selected": bool(item.get("selected")),
                "topology_length_mm": item.get("objective_length_mm"),
                "objective_time_s": item.get("objective_time_s"),
                "direction_change_count": item.get("direction_change_count"),
                "max_direction_changes": item.get("max_direction_changes"),
            }
            for item in (milp_candidate.get("candidates") or [])
        ]
        return milp_candidate
    if milp_failure:
        report("milp_fallback_to_beam", reason=milp_failure)
    feasible: list[dict[str, Any]] = []
    seen_routes: set[tuple[str, ...]] = set()

    beam_routes, beam_stats = _fast_beam_routes(
        graph,
        deadline_s=deadline_s,
        report=report,
    )
    beam_routes.sort(key=lambda route: _fast_nominal_time_key(graph, route))
    report("fast_verifying_candidates", routes=len(beam_routes))
    verification_rejections: list[str] = []
    geometry_checks = 0
    for route in beam_routes:
        if time.perf_counter() >= deadline_s:
            break
        if geometry_checks >= FAST_MAX_GEOMETRY_CHECKS:
            break
        geometry_checks += 1
        rejection: list[str] = []
        candidate = _verify_generated_route_candidate(
            graph,
            list(route),
            deadline_s=deadline_s,
            model_info=model_info,
            rejection=rejection,
        )
        if candidate is None:
            if rejection:
                verification_rejections.append(rejection[0])
            continue
        key = tuple(str(name) for name in candidate["route"])
        if key in seen_routes:
            continue
        seen_routes.add(key)
        feasible.append(candidate)
        report(
            "fast_candidate_accepted",
            candidates=len(feasible),
            max_direction_changes=(
                candidate["verification"]["fast_direction_change_metrics"]
                ["max_direction_changes"]
            ),
        )
        if len(feasible) >= FAST_MAX_FEASIBLE_CANDIDATES:
            break

    lns_attempts = 0
    if feasible and time.perf_counter() < deadline_s:
        ordered_feasible = _fast_ranked_candidate_pool(feasible)
        shortest_candidate = min(
            feasible,
            key=lambda item: (
                int(item.get("objective_length_nm", 1 << 62)),
                _fast_candidate_sort_key(item),
            ),
        )
        lns_sources = [ordered_feasible[0], shortest_candidate]
        if tuple(ordered_feasible[0]["route"]) == tuple(shortest_candidate["route"]):
            lns_sources.pop()
        vertex_names = tuple(sorted(name for name in graph.required if name != graph.start))
        required_edge_keys = tuple(sorted({edge for edge, _ in graph.required_edges.values()}))
        vertex_bits = {name: index for index, name in enumerate(vertex_names)}
        edge_bits = {edge: index for index, edge in enumerate(required_edge_keys)}
        lns_geometry_checks = 0
        for source_candidate in lns_sources:
            canonical_route = tuple(
                graph.aliases.get(name, name)
                for name in source_candidate["route"]
                if graph.aliases.get(name, name) not in graph.required_edges
            )
            for proposal in _fast_two_opt_proposals(
                graph,
                canonical_route,
                vertex_bits,
                edge_bits,
                (1 << len(vertex_names)) - 1,
                (1 << len(required_edge_keys)) - 1,
            ):
                if time.perf_counter() >= deadline_s:
                    break
                if lns_geometry_checks >= FAST_LNS_GEOMETRY_CHECKS:
                    break
                lns_attempts += 1
                rejection = []
                lns_geometry_checks += 1
                candidate = _verify_generated_route_candidate(
                    graph,
                    list(proposal),
                    deadline_s=deadline_s,
                    model_info=model_info,
                    rejection=rejection,
                )
                if candidate is None:
                    continue
                key = tuple(str(name) for name in candidate["route"])
                if key in seen_routes:
                    continue
                seen_routes.add(key)
                feasible.append(candidate)
                feasible = _fast_ranked_candidate_pool(feasible)[
                    :FAST_MAX_FEASIBLE_CANDIDATES
                ]
            if (
                time.perf_counter() >= deadline_s
                or lns_geometry_checks >= FAST_LNS_GEOMETRY_CHECKS
            ):
                break
        report("fast_lns_complete", attempts=lns_attempts, candidates=len(feasible))

    if not feasible:
        reason = "no_fast_feasible_route"
        detail = (
            verification_rejections[0]
            if verification_rejections
            else "Beam Search 在时间预算内没有生成通过拓扑与速度账本校验的闭环路线"
        )
        report("rejected", reason=reason, message=detail, **beam_stats)
        return _not_proven(
            reason,
            detail,
            model={**model_info, **beam_stats},
        )

    ordered_feasible = _fast_ranked_candidate_pool(feasible)
    selected = dict(ordered_feasible[0])
    metrics = dict(
        selected["verification"].get("fast_direction_change_metrics") or {}
    )
    selected_verification = selected["verification"]
    selected.update({
        "status": "feasible",
        "proven_optimal": False,
        "commit_allowed": True,
        "approximate": True,
        "candidate_rank": 1,
        "objective_model": "time_then_length_then_direction_changes",
        "fast_direction_change_metrics": metrics,
        "fast_candidate_score": {
            "max_direction_changes": metrics.get("max_direction_changes"),
            "direction_change_count": metrics.get("direction_change_count"),
            "max_alternating_turns": metrics.get("max_alternating_turns"),
            "longest_same_direction_turns": metrics.get(
                "longest_same_direction_turns"
            ),
            "max_true_turns": metrics.get("max_true_turns"),
            "objective_time_us": selected["objective_time_us"],
            "objective_time_s": selected["objective_time_s"],
            "peak_speed_cmps": selected_verification["peak_speed_cmps"],
            "reached_max_speed_cap": selected_verification[
                "reached_max_speed_cap"
            ],
        },
        "fast_candidate_bounds": {
            "selection_policy": "minimum_predicted_time_then_length_then_direction_changes",
            "verified_candidates": len(ordered_feasible),
        },
    })
    selected["solver"] = {
        **dict(selected.get("solver") or {}),
        "name": "beam_k_shortest_2opt_lns",
        "verification_basis": "independently_verified_feasible_route_only",
        "global_optimality_proven": False,
        "routes_examined": len(feasible),
    }
    selected["model"] = {
        **model_info,
        **beam_stats,
        "lns_attempts": lns_attempts,
        "geometry_checks": geometry_checks,
    }
    selected["candidate_generation"] = {
        "requested": FAST_MAX_FEASIBLE_CANDIDATES,
        "returned": len(ordered_feasible),
        "examined": len(feasible),
        "complete": False,
        "method": "direction_aware_beam_k_shortest_plus_2opt_lns",
        "global_optimality_proven": False,
        "stop_reason": "best_verified_candidate_selected",
        "solve_time_s": time.perf_counter() - started,
        "direction_change_window_mm": FAST_DIRECTION_WINDOW_MM,
        "direction_change_equivalent_mm": FAST_DIRECTION_CHANGE_EQUIVALENT_MM,
        **beam_stats,
        "lns_attempts": lns_attempts,
        "geometry_checks": geometry_checks,
    }
    time_listing = sorted(
        ordered_feasible,
        key=lambda item: (
            int(item["objective_time_us"]),
            _fast_candidate_sort_key(item),
        ),
    )
    result = dict(selected)
    result["fast_candidate_listing_fast_to_slow"] = [
        {
            "time_rank": index + 1,
            "selected": item is ordered_feasible[0],
            "topology_length_mm": item["objective_length_mm"],
            "curve_path_length_mm": item["verification"]["curve_path_length_mm"],
            "straight_path_length_mm": item["verification"]["straight_path_length_mm"],
            "objective_time_s": item["objective_time_s"],
            "peak_speed_cmps": item["verification"]["peak_speed_cmps"],
            "reached_max_speed_cap": item["verification"][
                "reached_max_speed_cap"
            ],
        }
        for index, item in enumerate(time_listing)
    ]
    result["candidates"] = [
        _public_candidate_view(
            item,
            rank=index,
            selected=item is ordered_feasible[0],
        )
        for index, item in enumerate(ordered_feasible, 1)
    ]
    report(
        "complete_feasible",
        route_length_mm=selected["objective_length_mm"],
        smooth_length_mm=selected["smooth_path_length_mm"],
        objective_time_s=selected["objective_time_s"],
        max_direction_changes=metrics.get("max_direction_changes"),
        direction_change_count=metrics.get("direction_change_count"),
        max_true_turns=metrics.get("max_true_turns"),
        candidates=len(feasible),
    )
    return result


__all__ = ["REQUEST_SCHEMA", "RESULT_SCHEMA", "solve_anytime_route"]
