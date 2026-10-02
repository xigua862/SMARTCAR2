# -*- coding: utf-8 -*-
"""Certified exact route backend for the local map editor.

The fast planner already contains the complete directed-edge MILP, exact
SOURCE-rooted connectivity cuts, Euler expansion, and independent geometry
verifier. This module drives those same components in a proof-preserving
mode: the objective is physical topology length only, HiGHS incumbents are
accepted only at zero MIP gap, and a topology is excluded only after every
Euler order in that length class fails the geometry verifier.
"""

from __future__ import annotations

import copy
import time
from typing import Any, Mapping

import fast_route_solver as _fast


REQUEST_SCHEMA = "smartcar.exact-route.request.v6"
RESULT_SCHEMA = "smartcar.exact-route.result.v6"
EXACT_GEOMETRY_MODE = "pure_pursuit_polyline"
TIME_MODEL_VERSION = "constant_accel_speed_cap.v1"


def _seed_route_warm_start(
    graph: _fast.PlanGraph,
    arcs: list[_fast.ModelArc],
    model: _fast.SparseMilp,
    metadata: Mapping[str, Any],
    raw_seed: Any,
) -> list[float] | None:
    """Convert a previously displayed route into a feasible HiGHS start.

    This is only an incumbent hint.  The exact solver still runs with a zero
    gap and independently verifies the returned Euler witness, so a stale or
    invalid editor route is simply ignored.
    """
    if not isinstance(raw_seed, (list, tuple)) or len(raw_seed) < 3:
        return None
    names: list[str] = []
    for raw in raw_seed:
        if not isinstance(raw, str) or not raw:
            return None
        resolved = graph.aliases.get(raw, raw)
        # Interior service markers are display-only and must not become MILP
        # vertices, matching _verify_generated_route_candidate.
        if resolved in graph.required_edges:
            continue
        if resolved not in graph.coords:
            return None
        if not names or names[-1] != resolved:
            names.append(resolved)
    if len(names) < 3 or names[0] != graph.start or names[-1] != graph.start:
        return None
    if graph.start_direction is not None and names[1] != graph.start_direction:
        reversed_names = [names[0], *reversed(names[1:-1]), names[-1]]
        if len(reversed_names) < 3 or reversed_names[1] != graph.start_direction:
            return None
        names = reversed_names
    for left, right in zip(names, names[1:]):
        if right not in graph.adjacency.get(left, ()) or left == right:
            return None
    for previous, current, following in zip(names, names[1:], names[2:]):
        if previous == following:
            return None

    by_chain: dict[tuple[str, ...], list[int]] = {}
    for arc in arcs:
        by_chain.setdefault(arc.chain, []).append(arc.index)
    order: list[int] = []
    first = by_chain.get((names[0], names[1]), [])
    start_arcs = [index for index in first if arcs[index].kind == "start"]
    if not start_arcs:
        return None
    order.append(start_arcs[0])
    for left, right in zip(names[1:], names[2:]):
        candidates = [
            index for index in by_chain.get((left, right), [])
            if arcs[index].kind == "continue"
        ]
        if not candidates:
            return None
        order.append(candidates[0])
    if not order or arcs[order[-1]].target < 0:
        return None
    finish = [
        arc.index for arc in arcs
        if arc.kind == "finish"
        and arc.source == arcs[order[-1]].target
        and arc.target == _fast.SINK_NODE
    ]
    if not finish:
        return None
    order.append(finish[0])

    x_variables = metadata.get("x_variables") or []
    flow_variables = metadata.get("support_flow_variables") or []
    used_variables = metadata.get("state_used_variables") or []
    if not (
        len(arcs) == len(x_variables) == len(flow_variables)
        and len(used_variables) == len(graph.adjacency)  # checked below by index
    ):
        # The state list is not keyed by graph vertices; use the model width
        # checks below instead of assuming a particular graph size.
        if len(arcs) != len(x_variables) or len(arcs) != len(flow_variables):
            return None
    values = [0.0] * len(model.costs)
    counts = [0] * len(arcs)
    for arc_index in order:
        counts[arc_index] += 1
    for arc, variable, count in zip(arcs, x_variables, counts):
        if count > 0:
            upper = metadata["arc_upper_bounds"][arc.index]
            if upper <= 0 or count > upper:
                return None
            values[variable] = float(count)

    # One unit of support demand per distinct state.  Assigning flow as the
    # number of as-yet-unseen states makes every state balance exactly, even
    # when the route revisits an internal state.
    seen_states: set[int] = set()
    state_order = [arcs[index].target for index in order if arcs[index].target >= 0]
    unique_states = set(state_order)
    remaining = len(unique_states)
    for arc_index in order:
        flow_variable = flow_variables[arc_index]
        values[flow_variable] = float(remaining)
        target = arcs[arc_index].target
        if target >= 0 and target not in seen_states:
            seen_states.add(target)
            remaining -= 1
    for state_index in seen_states:
        if state_index < len(used_variables):
            values[used_variables[state_index]] = 1.0
    if remaining != 0:
        return None
    return values


def _as_fast_request(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Adapt the exact v6 request to the shared graph-model request shape."""
    request = copy.deepcopy(dict(payload))
    request["schema"] = _fast.REQUEST_SCHEMA
    settings = request.setdefault("route_settings", {})
    speed = settings.get("speed_plan") or {}
    straight = float(speed.get("straight_cmps", 700.0))
    settings["speed_plan"] = {
        "max_speed_cmps": straight,
        "max_lateral_accel_cmps2": float(speed.get("max_lateral_accel_cmps2", 13500.0)),
        "max_accel_cmps2": float(speed.get("max_accel_cmps2", 3000.0)),
        "max_decel_cmps2": float(speed.get("max_decel_cmps2", 5000.0)),
        "safety_margin_mm": float(speed.get("safety_margin_mm", 80.0)),
        "start_speed_cmps": float(speed.get("start_speed_cmps", 0.0)),
        "time_quantum_us": 1,
        "time_model": _fast.TIME_MODEL_VERSION,
    }
    settings["geometry_mode"] = _fast.TOPOLOGY_GEOMETRY_MODE
    settings["radius"] = 0
    settings["ease_mm"] = 0
    settings["smoothing_profile"] = _fast.SMOOTHING_PROFILE
    settings["composite_profile_version"] = _fast.COMPOSITE_PROFILE_VERSION
    settings["composite_partition_policy"] = _fast.COMPOSITE_PARTITION_POLICY
    settings["composite_corridor_profile"] = _fast.COMPOSITE_CORRIDOR_PROFILE
    settings["composite_collision_profile"] = _fast.COMPOSITE_COLLISION_PROFILE
    settings["smoothing_ease_mm"] = float(settings.get("smoothing_ease_mm", 0.0))
    settings["max_curvature_per_mm"] = float(
        settings.get("max_curvature_per_mm") or (1.0 / 150.0)
    )
    settings["composite_short_gap_mm"] = float(
        settings.get("composite_short_gap_mm") or 100.0
    )
    request.setdefault("solve_options", {})["return_to_start"] = True
    # The shared graph parser validates its legacy fast-planner budget at
    # <=120 s; the exact backend keeps its own outer deadline below.
    request["solve_options"]["timeout_s"] = min(
        120.0, max(1.0, float(request["solve_options"].get("timeout_s", 120.0)))
    )
    # seed_route is consumed by the exact backend to create a HiGHS warm
    # start. The shared fast parser deliberately rejects unknown seed fields.
    request["solve_options"].pop("seed_route", None)
    return request


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


def _exact_candidate(
    candidate: Mapping[str, Any],
    *,
    rank: int,
    solver_info: Mapping[str, Any],
    model_info: Mapping[str, Any],
    arc_counts: list[int],
    euler_orders: int,
) -> dict[str, Any]:
    """Turn a shared verifier candidate into an exact, auditable result."""
    result = copy.deepcopy(dict(candidate))
    result["schema"] = RESULT_SCHEMA
    result["status"] = "optimal"
    result["proven_optimal"] = True
    result["commit_allowed"] = True
    result["approximate"] = False
    result["candidate_rank"] = rank
    result["objective_model"] = "exact_topology_length_then_verified_smooth_route"
    result["time_model"] = TIME_MODEL_VERSION
    result["geometry_mode"] = EXACT_GEOMETRY_MODE
    verification = dict(result.get("verification") or {})
    verification["time_model"] = TIME_MODEL_VERSION
    verification["speed_straight_cmps"] = verification.get(
        "max_speed_cmps", model_info.get("max_speed_cmps", 0.0)
    )
    verification["speed_corner_cmps"] = verification.get(
        "max_speed_cmps", model_info.get("max_speed_cmps", 0.0)
    )
    result["verification"] = verification
    result["solver"] = {
        **dict(solver_info),
        "name": "HiGHS",
        "global_optimality_proven": True,
        "objective_certificate": "zero_gap_integer_lattice_minimum_topology",
        "proof": "zero_gap_minimum_length_class_plus_verified_euler_witness",
        "arc_count_vector": arc_counts,
        "euler_orders_checked": euler_orders,
    }
    result["model"] = {
        **dict(model_info),
        "solver_mode": "directed_edge_milp_exact",
        "milp_objective": "topology_physical_length_nm",
        "milp_global_optimality_proven": True,
    }
    result["candidate_generation"] = {
        "requested": 1,
        "returned": 1,
        "complete": True,
        "method": "zero_gap_minimum_length_class_and_verified_euler_witness",
        "global_optimality_proven": True,
        "stop_reason": "proven_route",
        "euler_orders": euler_orders,
    }
    result["candidates"] = []
    return result


def solve_exact_route(payload: Any, progress_callback=None) -> dict[str, Any]:
    """Solve one route and return it only after a global zero-gap proof."""
    if not isinstance(payload, Mapping):
        return _not_proven("invalid_request", "exact route request must be an object")
    started = time.perf_counter()
    solve_options = payload.get("solve_options") or {}
    seed_route = solve_options.get("seed_route")
    try:
        request = _as_fast_request(payload)
        graph = _fast._build_plan_graph(request)
    except Exception as exc:  # noqa: BLE001
        return _not_proven(
            getattr(exc, "reason", "invalid_request"),
            str(getattr(exc, "message", exc)),
        )

    timeout_s = float((payload.get("solve_options") or {}).get("timeout_s", 900.0))
    deadline = started + min(3600.0, max(1.0, timeout_s))
    report = progress_callback or (lambda *_args, **_kwargs: None)
    old_penalty = _fast.FAST_MILP_TURN_PENALTY_MM
    _fast.FAST_MILP_TURN_PENALTY_MM = 0.0
    topology_attempt = 0
    euler_orders_checked = 0
    try:
        try:
            states, arcs, model, metadata = _fast._build_directed_edge_model(graph)
        except Exception as exc:  # noqa: BLE001
            return _not_proven(
                getattr(exc, "reason", "model_error"),
                str(getattr(exc, "message", exc)),
            )
        model_info = {
            "vertices": len(graph.adjacency),
            "edge_segments": len(graph.edge_lengths),
            "directed_edge_states": len(states),
            "directed_state_arcs": len(arcs),
            **metadata,
        }
        warm_start = _seed_route_warm_start(
            graph, arcs, model, metadata, seed_route
        )
        if warm_start is not None:
            model_info["warm_start"] = "editor_route_hint"
            # A geometry-verified seed is a valid primal upper bound. Adding
            # that bound closes a large part of the root relaxation on the
            # final map, while remaining safe for the later shortest-proof
            # loop because the seed itself is a feasible witness.
            seed_rejection: list[str] = []
            seed_candidate = _fast._verify_generated_route_candidate(
                graph,
                seed_route,
                deadline_s=deadline,
                model_info=model_info,
                rejection=seed_rejection,
            )
            if seed_candidate is not None:
                seed_objective = sum(
                    model.costs[variable] * warm_start[variable]
                    for variable in metadata["x_variables"]
                )
                if seed_objective > 0.0:
                    model.add_row(
                        {
                            variable: model.costs[variable]
                            for variable in metadata["x_variables"]
                            if model.costs[variable]
                        },
                        upper=float(seed_objective),
                    )
                    model_info["warm_start_verified"] = True
                    model_info["warm_start_objective_units"] = seed_objective
            else:
                model_info["warm_start_verified"] = False
                model_info["warm_start_rejection"] = seed_rejection[:3]
        while time.perf_counter() < deadline:
            topology_attempt += 1
            report("exact_milp_solving", topology_attempt=topology_attempt)
            outcome, values, solver_info = _fast._run_highs_with_connectivity(
                model,
                arcs,
                metadata["x_variables"],
                metadata["arc_upper_bounds"],
                deadline,
                warm_start=warm_start,
                accept_incumbent=False,
                stop_after_first_incumbent=False,
            )
            # Warm starts are useful only for the first length-class solve.
            # Once an exact no-good cut is added the old vector is infeasible.
            warm_start = None
            if outcome == "infeasible":
                return _not_proven(
                    "no_feasible_route",
                    "没有满足所有必经点和拓扑约束的路线",
                    solver=solver_info,
                    model=model_info,
                )
            if outcome != "optimal" or values is None:
                reason = (
                    "solver_unavailable" if outcome == "solver_unavailable"
                    else ("timeout" if outcome == "not_optimal" else f"solver_{outcome}")
                )
                return _not_proven(
                    reason,
                    "HiGHS 未在时限内闭合当前最短拓扑长度类",
                    solver=solver_info,
                    model=model_info,
                )

            counts = _fast._selected_arc_counts(arcs, metadata["x_variables"], values)
            accepted = None
            class_complete = True
            report("exact_euler_enumeration", topology_attempt=topology_attempt)
            for order in _fast._all_euler_arc_orders(arcs, counts, deadline):
                euler_orders_checked += 1
                canonical_route, _ = _fast._route_from_arc_order(graph, arcs, order)
                candidate = _fast._verify_generated_route_candidate(
                    graph,
                    canonical_route,
                    deadline_s=deadline,
                    model_info=model_info,
                    rejection=[],
                )
                if candidate is not None:
                    accepted = candidate
                    break
                if time.perf_counter() >= deadline:
                    class_complete = False
                    break
            if accepted is not None and class_complete:
                return _exact_candidate(
                    accepted,
                    rank=1,
                    solver_info=solver_info,
                    model_info=model_info,
                    arc_counts=counts,
                    euler_orders=euler_orders_checked,
                )
            if not class_complete or time.perf_counter() >= deadline:
                return _not_proven(
                    "timeout",
                    "最短拓扑长度类尚未完成全部路序与平滑验证",
                    solver=solver_info,
                    model=model_info,
                    proof_progress={
                        "topology_attempts": topology_attempt,
                        "euler_orders_checked": euler_orders_checked,
                    },
                )

            # Every Euler order in this integer length class failed the hard
            # verifier. Exclude precisely this arc-count vector and prove the
            # next physical-length class; no heuristic penalty is introduced.
            _fast._exclude_arc_count_solution(
                model, metadata["x_variables"], counts
            )
            report(
                "exact_length_class_rejected",
                topology_attempt=topology_attempt,
                euler_orders_checked=euler_orders_checked,
            )
        return _not_proven("timeout", "精确求解时限已到，尚未证明最短路线")
    finally:
        _fast.FAST_MILP_TURN_PENALTY_MM = old_penalty


__all__ = ["REQUEST_SCHEMA", "RESULT_SCHEMA", "solve_exact_route"]
