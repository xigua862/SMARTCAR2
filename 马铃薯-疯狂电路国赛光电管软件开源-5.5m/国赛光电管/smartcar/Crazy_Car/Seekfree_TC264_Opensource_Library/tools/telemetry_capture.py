#!/usr/bin/env python3
import argparse
import csv
import html
import io
import json
import math
import re
import struct
import sys
import time
import zlib
from pathlib import Path


EMEM_SIZE = 0x50000
HEADER_SIZE = 256
HEADER_REGION_SIZE = 0x1000
LOCALIZATION_CONFIG_OFFSET = 0x100
LOCALIZATION_CONFIG_SIZE = 64
LOCALIZATION_CONFIG_VERSION = 2
HEADER_CRC_OFFSET = 24
SCHEMA_VERSION = 21
SUPPORTED_SCHEMA_VERSIONS = (SCHEMA_VERSION,)
PROTOCOL_VERSION = 2
SESSION_MAGIC = b"TLM1"
WIRE_MAGIC = b"TLG1"
LINK_MAGIC = b"TLD1"
LOCALIZATION_CONFIG_MAGIC = b"LCF1"
LOCALIZATION_FEATURE_OFFSET = 0x140
LOCALIZATION_FEATURE_HEADER_SIZE = 16
LOCALIZATION_FEATURE_RECORD_SIZE = 32
LOCALIZATION_FEATURE_CAPACITY = 116
LOCALIZATION_FEATURE_VERSION = 3
LOCALIZATION_FEATURE_MAGIC = b"LFD1"
TRACKING_CONFIG_OFFSET = 0xFD0
TRACKING_CONFIG_SIZE = 48
TRACKING_CONFIG_VERSION = 2
TRACKING_CONFIG_MAGIC = b"TRK1"
LINK_FLAG_LOGGER_ENABLED = 1
LINK_FLAG_RUN_FINALIZED = 2
LINK_FLAG_IDLE = 4
PAYLOAD_READ_CHUNK_SIZE = 4096
WIRE_FLAG_ZERO_RLE = 0x0001
WIRE_BLOCK_ZERO_FLAG = 0x8000
WIRE_BLOCK_LENGTH_MASK = 0x7FFF
STATE_DISABLED = 0
STATE_READY = 1
STATE_RECORDING = 2
STATE_FROZEN = 3
STATE_EXPORTING = 4

CONTROL_OFFSET = 0x01000
CONTROL_REGION_SIZE = 0x11B40
NAVIGATION_OFFSET = 0x12B40
NAVIGATION_REGION_SIZE = 0x073E0
VISION_OFFSET = 0x19F20
VISION_REGION_SIZE = 0x080C0
EVENT_OFFSET = 0x21FE0
EVENT_REGION_SIZE = 0x01000
EVENT_CAPACITY = 256
LOCALIZATION_CAPACITY = 512
LOCALIZATION_RECORD_SIZE = 44
IMU_OFFSET = 0x22FE0
IMU_REGION_SIZE = 0x13560
RESERVED_OFFSET = 0x36540
RESERVED_REGION_SIZE = 0x19AC0
LOCALIZATION_OFFSET = RESERVED_OFFSET
PHOTOTUBE_SCAN_OFFSET = LOCALIZATION_OFFSET + LOCALIZATION_CAPACITY * LOCALIZATION_RECORD_SIZE
PHOTOTUBE_SCAN_REGION_SIZE = RESERVED_OFFSET + RESERVED_REGION_SIZE - PHOTOTUBE_SCAN_OFFSET
PHOTOTUBE_SCAN_HEADER_SIZE = 56
PHOTOTUBE_SCAN_RECORD_SIZE = 16
PHOTOTUBE_SCAN_CAPACITY = 5120
PHOTOTUBE_SCAN_MAGIC = b"PTS1"
PHOTOTUBE_SCAN_VERSION = 14
PHOTOTUBE_SCAN_FIT_WINDOW_POINTS = 32
PHOTOTUBE_SCAN_MIN_FIT_SPAN_MM = 20.0
SCAN_HIT_SINGLE = 1 << 0
SCAN_HIT_MULTIPLE = 1 << 1
SCAN_HIT_FULL_COVERAGE = 1 << 2
SCAN_HIT_EDGE = 1 << 3
SCAN_HIT_RUN_SAMPLE = 1 << 4
SCAN_HIT_CENTER = 1 << 5
SCAN_HIT_BOUNDARY = 1 << 6

CONTROL_CAPACITY = 3296
NAVIGATION_CAPACITY = 1648
VISION_CAPACITY = 3296
IMU_CAPACITY = 3960
IMU_SAMPLE_HZ = 240
IMU_SOURCE_SAMPLE_HZ = 480
IMU_DECIMATION = 2
IMU_RECORD_SIZE = 20
CONTROL_LOG_DIVIDER = 5
CONTROL_LOG_RATE_HZ = 200

CONTROL_STRUCT = struct.Struct("<H10h")
NAVIGATION_STRUCT = struct.Struct("<2H6hH")
VISION_STRUCT = struct.Struct("<HHhHH")
EVENT_STRUCT = struct.Struct("<IBBHii")
IMU_STRUCT = struct.Struct("<H6hH2h")
PHOTOTUBE_SCAN_HEADER_STRUCT = struct.Struct("<4sHHHHI20H")
PHOTOTUBE_SCAN_STRUCT = struct.Struct("<HHhhhhHBB")
LOCALIZATION_STRUCT = struct.Struct("<II2h2H2B7h2H2hH2B")
LOCALIZATION_CONFIG_STRUCT = struct.Struct("<4sHHIIBBBBI10f")
LOCALIZATION_FEATURE_HEADER_STRUCT = struct.Struct("<4s6H")
LOCALIZATION_FEATURE_STRUCT = struct.Struct("<4B3I3H3B3BhH")
TRACKING_CONFIG_STRUCT = struct.Struct("<4sHHII7fHH")
WIRE_STRUCT = struct.Struct("<4sHHII")
LINK_STRUCT = struct.Struct("<4sHHII")


class TelemetryProtocolMismatchError(Exception):
    pass

if LOCALIZATION_STRUCT.size != LOCALIZATION_RECORD_SIZE:
    raise RuntimeError("localization struct size does not match schema 21")
if LOCALIZATION_FEATURE_HEADER_STRUCT.size != LOCALIZATION_FEATURE_HEADER_SIZE:
    raise RuntimeError("localization feature header size does not match schema 21")
if LOCALIZATION_FEATURE_STRUCT.size != LOCALIZATION_FEATURE_RECORD_SIZE:
    raise RuntimeError("localization feature record size does not match schema 21")
if TRACKING_CONFIG_STRUCT.size != TRACKING_CONFIG_SIZE:
    raise RuntimeError("tracking config size does not match schema 21")
if (
    LOCALIZATION_FEATURE_OFFSET
    + LOCALIZATION_FEATURE_HEADER_SIZE
    + LOCALIZATION_FEATURE_CAPACITY * LOCALIZATION_FEATURE_RECORD_SIZE
    != TRACKING_CONFIG_OFFSET
):
    raise RuntimeError("localization feature and tracking config regions are not contiguous")
if TRACKING_CONFIG_OFFSET + TRACKING_CONFIG_SIZE > HEADER_REGION_SIZE:
    raise RuntimeError("tracking config exceeds telemetry header region")
if PHOTOTUBE_SCAN_HEADER_STRUCT.size != PHOTOTUBE_SCAN_HEADER_SIZE:
    raise RuntimeError(
        f"phototube scan header size does not match version {PHOTOTUBE_SCAN_VERSION}"
    )
if PHOTOTUBE_SCAN_STRUCT.size != PHOTOTUBE_SCAN_RECORD_SIZE:
    raise RuntimeError(
        f"phototube scan struct size does not match version {PHOTOTUBE_SCAN_VERSION}"
    )
if IMU_SAMPLE_HZ * IMU_DECIMATION != IMU_SOURCE_SAMPLE_HZ:
    raise RuntimeError("IMU source/log sample rates do not match decimation")

CONTROL_FIELDS = (
    "tick_ms",
    "encoder_l",
    "encoder_r",
    "actual_l_cmps",
    "actual_r_cmps",
    "actual_avg_cmps",
    "plan_v_cmps",
    "target_l_cmps",
    "target_r_cmps",
    "accel_l_cmps2",
    "accel_r_cmps2",
    "ff_l_duty",
    "ff_r_duty",
    "pi_l_duty",
    "pi_r_duty",
    "pwm_l",
    "pwm_r",
    "yaw_rate_error_degps",
    "measured_yaw_rate_degps",
    "target_yaw_rate_degps",
    "imu_alignment_age_ms",
    "yaw_rate_feedforward_turn_cmps",
    "yaw_rate_feedback_turn_cmps",
    "yaw_rate_feedforward_from_navigation_cmps",
    "pd_d_turn_cmps",
    "gkd_turn_cmps",
    "pd_p_turn_inferred_cmps",
    "pd_p_turn_from_navigation_cmps",
    "navigation_alignment_age_ms",
    "tracking_turn_reconstructed_cmps",
    "tracking_turn_reconstruction_error_cmps",
    "line_valid",
    "line_position_mm",
    "phototube_alignment_age_ms",
    "tracking_turn_cmps",
    "final_turn_cmps",
    "turn_limit_adjustment_cmps",
    "speed_error_actual_minus_plan_cmps",
    "left_error_actual_minus_target_cmps",
    "right_error_actual_minus_target_cmps",
    "wheel_balance_actual_left_minus_right_cmps",
    "wheel_balance_target_left_minus_right_cmps",
)
NAVIGATION_FIELDS = (
    "timestamp_ms",
    "progress_index",
    "progress_points",
    "actual_x_mm",
    "actual_y_mm",
    "actual_yaw_deg",
    "actual_yaw_unwrapped_deg",
    "ideal_x_mm",
    "ideal_y_mm",
    "ideal_yaw_deg",
    "ideal_yaw_unwrapped_deg",
    "lookahead_progress_points",
    "lookahead_x_mm",
    "lookahead_y_mm",
    "lookahead_command_mm",
    "lookahead_base_mm",
    "lookahead_plan_v_cmps",
    "lookahead_speed_cmps",
    "tracking_speed_cmps",
    "lookahead_control_alignment_age_ms",
    "lookahead_curve_radius_mm",
    "lookahead_curve_distance_mm",
    "lookahead_distance_mm",
    "pure_pursuit_target_yaw_deg",
    "pure_pursuit_error_deg",
    "pure_pursuit_error_rad",
    "target_speed_cmps",
    "reference_acceleration_cmps2",
    "speed_segment_index",
    "speed_segment_end_index",
    "acceleration_segment_index",
    "acceleration_segment_end_index",
    "curvature_per_mm",
    "curvature_derivative_per_mm2",
    "curvature_radius_mm",
    "curvature_preview_time_s",
    "curvature_preview_progress_points",
    "curvature_preview_per_mm",
    "curvature_feedforward_yaw_rate_radps",
    "curvature_feedforward_yaw_rate_dps",
    "target_yaw_rate_from_navigation_radps",
    "target_yaw_rate_from_navigation_dps",
    "reference_yaw_rate_radps",
    "reference_yaw_rate_dps",
    "reference_lateral_acceleration_cmps2",
    "cross_track_mm",
    "along_track_mm",
    "heading_error_deg",
    "heading_error_rad",
    "map_pose_path_distance_mm",
    "raw_pose_path_distance_mm",
    "raw_x_mm",
    "raw_y_mm",
    "raw_yaw_deg",
    "raw_yaw_unwrapped_deg",
    "map_minus_raw_x_mm",
    "map_minus_raw_y_mm",
    "map_minus_raw_position_mm",
    "localization_update_seq",
    "map_from_odom_tx_mm",
    "map_from_odom_ty_mm",
    "map_from_odom_yaw_deg",
    "map_from_odom_yaw_unwrapped_deg",
    "map_from_odom_translation_norm_mm",
    "map_from_odom_transform_changed",
    "localization_transaction_index",
    "localization_transaction_status",
    "localization_commit_timestamp_ms",
    "localization_capture_frame_seq",
    "localization_feature_occurrence",
    "localization_match_type",
    "localization_t_before_tx_mm",
    "localization_t_before_ty_mm",
    "localization_t_before_yaw_deg",
    "localization_delta_odom_dx_mm",
    "localization_delta_odom_dy_mm",
    "localization_delta_odom_dyaw_deg",
    "localization_t_after_tx_mm",
    "localization_t_after_ty_mm",
    "localization_t_after_yaw_deg",
    "localization_map_before_x_mm",
    "localization_map_before_y_mm",
    "localization_map_before_yaw_deg",
    "localization_map_after_x_mm",
    "localization_map_after_y_mm",
    "localization_map_after_yaw_deg",
    "localization_progress_before_points",
    "localization_progress_after_points",
    "localization_map_before_cross_track_mm",
    "localization_candidate_cross_track_mm",
    "localization_map_after_cross_track_mm",
    "localization_cross_track_delta_mm",
    "localization_map_before_heading_error_deg",
    "localization_candidate_heading_error_deg",
    "localization_map_after_heading_error_deg",
    "localization_map_before_pp_p_turn_cmps",
    "localization_candidate_pp_p_turn_cmps",
    "localization_map_after_pp_p_turn_cmps",
    "localization_pp_p_turn_delta_cmps",
    "localization_rms_before_mm",
    "localization_rms_after_mm",
    "localization_inlier_count",
)
VISION_FIELDS = (
    "tick_ms",
    "tick_s",
    "frame_seq",
    "frame_seq_delta",
    "line_position_mm",
    "active_mask",
    "active_mask_hex",
    "observation_flags",
    "line_valid",
    "edge_valid",
    "full_coverage",
    "selection_ambiguous",
    "peak_count",
    "white_percent",
    "threshold_gray",
    "active_sensor_count",
    "active_span_sensors",
    "active_block_count",
    "active_first_sensor",
    "active_last_sensor",
    "active_centroid_sensor",
    "active_left_half_count",
    "active_right_half_count",
    "reject_reason",
    "reject_reason_name",
)
EVENT_FIELDS = (
    "timestamp_ms", "type", "type_name", "source", "code", "value0", "value1",
)
LOCALIZATION_FIELDS = (
    "transaction_index", "retained_index", "storage_slot", "timestamp_ms",
    "capture_frame_seq", "travel_distance_mm",
    "localization_update_seq", "route_feature_occurrence", "status",
    "status_name", "match_type", "match_type_name", "reject_reason",
    "reject_reason_name", "t_before_tx_mm", "t_before_ty_mm",
    "t_before_yaw_deg", "delta_odom_dx_mm", "delta_odom_dy_mm",
    "delta_odom_dyaw_deg", "raw_x_mm", "raw_y_mm", "raw_yaw_deg",
    "t_after_tx_mm", "t_after_ty_mm",
    "t_after_yaw_deg", "candidate_t_after_tx_mm", "candidate_t_after_ty_mm",
    "candidate_t_after_yaw_deg", "map_before_x_mm", "map_before_y_mm",
    "map_before_yaw_deg", "map_after_x_mm", "map_after_y_mm",
    "map_after_yaw_deg", "candidate_map_after_x_mm",
    "candidate_map_after_y_mm", "candidate_map_after_yaw_deg",
    "progress_before_points", "progress_after_points",
    "candidate_progress_after_points",
    "rms_before_mm", "rms_after_mm", "inlier_count",
    "delta_odom_translation_mm", "delta_odom_yaw_abs_deg",
    "map_pose_jump_mm", "map_pose_jump_yaw_deg",
    "progress_delta_points", "progress_delta_mm",
    "candidate_map_pose_jump_mm", "candidate_map_pose_jump_yaw_deg",
    "candidate_progress_delta_points", "candidate_progress_delta_mm",
    "candidate_transform_valid", "transform_changed",
    "map_before_cross_track_mm", "map_before_along_track_mm",
    "map_before_heading_error_deg", "map_before_path_distance_mm",
    "candidate_cross_track_mm", "candidate_along_track_mm",
    "candidate_heading_error_deg", "candidate_path_distance_mm",
    "map_after_cross_track_mm", "map_after_along_track_mm",
    "map_after_heading_error_deg", "map_after_path_distance_mm",
    "candidate_cross_track_delta_mm", "candidate_along_track_delta_mm",
    "candidate_heading_error_delta_deg",
    "cross_track_delta_mm", "along_track_delta_mm",
    "heading_error_delta_deg",
    "map_before_pp_error_deg", "candidate_pp_error_deg",
    "map_after_pp_error_deg",
    "map_before_pp_p_turn_cmps", "candidate_pp_p_turn_cmps",
    "map_after_pp_p_turn_cmps", "candidate_pp_p_turn_delta_cmps",
    "pp_p_turn_delta_cmps", "localization_control_alignment_age_ms",
    "feature_x_mm", "feature_y_mm",
    "map_before_to_feature_distance_mm", "map_after_to_feature_distance_mm",
    "candidate_map_after_to_feature_distance_mm",
    "timestamp_raw_ms",
)
LOCALIZATION_FEATURE_FIELDS = (
    "route_feature_occurrence", "feature_type", "feature_type_name",
    "feature_progress_points", "status_flags", "registered", "encountered",
    "accepted", "partial", "outcome", "attempt_count", "accepted_count",
    "last_timestamp_ms", "last_reject_reason",
    "last_reject_reason_name", "reject_reason_mask", "reject_reason_names",
    "detail_flags", "detail_names", "last_detail_flags", "last_detail_names",
    "observed_progress_points",
    "line0_point_count", "line1_point_count", "line2_point_count",
    "line0_frame_count", "line1_frame_count", "line2_frame_count",
    "failure_value", "failure_value_name", "failure_value_unit",
)
IMU_FIELDS = (
    "timestamp_us",
    "timestamp_ms",
    "acc_x_g",
    "acc_y_g",
    "acc_z_g",
    "gyro_x_dps",
    "gyro_y_dps",
    "gyro_z_dps",
    "quaternion_q0",
    "quaternion_q1",
    "quaternion_q2",
    "quaternion_q3",
    "quaternion_norm",
    "acc_norm_g",
    "gyro_norm_dps",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
)
PHOTOTUBE_SCAN_FIELDS = (
    "coordinate_frame",
    "retained_index",
    "storage_slot",
    "frame_seq",
    "frame_seq_delta",
    "frame_seq_delta_raw",
    "capture_ms",
    "capture_ms_raw",
    "x_mm",
    "y_mm",
    "lateral_mm",
    "raw_yaw_deg",
    "raw_axle_x_mm",
    "raw_axle_y_mm",
    "confidence",
    "point_flags",
    "point_type",
    "edge_hit",
    "centerline_hit",
    "boundary_hit",
    "travel_distance_mod_mm",
    "travel_distance_mm",
    "capture_time_s",
    "travel_distance_m",
    "point_weight",
    "map_x_mm",
    "map_y_mm",
    "map_axle_x_mm",
    "map_axle_y_mm",
    "final_map_x_mm",
    "final_map_y_mm",
    "final_map_axle_x_mm",
    "final_map_axle_y_mm",
    "map_yaw_deg",
    "final_map_yaw_deg",
    "map_transform_timestamp_ms",
    "map_localization_update_seq",
    "map_transform_source",
    "map_transform_time_boundary_ambiguous",
)
ROUTE_FIELDS = (
    "progress_index",
    "distance_mm",
    "x_mm",
    "y_mm",
    "yaw_deg",
    "curvature_per_mm",
    "curvature_derivative_per_mm2",
    "curvature_preview_progress_points",
    "curvature_feedforward_yaw_rate_dps",
    "curvature_radius_mm",
    "target_speed_cmps",
    "reference_acceleration_cmps2",
    "speed_segment_index",
    "acceleration_segment_index",
    "map_takeover_allowed",
    "map_takeover_region_index",
    "scan_feature_occurrence",
    "scan_feature_type",
)

# codex_packet.json is a second-level drill-down index, not a replacement for
# the complete CSV files.  Keep only signals needed to locate a suspicious
# time window; detailed analysis can then query the relevant CSV columns.
CONTROL_PREVIEW_FIELDS = (
    "tick_ms",
    "plan_v_cmps",
    "actual_avg_cmps",
    "target_l_cmps",
    "target_r_cmps",
    "actual_l_cmps",
    "actual_r_cmps",
    "pwm_l",
    "pwm_r",
    "yaw_rate_error_degps",
    "target_yaw_rate_degps",
    "yaw_rate_feedforward_turn_cmps",
    "yaw_rate_feedback_turn_cmps",
    "pd_p_turn_inferred_cmps",
    "pd_d_turn_cmps",
    "gkd_turn_cmps",
    "line_position_mm",
    "tracking_turn_cmps",
    "final_turn_cmps",
)
NAVIGATION_PREVIEW_FIELDS = (
    "timestamp_ms",
    "progress_index",
    "progress_points",
    "actual_x_mm",
    "actual_y_mm",
    "actual_yaw_deg",
    "ideal_x_mm",
    "ideal_y_mm",
    "ideal_yaw_deg",
    "lookahead_command_mm",
    "lookahead_base_mm",
    "lookahead_plan_v_cmps",
    "lookahead_speed_cmps",
    "tracking_speed_cmps",
    "curvature_preview_time_s",
    "curvature_preview_progress_points",
    "curvature_feedforward_yaw_rate_dps",
    "target_yaw_rate_from_navigation_dps",
    "pure_pursuit_target_yaw_deg",
    "pure_pursuit_error_deg",
    "target_speed_cmps",
    "reference_acceleration_cmps2",
    "curvature_per_mm",
    "curvature_derivative_per_mm2",
    "cross_track_mm",
    "along_track_mm",
    "heading_error_deg",
    "raw_x_mm",
    "raw_y_mm",
    "raw_yaw_deg",
    "map_from_odom_tx_mm",
    "map_from_odom_ty_mm",
    "map_from_odom_yaw_deg",
    "map_from_odom_translation_norm_mm",
    "map_from_odom_transform_changed",
    "localization_update_seq",
)
VISION_PREVIEW_FIELDS = (
    "tick_ms",
    "frame_seq",
    "line_position_mm",
    "active_mask_hex",
    "line_valid",
    "selection_ambiguous",
    "peak_count",
    "white_percent",
    "threshold_gray",
    "active_sensor_count",
    "reject_reason",
)
IMU_PREVIEW_FIELDS = (
    "timestamp_ms",
    "acc_x_g",
    "acc_y_g",
    "acc_z_g",
    "gyro_z_dps",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "quaternion_norm",
)
DECODED_OUTPUT_NAMES = (
    "metadata.json",
    "control.csv",
    "navigation.csv",
    "phototube.csv",
    "events.csv",
    "localization_transactions.csv",
    "localization_feature_diagnostics.csv",
    "imu.csv",
    "phototube_scan.csv",
    "planned_route.csv",
    "codex_packet.json",
    "codex_prompt.md",
    "telemetry_dashboard.html",
    "analysis_summary.json",
    "telemetry_overview.svg",
    "telemetry_overview.png",
)

ANALYSIS_SCHEMA_VERSION = 14

EVENT_TYPE_NAMES = {
    1: "run_start",
    2: "run_stop",
    3: "prestart",
    4: "navigation_state",
    5: "vision_accept_reject",
    6: "telemetry_drop",
    7: "saturation",
    8: "control_limit",
    9: "protection",
    10: "runtime_budget",
}
RUNTIME_BUDGET_CODE_NAMES = {
    1: "control_exec",
    2: "control_period",
    3: "phototube_scan",
    4: "duty_10ms",
    5: "nav_match",
    6: "emem_service",
    7: "imu_snapshot_failure",
    8: "imu_snapshot_age",
    9: "scan_epoch",
    10: "imu_isr",
    11: "imu_sample_failure",
    12: "phototube_adc_failure",
}
SCAN_MATCH_TYPE_NAMES = {
    1: "straight",
    2: "corner",
    3: "junction",
}
SCAN_MATCH_REJECT_NAMES = {
    1: "not_enough_points",
    2: "near_junction",
    3: "partial_corner",
    4: "geometry",
    5: "rms",
    6: "correction_bound",
    7: "ambiguous_feature",
    8: "partial_junction",
    9: "feature_consumed",
    10: "wait_confirmation",
    11: "no_fresh_evidence",
    12: "no_improvement",
    13: "projection_failed",
    14: "commit_busy",
    15: "feature_order_regression",
    16: "feature_progress_mismatch",
    17: "stopped_before_commit",
    18: "straight_not_significant",
    19: "ambiguous_straight_residual_clusters",
    20: "progress_regression",
    21: "strong_correction_needs_confirmation",
    22: "path_error_worsened",
    23: "straight_no_map_line",
    24: "straight_candidate_points",
    25: "straight_candidate_weight",
    26: "straight_residual_mode",
    27: "straight_mode_points",
    28: "straight_inlier_points",
    29: "straight_inlier_weight",
    30: "straight_inlier_ratio",
    31: "straight_span_frames_or_tangent",
}
SCAN_FEATURE_TYPE_NAMES = {
    1: "corner",
    2: "junction",
}
LOCALIZATION_FEATURE_DETAIL_NAMES = {
    1 << 0: "invalid_map_geometry",
    1 << 1: "insufficient_total_points",
    1 << 2: "insufficient_total_weight",
    1 << 3: "line0_insufficient_points",
    1 << 4: "line1_insufficient_points",
    1 << 5: "line2_insufficient_points",
    1 << 6: "line0_insufficient_weight",
    1 << 7: "line1_insufficient_weight",
    1 << 8: "line2_insufficient_weight",
    1 << 9: "line0_insufficient_span",
    1 << 10: "line1_insufficient_span",
    1 << 11: "line2_insufficient_span",
    1 << 12: "line0_insufficient_frames",
    1 << 13: "line1_insufficient_frames",
    1 << 14: "line2_insufficient_frames",
    1 << 15: "no_noncollinear_line_pair",
    1 << 16: "singular_se2_solver",
    1 << 17: "rms_exceeded",
    1 << 18: "translation_correction_exceeded",
    1 << 19: "yaw_correction_exceeded",
    1 << 20: "ambiguous_feature",
    1 << 21: "waiting_independent_confirmation",
    1 << 22: "confirmation_inconsistent",
    1 << 23: "insufficient_rms_improvement",
    1 << 24: "corrected_progress_projection_failed",
    1 << 25: "atomic_commit_busy",
    1 << 26: "feature_order_regression",
    1 << 27: "feature_progress_mismatch",
    1 << 28: "feature_already_consumed",
    1 << 29: "no_fresh_evidence",
    1 << 30: "progress_regression",
}
STOP_REASON_NAMES = {
    0: "none",
    1: "frame_watchdog",
    2: "line_lost_after_200mm_black_scan",
    3: "path_done",
    4: "plan_mismatch",
    5: "roll",
    6: "pitch",
    7: "assistant",
    8: "manual",
    9: "run_timeout",
    10: "speed_test_done",
    11: "motor_fault",
    12: "trigger_trap",
    13: "imu_stale",
}
PRESTART_STATE_NAMES = {
    0: "idle",
    1: "warmup",
    2: "wait_confirm",
    3: "suction_settling",
    4: "running",
    5: "stopping",
}
NAV_STATE_NAMES = {
    0: "idle",
    1: "navigating",
    2: "finished",
    3: "error",
}
EVENT_SOURCE_NAMES = {
    1: "prestart",
    2: "logger",
    3: "navigation",
    4: "control",
    5: "vision",
    6: "safety",
}
CONTROL_LIMIT_BIT_NAMES = {
    1: "feedback_authority",
    2: "anti_reverse_turn_limit",
    4: "left_pwm_soft_limit",
    8: "right_pwm_soft_limit",
    16: "slip_guard",
}

PARAMETER_NAMES = (
    "motor_ff_ks_left_duty",
    "motor_ff_kv_left_duty_per_cmps",
    "motor_ff_ka_left_accel_duty_per_cmps2",
    "motor_ff_ka_left_decel_duty_per_cmps2",
    "motor_ff_ks_right_duty",
    "motor_ff_kv_right_duty_per_cmps",
    "motor_ff_ka_right_accel_duty_per_cmps2",
    "motor_ff_ka_right_decel_duty_per_cmps2",
    "motor_ff_zero_speed_cmps",
    "motor_ff_allow_active_decel_duty",
    "speed_KP",
    "speed_KI",
    "motor_ff_active_decel_zero_speed_cmps",
    "speed_IncrementLimit",
    "motor_ff_active_decel_error_cmps",
    "speed_profile_max_accel_cmps2",
    "speed_profile_max_decel_cmps2",
    "speed_profile_safety_margin_mm",
    "motor_ff_max_accel_cmps2",
    "motor_ff_max_jerk_cmps3",
)
INTEGER_PARAMETER_INDICES = {9}

def _u16(payload, offset):
    return struct.unpack_from("<H", payload, offset)[0]


def _u32(payload, offset):
    return struct.unpack_from("<I", payload, offset)[0]


def _f32(payload, offset):
    return struct.unpack_from("<f", payload, offset)[0]


def crc32_excluding_header_crc(payload):
    view = memoryview(payload)
    if len(view) != EMEM_SIZE:
        raise ValueError(f"payload length must be {EMEM_SIZE} bytes")
    crc = zlib.crc32(view[:HEADER_CRC_OFFSET])
    crc = zlib.crc32(view[HEADER_CRC_OFFSET + 4 :], crc)
    return crc & 0xFFFFFFFF


def _decode_parameters(payload):
    words = struct.unpack_from("<20I", payload, 124)
    result = {}
    for index, (name, word) in enumerate(zip(PARAMETER_NAMES, words)):
        if name.startswith("reserved_"):
            # Retired Flash slots are not part of the current parameter model.
            result[name] = None
        elif index in INTEGER_PARAMETER_INDICES:
            result[name] = word & 0xFF
        else:
            result[name] = struct.unpack("<f", struct.pack("<I", word))[0]
    return result


def _decode_localization_config(payload):
    fields = LOCALIZATION_CONFIG_STRUCT.unpack_from(
        payload, LOCALIZATION_CONFIG_OFFSET
    )
    (
        magic,
        version,
        size,
        config_hash,
        algorithm_version,
        sensor_count,
        adc_group,
        adc_channel,
        mux_selector_count,
        scan_hz,
        sensor_spacing_mm,
        sensor_forward_mm,
        scan_min_step_mm,
        match_travel_window_mm,
        feature_radius_mm,
        max_feature_point_residual_mm,
        max_rms_mm,
        max_correction_mm,
        max_correction_yaw_deg,
        black_scan_stop_distance_mm,
    ) = fields
    if magic != LOCALIZATION_CONFIG_MAGIC:
        raise ValueError("invalid localization config magic")
    if version != LOCALIZATION_CONFIG_VERSION:
        raise ValueError("unsupported localization config version")
    if size != LOCALIZATION_CONFIG_SIZE:
        raise ValueError("invalid localization config size")
    return {
        "config_hash": f"0x{config_hash:08X}",
        "algorithm_version": algorithm_version,
        "sensor_count": sensor_count,
        "adc_group": adc_group,
        "adc_channel": adc_channel,
        "mux_selector_count": mux_selector_count,
        "scan_hz": scan_hz,
        "sensor_spacing_mm": sensor_spacing_mm,
        "sensor_forward_mm": sensor_forward_mm,
        "scan_min_step_mm": scan_min_step_mm,
        "match_travel_window_mm": match_travel_window_mm,
        "feature_radius_mm": feature_radius_mm,
        "max_feature_point_residual_mm": max_feature_point_residual_mm,
        "max_rms_mm": max_rms_mm,
        "max_correction_mm": max_correction_mm,
        "max_correction_yaw_deg": max_correction_yaw_deg,
        "black_scan_stop_distance_mm": black_scan_stop_distance_mm,
    }


def _decode_tracking_config(payload):
    fields = TRACKING_CONFIG_STRUCT.unpack_from(payload, TRACKING_CONFIG_OFFSET)
    (
        magic,
        version,
        size,
        config_hash,
        algorithm_version,
        wheel_track_mm,
        preview_time_s,
        min_lookahead_mm,
        max_lookahead_mm,
        kd_cmps_per_degps_per_speed_scale,
        gkd_cmps_per_radps,
        turn_limit_ratio,
        path_sample_mm_x100,
        target_rate_window_ms,
    ) = fields
    path_sample_mm = path_sample_mm_x100 / 100.0
    # The fixed TRK1 block predates the curve-only preview floor field, so the
    # tracking algorithm revision carries that negotiated constant.  v12 used
    # 200mm, v13/v14 used 240mm, and v15+ use 200mm for close alternating
    # turns where the larger floor looked across the following bend.
    if algorithm_version >= 15:
        curve_min_lookahead_mm = 200.0
    elif algorithm_version >= 13:
        curve_min_lookahead_mm = 240.0
    elif algorithm_version >= 12:
        curve_min_lookahead_mm = 200.0
    else:
        curve_min_lookahead_mm = min_lookahead_mm
    if magic != TRACKING_CONFIG_MAGIC:
        raise ValueError("invalid tracking config magic")
    if version != TRACKING_CONFIG_VERSION:
        raise ValueError("unsupported tracking config version")
    if size != TRACKING_CONFIG_SIZE:
        raise ValueError("invalid tracking config size")
    computed_hash = 2166136261
    for value in payload[
        TRACKING_CONFIG_OFFSET + 12 : TRACKING_CONFIG_OFFSET + size
    ]:
        computed_hash ^= value
        computed_hash = (computed_hash * 16777619) & 0xFFFFFFFF
    if computed_hash != config_hash:
        raise ValueError("tracking config hash mismatch")
    yaw_rate_controller = algorithm_version >= 16
    curvature_feedforward_controller = algorithm_version >= 17
    if not (
        wheel_track_mm > 0.0
        and preview_time_s > 0.0
        and min_lookahead_mm > 0.0
        and max_lookahead_mm >= min_lookahead_mm
        and path_sample_mm > 0.0
        and (
            (yaw_rate_controller
             and ((curvature_feedforward_controller and target_rate_window_ms > 0)
                  or (not curvature_feedforward_controller and target_rate_window_ms == 0))
             and math.isclose(
                 kd_cmps_per_degps_per_speed_scale, 0.0, abs_tol=1.0e-9
             )
             and gkd_cmps_per_radps > 0.0)
            or ((not yaw_rate_controller) and target_rate_window_ms > 0)
        )
    ):
        raise ValueError("invalid dynamic tracking constants")

    header_summary = (
        (_f32(payload, 112), min_lookahead_mm, "minimum lookahead"),
        (_f32(payload, 224), preview_time_s, "preview time"),
        (_f32(payload, 228), min_lookahead_mm, "minimum lookahead summary"),
        (_f32(payload, 232), max_lookahead_mm, "maximum lookahead summary"),
        (
            _f32(payload, 236),
            kd_cmps_per_degps_per_speed_scale,
            "tracking legacy KD slot summary",
        ),
        (_f32(payload, 108), path_sample_mm, "path sample"),
    )
    for actual, expected, label in header_summary:
        if not math.isclose(actual, expected, rel_tol=1.0e-6, abs_tol=1.0e-6):
            raise ValueError(f"tracking config/header {label} mismatch")
    return {
        "config_hash": f"0x{config_hash:08X}",
        "algorithm_version": algorithm_version,
        "controller_type": (
            "offline_curvature_yaw_rate" if curvature_feedforward_controller
            else ("pure_pursuit_yaw_rate" if yaw_rate_controller
                  else "legacy_target_yaw_pd_gkd")
        ),
        "curvature_preview_time_s": (
            target_rate_window_ms / 1000.0
            if curvature_feedforward_controller else None
        ),
        "wheel_track_mm": wheel_track_mm,
        "preview_time_s": preview_time_s,
        "min_lookahead_mm": min_lookahead_mm,
        "curve_min_lookahead_mm": curve_min_lookahead_mm,
        "max_lookahead_mm": max_lookahead_mm,
        "kd_cmps_per_degps_per_speed_scale": (
            kd_cmps_per_degps_per_speed_scale
        ),
        "gkd_cmps_per_radps": gkd_cmps_per_radps,
        "yaw_rate_feedback_gain_cmps_per_radps": (
            gkd_cmps_per_radps if yaw_rate_controller else None
        ),
        "turn_limit_ratio": turn_limit_ratio,
        "path_sample_mm": path_sample_mm,
        # For v17 this fixed-layout field is the offline curvature preview,
        # not a target-yaw differentiation window. Keep the raw name for
        # schema compatibility and expose the semantic name above.
        "target_rate_window_ms": target_rate_window_ms,
    }


def _decode_localization_features(payload):
    fields = LOCALIZATION_FEATURE_HEADER_STRUCT.unpack_from(
        payload, LOCALIZATION_FEATURE_OFFSET
    )
    (
        magic,
        version,
        header_size,
        record_size,
        capacity,
        registered_count,
        diagnostic_drop_count,
    ) = fields
    if magic != LOCALIZATION_FEATURE_MAGIC:
        raise ValueError("invalid localization feature diagnostic magic")
    if version != LOCALIZATION_FEATURE_VERSION:
        raise ValueError("unsupported localization feature diagnostic version")
    if header_size != LOCALIZATION_FEATURE_HEADER_SIZE:
        raise ValueError("invalid localization feature diagnostic header size")
    if record_size != LOCALIZATION_FEATURE_RECORD_SIZE:
        raise ValueError("invalid localization feature diagnostic record size")
    if capacity != LOCALIZATION_FEATURE_CAPACITY:
        raise ValueError("invalid localization feature diagnostic capacity")

    records = []
    for index in range(capacity):
        values = LOCALIZATION_FEATURE_STRUCT.unpack_from(
            payload,
            LOCALIZATION_FEATURE_OFFSET + header_size + index * record_size,
        )
        status_flags = values[1]
        if not status_flags & 1:
            continue
        reason_mask = values[4]
        detail_flags = values[5]
        last_reason = values[2]
        last_detail_flags = values[6]
        if last_detail_flags & (1 << 17):
            failure_value_name, failure_value_unit = "rms", "mm"
        elif last_detail_flags & (1 << 18):
            failure_value_name, failure_value_unit = "translation_correction", "mm"
        elif last_detail_flags & (1 << 19):
            failure_value_name, failure_value_unit = "yaw_correction", "deg"
        elif last_detail_flags & (1 << 23):
            failure_value_name, failure_value_unit = "rms_improvement", "mm"
        elif last_detail_flags & (1 << 1):
            failure_value_name, failure_value_unit = "total_point_count", "points"
        elif last_detail_flags & (1 << 2):
            failure_value_name, failure_value_unit = "effective_weight", "weight"
        else:
            failure_value_name, failure_value_unit = "none", ""
        records.append({
            "route_feature_occurrence": values[0],
            "feature_type": None,
            "feature_type_name": "unknown",
            "feature_progress_points": None,
            "status_flags": status_flags,
            "registered": True,
            "encountered": bool(status_flags & 2),
            "accepted": bool(status_flags & 4),
            "partial": bool(status_flags & 8),
            "outcome": "pending",
            "attempt_count": values[7],
            "accepted_count": 1 if status_flags & 4 else 0,
            "last_timestamp_ms": values[8] if status_flags & 2 else None,
            "last_reject_reason": last_reason,
            "last_reject_reason_name": (
                "none" if last_reason == 0 else
                SCAN_MATCH_REJECT_NAMES.get(last_reason, f"unknown_{last_reason}")
            ),
            "reject_reason_mask": f"0x{reason_mask:08X}",
            "reject_reason_names": "|".join(
                name for reason, name in SCAN_MATCH_REJECT_NAMES.items()
                if reason_mask & (1 << reason)
            ),
            "detail_flags": f"0x{detail_flags:08X}",
            "detail_names": "|".join(
                name for flag, name in LOCALIZATION_FEATURE_DETAIL_NAMES.items()
                if detail_flags & flag
            ),
            "last_detail_flags": f"0x{last_detail_flags:08X}",
            "last_detail_names": "|".join(
                name for flag, name in LOCALIZATION_FEATURE_DETAIL_NAMES.items()
                if last_detail_flags & flag
            ),
            "observed_progress_points": values[9],
            "line0_point_count": values[10],
            "line1_point_count": values[11],
            "line2_point_count": values[12],
            "line0_frame_count": values[13],
            "line1_frame_count": values[14],
            "line2_frame_count": values[15],
            "failure_value": values[16] / 10.0,
            "failure_value_name": failure_value_name,
            "failure_value_unit": failure_value_unit,
        })
    if len(records) != registered_count:
        raise ValueError(
            "localization feature registered count does not match fixed slots"
        )
    return records, diagnostic_drop_count


def _classify_localization_features(records, navigation, route_reference=None):
    max_progress = max(
        (row["progress_points"] for row in navigation),
        default=-1,
    )
    calibration_allowed = (route_reference or {}).get(
        "scan_corner_calibration_allowed"
    )
    for record in records:
        if record["accepted"]:
            record["outcome"] = "partial" if record["partial"] else "accepted"
        elif record["encountered"]:
            record["outcome"] = "failed"
        elif max_progress >= record["feature_progress_points"]:
            occurrence = record["route_feature_occurrence"]
            if (calibration_allowed is not None and
                    occurrence < len(calibration_allowed) and
                    not calibration_allowed[occurrence]):
                record["outcome"] = "disabled_by_map"
            else:
                record["outcome"] = "passed_without_candidate"
        else:
            record["outcome"] = "not_reached"
    return records


def _attach_localization_feature_reference(records, route_reference):
    feature_types = route_reference["scan_type"]
    feature_progress = route_reference["scan_progress"]
    for record in records:
        occurrence = record["route_feature_occurrence"]
        if occurrence >= len(feature_types):
            raise ValueError("localization feature occurrence exceeds nav_path.h")
        feature_type = feature_types[occurrence]
        record["feature_type"] = feature_type
        record["feature_type_name"] = SCAN_FEATURE_TYPE_NAMES.get(
            feature_type, f"unknown_{feature_type}"
        )
        record["feature_progress_points"] = feature_progress[occurrence]
    return records


def _validate_header(payload):
    if len(payload) != EMEM_SIZE:
        raise ValueError(f"payload length must be {EMEM_SIZE} bytes")
    if bytes(payload[0:4]) != SESSION_MAGIC:
        raise ValueError("invalid session magic")
    schema_version = _u16(payload, 4)
    if schema_version not in SUPPORTED_SCHEMA_VERSIONS:
        raise ValueError("unsupported schema version")
    if _u16(payload, 6) != HEADER_SIZE:
        raise ValueError("invalid header size")
    if payload[8] != STATE_FROZEN:
        raise ValueError("session is not frozen")
    if payload[9] != 1:
        raise ValueError("telemetry initialization was not successful")
    localization_config = _decode_localization_config(payload)
    tracking_config = _decode_tracking_config(payload)
    localization_features, diagnostic_drop_count = (
        _decode_localization_features(payload)
    )

    layout = {
        "control_offset": CONTROL_OFFSET,
        "control_capacity": CONTROL_CAPACITY,
        "navigation_offset": NAVIGATION_OFFSET,
        "navigation_capacity": NAVIGATION_CAPACITY,
        "vision_offset": VISION_OFFSET,
        "vision_capacity": VISION_CAPACITY,
        "event_offset": EVENT_OFFSET,
        "event_capacity": EVENT_CAPACITY,
        "localization_offset": LOCALIZATION_OFFSET,
        "localization_capacity": LOCALIZATION_CAPACITY,
        "imu_offset": IMU_OFFSET,
        "imu_capacity": IMU_CAPACITY,
    }
    if _u32(payload, 208) != IMU_OFFSET:
        raise ValueError("invalid IMU offset")
    if _u16(payload, 212) != IMU_STRUCT.size:
        raise ValueError("invalid IMU record size")
    if _u16(payload, 214) != IMU_CAPACITY:
        raise ValueError("invalid IMU capacity")
    if _u16(payload, 216) != IMU_SAMPLE_HZ:
        raise ValueError("invalid IMU sample rate")
    if _u16(payload, 218) != IMU_DECIMATION:
        raise ValueError("invalid IMU decimation")
    if abs(_f32(payload, 120) - IMU_SOURCE_SAMPLE_HZ) > 0.5:
        raise ValueError("invalid IMU source sample rate")
    if _u16(payload, 220) != CONTROL_LOG_DIVIDER:
        raise ValueError("invalid control log divider")
    if _u16(payload, 222) != CONTROL_LOG_RATE_HZ:
        raise ValueError("invalid control log rate")
    expected_layout = (
        (_u32(payload, 52), layout["control_offset"], "control offset"),
        (_u32(payload, 56), layout["navigation_offset"], "navigation offset"),
        (_u32(payload, 60), layout["vision_offset"], "vision offset"),
        (_u32(payload, 64), layout["event_offset"], "event offset"),
        (_u16(payload, 68), CONTROL_STRUCT.size, "control record size"),
        (_u16(payload, 70), NAVIGATION_STRUCT.size, "navigation record size"),
        (_u16(payload, 72), VISION_STRUCT.size, "vision record size"),
        (_u16(payload, 74), EVENT_STRUCT.size, "event record size"),
        (_u16(payload, 76), layout["control_capacity"], "control capacity"),
        (_u16(payload, 78), layout["navigation_capacity"], "navigation capacity"),
        (_u16(payload, 80), layout["vision_capacity"], "vision capacity"),
        (_u16(payload, 82), layout["event_capacity"], "event capacity"),
    )
    for actual, expected, label in expected_layout:
        if actual != expected:
            raise ValueError(f"invalid {label}: {actual}")

    counts = {
        "control": _u16(payload, 36),
        "navigation": _u16(payload, 38),
        "vision": _u16(payload, 40),
        "events": _u16(payload, 42),
        "localization": _u16(payload, 240),
    }
    capacities = {
        "control": layout["control_capacity"],
        "navigation": layout["navigation_capacity"],
        "vision": layout["vision_capacity"],
        "events": layout["event_capacity"],
        "localization": layout["localization_capacity"],
    }
    counts["imu"] = _u16(payload, 204)
    capacities["imu"] = layout["imu_capacity"]
    if _u32(payload, 244) != LOCALIZATION_OFFSET:
        raise ValueError("invalid localization offset")
    if _u16(payload, 248) != LOCALIZATION_RECORD_SIZE:
        raise ValueError("invalid localization record size")
    if _u16(payload, 250) != LOCALIZATION_CAPACITY:
        raise ValueError("invalid localization capacity")
    for name, count in counts.items():
        if count > capacities[name]:
            raise ValueError(f"{name} count exceeds capacity")

    stored_crc = _u32(payload, HEADER_CRC_OFFSET)
    if stored_crc == 0:
        raise ValueError("payload CRC is zero")
    computed_crc = crc32_excluding_header_crc(payload)
    if stored_crc != computed_crc:
        raise ValueError(
            f"payload CRC mismatch: stored=0x{stored_crc:08X} computed=0x{computed_crc:08X}"
        )
    return (
        counts,
        stored_crc,
        localization_config,
        tracking_config,
        localization_features,
        diagnostic_drop_count,
        schema_version,
        layout,
    )


def _decode_metadata(
    payload, counts, crc, localization_config, tracking_config,
    schema_version, layout
):
    start_uptime_ms = _u32(payload, 16)
    end_uptime_ms = _u32(payload, 20)
    constants = {
        "wheel_diameter_mm": _f32(payload, 88),
        "gear_ratio": _f32(payload, 92),
        "distance_per_pulse_mm": _f32(payload, 96),
        "cmps_per_count": _f32(payload, 100),
        "pid_sample_s": _f32(payload, 104),
        "nav_sample_mm": _f32(payload, 108),
        "nav_min_lookahead_mm": _f32(payload, 112),
        "encoder_ppr": _u16(payload, 116),
        "imu_flags": _u16(payload, 118),
        "imu_rate_hz": _f32(payload, 120),
    }
    imu_metadata = {
        "enabled": True,
        "sample_rate_hz": _u16(payload, 216),
        "source_sample_rate_hz": _f32(payload, 120),
        "decimation": _u16(payload, 218),
        "record_size": _u16(payload, 212),
        "capacity": _u16(payload, 214),
        "offset": _u32(payload, 208),
        "count": counts.get("imu", 0),
        "drop_count": _u16(payload, 206),
        "duration_capacity_s": (
            (IMU_CAPACITY / float(_u16(payload, 216)))
            if _u16(payload, 216)
            else 0.0
        ),
    }
    return {
        "schema_version": _u16(payload, 4),
        "state": payload[8],
        "init_status": payload[9],
        "stop_reason": payload[10],
        "stop_reason_name": STOP_REASON_NAMES.get(
            payload[10], f"unknown_{payload[10]}"
        ),
        "route_index": payload[11],
        "session_sequence": _u32(payload, 12),
        "record_time_base": "run_relative_ms",
        "record_timestamp_origin": "run_start",
        "session_bounds_time_base": "boot_uptime_ms",
        "start_uptime_ms": start_uptime_ms,
        "end_uptime_ms": end_uptime_ms,
        "duration_ms": (end_uptime_ms - start_uptime_ms) & 0xFFFFFFFF,
        "payload_crc32": f"0x{crc:08X}",
        "route_hash": f"0x{_u32(payload, 28):08X}",
        "route_point_count": _u16(payload, 32),
        "vision_first_frame_seq": (
            _u32(payload, 252) if counts.get("vision", 0) else None
        ),
        "final_prestart_state": payload[34],
        "final_prestart_state_name": PRESTART_STATE_NAMES.get(
            payload[34], f"unknown_{payload[34]}"
        ),
        "final_nav_state": payload[35],
        "final_nav_state_name": NAV_STATE_NAMES.get(
            payload[35], f"unknown_{payload[35]}"
        ),
        "firmware_config_fingerprint": f"0x{_u32(payload, 84):08X}",
        "localization_config": localization_config,
        "tracking_config": tracking_config,
        "counts": counts,
        "imu": imu_metadata,
        "localization": {
            "offset": _u32(payload, 244),
            "record_size": _u16(payload, 248),
            "capacity": _u16(payload, 250),
            "count": counts.get("localization", 0),
            "overwrite_count": _u16(payload, 242),
            "oldest_storage_slot": (
                _u16(payload, 242) % LOCALIZATION_CAPACITY
                if counts.get("localization", 0) == LOCALIZATION_CAPACITY else 0
            ),
            "first_retained_transaction_index": (
                _u16(payload, 242) if counts.get("localization", 0) else None
            ),
        },
        "drops": {
            "control": _u16(payload, 44),
            "navigation": _u16(payload, 46),
            "vision": _u16(payload, 48),
            "events": _u16(payload, 50),
            "imu": _u16(payload, 206),
            "localization_ledger_overwrite": _u16(payload, 242),
        },
        "constants": constants,
        "control_config": tracking_config,
        "parameters": _decode_parameters(payload),
        "units": {
            "control_speed": "cm/s",
            "navigation_position": "mm",
            "angle": "degree",
            "phototube_position": "mm (selected array candidate)",
            "vision_active_mask": "16-bit channel bitmap",
            "phototube_active_percent": "percent (active array channels)",
            "phototube_threshold": "ADC value / 16",
            "imu_acceleration": "g",
            "imu_angular_rate": "deg/s",
            "imu_quaternion": "unit quaternion",
            "imu_timestamp": "run-relative us",
        },
        "alignment_hint": "compare runs by navigation.progress_index",
    }


def _motor_feedforward(parameters, side, target_speed_cmps, target_accel_cmps2):
    k_s = float(parameters[f"motor_ff_ks_{side}_duty"])
    k_v = float(parameters[f"motor_ff_kv_{side}_duty_per_cmps"])
    phase = "accel" if target_accel_cmps2 >= 0 else "decel"
    k_a = float(parameters[
        f"motor_ff_ka_{side}_{phase}_duty_per_cmps2"
    ])
    zero_speed = float(parameters["motor_ff_zero_speed_cmps"])
    feedforward = k_v * target_speed_cmps + k_a * target_accel_cmps2
    if target_speed_cmps > zero_speed:
        feedforward += k_s
    elif target_speed_cmps < -zero_speed:
        feedforward -= k_s
    if (
        not parameters["motor_ff_allow_active_decel_duty"]
        and target_speed_cmps >= -zero_speed
        and feedforward < 0.0
    ):
        feedforward = 0.0
    return feedforward


def _decode_control(payload, count, cmps_per_count, parameters, offset=CONTROL_OFFSET):
    records = []
    for index in range(count):
        fields = CONTROL_STRUCT.unpack_from(payload, offset + index * CONTROL_STRUCT.size)
        (
            tick_ms,
            encoder_l,
            encoder_r,
            target_l_x10,
            target_r_x10,
            accel_l_cmps2,
            accel_r_cmps2,
            pwm_l,
            pwm_r,
            yaw_rate_error_degps_x10,
            tracking_turn_x10,
        ) = fields
        target_l_cmps = target_l_x10 / 10.0
        target_r_cmps = target_r_x10 / 10.0
        yaw_rate_error_degps = yaw_rate_error_degps_x10 / 10.0
        tracking_turn_cmps = tracking_turn_x10 / 10.0
        final_turn_cmps = 0.5 * (target_r_cmps - target_l_cmps)
        plan_v_cmps = 0.5 * (target_l_cmps + target_r_cmps)
        # Schema 21 keeps these columns for compatibility with older captures.
        # The current firmware follows the pre-race pure incremental-PI path,
        # so there is no motor feedforward to reconstruct from the header.
        ff_l_duty = 0.0
        ff_r_duty = 0.0
        pi_l_duty = float(pwm_l)
        pi_r_duty = float(pwm_r)
        records.append(
            {
                "tick_ms": tick_ms,
                "encoder_l": encoder_l,
                "encoder_r": encoder_r,
                "actual_l_cmps": encoder_l * cmps_per_count,
                "actual_r_cmps": encoder_r * cmps_per_count,
                "actual_avg_cmps": 0.5
                * (encoder_l * cmps_per_count + encoder_r * cmps_per_count),
                "plan_v_cmps": plan_v_cmps,
                "target_l_cmps": target_l_cmps,
                "target_r_cmps": target_r_cmps,
                "accel_l_cmps2": accel_l_cmps2,
                "accel_r_cmps2": accel_r_cmps2,
                "ff_l_duty": ff_l_duty,
                "ff_r_duty": ff_r_duty,
                "pi_l_duty": pi_l_duty,
                "pi_r_duty": pi_r_duty,
                "pwm_l": pwm_l,
                "pwm_r": pwm_r,
                "yaw_rate_error_degps": yaw_rate_error_degps,
                "measured_yaw_rate_degps": None,
                "target_yaw_rate_degps": None,
                "imu_alignment_age_ms": None,
                "yaw_rate_feedforward_turn_cmps": None,
                "yaw_rate_feedback_turn_cmps": None,
                "yaw_rate_feedforward_from_navigation_cmps": None,
                "pd_d_turn_cmps": None,
                "gkd_turn_cmps": None,
                "pd_p_turn_inferred_cmps": None,
                "pd_p_turn_from_navigation_cmps": None,
                "navigation_alignment_age_ms": None,
                "tracking_turn_reconstructed_cmps": None,
                "tracking_turn_reconstruction_error_cmps": None,
                "line_valid": False,
                "line_position_mm": None,
                "phototube_alignment_age_ms": None,
                "tracking_turn_cmps": tracking_turn_cmps,
                "final_turn_cmps": final_turn_cmps,
                "turn_limit_adjustment_cmps": (
                    final_turn_cmps - tracking_turn_cmps
                ),
            }
        )
        record = records[-1]
        record["speed_error_actual_minus_plan_cmps"] = (
            record["actual_avg_cmps"] - record["plan_v_cmps"]
        )
        record["left_error_actual_minus_target_cmps"] = (
            record["actual_l_cmps"] - record["target_l_cmps"]
        )
        record["right_error_actual_minus_target_cmps"] = (
            record["actual_r_cmps"] - record["target_r_cmps"]
        )
        record["wheel_balance_actual_left_minus_right_cmps"] = (
            record["actual_l_cmps"] - record["actual_r_cmps"]
        )
        record["wheel_balance_target_left_minus_right_cmps"] = (
            record["target_l_cmps"] - record["target_r_cmps"]
        )
    return records


def _route_interpolate(values, progress_points):
    if not values:
        return 0.0
    progress_points = max(0.0, min(float(progress_points), len(values) - 1))
    index = int(progress_points)
    if index >= len(values) - 1:
        return float(values[-1])
    fraction = progress_points - index
    return values[index] + fraction * (values[index + 1] - values[index])


def _route_interpolate_yaw_cdeg(values, progress_points):
    if not values:
        return 0.0
    progress_points = max(0.0, min(float(progress_points), len(values) - 1))
    index = int(progress_points)
    if index >= len(values) - 1:
        return float(values[-1])
    fraction = progress_points - index
    delta = (values[index + 1] - values[index] + 18000) % 36000 - 18000
    return values[index] + fraction * delta


def _route_pose_errors(route_reference, x_mm, y_mm, yaw_deg,
                       progress_points):
    """Reconstruct route-relative pose errors at a transaction projection."""
    if progress_points is None:
        return None
    ideal_x_mm = _route_interpolate(route_reference["x"], progress_points)
    ideal_y_mm = _route_interpolate(route_reference["y"], progress_points)
    ideal_yaw_deg = _wrap_degrees(
        _route_interpolate_yaw_cdeg(
            route_reference["yaw_cdeg"], progress_points
        ) / 100.0
    )
    path_yaw_rad = math.radians(ideal_yaw_deg)
    tangent_x = math.cos(path_yaw_rad)
    tangent_y = math.sin(path_yaw_rad)
    dx_mm = x_mm - ideal_x_mm
    dy_mm = y_mm - ideal_y_mm
    return {
        "cross_track_mm": -dx_mm * tangent_y + dy_mm * tangent_x,
        "along_track_mm": dx_mm * tangent_x + dy_mm * tangent_y,
        "heading_error_deg": _wrap_degrees(yaw_deg - ideal_yaw_deg),
        "path_distance_mm": math.hypot(dx_mm, dy_mm),
    }


def _pure_pursuit_p_term(route_reference, tracking_config,
                         x_mm, y_mm, yaw_deg, progress_points,
                         lookahead_mm, tracking_v_cmps):
    if progress_points is None or lookahead_mm <= 0.0:
        return None
    lookahead_points = lookahead_mm / route_reference["path_sample_mm"]
    target_progress = min(
        float(len(route_reference["x"]) - 1),
        progress_points + lookahead_points,
    )
    target_x_mm = _route_interpolate(route_reference["x"], target_progress)
    target_y_mm = _route_interpolate(route_reference["y"], target_progress)
    target_yaw_deg = math.degrees(
        math.atan2(target_y_mm - y_mm, target_x_mm - x_mm)
    )
    error_deg = _wrap_degrees(target_yaw_deg - yaw_deg)
    p_turn_cmps = -(
        tracking_v_cmps / 100.0
    ) * (
        100.0 * tracking_config["wheel_track_mm"] / lookahead_mm
    ) * math.sin(math.radians(error_deg))
    return {
        "error_deg": error_deg,
        "p_turn_cmps": p_turn_cmps,
    }


def _route_segment_at(progress_index, ends, values, default=0.0):
    """Return the compiled run-length segment containing a path sample."""
    if not ends or not values:
        return None, None, float(default)
    sample_index = max(0, int(progress_index))
    for segment_index, (segment_end, value) in enumerate(zip(ends, values)):
        if sample_index <= int(segment_end):
            return segment_index, int(segment_end), float(value)
    last_index = len(values) - 1
    return last_index, int(ends[-1]), float(values[last_index])


def _continuous_angle(previous, current):
    """Unwrap one wrapped degree sample relative to the previous sample."""
    if previous is None:
        return float(current)
    return float(previous) + _wrap_degrees(float(current) - float(previous))


def _dynamic_lookahead_mm(progress_points, lookahead_v_cmps, route_reference,
                          tracking_config):
    path_sample_mm = float(tracking_config["path_sample_mm"])
    base_lookahead_mm = float(lookahead_v_cmps) * 10.0 * float(
        tracking_config["preview_time_s"]
    )
    min_lookahead_mm = float(tracking_config["min_lookahead_mm"])
    curve_min_lookahead_mm = float(
        tracking_config.get("curve_min_lookahead_mm", min_lookahead_mm)
    )
    max_lookahead_mm = float(tracking_config["max_lookahead_mm"])
    base_lookahead_mm = max(
        min_lookahead_mm, min(max_lookahead_mm, base_lookahead_mm)
    )
    curvature = route_reference["curvature"]
    curvature_scale = float(route_reference["curvature_lsb_per_mm"])
    min_progress = max(0.0, progress_points - base_lookahead_mm / path_sample_mm)
    max_progress = min(
        float(len(curvature) - 1),
        progress_points + base_lookahead_mm / path_sample_mm,
    )
    first_index = int(min_progress)
    last_index = int(max_progress)
    if last_index < max_progress and last_index < len(curvature) - 1:
        last_index += 1

    nearest_curve_mm = base_lookahead_mm
    max_abs_curvature = 0.0
    for index in range(first_index, last_index + 1):
        abs_curvature = abs(float(curvature[index]) * curvature_scale)
        if abs_curvature > 1.0 / base_lookahead_mm:
            distance_mm = abs(float(index) - progress_points) * path_sample_mm
            nearest_curve_mm = min(nearest_curve_mm, distance_mm)
            max_abs_curvature = max(max_abs_curvature, abs_curvature)

    if max_abs_curvature <= 0.0:
        return base_lookahead_mm, base_lookahead_mm, None, None
    curve_lookahead_mm = max(curve_min_lookahead_mm, 1.0 / max_abs_curvature)
    influence = max(
        0.0, min(1.0, 1.0 - nearest_curve_mm / base_lookahead_mm)
    )
    lookahead_mm = base_lookahead_mm - influence * (
        base_lookahead_mm - curve_lookahead_mm
    )
    return (
        lookahead_mm,
        base_lookahead_mm,
        curve_lookahead_mm,
        nearest_curve_mm,
    )


def _decode_navigation(payload, count, route_reference, control,
                       tracking_config, offset=NAVIGATION_OFFSET):
    records = []
    route_x = route_reference["x"]
    route_y = route_reference["y"]
    route_yaw = route_reference["yaw_cdeg"]
    route_curvature = route_reference["curvature"]
    route_curvature_derivative = route_reference["curvature_derivative"]
    speed_end = route_reference["speed_end"]
    speed_cmps = route_reference["speed_cmps"]
    acceleration_end = route_reference["acceleration_end"]
    acceleration_cmps2 = route_reference["acceleration_cmps2"]
    curvature_scale = route_reference["curvature_lsb_per_mm"]
    curvature_derivative_scale = route_reference[
        "curvature_derivative_lsb_per_mm2"
    ]
    speed_scale = route_reference["speed_lsb_cmps"]
    acceleration_scale = route_reference["acceleration_lsb_cmps2"]
    control_index = 0
    previous_transform_yaw = None
    previous_actual_yaw = None
    previous_raw_yaw = None
    previous_ideal_yaw = None
    previous_update_seq = None
    for index in range(count):
        fields = NAVIGATION_STRUCT.unpack_from(
            payload, offset + index * NAVIGATION_STRUCT.size
        )
        actual_yaw_cdeg = fields[4]
        if actual_yaw_cdeg <= -18000:
            actual_yaw_cdeg = 18000
        raw_yaw_cdeg = fields[7]
        if raw_yaw_cdeg <= -18000:
            raw_yaw_cdeg = 18000
        actual_yaw_deg = actual_yaw_cdeg / 100.0
        raw_yaw_deg = raw_yaw_cdeg / 100.0
        progress_points = fields[1] / 4.0
        progress_index = min(
            len(route_x) - 1, max(0, int(progress_points + 0.5))
        )
        ideal_x_mm = _route_interpolate(route_x, progress_points)
        ideal_y_mm = _route_interpolate(route_y, progress_points)
        ideal_yaw_deg = _wrap_degrees(
            _route_interpolate_yaw_cdeg(route_yaw, progress_points) / 100.0
        )
        actual_yaw_unwrapped_deg = _continuous_angle(
            previous_actual_yaw, actual_yaw_deg
        )
        raw_yaw_unwrapped_deg = _continuous_angle(
            previous_raw_yaw, raw_yaw_deg
        )
        ideal_yaw_unwrapped_deg = _continuous_angle(
            previous_ideal_yaw, ideal_yaw_deg
        )
        if not control:
            raise ValueError(
                "schema 21 navigation reconstruction requires control records"
            )
        while (
            control_index + 1 < len(control)
            and abs(float(control[control_index + 1]["tick_ms"]) - fields[0])
            <= abs(float(control[control_index]["tick_ms"]) - fields[0])
        ):
            control_index += 1
        control_row = control[control_index]
        lookahead_plan_v_cmps = float(control_row["plan_v_cmps"])
        lookahead_actual_v_cmps = max(
            0.0, float(control_row["actual_avg_cmps"])
        )
        lookahead_speed_cmps = max(
            lookahead_plan_v_cmps, lookahead_actual_v_cmps
        )
        tracking_speed_cmps = lookahead_actual_v_cmps
        curvature_preview_time_s = (
            float(tracking_config.get("curvature_preview_time_s") or 0.0)
        )
        curvature_preview_progress_points = min(
            float(len(route_x) - 1),
            progress_points + (
                lookahead_actual_v_cmps * 10.0 * curvature_preview_time_s /
                route_reference["path_sample_mm"]
            ),
        )
        curvature_preview_per_mm = _route_interpolate(
            route_curvature, curvature_preview_progress_points
        ) * curvature_scale
        curvature_feedforward_yaw_rate_radps = (
            lookahead_actual_v_cmps * 10.0 * curvature_preview_per_mm
        )
        lookahead_control_alignment_age_ms = abs(
            float(control_row["tick_ms"]) - fields[0]
        )
        (
            lookahead_command_mm,
            lookahead_base_mm,
            lookahead_curve_radius_mm,
            lookahead_curve_distance_mm,
        ) = _dynamic_lookahead_mm(
            progress_points,
            lookahead_speed_cmps,
            route_reference,
            tracking_config,
        )
        lookahead_points = (
            lookahead_command_mm / route_reference["path_sample_mm"]
        )
        lookahead_progress_points = min(
            float(len(route_x) - 1), progress_points + lookahead_points
        )
        lookahead_x_mm = _route_interpolate(
            route_x, lookahead_progress_points
        )
        lookahead_y_mm = _route_interpolate(
            route_y, lookahead_progress_points
        )
        lookahead_dx = lookahead_x_mm - fields[2]
        lookahead_dy = lookahead_y_mm - fields[3]
        pure_pursuit_target_yaw_deg = math.degrees(
            math.atan2(lookahead_dy, lookahead_dx)
        )
        pure_pursuit_error_deg = _wrap_degrees(
            pure_pursuit_target_yaw_deg - actual_yaw_deg
        )
        speed_segment_index, speed_segment_end, speed_value = (
            _route_segment_at(progress_index, speed_end, speed_cmps)
        )
        accel_segment_index, accel_segment_end, accel_value = (
            _route_segment_at(progress_index, acceleration_end, acceleration_cmps2)
        )
        target_speed_cmps = speed_value * speed_scale
        reference_acceleration_cmps2 = accel_value * acceleration_scale
        path_yaw_rad = math.radians(ideal_yaw_deg)
        dx = fields[2] - ideal_x_mm
        dy = fields[3] - ideal_y_mm
        tangent_x = math.cos(path_yaw_rad)
        tangent_y = math.sin(path_yaw_rad)
        normal_right_x = -tangent_y
        normal_right_y = tangent_x
        along_track_mm = dx * tangent_x + dy * tangent_y
        cross_track_mm = (
            dx * normal_right_x + dy * normal_right_y
        )
        heading_error_deg = _wrap_degrees(actual_yaw_deg - ideal_yaw_deg)
        heading_correction_radps = 0.0
        cross_track_correction_radps = 0.0
        target_yaw_rate_from_navigation_radps = (
            curvature_feedforward_yaw_rate_radps
        )
        if int(tracking_config.get("algorithm_version", 0)) >= 17:
            correction_speed_mmps = max(
                lookahead_actual_v_cmps * 10.0,
                float(route_reference.get(
                    "min_correction_speed_mmps", 500.0
                )),
            )
            heading_correction_radps = -float(route_reference.get(
                "heading_correction_radps_per_rad", 2.0
            )) * math.radians(heading_error_deg)
            cross_track_correction_radps = -float(route_reference.get(
                "cross_track_correction_mmps_per_mm", 9.0
            )) * cross_track_mm / correction_speed_mmps
            target_yaw_rate_from_navigation_radps = (
                curvature_feedforward_yaw_rate_radps
                + heading_correction_radps
                + cross_track_correction_radps
            )
        transform_yaw_deg = _wrap_degrees(actual_yaw_deg - raw_yaw_deg)
        transform_yaw_unwrapped_deg = _continuous_angle(
            previous_transform_yaw, transform_yaw_deg
        )
        transform_yaw_rad = math.radians(transform_yaw_deg)
        transform_cos = math.cos(transform_yaw_rad)
        transform_sin = math.sin(transform_yaw_rad)
        curvature_per_mm = (
            _route_interpolate(route_curvature, progress_points)
            * curvature_scale
        )
        curvature_derivative_per_mm2 = (
            _route_interpolate(route_curvature_derivative, progress_points)
            * curvature_derivative_scale
        )
        reference_speed_mmps = target_speed_cmps * 10.0
        reference_yaw_rate_radps = reference_speed_mmps * curvature_per_mm
        records.append({
                "timestamp_ms": fields[0],
                "progress_index": progress_index,
                "progress_points": progress_points,
                "actual_x_mm": fields[2],
                "actual_y_mm": fields[3],
                "actual_yaw_deg": actual_yaw_deg,
                "actual_yaw_unwrapped_deg": actual_yaw_unwrapped_deg,
                "ideal_x_mm": ideal_x_mm,
                "ideal_y_mm": ideal_y_mm,
                "ideal_yaw_deg": ideal_yaw_deg,
                "ideal_yaw_unwrapped_deg": ideal_yaw_unwrapped_deg,
                "lookahead_progress_points": lookahead_progress_points,
                "lookahead_x_mm": lookahead_x_mm,
                "lookahead_y_mm": lookahead_y_mm,
                "lookahead_command_mm": lookahead_command_mm,
                "lookahead_base_mm": lookahead_base_mm,
                "lookahead_plan_v_cmps": lookahead_plan_v_cmps,
                "lookahead_speed_cmps": lookahead_speed_cmps,
                "tracking_speed_cmps": tracking_speed_cmps,
                "lookahead_control_alignment_age_ms": (
                    lookahead_control_alignment_age_ms
                ),
                "lookahead_curve_radius_mm": lookahead_curve_radius_mm,
                "lookahead_curve_distance_mm": lookahead_curve_distance_mm,
                "lookahead_distance_mm": math.hypot(
                    lookahead_dx, lookahead_dy
                ),
                "pure_pursuit_target_yaw_deg": pure_pursuit_target_yaw_deg,
                "pure_pursuit_error_deg": pure_pursuit_error_deg,
                "pure_pursuit_error_rad": math.radians(
                    pure_pursuit_error_deg
                ),
                "target_speed_cmps": target_speed_cmps,
                "reference_acceleration_cmps2": reference_acceleration_cmps2,
                "speed_segment_index": speed_segment_index,
                "speed_segment_end_index": speed_segment_end,
                "acceleration_segment_index": accel_segment_index,
                "acceleration_segment_end_index": accel_segment_end,
                "curvature_per_mm": curvature_per_mm,
                "curvature_derivative_per_mm2": curvature_derivative_per_mm2,
                "curvature_radius_mm": (
                    1.0 / abs(curvature_per_mm)
                    if abs(curvature_per_mm) > 1.0e-12 else None
                ),
                "reference_yaw_rate_radps": reference_yaw_rate_radps,
                "reference_yaw_rate_dps": math.degrees(
                    reference_yaw_rate_radps
                ),
                "curvature_preview_time_s": curvature_preview_time_s,
                "curvature_preview_progress_points": (
                    curvature_preview_progress_points
                ),
                "curvature_preview_per_mm": curvature_preview_per_mm,
                "curvature_feedforward_yaw_rate_radps": (
                    curvature_feedforward_yaw_rate_radps
                ),
                "curvature_feedforward_yaw_rate_dps": math.degrees(
                    curvature_feedforward_yaw_rate_radps
                ),
                "target_yaw_rate_from_navigation_radps": (
                    target_yaw_rate_from_navigation_radps
                ),
                "target_yaw_rate_from_navigation_dps": math.degrees(
                    target_yaw_rate_from_navigation_radps
                ),
                "reference_lateral_acceleration_cmps2": (
                    reference_speed_mmps * reference_speed_mmps
                    * curvature_per_mm / 10.0
                ),
                "cross_track_mm": cross_track_mm,
                "along_track_mm": along_track_mm,
                "heading_error_deg": heading_error_deg,
                "heading_error_rad": math.radians(heading_error_deg),
                "map_pose_path_distance_mm": math.hypot(dx, dy),
                "raw_pose_path_distance_mm": math.hypot(
                    fields[5] - ideal_x_mm, fields[6] - ideal_y_mm
                ),
                "raw_x_mm": fields[5],
                "raw_y_mm": fields[6],
                "raw_yaw_deg": raw_yaw_deg,
                "raw_yaw_unwrapped_deg": raw_yaw_unwrapped_deg,
                "map_minus_raw_x_mm": fields[2] - fields[5],
                "map_minus_raw_y_mm": fields[3] - fields[6],
                "map_minus_raw_position_mm": math.hypot(
                    fields[2] - fields[5], fields[3] - fields[6]
                ),
                "localization_update_seq": fields[8],
                "map_from_odom_tx_mm": fields[2] - (
                    transform_cos * fields[5] - transform_sin * fields[6]
                ),
                "map_from_odom_ty_mm": fields[3] - (
                    transform_sin * fields[5] + transform_cos * fields[6]
                ),
                "map_from_odom_yaw_deg": transform_yaw_deg,
                "map_from_odom_yaw_unwrapped_deg": transform_yaw_unwrapped_deg,
                "map_from_odom_translation_norm_mm": math.hypot(
                    fields[2] - (
                        transform_cos * fields[5] - transform_sin * fields[6]
                    ),
                    fields[3] - (
                        transform_sin * fields[5] + transform_cos * fields[6]
                    ),
                ),
                "map_from_odom_transform_changed": (
                    previous_update_seq is not None
                    and fields[8] != previous_update_seq
                ),
                "localization_transaction_index": None,
                "localization_transaction_status": None,
            })
        previous_transform_yaw = transform_yaw_unwrapped_deg
        previous_actual_yaw = actual_yaw_unwrapped_deg
        previous_raw_yaw = raw_yaw_unwrapped_deg
        previous_ideal_yaw = ideal_yaw_unwrapped_deg
        previous_update_seq = fields[8]
    return records


def _decode_vision(payload, count, offset=VISION_OFFSET, frame_seq_anchor=None):
    records = []
    previous_raw_frame_seq = None
    expanded_frame_seq = None
    first_expanded_frame_seq = None
    for index in range(count):
        fields = VISION_STRUCT.unpack_from(
            payload, offset + index * VISION_STRUCT.size
        )
        active_mask = fields[3]
        raw_frame_seq = fields[1]
        if previous_raw_frame_seq is None:
            expanded_frame_seq = raw_frame_seq
            if frame_seq_anchor is not None:
                expanded_frame_seq |= int(frame_seq_anchor) & ~0xFFFF
                if expanded_frame_seq - frame_seq_anchor > 0x8000:
                    expanded_frame_seq -= 0x10000
                elif frame_seq_anchor - expanded_frame_seq > 0x8000:
                    expanded_frame_seq += 0x10000
        else:
            expanded_frame_seq += (raw_frame_seq - previous_raw_frame_seq) & 0xFFFF
        if first_expanded_frame_seq is None:
            first_expanded_frame_seq = expanded_frame_seq
        previous_raw_frame_seq = raw_frame_seq
        diagnostics = fields[4]
        threshold_gray = diagnostics & 0x7F
        peak_count = (diagnostics >> 7) & 0x0F
        white_q4 = (diagnostics >> 11) & 0x0F
        full_coverage = bool(diagnostics & 0x8000)
        line_valid = fields[2] != -32768
        edge_valid = not bool(active_mask & 0x8001)
        selection_ambiguous = bool(
            not line_valid and (full_coverage or peak_count > 0)
        )
        observation_flags = 0x0040
        if line_valid:
            observation_flags |= 0x000B
        if edge_valid:
            observation_flags |= 0x0010
        if full_coverage:
            observation_flags |= 0x0080
        if selection_ambiguous:
            observation_flags |= 0x0100
        if line_valid:
            reject_reason = 0
            reject_reason_name = "accepted"
        elif full_coverage:
            reject_reason = 11
            reject_reason_name = "full_coverage"
        elif selection_ambiguous:
            reject_reason = 10
            reject_reason_name = "selection_ambiguous"
        else:
            reject_reason = 12
            reject_reason_name = "no_candidate"
        white_percent = white_q4 * 100.0 / 15.0
        line_position_mm = fields[2] / 100.0 if line_valid else None
        active_indices = [
            sensor for sensor in range(16) if active_mask & (1 << sensor)
        ]
        active_block_count = sum(
            sensor == 0 or not (active_mask & (1 << (sensor - 1)))
            for sensor in active_indices
        )
        records.append(
            {
                "tick_ms": fields[0],
                "tick_s": fields[0] / 1000.0,
                "frame_seq": expanded_frame_seq,
                "frame_seq_delta": (
                    expanded_frame_seq - first_expanded_frame_seq
                ),
                "line_position_mm": line_position_mm,
                "active_mask": active_mask,
                "active_mask_hex": f"0x{active_mask:04X}",
                "observation_flags": f"0x{observation_flags:04X}",
                "line_valid": line_valid,
                "edge_valid": edge_valid,
                "full_coverage": full_coverage,
                "selection_ambiguous": selection_ambiguous,
                "peak_count": peak_count,
                "white_percent": white_percent,
                "threshold_gray": threshold_gray,
                "active_sensor_count": active_mask.bit_count(),
                "active_first_sensor": (
                    active_indices[0] if active_indices else None
                ),
                "active_last_sensor": (
                    active_indices[-1] if active_indices else None
                ),
                "active_span_sensors": (
                    active_indices[-1] - active_indices[0] + 1
                    if active_indices else 0
                ),
                "active_centroid_sensor": (
                    sum(active_indices) / len(active_indices)
                    if active_indices else None
                ),
                "active_block_count": active_block_count,
                "active_left_half_count": (active_mask & 0x00FF).bit_count(),
                "active_right_half_count": (
                    (active_mask >> 8) & 0x00FF
                ).bit_count(),
                "reject_reason": reject_reason,
                "reject_reason_name": reject_reason_name,
            }
        )
    return records


def _fit_phototube_points(points):
    result = {
        "valid": False,
        "line_slope": 0.0,
        "line_intercept": 0.0,
        "line_center_x_mm": 0.0,
        "line_center_y_mm": 0.0,
        "line_direction_x": 0.0,
        "line_direction_y": 0.0,
        "rms_error_mm": 0.0,
        "max_abs_error_mm": 0.0,
    }
    fit_points = points[-PHOTOTUBE_SCAN_FIT_WINDOW_POINTS:]
    if len(fit_points) < 2:
        return result
    weights = [point["confidence"] / 255.0 for point in fit_points]
    weight_sum = sum(weights)
    if weight_sum < 1.0e-3:
        return result
    mean_x = sum(
        weight * point["x_mm"] for weight, point in zip(weights, fit_points)
    ) / weight_sum
    mean_y = sum(
        weight * point["y_mm"] for weight, point in zip(weights, fit_points)
    ) / weight_sum
    covariance_xx = 0.0
    covariance_xy = 0.0
    covariance_yy = 0.0
    for weight, point in zip(weights, fit_points):
        dx = point["x_mm"] - mean_x
        dy = point["y_mm"] - mean_y
        covariance_xx += weight * dx * dx
        covariance_xy += weight * dx * dy
        covariance_yy += weight * dy * dy
    angle = 0.5 * math.atan2(
        2.0 * covariance_xy, covariance_xx - covariance_yy
    )
    direction_x = math.cos(angle)
    direction_y = math.sin(angle)
    projection_min = math.inf
    projection_max = -math.inf
    rms_sum = 0.0
    max_abs_error = 0.0
    for weight, point in zip(weights, fit_points):
        dx = point["x_mm"] - mean_x
        dy = point["y_mm"] - mean_y
        projection = dx * direction_x + dy * direction_y
        error = -dx * direction_y + dy * direction_x
        projection_min = min(projection_min, projection)
        projection_max = max(projection_max, projection)
        rms_sum += weight * error * error
        max_abs_error = max(max_abs_error, abs(error))
    line_slope = 0.0
    line_intercept = 0.0
    if abs(direction_x) > 1.0e-3:
        line_slope = direction_y / direction_x
        line_intercept = mean_y - line_slope * mean_x
    result.update({
        "valid": (
            projection_max - projection_min >= PHOTOTUBE_SCAN_MIN_FIT_SPAN_MM
        ),
        "line_slope": line_slope,
        "line_intercept": line_intercept,
        "line_center_x_mm": mean_x,
        "line_center_y_mm": mean_y,
        "line_direction_x": direction_x,
        "line_direction_y": direction_y,
        "rms_error_mm": math.sqrt(rms_sum / weight_sum),
        "max_abs_error_mm": max_abs_error,
    })
    return result


def _decode_phototube_scan(payload, localization_config):
    """Decode raw odom points and reconstruct all derivable scan geometry."""
    if PHOTOTUBE_SCAN_REGION_SIZE < PHOTOTUBE_SCAN_HEADER_SIZE:
        return {"valid": False, "points": []}
    fields = PHOTOTUBE_SCAN_HEADER_STRUCT.unpack_from(payload, PHOTOTUBE_SCAN_OFFSET)
    (
        magic,
        version,
        header_size,
        point_count,
        record_size,
        first_frame_seq,
        pose_interpolated_count,
        pose_rejected_count,
        mailbox_skipped_count,
        source_frame_skipped_count,
        source_schedule_drop_count,
        source_schedule_skipped_count,
        max_processing_latency_ms,
        max_capture_to_cloud_latency_ms,
        received_frame_count,
        accepted_frame_count,
        accepted_hit_count,
        spatial_dedup_frame_count,
        travel_reversal_frame_count,
        online_ring_overwrite_point_count,
        empty_hit_frame_count,
        archive_oldest_storage_slot,
        staging_backpressure_count,
        staging_high_watermark,
        pose_wait_count,
        emem_queue_drop_count,
    ) = fields
    if magic != PHOTOTUBE_SCAN_MAGIC:
        raise ValueError("missing schema-21 phototube scan archive")
    if version != PHOTOTUBE_SCAN_VERSION:
        raise ValueError(
            "phototube scan protocol mismatch: "
            f"image={version}, decoder={PHOTOTUBE_SCAN_VERSION}"
        )
    if (
        header_size != PHOTOTUBE_SCAN_HEADER_SIZE
        or record_size != PHOTOTUBE_SCAN_RECORD_SIZE
        or point_count > PHOTOTUBE_SCAN_CAPACITY
        or archive_oldest_storage_slot >= PHOTOTUBE_SCAN_CAPACITY
        or (point_count < PHOTOTUBE_SCAN_CAPACITY and archive_oldest_storage_slot)
        or header_size + point_count * record_size > PHOTOTUBE_SCAN_REGION_SIZE
    ):
        raise ValueError("invalid schema-21 phototube scan archive layout")
    points = []
    records_offset = PHOTOTUBE_SCAN_OFFSET + header_size
    oldest_slot = (
        archive_oldest_storage_slot
        if point_count == PHOTOTUBE_SCAN_CAPACITY else 0
    )
    storage_slots = [
        (oldest_slot + index) % PHOTOTUBE_SCAN_CAPACITY
        for index in range(point_count)
    ]
    sensor_forward_mm = float(localization_config.get("sensor_forward_mm", 0.0))
    mounting_rad = 0.0
    previous_capture_raw = None
    expanded_capture_ms = None
    previous_frame_delta_raw = None
    expanded_frame_delta = None
    previous_travel_raw = None
    expanded_travel_mm = None
    for retained_index, storage_slot in enumerate(storage_slots):
        record = PHOTOTUBE_SCAN_STRUCT.unpack_from(
            payload, records_offset + storage_slot * record_size
        )
        frame_delta_raw = record[0]
        if previous_frame_delta_raw is None:
            expanded_frame_delta = frame_delta_raw
        else:
            expanded_frame_delta += (
                frame_delta_raw - previous_frame_delta_raw
            ) & 0xFFFF
        previous_frame_delta_raw = frame_delta_raw
        travel_raw = record[6]
        if previous_travel_raw is None:
            expanded_travel_mm = travel_raw
        else:
            expanded_travel_mm += (travel_raw - previous_travel_raw) & 0xFFFF
        previous_travel_raw = travel_raw
        capture_raw = record[1]
        if previous_capture_raw is None:
            expanded_capture_ms = capture_raw
        else:
            expanded_capture_ms += (capture_raw - previous_capture_raw) & 0xFFFF
        previous_capture_raw = capture_raw
        point_flags = record[8]
        if point_flags & SCAN_HIT_BOUNDARY:
            point_type = "boundary"
        elif point_flags & SCAN_HIT_FULL_COVERAGE:
            point_type = "full_coverage"
        elif point_flags & SCAN_HIT_MULTIPLE:
            point_type = "multiple"
        elif point_flags & SCAN_HIT_RUN_SAMPLE:
            point_type = "run_sample"
        elif point_flags & SCAN_HIT_SINGLE:
            point_type = "single"
        else:
            point_type = "unknown"
        lateral_mm = record[4] / 10.0
        raw_yaw_deg = record[5] / 100.0
        raw_yaw_rad = math.radians(raw_yaw_deg)
        body_x_mm = sensor_forward_mm - math.sin(mounting_rad) * lateral_mm
        body_y_mm = math.cos(mounting_rad) * lateral_mm
        cos_yaw = math.cos(raw_yaw_rad)
        sin_yaw = math.sin(raw_yaw_rad)
        raw_axle_x_mm = (
            record[2] - body_x_mm * cos_yaw + body_y_mm * sin_yaw
        )
        raw_axle_y_mm = (
            record[3] - body_x_mm * sin_yaw - body_y_mm * cos_yaw
        )
        points.append(
            {
                "coordinate_frame": "raw_odom",
                "retained_index": retained_index,
                "storage_slot": storage_slot,
                "frame_seq": first_frame_seq + expanded_frame_delta,
                "frame_seq_delta": expanded_frame_delta,
                "frame_seq_delta_raw": frame_delta_raw,
                "capture_ms": expanded_capture_ms,
                "capture_ms_raw": capture_raw,
                "x_mm": record[2],
                "y_mm": record[3],
                "lateral_mm": lateral_mm,
                "raw_yaw_deg": raw_yaw_deg,
                "raw_axle_x_mm": raw_axle_x_mm,
                "raw_axle_y_mm": raw_axle_y_mm,
                "confidence": record[7],
                "point_weight": record[7] / 255.0,
                "point_flags": point_flags,
                "point_type": point_type,
                "edge_hit": bool(point_flags & SCAN_HIT_EDGE),
                "centerline_hit": bool(point_flags & SCAN_HIT_CENTER),
                "boundary_hit": bool(point_flags & SCAN_HIT_BOUNDARY),
                "travel_distance_mod_mm": travel_raw,
                "travel_distance_mm": expanded_travel_mm,
                "capture_time_s": expanded_capture_ms / 1000.0,
                "travel_distance_m": expanded_travel_mm / 1000.0,
                "map_x_mm": None,
                "map_y_mm": None,
                "map_axle_x_mm": None,
                "map_axle_y_mm": None,
                "map_yaw_deg": None,
                "final_map_x_mm": None,
                "final_map_y_mm": None,
                "final_map_axle_x_mm": None,
                "final_map_axle_y_mm": None,
                "final_map_yaw_deg": None,
                "map_transform_timestamp_ms": None,
                "map_localization_update_seq": None,
                "map_transform_source": None,
                "map_transform_time_boundary_ambiguous": False,
            }
        )
    geometry = _fit_phototube_points(points)
    x_values = [point["x_mm"] for point in points]
    retained_frames = sorted({point["frame_seq"] for point in points})
    retained_missing_frame_span = sum(
        max(0, current - previous - 1)
        for previous, current in zip(retained_frames, retained_frames[1:])
    )
    point_type_counts = {}
    frame_weight_sums = {}
    for point in points:
        point_type_counts[point["point_type"]] = (
            point_type_counts.get(point["point_type"], 0) + 1
        )
        frame_weight_sums[point["frame_seq"]] = (
            frame_weight_sums.get(point["frame_seq"], 0.0)
            + point["point_weight"]
        )
    retained_weights = list(frame_weight_sums.values())
    capture_values = [point["capture_ms"] for point in points]
    travel_values = [point["travel_distance_mm"] for point in points]
    return {
        **geometry,
        "coordinate_frame": "raw_odom",
        "version": version,
        "point_count": point_count,
        "archive_capacity": PHOTOTUBE_SCAN_CAPACITY,
        "archive_record_size": PHOTOTUBE_SCAN_RECORD_SIZE,
        "archive_fill_ratio": point_count / float(PHOTOTUBE_SCAN_CAPACITY),
        "archive_oldest_storage_slot": oldest_slot,
        "first_frame_seq": first_frame_seq,
        "last_frame_seq": points[-1]["frame_seq"] if points else first_frame_seq,
        "origin_x_mm": 0.0,
        "origin_y_mm": 0.0,
        "origin_yaw_rad": 0.0,
        "x_min_mm": min(x_values) if x_values else 0.0,
        "x_max_mm": max(x_values) if x_values else 0.0,
        "capture_start_ms": min(capture_values) if capture_values else None,
        "capture_end_ms": max(capture_values) if capture_values else None,
        "capture_span_ms": (
            max(capture_values) - min(capture_values)
            if capture_values else 0
        ),
        "travel_start_mm": min(travel_values) if travel_values else None,
        "travel_end_mm": max(travel_values) if travel_values else None,
        "retained_travel_span_mm": (
            max(travel_values) - min(travel_values)
            if travel_values else 0
        ),
        "travel_distance_unwrapped_within_retained_window": True,
        "travel_distance_unwrap_origin_mod_mm": (
            travel_values[0] if travel_values else None
        ),
        "retained_unique_frame_count": len(retained_frames),
        "retained_frame_gap_count": sum(
            current > previous + 1
            for previous, current in zip(retained_frames, retained_frames[1:])
        ),
        "retained_missing_frame_span": retained_missing_frame_span,
        "retained_point_type_counts": point_type_counts,
        "retained_edge_hit_count": sum(point["edge_hit"] for point in points),
        "retained_centerline_hit_count": sum(
            point["centerline_hit"] for point in points
        ),
        "retained_boundary_hit_count": sum(
            point["boundary_hit"] for point in points
        ),
        "retained_point_weight_sum": sum(
            point["point_weight"] for point in points
        ),
        "retained_frame_weight_sum_min": (
            min(retained_weights) if retained_weights else None
        ),
        "retained_frame_weight_sum_max": (
            max(retained_weights) if retained_weights else None
        ),
        "retained_frame_weight_sum_mean": (
            sum(retained_weights) / len(retained_weights)
            if retained_weights else None
        ),
        "pose_interpolated_count": pose_interpolated_count,
        "pose_rejected_count": pose_rejected_count,
        "mailbox_skipped_count": mailbox_skipped_count,
        "source_frame_skipped_count": source_frame_skipped_count,
        "source_schedule_drop_count": source_schedule_drop_count,
        "source_schedule_skipped_count": source_schedule_skipped_count,
        "max_processing_latency_ms": max_processing_latency_ms,
        "max_capture_to_cloud_latency_ms": max_capture_to_cloud_latency_ms,
        "received_frame_count": received_frame_count,
        "accepted_frame_count": accepted_frame_count,
        "accepted_hit_count": accepted_hit_count,
        "spatial_dedup_frame_count": spatial_dedup_frame_count,
        "travel_reversal_frame_count": travel_reversal_frame_count,
        "online_ring_overwrite_point_count": online_ring_overwrite_point_count,
        "staging_backpressure_count": staging_backpressure_count,
        "staging_high_watermark": staging_high_watermark,
        "pose_wait_count": pose_wait_count,
        "emem_queue_drop_count": emem_queue_drop_count,
        "empty_hit_frame_count": empty_hit_frame_count,
        "points": points,
    }


def _decode_events(payload, count, offset=EVENT_OFFSET):
    records = []
    for index in range(count):
        fields = EVENT_STRUCT.unpack_from(payload, offset + index * EVENT_STRUCT.size)
        records.append({
            "timestamp_ms": fields[0],
            "type": fields[1],
            "type_name": EVENT_TYPE_NAMES.get(fields[1], f"unknown_{fields[1]}"),
            "source": fields[2],
            "code": fields[3],
            "value0": fields[4],
            "value1": fields[5],
        })
    return records


def _decode_localization(payload, count, route_reference,
                         offset=LOCALIZATION_OFFSET, overwrite_count=0):
    records = []
    oldest_slot = (
        overwrite_count % LOCALIZATION_CAPACITY
        if count == LOCALIZATION_CAPACITY else 0
    )
    storage_slots = [
        (oldest_slot + index) % LOCALIZATION_CAPACITY
        for index in range(count)
    ]
    for retained_index, storage_slot in enumerate(storage_slots):
        fields = LOCALIZATION_STRUCT.unpack_from(
            payload, offset + storage_slot * LOCALIZATION_STRUCT.size
        )
        t_before_tx = float(fields[2])
        t_before_ty = float(fields[3])
        t_before_yaw_deg = fields[8] / 100.0
        delta_x = float(fields[9])
        delta_y = float(fields[10])
        delta_yaw_deg = fields[11] / 100.0
        raw_x = float(fields[12])
        raw_y = float(fields[13])
        raw_yaw_deg = fields[14] / 100.0
        status = fields[20]
        candidate_t_after_yaw_deg = _wrap_degrees(
            t_before_yaw_deg + delta_yaw_deg
        )
        before_yaw_rad = math.radians(t_before_yaw_deg)
        candidate_after_yaw_rad = math.radians(candidate_t_after_yaw_deg)
        before_cos = math.cos(before_yaw_rad)
        before_sin = math.sin(before_yaw_rad)
        candidate_after_cos = math.cos(candidate_after_yaw_rad)
        candidate_after_sin = math.sin(candidate_after_yaw_rad)
        candidate_t_after_tx = (
            t_before_tx + before_cos * delta_x - before_sin * delta_y
        )
        candidate_t_after_ty = (
            t_before_ty + before_sin * delta_x + before_cos * delta_y
        )
        map_before_x = t_before_tx + before_cos * raw_x - before_sin * raw_y
        map_before_y = t_before_ty + before_sin * raw_x + before_cos * raw_y
        candidate_map_after_x = (
            candidate_t_after_tx
            + candidate_after_cos * raw_x
            - candidate_after_sin * raw_y
        )
        candidate_map_after_y = (
            candidate_t_after_ty
            + candidate_after_sin * raw_x
            + candidate_after_cos * raw_y
        )
        map_before_yaw_deg = _wrap_degrees(raw_yaw_deg + t_before_yaw_deg)
        candidate_map_after_yaw_deg = _wrap_degrees(
            raw_yaw_deg + candidate_t_after_yaw_deg
        )
        progress_before = fields[15] / 4.0
        candidate_progress_after = (
            None if fields[16] == 0xFFFF else fields[16] / 4.0
        )
        map_before_route_error = _route_pose_errors(
            route_reference,
            map_before_x,
            map_before_y,
            map_before_yaw_deg,
            progress_before,
        )
        candidate_route_error = _route_pose_errors(
            route_reference,
            candidate_map_after_x,
            candidate_map_after_y,
            candidate_map_after_yaw_deg,
            candidate_progress_after,
        )
        if status:
            t_after_tx = candidate_t_after_tx
            t_after_ty = candidate_t_after_ty
            t_after_yaw_deg = candidate_t_after_yaw_deg
            map_after_x = candidate_map_after_x
            map_after_y = candidate_map_after_y
            map_after_yaw_deg = candidate_map_after_yaw_deg
            progress_after = candidate_progress_after
            map_after_route_error = candidate_route_error
        else:
            # A rejected transaction does not mutate localization state.  Its
            # stored delta/progress describe the refused candidate only.
            t_after_tx = t_before_tx
            t_after_ty = t_before_ty
            t_after_yaw_deg = t_before_yaw_deg
            map_after_x = map_before_x
            map_after_y = map_before_y
            map_after_yaw_deg = map_before_yaw_deg
            progress_after = progress_before
            map_after_route_error = map_before_route_error
        match_type = fields[7]
        reject_reason = fields[21]
        timestamp_raw_ms = fields[0]
        timestamp_ms = timestamp_raw_ms
        delta_translation_mm = math.hypot(delta_x, delta_y)
        map_pose_jump_mm = math.hypot(
            map_after_x - map_before_x, map_after_y - map_before_y
        )
        progress_delta_points = (
            progress_after - progress_before
            if progress_after is not None else None
        )
        candidate_map_pose_jump_mm = math.hypot(
            candidate_map_after_x - map_before_x,
            candidate_map_after_y - map_before_y,
        )
        candidate_progress_delta_points = (
            candidate_progress_after - progress_before
            if candidate_progress_after is not None else None
        )
        candidate_transform_valid = bool(
            match_type != 0 and (
                fields[19] > 0
                or abs(delta_x) > 0.0
                or abs(delta_y) > 0.0
                or abs(delta_yaw_deg) > 0.0
                or abs(fields[17]) > 0
                or abs(fields[18]) > 0
            )
        )
        occurrence = fields[6]
        feature_x = None
        feature_y = None
        if occurrence < len(route_reference["scan_x"]):
            feature_x = float(route_reference["scan_x"][occurrence])
            feature_y = float(route_reference["scan_y"][occurrence])
        records.append({
            "transaction_index": overwrite_count + retained_index,
            "retained_index": retained_index,
            "storage_slot": storage_slot,
            "timestamp_ms": timestamp_ms,
            "timestamp_raw_ms": timestamp_raw_ms,
            "capture_frame_seq": fields[1],
            "travel_distance_mm": fields[4],
            "localization_update_seq": fields[5],
            "route_feature_occurrence": fields[6],
            "status": status,
            "status_name": "accepted" if status else "rejected",
            "match_type": match_type,
            "match_type_name": SCAN_MATCH_TYPE_NAMES.get(
                match_type, "none" if match_type == 0 else f"unknown_{match_type}"
            ),
            "reject_reason": reject_reason,
            "reject_reason_name": (
                "none" if reject_reason == 0 else
                SCAN_MATCH_REJECT_NAMES.get(
                    reject_reason, f"unknown_{reject_reason}"
                )
            ),
            "t_before_tx_mm": t_before_tx,
            "t_before_ty_mm": t_before_ty,
            "t_before_yaw_deg": t_before_yaw_deg,
            "delta_odom_dx_mm": delta_x,
            "delta_odom_dy_mm": delta_y,
            "delta_odom_dyaw_deg": delta_yaw_deg,
            "raw_x_mm": raw_x,
            "raw_y_mm": raw_y,
            "raw_yaw_deg": raw_yaw_deg,
            "t_after_tx_mm": t_after_tx,
            "t_after_ty_mm": t_after_ty,
            "t_after_yaw_deg": t_after_yaw_deg,
            "candidate_t_after_tx_mm": candidate_t_after_tx,
            "candidate_t_after_ty_mm": candidate_t_after_ty,
            "candidate_t_after_yaw_deg": candidate_t_after_yaw_deg,
            "map_before_x_mm": map_before_x,
            "map_before_y_mm": map_before_y,
            "map_before_yaw_deg": map_before_yaw_deg,
            "map_after_x_mm": map_after_x,
            "map_after_y_mm": map_after_y,
            "map_after_yaw_deg": map_after_yaw_deg,
            "candidate_map_after_x_mm": candidate_map_after_x,
            "candidate_map_after_y_mm": candidate_map_after_y,
            "candidate_map_after_yaw_deg": candidate_map_after_yaw_deg,
            "progress_before_points": progress_before,
            "progress_after_points": progress_after,
            "candidate_progress_after_points": candidate_progress_after,
            "rms_before_mm": fields[17] / 10.0,
            "rms_after_mm": fields[18] / 10.0,
            "inlier_count": fields[19],
            "delta_odom_translation_mm": delta_translation_mm,
            "delta_odom_yaw_abs_deg": abs(_wrap_degrees(delta_yaw_deg)),
            "map_pose_jump_mm": map_pose_jump_mm,
            "map_pose_jump_yaw_deg": _wrap_degrees(
                map_after_yaw_deg - map_before_yaw_deg
            ),
            "progress_delta_points": progress_delta_points,
            "progress_delta_mm": (
                progress_delta_points * route_reference["path_sample_mm"]
                if progress_delta_points is not None else None
            ),
            "candidate_map_pose_jump_mm": candidate_map_pose_jump_mm,
            "candidate_map_pose_jump_yaw_deg": _wrap_degrees(
                candidate_map_after_yaw_deg - map_before_yaw_deg
            ),
            "candidate_progress_delta_points": candidate_progress_delta_points,
            "candidate_progress_delta_mm": (
                candidate_progress_delta_points * route_reference["path_sample_mm"]
                if candidate_progress_delta_points is not None else None
            ),
            "candidate_transform_valid": candidate_transform_valid,
            "transform_changed": bool(
                status and (
                    delta_translation_mm > 0.0
                    or abs(delta_yaw_deg) > 0.0
                )
            ),
            "map_before_cross_track_mm": (
                map_before_route_error["cross_track_mm"]
                if map_before_route_error is not None else None
            ),
            "map_before_along_track_mm": (
                map_before_route_error["along_track_mm"]
                if map_before_route_error is not None else None
            ),
            "map_before_heading_error_deg": (
                map_before_route_error["heading_error_deg"]
                if map_before_route_error is not None else None
            ),
            "map_before_path_distance_mm": (
                map_before_route_error["path_distance_mm"]
                if map_before_route_error is not None else None
            ),
            "candidate_cross_track_mm": (
                candidate_route_error["cross_track_mm"]
                if candidate_route_error is not None else None
            ),
            "candidate_along_track_mm": (
                candidate_route_error["along_track_mm"]
                if candidate_route_error is not None else None
            ),
            "candidate_heading_error_deg": (
                candidate_route_error["heading_error_deg"]
                if candidate_route_error is not None else None
            ),
            "candidate_path_distance_mm": (
                candidate_route_error["path_distance_mm"]
                if candidate_route_error is not None else None
            ),
            "map_after_cross_track_mm": (
                map_after_route_error["cross_track_mm"]
                if map_after_route_error is not None else None
            ),
            "map_after_along_track_mm": (
                map_after_route_error["along_track_mm"]
                if map_after_route_error is not None else None
            ),
            "map_after_heading_error_deg": (
                map_after_route_error["heading_error_deg"]
                if map_after_route_error is not None else None
            ),
            "map_after_path_distance_mm": (
                map_after_route_error["path_distance_mm"]
                if map_after_route_error is not None else None
            ),
            "candidate_cross_track_delta_mm": (
                candidate_route_error["cross_track_mm"]
                - map_before_route_error["cross_track_mm"]
                if candidate_route_error is not None
                and map_before_route_error is not None else None
            ),
            "candidate_along_track_delta_mm": (
                candidate_route_error["along_track_mm"]
                - map_before_route_error["along_track_mm"]
                if candidate_route_error is not None
                and map_before_route_error is not None else None
            ),
            "candidate_heading_error_delta_deg": (
                _wrap_degrees(
                    candidate_route_error["heading_error_deg"]
                    - map_before_route_error["heading_error_deg"]
                )
                if candidate_route_error is not None
                and map_before_route_error is not None else None
            ),
            "cross_track_delta_mm": (
                map_after_route_error["cross_track_mm"]
                - map_before_route_error["cross_track_mm"]
                if map_after_route_error is not None
                and map_before_route_error is not None else None
            ),
            "along_track_delta_mm": (
                map_after_route_error["along_track_mm"]
                - map_before_route_error["along_track_mm"]
                if map_after_route_error is not None
                and map_before_route_error is not None else None
            ),
            "heading_error_delta_deg": (
                _wrap_degrees(
                    map_after_route_error["heading_error_deg"]
                    - map_before_route_error["heading_error_deg"]
                )
                if map_after_route_error is not None
                and map_before_route_error is not None else None
            ),
            "feature_x_mm": feature_x,
            "feature_y_mm": feature_y,
            "map_before_to_feature_distance_mm": (
                math.hypot(map_before_x - feature_x, map_before_y - feature_y)
                if feature_x is not None else None
            ),
            "map_after_to_feature_distance_mm": (
                math.hypot(map_after_x - feature_x, map_after_y - feature_y)
                if feature_x is not None else None
            ),
            "candidate_map_after_to_feature_distance_mm": (
                math.hypot(
                    candidate_map_after_x - feature_x,
                    candidate_map_after_y - feature_y,
                )
                if feature_x is not None else None
            ),
        })
    return records


def _attach_localization_to_navigation(navigation, transactions,
                                       route_reference, tracking_config):
    navigation_index = 0
    for tx in transactions:
        while (
            navigation_index + 1 < len(navigation)
            and abs(
                navigation[navigation_index + 1]["timestamp_ms"]
                - tx["timestamp_ms"]
            ) <= abs(
                navigation[navigation_index]["timestamp_ms"]
                - tx["timestamp_ms"]
            )
        ):
            navigation_index += 1
        nav_row = navigation[navigation_index] if navigation else None
        if nav_row is None:
            before_pp = None
            candidate_pp = None
            after_pp = None
            alignment_age_ms = None
        else:
            alignment_age_ms = abs(
                nav_row["timestamp_ms"] - tx["timestamp_ms"]
            )
            lookahead_mm = nav_row["lookahead_command_mm"]
            speed_cmps = nav_row["tracking_speed_cmps"]
            before_pp = _pure_pursuit_p_term(
                route_reference, tracking_config,
                tx["map_before_x_mm"], tx["map_before_y_mm"],
                tx["map_before_yaw_deg"], tx["progress_before_points"],
                lookahead_mm, speed_cmps,
            )
            candidate_pp = _pure_pursuit_p_term(
                route_reference, tracking_config,
                tx["candidate_map_after_x_mm"],
                tx["candidate_map_after_y_mm"],
                tx["candidate_map_after_yaw_deg"],
                tx["candidate_progress_after_points"],
                lookahead_mm, speed_cmps,
            )
            after_pp = candidate_pp if tx["status"] else before_pp
        tx["map_before_pp_error_deg"] = (
            before_pp["error_deg"] if before_pp is not None else None
        )
        tx["candidate_pp_error_deg"] = (
            candidate_pp["error_deg"] if candidate_pp is not None else None
        )
        tx["map_after_pp_error_deg"] = (
            after_pp["error_deg"] if after_pp is not None else None
        )
        tx["map_before_pp_p_turn_cmps"] = (
            before_pp["p_turn_cmps"] if before_pp is not None else None
        )
        tx["candidate_pp_p_turn_cmps"] = (
            candidate_pp["p_turn_cmps"] if candidate_pp is not None else None
        )
        tx["map_after_pp_p_turn_cmps"] = (
            after_pp["p_turn_cmps"] if after_pp is not None else None
        )
        tx["candidate_pp_p_turn_delta_cmps"] = (
            candidate_pp["p_turn_cmps"] - before_pp["p_turn_cmps"]
            if candidate_pp is not None and before_pp is not None else None
        )
        tx["pp_p_turn_delta_cmps"] = (
            after_pp["p_turn_cmps"] - before_pp["p_turn_cmps"]
            if after_pp is not None and before_pp is not None else None
        )
        tx["localization_control_alignment_age_ms"] = alignment_age_ms

    by_seq = {
        tx["localization_update_seq"]: tx
        for tx in transactions
        if tx.get("status_name") == "accepted"
    }
    mapping = {
        "localization_transaction_index": "transaction_index",
        "localization_transaction_status": "status_name",
        "localization_commit_timestamp_ms": "timestamp_ms",
        "localization_capture_frame_seq": "capture_frame_seq",
        "localization_feature_occurrence": "route_feature_occurrence",
        "localization_match_type": "match_type_name",
        "localization_t_before_tx_mm": "t_before_tx_mm",
        "localization_t_before_ty_mm": "t_before_ty_mm",
        "localization_t_before_yaw_deg": "t_before_yaw_deg",
        "localization_delta_odom_dx_mm": "delta_odom_dx_mm",
        "localization_delta_odom_dy_mm": "delta_odom_dy_mm",
        "localization_delta_odom_dyaw_deg": "delta_odom_dyaw_deg",
        "localization_t_after_tx_mm": "t_after_tx_mm",
        "localization_t_after_ty_mm": "t_after_ty_mm",
        "localization_t_after_yaw_deg": "t_after_yaw_deg",
        "localization_map_before_x_mm": "map_before_x_mm",
        "localization_map_before_y_mm": "map_before_y_mm",
        "localization_map_before_yaw_deg": "map_before_yaw_deg",
        "localization_map_after_x_mm": "map_after_x_mm",
        "localization_map_after_y_mm": "map_after_y_mm",
        "localization_map_after_yaw_deg": "map_after_yaw_deg",
        "localization_progress_before_points": "progress_before_points",
        "localization_progress_after_points": "progress_after_points",
        "localization_map_before_cross_track_mm": "map_before_cross_track_mm",
        "localization_candidate_cross_track_mm": "candidate_cross_track_mm",
        "localization_map_after_cross_track_mm": "map_after_cross_track_mm",
        "localization_cross_track_delta_mm": "cross_track_delta_mm",
        "localization_map_before_heading_error_deg": (
            "map_before_heading_error_deg"
        ),
        "localization_candidate_heading_error_deg": (
            "candidate_heading_error_deg"
        ),
        "localization_map_after_heading_error_deg": (
            "map_after_heading_error_deg"
        ),
        "localization_map_before_pp_p_turn_cmps": (
            "map_before_pp_p_turn_cmps"
        ),
        "localization_candidate_pp_p_turn_cmps": (
            "candidate_pp_p_turn_cmps"
        ),
        "localization_map_after_pp_p_turn_cmps": (
            "map_after_pp_p_turn_cmps"
        ),
        "localization_pp_p_turn_delta_cmps": "pp_p_turn_delta_cmps",
        "localization_rms_before_mm": "rms_before_mm",
        "localization_rms_after_mm": "rms_after_mm",
        "localization_inlier_count": "inlier_count",
    }
    for row in navigation:
        for name in mapping:
            row[name] = None
        tx = by_seq.get(row.get("localization_update_seq"))
        if tx is None:
            continue
        for output_name, source_name in mapping.items():
            row[output_name] = tx.get(source_name)
    return navigation


def _quaternion_to_euler_deg(q0, q1, q2, q3):
    norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    if norm <= 1e-6 or not math.isfinite(norm):
        return None, None, None, norm
    q0, q1, q2, q3 = (value / norm for value in (q0, q1, q2, q3))
    roll = math.degrees(math.atan2(
        2.0 * (q0 * q1 + q2 * q3),
        1.0 - 2.0 * (q1 * q1 + q2 * q2),
    ))
    sinp = 2.0 * (q0 * q2 - q3 * q1)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))
    yaw = math.degrees(math.atan2(
        2.0 * (q0 * q3 + q1 * q2),
        1.0 - 2.0 * (q2 * q2 + q3 * q3),
    ))
    return roll, pitch, yaw, norm


def _decode_imu(payload, count, offset=IMU_OFFSET):
    records = []
    timestamp_4us = 0
    previous_quaternion = None
    for index in range(count):
        fields = IMU_STRUCT.unpack_from(payload, offset + index * IMU_STRUCT.size)
        timestamp_4us += fields[0]
        timestamp_us = timestamp_4us * 4
        acc = [value / 1000.0 for value in fields[1:4]]
        gyro = [value / 10.0 for value in fields[4:7]]
        largest_index = (fields[7] >> 14) & 0x03
        packed = [fields[7] & 0x3FFF, fields[8] & 0x3FFF, fields[9] & 0x3FFF]
        packed = [value - 0x4000 if value & 0x2000 else value for value in packed]
        stored = [value / 11583.0 for value in packed]
        quaternion = []
        stored_index = 0
        omitted = math.sqrt(max(0.0, 1.0 - sum(value * value for value in stored)))
        for component_index in range(4):
            if component_index == largest_index:
                quaternion.append(omitted)
            else:
                quaternion.append(stored[stored_index])
                stored_index += 1
        if (
            previous_quaternion is not None
            and sum(a * b for a, b in zip(previous_quaternion, quaternion)) < 0.0
        ):
            quaternion = [-value for value in quaternion]
        previous_quaternion = quaternion
        roll, pitch, yaw, norm = _quaternion_to_euler_deg(*quaternion)
        acc_norm = math.sqrt(sum(value * value for value in acc))
        gyro_norm = math.sqrt(sum(value * value for value in gyro))
        records.append({
            "timestamp_us": timestamp_us,
            "timestamp_ms": timestamp_us / 1000.0,
            "acc_x_g": acc[0],
            "acc_y_g": acc[1],
            "acc_z_g": acc[2],
            "gyro_x_dps": gyro[0],
            "gyro_y_dps": gyro[1],
            "gyro_z_dps": gyro[2],
            "quaternion_q0": quaternion[0],
            "quaternion_q1": quaternion[1],
            "quaternion_q2": quaternion[2],
            "quaternion_q3": quaternion[3],
            "quaternion_norm": norm,
            "acc_norm_g": acc_norm,
            "gyro_norm_dps": gyro_norm,
            "roll_deg": roll,
            "pitch_deg": pitch,
            "yaw_deg": yaw,
        })
    return records


def _attach_control_context(control, vision, imu, navigation, tracking_config):
    """Recover duplicated sensor context and controller terms without new EMEM bytes."""
    if not control:
        return control

    stream_indices = {"vision": 0, "imu": 0, "navigation": 0}

    def nearest(stream_name, records, timestamp_ms):
        if not records:
            return None, None
        index = min(stream_indices[stream_name], len(records) - 1)
        key = "tick_ms" if stream_name == "vision" else "timestamp_ms"
        while (
            index + 1 < len(records)
            and abs(float(records[index + 1][key]) - timestamp_ms)
            <= abs(float(records[index][key]) - timestamp_ms)
        ):
            index += 1
        stream_indices[stream_name] = index
        age_ms = abs(float(records[index][key]) - timestamp_ms)
        return records[index], age_ms

    wheel_track_mm = float(tracking_config["wheel_track_mm"])
    kd = float(tracking_config["kd_cmps_per_degps_per_speed_scale"])
    gkd = float(tracking_config["gkd_cmps_per_radps"])
    yaw_rate_controller = int(tracking_config["algorithm_version"]) >= 16
    curvature_feedforward_controller = int(
        tracking_config["algorithm_version"]
    ) >= 17

    for record in control:
        timestamp_ms = float(record["tick_ms"])
        speed_scale = max(0.0, float(record["actual_avg_cmps"]) / 100.0)
        if yaw_rate_controller:
            record["yaw_rate_feedback_turn_cmps"] = (
                -gkd * math.radians(float(record["yaw_rate_error_degps"]))
            )
            record["yaw_rate_feedforward_turn_cmps"] = (
                float(record["tracking_turn_cmps"])
                - float(record["yaw_rate_feedback_turn_cmps"])
            )
            target_rate_radps = (
                -float(record["yaw_rate_feedforward_turn_cmps"])
                / (wheel_track_mm / 20.0)
            )
            record["target_yaw_rate_degps"] = math.degrees(
                target_rate_radps
            )
            record["measured_yaw_rate_degps"] = (
                record["target_yaw_rate_degps"]
                - float(record["yaw_rate_error_degps"])
            )
        else:
            record["pd_d_turn_cmps"] = (
                -speed_scale * kd * float(record["yaw_rate_error_degps"])
            )

        imu_row, imu_age_ms = nearest("imu", imu, timestamp_ms)
        if imu_row is not None and imu_age_ms <= 5.0:
            measured_rate_degps = float(imu_row["gyro_z_dps"])
            record["imu_alignment_age_ms"] = imu_age_ms
            if not yaw_rate_controller:
                record["measured_yaw_rate_degps"] = measured_rate_degps
                record["target_yaw_rate_degps"] = (
                    float(record["yaw_rate_error_degps"])
                    + measured_rate_degps
                )
                record["gkd_turn_cmps"] = (
                    gkd * math.radians(measured_rate_degps)
                )
                record["pd_p_turn_inferred_cmps"] = (
                    float(record["tracking_turn_cmps"])
                    - float(record["pd_d_turn_cmps"])
                    - float(record["gkd_turn_cmps"])
                )

        nav_row, nav_age_ms = nearest("navigation", navigation, timestamp_ms)
        if nav_row is not None and nav_age_ms <= 10.0:
            yaw_error_rad = math.radians(float(nav_row["pure_pursuit_error_deg"]))
            lookahead_mm = float(nav_row["lookahead_command_mm"])
            record["pd_p_turn_from_navigation_cmps"] = (
                -speed_scale * (100.0 * wheel_track_mm / lookahead_mm)
                * math.sin(yaw_error_rad)
            )
            if yaw_rate_controller and not curvature_feedforward_controller:
                record["yaw_rate_feedforward_from_navigation_cmps"] = (
                    record["pd_p_turn_from_navigation_cmps"]
                )
                record["pd_p_turn_from_navigation_cmps"] = None
            elif curvature_feedforward_controller:
                record["yaw_rate_feedforward_from_navigation_cmps"] = (
                    -(wheel_track_mm / 20.0) * math.radians(
                        float(nav_row[
                            "target_yaw_rate_from_navigation_dps"
                        ])
                    )
                )
                record["pd_p_turn_from_navigation_cmps"] = None
            record["navigation_alignment_age_ms"] = nav_age_ms

        if (yaw_rate_controller and
                record["yaw_rate_feedforward_from_navigation_cmps"] is not None):
            reconstructed = (
                float(record["yaw_rate_feedforward_from_navigation_cmps"])
                + float(record["yaw_rate_feedback_turn_cmps"])
            )
            record["tracking_turn_reconstructed_cmps"] = reconstructed
            record["tracking_turn_reconstruction_error_cmps"] = (
                reconstructed - float(record["tracking_turn_cmps"])
            )
        elif (
            record["pd_p_turn_from_navigation_cmps"] is not None
            and record["gkd_turn_cmps"] is not None
        ):
            reconstructed = (
                float(record["pd_p_turn_from_navigation_cmps"])
                + float(record["pd_d_turn_cmps"])
                + float(record["gkd_turn_cmps"])
            )
            record["tracking_turn_reconstructed_cmps"] = reconstructed
            record["tracking_turn_reconstruction_error_cmps"] = (
                reconstructed - float(record["tracking_turn_cmps"])
            )

        vision_row, vision_age_ms = nearest("vision", vision, timestamp_ms)
        if vision_row is not None and vision_age_ms <= 5.0:
            record["line_valid"] = bool(vision_row["line_valid"])
            record["line_position_mm"] = vision_row["line_position_mm"]
            record["phototube_alignment_age_ms"] = vision_age_ms
    return control


def _parse_c_integer_array(text, name):
    pattern = re.compile(
        rf"static\s+const\s+(?:signed|unsigned)\s+(?:short|char)\s+"
        rf"{re.escape(name)}\s*\[\s*(?P<count>\d+)\s*\]\s*=\s*"
        rf"\{{(?P<body>.*?)\}};",
        re.DOTALL,
    )
    match = pattern.search(text)
    if not match:
        return None
    values = [int(value) for value in re.findall(r"[-+]?\d+", match.group("body"))]
    return values if len(values) == int(match.group("count")) else None


def _parse_c_float_macro(text, name):
    match = re.search(
        rf"#define\s+{re.escape(name)}\s+\(\s*([-+0-9.eE]+)f?\s*\)",
        text,
    )
    return float(match.group(1)) if match else None


def _parse_c_integer_macro(text, name):
    match = re.search(
        rf"#define\s+{re.escape(name)}\s+\(\s*([-+]?\d+)[uUlL]*\s*\)",
        text,
    )
    return int(match.group(1)) if match else None


def _fnv1a_u8(hash_value, value):
    hash_value ^= int(value) & 0xFF
    return (hash_value * 16777619) & 0xFFFFFFFF


def _fnv1a_u16(hash_value, value):
    hash_value = _fnv1a_u8(hash_value, value)
    return _fnv1a_u8(hash_value, int(value) >> 8)


def _fnv1a_u32(hash_value, value):
    hash_value = _fnv1a_u8(hash_value, value)
    hash_value = _fnv1a_u8(hash_value, int(value) >> 8)
    hash_value = _fnv1a_u8(hash_value, int(value) >> 16)
    return _fnv1a_u8(hash_value, int(value) >> 24)


def _fnv1a_float(hash_value, value):
    bits = struct.unpack("<I", struct.pack("<f", float(value)))[0]
    return _fnv1a_u32(hash_value, bits)


def _read_route_sources(root):
    paths = (
        root / "code" / "nav_path.h",
        root / "code" / "navigation.c",
        root / "code" / "navigation.h",
        root / "code" / "control.h",
        root / "code" / "phototube_array.h",
        root / "code" / "competition_config.h",
        root / "code" / "phototube_scan.h",
        root / "code" / "globals.h",
    )
    return "\n".join(
        path.read_text(encoding="utf-8", errors="replace")
        for path in paths if path.is_file()
    )


def _parse_c_numeric_macro(text, name, integer=False):
    match = re.search(
        rf"#define\s+{re.escape(name)}\s+(?P<value>[^\r\n]*(?:\\\r?\n[^\r\n]*)?)",
        text,
    )
    if not match:
        return None
    expression = re.sub(r"\\\r?\n", " ", match.group("value"))
    # Centralized competition_config.h uses concise trailing /* ... */
    # comments on the same line as many macros.  Strip both C comment styles
    # before validating the numeric expression so comments never affect the
    # route/localization hash parser.
    expression = re.sub(r"/\*.*?\*/", "", expression, flags=re.S)
    expression = expression.split("//", 1)[0].strip()
    number = re.fullmatch(
        r"\(?\s*(?:\(float\)\s*)?([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
        r"[fFuUlL]*\s*\)?",
        expression,
    )
    if number is None:
        return None
    value = float(number.group(1))
    return int(value) if integer else value


def _embedded_route_hash(project_root, arrays, config_hash_out=None,
                         tracking_algorithm_version=None,
                         localization_config_hash_override=None):
    """Reproduce navigation.c's little-endian FNV-1a route fingerprint."""
    root = Path(project_root)
    source = _read_route_sources(root)

    def integer(name):
        value = _parse_c_numeric_macro(source, name, integer=True)
        if value is None:
            raise ValueError(f"cannot parse integer macro {name}")
        return value

    def scalar(name):
        if name == "NAV_SCAN_MATCH_SPATIAL_PROGRESS_LIMIT_MM":
            return scalar("NAV_SCAN_MATCH_TRAVEL_WINDOW_MM") + scalar(
                "NAV_SCAN_MATCH_FEATURE_RADIUS_MM"
            )
        value = _parse_c_numeric_macro(source, name)
        if value is None:
            raise ValueError(f"cannot parse float macro {name}")
        return value

    def hash_localization_config(hash_value):
        hash_value = _fnv1a_u32(hash_value, integer("NAV_SCAN_MATCH_ALGORITHM_VERSION"))
        for name in ("PHOTOTUBE_ARRAY_ALGORITHM_VERSION",
                     "PHOTOTUBE_ARRAY_COUNT",
                     "PHOTOTUBE_ARRAY_Y0_IS_LEFT"):
            hash_value = _fnv1a_u8(hash_value, integer(name))
        hash_value = _fnv1a_u32(hash_value, integer("PHOTOTUBE_ARRAY_SCAN_HZ"))
        hash_value = _fnv1a_u16(
            hash_value, integer("PHOTOTUBE_ARRAY_LINE_THRESHOLD_ADC")
        )
        for name in ("PHOTOTUBE_ARRAY_SPACING_MM",
                     "PHOTOTUBE_ARRAY_FORWARD_MM"):
            hash_value = _fnv1a_float(hash_value, scalar(name))
        hash_value = _fnv1a_u8(hash_value, integer("PHOTOTUBE_SCAN_POSE_MAX_AGE_MS"))
        hash_value = _fnv1a_float(hash_value, scalar("PHOTOTUBE_SCAN_MIN_STEP_MM"))
        hash_value = _fnv1a_u16(hash_value, integer("PHOTOTUBE_SCAN_MAX_POINTS"))
        hash_value = _fnv1a_u8(hash_value, integer("VISION_OBSERVATION_QUEUE_CAPACITY"))
        hash_value = _fnv1a_u8(hash_value, integer("VISION_SCAN_HIT_QUEUE_CAPACITY"))
        hash_value = _fnv1a_u8(hash_value, integer("VISION_SCAN_HIT_MAX_COUNT"))
        hash_value = _fnv1a_float(hash_value, scalar("NAV_SCAN_MATCH_TRAVEL_WINDOW_MM"))
        hash_value = _fnv1a_float(
            hash_value, scalar("NAV_SCAN_MATCH_STRAIGHT_TRAVEL_WINDOW_MM")
        )
        for name in ("NAV_SCAN_MATCH_MIN_POINT_COUNT",
                     "NAV_SCAN_MATCH_MIN_CORNER_SIDE_COUNT",
                     "NAV_SCAN_MATCH_MIN_SIDE_FRAME_COUNT",
                     "NAV_SCAN_MATCH_MIN_RUN_SIDE_FRAME_COUNT",
                     "NAV_SCAN_MATCH_MIN_SHORT_SIDE_CLEAN_POINTS",
                     "NAV_SCAN_MATCH_RUN_FRAME_MAX_GAP"):
            hash_value = _fnv1a_u16(hash_value, integer(name))
        hash_value = _fnv1a_float(
            hash_value, scalar("NAV_SCAN_MATCH_MIN_STRAIGHT_SPAN_MM")
        )
        hash_value = _fnv1a_u16(
            hash_value, integer("NAV_SCAN_MATCH_MIN_STRAIGHT_FRAME_COUNT")
        )
        for name in (
            "NAV_SCAN_MATCH_STRAIGHT_CONFIRM_TRAVEL_MM",
            "NAV_SCAN_MATCH_STRAIGHT_CONFIRM_POSITION_MM",
            "NAV_SCAN_MATCH_STRAIGHT_CONFIRM_YAW_DEG",
            "NAV_SCAN_MATCH_STRAIGHT_MIN_TRANSLATION_MM",
            "NAV_SCAN_MATCH_STRAIGHT_MIN_YAW_DEG",
            "NAV_SCAN_MATCH_STRAIGHT_SIGNIFICANCE_SIGMA",
            "NAV_SCAN_MATCH_MIN_FEATURE_SIDE_SPAN_MM",
            "NAV_SCAN_MATCH_FEATURE_RADIUS_MM",
            "NAV_SCAN_MATCH_FEATURE_LOCAL_RADIUS_MM",
            "NAV_SCAN_MATCH_FEATURE_SEARCH_TRANSLATION_MM",
            "NAV_SCAN_MATCH_FEATURE_SEARCH_YAW_DEG",
            "NAV_SCAN_MATCH_FEATURE_SEARCH_COARSE_STEP_MM",
            "NAV_SCAN_MATCH_FEATURE_SEARCH_COARSE_STEP_DEG",
            "NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_TRANSLATION_MM",
            "NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_YAW_DEG",
            "NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_STEP_MM",
            "NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_STEP_DEG",
        ):
            hash_value = _fnv1a_float(hash_value, scalar(name))
        hash_value = _fnv1a_u16(
            hash_value, integer("NAV_SCAN_MATCH_FEATURE_SEARCH_POINT_LIMIT")
        )
        hash_value = _fnv1a_float(
            hash_value, scalar("NAV_SCAN_MATCH_FEATURE_SEARCH_IDENTITY_MAX_SCORE_MM2")
        )
        for name in ("NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_POINT_COUNT",
                     "NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_FRAME_COUNT"):
            hash_value = _fnv1a_u16(hash_value, integer(name))
        for name in (
            "NAV_SCAN_MATCH_FEATURE_BOUNDARY_FALLBACK_MM",
            "NAV_SCAN_MATCH_SPATIAL_PROGRESS_LIMIT_MM",
            "NAV_SCAN_MATCH_FEATURE_MARGIN_POINTS",
            "NAV_SCAN_MATCH_SEGMENT_MARGIN_MM",
            "NAV_SCAN_MATCH_MAX_STRAIGHT_POINT_RESIDUAL_MM",
            "NAV_SCAN_MATCH_STRAIGHT_ROBUST_MIN_GATE_MM",
            "NAV_SCAN_MATCH_STRAIGHT_ROBUST_MAX_GATE_MM",
            "NAV_SCAN_MATCH_STRAIGHT_ROBUST_MAD_SCALE",
            "NAV_SCAN_MATCH_STRAIGHT_MIN_INLIER_RATIO",
            "NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_MM",
        ):
            hash_value = _fnv1a_float(hash_value, scalar(name))
        hash_value = _fnv1a_u16(
            hash_value, integer("NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_COUNT")
        )
        hash_value = _fnv1a_u16(
            hash_value, integer("NAV_SCAN_MATCH_STRAIGHT_MODE_HALF_WINDOW_BINS")
        )
        hash_value = _fnv1a_u16(
            hash_value, integer("NAV_SCAN_MATCH_STRAIGHT_MODE_MIN_SEPARATION_BINS")
        )
        for name in (
            "NAV_SCAN_MATCH_STRAIGHT_MODE_SEED_GATE_MM",
            "NAV_SCAN_MATCH_STRAIGHT_AMBIGUITY_WEIGHT_RATIO",
            "NAV_SCAN_MATCH_MAX_FEATURE_POINT_RESIDUAL_MM",
            "NAV_SCAN_MATCH_MAX_RMS_MM",
            "NAV_SCAN_MATCH_FEATURE_PROGRESS_AMBIGUITY_MM",
            "NAV_SCAN_MATCH_FEATURE_ASSOCIATION_MARGIN_MM",
            "NAV_SCAN_MATCH_FEATURE_AMBIGUOUS_WEIGHT_RATIO",
            "NAV_SCAN_MATCH_FEATURE_MIN_INLIER_WEIGHT_RATIO",
            "NAV_SCAN_MATCH_MAX_CORRECTION_MM",
            "NAV_SCAN_MATCH_MAX_CORRECTION_YAW_DEG",
            "NAV_SCAN_MATCH_COMPLETE_FEATURE_MAX_YAW_DEG",
            "NAV_SCAN_MATCH_MIN_EFFECTIVE_WEIGHT",
            "NAV_SCAN_MATCH_MIN_SIDE_EFFECTIVE_WEIGHT",
            "NAV_SCAN_MATCH_JUNCTION_MIN_SIDE_EFFECTIVE_WEIGHT",
        ):
            hash_value = _fnv1a_float(hash_value, scalar(name))
        hash_value = _fnv1a_u16(
            hash_value, integer("NAV_SCAN_MATCH_MIN_SUPPORT_SIDE_COUNT")
        )
        hash_value = _fnv1a_float(
            hash_value, scalar("NAV_SCAN_MATCH_MIN_SUPPORT_SIDE_EFFECTIVE_WEIGHT")
        )
        hash_value = _fnv1a_float(
            hash_value, scalar("NAV_SCAN_MATCH_MIN_SUPPORT_SIDE_SPAN_MM")
        )
        for name in (
            "NAV_SCAN_MATCH_MIN_RMS_IMPROVEMENT_MM",
            "NAV_SCAN_MATCH_FEATURE_CONFIRM_TRAVEL_MM",
            "NAV_SCAN_MATCH_FEATURE_CONFIRM_POSITION_MM",
            "NAV_SCAN_MATCH_FEATURE_CONFIRM_YAW_DEG",
            "NAV_SCAN_MATCH_CONFIRM_OVERLAP_MM",
            "NAV_SCAN_MATCH_MAX_PROGRESS_BACKTRACK_MM",
            "NAV_SCAN_MATCH_STRONG_CORRECTION_YAW_DEG",
            "NAV_SCAN_MATCH_STRONG_CORRECTION_POSE_JUMP_MM",
            "NAV_SCAN_MATCH_STRONG_CONFIRM_TRAVEL_MM",
            "NAV_SCAN_MATCH_STRONG_CONFIRM_MAX_RMS_MM",
        ):
            hash_value = _fnv1a_float(hash_value, scalar(name))
        hash_value = _fnv1a_u16(
            hash_value, integer("NAV_SCAN_MATCH_STRONG_CONFIRM_MIN_INLIER_COUNT")
        )
        for name in (
            "NAV_SCAN_MATCH_PATH_CROSS_TRACK_TOLERANCE_MM",
            "NAV_SCAN_MATCH_COMPLETE_FEATURE_PATH_CROSS_TRACK_TOLERANCE_MM",
            "NAV_SCAN_MATCH_MAX_PATH_CROSS_TRACK_AFTER_MM",
        ):
            hash_value = _fnv1a_float(hash_value, scalar(name))
        hash_value = _fnv1a_u8(
            hash_value, integer("NAV_SCAN_CORRECTION_TRANSACTION_VERSION")
        )
        for name in ("NAV_SCAN_COMMIT_HINT_BEHIND_POINTS",
                     "NAV_SCAN_COMMIT_HINT_SEARCH_POINTS"):
            hash_value = _fnv1a_u16(hash_value, integer(name))
        hash_value = _fnv1a_float(
            hash_value, scalar("PHOTOTUBE_ARRAY_BLACK_STOP_DISTANCE_MM")
        )
        return hash_value

    if tracking_algorithm_version is None:
        tracking_algorithm_version = integer(
            "NAV_TRACKING_REFERENCE_ALGORITHM_VERSION"
        )

    hash_value = 2166136261
    hash_value = _fnv1a_u16(hash_value, integer("NAV_SOURCE_ROUTE_INDEX"))
    hash_value = _fnv1a_u16(hash_value, len(arrays["x"]))
    for data, x, y in zip(arrays["yaw_cdeg"], arrays["x"], arrays["y"]):
        hash_value = _fnv1a_u16(hash_value, data)
        hash_value = _fnv1a_u16(hash_value, x)
        hash_value = _fnv1a_u16(hash_value, y)
    hash_value = _fnv1a_float(hash_value, scalar("NAV_REFERENCE_CURVATURE_LSB_PER_MM"))
    for value in arrays["curvature"]:
        hash_value = _fnv1a_u16(hash_value, value)
    hash_value = _fnv1a_float(hash_value, scalar("NAV_REFERENCE_CURVATURE_DERIVATIVE_LSB_PER_MM2"))
    hash_value = _fnv1a_float(hash_value, scalar("NAV_REFERENCE_SPEED_LSB_CMPS"))
    hash_value = _fnv1a_float(hash_value, scalar("NAV_REFERENCE_ACCELERATION_LSB_CMPS2"))
    hash_value = _fnv1a_float(hash_value, scalar("NAV_SPEED_PROFILE_MAX_SPEED_CMPS"))
    hash_value = _fnv1a_float(
        hash_value, scalar("NAV_SPEED_PROFILE_MAX_LATERAL_ACCEL_CMPS2")
    )
    hash_value = _fnv1a_float(hash_value, scalar("NAV_SPEED_PROFILE_MAX_ACCEL_CMPS2"))
    hash_value = _fnv1a_float(hash_value, scalar("NAV_SPEED_PROFILE_MAX_DECEL_CMPS2"))
    hash_value = _fnv1a_float(hash_value, scalar("NAV_SPEED_PROFILE_SAFETY_MARGIN_MM"))
    hash_value = _fnv1a_float(hash_value, scalar("NAV_SPEED_PROFILE_END_CMPS"))
    # Keep the host-side fingerprint in lockstep with navigation.c.  Blind-box
    # routes use this marker to distinguish their offline terminal stop policy
    # from ordinary routes; it is hashed immediately after END_CMPS on the car.
    blind_box = _parse_c_numeric_macro(source, "NAV_ROUTE_BLIND_BOX", integer=True)
    hash_value = _fnv1a_u8(hash_value, 0 if blind_box is None else blind_box)
    for value in arrays["curvature_derivative"]:
        hash_value = _fnv1a_u16(hash_value, value)
    hash_value = _fnv1a_u16(hash_value, len(arrays["acceleration_end"]))
    for end, value in zip(arrays["acceleration_end"], arrays["acceleration_cmps2"]):
        hash_value = _fnv1a_u16(hash_value, end)
        hash_value = _fnv1a_u16(hash_value, value)
    hash_value = _fnv1a_u16(hash_value, len(arrays["speed_end"]))
    for end, value in zip(arrays["speed_end"], arrays["speed_cmps"]):
        hash_value = _fnv1a_u16(hash_value, end)
        hash_value = _fnv1a_u16(hash_value, value)
    hash_value = _fnv1a_u8(hash_value, len(arrays["takeover_lo"]))
    for lo, hi in zip(arrays["takeover_lo"], arrays["takeover_hi"]):
        hash_value = _fnv1a_u16(hash_value, lo)
        hash_value = _fnv1a_u16(hash_value, hi)
    hash_value = _fnv1a_u16(hash_value, len(arrays["scan_progress"]))
    for index, (x, y, progress, kind, support_x, support_y) in enumerate(zip(
        arrays["scan_x"], arrays["scan_y"], arrays["scan_progress"],
        arrays["scan_type"], arrays["scan_support_x"], arrays["scan_support_y"],
    )):
        hash_value = _fnv1a_u16(hash_value, x)
        hash_value = _fnv1a_u16(hash_value, y)
        hash_value = _fnv1a_u16(hash_value, progress)
        hash_value = _fnv1a_u8(hash_value, kind)
        hash_value = _fnv1a_u8(
            hash_value, arrays["scan_corner_calibration_allowed"][index]
        )
        hash_value = _fnv1a_u16(hash_value, support_x)
        hash_value = _fnv1a_u16(hash_value, support_y)
    hash_value = _fnv1a_u16(hash_value, len(arrays["scan_segment_valid"]))
    for valid, calibration_allowed in zip(
        arrays["scan_segment_valid"],
        arrays["scan_segment_calibration_allowed"],
    ):
        hash_value = _fnv1a_u8(hash_value, valid)
        hash_value = _fnv1a_u8(hash_value, calibration_allowed)
    hash_value = _fnv1a_u8(hash_value, integer("NAV_SCAN_MATCH_ALGORITHM_VERSION"))
    hash_value = _fnv1a_u8(hash_value, integer("VISION_OBSERVATION_QUEUE_CAPACITY"))
    hash_value = _fnv1a_u8(hash_value, integer("VISION_SCAN_HIT_QUEUE_CAPACITY"))
    hash_value = _fnv1a_u8(hash_value, integer("VISION_SCAN_HIT_MAX_COUNT"))
    hash_value = _fnv1a_u8(hash_value, integer("PHOTOTUBE_SCAN_POSE_MAX_AGE_MS"))
    for name in ("PHOTOTUBE_SCAN_MIN_STEP_MM",):
        hash_value = _fnv1a_float(hash_value, scalar(name))
    hash_value = _fnv1a_u16(hash_value, integer("PHOTOTUBE_SCAN_MAX_POINTS"))
    hash_value = _fnv1a_float(hash_value, scalar("NAV_SCAN_MATCH_TRAVEL_WINDOW_MM"))
    hash_value = _fnv1a_float(
        hash_value, scalar("NAV_SCAN_MATCH_STRAIGHT_TRAVEL_WINDOW_MM")
    )
    for name in ("NAV_SCAN_MATCH_MIN_CORNER_SIDE_COUNT",
                 "NAV_SCAN_MATCH_MIN_SIDE_FRAME_COUNT",
                 "NAV_SCAN_MATCH_MIN_RUN_SIDE_FRAME_COUNT",
                 "NAV_SCAN_MATCH_MIN_SHORT_SIDE_CLEAN_POINTS",
                 "NAV_SCAN_MATCH_RUN_FRAME_MAX_GAP"):
        hash_value = _fnv1a_u16(hash_value, integer(name))
    hash_value = _fnv1a_float(
        hash_value, scalar("NAV_SCAN_MATCH_MIN_STRAIGHT_SPAN_MM")
    )
    hash_value = _fnv1a_u16(
        hash_value, integer("NAV_SCAN_MATCH_MIN_STRAIGHT_FRAME_COUNT")
    )
    for name in ("NAV_SCAN_MATCH_STRAIGHT_CONFIRM_TRAVEL_MM",
                 "NAV_SCAN_MATCH_STRAIGHT_CONFIRM_POSITION_MM",
                 "NAV_SCAN_MATCH_STRAIGHT_CONFIRM_YAW_DEG",
                 "NAV_SCAN_MATCH_STRAIGHT_MIN_TRANSLATION_MM",
                 "NAV_SCAN_MATCH_STRAIGHT_MIN_YAW_DEG",
                 "NAV_SCAN_MATCH_STRAIGHT_SIGNIFICANCE_SIGMA",
                 "NAV_SCAN_MATCH_MIN_FEATURE_SIDE_SPAN_MM",
                 "NAV_SCAN_MATCH_MAX_RMS_MM",
                 "NAV_SCAN_MATCH_MAX_STRAIGHT_POINT_RESIDUAL_MM",
                 "NAV_SCAN_MATCH_STRAIGHT_ROBUST_MIN_GATE_MM",
                 "NAV_SCAN_MATCH_STRAIGHT_ROBUST_MAX_GATE_MM",
                 "NAV_SCAN_MATCH_STRAIGHT_ROBUST_MAD_SCALE",
                 "NAV_SCAN_MATCH_STRAIGHT_MIN_INLIER_RATIO",
                 "NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_MM"):
        hash_value = _fnv1a_float(hash_value, scalar(name))
    for name in ("NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_COUNT",
                 "NAV_SCAN_MATCH_STRAIGHT_MODE_HALF_WINDOW_BINS",
                 "NAV_SCAN_MATCH_STRAIGHT_MODE_MIN_SEPARATION_BINS"):
        hash_value = _fnv1a_u16(hash_value, integer(name))
    for name in ("NAV_SCAN_MATCH_STRAIGHT_MODE_SEED_GATE_MM",
                 "NAV_SCAN_MATCH_STRAIGHT_AMBIGUITY_WEIGHT_RATIO",
                 "NAV_SCAN_MATCH_MAX_FEATURE_POINT_RESIDUAL_MM",
                 "NAV_SCAN_MATCH_FEATURE_RADIUS_MM",
                 "NAV_SCAN_MATCH_FEATURE_LOCAL_RADIUS_MM",
                 "NAV_SCAN_MATCH_FEATURE_SEARCH_TRANSLATION_MM",
                 "NAV_SCAN_MATCH_FEATURE_SEARCH_YAW_DEG",
                 "NAV_SCAN_MATCH_FEATURE_SEARCH_COARSE_STEP_MM",
                 "NAV_SCAN_MATCH_FEATURE_SEARCH_COARSE_STEP_DEG",
                 "NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_TRANSLATION_MM",
                 "NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_YAW_DEG",
                 "NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_STEP_MM",
                 "NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_STEP_DEG"):
        hash_value = _fnv1a_float(hash_value, scalar(name))
    hash_value = _fnv1a_u16(
        hash_value, integer("NAV_SCAN_MATCH_FEATURE_SEARCH_POINT_LIMIT")
    )
    hash_value = _fnv1a_float(
        hash_value, scalar("NAV_SCAN_MATCH_FEATURE_SEARCH_IDENTITY_MAX_SCORE_MM2")
    )
    for name in ("NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_POINT_COUNT",
                 "NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_FRAME_COUNT"):
        hash_value = _fnv1a_u16(hash_value, integer(name))
    for name in ("NAV_SCAN_MATCH_FEATURE_PROGRESS_AMBIGUITY_MM",
                 "NAV_SCAN_MATCH_FEATURE_ASSOCIATION_MARGIN_MM",
                 "NAV_SCAN_MATCH_FEATURE_AMBIGUOUS_WEIGHT_RATIO",
                 "NAV_SCAN_MATCH_FEATURE_MIN_INLIER_WEIGHT_RATIO",
                 "NAV_SCAN_MATCH_FEATURE_BOUNDARY_FALLBACK_MM",
                 "NAV_SCAN_MATCH_SPATIAL_PROGRESS_LIMIT_MM",
                 "NAV_SCAN_MATCH_FEATURE_MARGIN_POINTS",
                 "NAV_SCAN_MATCH_MIN_EFFECTIVE_WEIGHT",
                 "NAV_SCAN_MATCH_MIN_SIDE_EFFECTIVE_WEIGHT",
                 "NAV_SCAN_MATCH_JUNCTION_MIN_SIDE_EFFECTIVE_WEIGHT"):
        hash_value = _fnv1a_float(hash_value, scalar(name))
    hash_value = _fnv1a_u16(
        hash_value, integer("NAV_SCAN_MATCH_MIN_SUPPORT_SIDE_COUNT")
    )
    hash_value = _fnv1a_float(
        hash_value, scalar("NAV_SCAN_MATCH_MIN_SUPPORT_SIDE_EFFECTIVE_WEIGHT")
    )
    hash_value = _fnv1a_float(
        hash_value, scalar("NAV_SCAN_MATCH_MIN_SUPPORT_SIDE_SPAN_MM")
    )
    for name in ("NAV_SCAN_MATCH_MAX_CORRECTION_MM",
                 "NAV_SCAN_MATCH_MAX_CORRECTION_YAW_DEG",
                 "NAV_SCAN_MATCH_COMPLETE_FEATURE_MAX_YAW_DEG",
                 "NAV_SCAN_MATCH_MIN_RMS_IMPROVEMENT_MM",
                 "NAV_SCAN_MATCH_FEATURE_CONFIRM_TRAVEL_MM",
                 "NAV_SCAN_MATCH_FEATURE_CONFIRM_POSITION_MM",
                  "NAV_SCAN_MATCH_FEATURE_CONFIRM_YAW_DEG",
                  "NAV_SCAN_MATCH_CONFIRM_OVERLAP_MM",
                  "NAV_SCAN_MATCH_MAX_PROGRESS_BACKTRACK_MM",
                  "NAV_SCAN_MATCH_STRONG_CORRECTION_YAW_DEG",
                 "NAV_SCAN_MATCH_STRONG_CORRECTION_POSE_JUMP_MM",
                 "NAV_SCAN_MATCH_STRONG_CONFIRM_TRAVEL_MM",
                 "NAV_SCAN_MATCH_STRONG_CONFIRM_MAX_RMS_MM"):
        hash_value = _fnv1a_float(hash_value, scalar(name))
    hash_value = _fnv1a_u16(
        hash_value, integer("NAV_SCAN_MATCH_STRONG_CONFIRM_MIN_INLIER_COUNT")
    )
    for name in ("NAV_SCAN_MATCH_PATH_CROSS_TRACK_TOLERANCE_MM",
                 "NAV_SCAN_MATCH_COMPLETE_FEATURE_PATH_CROSS_TRACK_TOLERANCE_MM",
                 "NAV_SCAN_MATCH_MAX_PATH_CROSS_TRACK_AFTER_MM"):
        hash_value = _fnv1a_float(hash_value, scalar(name))
    hash_value = _fnv1a_u8(
        hash_value, integer("NAV_SCAN_CORRECTION_TRANSACTION_VERSION")
    )
    for name in ("NAV_SCAN_MATCH_PROGRESS_BEHIND_POINTS",
                 "NAV_SCAN_MATCH_PROGRESS_SEARCH_POINTS",
                 "NAV_SCAN_COMMIT_HINT_BEHIND_POINTS",
                 "NAV_SCAN_COMMIT_HINT_SEARCH_POINTS"):
        hash_value = _fnv1a_u16(hash_value, integer(name))
    localization_config_hash = hash_localization_config(2166136261)
    if config_hash_out is not None:
        config_hash_out.append(localization_config_hash)
    # A frozen log carries the exact localization-config fingerprint that was
    # used by the firmware.  When the route tables are unchanged but the host
    # source has since gained a new compatibility/config field, use that log
    # fingerprint for one explicit route-hash candidate.  The candidate is
    # accepted only if every route byte and every other hashed constant still
    # agrees; this does not weaken route mismatch protection.
    effective_localization_config_hash = (
        localization_config_hash
        if localization_config_hash_override is None
        else int(localization_config_hash_override) & 0xFFFFFFFF
    )
    hash_value = _fnv1a_u32(hash_value, effective_localization_config_hash)
    for name in ("DRIVE_WHEEL_TRACK_MM",
                 "NAV_PP_PREVIEW_TIME_S",
                 "NAV_PP_MIN_LOOKAHEAD_MM",
                 "NAV_PP_MAX_LOOKAHEAD_MM",
                 "NAV_PP_CURVE_MIN_LOOKAHEAD_MM"):
        hash_value = _fnv1a_float(hash_value, scalar(name))
    if tracking_algorithm_version >= 17:
        for name in (
            "NAV_TRACK_CURVATURE_PREVIEW_TIME_S",
            "NAV_TRACK_HEADING_CORRECTION_RADPS_PER_RAD",
            "NAV_TRACK_CROSS_TRACK_CORRECTION_MMPS_PER_MM",
            "NAV_TRACK_MIN_CORRECTION_SPEED_MMPS",
        ):
            hash_value = _fnv1a_float(hash_value, scalar(name))
    hash_value = _fnv1a_float(
        hash_value,
        scalar("NAV_TRACK_KD_CMPS_PER_DEGPS_PER_SPEED_SCALE"),
    )
    hash_value = _fnv1a_float(
        hash_value,
        scalar(
            "NAV_TRACK_YAW_RATE_FB_CMPS_PER_RADPS"
            if tracking_algorithm_version is not None
            and tracking_algorithm_version >= 16
            else "NAV_TRACK_GKD_CMPS_PER_RADPS"
        ),
    )
    hash_value = _fnv1a_float(hash_value, scalar("NAV_TURN_LIMIT_RATIO"))
    for name in ("NAV_TRACK_TARGET_RATE_WINDOW_MS",
                 "NAV_TRACK_TARGET_RATE_WINDOW_SAMPLES"):
        hash_value = _fnv1a_u16(hash_value, integer(name))
    if tracking_algorithm_version >= 11:
        for name in (
            "SLIP_GUARD_MIN_SPEED_CMPS",
            "SLIP_GUARD_MIN_YAW_RATE_DEGPS",
            "SLIP_GUARD_RATE_FILTER_TAU_S",
            "SLIP_GUARD_MISMATCH_DEADBAND_DEGPS",
            "SLIP_GUARD_MISMATCH_RELEASE_DEGPS",
            "SLIP_GUARD_GAIN_CMPS_PER_RADPS",
            "SLIP_GUARD_MAX_CORRECTION_CMPS",
        ):
            hash_value = _fnv1a_float(hash_value, scalar(name))
    return _fnv1a_u8(hash_value, tracking_algorithm_version)


def _load_route_reference(project_root, route_index, expected_count,
                          expected_route_hash=None,
                          tracking_algorithm_version=None,
                          logged_localization_config_hash=None):
    """Load the exact single-route tables and validate the embedded hash."""
    if route_index is None or route_index < 0:
        return None
    root = Path(project_root)
    path_file = root / "code" / "nav_path.h"
    if not path_file.is_file():
        return None
    text = path_file.read_text(encoding="utf-8", errors="replace")
    if _parse_c_integer_macro(text, "NAV_SOURCE_ROUTE_INDEX") != route_index:
        return None
    if (_parse_c_integer_macro(text, "NAV_SINGLE_ROUTE_PROTOCOL") != 1 or
            _parse_c_integer_macro(text, "NAV_PATH_COUNT") != 1 or
            _parse_c_integer_macro(
                text, "NAV_SPEED_PROFILE_PRECOMPUTED"
            ) != 1):
        return None
    arrays = {
        "x": _parse_c_integer_array(text, "nav_path_x_0"),
        "y": _parse_c_integer_array(text, "nav_path_y_0"),
        "yaw_cdeg": _parse_c_integer_array(text, "nav_path_data_0"),
        "curvature": _parse_c_integer_array(text, "nav_path_curvature_0"),
        "curvature_derivative": _parse_c_integer_array(
            text, "nav_path_curvature_derivative_0"
        ),
        "speed_end": _parse_c_integer_array(text, "nav_speed_end_0"),
        "speed_cmps": _parse_c_integer_array(text, "nav_speed_cmps_0"),
        "acceleration_end": _parse_c_integer_array(
            text, "nav_acceleration_end_0"
        ),
        "acceleration_cmps2": _parse_c_integer_array(
            text, "nav_acceleration_cmps2_0"
        ),
        "takeover_lo": _parse_c_integer_array(text, "nav_map_takeover_lo_0"),
        "takeover_hi": _parse_c_integer_array(text, "nav_map_takeover_hi_0"),
        "scan_progress": _parse_c_integer_array(
            text, "nav_scan_vertex_progress_0"
        ),
        "scan_type": _parse_c_integer_array(text, "nav_scan_vertex_type_0"),
        "scan_corner_calibration_allowed": _parse_c_integer_array(
            text, "nav_scan_vertex_corner_calibration_allowed_0"
        ),
        "scan_x": _parse_c_integer_array(text, "nav_scan_vertex_x_0"),
        "scan_y": _parse_c_integer_array(text, "nav_scan_vertex_y_0"),
        "scan_support_x": _parse_c_integer_array(
            text, "nav_scan_vertex_support_x_0"
        ),
        "scan_support_y": _parse_c_integer_array(
            text, "nav_scan_vertex_support_y_0"
        ),
        "scan_segment_valid": _parse_c_integer_array(
            text, "nav_scan_segment_valid_0"
        ),
        "scan_segment_calibration_allowed": _parse_c_integer_array(
            text, "nav_scan_segment_calibration_allowed_0"
        ),
    }
    if arrays["scan_corner_calibration_allowed"] is None:
        # Older route headers did not carry the map-side mask.  Treat their
        # vertices as allowed so legacy files remain readable; a new header
        # always emits the table and its bytes participate in route_hash.
        arrays["scan_corner_calibration_allowed"] = [
            1
        ] * len(arrays["scan_type"] or [])
    if arrays["scan_segment_calibration_allowed"] is None:
        # Headers generated before the line-segment calibration mask existed
        # are equivalent to an all-allowed table for route fingerprints.
        arrays["scan_segment_calibration_allowed"] = [
            1
        ] * len(arrays["scan_segment_valid"] or [])
    if any(values is None for values in arrays.values()):
        return None
    count = min(
        len(arrays["x"]), len(arrays["y"]), len(arrays["yaw_cdeg"]),
        len(arrays["curvature"]),
    )
    if any(len(arrays[name]) != count for name in
           ("x", "y", "yaw_cdeg", "curvature")):
        return None
    if expected_count and count != expected_count:
        return None
    if len(arrays["speed_end"]) != len(arrays["speed_cmps"]):
        return None
    if (len(arrays["scan_progress"]) != len(arrays["scan_type"]) or
            len(arrays["scan_corner_calibration_allowed"]) !=
            len(arrays["scan_progress"])):
        return None
    if any(len(arrays[name]) != len(arrays["scan_progress"]) for name in
           ("scan_x", "scan_y", "scan_support_x", "scan_support_y")):
        return None
    if len(arrays["scan_segment_valid"]) != max(
        0, len(arrays["scan_progress"]) - 1
    ):
        return None
    if len(arrays["scan_segment_calibration_allowed"]) != len(
        arrays["scan_segment_valid"]
    ):
        return None
    if len(arrays["curvature_derivative"]) != count:
        return None
    if len(arrays["acceleration_end"]) != len(arrays["acceleration_cmps2"]):
        return None
    if len(arrays["takeover_lo"]) != len(arrays["takeover_hi"]):
        return None
    if tracking_algorithm_version is None:
        tracking_algorithm_version = _parse_c_integer_macro(
            _read_route_sources(root),
            "NAV_TRACKING_REFERENCE_ALGORITHM_VERSION",
        )
    if tracking_algorithm_version is None:
        return None
    scalar_macros = {
        "curvature_lsb_per_mm": "NAV_REFERENCE_CURVATURE_LSB_PER_MM",
        "curvature_derivative_lsb_per_mm2": (
            "NAV_REFERENCE_CURVATURE_DERIVATIVE_LSB_PER_MM2"
        ),
        "speed_lsb_cmps": "NAV_REFERENCE_SPEED_LSB_CMPS",
        "acceleration_lsb_cmps2": "NAV_REFERENCE_ACCELERATION_LSB_CMPS2",
        "speed_profile_max_speed_cmps": "NAV_SPEED_PROFILE_MAX_SPEED_CMPS",
        "speed_profile_max_lateral_accel_cmps2": (
            "NAV_SPEED_PROFILE_MAX_LATERAL_ACCEL_CMPS2"
        ),
        "speed_profile_max_accel_cmps2": "NAV_SPEED_PROFILE_MAX_ACCEL_CMPS2",
        "speed_profile_max_decel_cmps2": "NAV_SPEED_PROFILE_MAX_DECEL_CMPS2",
        "speed_profile_safety_margin_mm": "NAV_SPEED_PROFILE_SAFETY_MARGIN_MM",
        "speed_profile_end_cmps": "NAV_SPEED_PROFILE_END_CMPS",
        "path_sample_mm": "NAV_PATH_SAMPLE_MM",
    }
    scalar_values = {
        name: _parse_c_float_macro(text, macro)
        for name, macro in scalar_macros.items()
    }
    # v17 Frenet recovery gains live in navigation.h rather than the fixed
    # TRK1 block. They are read for host-side reconstruction only.
    if tracking_algorithm_version >= 17:
        navigation_source = _read_route_sources(root)
        scalar_values.update({
            "heading_correction_radps_per_rad": _parse_c_numeric_macro(
                navigation_source,
                "NAV_TRACK_HEADING_CORRECTION_RADPS_PER_RAD",
            ),
            "cross_track_correction_mmps_per_mm": _parse_c_numeric_macro(
                navigation_source,
                "NAV_TRACK_CROSS_TRACK_CORRECTION_MMPS_PER_MM",
            ),
            "min_correction_speed_mmps": _parse_c_numeric_macro(
                navigation_source,
                "NAV_TRACK_MIN_CORRECTION_SPEED_MMPS",
            ),
        })
    if any(value is None for value in scalar_values.values()):
        return None
    arrays.update(scalar_values)
    arrays["map_takeover_entry_lead_mm"] = _parse_c_integer_macro(
        text, "NAV_MAP_TAKEOVER_ENTRY_LEAD_MM"
    )
    if arrays["map_takeover_entry_lead_mm"] is None:
        return None
    arrays["path_length_mm"] = max(0, count - 1) * arrays["path_sample_mm"]
    config_hash_out = []
    actual = _embedded_route_hash(
        root,
        arrays,
        config_hash_out,
        tracking_algorithm_version=tracking_algorithm_version,
    )
    if config_hash_out:
        arrays["_localization_config_hash"] = config_hash_out[0]
    if expected_route_hash is not None:
        expected = int(str(expected_route_hash), 0)
        if actual != expected:
            # A newer host checkout may have added fields to the localization
            # fingerprint after the run was flashed.  Recompute one candidate
            # with the fingerprint carried by the log.  This is safe because
            # the candidate still hashes the complete current route tables,
            # geometry, speed/acceleration tables and all other constants.
            if logged_localization_config_hash is not None:
                logged_config = int(str(logged_localization_config_hash), 0)
                compatibility_actual = _embedded_route_hash(
                    root,
                    arrays,
                    tracking_algorithm_version=tracking_algorithm_version,
                    localization_config_hash_override=logged_config,
                )
                if compatibility_actual == expected:
                    arrays["_localization_config_hash_current"] = arrays.get(
                        "_localization_config_hash"
                    )
                    arrays["_localization_config_hash"] = logged_config
                    arrays["_localization_config_compatibility"] = True
                    return arrays
            # Algorithm 11 added the encoder/gyro slip guard. Keep the one
            # explicit v10 fallback for logs that predate TRK1 versioned
            # controller reconstruction.
            legacy_actual = _embedded_route_hash(
                root, arrays, tracking_algorithm_version=10
            )
            if legacy_actual != expected:
                raise ValueError(
                    "route_hash mismatch: log=0x%08X current=0x%08X; "
                    "refusing to decode route-dependent telemetry" %
                    (expected, actual)
                )
            arrays["_tracking_algorithm_compatibility"] = 10
    return arrays


def _load_planned_route(route_reference):
    """Expand the already hash-validated route reference for CSV/display.

    ``decode_payload`` has already loaded and fingerprint-checked the exact
    single route used to reconstruct navigation.  Reusing that object here
    keeps the planned-route CSV, overview and dashboard on the same immutable
    reference instead of parsing ``nav_path.h`` a second time.
    """
    if not route_reference:
        return []
    reference = route_reference
    feature_by_progress = {}
    for occurrence, (progress, feature_type) in enumerate(zip(
        reference["scan_progress"], reference["scan_type"]
    )):
        feature_by_progress.setdefault(int(progress), []).append(
            (occurrence, feature_type)
        )
    type_names = {0: "straight", 1: "corner", 2: "junction"}
    result = []
    for index in range(len(reference["x"])):
        speed_segment, _, speed_value = _route_segment_at(
            index, reference["speed_end"], reference["speed_cmps"]
        )
        acceleration_segment, _, acceleration_value = _route_segment_at(
            index,
            reference["acceleration_end"],
            reference["acceleration_cmps2"],
        )
        takeover_region = None
        for region_index, (low, high) in enumerate(zip(
            reference["takeover_lo"], reference["takeover_hi"]
        )):
            if int(low) <= index <= int(high):
                takeover_region = region_index
                break
        curvature = (
            reference["curvature"][index]
            * reference["curvature_lsb_per_mm"]
        )
        features = feature_by_progress.get(index, [])
        result.append({
            "progress_index": index,
            "distance_mm": index * reference["path_sample_mm"],
            "x_mm": reference["x"][index],
            "y_mm": reference["y"][index],
            "yaw_deg": _wrap_degrees(reference["yaw_cdeg"][index] / 100.0),
            "curvature_per_mm": curvature,
            "curvature_derivative_per_mm2": (
                reference["curvature_derivative"][index]
                * reference["curvature_derivative_lsb_per_mm2"]
            ),
            "curvature_radius_mm": (
                1.0 / abs(curvature) if abs(curvature) > 1.0e-12 else None
            ),
            "target_speed_cmps": speed_value * reference["speed_lsb_cmps"],
            "reference_acceleration_cmps2": (
                acceleration_value * reference["acceleration_lsb_cmps2"]
            ),
            "speed_segment_index": speed_segment,
            "acceleration_segment_index": acceleration_segment,
            "map_takeover_allowed": takeover_region is not None,
            "map_takeover_region_index": takeover_region,
            "scan_feature_occurrence": "|".join(
                str(occurrence) for occurrence, _ in features
            ),
            "scan_feature_type": "|".join(
                type_names.get(feature_type, f"unknown_{feature_type}")
                for _, feature_type in features
            ),
        })
    return result


def _load_scan_features(route_reference):
    """Build the physical feature layer from the already validated route.

    Route loading validates the single-route protocol, point counts and the
    embedded route/configuration fingerprints.  Reusing those arrays here
    prevents a second, unchecked parse of ``nav_path.h`` from drifting away
    from the route that was actually used to decode the telemetry.
    """
    if not route_reference:
        return []
    arrays = route_reference
    required = ("scan_x", "scan_y", "scan_progress", "scan_type")
    if any(field not in arrays for field in required):
        return []
    count = min(len(arrays[field]) for field in required)
    if any(len(arrays[field]) != count for field in required):
        return []
    type_names = {0: "straight", 1: "corner", 2: "junction"}
    return [
        {
            "feature_index": index,
            "x_mm": arrays["scan_x"][index],
            "y_mm": arrays["scan_y"][index],
            "progress_index": arrays["scan_progress"][index],
            "feature_type": arrays["scan_type"][index],
            "feature_type_name": type_names.get(
                arrays["scan_type"][index], "unknown"
            ),
            "support_x_mm": (
                arrays.get("scan_support_x", [0] * count)[index]
                if index < len(arrays.get("scan_support_x", [])) else 0
            ),
            "support_y_mm": (
                arrays.get("scan_support_y", [0] * count)[index]
                if index < len(arrays.get("scan_support_y", [])) else 0
            ),
        }
        for index in range(count)
    ]


def _build_scan_feature_annotations(decoded, scan_features):
    """Combine accepted transactions and fixed per-feature diagnostics."""
    feature_by_index = {
        feature["feature_index"]: feature for feature in (scan_features or [])
    }
    annotations = {}

    def annotation_for(feature_index):
        feature = feature_by_index.get(feature_index)
        if feature is None:
            return None
        return annotations.setdefault(
            feature_index,
            {
                **feature,
                "accepted_times_ms": set(),
                "transactions": [],
                "diagnostic": None,
            },
        )
    for transaction in decoded.get("localization", []):
        feature_index = transaction.get("route_feature_occurrence")
        if feature_index is None:
            continue
        annotation = annotation_for(int(feature_index))
        if annotation is None:
            continue
        timestamp_ms = float(transaction.get("timestamp_ms") or 0.0)
        annotation["transactions"].append(transaction)
        if transaction.get("status_name") == "accepted":
            annotation["accepted_times_ms"].add(timestamp_ms)
    for diagnostic in decoded.get("localization_features", []):
        feature_index = diagnostic.get("route_feature_occurrence")
        if feature_index is None or diagnostic.get("outcome") == "not_reached":
            continue
        annotation = annotation_for(int(feature_index))
        if annotation is not None:
            annotation["diagnostic"] = diagnostic

    type_labels = {"corner": "直角", "junction": "路口", "straight": "直线"}
    result = []
    for feature_index in sorted(annotations):
        annotation = annotations[feature_index]
        accepted_times = sorted(annotation["accepted_times_ms"])
        diagnostic = annotation.get("diagnostic") or {}
        status_parts = []
        detail_parts = []
        if accepted_times:
            start = int(round(accepted_times[0]))
            end = int(round(accepted_times[-1]))
            time_text = f"{start}ms" if start == end else f"{start}–{end}ms"
            status_parts.append(
                f"部分校准{time_text}"
                if diagnostic.get("partial") else f"校准{time_text}"
            )
            details = [
                tx for tx in annotation["transactions"]
                if tx.get("status_name") == "accepted"
            ]
            latest = details[-1] if details else {}
            if diagnostic.get("partial"):
                detail_parts.append(
                    f"单边部分校准 {time_text}，可观测法向/航向已修正，"
                    f"RMS {latest.get('rms_before_mm', 0.0):.1f}→"
                    f"{latest.get('rms_after_mm', 0.0):.1f} mm，"
                    f"沿边方向保持不变"
                )
            else:
                detail_parts.append(
                    f"完整特征校准 {time_text}，RMS "
                    f"{latest.get('rms_before_mm', 0.0):.1f}→"
                    f"{latest.get('rms_after_mm', 0.0):.1f} mm，"
                    f"内点 {latest.get('inlier_count', 0)}，"
                    f"进度 {latest.get('progress_before_points', 0.0):.1f}→"
                    f"{latest.get('progress_after_points', 0.0):.1f}"
                )
        if diagnostic.get("outcome") == "partial" and not accepted_times:
            time_text = f"{int(diagnostic.get('last_timestamp_ms') or 0)}ms"
            status_parts.append(f"部分校准{time_text}")
            detail_parts.append(
                "已确认当前路线边，观测量不足以产生有效位姿跳变，"
                "保持原位姿并消费该特征"
            )
        outcome = diagnostic.get("outcome")
        if outcome == "failed":
            time_text = f"{int(diagnostic.get('last_timestamp_ms') or 0)}ms"
            status_parts.append(f"校准失败{time_text}")
            detail_parts.append(
                f"最后原因 {diagnostic.get('last_reject_reason_name', 'unknown')}，"
                f"最后门限 {diagnostic.get('last_detail_names') or '未细分'}，"
                f"尝试 {diagnostic.get('attempt_count', 0)} 次，"
                f"三线点数 {diagnostic.get('line0_point_count', 0)}/"
                f"{diagnostic.get('line1_point_count', 0)}/"
                f"{diagnostic.get('line2_point_count', 0)}，"
                f"三线帧数 {diagnostic.get('line0_frame_count', 0)}/"
                f"{diagnostic.get('line1_frame_count', 0)}/"
                f"{diagnostic.get('line2_frame_count', 0)}"
            )
        elif outcome == "passed_without_candidate":
            status_parts.append("未进候选门")
            detail_parts.append("车辆进度已越过该特征，但完整校准候选从未建立")
        elif outcome == "disabled_by_map":
            status_parts.append("地图禁用校准")
            detail_parts.append("该特征在路线导出表中被禁止校准，不计入运行失败")
        if not status_parts:
            continue
        annotation["type_label"] = type_labels.get(
            annotation["feature_type_name"], annotation["feature_type_name"]
        )
        annotation["status_label"] = "；".join(status_parts)
        annotation["detail_label"] = "；".join(detail_parts)
        result.append(annotation)
    return result


def _build_physical_feature_segments(scan_features):
    segments = []
    for index in range(max(0, len(scan_features) - 1)):
        first = scan_features[index]
        second = scan_features[index + 1]
        segments.append({
            "kind": "route_edge",
            "feature_index": index,
            "x1_mm": first["x_mm"],
            "y1_mm": first["y_mm"],
            "x2_mm": second["x_mm"],
            "y2_mm": second["y_mm"],
        })
    for feature in scan_features:
        support_x = feature.get("support_x_mm", 0)
        support_y = feature.get("support_y_mm", 0)
        if support_x == feature["x_mm"] and support_y == feature["y_mm"]:
            continue
        segments.append({
            "kind": "support_branch",
            "feature_index": feature["feature_index"],
            "x1_mm": feature["x_mm"],
            "y1_mm": feature["y_mm"],
            "x2_mm": support_x,
            "y2_mm": support_y,
        })
    return segments


def _scan_annotation_color(annotation):
    outcome = (annotation.get("diagnostic") or {}).get("outcome")
    if outcome == "failed":
        return "#dc2626"
    if outcome == "passed_without_candidate":
        return "#d97706"
    if outcome == "disabled_by_map":
        return "#64748b"
    if outcome == "partial":
        return "#2563eb"
    return "#16a34a"


def decode_payload(payload, project_root=None):
    payload = bytes(payload)
    (
        counts,
        crc,
        localization_config,
        tracking_config,
        localization_features,
        diagnostic_drop_count,
        schema_version,
        layout,
    ) = _validate_header(payload)
    metadata = _decode_metadata(
        payload, counts, crc, localization_config, tracking_config,
        schema_version, layout
    )
    metadata["localization_feature_diagnostics"] = {
        "registered_count": len(localization_features),
        "diagnostic_drop_count": diagnostic_drop_count,
    }
    events = _decode_events(payload, counts["events"], layout["event_offset"])
    phototube_scan = _decode_phototube_scan(payload, localization_config)
    phototube_archive_overwrite_count = next(
        (
            max(0, record["value0"])
            for record in reversed(events)
            if record["type"] == 6
            and record["source"] == 2
            and record["code"] == 7
        ),
        0,
    )
    phototube_scan["archive_overwrite_point_count"] = (
        phototube_archive_overwrite_count
    )
    metadata["drops"]["phototube_archive_overwrite"] = phototube_scan.get(
        "archive_overwrite_point_count", 0
    )
    metadata["drops"]["phototube_emem_queue"] = phototube_scan.get(
        "emem_queue_drop_count", 0
    )
    metadata["phototube_scan"] = {
        key: value for key, value in phototube_scan.items() if key != "points"
    }
    if project_root is None:
        project_root = Path(__file__).resolve().parents[1]
    logged_localization_hash = int(
        str(localization_config["config_hash"]), 0
    )
    route_reference = _load_route_reference(
        project_root,
        metadata["route_index"],
        metadata["route_point_count"],
        metadata["route_hash"],
        tracking_config["algorithm_version"],
        logged_localization_config_hash=logged_localization_hash,
    )
    if route_reference is None:
        raise ValueError(
            "schema 21 requires the matching single-route code/nav_path.h "
            "and route hash to reconstruct navigation reference fields"
        )
    embedded_localization_hash = route_reference.get(
        "_localization_config_hash"
    )
    if (embedded_localization_hash is None or
            embedded_localization_hash != logged_localization_hash):
        raise ValueError(
            "localization config hash mismatch: "
            f"log=0x{logged_localization_hash:08X} "
            f"current=0x{int(embedded_localization_hash or 0):08X}; "
            "refusing to decode localization-dependent telemetry"
        )
    metadata["localization_config"]["config_hash_valid"] = True
    if not math.isclose(
        float(tracking_config["path_sample_mm"]),
        float(route_reference["path_sample_mm"]),
        rel_tol=1.0e-6,
        abs_tol=1.0e-6,
    ):
        raise ValueError("tracking config/nav_path sample spacing mismatch")
    metadata["route_reference"] = {
        "localization_config_hash": (
            f"0x{embedded_localization_hash:08X}"
        ),
        "localization_config_hash_current": (
            f"0x{route_reference['_localization_config_hash_current']:08X}"
            if route_reference.get("_localization_config_compatibility")
            else None
        ),
        "localization_config_compatibility": bool(
            route_reference.get("_localization_config_compatibility")
        ),
        "path_sample_mm": route_reference["path_sample_mm"],
        "path_length_mm": route_reference["path_length_mm"],
        "curvature_lsb_per_mm": route_reference["curvature_lsb_per_mm"],
        "curvature_derivative_lsb_per_mm2": route_reference[
            "curvature_derivative_lsb_per_mm2"
        ],
        "speed_lsb_cmps": route_reference["speed_lsb_cmps"],
        "acceleration_lsb_cmps2": route_reference[
            "acceleration_lsb_cmps2"
        ],
        "speed_segment_count": len(route_reference["speed_end"]),
        "acceleration_segment_count": len(route_reference["acceleration_end"]),
        "map_takeover_region_count": len(route_reference["takeover_lo"]),
        "map_takeover_entry_lead_mm": route_reference[
            "map_takeover_entry_lead_mm"
        ],
        "scan_feature_count": len(route_reference["scan_progress"]),
    }
    control = _decode_control(
        payload,
        counts["control"],
        metadata["constants"]["cmps_per_count"],
        metadata["parameters"],
        layout["control_offset"],
    )
    navigation = _decode_navigation(
        payload, counts["navigation"], route_reference, control,
        metadata["tracking_config"],
        layout["navigation_offset"]
    )
    localization = _decode_localization(
        payload,
        counts["localization"],
        route_reference,
        layout["localization_offset"],
        metadata["localization"]["overwrite_count"],
    )
    localization_features = _attach_localization_feature_reference(
        localization_features, route_reference
    )
    localization_features = _classify_localization_features(
        localization_features, navigation, route_reference
    )
    _attach_localization_to_navigation(
        navigation, localization, route_reference,
        metadata["tracking_config"]
    )
    projection_summary = _attach_scan_map_projections(
        phototube_scan.get("points", []), localization, navigation
    )
    phototube_scan.update(projection_summary)
    phototube_scan["online_map_projection_count"] = sum(
        point.get("map_x_mm") is not None
        for point in phototube_scan.get("points", [])
    )
    phototube_scan["final_map_projection_count"] = sum(
        point.get("final_map_x_mm") is not None
        for point in phototube_scan.get("points", [])
    )
    metadata["phototube_scan"] = {
        key: value for key, value in phototube_scan.items() if key != "points"
    }
    vision = _decode_vision(
        payload,
        counts["vision"],
        layout["vision_offset"],
        metadata["vision_first_frame_seq"],
    )
    imu = _decode_imu(payload, counts["imu"], layout["imu_offset"])
    _attach_control_context(
        control, vision, imu, navigation, metadata["tracking_config"]
    )
    return {
        "metadata": metadata,
        "control": control,
        "navigation": navigation,
        "vision": vision,
        "events": events,
        "localization": localization,
        "localization_features": localization_features,
        "imu": imu,
        "phototube_scan": phototube_scan,
        # Keep the exact, hash-validated route arrays available to the
        # visualization builder; do not re-parse nav_path.h for a second
        # unverified physical-feature layer.
        "route_reference": route_reference,
    }


def _finite_numbers(values):
    result = []
    for value in values:
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(number):
            result.append(number)
    return result


def _percentile(values, fraction):
    numbers = sorted(_finite_numbers(values))
    if not numbers:
        return None
    if len(numbers) == 1:
        return numbers[0]
    position = (len(numbers) - 1) * fraction
    lower = int(position)
    upper = min(lower + 1, len(numbers) - 1)
    weight = position - lower
    return numbers[lower] + (numbers[upper] - numbers[lower]) * weight


def _series_stats(values):
    numbers = _finite_numbers(values)
    if not numbers:
        return {"count": 0}
    mean = sum(numbers) / len(numbers)
    return {
        "count": len(numbers),
        "mean": mean,
        "abs_mean": sum(abs(value) for value in numbers) / len(numbers),
        "rms": math.sqrt(sum(value * value for value in numbers) / len(numbers)),
        "min": min(numbers),
        "max": max(numbers),
        "p50": _percentile(numbers, 0.50),
        "p95_abs": _percentile([abs(value) for value in numbers], 0.95),
        "p99_abs": _percentile([abs(value) for value in numbers], 0.99),
    }


def _ratio(numerator, denominator):
    if not denominator:
        return None
    return numerator / denominator


def _wrap_degrees(value):
    return (value + 180.0) % 360.0 - 180.0


def _downsample(records, fields=None, limit=120):
    if not records:
        return []
    selected_fields = tuple(fields) if fields is not None else None
    if len(records) <= limit:
        indices = range(len(records))
    elif limit <= 1:
        indices = (0,)
    else:
        indices = sorted(
            {
                round(index * (len(records) - 1) / (limit - 1))
                for index in range(limit)
            }
        )
    result = []
    for index in indices:
        record = records[index]
        if selected_fields is None:
            result.append(dict(record))
        else:
            result.append({field: record.get(field) for field in selected_fields})
    return result


def _scan_points_in_capture_map(scan_points):
    """Use each point's capture-time map projection for route display.

    Applying the final accepted transform to an old point mixes coordinate
    frames across localization updates and visually smears the scan.  The raw
    odom point and capture-time projection remain immutable diagnostics; this
    only selects the display coordinates.
    """
    if not scan_points:
        return list(scan_points)
    if all(
        point.get("map_x_mm") is not None
        and point.get("map_y_mm") is not None
        for point in scan_points
    ):
        result = []
        for point in scan_points:
            mapped = dict(point)
            mapped["x_mm"] = point["map_x_mm"]
            mapped["y_mm"] = point["map_y_mm"]
            mapped["coordinate_frame"] = "map_from_capture_transform"
            result.append(mapped)
        return result
    # Missing ledger-derived projections are a diagnostic data gap.  Keep
    # raw odom coordinates visibly raw instead of silently substituting the
    # last navigation transform and presenting the result as reconstructed
    # localization history.
    return [
        dict(point, coordinate_frame="raw_odom_unprojected")
        for point in scan_points
    ]


def _attach_scan_map_projections(scan_points, localization, navigation=None):
    """Project immutable raw points using the localization ledger.

    Accepted transactions are authoritative when the retained ledger is
    contiguous.  If its ring has overwritten an accepted sequence, navigation
    samples are used only inside the missing interval (or after a missing tail)
    and the source is marked explicitly; a gap without a navigation fallback
    remains unprojected instead of being presented as a false map pose.
    """
    if not scan_points:
        return {
            "localization_ledger_gap_count": 0,
            "localization_projection_fallback_point_count": 0,
            "localization_projection_unresolved_point_count": 0,
            "localization_same_ms_boundary_point_count": 0,
            "localization_final_transform_source": "none",
        }

    accepted = sorted(
        (
            {
                "timestamp_ms": float(tx["timestamp_ms"]),
                "localization_update_seq": tx["localization_update_seq"],
                "map_from_odom_tx_mm": tx["t_after_tx_mm"],
                "map_from_odom_ty_mm": tx["t_after_ty_mm"],
                "map_from_odom_yaw_deg": tx["t_after_yaw_deg"],
                "source": "accepted_localization_transaction",
            }
            for tx in localization
            if tx.get("status_name") == "accepted"
        ),
        key=lambda transform: (
            transform["timestamp_ms"],
            int(transform["localization_update_seq"]),
        ),
    )
    navigation_transforms = sorted(
        (
            {
                "timestamp_ms": float(row["timestamp_ms"]),
                "localization_update_seq": int(
                    row.get("localization_update_seq", 0)
                ),
                "map_from_odom_tx_mm": float(row["map_from_odom_tx_mm"]),
                "map_from_odom_ty_mm": float(row["map_from_odom_ty_mm"]),
                "map_from_odom_yaw_deg": float(row["map_from_odom_yaw_deg"]),
                "source": "navigation_snapshot_gap_fallback",
            }
            for row in (navigation or [])
            if row.get("timestamp_ms") is not None
            and row.get("map_from_odom_tx_mm") is not None
            and row.get("map_from_odom_ty_mm") is not None
            and row.get("map_from_odom_yaw_deg") is not None
        ),
        key=lambda transform: (
            transform["timestamp_ms"],
            int(transform["localization_update_seq"]),
        ),
    )
    ledger_gaps = []
    for previous, current in zip(accepted, accepted[1:]):
        sequence_delta = (
            int(current["localization_update_seq"])
            - int(previous["localization_update_seq"])
        ) & 0xFFFF
        if sequence_delta != 1:
            ledger_gaps.append({
                "start_timestamp_ms": previous["timestamp_ms"],
                "end_timestamp_ms": current["timestamp_ms"],
                "previous_seq": previous["localization_update_seq"],
                "next_seq": current["localization_update_seq"],
                "missing_update_count": max(1, sequence_delta - 1),
            })
    identity = {
        "timestamp_ms": 0.0,
        "localization_update_seq": 0,
        "map_from_odom_tx_mm": 0.0,
        "map_from_odom_ty_mm": 0.0,
        "map_from_odom_yaw_deg": 0.0,
        "source": "run_start_identity",
    }
    baseline_transform = identity
    if localization:
        first_tx = localization[0]
        baseline_transform = {
            "timestamp_ms": 0.0,
            "localization_update_seq": (
                int(first_tx["localization_update_seq"])
                - (1 if first_tx.get("status_name") == "accepted" else 0)
            ) & 0xFFFF,
            "map_from_odom_tx_mm": first_tx["t_before_tx_mm"],
            "map_from_odom_ty_mm": first_tx["t_before_ty_mm"],
            "map_from_odom_yaw_deg": first_tx["t_before_yaw_deg"],
            "source": "oldest_retained_transaction_t_before",
        }
    tail_gap = False
    if accepted and navigation_transforms:
        tail_sequence_delta = (
            int(navigation_transforms[-1]["localization_update_seq"])
            - int(accepted[-1]["localization_update_seq"])
        ) & 0xFFFF
        tail_gap = 0 < tail_sequence_delta < 0x8000
    if tail_gap:
        final_transform = dict(navigation_transforms[-1])
        final_transform["source"] = "navigation_snapshot_tail_fallback"
    elif accepted:
        final_transform = accepted[-1]
    elif navigation_transforms:
        final_transform = dict(navigation_transforms[-1])
        final_transform["source"] = "navigation_snapshot_only"
    else:
        final_transform = baseline_transform

    def project_xy(raw_x, raw_y, transform):
        yaw_rad = math.radians(float(transform["map_from_odom_yaw_deg"]))
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        return (
            float(transform["map_from_odom_tx_mm"])
            + cos_yaw * float(raw_x) - sin_yaw * float(raw_y),
            float(transform["map_from_odom_ty_mm"])
            + sin_yaw * float(raw_x) + cos_yaw * float(raw_y),
        )

    transform_index = 0
    navigation_index = 0
    gap_index = 0
    current_transform = baseline_transform
    fallback_point_count = 0
    unresolved_point_count = 0
    same_ms_boundary_point_count = 0
    for point in scan_points:
        capture_ms = float(point["capture_ms"])
        same_ms_boundary = bool(
            (transform_index < len(accepted) and
             accepted[transform_index]["timestamp_ms"] == capture_ms) or
            (navigation_index < len(navigation_transforms) and
             navigation_transforms[navigation_index]["timestamp_ms"] ==
             capture_ms)
        )
        if same_ms_boundary:
            same_ms_boundary_point_count += 1
        while (
            transform_index < len(accepted)
            # Millisecond timestamps cannot order a scan capture and a
            # localization commit inside the same millisecond.  Apply the
            # transform only after that boundary so an old raw point is never
            # shown with a correction that may not have existed at capture.
            and accepted[transform_index]["timestamp_ms"] < capture_ms
        ):
            current_transform = accepted[transform_index]
            transform_index += 1
        while (
            navigation_index < len(navigation_transforms)
            and navigation_transforms[navigation_index]["timestamp_ms"]
            < capture_ms
        ):
            navigation_index += 1
        while (
            gap_index < len(ledger_gaps)
            and capture_ms > ledger_gaps[gap_index]["end_timestamp_ms"]
        ):
            gap_index += 1
        active_gap = (
            ledger_gaps[gap_index]
            if gap_index < len(ledger_gaps)
            and capture_ms > ledger_gaps[gap_index]["start_timestamp_ms"]
            and capture_ms <= ledger_gaps[gap_index]["end_timestamp_ms"]
            else None
        )
        tail_gap_active = bool(
            tail_gap and accepted
            and capture_ms > accepted[-1]["timestamp_ms"]
        )
        navigation_only = not accepted and bool(navigation_transforms)
        projection_transform = current_transform
        if active_gap is not None or tail_gap_active or navigation_only:
            if navigation_index > 0:
                projection_transform = navigation_transforms[navigation_index - 1]
                if navigation_only:
                    projection_transform = dict(projection_transform)
                    projection_transform["source"] = "navigation_snapshot_only"
                fallback_point_count += 1
            else:
                projection_transform = None
                unresolved_point_count += 1
        if projection_transform is not None:
            point["map_x_mm"], point["map_y_mm"] = project_xy(
                point["x_mm"], point["y_mm"], projection_transform
            )
            point["map_axle_x_mm"], point["map_axle_y_mm"] = project_xy(
                point["raw_axle_x_mm"],
                point["raw_axle_y_mm"],
                projection_transform,
            )
            point["map_yaw_deg"] = _wrap_degrees(
                point["raw_yaw_deg"]
                + float(projection_transform["map_from_odom_yaw_deg"])
            )
            point["map_transform_timestamp_ms"] = projection_transform[
                "timestamp_ms"
            ]
            point["map_localization_update_seq"] = projection_transform[
                "localization_update_seq"
            ]
            point["map_transform_source"] = projection_transform["source"]
        else:
            point["map_x_mm"] = None
            point["map_y_mm"] = None
            point["map_axle_x_mm"] = None
            point["map_axle_y_mm"] = None
            point["map_yaw_deg"] = None
            point["map_transform_timestamp_ms"] = None
            point["map_localization_update_seq"] = None
            point["map_transform_source"] = "localization_ledger_gap_unresolved"
        point["map_transform_time_boundary_ambiguous"] = same_ms_boundary
        point["final_map_x_mm"], point["final_map_y_mm"] = project_xy(
            point["x_mm"], point["y_mm"], final_transform
        )
        (
            point["final_map_axle_x_mm"],
            point["final_map_axle_y_mm"],
        ) = project_xy(
            point["raw_axle_x_mm"],
            point["raw_axle_y_mm"],
            final_transform,
        )
        point["final_map_yaw_deg"] = _wrap_degrees(
            point["raw_yaw_deg"]
            + float(final_transform["map_from_odom_yaw_deg"])
        )
    reported_gaps = list(ledger_gaps)
    if tail_gap:
        reported_gaps.append({
            "start_timestamp_ms": accepted[-1]["timestamp_ms"],
            "end_timestamp_ms": None,
            "previous_seq": accepted[-1]["localization_update_seq"],
            "next_seq": navigation_transforms[-1][
                "localization_update_seq"
            ],
            "missing_update_count": max(1, tail_sequence_delta),
        })
    if (not accepted and navigation_transforms and
            navigation_transforms[-1]["localization_update_seq"] != 0):
        reported_gaps.append({
            "start_timestamp_ms": None,
            "end_timestamp_ms": None,
            "previous_seq": None,
            "next_seq": navigation_transforms[-1][
                "localization_update_seq"
            ],
            "missing_update_count": navigation_transforms[-1][
                "localization_update_seq"
            ],
        })
    return {
        "localization_ledger_gap_count": len(reported_gaps),
        "localization_ledger_gaps": reported_gaps,
        "localization_projection_fallback_point_count": fallback_point_count,
        "localization_projection_unresolved_point_count": unresolved_point_count,
        "localization_same_ms_boundary_point_count":
            same_ms_boundary_point_count,
        "localization_final_transform_source": final_transform["source"],
    }


def _json_safe(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _analyze_control(records):
    speed_errors = []
    left_target_errors = []
    right_target_errors = []
    wheel_balance_errors = []
    for record in records:
        actual_average = (record["actual_l_cmps"] + record["actual_r_cmps"]) / 2.0
        speed_errors.append(actual_average - record["plan_v_cmps"])
        left_target_errors.append(record["actual_l_cmps"] - record["target_l_cmps"])
        right_target_errors.append(record["actual_r_cmps"] - record["target_r_cmps"])
        wheel_balance_errors.append(record["actual_l_cmps"] - record["actual_r_cmps"])

    time_values = _finite_numbers(record["tick_ms"] for record in records)
    time_span_ms = max(time_values) - min(time_values) if time_values else 0
    return {
        "sample_count": len(records),
        "time_span_ms": time_span_ms,
        "effective_sample_rate_hz": (
            len(records) * 1000.0 / time_span_ms if time_span_ms > 0 else None
        ),
        "speed_error_actual_average_minus_plan_cmps": _series_stats(speed_errors),
        "left_target_error_cmps": _series_stats(left_target_errors),
        "right_target_error_cmps": _series_stats(right_target_errors),
        "wheel_balance_actual_left_minus_right_cmps": _series_stats(wheel_balance_errors),
        "signals": {
            "plan_v_cmps": _series_stats(record["plan_v_cmps"] for record in records),
            "actual_avg_cmps": _series_stats(record["actual_avg_cmps"] for record in records),
            "actual_l_cmps": _series_stats(record["actual_l_cmps"] for record in records),
            "actual_r_cmps": _series_stats(record["actual_r_cmps"] for record in records),
            "target_l_cmps": _series_stats(record["target_l_cmps"] for record in records),
            "target_r_cmps": _series_stats(record["target_r_cmps"] for record in records),
            "accel_l_cmps2": _series_stats(
                record["accel_l_cmps2"] for record in records
            ),
            "accel_r_cmps2": _series_stats(
                record["accel_r_cmps2"] for record in records
            ),
            "ff_l_duty": _series_stats(record["ff_l_duty"] for record in records),
            "ff_r_duty": _series_stats(record["ff_r_duty"] for record in records),
            "pi_l_duty": _series_stats(record["pi_l_duty"] for record in records),
            "pi_r_duty": _series_stats(record["pi_r_duty"] for record in records),
            "pwm_l": _series_stats(record["pwm_l"] for record in records),
            "pwm_r": _series_stats(record["pwm_r"] for record in records),
            "yaw_rate_error_degps": _series_stats(
                record["yaw_rate_error_degps"] for record in records
            ),
            "target_yaw_rate_degps": _series_stats(
                record["target_yaw_rate_degps"] for record in records
            ),
            "measured_yaw_rate_degps": _series_stats(
                record["measured_yaw_rate_degps"] for record in records
            ),
            "yaw_rate_feedforward_turn_cmps": _series_stats(
                record["yaw_rate_feedforward_turn_cmps"] for record in records
            ),
            "yaw_rate_feedback_turn_cmps": _series_stats(
                record["yaw_rate_feedback_turn_cmps"] for record in records
            ),
            "pd_p_turn_inferred_cmps": _series_stats(
                record["pd_p_turn_inferred_cmps"] for record in records
            ),
            "pd_d_turn_cmps": _series_stats(
                record["pd_d_turn_cmps"] for record in records
            ),
            "gkd_turn_cmps": _series_stats(
                record["gkd_turn_cmps"] for record in records
            ),
            "tracking_turn_reconstruction_error_cmps": _series_stats(
                record["tracking_turn_reconstruction_error_cmps"]
                for record in records
            ),
        },
        "feedforward_logging": {
            "description": "schema-21 保留 ff_* 和 accel_* 字段以兼容旧日志；当前纯增量 PI 固件将 ff_* 与 accel_* 置零，pi_* 等于最终 pwm_*。",
            "control_record_rate_hz": CONTROL_LOG_RATE_HZ,
            "fields": [
                "target_l_cmps",
                "target_r_cmps",
                "accel_l_cmps2",
                "accel_r_cmps2",
                "ff_l_duty",
                "ff_r_duty",
                "pi_l_duty",
                "pi_r_duty",
                "pwm_l",
                "pwm_r",
            ],
        },
    }


def _speed_tuning_summary(records):
    """Summarize signals that decide the next speed-PI experiment."""
    if not records:
        return {
            "sample_count": 0,
            "usable": False,
            "reason": "no control ISR records",
        }

    def _mean(values):
        numbers = _finite_numbers(values)
        return sum(numbers) / len(numbers) if numbers else None

    def _abs_max(values):
        numbers = _finite_numbers(values)
        return max((abs(number) for number in numbers), default=None)

    control = _analyze_control(records)
    error = control["speed_error_actual_average_minus_plan_cmps"]
    pwm_l = control["signals"]["pwm_l"]
    pwm_r = control["signals"]["pwm_r"]
    left_error = control["left_target_error_cmps"]
    right_error = control["right_target_error_cmps"]
    plan_values = _finite_numbers(record["plan_v_cmps"] for record in records)
    return {
        "sample_count": len(records),
        "usable": len(records) >= 500,
        "minimum_recommended_samples": 500,
        "time_span_ms": control["time_span_ms"],
        "target_speed_min_cmps": min(plan_values) if plan_values else None,
        "target_speed_max_cmps": max(plan_values) if plan_values else None,
        "target_speed_range_cmps": (
            max(plan_values) - min(plan_values) if plan_values else None
        ),
        "speed_error_actual_average_minus_plan_cmps": error,
        "speed_error_mean_cmps": error.get("mean"),
        "speed_error_rms_cmps": error.get("rms"),
        "speed_error_p95_abs_cmps": error.get("p95_abs"),
        "steady_speed_error_mean_cmps": _mean(
            record["speed_error_actual_minus_plan_cmps"]
            for record in records[len(records) // 2 :]
        ),
        "left_target_error_p95_abs_cmps": left_error.get("p95_abs"),
        "right_target_error_p95_abs_cmps": right_error.get("p95_abs"),
        "actual_average_cmps": control["signals"].get("actual_avg_cmps", {}),
        "pwm_l_p95_abs": pwm_l.get("p95_abs"),
        "pwm_r_p95_abs": pwm_r.get("p95_abs"),
        "pwm_l_abs_max": _abs_max(record["pwm_l"] for record in records),
        "pwm_r_abs_max": _abs_max(record["pwm_r"] for record in records),
        "wheel_balance_actual_p95_abs_cmps": control[
            "wheel_balance_actual_left_minus_right_cmps"
        ].get("p95_abs"),
        "interpretation": {
            "speed_KP": "先根据起步/变速响应和振荡决定；只改变 KP 时观察速度误差收敛与 PWM 抖动。",
            "speed_KI": "KP 稳定后再根据稳态同向误差决定；不要用 KI 掩盖限幅或目标速度变化。",
            "speed_IncrementLimit": "只有出现单拍 PWM/轮速突变或频繁限幅时才处理；它不是首轮主调参数。",
        },
    }


def _analyze_imu(records):
    if not records:
        return {
            "sample_count": 0,
            "usable": False,
            "reason": "no IMU records",
        }
    timestamps = _finite_numbers(record["timestamp_us"] for record in records)
    time_span_us = max(timestamps) - min(timestamps) if timestamps else 0
    return {
        "sample_count": len(records),
        "usable": len(records) >= IMU_SAMPLE_HZ,
        "minimum_recommended_samples": IMU_SAMPLE_HZ,
        "time_span_us": time_span_us,
        "time_span_ms": time_span_us / 1000.0,
        "effective_sample_rate_hz": (
            (len(records) - 1) * 1_000_000.0 / time_span_us
            if time_span_us > 0 and len(records) > 1
            else None
        ),
        "signals": {
            "acc_x_g": _series_stats(record["acc_x_g"] for record in records),
            "acc_y_g": _series_stats(record["acc_y_g"] for record in records),
            "acc_z_g": _series_stats(record["acc_z_g"] for record in records),
            "acc_norm_g": _series_stats(record["acc_norm_g"] for record in records),
            "gyro_x_dps": _series_stats(record["gyro_x_dps"] for record in records),
            "gyro_y_dps": _series_stats(record["gyro_y_dps"] for record in records),
            "gyro_z_dps": _series_stats(record["gyro_z_dps"] for record in records),
            "gyro_norm_dps": _series_stats(record["gyro_norm_dps"] for record in records),
            "quaternion_norm": _series_stats(record["quaternion_norm"] for record in records),
            "roll_deg": _series_stats(record["roll_deg"] for record in records),
            "pitch_deg": _series_stats(record["pitch_deg"] for record in records),
            "yaw_deg": _series_stats(record["yaw_deg"] for record in records),
        },
        "interpretation": {
            "acc_norm_g": "静止/匀速时接近 1g；偏离通常表示振动、刹车或转弯向心加速度。",
            "quaternion_norm": "应接近 1；明显偏离表示量化、数据损坏或姿态状态异常。",
            "attitude": "Roll/Pitch/Yaw 由回传四元数在上位机按与固件相同公式计算。",
        },
    }


def _analyze_navigation(records, route_point_count):
    x_errors = []
    y_errors = []
    for record in records:
        x_errors.append(record["actual_x_mm"] - record["ideal_x_mm"])
        y_errors.append(record["actual_y_mm"] - record["ideal_y_mm"])

    progress = [record["progress_index"] for record in records]
    progress_start = min(progress) if progress else None
    progress_end = max(progress) if progress else None
    progress_span = (
        progress_end - progress_start
        if progress_start is not None and progress_end is not None
        else None
    )
    backward_steps = sum(
        current < previous for previous, current in zip(progress, progress[1:])
    )
    coverage = None
    if route_point_count > 1 and progress_span is not None:
        coverage = progress_span / float(route_point_count - 1)

    time_values = _finite_numbers(record["timestamp_ms"] for record in records)
    time_span_ms = max(time_values) - min(time_values) if time_values else 0
    return {
        "sample_count": len(records),
        "time_span_ms": time_span_ms,
        "effective_sample_rate_hz": (
            len(records) * 1000.0 / time_span_ms if time_span_ms > 0 else None
        ),
        "progress": {
            "start": progress_start,
            "end": progress_end,
            "span": progress_span,
            "route_point_count": route_point_count,
            "coverage_ratio": coverage,
            "backward_steps": backward_steps,
        },
        "errors": {
            "cross_track_mm": _series_stats(
                record["cross_track_mm"] for record in records
            ),
            "along_track_mm": _series_stats(
                record["along_track_mm"] for record in records
            ),
            "heading_actual_minus_ideal_deg": _series_stats(
                record["heading_error_deg"] for record in records
            ),
            "pure_pursuit_target_minus_actual_deg": _series_stats(
                record["pure_pursuit_error_deg"] for record in records
            ),
            "map_pose_path_distance_mm": _series_stats(
                record["map_pose_path_distance_mm"] for record in records
            ),
            "raw_pose_path_distance_mm": _series_stats(
                record["raw_pose_path_distance_mm"] for record in records
            ),
            "x_actual_minus_ideal_mm": _series_stats(x_errors),
            "y_actual_minus_ideal_mm": _series_stats(y_errors),
        },
        "signals": {
            "lookahead_command_mm": _series_stats(
                record["lookahead_command_mm"] for record in records
            ),
            "lookahead_base_mm": _series_stats(
                record["lookahead_base_mm"] for record in records
            ),
            "lookahead_control_alignment_age_ms": _series_stats(
                record["lookahead_control_alignment_age_ms"]
                for record in records
            ),
            "raw_x_mm": _series_stats(
                record["raw_x_mm"] for record in records
            ),
            "raw_y_mm": _series_stats(
                record["raw_y_mm"] for record in records
            ),
            "raw_yaw_deg": _series_stats(
                record["raw_yaw_deg"] for record in records
            ),
            "map_minus_raw_position_mm": _series_stats(
                record["map_minus_raw_position_mm"] for record in records
            ),
            "map_from_odom_tx_mm": _series_stats(
                record["map_from_odom_tx_mm"] for record in records
            ),
            "map_from_odom_ty_mm": _series_stats(
                record["map_from_odom_ty_mm"] for record in records
            ),
            "map_from_odom_yaw_deg": _series_stats(
                record["map_from_odom_yaw_deg"] for record in records
            ),
            "map_from_odom_translation_norm_mm": _series_stats(
                record["map_from_odom_translation_norm_mm"]
                for record in records
            ),
            # Straight-line/bench runs intentionally do not emit navigation
            # records.  Keep the summary useful for those runs instead of
            # indexing an empty list after the CSVs have already been written.
            "localization_update_count": (
                (
                    records[-1]["localization_update_seq"]
                    - records[0]["localization_update_seq"]
                ) & 0xFFFF
                if records
                else 0
            ),
            "target_speed_cmps": _series_stats(
                record["target_speed_cmps"] for record in records
            ),
            "curvature_per_mm": _series_stats(
                record["curvature_per_mm"] for record in records
            ),
            "curvature_derivative_per_mm2": _series_stats(
                record["curvature_derivative_per_mm2"]
                for record in records
            ),
            "reference_acceleration_cmps2": _series_stats(
                record["reference_acceleration_cmps2"]
                for record in records
            ),
            "reference_yaw_rate_dps": _series_stats(
                record["reference_yaw_rate_dps"] for record in records
            ),
            "reference_lateral_acceleration_cmps2": _series_stats(
                record["reference_lateral_acceleration_cmps2"]
                for record in records
            ),
        },
    }


def _analyze_localization(records):
    accepted = [row for row in records if row.get("status_name") == "accepted"]
    rejected = [row for row in records if row.get("status_name") == "rejected"]
    return {
        "transaction_count": len(records),
        "accepted_count": len(accepted),
        "rejected_count": len(rejected),
        "acceptance_ratio": _ratio(len(accepted), len(records)),
        "candidate_valid_count": sum(
            bool(row.get("candidate_transform_valid")) for row in records
        ),
        "accepted_delta_translation_mm": _series_stats(
            row["delta_odom_translation_mm"] for row in accepted
        ),
        "accepted_delta_yaw_abs_deg": _series_stats(
            row["delta_odom_yaw_abs_deg"] for row in accepted
        ),
        "accepted_map_pose_jump_mm": _series_stats(
            row["map_pose_jump_mm"] for row in accepted
        ),
        "accepted_progress_delta_mm": _series_stats(
            row["progress_delta_mm"] for row in accepted
        ),
        "accepted_cross_track_before_mm": _series_stats(
            row["map_before_cross_track_mm"] for row in accepted
        ),
        "accepted_cross_track_after_mm": _series_stats(
            row["map_after_cross_track_mm"] for row in accepted
        ),
        "accepted_cross_track_delta_mm": _series_stats(
            row["cross_track_delta_mm"] for row in accepted
        ),
        "accepted_along_track_delta_mm": _series_stats(
            row["along_track_delta_mm"] for row in accepted
        ),
        "accepted_heading_error_delta_deg": _series_stats(
            row["heading_error_delta_deg"] for row in accepted
        ),
        "accepted_path_distance_after_mm": _series_stats(
            row["map_after_path_distance_mm"] for row in accepted
        ),
        "accepted_pp_p_turn_delta_cmps": _series_stats(
            row["pp_p_turn_delta_cmps"] for row in accepted
        ),
        "accepted_control_alignment_age_ms": _series_stats(
            row["localization_control_alignment_age_ms"]
            for row in accepted
        ),
        "accepted_rms_before_mm": _series_stats(
            row["rms_before_mm"] for row in accepted
        ),
        "accepted_rms_after_mm": _series_stats(
            row["rms_after_mm"] for row in accepted
        ),
        "accepted_inlier_count": _series_stats(
            row["inlier_count"] for row in accepted
        ),
        "rejected_candidate_delta_translation_mm": _series_stats(
            row["delta_odom_translation_mm"]
            for row in rejected if row.get("candidate_transform_valid")
        ),
        "rejected_candidate_delta_yaw_abs_deg": _series_stats(
            row["delta_odom_yaw_abs_deg"]
            for row in rejected if row.get("candidate_transform_valid")
        ),
        "rejected_candidate_cross_track_delta_mm": _series_stats(
            row["candidate_cross_track_delta_mm"]
            for row in rejected if row.get("candidate_transform_valid")
        ),
        "rejected_candidate_along_track_delta_mm": _series_stats(
            row["candidate_along_track_delta_mm"]
            for row in rejected if row.get("candidate_transform_valid")
        ),
        "rejected_candidate_heading_error_delta_deg": _series_stats(
            row["candidate_heading_error_delta_deg"]
            for row in rejected if row.get("candidate_transform_valid")
        ),
        "rejected_candidate_pp_p_turn_delta_cmps": _series_stats(
            row["candidate_pp_p_turn_delta_cmps"]
            for row in rejected if row.get("candidate_transform_valid")
        ),
    }


def _analyze_vision(records):
    reason_counts = {}
    for record in records:
        reason = record["reject_reason"]
        name = record["reject_reason_name"]
        entry = reason_counts.setdefault(
            str(reason), {"name": name, "count": 0}
        )
        entry["count"] += 1
    accepted_count = sum(1 for record in records if record["line_valid"])
    ambiguous_count = sum(
        1 for record in records if record["selection_ambiguous"] is True
    )
    full_coverage_count = sum(
        1 for record in records if record["full_coverage"] is True
    )
    return {
        "sample_count": len(records),
        "accepted_count": accepted_count,
        "accepted_ratio": _ratio(accepted_count, len(records)),
        "ambiguous_count": ambiguous_count,
        "ambiguous_ratio": _ratio(ambiguous_count, len(records)),
        "full_coverage_count": full_coverage_count,
        "full_coverage_ratio": _ratio(full_coverage_count, len(records)),
        "signals": {
            "line_position_mm": _series_stats(
                record["line_position_mm"] for record in records
            ),
            "active_sensor_count": _series_stats(
                record["active_sensor_count"] for record in records
            ),
            "active_span_sensors": _series_stats(
                record["active_span_sensors"] for record in records
            ),
            "active_block_count": _series_stats(
                record["active_block_count"] for record in records
            ),
            "peak_count": _series_stats(
                record["peak_count"] for record in records
            ),
            "white_percent": _series_stats(
                record["white_percent"] for record in records
            ),
            "threshold_gray": _series_stats(
                record["threshold_gray"] for record in records
            ),
        },
        "reject_reasons": reason_counts,
    }


def _analyze_events(records, localization_records=None):
    type_counts = {}
    source_counts = {}
    value0_bit_counts = {}
    value1_bit_counts = {}
    runtime_budget = {}
    for record in records:
        event_type = record["type"]
        source = record["source"]
        type_entry = type_counts.setdefault(
            str(event_type),
            {"name": EVENT_TYPE_NAMES.get(event_type, f"unknown_{event_type}"), "count": 0},
        )
        type_entry["count"] += 1
        source_entry = source_counts.setdefault(
            str(source),
            {"name": EVENT_SOURCE_NAMES.get(source, f"unknown_{source}"), "count": 0},
        )
        source_entry["count"] += 1
        if event_type == 8:
            for bit, name in CONTROL_LIMIT_BIT_NAMES.items():
                if record["value0"] & bit:
                    value0_bit_counts[name] = value0_bit_counts.get(name, 0) + 1
                if record["value1"] & bit:
                    value1_bit_counts[name] = value1_bit_counts.get(name, 0) + 1
        elif event_type == 10:
            code = record["code"]
            name = RUNTIME_BUDGET_CODE_NAMES.get(code, f"unknown_{code}")
            if code <= 6:
                runtime_budget[name] = {
                    "max_us": record["value0"],
                    "overrun_count": record["value1"],
                }
            elif code == 7:
                runtime_budget[name] = {
                    "read_failure_count": record["value0"],
                    "max_consecutive_failure_count": record["value1"],
                }
            elif code == 8:
                runtime_budget[name] = {
                    "max_age_us": record["value0"],
                    "last_age_us": record["value1"],
                }
            elif code == 9:
                runtime_budget[name] = {
                    "filtered_frame_count": record["value0"],
                    "ack_timeout_count": record["value1"],
                }
            elif code == 10:
                runtime_budget[name] = {
                    "max_us": record["value0"],
                    "overrun_count": record["value1"],
                }
            elif code == 11:
                runtime_budget[name] = {
                    "sample_failure_count": record["value0"],
                    "sample_gap_count": record["value1"],
                }
            elif code == 12:
                runtime_budget[name] = {
                    "adc_failure_count": record["value0"],
                }
            else:
                runtime_budget[name] = {
                    "value0": record["value0"],
                    "value1": record["value1"],
                }

    localization_records = localization_records or []
    localization_status_counts = {}
    localization_reject_counts = {}
    for record in localization_records:
        status = record.get("status_name", "unknown")
        localization_status_counts[status] = (
            localization_status_counts.get(status, 0) + 1
        )
        reason = record.get("reject_reason_name")
        if status == "rejected" and reason:
            localization_reject_counts[reason] = (
                localization_reject_counts.get(reason, 0) + 1
            )

    return {
        "event_count": len(records),
        "type_counts": type_counts,
        "source_counts": source_counts,
        "control_limit_events": sum(1 for record in records if record["type"] == 8),
        "protection_events": sum(1 for record in records if record["type"] == 9),
        "telemetry_drop_events": sum(1 for record in records if record["type"] == 6),
        "localization_transaction_count": len(localization_records),
        "localization_status_counts": localization_status_counts,
        "localization_reject_counts": localization_reject_counts,
        "control_limit_value0_bit_counts": value0_bit_counts,
        "control_limit_value1_bit_counts": value1_bit_counts,
        "runtime_budget": runtime_budget,
    }


def _build_diagnostic_flags(decoded):
    metadata = decoded["metadata"]
    control = decoded["control"]
    navigation = decoded["navigation"]
    vision = decoded["vision"]
    imu = decoded.get("imu", [])
    events = decoded["events"]
    drops = metadata["drops"]
    flags = []
    # 直线速度/前馈测试主动绕过地图导航；这类运行没有 navigation 记录并不
    # 影响 control.csv 的有效性，不能把“未走地图”误报成速度数据损坏。
    speed_test_like = any(
        abs(record.get("plan_v_cmps", 0.0)) > 1.0
        or abs(record.get("accel_l_cmps2", 0.0)) > 1.0
        or abs(record.get("accel_r_cmps2", 0.0)) > 1.0
        for record in control
    )

    if not control:
        flags.append({"id": "missing_control", "severity": "error", "evidence": "control count is 0"})
    if not navigation and not speed_test_like:
        flags.append({"id": "missing_navigation", "severity": "warning", "evidence": "navigation count is 0"})
    if not vision:
        flags.append({"id": "missing_vision", "severity": "warning", "evidence": "vision count is 0"})
    if not imu:
        flags.append({"id": "missing_imu", "severity": "error", "evidence": "IMU count is 0"})
    if any(value > 0 for value in drops.values()):
        flags.append({"id": "telemetry_drops", "severity": "warning", "evidence": drops})
    scan_metadata = metadata.get("phototube_scan", {})
    if scan_metadata.get("localization_ledger_gap_count", 0):
        flags.append({
            "id": "localization_ledger_gap",
            "severity": "warning",
            "evidence": {
                "gap_count": scan_metadata.get(
                    "localization_ledger_gap_count", 0
                ),
                "navigation_fallback_points": scan_metadata.get(
                    "localization_projection_fallback_point_count", 0
                ),
                "unresolved_points": scan_metadata.get(
                    "localization_projection_unresolved_point_count", 0
                ),
            },
        })
    if scan_metadata.get(
            "localization_projection_unresolved_point_count", 0):
        flags.append({
            "id": "localization_projection_unresolved",
            "severity": "error",
            "evidence": {
                "point_count": scan_metadata[
                    "localization_projection_unresolved_point_count"
                ]
            },
        })
    if any(record["type"] == 9 for record in events):
        flags.append({"id": "protection_event", "severity": "error", "evidence": "event type 9 exists"})
    if any(record["type"] == 8 for record in events):
        flags.append({"id": "control_limit_event", "severity": "warning", "evidence": "event type 8 exists"})

    if navigation:
        progress = [record["progress_index"] for record in navigation]
        if any(current < previous for previous, current in zip(progress, progress[1:])):
            flags.append({"id": "progress_reversal", "severity": "warning", "evidence": "progress_index moved backwards"})
        cross_track = _series_stats(record["cross_track_mm"] for record in navigation)
        if cross_track.get("p95_abs", 0) > 20.0:
            flags.append({
                "id": "large_cross_track_error",
                "severity": "review",
                "evidence": {"cross_track_p95_abs_mm": cross_track["p95_abs"], "heuristic_threshold_mm": 20.0},
            })

        if len(set(progress)) <= 1 and not speed_test_like:
            flags.append({
                "id": "navigation_not_moving",
                "severity": "error",
                "evidence": {"progress_start": progress[0], "progress_end": progress[-1]},
            })

    if control:
        speed_error = _series_stats(
            (record["actual_l_cmps"] + record["actual_r_cmps"]) / 2.0 - record["plan_v_cmps"]
            for record in control
        )
        if speed_error.get("rms", 0) > 5.0:
            flags.append({
                "id": "speed_tracking_error",
                "severity": "review",
                "evidence": {"speed_error_rms_cmps": speed_error["rms"], "heuristic_threshold_cmps": 5.0},
            })

    if vision and _ratio(
        sum(record["line_valid"] for record in vision), len(vision)
    ) < 0.70:
        flags.append({
            "id": "low_vision_acceptance",
            "severity": "review",
            "evidence": {"accepted_ratio": _ratio(sum(record["line_valid"] for record in vision), len(vision)), "heuristic_threshold": 0.70},
        })
    localization_features = decoded.get("localization_features", [])
    failed_features = [
        record["route_feature_occurrence"]
        for record in localization_features
        if record.get("outcome") == "failed"
    ]
    missed_features = [
        record["route_feature_occurrence"]
        for record in localization_features
        if record.get("outcome") == "passed_without_candidate"
    ]
    if failed_features:
        flags.append({
            "id": "localization_feature_failed",
            "severity": "warning",
            "evidence": {"feature_occurrences": failed_features},
        })
    if missed_features:
        flags.append({
            "id": "localization_feature_candidate_missed",
            "severity": "error",
            "evidence": {"feature_occurrences": missed_features},
        })
    return flags


def _build_codex_packet(decoded, planned_route=None):
    metadata = decoded["metadata"]
    packet = {
        "analysis_schema_version": ANALYSIS_SCHEMA_VERSION,
        "purpose": "TC264 smart-car telemetry parameter analysis; generated from one frozen EMEM run",
        "data_contract": {
            "control_speed_unit": "cm/s",
            "navigation_position_unit": "mm",
            "angle_unit": "degree",
            "imu_acceleration_unit": "g",
            "imu_angular_rate_unit": "deg/s",
            "imu_quaternion_unit": "unit quaternion",
            "imu_timestamp_unit": "run-relative us",
            "vision_position_unit": "mm (array estimate)",
            "alignment_key": "navigation.progress_index",
            "control_tick_note": "control.tick_ms uses the shared run-relative millisecond clock; equal navigation timestamps identify the same control tick, while progress_index remains the preferred route-position comparison key",
            "raw_scan_note": "phototube_scan.x_mm/y_mm and reconstructed raw_axle_x/y remain immutable raw odom; map_x/y and map_axle_x/y use the accepted localization transform ledger at capture, with explicitly labelled navigation-snapshot fallback only across retained-ledger gaps; unresolved gaps stay unprojected, while final_map_* use the best final accepted/navigation transform",
        },
        "parameter_snapshot": metadata["parameters"],
        "runtime_constants": metadata["constants"],
        "control_config": metadata.get("control_config", {}),
        "localization_config": metadata["localization_config"],
        "run_quality": {
            "state": metadata["state"],
            "stop_reason": metadata["stop_reason"],
            "stop_reason_name": metadata.get("stop_reason_name"),
            "final_prestart_state": metadata["final_prestart_state"],
            "final_prestart_state_name": metadata.get("final_prestart_state_name"),
            "final_nav_state": metadata["final_nav_state"],
            "final_nav_state_name": metadata.get("final_nav_state_name"),
            "duration_ms": metadata["duration_ms"],
            "counts": metadata["counts"],
            "drops": metadata["drops"],
            "route_index": metadata["route_index"],
            "route_hash": metadata["route_hash"],
            "route_point_count": metadata["route_point_count"],
            "firmware_config_fingerprint": metadata["firmware_config_fingerprint"],
        },
        "control_summary": _analyze_control(decoded["control"]),
        "speed_tuning_summary": _speed_tuning_summary(decoded["control"]),
        "imu_summary": _analyze_imu(decoded.get("imu", [])),
        "navigation_summary": _analyze_navigation(
            decoded["navigation"], metadata["route_point_count"]
        ),
        "localization_summary": _analyze_localization(
            decoded.get("localization", [])
        ),
        "vision_summary": _analyze_vision(decoded["vision"]),
        "event_summary": _analyze_events(
            decoded["events"], decoded.get("localization", [])
        ),
        "phototube_scan": {
            key: value
            for key, value in decoded.get("phototube_scan", {}).items()
            if key != "points"
        },
        "localization_feature_diagnostics": decoded.get(
            "localization_features", []
        ),
        "diagnostic_flags": _build_diagnostic_flags(decoded),
        "trace_preview": {
            "control": _downsample(
                decoded["control"], CONTROL_PREVIEW_FIELDS, 80
            ),
            "navigation": _downsample(
                decoded["navigation"], NAVIGATION_PREVIEW_FIELDS, 80
            ),
            "vision": _downsample(
                decoded["vision"], VISION_PREVIEW_FIELDS, 80
            ),
            "imu": _downsample(
                decoded.get("imu", []), IMU_PREVIEW_FIELDS, 120
            ),
            "phototube_scan": _downsample(
                decoded.get("phototube_scan", {}).get("points", []),
                PHOTOTUBE_SCAN_FIELDS,
                120,
            ),
            "planned_route": _downsample(
                planned_route or [], ROUTE_FIELDS, 160
            ),
            "events": decoded["events"],
            "localization": decoded.get("localization", []),
            "localization_features": decoded.get(
                "localization_features", []
            ),
        },
        "source_files": [
            "telemetry.bin",
            "metadata.json",
            "control.csv",
            "navigation.csv",
            "phototube.csv",
            "events.csv",
            "localization_transactions.csv",
            "localization_feature_diagnostics.csv",
            "imu.csv",
            "phototube_scan.csv",
            "planned_route.csv",
        ],
    }
    return _json_safe(packet)


def _stats_abs_max(stats):
    values = [stats.get("min"), stats.get("max")]
    values = [abs(value) for value in values if isinstance(value, (int, float))]
    return max(values) if values else None


def _build_analysis_summary(decoded, packet):
    """Build the small, first-read artifact used before any CSV drill-down."""
    control = decoded["control"]
    navigation = packet["navigation_summary"]
    vision = packet["vision_summary"]
    imu = packet["imu_summary"]
    speed = packet["speed_tuning_summary"]
    localization = packet.get("localization_summary", {})
    visible_parameters = {
        name: value
        for name, value in packet["parameter_snapshot"].items()
        if not name.startswith("reserved_")
    }

    final_turn = _series_stats(record["final_turn_cmps"] for record in control)
    tracking_turn = _series_stats(
        record["tracking_turn_cmps"] for record in control
    )
    yaw_rate_error = _series_stats(
        record["yaw_rate_error_degps"] for record in control
    )
    pd_p_turn = _series_stats(
        record["pd_p_turn_inferred_cmps"] for record in control
    )
    pd_d_turn = _series_stats(record["pd_d_turn_cmps"] for record in control)
    gkd_turn = _series_stats(record["gkd_turn_cmps"] for record in control)
    yaw_rate_feedforward_turn = _series_stats(
        record["yaw_rate_feedforward_turn_cmps"] for record in control
    )
    yaw_rate_feedback_turn = _series_stats(
        record["yaw_rate_feedback_turn_cmps"] for record in control
    )
    line_position = _series_stats(record["line_position_mm"] for record in control)
    actual_average = _series_stats(record["actual_avg_cmps"] for record in control)
    imu_signals = imu.get("signals", {})
    navigation_errors = navigation.get("errors", {})
    vision_signals = vision.get("signals", {})
    flags = packet["diagnostic_flags"]
    localization_features = decoded.get("localization_features", [])
    localization_feature_outcomes = {
        outcome: sum(
            1 for record in localization_features
            if record.get("outcome") == outcome
        )
        for outcome in (
            "accepted", "partial", "failed", "passed_without_candidate",
            "disabled_by_map", "not_reached"
        )
    }
    localization_feature_problems = [
        {
            "feature": f"F{record['route_feature_occurrence']}",
            "type": record["feature_type_name"],
            "progress_points": record["feature_progress_points"],
            "outcome": record["outcome"],
            "attempt_count": record["attempt_count"],
            "last_reject_reason": record["last_reject_reason_name"],
            "last_failure_details": record["last_detail_names"],
            "all_failure_details": record["detail_names"],
            "line_point_counts": [
                record["line0_point_count"],
                record["line1_point_count"],
                record["line2_point_count"],
            ],
            "line_frame_counts": [
                record["line0_frame_count"],
                record["line1_frame_count"],
                record["line2_frame_count"],
            ],
            "failure_value": record["failure_value"],
            "failure_value_name": record["failure_value_name"],
            "failure_value_unit": record["failure_value_unit"],
        }
        for record in localization_features
        if record.get("outcome") in (
            "failed", "passed_without_candidate", "disabled_by_map"
        )
    ]

    return _json_safe({
        "analysis_schema_version": ANALYSIS_SCHEMA_VERSION,
        "read_order": [
            "View telemetry_overview.png as an image.",
            "Read this analysis_summary.json for exact values.",
            "Query only the relevant columns and time window from CSV if needed.",
        ],
        "run": packet["run_quality"],
        "parameters": visible_parameters,
        "controller": packet.get("control_config", {}),
        "quality": {
            "usable_without_drilldown": not any(
                flag.get("severity") == "error" for flag in flags
            ),
            "diagnostic_flags": flags,
        },
        "metrics": {
            "speed": {
                "sample_count": speed.get("sample_count"),
                "time_span_ms": speed.get("time_span_ms"),
                "tuning_usable": speed.get("usable"),
                "plan_min_cmps": speed.get("target_speed_min_cmps"),
                "plan_max_cmps": speed.get("target_speed_max_cmps"),
                "actual_mean_cmps": actual_average.get("mean"),
                "actual_abs_max_cmps": _stats_abs_max(actual_average),
                "error_mean_cmps": speed.get("speed_error_mean_cmps"),
                "error_rms_cmps": speed.get("speed_error_rms_cmps"),
                "error_p95_abs_cmps": speed.get("speed_error_p95_abs_cmps"),
                "wheel_balance_p95_abs_cmps": speed.get(
                    "wheel_balance_actual_p95_abs_cmps"
                ),
                "pwm_l_abs_max": speed.get("pwm_l_abs_max"),
                "pwm_r_abs_max": speed.get("pwm_r_abs_max"),
            },
            "steering": {
                "final_turn_p95_abs_cmps": final_turn.get("p95_abs"),
                "final_turn_abs_max_cmps": _stats_abs_max(final_turn),
                "tracking_turn_p95_abs_cmps": tracking_turn.get("p95_abs"),
                "yaw_rate_error_p95_abs_degps": yaw_rate_error.get("p95_abs"),
                "pd_p_turn_p95_abs_cmps": pd_p_turn.get("p95_abs"),
                "pd_d_turn_p95_abs_cmps": pd_d_turn.get("p95_abs"),
                "gkd_turn_p95_abs_cmps": gkd_turn.get("p95_abs"),
                "yaw_rate_feedforward_turn_p95_abs_cmps": (
                    yaw_rate_feedforward_turn.get("p95_abs")
                ),
                "yaw_rate_feedback_turn_p95_abs_cmps": (
                    yaw_rate_feedback_turn.get("p95_abs")
                ),
                "line_position_p95_abs_mm": line_position.get("p95_abs"),
            },
            "navigation": {
                "sample_count": navigation.get("sample_count"),
                "progress": navigation.get("progress"),
                "cross_track_p95_abs_mm": navigation_errors.get(
                    "cross_track_mm", {}
                ).get("p95_abs"),
                "heading_error_p95_abs_deg": navigation_errors.get(
                    "heading_actual_minus_ideal_deg", {}
                ).get("p95_abs"),
                "pure_pursuit_error_p95_abs_deg": navigation_errors.get(
                    "pure_pursuit_target_minus_actual_deg", {}
                ).get("p95_abs"),
                "along_track_p95_abs_mm": navigation_errors.get(
                    "along_track_mm", {}
                ).get("p95_abs"),
                "map_pose_path_distance_p95_mm": navigation_errors.get(
                    "map_pose_path_distance_mm", {}
                ).get("p95_abs"),
            },
            "localization": localization,
            "vision": {
                "sample_count": vision.get("sample_count"),
                "accepted_ratio": vision.get("accepted_ratio"),
                "ambiguous_ratio": vision.get("ambiguous_ratio"),
                "full_coverage_ratio": vision.get("full_coverage_ratio"),
                "position_p95_abs_mm": vision_signals.get(
                    "line_position_mm", {}
                ).get("p95_abs"),
                "active_sensor_mean": vision_signals.get(
                    "active_sensor_count", {}
                ).get("mean"),
                "white_percent_mean": vision_signals.get(
                    "white_percent", {}
                ).get("mean"),
                "peak_count_mean": vision_signals.get(
                    "peak_count", {}
                ).get("mean"),
                "threshold_gray_mean": vision_signals.get(
                    "threshold_gray", {}
                ).get("mean"),
                "reject_reasons": vision.get("reject_reasons", {}),
            },
            "imu": {
                "sample_count": imu.get("sample_count"),
                "effective_sample_rate_hz": imu.get("effective_sample_rate_hz"),
                "gyro_z_p95_abs_dps": imu_signals.get("gyro_z_dps", {}).get(
                    "p95_abs"
                ),
                "gyro_z_abs_max_dps": _stats_abs_max(
                    imu_signals.get("gyro_z_dps", {})
                ),
                "roll_p95_abs_deg": imu_signals.get("roll_deg", {}).get(
                    "p95_abs"
                ),
                "pitch_p95_abs_deg": imu_signals.get("pitch_deg", {}).get(
                    "p95_abs"
                ),
                "acc_norm_mean_g": imu_signals.get("acc_norm_g", {}).get("mean"),
                "quaternion_norm_mean": imu_signals.get(
                    "quaternion_norm", {}
                ).get("mean"),
            },
            "events": packet["event_summary"],
            "localization_features": {
                "registered_count": len(localization_features),
                "localization_transaction_overwrite_count": decoded[
                    "metadata"
                ].get("localization", {}).get("overwrite_count", 0),
                "diagnostic_drop_count": decoded["metadata"].get(
                    "localization_feature_diagnostics", {}
                ).get(
                    "diagnostic_drop_count", 0
                ),
                "outcome_counts": localization_feature_outcomes,
                "problems": localization_feature_problems,
            },
            "phototube_scan": packet.get("phototube_scan", {}),
        },
        "drilldown_files": {
            "speed_or_steering": "control.csv",
            "path": "navigation.csv",
            "localization": "localization_transactions.csv",
            "localization_features": "localization_feature_diagnostics.csv",
            "phototube_array": "phototube.csv",
            "body_motion": "imu.csv",
            "stop_or_limit": "events.csv",
            "scan_geometry": "phototube_scan.csv",
        },
    })


def _svg_number(value, digits=1):
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        return "--"
    return f"{value:.{digits}f}"


def _finite_xy_points(values):
    return [
        (float(point[0]), float(point[1]))
        for point in values
        if (
            isinstance(point, (tuple, list))
            and len(point) >= 2
            and isinstance(point[0], (int, float))
            and isinstance(point[1], (int, float))
            and math.isfinite(point[0])
            and math.isfinite(point[1])
        )
    ]


def _svg_line_chart(parts, x, y, width, height, title, series, include_zero=False):
    parts.append(
        f'<rect x="{x}" y="{y}" width="{width}" height="{height}" rx="8" '
        'fill="#ffffff" stroke="#d8dee8"/>'
    )
    parts.append(
        f'<text x="{x + 16}" y="{y + 25}" class="panel-title">'
        f'{html.escape(title)}</text>'
    )
    plot_x = x + 54
    plot_y = y + 38
    plot_w = width - 72
    plot_h = height - 66
    series = [
        (label, color, _finite_xy_points(values))
        for label, color, values in series
    ]
    points = [point for item in series for point in item[2]]
    if not points:
        parts.append(
            f'<text x="{x + width / 2}" y="{y + height / 2}" '
            'class="empty" text-anchor="middle">no data</text>'
        )
        return
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    if include_zero:
        ys.append(0.0)
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    if x_max <= x_min:
        x_max = x_min + 1.0
    if y_max <= y_min:
        y_max = y_min + 1.0
    y_pad = (y_max - y_min) * 0.08
    y_min -= y_pad
    y_max += y_pad

    def project(point):
        px = plot_x + (point[0] - x_min) * plot_w / (x_max - x_min)
        py = plot_y + plot_h - (point[1] - y_min) * plot_h / (y_max - y_min)
        return px, py

    for grid_index in range(5):
        gy = plot_y + grid_index * plot_h / 4.0
        value = y_max - grid_index * (y_max - y_min) / 4.0
        parts.append(
            f'<line x1="{plot_x}" y1="{gy:.1f}" x2="{plot_x + plot_w}" '
            f'y2="{gy:.1f}" stroke="#edf0f4"/>'
        )
        parts.append(
            f'<text x="{plot_x - 7}" y="{gy + 4:.1f}" class="axis" '
            f'text-anchor="end">{_svg_number(value, 1)}</text>'
        )
    for legend_index, (label, color, values) in enumerate(series):
        if not values:
            continue
        polyline = " ".join(
            f"{px:.1f},{py:.1f}" for px, py in map(project, values)
        )
        parts.append(
            f'<polyline points="{polyline}" fill="none" stroke="{color}" '
            'stroke-width="2" stroke-linejoin="round" stroke-linecap="round"/>'
        )
        legend_x = plot_x + legend_index * 140
        parts.append(
            f'<line x1="{legend_x}" y1="{y + height - 13}" '
            f'x2="{legend_x + 18}" y2="{y + height - 13}" '
            f'stroke="{color}" stroke-width="3"/>'
        )
        parts.append(
            f'<text x="{legend_x + 23}" y="{y + height - 9}" class="legend">'
            f'{html.escape(label)}</text>'
        )
    parts.append(
        f'<text x="{plot_x}" y="{y + height - 9}" class="axis">'
        f'{_svg_number(x_min, 0)} ms</text>'
    )
    parts.append(
        f'<text x="{plot_x + plot_w}" y="{y + height - 9}" '
        f'class="axis" text-anchor="end">{_svg_number(x_max, 0)} ms</text>'
    )


def _svg_route_chart(
    parts, x, y, width, height, planned, actual, scan, scan_annotations=None
):
    parts.append(
        f'<rect x="{x}" y="{y}" width="{width}" height="{height}" rx="8" '
        'fill="#ffffff" stroke="#d8dee8"/>'
    )
    parts.append(
        f'<text x="{x + 16}" y="{y + 25}" class="panel-title">'
        'A 路径与转向 · 路线平面图</text>'
    )
    series = [
        ("规划", "#98a2b3", planned),
        ("地图位姿", "#e5484d", actual),
        ("扫图(采集时地图)", "#0f9d87", scan),
    ]
    all_points = [point for _, _, values in series for point in values]
    if not all_points:
        parts.append(
            f'<text x="{x + width / 2}" y="{y + height / 2}" '
            'class="empty" text-anchor="middle">no route data</text>'
        )
        return
    x_min = min(point[0] for point in all_points)
    x_max = max(point[0] for point in all_points)
    y_min = min(point[1] for point in all_points)
    y_max = max(point[1] for point in all_points)
    span_x = max(x_max - x_min, 1.0)
    span_y = max(y_max - y_min, 1.0)
    plot_x, plot_y = x + 45, y + 38
    plot_w, plot_h = width - 68, height - 72
    scale = min(plot_w / span_x, plot_h / span_y)
    used_w, used_h = span_x * scale, span_y * scale
    offset_x = plot_x + (plot_w - used_w) / 2.0
    offset_y = plot_y + (plot_h - used_h) / 2.0

    def project(point):
        return (
            offset_x + (point[0] - x_min) * scale,
            offset_y + (point[1] - y_min) * scale,
        )

    for legend_index, (label, color, values) in enumerate(series):
        if not values:
            continue
        projected = list(map(project, values))
        if label == "扫图(采集时地图)":
            for px, py in projected:
                parts.append(
                    f'<circle cx="{px:.1f}" cy="{py:.1f}" r="2.2" fill="{color}"/>'
                )
        else:
            polyline = " ".join(f"{px:.1f},{py:.1f}" for px, py in projected)
            parts.append(
                f'<polyline points="{polyline}" fill="none" stroke="{color}" '
                f'stroke-width="{1.4 if label == "规划" else 2.4}"/>'
            )
        legend_x = x + 16 + legend_index * 105
        parts.append(
            f'<text x="{legend_x}" y="{y + height - 14}" class="legend" '
            f'fill="{color}">● {label}</text>'
        )
    for annotation in scan_annotations or []:
        px, py = project((annotation["x_mm"], annotation["y_mm"]))
        marker_color = _scan_annotation_color(annotation)
        gate_radius = max(7.0, 225.0 * scale)
        parts.append(
            f'<circle cx="{px:.1f}" cy="{py:.1f}" r="{gate_radius:.1f}" '
            f'fill="{marker_color}" fill-opacity="0.08" stroke="{marker_color}" '
            'stroke-width="1.5" stroke-dasharray="3 2"/>'
        )
        if annotation["feature_type_name"] == "junction":
            points = " ".join(
                f"{vx:.1f},{vy:.1f}"
                for vx, vy in (
                    (px, py - 7),
                    (px + 7, py),
                    (px, py + 7),
                    (px - 7, py),
                )
            )
            parts.append(
                f'<polygon points="{points}" fill="#ffffff" stroke="{marker_color}" '
                'stroke-width="2"/>'
            )
        else:
            parts.append(
                f'<circle cx="{px:.1f}" cy="{py:.1f}" r="6.5" fill="#ffffff" '
                f'stroke="{marker_color}" stroke-width="2"/>'
            )
        parts.append(
            f'<text x="{px:.1f}" y="{py + 3.5:.1f}" text-anchor="middle" '
            f'class="axis" fill="{marker_color}">{annotation["feature_index"]}</text>'
        )
        label_right = (annotation["feature_index"] % 3) != 0
        label_x = px + (10.0 if label_right else -10.0)
        label_y = py - 9.0 if (annotation["feature_index"] % 2) else py + 19.0
        anchor = "start" if label_right else "end"
        label = (
            f'V{annotation["feature_index"]} {annotation["type_label"]} '
            f'{annotation["status_label"]}'
        )
        parts.append(
            f'<text x="{label_x:.1f}" y="{label_y:.1f}" class="legend" '
            f'text-anchor="{anchor}" stroke="#ffffff" stroke-width="3" '
            f'paint-order="stroke" fill="{marker_color}">{html.escape(label)}</text>'
        )
        parts.append(
            f'<title>顶点 {annotation["feature_index"]} · '
            f'{html.escape(annotation["type_label"])} · '
            f'{html.escape(annotation["detail_label"])}</title>'
        )


def _build_overview_svg(decoded, planned_route, summary, scan_features=None):
    """Render a compact, dependency-free first-look visualization."""
    control = _downsample(decoded["control"], limit=320)
    navigation = _downsample(decoded["navigation"], limit=320)
    imu = _downsample(decoded.get("imu", []), limit=320)
    planned = _downsample(planned_route or [], ROUTE_FIELDS, 640)
    scan = _downsample(
        decoded.get("phototube_scan", {}).get("points", []),
        PHOTOTUBE_SCAN_FIELDS,
        320,
    )
    scan = _scan_points_in_capture_map(scan)
    scan_annotations = _build_scan_feature_annotations(decoded, scan_features or [])
    run = summary["run"]
    metrics = summary["metrics"]
    flags = summary["quality"]["diagnostic_flags"]
    algorithm_version = int(
        decoded["metadata"].get("tracking_config", {}).get(
            "algorithm_version", 0
        )
    )
    if algorithm_version >= 16:
        turn_components = [
            ("几何前馈", "#2563eb", [
                (r["tick_ms"], r["yaw_rate_feedforward_turn_cmps"])
                for r in control
            ]),
            ("角速度反馈", "#f59e0b", [
                (r["tick_ms"], r["yaw_rate_feedback_turn_cmps"])
                for r in control
            ]),
        ]
    else:
        turn_components = [
            ("P(反推)", "#2563eb", [
                (r["tick_ms"], r["pd_p_turn_inferred_cmps"])
                for r in control
            ]),
            ("D", "#f59e0b", [
                (r["tick_ms"], r["pd_d_turn_cmps"]) for r in control
            ]),
            ("GKD", "#0f9d87", [
                (r["tick_ms"], r["gkd_turn_cmps"]) for r in control
            ]),
        ]
    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1600" height="1180" '
        'viewBox="0 0 1600 1180">',
        '<style>text{font-family:"Segoe UI","Microsoft YaHei",sans-serif;fill:#1f2937}'
        '.title{font-size:26px;font-weight:700}.subtitle{font-size:13px;fill:#667085}'
        '.metric-label{font-size:12px;fill:#667085}.metric-value{font-size:22px;font-weight:700}'
        '.panel-title{font-size:15px;font-weight:650}.axis{font-size:10px;fill:#7b8494}'
        '.legend{font-size:11px;fill:#4b5565}.empty{font-size:14px;fill:#98a2b3}'
        '.flag{font-size:13px}.flag-id{font-weight:700}</style>',
        '<rect width="1600" height="1180" fill="#f5f7fa"/>',
        '<text x="30" y="39" class="title">TC264 EMEM 运行概览</text>',
        f'<text x="30" y="62" class="subtitle">session {run.get("state", "--")} · '
        f'route {run.get("route_index", "--")} · {html.escape(str(run.get("route_hash", "--")))} · '
        f'firmware {html.escape(str(run.get("firmware_config_fingerprint", "--")))}</text>',
    ]

    cards = [
        ("时长", f'{_svg_number(run.get("duration_ms", 0) / 1000.0, 2)} s'),
        ("停车原因", str(run.get("stop_reason_name", "unknown"))),
        ("目标速度峰值", f'{_svg_number(metrics["speed"].get("plan_max_cmps"), 0)} cm/s'),
        ("速度误差 RMS", f'{_svg_number(metrics["speed"].get("error_rms_cmps"), 1)} cm/s'),
        ("最终转向峰值", f'{_svg_number(metrics["steering"].get("final_turn_abs_max_cmps"), 1)} cm/s'),
        ("记录丢失", str(sum(run.get("drops", {}).values()))),
    ]
    for index, (label, value) in enumerate(cards):
        card_x = 30 + index * 258
        parts.append(
            f'<rect x="{card_x}" y="80" width="238" height="72" rx="8" '
            'fill="#ffffff" stroke="#d8dee8"/>'
        )
        parts.append(
            f'<text x="{card_x + 14}" y="103" class="metric-label">'
            f'{html.escape(label)}</text>'
        )
        parts.append(
            f'<text x="{card_x + 14}" y="135" class="metric-value">'
            f'{html.escape(value)}</text>'
        )

    _svg_route_chart(
        parts,
        30,
        170,
        590,
        430,
        [(row["x_mm"], row["y_mm"]) for row in planned],
        [(row["actual_x_mm"], row["actual_y_mm"]) for row in navigation],
        [(row["x_mm"], row["y_mm"]) for row in scan],
        scan_annotations,
    )
    _svg_line_chart(
        parts,
        640,
        170,
        450,
        430,
        "A 路径与转向 · 横向 mm / 航向 deg",
        [
            (
                "横向",
                "#e5484d",
                [(r["timestamp_ms"], r["cross_track_mm"]) for r in navigation],
            ),
            (
                "航向",
                "#7c3aed",
                [
                    (
                        r["timestamp_ms"],
                        _wrap_degrees(r["actual_yaw_deg"] - r["ideal_yaw_deg"]),
                    )
                    for r in navigation
                ],
            ),
        ],
        include_zero=True,
    )
    _svg_line_chart(
        parts,
        1110,
        170,
        460,
        430,
        "A 路径与转向 · 转向因果 (cm/s 半轮差)",
        [
            ("最终", "#e5484d", [(r["tick_ms"], r["final_turn_cmps"]) for r in control]),
            ("控制器总输出", "#7c3aed", [(r["tick_ms"], r["tracking_turn_cmps"]) for r in control]),
        ] + turn_components,
        include_zero=True,
    )
    _svg_line_chart(
        parts,
        30,
        620,
        500,
        210,
        "B 动力执行 · 规划/实际 (cm/s)",
        [
            ("规划", "#2563eb", [(r["tick_ms"], r["plan_v_cmps"]) for r in control]),
            ("实际均值", "#0f9d87", [(r["tick_ms"], r["actual_avg_cmps"]) for r in control]),
        ],
        include_zero=True,
    )
    _svg_line_chart(
        parts,
        550,
        620,
        500,
        210,
        "B 动力执行 · 左右轮目标/实际 (cm/s)",
        [
            ("左实际", "#2563eb", [(r["tick_ms"], r["actual_l_cmps"]) for r in control]),
            ("左目标", "#60a5fa", [(r["tick_ms"], r["target_l_cmps"]) for r in control]),
            ("右实际", "#e5484d", [(r["tick_ms"], r["actual_r_cmps"]) for r in control]),
            ("右目标", "#fb7185", [(r["tick_ms"], r["target_r_cmps"]) for r in control]),
        ],
        include_zero=True,
    )
    _svg_line_chart(
        parts,
        1070,
        620,
        500,
        210,
        "B 动力执行 · 输出 PWM",
        [
            ("左 PWM", "#2563eb", [(r["tick_ms"], r["pwm_l"]) for r in control]),
            ("右 PWM", "#e5484d", [(r["tick_ms"], r["pwm_r"]) for r in control]),
        ],
        include_zero=True,
    )
    _svg_line_chart(
        parts,
        30,
        850,
        500,
        270,
        "C 感知与安全 · 光电位置 (mm)",
        [
            ("线路位置", "#f59e0b", [(r["tick_ms"], r["line_position_mm"]) for r in control]),
        ],
        include_zero=True,
    )
    _svg_line_chart(
        parts,
        550,
        850,
        500,
        270,
        "C 感知与安全 · gyro Z / Roll / Pitch",
        [
            ("gyro Z", "#e5484d", [(r["timestamp_ms"], r["gyro_z_dps"]) for r in imu]),
            ("Roll", "#7c3aed", [(r["timestamp_ms"], r["roll_deg"]) for r in imu]),
            ("Pitch", "#0f9d87", [(r["timestamp_ms"], r["pitch_deg"]) for r in imu]),
        ],
        include_zero=True,
    )

    parts.append(
        '<rect x="1070" y="850" width="500" height="270" rx="8" '
        'fill="#ffffff" stroke="#d8dee8"/>'
    )
    parts.append(
        '<text x="1086" y="877" class="panel-title">'
        'C 感知与安全 · 保护/限幅/诊断</text>'
    )
    if flags:
        for index, flag in enumerate(flags[:9]):
            evidence = json.dumps(flag.get("evidence"), ensure_ascii=False, sort_keys=True)
            if len(evidence) > 54:
                evidence = evidence[:51] + "..."
            color = "#b42318" if flag.get("severity") == "error" else "#b54708"
            parts.append(
                f'<text x="1090" y="{907 + index * 21}" class="flag">'
                f'<tspan fill="{color}" class="flag-id">[{html.escape(str(flag.get("severity")))}] '
                f'{html.escape(str(flag.get("id")))}</tspan>'
                f'<tspan fill="#667085">  {html.escape(evidence)}</tspan></text>'
            )
    else:
        parts.append('<text x="1090" y="909" class="flag" fill="#067647">no diagnostic flags</text>')
    parts.append(
        '<text x="30" y="1160" class="subtitle">阅读顺序 A 路径与转向 → '
        'B 动力执行 → C 感知与安全；默认只读 analysis_summary.json，'
        '需要时再定向查询 CSV。</text>'
    )
    parts.append('</svg>')
    return "\n".join(parts) + "\n"


def _build_overview_png(decoded, planned_route, summary, scan_features=None):
    """Render the same first-look board as a PNG that Codex can inspect directly."""
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError as exc:
        raise RuntimeError(
            "telemetry overview PNG requires Pillow: python -m pip install pillow"
        ) from exc

    width, height = 1600, 1180
    image = Image.new("RGB", (width, height), "#f5f7fa")
    draw = ImageDraw.Draw(image)

    def load_font(size, bold=False):
        candidates = [
            Path("C:/Windows/Fonts/msyhbd.ttc" if bold else "C:/Windows/Fonts/msyh.ttc"),
            Path("C:/Windows/Fonts/seguisb.ttf" if bold else "C:/Windows/Fonts/segoeui.ttf"),
            Path("C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf"),
        ]
        for candidate in candidates:
            if candidate.exists():
                return ImageFont.truetype(str(candidate), size=size)
        return ImageFont.load_default()

    fonts = {
        "title": load_font(27, True),
        "subtitle": load_font(13),
        "metric_label": load_font(12),
        "metric_value": load_font(22, True),
        "panel": load_font(15, True),
        "axis": load_font(10),
        "legend": load_font(11),
        "flag": load_font(12),
        "flag_bold": load_font(12, True),
    }
    colors = {
        "text": "#1f2937",
        "muted": "#667085",
        "axis": "#7b8494",
        "panel": "#ffffff",
        "border": "#d8dee8",
        "grid": "#edf0f4",
    }

    def panel(box, title):
        draw.rounded_rectangle(box, radius=8, fill=colors["panel"], outline=colors["border"])
        draw.text((box[0] + 16, box[1] + 10), title, font=fonts["panel"], fill=colors["text"])

    def line_chart(box, title, series, include_zero=False):
        panel(box, title)
        x, y, right, bottom = box
        plot_x, plot_y = x + 54, y + 38
        plot_w, plot_h = right - x - 72, bottom - y - 66
        series = [
            (label, color, _finite_xy_points(values))
            for label, color, values in series
        ]
        all_points = [point for _, _, values in series for point in values]
        if not all_points:
            draw.text(
                ((x + right) / 2, (y + bottom) / 2),
                "no data",
                font=fonts["panel"],
                fill="#98a2b3",
                anchor="mm",
            )
            return
        xs = [point[0] for point in all_points]
        ys = [point[1] for point in all_points]
        if include_zero:
            ys.append(0.0)
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        if x_max <= x_min:
            x_max = x_min + 1.0
        if y_max <= y_min:
            y_max = y_min + 1.0
        y_pad = (y_max - y_min) * 0.08
        y_min -= y_pad
        y_max += y_pad

        def project(point):
            return (
                plot_x + (point[0] - x_min) * plot_w / (x_max - x_min),
                plot_y + plot_h - (point[1] - y_min) * plot_h / (y_max - y_min),
            )

        for grid_index in range(5):
            gy = plot_y + grid_index * plot_h / 4.0
            value = y_max - grid_index * (y_max - y_min) / 4.0
            draw.line((plot_x, gy, plot_x + plot_w, gy), fill=colors["grid"], width=1)
            draw.text(
                (plot_x - 7, gy),
                _svg_number(value, 1),
                font=fonts["axis"],
                fill=colors["axis"],
                anchor="rm",
            )
        for legend_index, (label, color, values) in enumerate(series):
            if not values:
                continue
            projected = [project(point) for point in values]
            if len(projected) > 1:
                draw.line(projected, fill=color, width=2, joint="curve")
            elif projected:
                px, py = projected[0]
                draw.ellipse((px - 2, py - 2, px + 2, py + 2), fill=color)
            legend_x = plot_x + legend_index * max(92, plot_w // max(len(series), 1))
            legend_y = bottom - 13
            draw.line((legend_x, legend_y, legend_x + 17, legend_y), fill=color, width=3)
            draw.text(
                (legend_x + 22, legend_y),
                label,
                font=fonts["legend"],
                fill=colors["muted"],
                anchor="lm",
            )
        draw.text(
            (plot_x, bottom - 10),
            f"{_svg_number(x_min, 0)} ms",
            font=fonts["axis"],
            fill=colors["axis"],
            anchor="ls",
        )
        draw.text(
            (plot_x + plot_w, bottom - 10),
            f"{_svg_number(x_max, 0)} ms",
            font=fonts["axis"],
            fill=colors["axis"],
            anchor="rs",
        )

    def route_chart(box, planned, actual, scan, scan_annotations=None):
        panel(box, "A 路径与转向 · 路线平面图")
        x, y, right, bottom = box
        all_points = planned + actual + scan
        if not all_points:
            draw.text(
                ((x + right) / 2, (y + bottom) / 2),
                "no route data",
                font=fonts["panel"],
                fill="#98a2b3",
                anchor="mm",
            )
            return
        x_min = min(point[0] for point in all_points)
        x_max = max(point[0] for point in all_points)
        y_min = min(point[1] for point in all_points)
        y_max = max(point[1] for point in all_points)
        span_x = max(x_max - x_min, 1.0)
        span_y = max(y_max - y_min, 1.0)
        plot_x, plot_y = x + 38, y + 38
        plot_w, plot_h = right - x - 58, bottom - y - 72
        scale = min(plot_w / span_x, plot_h / span_y)
        used_w, used_h = span_x * scale, span_y * scale
        offset_x = plot_x + (plot_w - used_w) / 2.0
        offset_y = plot_y + (plot_h - used_h) / 2.0

        def project(point):
            return (
                offset_x + (point[0] - x_min) * scale,
                offset_y + (point[1] - y_min) * scale,
            )

        if len(planned) > 1:
            draw.line([project(point) for point in planned], fill="#98a2b3", width=2)
        if len(actual) > 1:
            draw.line([project(point) for point in actual], fill="#e5484d", width=3)
        for point in scan:
            px, py = project(point)
            draw.ellipse((px - 2, py - 2, px + 2, py + 2), fill="#0f9d87")
        for annotation in scan_annotations or []:
            px, py = project((annotation["x_mm"], annotation["y_mm"]))
            marker_color = _scan_annotation_color(annotation)
            gate_radius = max(7.0, 225.0 * scale)
            draw.ellipse(
                (px - gate_radius, py - gate_radius, px + gate_radius, py + gate_radius),
                outline=marker_color,
                width=2,
            )
            if annotation["feature_type_name"] == "junction":
                draw.polygon(
                    [(px, py - 7), (px + 7, py), (px, py + 7), (px - 7, py)],
                    fill="#ffffff",
                    outline=marker_color,
                )
            else:
                draw.ellipse(
                    (px - 6.5, py - 6.5, px + 6.5, py + 6.5),
                    fill="#ffffff",
                    outline=marker_color,
                    width=2,
                )
            draw.text(
                (px, py),
                str(annotation["feature_index"]),
                font=fonts["axis"],
                fill=marker_color,
                anchor="mm",
            )
            label_right = (annotation["feature_index"] % 3) != 0
            label_x = px + (10.0 if label_right else -10.0)
            label_y = py - 9.0 if (annotation["feature_index"] % 2) else py + 19.0
            label = (
                f'V{annotation["feature_index"]} {annotation["type_label"]} '
                f'{annotation["status_label"]}'
            )
            draw.text(
                (label_x, label_y),
                label,
                font=fonts["legend"],
                fill=marker_color,
                anchor="ls" if label_right else "rs",
                stroke_width=2,
                stroke_fill="#ffffff",
            )
        legend = [
            ("规划", "#98a2b3"),
            ("地图位姿", "#e5484d"),
            ("扫图(采集时地图)", "#0f9d87"),
        ]
        for index, (label, color) in enumerate(legend):
            draw.text(
                (x + 16 + index * 130, bottom - 16),
                f"● {label}",
                font=fonts["legend"],
                fill=color,
                anchor="ls",
            )

    control = _downsample(decoded["control"], limit=320)
    navigation = _downsample(decoded["navigation"], limit=320)
    imu = _downsample(decoded.get("imu", []), limit=320)
    planned = _downsample(planned_route or [], ROUTE_FIELDS, 640)
    scan = _downsample(
        decoded.get("phototube_scan", {}).get("points", []),
        PHOTOTUBE_SCAN_FIELDS,
        320,
    )
    scan = _scan_points_in_capture_map(scan)
    scan_annotations = _build_scan_feature_annotations(decoded, scan_features or [])
    run = summary["run"]
    metrics = summary["metrics"]
    flags = summary["quality"]["diagnostic_flags"]
    algorithm_version = int(
        decoded["metadata"].get("tracking_config", {}).get(
            "algorithm_version", 0
        )
    )
    if algorithm_version >= 16:
        turn_components = [
            ("几何前馈", "#2563eb", [
                (r["tick_ms"], r["yaw_rate_feedforward_turn_cmps"])
                for r in control
            ]),
            ("角速度反馈", "#f59e0b", [
                (r["tick_ms"], r["yaw_rate_feedback_turn_cmps"])
                for r in control
            ]),
        ]
    else:
        turn_components = [
            ("P(反推)", "#2563eb", [
                (r["tick_ms"], r["pd_p_turn_inferred_cmps"])
                for r in control
            ]),
            ("D", "#f59e0b", [
                (r["tick_ms"], r["pd_d_turn_cmps"]) for r in control
            ]),
            ("GKD", "#0f9d87", [
                (r["tick_ms"], r["gkd_turn_cmps"]) for r in control
            ]),
        ]

    draw.text((30, 22), "TC264 EMEM 运行概览", font=fonts["title"], fill=colors["text"])
    draw.text(
        (30, 57),
        f'session {run.get("state", "--")} · route {run.get("route_index", "--")} · '
        f'{run.get("route_hash", "--")} · firmware {run.get("firmware_config_fingerprint", "--")}',
        font=fonts["subtitle"],
        fill=colors["muted"],
    )
    cards = [
        ("时长", f'{_svg_number(run.get("duration_ms", 0) / 1000.0, 2)} s'),
        ("停车原因", str(run.get("stop_reason_name", "unknown"))),
        ("目标速度峰值", f'{_svg_number(metrics["speed"].get("plan_max_cmps"), 0)} cm/s'),
        ("速度误差 RMS", f'{_svg_number(metrics["speed"].get("error_rms_cmps"), 1)} cm/s'),
        ("最终转向峰值", f'{_svg_number(metrics["steering"].get("final_turn_abs_max_cmps"), 1)} cm/s'),
        ("记录丢失", str(sum(run.get("drops", {}).values()))),
    ]
    for index, (label, value) in enumerate(cards):
        left = 30 + index * 258
        draw.rounded_rectangle(
            (left, 80, left + 238, 152),
            radius=8,
            fill=colors["panel"],
            outline=colors["border"],
        )
        draw.text((left + 14, 91), label, font=fonts["metric_label"], fill=colors["muted"])
        draw.text((left + 14, 115), value, font=fonts["metric_value"], fill=colors["text"])

    route_chart(
        (30, 170, 620, 600),
        [(row["x_mm"], row["y_mm"]) for row in planned],
        [(row["actual_x_mm"], row["actual_y_mm"]) for row in navigation],
        [(row["x_mm"], row["y_mm"]) for row in scan],
        scan_annotations,
    )
    line_chart(
        (640, 170, 1090, 600),
        "A 路径与转向 · 横向 mm / 航向 deg",
        [
            ("横向", "#e5484d", [(r["timestamp_ms"], r["cross_track_mm"]) for r in navigation]),
            (
                "航向",
                "#7c3aed",
                [
                    (r["timestamp_ms"], _wrap_degrees(r["actual_yaw_deg"] - r["ideal_yaw_deg"]))
                    for r in navigation
                ],
            ),
        ],
        include_zero=True,
    )
    line_chart(
        (1110, 170, 1570, 600),
        "A 路径与转向 · 转向因果 (cm/s 半轮差)",
        [
            ("最终", "#e5484d", [(r["tick_ms"], r["final_turn_cmps"]) for r in control]),
            ("控制器总输出", "#7c3aed", [(r["tick_ms"], r["tracking_turn_cmps"]) for r in control]),
        ] + turn_components,
        include_zero=True,
    )
    line_chart(
        (30, 620, 530, 830),
        "B 动力执行 · 规划/实际 (cm/s)",
        [
            ("规划", "#2563eb", [(r["tick_ms"], r["plan_v_cmps"]) for r in control]),
            ("实际均值", "#0f9d87", [(r["tick_ms"], r["actual_avg_cmps"]) for r in control]),
        ],
        include_zero=True,
    )
    line_chart(
        (550, 620, 1050, 830),
        "B 动力执行 · 左右轮目标/实际 (cm/s)",
        [
            ("左实", "#2563eb", [(r["tick_ms"], r["actual_l_cmps"]) for r in control]),
            ("左目", "#60a5fa", [(r["tick_ms"], r["target_l_cmps"]) for r in control]),
            ("右实", "#e5484d", [(r["tick_ms"], r["actual_r_cmps"]) for r in control]),
            ("右目", "#fb7185", [(r["tick_ms"], r["target_r_cmps"]) for r in control]),
        ],
        include_zero=True,
    )
    line_chart(
        (1070, 620, 1570, 830),
        "B 动力执行 · 输出 PWM",
        [
            ("左 PWM", "#2563eb", [(r["tick_ms"], r["pwm_l"]) for r in control]),
            ("右 PWM", "#e5484d", [(r["tick_ms"], r["pwm_r"]) for r in control]),
        ],
        include_zero=True,
    )
    line_chart(
        (30, 850, 530, 1120),
        "C 感知与安全 · 光电位置 (mm)",
        [("线路位置", "#f59e0b", [(r["tick_ms"], r["line_position_mm"]) for r in control])],
        include_zero=True,
    )
    line_chart(
        (550, 850, 1050, 1120),
        "C 感知与安全 · gyro Z / Roll / Pitch",
        [
            ("gyro Z", "#e5484d", [(r["timestamp_ms"], r["gyro_z_dps"]) for r in imu]),
            ("Roll", "#7c3aed", [(r["timestamp_ms"], r["roll_deg"]) for r in imu]),
            ("Pitch", "#0f9d87", [(r["timestamp_ms"], r["pitch_deg"]) for r in imu]),
        ],
        include_zero=True,
    )
    panel((1070, 850, 1570, 1120), "C 感知与安全 · 保护/限幅/诊断")
    if flags:
        for index, flag in enumerate(flags[:9]):
            evidence = json.dumps(flag.get("evidence"), ensure_ascii=False, sort_keys=True)
            if len(evidence) > 46:
                evidence = evidence[:43] + "..."
            severity = str(flag.get("severity"))
            color = "#b42318" if severity == "error" else "#b54708"
            baseline = 887 + index * 22
            prefix = f'[{severity}] {flag.get("id")}'
            draw.text((1090, baseline), prefix, font=fonts["flag_bold"], fill=color)
            prefix_width = draw.textlength(prefix, font=fonts["flag_bold"])
            draw.text(
                (1096 + prefix_width, baseline),
                evidence,
                font=fonts["flag"],
                fill=colors["muted"],
            )
    else:
        draw.text((1090, 887), "no diagnostic flags", font=fonts["flag"], fill="#067647")
    draw.text(
        (30, 1146),
        "阅读顺序 A 路径与转向 → B 动力执行 → C 感知与安全；默认只读小摘要，需要时再定向查询 CSV。",
        font=fonts["subtitle"],
        fill=colors["muted"],
    )

    stream = io.BytesIO()
    image.save(stream, format="PNG", optimize=True)
    return stream.getvalue()


def _build_codex_prompt(packet):
    parameters = packet.get("parameter_snapshot", {})
    visible_parameters = {
        name: value
        for name, value in parameters.items()
        if not name.startswith("reserved_")
    }
    flags = packet.get("diagnostic_flags", [])
    flag_ids = [flag["id"] for flag in flags]
    return (
        "# TC264 智能车遥测参数分析任务\n\n"
        "先查看同目录的 `telemetry_overview.png`，再读取 `analysis_summary.json`。不要读取 SVG/XML 或 HTML 源码，也不要整文件读取 CSV。只有概览图和小摘要无法解释某个异常时，才按问题域查询对应 CSV 的少量列和时间窗；`codex_packet.json` 仅作二级下钻。\n"
        "你是本项目的速度环整定工程师，不要修改代码；由你根据本次实车数据决定下一轮参数，用户不需要自行猜参数。\n\n"
        "## 当前运行摘要\n\n"
        f"- 参数快照：`{json.dumps(visible_parameters, ensure_ascii=False, sort_keys=True)}`\n"
        f"- 自动检查标记：`{', '.join(flag_ids) if flag_ids else 'none'}`\n\n"
        f"- 停车原因：`{packet.get('run_quality', {}).get('stop_reason_name', 'unknown')}`；最终发车状态：`{packet.get('run_quality', {}).get('final_prestart_state_name', 'unknown')}`；最终导航状态：`{packet.get('run_quality', {}).get('final_nav_state_name', 'unknown')}`\n\n"
        "## 请按以下顺序分析\n\n"
        "1. 先判断数据是否完整：CRC、记录数量、丢包、保护事件、控制限幅事件。\n"
        "2. 用 `navigation.progress_index` 对齐路径阶段；不要把不同时间基准的时间戳直接硬拼。\n"
        "3. 分开判断速度环、路径横向控制、视觉有效性和执行器限幅，给出每个结论的数值证据。\n"
        "4. 分域判断数据有效性：`missing_control` 或 `speed_tuning_summary.usable=false` 时拒绝速度环调参；`navigation_not_moving` 只阻止路径/地图结论；`low_vision_acceptance` 只阻止视觉结论，不能因为视觉拒帧就否定有足够控制记录的速度环数据。\n"
        "5. 数据有效时必须给出唯一的下一轮执行方案：最多同时改 1--2 个主参数，明确写出旧值 → 新值，不要只给一串让用户自己选择的范围。其余参数明确保持不变。\n"
        "6. 给出下一轮固定路线、固定起步方式、固定速度档位的验证步骤；明确验收指标。\n\n"
        "## 输出格式\n\n"
        "### 1. 数据有效性\n"
        "### 2. 主要问题与证据\n"
        "### 3. 速度环唯一调参结论\n"
        "列：参数、当前值、建议下一值/范围、修改理由、预期变化、风险、验收指标。\n"
        "必须写成旧值 → 新值；如果保持不变，也要明确写出保持不变。\n"
        "### 4. 下一轮实验\n"
        "### 5. 路径/视觉附带结论\n"
        "### 6. 若证据不足\n"
        "明确还需要哪一类数据，不要编造结论。\n\n"
        "## 调参约束\n\n"
        "- 不要同时大幅修改 `speed_KP`、`speed_KI`、限幅参数和路径参数。\n"
        "- 当前固件的 `pi_increment()` 实际使用 `speed_KP`、`speed_KI` 和 `speed_IncrementLimit`；参数快照中的 `reserved_*` 槽位不参与控制。\n"
        "- 速度环优先看实际平均速度与 `plan_v_cmps` 的误差、PWM 和控制限幅事件；不要只看单个峰值。\n"
        "- 若本轮是直线速度环测试，当前固件是预赛完赛版纯增量 PI：`control.csv` 的 `accel_*_cmps2`、`ff_*_duty` 为兼容字段并应为 0，`pi_*_duty` 等于最终 `pwm_*`；只用目标/实际轮速与 PWM 评估 KP/KI 响应。\n"
        "- 路径控制优先看 `cross_track_mm`、`heading_error_deg`、`tracking_turn_cmps` 与 `final_turn_cmps`；v17再用 `curvature_feedforward_yaw_rate_dps`、`target_yaw_rate_degps`、`measured_yaw_rate_degps`、`yaw_rate_error_degps`、`yaw_rate_feedforward_turn_cmps` 和 `yaw_rate_feedback_turn_cmps` 拆分转向来源。`lookahead_*` 与 `pure_pursuit_*` 仅是兼容诊断字段，不是v17控制输入；旧P/D/GKD字段只用于v15及以前日志。\n"
        "- 车身姿态优先看 `imu.csv` 的 acc_x/y/z、gyro_x/y/z、四元数模长和 Roll/Pitch/Yaw；把姿态保护事件与同一时间段的姿态峰值对齐。\n"
        "- 光电参数只有在候选峰数、活动位图、歧义率、固定阈值、线路占比和路径误差相互支持时才建议修改。\n"
        "- `reserved_*` 不是调参项；不确定是否运行时生效的参数要标注“需结合源码确认”。\n"
        "- 输出中必须包含：数据是否可用于调参、唯一推荐参数变更、证据数值、风险、回滚值、下一轮验收阈值，以及用户只需执行的测试动作。\n"
        "- 显示屏、EMEM 初始化不属于本次参数分析范围；陀螺仪/IMU 数据属于本次车身姿态分析输入。\n"
    )


def _html_json(value):
    return json.dumps(
        _json_safe(value), ensure_ascii=False, separators=(",", ":")
    ).replace("</", "<\\/")


def _build_dashboard_html(decoded, planned_route=None, scan_features=None):
    """Create a self-contained offline dashboard for one decoded run."""
    event_analysis = _analyze_events(
        decoded["events"], decoded.get("localization", [])
    )
    dashboard_data = {
        "metadata": decoded["metadata"],
        "control": _downsample(decoded["control"], CONTROL_FIELDS, 900),
        "speed_tuning_summary": _speed_tuning_summary(decoded["control"]),
        "navigation": _downsample(decoded["navigation"], NAVIGATION_FIELDS, 500),
        "vision": _downsample(decoded["vision"], VISION_FIELDS, 700),
        "imu": _downsample(decoded.get("imu", []), IMU_FIELDS, 1400),
        "imu_summary": _analyze_imu(decoded.get("imu", [])),
        "phototube_scan": decoded.get("phototube_scan", {}),
        "planned_route": _downsample(planned_route or [], ROUTE_FIELDS, 900),
        "physical_feature_map": _build_physical_feature_segments(
            scan_features or []
        ),
        "localization": decoded.get("localization", []),
        "localization_features": decoded.get("localization_features", []),
        "events": decoded["events"],
        "runtime_budget": event_analysis.get("runtime_budget", {}),
    }
    data_json = _html_json(dashboard_data)
    yaw_rate_controller = int(
        decoded["metadata"].get("tracking_config", {}).get(
            "algorithm_version", 0
        )
    ) >= 16
    if yaw_rate_controller:
        turn_legend_html = (
            '<span style="color:var(--blue)"><i class="swatch"></i>'
            '最终 turn</span><span style="color:var(--green)"><i class="swatch">'
            '</i>限幅前 turn</span><span style="color:var(--purple)">'
            '<i class="swatch"></i>几何前馈</span><span style="color:var(--orange)">'
            '<i class="swatch"></i>角速度反馈</span>'
        )
    else:
        turn_legend_html = (
            '<span style="color:var(--blue)"><i class="swatch"></i>'
            '最终 turn</span><span style="color:var(--green)"><i class="swatch">'
            '</i>限幅前 turn</span><span style="color:var(--purple)">'
            '<i class="swatch"></i>几何 P</span><span style="color:var(--orange)">'
            '<i class="swatch"></i>D</span><span style="color:var(--red)">'
            '<i class="swatch"></i>GKD</span>'
        )
    return f'''<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>TC264 telemetry dashboard</title>
<style>
:root {{ color-scheme: light dark; --bg: light-dark(#f4f6f8, #101418); --fg: light-dark(#182027, #e8edf2); --muted: light-dark(#5d6a74, #aab6c0); --line: light-dark(#cbd4da, #35424d); --panel: light-dark(#ffffff, #171d22); --blue: light-dark(#1769aa, #65b7ff); --orange: light-dark(#b65b00, #ffad66); --green: light-dark(#147a4a, #70d6a5); --red: light-dark(#b42318, #ff8b82); --purple: light-dark(#6d4bc0, #c2a8ff); }}
* {{ box-sizing: border-box; }}
body {{ margin: 0; padding: 20px; background: var(--bg); color: var(--fg); font: 14px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif; }}
main {{ max-width: 1180px; margin: 0 auto; }}
h1 {{ margin: 0 0 4px; font-size: 22px; font-weight: 500; }}
h2 {{ margin: 0 0 10px; font-size: 15px; font-weight: 500; }}
.muted {{ color: var(--muted); }}
.topline {{ display: flex; flex-wrap: wrap; gap: 6px 18px; margin-bottom: 16px; color: var(--muted); }}
.metrics {{ display: grid; grid-template-columns: repeat(6, minmax(0, 1fr)); gap: 10px; margin-bottom: 16px; }}
.metric {{ background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 10px 12px; }}
.metric-label {{ color: var(--muted); font-size: 12px; }}
.metric-value {{ margin-top: 2px; font-size: 20px; font-variant-numeric: tabular-nums; }}
.grid {{ display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 14px; }}
.panel {{ min-width: 0; }}
.panel.full {{ grid-column: 1 / -1; }}
svg {{ display: block; width: 100%; height: auto; overflow: visible; border: 1px solid var(--line); background: var(--panel); border-radius: 8px; }}
.axis path, .axis line {{ stroke: var(--line); }}
.axis text {{ fill: var(--muted); font-size: 11px; }}
.gridline {{ stroke: var(--line); stroke-opacity: .55; }}
.line {{ fill: none; stroke-width: 1.8; vector-effect: non-scaling-stroke; }}
.event-line {{ stroke: var(--red); stroke-width: 1; stroke-dasharray: 3 3; opacity: .8; vector-effect: non-scaling-stroke; }}
.legend {{ display: flex; flex-wrap: wrap; gap: 6px 14px; margin: 7px 0 0 56px; color: var(--muted); font-size: 12px; }}
.legend span {{ white-space: nowrap; }}
.swatch {{ display: inline-block; width: 18px; height: 2px; margin: 0 5px 3px 0; vertical-align: middle; background: currentColor; }}
.event-label {{ margin-top: 7px; color: var(--muted); font-size: 12px; }}
.transaction-ledger {{ margin-top: 8px; overflow-x: auto; }}
.transaction-ledger table {{ width: 100%; border-collapse: collapse; font-size: 12px; font-variant-numeric: tabular-nums; }}
.transaction-ledger th, .transaction-ledger td {{ padding: 5px 7px; border-bottom: 1px solid var(--line); text-align: left; white-space: nowrap; }}
.transaction-ledger th {{ color: var(--muted); font-weight: 500; }}
.runtime-budget {{ padding: 10px 12px; overflow-x: auto; border: 1px solid var(--line); border-radius: 8px; background: var(--panel); }}
.runtime-budget-table {{ width: 100%; min-width: 640px; border-collapse: collapse; font-size: 12px; font-variant-numeric: tabular-nums; }}
.runtime-budget-table th, .runtime-budget-table td {{ padding: 5px 7px; border-bottom: 1px solid var(--line); text-align: right; white-space: nowrap; }}
.runtime-budget-table th {{ color: var(--muted); font-weight: 500; }}
.runtime-budget-table th:first-child, .runtime-budget-table td:first-child {{ text-align: left; }}
.runtime-budget-ok {{ color: var(--green); }}
.runtime-budget-overrun {{ color: var(--red); font-weight: 600; }}
.runtime-budget-counters {{ display: flex; flex-wrap: wrap; gap: 7px 18px; margin-top: 9px; color: var(--muted); font-size: 12px; font-variant-numeric: tabular-nums; }}
@media (max-width: 760px) {{ body {{ padding: 14px; }} .metrics {{ grid-template-columns: repeat(2, minmax(0, 1fr)); }} .grid {{ grid-template-columns: 1fr; }} .panel.full {{ grid-column: auto; }} }}
@media (max-width: 390px) {{ .metrics {{ grid-template-columns: 1fr 1fr; gap: 7px; }} .metric-value {{ font-size: 17px; }} }}
</style>
</head>
<body>
<main id="telemetry-dashboard">
  <h1>TC264 遥测分析</h1>
  <div class="topline"><span id="run-id"></span><span id="route-id"></span><span id="firmware-id"></span></div>
  <section class="metrics" aria-label="运行摘要">
    <div class="metric"><div class="metric-label">运行时长</div><div class="metric-value" id="metric-duration">—</div></div>
    <div class="metric"><div class="metric-label">横向误差 P95</div><div class="metric-value" id="metric-cross-track">—</div></div>
    <div class="metric"><div class="metric-label">速度误差 RMS</div><div class="metric-value" id="metric-speed-error">—</div></div>
    <div class="metric"><div class="metric-label">视觉接受率</div><div class="metric-value" id="metric-vision">—</div></div>
    <div class="metric"><div class="metric-label">限幅 / 保护</div><div class="metric-value" id="metric-events">—</div></div>
    <div class="metric"><div class="metric-label">IMU 记录</div><div class="metric-value" id="metric-imu">—</div></div>
    <div class="metric"><div class="metric-label">定位接受 / 拒绝</div><div class="metric-value" id="metric-localization">—</div></div>
  </section>
  <section class="grid">
    <div class="panel full"><h2>实时预算与跨核健康</h2><div class="runtime-budget" id="runtime-budget"></div></div>
    <div class="panel full"><h2>A. 地图系主视图</h2><label class="event-label" for="transform-selector">显示变换 <select id="transform-selector"></select></label><div id="map-chart"></div><div class="legend"><span style="color:var(--purple)"><i class="swatch"></i>physical feature map</span><span style="color:var(--blue)"><i class="swatch"></i>nav_path</span><span style="color:var(--green)"><i class="swatch"></i>online corrected trajectory</span><span style="color:var(--orange)"><i class="swatch"></i>capture-time / selected-T point cloud</span><span style="color:var(--red)"><i class="swatch"></i>accepted before/after · rejected candidate dashed</span></div><div class="event-label" id="map-detail"></div><div class="transaction-ledger" id="localization-ledger"></div><div class="transaction-ledger" id="localization-feature-ledger"></div></div>
    <div class="panel full"><h2>B. 原始 odom 诊断视图</h2><div id="odom-map-chart"></div><div class="legend"><span style="color:var(--green)"><i class="swatch"></i>raw trajectory</span><span style="color:var(--orange)"><i class="swatch"></i>raw point cloud</span><span style="color:var(--purple)"><i class="swatch"></i>physical feature map · inverse(T)</span><span style="color:var(--blue)"><i class="swatch"></i>nav_path · inverse(T)</span></div><div class="event-label" id="odom-map-detail"></div></div>
    <div class="panel full"><h2>原始点云局部拟合诊断</h2><div id="phototube-scan-chart"></div><div class="legend"><span style="color:var(--green)"><i class="swatch"></i>raw point cloud</span><span style="color:var(--orange)"><i class="swatch"></i>TLS/PCA local fit</span><span style="color:var(--blue)"><i class="swatch"></i>odom y=0</span></div><div class="event-label" id="phototube-scan-detail"></div></div>
    <div class="panel full"><h2>速度跟踪与左右轮参考</h2><div id="speed-chart"></div><div class="legend"><span style="color:var(--blue)"><i class="swatch"></i>规划速度</span><span style="color:var(--green)"><i class="swatch"></i>左实际</span><span style="color:var(--orange)"><i class="swatch"></i>右实际</span><span style="color:var(--purple)"><i class="swatch"></i>左目标</span><span style="color:var(--red)"><i class="swatch"></i>右目标</span></div></div>
    <div class="panel full"><h2>速度环误差</h2><div id="speed-error-chart"></div><div class="legend"><span style="color:var(--blue)"><i class="swatch"></i>车体平均速度误差</span><span style="color:var(--green)"><i class="swatch"></i>左轮相对参考误差</span><span style="color:var(--orange)"><i class="swatch"></i>右轮相对参考误差</span></div><div class="event-label" id="speed-tuning-detail"></div></div>
    <div class="panel full"><h2>速度环输出与左右轮平衡</h2><div id="pwm-chart"></div><div class="legend"><span style="color:var(--green)"><i class="swatch"></i>左 PWM</span><span style="color:var(--orange)"><i class="swatch"></i>右 PWM</span><span style="color:var(--purple)"><i class="swatch"></i>实际左−右速度</span><span style="color:var(--blue)"><i class="swatch"></i>目标左−右速度</span></div></div>
    <div class="panel full"><h2>IMU 三轴加速度</h2><div id="imu-acc-chart"></div><div class="legend"><span style="color:var(--blue)"><i class="swatch"></i>Acc X</span><span style="color:var(--green)"><i class="swatch"></i>Acc Y</span><span style="color:var(--orange)"><i class="swatch"></i>Acc Z</span><span style="color:var(--purple)"><i class="swatch"></i>|Acc|</span></div></div>
    <div class="panel full"><h2>IMU 三轴角速度</h2><div id="imu-gyro-chart"></div><div class="legend"><span style="color:var(--blue)"><i class="swatch"></i>Gyro X</span><span style="color:var(--green)"><i class="swatch"></i>Gyro Y</span><span style="color:var(--orange)"><i class="swatch"></i>Gyro Z</span><span style="color:var(--purple)"><i class="swatch"></i>|Gyro|</span></div></div>
    <div class="panel full"><h2>四元数</h2><div id="imu-attitude-chart"></div><div class="legend"><span style="color:var(--blue)"><i class="swatch"></i>q0</span><span style="color:var(--green)"><i class="swatch"></i>q1</span><span style="color:var(--orange)"><i class="swatch"></i>q2</span><span style="color:var(--purple)"><i class="swatch"></i>q3</span></div></div>
    <div class="panel full"><h2>姿态角</h2><div id="imu-euler-chart"></div><div class="legend"><span style="color:var(--blue)"><i class="swatch"></i>Roll</span><span style="color:var(--green)"><i class="swatch"></i>Pitch</span><span style="color:var(--orange)"><i class="swatch"></i>Yaw</span></div><div class="event-label" id="imu-detail"></div></div>
    <div class="panel"><h2>横向误差与航向误差</h2><div id="navigation-chart"></div><div class="legend"><span style="color:var(--blue)"><i class="swatch"></i>横向误差 mm</span><span style="color:var(--orange)"><i class="swatch"></i>实际−路径切线 °</span></div></div>
    <div class="panel"><h2>离线曲率前馈</h2><div id="lookahead-chart"></div><div class="legend"><span style="color:var(--blue)"><i class="swatch"></i>曲率前馈偏航率 °/s</span><span style="color:var(--green)"><i class="swatch"></i>前馈取样进度点</span><span style="color:var(--orange)"><i class="swatch"></i>曲率半径 mm</span></div></div>
    <div class="panel"><h2>离线速度与加速度参考</h2><div id="route-dynamics-chart"></div><div class="legend"><span style="color:var(--blue)"><i class="swatch"></i>vref cm/s</span><span style="color:var(--orange)"><i class="swatch"></i>aref cm/s²</span></div></div>
    <div class="panel"><h2>离线曲率参考</h2><div id="route-curvature-chart"></div><div class="legend"><span style="color:var(--green)"><i class="swatch"></i>κ 1/m</span><span style="color:var(--purple)"><i class="swatch"></i>κ′ 1/m²</span></div></div>
    <div class="panel"><h2>转向分量</h2><div id="turn-chart"></div><div class="legend">{turn_legend_html}</div></div>
    <div class="panel"><h2>视觉质量与接受状态</h2><div id="vision-chart"></div><div class="legend"><span style="color:var(--blue)"><i class="swatch"></i>阵列位置估计 mm</span><span style="color:var(--green)"><i class="swatch"></i>有效传感器数</span></div><div class="event-label" id="vision-reasons"></div></div>
    <div class="panel"><h2>控制事件时间线</h2><div id="event-chart"></div><div class="event-label" id="event-summary"></div></div>
  </section>
</main>
<script>
const DATA = {data_json};
const root = document.getElementById("telemetry-dashboard");
const css = name => getComputedStyle(root).getPropertyValue(name).trim();
const fmt = (value, digits=1) => value == null || !Number.isFinite(Number(value)) ? "—" : Number(value).toFixed(digits);
const esc = value => String(value).replace(/[&<>"']/g, ch => ({{"&":"&amp;","<":"&lt;",">":"&gt;","\\\"":"&quot;","'":"&#39;"}}[ch]));
const metadata = DATA.metadata || {{}};
const yawRateController = Number(metadata.tracking_config?.algorithm_version || 0) >= 16;
const control = DATA.control || [];
const navigation = DATA.navigation || [];
const vision = DATA.vision || [];
const imu = DATA.imu || [];
const phototubeScan = DATA.phototube_scan || {{}};
const plannedRoute = DATA.planned_route || [];
const physicalFeatureMap = DATA.physical_feature_map || [];
const localization = DATA.localization || [];
const localizationFeatures = DATA.localization_features || [];
const events = DATA.events || [];
const runtimeBudget = DATA.runtime_budget || {{}};
const speedTuning = DATA.speed_tuning_summary || {{}};
const controlTuning = control.map(row => ({{x: row.tick_ms, speedError: row.speed_error_actual_minus_plan_cmps, leftError: row.left_error_actual_minus_target_cmps, rightError: row.right_error_actual_minus_target_cmps, wheelBalance: row.wheel_balance_actual_left_minus_right_cmps}}));
const points = values => values.filter(point => Number.isFinite(Number(point.x)) && Number.isFinite(Number(point.y)));
document.getElementById("run-id").textContent = `第 ${{metadata.session_sequence ?? "—"}} 次运行 · ${{fmt((metadata.duration_ms || 0) / 1000, 2)}} s`;
document.getElementById("route-id").textContent = `路线 ${{metadata.route_index ?? "—"}} · ${{metadata.route_hash || "—"}}`;
document.getElementById("firmware-id").textContent = `固件 ${{metadata.firmware_config_fingerprint || "—"}}`;

function extent(values) {{
  const finite = values.filter(value => Number.isFinite(Number(value))).map(Number);
  if (!finite.length) return [0, 1];
  let lo = Math.min(...finite), hi = Math.max(...finite);
  if (lo === hi) {{ const pad = Math.abs(lo) * .1 || 1; lo -= pad; hi += pad; }}
  const pad = (hi - lo) * .08;
  return [lo - pad, hi + pad];
}}
function drawChart(targetId, series, xLabel, yLabel, xValues, eventValues=[]) {{
  const target = document.getElementById(targetId);
  const width = Math.max(320, target.clientWidth || 720), height = 250;
  const m = {{top: 16, right: 16, bottom: 42, left: 58}}, innerW = width - m.left - m.right, innerH = height - m.top - m.bottom;
  const xDomain = extent(xValues), yDomain = extent(series.flatMap(item => item.values.map(point => point.y)));
  const sx = x => m.left + (Number(x) - xDomain[0]) / (xDomain[1] - xDomain[0]) * innerW;
  const sy = y => m.top + (1 - (Number(y) - yDomain[0]) / (yDomain[1] - yDomain[0])) * innerH;
  const tick = (value, digits=0) => Number(value).toFixed(digits);
  const xTicks = Array.from({{length: 5}}, (_, i) => xDomain[0] + (xDomain[1]-xDomain[0])*i/4);
  const yTicks = Array.from({{length: 5}}, (_, i) => yDomain[0] + (yDomain[1]-yDomain[0])*i/4);
  const linePath = values => values.length ? values.map((point, i) => `${{i ? "L" : "M"}} ${{sx(point.x).toFixed(2)}} ${{sy(point.y).toFixed(2)}}`).join(" ") : "";
  const grid = yTicks.map(value => `<line class="gridline" x1="${{m.left}}" x2="${{width-m.right}}" y1="${{sy(value)}}" y2="${{sy(value)}}"></line>`).join("");
  const xAxis = xTicks.map(value => `<text x="${{sx(value)}}" y="${{height-17}}" text-anchor="middle">${{tick(value, 0)}}</text>`).join("");
  const yAxis = yTicks.map(value => `<text x="${{m.left-8}}" y="${{sy(value)+4}}" text-anchor="end">${{tick(value, 1)}}</text>`).join("");
  const eventLines = eventValues.map(event => `<line class="event-line" x1="${{sx(event.x)}}" x2="${{sx(event.x)}}" y1="${{m.top}}" y2="${{height-m.bottom}}" data-tooltip="事件 ${{esc(event.label)}}"></line>`).join("");
  const paths = series.map(item => `<path class="line" style="stroke:${{item.color}}" d="${{linePath(points(item.values))}}"></path>`).join("");
  target.innerHTML = `<svg viewBox="0 0 ${{width}} ${{height}}" role="img" aria-label="${{esc(yLabel)}} 随 ${{esc(xLabel)}} 变化"><title>${{esc(yLabel)}} / ${{esc(xLabel)}}</title><g class="axis">${{grid}}${{xAxis}}${{yAxis}}<text x="${{m.left+innerW/2}}" y="${{height-2}}" text-anchor="middle">${{esc(xLabel)}}</text><text x="14" y="${{m.top+innerH/2}}" text-anchor="middle" transform="rotate(-90 14 ${{m.top+innerH/2}})">${{esc(yLabel)}}</text></g>${{eventLines}}${{paths}}</svg>`;
}}

function drawRuntimeBudget() {{
  const target = document.getElementById("runtime-budget");
  if (!target) return;
  const components = [
    ["control_exec", "1 ms 控制 ISR 执行", 1000],
    ["control_period", "控制 ISR 周期", 1000],
    ["phototube_scan", "CPU1 光电完整扫描", 1000],
    ["duty_10ms", "CPU0 10 ms 任务", 10000],
    ["nav_match", "点云地图匹配", 5000],
    ["emem_service", "EMEM 后台服务", 500],
  ];
  if (!Object.keys(runtimeBudget).length) {{
    target.textContent = "本次日志没有运行预算账本，无法判断实时余量。";
    return;
  }}
  const rows = components.map(([key, label, budgetUs]) => {{
    const entry = runtimeBudget[key];
    if (!entry) return `<tr><td>${{label}}</td><td>无记录</td><td>${{budgetUs}} us</td><td>—</td><td>—</td></tr>`;
    const maxUs = Number(entry.max_us);
    const overrunCount = Number(entry.overrun_count);
    const utilization = Number.isFinite(maxUs) ? maxUs / budgetUs * 100 : null;
    const hasOverrun =
      (Number.isFinite(overrunCount) && overrunCount > 0) ||
      (Number.isFinite(maxUs) && maxUs > budgetUs);
    const statusClass = hasOverrun ? "runtime-budget-overrun" : "runtime-budget-ok";
    const status = hasOverrun ? "超限" : "正常";
    return `<tr><td>${{label}}</td><td>${{Number.isFinite(maxUs) ? fmt(maxUs, 0) + " us" : "—"}}</td><td>${{budgetUs}} us</td><td>${{utilization == null ? "—" : fmt(utilization, 1) + "%"}}</td><td class="${{statusClass}}">${{Number.isFinite(overrunCount) ? fmt(overrunCount, 0) : "—"}} · ${{status}}</td></tr>`;
  }}).join("");
  const imuFailure = runtimeBudget.imu_snapshot_failure;
  const imuAge = runtimeBudget.imu_snapshot_age;
  const scanEpoch = runtimeBudget.scan_epoch;
  const counters = [];
  if (imuFailure) counters.push(`IMU 快照读取失败 ${{fmt(imuFailure.read_failure_count, 0)}} 次 · 最大连续 ${{fmt(imuFailure.max_consecutive_failure_count, 0)}} 次`);
  if (imuAge) counters.push(`IMU 快照年龄 最大 / 末次 ${{fmt(imuAge.max_age_us, 0)}} / ${{fmt(imuAge.last_age_us, 0)}} us`);
  if (scanEpoch) counters.push(`扫描 epoch 过滤 ${{fmt(scanEpoch.filtered_frame_count, 0)}} 帧 · 关闭确认超时 ${{fmt(scanEpoch.ack_timeout_count, 0)}} 次`);
  const counterHtml = counters.length
    ? `<div class="runtime-budget-counters">${{counters.map(value => `<span>${{value}}</span>`).join("")}}</div>`
    : `<div class="runtime-budget-counters"><span>跨核健康计数：无记录</span></div>`;
  target.innerHTML = `<table class="runtime-budget-table"><thead><tr><th>组件</th><th>最大耗时/周期</th><th>预算</th><th>预算占用</th><th>超限次数 / 状态</th></tr></thead><tbody>${{rows}}</tbody></table>${{counterHtml}}`;
}}

function transformPoint(point, transform) {{
  const c = Math.cos(transform.yaw), s = Math.sin(transform.yaw);
  return {{x: transform.tx + c * point.x - s * point.y, y: transform.ty + s * point.x + c * point.y}};
}}
function inversePoint(point, transform) {{
  const c = Math.cos(transform.yaw), s = Math.sin(transform.yaw);
  const dx = point.x - transform.tx, dy = point.y - transform.ty;
  return {{x: c * dx + s * dy, y: -s * dx + c * dy}};
}}
function selectedTransform() {{
  const selector = document.getElementById("transform-selector");
  const value = selector ? selector.value : "final";
  if (value.startsWith("tx:")) {{
    const tx = localization[Number(value.slice(3))];
    if (tx) return {{tx:tx.t_after_tx_mm, ty:tx.t_after_ty_mm, yaw:tx.t_after_yaw_deg*Math.PI/180, label:`事务 ${{tx.localization_update_seq}} @ ${{tx.timestamp_ms}} ms`}};
  }}
  const accepted = localization.filter(tx => tx.status_name === "accepted");
  const lastNavigation = navigation[navigation.length - 1];
  const finalSource = String(phototubeScan.localization_final_transform_source || "");
  if (lastNavigation && finalSource.includes("navigation_snapshot")) return {{
    tx:Number(lastNavigation.map_from_odom_tx_mm)||0,
    ty:Number(lastNavigation.map_from_odom_ty_mm)||0,
    yaw:(Number(lastNavigation.map_from_odom_yaw_deg)||0)*Math.PI/180,
    label:"最终 T_map_from_odom（导航快照补洞）"
  }};
  const lastTx = accepted[accepted.length - 1];
  if (lastTx) return {{tx:Number(lastTx.t_after_tx_mm)||0, ty:Number(lastTx.t_after_ty_mm)||0, yaw:(Number(lastTx.t_after_yaw_deg)||0)*Math.PI/180, label:"最终定位事务 T_map_from_odom"}};
  const last = navigation[navigation.length - 1] || {{}};
  return {{tx:Number(last.map_from_odom_tx_mm)||0, ty:Number(last.map_from_odom_ty_mm)||0, yaw:(Number(last.map_from_odom_yaw_deg)||0)*Math.PI/180, label:"导航日志最终 T_map_from_odom"}};
}}
function drawLayeredMap(targetId, frame) {{
  const target = document.getElementById(targetId);
  if (!target) return;
  const transform = selectedTransform();
  const scanFrames=[];
  let previousScanFrame=null;
  for(const row of (phototubeScan.points || [])) {{
    if(row.frame_seq===previousScanFrame) continue;
    previousScanFrame=row.frame_seq;
    scanFrames.push(row);
  }}
  const rawTrajectory = navigation.map(row => ({{t:row.timestamp_ms,x:row.raw_x_mm,y:row.raw_y_mm,yaw:row.raw_yaw_deg}})).concat(scanFrames.map(row => ({{t:row.capture_ms,x:row.raw_axle_x_mm,y:row.raw_axle_y_mm,yaw:row.raw_yaw_deg}}))).sort((a,b)=>a.t-b.t);
  const onlineTrajectory = navigation.map(row => ({{t:row.timestamp_ms,x:row.actual_x_mm,y:row.actual_y_mm,yaw:row.actual_yaw_deg}})).concat(scanFrames.filter(row => Number.isFinite(Number(row.map_axle_x_mm)) && Number.isFinite(Number(row.map_axle_y_mm))).map(row => ({{t:row.capture_ms,x:row.map_axle_x_mm,y:row.map_axle_y_mm,yaw:row.map_yaw_deg ?? row.raw_yaw_deg}}))).sort((a,b)=>a.t-b.t);
  const rawCloud = (phototubeScan.points || []).map(row => ({{x:row.x_mm, y:row.y_mm}}));
  const captureMapCloud = (phototubeScan.points || [])
    .filter(row => row.map_x_mm != null && row.map_y_mm != null)
    .map(row => ({{x:Number(row.map_x_mm), y:Number(row.map_y_mm)}}));
  const navMap = (plannedRoute.length ? plannedRoute : navigation).map(row => ({{x:plannedRoute.length ? row.x_mm : row.ideal_x_mm, y:plannedRoute.length ? row.y_mm : row.ideal_y_mm}}));
  const featureSegments = physicalFeatureMap.map(segment => ({{
    a:{{x:segment.x1_mm,y:segment.y1_mm}}, b:{{x:segment.x2_mm,y:segment.y2_mm}}, kind:segment.kind
  }}));
  const isMap = frame === "map";
  const trajectory = isMap ? onlineTrajectory : rawTrajectory;
  const useCaptureMapCloud = isMap &&
    document.getElementById("transform-selector")?.value === "capture";
  const cloud = isMap
    ? (useCaptureMapCloud ? captureMapCloud : rawCloud.map(point => transformPoint(point, transform)))
    : rawCloud;
  const navLayer = isMap ? navMap : navMap.map(point => inversePoint(point, transform));
  const features = isMap ? featureSegments : featureSegments.map(segment => ({{...segment,a:inversePoint(segment.a,transform),b:inversePoint(segment.b,transform)}}));
  const finitePoints = values => values.filter(point => Number.isFinite(Number(point.x)) && Number.isFinite(Number(point.y)));
  const all = finitePoints(trajectory.concat(cloud, navLayer, features.flatMap(segment => [segment.a, segment.b])));
  if (!all.length) {{ target.textContent = "无可显示数据"; return; }}
  const width = Math.max(320, target.clientWidth || 720), height = 420;
  const m = {{top:16,right:20,bottom:48,left:64}};
  const xDomain = extent(all.map(point => point.x)), yDomain = extent(all.map(point => point.y));
  const scale = Math.min((width-m.left-m.right)/(xDomain[1]-xDomain[0]),(height-m.top-m.bottom)/(yDomain[1]-yDomain[0]));
  const xCenter=(xDomain[0]+xDomain[1])/2, yCenter=(yDomain[0]+yDomain[1])/2;
  const sx=value=>width/2+(value-xCenter)*scale, sy=value=>height/2+(value-yCenter)*scale;
  const path=values=>values.length?values.map((point,index)=>`${{index?"L":"M"}} ${{sx(point.x).toFixed(2)}} ${{sy(point.y).toFixed(2)}}`).join(" "):"";
  const xTicks=Array.from({{length:5}},(_,i)=>xDomain[0]+(xDomain[1]-xDomain[0])*i/4), yTicks=Array.from({{length:5}},(_,i)=>yDomain[0]+(yDomain[1]-yDomain[0])*i/4);
  const grid=xTicks.map(value=>`<line class="gridline" x1="${{sx(value)}}" x2="${{sx(value)}}" y1="${{m.top}}" y2="${{height-m.bottom}}"></line>`).join("")+yTicks.map(value=>`<line class="gridline" x1="${{m.left}}" x2="${{width-m.right}}" y1="${{sy(value)}}" y2="${{sy(value)}}"></line>`).join("");
  const labels=xTicks.map(value=>`<text x="${{sx(value)}}" y="${{height-18}}" text-anchor="middle">${{fmt(value,0)}}</text>`).join("")+yTicks.map(value=>`<text x="${{m.left-8}}" y="${{sy(value)+4}}" text-anchor="end">${{fmt(value,0)}}</text>`).join("");
  const featureLines=features.map(segment=>`<line x1="${{sx(segment.a.x)}}" y1="${{sy(segment.a.y)}}" x2="${{sx(segment.b.x)}}" y2="${{sy(segment.b.y)}}" stroke="var(--purple)" stroke-width="${{segment.kind==="support_branch"?2.6:1.8}}"></line>`).join("");
  const cloudDots=cloud.map(point=>`<circle cx="${{sx(point.x)}}" cy="${{sy(point.y)}}" r="1.8" fill="var(--orange)" opacity=".72"></circle>`).join("");
  let transactionMarks="";
  if (isMap) transactionMarks=localization.map(tx=>{{
    const accepted=tx.status_name==="accepted";
    const before={{x:tx.map_before_x_mm,y:tx.map_before_y_mm}};
    const candidate={{x:tx.candidate_map_after_x_mm,y:tx.candidate_map_after_y_mm}};
    const target=accepted?{{x:tx.map_after_x_mm,y:tx.map_after_y_mm}}:candidate;
    const progressTarget=accepted?tx.progress_after_points:tx.candidate_progress_after_points;
    const title=`${{accepted?"接受并提交":"拒绝，候选未提交"}} · feature ${{tx.route_feature_occurrence}} · ${{tx.match_type_name}} · Δ(${{fmt(tx.delta_odom_dx_mm,1)}}, ${{fmt(tx.delta_odom_dy_mm,1)}}, ${{fmt(tx.delta_odom_dyaw_deg,2)}}°) · RMS ${{fmt(tx.rms_before_mm,1)}}→${{fmt(tx.rms_after_mm,1)}} mm · cross-track ${{fmt(tx.map_before_cross_track_mm,1)}}→${{fmt(tx.candidate_cross_track_mm,1)}} mm · heading error ${{fmt(tx.map_before_heading_error_deg,1)}}→${{fmt(tx.candidate_heading_error_deg,1)}}° · inliers ${{tx.inlier_count}} · candidate progress ${{fmt(tx.progress_before_points,1)}}→${{fmt(progressTarget,1)}}`;
    return `<g><title>${{esc(title)}}</title><line x1="${{sx(before.x)}}" y1="${{sy(before.y)}}" x2="${{sx(target.x)}}" y2="${{sy(target.y)}}" stroke="var(--red)" stroke-dasharray="${{accepted?"":"4 3"}}"></line><circle cx="${{sx(before.x)}}" cy="${{sy(before.y)}}" r="4" fill="none" stroke="var(--red)"></circle><circle cx="${{sx(target.x)}}" cy="${{sy(target.y)}}" r="4" fill="${{accepted?"var(--red)":"none"}}" stroke="var(--red)"></circle><text x="${{sx(target.x)+6}}" y="${{sy(target.y)-6}}" fill="var(--red)">F${{tx.route_feature_occurrence}} ${{accepted?"✓":"×"}}</text></g>`;
  }}).join("");
  const lastRaw=rawTrajectory[rawTrajectory.length-1];
  const current=lastRaw?(isMap?transformPoint(lastRaw,transform):lastRaw):null;
  const rawYaw=(Number(lastRaw?.yaw)||0)*Math.PI/180;
  const bodyYaw=rawYaw+(isMap?transform.yaw:0);
  const body=current?`<g><circle cx="${{sx(current.x)}}" cy="${{sy(current.y)}}" r="5" fill="var(--red)"></circle><line x1="${{sx(current.x)}}" y1="${{sy(current.y)}}" x2="${{sx(current.x+80*Math.cos(bodyYaw))}}" y2="${{sy(current.y+80*Math.sin(bodyYaw))}}" stroke="var(--red)" stroke-width="2"></line></g>`:"";
  target.innerHTML=`<svg viewBox="0 0 ${{width}} ${{height}}" role="img"><title>${{isMap?"map frame":"raw odom frame"}}</title>${{grid}}${{featureLines}}<path class="line" style="stroke:var(--blue)" d="${{path(navLayer)}}"></path><path class="line" style="stroke:var(--green)" d="${{path(trajectory)}}"></path>${{cloudDots}}${{transactionMarks}}${{body}}${{labels}}<text x="${{width/2}}" y="${{height-2}}" text-anchor="middle">${{isMap?"map":"raw odom"}} x mm</text><text x="14" y="${{height/2}}" text-anchor="middle" transform="rotate(-90 14 ${{height/2}})">${{isMap?"map":"raw odom"}} y（右侧+，向下）mm</text></svg>`;
  const detail=document.getElementById(isMap?"map-detail":"odom-map-detail");
  if(detail) detail.textContent=`${{transform.label}} · raw点 ${{rawCloud.length}} · 物理线段 ${{features.length}} · 定位事务 ${{localization.length}}`;
}}

function drawMap() {{ drawLayeredMap("map-chart","map"); drawLayeredMap("odom-map-chart","odom"); }}

function drawLocalizationLedger() {{
  const target=document.getElementById("localization-ledger");
  if(!target) return;
  if(!localization.length) {{ target.textContent="无定位事务"; return; }}
  const rows=localization.map(tx=>`<tr><td>${{tx.timestamp_ms}}</td><td>${{tx.localization_update_seq}}</td><td>${{tx.status_name==="accepted"?"接受":"拒绝 · "+esc(tx.reject_reason_name)}}</td><td>F${{tx.route_feature_occurrence}} · ${{esc(tx.match_type_name)}}</td><td>${{fmt(tx.delta_odom_dx_mm,1)}} / ${{fmt(tx.delta_odom_dy_mm,1)}} / ${{fmt(tx.delta_odom_dyaw_deg,2)}}°</td><td>${{fmt(tx.candidate_map_pose_jump_mm,1)}} / ${{fmt(tx.map_pose_jump_mm,1)}}</td><td>${{fmt(tx.map_before_cross_track_mm,1)}} → 候选 ${{fmt(tx.candidate_cross_track_mm,1)}} / 实际 ${{fmt(tx.map_after_cross_track_mm,1)}}</td><td>${{fmt(tx.map_before_heading_error_deg,1)}}° → 候选 ${{fmt(tx.candidate_heading_error_deg,1)}}° / 实际 ${{fmt(tx.map_after_heading_error_deg,1)}}°</td><td>${{fmt(tx.map_before_pp_p_turn_cmps,1)}} → 候选 ${{fmt(tx.candidate_pp_p_turn_cmps,1)}} / 实际 ${{fmt(tx.map_after_pp_p_turn_cmps,1)}}</td><td>${{fmt(tx.rms_before_mm,1)}} → ${{fmt(tx.rms_after_mm,1)}}</td><td>${{tx.inlier_count}}</td><td>${{fmt(tx.progress_before_points,1)}} → 候选 ${{fmt(tx.candidate_progress_after_points,1)}} / 实际 ${{fmt(tx.progress_after_points,1)}}</td></tr>`).join("");
  target.innerHTML=`<table><thead><tr><th>ms</th><th>seq</th><th>结果</th><th>特征/类型</th><th>候选 delta dx/dy/dyaw</th><th>候选 / 实际位姿跳变 mm</th><th>横向 before → candidate / actual mm</th><th>航向误差 before → candidate / actual</th><th>PP P项 before → candidate / actual cm/s</th><th>RMS mm</th><th>内点</th><th>progress before → candidate / actual</th></tr></thead><tbody>${{rows}}</tbody></table>`;
}}

function drawLocalizationFeatureLedger() {{
  const target=document.getElementById("localization-feature-ledger");
  if(!target) return;
  const relevant=localizationFeatures.filter(row=>row.outcome!=="not_reached");
  if(!relevant.length) {{ target.textContent="没有已到达的定位特征"; return; }}
  const outcome=row=>row.outcome==="accepted"?"完整校准":row.outcome==="partial"?"部分校准":row.outcome==="failed"?"校准失败":"经过但未进入候选门";
  const rows=relevant.map(row=>`<tr><td>F${{row.route_feature_occurrence}}</td><td>${{esc(row.feature_type_name)}}</td><td>${{row.feature_progress_points}}</td><td>${{outcome(row)}}</td><td>${{row.attempt_count}}</td><td>${{esc(row.last_reject_reason_name)}}</td><td>${{esc(row.last_detail_names||"—")}}</td><td>${{esc(row.detail_names||"—")}}</td><td>${{row.line0_point_count}}/${{row.line1_point_count}}/${{row.line2_point_count}}</td><td>${{row.line0_frame_count}}/${{row.line1_frame_count}}/${{row.line2_frame_count}}</td><td>${{row.failure_value_name==="none"?"—":fmt(row.failure_value,1)+" "+esc(row.failure_value_unit)}}</td></tr>`).join("");
  target.innerHTML=`<table><thead><tr><th>特征</th><th>类型</th><th>progress</th><th>结果</th><th>尝试</th><th>最后原因</th><th>最后失败门限</th><th>全部失败门限</th><th>三线点数</th><th>三线帧数</th><th>失败值</th></tr></thead><tbody>${{rows}}</tbody></table>`;
}}

function drawPhototubeScan() {{
  const target = document.getElementById("phototube-scan-chart");
  if (!target) return;
  const points = phototubeScan.points || [];
  const width = Math.max(320, target.clientWidth || 720), height = 300;
  const m = {{top: 16, right: 20, bottom: 44, left: 62}};
  const xValues = points.map(row => row.x_mm);
  const yValues = points.map(row => row.y_mm).concat([0]);
  const xDomain = extent(xValues), yDomain = extent(yValues);
  const sx = value => m.left + (Number(value) - xDomain[0]) / (xDomain[1] - xDomain[0]) * (width - m.left - m.right);
  const sy = value => m.top + (Number(value) - yDomain[0]) / (yDomain[1] - yDomain[0]) * (height - m.top - m.bottom);
  const xTicks = Array.from({{length: 5}}, (_, i) => xDomain[0] + (xDomain[1] - xDomain[0]) * i / 4);
  const yTicks = Array.from({{length: 5}}, (_, i) => yDomain[0] + (yDomain[1] - yDomain[0]) * i / 4);
  const grid = xTicks.map(value => `<line class="gridline" x1="${{sx(value)}}" x2="${{sx(value)}}" y1="${{m.top}}" y2="${{height-m.bottom}}"></line>`).join("") + yTicks.map(value => `<line class="gridline" x1="${{m.left}}" x2="${{width-m.right}}" y1="${{sy(value)}}" y2="${{sy(value)}}"></line>`).join("");
  const labels = xTicks.map(value => `<text x="${{sx(value)}}" y="${{height-18}}" text-anchor="middle">${{fmt(value, 0)}}</text>`).join("") + yTicks.map(value => `<text x="${{m.left-8}}" y="${{sy(value)+4}}" text-anchor="end">${{fmt(value, 0)}}</text>`).join("");
  const dots = points.map(row => `<circle cx="${{sx(row.x_mm)}}" cy="${{sy(row.y_mm)}}" r="2.4" fill="var(--green)"></circle>`).join("");
  const zero = `<line x1="${{m.left}}" x2="${{width-m.right}}" y1="${{sy(0)}}" y2="${{sy(0)}}" stroke="var(--blue)" stroke-width="1.4"></line>`;
  let fit = "";
  if (phototubeScan.valid && Number.isFinite(Number(phototubeScan.line_direction_x)) && Number.isFinite(Number(phototubeScan.line_direction_y))) {{
    const length = Math.hypot(xDomain[1]-xDomain[0], yDomain[1]-yDomain[0]);
    const x0 = phototubeScan.line_center_x_mm - phototubeScan.line_direction_x * length;
    const y0 = phototubeScan.line_center_y_mm - phototubeScan.line_direction_y * length;
    const x1 = phototubeScan.line_center_x_mm + phototubeScan.line_direction_x * length;
    const y1 = phototubeScan.line_center_y_mm + phototubeScan.line_direction_y * length;
    fit = `<line x1="${{sx(x0)}}" y1="${{sy(y0)}}" x2="${{sx(x1)}}" y2="${{sy(y1)}}" stroke="var(--orange)" stroke-width="2"></line>`;
  }}
  target.innerHTML = `<svg viewBox="0 0 ${{width}} ${{height}}" role="img" aria-label="光电管原始 odom 扫图"><title>光电管原始 odom 扫图</title>${{grid}}${{zero}}${{fit}}${{dots}}${{labels}}<text x="${{width/2}}" y="${{height-2}}" text-anchor="middle">raw odom x mm</text><text x="14" y="${{height/2}}" text-anchor="middle" transform="rotate(-90 14 ${{height/2}})">raw odom y（右侧+，向下）mm</text></svg>`;
  const detail = document.getElementById("phototube-scan-detail");
  if (detail) detail.textContent = `EMEM保留最新点 ${{points.length}} / 接受帧 ${{phototubeScan.accepted_frame_count || 0}} / 收到帧 ${{phototubeScan.received_frame_count || 0}} · RMS ${{fmt(phototubeScan.rms_error_mm, 2)}} mm · capture位姿插值 ${{phototubeScan.pose_interpolated_count || 0}} · 等待后一拍位姿 ${{phototubeScan.pose_wait_count || 0}} · 位姿真正过期 ${{phototubeScan.pose_rejected_count || 0}} · staging峰值 ${{phototubeScan.staging_high_watermark || 0}}/64 · staging回压次数 ${{phototubeScan.staging_backpressure_count || 0}} · 空间去重 ${{phototubeScan.spatial_dedup_frame_count || 0}} · 里程倒退 ${{phototubeScan.travel_reversal_frame_count || 0}} · 在线环覆盖 ${{phototubeScan.online_ring_overwrite_point_count || 0}}（不影响EMEM） · EMEM覆盖旧点 ${{phototubeScan.archive_overwrite_point_count || 0}} · EMEM后台队列丢失 ${{phototubeScan.emem_queue_drop_count || 0}} · 空命中 ${{phototubeScan.empty_hit_frame_count || 0}} · CPU1→CPU0邮箱丢失 ${{phototubeScan.mailbox_skipped_count || 0}} · 光电源帧号缺口 ${{phototubeScan.source_frame_skipped_count || 0}} · CPU1扫描调度延迟 ${{phototubeScan.source_schedule_drop_count || 0}}次/跳过${{phototubeScan.source_schedule_skipped_count || 0}}个1ms时隙 · 最大入云延迟 ${{phototubeScan.max_capture_to_cloud_latency_ms || 0}} ms · 定位账本缺口 ${{phototubeScan.localization_ledger_gap_count || 0}} · 同毫秒边界点 ${{phototubeScan.localization_same_ms_boundary_point_count || 0}} · 导航快照补洞点 ${{phototubeScan.localization_projection_fallback_point_count || 0}} · 未能重投影点 ${{phototubeScan.localization_projection_unresolved_point_count || 0}}`;
}}

const transformSelector=document.getElementById("transform-selector");
if(transformSelector) {{
  transformSelector.innerHTML='<option value="capture">采集时 T（推荐）</option><option value="final">最终有效 T</option>'+localization.map((tx,index)=>tx.status_name==="accepted"?`<option value="tx:${{index}}">seq ${{tx.localization_update_seq}} · ${{tx.timestamp_ms}} ms · F${{tx.route_feature_occurrence}}</option>`:"").join("");
  transformSelector.addEventListener("change",drawMap);
}}
drawMap();
drawLocalizationLedger();
drawLocalizationFeatureLedger();
drawPhototubeScan();
drawRuntimeBudget();
const speedX = control.map(row => row.tick_ms);
drawChart("speed-chart", [
  {{color: css("--blue"), values: control.map(row => ({{x:row.tick_ms, y:row.plan_v_cmps}}))}},
  {{color: css("--green"), values: control.map(row => ({{x:row.tick_ms, y:row.actual_l_cmps}}))}},
  {{color: css("--orange"), values: control.map(row => ({{x:row.tick_ms, y:row.actual_r_cmps}}))}},
  {{color: css("--purple"), values: control.map(row => ({{x:row.tick_ms, y:row.target_l_cmps}}))}},
  {{color: css("--red"), values: control.map(row => ({{x:row.tick_ms, y:row.target_r_cmps}}))}},
], "控制 tick ms", "速度 cm/s", speedX, events.filter(row => row.type === 8 || row.type === 9).map(row => ({{x:row.timestamp_ms, label:`${{row.type === 9 ? "保护" : "限幅"}} code=${{row.code}}`}})));

drawChart("speed-error-chart", [
  {{color: css("--blue"), values: controlTuning.map(row => ({{x:row.x, y:row.speedError}}))}},
  {{color: css("--green"), values: controlTuning.map(row => ({{x:row.x, y:row.leftError}}))}},
  {{color: css("--orange"), values: controlTuning.map(row => ({{x:row.x, y:row.rightError}}))}},
], "控制 tick ms", "速度误差 cm/s", speedX);

drawChart("pwm-chart", [
  {{color: css("--green"), values: control.map(row => ({{x:row.tick_ms, y:row.pwm_l}}))}},
  {{color: css("--orange"), values: control.map(row => ({{x:row.tick_ms, y:row.pwm_r}}))}},
  {{color: css("--purple"), values: controlTuning.map(row => ({{x:row.x, y:row.wheelBalance}}))}},
  {{color: css("--blue"), values: control.map(row => ({{x:row.tick_ms, y:row.wheel_balance_target_left_minus_right_cmps}}))}},
], "控制 tick ms", "PWM / 轮间差 cm/s", speedX);

const imuX = imu.map(row => row.timestamp_ms);
drawChart("imu-acc-chart", [
  {{color: css("--blue"), values: imu.map(row => ({{x:row.timestamp_ms, y:row.acc_x_g}}))}},
  {{color: css("--green"), values: imu.map(row => ({{x:row.timestamp_ms, y:row.acc_y_g}}))}},
  {{color: css("--orange"), values: imu.map(row => ({{x:row.timestamp_ms, y:row.acc_z_g}}))}},
  {{color: css("--purple"), values: imu.map(row => ({{x:row.timestamp_ms, y:row.acc_norm_g}}))}},
], "IMU 时间 ms", "加速度 g", imuX);

drawChart("imu-gyro-chart", [
  {{color: css("--blue"), values: imu.map(row => ({{x:row.timestamp_ms, y:row.gyro_x_dps}}))}},
  {{color: css("--green"), values: imu.map(row => ({{x:row.timestamp_ms, y:row.gyro_y_dps}}))}},
  {{color: css("--orange"), values: imu.map(row => ({{x:row.timestamp_ms, y:row.gyro_z_dps}}))}},
  {{color: css("--purple"), values: imu.map(row => ({{x:row.timestamp_ms, y:row.gyro_norm_dps}}))}},
], "IMU 时间 ms", "角速度 deg/s", imuX);

drawChart("imu-attitude-chart", [
  {{color: css("--blue"), values: imu.map(row => ({{x:row.timestamp_ms, y:row.quaternion_q0}}))}},
  {{color: css("--green"), values: imu.map(row => ({{x:row.timestamp_ms, y:row.quaternion_q1}}))}},
  {{color: css("--orange"), values: imu.map(row => ({{x:row.timestamp_ms, y:row.quaternion_q2}}))}},
  {{color: css("--purple"), values: imu.map(row => ({{x:row.timestamp_ms, y:row.quaternion_q3}}))}},
], "IMU 时间 ms", "四元数分量", imuX);

drawChart("imu-euler-chart", [
  {{color: css("--blue"), values: imu.map(row => ({{x:row.timestamp_ms, y:row.roll_deg}}))}},
  {{color: css("--green"), values: imu.map(row => ({{x:row.timestamp_ms, y:row.pitch_deg}}))}},
  {{color: css("--orange"), values: imu.map(row => ({{x:row.timestamp_ms, y:row.yaw_deg}}))}},
], "IMU 时间 ms", "姿态角 °", imuX);

const navX = navigation.map(row => row.progress_index);
drawChart("navigation-chart", [
  {{color: css("--blue"), values: navigation.map(row => ({{x:row.progress_index, y:row.cross_track_mm}}))}},
  {{color: css("--orange"), values: navigation.map(row => ({{x:row.progress_index, y:row.heading_error_deg}}))}},
], "路径 progress index", "横向 mm / 航向 °", navX);

const lookaheadX = navigation.map(row => row.timestamp_ms);
drawChart("lookahead-chart", [
  {{color: css("--blue"), values: navigation.map(row => ({{x:row.timestamp_ms, y:row.curvature_feedforward_yaw_rate_dps}}))}},
  {{color: css("--green"), values: navigation.map(row => ({{x:row.timestamp_ms, y:row.curvature_preview_progress_points}}))}},
  {{color: css("--orange"), values: navigation.map(row => ({{x:row.timestamp_ms, y:row.curvature_radius_mm}}))}},
], "运行时间 ms", "曲率前馈 °/s / 进度点 / 半径 mm", lookaheadX);

const routeX = plannedRoute.map(row => row.progress_index);
drawChart("route-dynamics-chart", [
  {{color: css("--blue"), values: plannedRoute.map(row => ({{x:row.progress_index, y:row.target_speed_cmps}}))}},
  {{color: css("--orange"), values: plannedRoute.map(row => ({{x:row.progress_index, y:row.reference_acceleration_cmps2}}))}},
], "路径 progress index", "vref / aref", routeX);
drawChart("route-curvature-chart", [
  {{color: css("--green"), values: plannedRoute.map(row => ({{x:row.progress_index, y:row.curvature_per_mm*1000}}))}},
  {{color: css("--purple"), values: plannedRoute.map(row => ({{x:row.progress_index, y:row.curvature_derivative_per_mm2*1000000}}))}},
], "路径 progress index", "κ 1/m / κ′ 1/m²", routeX);

const turnComponents = yawRateController ? [
  {{color: css("--purple"), values: control.map(row => ({{x:row.tick_ms, y:row.yaw_rate_feedforward_turn_cmps}}))}},
  {{color: css("--orange"), values: control.map(row => ({{x:row.tick_ms, y:row.yaw_rate_feedback_turn_cmps}}))}},
] : [
  {{color: css("--purple"), values: control.map(row => ({{x:row.tick_ms, y:row.pd_p_turn_inferred_cmps}}))}},
  {{color: css("--orange"), values: control.map(row => ({{x:row.tick_ms, y:row.pd_d_turn_cmps}}))}},
  {{color: css("--red"), values: control.map(row => ({{x:row.tick_ms, y:row.gkd_turn_cmps}}))}},
];
drawChart("turn-chart", [
  {{color: css("--blue"), values: control.map(row => ({{x:row.tick_ms, y:row.final_turn_cmps}}))}},
  {{color: css("--green"), values: control.map(row => ({{x:row.tick_ms, y:row.tracking_turn_cmps}}))}},
].concat(turnComponents), "控制 tick ms", "转向分量 cm/s", speedX);

const visionX = vision.map(row => row.tick_ms);
drawChart("vision-chart", [
  {{color: css("--blue"), values: vision.map(row => ({{x:row.tick_ms, y:row.line_position_mm}}))}},
  {{color: css("--green"), values: vision.map(row => ({{x:row.tick_ms, y:row.active_sensor_count}}))}},
  {{color: css("--orange"), values: vision.map(row => ({{x:row.tick_ms, y:row.peak_count}}))}},
], "视觉 tick ms", "视觉值", visionX);

const eventX = events.map(row => row.timestamp_ms);
drawChart("event-chart", [{{color: css("--red"), values: eventX.map(x => ({{x, y:1}}))}}], "运行时间 ms", "事件", eventX.map((x, i) => ({{x, label:`type=${{events[i].type}} code=${{events[i].code}}`}})));

const reasons = {{}};
vision.forEach(row => {{ reasons[row.reject_reason_name] = (reasons[row.reject_reason_name] || 0) + 1; }});
document.getElementById("vision-reasons").textContent = "视觉原因：" + Object.entries(reasons).map(([name, count]) => `${{name}} ${{count}}`).join(" · ") || "视觉原因：无记录";
const limitCount = events.filter(row => row.type === 8).length, protectionCount = events.filter(row => row.type === 9).length;
document.getElementById("event-summary").textContent = `事件 ${{events.length}} 条 · 控制限幅 ${{limitCount}} · 保护 ${{protectionCount}}`;
const navAbs = navigation.map(row => Math.abs(row.cross_track_mm)).sort((a,b) => a-b);
const p95 = navAbs.length ? navAbs[Math.min(navAbs.length - 1, Math.floor(navAbs.length * .95))] : null;
const speedErrors = control.map(row => ((row.actual_l_cmps + row.actual_r_cmps) / 2) - row.plan_v_cmps);
const speedRms = speedErrors.length ? Math.sqrt(speedErrors.reduce((sum, value) => sum + value * value, 0) / speedErrors.length) : null;
const accepted = vision.filter(row => row.line_valid === true).length;
document.getElementById("metric-duration").textContent = `${{fmt((metadata.duration_ms || 0) / 1000, 2)}} s`;
document.getElementById("metric-cross-track").textContent = p95 == null ? "—" : `${{fmt(p95, 1)}} mm`;
document.getElementById("metric-speed-error").textContent = speedRms == null ? "—" : `${{fmt(speedRms, 1)}} cm/s`;
document.getElementById("metric-vision").textContent = vision.length ? `${{fmt(accepted / vision.length * 100, 1)}}%` : "—";
document.getElementById("metric-events").textContent = `${{limitCount}} / ${{protectionCount}}`;
document.getElementById("metric-imu").textContent = imu.length ? `${{imu.length}} · ${{fmt((imu[imu.length - 1].timestamp_us || 0) / 1000, 1)}} ms` : "—";
const acceptedLocalization = localization.filter(row => row.status_name === "accepted").length;
document.getElementById("metric-localization").textContent = `${{acceptedLocalization}} / ${{localization.length - acceptedLocalization}}`;
const imuDetail = document.getElementById("imu-detail");
if (imuDetail) {{
  const summary = DATA.imu_summary || {{}};
  imuDetail.textContent = imu.length
    ? `IMU ${{imu.length}} 条 · 日志 ${{fmt(metadata.imu?.sample_rate_hz, 0)}} Hz / VQF ${{fmt(metadata.imu?.source_sample_rate_hz, 0)}} Hz · 有效采样率 ${{fmt(summary.effective_sample_rate_hz, 1)}} Hz · 加速度模长均值 ${{fmt(summary.signals?.acc_norm_g?.mean, 3)}} g · 四元数模长均值 ${{fmt(summary.signals?.quaternion_norm?.mean, 5)}}`
    : "IMU：暂无 240 Hz 日志记录（VQF仍运行于480 Hz）";
}}
const tuningDetail = document.getElementById("speed-tuning-detail");
if (tuningDetail) {{
  if (!speedTuning.sample_count) {{
    tuningDetail.textContent = "速度环整定：暂无控制 ISR 记录，当前运行不能用于调 KP/KI。";
  }} else {{
    tuningDetail.textContent = `速度环样本 ${{speedTuning.sample_count}} 条 · 平均速度误差 RMS ${{fmt(speedTuning.speed_error_rms_cmps, 1)}} cm/s · 稳态均值 ${{fmt(speedTuning.steady_speed_error_mean_cmps, 1)}} cm/s · 左/右 PWM 峰值 ${{fmt(speedTuning.pwm_l_abs_max, 0)}} / ${{fmt(speedTuning.pwm_r_abs_max, 0)}}`;
  }}
}}
window.addEventListener("resize", () => window.location.reload());
</script>
</body>
</html>
'''


def _write_csv(path, records, fieldnames):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)


def write_outputs(payload, output_directory):
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    payload = bytes(payload)
    (output / "telemetry.bin").write_bytes(payload)
    try:
        decoded = decode_payload(payload)
    except Exception:
        for name in DECODED_OUTPUT_NAMES:
            try:
                (output / name).unlink()
            except FileNotFoundError:
                pass
        raise

    planned_route = _load_planned_route(decoded["route_reference"])
    scan_features = _load_scan_features(
        decoded["route_reference"],
    )

    (output / "metadata.json").write_text(
        json.dumps(decoded["metadata"], indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    _write_csv(
        output / "control.csv",
        decoded["control"],
        CONTROL_FIELDS,
    )
    _write_csv(
        output / "navigation.csv",
        decoded["navigation"],
        NAVIGATION_FIELDS,
    )
    _write_csv(
        output / "phototube.csv",
        decoded["vision"],
        VISION_FIELDS,
    )
    _write_csv(
        output / "events.csv",
        decoded["events"],
        EVENT_FIELDS,
    )
    _write_csv(
        output / "localization_transactions.csv",
        decoded.get("localization", []),
        LOCALIZATION_FIELDS,
    )
    _write_csv(
        output / "localization_feature_diagnostics.csv",
        decoded.get("localization_features", []),
        LOCALIZATION_FEATURE_FIELDS,
    )
    _write_csv(
        output / "imu.csv",
        decoded.get("imu", []),
        IMU_FIELDS,
    )
    _write_csv(
        output / "phototube_scan.csv",
        decoded.get("phototube_scan", {}).get("points", []),
        PHOTOTUBE_SCAN_FIELDS,
    )
    _write_csv(output / "planned_route.csv", planned_route, ROUTE_FIELDS)
    codex_packet = _build_codex_packet(decoded, planned_route)
    analysis_summary = _build_analysis_summary(decoded, codex_packet)
    (output / "analysis_summary.json").write_text(
        json.dumps(analysis_summary, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    (output / "telemetry_overview.svg").write_text(
        _build_overview_svg(decoded, planned_route, analysis_summary, scan_features),
        encoding="utf-8",
    )
    (output / "telemetry_overview.png").write_bytes(
        _build_overview_png(decoded, planned_route, analysis_summary, scan_features)
    )
    (output / "codex_packet.json").write_text(
        json.dumps(codex_packet, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    (output / "codex_prompt.md").write_text(
        _build_codex_prompt(codex_packet),
        encoding="utf-8",
    )
    (output / "telemetry_dashboard.html").write_text(
        _build_dashboard_html(decoded, planned_route, scan_features),
        encoding="utf-8",
    )
    return decoded


def _read_exact(
    stream,
    size,
    empty_read_limit,
    progress_callback=None,
    progress_total=None,
    read_chunk_size=None,
):
    data = bytearray()
    empty_reads = 0
    while len(data) < size:
        request_size = size - len(data)
        if read_chunk_size is not None:
            request_size = min(request_size, read_chunk_size)
        chunk = stream.read(request_size)
        if chunk:
            data.extend(chunk)
            if progress_callback is not None:
                progress_callback(
                    len(data), size if progress_total is None else progress_total
                )
            empty_reads = 0
            continue
        empty_reads += 1
        if empty_read_limit is not None and empty_reads >= empty_read_limit:
            raise TimeoutError(f"serial stream ended after {len(data)} of {size} bytes")
    return bytes(data)


def _read_until_magic(stream, magic, empty_read_limit, timeout_message):
    window = bytearray()
    empty_reads = 0
    while bytes(window) != magic:
        chunk = stream.read(1)
        if chunk:
            window.extend(chunk)
            del window[: -len(magic)]
            empty_reads = 0
            continue
        empty_reads += 1
        if empty_read_limit is not None and empty_reads >= empty_read_limit:
            raise TimeoutError(timeout_message)
    return bytes(window)


def read_link_beacon(
    stream,
    empty_read_limit=None,
    deadline=None,
    clock=time.monotonic,
):
    buffer = bytearray()
    empty_reads = 0
    timeout_message = "timed out waiting for TLD1 telemetry beacon"

    while True:
        if deadline is not None and clock() >= deadline:
            raise TimeoutError(timeout_message)

        chunk = stream.read(1)
        if chunk:
            buffer.extend(chunk)
            empty_reads = 0
        else:
            empty_reads += 1
            if empty_read_limit is not None and empty_reads >= empty_read_limit:
                raise TimeoutError(timeout_message)
            continue

        marker_index = buffer.find(LINK_MAGIC)
        if marker_index < 0:
            keep = len(LINK_MAGIC) - 1
            if len(buffer) > keep:
                del buffer[: len(buffer) - keep]
            continue
        if marker_index > 0:
            del buffer[:marker_index]
        if len(buffer) < LINK_STRUCT.size:
            continue

        candidate = bytes(buffer[: LINK_STRUCT.size])
        magic, version, flags, payload_size, session_sequence = LINK_STRUCT.unpack(
            candidate
        )
        if magic == LINK_MAGIC and version != PROTOCOL_VERSION:
            raise TelemetryProtocolMismatchError(
                "TC264 telemetry beacon uses wire protocol "
                f"{version}, but this host supports only {PROTOCOL_VERSION}; "
                "flash firmware and use telemetry_capture.py from the same workspace"
            )
        if (
            magic == LINK_MAGIC
            and version == PROTOCOL_VERSION
            and payload_size == EMEM_SIZE
        ):
            return {
                "protocol_version": version,
                "flags": flags,
                "payload_size": payload_size,
                "session_sequence": session_sequence,
            }

        # Keep the remaining bytes so a valid marker overlapping this false
        # candidate can still be found.
        del buffer[0]


def find_telemetry_port(
    port_names,
    open_port,
    empty_read_limit=10,
    probe_timeout=None,
    clock=time.monotonic,
):
    failures = []
    protocol_mismatches = []
    for port_name in port_names:
        stream = None
        try:
            stream = open_port(port_name)
            deadline = None
            if probe_timeout is not None:
                deadline = clock() + probe_timeout
            beacon = read_link_beacon(
                stream,
                empty_read_limit=empty_read_limit,
                deadline=deadline,
                clock=clock,
            )
            return port_name, stream, beacon
        except TelemetryProtocolMismatchError as exc:
            protocol_mismatches.append(f"{port_name}: {exc}")
            if stream is not None:
                close = getattr(stream, "close", None)
                if close is not None:
                    close()
        except (OSError, TimeoutError, ValueError) as exc:
            failures.append(f"{port_name}: {exc}")
            if stream is not None:
                close = getattr(stream, "close", None)
                if close is not None:
                    close()
    if protocol_mismatches:
        raise TelemetryProtocolMismatchError("; ".join(protocol_mismatches))
    detail = "; ".join(failures) if failures else "no serial ports"
    raise RuntimeError(f"no TC264 telemetry beacon found ({detail})")


def format_progress(received, total, width=30):
    if total <= 0:
        ratio = 0.0
    else:
        ratio = max(0.0, min(1.0, received / total))
    filled = int(round(width * ratio))
    bar = "#" * filled + "-" * (width - filled)
    return f"\rReceiving [{bar}] {ratio * 100.0:5.1f}% {received}/{total} bytes"


def read_wire_payload(
    stream,
    empty_read_limit=None,
    payload_empty_read_limit=None,
    progress_callback=None,
):
    # Waiting for the operator to press KEY_1 may be indefinite, but once the
    # wire magic arrives a disconnected stream must eventually fail.
    if payload_empty_read_limit is None:
        payload_empty_read_limit = empty_read_limit
    _read_until_magic(
        stream,
        WIRE_MAGIC,
        empty_read_limit,
        "timed out waiting for TLG1",
    )

    prefix = WIRE_MAGIC + _read_exact(
        stream, WIRE_STRUCT.size - 4, payload_empty_read_limit
    )
    magic, version, flags, payload_size, wire_crc = WIRE_STRUCT.unpack(prefix)
    if magic != WIRE_MAGIC:
        raise ValueError("invalid wire magic")
    if version != PROTOCOL_VERSION:
        raise ValueError("unsupported wire protocol version")
    if payload_size != EMEM_SIZE:
        raise ValueError(f"invalid wire payload length: {payload_size}")
    if flags != WIRE_FLAG_ZERO_RLE:
        raise ValueError(f"unsupported wire flags: 0x{flags:04X}")

    payload_buffer = bytearray()
    while len(payload_buffer) < payload_size:
        block_header = struct.unpack(
            "<H", _read_exact(stream, 2, payload_empty_read_limit)
        )[0]
        block_length = (block_header & WIRE_BLOCK_LENGTH_MASK) + 1
        if len(payload_buffer) + block_length > payload_size:
            raise ValueError("wire block exceeds logical payload length")
        if block_header & WIRE_BLOCK_ZERO_FLAG:
            payload_buffer.extend(b"\x00" * block_length)
        else:
            payload_buffer.extend(
                _read_exact(
                    stream,
                    block_length,
                    payload_empty_read_limit,
                    read_chunk_size=PAYLOAD_READ_CHUNK_SIZE,
                )
            )
        if progress_callback is not None:
            progress_callback(len(payload_buffer), payload_size)
    payload = bytes(payload_buffer)
    computed_crc = crc32_excluding_header_crc(payload)
    if wire_crc != computed_crc:
        raise ValueError(
            f"wire CRC mismatch: prefix=0x{wire_crc:08X} computed=0x{computed_crc:08X}"
        )
    return payload


def _wait_for_telemetry_port(serial_module, args):
    from serial.tools import list_ports

    detect_read_timeout = 0.1
    detect_empty_limit = max(
        1, int(math.ceil(args.detect_timeout / detect_read_timeout))
    )
    while True:
        port_names = sorted(
            {
                port.device
                for port in list_ports.comports()
                if getattr(port, "device", None)
            }
        )
        if port_names:
            try:
                port_name, stream, beacon = find_telemetry_port(
                    port_names,
                    lambda name: serial_module.Serial(
                        name,
                        args.baud,
                        timeout=detect_read_timeout,
                    ),
                    empty_read_limit=detect_empty_limit,
                    probe_timeout=args.detect_timeout,
                )
                stream.timeout = args.timeout
                return port_name, stream, beacon
            except TelemetryProtocolMismatchError as exc:
                raise SystemExit(f"Telemetry protocol mismatch: {exc}") from exc
            except RuntimeError:
                pass
        sys.stdout.write("\rWaiting for TC264 telemetry beacon (TLD1)...")
        sys.stdout.flush()
        time.sleep(args.detect_interval)


def _capture_command(args):
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("capture requires pyserial: D:\\Miniconda3\\python.exe -m pip install pyserial") from exc

    if args.port is None or args.port.lower() == "auto":
        port_name, stream, beacon = _wait_for_telemetry_port(serial, args)
        ready = bool(beacon["flags"] & LINK_FLAG_RUN_FINALIZED)
        print(
            f"\nTC264 telemetry detected on {port_name}; "
            f"frozen run ready={'yes' if ready else 'no'}. "
            "Press KEY_1/S1 when the car is idle to export."
        )
    else:
        port_name = args.port
        stream = serial.Serial(port_name, args.baud, timeout=args.timeout)
        print(f"Connected to telemetry port {port_name}; press KEY_1/S1 when idle to export.")

    progress_started = False
    progress_last_update = 0.0

    def on_progress(received, total):
        nonlocal progress_started, progress_last_update
        now = time.monotonic()
        if received < total and (now - progress_last_update) < 0.05:
            return
        progress_started = True
        progress_last_update = now
        sys.stdout.write(format_progress(received, total))
        sys.stdout.flush()

    with stream:
        payload = read_wire_payload(
            stream,
            empty_read_limit=None,
            payload_empty_read_limit=3,
            progress_callback=on_progress,
        )
    if progress_started:
        print()
    write_outputs(payload, args.out)
    print(f"captured {len(payload)} bytes into {Path(args.out).resolve()}")


def _decode_command(args):
    payload = Path(args.input).read_bytes()
    write_outputs(payload, args.out)
    print(f"decoded {len(payload)} bytes into {Path(args.out).resolve()}")


def build_parser():
    parser = argparse.ArgumentParser(description="Capture and decode TC26x telemetry dumps")
    subparsers = parser.add_subparsers(dest="command", required=True)

    capture = subparsers.add_parser("capture", help="receive one UART telemetry dump")
    capture.add_argument(
        "--port",
        default=None,
        help="serial port, for example COM6; omit to auto-detect TLD1",
    )
    capture.add_argument("--out", required=True, help="output directory")
    capture.add_argument("--baud", type=int, default=921600)
    capture.add_argument("--timeout", type=float, default=1.0)
    capture.add_argument(
        "--detect-timeout",
        type=float,
        default=1.2,
        help="seconds to listen on each port during auto-detection",
    )
    capture.add_argument(
        "--detect-interval",
        type=float,
        default=0.5,
        help="seconds between auto-detection scans",
    )
    capture.set_defaults(handler=_capture_command)

    decode = subparsers.add_parser("decode", help="decode an existing 0x50000-byte payload")
    decode.add_argument("input", help="telemetry.bin path")
    decode.add_argument("--out", required=True, help="output directory")
    decode.set_defaults(handler=_decode_command)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    args.handler(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
