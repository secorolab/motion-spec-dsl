# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What the Borghesan Table II operators actually computed, against the geometry by hand.

The row-emission checks say a gradient reaches the solver; they say nothing about the maths
behind it. Both line-family operators once measured `B - A` where every point-family one measures
`A - B`, which no run and no compile can report -- the sign is only wrong against the scene.

Recorded run of `models/tableii_probe`:

    motion-spec run models/tableii_probe/tableii_probe.robmot \
        -o generations/tableii_probe --prefix <install> --run-id <id> --headless
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import pytest

from motion_spec.introspection.frame_log_pb import frame_records, read_contract


RUN_ENV = "MOTION_SPEC_TABLEII_RUN"
RUN_GLOB = "generations/tableii_probe/tableii_probe/*/runs/*/logs/frame_log.pb"
# The band `tilt.align-forearm` tolerates, as authored; the rotation vector answers for what
# exceeds it, not for the whole angle.
ALIGN_CONE = 0.05
TOLERANCE = 1e-9

UP = (0.0, 0.0, 1.0)
EX = (1.0, 0.0, 0.0)
EY = (0.0, 1.0, 0.0)
# `shared.spec.shoulder-up`, as seen by the shoulder.
SHOULDER_UP = (0.0, 0.0, -1.0)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def _norm(a):
    return math.sqrt(_dot(a, a))


def _unit(a):
    length = _norm(a)
    return _scale(a, 1.0 / length) if length > 1e-12 else (0.0, 0.0, 0.0)


def _rotate(pose, v):
    """`v` turned by the pose's quaternion: v + 2q_w(q_v x v) + 2 q_v x (q_v x v)."""
    q = (pose["qx"], pose["qy"], pose["qz"])
    t = _scale(_cross(q, v), 2.0)
    return tuple(a + pose["qw"] * b + c for a, b, c in zip(v, t, _cross(q, t)))


def _angle(a, b):
    return math.atan2(_norm(_cross(a, b)), _dot(a, b))


def _expected(poses):
    """Every operator's scalar and gradient, from the scene's geometry alone.

    The table top is the arm's own base frame, so its logged pose is subtracted rather than
    assumed away; `rail-x`, `rail-vert` and `table` all take their origin from it.
    """
    scalars, vectors = {}, {}
    if {"tcp", "pose_table_top"} <= set(poses):
        tcp, table = poses["tcp"], poses["pose_table_top"]
        # The tool tip relative to the table top: what every point-family operator reads.
        p = _sub((tcp["px"], tcp["py"], tcp["pz"]), (table["px"], table["py"], table["pz"]))
        radial = _sub(p, _scale(UP, _dot(p, UP)))
        tool_axis = _rotate(tcp, (0.0, 0.0, 1.0))
        tool_side = _rotate(tcp, (1.0, 0.0, 0.0))
        # `probe-line` runs along +y through the tool tip, `rail-x` along +x through the table
        # top, so their common normal is -z: the gap is how far the tip sits below the rail.
        common_normal = _unit(_cross(EY, EX))
        scalars |= {
            "geo_distance_traverse_table_clearance_point_plane_distance": _dot(UP, p),
            "geo_distance_traverse_tcp_radius_point_line_distance": _norm(radial),
            "geo_distance_traverse_rail_travel_point_line_projection": _dot(p, EX),
            "geo_distance_lines_line_gap_line_line_distance": _dot(p, common_normal),
            "geo_distance_lines_line_travel_line_line_projection": _dot(p, EX),
            "tcp_alignment_tool_axis_table_tool_incline_target_cone": math.asin(
                max(-1.0, min(1.0, _dot(tool_axis, UP)))
            ),
            "tcp_alignment_tool_side_base_up_tool_side_level_target_cone": _angle(tool_side, UP),
        }
        vectors |= {
            "geo_distance_traverse_table_clearance_point_plane_distance_gradient": UP,
            "geo_distance_traverse_tcp_radius_point_line_distance_gradient": _unit(radial),
            "geo_distance_traverse_rail_travel_point_line_projection_gradient": EX,
            "geo_distance_lines_line_gap_line_line_distance_gradient": common_normal,
            "geo_distance_lines_line_travel_line_line_projection_gradient": EX,
            "tcp_alignment_tool_axis_table_tool_incline_target_cone_gradient": _unit(
                _cross(tool_axis, UP)
            ),
            "tcp_alignment_tool_side_base_up_tool_side_level_target_cone_gradient": _unit(
                _cross(tool_side, UP)
            ),
        }
    # The forearm/shoulder cone belongs to `tilt` alone, so its pose is logged only there.
    if "pose_forearm_shoulder" in poses:
        forearm_up = _rotate(poses["pose_forearm_shoulder"], (0.0, 1.0, 0.0))
        cone = _angle(forearm_up, SHOULDER_UP)
        cone_axis = _cross(forearm_up, SHOULDER_UP)
        sin_cone = _norm(cone_axis)
        scalars["pose_forearm_shoulder_alignment_forearm_up_shoulder_up_align_cone"] = cone
        vectors["pose_forearm_shoulder_alignment_forearm_up_shoulder_up_align_cone_error"] = (
            _scale(cone_axis, max(0.0, cone - ALIGN_CONE) / sin_cone)
            if sin_cone > 1e-9
            else (0.0, 0.0, 0.0)
        )
    return scalars, vectors


def _frame_log() -> Path:
    override = os.environ.get(RUN_ENV)
    if override:
        return Path(override)
    workspace = Path(__file__).resolve().parents[4]
    logs = sorted(workspace.glob(RUN_GLOB))
    if not logs:
        pytest.skip(f"no recorded tableii_probe run under {workspace / RUN_GLOB}")
    return logs[-1]


def test_table_ii_operators_compute_the_scene_geometry() -> None:
    log = _frame_log()
    contract = read_contract(log)
    pose_names = [field["id"] for field in contract.fields["poses"]]

    seen: set[str] = set()
    wanted: set[str] = set()
    last = None
    for record in frame_records(log, contract):
        last = record
        quantities, poses = record["quantities"], record["poses"]
        if not quantities or poses is None:
            continue
        by_name = {name: pose for name, pose in zip(pose_names, poses) if pose is not None}
        scalars, vectors = _expected(by_name)
        wanted |= set(scalars) | set(vectors)
        for name, value in scalars.items():
            if name in quantities:
                seen.add(name)
                assert quantities[name] == pytest.approx(value, abs=TOLERANCE), name
        for name, value in vectors.items():
            axes = [quantities.get(f"{name}.{axis}") for axis in "xyz"]
            if all(axis is not None for axis in axes):
                seen.add(name)
                assert _norm(_sub(tuple(axes), value)) < TOLERANCE, f"{name}={axes} want {value}"

    missing = sorted(wanted - seen)
    assert wanted and not missing, f"the recorded run logs none of: {missing} -- regenerate it"
    assert last is not None
    assert last["fsm_state"] == contract.header.end_state, "the recorded run never reached S_DONE"
