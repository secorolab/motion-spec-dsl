# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What the Borghesan Table II operators actually computed, against the geometry by hand.

The row-emission checks say a gradient reaches the solver; they say nothing about the maths
behind it. Both line-family operators once measured `B - A` where every point-family one measures
`A - B`, which no run and no compile can report -- the sign is only wrong against the scene.

Recorded run of `src/motion_spec_dsl/models/drawer_open`, Borghesan's own worked example:

    motion-spec run src/motion_spec_dsl/models/drawer_open/drawer_open.robmot \
        -o generations/drawer_open --prefix <install> --run-id <id> --headless
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import pytest

from motion_spec.introspection.frame_log_pb import frame_records, read_contract


RUN_ENV = "MOTION_SPEC_DRAWER_OPEN_RUN"
RUN_GLOB = "generations/drawer_open/drawer_open/*/runs/*/logs/frame_log.pb"
TOLERANCE = 1e-9

# The handle's own axes and the cabinet's inward normal, as the model states them in the base
# frame: hx the pull-out axis, hz along the bar, hy the vertical.
HX = (-1.0, 0.0, 0.0)
HY = (0.0, 0.0, -1.0)
HZ = (0.0, -1.0, 0.0)
INWARD = (1.0, 0.0, 0.0)
# The tool's own axes: z the approach direction, y the axis the pads close along.
APPROACH = (0.0, 0.0, 1.0)
GRASP_NORMAL = (0.0, 1.0, 0.0)


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

    The handle stands still in the kinematic graph, so its logged pose is subtracted rather than
    assumed away; every line and the face plane take their origin from it.
    """
    scalars, vectors = {}, {}
    if not {"pose_grasp", "pose_handle"} <= set(poses):
        return scalars, vectors

    grasp, handle = poses["pose_grasp"], poses["pose_handle"]
    # The grasp point relative to the handle: what every point-family operator reads.
    p = _sub((grasp["px"], grasp["py"], grasp["pz"]), (handle["px"], handle["py"], handle["pz"]))
    radial = _sub(p, _scale(HZ, _dot(p, HZ)))
    approach = _rotate(grasp, APPROACH)
    grasp_normal = _rotate(grasp, GRASP_NORMAL)
    # `grasp-inward-line` runs along +x through the pinch, `handle-hz` along -y through the
    # handle, so their common normal is -z: the gap is how far the tip sits above the bar.
    common_normal = _unit(_cross(INWARD, HZ))

    # The same projection is emitted once per motion that names the row.
    for motion in ("pre_approach", "approach_handle", "grasp_handle", "pull_drawer"):
        for row, axis in (("at_handle_x", HX), ("at_handle_y", HY), ("at_handle_z", HZ)):
            name = f"geo_distance_{motion}_{row}_point_line_projection"
            scalars[name] = _dot(p, axis)
            vectors[f"{name}_gradient"] = axis
    for row, axis in (("centered_x", HX), ("centered_y", HY), ("centered_z", HZ)):
        name = f"geo_distance_approach_handle_{row}_point_line_projection"
        scalars[name] = _dot(p, axis)
        vectors[f"{name}_gradient"] = axis

    scalars |= {
        "geo_distance_pre_approach_at_pre_x_point_line_projection": _dot(p, HX),
        "geo_distance_pull_drawer_pull_open_point_plane_distance": _dot(INWARD, p),
        "geo_distance_approach_handle_face_clearance_point_plane_distance": _dot(INWARD, p),
        "geo_distance_approach_handle_off_bar_axis_point_line_distance": _norm(radial),
        "geo_distance_approach_handle_axis_gap_line_line_distance": _dot(p, common_normal),
        "geo_distance_pull_drawer_axis_travel_line_line_projection": _dot(p, HX),
        "pose_grasp_alignment_approach_axis_handle_face_square_target_cone": math.asin(
            max(-1.0, min(1.0, _dot(approach, INWARD)))
        ),
        "pose_grasp_alignment_grasp_normal_handle_z_base_aligned_cone": _angle(grasp_normal, HZ),
    }
    vectors |= {
        "geo_distance_pre_approach_at_pre_x_point_line_projection_gradient": HX,
        "geo_distance_pull_drawer_pull_open_point_plane_distance_gradient": INWARD,
        "geo_distance_approach_handle_face_clearance_point_plane_distance_gradient": INWARD,
        "geo_distance_approach_handle_off_bar_axis_point_line_distance_gradient": _unit(radial),
        "geo_distance_approach_handle_axis_gap_line_line_distance_gradient": common_normal,
        "geo_distance_pull_drawer_axis_travel_line_line_projection_gradient": HX,
        "pose_grasp_alignment_approach_axis_handle_face_square_target_cone_gradient": _unit(
            _cross(approach, INWARD)
        ),
        # Opposite in sign to the plane operator above, and correctly so: turning the tool
        # about cross(a, b) carries a towards b, which raises the elevation over a plane but
        # closes the angle between two directions.
        "pose_grasp_alignment_grasp_normal_handle_z_base_aligned_cone_gradient": _unit(
            _cross(HZ, grasp_normal)
        ),
    }
    return scalars, vectors


def _frame_log() -> Path:
    override = os.environ.get(RUN_ENV)
    if override:
        return Path(override)
    workspace = Path(__file__).resolve().parents[4]
    logs = sorted(workspace.glob(RUN_GLOB))
    if not logs:
        pytest.skip(f"no recorded drawer_open run under {workspace / RUN_GLOB}")
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
        for name, value in scalars.items():
            if name in quantities:
                seen.add(name)
                wanted.add(name)
                assert quantities[name] == pytest.approx(value, abs=TOLERANCE), name
        for name, value in vectors.items():
            axes = [quantities.get(f"{name}.{axis}") for axis in "xyz"]
            if all(axis is not None for axis in axes):
                seen.add(name)
                wanted.add(name)
                assert _norm(_sub(tuple(axes), value)) < TOLERANCE, f"{name}={axes} want {value}"

    # Every Table II family has to have been exercised, or the run proves nothing.
    families = {
        "point_plane_distance",
        "point_line_distance",
        "point_line_projection",
        "line_line_distance",
        "line_line_projection",
        "square_target_cone",
        "aligned_cone",
    }
    missing = sorted(family for family in families if not any(family in name for name in seen))
    assert not missing, f"the recorded run logs no: {missing} -- regenerate it"
    assert last is not None
    assert last["fsm_state"] == contract.header.end_state, "the recorded run never reached S_DONE"
