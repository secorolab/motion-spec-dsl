# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A joint-space torque command reaches its solver as a joint force, whichever algorithm solves.

Both serial-chain algorithms consume one: ACHD takes it as the feed-forward torque input, RNE
adds it to the torque it computed. Emitting it for only one of them silently drops the command --
the controller still runs and still writes its signal, but nothing reads it.
"""

from __future__ import annotations

import pytest

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import SLV

WORLD_ANCHOR = """        velocity-twist twist-ee-base {
            of:         <gripper.g_base.g_pinch>,
            wrt:        <kinova.base_link.base_link_origin>,
            as-seen-by: <kinova.base_link.base_link_origin>
        }"""
WORLD_WITH_JOINT = (
    WORLD_ANCHOR
    + """,
        joint-position elbow-q {
            joint: <kinova.joint_4>
        }"""
)

SPEC_ANCHOR = "        linear-velocity zero-linvel = 0.0 m/s"
SPEC_WITH_JOINT = (
    SPEC_ANCHOR
    + """,
        angle elbow-hold = 0.5 rad,
        angle satisfied-band-ang = 0.01 rad"""
)

WHILE_ANCHOR = (
    "        hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
)
WHILE_WITH_JOINT = (
    WHILE_ANCHOR
    + """,
        hold-elbow-q: keeping <shared.world.elbow-q> equal to <shared.spec.elbow-hold> within <shared.spec.satisfied-band-ang>"""
)

CTRL_ANCHOR = (
    "        pid ctrl-hold-position { constraint: <home.hold-position>, "
    "Kp: 200, Ki: 100, Kd: 40, decay: 0 }"
)
CTRL_WITH_JOINT = (
    CTRL_ANCHOR
    + """,
        pid ctrl-elbow-q { constraint: <home.hold-elbow-q>, Kp: 5, Ki: 0, Kd: 1, decay: 0 } as torque"""
)


def _swap(source: str, anchor: str, replacement: str) -> str:
    assert anchor in source, anchor
    return source.replace(anchor, replacement, 1)


def _posture_torque_source(base_source: str, algorithm: str) -> str:
    source = _swap(base_source, WORLD_ANCHOR, WORLD_WITH_JOINT)
    source = _swap(source, SPEC_ANCHOR, SPEC_WITH_JOINT)
    source = _swap(source, WHILE_ANCHOR, WHILE_WITH_JOINT)
    source = _swap(source, CTRL_ANCHOR, CTRL_WITH_JOINT)
    return _swap(source, "algorithm: achd,", f"algorithm: {algorithm},")


@pytest.mark.parametrize("algorithm", ["achd", "rne"])
def test_a_posture_torque_reaches_its_solver_as_a_joint_force(
    parse_source, base_source, algorithm
) -> None:
    source = _posture_torque_source(base_source, algorithm)
    graph = MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph

    specs = list(graph.objects(None, SLV["joint-force"]))
    assert specs, f"{algorithm} solver got no joint force for its posture-torque command"
    # The force is a torque about the joint the constraint named, not a bare scalar.
    torque = graph.value(specs[0], SLV.force)
    assert torque is not None
    assert graph.value(specs[0], SLV["attached-to"]) is not None
