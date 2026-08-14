# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""An angular constraint commands a moment through f_ext, a linear one still commands a force.

Before this, an impedance on an orientation constraint emitted a *linear* force along the base
axis with magnitude `stiffness x angle_error` -- an orientation error lifted the arm, and nothing
warned. The commanded quantity now follows the constrained subspace.
"""

from __future__ import annotations

import pytest
from rdflib.namespace import RDF

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import QUDT_SCHEMA, RBDYN_OP, RBDYN_OP_EXT, SLV

SPEC_ANCHOR = "        linear-velocity zero-linvel = 0.0 m/s"
SPEC_WITH_ROT = SPEC_ANCHOR + ",\n        angle satisfied-band-rot = 0.01 rad"

WHILE_ANCHOR = (
    "        hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
)
CTRL_ANCHOR = (
    "        pid ctrl-hold-position { constraint: <home.hold-position>, "
    "Kp: 200, Ki: 100, Kd: 40, decay: 0 }"
)


def _swap(source: str, anchor: str, replacement: str) -> str:
    assert anchor in source, anchor
    return source.replace(anchor, replacement, 1)


def _orientation_source(base_source: str, subspace: str, controller: str) -> str:
    """The base model with an extra orientation constraint and the controller driving it."""
    source = _swap(base_source, SPEC_ANCHOR, SPEC_WITH_ROT)
    source = _swap(
        source,
        WHILE_ANCHOR,
        WHILE_ANCHOR + f",\n        hold-ori: keeping <shared.world.pose-ee-base>.{subspace}"
        " equal to <spec.home-pose>.orientation within <shared.spec.satisfied-band-rot>",
    )
    return _swap(source, CTRL_ANCHOR, CTRL_ANCHOR + ",\n        " + controller)


def _graph(parse_source, source):
    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


IMPEDANCE = (
    "impedance ctrl-hold-ori { constraint: <home.hold-ori>, stiffness: 40, damping: 8 }"
    " apply at <gripper.g_base>"
)


def test_orientation_impedance_emits_moment_wrench(parse_source, base_source) -> None:
    graph = _graph(parse_source, _orientation_source(base_source, "orientation", IMPEDANCE))

    moments = list(graph.subjects(RDF.type, RBDYN_OP_EXT.WrenchFromDirectionAndMoment))
    assert len(moments) == 3, "a whole orientation commands one moment per angular axis"
    assert len(list(graph.subjects(RDF.type, RBDYN_OP.AddWrench))) == 2

    specs = [
        spec
        for spec in graph.subjects(RDF.type, SLV.CartesianForceSpecification)
        if "ctrl-hold-ori" in str(spec)
    ]
    assert len(specs) == 1
    assert "g_base" in str(graph.value(specs[0], SLV["attached-to"]))

    # The moment goes to f_ext, so it claims no solver acceleration row.
    constraints = [
        node
        for node in graph.subjects(RDF.type, SLV.AccelerationConstraint)
        if "ctrl-hold-ori" in str(node)
    ]
    assert not constraints, f"moment command claimed solver rows: {constraints}"


def test_single_axis_orientation_impedance_emits_one_moment(parse_source, base_source) -> None:
    graph = _graph(parse_source, _orientation_source(base_source, "orientation.z", IMPEDANCE))

    ops = list(graph.subjects(RDF.type, RBDYN_OP_EXT.WrenchFromDirectionAndMoment))
    assert len(ops) == 1
    assert not list(graph.subjects(RDF.type, RBDYN_OP.AddWrench))

    magnitude = graph.value(ops[0], RBDYN_OP_EXT.moment)
    assert str(magnitude).endswith("moment-ctrl-hold-ori")
    # A moment is bounded and logged in N-m, never in N.
    assert graph.value(magnitude, QUDT_SCHEMA.unit) is not None
    assert "N-M" in str(graph.value(magnitude, QUDT_SCHEMA.unit))


def test_force_command_on_angular_subspace_is_rejected(parse_source, base_source) -> None:
    controller = (
        "pid ctrl-hold-ori { constraint: <home.hold-ori>, Kp: 40, Ki: 0, Kd: 8, decay: 0 }"
        " as force apply at <gripper.g_base>"
    )
    with pytest.raises(ValueError, match="commands a force on the angular subspace"):
        _graph(parse_source, _orientation_source(base_source, "orientation.z", controller))


def test_linear_impedance_still_commands_a_force(parse_source, base_source) -> None:
    """The regression guard for pick_place_dual's elbow-support controllers, which hold a
    `.position.z` with an impedance."""
    source = _swap(
        base_source,
        WHILE_ANCHOR,
        WHILE_ANCHOR + ",\n        support-z: keeping <shared.world.pose-ee-base>.position.z"
        " equal to <spec.home-pose>.position within <shared.spec.satisfied-band>",
    )
    source = _swap(
        source,
        CTRL_ANCHOR,
        CTRL_ANCHOR + ",\n        impedance ctrl-support-z { constraint: <home.support-z>,"
        " stiffness: 800, damping: 80 } apply at <gripper.g_base>",
    )
    graph = _graph(parse_source, source)

    assert list(graph.subjects(RDF.type, RBDYN_OP.WrenchFromPositionDirectionAndMagnitude))
    assert not list(graph.subjects(RDF.type, RBDYN_OP_EXT.WrenchFromDirectionAndMoment))
