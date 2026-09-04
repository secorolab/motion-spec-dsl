# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""`norm of <q>.<subspace> [across <direction>]` (plan 001): one generic scalar view backed by
one `geom-op-ext:VectorNorm` operator, with the vector's own kind and unit.
"""

from __future__ import annotations

import pytest
from rdflib.namespace import RDF

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import (
    CSTR,
    GEOM_COORD,
    GEOM_OP,
    GEOM_OP_EXT,
    MAP,
    MAP_EXT,
    QUDT_QKIND,
    QUDT_SCHEMA,
)

SPEC_ANCHOR = "        linear-velocity zero-linvel = 0.0 m/s"
WHILE_ANCHOR = (
    "        hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
)
CONTROLLER_ANCHOR = (
    "        pid ctrl-hold-position { constraint: <home.hold-position>, "
    "Kp: 200, Ki: 100, Kd: 40, decay: 0 }"
)

TABLE_DECLS = ",\n        direction table-normal { as-seen-by: <kinova.base_link.base_link_origin> } = (0, 0, 1)"
# Seen by the gripper, not the base: the same numbers name a different physical direction, which
# is exactly what the frame check has to catch.
FOREIGN_DECLS = (
    ",\n        direction gripper-normal { as-seen-by: <gripper.g_base.g_pinch> } = (0, 0, 1)"
)


def _source(
    base_source: str, *, spec: str = "", while_constraint: str, controller: str = ""
) -> str:
    source = base_source.replace(SPEC_ANCHOR, SPEC_ANCHOR + spec, 1)
    source = source.replace(WHILE_ANCHOR, WHILE_ANCHOR + f",\n{while_constraint}", 1)
    return source.replace(CONTROLLER_ANCHOR, CONTROLLER_ANCHOR + controller, 1)


def _graph(parse_source, source: str):
    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


def _norm_op(graph):
    ops = list(graph.subjects(RDF.type, GEOM_OP_EXT.VectorNorm))
    assert len(ops) == 1
    return ops[0]


def test_twist_linvel_norm_emits_operator_over_the_whole_vector(parse_source, base_source) -> None:
    source = _source(
        base_source,
        while_constraint=(
            "        speed: norm of <shared.world.twist-ee-base>.linvel greater than 0.05 m/s"
        ),
    )
    graph = _graph(parse_source, source)

    op = _norm_op(graph)
    assert graph.value(op, GEOM_OP.direction) is None

    vector = graph.value(op, GEOM_OP["in"])
    assert vector is not None
    views = [
        view
        for view in graph.subjects(MAP.subobject, vector)
        if (view, RDF.type, MAP_EXT.VelocityTwistCoordinateView) in graph
    ]
    assert len(views) == 1
    assert str(graph.value(views[0], MAP.superobject)).endswith("twist-ee-base")

    norm = graph.value(op, GEOM_OP_EXT.norm)
    assert norm is not None
    assert (norm, RDF.type, QUDT_SCHEMA.Quantity) in graph
    assert graph.value(norm, QUDT_SCHEMA.hasQuantityKind) == QUDT_QKIND.LinearVelocity
    assert "M-PER-SEC" in str(graph.value(norm, QUDT_SCHEMA.unit))

    constraints = [c for c in graph.subjects(CSTR.quantity, norm)]
    assert len(constraints) == 1


def test_norm_across_a_direction_wires_it_and_qualifies_the_id(parse_source, base_source) -> None:
    source = _source(
        base_source,
        spec=TABLE_DECLS,
        while_constraint=(
            "        speed: norm of <shared.world.twist-ee-base>.linvel across "
            "<shared.spec.table-normal> greater than 0.05 m/s"
        ),
    )
    graph = _graph(parse_source, source)

    op = _norm_op(graph)
    direction = graph.value(op, GEOM_OP.direction)
    assert direction is not None and str(direction).endswith("table-normal")

    norm = graph.value(op, GEOM_OP_EXT.norm)
    assert "norm-across-table-normal" in str(norm)


def test_pose_position_norm_is_a_linear_distance(parse_source, base_source) -> None:
    source = _source(
        base_source,
        while_constraint=(
            "        reach: norm of <shared.world.pose-ee-base>.position less than 0.5 m"
        ),
    )
    graph = _graph(parse_source, source)

    norm = graph.value(_norm_op(graph), GEOM_OP_EXT.norm)
    assert norm is not None
    assert (norm, RDF.type, GEOM_COORD.LinearDistanceCoordinate) in graph


def test_norm_of_an_orientation_is_rejected(parse_source, base_source) -> None:
    source = _source(
        base_source,
        while_constraint=(
            "        twisted: norm of <shared.world.pose-ee-base>.orientation less than 0.5 rad"
        ),
    )
    with pytest.raises(ValueError, match="not a 3-vector view"):
        _graph(parse_source, source)


def test_direction_in_another_frame_is_rejected(parse_source, base_source) -> None:
    source = _source(
        base_source,
        spec=FOREIGN_DECLS,
        while_constraint=(
            "        speed: norm of <shared.world.twist-ee-base>.linvel across "
            "<shared.spec.gripper-normal> greater than 0.05 m/s"
        ),
    )
    with pytest.raises(ValueError, match="seen by"):
        _graph(parse_source, source)


def test_controller_on_a_norm_view_is_rejected(parse_source, base_source) -> None:
    source = _source(
        base_source,
        while_constraint=(
            "        speed: norm of <shared.world.twist-ee-base>.linvel greater than 0.05 m/s"
        ),
        controller=(
            ",\n        pid ctrl-speed { constraint: <home.speed>, Kp: 1, Ki: 0, Kd: 0, decay: 0 }"
        ),
    )
    with pytest.raises(ValueError, match="nothing can command"):
        _graph(parse_source, source)
