# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Table IIa distance/projection expressions (plan 08): `distance of A from B`, `projection of
A on B`, dispatched on operand type to one of five geom-op-ext operators.
"""

from __future__ import annotations

import pytest
from rdflib.namespace import RDF
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import (
    GEOM_COORD,
    GEOM_ENT,
    GEOM_EXT,
    GEOM_OP,
    GEOM_OP_EXT,
    GEOM_REL,
    GEOM_REL_EXT,
    QUDT_SCHEMA,
)

SPEC_ANCHOR = "        linear-velocity zero-linvel = 0.0 m/s"
WHILE_ANCHOR = (
    "        hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
)
# A Table IIa expression reads an already-computed pose (same requirement `angle between`
# already has); it does not derive one, so every primitive's origin needs a matching
# `context.world` declaration -- table.table_top gives a frame distinct from pose-ee-base's own.
WORLD_ANCHOR = (
    "        velocity-twist twist-ee-base {\n"
    "            of:         <gripper.g_base.g_pinch>,\n"
    "            wrt:        <kinova.base_link.base_link_origin>,\n"
    "            as-seen-by: <kinova.base_link.base_link_origin>\n"
    "        }"
)
POSE_TABLE_TOP_DECL = (
    ",\n        pose pose-table-top {\n"
    "            of:         <table.table_top>,\n"
    "            wrt:        <kinova.base_link.base_link_origin>,\n"
    "            as-seen-by: <kinova.base_link.base_link_origin>\n"
    "        }"
)

TABLE_DECLS = (
    ",\n        direction table-normal { as-seen-by: <kinova.base_link.base_link_origin> } = (0, 0, 1)"
    ",\n        plane table { of: <table.table_top>, "
    "normal: <shared.spec.table-normal> }"
)
RAIL_DECLS = (
    ",\n        direction rail-axis { as-seen-by: <kinova.base_link.base_link_origin> } = (1, 0, 0)"
    ",\n        line rail { of: <table.table_top>, along: <shared.spec.rail-axis> }"
)
RAIL_A_DECLS = (
    ",\n        direction rail-a-axis { as-seen-by: <kinova.base_link.base_link_origin> } = (1, 0, 0)"
    ",\n        line rail-a { of: <gripper.g_base.g_pinch>, along: <shared.spec.rail-a-axis> }"
)
RAIL_B_DECLS = (
    ",\n        direction rail-b-axis { as-seen-by: <kinova.base_link.base_link_origin> } = (0, 1, 0)"
    ",\n        line rail-b { of: <table.table_top>, along: <shared.spec.rail-b-axis> }"
)


def _source(
    base_source: str, *, spec: str = "", while_constraint: str, world: str = POSE_TABLE_TOP_DECL
) -> str:
    source = base_source.replace(WORLD_ANCHOR, WORLD_ANCHOR + world, 1)
    source = source.replace(SPEC_ANCHOR, SPEC_ANCHOR + spec, 1)
    return source.replace(WHILE_ANCHOR, WHILE_ANCHOR + f",\n{while_constraint}", 1)


def _graph(parse_source, source: str):
    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


def test_point_plane_distance_emits_operator_with_gradient(parse_source, base_source) -> None:
    source = _source(
        base_source,
        spec=TABLE_DECLS,
        while_constraint=(
            "        above-table: keeping distance of <shared.world.pose-ee-base> from "
            "<shared.spec.table> greater than 0.05 m"
        ),
    )
    graph = _graph(parse_source, source)

    ops = list(graph.subjects(RDF.type, GEOM_OP_EXT.PointPlaneToLinearDistance))
    assert len(ops) == 1
    op = ops[0]

    distance = graph.value(op, GEOM_OP.distance)
    assert distance is not None
    assert (distance, RDF.type, GEOM_COORD.LinearDistanceCoordinate) in graph

    gradient = graph.value(op, GEOM_OP_EXT.gradient)
    assert gradient is not None
    assert (gradient, RDF.type, GEOM_COORD.DirectionCoordinate) in graph
    assert "UNITLESS" in str(graph.value(gradient, QUDT_SCHEMA.unit))


def test_line_line_projection_preserves_operand_order(parse_source, base_source) -> None:
    source = _source(
        base_source,
        spec=RAIL_A_DECLS + RAIL_B_DECLS,
        while_constraint=(
            "        rail-progress: keeping projection of <shared.spec.rail-a> on "
            "<shared.spec.rail-b> equal to 0.02 m within <shared.spec.satisfied-band>"
        ),
    )
    graph = _graph(parse_source, source)

    ops = list(graph.subjects(RDF.type, GEOM_OP_EXT.LineOnLineProjection))
    assert len(ops) == 1
    op = ops[0]

    in1 = graph.value(op, GEOM_OP.in1)
    in2 = graph.value(op, GEOM_OP.in2)
    assert in1 is not None and str(in1).endswith("rail-a-axis")
    assert in2 is not None and str(in2).endswith("rail-b-axis")

    pose = graph.value(op, GEOM_OP.pose)
    assert pose is not None
    diff_ops = [
        node
        for node in graph.subjects(RDF.type, GEOM_OP_EXT.PoseDiffEvaluator)
        if graph.value(node, GEOM_OP.out) == pose
    ]
    assert len(diff_ops) == 1
    diff_in1 = graph.value(diff_ops[0], GEOM_OP.in1)
    diff_in2 = graph.value(diff_ops[0], GEOM_OP.in2)
    assert diff_in1 is not None and diff_in1 != diff_in2
    # Order is carried through to which declared world pose lands as diff.in1/in2: rail-a's own
    # origin frame (gripper.g_base.g_pinch, i.e. pose-ee-base) first, rail-b's second. Swapping
    # the authored operands would swap these, silently computing s2 instead of s1.
    assert str(diff_in1).endswith("pose-ee-base")
    assert str(diff_in2).endswith("pose-table-top")


def test_distance_relation_is_shared_by_operand_pair(parse_source, base_source) -> None:
    source = _source(
        base_source,
        spec=TABLE_DECLS,
        while_constraint=(
            "        above-table: keeping distance of <shared.world.pose-ee-base> from "
            "<shared.spec.table> greater than 0.05 m,\n"
            "        clear-of-table: keeping distance of <shared.world.pose-ee-base> from "
            "<shared.spec.table> less than 0.5 m"
        ),
    )
    graph = _graph(parse_source, source)

    # A point-plane operand pair mints geom-rel-ext:PointPlaneDistance, never the bare
    # geom-rel:LinearDistance base -- the relation must say *which* kind of pair this is.
    assert list(graph.subjects(RDF.type, GEOM_REL.LinearDistance)) == []
    relations = list(graph.subjects(RDF.type, GEOM_REL_EXT.PointPlaneDistance))
    assert len(relations) == 1
    relation = relations[0]
    coordinates = list(graph.subjects(GEOM_COORD.of, relation))
    assert len(coordinates) == 2

    entities = list(graph.objects(relation, GEOM_REL["between-entities"]))
    assert len(entities) == 2
    assert sum(1 for e in entities if (e, RDF.type, GEOM_ENT.Point) in graph) == 1
    assert sum(1 for e in entities if (e, RDF.type, GEOM_EXT.Plane) in graph) == 1


def test_unsigned_distance_to_zero_is_rejected(parse_source, base_source) -> None:
    source = _source(
        base_source,
        spec=RAIL_DECLS,
        while_constraint=(
            "        on-rail: keeping distance of <shared.world.pose-ee-base> from "
            "<shared.spec.rail> equal to 0 m"
        ),
    )
    with pytest.raises(TextXSemanticError, match="drives an unsigned distance to zero"):
        parse_source(source)


def test_signed_distance_to_zero_is_allowed(parse_source, base_source) -> None:
    source = _source(
        base_source,
        spec=TABLE_DECLS,
        while_constraint=(
            "        touching-table: keeping distance of <shared.world.pose-ee-base> from "
            "<shared.spec.table> equal to 0 m within <shared.spec.satisfied-band>"
        ),
    )
    _graph(parse_source, source)


def test_projection_on_a_plane_is_rejected(parse_source, base_source) -> None:
    source = _source(
        base_source,
        spec=TABLE_DECLS,
        while_constraint=(
            "        bad-projection: keeping projection of <shared.world.pose-ee-base> on "
            "<shared.spec.table> equal to 0 m"
        ),
    )
    with pytest.raises(TextXSemanticError, match="projects onto a line"):
        parse_source(source)


def test_reversed_operand_order_is_rejected(parse_source, base_source) -> None:
    source = _source(
        base_source,
        spec=TABLE_DECLS,
        while_constraint=(
            "        bad-order: keeping distance of <shared.spec.table> from "
            "<shared.world.pose-ee-base> greater than 0.05 m"
        ),
    )
    with pytest.raises(TextXSemanticError, match="the point .or line. must come first"):
        parse_source(source)
