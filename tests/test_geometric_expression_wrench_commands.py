# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A force or moment controller on a Table IIa/IIb geometric expression (plans 08-10) has no
named axis to build its command wrench from; it must reuse the expression's own runtime
gradient `DirectionCoordinate`, the one `_emit_map_operations` already publishes.
"""

from __future__ import annotations

from rdflib.namespace import RDF

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import GEOM_OP_EXT, RBDYN_OP, RBDYN_OP_EXT

SPEC_ANCHOR = "        linear-velocity zero-linvel = 0.0 m/s"
WHILE_ANCHOR = (
    "        hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
)
CTRL_ANCHOR = (
    "        pid ctrl-hold-position { constraint: <home.hold-position>, "
    "Kp: 200, Ki: 100, Kd: 40, decay: 0 }"
)
WORLD_ANCHOR = (
    "        velocity-twist twist-ee-base {\n"
    "            of:         <gripper.g_base.g_pinch>,\n"
    "            wrt:        <kinova.base_link.base_link_origin>,\n"
    "            as-seen-by: <kinova.base_link.base_link_origin>\n"
    "        }"
)


def _swap(source: str, anchor: str, replacement: str) -> str:
    assert anchor in source, anchor
    return source.replace(anchor, replacement, 1)


def _graph(parse_source, source: str):
    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


POSE_TABLE_TOP_DECL = (
    ",\n        pose pose-table-top {\n"
    "            of:         <table.table_top>,\n"
    "            wrt:        <kinova.base_link.base_link_origin>,\n"
    "            as-seen-by: <kinova.base_link.base_link_origin>\n"
    "        }"
)
TABLE_DECLS = (
    ",\n        direction table-normal { as-seen-by: <kinova.base_link.base_link_origin> } = (0, 0, 1)"
    ",\n        plane table { of: <table.table_top>, normal: <shared.spec.table-normal> }"
)


def test_force_command_on_point_plane_distance_wires_gradient_direction(
    parse_source, base_source
) -> None:
    """A force controller on `distance of <pose> from <plane>` (plan 08) has no axis and is not
    the point-point `distance between` form either -- it must fall onto the PointPlaneToLinearDistance
    op's own gradient, not raise.
    """
    source = _swap(base_source, WORLD_ANCHOR, WORLD_ANCHOR + POSE_TABLE_TOP_DECL)
    source = _swap(source, SPEC_ANCHOR, SPEC_ANCHOR + TABLE_DECLS)
    source = _swap(
        source,
        WHILE_ANCHOR,
        WHILE_ANCHOR + ",\n        above-table: keeping distance of "
        "<shared.world.pose-ee-base> from <shared.spec.table> greater than 0.05 m",
    )
    source = _swap(
        source,
        CTRL_ANCHOR,
        CTRL_ANCHOR + ",\n        pid ctrl-above-table { constraint: <home.above-table>, "
        "Kp: 100, Ki: 0, Kd: 0, decay: 0 } as force apply at <gripper.g_base>",
    )
    graph = _graph(parse_source, source)

    distance_ops = list(graph.subjects(RDF.type, GEOM_OP_EXT.PointPlaneToLinearDistance))
    assert len(distance_ops) == 1
    gradient = graph.value(distance_ops[0], GEOM_OP_EXT.gradient)
    assert gradient is not None

    wrench_ops = list(graph.subjects(RDF.type, RBDYN_OP.WrenchFromPositionDirectionAndMagnitude))
    assert len(wrench_ops) == 1
    assert graph.value(wrench_ops[0], RBDYN_OP.direction) == gradient


def test_moment_command_on_incident_angle_wires_gradient_direction(
    parse_source, base_source
) -> None:
    """An impedance on `angle between <versor> and <plane>` (incident angle, plan 09) commands a
    torque with no named angular axis -- it must fall onto the DirectionPlaneToAngularDistance op's own gradient,
    one wrench, no AddWrench fold.
    """
    spec_additions = (
        ",\n        direction tool-up { as-seen-by: <gripper.g_base.g_pinch> } = (0, 0, -1)"
        ",\n        direction table-normal { as-seen-by: <kinova.base_link.base_link_origin> } = (0, 0, 1)"
        ",\n        plane table { of: <kinova.base_link.base_link_origin>, "
        "normal: <shared.spec.table-normal> }"
        ",\n        angle align-band = 0.05 rad"
    )
    source = _swap(base_source, SPEC_ANCHOR, SPEC_ANCHOR + spec_additions)
    source = _swap(
        source,
        WHILE_ANCHOR,
        WHILE_ANCHOR + ",\n        tool-in-table: keeping angle between "
        "<shared.spec.tool-up> and <shared.spec.table> equal to 0 rad within "
        "<shared.spec.align-band>",
    )
    source = _swap(
        source,
        CTRL_ANCHOR,
        CTRL_ANCHOR + ",\n        impedance ctrl-tool-in-table { "
        "constraint: <home.tool-in-table>, stiffness: 40, damping: 8 } "
        "apply at <gripper.g_base>",
    )
    graph = _graph(parse_source, source)

    angle_ops = list(graph.subjects(RDF.type, GEOM_OP_EXT.DirectionPlaneToAngularDistance))
    assert len(angle_ops) == 1
    gradient = graph.value(angle_ops[0], GEOM_OP_EXT.gradient)
    assert gradient is not None

    moment_ops = list(graph.subjects(RDF.type, RBDYN_OP_EXT.WrenchFromDirectionAndMoment))
    assert len(moment_ops) == 1
    assert graph.value(moment_ops[0], RBDYN_OP.direction) == gradient
    assert not list(graph.subjects(RDF.type, RBDYN_OP.AddWrench))
