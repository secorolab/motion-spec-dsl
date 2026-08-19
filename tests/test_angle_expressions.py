# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Table IIb's other two `angle between` forms: incident angle (versor-plane) and the angle
between planes (plane-plane), dispatched off the same `angle between A and B` production
versor-versor already uses. Sign tests come first -- a flipped gradient drives a controller away
from its target and no graph assertion catches it.
"""

from __future__ import annotations

import math

import pytest
from rdflib.namespace import RDF
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import GEOM_COORD, GEOM_OP, GEOM_OP_EXT, QUDT_SCHEMA


def _cross(a: tuple, b: tuple) -> tuple:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot(a: tuple, b: tuple) -> float:
    return sum(x * y for x, y in zip(a, b))


def _normalize(v: tuple) -> tuple:
    norm = math.sqrt(_dot(v, v))
    return tuple(c / norm for c in v)


def _rotate(vector: tuple, axis: tuple, angle: float) -> tuple:
    """Rodrigues: `vector` turned by `angle` about the unit `axis`."""
    cross = _cross(axis, vector)
    dot = _dot(axis, vector)
    return tuple(
        vector[i] * math.cos(angle)
        + cross[i] * math.sin(angle)
        + axis[i] * dot * (1.0 - math.cos(angle))
        for i in range(3)
    )


def test_incident_angle_gradient_increases_the_angle() -> None:
    """theta = asin(n.v); gradient = normalize(v x n). Sanity check from plan 09 Sec.1:
    v=x-hat, n=z-hat => v x n = -y-hat; rotating x-hat about -y-hat carries it toward z-hat.
    """
    v = (1.0, 0.0, 0.0)
    n = (0.0, 0.0, 1.0)
    gradient = _normalize(_cross(v, n))
    before = math.asin(_dot(n, v))
    turned = _rotate(v, gradient, 1e-3)
    after = math.asin(_dot(n, turned))
    assert after > before


def test_plane_angle_gradient_increases_the_angle() -> None:
    """theta = atan2(norm(n1 x n2), n1.n2); gradient = normalize(n2 x n1)."""
    n1 = (1.0, 0.0, 0.0)
    n2 = (0.0, 1.0, 0.0)
    gradient = _normalize(_cross(n2, n1))

    def theta(a: tuple, b: tuple) -> float:
        return math.atan2(math.sqrt(_dot(_cross(a, b), _cross(a, b))), _dot(a, b))

    before = theta(n1, n2)
    turned = _rotate(n1, gradient, 1e-3)
    after = theta(turned, n2)
    assert after > before


SPEC_ANCHOR = "        linear-velocity zero-linvel = 0.0 m/s"
SPEC_WITH_PRIMITIVES = SPEC_ANCHOR + (
    ",\n        direction tool-up { as-seen-by: <gripper.g_base.g_pinch> } = (0, 0, -1)"
    ",\n        direction table-normal { as-seen-by: <kinova.base_link.base_link_origin> } = (0, 0, 1)"
    ",\n        plane gripper-plane { of: <gripper.g_base.g_pinch>, normal: <shared.spec.tool-up> }"
    ",\n        plane table { of: <kinova.base_link.base_link_origin>, normal: <shared.spec.table-normal> }"
    ",\n        angle align-band = 0.05 rad"
)
WHILE_ANCHOR = (
    "        hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
)
WHILE_WITH_INCIDENT = WHILE_ANCHOR + (
    ",\n        tool-in-table: keeping angle between <shared.spec.tool-up> and "
    "<shared.spec.table> equal to 0 rad within <shared.spec.align-band>"
)
WHILE_WITH_PLANE_ANGLE = WHILE_ANCHOR + (
    ",\n        planes-square: keeping angle between <shared.spec.gripper-plane> and "
    "<shared.spec.table> equal to 0 rad within <shared.spec.align-band>"
)
WHILE_WITH_VERSOR = WHILE_ANCHOR + (
    ",\n        align-tool-z: keeping angle between <shared.spec.tool-up> and "
    "<shared.spec.table-normal> equal to 0 rad within <shared.spec.align-band>"
)
CTRL_ANCHOR = (
    "        pid ctrl-hold-position { constraint: <home.hold-position>, "
    "Kp: 200, Ki: 100, Kd: 40, decay: 0 }"
)
CTRL_WITH_INCIDENT = CTRL_ANCHOR + (
    ",\n        pid ctrl-tool-in-table { constraint: <home.tool-in-table>, "
    "Kp: 120, Ki: 50, Kd: 80, decay: 0 }"
)
CTRL_WITH_PLANE_ANGLE = CTRL_ANCHOR + (
    ",\n        pid ctrl-planes-square { constraint: <home.planes-square>, "
    "Kp: 120, Ki: 50, Kd: 80, decay: 0 }"
)
CTRL_WITH_VERSOR = CTRL_ANCHOR + (
    ",\n        pid ctrl-align-tool-z { constraint: <home.align-tool-z>, "
    "Kp: 120, Ki: 50, Kd: 80, decay: 0 }"
)


def _source(base_source: str, while_with: str, ctrl_with: str) -> str:
    source = base_source.replace(SPEC_ANCHOR, SPEC_WITH_PRIMITIVES, 1)
    source = source.replace(WHILE_ANCHOR, while_with, 1)
    return source.replace(CTRL_ANCHOR, ctrl_with, 1)


def _graph(parse_source, base_source: str, while_with: str, ctrl_with: str):
    source = _source(base_source, while_with, ctrl_with)
    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


def test_incident_angle_emits_one_operator_with_gradient(parse_source, base_source) -> None:
    graph = _graph(parse_source, base_source, WHILE_WITH_INCIDENT, CTRL_WITH_INCIDENT)

    ops = list(graph.subjects(RDF.type, GEOM_OP_EXT.IncidentAngle))
    assert len(ops) == 1

    theta = graph.value(ops[0], GEOM_OP.angle)
    assert theta is not None
    assert "RAD" in str(graph.value(theta, QUDT_SCHEMA.unit))

    gradient = graph.value(ops[0], GEOM_OP_EXT.gradient)
    assert gradient is not None
    assert GEOM_COORD.DirectionCoordinate in set(graph.objects(gradient, RDF.type))
    assert "UNITLESS" in str(graph.value(gradient, QUDT_SCHEMA.unit))


def test_plane_angle_reuses_planar_angle_and_gradient_ops(parse_source, base_source) -> None:
    graph = _graph(parse_source, base_source, WHILE_WITH_PLANE_ANGLE, CTRL_WITH_PLANE_ANGLE)

    assert list(graph.subjects(RDF.type, GEOM_OP_EXT.IncidentAngle)) == []
    angle_ops = list(graph.subjects(RDF.type, GEOM_OP.PlanarAngleFromDirections))
    grad_ops = list(graph.subjects(RDF.type, GEOM_OP_EXT.AngleGradientFromDirections))
    assert len(angle_ops) == 1
    assert len(grad_ops) == 1

    # Operand-order guard: the scalar's from-directions is unordered, but the gradient op's
    # in1/in2 must carry the authored order (first-named plane's normal, then the second's).
    rotated_gripper_normal = graph.value(grad_ops[0], GEOM_OP.in1)
    table_normal = graph.value(grad_ops[0], GEOM_OP.in2)
    assert rotated_gripper_normal is not None
    assert table_normal is not None
    assert table_normal in set(graph.objects(angle_ops[0], GEOM_OP["from-directions"]))
    assert rotated_gripper_normal in set(graph.objects(angle_ops[0], GEOM_OP["from-directions"]))


def test_versor_alignment_is_unchanged(parse_source, base_source) -> None:
    graph = _graph(parse_source, base_source, WHILE_WITH_VERSOR, CTRL_WITH_VERSOR)

    rotate_ops = list(graph.subjects(RDF.type, GEOM_OP.RotateDirectionDistalToProximalWithPose))
    angle_ops = list(graph.subjects(RDF.type, GEOM_OP.PlanarAngleFromDirections))
    vector_ops = list(graph.subjects(RDF.type, GEOM_OP_EXT.RotationVectorFromDirections))
    assert len(rotate_ops) == 1
    assert len(angle_ops) == 1
    assert len(vector_ops) == 1
    assert list(graph.subjects(RDF.type, GEOM_OP_EXT.IncidentAngle)) == []
    assert list(graph.subjects(RDF.type, GEOM_OP_EXT.AngleGradientFromDirections)) == []


def test_plane_operand_needs_no_frame_axis(parse_source, base_source) -> None:
    diag_primitives = SPEC_ANCHOR + (
        ",\n        direction tool-up { as-seen-by: <gripper.g_base.g_pinch> } = (0, 0, -1)"
        ",\n        direction diag-normal "
        "{ as-seen-by: <kinova.base_link.base_link_origin> } = (0.7071, 0.7071, 0)"
        ",\n        plane diag-plane { of: <kinova.base_link.base_link_origin>, "
        "normal: <shared.spec.diag-normal> }"
        ",\n        angle align-band = 0.05 rad"
    )
    while_incident = WHILE_ANCHOR + (
        ",\n        tool-in-diag: keeping angle between <shared.spec.tool-up> and "
        "<shared.spec.diag-plane> equal to 0 rad within <shared.spec.align-band>"
    )
    accepted = base_source.replace(SPEC_ANCHOR, diag_primitives, 1).replace(
        WHILE_ANCHOR, while_incident, 1
    )
    parse_source(accepted)  # the plane's non-axis-aligned normal is fine for expression 2

    while_versor = WHILE_ANCHOR + (
        ",\n        tool-vs-diag: keeping angle between <shared.spec.tool-up> and "
        "<shared.spec.diag-normal> equal to 0 rad within <shared.spec.align-band>"
    )
    rejected = base_source.replace(SPEC_ANCHOR, diag_primitives, 1).replace(
        WHILE_ANCHOR, while_versor, 1
    )
    with pytest.raises(TextXSemanticError, match="signed unit frame axis"):
        parse_source(rejected)


def test_reversed_plane_versor_order_is_rejected(parse_source, base_source) -> None:
    while_reversed = WHILE_ANCHOR + (
        ",\n        table-vs-tool: keeping angle between <shared.spec.table> and "
        "<shared.spec.tool-up> equal to 0 rad within <shared.spec.align-band>"
    )
    source = base_source.replace(SPEC_ANCHOR, SPEC_WITH_PRIMITIVES, 1).replace(
        WHILE_ANCHOR, while_reversed, 1
    )
    with pytest.raises(TextXSemanticError, match="does not support"):
        parse_source(source)
