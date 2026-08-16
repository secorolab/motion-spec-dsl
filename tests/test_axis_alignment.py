# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""`angle between` derives its three ops and the theta/vector quantities they wire."""

from __future__ import annotations

from rdflib.namespace import RDF

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import GEOM_OP, GEOM_OP_EXT, QUDT_SCHEMA

SPEC_ANCHOR = "        linear-velocity zero-linvel = 0.0 m/s"
SPEC_WITH_ALIGN = SPEC_ANCHOR + (
    ",\n        direction tool-up { as-seen-by: <gripper.g_base.g_pinch> } = (0, 0, -1)"
    ",\n        direction base-up { as-seen-by: <kinova.base_link.base_link_origin> } = (0, 0, 1)"
    ",\n        angle align-band = 0.05 rad"
)
WHILE_ANCHOR = (
    "        hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
)
WHILE_WITH_ALIGN = WHILE_ANCHOR + (
    ",\n        align-tool-z: keeping angle between <shared.spec.tool-up> and "
    "<shared.spec.base-up> equal to 0 rad within <shared.spec.align-band>"
)
CTRL_ANCHOR = (
    "        pid ctrl-hold-position { constraint: <home.hold-position>, "
    "Kp: 200, Ki: 100, Kd: 40, decay: 0 }"
)
CTRL_WITH_ALIGN = CTRL_ANCHOR + (
    ",\n        pid ctrl-align-tool-z { constraint: <home.align-tool-z>, "
    "Kp: 120, Ki: 50, Kd: 80, decay: 0 }"
)


def _graph(parse_source, base_source):
    source = base_source.replace(SPEC_ANCHOR, SPEC_WITH_ALIGN, 1)
    source = source.replace(WHILE_ANCHOR, WHILE_WITH_ALIGN, 1)
    source = source.replace(CTRL_ANCHOR, CTRL_WITH_ALIGN, 1)
    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


def test_angle_between_emits_the_three_alignment_ops(parse_source, base_source) -> None:
    graph = _graph(parse_source, base_source)

    rotate_ops = list(graph.subjects(RDF.type, GEOM_OP.RotateDirectionDistalToProximalWithPose))
    angle_ops = list(graph.subjects(RDF.type, GEOM_OP.PlanarAngleFromDirections))
    vector_ops = list(graph.subjects(RDF.type, GEOM_OP_EXT.RotationVectorFromDirections))
    assert len(rotate_ops) == 1
    assert len(angle_ops) == 1
    assert len(vector_ops) == 1

    theta = graph.value(angle_ops[0], GEOM_OP.angle)
    assert theta is not None
    assert "RAD" in str(graph.value(theta, QUDT_SCHEMA.unit))

    error_vector = graph.value(vector_ops[0], GEOM_OP.out)
    assert error_vector is not None
    assert "RAD" in str(graph.value(error_vector, QUDT_SCHEMA.unit))

    rotated = graph.value(rotate_ops[0], GEOM_OP.to)
    assert rotated in set(graph.objects(angle_ops[0], GEOM_OP["from-directions"]))
    assert rotated == graph.value(vector_ops[0], GEOM_OP.in1)
