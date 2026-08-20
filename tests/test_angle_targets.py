# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Nonzero angle targets and angle inequalities (plan 10): a point target on the sphere still
drives the exact rotation-vector row; a cone target -- `equal to <nonzero>`, `greater than`, a
band not opening at zero, `outside`, or `less than` written as a bound on the same cone as the
point-target band -- swaps in the runtime gradient row, sharing the same theta scalar.
"""

from __future__ import annotations

import pytest
from rdflib.namespace import RDF
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import GEOM_OP, GEOM_OP_EXT

SPEC_ANCHOR = "        linear-velocity zero-linvel = 0.0 m/s"
SPEC_WITH_ALIGN = SPEC_ANCHOR + (
    ",\n        direction tool-up { as-seen-by: <gripper.g_base.g_pinch> } = (0, 0, -1)"
    ",\n        direction base-up { as-seen-by: <kinova.base_link.base_link_origin> } = (0, 0, 1)"
    ",\n        angle align-band = 0.05 rad"
)
SPEC_WITH_NON_AXIS = SPEC_ANCHOR + (
    ",\n        direction tool-up { as-seen-by: <gripper.g_base.g_pinch> } = (0, 0, -1)"
    ",\n        direction diag-up "
    "{ as-seen-by: <kinova.base_link.base_link_origin> } = (0.7071, 0.7071, 0)"
    ",\n        angle align-band = 0.05 rad"
)
SPEC_WITH_PLANES = SPEC_ANCHOR + (
    ",\n        direction table-normal { as-seen-by: <kinova.base_link.base_link_origin> } = (0, 0, 1)"
    ",\n        direction wall-normal { as-seen-by: <gripper.g_base.g_pinch> } = (1, 0, 0)"
    ",\n        plane table { of: <kinova.base_link.base_link_origin>, "
    "normal: <shared.spec.table-normal> }"
    ",\n        plane wall { of: <gripper.g_base.g_pinch>, "
    "normal: <shared.spec.wall-normal> }"
    ",\n        angle align-band = 0.05 rad"
)

WHILE_ANCHOR = (
    "        hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
)
CTRL_ANCHOR = (
    "        pid ctrl-hold-position { constraint: <home.hold-position>, "
    "Kp: 200, Ki: 100, Kd: 40, decay: 0 }"
)


def _source(base_source: str, spec_with: str, while_with: str, ctrl_with: str) -> str:
    source = base_source.replace(SPEC_ANCHOR, spec_with, 1)
    source = source.replace(WHILE_ANCHOR, while_with, 1)
    return source.replace(CTRL_ANCHOR, ctrl_with, 1)


def _graph(parse_source, base_source: str, spec_with: str, while_with: str, ctrl_with: str):
    source = _source(base_source, spec_with, while_with, ctrl_with)
    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


def _while_ctrl(
    name: str, relation: str, operands: str = "tool-up> and <shared.spec.base-up"
) -> tuple[str, str]:
    while_with = WHILE_ANCHOR + (
        f",\n        {name}: keeping angle between <shared.spec.{operands}> {relation}"
    )
    ctrl_with = CTRL_ANCHOR + (
        f",\n        pid ctrl-{name} {{ constraint: <home.{name}>, "
        "Kp: 120, Ki: 50, Kd: 80, decay: 0 }"
    )
    return while_with, ctrl_with


def test_zero_target_still_emits_the_rotation_vector_op(parse_source, base_source) -> None:
    while_with, ctrl_with = _while_ctrl(
        "align-tool-z", "equal to 0 rad within <shared.spec.align-band>"
    )
    graph = _graph(parse_source, base_source, SPEC_WITH_ALIGN, while_with, ctrl_with)

    assert len(list(graph.subjects(RDF.type, GEOM_OP_EXT.RotationVectorFromDirections))) == 1
    assert list(graph.subjects(RDF.type, GEOM_OP_EXT.AngleGradientFromDirections)) == []


def test_nonzero_target_emits_the_gradient_op(parse_source, base_source) -> None:
    while_with, ctrl_with = _while_ctrl(
        "align-tilt", "equal to pi/6 rad within <shared.spec.align-band>"
    )
    graph = _graph(parse_source, base_source, SPEC_WITH_ALIGN, while_with, ctrl_with)

    assert list(graph.subjects(RDF.type, GEOM_OP_EXT.RotationVectorFromDirections)) == []
    assert len(list(graph.subjects(RDF.type, GEOM_OP_EXT.AngleGradientFromDirections))) == 1
    # The shared ops still fire: no second theta is minted for the cone path.
    assert len(list(graph.subjects(RDF.type, GEOM_OP.RotateDirectionDistalToProximalWithPose))) == 1
    assert len(list(graph.subjects(RDF.type, GEOM_OP.PlanarAngleFromDirections))) == 1


def test_less_than_stays_pointwise(parse_source, base_source) -> None:
    """The subtle case: `less than <cone>` is `between 0 and <cone>` written as a bound, so it
    must keep the 2-DOF rotation-vector row. Routing it to the gradient row would silently free
    a rotational DOF the author never asked for.
    """
    while_with, ctrl_with = _while_ctrl("keep-clear", "less than 0.1 rad")
    graph = _graph(parse_source, base_source, SPEC_WITH_ALIGN, while_with, ctrl_with)

    assert len(list(graph.subjects(RDF.type, GEOM_OP_EXT.RotationVectorFromDirections))) == 1
    assert list(graph.subjects(RDF.type, GEOM_OP_EXT.AngleGradientFromDirections)) == []


def test_greater_than_emits_the_gradient_op(parse_source, base_source) -> None:
    while_with, ctrl_with = _while_ctrl("stay-clear", "greater than 0.3491 rad")
    graph = _graph(parse_source, base_source, SPEC_WITH_ALIGN, while_with, ctrl_with)

    assert list(graph.subjects(RDF.type, GEOM_OP_EXT.RotationVectorFromDirections)) == []
    assert len(list(graph.subjects(RDF.type, GEOM_OP_EXT.AngleGradientFromDirections))) == 1


def test_band_from_zero_stays_pointwise(parse_source, base_source) -> None:
    while_with, ctrl_with = _while_ctrl("band-from-zero", "between 0 rad and 0.1 rad")
    graph = _graph(parse_source, base_source, SPEC_WITH_ALIGN, while_with, ctrl_with)

    assert len(list(graph.subjects(RDF.type, GEOM_OP_EXT.RotationVectorFromDirections))) == 1
    assert list(graph.subjects(RDF.type, GEOM_OP_EXT.AngleGradientFromDirections)) == []


def test_band_above_zero_emits_the_gradient_op(parse_source, base_source) -> None:
    while_with, ctrl_with = _while_ctrl("band-above-zero", "between 0.2 rad and 0.4 rad")
    graph = _graph(parse_source, base_source, SPEC_WITH_ALIGN, while_with, ctrl_with)

    assert list(graph.subjects(RDF.type, GEOM_OP_EXT.RotationVectorFromDirections)) == []
    assert len(list(graph.subjects(RDF.type, GEOM_OP_EXT.AngleGradientFromDirections))) == 1


def test_target_above_pi_is_rejected(parse_source, base_source) -> None:
    while_with, ctrl_with = _while_ctrl("too-big", "equal to 4 rad")
    source = _source(base_source, SPEC_WITH_ALIGN, while_with, ctrl_with)

    with pytest.raises(TextXSemanticError, match="the angle between two directions lies in"):
        parse_source(source)


def test_nonzero_target_accepts_a_non_axis_reference(parse_source, base_source) -> None:
    """Plan 09 Sec.6 test 4 extended to versor-versor: the frame-axis rule only exists for the
    2-axis rotation-vector row, so it relaxes exactly where that row is no longer used.
    """
    while_nonzero, ctrl_with = _while_ctrl(
        "tool-vs-diag",
        "equal to pi/6 rad within <shared.spec.align-band>",
        operands="tool-up> and <shared.spec.diag-up",
    )
    accepted = _source(base_source, SPEC_WITH_NON_AXIS, while_nonzero, ctrl_with)
    parse_source(accepted)  # builds fine at a nonzero target

    while_zero, _ = _while_ctrl(
        "tool-vs-diag",
        "equal to 0 rad within <shared.spec.align-band>",
        operands="tool-up> and <shared.spec.diag-up",
    )
    rejected = _source(base_source, SPEC_WITH_NON_AXIS, while_zero, ctrl_with)
    with pytest.raises(TextXSemanticError, match="signed unit frame axis"):
        parse_source(rejected)


def test_nonzero_dihedral_target_is_accepted(parse_source, base_source) -> None:
    """Plan 09 shipped `angle between <plane> and <plane>` only at a zero target; this plan is
    what makes a nonzero dihedral authorable. Its row was already always the gradient row.
    """
    while_with, ctrl_with = _while_ctrl(
        "surfaces-30",
        "equal to pi/6 rad within <shared.spec.align-band>",
        operands="wall> and <shared.spec.table",
    )
    graph = _graph(parse_source, base_source, SPEC_WITH_PLANES, while_with, ctrl_with)

    assert len(list(graph.subjects(RDF.type, GEOM_OP_EXT.AngleGradientFromDirections))) == 1
    assert len(list(graph.subjects(RDF.type, GEOM_OP.PlanarAngleFromDirections))) == 1


SECOND_MOTION = (
    "\n\nguarded-motion (ns=app) home2 {\n"
    "    context {}\n"
    "    when {}\n"
    "    while {\n"
    "        align2: keeping angle between <shared.spec.tool-up> and "
    "<shared.spec.base-up> equal to pi/4 rad within <shared.spec.align-band>\n"
    "    }\n"
    "    until {}\n"
    "}\n\n"
    "constraint-handler (ns=app) handler-home2 {\n"
    "    handles: <home2>\n"
    "    controllers {\n"
    "        pid ctrl-align2 { constraint: <home2.align2>, Kp: 120, Ki: 50, Kd: 80, decay: 0 }\n"
    "    }\n"
    "    solvers {\n"
    "        arm-solver2: serial-chain {\n"
    "            agent: <agents.kinova_ft_2f85>,\n"
    "            algorithm: achd,\n"
    "            gravity: (0.0, 0.0, 9.81) m/s^2\n"
    "        }\n"
    "    }\n"
    "}\n"
)


def test_two_motions_with_different_targets_get_distinct_ops(parse_source, base_source) -> None:
    while_with, ctrl_with = _while_ctrl(
        "align1", "equal to pi/6 rad within <shared.spec.align-band>"
    )
    source = _source(base_source, SPEC_WITH_ALIGN, while_with, ctrl_with) + SECOND_MOTION
    graph = MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph

    grad_ops = list(graph.subjects(RDF.type, GEOM_OP_EXT.AngleGradientFromDirections))
    assert len(grad_ops) == 2
    angle_ops = list(graph.subjects(RDF.type, GEOM_OP.PlanarAngleFromDirections))
    assert len(angle_ops) == 2
    thetas = {graph.value(op, GEOM_OP.angle) for op in angle_ops}
    assert len(thetas) == 2


def _second_motion(relation: str) -> str:
    return SECOND_MOTION.replace("equal to pi/4 rad within <shared.spec.align-band>", relation, 1)


def test_a_bound_and_a_cone_at_one_value_do_not_share_a_chain(parse_source, base_source) -> None:
    """`less than 0.5` is pointwise (2-DOF) and `equal to 0.5` is a cone (1-DOF), but they agree
    on every token the scalar id folds. Only the row shape separates them, so the id has to say
    so -- the emission loop dedupes on it, and the loser silently gets the winner's chain.
    """
    while_with, ctrl_with = _while_ctrl("align1", "less than 0.5 rad")
    source = _source(base_source, SPEC_WITH_ALIGN, while_with, ctrl_with) + _second_motion(
        "equal to 0.5 rad within <shared.spec.align-band>"
    )
    graph = MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph

    assert len(list(graph.subjects(RDF.type, GEOM_OP_EXT.RotationVectorFromDirections))) == 1
    assert len(list(graph.subjects(RDF.type, GEOM_OP_EXT.AngleGradientFromDirections))) == 1
