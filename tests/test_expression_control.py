# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A controlled expression is driven along its gradient, so the model has to state that gradient:
affine in the views a solver moves, coefficients that are literals, one frame for all of them.
A monitor only reads the value, and keeps the full algebra.
"""

from __future__ import annotations

import pytest
from rdflib.namespace import RDF
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import ALGO_EXT, CSTR, QUDT_SCHEMA

WORLD_ANCHOR = """        velocity-twist twist-ee-base {
            of:         <gripper.g_base.g_pinch>,
            wrt:        <kinova.base_link.base_link_origin>,
            as-seen-by: <kinova.base_link.base_link_origin>
        }"""
SPEC_ANCHOR = "linear-velocity zero-linvel = 0.0 m/s"
HOLD_ANCHOR = (
    "hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
)
UNTIL_ANCHOR = (
    "settled-z: <shared.world.twist-ee-base>.linvel.z equal to <shared.spec.zero-linvel> "
    "within <shared.spec.satisfied-band-vel>"
)

# A wrench seen by the same frame as the pose, so a product of the two is one gradient question
# and not also a frame question; and a pose seen by another frame, which is the frame question.
PRESS_WRENCH = """        wrench press-wrench {
            of:         <gripper.g_base.g_pinch>,
            ref-point:  <gripper.g_base.g_pinch>,
            as-seen-by: <kinova.base_link.base_link_origin>
        }"""
FOREARM_POSE = """        pose pose-ee-forearm {
            of:         <gripper.g_base.g_pinch>,
            wrt:        <kinova.forearm_link.forearm_link_origin>,
            as-seen-by: <kinova.forearm_link.forearm_link_origin>
        }"""
TORQUE_BAND = "torque tq-lo = -1.0 Nm,\n        torque tq-hi = 1.0 Nm"
LENGTH_BAND = "length len-lo = -1.0 m,\n        length len-hi = 1.0 m"
MEASURED_PRODUCT = "(<shared.world.press-wrench>.force.z * <shared.world.pose-ee-base>.position.x)"
TWO_FRAME_SUM = (
    "(<shared.world.pose-ee-base>.position.x + <shared.world.pose-ee-forearm>.position.y)"
)


def _source(base_source: str, *, world: str = "", spec: str = "", hold: str = "", until: str = ""):
    source = base_source
    if world:
        source = source.replace(WORLD_ANCHOR, f"{WORLD_ANCHOR},\n{world}", 1)
    if spec:
        source = source.replace(SPEC_ANCHOR, f"{SPEC_ANCHOR},\n        {spec}", 1)
    if hold:
        source = source.replace(HOLD_ANCHOR, hold, 1)
    if until:
        source = source.replace(UNTIL_ANCHOR, f"{UNTIL_ANCHOR},\n        {until}", 1)
    return source


def test_a_product_of_two_measured_views_cannot_be_controlled(parse_source, base_source) -> None:
    with pytest.raises(TextXSemanticError, match="multiplies two measured views"):
        parse_source(
            _source(
                base_source,
                world=PRESS_WRENCH,
                spec=TORQUE_BAND,
                hold=f"hold-position: keeping {MEASURED_PRODUCT} outside <spec.tq-lo> and <spec.tq-hi>",
            )
        )


def test_the_same_product_is_accepted_where_only_a_monitor_reads_it(
    parse_source, base_source
) -> None:
    model = parse_source(
        _source(
            base_source,
            world=PRESS_WRENCH,
            spec=TORQUE_BAND,
            until=f"torque-load: {MEASURED_PRODUCT} outside <spec.tq-lo> and <spec.tq-hi>",
        )
    )
    assert model is not None


def test_a_controlled_expression_states_one_frame_for_every_measured_view(
    parse_source, base_source
) -> None:
    with pytest.raises(TextXSemanticError, match="different frames"):
        parse_source(
            _source(
                base_source,
                world=FOREARM_POSE,
                spec=LENGTH_BAND,
                hold=f"hold-position: keeping {TWO_FRAME_SUM} outside <spec.len-lo> and <spec.len-hi>",
            )
        )


def test_a_measured_view_may_not_be_a_divisor_of_a_controlled_expression(
    parse_source, base_source
) -> None:
    with pytest.raises(TextXSemanticError, match="divides by a view it measures"):
        parse_source(
            _source(
                base_source,
                spec="duration dur-lo = -1.0 s,\n        duration dur-hi = 1.0 s",
                hold=(
                    "hold-position: keeping (<shared.world.pose-ee-base>.position.x / "
                    "<shared.world.twist-ee-base>.linvel.x) outside <spec.dur-lo> and <spec.dur-hi>"
                ),
            )
        )


def test_a_runtime_coefficient_cannot_scale_a_controlled_measured_view(
    parse_source, base_source
) -> None:
    with pytest.raises(TextXSemanticError, match="coefficients it can state"):
        parse_source(
            _source(
                base_source,
                spec="dimensionless gain = 2.0 1,\n        " + LENGTH_BAND,
                hold=(
                    "hold-position: keeping (<spec.gain> * "
                    "<shared.world.pose-ee-base>.position.x) outside <spec.len-lo> and "
                    "<spec.len-hi>"
                ),
            )
        )


def test_a_controlled_expression_emits_only_individuals_of_existing_terms(
    parse_source, base_source
) -> None:
    """The op chain and its coefficient are ordinary ALGO operations and qudt quantities; nothing
    a controlled expression emits is a term of its own.
    """
    model = parse_source(
        _source(
            base_source,
            spec=LENGTH_BAND,
            hold=(
                "hold-position: keeping (<shared.world.pose-ee-base>.position.x * 2.0 1) "
                "outside <spec.len-lo> and <spec.len-hi>"
            ),
        )
    )
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph
    (constraint,) = [
        node for node in graph.subjects(RDF.type, CSTR.Constraint) if "hold-position" in node
    ]
    (root,) = list(graph.objects(constraint, CSTR.quantity))
    (operation,) = list(graph.subjects(ALGO_EXT.out, root))
    assert ALGO_EXT.Multiplication in set(graph.objects(operation, RDF.type))
    (coefficient,) = [
        operand
        for operand in graph.objects(operation, ALGO_EXT["in"])
        if graph.value(operand, QUDT_SCHEMA.value) is not None
    ]
    assert set(graph.objects(coefficient, RDF.type)) == {QUDT_SCHEMA.Quantity}
    assert float(graph.value(coefficient, QUDT_SCHEMA.value)) == 2.0
