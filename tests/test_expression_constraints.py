# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A constraint's view can name a scalar context quantity directly, or an inline parenthesized
expression the constraint owns; both flow into the same relation/tolerance emission a world
quantity's view does.
"""

from __future__ import annotations

import pytest
from rdflib.namespace import RDF

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import ALGO_EXT, CSTR, CSTR_EXT

SPEC_ANCHOR = "linear-velocity zero-linvel = 0.0 m/s"
WHILE_ANCHOR = (
    "hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
)

FORCE_SPEC = (
    "force f = 10.0 N,\n"
    "        mass k = 1.2 kg,\n"
    "        force lo = -5.0 N,\n"
    "        force hi = 5.0 N,\n"
    "        force residual = <spec.f> - <spec.k> * 1.0 m/s^2"
)


def _build(parse_source, base_source: str, while_addition: str, spec_addition: str = FORCE_SPEC):
    source = base_source.replace(SPEC_ANCHOR, f"{SPEC_ANCHOR},\n        {spec_addition}")
    source = source.replace(WHILE_ANCHOR, f"{WHILE_ANCHOR},\n        {while_addition}")
    model = parse_source(source)
    return MotionSpecDatasetBuilder(model).build()[0].default_graph


def test_a_named_context_quantity_view_uses_its_own_quantity_node(
    parse_source, base_source
) -> None:
    graph = _build(
        parse_source,
        base_source,
        "residual-band: keeping <spec.residual> outside <spec.lo> and <spec.hi>",
    )
    (node,) = [
        s for s in graph.subjects(RDF.type, CSTR_EXT.OutsideConstraint) if "residual-band" in s
    ]
    (qty,) = list(graph.objects(node, CSTR.quantity))
    assert qty.endswith("residual")


def test_an_inline_expression_view_mints_a_gensym_root_and_op_chain(
    parse_source, base_source
) -> None:
    graph = _build(
        parse_source,
        base_source,
        "residual-band: keeping (<spec.f> - <spec.k> * 1.0 m/s^2) outside <spec.lo> and <spec.hi>",
    )
    (node,) = [
        s for s in graph.subjects(RDF.type, CSTR_EXT.OutsideConstraint) if "residual-band" in s
    ]
    (qty,) = list(graph.objects(node, CSTR.quantity))
    assert str(qty).endswith("/expr")
    assert list(graph.subjects(ALGO_EXT.out, qty))


def test_a_whole_pose_context_quantity_view_is_rejected_clearly(parse_source, base_source) -> None:
    """An order relation on a whole pose is already rejected at validation (pre-existing);
    an equality relation reaches RDF emission, where plan 05's own guard rejects it."""
    addition = "pose some-pose = snapshot of <shared.world.pose-ee-base>"
    with pytest.raises(ValueError, match="whole pose"):
        _build(
            parse_source,
            base_source,
            "pose-band: keeping <spec.some-pose> equal to <spec.home-pose> "
            "within <shared.spec.satisfied-band>",
            spec_addition=addition,
        )


def test_a_mass_expression_view_has_no_constraint_type(parse_source, base_source) -> None:
    with pytest.raises(ValueError, match="no constraint type for kind Mass"):
        _build(
            parse_source,
            base_source,
            "mass-band: keeping <spec.k> outside <spec.lo> and <spec.hi>",
        )


def test_a_controller_may_attach_to_an_inline_expression_constraint(
    parse_source, base_source
) -> None:
    """No restriction at DSL validation -- RDF-emission-level solver/controller derivation for
    a controlled expression constraint is plan 05 phase E2's job, not validated here."""
    source = (
        base_source.replace(SPEC_ANCHOR, f"{SPEC_ANCHOR},\n        {FORCE_SPEC}")
        .replace(
            WHILE_ANCHOR,
            f"{WHILE_ANCHOR},\n        "
            "residual-hold: keeping (<spec.f> - <spec.k> * 1.0 m/s^2) equal to 0.0 N "
            "within <shared.spec.satisfied-band>",
        )
        .replace(
            "pid ctrl-hold-position { constraint: <home.hold-position>, Kp: 200, Ki: 100, Kd: 40, decay: 0 }",
            "pid ctrl-hold-position { constraint: <home.hold-position>, Kp: 200, Ki: 100, Kd: 40, decay: 0 },\n"
            "        pid ctrl-residual { constraint: <home.residual-hold>, Kp: 1, Ki: 0, Kd: 0, decay: 0 }",
        )
    )
    parse_source(source)
