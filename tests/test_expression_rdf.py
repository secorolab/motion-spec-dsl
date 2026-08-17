# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""RDF emission of quantity-expression trees: mixed operators lower to nested ALGO ops, and
generated intermediate nodes carry inferred kind/unit and regenerate identically.
"""

from __future__ import annotations

from rdflib.namespace import RDF

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import ALGO_EXT, QUDT_QKIND, QUDT_SCHEMA
from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT

SPEC_ANCHOR = "linear-velocity zero-linvel = 0.0 m/s"

EXPR = (
    "force f = 10.0 N,\n"
    "        mass k = 1.2 kg,\n"
    "        linear-acceleration a = 9.81 m/s^2,\n"
    "        force residual = <spec.f> - <spec.k> * <spec.a>"
)


def _build(parse_mutated, base_source: str):
    model = parse_mutated(SPEC_ANCHOR, f"{SPEC_ANCHOR},\n        {EXPR}")
    return MotionSpecDatasetBuilder(model).build()[0].default_graph


def test_mixed_operators_nest_a_multiplication_inside_a_subtraction(
    parse_mutated, base_source
) -> None:
    graph = _build(parse_mutated, base_source)

    (sub_node,) = list(graph.subjects(RDF.type, ALGO_EXT.Subtraction))
    (mul_node,) = list(graph.subjects(RDF.type, ALGO_EXT.Multiplication))
    (subtrahend,) = list(graph.objects(sub_node, ALGO_EXT.subtrahend))
    (mul_out,) = list(graph.objects(mul_node, ALGO_EXT.out))
    assert subtrahend == mul_out


def test_the_intermediate_result_carries_the_inferred_kind_and_unit(
    parse_mutated, base_source
) -> None:
    graph = _build(parse_mutated, base_source)

    (mul_node,) = list(graph.subjects(RDF.type, ALGO_EXT.Multiplication))
    (mul_out,) = list(graph.objects(mul_node, ALGO_EXT.out))
    assert (mul_out, RDF.type, QUDT_SCHEMA.Quantity) in graph
    assert graph.value(mul_out, QUDT_SCHEMA.hasQuantityKind) == QUDT_QKIND.Force
    assert graph.value(mul_out, QUDT_SCHEMA.unit) == QUDT_UNIT.N


def test_generated_node_names_are_stable_across_two_builds(parse_mutated, base_source) -> None:
    model = parse_mutated(SPEC_ANCHOR, f"{SPEC_ANCHOR},\n        {EXPR}")
    first = MotionSpecDatasetBuilder(model).build()[0].default_graph
    second = MotionSpecDatasetBuilder(model).build()[0].default_graph
    assert set(first.subjects()) == set(second.subjects())
