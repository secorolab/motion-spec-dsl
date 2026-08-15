# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""An offset states which way it applies.

`+` collects its operands under one predicate, because an addition reads the same either way
round. `-` does not, so the operands are named: what the declaration samples is the minuend, and
what the author wrote the sign in front of is taken from it.
"""

from __future__ import annotations

from rdflib.namespace import RDF

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import ALGO_EXT

SAMPLE = "pose home-pose = snapshot of <shared.world.pose-ee-base>"


def _graph(parse_source, base_source: str, sign: str):
    """The base model, with a height sampled `sign` an offset beside its own declarations."""
    declarations = (
        f"{SAMPLE},\n"
        "            length lift = 0.05 m,\n"
        "            length support-z = snapshot of <shared.world.pose-ee-base>.position.z "
        f"{sign} <spec.lift>"
    )
    model = parse_source(base_source.replace(SAMPLE, declarations))
    return MotionSpecDatasetBuilder(model).build()[0].default_graph


def test_a_plus_offset_adds_both_operands_under_one_predicate(parse_source, base_source) -> None:
    graph = _graph(parse_source, base_source, "+")

    (node,) = list(graph.subjects(RDF.type, ALGO_EXT.Addition))
    assert len(list(graph.objects(node, ALGO_EXT["in"]))) == 2
    assert not list(graph.subjects(RDF.type, ALGO_EXT.Subtraction))


def test_a_minus_offset_names_the_operand_it_takes_away(parse_source, base_source) -> None:
    graph = _graph(parse_source, base_source, "-")

    (node,) = list(graph.subjects(RDF.type, ALGO_EXT.Subtraction))
    assert not list(graph.subjects(RDF.type, ALGO_EXT.Addition))
    # The sample is what the offset comes off, whichever order the graph is read in.
    (minuend,) = list(graph.objects(node, ALGO_EXT.minuend))
    (subtrahend,) = list(graph.objects(node, ALGO_EXT.subtrahend))
    assert str(subtrahend).endswith("/lift")
    assert "position.z" in str(minuend) or str(minuend).endswith("z")
    assert len(list(graph.objects(node, ALGO_EXT.out))) == 1
