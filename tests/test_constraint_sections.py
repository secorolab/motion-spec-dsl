# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""An authored section logic reaches the graph as an expression node.

`until any` and `until all` are the same kind of statement, so they lower the same way: one
conjunction or disjunction the motion points at and a whole-section monitor targets. Without it
the authored `all` is erased, the monitor names the members one by one, and nothing downstream
can tell a section that means "all of these" from one that never said.
"""

from __future__ import annotations

from rdflib.namespace import RDF

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import CSTR_EXT, CSTR_HDL, MOT

SETTLED_Z = (
    "settled-z: <shared.world.twist-ee-base>.linvel.z equal to <shared.spec.zero-linvel> "
    "within <shared.spec.satisfied-band-vel>"
)
SETTLED_X = (
    "settled-x: <shared.world.twist-ee-base>.linvel.x equal to <shared.spec.zero-linvel> "
    "within <shared.spec.satisfied-band-vel>"
)


def _graph(parse_source, source: str):
    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


def _name(node) -> str:
    return str(node).rsplit("/", 1)[-1]


def _monitored(graph):
    """What the fixture's whole-section monitor names, by local name."""
    monitor = next(graph.subjects(RDF.type, CSTR_HDL.EdgeTriggeredMonitor))
    return sorted(_name(node) for node in graph.objects(monitor, CSTR_HDL.constraint))


def test_an_explicit_section_all_becomes_one_conjunction(parse_source, base_source) -> None:
    graph = _graph(
        parse_source, base_source.replace(SETTLED_Z, f"{SETTLED_Z},\n        {SETTLED_X}")
    )

    conjunctions = list(graph.subjects(RDF.type, CSTR_EXT.ConstraintConjunction))
    assert len(conjunctions) == 1
    node = conjunctions[0]
    assert (None, MOT.until, node) in graph
    assert {_name(m) for m in graph.objects(node, CSTR_EXT["has-constraint"])} == {
        "settled-z",
        "settled-x",
    }
    # The monitor targets the node, so the logic travels with what it watches.
    assert _monitored(graph) == [_name(node)]


def test_a_single_member_section_all_stays_flat(parse_source, base_source) -> None:
    """An `all` of one member states nothing beyond that member."""
    graph = _graph(parse_source, base_source)

    assert not list(graph.subjects(RDF.type, CSTR_EXT.ConstraintConjunction))
    assert _monitored(graph) == ["settled-z"]
