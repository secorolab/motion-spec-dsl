# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""SHACL shapes for the temporal layer: instants, intervals, durations, tolerances."""

from __future__ import annotations

from pathlib import Path

import pyshacl
from rdflib import Graph

WORKSPACE = Path(__file__).resolve().parents[3]
METAMODELS = WORKSPACE / "src" / "metamodels"

PREFIXES = """
@prefix app: <https://example.org/app#> .
@prefix cstr: <https://comp-rob2b.github.io/metamodels/task/constraint#> .
@prefix cstr-ext: <https://secorolab.github.io/metamodels/task/constraint#> .
@prefix qudt: <http://qudt.org/schema/qudt/> .
@prefix qkind: <http://qudt.org/vocab/quantitykind/> .
@prefix unit: <http://qudt.org/vocab/unit/> .
@prefix time: <http://www.w3.org/2006/time#> .
@prefix xsd: <http://www.w3.org/2001/XMLSchema#> .
"""

# An `elapsed greater than 5 s` guard: the interval from motion entry to now, its
# duration coordinate, and the constraint over it.
ELAPSED = """
app:clock a time:TRS .

app:entry a time:Instant ;
    time:inTimePosition app:entry-position .
app:entry-position a time:TimePosition ;
    time:hasTRS app:clock .
app:now a time:Instant .

app:elapsed a time:ProperInterval ;
    time:hasBeginning app:entry ;
    time:hasEnd app:now ;
    time:hasDuration app:elapsed-duration .

app:elapsed-duration a time:Duration, qudt:Quantity ;
    qudt:hasQuantityKind qkind:Time ;
    qudt:unit unit:SEC .

app:wait5s a cstr-ext:TimeConstraint, cstr:UnilateralConstraint, cstr:GreaterThanConstraint ;
    cstr:quantity app:elapsed-duration ;
    cstr:threshold app:wait5s-threshold .

app:wait5s-threshold a time:Duration, qudt:Quantity ;
    qudt:hasQuantityKind qkind:Time ;
    qudt:unit unit:SEC ;
    qudt:value "5.0"^^xsd:double .
"""


def _shapes() -> Graph:
    g = Graph()
    for name in ("time.shacl.ttl", "task/constraint-extension.shacl.ttl"):
        g.parse(METAMODELS / name, format="turtle")
    return g


def _conforms(data: str) -> bool:
    g = Graph().parse(data=PREFIXES + data, format="turtle")
    conforms, _, _ = pyshacl.validate(g, shacl_graph=_shapes(), advanced=True)
    return conforms


def test_elapsed_guard_conforms() -> None:
    assert _conforms(ELAPSED)


def test_interval_needs_both_endpoints() -> None:
    assert not _conforms(ELAPSED.replace("    time:hasEnd app:now ;\n", ""))


def test_duration_is_not_read_against_a_clock() -> None:
    assert not _conforms(ELAPSED + "app:elapsed-duration time:hasTRS app:clock .")


def test_constraint_quantity_must_be_a_duration() -> None:
    assert not _conforms(ELAPSED.replace("app:elapsed-duration a time:Duration,", "app:elapsed-duration a"))


def _equality(tolerance: str = "", kind: str = "qkind:Time", unit: str = "unit:SEC") -> str:
    """The elapsed guard rewritten as `elapsed equals 5 s`, optionally with a tolerance."""
    equality = ELAPSED.replace(
        "cstr:UnilateralConstraint, cstr:GreaterThanConstraint", "cstr:EqualityConstraint"
    ).replace("cstr:threshold app:wait5s-threshold", "cstr:reference-value app:wait5s-threshold")
    if not tolerance:
        return equality
    return equality + f"""
app:wait5s cstr-ext:tolerance app:wait5s-tolerance .
app:wait5s-tolerance a time:Duration, qudt:Quantity ;
    qudt:hasQuantityKind {kind} ;
    qudt:unit {unit} ;
    qudt:value "{tolerance}"^^xsd:double .
"""


def test_time_equality_requires_a_tolerance() -> None:
    assert not _conforms(_equality())
    assert _conforms(_equality("0.1"))


def test_tolerance_cannot_be_negative() -> None:
    assert not _conforms(_equality("-0.1"))


def test_only_an_equality_takes_a_tolerance() -> None:
    assert not _conforms(
        ELAPSED + """
app:wait5s cstr-ext:tolerance app:wait5s-tolerance .
app:wait5s-tolerance a time:Duration, qudt:Quantity ;
    qudt:hasQuantityKind qkind:Time ;
    qudt:unit unit:SEC ;
    qudt:value "0.1"^^xsd:double .
"""
    )


def test_tolerance_must_share_the_kind_of_what_it_tolerances() -> None:
    # `elapsed equals 5 s +/- 0.1 N` -- right shape, wrong dimension.
    assert not _conforms(_equality("0.1", kind="qkind:Force", unit="unit:N"))


def test_tolerance_needs_a_unit() -> None:
    bare = _equality("0.1").replace("    qudt:unit unit:SEC ;\n", "")
    assert not _conforms(bare)
