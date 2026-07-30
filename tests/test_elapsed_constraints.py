# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Elapsed (timing) constraints: grammar, validation, and native OWL-Time RDF emission."""

from __future__ import annotations

import pytest
from rdflib.namespace import RDF

from motion_spec.rdf_parser.vocab import CSTR, CSTR_EXT, TIME
from motion_spec_dsl.rdf.builder import MotionSpecDatasetBuilder
from motion_spec_dsl.registration import _build_manifest

# hold-position is the fixture's only WHILE constraint; a WHILE section just needs the
# handler to have some controller/monitor overall (already true), not one per constraint,
# so appending an elapsed constraint here needs no extra monitor/event wiring.
WHILE_ANCHOR = (
    "hold-position: keeping <shared.world.pose-ee-base>.position equal to <spec.home-pose>.position"
)


def _while(extra: str) -> str:
    return f"{WHILE_ANCHOR},\n        {extra}"


def _build(parse_mutated, body: str):
    model = parse_mutated(WHILE_ANCHOR, _while(body))
    dataset, _ = MotionSpecDatasetBuilder(model).build()
    return dataset.default_graph


def test_elapsed_greater_than_emits_native_time_constraint(parse_mutated):
    g = _build(parse_mutated, "wait5: elapsed greater than 5.0 s")

    cstr = next(g.subjects(RDF.type, CSTR_EXT.TimeConstraint))
    assert (cstr, RDF.type, CSTR.GreaterThanConstraint) in g

    measured = g.value(cstr, CSTR.quantity)
    assert (measured, RDF.type, TIME.Duration) in g
    assert g.value(measured, TIME.numericDuration) is None  # runtime state, unmeasured

    interval = next(g.subjects(TIME.hasDuration, measured))
    assert (interval, RDF.type, TIME.ProperInterval) in g
    beginning = g.value(interval, TIME.hasBeginning)
    end = g.value(interval, TIME.hasEnd)
    assert (beginning, RDF.type, TIME.Instant) in g
    assert (end, RDF.type, TIME.Instant) in g

    threshold = g.value(cstr, CSTR.threshold)
    assert float(g.value(threshold, TIME.numericDuration)) == pytest.approx(5.0)
    assert g.value(threshold, TIME.unitType) == TIME.unitSecond


def test_elapsed_threshold_normalizes_milliseconds_to_seconds(parse_mutated):
    g = _build(parse_mutated, "wait5: elapsed less than 10.0 ms")
    cstr = next(g.subjects(RDF.type, CSTR_EXT.TimeConstraint))
    threshold = g.value(cstr, CSTR.threshold)
    assert float(g.value(threshold, TIME.numericDuration)) == pytest.approx(0.01)


def test_elapsed_equality_emits_reference_and_tolerance(parse_mutated):
    g = _build(parse_mutated, "wait5: elapsed equal to 5.0 s within 10.0 ms")
    cstr = next(g.subjects(RDF.type, CSTR_EXT.TimeConstraint))
    assert (cstr, RDF.type, CSTR.EqualityConstraint) in g

    reference = g.value(cstr, CSTR["reference-value"])
    assert float(g.value(reference, TIME.numericDuration)) == pytest.approx(5.0)

    tolerance = g.value(cstr, CSTR_EXT.tolerance)
    assert float(g.value(tolerance, TIME.numericDuration)) == pytest.approx(0.01)


def test_manifest_includes_time_shacl_exactly_once():
    constraints = _build_manifest([])["@graph"][0]["constraints"]
    matches = [c for c in constraints if c.endswith("/time.shacl.ttl")]
    assert matches == ["https://secorolab.github.io/metamodels/time.shacl.ttl"]
