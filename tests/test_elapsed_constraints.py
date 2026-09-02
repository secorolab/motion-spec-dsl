# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Elapsed (timing) constraints: grammar, validation, and native OWL-Time RDF emission.

A duration keeps the unit it was written in; owl-time cannot say `10 ms`, so the magnitude
rides on qudt and only a reader turns it into seconds.
"""

from __future__ import annotations

import pytest
from rdflib.namespace import RDF

from motion_spec_dsl.rdf_parser.vocab import CSTR, CSTR_EXT, QUDT_SCHEMA, SOSA, TIME
from rdf_utils.namespace import NS_MM_QUDT_QTY as QKIND, NS_MM_QUDT_UNIT as QUDT_UNIT
from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.gens import _build_manifest

# hold-position is the fixture's only WHILE constraint; a WHILE section just needs the
# handler to have some controller/monitor overall (already true), not one per constraint,
# so appending an elapsed constraint here needs no extra monitor/event wiring.
WHILE_ANCHOR = (
    "hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
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
    assert (measured, RDF.type, QUDT_SCHEMA.Quantity) in g
    assert g.value(measured, QUDT_SCHEMA.hasQuantityKind) == QKIND["Time"]
    assert g.value(measured, QUDT_SCHEMA.value) is None  # runtime state, unmeasured
    assert g.value(measured, QUDT_SCHEMA.unit) == QUDT_UNIT["SEC"]

    interval = next(g.subjects(RDF.type, TIME.ProperInterval))
    assert (interval, RDF.type, TIME.ProperInterval) in g
    assert g.value(cstr, TIME.hasTime) == interval
    beginning = g.value(interval, TIME.hasBeginning)
    end = g.value(interval, TIME.hasEnd)
    assert (beginning, RDF.type, TIME.Instant) in g
    assert (end, RDF.type, TIME.Instant) in g

    threshold = g.value(cstr, CSTR.threshold)
    assert (threshold, RDF.type, TIME.Duration) in g
    assert float(g.value(threshold, QUDT_SCHEMA.value)) == pytest.approx(5.0)
    assert g.value(threshold, QUDT_SCHEMA.unit) == QUDT_UNIT["SEC"]


def test_elapsed_threshold_keeps_the_milliseconds_it_was_written_in(parse_mutated):
    g = _build(parse_mutated, "wait5: elapsed less than 10.0 ms")
    cstr = next(g.subjects(RDF.type, CSTR_EXT.TimeConstraint))
    threshold = g.value(cstr, CSTR.threshold)
    assert float(g.value(threshold, QUDT_SCHEMA.value)) == pytest.approx(10.0)
    assert g.value(threshold, QUDT_SCHEMA.unit) == QUDT_UNIT["MilliSEC"]


def test_elapsed_equality_emits_reference_and_tolerance(parse_mutated):
    g = _build(parse_mutated, "wait5: elapsed equal to 5.0 s within 10.0 ms")
    cstr = next(g.subjects(RDF.type, CSTR_EXT.TimeConstraint))
    assert (cstr, RDF.type, CSTR.EqualityConstraint) in g

    reference = g.value(cstr, CSTR["reference-value"])
    assert float(g.value(reference, QUDT_SCHEMA.value)) == pytest.approx(5.0)
    assert g.value(reference, QUDT_SCHEMA.unit) == QUDT_UNIT["SEC"]

    tolerance = g.value(cstr, CSTR_EXT.tolerance)
    assert float(g.value(tolerance, QUDT_SCHEMA.value)) == pytest.approx(10.0)
    assert g.value(tolerance, QUDT_SCHEMA.unit) == QUDT_UNIT["MilliSEC"]


def test_plain_elapsed_interval_begins_at_motion_entry(parse_mutated):
    g = _build(parse_mutated, "wait5: elapsed greater than 5.0 s")

    cstr = next(g.subjects(RDF.type, CSTR_EXT.TimeConstraint))
    interval = g.value(cstr, TIME.hasTime)
    assert (interval, RDF.type, TIME.ProperInterval) in g
    assert str(g.value(interval, TIME.hasBeginning)).endswith("/motion-entry")


def test_elapsed_since_observed_hangs_the_interval_on_the_quantitys_phenomenon_time(
    parse_mutated,
):
    g = _build(
        parse_mutated,
        "seen: elapsed since <shared.world.pose-ee-base> observed less than 1.0 s",
    )

    cstr = next(g.subjects(RDF.type, CSTR_EXT.TimeConstraint))
    interval = g.value(cstr, TIME.hasTime)
    instant = g.value(interval, TIME.hasBeginning)
    assert (instant, RDF.type, TIME.Instant) in g

    pose = next(g.subjects(SOSA.phenomenonTime, instant))
    assert str(pose).endswith("/shared/world/pose-ee-base")


def test_manifest_includes_time_shacl_exactly_once():
    constraints = _build_manifest([])["@graph"][0]["constraints"]
    matches = [c for c in constraints if c.endswith("/time.shacl.ttl")]
    assert matches == ["https://secorolab.github.io/metamodels/time.shacl.ttl"]
