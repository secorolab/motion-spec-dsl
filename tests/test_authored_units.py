# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The graph records the unit a model was written in.

A reader converts when it needs a number, and for coordinates `rdf-utils` already does:
orientations come back in radians and positions in metres however they were authored. A
`time:Duration` is the exception -- its vocabulary has no unit below the second, so it is
written in seconds.
"""

from __future__ import annotations

import pytest
from rdflib.namespace import RDF

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import QUDT_SCHEMA, TIME
from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT

SPEC_ANCHOR = "linear-velocity zero-linvel = 0.0 m/s"

AUTHORED = [
    pytest.param("distance", "d-cm", "5.0 cm", 5.0, "CentiM", id="cm"),
    pytest.param("distance", "d-mm", "1.0 mm", 1.0, "MilliM", id="mm"),
    pytest.param("angle", "a-deg", "90.0 deg", 90.0, "DEG", id="deg"),
    pytest.param("angular-velocity", "av", "10.0 deg/s", 10.0, "DEG-PER-SEC", id="deg_per_s"),
    pytest.param("linear-velocity", "lv", "5.0 cm/s", 5.0, "CentiM-PER-SEC", id="cm_per_s"),
    pytest.param(
        "angular-acceleration", "aa", "90.0 deg/s^2", 90.0, "DEG-PER-SEC2", id="deg_per_s2"
    ),
]


def _build(parse_mutated, declaration: str):
    model = parse_mutated(SPEC_ANCHOR, f"{SPEC_ANCHOR},\n        {declaration}")
    return MotionSpecDatasetBuilder(model).build()[0].default_graph


def _quantity(graph, name: str):
    [node] = [s for s in graph.subjects(RDF.type, QUDT_SCHEMA.Quantity) if str(s).endswith(name)]
    return node


@pytest.mark.parametrize(("kind", "name", "measure", "value", "unit"), AUTHORED)
def test_a_quantity_keeps_the_unit_it_was_written_in(
    parse_mutated, kind, name, measure, value, unit
):
    graph = _build(parse_mutated, f"{kind} {name} = {measure}")
    node = _quantity(graph, name)
    assert float(graph.value(node, QUDT_SCHEMA.value)) == pytest.approx(value)
    assert graph.value(node, QUDT_SCHEMA.unit) == QUDT_UNIT[unit]


def test_a_duration_is_normalised_because_owl_time_has_no_smaller_unit(parse_mutated):
    graph = _build(parse_mutated, "duration du-ms = 500.0 ms")
    node = next(s for s in graph.subjects(RDF.type, TIME.Duration) if str(s).endswith("du-ms"))
    assert float(graph.value(node, TIME.numericDuration)) == pytest.approx(0.5)
    assert graph.value(node, TIME.unitType) == TIME.unitSecond


def test_a_debounce_carries_the_unit_of_its_value(parse_mutated):
    """A value and its unit travel together, or 300 ms silently becomes 300 s."""
    graph = (
        MotionSpecDatasetBuilder(
            parse_mutated("after active for 0.3 s", "after active for 300.0 ms")
        )
        .build()[0]
        .default_graph
    )
    node = _quantity(graph, ".debounce")
    assert float(graph.value(node, QUDT_SCHEMA.value)) == pytest.approx(300.0)
    assert graph.value(node, QUDT_SCHEMA.unit) == QUDT_UNIT["MilliSEC"]
