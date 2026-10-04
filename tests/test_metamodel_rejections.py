# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Rules only the local SHACL extensions check: no DSL validation states them."""

from pathlib import Path

import pytest
from rdflib import Graph, Literal, Namespace, RDF, URIRef
from rdflib.namespace import SH, XSD

pyshacl = pytest.importorskip("pyshacl")

METAMODELS = Path(__file__).resolve().parents[2] / "metamodels"
QUDT = Namespace("http://qudt.org/schema/qudt/")
QKIND = Namespace("http://qudt.org/vocab/quantitykind/")
GEOM_COORD = Namespace("https://comp-rob2b.github.io/metamodels/geometry/coordinates#")
GEOM_PATH = Namespace("https://secorolab.github.io/metamodels/geometry/path#")
TIME = Namespace("http://www.w3.org/2006/time#")
UNIT = Namespace("http://qudt.org/vocab/unit/")
FOCUS = URIRef("urn:test:focus")
DURATION = [(RDF.type, TIME.Duration), (QUDT.hasQuantityKind, QKIND.Time)]
SECONDS = (QUDT.value, Literal(0.01, datatype=XSD.double))


@pytest.mark.parametrize(
    ("shape_file", "shape", "triples", "conforms"),
    [
        pytest.param(
            "geometry/path.shacl.ttl",
            GEOM_PATH.PositiveDistanceShape,
            [(RDF.type, QUDT.Quantity), (QUDT.hasQuantityKind, QKIND.Distance)],
            False,
            id="positive_distance_without_value",
        ),
        pytest.param(
            "time.shacl.ttl", TIME.Duration, [*DURATION, (QUDT.unit, UNIT.SEC)], False, id="duration_without_value"
        ),
        pytest.param(
            "time.shacl.ttl",
            TIME.Duration,
            [*DURATION, (QUDT.value, Literal(10.0, datatype=XSD.double)), (QUDT.unit, UNIT.MilliSEC)],
            True,
            id="duration_in_milliseconds",
        ),
        # Emitting time:numericDuration means the value was rescaled to seconds.
        pytest.param(
            "time.shacl.ttl",
            TIME.Duration,
            [*DURATION, SECONDS, (QUDT.unit, UNIT.SEC), (TIME.numericDuration, Literal("0.01", datatype=XSD.decimal))],
            False,
            id="duration_with_owl_time_magnitude",
        ),
        pytest.param(
            "geometry/geometry.shacl.ttl",
            GEOM_COORD.QuaternionShape,
            [
                (RDF.type, GEOM_COORD.Quaternion),
                (GEOM_COORD.x, Literal(0.0, datatype=XSD.double)),
                (GEOM_COORD.y, Literal(0.0, datatype=XSD.double)),
                (GEOM_COORD.z, Literal(0.0, datatype=XSD.double)),
                (GEOM_COORD.w, Literal(2.0, datatype=XSD.double)),
            ],
            False,
            id="quaternion_of_non_unit_length",
        ),
    ],
)
def test_a_shape_accepts_or_rejects_its_focus(shape_file, shape, triples, conforms) -> None:
    shapes = Graph().parse(METAMODELS / shape_file, format="turtle")
    shapes.add((shape, SH.targetNode, FOCUS))
    data = Graph()
    for predicate, value in triples:
        data.add((FOCUS, predicate, value))
    assert pyshacl.validate(data, shacl_graph=shapes)[0] is conforms
