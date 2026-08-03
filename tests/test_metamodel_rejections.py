# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Invalid RDF rejected by the local SHACL extensions."""

from pathlib import Path

import pytest
from rdflib import BNode, Graph, Literal, Namespace, RDF, URIRef
from rdflib.namespace import SH, XSD

pyshacl = pytest.importorskip("pyshacl")

METAMODELS = Path(__file__).resolve().parents[2] / "metamodels"
QUDT = Namespace("http://qudt.org/schema/qudt/")
QKIND = Namespace("http://qudt.org/vocab/quantitykind/")
ALGO = Namespace("https://secorolab.github.io/metamodels/algorithm#")
CSTR = Namespace("https://comp-rob2b.github.io/metamodels/task/constraint#")
GEOM_COORD = Namespace("https://comp-rob2b.github.io/metamodels/geometry/coordinates#")
GEOM_OP = Namespace("https://secorolab.github.io/metamodels/geometry/spatial-operators#")
GEOM_PATH = Namespace("https://secorolab.github.io/metamodels/geometry/path#")
TIME = Namespace("http://www.w3.org/2006/time#")
UNIT = Namespace("http://qudt.org/vocab/unit/")
FOCUS = URIRef("urn:test:focus")


def _conforms(shape_file: str, shape: URIRef, data: Graph) -> bool:
    shapes = Graph().parse(METAMODELS / shape_file, format="turtle")
    shapes.add((shape, SH.targetNode, FOCUS))
    return pyshacl.validate(data, shacl_graph=shapes)[0]


def test_positive_distance_requires_an_authored_value() -> None:
    data = Graph()
    data.add((FOCUS, RDF.type, QUDT.Quantity))
    data.add((FOCUS, QUDT.hasQuantityKind, QKIND.Distance))
    assert not _conforms("geometry/path.shacl.ttl", GEOM_PATH.PositiveDistanceShape, data)


def test_progress_constraint_requires_linear_velocity_operands() -> None:
    data = Graph()
    path, measured, threshold = URIRef("urn:test:path"), BNode(), BNode()
    data.add((FOCUS, RDF.type, ALGO.ProgressConstraint))
    data.add((FOCUS, RDF.type, CSTR.GreaterThanConstraint))
    data.add((FOCUS, GEOM_OP.path, path))
    data.add((path, RDF.type, GEOM_PATH.Path))
    data.add((FOCUS, CSTR.quantity, measured))
    data.add((measured, RDF.type, QUDT.Quantity))
    data.add((measured, QUDT.hasQuantityKind, QKIND.LinearVelocity))
    data.add((FOCUS, CSTR.threshold, threshold))
    assert not _conforms("algorithm-extension.shacl.ttl", ALGO.ProgressConstraint, data)


def _duration(**triples) -> Graph:
    data = Graph()
    data.add((FOCUS, RDF.type, TIME.Duration))
    data.add((FOCUS, QUDT.hasQuantityKind, QKIND.Time))
    for predicate, value in triples.items():
        data.add((FOCUS, QUDT[predicate], value))
    return data


def test_owl_time_duration_requires_a_value() -> None:
    assert not _conforms("time.shacl.ttl", TIME.Duration, _duration(unit=UNIT.SEC))


def test_owl_time_duration_accepts_milliseconds() -> None:
    """The whole reason the magnitude is qudt: owl-time has no unit below the second."""
    data = _duration(value=Literal(10.0, datatype=XSD.double), unit=UNIT.MilliSEC)
    assert _conforms("time.shacl.ttl", TIME.Duration, data)


def test_owl_time_duration_rejects_the_owl_time_magnitude() -> None:
    """Emitting time:numericDuration means the value was rescaled to seconds."""
    data = _duration(value=Literal(0.01, datatype=XSD.double), unit=UNIT.SEC)
    data.add((FOCUS, TIME.numericDuration, Literal("0.01", datatype=XSD.decimal)))
    assert not _conforms("time.shacl.ttl", TIME.Duration, data)


def test_orientation_quaternion_must_have_unit_length() -> None:
    data = Graph()
    data.add((FOCUS, RDF.type, GEOM_COORD.Quaternion))
    for predicate, value in zip(
        (GEOM_COORD.x, GEOM_COORD.y, GEOM_COORD.z, GEOM_COORD.w),
        (0.0, 0.0, 0.0, 2.0),
        strict=True,
    ):
        data.add((FOCUS, predicate, Literal(value, datatype=XSD.double)))
    assert not _conforms(
        "geometry/geometry.shacl.ttl", GEOM_COORD.QuaternionShape, data
    )
