# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""The one draw a randomized model gets: every `distrib:SampledQuantity` becomes a plain value.

The numbers are drawn into the rdflib graph before anything reads it, so the serializer and the
KDL walk both see a sampled quantity exactly as they would an authored constant.
"""

from __future__ import annotations

import math
import random

from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdf_utils.models.vocab import (
    URI_DISTRIB_PRED_COV,
    URI_DISTRIB_PRED_FROM_DISTRIB,
    URI_DISTRIB_PRED_LOWER,
    URI_DISTRIB_PRED_MEAN,
    URI_DISTRIB_PRED_STD,
    URI_DISTRIB_PRED_UPPER,
    URI_DISTRIB_TYPE_NORMAL,
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_DISTRIB_TYPE_UNIFORM,
    URI_DISTRIB_TYPE_UNIFORM_ROT,
)
from rdflib import Literal
from rdflib.namespace import RDF, XSD

from motion_spec_dsl.rdf_parser.vocab import GEOM_COORD, QUDT_SCHEMA


def draw_samples(graph, rng: random.Random) -> dict[str, list[float]]:
    """Draw every sampled quantity `graph` declares and write the numbers into `graph`.

    Parameters:
        graph: an rdflib graph, drawn in place
        rng: the caller's generator, so one seed covers every graph of a generation

    Returns:
        ``{node_uri: [values]}``, or ``{}`` when nothing is sampled
    """
    draws: dict[str, list[float]] = {}
    for node in sorted(graph.subjects(RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY), key=str):
        # A graph may be visited twice; an already-drawn node keeps its numbers.
        if graph.value(node, QUDT_SCHEMA.value) is not None:
            continue
        if graph.value(node, GEOM_COORD.x) is not None:
            continue
        values = _draw(graph, node, rng)
        for predicate, value in zip(_predicates(node, values), values):
            graph.add((node, predicate, Literal(float(value), datatype=XSD.double)))
        draws[str(node)] = values
    return draws


def _predicates(node, values) -> list:
    """The predicates the drawn numbers are written on: a scalar value, or an xyz position."""
    if len(values) == 1:
        return [QUDT_SCHEMA.value]
    if len(values) == 3:
        return [GEOM_COORD.x, GEOM_COORD.y, GEOM_COORD.z]
    raise ConstraintViolation(
        "sampling",
        f"sampled quantity '{node}' drew {len(values)} numbers; only a scalar or an xyz position "
        "can be written back into the model graph",
    )


def _numbers(graph, node, predicate) -> list[float] | None:
    """A distribution parameter as numbers: an RDF list, or a bare literal standing for one."""
    value = graph.value(node, predicate)
    if value is None:
        return None
    if isinstance(value, Literal):
        return [float(value)]
    return [float(item) for item in graph.items(value)]


def _deviations(graph, distribution, count) -> list[float]:
    """The per-component standard deviations of a normal distribution."""
    spread = _numbers(graph, distribution, URI_DISTRIB_PRED_STD)
    if spread is not None:
        return spread * count if len(spread) == 1 else spread
    covariance = graph.value(distribution, URI_DISTRIB_PRED_COV)
    if covariance is None:
        raise ConstraintViolation(
            "sampling",
            f"normal distribution '{distribution}' states neither a standard deviation nor a "
            "covariance, so there is nothing to draw around its mean",
        )
    rows = [[float(entry) for entry in graph.items(row)] for row in graph.items(covariance)]
    for i, row in enumerate(rows):
        for j, entry in enumerate(row):
            if i != j and entry != 0.0:
                raise ConstraintViolation(
                    "sampling",
                    f"covariance of distribution '{distribution}' is not diagonal; correlated "
                    "components are not drawn",
                )
    return [math.sqrt(rows[i][i]) for i in range(len(rows))]


def _draw(graph, node, rng) -> list[float]:
    """One draw from the distribution a sampled quantity names."""
    distribution = graph.value(node, URI_DISTRIB_PRED_FROM_DISTRIB)
    if distribution is None:
        raise ConstraintViolation(
            "sampling", f"sampled quantity '{node}' names no distribution to draw from"
        )
    types = get_node_types(graph, distribution)
    if URI_DISTRIB_TYPE_UNIFORM_ROT not in types:
        if URI_DISTRIB_TYPE_UNIFORM in types:
            lower = _numbers(graph, distribution, URI_DISTRIB_PRED_LOWER) or []
            upper = _numbers(graph, distribution, URI_DISTRIB_PRED_UPPER) or []
            return [rng.uniform(low, high) for low, high in zip(lower, upper)]
        if URI_DISTRIB_TYPE_NORMAL in types:
            mean = _numbers(graph, distribution, URI_DISTRIB_PRED_MEAN) or []
            deviations = _deviations(graph, distribution, len(mean))
            return [rng.gauss(m, sd) for m, sd in zip(mean, deviations)]
    raise ConstraintViolation(
        "sampling",
        f"sampled quantity '{node}' draws from '{distribution}'; only uniform and normal "
        "distributions are drawn",
    )


def demo() -> None:
    """A scalar and a 3-vector draw land on their own predicates and in bounds, a second pass
    over the same graph draws nothing, and one seed reproduces both."""
    import rdflib
    from rdflib import URIRef
    from rdflib.collection import Collection

    base = "http://example.org/demo/"

    def rdf_list(graph, values):
        head = rdflib.BNode()
        Collection(graph, head, [Literal(float(v)) for v in values])
        return head

    def build():
        graph = rdflib.Graph()
        for name, lower, upper in (
            ("t", [1.0], [2.0]),
            ("p", [0.0, -1.0, 4.0], [1.0, 0.0, 5.0]),
        ):
            node = URIRef(f"{base}{name}")
            distribution = URIRef(f"{base}{name}-distrib")
            graph.add((node, RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY))
            graph.add((node, URI_DISTRIB_PRED_FROM_DISTRIB, distribution))
            graph.add((distribution, RDF.type, URI_DISTRIB_TYPE_UNIFORM))
            graph.add((distribution, URI_DISTRIB_PRED_LOWER, rdf_list(graph, lower)))
            graph.add((distribution, URI_DISTRIB_PRED_UPPER, rdf_list(graph, upper)))
        return graph

    graph = build()
    draws = draw_samples(graph, random.Random(7))
    assert sorted(draws) == [f"{base}p", f"{base}t"], draws

    scalar = graph.value(URIRef(f"{base}t"), QUDT_SCHEMA.value)
    assert scalar is not None and 1.0 <= float(scalar) <= 2.0, scalar
    assert graph.value(URIRef(f"{base}t"), GEOM_COORD.x) is None
    position = [
        float(graph.value(URIRef(f"{base}p"), axis))
        for axis in (GEOM_COORD.x, GEOM_COORD.y, GEOM_COORD.z)
    ]
    assert all(lo <= v <= hi for v, lo, hi in zip(position, [0, -1, 4], [1, 0, 5])), position
    assert position == draws[f"{base}p"], draws
    assert graph.value(URIRef(f"{base}p"), QUDT_SCHEMA.value) is None

    # A second pass sees values already there and leaves them alone.
    assert draw_samples(graph, random.Random(11)) == {}, "redrew an already-drawn quantity"
    assert float(graph.value(URIRef(f"{base}t"), QUDT_SCHEMA.value)) == float(scalar)

    # The same seed over a fresh graph reproduces the same numbers.
    assert draw_samples(build(), random.Random(7)) == draws
    print("sampling demo ok")


if __name__ == "__main__":
    demo()
