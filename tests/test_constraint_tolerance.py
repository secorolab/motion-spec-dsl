# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""An authored `within` band on an equality constraint reaches the graph.

The grammar has always parsed it and the metamodel has always defined
`cstr-ext:tolerance`; the spatial path dropped it between the two, so every satisfaction
verdict came from one global constant instead.
"""

from __future__ import annotations

import pytest
from rdflib.namespace import RDF

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import CSTR, CSTR_EXT, QUDT_SCHEMA

ANCHOR = (
    "hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
)
def _graph(parse_source, base_source: str, constraint: str):
    """Build the base model with `constraint` in place of its anchor equality."""
    source = base_source.replace(ANCHOR, constraint, 1)
    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


def test_an_authored_band_reaches_its_equality_constraint(parse_source, base_source) -> None:
    graph = _graph(parse_source, base_source, ANCHOR)

    constraint, tolerance = next(iter(graph.subject_objects(CSTR_EXT.tolerance)))
    assert (constraint, RDF.type, CSTR.EqualityConstraint) in graph
    # The band is the authored quantity itself, carrying the kind and unit of what it bounds.
    band = graph.value(tolerance, QUDT_SCHEMA.value)
    assert band is not None and float(band) == 0.01
    assert graph.value(tolerance, QUDT_SCHEMA.unit) is not None


def test_an_equality_without_a_band_is_rejected(parse_source, base_source) -> None:
    """Nothing implicit is left to fall back on, so the model has to say it."""
    bandless = ANCHOR.replace(" within <shared.spec.satisfied-band>", "")
    with pytest.raises(ValueError, match="states no band"):
        _graph(parse_source, base_source, bandless)


def test_a_whole_pose_cannot_state_one_band(parse_source, base_source) -> None:
    """Its error mixes metres and radians, and a band carries one kind and one unit."""
    # Same constraint name: a monitor elsewhere in the fixture refers to it.
    whole_pose = (
        "hold-position: keeping <shared.world.pose-ee-base> equal to <spec.home-pose> "
        "within <shared.spec.satisfied-band>"
    )
    with pytest.raises(ValueError, match="mixes a position and an orientation"):
        _graph(parse_source, base_source, whole_pose)
