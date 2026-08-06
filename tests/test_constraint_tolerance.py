# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A constraint's satisfaction band reaches the graph, whatever kind the constraint is.

The band is what makes a comparison decidable: an equality's target is a single point that is
never met exactly, and a one-sided gate on a motion that only asymptotes onto its threshold
never strictly crosses it either. Both say how close counts, so both carry one -- authored per
constraint, or defaulted per quantity kind for the whole model.
"""

from __future__ import annotations

import re

import pytest
from rdflib.namespace import RDF
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import CSTR, CSTR_EXT, QUDT_SCHEMA

ANCHOR = (
    "hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
)
SETTLED = "settled-z: <shared.world.twist-ee-base>.linvel.z"
SETTLED_EQUAL = f"{SETTLED} equal to <shared.spec.zero-linvel> within <shared.spec.satisfied-band-vel>"
SETTLED_GATE = f"{SETTLED} less than <shared.spec.zero-linvel>"


def _swap(source: str, anchor: str, replacement: str) -> str:
    assert anchor in source, anchor
    return source.replace(anchor, replacement, 1)


def _graph(parse_source, source: str):
    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


def _band_of(graph, constraint_name: str) -> float | None:
    """Value of the band linked to the constraint named `constraint_name`, if it has one."""
    for constraint, band in graph.subject_objects(CSTR_EXT.tolerance):
        if str(constraint).rsplit("/", 1)[-1] == constraint_name:
            return float(str(graph.value(band, QUDT_SCHEMA.value)))
    return None


def test_an_authored_band_reaches_its_equality_constraint(parse_source, base_source) -> None:
    graph = _graph(parse_source, base_source)

    constraint, tolerance = next(iter(graph.subject_objects(CSTR_EXT.tolerance)))
    assert (constraint, RDF.type, CSTR.EqualityConstraint) in graph
    # The band is the authored quantity itself, carrying the kind and unit of what it bounds.
    band = graph.value(tolerance, QUDT_SCHEMA.value)
    assert band is not None and float(str(band)) == 0.01
    assert graph.value(tolerance, QUDT_SCHEMA.unit) is not None


def test_an_equality_without_a_band_is_rejected(parse_source, base_source) -> None:
    """Nothing implicit is left to fall back on, so the model has to say it."""
    bandless = ANCHOR.replace(" within <shared.spec.satisfied-band>", "")
    with pytest.raises(ValueError, match="states no band"):
        _graph(parse_source, _swap(base_source, ANCHOR, bandless))


def test_a_whole_pose_cannot_state_one_band(parse_source, base_source) -> None:
    """Its error mixes metres and radians, and a band carries one kind and one unit."""
    # Same constraint name: a monitor elsewhere in the fixture refers to it.
    whole_pose = (
        "hold-position: keeping <shared.world.pose-ee-base> equal to <spec.home-pose> "
        "within <shared.spec.satisfied-band>"
    )
    with pytest.raises(ValueError, match="mixes a position and an orientation"):
        _graph(parse_source, _swap(base_source, ANCHOR, whole_pose))


def test_a_one_sided_gate_takes_an_authored_band(parse_source, base_source) -> None:
    """How close to the threshold counts as arrived."""
    gate = f"{SETTLED_GATE} within <shared.spec.satisfied-band-vel>"
    graph = _graph(parse_source, _swap(base_source, SETTLED_EQUAL, gate))

    settled = next(graph.subjects(RDF.type, CSTR.LessThanConstraint))
    assert (settled, CSTR_EXT.tolerance, None) in graph
    assert _band_of(graph, "settled-z") == 0.01


def test_a_gate_may_state_no_band(parse_source, base_source) -> None:
    """Its admissible region has an interior, so the boundary itself is a usable answer."""
    graph = _graph(parse_source, _swap(base_source, SETTLED_EQUAL, SETTLED_GATE))

    assert _band_of(graph, "settled-z") is None


def test_a_model_wide_default_bands_every_constraint_of_that_kind(
    parse_source, base_source
) -> None:
    """Stated once and taken by every constraint whose error carries that kind, so the band a
    model is tuned against is visible in it rather than implied by a constant."""
    source = _swap(base_source, SETTLED_EQUAL, SETTLED_GATE)
    source = _swap(source, "guarded-motion", "tolerances { linear-velocity: 0.02 m/s }\n\nguarded-motion")
    graph = _graph(parse_source, source)

    assert _band_of(graph, "settled-z") == 0.02
    # An inline default is one shared node, not a copy per constraint that takes it.
    bands = set(graph.objects(None, CSTR_EXT.tolerance))
    assert len(bands) == 2  # the authored metre band, and the one default


def test_an_authored_band_wins_over_the_default(parse_source, base_source) -> None:
    source = _swap(
        base_source, "guarded-motion", "tolerances { linear-velocity: 0.02 m/s }\n\nguarded-motion"
    )
    graph = _graph(parse_source, source)

    assert _band_of(graph, "settled-z") == 0.01


def test_a_default_must_be_measured_in_its_kind(parse_source, base_source) -> None:
    """It applies to every constraint of that kind, so a wrong unit is a wrong band on all
    of them -- and the value rule is shared across kinds, so the grammar cannot tell."""
    source = _swap(
        base_source, "guarded-motion", "tolerances { linear-velocity: 0.02 m }\n\nguarded-motion"
    )
    with pytest.raises(TextXSemanticError, match="not measured in 'm'"):
        _graph(parse_source, source)


def test_a_kind_takes_one_default(parse_source, base_source) -> None:
    """`distance` and `linear-distance` name one kind, so the collision only shows up here."""
    source = _swap(
        base_source,
        "guarded-motion",
        "tolerances { linear-distance: 0.01 m, distance: 0.02 m }\n\nguarded-motion",
    )
    with pytest.raises(TextXSemanticError, match="already has a default band"):
        _graph(parse_source, source)


def test_a_tracked_path_states_a_band_too() -> None:
    """`on <path>` is an equality against a curve rather than a point; a controller drives
    the error to zero but never reaches it, so it needs the same band `equal to` does."""
    from pathlib import Path

    from motion_spec_dsl.langs import motion_spec_metamodel

    model_path = Path(__file__).resolve().parents[1] / "models" / "pick_place_single"
    source = (model_path / "pick_place_single.robmot").read_text()
    # The model now declares model-wide defaults, which would supply the very band this test
    # asserts the absence of; drop them so the constraint really states none.
    source = re.sub(r"\ntolerances \{[^}]*\}\n", "\n", source)
    bandless = _swap(
        source,
        "on <spec.approach-path> within <shared.spec.satisfied-band>,",
        "on <spec.approach-path>,",
    )
    parsed = motion_spec_metamodel().model_from_str(
        bandless, file_name=str(model_path / "pick_place_single.robmot")
    )
    with pytest.raises(ValueError, match="states no band"):
        MotionSpecDatasetBuilder(parsed).build()
