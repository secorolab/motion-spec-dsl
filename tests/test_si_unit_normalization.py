# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Authored non-SI units convert to their canonical SI unit at the DSL->RDF boundary."""

from __future__ import annotations

import math

import pytest
from rdflib.compare import graph_diff, to_isomorphic
from rdflib.namespace import RDF
from textx.exceptions import TextXSemanticError

from motion_spec.rdf_parser.vocab import GEOM_COORD, QUDT_SCHEMA, TIME
from motion_spec_dsl.rdf.builder import MotionSpecDatasetBuilder
from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT

SPEC_ANCHOR = "linear-velocity zero-linvel = 0.0 m/s"
POSE_ANCHOR = "pose home-pose = snapshot of <shared.world.pose-ee-base>"

FORBIDDEN_UNITS = {
    QUDT_UNIT["CentiM"],
    QUDT_UNIT["MilliM"],
    QUDT_UNIT["DEG"],
    QUDT_UNIT["DEG-PER-SEC"],
    QUDT_UNIT["DEG-PER-SEC2"],
    QUDT_UNIT["CentiM-PER-SEC"],
    QUDT_UNIT["MilliSEC"],
}


def _build(parse_mutated, anchor: str, declaration: str):
    model = parse_mutated(anchor, f"{anchor},\n        {declaration}")
    return MotionSpecDatasetBuilder(model).build()[0].default_graph


def _quantity_node(graph, name: str):
    matches = [
        s for s in graph.subjects(RDF.type, QUDT_SCHEMA.Quantity) if str(s).endswith(name)
    ]
    assert len(matches) == 1, f"expected exactly one Quantity node for {name!r}, got {matches}"
    return matches[0]


CONVERSIONS = [
    pytest.param("distance", "d-cm", "5.0 cm", 0.05, "M", id="cm_to_m"),
    pytest.param("distance", "d-mm", "1.0 mm", 0.001, "M", id="mm_to_m"),
    pytest.param("angle", "a-deg", "90.0 deg", math.pi / 2, "RAD", id="deg_to_rad"),
    pytest.param(
        "angular-velocity", "av-deg", "10.0 deg/s", 10.0 * math.pi / 180.0, "RAD-PER-SEC",
        id="deg_per_s_to_rad_per_s",
    ),
    pytest.param("linear-velocity", "lv-cms", "5.0 cm/s", 0.05, "M-PER-SEC", id="cm_per_s_to_m_per_s"),
    pytest.param(
        "angular-acceleration", "aa-deg2", "90.0 deg/s^2", 90.0 * math.pi / 180.0, "RAD-PER-SEC2",
        id="deg_per_s2_to_rad_per_s2",
    ),
]


@pytest.mark.parametrize(("kind", "name", "measure", "expected_value", "unit_token"), CONVERSIONS)
def test_non_si_token_converts(parse_mutated, kind, name, measure, expected_value, unit_token):
    graph = _build(parse_mutated, SPEC_ANCHOR, f"{kind} {name} = {measure}")
    node = _quantity_node(graph, name)
    assert float(graph.value(node, QUDT_SCHEMA.value)) == pytest.approx(expected_value)
    assert graph.value(node, QUDT_SCHEMA.unit) == QUDT_UNIT[unit_token]


def test_duration_ms_converts_to_seconds(parse_mutated):
    graph = _build(parse_mutated, SPEC_ANCHOR, "duration du-ms = 500.0 ms")
    node = next(s for s in graph.subjects(RDF.type, TIME.Duration) if str(s).endswith("du-ms"))
    assert float(graph.value(node, TIME.numericDuration)) == pytest.approx(0.5)
    assert graph.value(node, TIME.unitType) == TIME.unitSecond


def test_monitor_debounce_ms_converts_to_seconds(parse_mutated):
    graph = MotionSpecDatasetBuilder(
        parse_mutated("after active for 0.3 s", "after active for 300.0 ms")
    ).build()[0].default_graph
    node = next(
        s for s in graph.subjects(RDF.type, QUDT_SCHEMA.Quantity) if str(s).endswith(".debounce")
    )
    assert float(graph.value(node, QUDT_SCHEMA.value)) == pytest.approx(0.3)
    assert graph.value(node, QUDT_SCHEMA.unit) == QUDT_UNIT["SEC"]


def test_timestep_ms_converts_to_seconds(parse_mutated):
    model = parse_mutated("timestep:   1.0 ms", "timestep:   2.0 ms")
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph
    node = next(
        s for s in graph.subjects(RDF.type, QUDT_SCHEMA.Quantity) if str(s).endswith(".timestep")
    )
    assert float(graph.value(node, QUDT_SCHEMA.value)) == pytest.approx(0.002)
    assert graph.value(node, QUDT_SCHEMA.unit) == QUDT_UNIT["SEC"]


def test_no_forbidden_unit_uri_in_emitted_graph(parse_mutated):
    declarations = ",\n        ".join(
        [
            "distance d-cm = 5.0 cm",
            "angle a-deg = 90.0 deg",
            "angular-velocity av-deg = 10.0 deg/s",
            "linear-velocity lv-cms = 5.0 cm/s",
            "angular-acceleration aa-deg2 = 90.0 deg/s^2",
            "duration du-ms = 500.0 ms",
        ]
    )
    model = parse_mutated(SPEC_ANCHOR, f"{SPEC_ANCHOR},\n        {declarations}")
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph
    units = set(graph.objects(None, QUDT_SCHEMA.unit))
    assert not units & FORBIDDEN_UNITS


ISOMORPHIC_PAIRS = [
    pytest.param("distance d-equiv", "5.0 cm", "0.05 m", id="distance_cm_vs_m"),
    pytest.param("angle a-equiv", "90.0 deg", f"{math.pi / 2!r} rad", id="angle_deg_vs_rad"),
]


@pytest.mark.parametrize(("decl_head", "non_si", "si"), ISOMORPHIC_PAIRS)
def test_two_spellings_of_the_same_quantity_are_isomorphic(parse_mutated, decl_head, non_si, si):
    graph_non_si = _build(parse_mutated, SPEC_ANCHOR, f"{decl_head} = {non_si}")
    graph_si = _build(parse_mutated, SPEC_ANCHOR, f"{decl_head} = {si}")
    iso_non_si, iso_si = to_isomorphic(graph_non_si), to_isomorphic(graph_si)
    _, in_non_si, in_si = graph_diff(iso_non_si, iso_si)
    assert len(in_non_si) == 0 and len(in_si) == 0


def test_velocity_twist_both_subspaces_convert_to_si(parse_mutated):
    decl = (
        "velocity-twist vt-nonsi = { angular-velocity: (10.0, 0.0, 0.0) deg/s,"
        " linear-velocity: (5.0, 0.0, 0.0) cm/s }"
    )
    graph = _build(parse_mutated, SPEC_ANCHOR, decl)
    values = {float(v) for v in graph.objects(None, QUDT_SCHEMA.value)}
    assert any(v == pytest.approx(10.0 * math.pi / 180.0) for v in values)
    assert any(v == pytest.approx(0.05) for v in values)
    units = set(graph.objects(None, QUDT_SCHEMA.unit))
    assert not units & FORBIDDEN_UNITS


def test_acceleration_twist_both_subspaces_in_si(parse_mutated):
    decl = (
        "acceleration-twist at-nonsi = { angular-acceleration: (90.0, 0.0, 0.0) deg/s^2,"
        " linear-acceleration: (2.0, 0.0, 0.0) m/s^2 }"
    )
    graph = _build(parse_mutated, SPEC_ANCHOR, decl)
    values = {float(v) for v in graph.objects(None, QUDT_SCHEMA.value)}
    assert any(v == pytest.approx(90.0 * math.pi / 180.0) for v in values)
    assert any(v == pytest.approx(2.0) for v in values)
    units = set(graph.objects(None, QUDT_SCHEMA.unit))
    assert not units & FORBIDDEN_UNITS


def test_wrench_both_subspaces_in_si(parse_mutated):
    decl = "wrench w-nonsi = { torque: (1.0, 0.0, 0.0) Nm, force: (3.0, 0.0, 0.0) N }"
    graph = _build(parse_mutated, SPEC_ANCHOR, decl)
    values = {float(v) for v in graph.objects(None, QUDT_SCHEMA.value)}
    assert any(v == pytest.approx(1.0) for v in values)
    assert any(v == pytest.approx(3.0) for v in values)
    units = set(graph.objects(None, QUDT_SCHEMA.unit))
    assert QUDT_UNIT["N-M"] in units
    assert QUDT_UNIT.N in units


def test_pose_position_and_orientation_convert_to_si(parse_mutated):
    decl = (
        "pose pose-nonsi = { position: (5.0, 0.0, 0.0) cm,"
        " orientation: euler { axes: xyz extrinsic, angles: (90.0, 0.0, 0.0) deg } }"
    )
    model = parse_mutated(POSE_ANCHOR, f"{POSE_ANCHOR},\n            {decl}")
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph
    values = {float(v) for v in graph.objects(None, QUDT_SCHEMA.value)}
    assert any(v == pytest.approx(0.05) for v in values)
    assert any(v == pytest.approx(math.pi / 2) for v in values)
    units = set(graph.objects(None, QUDT_SCHEMA.unit))
    assert not units & FORBIDDEN_UNITS


def test_coordinates_tuple_scales_only_literal_elements(parse_mutated):
    decl = "position p-mixed = (5.0, <shared.spec.zero-linvel>, 0.0) cm"
    graph = _build(parse_mutated, SPEC_ANCHOR, decl)
    node = _quantity_node(graph, "p-mixed")
    x_value = graph.value(node, GEOM_COORD["x"])
    y_value = graph.value(node, GEOM_COORD["y"])
    assert float(x_value) == pytest.approx(0.05)
    assert str(y_value).endswith("zero-linvel")


DIMENSIONLESS_WRONG_UNIT = [
    pytest.param(
        "direction d-wrong { as-seen-by: <kinova.base_link.base_link_origin> } = (1.0, 0.0, 0.0) cm",
        "Direction",
        id="direction_in_metres",
    ),
    pytest.param(
        "path-parameter pp-wrong = 0.5 cm", "PathParameter", id="path_parameter_in_metres"
    ),
]


@pytest.mark.parametrize(("declaration", "message"), DIMENSIONLESS_WRONG_UNIT)
def test_dimensional_unit_on_dimensionless_quantity_is_rejected(
    parse_mutated, declaration, message
):
    with pytest.raises(TextXSemanticError, match=message):
        parse_mutated(SPEC_ANCHOR, f"{SPEC_ANCHOR},\n        {declaration}")
