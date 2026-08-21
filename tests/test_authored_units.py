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


POSE_ANCHOR = "            pose home-pose = snapshot of <shared.world.pose-ee-base> on event <aas.E_HOME_ENTERED>"


def _pose_spec(position: str) -> str:
    return f"""{POSE_ANCHOR},
            pose lit-pose {{
                of:         <gripper.g_base.g_pinch>,
                wrt:        <kinova.base_link.base_link_origin>,
                as-seen-by: <kinova.base_link.base_link_origin>
            }} = {{
                position: {position} cm,
                orientation: euler {{ axes: xyz extrinsic, angles: (90.0, 0.0, 0.0) deg }}
            }}"""


@pytest.mark.parametrize(
    "position",
    ["(10.0, 20.0, 30.0)", "(<spec.zero-linvel>, 20.0, 30.0)"],
    ids=["all_literal", "mixed_ref"],
)
def test_a_pose_keeps_the_units_its_coordinates_were_written_in(parse_mutated, position):
    """A vector is no different from a scalar: 10 cm must not be labelled 10 metres,
    whether every axis is literal or one of them references another quantity."""
    model = parse_mutated(POSE_ANCHOR, _pose_spec(position))
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph
    position_node = next(s for s in graph.subjects() if str(s).endswith("lit-pose.position"))
    orientation_node = next(s for s in graph.subjects() if str(s).endswith("lit-pose.orientation"))
    assert graph.value(position_node, QUDT_SCHEMA.unit) == QUDT_UNIT["CentiM"]
    assert graph.value(orientation_node, QUDT_SCHEMA.unit) == QUDT_UNIT["DEG"]


TWO_SUBSPACE = [
    pytest.param(
        "velocity-twist tw = { angular-velocity: (1.0, 0.0, 0.0) deg/s, "
        "linear-velocity: (5.0, 0.0, 0.0) cm/s }",
        "tw",
        ("angular-velocity", "linear-velocity"),
        ("DEG-PER-SEC", "CentiM-PER-SEC"),
        id="velocity_twist",
    ),
    pytest.param(
        "acceleration-twist acc = { angular-acceleration: (1.0, 0.0, 0.0) deg/s^2, "
        "linear-acceleration: (1.0, 0.0, 0.0) m/s^2 }",
        "acc",
        ("angular-acceleration", "linear-acceleration"),
        ("DEG-PER-SEC2", "M-PER-SEC2"),
        id="acceleration_twist",
    ),
]


@pytest.mark.parametrize(("declaration", "name", "labels", "units"), TWO_SUBSPACE)
def test_a_twist_keeps_a_unit_per_subspace(parse_mutated, declaration, name, labels, units):
    """Each subspace carries its own authored unit, and the container's pair reports both
    -- a twist written in deg/s and cm/s is not a twist in rad/s and m/s."""
    graph = _build(parse_mutated, declaration)
    container = next(s for s in graph.subjects() if str(s).endswith(f"/{name}"))
    assert set(graph.objects(container, QUDT_SCHEMA.unit)) == {QUDT_UNIT[u] for u in units}
    for label, unit in zip(labels, units):
        node = next(s for s in graph.subjects() if str(s).endswith(f"{name}.{label}"))
        assert graph.value(node, QUDT_SCHEMA.unit) == QUDT_UNIT[unit]


def test_a_duration_carries_qudt_magnitude_because_owl_time_has_no_smaller_unit(parse_mutated):
    """`time:unitType` bottoms out at `time:unitSecond`, so owl-time's own magnitude
    properties could only record 500 ms by rescaling it. The class stays; the magnitude
    is a qudt Time-kind scalar that can say milliseconds."""
    graph = _build(parse_mutated, "duration du-ms = 500.0 ms")
    node = next(s for s in graph.subjects(RDF.type, TIME.Duration) if str(s).endswith("du-ms"))
    assert float(graph.value(node, QUDT_SCHEMA.value)) == pytest.approx(500.0)
    assert graph.value(node, QUDT_SCHEMA.unit) == QUDT_UNIT["MilliSEC"]
    assert graph.value(node, TIME.numericDuration) is None


def test_a_debounce_carries_the_unit_of_its_value(parse_mutated):
    """A value and its unit travel together, or 300 ms silently becomes 300 s."""
    graph = (
        MotionSpecDatasetBuilder(parse_mutated("satisfied for 0.3 s", "satisfied for 300.0 ms"))
        .build()[0]
        .default_graph
    )
    node = _quantity(graph, ".debounce")
    assert float(graph.value(node, QUDT_SCHEMA.value)) == pytest.approx(300.0)
    assert graph.value(node, QUDT_SCHEMA.unit) == QUDT_UNIT["MilliSEC"]
