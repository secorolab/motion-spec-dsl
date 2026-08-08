# SPDX-License-Identifier: MPL-2.0
"""Topic declarations resolve by namespace, and a publishing monitor states the fields it
writes: one row per assignment, each under the condition its state block holds.
"""

from __future__ import annotations

import pytest
from motion_spec_dsl.rdf.model import CSTR, CSTR_EXT, CSTR_HDL, ROS
from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from rdflib.namespace import RDF, RDFS
from textx.exceptions import TextXSemanticError, TextXSyntaxError

TOPICS = """ros-topics (ns=app) {
    settled: topic "/base/settled" message "std_msgs/msg/Float64",
    other: topic "/base/other" message "std_msgs/msg/Float64",
}

exec-context (ns=app) base-exec {"""

MONITOR = "satisfied for 0.3 s { trigger: event <aas.E_HOME_SETTLED> },"

# An order relation to monitor: only its complement is expressible, so only it may be
# published on violated.
UNTIL = """until all {
        settled-z: <shared.world.twist-ee-base>.linvel.z equal to <shared.spec.zero-linvel> within <shared.spec.satisfied-band-vel>
    }"""
ORDERED_UNTIL = """until all {
        risen-z: <shared.world.twist-ee-base>.linvel.z greater than <shared.spec.zero-linvel>
    }"""

BOTH_STATES = """satisfied { publish: TRUE to <app.settled> },
        violated { publish: FALSE to <app.settled> },"""
BLOCK_FORM = """satisfied { publish: to <app.settled> { data: 1.0 } },
        violated { publish: to <app.settled> { data: 0.0 } },"""


def _graph(parse_source, base_source: str, monitor: str, *, target: str = "home.until"):
    source = base_source.replace("exec-context (ns=app) base-exec {", TOPICS, 1)
    source = source.replace(UNTIL, ORDERED_UNTIL, 1)
    source = source.replace("monitor <home.until>", f"monitor <{target}>", 1)
    source = source.replace(MONITOR, monitor, 1)

    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


def _rows(graph):
    """Every publish row, as `(field-path, value, condition)`, in authored order."""
    monitor = next(graph.subjects(RDF.type, ROS.Topic))
    return [
        (
            str(graph.value(row, ROS["field-path"])),
            str(graph.value(row, RDF.value)),
            graph.value(row, CSTR_EXT["has-constraint"]),
        )
        for row in sorted(graph.objects(monitor, RDFS.member))
    ]


def test_the_sugar_states_one_row_per_state_under_its_own_condition(parse_source, base_source):
    """Satisfied points at the watched constraint; violated at the complement minted beside the
    monitor. The sugar leaves the field for the message type to resolve."""
    graph = _graph(parse_source, base_source, BOTH_STATES, target="home.risen-z")
    monitor = next(graph.subjects(RDF.type, ROS.Topic))
    assert str(graph.value(monitor, ROS["channel-name"])) == "/base/settled"
    assert str(graph.value(monitor, ROS["type-name"])) == "std_msgs/msg/Float64"

    (satisfied, violated) = _rows(graph)
    assert satisfied[:2] == ("", "TRUE")
    assert violated[:2] == ("", "FALSE")
    assert satisfied[2] == graph.value(monitor, CSTR_HDL["constraint"])
    assert str(violated[2]).endswith("mon-home-settled.complement")
    # The complement states the opposite order relation over the same quantity and threshold.
    assert CSTR.LessThanConstraint in set(graph.objects(violated[2], RDF.type))
    assert graph.value(violated[2], CSTR["threshold"]) == graph.value(
        satisfied[2], CSTR["threshold"]
    )


def test_the_block_form_states_the_field_it_writes(parse_source, base_source):
    graph = _graph(parse_source, base_source, BLOCK_FORM, target="home.risen-z")
    assert [(path, value) for path, value, _c in _rows(graph)] == [
        ("data", "1.0"),
        ("data", "0.0"),
    ]


def test_a_satisfied_only_publish_is_legal(parse_source, base_source):
    """TRUE while satisfied, silence otherwise -- no complement is needed, so a conjunction may
    be watched."""
    graph = _graph(parse_source, base_source, "satisfied { publish: TRUE to <app.settled> },")
    monitor = next(graph.subjects(RDF.type, ROS.Topic))
    (row,) = _rows(graph)
    assert row[:2] == ("", "TRUE")
    assert row[2] in set(graph.objects(monitor, CSTR_HDL["constraint"]))
    assert not [node for node in graph.subjects() if str(node).endswith(".complement")]


def test_publishing_when_violated_needs_a_single_constraint(parse_source, base_source):
    with pytest.raises(TextXSemanticError, match="complement of a conjunction"):
        _graph(parse_source, base_source, BOTH_STATES)


def test_every_state_publishes_the_same_channel(parse_source, base_source):
    with pytest.raises(TextXSemanticError, match="more than one topic"):
        _graph(
            parse_source,
            base_source,
            """satisfied { publish: TRUE to <app.settled> },
        violated { publish: FALSE to <app.other> },""",
            target="home.risen-z",
        )


def test_a_state_publishes_once(parse_source, base_source):
    with pytest.raises(TextXSemanticError, match="more than one 'publish' in one state"):
        _graph(
            parse_source,
            base_source,
            """satisfied {
            publish: TRUE to <app.settled>,
            publish: TRUE to <app.settled>,
        },""",
        )


def test_the_monitor_level_publish_is_gone(parse_source, base_source):
    """A publish belongs to a state block; there is no monitor-wide form to fall back on."""
    with pytest.raises(TextXSyntaxError):
        _graph(parse_source, base_source, "publish: to <app.settled>,")
