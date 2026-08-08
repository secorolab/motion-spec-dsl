# SPDX-License-Identifier: MPL-2.0
"""Topic declarations resolve by namespace, and a publishing monitor is typed `ros:Topic`."""

from __future__ import annotations

import pytest
from motion_spec_dsl.rdf.model import ROS
from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from rdflib.namespace import RDF
from textx.exceptions import TextXSemanticError

TOPICS = """ros-topics (ns=app) {
    settled: topic "/base/settled" message "std_msgs/msg/Float64",
    other: topic "/base/other" message "std_msgs/msg/Float64",
}

exec-context (ns=app) base-exec {"""

MONITOR = "satisfied for 0.3 s { trigger: event <aas.E_HOME_SETTLED> },"


def _graph(parse_source, base_source: str, monitor: str):
    source = base_source.replace("exec-context (ns=app) base-exec {", TOPICS, 1)
    source = source.replace(MONITOR, monitor, 1)

    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


def test_a_publishing_monitor_carries_only_the_channel_and_the_type(parse_source, base_source):
    """The graph states where the monitor publishes and what it carries -- nothing else. What
    goes into the message is the message type's contract, fixed in codegen.
    """
    graph = _graph(
        parse_source,
        base_source,
        """satisfied for 0.3 s { trigger: event <aas.E_HOME_SETTLED> },
        publish: to <app.settled>,""",
    )
    monitor = next(graph.subjects(RDF.type, ROS.Topic))
    assert str(graph.value(monitor, ROS["channel-name"])) == "/base/settled"
    assert str(graph.value(monitor, ROS["type-name"])) == "std_msgs/msg/Float64"
    ros_predicates = {p for _s, p, _o in graph if str(p).startswith(str(ROS))}
    assert ros_predicates == {ROS["channel-name"], ROS["type-name"]}


def test_publish_only_monitor_stays_a_plain_monitor(parse_source, base_source):
    """No trigger, no flag: the monitor is neither edge- nor level-typed, but still a topic."""
    graph = _graph(parse_source, base_source, "publish: to <app.settled>,")
    monitor = next(graph.subjects(RDF.type, ROS.Topic))
    types = set(graph.objects(monitor, RDF.type))
    assert ROS.Topic in types
    assert not [t for t in types if "Triggered" in str(t)]


def test_a_monitor_publishes_once(parse_source, base_source):
    with pytest.raises(TextXSemanticError, match="more than one 'publish'"):
        _graph(
            parse_source,
            base_source,
            """publish: to <app.settled>,
        publish: to <app.other>,""",
        )
