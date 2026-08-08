# SPDX-License-Identifier: MPL-2.0
"""Topic declarations resolve by namespace, and a monitor's publish actions become field nodes."""

from __future__ import annotations

import pytest
from motion_spec_dsl.rdf.model import ROS
from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from rdflib import URIRef
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


def test_publish_actions_become_field_nodes(parse_source, base_source):
    """Each (state, field) pair is its own node, carrying the state it is published in and
    either the authored value or the quantity it is read from."""
    graph = _graph(
        parse_source,
        base_source,
        """satisfied for 0.3 s {
            trigger: event <aas.E_HOME_SETTLED>,
            publish: to <app.settled> { data: 1.0 },
        },
        violated { publish: to <app.settled> { data: <shared.spec.zero-linvel> } },
        inactive { publish: 0.0 to <app.settled> },""",
    )
    monitor = next(graph.subjects(RDF.type, ROS.Topic))
    assert str(graph.value(monitor, ROS["channel-name"])) == "/base/settled"
    assert str(graph.value(monitor, ROS["type-name"])) == "std_msgs/msg/Float64"

    fields = {
        str(graph.value(node, ROS["publish-on"])): (
            str(graph.value(node, ROS["field-path"])),
            str(graph.value(node, ROS["value"]) or ""),
            graph.value(node, ROS["value-from"]),
        )
        for node in graph.objects(monitor, ROS["field"])
    }
    assert fields["satisfied"] == ("data", "1.0", None)
    # A context quantity is named, never restated as a literal.
    assert fields["violated"][0] == "data" and not fields["violated"][1]
    assert fields["violated"][2] == URIRef(
        "https://secorolab.github.io/models/base/shared/spec/zero-linvel"
    )
    # The sugar form states no path: only the message shape can resolve which field it means.
    assert fields["inactive"] == ("", "0.0", None)


def test_a_monitor_publishes_to_one_topic(parse_source, base_source):
    with pytest.raises(TextXSemanticError, match="more than one topic"):
        _graph(
            parse_source,
            base_source,
            """satisfied { publish: 1.0 to <app.settled> },
        violated { publish: 0.0 to <app.other> },""",
        )
