# SPDX-License-Identifier: MPL-2.0
"""One `ros` block states what a model publishes, calls and serves. A served action carries the
event an accepted goal produces as its valueless member, and what a completed run answers with as
its field rows. What a scenario observes while the run plays out is authored on the monitors: a
monitor that publishes an occurrence carries the event it triggers as its member.
"""

from __future__ import annotations

import pytest
from motion_spec_dsl.rdf.model import ROS
from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from rdflib import Literal
from rdflib.namespace import RDF, RDFS
from textx.exceptions import TextXSemanticError

AAS = "https://secorolab.github.io/models/admittance-arc-single/fsm/"
ANCHOR = "guarded-motion (ns=app) home {"

SERVER = """ros (ns=app) {
    action-servers {
        arc-behaviour: action "run_arc" type "bdd_ros2_interfaces/action/Behaviour" {
            on-goal: produce event <aas.E_HOME_SETTLED>,
            on-end: { result.trinary.value: TRUE },
        },
    },
}

"""


TOPICS = """ros (ns=app) {
    publishers {
        bdd-events: topic "/bdd/events" message "bdd_ros2_interfaces/msg/Event",
    },
}

exec-context (ns=app) base-exec {"""

MONITOR = "satisfied for 0.3 s { trigger: event <aas.E_HOME_SETTLED> },"
OCCURRENCE = (
    "satisfied for 0.3 s { trigger: event <aas.E_HOME_SETTLED>, "
    "publish: events { <aas.E_HOME_SETTLED> } to <ros.publishers.bdd-events> },"
)


def _with(base_source: str, block: str) -> str:
    return base_source.replace(ANCHOR, block + ANCHOR, 1)


def _with_occurrence(base_source: str) -> str:
    """The fixture's one monitor, publishing the event it triggers on a topic of its own."""
    return base_source.replace("exec-context (ns=app) base-exec {", TOPICS, 1).replace(
        MONITOR, OCCURRENCE, 1
    )


def _graph(parse_source, base_source: str, block: str = SERVER):
    return (
        MotionSpecDatasetBuilder(parse_source(_with(base_source, block))).build()[0].default_graph
    )


def test_the_server_names_its_action_and_the_event_a_goal_produces(parse_source, base_source):
    graph = _graph(parse_source, base_source)
    server = next(graph.subjects(RDF.type, ROS.Action))
    assert str(server).endswith("arc-behaviour")
    assert str(graph.value(server, ROS["channel-name"])) == "run_arc"
    assert str(graph.value(server, ROS["type-name"])) == "bdd_ros2_interfaces/action/Behaviour"
    members = list(graph.objects(server, RDFS.member))
    goal_event = [m for m in members if graph.value(m, RDF.value) is None]
    assert [str(uri) for uri in goal_event] == [f"{AAS}E_HOME_SETTLED"]


def test_the_server_states_what_a_completed_run_answers_with(parse_source, base_source):
    """A row is a field the result carries; a run that never finishes states none of them."""
    graph = _graph(parse_source, base_source)
    server = next(graph.subjects(RDF.type, ROS.Action))
    rows = [row for row in graph.objects(server, RDFS.member) if graph.value(row, RDF.value)]
    assert [
        (str(graph.value(row, ROS["field-path"])), str(graph.value(row, RDF.value))) for row in rows
    ] == [("result.trinary.value", "TRUE")]


def test_a_server_that_authors_no_result_states_no_rows(parse_source, base_source):
    block = SERVER.replace("\n            on-end: { result.trinary.value: TRUE },", "", 1)
    graph = _graph(parse_source, base_source, block)
    server = next(graph.subjects(RDF.type, ROS.Action))
    assert not [row for row in graph.objects(server, RDFS.member) if graph.value(row, RDF.value)]


def test_a_monitor_publishes_its_event_as_its_topics_member(parse_source, base_source):
    """The occurrence authors no field row: its one member is the event, and a member with no
    value is how the graph says occurrence rather than payload."""
    graph = MotionSpecDatasetBuilder(parse_source(_with_occurrence(base_source))).build()[0]
    graph = graph.default_graph
    monitor = next(graph.subjects(ROS["type-name"], Literal("bdd_ros2_interfaces/msg/Event")))
    assert (monitor, RDF.type, ROS.Topic) in graph
    assert str(graph.value(monitor, ROS["channel-name"])) == "/bdd/events"
    (member,) = graph.objects(monitor, RDFS.member)
    assert str(member) == f"{AAS}E_HOME_SETTLED"
    assert graph.value(member, RDF.value) is None


def test_a_monitor_announces_events_it_does_not_trigger(parse_source, base_source):
    """The set is the author's choice, so it is not tied to what this monitor fires -- each
    named event becomes one member of the topic."""
    source = _with_occurrence(base_source).replace(
        "publish: events { <aas.E_HOME_SETTLED> }",
        "publish: events { <aas.E_HOME_SETTLED>, <aas.E_CONTACT> }",
        1,
    )
    graph = MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph
    monitor = next(graph.subjects(ROS["type-name"], Literal("bdd_ros2_interfaces/msg/Event")))
    assert sorted(str(member) for member in graph.objects(monitor, RDFS.member)) == [
        f"{AAS}E_CONTACT",
        f"{AAS}E_HOME_SETTLED",
    ]


def test_announcing_without_a_trigger_is_legal(parse_source, base_source):
    """An announced event rides the FSM, so a monitor may report one it never fires itself."""
    source = _with_occurrence(base_source).replace(
        "trigger: event <aas.E_HOME_SETTLED>, publish", "publish", 1
    )
    graph = MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph
    monitor = next(graph.subjects(ROS["type-name"], Literal("bdd_ros2_interfaces/msg/Event")))
    assert [str(member) for member in graph.objects(monitor, RDFS.member)] == [
        f"{AAS}E_HOME_SETTLED"
    ]


def test_the_same_event_announced_twice_is_rejected(parse_source, base_source):
    source = _with_occurrence(base_source).replace(
        "publish: events { <aas.E_HOME_SETTLED> }",
        "publish: events { <aas.E_HOME_SETTLED>, <aas.E_HOME_SETTLED> }",
        1,
    )
    with pytest.raises(TextXSemanticError, match="more than once"):
        parse_source(source)


def test_an_occurrence_beside_authored_fields_is_rejected(parse_source, base_source):
    source = _with_occurrence(base_source).replace(
        OCCURRENCE,
        OCCURRENCE
        + '\n            violated { publish: to <ros.publishers.bdd-events> { uri: "x" } },',
        1,
    )
    with pytest.raises(TextXSemanticError, match="both events and authored fields"):
        parse_source(source)


def test_a_model_without_the_block_states_no_server(parse_source, base_source):
    graph = MotionSpecDatasetBuilder(parse_source(base_source)).build()[0].default_graph
    assert not list(graph.subjects(RDF.type, ROS.Action))


def test_an_event_the_fsm_does_not_declare_is_rejected(parse_source, base_source):
    """A standalone event is monitor-owned: it never reaches the FSM, so no goal can start it."""
    block = SERVER.replace("<aas.E_HOME_SETTLED>", "<E_INVENTED>", 1)
    with pytest.raises(TextXSemanticError, match="E_INVENTED"):
        parse_source(_with(base_source, block))


def test_a_goal_event_the_fsm_never_reacts_to_is_rejected(parse_source, base_source):
    """E_ARC_ENTERED is fired by a reaction but reacted to by none, so an accepted goal would
    start nothing."""
    block = SERVER.replace("<aas.E_HOME_SETTLED>", "<aas.E_ARC_ENTERED>", 1)
    with pytest.raises(TextXSemanticError, match="declares no reaction to it"):
        parse_source(_with(base_source, block))


def test_a_second_ros_block_is_rejected(parse_source, base_source):
    second = SERVER.replace("arc-behaviour", "other-behaviour", 1)
    with pytest.raises(TextXSemanticError, match="2 'ros' blocks"):
        parse_source(_with(base_source, SERVER + second))


def test_a_second_served_action_is_rejected(parse_source, base_source):
    """One runtime answers one action: a second server has no second FSM to start."""
    block = SERVER.replace(
        "        },\n    },",
        """        },
        other-behaviour: action "run_other" type "bdd_ros2_interfaces/action/Behaviour" {
            on-goal: produce event <aas.E_HOME_SETTLED>,
        },
    },""",
        1,
    )
    with pytest.raises(TextXSemanticError, match="serves 2 actions"):
        parse_source(_with(base_source, block))
