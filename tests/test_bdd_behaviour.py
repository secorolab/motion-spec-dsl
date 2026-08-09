# SPDX-License-Identifier: MPL-2.0
"""A model that serves BDD scenario goals states the action a scenario sends its goal to, whose
one member is the event that goal produces. What a scenario observes is authored on the
monitors: a monitor that publishes an occurrence carries the event it triggers as its member.
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

BLOCK = """bdd-behaviour (ns=app) arc-behaviour {
    action:  "run_arc"
    on-goal: event <aas.E_HOME_SETTLED>
}

"""


TOPICS = """ros-topics (ns=app) {
    bdd-events: topic "/bdd/events" message "bdd_ros2_interfaces/msg/Event",
}

exec-context (ns=app) base-exec {"""

MONITOR = "satisfied for 0.3 s { trigger: event <aas.E_HOME_SETTLED> },"
OCCURRENCE = (
    "satisfied for 0.3 s { trigger: event <aas.E_HOME_SETTLED>, "
    "publish: event to <app.bdd-events> },"
)


def _with(base_source: str, block: str) -> str:
    return base_source.replace(ANCHOR, block + ANCHOR, 1)


def _with_occurrence(base_source: str) -> str:
    """The fixture's one monitor, publishing the event it triggers on a topic of its own."""
    return base_source.replace("exec-context (ns=app) base-exec {", TOPICS, 1).replace(
        MONITOR, OCCURRENCE, 1
    )


def _graph(parse_source, base_source: str, block: str = BLOCK):
    return (
        MotionSpecDatasetBuilder(parse_source(_with(base_source, block))).build()[0].default_graph
    )


def test_the_server_names_its_action_and_the_event_a_goal_produces(parse_source, base_source):
    graph = _graph(parse_source, base_source)
    server = next(graph.subjects(RDF.type, ROS.Action))
    assert str(server).endswith("arc-behaviour")
    assert str(graph.value(server, ROS["channel-name"])) == "run_arc"
    assert str(graph.value(server, ROS["type-name"])) == "bdd_ros2_interfaces/action/Behaviour"
    assert [str(uri) for uri in graph.objects(server, RDFS.member)] == [f"{AAS}E_HOME_SETTLED"]


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


def test_an_occurrence_without_a_trigger_is_rejected(parse_source, base_source):
    source = _with_occurrence(base_source).replace(
        "trigger: event <aas.E_HOME_SETTLED>, publish", "publish", 1
    )
    with pytest.raises(TextXSemanticError, match="triggers no event"):
        parse_source(source)


def test_an_occurrence_beside_authored_fields_is_rejected(parse_source, base_source):
    source = _with_occurrence(base_source).replace(
        OCCURRENCE,
        OCCURRENCE + '\n            violated { publish: to <app.bdd-events> { uri: "x" } },',
        1,
    )
    with pytest.raises(TextXSemanticError, match="both an occurrence and authored fields"):
        parse_source(source)


def test_a_model_without_the_block_states_no_server(parse_source, base_source):
    graph = MotionSpecDatasetBuilder(parse_source(base_source)).build()[0].default_graph
    assert not list(graph.subjects(RDF.type, ROS.Action))


def test_an_event_the_fsm_does_not_declare_is_rejected(parse_source, base_source):
    """A standalone event is monitor-owned: it never reaches the FSM, so no goal can start it."""
    block = BLOCK.replace("<aas.E_HOME_SETTLED>", "<E_INVENTED>", 1)
    with pytest.raises(TextXSemanticError, match="E_INVENTED"):
        parse_source(_with(base_source, block))


def test_a_goal_event_the_fsm_never_reacts_to_is_rejected(parse_source, base_source):
    """E_ARC_ENTERED is fired by a reaction but reacted to by none, so an accepted goal would
    start nothing."""
    block = BLOCK.replace("<aas.E_HOME_SETTLED>", "<aas.E_ARC_ENTERED>", 1)
    with pytest.raises(TextXSemanticError, match="declares no reaction to it"):
        parse_source(_with(base_source, block))


def test_a_second_block_is_rejected(parse_source, base_source):
    second = BLOCK.replace("arc-behaviour", "other-behaviour", 1)
    with pytest.raises(TextXSemanticError, match="2 'bdd-behaviour' blocks"):
        parse_source(_with(base_source, BLOCK + second))
