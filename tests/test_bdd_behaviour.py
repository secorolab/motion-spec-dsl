# SPDX-License-Identifier: MPL-2.0
"""A model that serves BDD scenario goals states two nodes: the action a scenario sends its
goal to, whose one member is the event that goal produces, and the topic the exported events
leave by, whose members are those events.
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
    events   { <aas.E_CONTACT>, <aas.E_FORWARD_DONE> }
    events-channel: "/bdd/events"
}

"""


def _with(base_source: str, block: str) -> str:
    return base_source.replace(ANCHOR, block + ANCHOR, 1)


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


def test_the_exported_events_are_the_members_of_their_own_topic(parse_source, base_source):
    graph = _graph(parse_source, base_source)
    server = next(graph.subjects(RDF.type, ROS.Action))
    events = next(graph.subjects(ROS["type-name"], Literal("bdd_ros2_interfaces/msg/Event")))
    assert str(events) == f"{server}.events"
    assert (events, RDF.type, ROS.Topic) in graph
    assert str(graph.value(events, ROS["channel-name"])) == "/bdd/events"
    assert sorted(str(uri) for uri in graph.objects(events, RDFS.member)) == [
        f"{AAS}E_CONTACT",
        f"{AAS}E_FORWARD_DONE",
    ]


def test_a_model_without_the_block_states_no_server(parse_source, base_source):
    graph = MotionSpecDatasetBuilder(parse_source(base_source)).build()[0].default_graph
    assert not list(graph.subjects(RDF.type, ROS.Action))


def test_an_event_the_fsm_does_not_declare_is_rejected(parse_source, base_source):
    """A standalone event is monitor-owned: it never reaches the FSM, so it can export nothing."""
    block = BLOCK.replace("<aas.E_CONTACT>", "<E_INVENTED>", 1)
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
