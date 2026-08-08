# SPDX-License-Identifier: MPL-2.0
"""A detect act is one ros:Action node naming the scene objects it locates, and its goal status
is compared by an ordinary equality whose reference is the GoalStatus constant.
"""

from __future__ import annotations

import pytest
from motion_spec_dsl.rdf.model import CSTR, ROS
from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from rdflib.namespace import PROV, RDF, SOSA
from textx.exceptions import TextXSemanticError

ACTIONS = """ros-actions (ns=app) {
    locate: action "/perception/locate" type "aruco_perception/action/LocateObjects",
}

exec-context (ns=app) base-exec {"""

DETECT = "    find-ee: detect <gripper.g_base.g_pinch> using <app.locate>\n\n    when {}"

UNTIL = """until all {
        settled-z: <shared.world.twist-ee-base>.linvel.z equal to <shared.spec.zero-linvel> within <shared.spec.satisfied-band-vel>
    }"""
UNTIL_WITH_STATUS = """until all {
        settled-z: <shared.world.twist-ee-base>.linvel.z equal to <shared.spec.zero-linvel> within <shared.spec.satisfied-band-vel>,
        located: <find-ee>.status equal to succeeded
    }"""


def _graph(parse_source, base_source: str, *, detect: str = DETECT, until: str = UNTIL_WITH_STATUS):
    source = base_source.replace("exec-context (ns=app) base-exec {", ACTIONS, 1)
    source = source.replace("    when {}", detect, 1)
    source = source.replace(UNTIL, until, 1)

    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


def _act(graph):
    return next(graph.subjects(RDF.type, ROS.Action))


def test_the_act_names_its_channel_its_action_type_and_every_object_it_locates(
    parse_source, base_source
):
    graph = _graph(parse_source, base_source)
    act = _act(graph)
    assert str(graph.value(act, ROS["channel-name"])) == "/perception/locate"
    assert str(graph.value(act, ROS["type-name"])) == "aruco_perception/action/LocateObjects"
    assert len(list(graph.objects(act, SOSA.hasFeatureOfInterest))) == 1


def test_the_status_slot_is_derived_from_the_act_it_reports(parse_source, base_source):
    graph = _graph(parse_source, base_source)
    act = _act(graph)
    (status,) = list(graph.subjects(PROV.wasDerivedFrom, act))
    assert str(status) == f"{act}.status"


def test_the_goal_status_item_is_an_equality_on_the_status_slot(parse_source, base_source):
    """The operand is the act's status; the reference carries the GoalStatus constant."""
    graph = _graph(parse_source, base_source)
    (status,) = list(graph.subjects(PROV.wasDerivedFrom, _act(graph)))
    (constraint,) = list(graph.subjects(CSTR.quantity, status))
    assert CSTR.EqualityConstraint in set(graph.objects(constraint, RDF.type))
    reference = graph.value(constraint, CSTR["reference-value"])
    assert str(graph.value(reference, RDF.value)) == "STATUS_SUCCEEDED"


def test_a_target_no_world_pose_is_declared_of_is_rejected(parse_source, base_source):
    """The result is a pose: without a world pose `of:` the object, it has nowhere to land."""
    with pytest.raises(TextXSemanticError, match="nowhere to land"):
        _graph(
            parse_source,
            base_source,
            detect="    find-ee: detect <world_tree.table> using <app.locate>\n\n    when {}",
        )


def test_a_status_item_reading_another_motions_detect_is_rejected(parse_source, base_source):
    """A goal status is the outcome of an act this motion sends."""
    source = base_source.replace("exec-context (ns=app) base-exec {", ACTIONS, 1)
    source = source.replace(UNTIL, UNTIL_WITH_STATUS, 1)
    with pytest.raises(TextXSemanticError, match="does not declare"):
        MotionSpecDatasetBuilder(parse_source(source + _SECOND_MOTION)).build()


# A second motion owning the act, so the first motion's until reads a status it never sends.
_SECOND_MOTION = """
guarded-motion (ns=app) probe {
    context {
        spec {
            pose probe-pose = snapshot of <shared.world.pose-ee-base>
        }
    }

    find-ee: detect <gripper.g_base.g_pinch> using <app.locate>

    when {}

    while {
        hold-position: keeping <shared.world.pose-ee-base>.position equal to <spec.probe-pose>.position within <shared.spec.satisfied-band>
    }

    until {}
}

constraint-handler (ns=app) handler-probe {
    handles: <probe>
    controllers {
        pid ctrl-probe-hold { constraint: <probe.hold-position>, Kp: 200, Ki: 100, Kd: 40, decay: 0 }
    }
    solvers {
        <handler-home.arm-solver>
    }
}
"""
