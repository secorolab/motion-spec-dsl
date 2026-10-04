# SPDX-License-Identifier: MPL-2.0
"""The `ros` block: served goals and their answers, publishes, subscriptions, and detect acts."""

from __future__ import annotations

from motion_spec_dsl.langs import motion_spec_metamodel
from motion_spec_dsl.rdf.model import CSTR, ROS
from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import GEOM_REL, QUDT_SCHEMA, SENSORS
from rdflib import Literal
from rdflib.namespace import PROV, RDF, RDFS, SOSA

from support import (
    ACTIONS,
    BASE,
    BASE_TEXT,
    EXEC,
    LOCATED,
    MODELS,
    MONITOR,
    MOTION,
    OCCURRENCE,
    SERVER,
    TOPICS,
    TWIST,
    UNTIL,
)


def test_the_answer_states_its_status_and_its_result_fields() -> None:
    source = BASE_TEXT.replace(MOTION, SERVER + MOTION, 1).replace(
        "trigger: event <aas.E_HOME_SETTLED>",
        "trigger: event <aas.E_HOME_SETTLED>, "
        "result: succeeded <ros.action-servers.arc-behaviour> { position: 0.5 }",
        1,
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph

    answer = next(s for s in graph.subjects(RDF.type, ROS.Action) if str(s).endswith(".answer"))
    members = {
        (str(graph.value(m, ROS["field-path"])), str(graph.value(m, RDF.value)))
        for m in graph.objects(answer, RDFS.member)
    }
    assert members == {("None", "STATUS_SUCCEEDED"), ("position", "0.5")}


def test_a_monitor_publishes_its_event_as_its_topics_member() -> None:
    """A member with no value is how the graph says occurrence rather than payload."""
    source = BASE_TEXT.replace(EXEC, TOPICS + EXEC, 1).replace(MONITOR, OCCURRENCE, 1)
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph

    topic = next(graph.subjects(ROS["channel-name"], Literal("/events")))
    (member,) = graph.objects(topic, RDFS.member)
    assert str(member).endswith("/E_HOME_SETTLED")
    assert graph.value(member, RDF.value) is None


def test_a_standing_publish_reports_its_quantity_at_its_rate() -> None:
    block = """ros (ns=app) {
    publishers {
        ee: topic "/ee" message "geometry_msgs/msg/PoseStamped",
    },
    always {
        publish at 10.0 Hz to <ros.publishers.ee> with <shared.world.pose-ee-base>,
    },
}

"""
    source = BASE_TEXT.replace(EXEC, block + EXEC, 1)
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph

    topic = next(graph.subjects(ROS["channel-name"], Literal("/ee")))
    [row] = graph.objects(topic, RDFS.member)
    assert str(graph.value(row, RDF.value)).endswith("/pose-ee-base")
    assert float(graph.value(graph.value(topic, SENSORS["update-rate"]), QUDT_SCHEMA.value)) == 10.0


def test_a_detect_act_locates_its_object_and_its_status_is_an_equality() -> None:
    source = (
        BASE_TEXT.replace(EXEC, ACTIONS + EXEC, 1)
        .replace(
            "    when {}",
            "    find-ee: detect <gripper.g_base.g_pinch> using <ros.action-clients.locate>\n\n    when {}",
            1,
        )
        .replace(UNTIL, LOCATED, 1)
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph

    act = next(graph.subjects(RDF.type, ROS.Action))
    assert str(graph.value(act, ROS["type-name"])) == "aruco_perception/action/LocateObjects"
    assert len(list(graph.objects(act, SOSA.hasFeatureOfInterest))) == 1
    (status,) = graph.subjects(PROV.wasDerivedFrom, act)
    (constraint,) = graph.subjects(CSTR.quantity, status)
    assert (constraint, RDF.type, CSTR.EqualityConstraint) in graph
    reference = graph.value(constraint, CSTR["reference-value"])
    assert str(graph.value(reference, RDF.value)) == "STATUS_SUCCEEDED"


def test_a_pose_subscription_states_the_pose_it_reads() -> None:
    subscribers = """ros (ns=app) {
    subscribers {
        table-top: topic "/recognized_objects" message "vision_msgs/msg/Detection3DArray" {
            observes { <shared.world.pose-table-cam> }
            pose from results {
                of:  <table.table_top>,
                wrt: <ft_tree.wrist_ft_body.wrist_ft_site>,
            }
        },
    },
}

"""
    table_in_wrist = """,
        pose pose-table-cam {
            of:         <table.table_top>,
            wrt:        <ft_tree.wrist_ft_body.wrist_ft_site>,
            as-seen-by: <ft_tree.wrist_ft_body.wrist_ft_site>
        }"""
    source = BASE_TEXT.replace(EXEC, subscribers + EXEC, 1).replace(TWIST, TWIST + table_in_wrist, 1)
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph

    topic = next(graph.subjects(ROS["channel-name"], Literal("/recognized_objects")))
    assert str(graph.value(topic, ROS["field-path"])) == "results.pose"
    assert str(graph.value(topic, SOSA.hasFeatureOfInterest)).endswith("/pose-table-cam")
    observed = graph.value(topic, SOSA.observedProperty)
    assert str(graph.value(observed, GEOM_REL.of)).endswith("/table_top")


def test_a_camera_subscription_reads_the_scenes_camera() -> None:
    """An image has no pose to place, so the channel names the camera and no field path."""
    robmot = MODELS / "01_pick_and_place" / "pick_and_place.robmot"
    subscribers = """ros (ns=app) {
    subscribers {
        wrist-image: topic "/wrist/image" message "sensor_msgs/msg/Image" {
            observes { <wrist> }
        },
    },
}

"""
    source = robmot.read_text().replace("exec-context (ns=app)", subscribers + "exec-context (ns=app)", 1)
    model = motion_spec_metamodel().model_from_str(source, file_name=str(robmot))
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph

    topic = next(graph.subjects(ROS["channel-name"], Literal("/wrist/image")))
    assert str(graph.value(topic, SOSA.hasFeatureOfInterest)).endswith("/wrist")
    assert graph.value(topic, ROS["field-path"]) is None
