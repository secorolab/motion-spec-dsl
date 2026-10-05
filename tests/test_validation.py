# SPDX-License-Identifier: MPL-2.0
"""One rejection per concept: an edit to a valid model that validation must refuse."""

from __future__ import annotations

import pytest
from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT
from rdflib.namespace import RDF
from textx.exceptions import TextXError

from motion_spec_dsl.langs import motion_spec_metamodel
from motion_spec_dsl.rdf.dataset import build_dataset
from motion_spec_dsl.rdf_parser.vocab import AGN, EST, QUDT_SCHEMA
from support import (
    ACTIONS,
    BASE,
    CTRL,
    EXEC,
    HOLD,
    LOCATED,
    MODELS,
    MONITOR,
    MOTION,
    OCCURRENCE,
    PERTURBATION,
    POSE_TABLE_TOP,
    PUSH,
    SERVER,
    SIM,
    SNAPSHOT,
    SOLVERS_END,
    SPEC,
    TABLE_PLANE,
    TOPICS,
    TWIST,
    UNTIL,
)

PICK = MODELS / "01_pick_and_place" / "pick_and_place.robmot"
ARC = MODELS / "06_arc_tracing_with_admittance" / "arc_tracing_with_admittance.robmot"
REAL = 'platform:   real-world { <agents.arm1> realized by KinovaGen3-2F85 }'
ESTIMATED = """,
        wrench press-wrench {
            ref-point:      <ft_tree.wrist_ft_body.wrist_ft_site>,
            as-seen-by:     <kinova.base_link.base_link_origin>,
            estimated-from: <agents.arm1> { gain: 30.0 Hz, filter: 0.5 }%s
        }"""
MEASURED = """,
        wrench press-wrench {
            ref-point:  <ft_tree.wrist_ft_body.wrist_ft_site>,
            as-seen-by: <kinova.base_link.base_link_origin>,
            ft-sensor:  <wrist_ft>,
            re-tare-on: { <aas.E_HOME_SETTLED> }
        }"""
PRESS_WRENCH = """,
        wrench press-wrench {
            of:         <gripper.g_base.g_pinch>,
            ref-point:  <gripper.g_base.g_pinch>,
            as-seen-by: <kinova.base_link.base_link_origin>
        }"""


AFTER_SPEC = f"{SPEC},\n        "
AFTER_HOLD = f"{HOLD},\n        "
AFTER_CTRL = f"{CTRL},\n        "


REJECTIONS = [
    pytest.param(BASE, [("settled-z:", "hold-position:")], "hold-position", id="constraint_name_reused"),
    pytest.param(
        BASE,
        [('models/base/"', 'models/base"')],
        "must end with",
        id="namespace_without_separator",
    ),
    pytest.param(
        BASE, [(MONITOR, "violated { flag: settled },")], "belongs in satisfied", id="monitor_flag"
    ),
    pytest.param(BASE, [(SIM, REAL)], "no 'config'", id="device_binding_without_config"),
    pytest.param(
        BASE,
        [
            (SPEC, AFTER_SPEC + "pose look-at-table = [config.poses.table] for <shared.world.pose-ee-base>"),
            ("<spec.home-pose>.position within", "<shared.spec.look-at-table>.position within"),
        ],
        "declares no",
        id="config_pose_without_config",
    ),
    pytest.param(
        ARC,
        [("deadband: 1.0 N\n", "deadband: 1.0 N,\n                release-threshold: 2.0 N\n")],
        "release-threshold 2.0 must not exceed deadband 1.0",
        id="admittance_release_above_deadband",
    ),
    pytest.param(
        BASE,
        [
            (TWIST, TWIST + MEASURED),
            (SPEC, AFTER_SPEC + "force press-force = -5.0 N,\n        force satisfied-band-force = 0.5 N"),
            (HOLD, AFTER_HOLD + 
                "press-down: keeping <shared.world.press-wrench>.force.z equal to "
                "<shared.spec.press-force> within <shared.spec.satisfied-band-force>"
            ),
            (CTRL, AFTER_CTRL + 
                "feed-forward ctrl-press-down { constraint: <home.press-down> }"
                " as force apply at <gripper.g_base>"
            ),
        ],
        "'wrist_ft' measures or estimates it",
        id="sensed_wrench_commanded",
    ),
    pytest.param(
        BASE, [(TWIST, TWIST + ESTIMATED % "")], "names no re-tare-on", id="estimate_never_retared"
    ),
    pytest.param(
        BASE,
        [(MOTION, SERVER + SERVER.replace("arc-behaviour", "other-behaviour") + MOTION)],
        "2 'ros' blocks",
        id="second_ros_block",
    ),
    pytest.param(
        BASE,
        [(MOTION, SERVER.replace("<aas.E_HOME_SETTLED>", "<aas.E_ARC_ENTERED>") + MOTION)],
        "declares no reaction to it",
        id="goal_event_nothing_reacts_to",
    ),
    pytest.param(
        BASE,
        [
            (MOTION, SERVER + MOTION),
            (
                "trigger: event <aas.E_HOME_SETTLED>",
                "trigger: event <aas.E_HOME_SETTLED>, "
                "result: canceled <ros.action-servers.arc-behaviour> { position: 0.5 }",
            ),
        ],
        "a cancel is the client's",
        id="server_cancels_its_own_goal",
    ),
    pytest.param(
        BASE,
        [(EXEC, TOPICS + EXEC), (MONITOR, "violated { publish: FALSE to <ros.publishers.settled> },")],
        "author the satisfied publish too",
        id="violated_only_publish",
    ),
    pytest.param(
        BASE,
        [
            (EXEC, TOPICS + EXEC),
            (MONITOR, OCCURRENCE.replace("<aas.E_HOME_SETTLED> }", "<aas.E_HOME_SETTLED>, <aas.E_HOME_SETTLED> }")),
        ],
        "more than once",
        id="event_announced_twice",
    ),
    pytest.param(
        BASE,
        [
            (EXEC, ACTIONS + EXEC),
            ("    when {}", "    find-ee: detect <world_tree.table> using <ros.action-clients.locate>\n\n    when {}"),
            (UNTIL, LOCATED),
        ],
        "nowhere to land",
        id="detect_target_without_world_pose",
    ),
    pytest.param(
        BASE,
        [(SPEC, AFTER_SPEC + "length sum-bad = <shared.spec.satisfied-band> + <shared.world.twist-ee-base>.linvel.z")],
        "different kinds of quantity",
        id="expression_adds_two_kinds",
    ),
    pytest.param(
        BASE,
        [(SPEC, AFTER_SPEC + "length bad-mul = <shared.world.pose-ee-base>.position * 2.0 1")],
        "geometry kind",
        id="expression_multiplies_a_geometry_kind",
    ),
    pytest.param(
        BASE,
        [
            (TWIST, TWIST + PRESS_WRENCH),
            (SPEC, AFTER_SPEC + "force f-lo = -1.0 N,\n        force f-hi = 1.0 N,\n        length arm = 1.0 m"),
            (
                HOLD,
                "hold-position: keeping (<shared.world.press-wrench>.force.z * <spec.arm> / "
                "<shared.world.pose-ee-base>.position.x) outside <spec.f-lo> and <spec.f-hi>",
            ),
        ],
        "divides by a term the motion moves",
        id="controlled_expression_dividing_by_a_moved_term",
    ),
    pytest.param(BASE, [(SPEC, AFTER_SPEC + "force zero-force = (0.0, 0.0, 0.0) m")], "Force", id="unit_of_wrong_kind"),
    pytest.param(
        BASE,
        [
            (SPEC, AFTER_SPEC + 
                "pose test-pose { of: <gripper.g_base.g_pinch>, wrt: <kinova.base_link.base_link_origin>,"
                " as-seen-by: <kinova.base_link.base_link_origin> } = { position: (0.1, 0.2, 0.3) m,"
                " orientation: quat { xyzw: (0.0, 0.0, 0.0) } }"
            )
        ],
        "quaternion 'xyzw' has 3 components -- it takes 4",
        id="orientation_arity",
    ),
    pytest.param(
        BASE, [(SPEC, AFTER_SPEC + "velocity-twist vt = (0.0, 0.0, 0.0) rad/s")], "two-subspace", id="bare_twist_vector"
    ),
    pytest.param(
        BASE,
        [
            (
                SNAPSHOT,
                f"{SNAPSHOT},\n            pose turned-pose = {{ position: (0.1, 0.2, 0.3) m,"
                " orientation: <spec.home-pose>.orientation rotated by quat { xyzw: (0.0, 0.0, 0.0, 1.0) } }",
            )
        ],
        "states no basis frame",
        id="relative_quaternion_without_basis",
    ),
    pytest.param(
        BASE,
        [
            (SPEC, AFTER_SPEC + 
                "direction to-table { as-seen-by: <kinova.base_link.base_link_origin> } ="
                " from <kinova.base_link.base_link_origin> to <table.table_top>"
            ),
            (HOLD, AFTER_HOLD + 
                "speed: norm of <shared.world.twist-ee-base>.linvel across <shared.spec.to-table>"
                " greater than 0.05 m/s"
            ),
        ],
        "does not declare",
        id="direction_between_unrelated_frames",
    ),
    pytest.param(
        BASE,
        [
            (
                TWIST,
                TWIST + ",\n        pose pose-ee-table {\n"
                "            of:         <gripper.g_base.g_pinch>,\n"
                "            wrt:        <table.table_top>,\n"
                "            as-seen-by: <gripper.g_base.g_pinch>\n        }",
            ),
            (SPEC, AFTER_SPEC +
                "direction tool-up { as-seen-by: <gripper.g_base.g_pinch> } = (0, 0, -1),\n"
                "        direction table-up { as-seen-by: <table.table_top> } = (0, 0, 1),\n"
                "        angle align-band = 0.05 rad"
            ),
            (HOLD, AFTER_HOLD +
                "align: keeping angle between <shared.spec.tool-up> and <shared.spec.table-up>"
                " equal to 0 rad within <shared.spec.align-band>"
            ),
            (CTRL, AFTER_CTRL + "pid ctrl-align { constraint: <home.align>, Kp: 120, Ki: 50, Kd: 80, decay: 0 }"),
        ],
        "answers in the frame the pose is seen by",
        id="angle_reads_a_pose_seen_by_another_frame",
    ),
    pytest.param(
        PICK,
        [
            (
                "velocity-twist twist-ee-base {\n            of:         <gripper.g_base.g_pinch>,",
                "velocity-twist twist-ee-base {\n            of:         <pick_and_place_graph.cube>,",
            )
        ],
        "declares no velocity-twist",
        id="path_speed_without_its_twist",
    ),
    pytest.param(
        BASE,
        [(HOLD, "hold-position: keeping <shared.world.pose-ee-base> equal to <spec.home-pose> within <shared.spec.satisfied-band>")],
        "band on a whole pose",
        id="band_on_a_whole_pose",
    ),
    pytest.param(
        BASE,
        [(HOLD, "hold-position: keeping <spec.home-pose> equal to <spec.home-pose>")],
        "constrains a whole pose",
        id="constraint_on_a_whole_context_pose",
    ),
    pytest.param(
        BASE,
        [
            (SPEC, AFTER_SPEC + 
                "direction tool-up { as-seen-by: <gripper.g_base.g_pinch> } = (0, 0, -1),\n"
                "        direction diag-up { as-seen-by: <kinova.base_link.base_link_origin> }"
                " = (0.7071, 0.7071, 0),\n        angle align-band = 0.05 rad"
            ),
            (HOLD, AFTER_HOLD + 
                "align: keeping angle between <shared.spec.tool-up> and <shared.spec.diag-up>"
                " equal to 0 rad within <shared.spec.align-band>"
            ),
            (CTRL, AFTER_CTRL + "pid ctrl-align { constraint: <home.align>, Kp: 120, Ki: 50, Kd: 80, decay: 0 }"),
        ],
        "signed unit frame axis",
        id="zero_angle_off_a_frame_axis",
    ),
    pytest.param(
        BASE,
        [
            (TWIST, TWIST + POSE_TABLE_TOP),
            (SPEC, AFTER_SPEC + 
                "direction rail-axis { as-seen-by: <kinova.base_link.base_link_origin> } = (1, 0, 0),\n"
                "        line rail { of: <table.table_top>, along: <shared.spec.rail-axis> }"
            ),
            (HOLD, AFTER_HOLD + "on-rail: keeping distance of <shared.world.pose-ee-base> from <shared.spec.rail> equal to 0 m"),
        ],
        "drives an unsigned distance to zero",
        id="unsigned_distance_to_zero",
    ),
    pytest.param(
        BASE,
        [
            (TWIST, TWIST + POSE_TABLE_TOP),
            (SPEC, SPEC + TABLE_PLANE),
            (HOLD, AFTER_HOLD + 
                "bad-projection: keeping projection of <shared.world.pose-ee-base> on"
                " <shared.spec.table> equal to 0 m"
            ),
        ],
        "projects a point on a plane",
        id="projection_on_a_plane",
    ),
    pytest.param(
        BASE, [(SPEC, AFTER_SPEC + "plane flat { of: <gripper.g_base.g_pinch> }")], "takes exactly one 'normal'", id="plane_without_normal"
    ),
    pytest.param(
        BASE,
        [
            (SPEC, AFTER_SPEC + "angle satisfied-band-rot = 0.01 rad"),
            (HOLD, AFTER_HOLD + 
                "held: keeping <shared.world.pose-ee-base>.orientation.z equal to"
                " <spec.home-pose>.orientation within <shared.spec.satisfied-band-rot>"
            ),
            (CTRL, AFTER_CTRL + 
                "pid ctrl-held { constraint: <home.held>, Kp: 40, Ki: 0, Kd: 8, decay: 0 }"
                " as force apply at <gripper.g_base>"
            ),
        ],
        "commands a force on the angular subspace",
        id="force_on_angular_subspace",
    ),
    pytest.param(
        BASE,
        [
            (HOLD, AFTER_HOLD + "speed: norm of <shared.world.twist-ee-base>.linvel greater than 0.05 m/s"),
            (CTRL, AFTER_CTRL + "pid ctrl-speed { constraint: <home.speed>, Kp: 1, Ki: 0, Kd: 0, decay: 0 }"),
        ],
        "nothing can command",
        id="controller_on_a_norm",
    ),
    pytest.param(
        PICK,
        [
            (
                "<spec.approach-path> more than <spec.min-approach-speed>",
                "<spec.approach-path> more than <spec.min-approach-speed>,\n"
                "        pinned: keeping <shared.world.pose-ee-base>.position equal to"
                " <spec.approach-path>.position",
            )
        ],
        "already follows",
        id="path_restated_as_setpoint",
    ),
    pytest.param(
        BASE,
        [("guarded-motion", "tolerances { linear-velocity: 0.02 m }\n\nguarded-motion")],
        "LinearVelocity is measured in 'm'",
        id="default_band_of_wrong_kind",
    ),
    pytest.param(
        BASE,
        [(SPEC, SPEC + PUSH), (SOLVERS_END, PERTURBATION), (SIM, f'{REAL}\n    config:     "robot.toml"')],
        "nothing on hardware can apply them",
        id="perturbation_on_hardware",
    ),
    pytest.param(
        BASE,
        [(HOLD, HOLD.replace(" within <shared.spec.satisfied-band>", ""))],
        "states no band",
        id="equality_without_band",
    ),
    pytest.param(BASE, [("    when {}\n", "")], "states 0 'when' sections", id="motion_without_when"),
    pytest.param(
        BASE,
        [("    handles: <home>", "    context { pre { length extra = 0.01 m } }\n    handles: <home>")],
        "declares a 'pre' context",
        id="handler_pre_context",
    ),
    pytest.param(
        BASE, [(MONITOR, MONITOR.replace("0.3 s", "0.3 m"))], "debounces in 'm'", id="debounce_not_a_duration"
    ),
    pytest.param(
        BASE,
        [
            (
                EXEC,
                'ros (ns=app) {\n    publishers {\n        a: topic "/a" message "std_msgs/msg/Float64",\n    },\n'
                '    publishers {\n        b: topic "/b" message "std_msgs/msg/Float64",\n    },\n}\n\n' + EXEC,
            )
        ],
        "declares 'publishers' twice",
        id="ros_group_repeated",
    ),
    pytest.param(
        BASE,
        [
            (TWIST, f"{TWIST},\n        joint-velocity finger {{ joint: <gripper.g_left_driver_joint> }}"),
            (SPEC, f"{AFTER_SPEC}angular-velocity idle = 0.02 rad/s"),
            (UNTIL, f"{UNTIL},\n        stopped: <shared.world.finger> less than <shared.spec.idle>"),
            (CTRL, f"{AFTER_CTRL}pid ctrl-finger {{ constraint: <home.stopped>, Kp: 1, Ki: 0, Kd: 0 }}"),
        ],
        "a joint is driven through its joint torque",
        id="joint_controller_without_torque",
    ),
    pytest.param(
        BASE,
        [
            (
                TWIST,
                f"{TWIST},\n        joint-position left {{ joint: <gripper.g_left_driver_joint> }}"
                ",\n        joint-position right { joint: <gripper.g_left_driver_joint> }",
            ),
            (HOLD, f"{AFTER_HOLD}gap: keeping (<shared.world.left> - <shared.world.right>) greater than 0.05 rad"),
            (CTRL, f"{AFTER_CTRL}pid ctrl-gap {{ constraint: <home.gap>, Kp: 1, Ki: 0, Kd: 0 }}"),
        ],
        "is a joint, which moves in no Cartesian direction",
        id="controlled_expression_over_joints",
    ),
    pytest.param(
        BASE,
        [
            (
                "equal to <shared.spec.zero-linvel>",
                "equal to (<shared.spec.satisfied-band> + <shared.spec.satisfied-band>)",
            )
        ],
        "compares a LinearVelocity with an expression that infers",
        id="inline_expression_of_another_kind",
    ),
    pytest.param(
        BASE,
        [
            (
                "equal to <shared.spec.zero-linvel>",
                "equal to (<shared.world.pose-ee-base> + <shared.world.pose-ee-base>)",
            )
        ],
        "compares a LinearVelocity with an expression that infers Pose",
        id="geometric_expression",
    ),
    pytest.param(
        BASE,
        [
            (
                SNAPSHOT,
                f"{SNAPSHOT},\n            pose loose-pose = {{ position: (0.1, 0.2, 0.3) m,"
                " orientation: euler { axes: xyz extrinsic, angles: (0.0, 0.0, 0.0) } }",
            )
        ],
        "states no frames",
        id="pose_without_frames",
    ),
]


@pytest.mark.parametrize(("model", "edits", "message"), REJECTIONS)
def test_an_invalid_model_is_rejected(model, edits, message) -> None:
    source = model.read_text()
    for old, new in edits:
        assert old in source, old
        source = source.replace(old, new, 1)
    with pytest.raises(TextXError, match=message):
        motion_spec_metamodel().model_from_str(source, file_name=str(model))


def test_an_estimated_wrench_names_its_observer() -> None:
    """The observer is a node of its own: one agent, a gain in Hz, a dimensionless filter."""
    source = BASE.read_text().replace(
        TWIST, TWIST + ESTIMATED % ",\n            re-tare-on:     { <aas.E_HOME_SETTLED> }", 1
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph
    wrench = next(s for s in graph.subjects() if str(s).endswith("/world/press-wrench"))
    observer = graph.value(wrench, EST["estimated-by"])

    assert (observer, RDF.type, EST.MomentumObserver) in graph
    assert str(graph.value(observer, AGN["of-agent"])).endswith("/arm1")
    gain = graph.value(observer, EST["estimation-gain"])
    assert (graph.value(gain, QUDT_SCHEMA.value).toPython(), graph.value(gain, QUDT_SCHEMA.unit)) == (
        30.0,
        QUDT_UNIT.HZ,
    )
    filter_ = graph.value(observer, EST["filter-constant"])
    assert (
        graph.value(filter_, QUDT_SCHEMA.value).toPython(),
        graph.value(filter_, QUDT_SCHEMA.unit),
    ) == (0.5, QUDT_UNIT.UNITLESS)
