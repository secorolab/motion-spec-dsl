# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

from pathlib import Path
from typing import cast

import pytest
from rdflib import Literal, URIRef
from rdflib.namespace import RDF
from textx.exceptions import TextXSemanticError

from motion_spec.ir_gen import Parser
from motion_spec.namespace import (
    CSTR,
    SNAP,
    CSTR_HDL,
    CSTR_HDL_EXT,
    EL,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_OP,
    GEOM_REL,
    KC,
    KC_STAT,
    MAP,
    MJ,
    MOT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    QUDT_UNIT,
    RBDYN_COORD,
    RBDYN_OP,
    SLV,
    TRAJ,
)
from motion_spec_dsl.rdf import (
    MotionSpecDatasetBuilder,
    _evaluator_id,
    _scalar_id,
)
from motion_spec_dsl.domain import _resolved_spec
from motion_spec_dsl.registration import (
    _canonicalize_jsonld,
    motion_spec_metamodel,
)


FIXTURES = Path(__file__).parent / "fixtures"
VALID_FIXTURES = FIXTURES / "valid"
MODELS = Path(__file__).parents[1] / "models"

CIRCLE_TRAJECTORY = "05_trajectories/01_circle.robmot"
ARC_TRAJECTORY = "05_trajectories/02_arc.robmot"
HELIX_TRAJECTORY = "05_trajectories/03_helix.robmot"
FIGURE8_TRAJECTORY = "05_trajectories/04_figure8.robmot"
STANDALONE_MANIPULATOR = "01_core_semantics/01_standalone_manipulator.robmot"
REUSED_CONSTRAINT = "01_core_semantics/02_reused_constraint.robmot"
SNAPSHOT_POSE = "01_core_semantics/03_snapshot_pose.robmot"
POSE_DISTANCE = "01_core_semantics/04_pose_distance.robmot"
MONITOR_EVENT_FLAG = "01_core_semantics/06_monitor_event_flag.robmot"
POSE_DISTANCE_CROSS_FRAME = "01_core_semantics/07_pose_distance_cross_frame.robmot"
SLIDING_TABLE_5DOF = "02_acceleration_constraints/01_sliding_table_5dof.robmot"
ACCELERATION_FRAME_TRANSFORM = "02_acceleration_constraints/02_acceleration_frame_transform.robmot"
MULTI_SOLVER_ACCELERATION_FORCE = (
    "02_acceleration_constraints/03_multi_solver_acceleration_force.robmot"
)
VELOCITY_TWIST_FRAME_TRANSFORM = (
    "02_acceleration_constraints/04_velocity_twist_frame_transform.robmot"
)
POSE_COMPONENT_SUBSETS = "02_acceleration_constraints/05_pose_component_subsets.robmot"
PID_GAIN_VARIANTS = "02_acceleration_constraints/06_pid_gain_variants.robmot"
FORCE_CONTROLLER = "03_force_commands/01_force_controller.robmot"
POSITION_FORCE_CONTROLLER = "03_force_commands/02_position_force_controller.robmot"
POSTURE_CONTROLLER = "04_posture_control/01_posture_controller.robmot"
JOINT_LIMIT_POSTURE = "04_posture_control/02_joint_limit_posture.robmot"


def _build_dataset(fixture: str):
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_file(VALID_FIXTURES / fixture)
    builder = MotionSpecDatasetBuilder(model)
    dataset, context = builder.build()
    return builder, dataset.default_graph, cast(dict[str, str], context)


def _build_model_dataset(model_name: str):
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_file(MODELS / model_name)
    builder = MotionSpecDatasetBuilder(model)
    dataset, context = builder.build()
    return builder, dataset.default_graph, cast(dict[str, str], context)


def _build_string_dataset(source: str):
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_str(source)
    builder = MotionSpecDatasetBuilder(model)
    dataset, context = builder.build()
    return builder, dataset.default_graph, cast(dict[str, str], context)



def test_env_position_orientation_zero_defaults_are_intentional() -> None:
    _, graph, _ = _build_string_dataset(
        """
        ns app = "https://secorolab.github.io/models/test/"

        ENVIRONMENT (ns=app) world {
            runtime: MuJoCo,
            ASSETS {
                cube-asset: SceneObject
            },
            ASSEMBLY {
                Object cube using <cube-asset> {
                    position: { x: 1.25 m },
                    orientation: {}
                }
            }
        }
        """
    )

    cube = URIRef("https://secorolab.github.io/models/test/world/cube")
    position = URIRef("https://secorolab.github.io/models/test/world/cube.position")

    assert (position, GEOM_REL.of, cube) in graph
    assert (position, GEOM_COORD.x, Literal(1.25)) in graph
    assert (position, GEOM_COORD.y, Literal(0.0)) in graph
    assert (position, GEOM_COORD.z, Literal(0.0)) in graph




def test_environment_typed_assembly_and_member_attach_target() -> None:
    _, graph, _ = _build_string_dataset(
        """
        ns app = "https://secorolab.github.io/models/test/"

        ENVIRONMENT (ns=app) world {
            runtime: MuJoCo,
            ASSETS {
                table-mjcf: SceneObject { xml: "table.xml" },
                kinova-mjcf: RobotAsset { model: KinovaGen3, xml: "gen3.xml" },
                gripper-mjcf: AttachmentAsset { xml: "2f85.xml" }
            },
            ASSEMBLY {
                Object table using <table-mjcf>,
                Robot robot using <kinova-mjcf> {
                    attach-to: <table>.site(table_top),
                    chain: { root: base_link, end: pinch_site }
                },
                Attachment gripper using <gripper-mjcf> {
                    attach-to: <robot>.site(pinch_site),
                    actuator: fingers_actuator
                }
            }
        }
        """
    )

    robot = URIRef("https://secorolab.github.io/models/test/world/robot")
    gripper = URIRef("https://secorolab.github.io/models/test/world/gripper")

    table = URIRef("https://secorolab.github.io/models/test/world/table")

    assert (robot, MJ["attach-kind"], Literal("site")) in graph
    assert (robot, MJ["attach-name"], Literal("table_top")) in graph
    assert (robot, SLV["attached-to"], table) in graph
    assert (gripper, MJ["attach-kind"], Literal("site")) in graph
    assert (gripper, MJ["attach-name"], Literal("pinch_site")) in graph
    assert (gripper, SLV["attached-to"], robot) in graph
    assert (gripper, MJ["actuator-name"], Literal("fingers_actuator")) in graph


def test_environment_assembly_type_must_match_asset_type() -> None:
    with pytest.raises(TextXSemanticError, match="declared as Robot.*SceneObject asset"):
        _build_string_dataset(
            """
            ns app = "https://secorolab.github.io/models/test/"

            ENVIRONMENT (ns=app) world {
                runtime: MuJoCo,
                ASSETS {
                    cube-asset: SceneObject
                },
                ASSEMBLY {
                    Robot cube using <cube-asset>
                }
            }
            """
        )

def test_standalone_builder_emits_motion_constraint_and_evaluator_nodes() -> None:
    builder, graph, context = _build_dataset(STANDALONE_MANIPULATOR)

    handler = builder.authored_handlers[0]
    motion = handler.motion
    constraint = _resolved_spec(motion.while_.constraints[0])
    motion_node = builder.root_uri(f"motion-{motion.name}", owner=motion)
    evaluator_node = builder.root_uri(_evaluator_id(constraint), owner=constraint.parent)
    error_node = builder.root_uri("twist-ee-base.linear.z-err-m_move", owner=constraint.parent)
    twist_node = URIRef(motion.context[0].declaration[0].uri)
    base_node = builder.root_uri("link-base", owner=motion.context[0].declaration[0])

    assert context["app"] == motion.ns.uri
    assert (motion_node, MOT["while"], URIRef(constraint.uri)) in graph
    assert (evaluator_node, CSTR_HDL.constraint, URIRef(constraint.uri)) in graph
    assert (evaluator_node, CSTR_HDL.error, error_node) in graph
    assert (error_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.LinearVelocity) in graph
    assert (twist_node, GEOM_COORD["as-seen-by"], base_node) in graph
    assert (base_node, RDF.type, GEOM_ENT.Frame) in graph


def test_pid_gain_variants_emit_all_three_gains() -> None:
    builder, graph, _ = _build_dataset(PID_GAIN_VARIANTS)

    controllers = {
        controller.name: controller for controller in builder.authored_handlers[0].controllers
    }

    expected = {
        "ctrl-p":  {"proportional-gain": "1.0", "integral-gain": "0.0", "derivative-gain": "0.0"},
        "ctrl-pi": {"proportional-gain": "1.0", "integral-gain": "0.2", "derivative-gain": "0.0"},
        "ctrl-pd": {"proportional-gain": "1.0", "integral-gain": "0.0", "derivative-gain": "0.3"},
    }
    for name, gains in expected.items():
        controller_node = URIRef(controllers[name].uri)
        assert (controller_node, RDF.type, CSTR_HDL.ProportionalIntegralDerivative) in graph
        for predicate, value in gains.items():
            literal = graph.value(controller_node, CSTR_HDL[predicate])
            assert literal is not None, f"{name}: {predicate} missing from graph"
            assert float(literal) == float(value)


def test_force_controller_builder_emits_force_scalar_view_and_solver_specs() -> None:
    builder, graph, _ = _build_dataset(FORCE_CONTROLLER)

    handler = builder.authored_handlers[0]
    force_quantity = handler.motion.context[0].declaration[0]
    scalar_id = f"{force_quantity.name}.force.z"
    scalar_node = builder.root_uri(scalar_id, owner=force_quantity)
    view_node = builder.root_uri(f"view-{scalar_id}", owner=force_quantity)
    controller = handler.controllers[0]
    spec_node = builder.root_uri(f"spec-{controller.name}", owner=handler)
    driver_node = builder.root_uri(f"driver-{handler.motion.name}", owner=handler)
    signal_node = builder.root_uri(f"force-{controller.name}", owner=handler)
    wrench_node = builder.root_uri(f"wrench-force-{controller.name}", owner=handler.motion)
    wrench_op_node = builder.root_uri(
        f"compute-wrench-force-{controller.name}", owner=handler.motion
    )

    assert (scalar_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Force) in graph
    assert (scalar_node, QUDT_SCHEMA.unit, QUDT_UNIT.N) in graph
    assert (view_node, MAP.subobject, scalar_node) in graph
    assert (URIRef(controller.uri), CSTR_HDL["control-signal"], signal_node) in graph
    assert (signal_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Force) in graph
    assert (wrench_node, RDF.type, RBDYN_COORD.WrenchCoordinate) in graph
    assert (wrench_op_node, RDF.type, RBDYN_OP.WrenchFromPositionDirectionAndMagnitude) in graph
    assert (wrench_op_node, RBDYN_OP.magnitude, signal_node) in graph
    assert (spec_node, SLV.force, wrench_node) in graph
    assert (driver_node, SLV["cartesian-force"], spec_node) in graph


def test_explicit_force_command_overrides_acceleration_energy_signal() -> None:
    builder, graph, _ = _build_dataset(POSITION_FORCE_CONTROLLER)

    handler = builder.authored_handlers[0]
    controller = handler.controllers[0]
    motion = handler.motion
    signal_node = builder.root_uri(f"force-{controller.name}", owner=handler)
    acceleration_energy_node = builder.root_uri("eacc-pose-ee-base.distance.z-m_push", owner=motion)
    spec_node = builder.root_uri(f"spec-{controller.name}", owner=handler)
    wrench_node = builder.root_uri(f"wrench-force-{controller.name}", owner=motion)

    assert (URIRef(controller.uri), CSTR_HDL["control-signal"], signal_node) in graph
    assert (
        URIRef(controller.uri),
        CSTR_HDL["control-signal"],
        acceleration_energy_node,
    ) not in graph
    assert (signal_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Force) in graph
    assert (spec_node, SLV.force, wrench_node) in graph


def test_standalone_builder_emits_acceleration_energy_and_solver_links() -> None:
    builder, graph, _ = _build_dataset(STANDALONE_MANIPULATOR)

    handler = builder.authored_handlers[0]
    motion = handler.motion
    driver_node = builder.root_uri(f"driver-{motion.name}", owner=handler)
    solver_node = builder.root_uri(f"{handler.solvers[0].name}-{motion.name}", owner=handler)
    energy_node = builder.root_uri("eacc-twist-ee-base.linear.z-m_move", owner=motion)
    acc_node = builder.root_uri("acc-cstr-twist-ee-base.linear.z-m_move", owner=motion)
    driver_acc_specs = list(graph.objects(driver_node, SLV["acceleration-constraint"]))

    assert (energy_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.AccelerationEnergy) in graph
    assert (energy_node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M2-PER-SEC2"]) in graph
    assert (acc_node, SLV["acceleration-energy"], energy_node) in graph
    assert len(driver_acc_specs) == 1
    assert (solver_node, SLV["motion-drivers"], driver_node) in graph


def test_sliding_table_emits_only_five_acceleration_constraints() -> None:
    builder, graph, _ = _build_dataset(SLIDING_TABLE_5DOF)

    handler = builder.authored_handlers[0]
    motion = handler.motion
    driver_node = builder.root_uri(f"driver-{motion.name}", owner=handler)
    spec_nodes = list(graph.objects(driver_node, SLV["acceleration-constraint"]))

    assert len(spec_nodes) == 1
    constraint_nodes = set(graph.objects(spec_nodes[0], SLV.constraints))
    expected = {
        ("distance.x", "linear-acceleration", "x"),
        ("distance.y", "linear-acceleration", "y"),
        ("rotation.x", "angular-acceleration", "x"),
        ("rotation.y", "angular-acceleration", "y"),
        ("rotation.z", "angular-acceleration", "z"),
    }
    for suffix, subspace, axis in expected:
        acc_node = builder.root_uri(f"acc-cstr-pose-ee-base.{suffix}-m_slide", owner=motion)
        energy_node = builder.root_uri(f"eacc-pose-ee-base.{suffix}-m_slide", owner=motion)
        assert acc_node in constraint_nodes
        assert (acc_node, SLV.subspace, SLV[subspace]) in graph
        assert (acc_node, SLV.axis, SLV[axis]) in graph
        assert (acc_node, SLV["acceleration-energy"], energy_node) in graph

    lin_z_node = builder.root_uri("acc-cstr-pose-ee-base.distance.z-m_slide", owner=motion)
    assert lin_z_node not in constraint_nodes
    assert len(constraint_nodes) == 5


def test_acceleration_constraint_records_authored_axis_frame() -> None:
    builder, graph, _ = _build_dataset(ACCELERATION_FRAME_TRANSFORM)

    motion = builder.authored_handlers[0].motion
    acc_node = builder.root_uri(
        "acc-cstr-twist-ee-base-ee.linear.x-m_frame",
        owner=motion,
    )
    frame_node = builder.root_uri("frame-ee", owner=motion.context[0].declaration[0])

    assert (acc_node, GEOM_COORD["as-seen-by"], frame_node) in graph


def test_pose_diff_supports_position_and_orientation_component_subsets() -> None:
    builder, graph, _ = _build_dataset(POSE_COMPONENT_SUBSETS)

    handler = builder.authored_handlers[0]
    motion = handler.motion
    driver_node = builder.root_uri(f"driver-{motion.name}", owner=handler)
    spec_nodes = list(graph.objects(driver_node, SLV["acceleration-constraint"]))

    assert len(spec_nodes) == 1
    constraint_nodes = set(graph.objects(spec_nodes[0], SLV.constraints))

    expected = {
        "ctrl-position": {
            ("lin-x", "linear-acceleration", "x"),
            ("lin-y", "linear-acceleration", "y"),
            ("lin-z", "linear-acceleration", "z"),
        },
        "ctrl-orientation": {
            ("ang-x", "angular-acceleration", "x"),
            ("ang-y", "angular-acceleration", "y"),
            ("ang-z", "angular-acceleration", "z"),
        },
    }
    position_node = builder.root_uri("pose-ee-base.position", owner=motion)
    orientation_node = builder.root_uri("pose-ee-base.orientation", owner=motion)
    assert (position_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Position) in graph
    assert (orientation_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.PlaneAngle) in graph

    for controller_name, components in expected.items():
        diff_node = builder.root_uri(f"pose-diff-{controller_name}", owner=motion.while_)
        eval_node = builder.root_uri(f"eval-pose-diff-{controller_name}", owner=motion.while_)
        assert (eval_node, GEOM_OP.out, diff_node) in graph
        for suffix, subspace, axis in components:
            err_node = builder.root_uri(f"{controller_name}-err-{suffix}", owner=motion.while_)
            view_node = builder.root_uri(
                f"view-{controller_name}-err-{suffix}", owner=motion.while_
            )
            energy_node = builder.root_uri(f"eacc-{controller_name}-{suffix}", owner=motion)
            acc_node = builder.root_uri(f"acc-cstr-{controller_name}-{suffix}", owner=motion)
            assert (view_node, MAP.superobject, diff_node) in graph
            assert (view_node, MAP.subobject, err_node) in graph
            assert (view_node, MAP.subspace, MAP[subspace]) in graph
            assert (view_node, MAP.axis, MAP[axis]) in graph
            assert (acc_node, SLV["acceleration-energy"], energy_node) in graph
            assert acc_node in constraint_nodes

    assert len(constraint_nodes) == 6


def test_snapshot_pose_constraint_uses_snapshot_value_node() -> None:
    builder, graph, _ = _build_dataset(SNAPSHOT_POSE)

    idle = next(
        handler.motion for handler in builder.authored_handlers if handler.motion.name == "idle"
    )
    live_pose = idle.context[0].declaration[0]
    snapshot = idle.context[1].declaration[0]
    snapshot_node = URIRef(snapshot.uri)
    live_pose_node = URIRef(live_pose.uri)
    constraint_node = URIRef(idle.while_.constraints[0].uri)

    assert (snapshot_node, RDF.type, SNAP.Snapshot) in graph
    assert (snapshot_node, SNAP["snapshot-of"], live_pose_node) in graph
    assert (constraint_node, RDF.type, URIRef(str(CSTR._NS) + "PoseConstraint")) in graph
    assert (constraint_node, CSTR["reference-value"], snapshot_node) in graph


def test_reused_constraint_emits_shared_uri_and_error_signal() -> None:
    builder, graph, _ = _build_dataset(REUSED_CONSTRAINT)

    motions = {handler.motion.name: handler.motion for handler in builder.authored_handlers}
    motion_a_node = builder.root_uri("motion-move_a", owner=motions["move_a"])
    motion_b_node = builder.root_uri("motion-move_b", owner=motions["move_b"])
    constraint = motions["move_a"].while_.constraints[0]
    constraint_uri = URIRef(constraint.uri)
    error_node = builder.root_uri("twist-ee-base.linear.z-err", owner=constraint.parent)

    assert (motion_a_node, MOT["while"], constraint_uri) in graph
    assert (motion_b_node, MOT["while"], constraint_uri) in graph
    assert (error_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.LinearVelocity) in graph


def test_multi_solver_builder_emits_one_driver_and_solver_per_solver_and_uses_motion_owned_signals() -> (
    None
):
    builder, graph, _ = _build_dataset(MULTI_SOLVER_ACCELERATION_FORCE)

    handler = builder.authored_handlers[0]
    motion = handler.motion
    controllers = {controller.name: controller for controller in handler.controllers}

    solver_a_node = builder.root_uri(f"solver_a-{motion.name}", owner=handler)
    solver_b_node = builder.root_uri(f"solver_b-{motion.name}", owner=handler)
    driver_a_node = builder.root_uri(f"driver-solver_a-{motion.name}", owner=handler)
    driver_b_node = builder.root_uri(f"driver-solver_b-{motion.name}", owner=handler)

    assert (solver_a_node, SLV["motion-drivers"], driver_a_node) in graph
    assert (solver_b_node, SLV["motion-drivers"], driver_b_node) in graph

    signal_a = builder.root_uri("eacc-twist-ee-base.linear.z-m_move", owner=motion)
    signal_b = builder.root_uri("force-ctrl-c2", owner=handler)

    assert (
        URIRef(controllers["ctrl-c1"].uri),
        CSTR_HDL["control-signal"],
        signal_a,
    ) in graph
    assert (
        URIRef(controllers["ctrl-c2"].uri),
        CSTR_HDL["control-signal"],
        signal_b,
    ) in graph


def test_posture_controller_emits_joint_force_specification() -> None:
    builder, graph, _ = _build_dataset(POSTURE_CONTROLLER)

    handler = builder.authored_handlers[0]
    controller = handler.controllers[0]
    handler_node = URIRef(handler.uri)
    solver_node = builder.root_uri(f"{handler.solvers[0].name}-{handler.motion.name}", owner=handler)
    driver_node = builder.root_uri(f"driver-{handler.motion.name}", owner=handler)
    torque_node = builder.root_uri("tau-ctrl-j2-posture", owner=handler)
    spec_node = builder.root_uri("jf-spec-ctrl-j2-posture", owner=handler)
    joint_position = handler.motion.context[0].declaration[0]
    joint_position_node = builder.node(joint_position)
    joint_target_node = builder.root_uri("joint-2", owner=handler.motion)

    assert (handler_node, CSTR_HDL["control-mode"], CSTR_HDL.JointTorque) in graph

    # Torque quantity: control signal for the controller, typed as JointForceCoordinate
    assert (URIRef(controller.uri), CSTR_HDL["control-signal"], torque_node) in graph
    assert (torque_node, RDF.type, QUDT_SCHEMA.Quantity) in graph
    assert (torque_node, RDF.type, KC_STAT.JointForceCoordinate) in graph
    assert (torque_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Torque) in graph
    assert (torque_node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M"]) in graph

    # JointForceSpecification: first-class motion driver linking force to joint
    assert (spec_node, RDF.type, SLV.JointForceSpecification) in graph
    assert (spec_node, SLV.force, torque_node) in graph
    assert (spec_node, SLV["attached-to"], joint_target_node) in graph
    assert (driver_node, SLV["joint-force"], spec_node) in graph

    # JointPosition world quantity still carries its own joint reference
    assert (solver_node, SLV["output"], joint_position_node) in graph
    assert (joint_position_node, RDF.type, KC_STAT.JointPositionCoordinate) in graph
    assert (joint_position_node, GEOM_REL.of, joint_target_node) in graph
    assert (joint_target_node, RDF.type, KC.Joint) in graph

    parsed_solver = Parser(graph).solver_with_input_and_output(solver_node)
    assert parsed_solver.output[0].joint_name == "joint-2"
    assert parsed_solver.motion_drivers[0].joint_force[0].joint_name == "joint-2"

    parsed_handler = Parser(graph).constraint_handler(handler_node)
    assert parsed_handler.control_mode == "JointTorque"


def test_posture_joint_limit_constraints_emit_joint_force_and_error_signals() -> None:
    builder, graph, _ = _build_dataset(JOINT_LIMIT_POSTURE)

    handler = builder.authored_handlers[0]
    driver_node = builder.root_uri(f"driver-{handler.motion.name}", owner=handler)
    joint_force_nodes = list(graph.objects(driver_node, SLV["joint-force"]))
    assert len(joint_force_nodes) == 2

    for error_id in ("q-j2-err-m-joint-limits", "q-j4-err-m-joint-limits"):
        error_node = builder.root_uri(error_id, owner=handler.motion.while_)
        assert (error_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND["Angle"]) in graph
        assert (error_node, QUDT_SCHEMA.unit, QUDT_UNIT["RAD"]) in graph


def test_pose_position_without_axis_emits_linear_distance_operation() -> None:
    builder, graph, _ = _build_dataset(POSE_DISTANCE)

    motion = builder.authored_handlers[0].motion
    constraint = _resolved_spec(motion.while_.constraints[-1])
    world_qtys = builder._collect_world_quantities(  # noqa: SLF001 - semantic graph fixture check.
        motion, builder.authored_handlers[0]
    )
    pose_quantity = builder._resolve_constraint_quantity(  # noqa: SLF001 - semantic graph fixture check.
        constraint, world_qtys
    )
    assert pose_quantity is not None
    distance_id = _scalar_id(pose_quantity, "distance", None)
    pose_node = URIRef(pose_quantity.uri)
    distance_node = builder.root_uri(distance_id, owner=motion)
    op_node = builder.root_uri(f"compute-{distance_id}", owner=motion)

    assert (distance_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Distance) in graph
    assert (distance_node, QUDT_SCHEMA.unit, QUDT_UNIT.M) in graph
    assert (op_node, RDF.type, GEOM_OP.PoseToLinearDistance) in graph
    assert (op_node, GEOM_OP.pose, pose_node) in graph
    assert (op_node, GEOM_OP.distance, distance_node) in graph
    assert (URIRef(constraint.uri), RDF.type, CSTR.DistanceConstraint) in graph
    assert (URIRef(constraint.uri), CSTR.quantity, distance_node) in graph


def test_pose_distance_composes_cross_reference_frame_transform_path() -> None:
    builder, graph, _ = _build_dataset(POSE_DISTANCE_CROSS_FRAME)

    motion = builder.authored_handlers[0].motion
    constraint = _resolved_spec(motion.while_.constraints[0])
    target_pose_node = URIRef(
        str(motion.namespace) + "m_cross_distance/pose-frame-ee-frame-shoulder"
    )
    end_in_start_ref_node = builder.root_uri(
        "pose-pose-ee-table-in-frame-base-c_dist_cross",
        owner=motion,
    )
    reference_compose_node = builder.root_uri(
        "compute-pose-ee-table-in-frame-base-c_dist_cross",
        owner=motion,
    )
    target_compose_node = builder.root_uri("compute-pose-frame-ee-frame-shoulder", owner=motion)
    distance_node = builder.root_uri("pose-frame-ee-frame-shoulder.distance", owner=motion)
    distance_op_node = builder.root_uri(
        "compute-pose-frame-ee-frame-shoulder.distance",
        owner=motion,
    )

    assert (reference_compose_node, RDF.type, GEOM_OP.ComposePose) in graph
    assert (reference_compose_node, GEOM_OP.composite, end_in_start_ref_node) in graph
    assert (target_compose_node, RDF.type, GEOM_OP.ComposePose) in graph
    assert (target_compose_node, GEOM_OP.composite, target_pose_node) in graph
    assert (distance_op_node, RDF.type, GEOM_OP.PoseToLinearDistance) in graph
    assert (distance_op_node, GEOM_OP.pose, target_pose_node) in graph
    assert (distance_op_node, GEOM_OP.distance, distance_node) in graph
    assert (URIRef(constraint.uri), CSTR.quantity, distance_node) in graph


def test_monitor_event_and_flag_emit_signal_nodes_and_evaluators() -> None:
    builder, graph, _ = _build_dataset(MONITOR_EVENT_FLAG)

    handler = builder.authored_handlers[0]
    motion = handler.motion
    start_constraint = _resolved_spec(motion.when.constraints[0])
    stop_constraint = _resolved_spec(motion.until.constraints[0])
    start_monitor = handler.monitors[0]
    stop_monitor = handler.monitors[1]

    start_monitor_node = URIRef(start_monitor.uri)
    stop_monitor_node = URIRef(stop_monitor.uri)
    start_event_node = URIRef(f"{start_monitor.uri}.evt-start")
    stop_flag_node = URIRef(f"{stop_monitor.uri}.flag-stop")
    start_eval_node = builder.root_uri(
        _evaluator_id(start_constraint), owner=start_constraint.parent
    )
    stop_eval_node = builder.root_uri(_evaluator_id(stop_constraint), owner=stop_constraint.parent)
    motion_node = builder.root_uri("motion-m_guarded", owner=motion)
    start_error_node = builder.root_uri(
        "twist-ee-base.linear.z-err",
        owner=start_constraint.parent,
    )
    stop_error_node = builder.root_uri(
        "twist-ee-base.linear.z-err",
        owner=stop_constraint.parent,
    )

    assert (start_monitor_node, RDF.type, CSTR_HDL.Monitor) in graph
    assert (start_monitor_node, RDF.type, CSTR_HDL.EdgeTriggeredMonitor) in graph
    assert (start_monitor_node, CSTR_HDL.event, start_event_node) in graph
    assert (start_event_node, RDF.type, EL.Event) in graph
    assert (start_eval_node, CSTR_HDL.constraint, URIRef(start_constraint.uri)) in graph
    assert (start_eval_node, CSTR_HDL.error, start_error_node) in graph

    assert (stop_monitor_node, RDF.type, CSTR_HDL.Monitor) in graph
    assert (stop_monitor_node, RDF.type, CSTR_HDL.LevelTriggeredMonitor) in graph
    assert (stop_monitor_node, CSTR_HDL_EXT["monitors-until"], motion_node) in graph
    assert (stop_monitor_node, CSTR_HDL.flag, stop_flag_node) in graph
    assert (stop_flag_node, RDF.type, EL.Flag) in graph
    assert (stop_eval_node, CSTR_HDL.constraint, URIRef(stop_constraint.uri)) in graph
    assert (stop_eval_node, CSTR_HDL.error, stop_error_node) in graph


def test_namespace_qualified_monitor_event_uses_foreign_namespace_uri() -> None:
    # A namespace-qualified monitor event (e.g. an FSM event) resolves under the
    # referenced namespace, while a standalone event stays monitor-owned.
    source = (VALID_FIXTURES / MONITOR_EVENT_FLAG).read_text().replace(
        'ns app = "https://secorolab.github.io/models/test/"',
        'ns app = "https://secorolab.github.io/models/test/"\nns coord = "http://example.org/coord/"',
    ).replace("trigger event evt-start when active", "trigger event coord.E_OBJ_REACHED when active")

    builder, graph, _ = _build_string_dataset(source)
    start_monitor = builder.authored_handlers[0].monitors[0]
    event_node = URIRef("http://example.org/coord/E_OBJ_REACHED")

    assert start_monitor.event.uri == "http://example.org/coord/E_OBJ_REACHED"
    assert (URIRef(start_monitor.uri), CSTR_HDL.event, event_node) in graph
    assert (event_node, RDF.type, EL.Event) in graph
    # The qualified URI does NOT fall under the monitor's own namespace.
    assert not str(event_node).startswith(start_monitor.uri)


def test_derived_velocity_twist_transform_emits_rotate_operation() -> None:
    builder, graph, _ = _build_dataset(VELOCITY_TWIST_FRAME_TRANSFORM)

    motion = builder.authored_handlers[0].motion
    world_decl = motion.context[0].declaration
    source_twist = world_decl[3]  # twist-ee-base-base
    target_twist = world_decl[4]  # twist-ee-base-ee

    source_node = URIRef(source_twist.uri)
    target_node = URIRef(target_twist.uri)
    inverse_pose_node = builder.root_uri("inverse-pose-ee-base", owner=motion)
    op_node = builder.root_uri(
        "transform-twist-ee-base-base-to-twist-ee-base-ee",
        owner=motion,
    )

    assert (op_node, RDF.type, GEOM_OP.RotateVelocityTwistToProximalWithPose) in graph
    assert (op_node, GEOM_OP.pose, inverse_pose_node) in graph
    assert (op_node, GEOM_OP["from"], source_node) in graph
    assert (op_node, GEOM_OP.to, target_node) in graph


def _frame_trajectory_model(*, lhs: str, traj_wrt: str, start_pose: str, goal_name: str = "goal-pose") -> str:
    return f"""ns app = "https://secorolab.github.io/models/tests/"

ENVIRONMENT (ns=app) world {{
    runtime: RealRobot,
    ASSETS {{
        kinova-urdf: RobotAsset {{ model: KinovaGen3, urdf: "../robots/kg3.urdf" }}
    }},
    ASSEMBLY {{
        Robot kinova using <kinova-urdf> {{
            chain: {{ root: frame-base, end: frame-ee }}
        }}
    }}
}}

MOTION_SPEC (ns=app) m_frame_traj {{
    CONTEXT {{
        w: World {{
            pose-ee-base: Pose {{ of: frame-ee, wrt: frame-base, as-seen-by: frame-base }},
            pose-cube-base: Pose {{ of: frame-cube, wrt: frame-base, as-seen-by: frame-base }},
            pose-ee-cube: Pose {{ of: frame-ee, wrt: frame-cube, as-seen-by: frame-base }}
        }},
        s: Spec {{
            alpha: TrajectoryProgress,
            start-pose: Pose {{ of: frame-ee, wrt: {traj_wrt}, as-seen-by: frame-base }} = Snapshot of <w.{start_pose}>,
            {goal_name}: Pose {{ of: frame-ee, wrt: {traj_wrt}, as-seen-by: frame-base }} {{
                position: {{ x: 0.0 m, y: 0.0 m, z: 0.20 m }},
                orientation: {{ roll: 3.14159 rad, pitch: 0.0 rad, yaw: 1.5708 rad }}
            }},
            traj: Trajectory {{ of: frame-ee, wrt: {traj_wrt}, as-seen-by: frame-base }} = Lerp {{
                start: <s.start-pose>,
                goal:  <s.{goal_name}>,
                alpha: <s.alpha>
            }}
        }}
    }}

    WHEN {{}}

    WHILE {{
        follow: keeping <w.{lhs}> equal to <s.traj>
    }}

    UNTIL {{}}
}}

CONSTRAINT_HANDLER (ns=app) handler_frame_traj {{
    CONTEXT {{
        hw: World {{ gravity: Gravity }},
        hs: Spec {{ gravity-vec: FreeVector {{ x = 0.0, y = 0.0, z = -9.81 m/s2 }} }}
    }}

    MOTION: <m_frame_traj>
    CONTROL_MODE: JointTorque

    CONTROLLERS {{
        ctrl-follow: PID {{ constraint: <m_frame_traj.follow>, Kp = 1.0, Ki = 0.0, Kd = 0.1 }}
    }}

    SOLVERS {{
        arm-solver: Solver {{
            robot: <world.kinova>,
            algorithm: ACHD,
            root: <world.kinova.chain.root>,
            end: <world.kinova.chain.end>,
            gravity: <hw.gravity> equal to <hs.gravity-vec>
        }}
    }}
}}
"""


def test_frame_aware_trajectory_reference_composes_cube_trajectory_into_base_frame() -> None:
    builder, graph, _ = _build_string_dataset(
        _frame_trajectory_model(
            lhs="pose-ee-base",
            traj_wrt="frame-cube",
            start_pose="pose-ee-cube",
        )
    )

    motion = builder.authored_handlers[0].motion
    constraint = _resolved_spec(motion.while_.constraints[0])
    constraint_node = URIRef(constraint.uri)
    traj_quantity = next(
        item
        for ctx in motion.context
        for item in getattr(ctx, "declaration", [])
        if getattr(item, "name", None) == "traj"
    )
    cube_pose_quantity = next(
        item
        for ctx in motion.context
        for item in getattr(ctx, "declaration", [])
        if getattr(item, "name", None) == "pose-cube-base"
    )
    traj_node = URIRef(traj_quantity.uri)
    cube_pose_node = URIRef(cube_pose_quantity.uri)
    transformed_node = builder.root_uri("traj-in-frame-base-follow", owner=motion)
    compose_node = builder.root_uri("compute-traj-in-frame-base-follow", owner=motion)

    assert (compose_node, RDF.type, GEOM_OP.ComposePose) in graph
    assert (compose_node, GEOM_OP.in1, cube_pose_node) in graph
    assert (compose_node, GEOM_OP.in2, traj_node) in graph
    assert (compose_node, GEOM_OP.composite, transformed_node) in graph
    assert (constraint_node, CSTR["reference-value"], transformed_node) in graph

    # The pose-diff evaluator must diff against the world-frame trajectory, not the
    # raw cube-frame one — otherwise the controller drives the EE to the wrong place.
    ee_pose_quantity = next(
        item
        for ctx in motion.context
        for item in getattr(ctx, "declaration", [])
        if getattr(item, "name", None) == "pose-ee-base"
    )
    eval_node = builder.root_uri("eval-pose-diff-ctrl-follow", owner=motion)
    assert (eval_node, RDF.type, GEOM_OP["PoseDiffEvaluator"]) in graph
    assert (eval_node, GEOM_OP.in1, URIRef(ee_pose_quantity.uri)) in graph
    assert (eval_node, GEOM_OP.in2, transformed_node) in graph
    assert (eval_node, GEOM_OP.in2, traj_node) not in graph


def test_frame_aware_trajectory_reference_inverts_path_when_target_is_cube_frame() -> None:
    builder, graph, _ = _build_string_dataset(
        _frame_trajectory_model(
            lhs="pose-ee-cube",
            traj_wrt="frame-base",
            start_pose="pose-ee-base",
            goal_name="goal-pose-base",
        )
    )

    motion = builder.authored_handlers[0].motion
    constraint = _resolved_spec(motion.while_.constraints[0])
    constraint_node = URIRef(constraint.uri)
    traj_quantity = next(
        item
        for ctx in motion.context
        for item in getattr(ctx, "declaration", [])
        if getattr(item, "name", None) == "traj"
    )
    cube_pose_quantity = next(
        item
        for ctx in motion.context
        for item in getattr(ctx, "declaration", [])
        if getattr(item, "name", None) == "pose-cube-base"
    )
    traj_node = URIRef(traj_quantity.uri)
    cube_pose_node = URIRef(cube_pose_quantity.uri)
    inverse_node = builder.root_uri("inverse-pose-cube-base", owner=motion)
    invert_node = builder.root_uri("compute-inverse-pose-cube-base", owner=motion)
    transformed_node = builder.root_uri("traj-in-frame-cube-follow", owner=motion)
    compose_node = builder.root_uri("compute-traj-in-frame-cube-follow", owner=motion)

    assert (invert_node, RDF.type, GEOM_OP.InvertPose) in graph
    assert (invert_node, GEOM_OP.pose, cube_pose_node) in graph
    assert (invert_node, GEOM_OP.out, inverse_node) in graph
    assert (compose_node, RDF.type, GEOM_OP.ComposePose) in graph
    assert (compose_node, GEOM_OP.in1, inverse_node) in graph
    assert (compose_node, GEOM_OP.in2, traj_node) in graph
    assert (compose_node, GEOM_OP.composite, transformed_node) in graph
    assert (constraint_node, CSTR["reference-value"], transformed_node) in graph


def _trajectory_quantity(motion, name: str):
    for ctx in motion.context:
        for item in getattr(ctx, "declaration", []):
            if getattr(item, "name", None) == name:
                return item
    raise AssertionError(f"trajectory quantity '{name}' not found")


def test_circle_trajectory_emits_operator_and_input_edges() -> None:
    builder, graph, _ = _build_dataset(CIRCLE_TRAJECTORY)
    motion = builder.authored_handlers[0].motion
    traj_quantity = _trajectory_quantity(motion, "traj")
    traj_node = URIRef(traj_quantity.uri)
    op_node = builder.root_uri("circle-traj", owner=traj_quantity)

    assert (op_node, RDF.type, TRAJ.Circle) in graph
    assert (op_node, TRAJ.start, URIRef(_trajectory_quantity(motion, "start-pose").uri)) in graph
    assert (op_node, TRAJ.center, URIRef(_trajectory_quantity(motion, "center-pos").uri)) in graph
    assert (op_node, TRAJ["plane-normal"], URIRef(_trajectory_quantity(motion, "plane-normal").uri)) in graph
    assert (op_node, TRAJ.alpha, URIRef(_trajectory_quantity(motion, "alpha").uri)) in graph
    assert (op_node, TRAJ.trajectory, traj_node) in graph
    assert (traj_node, RDF.type, TRAJ.Trajectory) in graph
    assert (traj_node, RDF.type, GEOM_REL.Pose) in graph


def test_arc_trajectory_emits_operator_and_input_edges() -> None:
    builder, graph, _ = _build_dataset(ARC_TRAJECTORY)
    motion = builder.authored_handlers[0].motion
    traj_quantity = _trajectory_quantity(motion, "traj")
    traj_node = URIRef(traj_quantity.uri)
    op_node = builder.root_uri("arc-traj", owner=traj_quantity)

    assert (op_node, RDF.type, TRAJ.Arc) in graph
    assert (op_node, TRAJ.start, URIRef(_trajectory_quantity(motion, "start-pose").uri)) in graph
    assert (op_node, TRAJ.end, URIRef(_trajectory_quantity(motion, "end-point").uri)) in graph
    assert (op_node, TRAJ.amplitude, URIRef(_trajectory_quantity(motion, "amplitude").uri)) in graph
    assert (op_node, TRAJ["plane-normal"], URIRef(_trajectory_quantity(motion, "plane-normal").uri)) in graph
    assert (op_node, TRAJ.alpha, URIRef(_trajectory_quantity(motion, "alpha").uri)) in graph
    assert (op_node, TRAJ.trajectory, traj_node) in graph
    assert (traj_node, RDF.type, TRAJ.Trajectory) in graph
    assert (traj_node, RDF.type, GEOM_REL.Pose) in graph


def test_figure8_trajectory_emits_operator_and_input_edges() -> None:
    builder, graph, _ = _build_dataset(FIGURE8_TRAJECTORY)
    motion = builder.authored_handlers[0].motion
    traj_quantity = _trajectory_quantity(motion, "traj")
    traj_node = URIRef(traj_quantity.uri)
    op_node = builder.root_uri("figure8-traj", owner=traj_quantity)

    assert (op_node, RDF.type, TRAJ.Figure8) in graph
    assert (op_node, TRAJ.anchor, URIRef(_trajectory_quantity(motion, "start-pose").uri)) in graph
    assert (op_node, TRAJ.radius, URIRef(_trajectory_quantity(motion, "radius").uri)) in graph
    assert (op_node, TRAJ["plane-normal"], URIRef(_trajectory_quantity(motion, "plane-normal").uri)) in graph
    assert (op_node, TRAJ.alpha, URIRef(_trajectory_quantity(motion, "alpha").uri)) in graph
    # 04_figure8.robmot selects the Bernoulli form explicitly.
    assert (op_node, TRAJ.form, Literal("Bernoulli")) in graph
    assert (op_node, TRAJ.trajectory, traj_node) in graph
    assert (traj_node, RDF.type, TRAJ.Trajectory) in graph
    assert (traj_node, RDF.type, GEOM_REL.Pose) in graph


def test_figure8_trajectory_defaults_form_to_gerono_when_omitted() -> None:
    source = (VALID_FIXTURES / FIGURE8_TRAJECTORY).read_text().replace(
        ",\n                form:         Bernoulli", ""
    )
    builder, graph, _ = _build_string_dataset(source)
    motion = builder.authored_handlers[0].motion
    traj_quantity = _trajectory_quantity(motion, "traj")
    op_node = builder.root_uri("figure8-traj", owner=traj_quantity)

    assert (op_node, RDF.type, TRAJ.Figure8) in graph
    assert (op_node, TRAJ.form, Literal("Gerono")) in graph


def test_helix_trajectory_emits_operator_and_input_edges() -> None:
    builder, graph, _ = _build_dataset(HELIX_TRAJECTORY)
    motion = builder.authored_handlers[0].motion
    traj_quantity = _trajectory_quantity(motion, "traj")
    traj_node = URIRef(traj_quantity.uri)
    op_node = builder.root_uri("helix-traj", owner=traj_quantity)

    assert (op_node, RDF.type, TRAJ.Helix) in graph
    assert (op_node, TRAJ.start, URIRef(_trajectory_quantity(motion, "start-pose").uri)) in graph
    assert (op_node, TRAJ.center, URIRef(_trajectory_quantity(motion, "center-pos").uri)) in graph
    assert (op_node, TRAJ.axis, URIRef(_trajectory_quantity(motion, "axis").uri)) in graph
    assert (op_node, TRAJ.pitch, URIRef(_trajectory_quantity(motion, "pitch").uri)) in graph
    assert (op_node, TRAJ.revolutions, URIRef(_trajectory_quantity(motion, "revolutions").uri)) in graph
    assert (op_node, TRAJ.alpha, URIRef(_trajectory_quantity(motion, "alpha").uri)) in graph
    assert (op_node, TRAJ.trajectory, traj_node) in graph
    assert (traj_node, RDF.type, TRAJ.Trajectory) in graph
    assert (traj_node, RDF.type, GEOM_REL.Pose) in graph


def test_canonicalize_jsonld_orders_graph_by_id_deterministically() -> None:
    import json

    # rdflib emits @graph in hash-seeded order; emission must not depend on it.
    doc = {
        "@context": {"app": "https://example.org/"},
        "@graph": [
            {"@id": "app:charlie", "v": 1},
            {"@id": "app:alpha", "v": 2},
            {"@id": "app:bravo", "v": 3},
        ],
    }
    out = json.loads(_canonicalize_jsonld(json.dumps(doc)))

    assert [n["@id"] for n in out["@graph"]] == ["app:alpha", "app:bravo", "app:charlie"]
    # @context is preserved and node payloads are untouched (only order changes).
    assert out["@context"] == doc["@context"]
    assert {n["@id"]: n["v"] for n in out["@graph"]} == {"app:charlie": 1, "app:alpha": 2, "app:bravo": 3}
    # Idempotent: canonicalizing an already-canonical document is a no-op.
    assert _canonicalize_jsonld(_canonicalize_jsonld(json.dumps(doc))) == _canonicalize_jsonld(json.dumps(doc))
