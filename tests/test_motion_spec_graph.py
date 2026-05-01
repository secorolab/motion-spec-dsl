# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

from pathlib import Path
from typing import cast

from rdflib import Literal, URIRef
from rdflib.namespace import RDF

from motion_spec.ir_gen import Parser
from motion_spec.namespace import (
    APP,
    CSTR,
    CSTR_HDL,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_OP,
    GEOM_REL,
    KC,
    KC_STAT,
    MAP,
    MOT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    QUDT_UNIT,
    RBDYN_COORD,
    RBDYN_OP,
    SLV,
)
from motion_spec_dsl.rdf import (
    MotionSpecDatasetBuilder,
    _evaluator_id,
    _scalar_id,
)
from motion_spec_dsl.domain import _resolved_spec
from motion_spec_dsl.registration import (
    motion_spec_metamodel,
)


FIXTURES = Path(__file__).parent / "fixtures"
VALID_FIXTURES = FIXTURES / "valid"
MODELS = Path(__file__).parents[1] / "models"

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


def test_pid_gain_variants_only_emit_authored_gain_terms() -> None:
    builder, graph, _ = _build_dataset(PID_GAIN_VARIANTS)

    controllers = {
        controller.name: controller for controller in builder.authored_handlers[0].controllers
    }

    expected = {
        "ctrl-p": {"proportional-gain": "1.0"},
        "ctrl-pi": {"proportional-gain": "1.0", "integral-gain": "0.2"},
        "ctrl-pd": {"proportional-gain": "1.0", "derivative-gain": "0.3"},
    }
    for name, gains in expected.items():
        controller_node = URIRef(controllers[name].uri)
        assert (controller_node, RDF.type, CSTR_HDL.ProportionalIntegralDerivative) in graph
        for predicate, value in gains.items():
            assert (controller_node, CSTR_HDL[predicate], Literal(value)) in graph
        for predicate in {"proportional-gain", "integral-gain", "derivative-gain"} - set(gains):
            assert not list(graph.objects(controller_node, CSTR_HDL[predicate]))


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
    solver_node = builder.root_uri(handler.solvers[0].name, owner=handler)
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
    assert (orientation_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Direction) in graph

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

    assert (snapshot_node, RDF.type, URIRef(str(APP._NS) + "Snapshot")) in graph
    assert (snapshot_node, URIRef(str(APP._NS) + "snapshot-of"), live_pose_node) in graph
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

    solver_a_node = builder.root_uri("solver_a", owner=handler)
    solver_b_node = builder.root_uri("solver_b", owner=handler)
    driver_a_node = builder.root_uri("driver-solver_a", owner=handler)
    driver_b_node = builder.root_uri("driver-solver_b", owner=handler)

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
    solver_node = builder.root_uri(handler.solvers[0].name, owner=handler.solvers[0])
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
    start_event_node = builder.root_uri("evt-start", owner=handler)
    stop_flag_node = builder.root_uri("flag-stop", owner=handler)
    start_eval_node = builder.root_uri(
        _evaluator_id(start_constraint), owner=start_constraint.parent
    )
    stop_eval_node = builder.root_uri(_evaluator_id(stop_constraint), owner=stop_constraint.parent)
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
    assert (start_event_node, RDF.type, CSTR_HDL.Event) in graph
    assert (start_eval_node, CSTR_HDL.constraint, URIRef(start_constraint.uri)) in graph
    assert (start_eval_node, CSTR_HDL.error, start_error_node) in graph

    assert (stop_monitor_node, RDF.type, CSTR_HDL.Monitor) in graph
    assert (stop_monitor_node, RDF.type, CSTR_HDL.LevelTriggeredMonitor) in graph
    assert (stop_monitor_node, CSTR_HDL.flag, stop_flag_node) in graph
    assert (stop_flag_node, RDF.type, CSTR_HDL.Flag) in graph
    assert (stop_eval_node, CSTR_HDL.constraint, URIRef(stop_constraint.uri)) in graph
    assert (stop_eval_node, CSTR_HDL.error, stop_error_node) in graph


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
