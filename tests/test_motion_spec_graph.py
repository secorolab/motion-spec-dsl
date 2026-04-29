# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

from pathlib import Path
from typing import cast

from rdflib import URIRef
from rdflib.namespace import RDF

from motion_spec.namespace import (
    APP,
    CSTR,
    CSTR_HDL,
    GEOM_OP,
    GEOM_REL,
    KC,
    MAP,
    MOT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    QUDT_UNIT,
    SLV,
)
from motion_spec_dsl.generators.motion_spec_graph import (
    MotionSpecDatasetBuilder,
    _evaluator_id,
)
from motion_spec_dsl.generators.classes import _resolved_spec
from motion_spec_dsl.generators.registration import motion_spec_metamodel


FIXTURES = Path(__file__).parent / "fixtures"
MODELS = Path(__file__).parents[1] / "models"


def _build_dataset(fixture: str):
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_file(FIXTURES / fixture)
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
    builder, graph, context = _build_dataset("standalone_manipulator.robmot")

    handler = builder.authored_handlers[0]
    motion = handler.motion
    constraint = _resolved_spec(motion.while_.constraints[0])
    motion_node = builder.root_uri(f"motion-{motion.name}", owner=motion)
    evaluator_node = builder.root_uri(
        _evaluator_id(constraint), owner=constraint.parent
    )
    error_node = builder.root_uri("twist-ee-base.linear.z-err-m_move", owner=constraint.parent)

    assert context["app"] == motion.ns.uri
    assert (motion_node, MOT["while"], URIRef(constraint.uri)) in graph
    assert (evaluator_node, CSTR_HDL.constraint, URIRef(constraint.uri)) in graph
    assert (evaluator_node, CSTR_HDL.error, error_node) in graph
    assert (error_node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.LinearVelocity) in graph


def test_force_controller_builder_emits_force_scalar_view_and_solver_specs() -> None:
    builder, graph, _ = _build_dataset("force_controller.robmot")

    handler = builder.authored_handlers[0]
    force_quantity = handler.motion.context[0].declaration[0]
    scalar_id = f"{force_quantity.name}.force.z"
    scalar_node = builder.root_uri(scalar_id, owner=force_quantity)
    view_node = builder.root_uri(f"view-{scalar_id}", owner=force_quantity)
    spec_node = builder.root_uri(f"spec-{force_quantity.name}", owner=handler)
    driver_node = builder.root_uri(f"drv-{handler.motion.name}", owner=handler)

    assert (scalar_node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Force) in graph
    assert (scalar_node, QUDT_SCHEMA.unit, QUDT_UNIT.N) in graph
    assert (view_node, MAP.subobject, scalar_node) in graph
    assert (spec_node, SLV.force, builder.node(force_quantity)) in graph
    assert (driver_node, SLV["cartesian-force"], spec_node) in graph


def test_standalone_builder_emits_acceleration_energy_and_solver_links() -> None:
    builder, graph, _ = _build_dataset("standalone_manipulator.robmot")

    handler = builder.authored_handlers[0]
    motion = handler.motion
    driver_node = builder.root_uri(f"drv-{motion.name}", owner=handler)
    solver_node = builder.root_uri(f"slv-{motion.name}", owner=handler)
    energy_node = builder.root_uri("eacc-twist-ee-base.linear.z-m_move", owner=motion)
    acc_node = builder.root_uri("acc-cstr-twist-ee-base.linear.z-m_move", owner=motion)
    driver_acc_specs = list(graph.objects(driver_node, SLV["acceleration-constraint"]))

    assert (energy_node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.AccelerationEnergy) in graph
    assert (energy_node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M2-PER-SEC2"]) in graph
    assert (acc_node, SLV["acceleration-energy"], energy_node) in graph
    assert len(driver_acc_specs) == 1
    assert (solver_node, SLV["motion-drivers"], driver_node) in graph


def test_snapshot_pose_constraint_uses_snapshot_value_node() -> None:
    builder, graph, _ = _build_model_dataset("grc_no_traj-collab.robmot")

    idle = next(
        handler.motion for handler in builder.authored_handlers if handler.motion.name == "idle"
    )
    live_pose = idle.context[0].declaration[1]
    snapshot = idle.context[1].declaration[0]
    snapshot_node = URIRef(snapshot.uri)
    live_pose_node = URIRef(live_pose.uri)
    constraint_node = URIRef(idle.while_.constraints[0].uri)

    assert (snapshot_node, RDF.type, URIRef(str(APP._NS) + "Snapshot")) in graph
    assert (snapshot_node, URIRef(str(APP._NS) + "snapshot-of"), live_pose_node) in graph
    assert (constraint_node, RDF.type, URIRef(str(CSTR._NS) + "PoseConstraint")) in graph
    assert (constraint_node, CSTR["reference-value"], snapshot_node) in graph


def test_reused_constraint_emits_shared_uri_and_error_signal() -> None:
    builder, graph, _ = _build_dataset("reused_constraint.robmot")

    motions = {handler.motion.name: handler.motion for handler in builder.authored_handlers}
    motion_a_node = builder.root_uri("motion-move_a", owner=motions["move_a"])
    motion_b_node = builder.root_uri("motion-move_b", owner=motions["move_b"])
    constraint = motions["move_a"].while_.constraints[0]
    constraint_uri = URIRef(constraint.uri)
    error_node = builder.root_uri("twist-ee-base.linear.z-err", owner=constraint.parent)

    assert (motion_a_node, MOT["while"], constraint_uri) in graph
    assert (motion_b_node, MOT["while"], constraint_uri) in graph
    assert (error_node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.LinearVelocity) in graph


def test_multi_solver_builder_emits_one_driver_and_solver_per_solver_and_uses_motion_owned_signals() -> None:
    builder, graph, _ = _build_dataset("multi_solver_cross_ns.robmot")

    handler = builder.authored_handlers[0]
    motion = handler.motion
    controllers = {controller.name: controller for controller in handler.controllers}

    solver_a_node = builder.root_uri("slv-solver_a", owner=handler)
    solver_b_node = builder.root_uri("slv-solver_b", owner=handler)
    driver_a_node = builder.root_uri("drv-solver_a", owner=handler)
    driver_b_node = builder.root_uri("drv-solver_b", owner=handler)

    assert (solver_a_node, SLV["motion-drivers"], driver_a_node) in graph
    assert (solver_b_node, SLV["motion-drivers"], driver_b_node) in graph

    signal_a = builder.root_uri("eacc-twist-ee-base.linear.z-m_move", owner=motion)
    signal_b = builder.root_uri("wrench-ee.force.z", owner=motion)

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


def test_joint_force_interfaces_are_materialized_when_present() -> None:
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_file(FIXTURES / "standalone_manipulator.robmot")
    handler = next(spec for spec in model.specs if getattr(spec, "name", "") == "handler_move")
    solver = handler.solvers[0]
    solver.joint_force = ["tau-j1"]

    builder = MotionSpecDatasetBuilder(model)
    dataset, _ = builder.build()
    graph = dataset.default_graph

    driver_node = builder.root_uri(f"drv-{handler.motion.name}", owner=handler)
    joint_force_node = builder.root_uri("tau-j1", owner=handler)

    assert (driver_node, SLV["joint-force"], joint_force_node) in graph
    assert (joint_force_node, RDF.type, SLV.JointForce) in graph


def test_posture_controller_emits_joint_force_torque_signal() -> None:
    builder, graph, _ = _build_dataset("posture_controller.robmot")

    handler = builder.authored_handlers[0]
    controller = handler.controllers[0]
    driver_node = builder.root_uri(f"drv-{handler.motion.name}", owner=handler)
    joint_force_node = builder.root_uri("tau-ctrl-j2-posture", owner=handler)
    joint_position = handler.motion.context[0].declaration[0]
    joint_position_node = builder.node(joint_position)
    joint_target_node = builder.root_uri("joint-2", owner=handler.motion)

    assert (
        URIRef(controller.uri),
        CSTR_HDL["control-signal"],
        joint_force_node,
    ) in graph
    assert (driver_node, SLV["joint-force"], joint_force_node) in graph
    assert (joint_force_node, RDF.type, SLV.JointForce) in graph
    assert (joint_force_node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Torque) in graph
    assert (joint_force_node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M"]) in graph
    assert (joint_position_node, GEOM_REL.of, joint_target_node) in graph
    assert (joint_target_node, RDF.type, KC.Joint) in graph


def test_posture_joint_limit_constraints_emit_joint_force_and_error_signals() -> None:
    builder, graph, _ = _build_dataset("joint_limit_posture.robmot")

    handler = builder.authored_handlers[0]
    driver_node = builder.root_uri(f"drv-{handler.motion.name}", owner=handler)
    joint_force_nodes = list(graph.objects(driver_node, SLV["joint-force"]))
    assert len(joint_force_nodes) == 2

    for error_id in ("q-j2-err-m-joint-limits", "q-j4-err-m-joint-limits"):
        error_node = builder.root_uri(error_id, owner=handler.motion.while_)
        assert (error_node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND["Angle"]) in graph
        assert (error_node, QUDT_SCHEMA.unit, QUDT_UNIT["RAD"]) in graph


def test_pose_position_without_axis_emits_linear_distance_operation() -> None:
    builder, graph, _ = _build_model_dataset("sc1.robmot")

    motion = builder.authored_handlers[0].motion
    pose_node = builder.root_uri("pose-rightarm-shoulder-ee", owner=motion)
    distance_node = builder.root_uri("pose-rightarm-shoulder-ee.distance", owner=motion)
    op_node = builder.root_uri("compute-pose-rightarm-shoulder-ee.distance", owner=motion)
    constraint = _resolved_spec(motion.while_.constraints[-1])

    assert (distance_node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Distance) in graph
    assert (distance_node, QUDT_SCHEMA.unit, QUDT_UNIT.M) in graph
    assert (op_node, RDF.type, GEOM_OP.PoseToLinearDistance) in graph
    assert (op_node, GEOM_OP.pose, pose_node) in graph
    assert (op_node, GEOM_OP.distance, distance_node) in graph
    assert (URIRef(constraint.uri), RDF.type, CSTR.DistanceConstraint) in graph
    assert (URIRef(constraint.uri), CSTR.quantity, distance_node) in graph
