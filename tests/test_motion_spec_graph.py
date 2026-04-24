# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

from pathlib import Path
from typing import cast

import pytest
from rdflib import URIRef
from rdflib.namespace import RDF

from motion_spec.namespace import CSTR_HDL, GEOM_ENT, GEOM_REL, MAP, MOT, QUDT_QKIND, QUDT_SCHEMA, QUDT_UNIT, SLV
from motion_spec_dsl.generators.motion_spec_graph import (
    AccelerationConstraintInterface,
    CartesianForceInterface,
    MotionSpecDatasetBuilder,
)
from motion_spec_dsl.generators.registration import motion_spec_metamodel


FIXTURES = Path(__file__).parent / "fixtures"


def _build_dataset(fixture: str):
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_file(FIXTURES / fixture)
    builder = MotionSpecDatasetBuilder(model)
    dataset, context = builder.build()
    return builder, dataset.default_graph, cast(dict[str, str], context)


def test_standalone_builder_emits_motion_constraint_and_evaluator_nodes() -> None:
    builder, graph, context = _build_dataset("standalone_manipulator.robmot")

    motion = next(iter(builder.motion_scope.values())).motion
    constraint = builder.controlled_constraints[0]
    motion_node = builder.root_uri(f"motion-{motion.name}", owner=motion)
    evaluator_node = builder.root_uri(
        f"eval-{constraint.constraint.name}", owner=constraint.constraint.parent
    )
    assert constraint.error_signal_id is not None
    error_node = builder.root_uri(
        constraint.error_signal_id, owner=constraint.constraint.parent
    )

    assert context["app"] == motion.ns.uri
    assert (motion_node, MOT["while"], URIRef(constraint.constraint.uri)) in graph
    assert (evaluator_node, CSTR_HDL.constraint, URIRef(constraint.constraint.uri)) in graph
    assert (evaluator_node, CSTR_HDL.error, error_node) in graph
    assert (error_node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.LinearVelocity) in graph


def test_force_controller_builder_emits_force_scalar_view_and_solver_specs() -> None:
    builder, graph, _ = _build_dataset("force_controller.robmot")

    handler = builder.authored_handlers[0]
    interfaces = [
        interface
        for interface in builder.solver_interfaces(handler)
        if isinstance(interface, CartesianForceInterface)
    ]

    assert len(interfaces) == 1

    force_quantity = builder.resolve_world_quantity(
        interfaces[0].force_name,
        motion=handler.motion,
        handler=handler,
        reason="test fixture",
    )
    scalar_id = f"{force_quantity.name}.force.z"
    scalar_node = builder.root_uri(scalar_id, owner=force_quantity)
    view_node = builder.root_uri(f"view-{scalar_id}", owner=force_quantity)
    spec_node = builder.root_uri(interfaces[0].node_id, owner=handler)
    driver_node = builder.root_uri(f"drv-{handler.motion.name}", owner=handler)

    assert (scalar_node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Force) in graph
    assert (scalar_node, QUDT_SCHEMA.unit, QUDT_UNIT.N) in graph
    assert (view_node, MAP.subobject, scalar_node) in graph
    assert (spec_node, SLV.force, builder.node(force_quantity)) in graph
    assert (driver_node, SLV["cartesian-force"], spec_node) in graph


def test_standalone_builder_emits_acceleration_energy_and_solver_links() -> None:
    builder, graph, _ = _build_dataset("standalone_manipulator.robmot")

    handler = builder.authored_handlers[0]
    constraint = builder.controlled_constraints[0]
    motion = handler.motion
    driver_node = builder.root_uri(f"drv-{motion.name}", owner=handler)
    solver_node = builder.root_uri(f"slv-{motion.name}", owner=handler)
    dispatch = builder.controller_dispatches(handler)[0][2]
    acc_interface = next(
        interface
        for interface in dispatch.interfaces
        if isinstance(interface, AccelerationConstraintInterface)
    )

    energy_node = builder.root_uri(dispatch.signal.node_id, owner=motion)
    acc_node_id = acc_interface.node_id
    acc_node = builder.root_uri(acc_node_id, owner=motion)
    driver_acc_specs = list(graph.objects(driver_node, SLV["acceleration-constraint"]))

    assert (energy_node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.AccelerationEnergy) in graph
    assert (energy_node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M2-PER-SEC2"]) in graph
    assert (acc_node, SLV["acceleration-energy"], builder.root_uri(dispatch.signal.node_id, owner=motion)) in graph
    assert len(driver_acc_specs) == 1
    assert (solver_node, SLV["motion-drivers"], driver_node) in graph


def test_reused_constraint_emits_shared_uri_and_error_signal() -> None:
    builder, graph, _ = _build_dataset("reused_constraint.robmot")

    motions = {scope.motion.name: scope.motion for scope in builder.motion_scope.values()}
    motion_a_node = builder.root_uri("motion-move_a", owner=motions["move_a"])
    motion_b_node = builder.root_uri("motion-move_b", owner=motions["move_b"])
    shared_constraints = [
        constraint for constraint in builder.controlled_constraints if constraint.constraint.name == "shared"
    ]

    assert len(shared_constraints) == 2
    assert len({constraint.constraint.uri for constraint in shared_constraints}) == 1
    assert len({constraint.error_signal_id for constraint in shared_constraints}) == 1

    constraint_uri = URIRef(shared_constraints[0].constraint.uri)
    assert shared_constraints[0].error_signal_id is not None
    error_node = builder.root_uri(shared_constraints[0].error_signal_id, owner=shared_constraints[0].constraint.parent)

    assert (motion_a_node, MOT["while"], constraint_uri) in graph
    assert (motion_b_node, MOT["while"], constraint_uri) in graph
    assert (error_node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.LinearVelocity) in graph


def test_resolve_world_quantity_raises_on_ambiguous_name_fallback() -> None:
    builder, _, _ = _build_dataset("standalone_manipulator.robmot")
    quantity = next(iter(builder.world_quantities.values()))
    builder.world_quantities_by_name = {"dup": (quantity, quantity)}

    with pytest.raises(ValueError, match="Ambiguous world quantity reference 'dup'"):
        builder.resolve_world_quantity("dup", reason="test ambiguity")


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

    dispatch_a = builder.controller_dispatches(handler)[0][2]
    dispatch_b = builder.controller_dispatches(handler)[1][2]
    signal_a = builder.root_uri(dispatch_a.signal.node_id, owner=motion)
    signal_b = builder.root_uri(dispatch_b.signal.node_id, owner=motion)

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
    joint_position = builder.resolve_world_quantity(
        "q-j2",
        motion=handler.motion,
        handler=handler,
        reason="test posture joint position",
    )
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
    assert (joint_target_node, RDF.type, GEOM_ENT.SimplicialComplex) in graph
