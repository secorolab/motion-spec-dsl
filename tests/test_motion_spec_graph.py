# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

from pathlib import Path

import pytest
from rdflib import URIRef

from motion_spec.namespace import CSTR_HDL, MAP, MOT, QUDT_QKIND, QUDT_SCHEMA, QUDT_UNIT, SLV
from motion_spec_dsl.generators.motion_spec_graph import MotionSpecDatasetBuilder
from motion_spec_dsl.generators.registration import motion_spec_metamodel


FIXTURES = Path(__file__).parent / "fixtures"


def _build_dataset(fixture: str):
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_file(FIXTURES / fixture)
    builder = MotionSpecDatasetBuilder(model)
    dataset, context = builder.build()
    return builder, dataset.default_graph, context


def test_standalone_builder_emits_motion_constraint_and_evaluator_nodes() -> None:
    builder, graph, context = _build_dataset("standalone_manipulator.robmot")

    motion = next(iter(builder.motion_scope.values())).motion
    constraint = builder.while_constraints[0]
    motion_node = builder.root_uri(f"motion-{motion.name}", owner=motion)
    evaluator_node = builder.root_uri(
        f"eval-{constraint.constraint.name}", owner=constraint.constraint.parent
    )
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
    bindings = builder.cartesian_force_bindings(handler)

    assert len(bindings) == 1

    force_quantity = builder.resolve_world_quantity(
        bindings[0].force_name,
        motion=handler.motion,
        handler=handler,
        reason="test fixture",
    )
    scalar_id = f"{force_quantity.name}.force.z"
    scalar_node = builder.root_uri(scalar_id, owner=force_quantity)
    view_node = builder.root_uri(f"view-{scalar_id}", owner=force_quantity)
    spec_node = builder.root_uri(bindings[0].spec_name, owner=handler)
    driver_node = builder.root_uri(f"drv-{handler.motion.name}", owner=handler)

    assert (scalar_node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Force) in graph
    assert (scalar_node, QUDT_SCHEMA.unit, QUDT_UNIT.N) in graph
    assert (view_node, MAP.subobject, scalar_node) in graph
    assert (spec_node, SLV.force, builder.node(force_quantity)) in graph
    assert (driver_node, SLV["cartesian-force"], spec_node) in graph


def test_standalone_builder_emits_acceleration_energy_and_solver_links() -> None:
    builder, graph, _ = _build_dataset("standalone_manipulator.robmot")

    handler = builder.authored_handlers[0]
    constraint = builder.while_constraints[0]
    motion = handler.motion
    driver_node = builder.root_uri(f"drv-{motion.name}", owner=handler)
    solver_node = builder.root_uri(f"slv-{motion.name}", owner=handler)

    assert constraint.acceleration_energy_id is not None

    energy_node = builder.root_uri(
        constraint.acceleration_energy_id, owner=constraint.constraint.parent
    )
    acc_node_id = f"acc-cstr-{constraint.quantity.name}.{constraint.property_name}.{constraint.axis}-{motion.name}"
    acc_node = builder.root_uri(acc_node_id, owner=motion)
    driver_acc_specs = list(graph.objects(driver_node, SLV["acceleration-constraint"]))

    assert (energy_node, QUDT_SCHEMA["quantity-kind"], builder.root_uri("AccelerationEnergy", owner=constraint.constraint.parent)) in graph
    assert (acc_node, SLV["acceleration-energy"], builder.root_uri(constraint.acceleration_energy_id, owner=motion)) in graph
    assert len(driver_acc_specs) == 1
    assert (solver_node, SLV["motion-drivers"], driver_node) in graph


def test_resolve_world_quantity_raises_on_ambiguous_name_fallback() -> None:
    builder, _, _ = _build_dataset("standalone_manipulator.robmot")
    quantity = next(iter(builder.world_quantities.values()))
    builder.world_quantities_by_name = {"dup": (quantity, quantity)}

    with pytest.raises(ValueError, match="Ambiguous world quantity reference 'dup'"):
        builder.resolve_world_quantity("dup", reason="test ambiguity")
