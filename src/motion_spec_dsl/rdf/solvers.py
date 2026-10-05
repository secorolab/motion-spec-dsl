# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""A handler's solvers, their motion drivers, and the command wrenches controllers feed them."""

from __future__ import annotations

from typing import Any

from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT
from rdflib.namespace import PROV, RDF, XSD
from rdflib.term import Literal, URIRef

from motion_spec_dsl.classes.constraint_handler import (
    CommandForwardingSolver,
    ConstraintHandler,
    ControllerAlias,
    ControllerEntry,
    ControllerType,
    MobilePlatformSolver,
    SolverAlias,
)
from motion_spec_dsl.classes.constraints import (
    ConstraintSpecification,
    ViewForm,
    view_form,
)
from motion_spec_dsl.classes.context import (
    JOINT_SCALAR_TYPES,
    ContextQuantity,
    ContextQuantityAlias,
    QuantityType,
    WorldQuantityType,
    geo_prop,
    scalar_type,
)
from motion_spec_dsl.classes.controller_semantics import (
    alignment_is_pointwise,
    constraint_view_subspace,
    controller_command_record,
    controller_solver,
)
from motion_spec_dsl.classes.motion_spec import GuardedMotion
from motion_spec_dsl.classes.units import DSL_UNITS
from motion_spec_dsl.rdf.common import AXIS_VECTORS, gradient_scalar_id
from motion_spec_dsl.rdf.emission import Emission, add_quantity, declared_uri, owned_uri
from motion_spec_dsl.rdf.expressions import emit_context_ref_node
from motion_spec_dsl.rdf.geometry import (
    emit_direction_coordinate,
    emit_pose_to_direction,
    emit_wrench_coordinate,
    emit_zero_position_coordinate,
)
from motion_spec_dsl.rdf.handlers import emit_saturation, error_scalar
from motion_spec_dsl.rdf.plans import resolve_constraint_quantity
from motion_spec_dsl.rdf_parser.vocab import (
    AGN,
    CSTR,
    CSTR_HDL,
    CSTR_HDL_EXT,
    GEOM_COORD,
    GEOM_ENT,
    KC_OP,
    KC_OP_EXT,
    KC_STAT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    RBDYN_OP,
    RBDYN_OP_EXT,
    SLV,
    SLV_EXT,
)

_MOBILE_PLATFORM_ALGORITHM_RDF = {
    "VelocityComposition": (SLV.VelocityCompositionSolver, SLV.velocity),
    "VelocityDistribution": (SLV_EXT.VelocityDistributionSolver, SLV.velocity),
    "ForceDistribution": (SLV.ForceDistributionSolver, SLV.force),
    "ForceComposition": (SLV_EXT.ForceCompositionSolver, SLV.force),
}
_SERIAL_CHAIN_ALGORITHM_RDF = {
    "ACHD": SLV.AccelerationConstrainedHybridDynamicsAlgorithm,
    "RNE": SLV.RecursiveNewtonEulerAlgorithm,
    "FPK": KC_OP.ForwardPositionKinematics,
    "FVK": KC_OP_EXT.ForwardVelocityKinematics,
}


def emit_force_command_wrench(
    em: Emission,
    ctrl: ControllerEntry,
    spec: ConstraintSpecification,
    qty: Any,
    motion: GuardedMotion,
) -> URIRef:
    """A force controller's command wrench, from its control signal along an axis or a runtime direction."""
    signal = declared_uri(f"force-{ctrl.name}", ctrl)
    add_quantity(em, signal, QuantityType.Force)
    em.graph.add((URIRef(ctrl.uri), CSTR_HDL["control-signal"], signal))
    axis = str(spec.view.axis) if spec.view.axis is not None else None
    expression = em.expression_plans.get(em.graph.value(URIRef(spec.uri), CSTR.quantity))
    if expression is not None:
        # An expression pushes along its own gradient, from the frame its terms share.
        as_seen_by, direction_node, point_node = expression.frame, expression.gradient, expression.frame
    else:
        as_seen_by = owned_uri(em, geo_prop(qty.props, "as-seen-by") or geo_prop(qty.props, "wrt"), qty)
        direction_node = declared_uri(f"direction-{ctrl.name}", ctrl)
        if qty.type == WorldQuantityType.Pose and constraint_view_subspace(spec) == "distance" and axis is None:
            emit_pose_to_direction(em, direction_node, qty, as_seen_by, motion, ctrl.name)
        elif axis is not None:
            emit_direction_coordinate(em, direction_node, as_seen_by, AXIS_VECTORS[axis])
        else:
            direction_node = owned_uri(em, gradient_scalar_id(qty, spec), motion)
        # The force acts where the constrained quantity is taken: its `of` frame.
        of_name = geo_prop(qty.props, "of")
        point_node = owned_uri(em, of_name, qty) if of_name is not None else as_seen_by
    position_node = declared_uri(f"position-force-{ctrl.name}", ctrl)
    emit_zero_position_coordinate(em, position_node, point_node, as_seen_by)
    # A declared wrench no sensor observes is the one the command realizes; its coordinate exists.
    commanded = (
        qty is not None
        and qty.type == WorldQuantityType.Wrench
        and geo_prop(qty.props, "ft-sensor") is None
        and geo_prop(qty.props, "estimated-from") is None
    )
    if commanded:
        wrench_node = URIRef(qty.uri)
    else:
        wrench_node = declared_uri(f"wrench-force-{ctrl.name}", ctrl)
        emit_wrench_coordinate(em, wrench_node, point_node, as_seen_by)
    op_node = declared_uri(f"compute-wrench-force-{ctrl.name}", ctrl)
    em.graph.add((op_node, RDF.type, RBDYN_OP.WrenchFromPositionDirectionAndMagnitude))
    em.graph.add((op_node, RBDYN_OP.magnitude, signal))
    em.graph.add((op_node, RBDYN_OP.direction, direction_node))
    em.graph.add((op_node, RBDYN_OP.position, position_node))
    em.graph.add((op_node, RBDYN_OP.wrench, wrench_node))
    return wrench_node


def emit_moment_command_wrench(
    em: Emission,
    ctrl: ControllerEntry,
    spec: ConstraintSpecification,
    qty: Any,
    command,
    handler: ConstraintHandler,
    motion: GuardedMotion,
) -> URIRef:
    """A moment controller's command wrench: one couple per commanded angular axis, summed.

    A view with no named axis, or a cone, is one couple along its expression's runtime gradient.
    A couple is reference-point independent, so no position enters the ops.
    """
    expression = em.expression_plans.get(em.graph.value(URIRef(spec.uri), CSTR.quantity))
    if expression is not None:
        as_seen_by = expression.frame
    else:
        as_seen_by = owned_uri(em, geo_prop(qty.props, "as-seen-by") or geo_prop(qty.props, "wrt"), qty)
    axes = [axis for _subspace, axis in command.controlled_axes if axis is not None]
    # A cone or an expression is one degree of freedom: its nominal axes are solver rows, not
    # written components.
    if expression is not None or (view_form(spec) == ViewForm.Alignment and not alignment_is_pointwise(spec)):
        axes = []
    # A couple needs no particular point; the frame's own origin is one the runtime can place.
    position_node = declared_uri(f"position-moment-{ctrl.name}", ctrl)
    emit_zero_position_coordinate(em, position_node, as_seen_by, as_seen_by)
    # (stem, direction node, signal node) per couple.
    couples = []
    if not axes:
        gradient = expression.gradient if expression is not None else owned_uri(em, gradient_scalar_id(qty, spec), motion)
        couples.append((ctrl.name, gradient, owned_uri(em, f"moment-{ctrl.name}", handler)))
    for axis in axes:
        direction_node = declared_uri(f"direction-moment-{ctrl.name}-ang-{axis}", ctrl)
        emit_direction_coordinate(em, direction_node, as_seen_by, AXIS_VECTORS[axis])
        signal_name = f"moment-{ctrl.name}-ang-{axis}" if len(axes) > 1 else f"moment-{ctrl.name}"
        couples.append((f"{ctrl.name}-ang-{axis}", direction_node, owned_uri(em, signal_name, handler)))
    wrench_nodes = []
    for stem, direction_node, signal in couples:
        add_quantity(em, signal, QuantityType.Torque)
        em.graph.add((URIRef(ctrl.uri), CSTR_HDL["control-signal"], signal))
        wrench_node = declared_uri(f"wrench-moment-{stem}", ctrl)
        emit_wrench_coordinate(em, wrench_node, as_seen_by, as_seen_by)
        op_node = declared_uri(f"compute-wrench-moment-{stem}", ctrl)
        em.graph.add((op_node, RDF.type, RBDYN_OP_EXT.WrenchFromDirectionAndMoment))
        em.graph.add((op_node, RBDYN_OP_EXT.moment, signal))
        em.graph.add((op_node, RBDYN_OP.direction, direction_node))
        em.graph.add((op_node, RBDYN_OP.wrench, wrench_node))
        wrench_nodes.append(wrench_node)
    total = wrench_nodes[0]
    for index, addend in enumerate(wrench_nodes[1:], start=1):
        sum_node = declared_uri(f"wrench-moment-{ctrl.name}-sum-{index}", ctrl)
        emit_wrench_coordinate(em, sum_node, as_seen_by, as_seen_by)
        add_node = declared_uri(f"add-wrench-{ctrl.name}-{index}", ctrl)
        em.graph.add((add_node, RDF.type, RBDYN_OP.AddWrench))
        em.graph.add((add_node, RBDYN_OP.in1, total))
        em.graph.add((add_node, RBDYN_OP.in2, addend))
        em.graph.add((add_node, RBDYN_OP.out, sum_node))
        total = sum_node
    return total


def emit_cartesian_force_spec(em: Emission, ctrl: ControllerEntry, wrench_node: URIRef, driver_node: URIRef) -> None:
    """A commanded wrench as the driver's Cartesian force specification on the `apply at` body."""
    spec_node = declared_uri(f"spec-{ctrl.name}", ctrl)
    em.graph.add((spec_node, RDF.type, SLV.CartesianForceSpecification))
    em.graph.add((spec_node, PROV.wasDerivedFrom, URIRef(ctrl.uri)))
    em.graph.add((spec_node, SLV.force, wrench_node))
    if ctrl.apply_at is not None:
        body_node = URIRef(str(ctrl.apply_at.uri))
        em.graph.add((body_node, RDF.type, GEOM_ENT.SimplicialComplex))
        em.graph.add((spec_node, SLV["attached-to"], body_node))
    em.graph.add((driver_node, SLV["cartesian-force"], spec_node))


def emit_solver_interfaces(
    em: Emission,
    handler: ConstraintHandler,
    motion: GuardedMotion,
    solver: Any,
    solver_node: URIRef,
    driver_node: URIRef,
    world_qtys: dict,
) -> None:
    """What each controller routed to SOLVER feeds it: a forwarded command, a wrench, or a joint force."""
    for item in handler.controllers:
        ctrl = item.ref.controller if isinstance(item, ControllerAlias) else item
        if controller_solver(handler, ctrl) is not solver:
            continue
        spec = ctrl.params.constraint.constraint
        qty, along_id, _scalar_t = error_scalar(em, spec, world_qtys)
        subspace = None if qty is None and along_id is None else constraint_view_subspace(spec)
        axis = str(spec.view.axis) if spec.view.axis is not None else None
        command = controller_command_record(ctrl)
        if solver.algorithm == "CommandForwarding" and ctrl.type == ControllerType.FeedForward:
            signal = declared_uri(f"cmd-{ctrl.name}", ctrl)
            add_quantity(em, signal, scalar_type(qty, subspace, axis) if qty else QuantityType.FreeVector)
            em.graph.add((solver_node, SLV.output, signal))
            continue
        if command.is_force_command:
            wrench_node = emit_force_command_wrench(em, ctrl, spec, qty, motion)
            emit_cartesian_force_spec(em, ctrl, wrench_node, driver_node)
        elif command.is_moment_command:
            wrench_node = emit_moment_command_wrench(em, ctrl, spec, qty, command, handler, motion)
            emit_cartesian_force_spec(em, ctrl, wrench_node, driver_node)
        # ACHD takes a joint force as its feed-forward torque; RNE adds it to the torque it computes.
        if (
            solver.algorithm in {"ACHD", "RNE"}
            and command.is_posture_torque_command
            and qty is not None
            and qty.type in JOINT_SCALAR_TYPES
        ):
            torque_node = owned_uri(em, f"tau-{ctrl.name}", handler)
            em.graph.add((URIRef(ctrl.uri), CSTR_HDL["control-signal"], torque_node))
            for rdf_type in (QUDT_SCHEMA.Quantity, KC_STAT.JointReference, KC_STAT.JointForce, KC_STAT.JointForceCoordinate):
                em.graph.add((torque_node, RDF.type, rdf_type))
            em.graph.add((torque_node, QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND.Torque))
            em.graph.add((torque_node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M"]))
            joint_node = owned_uri(em, geo_prop(qty.props, "joint"), qty)
            em.graph.add((torque_node, KC_STAT["of-joint"], joint_node))
            spec_node = declared_uri(f"jf-spec-{ctrl.name}", ctrl)
            em.graph.add((spec_node, RDF.type, SLV.JointForceSpecification))
            em.graph.add((spec_node, PROV.wasDerivedFrom, URIRef(ctrl.uri)))
            em.graph.add((spec_node, SLV.force, torque_node))
            em.graph.add((solver_node, SLV.output, URIRef(qty.uri)))
            em.graph.add((spec_node, SLV["attached-to"], joint_node))
            em.graph.add((driver_node, SLV["joint-force"], spec_node))


def emit_platform_force_drivers(
    em: Emission,
    handler: ConstraintHandler,
    motion: GuardedMotion,
    solver: MobilePlatformSolver,
    solver_node: URIRef,
    world_qtys: dict,
) -> None:
    """The command wrenches of the force and moment controllers routed to a force distribution."""
    driver_node = owned_uri(em, f"driver-{solver.name}-{handler.name}", handler)
    em.graph.add((driver_node, RDF.type, SLV.MotionDrivers))
    em.graph.add((driver_node, PROV.wasDerivedFrom, URIRef(handler.uri)))
    em.graph.add((solver_node, SLV["motion-drivers"], driver_node))
    for item in handler.controllers:
        ctrl = item.ref.controller if isinstance(item, ControllerAlias) else item
        if controller_solver(handler, ctrl) is not solver:
            continue
        spec = ctrl.params.constraint.constraint
        command = controller_command_record(ctrl)
        if command.is_moment_command:
            qty = resolve_constraint_quantity(em, spec, world_qtys)
            wrench_node = emit_moment_command_wrench(em, ctrl, spec, qty, command, handler, motion)
        elif command.is_force_command:
            qty = resolve_constraint_quantity(em, spec, world_qtys)
            # A platform's twist is stated in context; its frames give the axis as a world twist's would.
            context_qty = spec.view.quantity
            if qty is None and isinstance(context_qty, ContextQuantity):
                context_qty = context_qty.ref if isinstance(context_qty, ContextQuantityAlias) else context_qty
                if context_qty.props is not None:
                    qty = context_qty
            wrench_node = emit_force_command_wrench(em, ctrl, spec, qty, motion)
        else:
            continue
        emit_cartesian_force_spec(em, ctrl, wrench_node, driver_node)


def emit_solvers(em: Emission, handler: ConstraintHandler, motion: GuardedMotion, world_qtys: dict) -> None:
    """A handler's solvers by mechanism: serial-chain dynamics or kinematics, mobile platform, or forwarding."""
    solvers = [item.ref.solver if isinstance(item, SolverAlias) else item for item in handler.solvers]
    for solver in solvers:
        # A solver belongs to the handler running it: two handlers of one motion drive their own.
        solver_node = owned_uri(em, f"{solver.name}-{handler.name}", handler)
        # The declared solvers are the runtimes; a monitor-only arm has no controller routing to it.
        em.graph.add((URIRef(handler.uri), CSTR_HDL_EXT["runs-solver"], solver_node))
        em.graph.add((solver_node, AGN["of-agent"], URIRef(solver.agent.uri)))
        if isinstance(solver, MobilePlatformSolver):
            em.graph.add((solver_node, SLV.configuration, Literal(solver.configuration)))
            rdf_class, predicate = _MOBILE_PLATFORM_ALGORITHM_RDF[solver.algorithm]
            em.graph.add((solver_node, RDF.type, rdf_class))
            if solver.quantity.quantity is not None:
                em.graph.add((solver_node, predicate, URIRef(solver.quantity.quantity.uri)))
            # A distribution is fed wrenches: a bare scalar has no direction to distribute.
            if solver.algorithm == "ForceDistribution":
                emit_platform_force_drivers(em, handler, motion, solver, solver_node, world_qtys)
            continue
        driver_stem = f"{solver.name}-{handler.name}" if len(solvers) > 1 else handler.name
        driver_node = owned_uri(em, f"driver-{driver_stem}", handler)
        em.graph.add((driver_node, RDF.type, SLV.MotionDrivers))
        em.graph.add((driver_node, PROV.wasDerivedFrom, URIRef(handler.uri)))
        if isinstance(solver, CommandForwardingSolver):
            em.graph.add((solver_node, RDF.type, SLV_EXT.CommandForwardingSolver))
            em.graph.add((solver_node, SLV["motion-drivers"], driver_node))
            emit_solver_interfaces(em, handler, motion, solver, solver_node, driver_node, world_qtys)
            continue
        em.graph.add((solver_node, RDF.type, SLV.SolverWithInputAndOutput))
        em.graph.add((solver_node, SLV.solver, _SERIAL_CHAIN_ALGORITHM_RDF[solver.algorithm]))
        gravity = solver.gravity_value
        if gravity is not None and gravity.ref is not None:
            em.graph.add((solver_node, SLV.gravity, URIRef(gravity.ref.quantity.uri)))
        elif gravity is not None:
            gravity_node = owned_uri(em, f"gravity-value-{solver.name}", handler)
            add_quantity(em, gravity_node, QuantityType.FreeVector)
            em.graph.add((gravity_node, RDF.type, GEOM_COORD.VectorXYZ))
            for label, element in zip(("x", "y", "z"), gravity.coords.values):
                if element.ref is not None:
                    component = emit_context_ref_node(em, element.ref, handler, f"gravity-{label}")
                else:
                    component = Literal(float(element.value), datatype=XSD.double)
                em.graph.add((gravity_node, GEOM_COORD[label], component))
            em.graph.remove((gravity_node, QUDT_SCHEMA.unit, None))
            em.graph.add((gravity_node, QUDT_SCHEMA.unit, DSL_UNITS[gravity.unit].iri))
            em.graph.add((solver_node, SLV.gravity, gravity_node))
        for entry in solver.limits.entries if solver.limits is not None else []:
            signal = owned_uri(em, f"torque-output-{solver.name}", handler)
            add_quantity(em, signal, QuantityType.Torque)
            emit_saturation(
                em, solver_node, owned_uri(em, f"sat-{entry.target}-{solver.name}", handler), entry.saturation, signal, solver
            )
        em.graph.add((solver_node, SLV["motion-drivers"], driver_node))
        emit_solver_interfaces(em, handler, motion, solver, solver_node, driver_node, world_qtys)
