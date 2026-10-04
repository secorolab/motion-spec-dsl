# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Constraint handlers: their controllers, monitors, error evaluators and perturbations."""

from __future__ import annotations

from typing import Any

from rdf_utils.namespace import NS_MM_QUDT_QTY
from rdflib.namespace import PROV, RDF, RDFS, XSD
from rdflib.term import Literal, URIRef

from motion_spec_dsl.classes.constraint_handler import (
    ConstraintHandler,
    ControllerAlias,
    ControllerEntry,
    ControllerType,
    SaturationSpec,
    UntilMonitorRef,
    WhenMonitorRef,
)
from motion_spec_dsl.classes.constraints import (
    ConstraintAlias,
    ConstraintGroup,
    ConstraintSpecification,
    EqualityConstraint,
    GoalStatusConstraint,
    ViewForm,
    flatten_constraint_items,
    view_form,
)
from motion_spec_dsl.classes.context import (
    ContextQuantityAlias,
    QuantityType,
    WorldQuantity,
    WorldQuantityAlias,
    WorldQuantityType,
    geo_prop,
)
from motion_spec_dsl.classes.controller_semantics import (
    constraint_view_subspace,
    controller_command_record,
    controller_solver,
)
from motion_spec_dsl.classes.motion_spec import GuardedMotion
from motion_spec_dsl.classes.units import DSL_UNITS
from motion_spec_dsl.classes.views import constraint_kind
from motion_spec_dsl.rdf.common import alignment_id, constraint_scalar_id, evaluator_id
from motion_spec_dsl.rdf.constraints import (
    duration_node,
    section_expression,
    section_expression_type,
)
from motion_spec_dsl.rdf.emission import (
    Emission,
    add_quantity,
    declared_uri,
    emit_scalar_quantity,
    owned_uri,
)
from motion_spec_dsl.rdf.expressions import emit_context_ref_node
from motion_spec_dsl.rdf.geometry import (
    emit_angle_normalization,
    emit_profile_view_node,
    emit_wrench_coordinate,
    emit_zero_position_coordinate,
)
from motion_spec_dsl.rdf.model import ROS
from motion_spec_dsl.rdf.plans import resolve_constraint_quantity
from motion_spec_dsl.rdf.quantities import emit_context_members
from motion_spec_dsl.rdf_parser.vocab import (
    ALGO_EXT,
    APP,
    CSTR_EXT,
    CSTR_HDL,
    CSTR_HDL_EXT,
    EL,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_REL,
    QUDT_SCHEMA,
    RBDYN_OP,
    RBDYN_OP_EXT,
    SENSORS,
    SIM,
    SLV,
    TIME,
)

# Subspaces whose pose equality the pose-difference machinery evaluates, not a scalar error.
_POSE_COMMAND_SUBSPACES = {"pose", "position", "orientation", "distance", "rotation"}


def error_scalar(em: Emission, spec: ConstraintSpecification, world_qtys: dict) -> tuple[Any, str | None, Any]:
    """A constraint's (world quantity, along-path speed id, scalar kind) its error is measured in.

    The quantity is None for an along-path, context or expression view.
    """
    operand = spec.view.moving or spec.view.progress
    if operand is not None:
        path = operand.path.quantity
        path = path.ref if isinstance(path, ContextQuantityAlias) else path
        return None, f"{path.name}-along-speed", constraint_kind(spec)
    return resolve_constraint_quantity(em, spec, world_qtys), None, constraint_kind(spec)


def emit_controller_base(em: Emission, ctrl_node: URIRef, ctrl: ControllerEntry, command) -> None:
    """A controller's type and gains: PID, impedance, or feed-forward."""
    em.graph.add((ctrl_node, RDF.type, CSTR_HDL.Controller))
    # Only a derived moment is stated beyond what was authored, so ids and force commands stay.
    command_type = ctrl.command_type
    if command_type is None and command.is_moment_command:
        command_type = command.command_type
    if command_type is not None:
        em.graph.add((ctrl_node, APP["command-type"], Literal(command_type.value)))
    params = ctrl.params
    if ctrl.type == ControllerType.PID:
        em.graph.add((ctrl_node, RDF.type, CSTR_HDL.ProportionalIntegralDerivative))
        for predicate, gain in (
            (CSTR_HDL["proportional-gain"], params.kp),
            (CSTR_HDL["integral-gain"], params.ki),
            (CSTR_HDL["derivative-gain"], params.kd),
        ):
            if gain is not None:
                em.graph.add((ctrl_node, predicate, Literal(float(gain), datatype=XSD.double)))
        if params.decay is not None:
            em.graph.add((ctrl_node, RDF.type, CSTR_HDL.DecayingIntegralTerm))
            em.graph.add((ctrl_node, CSTR_HDL["decay-rate"], Literal(float(params.decay), datatype=XSD.double)))
    elif ctrl.type == ControllerType.Impedance:
        em.graph.add((ctrl_node, RDF.type, CSTR_HDL.ImpedanceController))
        for predicate, suffix, gain in (
            (CSTR_HDL.stiffness, "stiffness", params.stiffness),
            (CSTR_HDL.damping, "damping", params.damping),
            (CSTR_HDL["integral-gain"], "integral", params.ki),
        ):
            if gain is not None:
                gain_node = URIRef(f"{ctrl_node}-{suffix}")
                em.graph.add((gain_node, QUDT_SCHEMA.value, Literal(float(gain), datatype=XSD.double)))
                em.graph.add((ctrl_node, predicate, gain_node))
    else:
        em.graph.add((ctrl_node, RDF.type, CSTR_HDL_EXT.FeedForwardController))


def emit_saturation(
    em: Emission, owner_node: URIRef, node: URIRef, saturation: SaturationSpec, signal: URIRef, owner: Any
) -> None:
    """A saturation clamping SIGNAL in place, to a magnitude or to a lower and upper bound."""
    em.graph.add((node, RDF.type, ALGO_EXT.Saturation))
    em.graph.add((node, ALGO_EXT["in"], signal))
    em.graph.add((node, ALGO_EXT.out, signal))
    if saturation.maximum is not None:
        em.graph.add((node, ALGO_EXT["maximum-absolute-value"], emit_context_ref_node(em, saturation.maximum, owner, "max")))
    else:
        em.graph.add((node, ALGO_EXT["lower-bound"], emit_context_ref_node(em, saturation.lower, owner, "lower")))
        em.graph.add((node, ALGO_EXT["upper-bound"], emit_context_ref_node(em, saturation.upper, owner, "upper")))
    em.graph.add((owner_node, ALGO_EXT.limits, node))


def emit_controller_limits(em: Emission, controller_node: URIRef, controller: ControllerEntry, command) -> None:
    """A controller's authored output and integral saturations and its error normalization."""
    params = controller.params
    if params.output_saturation is not None:
        output = declared_uri(f"output-{controller.name}", controller)
        add_quantity(em, output, command.command_type)
        emit_saturation(
            em,
            controller_node,
            declared_uri(f"sat-output-{controller.name}", controller),
            params.output_saturation,
            output,
            controller,
        )
    if params.error_normalization is not None:
        emit_angle_normalization(
            em, controller_node, params.error_normalization, controller, f"err-norm-{controller.name}"
        )
    if params.integral_saturation is not None:
        integral = declared_uri(f"integral-state-{controller.name}", controller)
        em.graph.add((integral, RDF.type, QUDT_SCHEMA.Quantity))
        emit_saturation(
            em,
            controller_node,
            declared_uri(f"sat-integral-{controller.name}", controller),
            params.integral_saturation,
            integral,
            controller,
        )


def constraint_error_node(
    em: Emission,
    spec: ConstraintSpecification,
    motion: GuardedMotion,
    world_qtys: dict,
    seen_error_ids: set[str],
    error_ids: dict[str, str],
) -> URIRef:
    """The error signal a constraint writes, its quantity minted the first time it is named.

    A goal status reads its act's status slot and a timing constraint its elapsed clock; any
    other takes the id in ERROR_IDS, so a monitor reads the error a controller already writes.
    """
    if isinstance(spec, GoalStatusConstraint):
        return URIRef(f"{spec.act.uri}.status")
    if spec.view.elapsed is not None:
        return owned_uri(em, f"{spec.name}-elapsed", motion)
    qty, _along, scalar_t = error_scalar(em, spec, world_qtys)
    error_id = error_ids.get(spec.uri, f"{evaluator_id(spec)}-err")
    error_node = owned_uri(em, error_id, spec.parent)
    if error_id in seen_error_ids:
        return error_node
    seen_error_ids.add(error_id)
    add_quantity(em, error_node, scalar_t)
    of_v = geo_prop(qty.props, "of") if scalar_t == QuantityType.Pose and qty is not None else None
    wrt_v = geo_prop(qty.props, "wrt") if of_v else None
    if of_v and wrt_v:
        em.graph.add((error_node, GEOM_REL.of, owned_uri(em, of_v, qty)))
        em.graph.add((error_node, GEOM_REL["with-respect-to"], owned_uri(em, wrt_v, qty)))
        em.graph.add((error_node, GEOM_COORD["as-seen-by"], owned_uri(em, wrt_v, qty)))
    return error_node


def emit_error_evaluator(
    em: Emission, handler_node: URIRef, spec: ConstraintSpecification, error_node: URIRef, seen_eval_ids: set[str]
) -> None:
    """The evaluator linking a constraint to its error, once per id, attached to the handler."""
    eval_id = evaluator_id(spec)
    eval_node = owned_uri(em, eval_id, spec.parent)
    if eval_id not in seen_eval_ids:
        seen_eval_ids.add(eval_id)
        em.graph.add((eval_node, RDF.type, CSTR_HDL.ConstraintEvaluator))
        em.graph.add((eval_node, RDF.type, CSTR_HDL.ErrorEvaluator))
        em.graph.add((eval_node, CSTR_HDL.constraint, URIRef(spec.uri)))
        em.graph.add((eval_node, CSTR_HDL.error, error_node))
    em.graph.add((handler_node, CSTR_HDL.evaluators, eval_node))


def emit_ros_publication(em: Emission, monitor, monitor_node: URIRef, watched: list[URIRef]) -> None:
    """The goal a monitor answers, and the channel, message and field rows it publishes."""
    if monitor.answer is not None:
        # A member of the monitor, so the same monitor may also publish: one node, one message.
        block, action = monitor.answer
        answer_node = URIRef(f"{monitor.uri}.answer")
        em.graph.add((monitor_node, RDFS.member, answer_node))
        em.graph.add((answer_node, RDF.type, ROS.Action))
        em.graph.add((answer_node, ROS["channel-name"], Literal(action.server.channel_name)))
        em.graph.add((answer_node, ROS["type-name"], Literal(action.server.type_name)))
        outcome = URIRef(f"{answer_node}.outcome")
        em.graph.add((answer_node, RDFS.member, outcome))
        em.graph.add((outcome, RDF.value, Literal(f"STATUS_{action.outcome.upper()}")))
        for condition in watched if block.state == "satisfied" else []:
            em.graph.add((outcome, CSTR_EXT["has-constraint"], condition))
        for row_index, (path, value) in enumerate(action.assignments):
            row = URIRef(f"{answer_node}.f{row_index}")
            em.graph.add((answer_node, RDFS.member, row))
            em.graph.add((row, ROS["field-path"], Literal(path)))
            em.graph.add((row, RDF.value, Literal(value)))
    if monitor.occurrence_topic is not None:
        # Each announced event is a whole message, with no value to tell it from a field row.
        em.graph.add((monitor_node, RDF.type, ROS.Topic))
        em.graph.add((monitor_node, ROS["channel-name"], Literal(monitor.occurrence_topic.channel_name)))
        em.graph.add((monitor_node, ROS["type-name"], Literal(monitor.occurrence_topic.type_name)))
        for event in monitor.announced_events:
            em.graph.add((monitor_node, RDFS.member, URIRef(event.uri)))
        return
    if monitor.topic is None:
        return
    em.graph.add((monitor_node, RDF.type, ROS.Topic))
    em.graph.add((monitor_node, ROS["channel-name"], Literal(monitor.topic.channel_name)))
    em.graph.add((monitor_node, ROS["type-name"], Literal(monitor.topic.type_name)))
    rows = []
    for block, action in monitor.published:
        if action.rate is not None:
            rate = URIRef(f"{monitor.uri}.rate")
            emit_scalar_quantity(
                em, rate, float(action.rate.value), NS_MM_QUDT_QTY["Frequency"], DSL_UNITS[action.rate.unit].iri
            )
            em.graph.add((monitor_node, SENSORS["update-rate"], rate))
        # A violated state's rows are the otherwise, under no condition.
        conditions = watched if block.state == "satisfied" else []
        rows += [(path, value, conditions) for path, value in action.assignments]
    for row_index, (path, value, conditions) in enumerate(rows):
        row = URIRef(f"{monitor.uri}.f{row_index}")
        em.graph.add((monitor_node, RDFS.member, row))
        em.graph.add((row, ROS["field-path"], Literal(path)))
        em.graph.add((row, RDF.value, Literal(value)))
        for condition in conditions:
            em.graph.add((row, CSTR_EXT["has-constraint"], condition))


def emit_constraint_handler(
    em: Emission,
    handler: ConstraintHandler,
    motion: GuardedMotion,
    world_qtys: dict,
    shared_specs: set[ConstraintSpecification],
    handler_order: int,
) -> None:
    """A handler: its controllers, monitors, evaluators, event loop and perturbations."""
    handler_node = URIRef(handler.uri)
    em.graph.add((handler_node, RDF.type, CSTR_HDL.ConstraintHandler))
    emit_context_members(em, handler_node, handler)
    em.graph.add((handler_node, APP.order, Literal(handler_order)))
    em.graph.add((handler_node, CSTR_HDL.motion, owned_uri(em, f"motion-{motion.name}", motion)))
    # A motion meant to keep running never fires the event that would bind it to a state.
    if handler.runs_in is not None:
        em.graph.add((handler_node, CSTR_HDL_EXT["runs-in-state"], URIRef(handler.runs_in.uri)))
    event_loop_node = URIRef(f"{handler.uri}.event-loop")
    seen_error_ids: set[str] = set()
    seen_eval_ids: set[str] = set()
    error_ids: dict[str, str] = {}
    # Constraints a controller drives or a monitor watches; the while sweep evaluates the rest.
    claimed: set[str] = set()
    for controller_order, item in enumerate(handler.controllers):
        ctrl = item.ref.controller if isinstance(item, ControllerAlias) else item
        spec = ctrl.params.constraint.constraint
        if spec.disabled:
            continue
        claimed.add(spec.uri)
        qty, along_id, _scalar_t = error_scalar(em, spec, world_qtys)
        quantityless = qty is None and along_id is None
        subspace = None if quantityless else constraint_view_subspace(spec)
        command = controller_command_record(ctrl)
        ctrl_node = URIRef(ctrl.uri)
        emit_controller_base(em, ctrl_node, ctrl, command)
        em.graph.add((ctrl_node, APP.order, Literal(controller_order)))
        em.graph.add((ctrl_node, CSTR_HDL.constraint, URIRef(spec.uri)))
        derivative = ctrl.params.measured_derivative
        if derivative is not None and isinstance(derivative.quantity, WorldQuantity):
            # A bare subspace leaves the per-axis controller to read the component it drives.
            if derivative.axis is not None:
                derivative_node = emit_profile_view_node(em, derivative, handler)
            elif isinstance(derivative.quantity, WorldQuantityAlias):
                derivative_node = URIRef(derivative.quantity.ref.uri)
            else:
                derivative_node = URIRef(derivative.quantity.uri)
            em.graph.add((ctrl_node, CSTR_HDL["measured-velocity"], derivative_node))
        solver = controller_solver(handler, ctrl)
        if solver is not None:
            em.graph.add((ctrl_node, CSTR_HDL_EXT.solver, owned_uri(em, f"{solver.name}-{handler.name}", handler)))
        emit_controller_limits(em, ctrl_node, ctrl, command)
        # A context or expression view names no scalar, so its error is named after its evaluator;
        # an alignment by its operands and cone, so two angles on one pose keep two errors.
        if along_id is not None:
            sid = along_id
        elif quantityless:
            sid = evaluator_id(spec)
        elif subspace == "alignment":
            sid = alignment_id(qty, spec)
        else:
            sid = constraint_scalar_id(qty, spec)
        error_id = f"{sid}-err" if spec in shared_specs else f"{sid}-err-{motion.name}"
        feed_forward = ctrl.type == ControllerType.FeedForward
        if (
            qty is not None
            and qty.type == WorldQuantityType.Pose
            and subspace in _POSE_COMMAND_SUBSPACES
            and spec.view.axis is None
            and command.controlled_axes
            and (isinstance(spec.expr, EqualityConstraint) or spec.view.on is not None)
            and view_form(spec) != ViewForm.DistanceBetween
        ):
            em.graph.add((handler_node, CSTR_HDL.controllers, ctrl_node))
            continue
        if not feed_forward:
            em.graph.add((ctrl_node, CSTR_HDL["error-signal"], owned_uri(em, error_id, spec.parent)))
        if feed_forward and isinstance(spec.expr, EqualityConstraint) and spec.expr.reference.quantity is not None:
            em.graph.add(
                (ctrl_node, CSTR_HDL_EXT["reference-signal"], URIRef(spec.expr.reference.quantity.uri))
            )
        em.graph.add((handler_node, CSTR_HDL.controllers, ctrl_node))
        if not feed_forward or isinstance(spec.expr, EqualityConstraint):
            error_ids[spec.uri] = error_id
            error_node = constraint_error_node(em, spec, motion, world_qtys, seen_error_ids, error_ids)
            emit_error_evaluator(em, handler_node, spec, error_node, seen_eval_ids)
    for monitor in handler.monitors:
        target = monitor.constraint
        group = target.constraint if not isinstance(target, (UntilMonitorRef, WhenMonitorRef)) else None
        if isinstance(group, ConstraintGroup) or isinstance(target, (UntilMonitorRef, WhenMonitorRef)):
            # A group or whole section aggregates its members and points at the node carrying its logic.
            if isinstance(group, ConstraintGroup):
                items = group.constraints
            else:
                section = target.motion.when if isinstance(target, WhenMonitorRef) else target.motion.until
                items = flatten_constraint_items(section.constraints)
            specs = [item.constraint if isinstance(item, ConstraintAlias) else item for item in items]
            specs = [spec for spec in specs if not spec.disabled]
            if isinstance(group, ConstraintGroup):
                guards = [URIRef(group.uri)]
            else:
                section_node, _node_type, _members = section_expression(em, target.motion, section)
                guards = [section_node] if section_node is not None else [URIRef(spec.uri) for spec in specs]
            claimed.update(spec.uri for spec in specs)
            errors = []
            for spec in specs:
                error_node = constraint_error_node(em, spec, target.motion, world_qtys, seen_error_ids, error_ids)
                emit_error_evaluator(em, handler_node, spec, error_node, seen_eval_ids)
                errors.append(error_node)
            if len(errors) == 1:
                error_node = errors[0]
            else:
                error_node = URIRef(f"{monitor.uri}.error")
                add_quantity(em, error_node, QuantityType.FreeVector)
        else:
            spec = target.constraint
            if spec.disabled:
                continue
            claimed.add(spec.uri)
            error_node = constraint_error_node(em, spec, target.motion, world_qtys, seen_error_ids, error_ids)
            emit_error_evaluator(em, handler_node, spec, error_node, seen_eval_ids)
            guards = [URIRef(spec.uri)]
        monitor_node = URIRef(monitor.uri)
        if monitor.trigger is not None:
            signal_node = URIRef(monitor.event.uri)
            em.graph.add((signal_node, RDF.type, EL.Event))
            em.graph.add((event_loop_node, EL["has-event"], signal_node))
        elif monitor.flag:
            signal_node = URIRef(f"{monitor.uri}.{monitor.flag}")
            em.graph.add((signal_node, RDF.type, EL.Flag))
            em.graph.add((event_loop_node, EL["has-flag"], signal_node))
        em.graph.add((monitor_node, RDF.type, CSTR_HDL.Monitor))
        em.graph.add((monitor_node, CSTR_HDL.error, error_node))
        for guard in guards:
            em.graph.add((monitor_node, CSTR_HDL.constraint, guard))
        emit_ros_publication(em, monitor, monitor_node, guards)
        if monitor.trigger is not None:
            em.graph.add((event_loop_node, RDF.type, EL.EventLoop))
            em.graph.add((monitor_node, RDF.type, CSTR_HDL.EdgeTriggeredMonitor))
            em.graph.add((monitor_node, CSTR_HDL.event, signal_node))
            em.graph.add((monitor_node, CSTR_HDL["event-queue"], event_loop_node))
            if monitor.fallback is not None:
                fallback = owned_uri(em, f"motion-{monitor.fallback.name}", monitor.fallback)
                em.graph.add((monitor_node, CSTR_HDL_EXT["fallback-motion"], fallback))
            if monitor.debounce_duration is not None:
                debounce = URIRef(f"{monitor.uri}.debounce")
                emit_scalar_quantity(
                    em, debounce, monitor.debounce_duration, NS_MM_QUDT_QTY["Time"], DSL_UNITS[monitor.debounce_unit].iri
                )
                em.graph.add((monitor_node, CSTR_HDL_EXT["debounce-duration"], debounce))
        elif monitor.flag:
            em.graph.add((monitor_node, RDF.type, CSTR_HDL.LevelTriggeredMonitor))
            em.graph.add((monitor_node, CSTR_HDL.flag, signal_node))
        em.graph.add((handler_node, CSTR_HDL.monitors, monitor_node))
    # An unclaimed `while` bound is still compared and logged, not left a constant nothing reads.
    for item in flatten_constraint_items(motion.while_.constraints):
        spec = item.constraint if isinstance(item, ConstraintAlias) else item
        if spec.disabled or spec.uri in claimed:
            continue
        error_node = constraint_error_node(em, spec, motion, world_qtys, seen_error_ids, error_ids)
        emit_error_evaluator(em, handler_node, spec, error_node, seen_eval_ids)
    for perturbation in handler.perturbations:
        emit_perturbation(em, perturbation, handler, handler_node, motion, world_qtys, seen_error_ids, error_ids, seen_eval_ids)


def emit_perturbation(
    em: Emission,
    perturbation,
    handler: ConstraintHandler,
    handler_node: URIRef,
    motion: GuardedMotion,
    world_qtys: dict,
    seen_error_ids: set[str],
    error_ids: dict[str, str],
    seen_eval_ids: set[str],
) -> None:
    """A perturbation: its wrench, the body it acts on, the state arming it, its gate and its window."""
    node = URIRef(perturbation.uri)
    em.graph.add((node, RDF.type, SIM.Perturbation))
    em.graph.add((node, PROV.wasDerivedFrom, handler_node))
    body_node = URIRef(str(perturbation.body.uri))
    em.graph.add((body_node, RDF.type, GEOM_ENT.SimplicialComplex))
    em.graph.add((node, SLV["attached-to"], body_node))
    if handler.runs_in is not None:
        em.graph.add((node, CSTR_HDL_EXT["runs-in-state"], URIRef(handler.runs_in.uri)))
    # The wrench is composed by the ops a force or moment command uses, in its direction's frame.
    direction = (perturbation.force_direction or perturbation.moment_direction).quantity
    direction = direction.ref if isinstance(direction, ContextQuantityAlias) else direction
    as_seen_by = owned_uri(em, geo_prop(direction.props, "as-seen-by") or geo_prop(direction.props, "wrt"), direction)
    point_node = declared_uri(f"point-{perturbation.name}", perturbation)
    position_node = declared_uri(f"position-{perturbation.name}", perturbation)
    emit_zero_position_coordinate(em, position_node, point_node, as_seen_by)
    # (kind, magnitude, direction, operator type, magnitude predicate) per stated part.
    parts = []
    if perturbation.force is not None:
        parts.append(
            ("force", perturbation.force, perturbation.force_direction, RBDYN_OP.WrenchFromPositionDirectionAndMagnitude, RBDYN_OP.magnitude)
        )
    if perturbation.moment is not None:
        parts.append(
            ("moment", perturbation.moment, perturbation.moment_direction, RBDYN_OP_EXT.WrenchFromDirectionAndMoment, RBDYN_OP_EXT.moment)
        )
    wrench_nodes = []
    for kind, magnitude, part_direction, op_type, magnitude_predicate in parts:
        magnitude_qty = magnitude.quantity
        direction_qty = part_direction.quantity
        wrench_node = declared_uri(f"wrench-{kind}-{perturbation.name}", perturbation)
        emit_wrench_coordinate(em, wrench_node, point_node, as_seen_by)
        op_node = declared_uri(f"compute-wrench-{kind}-{perturbation.name}", perturbation)
        em.graph.add((op_node, RDF.type, op_type))
        em.graph.add(
            (
                op_node,
                magnitude_predicate,
                URIRef((magnitude_qty.ref if isinstance(magnitude_qty, ContextQuantityAlias) else magnitude_qty).uri),
            )
        )
        em.graph.add(
            (
                op_node,
                RBDYN_OP.direction,
                URIRef((direction_qty.ref if isinstance(direction_qty, ContextQuantityAlias) else direction_qty).uri),
            )
        )
        if kind == "force":
            em.graph.add((op_node, RBDYN_OP.position, position_node))
        em.graph.add((op_node, RBDYN_OP.wrench, wrench_node))
        wrench_nodes.append(wrench_node)
    wrench = wrench_nodes[0]
    if len(wrench_nodes) > 1:
        wrench = declared_uri(f"wrench-{perturbation.name}", perturbation)
        emit_wrench_coordinate(em, wrench, point_node, as_seen_by)
        add_node = declared_uri(f"add-wrench-{perturbation.name}", perturbation)
        em.graph.add((add_node, RDF.type, RBDYN_OP.AddWrench))
        em.graph.add((add_node, RBDYN_OP.in1, wrench_nodes[0]))
        em.graph.add((add_node, RBDYN_OP.in2, wrench_nodes[1]))
        em.graph.add((add_node, RBDYN_OP.out, wrench))
    em.graph.add((node, SLV.force, wrench))
    if perturbation.duration is not None:
        em.graph.add(
            (node, TIME.hasDuration, duration_node(em, perturbation.duration, perturbation, f"duration-{perturbation.name}"))
        )
    members = [item.constraint if isinstance(item, ConstraintAlias) else item for item in perturbation.gate]
    members = [member for member in members if not member.disabled]
    for spec in members:
        error_node = constraint_error_node(em, spec, motion, world_qtys, seen_error_ids, error_ids)
        emit_error_evaluator(em, handler_node, spec, error_node, seen_eval_ids)
    node_type = section_expression_type(perturbation.gate_logic, len(members))
    holder = node
    if node_type is not None:
        holder = declared_uri(f"when-{perturbation.name}", perturbation)
        em.graph.add((holder, RDF.type, node_type))
        em.graph.add((node, CSTR_EXT["has-constraint"], holder))
    for spec in members:
        em.graph.add((holder, CSTR_EXT["has-constraint"], URIRef(spec.uri)))
