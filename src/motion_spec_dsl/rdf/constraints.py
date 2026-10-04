# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Constraints, their references and bands, and the guarded motions whose sections hold them."""

from __future__ import annotations

from typing import Any

from rdf_utils.models.vocab import URI_QUDT_QK_LENGTH, URI_QUDT_QK_MASS
from rdf_utils.namespace import NS_MM_QUDT_QTY
from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT
from rdflib.namespace import PROV, RDF, SDO, XSD
from rdflib.term import Literal, URIRef

from motion_spec_dsl.classes.constraint_handler import ControllerEntry
from motion_spec_dsl.classes.constraints import (
    ANGLE_VIEW_FORMS,
    BilateralConstraint,
    ConstraintAlias,
    ConstraintGroup,
    ConstraintSpecification,
    EqualityConstraint,
    GoalStatusConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    OutsideConstraint,
    ViewForm,
    flatten_constraint_items,
    view_form,
)
from motion_spec_dsl.classes.context import (
    ContextQuantity,
    ContextQuantityAlias,
    ContextRef,
    QOpNode,
    QuantityType,
    ReferenceGeneratorType,
    WorldQuantityType,
    op_tree,
)
from motion_spec_dsl.classes.controller_semantics import constraint_view_subspace
from motion_spec_dsl.classes.dimensions import infer
from motion_spec_dsl.classes.motion_spec import GuardedMotion
from motion_spec_dsl.classes.units import DSL_UNITS, QUDT_KIND_BY_QUANTITY_TYPE
from motion_spec_dsl.classes.views import TOLERANCE_DEFAULT_KIND, constraint_kind, context_subspace_kind
from motion_spec_dsl.rdf.common import JOINT_TYPES, alignment_id, constraint_scalar_id
from motion_spec_dsl.rdf.emission import (
    Emission,
    add_quantity,
    declared_uri,
    emit_quantity_kind,
    emit_scalar_quantity,
    owned_uri,
)
from motion_spec_dsl.rdf.expressions import (
    emit_context_ref_node,
    emit_context_ref_view_node,
    emit_expr_leaf,
    emit_expression_gradient,
    emit_op_tree,
)
from motion_spec_dsl.rdf.geometry import emit_profile_view_node, view_node
from motion_spec_dsl.rdf.model import CONSTRAINT_TYPE_OVERRIDE, CSTR_TYPE_NAME, ROS, SCALAR_UNIT
from motion_spec_dsl.rdf.plans import resolve_constraint_quantity
from motion_spec_dsl.rdf.quantities import emit_context_members, emit_duration_measure, path_geometry_node
from motion_spec_dsl.rdf_parser.vocab import (
    ALGO_EXT,
    CSTR,
    CSTR_EXT,
    CSTR_HDL_EXT,
    GEOM_COORD,
    GEOM_OP_EXT,
    MOT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    SOSA,
    TIME,
)

_SECTION_EXPRESSION_SUFFIX = {
    CSTR_EXT.ConstraintDisjunction: "disjunction",
    CSTR_EXT.ConstraintConjunction: "conjunction",
}


def constraint_type_iri(scalar_t: Any) -> URIRef:
    """The domain constraint class for a scalar kind."""
    name = CSTR_TYPE_NAME.get(scalar_t, scalar_t)
    if name in CONSTRAINT_TYPE_OVERRIDE:
        namespace, local = CONSTRAINT_TYPE_OVERRIDE[name]
        return namespace[local]
    return CSTR[f"{name}Constraint"]


def emit_constraint_tolerance(
    em: Emission, node: URIRef, spec: ConstraintSpecification, motion: Any, scalar_t: Any
) -> None:
    """A constraint's `within` band, else for an equality the model-wide default of its unit family.

    Validation ensures every equality ends with one. A gate's region has an interior, so it never
    inherits the default and never trips early.
    """
    band = spec.tolerance
    owner, suffix = motion, f"{spec.name}-tolerance"
    is_equality = isinstance(spec.expr, EqualityConstraint) or spec.view.on is not None or spec.view.moving is not None
    if band is None and is_equality:
        # One node per kind, so every constraint taking the default points at the same band.
        band = em.tolerance_defaults.get(TOLERANCE_DEFAULT_KIND.get(scalar_t, scalar_t))
        owner, suffix = em.default_ns_owner, f"default-tolerance-{scalar_t}"
    if band is None:
        return
    if band.bare is None:
        band_node = emit_context_ref_node(em, band, owner, suffix)
    else:
        # An inline band takes its kind from the value it bounds.
        band_node = owned_uri(em, suffix, owner)
        if band_node not in em.emitted_bands:
            em.emitted_bands.add(band_node)
            em.graph.add((band_node, RDF.type, QUDT_SCHEMA.Quantity))
            if scalar_t == QuantityType.Distance:
                em.graph.add((band_node, RDF.type, GEOM_COORD.LinearDistanceCoordinate))
            emit_quantity_kind(em, band_node, QUDT_KIND_BY_QUANTITY_TYPE[scalar_t])
            em.graph.add((band_node, QUDT_SCHEMA.unit, DSL_UNITS[band.bare.unit].iri))
            em.graph.add((band_node, QUDT_SCHEMA.value, Literal(float(band.bare.value), datatype=XSD.double)))
    em.graph.add((node, CSTR_EXT.tolerance, band_node))


def constraint_reference_node(
    em: Emission, ref: ContextRef, owner: Any, suffix: str, subspace: str, axis: str | None, scalar_t: Any
) -> URIRef:
    """The node a constraint compares against; a whole pose or path part is named by its component view."""
    ref_node = emit_context_ref_node(em, ref, owner, suffix, scalar_t)
    quantity = ref.quantity
    if not isinstance(quantity, ContextQuantity) or axis is not None:
        return ref_node
    if isinstance(quantity, ContextQuantityAlias):
        quantity = quantity.ref
    if quantity.type in {QuantityType.Pose, ReferenceGeneratorType.Path} and subspace in {"position", "orientation"}:
        # The pooled relation is the target's too: naming it would state measured == reference.
        return emit_context_ref_view_node(em, quantity, subspace, None)
    return ref_node


def quantityless_view(
    em: Emission,
    spec: ConstraintSpecification,
    motion: Any,
    context_qty: ContextQuantity | None,
    axis: str | None,
    world_qtys: dict,
) -> URIRef:
    """The quantity node of a view naming no world quantity: a context quantity or an expression.

    A controlled expression also carries the gradient it is driven along.
    """
    if context_qty is not None:
        subspace = str(spec.view.subspace) if spec.view.subspace is not None else None
        if subspace is not None and context_subspace_kind(context_qty, subspace, axis) is not None:
            return emit_context_ref_view_node(em, context_qty, subspace, axis)
        return URIRef(context_qty.uri)
    tree = op_tree(spec.view.expr)
    if not isinstance(tree, QOpNode):
        return emit_expr_leaf(em, tree, motion, "expr")
    scalar_t = infer(tree)
    # The downstream id is the last segment alone, so it carries the motion.
    qty_node = URIRef(f"{spec.uri}/expr-{motion.name}-{spec.name}")
    emit_op_tree(em, tree, motion, qty_node, qty_node)
    em.graph.add((qty_node, RDF.type, QUDT_SCHEMA.Quantity))
    emit_quantity_kind(em, qty_node, QUDT_KIND_BY_QUANTITY_TYPE[scalar_t])
    em.graph.add((qty_node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(scalar_t, QUDT_UNIT.UNITLESS)))
    if spec in em.controller_by_spec:
        emit_expression_gradient(em, tree, qty_node, motion, world_qtys)
    return qty_node


def emit_constraints(
    em: Emission, motion: GuardedMotion, constraints: list[ConstraintSpecification], world_qtys: dict
) -> None:
    """Each elapsed, along-path, or quantity constraint of a motion, once."""
    seen = set()
    for spec in constraints:
        if spec.uri in seen:
            continue
        seen.add(spec.uri)
        node = URIRef(spec.uri)
        view = spec.view
        if view.elapsed is not None:
            emit_elapsed_constraint(em, node, spec, motion)
            continue
        if view.moving is not None or view.progress is not None:
            emit_along_path_constraint(em, node, spec, motion)
            continue
        qty = resolve_constraint_quantity(em, spec, world_qtys)
        axis = str(view.axis) if view.axis is not None else None
        if qty is None:
            context_qty = view.quantity if isinstance(view.quantity, ContextQuantity) else None
            if isinstance(context_qty, ContextQuantityAlias):
                context_qty = context_qty.ref
            if context_qty is None and view.expr is None:
                raise ValueError(f"constraint '{spec.name}' resolves to no world quantity")
            # A platform's twist and wrench are context 3-vectors a solver writes.
            subspace = constraint_view_subspace(spec) if context_qty is not None and view.subspace else None
            qty_node = quantityless_view(
                em, spec, motion, context_qty, axis if context_qty is not None else None, world_qtys
            )
        else:
            subspace = constraint_view_subspace(spec)
            pose = qty.type == WorldQuantityType.Pose
            if pose and axis is None and (
                subspace == "orientation"
                or subspace == "rotation"
                or (subspace in {"position", "distance"} and view_form(spec) != ViewForm.DistanceBetween)
            ):
                # The operand is coordinate plus component, not the relation poses over these frames share.
                qty_node = view_node(em, view, motion)
            elif axis is None and ((subspace == "pose" and pose) or qty.type in JOINT_TYPES):
                qty_node = URIRef(qty.uri)
            elif view_form(spec) in ANGLE_VIEW_FORMS:
                qty_node = owned_uri(em, alignment_id(qty, spec), motion)
            else:
                qty_node = owned_uri(em, constraint_scalar_id(qty, spec), motion)
        scalar_t = constraint_kind(spec)
        em.graph.add((node, RDF.type, CSTR.Constraint))
        em.graph.add((node, RDF.type, constraint_type_iri(scalar_t)))
        em.graph.add((node, CSTR.quantity, qty_node))
        if view.on is not None:
            # The path is evaluated where the frame already is, so it is the reference.
            path = view.on.path.quantity
            path = path.ref if isinstance(path, ContextQuantityAlias) else path
            em.graph.add((node, RDF.type, CSTR.EqualityConstraint))
            em.graph.add((node, CSTR["reference-value"], emit_context_ref_view_node(em, path, subspace, axis)))
            em.graph.add((node, GEOM_OP_EXT.path, path_geometry_node(path)))
            emit_constraint_tolerance(em, node, spec, motion, scalar_t)
            continue
        expr = spec.expr
        if isinstance(expr, EqualityConstraint):
            em.graph.add((node, RDF.type, CSTR.EqualityConstraint))
            ref_node = constraint_reference_node(
                em, expr.reference, motion, f"{spec.name}-ref", subspace, axis, scalar_t
            )
            profiled = em.profiled_controller_by_spec.get(spec)
            if profiled is not None:
                ref_node = emit_velocity_profile_reference(em, profiled, spec, ref_node, scalar_t, None)
            admittance = expr.reference.quantity
            if isinstance(admittance, ContextQuantityAlias):
                admittance = admittance.ref
            if isinstance(admittance, ContextQuantity) and admittance.type == ReferenceGeneratorType.Admittance:
                ref_node = emit_admittance_reference(em, em.controller_by_spec.get(spec), spec, admittance, scalar_t)
            em.graph.add((node, CSTR["reference-value"], ref_node))
        elif isinstance(expr, (GreaterThanConstraint, LessThanConstraint)):
            em.graph.add((node, RDF.type, CSTR.UnilateralConstraint))
            em.graph.add(
                (
                    node,
                    RDF.type,
                    CSTR.GreaterThanConstraint if isinstance(expr, GreaterThanConstraint) else CSTR.LessThanConstraint,
                )
            )
            threshold = emit_context_ref_node(em, expr.threshold, motion, f"{spec.name}-threshold", scalar_t)
            em.graph.add((node, CSTR.threshold, threshold))
        elif isinstance(expr, (BilateralConstraint, OutsideConstraint)):
            em.graph.add(
                (
                    node,
                    RDF.type,
                    CSTR.BilateralConstraint if isinstance(expr, BilateralConstraint) else CSTR_EXT.OutsideConstraint,
                )
            )
            lower = emit_context_ref_node(em, expr.lower, motion, f"{spec.name}-lower", scalar_t)
            upper = emit_context_ref_node(em, expr.upper, motion, f"{spec.name}-upper", scalar_t)
            em.graph.add((node, CSTR["lower-threshold"], lower))
            em.graph.add((node, CSTR["upper-threshold"], upper))
        emit_constraint_tolerance(em, node, spec, motion, scalar_t)


def emit_admittance_reference(
    em: Emission, ctrl: ControllerEntry, spec: ConstraintSpecification, admittance: ContextQuantity, scalar_t: Any
) -> URIRef:
    """A velocity-capped admittance filter from force to velocity, its state on the constraint's controller."""
    value = admittance.value
    out_node = declared_uri(f"{spec.name}-{ctrl.name}-admit-ref", ctrl)
    add_quantity(em, out_node, scalar_t)
    op_node = declared_uri(f"admit-{spec.name}-{ctrl.name}", ctrl)
    em.graph.add((op_node, RDF.type, ALGO_EXT.Admittance))
    em.graph.add((op_node, RDF.type, CSTR_HDL_EXT.SetpointGenerator))
    em.graph.add((op_node, ALGO_EXT["in"], emit_profile_view_node(em, value.force, admittance)))
    deadband_unit = DSL_UNITS[value.deadband_unit or "N"].iri
    # (term, value, kind, unit) per parameter.
    parameters = [
        (ALGO_EXT.mass, "mass", value.mass, URI_QUDT_QK_MASS, QUDT_UNIT["KiloGM"]),
        (ALGO_EXT.damping, "damping", value.damping, None, QUDT_UNIT["N-SEC-PER-M"]),
        (ALGO_EXT.stiffness, "stiffness", value.stiffness, None, QUDT_UNIT["N-PER-M"]),
        (
            ALGO_EXT["maximum-velocity"],
            "max-velocity",
            value.max_velocity,
            QUDT_QKIND.LinearVelocity,
            DSL_UNITS[value.max_velocity_unit or "m/s"].iri,
        ),
        # The excursion bound saturates how far the yield travels.
        (
            ALGO_EXT["maximum-absolute-value"],
            "max-excursion",
            value.max_excursion,
            URI_QUDT_QK_LENGTH,
            DSL_UNITS[value.max_excursion_unit or "m"].iri,
        ),
        # The deadband is an outside-band on the input force.
        (CSTR["lower-threshold"], "lower-threshold", -float(value.deadband), QUDT_QKIND.Force, deadband_unit),
        (CSTR["upper-threshold"], "upper-threshold", float(value.deadband), QUDT_QKIND.Force, deadband_unit),
        (
            ALGO_EXT["release-threshold"],
            "release-threshold",
            value.release_threshold,
            QUDT_QKIND.Force,
            DSL_UNITS[value.release_threshold_unit or value.deadband_unit or "N"].iri,
        ),
    ]
    for term, suffix, number, qkind, unit in parameters:
        em.graph.add((op_node, term, emit_scalar_quantity(em, URIRef(f"{op_node}-{suffix}"), number, qkind, unit)))
    em.graph.add((op_node, ALGO_EXT.out, out_node))
    return out_node


def emit_velocity_profile_reference(
    em: Emission,
    ctrl: ControllerEntry,
    spec: ConstraintSpecification,
    goal_node: URIRef | None,
    scalar_t: Any,
    profile_qty: ContextQuantity | None,
) -> URIRef:
    """A profiled setpoint toward GOAL_NODE (the profile's top speed when None), in the driven quantity's kind."""
    if profile_qty is None:
        profile_qty = ctrl.params.profile.quantity
    if isinstance(profile_qty, ContextQuantityAlias):
        profile_qty = profile_qty.ref
    profile = profile_qty.value
    out_node = declared_uri(f"{spec.name}-{ctrl.name}-profile-ref", ctrl)
    add_quantity(em, out_node, scalar_t)
    op_node = declared_uri(f"profile-{spec.name}-{ctrl.name}", ctrl)
    em.graph.add((op_node, RDF.type, ALGO_EXT.VelocityProfile))
    em.graph.add((op_node, RDF.type, CSTR_HDL_EXT.SetpointGenerator))
    # (term, limit, suffix, kind) per authored limit; it starts from the constraint's own quantity.
    limits = [
        (ALGO_EXT["maximum-velocity"], profile.max_velocity, "max-velocity", QuantityType.LinearVelocity),
        (ALGO_EXT["maximum-acceleration"], profile.max_acceleration, "max-acceleration", QuantityType.LinearAcceleration),
    ]
    if profile.max_jerk is not None:
        limits.append((ALGO_EXT["maximum-jerk"], profile.max_jerk, "max-jerk", QuantityType.LinearJerk))
    limit_nodes = {}
    for term, ref, suffix, kind in limits:
        if ref.bare is not None:
            limit_node = emit_scalar_quantity(
                em,
                declared_uri(f"{profile_qty.name}-{suffix}", profile_qty),
                float(ref.bare.value),
                QUDT_KIND_BY_QUANTITY_TYPE[kind],
                DSL_UNITS[ref.bare.unit].iri,
            )
        else:
            limit_node = emit_context_ref_node(em, ref, profile_qty, suffix)
        limit_nodes[term] = limit_node
        em.graph.add((op_node, term, limit_node))
    em.graph.add((op_node, ALGO_EXT.target, goal_node or limit_nodes[ALGO_EXT["maximum-velocity"]]))
    if profile.measured_velocity is not None:
        em.graph.add((op_node, ALGO_EXT["in"], emit_profile_view_node(em, profile.measured_velocity, profile_qty)))
    em.graph.add((op_node, ALGO_EXT.shape, ALGO_EXT[profile.shape or "trapezoidal"]))
    em.graph.add((op_node, ALGO_EXT.out, out_node))
    return out_node


def emit_elapsed_constraint(em: Emission, node: URIRef, spec: ConstraintSpecification, motion: GuardedMotion) -> None:
    """A cstr-ext:TimeConstraint on the interval from motion entry, or a quantity's last reading, to now."""
    em.graph.add((node, RDF.type, CSTR.Constraint))
    em.graph.add((node, RDF.type, CSTR_EXT.TimeConstraint))
    qty_node = owned_uri(em, f"{spec.name}-elapsed", motion)
    em.graph.add((qty_node, RDF.type, QUDT_SCHEMA.Quantity))
    emit_quantity_kind(em, qty_node, NS_MM_QUDT_QTY["Time"])
    # The clock ticks in seconds.
    em.graph.add((qty_node, QUDT_SCHEMA.unit, DSL_UNITS["s"].iri))
    em.graph.add((node, CSTR.quantity, qty_node))
    if str(motion.uri) not in em.motion_time_endpoints:
        entry_node = owned_uri(em, "motion-entry", motion)
        current_node = owned_uri(em, "current-time", motion)
        em.graph.add((entry_node, RDF.type, TIME.Instant))
        em.graph.add((current_node, RDF.type, TIME.Instant))
        em.motion_time_endpoints[str(motion.uri)] = (entry_node, current_node)
    entry_node, current_node = em.motion_time_endpoints[str(motion.uri)]
    observed = spec.view.elapsed.observed
    begin_node = entry_node
    if observed is not None:
        if str(observed.uri) not in em.observation_instants:
            # SOSA's phenomenonTime on the observed property, filled by the run.
            instant = owned_uri(em, f"{observed.name}-observed", observed)
            em.graph.add((instant, RDF.type, TIME.Instant))
            em.graph.add((URIRef(observed.uri), SOSA.phenomenonTime, instant))
            em.observation_instants[str(observed.uri)] = instant
        begin_node = em.observation_instants[str(observed.uri)]
    interval_node = owned_uri(em, f"{spec.name}-interval", motion)
    em.graph.add((interval_node, RDF.type, TIME.ProperInterval))
    em.graph.add((interval_node, TIME.hasBeginning, begin_node))
    em.graph.add((interval_node, TIME.hasEnd, current_node))
    em.graph.add((node, TIME.hasTime, interval_node))
    expr = spec.expr
    if isinstance(expr, (GreaterThanConstraint, LessThanConstraint)):
        em.graph.add((node, RDF.type, CSTR.UnilateralConstraint))
        em.graph.add(
            (
                node,
                RDF.type,
                CSTR.GreaterThanConstraint if isinstance(expr, GreaterThanConstraint) else CSTR.LessThanConstraint,
            )
        )
        durations = [(CSTR.threshold, expr.threshold, f"{spec.name}-threshold")]
    else:
        em.graph.add((node, RDF.type, CSTR.EqualityConstraint))
        durations = [
            (CSTR["reference-value"], expr.reference, f"{spec.name}-reference"),
            (
                CSTR_EXT.tolerance,
                spec.tolerance or em.tolerance_defaults.get(QuantityType.Duration),
                f"{spec.name}-tolerance",
            ),
        ]
    for predicate, ref, suffix in durations:
        em.graph.add((node, predicate, duration_node(em, ref, motion, suffix)))


def duration_node(em: Emission, ref: ContextRef, owner: Any, suffix: str) -> URIRef:
    """A time:Duration: an inline literal, or the declared duration quantity REF names."""
    if ref.bare is not None:
        node = owned_uri(em, suffix, owner)
        emit_duration_measure(em, node, ref.bare)
        return node
    quantity = ref.quantity
    return URIRef((quantity.ref if isinstance(quantity, ContextQuantityAlias) else quantity).uri)


def emit_along_path_constraint(
    em: Emission, node: URIRef, spec: ConstraintSpecification, motion: GuardedMotion
) -> None:
    """A driver commanding, or a progress guard bounding from below, the speed measured along a path.

    The guard only decides whether the current action may go on; it never adds a solver row.
    """
    operand = spec.view.moving or spec.view.progress
    path = operand.path.quantity
    path = path.ref if isinstance(path, ContextQuantityAlias) else path
    speed_node = declared_uri(f"{path.name}-along-speed", path)
    em.graph.add((node, RDF.type, CSTR.Constraint))
    em.graph.add((node, RDF.type, constraint_type_iri(QuantityType.LinearVelocity)))
    em.graph.add((node, CSTR.quantity, speed_node))
    em.graph.add((node, GEOM_OP_EXT.path, path_geometry_node(path)))
    if spec.view.moving is None:
        em.graph.add((node, RDF.type, CSTR.UnilateralConstraint))
        em.graph.add((node, RDF.type, CSTR.GreaterThanConstraint))
        threshold = emit_context_ref_node(em, spec.expr.threshold, motion, f"{spec.name}-threshold")
        em.graph.add((node, CSTR.threshold, threshold))
        return
    em.graph.add((node, RDF.type, CSTR.EqualityConstraint))
    ctrl = em.controller_by_spec.get(spec)
    profile_qty = operand.profile.quantity
    if isinstance(profile_qty, ContextQuantityAlias):
        profile_qty = profile_qty.ref
    ref_node = emit_velocity_profile_reference(em, ctrl, spec, None, QuantityType.LinearVelocity, profile_qty)
    profile_node = declared_uri(f"profile-{spec.name}-{ctrl.name}", ctrl)
    em.graph.add((profile_node, GEOM_OP_EXT.path, path_geometry_node(path)))
    em.graph.add((profile_node, GEOM_OP_EXT["path-parameter"], declared_uri(f"{path.name}-s", path)))
    em.graph.add((node, CSTR["reference-value"], ref_node))
    # A commanded speed is never met exactly either.
    emit_constraint_tolerance(em, node, spec, motion, QuantityType.LinearVelocity)


def section_expression_type(logic: str | None, member_count: int) -> URIRef | None:
    """The expression node `any`, or `all` of several members, mints; None keeps members flat."""
    if logic == "any" and member_count:
        return CSTR_EXT.ConstraintDisjunction
    if logic == "all" and member_count > 1:
        return CSTR_EXT.ConstraintConjunction
    return None


def section_expression(em: Emission, motion: GuardedMotion, section) -> tuple[URIRef | None, URIRef | None, list]:
    """A when or until section's expression node, its type, and its enabled ungrouped members.

    The node is None when the members link flat. The motion and any whole-section monitor both
    resolve through here, so a monitor never points at a node the section did not mint.
    """
    members = [
        item.constraint if isinstance(item, ConstraintAlias) else item
        for item in section.constraints
        if not isinstance(item, ConstraintGroup)
    ]
    members = [member for member in members if not member.disabled]
    node_type = section_expression_type(section.logic, len(members))
    if node_type is None:
        return None, None, members
    name = f"motion-{motion.name}-{section.kind}-{_SECTION_EXPRESSION_SUFFIX[node_type]}"
    return owned_uri(em, name, motion), node_type, members


def emit_section_constraints(em: Emission, motion: GuardedMotion, motion_node: URIRef, section, predicate: URIRef) -> None:
    """A section's enabled constraints, under its expression node when it has one."""
    expression_node, node_type, members = section_expression(em, motion, section)
    if expression_node is None:
        for member in members:
            em.graph.add((motion_node, predicate, URIRef(member.uri)))
        return
    em.graph.add((expression_node, RDF.type, node_type))
    em.graph.add((motion_node, predicate, expression_node))
    for member in members:
        em.graph.add((expression_node, CSTR_EXT["has-constraint"], URIRef(member.uri)))


def emit_detect_acts(em: Emission, motion: GuardedMotion) -> None:
    """Each detect act as a ros:Action, the status slot its outcome lands in, and the until items reading it."""
    for act in motion.detects:
        act_node = URIRef(act.uri)
        em.graph.add((act_node, RDF.type, ROS.Action))
        em.graph.add((act_node, ROS["channel-name"], Literal(act.action.channel_name)))
        em.graph.add((act_node, ROS["type-name"], Literal(act.action.type_name)))
        if act.action.pose_path:
            em.graph.add((act_node, ROS["field-path"], Literal(act.action.pose_path)))
        for target in act.targets:
            em.graph.add((act_node, SOSA.hasFeatureOfInterest, URIRef(str(target.ref.uri))))
        em.graph.add((URIRef(f"{act.uri}.status"), PROV.wasDerivedFrom, act_node))
    for item in flatten_constraint_items(motion.until.constraints):
        if isinstance(item, GoalStatusConstraint):
            node = URIRef(item.uri)
            em.graph.add((node, RDF.type, CSTR.Constraint))
            em.graph.add((node, RDF.type, CSTR.EqualityConstraint))
            em.graph.add((node, CSTR.quantity, URIRef(f"{item.act.uri}.status")))
            reference = URIRef(f"{item.uri}-reference")
            em.graph.add((reference, RDF.value, Literal(item.status_constant)))
            em.graph.add((node, CSTR["reference-value"], reference))


def emit_motion_spec(em: Emission, motion: GuardedMotion) -> None:
    """The guarded motion: its context members and its when, while and until constraints."""
    motion_node = owned_uri(em, f"motion-{motion.name}", motion)
    em.graph.add((motion_node, RDF.type, MOT.GuardedMotion))
    emit_context_members(em, motion_node, motion)
    em.graph.add((motion_node, SDO.name, Literal(motion.name)))
    if motion.description:
        em.graph.add((motion_node, SDO.description, Literal(motion.description)))
    emit_section_constraints(em, motion, motion_node, motion.when, MOT.when)
    for item in motion.while_.constraints:
        spec = item.constraint if isinstance(item, ConstraintAlias) else item
        if not spec.disabled:
            em.graph.add((motion_node, MOT["while"], URIRef(spec.uri)))
    # A named group is one transition condition, which a monitor can target.
    for group in motion.until.constraints:
        if not isinstance(group, ConstraintGroup):
            continue
        members = [item.constraint if isinstance(item, ConstraintAlias) else item for item in group.constraints]
        members = [member for member in members if not member.disabled]
        if not members:
            continue
        group_node = URIRef(group.uri)
        em.graph.add(
            (
                group_node,
                RDF.type,
                CSTR_EXT.ConstraintDisjunction if group.logic == "any" else CSTR_EXT.ConstraintConjunction,
            )
        )
        em.graph.add((motion_node, MOT.until, group_node))
        for member in members:
            em.graph.add((group_node, CSTR_EXT["has-constraint"], URIRef(member.uri)))
    emit_section_constraints(em, motion, motion_node, motion.until, MOT.until)
