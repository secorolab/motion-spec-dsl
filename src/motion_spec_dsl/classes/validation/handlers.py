# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Validate constraint-handler assembly and coverage."""

from __future__ import annotations

from textx import get_children_of_type, get_location
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.classes.constraint_handler import (
    KINEMATICS_ALGORITHMS,
    ConstraintHandler,
    ControllerAlias,
    ControllerType,
    MobilePlatformSolver,
    SerialChainSolver,
    SolverAlias,
    UntilMonitorRef,
    WhenMonitorRef,
)
from motion_spec_dsl.classes.constraints import (
    ConstraintAlias,
    ConstraintGroup,
    ConstraintSpecification,
    EqualityConstraint,
    ViewForm,
    flatten_constraint_items,
    view_form,
)
from motion_spec_dsl.classes.context import (
    ContextQuantity,
    ContextQuantityAlias,
    GeometricPropKey,
    GeometricProps,
    QuantityType,
    ReferenceGeneratorType,
    WorldQuantityType,
    geo_prop,
)
from motion_spec_dsl.classes.controller_semantics import (
    ANGULAR_SUBSPACES,
    alignment_is_pointwise,
    constraint_view_subspace,
    controller_command_record,
    controller_solver,
    resolved_constraint_quantity,
)
from motion_spec_dsl.classes.motion_spec import ContextDeclReference, GuardedMotion, Model
from motion_spec_dsl.classes.path import ProfileSpec
from motion_spec_dsl.classes.validation.common import motion_constraints

# Resolved view subspaces whose command is a force, not a moment.
_LINEAR_SUBSPACES = {"position", "distance", "linear", "linear-velocity"}
_MOBILE_PLATFORM_QUANTITY_TYPE = {
    "VelocityComposition": "VelocityTwist",
    "VelocityDistribution": "VelocityTwist",
    "ForceDistribution": "Wrench",
    "ForceComposition": "Wrench",
}
# Measured joint quantities no controller commands.
_MEASURED_ONLY = {
    WorldQuantityType.JointCurrent: "joint current",
    WorldQuantityType.JointVelocity: "joint velocity",
}


def validate_handler_constraint_assembly(model: Model) -> None:
    """Reject handler contexts other than world and spec, and targets the motion does not assemble."""
    for handler in get_children_of_type(ConstraintHandler, model):
        for ctx in handler.context:
            ctx = ctx.ref if isinstance(ctx, ContextDeclReference) else ctx
            if ctx.name not in ("world", "spec"):
                raise TextXSemanticError(
                    f"handler '{handler.name}' declares a '{ctx.name}' context -- a handler adds "
                    "world and spec quantities only",
                    **get_location(handler),
                )
        # An until group is assembled in its own right: a monitor may target the group.
        items = [item for section in handler.motion.sections for item in section.constraints]
        assembled = {
            item.constraint if isinstance(item, ConstraintAlias) else item
            for item in flatten_constraint_items(items)
        }
        assembled |= {item for item in items if isinstance(item, ConstraintGroup)}
        for monitor in handler.monitors:
            if isinstance(monitor.constraint, (UntilMonitorRef, WhenMonitorRef)):
                if monitor.constraint.motion is not handler.motion:
                    raise TextXSemanticError(
                        f"monitor '{monitor.name}' watches '{monitor.constraint}' -- handler "
                        f"'{handler.name}' handles '{handler.motion.name}'",
                        **get_location(monitor),
                    )
            elif monitor.constraint.constraint not in assembled:
                raise TextXSemanticError(
                    f"monitor '{monitor.name}' watches '{monitor.constraint}' -- motion "
                    f"'{handler.motion.name}' of handler '{handler.name}' does not assemble it",
                    **get_location(monitor),
                )
        for controller in handler.controllers:
            resolved = controller.ref.controller if isinstance(controller, ControllerAlias) else controller
            if resolved.params.constraint.constraint not in assembled:
                raise TextXSemanticError(
                    f"controller '{controller.name}' drives '{resolved.params.constraint}' -- "
                    f"motion '{handler.motion.name}' of handler '{handler.name}' does not "
                    "assemble it",
                    **get_location(controller),
                )


def validate_handler_requirements(model: Model) -> None:
    """Every while constraint is controlled or monitored, and every guard is monitored once."""
    for handler in get_children_of_type(ConstraintHandler, model):
        motion = handler.motion
        location = get_location(handler)
        if motion.while_.constraints and not (handler.controllers or handler.monitors):
            raise TextXSemanticError(
                f"handler '{handler.name}' has neither controllers nor monitors -- motion "
                f"'{motion.name}' holds while constraints",
                **location,
            )
        when_items = [
            item.constraint if isinstance(item, ConstraintAlias) else item
            for item in flatten_constraint_items(motion.when.constraints)
        ]
        until_items = [
            item.constraint if isinstance(item, ConstraintAlias) else item
            for item in flatten_constraint_items(motion.until.constraints)
        ]
        if (when_items or until_items) and not handler.monitors:
            raise TextXSemanticError(
                f"handler '{handler.name}' has no monitor -- motion '{motion.name}' has when or "
                "until constraints",
                **location,
            )
        aggregate_until = [
            monitor for monitor in handler.monitors if isinstance(monitor.constraint, UntilMonitorRef)
        ]
        individual = [
            monitor
            for monitor in handler.monitors
            if not isinstance(monitor.constraint, (UntilMonitorRef, WhenMonitorRef))
        ]
        if until_items:
            individual_until = [
                monitor
                for monitor in individual
                if monitor.constraint.constraint in until_items
                or isinstance(monitor.constraint.constraint, ConstraintGroup)
            ]
            if aggregate_until and individual_until:
                raise TextXSemanticError(
                    f"handler '{handler.name}' mixes aggregate and individual until monitors",
                    **get_location(individual_until[0]),
                )
            if len(aggregate_until) > 1:
                raise TextXSemanticError(
                    f"handler '{handler.name}' monitors <{motion.name}.until> more than once",
                    **location,
                )
            if not aggregate_until:
                # Monitoring a group covers every constraint inside it.
                monitored = set()
                for monitor in individual_until:
                    target = monitor.constraint.constraint
                    if isinstance(target, ConstraintGroup):
                        monitored |= {
                            item.constraint if isinstance(item, ConstraintAlias) else item
                            for item in target.constraints
                        }
                    else:
                        monitored.add(target)
                missing = [item.name for item in until_items if item not in monitored]
                if missing:
                    raise TextXSemanticError(
                        f"handler '{handler.name}' leaves until constraints unmonitored: "
                        f"{', '.join(missing)}",
                        **location,
                    )
        elif aggregate_until:
            raise TextXSemanticError(
                f"handler '{handler.name}' monitors <{motion.name}.until> -- the motion has no "
                "until constraints",
                **get_location(aggregate_until[0]),
            )
        # A when guard is monitored per constraint, or by one aggregate <motion.when> monitor.
        if any(isinstance(monitor.constraint, WhenMonitorRef) for monitor in handler.monitors):
            continue
        monitored = {monitor.constraint.constraint for monitor in individual}
        for constraint in when_items:
            if constraint not in monitored:
                raise TextXSemanticError(
                    f"handler '{handler.name}' leaves when constraint '{constraint.name}' "
                    "unmonitored",
                    **get_location(constraint),
                )


def validate_controller_solver_assembly(model: Model) -> None:
    """Every controller resolves to a solver its handler assembles."""
    for handler in get_children_of_type(ConstraintHandler, model):
        assembled = {
            item.ref.solver if isinstance(item, SolverAlias) else item for item in handler.solvers
        }
        for controller in handler.controllers:
            solver = controller_solver(handler, controller)
            if solver is None:
                raise TextXSemanticError(
                    f"controller '{controller.name}' has no unique compatible solver in handler "
                    f"'{handler.name}' -- name one with 'via'",
                    **get_location(controller),
                )
            if solver not in assembled:
                raise TextXSemanticError(
                    f"controller '{controller.name}' routes through solver '{solver.name}' -- "
                    f"handler '{handler.name}' does not assemble it",
                    **get_location(controller),
                )


def validate_commanded_quantity_is_measured(model: Model) -> None:
    """No controller commands a quantity a sensor or observer supplies, or a measured joint state."""
    for handler in get_children_of_type(ConstraintHandler, model):
        for controller in handler.controllers:
            resolved = controller.ref.controller if isinstance(controller, ControllerAlias) else controller
            constraint = resolved.params.constraint.constraint
            quantity = resolved_constraint_quantity(constraint)
            if quantity is None:
                continue
            location = get_location(controller)
            if quantity.type in _MEASURED_ONLY:
                raise TextXSemanticError(
                    f"controller '{resolved.name}' drives '{constraint.name}' on "
                    f"'{quantity.name}' -- a {_MEASURED_ONLY[quantity.type]} is measured, no "
                    "controller commands it",
                    **location,
                )
            if resolved.type != ControllerType.FeedForward or not isinstance(
                quantity.props, GeometricProps
            ):
                continue
            # A quantity with `ft-sensor` or `estimated-from` takes its value from there only.
            sources = [
                pair.sensor or pair.agent
                for pair in quantity.props.pairs
                if pair.key in (GeometricPropKey.FtSensor, GeometricPropKey.EstimatedFrom)
                and (pair.sensor or pair.agent) is not None
            ]
            if sources:
                raise TextXSemanticError(
                    f"controller '{controller.name}' assigns to '{quantity.name}' -- "
                    f"'{sources[0].name}' measures or estimates it; command a quantity of its own",
                    **location,
                )


def validate_kinematics_solvers(model: Model) -> None:
    """A forward-kinematics solver reads a chain: no driving controller, gravity or torque limit."""
    for handler in get_children_of_type(ConstraintHandler, model):
        for item in handler.solvers:
            solver = item.ref.solver if isinstance(item, SolverAlias) else item
            if not isinstance(solver, SerialChainSolver) or solver.algorithm not in KINEMATICS_ALGORITHMS:
                continue
            for value, label in ((solver.gravity_value, "gravity"), (solver.limits, "limits")):
                if value is not None:
                    raise TextXSemanticError(
                        f"serial-chain solver '{solver.name}' ({solver.algorithm}) takes no "
                        f"{label} -- it only reads the chain",
                        **get_location(solver),
                    )
        for controller in handler.controllers:
            solver = controller_solver(handler, controller)
            if isinstance(solver, SerialChainSolver) and solver.algorithm in KINEMATICS_ALGORITHMS:
                raise TextXSemanticError(
                    f"controller '{controller.name}' routes through '{solver.name}' "
                    f"({solver.algorithm}) -- it only reads the chain; a driven arm needs 'achd' "
                    "or 'rne'",
                    **get_location(controller),
                )


def validate_mobile_platform_solver_quantity(model: Model) -> None:
    """A platform solver's quantity is a velocity twist for velocities and a wrench for forces."""
    for solver in get_children_of_type(MobilePlatformSolver, model):
        quantity = solver.quantity.quantity
        wanted = _MOBILE_PLATFORM_QUANTITY_TYPE[solver.algorithm]
        found = quantity.type if isinstance(quantity, ContextQuantity) else None
        if found != wanted:
            raise TextXSemanticError(
                f"mobile-platform solver '{solver.name}' ({solver.algorithm}) takes a '{wanted}' "
                f"quantity, not '{found}'",
                **get_location(solver),
            )


def validate_controller_commands(model: Model) -> None:
    """Every controller command can be built: right subspace, a profile, and a line of action."""
    controlled = set()
    for handler in get_children_of_type(ConstraintHandler, model):
        for item in handler.controllers:
            ctrl = item.ref.controller if isinstance(item, ControllerAlias) else item
            spec = ctrl.params.constraint.constraint
            if not isinstance(spec, ConstraintSpecification):
                continue
            controlled.add(spec)
            location = get_location(ctrl)
            command = controller_command_record(ctrl)
            form = view_form(spec)
            if not spec.disabled:
                if form == ViewForm.Norm:
                    raise TextXSemanticError(
                        f"controller '{ctrl.name}' constrains a norm view -- nothing can command a "
                        "norm",
                        **location,
                    )
                if command.view_subspace in ANGULAR_SUBSPACES and ctrl.command_type == QuantityType.Force:
                    raise TextXSemanticError(
                        f"controller '{ctrl.name}' commands a force on the angular subspace of "
                        f"'{spec.name}' -- an angular constraint commands a moment ('as torque')",
                        **location,
                    )
                if command.view_subspace in _LINEAR_SUBSPACES and ctrl.command_type == QuantityType.Torque:
                    raise TextXSemanticError(
                        f"controller '{ctrl.name}' commands a moment on the linear subspace of "
                        f"'{spec.name}' -- a linear constraint commands a force ('as force')",
                        **location,
                    )
            if ctrl.params.profile is not None:
                profile = ctrl.params.profile.quantity
                if isinstance(profile, ContextQuantityAlias):
                    profile = profile.ref
                if not isinstance(profile, ContextQuantity) or not isinstance(profile.value, ProfileSpec):
                    raise TextXSemanticError(
                        f"controller '{ctrl.name}' names a profile that is no velocity profile",
                        **location,
                    )
            solver = controller_solver(handler, ctrl)
            if solver is None:
                continue
            quantity = resolved_constraint_quantity(spec)
            platform = isinstance(solver, MobilePlatformSolver)
            if (
                not platform
                and solver.algorithm in {"ACHD", "RNE"}
                and command.is_posture_torque_command
                and quantity is not None
                and quantity.type == WorldQuantityType.JointPosition
                and geo_prop(quantity.props, "joint") is None
            ):
                raise TextXSemanticError(
                    f"joint torque command 'tau-{ctrl.name}' has no target joint", **location
                )
            if not (command.is_force_command or command.is_moment_command):
                continue
            if platform and solver.algorithm != "ForceDistribution":
                continue
            if solver.algorithm == "CommandForwarding" and ctrl.type == ControllerType.FeedForward:
                continue
            # A platform builds a moment first, a chain a force first.
            moment = command.is_moment_command if platform else not command.is_force_command
            label = "moment" if moment else "force"
            if ctrl.apply_at is None:
                raise TextXSemanticError(
                    f"{label} controller '{ctrl.name}' states no 'apply at <link>'", **location
                )
            # An expression states its own direction and frame: the gradient its terms share.
            expression = spec.view.expr is not None
            if platform and not moment and quantity is None and spec.view.binary is None and not expression:
                stated = spec.view.quantity
                if isinstance(stated, ContextQuantityAlias):
                    stated = stated.ref
                if not isinstance(stated, ContextQuantity) or not isinstance(stated.props, GeometricProps):
                    raise TextXSemanticError(
                        f"controller '{ctrl.name}' routes a force to platform solver "
                        f"'{solver.name}' -- constraint '{spec.name}' names no quantity with "
                        "frames, so the wrench has no direction",
                        **location,
                    )
                quantity = stated
            # A binary view's operands already state the frame its target is taken in.
            if quantity is not None and (
                geo_prop(quantity.props, "as-seen-by") or geo_prop(quantity.props, "wrt")
            ) is None:
                raise TextXSemanticError(
                    f"{label} controller '{ctrl.name}' takes no frame from the constrained quantity",
                    **location,
                )
            cone = form == ViewForm.Alignment and not alignment_is_pointwise(spec)
            gradient = expression or cone or form in (
                ViewForm.DistanceFrom,
                ViewForm.ProjectionOn,
                ViewForm.IncidentAngle,
                ViewForm.PlaneAngle,
            )
            if moment:
                axes = [] if cone else [axis for _, axis in command.controlled_axes if axis is not None]
                if not axes and not gradient:
                    raise TextXSemanticError(
                        f"moment controller '{ctrl.name}' commands no angular axis", **location
                    )
                continue
            if spec.view.axis is not None or form == ViewForm.DistanceBetween:
                continue
            if (
                quantity is not None
                and quantity.type == WorldQuantityType.Pose
                and constraint_view_subspace(spec) == "distance"
            ):
                continue
            if not gradient:
                raise TextXSemanticError(
                    f"force controller '{ctrl.name}' has no line of action -- it needs an axis or "
                    "a distance",
                    **location,
                )
    for motion in get_children_of_type(GuardedMotion, model):
        for spec in motion_constraints(motion):
            if not isinstance(spec, ConstraintSpecification) or spec.disabled or spec in controlled:
                continue
            if spec.view.moving is not None:
                raise TextXSemanticError(
                    f"profiled path constraint '{spec.name}' has no tracking controller",
                    **get_location(spec),
                )
            if isinstance(spec.expr, EqualityConstraint):
                reference = spec.expr.reference.quantity
                if isinstance(reference, ContextQuantityAlias):
                    reference = reference.ref
                if (
                    isinstance(reference, ContextQuantity)
                    and reference.type == ReferenceGeneratorType.Admittance
                ):
                    raise TextXSemanticError(
                        f"admittance constraint '{spec.name}' has no tracking PID -- the PID hosts "
                        "the filter's per-step integrator state",
                        **get_location(spec),
                    )
