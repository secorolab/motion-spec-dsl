# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Validate constraint-handler assembly and coverage."""

from __future__ import annotations

from motion_spec_dsl.classes.constraint_handler import (
    KINEMATICS_ALGORITHMS,
    ControllerType,
    MobilePlatformSolver,
    UntilMonitorRef,
    WhenMonitorRef,
    _resolved_controller,
    _resolved_solver,
)
from motion_spec_dsl.classes.constraints import (
    ConstraintGroup,
    _flatten_constraint_items,
    _resolved_spec,
)
from motion_spec_dsl.classes.constraints import EqualityConstraint
from motion_spec_dsl.classes.context import (
    ContextQuantity,
    GeometricPropKey,
    GeometricProps,
    QuantityType,
    ReferenceGeneratorType,
    WorldQuantityType,
    _resolved_context_quantity,
)
from motion_spec_dsl.classes.motion_spec import Model
from motion_spec_dsl.classes.controller_semantics import (
    ANGULAR_SUBSPACES,
    _alignment_is_pointwise,
    axis_label,
    constraint_view_subspace,
    controller_command_record,
    controller_solver,
    resolved_constraint_quantity,
)
from motion_spec_dsl.classes.path import ProfileSpec
from motion_spec_dsl.classes.validation.common import (
    constraint_handlers,
    motion_constraint_items,
    motion_constraints,
    motion_specs,
    semantic_error,
)
from motion_spec_dsl.rdf.common import (
    _binary_view,
    _context_quantity,
    _geo_prop,
    _is_alignment_view,
    _is_difference_view,
    _is_distance_view,
    _is_geometric_distance_view,
    _is_incident_angle_view,
    _is_norm_view,
    _is_plane_angle_view,
    _is_projection_view,
)

# Resolved view subspaces whose command is a force, not a moment.
_LINEAR_SUBSPACES = frozenset({"position", "distance", "linear", "linear-velocity"})
# Binary views that publish a runtime gradient a force or moment can act along.
_GRADIENT_VIEWS = frozenset({"AngleBetweenView", "DistanceFromView", "ProjectionOnView"})


def validate_handler_constraint_assembly(model: Model) -> None:
    """Raise if a handler's monitors or controllers reference constraints its primary motion
    does not assemble.
    """
    for handler in constraint_handlers(model):
        assembled_specs = {_resolved_spec(item) for item in motion_constraint_items(handler.motion)}
        # An until group is assembled by its motion in its own right: a monitor may target the
        # group instead of any single constraint inside it.
        assembled_specs |= {
            item
            for section in handler.motion.sections
            for item in section.constraints
            if isinstance(item, ConstraintGroup)
        }

        for monitor in handler.monitors:
            if isinstance(monitor.constraint, (UntilMonitorRef, WhenMonitorRef)):
                if monitor.constraint.motion is not handler.motion:
                    raise semantic_error(
                        f"Monitor '{monitor.name}' references guard "
                        f"'{monitor.constraint}', but handler '{handler.name}' primary motion "
                        f"is '{handler.motion.name}'.",
                        monitor,
                    )
                continue
            if monitor.constraint.constraint not in assembled_specs:
                raise semantic_error(
                    f"Monitor '{monitor.name}' references constraint "
                    f"'{monitor.constraint}', but handler '{handler.name}' primary motion "
                    f"'{handler.motion.name}' does not assemble it.",
                    monitor,
                )

        for controller in handler.controllers:
            resolved_controller = _resolved_controller(controller)
            if resolved_controller.params.constraint.constraint not in assembled_specs:
                raise semantic_error(
                    f"Controller '{controller.name}' references constraint "
                    f"'{resolved_controller.params.constraint}', but handler '{handler.name}' "
                    f"primary motion '{handler.motion.name}' does not assemble it.",
                    controller,
                )


def validate_handler_requirements(model: Model) -> None:
    """Raise if a handler lacks the controllers/monitors its WHILE/WHEN/UNTIL constraints
    require, mixes aggregate and individual UNTIL/WHEN monitors, or leaves guards unmonitored.
    """
    for handler in constraint_handlers(model):
        if handler.motion.while_.constraints and not (handler.controllers or handler.monitors):
            raise semantic_error(
                "ConstraintHandler with WHILE constraints must have at least one controller or monitor.",
                handler,
            )

        guard_constraints = [
            _resolved_spec(item)
            for item in _flatten_constraint_items(
                [*handler.motion.when.constraints, *handler.motion.until.constraints]
            )
        ]
        if guard_constraints and not handler.monitors:
            raise semantic_error(
                "ConstraintHandler with WHEN or UNTIL constraints must have at least one monitor.",
                handler,
            )

        until_items = [
            _resolved_spec(item)
            for item in _flatten_constraint_items(handler.motion.until.constraints)
        ]
        until_monitor_refs = [
            mon for mon in handler.monitors if isinstance(mon.constraint, UntilMonitorRef)
        ]
        if until_items:
            individual_until_monitors = [
                mon
                for mon in handler.monitors
                if not isinstance(mon.constraint, (UntilMonitorRef, WhenMonitorRef))
                and (
                    mon.constraint.constraint in set(until_items)
                    or isinstance(mon.constraint.constraint, ConstraintGroup)
                )
            ]
            if until_monitor_refs and individual_until_monitors:
                raise semantic_error(
                    f"ConstraintHandler '{handler.name}' mixes aggregate and individual UNTIL monitors.",
                    individual_until_monitors[0],
                )
            if len(until_monitor_refs) > 1:
                raise semantic_error(
                    f"ConstraintHandler '{handler.name}' must monitor <{handler.motion.name}.until> at most once.",
                    handler,
                )
            if not until_monitor_refs:
                # Monitoring a group covers every constraint inside it.
                monitored = set()
                for mon in individual_until_monitors:
                    target = mon.constraint.constraint
                    if isinstance(target, ConstraintGroup):
                        monitored |= {_resolved_spec(i) for i in target.constraints}
                    else:
                        monitored.add(target)
                missing = [c.name for c in until_items if c not in monitored]
                if missing:
                    raise semantic_error(
                        f"ConstraintHandler '{handler.name}' has unmonitored UNTIL constraint(s): "
                        f"{', '.join(missing)}.",
                        handler,
                    )
        elif until_monitor_refs:
            raise semantic_error(
                f"ConstraintHandler '{handler.name}' monitors <{handler.motion.name}.until>, "
                "but the motion has no UNTIL constraints.",
                until_monitor_refs[0],
            )

        # A WHEN guard may be monitored either per-constraint (single-arm style) or
        # by one aggregate <motion.when> monitor that ANDs all WHEN constraints.
        when_aggregate_monitored = any(
            isinstance(mon.constraint, WhenMonitorRef) for mon in handler.monitors
        )
        monitored = {
            mon.constraint.constraint
            for mon in handler.monitors
            if not isinstance(mon.constraint, (UntilMonitorRef, WhenMonitorRef))
        }
        if not when_aggregate_monitored:
            for constraint in [
                _resolved_spec(item)
                for item in _flatten_constraint_items(handler.motion.when.constraints)
            ]:
                if constraint not in monitored:
                    raise semantic_error(
                        f"ConstraintHandler '{handler.name}' has WHEN constraint "
                        f"'{constraint.name}' without a monitor.",
                        constraint,
                    )


def validate_controller_solver_assembly(model: Model) -> None:
    """Require every controller to resolve to a solver assembled by its handler."""
    for handler in constraint_handlers(model):
        assembled = {_resolved_solver(item) for item in handler.solvers}
        for controller in handler.controllers:
            solver = controller_solver(handler, controller)
            if solver is None:
                raise semantic_error(
                    f"Controller '{controller.name}' has no unique compatible solver in "
                    f"handler '{handler.name}'; author an explicit 'via' reference.",
                    controller,
                )
            if solver not in assembled:
                raise semantic_error(
                    f"Controller '{controller.name}' references solver '{solver.name}', but "
                    f"handler '{handler.name}' does not assemble it.",
                    controller,
                )


def _observing_sensor(quantity) -> object | None:
    """The sensor or observer a world quantity is read from, or None when nothing supplies it."""
    props = getattr(quantity, "props", None)
    if not isinstance(props, GeometricProps):
        return None
    return next(
        (
            pair.sensor or pair.agent
            for pair in props.pairs
            if pair.key in (GeometricPropKey.FtSensor, GeometricPropKey.EstimatedFrom)
            and (pair.sensor or pair.agent) is not None
        ),
        None,
    )


def validate_commanded_quantity_is_measured(model: Model) -> None:
    """Raise if a feed-forward controller assigns onto an observed quantity.

    A quantity carrying `ft-sensor` or `estimated-from` states what that source reads; its value
    comes from there and from nowhere else, so no controller may command it.
    """
    for handler in constraint_handlers(model):
        for controller in handler.controllers:
            resolved = _resolved_controller(controller)
            if resolved.type != ControllerType.FeedForward:
                continue
            quantity = resolved_constraint_quantity(resolved.params.constraint.constraint)
            sensor = _observing_sensor(quantity)
            if sensor is None:
                continue
            raise semantic_error(
                f"Controller '{controller.name}' assigns to '{quantity.name}', which "
                f"'{getattr(sensor, 'name', sensor)}' measures or estimates. Command a quantity "
                f"of its own instead: declare one without 'ft-sensor' or 'estimated-from'.",
                controller,
            )


_MOBILE_PLATFORM_QUANTITY_TYPE = {
    "VelocityComposition": "VelocityTwist",
    "VelocityDistribution": "VelocityTwist",
    "ForceDistribution": "Wrench",
    "ForceComposition": "Wrench",
}


def validate_kinematics_solvers(model: Model) -> None:
    """Reject dynamics a forward-kinematics solver cannot take: a driving controller, gravity,
    or a torque limit."""
    for handler in constraint_handlers(model):
        for item in handler.solvers:
            solver = _resolved_solver(item)
            if getattr(solver, "algorithm", None) not in KINEMATICS_ALGORITHMS:
                continue
            for field in ("gravity_value", "limits"):
                if getattr(solver, field, None) is not None:
                    raise semantic_error(
                        f"Serial-chain solver '{solver.name}' ({solver.algorithm}) only reads "
                        f"the chain; it takes no {field.removesuffix('_value')}.",
                        solver,
                    )
        for controller in handler.controllers:
            solver = controller_solver(handler, controller)
            if getattr(solver, "algorithm", None) in KINEMATICS_ALGORITHMS:
                raise semantic_error(
                    f"Controller '{controller.name}' routes through '{solver.name}' "
                    f"({solver.algorithm}), which only reads the chain; a driven arm needs "
                    "'achd' or 'rne'.",
                    controller,
                )


def validate_mobile_platform_solver_quantity(model: Model) -> None:
    """Require a mobile-platform solver's quantity kind to match its algorithm: a
    velocity-twist for velocity-composition/velocity-distribution, a wrench for
    force-distribution/force-composition.
    """
    for handler in constraint_handlers(model):
        for item in handler.solvers:
            solver = _resolved_solver(item)
            if not isinstance(solver, MobilePlatformSolver):
                continue
            ref = solver.quantity
            quantity = getattr(ref, "quantity", None)
            wanted = _MOBILE_PLATFORM_QUANTITY_TYPE[solver.algorithm]
            if quantity is None or getattr(quantity, "type", None) != wanted:
                got = getattr(quantity, "type", None)
                raise semantic_error(
                    f"Mobile-platform solver '{solver.name}' ({solver.algorithm}) requires a "
                    f"'{wanted}' quantity, got '{got}'.",
                    solver,
                )


def validate_controller_commands(model: Model) -> None:
    """Raise if a controller's command cannot be built: a norm nothing can command, a force or
    moment on the wrong subspace, a profile that is none, or a wrench with no body, frame or
    line of action.
    """
    controlled = set()
    for handler in constraint_handlers(model):
        for item in handler.controllers:
            ctrl = item.ref.controller if hasattr(item, "ref") else item
            spec = getattr(ctrl.params.constraint, "constraint", None)
            if spec is None:
                continue
            controlled.add(spec)
            command = controller_command_record(ctrl)
            if not spec.disabled:
                if _is_norm_view(spec):
                    raise semantic_error(
                        f"controller '{ctrl.name}' constrains a norm view, which nothing can "
                        "command",
                        ctrl,
                    )
                if command.view_subspace in ANGULAR_SUBSPACES and (
                    ctrl.command_type == QuantityType.Force
                ):
                    raise semantic_error(
                        f"Controller '{ctrl.name}' commands a force on the angular subspace of "
                        f"'{spec.name}'; an angular constraint commands a moment ('as torque').",
                        ctrl,
                    )
                if command.view_subspace in _LINEAR_SUBSPACES and (
                    ctrl.command_type == QuantityType.Torque
                ):
                    raise semantic_error(
                        f"Controller '{ctrl.name}' commands a moment on the linear subspace of "
                        f"'{spec.name}'; a linear constraint commands a force ('as force').",
                        ctrl,
                    )
            if getattr(ctrl.params, "profile", None) is not None:
                profile = _context_quantity(ctrl.params.profile)
                if not isinstance(profile, ContextQuantity):
                    raise semantic_error(
                        f"Controller '{ctrl.name}' has an unresolved velocity profile.", ctrl
                    )
                profile = _resolved_context_quantity(profile)
                if not isinstance(profile.value, ProfileSpec):
                    raise semantic_error(
                        f"Controller '{ctrl.name}' profile '{profile.name}' is not a Profile.",
                        ctrl,
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
                and _geo_prop(
                    quantity.props if isinstance(quantity.props, GeometricProps) else None,
                    "joint",
                )
                is None
            ):
                raise semantic_error(
                    f"Joint torque command 'tau-{ctrl.name}' has no target joint", ctrl
                )
            if not (command.is_force_command or command.is_moment_command):
                continue
            if platform and solver.algorithm != "ForceDistribution":
                continue
            if (
                not platform
                and solver.algorithm == "CommandForwarding"
                and ctrl.type == ControllerType.FeedForward
            ):
                continue
            # A platform builds a moment first, a chain a force first.
            moment = command.is_moment_command if platform else not command.is_force_command
            label = "Moment" if moment else "Force"
            apply_at = getattr(ctrl, "apply_at", None)
            if apply_at is None or not hasattr(apply_at, "uri"):
                raise semantic_error(
                    f"{label} controller '{ctrl.name}' must specify 'apply at <link>'.", ctrl
                )
            if platform and not moment and quantity is None and _binary_view(spec) is None:
                stated = getattr(spec.view, "quantity", None)
                if isinstance(stated, ContextQuantity):
                    stated = _resolved_context_quantity(stated)
                if not isinstance(stated, ContextQuantity) or not isinstance(
                    stated.props, GeometricProps
                ):
                    raise semantic_error(
                        f"Controller '{ctrl.name}' routes a force to platform solver "
                        f"'{solver.name}', but its constraint '{spec.name}' names no quantity "
                        "with frames, so no direction can be taken for the wrench.",
                        ctrl,
                    )
                quantity = stated
            # A binary view's operands already state the frame its target is taken in.
            if quantity is not None:
                props = quantity.props if isinstance(quantity.props, GeometricProps) else None
                if (_geo_prop(props, "as-seen-by") or _geo_prop(props, "wrt")) is None:
                    raise semantic_error(
                        f"{label} controller '{ctrl.name}' needs a frame from the constrained "
                        "quantity.",
                        ctrl,
                    )
            gradient = (
                (_is_alignment_view(spec) and not _alignment_is_pointwise(spec))
                or _is_geometric_distance_view(spec)
                or _is_projection_view(spec)
                or _is_incident_angle_view(spec)
                or _is_plane_angle_view(spec)
            )
            if moment:
                axes = [axis for _, axis in command.controlled_axes if axis is not None]
                if _is_alignment_view(spec) and not _alignment_is_pointwise(spec):
                    axes = []
                if not axes and not gradient:
                    raise semantic_error(
                        f"Moment controller '{ctrl.name}' commands no angular axis.", ctrl
                    )
                continue
            if axis_label(spec.view.axis) is not None or _is_distance_view(spec):
                continue
            if (
                quantity is not None
                and quantity.type == WorldQuantityType.Pose
                and constraint_view_subspace(spec) == "distance"
            ):
                continue
            if _is_difference_view(spec):
                left = _binary_view(spec).left
                left = (
                    _resolved_context_quantity(left) if isinstance(left, ContextQuantity) else left
                )
                value = getattr(left, "value", None)
                if (
                    type(value).__name__ != "DerivedScalarValue"
                    or type(value.view).__name__ not in _GRADIENT_VIEWS
                ):
                    raise semantic_error(
                        f"Force controller '{ctrl.name}' holds a difference whose operands "
                        "publish no gradient, so the command has no line of action.",
                        ctrl,
                    )
            elif not gradient:
                raise semantic_error(
                    f"Force controller '{ctrl.name}' needs an axis or a distance pose.", ctrl
                )

    for motion in motion_specs(model):
        for spec in motion_constraints(motion):
            if spec.disabled or spec in controlled:
                continue
            if getattr(spec.view, "moving", None) is not None:
                raise semantic_error(
                    f"Profiled path constraint '{spec.name}' needs a tracking controller.", spec
                )
            if isinstance(spec.expr, EqualityConstraint):
                reference = _context_quantity(spec.expr.reference)
                if (
                    isinstance(reference, ContextQuantity)
                    and _resolved_context_quantity(reference).type
                    == ReferenceGeneratorType.Admittance
                ):
                    raise semantic_error(
                        f"Admittance constraint '{spec.name}' needs a tracking PID to host the "
                        "filter's per-step integrator state.",
                        spec,
                    )
