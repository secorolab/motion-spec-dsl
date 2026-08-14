# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Validate constraint-handler assembly and coverage."""

from __future__ import annotations

from motion_spec_dsl.classes.constraint_handler import (
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
from motion_spec_dsl.classes.motion_spec import Model
from motion_spec_dsl.classes.controller_semantics import controller_solver
from motion_spec_dsl.classes.validation.common import (
    constraint_handlers,
    motion_constraint_items,
    semantic_error,
)


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


_MOBILE_PLATFORM_QUANTITY_TYPE = {
    "VelocityComposition": "VelocityTwist",
    "VelocityDistribution": "VelocityTwist",
    "ForceDistribution": "Wrench",
    "ForceComposition": "Wrench",
}


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
