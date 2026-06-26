# SPDX-License-Identifier: MPL-2.0
"""Constraint handler assembly and coverage validation."""

from __future__ import annotations

from motion_spec_dsl.domain import (
    Model,
    UntilMonitorRef,
    WhenMonitorRef,
    _resolved_controller,
    _resolved_spec,
)
from motion_spec_dsl.validation.common import (
    constraint_handlers,
    motion_constraint_items,
    motion_specs,
    semantic_error,
)


def validate_handler_aliases(model: Model) -> None:
    del model


def validate_handler_constraint_assembly(model: Model) -> None:
    for handler in constraint_handlers(model):
        assembled_specs = {
            id(_resolved_spec(item)) for item in motion_constraint_items(handler.motion)
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
            if id(monitor.constraint.constraint) not in assembled_specs:
                raise semantic_error(
                    f"Monitor '{monitor.name}' references constraint "
                    f"'{monitor.constraint}', but handler '{handler.name}' primary motion "
                    f"'{handler.motion.name}' does not assemble it.",
                    monitor,
                )

        for controller in handler.controllers:
            resolved_controller = _resolved_controller(controller)
            if id(resolved_controller.params.constraint.constraint) not in assembled_specs:
                raise semantic_error(
                    f"Controller '{controller.name}' references constraint "
                    f"'{resolved_controller.params.constraint}', but handler '{handler.name}' "
                    f"primary motion '{handler.motion.name}' does not assemble it.",
                    controller,
                )


def validate_handler_constraint_refs(model: Model) -> None:
    del model


def validate_handler_requirements(model: Model) -> None:
    for handler in constraint_handlers(model):
        if handler.motion.while_.constraints and not (handler.controllers or handler.monitors):
            raise semantic_error(
                "ConstraintHandler with WHILE constraints must have at least one controller or monitor.",
                handler,
            )

        guard_constraints = [
            _resolved_spec(item)
            for item in [*handler.motion.when.constraints, *handler.motion.until.constraints]
        ]
        if guard_constraints and not handler.monitors:
            raise semantic_error(
                "ConstraintHandler with WHEN or UNTIL constraints must have at least one monitor.",
                handler,
            )

        until_items = [_resolved_spec(item) for item in handler.motion.until.constraints]
        until_monitor_refs = [mon for mon in handler.monitors if isinstance(mon.constraint, UntilMonitorRef)]
        if until_items:
            if len(until_monitor_refs) != 1:
                raise semantic_error(
                    f"ConstraintHandler '{handler.name}' must monitor the UNTIL guard with exactly one "
                    f"aggregate monitor like <{handler.motion.name}.until>.",
                    handler,
                )
            individual_until_monitors = [
                mon
                for mon in handler.monitors
                if not isinstance(mon.constraint, (UntilMonitorRef, WhenMonitorRef))
                and id(mon.constraint.constraint) in {id(c) for c in until_items}
            ]
            if individual_until_monitors:
                raise semantic_error(
                    f"ConstraintHandler '{handler.name}' monitors individual UNTIL constraints; "
                    f"use one aggregate monitor <{handler.motion.name}.until> instead.",
                    individual_until_monitors[0],
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
            id(mon.constraint.constraint)
            for mon in handler.monitors
            if not isinstance(mon.constraint, (UntilMonitorRef, WhenMonitorRef))
        }
        if not when_aggregate_monitored:
            for constraint in [_resolved_spec(item) for item in handler.motion.when.constraints]:
                if id(constraint) not in monitored:
                    raise semantic_error(
                        f"ConstraintHandler '{handler.name}' has WHEN constraint "
                        f"'{constraint.name}' without a monitor.",
                        constraint,
                    )


def validate_motion_spec_coverage(model: Model) -> None:
    referenced = {
        handler.motion.name
        for handler in constraint_handlers(model)
        if handler.motion is not None
    }
    for motion in motion_specs(model):
        if motion.name not in referenced:
            raise semantic_error(
                f"MotionSpec '{motion.name}' is not referenced by any ConstraintHandler. "
                "Every MotionSpec must be bound to exactly one ConstraintHandler.",
                motion,
            )
