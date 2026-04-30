# SPDX-License-Identifier: MPL-2.0
"""Constraint handler assembly and coverage validation."""

from __future__ import annotations

from motion_spec_dsl.domain import Model, _resolved_controller, _resolved_spec
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
        if (handler.motion.when.constraints or handler.motion.until.constraints) and not handler.monitors:
            raise semantic_error(
                "ConstraintHandler with WHEN or UNTIL constraints must have at least one monitor.",
                handler,
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
