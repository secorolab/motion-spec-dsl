# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Validate the disturbances a handler asks the simulator to apply."""

from __future__ import annotations

from motion_spec_dsl.classes.context import (
    ContextQuantity,
    QuantityType,
    _resolved_context_quantity,
)
from motion_spec_dsl.classes.motion_spec import ExecutionContext, Model
from motion_spec_dsl.classes.validation.common import constraint_handlers, semantic_error

# What each authored slot of a perturbation's apply clause must name.
_SLOT_TYPES = (
    ("force", QuantityType.Force, "force magnitude"),
    ("force_direction", QuantityType.Direction, "force direction"),
    ("moment", QuantityType.Torque, "torque magnitude"),
    ("moment_direction", QuantityType.Direction, "torque direction"),
)


def _named_quantity(ref) -> ContextQuantity | None:
    quantity = getattr(ref, "quantity", None)
    return _resolved_context_quantity(quantity) if isinstance(quantity, ContextQuantity) else None


def validate_perturbations(model: Model) -> None:
    """Raise if a perturbation runs on hardware or names a quantity of the wrong kind for the
    slot it fills.

    That the body is one the run's scene holds is checked where the scene's object table
    exists -- the IR pass that resolves it to a simulator body name.
    """
    context = next((spec for spec in model.specs if isinstance(spec, ExecutionContext)), None)
    for handler in constraint_handlers(model):
        perturbations = getattr(handler, "perturbations", []) or []
        if not perturbations:
            continue
        if context is None or getattr(context.platform, "kind", "") != "simulation":
            raise semantic_error(
                f"Handler '{handler.name}' authors perturbations, but the execution context "
                "does not run on a simulation platform: nothing on hardware can apply them.",
                perturbations[0],
            )
        for perturbation in perturbations:
            for attribute, expected, role in _SLOT_TYPES:
                ref = getattr(perturbation, attribute, None)
                if ref is None:
                    continue
                quantity = _named_quantity(ref)
                if quantity is None:
                    raise semantic_error(
                        f"Perturbation '{perturbation.name}' states its {role} inline; it must "
                        "name a declared quantity, so a richer force pattern can arrive as a "
                        "new kind of quantity rather than as new handler syntax.",
                        perturbation,
                    )
                if quantity.type != expected:
                    raise semantic_error(
                        f"Perturbation '{perturbation.name}' names '{quantity.name}' "
                        f"({quantity.type}) as its {role}, which must be a {expected} quantity.",
                        perturbation,
                    )
            unit = getattr(perturbation.duration, "unit", None)
            if perturbation.duration is not None and unit not in ("s", "ms"):
                raise semantic_error(
                    f"Perturbation '{perturbation.name}' holds for '{unit}', which is not a "
                    "duration; a window is stated in 's' or 'ms'.",
                    perturbation,
                )
