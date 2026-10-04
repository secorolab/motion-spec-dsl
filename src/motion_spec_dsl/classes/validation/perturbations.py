# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Validate the disturbances a handler asks the simulator to apply."""

from __future__ import annotations

from textx import get_children_of_type, get_location
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.classes.constraint_handler import ConstraintHandler
from motion_spec_dsl.classes.context import (
    ContextQuantity,
    ContextQuantityAlias,
    QuantityType,
    geo_prop,
)
from motion_spec_dsl.classes.motion_spec import ExecutionContext, Model


def validate_perturbations(model: Model) -> None:
    """Reject perturbations on hardware, and slots that name no quantity or one of the wrong kind.

    That the body is in the run's scene is checked where the scene's object table exists.
    """
    context = next((spec for spec in model.specs if isinstance(spec, ExecutionContext)), None)
    for handler in get_children_of_type(ConstraintHandler, model):
        if not handler.perturbations:
            continue
        if context is None or context.platform.kind != "simulation":
            raise TextXSemanticError(
                f"handler '{handler.name}' authors perturbations, but the execution context is no "
                "simulation -- nothing on hardware can apply them",
                **get_location(handler.perturbations[0]),
            )
        for perturbation in handler.perturbations:
            location = get_location(perturbation)
            slots = [
                (perturbation.force, QuantityType.Force, "force magnitude"),
                (perturbation.force_direction, QuantityType.Direction, "force direction"),
                (perturbation.moment, QuantityType.Torque, "torque magnitude"),
                (perturbation.moment_direction, QuantityType.Direction, "torque direction"),
                (perturbation.duration, QuantityType.Duration, "window"),
            ]
            for ref, expected, role in slots:
                if ref is None:
                    continue
                # An inline window is a duration too; every other slot names a declared quantity.
                if expected == QuantityType.Duration and ref.bare is not None:
                    if ref.bare.unit not in ("s", "ms"):
                        raise TextXSemanticError(
                            f"perturbation '{perturbation.name}' states a window in "
                            f"'{ref.bare.unit}' -- a window is a duration, in 's' or 'ms'",
                            **location,
                        )
                    continue
                quantity = ref.quantity
                if isinstance(quantity, ContextQuantityAlias):
                    quantity = quantity.ref
                if not isinstance(quantity, ContextQuantity):
                    raise TextXSemanticError(
                        f"perturbation '{perturbation.name}' states its {role} inline -- it names "
                        "a declared quantity, so a richer pattern arrives as a new kind of quantity",
                        **location,
                    )
                if quantity.type != expected:
                    raise TextXSemanticError(
                        f"perturbation '{perturbation.name}' names '{quantity.name}' "
                        f"({quantity.type}) as its {role} -- it must be a {expected}",
                        **location,
                    )
                if expected == QuantityType.Direction and not (
                    geo_prop(quantity.props, "as-seen-by") or geo_prop(quantity.props, "wrt")
                ):
                    raise TextXSemanticError(
                        f"perturbation '{perturbation.name}' direction '{quantity.name}' states no "
                        "'as-seen-by' frame -- it is the frame the applied wrench is stated in",
                        **location,
                    )
