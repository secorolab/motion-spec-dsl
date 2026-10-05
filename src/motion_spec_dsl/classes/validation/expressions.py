# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Validate quantity expressions with the same `infer` the RDF emission uses."""

from __future__ import annotations

from scene_dsl.classes.distrib import NormalDistribution, UniformDistribution
from textx import get_children_of_type, get_location
from textx.exceptions import TextXSemanticError
from textx.scoping import get_included_models

from motion_spec_dsl.classes.constraint_handler import (
    ConstraintHandler,
    ControllerAlias,
    motion_context_quantities,
    motion_world_quantities,
)
from motion_spec_dsl.classes.constraints import ConstraintSpecification
from motion_spec_dsl.classes.context import (
    ContextQuantity,
    ContextQuantityAlias,
    ContextRef,
    QOpNode,
    QuantityType,
    ReferenceValue,
    SampledValue,
    SnapshotValue,
    View,
    op_tree,
)
from motion_spec_dsl.classes.controller_semantics import controller_command_record
from motion_spec_dsl.classes.dimensions import DimensionError, infer, same_scalar_dimension
from motion_spec_dsl.classes.motion_spec import Model
from motion_spec_dsl.classes.views import leaf_gradient

# Geometry is stated through map views of its components, never aliased whole.
_GEOMETRIC_TYPES = {
    QuantityType.Pose,
    QuantityType.Position,
    QuantityType.Orientation,
    QuantityType.VelocityTwist,
    QuantityType.AccelerationTwist,
    QuantityType.Wrench,
    QuantityType.Direction,
}


def validate_expression_dimensions(model: Model) -> None:
    """Every expression must type-check, and a declared quantity must be what it infers."""
    # (expression, object it is reported on, the quantity whose declared kind it must match)
    expressions = []
    for quantity in get_children_of_type(ContextQuantity, model):
        if isinstance(quantity, ContextQuantityAlias):
            continue
        value = quantity.value
        if isinstance(value, ReferenceValue):
            if quantity.type in _GEOMETRIC_TYPES:
                raise TextXSemanticError(
                    f"'{quantity.name}' aliases a whole {quantity.type} -- reference its "
                    "components through their views",
                    **get_location(quantity),
                )
            expressions.append((value.expr, quantity, quantity))
        elif isinstance(value, SnapshotValue) and value.tail:
            expressions.append((value, quantity, quantity))
    expressions += [
        (ref.expr, ref, None) for ref in get_children_of_type(ContextRef, model) if ref.expr
    ]
    expressions += [
        (view.expr, view, None) for view in get_children_of_type(View, model) if view.expr
    ]
    for expr, obj, declared in expressions:
        try:
            inferred = infer(expr)
        except DimensionError as exc:
            raise TextXSemanticError(str(exc), **get_location(obj)) from exc
        if declared is not None and not same_scalar_dimension(declared.type, inferred):
            raise TextXSemanticError(
                f"'{declared.name}' is declared {declared.type} but its expression infers "
                f"{inferred}",
                **get_location(declared),
            )


def validate_sampled_quantities(model: Model) -> None:
    """A sampled scalar draws one number: its distribution is a 1-D uniform or normal."""
    for quantity in get_children_of_type(ContextQuantity, model):
        if isinstance(quantity, ContextQuantityAlias) or not isinstance(quantity.value, SampledValue):
            continue
        spec = quantity.value.distribution.spec
        if isinstance(spec, (UniformDistribution, NormalDistribution)) and spec.dimension == 1:
            continue
        raise TextXSemanticError(
            f"'{quantity.name}' samples a scalar from a distribution that is no 1-D uniform or "
            "normal -- a scalar draws one number, and a 3-D distribution belongs on a scene pose",
            **get_location(quantity),
        )


def _moved_leaves(node, world: dict) -> list:
    """Every leaf of NODE the motion moves, with the (frame, half) it moves along."""
    if isinstance(node, QOpNode):
        return [moved for operand in node.operands for moved in _moved_leaves(operand, world)]
    gradient = leaf_gradient(node, world)
    return [] if gradient is None else [(node, gradient)]


def _check_divisors(node, spec, location, world: dict) -> None:
    if not isinstance(node, QOpNode):
        return
    if node.op == "divide" and _moved_leaves(node.operands[1], world):
        raise TextXSemanticError(
            f"constraint '{spec.name}' divides by a term the motion moves -- the expression has "
            "no value where that term crosses zero; a monitor accepts it",
            **location,
        )
    for operand in node.operands:
        _check_divisors(operand, spec, location, world)


def validate_controlled_expressions(model: Model) -> None:
    """A controlled expression is driven along its own gradient, recomputed every cycle.

    So every term the motion moves states a direction, all of them in one frame and one half, and
    nothing divides by a moved term. A monitor keeps the full algebra.
    """
    models = get_included_models(model)
    for handler in get_children_of_type(ConstraintHandler, model):
        motion = handler.motion
        context_quantities = motion_context_quantities(models, motion, handler)
        world = motion_world_quantities(models, motion, handler, context_quantities)
        for entry in handler.controllers:
            controller = entry.ref.controller if isinstance(entry, ControllerAlias) else entry
            spec = controller.params.constraint.constraint
            if not isinstance(spec, ConstraintSpecification):
                continue
            view = spec.view
            quantity = view.quantity
            if isinstance(quantity, ContextQuantityAlias):
                quantity = quantity.ref
            # A snapshot holds a sampled value rather than tracking one.
            if view.expr is not None:
                tree = op_tree(view.expr)
            elif isinstance(quantity, ContextQuantity) and isinstance(quantity.value, ReferenceValue):
                tree = op_tree(quantity.value.expr)
            else:
                continue
            location = get_location(controller)
            try:
                moved = _moved_leaves(tree, world)
            except ValueError as exc:
                raise TextXSemanticError(
                    f"constraint '{spec.name}' has no direction to be driven along -- {exc}",
                    **location,
                ) from exc
            if not moved:
                raise TextXSemanticError(
                    f"constraint '{spec.name}' combines nothing the motion moves -- there is no "
                    "direction to drive it along",
                    **location,
                )
            _check_divisors(tree, spec, location, world)
            frames = {frame for _leaf, (frame, _half) in moved}
            if len(frames) > 1:
                raise TextXSemanticError(
                    f"constraint '{spec.name}' combines terms moving in different frames "
                    f"({', '.join(str(frame) for frame in frames)}) -- the gradient is one vector "
                    "in one frame",
                    **location,
                )
            halves = {half for _leaf, (_frame, half) in moved}
            if len(halves) > 1:
                raise TextXSemanticError(
                    f"constraint '{spec.name}' spans the linear and the angular half -- one solver "
                    "row carries one of them, so split it into one constraint per half",
                    **location,
                )
            command = controller_command_record(controller)
            if (command.is_force_command and halves == {"angular"}) or (
                command.is_moment_command and halves == {"linear"}
            ):
                raise TextXSemanticError(
                    f"controller '{controller.name}' commands a "
                    f"{'force' if command.is_force_command else 'moment'} along the "
                    f"{next(iter(halves))} half of '{spec.name}' -- a force pushes along a line "
                    "and a moment turns about an axis",
                    **location,
                )
