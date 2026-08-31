# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Validate quantity-expression trees: dimension inference agrees with what `rdf/motion_spec.py`
emits, because both call `classes.dimensions.infer`.
"""

from __future__ import annotations

from scene_dsl.classes.distrib import NormalDistribution, UniformDistribution
from textx import get_children_of_type

from motion_spec_dsl.classes.constraint_handler import _resolved_controller
from motion_spec_dsl.classes.context import (
    ContextQuantity,
    ContextRef,
    QOpNode,
    ReferenceValue,
    SampledValue,
    SnapshotValue,
    View,
    WorldQuantity,
    _resolved_context_quantity,
    _resolved_world_quantity,
)
from motion_spec_dsl.classes.dimensions import DimensionError, infer, same_scalar_dimension
from motion_spec_dsl.classes.motion_spec import Model
from motion_spec_dsl.classes.validation.common import constraint_handlers, semantic_error
from motion_spec_dsl.rdf.common import _quantity_axis_frame


def validate_expression_dimensions(model: Model) -> None:
    """Every authored quantity expression must type-check, and a declared quantity's stated
    type must equal what its expression infers.
    """
    for quantity in get_children_of_type(ContextQuantity, model):
        quantity = _resolved_context_quantity(quantity)
        value = quantity.value
        if isinstance(value, ReferenceValue):
            _check_declared(quantity, value.expr)
        elif isinstance(value, SnapshotValue) and value.tail:
            _check_declared(quantity, value)

    for ref in get_children_of_type(ContextRef, model):
        if ref.expr is not None:
            _infer_or_raise(ref.expr, ref)

    for view in get_children_of_type(View, model):
        if view.expr is not None:
            _infer_or_raise(view.expr, view)


def validate_sampled_quantities(model: Model) -> None:
    """A `sample <distribution>` scalar must draw from a 1-D uniform or normal distribution."""
    for quantity in get_children_of_type(ContextQuantity, model):
        quantity = _resolved_context_quantity(quantity)
        if not isinstance(quantity.value, SampledValue):
            continue
        spec = getattr(quantity.value.distribution, "spec", None)
        if isinstance(spec, (UniformDistribution, NormalDistribution)) and spec.dimension == 1:
            continue
        raise semantic_error(
            f"'{quantity.name}' samples a scalar quantity, which draws one number, so its "
            "distribution must be a 1-D uniform or normal; a 3-D distribution belongs on a "
            "scene pose.",
            quantity,
        )


def validate_controlled_expressions(model: Model) -> None:
    """A controlled expression is driven along its own gradient, so that gradient has to be a
    fact the model states rather than one the cycle discovers: the expression must be affine in
    the views a solver moves, its coefficients must be measure literals, and all those views must
    be seen by one frame. A monitor reads the value and keeps the full algebra.
    """
    for handler in constraint_handlers(model):
        for entry in handler.controllers:
            controller = _resolved_controller(entry)
            spec = controller.params.constraint.constraint
            tree = _controlled_tree(spec)
            if tree is None:
                continue
            _check_affine(tree, spec, controller)
            _check_one_frame(tree, spec, controller)


def _controlled_tree(spec):
    """The op tree a constraint acts on, or None when its view is not an expression. A snapshot
    holds a sampled value rather than tracking one, so it is not one either.
    """
    view = getattr(spec, "view", None)
    expr = getattr(view, "expr", None)
    if expr is not None:
        return expr.as_op_tree()
    quantity = getattr(view, "quantity", None)
    if isinstance(quantity, ContextQuantity):
        value = _resolved_context_quantity(quantity).value
        if isinstance(value, ReferenceValue):
            return value.expr.as_op_tree()
    return None


def _measured_leaves(node) -> list:
    """Every leaf of `node` viewing a world quantity -- the ones a solver actually moves."""
    if isinstance(node, QOpNode):
        return [leaf for operand in node.operands for leaf in _measured_leaves(operand)]
    return [node] if isinstance(getattr(node, "quantity", None), WorldQuantity) else []


def _is_literal(node) -> bool:
    """Whether `node` is built only from measure literals, so its value is a generation-time
    number a coefficient can be.
    """
    if isinstance(node, QOpNode):
        return all(_is_literal(operand) for operand in node.operands)
    return getattr(node, "bare", None) is not None


def _check_affine(node, spec, controller) -> None:
    if not isinstance(node, QOpNode):
        return
    # Only a product that reaches a measured view states a gradient; one built purely from
    # context quantities is a coefficient in its own right and may be computed per cycle.
    if node.op in ("multiply", "divide") and _measured_leaves(node):
        if node.op == "divide" and _measured_leaves(node.operands[1]):
            raise semantic_error(
                f"Constraint '{spec.name}' divides by a view it measures; a controlled "
                "expression divides only by a literal. A monitor accepts it.",
                controller,
            )
        if len([operand for operand in node.operands if _measured_leaves(operand)]) > 1:
            raise semantic_error(
                f"Constraint '{spec.name}' multiplies two measured views, so its gradient "
                "depends on what it measures; a controller cannot drive that yet. A monitor "
                "accepts it.",
                controller,
            )
        for operand in node.operands:
            if not _measured_leaves(operand) and not _is_literal(operand):
                raise semantic_error(
                    f"Constraint '{spec.name}' scales a measured view by a runtime value; a "
                    "controlled expression needs coefficients it can state at generation time. "
                    "A monitor accepts it.",
                    controller,
                )
    for operand in node.operands:
        _check_affine(operand, spec, controller)


def _check_one_frame(tree, spec, controller) -> None:
    frames = {
        _quantity_axis_frame(_resolved_world_quantity(leaf.quantity))
        for leaf in _measured_leaves(tree)
    }
    if len(frames) > 1:
        rendered = ", ".join(sorted(str(frame) for frame in frames))
        raise semantic_error(
            f"Constraint '{spec.name}' combines views seen by different frames ({rendered}); "
            "state them all as-seen-by one frame, since the gradient is one vector in one frame.",
            controller,
        )


def _check_declared(quantity: ContextQuantity, expr) -> None:
    inferred = _infer_or_raise(expr, quantity)
    if not same_scalar_dimension(quantity.type, inferred):
        raise semantic_error(
            f"'{quantity.name}' is declared {quantity.type}, but its expression infers {inferred}.",
            quantity,
        )


def _infer_or_raise(expr, obj):
    try:
        return infer(expr)
    except DimensionError as exc:
        raise semantic_error(str(exc), obj) from exc
