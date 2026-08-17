# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Validate quantity-expression trees: dimension inference agrees with what `rdf/motion_spec.py`
emits, because both call `classes.dimensions.infer`.
"""

from __future__ import annotations

from textx import get_children_of_type

from motion_spec_dsl.classes.context import (
    ContextQuantity,
    ContextRef,
    ReferenceValue,
    SnapshotValue,
    View,
    _resolved_context_quantity,
)
from motion_spec_dsl.classes.dimensions import DimensionError, infer
from motion_spec_dsl.classes.motion_spec import Model
from motion_spec_dsl.classes.validation.common import semantic_error


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


def _check_declared(quantity: ContextQuantity, expr) -> None:
    inferred = _infer_or_raise(expr, quantity)
    if quantity.type != inferred:
        raise semantic_error(
            f"'{quantity.name}' is declared {quantity.type}, but its expression infers {inferred}.",
            quantity,
        )


def _infer_or_raise(expr, obj):
    try:
        return infer(expr)
    except DimensionError as exc:
        raise semantic_error(str(exc), obj) from exc
