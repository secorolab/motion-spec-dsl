# SPDX-License-Identifier: MPL-2.0
"""Constraint, context quantity, and value-shape validation."""

from __future__ import annotations

from collections import defaultdict

from motion_spec_dsl.controller_semantics import axis_label
from motion_spec_dsl.domain import (
    BilateralConstraint,
    ConstraintSpecification,
    ContextQuantity,
    ContextRef,
    EqualityConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    Model,
    MotionSpec,
    QuantityType,
    SnapshotValue,
    SubSpace,
    WorldQuantity,
    WorldQuantityType,
    _resolved_context_quantity,
    _resolved_world_quantity,
)
from motion_spec_dsl.validation.common import (
    motion_constraint_items,
    motion_constraints,
    motion_specs,
    semantic_error,
)


def _is_orientation_axis(axis: str | None) -> bool:
    return axis in {"x", "y", "z", "roll", "pitch", "yaw"}


def _view_shape(view) -> QuantityType | None:
    if getattr(view, "distance_from", None) is not None and getattr(view, "distance_to", None) is not None:
        return QuantityType.Distance

    quantity = getattr(view, "quantity", None)
    if not isinstance(quantity, WorldQuantity):
        return None
    quantity = _resolved_world_quantity(quantity)
    subspace = getattr(view, "subspace", None)
    axis = axis_label(getattr(view, "axis", None))

    if quantity.type == WorldQuantityType.JointPosition:
        return QuantityType.Angle
    if quantity.type == WorldQuantityType.VelocityTwist:
        if subspace == SubSpace.LinVel:
            return QuantityType.LinearVelocity
        if subspace == SubSpace.AngVel:
            return QuantityType.AngularVelocity
    if quantity.type == WorldQuantityType.Wrench:
        if subspace == SubSpace.Force:
            return QuantityType.Force
        if subspace == SubSpace.Torque:
            return QuantityType.Torque
    if quantity.type == WorldQuantityType.Pose:
        if subspace is None:
            return QuantityType.Pose
        if subspace == SubSpace.Position:
            return QuantityType.Distance if axis is not None else QuantityType.Position
        if subspace == SubSpace.Orientation:
            return QuantityType.Angle if _is_orientation_axis(axis) else QuantityType.Orientation
    return None


def _context_quantity_shape(quantity: ContextQuantity) -> QuantityType:
    quantity = _resolved_context_quantity(quantity)
    return quantity.type


def _constraint_reference_shapes(constraint: ConstraintSpecification) -> list[tuple[ContextRef, QuantityType]]:
    refs = constraint_context_refs(constraint)
    pairs: list[tuple[ContextRef, QuantityType]] = []
    for ref in refs:
        value = context_ref_value(ref)
        if isinstance(value, ContextQuantity):
            pairs.append((ref, _context_quantity_shape(value)))
    return pairs


def _types_match(left: QuantityType | None, right: QuantityType | None) -> bool:
    if left is None or right is None:
        return True
    compatible = {
        QuantityType.Angle: {QuantityType.Angle, QuantityType.AngularDistance, QuantityType.PlaneAngle},
        QuantityType.AngularDistance: {QuantityType.Angle, QuantityType.AngularDistance, QuantityType.PlaneAngle},
        QuantityType.PlaneAngle: {QuantityType.Angle, QuantityType.AngularDistance, QuantityType.PlaneAngle},
        QuantityType.Distance: {QuantityType.Distance},
        QuantityType.Position: {QuantityType.Position, QuantityType.Pose},
        QuantityType.Orientation: {QuantityType.Orientation, QuantityType.Pose},
    }
    return right in compatible.get(left, {left})


def validate_context_quantity_values(model: Model) -> None:
    for motion in motion_specs(model):
        for ctx in motion.context:
            for item in getattr(ctx, "declaration", []):
                if not isinstance(item, ContextQuantity):
                    continue
                quantity = _resolved_context_quantity(item)
                value = getattr(quantity, "value", None)
                if not isinstance(value, SnapshotValue):
                    continue
                source_shape = _view_shape(value.source)
                if not _types_match(quantity.type, source_shape):
                    raise semantic_error(
                        f"Snapshot '{quantity.name}' is declared as {quantity.type}, "
                        f"but its source has type {source_shape}.",
                        quantity,
                    )


def validate_constraint_value_types(model: Model) -> None:
    for motion in motion_specs(model):
        for constraint in motion_constraints(motion):
            view_shape = _view_shape(constraint.view)
            for ref, ref_shape in _constraint_reference_shapes(constraint):
                if not _types_match(view_shape, ref_shape):
                    raise semantic_error(
                        f"Constraint '{constraint.name}' compares {view_shape} "
                        f"with {ref_shape}.",
                        ref,
                    )


def validate_unique_constraint_names(model: Model) -> None:
    for motion in motion_specs(model):
        items_by_name: dict[str, list[object]] = defaultdict(list)
        for item in motion_constraint_items(motion):
            items_by_name[item.name].append(item)

        duplicates = {name: items for name, items in items_by_name.items() if len(items) > 1}
        if duplicates:
            names = ", ".join(sorted(duplicates))
            first_duplicate = next(iter(duplicates.values()))[1]
            raise semantic_error(
                f"Motion '{motion.name}' has duplicate constraint name(s): {names}. "
                "Constraint names must be unique across WHEN, WHILE, and UNTIL.",
                first_duplicate,
            )


def validate_constraint_aliases(model: Model) -> None:
    del model


def validate_context_aliases(model: Model) -> None:
    del model


def _decl_motion(obj: object) -> MotionSpec | None:
    context = getattr(obj, "parent", None)
    motion = getattr(context, "parent", None)
    return motion if isinstance(motion, MotionSpec) else None


def context_ref_value(ref: ContextRef) -> ContextQuantity | None:
    value = (
        getattr(ref, "quantity", None)
        or getattr(ref, "value", None)
        or getattr(ref, "inline_quantity", None)
    )
    return value if isinstance(value, ContextQuantity) else None


def _is_inline_context_value(value: ContextQuantity) -> bool:
    return isinstance(getattr(value, "parent", None), ContextRef) and getattr(
        value.parent,
        "context_scope",
        None,
    ) is not None


def constraint_context_refs(constraint: ConstraintSpecification) -> list[ContextRef]:
    expr = constraint.expr
    if isinstance(expr, EqualityConstraint):
        return [expr.reference]
    if isinstance(expr, (GreaterThanConstraint, LessThanConstraint)):
        return [expr.threshold]
    if isinstance(expr, BilateralConstraint):
        return [expr.lower, expr.upper]
    return []


def _constraint_view_quantities(constraint: ConstraintSpecification) -> list[object | None]:
    if (
        getattr(constraint.view, "distance_from", None) is not None
        and getattr(constraint.view, "distance_to", None) is not None
    ):
        return [constraint.view.distance_from, constraint.view.distance_to]
    return [constraint.view.quantity]


def validate_constraint_context_refs(model: Model) -> None:
    for motion in motion_specs(model):
        for constraint in motion_constraints(motion):
            for quantity in _constraint_view_quantities(constraint):
                quantity = (
                    _resolved_world_quantity(quantity)
                    if isinstance(quantity, WorldQuantity)
                    else quantity
                )
                if not isinstance(quantity, WorldQuantity):
                    raise semantic_error(
                        f"Constraint '{constraint.name}' references world quantity "
                        f"'{quantity}', but it is not resolved.",
                        constraint,
                    )

            for ref in constraint_context_refs(constraint):
                value = context_ref_value(ref)
                value = _resolved_context_quantity(value) if isinstance(value, ContextQuantity) else value
                if value is None:
                    raise semantic_error(
                        f"Constraint '{constraint.name}' has an unresolved context reference.",
                        constraint,
                    )
                if _decl_motion(value) is None and _is_inline_context_value(value):
                    continue
                if _decl_motion(value) is None:
                    raise semantic_error(
                        f"Constraint '{constraint.name}' references value '{value.name}', "
                        "but it is not resolved.",
                        ref,
                    )
