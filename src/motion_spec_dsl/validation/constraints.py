# SPDX-License-Identifier: MPL-2.0
"""Constraint, context quantity, and value-shape validation."""

from __future__ import annotations

from collections import defaultdict

from motion_spec_dsl.controller_semantics import axis_label
from motion_spec_dsl.domain import (
    AdmittanceSpec,
    BilateralConstraint,
    ConstraintSpecification,
    ContextQuantity,
    ContextDeclReference,
    ContextSpec,
    ContextRef,
    EqualityConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    Measure,
    Model,
    MotionSpec,
    ProfileSpec,
    QuantityType,
    ReferenceGeneratorType,
    ReferenceValue,
    SaturationSpec,
    SnapshotValue,
    SubSpace,
    TrajectoryValue,
    WorldQuantity,
    WorldQuantityType,
    _resolved_context_quantity,
    _resolved_world_quantity,
)
from motion_spec_dsl.validation.common import (
    constraint_handlers,
    motion_constraint_items,
    motion_constraints,
    motion_specs,
    semantic_error,
)


def _is_orientation_axis(axis: str | None) -> bool:
    return axis in {"x", "y", "z", "roll", "pitch", "yaw"}


_LINEAR_AXES = {"x", "y", "z"}
_SUBSPACE_AXES = {
    SubSpace.Position: _LINEAR_AXES,
    SubSpace.LinVel: _LINEAR_AXES,
    SubSpace.LinAcc: _LINEAR_AXES,
    SubSpace.Force: _LINEAR_AXES,
    SubSpace.AngVel: _LINEAR_AXES,
    SubSpace.AngAcc: _LINEAR_AXES,
    SubSpace.Torque: _LINEAR_AXES,
    SubSpace.Orientation: {"x", "y", "z", "roll", "pitch", "yaw"},
}


def _validate_axis(subspace: object, axis: str | None, owner: object) -> None:
    if axis is None:
        return
    allowed = _SUBSPACE_AXES.get(subspace)
    if allowed is None or axis not in allowed:
        raise semantic_error(
            f"Selector subspace '{subspace}' does not support axis '{axis}'.",
            owner,
        )


def _view_shape(view) -> QuantityType | None:
    if getattr(view, "is_elapsed", False):
        return QuantityType.Duration
    if getattr(view, "distance_from", None) is not None and getattr(view, "distance_to", None) is not None:
        return QuantityType.Distance
    norm_source = getattr(view, "norm_source", None)
    if norm_source is not None:
        inner_shape = _view_shape(norm_source)
        return {
            QuantityType.Position: QuantityType.Distance,
            QuantityType.Orientation: QuantityType.Angle,
        }.get(inner_shape, inner_shape)

    quantity = getattr(view, "quantity", None)
    if not isinstance(quantity, WorldQuantity):
        return None
    quantity = _resolved_world_quantity(quantity)
    subspace = getattr(view, "subspace", None)
    raw_axis = getattr(view, "axis", None)
    raw_axis_value = str(getattr(raw_axis, "value", raw_axis)) if raw_axis is not None else None
    _validate_axis(subspace, raw_axis_value, view)
    axis = axis_label(raw_axis)

    if quantity.type == WorldQuantityType.JointPosition:
        return QuantityType.Angle
    if quantity.type == WorldQuantityType.VelocityTwist:
        if subspace is None:
            return QuantityType.VelocityTwist
        if subspace == SubSpace.LinVel:
            return QuantityType.LinearVelocity
        if subspace == SubSpace.AngVel:
            return QuantityType.AngularVelocity
    if quantity.type == WorldQuantityType.Wrench:
        if subspace is None:
            return QuantityType.Wrench
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


def _context_quantity_shape(quantity: ContextQuantity) -> QuantityType | ReferenceGeneratorType:
    quantity = _resolved_context_quantity(quantity)
    return quantity.type


def _context_ref_shape(ref: ContextRef) -> QuantityType | ReferenceGeneratorType | None:
    value = context_ref_value(ref)
    if not isinstance(value, ContextQuantity):
        return None
    base_shape = _context_quantity_shape(value)
    subspace_raw = getattr(ref, "subspace", None)
    raw_axis = getattr(ref, "axis", None)
    raw_axis_value = str(getattr(raw_axis, "value", raw_axis)) if raw_axis is not None else None
    axis = axis_label(raw_axis)
    if subspace_raw is None:
        return base_shape
    subspace_value = str(getattr(subspace_raw, "value", subspace_raw))
    subspace = (
        SubSpace(subspace_value)
        if subspace_value in SubSpace._value2member_map_
        else subspace_value
    )
    _validate_axis(subspace, raw_axis_value, ref)
    if base_shape in {QuantityType.Pose, ReferenceGeneratorType.Trajectory}:
        if subspace == SubSpace.Position:
            return QuantityType.Distance if axis is not None else QuantityType.Position
        if subspace == SubSpace.Orientation:
            return QuantityType.Angle if _is_orientation_axis(axis) else QuantityType.Orientation
    if base_shape == QuantityType.VelocityTwist:
        if subspace == SubSpace.LinVel:
            return QuantityType.LinearVelocity
        if subspace == SubSpace.AngVel:
            return QuantityType.AngularVelocity
    if base_shape == QuantityType.AccelerationTwist:
        if subspace == SubSpace.LinAcc:
            return QuantityType.LinearAcceleration
        if subspace == SubSpace.AngAcc:
            return QuantityType.AngularAcceleration
    if base_shape == QuantityType.Wrench:
        if subspace == SubSpace.Force:
            return QuantityType.Force
        if subspace == SubSpace.Torque:
            return QuantityType.Torque
    return None


def _require_shape(shape: object | None, owner: object, message: str) -> object:
    if shape is None:
        raise semantic_error(message, owner)
    return shape


def _constraint_reference_shapes(
    constraint: ConstraintSpecification,
) -> list[tuple[ContextRef, QuantityType | ReferenceGeneratorType]]:
    refs = constraint_context_refs(constraint)
    pairs: list[tuple[ContextRef, QuantityType | ReferenceGeneratorType]] = []
    for ref in refs:
        bare = getattr(ref, "bare", None)
        if bare is not None:
            if getattr(bare, "unit", None) in {"s", "ms"}:
                pairs.append((ref, QuantityType.Duration))
            continue
        ref_shape = _require_shape(
            _context_ref_shape(ref),
            ref,
            f"Constraint '{constraint.name}' has an invalid reference selector.",
        )
        pairs.append((ref, ref_shape))
    return pairs


def _types_match(
    left: QuantityType | ReferenceGeneratorType | None,
    right: QuantityType | ReferenceGeneratorType | None,
) -> bool:
    if left is None or right is None:
        return True
    compatible = {
        QuantityType.Angle: {QuantityType.Angle, QuantityType.PlaneAngle},
        QuantityType.PlaneAngle: {QuantityType.Angle, QuantityType.PlaneAngle},
        QuantityType.Distance: {QuantityType.Distance},
        QuantityType.Position: {QuantityType.Position},
        QuantityType.Orientation: {QuantityType.Orientation},
        QuantityType.Pose: {QuantityType.Pose, ReferenceGeneratorType.Trajectory},
        QuantityType.Duration: {QuantityType.Duration},
        QuantityType.Dimensionless: {QuantityType.Dimensionless},
        # An Admittance quantity is a per-axis velocity reference: a velocity
        # constraint may track it exactly like a plain LinearVelocity setpoint.
        QuantityType.LinearVelocity: {
            QuantityType.LinearVelocity,
            ReferenceGeneratorType.Admittance,
        },
    }
    return right in compatible.get(left, {left})


def _static_scalar(ref: ContextRef) -> Measure | None:
    value = context_ref_value(ref)
    value = _resolved_context_quantity(value) if isinstance(value, ContextQuantity) else value
    scalar = getattr(value, "value", None)
    return scalar if isinstance(scalar, Measure) else None


def _check_profile_ref(
    profile: ProfileSpec,
    attr: str,
    expected: QuantityType,
    *,
    required: bool = True,
) -> None:
    ref = getattr(profile, attr)
    if ref is None:
        if required:
            raise semantic_error(f"VelocityProfile is missing {attr}.", profile)
        return
    actual = _context_ref_shape(ref)
    if actual != expected:
        raise semantic_error(
            f"VelocityProfile {attr} must reference {expected}, got {actual}.",
            ref,
        )
    scalar = _static_scalar(ref)
    if scalar is not None and scalar.value <= 0:
        raise semantic_error(f"VelocityProfile {attr} must be positive.", ref)


def validate_saturation_spec(
    saturation: SaturationSpec,
    *,
    expected: QuantityType | None,
    owner: object,
    label: str,
) -> None:
    refs = []
    if saturation.maximum is not None:
        refs.append(("max", saturation.maximum))
    else:
        if saturation.lower is None or saturation.upper is None:
            raise semantic_error(f"{label} saturation must specify max or lower and upper.", owner)
        refs.extend([("lower", saturation.lower), ("upper", saturation.upper)])

    for name, ref in refs:
        actual = _context_ref_shape(ref)
        if actual is None:
            raise semantic_error(f"{label} saturation {name} must reference a quantity.", ref)
        if expected is not None and actual != expected:
            raise semantic_error(
                f"{label} saturation {name} must reference {expected}, got {actual}.",
                ref,
            )
        scalar = _static_scalar(ref)
        if name == "max" and scalar is not None and scalar.value <= 0:
            raise semantic_error(f"{label} saturation max must be positive.", ref)


def _validate_profile_quantity(quantity: ContextQuantity) -> None:
    value = getattr(quantity, "value", None)
    if not isinstance(value, ProfileSpec):
        return
    if quantity.type != ReferenceGeneratorType.VelocityProfile:
        raise semantic_error(
            f"Profile '{quantity.name}' must be declared as VelocityProfile.",
            quantity,
        )
    shape = value.shape or "Trapezoidal"
    if shape not in {"Trapezoidal", "SCurve"}:
        raise semantic_error(f"VelocityProfile '{quantity.name}' has unsupported shape '{shape}'.", quantity)
    _check_profile_ref(value, "max_velocity", QuantityType.LinearVelocity)
    _check_profile_ref(value, "max_acceleration", QuantityType.LinearAcceleration)
    if value.measured_velocity is not None:
        measured_shape = _view_shape(value.measured_velocity)
        if measured_shape != QuantityType.LinearVelocity:
            raise semantic_error(
                f"VelocityProfile '{quantity.name}' measured_velocity must be LinearVelocity, got {measured_shape}.",
                value.measured_velocity,
            )
    if shape == "SCurve":
        _check_profile_ref(value, "max_jerk", QuantityType.LinearJerk)
    elif value.max_jerk is not None:
        raise semantic_error(
            f"VelocityProfile '{quantity.name}' may only specify max_jerk for shape SCurve.",
            value.max_jerk,
        )


def _axis_value(axis: object | None) -> str | None:
    return axis_label(axis)


def _geo_prop(obj: object, key: str) -> str | None:
    props = getattr(obj, "props", None)
    for pair in getattr(props, "pairs", []):
        if str(getattr(pair, "key", "")) == key:
            return getattr(pair, "value", None)
    return None


def _max_velocity_unit(value: AdmittanceSpec) -> str:
    unit = getattr(value, "max_velocity_unit", None)
    return str(getattr(unit, "value", unit)) if unit else "m/s"


def _validate_admittance_quantity(quantity: ContextQuantity) -> None:
    value = getattr(quantity, "value", None)
    if not isinstance(value, AdmittanceSpec):
        return
    if quantity.type != ReferenceGeneratorType.Admittance:
        raise semantic_error(
            f"Admittance '{quantity.name}' must be declared as Admittance.",
            quantity,
        )
    if value.mass <= 0:
        raise semantic_error(f"Admittance '{quantity.name}' mass must be positive.", value)
    if value.damping < 0:
        raise semantic_error(f"Admittance '{quantity.name}' damping must be non-negative.", value)
    if value.stiffness < 0:
        raise semantic_error(f"Admittance '{quantity.name}' stiffness must be non-negative.", value)
    if value.max_velocity <= 0:
        raise semantic_error(f"Admittance '{quantity.name}' max-velocity must be positive.", value)
    if _max_velocity_unit(value) not in {"m/s", "cm/s"}:
        raise semantic_error(
            f"Admittance '{quantity.name}' max-velocity must use m/s or cm/s.",
            value,
        )
    force = value.force
    force_shape = _require_shape(
        _view_shape(force),
        force,
        f"Admittance '{quantity.name}' has an invalid force selector.",
    )
    force_quantity = getattr(force, "quantity", None)
    if isinstance(force_quantity, WorldQuantity):
        force_quantity = _resolved_world_quantity(force_quantity)
    if (
        force_shape != QuantityType.Force
        or not isinstance(force_quantity, WorldQuantity)
        or force_quantity.type != WorldQuantityType.Wrench
        or getattr(force, "subspace", None) != SubSpace.Force
        or getattr(force, "axis", None) is None
    ):
        raise semantic_error(
            f"Admittance '{quantity.name}' force must reference a Wrench force axis.",
            force,
        )


def _validate_admittance_tracking(
    constraint: ConstraintSpecification,
    ref: ContextRef,
) -> None:
    admit_qty = context_ref_value(ref)
    if not isinstance(admit_qty, ContextQuantity):
        return
    admit_qty = _resolved_context_quantity(admit_qty)
    if admit_qty.type != ReferenceGeneratorType.Admittance:
        return
    value = getattr(admit_qty, "value", None)
    if not isinstance(value, AdmittanceSpec):
        return
    view = constraint.view
    if (
        _view_shape(view) != QuantityType.LinearVelocity
        or getattr(view, "subspace", None) != SubSpace.LinVel
        or getattr(view, "axis", None) is None
    ):
        raise semantic_error(
            f"Constraint '{constraint.name}' must track Admittance with a linear velocity axis.",
            constraint,
        )
    if _axis_value(getattr(view, "axis", None)) != _axis_value(getattr(value.force, "axis", None)):
        raise semantic_error(
            f"Constraint '{constraint.name}' Admittance force axis must match the tracked velocity axis.",
            ref,
        )
    velocity_quantity = getattr(view, "quantity", None)
    force_quantity = getattr(value.force, "quantity", None)
    if isinstance(velocity_quantity, WorldQuantity):
        velocity_quantity = _resolved_world_quantity(velocity_quantity)
    if isinstance(force_quantity, WorldQuantity):
        force_quantity = _resolved_world_quantity(force_quantity)
    velocity_frame = _geo_prop(velocity_quantity, "as-seen-by")
    force_frame = _geo_prop(force_quantity, "as-seen-by")
    if velocity_frame and force_frame and velocity_frame != force_frame:
        raise semantic_error(
            f"Constraint '{constraint.name}' Admittance force and velocity must use the same as-seen-by frame.",
            ref,
        )


def _check_trajectory_ref(
    quantity: ContextQuantity,
    attr: str,
    ref: ContextRef,
    expected: QuantityType,
) -> None:
    actual = _require_shape(
        _context_ref_shape(ref),
        ref,
        f"Trajectory '{quantity.name}' {attr} has an invalid selector.",
    )
    if actual != expected:
        raise semantic_error(
            f"Trajectory '{quantity.name}' {attr} must reference {expected}, got {actual}.",
            ref,
        )


def _validate_trajectory_quantity(quantity: ContextQuantity) -> None:
    value = getattr(quantity, "value", None)
    if not isinstance(value, TrajectoryValue):
        return
    if quantity.type != ReferenceGeneratorType.Trajectory:
        raise semantic_error(
            f"Trajectory '{quantity.name}' must be declared as Trajectory.",
            quantity,
        )
    if value.lerp is not None:
        _check_trajectory_ref(quantity, "start", value.lerp.start, QuantityType.Pose)
        _check_trajectory_ref(quantity, "goal", value.lerp.goal, QuantityType.Pose)
        _check_trajectory_ref(
            quantity, "alpha", value.lerp.alpha, QuantityType.TrajectoryProgress
        )
        return
    if value.circle is not None:
        _check_trajectory_ref(quantity, "start", value.circle.start, QuantityType.Pose)
        _check_trajectory_ref(quantity, "center", value.circle.center, QuantityType.Position)
        _check_trajectory_ref(
            quantity, "plane-normal", value.circle.plane_normal, QuantityType.Direction
        )
        _check_trajectory_ref(
            quantity, "alpha", value.circle.alpha, QuantityType.TrajectoryProgress
        )
        return
    if value.arc is not None:
        _check_trajectory_ref(quantity, "start", value.arc.start, QuantityType.Pose)
        _check_trajectory_ref(quantity, "end", value.arc.end, QuantityType.Pose)
        _check_trajectory_ref(quantity, "amplitude", value.arc.amplitude, QuantityType.Distance)
        _check_trajectory_ref(
            quantity, "plane-normal", value.arc.plane_normal, QuantityType.Direction
        )
        _check_trajectory_ref(
            quantity, "alpha", value.arc.alpha, QuantityType.TrajectoryProgress
        )
        return
    if value.helix is not None:
        _check_trajectory_ref(quantity, "start", value.helix.start, QuantityType.Pose)
        _check_trajectory_ref(quantity, "center", value.helix.center, QuantityType.Position)
        _check_trajectory_ref(quantity, "axis", value.helix.axis, QuantityType.Direction)
        _check_trajectory_ref(quantity, "pitch", value.helix.pitch, QuantityType.Distance)
        _check_trajectory_ref(
            quantity, "revolutions", value.helix.revolutions, QuantityType.Dimensionless
        )
        _check_trajectory_ref(
            quantity, "alpha", value.helix.alpha, QuantityType.TrajectoryProgress
        )
        return
    if value.figure8 is not None:
        _check_trajectory_ref(quantity, "anchor", value.figure8.anchor, QuantityType.Pose)
        _check_trajectory_ref(quantity, "radius", value.figure8.radius, QuantityType.Distance)
        _check_trajectory_ref(
            quantity, "plane-normal", value.figure8.plane_normal, QuantityType.Direction
        )
        _check_trajectory_ref(
            quantity, "alpha", value.figure8.alpha, QuantityType.TrajectoryProgress
        )
        return
    raise semantic_error(f"Trajectory '{quantity.name}' has no populated spec.", quantity)


def validate_context_quantity_values(model: Model) -> None:
    for motion in motion_specs(model):
        for ctx in motion.context:
            ctx = _resolved_context_decl(ctx)
            for item in getattr(ctx, "declaration", []):
                if not isinstance(item, ContextQuantity):
                    continue
                quantity = _resolved_context_quantity(item)
                value = getattr(quantity, "value", None)
                _validate_profile_quantity(quantity)
                _validate_admittance_quantity(quantity)
                _validate_trajectory_quantity(quantity)
                if isinstance(value, ReferenceValue):
                    source_shape = _require_shape(
                        _context_ref_shape(value.source),
                        value.source,
                        f"Reference '{quantity.name}' has an invalid source selector.",
                    )
                    if not _types_match(quantity.type, source_shape):
                        raise semantic_error(
                            f"Reference '{quantity.name}' is declared as {quantity.type}, "
                            f"but its source has type {source_shape}.",
                            quantity,
                        )
                    if value.offset is not None:
                        offset_shape = _require_shape(
                            _context_ref_shape(value.offset),
                            value.offset,
                            f"Reference '{quantity.name}' has an invalid offset selector.",
                        )
                        if not _types_match(quantity.type, offset_shape):
                            raise semantic_error(
                                f"Reference '{quantity.name}' is declared as {quantity.type}, "
                                f"but its offset has type {offset_shape}.",
                                quantity,
                            )
                if not isinstance(value, SnapshotValue):
                    continue
                source_shape = _require_shape(
                    _view_shape(value.source),
                    value.source,
                    f"Snapshot '{quantity.name}' has an invalid source selector.",
                )
                if not _types_match(quantity.type, source_shape):
                    raise semantic_error(
                        f"Snapshot '{quantity.name}' is declared as {quantity.type}, "
                        f"but its source has type {source_shape}.",
                        quantity,
                    )
    for handler in constraint_handlers(model):
        for ctx in handler.context:
            ctx = _resolved_context_decl(ctx)
            for item in getattr(ctx, "declaration", []):
                if isinstance(item, ContextQuantity):
                    quantity = _resolved_context_quantity(item)
                    _validate_profile_quantity(quantity)
                    _validate_admittance_quantity(quantity)
                    _validate_trajectory_quantity(quantity)


def validate_constraint_value_types(model: Model) -> None:
    for motion in motion_specs(model):
        for constraint in motion_constraints(motion):
            view_shape = _require_shape(
                _view_shape(constraint.view),
                constraint.view,
                f"Constraint '{constraint.name}' has an invalid view selector.",
            )
            for ref, ref_shape in _constraint_reference_shapes(constraint):
                if not _types_match(view_shape, ref_shape):
                    raise semantic_error(
                        f"Constraint '{constraint.name}' compares {view_shape} "
                        f"with {ref_shape}.",
                        ref,
                    )
                _validate_admittance_tracking(constraint, ref)


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


def _decl_context_spec(obj: object) -> ContextSpec | None:
    context = getattr(obj, "parent", None)
    spec = getattr(context, "parent", None)
    return spec if isinstance(spec, ContextSpec) else None


def _resolved_context_decl(ctx: object) -> object:
    return ctx.ref if isinstance(ctx, ContextDeclReference) else ctx


def context_ref_value(ref: ContextRef) -> ContextQuantity | None:
    if getattr(ref, "bare", None) is not None:
        return None
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
    view = constraint.view
    if view is None or getattr(view, "is_elapsed", False):
        return []
    # `Norm of <inner>` — unwrap to the reduced view's world quantity.
    while getattr(view, "norm_source", None) is not None:
        view = view.norm_source
    if getattr(view, "is_elapsed", False):
        return []
    if (
        getattr(view, "distance_from", None) is not None
        and getattr(view, "distance_to", None) is not None
    ):
        return [view.distance_from, view.distance_to]
    return [view.quantity]


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
                if getattr(ref, "bare", None) is not None:
                    continue
                value = context_ref_value(ref)
                value = _resolved_context_quantity(value) if isinstance(value, ContextQuantity) else value
                if value is None:
                    raise semantic_error(
                        f"Constraint '{constraint.name}' has an unresolved context reference.",
                        constraint,
                    )
                if _decl_motion(value) is None and _decl_context_spec(value) is None and _is_inline_context_value(value):
                    continue
                if _decl_motion(value) is None and _decl_context_spec(value) is None:
                    raise semantic_error(
                        f"Constraint '{constraint.name}' references value '{value.name}', "
                        "but it is not resolved.",
                        ref,
                    )
