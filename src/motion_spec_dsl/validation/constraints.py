# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Constraint, context quantity, and value-shape validation."""

from __future__ import annotations

from collections import defaultdict
import math

from motion_spec_dsl.controller_semantics import axis_label
from motion_spec_dsl.classes import (
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
    GuardedMotion,
    OutsideConstraint,
    PoseValue,
    ProfileSpec,
    ProgressConstraint,
    ProgressObjective,
    QuantityType,
    ReferenceGeneratorType,
    ReferenceValue,
    SaturationSpec,
    SnapshotValue,
    SubSpace,
    PathValue,
    VectorQuantity,
    WorldQuantity,
    WorldQuantityType,
    _resolved_context_quantity,
    _resolved_solver,
    _resolved_world_quantity,
    _resolved_spec,
)
from motion_spec_dsl.validation.common import (
    constraint_handlers,
    motion_constraint_items,
    motion_constraints,
    motion_specs,
    semantic_error,
)


def _is_orientation_axis(axis: str | None) -> bool:
    """Whether `axis` is a valid axis token (x/y/z or roll/pitch/yaw)."""
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
    """Raise if `axis` is not permitted for `subspace`."""
    if axis is None:
        return
    allowed = _SUBSPACE_AXES.get(subspace)
    if allowed is None or axis not in allowed:
        raise semantic_error(
            f"Selector subspace '{subspace}' does not support axis '{axis}'.",
            owner,
        )


def _view_shape(view) -> QuantityType | None:
    """The QuantityType a constraint view resolves to, or None."""
    if getattr(view, "is_elapsed", False):
        return QuantityType.Duration
    if (
        getattr(view, "distance_from", None) is not None
        and getattr(view, "distance_to", None) is not None
    ):
        return QuantityType.Distance
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
    """The resolved type of a context quantity."""
    quantity = _resolved_context_quantity(quantity)
    return quantity.type


def _context_ref_shape(ref: ContextRef) -> QuantityType | ReferenceGeneratorType | None:
    """The QuantityType/generator a context reference resolves to for its subspace/axis,
    or None.
    """
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
    if base_shape in {QuantityType.Pose, ReferenceGeneratorType.Path}:
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
    """Return `shape`, raising `message` when it is None."""
    if shape is None:
        raise semantic_error(message, owner)
    return shape


def _constraint_reference_shapes(
    constraint: ConstraintSpecification,
) -> list[tuple[ContextRef, QuantityType | ReferenceGeneratorType]]:
    """The (ref, resolved-shape) pairs for a constraint's context references, including
    bare duration literals.
    """
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
    """Whether two constraint operand types are comparable (with the allowed equivalences)."""
    if left is None or right is None:
        return True
    compatible = {
        QuantityType.Angle: {QuantityType.Angle, QuantityType.PlaneAngle},
        QuantityType.PlaneAngle: {QuantityType.Angle, QuantityType.PlaneAngle},
        QuantityType.Distance: {QuantityType.Distance},
        QuantityType.Position: {QuantityType.Position},
        QuantityType.Orientation: {QuantityType.Orientation},
        QuantityType.Pose: {QuantityType.Pose, ReferenceGeneratorType.Path},
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
    """The static Measure a reference resolves to (bare literal or declared value), or None."""
    bare = getattr(ref, "bare", None)
    if isinstance(bare, Measure):
        return bare
    value = context_ref_value(ref)
    value = _resolved_context_quantity(value) if isinstance(value, ContextQuantity) else value
    scalar = getattr(value, "value", None)
    return scalar if isinstance(scalar, Measure) else None


_Vec3 = tuple[float, float, float]


def _static_vector(ref: ContextRef) -> _Vec3 | None:
    """The static (x, y, z) a Direction/Position reference resolves to, or None when it is
    dynamic (a snapshot/reference) rather than a literal vector.
    """
    literal = getattr(ref, "literal_value", None)
    if isinstance(literal, VectorQuantity):
        return (literal.x, literal.y, literal.z)
    value = context_ref_value(ref)
    value = _resolved_context_quantity(value) if isinstance(value, ContextQuantity) else value
    value = getattr(value, "value", value)
    return (value.x, value.y, value.z) if isinstance(value, VectorQuantity) else None


def _static_position(ref: ContextRef) -> _Vec3 | None:
    """The static (x, y, z) a Position/Pose reference resolves to, or None when any
    component depends on runtime state (a snapshot or a non-literal reference).
    """
    vector = _static_vector(ref)
    if vector is not None:
        return vector
    value = context_ref_value(ref)
    value = _resolved_context_quantity(value) if isinstance(value, ContextQuantity) else value
    value = getattr(value, "value", value)
    if not isinstance(value, PoseValue):
        return None
    axes: dict[str, float] = {}
    for term in value.position.terms:
        if term.ref is not None:
            scalar = _static_scalar(term.ref)
            if scalar is None:
                return None
            axes[term.axis] = scalar.value
        else:
            axes[term.axis] = term.value
    if not {"x", "y", "z"} <= axes.keys():
        return None
    return (axes["x"], axes["y"], axes["z"])


def _vec_sub(a: _Vec3, b: _Vec3) -> _Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vec_dot(a: _Vec3, b: _Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _vec_norm(a: _Vec3) -> float:
    return math.sqrt(_vec_dot(a, a))


_GEOMETRY_TOLERANCE = 1e-6


def _require_finite_vector(vector: _Vec3, owner: object, message: str) -> None:
    if not all(math.isfinite(c) for c in vector):
        raise semantic_error(message, owner)


def _require_nonzero_direction(ref: ContextRef, label: str, owner: object) -> None:
    """Raise if a statically-known direction is zero or not unit length; do nothing for a
    dynamic (runtime-resolved) direction, which cannot be checked at author time.
    """
    vector = _static_vector(ref)
    if vector is None:
        return
    _require_finite_vector(vector, owner, f"{label} must be finite.")
    norm = _vec_norm(vector)
    if norm < _GEOMETRY_TOLERANCE:
        raise semantic_error(f"{label} must be a non-zero vector.", owner)
    if abs(norm - 1.0) > 1e-3:
        raise semantic_error(f"{label} must be a unit-length direction.", owner)


def _check_profile_ref(
    profile: ProfileSpec,
    attr: str,
    expected: QuantityType,
    *,
    required: bool = True,
) -> None:
    """Raise if a velocity profile's `attr` reference is missing, mis-typed, or non-positive."""
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
    """Raise if a saturation spec is malformed or its limit references have the wrong type."""
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
    """Raise if a velocity-profile quantity has invalid physical motion limits."""
    value = getattr(quantity, "value", None)
    if not isinstance(value, ProfileSpec):
        return
    if quantity.type != ReferenceGeneratorType.VelocityProfile:
        raise semantic_error(
            f"Profile '{quantity.name}' must be declared as VelocityProfile.",
            quantity,
        )
    shape = value.shape or "trapezoidal"
    if shape not in {"trapezoidal", "s-curve"}:
        raise semantic_error(
            f"VelocityProfile '{quantity.name}' has unsupported shape '{shape}'.", quantity
        )
    _check_profile_ref(value, "max_velocity", QuantityType.LinearVelocity)
    _check_profile_ref(value, "max_acceleration", QuantityType.LinearAcceleration)
    if value.measured_velocity is not None:
        measured_shape = _view_shape(value.measured_velocity)
        if measured_shape != QuantityType.LinearVelocity:
            raise semantic_error(
                f"VelocityProfile '{quantity.name}' measured_velocity must be LinearVelocity, got {measured_shape}.",
                value.measured_velocity,
            )
    if shape == "s-curve":
        _check_profile_ref(value, "max_jerk", QuantityType.LinearJerk)
    elif value.max_jerk is not None:
        raise semantic_error(
            f"VelocityProfile '{quantity.name}' may only specify max_jerk for shape s-curve.",
            value.max_jerk,
        )


def _axis_value(axis: object | None) -> str | None:
    """Normalize an axis token to x/y/z (alias of axis_label)."""
    return axis_label(axis)


def _geo_prop(obj: object, key: str) -> str | None:
    """The value of geometric prop `key` on `obj`'s props, or None."""
    props = getattr(obj, "props", None)
    for pair in getattr(props, "pairs", []):
        if str(getattr(pair, "key", "")) == key:
            return getattr(pair, "value", None)
    return None


def _max_velocity_unit(value: AdmittanceSpec) -> str:
    """The admittance spec's max-velocity unit string (default m/s)."""
    unit = getattr(value, "max_velocity_unit", None)
    return str(getattr(unit, "value", unit)) if unit else "m/s"


def _validate_admittance_quantity(quantity: ContextQuantity) -> None:
    """Raise if an admittance quantity is mis-declared, has non-positive mass/velocity or
    negative damping/stiffness, or a force selector that is not a Wrench force axis.
    """
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
    """Raise unless `constraint` tracks the admittance reference on a matching
    linear-velocity axis and as-seen-by frame.
    """
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


def _check_path_ref(
    quantity: ContextQuantity,
    attr: str,
    ref: ContextRef,
    expected: QuantityType,
) -> None:
    """Raise if a path's `attr` reference is missing or not `expected`."""
    actual = _require_shape(
        _context_ref_shape(ref),
        ref,
        f"Path '{quantity.name}' {attr} has an invalid selector.",
    )
    if actual != expected:
        raise semantic_error(
            f"Path '{quantity.name}' {attr} must reference {expected}, got {actual}.",
            ref,
        )


def _require_positive_scalar(ref: ContextRef, label: str, owner: object) -> None:
    """Raise if a statically-known scalar is non-finite or non-positive; skip a dynamic one."""
    scalar = _static_scalar(ref)
    if scalar is None:
        return
    if not math.isfinite(scalar.value):
        raise semantic_error(f"{label} must be finite.", owner)
    if scalar.value <= 0.0:
        raise semantic_error(f"{label} must be positive.", owner)


def _validate_circle_geometry(quantity: ContextQuantity, circle) -> None:
    _require_nonzero_direction(circle.plane_normal, f"Circle '{quantity.name}' plane-normal", circle)
    start = _static_position(circle.start)
    center = _static_position(circle.center)
    normal = _static_vector(circle.plane_normal)
    if start is None or center is None:
        return
    radial = _vec_sub(start, center)
    _require_finite_vector(radial, circle, f"Circle '{quantity.name}' start/center must be finite.")
    if _vec_norm(radial) < _GEOMETRY_TOLERANCE:
        raise semantic_error(
            f"Circle '{quantity.name}' start must be a non-zero radius away from center.", circle
        )
    if normal is not None and abs(_vec_dot(radial, normal)) > _GEOMETRY_TOLERANCE * max(
        1.0, _vec_norm(radial)
    ):
        raise semantic_error(
            f"Circle '{quantity.name}' start-to-center radius must be perpendicular to plane-normal.",
            circle,
        )


def _validate_arc_geometry(quantity: ContextQuantity, arc) -> None:
    _require_nonzero_direction(arc.plane_normal, f"Arc '{quantity.name}' plane-normal", arc)
    _require_positive_scalar(arc.amplitude, f"Arc '{quantity.name}' amplitude", arc)
    start = _static_position(arc.start)
    end = _static_position(arc.end)
    normal = _static_vector(arc.plane_normal)
    if start is None or end is None:
        return
    chord = _vec_sub(end, start)
    _require_finite_vector(chord, arc, f"Arc '{quantity.name}' start/end must be finite.")
    chord_len = _vec_norm(chord)
    if chord_len < _GEOMETRY_TOLERANCE:
        raise semantic_error(f"Arc '{quantity.name}' start and end must be distinct.", arc)
    if normal is not None and abs(_vec_dot(chord, normal)) > _GEOMETRY_TOLERANCE * chord_len:
        raise semantic_error(
            f"Arc '{quantity.name}' plane-normal must be perpendicular to the start-end chord.", arc
        )
    amplitude = _static_scalar(arc.amplitude)
    if amplitude is not None and amplitude.value > 0.5 * chord_len + _GEOMETRY_TOLERANCE:
        raise semantic_error(
            f"Arc '{quantity.name}' amplitude (sagitta) must not exceed half the chord length.",
            arc.amplitude,
        )


def _validate_helix_geometry(quantity: ContextQuantity, helix) -> None:
    _require_nonzero_direction(helix.axis, f"Helix '{quantity.name}' axis", helix)
    _require_positive_scalar(helix.revolutions, f"Helix '{quantity.name}' revolutions", helix)
    pitch = _static_scalar(helix.pitch)
    if pitch is not None and not math.isfinite(pitch.value):
        raise semantic_error(f"Helix '{quantity.name}' pitch must be finite.", helix.pitch)
    start = _static_position(helix.start)
    center = _static_position(helix.center)
    axis = _static_vector(helix.axis)
    if start is None or center is None or axis is None:
        return
    arm = _vec_sub(start, center)
    axial = _vec_dot(arm, axis)
    radial = _vec_sub(arm, (axial * axis[0], axial * axis[1], axial * axis[2]))
    if _vec_norm(radial) < _GEOMETRY_TOLERANCE:
        raise semantic_error(
            f"Helix '{quantity.name}' start must have a non-zero radial offset from center "
            "(perpendicular to axis).",
            helix,
        )


def _validate_figure8_geometry(quantity: ContextQuantity, figure8) -> None:
    _require_nonzero_direction(figure8.plane_normal, f"Figure8 '{quantity.name}' plane-normal", figure8)
    _require_nonzero_direction(figure8.direction, f"Figure8 '{quantity.name}' direction", figure8)
    _require_positive_scalar(figure8.radius, f"Figure8 '{quantity.name}' radius", figure8)


def _validate_path_quantity(quantity: ContextQuantity) -> None:
    """Raise if a path quantity is mis-declared or its shape inputs are invalid."""
    value = getattr(quantity, "value", None)
    if not isinstance(value, PathValue):
        return
    if quantity.type != ReferenceGeneratorType.Path:
        raise semantic_error(
            f"Path '{quantity.name}' must be declared as Path.",
            quantity,
        )
    if value.lerp is not None:
        _check_path_ref(quantity, "start", value.lerp.start, QuantityType.Pose)
        _check_path_ref(quantity, "goal", value.lerp.goal, QuantityType.Pose)
        return
    if value.circle is not None:
        _check_path_ref(quantity, "start", value.circle.start, QuantityType.Pose)
        _check_path_ref(quantity, "center", value.circle.center, QuantityType.Position)
        _check_path_ref(
            quantity, "plane-normal", value.circle.plane_normal, QuantityType.Direction
        )
        _validate_circle_geometry(quantity, value.circle)
        return
    if value.arc is not None:
        _check_path_ref(quantity, "start", value.arc.start, QuantityType.Pose)
        _check_path_ref(quantity, "end", value.arc.end, QuantityType.Pose)
        _check_path_ref(quantity, "amplitude", value.arc.amplitude, QuantityType.Distance)
        _check_path_ref(
            quantity, "plane-normal", value.arc.plane_normal, QuantityType.Direction
        )
        _validate_arc_geometry(quantity, value.arc)
        return
    if value.helix is not None:
        _check_path_ref(quantity, "start", value.helix.start, QuantityType.Pose)
        _check_path_ref(quantity, "center", value.helix.center, QuantityType.Position)
        _check_path_ref(quantity, "axis", value.helix.axis, QuantityType.Direction)
        _check_path_ref(quantity, "pitch", value.helix.pitch, QuantityType.Distance)
        _check_path_ref(
            quantity, "revolutions", value.helix.revolutions, QuantityType.Dimensionless
        )
        _validate_helix_geometry(quantity, value.helix)
        return
    if value.figure8 is not None:
        _check_path_ref(quantity, "anchor", value.figure8.anchor, QuantityType.Pose)
        _check_path_ref(quantity, "radius", value.figure8.radius, QuantityType.Distance)
        _check_path_ref(
            quantity, "plane-normal", value.figure8.plane_normal, QuantityType.Direction
        )
        _check_path_ref(
            quantity, "direction", value.figure8.direction, QuantityType.Direction
        )
        _validate_figure8_geometry(quantity, value.figure8)
        return
    raise semantic_error(f"Path '{quantity.name}' has no populated spec.", quantity)


def validate_context_quantity_values(model: Model) -> None:
    """Validate every context quantity's value: profile/admittance/path specs and
    reference/snapshot source-type matching.
    """
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
                _validate_path_quantity(quantity)
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
                # An event-triggered snapshot re-samples on a declared FSM event, so the
                # trigger must name one: the standalone form mints a monitor-owned event.
                if value.trigger is not None and value.trigger.event is None:
                    raise semantic_error(
                        f"Snapshot '{quantity.name}' triggers on "
                        f"'{value.trigger.standalone}', which is not a declared FSM event. "
                        "Use the namespace-qualified form, e.g. 'on event ns.E_NAME'.",
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
                    _validate_path_quantity(quantity)


def validate_constraint_value_types(model: Model) -> None:
    """Raise if a constraint compares operands of incompatible types, or mis-tracks an
    admittance reference.
    """
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
                        f"Constraint '{constraint.name}' compares {view_shape} with {ref_shape}.",
                        ref,
                    )
                _validate_admittance_tracking(constraint, ref)


def validate_elapsed_constraint_relations(model: Model) -> None:
    """Elapsed (timing) views only support greater-than, less-than, and tolerance-bound
    equality: a sampled control loop cannot land on one exact instant, and a bilateral/
    outside band over a duration has no specified controller/monitor semantics. Tolerance
    itself is scoped to elapsed equality until non-time equality semantics are decided.
    """
    for motion in motion_specs(model):
        for constraint in motion_constraints(motion):
            is_elapsed = getattr(constraint.view, "is_elapsed", False)
            expr = constraint.expr
            if is_elapsed and isinstance(expr, (BilateralConstraint, OutsideConstraint)):
                raise semantic_error(
                    f"Constraint '{constraint.name}' is elapsed but uses a between/outside "
                    "relation; use 'greater than', 'less than', or 'equal to ... within ...'.",
                    constraint,
                )
            if not isinstance(expr, EqualityConstraint):
                continue
            if not is_elapsed:
                if expr.tolerance is not None:
                    raise semantic_error(
                        f"Constraint '{constraint.name}' has a 'within' tolerance, but only "
                        "elapsed equality constraints support tolerance in this plan.",
                        expr.tolerance,
                    )
                continue
            if expr.tolerance is None:
                raise semantic_error(
                    f"Constraint '{constraint.name}' is an elapsed equality and needs "
                    "'within <tolerance>': a sampled clock cannot land on one exact instant.",
                    expr,
                )
            scalar = _static_scalar(expr.tolerance)
            if scalar is None:
                continue
            if not math.isfinite(scalar.value) or scalar.value < 0:
                raise semantic_error(
                    f"Constraint '{constraint.name}' tolerance must be a finite, "
                    "non-negative duration.",
                    expr.tolerance,
                )
            if scalar.unit not in {"s", "ms"}:
                raise semantic_error(
                    f"Constraint '{constraint.name}' tolerance must use 's' or 'ms', "
                    f"not '{scalar.unit}'.",
                    expr.tolerance,
                )


# No solver algorithm currently exposes an optimization input for a scalar path
# objective; add its name here once one does.
_OBJECTIVE_CAPABLE_SOLVER_ALGORITHMS: frozenset[str] = frozenset()


def _progress_parameter(entry) -> ContextQuantity:
    """Raise unless `entry.parameter` resolves to a declared PathParameter quantity."""
    parameter = context_ref_value(entry.parameter)
    parameter = (
        _resolved_context_quantity(parameter) if isinstance(parameter, ContextQuantity) else parameter
    )
    if not isinstance(parameter, ContextQuantity) or parameter.type != QuantityType.PathParameter:
        raise semantic_error("Progress must maximize a PathParameter.", entry.parameter)
    return parameter


def _progress_paths(entry) -> list[ContextQuantity]:
    """Raise unless every path `entry` selects resolves to a distinct declared Path."""
    paths: list[ContextQuantity] = []
    for path_ref in entry.path_refs:
        path = context_ref_value(path_ref)
        path = _resolved_context_quantity(path) if isinstance(path, ContextQuantity) else path
        if not isinstance(path, ContextQuantity) or path.type != ReferenceGeneratorType.Path:
            raise semantic_error("Progress must select a Path.", path_ref)
        if path in paths:
            raise semantic_error(
                f"Progress policy '{entry.name}' selects path '{path.name}' more than once.",
                path_ref,
            )
        paths.append(path)
    return paths


def _require_progress_tracking_constraint(motion: GuardedMotion, entry, path: ContextQuantity) -> None:
    """Raise unless an active WHILE equality constraint tracks `path`."""
    governing = []
    for item in motion.while_.constraints:
        constraint = _resolved_spec(item)
        if constraint.disabled or not isinstance(constraint.expr, EqualityConstraint):
            continue
        reference = context_ref_value(constraint.expr.reference)
        reference = (
            _resolved_context_quantity(reference) if isinstance(reference, ContextQuantity) else reference
        )
        if reference is path:
            governing.append(constraint)
    if not governing:
        raise semantic_error(
            f"Progress along '{path.name}' needs a WHILE equality constraint tracking that path or one of its views.",
            entry,
        )


def validate_progress_entries(model: Model) -> None:
    """Validate named progress entries dispatched by kind: a ProgressConstraint authors
    its advancement law, a ProgressObjective asks a compatible solver to maximize the
    parameter. Both need a declared PathParameter, one or more declared Paths, and an
    active WHILE equality tracking each selected path.
    """
    for handler in constraint_handlers(model):
        motion = handler.motion
        names: set[str] = set()
        has_objective = False
        for entry in handler.progress:
            if entry.name in names:
                raise semantic_error(
                    f"Handler '{handler.name}' has duplicate progress policy name "
                    f"'{entry.name}'.",
                    entry,
                )
            names.add(entry.name)

            if isinstance(entry, ProgressObjective):
                # No solver can consume an objective yet, so this always fails today;
                # check it first rather than making the author fix unrelated details
                # (parameter/path/tracking) for an entry that can never pass anyway.
                capable = any(
                    _resolved_solver(solver).algorithm in _OBJECTIVE_CAPABLE_SOLVER_ALGORITHMS
                    for solver in handler.solvers
                )
                if not capable:
                    raise semantic_error(
                        f"Progress objective '{entry.name}' needs a solver in handler "
                        f"'{handler.name}' that can consume a path-parameter objective; "
                        "no available solver exposes that input.",
                        entry,
                    )
                if has_objective:
                    raise semantic_error(
                        f"Handler '{handler.name}' may have at most one progress objective.",
                        entry,
                    )
                has_objective = True

            _progress_parameter(entry)
            paths = _progress_paths(entry)
            for path in paths:
                _require_progress_tracking_constraint(motion, entry, path)

            if isinstance(entry, ProgressObjective):
                continue

            assert isinstance(entry, ProgressConstraint)
            if not math.isfinite(entry.advancement) or entry.advancement <= 0.0:
                raise semantic_error("Progress advancement must be a positive finite rate.", entry)


def validate_unique_constraint_names(model: Model) -> None:
    """Raise if a motion has duplicate constraint names across WHEN/WHILE/UNTIL."""
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


def _decl_motion(obj: object) -> GuardedMotion | None:
    """The GuardedMotion a context declaration belongs to, or None."""
    context = getattr(obj, "parent", None)
    motion = getattr(context, "parent", None)
    return motion if isinstance(motion, GuardedMotion) else None


def _decl_context_spec(obj: object) -> ContextSpec | None:
    """The ContextSpec a context declaration belongs to, or None."""
    context = getattr(obj, "parent", None)
    spec = getattr(context, "parent", None)
    return spec if isinstance(spec, ContextSpec) else None


def _resolved_context_decl(ctx: object) -> object:
    """Dereference a context-decl reference to its target, or return `ctx` unchanged."""
    return ctx.ref if isinstance(ctx, ContextDeclReference) else ctx


def context_ref_value(ref: ContextRef) -> ContextQuantity | None:
    """The ContextQuantity a reference points at (named or inline), or None for a bare literal."""
    if getattr(ref, "bare", None) is not None:
        return None
    value = (
        getattr(ref, "quantity", None)
        or getattr(ref, "value", None)
        or getattr(ref, "inline_quantity", None)
    )
    return value if isinstance(value, ContextQuantity) else None


def _is_inline_context_value(value: ContextQuantity) -> bool:
    """Whether a context quantity was declared inline inside a reference."""
    return (
        isinstance(getattr(value, "parent", None), ContextRef)
        and getattr(
            value.parent,
            "context_scope",
            None,
        )
        is not None
    )


def constraint_context_refs(constraint: ConstraintSpecification) -> list[ContextRef]:
    """The context references a constraint compares against (reference / threshold / bounds)."""
    expr = constraint.expr
    if isinstance(expr, EqualityConstraint):
        refs = [expr.reference]
        if expr.tolerance is not None:
            refs.append(expr.tolerance)
        return refs
    if isinstance(expr, (GreaterThanConstraint, LessThanConstraint)):
        return [expr.threshold]
    if isinstance(expr, BilateralConstraint):
        return [expr.lower, expr.upper]
    return []


def _constraint_view_quantities(constraint: ConstraintSpecification) -> list[object | None]:
    """The world quantities read by a constraint view."""
    view = constraint.view
    if view is None or getattr(view, "is_elapsed", False):
        return []
    if (
        getattr(view, "distance_from", None) is not None
        and getattr(view, "distance_to", None) is not None
    ):
        return [view.distance_from, view.distance_to]
    return [view.quantity]


def validate_constraint_context_refs(model: Model) -> None:
    """Raise if a constraint's view quantities or context references fail to resolve."""
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
                value = (
                    _resolved_context_quantity(value)
                    if isinstance(value, ContextQuantity)
                    else value
                )
                if value is None:
                    raise semantic_error(
                        f"Constraint '{constraint.name}' has an unresolved context reference.",
                        constraint,
                    )
                if (
                    _decl_motion(value) is None
                    and _decl_context_spec(value) is None
                    and _is_inline_context_value(value)
                ):
                    continue
                if _decl_motion(value) is None and _decl_context_spec(value) is None:
                    raise semantic_error(
                        f"Constraint '{constraint.name}' references value '{value.name}', "
                        "but it is not resolved.",
                        ref,
                    )
