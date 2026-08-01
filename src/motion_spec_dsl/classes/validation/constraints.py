# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Validate constraint and path invariants not expressible in textX or SHACL."""

from __future__ import annotations

from collections import defaultdict
import math

from textx import get_children_of_type

from motion_spec_dsl.classes.constraints import (
    ConstraintSpecification,
    EqualityConstraint,
    GreaterThanConstraint,
)
from motion_spec_dsl.classes.context import (
    ContextQuantity,
    ContextRef,
    Measure,
    QuantityType,
    ReferenceGeneratorType,
    VectorXYZ,
    _resolved_context_quantity,
)
from motion_spec_dsl.classes.coordinates import (
    AccelerationTwistCoordinate,
    Coordinates,
    DirectionCosineXYZ,
    EulerAngles,
    Quaternion,
    VelocityTwistCoordinate,
    WrenchCoordinate,
)
from motion_spec_dsl.classes.motion_spec import Model
from motion_spec_dsl.classes.path import PathValue
from motion_spec_dsl.classes.validation.common import (
    motion_constraint_items,
    motion_constraints,
    motion_specs,
    semantic_error,
)


def context_ref_value(ref: ContextRef) -> ContextQuantity | None:
    """Return the declared or inline context quantity selected by `ref`."""
    if getattr(ref, "bare", None) is not None:
        return None
    value = getattr(ref, "quantity", None) or getattr(ref, "inline_quantity", None)
    return value if isinstance(value, ContextQuantity) else None


def _resolved_ref_quantity(ref: ContextRef) -> ContextQuantity | None:
    quantity = context_ref_value(ref)
    return _resolved_context_quantity(quantity) if quantity is not None else None


def _static_scalar(ref: ContextRef) -> Measure | None:
    bare = getattr(ref, "bare", None)
    if isinstance(bare, Measure):
        return bare
    quantity = _resolved_ref_quantity(ref)
    scalar = getattr(quantity, "value", None)
    return scalar if isinstance(scalar, Measure) else None


def _literal_xyz(coords: Coordinates) -> tuple[float, float, float] | None:
    """The 3 literal floats of `coords`, or None if it has the wrong arity or any
    element is a reference rather than a literal."""
    if len(coords.values) != 3:
        return None
    if any(element.ref is not None for element in coords.values):
        return None
    return tuple(float(element.value) for element in coords.values)


def _static_vector(ref: ContextRef) -> tuple[float, float, float] | None:
    literal = getattr(ref, "literal_value", None)
    if isinstance(literal, VectorXYZ) and literal.coords is not None:
        return _literal_xyz(literal.coords)
    quantity = _resolved_ref_quantity(ref)
    value = getattr(quantity, "value", None)
    if isinstance(value, VectorXYZ) and value.coords is not None:
        return _literal_xyz(value.coords)
    return None


def _require_direction(ref: ContextRef, label: str, owner: object) -> None:
    """Reject a statically authored direction that is non-finite, zero, or non-unit."""
    vector = _static_vector(ref)
    if vector is None:
        return
    if not all(math.isfinite(value) for value in vector):
        raise semantic_error(f"{label} must be finite.", owner)
    norm = math.sqrt(sum(value * value for value in vector))
    if norm < 1e-6:
        raise semantic_error(f"{label} must be a non-zero vector.", owner)
    if abs(norm - 1.0) > 1e-3:
        raise semantic_error(f"{label} must be a unit-length direction.", owner)


def _validate_path_geometry(quantity: ContextQuantity) -> None:
    """Validate only unit-independent degeneracies of statically authored path inputs."""
    value = quantity.value
    if not isinstance(value, PathValue):
        return
    if value.circle is not None:
        _require_direction(
            value.circle.plane_normal, f"Circle '{quantity.name}' plane-normal", value.circle
        )
    elif value.arc is not None:
        _require_direction(value.arc.plane_normal, f"Arc '{quantity.name}' plane-normal", value.arc)
    elif value.helix is not None:
        _require_direction(value.helix.axis, f"Helix '{quantity.name}' axis", value.helix)
        pitch = _static_scalar(value.helix.pitch)
        if pitch is not None and not math.isfinite(pitch.value):
            raise semantic_error(f"Helix '{quantity.name}' pitch must be finite.", value.helix)
    elif value.figure8 is not None:
        _require_direction(
            value.figure8.plane_normal,
            f"Figure8 '{quantity.name}' plane-normal",
            value.figure8,
        )
        _require_direction(
            value.figure8.direction,
            f"Figure8 '{quantity.name}' direction",
            value.figure8,
        )


def validate_static_path_geometry(model: Model) -> None:
    """Check static path degeneracies across every context, including reusable contexts."""
    checked: set[ContextQuantity] = set()
    for item in get_children_of_type(ContextQuantity, model):
        quantity = _resolved_context_quantity(item)
        if quantity in checked:
            continue
        checked.add(quantity)
        _validate_path_geometry(quantity)


def validate_euler_convention(model: Model) -> None:
    """Reject any Euler orientation block but extrinsic-XYZ, the only convention the backend
    supports, and require exactly 3 angle components."""
    for orientation in get_children_of_type(EulerAngles, model):
        if orientation.axes != "xyz" or not orientation.extrinsic:
            raise semantic_error(
                "Orientation must use 'axes: xyz extrinsic' -- the backend's "
                "KDL::Rotation::RPY constructor only supports extrinsic-XYZ Euler angles.",
                orientation,
            )
        _require_arity(orientation.angles, 3, "Euler 'angles'", orientation)


def _require_arity(coords: Coordinates, expected: int, label: str, owner: object) -> None:
    if len(coords.values) != expected:
        raise semantic_error(
            f"{label} needs exactly {expected} component(s), got {len(coords.values)}.", owner
        )


def validate_quaternion_components(model: Model) -> None:
    """Require exactly 4 xyzw components per quaternion orientation block."""
    for orientation in get_children_of_type(Quaternion, model):
        _require_arity(orientation.xyzw, 4, "Quaternion 'xyzw'", orientation)


def validate_direction_cosine_components(model: Model) -> None:
    """Require exactly 3 components per direction-cosine axis."""
    for orientation in get_children_of_type(DirectionCosineXYZ, model):
        _require_arity(orientation.x_axis, 3, "Direction-cosine 'x'", orientation)
        _require_arity(orientation.y_axis, 3, "Direction-cosine 'y'", orientation)
        _require_arity(orientation.z_axis, 3, "Direction-cosine 'z'", orientation)


def _path_operand(view) -> object | None:
    """The driver, geometry or guard operand of a view that names a path."""
    return (
        getattr(view, "moving", None)
        or getattr(view, "on", None)
        or getattr(view, "progress", None)
    )


def _path_of(view) -> object | None:
    """The context path a path-following view names, if it resolves to one."""
    operand = _path_operand(view)
    quantity = getattr(getattr(operand, "path", None), "quantity", None)
    return _resolved_context_quantity(quantity) if quantity is not None else None


_ON_PATH_SUBSPACES = ("position", "orientation")


def _require_path_reference(spec: ConstraintSpecification, operand) -> None:
    """Require the operand's `path` to name a declared geometric path."""
    path = _path_of(spec.view)
    if path is None or path.type != ReferenceGeneratorType.Path:
        raise semantic_error(
            f"'{spec.name}' follows a path, so its reference must be a declared 'path'.",
            operand,
        )


def _require_speed_reference(spec: ConstraintSpecification, ref: ContextRef, role: str) -> None:
    """Require a speed operand to be a linear-velocity quantity rather than a bare number."""
    quantity = _resolved_ref_quantity(ref)
    if quantity is None or quantity.type != QuantityType.LinearVelocity:
        raise semantic_error(
            f"'{spec.name}' states a {role} along a path, which is a linear velocity.",
            spec,
        )


def validate_path_following(model: Model) -> None:
    """Check the driver/geometry/guard split a path specification rests on.

    A path constrains geometry but not timing, so `moving ... along ... at ...` drives the
    motion and `keeping ... on ...` holds it on the geometry -- neither compares anything, so
    neither carries a relation -- while `progress ... along ...` only guards continuation and
    must. Every other view still compares something, and the grammar can no longer require it.
    """
    for spec in get_children_of_type(ConstraintSpecification, model):
        view = spec.view
        driver = getattr(view, "moving", None)
        geometry = getattr(view, "on", None)
        guard = getattr(view, "progress", None)
        if driver is not None or geometry is not None:
            if spec.expr is not None:
                raise semantic_error(
                    f"'{spec.name}' constrains motion to a path's geometry, so the path is its "
                    "reference; drop the comparison.",
                    spec,
                )
        elif spec.expr is None:
            raise semantic_error(f"Constraint '{spec.name}' is missing its relation.", spec)
        elif guard is not None and not isinstance(spec.expr, GreaterThanConstraint):
            raise semantic_error(
                f"'{spec.name}' guards progress along a path, which is one-sided: use 'more than'.",
                spec,
            )

        operand = driver or geometry or guard
        if operand is None:
            continue
        _require_path_reference(spec, operand)
        if driver is not None:
            _require_speed_reference(spec, driver.speed, "commanded speed")
        elif guard is not None:
            _require_speed_reference(spec, spec.expr.threshold, "minimum speed")
        else:
            subspace = str(getattr(view.subspace, "value", view.subspace or ""))
            if subspace not in _ON_PATH_SUBSPACES or view.axis is not None:
                raise semantic_error(
                    f"'{spec.name}' must hold either '.position' or '.orientation' on the path: "
                    "the two are separate control laws and the tangent belongs to the driver.",
                    spec,
                )

    for motion in motion_specs(model):
        followed = {
            _path_of(spec.view): spec
            for spec in motion_constraints(motion)
            if getattr(spec.view, "moving", None) is not None
            or getattr(spec.view, "on", None) is not None
        }
        followed.pop(None, None)
        if not followed:
            continue
        for spec in motion_constraints(motion):
            if not isinstance(spec.expr, EqualityConstraint):
                continue
            reference = _resolved_ref_quantity(spec.expr.reference)
            if reference in followed:
                raise semantic_error(
                    f"'{spec.name}' pins '{motion.name}' to a setpoint on the same path "
                    f"'{followed[reference].name}' already follows. A path constrains geometry, "
                    "not timing; drop the equality.",
                    spec,
                )


# Types absent here carry no bare Measure/VectorXYZ value (pose, the two-subspace types).
_UNITS_BY_QUANTITY_TYPE: dict[QuantityType, tuple[str, ...]] = {
    QuantityType.Position: ("mm", "cm", "m"),
    QuantityType.Distance: ("mm", "cm", "m"),
    QuantityType.Direction: ("1",),
    QuantityType.FreeVector: ("1",),
    QuantityType.Orientation: ("rad", "deg"),
    QuantityType.Angle: ("rad", "deg"),
    QuantityType.PlaneAngle: ("rad", "deg"),
    QuantityType.LinearVelocity: ("m/s", "cm/s"),
    QuantityType.AngularVelocity: ("rad/s", "deg/s"),
    QuantityType.LinearAcceleration: ("m/s^2",),
    QuantityType.AngularAcceleration: ("rad/s^2", "deg/s^2"),
    QuantityType.LinearJerk: ("m/s^3",),
    QuantityType.Force: ("N",),
    QuantityType.Torque: ("Nm",),
    QuantityType.Duration: ("s", "ms"),
    QuantityType.Dimensionless: ("1",),
    QuantityType.PathParameter: ("1",),
}


def validate_unit_kinds(model: Model) -> None:
    """Reject a unit that does not belong to its quantity's kind. The grammar cannot decide
    this -- the value rule is shared across every quantity type -- so it is checked here,
    where the message can name the quantity and the units it does accept.
    """
    for item in get_children_of_type(ContextQuantity, model):
        quantity = _resolved_context_quantity(item)
        allowed = _UNITS_BY_QUANTITY_TYPE.get(quantity.type)
        value = quantity.value
        if allowed is None or not isinstance(value, (Measure, VectorXYZ)):
            continue
        unit = getattr(value, "unit", None)
        if unit is None or unit in allowed:
            continue
        raise semantic_error(
            f"'{quantity.name}' is a {quantity.type} quantity: '{unit}' is not one of its "
            f"units ({', '.join(allowed)}).",
            quantity,
        )


_TWO_SUBSPACE_TYPE_NAMES = {
    QuantityType.VelocityTwist: "velocity-twist",
    QuantityType.AccelerationTwist: "acceleration-twist",
    QuantityType.Wrench: "wrench",
}


def validate_two_subspace_coordinates(model: Model) -> None:
    """A velocity-twist/acceleration-twist/wrench value has two named subspaces; a bare
    vector literal cannot express which one it is, so reject it, and require each
    subspace's Coordinates to carry exactly 3 components.
    """
    for item in get_children_of_type(ContextQuantity, model):
        quantity = _resolved_context_quantity(item)
        type_name = _TWO_SUBSPACE_TYPE_NAMES.get(quantity.type)
        if type_name is None:
            continue
        if isinstance(quantity.value, VectorXYZ):
            raise semantic_error(
                f"'{quantity.name}' is a {type_name}: a single vector literal cannot say "
                f"which subspace it is. Use the two-subspace form instead.",
                quantity,
            )
        if isinstance(quantity.value, (VelocityTwistCoordinate, AccelerationTwistCoordinate)):
            _require_arity(quantity.value.angular, 3, f"{type_name} 'angular' subspace", quantity)
            _require_arity(quantity.value.linear, 3, f"{type_name} 'linear' subspace", quantity)
        elif isinstance(quantity.value, WrenchCoordinate):
            _require_arity(quantity.value.torque, 3, "wrench 'torque' subspace", quantity)
            _require_arity(quantity.value.force, 3, "wrench 'force' subspace", quantity)


def validate_unique_constraint_names(model: Model) -> None:
    """Require unambiguous local constraint names within each guarded motion."""
    for motion in motion_specs(model):
        by_name: dict[str, list[object]] = defaultdict(list)
        for item in motion_constraint_items(motion):
            by_name[item.name].append(item)
        duplicates = {name: items for name, items in by_name.items() if len(items) > 1}
        if duplicates:
            names = ", ".join(sorted(duplicates))
            raise semantic_error(
                f"Motion '{motion.name}' has duplicate constraint name(s): {names}.",
                next(iter(duplicates.values()))[1],
            )
