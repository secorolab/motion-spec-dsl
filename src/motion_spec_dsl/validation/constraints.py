# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Cross-object constraint and path invariants not expressible in textX or SHACL."""

from __future__ import annotations

from collections import defaultdict
import math

from textx import get_children_of_type

from motion_spec_dsl.classes import (
    AccelerationTwistCoordinate,
    ContextQuantity,
    ContextRef,
    Coordinates,
    DirectionCosineXYZ,
    EqualityConstraint,
    EulerAngles,
    Measure,
    Model,
    PathValue,
    QuantityType,
    Quaternion,
    VectorXYZ,
    VelocityTwistCoordinate,
    WrenchCoordinate,
    _resolved_context_quantity,
    _resolved_spec,
)
from motion_spec_dsl.validation.common import (
    constraint_handlers,
    motion_constraint_items,
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
        _require_direction(
            value.arc.plane_normal, f"Arc '{quantity.name}' plane-normal", value.arc
        )
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


def _progress_paths(entry) -> list[ContextQuantity]:
    paths = []
    seen: set[ContextQuantity] = set()
    for ref in entry.path_refs:
        path = _resolved_ref_quantity(ref)
        if path is None:
            continue
        if path in seen:
            raise semantic_error(
                f"Progress policy '{entry.name}' selects path '{path.name}' more than once.",
                ref,
            )
        seen.add(path)
        paths.append(path)
    return paths


def _require_progress_tracking_constraint(motion, entry, path: ContextQuantity) -> None:
    """Require an active WHILE equality to track each progress path."""
    for item in motion.while_.constraints:
        constraint = _resolved_spec(item)
        if constraint.disabled or not isinstance(constraint.expr, EqualityConstraint):
            continue
        if _resolved_ref_quantity(constraint.expr.reference) is path:
            return
    raise semantic_error(
        f"Progress along '{path.name}' needs a WHILE equality constraint tracking that path.",
        entry,
    )


def validate_progress_structure(model: Model) -> None:
    """Check progress identity and its structural link to active tracking constraints."""
    for handler in constraint_handlers(model):
        names: set[str] = set()
        for entry in handler.progress:
            if entry.name in names:
                raise semantic_error(
                    f"Handler '{handler.name}' has duplicate progress policy name '{entry.name}'.",
                    entry,
                )
            names.add(entry.name)
            for path in _progress_paths(entry):
                _require_progress_tracking_constraint(handler.motion, entry, path)


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
