# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Validate constraint and path invariants not expressible in textX or SHACL."""

from __future__ import annotations

import math
from collections import defaultdict

from textx import get_children_of_type

from motion_spec_dsl.classes.constraints import (
    BilateralConstraint,
    ConstraintSpecification,
    EqualityConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    OutsideConstraint,
)
from motion_spec_dsl.classes.context import (
    GEOMETRIC_DISTANCE_OPS,
    GEOMETRIC_PROJECTION_OPS,
    UNSIGNED_GEOMETRIC_DISTANCE_OP,
    ContextQuantity,
    ContextRef,
    GeometricPropKey,
    Measure,
    QuantityType,
    ReferenceGeneratorType,
    VectorXYZ,
    WorldQuantityType,
    _geometric_operand_kind,
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
from motion_spec_dsl.classes.motion_spec import Model, ToleranceDefault
from motion_spec_dsl.classes.path import PathValue, ProfileSpec
from motion_spec_dsl.classes.validation.common import (
    motion_constraint_items,
    motion_constraints,
    motion_specs,
    semantic_error,
)
from motion_spec_dsl.rdf.common import _is_distance_view


def context_ref_value(ref: ContextRef) -> ContextQuantity | None:
    """Return the context quantity selected by `ref`."""
    if getattr(ref, "bare", None) is not None:
        return None
    value = getattr(ref, "quantity", None)
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


def validate_static_path_geometry(model: Model) -> None:
    """Check static path degeneracies across every context, including reusable contexts."""
    checked: set[ContextQuantity] = set()
    for item in get_children_of_type(ContextQuantity, model):
        quantity = _resolved_context_quantity(item)
        if quantity in checked:
            continue
        checked.add(quantity)
        _validate_path_geometry(quantity)


def _require_arity(coords: Coordinates, expected: int, label: str, owner: object) -> None:
    if len(coords.values) != expected:
        raise semantic_error(
            f"{label} needs exactly {expected} component(s), got {len(coords.values)}.", owner
        )


def validate_euler_components(model: Model) -> None:
    """Require exactly 3 angle components per Euler orientation block.

    The sequence itself is unconstrained: any of the six composes, intrinsic or extrinsic,
    because the backend builds a rotation as a product of per-axis quaternions in the
    authored order rather than through one fixed constructor.
    """
    for orientation in get_children_of_type(EulerAngles, model):
        _require_arity(orientation.angles, 3, "Euler 'angles'", orientation)


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


def _require_profile_reference(spec: ConstraintSpecification, ref: ContextRef) -> None:
    """Require a path driver to name a velocity profile."""
    quantity = _resolved_ref_quantity(ref)
    if quantity is None or not isinstance(quantity.value, ProfileSpec):
        raise semantic_error(
            f"'{spec.name}' drives a path, so 'with' must name a velocity profile.",
            spec,
        )


def validate_path_following(model: Model) -> None:
    """Check the driver/geometry/guard split a path specification rests on.

    A path constrains geometry but not timing, so `moving ... along ... with ...` drives the
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
            _require_profile_reference(spec, driver.profile)
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


_ORDER_RELATIONS = (
    GreaterThanConstraint,
    LessThanConstraint,
    BilateralConstraint,
    OutsideConstraint,
)
_COMPOSITE_WORLD_TYPES = {
    WorldQuantityType.Pose,
    WorldQuantityType.VelocityTwist,
    WorldQuantityType.Wrench,
}


def validate_scalar_order_relations(model: Model) -> None:
    """Reject order relations (greater/less/between/outside) on multi-dimensional views.

    Nothing physical orders 3-D quantities; such a comparison needs a scalar view — an
    axis component or a `distance between` two poses.
    """
    for motion in motion_specs(model):
        for spec in motion_constraints(motion):
            if not isinstance(spec.expr, _ORDER_RELATIONS):
                continue
            view = spec.view
            if (
                getattr(view, "is_elapsed", False)
                or getattr(view, "distance_from", None) is not None
                or getattr(view, "progress", None) is not None
                or getattr(view, "moving", None) is not None
                or getattr(view, "on", None) is not None
                or getattr(view, "axis", None) is not None
            ):
                continue
            quantity = getattr(view, "quantity", None)
            if (
                getattr(view, "subspace", None) is not None
                or getattr(quantity, "type", None) in _COMPOSITE_WORLD_TYPES
            ):
                raise semantic_error(
                    f"Constraint '{spec.name}': an order relation needs a scalar view; "
                    "select a single axis or author `distance between` two poses.",
                    spec,
                )


def _direction_frame(quantity: ContextQuantity) -> object | None:
    """The as-seen-by (or wrt) frame of a direction context quantity, if stated."""
    if quantity.props is None:
        return None
    for pair in quantity.props.pairs:
        if pair.key in (GeometricPropKey.AsSeenBy, GeometricPropKey.Wrt):
            return pair.value
    return None


def _alignment_operand(
    quantity: ContextQuantity, label: str, owner: object
) -> tuple[float, float, float]:
    """Require `quantity` to be a `direction` with a frame and literal 3-vector; return it."""
    quantity = _resolved_context_quantity(quantity)
    if quantity.type != QuantityType.Direction:
        raise semantic_error(f"{label} must be a 'direction' context quantity.", owner)
    if _direction_frame(quantity) is None:
        raise semantic_error(f"{label} '{quantity.name}' needs an 'as-seen-by' frame.", owner)
    vector = quantity.value
    vector = (
        _literal_xyz(vector.coords) if isinstance(vector, VectorXYZ) and vector.coords else None
    )
    if vector is None:
        raise semantic_error(f"{label} '{quantity.name}' needs a literal 3-vector value.", owner)
    return vector


def _alignment_target(spec) -> None:
    """An alignment either drives the angle to zero or tolerates a cone around it: equality to a
    bare zero, or a band starting at zero. A band starting anywhere else asks the directions to
    hold an angle apart, which is a different constraint than aligning them.
    """
    expr = spec.expr
    if isinstance(expr, EqualityConstraint):
        measure = expr.reference.bare
        if measure is None or measure.value != 0.0:
            raise semantic_error(
                f"'{spec.name}' must align to a bare zero angle -- nonzero alignment targets "
                "are not supported.",
                spec,
            )
        return
    if isinstance(expr, BilateralConstraint):
        lower = expr.lower.bare
        if lower is None or lower.value != 0.0:
            raise semantic_error(
                f"'{spec.name}' must open its band at zero -- a band that starts elsewhere holds "
                "the directions apart rather than aligning them.",
                spec,
            )
        upper = expr.upper.bare
        if upper is not None:
            if upper.value <= 0.0:
                raise semantic_error(
                    f"'{spec.name}' needs a positive upper bound on its band.", spec
                )
            return
        # A declared bound says how wide the tolerated cone is, so it has to be an angle.
        quantity = _resolved_ref_quantity(expr.upper)
        if quantity is None or quantity.type != QuantityType.Angle:
            raise semantic_error(
                f"'{spec.name}' needs its band's upper bound stated as an angle.", spec
            )
        return
    raise semantic_error(
        f"'{spec.name}' aligns two directions, which supports equality to zero or a band "
        "from zero.",
        spec,
    )


def validate_alignment_views(model: Model) -> None:
    """Check `angle between <a> and <b>` views: both operands are authored directions, the
    reference is a signed frame axis, and the target is zero or a band opening at zero.
    """
    for spec in get_children_of_type(ConstraintSpecification, model):
        view = spec.view
        angle_from = getattr(view, "angle_from", None)
        angle_to = getattr(view, "angle_to", None)
        if angle_from is None or angle_to is None:
            continue
        _alignment_operand(angle_from, f"'{spec.name}' first operand", spec)
        reference = _alignment_operand(angle_to, f"'{spec.name}' reference operand", spec)
        nonzero = [v for v in reference if abs(v) > 1e-9]
        if len(nonzero) != 1 or abs(abs(nonzero[0]) - 1.0) > 1e-6:
            raise semantic_error(
                f"'{spec.name}' reference operand must be a signed unit frame axis "
                "(exactly one component, +-1).",
                spec,
            )
        _alignment_target(spec)


_PRIMITIVE_DIRECTION_KEY = {
    QuantityType.Plane: GeometricPropKey.Normal,
    QuantityType.Line: GeometricPropKey.Along,
}


def validate_line_plane_primitives(model: Model) -> None:
    """A line or plane composes a frame origin with a declared unit direction: both roles are
    required, and the direction role must name a `direction` quantity, not an arbitrary one.
    """
    for item in get_children_of_type(ContextQuantity, model):
        quantity = _resolved_context_quantity(item)
        direction_key = _PRIMITIVE_DIRECTION_KEY.get(quantity.type)
        if direction_key is None:
            continue
        keys = [pair.key for pair in quantity.props.pairs]
        allowed = {GeometricPropKey.Of, direction_key}
        for key in keys:
            if key not in allowed:
                raise semantic_error(
                    f"'{quantity.name}' is a {quantity.type} and takes only "
                    f"'of' and '{direction_key}'; '{key}' is not one of them.",
                    quantity,
                )
        for key in sorted(allowed):
            if keys.count(key) != 1:
                raise semantic_error(
                    f"'{quantity.name}' is a {quantity.type} and needs exactly one '{key}'.",
                    quantity,
                )
        referent = next(pair.value for pair in quantity.props.pairs if pair.key == direction_key)
        _alignment_operand(referent, f"'{quantity.name}' {direction_key}", quantity)


def _drives_unsigned_distance_to_zero(spec) -> bool:
    """Whether `spec`'s expression pins an unsigned distance to exactly zero: a bare-zero
    equality, or a band whose bounds are both zero. Its derivative is undefined there.
    """
    expr = spec.expr
    if isinstance(expr, EqualityConstraint):
        measure = expr.reference.bare
        return measure is not None and measure.value == 0.0
    if isinstance(expr, BilateralConstraint):
        lower = expr.lower.bare
        upper = expr.upper.bare
        return lower is not None and upper is not None and lower.value == 0.0 and upper.value == 0.0
    return False


def _reject_unsigned_distance_to_zero(spec) -> None:
    if not _drives_unsigned_distance_to_zero(spec):
        return
    raise semantic_error(
        f"'{spec.name}' drives an unsigned distance to zero, where its derivative is undefined "
        "and control goes locally unstable. State the coincidence as signed projections instead "
        "(Borghesan et al. 2016, section VII-B).",
        spec,
    )


def validate_geometric_distance_views(model: Model) -> None:
    """Check Table IIa `distance of <A> from <B>` / `projection of <A> on <B>` views (plan 08):
    operand typing, the point-first operand order, projection's line-only second operand, and
    the unsigned-zero rule -- which also applies to the pre-existing point-point
    `distance between`, since its scalar is the same kind of unsigned magnitude.
    """
    for spec in get_children_of_type(ConstraintSpecification, model):
        view = spec.view
        if _is_distance_view(spec):
            _reject_unsigned_distance_to_zero(spec)
            continue

        distance_of = getattr(view, "distance_of", None)
        distance_from_primitive = getattr(view, "distance_from_primitive", None)
        if distance_of is not None and distance_from_primitive is not None:
            a_kind = _geometric_operand_kind(distance_of)
            b_kind = _geometric_operand_kind(distance_from_primitive)
            op_type = GEOMETRIC_DISTANCE_OPS.get((a_kind, b_kind))
            if op_type is None:
                if a_kind != b_kind and GEOMETRIC_DISTANCE_OPS.get((b_kind, a_kind)):
                    raise semantic_error(
                        f"'{spec.name}' takes a distance of a pose from a plane, a pose from a "
                        "line, or a line from a line; the point (or line) must come first, since "
                        "the sign is stated in terms of the primitive's normal/direction.",
                        spec,
                    )
                raise semantic_error(
                    f"'{spec.name}' takes a distance of a pose from a plane, a pose from a line, "
                    f"or a line from a line; got {a_kind or 'unknown'} from {b_kind or 'unknown'}.",
                    spec,
                )
            if op_type == UNSIGNED_GEOMETRIC_DISTANCE_OP:
                _reject_unsigned_distance_to_zero(spec)
            continue

        projection_of = getattr(view, "projection_of", None)
        projection_on = getattr(view, "projection_on", None)
        if projection_of is not None and projection_on is not None:
            a_kind = _geometric_operand_kind(projection_of)
            b_kind = _geometric_operand_kind(projection_on)
            if b_kind != "line":
                raise semantic_error(
                    f"'{spec.name}' projects onto a line; got {b_kind or 'unknown'}.", spec
                )
            if a_kind not in ("point", "line"):
                raise semantic_error(
                    f"'{spec.name}' projects a pose or a line onto a line; got "
                    f"{a_kind or 'unknown'}.",
                    spec,
                )


def validate_tolerance_defaults(model: Model) -> None:
    """A model-wide band applies to every constraint of its kind, so it has to be stated in
    that kind's units and stated once. Neither is decidable in the grammar: the value rule is
    shared across kinds, and the entries are a list, which admits a repeated key.
    """
    seen: dict[QuantityType, ToleranceDefault] = {}
    for entry in get_children_of_type(ToleranceDefault, model):
        if entry.kind in seen:
            raise semantic_error(
                f"'{entry.kind}' already has a default band; a kind takes one.", entry
            )
        seen[entry.kind] = entry
        allowed = _UNITS_BY_QUANTITY_TYPE.get(entry.kind)
        unit = getattr(entry.band.bare, "unit", None)
        if allowed is None or unit is None or unit in allowed:
            continue
        raise semantic_error(
            f"a default band for {entry.kind} is not measured in '{unit}' ({', '.join(allowed)}).",
            entry,
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
