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
from motion_spec_dsl.classes.controller_semantics import (
    SUBSPACE_ALIAS,
    _alignment_is_pointwise,
    axis_label,
    constraint_view_subspace,
    resolved_constraint_quantity,
)
from motion_spec_dsl.classes.context import (
    GEOMETRIC_DISTANCE_OPS,
    GEOMETRIC_PROJECTION_OPS,
    UNSIGNED_GEOMETRIC_DISTANCE_OPS,
    ConfigValue,
    ContextQuantity,
    ContextRef,
    DirectionBetween,
    GeometricPropKey,
    GeometricProps,
    GeoPropPair,
    Measure,
    QuantityLeaf,
    QuantityType,
    ReferenceGeneratorType,
    SampledValue,
    VectorXYZ,
    View,
    WorldQuantity,
    WorldQuantityType,
    _geometric_operand_kind,
    _resolved_context_quantity,
    _resolved_world_quantity,
)
from motion_spec_dsl.classes.coordinates import (
    AccelerationTwistCoordinate,
    Coordinates,
    DirectionCosineXYZ,
    EulerAngles,
    PoseCoordinate,
    Quaternion,
    VelocityTwistCoordinate,
    WrenchCoordinate,
)
from motion_spec_dsl.classes.dimensions import VECTOR_COMPONENT_TYPE
from motion_spec_dsl.classes.motion_spec import Model, ToleranceDefault
from motion_spec_dsl.classes.path import PathValue, ProfileSpec
from motion_spec_dsl.classes.units import UNIT_KINDS
from motion_spec_dsl.classes.validation.common import (
    motion_constraint_items,
    motion_constraints,
    motion_specs,
    semantic_error,
)
from motion_spec_dsl.rdf.common import (
    _NORM_SCALAR_TYPES,
    _binary_view,
    _context_quantity,
    _geo_prop,
    _geo_prop_events,
    _is_angle_between_view,
    _is_difference_view,
    _is_distance_view,
    _is_geometric_distance_view,
    _is_projection_view,
    _node_name,
    _path_pose_endpoints,
    _pose_frame_names,
    _quantity_axis_frame,
    _scalar_type,
)
from motion_spec_dsl.rdf.model import QUDT_KIND_BY_QUANTITY_TYPE, _qudt_kind


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


def validate_unit_kinds(model: Model) -> None:
    """Reject a unit that does not belong to its quantity's kind. The grammar cannot decide
    this -- the value rule is shared across every quantity type -- so it is checked here,
    where the message can name the quantity and the units it does accept.
    """
    for item in get_children_of_type(ContextQuantity, model):
        quantity = _resolved_context_quantity(item)
        value = quantity.value
        unit = getattr(value, "unit", None)
        if not isinstance(value, (Measure, VectorXYZ)) or unit is None:
            continue
        kind = _qudt_kind(quantity.type)
        allowed = [token for token, kinds in UNIT_KINDS.items() if kind in kinds]
        if unit in allowed:
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
                or _is_distance_view(spec)
                or getattr(view, "progress", None) is not None
                or getattr(view, "moving", None) is not None
                or getattr(view, "on", None) is not None
                # A norm collapses the 3-vector it reads to one number, so it orders.
                or getattr(view, "norm", None) is not None
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


def _validate_cone_bound(ref: ContextRef, spec, *, positive_required: bool) -> None:
    """A cone-width bound: a bare Measure (positive, when `positive_required`), or a named
    quantity resolving to `QuantityType.Angle`. Shared by the bilateral upper bound and the
    `less than` threshold -- the same cone, stated two ways.
    """
    bare = ref.bare
    if bare is not None:
        if positive_required and bare.value <= 0.0:
            raise semantic_error(f"'{spec.name}' needs a positive upper bound on its band.", spec)
        return
    quantity = _resolved_ref_quantity(ref)
    if quantity is None or quantity.type != QuantityType.Angle:
        raise semantic_error(
            f"'{spec.name}' needs its band's upper bound stated as an angle.", spec
        )


def _alignment_bound_radians(ref: ContextRef, spec, what: str) -> float | None:
    """Radian value of a cone-target bound if statically known; raises unless `ref` resolves to
    an angle. Returns None for a named quantity with no literal value -- its range cannot be
    checked at authoring time, but its type already was.
    """
    measure = _static_scalar(ref)
    if measure is not None:
        if measure.unit not in ("rad", "deg", ""):
            raise semantic_error(f"'{spec.name}' needs {what} stated as an angle.", spec)
        scale = math.pi / 180.0 if measure.unit == "deg" else 1.0
        return measure.value * scale
    quantity = _resolved_ref_quantity(ref)
    if quantity is None or quantity.type != QuantityType.Angle:
        raise semantic_error(f"'{spec.name}' needs {what} stated as an angle.", spec)
    return None


def _require_angle_in_range(ref: ContextRef, spec, what: str) -> None:
    """A cone target's bound must be an angle in [0, pi] -- the angle between two directions
    cannot exceed pi, so anything past it is unsatisfiable and gets rejected at authoring time.
    """
    value = _alignment_bound_radians(ref, spec, what)
    if value is not None and not (0.0 <= value <= math.pi):
        raise semantic_error(
            f"'{spec.name}' targets {value} rad; the angle between two directions lies in [0, pi].",
            spec,
        )


def _cone_alignment_target(spec) -> None:
    """A cone target -- 1 rotational DOF: `equal to <nonzero>`, `greater than`, a band not
    opening at zero, or `outside`. Every bound must be a statically-in-range angle, and a band's
    lower bound must sit below its upper.
    """
    expr = spec.expr
    if isinstance(expr, EqualityConstraint):
        _require_angle_in_range(expr.reference, spec, "its target")
        return
    if isinstance(expr, GreaterThanConstraint):
        _require_angle_in_range(expr.threshold, spec, "its threshold")
        return
    if isinstance(expr, (BilateralConstraint, OutsideConstraint)):
        _require_angle_in_range(expr.lower, spec, "its lower bound")
        _require_angle_in_range(expr.upper, spec, "its upper bound")
        lower_v = _alignment_bound_radians(expr.lower, spec, "its lower bound")
        upper_v = _alignment_bound_radians(expr.upper, spec, "its upper bound")
        if lower_v is not None and upper_v is not None and not lower_v < upper_v:
            raise semantic_error(f"'{spec.name}' needs a lower bound below its upper bound.", spec)
        return
    raise semantic_error(
        f"'{spec.name}' aligns two directions, which supports equality, an inequality, or a "
        "band on the angle between them.",
        spec,
    )


def _alignment_target(spec) -> None:
    """An alignment either drives the angle to a point on the sphere (2 DOF) or tolerates/targets
    a cone around it (1 DOF); see `_alignment_is_pointwise`. The point-target checks are the
    original zero-target rule, plus `less than` validated exactly like the bilateral upper bound
    it is the same cone as. The cone-target checks are new: every bound is a statically-in-range
    angle, and a band's lower sits below its upper.
    """
    expr = spec.expr
    if not _alignment_is_pointwise(spec):
        _cone_alignment_target(spec)
        return
    if isinstance(expr, EqualityConstraint):
        # _alignment_is_pointwise already confirmed a bare zero.
        return
    if isinstance(expr, LessThanConstraint):
        _validate_cone_bound(expr.threshold, spec, positive_required=True)
        return
    # BilateralConstraint with a lower bound of zero.
    upper = expr.upper.bare
    if upper is not None:
        if upper.value <= 0.0:
            raise semantic_error(f"'{spec.name}' needs a positive upper bound on its band.", spec)
        return
    _validate_cone_bound(expr.upper, spec, positive_required=False)


def validate_alignment_views(model: Model) -> None:
    """Check `angle between <a> and <b>` views, dispatched on operand type (Borghesan Table IIb):
    versor-versor keeps the signed-unit-frame-axis rule on its reference operand for a point
    target only, since that is the only case still driving the 2-axis rotation-vector row;
    versor-plane and plane-plane place no frame-axis requirement on their plane operand(s), since
    their rows are always runtime gradients. All three share the point/cone target rule.
    """
    for spec in get_children_of_type(ConstraintSpecification, model):
        if not _is_angle_between_view(spec):
            continue
        binary = _binary_view(spec)
        left, right = binary.left, binary.right
        from_is_plane = _resolved_context_quantity(left).type == QuantityType.Plane
        to_is_plane = _resolved_context_quantity(right).type == QuantityType.Plane
        if from_is_plane and not to_is_plane:
            raise semantic_error(
                f"'{spec.name}' does not support `angle between <plane> and <direction>`; "
                "author it as `angle between <direction> and <plane>`.",
                spec,
            )
        if not to_is_plane:
            _alignment_operand(left, f"'{spec.name}' first operand", spec)
            reference = _alignment_operand(right, f"'{spec.name}' reference operand", spec)
            # The signed-unit-frame-axis requirement only exists for the 2-axis rotation-vector
            # row: a cone target's row is a runtime gradient and cares nothing about the
            # reference's orientation.
            if _alignment_is_pointwise(spec):
                nonzero = [v for v in reference if abs(v) > 1e-9]
                if len(nonzero) != 1 or abs(abs(nonzero[0]) - 1.0) > 1e-6:
                    raise semantic_error(
                        f"'{spec.name}' reference operand must be a signed unit frame axis "
                        "(exactly one component, +-1).",
                        spec,
                    )
        elif not from_is_plane:
            _alignment_operand(left, f"'{spec.name}' first operand", spec)
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
        if _is_distance_view(spec):
            _reject_unsigned_distance_to_zero(spec)
            continue

        if _is_geometric_distance_view(spec):
            binary = _binary_view(spec)
            a_kind = _geometric_operand_kind(binary.left)
            b_kind = _geometric_operand_kind(binary.right)
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
            if op_type in UNSIGNED_GEOMETRIC_DISTANCE_OPS:
                _reject_unsigned_distance_to_zero(spec)
            continue

        if _is_projection_view(spec):
            binary = _binary_view(spec)
            a_kind = _geometric_operand_kind(binary.left)
            b_kind = _geometric_operand_kind(binary.right)
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
        unit = getattr(entry.band.bare, "unit", None)
        if unit is None:
            continue
        kind = _qudt_kind(entry.kind)
        allowed = [token for token, kinds in UNIT_KINDS.items() if kind in kinds]
        if unit in allowed:
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


def validate_bare_axis_selectors(model: Model) -> None:
    """A bare `.x` names one component of a quantity that is itself a 3-vector. A world
    quantity always names its subspace first, and a scalar has no components at all.
    """
    for holder in (View, ContextRef, QuantityLeaf):
        for item in get_children_of_type(holder, model):
            if item.axis is None or item.subspace is not None:
                continue
            quantity = getattr(item, "quantity", None)
            if isinstance(quantity, ContextQuantity):
                resolved = _resolved_context_quantity(quantity)
                if resolved.type in VECTOR_COMPONENT_TYPE:
                    continue
                raise semantic_error(
                    f"'{resolved.name}' is a {resolved.type}, which has no axis components; "
                    f"a bare '.{item.axis}' selects one axis of a 3-vector context quantity "
                    "(direction, free-vector, position, linear-velocity, force, torque).",
                    item,
                )
            name = getattr(quantity, "name", None)
            raise semantic_error(
                f"'{name}' names its subspace before an axis, as in '.linvel.{item.axis}'; "
                f"a bare '.{item.axis}' belongs only on a 3-vector context quantity.",
                item,
            )


def validate_world_quantities(model: Model) -> None:
    """Raise if a world pose names no frames, or a world wrench lacks what its source needs:
    the frames it is read in, and the events that tare it."""
    for quantity in get_children_of_type(WorldQuantity, model):
        quantity = _resolved_world_quantity(quantity)
        if quantity.type == WorldQuantityType.Pose and _pose_frame_names(quantity) is None:
            raise semantic_error(f"Pose '{quantity.uri}' has no frame endpoints", quantity)
        if quantity.type != WorldQuantityType.Wrench:
            continue
        props = quantity.props if isinstance(quantity.props, GeometricProps) else None
        pairs = [pair for pair in (props.pairs if props else []) if isinstance(pair, GeoPropPair)]
        ft_sensor = next(
            (p.sensor for p in pairs if p.key == "ft-sensor" and p.sensor is not None), None
        )
        estimated = any(p.key == "estimated-from" and p.agent is not None for p in pairs)
        if ft_sensor is not None and estimated:
            raise semantic_error(
                f"Wrench '{quantity.name}' names both ft-sensor and estimated-from; "
                "a wrench is measured or estimated, not both.",
                quantity,
            )
        sensor_frame = str(ft_sensor.frame.uri) if ft_sensor is not None else None
        if estimated and _geo_prop(props, "ref-point") is None:
            raise semantic_error(f"Wrench '{quantity.name}' has no ref-point frame", quantity)
        if (_geo_prop(props, "as-seen-by") or sensor_frame) is None:
            raise semantic_error(f"Wrench '{quantity.name}' has no as-seen-by frame", quantity)
        events = _geo_prop_events(props, "re-tare-on")
        tared = ft_sensor is not None or estimated
        if events and not tared:
            raise semantic_error(
                f"Wrench '{quantity.name}' names re-tare-on events but nothing to tare.",
                quantity,
            )
        # There is no implicit "once at startup": the model names its own run-start event.
        if tared and not events:
            source = "an ft-sensor" if ft_sensor is not None else "estimated-from"
            raise semantic_error(
                f"Wrench '{quantity.name}' has {source} but names no re-tare-on events; "
                "name the model's run-start event to tare once at startup.",
                quantity,
            )
        for event in events:
            if event.event is None:
                raise semantic_error(
                    f"Wrench '{quantity.name}' re-tare-on '{event.name}' carries no namespace, "
                    "so it resolves to no declared event; write it as ns.EVENT.",
                    quantity,
                )


def validate_context_geometry(model: Model) -> None:
    """Raise if a declared pose, wrench, orientation, direction or path cannot be placed:
    no frames to state it in, components that disagree on them, or a rotation the backend
    cannot compose."""
    for quantity in get_children_of_type(ContextQuantity, model):
        quantity = _resolved_context_quantity(quantity)
        value = quantity.value
        if quantity.type == QuantityType.Pose and not isinstance(
            value, (PoseCoordinate, ConfigValue, SampledValue)
        ):
            if _pose_frame_names(quantity) is None:
                raise semantic_error(f"Pose '{quantity.uri}' has no frame endpoints", quantity)
        if quantity.type == QuantityType.Wrench and not isinstance(value, SampledValue):
            props = quantity.props if isinstance(quantity.props, GeometricProps) else None
            source = getattr(getattr(value, "source", None), "quantity", None)
            if props is None and isinstance(source, WorldQuantity):
                props = source.props if isinstance(source.props, GeometricProps) else None
            if _geo_prop(props, "as-seen-by") is None:
                raise semantic_error(f"Wrench '{quantity.name}' has no as-seen-by frame", quantity)
        if isinstance(value, PoseCoordinate):
            _validate_pose_value(quantity)
        if quantity.type == QuantityType.Direction:
            _validate_direction_value(quantity)
        if quantity.type == ReferenceGeneratorType.Path:
            _validate_path_endpoints(quantity)


def _validate_pose_value(quantity: ContextQuantity) -> None:
    """A literal pose: components agreeing on frames, and literal rotations the backend takes."""
    value = quantity.value
    if _pose_frame_names(quantity) is None:
        sources = {
            source
            for source in (
                _context_quantity(value.position.ref),
                _context_quantity(value.orientation.ref),
            )
            if source is not None
        }
        frames = {names for source in sources if (names := _pose_frame_names(source))}
        if len(frames) > 1:
            raise semantic_error(
                f"Pose '{quantity.uri}' component references disagree on frame endpoints",
                quantity,
            )
    orientation = value.orientation
    symbolic = f"Orientation coordinate '{quantity.uri}.orientation' cannot use symbolic components"
    coords = (
        orientation.quat.xyzw
        if orientation.quat is not None
        else orientation.euler.angles
        if orientation.euler is not None
        else None
    )
    if (
        coords is not None
        and any(element.ref is not None for element in coords.values)
        and (orientation.quat is not None or len(set(orientation.euler.axes)) != 3)
    ):
        raise semantic_error(symbolic, quantity)
    cosine = orientation.direction_cosine
    if cosine is not None and any(
        element.ref is not None
        for axis in (cosine.x_axis, cosine.y_axis, cosine.z_axis)
        for element in axis.values
    ):
        raise semantic_error(symbolic, quantity)

    relative = orientation.relative
    if relative is None:
        return
    base = _context_quantity(relative.base)
    if not isinstance(base, ContextQuantity):
        raise semantic_error(
            f"Relative orientation on '{quantity.uri}' needs a resolvable base pose to "
            "derive its composition frames.",
            quantity,
        )
    frames = _pose_frame_names(base)
    if frames is None:
        raise semantic_error(
            f"Relative orientation on '{quantity.uri}' cannot resolve its base pose's "
            "of/with-respect-to/as-seen-by frames.",
            quantity,
        )
    of_frame, _wrt_frame, base_as_seen_by = frames
    if relative.frame is not None:
        basis = str(getattr(relative.frame, "uri", relative.frame))
    elif relative.euler is None:
        raise semantic_error(
            f"Relative orientation on '{quantity.uri}' needs an explicit basis frame for "
            "a quaternion or direction-cosine delta.",
            quantity,
        )
    else:
        basis = base_as_seen_by if relative.euler.extrinsic else of_frame
    if basis not in (of_frame, base_as_seen_by):
        raise semantic_error(
            f"Relative orientation on '{quantity.uri}' turns its delta in '{basis}', "
            f"which is neither the base's body frame '{of_frame}' nor its coordinate "
            f"basis '{base_as_seen_by}'. Composing it needs a change of basis, which is "
            "not supported.",
            quantity,
        )
    cosine = relative.direction_cosine
    if cosine is not None:
        if any(
            element.ref is not None
            for axis in (cosine.x_axis, cosine.y_axis, cosine.z_axis)
            for element in axis.values
        ):
            raise semantic_error(
                f"Relative orientation '{quantity.uri}' has a symbolic direction cosine",
                quantity,
            )
        return
    coords = relative.quat.xyzw if relative.quat is not None else relative.euler.angles
    if any(element.ref is not None for element in coords.values):
        raise semantic_error(
            f"Relative orientation '{quantity.uri}' has symbolic components", quantity
        )


def _validate_direction_value(quantity: ContextQuantity) -> None:
    """A direction: the frame it is seen in, and a literal unit vector or a pose to compute from."""
    if (_geo_prop(quantity.props, "as-seen-by") or _geo_prop(quantity.props, "wrt")) is None:
        raise semantic_error(
            f"Direction quantity '{quantity.name}' needs an 'as-seen-by: <frame>' prop.", quantity
        )
    value = quantity.value
    if isinstance(value, VectorXYZ):
        elements = value.coords.values
        if len(elements) != 3 or any(element.ref is not None for element in elements):
            raise semantic_error(
                f"Direction quantity '{quantity.name}' value must be 3 literal components.",
                quantity,
            )
    elif value is not None and not isinstance(value, DirectionBetween):
        raise semantic_error(
            f"Direction quantity '{quantity.name}' value must be a Vector literal.", quantity
        )


def _validate_path_endpoints(quantity: ContextQuantity) -> None:
    """A path's pose endpoints: one kind for a lerp, one frame tuple when the path has none."""
    value = quantity.value
    lerp = getattr(value, "lerp", None)
    if lerp is not None:
        start = _context_quantity(lerp.start)
        goal = _context_quantity(lerp.goal)
        start_kind = QUDT_KIND_BY_QUANTITY_TYPE.get(getattr(start, "type", None))
        goal_kind = QUDT_KIND_BY_QUANTITY_TYPE.get(getattr(goal, "type", None))
        if start_kind != goal_kind:
            raise semantic_error(
                f"Lerp path start type '{getattr(start, 'type', None)}' "
                f"does not match goal type '{getattr(goal, 'type', None)}'",
                quantity,
            )
        if getattr(start, "type", None) != QuantityType.Pose:
            return
    if _pose_frame_names(quantity) is not None:
        return
    endpoint_frames = [
        (endpoint, frames)
        for endpoint in _path_pose_endpoints(quantity)
        if (frames := _pose_frame_names(endpoint)) is not None
    ]
    if not endpoint_frames:
        raise semantic_error(
            f"Path '{quantity.name}' has no endpoint with frame metadata", quantity
        )
    if len({frames for _, frames in endpoint_frames}) != 1:
        details = ", ".join(
            f"{endpoint.name}={tuple(map(str, frames))}" for endpoint, frames in endpoint_frames
        )
        raise semantic_error(
            f"Path '{quantity.name}' endpoint frames disagree: {details}", quantity
        )


def validate_view_operands(model: Model) -> None:
    """Raise if a constraint's view names operands its kind cannot be read from: a world
    quantity with no subspace, a norm of no 3-vector, a difference or distance of the wrong
    kind of operand, Table IIa primitives in frames that do not meet, or a timing view with
    no clock to compare."""
    defaults = {entry.kind for entry in get_children_of_type(ToleranceDefault, model)}
    for spec in get_children_of_type(ConstraintSpecification, model):
        if spec.disabled:
            continue
        view = spec.view
        binary = _binary_view(spec)
        if getattr(view, "is_elapsed", False):
            _validate_timing(spec, QuantityType.Duration in defaults)
            continue
        quantity = resolved_constraint_quantity(spec)
        if binary is None and quantity is not None:
            subspace = constraint_view_subspace(spec)
            if subspace is None:
                raise semantic_error(f"Constraint '{spec.name}' must define a view subspace.", spec)
            if (
                quantity.type == WorldQuantityType.Pose
                and subspace == "distance"
                and axis_label(view.axis) is None
            ):
                raise semantic_error(
                    f"Constraint '{spec.name}' must use explicit "
                    "'distance between <pose-a> and <pose-b>' syntax.",
                    spec,
                )
        if _is_difference_view(spec):
            for ref in (binary.left, binary.right):
                resolved = (
                    _resolved_context_quantity(ref) if isinstance(ref, ContextQuantity) else ref
                )
                derived = type(getattr(resolved, "value", None)).__name__ == "DerivedScalarValue"
                if not derived and not isinstance(ref, WorldQuantity):
                    raise semantic_error(
                        f"Constraint '{spec.name}' takes a difference of '{_node_name(ref)}', "
                        "which is neither a declared quantity nor a scalar defined by a view.",
                        spec,
                    )
        if _is_distance_view(spec):
            for ref in (binary.left, binary.right):
                _validate_distance_endpoint(ref, f"Distance constraint '{spec.name}'", spec)
        if _is_geometric_distance_view(spec) or _is_projection_view(spec):
            _validate_table_iia_operands(spec)
    for view in get_children_of_type(View, model):
        if view.norm is not None:
            _validate_norm_view(view)


def _validate_timing(spec: ConstraintSpecification, duration_default: bool) -> None:
    """An elapsed view compares the clock one-sidedly or within a band, against durations."""
    expr = spec.expr
    if isinstance(expr, (GreaterThanConstraint, LessThanConstraint)):
        refs = [expr.threshold]
    elif isinstance(expr, EqualityConstraint):
        if spec.tolerance is None and not duration_default:
            raise semantic_error(
                f"Elapsed equality '{spec.name}' states no band. A sampled clock never lands "
                "exactly on the reference, so it must say how close counts: "
                "'... equal to <t> within <band>'.",
                spec,
            )
        refs = [expr.reference, spec.tolerance]
    else:
        raise semantic_error(
            f"Timing constraint '{spec.name}' must use 'greater than', 'less than', or 'equal to'.",
            spec,
        )
    for ref in refs:
        if ref is None or getattr(ref, "bare", None) is not None:
            continue
        if not isinstance(_context_quantity(ref), ContextQuantity):
            raise semantic_error(
                "Timing threshold must be a declared Duration quantity or an inline literal "
                "like `5.0 s`.",
                spec,
            )


def _validate_distance_endpoint(ref, context: str, spec: ConstraintSpecification) -> None:
    """A `distance between` endpoint is a pose whose of/wrt frames are stated."""
    if isinstance(ref, WorldQuantity):
        quantity = _resolved_world_quantity(ref)
        if quantity.type != WorldQuantityType.Pose or not isinstance(
            quantity.props, GeometricProps
        ):
            raise semantic_error(f"{context} needs Pose quantities with explicit endpoints.", spec)
    elif getattr(ref, "type", None) != QuantityType.Pose:
        raise semantic_error(f"{context} must reference Pose quantities.", spec)
    if _pose_frame_names(ref) is None:
        raise semantic_error(f"{context} needs explicit 'of' and 'wrt' frames.", spec)


def _validate_table_iia_operands(spec: ConstraintSpecification) -> None:
    """Table IIa operands meet in one frame: both lines' directions, or the point's reference
    frame and the primitive's direction."""
    binary = _binary_view(spec)
    table = (
        GEOMETRIC_DISTANCE_OPS if _is_geometric_distance_view(spec) else GEOMETRIC_PROJECTION_OPS
    )
    op_type = table.get(
        (_geometric_operand_kind(binary.left), _geometric_operand_kind(binary.right))
    )
    context = f"Constraint '{spec.name}'"
    # The frame each primitive's own direction is seen in: a line's `along`, a plane's `normal`.
    seen_by = []
    for ref in (binary.left, binary.right):
        primitive = _resolved_context_quantity(ref) if isinstance(ref, ContextQuantity) else None
        if primitive is None or primitive.type not in (QuantityType.Line, QuantityType.Plane):
            seen_by.append(None)
            continue
        key = (
            GeometricPropKey.Normal
            if primitive.type == QuantityType.Plane
            else GeometricPropKey.Along
        )
        referent = next(pair.value for pair in primitive.props.pairs if pair.key == key)
        seen_by.append(_geo_prop(_resolved_context_quantity(referent).props, "as-seen-by"))

    if op_type in ("LineLineToLinearDistance", "LineOnLineProjection"):
        if seen_by[0] is None or seen_by[1] != seen_by[0]:
            raise semantic_error(
                f"{context} needs both lines' directions stated 'as-seen-by' the same frame.",
                spec,
            )
        return
    point = binary.left
    point = _resolved_world_quantity(point) if isinstance(point, WorldQuantity) else point
    frames = _pose_frame_names(point)
    if frames is None:
        raise semantic_error(f"{context} needs an explicit-frame pose operand.", spec)
    primitive = _resolved_context_quantity(binary.right)
    if seen_by[1] != frames[1]:
        role = "normal" if primitive.type == QuantityType.Plane else "direction"
        raise semantic_error(
            f"{context} needs '{primitive.name}' {role} stated 'as-seen-by' "
            f"'{frames[1]}', the point operand's own reference frame.",
            spec,
        )


def _validate_norm_view(view: View) -> None:
    """A norm reads a whole 3-vector, across a direction seen in the vector's own frame."""
    quantity = view.quantity
    if not isinstance(quantity, WorldQuantity) or view.subspace is None or view.axis is not None:
        raise semantic_error(f"norm of '{_node_name(quantity)}' is not a 3-vector view", view)
    quantity = _resolved_world_quantity(quantity)
    raw = str(getattr(view.subspace, "value", view.subspace))
    mapped = (
        raw
        if quantity.type == WorldQuantityType.Pose and raw == "position"
        else SUBSPACE_ALIAS.get(raw, raw)
    )
    if _scalar_type(quantity, mapped, None) not in _NORM_SCALAR_TYPES:
        raise semantic_error(f"norm of '{quantity.name}.{mapped}' is not a 3-vector view", view)
    across = view.norm.across
    if across is None:
        return
    direction = _context_quantity(across)
    direction = _resolved_context_quantity(direction) if direction is not None else None
    if direction is None or direction.type != QuantityType.Direction:
        raise semantic_error(
            f"norm of '{quantity.name}.{raw}' across "
            f"'{getattr(direction, 'name', across)}': not a direction",
            view,
        )
    vector_frame = _quantity_axis_frame(quantity)
    direction_frame = _geo_prop(direction.props, "as-seen-by")
    if vector_frame != direction_frame:
        raise semantic_error(
            f"norm of '{quantity.name}.{raw}' across '{direction.name}': direction is "
            f"seen by '{direction_frame}', the vector by '{vector_frame}'",
            view,
        )


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
