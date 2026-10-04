# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Validate constraint, view and path invariants the grammar and SHACL cannot state."""

from __future__ import annotations

import math

from textx import get_children_of_type, get_location
from textx.exceptions import TextXSemanticError
from textx.scoping import get_included_models

from motion_spec_dsl.classes.constraint_handler import (
    ConstraintHandler,
    motion_context_quantities,
    motion_world_quantities,
    perturbation_conditions,
    resolved_constraint_items,
)
from motion_spec_dsl.classes.constraints import (
    ANGLE_VIEW_FORMS,
    BilateralConstraint,
    ConstraintSpecification,
    EqualityConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    OutsideConstraint,
    ViewForm,
    flatten_constraint_items,
    view_form,
)
from motion_spec_dsl.classes.context import (
    GEOMETRIC_DISTANCE_OPS,
    GEOMETRIC_PROJECTION_OPS,
    NORM_SCALAR_TYPES,
    UNSIGNED_GEOMETRIC_DISTANCE_OPS,
    AngleBetweenView,
    ConfigValue,
    ContextQuantity,
    ContextQuantityAlias,
    ContextRef,
    DirectionBetween,
    GeometricPropKey,
    GeometricProps,
    Measure,
    QuantityLeaf,
    QuantityType,
    ReferenceGeneratorType,
    SampledValue,
    SnapshotValue,
    VectorXYZ,
    View,
    WorldQuantity,
    WorldQuantityAlias,
    WorldQuantityType,
    geo_prop,
    geo_prop_events,
    geo_prop_value,
    geometric_operand_kind,
    op_tree,
    path_pose_endpoints,
    pose_frame_names,
    quantity_axis_frame,
    scalar_type,
)
from motion_spec_dsl.classes.controller_semantics import (
    SUBSPACE_ALIAS,
    alignment_is_pointwise,
    constraint_view_subspace,
    resolved_constraint_quantity,
)
from motion_spec_dsl.classes.coordinates import (
    AccelerationTwistCoordinate,
    DirectionCosineXYZ,
    EulerAngles,
    PoseCoordinate,
    Quaternion,
    VelocityTwistCoordinate,
    WrenchCoordinate,
)
from motion_spec_dsl.classes.dimensions import DIMENSION_VECTOR, VECTOR_COMPONENT_TYPE, infer, same_scalar_dimension
from motion_spec_dsl.classes.motion_spec import GuardedMotion, Model, ToleranceDefault
from motion_spec_dsl.classes.path import AdmittanceSpec, PathValue, ProfileSpec
from motion_spec_dsl.classes.units import DSL_UNITS, QUDT_KIND_BY_QUANTITY_TYPE
from motion_spec_dsl.classes.validation.common import motion_constraints
from motion_spec_dsl.classes.views import (
    TOLERANCE_DEFAULT_KIND,
    constraint_kind,
    pose_frame_source,
)

_TWO_SUBSPACE_TYPE_NAMES = {
    QuantityType.VelocityTwist: "velocity-twist",
    QuantityType.AccelerationTwist: "acceleration-twist",
    QuantityType.Wrench: "wrench",
}
_ORDER_RELATIONS = (GreaterThanConstraint, LessThanConstraint, BilateralConstraint, OutsideConstraint)
_COMPOSITE_WORLD_TYPES = {WorldQuantityType.Pose, WorldQuantityType.VelocityTwist, WorldQuantityType.Wrench}
_PRIMITIVE_DIRECTION_KEY = {
    QuantityType.Plane: GeometricPropKey.Normal,
    QuantityType.Line: GeometricPropKey.Along,
}
_LINE_LINE_OPS = ("LineLineToLinearDistance", "LineOnLineProjection")


def _literal_xyz(coords) -> tuple[float, float, float] | None:
    """The three literal numbers of COORDS, or None for another arity or a reference."""
    if len(coords.values) != 3 or any(element.ref is not None for element in coords.values):
        return None
    return tuple(float(element.value) for element in coords.values)


def validate_motion_sections(model: Model) -> None:
    """A motion states each of when, while and until once, and holds at least one while constraint."""
    for motion in get_children_of_type(GuardedMotion, model):
        kinds = [section.kind for section in motion.sections]
        for kind in ("when", "while", "until"):
            if kinds.count(kind) != 1:
                raise TextXSemanticError(
                    f"motion '{motion.name}' states {kinds.count(kind)} '{kind}' sections -- a "
                    "motion states each of when, while and until once",
                    **get_location(motion),
                )
        if not motion.while_.constraints:
            raise TextXSemanticError(
                f"motion '{motion.name}' holds no while constraint -- a motion is held by at "
                "least one",
                **get_location(motion),
            )


def validate_unique_constraint_names(model: Model) -> None:
    """Local constraint names are unambiguous within a motion."""
    for motion in get_children_of_type(GuardedMotion, model):
        items = flatten_constraint_items(
            [item for section in motion.sections for item in section.constraints]
        )
        seen = {}
        for item in items:
            if item.name in seen:
                raise TextXSemanticError(
                    f"motion '{motion.name}' names two constraints '{item.name}'",
                    **get_location(item),
                )
            seen[item.name] = item


def validate_coordinate_components(model: Model) -> None:
    """Every orientation, twist and wrench literal has the components its form takes."""
    # (coordinates, components expected, what they are, object reported on)
    arities = []
    for euler in get_children_of_type(EulerAngles, model):
        arities.append((euler.angles, 3, "Euler 'angles'", euler))
    for quaternion in get_children_of_type(Quaternion, model):
        arities.append((quaternion.xyzw, 4, "quaternion 'xyzw'", quaternion))
    for cosine in get_children_of_type(DirectionCosineXYZ, model):
        arities += [
            (cosine.x_axis, 3, "direction-cosine 'x'", cosine),
            (cosine.y_axis, 3, "direction-cosine 'y'", cosine),
            (cosine.z_axis, 3, "direction-cosine 'z'", cosine),
        ]
    for quantity in get_children_of_type(ContextQuantity, model):
        if isinstance(quantity, ContextQuantityAlias) or quantity.type not in _TWO_SUBSPACE_TYPE_NAMES:
            continue
        type_name = _TWO_SUBSPACE_TYPE_NAMES[quantity.type]
        value = quantity.value
        if isinstance(value, VectorXYZ):
            raise TextXSemanticError(
                f"'{quantity.name}' is a {type_name} stated as one vector -- a single literal "
                "cannot say which subspace it is; use the two-subspace form",
                **get_location(quantity),
            )
        if isinstance(value, (VelocityTwistCoordinate, AccelerationTwistCoordinate)):
            arities += [
                (value.angular, 3, f"{type_name} 'angular' subspace", quantity),
                (value.linear, 3, f"{type_name} 'linear' subspace", quantity),
            ]
        elif isinstance(value, WrenchCoordinate):
            arities += [
                (value.torque, 3, "wrench 'torque' subspace", quantity),
                (value.force, 3, "wrench 'force' subspace", quantity),
            ]
    for coords, expected, label, owner in arities:
        if len(coords.values) != expected:
            raise TextXSemanticError(
                f"{label} has {len(coords.values)} components -- it takes {expected}",
                **get_location(owner),
            )


def validate_static_path_geometry(model: Model) -> None:
    """A path's literal normals and axes are finite unit vectors, and a helix pitch is finite."""
    # (direction ref, what it is, object reported on)
    directions = []
    for quantity in get_children_of_type(ContextQuantity, model):
        value = quantity.value
        if isinstance(quantity, ContextQuantityAlias) or not isinstance(value, PathValue):
            continue
        if value.circle is not None:
            directions.append((value.circle.plane_normal, f"circle '{quantity.name}' plane-normal", value.circle))
        elif value.arc is not None:
            directions.append((value.arc.plane_normal, f"arc '{quantity.name}' plane-normal", value.arc))
        elif value.figure8 is not None:
            directions.append(
                (value.figure8.plane_normal, f"figure8 '{quantity.name}' plane-normal", value.figure8)
            )
        elif value.helix is not None:
            directions.append((value.helix.axis, f"helix '{quantity.name}' axis", value.helix))
            pitch = value.helix.pitch.bare
            pitch_quantity = value.helix.pitch.quantity
            if isinstance(pitch_quantity, ContextQuantityAlias):
                pitch_quantity = pitch_quantity.ref
            if pitch is None and isinstance(pitch_quantity, ContextQuantity):
                pitch = pitch_quantity.value
            if isinstance(pitch, Measure) and not math.isfinite(pitch.value):
                raise TextXSemanticError(
                    f"helix '{quantity.name}' pitch is not finite", **get_location(value.helix)
                )
    for ref, label, owner in directions:
        quantity = ref.quantity
        if isinstance(quantity, ContextQuantityAlias):
            quantity = quantity.ref
        if not isinstance(quantity, ContextQuantity) or not isinstance(quantity.value, VectorXYZ):
            continue
        vector = _literal_xyz(quantity.value.coords)
        if vector is None:
            continue
        norm = math.sqrt(sum(component * component for component in vector))
        if not all(math.isfinite(component) for component in vector):
            problem = "is not finite"
        elif norm < 1e-6:
            problem = "is a zero vector"
        elif abs(norm - 1.0) > 1e-3:
            problem = "is not of unit length"
        else:
            continue
        raise TextXSemanticError(f"{label} {problem} -- it is a direction", **get_location(owner))


def validate_path_following(model: Model) -> None:
    """A path driver and a geometry hold compare nothing; a progress guard is one-sided.

    A path constrains geometry, not timing: `moving ... along ... with` drives the motion and
    `keeping ... on` holds it on the geometry, while `progress ... along` only guards continuation.
    """
    for spec in get_children_of_type(ConstraintSpecification, model):
        view = spec.view
        location = get_location(spec)
        driver, geometry, guard = view.moving, view.on, view.progress
        if driver is not None or geometry is not None:
            if spec.expr is not None:
                raise TextXSemanticError(
                    f"'{spec.name}' holds a path's geometry, which is its reference -- drop the "
                    "comparison",
                    **location,
                )
        elif spec.expr is None:
            raise TextXSemanticError(f"constraint '{spec.name}' states no relation", **location)
        elif guard is not None and not isinstance(spec.expr, GreaterThanConstraint):
            raise TextXSemanticError(
                f"'{spec.name}' guards progress along a path -- a guard is one-sided, use 'more "
                "than'",
                **location,
            )
        operand = driver or geometry or guard
        if operand is None:
            continue
        path = operand.path.quantity
        if isinstance(path, ContextQuantityAlias):
            path = path.ref
        if not isinstance(path, ContextQuantity) or path.type != ReferenceGeneratorType.Path:
            raise TextXSemanticError(
                f"'{spec.name}' follows a path -- its reference must be a declared 'path'",
                **get_location(operand),
            )
        if driver is not None:
            profile = driver.profile.quantity
            if isinstance(profile, ContextQuantityAlias):
                profile = profile.ref
            if not isinstance(profile, ContextQuantity) or not isinstance(profile.value, ProfileSpec):
                raise TextXSemanticError(
                    f"'{spec.name}' drives along a path -- 'with' must name a velocity profile",
                    **location,
                )
        elif guard is not None:
            speed = spec.expr.threshold.quantity
            if isinstance(speed, ContextQuantityAlias):
                speed = speed.ref
            if not isinstance(speed, ContextQuantity) or speed.type != QuantityType.LinearVelocity:
                raise TextXSemanticError(
                    f"'{spec.name}' states a minimum speed along a path that is no linear velocity",
                    **location,
                )
        elif str(view.subspace) not in ("position", "orientation") or view.axis is not None:
            raise TextXSemanticError(
                f"'{spec.name}' holds neither '.position' nor '.orientation' on the path -- they "
                "are separate control laws, and the tangent belongs to the driver",
                **location,
            )
    for motion in get_children_of_type(GuardedMotion, model):
        specs = [spec for spec in motion_constraints(motion) if isinstance(spec, ConstraintSpecification)]
        followed = {}
        for spec in specs:
            operand = spec.view.moving or spec.view.on
            if operand is not None:
                path = operand.path.quantity
                followed[path.ref if isinstance(path, ContextQuantityAlias) else path] = spec
        for spec in specs:
            if not followed or not isinstance(spec.expr, EqualityConstraint):
                continue
            reference = spec.expr.reference.quantity
            if isinstance(reference, ContextQuantityAlias):
                reference = reference.ref
            if reference is not None and reference in followed:
                raise TextXSemanticError(
                    f"'{spec.name}' pins '{motion.name}' to a setpoint on path "
                    f"'{followed[reference].name}' follows -- a path constrains geometry, not "
                    "timing; drop the equality",
                    **get_location(spec),
                )


def validate_unit_kinds(model: Model) -> None:
    """A value and a default band are measured in units of their kind; a kind has one default."""
    # (unit token, kind, quantity name, object reported on)
    authored = []
    for quantity in get_children_of_type(ContextQuantity, model):
        value = quantity.value
        if isinstance(quantity, ContextQuantityAlias) or not isinstance(value, (Measure, VectorXYZ)):
            continue
        if value.unit:
            authored.append((value.unit, quantity.type, f"'{quantity.name}'", quantity))
    defaults = set()
    for entry in get_children_of_type(ToleranceDefault, model):
        if entry.kind in defaults:
            raise TextXSemanticError(
                f"'{entry.kind}' has two default bands -- a kind takes one", **get_location(entry)
            )
        defaults.add(entry.kind)
        if entry.band.bare is not None:
            authored.append((entry.band.bare.unit, entry.kind, "the default band", entry))
    for unit, quantity_type, what, owner in authored:
        kind = QUDT_KIND_BY_QUANTITY_TYPE.get(quantity_type)
        allowed = [token for token, dsl_unit in DSL_UNITS.items() if kind in dsl_unit.kinds]
        if unit not in allowed:
            raise TextXSemanticError(
                f"{what} of a {quantity_type} is measured in '{unit}' -- its units are "
                f"{', '.join(allowed)}",
                **get_location(owner),
            )


def validate_admittance(model: Model) -> None:
    """An admittance releases at or below the force that engages it."""
    for admittance in get_children_of_type(AdmittanceSpec, model):
        if admittance.release_threshold > admittance.deadband:
            raise TextXSemanticError(
                f"admittance release-threshold {admittance.release_threshold} must not exceed "
                f"deadband {admittance.deadband} -- the yield would never release",
                **get_location(admittance),
            )


def validate_scalar_order_relations(model: Model) -> None:
    """An order relation compares one number: an axis, a distance, a norm, or a path or clock view."""
    for motion in get_children_of_type(GuardedMotion, model):
        for spec in motion_constraints(motion):
            if not isinstance(spec, ConstraintSpecification) or not isinstance(spec.expr, _ORDER_RELATIONS):
                continue
            view = spec.view
            # A norm collapses the 3-vector it reads to one number.
            if (
                view.elapsed is not None
                or view_form(spec) == ViewForm.DistanceBetween
                or view.progress is not None
                or view.moving is not None
                or view.on is not None
                or view.norm is not None
                or view.axis is not None
            ):
                continue
            # A context pose, twist or wrench is as multi-dimensional as a world one.
            composite = view.quantity is not None and view.quantity.type in _COMPOSITE_WORLD_TYPES
            if view.subspace is not None or composite:
                raise TextXSemanticError(
                    f"constraint '{spec.name}' orders a multi-dimensional view -- select one axis "
                    "or author `distance between` two poses",
                    **get_location(spec),
                )


def validate_direction_operands(model: Model) -> None:
    """Angle operands and line or plane directions are framed literal 3-vector directions."""
    # (direction quantity, what it is, object reported on)
    operands = []
    for spec in get_children_of_type(ConstraintSpecification, model):
        binary = spec.view.binary
        if not isinstance(binary, AngleBetweenView):
            continue
        location = get_location(spec)
        form = view_form(spec)
        if form is None:
            raise TextXSemanticError(
                f"'{spec.name}' takes the angle from a plane to a direction -- author it as "
                "`angle between <direction> and <plane>`",
                **location,
            )
        if form != ViewForm.PlaneAngle:
            operands.append((binary.left, f"'{spec.name}' first operand", spec))
        if form == ViewForm.Alignment:
            operands.append((binary.right, f"'{spec.name}' reference operand", spec))
    for quantity in get_children_of_type(ContextQuantity, model):
        if isinstance(quantity, ContextQuantityAlias) or quantity.type not in _PRIMITIVE_DIRECTION_KEY:
            continue
        direction_key = _PRIMITIVE_DIRECTION_KEY[quantity.type]
        keys = [pair.key for pair in quantity.props.pairs]
        for key in keys:
            if key not in (GeometricPropKey.Of, direction_key):
                raise TextXSemanticError(
                    f"'{quantity.name}' is a {quantity.type} -- it takes 'of' and "
                    f"'{direction_key}', not '{key}'",
                    **get_location(quantity),
                )
        for key in (GeometricPropKey.Of, direction_key):
            if keys.count(key) != 1:
                raise TextXSemanticError(
                    f"'{quantity.name}' is a {quantity.type} -- it takes exactly one '{key}'",
                    **get_location(quantity),
                )
        operands.append(
            (geo_prop_value(quantity.props, direction_key), f"'{quantity.name}' {direction_key}", quantity)
        )
    for operand, label, owner in operands:
        location = get_location(owner)
        direction = operand.ref if isinstance(operand, ContextQuantityAlias) else operand
        if direction.type != QuantityType.Direction:
            raise TextXSemanticError(f"{label} is no 'direction' quantity", **location)
        if (
            geo_prop_value(direction.props, GeometricPropKey.AsSeenBy)
            or geo_prop_value(direction.props, GeometricPropKey.Wrt)
        ) is None:
            raise TextXSemanticError(
                f"{label} '{direction.name}' states no 'as-seen-by' frame", **location
            )
        value = direction.value
        if not isinstance(value, VectorXYZ) or _literal_xyz(value.coords) is None:
            raise TextXSemanticError(
                f"{label} '{direction.name}' is no literal 3-vector", **location
            )


def validate_alignment_targets(model: Model) -> None:
    """An angle drives to a direction (two DOF) or to a cone around it (one), with bounds in [0, pi]."""
    for spec in get_children_of_type(ConstraintSpecification, model):
        if view_form(spec) not in ANGLE_VIEW_FORMS:
            continue
        location = get_location(spec)
        expr = spec.expr
        if alignment_is_pointwise(spec):
            # Only the point target drives the two-axis rotation-vector row.
            if view_form(spec) == ViewForm.Alignment:
                reference = spec.view.binary.right
                reference = reference.ref if isinstance(reference, ContextQuantityAlias) else reference
                nonzero = [v for v in _literal_xyz(reference.value.coords) if abs(v) > 1e-9]
                if len(nonzero) != 1 or abs(abs(nonzero[0]) - 1.0) > 1e-6:
                    raise TextXSemanticError(
                        f"'{spec.name}' reference operand is no signed unit frame axis -- a point "
                        "target aligns with exactly one axis, +-1",
                        **location,
                    )
            cone = expr.threshold if isinstance(expr, LessThanConstraint) else None
            cone = expr.upper if isinstance(expr, BilateralConstraint) else cone
            if cone is None:
                continue
            named = cone.quantity.ref if isinstance(cone.quantity, ContextQuantityAlias) else cone.quantity
            if cone.bare is not None and cone.bare.value <= 0.0:
                raise TextXSemanticError(
                    f"'{spec.name}' bounds its band at {cone.bare.value} -- the bound is positive",
                    **location,
                )
            if cone.bare is None and (not isinstance(named, ContextQuantity) or named.type != QuantityType.Angle):
                raise TextXSemanticError(
                    f"'{spec.name}' bounds its band with no angle", **location
                )
            continue
        if isinstance(expr, EqualityConstraint):
            bounds = [(expr.reference, "target")]
        elif isinstance(expr, GreaterThanConstraint):
            bounds = [(expr.threshold, "threshold")]
        elif isinstance(expr, (BilateralConstraint, OutsideConstraint)):
            bounds = [(expr.lower, "lower bound"), (expr.upper, "upper bound")]
        else:
            raise TextXSemanticError(
                f"'{spec.name}' relates the angle between two directions by neither equality, an "
                "inequality, nor a band",
                **location,
            )
        radians = []
        for ref, what in bounds:
            named = ref.quantity.ref if isinstance(ref.quantity, ContextQuantityAlias) else ref.quantity
            measure = ref.bare
            if measure is None and isinstance(named, ContextQuantity) and isinstance(named.value, Measure):
                measure = named.value
            if measure is None:
                # A named angle with no literal value: its type is all that can be checked.
                if not isinstance(named, ContextQuantity) or named.type != QuantityType.Angle:
                    raise TextXSemanticError(f"'{spec.name}' states its {what} as no angle", **location)
                radians.append(None)
                continue
            if measure.unit not in ("rad", "deg", ""):
                raise TextXSemanticError(f"'{spec.name}' states its {what} as no angle", **location)
            value = measure.value * (math.pi / 180.0 if measure.unit == "deg" else 1.0)
            if not 0.0 <= value <= math.pi:
                raise TextXSemanticError(
                    f"'{spec.name}' {what} is {value} rad -- the angle between two directions "
                    "lies in [0, pi]",
                    **location,
                )
            radians.append(value)
        if len(radians) == 2 and None not in radians and not radians[0] < radians[1]:
            raise TextXSemanticError(
                f"'{spec.name}' lower bound is not below its upper bound", **location
            )


def validate_geometric_distance_views(model: Model) -> None:
    """Table II operands come in a supported kind and order; no unsigned distance is driven to zero.

    Its derivative is undefined at zero (Borghesan et al. 2016, section VII-B).
    """
    for spec in get_children_of_type(ConstraintSpecification, model):
        form = view_form(spec)
        if form not in (ViewForm.DistanceBetween, ViewForm.DistanceFrom, ViewForm.ProjectionOn):
            continue
        location = get_location(spec)
        binary = spec.view.binary
        a_kind = geometric_operand_kind(binary.left)
        b_kind = geometric_operand_kind(binary.right)
        unsigned = form == ViewForm.DistanceBetween
        if form == ViewForm.DistanceFrom:
            op_type = GEOMETRIC_DISTANCE_OPS.get((a_kind, b_kind))
            if op_type is None and a_kind != b_kind and GEOMETRIC_DISTANCE_OPS.get((b_kind, a_kind)):
                raise TextXSemanticError(
                    f"'{spec.name}' takes the distance of a {a_kind} from a {b_kind} -- the point "
                    "or line comes first, since the sign follows the primitive's direction",
                    **location,
                )
            if op_type is None:
                raise TextXSemanticError(
                    f"'{spec.name}' takes the distance of a {a_kind or 'unknown'} from a "
                    f"{b_kind or 'unknown'} -- a distance is of a pose from a plane or line, or of "
                    "a line from a line",
                    **location,
                )
            unsigned = op_type in UNSIGNED_GEOMETRIC_DISTANCE_OPS
        elif form == ViewForm.ProjectionOn and (b_kind != "line" or a_kind not in ("point", "line")):
            raise TextXSemanticError(
                f"'{spec.name}' projects a {a_kind or 'unknown'} on a {b_kind or 'unknown'} -- a "
                "projection is of a pose or a line on a line",
                **location,
            )
        expr = spec.expr
        to_zero = (
            isinstance(expr, EqualityConstraint)
            and expr.reference.bare is not None
            and expr.reference.bare.value == 0.0
        ) or (
            isinstance(expr, BilateralConstraint)
            and expr.lower.bare is not None
            and expr.upper.bare is not None
            and expr.lower.bare.value == 0.0
            and expr.upper.bare.value == 0.0
        )
        if unsigned and to_zero:
            raise TextXSemanticError(
                f"'{spec.name}' drives an unsigned distance to zero, where its derivative is "
                "undefined -- state the coincidence as signed projections",
                **location,
            )


def validate_bare_axis_selectors(model: Model) -> None:
    """A bare `.x` selects one component of a 3-vector context quantity; others name a subspace."""
    holders = [
        *get_children_of_type(View, model),
        *get_children_of_type(ContextRef, model),
        *get_children_of_type(QuantityLeaf, model),
    ]
    for item in holders:
        if item.axis is None or item.subspace is not None:
            continue
        quantity = item.quantity.ref if isinstance(item.quantity, ContextQuantityAlias) else item.quantity
        if not isinstance(quantity, ContextQuantity):
            raise TextXSemanticError(
                f"'{quantity.name}' names no subspace before '.{item.axis}' -- as in "
                f"'.linvel.{item.axis}'; a bare axis belongs on a 3-vector context quantity",
                **get_location(item),
            )
        if quantity.type not in VECTOR_COMPONENT_TYPE:
            raise TextXSemanticError(
                f"'{quantity.name}' is a {quantity.type} with no axis components -- a bare "
                f"'.{item.axis}' selects one axis of a direction, free-vector, position, "
                "linear-velocity, force or torque",
                **get_location(item),
            )


def validate_world_quantities(model: Model) -> None:
    """A world pose has frames; a world wrench has its frames, its source, and events that tare it."""
    for quantity in get_children_of_type(WorldQuantity, model):
        if isinstance(quantity, WorldQuantityAlias):
            continue
        location = get_location(quantity)
        if quantity.type == WorldQuantityType.Pose and pose_frame_names(quantity) is None:
            raise TextXSemanticError(f"pose '{quantity.name}' states no of and wrt frames", **location)
        if quantity.type != WorldQuantityType.Wrench:
            continue
        props = quantity.props
        ft_sensor = geo_prop_value(props, "ft-sensor")
        estimated = geo_prop_value(props, "estimated-from") is not None
        if ft_sensor is not None and estimated:
            raise TextXSemanticError(
                f"wrench '{quantity.name}' names both ft-sensor and estimated-from -- a wrench is "
                "measured or estimated, not both",
                **location,
            )
        if estimated and geo_prop(props, "ref-point") is None:
            raise TextXSemanticError(f"wrench '{quantity.name}' states no ref-point frame", **location)
        sensor_frame = str(ft_sensor.frame.uri) if ft_sensor is not None else None
        if (geo_prop(props, "as-seen-by") or sensor_frame) is None:
            raise TextXSemanticError(f"wrench '{quantity.name}' states no as-seen-by frame", **location)
        events = geo_prop_events(props, "re-tare-on")
        tared = ft_sensor is not None or estimated
        if events and not tared:
            raise TextXSemanticError(
                f"wrench '{quantity.name}' names re-tare-on events -- nothing measures it to tare",
                **location,
            )
        # No implicit tare at startup: the model names its own run-start event.
        if tared and not events:
            raise TextXSemanticError(
                f"wrench '{quantity.name}' names no re-tare-on events -- name the run-start event "
                "to tare once at startup",
                **location,
            )
        for event in events:
            if event.event is None:
                raise TextXSemanticError(
                    f"wrench '{quantity.name}' re-tares on '{event.name}', which carries no "
                    "namespace -- write it as ns.EVENT",
                    **location,
                )


def validate_context_geometry(model: Model) -> None:
    """Poses, wrenches, orientations, directions and paths have frames and literal rotations."""
    for quantity in get_children_of_type(ContextQuantity, model):
        if isinstance(quantity, ContextQuantityAlias):
            continue
        location = get_location(quantity)
        value = quantity.value
        if (
            quantity.type == QuantityType.Pose
            and not isinstance(value, (PoseCoordinate, ConfigValue, SampledValue))
            and pose_frame_names(quantity) is None
        ):
            raise TextXSemanticError(f"pose '{quantity.name}' states no of and wrt frames", **location)
        if quantity.type == QuantityType.Wrench and not isinstance(value, SampledValue):
            props = quantity.props
            if not isinstance(props, GeometricProps) and isinstance(value, SnapshotValue):
                source = value.source.quantity
                props = source.props if isinstance(source, WorldQuantity) else None
            if geo_prop(props, "as-seen-by") is None:
                raise TextXSemanticError(
                    f"wrench '{quantity.name}' states no as-seen-by frame", **location
                )
        if quantity.type == QuantityType.Direction:
            if (geo_prop(quantity.props, "as-seen-by") or geo_prop(quantity.props, "wrt")) is None:
                raise TextXSemanticError(
                    f"direction '{quantity.name}' states no 'as-seen-by' frame", **location
                )
            if isinstance(value, VectorXYZ) and _literal_xyz(value.coords) is None:
                raise TextXSemanticError(
                    f"direction '{quantity.name}' is no three literal components", **location
                )
            if value is not None and not isinstance(value, (VectorXYZ, DirectionBetween)):
                raise TextXSemanticError(
                    f"direction '{quantity.name}' is no vector literal", **location
                )
        if isinstance(value, PoseCoordinate):
            orientation = value.orientation
            if pose_frame_names(quantity) is None:
                sources = [
                    ref.quantity
                    for ref in (value.position.ref, orientation.ref)
                    if ref is not None and ref.quantity is not None
                ]
                frames = {names for source in sources if (names := pose_frame_names(source))}
                if len(frames) > 1:
                    raise TextXSemanticError(
                        f"pose '{quantity.name}' components disagree on their frames", **location
                    )
            relative = orientation.relative
            # (coordinates, whether a symbolic component is allowed there)
            literal = []
            if orientation.quat is not None:
                literal.append(orientation.quat.xyzw)
            if orientation.euler is not None and len(set(orientation.euler.axes)) != 3:
                literal.append(orientation.euler.angles)
            for cosine in (orientation.direction_cosine, relative.direction_cosine if relative else None):
                if cosine is not None:
                    literal += [cosine.x_axis, cosine.y_axis, cosine.z_axis]
            if relative is not None and relative.direction_cosine is None:
                literal.append(relative.quat.xyzw if relative.quat is not None else relative.euler.angles)
            if any(element.ref is not None for coords in literal for element in coords.values):
                raise TextXSemanticError(
                    f"orientation of '{quantity.name}' has symbolic components -- the backend "
                    "composes literal rotations",
                    **location,
                )
            if relative is not None:
                base = relative.base.quantity
                frames = pose_frame_names(base) if isinstance(base, ContextQuantity) else None
                if frames is None:
                    raise TextXSemanticError(
                        f"relative orientation on '{quantity.name}' has no base pose with of, wrt "
                        "and as-seen-by frames to compose in",
                        **location,
                    )
                of_frame, _wrt_frame, base_as_seen_by = frames
                if relative.frame is not None:
                    basis = str(relative.frame.uri)
                elif relative.euler is None:
                    raise TextXSemanticError(
                        f"relative orientation on '{quantity.name}' states no basis frame -- a "
                        "quaternion or direction-cosine delta needs one",
                        **location,
                    )
                else:
                    basis = base_as_seen_by if relative.euler.extrinsic else of_frame
                if basis not in (of_frame, base_as_seen_by):
                    raise TextXSemanticError(
                        f"relative orientation on '{quantity.name}' turns in '{basis}' -- it is "
                        f"neither the base's body frame '{of_frame}' nor its basis "
                        f"'{base_as_seen_by}', and a change of basis is not supported",
                        **location,
                    )
        if quantity.type != ReferenceGeneratorType.Path:
            continue
        if value.lerp is not None:
            start, goal = value.lerp.start.quantity, value.lerp.goal.quantity
            start_type = start.type if isinstance(start, ContextQuantity) else None
            goal_type = goal.type if isinstance(goal, ContextQuantity) else None
            if QUDT_KIND_BY_QUANTITY_TYPE.get(start_type) != QUDT_KIND_BY_QUANTITY_TYPE.get(goal_type):
                raise TextXSemanticError(
                    f"lerp path '{quantity.name}' runs from a {start_type} to a {goal_type} -- "
                    "both ends are one kind",
                    **location,
                )
            if start_type != QuantityType.Pose:
                continue
        if pose_frame_names(quantity) is not None:
            continue
        endpoint_frames = [
            (endpoint, frames)
            for endpoint in path_pose_endpoints(quantity)
            if (frames := pose_frame_names(endpoint)) is not None
        ]
        if not endpoint_frames:
            raise TextXSemanticError(
                f"path '{quantity.name}' has no endpoint with frames", **location
            )
        if len({frames for _endpoint, frames in endpoint_frames}) != 1:
            details = ", ".join(
                f"{endpoint.name}={tuple(map(str, frames))}" for endpoint, frames in endpoint_frames
            )
            raise TextXSemanticError(
                f"path '{quantity.name}' endpoints disagree on their frames: {details}", **location
            )


def validate_view_operands(model: Model) -> None:
    """A view's operands can be read: a subspace, a clock, poses, framed primitives, a 3-vector."""
    duration_default = QuantityType.Duration in {
        entry.kind for entry in get_children_of_type(ToleranceDefault, model)
    }
    for spec in get_children_of_type(ConstraintSpecification, model):
        if spec.disabled:
            continue
        location = get_location(spec)
        view = spec.view
        binary = view.binary
        form = view_form(spec)
        if view.elapsed is not None:
            expr = spec.expr
            if isinstance(expr, (GreaterThanConstraint, LessThanConstraint)):
                refs = [expr.threshold]
            elif isinstance(expr, EqualityConstraint):
                # A sampled clock never lands exactly on the reference.
                if spec.tolerance is None and not duration_default:
                    raise TextXSemanticError(
                        f"elapsed equality '{spec.name}' states no band -- a sampled clock never "
                        "lands exactly on the reference; write '... equal to <t> within <band>'",
                        **location,
                    )
                refs = [expr.reference, spec.tolerance]
            else:
                raise TextXSemanticError(
                    f"timing constraint '{spec.name}' relates the clock by neither 'greater than', "
                    "'less than' nor 'equal to'",
                    **location,
                )
            for ref in refs:
                if ref is not None and ref.bare is None and not isinstance(ref.quantity, ContextQuantity):
                    raise TextXSemanticError(
                        f"timing constraint '{spec.name}' compares with no declared duration or "
                        "inline literal like `5.0 s`",
                        **location,
                    )
            continue
        quantity = resolved_constraint_quantity(spec)
        if binary is None and quantity is not None:
            subspace = constraint_view_subspace(spec)
            if subspace is None:
                raise TextXSemanticError(f"constraint '{spec.name}' selects no subspace", **location)
            if quantity.type == WorldQuantityType.Pose and subspace == "distance" and view.axis is None:
                raise TextXSemanticError(
                    f"constraint '{spec.name}' takes a distance of one pose -- write "
                    "'distance between <pose-a> and <pose-b>'",
                    **location,
                )
        if form == ViewForm.DistanceBetween:
            for ref in (binary.left, binary.right):
                pose = ref.ref if isinstance(ref, (WorldQuantityAlias, ContextQuantityAlias)) else ref
                framed = isinstance(pose, ContextQuantity) or isinstance(pose.props, GeometricProps)
                if pose.type not in (WorldQuantityType.Pose, QuantityType.Pose) or not framed:
                    raise TextXSemanticError(
                        f"distance constraint '{spec.name}' takes '{ref.name}' -- it is no pose "
                        "with frames",
                        **location,
                    )
                if pose_frame_names(ref) is None:
                    raise TextXSemanticError(
                        f"distance constraint '{spec.name}' takes '{ref.name}', which states no "
                        "of and wrt frames",
                        **location,
                    )
        if form not in (ViewForm.DistanceFrom, ViewForm.ProjectionOn):
            continue
        table = GEOMETRIC_DISTANCE_OPS if form == ViewForm.DistanceFrom else GEOMETRIC_PROJECTION_OPS
        op_type = table.get((geometric_operand_kind(binary.left), geometric_operand_kind(binary.right)))
        # The frame each primitive's own direction is seen in: a line's `along`, a plane's `normal`.
        seen_by = []
        for ref in (binary.left, binary.right):
            primitive = ref.ref if isinstance(ref, ContextQuantityAlias) else ref
            if not isinstance(primitive, ContextQuantity) or primitive.type not in _PRIMITIVE_DIRECTION_KEY:
                seen_by.append(None)
                continue
            direction = geo_prop_value(primitive.props, _PRIMITIVE_DIRECTION_KEY[primitive.type])
            direction = direction.ref if isinstance(direction, ContextQuantityAlias) else direction
            seen_by.append(geo_prop(direction.props, "as-seen-by"))
        if op_type in _LINE_LINE_OPS:
            if seen_by[0] is None or seen_by[1] != seen_by[0]:
                raise TextXSemanticError(
                    f"constraint '{spec.name}' states its lines' directions in different frames "
                    "-- both are seen by one",
                    **location,
                )
            continue
        frames = pose_frame_names(binary.left)
        if frames is None:
            raise TextXSemanticError(
                f"constraint '{spec.name}' takes a pose operand with no frames", **location
            )
        if seen_by[1] != frames[1]:
            primitive = binary.right.ref if isinstance(binary.right, ContextQuantityAlias) else binary.right
            role = "normal" if primitive.type == QuantityType.Plane else "direction"
            raise TextXSemanticError(
                f"constraint '{spec.name}' states '{primitive.name}' {role} as seen by "
                f"'{seen_by[1]}' -- it is seen by '{frames[1]}', the point's reference frame",
                **location,
            )
    for view in get_children_of_type(View, model):
        if view.norm is None:
            continue
        location = get_location(view)
        quantity = view.quantity.ref if isinstance(view.quantity, WorldQuantityAlias) else view.quantity
        if not isinstance(quantity, WorldQuantity) or view.subspace is None or view.axis is not None:
            raise TextXSemanticError(f"norm of '{quantity.name}' is no 3-vector view", **location)
        raw = str(view.subspace)
        mapped = (
            raw
            if quantity.type == WorldQuantityType.Pose and raw == "position"
            else SUBSPACE_ALIAS.get(raw, raw)
        )
        if scalar_type(quantity, mapped, None) not in NORM_SCALAR_TYPES:
            raise TextXSemanticError(
                f"norm of '{quantity.name}.{mapped}' is no 3-vector view", **location
            )
        across = view.norm.across
        if across is None:
            continue
        direction = across.quantity.ref if isinstance(across.quantity, ContextQuantityAlias) else across.quantity
        if not isinstance(direction, ContextQuantity) or direction.type != QuantityType.Direction:
            raise TextXSemanticError(
                f"norm of '{quantity.name}.{raw}' is taken across no direction", **location
            )
        vector_frame = quantity_axis_frame(quantity)
        direction_frame = geo_prop(direction.props, "as-seen-by")
        if vector_frame != direction_frame:
            raise TextXSemanticError(
                f"norm of '{quantity.name}.{raw}' across '{direction.name}' -- the direction is "
                f"seen by '{direction_frame}', the vector by '{vector_frame}'",
                **location,
            )


def validate_motion_world_scope(model: Model) -> None:
    """What a motion reads is declared in its own scope, and a constraint names one scalar kind.

    An angle, a Table II expression and a `from/to` direction read a declared pose; a followed
    path's speed reads a declared twist; no constraint names a whole pose or a mass.
    """
    models = get_included_models(model)
    whole_kinds = {
        QuantityType.Pose: "a whole pose -- select '.position' or '.orientation', each with its "
        "own tolerance",
        QuantityType.Mass: "a mass -- no constraint type exists for that kind",
    }
    for handler in get_children_of_type(ConstraintHandler, model):
        motion = handler.motion
        context_quantities = motion_context_quantities(models, motion, handler)
        world = motion_world_quantities(models, motion, handler, context_quantities)
        # First declaration wins, as it does for the emitter reading the scope.
        poses: dict[tuple, WorldQuantity] = {}
        twists: dict[tuple, WorldQuantity] = {}
        for quantity in world.values():
            frames = (geo_prop(quantity.props, "of"), geo_prop(quantity.props, "wrt"))
            if quantity.type == WorldQuantityType.Pose:
                poses.setdefault(frames, quantity)
            elif quantity.type == WorldQuantityType.VelocityTwist:
                twists.setdefault(frames, quantity)
        # (reader, (of, wrt) frames of the pose it reads, frame it must be seen by or None, reader kind)
        pose_reads = []
        # (constraint, moved quantity, (of, wrt) frames of the twist it reads)
        twist_reads = []
        for spec in resolved_constraint_items(motion) + perturbation_conditions(handler):
            location = get_location(spec)
            view = spec.view
            form = view_form(spec)
            if form in ANGLE_VIEW_FORMS:
                operands = (view.binary.left, view.binary.right)
                planes = (form == ViewForm.PlaneAngle, form != ViewForm.Alignment)
                frames = []
                for operand, is_plane in zip(operands, planes):
                    operand = operand.ref if isinstance(operand, ContextQuantityAlias) else operand
                    direction = geo_prop_value(operand.props, GeometricPropKey.Normal) if is_plane else operand
                    direction = direction.ref if isinstance(direction, ContextQuantityAlias) else direction
                    frames.append(geo_prop(direction.props, "as-seen-by") or geo_prop(direction.props, "wrt"))
                pose_reads.append((spec, tuple(frames), frames[1], "an angle"))
            elif form in (ViewForm.DistanceFrom, ViewForm.ProjectionOn):
                a_ref, b_ref = view.binary.left, view.binary.right
                table = GEOMETRIC_DISTANCE_OPS if form == ViewForm.DistanceFrom else GEOMETRIC_PROJECTION_OPS
                op_type = table.get((geometric_operand_kind(a_ref), geometric_operand_kind(b_ref)))
                primitive = b_ref.ref if isinstance(b_ref, ContextQuantityAlias) else b_ref
                if op_type in _LINE_LINE_OPS:
                    line_a = a_ref.ref if isinstance(a_ref, ContextQuantityAlias) else a_ref
                    along = geo_prop_value(line_a.props, GeometricPropKey.Along)
                    along = along.ref if isinstance(along, ContextQuantityAlias) else along
                    frame = geo_prop(along.props, "as-seen-by")
                    for line in (line_a, primitive):
                        pose_reads.append((spec, (geo_prop(line.props, "of"), frame), None, "a Table II expression"))
                elif op_type is not None:
                    point = a_ref if isinstance(a_ref, WorldQuantity) else world.get(a_ref.name, a_ref)
                    frames = pose_frame_names(point)
                    primitive_frame = geo_prop(primitive.props, "of")
                    # A line through the frame the point is measured against needs no origin pose.
                    body_line = (
                        op_type == "PointLineToLinearDistance"
                        and frames is not None
                        and primitive_frame == frames[1]
                    )
                    if frames is not None and not body_line:
                        pose_reads.append((spec, (primitive_frame, frames[1]), None, "a Table II expression"))
            operand = view.moving or view.on or view.progress
            if operand is not None:
                moved = operand.moved if isinstance(operand.moved, WorldQuantity) else world.get(operand.moved.name)
                frames = pose_frame_names(moved) if moved is not None else None
                if frames is not None:
                    twist_reads.append((spec, moved, frames[:2]))
            quantity = view.quantity
            if (
                spec.tolerance is not None
                and isinstance(quantity, WorldQuantity)
                and quantity.type == WorldQuantityType.Pose
                and constraint_view_subspace(spec) == "pose"
                and view.axis is None
            ):
                raise TextXSemanticError(
                    f"constraint '{spec.name}' states a band on a whole pose -- its error mixes a "
                    "position and an orientation; state one band on '.position' and one on "
                    "'.orientation'",
                    **location,
                )
            kind = None
            if isinstance(quantity, ContextQuantity) and view.subspace is None and view.on is None:
                kind = quantity.type
            elif view.expr is not None and quantity is None and view.elapsed is None:
                kind = infer(view.expr)
            if kind in whole_kinds:
                raise TextXSemanticError(f"constraint '{spec.name}' constrains {whole_kinds[kind]}", **location)
            if kind is not None and kind not in DIMENSION_VECTOR:
                raise TextXSemanticError(
                    f"constraint '{spec.name}' names '{kind}' -- it is no scalar kind", **location
                )
        for quantity in context_quantities.values():
            if isinstance(quantity.value, DirectionBetween):
                pose_reads.append(
                    (
                        quantity,
                        (str(quantity.value.to_frame.uri), str(quantity.value.from_frame.uri)),
                        geo_prop(quantity.props, "as-seen-by") or geo_prop(quantity.props, "wrt"),
                        "a direction between two frames",
                    )
                )
        for reader, (of_frame, wrt_frame), seen_by, what in pose_reads:
            pose = poses.get((of_frame, wrt_frame))
            if pose is None:
                raise TextXSemanticError(
                    f"'{reader.name}' reads a world pose of '{of_frame}' wrt '{wrt_frame}' that "
                    f"motion '{motion.name}' does not declare -- {what} reads an already computed "
                    "pose, it does not derive one",
                    **get_location(reader),
                )
            pose_seen_by = geo_prop(pose.props, "as-seen-by") or wrt_frame
            if seen_by is not None and pose_seen_by != seen_by:
                raise TextXSemanticError(
                    f"'{reader.name}' reads '{pose.name}', seen by '{pose_seen_by}', from "
                    f"'{seen_by}' -- {what} answers in the frame the pose is seen by",
                    **get_location(reader),
                )
        for spec, moved, (of_frame, wrt_frame) in twist_reads:
            if (of_frame, wrt_frame) not in twists:
                raise TextXSemanticError(
                    f"constraint '{spec.name}' follows a path with '{moved.name}' -- motion "
                    f"'{motion.name}' declares no velocity-twist of '{of_frame}' wrt '{wrt_frame}', "
                    "the measured speed along the path",
                    **get_location(spec),
                )


def validate_resolved_views(model: Model) -> None:
    """What a view resolves to in its motion's scope can be emitted: a band, a frame, a kind, a pose's frames."""
    models = get_included_models(model)
    defaults = {entry.kind for included in models for entry in get_children_of_type(ToleranceDefault, included)}
    for handler in get_children_of_type(ConstraintHandler, model):
        motion = handler.motion
        context_quantities = motion_context_quantities(models, motion, handler)
        world = motion_world_quantities(models, motion, handler, context_quantities)
        constraints = resolved_constraint_items(motion) + perturbation_conditions(handler)
        for spec in constraints:
            location = get_location(spec)
            view = spec.view
            if view.elapsed is not None:
                continue
            kind = constraint_kind(spec)
            expr = spec.expr
            # An equality is never met exactly, so it ends with a band.
            equality = isinstance(expr, EqualityConstraint) or view.on is not None or view.moving is not None
            default_kind = TOLERANCE_DEFAULT_KIND.get(kind, kind)
            if equality and spec.tolerance is None and default_kind not in defaults:
                raise TextXSemanticError(
                    f"equality constraint '{spec.name}' states no band -- an equality is only ever met "
                    f"within one; write '... within <band>' or declare a default for '{default_kind}' "
                    "in a 'tolerances' block",
                    **location,
                )
            if view.progress is not None:
                continue
            if isinstance(expr, EqualityConstraint):
                slots = [expr.reference]
            elif isinstance(expr, (GreaterThanConstraint, LessThanConstraint)):
                slots = [expr.threshold]
            elif isinstance(expr, (BilateralConstraint, OutsideConstraint)):
                slots = [expr.lower, expr.upper]
            else:
                slots = []
            for ref in slots:
                if ref.expr is None:
                    continue
                inferred = infer(op_tree(ref.expr))
                if not same_scalar_dimension(kind, inferred):
                    raise TextXSemanticError(
                        f"constraint '{spec.name}' compares a {kind} with an expression that infers "
                        f"{inferred}",
                        **location,
                    )
        for quantity in context_quantities.values():
            if not isinstance(quantity.value, PoseCoordinate) or pose_frame_names(quantity) is not None:
                continue
            if pose_frame_source(quantity, constraints, world) is None:
                raise TextXSemanticError(
                    f"pose '{quantity.name}' states no frames -- give it of, wrt and as-seen-by, or "
                    "make it a path endpoint, a relative orientation, a component reference, or an "
                    "equality constraint's reference",
                    **get_location(quantity),
                )
