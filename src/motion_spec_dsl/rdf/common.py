# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Pure helpers and plan records shared by motion-spec RDF emitters."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from rdflib.term import URIRef

from motion_spec_dsl.classes.constraints import (
    ConstraintSpecification,
    GoalStatusConstraint,
    _flatten_constraint_items,
    _resolved_spec,
)
from motion_spec_dsl.classes.context import (
    ContextQuantity,
    ContextRef,
    GeometricProps,
    GeoPropPair,
    QuantityType,
    WorldQuantity,
    WorldQuantityType,
    _resolved_context_quantity,
)
from motion_spec_dsl.classes.controller_semantics import (
    _alignment_is_pointwise,
    constraint_view_subspace,
)
from motion_spec_dsl.classes.coordinates import const_value
from motion_spec_dsl.classes.motion_spec import GuardedMotion
from motion_spec_dsl.classes.units import (
    ANGLE_UNITS as ANGLE_UNITS,
)
from motion_spec_dsl.classes.units import (
    _angle_unit as _angle_unit,
)
from motion_spec_dsl.classes.units import (
    _dsl_unit as _dsl_unit,
)
from motion_spec_dsl.rdf.model import WORLD_SPECS


def _ns_term(namespace: Any, name: str) -> URIRef:
    """URI for `name` in `namespace` (the namespace's base IRI concatenated with `name`)."""
    return URIRef(str(namespace._NS) + name)


def _node_name(value: Any) -> str:
    """The `name` attribute of `value`, falling back to `str(value)`."""
    return value.name if hasattr(value, "name") else str(value)


def _geo_prop(props: GeometricProps | None, key: str) -> str | None:
    """Value of geometric prop `key` (of/wrt/as-seen-by/ref-point/...) in `props`, or None."""
    if props is None:
        return None
    for pair in props.pairs:
        if isinstance(pair, GeoPropPair) and pair.key == key:
            value = pair.frame or pair.joint or pair.sensor or pair.value
            return str(getattr(value, "uri", value))
    return None


def _angle_bound(bound) -> float:
    """An authored angle bound as a number: any constant expression over literals and pi."""
    return const_value(getattr(bound, "value", bound))


def _geo_prop_events(props: GeometricProps | None, key: str) -> list:
    """The event list carried by geometric prop `key`, or empty."""
    if props is None:
        return []
    for pair in props.pairs:
        if isinstance(pair, GeoPropPair) and pair.key == key:
            return list(pair.events or [])
    return []


def _geo_prop_value(props: GeometricProps | None, key: str):
    """The raw value of geometric prop `key`, for a prop carrying a structure rather than a
    reference to something already named in the graph."""
    if props is None:
        return None
    for pair in props.pairs:
        if isinstance(pair, GeoPropPair) and pair.key == key:
            return pair.normalization or pair.value
    return None


def _binary_view(constraint: ConstraintSpecification):
    """The constraint's view as its `BinaryView` instance (any of the 4 binary forms), or None."""
    return getattr(constraint.view, "binary", None)


def _binary_view_kind(constraint: ConstraintSpecification) -> str | None:
    """The grammar rule name of the constraint's binary view, or None."""
    binary = _binary_view(constraint)
    return type(binary).__name__ if binary is not None else None


def _is_distance_view(constraint: ConstraintSpecification) -> bool:
    """Whether the constraint's view is a `distance between A and B` form."""
    return _binary_view_kind(constraint) == "DistanceBetweenView"


def _is_angle_between_view(constraint: ConstraintSpecification) -> bool:
    """Whether the constraint's view is any `angle between A and B` form."""
    return _binary_view_kind(constraint) == "AngleBetweenView"


def _is_geometric_distance_view(constraint: ConstraintSpecification) -> bool:
    """Whether the constraint's view is a `distance of A from B` (Table IIa) form."""
    return _binary_view_kind(constraint) == "DistanceFromView"


def _is_projection_view(constraint: ConstraintSpecification) -> bool:
    """Whether the constraint's view is a `projection of A on B` (Table IIa) form."""
    return _binary_view_kind(constraint) == "ProjectionOnView"


def _is_plane_operand(operand: ContextQuantity) -> bool:
    """Whether an `angle between` operand is a plane rather than a versor direction."""
    return _resolved_context_quantity(operand).type == QuantityType.Plane


def _is_alignment_view(constraint: ConstraintSpecification) -> bool:
    """Whether the constraint's view is the versor-versor `angle between A and B` form. Any new
    call site must decide explicitly whether it wants this (versor-versor only) or the broader
    `_is_angle_between_view`.
    """
    if not _is_angle_between_view(constraint):
        return False
    binary = _binary_view(constraint)
    return not _is_plane_operand(binary.left) and not _is_plane_operand(binary.right)


def _is_incident_angle_view(constraint: ConstraintSpecification) -> bool:
    """Whether the constraint's view is the versor-plane `angle between A and B` form."""
    if not _is_angle_between_view(constraint):
        return False
    binary = _binary_view(constraint)
    return not _is_plane_operand(binary.left) and _is_plane_operand(binary.right)


def _is_plane_angle_view(constraint: ConstraintSpecification) -> bool:
    """Whether the constraint's view is the plane-plane `angle between A and B` form."""
    if not _is_angle_between_view(constraint):
        return False
    binary = _binary_view(constraint)
    return _is_plane_operand(binary.left) and _is_plane_operand(binary.right)


def _alignment_bound_token(ref: Any) -> str | None:
    """Id fragment for one target/bound ref of an alignment's relation, or None for a literal
    zero -- every zero-target alignment means the same geometry, so a zero must not fragment the
    shared op chain (plan 10 STOP 1).
    """
    bare = getattr(ref, "bare", None)
    if bare is not None:
        return None if bare.value == 0.0 else f"{bare.value}{bare.unit}"
    quantity = _context_quantity(ref)
    return _resolved_context_quantity(quantity).name if quantity is not None else None


def _alignment_id(quantity: WorldQuantity, constraint: ConstraintSpecification) -> str:
    """Scalar id of an `angle between` view: the carrier pose and both direction operands. The
    operands belong in it because two alignments can share one pose and mean different angles.
    """
    binary = _binary_view(constraint)
    moving = _resolved_context_quantity(binary.left).name
    reference = _resolved_context_quantity(binary.right).name
    stem = f"alignment-{moving}-{reference}"
    # A tolerated cone or a nonzero target is part of what the angle is computed for, so two
    # motions that differ in either get their own op chain rather than one that can only answer
    # for one of them (the 2-DOF row's `_bind_alignment_band` raises on exactly this collision;
    # the 1-DOF row has no such guard, so the id has to do the separating itself).
    for attr in ("reference", "threshold", "lower", "upper"):
        bound = getattr(constraint.expr, attr, None)
        if bound is None:
            continue
        token = _alignment_bound_token(bound)
        if token is not None:
            stem = f"{stem}-{token}"
    # A bound and a target at one value fold to the same token, but they drive different rows --
    # `less than 0.5` is the 2-DOF cone bound, `equal to 0.5` the 1-DOF cone target. Without this
    # the emission loop dedupes them onto one chain and the second constraint loses its row.
    if not _alignment_is_pointwise(constraint):
        stem = f"{stem}-cone"
    return _scalar_id(quantity, stem, None)


def _view_subspace(constraint: ConstraintSpecification) -> str:
    """The constraint's resolved view subspace; raises if it declares none."""
    subspace = constraint_view_subspace(constraint)
    if subspace is None:
        raise ValueError(f"Constraint '{constraint.name}' must define a view subspace.")
    return subspace


def _scalar_id(quantity: WorldQuantity, subspace: str, axis: str | None) -> str:
    """Id stem for a scalar view of `quantity`: `<name>.<subspace>[.<axis>]`
    (bare `<name>` for joint positions)."""
    if quantity.type == WorldQuantityType.JointPosition:
        return quantity.name
    if axis is None:
        return f"{quantity.name}.{subspace}"
    return f"{quantity.name}.{subspace}.{axis}"


def _gradient_scalar_id(quantity: WorldQuantity, constraint: ConstraintSpecification) -> str | None:
    """Id of the runtime gradient `DirectionCoordinate` `_emit_map_operations` publishes for
    `constraint`'s view, or None when the view has no gradient: axis-based, point-point
    `distance between` (runtime `PoseToDirection`, not a gradient), or a pointwise alignment
    (rotation-vector error, not a 1-DOF gradient row).
    """
    if _is_alignment_view(constraint):
        if _alignment_is_pointwise(constraint):
            return None
        base = _alignment_id(quantity, constraint)
    elif _is_geometric_distance_view(constraint) or _is_projection_view(constraint):
        base = _scalar_id(quantity, _view_subspace(constraint), None)
    elif _is_incident_angle_view(constraint) or _is_plane_angle_view(constraint):
        base = _alignment_id(quantity, constraint)
    else:
        return None
    return f"{base}-gradient"


def _axis_vector(axis: str) -> tuple[float, float, float]:
    """Unit vector for axis `'x'|'y'|'z'`."""
    return {
        "x": (1.0, 0.0, 0.0),
        "y": (0.0, 1.0, 0.0),
        "z": (0.0, 0.0, 1.0),
    }[axis]


def _quantity_axis_frame(quantity: WorldQuantity) -> str | None:
    """Frame the quantity's axes are expressed in: its `as-seen-by`, or `wrt` for a Pose."""
    props = quantity.props if isinstance(quantity.props, GeometricProps) else None
    axis_frame = _geo_prop(props, "as-seen-by")
    if axis_frame is not None:
        return axis_frame
    if quantity.type == WorldQuantityType.Pose:
        return _geo_prop(props, "wrt")
    return None


# Table IIa (plan 08) constraint-view subspace tokens: every one of them is a length, signed or
# not, so they all resolve to QuantityType.Distance -- same as the existing "distance" subspace.
_GEOMETRIC_DISTANCE_SUBSPACES = frozenset(
    {
        "point-plane-distance",
        "point-line-distance",
        "point-line-projection",
        "line-line-distance",
        "line-line-projection",
    }
)


def _scalar_type(quantity: WorldQuantity, subspace: str, axis: str | None) -> Any:
    """QuantityType of the scalar/vector a `quantity.subspace[.axis]` view resolves to
    (e.g. Pose.position -> Position, Pose.position.x -> Distance)."""
    if quantity.type == WorldQuantityType.JointPosition:
        return QuantityType.Angle
    if quantity.type == WorldQuantityType.Pose:
        if subspace == "pose":
            return QuantityType.Pose
        if subspace == "position":
            return QuantityType.Position if axis is None else QuantityType.Distance
        if subspace == "orientation":
            return QuantityType.Orientation if axis is None else QuantityType.Angle
        if subspace == "distance" or subspace in _GEOMETRIC_DISTANCE_SUBSPACES:
            return QuantityType.Distance
        if subspace == "rotation":
            return QuantityType.PlaneAngle
        if subspace in ("alignment", "incident-angle", "plane-angle"):
            return QuantityType.Angle
    spec = WORLD_SPECS.get(quantity.type)
    if spec is None:
        return subspace
    prop = spec[3].get(subspace)
    return prop[3] if prop else subspace


def _evaluator_id(spec: ConstraintSpecification) -> str:
    """Stable id for a constraint's monitor evaluator, qualified by motion + section kind."""
    section = getattr(spec, "parent", None)
    motion = getattr(section, "parent", None) if section is not None else None
    section_kind = getattr(section, "kind", None)
    motion_name = getattr(motion, "name", None)
    if motion_name and section_kind:
        return f"eval-{motion_name}-{section_kind}-{spec.name}"
    return f"eval-{spec.name}"


def _context_quantity(ref: ContextRef) -> ContextQuantity | None:
    """The ContextQuantity a ref points at, whether named or declared inline."""
    return getattr(ref, "quantity", None)


def _resolved_constraint_items(motion: GuardedMotion) -> list[ConstraintSpecification]:
    """Enabled, alias-resolved constraint specs from the motion's when/while/until sections."""
    out = []
    for section in (motion.when, motion.while_, motion.until):
        for item in _flatten_constraint_items(section.constraints):
            spec = _resolved_spec(item)
            # A goal-status item compares an action's outcome, not a world quantity: it has no
            # view, no reference quantity and no scalar to project.
            if isinstance(spec, GoalStatusConstraint):
                continue
            if not spec.disabled:
                out.append(spec)
    return out


@dataclass(frozen=True)
class _DistancePlan:
    """Resolved endpoints and scalar-view carrier for an authored distance relation.

    `relation_a`/`relation_b` are the endpoints' origin **Point** entities -- not the pose
    coordinates themselves -- so `between-entities` names entities, matching
    `_GeometricDistancePlan`.
    """

    start: WorldQuantity
    end: WorldQuantity
    target: WorldQuantity
    relation_a: str
    relation_b: str


@dataclass(frozen=True)
class _AlignmentPlan:
    """Resolved direction operands and pose carrier for an authored `angle between` view.

    `relation_a`/`relation_b` are the between-entities pair: the two versors themselves for
    versor-versor, a versor and the *plane* (not its normal direction) for versor-plane, the
    two planes (not their normals) for plane-plane.
    """

    moving: ContextQuantity
    reference: ContextQuantity
    target: WorldQuantity
    relation_a: str
    relation_b: str


@dataclass(frozen=True)
class _GeometricDistancePlan:
    """Resolved operands and scalar-view carrier for an authored Table IIa distance/projection
    view (plan 08): a point-plane/point-line/point-on-line expression, or a line-line one.

    `direction`/`pose` are mutually exclusive with `diff_in1`/`diff_in2`: the first three ops
    (point vs. primitive) carry a direction role; the line-line pair instead composes a
    PoseDiffEvaluator over the two lines' origins and carries its `pose` output plus the two
    origin poses that fed it.
    """

    op_type: str
    in1: str
    in2: str
    direction: str | None
    pose: str | None
    diff_in1: str | None
    diff_in2: str | None
    # The between-entities pair: the point operand's origin Point entity (not its pose
    # coordinate) for ops 1-3, the two Line entities themselves for ops 4-5.
    relation_a: str
    relation_b: str
    # The frame the gradient's DirectionCoordinate is stated in: the point operand's own
    # reference frame for ops 1-3, the (shared, validated-equal) direction frame for ops 4-5.
    gradient_frame: str
    target: WorldQuantity
