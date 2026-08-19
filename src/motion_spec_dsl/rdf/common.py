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
from motion_spec_dsl.classes.controller_semantics import constraint_view_subspace
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


def _is_distance_view(constraint: ConstraintSpecification) -> bool:
    """Whether the constraint's view is a `distance between A and B` form."""
    return (
        getattr(constraint.view, "distance_from", None) is not None
        and getattr(constraint.view, "distance_to", None) is not None
    )


def _is_alignment_view(constraint: ConstraintSpecification) -> bool:
    """Whether the constraint's view is an `angle between A and B` form."""
    return (
        getattr(constraint.view, "angle_from", None) is not None
        and getattr(constraint.view, "angle_to", None) is not None
    )


def _is_geometric_distance_view(constraint: ConstraintSpecification) -> bool:
    """Whether the constraint's view is a `distance of A from B` (Table IIa) form."""
    return (
        getattr(constraint.view, "distance_of", None) is not None
        and getattr(constraint.view, "distance_from_primitive", None) is not None
    )


def _is_projection_view(constraint: ConstraintSpecification) -> bool:
    """Whether the constraint's view is a `projection of A on B` (Table IIa) form."""
    return (
        getattr(constraint.view, "projection_of", None) is not None
        and getattr(constraint.view, "projection_on", None) is not None
    )


def _alignment_id(quantity: WorldQuantity, constraint: ConstraintSpecification) -> str:
    """Scalar id of an `angle between` view: the carrier pose and both direction operands. The
    operands belong in it because two alignments can share one pose and mean different angles.
    """
    moving = _resolved_context_quantity(constraint.view.angle_from).name
    reference = _resolved_context_quantity(constraint.view.angle_to).name
    stem = f"alignment-{moving}-{reference}"
    # The tolerated cone is part of what the angle is computed for, so two motions that tolerate
    # different cones get their own op chain rather than one that can only answer for one of them.
    upper = getattr(constraint.expr, "upper", None)
    band = _resolved_context_quantity(_context_quantity(upper)) if upper is not None else None
    if band is not None:
        stem = f"{stem}-{band.name}"
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
        if subspace == "alignment":
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
    """Resolved endpoints and scalar-view carrier for an authored distance relation."""

    start: WorldQuantity
    end: WorldQuantity
    target: WorldQuantity


@dataclass(frozen=True)
class _AlignmentPlan:
    """Resolved direction operands and pose carrier for an authored `angle between` view."""

    moving: ContextQuantity
    reference: ContextQuantity
    target: WorldQuantity


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
    relation_a: str
    relation_b: str
    # The frame the gradient's DirectionCoordinate is stated in: the point operand's own
    # reference frame for ops 1-3, the (shared, validated-equal) direction frame for ops 4-5.
    gradient_frame: str
    target: WorldQuantity
