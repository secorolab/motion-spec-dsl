# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Pure helpers and plan records shared by motion-spec RDF emitters."""

from __future__ import annotations

import math

from dataclasses import dataclass
from typing import Any

from rdflib.term import URIRef


from motion_spec_dsl.classes.controller_semantics import constraint_view_subspace
from motion_spec_dsl.classes.constraints import (
    ConstraintSpecification,
    GoalStatusConstraint,
    _flatten_constraint_items,
    _resolved_spec,
)
from motion_spec_dsl.classes.context import (
    ContextRef,
    GeoPropPair,
    GeometricProps,
    QuantityType,
    ContextQuantity,
    WorldQuantity,
    WorldQuantityType,
)
from motion_spec_dsl.classes.motion_spec import GuardedMotion

from motion_spec_dsl.rdf.model import WORLD_SPECS
from motion_spec_dsl.classes.units import (
    ANGLE_UNITS as ANGLE_UNITS,
    _angle_unit as _angle_unit,
    _dsl_unit as _dsl_unit,
)


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
    """An authored angle bound as a number: a literal, or a multiple of pi."""
    term = getattr(bound, "pi", None)
    if term is None:
        return float(getattr(bound, "value", bound))
    multiplier = getattr(term, "multiplier", None)
    magnitude = math.pi * (float(multiplier) if multiplier not in (None, "") else 1.0)
    return -magnitude if getattr(term, "negative", False) else magnitude


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
        if subspace == "distance":
            return QuantityType.Distance
        if subspace == "rotation":
            return QuantityType.PlaneAngle
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
