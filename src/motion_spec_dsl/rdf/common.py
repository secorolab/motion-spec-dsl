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
    _binary_view,
    _is_alignment_view,
    _is_geometric_distance_view,
    _is_incident_angle_view,
    _is_norm_view,
    _is_plane_angle_view,
    _is_projection_view,
)
from motion_spec_dsl.classes.context import (
    _NORM_SCALAR_TYPES,
    ContextQuantity,
    WorldQuantity,
    WorldQuantityType,
    _context_quantity,
    _resolved_context_quantity,
    _scalar_type,
)
from motion_spec_dsl.classes.controller_semantics import (
    _alignment_is_pointwise,
    axis_label as semantic_axis_label,
    constraint_view_subspace,
)
from motion_spec_dsl.classes.coordinates import const_value
from motion_spec_dsl.classes.units import (
    ANGLE_UNITS as ANGLE_UNITS,
)
from motion_spec_dsl.classes.units import (
    _angle_unit as _angle_unit,
)
from motion_spec_dsl.classes.units import (
    _dsl_unit as _dsl_unit,
)


def _ns_term(namespace: Any, name: str) -> URIRef:
    """URI for `name` in `namespace` (the namespace's base IRI concatenated with `name`)."""
    return URIRef(str(namespace._NS) + name)


def _angle_bound(bound) -> float:
    """An authored angle bound as a number: any constant expression over literals and pi."""
    return const_value(getattr(bound, "value", bound))


def _alignment_bound_token(ref: Any) -> str | None:
    """Id fragment for one target/bound ref of an alignment's relation, or None for a literal
    zero -- every zero-target alignment means the same geometry, so a zero must not fragment the
    shared op chain.
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


def _scalar_id(quantity: WorldQuantity, subspace: str, axis: str | None) -> str:
    """Id stem for a scalar view of `quantity`: `<name>.<subspace>[.<axis>]`
    (bare `<name>` for joint positions)."""
    if quantity.type in (
        WorldQuantityType.JointPosition,
        WorldQuantityType.JointVelocity,
        WorldQuantityType.JointCurrent,
    ):
        return quantity.name
    if axis is None:
        return f"{quantity.name}.{subspace}"
    return f"{quantity.name}.{subspace}.{axis}"


def _norm_id(quantity: WorldQuantity, subspace: str, across: str | None) -> str:
    """Id of the norm scalar of a `quantity.subspace` vector view, qualified by the direction
    it is taken across when there is one."""
    stem = f"{_scalar_id(quantity, subspace, None)}.norm"
    return stem if across is None else f"{stem}-across-{across}"


def _constraint_scalar_id(quantity: WorldQuantity, constraint: ConstraintSpecification) -> str:
    """The scalar id a plain or norm view resolves to; alignment/angle views are the caller's."""
    subspace = constraint_view_subspace(constraint)
    if _is_norm_view(constraint):
        # Resolve through an alias so this id matches the one the emitter builds.
        across = _resolved_context_quantity(_context_quantity(constraint.view.norm.across))
        return _norm_id(quantity, subspace, getattr(across, "name", None))
    return _scalar_id(quantity, subspace, semantic_axis_label(constraint.view.axis))


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
        base = _scalar_id(quantity, constraint_view_subspace(constraint), None)
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


def _constraint_scalar_type(quantity: WorldQuantity, constraint: ConstraintSpecification) -> Any:
    """The scalar kind a plain or norm view resolves to."""
    subspace = constraint_view_subspace(constraint)
    if _is_norm_view(constraint):
        return _NORM_SCALAR_TYPES[_scalar_type(quantity, subspace, None)]
    return _scalar_type(quantity, subspace, semantic_axis_label(constraint.view.axis))


def _owning_section(spec: ConstraintSpecification):
    """The when/while/until section a constraint belongs to, through a named until group."""
    section = getattr(spec, "parent", None)
    while section is not None and not getattr(section, "kind", ""):
        section = getattr(section, "parent", None)
    return section


def _owning_motion(spec: ConstraintSpecification):
    """The guarded motion a constraint belongs to; for one in no section (a derived scalar's
    shim), the context that declares it."""
    section = _owning_section(spec)
    if section is None:
        return getattr(getattr(spec, "parent", None), "parent", None)
    return getattr(section, "parent", None)


def _evaluator_id(spec: ConstraintSpecification) -> str:
    """Stable id for a constraint's monitor evaluator, qualified by motion + section kind."""
    section = _owning_section(spec)
    motion = _owning_motion(spec)
    section_kind = getattr(section, "kind", None)
    motion_name = getattr(motion, "name", None)
    if motion_name and section_kind:
        return f"eval-{motion_name}-{section_kind}-{spec.name}"
    return f"eval-{spec.name}"


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
    view: a point-plane/point-line/point-on-line expression, or a line-line one.

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
