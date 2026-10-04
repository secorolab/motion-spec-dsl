# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Node ids and plan records shared by the motion-spec RDF emitters."""

from __future__ import annotations

from dataclasses import dataclass

from rdflib.term import URIRef

from motion_spec_dsl.classes.constraints import (
    ANGLE_VIEW_FORMS,
    BilateralConstraint,
    ConstraintSpecification,
    EqualityConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    OutsideConstraint,
    ViewForm,
    view_form,
)
from motion_spec_dsl.classes.context import (
    ContextQuantity,
    ContextQuantityAlias,
    WorldQuantity,
    WorldQuantityType,
)
from motion_spec_dsl.classes.controller_semantics import (
    alignment_is_pointwise,
    constraint_view_subspace,
)
from motion_spec_dsl.classes.motion_spec import ConstraintSection

AXIS_VECTORS = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}

JOINT_TYPES = {
    WorldQuantityType.JointPosition,
    WorldQuantityType.JointVelocity,
    WorldQuantityType.JointCurrent,
}


def scalar_id(quantity: WorldQuantity, subspace: str, axis: str | None) -> str:
    """The id stem of a scalar view: `<name>.<subspace>[.<axis>]`, the bare name for a joint."""
    if quantity.type in JOINT_TYPES:
        return quantity.name
    if axis is None:
        return f"{quantity.name}.{subspace}"
    return f"{quantity.name}.{subspace}.{axis}"


def alignment_id(quantity: WorldQuantity, constraint: ConstraintSpecification) -> str:
    """The scalar id of an angle view: its carrier pose, both operands, and what it is computed for.

    Two alignments can share a pose and mean different angles; a nonzero target or a cone gets
    its own op chain, and a bound and a target at one value drive different rows.
    """
    binary = constraint.view.binary
    operands = [
        operand.ref if isinstance(operand, ContextQuantityAlias) else operand
        for operand in (binary.left, binary.right)
    ]
    stem = f"alignment-{operands[0].name}-{operands[1].name}"
    expr = constraint.expr
    if isinstance(expr, EqualityConstraint):
        refs = [expr.reference]
    elif isinstance(expr, (GreaterThanConstraint, LessThanConstraint)):
        refs = [expr.threshold]
    elif isinstance(expr, (BilateralConstraint, OutsideConstraint)):
        refs = [expr.lower, expr.upper]
    else:
        refs = []
    for ref in refs:
        # Every zero target means the same geometry, so a zero does not fragment the chain.
        if ref.bare is not None:
            token = None if ref.bare.value == 0.0 else f"{ref.bare.value}{ref.bare.unit}"
        elif isinstance(ref.quantity, ContextQuantityAlias):
            token = ref.quantity.ref.name
        elif isinstance(ref.quantity, ContextQuantity):
            token = ref.quantity.name
        else:
            token = None
        if token is not None:
            stem = f"{stem}-{token}"
    if not alignment_is_pointwise(constraint):
        stem = f"{stem}-cone"
    return scalar_id(quantity, stem, None)


def norm_id(quantity: WorldQuantity, subspace: str, across: str | None) -> str:
    """The id of a norm scalar, qualified by the direction it is taken across."""
    stem = f"{scalar_id(quantity, subspace, None)}.norm"
    return stem if across is None else f"{stem}-across-{across}"


def constraint_scalar_id(quantity: WorldQuantity, constraint: ConstraintSpecification) -> str:
    """The id of the scalar a plain or norm view reads; angle views are the caller's."""
    subspace = constraint_view_subspace(constraint)
    view = constraint.view
    if view_form(constraint) == ViewForm.Norm:
        across = view.norm.across.quantity if view.norm.across is not None else None
        if isinstance(across, ContextQuantityAlias):
            across = across.ref
        return norm_id(quantity, subspace, across.name if across is not None else None)
    return scalar_id(quantity, subspace, str(view.axis) if view.axis is not None else None)


def gradient_scalar_id(quantity: WorldQuantity, constraint: ConstraintSpecification) -> str | None:
    """The id of the gradient direction a view publishes; None for an axis, distance or point target."""
    form = view_form(constraint)
    if form == ViewForm.Alignment and alignment_is_pointwise(constraint):
        return None
    if form in ANGLE_VIEW_FORMS:
        return f"{alignment_id(quantity, constraint)}-gradient"
    if form in (ViewForm.DistanceFrom, ViewForm.ProjectionOn):
        return f"{scalar_id(quantity, constraint_view_subspace(constraint), None)}-gradient"
    return None


def evaluator_id(spec: ConstraintSpecification) -> str:
    """The id of a constraint's monitor evaluator, qualified by motion and section."""
    section = spec.parent
    while section is not None and not isinstance(section, ConstraintSection):
        section = section.parent
    if section is None:
        return f"eval-{spec.name}"
    return f"eval-{section.parent.name}-{section.kind}-{spec.name}"


@dataclass(frozen=True)
class DistancePlan:
    """A distance relation's pose endpoints, carrier, and the origin Points it is between."""

    start: WorldQuantity
    end: WorldQuantity
    target: WorldQuantity
    relation_a: str
    relation_b: str


@dataclass(frozen=True)
class AlignmentPlan:
    """An angle's directions (a plane by its normal), carrier pose, and the operands it is between."""

    moving: ContextQuantity
    reference: ContextQuantity
    target: WorldQuantity
    relation_a: str
    relation_b: str


@dataclass(frozen=True)
class GeometricDistancePlan:
    """A Table II expression's operator, operands and carrier.

    A point operator carries a `direction`; a line-line one a `pose` difference of its two
    origin poses `diff_in1`/`diff_in2`.
    """

    op_type: str
    in1: str
    in2: str
    direction: str | None
    pose: str | None
    diff_in1: str | None
    diff_in2: str | None
    # The point's origin Point (ops 1-3), or the two lines (ops 4-5).
    relation_a: str
    relation_b: str
    # The point's reference frame (ops 1-3), or the shared direction frame (ops 4-5).
    gradient_frame: str
    target: WorldQuantity


@dataclass(frozen=True)
class ExpressionPlan:
    """A controlled expression's unit gradient and angular companion, the norm both were divided by,
    and the frame they are stated in."""

    gradient: URIRef
    moment: URIRef | None
    norm: URIRef
    frame: URIRef
