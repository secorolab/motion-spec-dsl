# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Classes bound to constraint and comparison grammar rules."""

from __future__ import annotations

from enum import StrEnum

from textx import get_parent_of_type

from motion_spec_dsl.classes.common import NamedNamespaceObject
from motion_spec_dsl.classes.context import (
    AngleBetweenView,
    DistanceBetweenView,
    DistanceFromView,
    ProjectionOnView,
    QuantityType,
)


class ConstraintSpecification(NamedNamespaceObject):
    """A view compared against a reference, satisfied within its own band or the kind's default."""

    def __init__(self, parent, disabled, name, view, expr, tolerance) -> None:
        super().__init__(parent=parent, name=name)
        self.disabled = disabled
        self.view = view
        self.expr = expr
        self.tolerance = tolerance


class GoalStatusConstraint(NamedNamespaceObject):
    """An until item met when a detect act's goal reaches a status; it has no view or band."""

    def __init__(self, parent, name, act, status) -> None:
        super().__init__(parent=parent, name=name)
        self.act = act
        self.status = status
        self.status_constant = f"STATUS_{status.upper()}"
        self.disabled = False
        self.view = None
        self.expr = None
        self.tolerance = None


class ConstraintGroup(NamedNamespaceObject):
    """Until constraints monitored as one condition, so a motion can have several transitions."""

    def __init__(self, parent, name, logic, constraints) -> None:
        super().__init__(parent=parent, name=name)
        self.logic = logic
        self.constraints = constraints


class ConstraintRef:
    """A reference to a constraint declared within a motion, through any aliases."""

    def __init__(self, parent, target) -> None:
        self.parent = parent
        self.target = target
        # Only grammar attributes here: textX may initialize the aliases after this ref.
        constraint = target
        while isinstance(constraint, ConstraintAlias):
            constraint = constraint.ref.target
        self.constraint = constraint
        self.name = target.name or constraint.name

    @property
    def motion(self):
        motion = get_parent_of_type("GuardedMotion", self.target)
        assert motion is not None
        return motion

    def __str__(self) -> str:
        return f"{self.motion.name}.{self.name}"


class ConstraintAlias(NamedNamespaceObject):
    """A local name in a section for a constraint of another motion."""

    def __init__(self, parent, name, ref) -> None:
        constraint = ref.target
        while isinstance(constraint, ConstraintAlias):
            constraint = constraint.ref.target
        super().__init__(parent=parent, name=name or constraint.name)
        self.ref = ref
        self.constraint = constraint


def flatten_constraint_items(items) -> list:
    """ITEMS with until groups expanded into their members."""
    out = []
    for item in items:
        if isinstance(item, ConstraintGroup):
            out.extend(item.constraints)
        else:
            out.append(item)
    return out


class ViewForm(StrEnum):
    DistanceBetween = "distance-between"
    DistanceFrom = "distance-from"
    ProjectionOn = "projection-on"
    Alignment = "alignment"
    IncidentAngle = "incident-angle"
    PlaneAngle = "plane-angle"
    Norm = "norm"


BINARY_VIEW_FORMS = {
    DistanceBetweenView: ViewForm.DistanceBetween,
    DistanceFromView: ViewForm.DistanceFrom,
    ProjectionOnView: ViewForm.ProjectionOn,
}

ANGLE_VIEW_FORMS = {ViewForm.Alignment, ViewForm.IncidentAngle, ViewForm.PlaneAngle}


def view_form(constraint) -> ViewForm | None:
    """The form of a constraint's view; None for a plain quantity or expression view.

    An angle from a plane to a direction has no form: validation rejects that operand order.
    """
    view = constraint.view
    if view is None:
        return None
    if view.norm is not None:
        return ViewForm.Norm
    binary = view.binary
    if not isinstance(binary, AngleBetweenView):
        return BINARY_VIEW_FORMS.get(type(binary))
    left_plane = binary.left.type == QuantityType.Plane
    right_plane = binary.right.type == QuantityType.Plane
    if left_plane and right_plane:
        return ViewForm.PlaneAngle
    if right_plane:
        return ViewForm.IncidentAngle
    if not left_plane:
        return ViewForm.Alignment
    return None


class EqualityConstraint:
    def __init__(self, parent, reference) -> None:
        self.parent = parent
        self.reference = reference


class GreaterThanConstraint:
    def __init__(self, parent, threshold) -> None:
        self.parent = parent
        self.threshold = threshold


class LessThanConstraint:
    def __init__(self, parent, threshold) -> None:
        self.parent = parent
        self.threshold = threshold


class BilateralConstraint:
    """Satisfied within [lower, upper]."""

    def __init__(self, parent, lower, upper) -> None:
        self.parent = parent
        self.lower = lower
        self.upper = upper


class OutsideConstraint:
    """Satisfied outside [lower, upper]: a value leaving a band."""

    def __init__(self, parent, lower, upper) -> None:
        self.parent = parent
        self.lower = lower
        self.upper = upper
