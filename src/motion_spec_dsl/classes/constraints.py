# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Classes bound to constraint and comparison grammar rules."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from textx import get_parent_of_type

from motion_spec_dsl.classes.common import NamedNamespaceObject
from motion_spec_dsl.classes.context import ContextRef, View

if TYPE_CHECKING:
    from motion_spec_dsl.classes.motion_spec import GuardedMotion


@dataclass(eq=False)
class ConstraintSpecification(NamedNamespaceObject):
    """A single constraint: a view (LHS) compared against a reference by an expression,
    satisfied within a band.

    The band belongs to the constraint rather than to the comparison: an equality needs one
    because its target is a single point, and a one-sided gate needs one to say how close to
    its threshold counts as arrived. Unstated, it falls back to the model-wide default for
    the kind the error carries.
    """

    parent: object
    name: str
    view: View | None = None
    expr: (
        EqualityConstraint
        | GreaterThanConstraint
        | LessThanConstraint
        | BilateralConstraint
        | OutsideConstraint
        | None
    ) = None
    tolerance: ContextRef | None = None
    disabled: bool = False

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


@dataclass(eq=False)
class GoalStatusConstraint(NamedNamespaceObject):
    """An until item met when a detect act's goal reaches the status it names.

    It compares no world quantity, so it carries neither a view nor a band: the status is a
    fact of the goal, not a measurement.
    """

    parent: object
    name: str
    act: object
    status: str = ""
    view: None = None
    expr: None = None
    tolerance: None = None
    disabled: bool = False

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)

    @property
    def status_constant(self) -> str:
        """The action_msgs GoalStatus constant this compares against."""
        return f"STATUS_{self.status.upper()}"


@dataclass(eq=False)
class ConstraintGroup(NamedNamespaceObject):
    """A named set of until constraints evaluated as one condition, so a motion can carry
    several independent transitions -- each group is monitored on its own.
    """

    parent: object
    name: str
    logic: str = "all"
    constraints: list = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


@dataclass
class ConstraintRef:
    """A reference to a constraint declared within a motion."""

    target: ConstraintSpecification | ConstraintGroup | ConstraintAlias
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def motion(self) -> GuardedMotion:
        motion = get_parent_of_type("GuardedMotion", self.target)
        assert motion is not None
        return motion

    @property
    def constraint(self) -> ConstraintSpecification | ConstraintGroup:
        return (
            self.target.ref.constraint if isinstance(self.target, ConstraintAlias) else self.target
        )

    @property
    def motion_name(self) -> str:
        return self.motion.name

    @property
    def name(self) -> str:
        return self.target.name

    def __str__(self) -> str:
        return f"{self.motion.name}.{self.target.name}"


@dataclass
class ConstraintAlias(NamedNamespaceObject):
    """Local name in a section that references a constraint from another motion."""

    parent: object
    name: str
    ref: ConstraintRef

    def __post_init__(self):
        if not self.name:
            self.name = self.ref.constraint.name
        super().__init__(parent=self.parent, name=self.name)

    @property
    def constraint(self) -> ConstraintSpecification:
        return self.ref.constraint


def _flatten_constraint_items(items) -> list:
    """Expand until groups into their member items; everything else passes through.

    Callers that want the individual constraints -- validation, view emission, evaluators --
    should not have to know whether a motion grouped them.
    """
    out = []
    for item in items:
        if isinstance(item, ConstraintGroup):
            out.extend(item.constraints)
        else:
            out.append(item)
    return out


def _resolved_spec(item: ConstraintSpecification | ConstraintAlias) -> ConstraintSpecification:
    """Return the underlying ConstraintSpecification, resolving aliases."""
    return item.ref.constraint if isinstance(item, ConstraintAlias) else item


@dataclass
class EqualityConstraint:
    """An equality constraint against a reference value."""

    reference: ContextRef
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        assert self.reference is not None


@dataclass
class GreaterThanConstraint:
    """A greater-than comparison against a threshold."""

    threshold: ContextRef
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        assert self.threshold is not None


@dataclass
class LessThanConstraint:
    """A less-than comparison against a threshold."""

    threshold: ContextRef
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        assert self.threshold is not None


@dataclass
class BilateralConstraint:
    """A within-bounds (lower..upper) constraint."""

    lower: ContextRef
    upper: ContextRef
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        assert self.lower is not None
        assert self.upper is not None


@dataclass
class OutsideConstraint:
    """Satisfied when the quantity is outside [lower, upper] (the complement of
    BilateralConstraint's in-band). Used to detect a value leaving a ±band."""

    lower: ContextRef
    upper: ContextRef
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        assert self.lower is not None
        assert self.upper is not None
