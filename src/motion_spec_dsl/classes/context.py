# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Classes bound to world and motion-context grammar rules."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from typing import TypeVar

from rdflib.namespace import Namespace

from motion_spec_dsl.classes.common import NamedNamespaceObject
from motion_spec_dsl.classes.coordinates import (
    AccelerationTwistCoordinate,
    Coordinates,
    PoseCoordinate,
    VelocityTwistCoordinate,
    WrenchCoordinate,
    const_value,
)
from motion_spec_dsl.classes.path import AdmittanceSpec, PathValue, ProfileSpec

_EnumT = TypeVar("_EnumT", bound=StrEnum)


class WorldQuantityType(StrEnum):
    Pose = "Pose"
    VelocityTwist = "VelocityTwist"
    Wrench = "Wrench"
    JointPosition = "JointPosition"


def _authored_enum(enum_type: type[_EnumT], keyword: str) -> _EnumT:
    """Translate a kebab-case DSL keyword to its established enum member."""
    normalized = keyword.replace("-", "").casefold()
    for member in enum_type:
        if member.value.replace("-", "").casefold() == normalized:
            return member
    raise ValueError(f"'{keyword}' is not a valid {enum_type.__name__}")


@dataclass(eq=False)
class WorldQuantity(NamedNamespaceObject):
    """A physical world quantity (pose/twist/wrench/joint-position/...) with geometric props."""

    parent: object
    type: WorldQuantityType
    name: str = ""
    props: GeometricProps | None = field(default=None, kw_only=True)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
        raw_type = str(self.type)
        self.type = _authored_enum(WorldQuantityType, raw_type)


@dataclass(kw_only=True)
class WorldQuantityAlias(WorldQuantity):
    """An alias referring to a WorldQuantity."""

    parent: object
    name: str
    ref: WorldQuantity
    type: WorldQuantityType = field(init=False)
    props: GeometricProps | None = field(init=False, default=None)

    def __post_init__(self):
        if not self.name:
            self.name = self.ref.name
        NamedNamespaceObject.__init__(self, parent=self.parent, name=self.name)
        self._uri = self.ref.uri
        self.type = self.ref.type
        self.props = self.ref.props


@dataclass(eq=False)
class GeometricProps:
    pairs: list[GeoPropPair]
    parent: object | None = field(default=None, repr=False, compare=False)


class GeometricPropKey(StrEnum):
    Of = "of"
    Wrt = "wrt"
    RefPoint = "ref-point"
    AsSeenBy = "as-seen-by"
    Joint = "joint"
    FtSensor = "ft-sensor"
    ReTareOn = "re-tare-on"
    Normalization = "normalization"
    Normal = "normal"
    Along = "along"


@dataclass
class GeoPropPair:
    """A geometric-property key/value pair (of/wrt/as-seen-by/ref-point/...)."""

    key: GeometricPropKey
    value: object | None = None
    frame: object | None = None
    joint: object | None = None
    sensor: object | None = None
    quantity: object | None = None
    events: list = field(default_factory=list)
    normalization: object | None = None
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        self.key = GeometricPropKey(self.key)
        if self.value is None:
            self.value = (
                self.frame or self.joint or self.sensor or self.quantity or self.normalization
            )


class QuantityType(StrEnum):
    Pose = "Pose"
    Position = "Position"
    Orientation = "Orientation"
    VelocityTwist = "VelocityTwist"
    AccelerationTwist = "AccelerationTwist"
    Wrench = "Wrench"
    Direction = "Direction"
    Line = "Line"
    Plane = "Plane"
    Length = "Length"
    Distance = "Distance"
    Angle = "Angle"
    PlaneAngle = "PlaneAngle"
    LinearVelocity = "LinearVelocity"
    AngularVelocity = "AngularVelocity"
    LinearAcceleration = "LinearAcceleration"
    AngularAcceleration = "AngularAcceleration"
    LinearJerk = "LinearJerk"
    Force = "Force"
    Torque = "Torque"
    Mass = "Mass"
    FreeVector = "FreeVector"
    Dimensionless = "Dimensionless"
    Duration = "Duration"
    PathParameter = "PathParameter"


class ReferenceGeneratorType(StrEnum):
    Path = "Path"
    VelocityProfile = "VelocityProfile"
    Admittance = "Admittance"


ContextDeclarationType = QuantityType | ReferenceGeneratorType


@dataclass(eq=False)
class ContextQuantity(NamedNamespaceObject):
    """A context quantity: a reference, snapshot, profile, path, or literal value."""

    parent: object
    name: str
    type: ContextDeclarationType
    value: (
        Measure
        | VectorXYZ
        | ReferenceValue
        | SnapshotValue
        | PathValue
        | ProfileSpec
        | AdmittanceSpec
        | PoseCoordinate
        | VelocityTwistCoordinate
        | AccelerationTwistCoordinate
        | WrenchCoordinate
        | ConfigValue
        | DirectionBetween
        | None
    ) = None
    props: GeometricProps | None = field(default=None, kw_only=True)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
        raw_type = str(self.type)
        try:
            self.type = _authored_enum(ReferenceGeneratorType, raw_type)
        except ValueError:
            self.type = _authored_enum(QuantityType, raw_type)


@dataclass(eq=False, kw_only=True)
class ContextQuantityAlias(ContextQuantity):
    """An alias referring to a ContextQuantity."""

    parent: object
    name: str
    ref: ContextQuantity
    type: ContextDeclarationType = field(init=False)
    value: object | None = field(init=False, default=None)

    def __post_init__(self):
        if not self.name:
            self.name = self.ref.name
        NamedNamespaceObject.__init__(self, parent=self.parent, name=self.name)
        self._uri = self.ref.uri
        self.type = self.ref.type
        self.value = self.ref.value


class ContextPath(ContextQuantity):
    """A path reference generator declared in a context block."""

    def __init__(self, parent: object, name: str, value: PathValue, **_):
        super().__init__(
            parent=parent,
            name=name,
            type=ReferenceGeneratorType.Path,
            value=value,
        )


@dataclass
class Measure:
    """A dimensioned scalar literal (value+unit) — `= 5.0 N`, `5.0 s`, `timestep: 1.0 ms`."""

    value: float = 0.0
    unit: str = ""
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self) -> None:
        self.value = const_value(self.value)


@dataclass
class VectorXYZ:
    coords: Coordinates | None = None
    unit: str = ""
    parent: object | None = field(default=None, repr=False, compare=False)


def _add_chain_tree(head, tail: list):
    """Normalize a `head [+/- term]*` chain into the nested op structure the emitter and
    validator share: runs of `+` collapse to one n-ary node, `-` is left-associative binary,
    and a chain with no tail collapses to `head` itself. Shared by `QExpr` and `SnapshotValue`,
    whose tails are both `QAddTail` chains over a different kind of head.
    """
    acc = head
    group: list | None = None
    for step in tail:
        operand = step.operand._term_tree()
        if step.op == "+":
            group = [acc] if group is None else group
            group.append(operand)
            acc = QOpNode("add", group)
        else:
            group = None
            acc = QOpNode("subtract", [acc, operand])
    return acc


@dataclass
class QExpr:
    """An expression over quantity refs and measures: `+`/`-` terms of `*`/`/` factors."""

    head: QTerm
    tail: list = field(default_factory=list)
    parent: object | None = field(default=None, repr=False, compare=False)

    def as_op_tree(self):
        """Normalize into the nested op structure the emitter and validator share (see
        `_add_chain_tree`); a bare leaf with no tail and no group collapses to itself.
        """
        return _add_chain_tree(self.head._term_tree(), self.tail)


@dataclass
class QAddTail:
    op: str = "+"
    operand: QTerm | None = None
    parent: object | None = field(default=None, repr=False, compare=False)


@dataclass
class QTerm:
    head: QFactor
    tail: list = field(default_factory=list)
    parent: object | None = field(default=None, repr=False, compare=False)

    def _term_tree(self):
        acc = self.head._factor_tree()
        group: list | None = None
        for step in self.tail:
            operand = step.operand._factor_tree()
            if step.op == "*":
                group = [acc] if group is None else group
                group.append(operand)
                acc = QOpNode("multiply", group)
            else:
                group = None
                acc = QOpNode("divide", [acc, operand])
        return acc


@dataclass
class QMulTail:
    op: str = "*"
    operand: QFactor | None = None
    parent: object | None = field(default=None, repr=False, compare=False)


@dataclass
class QFactor:
    neg: bool = False
    group: QExpr | None = None
    leaf: QuantityLeaf | None = None
    parent: object | None = field(default=None, repr=False, compare=False)

    def _factor_tree(self):
        inner = self.group.as_op_tree() if self.group is not None else self.leaf
        if not self.neg:
            return inner
        neg_one = QuantityLeaf(bare=Measure(value=-1.0, unit="1", parent=self), parent=self)
        return QOpNode("multiply", [neg_one, inner])


@dataclass
class QuantityLeaf:
    """A quantity-expression leaf: a world/context quantity ref, or a bare measure literal."""

    quantity: object | None = None
    bare: Measure | None = None
    subspace: SubSpace | None = None
    axis: Axis | None = None
    selector: SelectorTail | None = None
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        if self.selector is not None:
            self.subspace = self.selector.subspace
            self.axis = self.selector.axis
        if isinstance(self.subspace, str):
            self.subspace = SubSpace(self.subspace)
        if self.axis is not None and isinstance(self.axis, str):
            self.axis = Axis(self.axis)


@dataclass
class QOpNode:
    """A normalized interior node of a quantity-expression op tree."""

    op: str
    operands: list


def _leftmost_leaf(expr: QExpr):
    """The leaf nothing in `expr` combines into -- frame/metadata inheritance reads it."""
    factor = expr.head.head
    return _leftmost_leaf(factor.group) if factor.group is not None else factor.leaf


@dataclass
class ReferenceValue:
    expr: QExpr
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def source(self) -> QuantityLeaf:
        """The expression's leftmost quantity leaf; frame/metadata inheritance reads this."""
        return _leftmost_leaf(self.expr)


@dataclass
class SnapshotValue:
    """A quantity view sampled with an optional trailing expression, re-sampled on every
    occurrence of `trigger` -- the coordinator event that names when this quantity is captured.
    """

    source: View
    trigger: object
    tail: list = field(default_factory=list)
    parent: object | None = field(default=None, repr=False, compare=False)

    def as_op_tree(self):
        """Normalize `source [+/- term]*` into the nested op structure (see `_add_chain_tree`);
        `source` itself collapses to itself when `tail` is empty.
        """
        return _add_chain_tree(self.source, self.tail)


@dataclass
class ConfigValue:
    """A pose whose numbers the deployment states, read at startup from the config file the
    exec-context declares.

    `key` is the path into that file, written `[config.<key>]` so a deployment lookup never
    looks like a `<model element>` reference. `source` is the quantity it is a value for: it
    contributes no number, only the of/wrt/as-seen-by frames the target is stated in -- the same
    way a snapshot takes them from what it samples.
    """

    key: str
    source: View
    parent: object | None = field(default=None, repr=False, compare=False)


@dataclass
class DirectionBetween:
    """A direction recomputed every cycle from where two frames currently are: the unit vector
    from `from_frame`'s origin to `to_frame`'s origin, read in the quantity's `as-seen-by` frame.

    It carries no number of its own -- the pose relating the two frames does, and the runtime
    normalizes that pose's translation.
    """

    from_frame: object
    to_frame: object
    parent: object | None = field(default=None, repr=False, compare=False)


def _resolved_world_quantity(item: WorldQuantity | WorldQuantityAlias) -> WorldQuantity:
    return item.ref if isinstance(item, WorldQuantityAlias) else item


def _resolved_context_quantity(item: ContextQuantity | ContextQuantityAlias) -> ContextQuantity:
    return item.ref if isinstance(item, ContextQuantityAlias) else item


def is_body_line_distance(binary) -> bool:
    """Whether a point-from-line view's line rides the frame the point is measured against:
    the line's `of` frame equals the point pose's `wrt` frame, so the line moves with the
    measurement frame and dispatch goes to `BODY_LINE_DISTANCE_OP`.
    """
    left, right = binary.left, binary.right
    if _geometric_operand_kind(left) != "point" or _geometric_operand_kind(right) != "line":
        return False

    def prop(props, key):
        for pair in getattr(props, "pairs", ()):
            if isinstance(pair, GeoPropPair) and pair.key == key:
                value = pair.frame or pair.joint or pair.sensor or pair.value
                return str(getattr(value, "uri", value))
        return None

    point = _resolved_world_quantity(left)
    line = _resolved_context_quantity(right)
    wrt = prop(point.props, GeometricPropKey.Wrt)
    of = prop(line.props, GeometricPropKey.Of)
    return wrt is not None and wrt == of


def _geometric_operand_kind(operand: object) -> str | None:
    """Table IIa operand kind ('point'/'line'/'plane') of a distance/projection operand, or
    None when `operand` is neither a pose, a line, nor a plane."""
    if isinstance(operand, WorldQuantity):
        resolved = _resolved_world_quantity(operand)
        return "point" if resolved.type == WorldQuantityType.Pose else None
    if isinstance(operand, ContextQuantity):
        resolved = _resolved_context_quantity(operand)
        return {
            QuantityType.Pose: "point",
            QuantityType.Line: "line",
            QuantityType.Plane: "plane",
        }.get(resolved.type)
    return None


# Table IIa distance/projection operators, keyed by (first operand kind, second operand kind).
# Line-line operand order is not a dispatch key: both directions carry the same op, and which
# line is `in1`/`in2` is what makes `LineOnLineProjection` compute s1 vs s2 (see plan 08 ruling 2).
GEOMETRIC_DISTANCE_OPS: dict[tuple[str, str], str] = {
    ("point", "plane"): "PointPlaneToLinearDistance",
    ("point", "line"): "PointLineToLinearDistance",
    ("line", "line"): "LineLineToLinearDistance",
}
GEOMETRIC_PROJECTION_OPS: dict[tuple[str, str], str] = {
    ("point", "line"): "PointOnLineProjection",
    ("line", "line"): "LineOnLineProjection",
}
# The Table IIa expressions whose scalar is an unsigned magnitude (no derivative at zero).
UNSIGNED_GEOMETRIC_DISTANCE_OPS = {"PointLineToLinearDistance", "PointBodyLineToLinearDistance"}

# The point-from-line case where the line rides the frame the point is measured in (Borghesan's
# line-point distance from the grasping axis): the line's origin IS that frame, so the operator
# takes no origin pose, and its gradient carries an angular half -- tilting the axis sweeps it
# across the point.
BODY_LINE_DISTANCE_OP = "PointBodyLineToLinearDistance"

# Constraint-view subspace token per Table IIa operator, matching the lowercase style
# ("alignment", "distance") the rest of `constraint_view_subspace` already uses.
GEOMETRIC_DISTANCE_SUBSPACE: dict[str, str] = {
    "PointPlaneToLinearDistance": "point-plane-distance",
    "PointLineToLinearDistance": "point-line-distance",
    "PointBodyLineToLinearDistance": "point-body-line-distance",
    "PointOnLineProjection": "point-line-projection",
    "LineLineToLinearDistance": "line-line-distance",
    "LineOnLineProjection": "line-line-projection",
}

# geom-rel-ext relation (the entity pair) each Table IIa operator's coordinate is `of`. A
# distance op and its projection sibling over the same pair share one relation -- see
# `_linear_distance_relation`'s pair-keyed cache.
GEOMETRIC_DISTANCE_RELATION: dict[str, str] = {
    "PointPlaneToLinearDistance": "PointPlaneDistance",
    "PointLineToLinearDistance": "PointLineDistance",
    # The body-line case is the paper's collinearity constraint, and comp-rob2b already
    # declares the relation for it -- the coordinate samples that relation's distance.
    "PointBodyLineToLinearDistance": "PointLineCollinearity",
    "PointOnLineProjection": "PointLineDistance",
    "LineLineToLinearDistance": "LineLineDistance",
    "LineOnLineProjection": "LineLineDistance",
}


class SubSpace(StrEnum):
    Position = "position"
    Orientation = "orientation"
    LinVel = "linvel"
    AngVel = "angvel"
    LinAcc = "linacc"
    AngAcc = "angacc"
    Force = "force"
    Torque = "torque"


class Axis(StrEnum):
    X = "x"
    Y = "y"
    Z = "z"


@dataclass
class ElapsedTime:
    marker: bool = False
    parent: object | None = field(default=None, repr=False, compare=False)


@dataclass
class ProgressAlong:
    """How fast a moved frame is travelling along a path, measured on the path's tangent.

    A path constrains geometry but not timing, so the rate of travel along it has to be
    stated as its own constraint rather than baked into the path parameter.
    """

    moved: object
    path: object
    parent: object | None = field(default=None, repr=False, compare=False)


@dataclass
class MovingAlong:
    """A profiled drive along a path, holding the frame on the path's geometry.

    This is the driver of the motion, not a comparison, so it carries no relation: the
    profile commands the tangent speed while the directions normal to it are held at zero.
    """

    moved: object
    path: object
    profile: object
    parent: object | None = field(default=None, repr=False, compare=False)


@dataclass
class OnPath:
    """A moved frame held on a path's geometry, with no timing along it.

    The path is the reference, evaluated at the frame's own closest-point projection, so
    this constrains only the directions normal to the path -- the tangent is left to the
    driver that commands a speed along it.
    """

    moved: object
    path: object
    selector: "SelectorTail | None" = None
    parent: object | None = field(default=None, repr=False, compare=False)


@dataclass
class SelectorTail:
    """The subspace/axis selector applied to a quantity in a view or reference."""

    subspace: SubSpace
    axis: Axis | None = None
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        if isinstance(self.subspace, str):
            self.subspace = SubSpace(self.subspace)
        if self.axis is not None and isinstance(self.axis, str):
            self.axis = Axis(self.axis)


@dataclass
class View:
    """A quantity selector, distance relation, or elapsed-time constraint operand."""

    parent: object
    quantity: WorldQuantity | None = None
    expr: QExpr | None = None
    subspace: SubSpace | None = None
    axis: Axis | None = None
    selector: SelectorTail | None = None
    binary: object | None = None
    elapsed: ElapsedTime | None = None
    progress: ProgressAlong | None = None
    moving: MovingAlong | None = None
    on: OnPath | None = None

    def __post_init__(self):
        # An on-path view still selects a subspace of a world quantity, so it fills the same
        # slots every other view does and needs no special case downstream.
        if self.on is not None:
            self.quantity = self.on.moved
            self.selector = self.on.selector
        # Driving along a path and guarding progress along it both act on the one speed
        # measured along the tangent, so they command in the linear-velocity subspace.
        if self.moving is not None or self.progress is not None:
            self.subspace = SubSpace.LinVel
        if self.selector is not None:
            self.subspace = self.selector.subspace
            self.axis = self.selector.axis
        if isinstance(self.subspace, str):
            self.subspace = SubSpace(self.subspace)
        if self.axis is not None and isinstance(self.axis, str):
            self.axis = Axis(self.axis)

    @property
    def is_elapsed(self) -> bool:
        return self.elapsed is not None


@dataclass
class ContextRef:
    """A reference to a context value, optionally with a subspace/axis or a bare literal."""

    quantity: ContextQuantity | None = None
    bare: Measure | None = None
    expr: QExpr | None = None
    subspace: SubSpace | None = None
    axis: Axis | None = None
    selector: SelectorTail | None = None
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        if self.selector is not None:
            self.subspace = self.selector.subspace
            self.axis = self.selector.axis
        if isinstance(self.subspace, str):
            self.subspace = SubSpace(self.subspace)
        if self.axis is not None and isinstance(self.axis, str):
            self.axis = Axis(self.axis)

    @property
    def name(self) -> str:
        return "ref"

    @property
    def namespace(self) -> Namespace:
        current = self.parent
        while current is not None:
            namespace = getattr(current, "namespace", None)
            name = getattr(current, "name", None)
            if namespace is not None and name is not None:
                return Namespace(str(namespace) + f"{name}/")
            current = getattr(current, "parent", None)
        raise AttributeError("ContextRef namespace is not resolved yet")

    @property
    def value(self) -> ContextQuantity:
        assert self.quantity is not None, "ContextRef quantity is not resolved yet"
        return self.quantity
