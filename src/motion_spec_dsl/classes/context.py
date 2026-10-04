# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Classes bound to world and motion-context grammar rules."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import TypeVar

from rdflib.namespace import Namespace

from motion_spec_dsl.classes.common import IHasNamespaceDeclare, NamedNamespaceObject
from motion_spec_dsl.classes.coordinates import const_value
from motion_spec_dsl.classes.path import PathValue

EnumT = TypeVar("EnumT", bound=StrEnum)


class WorldQuantityType(StrEnum):
    Pose = "Pose"
    VelocityTwist = "VelocityTwist"
    Wrench = "Wrench"
    JointPosition = "JointPosition"
    JointVelocity = "JointVelocity"
    JointCurrent = "JointCurrent"


# DSL keywords whose established enum spelling is not their kebab-case form.
_AUTHORED_KEYWORD_ALIASES = {"current": "ElectricCurrent"}


def authored_enum(enum_type: type[EnumT], keyword: str) -> EnumT:
    """The enum member a kebab-case DSL keyword names."""
    keyword = _AUTHORED_KEYWORD_ALIASES.get(keyword, keyword)
    normalized = keyword.replace("-", "").casefold()
    for member in enum_type:
        if member.value.replace("-", "").casefold() == normalized:
            return member
    raise ValueError(f"'{keyword}' is not a valid {enum_type.__name__}")


class WorldQuantity(NamedNamespaceObject):
    """A physical world quantity (pose, twist, wrench, joint state) with geometric props."""

    def __init__(self, parent, type, name, props=None) -> None:
        super().__init__(parent=parent, name=name)
        self.type = authored_enum(WorldQuantityType, str(type))
        self.props = props


class WorldQuantityAlias(WorldQuantity):
    """An alias naming a world quantity declared elsewhere."""

    def __init__(self, parent, name, ref) -> None:
        NamedNamespaceObject.__init__(self, parent=parent, name=name or ref.name)
        self.ref = ref
        self._uri = ref.uri
        self.type = ref.type
        self.props = ref.props


class GeometricProps:
    def __init__(self, parent, pairs) -> None:
        self.parent = parent
        self.pairs = pairs


class GeometricPropKey(StrEnum):
    Of = "of"
    Wrt = "wrt"
    RefPoint = "ref-point"
    AsSeenBy = "as-seen-by"
    Joint = "joint"
    FtSensor = "ft-sensor"
    EstimatedFrom = "estimated-from"
    ReTareOn = "re-tare-on"
    Normalization = "normalization"
    Normal = "normal"
    Along = "along"


class ObserverSpec:
    """Tuning of a momentum observer: the residual's cut-off and the joint-torque low-pass."""

    def __init__(self, parent, gain, gain_unit, filter) -> None:
        self.parent = parent
        self.gain = gain
        self.gain_unit = gain_unit
        self.filter = filter


class GeoPropPair:
    """One geometric property: of, wrt, as-seen-by, ref-point, joint, sensor, and so on.

    `re-tare-on` re-takes a wrench's tare on each named event: the load below the sensor changes
    when the robot picks something up, and the startup bias would report it as an external push.
    """

    def __init__(
        self,
        parent,
        key,
        frame=None,
        joint=None,
        sensor=None,
        agent=None,
        observer=None,
        events=None,
        normalization=None,
        quantity=None,
    ) -> None:
        self.parent = parent
        self.key = GeometricPropKey(key)
        self.frame = frame
        self.joint = joint
        self.sensor = sensor
        self.agent = agent
        self.observer = observer
        self.events = events or []
        self.normalization = normalization
        self.quantity = quantity
        self.value = frame or joint or sensor or agent or quantity or normalization


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
    ElectricCurrent = "ElectricCurrent"
    FreeVector = "FreeVector"
    Dimensionless = "Dimensionless"
    Duration = "Duration"
    PathParameter = "PathParameter"


class ReferenceGeneratorType(StrEnum):
    Path = "Path"
    VelocityProfile = "VelocityProfile"
    Admittance = "Admittance"


ContextDeclarationType = QuantityType | ReferenceGeneratorType


class ContextQuantity(NamedNamespaceObject):
    """A context quantity: a reference, snapshot, profile, path, or literal value."""

    def __init__(self, parent, type, name, value=None, props=None) -> None:
        super().__init__(parent=parent, name=name)
        try:
            self.type = authored_enum(ReferenceGeneratorType, str(type))
        except ValueError:
            self.type = authored_enum(QuantityType, str(type))
        self.value = value
        self.props = props


class ContextQuantityAlias(ContextQuantity):
    """An alias naming a context quantity declared elsewhere."""

    def __init__(self, parent, name, ref) -> None:
        NamedNamespaceObject.__init__(self, parent=parent, name=name or ref.name)
        self.ref = ref
        self._uri = ref.uri
        self.type = ref.type
        self.value = ref.value
        self.props = ref.props


class ContextPath(ContextQuantity):
    """A path reference generator declared in a context block."""

    def __init__(self, parent, name, value) -> None:
        super().__init__(parent, ReferenceGeneratorType.Path, name, value)


class Measure:
    """A dimensioned scalar literal: `= 5.0 N`, `timestep: 1.0 ms`."""

    def __init__(self, parent, value, unit) -> None:
        self.parent = parent
        self.value = const_value(value)
        self.unit = unit


class SampledValue:
    """A scalar drawn per generation from an imported scene distribution."""

    def __init__(self, parent, distribution, unit) -> None:
        self.parent = parent
        self.distribution = distribution
        self.unit = unit


class VectorXYZ:
    def __init__(self, parent, coords, unit) -> None:
        self.parent = parent
        self.coords = coords
        self.unit = unit


class QExpr:
    """An expression over quantity refs and measures: `+`/`-` terms of `*`/`/` factors."""

    def __init__(self, parent, head, tail) -> None:
        self.parent = parent
        self.head = head
        self.tail = tail


class QAddTail:
    def __init__(self, parent, op, operand) -> None:
        self.parent = parent
        self.op = op
        self.operand = operand


class QTerm:
    def __init__(self, parent, head, tail) -> None:
        self.parent = parent
        self.head = head
        self.tail = tail


class QMulTail:
    def __init__(self, parent, op, operand) -> None:
        self.parent = parent
        self.op = op
        self.operand = operand


class QFactor:
    def __init__(self, parent, neg, group, leaf) -> None:
        self.parent = parent
        self.neg = neg
        self.group = group
        self.leaf = leaf


class QuantityLeaf:
    """A quantity-expression leaf: a world or context quantity ref, or a bare measure."""

    def __init__(self, parent, quantity, bare, selector) -> None:
        self.parent = parent
        self.quantity = quantity
        self.bare = bare
        self.selector = selector
        self.subspace = SubSpace(selector.subspace) if selector and selector.subspace else None
        self.axis = Axis(selector.axis) if selector and selector.axis else None


@dataclass
class QOpNode:
    """A normalized interior node of a quantity-expression op tree."""

    op: str
    operands: list


_CHAIN_OPS = {"+": "add", "-": "subtract", "*": "multiply", "/": "divide"}


def op_tree(node):
    """NODE as nested `QOpNode`s: `+`/`*` runs collapse to one n-ary node, `-`/`/` stay binary."""
    if isinstance(node, QFactor):
        inner = op_tree(node.group) if node.group is not None else node.leaf
        if not node.neg:
            return inner
        neg_one = QuantityLeaf(node, None, Measure(node, -1.0, "1"), None)
        return QOpNode("multiply", [neg_one, inner])
    if isinstance(node, (QExpr, QTerm)):
        acc = op_tree(node.head)
    elif isinstance(node, SnapshotValue):
        acc = node.source
    else:
        return node
    group = None
    for step in node.tail:
        operand = op_tree(step.operand)
        if step.op in ("+", "*"):
            group = [acc] if group is None else group
            group.append(operand)
            acc = QOpNode(_CHAIN_OPS[step.op], group)
        else:
            group = None
            acc = QOpNode(_CHAIN_OPS[step.op], [acc, operand])
    return acc


class ReferenceValue:
    """A quantity stated as an expression; `source` is the leaf frames are inherited from."""

    def __init__(self, parent, expr) -> None:
        self.parent = parent
        self.expr = expr
        factor = expr.head.head
        while factor.group is not None:
            factor = factor.group.head.head
        self.source = factor.leaf


class SnapshotValue:
    """A quantity view sampled, plus an optional expression tail, on every `trigger` event."""

    def __init__(self, parent, source, tail, trigger) -> None:
        self.parent = parent
        self.source = source
        self.tail = tail
        self.trigger = trigger


class ConfigValue:
    """A pose the deployment's config file states, framed like the `source` quantity."""

    def __init__(self, parent, key, source) -> None:
        self.parent = parent
        self.key = key
        self.source = source


class DirectionBetween:
    """The unit vector from one frame's origin to another's, recomputed every cycle."""

    def __init__(self, parent, from_frame, to_frame) -> None:
        self.parent = parent
        self.from_frame = from_frame
        self.to_frame = to_frame


def geo_prop(props: GeometricProps | None, key: str) -> str | None:
    """The IRI geometric prop KEY of PROPS names, or None."""
    if not isinstance(props, GeometricProps):
        return None
    for pair in props.pairs:
        if pair.key == key:
            value = pair.frame or pair.joint or pair.sensor or pair.value
            return str(value.uri) if isinstance(value, NamedNamespaceObject) else str(value)
    return None


def geo_prop_events(props: GeometricProps | None, key: str) -> list:
    """The events geometric prop KEY of PROPS lists, or none."""
    if not isinstance(props, GeometricProps):
        return []
    for pair in props.pairs:
        if pair.key == key:
            return list(pair.events)
    return []


def geo_prop_value(props: GeometricProps | None, key: str) -> object | None:
    """The structure geometric prop KEY of PROPS carries, or None."""
    if not isinstance(props, GeometricProps):
        return None
    for pair in props.pairs:
        if pair.key == key:
            return pair.normalization or pair.value
    return None


def pose_frame_names(quantity) -> tuple[str, str, str] | None:
    """A pose's (of, wrt, as-seen-by) frames, through a snapshot's source when it states none."""
    props = quantity.props
    if geo_prop(props, "of") is None and isinstance(quantity.value, (SnapshotValue, ConfigValue)):
        props = quantity.value.source.quantity.props
    of_frame = geo_prop(props, "of")
    wrt_frame = geo_prop(props, "wrt")
    if of_frame is None or wrt_frame is None:
        return None
    return of_frame, wrt_frame, geo_prop(props, "as-seen-by") or wrt_frame


def quantity_axis_frame(quantity: WorldQuantity) -> str | None:
    """The frame a quantity's axes are in: its `as-seen-by`, or `wrt` for a pose."""
    axis_frame = geo_prop(quantity.props, "as-seen-by")
    if axis_frame is None and quantity.type == WorldQuantityType.Pose:
        return geo_prop(quantity.props, "wrt")
    return axis_frame


def path_pose_endpoints(quantity) -> tuple:
    """The pose endpoints whose frame relation defines a geometric path."""
    value = quantity.value
    if not isinstance(value, PathValue):
        return ()
    if value.lerp is not None:
        refs = (value.lerp.start, value.lerp.goal)
    elif value.arc is not None:
        refs = (value.arc.start, value.arc.end)
    elif value.circle is not None:
        refs = (value.circle.start,)
    elif value.helix is not None:
        refs = (value.helix.start,)
    elif value.figure8 is not None:
        refs = (value.figure8.anchor,)
    else:
        return ()
    return tuple(ref.quantity for ref in refs if ref.quantity is not None)


# The constraint-view subspace each geometric operator's coordinate is read through. Every one is
# a length, signed or not.
GEOMETRIC_DISTANCE_SUBSPACE: dict[str, str] = {
    "PointPlaneToLinearDistance": "point-plane-distance",
    "PointLineToLinearDistance": "point-line-distance",
    "PointBodyLineToLinearDistance": "point-body-line-distance",
    "PointOnLineProjection": "point-line-projection",
    "LineLineToLinearDistance": "line-line-distance",
    "LineOnLineProjection": "line-line-projection",
}

WORLD_SUBSPACE_SCALAR_TYPES = {
    (WorldQuantityType.VelocityTwist, "angular"): QuantityType.AngularVelocity,
    (WorldQuantityType.VelocityTwist, "linear"): QuantityType.LinearVelocity,
    (WorldQuantityType.Wrench, "torque"): QuantityType.Torque,
    (WorldQuantityType.Wrench, "force"): QuantityType.Force,
}

JOINT_SCALAR_TYPES = {
    WorldQuantityType.JointPosition: QuantityType.Angle,
    WorldQuantityType.JointVelocity: QuantityType.AngularVelocity,
    WorldQuantityType.JointCurrent: QuantityType.ElectricCurrent,
}

POSE_SUBSPACE_SCALAR_TYPES = {
    "pose": QuantityType.Pose,
    "distance": QuantityType.Distance,
    "rotation": QuantityType.PlaneAngle,
    "alignment": QuantityType.Angle,
    "incident-angle": QuantityType.Angle,
    "plane-angle": QuantityType.Angle,
    **{subspace: QuantityType.Distance for subspace in GEOMETRIC_DISTANCE_SUBSPACE.values()},
}


def scalar_type(quantity: WorldQuantity, subspace: str, axis: str | None):
    """The kind a `quantity.subspace[.axis]` view reads: Pose.position is a Position, .x a Distance."""
    if quantity.type in JOINT_SCALAR_TYPES:
        return JOINT_SCALAR_TYPES[quantity.type]
    if quantity.type == WorldQuantityType.Pose:
        if subspace == "position":
            return QuantityType.Position if axis is None else QuantityType.Distance
        if subspace == "orientation":
            return QuantityType.Orientation if axis is None else QuantityType.Angle
        if subspace in POSE_SUBSPACE_SCALAR_TYPES:
            return POSE_SUBSPACE_SCALAR_TYPES[subspace]
    return WORLD_SUBSPACE_SCALAR_TYPES.get((quantity.type, subspace), subspace)


# Vector views a norm applies to, and the kind of their length.
NORM_SCALAR_TYPES = {
    QuantityType.Position: QuantityType.Distance,
    QuantityType.LinearVelocity: QuantityType.LinearVelocity,
    QuantityType.AngularVelocity: QuantityType.AngularVelocity,
    QuantityType.Force: QuantityType.Force,
    QuantityType.Torque: QuantityType.Torque,
}


def geometric_operand_kind(operand: object) -> str | None:
    """The Table II kind of a distance or projection operand: point, line, plane, or None."""
    if isinstance(operand, WorldQuantity):
        return "point" if operand.type == WorldQuantityType.Pose else None
    if isinstance(operand, ContextQuantity):
        return GEOMETRIC_OPERAND_KINDS.get(operand.type)
    return None


GEOMETRIC_OPERAND_KINDS = {
    QuantityType.Pose: "point",
    QuantityType.Line: "line",
    QuantityType.Plane: "plane",
}


def is_body_line_distance(binary) -> bool:
    """Whether a point-from-line view's line rides the frame the point is measured against."""
    left, right = binary.left, binary.right
    if geometric_operand_kind(left) != "point" or geometric_operand_kind(right) != "line":
        return False
    wrt = geo_prop(left.props, GeometricPropKey.Wrt)
    return wrt is not None and wrt == geo_prop(right.props, GeometricPropKey.Of)


# Table II distance/projection operators by (first, second) operand kind. Line-line order is no
# key: `in1`/`in2` decide whether `LineOnLineProjection` computes s1 or s2.
GEOMETRIC_DISTANCE_OPS: dict[tuple[str, str], str] = {
    ("point", "plane"): "PointPlaneToLinearDistance",
    ("point", "line"): "PointLineToLinearDistance",
    ("line", "line"): "LineLineToLinearDistance",
}
GEOMETRIC_PROJECTION_OPS: dict[tuple[str, str], str] = {
    ("point", "line"): "PointOnLineProjection",
    ("line", "line"): "LineOnLineProjection",
}
# Unsigned magnitudes: no derivative at zero.
UNSIGNED_GEOMETRIC_DISTANCE_OPS = {"PointLineToLinearDistance", "PointBodyLineToLinearDistance"}

# A line riding the point's measurement frame takes no origin pose, and its gradient has an
# angular half: tilting the axis sweeps it across the point.
BODY_LINE_DISTANCE_OP = "PointBodyLineToLinearDistance"

# The relation each operator's coordinate is `of`; a distance and its projection share one.
GEOMETRIC_DISTANCE_RELATION: dict[str, str] = {
    "PointPlaneToLinearDistance": "PointPlaneDistance",
    "PointLineToLinearDistance": "PointLineDistance",
    # comp-rob2b's collinearity relation: the body-line case is the paper's collinearity constraint.
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


class ElapsedTime:
    """Time since motion entry, or since the last reading of `observed` landed."""

    def __init__(self, parent, marker, observed) -> None:
        self.parent = parent
        self.marker = marker
        self.observed = observed


class ProgressAlong:
    """How fast a frame travels along a path, measured on the path's tangent."""

    def __init__(self, parent, moved, path) -> None:
        self.parent = parent
        self.moved = moved
        self.path = path


class MovingAlong:
    """A profiled drive along a path: the profile commands the tangent speed, the normals are held."""

    def __init__(self, parent, moved, path, profile) -> None:
        self.parent = parent
        self.moved = moved
        self.path = path
        self.profile = profile


class OnPath:
    """A frame held on a path's geometry, evaluated at its own closest point, with no timing."""

    def __init__(self, parent, moved, selector, path) -> None:
        self.parent = parent
        self.moved = moved
        self.selector = selector
        self.path = path


class NormView:
    """The Euclidean norm of a 3-vector view, optionally of its component across a direction."""

    def __init__(self, parent, quantity, selector, across) -> None:
        self.parent = parent
        self.quantity = quantity
        self.selector = selector
        self.across = across


class SelectorTail:
    """A subspace/axis selector; a bare axis names a component of a quantity that is a 3-vector."""

    def __init__(self, parent, subspace, axis) -> None:
        self.parent = parent
        self.subspace = SubSpace(subspace) if subspace else None
        self.axis = Axis(axis) if axis else None


class DistanceBetweenView:
    """`distance between <a> and <b>`: the point-to-point distance of two frames."""

    def __init__(self, parent, left, right) -> None:
        self.parent = parent
        self.left = left
        self.right = right


class AngleBetweenView:
    """`angle between <a> and <b>` over two directions, a direction and a plane, or two planes."""

    def __init__(self, parent, left, right) -> None:
        self.parent = parent
        self.left = left
        self.right = right


class DistanceFromView:
    """`distance of <a> from <b>`: a Table II point/line to line/plane distance."""

    def __init__(self, parent, left, right) -> None:
        self.parent = parent
        self.left = left
        self.right = right


class ProjectionOnView:
    """`projection of <a> on <b>`: a Table II point/line projection on a line."""

    def __init__(self, parent, left, right) -> None:
        self.parent = parent
        self.left = left
        self.right = right


class DerivedScalarValue:
    """A scalar the run computes from a binary view, named so other views can use it."""

    def __init__(self, parent, view) -> None:
        self.parent = parent
        self.view = view


class View:
    """A quantity selector, distance relation, or elapsed-time constraint operand."""

    def __init__(
        self, parent, binary, elapsed, progress, moving, on, norm, expr, quantity, selector
    ) -> None:
        self.parent = parent
        self.binary = binary
        self.elapsed = elapsed
        self.progress = progress
        self.moving = moving
        self.on = on
        self.norm = norm
        self.expr = expr
        self.quantity = quantity
        self.selector = selector
        # On-path and norm views still select a subspace of a world quantity.
        if on is not None:
            self.quantity = on.moved
            self.selector = on.selector
        if norm is not None:
            self.quantity = norm.quantity
            self.selector = norm.selector
        # textX may initialize the selector after this view.
        selector = self.selector
        self.subspace = SubSpace(selector.subspace) if selector and selector.subspace else None
        self.axis = Axis(selector.axis) if selector and selector.axis else None
        # Driving along a path and guarding progress along it act on the tangent speed.
        if moving is not None or progress is not None:
            self.subspace = SubSpace.LinVel


class ContextRef:
    """A reference to a context value, optionally with a subspace/axis, or a bare literal."""

    name = "ref"

    def __init__(self, parent, quantity, selector, bare, expr) -> None:
        self.parent = parent
        self.quantity = quantity
        self.selector = selector
        self.bare = bare
        self.expr = expr
        self.subspace = SubSpace(selector.subspace) if selector and selector.subspace else None
        self.axis = Axis(selector.axis) if selector and selector.axis else None

    @property
    def namespace(self) -> Namespace:
        current = self.parent
        while current is not None:
            if isinstance(current, (NamedNamespaceObject, IHasNamespaceDeclare)):
                return Namespace(str(current.namespace) + f"{current.name}/")
            current = current.parent
        raise AttributeError("ContextRef namespace is not resolved yet")
