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
    Normalization = "normalization"


@dataclass
class GeoPropPair:
    """A geometric-property key/value pair (of/wrt/as-seen-by/ref-point/...)."""

    key: GeometricPropKey
    value: object | None = None
    frame: object | None = None
    joint: object | None = None
    sensor: object | None = None
    normalization: object | None = None
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        self.key = GeometricPropKey(self.key)
        if self.value is None:
            self.value = self.frame or self.joint or self.sensor or self.normalization


class QuantityType(StrEnum):
    Pose = "Pose"
    Position = "Position"
    Orientation = "Orientation"
    VelocityTwist = "VelocityTwist"
    AccelerationTwist = "AccelerationTwist"
    Wrench = "Wrench"
    Direction = "Direction"
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


@dataclass
class VectorXYZ:
    coords: Coordinates | None = None
    unit: str = ""
    parent: object | None = field(default=None, repr=False, compare=False)


@dataclass
class ReferenceValue:
    source: ContextRef
    offset: ContextRef | None = None
    parent: object | None = field(default=None, repr=False, compare=False)


@dataclass
class SnapshotValue:
    """A quantity view sampled with an optional offset. Sampled once when its owning motion
    starts, or re-sampled on every occurrence of `trigger` when one is given.
    """

    source: View
    offset: ContextRef | None = None
    trigger: object | None = None
    parent: object | None = field(default=None, repr=False, compare=False)


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


def _resolved_world_quantity(item: WorldQuantity | WorldQuantityAlias) -> WorldQuantity:
    return item.ref if isinstance(item, WorldQuantityAlias) else item


def _resolved_context_quantity(item: ContextQuantity | ContextQuantityAlias) -> ContextQuantity:
    return item.ref if isinstance(item, ContextQuantityAlias) else item


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
    """A drive along a path at a commanded speed, holding the frame on the path's geometry.

    This is the driver of the motion, not a comparison, so it carries no relation: the
    tangent is commanded at `speed` while the directions normal to it are held at zero.
    """

    moved: object
    path: object
    speed: object
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
    subspace: SubSpace | None = None
    axis: Axis | None = None
    selector: SelectorTail | None = None
    distance_from: WorldQuantity | None = None
    distance_to: WorldQuantity | None = None
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
