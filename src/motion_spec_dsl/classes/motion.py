# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Domain classes for the motion-spec DSL grammar constructs."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from typing import TYPE_CHECKING

from rdflib.namespace import Namespace

from motion_spec_dsl.classes.common import (
    NamespaceDeclLike,
    IHasNamespaceDeclare,
    NamedNamespaceObject,
)

if TYPE_CHECKING:
    from motion_spec_dsl.classes.constraint_handler import ConstraintHandler

_TIME_UNIT_SECONDS = {"s": 1.0, "ms": 0.001}


class NamespaceDeclare:
    """A namespace declaration: a prefix `name` and its `uri`."""

    def __init__(self, name: str = "", uri: str = "", **_):
        self.name = name
        self.uri = uri


class Import:
    """An `importURI` reference to another model or FSM file."""

    def __init__(self, importURI: str = "", **_):
        self.importURI = importURI


class Model:
    """Root of a parsed motion-spec model: its imports, namespaces and top-level specs."""

    def __init__(
        self,
        imports: list[Import] | None = None,
        namespaces: list[NamespaceDeclare] | None = None,
        specs: list[ExecutionContext | ContextSpec | GuardedMotion | ConstraintHandler]
        | None = None,
        **_,
    ):
        self.imports = imports or []
        self.namespaces = namespaces or []
        self.specs = specs or []


@dataclass
class PositionTerm:
    parent: object
    axis: str
    value: float = 0.0
    unit: str = "m"
    ref: object | None = None


@dataclass
class PositionValue:
    parent: object
    terms: list[PositionTerm] = field(default_factory=list)


@dataclass
class OrientationTerm:
    parent: object
    axis: str
    value: float = 0.0
    unit: str = "rad"
    ref: object | None = None


@dataclass
class OrientationValue:
    parent: object
    terms: list[OrientationTerm] = field(default_factory=list)


@dataclass
class ExecutionContext(IHasNamespaceDeclare):
    """Select the scene and platform on which this motion specification runs."""

    parent: object
    ns: NamespaceDeclLike
    name: str
    scene: object
    platform: object
    timestep: float
    timestep_unit: str

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)


@dataclass
class ContextSpec(IHasNamespaceDeclare):
    """A named context block declaring world and context quantities."""

    parent: object
    ns: NamespaceDeclLike
    name: str
    context: list[WorldContextDecl | PreContextDecl | SpecContextDecl | PostContextDecl]

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)


@dataclass
class LerpSpec:
    parent: object
    start: object  # ContextRef
    goal: object  # ContextRef


@dataclass
class CircleSpec:
    parent: object
    start: (
        object  # ContextRef (Pose on the curve: position -> start point, rotation -> orientation)
    )
    center: object  # ContextRef (Position the curve orbits; radius = |start - center| in-plane)
    plane_normal: object  # ContextRef


@dataclass
class ArcSpec:
    parent: object
    start: (
        object  # ContextRef (Pose on the curve: position -> start point, rotation -> orientation)
    )
    end: object  # ContextRef (Pose: the other endpoint and orientation target)
    amplitude: object  # ContextRef (LinearDistance: how far the arc bows from the chord; = chord/2 -> semicircle)
    plane_normal: object  # ContextRef


@dataclass
class HelixSpec:
    parent: object
    start: (
        object  # ContextRef (Pose on the curve: position -> start point, rotation -> orientation)
    )
    center: (
        object  # ContextRef (Position the helix winds around; radius = |start - center| in-plane)
    )
    axis: object  # ContextRef
    pitch: object  # ContextRef
    revolutions: object  # ContextRef


@dataclass
class Figure8Spec:
    parent: object
    anchor: object  # ContextRef (Pose: position -> center, rotation -> orientation)
    radius: object  # ContextRef
    plane_normal: object  # ContextRef
    form: str = "gerono"


@dataclass
class PathValue:
    parent: object
    lerp: LerpSpec | None = None
    circle: CircleSpec | None = None
    arc: ArcSpec | None = None
    helix: HelixSpec | None = None
    figure8: Figure8Spec | None = None


@dataclass
class ProfileSpec:
    """Authored limits and shape for an online velocity profile."""

    parent: object
    max_velocity: object
    max_acceleration: object
    measured_velocity: object | None = None
    max_jerk: object | None = None
    shape: str = "trapezoidal"


@dataclass
class AdmittanceSpec:
    parent: object
    force: object  # View onto a measured-Wrench force axis (force-in)
    mass: float
    damping: float
    stiffness: float = 0.0
    max_velocity: float = 0.25
    max_velocity_unit: object | None = None


@dataclass
class PoseValue:
    parent: object
    position: PositionValue
    orientation: OrientationValue


@dataclass
class GuardedMotion(IHasNamespaceDeclare):
    """A guarded motion: its when/while/until constraint sections and context."""

    parent: object
    ns: NamespaceDeclLike
    name: str
    move: str | None
    context: list[
        WorldContextDecl | PreContextDecl | SpecContextDecl | PostContextDecl | ContextDeclReference
    ]
    sections: list[WhenSection | WhileSection | UntilSection]

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)
        self.when = self._section("when")
        self.while_ = self._section("while")
        self.until = self._section("until")
        assert len(self.while_.constraints) > 0, (
            "GuardedMotion must have at least one 'while' constraint"
        )

    def _section(self, name: str) -> WhenSection | WhileSection | UntilSection:
        for section in self.sections:
            if section.name == name:
                return section
        raise ValueError(f"GuardedMotion '{self.name}' is missing required {name.upper()} section")


@dataclass
class QuantityContextDecl(NamedNamespaceObject):
    """A context declaration of quantities (world or context)."""

    kind = None

    parent: object
    name: str = ""
    declaration: list[ContextQuantity | WorldQuantity] = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)

    @property
    def namespace(self):
        assert self.kind is not None, "QuantityContextDecl must have 'kind' defined"
        parent_namespace = getattr(self.parent, "namespace")
        parent_name = getattr(self.parent, "name")
        return Namespace(str(parent_namespace) + f"{parent_name}/{self.kind}/")


class WorldContextDecl(QuantityContextDecl):
    kind = "World"


class PreContextDecl(QuantityContextDecl):
    kind = "Pre"


class SpecContextDecl(QuantityContextDecl):
    kind = "Spec"


class PostContextDecl(QuantityContextDecl):
    kind = "Post"


@dataclass
class ContextDeclReference:
    parent: object
    ref: QuantityContextDecl


@dataclass
class ConstraintSection(NamedNamespaceObject):
    """A when/while/until section holding constraint items and its combination logic."""

    kind = ""

    parent: object
    constraints: list[ConstraintSpecification | ConstraintAlias] = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.kind)


@dataclass
class WhenSection(ConstraintSection):
    kind = "when"
    logic: str | None = None


class WhileSection(ConstraintSection):
    kind = "while"


@dataclass
class UntilSection(ConstraintSection):
    kind = "until"
    logic: str | None = None


class WorldQuantityType(StrEnum):
    Pose = "Pose"
    VelocityTwist = "VelocityTwist"
    Wrench = "Wrench"
    JointPosition = "JointPosition"


def _authored_enum(enum_type: type[StrEnum], keyword: str) -> StrEnum:
    """Translate a kebab-case DSL keyword to its established enum member."""
    normalized = keyword.replace("-", "").casefold()
    for member in enum_type:
        if member.value.replace("-", "").casefold() == normalized:
            return member
    raise ValueError(f"'{keyword}' is not a valid {enum_type.__name__}")


# eq=False: a model entity has identity semantics (its `parent` back-ref makes value-eq
# cyclic anyway); identity equality is also what makes it usable as a dict/set key.
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


# eq=False: identity semantics (a props bag is owned by its parent, like the other model
# entities) -- also makes it hashable so `_geo_prop` lookups can be memoized.
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


@dataclass
class GeoPropPair:
    """A geometric-property key/value pair (of/wrt/as-seen-by/ref-point/...)."""

    key: GeometricPropKey
    value: object | None = None
    frame: object | None = None
    joint: object | None = None
    sensor: object | None = None
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        self.key = GeometricPropKey(self.key)
        if self.value is None:
            self.value = self.frame or self.joint or self.sensor


class QuantityType(StrEnum):
    Pose = "Pose"
    Position = "Position"
    Orientation = "Orientation"
    VelocityTwist = "VelocityTwist"
    AccelerationTwist = "AccelerationTwist"
    Wrench = "Wrench"
    Direction = "Direction"
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

QUANTITY_TYPE_ALIASES = {
    "linear-distance": QuantityType.Distance,
    "angular-distance": QuantityType.Angle,
}


@dataclass
class ContextQuantity(NamedNamespaceObject):
    """A context quantity: a reference, snapshot, profile, path, or literal value."""

    parent: object
    name: str
    type: ContextDeclarationType
    value: (
        Measure
        | VectorQuantity
        | ReferenceValue
        | SnapshotValue
        | PathValue
        | ProfileSpec
        | AdmittanceSpec
        | PoseValue
        | None
    ) = None
    props: GeometricProps | None = field(default=None, kw_only=True)

    _SCALAR_TYPES = frozenset(
        {
            "Distance",
            "LinearDistance",
            "Angle",
            "PlaneAngle",
            "AngularDistance",
            "PathParameter",
            "Dimensionless",
            "Duration",
        }
    )

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
        raw_type = str(self.type)
        if raw_type in QUANTITY_TYPE_ALIASES:
            self.type = QUANTITY_TYPE_ALIASES[raw_type]
        else:
            try:
                self.type = _authored_enum(ReferenceGeneratorType, raw_type)
            except ValueError:
                self.type = _authored_enum(QuantityType, raw_type)
        if self.props is not None and raw_type in self._SCALAR_TYPES:
            raise ValueError(
                f"geometric props block is not valid for scalar quantity type '{self.type}'"
            )


@dataclass(kw_only=True)
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
class VectorQuantity:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
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


# eq=False: identity semantics (see WorldQuantity) -- also makes it a valid dict/set key.
@dataclass(eq=False)
class ConstraintSpecification(NamedNamespaceObject):
    """A single constraint: a view (LHS) compared against a reference by an expression."""

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
    disabled: bool = False

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


# eq=False: identity semantics, matching ConstraintSpecification.
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

    motion: GuardedMotion
    constraint: ConstraintSpecification
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def motion_name(self) -> str:
        return self.motion.name

    @property
    def name(self) -> str:
        return self.constraint.name

    def __str__(self) -> str:
        return f"{self.motion.name}.{self.constraint.name}"


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
    Roll = "roll"
    Pitch = "pitch"
    Yaw = "yaw"


@dataclass
class ElapsedTime:
    marker: bool = False
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

    def __post_init__(self):
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
    inline_quantity: ContextQuantity | None = None
    context_scope: str | None = None
    literal_value: Measure | VectorQuantity | None = None
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
        if self.quantity is None:
            self.quantity = self.inline_quantity

    @property
    def name(self) -> str:
        return self.context_scope or "ref"

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


@dataclass
class EqualityConstraint:
    """An equality constraint against a reference value, optionally within a tolerance."""

    reference: ContextRef
    tolerance: ContextRef | None = None
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
