# SPDX-License-Identifier: MPL-2.0
"""Domain classes for the motion_spec DSL grammar constructs."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

from rdflib.namespace import Namespace

from motion_spec_dsl.namespaces import (
    NamespaceDeclLike,
    IHasNamespaceDeclare,
    NamedNamespaceObject,
)


class NamespaceDeclare:
    def __init__(self, name: str = "", uri: str = "", **_):
        self.name = name
        self.uri = uri


class Import:
    def __init__(self, importURI: str = "", **_):
        self.importURI = importURI


class Model:
    def __init__(
        self,
        imports: list[Import] | None = None,
        namespaces: list[NamespaceDeclare] | None = None,
        specs: list[EnvironmentSpec | ContextSpec | MotionSpec | ConstraintHandler] | None = None,
        **_,
    ):
        self.imports = imports or []
        self.namespaces = namespaces or []
        self.specs = specs or []


class EnvironmentRuntime(StrEnum):
    MuJoCo = "MuJoCo"
    RealRobot = "RealRobot"


class EnvironmentAssetType(StrEnum):
    RobotAsset = "RobotAsset"
    AttachmentAsset = "AttachmentAsset"
    SceneObject = "SceneObject"


class EnvironmentAssemblyType(StrEnum):
    Object = "Object"
    Robot = "Robot"
    Attachment = "Attachment"


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
class EnvironmentAsset(NamedNamespaceObject):
    parent: object
    name: str
    type: EnvironmentAssetType
    model: str = ""
    xml: str = ""
    urdf: str = ""

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
        self.type = EnvironmentAssetType(self.type)


@dataclass
class EnvironmentAttachmentPrefixEntry:
    parent: object
    value: str


@dataclass
class EnvironmentAttachmentActuatorEntry:
    parent: object
    value: str


@dataclass
class EnvironmentPositionEntry:
    parent: object
    value: PositionValue


@dataclass
class EnvironmentOrientationEntry:
    parent: object
    value: OrientationValue


@dataclass
class EnvironmentFreeEntry:
    parent: object
    value: bool = False


@dataclass
class EnvironmentAttachTargetRef:
    parent: object
    assembly: "EnvironmentAssembly"
    kind: str
    name: str


@dataclass
class EnvironmentAttachTargetEntry:
    parent: object
    target: EnvironmentAttachTargetRef

    @property
    def kind(self) -> str:
        return self.target.kind

    @property
    def name(self) -> str:
        return self.target.name

    @property
    def target_assembly(self) -> "EnvironmentAssembly":
        return self.target.assembly


@dataclass
class EnvironmentToolBodyEntry:
    parent: object
    value: str


@dataclass
class EnvironmentTcpSiteEntry:
    parent: object
    value: str


@dataclass
class EnvironmentFtSensorEntry:
    parent: object
    name: str
    frame_site: str


@dataclass
class EnvironmentShapeEntry:
    parent: object
    value: str


@dataclass
class EnvironmentSizeEntry:
    parent: object
    value: PositionValue


@dataclass
class EnvironmentMassEntry:
    parent: object
    value: float


@dataclass
class ColorTerm:
    parent: object
    channel: str
    value: float = 0.0


@dataclass
class ColorValue:
    parent: object
    terms: list[ColorTerm] = field(default_factory=list)


@dataclass
class EnvironmentColorEntry:
    parent: object
    value: ColorValue


@dataclass
class FrictionTerm:
    parent: object
    axis: str
    value: float = 0.0


@dataclass
class FrictionValue:
    parent: object
    terms: list[FrictionTerm] = field(default_factory=list)


@dataclass
class EnvironmentFrictionEntry:
    parent: object
    value: FrictionValue


@dataclass
class EnvironmentChainEntry:
    parent: object
    root: str
    end: str


@dataclass
class EnvironmentAssembly(NamedNamespaceObject):
    parent: object
    type: EnvironmentAssemblyType
    name: str
    asset: EnvironmentAsset
    entries: list = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
        self.type = EnvironmentAssemblyType(self.type)

    @property
    def chain(self) -> EnvironmentChainEntry | None:
        return next(
            (entry for entry in self.entries if isinstance(entry, EnvironmentChainEntry)), None
        )

    @property
    def root(self) -> str:
        return self.chain.root if self.chain is not None else ""

    @property
    def end(self) -> str:
        return self.chain.end if self.chain is not None else ""

    @property
    def model(self) -> str:
        return self.asset.model

    @property
    def urdf(self) -> str:
        return self.asset.urdf


@dataclass
class EnvironmentTrace:
    """Live EE trajectory trace overlay configuration for the MuJoCo viewer.

    color is a ColorValue (r/g/b/a channels in [0, 1]); when omitted the runtime
    falls back to its default warm orange. length is the maximum number of recent
    EE positions retained in the trace ring buffer.
    """

    parent: object
    enabled: bool = False
    length: int = 0
    color: ColorValue | None = None
    targets: list = field(default_factory=list)

    def channel(self, name: str, default: float) -> float:
        if self.color is None:
            return default
        return next((t.value for t in self.color.terms if t.channel == name), default)


@dataclass
class EnvironmentSpec(IHasNamespaceDeclare):
    parent: object
    ns: NamespaceDeclLike
    name: str
    runtime: EnvironmentRuntime
    # Optional authored physics timestep (e.g. `timestep: 2.0 ms`). Absent -> the
    # backend default. Decoupled from CONTROL_PERIOD on purpose: the sim step is a
    # physics-fidelity/tuning choice the author owns, not a derived value.
    timestep: BareScalar | None = None
    assets: list[EnvironmentAsset] = field(default_factory=list)
    assembly: list[EnvironmentAssembly] = field(default_factory=list)
    trace: EnvironmentTrace | None = None

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)
        self.runtime = EnvironmentRuntime(self.runtime)

    @property
    def timestep_seconds(self) -> float | None:
        if self.timestep is None:
            return None
        scale = _DURATION_UNIT_SECONDS.get(self.timestep.unit)
        if scale is None:
            raise ValueError(
                f"Environment '{self.name}' timestep unit '{self.timestep.unit}' must be 's' or 'ms'."
            )
        return self.timestep.value * scale


@dataclass
class EnvironmentRobotRef:
    parent: object
    environment: EnvironmentSpec
    assembly: str

    @property
    def assembly_spec(self) -> EnvironmentAssembly | None:
        return next(
            (entry for entry in self.environment.assembly if entry.name == self.assembly), None
        )

    @property
    def name(self) -> str:
        return f"{self.environment.name}.{self.assembly}"

    def __str__(self) -> str:
        return self.name


@dataclass
class ContextSpec(IHasNamespaceDeclare):
    parent: object
    ns: NamespaceDeclLike
    name: str
    context: list[WorldContextDecl | PreContextDecl | SpecContextDecl | PostContextDecl]

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)


@dataclass
class RobotRef:
    environment_robot: EnvironmentRobotRef | None = None
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def component_name(self) -> str | None:
        if self.environment_robot is not None:
            return self.environment_robot.assembly
        return None

    @property
    def name(self) -> str:
        if self.environment_robot is not None:
            return self.environment_robot.name
        return ""

    def __str__(self) -> str:
        return self.name


@dataclass
class RobotAnchorRef:
    anchor: str
    environment_robot: EnvironmentRobotRef | None = None
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def name(self) -> str:
        if self.environment_robot is not None:
            return f"{self.environment_robot.name}.chain.{self.anchor}"
        return ""

    @property
    def component_name(self) -> str | None:
        if self.environment_robot is not None:
            return self.environment_robot.assembly
        return None

    def __str__(self) -> str:
        return self.name


@dataclass
class LerpSpec:
    parent: object
    start: object  # ContextRef
    goal: object  # ContextRef
    alpha: object  # ContextRef
    profile: str = "EaseInOut"  # progress easing: Linear | EaseIn | EaseOut | EaseInOut


@dataclass
class CircleSpec:
    parent: object
    start: (
        object  # ContextRef (Pose on the curve: position -> start point, rotation -> orientation)
    )
    center: object  # ContextRef (Position the curve orbits; radius = |start - center| in-plane)
    plane_normal: object  # ContextRef
    alpha: object  # ContextRef


@dataclass
class ArcSpec:
    parent: object
    start: (
        object  # ContextRef (Pose on the curve: position -> start point, rotation -> orientation)
    )
    end: object  # ContextRef (Pose: the other endpoint and orientation target)
    amplitude: object  # ContextRef (LinearDistance: how far the arc bows from the chord; = chord/2 -> semicircle)
    plane_normal: object  # ContextRef
    alpha: object  # ContextRef


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
    alpha: object  # ContextRef


@dataclass
class Figure8Spec:
    parent: object
    anchor: object  # ContextRef (Pose: position -> center, rotation -> orientation)
    radius: object  # ContextRef
    plane_normal: object  # ContextRef
    alpha: object  # ContextRef
    form: str = "Gerono"


@dataclass
class TrajectoryValue:
    parent: object
    lerp: LerpSpec | None = None
    circle: CircleSpec | None = None
    arc: ArcSpec | None = None
    helix: HelixSpec | None = None
    figure8: Figure8Spec | None = None


@dataclass
class ProfileSpec:
    parent: object
    max_velocity: object
    max_acceleration: object
    measured_velocity: object | None = None
    max_jerk: object | None = None
    shape: str = "Trapezoidal"


@dataclass
class AdmittanceSpec:
    parent: object
    force: object  # View onto an ExternalForce axis (force-in)
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
class TrajectorySpec:
    parent: object
    type: str


@dataclass
class MotionSpec(IHasNamespaceDeclare):
    parent: object
    ns: NamespaceDeclLike
    name: str
    move: str | None
    trajectory: TrajectorySpec | None
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
            "MotionSpec must have at least one 'while' constraint"
        )

    def _section(self, name: str) -> WhenSection | WhileSection | UntilSection:
        for section in self.sections:
            if section.name == name:
                return section
        raise ValueError(f"MotionSpec '{self.name}' is missing required {name.upper()} section")


@dataclass
class QuantityContextDecl(NamedNamespaceObject):
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
    Frame = "Frame"
    Pose = "Pose"
    VelocityTwist = "VelocityTwist"
    Wrench = "Wrench"
    ExternalForceMagnitude = "ExternalForceMagnitude"
    ExternalForce = "ExternalForce"
    JointPosition = "JointPosition"
    KinematicChain = "KinematicChain"
    Link = "Link"
    SceneObject = "SceneObject"
    Gravity = "Gravity"


@dataclass
class WorldQuantity(NamedNamespaceObject):
    parent: object
    name: str
    type: WorldQuantityType
    props: GeometricProps | None = field(default=None, kw_only=True)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
        self.type = WorldQuantityType(self.type)


@dataclass(kw_only=True)
class WorldQuantityAlias(WorldQuantity):
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


@dataclass
class GeometricProps:
    pairs: list[GeoPropPair]
    parent: object | None = field(default=None, repr=False, compare=False)


class GeometricPropKey(StrEnum):
    Of = "of"
    Wrt = "wrt"
    RefPoint = "ref-point"
    AsSeenBy = "as-seen-by"
    FtSensor = "ft-sensor"
    Deadband = "deadband"


@dataclass
class GeoPropPair:
    key: GeometricPropKey
    value: str = ""
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        self.key = GeometricPropKey(self.key)


class QuantityType(StrEnum):
    Pose = "Pose"
    Position = "Position"
    Orientation = "Orientation"
    Direction = "Direction"
    Distance = "Distance"
    Angle = "Angle"
    PlaneAngle = "PlaneAngle"
    AngularDistance = "AngularDistance"
    LinearVelocity = "LinearVelocity"
    AngularVelocity = "AngularVelocity"
    LinearAcceleration = "LinearAcceleration"
    AngularAcceleration = "AngularAcceleration"
    LinearJerk = "LinearJerk"
    Force = "Force"
    Torque = "Torque"
    FreeVector = "FreeVector"
    Duration = "Duration"
    Trajectory = "Trajectory"
    TrajectoryProgress = "TrajectoryProgress"
    VelocityProfile = "VelocityProfile"
    Admittance = "Admittance"


class HandlerControlMode(StrEnum):
    JointTorque = "JointTorque"


class ControllerType(StrEnum):
    PID = "PID"
    Impedance = "Impedance"
    ABAG = "ABAG"
    FeedForward = "FeedForward"


class ControllerParamName(StrEnum):
    Kp = "Kp"
    Ki = "Ki"
    Kd = "Kd"
    Stiffness = "Stiffness"
    Damping = "Damping"
    Decay = "decay"


@dataclass
class ContextQuantity(NamedNamespaceObject):
    parent: object
    name: str
    type: QuantityType
    value: (
        ScalarQuantity
        | VectorQuantity
        | SnapshotValue
        | TrajectoryValue
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
            "TrajectoryProgress",
            "Duration",
        }
    )

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
        if self.type == "LinearDistance":
            self.type = QuantityType.Distance
        elif self.type == "Trajectory":
            self.type = QuantityType.Trajectory
        elif self.type == "TrajectoryProgress":
            self.type = QuantityType.TrajectoryProgress
        elif self.type == "VelocityProfile":
            self.type = QuantityType.VelocityProfile
        elif self.type == "Admittance":
            self.type = QuantityType.Admittance
        else:
            self.type = QuantityType(self.type)
        if self.props is not None and str(self.type) in self._SCALAR_TYPES:
            raise ValueError(
                f"geometric props block is not valid for scalar quantity type '{self.type}'"
            )


@dataclass(kw_only=True)
class ContextQuantityAlias(ContextQuantity):
    parent: object
    name: str
    ref: ContextQuantity
    type: QuantityType = field(init=False)
    value: object | None = field(init=False, default=None)

    def __post_init__(self):
        if not self.name:
            self.name = self.ref.name
        NamedNamespaceObject.__init__(self, parent=self.parent, name=self.name)
        self._uri = self.ref.uri
        self.type = self.ref.type
        self.value = self.ref.value


@dataclass
class ScalarQuantity:
    value: float = 0.0
    unit: str = ""
    parent: object | None = field(default=None, repr=False, compare=False)


@dataclass
class BareScalar:
    """An anonymous literal magnitude+unit used directly as a threshold (e.g. `5.0 s`)."""

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
class SnapshotValue:
    source: View
    offset: ContextRef | None = None
    # Sampling clock (snap:sampled-on): "task" = sampled once, held for the run
    # (default); "entry" = re-sampled on each state (re-)entry of the owning motion.
    clock: str = "task"
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        # textX leaves an unmatched optional string assignment empty; normalise to
        # the default task clock so an omitted `on <clock>` == sampled-once.
        if not self.clock:
            self.clock = "task"


@dataclass
class ConstraintSpecification(NamedNamespaceObject):
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


@dataclass
class ConstraintRef:
    motion: MotionSpec
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
class UntilMonitorRef:
    motion: MotionSpec
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def motion_name(self) -> str:
        return self.motion.name

    @property
    def name(self) -> str:
        return "until"

    def __str__(self) -> str:
        return f"{self.motion.name}.until"


@dataclass
class WhenMonitorRef:
    motion: MotionSpec
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def motion_name(self) -> str:
        return self.motion.name

    @property
    def name(self) -> str:
        return "when"

    def __str__(self) -> str:
        return f"{self.motion.name}.when"


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
class View:
    parent: object
    quantity: WorldQuantity | None = None
    subspace: SubSpace | None = None
    axis: Axis | None = None
    distance_from: WorldQuantity | None = None
    distance_to: WorldQuantity | None = None
    is_elapsed: bool = False

    def __post_init__(self):
        if isinstance(self.subspace, str):
            self.subspace = SubSpace(self.subspace)
        if self.axis is not None and isinstance(self.axis, str):
            self.axis = Axis(self.axis)


@dataclass
class ContextRef:
    quantity: ContextQuantity | None = None
    inline_quantity: ContextQuantity | None = None
    context_scope: str | None = None
    literal_value: ScalarQuantity | VectorQuantity | None = None
    bare: BareScalar | None = None
    subspace: str | None = None
    axis: str | None = None
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
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
    reference: ContextRef
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        assert self.reference is not None


@dataclass
class GreaterThanConstraint:
    threshold: ContextRef
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        assert self.threshold is not None


@dataclass
class LessThanConstraint:
    threshold: ContextRef
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        assert self.threshold is not None


@dataclass
class BilateralConstraint:
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


@dataclass
class ConstraintHandler(IHasNamespaceDeclare):
    parent: object
    ns: NamespaceDeclLike
    name: str
    context: list[WorldContextDecl | SpecContextDecl | ContextDeclReference]
    motion: MotionSpec
    control_mode: HandlerControlMode
    solvers: list[SolverEntry | SolverAlias]
    monitors: list[MonitorEntry] = field(default_factory=list)
    controllers: list[ControllerEntry | ControllerAlias] = field(default_factory=list)
    control_period: BareScalar | None = None

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)
        self.control_mode = HandlerControlMode(self.control_mode)


@dataclass
class EventName:
    parent: object
    # Qualified form: `ns.EventName`  →  event is the resolved fsm.Event object
    event: object = None
    ns: NamespaceDeclLike | None = None
    # Standalone form: bare name with no namespace prefix
    standalone: str = ""

    @property
    def name(self) -> str:
        """Bare event name string (works for both qualified and standalone forms)."""
        return self.event.name if self.event is not None else self.standalone

    @property
    def uri(self) -> str:
        if self.ns is not None and self.event is not None:
            # Qualified: use the locally-declared namespace URI (same as in the .fsm).
            return str(Namespace(self.ns.uri)[self.event.name])
        # Standalone (unqualified) event stays monitor-owned.
        return f"{self.parent.uri}.{self.standalone}"


_DURATION_UNIT_SECONDS = {"s": 1.0, "ms": 0.001}


@dataclass
class MonitorEntry(NamedNamespaceObject):
    parent: object
    name: str
    constraint: ConstraintRef | UntilMonitorRef
    event: EventName | None = None
    fallback: MotionSpec | None = None
    flag: str = ""
    # Optional `for <FLOAT> <Unit>` debounce clause: the monitored condition must
    # hold continuously for this long before the edge-triggered monitor fires.
    # Absent (None) == current byte-identical rising-edge behaviour.
    debounce: BareScalar | None = None

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)

    @property
    def debounce_seconds(self) -> float | None:
        if self.debounce is None:
            return None
        scale = _DURATION_UNIT_SECONDS.get(self.debounce.unit)
        if scale is None:
            raise ValueError(
                f"Monitor '{self.name}' debounce unit '{self.debounce.unit}' must be 's' or 'ms'."
            )
        return self.debounce.value * scale

    @property
    def constraint_name(self) -> str:
        return self.constraint.name

    @property
    def is_until_monitor(self) -> bool:
        return isinstance(self.constraint, UntilMonitorRef)


@dataclass
class ControllerEntry(NamedNamespaceObject):
    parent: object
    name: str
    type: ControllerType
    params: ControllerParams
    solver: SolverRef | None = None
    command_type: QuantityType | None = None
    apply_at: WorldQuantity | None = None

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
        self.type = ControllerType(self.type)
        if self.command_type is not None and isinstance(self.command_type, str):
            self.command_type = QuantityType(self.command_type)


@dataclass
class ControllerRef:
    handler: ConstraintHandler
    controller: ControllerEntry
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def name(self) -> str:
        return self.controller.name

    def __str__(self) -> str:
        return f"{self.handler.name}.{self.controller.name}"


@dataclass(kw_only=True)
class ControllerAlias(ControllerEntry):
    parent: object
    name: str
    ref: ControllerRef
    type: ControllerType = field(init=False)
    params: ControllerParams = field(init=False)
    solver: SolverRef | None = field(init=False, default=None)
    command_type: QuantityType | None = field(init=False, default=None)
    apply_at: WorldQuantity | None = field(init=False, default=None)

    def __post_init__(self):
        if not self.name:
            self.name = self.ref.controller.name
        NamedNamespaceObject.__init__(self, parent=self.parent, name=self.name)
        self._uri = self.ref.controller.uri
        self.type = self.ref.controller.type
        self.params = self.ref.controller.params
        self.solver = self.ref.controller.solver
        self.command_type = self.ref.controller.command_type
        self.apply_at = self.ref.controller.apply_at


@dataclass
class ControllerParam:
    name: ControllerParamName
    value: float
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        self.name = ControllerParamName(self.name)


@dataclass
class ControllerParams:
    constraint: ConstraintRef
    profile: ContextRef | None = None
    measured_derivative: View | None = None
    terms: list[ControllerParam] = field(default_factory=list)
    kp: float | None = field(init=False, default=None)
    ki: float | None = field(init=False, default=None)
    kd: float | None = field(init=False, default=None)
    stiffness: float | None = field(init=False, default=None)
    damping: float | None = field(init=False, default=None)
    decay: float | None = field(init=False, default=None)
    duplicate_terms: list[str] = field(init=False, default_factory=list)
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        seen: set[str] = set()
        for term in self.terms:
            name = term.name
            if name in seen:
                self.duplicate_terms.append(name.value)
                continue
            seen.add(name)
            if name == ControllerParamName.Kp:
                self.kp = term.value
            elif name == ControllerParamName.Ki:
                self.ki = term.value
            elif name == ControllerParamName.Kd:
                self.kd = term.value
            elif name == ControllerParamName.Stiffness:
                self.stiffness = term.value
            elif name == ControllerParamName.Damping:
                self.damping = term.value
            elif name == ControllerParamName.Decay:
                self.decay = term.value

    @property
    def constraint_name(self) -> str:
        return self.constraint.name

    @property
    def pid_gains(self) -> tuple[float | None, float | None, float | None]:
        return (self.kp, self.ki, self.kd)

    @property
    def has_pid_gains(self) -> bool:
        return all(gain is not None for gain in self.pid_gains)

    @property
    def has_impedance_terms(self) -> bool:
        return self.stiffness is not None or self.damping is not None


@dataclass
class SolverEntry(NamedNamespaceObject):
    parent: object
    name: str
    robot: RobotRef
    algorithm: str
    # Optional control-loop tuning (DLS damping lambda, torque-limit override,
    # Cartesian-accel "beta" clamp overrides). Unauthored FLOAT grammar attrs
    # default to 0.0, not None; 0.0 is never a valid authored value (SHACL
    # requires > 0), so it doubles as the "unauthored" sentinel downstream.
    damping: float = 0.0
    torque_limit: float = 0.0
    max_linear_accel: float = 0.0
    max_angular_accel: float = 0.0
    root: RobotAnchorRef | None = None
    gravity: WorldQuantity | None = None
    gravity_value: ContextRef | None = None
    end: RobotAnchorRef | None = None

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


@dataclass
class SolverRef:
    solver: SolverEntry
    handler: ConstraintHandler | None = field(default=None)
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def name(self) -> str:
        return self.solver.name

    def __str__(self) -> str:
        if self.handler is not None:
            return f"{self.handler.name}.{self.solver.name}"
        return self.solver.name


@dataclass(kw_only=True)
class SolverAlias(SolverEntry):
    parent: object
    name: str
    ref: SolverRef
    robot: RobotRef = field(init=False)
    algorithm: str = field(init=False)
    damping: float = field(init=False, default=0.0)
    torque_limit: float = field(init=False, default=0.0)
    max_linear_accel: float = field(init=False, default=0.0)
    max_angular_accel: float = field(init=False, default=0.0)
    root: RobotAnchorRef | None = field(init=False, default=None)
    gravity: WorldQuantity | None = field(init=False, default=None)
    gravity_value: ContextRef | None = field(init=False, default=None)
    end: RobotAnchorRef | None = field(init=False, default=None)

    def __post_init__(self):
        if not self.name:
            self.name = self.ref.solver.name
        NamedNamespaceObject.__init__(self, parent=self.parent, name=self.name)
        self._uri = self.ref.solver.uri
        self.robot = self.ref.solver.robot
        self.algorithm = self.ref.solver.algorithm
        self.damping = self.ref.solver.damping
        self.torque_limit = self.ref.solver.torque_limit
        self.max_linear_accel = self.ref.solver.max_linear_accel
        self.max_angular_accel = self.ref.solver.max_angular_accel
        self.root = self.ref.solver.root
        self.gravity = self.ref.solver.gravity
        self.gravity_value = self.ref.solver.gravity_value
        self.end = self.ref.solver.end


def _resolved_controller(item: ControllerEntry | ControllerAlias) -> ControllerEntry:
    return item.ref.controller if isinstance(item, ControllerAlias) else item


def _resolved_solver(item: SolverEntry | SolverAlias) -> SolverEntry:
    return item.ref.solver if isinstance(item, SolverAlias) else item
