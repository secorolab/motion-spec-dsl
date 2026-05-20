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
    body: str = ""

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
        self.type = EnvironmentAssetType(self.type)


@dataclass
class EnvironmentAttachEntry:
    parent: object
    attachment: EnvironmentAsset
    attach_to: str
    entries: list = field(default_factory=list)


@dataclass
class EnvironmentAttachmentPrefixEntry:
    parent: object
    value: str


@dataclass
class EnvironmentAttachmentPositionEntry:
    parent: object
    value: PositionValue


@dataclass
class EnvironmentAttachmentOrientationEntry:
    parent: object
    value: OrientationValue


@dataclass
class EnvironmentAttachmentActuatorEntry:
    parent: object
    value: str


@dataclass
class EnvironmentAttachmentOpenCommandEntry:
    parent: object
    value: float


@dataclass
class EnvironmentAttachmentCloseCommandEntry:
    parent: object
    value: float


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
class EnvironmentToolBodyEntry:
    parent: object
    value: str


@dataclass
class EnvironmentTcpSiteEntry:
    parent: object
    value: str


@dataclass
class EnvironmentBodyEntry:
    parent: object
    value: str


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
    name: str
    asset: EnvironmentAsset
    entries: list = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)

    @property
    def chain(self) -> EnvironmentChainEntry | None:
        return next((entry for entry in self.entries if isinstance(entry, EnvironmentChainEntry)), None)

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
class EnvironmentSpec(IHasNamespaceDeclare):
    parent: object
    ns: NamespaceDeclLike
    name: str
    runtime: EnvironmentRuntime
    assets: list[EnvironmentAsset] = field(default_factory=list)
    assembly: list[EnvironmentAssembly] = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)
        self.runtime = EnvironmentRuntime(self.runtime)


@dataclass
class EnvironmentRobotRef:
    parent: object
    environment: EnvironmentSpec
    assembly: str

    @property
    def assembly_spec(self) -> EnvironmentAssembly | None:
        return next((entry for entry in self.environment.assembly if entry.name == self.assembly), None)

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
    goal: object   # ContextRef
    alpha: object  # ContextRef


@dataclass
class TrajectoryValue:
    parent: object
    lerp: LerpSpec


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
        WorldContextDecl
        | PreContextDecl
        | SpecContextDecl
        | PostContextDecl
        | ContextDeclReference
    ]
    sections: list[WhenSection | WhileSection | UntilSection]

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)
        self.when = self._section("when")
        self.while_ = self._section("while")
        self.until = self._section("until")
        assert len(self.while_.constraints) > 0, "MotionSpec must have at least one 'while' constraint"

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


class WhenSection(ConstraintSection):
    kind = "when"


class WhileSection(ConstraintSection):
    kind = "while"


@dataclass
class UntilSection(ConstraintSection):
    kind = "until"
    logic: str | None = None


class WorldQuantityType(StrEnum):
    Frame          = "Frame"
    Pose           = "Pose"
    VelocityTwist  = "VelocityTwist"
    Wrench         = "Wrench"
    JointPosition  = "JointPosition"
    KinematicChain = "KinematicChain"
    Link           = "Link"
    SceneObject    = "SceneObject"
    Gravity        = "Gravity"


@dataclass
class WorldQuantity(NamedNamespaceObject):
    parent: object
    name: str
    type: WorldQuantityType
    props: GeometricProps | None = None

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
        NamedNamespaceObject.__init__(self, parent=self.parent, name=self.name)
        self._uri = self.ref.uri
        self.type = self.ref.type
        self.props = self.ref.props


@dataclass
class WorldQuantityReference(WorldQuantityAlias):
    parent: object
    ref: WorldQuantity
    name: str = field(init=False)

    def __post_init__(self):
        self.name = self.ref.name
        super().__post_init__()


@dataclass
class GeometricProps:
    pairs: list[GeoPropPair]
    parent: object | None = field(default=None, repr=False, compare=False)


class GeometricPropKey(StrEnum):
    Of        = "of"
    Wrt       = "wrt"
    RefPoint  = "ref-point"
    AsSeenBy    = "as-seen-by"


@dataclass
class GeoPropPair:
    key: GeometricPropKey
    value: str = ""
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        self.key = GeometricPropKey(self.key)


class QuantityType(StrEnum):
    Pose                = "Pose"
    Position            = "Position"
    Orientation         = "Orientation"
    Distance            = "Distance"
    Angle               = "Angle"
    PlaneAngle          = "PlaneAngle"
    AngularDistance     = "AngularDistance"
    LinearVelocity      = "LinearVelocity"
    AngularVelocity     = "AngularVelocity"
    LinearAcceleration  = "LinearAcceleration"
    AngularAcceleration = "AngularAcceleration"
    Force               = "Force"
    Torque              = "Torque"
    Vector              = "Vector"
    Trajectory          = "Trajectory"
    TrajectoryProgress  = "TrajectoryProgress"


class HandlerControlMode(StrEnum):
    JointTorque = "JointTorque"


class ControllerType(StrEnum):
    PID = "PID"
    Impedance = "Impedance"
    ABAG = "ABAG"


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
    value: ScalarQuantity | VectorQuantity | SnapshotValue | TrajectoryValue | PoseValue | None = None

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
        if self.type == "LinearDistance":
            self.type = QuantityType.Distance
            return
        if self.type == "Trajectory":
            self.type = QuantityType.Trajectory
            return
        if self.type == "TrajectoryProgress":
            self.type = QuantityType.TrajectoryProgress
            return
        self.type = QuantityType(self.type)


@dataclass(kw_only=True)
class ContextQuantityAlias(ContextQuantity):
    parent: object
    name: str
    ref: ContextQuantity
    type: QuantityType = field(init=False)
    value: ScalarQuantity | VectorQuantity | SnapshotValue | None = field(init=False, default=None)

    def __post_init__(self):
        NamedNamespaceObject.__init__(self, parent=self.parent, name=self.name)
        self._uri = self.ref.uri
        self.type = self.ref.type
        self.value = self.ref.value


@dataclass
class ContextQuantityReference(ContextQuantityAlias):
    parent: object
    ref: ContextQuantity
    name: str = field(init=False)

    def __post_init__(self):
        self.name = self.ref.name
        super().__post_init__()


@dataclass
class ScalarQuantity:
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
    parent: object | None = field(default=None, repr=False, compare=False)


@dataclass
class ConstraintSpecification(NamedNamespaceObject):
    parent: object
    name: str
    view: View
    expr: EqualityConstraint | GreaterThanConstraint | LessThanConstraint | BilateralConstraint

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
class ConstraintAlias(NamedNamespaceObject):
    """Local name in a section that references a constraint from another motion."""
    parent: object
    name: str
    ref: ConstraintRef

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)

    @property
    def constraint(self) -> ConstraintSpecification:
        return self.ref.constraint


@dataclass
class ConstraintReference(ConstraintAlias):
    parent: object
    ref: ConstraintRef
    name: str = field(init=False)

    def __post_init__(self):
        self.name = self.ref.constraint.name
        super().__post_init__()


def _resolved_spec(item: ConstraintSpecification | ConstraintAlias) -> ConstraintSpecification:
    """Return the underlying ConstraintSpecification, resolving aliases."""
    return item.ref.constraint if isinstance(item, ConstraintAlias) else item


def _resolved_world_quantity(item: WorldQuantity | WorldQuantityAlias) -> WorldQuantity:
    return item.ref if isinstance(item, WorldQuantityAlias) else item


def _resolved_context_quantity(item: ContextQuantity | ContextQuantityAlias) -> ContextQuantity:
    return item.ref if isinstance(item, ContextQuantityAlias) else item


class SubSpace(StrEnum):
    Position       = "position"
    Orientation    = "orientation"
    LinVel         = "linvel"
    AngVel         = "angvel"
    LinAcc         = "linacc"
    AngAcc         = "angacc"
    Force          = "force"
    Torque         = "torque"


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
class ConstraintHandler(IHasNamespaceDeclare):
    parent: object
    ns: NamespaceDeclLike
    name: str
    context: list[WorldContextDecl | SpecContextDecl | ContextDeclReference]
    motion: MotionSpec
    control_mode: HandlerControlMode
    solvers: list[SolverEntry | SolverAlias]
    monitors: list[MonitorEntry] = field(default_factory=list)
    actions: list = field(default_factory=list)
    controllers: list[ControllerEntry | ControllerAlias] = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)
        self.control_mode = HandlerControlMode(self.control_mode)


@dataclass
class GripperAction:
    parent: object
    name: str
    attachment: EnvironmentAsset
    command: str


@dataclass
class MonitorEntry(NamedNamespaceObject):
    parent: object
    name: str
    constraint: ConstraintRef
    event: str = ""
    flag: str = ""

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)

    @property
    def constraint_name(self) -> str:
        return self.constraint.name


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
        NamedNamespaceObject.__init__(self, parent=self.parent, name=self.name)
        self._uri = self.ref.controller.uri
        self.type = self.ref.controller.type
        self.params = self.ref.controller.params
        self.solver = self.ref.controller.solver
        self.command_type = self.ref.controller.command_type
        self.apply_at = self.ref.controller.apply_at


@dataclass
class ControllerReference(ControllerAlias):
    parent: object
    ref: ControllerRef
    name: str = field(init=False)

    def __post_init__(self):
        self.name = self.ref.controller.name
        super().__post_init__()


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
        return any(gain is not None for gain in self.pid_gains)

    @property
    def has_impedance_terms(self) -> bool:
        return self.stiffness is not None or self.damping is not None


@dataclass
class SolverEntry(NamedNamespaceObject):
    parent: object
    name: str
    robot: RobotRef
    algorithm: str
    root: RobotAnchorRef
    gravity: WorldQuantity
    gravity_value: ContextRef
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
    root: RobotAnchorRef = field(init=False)
    gravity: WorldQuantity = field(init=False)
    gravity_value: ContextRef = field(init=False)
    end: RobotAnchorRef | None = field(init=False, default=None)

    def __post_init__(self):
        NamedNamespaceObject.__init__(self, parent=self.parent, name=self.name)
        self._uri = self.ref.solver.uri
        self.robot = self.ref.solver.robot
        self.algorithm = self.ref.solver.algorithm
        self.root = self.ref.solver.root
        self.gravity = self.ref.solver.gravity
        self.gravity_value = self.ref.solver.gravity_value
        self.end = self.ref.solver.end


@dataclass
class SolverReference(SolverAlias):
    parent: object
    ref: SolverRef
    name: str = field(init=False)

    def __post_init__(self):
        self.name = self.ref.solver.name
        super().__post_init__()


def _resolved_controller(item: ControllerEntry | ControllerAlias) -> ControllerEntry:
    return item.ref.controller if isinstance(item, ControllerAlias) else item


def _resolved_solver(item: SolverEntry | SolverAlias) -> SolverEntry:
    return item.ref.solver if isinstance(item, SolverAlias) else item
