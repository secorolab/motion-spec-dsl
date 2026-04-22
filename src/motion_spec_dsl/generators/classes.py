# SPDX-License-Identifier: MPL-2.0
"""Domain classes for the motion_spec DSL grammar constructs."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

from rdflib.namespace import Namespace

from motion_spec_dsl.generators.common import (
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
        specs: list[RobotSpec | MotionSpec | ConstraintHandler] | None = None,
        **_,
    ):
        self.imports = imports or []
        self.namespaces = namespaces or []
        self.specs = specs or []


class RobotType(StrEnum):
    Manipulator = "Manipulator"
    MobileBase = "MobileBase"
    MobileManipulator = "MobileManipulator"


@dataclass
class RobotSpec(IHasNamespaceDeclare):
    parent: object
    ns: NamespaceDeclLike
    name: str
    type: RobotType
    urdf: str
    base: RobotBaseComponent | None = None
    chain: RobotChainComponent | None = None
    manipulators: list[RobotManipulatorComponent] = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)
        self.type = RobotType(self.type)


@dataclass
class RobotBaseComponent(NamedNamespaceObject):
    parent: object
    root: str
    name: str = "base"

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


@dataclass
class RobotChainComponent(NamedNamespaceObject):
    parent: object
    root: str
    end: str = ""
    name: str = "chain"

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


@dataclass
class RobotManipulatorComponent(NamedNamespaceObject):
    parent: object
    name: str
    root: str
    end: str

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


@dataclass
class RobotRef:
    component: RobotComponentRef | None = None
    robot: RobotSpec | None = None
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def robot_spec(self) -> RobotSpec:
        if self.component is not None:
            return self.component.robot
        if self.robot is None:
            raise AttributeError("RobotRef is not resolved yet")
        return self.robot

    @property
    def component_name(self) -> str | None:
        return self.component.component if self.component is not None else None

    @property
    def name(self) -> str:
        if self.component is not None:
            return str(self.component)
        return self.robot.name if self.robot is not None else ""

    def __str__(self) -> str:
        return self.name


@dataclass
class RobotComponentRef:
    robot: RobotSpec
    component: str
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def name(self) -> str:
        return f"{self.robot.name}.{self.component}"

    def __str__(self) -> str:
        return self.name


@dataclass
class RobotAnchorRef:
    anchor: str
    component: RobotComponentRef | None = None
    robot: RobotSpec | None = None
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def name(self) -> str:
        if self.component is not None:
            return f"{self.component}.{self.anchor}"
        if self.robot is None:
            return ""
        return f"{self.robot.name}.chain.{self.anchor}"

    @property
    def robot_spec(self) -> RobotSpec:
        if self.component is not None:
            return self.component.robot
        if self.robot is None:
            raise AttributeError("RobotAnchorRef is not resolved yet")
        return self.robot

    @property
    def component_name(self) -> str | None:
        return self.component.component if self.component is not None else None

    def __str__(self) -> str:
        return self.name


@dataclass
class MotionSpec(IHasNamespaceDeclare):
    parent: object
    ns: NamespaceDeclLike
    name: str
    move: str | None
    context: list[WorldContextDecl | PreContextDecl | SpecContextDecl | PostContextDecl]
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
class ValVarContextDecl(NamedNamespaceObject):
    kind = None

    parent: object
    name: str = ""
    declaration: list[ValueVariable | WorldQuantity] = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)

    @property
    def namespace(self):
        assert self.kind is not None, "ValVarContextDecl must have 'kind' defined"
        return Namespace(str(self.parent.namespace) + f"{self.parent.name}/{self.kind}/")


class WorldContextDecl(ValVarContextDecl):
    kind = "World"


class PreContextDecl(ValVarContextDecl):
    kind = "Pre"


class SpecContextDecl(ValVarContextDecl):
    kind = "Spec"


class PostContextDecl(ValVarContextDecl):
    kind = "Post"


@dataclass
class ConstraintSection(NamedNamespaceObject):
    kind = ""

    parent: object
    constraints: list[ConstraintSpecification] = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.kind)


class WhenSection(ConstraintSection):
    kind = "when"


class WhileSection(ConstraintSection):
    kind = "while"


class UntilSection(ConstraintSection):
    kind = "until"


class WorldQuantityType(StrEnum):
    Frame          = "Frame"
    Pose           = "Pose"
    VelocityTwist  = "VelocityTwist"
    Wrench         = "Wrench"
    KinematicChain = "KinematicChain"
    Link           = "Link"
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
    LinearDistance   = "LinearDistance"
    AngularDistance  = "AngularDistance"
    LinearVelocity   = "LinearVelocity"
    AngularVelocity  = "AngularVelocity"
    Force            = "Force"
    Torque           = "Torque"
    Vector           = "Vector"


@dataclass
class ValueVariable(NamedNamespaceObject):
    parent: object
    name: str
    type: QuantityType
    value: ScalarQuantity | VectorQuantity | None = None

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
        self.type = QuantityType(self.type)


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


@dataclass
class View:
    parent: object
    quantity: WorldQuantity
    subspace: SubSpace | None = None
    axis: Axis | None = None

    def __post_init__(self):
        if isinstance(self.subspace, str):
            self.subspace = SubSpace(self.subspace)
        if self.axis is not None and isinstance(self.axis, str):
            self.axis = Axis(self.axis)


@dataclass
class ContextRef:
    valRef: ValueVariable
    quantityValue: ScalarQuantity | VectorQuantity | None = None
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def variable(self) -> str:
        return self.value.name


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
    context: list[WorldContextDecl | SpecContextDecl]
    motion: MotionSpec
    solvers: list[SolverEntry]
    monitors: list[MonitorEntry] = field(default_factory=list)
    controllers: list[ControllerEntry] = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)


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
    type: str
    params: ControllerParams
    command_type: QuantityType | None = None
    apply_at: WorldQuantity | None = None

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
        if self.command_type is not None and isinstance(self.command_type, str):
            self.command_type = QuantityType(self.command_type)


@dataclass
class ControllerParams:
    constraint: ConstraintRef
    solver: SolverEntry
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    decay: float | None = None
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def constraint_name(self) -> str:
        return self.constraint.name


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


class DerivedEntity(NamedNamespaceObject):
    """Base class for entities derived during graph generation (not from DSL)."""
    pass


@dataclass
class ScalarView(DerivedEntity):
    parent: object
    name: str
    quantity_name: str
    prop: str
    axis: str | None
    scalar_type: str
    view_type: object
    subspace: str | None

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


@dataclass
class ErrorSignal(DerivedEntity):
    parent: object
    name: str
    scalar_type: str

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


@dataclass
class AccelerationEnergy(DerivedEntity):
    parent: object
    name: str

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


@dataclass
class Motion(DerivedEntity):
    parent: object
    name: str
    motion_spec_block: object

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


@dataclass
class Evaluator(DerivedEntity):
    parent: object
    name: str
    constraint: object
    error_signal: object

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


@dataclass
class ConstraintHandlera(DerivedEntity):
    parent: object
    name: str
    motion: Motion
    evaluators: list[Evaluator] = field(default_factory=list)
    controllers: list[ControllerEntry] = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
