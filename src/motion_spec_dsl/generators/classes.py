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
        specs: list[MotionSpec | ConstraintHandler] | None = None,
        **_,
    ):
        self.imports = imports or []
        self.namespaces = namespaces or []
        self.specs = specs or []


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
class WorldContextDecl(NamedNamespaceObject):
    kind = "World"

    parent: object
    name: str = ""
    declaration: list[WorldQuantity] = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)

    @property
    def namespace(self):
        return Namespace(str(self.parent.namespace) + f"{self.parent.name}/{self.kind}/")


@dataclass
class ValVarContextDecl(NamedNamespaceObject):
    kind = None

    parent: object
    name: str = ""
    declaration: list[ValueVariable] = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)

    @property
    def namespace(self):
        assert self.kind is not None, "ValVarContextDecl must have 'kind' defined"
        return Namespace(str(self.parent.namespace) + f"{self.parent.name}/{self.kind}/")


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


@dataclass
class GeoPropPair:
    key: str = ""
    value: str = ""
    parent: object | None = field(default=None, repr=False, compare=False)


@dataclass
class ValueVariable(NamedNamespaceObject):
    parent: object
    name: str = ""
    type: str = ""
    value: ScalarQuantity | VectorQuantity | None = None

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


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
    name: str = ""
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def motion_name(self) -> str:
        return self.motion.name

    def __str__(self) -> str:
        return self.name


class ViewProperty(StrEnum):
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
    property: ViewProperty
    axis: Axis | None = None

    def __post_init__(self):
        if isinstance(self.property, str):
            self.property = ViewProperty(self.property)
        if self.axis is not None and isinstance(self.axis, str):
            self.axis = Axis(self.axis)


@dataclass
class ContextRef:
    value: ValueVariable
    quantity: ScalarQuantity | VectorQuantity | None = None
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
    solver: SolverSpec
    context: list[WorldContextDecl | SpecContextDecl]
    motion: MotionSpec
    monitors: list[MonitorEntry] = field(default_factory=list)
    controllers: list[ControllerEntry] = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)
        if len(self.motion.while_.constraints) > 0 and len(self.controllers) == 0:
            raise ValueError(
                "ConstraintHandler with 'while' constraints must have at least one controller"
            )
        if (len(self.motion.when.constraints) > 0 or len(self.motion.until.constraints) > 0) and len(self.monitors) == 0:
            raise ValueError(
                "ConstraintHandler with 'when' or 'until' constraints must have at least one monitor"
            )


@dataclass
class MonitorEntry(NamedNamespaceObject):
    parent: object
    constraint: ConstraintRef
    event: str = ""
    flag: str = ""

    def __post_init__(self):
        signal_name = self.event or self.flag
        name = f"mon-{signal_name}" if signal_name else f"mon-{self.constraint}"
        super().__init__(parent=self.parent, name=name)

    @property
    def constraint_name(self) -> str:
        return self.constraint.name


@dataclass
class ControllerEntry(NamedNamespaceObject):
    parent: object
    name: str
    type: str
    params: ControllerParams
    output_type: str = ""
    apply_at: str = ""
    feed_scope: str = ""
    feed_kind: str = ""

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


@dataclass
class ControllerParams:
    constraint: ConstraintRef
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    decay: float | None = None
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def constraint_name(self) -> str:
        return self.constraint.name


@dataclass
class SolverSpec:
    parent: object
    algorithm: str
    chain: WorldQuantity
    root: WorldQuantity
    gravity: WorldQuantity
    gravity_value: ContextRef
    velocity_solvers: list[VelocitySolverEntry] = field(default_factory=list)
    force_solvers: list[ForceSolverEntry] = field(default_factory=list)

    @property
    def namespace(self):
        return self.parent.namespace

    @property
    def name(self) -> str:
        return self.parent.name


@dataclass
class VelocitySolverEntry(NamedNamespaceObject):
    parent: object
    name: str = ""
    configuration: str = ""
    velocity: str = ""

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


@dataclass
class ForceSolverEntry(NamedNamespaceObject):
    parent: object
    name: str = ""
    configuration: str = ""
    force: str = ""

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
