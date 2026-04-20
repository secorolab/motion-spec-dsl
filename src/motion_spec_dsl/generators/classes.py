# SPDX-License-Identifier: MPL-2.0
"""Domain classes for the motion_spec DSL grammar constructs."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, List, Any

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
        imports: Optional[List[Import]] = None,
        namespaces: Optional[List[NamespaceDeclare]] = None,
        specs: Optional[List[Any]] = None,
        **_,
    ):
        self.imports: List[Import] = imports or []
        self.namespaces: List[NamespaceDeclare] = namespaces or []
        self.specs: List[MotionSpec | ConstraintHandler] = specs or []


@dataclass
class MotionSpec(IHasNamespaceDeclare):
    parent: object
    ns: NamespaceDeclLike
    name: str
    move: Optional[str]
    context: List[Any] = field(default_factory=list)
    when: Any = None
    while_: Any = None
    until: Any = None

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)


@dataclass
class ConstraintHandler(IHasNamespaceDeclare):
    parent: object
    ns: NamespaceDeclLike
    name: str
    motion: MotionSpec
    solver: SolverSpec
    context: List[Any] = field(default_factory=list)
    monitors: List[MonitorEntry] = field(default_factory=list)
    controllers: List[ControllerEntry] = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)
        while_constraints = self.motion.while_.constraints if self.motion.while_ else []
        when_constraints = self.motion.when.constraints if self.motion.when else []
        until_constraints = self.motion.until.constraints if self.motion.until else []
        if len(while_constraints) > 0 and len(self.controllers) == 0:
            raise ValueError(
                "ConstraintHandler with 'while' constraints must have at least one controller"
            )
        if (len(when_constraints) > 0 or len(until_constraints) > 0) and len(self.monitors) == 0:
            raise ValueError(
                "ConstraintHandler with 'when' or 'until' constraints must have at least one monitor"
            )


class WorldContextDecl:
    kind = "World"

    def __init__(self, parent, label: str, decl: WorldDeclarationList, **_):
        self.parent = parent
        self.label = label
        self.decl: WorldDeclarationList = decl

    @property
    def namespace(self):
        return self.parent.namespace

    @property
    def name(self) -> str:
        return self.parent.name


class PreContextDecl:
    kind = "Pre"

    def __init__(self, parent, label: str, decl: ValueDeclarationList, **_):
        self.parent = parent
        self.label = label
        self.decl: ValueDeclarationList = decl

    @property
    def namespace(self):
        return self.parent.namespace

    @property
    def name(self) -> str:
        return self.parent.name


class SpecContextDecl:
    kind = "Spec"

    def __init__(self, parent, label: str, decl: ValueDeclarationList, **_):
        self.parent = parent
        self.label = label
        self.decl: ValueDeclarationList = decl

    @property
    def namespace(self):
        return self.parent.namespace

    @property
    def name(self) -> str:
        return self.parent.name


class PostContextDecl:
    kind = "Post"

    def __init__(self, parent, label: str, decl: ValueDeclarationList, **_):
        self.parent = parent
        self.label = label
        self.decl: ValueDeclarationList = decl

    @property
    def namespace(self):
        return self.parent.namespace

    @property
    def name(self) -> str:
        return self.parent.name


class WhenSection(NamedNamespaceObject):
    kind = "when"

    def __init__(self, parent, constraints: Optional[List[ConstraintSpecification]] = None, **_):
        super().__init__(parent=parent, name=self.__class__.kind)
        self.constraints: List[ConstraintSpecification] = constraints or []


class WhileSection(NamedNamespaceObject):
    kind = "while"

    def __init__(self, parent, constraints: Optional[List[ConstraintSpecification]] = None, **_):
        super().__init__(parent=parent, name=self.__class__.kind)
        self.constraints: List[ConstraintSpecification] = constraints or []


class UntilSection(NamedNamespaceObject):
    kind = "until"

    def __init__(self, parent, constraints: Optional[List[ConstraintSpecification]] = None, **_):
        super().__init__(parent=parent, name=self.__class__.kind)
        self.constraints: List[ConstraintSpecification] = constraints or []


class WorldDeclarationList(NamedNamespaceObject):
    def __init__(self, parent, declaration: List[WorldQuantity], **_):
        super().__init__(parent=parent, name=parent.kind)
        self.declaration: List[WorldQuantity] = declaration


class WorldQuantity(NamedNamespaceObject):
    def __init__(
        self,
        parent,
        name: str = "",
        type: str = "",
        props: GeometricProps | None = None,
        **_,
    ):
        super().__init__(parent=parent, name=name)
        self.type: str = type
        self.props: GeometricProps | None = props

    @property
    def entity_abbrev(self) -> str:
        parts = self.name.split("-")
        return parts[1] if len(parts) > 1 else self.name


class GeometricProps:
    def __init__(self, pairs: List[GeoPropPair], **_):
        self.pairs: List[GeoPropPair] = pairs


class GeoPropPair:
    def __init__(self, key: str = "", value: str = "", **_):
        self.key: str = key
        self.value: str = value


class ValueDeclarationList:
    def __init__(self, parent, declaration: List[ValueVariable], **_):
        self.parent = parent
        self.declaration: List[ValueVariable] = declaration

    @property
    def namespace(self):
        return Namespace(self.parent.namespace + self.parent.name + "/")

    @property
    def name(self) -> str:
        return self.parent.kind


class ValueVariable(NamedNamespaceObject):
    def __init__(self, parent, name: str = "", type: str = "", value: Any = None, **_):
        super().__init__(parent=parent, name=name)
        self.type: str = type
        self.value: Any = value


class ScalarQuantity:
    def __init__(self, value: float = 0.0, unit: str = "", **_):
        self.value: float = value
        self.unit: str = unit


class VectorQuantity:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, unit: str = "", **_):
        self.x: float = x
        self.y: float = y
        self.z: float = z
        self.unit: str = unit


class ConstraintSpecification(NamedNamespaceObject):
    def __init__(self, parent, name: str, view: QuantityRef, expr: Any, **_):
        super().__init__(parent=parent, name=name)
        self.view: QuantityRef = view
        self.expr: EqualityConstraint | GreaterThanConstraint | LessThanConstraint | BilateralConstraint = expr


class QuantityRef(NamedNamespaceObject):
    def __init__(self, parent, quantity: WorldQuantity, property: List[str], axis: List[str], **_):
        super().__init__(parent=parent, name="")
        self.quantity: WorldQuantity = quantity
        self.property: List[str] = property
        self.axis: List[str] = axis


class EqualityConstraint:
    def __init__(self, reference: Any = None, **_):
        assert reference is not None
        self.reference: PreLookup | SpecLookup | PostLookup = reference


class GreaterThanConstraint:
    def __init__(self, threshold: Any = None, **_):
        assert threshold is not None
        self.threshold: PreLookup | SpecLookup | PostLookup = threshold


class LessThanConstraint:
    def __init__(self, threshold: Any = None, **_):
        assert threshold is not None
        self.threshold: PreLookup | SpecLookup | PostLookup = threshold


class BilateralConstraint:
    def __init__(self, lower: Any = None, upper: Any = None, **_):
        assert lower is not None
        assert upper is not None
        self.lower: PreLookup | SpecLookup | PostLookup = lower
        self.upper: PreLookup | SpecLookup | PostLookup = upper


class PreLookup:
    def __init__(self, variable: str = "", value: Any = None, **_):
        self.variable: str = variable
        self.value: ScalarQuantity | VectorQuantity | None = value


class SpecLookup:
    def __init__(self, variable: str = "", value: Any = None, **_):
        self.variable: str = variable
        self.value: ScalarQuantity | VectorQuantity | None = value


class PostLookup:
    def __init__(self, variable: str = "", value: Any = None, **_):
        self.variable: str = variable
        self.value: ScalarQuantity | VectorQuantity | None = value


class MonitorEntry(NamedNamespaceObject):
    def __init__(self, parent, constraint: str = "", event: str = "", flag: str = "", **_):
        signal_name = event or flag
        name = f"mon-{signal_name}" if signal_name else f"mon-{constraint}"
        super().__init__(parent=parent, name=name)
        self.constraint: str = constraint
        self.event: str = event
        self.flag: str = flag


class ControllerEntry(NamedNamespaceObject):
    def __init__(
        self,
        parent,
        name: str,
        type: str,
        params: ControllerParams,
        output_type: str = "",
        apply_at: str = "",
        feed_scope: str = "",
        feed_kind: str = "",
        **_,
    ):
        super().__init__(parent=parent, name=name)
        self.type: str = type
        self.params: ControllerParams = params
        self.output_type: str = output_type
        self.apply_at: str = apply_at
        self.feed_scope: str = feed_scope
        self.feed_kind: str = feed_kind


class ControllerParams:
    def __init__(
        self,
        constraint: str = "",
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        decay: Optional[float] = None,
        **_,
    ):
        self.constraint: str = constraint
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd
        self.decay: Optional[float] = decay


class SolverSpec:
    def __init__(
        self,
        parent,
        algorithm: str = "",
        chain: str = "",
        root: str = "",
        gravity: str = "",
        gravity_value: Any = None,
        velocity_solvers: Optional[List[VelocitySolverEntry]] = None,
        force_solvers: Optional[List[ForceSolverEntry]] = None,
        **_,
    ):
        self.parent = parent
        self.algorithm: str = algorithm
        self.chain: str = chain
        self.root: str = root
        self.gravity: str = gravity
        self.gravity_value: Any = gravity_value
        self.velocity_solvers: List[VelocitySolverEntry] = velocity_solvers or []
        self.force_solvers: List[ForceSolverEntry] = force_solvers or []

    @property
    def namespace(self):
        return self.parent.namespace

    @property
    def name(self) -> str:
        return self.parent.name


class VelocitySolverEntry(NamedNamespaceObject):
    def __init__(self, parent, name: str = "", configuration: str = "", velocity: str = "", **_):
        super().__init__(parent=parent, name=name)
        self.configuration: str = configuration
        self.velocity: str = velocity


class ForceSolverEntry(NamedNamespaceObject):
    def __init__(self, parent, name: str = "", configuration: str = "", force: str = "", **_):
        super().__init__(parent=parent, name=name)
        self.configuration: str = configuration
        self.force: str = force


class WorldLookup:
    def __init__(self, variable: str = "", **_):
        self.variable: str = variable


class DerivedEntity(NamedNamespaceObject):
    """Base class for entities derived during graph generation (not from DSL)."""
    pass


class ScalarView(DerivedEntity):
    def __init__(self, parent, name, quantity_name, prop, axis, scalar_type, view_type, subspace):
        super().__init__(parent=parent, name=name)
        self.quantity_name = quantity_name
        self.prop = prop
        self.axis = axis
        self.scalar_type = scalar_type
        self.view_type = view_type
        self.subspace = subspace


class ErrorSignal(DerivedEntity):
    def __init__(self, parent, name, scalar_type):
        super().__init__(parent=parent, name=name)
        self.scalar_type = scalar_type


class AccelerationEnergy(DerivedEntity):
    def __init__(self, parent, name):
        super().__init__(parent=parent, name=name)


class Motion(DerivedEntity):
    def __init__(self, parent, name, motion_spec_block):
        super().__init__(parent=parent, name=name)
        self.motion_spec = motion_spec_block


class Evaluator(DerivedEntity):
    def __init__(self, parent, name, constraint, error_signal):
        super().__init__(parent=parent, name=name)
        self.constraint = constraint
        self.error_signal = error_signal


class ConstraintHandlera(DerivedEntity):
    def __init__(self, parent, name, motion):
        super().__init__(parent=parent, name=name)
        self.motion = motion
        self.evaluators = []
        self.controllers = []
