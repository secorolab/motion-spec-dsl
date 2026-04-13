# SPDX-License-Identifier: MPL-2.0
"""Domain classes for the motion_spec DSL grammar constructs."""

from __future__ import annotations

from typing import Optional
from motion_spec_dsl.generators.common import (
    HasParentNamespace,
    IHasNamespaceDeclare,
    NamedNamespaceObject,
)


class NamespaceDeclare:
    def __init__(self, **kwargs):
        self.name: str = kwargs.get("name", "")
        self.uri: str = kwargs.get("uri", "")


class ImportDecl:
    def __init__(self, **kwargs):
        self.importURI: str = kwargs.get("importURI", "")


class Model:
    def __init__(self, **kwargs):
        self.imports: list[ImportDecl] = kwargs.get("imports", [])
        self.namespaces: list[NamespaceDeclare] = kwargs.get("namespaces", [])
        self.specs: list[MotionSpecBlock | ConstraintHandlerBlock] = kwargs.get("specs", [])


class MotionSpecBlock(IHasNamespaceDeclare):
    def __init__(
        self,
        parent,
        ns,
        name: str,
        move: str | None = None,
        spec: GuardedMotionSpecification | None = None,
    ):
        super().__init__(parent=parent, ns=ns, name=name)
        self.move: Optional[str] = move
        assert spec is not None
        self.spec = spec

    @property
    def motion_suffix(self) -> str:
        """Extract suffix from motion spec name (e.g., 'motion_find' -> 'find')"""
        assert self.name is not None
        return self.name.split("_")[-1]


class ConstraintHandlerBlock(IHasNamespaceDeclare):
    def __init__(
        self, parent, ns, name: str, spec: ConstraintHandlerSpecification | None = None
    ):
        super().__init__(parent=parent, ns=ns, name=name)
        assert spec is not None
        self.spec = spec


class GuardedMotionSpecification(HasParentNamespace):
    def __init__(self, **kwargs):
        context = kwargs.get("context")
        assert context is not None
        self.context: MotionContext = context
        self.when: list[ConstraintSpecification] = kwargs.get("when", [])
        self.while_: list[ConstraintSpecification] = kwargs.get("while_", [])
        self.until: list[ConstraintSpecification] = kwargs.get("until", [])


class MotionContext(HasParentNamespace):
    def __init__(self, **kwargs):
        self.items: list[
            UnitsContextDecl | WorldContextDecl | PreContextDecl | SpecContextDecl | PostContextDecl
        ] = kwargs.get("items", [])


class UnitsContextDecl:
    def __init__(self, **kwargs):
        self.name: str = kwargs.get("name", "")
        self.decl = kwargs.get("decl")


class WorldContextDecl(HasParentNamespace):
    def __init__(self, **kwargs):
        self.name: str = kwargs.get("name", "")
        decl = kwargs.get("decl")
        assert decl is not None
        self.decl: WorldDeclarationList = decl


class PreContextDecl(HasParentNamespace):
    def __init__(self, **kwargs):
        self.name: str = kwargs.get("name", "")
        decl = kwargs.get("decl")
        assert decl is not None
        self.decl: ValueDeclarationList = decl


class SpecContextDecl(HasParentNamespace):
    def __init__(self, **kwargs):
        self.name: str = kwargs.get("name", "")
        decl = kwargs.get("decl")
        assert decl is not None
        self.decl: ValueDeclarationList = decl


class PostContextDecl(HasParentNamespace):
    def __init__(self, **kwargs):
        self.name: str = kwargs.get("name", "")
        decl = kwargs.get("decl")
        assert decl is not None
        self.decl: ValueDeclarationList = decl


class WorldDeclarationList(HasParentNamespace):
    def __init__(self, **kwargs):
        self.declaration: list["WorldQuantity"] = kwargs.get("declaration", [])


class WorldQuantity(NamedNamespaceObject):
    def __init__(
        self,
        parent,
        name: str,
        type: str = "",
        props: GeometricProps | GravitationalFieldProps | None = None,
    ):
        super().__init__(parent=parent, name=name)
        self.type: str = type
        self.props = props

    @property
    def entity_abbrev(self) -> str:
        """Extract entity abbreviation from name (e.g., 'twist-ee-base' -> 'ee')"""
        parts = self.name.split("-")
        return parts[1] if len(parts) > 1 else self.name


class GeometricProps:
    def __init__(self, **kwargs):
        self.pairs: list = kwargs.get("pairs", [])


class GeoPropPair:
    def __init__(self, **kwargs):
        self.key: Optional[str] = kwargs.get("key")
        self.value: Optional[str] = kwargs.get("value")
        self.between: list[str] = kwargs.get("between", [])


class GravitationalFieldProps:
    def __init__(self, **kwargs):
        self.x: float = kwargs.get("x", 0.0)
        self.y: float = kwargs.get("y", 0.0)
        self.z: float = kwargs.get("z", 0.0)
        self.unit: str = kwargs.get("unit", "")


class ValueDeclarationList(HasParentNamespace):
    def __init__(self, **kwargs):
        self.declaration: list["ValueVariable"] = kwargs.get("declaration", [])


class ValueVariable(NamedNamespaceObject):
    def __init__(self, parent, name: str, type: str = "", value: Quantity | None = None):
        super().__init__(parent=parent, name=name)
        self.type: str = type
        assert value is not None
        self.value = value


class Quantity:
    def __init__(self, **kwargs):
        self.value: float = kwargs.get("value", 0.0)
        self.unit: str = kwargs.get("unit", "")


class ConstraintSpecification(NamedNamespaceObject):
    def __init__(
        self,
        parent,
        name: str,
        view: QuantityRef | None = None,
        expr: EqualityConstraint
        | GreaterThanConstraint
        | LessThanConstraint
        | BilateralConstraint
        | None = None,
    ):
        super().__init__(parent=parent, name=name)
        assert view is not None
        assert expr is not None
        self.view = view
        self.expr = expr


class QuantityRef:
    def __init__(self, **kwargs):
        self.quantity: str = kwargs.get("quantity", "")
        self.property: str = kwargs.get("property", "")
        self.axis: Optional[str] = kwargs.get("axis")


class EqualityConstraint:
    def __init__(self, **kwargs):
        reference = kwargs.get("reference")
        assert reference is not None
        self.reference: PreLookup | SpecLookup | PostLookup | WorldLookup = reference


class GreaterThanConstraint:
    def __init__(self, **kwargs):
        threshold = kwargs.get("threshold")
        assert threshold is not None
        self.threshold: PreLookup | SpecLookup | PostLookup | WorldLookup = threshold


class LessThanConstraint:
    def __init__(self, **kwargs):
        threshold = kwargs.get("threshold")
        assert threshold is not None
        self.threshold: PreLookup | SpecLookup | PostLookup | WorldLookup = threshold


class BilateralConstraint:
    def __init__(self, **kwargs):
        lower = kwargs.get("lower")
        upper = kwargs.get("upper")
        assert lower is not None
        assert upper is not None
        self.lower: PreLookup | SpecLookup | PostLookup | WorldLookup = lower
        self.upper: PreLookup | SpecLookup | PostLookup | WorldLookup = upper


class PreLookup:
    def __init__(self, **kwargs):
        self.variable: str = kwargs.get("variable", "")


class SpecLookup:
    def __init__(self, **kwargs):
        self.variable: str = kwargs.get("variable", "")


class PostLookup:
    def __init__(self, **kwargs):
        self.variable: str = kwargs.get("variable", "")


class WorldLookup:
    def __init__(self, **kwargs):
        self.variable: str = kwargs.get("variable", "")


class ConstraintHandlerSpecification(HasParentNamespace):
    def __init__(self, **kwargs):
        context = kwargs.get("context")
        assert context is not None
        self.context: ControllerContext = context
        self.motion: str = kwargs.get("motion", "")
        self.monitors: list[MonitorEntry] = kwargs.get("monitors", [])
        self.controllers: list[ControllerEntry] = kwargs.get("controllers", [])
        self.solver: SolverSpec | None = kwargs.get("solver")


class ControllerContext(HasParentNamespace):
    def __init__(self, **kwargs):
        self.items: list[CtrlWorldContextDecl] = kwargs.get("items", [])


class CtrlWorldContextDecl(HasParentNamespace):
    def __init__(self, **kwargs):
        self.name: str = kwargs.get("name", "")
        decl = kwargs.get("decl")
        assert decl is not None
        self.decl: CtrlWorldDeclarationList = decl


class CtrlWorldDeclarationList(HasParentNamespace):
    def __init__(self, **kwargs):
        self.declaration: list["CtrlWorldQuantity"] = kwargs.get("declaration", [])


class CtrlWorldQuantity(NamedNamespaceObject):
    def __init__(
        self,
        parent,
        name: str,
        type: str = "",
        props: GeometricProps | GravitationalFieldProps | None = None,
    ):
        super().__init__(parent=parent, name=name)
        self.type: str = type
        self.props = props


class MonitorEntry(NamedNamespaceObject):
    def __init__(self, parent, constraint: str = "", event: str = "", flag: str = ""):
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
        type: str = "",
        params: ControllerParams | None = None,
        output_type: str = "",
        apply_at: str = "",
        feed_scope: str = "",
        feed_kind: str = "",
    ):
        super().__init__(parent=parent, name=name)
        self.type: str = type
        assert params is not None
        self.params = params
        self.output_type: str = output_type
        self.apply_at: str = apply_at
        self.feed_scope: str = feed_scope
        self.feed_kind: str = feed_kind


class ControllerParams:
    def __init__(self, **kwargs):
        self.constraint: str = kwargs.get("constraint", "")
        self.kp: float = kwargs.get("kp", 0.0)
        self.ki: float = kwargs.get("ki", 0.0)
        self.kd: float = kwargs.get("kd", 0.0)
        self.decay: Optional[float] = kwargs.get("decay")


class SolverSpec:
    def __init__(self, **kwargs):
        self.algorithm: str = kwargs.get("algorithm", "")
        self.chain: str = kwargs.get("chain", "")
        self.root: str = kwargs.get("root", "")
        self.gravity: str = kwargs.get("gravity", "")
        self.cartesian_force: list[str] = kwargs.get("cartesian_force", [])
        self.joint_force: list[str] = kwargs.get("joint_force", [])
        self.velocity_solvers: list[VelocitySolverEntry] = kwargs.get("velocity_solvers", [])
        self.force_solvers: list[ForceSolverEntry] = kwargs.get("force_solvers", [])


class VelocitySolverEntry(NamedNamespaceObject):
    def __init__(self, parent, name: str, configuration: str = "", velocity: str = ""):
        super().__init__(parent=parent, name=name)
        self.configuration: str = configuration
        self.velocity: str = velocity


class ForceSolverEntry(NamedNamespaceObject):
    def __init__(self, parent, name: str, configuration: str = "", force: str = ""):
        super().__init__(parent=parent, name=name)
        self.configuration: str = configuration
        self.force: str = force


class DerivedEntity(NamedNamespaceObject):
    """Base class for entities derived during graph generation (not from DSL)."""

    pass


class ScalarView(DerivedEntity):
    """View that maps a world quantity property+axis to a scalar."""

    def __init__(self, parent, name, quantity_name, prop, axis, scalar_type, view_type, subspace):
        super().__init__(parent=parent, name=name)
        self.quantity_name = quantity_name
        self.prop = prop
        self.axis = axis
        self.scalar_type = scalar_type
        self.view_type = view_type
        self.subspace = subspace


class ErrorSignal(DerivedEntity):
    """Error signal quantity for constraint evaluation."""

    def __init__(self, parent, name, scalar_type):
        super().__init__(parent=parent, name=name)
        self.scalar_type = scalar_type


class AccelerationEnergy(DerivedEntity):
    """Acceleration energy quantity for control."""

    def __init__(self, parent, name):
        super().__init__(parent=parent, name=name)


class Motion(DerivedEntity):
    """Motion entity derived from MotionSpecBlock."""

    def __init__(self, parent, name, motion_spec_block):
        super().__init__(parent=parent, name=name)
        self.motion_spec = motion_spec_block


class Evaluator(DerivedEntity):
    """Constraint evaluator."""

    def __init__(self, parent, name, constraint, error_signal):
        super().__init__(parent=parent, name=name)
        self.constraint = constraint
        self.error_signal = error_signal


class ConstraintHandler(DerivedEntity):
    """Handler for a guarded motion."""

    def __init__(self, parent, name, motion):
        super().__init__(parent=parent, name=name)
        self.motion = motion
        self.evaluators = []
        self.controllers = []
        self.monitors = []
