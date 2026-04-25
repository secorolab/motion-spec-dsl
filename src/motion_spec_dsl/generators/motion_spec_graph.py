# SPDX-License-Identifier: MPL-2.0
"""Build the JSON-LD output graphs for the motion-spec DSL.

The implementation is intentionally kept in a single file, but it is organized
around one cached analysis object. Emitters consume derived records rather than
re-walking the DSL model and recomputing identifiers.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass
from enum import StrEnum
from functools import cached_property
from typing import Any, TypeAlias

from rdflib.graph import Dataset
from rdflib.namespace import Namespace, RDF
from rdflib.term import Literal, URIRef
from textx.scoping import get_included_models

from motion_spec.namespace import (
    CSTR,
    CSTR_HDL,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_OP,
    GEOM_REL,
    KC,
    MAP,
    MOT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    QUDT_UNIT,
    RBDYN_COORD,
    RBDYN_ENT,
    SLV,
)
from motion_spec_dsl.generators.classes import (
    BilateralConstraint,
    ConstraintHandler,
    ConstraintSpecification,
    ControllerMode,
    ContextRef,
    EqualityConstraint,
    GeoPropPair,
    GeometricProps,
    GreaterThanConstraint,
    LessThanConstraint,
    Model,
    MotionSpec,
    PostContextDecl,
    PreContextDecl,
    QuantityType,
    SpecContextDecl,
    ScalarQuantity,
    ValueVariable,

    WorldContextDecl,
    WorldQuantity,
    WorldQuantityType,
    _resolved_spec,
    _resolved_solver,
)

Node: TypeAlias = Any
NamespaceBinding: TypeAlias = tuple[str, Any]
ContextLike: TypeAlias = dict[str, str] | list[str | dict[str, str]]
DatasetOutput: TypeAlias = tuple[Dataset, ContextLike]
ViewAxis: TypeAlias = str
WorldQuantityLike: TypeAlias = WorldQuantity
ViewKey: TypeAlias = tuple[str, "ViewProperty", ViewAxis]


class ViewProperty(StrEnum):
    ANGULAR = "angular"
    LINEAR = "linear"
    ROTATION = "rotation"
    DISTANCE = "distance"
    FORCE = "force"
    TORQUE = "torque"
    JOINT_POSITION = "joint-position"


class ConstraintKind(StrEnum):
    EQUALITY = "EqualityConstraint"
    GREATER_THAN = "GreaterThanConstraint"
    LESS_THAN = "LessThanConstraint"
    BILATERAL = "BilateralConstraint"


class SolverAlgorithm(StrEnum):
    ACHD = "ACHD"
    RNE = "RNE"


class SignalKind(StrEnum):
    ACCELERATION_ENERGY = "AccelerationEnergy"
    FORCE = "Force"
    GENERIC = "generic"


@dataclass(frozen=True)
class PropertySpec:
    scalar_type: QuantityType
    accel_prefix: str | None = None
    view_type: Any = None
    view_subspace: str | None = None
    accel_subspace: str | None = None


@dataclass(frozen=True)
class WorldSpec:
    rdf_types: tuple[Any, ...]
    qkinds: tuple[Any, ...]
    units: tuple[Any, ...]
    properties: dict[ViewProperty, PropertySpec]


@dataclass(frozen=True)
class MotionScope:
    motion_id: str
    motion: MotionSpec
    quantities: dict[str, WorldQuantity]
    values: dict[str, ValueVariable]
    constraints: tuple[ConstraintSpecification, ...]


@dataclass(frozen=True)
class ConstraintData:
    motion_id: str
    motion_name: str
    constraint: ConstraintSpecification
    quantity: WorldQuantity | None
    kind: ConstraintKind
    scalar_type: QuantityType | str
    quantity_node_id: str
    property_name: ViewProperty
    axis: str | None
    reference_var: str | None = None
    threshold_var: str | None = None
    lower_var: str | None = None
    upper_var: str | None = None
    error_signal_id: str | None = None
    shared: bool = False


@dataclass(frozen=True)
class ResolvedConstraintView:
    quantity_name: str
    property_name: ViewProperty
    axis: str | None
    quantity: WorldQuantity | None
    property_spec: PropertySpec | None
    scalar_type: QuantityType | str | None
    scalar_id: str | None


@dataclass(frozen=True)
class SignalDescriptor:
    node_id: str
    kind: SignalKind
    owner: Any
    scalar_type: QuantityType | str | None = None


@dataclass(frozen=True)
class ScalarViewRequirement:
    quantity_name: str
    property_name: ViewProperty
    axis: str
    scalar_type: QuantityType | str


@dataclass(frozen=True, kw_only=True)
class SolverInterface:
    node_id: str
    signal_id: str
    owner: Any
    solver_name: str | None = None


@dataclass(frozen=True)
class AccelerationConstraintInterface(SolverInterface):
    subspace: str
    axis: str


@dataclass(frozen=True)
class CartesianForceInterface(SolverInterface):
    force_name: str
    attached_to: str | None = None
    axis: str | None = None


@dataclass(frozen=True)
class JointForceInterface(SolverInterface):
    pass


@dataclass(frozen=True)
class VelocitySolverInterface(SolverInterface):
    quantity_name: str
    configuration: str


@dataclass(frozen=True)
class ForceSolverInterface(SolverInterface):
    quantity_name: str
    configuration: str


@dataclass(frozen=True)
class ControllerDispatch:
    solver_name: str | None
    signal: SignalDescriptor
    interfaces: tuple[SolverInterface, ...] = ()


WORLD_SPECS: dict[WorldQuantityType, WorldSpec] = {
    WorldQuantityType.VelocityTwist: WorldSpec(
        rdf_types=(
            GEOM_REL.VelocityTwist,
            GEOM_COORD.VelocityTwistCoordinate,
            GEOM_COORD.VectorXYZ,
        ),
        qkinds=(QUDT_QKIND.AngularVelocity, QUDT_QKIND.LinearVelocity),
        units=(QUDT_UNIT["RAD-PER-SEC"], QUDT_UNIT["M-PER-SEC"]),
        properties={
            ViewProperty.ANGULAR: PropertySpec(
                scalar_type=QuantityType.AngularVelocity,
                accel_prefix="ang",
                view_type=MAP.VelocityTwistCoordinateView,
                view_subspace="angular-velocity",
                accel_subspace="angular-acceleration",
            ),
            ViewProperty.LINEAR: PropertySpec(
                scalar_type=QuantityType.LinearVelocity,
                accel_prefix="lin",
                view_type=MAP.VelocityTwistCoordinateView,
                view_subspace="linear-velocity",
                accel_subspace="linear-acceleration",
            ),
        },
    ),
    WorldQuantityType.Wrench: WorldSpec(
        rdf_types=(RBDYN_ENT.Wrench, RBDYN_COORD.WrenchCoordinate, GEOM_COORD.VectorXYZ),
        qkinds=(QUDT_QKIND.Torque, QUDT_QKIND.Force),
        units=(QUDT_UNIT["N-M"], QUDT_UNIT.N),
        properties={
            ViewProperty.TORQUE: PropertySpec(
                scalar_type=QuantityType.Torque,
                view_type=MAP.WrenchCoordinateView,
                view_subspace="torque",
            ),
            ViewProperty.FORCE: PropertySpec(
                scalar_type=QuantityType.Force,
                view_type=MAP.WrenchCoordinateView,
                view_subspace="force",
            ),
        },
    ),
    WorldQuantityType.Pose: WorldSpec(
        rdf_types=(
            GEOM_REL.Pose,
            GEOM_COORD.PoseCoordinate,
            GEOM_COORD.DirectionCosineXYZ,
            GEOM_COORD.VectorXYZ,
        ),
        qkinds=(QUDT_QKIND.PlaneAngle, QUDT_QKIND.Length),
        units=(QUDT_UNIT.UNITLESS, QUDT_UNIT.M),
        properties={
            ViewProperty.ROTATION: PropertySpec(
                scalar_type=QuantityType.PlaneAngle,
                view_type=MAP.PoseCoordinateView,
                view_subspace="rotation",
            ),
            ViewProperty.DISTANCE: PropertySpec(
                scalar_type=QuantityType.Distance,
                view_type=MAP.PoseCoordinateView,
                view_subspace="position",
            ),
        },
    ),
    WorldQuantityType.JointPosition: WorldSpec(
        rdf_types=(QUDT_SCHEMA.Quantity,),
        qkinds=(QUDT_QKIND.PlaneAngle,),
        units=(QUDT_UNIT.RAD,),
        properties={},
    ),
}

CSTR_TYPE_NAME: dict[QuantityType | str, str] = {
    QuantityType.PlaneAngle: QuantityType.Angle,
}

SCALAR_UNIT: dict[QuantityType | str, Node] = {
    QuantityType.AngularVelocity: QUDT_UNIT["RAD-PER-SEC"],
    QuantityType.LinearVelocity: QUDT_UNIT["M-PER-SEC"],
    QuantityType.Torque: QUDT_UNIT["N-M"],
    QuantityType.Force: QUDT_UNIT.N,
    "Position": QUDT_UNIT.M,
    QuantityType.Angle: QUDT_UNIT["RAD"],
    QuantityType.PlaneAngle: QUDT_UNIT["RAD"],
    QuantityType.Distance: QUDT_UNIT.M,
}

DSL_UNIT = {
    "rad/s": QUDT_UNIT["RAD-PER-SEC"],
    "rad": QUDT_UNIT["RAD"],
    "m/s": QUDT_UNIT["M-PER-SEC"],
    "m": QUDT_UNIT.M,
    "Nm": QUDT_UNIT["N-M"],
    "N": QUDT_UNIT.N,
    "deg/s": QUDT_UNIT["DEG-PER-SEC"],
    "deg": QUDT_UNIT["DEG"],
    "cm/s": QUDT_UNIT["CentiM-PER-SEC"],
    "cm": QUDT_UNIT["CentiM"],
    "m/s2": QUDT_UNIT["M-PER-SEC2"],
}

CONSTRAINT_PATH_BY_PREFIX = {
    "geom-rel": "geometry/spatial-relations.ttl",
    "geom-coord": "geometry/coordinates.ttl",
    "geom-ent": "geometry/structural-entities.ttl",
    "rbdyn-ent": "newtonian-rigid-body-dynamics/structural-entities.ttl",
    "rbdyn-coord": "newtonian-rigid-body-dynamics/coordinates.ttl",
    "map": "task/map.ttl",
    "cstr": "task/constraint.ttl",
    "mot": "task/motion-specification.ttl",
    "cstr-hdl": "task/constraint-handler.ttl",
    "slv": "task/solver-specification.ttl",
}

GRAPH_BINDINGS: tuple[NamespaceBinding, ...] = (
    ("kc", KC),
    ("geom-ent", GEOM_ENT),
    ("geom-rel", GEOM_REL),
    ("geom-coord", GEOM_COORD),
    ("geom-op", GEOM_OP),
    ("rbdyn-ent", RBDYN_ENT),
    ("rbdyn-coord", RBDYN_COORD),
    ("qudt", QUDT_SCHEMA),
    ("qkind", QUDT_QKIND),
    ("unit", QUDT_UNIT),
    ("map", MAP),
    ("cstr", CSTR),
    ("mot", MOT),
    ("cstr-hdl", CSTR_HDL),
    ("slv", SLV),
)

WORLD_STRUCTURE_TYPES: dict[WorldQuantityType, Node] = {
    WorldQuantityType.Frame: GEOM_ENT.Frame,
    WorldQuantityType.Link: GEOM_ENT.SimplicialComplex,
    WorldQuantityType.KinematicChain: GEOM_ENT.KinematicChain,
    WorldQuantityType.Gravity: GEOM_ENT.UniformGravitationalField,
}


def _dataset(*bindings: NamespaceBinding) -> Dataset:
    dataset = Dataset()
    for prefix, namespace in bindings:
        dataset.bind(prefix, namespace)
    return dataset


def _add_types(graph: Any, node: Node, *rdf_types: Node) -> None:
    for rdf_type in rdf_types:
        graph.add((node, RDF.type, rdf_type))


def _add_quantity(graph: Any, node: Node, scalar_type: QuantityType | str) -> None:
    qkind = QUDT_QKIND[scalar_type]
    _add_types(graph, node, QUDT_SCHEMA.Quantity, qkind)
    graph.add((node, QUDT_SCHEMA["quantity-kind"], qkind))
    graph.add((node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(scalar_type, QUDT_UNIT.UNITLESS)))


def _property_spec(
    world_type: WorldQuantityType, property_name: ViewProperty
) -> PropertySpec | None:
    spec = WORLD_SPECS.get(world_type)
    return spec.properties.get(property_name) if spec else None


def _entity_id(value: Any) -> str:
    if value is None:
        return ""
    if hasattr(value, "uri"):
        return str(value.uri)
    if hasattr(value, "name"):
        return f"name:{value.name}"
    return str(value)


def _evaluator_id(spec: Any) -> str:
    section = getattr(spec, "parent", None)
    motion = getattr(section, "parent", None) if section is not None else None
    section_kind = getattr(section, "kind", None)
    motion_name = getattr(motion, "name", None)
    if motion_name and section_kind:
        return f"eval-{motion_name}-{section_kind}-{spec.name}"
    return f"eval-{spec.name}"


def _dsl_unit(unit_name: str) -> Node:
    try:
        return DSL_UNIT[unit_name]
    except KeyError as exc:
        supported = ", ".join(sorted(DSL_UNIT))
        raise ValueError(
            f"Unsupported DSL unit '{unit_name}'. Use compact DSL units only. "
            f"Supported units: {supported}."
        ) from exc



def _view_scalar_type(
    quantity: WorldQuantity, property_name: ViewProperty, axis: str | None
) -> QuantityType | str | None:
    if quantity.type == WorldQuantityType.JointPosition:
        return QuantityType.Angle
    if quantity.type == WorldQuantityType.Pose and property_name == ViewProperty.DISTANCE:
        return "Position" if axis is not None else "Length"
    prop = _property_spec(quantity.type, property_name)
    return prop.scalar_type if prop else None


def _scalar_id(
    quantity: WorldQuantity, property_name: ViewProperty, axis: str | None
) -> str | None:
    if quantity.type == WorldQuantityType.JointPosition and property_name == ViewProperty.JOINT_POSITION:
        return quantity.name
    if axis is None:
        return f"{quantity.name}.{property_name}"
    return f"{quantity.name}.{property_name}.{axis}"


def _controller_error_signal_id(
    motion_name: str,
    quantity: WorldQuantity,
    property_name: ViewProperty,
    axis: str | None,
    shared: bool,
) -> str | None:
    scalar_id = _scalar_id(quantity, property_name, axis)
    if scalar_id is None:
        return None
    # Shared WHILE equality constraints use the same error signal across motions.
    return scalar_id + "-err" if shared else f"{scalar_id}-err-{motion_name}"


def _acceleration_energy_id(
    motion_name: str,
    quantity: WorldQuantity,
    property_name: ViewProperty,
    axis: str | None,
    shared: bool,
) -> str | None:
    prop = _property_spec(quantity.type, property_name)
    if prop is None or prop.accel_prefix is None:
        return None
    stem = f"eacc-{quantity.name}.{property_name}{'.' + axis if axis else ''}"
    return stem if shared else f"{stem}-{motion_name}"


def _acceleration_constraint_id(
    motion_name: str,
    quantity: WorldQuantity | None,
    property_name: ViewProperty,
    axis: str | None,
    shared: bool,
) -> str:
    if quantity is None:
        stem = f"acc-cstr-{property_name}{'.' + axis if axis else ''}"
    else:
        stem = f"acc-cstr-{quantity.name}.{property_name}{'.' + axis if axis else ''}"
    return stem if shared else f"{stem}-{motion_name}"


def _geometric_property(props: GeometricProps | None, key: str) -> str | None:
    if props is None:
        return None
    for pair in props.pairs:
        if isinstance(pair, GeoPropPair) and pair.key == key:
            return pair.value
    return None


def _required_world_quantity(
    builder: "MotionSpecDatasetBuilder",
    name: str,
    reason: str,
    *,
    motion: MotionSpec | None = None,
    handler: ConstraintHandler | None = None,
) -> WorldQuantityLike:
    return builder.resolve_world_quantity(name, motion=motion, handler=handler, reason=reason)


def _node_name(value: Any) -> str:
    return value.name if hasattr(value, "name") else str(value)


def _view_property_name(constraint: ConstraintSpecification) -> ViewProperty:
    subspace = constraint.view.subspace
    quantity = constraint.view.quantity
    if subspace is None:
        if isinstance(quantity, WorldQuantity) and quantity.type == WorldQuantityType.JointPosition:
            return ViewProperty.JOINT_POSITION
        raise ValueError(f"Constraint '{constraint.name}' must define a view subspace.")
    value = getattr(subspace, "value", subspace)
    normalized = {
        "angvel": ViewProperty.ANGULAR,
        "linvel": ViewProperty.LINEAR,
        "orientation": ViewProperty.ROTATION,
        "position": ViewProperty.DISTANCE,
        "force": ViewProperty.FORCE,
        "torque": ViewProperty.TORQUE,
    }.get(str(value))
    if normalized is None:
        return ViewProperty(str(value))
    return normalized


def _resolve_constraint_view(
    builder: "MotionSpecDatasetBuilder",
    scope: MotionScope,
    constraint: ConstraintSpecification,
) -> ResolvedConstraintView:
    quantity_name = _node_name(constraint.view.quantity)
    property_name = _view_property_name(constraint)
    axis_value = constraint.view.axis
    axis = None if axis_value is None else str(getattr(axis_value, "value", axis_value))
    quantity = scope.quantities.get(quantity_name)
    if quantity is None:
        try:
            quantity = builder.resolve_world_quantity(
                constraint.view.quantity,
                motion=scope.motion,
                reason=f"constraint '{constraint.name}' view",
            )
        except ValueError:
            quantity = None
    property_spec = _property_spec(quantity.type, property_name) if quantity is not None else None
    scalar_type = (
        _view_scalar_type(quantity, property_name, axis) if quantity is not None else None
    ) or (property_spec.scalar_type if property_spec else property_name)
    scalar_id = _scalar_id(quantity, property_name, axis) if quantity is not None else None
    return ResolvedConstraintView(
        quantity_name=quantity_name,
        property_name=property_name,
        axis=axis,
        quantity=quantity,
        property_spec=property_spec,
        scalar_type=scalar_type,
        scalar_id=scalar_id,
    )


def _constraint_context_refs(constraint: ConstraintSpecification) -> list[ContextRef]:
    expr = constraint.expr
    if isinstance(expr, EqualityConstraint):
        return [expr.reference]
    if isinstance(expr, (GreaterThanConstraint, LessThanConstraint)):
        return [expr.threshold]
    if isinstance(expr, BilateralConstraint):
        return [expr.lower, expr.upper]
    return []


class MotionSpecDatasetBuilder:
    """Handler-rooted builder that resolves semantics and materializes a Dataset."""

    def __init__(self, model: Model):
        self.model = model
        self.models = get_included_models(model)
        self.dataset = _dataset(*GRAPH_BINDINGS)
        self.graph = self.dataset.default_graph

    @cached_property
    def namespace_owners(self) -> list[Any]:
        return [*self.authored_handlers, *[scope.motion for scope in self.motion_scope.values()]]

    @cached_property
    def default_namespace_owner(self) -> Any:
        owner = next(iter(self.namespace_owners), None)
        assert owner is not None, "No MOTION_SPEC or CONSTRAINT_HANDLER found in model"
        return owner

    def _namespace_owner(self, obj: Any | None) -> Any:
        current = obj
        while current is not None:
            if hasattr(current, "ns") and hasattr(current, "ns_prefix"):
                return current
            current = getattr(current, "parent", None)
        return self.default_namespace_owner

    def root_uri(self, name: str, *, owner: Any | None = None) -> URIRef:
        ns_owner = self._namespace_owner(owner)
        return URIRef(Namespace(ns_owner.ns.uri)[name])

    def node(self, value: Any, *, owner: Any | None = None) -> URIRef:
        if hasattr(value, "uri"):
            return URIRef(str(value.uri))
        return self.root_uri(_node_name(value), owner=owner)

    def context(self, *bindings: NamespaceBinding) -> dict[str, str]:
        context: dict[str, str] = {}
        for owner in self.namespace_owners:
            context.setdefault(owner.ns_prefix, owner.ns.uri)
        for prefix, namespace in bindings:
            context[prefix] = str(namespace._NS)
        return context

    def all_constraints(self, spec: MotionSpec) -> list[ConstraintSpecification]:
        return [
            *(_resolved_spec(constraint) for constraint in spec.when.constraints),
            *(_resolved_spec(constraint) for constraint in spec.while_.constraints),
            *(_resolved_spec(constraint) for constraint in spec.until.constraints),
        ]

    def scope_for_motion(self, motion: MotionSpec | None) -> MotionScope | None:
        if motion is None:
            return None
        return self.motion_scope.get(_entity_id(motion))

    def constraint_data_for_motion(
        self,
        motion: MotionSpec | None,
        constraint_ref: Any,
    ) -> ConstraintData | None:
        if motion is None:
            return None
        target = getattr(constraint_ref, "constraint", constraint_ref)
        motion_id = _entity_id(motion)
        for constraint in self.constraints_by_motion.get(motion_id, ()):
            if constraint.constraint is target:
                return constraint
        return None

    def _decode_acceleration_interface(
        self,
        motion_spec: MotionSpec,
        handler_spec: ConstraintHandler,
        controller: Any,
        constraint: ConstraintData,
    ) -> ControllerDispatch | None:
        solver = self.controller_solver(handler_spec, controller)
        solver_name = getattr(solver, "name", None)
        algorithm = getattr(solver, "algorithm", None)
        property_spec = (
            _property_spec(constraint.quantity.type, constraint.property_name)
            if constraint.quantity is not None
            else None
        )
        if (
            algorithm != SolverAlgorithm.ACHD
            or constraint.quantity is None
            or constraint.axis is None
            or property_spec is None
            or property_spec.accel_prefix is None
            or property_spec.accel_subspace is None
        ):
            return None
        control_signal_id = _acceleration_energy_id(
            motion_spec.name,
            constraint.quantity,
            constraint.property_name,
            constraint.axis,
            constraint.shared,
        )
        if control_signal_id is None:
            return None
        return ControllerDispatch(
            solver_name=solver_name,
            signal=SignalDescriptor(
                node_id=control_signal_id,
                kind=SignalKind.ACCELERATION_ENERGY,
                owner=motion_spec,
                scalar_type="AccelerationEnergy",
            ),
            interfaces=(
                AccelerationConstraintInterface(
                    node_id=_acceleration_constraint_id(
                        motion_spec.name,
                        constraint.quantity,
                        constraint.property_name,
                        constraint.axis,
                        constraint.shared,
                    ),
                    signal_id=control_signal_id,
                    owner=motion_spec,
                    solver_name=solver_name,
                    subspace=property_spec.accel_subspace,
                    axis=constraint.axis,
                ),
            ),
        )

    def _decode_cartesian_force_interface(
        self,
        motion_spec: MotionSpec,
        handler_spec: ConstraintHandler,
        controller: Any,
        constraint: ConstraintData,
    ) -> ControllerDispatch | None:
        solver = self.controller_solver(handler_spec, controller)
        solver_name = getattr(solver, "name", None)
        command_type = (
            str(getattr(controller.command_type, "value", controller.command_type))
            if getattr(controller, "command_type", None)
            else None
        )
        if (
            constraint.quantity is None
            or (
                constraint.property_name != ViewProperty.FORCE
                and command_type != QuantityType.Force
            )
        ):
            return None
        return ControllerDispatch(
            solver_name=solver_name,
            signal=SignalDescriptor(
                node_id=constraint.quantity_node_id,
                kind=SignalKind.FORCE,
                owner=motion_spec,
                scalar_type=QuantityType.Force,
            ),
            interfaces=(
                CartesianForceInterface(
                    node_id=f"spec-{constraint.quantity.name}",
                    signal_id=constraint.quantity_node_id,
                    owner=motion_spec,
                    solver_name=solver_name,
                    force_name=constraint.quantity.name,
                    attached_to=(
                        str(controller.apply_at.uri)
                        if getattr(controller, "apply_at", None) and hasattr(controller.apply_at, "uri")
                        else None
                    ),
                    axis=constraint.axis if constraint.property_name == ViewProperty.FORCE else None,
                ),
            ),
        )

    def _decode_joint_force_interface(
        self,
        motion_spec: MotionSpec,
        handler_spec: ConstraintHandler,
        controller: Any,
        constraint: ConstraintData,
    ) -> ControllerDispatch | None:
        del motion_spec
        solver = self.controller_solver(handler_spec, controller)
        solver_name = getattr(solver, "name", None)
        algorithm = getattr(solver, "algorithm", None)
        command_type = getattr(controller, "command_type", None)
        control_mode = getattr(controller, "control_mode", None)
        quantity = constraint.quantity
        if (
            algorithm != SolverAlgorithm.ACHD
            or command_type != QuantityType.Torque
            or control_mode != ControllerMode.Posture
            or quantity is None
            or quantity.type != WorldQuantityType.JointPosition
        ):
            return None
        signal_id = f"tau-{controller.name}"
        return ControllerDispatch(
            solver_name=solver_name,
            signal=SignalDescriptor(
                node_id=signal_id,
                kind=SignalKind.GENERIC,
                owner=handler_spec,
                scalar_type=QuantityType.Torque,
            ),
            interfaces=(
                JointForceInterface(
                    node_id=signal_id,
                    signal_id=signal_id,
                    owner=handler_spec,
                    solver_name=solver_name,
                ),
            ),
        )

    def _controller_interface_decoders(self) -> tuple[Any, ...]:
        return (
            self._decode_acceleration_interface,
            self._decode_cartesian_force_interface,
            self._decode_joint_force_interface,
        )

    def _solver_interface_priority(self, interface: SolverInterface) -> int:
        return 0 if isinstance(interface.owner, MotionSpec) else 1

    def _decode_controller_dispatch(
        self,
        motion_spec: MotionSpec,
        handler_spec: ConstraintHandler,
        controller: Any,
        constraint: ConstraintData,
    ) -> ControllerDispatch:
        for decoder in self._controller_interface_decoders():
            dispatch = decoder(motion_spec, handler_spec, controller, constraint)
            if dispatch is not None:
                return dispatch

        return ControllerDispatch(
            solver_name=getattr(self.controller_solver(handler_spec, controller), "name", None),
            signal=SignalDescriptor(
                node_id=f"eacc-{controller.name}",
                kind=SignalKind.GENERIC,
                owner=motion_spec,
            ),
        )

    def controller_dispatches(
        self, handler_spec: ConstraintHandler
    ) -> tuple[tuple[Any, ConstraintData, ControllerDispatch], ...]:
        motion = getattr(handler_spec, "motion", None)
        motion_id = _entity_id(motion)
        scope = self.motion_scope.get(motion_id)
        if scope is None:
            return ()
        dispatches: list[tuple[Any, ConstraintData, ControllerDispatch]] = []
        for controller in getattr(handler_spec, "controllers", []):
            constraint = self.constraint_data_for_motion(scope.motion, controller.params.constraint)
            if constraint is None:
                raise ValueError(
                    f"Controller '{controller.name}' references constraint "
                    f"'{controller.params.constraint}' that is not assembled in motion '{scope.motion.name}'."
                )
            dispatches.append(
                (
                    controller,
                    constraint,
                    self._decode_controller_dispatch(scope.motion, handler_spec, controller, constraint),
                )
            )
        return tuple(dispatches)

    def solver_interfaces(
        self,
        handler_spec: ConstraintHandler,
        *,
        solver_name: str | None = None,
    ) -> tuple[SolverInterface, ...]:
        motion = getattr(handler_spec, "motion", None)
        motion_id = _entity_id(motion)
        scope = self.motion_scope.get(motion_id)
        if scope is None:
            return ()

        interfaces: dict[tuple[type[SolverInterface], str], SolverInterface] = {}
        for _, _, dispatch in self.controller_dispatches(handler_spec):
            if solver_name is not None and dispatch.solver_name != solver_name:
                continue
            for interface in dispatch.interfaces:
                interfaces[(type(interface), interface.node_id)] = interface

        for solver in self.handler_solvers(handler_spec):
            if solver_name is not None and solver.name != solver_name:
                continue
            for velocity_solver in getattr(solver, "velocity_solvers", []):
                interface = VelocitySolverInterface(
                    node_id=velocity_solver.name,
                    signal_id=_node_name(velocity_solver.velocity),
                    owner=handler_spec,
                    solver_name=solver.name,
                    quantity_name=_node_name(velocity_solver.velocity),
                    configuration=velocity_solver.configuration,
                )
                key = (VelocitySolverInterface, velocity_solver.name)
                if key not in interfaces or self._solver_interface_priority(interface) < self._solver_interface_priority(interfaces[key]):
                    interfaces[key] = interface
            for force_solver in getattr(solver, "force_solvers", []):
                interface = ForceSolverInterface(
                    node_id=force_solver.name,
                    signal_id=_node_name(force_solver.force),
                    owner=handler_spec,
                    solver_name=solver.name,
                    quantity_name=_node_name(force_solver.force),
                    configuration=force_solver.configuration,
                )
                key = (ForceSolverInterface, force_solver.name)
                if key not in interfaces or self._solver_interface_priority(interface) < self._solver_interface_priority(interfaces[key]):
                    interfaces[key] = interface
            for force_name in getattr(solver, "cartesian_force", []):
                interface = CartesianForceInterface(
                    node_id=f"spec-{force_name}",
                    signal_id=force_name,
                    owner=handler_spec,
                    solver_name=solver.name,
                    force_name=force_name,
                )
                key = (CartesianForceInterface, f"spec-{force_name}")
                if key not in interfaces or self._solver_interface_priority(interface) < self._solver_interface_priority(interfaces[key]):
                    interfaces[key] = interface
            for joint_force in getattr(solver, "joint_force", []):
                interface = JointForceInterface(
                    node_id=joint_force,
                    signal_id=joint_force,
                    owner=handler_spec,
                    solver_name=solver.name,
                )
                key = (JointForceInterface, joint_force)
                if key not in interfaces or self._solver_interface_priority(interface) < self._solver_interface_priority(interfaces[key]):
                    interfaces[key] = interface

        return tuple(
            interface for _, interface in sorted(interfaces.items(), key=lambda item: (item[0][0].__name__, item[0][1]))
        )

    def interface_scalar_view_requirements(
        self, handler_spec: ConstraintHandler
    ) -> tuple[ScalarViewRequirement, ...]:
        requirements: dict[tuple[str, ViewProperty, str], ScalarViewRequirement] = {}
        for interface in self.solver_interfaces(handler_spec):
            if not isinstance(interface, CartesianForceInterface) or interface.axis is None:
                continue
            requirements.setdefault(
                (interface.force_name, ViewProperty.FORCE, interface.axis),
                ScalarViewRequirement(
                    quantity_name=interface.force_name,
                    property_name=ViewProperty.FORCE,
                    axis=interface.axis,
                    scalar_type=QuantityType.Force,
                ),
            )
        return tuple(requirements.values())

    def handler_solvers(self, handler_spec: ConstraintHandler) -> tuple[Any, ...]:
        return tuple(_resolved_solver(solver) for solver in getattr(handler_spec, "solvers", ()))

    def controller_solver(
        self,
        handler_spec: ConstraintHandler,
        controller: Any,
    ) -> Any | None:
        explicit_solver = getattr(getattr(controller, "solver", None), "solver", None)
        if explicit_solver is not None:
            return explicit_solver
        solvers = self.handler_solvers(handler_spec)
        if len(solvers) == 1:
            return solvers[0]
        return None

    def resolve_world_quantity(
        self,
        ref: Any,
        *,
        motion: MotionSpec | None = None,
        handler: ConstraintHandler | None = None,
        reason: str = "",
    ) -> WorldQuantityLike:
        if isinstance(ref, WorldQuantity):
            return ref

        uri = _entity_id(ref) if hasattr(ref, "uri") else None
        if uri:
            quantity = self.world_quantities.get(uri)
            if quantity is not None:
                return quantity

        name = _node_name(ref)
        scope = self.scope_for_motion(motion)
        if scope is not None and name in scope.quantities:
            return scope.quantities[name]

        if handler is not None:
            for item in getattr(handler, "context", []):
                for quantity in getattr(item, "declaration", []):
                    if isinstance(quantity, WorldQuantity) and quantity.name == name:
                        return quantity

        matches = self.world_quantities_by_name.get(name, ())
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            raise ValueError(f"Ambiguous world quantity reference '{name}' in context: {reason}")
        raise ValueError(f"Missing required world quantity '{name}' in context: {reason}")

    def _emit_acceleration_energy(self, energy_id: str, *, owner: Any) -> None:
        node = self.root_uri(energy_id, owner=owner)
        _add_types(self.graph, node, QUDT_SCHEMA.Quantity, QUDT_QKIND.AccelerationEnergy)
        self.graph.add((node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.AccelerationEnergy))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M2-PER-SEC2"]))

    def _emit_acceleration_constraint(
        self,
        motion_spec: MotionSpec,
        interface: AccelerationConstraintInterface,
    ) -> None:
        node = self.root_uri(interface.node_id, owner=motion_spec)
        _add_types(self.graph, node, SLV.AccelerationConstraint, SLV.AxisAligned)
        self.graph.add((node, SLV.subspace, SLV[interface.subspace]))
        self.graph.add((node, SLV.axis, SLV[interface.axis]))
        self.graph.add(
            (
                node,
                SLV["acceleration-energy"],
                self.root_uri(interface.signal_id, owner=motion_spec),
            )
        )

    def _emit_cartesian_force_spec(
        self,
        handler_spec: ConstraintHandler,
        motion_spec: MotionSpec,
        interface: CartesianForceInterface,
    ) -> None:
        force_quantity = self.resolve_world_quantity(
            interface.force_name,
            motion=motion_spec,
            handler=handler_spec,
            reason=f"referenced by cartesian-force solver output '{interface.node_id}'",
        )
        spec_node = self.root_uri(interface.node_id, owner=handler_spec)
        _add_types(self.graph, spec_node, SLV.CartesianForceSpecification)
        self.graph.add((spec_node, SLV.force, self.node(force_quantity)))
        if interface.attached_to is not None:
            self.graph.add((spec_node, SLV["attached-to"], URIRef(interface.attached_to)))

    def _emit_authored_solver_interface(
        self,
        handler_spec: ConstraintHandler,
        interface: SolverInterface,
    ) -> None:
        emitter = self._solver_interface_emitters().get(type(interface))
        if emitter is not None:
            emitter(handler_spec, interface)

    def _emit_velocity_solver_interface(
        self, handler_spec: ConstraintHandler, interface: VelocitySolverInterface
    ) -> None:
        _required_world_quantity(
            self,
            interface.quantity_name,
            f"referenced by base velocity solver '{interface.node_id}'",
            handler=handler_spec,
        )
        node = self.root_uri(interface.node_id, owner=handler_spec)
        _add_types(self.graph, node, SLV.VelocityCompositionSolver)
        self.graph.add((node, SLV.configuration, Literal(interface.configuration)))
        velocity_quantity = self.resolve_world_quantity(
            interface.quantity_name,
            handler=handler_spec,
            reason=f"referenced by base velocity solver '{interface.node_id}'",
        )
        self.graph.add((node, SLV.velocity, self.node(velocity_quantity)))

    def _emit_force_solver_interface(
        self, handler_spec: ConstraintHandler, interface: ForceSolverInterface
    ) -> None:
        _required_world_quantity(
            self,
            interface.quantity_name,
            f"referenced by base force solver '{interface.node_id}'",
            handler=handler_spec,
        )
        node = self.root_uri(interface.node_id, owner=handler_spec)
        _add_types(self.graph, node, SLV.ForceDistributionSolver)
        self.graph.add((node, SLV.configuration, Literal(interface.configuration)))
        force_quantity = self.resolve_world_quantity(
            interface.quantity_name,
            handler=handler_spec,
            reason=f"referenced by base force solver '{interface.node_id}'",
        )
        self.graph.add((node, SLV.force, self.node(force_quantity)))

    def _emit_joint_force_interface(
        self, handler_spec: ConstraintHandler, interface: JointForceInterface
    ) -> None:
        node = self.root_uri(interface.node_id, owner=handler_spec)
        _add_types(self.graph, node, SLV.JointForce)

    def _solver_interface_emitters(self) -> dict[type[SolverInterface], Any]:
        return {
            VelocitySolverInterface: self._emit_velocity_solver_interface,
            ForceSolverInterface: self._emit_force_solver_interface,
            JointForceInterface: self._emit_joint_force_interface,
        }

    def _solver_node_stem(
        self,
        handler_spec: ConstraintHandler,
        motion_spec: MotionSpec,
        solver: Any,
    ) -> str:
        if len(self.handler_solvers(handler_spec)) == 1:
            return motion_spec.name or handler_spec.name
        return solver.name

    def _spec_acc_node(
        self,
        motion_spec: MotionSpec,
        solver: Any,
        *,
        handler_spec: ConstraintHandler,
    ) -> URIRef:
        stem = self._solver_node_stem(handler_spec, motion_spec, solver)
        return self.root_uri(f"spec-acc-{stem}", owner=motion_spec)

    def _driver_attachment(
        self,
        interface: SolverInterface,
        *,
        handler_spec: ConstraintHandler,
    ) -> tuple[Node, Node] | None:
        predicate = self._driver_attachment_predicates().get(type(interface))
        if predicate is None:
            return None
        return (predicate, self.root_uri(interface.node_id, owner=handler_spec))

    def _driver_attachment_predicates(self) -> dict[type[SolverInterface], Node]:
        return {
            CartesianForceInterface: SLV["cartesian-force"],
            JointForceInterface: SLV["joint-force"],
        }

    def constraint_quantity_node(self, constraint: ConstraintData) -> Node:
        owning_motion = getattr(getattr(constraint.constraint, "parent", None), "parent", None)
        owning_scope = self.scope_for_motion(owning_motion)
        scope = owning_scope or self.motion_scope.get(constraint.motion_id)
        return self.node(constraint.quantity_node_id, owner=scope.motion if scope else None)

    @cached_property
    def authored_handlers(self) -> list[ConstraintHandler]:
        return [
            spec
            for model in self.models
            for spec in model.specs
            if isinstance(spec, ConstraintHandler)
        ]

    @cached_property
    def motions(self) -> dict[str, MotionSpec]:
        motions: dict[str, MotionSpec] = {}
        for handler in self.authored_handlers:
            motion = getattr(handler, "motion", None)
            motion_id = _entity_id(motion)
            if not isinstance(motion, MotionSpec) or motion_id in motions:
                continue
            motions[motion_id] = motion
        return motions

    @cached_property
    def motion_scope(self) -> dict[str, MotionScope]:
        scopes: dict[str, MotionScope] = {}
        for motion_id, motion in self.motions.items():
            quantities: dict[str, WorldQuantity] = {}
            values: dict[str, ValueVariable] = {}
            for context in motion.context:
                if isinstance(context, WorldContextDecl):
                    for quantity in context.declaration:
                        if not isinstance(quantity, WorldQuantity):
                            continue
                        quantities[quantity.name] = quantity
                elif isinstance(context, (PreContextDecl, SpecContextDecl, PostContextDecl)):
                    for value in context.declaration:
                        if not isinstance(value, ValueVariable):
                            continue
                        values[value.name] = value
            scopes[motion_id] = MotionScope(
                motion_id=motion_id,
                motion=motion,
                quantities=quantities,
                values=values,
                constraints=tuple(self.all_constraints(motion)),
            )
        return scopes

    @cached_property
    def world_quantities(self) -> dict[str, WorldQuantityLike]:
        quantities: dict[str, WorldQuantityLike] = {}
        for scope in self.motion_scope.values():
            for quantity in scope.quantities.values():
                quantities[str(quantity.uri)] = quantity
        for handler in self.authored_handlers:
            for item in getattr(handler, "context", []):
                for quantity in getattr(item, "declaration", []):
                    if isinstance(quantity, WorldQuantity):
                        quantities[str(quantity.uri)] = quantity
        return quantities

    @cached_property
    def world_quantities_by_name(self) -> dict[str, tuple[WorldQuantityLike, ...]]:
        grouped: dict[str, list[WorldQuantityLike]] = {}
        for quantity in self.world_quantities.values():
            grouped.setdefault(quantity.name, []).append(quantity)
        return {name: tuple(values) for name, values in grouped.items()}

    @cached_property
    def value_variables(self) -> dict[str, ValueVariable]:
        values: dict[str, ValueVariable] = {}
        for scope in self.motion_scope.values():
            for val in scope.values.values():
                values[str(val.uri)] = val
        for scope in self.motion_scope.values():
            for constraint in scope.constraints:
                for ref in _constraint_context_refs(constraint):
                    value = getattr(ref, "value", None) or getattr(ref, "valRef", None)
                    if isinstance(value, ValueVariable):
                        values[str(value.uri)] = value
        for handler in self.authored_handlers:
            for solver in handler.solvers:
                gravity_value = getattr(solver, "gravity_value", None)
                value = getattr(gravity_value, "value", None) or getattr(gravity_value, "valRef", None)
                if isinstance(value, ValueVariable):
                    values[str(value.uri)] = value
        return values

    @cached_property
    def value_variables_by_name(self) -> dict[str, tuple[ValueVariable, ...]]:
        grouped: dict[str, list[ValueVariable]] = {}
        for value in self.value_variables.values():
            grouped.setdefault(value.name, []).append(value)
        return {name: tuple(values) for name, values in grouped.items()}

    def resolve_value_variable(
        self,
        ref: Any,
        *,
        motion: MotionSpec | None = None,
        reason: str = "",
    ) -> ValueVariable | None:
        if isinstance(ref, ValueVariable):
            return ref

        uri = _entity_id(ref) if hasattr(ref, "uri") else None
        if uri:
            value = self.value_variables.get(uri)
            if value is not None:
                return value

        name = _node_name(ref)
        scope = self.scope_for_motion(motion)
        if scope is not None and name in scope.values:
            return scope.values[name]

        matches = self.value_variables_by_name.get(name, ())
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            raise ValueError(f"Ambiguous value reference '{name}' in context: {reason}")
        return None

    @cached_property
    def implicit_world_entities(self) -> dict[str, Node]:
        entities: dict[str, Node] = {}
        for quantity in self.world_quantities.values():
            if not isinstance(quantity.props, GeometricProps):
                continue
            if quantity.type == WorldQuantityType.Pose:
                for key in ("of", "wrt", "as-seen-by"):
                    target = _geometric_property(quantity.props, key)
                    if target:
                        entities.setdefault(target, GEOM_ENT.Frame)
            elif quantity.type == WorldQuantityType.VelocityTwist:
                for key in ("of", "wrt"):
                    target = _geometric_property(quantity.props, key)
                    if target:
                        entities.setdefault(target, GEOM_ENT.SimplicialComplex)
                point_id = _geometric_property(quantity.props, "ref-point")
                if point_id:
                    entities.setdefault(point_id, GEOM_ENT.Point)
                frame_id = _geometric_property(quantity.props, "as-seen-by")
                if frame_id:
                    entities.setdefault(frame_id, GEOM_ENT.Frame)
            elif quantity.type == WorldQuantityType.Wrench:
                point_id = _geometric_property(quantity.props, "ref-point")
                if point_id:
                    entities.setdefault(point_id, GEOM_ENT.Point)
                frame_id = _geometric_property(quantity.props, "as-seen-by")
                if frame_id:
                    entities.setdefault(frame_id, GEOM_ENT.Frame)
            elif quantity.type == WorldQuantityType.JointPosition:
                target = _geometric_property(quantity.props, "of")
                if target:
                    entities.setdefault(target, KC.Joint)
        return entities

    @cached_property
    def controlled_constraint_names(self) -> dict[str, frozenset[str]]:
        controlled: dict[str, set[str]] = {motion_id: set() for motion_id in self.motion_scope}
        for handler in self.authored_handlers:
            motion_id = _entity_id(getattr(handler, "motion", None))
            if motion_id not in controlled:
                continue
            for controller in getattr(handler, "controllers", []):
                controlled[motion_id].add(controller.params.constraint_name)
        return {motion_id: frozenset(names) for motion_id, names in controlled.items()}

    @cached_property
    def controlled_constraint_specs(self) -> dict[str, frozenset[int]]:
        controlled: dict[str, set[int]] = {motion_id: set() for motion_id in self.motion_scope}
        for handler in self.authored_handlers:
            motion = getattr(handler, "motion", None)
            motion_id = _entity_id(motion)
            if motion_id not in controlled:
                continue
            for controller in getattr(handler, "controllers", []):
                target = getattr(controller.params.constraint, "constraint", None)
                if target is not None:
                    controlled[motion_id].add(id(target))
        return {motion_id: frozenset(specs) for motion_id, specs in controlled.items()}

    @cached_property
    def monitored_constraint_names(self) -> dict[str, frozenset[str]]:
        monitored: dict[str, set[str]] = {motion_id: set() for motion_id in self.motion_scope}
        for handler in self.authored_handlers:
            motion_id = _entity_id(getattr(handler, "motion", None))
            if motion_id not in monitored:
                continue
            for monitor in getattr(handler, "monitors", []):
                monitored[motion_id].add(monitor.constraint_name)
        return {motion_id: frozenset(names) for motion_id, names in monitored.items()}

    @cached_property
    def monitored_constraint_specs(self) -> dict[str, frozenset[int]]:
        monitored: dict[str, set[int]] = {motion_id: set() for motion_id in self.motion_scope}
        for handler in self.authored_handlers:
            motion_id = _entity_id(getattr(handler, "motion", None))
            if motion_id not in monitored:
                continue
            for monitor in getattr(handler, "monitors", []):
                target = getattr(monitor.constraint, "constraint", None)
                if target is not None:
                    monitored[motion_id].add(id(target))
        return {motion_id: frozenset(specs) for motion_id, specs in monitored.items()}

    @cached_property
    def shared_constraint_specs(self) -> frozenset[int]:
        usage: dict[int, set[str]] = {}
        for scope in self.motion_scope.values():
            seen_specs: set[int] = set()
            for section in (scope.motion.when, scope.motion.while_, scope.motion.until):
                for item in section.constraints:
                    spec_id = id(_resolved_spec(item))
                    if spec_id in seen_specs:
                        continue
                    seen_specs.add(spec_id)
                    usage.setdefault(spec_id, set()).add(scope.motion_id)
        return frozenset(
            spec_id for spec_id, motions in usage.items() if len(motions) > 1
        )

    @cached_property
    def constraints(self) -> list[ConstraintData]:
        derived_constraints: list[ConstraintData] = []
        for motion_id, scope in self.motion_scope.items():
            controlled_names = self.controlled_constraint_names.get(motion_id, frozenset())
            controlled_specs = self.controlled_constraint_specs.get(motion_id, frozenset())
            monitored_names = self.monitored_constraint_names.get(motion_id, frozenset())
            monitored_specs = self.monitored_constraint_specs.get(motion_id, frozenset())
            for constraint in scope.constraints:
                resolved = _resolve_constraint_view(self, scope, constraint)
                quantity = resolved.quantity
                if quantity is None:
                    raise ValueError(
                        f"Constraint '{constraint.name}' references world quantity "
                        f"'{resolved.quantity_name}' that is not defined in the motion context."
                    )
                quantity_node_id = resolved.scalar_id or resolved.quantity_name

                expr = constraint.expr
                error_signal_id = None
                shared = id(constraint) in self.shared_constraint_specs

                if isinstance(expr, EqualityConstraint):
                    kind = ConstraintKind.EQUALITY
                    reference_var = _node_name(
                        getattr(expr.reference, "value", None)
                        or getattr(expr.reference, "valRef", None)
                    )
                    threshold_var = lower_var = upper_var = None
                    if (
                        constraint.name in controlled_names
                        or id(constraint) in controlled_specs
                    ) and quantity is not None:
                        error_signal_id = _controller_error_signal_id(
                            scope.motion.name,
                            quantity,
                            resolved.property_name,
                            resolved.axis,
                            shared,
                        )
                elif isinstance(expr, GreaterThanConstraint):
                    kind = ConstraintKind.GREATER_THAN
                    reference_var = None
                    threshold_var = _node_name(
                        getattr(expr.threshold, "value", None)
                        or getattr(expr.threshold, "valRef", None)
                    )
                    lower_var = upper_var = None
                elif isinstance(expr, LessThanConstraint):
                    kind = ConstraintKind.LESS_THAN
                    reference_var = None
                    threshold_var = _node_name(
                        getattr(expr.threshold, "value", None)
                        or getattr(expr.threshold, "valRef", None)
                    )
                    lower_var = upper_var = None
                else:
                    assert isinstance(expr, BilateralConstraint)
                    kind = ConstraintKind.BILATERAL
                    reference_var = threshold_var = None
                    lower_var = _node_name(
                        getattr(expr.lower, "value", None) or getattr(expr.lower, "valRef", None)
                    )
                    upper_var = _node_name(
                        getattr(expr.upper, "value", None) or getattr(expr.upper, "valRef", None)
                    )

                if (
                    error_signal_id is None
                    and (
                        constraint.name in controlled_names
                        or id(constraint) in controlled_specs
                    )
                    and quantity is not None
                    and quantity.type == WorldQuantityType.JointPosition
                ):
                    error_signal_id = _controller_error_signal_id(
                        scope.motion.name,
                        quantity,
                        resolved.property_name,
                        resolved.axis,
                        shared,
                    )

                if (
                    constraint.name in monitored_names
                    or id(constraint) in monitored_specs
                ) and quantity is not None and error_signal_id is None:
                    error_signal_id = f"{quantity_node_id}-err"

                derived_constraints.append(
                    ConstraintData(
                        motion_id=motion_id,
                        motion_name=scope.motion.name,
                        constraint=constraint,
                        quantity=quantity,
                        kind=kind,
                        scalar_type=resolved.scalar_type or resolved.property_name,
                        quantity_node_id=quantity_node_id,
                        property_name=resolved.property_name,
                        axis=resolved.axis,
                        reference_var=reference_var,
                        threshold_var=threshold_var,
                        lower_var=lower_var,
                        upper_var=upper_var,
                        error_signal_id=error_signal_id,
                        shared=shared,
                    )
                )
        return derived_constraints

    @cached_property
    def constraints_by_motion(self) -> dict[str, tuple[ConstraintData, ...]]:
        grouped: dict[str, list[ConstraintData]] = {}
        for constraint in self.constraints:
            grouped.setdefault(constraint.motion_id, []).append(constraint)
        return {motion_id: tuple(values) for motion_id, values in grouped.items()}

    @cached_property
    def controlled_constraints(self) -> list[ConstraintData]:
        return [
            constraint
            for constraint in self.constraints
            if (
                constraint.constraint.name
                in self.controlled_constraint_names.get(constraint.motion_id, frozenset())
                or id(constraint.constraint)
                in self.controlled_constraint_specs.get(constraint.motion_id, frozenset())
            )
        ]

    @cached_property
    def monitored_constraints(self) -> list[ConstraintData]:
        return [
            constraint
            for constraint in self.constraints
            if (
                constraint.constraint.name
                in self.monitored_constraint_names.get(constraint.motion_id, frozenset())
                or id(constraint.constraint)
                in self.monitored_constraint_specs.get(constraint.motion_id, frozenset())
            )
        ]

    def build(self) -> DatasetOutput:
        for owner in self.namespace_owners:
            self.dataset.bind(owner.ns_prefix, owner.ns.uri)
        self.materialize_authored()
        self.materialize_derived()
        return self.dataset, self.context(*GRAPH_BINDINGS)

    def materialize_authored(self) -> None:
        self._add_structural_entities()
        self._add_world_quantities()
        self._add_value_variables()
        self._add_constraints()
        self._add_motion_specs()
        self._add_constraint_handlers()

    def materialize_derived(self) -> None:
        self._add_scalar_views()
        self._add_error_signals()
        self._add_solver_entities()
        self._add_map_operations()
        self._add_transform_operations()

    def _add_structural_entities(self) -> None:
        for quantity in self.world_quantities.values():
            if WORLD_SPECS.get(quantity.type) is None:
                rdf_type = WORLD_STRUCTURE_TYPES.get(quantity.type)
                if rdf_type is not None:
                    _add_types(self.graph, URIRef(quantity.uri), rdf_type)
        for entity_name, rdf_type in self.implicit_world_entities.items():
            _add_types(self.graph, self.root_uri(entity_name), rdf_type)

    def _add_world_quantities(self) -> None:
        for quantity in self.world_quantities.values():
            world_spec = WORLD_SPECS.get(quantity.type)
            if world_spec is None:
                continue
            node = URIRef(quantity.uri)
            _add_types(self.graph, node, *world_spec.rdf_types)
            for qkind in world_spec.qkinds:
                self.graph.add((node, QUDT_SCHEMA["quantity-kind"], qkind))
            for unit in world_spec.units:
                self.graph.add((node, QUDT_SCHEMA.unit, unit))
            if not isinstance(quantity.props, GeometricProps):
                continue
            of_value = _geometric_property(quantity.props, "of")
            wrt_value = _geometric_property(quantity.props, "wrt")
            reference_point = _geometric_property(quantity.props, "ref-point")
            as_seen_by = _geometric_property(quantity.props, "as-seen-by")
            if of_value:
                self.graph.add((node, GEOM_REL.of, self.root_uri(of_value, owner=quantity)))
            if wrt_value:
                self.graph.add(
                    (node, GEOM_REL["with-respect-to"], self.root_uri(wrt_value, owner=quantity))
                )
            if reference_point:
                self.graph.add(
                    (
                        node,
                        GEOM_REL["reference-point"],
                        self.root_uri(reference_point, owner=quantity),
                    )
                )
            if as_seen_by:
                self.graph.add(
                    (node, GEOM_COORD["as-seen-by"], self.root_uri(as_seen_by, owner=quantity))
                )
            elif quantity.type == WorldQuantityType.Pose and wrt_value:
                self.graph.add(
                    (node, GEOM_COORD["as-seen-by"], self.root_uri(wrt_value, owner=quantity))
                )
            if (
                quantity.type in {WorldQuantityType.VelocityTwist, WorldQuantityType.Wrench}
                and not reference_point
            ):
                warnings.warn(
                    f"'{quantity.name}' has no explicit ref-point; not inferring one from its name."
                )

    def _add_value_variables(self) -> None:
        for variable in self.value_variables.values():
            node = URIRef(variable.uri)
            qkind = (
                QUDT_QKIND.Vector
                if variable.type == QuantityType.Vector
                else QUDT_QKIND[variable.type]
            )
            _add_types(self.graph, node, QUDT_SCHEMA.Quantity, qkind)
            self.graph.add((node, QUDT_SCHEMA["quantity-kind"], qkind))
            if variable.value is None:
                continue
            self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(variable.value.unit)))
            if isinstance(variable.value, ScalarQuantity):
                self.graph.add((node, QUDT_SCHEMA.value, Literal(str(variable.value.value))))

    def _add_constraints(self) -> None:
        seen_uris: set[str] = set()
        for constraint in self.constraints:
            uri_str = str(constraint.constraint.uri)
            if uri_str in seen_uris:
                continue
            seen_uris.add(uri_str)
            node = URIRef(constraint.constraint.uri)
            type_name = CSTR_TYPE_NAME.get(constraint.scalar_type, constraint.scalar_type)
            _add_types(self.graph, node, CSTR.Constraint, CSTR[f"{type_name}Constraint"])
            self.graph.add((node, CSTR.quantity, self.constraint_quantity_node(constraint)))
            owning_motion = getattr(getattr(constraint.constraint, "parent", None), "parent", None)
            owning_scope = self.scope_for_motion(owning_motion)
            scope = owning_scope or self.motion_scope[constraint.motion_id]
            motion_spec = scope.motion
            motion_values = scope.values
            if constraint.kind == ConstraintKind.EQUALITY:
                reference = motion_values.get(constraint.reference_var or "") or self.resolve_value_variable(
                    constraint.reference_var,
                    motion=motion_spec,
                    reason=f"constraint '{constraint.constraint.name}' equality reference",
                )
                _add_types(self.graph, node, CSTR.EqualityConstraint)
                ref_node = (
                    URIRef(reference.uri)
                    if reference
                    else self.root_uri(constraint.reference_var or "", owner=motion_spec)
                )
                self.graph.add((node, CSTR["reference-value"], ref_node))
            elif constraint.kind == ConstraintKind.GREATER_THAN:
                threshold = motion_values.get(constraint.threshold_var or "") or self.resolve_value_variable(
                    constraint.threshold_var,
                    motion=motion_spec,
                    reason=f"constraint '{constraint.constraint.name}' threshold",
                )
                _add_types(self.graph, node, CSTR.UnilateralConstraint, CSTR.GreaterThanConstraint)
                threshold_node = (
                    URIRef(threshold.uri)
                    if threshold
                    else self.root_uri(constraint.threshold_var or "", owner=motion_spec)
                )
                self.graph.add((node, CSTR.threshold, threshold_node))
            elif constraint.kind == ConstraintKind.LESS_THAN:
                threshold = motion_values.get(constraint.threshold_var or "") or self.resolve_value_variable(
                    constraint.threshold_var,
                    motion=motion_spec,
                    reason=f"constraint '{constraint.constraint.name}' threshold",
                )
                _add_types(self.graph, node, CSTR.UnilateralConstraint, CSTR.LessThanConstraint)
                threshold_node = (
                    URIRef(threshold.uri)
                    if threshold
                    else self.root_uri(constraint.threshold_var or "", owner=motion_spec)
                )
                self.graph.add((node, CSTR.threshold, threshold_node))
            else:
                lower = motion_values.get(constraint.lower_var or "") or self.resolve_value_variable(
                    constraint.lower_var,
                    motion=motion_spec,
                    reason=f"constraint '{constraint.constraint.name}' lower threshold",
                )
                upper = motion_values.get(constraint.upper_var or "") or self.resolve_value_variable(
                    constraint.upper_var,
                    motion=motion_spec,
                    reason=f"constraint '{constraint.constraint.name}' upper threshold",
                )
                _add_types(self.graph, node, CSTR.BilateralConstraint)
                lower_node = (
                    URIRef(lower.uri)
                    if lower
                    else self.root_uri(constraint.lower_var or "", owner=motion_spec)
                )
                upper_node = (
                    URIRef(upper.uri)
                    if upper
                    else self.root_uri(constraint.upper_var or "", owner=motion_spec)
                )
                self.graph.add((node, CSTR["lower-threshold"], lower_node))
                self.graph.add((node, CSTR["upper-threshold"], upper_node))

    def _add_motion_specs(self) -> None:
        for scope in self.motion_scope.values():
            motion_spec = scope.motion
            motion_node = self.root_uri(f"motion-{motion_spec.name}", owner=motion_spec)
            _add_types(self.graph, motion_node, MOT.GuardedMotion)
            for constraint in motion_spec.when.constraints:
                self.graph.add((motion_node, MOT.when, URIRef(_resolved_spec(constraint).uri)))
            for constraint in motion_spec.while_.constraints:
                self.graph.add((motion_node, MOT["while"], URIRef(_resolved_spec(constraint).uri)))
            for constraint in motion_spec.until.constraints:
                self.graph.add((motion_node, MOT.until, URIRef(_resolved_spec(constraint).uri)))
            raw_logic = getattr(motion_spec.until, "logic", None)
            until_logic = raw_logic if raw_logic in ("any", "all") else "any"
            self.graph.add((motion_node, MOT.untilLogic, Literal(until_logic)))

    def _add_constraint_handlers(self) -> None:
        for handler_spec in self.authored_handlers:
            motion = getattr(handler_spec, "motion", None)
            motion_id = _entity_id(motion)
            if not motion_id:
                continue
            scope = self.motion_scope.get(motion_id)
            if scope is None:
                continue
            node = URIRef(handler_spec.uri)
            _add_types(self.graph, node, CSTR_HDL.ConstraintHandler)
            self.graph.add(
                (
                    node,
                    CSTR_HDL.motion,
                    self.root_uri(f"motion-{scope.motion.name}", owner=scope.motion),
                )
            )

            for constraint in self.controlled_constraints:
                if constraint.motion_id != motion_id:
                    continue
                self.graph.add(
                    (
                        node,
                        CSTR_HDL.evaluators,
                        self.root_uri(
                            _evaluator_id(constraint.constraint), owner=constraint.constraint.parent
                        ),
                    )
                )

            for controller, constraint, dispatch in self.controller_dispatches(handler_spec):
                controller_node = URIRef(controller.uri)
                _add_types(
                    self.graph,
                    controller_node,
                    CSTR_HDL.Controller,
                    CSTR_HDL.ProportionalIntegralDerivative,
                )
                if constraint.error_signal_id:
                    self.graph.add(
                        (
                            controller_node,
                            CSTR_HDL["error-signal"],
                            self.root_uri(
                                constraint.error_signal_id, owner=constraint.constraint.parent
                            ),
                        )
                    )
                self.graph.add(
                    (
                        controller_node,
                        CSTR_HDL["control-signal"],
                        self.root_uri(dispatch.signal.node_id, owner=dispatch.signal.owner),
                    )
                )
                self.graph.add(
                    (
                        controller_node,
                        CSTR_HDL["proportional-gain"],
                        Literal(str(controller.params.kp)),
                    )
                )
                self.graph.add(
                    (controller_node, CSTR_HDL["integral-gain"], Literal(str(controller.params.ki)))
                )
                self.graph.add(
                    (
                        controller_node,
                        CSTR_HDL["derivative-gain"],
                        Literal(str(controller.params.kd)),
                    )
                )
                self.graph.add((node, CSTR_HDL.controllers, controller_node))
                if dispatch.signal.kind == "AccelerationEnergy":
                    self._emit_acceleration_energy(dispatch.signal.node_id, owner=dispatch.signal.owner)
                elif dispatch.signal.scalar_type is not None:
                    _add_quantity(
                        self.graph,
                        self.root_uri(dispatch.signal.node_id, owner=dispatch.signal.owner),
                        dispatch.signal.scalar_type,
                    )

            for monitor in getattr(handler_spec, "monitors", []):
                constraint = self.constraint_data_for_motion(scope.motion, monitor.constraint)
                if constraint is None:
                    continue
                assert constraint.error_signal_id is not None
                signal_name = monitor.event or monitor.flag
                signal_kind = "event" if monitor.event else "flag"
                signal_node = self.root_uri(signal_name)
                _add_types(
                    self.graph,
                    signal_node,
                    CSTR_HDL.Event if signal_kind == "event" else CSTR_HDL.Flag,
                )
                monitor_node = URIRef(monitor.uri)
                _add_types(self.graph, monitor_node, CSTR_HDL.Monitor)
                self.graph.add(
                    (
                        monitor_node,
                        CSTR_HDL.error,
                        self.root_uri(constraint.error_signal_id, owner=constraint.constraint.parent),
                    )
                )
                if signal_kind == "event":
                    _add_types(self.graph, monitor_node, CSTR_HDL.EdgeTriggeredMonitor)
                    self.graph.add((monitor_node, CSTR_HDL.event, signal_node))
                else:
                    _add_types(self.graph, monitor_node, CSTR_HDL.LevelTriggeredMonitor)
                    self.graph.add((monitor_node, CSTR_HDL.flag, signal_node))
                self.graph.add((node, CSTR_HDL.monitors, monitor_node))
                self.graph.add(
                    (
                        node,
                        CSTR_HDL.evaluators,
                        self.root_uri(_evaluator_id(constraint.constraint), owner=constraint.constraint.parent),
                    )
                )

    def _add_scalar_views(self) -> None:
        existing_scalar_views: set[str] = set()
        seen_view_keys: set[ViewKey] = set()
        rotation_ids: set[str] = set()
        for scope in self.motion_scope.values():
            for constraint in scope.constraints:
                resolved = _resolve_constraint_view(self, scope, constraint)
                quantity = resolved.quantity
                if quantity is None:
                    continue
                if (
                    quantity.type == WorldQuantityType.Pose
                    and resolved.property_name == ViewProperty.ROTATION
                    and resolved.axis is None
                ):
                    rotation_ids.add(f"rotation-{scope.motion.name}")
                    continue
                if resolved.axis is None:
                    continue
                if (
                    resolved.property_spec is None
                    or resolved.property_spec.view_type is None
                    or resolved.scalar_id is None
                    or resolved.scalar_type is None
                ):
                    continue
                if (
                    quantity.type == WorldQuantityType.Pose
                    and resolved.property_name == ViewProperty.ROTATION
                ):
                    continue
                view_key: ViewKey = (resolved.quantity_name, resolved.property_name, resolved.axis)
                if view_key in seen_view_keys:
                    continue
                seen_view_keys.add(view_key)
                existing_scalar_views.add(resolved.scalar_id)
                _add_quantity(
                    self.graph,
                    self.root_uri(resolved.scalar_id, owner=scope.motion),
                    resolved.scalar_type,
                )
                node = self.root_uri(f"view-{resolved.scalar_id}", owner=scope.motion)
                _add_types(self.graph, node, MAP.View)
                _add_types(self.graph, node, resolved.property_spec.view_type)
                self.graph.add((node, MAP.superobject, self.node(quantity)))
                self.graph.add(
                    (node, MAP.subobject, self.root_uri(resolved.scalar_id, owner=scope.motion))
                )
                if resolved.property_spec.view_subspace:
                    self.graph.add((node, MAP.subspace, MAP[resolved.property_spec.view_subspace]))
                self.graph.add((node, MAP.axis, MAP[resolved.axis]))
        for constraint in self.constraints:
            if constraint.scalar_type in (QuantityType.Angle, QuantityType.PlaneAngle):
                _add_quantity(
                    self.graph, self.constraint_quantity_node(constraint), constraint.scalar_type
                )
        for rotation_id in rotation_ids:
            _add_quantity(self.graph, self.root_uri(rotation_id), QuantityType.Angle)
        for handler_spec in self.authored_handlers:
            for requirement in self.interface_scalar_view_requirements(handler_spec):
                quantity = self.resolve_world_quantity(
                    requirement.quantity_name,
                    motion=getattr(handler_spec, "motion", None),
                    handler=handler_spec,
                    reason=f"referenced by solver interface scalar view '{requirement.quantity_name}'",
                )
                if quantity is None or quantity.type != WorldQuantityType.Wrench:
                    continue
                scalar_id = _scalar_id(quantity, requirement.property_name, requirement.axis)
                if scalar_id is None or scalar_id in existing_scalar_views:
                    continue
                existing_scalar_views.add(scalar_id)
                _add_quantity(
                    self.graph, self.root_uri(scalar_id, owner=quantity), requirement.scalar_type
                )
                node = self.root_uri(f"view-{scalar_id}", owner=quantity)
                _add_types(self.graph, node, MAP.View, MAP.WrenchCoordinateView)
                self.graph.add((node, MAP.superobject, self.node(quantity)))
                self.graph.add((node, MAP.subobject, self.root_uri(scalar_id, owner=quantity)))
                self.graph.add((node, MAP.subspace, MAP[requirement.property_name]))
                self.graph.add((node, MAP.axis, MAP[requirement.axis]))

    def _add_error_signals(self) -> None:
        seen_error_ids: set[str] = set()
        seen_eval_constraints: set[str] = set()
        for constraint in [*self.controlled_constraints, *self.monitored_constraints]:
            if constraint.error_signal_id is not None and constraint.error_signal_id not in seen_error_ids:
                seen_error_ids.add(constraint.error_signal_id)
                _add_quantity(
                    self.graph,
                    self.root_uri(constraint.error_signal_id, owner=constraint.constraint.parent),
                    constraint.scalar_type,
                )
            constraint_id = _entity_id(constraint.constraint)
            if constraint.error_signal_id is None or constraint_id in seen_eval_constraints:
                continue
            seen_eval_constraints.add(constraint_id)
            spec = constraint.constraint
            node = self.root_uri(_evaluator_id(spec), owner=spec.parent)
            _add_types(self.graph, node, CSTR_HDL.ConstraintEvaluator, CSTR_HDL.ErrorEvaluator)
            self.graph.add((node, CSTR_HDL.constraint, URIRef(constraint.constraint.uri)))
            self.graph.add(
                (
                    node,
                    CSTR_HDL.error,
                    self.root_uri(constraint.error_signal_id, owner=constraint.constraint.parent),
                )
            )

    def _add_solver_entities(self) -> None:
        self._add_handler_solvers()

    def _add_handler_solvers(self) -> None:
        for handler_spec in self.authored_handlers:
            motion = getattr(handler_spec, "motion", None)
            motion_id = _entity_id(motion)
            if not motion_id:
                continue
            scope = self.motion_scope.get(motion_id)
            if scope is None:
                continue
            motion_spec = scope.motion
            for solver in self.handler_solvers(handler_spec):
                if not getattr(solver, "algorithm", ""):
                    continue
                interfaces = self.solver_interfaces(handler_spec, solver_name=solver.name)
                for interface in interfaces:
                    self._emit_authored_solver_interface(handler_spec, interface)
                acceleration_interfaces = [
                    interface
                    for interface in interfaces
                    if isinstance(interface, AccelerationConstraintInterface)
                ]
                cartesian_force_interfaces = [
                    interface for interface in interfaces if isinstance(interface, CartesianForceInterface)
                ]

                spec_acc_node = None
                if acceleration_interfaces:
                    spec_acc_node = self._spec_acc_node(
                        motion_spec,
                        solver,
                        handler_spec=handler_spec,
                    )
                    _add_types(self.graph, spec_acc_node, SLV.AccelerationConstraintSpecification)
                    for interface in acceleration_interfaces:
                        self._emit_acceleration_constraint(motion_spec, interface)
                        self.graph.add(
                            (
                                spec_acc_node,
                                SLV.constraints,
                                self.root_uri(interface.node_id, owner=motion_spec),
                            )
                        )

                for interface in cartesian_force_interfaces:
                    self._emit_cartesian_force_spec(handler_spec, motion_spec, interface)

                driver_name = self._solver_node_stem(handler_spec, motion_spec, solver)
                driver_node = self.root_uri(f"drv-{driver_name}", owner=handler_spec)
                _add_types(self.graph, driver_node, SLV.MotionDrivers)
                if spec_acc_node is not None:
                    self.graph.add((driver_node, SLV["acceleration-constraint"], spec_acc_node))
                for interface in interfaces:
                    attachment = self._driver_attachment(interface, handler_spec=handler_spec)
                    if attachment is not None:
                        self.graph.add((driver_node, *attachment))

                solver_node = self.root_uri(f"slv-{driver_name}", owner=handler_spec)
                solver_algorithm = (
                    SLV.AccelerationConstrainedHybridDynamicsAlgorithm
                    if solver.algorithm == SolverAlgorithm.ACHD
                    else SLV.NewtonEulerAlgorithm
                    if solver.algorithm == SolverAlgorithm.RNE
                    else SLV[solver.algorithm]
                )
                _add_types(self.graph, solver_node, SLV.SolverWithInputAndOutput)
                self.graph.add((solver_node, SLV.solver, solver_algorithm))
                gravity_name = _node_name(solver.gravity)
                gravity_quantity = self.resolve_world_quantity(
                    gravity_name,
                    handler=handler_spec,
                    reason=f"referenced by solver '{solver.name}' gravity",
                )
                self.graph.add(
                    (
                        solver_node,
                        SLV["kinematic-chain"],
                        self.root_uri(_node_name(solver.robot), owner=handler_spec),
                    )
                )
                root_node = self.root_uri(_node_name(solver.root), owner=handler_spec)
                _add_types(self.graph, root_node, GEOM_ENT.Frame)
                self.graph.add((solver_node, SLV.root, root_node))
                self.graph.add((solver_node, SLV.gravity, self.node(gravity_quantity)))
                self.graph.add((solver_node, SLV["motion-drivers"], driver_node))

    def _add_map_operations(self) -> None:
        seen_angle_ops: set[str] = set()
        for scope in self.motion_scope.values():
            rotation_pose = next(
                (
                    _node_name(constraint.view.quantity)
                    for constraint in scope.constraints
                    if _view_property_name(constraint) == ViewProperty.ROTATION
                    and constraint.view.axis is None
                ),
                None,
            )
            if rotation_pose is not None:
                rotation_id = f"rotation-{scope.motion.name}"
                op_node = self.root_uri(f"compute-{rotation_id}", owner=scope.motion)
                _add_types(self.graph, op_node, MAP.ComputeRotationFromPose)
                self.graph.add(
                    (op_node, MAP.pose, self.root_uri(rotation_pose, owner=scope.motion))
                )
                self.graph.add(
                    (op_node, MAP.rotation, self.root_uri(rotation_id, owner=scope.motion))
                )

            for constraint in scope.constraints:
                property_name = _view_property_name(constraint)
                axis_value = constraint.view.axis
                axis = None if axis_value is None else str(getattr(axis_value, "value", axis_value))
                quantity = scope.quantities.get(_node_name(constraint.view.quantity))
                if (
                    quantity is None
                    or quantity.type != WorldQuantityType.Pose
                    or property_name != ViewProperty.ROTATION
                    or axis is None
                ):
                    continue
                scalar_id = _scalar_id(quantity, ViewProperty.ROTATION, axis)
                if scalar_id is None or scalar_id in seen_angle_ops:
                    continue
                seen_angle_ops.add(scalar_id)
                op_node = self.root_uri(f"compute-{scalar_id}", owner=scope.motion)
                _add_types(self.graph, op_node, GEOM_OP.PoseToAngleAroundAxis)
                self.graph.add((op_node, GEOM_OP.pose, self.root_uri(quantity.name, owner=scope.motion)))
                self.graph.add((op_node, GEOM_OP.angle, self.root_uri(scalar_id, owner=scope.motion)))
                self.graph.add((op_node, GEOM_OP.axis, GEOM_OP[axis]))

    def _add_transform_operations(self) -> None:
        for constraint in self.controlled_constraints:
            quantity = constraint.quantity
            property_name = _view_property_name(constraint.constraint)
            axis_value = constraint.constraint.view.axis
            axis = None if axis_value is None else str(getattr(axis_value, "value", axis_value))
            if (
                quantity is None
                or quantity.type != WorldQuantityType.Pose
                or property_name != ViewProperty.DISTANCE
                or axis is not None
            ):
                continue
            observer_frame = (
                _geometric_property(quantity.props, "wrt")
                if isinstance(quantity.props, GeometricProps)
                else None
            )
            if observer_frame is None:
                raise NotImplementedError(
                    f"Distance derivation for '{quantity.name}' needs an explicit 'wrt' frame."
                )
            raise NotImplementedError(
                "Distance derivation needs model-backed proximal/distal/observer fields; "
                f"refusing to parse them from the authored name '{quantity.name}'."
            )
