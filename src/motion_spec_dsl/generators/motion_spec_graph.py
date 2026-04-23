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
    MAP,
    MOT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    RBDYN_OP,
    QUDT_UNIT,
    RBDYN_COORD,
    RBDYN_ENT,
    SLV,
)
from motion_spec_dsl.generators.classes import (
    BilateralConstraint,
    ConstraintHandler,
    ConstraintSpecification,
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
    VectorQuantity,
    WorldContextDecl,
    WorldQuantity,
    WorldQuantityType,
)

Node: TypeAlias = Any
NamespaceBinding: TypeAlias = tuple[str, Any]
ContextLike: TypeAlias = dict[str, str] | list[str | dict[str, str]]
DatasetOutput: TypeAlias = tuple[Dataset, ContextLike]
ViewAxis: TypeAlias = str
ConstraintSignature: TypeAlias = tuple[str, "ViewProperty", str | None, str]
ConstraintKey: TypeAlias = tuple[str, str]
WorldQuantityLike: TypeAlias = WorldQuantity
ViewKey: TypeAlias = tuple[str, "ViewProperty", ViewAxis]


class ViewProperty(StrEnum):
    ANGULAR = "angular"
    LINEAR = "linear"
    ROTATION = "rotation"
    DISTANCE = "distance"
    FORCE = "force"
    TORQUE = "torque"


class ConstraintKind(StrEnum):
    EQUALITY = "EqualityConstraint"
    GREATER_THAN = "GreaterThanConstraint"
    LESS_THAN = "LessThanConstraint"
    BILATERAL = "BilateralConstraint"


class SolverAlgorithm(StrEnum):
    VERESHCHAGIN = "Vereshchagin"
    NEWTON_EULER = "NewtonEuler"


@dataclass(frozen=True)
class PropertySpec:
    scalar_type: QuantityType
    scalar_prefix: str | None = None
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
    while_constraint_names: frozenset[str]


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
    signature: ConstraintSignature | None = None
    error_signal_id: str | None = None
    acceleration_energy_id: str | None = None
    shared_while: bool = False


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
class CartesianForceBinding:
    spec_name: str
    force_name: str
    attached_to: str | None = None


@dataclass(frozen=True)
class CartesianForceCandidate:
    spec_hint: str | None
    force_name: str
    attached_to: str | None
    explicit_force_command: bool


@dataclass(frozen=True)
class AccelerationConstraintBinding:
    node_id: str
    energy_id: str
    subspace: str
    axis: str


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
                scalar_prefix="angvel",
                accel_prefix="ang",
                view_type=MAP.VelocityTwistCoordinateView,
                view_subspace="angular-velocity",
                accel_subspace="angular-acceleration",
            ),
            ViewProperty.LINEAR: PropertySpec(
                scalar_type=QuantityType.LinearVelocity,
                scalar_prefix="linvel",
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
                scalar_prefix="torque",
                accel_prefix="torque",
                view_type=MAP.WrenchCoordinateView,
                view_subspace="torque",
            ),
            ViewProperty.FORCE: PropertySpec(
                scalar_type=QuantityType.Force,
                scalar_prefix="force",
                accel_prefix="force",
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
                scalar_type=QuantityType.Angle,
                view_type=MAP.PoseCoordinateView,
                view_subspace="rotation",
                accel_subspace="angular-acceleration",
            ),
            ViewProperty.DISTANCE: PropertySpec(
                scalar_type=QuantityType.Distance,
                view_type=MAP.PoseCoordinateView,
                view_subspace="position",
            ),
        },
    ),
}

SCALAR_UNIT: dict[QuantityType | str, Node] = {
    QuantityType.AngularVelocity: QUDT_UNIT["RAD-PER-SEC"],
    QuantityType.LinearVelocity: QUDT_UNIT["M-PER-SEC"],
    QuantityType.Torque: QUDT_UNIT["N-M"],
    QuantityType.Force: QUDT_UNIT.N,
    "Position": QUDT_UNIT.M,
    QuantityType.Angle: QUDT_UNIT["RAD"],
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
    ("geom-ent", GEOM_ENT),
    ("geom-rel", GEOM_REL),
    ("geom-coord", GEOM_COORD),
    ("geom-op", GEOM_OP),
    ("rbdyn-ent", RBDYN_ENT),
    ("rbdyn-coord", RBDYN_COORD),
    ("rbdyn-op", RBDYN_OP),
    ("qudt", QUDT_SCHEMA),
    ("qkind", QUDT_QKIND),
    ("unit", QUDT_UNIT),
    ("map", MAP),
    ("cstr", CSTR),
    ("mot", MOT),
    ("cstr-hdl", CSTR_HDL),
    ("slv", SLV),
)


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


def _dsl_unit(unit_name: str) -> Node:
    try:
        return DSL_UNIT[unit_name]
    except KeyError as exc:
        supported = ", ".join(sorted(DSL_UNIT))
        raise ValueError(
            f"Unsupported DSL unit '{unit_name}'. Use compact DSL units only. "
            f"Supported units: {supported}."
        ) from exc


def _infer_attached_link(
    builder: "MotionSpecDatasetBuilder", handler_spec: ConstraintHandler, solver: Any
) -> str | None:
    del solver
    for binding in builder.cartesian_force_bindings(handler_spec):
        if binding.attached_to is not None:
            return binding.attached_to

    motion = getattr(handler_spec, "motion", None)
    scope = builder.scope_for_motion(motion)
    if scope is not None:
        for quantity in scope.quantities.values():
            if quantity.type == WorldQuantityType.VelocityTwist and isinstance(
                quantity.props, GeometricProps
            ):
                attached_link = _geometric_property(quantity.props, "of")
                if attached_link is not None:
                    return attached_link

    for item in getattr(handler_spec, "context", []):
        for quantity in getattr(item, "declaration", []):
            if quantity.type == WorldQuantityType.VelocityTwist and isinstance(
                quantity.props, GeometricProps
            ):
                attached_link = _geometric_property(quantity.props, "of")
                if attached_link is not None:
                    return attached_link

    return None


def _collect_cartesian_force_bindings(
    builder: "MotionSpecDatasetBuilder", handler_spec: ConstraintHandler
) -> list[CartesianForceBinding]:
    motion = getattr(handler_spec, "motion", None)
    motion_id = _entity_id(motion)
    if not motion_id:
        return []

    spec_names: list[str] = []
    candidates: list[CartesianForceCandidate] = []
    seen_force_names: set[str] = set()
    solver = _primary_solver(handler_spec)
    if solver is None:
        return []
    for force_name in getattr(solver, "cartesian_force", []):
        spec_names.append(f"spec-{force_name}")
        candidates.append(CartesianForceCandidate(f"spec-{force_name}", force_name, None, True))
        seen_force_names.add(force_name)

    for controller in getattr(handler_spec, "controllers", []):
        constraint = builder.constraint_data(controller.params.constraint)
        if (
            constraint is None
            or constraint.quantity is None
            or constraint.quantity.type != WorldQuantityType.Wrench
        ):
            continue
        command_type = (
            str(getattr(controller.command_type, "value", controller.command_type))
            if getattr(controller, "command_type", None)
            else None
        )
        if (
            _view_property_name(constraint.constraint) != ViewProperty.FORCE
            and command_type != QuantityType.Force
        ):
            continue
        force_name = constraint.quantity.name
        if force_name in seen_force_names:
            continue
        explicit_cartesian_force = command_type == QuantityType.Force
        spec_hint = f"spec-{force_name}"
        attached_to = _node_name(controller.apply_at) if controller.apply_at else None
        candidates.append(
            CartesianForceCandidate(spec_hint, force_name, attached_to, explicit_cartesian_force)
        )
        seen_force_names.add(force_name)
        if spec_hint not in spec_names:
            spec_names.append(spec_hint)

    for constraint in builder.constraints_by_motion.get(motion_id, ()):
        if constraint.quantity is None or constraint.quantity.type != WorldQuantityType.Wrench:
            continue
        if constraint.property_name != ViewProperty.FORCE:
            continue
        force_name = constraint.quantity.name
        if force_name in seen_force_names:
            continue
        candidates.append(CartesianForceCandidate(f"spec-{force_name}", force_name, None, False))
        seen_force_names.add(force_name)
        spec_hint = f"spec-{force_name}"
        if spec_hint not in spec_names:
            spec_names.append(spec_hint)

    if not spec_names:
        return []

    bindings: list[CartesianForceBinding] = []
    used_forces: set[str] = set()
    remaining_specs = list(spec_names)
    remaining_candidates = list(candidates)

    for spec_name in list(remaining_specs):
        match = next(
            (candidate for candidate in remaining_candidates if candidate.spec_hint == spec_name),
            None,
        )
        if match is None:
            continue
        bindings.append(CartesianForceBinding(spec_name, match.force_name, match.attached_to))
        used_forces.add(match.force_name)
        remaining_specs.remove(spec_name)
        remaining_candidates.remove(match)

    fallback_candidates = [
        candidate
        for candidate in remaining_candidates
        if candidate.force_name not in used_forces and candidate.explicit_force_command
    ]
    for spec_name, candidate in zip(remaining_specs, fallback_candidates):
        bindings.append(
            CartesianForceBinding(spec_name, candidate.force_name, candidate.attached_to)
        )

    return bindings


def _view_scalar_type(
    quantity: WorldQuantity, property_name: ViewProperty, axis: str | None
) -> QuantityType | str | None:
    if quantity.type == WorldQuantityType.Pose and property_name == ViewProperty.DISTANCE:
        return "Position" if axis is not None else "Length"
    prop = _property_spec(quantity.type, property_name)
    return prop.scalar_type if prop else None


def _scalar_id(
    quantity: WorldQuantity, property_name: ViewProperty, axis: str | None
) -> str | None:
    if axis is None:
        return f"{quantity.name}.{property_name}"
    return f"{quantity.name}.{property_name}.{axis}"


def _while_signature(constraint: ConstraintSpecification) -> ConstraintSignature:
    assert isinstance(constraint.expr, EqualityConstraint)
    axis = constraint.view.axis
    return (
        _entity_id(constraint.view.quantity),
        _view_property_name(constraint),
        None if axis is None else str(getattr(axis, "value", axis)),
        _entity_id(
            getattr(constraint.expr.reference, "value", None)
            or getattr(constraint.expr.reference, "valRef", None)
        ),
    )


def _while_error_id(
    motion_name: str,
    quantity: WorldQuantity,
    property_name: ViewProperty,
    axis: str | None,
    shared: bool,
) -> str | None:
    scalar_id = _scalar_id(quantity, property_name, axis)
    if scalar_id is None:
        return None
    # Shared WHILE equality constraints reuse the same error signal across motions.
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
    builder: "MotionSpecDatasetBuilder", name: str, reason: str
) -> WorldQuantityLike:
    return builder.resolve_world_quantity(name, reason=reason)


def _node_name(value: Any) -> str:
    return value.name if hasattr(value, "name") else str(value)


def _view_property_name(constraint: ConstraintSpecification) -> ViewProperty:
    subspace = constraint.view.subspace
    if subspace is None:
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


def _primary_solver(handler: ConstraintHandler) -> Any | None:
    solvers = getattr(handler, "solvers", [])
    return solvers[0] if solvers else None


def _resolve_constraint_view(
    scope: MotionScope, constraint: ConstraintSpecification
) -> ResolvedConstraintView:
    quantity_name = _node_name(constraint.view.quantity)
    property_name = _view_property_name(constraint)
    axis_value = constraint.view.axis
    axis = None if axis_value is None else str(getattr(axis_value, "value", axis_value))
    quantity = scope.quantities.get(quantity_name)
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
        return [*spec.when.constraints, *spec.while_.constraints, *spec.until.constraints]

    def scope_for_motion(self, motion: MotionSpec | None) -> MotionScope | None:
        if motion is None:
            return None
        return self.motion_scope.get(_entity_id(motion))

    def constraint_data(self, constraint_ref: Any) -> ConstraintData | None:
        target = getattr(constraint_ref, "constraint", constraint_ref)
        return self.constraints_by_ref.get(_entity_id(target))

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

    def cartesian_force_bindings(
        self, handler_spec: ConstraintHandler
    ) -> tuple[CartesianForceBinding, ...]:
        return self.cartesian_force_bindings_by_handler.get(_entity_id(handler_spec), ())

    def _emit_acceleration_energy(self, constraint: ConstraintData) -> None:
        assert constraint.acceleration_energy_id is not None
        node = self.root_uri(constraint.acceleration_energy_id, owner=constraint.constraint.parent)
        acceleration_energy_kind = self.root_uri(
            "AccelerationEnergy", owner=constraint.constraint.parent
        )
        acceleration_energy_unit = self.root_uri(
            "N-M2-PER-SEC2", owner=constraint.constraint.parent
        )
        _add_types(self.graph, node, QUDT_SCHEMA.Quantity, acceleration_energy_kind)
        self.graph.add((node, QUDT_SCHEMA["quantity-kind"], acceleration_energy_kind))
        self.graph.add((node, QUDT_SCHEMA.unit, acceleration_energy_unit))

    def _emit_acceleration_constraint(
        self,
        motion_spec: MotionSpec,
        binding: AccelerationConstraintBinding,
    ) -> None:
        node = self.root_uri(binding.node_id, owner=motion_spec)
        _add_types(self.graph, node, SLV.AccelerationConstraint, SLV.AxisAligned)
        self.graph.add((node, SLV.subspace, SLV[binding.subspace]))
        self.graph.add((node, SLV.axis, SLV[binding.axis]))
        self.graph.add(
            (
                node,
                SLV["acceleration-energy"],
                self.root_uri(binding.energy_id, owner=motion_spec),
            )
        )

    def _emit_cartesian_force_spec(
        self,
        handler_spec: ConstraintHandler,
        motion_spec: MotionSpec,
        binding: CartesianForceBinding,
    ) -> None:
        force_quantity = self.resolve_world_quantity(
            binding.force_name,
            motion=motion_spec,
            handler=handler_spec,
            reason=f"referenced by cartesian-force solver output '{binding.spec_name}'",
        )
        spec_node = self.root_uri(binding.spec_name, owner=handler_spec)
        _add_types(self.graph, spec_node, SLV.CartesianForceSpecification)
        self.graph.add((spec_node, SLV.force, self.node(force_quantity)))
        if binding.attached_to is not None:
            self.graph.add(
                (
                    spec_node,
                    SLV["attached-to"],
                    self.root_uri(binding.attached_to, owner=handler_spec),
                )
            )

    def constraint_quantity_node(self, constraint: ConstraintData) -> Node:
        scope = self.motion_scope.get(constraint.motion_id)
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
                while_constraint_names=frozenset(
                    constraint.name for constraint in motion.while_.constraints
                ),
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
            values.update(scope.values)
        return values

    @cached_property
    def implicit_world_entities(self) -> dict[str, Node]:
        entities: dict[str, Node] = {}
        for quantity in self.world_quantities.values():
            if not isinstance(quantity.props, GeometricProps):
                continue
            if quantity.type == WorldQuantityType.VelocityTwist:
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
            elif quantity.type == WorldQuantityType.Pose:
                for key in ("of", "wrt", "as-seen-by"):
                    target = _geometric_property(quantity.props, key)
                    if target:
                        entities.setdefault(target, GEOM_ENT.Frame)
            elif quantity.type == WorldQuantityType.Wrench:
                point_id = _geometric_property(quantity.props, "ref-point")
                if point_id:
                    entities.setdefault(point_id, GEOM_ENT.Point)
                frame_id = _geometric_property(quantity.props, "as-seen-by")
                if frame_id:
                    entities.setdefault(frame_id, GEOM_ENT.Frame)
        return entities

    @cached_property
    def shared_while_signatures(self) -> set[ConstraintSignature]:
        usage: dict[ConstraintSignature, set[str]] = {}
        for scope in self.motion_scope.values():
            for constraint in scope.motion.while_.constraints:
                if not isinstance(constraint.expr, EqualityConstraint):
                    continue
                usage.setdefault(_while_signature(constraint), set()).add(scope.motion_id)
        return {signature for signature, motions in usage.items() if len(motions) > 1}

    @cached_property
    def constraints(self) -> list[ConstraintData]:
        derived_constraints: list[ConstraintData] = []
        for motion_id, scope in self.motion_scope.items():
            while_names = scope.while_constraint_names
            for constraint in scope.constraints:
                resolved = _resolve_constraint_view(scope, constraint)
                quantity = resolved.quantity
                if quantity is None:
                    raise ValueError(
                        f"Constraint '{constraint.name}' references world quantity "
                        f"'{resolved.quantity_name}' that is not defined in the motion context."
                    )
                quantity_node_id = resolved.scalar_id or resolved.quantity_name

                expr = constraint.expr
                signature = None
                error_signal_id = None
                acceleration_energy = None
                shared = False

                if isinstance(expr, EqualityConstraint):
                    kind = ConstraintKind.EQUALITY
                    reference_var = _node_name(
                        getattr(expr.reference, "value", None)
                        or getattr(expr.reference, "valRef", None)
                    )
                    threshold_var = lower_var = upper_var = None
                    shared = False
                    if constraint.name in while_names and quantity is not None:
                        signature = _while_signature(constraint)
                        shared = signature in self.shared_while_signatures
                        error_signal_id = _while_error_id(
                            scope.motion.name,
                            quantity,
                            resolved.property_name,
                            resolved.axis,
                            shared,
                        )
                        acceleration_energy = _acceleration_energy_id(
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
                    constraint.name in while_names
                    and quantity is not None
                    and error_signal_id is None
                ):
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
                        signature=signature,
                        error_signal_id=error_signal_id,
                        acceleration_energy_id=acceleration_energy,
                        shared_while=shared if isinstance(expr, EqualityConstraint) else False,
                    )
                )
        return derived_constraints

    @cached_property
    def constraints_by_ref(self) -> dict[str, ConstraintData]:
        return {_entity_id(constraint.constraint): constraint for constraint in self.constraints}

    @cached_property
    def constraints_by_motion(self) -> dict[str, tuple[ConstraintData, ...]]:
        grouped: dict[str, list[ConstraintData]] = {}
        for constraint in self.constraints:
            grouped.setdefault(constraint.motion_id, []).append(constraint)
        return {motion_id: tuple(values) for motion_id, values in grouped.items()}

    @cached_property
    def while_constraints(self) -> list[ConstraintData]:
        return [
            constraint
            for constraint in self.constraints
            if (
                (scope := self.motion_scope.get(constraint.motion_id)) is not None
                and constraint.constraint.name in scope.while_constraint_names
            )
        ]

    @cached_property
    def cartesian_force_bindings_by_handler(self) -> dict[str, tuple[CartesianForceBinding, ...]]:
        return {
            _entity_id(handler): tuple(_collect_cartesian_force_bindings(self, handler))
            for handler in self.authored_handlers
        }

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
        self._add_error_signals_and_energies()
        self._add_solver_entities()
        self._add_map_operations()
        self._add_transform_operations()

    def _add_structural_entities(self) -> None:
        entities: dict[str, tuple[Node, Node]] = {}
        for quantity in self.world_quantities.values():
            if WORLD_SPECS.get(quantity.type) is None:
                entities[quantity.name] = (URIRef(quantity.uri), GEOM_ENT[quantity.type])
        for entity_name, rdf_type in self.implicit_world_entities.items():
            entities.setdefault(entity_name, (self.root_uri(entity_name), rdf_type))
        for node, rdf_type in sorted(entities.values(), key=lambda item: str(item[0])):
            _add_types(self.graph, node, rdf_type)

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
            qkind = QUDT_QKIND[variable.type]
            _add_types(self.graph, node, QUDT_SCHEMA.Quantity, qkind)
            self.graph.add((node, QUDT_SCHEMA["quantity-kind"], qkind))
            if variable.value is None:
                continue
            self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(variable.value.unit)))
            if isinstance(variable.value, ScalarQuantity):
                self.graph.add((node, QUDT_SCHEMA.value, Literal(str(variable.value.value))))
            elif isinstance(variable.value, VectorQuantity):
                self.graph.add(
                    (
                        node,
                        QUDT_SCHEMA.value,
                        Literal(f"{variable.value.x} {variable.value.y} {variable.value.z}"),
                    )
                )

    def _add_constraints(self) -> None:
        for constraint in self.constraints:
            node = URIRef(constraint.constraint.uri)
            type_name = constraint.scalar_type
            _add_types(self.graph, node, CSTR.Constraint, CSTR[f"{type_name}Constraint"])
            self.graph.add((node, CSTR.quantity, self.constraint_quantity_node(constraint)))
            scope = self.motion_scope[constraint.motion_id]
            motion_spec = scope.motion
            motion_values = scope.values
            if constraint.kind == ConstraintKind.EQUALITY:
                reference = motion_values.get(constraint.reference_var or "")
                _add_types(self.graph, node, CSTR.EqualityConstraint)
                ref_node = (
                    URIRef(reference.uri)
                    if reference
                    else self.root_uri(constraint.reference_var or "", owner=motion_spec)
                )
                self.graph.add((node, CSTR["reference-value"], ref_node))
            elif constraint.kind == ConstraintKind.GREATER_THAN:
                threshold = motion_values.get(constraint.threshold_var or "")
                _add_types(self.graph, node, CSTR.UnilateralConstraint, CSTR.GreaterThanConstraint)
                threshold_node = (
                    URIRef(threshold.uri)
                    if threshold
                    else self.root_uri(constraint.threshold_var or "", owner=motion_spec)
                )
                self.graph.add((node, CSTR.threshold, threshold_node))
            elif constraint.kind == ConstraintKind.LESS_THAN:
                threshold = motion_values.get(constraint.threshold_var or "")
                _add_types(self.graph, node, CSTR.UnilateralConstraint, CSTR.LessThanConstraint)
                threshold_node = (
                    URIRef(threshold.uri)
                    if threshold
                    else self.root_uri(constraint.threshold_var or "", owner=motion_spec)
                )
                self.graph.add((node, CSTR.threshold, threshold_node))
            else:
                lower = motion_values.get(constraint.lower_var or "")
                upper = motion_values.get(constraint.upper_var or "")
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
                self.graph.add((motion_node, MOT.when, URIRef(constraint.uri)))
            for constraint in motion_spec.while_.constraints:
                self.graph.add((motion_node, MOT["while"], URIRef(constraint.uri)))
            for constraint in motion_spec.until.constraints:
                self.graph.add((motion_node, MOT.until, URIRef(constraint.uri)))

    def _add_constraint_handlers(self) -> None:
        while_constraints_by_motion: dict[str, list[ConstraintData]] = {}
        for constraint in self.while_constraints:
            while_constraints_by_motion.setdefault(constraint.motion_id, []).append(constraint)

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

            for constraint in while_constraints_by_motion.get(motion_id, []):
                self.graph.add(
                    (
                        node,
                        CSTR_HDL.evaluators,
                        self.root_uri(
                            f"eval-{constraint.constraint.name}", owner=constraint.constraint.parent
                        ),
                    )
                )

            for controller in getattr(handler_spec, "controllers", []):
                constraint = self.constraint_data(controller.params.constraint)
                if constraint is None:
                    raise ValueError(
                        f"Controller '{controller.name}' references constraint "
                        f"'{controller.params.constraint}' that is not defined for motion '{scope.motion.name}'."
                    )
                command_type = (
                    str(getattr(controller.command_type, "value", controller.command_type))
                    if getattr(controller, "command_type", None)
                    else None
                )
                control_signal_id = (
                    constraint.acceleration_energy_id
                    or (
                        constraint.quantity_node_id
                        if constraint.quantity is not None
                        and constraint.property_name == ViewProperty.FORCE
                        else None
                    )
                    or f"eacc-{controller.name}"
                )
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
                            self.root_uri(constraint.error_signal_id),
                        )
                    )
                self.graph.add(
                    (controller_node, CSTR_HDL["control-signal"], self.root_uri(control_signal_id))
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
                if command_type == QuantityType.Force:
                    _add_quantity(self.graph, self.root_uri(control_signal_id), QuantityType.Force)

            for monitor in getattr(handler_spec, "monitors", []):
                constraint = self.constraint_data(monitor.constraint)
                if constraint is None:
                    continue
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
                        self.root_uri(f"{monitor.constraint.name}-err", owner=monitor),
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
                        self.root_uri(f"eval-{monitor.constraint.name}", owner=monitor),
                    )
                )

    def _add_scalar_views(self) -> None:
        existing_scalar_views: set[str] = set()
        seen_view_keys: set[ViewKey] = set()
        rotation_ids: set[str] = set()
        for scope in self.motion_scope.values():
            for constraint in scope.constraints:
                resolved = _resolve_constraint_view(scope, constraint)
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
                self.graph.add(
                    (
                        node,
                        MAP.superobject,
                        self.root_uri(resolved.quantity_name, owner=scope.motion),
                    )
                )
                self.graph.add(
                    (node, MAP.subobject, self.root_uri(resolved.scalar_id, owner=scope.motion))
                )
                if resolved.property_spec.view_subspace:
                    self.graph.add((node, MAP.subspace, MAP[resolved.property_spec.view_subspace]))
                self.graph.add((node, MAP.axis, MAP[resolved.axis]))
        for constraint in self.constraints:
            if constraint.scalar_type == QuantityType.Angle:
                _add_quantity(
                    self.graph, self.constraint_quantity_node(constraint), QuantityType.Angle
                )
        for rotation_id in rotation_ids:
            _add_quantity(self.graph, self.root_uri(rotation_id), QuantityType.Angle)
        for handler_spec in self.authored_handlers:
            solver = _primary_solver(handler_spec)
            if solver is None or not getattr(solver, "algorithm", ""):
                continue
            for binding in self.cartesian_force_bindings(handler_spec):
                quantity = self.resolve_world_quantity(
                    binding.force_name,
                    motion=getattr(handler_spec, "motion", None),
                    handler=handler_spec,
                    reason=f"referenced by cartesian-force solver output '{binding.spec_name}'",
                )
                if quantity is None or quantity.type != WorldQuantityType.Wrench:
                    continue
                scalar_id = _scalar_id(quantity, ViewProperty.FORCE, "z")
                if scalar_id is None or scalar_id in existing_scalar_views:
                    continue
                existing_scalar_views.add(scalar_id)
                _add_quantity(
                    self.graph, self.root_uri(scalar_id, owner=quantity), QuantityType.Force
                )
                node = self.root_uri(f"view-{scalar_id}", owner=quantity)
                _add_types(self.graph, node, MAP.View, MAP.WrenchCoordinateView)
                self.graph.add((node, MAP.superobject, self.node(quantity)))
                self.graph.add((node, MAP.subobject, self.root_uri(scalar_id, owner=quantity)))
                self.graph.add((node, MAP.subspace, MAP.force))
                self.graph.add((node, MAP.axis, MAP.z))

    def _add_error_signals_and_energies(self) -> None:
        for constraint in self.while_constraints:
            if constraint.error_signal_id is not None:
                _add_quantity(
                    self.graph, self.root_uri(constraint.error_signal_id), constraint.scalar_type
                )
        for handler_spec in self.authored_handlers:
            motion = getattr(handler_spec, "motion", None)
            motion_id = _entity_id(motion)
            if not motion_id:
                continue
            for monitor in getattr(handler_spec, "monitors", []):
                constraint = self.constraint_data(monitor.constraint)
                if constraint is not None:
                    _add_quantity(
                        self.graph,
                        self.root_uri(f"{monitor.constraint.name}-err", owner=monitor),
                        constraint.scalar_type,
                    )
        for constraint in self.constraints:
            if constraint.signature is None:
                continue
            if constraint.acceleration_energy_id is None:
                continue
            self._emit_acceleration_energy(constraint)
        for constraint in self.while_constraints:
            if constraint.error_signal_id is None:
                continue
            node = self.root_uri(
                f"eval-{constraint.constraint.name}", owner=constraint.constraint.parent
            )
            _add_types(self.graph, node, CSTR_HDL.ConstraintEvaluator, CSTR_HDL.ErrorEvaluator)
            self.graph.add((node, CSTR_HDL.constraint, URIRef(constraint.constraint.uri)))
            self.graph.add(
                (
                    node,
                    CSTR_HDL.error,
                    self.root_uri(constraint.error_signal_id, owner=constraint.constraint.parent),
                )
            )
        for handler_spec in self.authored_handlers:
            motion = getattr(handler_spec, "motion", None)
            motion_id = _entity_id(motion)
            if not motion_id:
                continue
            for monitor in getattr(handler_spec, "monitors", []):
                constraint = self.constraint_data(monitor.constraint)
                if constraint is None:
                    continue
                node = self.root_uri(f"eval-{monitor.constraint.name}", owner=monitor)
                _add_types(self.graph, node, CSTR_HDL.ConstraintEvaluator, CSTR_HDL.ErrorEvaluator)
                self.graph.add((node, CSTR_HDL.constraint, URIRef(constraint.constraint.uri)))
                self.graph.add(
                    (
                        node,
                        CSTR_HDL.error,
                        self.root_uri(f"{monitor.constraint.name}-err", owner=monitor),
                    )
                )

    def _add_solver_entities(self) -> None:
        self._add_base_solvers()
        self._add_handler_solvers()

    def _add_base_solvers(self) -> None:
        for handler_spec in self.authored_handlers:
            solver_group = _primary_solver(handler_spec)
            if solver_group is None:
                continue
            for solver in getattr(solver_group, "velocity_solvers", []):
                _required_world_quantity(
                    self,
                    solver.velocity,
                    f"referenced by base velocity solver '{solver.name}'",
                )
                node = self.root_uri(solver.name)
                _add_types(self.graph, node, SLV.VelocityCompositionSolver)
                self.graph.add((node, SLV.configuration, Literal(solver.configuration)))
                velocity_quantity = self.resolve_world_quantity(
                    solver.velocity,
                    handler=handler_spec,
                    reason=f"referenced by base velocity solver '{solver.name}'",
                )
                self.graph.add((node, SLV.velocity, self.node(velocity_quantity)))
            for solver in getattr(solver_group, "force_solvers", []):
                _required_world_quantity(
                    self,
                    solver.force,
                    f"referenced by base force solver '{solver.name}'",
                )
                node = self.root_uri(solver.name)
                _add_types(self.graph, node, SLV.ForceDistributionSolver)
                self.graph.add((node, SLV.configuration, Literal(solver.configuration)))
                force_quantity = self.resolve_world_quantity(
                    solver.force,
                    handler=handler_spec,
                    reason=f"referenced by base force solver '{solver.name}'",
                )
                self.graph.add((node, SLV.force, self.node(force_quantity)))

    def _add_handler_solvers(self) -> None:
        for handler_spec in self.authored_handlers:
            solver = _primary_solver(handler_spec)
            motion = getattr(handler_spec, "motion", None)
            motion_id = _entity_id(motion)
            if not motion_id or solver is None or not getattr(solver, "algorithm", ""):
                continue
            scope = self.motion_scope.get(motion_id)
            if scope is None:
                continue
            motion_spec = scope.motion
            attached_link = _infer_attached_link(self, handler_spec, solver)
            spec_constraints: list[str] = []
            acceleration_constraints: dict[str, AccelerationConstraintBinding] = {}
            for constraint in self.constraints_by_motion.get(motion_id, ()):
                if constraint.signature is None:
                    continue
                quantity = constraint.quantity
                property_spec = (
                    _property_spec(quantity.type, constraint.property_name) if quantity else None
                )
                if (
                    quantity is None
                    or property_spec is None
                    or property_spec.accel_subspace is None
                ):
                    continue
                if constraint.acceleration_energy_id is None:
                    continue
                node_id = _acceleration_constraint_id(
                    motion_spec.name,
                    quantity,
                    constraint.property_name,
                    constraint.axis,
                    constraint.shared_while,
                )
                if node_id not in spec_constraints:
                    spec_constraints.append(node_id)
                acceleration_constraints.setdefault(
                    node_id,
                    AccelerationConstraintBinding(
                        node_id=node_id,
                        energy_id=constraint.acceleration_energy_id,
                        subspace=property_spec.accel_subspace,
                        axis=constraint.axis or "",
                    ),
                )
                self._emit_acceleration_constraint(
                    motion_spec,
                    acceleration_constraints[node_id],
                )

            spec_acc_node = (
                self.root_uri(f"spec-acc-{attached_link}-{motion_spec.name}", owner=motion_spec)
                if attached_link
                else self.root_uri(f"spec-acc-ee-{motion_spec.name}", owner=motion_spec)
            )
            _add_types(self.graph, spec_acc_node, SLV.AccelerationConstraintSpecification)
            for node_id in spec_constraints:
                self.graph.add(
                    (spec_acc_node, SLV.constraints, self.root_uri(node_id, owner=motion_spec))
                )
            attached_to = (
                self.root_uri(attached_link, owner=handler_spec)
                if attached_link
                else self.root_uri("link-ee", owner=handler_spec)
            )
            self.graph.add((spec_acc_node, SLV["attached-to"], attached_to))

            for binding in self.cartesian_force_bindings(handler_spec):
                self._emit_cartesian_force_spec(handler_spec, motion_spec, binding)

            driver_name = motion_spec.name or handler_spec.name
            driver_node = self.root_uri(f"drv-{driver_name}", owner=handler_spec)
            _add_types(self.graph, driver_node, SLV.MotionDrivers)
            self.graph.add((driver_node, SLV["acceleration-constraint"], spec_acc_node))
            for binding in self.cartesian_force_bindings(handler_spec):
                self.graph.add(
                    (
                        driver_node,
                        SLV["cartesian-force"],
                        self.root_uri(binding.spec_name, owner=handler_spec),
                    )
                )
            for joint_force in getattr(solver, "joint_force", []):
                self.graph.add(
                    (
                        driver_node,
                        SLV["joint-force"],
                        self.root_uri(joint_force, owner=handler_spec),
                    )
                )

            solver_node = self.root_uri(f"slv-{driver_name}", owner=handler_spec)
            solver_algorithm = (
                SLV.AccelerationConstrainedHybridDynamicsAlgorithm
                if solver.algorithm == SolverAlgorithm.VERESHCHAGIN
                else SLV.NewtonEulerAlgorithm
                if solver.algorithm == SolverAlgorithm.NEWTON_EULER
                else SLV[solver.algorithm]
            )
            _add_types(self.graph, solver_node, SLV.SolverWithInputAndOutput)
            self.graph.add((solver_node, SLV.solver, solver_algorithm))
            gravity_name = _node_name(solver.gravity)
            gravity_quantity = self.resolve_world_quantity(
                gravity_name,
                handler=handler_spec,
                reason=f"referenced by solver '{handler_spec.name}' gravity",
            )
            self.graph.add(
                (
                    solver_node,
                    SLV["kinematic-chain"],
                    self.root_uri(_node_name(solver.robot), owner=handler_spec),
                )
            )
            self.graph.add(
                (solver_node, SLV.root, self.root_uri(_node_name(solver.root), owner=handler_spec))
            )
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
                op_node = self.root_uri(f"compute-{scalar_id}")
                _add_types(self.graph, op_node, GEOM_OP.PoseToAngleAroundAxis)
                self.graph.add((op_node, GEOM_OP.pose, self.root_uri(quantity.name)))
                self.graph.add((op_node, GEOM_OP.angle, self.root_uri(scalar_id)))
                self.graph.add((op_node, GEOM_OP.axis, MAP[axis]))

    def _add_transform_operations(self) -> None:
        for constraint in self.while_constraints:
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
