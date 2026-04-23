# SPDX-License-Identifier: MPL-2.0
"""Build the JSON-LD output graphs for the motion-spec DSL.

The implementation is intentionally kept in a single file, but it is organized
around one cached analysis object. Emitters consume derived records rather than
re-walking the DSL model and recomputing identifiers.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass
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
    AccelerationEnergy,
    BilateralConstraint,
    ConstraintHandler,
    ConstraintSpecification,
    ControllerParams,
    EqualityConstraint,
    ErrorSignal,
    Evaluator,
    GeoPropPair,
    GeometricProps,
    GreaterThanConstraint,
    LessThanConstraint,
    Model,
    MonitorEntry,
    Motion,
    MotionSpec,
    PostContextDecl,
    PreContextDecl,
    ScalarView,
    SpecContextDecl,
    ValueVariable,
    WorldContextDecl,
    WorldQuantity,
)

Node: TypeAlias = Any
NamespaceBinding: TypeAlias = tuple[str, Any]
ContextLike: TypeAlias = dict[str, str] | list[str | dict[str, str]]
DatasetOutput: TypeAlias = tuple[Dataset, ContextLike]
ConstraintSignature: TypeAlias = tuple[str, str, str | None, str]
ConstraintKey: TypeAlias = tuple[str, str]
WorldQuantityLike: TypeAlias = WorldQuantity

@dataclass(frozen=True)
class PropertySpec:
    scalar_type: str
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
    properties: dict[str, PropertySpec]


@dataclass(frozen=True)
class ConstraintData:
    motion_name: str
    constraint: ConstraintSpecification
    quantity: WorldQuantity | None
    kind: str
    scalar_type: str
    quantity_node_id: str
    reference_var: str | None = None
    threshold_var: str | None = None
    lower_var: str | None = None
    upper_var: str | None = None
    signature: ConstraintSignature | None = None
    error_signal_id: str | None = None
    acceleration_energy_id: str | None = None


@dataclass(frozen=True)
class MonitorData:
    motion_name: str
    monitor: MonitorEntry
    constraint: ConstraintData
    evaluator: Evaluator
    error_signal: ErrorSignal
    signal_kind: str
    signal_id: str


@dataclass(frozen=True)
class ControllerData:
    motion_name: str
    controller: Any
    constraint: ConstraintData | None
    error_signal_id: str | None
    control_signal_id: str
    output_type: str | None
    apply_at: str | None
    feed_scope: str | None
    feed_kind: str | None
    params: ControllerParams


@dataclass(frozen=True)
class HandlerData:
    node_id: str
    motion_name: str
    motion: Motion
    evaluators: tuple[Evaluator, ...]
    controller_ids: tuple[str, ...]
    monitor_ids: tuple[str, ...]


@dataclass(frozen=True)
class AccelerationConstraintData:
    node_id: str
    energy_id: str
    subspace: str
    axis: str


@dataclass(frozen=True)
class BaseVelocitySolverData:
    node_id: str
    configuration: str
    velocity: str


@dataclass(frozen=True)
class BaseForceSolverData:
    node_id: str
    configuration: str
    force: str


@dataclass(frozen=True)
class CartesianForceBinding:
    spec_name: str
    force_name: str
    attached_to: str | None


@dataclass(frozen=True)
class PoseAngleOpData:
    op_id: str
    pose_name: str
    angle_id: str
    axis: str


@dataclass(frozen=True)
class DistanceDerivation:
    constraint_name: str
    pose_name: str
    group: str
    proximal: str
    distal: str
    observer_frame: str
    observer_point: str
    distance_id: str
    direction_id: str
    position_id: str
    force_id: str
    local_wrench_id: str
    pose_to_dist_op_id: str
    pose_to_dir_op_id: str
    wrench_op_id: str


WORLD_SPECS: dict[str, WorldSpec] = {
    "VelocityTwist": WorldSpec(
        rdf_types=(
            GEOM_REL.VelocityTwist,
            GEOM_COORD.VelocityTwistCoordinate,
            GEOM_COORD.VectorXYZ,
        ),
        qkinds=(QUDT_QKIND.AngularVelocity, QUDT_QKIND.LinearVelocity),
        units=(QUDT_UNIT["RAD-PER-SEC"], QUDT_UNIT["M-PER-SEC"]),
        properties={
            "angular": PropertySpec(
                scalar_type="AngularVelocity",
                scalar_prefix="angvel",
                accel_prefix="ang",
                view_type=MAP.VelocityTwistCoordinateView,
                view_subspace="angular-velocity",
                accel_subspace="angular-acceleration",
            ),
            "linear": PropertySpec(
                scalar_type="LinearVelocity",
                scalar_prefix="linvel",
                accel_prefix="lin",
                view_type=MAP.VelocityTwistCoordinateView,
                view_subspace="linear-velocity",
                accel_subspace="linear-acceleration",
            ),
        },
    ),
    "Wrench": WorldSpec(
        rdf_types=(RBDYN_ENT.Wrench, RBDYN_COORD.WrenchCoordinate, GEOM_COORD.VectorXYZ),
        qkinds=(QUDT_QKIND.Torque, QUDT_QKIND.Force),
        units=(QUDT_UNIT["N-M"], QUDT_UNIT.N),
        properties={
            "torque": PropertySpec(
                scalar_type="Torque",
                scalar_prefix="torque",
                accel_prefix="torque",
                view_type=MAP.WrenchCoordinateView,
                view_subspace="torque",
            ),
            "force": PropertySpec(
                scalar_type="Force",
                scalar_prefix="force",
                accel_prefix="force",
                view_type=MAP.WrenchCoordinateView,
                view_subspace="force",
            ),
        },
    ),
    "Pose": WorldSpec(
        rdf_types=(
            GEOM_REL.Pose,
            GEOM_COORD.PoseCoordinate,
            GEOM_COORD.DirectionCosineXYZ,
            GEOM_COORD.VectorXYZ,
        ),
        qkinds=(QUDT_QKIND.PlaneAngle, QUDT_QKIND.Length),
        units=(QUDT_UNIT.UNITLESS, QUDT_UNIT.M),
        properties={
            "rotation": PropertySpec(
                scalar_type="PlaneAngle",
                view_type=MAP.PoseCoordinateView,
                view_subspace="rotation",
                accel_subspace="angular-acceleration",
            ),
            "distance": PropertySpec(
                scalar_type="Length",
                view_type=MAP.PoseCoordinateView,
                view_subspace="position",
            ),
        },
    ),
}

SCALAR_UNIT = {
    "AngularVelocity": QUDT_UNIT["RAD-PER-SEC"],
    "LinearVelocity": QUDT_UNIT["M-PER-SEC"],
    "Torque": QUDT_UNIT["N-M"],
    "Force": QUDT_UNIT.N,
    "Position": QUDT_UNIT.M,
    "PlaneAngle": QUDT_UNIT["RAD"],
    "Length": QUDT_UNIT.M,
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


def _dataset(*bindings: NamespaceBinding) -> Dataset:
    dataset = Dataset()
    for prefix, namespace in bindings:
        dataset.bind(prefix, namespace)
    return dataset


def _add_types(graph: Any, node: Node, *rdf_types: Node) -> None:
    for rdf_type in rdf_types:
        graph.add((node, RDF.type, rdf_type))


def _add_quantity(graph: Any, node: Node, scalar_type: str) -> None:
    qkind = QUDT_QKIND[scalar_type]
    _add_types(graph, node, QUDT_SCHEMA.Quantity, qkind)
    graph.add((node, QUDT_SCHEMA["quantity-kind"], qkind))
    graph.add((node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(scalar_type, QUDT_UNIT.UNITLESS)))


def _property_spec(world_type: str, property_name: str) -> PropertySpec | None:
    spec = WORLD_SPECS.get(world_type)
    return spec.properties.get(property_name) if spec else None


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
    for binding in _infer_cartesian_force_bindings(builder, handler_spec):
        if binding.attached_to is not None:
            return binding.attached_to

    motion_name = _handler_motion_name(handler_spec)
    motion_spec = builder.motion_map.get(motion_name) if motion_name else None
    if motion_spec is not None:
        for quantity in builder.world_declarations(motion_spec):
            if quantity.type == "VelocityTwist" and isinstance(quantity.props, GeometricProps):
                attached_link = _geometric_property(quantity.props, "of")
                if attached_link is not None:
                    return attached_link

    for item in _handler_contexts(handler_spec):
        for quantity in _context_declarations(item):
            if quantity.type == "VelocityTwist" and isinstance(quantity.props, GeometricProps):
                attached_link = _geometric_property(quantity.props, "of")
                if attached_link is not None:
                    return attached_link

    return None


def _constraint_property_name(constraint: ConstraintData) -> str:
    return _view_property_name(constraint.constraint)


def _controller_command_kind(controller: ControllerData) -> str | None:
    return controller.output_type or controller.feed_kind


def _controller_force_quantity(controller: ControllerData) -> WorldQuantity | None:
    constraint = controller.constraint
    if constraint is None or constraint.quantity is None:
        return None
    if constraint.quantity.type != "Wrench":
        return None
    property_name = _constraint_property_name(constraint)
    if property_name == "force":
        return constraint.quantity
    if _controller_command_kind(controller) == "Force":
        return constraint.quantity
    return None


def _infer_cartesian_force_bindings(
    builder: "MotionSpecDatasetBuilder", handler_spec: ConstraintHandler
) -> list[CartesianForceBinding]:
    motion_name = _handler_motion_name(handler_spec)
    if not motion_name:
        return []

    spec_names: list[str] = []
    candidates: list[tuple[str | None, str, str | None, bool]] = []
    seen_force_names: set[str] = set()
    solver = _primary_solver(handler_spec)
    if solver is None:
        return []
    for force_name in getattr(solver, "cartesian_force", []):
        spec_names.append(f"spec-{force_name}")
        candidates.append(
            (
                f"spec-{force_name}",
                force_name,
                None,
                True,
            )
        )
        seen_force_names.add(force_name)

    motion_controllers = [
        controller
        for controller in builder.controller_data.values()
        if controller.motion_name == motion_name
    ]
    for controller in motion_controllers:
        force_quantity = _controller_force_quantity(controller)
        if force_quantity is None or force_quantity.name in seen_force_names:
            continue
        explicit_cartesian_force = _controller_command_kind(controller) == "Force"
        force_name = force_quantity.name

        spec_hint = None
        if explicit_cartesian_force and controller.apply_at:
            spec_hint = f"spec-{force_name}"
        else:
            spec_hint = f"spec-{force_name}"

        attached_to = controller.apply_at
        candidates.append((spec_hint, force_name, attached_to, explicit_cartesian_force))
        seen_force_names.add(force_name)
        if spec_hint is not None and spec_hint not in spec_names:
            spec_names.append(spec_hint)

    for constraint in builder.constraints:
        if constraint.motion_name != motion_name or constraint.quantity is None or constraint.quantity.type != "Wrench":
            continue
        if _constraint_property_name(constraint) != "force":
            continue
        force_name = constraint.quantity.name
        if force_name in seen_force_names:
            continue
        candidates.append(
            (f"spec-{force_name}", force_name, None, False)
        )
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
        match = next((candidate for candidate in remaining_candidates if candidate[0] == spec_name), None)
        if match is None:
            continue
        bindings.append(CartesianForceBinding(spec_name=spec_name, force_name=match[1], attached_to=match[2]))
        used_forces.add(match[1])
        remaining_specs.remove(spec_name)
        remaining_candidates.remove(match)

    fallback_candidates = [
        candidate for candidate in remaining_candidates if candidate[1] not in used_forces and candidate[3]
    ]
    for spec_name, candidate in zip(remaining_specs, fallback_candidates):
        bindings.append(CartesianForceBinding(spec_name=spec_name, force_name=candidate[1], attached_to=candidate[2]))

    return bindings


def _driver_suffix(handler_spec: ConstraintHandler) -> str:
    motion_name = _handler_motion_name(handler_spec)
    return motion_name if motion_name else handler_spec.name


def _view_scalar_type(quantity: WorldQuantity, property_name: str, axis: str | None) -> str | None:
    if quantity.type == "Pose" and property_name == "distance":
        return "Position" if axis is not None else "Length"
    prop = _property_spec(quantity.type, property_name)
    return prop.scalar_type if prop else None


def _scalar_id(quantity: WorldQuantity, property_name: str, axis: str | None) -> str | None:
    if axis is None:
        return f"{quantity.name}.{property_name}"
    return f"{quantity.name}.{property_name}.{axis}"


def _while_signature(constraint: ConstraintSpecification) -> ConstraintSignature:
    assert isinstance(constraint.expr, EqualityConstraint)
    return (
        _view_quantity_name(constraint),
        _view_property_name(constraint),
        _view_axis_name(constraint),
        _context_value_name(constraint.expr.reference),
    )


def _while_error_id(
    motion_name: str,
    quantity: WorldQuantity,
    property_name: str,
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
    property_name: str,
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
    property_name: str,
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


def _control_signal_id(constraint: ConstraintData | None) -> str | None:
    if constraint is None:
        return None
    if constraint.acceleration_energy_id:
        return constraint.acceleration_energy_id
    if constraint.quantity is not None and _constraint_property_name(constraint) == "force":
        return constraint.quantity_node_id
    return None


def _required_world_quantity(builder: "MotionSpecDatasetBuilder", name: str, reason: str) -> WorldQuantityLike:
    quantity = builder.world_quantities.get(name)
    if quantity is None:
        raise ValueError(f"Missing required world quantity '{name}' in context: {reason}")
    return quantity


def _reference_frame(quantity: WorldQuantityLike) -> str | None:
    if not isinstance(quantity.props, GeometricProps):
        return None
    return _geometric_property(quantity.props, "as-seen-by")


def _with_respect_to(quantity: WorldQuantityLike) -> str | None:
    if not isinstance(quantity.props, GeometricProps):
        return None
    return _geometric_property(quantity.props, "wrt")


def _of_frame(quantity: WorldQuantityLike) -> str | None:
    if not isinstance(quantity.props, GeometricProps):
        return None
    return _geometric_property(quantity.props, "of")


def _node_name(value: Any) -> str:
    return value.name if hasattr(value, "name") else str(value)


def _context_value_name(lookup: Any) -> str:
    value = getattr(lookup, "value", None) or getattr(lookup, "valRef", None)
    return _node_name(value)


def _view_quantity_name(constraint: ConstraintSpecification) -> str:
    return _node_name(constraint.view.quantity)


def _view_property_name(constraint: ConstraintSpecification) -> str:
    subspace = constraint.view.subspace
    if subspace is None:
        return ""
    value = getattr(subspace, "value", subspace)
    return {
        "angvel": "angular",
        "linvel": "linear",
        "orientation": "rotation",
        "position": "distance",
    }.get(str(value), str(value))


def _view_axis_name(constraint: ConstraintSpecification) -> str | None:
    axis = constraint.view.axis
    return None if axis is None else str(getattr(axis, "value", axis))


def _handler_motion(handler: ConstraintHandler) -> MotionSpec | None:
    motion = getattr(handler, "motion", None)
    return motion if isinstance(motion, MotionSpec) else None


def _handler_motion_name(handler: ConstraintHandler) -> str:
    motion = getattr(handler, "motion", None)
    return _node_name(motion) if motion is not None else ""


def _handler_contexts(handler: ConstraintHandler) -> list[Any]:
    return list(getattr(handler, "context", []))


def _handler_controllers(handler: ConstraintHandler) -> list[Any]:
    return list(getattr(handler, "controllers", []))


def _handler_monitors(handler: ConstraintHandler) -> list[Any]:
    return list(getattr(handler, "monitors", []))


def _handler_solvers(handler: ConstraintHandler) -> list[Any]:
    return list(getattr(handler, "solvers", []))


def _primary_solver(handler: ConstraintHandler) -> Any | None:
    solvers = _handler_solvers(handler)
    return solvers[0] if solvers else None


def _context_declarations(context: Any) -> list[Any]:
    return list(getattr(context, "declaration", []))


class MotionSpecDatasetBuilder:
    """Handler-rooted builder that resolves semantics and materializes a Dataset."""

    def __init__(self, model: Model):
        self.model = model
        self.models = get_included_models(model)
        self.dataset = _dataset(
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
        self.graph = self.dataset.default_graph

    @cached_property
    def namespace_owners(self) -> list[Any]:
        return [*self.handler_specs, *self.motion_specs]

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

    def _context_decl(self, spec: MotionSpec, kind: str) -> Any | None:
        for item in spec.context:
            if item.__class__.__name__ == kind:
                return item
        return None

    def world_declarations(self, spec: MotionSpec) -> list[WorldQuantity]:
        declaration = self._context_decl(spec, "WorldContextDecl")
        return declaration.declaration if isinstance(declaration, WorldContextDecl) else []

    def value_declarations(self, spec: MotionSpec, kind: str) -> list[ValueVariable]:
        declaration = self._context_decl(spec, kind)
        if isinstance(declaration, (PreContextDecl, SpecContextDecl, PostContextDecl)):
            return declaration.declaration
        return []

    def all_constraints(self, spec: MotionSpec) -> list[ConstraintSpecification]:
        return [*spec.when.constraints, *spec.while_.constraints, *spec.until.constraints]

    def world_quantity(self, spec: MotionSpec, name: str) -> WorldQuantity | None:
        name = _node_name(name)
        for quantity in self.world_declarations(spec):
            if quantity.name == name:
                return quantity
        return None

    def value_variable(self, spec: MotionSpec, name: str) -> ValueVariable | None:
        name = _node_name(name)
        for kind in ("PreContextDecl", "SpecContextDecl", "PostContextDecl"):
            for value in self.value_declarations(spec, kind):
                if value.name == name:
                    return value
        return None

    def constraint_quantity_node(self, constraint: ConstraintData) -> Node:
        return self.node(constraint.quantity_node_id, owner=self.motion_map.get(constraint.motion_name))

    @cached_property
    def handler_specs(self) -> list[ConstraintHandler]:
        return [
            spec
            for model in self.models
            for spec in model.specs
            if isinstance(spec, ConstraintHandler)
        ]

    @cached_property
    def motion_specs(self) -> list[MotionSpec]:
        motions: list[MotionSpec] = []
        seen: set[str] = set()
        for handler in self.handler_specs:
            motion = _handler_motion(handler)
            if motion is None or motion.name in seen:
                continue
            seen.add(motion.name)
            motions.append(motion)
        return motions

    @cached_property
    def motion_map(self) -> dict[str, MotionSpec]:
        return {motion.name: motion for motion in self.motion_specs}

    @cached_property
    def world_quantities(self) -> dict[str, WorldQuantityLike]:
        quantities: dict[str, WorldQuantityLike] = {}
        for motion in self.motion_specs:
            for quantity in self.world_declarations(motion):
                quantities[quantity.name] = quantity
        for handler in self.handler_specs:
            for item in _handler_contexts(handler):
                for quantity in _context_declarations(item):
                    if isinstance(quantity, WorldQuantity):
                        quantities[quantity.name] = quantity
        return quantities

    @cached_property
    def value_variables(self) -> dict[str, ValueVariable]:
        values: dict[str, ValueVariable] = {}
        for motion in self.motion_specs:
            for kind in ("PreContextDecl", "SpecContextDecl", "PostContextDecl"):
                for value in self.value_declarations(motion, kind):
                    values[value.name] = value
        return values

    @cached_property
    def implicit_world_entities(self) -> dict[str, Node]:
        entities: dict[str, Node] = {}
        for quantity in self.world_quantities.values():
            if not isinstance(quantity.props, GeometricProps):
                continue
            if quantity.type == "VelocityTwist":
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
            elif quantity.type == "Pose":
                for key in ("of", "wrt", "as-seen-by"):
                    target = _geometric_property(quantity.props, key)
                    if target:
                        entities.setdefault(target, GEOM_ENT.Frame)
            elif quantity.type == "Wrench":
                point_id = _geometric_property(quantity.props, "ref-point")
                if point_id:
                    entities.setdefault(point_id, GEOM_ENT.Point)
                frame_id = _geometric_property(quantity.props, "as-seen-by")
                if frame_id:
                    entities.setdefault(frame_id, GEOM_ENT.Frame)
        return entities

    @cached_property
    def rotation_ids(self) -> dict[str, str]:
        rotation_ids: dict[str, str] = {}
        for motion_spec in self.motion_specs:
            for constraint in self.all_constraints(motion_spec):
                property_name = _view_property_name(constraint)
                axis = _view_axis_name(constraint)
                quantity = self.world_quantity(motion_spec, _view_quantity_name(constraint))
                if (
                    quantity
                    and quantity.type == "Pose"
                    and property_name == "rotation"
                    and axis is None
                ):
                    rotation_ids[motion_spec.name] = f"rotation-{motion_spec.name}"
        return rotation_ids

    @cached_property
    def angle_from_pose_ops(self) -> list[PoseAngleOpData]:
        ops: list[PoseAngleOpData] = []
        seen: set[str] = set()
        for motion_spec in self.motion_specs:
            for constraint in self.all_constraints(motion_spec):
                property_name = _view_property_name(constraint)
                axis = _view_axis_name(constraint)
                quantity = self.world_quantity(motion_spec, _view_quantity_name(constraint))
                if (
                    quantity is None
                    or quantity.type != "Pose"
                    or property_name != "rotation"
                    or axis is None
                ):
                    continue
                scalar_id = _scalar_id(quantity, "rotation", axis)
                if scalar_id is None or scalar_id in seen:
                    continue
                seen.add(scalar_id)
                ops.append(
                    PoseAngleOpData(
                        op_id=f"compute-{scalar_id}",
                        pose_name=quantity.name,
                        angle_id=scalar_id,
                        axis=axis,
                    )
                )
        return ops

    @cached_property
    def scalar_views(self) -> dict[tuple[str, str, str], ScalarView]:
        views: dict[tuple[str, str, str], ScalarView] = {}
        for motion_spec in self.motion_specs:
            for constraint in self.all_constraints(motion_spec):
                quantity_name = _view_quantity_name(constraint)
                property_name = _view_property_name(constraint)
                axis = _view_axis_name(constraint)
                quantity = self.world_quantity(motion_spec, quantity_name)
                if quantity is None or axis is None:
                    continue
                prop = _property_spec(quantity.type, property_name)
                scalar_id = _scalar_id(quantity, property_name, axis)
                scalar_type = _view_scalar_type(quantity, property_name, axis)
                if prop is None or prop.view_type is None or scalar_id is None or scalar_type is None:
                    continue
                if quantity.type == "Pose" and property_name == "rotation":
                    continue
                key = (quantity_name, property_name, axis)
                views.setdefault(
                    key,
                    ScalarView(
                        parent=motion_spec,
                        name=scalar_id,
                        quantity_name=quantity_name,
                        prop=property_name,
                        axis=axis,
                        scalar_type=scalar_type,
                        view_type=prop.view_type,
                        subspace=prop.view_subspace,
                    ),
                )
        return views

    @cached_property
    def shared_while_signatures(self) -> set[ConstraintSignature]:
        usage: dict[ConstraintSignature, set[str]] = {}
        for motion in self.motion_specs:
            for constraint in motion.while_.constraints:
                if not isinstance(constraint.expr, EqualityConstraint):
                    continue
                usage.setdefault(_while_signature(constraint), set()).add(motion.name)
        return {signature for signature, motions in usage.items() if len(motions) > 1}

    @cached_property
    def constraints(self) -> list[ConstraintData]:
        derived_constraints: list[ConstraintData] = []
        for motion_spec in self.motion_specs:
            motion_name = motion_spec.name
            while_names = {constraint.name for constraint in motion_spec.while_.constraints}

            for constraint in self.all_constraints(motion_spec):
                quantity_name = _view_quantity_name(constraint)
                property_name = _view_property_name(constraint)
                axis = _view_axis_name(constraint)
                quantity = self.world_quantity(motion_spec, quantity_name)
                if quantity is None:
                    raise ValueError(
                        f"Constraint '{constraint.name}' references world quantity "
                        f"'{quantity_name}' that is not defined in the motion context."
                    )
                property_spec = _property_spec(quantity.type, property_name)
                scalar_type_name = (
                    _view_scalar_type(quantity, property_name, axis)
                    if quantity is not None
                    else None
                ) or (property_spec.scalar_type if property_spec else property_name)

                quantity_node_id = _scalar_id(quantity, property_name, axis) if quantity else None
                quantity_node_id = quantity_node_id or quantity_name

                expr = constraint.expr
                signature = None
                error_signal_id = None
                acceleration_energy = None

                if isinstance(expr, EqualityConstraint):
                    kind = "EqualityConstraint"
                    reference_var = _context_value_name(expr.reference)
                    threshold_var = lower_var = upper_var = None
                    if constraint.name in while_names and quantity is not None:
                        signature = _while_signature(constraint)
                        shared = signature in self.shared_while_signatures
                        error_signal_id = _while_error_id(
                            motion_name,
                            quantity,
                            property_name,
                            axis,
                            shared,
                        )
                        acceleration_energy = _acceleration_energy_id(
                            motion_name,
                            quantity,
                            property_name,
                            axis,
                            shared,
                        )
                elif isinstance(expr, GreaterThanConstraint):
                    kind = "GreaterThanConstraint"
                    reference_var = None
                    threshold_var = _context_value_name(expr.threshold)
                    lower_var = upper_var = None
                elif isinstance(expr, LessThanConstraint):
                    kind = "LessThanConstraint"
                    reference_var = None
                    threshold_var = _context_value_name(expr.threshold)
                    lower_var = upper_var = None
                else:
                    assert isinstance(expr, BilateralConstraint)
                    kind = "BilateralConstraint"
                    reference_var = threshold_var = None
                    lower_var = _context_value_name(expr.lower)
                    upper_var = _context_value_name(expr.upper)

                if constraint.name in while_names and quantity is not None and error_signal_id is None:
                    error_signal_id = f"{quantity_node_id}-err"

                derived_constraints.append(
                    ConstraintData(
                        motion_name=motion_name,
                        constraint=constraint,
                        quantity=quantity,
                        kind=kind,
                        scalar_type=scalar_type_name,
                        quantity_node_id=quantity_node_id,
                        reference_var=reference_var,
                        threshold_var=threshold_var,
                        lower_var=lower_var,
                        upper_var=upper_var,
                        signature=signature,
                        error_signal_id=error_signal_id,
                        acceleration_energy_id=acceleration_energy,
                    )
                )
        return derived_constraints

    @cached_property
    def constraint_map(self) -> dict[ConstraintKey, ConstraintData]:
        return {(constraint.motion_name, constraint.constraint.name): constraint for constraint in self.constraints}

    @cached_property
    def controlled_constraints(self) -> list[ConstraintData]:
        while_names_by_motion = {
            motion.name: {constraint.name for constraint in motion.while_.constraints}
            for motion in self.motion_specs
        }
        return [
            constraint
            for constraint in self.constraints
            if constraint.constraint.name in while_names_by_motion.get(constraint.motion_name, set())
        ]

    @cached_property
    def while_constraints(self) -> list[ConstraintData]:
        return [constraint for constraint in self.constraints if constraint.signature is not None]

    @cached_property
    def while_error_signals(self) -> dict[str, ErrorSignal]:
        return {
            constraint.error_signal_id: ErrorSignal(
                parent=constraint.constraint.parent,
                name=constraint.error_signal_id,
                scalar_type=constraint.scalar_type,
            )
            for constraint in self.controlled_constraints
            if constraint.error_signal_id is not None
        }

    @cached_property
    def acceleration_energies(self) -> dict[str, AccelerationEnergy]:
        return {
            constraint.acceleration_energy_id: AccelerationEnergy(
                parent=constraint.constraint.parent,
                name=constraint.acceleration_energy_id,
            )
            for constraint in self.while_constraints
            if constraint.acceleration_energy_id is not None
        }

    @cached_property
    def monitor_data(self) -> list[MonitorData]:
        monitors: list[MonitorData] = []
        for handler_spec in self.handler_specs:
            motion_name = _handler_motion_name(handler_spec)
            if not motion_name:
                continue
            motion_spec = self.motion_map.get(motion_name)
            if motion_spec is None:
                continue
            for monitor in _handler_monitors(handler_spec):
                constraint = self.constraint_map.get((motion_spec.name, monitor.constraint.name))
                if constraint is None:
                    continue
                is_edge = bool(monitor.event)
                signal_name = monitor.event or monitor.flag
                error_signal = ErrorSignal(
                    parent=monitor,
                    name=f"{monitor.constraint.name}-err",
                    scalar_type=constraint.scalar_type,
                )
                monitors.append(
                    MonitorData(
                        motion_name=motion_spec.name,
                        monitor=monitor,
                        constraint=constraint,
                        evaluator=Evaluator(
                            parent=monitor,
                            name=f"eval-{monitor.constraint.name}",
                            constraint=constraint.constraint,
                            error_signal=error_signal,
                        ),
                        error_signal=error_signal,
                        signal_kind="event" if is_edge else "flag",
                        signal_id=signal_name,
                    )
                )
        return monitors

    @cached_property
    def monitor_error_signals(self) -> dict[str, ErrorSignal]:
        return {monitor.error_signal.name: monitor.error_signal for monitor in self.monitor_data}

    @cached_property
    def controller_data(self) -> dict[str, ControllerData]:
        controllers: dict[str, ControllerData] = {}
        for handler_spec in self.handler_specs:
            motion_name = _handler_motion_name(handler_spec)
            if not motion_name:
                continue
            motion_spec = self.motion_map.get(motion_name)
            if motion_spec is None:
                continue
            for controller in _handler_controllers(handler_spec):
                key = str(URIRef(controller.uri))
                constraint = self.constraint_map.get((motion_spec.name, controller.params.constraint.name))
                if constraint is None:
                    raise ValueError(
                        f"Controller '{controller.name}' references constraint "
                        f"'{controller.params.constraint}' that is not defined for motion '{motion_spec.name}'."
                    )
                control_signal_id = _control_signal_id(constraint) or f"eacc-{controller.name}"
                controllers[key] = ControllerData(
                    motion_name=motion_spec.name,
                    controller=controller,
                    constraint=constraint,
                    error_signal_id=constraint.error_signal_id if constraint else None,
                    control_signal_id=control_signal_id,
                    output_type=str(getattr(controller.command_type, "value", controller.command_type))
                    if getattr(controller, "command_type", None)
                    else None,
                    apply_at=_node_name(controller.apply_at) if controller.apply_at else None,
                    feed_scope=None,
                    feed_kind=str(getattr(controller.command_type, "value", controller.command_type))
                    if getattr(controller, "command_type", None)
                    else None,
                    params=controller.params,
                )
        return controllers

    @cached_property
    def evaluator_defs(self) -> dict[str, Evaluator]:
        evaluators: dict[str, Evaluator] = {}
        for constraint in self.controlled_constraints:
            if constraint.error_signal_id is not None:
                error_signal = ErrorSignal(
                    parent=constraint.constraint.parent,
                    name=constraint.error_signal_id,
                    scalar_type=constraint.scalar_type,
                )
                evaluator = Evaluator(
                    parent=constraint.constraint.parent,
                    name=f"eval-{constraint.constraint.name}",
                    constraint=constraint.constraint,
                    error_signal=error_signal,
                )
                evaluators[evaluator.name] = evaluator
        for monitor in self.monitor_data:
            evaluators[monitor.evaluator.name] = monitor.evaluator
        return evaluators

    @cached_property
    def handlers(self) -> list[HandlerData]:
        monitors_by_motion: dict[str, list[MonitorData]] = {}
        for monitor in self.monitor_data:
            monitors_by_motion.setdefault(monitor.motion_name, []).append(monitor)

        handlers: list[HandlerData] = []
        for handler_spec in self.handler_specs:
            motion_name = _handler_motion_name(handler_spec)
            if not motion_name:
                continue
            motion_spec = self.motion_map.get(motion_name)
            if motion_spec is None:
                continue
            while_constraints = [
                constraint
                for constraint in self.controlled_constraints
                if constraint.motion_name == motion_spec.name
            ]
            motion_monitors = monitors_by_motion.get(motion_spec.name, [])
            motion = Motion(
                parent=motion_spec,
                name=f"motion-{motion_spec.name}",
                motion_spec_block=motion_spec,
            )
            handlers.append(
                HandlerData(
                    node_id=str(URIRef(handler_spec.uri)),
                    motion_name=motion_spec.name,
                    motion=motion,
                    evaluators=tuple(
                        [self.evaluator_defs[f"eval-{constraint.constraint.name}"] for constraint in while_constraints]
                        + [monitor.evaluator for monitor in motion_monitors]
                    ),
                    controller_ids=tuple(str(URIRef(controller.uri)) for controller in _handler_controllers(handler_spec)),
                    monitor_ids=tuple(str(URIRef(monitor.monitor.uri)) for monitor in motion_monitors),
                )
            )
        return handlers

    @cached_property
    def acceleration_constraints(self) -> dict[str, AccelerationConstraintData]:
        constraints: dict[str, AccelerationConstraintData] = {}
        for constraint in self.while_constraints:
            quantity = constraint.quantity
            property_name = _view_property_name(constraint.constraint)
            axis = _view_axis_name(constraint.constraint)
            prop = _property_spec(quantity.type, property_name) if quantity else None
            if (
                quantity is None
                or prop is None
                or prop.accel_subspace is None
                or constraint.acceleration_energy_id is None
            ):
                continue
            shared = constraint.signature in self.shared_while_signatures if constraint.signature else False
            node_id = _acceleration_constraint_id(
                constraint.motion_name,
                quantity,
                property_name,
                axis,
                shared,
            )
            constraints.setdefault(
                node_id,
                AccelerationConstraintData(
                    node_id=node_id,
                    energy_id=constraint.acceleration_energy_id,
                    subspace=prop.accel_subspace,
                    axis=axis or "",
                ),
            )
        return constraints

    @cached_property
    def base_velocity_solvers(self) -> list[BaseVelocitySolverData]:
        solvers: list[BaseVelocitySolverData] = []
        for handler_spec in self.handler_specs:
            solver_group = _primary_solver(handler_spec)
            if solver_group is None:
                continue
            for solver in getattr(solver_group, "velocity_solvers", []):
                _required_world_quantity(
                    self,
                    solver.velocity,
                    f"referenced by base velocity solver '{solver.name}'",
                )
                solvers.append(
                    BaseVelocitySolverData(
                        node_id=solver.name,
                        configuration=solver.configuration,
                        velocity=solver.velocity,
                    )
                )
        return solvers

    @cached_property
    def base_force_solvers(self) -> list[BaseForceSolverData]:
        solvers: list[BaseForceSolverData] = []
        for handler_spec in self.handler_specs:
            solver_group = _primary_solver(handler_spec)
            if solver_group is None:
                continue
            for solver in getattr(solver_group, "force_solvers", []):
                _required_world_quantity(
                    self,
                    solver.force,
                    f"referenced by base force solver '{solver.name}'",
                )
                solvers.append(
                    BaseForceSolverData(
                        node_id=solver.name,
                        configuration=solver.configuration,
                        force=solver.force,
                    )
                )
        return solvers

    @cached_property
    def distance_derivations(self) -> list[DistanceDerivation]:
        derivations: list[DistanceDerivation] = []
        for constraint in self.controlled_constraints:
            quantity = constraint.quantity
            property_name = _view_property_name(constraint.constraint)
            axis = _view_axis_name(constraint.constraint)
            if (
                quantity is None
                or quantity.type != "Pose"
                or property_name != "distance"
                or axis is not None
            ):
                continue
            observer_frame = _with_respect_to(quantity)
            if observer_frame is None:
                raise NotImplementedError(
                    f"Distance derivation for '{quantity.name}' needs an explicit 'wrt' frame."
                )
            raise NotImplementedError(
                "Distance derivation needs model-backed proximal/distal/observer fields; "
                f"refusing to parse them from the authored name '{quantity.name}'."
            )
        return derivations

    def build(self) -> DatasetOutput:
        for owner in self.namespace_owners:
            self.dataset.bind(owner.ns_prefix, owner.ns.uri)
        self.materialize_authored()
        self.materialize_derived()
        return self.dataset, self.context(
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
        self._add_evaluators()
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
                self.graph.add((node, GEOM_REL["with-respect-to"], self.root_uri(wrt_value, owner=quantity)))
            if reference_point:
                self.graph.add((node, GEOM_REL["reference-point"], self.root_uri(reference_point, owner=quantity)))
            if as_seen_by:
                self.graph.add((node, GEOM_COORD["as-seen-by"], self.root_uri(as_seen_by, owner=quantity)))
            elif quantity.type == "Pose" and wrt_value:
                self.graph.add((node, GEOM_COORD["as-seen-by"], self.root_uri(wrt_value, owner=quantity)))
            if quantity.type in {"VelocityTwist", "Wrench"} and not reference_point:
                warnings.warn(f"'{quantity.name}' has no explicit ref-point; not inferring one from its name.")

    def _add_value_variables(self) -> None:
        for variable in self.value_variables.values():
            node = URIRef(variable.uri)
            qkind = QUDT_QKIND[variable.type]
            _add_types(self.graph, node, QUDT_SCHEMA.Quantity, qkind)
            self.graph.add((node, QUDT_SCHEMA["quantity-kind"], qkind))
            if variable.value is None:
                continue
            self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(variable.value.unit)))
            self.graph.add((node, QUDT_SCHEMA.value, Literal(str(variable.value.value))))

    def _add_constraints(self) -> None:
        for constraint in self.constraints:
            node = URIRef(constraint.constraint.uri)
            type_name = {"PlaneAngle": "Angle", "Length": "Distance"}.get(constraint.scalar_type, constraint.scalar_type)
            _add_types(self.graph, node, CSTR.Constraint, CSTR[f"{type_name}Constraint"])
            self.graph.add((node, CSTR.quantity, self.constraint_quantity_node(constraint)))
            motion_spec = self.motion_map[constraint.motion_name]
            if constraint.kind == "EqualityConstraint":
                reference = self.value_variable(motion_spec, constraint.reference_var or "")
                _add_types(self.graph, node, CSTR.EqualityConstraint)
                ref_node = URIRef(reference.uri) if reference else self.root_uri(constraint.reference_var or "", owner=motion_spec)
                self.graph.add((node, CSTR["reference-value"], ref_node))
            elif constraint.kind == "GreaterThanConstraint":
                threshold = self.value_variable(motion_spec, constraint.threshold_var or "")
                _add_types(self.graph, node, CSTR.UnilateralConstraint, CSTR.GreaterThanConstraint)
                threshold_node = URIRef(threshold.uri) if threshold else self.root_uri(constraint.threshold_var or "", owner=motion_spec)
                self.graph.add((node, CSTR.threshold, threshold_node))
            elif constraint.kind == "LessThanConstraint":
                threshold = self.value_variable(motion_spec, constraint.threshold_var or "")
                _add_types(self.graph, node, CSTR.UnilateralConstraint, CSTR.LessThanConstraint)
                threshold_node = URIRef(threshold.uri) if threshold else self.root_uri(constraint.threshold_var or "", owner=motion_spec)
                self.graph.add((node, CSTR.threshold, threshold_node))
            else:
                lower = self.value_variable(motion_spec, constraint.lower_var or "")
                upper = self.value_variable(motion_spec, constraint.upper_var or "")
                _add_types(self.graph, node, CSTR.BilateralConstraint)
                lower_node = URIRef(lower.uri) if lower else self.root_uri(constraint.lower_var or "", owner=motion_spec)
                upper_node = URIRef(upper.uri) if upper else self.root_uri(constraint.upper_var or "", owner=motion_spec)
                self.graph.add((node, CSTR["lower-threshold"], lower_node))
                self.graph.add((node, CSTR["upper-threshold"], upper_node))

    def _add_motion_specs(self) -> None:
        for motion_spec in self.motion_specs:
            motion_node = self.root_uri(f"motion-{motion_spec.name}", owner=motion_spec)
            _add_types(self.graph, motion_node, MOT.GuardedMotion)
            for constraint in motion_spec.when.constraints:
                self.graph.add((motion_node, MOT.when, URIRef(constraint.uri)))
            for constraint in motion_spec.while_.constraints:
                self.graph.add((motion_node, MOT["while"], URIRef(constraint.uri)))
            for constraint in motion_spec.until.constraints:
                self.graph.add((motion_node, MOT.until, URIRef(constraint.uri)))

    def _add_constraint_handlers(self) -> None:
        for controller_key, controller in self.controller_data.items():
            node = URIRef(controller_key)
            _add_types(self.graph, node, CSTR_HDL.Controller, CSTR_HDL.ProportionalIntegralDerivative)
            if controller.error_signal_id:
                self.graph.add((node, CSTR_HDL["error-signal"], self.root_uri(controller.error_signal_id)))
            self.graph.add((node, CSTR_HDL["control-signal"], self.root_uri(controller.control_signal_id)))
            self.graph.add((node, CSTR_HDL["proportional-gain"], Literal(str(controller.params.kp))))
            self.graph.add((node, CSTR_HDL["integral-gain"], Literal(str(controller.params.ki))))
            self.graph.add((node, CSTR_HDL["derivative-gain"], Literal(str(controller.params.kd))))

        for monitor in self.monitor_data:
            signal_node = self.root_uri(monitor.signal_id)
            _add_types(self.graph, signal_node, CSTR_HDL.Event if monitor.signal_kind == "event" else CSTR_HDL.Flag)
            node = URIRef(monitor.monitor.uri)
            _add_types(self.graph, node, CSTR_HDL.Monitor)
            self.graph.add((node, CSTR_HDL.error, self.node(monitor.error_signal)))
            if monitor.signal_kind == "event":
                _add_types(self.graph, node, CSTR_HDL.EdgeTriggeredMonitor)
                self.graph.add((node, CSTR_HDL.event, signal_node))
            else:
                _add_types(self.graph, node, CSTR_HDL.LevelTriggeredMonitor)
                self.graph.add((node, CSTR_HDL.flag, signal_node))

        for handler in self.handlers:
            node = URIRef(handler.node_id)
            _add_types(self.graph, node, CSTR_HDL.ConstraintHandler)
            self.graph.add((node, CSTR_HDL.motion, self.node(handler.motion)))
            for evaluator in handler.evaluators:
                self.graph.add((node, CSTR_HDL.evaluators, self.root_uri(evaluator.name, owner=evaluator.parent)))
            for controller_id in handler.controller_ids:
                self.graph.add((node, CSTR_HDL.controllers, URIRef(controller_id)))
            for monitor_id in handler.monitor_ids:
                self.graph.add((node, CSTR_HDL.monitors, URIRef(monitor_id)))

    def _add_scalar_views(self) -> None:
        existing_scalar_views = {view.name for view in self.scalar_views.values()}
        for view in self.scalar_views.values():
            _add_quantity(self.graph, self.node(view), view.scalar_type)
            node = self.root_uri(f"view-{view.name}", owner=view.parent)
            _add_types(self.graph, node, MAP.View)
            if view.view_type:
                _add_types(self.graph, node, view.view_type)
            self.graph.add((node, MAP.superobject, self.root_uri(view.quantity_name, owner=view.parent)))
            self.graph.add((node, MAP.subobject, self.node(view)))
            if view.subspace:
                self.graph.add((node, MAP.subspace, MAP[view.subspace]))
            self.graph.add((node, MAP.axis, MAP[view.axis]))
        for constraint in self.constraints:
            if constraint.scalar_type == "PlaneAngle":
                _add_quantity(self.graph, self.constraint_quantity_node(constraint), "PlaneAngle")
        for rotation_id in self.rotation_ids.values():
            _add_quantity(self.graph, self.root_uri(rotation_id), "PlaneAngle")
        for controller in self.controller_data.values():
            if _controller_command_kind(controller) == "Force":
                _add_quantity(self.graph, self.root_uri(controller.control_signal_id), "Force")
        for handler_spec in self.handler_specs:
            solver = _primary_solver(handler_spec)
            if solver is None or not getattr(solver, "algorithm", ""):
                continue
            for binding in _infer_cartesian_force_bindings(self, handler_spec):
                quantity = self.world_quantities.get(binding.force_name)
                if quantity is None or quantity.type != "Wrench":
                    continue
                scalar_id = _scalar_id(quantity, "force", "z")
                if scalar_id is None or scalar_id in existing_scalar_views:
                    continue
                node = self.root_uri(f"view-{scalar_id}", owner=quantity)
                _add_types(self.graph, node, MAP.View, MAP.WrenchCoordinateView)
                self.graph.add((node, MAP.superobject, self.node(quantity)))
                self.graph.add((node, MAP.subobject, self.root_uri(scalar_id, owner=quantity)))
                self.graph.add((node, MAP.subspace, MAP.force))
                self.graph.add((node, MAP.axis, MAP.z))

    def _add_error_signals_and_energies(self) -> None:
        for signal in self.while_error_signals.values():
            _add_quantity(self.graph, self.node(signal), signal.scalar_type)
        for signal in self.monitor_error_signals.values():
            _add_quantity(self.graph, self.node(signal), signal.scalar_type)
        for energy in self.acceleration_energies.values():
            node = self.node(energy)
            _add_types(self.graph, node, QUDT_SCHEMA.Quantity, URIRef("AccelerationEnergy"))
            self.graph.add((node, QUDT_SCHEMA["quantity-kind"], URIRef("AccelerationEnergy")))
            self.graph.add((node, QUDT_SCHEMA.unit, URIRef("N-M2-PER-SEC2")))

    def _add_evaluators(self) -> None:
        for evaluator_id, evaluator in self.evaluator_defs.items():
            node = self.root_uri(evaluator_id, owner=evaluator.parent)
            _add_types(self.graph, node, CSTR_HDL.ConstraintEvaluator, CSTR_HDL.ErrorEvaluator)
            self.graph.add((node, CSTR_HDL.constraint, URIRef(evaluator.constraint.uri)))
            self.graph.add((node, CSTR_HDL.error, self.node(evaluator.error_signal)))

    def _add_solver_entities(self) -> None:
        for solver in self.base_velocity_solvers:
            node = self.root_uri(solver.node_id)
            _add_types(self.graph, node, SLV.VelocityCompositionSolver)
            self.graph.add((node, SLV.configuration, Literal(solver.configuration)))
            self.graph.add((node, SLV.velocity, self.root_uri(solver.velocity)))

        for solver in self.base_force_solvers:
            node = self.root_uri(solver.node_id)
            _add_types(self.graph, node, SLV.ForceDistributionSolver)
            self.graph.add((node, SLV.configuration, Literal(solver.configuration)))
            self.graph.add((node, SLV.force, self.root_uri(solver.force)))

        for handler_spec in self.handler_specs:
            solver = _primary_solver(handler_spec)
            motion_name = _handler_motion_name(handler_spec)
            if not motion_name or solver is None or not getattr(solver, "algorithm", ""):
                continue
            motion_spec = self.motion_map.get(motion_name)
            if motion_spec is None:
                continue
            attached_link = _infer_attached_link(self, handler_spec, solver)
            spec_constraints: list[str] = []
            for constraint in self.while_constraints:
                if constraint.motion_name != motion_name:
                    continue
                quantity = constraint.quantity
                property_name = _view_property_name(constraint.constraint)
                axis = _view_axis_name(constraint.constraint)
                property_spec = _property_spec(quantity.type, property_name) if quantity else None
                if quantity is None or property_spec is None or property_spec.accel_subspace is None:
                    continue
                if constraint.acceleration_energy_id is None:
                    continue
                shared = constraint.signature in self.shared_while_signatures if constraint.signature else False
                node_id = _acceleration_constraint_id(motion_name, quantity, property_name, axis, shared)
                if node_id not in spec_constraints:
                    spec_constraints.append(node_id)
                data = self.acceleration_constraints[node_id]
                node = self.root_uri(data.node_id, owner=motion_spec)
                _add_types(self.graph, node, SLV.AccelerationConstraint, SLV.AxisAligned)
                self.graph.add((node, SLV.subspace, SLV[data.subspace]))
                self.graph.add((node, SLV.axis, SLV[data.axis]))
                self.graph.add((node, SLV["acceleration-energy"], self.root_uri(data.energy_id, owner=motion_spec)))

            spec_acc_node = self.root_uri(f"spec-acc-{attached_link}-{motion_name}", owner=motion_spec) if attached_link else self.root_uri(f"spec-acc-ee-{motion_name}", owner=motion_spec)
            _add_types(self.graph, spec_acc_node, SLV.AccelerationConstraintSpecification)
            for node_id in spec_constraints:
                self.graph.add((spec_acc_node, SLV.constraints, self.root_uri(node_id, owner=motion_spec)))
            attached_to = self.root_uri(attached_link, owner=handler_spec) if attached_link else self.root_uri("link-ee", owner=handler_spec)
            self.graph.add((spec_acc_node, SLV["attached-to"], attached_to))

            for binding in _infer_cartesian_force_bindings(self, handler_spec):
                force_name = binding.force_name
                _required_world_quantity(self, force_name, f"referenced by cartesian-force solver output '{binding.spec_name}'")
                spec_node = self.root_uri(binding.spec_name, owner=handler_spec)
                _add_types(self.graph, spec_node, SLV.CartesianForceSpecification)
                self.graph.add((spec_node, SLV.force, self.root_uri(force_name, owner=handler_spec)))
                if binding.attached_to is not None:
                    self.graph.add((spec_node, SLV["attached-to"], self.root_uri(binding.attached_to, owner=handler_spec)))

            driver_node = self.root_uri(f"drv-{_driver_suffix(handler_spec)}", owner=handler_spec)
            _add_types(self.graph, driver_node, SLV.MotionDrivers)
            self.graph.add((driver_node, SLV["acceleration-constraint"], spec_acc_node))
            for binding in _infer_cartesian_force_bindings(self, handler_spec):
                self.graph.add((driver_node, SLV["cartesian-force"], self.root_uri(binding.spec_name, owner=handler_spec)))
            for joint_force in getattr(solver, "joint_force", []):
                self.graph.add((driver_node, SLV["joint-force"], self.root_uri(joint_force, owner=handler_spec)))

            solver_node = self.root_uri(f"slv-{_driver_suffix(handler_spec)}", owner=handler_spec)
            solver_algorithm = SLV.AccelerationConstrainedHybridDynamicsAlgorithm if solver.algorithm == "Vereshchagin" else SLV.NewtonEulerAlgorithm if solver.algorithm == "NewtonEuler" else SLV[solver.algorithm]
            _add_types(self.graph, solver_node, SLV.SolverWithInputAndOutput)
            self.graph.add((solver_node, SLV.solver, solver_algorithm))
            gravity_name = _node_name(solver.gravity)
            _required_world_quantity(self, gravity_name, f"referenced by solver '{handler_spec.name}' gravity")
            self.graph.add((solver_node, SLV["kinematic-chain"], self.root_uri(_node_name(solver.robot), owner=handler_spec)))
            self.graph.add((solver_node, SLV.root, self.root_uri(_node_name(solver.root), owner=handler_spec)))
            self.graph.add((solver_node, SLV.gravity, self.root_uri(gravity_name, owner=handler_spec)))
            self.graph.add((solver_node, SLV["motion-drivers"], driver_node))

    def _add_map_operations(self) -> None:
        for motion_name, rotation_id in self.rotation_ids.items():
            motion_spec = self.motion_map[motion_name]
            pose_quantity = next(
                (
                    _view_quantity_name(constraint)
                    for constraint in self.all_constraints(motion_spec)
                    if _view_property_name(constraint) == "rotation" and _view_axis_name(constraint) is None
                ),
                None,
            )
            if pose_quantity is None:
                continue
            op_node = self.root_uri(f"compute-{rotation_id}", owner=motion_spec)
            _add_types(self.graph, op_node, MAP.ComputeRotationFromPose)
            self.graph.add((op_node, MAP.pose, self.root_uri(pose_quantity, owner=motion_spec)))
            self.graph.add((op_node, MAP.rotation, self.root_uri(rotation_id, owner=motion_spec)))
        for op_data in self.angle_from_pose_ops:
            op_node = self.root_uri(op_data.op_id)
            _add_types(self.graph, op_node, GEOM_OP.PoseToAngleAroundAxis)
            self.graph.add((op_node, GEOM_OP.pose, self.root_uri(op_data.pose_name)))
            self.graph.add((op_node, GEOM_OP.angle, self.root_uri(op_data.angle_id)))
            self.graph.add((op_node, GEOM_OP.axis, MAP[op_data.axis]))

    def _add_transform_operations(self) -> None:
        for derivation in self.distance_derivations:
            _add_quantity(self.graph, self.root_uri(derivation.distance_id), "Length")
            direction = self.root_uri(derivation.direction_id)
            _add_types(self.graph, direction, GEOM_REL.Direction, GEOM_COORD.DirectionCoordinate, GEOM_COORD.VectorXYZ)
            self.graph.add((direction, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Direction))
            self.graph.add((direction, QUDT_SCHEMA.unit, QUDT_UNIT.UNITLESS))
            self.graph.add((direction, GEOM_COORD["as-seen-by"], self.root_uri(derivation.observer_frame)))
            position = self.root_uri(derivation.position_id)
            _add_types(self.graph, position, GEOM_REL.Position, GEOM_COORD.PositionCoordinate, GEOM_COORD.VectorXYZ)
            self.graph.add((position, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Length))
            self.graph.add((position, QUDT_SCHEMA.unit, QUDT_UNIT.M))
            self.graph.add((position, GEOM_REL.of, self.root_uri(derivation.observer_point)))
            self.graph.add((position, GEOM_REL["with-respect-to"], self.root_uri(derivation.observer_point)))
            self.graph.add((position, GEOM_COORD["as-seen-by"], self.root_uri(derivation.observer_frame)))
            self.graph.add((position, GEOM_COORD.x, Literal(0)))
            self.graph.add((position, GEOM_COORD.y, Literal(0)))
            self.graph.add((position, GEOM_COORD.z, Literal(0)))
            wrench = self.root_uri(derivation.local_wrench_id)
            _add_types(self.graph, wrench, RBDYN_ENT.Wrench, RBDYN_COORD.WrenchCoordinate, GEOM_COORD.VectorXYZ)
            self.graph.add((wrench, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Torque))
            self.graph.add((wrench, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Force))
            self.graph.add((wrench, QUDT_SCHEMA.unit, QUDT_UNIT["N-M"]))
            self.graph.add((wrench, QUDT_SCHEMA.unit, QUDT_UNIT.N))
            self.graph.add((wrench, GEOM_REL["reference-point"], self.root_uri(derivation.observer_point)))
            self.graph.add((wrench, RBDYN_COORD["as-seen-by"], self.root_uri(derivation.observer_frame)))
            dist_node = self.root_uri(derivation.pose_to_dist_op_id)
            _add_types(self.graph, dist_node, GEOM_OP.PoseToLinearDistance)
            self.graph.add((dist_node, GEOM_OP.pose, self.root_uri(derivation.pose_name)))
            self.graph.add((dist_node, GEOM_OP.distance, self.root_uri(derivation.distance_id)))
            dir_node = self.root_uri(derivation.pose_to_dir_op_id)
            _add_types(self.graph, dir_node, GEOM_OP.PoseToDirection)
            self.graph.add((dir_node, GEOM_OP.pose, self.root_uri(derivation.pose_name)))
            self.graph.add((dir_node, GEOM_OP.direction, self.root_uri(derivation.direction_id)))
            wrench_op = self.root_uri(derivation.wrench_op_id)
            _add_types(self.graph, wrench_op, RBDYN_OP.WrenchFromPositionDirectionAndMagnitude)
            self.graph.add((wrench_op, RBDYN_OP.magnitude, self.root_uri(derivation.force_id)))
            self.graph.add((wrench_op, RBDYN_OP.position, self.root_uri(derivation.position_id)))
            self.graph.add((wrench_op, RBDYN_OP.direction, self.root_uri(derivation.direction_id)))
            self.graph.add((wrench_op, RBDYN_OP.wrench, wrench))

        transforms_by_force, add_ops_by_force = _derived_transformed_wrenches(self)
        for solver_force, transforms in transforms_by_force.items():
            target_quantity = _required_world_quantity(self, solver_force, "base force target")
            target_frame = _reference_frame(target_quantity)
            if target_frame is None:
                continue
            target_point = _geometric_property(target_quantity.props, "ref-point") if isinstance(target_quantity.props, GeometricProps) else None
            for derivation, pose_name, transformed_wrench in transforms:
                op_node = self.root_uri(f"tf-{derivation.local_wrench_id}-{target_frame}")
                _add_types(self.graph, op_node, RBDYN_OP.TransformWrenchToProximal)
                self.graph.add((op_node, RBDYN_OP.pose, self.root_uri(pose_name)))
                self.graph.add((op_node, RBDYN_OP["from"], self.root_uri(derivation.local_wrench_id)))
                self.graph.add((op_node, RBDYN_OP.to, self.root_uri(transformed_wrench)))
                node = self.root_uri(transformed_wrench)
                _add_types(self.graph, node, RBDYN_ENT.Wrench, RBDYN_COORD.WrenchCoordinate, GEOM_COORD.VectorXYZ)
                self.graph.add((node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Torque))
                self.graph.add((node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Force))
                self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M"]))
                self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.N))
                if target_point:
                    self.graph.add((node, GEOM_REL["reference-point"], self.root_uri(target_point, owner=target_quantity)))
                self.graph.add((node, RBDYN_COORD["as-seen-by"], self.root_uri(target_frame, owner=target_quantity)))
            add_op = add_ops_by_force.get(solver_force)
            if add_op and len(transforms) >= 2:
                op_node = self.root_uri(add_op)
                _add_types(self.graph, op_node, RBDYN_OP.AddWrench)
                self.graph.add((op_node, RBDYN_OP.in1, self.root_uri(transforms[0][2])))
                self.graph.add((op_node, RBDYN_OP.in2, self.root_uri(transforms[1][2])))
                self.graph.add((op_node, RBDYN_OP.out, self.root_uri(solver_force)))


def _find_pose(
    builder: "MotionSpecDatasetBuilder", *, of_frame: str, wrt_frame: str
) -> WorldQuantityLike | None:
    for quantity in builder.world_quantities.values():
        if quantity.type != "Pose":
            continue
        if _of_frame(quantity) == of_frame and _with_respect_to(quantity) == wrt_frame:
            return quantity
    return None


def _derived_transformed_wrenches(
    builder: "MotionSpecDatasetBuilder",
) -> tuple[dict[str, list[tuple[DistanceDerivation, str, str]]], dict[str, str]]:
    transforms_by_force: dict[str, list[tuple[DistanceDerivation, str, str]]] = {}
    add_ops_by_force: dict[str, str] = {}

    for solver in builder.base_force_solvers:
        target_quantity = _required_world_quantity(
            builder,
            solver.force,
            f"referenced by base force solver '{solver.node_id}'",
        )
        target_frame = _reference_frame(target_quantity)
        if target_frame is None:
            raise ValueError(
                f"Base force solver '{solver.node_id}' references '{solver.force}', which is missing "
                "'as-seen-by' in context."
            )

        transformed: list[tuple[DistanceDerivation, str, str]] = []
        for derivation in builder.distance_derivations:
            pose = _find_pose(builder, of_frame=derivation.observer_frame, wrt_frame=target_frame)
            if pose is None:
                warnings.warn(
                    f"Skipping transform for '{derivation.local_wrench_id}' because no pose was found "
                    f"from '{derivation.observer_frame}' to '{target_frame}'."
                )
                continue
            transformed_wrench = f"{derivation.local_wrench_id}.as-seen-by.{target_frame}"
            transformed.append((derivation, pose.name, transformed_wrench))

        transforms_by_force[solver.force] = transformed
        if len(transformed) > 1:
            add_ops_by_force[solver.force] = f"add-{solver.force}"

    return transforms_by_force, add_ops_by_force

