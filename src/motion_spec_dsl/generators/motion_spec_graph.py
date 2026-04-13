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
from typing import Any, TypeAlias, cast

from rdflib.graph import Graph
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
    ConstraintHandlerBlock,
    ConstraintSpecification,
    CtrlWorldQuantity,
    ControllerParams,
    EqualityConstraint,
    ForceSolverEntry,
    GeoPropPair,
    GeometricProps,
    GreaterThanConstraint,
    LessThanConstraint,
    Model,
    MonitorEntry,
    MotionSpecBlock,
    PostContextDecl,
    PostLookup,
    PreContextDecl,
    PreLookup,
    SpecContextDecl,
    SpecLookup,
    ValueVariable,
    WorldLookup,
    WorldContextDecl,
    WorldQuantity,
)

Node: TypeAlias = Any
NamespaceBinding: TypeAlias = tuple[str, Any]
ContextLike: TypeAlias = dict[str, str] | list[str | dict[str, str]]
GraphOutput: TypeAlias = tuple[str, Graph, ContextLike]
ConstraintSignature: TypeAlias = tuple[str, str, str | None, str]
ConstraintKey: TypeAlias = tuple[str, str]
WorldQuantityLike: TypeAlias = WorldQuantity | CtrlWorldQuantity

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
class ScalarViewData:
    quantity_name: str
    property_name: str
    axis: str
    scalar_id: str
    scalar_type: str
    view_type: Any
    view_subspace: str | None


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
class ErrorSignalData:
    node_id: str
    scalar_type: str


@dataclass(frozen=True)
class MonitorData:
    motion_name: str
    monitor: MonitorEntry
    constraint: ConstraintData
    evaluator_id: str
    error_signal_id: str
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
    motion_id: str
    evaluator_ids: tuple[str, ...]
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

SCALAR_QKIND = {
    "AngularVelocity": QUDT_QKIND.AngularVelocity,
    "LinearVelocity": QUDT_QKIND.LinearVelocity,
    "Torque": QUDT_QKIND.Torque,
    "Force": QUDT_QKIND.Force,
    "Position": QUDT_QKIND.Position,
    "PlaneAngle": QUDT_QKIND.PlaneAngle,
    "Length": QUDT_QKIND.Length,
}

DSL_SCALAR_QKIND = {
    "AngularVelocity": QUDT_QKIND.AngularVelocity,
    "LinearVelocity": QUDT_QKIND.LinearVelocity,
    "Force": QUDT_QKIND.Force,
    "Torque": QUDT_QKIND.Torque,
    "LinearDistance": QUDT_QKIND.Length,
    "Angle": QUDT_QKIND.PlaneAngle,
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

MISC_ENTITY_TYPE = {
    "Frame": GEOM_ENT.Frame,
    "SimplicialComplex": GEOM_ENT.SimplicialComplex,
    "KinematicChain": GEOM_ENT["KinematicChain"],
    "Point": GEOM_ENT.Point,
    "UniformGravitationalField": GEOM_ENT["UniformGravitationalField"],
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


def _graph(*bindings: NamespaceBinding) -> Any:
    graph = cast(Any, Graph())
    for prefix, namespace in bindings:
        graph.bind(prefix, namespace)
    return graph


def _add_types(graph: Any, node: Node, *rdf_types: Node) -> None:
    for rdf_type in rdf_types:
        graph.add((node, RDF.type, rdf_type))


def _add_quantity(graph: Any, node: Node, scalar_type: str) -> None:
    qkind = _scalar_qkind(scalar_type)
    _add_types(graph, node, QUDT_SCHEMA.Quantity, qkind)
    graph.add((node, QUDT_SCHEMA["quantity-kind"], qkind))
    graph.add((node, QUDT_SCHEMA.unit, _scalar_unit(scalar_type)))


def _world_spec(world_type: str) -> WorldSpec | None:
    return WORLD_SPECS.get(world_type)


def _property_spec(world_type: str, property_name: str) -> PropertySpec | None:
    spec = _world_spec(world_type)
    return spec.properties.get(property_name) if spec else None


def _scalar_unit(scalar_type: str) -> Node:
    return SCALAR_UNIT.get(scalar_type, QUDT_UNIT.UNITLESS)


def _scalar_qkind(scalar_type: str) -> Node:
    return SCALAR_QKIND.get(scalar_type, URIRef(scalar_type))


def _dsl_scalar_qkind(scalar_type: str) -> Node:
    return DSL_SCALAR_QKIND.get(scalar_type, URIRef(scalar_type))


def _dsl_unit(unit_name: str) -> Node:
    try:
        return DSL_UNIT[unit_name]
    except KeyError as exc:
        supported = ", ".join(sorted(DSL_UNIT))
        raise ValueError(
            f"Unsupported DSL unit '{unit_name}'. Use compact DSL units only. "
            f"Supported units: {supported}."
        ) from exc


def _motion_suffix(name: str) -> str:
    return name.split("_")[-1]


def _entity_abbrev(name: str) -> str:
    parts = name.split("-")
    return parts[1] if len(parts) > 1 else name


def _attached_link_from_wrench_name(name: str) -> str | None:
    parts = name.split("-")
    if len(parts) < 3 or parts[0] != "wrench":
        return None
    return f"link-{parts[1]}-{parts[2]}"


def _infer_attached_link(index: "ModelIndex", handler_spec: ConstraintHandlerBlock, solver: Any) -> str | None:
    for binding in _infer_cartesian_force_bindings(index, handler_spec):
        if binding.attached_to is not None:
            return binding.attached_to

    for force_name in getattr(solver, "cartesian_force", []):
        attached_force_link = _attached_link_from_wrench_name(force_name)
        if attached_force_link is not None:
            return attached_force_link

    motion_name = handler_spec.spec.motion
    motion_spec = index.motion_map.get(motion_name) if motion_name else None
    if motion_spec is not None:
        for quantity in index.world_declarations(motion_spec):
            if quantity.type == "VelocityTwist" and isinstance(quantity.props, GeometricProps):
                attached_link = _geometric_property(quantity.props, "of")
                if attached_link is not None:
                    return attached_link

    for item in handler_spec.spec.context.items:
        for quantity in item.decl.declaration:
            if quantity.type == "VelocityTwist" and isinstance(quantity.props, GeometricProps):
                attached_link = _geometric_property(quantity.props, "of")
                if attached_link is not None:
                    return attached_link

    return None


def _handler_world_quantities(handler_spec: ConstraintHandlerBlock) -> list[WorldQuantityLike]:
    quantities: list[WorldQuantityLike] = []
    for item in handler_spec.spec.context.items:
        quantities.extend(item.decl.declaration)
    return quantities


def _wrench_name_for_scalar_signal(
    index: "ModelIndex", handler_spec: ConstraintHandlerBlock, signal_id: str
) -> str | None:
    candidates: list[WorldQuantityLike] = []
    for quantity in _handler_world_quantities(handler_spec):
        if quantity.type == "Wrench":
            candidates.append(quantity)
    for quantity in index.world_quantities.values():
        if quantity.type == "Wrench" and quantity.name not in {q.name for q in candidates}:
            candidates.append(quantity)

    matches = [q for q in candidates if _scalar_id(q, "force", "z") == signal_id]
    if not matches:
        return None

    platform_match = next(
        (
            q
            for q in matches
            if isinstance(q.props, GeometricProps) and _geometric_property(q.props, "as-seen-by") == "frame-platform"
        ),
        None,
    )
    return (platform_match or matches[0]).name


def _infer_cartesian_force_bindings(
    index: "ModelIndex", handler_spec: ConstraintHandlerBlock
) -> list[CartesianForceBinding]:
    motion_name = handler_spec.spec.motion
    if not motion_name:
        return []

    spec_names: list[str] = []
    candidates: list[tuple[str | None, str, str | None, bool]] = []
    seen_force_names: set[str] = set()
    solver = handler_spec.spec.solver
    for force_name in getattr(solver, "cartesian_force", []):
        spec_names.append(f"spec-{force_name.removeprefix('wrench-')}")
        candidates.append(
            (
                f"spec-{force_name.removeprefix('wrench-')}",
                force_name,
                _attached_link_from_wrench_name(force_name),
                True,
            )
        )
        seen_force_names.add(force_name)

    motion_controllers = [
        controller
        for controller in index.controller_data.values()
        if controller.motion_name == motion_name and controller.control_signal_id.startswith("frc-")
    ]
    for controller in motion_controllers:
        explicit_cartesian_force = controller.feed_scope == "cartesian" and controller.feed_kind == "force"
        force_name = None
        for derivation in index.distance_derivations:
            if derivation.force_id == controller.control_signal_id and derivation.group in motion_name:
                force_name = derivation.local_wrench_id
                break
        if force_name is None:
            force_name = _wrench_name_for_scalar_signal(index, handler_spec, controller.control_signal_id)
        if force_name is None or force_name in seen_force_names:
            continue

        spec_hint = None
        if explicit_cartesian_force and controller.apply_at:
            spec_hint = f"spec-frc-{controller.apply_at.removeprefix('link-')}"
        elif controller.control_signal_id:
            spec_hint = f"spec-{controller.control_signal_id}"

        attached_to = controller.apply_at or _attached_link_from_wrench_name(force_name)
        candidates.append((spec_hint, force_name, attached_to, explicit_cartesian_force))
        seen_force_names.add(force_name)
        if spec_hint is not None and spec_hint not in spec_names:
            spec_names.append(spec_hint)

    for constraint in index.constraints:
        if constraint.motion_name != motion_name or constraint.quantity is None or constraint.quantity.type != "Wrench":
            continue
        if not constraint.quantity_node_id.startswith("frc-"):
            continue
        force_name = _wrench_name_for_scalar_signal(index, handler_spec, constraint.quantity_node_id)
        if force_name is None or force_name in seen_force_names:
            continue
        candidates.append(
            (f"spec-{constraint.quantity_node_id}", force_name, _attached_link_from_wrench_name(force_name), False)
        )
        seen_force_names.add(force_name)
        spec_hint = f"spec-{constraint.quantity_node_id}"
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


def _driver_suffix(handler_spec: ConstraintHandlerBlock) -> str:
    return _motion_suffix(handler_spec.spec.motion) if handler_spec.spec.motion else handler_spec.name


def _view_scalar_type(quantity: WorldQuantity, property_name: str, axis: str | None) -> str | None:
    if quantity.type == "Pose" and property_name == "distance":
        return "Position" if axis is not None else "Length"
    prop = _property_spec(quantity.type, property_name)
    return prop.scalar_type if prop else None


def _axis_label(axis: str) -> str:
    return {"x": "anteroposterior", "y": "lateral", "z": "vertical"}.get(axis, axis)


def _scalar_id(quantity: WorldQuantity, property_name: str, axis: str | None) -> str | None:
    parts = quantity.name.split("-")
    if quantity.type == "VelocityTwist" and axis is not None and len(parts) >= 4:
        prefix = "angvel" if property_name == "angular" else "linvel"
        if parts[1] == "world" and len(parts) >= 5:
            return f"{prefix}-{parts[3]}-world-{parts[4]}-{_axis_label(axis)}"
        return f"{prefix}-{'-'.join(parts[1:4])}-{_axis_label(axis)}"

    if quantity.type == "Wrench" and property_name == "force":
        if len(parts) >= 5:
            return f"frc-{'-'.join(parts[1:-1])}"
        if len(parts) >= 3:
            return f"frc-{'-'.join(parts[1:3])}"

    if quantity.type == "Pose" and property_name == "distance":
        if axis is None and len(parts) >= 4:
            return f"dist-{'-'.join(parts[1:4])}"
        if axis == "z" and len(parts) >= 4:
            return f"pos-{'-'.join(parts[1:4])}-height"
    if quantity.type == "Pose" and property_name == "rotation" and axis == "z" and len(parts) >= 4:
        return f"ang-{'-'.join(parts[1:4])}-vertical"

    prop = _property_spec(quantity.type, property_name)
    if prop is None or prop.scalar_prefix is None or axis is None:
        return None
    return f"{prop.scalar_prefix}-{_entity_abbrev(quantity.name)}-{axis}"


def _while_signature(constraint: ConstraintSpecification) -> ConstraintSignature:
    assert isinstance(constraint.expr, EqualityConstraint)
    return (
        constraint.view.quantity,
        constraint.view.property,
        constraint.view.axis,
        constraint.expr.reference.variable,
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
    return scalar_id + "-err" if shared else f"{scalar_id}-err-{_motion_suffix(motion_name)}"


def _acceleration_energy_id(
    motion_name: str,
    quantity: WorldQuantity,
    property_name: str,
    axis: str | None,
    shared: bool,
) -> str | None:
    if quantity.type == "VelocityTwist" and axis is not None:
        parts = quantity.name.split("-")
        if parts[1] == "world" and len(parts) >= 5:
            stem = f"eacc-{parts[3]}-world-{parts[4]}-{'ang' if property_name == 'angular' else 'lin'}-{axis}"
        else:
            stem = f"eacc-{'-'.join(parts[1:4])}-{'ang' if property_name == 'angular' else 'lin'}-{axis}"
        return stem
    if quantity.type == "Pose" and property_name == "rotation" and axis == "z":
        parts = quantity.name.split("-")
        if len(parts) >= 4:
            return f"eacc-{'-'.join(parts[1:4])}-ang-z"

    prop = _property_spec(quantity.type, property_name)
    if prop is None or prop.accel_prefix is None:
        return None
    stem = f"eacc-ee-{prop.accel_prefix}-{axis}"
    return stem if shared else f"{stem}-{_motion_suffix(motion_name)}"


def _acceleration_constraint_id(
    motion_name: str,
    quantity: WorldQuantity | None,
    property_name: str,
    axis: str | None,
    shared: bool,
) -> str:
    if quantity is not None and quantity.type == "VelocityTwist" and axis is not None:
        parts = quantity.name.split("-")
        if parts[1] == "world" and len(parts) >= 5:
            return f"acc-cstr-{parts[3]}-world-{parts[4]}-{'ang' if property_name == 'angular' else 'lin'}-{axis}"
        return f"acc-cstr-{'-'.join(parts[1:4])}-{'ang' if property_name == 'angular' else 'lin'}-{axis}"
    if quantity is not None and quantity.type == "Pose" and property_name == "rotation" and axis == "z":
        parts = quantity.name.split("-")
        if len(parts) >= 4:
            return f"acc-cstr-{'-'.join(parts[1:4])}-ang-z"
    stem = f"acc-cstr-ee-{property_name[:3]}-{axis}"
    return stem if shared else f"{stem}-{_motion_suffix(motion_name)}"


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
    qid = constraint.quantity_node_id
    if qid.startswith("pos-"):
        parts = qid.split("-")
        if len(parts) >= 4:
            return f"frc-{parts[1]}-{parts[3]}"
    if qid.startswith("dist-"):
        parts = qid.split("-")
        if len(parts) >= 2:
            return f"frc-{parts[1]}-dist"
    return None


def _control_signal_id_from_wrench_name(name: str) -> str | None:
    parts = name.split("-")
    if len(parts) >= 4 and parts[0] == "wrench" and parts[2] == "dist":
        return f"frc-{parts[1]}-dist"
    return None


def _required_world_quantity(index: "ModelIndex", name: str, reason: str) -> WorldQuantityLike:
    quantity = index.world_quantities.get(name)
    if quantity is None:
        raise ValueError(f"Missing required world quantity '{name}' in context: {reason}")
    return quantity


def _frame_suffix(frame_id: str) -> str:
    return frame_id.removeprefix("frame-")


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


def _lookup_scope_name(lookup: Any) -> str:
    if isinstance(lookup, PreLookup):
        return "Pre"
    if isinstance(lookup, SpecLookup):
        return "Spec"
    if isinstance(lookup, PostLookup):
        return "Post"
    if isinstance(lookup, WorldLookup):
        return "World"
    return lookup.__class__.__name__.removesuffix("Lookup")


class ModelIndex:
    def __init__(self, model: Model):
        self.model = model
        self.models = get_included_models(model)

    @cached_property
    def motion_specs(self) -> list[MotionSpecBlock]:
        return [
            spec
            for model in self.models
            for spec in model.specs
            if isinstance(spec, MotionSpecBlock)
        ]

    @cached_property
    def handler_specs(self) -> list[ConstraintHandlerBlock]:
        return [
            spec
            for model in self.models
            for spec in model.specs
            if isinstance(spec, ConstraintHandlerBlock)
        ]

    @cached_property
    def motion_map(self) -> dict[str, MotionSpecBlock]:
        return {spec.name: spec for spec in self.motion_specs}

    def _context_decl(self, spec: MotionSpecBlock, kind: str) -> Any | None:
        for item in spec.spec.context.items:
            if item.__class__.__name__ == kind:
                return item
        return None

    def world_declarations(self, spec: MotionSpecBlock) -> list[WorldQuantity]:
        declaration = self._context_decl(spec, "WorldContextDecl")
        return declaration.decl.declaration if isinstance(declaration, WorldContextDecl) else []

    def value_declarations(self, spec: MotionSpecBlock, kind: str) -> list[ValueVariable]:
        declaration = self._context_decl(spec, kind)
        if isinstance(declaration, (PreContextDecl, SpecContextDecl, PostContextDecl)):
            return declaration.decl.declaration
        return []

    def all_constraints(self, spec: MotionSpecBlock) -> list[ConstraintSpecification]:
        return [*spec.spec.when, *spec.spec.while_, *spec.spec.until]

    def world_quantity(self, spec: MotionSpecBlock, name: str) -> WorldQuantity | None:
        for quantity in self.world_declarations(spec):
            if quantity.name == name:
                return quantity
        return None

    def value_variable(self, spec: MotionSpecBlock, name: str) -> ValueVariable | None:
        for kind in ("PreContextDecl", "SpecContextDecl", "PostContextDecl"):
            for value in self.value_declarations(spec, kind):
                if value.name == name:
                    return value
        return None

    @cached_property
    def world_quantities(self) -> dict[str, WorldQuantityLike]:
        quantities: dict[str, WorldQuantityLike] = {}
        for motion_spec in self.motion_specs:
            for quantity in self.world_declarations(motion_spec):
                quantities[quantity.name] = quantity
        for handler_spec in self.handler_specs:
            for item in handler_spec.spec.context.items:
                for quantity in item.decl.declaration:
                    quantities[quantity.name] = quantity
        return quantities

    @cached_property
    def value_variables(self) -> dict[str, ValueVariable]:
        values: dict[str, ValueVariable] = {}
        for motion_spec in self.motion_specs:
            for kind in ("PreContextDecl", "SpecContextDecl", "PostContextDecl"):
                for variable in self.value_declarations(motion_spec, kind):
                    values[variable.name] = variable
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
                        entities.setdefault(target, MISC_ENTITY_TYPE["SimplicialComplex"])
                point_id = _geometric_property(quantity.props, "ref-point")
                if point_id:
                    entities.setdefault(point_id, MISC_ENTITY_TYPE["Point"])
                frame_id = _geometric_property(quantity.props, "as-seen-by")
                if frame_id:
                    entities.setdefault(frame_id, MISC_ENTITY_TYPE["Frame"])
            elif quantity.type == "Pose":
                for key in ("of", "wrt", "as-seen-by"):
                    target = _geometric_property(quantity.props, key)
                    if target:
                        entities.setdefault(target, MISC_ENTITY_TYPE["Frame"])
            elif quantity.type == "Wrench":
                point_id = _geometric_property(quantity.props, "ref-point")
                if point_id:
                    entities.setdefault(point_id, MISC_ENTITY_TYPE["Point"])
                frame_id = _geometric_property(quantity.props, "as-seen-by")
                if frame_id:
                    entities.setdefault(frame_id, MISC_ENTITY_TYPE["Frame"])

        for quantity in self.world_quantities.values():
            if quantity.type in {"VelocityTwist", "Wrench"}:
                parts = quantity.name.split("-")
                if len(parts) >= 3:
                    entities.setdefault(f"frame-{parts[2]}", MISC_ENTITY_TYPE["Frame"])
                if len(parts) >= 2:
                    entities.setdefault(f"point-{parts[1]}-origin", MISC_ENTITY_TYPE["Point"])
            if quantity.type == "Wrench":
                parts = quantity.name.split("-")
                if len(parts) >= 3:
                    entities.setdefault(f"link-{parts[1]}-{parts[2]}", MISC_ENTITY_TYPE["SimplicialComplex"])

        return entities

    @cached_property
    def defined_world_names(self) -> set[str]:
        return set(self.world_quantities) | set(self.implicit_world_entities)

    def _require_value_lookup(self, motion_spec: MotionSpecBlock, constraint_name: str, lookup: Any) -> None:
        scope = _lookup_scope_name(lookup)
        variable = lookup.variable
        if scope == "World":
            if variable not in self.defined_world_names:
                raise ValueError(
                    f"Constraint '{constraint_name}' references World[{variable}], but '{variable}' "
                    "is not defined in any World context or derived from geometric properties."
                )
            return

        kind = f"{scope}ContextDecl"
        if not any(value.name == variable for value in self.value_declarations(motion_spec, kind)):
            raise ValueError(
                f"Constraint '{constraint_name}' references {scope}[{variable}], but '{variable}' "
                f"is not defined in the motion '{motion_spec.name}' {scope} context."
            )

    def validate_references(self) -> None:
        for motion_spec in self.motion_specs:
            for constraint in self.all_constraints(motion_spec):
                if self.world_quantity(motion_spec, constraint.view.quantity) is None:
                    raise ValueError(
                        f"Constraint '{constraint.name}' references world quantity "
                        f"'{constraint.view.quantity}' that is not defined in motion '{motion_spec.name}'."
                    )
                expr = constraint.expr
                if isinstance(expr, EqualityConstraint):
                    self._require_value_lookup(motion_spec, constraint.name, expr.reference)
                elif isinstance(expr, (GreaterThanConstraint, LessThanConstraint)):
                    self._require_value_lookup(motion_spec, constraint.name, expr.threshold)
                elif isinstance(expr, BilateralConstraint):
                    self._require_value_lookup(motion_spec, constraint.name, expr.lower)
                    self._require_value_lookup(motion_spec, constraint.name, expr.upper)

        for handler_spec in self.handler_specs:
            motion_spec = self.motion_map.get(handler_spec.spec.motion) if handler_spec.spec.motion else None
            if handler_spec.spec.motion and motion_spec is None:
                raise ValueError(
                    f"Constraint handler '{handler_spec.name}' references motion "
                    f"'{handler_spec.spec.motion}', but it is not defined."
                )

            for monitor in handler_spec.spec.monitors:
                if motion_spec is None:
                    raise ValueError(
                        f"Monitor '{monitor.name}' in handler '{handler_spec.name}' requires a MOTION reference."
                    )
                if (motion_spec.name, monitor.constraint) not in self.constraint_map:
                    raise ValueError(
                        f"Monitor '{monitor.name}' references constraint '{monitor.constraint}', but it is not "
                        f"defined for motion '{motion_spec.name}'."
                    )

            for controller in handler_spec.spec.controllers:
                if motion_spec is None:
                    raise ValueError(
                        f"Controller '{controller.name}' in handler '{handler_spec.name}' requires a MOTION reference."
                    )
                if (motion_spec.name, controller.params.constraint) not in self.constraint_map:
                    raise ValueError(
                        f"Controller '{controller.name}' references constraint "
                        f"'{controller.params.constraint}', but it is not defined for motion '{motion_spec.name}'."
                    )
                if controller.apply_at and controller.apply_at not in self.defined_world_names:
                    raise ValueError(
                        f"Controller '{controller.name}' references World[{controller.apply_at}], but "
                        f"'{controller.apply_at}' is not defined in any World context or derived from geometric properties."
                    )

            solver = handler_spec.spec.solver
            if solver is None:
                raise ValueError(f"Constraint handler '{handler_spec.name}' is missing required SOLVER section.")
            for attr in ("chain", "root", "gravity"):
                value = getattr(solver, attr, "")
                if value and value not in self.defined_world_names:
                    raise ValueError(
                        f"Solver in handler '{handler_spec.name}' references World[{value}] for {attr}, but "
                        f"'{value}' is not defined in any World context or derived from geometric properties."
                    )
            for force_name in getattr(solver, "cartesian_force", []):
                if force_name not in self.defined_world_names:
                    raise ValueError(
                        f"Solver in handler '{handler_spec.name}' references cartesian-force '{force_name}', "
                        "but it is not defined in any World context or derived from geometric properties."
                    )
            for force_name in getattr(solver, "joint_force", []):
                if force_name not in self.defined_world_names:
                    raise ValueError(
                        f"Solver in handler '{handler_spec.name}' references joint-force '{force_name}', "
                        "but it is not defined in any World context or derived from geometric properties."
                    )
            for velocity_solver in getattr(solver, "velocity_solvers", []):
                if velocity_solver.velocity not in self.defined_world_names:
                    raise ValueError(
                        f"Velocity solver '{velocity_solver.name}' references World[{velocity_solver.velocity}], "
                        "but it is not defined in any World context or derived from geometric properties."
                    )
            for force_solver in getattr(solver, "force_solvers", []):
                if force_solver.force not in self.defined_world_names:
                    raise ValueError(
                        f"Force solver '{force_solver.name}' references World[{force_solver.force}], "
                        "but it is not defined in any World context or derived from geometric properties."
                    )

    @cached_property
    def rotation_ids(self) -> dict[str, str]:
        # Only for axis-less rotation constraints (property == "rotation" with no axis).
        # Axis-based rotation constraints (e.g. rotation.z) are handled by angle_from_pose_ops.
        rotation_ids: dict[str, str] = {}
        for motion_spec in self.motion_specs:
            for constraint in self.all_constraints(motion_spec):
                quantity = self.world_quantity(motion_spec, constraint.view.quantity)
                if (
                    quantity
                    and quantity.type == "Pose"
                    and constraint.view.property == "rotation"
                    and constraint.view.axis is None
                ):
                    rotation_ids[motion_spec.name] = f"rotation-{_motion_suffix(motion_spec.name)}"
        return rotation_ids

    @cached_property
    def angle_from_pose_ops(self) -> list[PoseAngleOpData]:
        # For pose.rotation.z (explicit axis) constraints: generate GEOM_OP.PoseToAngleAroundAxis
        # ops so that ir_gen.py can schedule the computation of the angle scalar.
        ops: list[PoseAngleOpData] = []
        seen: set[str] = set()
        for motion_spec in self.motion_specs:
            for constraint in self.all_constraints(motion_spec):
                quantity = self.world_quantity(motion_spec, constraint.view.quantity)
                if (
                    quantity is None
                    or quantity.type != "Pose"
                    or constraint.view.property != "rotation"
                    or constraint.view.axis is None
                ):
                    continue
                scalar_id = _scalar_id(quantity, "rotation", constraint.view.axis)
                if scalar_id is None or scalar_id in seen:
                    continue
                seen.add(scalar_id)
                ops.append(
                    PoseAngleOpData(
                        op_id=f"compute-{scalar_id}",
                        pose_name=quantity.name,
                        angle_id=scalar_id,
                        axis=constraint.view.axis,
                    )
                )
        return ops

    @cached_property
    def scalar_views(self) -> dict[tuple[str, str, str], ScalarViewData]:
        views: dict[tuple[str, str, str], ScalarViewData] = {}
        for motion_spec in self.motion_specs:
            for constraint in self.all_constraints(motion_spec):
                quantity = self.world_quantity(motion_spec, constraint.view.quantity)
                axis = constraint.view.axis
                if quantity is None or axis is None:
                    continue
                prop = _property_spec(quantity.type, constraint.view.property)
                scalar_id = _scalar_id(quantity, constraint.view.property, axis)
                scalar_type = _view_scalar_type(quantity, constraint.view.property, axis)
                if prop is None or prop.view_type is None or scalar_id is None or scalar_type is None:
                    continue
                if quantity.type == "Pose" and constraint.view.property == "rotation":
                    continue
                key = (constraint.view.quantity, constraint.view.property, axis)
                views.setdefault(
                    key,
                    ScalarViewData(
                        quantity_name=constraint.view.quantity,
                        property_name=constraint.view.property,
                        axis=axis,
                        scalar_id=scalar_id,
                        scalar_type=scalar_type,
                        view_type=prop.view_type,
                        view_subspace=prop.view_subspace,
                    ),
                )
        return views

    @cached_property
    def shared_while_signatures(self) -> set[ConstraintSignature]:
        usage: dict[ConstraintSignature, set[str]] = {}
        for motion_spec in self.motion_specs:
            for constraint in motion_spec.spec.while_:
                if not isinstance(constraint.expr, EqualityConstraint):
                    continue
                usage.setdefault(_while_signature(constraint), set()).add(motion_spec.name)
        return {signature for signature, motions in usage.items() if len(motions) > 1}

    @cached_property
    def controlled_constraints(self) -> list[ConstraintData]:
        while_names_by_motion = {
            motion_spec.name: {constraint.name for constraint in motion_spec.spec.while_}
            for motion_spec in self.motion_specs
        }
        return [
            constraint
            for constraint in self.constraints
            if constraint.constraint.name in while_names_by_motion.get(constraint.motion_name, set())
        ]

    @cached_property
    def constraints(self) -> list[ConstraintData]:
        derived_constraints: list[ConstraintData] = []
        for motion_spec in self.motion_specs:
            motion_name = motion_spec.name
            while_names = {constraint.name for constraint in motion_spec.spec.while_}

            for constraint in self.all_constraints(motion_spec):
                quantity = self.world_quantity(motion_spec, constraint.view.quantity)
                if quantity is None:
                    raise ValueError(
                        f"Constraint '{constraint.name}' references world quantity "
                        f"'{constraint.view.quantity}' that is not defined in the motion context."
                    )
                property_spec = _property_spec(quantity.type if quantity else "Unknown", constraint.view.property)
                scalar_type_name = (
                    _view_scalar_type(quantity, constraint.view.property, constraint.view.axis)
                    if quantity is not None
                    else None
                ) or (property_spec.scalar_type if property_spec else constraint.view.property)

                quantity_node_id = _scalar_id(quantity, constraint.view.property, constraint.view.axis) if quantity else None
                quantity_node_id = quantity_node_id or constraint.view.quantity

                expr = constraint.expr
                signature = None
                error_signal_id = None
                acceleration_energy = None

                if isinstance(expr, EqualityConstraint):
                    kind = "EqualityConstraint"
                    reference_var = expr.reference.variable
                    threshold_var = lower_var = upper_var = None
                    if constraint.name in while_names and quantity is not None:
                        signature = _while_signature(constraint)
                        shared = signature in self.shared_while_signatures
                        error_signal_id = _while_error_id(
                            motion_name,
                            quantity,
                            constraint.view.property,
                            constraint.view.axis,
                            shared,
                        )
                        acceleration_energy = _acceleration_energy_id(
                            motion_name,
                            quantity,
                            constraint.view.property,
                            constraint.view.axis,
                            shared,
                        )
                elif isinstance(expr, GreaterThanConstraint):
                    kind = "GreaterThanConstraint"
                    reference_var = None
                    threshold_var = expr.threshold.variable
                    lower_var = upper_var = None
                elif isinstance(expr, LessThanConstraint):
                    kind = "LessThanConstraint"
                    reference_var = None
                    threshold_var = expr.threshold.variable
                    lower_var = upper_var = None
                else:
                    assert isinstance(expr, BilateralConstraint)
                    kind = "BilateralConstraint"
                    reference_var = threshold_var = None
                    lower_var = expr.lower.variable
                    upper_var = expr.upper.variable

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
    def while_constraints(self) -> list[ConstraintData]:
        return [constraint for constraint in self.constraints if constraint.signature is not None]

    @cached_property
    def while_error_signals(self) -> dict[str, ErrorSignalData]:
        return {
            constraint.error_signal_id: ErrorSignalData(constraint.error_signal_id, constraint.scalar_type)
            for constraint in self.controlled_constraints
            if constraint.error_signal_id is not None
        }

    @cached_property
    def acceleration_energies(self) -> dict[str, str]:
        return {
            constraint.acceleration_energy_id: constraint.acceleration_energy_id
            for constraint in self.while_constraints
            if constraint.acceleration_energy_id is not None
        }

    @cached_property
    def monitor_data(self) -> list[MonitorData]:
        monitors: list[MonitorData] = []
        for handler_spec in self.handler_specs:
            if not handler_spec.spec.motion:
                continue
            motion_spec = self.motion_map.get(handler_spec.spec.motion)
            if motion_spec is None:
                continue
            for monitor in handler_spec.spec.monitors:
                constraint = self.constraint_map.get((motion_spec.name, monitor.constraint))
                if constraint is None:
                    continue
                is_edge = bool(monitor.event)
                signal_name = monitor.event or monitor.flag
                monitors.append(
                    MonitorData(
                        motion_name=motion_spec.name,
                        monitor=monitor,
                        constraint=constraint,
                        evaluator_id=f"eval-{monitor.constraint.removeprefix('cstr-')}",
                        error_signal_id=f"{monitor.constraint}-err",
                        signal_kind="event" if is_edge else "flag",
                        signal_id=signal_name,
                    )
                )
        return monitors

    @cached_property
    def monitor_error_signals(self) -> dict[str, ErrorSignalData]:
        return {
            monitor.error_signal_id: ErrorSignalData(monitor.error_signal_id, monitor.constraint.scalar_type)
            for monitor in self.monitor_data
        }

    @cached_property
    def controller_data(self) -> dict[str, ControllerData]:
        controllers: dict[str, ControllerData] = {}
        for handler_spec in self.handler_specs:
            if not handler_spec.spec.motion:
                continue
            motion_spec = self.motion_map.get(handler_spec.spec.motion)
            if motion_spec is None:
                continue
            for controller in handler_spec.spec.controllers:
                key = str(URIRef(controller.uri))
                constraint = self.constraint_map.get((motion_spec.name, controller.params.constraint))
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
                    output_type=controller.output_type or None,
                    apply_at=controller.apply_at or None,
                    feed_scope=controller.feed_scope or None,
                    feed_kind=controller.feed_kind or None,
                    params=controller.params,
                )
        return controllers

    @cached_property
    def evaluator_defs(self) -> dict[str, tuple[str, str]]:
        evaluators: dict[str, tuple[str, str]] = {}
        for constraint in self.controlled_constraints:
            if constraint.error_signal_id is not None:
                evaluators[f"eval-{constraint.constraint.name.removeprefix('cstr-')}"] = (
                    constraint.constraint.uri,
                    constraint.error_signal_id,
                )
        for monitor in self.monitor_data:
            evaluators[monitor.evaluator_id] = (monitor.constraint.constraint.uri, monitor.error_signal_id)
        return evaluators

    @cached_property
    def handlers(self) -> list[HandlerData]:
        handlers: list[HandlerData] = []
        monitors_by_motion: dict[str, list[MonitorData]] = {}
        for monitor in self.monitor_data:
            monitors_by_motion.setdefault(monitor.motion_name, []).append(monitor)

        for handler_spec in self.handler_specs:
            if not handler_spec.spec.motion:
                continue
            motion_spec = self.motion_map.get(handler_spec.spec.motion)
            if motion_spec is None:
                continue

            while_constraints = [
                constraint
                for constraint in self.controlled_constraints
                if constraint.motion_name == motion_spec.name
            ]
            motion_monitors = monitors_by_motion.get(motion_spec.name, [])
            handlers.append(
                HandlerData(
                    node_id=str(URIRef(handler_spec.uri)),
                    motion_name=motion_spec.name,
                    motion_id=f"motion-{_motion_suffix(motion_spec.name)}",
                    evaluator_ids=tuple(
                        [f"eval-{constraint.constraint.name.removeprefix('cstr-')}" for constraint in while_constraints]
                        + [monitor.evaluator_id for monitor in motion_monitors]
                    ),
                    controller_ids=tuple(str(URIRef(controller.uri)) for controller in handler_spec.spec.controllers),
                    monitor_ids=tuple(str(URIRef(monitor.monitor.uri)) for monitor in motion_monitors),
                )
            )
        return handlers

    @cached_property
    def acceleration_constraints(self) -> dict[str, AccelerationConstraintData]:
        constraints: dict[str, AccelerationConstraintData] = {}
        for constraint in self.while_constraints:
            quantity = constraint.quantity
            prop = _property_spec(quantity.type, constraint.constraint.view.property) if quantity else None
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
                constraint.constraint.view.property,
                constraint.constraint.view.axis,
                shared,
            )
            constraints.setdefault(
                node_id,
                AccelerationConstraintData(
                    node_id=node_id,
                    energy_id=constraint.acceleration_energy_id,
                    subspace=prop.accel_subspace,
                    axis=constraint.constraint.view.axis or "",
                ),
            )
        return constraints

    @cached_property
    def base_velocity_solvers(self) -> list[BaseVelocitySolverData]:
        solvers: list[BaseVelocitySolverData] = []
        for handler_spec in self.handler_specs:
            if handler_spec.spec.solver is None:
                continue
            for solver in getattr(handler_spec.spec.solver, "velocity_solvers", []):
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
            if handler_spec.spec.solver is None:
                continue
            for solver in getattr(handler_spec.spec.solver, "force_solvers", []):
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

    def constraint_quantity_node(self, constraint: ConstraintData, base_ns: Namespace) -> Node:
        return base_ns[constraint.quantity_node_id]

    @cached_property
    def distance_derivations(self) -> list[DistanceDerivation]:
        derivations: list[DistanceDerivation] = []
        for constraint in self.controlled_constraints:
            quantity = constraint.quantity
            if (
                quantity is None
                or quantity.type != "Pose"
                or constraint.constraint.view.property != "distance"
                or constraint.constraint.view.axis is not None
            ):
                continue

            parts = quantity.name.split("-")
            observer_frame = _with_respect_to(quantity)
            if len(parts) < 4 or observer_frame is None:
                warnings.warn(
                    f"Skipping distance derivation for '{quantity.name}' because the pose does not "
                    "follow the expected naming or is missing 'wrt'."
                )
                continue

            group, proximal, distal = parts[1], parts[2], parts[3]
            observer_point = f"point-{group}-{proximal}-origin"
            derivations.append(
                DistanceDerivation(
                    constraint_name=constraint.constraint.name,
                    pose_name=quantity.name,
                    group=group,
                    proximal=proximal,
                    distal=distal,
                    observer_frame=observer_frame,
                    observer_point=observer_point,
                    distance_id=constraint.quantity_node_id,
                    direction_id=f"dir-{group}-{proximal}-to-{distal}",
                    position_id=f"pos-{group}-{proximal}-{proximal}",
                    force_id=f"frc-{group}-dist",
                    local_wrench_id=f"wrench-{group}-dist-{proximal}",
                    pose_to_dist_op_id=f"pose-to-dist-{group}",
                    pose_to_dir_op_id=f"pose-to-dir-{group}",
                    wrench_op_id=f"compute-wrench-{group}-dist-{proximal}",
                )
            )
        return derivations


def _find_pose(index: ModelIndex, *, of_frame: str, wrt_frame: str) -> WorldQuantityLike | None:
    for quantity in index.world_quantities.values():
        if quantity.type != "Pose":
            continue
        if _of_frame(quantity) == of_frame and _with_respect_to(quantity) == wrt_frame:
            return quantity
    return None


def _derived_transformed_wrenches(
    index: ModelIndex,
) -> tuple[dict[str, list[tuple[DistanceDerivation, str, str]]], dict[str, str]]:
    transforms_by_force: dict[str, list[tuple[DistanceDerivation, str, str]]] = {}
    add_ops_by_force: dict[str, str] = {}

    for solver in index.base_force_solvers:
        target_quantity = _required_world_quantity(
            index,
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
        for derivation in index.distance_derivations:
            pose = _find_pose(index, of_frame=derivation.observer_frame, wrt_frame=target_frame)
            if pose is None:
                warnings.warn(
                    f"Skipping transform for '{derivation.local_wrench_id}' because no pose was found "
                    f"from '{derivation.observer_frame}' to '{target_frame}'."
                )
                continue
            target_suffix = _frame_suffix(target_frame)
            transformed_wrench = derivation.local_wrench_id.rsplit("-", 1)[0] + f"-{target_suffix}"
            transformed.append((derivation, pose.name, transformed_wrench))

        transforms_by_force[solver.force] = transformed
        if len(transformed) > 1:
            add_ops_by_force[solver.force] = f"add-{solver.force.removesuffix('-' + _frame_suffix(target_frame))}"

    return transforms_by_force, add_ops_by_force


def gen_misc(index: ModelIndex, base_ns: Namespace) -> Graph:
    graph = _graph(
        ("app", base_ns),
        ("geom-ent", GEOM_ENT),
        ("geom-op", GEOM_OP),
        ("rbdyn-op", RBDYN_OP),
        ("slv", SLV),
    )
    entities: dict[str, tuple[Node, Node]] = {}

    for quantity in index.world_quantities.values():
        rdf_type = MISC_ENTITY_TYPE.get(quantity.type)
        if rdf_type is not None:
            entities[quantity.name] = (URIRef(quantity.uri), rdf_type)

    for entity_name, rdf_type in index.implicit_world_entities.items():
        entities.setdefault(entity_name, (base_ns[entity_name], rdf_type))

    for quantity in index.world_quantities.values():
        if quantity.type == "VelocityTwist" and isinstance(quantity.props, GeometricProps):
            for key in ("of", "wrt"):
                target = _geometric_property(quantity.props, key)
                if target:
                    entities.setdefault(target, (base_ns[target], MISC_ENTITY_TYPE["SimplicialComplex"]))
            point_id = _geometric_property(quantity.props, "ref-point")
            if point_id:
                entities.setdefault(point_id, (base_ns[point_id], MISC_ENTITY_TYPE["Point"]))
            frame_id = _geometric_property(quantity.props, "as-seen-by")
            if frame_id:
                entities.setdefault(frame_id, (base_ns[frame_id], MISC_ENTITY_TYPE["Frame"]))
        elif quantity.type == "Pose" and isinstance(quantity.props, GeometricProps):
            for key in ("of", "wrt", "as-seen-by"):
                target = _geometric_property(quantity.props, key)
                if target:
                    entities.setdefault(target, (base_ns[target], MISC_ENTITY_TYPE["Frame"]))
        elif quantity.type == "Wrench":
            point_id = (
                _geometric_property(quantity.props, "ref-point")
                if isinstance(quantity.props, GeometricProps)
                else None
            )
            frame_id = (
                _geometric_property(quantity.props, "as-seen-by")
                if isinstance(quantity.props, GeometricProps)
                else None
            )
            if point_id is None:
                parts = quantity.name.split("-")
                if len(parts) >= 2:
                    point_id = f"point-{parts[1]}-origin"
            if point_id:
                entities.setdefault(point_id, (base_ns[point_id], MISC_ENTITY_TYPE["Point"]))
            if frame_id:
                entities.setdefault(frame_id, (base_ns[frame_id], MISC_ENTITY_TYPE["Frame"]))
            parts = quantity.name.split("-")
            if len(parts) >= 3:
                link_id = f"link-{parts[1]}-{parts[2]}"
                entities.setdefault(link_id, (base_ns[link_id], MISC_ENTITY_TYPE["SimplicialComplex"]))

    for derivation in index.distance_derivations:
        entities.setdefault(derivation.observer_point, (base_ns[derivation.observer_point], MISC_ENTITY_TYPE["Point"]))

    for node, rdf_type in sorted(entities.values(), key=lambda item: str(item[0])):
        _add_types(graph, node, rdf_type)

    for solver in index.base_velocity_solvers:
        node = base_ns[solver.node_id]
        _add_types(graph, node, SLV.VelocityCompositionSolver)
        graph.add((node, SLV.configuration, Literal(solver.configuration)))
        graph.add((node, SLV.velocity, base_ns[solver.velocity]))

        source_quantity = _required_world_quantity(
            index,
            solver.velocity,
            f"referenced by base velocity solver '{solver.node_id}'",
        )
        source_frame = _reference_frame(source_quantity)
        if source_frame is None:
            raise ValueError(
                f"Base velocity solver '{solver.node_id}' references '{solver.velocity}', which is missing "
                "'as-seen-by' in context."
            )

        for quantity in index.world_quantities.values():
            if quantity.type != "VelocityTwist" or quantity.name == solver.velocity:
                continue
            if _geometric_property(quantity.props, "of") != _geometric_property(source_quantity.props, "of"):
                continue
            if _geometric_property(quantity.props, "wrt") != _geometric_property(source_quantity.props, "wrt"):
                continue
            target_frame = _reference_frame(quantity)
            if target_frame is None or target_frame == source_frame:
                continue
            pose = _find_pose(index, of_frame=target_frame, wrt_frame=source_frame)
            if pose is None:
                warnings.warn(
                    f"Skipping derived twist rotation for '{quantity.name}' because no pose was found "
                    f"from '{target_frame}' to '{source_frame}'."
                )
                continue
            op_node = base_ns[f"rot-{_frame_suffix(source_frame)}-to-{_frame_suffix(target_frame)}"]
            _add_types(graph, op_node, GEOM_OP.RotateVelocityTwistToProximalWithPose)
            graph.add((op_node, GEOM_OP.pose, base_ns[pose.name]))
            graph.add((op_node, GEOM_OP["from"], base_ns[solver.velocity]))
            graph.add((op_node, GEOM_OP.to, base_ns[quantity.name]))

    transforms_by_force, add_ops_by_force = _derived_transformed_wrenches(index)
    for solver_force, transforms in transforms_by_force.items():
        for derivation, pose_name, transformed_wrench in transforms:
            op_node = base_ns[f"tf-{derivation.local_wrench_id}-{_frame_suffix(_reference_frame(_required_world_quantity(index, solver_force, 'base force target')) or '')}"]
            _add_types(graph, op_node, RBDYN_OP.TransformWrenchToProximal)
            graph.add((op_node, RBDYN_OP.pose, base_ns[pose_name]))
            graph.add((op_node, RBDYN_OP["from"], base_ns[derivation.local_wrench_id]))
            graph.add((op_node, RBDYN_OP.to, base_ns[transformed_wrench]))

        add_op = add_ops_by_force.get(solver_force)
        if add_op and len(transforms) >= 2:
            op_node = base_ns[add_op]
            _add_types(graph, op_node, RBDYN_OP.AddWrench)
            graph.add((op_node, RBDYN_OP.in1, base_ns[transforms[0][2]]))
            graph.add((op_node, RBDYN_OP.in2, base_ns[transforms[1][2]]))
            graph.add((op_node, RBDYN_OP.out, base_ns[solver_force]))

    return graph


def gen_world_model(index: ModelIndex, base_ns: Namespace) -> Graph:
    graph = _graph(
        ("app", base_ns),
        ("geom-ent", GEOM_ENT),
        ("geom-rel", GEOM_REL),
        ("geom-coord", GEOM_COORD),
        ("rbdyn-ent", RBDYN_ENT),
        ("rbdyn-coord", RBDYN_COORD),
        ("qudt", QUDT_SCHEMA),
        ("qkind", QUDT_QKIND),
        ("unit", QUDT_UNIT),
    )

    for quantity in index.world_quantities.values():
        world_spec = _world_spec(quantity.type)
        if world_spec is None:
            continue

        node = URIRef(quantity.uri)
        _add_types(graph, node, *world_spec.rdf_types)
        for qkind in world_spec.qkinds:
            graph.add((node, QUDT_SCHEMA["quantity-kind"], qkind))
        for unit in world_spec.units:
            graph.add((node, QUDT_SCHEMA.unit, unit))

        reference_point = None
        as_seen_by = None
        of_value = None
        wrt_value = None
        if isinstance(quantity.props, GeometricProps):
            of_value = _geometric_property(quantity.props, "of")
            wrt_value = _geometric_property(quantity.props, "wrt")
            reference_point = _geometric_property(quantity.props, "ref-point")
            as_seen_by = _geometric_property(quantity.props, "as-seen-by")
            if of_value:
                graph.add((node, GEOM_REL.of, base_ns[of_value]))
            if wrt_value:
                graph.add((node, GEOM_REL["with-respect-to"], base_ns[wrt_value]))
            if reference_point:
                graph.add((node, GEOM_REL["reference-point"], base_ns[reference_point]))
            if as_seen_by:
                graph.add((node, GEOM_COORD["as-seen-by"], base_ns[as_seen_by]))
            # Pose quantities default to their reference frame as observer when the DSL omits it.
            if quantity.type == "Pose" and not as_seen_by and wrt_value:
                graph.add((node, GEOM_COORD["as-seen-by"], base_ns[wrt_value]))
                as_seen_by = wrt_value

        parts = quantity.name.split("-")
        if quantity.type in {"VelocityTwist", "Wrench"} and not as_seen_by and len(parts) >= 3:
            graph.add((node, GEOM_COORD["as-seen-by"], base_ns[f"frame-{parts[2]}"]))
        if quantity.type in {"VelocityTwist", "Wrench"} and not reference_point:
            point_owner = None
            if of_value and of_value.startswith("link-"):
                point_owner = of_value.removeprefix("link-")
            elif len(parts) >= 2:
                point_owner = parts[1]
            if point_owner:
                graph.add((node, GEOM_REL["reference-point"], base_ns[f"point-{point_owner}-origin"]))

    for view in index.scalar_views.values():
        _add_quantity(graph, base_ns[view.scalar_id], view.scalar_type)

    for constraint in index.constraints:
        if constraint.scalar_type == "PlaneAngle" and constraint.quantity_node_id.startswith("ang-"):
            _add_quantity(graph, base_ns[constraint.quantity_node_id], "PlaneAngle")

    for rotation_id in index.rotation_ids.values():
        _add_quantity(graph, base_ns[rotation_id], "PlaneAngle")

    for variable in index.value_variables.values():
        node = URIRef(variable.uri)
        qkind = _dsl_scalar_qkind(variable.type)
        _add_types(graph, node, QUDT_SCHEMA.Quantity, qkind)
        graph.add((node, QUDT_SCHEMA["quantity-kind"], qkind))
        graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(variable.value.unit)))
        graph.add((node, QUDT_SCHEMA.value, Literal(str(variable.value.value))))

    for signal in index.while_error_signals.values():
        _add_quantity(graph, base_ns[signal.node_id], signal.scalar_type)

    for signal in index.monitor_error_signals.values():
        _add_quantity(graph, base_ns[signal.node_id], signal.scalar_type)

    for energy_id in index.acceleration_energies.values():
        node = base_ns[energy_id]
        _add_types(graph, node, QUDT_SCHEMA.Quantity, URIRef("AccelerationEnergy"))
        graph.add((node, QUDT_SCHEMA["quantity-kind"], URIRef("AccelerationEnergy")))
        graph.add((node, QUDT_SCHEMA.unit, URIRef("N-M2-PER-SEC2")))

    for controller in index.controller_data.values():
        if controller.control_signal_id.startswith("frc-"):
            _add_quantity(graph, base_ns[controller.control_signal_id], "Force")

    for derivation in index.distance_derivations:
        _add_quantity(graph, base_ns[derivation.distance_id], "Length")

        node = base_ns[derivation.direction_id]
        _add_types(graph, node, GEOM_REL.Direction, GEOM_COORD.DirectionCoordinate, GEOM_COORD.VectorXYZ)
        graph.add((node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Direction))
        graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.UNITLESS))
        graph.add((node, GEOM_COORD["as-seen-by"], base_ns[derivation.observer_frame]))

        pos_node = base_ns[derivation.position_id]
        _add_types(graph, pos_node, GEOM_REL.Position, GEOM_COORD.PositionCoordinate, GEOM_COORD.VectorXYZ)
        graph.add((pos_node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Length))
        graph.add((pos_node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        graph.add((pos_node, GEOM_REL.of, base_ns[derivation.observer_point]))
        graph.add((pos_node, GEOM_REL["with-respect-to"], base_ns[derivation.observer_point]))
        graph.add((pos_node, GEOM_COORD["as-seen-by"], base_ns[derivation.observer_frame]))
        graph.add((pos_node, GEOM_COORD.x, Literal(0)))
        graph.add((pos_node, GEOM_COORD.y, Literal(0)))
        graph.add((pos_node, GEOM_COORD.z, Literal(0)))

        wrench_node = base_ns[derivation.local_wrench_id]
        _add_types(graph, wrench_node, RBDYN_ENT.Wrench, RBDYN_COORD.WrenchCoordinate, GEOM_COORD.VectorXYZ)
        graph.add((wrench_node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Torque))
        graph.add((wrench_node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Force))
        graph.add((wrench_node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M"]))
        graph.add((wrench_node, QUDT_SCHEMA.unit, QUDT_UNIT.N))
        graph.add((wrench_node, GEOM_REL["reference-point"], base_ns[derivation.observer_point]))
        graph.add((wrench_node, RBDYN_COORD["as-seen-by"], base_ns[derivation.observer_frame]))

    transforms_by_force, _ = _derived_transformed_wrenches(index)
    for solver_force, transforms in transforms_by_force.items():
        target_quantity = _required_world_quantity(index, solver_force, "base force target")
        target_frame = _reference_frame(target_quantity)
        if target_frame is None:
            continue
        target_point = _geometric_property(target_quantity.props, "ref-point") if isinstance(target_quantity.props, GeometricProps) else None
        for _, _, transformed_wrench in transforms:
            node = base_ns[transformed_wrench]
            _add_types(graph, node, RBDYN_ENT.Wrench, RBDYN_COORD.WrenchCoordinate, GEOM_COORD.VectorXYZ)
            graph.add((node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Torque))
            graph.add((node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Force))
            graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M"]))
            graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.N))
            if target_point:
                graph.add((node, GEOM_REL["reference-point"], base_ns[target_point]))
            graph.add((node, RBDYN_COORD["as-seen-by"], base_ns[target_frame]))

        if transforms and solver_force not in index.world_quantities:
            node = base_ns[solver_force]
            _add_types(graph, node, RBDYN_ENT.Wrench, RBDYN_COORD.WrenchCoordinate, GEOM_COORD.VectorXYZ)
            graph.add((node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Torque))
            graph.add((node, QUDT_SCHEMA["quantity-kind"], QUDT_QKIND.Force))
            graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M"]))
            graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.N))
            if target_point:
                graph.add((node, GEOM_REL["reference-point"], base_ns[target_point]))
            graph.add((node, RBDYN_COORD["as-seen-by"], base_ns[target_frame]))

    return graph


def gen_map(index: ModelIndex, base_ns: Namespace) -> Graph:
    graph = _graph(("app", base_ns), ("map", MAP), ("geom-op", GEOM_OP), ("rbdyn-op", RBDYN_OP))

    for view in index.scalar_views.values():
        node = base_ns[f"view-{view.scalar_id}"]
        _add_types(graph, node, MAP.View)
        if view.view_type:
            _add_types(graph, node, view.view_type)
        graph.add((node, MAP.superobject, base_ns[view.quantity_name]))
        graph.add((node, MAP.subobject, base_ns[view.scalar_id]))
        if view.view_subspace:
            graph.add((node, MAP.subspace, MAP[view.view_subspace]))
        graph.add((node, MAP.axis, MAP[view.axis]))

    for motion_name, rotation_id in index.rotation_ids.items():
        motion_spec = index.motion_map[motion_name]
        # rotation_ids only covers axis-less rotation constraints
        pose_quantity = next(
            (
                constraint.view.quantity
                for constraint in index.all_constraints(motion_spec)
                if constraint.view.property == "rotation" and constraint.view.axis is None
            ),
            None,
        )
        if pose_quantity is None:
            continue
        op_node = base_ns[f"compute-{rotation_id}"]
        _add_types(graph, op_node, MAP.ComputeRotationFromPose)
        graph.add((op_node, MAP.pose, base_ns[pose_quantity]))
        graph.add((op_node, MAP.rotation, base_ns[rotation_id]))

    for op_data in index.angle_from_pose_ops:
        op_node = base_ns[op_data.op_id]
        _add_types(graph, op_node, GEOM_OP.PoseToAngleAroundAxis)
        graph.add((op_node, GEOM_OP.pose, base_ns[op_data.pose_name]))
        graph.add((op_node, GEOM_OP.angle, base_ns[op_data.angle_id]))
        graph.add((op_node, GEOM_OP.axis, MAP[op_data.axis]))

    existing_scalar_views = {view.scalar_id for view in index.scalar_views.values()}
    for handler_spec in index.handler_specs:
        solver = handler_spec.spec.solver
        if solver is None or not getattr(solver, "algorithm", ""):
            continue
        for binding in _infer_cartesian_force_bindings(index, handler_spec):
            wrench_name = binding.force_name
            quantity = index.world_quantities.get(wrench_name)
            if quantity is None or quantity.type != "Wrench":
                continue
            scalar_id = _scalar_id(quantity, "force", "z")
            if scalar_id is None or scalar_id in existing_scalar_views:
                continue
            node = base_ns[f"view-{scalar_id}"]
            _add_types(graph, node, MAP.View, MAP.WrenchCoordinateView)
            graph.add((node, MAP.superobject, base_ns[wrench_name]))
            graph.add((node, MAP.subobject, base_ns[scalar_id]))
            graph.add((node, MAP.subspace, MAP.force))
            graph.add((node, MAP.axis, MAP.z))

    controller_signals = {controller.control_signal_id for controller in index.controller_data.values()}
    for derivation in index.distance_derivations:
        dist_node = base_ns[derivation.pose_to_dist_op_id]
        _add_types(graph, dist_node, GEOM_OP.PoseToLinearDistance)
        graph.add((dist_node, GEOM_OP.pose, base_ns[derivation.pose_name]))
        graph.add((dist_node, GEOM_OP.distance, base_ns[derivation.distance_id]))

        dir_node = base_ns[derivation.pose_to_dir_op_id]
        _add_types(graph, dir_node, GEOM_OP.PoseToDirection)
        graph.add((dir_node, GEOM_OP.pose, base_ns[derivation.pose_name]))
        graph.add((dir_node, GEOM_OP.direction, base_ns[derivation.direction_id]))

        if derivation.force_id not in controller_signals:
            warnings.warn(
                f"Skipping wrench derivation for '{derivation.constraint_name}' because no controller "
                f"produces '{derivation.force_id}'."
            )
            continue
        wrench_node = base_ns[derivation.wrench_op_id]
        _add_types(graph, wrench_node, RBDYN_OP.WrenchFromPositionDirectionAndMagnitude)
        graph.add((wrench_node, RBDYN_OP.magnitude, base_ns[derivation.force_id]))
        graph.add((wrench_node, RBDYN_OP.position, base_ns[derivation.position_id]))
        graph.add((wrench_node, RBDYN_OP.direction, base_ns[derivation.direction_id]))
        graph.add((wrench_node, RBDYN_OP.wrench, base_ns[derivation.local_wrench_id]))

    return graph


def gen_constraints(index: ModelIndex, base_ns: Namespace) -> Graph:
    graph = _graph(("app", base_ns), ("cstr", CSTR))

    for constraint in index.constraints:
        node = URIRef(constraint.constraint.uri)
        type_name = {"PlaneAngle": "Angle", "Length": "Distance"}.get(
            constraint.scalar_type, constraint.scalar_type
        )
        _add_types(graph, node, CSTR.Constraint, CSTR[f"{type_name}Constraint"])
        graph.add((node, CSTR.quantity, index.constraint_quantity_node(constraint, base_ns)))

        if constraint.kind == "EqualityConstraint":
            reference = index.value_variable(index.motion_map[constraint.motion_name], constraint.reference_var or "")
            _add_types(graph, node, CSTR.EqualityConstraint)
            graph.add(
                (
                    node,
                    CSTR["reference-value"],
                    URIRef(reference.uri) if reference else base_ns[constraint.reference_var or ""],
                )
            )
        elif constraint.kind == "GreaterThanConstraint":
            threshold = index.value_variable(index.motion_map[constraint.motion_name], constraint.threshold_var or "")
            _add_types(graph, node, CSTR.UnilateralConstraint, CSTR.GreaterThanConstraint)
            graph.add(
                (
                    node,
                    CSTR.threshold,
                    URIRef(threshold.uri) if threshold else base_ns[constraint.threshold_var or ""],
                )
            )
        elif constraint.kind == "LessThanConstraint":
            threshold = index.value_variable(index.motion_map[constraint.motion_name], constraint.threshold_var or "")
            _add_types(graph, node, CSTR.UnilateralConstraint, CSTR.LessThanConstraint)
            graph.add(
                (
                    node,
                    CSTR.threshold,
                    URIRef(threshold.uri) if threshold else base_ns[constraint.threshold_var or ""],
                )
            )
        else:
            lower = index.value_variable(index.motion_map[constraint.motion_name], constraint.lower_var or "")
            upper = index.value_variable(index.motion_map[constraint.motion_name], constraint.upper_var or "")
            _add_types(graph, node, CSTR.BilateralConstraint)
            graph.add(
                (
                    node,
                    CSTR["lower-threshold"],
                    URIRef(lower.uri) if lower else base_ns[constraint.lower_var or ""],
                )
            )
            graph.add(
                (
                    node,
                    CSTR["upper-threshold"],
                    URIRef(upper.uri) if upper else base_ns[constraint.upper_var or ""],
                )
            )

    return graph


def gen_motion_specification(index: ModelIndex, base_ns: Namespace) -> Graph:
    graph = _graph(("app", base_ns), ("mot", MOT))
    for motion_spec in index.motion_specs:
        motion_node = base_ns[f"motion-{_motion_suffix(motion_spec.name)}"]
        _add_types(graph, motion_node, MOT.GuardedMotion)
        for constraint in motion_spec.spec.when:
            graph.add((motion_node, MOT.when, URIRef(constraint.uri)))
        for constraint in motion_spec.spec.while_:
            graph.add((motion_node, MOT["while"], URIRef(constraint.uri)))
        for constraint in motion_spec.spec.until:
            graph.add((motion_node, MOT.until, URIRef(constraint.uri)))
    return graph


def gen_constraint_handler(index: ModelIndex, base_ns: Namespace) -> Graph:
    graph = _graph(("app", base_ns), ("cstr-hdl", CSTR_HDL))

    for evaluator_id, (constraint_uri, error_id) in index.evaluator_defs.items():
        node = base_ns[evaluator_id]
        _add_types(graph, node, CSTR_HDL.ConstraintEvaluator, CSTR_HDL.ErrorEvaluator)
        graph.add((node, CSTR_HDL.constraint, URIRef(constraint_uri)))
        graph.add((node, CSTR_HDL.error, base_ns[error_id]))

    for controller_key, controller in index.controller_data.items():
        node = URIRef(controller_key)
        _add_types(graph, node, CSTR_HDL.Controller, CSTR_HDL.ProportionalIntegralDerivative)
        if controller.error_signal_id:
            graph.add((node, CSTR_HDL["error-signal"], base_ns[controller.error_signal_id]))
        graph.add((node, CSTR_HDL["control-signal"], base_ns[controller.control_signal_id]))
        graph.add((node, CSTR_HDL["proportional-gain"], Literal(str(controller.params.kp))))
        graph.add((node, CSTR_HDL["integral-gain"], Literal(str(controller.params.ki))))
        graph.add((node, CSTR_HDL["derivative-gain"], Literal(str(controller.params.kd))))

    for monitor in index.monitor_data:
        signal_node = base_ns[monitor.signal_id]
        _add_types(graph, signal_node, CSTR_HDL.Event if monitor.signal_kind == "event" else CSTR_HDL.Flag)

        node = URIRef(monitor.monitor.uri)
        _add_types(graph, node, CSTR_HDL.Monitor)
        graph.add((node, CSTR_HDL.error, base_ns[monitor.error_signal_id]))
        if monitor.signal_kind == "event":
            _add_types(graph, node, CSTR_HDL.EdgeTriggeredMonitor)
            graph.add((node, CSTR_HDL.event, signal_node))
        else:
            _add_types(graph, node, CSTR_HDL.LevelTriggeredMonitor)
            graph.add((node, CSTR_HDL.flag, signal_node))

    for handler in index.handlers:
        node = URIRef(handler.node_id)
        _add_types(graph, node, CSTR_HDL.ConstraintHandler)
        graph.add((node, CSTR_HDL.motion, base_ns[handler.motion_id]))
        for evaluator_id in handler.evaluator_ids:
            graph.add((node, CSTR_HDL.evaluators, base_ns[evaluator_id]))
        for controller_id in handler.controller_ids:
            graph.add((node, CSTR_HDL.controllers, URIRef(controller_id)))
        for monitor_id in handler.monitor_ids:
            graph.add((node, CSTR_HDL.monitors, URIRef(monitor_id)))

    return graph


def gen_solver_specification(index: ModelIndex, base_ns: Namespace) -> Graph:
    graph = _graph(("app", base_ns), ("slv", SLV))

    for handler_spec in index.handler_specs:
        solver = handler_spec.spec.solver
        if not handler_spec.spec.motion or solver is None or not getattr(solver, "algorithm", ""):
            continue
        motion_name = handler_spec.spec.motion
        motion_spec = index.motion_map.get(motion_name)
        if motion_spec is None:
            continue

        attached_link = _infer_attached_link(index, handler_spec, solver)

        spec_constraints: list[str] = []
        for constraint in index.while_constraints:
            if constraint.motion_name != motion_name:
                continue
            quantity = constraint.quantity
            property_spec = (
                _property_spec(quantity.type, constraint.constraint.view.property) if quantity else None
            )
            if (
                quantity is None
                or property_spec is None
                or property_spec.accel_subspace is None
                or constraint.acceleration_energy_id is None
            ):
                continue

            shared = constraint.signature in index.shared_while_signatures if constraint.signature else False
            node_id = _acceleration_constraint_id(
                motion_name,
                quantity,
                constraint.constraint.view.property,
                constraint.constraint.view.axis,
                shared,
            )
            if node_id not in spec_constraints:
                spec_constraints.append(node_id)
            data = index.acceleration_constraints[node_id]
            node = base_ns[data.node_id]
            _add_types(graph, node, SLV.AccelerationConstraint, SLV.AxisAligned)
            graph.add((node, SLV.subspace, SLV[data.subspace]))
            graph.add((node, SLV.axis, SLV[data.axis]))
            graph.add((node, SLV["acceleration-energy"], base_ns[data.energy_id]))

        motion_suffix = _motion_suffix(motion_name)
        if attached_link:
            abbrev = _entity_abbrev(attached_link) if "-" in attached_link else attached_link
            spec_acc_node = base_ns[f"spec-acc-{abbrev}-{motion_suffix}"]
        else:
            spec_acc_node = base_ns[f"spec-acc-ee-{motion_suffix}"]

        for binding in _infer_cartesian_force_bindings(index, handler_spec):
            force_name = binding.force_name
            spec_name = binding.spec_name
            _required_world_quantity(
                index,
                force_name,
                f"referenced by cartesian-force solver output '{spec_name}'",
            )
            spec_node = base_ns[spec_name]
            _add_types(graph, spec_node, SLV.CartesianForceSpecification)
            graph.add((spec_node, SLV.force, base_ns[force_name]))
            attached_force_link = binding.attached_to
            if attached_force_link is None:
                attached_force_link = _attached_link_from_wrench_name(force_name)
            if attached_force_link is not None:
                graph.add((spec_node, SLV["attached-to"], base_ns[attached_force_link]))

    return graph


def gen_scenario(index: ModelIndex, base_ns: Namespace) -> Graph:
    graph = _graph(("app", base_ns), ("slv", SLV))

    for solver in index.base_force_solvers:
        node = base_ns[solver.node_id]
        _add_types(graph, node, SLV.ForceDistributionSolver)
        graph.add((node, SLV.configuration, Literal(solver.configuration)))
        graph.add((node, SLV.force, base_ns[solver.force]))

    for handler_spec in index.handler_specs:
        solver = handler_spec.spec.solver
        if not handler_spec.spec.motion or solver is None or not getattr(solver, "algorithm", ""):
            continue

        motion_name = handler_spec.spec.motion
        motion_suffix = _motion_suffix(motion_name)
        driver_suffix = _driver_suffix(handler_spec)
        motion_spec = index.motion_map.get(motion_name)
        if motion_spec is None:
            continue

        attached_link = _infer_attached_link(index, handler_spec, solver)

        if attached_link:
            abbrev = _entity_abbrev(attached_link) if "-" in attached_link else attached_link
            spec_acc_node = base_ns[f"spec-acc-{abbrev}-{motion_suffix}"]
        else:
            spec_acc_node = base_ns[f"spec-acc-ee-{motion_suffix}"]

        spec_constraints: list[str] = []
        for constraint in index.while_constraints:
            if constraint.motion_name != motion_name:
                continue
            quantity = constraint.quantity
            property_spec = (
                _property_spec(quantity.type, constraint.constraint.view.property) if quantity else None
            )
            if (
                quantity is None
                or property_spec is None
                or property_spec.accel_subspace is None
                or constraint.acceleration_energy_id is None
            ):
                continue
            shared = constraint.signature in index.shared_while_signatures if constraint.signature else False
            node_id = _acceleration_constraint_id(
                motion_name,
                quantity,
                constraint.constraint.view.property,
                constraint.constraint.view.axis,
                shared,
            )
            if node_id not in spec_constraints:
                spec_constraints.append(node_id)

        _add_types(graph, spec_acc_node, SLV.AccelerationConstraintSpecification)
        for node_id in spec_constraints:
            graph.add((spec_acc_node, SLV.constraints, base_ns[node_id]))
        graph.add(
            (
                spec_acc_node,
                SLV["attached-to"],
                base_ns[attached_link] if attached_link else base_ns["link-ee"],
            )
        )

        driver_node = base_ns[f"drv-{driver_suffix}"]
        _add_types(graph, driver_node, SLV.MotionDrivers)
        graph.add((driver_node, SLV["acceleration-constraint"], spec_acc_node))
        for binding in _infer_cartesian_force_bindings(index, handler_spec):
            spec_name = binding.spec_name
            graph.add((driver_node, SLV["cartesian-force"], base_ns[spec_name]))
        for joint_force in getattr(solver, "joint_force", []):
            graph.add((driver_node, SLV["joint-force"], base_ns[joint_force]))
        solver_node = base_ns[f"slv-{driver_suffix}"]
        if solver.algorithm == "Vereshchagin":
            solver_algorithm = SLV.AccelerationConstrainedHybridDynamicsAlgorithm
        elif solver.algorithm == "NewtonEuler":
            solver_algorithm = SLV.NewtonEulerAlgorithm
        else:
            solver_algorithm = SLV[solver.algorithm]
        _add_types(graph, solver_node, SLV.SolverWithInputAndOutput)
        graph.add((solver_node, SLV.solver, solver_algorithm))
        _required_world_quantity(index, solver.chain, f"referenced by solver '{handler_spec.name}' chain")
        _required_world_quantity(index, solver.root, f"referenced by solver '{handler_spec.name}' root")
        _required_world_quantity(index, solver.gravity, f"referenced by solver '{handler_spec.name}' gravity")
        graph.add((solver_node, SLV["kinematic-chain"], base_ns[solver.chain]))
        graph.add((solver_node, SLV.root, base_ns[solver.root]))
        graph.add((solver_node, SLV.gravity, base_ns[solver.gravity]))
        graph.add((solver_node, SLV["motion-drivers"], driver_node))

    return graph


def _context_dict(base_prefix: str, base_ns: Namespace, *bindings: NamespaceBinding) -> dict[str, str]:
    context = {base_prefix: str(base_ns)}
    for prefix, namespace in bindings:
        context[prefix] = str(namespace._NS)
    return context


def _base_namespace(index: ModelIndex) -> tuple[str, Namespace]:
    first_spec = next(iter([*index.motion_specs, *index.handler_specs]), None)
    assert first_spec is not None, "No MOTION_SPEC or CONSTRAINT_HANDLER found in model"
    return first_spec.ns_prefix, Namespace(first_spec.ns.uri)


def get_motion_spec_graphs(model: Model) -> list[GraphOutput]:
    index = ModelIndex(model)
    index.validate_references()
    base_prefix, base_ns = _base_namespace(index)

    return [
        ("00-misc.json", gen_misc(index, base_ns), _context_dict(base_prefix, base_ns, ("geom-ent", GEOM_ENT))),
        (
            "01-world-model.json",
            gen_world_model(index, base_ns),
            _context_dict(
                base_prefix,
                base_ns,
                ("geom-ent", GEOM_ENT),
                ("geom-rel", GEOM_REL),
                ("geom-coord", GEOM_COORD),
                ("rbdyn-ent", RBDYN_ENT),
                ("rbdyn-coord", RBDYN_COORD),
                ("qudt", QUDT_SCHEMA),
                ("qkind", QUDT_QKIND),
                ("unit", QUDT_UNIT),
            ),
        ),
        (
            "02-map.json",
            gen_map(index, base_ns),
            ["https://comp-rob2b.github.io/metamodels/task/map.json", {"@base": str(base_ns)}],
        ),
        ("03-constraints.json", gen_constraints(index, base_ns), _context_dict(base_prefix, base_ns, ("cstr", CSTR))),
        (
            "04-motion-specification.json",
            gen_motion_specification(index, base_ns),
            _context_dict(base_prefix, base_ns, ("mot", MOT)),
        ),
        (
            "05-constraint-handler.json",
            gen_constraint_handler(index, base_ns),
            _context_dict(base_prefix, base_ns, ("cstr-hdl", CSTR_HDL)),
        ),
        (
            "06-solver-specification.json",
            gen_solver_specification(index, base_ns),
            [
                "https://comp-rob2b.github.io/metamodels/task/solver-specification.json",
                {"@base": str(base_ns)},
            ],
        ),
        (
            "07-scenario.json",
            gen_scenario(index, base_ns),
            [
                "https://comp-rob2b.github.io/metamodels/task/solver-specification.json",
                {"@base": str(base_ns)},
            ],
        ),
    ]
