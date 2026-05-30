# SPDX-License-Identifier: MPL-2.0
"""Simple RDF graph builder — walks from ConstraintHandlers, no caching layer."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

from rdflib.graph import Dataset
from rdflib.namespace import Namespace, RDF, XSD
from rdflib.term import Literal, URIRef

from textx.scoping import get_included_models

from motion_spec.namespace import (
    EL,
    APP,
    CSTR,
    CSTR_HDL,
    ENV,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_OP,
    GEOM_REL,
    KC,
    KC_STAT,
    MAP,
    MJ,
    MOT,
    MOT_EXT,
    POLY,
    TRAJ,

    QUDT_QKIND,
    QUDT_SCHEMA,
    QUDT_UNIT,
    RBDYN_COORD,
    RBDYN_ENT,
    RBDYN_OP,
    RT,
    SIM,
    SNAP,
    SLV,
    VALUE_ROLE,
)
from motion_spec_dsl.controller_semantics import (
    AccelerationConstraintRecord,
    SUBSPACE_ALIAS,
    axis_label as semantic_axis_label,
    constraint_view_subspace,
    controller_command_record,
)
from motion_spec_dsl.domain import (
    BilateralConstraint,
    ConstraintHandler,
    ConstraintSpecification,
    ContextDeclReference,
    ControllerEntry,
    ControllerType,
    ContextRef,
    ContextSpec,
    EqualityConstraint,
    EnvironmentRuntime,
    EnvironmentSpec,
    GeoPropPair,
    GeometricPropKey,
    GeometricProps,
    GreaterThanConstraint,
    LessThanConstraint,
    Model,
    MotionSpec,
    PostContextDecl,
    PreContextDecl,
    QuantityType,
    PoseValue,
    ScalarQuantity,
    SnapshotValue,
    SpecContextDecl,
    ContextQuantity,
    TrajectoryValue,
    UntilMonitorRef,
    VectorQuantity,
    WorldContextDecl,
    WorldQuantity,
    WorldQuantityType,
    _resolved_context_quantity,
    _resolved_spec,
    _resolved_solver,
)

# Each entry: (rdf_types, qkinds, units, prop_map)
# prop_map[subspace] = (view_subspace_uri, accel_subspace_uri, accel_prefix, scalar_type, view_rdf_type)
WORLD_SPECS: dict[WorldQuantityType, tuple] = {
    WorldQuantityType.VelocityTwist: (
        (
            QUDT_SCHEMA.Quantity,
            GEOM_REL.VelocityTwist,
            GEOM_COORD.VelocityTwistCoordinate,
            GEOM_COORD.VectorXYZ,
        ),
        (QUDT_QKIND.AngularVelocity, QUDT_QKIND.LinearVelocity),
        (QUDT_UNIT["RAD-PER-SEC"], QUDT_UNIT["M-PER-SEC"]),
        {
            "angular": (
                "angular-velocity",
                "angular-acceleration",
                "ang",
                QuantityType.AngularVelocity,
                MAP.VelocityTwistCoordinateView,
            ),
            "linear": (
                "linear-velocity",
                "linear-acceleration",
                "lin",
                QuantityType.LinearVelocity,
                MAP.VelocityTwistCoordinateView,
            ),
        },
    ),
    WorldQuantityType.Wrench: (
        (
            QUDT_SCHEMA.Quantity,
            RBDYN_ENT.Wrench,
            RBDYN_COORD.WrenchCoordinate,
            GEOM_COORD.VectorXYZ,
        ),
        (QUDT_QKIND.Torque, QUDT_QKIND.Force),
        (QUDT_UNIT["N-M"], QUDT_UNIT.N),
        {
            "torque": ("torque", None, None, QuantityType.Torque, MAP.WrenchCoordinateView),
            "force": ("force", None, None, QuantityType.Force, MAP.WrenchCoordinateView),
        },
    ),
    WorldQuantityType.Pose: (
        (
            QUDT_SCHEMA.Quantity,
            GEOM_REL.Pose,
            GEOM_COORD.PoseCoordinate,
            GEOM_COORD.DirectionCosineXYZ,
            GEOM_COORD.VectorXYZ,
        ),
        (QUDT_QKIND.PlaneAngle, QUDT_QKIND.Length),
        (QUDT_UNIT.UNITLESS, QUDT_UNIT.M),
        {
            "rotation": (
                "rotation",
                "angular-acceleration",
                "ang",
                QuantityType.PlaneAngle,
                MAP.PoseCoordinateView,
            ),
            "distance": (
                "position",
                "linear-acceleration",
                "lin",
                QuantityType.Distance,
                MAP.PoseCoordinateView,
            ),
        },
    ),
    WorldQuantityType.JointPosition: (
        (QUDT_SCHEMA.Quantity, KC_STAT.JointPositionCoordinate),
        (QUDT_QKIND.PlaneAngle,),
        (QUDT_UNIT.RAD,),
        {},
    ),
}

WORLD_STRUCTURE_TYPES: dict[WorldQuantityType, Any] = {
    WorldQuantityType.Frame: GEOM_ENT.Frame,
    WorldQuantityType.Link: GEOM_ENT.SimplicialComplex,
    WorldQuantityType.SceneObject: ENV.RigidObject,
    WorldQuantityType.KinematicChain: GEOM_ENT.KinematicChain,
    WorldQuantityType.Gravity: GEOM_ENT.UniformGravitationalField,
}

SCALAR_UNIT: dict[Any, Any] = {
    QuantityType.Pose: QUDT_UNIT.UNITLESS,
    QuantityType.Position: QUDT_UNIT.M,
    QuantityType.Orientation: QUDT_UNIT.UNITLESS,
    QuantityType.Distance: QUDT_UNIT.M,
    QuantityType.Angle: QUDT_UNIT["RAD"],
    QuantityType.PlaneAngle: QUDT_UNIT["RAD"],
    QuantityType.LinearVelocity: QUDT_UNIT["M-PER-SEC"],
    QuantityType.AngularVelocity: QUDT_UNIT["RAD-PER-SEC"],
    QuantityType.LinearAcceleration: QUDT_UNIT["M-PER-SEC2"],
    QuantityType.AngularAcceleration: QUDT_UNIT["RAD-PER-SEC2"],
    QuantityType.Force: QUDT_UNIT.N,
    QuantityType.Torque: QUDT_UNIT["N-M"],
    QuantityType.TrajectoryProgress: QUDT_UNIT.UNITLESS,
}

DSL_UNIT: dict[str, Any] = {
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
    "geom-rel": "https://comp-rob2b.github.io/metamodels/geometry/spatial-relations.ttl",
    "geom-coord": "https://comp-rob2b.github.io/metamodels/geometry/coordinates.ttl",
    "geom-op": "https://comp-rob2b.github.io/metamodels/geometry/spatial-operators.ttl",
    "rbdyn-op": "https://comp-rob2b.github.io/metamodels/newtonian-rigid-body-dynamics/operators.ttl",
    "map": "https://comp-rob2b.github.io/metamodels/task/map.ttl",
    "cstr": "https://comp-rob2b.github.io/metamodels/task/constraint.ttl",
    "mot": "https://comp-rob2b.github.io/metamodels/task/motion-specification.ttl",
    "cstr-hdl": "https://secorolab.github.io/metamodels/task/constraint-handler.shacl.ttl",
    "slv": "https://comp-rob2b.github.io/metamodels/task/solver-specification.ttl",
}

CSTR_TYPE_NAME: dict[Any, str] = {
    QuantityType.PlaneAngle: QuantityType.Angle,
}

QUDT_KIND_BY_QUANTITY_TYPE: dict[Any, Any] = {
    QuantityType.Pose: GEOM_REL.Pose,
    QuantityType.Position: QUDT_QKIND.Position,
    QuantityType.Orientation: QUDT_QKIND.Direction,
    QuantityType.Direction: QUDT_QKIND.Direction,
    QuantityType.Vector: QUDT_QKIND.Vector,
    QuantityType.Trajectory: TRAJ.Trajectory,
    QuantityType.TrajectoryProgress: TRAJ.Progress,
}

GRAPH_BINDINGS: tuple[tuple[str, Any], ...] = (
    ("kc", KC),
    ("kc-stat", KC_STAT),
    ("geom-ent", GEOM_ENT),
    ("geom-rel", GEOM_REL),
    ("geom-coord", GEOM_COORD),
    ("geom-op", GEOM_OP),
    ("env", ENV),
    ("sim", SIM),

    ("el", EL),
    ("rt", RT),
    ("mj", MJ),
    ("poly", POLY),
    ("snap", SNAP),
    ("rbdyn-ent", RBDYN_ENT),
    ("rbdyn-coord", RBDYN_COORD),
    ("rbdyn-op", RBDYN_OP),
    ("qudt", QUDT_SCHEMA),
    ("qkind", QUDT_QKIND),
    ("unit", QUDT_UNIT),
    ("map", MAP),
    ("cstr", CSTR),
    ("mot", MOT),
    ("mot-ext", MOT_EXT),
    ("cstr-hdl", CSTR_HDL),
    ("slv", SLV),
)


def _ns_term(namespace: Any, name: str) -> URIRef:
    return URIRef(str(namespace._NS) + name)


def _node_name(value: Any) -> str:
    return value.name if hasattr(value, "name") else str(value)


def _body_name(name: str) -> str:
    for prefix in ("frame-", "link-"):
        if name.startswith(prefix):
            return name[len(prefix) :]
    return name


def _geo_prop(props: GeometricProps | None, key: str) -> str | None:
    if props is None:
        return None
    for pair in props.pairs:
        if isinstance(pair, GeoPropPair) and pair.key == key:
            return pair.value
    return None


def _is_distance_view(constraint: ConstraintSpecification) -> bool:
    return (
        getattr(constraint.view, "distance_from", None) is not None
        and getattr(constraint.view, "distance_to", None) is not None
    )


def _view_subspace(constraint: ConstraintSpecification) -> str:
    subspace = constraint_view_subspace(constraint)
    if subspace is None:
        raise ValueError(f"Constraint '{constraint.name}' must define a view subspace.")
    return subspace


def _scalar_id(quantity: WorldQuantity, subspace: str, axis: str | None) -> str:
    if quantity.type == WorldQuantityType.JointPosition:
        return quantity.name
    if axis is None:
        return f"{quantity.name}.{subspace}"
    return f"{quantity.name}.{subspace}.{axis}"


def _axis_vector(axis: str) -> tuple[float, float, float]:
    return {
        "x": (1.0, 0.0, 0.0),
        "y": (0.0, 1.0, 0.0),
        "z": (0.0, 0.0, 1.0),
    }[axis]


def _quantity_axis_frame(quantity: WorldQuantity) -> str | None:
    props = quantity.props if isinstance(quantity.props, GeometricProps) else None
    axis_frame = _geo_prop(props, "as-seen-by")
    if axis_frame is not None:
        return axis_frame
    if quantity.type == WorldQuantityType.Pose:
        return _geo_prop(props, "wrt")
    return None


def _pose_diff_error_id(ctrl: ControllerEntry, component: AccelerationConstraintRecord) -> str:
    return f"{ctrl.name}-err-{component.suffix}"


def _pose_diff_controller_id(ctrl: ControllerEntry, component: AccelerationConstraintRecord) -> str:
    return f"{ctrl.name}-{component.suffix}"


def _pose_diff_energy_id(ctrl: ControllerEntry, component: AccelerationConstraintRecord) -> str:
    return f"eacc-{ctrl.name}-{component.suffix}"


def _scalar_type(quantity: WorldQuantity, subspace: str, axis: str | None) -> Any:
    if quantity.type == WorldQuantityType.JointPosition:
        return QuantityType.Angle
    if quantity.type == WorldQuantityType.Pose:
        if subspace == "pose":
            return QuantityType.Pose
        if subspace == "position":
            return QuantityType.Position if axis is None else QuantityType.Distance
        if subspace == "orientation":
            return QuantityType.Orientation if axis is None else QuantityType.Angle
        if subspace == "distance":
            return QuantityType.Distance
        if subspace == "rotation":
            return QuantityType.PlaneAngle
    spec = WORLD_SPECS.get(quantity.type)
    if spec is None:
        return subspace
    prop = spec[3].get(subspace)
    return prop[3] if prop else subspace


def _evaluator_id(spec: ConstraintSpecification) -> str:
    section = getattr(spec, "parent", None)
    motion = getattr(section, "parent", None) if section is not None else None
    section_kind = getattr(section, "kind", None)
    motion_name = getattr(motion, "name", None)
    if motion_name and section_kind:
        return f"eval-{motion_name}-{section_kind}-{spec.name}"
    return f"eval-{spec.name}"


def _dsl_unit(unit_name: str) -> Any:
    try:
        return DSL_UNIT[unit_name]
    except KeyError as exc:
        raise ValueError(f"Unsupported DSL unit '{unit_name}'.") from exc


def _context_quantity(ref: ContextRef) -> ContextQuantity | None:
    return getattr(ref, "quantity", None) or getattr(ref, "value", None)



def _resolved_constraint_items(motion: MotionSpec) -> list[ConstraintSpecification]:
    out = []
    for section in (motion.when, motion.while_, motion.until):
        for item in section.constraints:
            spec = _resolved_spec(item)
            if not spec.disabled:
                out.append(spec)
    return out


@dataclass(frozen=True)
class _PosePathStep:
    quantity: WorldQuantity
    inverted: bool


@dataclass(frozen=True)
class _RelativePosePlan:
    start: WorldQuantity
    end: WorldQuantity
    target: WorldQuantity
    reference_path: tuple[_PosePathStep, ...]


class MotionSpecDatasetBuilder:
    def __init__(self, model: Model):
        self.model = model
        self.models = get_included_models(model)
        self.dataset = Dataset()
        for prefix, ns in GRAPH_BINDINGS:
            self.dataset.bind(prefix, ns)
        self.authored_handlers: list[ConstraintHandler] = [
            spec for m in self.models for spec in m.specs if isinstance(spec, ConstraintHandler)
        ]
        self.graph = self.dataset.default_graph
        self._default_ns_owner: Any | None = next(iter(self.authored_handlers), None)
        self._relative_pose_plans: dict[tuple[Any, ...], _RelativePosePlan] = {}

    def build(self) -> tuple[Dataset, dict[str, str]]:
        handlers = self.authored_handlers

        shared_spec_ids = self._compute_shared_specs(handlers)

        context: dict[str, str] = {}
        for prefix, ns in GRAPH_BINDINGS:
            context[prefix] = str(ns._NS)

        for model in self.models:
            for spec in model.specs:
                if isinstance(spec, EnvironmentSpec):
                    self._emit_environment(spec)
                    self.dataset.bind(spec.ns_prefix, spec.ns.uri)
                    context[spec.ns_prefix] = spec.ns.uri
                elif isinstance(spec, ContextSpec):
                    self.dataset.bind(spec.ns_prefix, spec.ns.uri)
                    context[spec.ns_prefix] = spec.ns.uri

        for handler_order, handler in enumerate(handlers):
            motion = handler.motion
            if not isinstance(motion, MotionSpec):
                continue

            self.dataset.bind(handler.ns_prefix, handler.ns.uri)
            self.dataset.bind(motion.ns_prefix, motion.ns.uri)
            context[handler.ns_prefix] = handler.ns.uri
            context[motion.ns_prefix] = motion.ns.uri

            world_qtys = self._collect_world_quantities(motion, handler)
            context_quantities = self._collect_context_quantities(motion, handler)
            constraints = _resolved_constraint_items(motion)

            self._emit_structural_entities(world_qtys)
            self._emit_world_quantities(world_qtys)
            self._emit_derived_quantity_transforms(motion, world_qtys)
            self._emit_context_quantities(context_quantities, constraints, world_qtys)
            self._emit_constraints(motion, constraints, world_qtys)
            self._emit_motion_spec(motion)
            self._emit_scalar_views(motion, constraints, world_qtys)
            self._emit_map_operations(motion, constraints, world_qtys)
            self._emit_constraint_handler(
                handler, motion, world_qtys, shared_spec_ids, handler_order
            )
            self._emit_solvers(handler, motion, world_qtys, shared_spec_ids)

        return self.dataset, context

    @staticmethod
    def _model_uri(owner: Any, model: str) -> URIRef:
        return URIRef(f"{owner.ns.uri}{model}") if model else URIRef(f"{owner.uri}.model")

    def _emit_object_model(
        self, model_node: URIRef, *, resource_path: str = "", model_type: Any | None = None
    ) -> None:
        self.graph.add((model_node, RDF.type, ENV.ObjectModel))
        if model_type is not None:
            self.graph.add((model_node, RDF.type, model_type))
        if resource_path:
            self.graph.add((model_node, RDF.type, SIM.ResourceWithPath))
            self.graph.add((model_node, SIM.path, Literal(resource_path)))

    def _emit_runtime_chain_binding(
        self,
        object_node: URIRef,
        *,
        root_name: str,
        end_name: str,
        tool_body: str = "",
        tcp_site: str = "",
    ) -> None:
        chain_node = URIRef(f"{object_node}.chain")
        root_node = URIRef(f"{object_node}.chain.root")
        end_node = URIRef(f"{object_node}.chain.end")
        self.graph.add((chain_node, RDF.type, GEOM_ENT.KinematicChain))
        self.graph.add((root_node, RDF.type, GEOM_ENT.Frame))
        self.graph.add((end_node, RDF.type, GEOM_ENT.Frame))
        self.graph.add((object_node, GEOM_ENT["kinematic-chain"], chain_node))
        self.graph.add((chain_node, GEOM_ENT.start, root_node))
        self.graph.add((chain_node, GEOM_ENT.end, end_node))
        self.graph.add((root_node, MJ["body-name"], Literal(root_name)))
        self.graph.add((end_node, MJ["body-name"], Literal(end_name)))
        if tool_body:
            body_node = URIRef(f"{object_node}.tool-body")
            self.graph.add((body_node, RDF.type, MJ.MuJoCoBody))
            self.graph.add((body_node, MJ["body-name"], Literal(tool_body)))
            self.graph.add((object_node, MJ["tool-body"], body_node))
        if tcp_site:
            site_node = URIRef(f"{object_node}.tcp-site")
            self.graph.add((site_node, RDF.type, MJ.MuJoCoSite))
            self.graph.add((site_node, MJ["site-name"], Literal(tcp_site)))
            self.graph.add((object_node, MJ["tcp-site"], site_node))

    def _emit_position_xyz(self, node: URIRef, value) -> None:
        values = {term.axis: term.value for term in value.terms}
        self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
        self.graph.add((node, GEOM_COORD.x, Literal(float(values.get("x", 0.0)), datatype=XSD.double)))
        self.graph.add((node, GEOM_COORD.y, Literal(float(values.get("y", 0.0)), datatype=XSD.double)))
        self.graph.add((node, GEOM_COORD.z, Literal(float(values.get("z", 0.0)), datatype=XSD.double)))

    def _emit_orientation_rpy(
        self, node: URIRef, value, as_seen_by: URIRef | None = None
    ) -> None:
        self.graph.add((node, RDF.type, GEOM_REL.Orientation))
        self.graph.add((node, RDF.type, GEOM_COORD.OrientationCoordinate))
        if as_seen_by is not None:
            self.graph.add((node, GEOM_COORD["as-seen-by"], as_seen_by))
        for term in value.terms:
            term_node = URIRef(f"{node}.{term.axis}")
            self.graph.add((term_node, RDF.type, GEOM_COORD.OrientationCoordinate))
            if as_seen_by is not None:
                self.graph.add((term_node, GEOM_COORD["as-seen-by"], as_seen_by))
            self.graph.add((term_node, GEOM_COORD["angle-axis"], Literal(term.axis)))
            self.graph.add((term_node, QUDT_SCHEMA.value, Literal(term.value)))
            self.graph.add((term_node, QUDT_SCHEMA.unit, URIRef(str(QUDT_UNIT._NS) + term.unit.upper())))
            self.graph.add((node, GEOM_COORD["has-coordinate"], term_node))

    def _emit_color_rgba(self, owner_node, r, g, b, a) -> None:
        """Attach a grouped mj:ColorRGBA value node to owner_node via mj:color.

        One canonical colour representation shared by scene objects and the
        trajectory-trace overlay, mirroring how positions group into VectorXYZ.
        """
        color_node = URIRef(f"{owner_node}.color")
        self.graph.add((owner_node, MJ["color"], color_node))
        self.graph.add((color_node, RDF.type, MJ.ColorRGBA))
        self.graph.add((color_node, MJ["color-r"], Literal(float(r), datatype=XSD.double)))
        self.graph.add((color_node, MJ["color-g"], Literal(float(g), datatype=XSD.double)))
        self.graph.add((color_node, MJ["color-b"], Literal(float(b), datatype=XSD.double)))
        self.graph.add((color_node, MJ["color-a"], Literal(float(a), datatype=XSD.double)))

    def _emit_environment(self, env: EnvironmentSpec) -> None:
        env_node = URIRef(env.uri)
        world_node = URIRef(f"{env.uri}.world")
        self.graph.add((env_node, RDF.type, ENV.Workspace))
        self.graph.add((world_node, RDF.type, GEOM_ENT.Frame))
        self.graph.add((world_node, RDF.type, GEOM_ENT.Point))
        if env.runtime == EnvironmentRuntime.MuJoCo:
            self.graph.add((env_node, RT["uses-runtime"], RT.MuJoCoRuntime))
        elif env.runtime == EnvironmentRuntime.RealRobot:
            self.graph.add((env_node, RT["uses-runtime"], RT.RealRobotRuntime))

        if env.trace is not None:
            trace_node = URIRef(f"{env.uri}.trace")
            self.graph.add((env_node, MJ["has-trace"], trace_node))
            self.graph.add((trace_node, RDF.type, MJ.TrajectoryTrace))
            self.graph.add((trace_node, MJ["trace-enabled"], Literal(env.trace.enabled)))
            self.graph.add(
                (trace_node, MJ["trace-length"], Literal(env.trace.length or 4096))
            )
            self._emit_color_rgba(
                trace_node,
                env.trace.channel("r", 1.0),
                env.trace.channel("g", 0.5),
                env.trace.channel("b", 0.1),
                env.trace.channel("a", 1.0),
            )

        for asset in env.assets:
            asset_node = URIRef(asset.uri)
            self.graph.add((env_node, ENV["has-object"], asset_node))
            self.graph.add((asset_node, RDF.type, ENV.Object))
            if asset.model:
                self._emit_object_model(
                    self._model_uri(env, asset.model),
                    resource_path=asset.xml,
                    model_type=MJ.MjcfModel if asset.xml else None,
                )
            self._emit_object_model(asset_node)
            if asset.xml:
                self.graph.add((asset_node, RDF.type, MJ.MjcfModel))
                self.graph.add((asset_node, SIM.path, Literal(asset.xml)))
        robot_instances = [
            instance
            for instance in env.assembly
            if str(instance.type) == "Robot"
        ]

        for instance in env.assembly:
            is_attachment_instance = str(instance.type) == "Attachment"
            instance_node = URIRef(instance.uri)
            model_node = (
                self._model_uri(env, instance.asset.model)
                if instance.asset.model
                else URIRef(instance.asset.uri)
            )
            self.graph.add((env_node, ENV["has-object"], instance_node))
            self.graph.add((instance_node, RDF.type, ENV.ModelledObject))
            if not is_attachment_instance:
                self.graph.add((instance_node, RDF.type, ENV.RigidObject))
                self.graph.add((instance_node, RDF.type, GEOM_ENT.Frame))
                self.graph.add((instance_node, RDF.type, GEOM_ENT.Point))
                self.graph.add((instance_node, RDF.type, GEOM_ENT.SimplicialComplex))
            self.graph.add((instance_node, ENV["of-object"], URIRef(instance.asset.uri)))
            self.graph.add((instance_node, ENV["has-object-model"], model_node))
            if is_attachment_instance and len(robot_instances) == 1:
                self.graph.add((instance_node, SLV["attached-to"], URIRef(robot_instances[0].uri)))
            if str(instance.type) == "Object":
                self.graph.add((instance_node, RDF.type, ENV.Object))
            for entry in instance.entries:
                entry_type = entry.__class__.__name__
                if entry_type == "EnvironmentPositionEntry":
                    if is_attachment_instance:
                        pos_node = URIRef(f"{instance.uri}.attach-position")
                        self._emit_position_xyz(pos_node, entry.value)
                        self.graph.add((instance_node, MJ["attach-position"], pos_node))
                        continue
                    pos_node = URIRef(f"{instance.uri}.position")
                    self._emit_position_xyz(pos_node, entry.value)
                    # Position-specific semantics layered on the shared VectorXYZ core.
                    self.graph.add((pos_node, RDF.type, GEOM_REL.Position))
                    self.graph.add((pos_node, RDF.type, GEOM_COORD.PositionCoordinate))
                    self.graph.add((pos_node, GEOM_REL.of, instance_node))
                    self.graph.add((pos_node, GEOM_REL["with-respect-to"], world_node))
                    self.graph.add((pos_node, GEOM_COORD["as-seen-by"], world_node))
                    self.graph.add((pos_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Position))
                    self.graph.add((pos_node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
                elif entry_type == "EnvironmentOrientationEntry":
                    if is_attachment_instance:
                        orient_node = URIRef(f"{instance.uri}.attach-orientation")
                        self._emit_orientation_rpy(orient_node, entry.value, world_node)
                        self.graph.add((instance_node, MJ["attach-orientation"], orient_node))
                        continue
                    orient_node = URIRef(f"{instance.uri}.orientation")
                    self.graph.add((orient_node, RDF.type, GEOM_REL.Orientation))
                    self.graph.add((orient_node, RDF.type, GEOM_COORD.OrientationCoordinate))
                    self.graph.add((orient_node, GEOM_REL.of, instance_node))
                    self.graph.add((orient_node, GEOM_REL["with-respect-to"], world_node))
                    self.graph.add((orient_node, GEOM_COORD["as-seen-by"], world_node))
                elif entry_type == "EnvironmentFreeEntry":
                    if bool(entry.value):
                        self.graph.add((instance_node, RDF.type, GEOM_ENT.RigidBody))
                elif entry_type == "EnvironmentAttachTargetEntry":
                    attach_target_node = URIRef(f"{instance.uri}.attach-to")
                    if entry.kind == "site":
                        self.graph.add((attach_target_node, RDF.type, MJ.MuJoCoSite))
                        self.graph.add((attach_target_node, MJ["site-name"], Literal(entry.name)))
                    else:
                        self.graph.add((attach_target_node, RDF.type, MJ.MuJoCoBody))
                        self.graph.add((attach_target_node, MJ["body-name"], Literal(entry.name)))
                    self.graph.add((instance_node, MJ["attach-kind"], Literal(entry.kind)))
                    self.graph.add((instance_node, MJ["attach-name"], Literal(entry.name)))
                    self.graph.add((instance_node, SLV["attached-to"], URIRef(entry.target_assembly.uri)))
                    if is_attachment_instance:
                        self.graph.add((instance_node, MJ["attach-to-body"], attach_target_node))
                elif entry_type == "EnvironmentToolBodyEntry":
                    if is_attachment_instance:
                        body_node = URIRef(f"{instance.uri}.tool-body")
                        self.graph.add((body_node, RDF.type, MJ.MuJoCoBody))
                        self.graph.add((body_node, MJ["body-name"], Literal(entry.value)))
                        self.graph.add((instance_node, MJ["tool-body"], body_node))
                        continue
                    self._emit_runtime_chain_binding(
                        instance_node,
                        root_name=instance.root,
                        end_name=instance.end,
                        tool_body=entry.value,
                    )
                elif entry_type == "EnvironmentTcpSiteEntry":
                    if is_attachment_instance:
                        site_node = URIRef(f"{instance.uri}.tcp-site")
                        self.graph.add((site_node, RDF.type, MJ.MuJoCoSite))
                        self.graph.add((site_node, MJ["site-name"], Literal(entry.value)))
                        self.graph.add((instance_node, MJ["tcp-site"], site_node))
                        continue
                    self._emit_runtime_chain_binding(
                        instance_node,
                        root_name=instance.root,
                        end_name=instance.end,
                        tcp_site=entry.value,
                    )
                elif entry_type == "EnvironmentAttachmentPrefixEntry":
                    if is_attachment_instance:
                        self.graph.add((instance_node, MJ["attach-prefix"], Literal(entry.value)))
                    else:
                        self.graph.add((instance_node, MJ["prefix"], Literal(entry.value)))
                elif entry_type == "EnvironmentAttachmentActuatorEntry":
                    self.graph.add((instance_node, MJ["actuator-name"], Literal(entry.value)))
                elif entry_type == "EnvironmentChainEntry":
                    self._emit_runtime_chain_binding(
                        instance_node,
                        root_name=entry.root,
                        end_name=entry.end,
                    )
                elif entry_type == "EnvironmentShapeEntry":
                    if str(entry.value).lower() == "box":
                        self.graph.add((instance_node, RDF.type, POLY.CuboidWithSize))
                    else:
                        self.graph.add((instance_node, MJ["shape"], Literal(entry.value)))
                elif entry_type == "EnvironmentSizeEntry":
                    values = {term.axis: term.value for term in entry.value.terms}
                    missing = [axis for axis in ("x", "y", "z") if axis not in values]
                    if missing:
                        raise ValueError(
                            f"Scene object '{instance.name}' has size block missing axes "
                            f"{missing}; all of x, y, z must be specified."
                        )
                    self.graph.add((instance_node, POLY["x-size"], Literal(float(values["x"]), datatype=XSD.double)))
                    self.graph.add((instance_node, POLY["y-size"], Literal(float(values["y"]), datatype=XSD.double)))
                    self.graph.add((instance_node, POLY["z-size"], Literal(float(values["z"]), datatype=XSD.double)))
                elif entry_type == "EnvironmentMassEntry":
                    mass_node = URIRef(f"{instance.uri}.mass")
                    self.graph.add((mass_node, RDF.type, RBDYN_ENT.Mass))
                    self.graph.add((mass_node, RBDYN_ENT["of-body"], instance_node))
                    self.graph.add((mass_node, RBDYN_ENT.mass, Literal(float(entry.value), datatype=XSD.double)))
                elif entry_type == "EnvironmentColorEntry":
                    values = {term.channel: term.value for term in entry.value.terms}
                    missing = [ch for ch in ("r", "g", "b", "a") if ch not in values]
                    if missing:
                        raise ValueError(
                            f"Scene object '{instance.name}' has color block missing channels "
                            f"{missing}; all of r, g, b, a must be specified."
                        )
                    self._emit_color_rgba(
                        instance_node, values["r"], values["g"], values["b"], values["a"]
                    )
                elif entry_type == "EnvironmentFrictionEntry":
                    values = {term.axis: term.value for term in entry.value.terms}
                    missing = [axis for axis in ("slide", "torsion", "roll") if axis not in values]
                    if missing:
                        raise ValueError(
                            f"Scene object '{instance.name}' has friction block missing axes "
                            f"{missing}; all of slide, torsion, roll must be specified."
                        )
                    self.graph.add((instance_node, MJ["friction-slide"], Literal(float(values["slide"]), datatype=XSD.double)))
                    self.graph.add((instance_node, MJ["friction-torsion"], Literal(float(values["torsion"]), datatype=XSD.double)))
                    self.graph.add((instance_node, MJ["friction-roll"], Literal(float(values["roll"]), datatype=XSD.double)))

    def _namespace_owner(self, obj: Any | None) -> Any:
        """Return the namespace declaration that should own a generated node."""
        current = obj
        while current is not None:
            if hasattr(current, "ns") and hasattr(current, "ns_prefix"):
                return current
            current = getattr(current, "parent", None)
        if self._default_ns_owner is None:
            raise ValueError("Cannot create generated URI without a namespace owner.")
        return self._default_ns_owner

    def _owned_uri(self, name: str, owner: Any | None) -> URIRef:
        """Create a URI in the nearest namespace owned by owner or its parents."""
        ns_owner = self._namespace_owner(owner)
        return URIRef(Namespace(ns_owner.ns.uri)[name])

    def root_uri(self, name: str, *, owner: Any | None = None) -> URIRef:
        """Public wrapper for generated URIs kept for callers and tests."""
        return self._owned_uri(name, owner)

    def node(self, value: Any, *, owner: Any | None = None) -> URIRef:
        if hasattr(value, "uri"):
            return URIRef(str(value.uri))
        return self._owned_uri(_node_name(value), owner)

    def _collect_world_quantities(
        self, motion: MotionSpec, handler: ConstraintHandler
    ) -> dict[str, WorldQuantity]:
        qtys: dict[str, WorldQuantity] = {}
        for ctx in motion.context:
            ctx = self._resolved_context_decl(ctx)
            if isinstance(ctx, WorldContextDecl):
                for item in ctx.declaration:
                    if isinstance(item, WorldQuantity):
                        qtys[item.name] = item
        for ctx in getattr(handler, "context", []):
            ctx = self._resolved_context_decl(ctx)
            if isinstance(ctx, WorldContextDecl):
                for item in ctx.declaration:
                    if isinstance(item, WorldQuantity):
                        qtys.setdefault(item.name, item)
        return qtys

    @staticmethod
    def _resolved_context_decl(ctx: Any) -> Any:
        if isinstance(ctx, ContextDeclReference):
            return ctx.ref
        return ctx

    def _collect_context_quantities(
        self, motion: MotionSpec, handler: ConstraintHandler
    ) -> dict[str, ContextQuantity]:
        quantities: dict[str, ContextQuantity] = {}
        for ctx in motion.context:
            ctx = self._resolved_context_decl(ctx)
            if isinstance(ctx, (PreContextDecl, SpecContextDecl, PostContextDecl)):
                for item in ctx.declaration:
                    if isinstance(item, ContextQuantity):
                        quantities[item.name] = item
        for ctx in getattr(handler, "context", []):
            ctx = self._resolved_context_decl(ctx)
            if isinstance(ctx, SpecContextDecl):
                for item in ctx.declaration:
                    if isinstance(item, ContextQuantity):
                        quantities.setdefault(item.name, item)
        for constraint in _resolved_constraint_items(motion):
            expr = constraint.expr
            refs: list[ContextRef] = []
            if isinstance(expr, EqualityConstraint):
                refs = [expr.reference]
            elif isinstance(expr, (GreaterThanConstraint, LessThanConstraint)):
                refs = [expr.threshold]
            elif isinstance(expr, BilateralConstraint):
                refs = [expr.lower, expr.upper]
            for ref in refs:
                quantity = _context_quantity(ref)
                if isinstance(quantity, ContextQuantity):
                    quantities.setdefault(quantity.name, quantity)
        for solver in getattr(handler, "solvers", []):
            s = _resolved_solver(solver)
            gv = getattr(s, "gravity_value", None)
            if gv is not None:
                quantity = _context_quantity(gv)
                if isinstance(quantity, ContextQuantity):
                    quantities.setdefault(quantity.name, quantity)
        return quantities

    def _resolve_qty(self, ref: Any, world_qtys: dict[str, WorldQuantity]) -> WorldQuantity | None:
        if isinstance(ref, WorldQuantity):
            return ref
        return world_qtys.get(_node_name(ref))

    def _resolve_constraint_quantity(
        self,
        spec: ConstraintSpecification,
        world_qtys: dict[str, WorldQuantity],
    ) -> WorldQuantity | None:
        if _is_distance_view(spec):
            return self._resolve_distance_pose(spec, world_qtys)
        return self._resolve_qty(spec.view.quantity, world_qtys)

    def _resolve_distance_pose(
        self,
        spec: ConstraintSpecification,
        world_qtys: dict[str, WorldQuantity],
    ) -> WorldQuantity:
        return self._distance_relative_pose_plan(spec, world_qtys).target

    def _pose_frames(self, quantity: WorldQuantity, context: str) -> tuple[str, str]:
        if quantity.type != WorldQuantityType.Pose or not isinstance(
            quantity.props, GeometricProps
        ):
            raise ValueError(f"{context} needs Pose quantities with explicit endpoints.")
        of_frame = _geo_prop(quantity.props, "of")
        wrt_frame = _geo_prop(quantity.props, "wrt")
        if of_frame is None or wrt_frame is None:
            raise ValueError(f"{context} needs explicit 'of' and 'wrt' frames.")
        return of_frame, wrt_frame

    def _distance_relative_pose_plan(
        self,
        spec: ConstraintSpecification,
        world_qtys: dict[str, WorldQuantity],
    ) -> _RelativePosePlan:
        start = self._resolve_qty(spec.view.distance_from, world_qtys)
        end = self._resolve_qty(spec.view.distance_to, world_qtys)
        if start is None or end is None:
            raise ValueError(f"Distance constraint '{spec.name}' references an unknown pose.")
        if start.type != WorldQuantityType.Pose or end.type != WorldQuantityType.Pose:
            raise ValueError(f"Distance constraint '{spec.name}' must reference Pose quantities.")
        return self._relative_pose_plan(
            start=start,
            end=end,
            world_qtys=world_qtys,
            owner=getattr(getattr(spec, "parent", None), "parent", None),
            cache_key=(id(spec), id(start), id(end)),
            context=f"Distance constraint '{spec.name}'",
        )

    def _relative_pose_plan(
        self,
        *,
        start: WorldQuantity,
        end: WorldQuantity,
        world_qtys: dict[str, WorldQuantity],
        owner: Any,
        cache_key: tuple[Any, ...],
        context: str,
    ) -> _RelativePosePlan:
        cached = self._relative_pose_plans.get(cache_key)
        if cached is not None:
            return cached

        start_frame, start_reference = self._pose_frames(start, context)
        end_frame, end_reference = self._pose_frames(end, context)

        target: WorldQuantity | None = None
        for quantity in world_qtys.values():
            if quantity.type != WorldQuantityType.Pose or not isinstance(
                quantity.props, GeometricProps
            ):
                continue
            of_frame = _geo_prop(quantity.props, "of")
            wrt_frame = _geo_prop(quantity.props, "wrt")
            if (of_frame, wrt_frame) == (end_frame, start_frame):
                target = quantity
                break

        if target is None:
            props = GeometricProps(
                [
                    GeoPropPair(GeometricPropKey.Of, end_frame),
                    GeoPropPair(GeometricPropKey.Wrt, start_frame),
                    GeoPropPair(GeometricPropKey.AsSeenBy, start_frame),
                ]
            )
            target = WorldQuantity(
                parent=owner,
                name=f"pose-{end_frame}-{start_frame}",
                type=WorldQuantityType.Pose,
                props=props,
            )

        reference_path = self._find_pose_path(world_qtys, start_reference, end_reference)
        if reference_path is None:
            raise ValueError(
                f"{context} needs a transform path from '{start_reference}' to '{end_reference}'."
            )

        plan = _RelativePosePlan(start, end, target, reference_path)
        self._relative_pose_plans[cache_key] = plan
        return plan

    def _find_pose_path(
        self,
        world_qtys: dict[str, WorldQuantity],
        start_frame: str,
        end_frame: str,
    ) -> tuple[_PosePathStep, ...] | None:
        if start_frame == end_frame:
            return ()

        edges: dict[str, list[tuple[str, _PosePathStep]]] = {}
        for quantity in world_qtys.values():
            if quantity.type != WorldQuantityType.Pose or not isinstance(
                quantity.props, GeometricProps
            ):
                continue
            of_frame = _geo_prop(quantity.props, "of")
            wrt_frame = _geo_prop(quantity.props, "wrt")
            if of_frame is None or wrt_frame is None:
                continue
            edges.setdefault(wrt_frame, []).append((of_frame, _PosePathStep(quantity, False)))
            edges.setdefault(of_frame, []).append((wrt_frame, _PosePathStep(quantity, True)))

        queue: deque[tuple[str, tuple[_PosePathStep, ...]]] = deque([(start_frame, ())])
        seen = {start_frame}
        while queue:
            frame, path = queue.popleft()
            for next_frame, step in edges.get(frame, []):
                if next_frame in seen:
                    continue
                next_path = (*path, step)
                if next_frame == end_frame:
                    return next_path
                seen.add(next_frame)
                queue.append((next_frame, next_path))
        return None

    def _emit_pose_path_step(
        self,
        step: _PosePathStep,
        context: str,
        motion: MotionSpec,
    ) -> tuple[URIRef, str, str]:
        of_frame, wrt_frame = self._pose_frames(step.quantity, context)
        if not step.inverted:
            return URIRef(step.quantity.uri), of_frame, wrt_frame

        inverse_id = f"inverse-{step.quantity.name}"
        inverse_node = self._owned_uri(inverse_id, motion)
        self._emit_pose_coordinate(inverse_node, wrt_frame, of_frame, motion)
        invert_node = self._owned_uri(f"compute-{inverse_id}", motion)
        self.graph.add((invert_node, RDF.type, GEOM_OP.InvertPose))
        self.graph.add((invert_node, GEOM_OP.pose, URIRef(step.quantity.uri)))
        self.graph.add((invert_node, GEOM_OP.out, inverse_node))
        return inverse_node, wrt_frame, of_frame

    def _emit_pose_composition(
        self,
        *,
        op_id: str,
        in1: URIRef,
        in2: URIRef,
        out: URIRef,
        of_frame: str,
        wrt_frame: str,
        motion: MotionSpec,
    ) -> None:
        self._emit_pose_coordinate(out, of_frame, wrt_frame, motion)
        compose_node = self._owned_uri(op_id, motion)
        self.graph.add((compose_node, RDF.type, GEOM_OP.ComposePose))
        self.graph.add((compose_node, GEOM_OP.in1, in1))
        self.graph.add((compose_node, GEOM_OP.in2, in2))
        self.graph.add((compose_node, GEOM_OP.composite, out))

    def _emit_relative_pose(
        self,
        plan: _RelativePosePlan,
        motion: MotionSpec,
        *,
        context: str,
        stem: str,
    ) -> URIRef:
        start_of, start_wrt = self._pose_frames(plan.start, context)
        end_of, end_wrt = self._pose_frames(plan.end, context)

        current_node: URIRef | None = None
        current_wrt = start_wrt
        for step_index, step in enumerate(plan.reference_path):
            step_node, step_of, step_wrt = self._emit_pose_path_step(step, context, motion)
            if current_node is None:
                current_node = step_node
                current_wrt = step_wrt
                continue

            composed_node = self._owned_uri(
                f"compose-ref-path-{stem}-{step_index}",
                motion,
            )
            self._emit_pose_composition(
                op_id=f"compute-compose-ref-path-{stem}-{step_index}",
                in1=current_node,
                in2=step_node,
                out=composed_node,
                of_frame=step_of,
                wrt_frame=current_wrt,
                motion=motion,
            )
            current_node = composed_node

        end_in_start_ref_node = URIRef(plan.end.uri)
        if current_node is not None:
            end_in_start_ref_node = self._owned_uri(
                f"pose-{plan.end.name}-in-{start_wrt}-{stem}",
                motion,
            )
            self._emit_pose_composition(
                op_id=f"compute-{plan.end.name}-in-{start_wrt}-{stem}",
                in1=current_node,
                in2=URIRef(plan.end.uri),
                out=end_in_start_ref_node,
                of_frame=end_of,
                wrt_frame=current_wrt,
                motion=motion,
            )
        elif end_wrt != start_wrt:
            raise ValueError(f"{context} needs a transform path from '{start_wrt}' to '{end_wrt}'.")

        inverse_id = f"inverse-{plan.start.name}"
        inverse_node = self._owned_uri(inverse_id, motion)
        self._emit_pose_coordinate(inverse_node, start_wrt, start_of, motion)
        invert_node = self._owned_uri(f"compute-{inverse_id}", motion)
        self.graph.add((invert_node, RDF.type, GEOM_OP.InvertPose))
        self.graph.add((invert_node, GEOM_OP.pose, URIRef(plan.start.uri)))
        self.graph.add((invert_node, GEOM_OP.out, inverse_node))

        target_node = URIRef(plan.target.uri)
        self._emit_pose_composition(
            op_id=f"compute-{plan.target.name}",
            in1=inverse_node,
            in2=end_in_start_ref_node,
            out=target_node,
            of_frame=end_of,
            wrt_frame=start_of,
            motion=motion,
        )
        return target_node

    def _emit_frame_pose(
        self,
        *,
        world_qtys: dict[str, WorldQuantity],
        of_frame: str,
        wrt_frame: str,
        motion: MotionSpec,
        context: str,
        stem: str,
    ) -> URIRef:
        path = self._find_pose_path(world_qtys, wrt_frame, of_frame)
        if path is None:
            raise ValueError(
                f"{context} needs a transform path from '{wrt_frame}' to '{of_frame}'."
            )
        if not path:
            raise ValueError(f"{context} requested an identity transform.")

        current_node: URIRef | None = None
        current_wrt = wrt_frame
        for step_index, step in enumerate(path):
            step_node, step_of, step_wrt = self._emit_pose_path_step(step, context, motion)
            if current_node is None:
                current_node = step_node
                current_wrt = step_wrt
                continue

            composed_node = self._owned_uri(
                f"pose-frame-path-{stem}-{step_index}",
                motion,
            )
            self._emit_pose_composition(
                op_id=f"compute-pose-frame-path-{stem}-{step_index}",
                in1=current_node,
                in2=step_node,
                out=composed_node,
                of_frame=step_of,
                wrt_frame=current_wrt,
                motion=motion,
            )
            current_node = composed_node

        if current_node is None:
            raise ValueError(f"{context} did not produce a transform path.")
        return current_node

    def _emit_pose_coordinate(
        self, node: URIRef, of_frame: str, wrt_frame: str, owner: Any
    ) -> None:
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((node, RDF.type, GEOM_REL.Pose))
        self.graph.add((node, RDF.type, VALUE_ROLE.Computed))
        self.graph.add((node, RDF.type, GEOM_COORD.PoseCoordinate))
        self.graph.add((node, RDF.type, GEOM_COORD.DirectionCosineXYZ))
        self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.PlaneAngle))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Length))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.UNITLESS))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        self.graph.add((node, GEOM_REL.of, self._owned_uri(of_frame, owner)))
        self.graph.add((node, GEOM_REL["with-respect-to"], self._owned_uri(wrt_frame, owner)))
        self.graph.add((node, GEOM_COORD["as-seen-by"], self._owned_uri(wrt_frame, owner)))

    def _emit_snapshot_position_metadata(
        self,
        node: URIRef,
        quantity: ContextQuantity,
    ) -> None:
        # A snapshot of <pose>.position needs Position-coordinate RDF metadata
        # so the IR parser surfaces it as a Position record (mapping to
        # KDL::Vector in shared) instead of a generic Quantity (double).
        source = getattr(quantity.value, "source", None)
        source_qty = getattr(source, "quantity", None) if source is not None else None
        if not isinstance(source_qty, WorldQuantity) or not isinstance(
            source_qty.props, GeometricProps
        ):
            return
        of_frame = _geo_prop(source_qty.props, "of")
        wrt_frame = _geo_prop(source_qty.props, "wrt")
        if of_frame is None or wrt_frame is None:
            return
        as_seen_by = _geo_prop(source_qty.props, "as-seen-by") or wrt_frame
        self.graph.add((node, RDF.type, GEOM_REL.Position))
        self.graph.add((node, RDF.type, GEOM_COORD.PositionCoordinate))
        self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
        self.graph.add((node, GEOM_REL.of, self._owned_uri(of_frame, source_qty)))
        self.graph.add(
            (node, GEOM_REL["with-respect-to"], self._owned_uri(wrt_frame, source_qty))
        )
        self.graph.add(
            (node, GEOM_COORD["as-seen-by"], self._owned_uri(as_seen_by, source_qty))
        )

    def _emit_declared_pose_frame_metadata(
        self,
        node: URIRef,
        quantity: ContextQuantity,
        constraints: list[ConstraintSpecification] | None = None,
        world_qtys: dict[str, WorldQuantity] | None = None,
    ) -> None:
        props = quantity.props if isinstance(quantity.props, GeometricProps) else None
        if props is not None:
            of_frame = _geo_prop(props, "of")
            wrt_frame = _geo_prop(props, "wrt")
            if of_frame is None or wrt_frame is None:
                return
            self.graph.add((node, GEOM_REL.of, self._owned_uri(of_frame, quantity)))
            self.graph.add((node, GEOM_REL["with-respect-to"], self._owned_uri(wrt_frame, quantity)))
            self.graph.add((node, GEOM_COORD["as-seen-by"], self._owned_uri(wrt_frame, quantity)))
            return

        if not constraints or not world_qtys:
            return
        world_qty = self._find_constraining_world_pose(quantity, constraints, world_qtys)
        if world_qty is None or not isinstance(world_qty.props, GeometricProps):
            return
        of_frame = _geo_prop(world_qty.props, "of")
        wrt_frame = _geo_prop(world_qty.props, "wrt")
        if of_frame is None or wrt_frame is None:
            return
        self.graph.add((node, GEOM_REL.of, self._owned_uri(of_frame, world_qty)))
        self.graph.add((node, GEOM_REL["with-respect-to"], self._owned_uri(wrt_frame, world_qty)))
        self.graph.add((node, GEOM_COORD["as-seen-by"], self._owned_uri(wrt_frame, world_qty)))

    def _find_constraining_world_pose(
        self,
        quantity: ContextQuantity,
        constraints: list[ConstraintSpecification],
        world_qtys: dict[str, WorldQuantity],
    ) -> WorldQuantity | None:
        # Case 1: quantity is a Snapshot of a world Pose — inherit its source's frames.
        if isinstance(quantity.value, SnapshotValue):
            source_qty = getattr(quantity.value.source, "quantity", None)
            if source_qty is not None:
                world_qty = world_qtys.get(getattr(source_qty, "name", ""))
                if world_qty is not None and world_qty.type == WorldQuantityType.Pose:
                    return world_qty

        # Case 2: quantity appears as reference-value in an equality constraint.
        for spec in constraints:
            if not isinstance(spec.expr, EqualityConstraint):
                continue
            ref = spec.expr.reference
            ref_qty = _context_quantity(ref)
            if ref_qty is None:
                continue
            if getattr(ref_qty, "name", None) != quantity.name:
                continue
            view_qty = self._resolve_qty(spec.view.quantity, world_qtys)
            if view_qty is not None and view_qty.type == WorldQuantityType.Pose:
                return view_qty

        # Case 3: quantity is the Lerp goal/start of a trajectory that is constrained against a world Pose.
        for spec in constraints:
            if not isinstance(spec.expr, EqualityConstraint):
                continue
            ref = spec.expr.reference
            ref_qty = _context_quantity(ref)
            if ref_qty is None or not isinstance(getattr(ref_qty, "value", None), TrajectoryValue):
                continue
            lerp = ref_qty.value.lerp
            lerp_start = _context_quantity(lerp.start)
            lerp_goal = _context_quantity(lerp.goal)
            if (getattr(lerp_start, "name", None) == quantity.name or
                    getattr(lerp_goal, "name", None) == quantity.name):
                view_qty = self._resolve_qty(spec.view.quantity, world_qtys)
                if view_qty is not None and view_qty.type == WorldQuantityType.Pose:
                    return view_qty

        return None

    @staticmethod
    def _context_pose_frames(quantity: ContextQuantity | None) -> tuple[str | None, str | None]:
        if quantity is None or not isinstance(quantity.props, GeometricProps):
            return None, None
        return _geo_prop(quantity.props, "of"), _geo_prop(quantity.props, "wrt")

    def _transform_pose_reference_to_world_frame(
        self,
        *,
        ref_quantity: ContextQuantity | None,
        ref_node: URIRef,
        target_quantity: WorldQuantity,
        world_qtys: dict[str, WorldQuantity],
        motion: MotionSpec,
        stem: str,
    ) -> URIRef:
        # Only applies to full-pose equality constraints (subspace == "pose", axis is None).
        # Subspace constraints (.position.x, .orientation) with a mismatched-frame trajectory
        # reference are intentionally out of scope and are left unchanged by callers.
        source_of, source_wrt = self._context_pose_frames(ref_quantity)
        target_of, target_wrt = self._pose_frames(target_quantity, f"Constraint '{stem}'")
        if source_of is None or source_wrt is None:
            return ref_node
        if source_of != target_of:
            raise ValueError(
                f"Constraint '{stem}' cannot compare pose of '{target_of}' to reference of '{source_of}'."
            )
        if source_wrt == target_wrt:
            return ref_node

        path = self._find_pose_path(world_qtys, target_wrt, source_wrt)
        if path is None:
            raise ValueError(
                f"Constraint '{stem}' needs a transform path from '{target_wrt}' to '{source_wrt}'."
            )
        if not path:
            return ref_node

        current_node: URIRef | None = None
        current_wrt = target_wrt
        for step_index, step in enumerate(path):
            step_node, step_of, step_wrt = self._emit_pose_path_step(step, f"Constraint '{stem}'", motion)
            if current_node is None:
                current_node = step_node
                current_wrt = step_wrt
                continue
            composed_node = self._owned_uri(f"pose-ref-frame-path-{stem}-{step_index}", motion)
            self._emit_pose_composition(
                op_id=f"compute-pose-ref-frame-path-{stem}-{step_index}",
                in1=current_node,
                in2=step_node,
                out=composed_node,
                of_frame=step_of,
                wrt_frame=current_wrt,
                motion=motion,
            )
            current_node = composed_node

        assert current_node is not None
        out_node = self._owned_uri(f"{getattr(ref_quantity, 'name', 'ref')}-in-{target_wrt}-{stem}", motion)
        self._emit_pose_composition(
            op_id=f"compute-{getattr(ref_quantity, 'name', 'ref')}-in-{target_wrt}-{stem}",
            in1=current_node,
            in2=ref_node,
            out=out_node,
            of_frame=source_of,
            wrt_frame=target_wrt,
            motion=motion,
        )
        return out_node

    def _force_control_signal_node(
        self, ctrl: ControllerEntry, handler: ConstraintHandler
    ) -> URIRef:
        signal_node = self._owned_uri(f"force-{ctrl.name}", handler)
        self._add_quantity(signal_node, QuantityType.Force)
        return signal_node

    def _emit_direction_coordinate(
        self,
        node: URIRef,
        as_seen_by: URIRef,
        vector: tuple[float, float, float] | None = None,
    ) -> None:
        self.graph.add((node, RDF.type, GEOM_REL.Direction))
        self.graph.add((node, RDF.type, GEOM_COORD.DirectionCoordinate))
        self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Direction))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.UNITLESS))
        self.graph.add((node, GEOM_COORD["as-seen-by"], as_seen_by))
        if vector is not None:
            x, y, z = vector
            self.graph.add((node, GEOM_COORD.x, Literal(float(x), datatype=XSD.double)))
            self.graph.add((node, GEOM_COORD.y, Literal(float(y), datatype=XSD.double)))
            self.graph.add((node, GEOM_COORD.z, Literal(float(z), datatype=XSD.double)))

    def _emit_zero_position_coordinate(
        self,
        node: URIRef,
        point_node: URIRef,
        as_seen_by: URIRef,
    ) -> None:
        self.graph.add((point_node, RDF.type, GEOM_ENT.Point))
        self.graph.add((node, RDF.type, GEOM_REL.Position))
        self.graph.add((node, RDF.type, GEOM_COORD.PositionCoordinate))
        self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
        self.graph.add((node, RDF.type, QUDT_QKIND.Position))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Position))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        self.graph.add((node, GEOM_REL.of, point_node))
        self.graph.add((node, GEOM_REL["with-respect-to"], point_node))
        self.graph.add((node, GEOM_COORD["as-seen-by"], as_seen_by))
        self.graph.add((node, GEOM_COORD.x, Literal(0.0, datatype=XSD.double)))
        self.graph.add((node, GEOM_COORD.y, Literal(0.0, datatype=XSD.double)))
        self.graph.add((node, GEOM_COORD.z, Literal(0.0, datatype=XSD.double)))

    def _emit_wrench_coordinate(
        self,
        node: URIRef,
        reference_point: URIRef,
        as_seen_by: URIRef,
    ) -> None:
        self.graph.add((node, RDF.type, RBDYN_ENT.Wrench))
        self.graph.add((node, RDF.type, RBDYN_COORD.WrenchCoordinate))
        self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Torque))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Force))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M"]))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.N))
        self.graph.add((node, RBDYN_ENT["reference-point"], reference_point))
        self.graph.add((node, RBDYN_COORD["as-seen-by"], as_seen_by))

    def _emit_force_command_wrench(
        self,
        ctrl: ControllerEntry,
        spec: ConstraintSpecification,
        qty: WorldQuantity,
        axis: str | None,
        magnitude_node: URIRef,
        motion: MotionSpec,
    ) -> URIRef:
        apply_at = getattr(ctrl, "apply_at", None)
        if apply_at is None or not hasattr(apply_at, "uri"):
            raise ValueError(f"Force controller '{ctrl.name}' must specify 'apply at <link>'.")

        props = qty.props if isinstance(qty.props, GeometricProps) else None
        as_seen_by_name = _geo_prop(props, "as-seen-by") or _geo_prop(props, "wrt")
        if as_seen_by_name is None:
            raise ValueError(
                f"Force controller '{ctrl.name}' needs a frame from the constrained quantity."
            )
        as_seen_by_node = self._owned_uri(as_seen_by_name, qty)

        direction_node = self._owned_uri(f"direction-{ctrl.name}", motion)
        if qty.type == WorldQuantityType.Pose and _view_subspace(spec) == "distance" and axis is None:
            self._emit_direction_coordinate(direction_node, as_seen_by_node)
            op_node = self._owned_uri(f"compute-direction-{ctrl.name}", motion)
            self.graph.add((op_node, RDF.type, GEOM_OP.PoseToDirection))
            self.graph.add((op_node, GEOM_OP.pose, URIRef(qty.uri)))
            self.graph.add((op_node, GEOM_OP.direction, direction_node))
        else:
            if axis is None:
                raise ValueError(
                    f"Force controller '{ctrl.name}' needs an axis or a distance pose."
                )
            self._emit_direction_coordinate(direction_node, as_seen_by_node, _axis_vector(axis))

        point_node = self._owned_uri(f"point-force-{ctrl.name}", motion)
        position_node = self._owned_uri(f"position-force-{ctrl.name}", motion)
        wrench_node = self._owned_uri(f"wrench-force-{ctrl.name}", motion)
        self._emit_zero_position_coordinate(position_node, point_node, as_seen_by_node)
        self._emit_wrench_coordinate(wrench_node, point_node, as_seen_by_node)

        op_node = self._owned_uri(f"compute-wrench-force-{ctrl.name}", motion)
        self.graph.add((op_node, RDF.type, RBDYN_OP.WrenchFromPositionDirectionAndMagnitude))
        self.graph.add((op_node, RBDYN_OP.magnitude, magnitude_node))
        self.graph.add((op_node, RBDYN_OP.direction, direction_node))
        self.graph.add((op_node, RBDYN_OP.position, position_node))
        self.graph.add((op_node, RBDYN_OP.wrench, wrench_node))

        return wrench_node

    def _compute_shared_specs(self, handlers: list[ConstraintHandler]) -> frozenset[int]:
        usage: dict[int, set[str]] = {}
        for handler in handlers:
            motion = handler.motion
            if not isinstance(motion, MotionSpec):
                continue
            motion_id = str(getattr(motion, "uri", id(motion)))
            seen: set[int] = set()
            for spec in _resolved_constraint_items(motion):
                sid = id(spec)
                if sid in seen:
                    continue
                seen.add(sid)
                usage.setdefault(sid, set()).add(motion_id)
        return frozenset(sid for sid, motions in usage.items() if len(motions) > 1)

    def _emit_structural_entities(self, world_qtys: dict[str, WorldQuantity]) -> None:
        explicit_structural_nodes: set[URIRef] = set()
        for qty in world_qtys.values():
            rdf_type = WORLD_STRUCTURE_TYPES.get(qty.type)
            if rdf_type is not None and WORLD_SPECS.get(qty.type) is None:
                for node in {URIRef(qty.uri), self._owned_uri(qty.name, None)}:
                    explicit_structural_nodes.add(node)
                    self.graph.add((node, RDF.type, rdf_type))
                    if qty.type == WorldQuantityType.SceneObject:
                        self.graph.add((node, RDF.type, GEOM_ENT.Frame))
                        self.graph.add((node, RDF.type, GEOM_ENT.Point))
                        self.graph.add((node, RDF.type, GEOM_ENT.SimplicialComplex))

        # Implicit entities inferred from geo props. Emit them both where prop
        # references resolve and in the handler-root namespace.
        implicit: dict[tuple[URIRef, Any], None] = {}

        def add_implicit(name: str | None, rdf_type: Any, owner: Any) -> None:
            if not name:
                return
            owned_node = self._owned_uri(name, owner)
            default_node = self._owned_uri(name, None)
            if owned_node not in explicit_structural_nodes:
                implicit.setdefault((owned_node, rdf_type), None)
            if default_node not in explicit_structural_nodes:
                implicit.setdefault((default_node, rdf_type), None)

        for qty in world_qtys.values():
            props = qty.props if isinstance(qty.props, GeometricProps) else None
            if qty.type == WorldQuantityType.Pose:
                for key in ("of", "wrt", "as-seen-by"):
                    add_implicit(_geo_prop(props, key), GEOM_ENT.Frame, qty)
            elif qty.type == WorldQuantityType.VelocityTwist:
                for key in ("of", "wrt"):
                    add_implicit(_geo_prop(props, key), GEOM_ENT.SimplicialComplex, qty)
                add_implicit(_geo_prop(props, "ref-point"), GEOM_ENT.Point, qty)
                add_implicit(
                    _geo_prop(props, "as-seen-by") or _geo_prop(props, "wrt"),
                    GEOM_ENT.Frame,
                    qty,
                )
            elif qty.type == WorldQuantityType.Wrench:
                add_implicit(_geo_prop(props, "ref-point"), GEOM_ENT.Point, qty)
                add_implicit(_geo_prop(props, "as-seen-by"), GEOM_ENT.Frame, qty)
            elif qty.type == WorldQuantityType.JointPosition:
                add_implicit(_geo_prop(props, "of"), KC.Joint, qty)

        for node, rdf_type in implicit:
            self.graph.add((node, RDF.type, rdf_type))

    def _emit_world_quantities(self, world_qtys: dict[str, WorldQuantity]) -> None:
        for qty in world_qtys.values():
            if qty.type == WorldQuantityType.Gravity:
                node = URIRef(qty.uri)
                self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
                self.graph.add((node, RDF.type, GEOM_ENT.UniformGravitationalField))
                self.graph.add((node, RDF.type, RBDYN_COORD.UniformGravitationalFieldCoordinate))
                props = qty.props if isinstance(qty.props, GeometricProps) else None
                asb_v = _geo_prop(props, "as-seen-by") or _geo_prop(props, "wrt")
                if asb_v:
                    self.graph.add((node, RBDYN_COORD["as-seen-by"], self._owned_uri(asb_v, qty)))
                continue
            spec = WORLD_SPECS.get(qty.type)
            if spec is None:
                continue
            rdf_types, qkinds, units, _ = spec
            node = URIRef(qty.uri)
            for t in rdf_types:
                self.graph.add((node, RDF.type, t))
            self.graph.add((node, RDF.type, VALUE_ROLE.Measured))
            for qk in qkinds:
                self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], qk))
            for u in units:
                self.graph.add((node, QUDT_SCHEMA.unit, u))

            props = qty.props if isinstance(qty.props, GeometricProps) else None
            of_v = _geo_prop(props, "of")
            wrt_v = _geo_prop(props, "wrt")
            rp_v = _geo_prop(props, "ref-point")
            asb_v = _geo_prop(props, "as-seen-by")

            # geo-prop targets: walk up qty → WorldContextDecl → MotionSpec → motion_ns
            if of_v:
                self.graph.add((node, GEOM_REL.of, self._owned_uri(of_v, qty)))
            if wrt_v:
                self.graph.add((node, GEOM_REL["with-respect-to"], self._owned_uri(wrt_v, qty)))
            if rp_v:
                ref_predicate = (
                    RBDYN_ENT["reference-point"]
                    if qty.type == WorldQuantityType.Wrench
                    else GEOM_REL["reference-point"]
                )
                self.graph.add((node, ref_predicate, self._owned_uri(rp_v, qty)))
            elif qty.type in {WorldQuantityType.VelocityTwist, WorldQuantityType.Wrench}:
                ref_predicate = (
                    RBDYN_ENT["reference-point"]
                    if qty.type == WorldQuantityType.Wrench
                    else GEOM_REL["reference-point"]
                )
                point_node = self._owned_uri(f"point-{qty.name}-origin", qty)
                self.graph.add((point_node, RDF.type, GEOM_ENT.Point))
                self.graph.add((node, ref_predicate, point_node))
            if qty.type == WorldQuantityType.Pose and wrt_v:
                self.graph.add((node, GEOM_COORD["as-seen-by"], self._owned_uri(wrt_v, qty)))
            elif asb_v:
                asb_predicate = (
                    RBDYN_COORD["as-seen-by"]
                    if qty.type == WorldQuantityType.Wrench
                    else GEOM_COORD["as-seen-by"]
                )
                self.graph.add((node, asb_predicate, self._owned_uri(asb_v, qty)))
            elif qty.type == WorldQuantityType.VelocityTwist and wrt_v:
                self.graph.add((node, GEOM_COORD["as-seen-by"], self._owned_uri(wrt_v, qty)))

    def _emit_derived_quantity_transforms(
        self,
        motion: MotionSpec,
        world_qtys: dict[str, WorldQuantity],
    ) -> None:
        quantities = list(world_qtys.values())
        for target_index, target in enumerate(quantities):
            if target.type not in {WorldQuantityType.VelocityTwist, WorldQuantityType.Wrench}:
                continue
            target_props = target.props if isinstance(target.props, GeometricProps) else None
            target_as_seen_by = _geo_prop(target_props, "as-seen-by")
            if target_as_seen_by is None:
                continue

            for source in quantities[:target_index]:
                if source.type != target.type:
                    continue
                source_props = source.props if isinstance(source.props, GeometricProps) else None
                source_as_seen_by = _geo_prop(source_props, "as-seen-by")
                if source_as_seen_by is None or source_as_seen_by == target_as_seen_by:
                    continue
                if not self._can_transform_quantity(source, target):
                    continue

                pose_node = self._emit_frame_pose(
                    world_qtys=world_qtys,
                    of_frame=source_as_seen_by,
                    wrt_frame=target_as_seen_by,
                    motion=motion,
                    context=(f"Transform from '{source.name}' to '{target.name}'"),
                    stem=f"{source.name}-to-{target.name}",
                )
                self._emit_quantity_transform_operation(source, target, pose_node, motion)
                break

    def _can_transform_quantity(
        self,
        source: WorldQuantity,
        target: WorldQuantity,
    ) -> bool:
        source_props = source.props if isinstance(source.props, GeometricProps) else None
        target_props = target.props if isinstance(target.props, GeometricProps) else None

        if source.type == WorldQuantityType.VelocityTwist:
            return (
                _geo_prop(source_props, "of") == _geo_prop(target_props, "of")
                and _geo_prop(source_props, "wrt") == _geo_prop(target_props, "wrt")
                and _geo_prop(source_props, "ref-point") == _geo_prop(target_props, "ref-point")
            )

        if source.type == WorldQuantityType.Wrench:
            return _geo_prop(source_props, "of") == _geo_prop(target_props, "of") and _geo_prop(
                source_props, "wrt"
            ) == _geo_prop(target_props, "wrt")

        return False

    def _emit_quantity_transform_operation(
        self,
        source: WorldQuantity,
        target: WorldQuantity,
        pose_node: URIRef,
        motion: MotionSpec,
    ) -> None:
        op_node = self._owned_uri(f"transform-{source.name}-to-{target.name}", motion)
        source_node = URIRef(source.uri)
        target_node = URIRef(target.uri)

        if source.type == WorldQuantityType.VelocityTwist:
            self.graph.add((op_node, RDF.type, GEOM_OP.RotateVelocityTwistToProximalWithPose))
            self.graph.add((op_node, GEOM_OP.pose, pose_node))
            self.graph.add((op_node, GEOM_OP["from"], source_node))
            self.graph.add((op_node, GEOM_OP.to, target_node))
            return

        if source.type == WorldQuantityType.Wrench:
            source_props = source.props if isinstance(source.props, GeometricProps) else None
            target_props = target.props if isinstance(target.props, GeometricProps) else None
            source_ref = _geo_prop(source_props, "ref-point")
            target_ref = _geo_prop(target_props, "ref-point")
            op_type = (
                RBDYN_OP.RotateWrenchToProximalWithPose
                if source_ref == target_ref
                else RBDYN_OP.TransformWrenchToProximal
            )
            self.graph.add((op_node, RDF.type, op_type))
            self.graph.add((op_node, RBDYN_OP.pose, pose_node))
            self.graph.add((op_node, RBDYN_OP["from"], source_node))
            self.graph.add((op_node, RBDYN_OP.to, target_node))

    def _view_node(self, view: Any, owner: Any) -> URIRef:
        if (
            getattr(view, "distance_from", None) is not None
            and getattr(view, "distance_to", None) is not None
        ):
            return self._owned_uri(
                f"distance-{_node_name(view.distance_from)}-{_node_name(view.distance_to)}",
                owner,
            )

        quantity = getattr(view, "quantity", None)
        if isinstance(quantity, WorldQuantity):
            subspace_raw = getattr(view, "subspace", None)
            axis_raw = getattr(view, "axis", None)
            if subspace_raw is None and quantity.type == WorldQuantityType.Pose:
                return URIRef(quantity.uri)
            if subspace_raw is None and quantity.type == WorldQuantityType.JointPosition:
                return URIRef(quantity.uri)
            subspace = str(getattr(subspace_raw, "value", subspace_raw))
            if (
                quantity.type == WorldQuantityType.Pose
                and subspace in {"position", "orientation"}
                and axis_raw is None
            ):
                mapped_subspace = subspace
            else:
                mapped_subspace = SUBSPACE_ALIAS.get(subspace, subspace)
            axis = semantic_axis_label(axis_raw)
            scalar_uri = self._owned_uri(_scalar_id(quantity, mapped_subspace, axis), owner)
            if (
                quantity.type == WorldQuantityType.Pose
                and mapped_subspace in {"distance", "rotation"}
                and axis is not None
            ):
                self._register_pose_component_view(scalar_uri, quantity, mapped_subspace, axis, owner)
            elif (
                quantity.type == WorldQuantityType.Pose
                and mapped_subspace == "position"
                and axis is None
            ):
                self._register_pose_position_view(scalar_uri, quantity, owner)
            return scalar_uri

        return self._owned_uri(_node_name(quantity), owner)

    def _register_pose_position_view(
        self,
        scalar_uri: URIRef,
        quantity: "WorldQuantity",
        owner: Any,
    ) -> None:
        # Promote `<pose>.position` to a Position-coordinate node so IR.position()
        # picks it up as a Position record (KDL::Vector in shared) instead of a
        # generic Quantity (double). Also register a MAP.View so access-expr
        # resolves reads of the view to `shared.<pose>.p` at runtime — the
        # subobject field itself is never assigned to. Idempotent.
        if (scalar_uri, RDF.type, GEOM_COORD.PositionCoordinate) in self.graph:
            return
        props = quantity.props if isinstance(quantity.props, GeometricProps) else None
        of_frame = _geo_prop(props, "of") if props is not None else None
        wrt_frame = _geo_prop(props, "wrt") if props is not None else None
        if of_frame is None or wrt_frame is None:
            return
        as_seen_by = _geo_prop(props, "as-seen-by") or wrt_frame
        self.graph.add((scalar_uri, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((scalar_uri, RDF.type, GEOM_REL.Position))
        self.graph.add((scalar_uri, RDF.type, GEOM_COORD.PositionCoordinate))
        self.graph.add((scalar_uri, RDF.type, GEOM_COORD.VectorXYZ))
        self.graph.add((scalar_uri, RDF.type, QUDT_QKIND.Position))
        self.graph.add((scalar_uri, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Position))
        self.graph.add((scalar_uri, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        self.graph.add((scalar_uri, GEOM_REL.of, self._owned_uri(of_frame, quantity)))
        self.graph.add(
            (scalar_uri, GEOM_REL["with-respect-to"], self._owned_uri(wrt_frame, quantity))
        )
        self.graph.add(
            (scalar_uri, GEOM_COORD["as-seen-by"], self._owned_uri(as_seen_by, quantity))
        )

        view_uri = self._owned_uri(f"view-{_scalar_id(quantity, 'position', None)}", owner)
        if (view_uri, RDF.type, MAP.View) not in self.graph:
            self.graph.add((view_uri, RDF.type, MAP.View))
            self.graph.add((view_uri, RDF.type, MAP.PoseCoordinateView))
            self.graph.add((view_uri, MAP.superobject, URIRef(quantity.uri)))
            self.graph.add((view_uri, MAP.subobject, scalar_uri))
            self.graph.add((view_uri, MAP.subspace, MAP.position))
            # axis intentionally omitted: this view exposes the whole 3-vector.

    def _register_pose_component_view(
        self,
        scalar_uri: "URIRef",
        quantity: "WorldQuantity",
        mapped_subspace: str,
        axis: str,
        owner: Any,
    ) -> None:
        sid = _scalar_id(quantity, mapped_subspace, axis)
        view_uri = self._owned_uri(f"view-{sid}", owner)
        if (view_uri, RDF.type, MAP.View) in self.graph:
            return
        pose_specs = WORLD_SPECS[WorldQuantityType.Pose][3]
        props = pose_specs.get(mapped_subspace)
        if props is None:
            return
        view_subspace_uri, _, _, scalar_t, view_type = props
        self._add_quantity(scalar_uri, scalar_t)
        if view_type == MAP.PoseCoordinateView:
            self.graph.add((scalar_uri, RDF.type, QUDT_QKIND.Position))
        self.graph.add((view_uri, RDF.type, MAP.View))
        self.graph.add((view_uri, RDF.type, view_type))
        self.graph.add((view_uri, MAP.superobject, URIRef(quantity.uri)))
        self.graph.add((view_uri, MAP.subobject, scalar_uri))
        self.graph.add((view_uri, MAP.subspace, MAP[view_subspace_uri]))
        self.graph.add((view_uri, MAP.axis, MAP[axis]))

    def _emit_context_quantities(
        self,
        context_quantities: dict[str, ContextQuantity],
        constraints: list[ConstraintSpecification],
        world_qtys: dict[str, WorldQuantity],
    ) -> None:
        for quantity in context_quantities.values():
            node = URIRef(quantity.uri)
            self.graph.add((node, RDF.type, VALUE_ROLE.Declared))
            if quantity.type == QuantityType.Trajectory:
                self._emit_trajectory_quantity(node, quantity, constraints, world_qtys)
                continue
            if quantity.type == QuantityType.Direction:
                self._emit_direction_quantity(node, quantity)
                continue
            if isinstance(quantity.value, PoseValue):
                self._emit_pose_value_quantity(node, quantity, constraints, world_qtys)
                continue
            qkind = QUDT_KIND_BY_QUANTITY_TYPE.get(quantity.type)
            if qkind is None:
                qkind = QUDT_QKIND[quantity.type]
            self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
            self.graph.add((node, RDF.type, qkind))
            self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], qkind))
            if quantity.value is None:
                continue
            if isinstance(quantity.value, SnapshotValue):
                self.graph.add((node, RDF.type, SNAP.Snapshot))
                self.graph.add((node, RDF.type, VALUE_ROLE.Snapshot))
                view_node = self._view_node(quantity.value.source, quantity)
                if quantity.value.offset is not None:
                    offset_ref_node = self._emit_context_ref_node(
                        quantity.value.offset, quantity, "add-offset"
                    )
                    add_node = self._owned_uri(f"{quantity.name}-add", quantity)
                    out_node = self._owned_uri(f"{quantity.name}-add-out", quantity)
                    qkind = QUDT_KIND_BY_QUANTITY_TYPE.get(quantity.type) or QUDT_QKIND[quantity.type]
                    self.graph.add((add_node, RDF.type, RBDYN_OP["AddQuantity"]))
                    self.graph.add((add_node, RBDYN_OP["in1"], view_node))
                    self.graph.add((add_node, RBDYN_OP["in2"], offset_ref_node))
                    self.graph.add((add_node, RBDYN_OP["out"], out_node))
                    self.graph.add((out_node, RDF.type, QUDT_SCHEMA.Quantity))
                    self.graph.add((out_node, RDF.type, VALUE_ROLE.Computed))
                    self.graph.add((out_node, RDF.type, qkind))
                    self.graph.add((out_node, QUDT_SCHEMA["hasQuantityKind"], qkind))
                    self.graph.add((out_node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(quantity.type, QUDT_UNIT.UNITLESS)))
                    snap_source = out_node
                else:
                    snap_source = view_node
                self.graph.add((node, SNAP["snapshot-of"], snap_source))
                self.graph.add(
                    (node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(quantity.type, QUDT_UNIT.UNITLESS))
                )
                if qkind == GEOM_REL.Pose:
                    self._emit_declared_pose_frame_metadata(node, quantity, constraints, world_qtys)
                elif quantity.type == QuantityType.Position:
                    self._emit_snapshot_position_metadata(node, quantity)
                continue
            self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(quantity.value.unit)))
            if isinstance(quantity.value, ScalarQuantity):
                self.graph.add((node, QUDT_SCHEMA.value, Literal(float(quantity.value.value), datatype=XSD.double)))
            elif isinstance(quantity.value, VectorQuantity):
                self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
                self.graph.add((node, GEOM_COORD.x, Literal(float(quantity.value.x), datatype=XSD.double)))
                self.graph.add((node, GEOM_COORD.y, Literal(float(quantity.value.y), datatype=XSD.double)))
                self.graph.add((node, GEOM_COORD.z, Literal(float(quantity.value.z), datatype=XSD.double)))

    def _emit_pose_value_quantity(
        self,
        node: URIRef,
        quantity: ContextQuantity,
        constraints: list[ConstraintSpecification] | None = None,
        world_qtys: dict[str, WorldQuantity] | None = None,
    ) -> None:
        assert isinstance(quantity.value, PoseValue)
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((node, RDF.type, VALUE_ROLE.Declared))
        self.graph.add((node, RDF.type, GEOM_REL.Pose))
        self.graph.add((node, RDF.type, GEOM_COORD.PoseCoordinate))
        self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.PlaneAngle))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Length))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.UNITLESS))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        self._emit_declared_pose_frame_metadata(node, quantity, constraints or [], world_qtys or {})

        position_node = self._owned_uri(f"{quantity.name}.position", quantity)
        orientation_node = self._owned_uri(f"{quantity.name}.orientation", quantity)
        self.graph.add((position_node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((position_node, RDF.type, GEOM_COORD.VectorXYZ))
        self.graph.add((position_node, RDF.type, QUDT_QKIND.Position))
        self.graph.add((position_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Position))
        self.graph.add((position_node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        self.graph.add((orientation_node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((orientation_node, RDF.type, GEOM_REL.Orientation))
        self.graph.add((orientation_node, RDF.type, GEOM_COORD.OrientationCoordinate))
        self.graph.add((orientation_node, RDF.type, GEOM_COORD["EulerAngles"]))
        self.graph.add((orientation_node, GEOM_COORD["axes-sequence"], Literal("xyz")))
        self.graph.add((orientation_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.PlaneAngle))
        self.graph.add((orientation_node, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
        self.graph.add((node, GEOM_COORD["has-coordinate"], position_node))
        self.graph.add((node, GEOM_COORD["has-coordinate"], orientation_node))

        pose_of = self.graph.value(node, GEOM_REL.of)
        pose_wrt = self.graph.value(node, GEOM_REL["with-respect-to"])
        pose_asb = self.graph.value(node, GEOM_COORD["as-seen-by"])
        if pose_of and pose_wrt:
            self.graph.add((orientation_node, GEOM_REL.of, pose_of))
            self.graph.add((orientation_node, GEOM_REL["with-respect-to"], pose_wrt))
            self.graph.add((orientation_node, GEOM_COORD["as-seen-by"], pose_asb or pose_wrt))

        for term in quantity.value.position.terms:
            component_node = self._owned_uri(f"{quantity.name}.position.{term.axis}", quantity)
            view_node = self._owned_uri(f"view-{quantity.name}.position.{term.axis}", quantity)
            self._add_quantity(component_node, QuantityType.Distance)
            self.graph.add((component_node, RDF.type, QUDT_QKIND.Position))
            self.graph.add((position_node, GEOM_COORD["has-coordinate"], component_node))
            self.graph.add((view_node, RDF.type, MAP.View))
            self.graph.add((view_node, RDF.type, MAP.PoseCoordinateView))
            self.graph.add((view_node, MAP.superobject, node))
            self.graph.add((view_node, MAP.subobject, component_node))
            self.graph.add((view_node, MAP.subspace, MAP.position))
            self.graph.add((view_node, MAP.axis, MAP[term.axis]))
            if term.ref is not None:
                ref_node = self._emit_context_ref_node(term.ref, quantity, term.axis)
                self.graph.add((ref_node, RDF.type, VALUE_ROLE.Reference))
                self.graph.add((component_node, CSTR["reference-value"], ref_node))
            else:
                self.graph.add((component_node, QUDT_SCHEMA.value, Literal(float(term.value), datatype=XSD.double)))
                self.graph.add((component_node, QUDT_SCHEMA.unit, _dsl_unit(term.unit)))

        for term in quantity.value.orientation.terms:
            axis = {
                "roll": "x",
                "pitch": "y",
                "yaw": "z",
            }[term.axis]
            component_node = self._owned_uri(f"{quantity.name}.orientation.{axis}", quantity)
            view_node = self._owned_uri(f"view-{quantity.name}.orientation.{axis}", quantity)
            self._add_quantity(component_node, QuantityType.Angle)
            self.graph.add((orientation_node, GEOM_COORD["has-coordinate"], component_node))
            self.graph.add((view_node, RDF.type, MAP.View))
            self.graph.add((view_node, RDF.type, MAP.PoseCoordinateView))
            self.graph.add((view_node, MAP.superobject, node))
            self.graph.add((view_node, MAP.subobject, component_node))
            self.graph.add((view_node, MAP.subspace, MAP.rotation))
            self.graph.add((view_node, MAP.axis, MAP[axis]))
            if term.ref is not None:
                ref_node = self._emit_context_ref_node(term.ref, quantity, term.axis)
                self.graph.add((ref_node, RDF.type, VALUE_ROLE.Reference))
                self.graph.add((component_node, CSTR["reference-value"], ref_node))
            else:
                self.graph.add((component_node, QUDT_SCHEMA.value, Literal(float(term.value), datatype=XSD.double)))
                self.graph.add((component_node, QUDT_SCHEMA.unit, _dsl_unit(term.unit)))

    @staticmethod
    def _lerp_value_kind(lerp) -> Any | None:
        start_qty = _context_quantity(lerp.start)
        goal_qty = _context_quantity(lerp.goal)
        start_kind = QUDT_KIND_BY_QUANTITY_TYPE.get(getattr(start_qty, "type", None))
        goal_kind = QUDT_KIND_BY_QUANTITY_TYPE.get(getattr(goal_qty, "type", None))
        if start_kind is None and goal_kind is None:
            return None
        if start_kind != goal_kind:
            raise ValueError(
                f"Lerp trajectory start type '{getattr(start_qty, 'type', None)}' "
                f"does not match goal type '{getattr(goal_qty, 'type', None)}'"
            )
        return start_kind

    def _emit_direction_quantity(
        self,
        node: URIRef,
        quantity: ContextQuantity,
    ) -> None:
        as_seen_by_name = _geo_prop(quantity.props, "as-seen-by") or _geo_prop(quantity.props, "wrt")
        if as_seen_by_name is None:
            raise ValueError(
                f"Direction quantity '{quantity.name}' needs an 'as-seen-by: <frame>' prop."
            )
        as_seen_by_node = self._owned_uri(as_seen_by_name, quantity)
        vector: tuple[float, float, float] | None = None
        if isinstance(quantity.value, VectorQuantity):
            vector = (float(quantity.value.x), float(quantity.value.y), float(quantity.value.z))
        elif quantity.value is not None:
            raise ValueError(
                f"Direction quantity '{quantity.name}' value must be a Vector literal."
            )
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        self._emit_direction_coordinate(node, as_seen_by_node, vector)

    def _emit_trajectory_quantity(
        self,
        node: URIRef,
        quantity: ContextQuantity,
        constraints: list[ConstraintSpecification] | None = None,
        world_qtys: dict[str, WorldQuantity] | None = None,
    ) -> None:
        assert isinstance(quantity.value, TrajectoryValue)
        value = quantity.value
        if value.lerp is not None:
            self._emit_lerp_trajectory(node, quantity, value.lerp, constraints, world_qtys)
        elif value.circle is not None:
            self._emit_geometric_trajectory(
                node, quantity, value.circle, TRAJ.Circle, "circle",
                [("center", TRAJ.center, value.circle.center),
                 ("radius", TRAJ.radius, value.circle.radius),
                 ("plane-normal", TRAJ["plane-normal"], value.circle.plane_normal),
                 ("orientation", TRAJ.orientation, value.circle.orientation),
                 ("alpha", TRAJ.alpha, value.circle.alpha)],
                constraints, world_qtys,
            )
        elif value.semicircle is not None:
            self._emit_geometric_trajectory(
                node, quantity, value.semicircle, TRAJ.SemiCircle, "semicircle",
                [("start", TRAJ.start, value.semicircle.start),
                 ("end", TRAJ.end, value.semicircle.end),
                 ("radius", TRAJ.radius, value.semicircle.radius),
                 ("plane-normal", TRAJ["plane-normal"], value.semicircle.plane_normal),
                 ("orientation", TRAJ.orientation, value.semicircle.orientation),
                 ("alpha", TRAJ.alpha, value.semicircle.alpha)],
                constraints, world_qtys,
            )
        elif value.helix is not None:
            self._emit_geometric_trajectory(
                node, quantity, value.helix, TRAJ.Helix, "helix",
                [("center", TRAJ.center, value.helix.center),
                 ("radius", TRAJ.radius, value.helix.radius),
                 ("axis", TRAJ.axis, value.helix.axis),
                 ("pitch", TRAJ.pitch, value.helix.pitch),
                 ("revolutions", TRAJ.revolutions, value.helix.revolutions),
                 ("orientation", TRAJ.orientation, value.helix.orientation),
                 ("alpha", TRAJ.alpha, value.helix.alpha)],
                constraints, world_qtys,
            )
        else:
            raise ValueError(
                f"TrajectoryValue on '{quantity.name}' has no populated spec"
            )

    def _emit_trajectory_pose_metadata(
        self,
        node: URIRef,
        quantity: ContextQuantity,
        value_kind: Any | None,
        constraints: list[ConstraintSpecification] | None,
        world_qtys: dict[str, WorldQuantity] | None,
    ) -> None:
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((node, RDF.type, VALUE_ROLE.Declared))
        self.graph.add((node, RDF.type, TRAJ.Trajectory))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], TRAJ.Trajectory))
        if value_kind is not None:
            self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], value_kind))
        if value_kind == GEOM_REL.Pose:
            self.graph.add((node, RDF.type, GEOM_REL.Pose))
            self.graph.add((node, RDF.type, GEOM_COORD.PoseCoordinate))
            self.graph.add((node, RDF.type, GEOM_COORD.DirectionCosineXYZ))
            self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
            self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.PlaneAngle))
            self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Length))
            self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.UNITLESS))
            self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        self._emit_declared_pose_frame_metadata(node, quantity, constraints or [], world_qtys or {})

    def _emit_lerp_trajectory(
        self,
        node: URIRef,
        quantity: ContextQuantity,
        lerp: Any,
        constraints: list[ConstraintSpecification] | None,
        world_qtys: dict[str, WorldQuantity] | None,
    ) -> None:
        lerp_node = self._owned_uri(f"lerp-{quantity.name}", quantity)
        value_kind = self._lerp_value_kind(lerp)
        self._emit_trajectory_pose_metadata(node, quantity, value_kind, constraints, world_qtys)
        self.graph.add((lerp_node, RDF.type, TRAJ.Lerp))
        self.graph.add((lerp_node, TRAJ.start, self._emit_context_ref_node(lerp.start, quantity, "start")))
        self.graph.add((lerp_node, TRAJ.goal, self._emit_context_ref_node(lerp.goal, quantity, "goal")))
        self.graph.add((lerp_node, TRAJ.alpha, self._emit_context_ref_node(lerp.alpha, quantity, "alpha")))
        self.graph.add((lerp_node, TRAJ.trajectory, node))

    def _emit_geometric_trajectory(
        self,
        node: URIRef,
        quantity: ContextQuantity,
        spec: Any,
        spec_type: URIRef,
        spec_prefix: str,
        inputs: list[tuple[str, URIRef, Any]],
        constraints: list[ConstraintSpecification] | None,
        world_qtys: dict[str, WorldQuantity] | None,
    ) -> None:
        # All geometric trajectories output a Pose (KDL::Frame).
        self._emit_trajectory_pose_metadata(node, quantity, GEOM_REL.Pose, constraints, world_qtys)
        op_node = self._owned_uri(f"{spec_prefix}-{quantity.name}", quantity)
        self.graph.add((op_node, RDF.type, spec_type))
        for suffix, predicate, ref in inputs:
            self.graph.add(
                (op_node, predicate, self._emit_context_ref_node(ref, quantity, suffix))
            )
        self.graph.add((op_node, TRAJ.trajectory, node))

    def _emit_context_ref_node(self, ref: ContextRef, owner: Any, suffix: str) -> URIRef:
        quantity = _context_quantity(ref)
        if not isinstance(quantity, ContextQuantity):
            return self._owned_uri(_node_name(quantity), owner)

        if ref.literal_value is None:
            return URIRef(quantity.uri)

        quantity = _resolved_context_quantity(quantity)
        node = self._owned_uri(f"{quantity.name}-{suffix}", owner)
        qkind = QUDT_KIND_BY_QUANTITY_TYPE.get(quantity.type)
        if qkind is None:
            qkind = QUDT_QKIND[quantity.type]

        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((node, RDF.type, qkind))
        if quantity.type == QuantityType.Distance:
            self.graph.add((node, RDF.type, GEOM_COORD.LinearDistanceCoordinate))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], qkind))
        self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(ref.literal_value.unit)))
        if isinstance(ref.literal_value, ScalarQuantity):
            self.graph.add((node, QUDT_SCHEMA.value, Literal(float(ref.literal_value.value), datatype=XSD.double)))
        elif isinstance(ref.literal_value, VectorQuantity):
            self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
            self.graph.add((node, GEOM_COORD.x, Literal(float(ref.literal_value.x), datatype=XSD.double)))
            self.graph.add((node, GEOM_COORD.y, Literal(float(ref.literal_value.y), datatype=XSD.double)))
            self.graph.add((node, GEOM_COORD.z, Literal(float(ref.literal_value.z), datatype=XSD.double)))
        return node

    def _constraint_reference_node(
        self,
        ref: ContextRef,
        owner: Any,
        suffix: str,
        subspace: str,
        axis: str | None,
    ) -> URIRef:
        ref_node = self._emit_context_ref_node(ref, owner, suffix)
        quantity = _context_quantity(ref)
        if not isinstance(quantity, ContextQuantity) or axis is not None:
            return ref_node
        quantity = _resolved_context_quantity(quantity)
        if quantity.type == QuantityType.Pose:
            if subspace == "position":
                position_node = self._owned_uri(f"{quantity.name}.position", quantity)
                self.graph.add((position_node, RDF.type, QUDT_SCHEMA.Quantity))
                self.graph.add((position_node, RDF.type, QUDT_QKIND.Position))
                self.graph.add((position_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Position))
                self.graph.add((position_node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
                return position_node
            if subspace == "orientation":
                orientation_node = self._owned_uri(f"{quantity.name}.orientation", quantity)
                self.graph.add((orientation_node, RDF.type, QUDT_SCHEMA.Quantity))
                self.graph.add((orientation_node, RDF.type, GEOM_REL.Orientation))
                self.graph.add((orientation_node, RDF.type, GEOM_COORD.OrientationCoordinate))
                self.graph.add((orientation_node, RDF.type, GEOM_COORD["EulerAngles"]))
                self.graph.add((orientation_node, GEOM_COORD["axes-sequence"], Literal("xyz")))
                self.graph.add((orientation_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.PlaneAngle))
                self.graph.add((orientation_node, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
                pose_node = URIRef(quantity.uri)
                self.graph.add((pose_node, GEOM_COORD["has-coordinate"], orientation_node))
                pose_of = self.graph.value(pose_node, GEOM_REL.of)
                pose_wrt = self.graph.value(pose_node, GEOM_REL["with-respect-to"])
                pose_asb = self.graph.value(pose_node, GEOM_COORD["as-seen-by"])
                if pose_of and pose_wrt:
                    self.graph.add((orientation_node, GEOM_REL.of, pose_of))
                    self.graph.add((orientation_node, GEOM_REL["with-respect-to"], pose_wrt))
                    self.graph.add((orientation_node, GEOM_COORD["as-seen-by"], pose_asb or pose_wrt))
                return orientation_node
        if quantity.type == QuantityType.Trajectory:
            if subspace == "position":
                self.graph.add((ref_node, RDF.type, QUDT_SCHEMA.Quantity))
                self.graph.add((ref_node, RDF.type, QUDT_QKIND.Position))
                self.graph.add((ref_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Position))
                self.graph.add((ref_node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
            elif subspace == "orientation":
                self.graph.add((ref_node, RDF.type, GEOM_REL.Orientation))
                self.graph.add((ref_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Direction))
        return ref_node

    def _emit_constraints(
        self,
        motion: MotionSpec,
        constraints: list[ConstraintSpecification],
        world_qtys: dict[str, WorldQuantity],
    ) -> None:
        seen_uris: set[str] = set()
        for spec in constraints:
            uri_str = str(spec.uri)
            if uri_str in seen_uris:
                continue
            seen_uris.add(uri_str)

            node = URIRef(spec.uri)
            qty = self._resolve_constraint_quantity(spec, world_qtys)
            if qty is None:
                raise ValueError(
                    f"Constraint '{spec.name}' does not resolve to a world quantity."
                )
            subspace = _view_subspace(spec)
            axis_raw = spec.view.axis
            axis = semantic_axis_label(axis_raw)
            if (
                qty is not None
                and qty.type == WorldQuantityType.Pose
                and subspace == "distance"
                and axis is None
                and not _is_distance_view(spec)
            ):
                raise ValueError(
                    f"Constraint '{spec.name}' must use explicit "
                    "'distance between <pose-a> and <pose-b>' syntax."
                )

            if qty is not None:
                if (
                    subspace == "pose"
                    and axis is None
                    and qty.type == WorldQuantityType.Pose
                ):
                    qty_node = URIRef(qty.uri)
                else:
                    sid = _scalar_id(qty, subspace, axis)
                    qty_node = self._owned_uri(sid, motion)
            else:
                qty_node = None

            scalar_t = _scalar_type(qty, subspace, axis) if qty else subspace
            type_name = CSTR_TYPE_NAME.get(scalar_t, scalar_t)
            self.graph.add((node, RDF.type, CSTR.Constraint))
            self.graph.add((node, RDF.type, _ns_term(CSTR, f"{type_name}Constraint")))
            if qty_node is not None:
                self.graph.add((node, CSTR.quantity, qty_node))

            expr = spec.expr
            if isinstance(expr, EqualityConstraint):
                self.graph.add((node, RDF.type, CSTR.EqualityConstraint))
                ref_node = self._constraint_reference_node(expr.reference, motion, f"{spec.name}-ref", subspace, axis)
                if (
                    qty is not None
                    and qty.type == WorldQuantityType.Pose
                    and subspace == "pose"
                    and axis is None
                ):
                    ref_node = self._transform_pose_reference_to_world_frame(
                        ref_quantity=_context_quantity(expr.reference),
                        ref_node=ref_node,
                        target_quantity=qty,
                        world_qtys=world_qtys,
                        motion=motion,
                        stem=spec.name,
                    )
                self.graph.add((node, CSTR["reference-value"], ref_node))
            elif isinstance(expr, GreaterThanConstraint):
                self.graph.add((node, RDF.type, CSTR.UnilateralConstraint))
                self.graph.add((node, RDF.type, CSTR.GreaterThanConstraint))
                thr_node = self._emit_context_ref_node(
                    expr.threshold, motion, f"{spec.name}-threshold"
                )
                self.graph.add((node, CSTR.threshold, thr_node))
            elif isinstance(expr, LessThanConstraint):
                self.graph.add((node, RDF.type, CSTR.UnilateralConstraint))
                self.graph.add((node, RDF.type, CSTR.LessThanConstraint))
                thr_node = self._emit_context_ref_node(
                    expr.threshold, motion, f"{spec.name}-threshold"
                )
                self.graph.add((node, CSTR.threshold, thr_node))
            elif isinstance(expr, BilateralConstraint):
                self.graph.add((node, RDF.type, CSTR.BilateralConstraint))
                lo_node = self._emit_context_ref_node(expr.lower, motion, f"{spec.name}-lower")
                up_node = self._emit_context_ref_node(expr.upper, motion, f"{spec.name}-upper")
                self.graph.add((node, CSTR["lower-threshold"], lo_node))
                self.graph.add((node, CSTR["upper-threshold"], up_node))

    def _emit_motion_spec(self, motion: MotionSpec) -> None:
        motion_node = self._owned_uri(f"motion-{motion.name}", motion)
        self.graph.add((motion_node, RDF.type, MOT.GuardedMotion))
        if getattr(motion, "trajectory", None) is not None:
            lerp_node = self._owned_uri(f"motion-{motion.name}-lerp", motion)
            self.graph.add((lerp_node, RDF.type, TRAJ.Lerp))
            self.graph.add((motion_node, TRAJ.trajectory, lerp_node))
        for item in motion.when.constraints:
            spec = _resolved_spec(item)
            if not spec.disabled:
                self.graph.add((motion_node, MOT.when, URIRef(spec.uri)))
        for item in motion.while_.constraints:
            spec = _resolved_spec(item)
            if not spec.disabled:
                self.graph.add((motion_node, MOT["while"], URIRef(spec.uri)))
        raw_logic = getattr(motion.until, "logic", None)
        until_constraints = [i for i in motion.until.constraints if not _resolved_spec(i).disabled]
        if raw_logic == "any" and len(until_constraints) > 1:
            disjunction_node = self._owned_uri(f"motion-{motion.name}-until-disjunction", motion)
            self.graph.add((disjunction_node, RDF.type, MOT_EXT.ConstraintDisjunction))
            self.graph.add((motion_node, MOT.until, disjunction_node))
            for item in until_constraints:
                self.graph.add((disjunction_node, MOT_EXT["has-constraint"], URIRef(_resolved_spec(item).uri)))
        else:
            for item in until_constraints:
                self.graph.add((motion_node, MOT.until, URIRef(_resolved_spec(item).uri)))

    def _emit_scalar_views(
        self,
        motion: MotionSpec,
        constraints: list[ConstraintSpecification],
        world_qtys: dict[str, WorldQuantity],
    ) -> None:
        seen: set[tuple] = set()
        rotation_motions: set[str] = set()

        for spec in constraints:
            qty = self._resolve_constraint_quantity(spec, world_qtys)
            if qty is None:
                continue
            subspace = _view_subspace(spec)
            axis_raw = spec.view.axis
            axis = semantic_axis_label(axis_raw)

            ws_spec = WORLD_SPECS.get(qty.type)
            if ws_spec is None:
                continue
            prop = ws_spec[3].get(subspace)

            if qty.type == WorldQuantityType.Pose and subspace == "rotation" and axis is None:
                rotation_motions.add(motion.name)
                continue

            if qty.type == WorldQuantityType.Pose and subspace == "pose":
                key = (qty.name, "pose", None)
                if key not in seen:
                    seen.add(key)
                    sid = _scalar_id(qty, "pose", None)
                    pose_scalar = self._owned_uri(sid, motion)
                    self._add_quantity(pose_scalar, QuantityType.Pose)
                    if isinstance(qty.props, GeometricProps):
                        of_v = _geo_prop(qty.props, "of")
                        wrt_v = _geo_prop(qty.props, "wrt")
                        if of_v and wrt_v:
                            self.graph.add((pose_scalar, GEOM_REL.of, self._owned_uri(of_v, qty)))
                            self.graph.add((pose_scalar, GEOM_REL["with-respect-to"], self._owned_uri(wrt_v, qty)))
                            self.graph.add((pose_scalar, GEOM_COORD["as-seen-by"], self._owned_uri(wrt_v, qty)))
                continue

            if (
                qty.type == WorldQuantityType.Pose
                and subspace in {"position", "orientation"}
                and axis is None
            ):
                key = (qty.name, subspace, None)
                if key not in seen:
                    seen.add(key)
                    sid = _scalar_id(qty, subspace, None)
                    scalar_node = self._owned_uri(sid, motion)
                    scalar_type = (
                        QuantityType.Position
                        if subspace == "position"
                        else QuantityType.Orientation
                    )
                    self._add_quantity(scalar_node, scalar_type)
                continue

            if axis is None or prop is None or prop[4] is None:
                continue

            key = (qty.name, subspace, axis)
            if key in seen:
                continue
            seen.add(key)

            view_subspace_uri, _, _, scalar_t, view_type = prop
            sid = _scalar_id(qty, subspace, axis)
            scalar_node = self._owned_uri(sid, motion)
            self._add_quantity(scalar_node, scalar_t)
            if view_type == MAP.PoseCoordinateView:
                self.graph.add((scalar_node, RDF.type, QUDT_QKIND.Position))

            view_node = self._owned_uri(f"view-{sid}", motion)
            self.graph.add((view_node, RDF.type, MAP.View))
            self.graph.add((view_node, RDF.type, view_type))
            self.graph.add((view_node, MAP.superobject, URIRef(qty.uri)))
            self.graph.add((view_node, MAP.subobject, scalar_node))
            self.graph.add((view_node, MAP.subspace, MAP[view_subspace_uri]))
            self.graph.add((view_node, MAP.axis, MAP[axis]))

        for spec in constraints:
            qty = self._resolve_constraint_quantity(spec, world_qtys)
            if qty is None:
                continue
            subspace = _view_subspace(spec)
            axis_raw = spec.view.axis
            axis = semantic_axis_label(axis_raw)
            if _scalar_type(qty, subspace, None) in (QuantityType.Angle, QuantityType.PlaneAngle):
                sid = _scalar_id(qty, subspace, axis)
                self._add_quantity(self._owned_uri(sid, motion), _scalar_type(qty, subspace, axis))

        for mn in rotation_motions:
            self._add_quantity(self._owned_uri(f"rotation-{mn}", motion), QuantityType.Angle)

    def _emit_map_operations(
        self,
        motion: MotionSpec,
        constraints: list[ConstraintSpecification],
        world_qtys: dict[str, WorldQuantity],
    ) -> None:
        rotation_pose = next(
            (
                _node_name(spec.view.quantity)
                for spec in constraints
                if _view_subspace(spec) == "rotation" and spec.view.axis is None
            ),
            None,
        )
        if rotation_pose is not None:
            rotation_id = f"rotation-{motion.name}"
            op_node = self._owned_uri(f"compute-{rotation_id}", motion)
            self.graph.add((op_node, RDF.type, MAP.ComputeRotationFromPose))
            self.graph.add((op_node, MAP.pose, self._owned_uri(rotation_pose, motion)))
            self.graph.add((op_node, MAP.rotation, self._owned_uri(rotation_id, motion)))

        seen_angle_ops: set[str] = set()
        for spec in constraints:
            qty = self._resolve_constraint_quantity(spec, world_qtys)
            if qty is None or qty.type != WorldQuantityType.Pose:
                continue
            subspace = _view_subspace(spec)
            axis_raw = spec.view.axis
            axis = semantic_axis_label(axis_raw)
            if subspace != "rotation" or axis is None:
                continue
            scalar_id = _scalar_id(qty, subspace, axis)
            if scalar_id in seen_angle_ops:
                continue
            seen_angle_ops.add(scalar_id)
            op_node = self._owned_uri(f"compute-{scalar_id}", motion)
            self.graph.add((op_node, RDF.type, GEOM_OP.PoseToAngleAroundAxis))
            self.graph.add((op_node, GEOM_OP.pose, self._owned_uri(qty.name, motion)))
            self.graph.add((op_node, GEOM_OP.angle, self._owned_uri(scalar_id, motion)))
            self.graph.add((op_node, GEOM_OP.axis, GEOM_OP[axis]))

        seen_distance_ops: set[str] = set()
        for spec in constraints:
            qty = self._resolve_constraint_quantity(spec, world_qtys)
            if qty is None or qty.type != WorldQuantityType.Pose:
                continue
            axis_raw = spec.view.axis
            axis = semantic_axis_label(axis_raw)
            if (
                not _is_distance_view(spec)
                or _view_subspace(spec) != "distance"
                or axis is not None
            ):
                continue
            if not isinstance(qty.props, GeometricProps):
                raise ValueError(f"Distance derivation for '{qty.name}' needs pose endpoints.")
            rel_of = _geo_prop(qty.props, "of")
            rel_wrt = _geo_prop(qty.props, "wrt")
            if rel_of is None or rel_wrt is None:
                raise ValueError(
                    f"Distance derivation for '{qty.name}' needs explicit 'of' and 'wrt' frames."
                )
            distance_id = _scalar_id(qty, "distance", None)
            if distance_id in seen_distance_ops:
                continue
            seen_distance_ops.add(distance_id)

            plan = self._distance_relative_pose_plan(spec, world_qtys)
            target_node = self._emit_relative_pose(
                plan,
                motion,
                context=f"Distance constraint '{spec.name}'",
                stem=spec.name,
            )

            distance_node = self._owned_uri(distance_id, motion)
            self._add_quantity(distance_node, QuantityType.Distance)
            op_node = self._owned_uri(f"compute-{distance_id}", motion)
            self.graph.add((op_node, RDF.type, GEOM_OP.PoseToLinearDistance))
            self.graph.add((op_node, GEOM_OP.pose, target_node))
            self.graph.add((op_node, GEOM_OP.distance, distance_node))

    def _emit_controller_base(self, ctrl_node: URIRef, ctrl: ControllerEntry) -> None:
        self.graph.add((ctrl_node, RDF.type, CSTR_HDL.Controller))
        if ctrl.type == ControllerType.PID:
            self.graph.add((ctrl_node, RDF.type, CSTR_HDL.ProportionalIntegralDerivative))
            if ctrl.params.kp is not None:
                self.graph.add(
                    (ctrl_node, CSTR_HDL["proportional-gain"], Literal(float(ctrl.params.kp), datatype=XSD.double))
                )
            if ctrl.params.ki is not None:
                self.graph.add((ctrl_node, CSTR_HDL["integral-gain"], Literal(float(ctrl.params.ki), datatype=XSD.double)))
            if ctrl.params.kd is not None:
                self.graph.add(
                    (ctrl_node, CSTR_HDL["derivative-gain"], Literal(float(ctrl.params.kd), datatype=XSD.double))
                )
            if ctrl.params.decay is not None:
                self.graph.add((ctrl_node, RDF.type, CSTR_HDL.DecayingIntegralTerm))
                self.graph.add((ctrl_node, CSTR_HDL["decay-rate"], Literal(float(ctrl.params.decay), datatype=XSD.double)))
        elif ctrl.type == ControllerType.Impedance:
            self.graph.add((ctrl_node, RDF.type, CSTR_HDL.ImpedanceController))
            if ctrl.params.stiffness is not None:
                k_node = URIRef(f"{ctrl_node}-stiffness")
                self.graph.add((k_node, QUDT_SCHEMA.value, Literal(float(ctrl.params.stiffness), datatype=XSD.double)))
                self.graph.add((ctrl_node, CSTR_HDL["stiffness"], k_node))
            if ctrl.params.damping is not None:
                d_node = URIRef(f"{ctrl_node}-damping")
                self.graph.add((d_node, QUDT_SCHEMA.value, Literal(float(ctrl.params.damping), datatype=XSD.double)))
                self.graph.add((ctrl_node, CSTR_HDL["damping"], d_node))
        elif ctrl.type == ControllerType.FeedForward:
            self.graph.add((ctrl_node, RDF.type, CSTR_HDL.FeedForwardController))
        else:
            raise ValueError(
                f"Controller '{ctrl.name}' uses {ctrl.type.value}, "
                "which is not supported for graph emission."
            )

    def _emit_acceleration_energy_quantity(self, energy_node: URIRef) -> None:
        self.graph.add((energy_node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((energy_node, RDF.type, QUDT_QKIND.AccelerationEnergy))
        self.graph.add((energy_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.AccelerationEnergy))
        self.graph.add((energy_node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M2-PER-SEC2"]))

    def _emit_error_evaluator(
        self,
        handler_node: URIRef,
        spec: ConstraintSpecification,
        error_node: URIRef,
        seen_eval_ids: set[str],
    ) -> None:
        eval_id = _evaluator_id(spec)
        eval_node = self._owned_uri(eval_id, spec.parent)
        if eval_id not in seen_eval_ids:
            seen_eval_ids.add(eval_id)
            self.graph.add((eval_node, RDF.type, CSTR_HDL.ConstraintEvaluator))
            self.graph.add((eval_node, RDF.type, CSTR_HDL.ErrorEvaluator))
            self.graph.add((eval_node, CSTR_HDL.constraint, URIRef(spec.uri)))
            self.graph.add((eval_node, CSTR_HDL.error, error_node))
        self.graph.add((handler_node, CSTR_HDL.evaluators, eval_node))

    def _emit_pose_diff_evaluator(
        self,
        handler_node: URIRef,
        ctrl: ControllerEntry,
        spec: ConstraintSpecification,
        qty: WorldQuantity,
        ref_uri: URIRef,
        components: tuple[AccelerationConstraintRecord, ...],
        seen_eval_ids: set[str],
    ) -> None:
        eval_id = f"eval-pose-diff-{ctrl.name}"
        eval_node = self._owned_uri(eval_id, spec.parent)
        if eval_id not in seen_eval_ids:
            seen_eval_ids.add(eval_id)
            diff_id = f"pose-diff-{ctrl.name}"
            diff_node = self._owned_uri(diff_id, spec.parent)
            ref_point_node = self._owned_uri(f"point-{diff_id}-origin", spec.parent)
            props = qty.props if isinstance(qty.props, GeometricProps) else None
            as_seen_by = _geo_prop(props, "as-seen-by") or _geo_prop(props, "wrt")
            if as_seen_by is None:
                raise ValueError(
                    f"Pose equality constraint '{spec.name}' needs an as-seen-by or wrt frame."
                )

            self.graph.add((eval_node, RDF.type, CSTR_HDL.ConstraintEvaluator))
            self.graph.add((eval_node, RDF.type, GEOM_OP["PoseDiffEvaluator"]))
            self.graph.add((eval_node, CSTR_HDL.constraint, URIRef(spec.uri)))
            self.graph.add((eval_node, GEOM_OP.in1, URIRef(qty.uri)))
            self.graph.add((eval_node, GEOM_OP.in2, ref_uri))
            self.graph.add((eval_node, GEOM_OP.out, diff_node))
            self.graph.add((diff_node, RDF.type, GEOM_REL.AccelerationTwist))
            self.graph.add((diff_node, RDF.type, GEOM_COORD.AccelerationTwistCoordinate))
            self.graph.add((diff_node, RDF.type, GEOM_COORD.VectorXYZ))
            self.graph.add(
                (diff_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.AngularAcceleration)
            )
            self.graph.add(
                (diff_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.LinearAcceleration)
            )
            self.graph.add((diff_node, QUDT_SCHEMA.unit, QUDT_UNIT["RAD-PER-SEC2"]))
            self.graph.add((diff_node, QUDT_SCHEMA.unit, QUDT_UNIT["M-PER-SEC2"]))
            self.graph.add((diff_node, GEOM_REL["reference-point"], ref_point_node))
            self.graph.add((diff_node, GEOM_COORD["as-seen-by"], self._owned_uri(as_seen_by, qty)))
            of_v = _geo_prop(props, "of")
            wrt_v = _geo_prop(props, "wrt")
            if of_v:
                self.graph.add((diff_node, GEOM_REL.of, self._owned_uri(of_v, qty)))
            if wrt_v:
                self.graph.add((diff_node, GEOM_REL["with-respect-to"], self._owned_uri(wrt_v, qty)))
            self.graph.add((ref_point_node, RDF.type, GEOM_ENT.Point))

            for component in components:
                err_id = _pose_diff_error_id(ctrl, component)
                err_node = self._owned_uri(err_id, spec.parent)
                self._add_quantity(err_node, component.quantity_type)
                view_node = self._owned_uri(f"view-{err_id}", spec.parent)
                self.graph.add((view_node, RDF.type, MAP.View))
                self.graph.add((view_node, RDF.type, MAP.AccelerationTwistCoordinateView))
                self.graph.add((view_node, MAP.superobject, diff_node))
                self.graph.add((view_node, MAP.subobject, err_node))
                self.graph.add((view_node, MAP.subspace, MAP[component.subspace]))
                self.graph.add((view_node, MAP.axis, MAP[component.axis]))
        self.graph.add((handler_node, CSTR_HDL.evaluators, eval_node))

    def _emit_pose_diff_component_controllers(
        self,
        handler_node: URIRef,
        handler: ConstraintHandler,
        ctrl: ControllerEntry,
        spec: ConstraintSpecification,
        motion: MotionSpec,
        components: tuple[AccelerationConstraintRecord, ...],
    ) -> None:
        for component in components:
            err_id = _pose_diff_error_id(ctrl, component)
            comp_ctrl_node = self._owned_uri(_pose_diff_controller_id(ctrl, component), handler)
            energy_node = self._owned_uri(_pose_diff_energy_id(ctrl, component), motion)
            self._emit_controller_base(comp_ctrl_node, ctrl)
            self.graph.add(
                (comp_ctrl_node, CSTR_HDL["error-signal"], self._owned_uri(err_id, spec.parent))
            )
            self.graph.add((comp_ctrl_node, CSTR_HDL["control-signal"], energy_node))
            self._emit_acceleration_energy_quantity(energy_node)
            self.graph.add((handler_node, CSTR_HDL.controllers, comp_ctrl_node))

    def _emit_constraint_handler(
        self,
        handler: ConstraintHandler,
        motion: MotionSpec,
        world_qtys: dict[str, WorldQuantity],
        shared_spec_ids: frozenset[int],
        handler_order: int,
    ) -> None:
        handler_node = URIRef(handler.uri)
        self.graph.add((handler_node, RDF.type, CSTR_HDL.ConstraintHandler))
        self.graph.add((handler_node, APP.order, Literal(handler_order)))
        self.graph.add(
            (handler_node, CSTR_HDL.motion, self._owned_uri(f"motion-{motion.name}", motion))
        )
        self.graph.add(
            (handler_node, CSTR_HDL["control-mode"], CSTR_HDL[handler.control_mode.value])
        )
        event_loop_node = URIRef(f"{handler.uri}.event-loop")
        self.graph.add((event_loop_node, RDF.type, EL.EventLoop))

        seen_error_ids: set[str] = set()
        seen_eval_ids: set[str] = set()

        for ctrl_item in getattr(handler, "controllers", []):
            ctrl = ctrl_item.ref.controller if hasattr(ctrl_item, "ref") else ctrl_item
            cref = ctrl.params.constraint
            spec = cref.constraint if hasattr(cref, "constraint") else None
            if spec is None:
                continue
            if spec.disabled:
                continue

            qty = self._resolve_constraint_quantity(spec, world_qtys)
            if qty is None:
                raise ValueError(
                    f"Controller '{ctrl.name}' constraint '{spec.name}' does not resolve to a world quantity."
                )
            subspace = _view_subspace(spec)
            axis_raw = spec.view.axis
            axis = semantic_axis_label(axis_raw)
            shared = id(spec) in shared_spec_ids
            scalar_t = _scalar_type(qty, subspace, axis) if qty else subspace
            command = controller_command_record(ctrl)

            error_id: str | None = None
            if qty is not None and ctrl.type != ControllerType.FeedForward:
                sid = _scalar_id(qty, subspace, axis)
                error_id = (sid + "-err") if shared else f"{sid}-err-{motion.name}"

            # Pose equality can expand to any requested acceleration components:
            # full pose -> 6D, position-only -> 3D, or orientation-only -> 3D.
            if (
                qty is not None
                and qty.type == WorldQuantityType.Pose
                and subspace in {"pose", "position", "orientation", "distance", "rotation"}
                and axis is None
                and command.acceleration_constraints
                and isinstance(spec.expr, EqualityConstraint)
            ):
                ref_qty = _context_quantity(spec.expr.reference)
                ref_uri = None
                if ref_qty is not None and getattr(ref_qty, "type", None) in {QuantityType.Pose, QuantityType.Trajectory}:
                    ref_uri = URIRef(ref_qty.uri)
                if ref_uri is None:
                    ref_uri = self.graph.value(URIRef(spec.uri), CSTR["reference-value"])
                if ref_uri is not None:
                    self._emit_pose_diff_evaluator(
                        handler_node,
                        ctrl,
                        spec,
                        qty,
                        URIRef(ref_uri),
                        command.acceleration_constraints,
                        seen_eval_ids,
                    )
                    self._emit_pose_diff_component_controllers(
                        handler_node,
                        handler,
                        ctrl,
                        spec,
                        motion,
                        command.acceleration_constraints,
                    )
                    continue

            ctrl_node = URIRef(ctrl.uri)
            self._emit_controller_base(ctrl_node, ctrl)
            self.graph.add((ctrl_node, CSTR_HDL.constraint, URIRef(spec.uri)))

            if error_id:
                self.graph.add(
                    (ctrl_node, CSTR_HDL["error-signal"], self._owned_uri(error_id, spec.parent))
                )

            if ctrl.type == ControllerType.FeedForward and isinstance(spec.expr, EqualityConstraint):
                ref_qty = _context_quantity(spec.expr.reference)
                if ref_qty is not None:
                    self.graph.add((ctrl_node, CSTR_HDL["reference-signal"], URIRef(ref_qty.uri)))
                eval_id = _evaluator_id(spec)
                eval_node = self._owned_uri(eval_id, spec.parent)
                if eval_id not in seen_eval_ids:
                    seen_eval_ids.add(eval_id)
                    self.graph.add((eval_node, RDF.type, CSTR_HDL.ConstraintEvaluator))
                    self.graph.add((eval_node, RDF.type, CSTR_HDL.AssignmentEvaluator))
                    self.graph.add((eval_node, CSTR_HDL.constraint, URIRef(spec.uri)))
                self.graph.add((handler_node, CSTR_HDL.evaluators, eval_node))

            control_signal_node = self._decode_control_signal(
                ctrl, qty, subspace, axis, motion, handler, shared
            )
            self.graph.add((ctrl_node, CSTR_HDL["control-signal"], control_signal_node))
            self.graph.add((handler_node, CSTR_HDL.controllers, ctrl_node))

            if error_id and error_id not in seen_error_ids:
                seen_error_ids.add(error_id)
                err_node = self._owned_uri(error_id, spec.parent)
                self._add_quantity(err_node, scalar_t)
                if scalar_t == QuantityType.Pose and qty is not None and isinstance(qty.props, GeometricProps):
                    of_v = _geo_prop(qty.props, "of")
                    wrt_v = _geo_prop(qty.props, "wrt")
                    if of_v and wrt_v:
                        self.graph.add((err_node, GEOM_REL.of, self._owned_uri(of_v, qty)))
                        self.graph.add((err_node, GEOM_REL["with-respect-to"], self._owned_uri(wrt_v, qty)))
                        self.graph.add((err_node, GEOM_COORD["as-seen-by"], self._owned_uri(wrt_v, qty)))

            if error_id:
                self._emit_error_evaluator(
                    handler_node,
                    spec,
                    self._owned_uri(error_id, spec.parent),
                    seen_eval_ids,
                )

        for mon in getattr(handler, "monitors", []):
            cref = mon.constraint
            signal_name = mon.event or mon.flag
            signal_kind = "event" if mon.event else "flag"
            signal_node = URIRef(f"{mon.uri}.{signal_name}")
            mon_node = URIRef(mon.uri)

            if isinstance(cref, UntilMonitorRef):
                aggregate_error_node = URIRef(f"{mon.uri}.error")
                self._add_quantity(aggregate_error_node, QuantityType.Vector)
                for item in cref.motion.until.constraints:
                    spec = _resolved_spec(item)
                    qty = self._resolve_constraint_quantity(spec, world_qtys)
                    if qty is None:
                        raise ValueError(
                            f"Until monitor '{mon.name}' constraint '{spec.name}' does not resolve to a world quantity."
                        )
                    subspace = _view_subspace(spec)
                    axis_raw = spec.view.axis
                    axis = semantic_axis_label(axis_raw)
                    scalar_t = _scalar_type(qty, subspace, axis) if qty else subspace
                    qty_node_id = _scalar_id(qty, subspace, axis) if qty else spec.name
                    error_id = f"{qty_node_id}-err"

                    if error_id not in seen_error_ids:
                        seen_error_ids.add(error_id)
                        err_node = self._owned_uri(error_id, spec.parent)
                        self._add_quantity(err_node, scalar_t)
                        if scalar_t == QuantityType.Pose and qty is not None and isinstance(qty.props, GeometricProps):
                            of_v = _geo_prop(qty.props, "of")
                            wrt_v = _geo_prop(qty.props, "wrt")
                            if of_v and wrt_v:
                                self.graph.add((err_node, GEOM_REL.of, self._owned_uri(of_v, qty)))
                                self.graph.add((err_node, GEOM_REL["with-respect-to"], self._owned_uri(wrt_v, qty)))
                                self.graph.add((err_node, GEOM_COORD["as-seen-by"], self._owned_uri(wrt_v, qty)))

                    self._emit_error_evaluator(
                        handler_node,
                        spec,
                        self._owned_uri(error_id, spec.parent),
                        seen_eval_ids,
                    )

                self.graph.add(
                    (signal_node, RDF.type, EL.Event if signal_kind == "event" else EL.Flag)
                )
                self.graph.add(
                    (event_loop_node, EL["has-event"] if signal_kind == "event" else EL["has-flag"], signal_node)
                )
                self.graph.add((mon_node, RDF.type, CSTR_HDL.Monitor))
                self.graph.add((mon_node, CSTR_HDL.error, aggregate_error_node))
                self.graph.add((mon_node, CSTR_HDL["monitors-until"], self._owned_uri(f"motion-{cref.motion.name}", cref.motion)))
                if signal_kind == "event":
                    self.graph.add((mon_node, RDF.type, CSTR_HDL.EdgeTriggeredMonitor))
                    self.graph.add((mon_node, CSTR_HDL.event, signal_node))
                    self.graph.add((mon_node, CSTR_HDL["event-queue"], event_loop_node))
                else:
                    self.graph.add((mon_node, RDF.type, CSTR_HDL.LevelTriggeredMonitor))
                    self.graph.add((mon_node, CSTR_HDL.flag, signal_node))
                self.graph.add((handler_node, CSTR_HDL.monitors, mon_node))
                continue

            spec = cref.constraint if hasattr(cref, "constraint") else None
            if spec is None:
                continue
            if spec.disabled:
                continue

            qty = self._resolve_constraint_quantity(spec, world_qtys)
            if qty is None:
                raise ValueError(
                    f"Monitor '{mon.name}' constraint '{spec.name}' does not resolve to a world quantity."
                )
            subspace = _view_subspace(spec)
            axis_raw = spec.view.axis
            axis = semantic_axis_label(axis_raw)
            scalar_t = _scalar_type(qty, subspace, axis)
            qty_node_id = _scalar_id(qty, subspace, axis) if qty else spec.name
            error_id = f"{qty_node_id}-err"

            if error_id not in seen_error_ids:
                seen_error_ids.add(error_id)
                self._add_quantity(self._owned_uri(error_id, spec.parent), scalar_t)

            self._emit_error_evaluator(
                handler_node,
                spec,
                self._owned_uri(error_id, spec.parent),
                seen_eval_ids,
            )

            self.graph.add(
                (signal_node, RDF.type, EL.Event if signal_kind == "event" else EL.Flag)
            )
            self.graph.add(
                (event_loop_node, EL["has-event"] if signal_kind == "event" else EL["has-flag"], signal_node)
            )
            self.graph.add((mon_node, RDF.type, CSTR_HDL.Monitor))
            self.graph.add((mon_node, CSTR_HDL.constraint, URIRef(spec.uri)))
            self.graph.add((mon_node, CSTR_HDL.error, self._owned_uri(error_id, spec.parent)))
            if signal_kind == "event":
                self.graph.add((mon_node, RDF.type, CSTR_HDL.EdgeTriggeredMonitor))
                self.graph.add((mon_node, CSTR_HDL.event, signal_node))
                self.graph.add((mon_node, CSTR_HDL["event-queue"], event_loop_node))
            else:
                self.graph.add((mon_node, RDF.type, CSTR_HDL.LevelTriggeredMonitor))
                self.graph.add((mon_node, CSTR_HDL.flag, signal_node))
            self.graph.add((handler_node, CSTR_HDL.monitors, mon_node))


    def _decode_control_signal(
        self,
        ctrl: ControllerEntry,
        qty: WorldQuantity | None,
        subspace: str,
        axis: str | None,
        motion: MotionSpec,
        handler: ConstraintHandler,
        shared: bool,
    ) -> URIRef:
        solver = self._controller_solver(handler, ctrl)
        command = controller_command_record(ctrl)

        if ctrl.type == ControllerType.FeedForward:
            signal_node = self._owned_uri(f"cmd-{ctrl.name}", handler)
            signal_type = _scalar_type(qty, subspace, axis) if qty is not None else QuantityType.Vector
            self._add_quantity(signal_node, signal_type)
            return signal_node

        if solver is None:
            raise ValueError(
                f"Controller '{ctrl.name}' cannot resolve a solver; specify 'via <solver>' "
                "or ensure the handler has exactly one compatible solver."
            )
        if qty is None:
            raise ValueError(
                f"Controller '{ctrl.name}' cannot emit a control signal without a resolved quantity."
            )
        algorithm = getattr(solver, "algorithm", None)

        ws_spec = WORLD_SPECS.get(qty.type) if qty else None
        prop = ws_spec[3].get(subspace) if ws_spec else None
        accel_prefix = prop[2] if prop else None
        accel_subspace_label = prop[1] if prop else None

        # Cartesian force command: controller output is a force magnitude.
        if qty is not None and command.is_force_command:
            return self._force_control_signal_node(ctrl, handler)

        # ACHD acceleration energy
        if (
            algorithm == "ACHD"
            and qty is not None
            and axis is not None
            and accel_prefix is not None
            and accel_subspace_label is not None
            and command.acceleration_constraints
        ):
            sid = _scalar_id(qty, subspace, axis)
            stem = f"eacc-{sid}"
            energy_id = stem if shared else f"{stem}-{motion.name}"
            energy_node = self._owned_uri(energy_id, motion)
            self.graph.add((energy_node, RDF.type, QUDT_SCHEMA.Quantity))
            self.graph.add((energy_node, RDF.type, QUDT_QKIND.AccelerationEnergy))
            self.graph.add(
                (energy_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.AccelerationEnergy)
            )
            self.graph.add((energy_node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M2-PER-SEC2"]))
            return energy_node

        # Direct joint-space torque
        if (
            algorithm == "ACHD"
            and command.is_posture_torque_command
            and qty is not None
            and qty.type == WorldQuantityType.JointPosition
        ):
            signal_id = f"tau-{ctrl.name}"
            signal_node = self._owned_uri(signal_id, handler)
            self.graph.add((signal_node, RDF.type, QUDT_SCHEMA.Quantity))
            self.graph.add((signal_node, RDF.type, QUDT_QKIND.Torque))
            self.graph.add((signal_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Torque))
            self.graph.add((signal_node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M"]))
            return signal_node

        energy_node = self._owned_uri(f"eacc-{ctrl.name}", motion)
        self.graph.add((energy_node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((energy_node, RDF.type, QUDT_QKIND.AccelerationEnergy))
        self.graph.add((energy_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.AccelerationEnergy))
        self.graph.add((energy_node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M2-PER-SEC2"]))
        return energy_node

    def _emit_solvers(
        self,
        handler: ConstraintHandler,
        motion: MotionSpec,
        world_qtys: dict[str, WorldQuantity],
        shared_spec_ids: frozenset[int],
    ) -> None:
        solvers = [_resolved_solver(s) for s in getattr(handler, "solvers", [])]
        multi = len(solvers) > 1

        for solver in solvers:
            if not getattr(solver, "algorithm", ""):
                continue

            solver_stem = solver.name
            driver_stem = (
                f"{solver.name}-{motion.name or handler.name}"
                if multi
                else (motion.name or handler.name)
            )

            driver_node = self._owned_uri(f"driver-{driver_stem}", handler)
            self.graph.add((driver_node, RDF.type, SLV.MotionDrivers))

            solver_node = self._owned_uri(f"{solver_stem}-{motion.name}", handler)

            alg = solver.algorithm
            if alg == "CommandForwarding":
                self.graph.add((solver_node, RDF.type, SLV.CommandForwardingSolver))
                self.graph.add((solver_node, SLV.solver, SLV.CommandForwardingAlgorithm))
                self._emit_solver_interfaces(
                    handler,
                    motion,
                    solver,
                    solver_node,
                    driver_stem,
                    driver_node,
                    world_qtys,
                    shared_spec_ids,
                )
                continue

            self.graph.add((solver_node, RDF.type, SLV.SolverWithInputAndOutput))

            alg_node = (
                SLV.AccelerationConstrainedHybridDynamicsAlgorithm
                if alg == "ACHD"
                else SLV.NewtonEulerAlgorithm
                if alg == "RNE"
                else SLV[alg]
            )
            self.graph.add((solver_node, SLV.solver, alg_node))

            root_node = self._owned_uri(_node_name(solver.root), handler)
            self.graph.add((root_node, RDF.type, GEOM_ENT.Frame))
            self.graph.add((solver_node, SLV.root, root_node))
            gravity_qty = self._resolve_qty(solver.gravity, world_qtys)
            if gravity_qty is not None:
                self.graph.add((solver_node, SLV.gravity, URIRef(gravity_qty.uri)))
            gravity_ref = getattr(solver, "gravity_value", None)
            gravity_value = _context_quantity(gravity_ref) if gravity_ref is not None else None
            if gravity_value is not None:
                self.graph.add((solver_node, SLV["gravity-value"], URIRef(gravity_value.uri)))

            chain_root_name = getattr(
                getattr(getattr(solver.robot, "environment_robot", None), "assembly_spec", None),
                "root", ""
            )
            self.graph.add((solver_node, SLV["motion-drivers"], driver_node))

            if chain_root_name:
                root_body = _body_name(chain_root_name)
                for qty in world_qtys.values():
                    if qty.type not in (WorldQuantityType.Pose, WorldQuantityType.VelocityTwist):
                        continue
                    props = qty.props if isinstance(qty.props, GeometricProps) else None
                    wrt = _geo_prop(props, "wrt")
                    if wrt and _body_name(wrt) == root_body:
                        self.graph.add((solver_node, SLV["output"], URIRef(qty.uri)))

            self._emit_solver_interfaces(
                handler,
                motion,
                solver,
                solver_node,
                driver_stem,
                driver_node,
                world_qtys,
                shared_spec_ids,
            )

    def _emit_solver_interfaces(
        self,
        handler: ConstraintHandler,
        motion: MotionSpec,
        solver: Any,
        solver_node: URIRef,
        stem: str,
        driver_node: URIRef,
        world_qtys: dict[str, WorldQuantity],
        shared_spec_ids: frozenset[int],
    ) -> None:
        acc_constraint_nodes: list[URIRef] = []

        for ctrl_item in getattr(handler, "controllers", []):
            ctrl = ctrl_item.ref.controller if hasattr(ctrl_item, "ref") else ctrl_item
            if self._controller_solver(handler, ctrl) is not solver:
                continue

            cref = ctrl.params.constraint
            spec = cref.constraint if hasattr(cref, "constraint") else None
            if spec is None:
                continue

            qty = self._resolve_constraint_quantity(spec, world_qtys)
            if qty is None:
                raise ValueError(
                    f"Controller '{ctrl.name}' constraint '{spec.name}' does not resolve to a world quantity."
                )

            subspace = _view_subspace(spec)
            axis_raw = spec.view.axis
            axis = semantic_axis_label(axis_raw)
            shared = id(spec) in shared_spec_ids
            command = controller_command_record(ctrl)
            axis_acceleration = (
                command.acceleration_constraints[0]
                if axis is not None and command.acceleration_constraints
                else None
            )

            if solver.algorithm == "CommandForwarding" and ctrl.type == ControllerType.FeedForward:
                target_name = _geo_prop(qty.props if isinstance(qty.props, GeometricProps) else None, "of")
                if target_name:
                    control_signal_node = self._decode_control_signal(
                        ctrl, qty, subspace, axis, motion, handler, shared
                    )
                    spec_node = self._owned_uri(f"cmd-fwd-{ctrl.name}", handler)
                    self.graph.add((spec_node, RDF.type, SLV.CommandForwardingSpecification))
                    self.graph.add((spec_node, SLV["control-signal"], control_signal_node))
                    self.graph.add((spec_node, SLV["attached-to"], self._owned_uri(target_name, qty)))
                    self.graph.add((solver_node, SLV["command-forwarding"], spec_node))
                continue

            if (
                solver.algorithm == "ACHD"
                and qty.type == WorldQuantityType.Pose
                and subspace in {"pose", "position", "orientation"}
                and axis is None
                and command.acceleration_constraints
            ):
                axis_frame = _quantity_axis_frame(qty)
                for component in command.acceleration_constraints:
                    energy_node = self._owned_uri(_pose_diff_energy_id(ctrl, component), motion)
                    acc_node = self._owned_uri(f"acc-cstr-{ctrl.name}-{component.suffix}", motion)
                    self.graph.add((acc_node, RDF.type, SLV.AccelerationConstraint))
                    self.graph.add((acc_node, RDF.type, SLV.AxisAligned))
                    self.graph.add((acc_node, SLV.subspace, SLV[component.subspace]))
                    self.graph.add((acc_node, SLV.axis, SLV[component.axis]))
                    self.graph.add((acc_node, SLV["acceleration-energy"], energy_node))
                    if axis_frame:
                        self.graph.add(
                            (acc_node, GEOM_COORD["as-seen-by"], self._owned_uri(axis_frame, qty))
                        )
                    acc_constraint_nodes.append(acc_node)

            if solver.algorithm == "ACHD" and axis is not None and axis_acceleration is not None:
                sid = _scalar_id(qty, subspace, axis)
                energy_stem = f"eacc-{sid}"
                energy_id = energy_stem if shared else f"{energy_stem}-{motion.name}"
                acc_stem = f"acc-cstr-{sid}"
                acc_id = acc_stem if shared else f"{acc_stem}-{motion.name}"

                acc_node = self._owned_uri(acc_id, motion)
                energy_node = self._owned_uri(energy_id, motion)
                self.graph.add((acc_node, RDF.type, SLV.AccelerationConstraint))
                self.graph.add((acc_node, RDF.type, SLV.AxisAligned))
                self.graph.add((acc_node, SLV.subspace, SLV[axis_acceleration.subspace]))
                self.graph.add((acc_node, SLV.axis, SLV[axis]))
                self.graph.add((acc_node, SLV["acceleration-energy"], energy_node))
                axis_frame = _quantity_axis_frame(qty)
                if axis_frame:
                    self.graph.add(
                        (acc_node, GEOM_COORD["as-seen-by"], self._owned_uri(axis_frame, qty))
                    )
                acc_constraint_nodes.append(acc_node)

            if command.is_force_command:
                force_signal_node = self._force_control_signal_node(ctrl, handler)
                wrench_node = self._emit_force_command_wrench(
                    ctrl, spec, qty, axis, force_signal_node, motion
                )
                spec_node = self._owned_uri(f"spec-{ctrl.name}", handler)
                self.graph.add((spec_node, RDF.type, SLV.CartesianForceSpecification))
                self.graph.add((spec_node, SLV.force, wrench_node))
                apply_at = getattr(ctrl, "apply_at", None)
                if apply_at is not None and hasattr(apply_at, "uri"):
                    self.graph.add((spec_node, SLV["attached-to"], URIRef(apply_at.uri)))
                self.graph.add((driver_node, SLV["cartesian-force"], spec_node))

            if (
                solver.algorithm == "ACHD"
                and command.is_posture_torque_command
                and qty.type == WorldQuantityType.JointPosition
            ):
                torque_id = f"tau-{ctrl.name}"
                torque_node = self._owned_uri(torque_id, handler)
                self.graph.add((torque_node, RDF.type, KC_STAT.JointForceCoordinate))

                spec_node = self._owned_uri(f"jf-spec-{ctrl.name}", handler)
                self.graph.add((spec_node, RDF.type, SLV.JointForceSpecification))
                self.graph.add((spec_node, SLV.force, torque_node))
                self.graph.add((solver_node, SLV["output"], URIRef(qty.uri)))
                joint_name = _geo_prop(
                    qty.props if isinstance(qty.props, GeometricProps) else None, "of"
                )
                if joint_name:
                    self.graph.add(
                        (spec_node, SLV["attached-to"], self._owned_uri(joint_name, qty))
                    )
                self.graph.add((driver_node, SLV["joint-force"], spec_node))

        for vel_solver in getattr(solver, "velocity_solvers", []):
            vs_node = self._owned_uri(vel_solver.name, handler)
            self.graph.add((vs_node, RDF.type, SLV.VelocityCompositionSolver))
            self.graph.add((vs_node, SLV.configuration, Literal(vel_solver.configuration)))
            v_qty = self._resolve_qty(vel_solver.velocity, world_qtys)
            if v_qty:
                self.graph.add((vs_node, SLV.velocity, URIRef(v_qty.uri)))

        for force_solver in getattr(solver, "force_solvers", []):
            fs_node = self._owned_uri(force_solver.name, handler)
            self.graph.add((fs_node, RDF.type, SLV.ForceDistributionSolver))
            self.graph.add((fs_node, SLV.configuration, Literal(force_solver.configuration)))
            f_qty = self._resolve_qty(force_solver.force, world_qtys)
            if f_qty:
                self.graph.add((fs_node, SLV.force, URIRef(f_qty.uri)))

        if acc_constraint_nodes:
            spec_acc_node = self._owned_uri(f"spec-acc-{stem}", motion)
            self.graph.add((spec_acc_node, RDF.type, SLV.AccelerationConstraintSpecification))
            attached_to: URIRef | None = None
            for ctrl_item in getattr(handler, "controllers", []):
                ctrl = ctrl_item.ref.controller if hasattr(ctrl_item, "ref") else ctrl_item
                if self._controller_solver(handler, ctrl) is not solver:
                    continue
                cref = ctrl.params.constraint
                spec = cref.constraint if hasattr(cref, "constraint") else None
                if spec is None:
                    continue
                qty = self._resolve_constraint_quantity(spec, world_qtys)
                props = qty.props if qty is not None and isinstance(qty.props, GeometricProps) else None
                target_name = _geo_prop(props, "of")
                if target_name:
                    attached_to = self._owned_uri(target_name, qty)
                    self.graph.add((attached_to, RDF.type, GEOM_ENT.SimplicialComplex))
                    break
            if attached_to is not None:
                self.graph.add((spec_acc_node, SLV["attached-to"], attached_to))
            for acc_node in acc_constraint_nodes:
                self.graph.add((spec_acc_node, SLV.constraints, acc_node))
            self.graph.add((driver_node, SLV["acceleration-constraint"], spec_acc_node))

    def _controller_solver(self, handler: ConstraintHandler, ctrl: ControllerEntry) -> Any:
        explicit = getattr(getattr(ctrl, "solver", None), "solver", None)
        if explicit is not None:
            return explicit
        solvers = [_resolved_solver(s) for s in getattr(handler, "solvers", [])]
        if len(solvers) == 1:
            return solvers[0]
        if ctrl.type == ControllerType.FeedForward:
            candidates = [solver for solver in solvers if str(solver.algorithm) == "CommandForwarding"]
        else:
            candidates = [solver for solver in solvers if str(solver.algorithm) != "CommandForwarding"]
        return candidates[0] if len(candidates) == 1 else None

    def _add_quantity(self, node: URIRef, scalar_type: Any) -> None:
        qkind = QUDT_KIND_BY_QUANTITY_TYPE.get(scalar_type)
        if qkind is None:
            qkind = QUDT_QKIND[scalar_type]
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((node, RDF.type, qkind))
        if scalar_type == QuantityType.Distance:
            self.graph.add((node, RDF.type, GEOM_COORD.LinearDistanceCoordinate))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], qkind))
        self.graph.add((node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(scalar_type, QUDT_UNIT.UNITLESS)))
