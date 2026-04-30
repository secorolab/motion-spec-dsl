# SPDX-License-Identifier: MPL-2.0
"""Simple RDF graph builder — walks from ConstraintHandlers, no caching layer."""

from __future__ import annotations

from typing import Any

from rdflib.graph import Dataset
from rdflib.namespace import Namespace, RDF
from rdflib.term import Literal, URIRef
from textx.scoping import get_included_models

from motion_spec.namespace import (
    APP, CSTR, CSTR_HDL, GEOM_COORD, GEOM_ENT, GEOM_OP, GEOM_REL,
    KC, MAP, MOT, QUDT_QKIND, QUDT_SCHEMA, QUDT_UNIT, RBDYN_COORD, RBDYN_ENT, RBDYN_OP, SLV,
)
from motion_spec_dsl.generators.classes import (
    BilateralConstraint, ConstraintHandler, ConstraintSpecification,
    ControllerEntry, ControllerMode, ContextRef, EqualityConstraint, GeoPropPair,
    GeometricProps, GreaterThanConstraint, LessThanConstraint, Model,
    MotionSpec, PostContextDecl, PreContextDecl, QuantityType, ScalarQuantity,
    SnapshotValue, SpecContextDecl, ContextQuantity, VectorQuantity, WorldContextDecl,
    WorldQuantity, WorldQuantityType,
    _resolved_spec, _resolved_solver,
)

# Each entry: (rdf_types, qkinds, units, prop_map)
# prop_map[subspace] = (view_subspace_uri, accel_subspace_uri, accel_prefix, scalar_type, view_rdf_type)
WORLD_SPECS: dict[WorldQuantityType, tuple] = {
    WorldQuantityType.VelocityTwist: (
        (GEOM_REL.VelocityTwist, GEOM_COORD.VelocityTwistCoordinate, GEOM_COORD.VectorXYZ),
        (QUDT_QKIND.AngularVelocity, QUDT_QKIND.LinearVelocity),
        (QUDT_UNIT["RAD-PER-SEC"], QUDT_UNIT["M-PER-SEC"]),
        {
            "angular": ("angular-velocity", "angular-acceleration", "ang", QuantityType.AngularVelocity, MAP.VelocityTwistCoordinateView),
            "linear":  ("linear-velocity",  "linear-acceleration",  "lin", QuantityType.LinearVelocity,  MAP.VelocityTwistCoordinateView),
        },
    ),
    WorldQuantityType.Wrench: (
        (RBDYN_ENT.Wrench, RBDYN_COORD.WrenchCoordinate, GEOM_COORD.VectorXYZ),
        (QUDT_QKIND.Torque, QUDT_QKIND.Force),
        (QUDT_UNIT["N-M"], QUDT_UNIT.N),
        {
            "torque": ("torque", None, None, QuantityType.Torque, MAP.WrenchCoordinateView),
            "force":  ("force",  None, None, QuantityType.Force,  MAP.WrenchCoordinateView),
        },
    ),
    WorldQuantityType.Pose: (
        (GEOM_REL.Pose, GEOM_COORD.PoseCoordinate, GEOM_COORD.DirectionCosineXYZ, GEOM_COORD.VectorXYZ),
        (QUDT_QKIND.PlaneAngle, QUDT_QKIND.Length),
        (QUDT_UNIT.UNITLESS, QUDT_UNIT.M),
        {
            "rotation": ("rotation", "angular-acceleration", "ang", QuantityType.PlaneAngle, MAP.PoseCoordinateView),
            "distance": ("position",  "linear-acceleration",  "lin", QuantityType.Distance,  MAP.PoseCoordinateView),
        },
    ),
    WorldQuantityType.JointPosition: (
        (QUDT_SCHEMA.Quantity,),
        (QUDT_QKIND.PlaneAngle,),
        (QUDT_UNIT.RAD,),
        {},
    ),
}

WORLD_STRUCTURE_TYPES: dict[WorldQuantityType, Any] = {
    WorldQuantityType.Frame:          GEOM_ENT.Frame,
    WorldQuantityType.Link:           GEOM_ENT.SimplicialComplex,
    WorldQuantityType.KinematicChain: GEOM_ENT.KinematicChain,
    WorldQuantityType.Gravity:        GEOM_ENT.UniformGravitationalField,
}

SCALAR_UNIT: dict[Any, Any] = {
    QuantityType.Pose:            QUDT_UNIT.UNITLESS,
    QuantityType.Position:        QUDT_UNIT.M,
    QuantityType.Orientation:     QUDT_UNIT.UNITLESS,
    QuantityType.AngularVelocity: QUDT_UNIT["RAD-PER-SEC"],
    QuantityType.LinearVelocity:  QUDT_UNIT["M-PER-SEC"],
    QuantityType.Torque:          QUDT_UNIT["N-M"],
    QuantityType.Force:           QUDT_UNIT.N,
    "Position":                   QUDT_UNIT.M,
    "LinearAcceleration":         QUDT_UNIT["M-PER-SEC2"],
    "AngularAcceleration":        QUDT_UNIT["RAD-PER-SEC2"],
    QuantityType.Angle:           QUDT_UNIT["RAD"],
    QuantityType.PlaneAngle:      QUDT_UNIT["RAD"],
    QuantityType.Distance:        QUDT_UNIT.M,
}

DSL_UNIT: dict[str, Any] = {
    "rad/s": QUDT_UNIT["RAD-PER-SEC"],
    "rad":   QUDT_UNIT["RAD"],
    "m/s":   QUDT_UNIT["M-PER-SEC"],
    "m":     QUDT_UNIT.M,
    "Nm":    QUDT_UNIT["N-M"],
    "N":     QUDT_UNIT.N,
    "deg/s": QUDT_UNIT["DEG-PER-SEC"],
    "deg":   QUDT_UNIT["DEG"],
    "cm/s":  QUDT_UNIT["CentiM-PER-SEC"],
    "cm":    QUDT_UNIT["CentiM"],
    "m/s2":  QUDT_UNIT["M-PER-SEC2"],
}

POSE_DOF_SPECS = [
    ("lin-x", "linear-acceleration", "x", "vel", 0),
    ("lin-y", "linear-acceleration", "y", "vel", 1),
    ("lin-z", "linear-acceleration", "z", "vel", 2),
    ("ang-x", "angular-acceleration", "x", "rot", 0),
    ("ang-y", "angular-acceleration", "y", "rot", 1),
    ("ang-z", "angular-acceleration", "z", "rot", 2),
]

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

SUBSPACE_ALIAS: dict[str, str] = {
    "angvel":      "angular",
    "linvel":      "linear",
    "orientation": "rotation",
    "position":    "distance",
    "force":       "force",
    "torque":      "torque",
}

CSTR_TYPE_NAME: dict[Any, str] = {
    QuantityType.PlaneAngle: QuantityType.Angle,
}

QUDT_KIND_BY_QUANTITY_TYPE: dict[Any, Any] = {
    QuantityType.Pose: GEOM_REL.Pose,
    QuantityType.Position: QUDT_QKIND.Position,
    QuantityType.Orientation: QUDT_QKIND.Direction,
    QuantityType.Vector: QUDT_QKIND.Vector,
}

GRAPH_BINDINGS: tuple[tuple[str, Any], ...] = (
    ("kc",          KC),
    ("geom-ent",    GEOM_ENT),
    ("geom-rel",    GEOM_REL),
    ("geom-coord",  GEOM_COORD),
    ("geom-op",     GEOM_OP),
    ("rbdyn-ent",   RBDYN_ENT),
    ("rbdyn-coord", RBDYN_COORD),
    ("qudt",        QUDT_SCHEMA),
    ("qkind",       QUDT_QKIND),
    ("unit",        QUDT_UNIT),
    ("map",         MAP),
    ("cstr",        CSTR),
    ("mot",         MOT),
    ("cstr-hdl",    CSTR_HDL),
    ("slv",         SLV),
)


def _ns_term(namespace: Any, name: str) -> URIRef:
    return URIRef(str(namespace._NS) + name)


def _node_name(value: Any) -> str:
    return value.name if hasattr(value, "name") else str(value)


def _body_name(name: str) -> str:
    for prefix in ("frame-", "link-"):
        if name.startswith(prefix):
            return name[len(prefix):]
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
    if _is_distance_view(constraint):
        return "distance"
    subspace = constraint.view.subspace
    if subspace is None:
        qty = constraint.view.quantity
        if isinstance(qty, WorldQuantity) and qty.type == WorldQuantityType.JointPosition:
            return "joint-position"
        if isinstance(qty, WorldQuantity) and qty.type == WorldQuantityType.Pose:
            return "pose"
        raise ValueError(f"Constraint '{constraint.name}' must define a view subspace.")
    raw = str(getattr(subspace, "value", subspace))
    if (
        isinstance(constraint.view.quantity, WorldQuantity)
        and constraint.view.quantity.type == WorldQuantityType.Pose
        and raw in {"position", "orientation"}
        and constraint.view.axis is None
    ):
        return raw
    return SUBSPACE_ALIAS.get(raw, raw)


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
            out.append(_resolved_spec(item))
    return out

class MotionSpecDatasetBuilder:

    def __init__(self, model: Model):
        self.model = model
        self.models = get_included_models(model)
        self.dataset = Dataset()
        for prefix, ns in GRAPH_BINDINGS:
            self.dataset.bind(prefix, ns)
        self.authored_handlers: list[ConstraintHandler] = [
            spec
            for m in self.models
            for spec in m.specs
            if isinstance(spec, ConstraintHandler)
        ]
        self.graph = self.dataset.default_graph
        self._default_ns_owner: Any | None = next(iter(self.authored_handlers), None)

    def build(self) -> tuple[Dataset, dict[str, str]]:
        handlers = self.authored_handlers

        shared_spec_ids = self._compute_shared_specs(handlers)

        context: dict[str, str] = {}
        for prefix, ns in GRAPH_BINDINGS:
            context[prefix] = str(ns._NS)

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
            self._emit_context_quantities(context_quantities)
            self._emit_constraints(motion, constraints, world_qtys)
            self._emit_motion_spec(motion)
            self._emit_scalar_views(motion, constraints, world_qtys)
            self._emit_map_operations(motion, constraints, world_qtys)
            self._emit_constraint_handler(
                handler, motion, constraints, world_qtys, shared_spec_ids, handler_order
            )
            self._emit_solvers(handler, motion, world_qtys, shared_spec_ids)

        return self.dataset, context

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
            if isinstance(ctx, WorldContextDecl):
                for item in ctx.declaration:
                    if isinstance(item, WorldQuantity):
                        qtys[item.name] = item
        for ctx in getattr(handler, "context", []):
            if isinstance(ctx, WorldContextDecl):
                for item in ctx.declaration:
                    if isinstance(item, WorldQuantity):
                        qtys.setdefault(item.name, item)
        return qtys

    def _collect_context_quantities(
        self, motion: MotionSpec, handler: ConstraintHandler
    ) -> dict[str, ContextQuantity]:
        quantities: dict[str, ContextQuantity] = {}
        for ctx in motion.context:
            if isinstance(ctx, (PreContextDecl, SpecContextDecl, PostContextDecl)):
                for item in ctx.declaration:
                    if isinstance(item, ContextQuantity):
                        quantities[item.name] = item
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

    def _resolve_qty(
        self, ref: Any, world_qtys: dict[str, WorldQuantity]
    ) -> WorldQuantity | None:
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
        start = self._resolve_qty(spec.view.distance_from, world_qtys)
        end = self._resolve_qty(spec.view.distance_to, world_qtys)
        if start is None or end is None:
            raise ValueError(f"Distance constraint '{spec.name}' references an unknown pose.")
        if start.type != WorldQuantityType.Pose or end.type != WorldQuantityType.Pose:
            raise ValueError(f"Distance constraint '{spec.name}' must reference Pose quantities.")
        if not isinstance(start.props, GeometricProps) or not isinstance(end.props, GeometricProps):
            raise ValueError(f"Distance constraint '{spec.name}' needs pose endpoints.")

        start_frame = _geo_prop(start.props, "of")
        end_frame = _geo_prop(end.props, "of")
        start_reference = _geo_prop(start.props, "wrt")
        end_reference = _geo_prop(end.props, "wrt")
        if None in {start_frame, end_frame, start_reference, end_reference}:
            raise ValueError(
                f"Distance constraint '{spec.name}' needs explicit 'of' and 'wrt' frames."
            )
        if start_reference != end_reference:
            raise ValueError(
                f"Distance constraint '{spec.name}' needs poses expressed with the same 'wrt' frame."
            )

        for quantity in world_qtys.values():
            if quantity.type != WorldQuantityType.Pose or not isinstance(quantity.props, GeometricProps):
                continue
            of_frame = _geo_prop(quantity.props, "of")
            wrt_frame = _geo_prop(quantity.props, "wrt")
            if (of_frame, wrt_frame) in {
                (end_frame, start_frame),
                (start_frame, end_frame),
            }:
                return quantity

        raise ValueError(
            f"Distance constraint '{spec.name}' needs a relative Pose between "
            f"'{start_frame}' and '{end_frame}'."
        )

    def _emit_pose_coordinate(
        self, node: URIRef, of_frame: str, wrt_frame: str, owner: Any
    ) -> None:
        self.graph.add((node, RDF.type, GEOM_REL.Pose))
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

    def _force_control_signal_node(self, ctrl: ControllerEntry, handler: ConstraintHandler) -> URIRef:
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
            self.graph.add((node, GEOM_COORD.x, Literal(str(x))))
            self.graph.add((node, GEOM_COORD.y, Literal(str(y))))
            self.graph.add((node, GEOM_COORD.z, Literal(str(z))))

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
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Position))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        self.graph.add((node, GEOM_REL.of, point_node))
        self.graph.add((node, GEOM_REL["with-respect-to"], point_node))
        self.graph.add((node, GEOM_COORD["as-seen-by"], as_seen_by))
        self.graph.add((node, GEOM_COORD.x, Literal("0.0")))
        self.graph.add((node, GEOM_COORD.y, Literal("0.0")))
        self.graph.add((node, GEOM_COORD.z, Literal("0.0")))

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
            raise ValueError(
                f"Force controller '{ctrl.name}' must specify 'apply at <link>'."
            )

        props = qty.props if isinstance(qty.props, GeometricProps) else None
        as_seen_by_name = _geo_prop(props, "as-seen-by") or _geo_prop(props, "wrt")
        if as_seen_by_name is None:
            raise ValueError(
                f"Force controller '{ctrl.name}' needs a frame from the constrained quantity."
            )
        as_seen_by_node = self._owned_uri(as_seen_by_name, qty)

        direction_node = self._owned_uri(f"direction-{ctrl.name}", motion)
        if qty.type == WorldQuantityType.Pose and _view_subspace(spec) == "distance":
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
        for qty in world_qtys.values():
            rdf_type = WORLD_STRUCTURE_TYPES.get(qty.type)
            if rdf_type is not None and WORLD_SPECS.get(qty.type) is None:
                self.graph.add((URIRef(qty.uri), RDF.type, rdf_type))

        # Implicit entities inferred from geo props — use default_ns_owner (first handler)
        implicit: dict[str, Any] = {}
        for qty in world_qtys.values():
            props = qty.props if isinstance(qty.props, GeometricProps) else None
            if qty.type == WorldQuantityType.Pose:
                for key in ("of", "wrt", "as-seen-by"):
                    t = _geo_prop(props, key)
                    if t:
                        implicit.setdefault(t, GEOM_ENT.Frame)
            elif qty.type == WorldQuantityType.VelocityTwist:
                for key in ("of", "wrt"):
                    t = _geo_prop(props, key)
                    if t:
                        implicit.setdefault(t, GEOM_ENT.SimplicialComplex)
                pt = _geo_prop(props, "ref-point")
                if pt:
                    implicit.setdefault(pt, GEOM_ENT.Point)
                fr = _geo_prop(props, "as-seen-by")
                if fr:
                    implicit.setdefault(fr, GEOM_ENT.Frame)
            elif qty.type == WorldQuantityType.Wrench:
                pt = _geo_prop(props, "ref-point")
                if pt:
                    implicit.setdefault(pt, GEOM_ENT.Point)
                fr = _geo_prop(props, "as-seen-by")
                if fr:
                    implicit.setdefault(fr, GEOM_ENT.Frame)
            elif qty.type == WorldQuantityType.JointPosition:
                t = _geo_prop(props, "of")
                if t:
                    implicit.setdefault(t, KC.Joint)

        for name, rdf_type in implicit.items():
            # Implicit authored names have no object owner; place them in the handler namespace.
            self.graph.add((self._owned_uri(name, None), RDF.type, rdf_type))

    def _emit_world_quantities(self, world_qtys: dict[str, WorldQuantity]) -> None:
        for qty in world_qtys.values():
            spec = WORLD_SPECS.get(qty.type)
            if spec is None:
                continue
            rdf_types, qkinds, units, _ = spec
            node = URIRef(qty.uri)
            for t in rdf_types:
                self.graph.add((node, RDF.type, t))
            for qk in qkinds:
                self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], qk))
            for u in units:
                self.graph.add((node, QUDT_SCHEMA.unit, u))

            props = qty.props if isinstance(qty.props, GeometricProps) else None
            of_v  = _geo_prop(props, "of")
            wrt_v = _geo_prop(props, "wrt")
            rp_v  = _geo_prop(props, "ref-point")
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
            if asb_v:
                asb_predicate = (
                    RBDYN_COORD["as-seen-by"]
                    if qty.type == WorldQuantityType.Wrench
                    else GEOM_COORD["as-seen-by"]
                )
                self.graph.add((node, asb_predicate, self._owned_uri(asb_v, qty)))
            elif qty.type == WorldQuantityType.Pose and wrt_v:
                self.graph.add((node, GEOM_COORD["as-seen-by"], self._owned_uri(wrt_v, qty)))

    def _view_node(self, view: Any, owner: Any) -> URIRef:
        if getattr(view, "distance_from", None) is not None and getattr(view, "distance_to", None) is not None:
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
            if quantity.type == WorldQuantityType.Pose and subspace in {"position", "orientation"} and axis_raw is None:
                mapped_subspace = subspace
            else:
                mapped_subspace = SUBSPACE_ALIAS.get(subspace, subspace)
            axis = None if axis_raw is None else str(getattr(axis_raw, "value", axis_raw))
            return self._owned_uri(_scalar_id(quantity, mapped_subspace, axis), owner)

        return self._owned_uri(_node_name(quantity), owner)

    def _emit_context_quantities(self, context_quantities: dict[str, ContextQuantity]) -> None:
        for quantity in context_quantities.values():
            node = URIRef(quantity.uri)
            qkind = QUDT_KIND_BY_QUANTITY_TYPE.get(quantity.type)
            if qkind is None:
                qkind = QUDT_QKIND[quantity.type]
            self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
            self.graph.add((node, RDF.type, qkind))
            self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], qkind))
            if quantity.value is None:
                continue
            if isinstance(quantity.value, SnapshotValue):
                self.graph.add((node, RDF.type, _ns_term(APP, "Snapshot")))
                self.graph.add((
                    node,
                    _ns_term(APP, "snapshot-of"),
                    self._view_node(quantity.value.source, quantity),
                ))
                self.graph.add((node, _ns_term(APP, "snapshot-time"), Literal("motion-start")))
                self.graph.add((node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(quantity.type, QUDT_UNIT.UNITLESS)))
                continue
            self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(quantity.value.unit)))
            if isinstance(quantity.value, ScalarQuantity):
                self.graph.add((node, QUDT_SCHEMA.value, Literal(str(quantity.value.value))))
            elif isinstance(quantity.value, VectorQuantity):
                self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
                self.graph.add((node, GEOM_COORD.x, Literal(str(quantity.value.x))))
                self.graph.add((node, GEOM_COORD.y, Literal(str(quantity.value.y))))
                self.graph.add((node, GEOM_COORD.z, Literal(str(quantity.value.z))))

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
            subspace = _view_subspace(spec)
            axis_raw = spec.view.axis
            axis = None if axis_raw is None else str(getattr(axis_raw, "value", axis_raw))
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
                reference_quantity = _context_quantity(expr.reference)
                ref_node = (
                    URIRef(reference_quantity.uri)
                    if isinstance(reference_quantity, ContextQuantity)
                    else self._owned_uri(_node_name(reference_quantity), motion)
                )
                self.graph.add((node, CSTR["reference-value"], ref_node))
            elif isinstance(expr, GreaterThanConstraint):
                self.graph.add((node, RDF.type, CSTR.UnilateralConstraint))
                self.graph.add((node, RDF.type, CSTR.GreaterThanConstraint))
                threshold_quantity = _context_quantity(expr.threshold)
                thr_node = (
                    URIRef(threshold_quantity.uri)
                    if isinstance(threshold_quantity, ContextQuantity)
                    else self._owned_uri(_node_name(threshold_quantity), motion)
                )
                self.graph.add((node, CSTR.threshold, thr_node))
            elif isinstance(expr, LessThanConstraint):
                self.graph.add((node, RDF.type, CSTR.UnilateralConstraint))
                self.graph.add((node, RDF.type, CSTR.LessThanConstraint))
                threshold_quantity = _context_quantity(expr.threshold)
                thr_node = (
                    URIRef(threshold_quantity.uri)
                    if isinstance(threshold_quantity, ContextQuantity)
                    else self._owned_uri(_node_name(threshold_quantity), motion)
                )
                self.graph.add((node, CSTR.threshold, thr_node))
            elif isinstance(expr, BilateralConstraint):
                self.graph.add((node, RDF.type, CSTR.BilateralConstraint))
                lower_quantity = _context_quantity(expr.lower)
                upper_quantity = _context_quantity(expr.upper)
                lo_node = (
                    URIRef(lower_quantity.uri)
                    if isinstance(lower_quantity, ContextQuantity)
                    else self._owned_uri(_node_name(lower_quantity), motion)
                )
                up_node = (
                    URIRef(upper_quantity.uri)
                    if isinstance(upper_quantity, ContextQuantity)
                    else self._owned_uri(_node_name(upper_quantity), motion)
                )
                self.graph.add((node, CSTR["lower-threshold"], lo_node))
                self.graph.add((node, CSTR["upper-threshold"], up_node))

    def _emit_motion_spec(self, motion: MotionSpec) -> None:
        motion_node = self._owned_uri(f"motion-{motion.name}", motion)
        self.graph.add((motion_node, RDF.type, MOT.GuardedMotion))
        for item in motion.when.constraints:
            self.graph.add((motion_node, MOT.when, URIRef(_resolved_spec(item).uri)))
        for item in motion.while_.constraints:
            self.graph.add((motion_node, MOT["while"], URIRef(_resolved_spec(item).uri)))
        for item in motion.until.constraints:
            self.graph.add((motion_node, MOT.until, URIRef(_resolved_spec(item).uri)))
        raw_logic = getattr(motion.until, "logic", None)
        self.graph.add((motion_node, MOT.untilLogic,
                        Literal(raw_logic if raw_logic in ("any", "all") else "any")))

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
            axis = None if axis_raw is None else str(getattr(axis_raw, "value", axis_raw))

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
                    self._add_quantity(self._owned_uri(sid, motion), QuantityType.Pose)
                continue

            if axis is None or prop is None or prop[4] is None:
                continue

            if qty.type == WorldQuantityType.Pose and subspace == "rotation":
                continue

            key = (qty.name, subspace, axis)
            if key in seen:
                continue
            seen.add(key)

            view_subspace_uri, _, _, scalar_t, view_type = prop
            sid = _scalar_id(qty, subspace, axis)
            scalar_node = self._owned_uri(sid, motion)
            self._add_quantity(scalar_node, scalar_t)

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
            axis = None if axis_raw is None else str(getattr(axis_raw, "value", axis_raw))
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
            axis = None if axis_raw is None else str(getattr(axis_raw, "value", axis_raw))
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
            axis = None if axis_raw is None else str(getattr(axis_raw, "value", axis_raw))
            if not _is_distance_view(spec) or _view_subspace(spec) != "distance" or axis is not None:
                continue
            if not isinstance(qty.props, GeometricProps):
                raise ValueError(
                    f"Distance derivation for '{qty.name}' needs pose endpoints."
                )
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
            start = self._resolve_qty(spec.view.distance_from, world_qtys)
            end = self._resolve_qty(spec.view.distance_to, world_qtys)
            if start is None or end is None:
                raise ValueError(f"Distance constraint '{spec.name}' references an unknown pose.")
            start_props = start.props if isinstance(start.props, GeometricProps) else None
            start_of = _geo_prop(start_props, "of")
            start_wrt = _geo_prop(start_props, "wrt")
            if start_of is None or start_wrt is None:
                raise ValueError(
                    f"Distance constraint '{spec.name}' needs explicit start pose endpoints."
                )
            inverse_id = f"inverse-{start.name}"
            inverse_node = self._owned_uri(inverse_id, motion)
            self._emit_pose_coordinate(inverse_node, start_wrt, start_of, motion)
            invert_node = self._owned_uri(f"compute-{inverse_id}", motion)
            self.graph.add((invert_node, RDF.type, GEOM_OP.InvertPose))
            self.graph.add((invert_node, GEOM_OP.pose, URIRef(start.uri)))
            self.graph.add((invert_node, GEOM_OP.out, inverse_node))

            compose_node = self._owned_uri(f"compute-{qty.name}", motion)
            self.graph.add((compose_node, RDF.type, GEOM_OP.ComposePose))
            self.graph.add((compose_node, GEOM_OP.in1, inverse_node))
            self.graph.add((compose_node, GEOM_OP.in2, URIRef(end.uri)))
            self.graph.add((compose_node, GEOM_OP.composite, URIRef(qty.uri)))

            distance_node = self._owned_uri(distance_id, motion)
            self._add_quantity(distance_node, QuantityType.Distance)
            op_node = self._owned_uri(f"compute-{distance_id}", motion)
            self.graph.add((op_node, RDF.type, GEOM_OP.PoseToLinearDistance))
            self.graph.add((op_node, GEOM_OP.pose, URIRef(qty.uri)))
            self.graph.add((op_node, GEOM_OP.distance, distance_node))

    def _emit_constraint_handler(
        self,
        handler: ConstraintHandler,
        motion: MotionSpec,
        constraints: list[ConstraintSpecification],
        world_qtys: dict[str, WorldQuantity],
        shared_spec_ids: frozenset[int],
        handler_order: int,
    ) -> None:
        handler_node = URIRef(handler.uri)
        self.graph.add((handler_node, RDF.type, CSTR_HDL.ConstraintHandler))
        self.graph.add((handler_node, APP.order, Literal(handler_order)))
        self.graph.add((handler_node, CSTR_HDL.motion, self._owned_uri(f"motion-{motion.name}", motion)))

        seen_error_ids: set[str] = set()
        seen_eval_ids: set[str] = set()

        for ctrl_item in getattr(handler, "controllers", []):
            ctrl = ctrl_item.ref.controller if hasattr(ctrl_item, "ref") else ctrl_item
            cref = ctrl.params.constraint
            spec = cref.constraint if hasattr(cref, "constraint") else None
            if spec is None:
                continue

            qty = self._resolve_constraint_quantity(spec, world_qtys)
            subspace = _view_subspace(spec)
            axis_raw = spec.view.axis
            axis = None if axis_raw is None else str(getattr(axis_raw, "value", axis_raw))
            shared = id(spec) in shared_spec_ids
            scalar_t = _scalar_type(qty, subspace, axis) if qty else subspace

            error_id: str | None = None
            if qty is not None:
                sid = _scalar_id(qty, subspace, axis)
                error_id = (sid + "-err") if shared else f"{sid}-err-{motion.name}"

            # Pose equality: expand to PoseDiffEvaluator + 6 per-DOF controllers
            if (qty is not None and qty.type == WorldQuantityType.Pose
                    and subspace == "pose" and isinstance(spec.expr, EqualityConstraint)):
                ref_qty = _context_quantity(spec.expr.reference)
                ref_uri = URIRef(ref_qty.uri) if ref_qty else None
                if ref_uri is not None:
                    eval_id = f"eval-pose-diff-{ctrl.name}"
                    if eval_id not in seen_eval_ids:
                        seen_eval_ids.add(eval_id)
                        eval_node = self._owned_uri(eval_id, spec.parent)
                        diff_id = f"pose-diff-{ctrl.name}"
                        diff_node = self._owned_uri(diff_id, spec.parent)
                        ref_point_node = self._owned_uri(f"point-{diff_id}-origin", spec.parent)
                        props = qty.props if isinstance(qty.props, GeometricProps) else None
                        as_seen_by = _geo_prop(props, "as-seen-by") or _geo_prop(props, "wrt")
                        if as_seen_by is None:
                            raise ValueError(
                                f"Pose hold constraint '{spec.name}' needs an as-seen-by or wrt frame."
                            )

                        self.graph.add((eval_node, RDF.type, GEOM_OP["PoseDiffEvaluator"]))
                        self.graph.add((eval_node, CSTR_HDL.constraint, URIRef(spec.uri)))
                        self.graph.add((eval_node, GEOM_OP.in1, ref_uri))
                        self.graph.add((eval_node, GEOM_OP.in2, URIRef(qty.uri)))
                        self.graph.add((eval_node, GEOM_OP.out, diff_node))
                        self.graph.add((diff_node, RDF.type, GEOM_REL.AccelerationTwist))
                        self.graph.add((diff_node, RDF.type, GEOM_COORD.AccelerationTwistCoordinate))
                        self.graph.add((diff_node, RDF.type, GEOM_COORD.VectorXYZ))
                        self.graph.add((diff_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.AngularAcceleration))
                        self.graph.add((diff_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.LinearAcceleration))
                        self.graph.add((diff_node, QUDT_SCHEMA.unit, QUDT_UNIT["RAD-PER-SEC2"]))
                        self.graph.add((diff_node, QUDT_SCHEMA.unit, QUDT_UNIT["M-PER-SEC2"]))
                        self.graph.add((diff_node, GEOM_REL["reference-point"], ref_point_node))
                        self.graph.add((diff_node, GEOM_COORD["as-seen-by"], self._owned_uri(as_seen_by, qty)))
                        self.graph.add((ref_point_node, RDF.type, GEOM_ENT.Point))

                        for (suffix, accel_subspace, axis_label, _, _) in POSE_DOF_SPECS:
                            err_id = f"{ctrl.name}-err-{suffix}"
                            err_node = self._owned_uri(err_id, spec.parent)
                            scalar_t = (
                                "LinearAcceleration"
                                if accel_subspace == "linear-acceleration"
                                else "AngularAcceleration"
                            )
                            self._add_quantity(err_node, scalar_t)
                            view_node = self._owned_uri(f"view-{err_id}", spec.parent)
                            self.graph.add((view_node, RDF.type, MAP.View))
                            self.graph.add((view_node, RDF.type, MAP.AccelerationTwistCoordinateView))
                            self.graph.add((view_node, MAP.superobject, diff_node))
                            self.graph.add((view_node, MAP.subobject, err_node))
                            self.graph.add((view_node, MAP.subspace, MAP[accel_subspace]))
                            self.graph.add((view_node, MAP.axis, MAP[axis_label]))
                        self.graph.add((handler_node, CSTR_HDL.evaluators, eval_node))

                    for (suffix, _, _, _, _) in POSE_DOF_SPECS:
                        err_id = f"{ctrl.name}-err-{suffix}"
                        comp_ctrl_id = f"{ctrl.name}-{suffix}"
                        energy_id = f"eacc-{ctrl.name}-{suffix}"
                        comp_ctrl_node = self._owned_uri(comp_ctrl_id, handler)
                        energy_node = self._owned_uri(energy_id, motion)
                        self.graph.add((comp_ctrl_node, RDF.type, CSTR_HDL.Controller))
                        self.graph.add((comp_ctrl_node, RDF.type, CSTR_HDL.ProportionalIntegralDerivative))
                        self.graph.add((comp_ctrl_node, CSTR_HDL["proportional-gain"], Literal(str(ctrl.params.kp))))
                        self.graph.add((comp_ctrl_node, CSTR_HDL["integral-gain"], Literal(str(ctrl.params.ki))))
                        self.graph.add((comp_ctrl_node, CSTR_HDL["derivative-gain"], Literal(str(ctrl.params.kd))))
                        self.graph.add((comp_ctrl_node, CSTR_HDL["error-signal"],
                                        self._owned_uri(err_id, spec.parent)))
                        self.graph.add((comp_ctrl_node, CSTR_HDL["control-signal"], energy_node))
                        self.graph.add((energy_node, RDF.type, QUDT_SCHEMA.Quantity))
                        self.graph.add((energy_node, RDF.type, QUDT_QKIND.AccelerationEnergy))
                        self.graph.add((energy_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.AccelerationEnergy))
                        self.graph.add((energy_node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M2-PER-SEC2"]))
                        self.graph.add((handler_node, CSTR_HDL.controllers, comp_ctrl_node))
                    continue

            ctrl_node = URIRef(ctrl.uri)
            self.graph.add((ctrl_node, RDF.type, CSTR_HDL.Controller))
            self.graph.add((ctrl_node, RDF.type, CSTR_HDL.ProportionalIntegralDerivative))
            self.graph.add((ctrl_node, CSTR_HDL["proportional-gain"], Literal(str(ctrl.params.kp))))
            self.graph.add((ctrl_node, CSTR_HDL["integral-gain"],     Literal(str(ctrl.params.ki))))
            self.graph.add((ctrl_node, CSTR_HDL["derivative-gain"],   Literal(str(ctrl.params.kd))))

            if error_id:
                self.graph.add((ctrl_node, CSTR_HDL["error-signal"],
                                self._owned_uri(error_id, spec.parent)))

            control_signal_node = self._decode_control_signal(
                ctrl, spec, qty, subspace, axis, motion, handler, shared
            )
            self.graph.add((ctrl_node, CSTR_HDL["control-signal"], control_signal_node))
            self.graph.add((handler_node, CSTR_HDL.controllers, ctrl_node))

            if error_id and error_id not in seen_error_ids:
                seen_error_ids.add(error_id)
                self._add_quantity(self._owned_uri(error_id, spec.parent), scalar_t)

            if error_id:
                eval_id = _evaluator_id(spec)
                if eval_id not in seen_eval_ids:
                    seen_eval_ids.add(eval_id)
                    eval_node = self._owned_uri(eval_id, spec.parent)
                    self.graph.add((eval_node, RDF.type, CSTR_HDL.ConstraintEvaluator))
                    self.graph.add((eval_node, RDF.type, CSTR_HDL.ErrorEvaluator))
                    self.graph.add((eval_node, CSTR_HDL.constraint, URIRef(spec.uri)))
                    self.graph.add((eval_node, CSTR_HDL.error,
                                    self._owned_uri(error_id, spec.parent)))
                self.graph.add((handler_node, CSTR_HDL.evaluators,
                                self._owned_uri(eval_id, spec.parent)))

        for mon in getattr(handler, "monitors", []):
            cref = mon.constraint
            spec = cref.constraint if hasattr(cref, "constraint") else None
            if spec is None:
                continue

            qty = self._resolve_constraint_quantity(spec, world_qtys)
            subspace = _view_subspace(spec)
            axis_raw = spec.view.axis
            axis = None if axis_raw is None else str(getattr(axis_raw, "value", axis_raw))
            scalar_t = _scalar_type(qty, subspace, axis) if qty else subspace
            qty_node_id = _scalar_id(qty, subspace, axis) if qty else spec.name
            error_id = f"{qty_node_id}-err"

            if error_id not in seen_error_ids:
                seen_error_ids.add(error_id)
                self._add_quantity(self._owned_uri(error_id, spec.parent), scalar_t)

            eval_id = _evaluator_id(spec)
            if eval_id not in seen_eval_ids:
                seen_eval_ids.add(eval_id)
                eval_node = self._owned_uri(eval_id, spec.parent)
                self.graph.add((eval_node, RDF.type, CSTR_HDL.ConstraintEvaluator))
                self.graph.add((eval_node, RDF.type, CSTR_HDL.ErrorEvaluator))
                self.graph.add((eval_node, CSTR_HDL.constraint, URIRef(spec.uri)))
                self.graph.add((eval_node, CSTR_HDL.error,
                                self._owned_uri(error_id, spec.parent)))

            signal_name = mon.event or mon.flag
            signal_kind = "event" if mon.event else "flag"
            signal_node = self._owned_uri(signal_name, handler)
            mon_node = URIRef(mon.uri)
            self.graph.add((signal_node, RDF.type,
                            CSTR_HDL.Event if signal_kind == "event" else CSTR_HDL.Flag))
            self.graph.add((mon_node, RDF.type, CSTR_HDL.Monitor))
            self.graph.add((mon_node, CSTR_HDL.constraint, URIRef(spec.uri)))
            self.graph.add((mon_node, CSTR_HDL.error, self._owned_uri(error_id, spec.parent)))
            if signal_kind == "event":
                self.graph.add((mon_node, RDF.type, CSTR_HDL.EdgeTriggeredMonitor))
                self.graph.add((mon_node, CSTR_HDL.event, signal_node))
            else:
                self.graph.add((mon_node, RDF.type, CSTR_HDL.LevelTriggeredMonitor))
                self.graph.add((mon_node, CSTR_HDL.flag, signal_node))
            self.graph.add((handler_node, CSTR_HDL.monitors, mon_node))
            self.graph.add((handler_node, CSTR_HDL.evaluators,
                            self._owned_uri(eval_id, spec.parent)))

    def _decode_control_signal(
        self,
        ctrl: ControllerEntry,
        spec: ConstraintSpecification,
        qty: WorldQuantity | None,
        subspace: str,
        axis: str | None,
        motion: MotionSpec,
        handler: ConstraintHandler,
        shared: bool,
    ) -> URIRef:
        solver = self._controller_solver(handler, ctrl)
        algorithm = getattr(solver, "algorithm", None)
        command_type = getattr(ctrl, "command_type", None)
        control_mode = getattr(ctrl, "control_mode", None)

        ws_spec = WORLD_SPECS.get(qty.type) if qty else None
        prop = ws_spec[3].get(subspace) if ws_spec else None
        accel_prefix = prop[2] if prop else None
        accel_subspace_label = prop[1] if prop else None

        # Cartesian force command: controller output is a force magnitude.
        if qty is not None and (subspace == "force" or command_type == QuantityType.Force):
            return self._force_control_signal_node(ctrl, handler)

        # ACHD acceleration energy
        if (
            algorithm == "ACHD"
            and qty is not None
            and axis is not None
            and accel_prefix is not None
            and accel_subspace_label is not None
        ):
            sid = _scalar_id(qty, subspace, axis)
            stem = f"eacc-{sid}"
            energy_id = stem if shared else f"{stem}-{motion.name}"
            energy_node = self._owned_uri(energy_id, motion)
            self.graph.add((energy_node, RDF.type, QUDT_SCHEMA.Quantity))
            self.graph.add((energy_node, RDF.type, QUDT_QKIND.AccelerationEnergy))
            self.graph.add((energy_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.AccelerationEnergy))
            self.graph.add((energy_node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M2-PER-SEC2"]))
            return energy_node

        # Posture joint torque
        if (
            algorithm == "ACHD"
            and command_type == QuantityType.Torque
            and control_mode == ControllerMode.Posture
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
            driver_stem = solver.name if multi else (motion.name or handler.name)

            driver_node = self._owned_uri(f"driver-{driver_stem}", handler)
            self.graph.add((driver_node, RDF.type, SLV.MotionDrivers))

            solver_node = self._owned_uri(solver_stem, solver)
            self.graph.add((solver_node, RDF.type, SLV.SolverWithInputAndOutput))

            alg = solver.algorithm
            alg_node = (
                SLV.AccelerationConstrainedHybridDynamicsAlgorithm if alg == "ACHD"
                else SLV.NewtonEulerAlgorithm if alg == "RNE"
                else SLV[alg]
            )
            self.graph.add((solver_node, SLV.solver, alg_node))

            root_node = self._owned_uri(_node_name(solver.root), handler)
            self.graph.add((root_node, RDF.type, GEOM_ENT.Frame))
            self.graph.add((solver_node, SLV.root, root_node))
            self.graph.add((solver_node, SLV["kinematic-chain"],
                            self._owned_uri(_node_name(solver.robot), handler)))

            gravity_qty = self._resolve_qty(solver.gravity, world_qtys)
            if gravity_qty is not None:
                self.graph.add((solver_node, SLV.gravity, URIRef(gravity_qty.uri)))
            gravity_ref = getattr(solver, "gravity_value", None)
            gravity_value = _context_quantity(gravity_ref) if gravity_ref is not None else None
            if gravity_value is not None:
                self.graph.add((solver_node, SLV["gravity-value"], URIRef(gravity_value.uri)))

            self.graph.add((solver_node, SLV["motion-drivers"], driver_node))

            robot_spec = solver.root.robot_spec
            self.graph.add((solver_node, APP["urdf"], Literal(robot_spec.urdf)))
            self.graph.add((solver_node, APP["robot-type"], Literal(str(robot_spec.type))))

            component_ref = solver.root.component
            chain_root_name = None
            if component_ref is not None:
                arm = next(
                    (m for m in robot_spec.manipulators if m.name == component_ref.component),
                    None,
                )
                if arm is not None:
                    self.graph.add((solver_node, APP["chain-root"], Literal(arm.root)))
                    self.graph.add((solver_node, APP["chain-end"],  Literal(arm.end)))
                    self.graph.add((solver_node, APP["robot-model"], Literal(arm.model)))
                    chain_root_name = arm.root
            else:
                chain = robot_spec.chain
                if chain is not None:
                    self.graph.add((solver_node, APP["chain-root"], Literal(chain.root)))
                    if chain.end:
                        self.graph.add((solver_node, APP["chain-end"], Literal(chain.end)))
                    self.graph.add((solver_node, APP["robot-model"], Literal(robot_spec.model)))
                    chain_root_name = chain.root

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
                handler, motion, solver, driver_stem, driver_node, world_qtys, shared_spec_ids
            )

    def _emit_solver_interfaces(
        self,
        handler: ConstraintHandler,
        motion: MotionSpec,
        solver: Any,
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
                continue

            subspace = _view_subspace(spec)
            axis_raw = spec.view.axis
            axis = None if axis_raw is None else str(getattr(axis_raw, "value", axis_raw))
            shared = id(spec) in shared_spec_ids

            ws_spec = WORLD_SPECS.get(qty.type)
            prop = ws_spec[3].get(subspace) if ws_spec else None
            accel_subspace_label = prop[1] if prop else None
            accel_prefix = prop[2] if prop else None
            command_type = getattr(ctrl, "command_type", None)
            control_mode = getattr(ctrl, "control_mode", None)
            is_force_command = subspace == "force" or command_type == QuantityType.Force

            if (
                solver.algorithm == "ACHD"
                and qty.type == WorldQuantityType.Pose
                and subspace == "pose"
                and not is_force_command
            ):
                for (suffix, accel_sub, axis_label, _, _) in POSE_DOF_SPECS:
                    energy_node = self._owned_uri(f"eacc-{ctrl.name}-{suffix}", motion)
                    acc_node = self._owned_uri(f"acc-cstr-{ctrl.name}-{suffix}", motion)
                    self.graph.add((acc_node, RDF.type, SLV.AccelerationConstraint))
                    self.graph.add((acc_node, RDF.type, SLV.AxisAligned))
                    self.graph.add((acc_node, SLV.subspace, SLV[accel_sub]))
                    self.graph.add((acc_node, SLV.axis, SLV[axis_label]))
                    self.graph.add((acc_node, SLV["acceleration-energy"], energy_node))
                    acc_constraint_nodes.append(acc_node)

            if (
                solver.algorithm == "ACHD"
                and axis is not None
                and accel_prefix is not None
                and accel_subspace_label is not None
                and not is_force_command
            ):
                sid = _scalar_id(qty, subspace, axis)
                energy_stem = f"eacc-{sid}"
                energy_id = energy_stem if shared else f"{energy_stem}-{motion.name}"
                acc_stem = f"acc-cstr-{sid}"
                acc_id = acc_stem if shared else f"{acc_stem}-{motion.name}"

                acc_node = self._owned_uri(acc_id, motion)
                energy_node = self._owned_uri(energy_id, motion)
                self.graph.add((acc_node, RDF.type, SLV.AccelerationConstraint))
                self.graph.add((acc_node, RDF.type, SLV.AxisAligned))
                self.graph.add((acc_node, SLV.subspace, SLV[accel_subspace_label]))
                self.graph.add((acc_node, SLV.axis, SLV[axis]))
                self.graph.add((acc_node, SLV["acceleration-energy"], energy_node))
                acc_constraint_nodes.append(acc_node)

            if is_force_command:
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
                and command_type == QuantityType.Torque
                and control_mode == ControllerMode.Posture
                and qty.type == WorldQuantityType.JointPosition
            ):
                jf_id = f"tau-{ctrl.name}"
                jf_node = self._owned_uri(jf_id, handler)
                self.graph.add((driver_node, SLV["joint-force"], jf_node))
                self.graph.add((jf_node, RDF.type, SLV.JointForce))

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

        for jf_name in getattr(solver, "joint_force", []):
            jf_node = self._owned_uri(jf_name, handler)
            self.graph.add((jf_node, RDF.type, SLV.JointForce))
            self.graph.add((driver_node, SLV["joint-force"], jf_node))

        if acc_constraint_nodes:
            spec_acc_node = self._owned_uri(f"spec-acc-{stem}", motion)
            self.graph.add((spec_acc_node, RDF.type, SLV.AccelerationConstraintSpecification))
            for acc_node in acc_constraint_nodes:
                self.graph.add((spec_acc_node, SLV.constraints, acc_node))
            self.graph.add((driver_node, SLV["acceleration-constraint"], spec_acc_node))

    def _controller_solver(self, handler: ConstraintHandler, ctrl: ControllerEntry) -> Any:
        explicit = getattr(getattr(ctrl, "solver", None), "solver", None)
        if explicit is not None:
            return explicit
        solvers = [_resolved_solver(s) for s in getattr(handler, "solvers", [])]
        return solvers[0] if len(solvers) == 1 else None

    def _add_quantity(self, node: URIRef, scalar_type: Any) -> None:
        qkind = QUDT_KIND_BY_QUANTITY_TYPE.get(scalar_type)
        if qkind is None:
            qkind = QUDT_QKIND[scalar_type]
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((node, RDF.type, qkind))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], qkind))
        self.graph.add((node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(scalar_type, QUDT_UNIT.UNITLESS)))
