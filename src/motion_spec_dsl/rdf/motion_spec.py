# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Emit motion-specification RDF/JSON-LD from a parsed DSL model.

`MotionSpecDatasetBuilder` walks the authored `ConstraintHandler`s and their motions
and writes the world quantities, constraints, controllers, coordinates and transform
operations as RDF triples for downstream SHACL validation and C++ code generation.

Resolution is kept separate from emission: frames, reference values and
controllers-per-constraint are resolved into Python indexes up front, and emission is
write-only (it never queries the graph back). Idempotency is tracked with emitted-node
registries rather than graph-membership checks.
"""

from __future__ import annotations

from typing import Any
from urllib.parse import urlsplit

from rdflib.graph import Dataset
from rdflib.namespace import Namespace, RDF, SDO, XSD
from rdflib.term import Literal, URIRef
from rdf_utils.collection import add_literal_list_pred
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.vocab import (
    URI_GEOM_PRED_ALPHA,
    URI_GEOM_PRED_AXES_SEQ,
    URI_GEOM_PRED_BETA,
    URI_GEOM_PRED_DIRECTION_COSINE_X,
    URI_GEOM_PRED_DIRECTION_COSINE_Y,
    URI_GEOM_PRED_DIRECTION_COSINE_Z,
    URI_GEOM_PRED_GAMMA,
    URI_GEOM_PRED_OF,
    URI_GEOM_PRED_OF_ORIENT,
    URI_GEOM_PRED_OF_POSITION,
    URI_GEOM_PRED_ORIGIN,
    URI_GEOM_PRED_SEEN_BY,
    URI_GEOM_PRED_WRT,
    URI_GEOM_PRED_W,
    URI_GEOM_PRED_X,
    URI_GEOM_PRED_Y,
    URI_GEOM_PRED_Z,
    URI_GEOM_TYPE_ANGLES_ABG,
    URI_GEOM_TYPE_DIRECTION_COSINE_XYZ,
    URI_GEOM_TYPE_EULER_ANGLES,
    URI_GEOM_TYPE_EXTRINSIC,
    URI_GEOM_TYPE_INTRINSIC,
    URI_GEOM_TYPE_FRAME,
    URI_GEOM_TYPE_QUATERNION,
    URI_GEOM_TYPE_ORIENT_REF,
    URI_GEOM_TYPE_POINT,
    URI_GEOM_TYPE_POSE,
    URI_GEOM_TYPE_POSITION_REF,
    URI_GEOM_TYPE_VECTOR_XYZ,
    URI_QUDT_QK_LENGTH,
    URI_QUDT_QK_MASS,
)
from rdf_utils.namespace import (
    NS_MM_GEOM_REL,
    NS_MM_QUDT_QTY,
    NS_MM_QUDT_UNIT as QUDT_UNIT,
)

from textx.scoping import get_included_models

from motion_spec_dsl.rdf_parser.vocab import (
    AGN,
    ALGO_EXT,
    EL,
    APP,
    CSTR,
    CSTR_EXT,
    CSTR_HDL,
    CSTR_HDL_EXT,
    EXEC,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_OP,
    GEOM_OP_EXT,
    GEOM_PATH,
    GEOM_REL,
    KC_STAT,
    MAP,
    MAP_EXT,
    MOT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    RBDYN_COORD,
    RBDYN_ENT,
    RBDYN_OP,
    SLV,
    SLV_EXT,
    SOSA,
    TIME,
)
from motion_spec_dsl.classes.controller_semantics import (
    SUBSPACE_ALIAS,
    axis_label as semantic_axis_label,
    controller_command_record,
    controller_solver,
)
from motion_spec_dsl.classes.constraint_handler import (
    ConstraintHandler,
    ControllerEntry,
    ControllerType,
    SaturationSpec,
    MobilePlatformSolver,
    CommandForwardingSolver,
    UntilMonitorRef,
    WhenMonitorRef,
    _resolved_solver,
)
from motion_spec_dsl.classes.constraints import (
    BilateralConstraint,
    OutsideConstraint,
    ConstraintGroup,
    ConstraintSpecification,
    EqualityConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    _flatten_constraint_items,
    _resolved_spec,
)
from motion_spec_dsl.classes.context import (
    ContextRef,
    GeoPropPair,
    GeometricPropKey,
    GeometricProps,
    QuantityType,
    ReferenceGeneratorType,
    ReferenceValue,
    Measure,
    SnapshotValue,
    ContextQuantity,
    VectorXYZ,
    WorldQuantity,
    WorldQuantityType,
    _resolved_context_quantity,
    _resolved_world_quantity,
)
from motion_spec_dsl.classes.coordinates import (
    AccelerationTwistCoordinate,
    Coordinates,
    OrientationCoordinate,
    PoseCoordinate,
    VelocityTwistCoordinate,
    WrenchCoordinate,
)
from motion_spec_dsl.classes.motion_spec import (
    ContextDeclReference,
    ContextSpec,
    ExecutionContext,
    Model,
    GuardedMotion,
    PostContextDecl,
    PreContextDecl,
    SpecContextDecl,
    WorldContextDecl,
)
from motion_spec_dsl.classes.path import (
    ProfileSpec,
    AdmittanceSpec,
    PathValue,
)

from motion_spec_dsl.rdf.model import (
    GEOM_DOMAIN_SPLIT,
    WORLD_SPECS,
    SCALAR_UNIT,
    CSTR_TYPE_NAME,
    CONSTRAINT_TYPE_OVERRIDE,
    QUDT_KIND_BY_QUANTITY_TYPE,
    _QKIND_PREFIXES,
    CONTEXT_COMPOSITE_WORLD_TYPE,
    GRAPH_BINDINGS,
    ROS,
)
from motion_spec_dsl.rdf.common import (
    _ns_term,
    _node_name,
    _geo_prop,
    _is_distance_view,
    _view_subspace,
    _scalar_id,
    _axis_vector,
    _scalar_type,
    _evaluator_id,
    ANGLE_UNITS,
    _angle_unit,
    _dsl_unit,
    _context_quantity,
    _resolved_constraint_items,
    _DistancePlan,
)


_MOBILE_PLATFORM_ALGORITHM_RDF: dict[str, tuple[URIRef, URIRef]] = {
    "VelocityComposition": (SLV.VelocityCompositionSolver, SLV.velocity),
    "VelocityDistribution": (SLV_EXT.VelocityDistributionSolver, SLV.velocity),
    "ForceDistribution": (SLV.ForceDistributionSolver, SLV.force),
    "ForceComposition": (SLV_EXT.ForceCompositionSolver, SLV.force),
}


def _path_operand(view: Any) -> Any | None:
    """The driver, geometry or guard operand of a view that follows a path."""
    return (
        getattr(view, "moving", None)
        or getattr(view, "on", None)
        or getattr(view, "progress", None)
    )


def _constraint_type_iri(scalar_t: Any) -> URIRef:
    """Grounded domain-constraint IRI for a scalar quantity type."""
    name = CSTR_TYPE_NAME.get(scalar_t, scalar_t)
    override = CONSTRAINT_TYPE_OVERRIDE.get(name)
    if override is not None:
        namespace, local = override
        return _ns_term(namespace, local)
    return _ns_term(CSTR, f"{name}Constraint")


def _pose_frame_names(quantity) -> tuple[str, str, str] | None:
    """(of, wrt, as-seen-by) frame URIs of a pose quantity, resolved through a snapshot's
    source quantity when the pose declares none. None when they are not resolvable."""
    props = quantity.props if isinstance(quantity.props, GeometricProps) else None
    if _geo_prop(props, "of") is None:
        source = getattr(getattr(quantity, "value", None), "source", None)
        source_props = getattr(getattr(source, "quantity", None), "props", None)
        props = source_props if isinstance(source_props, GeometricProps) else None
    of_frame = _geo_prop(props, "of")
    wrt_frame = _geo_prop(props, "wrt")
    if of_frame is None or wrt_frame is None:
        return None
    return of_frame, wrt_frame, _geo_prop(props, "as-seen-by") or wrt_frame


class MotionSpecDatasetBuilder:
    """Builds the motion-specification RDF dataset from a parsed DSL `Model`.

    Construct with the root model, then call `build()` to get the populated
    `Dataset` and its JSON-LD namespace context. One instance emits one dataset;
    the resolution indexes and emitted-node registries are per-build state.
    """

    def __init__(self, model: Model):
        """Set up the dataset, namespace bindings, authored handlers and per-build indexes."""
        self.model = model
        self.models = get_included_models(model)
        self.dataset = Dataset()
        for prefix, ns in GRAPH_BINDINGS:
            self.dataset.bind(prefix, ns)
        self.authored_handlers: list[ConstraintHandler] = [
            spec
            for m in self.models
            for spec in getattr(m, "specs", [])
            if isinstance(spec, ConstraintHandler)
        ]
        self.graph = self.dataset.default_graph
        self._default_ns_owner: Any | None = next(iter(self.authored_handlers), None)

        # Resolution indexes, populated once and read during emission (see module docstring).
        self._distance_plans: dict[ConstraintSpecification, _DistancePlan] = {}
        self._frame_coords_index: dict[URIRef, tuple[URIRef, URIRef, URIRef]] = {}
        self._reference_value_index: dict[URIRef, URIRef] = {}
        self._controller_by_spec: dict[ConstraintSpecification, ControllerEntry] = {}
        self._profiled_controller_by_spec: dict[ConstraintSpecification, ControllerEntry] = {}
        for handler in self.authored_handlers:
            for ctrl_item in getattr(handler, "controllers", []):
                ctrl = ctrl_item.ref.controller if hasattr(ctrl_item, "ref") else ctrl_item
                spec = getattr(ctrl.params.constraint, "constraint", None)
                if spec is not None:
                    self._controller_by_spec.setdefault(spec, ctrl)
                    if getattr(ctrl.params, "profile", None) is not None:
                        self._profiled_controller_by_spec.setdefault(spec, ctrl)

        # Emitted-node registries for idempotency (keep emission write-only).
        self._emitted_distance_ops: set[str] = set()
        self._emitted_views: set[URIRef] = set()
        self._emitted_position_coords: set[URIRef] = set()
        self._emitted_orientation_coords: set[URIRef] = set()
        self._path_projections: set[URIRef] = set()
        self._motion_time_endpoints_index: dict[str, tuple[URIRef, URIRef]] = {}

    def build(self) -> tuple[Dataset, dict[str, str]]:
        """Emit the full dataset and return it with its JSON-LD namespace context.

        Emits context specs once, then for each authored handler+motion runs
        the ordered emission phases (structural entities, world/context quantities,
        transforms, constraints, motion spec, scalar views, map ops, handler, solvers).
        The returned dict maps namespace prefixes to their URIs for JSON-LD serialization.
        """
        handlers = self.authored_handlers

        shared_spec_ids = self._compute_shared_specs(handlers)

        context: dict[str, str] = {}
        for prefix, ns in GRAPH_BINDINGS:
            context[prefix] = str(ns if isinstance(ns, Namespace) else ns._NS)

        for model in self.models:
            for spec in getattr(model, "specs", []):
                if isinstance(spec, ExecutionContext):
                    self._emit_execution_context(spec)
                    self.dataset.bind(spec.ns_prefix, spec.ns.uri)
                    context[spec.ns_prefix] = spec.ns.uri
                    scene = spec.scene
                    self.dataset.bind(scene.ns_prefix, scene.ns.uri)
                    context[scene.ns_prefix] = scene.ns.uri
                    for modelled_agent in scene.modelled_agns:
                        agent_set = modelled_agent.agn.parent
                        self.dataset.bind(agent_set.ns_prefix, agent_set.ns.uri)
                        context[agent_set.ns_prefix] = agent_set.ns.uri
                elif isinstance(spec, ContextSpec):
                    self.dataset.bind(spec.ns_prefix, spec.ns.uri)
                    context[spec.ns_prefix] = spec.ns.uri

        for handler_order, handler in enumerate(handlers):
            motion = handler.motion
            if not isinstance(motion, GuardedMotion):
                continue

            self.dataset.bind(handler.ns_prefix, handler.ns.uri)
            self.dataset.bind(motion.ns_prefix, motion.ns.uri)
            context[handler.ns_prefix] = handler.ns.uri
            context[motion.ns_prefix] = motion.ns.uri

            context_quantities = self._collect_context_quantities(motion, handler)
            world_qtys = self._collect_world_quantities(motion, handler, context_quantities)
            constraints = _resolved_constraint_items(motion)

            self._emit_world_quantities(world_qtys)
            self._emit_context_quantities(context_quantities, constraints, world_qtys)
            self._emit_path_following(constraints, world_qtys)
            self._emit_constraints(motion, constraints, world_qtys)
            self._emit_motion_spec(motion)
            self._emit_scalar_views(motion, constraints, world_qtys)
            self._emit_map_operations(motion, constraints, world_qtys)
            self._emit_constraint_handler(
                handler, motion, world_qtys, shared_spec_ids, handler_order
            )
            self._emit_solvers(handler, motion, world_qtys)

        return self.dataset, context

    def _emit_execution_context(self, context: ExecutionContext) -> None:
        """Emit the authored scene, platform, and control-period binding."""
        node = URIRef(context.uri)
        self.graph.add((node, RDF.type, EXEC.ExecutionContext))
        self.graph.add(
            (
                node,
                RDF.type,
                EXEC.Simulation if context.platform.kind == "simulation" else EXEC.RealWorld,
            )
        )
        self.graph.add((node, EXEC["runs-scene"], URIRef(context.scene.uri)))
        timestep = URIRef(f"{context.uri}.timestep")
        self._emit_scalar_quantity(
            timestep,
            float(context.timestep),
            NS_MM_QUDT_QTY["Time"],
            _dsl_unit(context.timestep_unit),
        )
        self.graph.add((node, EXEC.timestep, timestep))
        if context.platform.name:
            self.graph.add((node, EXEC["platform-name"], Literal(context.platform.name)))
        if context.platform.version:
            self.graph.add((node, EXEC["platform-version"], Literal(context.platform.version)))

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
        if urlsplit(str(name)).scheme:
            return URIRef(name)
        ns_uri = str(self._namespace_owner(owner).ns.uri)
        return Namespace(ns_uri)[name]

    def _emit_scalar_quantity(
        self, node: URIRef, value: float, qkind: URIRef | None, unit: URIRef
    ) -> URIRef:
        """Emit a unit-bearing qudt:Quantity node wrapping a single float value."""
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        if qkind is not None:
            self._emit_quantity_kind(node, qkind)
        self.graph.add((node, QUDT_SCHEMA.unit, unit))
        self.graph.add((node, QUDT_SCHEMA.value, Literal(float(value), datatype=XSD.double)))
        return node

    def _emit_quantity_kind(self, node: URIRef, qkind: URIRef) -> None:
        """Type `node` with `qkind`: always via hasQuantityKind, plus rdf:type for
        structural (non-QUDT) kinds. QUDT quantity-kinds are individuals, not classes.
        """
        if not str(qkind).startswith(_QKIND_PREFIXES):
            self.graph.add((node, RDF.type, qkind))
        elif qkind == QUDT_QKIND.PlaneAngle:
            # The angle shapes express quantity-kind membership with sh:class.
            self.graph.add((node, RDF.type, qkind))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], qkind))


    def _emit_geom_relation(
        self,
        coord_node: URIRef,
        domain: str,
        of_node: URIRef | None,
        wrt_node: URIRef | None,
        as_seen_by: URIRef | None = None,
        qkinds: tuple[URIRef, ...] = (),
    ) -> URIRef:
        """Split a geometry quantity into comp-rob2b's relation/coordinate pair.

        The relation says what the quantity is about (`of`, `with-respect-to`,
        `hasQuantityKind`); `coord_node` becomes the coordinate, typed
        `<Domain>Reference` + `<Domain>Coordinate` and linked back by `of-<domain>`,
        carrying only `as-seen-by`, `unit` and the literal values.
        """
        ref_type, coord_type, of_pred, rel_type = GEOM_DOMAIN_SPLIT[domain]
        rel_node = URIRef(f"{coord_node}-{domain}-rel")
        self.graph.add((rel_node, RDF.type, rel_type))
        self.graph.add((rel_node, RDF.type, QUDT_SCHEMA.Quantity))
        if of_node is not None:
            self.graph.add((rel_node, URI_GEOM_PRED_OF, of_node))
        if wrt_node is not None:
            self.graph.add((rel_node, URI_GEOM_PRED_WRT, wrt_node))
        for qkind in qkinds:
            self.graph.add((rel_node, QUDT_SCHEMA["hasQuantityKind"], qkind))
        self.graph.add((coord_node, RDF.type, ref_type))
        self.graph.add((coord_node, RDF.type, coord_type))
        self.graph.add((coord_node, of_pred, rel_node))
        if as_seen_by is not None:
            self.graph.add((coord_node, URI_GEOM_PRED_SEEN_BY, as_seen_by))
        return rel_node

    def _frame_origin(self, frame: URIRef) -> URIRef:
        """Return the frame's origin Point, materializing the established fallback URI."""
        origin = self.graph.value(frame, URI_GEOM_PRED_ORIGIN)
        if not isinstance(origin, URIRef):
            origin = URIRef(f"{frame}-origin")
            self.graph.add((frame, URI_GEOM_PRED_ORIGIN, origin))
        self.graph.add((frame, RDF.type, URI_GEOM_TYPE_FRAME))
        self.graph.add((origin, RDF.type, URI_GEOM_TYPE_POINT))
        return origin

    def _emit_combined_pose_coordinate(
        self, node: URIRef, pose_relation: URIRef, orientation: OrientationCoordinate | None = None
    ) -> None:
        """Complete a runtime pose coordinate whose position and orientation share one node."""
        pose_of, pose_wrt, pose_asb = self._frame_coords(node)
        position_relation = self._emit_geom_relation(
            node,
            "position",
            self._frame_origin(pose_of),
            self._frame_origin(pose_wrt),
            pose_asb,
            (URI_QUDT_QK_LENGTH,),
        )
        orientation_relation = self._emit_geom_relation(
            node, "orientation", pose_of, pose_wrt, pose_asb, (QUDT_QKIND.PlaneAngle,)
        )
        self.graph.add((node, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
        if orientation is not None:
            self._emit_orientation_type(node, orientation)
        self.graph.add((pose_relation, RDF.type, URI_GEOM_TYPE_POSITION_REF))
        self.graph.add((pose_relation, RDF.type, URI_GEOM_TYPE_ORIENT_REF))
        self.graph.add((pose_relation, URI_GEOM_PRED_OF_POSITION, position_relation))
        self.graph.add((pose_relation, URI_GEOM_PRED_OF_ORIENT, orientation_relation))

    @staticmethod
    def _add_world_quantity(
        qtys: dict[str, WorldQuantity], quantity: Any, *, overwrite: bool = False
    ) -> None:
        """Resolve `quantity` and insert it into `qtys` keyed by name (setdefault unless `overwrite`)."""
        if isinstance(quantity, WorldQuantity):
            quantity = _resolved_world_quantity(quantity)
            if overwrite:
                qtys[quantity.name] = quantity
            else:
                qtys.setdefault(quantity.name, quantity)

    def _add_view_world_quantities(self, qtys: dict[str, WorldQuantity], view: Any) -> None:
        """Collect the WorldQuantities a view references (quantity, distance_from/to) into `qtys`."""
        if view is None:
            return
        for attr in ("quantity", "distance_from", "distance_to"):
            self._add_world_quantity(qtys, getattr(view, attr, None))

    def _add_value_world_quantities(self, qtys: dict[str, WorldQuantity], value: Any) -> None:
        """Collect WorldQuantities referenced by a context value's source (snapshot/profile/admittance)."""
        if isinstance(value, SnapshotValue):
            self._add_view_world_quantities(qtys, value.source)
        elif isinstance(value, ProfileSpec):
            self._add_view_world_quantities(qtys, value.measured_velocity)
        elif isinstance(value, AdmittanceSpec):
            self._add_view_world_quantities(qtys, value.force)

    def _collect_world_quantities(
        self,
        motion: GuardedMotion,
        handler: ConstraintHandler,
        context_quantities: dict[str, ContextQuantity] | None = None,
    ) -> dict[str, WorldQuantity]:
        """Gather every world quantity in scope for a motion: context declarations, constraint
        views, context-quantity sources, controller measured-derivatives and solver gravity.
        """
        context_quantities = context_quantities or self._collect_context_quantities(motion, handler)
        qtys: dict[str, WorldQuantity] = {}
        for model in self.models:
            for spec in getattr(model, "specs", []):
                if not isinstance(spec, ContextSpec):
                    continue
                for ctx in spec.context:
                    if isinstance(ctx, WorldContextDecl):
                        for item in ctx.declaration:
                            self._add_world_quantity(qtys, item)
        for ctx in motion.context:
            ctx = self._resolved_context_decl(ctx)
            if isinstance(ctx, WorldContextDecl):
                for item in ctx.declaration:
                    self._add_world_quantity(qtys, item, overwrite=True)
        for ctx in getattr(handler, "context", []):
            ctx = self._resolved_context_decl(ctx)
            if isinstance(ctx, WorldContextDecl):
                for item in ctx.declaration:
                    self._add_world_quantity(qtys, item)
        for constraint in _resolved_constraint_items(motion):
            self._add_view_world_quantities(qtys, constraint.view)
        for quantity in context_quantities.values():
            self._add_value_world_quantities(qtys, getattr(quantity, "value", None))
        for ctrl_item in getattr(handler, "controllers", []):
            ctrl = ctrl_item.ref.controller if hasattr(ctrl_item, "ref") else ctrl_item
            self._add_view_world_quantities(qtys, getattr(ctrl.params, "measured_derivative", None))
            self._add_world_quantity(qtys, getattr(ctrl, "apply_at", None))
        for solver in getattr(handler, "solvers", []):
            solver = _resolved_solver(solver)
            self._add_world_quantity(qtys, getattr(solver, "gravity", None))
        return qtys

    @staticmethod
    def _resolved_context_decl(ctx: Any) -> Any:
        """Dereference a context-decl reference to its target, or return `ctx` unchanged."""
        if isinstance(ctx, ContextDeclReference):
            return ctx.ref
        return ctx

    def _collect_context_quantities(
        self, motion: GuardedMotion, handler: ConstraintHandler
    ) -> dict[str, ContextQuantity]:
        """Gather every context quantity in scope for a motion: context declarations, constraint
        references/thresholds, solver gravity values and controller profiles.
        """
        quantities: dict[str, ContextQuantity] = {}
        for model in self.models:
            for spec in getattr(model, "specs", []):
                if not isinstance(spec, ContextSpec):
                    continue
                for ctx in spec.context:
                    if isinstance(ctx, (PreContextDecl, SpecContextDecl, PostContextDecl)):
                        for item in ctx.declaration:
                            if isinstance(item, ContextQuantity):
                                quantities.setdefault(item.name, _resolved_context_quantity(item))
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
            elif isinstance(expr, (BilateralConstraint, OutsideConstraint)):
                refs = [expr.lower, expr.upper]
            for ref in refs:
                quantity = _context_quantity(ref)
                if isinstance(quantity, ContextQuantity):
                    quantities.setdefault(quantity.name, quantity)
        for solver in getattr(handler, "solvers", []):
            s = _resolved_solver(solver)
            gv = getattr(s, "gravity_value", None)
            if gv is not None:
                quantity = _context_quantity(getattr(gv, "ref", None))
                if isinstance(quantity, ContextQuantity):
                    quantities.setdefault(quantity.name, quantity)
        for ctrl_item in getattr(handler, "controllers", []):
            ctrl = ctrl_item.ref.controller if hasattr(ctrl_item, "ref") else ctrl_item
            quantity = _context_quantity(getattr(ctrl.params, "profile", None))
            if isinstance(quantity, ContextQuantity):
                quantities.setdefault(quantity.name, _resolved_context_quantity(quantity))
        return quantities

    def _resolve_qty(self, ref: Any, world_qtys: dict[str, WorldQuantity]) -> WorldQuantity | None:
        """Resolve a quantity reference to its WorldQuantity via `world_qtys`, or pass one through."""
        if isinstance(ref, WorldQuantity):
            return ref
        return world_qtys.get(_node_name(ref))

    def _resolve_constraint_quantity(
        self,
        spec: ConstraintSpecification,
        world_qtys: dict[str, WorldQuantity],
    ) -> WorldQuantity | None:
        """The WorldQuantity a constraint acts on: None for elapsed views, the relative pose for
        distance views, else the view's quantity.
        """
        if getattr(spec.view, "is_elapsed", False):
            return None
        if _is_distance_view(spec):
            return self._distance_plan(spec, world_qtys).target
        return self._resolve_qty(spec.view.quantity, world_qtys)

    def _pose_frames(self, quantity: WorldQuantity, context: str) -> tuple[str, str]:
        """The (of, wrt) frame names of a fully-specified Pose quantity; raises otherwise."""
        if quantity.type != WorldQuantityType.Pose or not isinstance(
            quantity.props, GeometricProps
        ):
            raise ValueError(f"{context} needs Pose quantities with explicit endpoints.")
        of_frame = _geo_prop(quantity.props, "of")
        wrt_frame = _geo_prop(quantity.props, "wrt")
        if of_frame is None or wrt_frame is None:
            raise ValueError(f"{context} needs explicit 'of' and 'wrt' frames.")
        return of_frame, wrt_frame

    def _distance_plan(
        self,
        spec: ConstraintSpecification,
        world_qtys: dict[str, WorldQuantity],
    ) -> _DistancePlan:
        """Resolve an authored distance relation's pose endpoints and scalar-view carrier."""
        cached = self._distance_plans.get(spec)
        if cached is not None:
            return cached

        start = self._resolve_qty(spec.view.distance_from, world_qtys)
        end = self._resolve_qty(spec.view.distance_to, world_qtys)
        if start is None or end is None:
            raise ValueError(f"Distance constraint '{spec.name}' references an unknown pose.")
        if start.type != WorldQuantityType.Pose or end.type != WorldQuantityType.Pose:
            raise ValueError(f"Distance constraint '{spec.name}' must reference Pose quantities.")
        context = f"Distance constraint '{spec.name}'"
        start_frame, _ = self._pose_frames(start, context)
        end_frame, _ = self._pose_frames(end, context)
        props = GeometricProps(
            [
                GeoPropPair(GeometricPropKey.Of, end_frame),
                GeoPropPair(GeometricPropKey.Wrt, start_frame),
                GeoPropPair(GeometricPropKey.AsSeenBy, start_frame),
            ]
        )
        target = WorldQuantity(
            parent=getattr(getattr(spec, "parent", None), "parent", None),
            name=f"distance-{spec.name}",
            type=WorldQuantityType.Pose,
            props=props,
        )
        plan = _DistancePlan(start, end, target)
        self._distance_plans[spec] = plan
        return plan

    def _frame_coords(self, node: URIRef) -> tuple[URIRef, URIRef, URIRef] | None:
        """Resolved (of, wrt, as-seen-by) frame nodes recorded when `node`'s pose
        coordinate was emitted, or None if none were emitted for it."""
        return self._frame_coords_index.get(node)

    def _emit_view(self, view_node: URIRef) -> None:
        """Type a node as a map:View and record it, so idempotency checks read the
        Python registry instead of querying the graph."""
        self.graph.add((view_node, RDF.type, MAP.View))
        self._emitted_views.add(view_node)

    def _emit_snapshot_position_metadata(
        self,
        node: URIRef,
        quantity: ContextQuantity,
    ) -> None:
        """Tag a snapshot of a `<pose>.position` with Position-coordinate metadata so the IR
        surfaces it as a Position vector rather than a plain scalar.
        """
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
        self.graph.add((node, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
        self._emit_geom_relation(
            node,
            "position",
            self._owned_uri(of_frame, source_qty),
            self._owned_uri(wrt_frame, source_qty),
            self._owned_uri(as_seen_by, source_qty),
            (URI_QUDT_QK_LENGTH,),
        )

    def _emit_declared_pose_frame_metadata(
        self,
        node: URIRef,
        quantity: ContextQuantity,
    ) -> URIRef | None:
        """Attach explicitly authored of/wrt/as-seen-by frames to a context pose."""
        frames = _pose_frame_names(quantity)
        if frames is None:
            return None
        of_frame, wrt_frame, as_seen_by = frames
        of_node = self._owned_uri(of_frame, quantity)
        wrt_node = self._owned_uri(wrt_frame, quantity)
        seen_by_node = self._owned_uri(as_seen_by, quantity)
        relation = self._emit_geom_relation(
            node,
            "pose",
            of_node,
            wrt_node,
            seen_by_node,
            (QUDT_QKIND.PlaneAngle, URI_QUDT_QK_LENGTH),
        )
        self._frame_coords_index[node] = (of_node, wrt_node, seen_by_node)
        return relation

    @staticmethod
    def _path_pose_endpoints(quantity: ContextQuantity) -> tuple[ContextQuantity, ...]:
        """Pose-valued endpoints whose frame relation defines a geometric path."""
        value = quantity.value
        for spec_name, endpoint_names in (
            ("lerp", ("start", "goal")),
            ("arc", ("start", "end")),
            ("circle", ("start",)),
            ("helix", ("start",)),
            ("figure8", ("anchor",)),
        ):
            spec = getattr(value, spec_name, None)
            if spec is None:
                continue
            return tuple(
                endpoint
                for name in endpoint_names
                if (endpoint := _context_quantity(getattr(spec, name, None))) is not None
            )
        return ()

    def _path_frame_nodes(self, quantity: ContextQuantity) -> tuple[URIRef, URIRef, URIRef]:
        """Resolve and validate the single frame tuple shared by a path's pose endpoints."""
        endpoint_frames = [
            (
                endpoint,
                tuple(self._owned_uri(name, endpoint) for name in frames),
            )
            for endpoint in self._path_pose_endpoints(quantity)
            if (frames := _pose_frame_names(endpoint)) is not None
        ]
        if not endpoint_frames:
            raise ConstraintViolation(
                "geometry", f"Path '{quantity.name}' has no endpoint with frame metadata"
            )
        distinct = {frames for _, frames in endpoint_frames}
        if len(distinct) != 1:
            details = ", ".join(
                f"{endpoint.name}={tuple(map(str, frames))}" for endpoint, frames in endpoint_frames
            )
            raise ConstraintViolation(
                "geometry", f"Path '{quantity.name}' endpoint frames disagree: {details}"
            )
        return endpoint_frames[0][1]

    def _force_control_signal_node(
        self, ctrl: ControllerEntry, handler: ConstraintHandler
    ) -> URIRef:
        """Owned Force-quantity node carrying a force controller's control signal."""
        signal_node = self._owned_uri(f"force-{ctrl.name}", handler)
        self._add_quantity(signal_node, QuantityType.Force)
        return signal_node

    def _emit_direction_coordinate(
        self,
        node: URIRef,
        as_seen_by: URIRef,
        vector: tuple[float, float, float] | None = None,
    ) -> None:
        """Emit a unit direction coordinate (Dimensionless VectorXYZ, as-seen-by `as_seen_by`),
        with optional explicit x/y/z components.
        """
        self.graph.add((node, RDF.type, NS_MM_GEOM_REL["Direction"]))
        self.graph.add((node, RDF.type, GEOM_COORD.DirectionCoordinate))
        self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
        # A direction is a normalized (unit) vector: its quantity kind is Dimensionless.
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], NS_MM_QUDT_QTY["Dimensionless"]))
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
        """Emit a zero-valued Position coordinate at `point_node` (a force command's application point), as seen by `as_seen_by`."""
        self.graph.add((point_node, RDF.type, GEOM_ENT.Point))
        self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        self._emit_geom_relation(
            node, "position", point_node, point_node, as_seen_by, (URI_QUDT_QK_LENGTH,)
        )
        self.graph.add((node, GEOM_COORD.x, Literal(0.0, datatype=XSD.double)))
        self.graph.add((node, GEOM_COORD.y, Literal(0.0, datatype=XSD.double)))
        self.graph.add((node, GEOM_COORD.z, Literal(0.0, datatype=XSD.double)))

    def _emit_wrench_coordinate(
        self,
        node: URIRef,
        reference_point: URIRef,
        as_seen_by: URIRef,
        acts_on: URIRef | None = None,
    ) -> URIRef:
        """Emit a Wrench relation and its coordinate/reference node."""
        relation = URIRef(f"{node}-wrench-rel")
        self.graph.add((relation, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((relation, RDF.type, RBDYN_ENT.Wrench))
        self.graph.add((relation, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Torque))
        self.graph.add((relation, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Force))
        self.graph.add((relation, RBDYN_ENT["reference-point"], reference_point))
        if acts_on is not None:
            self.graph.add((relation, RBDYN_ENT["acts-on"], acts_on))

        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((node, RDF.type, RBDYN_COORD.WrenchReference))
        self.graph.add((node, RDF.type, RBDYN_COORD.WrenchCoordinate))
        self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M"]))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.N))
        self.graph.add((node, RBDYN_COORD["of-wrench"], relation))
        self.graph.add((node, RBDYN_COORD["as-seen-by"], as_seen_by))
        return relation

    def _emit_pose_to_direction(
        self,
        direction_node: URIRef,
        pose_qty: WorldQuantity,
        as_seen_by_node: URIRef,
        motion: GuardedMotion,
        stem: str,
    ) -> URIRef:
        """Runtime unit geom-rel:Direction from a pose (geom-op:PoseToDirection),
        as seen by `as_seen_by_node` -- shared by the force-command distance path
        and the ACHD direction-aligned distance-control path."""
        self._emit_direction_coordinate(direction_node, as_seen_by_node)
        op_node = self._owned_uri(f"compute-direction-{stem}", motion)
        self.graph.add((op_node, RDF.type, GEOM_OP.PoseToDirection))
        self.graph.add((op_node, GEOM_OP.pose, URIRef(pose_qty.uri)))
        self.graph.add((op_node, GEOM_OP.direction, direction_node))
        return direction_node

    def _emit_force_command_wrench(
        self,
        ctrl: ControllerEntry,
        spec: ConstraintSpecification,
        qty: WorldQuantity,
        axis: str | None,
        magnitude_node: URIRef,
        motion: GuardedMotion,
    ) -> URIRef:
        """Emit the op chain building a force controller's command wrench from `magnitude_node`
        and a direction -- an axis unit vector, or a runtime pose-to-direction for a distance
        view. Returns the wrench node.
        """
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
        if (
            qty.type == WorldQuantityType.Pose
            and _view_subspace(spec) == "distance"
            and axis is None
        ):
            self._emit_pose_to_direction(direction_node, qty, as_seen_by_node, motion, ctrl.name)
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

    def _compute_shared_specs(
        self, handlers: list[ConstraintHandler]
    ) -> frozenset[ConstraintSpecification]:
        """Constraint specs referenced by more than one motion, whose controllers/solvers must
        therefore be emitted once rather than per motion.
        """
        usage: dict[ConstraintSpecification, set[str]] = {}
        for handler in handlers:
            motion = handler.motion
            if not isinstance(motion, GuardedMotion):
                continue
            motion_uri = str(motion.uri)
            seen: set[ConstraintSpecification] = set()
            for spec in _resolved_constraint_items(motion):
                if spec in seen:
                    continue
                seen.add(spec)
                usage.setdefault(spec, set()).add(motion_uri)
        return frozenset(spec for spec, motions in usage.items() if len(motions) > 1)

    def _emit_world_quantities(self, world_qtys: dict[str, WorldQuantity]) -> None:
        """Emit RDF typing and geometric relations for each world quantity."""
        for qty in world_qtys.values():
            spec = WORLD_SPECS.get(qty.type)
            if spec is None:
                continue
            if qty.type == WorldQuantityType.Pose:
                node = URIRef(qty.uri)
                self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
                self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
                self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
                frames = _pose_frame_names(qty)
                if frames is None:
                    raise ConstraintViolation("geometry", f"Pose '{node}' has no frame endpoints")
                of_frame, wrt_frame, as_seen_by = frames
                of_node = self._owned_uri(of_frame, qty)
                wrt_node = self._owned_uri(wrt_frame, qty)
                seen_by_node = self._owned_uri(as_seen_by, qty)
                self._frame_coords_index[node] = (of_node, wrt_node, seen_by_node)
                pose_relation = self._emit_geom_relation(
                    node,
                    "pose",
                    of_node,
                    wrt_node,
                    seen_by_node,
                    (QUDT_QKIND.PlaneAngle, URI_QUDT_QK_LENGTH),
                )
                self._emit_combined_pose_coordinate(node, pose_relation)
                continue
            if qty.type == WorldQuantityType.Wrench:
                node = URIRef(qty.uri)
                props = qty.props if isinstance(qty.props, GeometricProps) else None
                ft_sensor = (
                    next(
                        (
                            pair.sensor
                            for pair in props.pairs
                            if isinstance(pair, GeoPropPair)
                            and pair.key == "ft-sensor"
                            and pair.sensor is not None
                        ),
                        None,
                    )
                    if props is not None
                    else None
                )
                sensor_frame_name = str(ft_sensor.frame.uri) if ft_sensor is not None else None
                reference_name = _geo_prop(props, "ref-point") or sensor_frame_name
                reference_point = (
                    self._owned_uri(reference_name, qty)
                    if reference_name
                    else self._owned_uri(f"point-{qty.name}-origin", qty)
                )
                self.graph.add((reference_point, RDF.type, GEOM_ENT.Point))
                seen_name = _geo_prop(props, "as-seen-by") or sensor_frame_name
                if seen_name is None:
                    raise ConstraintViolation(
                        "dynamics", f"Wrench '{qty.name}' has no as-seen-by frame"
                    )
                acts_on_name = _geo_prop(props, "of")
                self._emit_wrench_coordinate(
                    node,
                    reference_point,
                    self._owned_uri(seen_name, qty),
                    self._owned_uri(acts_on_name, qty) if acts_on_name else None,
                )
                ft_ref = str(ft_sensor.uri) if ft_sensor is not None else None
                if ft_ref:
                    self.graph.add((node, RDF.type, SOSA.Observation))
                    self.graph.add((node, SOSA.madeBySensor, URIRef(ft_ref)))
                continue
            rdf_types, qkinds, units, _ = spec
            node = URIRef(qty.uri)
            for t in rdf_types:
                self.graph.add((node, RDF.type, t))
            for qk in qkinds:
                self._emit_quantity_kind(node, qk)
            for u in units:
                self.graph.add((node, QUDT_SCHEMA.unit, u))

            props = qty.props if isinstance(qty.props, GeometricProps) else None
            of_v = _geo_prop(props, "of")
            wrt_v = _geo_prop(props, "wrt")
            rp_v = _geo_prop(props, "ref-point")
            asb_v = _geo_prop(props, "as-seen-by")

            if qty.type == WorldQuantityType.JointPosition:
                joint = _geo_prop(props, "joint")
                if joint:
                    self.graph.add((node, KC_STAT["of-joint"], self._owned_uri(joint, qty)))

            if of_v:
                self.graph.add((node, GEOM_REL.of, self._owned_uri(of_v, qty)))
            if wrt_v:
                self.graph.add((node, GEOM_REL["with-respect-to"], self._owned_uri(wrt_v, qty)))
            if rp_v:
                ref_node = self._owned_uri(rp_v, qty)
                self.graph.add((node, GEOM_REL["reference-point"], ref_node))
            elif qty.type == WorldQuantityType.VelocityTwist:
                point_node = self._owned_uri(f"point-{qty.name}-origin", qty)
                self.graph.add((point_node, RDF.type, GEOM_ENT.Point))
                self.graph.add((node, GEOM_REL["reference-point"], point_node))
            if qty.type == WorldQuantityType.Pose and wrt_v:
                self.graph.add((node, GEOM_COORD["as-seen-by"], self._owned_uri(wrt_v, qty)))
            elif asb_v:
                self.graph.add((node, GEOM_COORD["as-seen-by"], self._owned_uri(asb_v, qty)))
            elif qty.type == WorldQuantityType.VelocityTwist and wrt_v:
                self.graph.add((node, GEOM_COORD["as-seen-by"], self._owned_uri(wrt_v, qty)))

            if qty.type == WorldQuantityType.Pose and of_v and wrt_v:
                self._frame_coords_index[node] = (
                    self._owned_uri(of_v, qty),
                    self._owned_uri(wrt_v, qty),
                    self._owned_uri(wrt_v, qty),
                )

    def _view_node(self, view: Any, owner: Any) -> URIRef:
        """Resolve a constraint/controller view to its RDF value node."""
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
            if subspace_raw is None and quantity.type in {
                WorldQuantityType.Pose,
                WorldQuantityType.VelocityTwist,
                WorldQuantityType.Wrench,
                WorldQuantityType.JointPosition,
            }:
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
                self._register_pose_component_view(
                    scalar_uri, quantity, mapped_subspace, axis, owner
                )
            elif (
                quantity.type == WorldQuantityType.Pose
                and mapped_subspace in {"distance", "position"}
                and axis is None
            ):
                self._register_pose_position_view(scalar_uri, quantity, owner)
            elif (
                quantity.type == WorldQuantityType.Pose
                and mapped_subspace in {"orientation", "rotation"}
                and axis is None
            ):
                self._register_pose_orientation_view(scalar_uri, quantity, owner)
            elif axis is not None:
                self._register_world_component_view(
                    scalar_uri, quantity, mapped_subspace, axis, owner
                )
            elif quantity.type == WorldQuantityType.Wrench and mapped_subspace in {
                "force",
                "torque",
            }:
                # Whole force/torque 3-vector (no axis).
                self._register_wrench_vector_view(scalar_uri, quantity, mapped_subspace, owner)
            if quantity.type == WorldQuantityType.Pose and axis is None:
                if mapped_subspace in {"distance", "position"}:
                    return URIRef(f"{quantity.uri}-position-rel")
                if mapped_subspace in {"orientation", "rotation"}:
                    return URIRef(f"{quantity.uri}-orientation-rel")
            return scalar_uri

        return self._owned_uri(_node_name(quantity), owner)

    def _register_wrench_vector_view(
        self,
        scalar_uri: URIRef,
        quantity: "WorldQuantity",
        mapped_subspace: str,
        owner: Any,
    ) -> None:
        """Register a wrench view exposing its whole force/torque vector."""
        # Whole force/torque 3-vector view (no axis); resolves to shared.<w>.force|.torque.
        prop = WORLD_SPECS.get(quantity.type, (None, None, None, {}))[3].get(mapped_subspace)
        scalar_t = prop[3] if prop is not None else QuantityType.Force
        self._add_quantity(scalar_uri, scalar_t)
        view_uri = self._owned_uri(f"view-{_scalar_id(quantity, mapped_subspace, None)}", owner)
        if view_uri in self._emitted_views:
            return
        self._emit_view(view_uri)
        self.graph.add((view_uri, RDF.type, MAP_EXT.WrenchCoordinateView))
        self.graph.add((view_uri, MAP.superobject, URIRef(quantity.uri)))
        self.graph.add((view_uri, MAP.subobject, scalar_uri))
        self.graph.add((view_uri, MAP.subspace, MAP[mapped_subspace]))

    def _register_pose_position_view(
        self,
        scalar_uri: URIRef,
        quantity: "WorldQuantity",
        owner: Any,
    ) -> None:
        """Promote `<pose>.position` and register its whole-vector coordinate view."""
        if scalar_uri in self._emitted_position_coords:
            return
        self._emitted_position_coords.add(scalar_uri)
        position_relation = URIRef(f"{quantity.uri}-position-rel")

        view_uri = self._owned_uri(f"view-{_scalar_id(quantity, 'position', None)}", owner)
        if view_uri not in self._emitted_views:
            self._emit_view(view_uri)
            self.graph.add((view_uri, RDF.type, MAP_EXT.PoseCoordinateView))
            self.graph.add((view_uri, MAP.superobject, URIRef(quantity.uri)))
            self.graph.add((view_uri, MAP.subobject, position_relation))
            self.graph.add((view_uri, MAP.subspace, MAP_EXT.position))
            # axis intentionally omitted: this view exposes the whole 3-vector.

    def _register_pose_component_view(
        self,
        scalar_uri: "URIRef",
        quantity: "WorldQuantity",
        mapped_subspace: str,
        axis: str,
        owner: Any,
    ) -> None:
        """Register a per-axis pose component view (position distance / orientation angle)."""
        sid = _scalar_id(quantity, mapped_subspace, axis)
        view_uri = self._owned_uri(f"view-{sid}", owner)
        if view_uri in self._emitted_views:
            return
        pose_specs = WORLD_SPECS[WorldQuantityType.Pose][3]
        props = pose_specs.get(mapped_subspace)
        if props is None:
            return
        view_subspace_uri, _, _, scalar_t, view_type = props
        self._add_quantity(scalar_uri, scalar_t)
        self._emit_view(view_uri)
        self.graph.add((view_uri, MAP.superobject, URIRef(quantity.uri)))
        self.graph.add((view_uri, MAP.subobject, scalar_uri))
        subspace = MAP_EXT.orientation if view_subspace_uri == "rotation" else MAP_EXT.position
        self.graph.add((view_uri, MAP.subspace, subspace))
        self.graph.add((view_uri, MAP.axis, MAP[axis]))

    @staticmethod
    def _is_euler_orientation(orientation: OrientationCoordinate | None) -> bool:
        """True for an Euler-represented (or unresolvable/derived) orientation -- the only
        case the RAD-unit, PlaneAngle-kind pairing applies to."""
        return orientation is None or (
            orientation.quat is None and orientation.direction_cosine is None
        )

    def _emit_orientation_type(
        self, node: URIRef, orientation: OrientationCoordinate | None
    ) -> None:
        """Type an orientation coordinate node by the representation the author actually
        wrote, and emit or withdraw its PlaneAngle-kind/angle-unit pairing to match:
        Euler angles are an angular scalar per axis in the unit they were written in,
        quaternion and direction-cosine components are dimensionless.

        `orientation=None` covers sites with no authored coordinate to inspect (a
        runtime-decomposed view, or a quantity typed 'orientation' directly); those
        default to extrinsic-XYZ Euler, this DSL's KDL backend convention (RPY).
        """
        quat = orientation.quat if orientation is not None else None
        direction_cosine = orientation.direction_cosine if orientation is not None else None
        euler = None
        if quat is not None:
            self.graph.add((node, RDF.type, URI_GEOM_TYPE_QUATERNION))
        elif direction_cosine is not None:
            self.graph.add((node, RDF.type, URI_GEOM_TYPE_DIRECTION_COSINE_XYZ))
        else:
            euler = orientation.euler if orientation is not None else None
            self.graph.add((node, RDF.type, URI_GEOM_TYPE_EULER_ANGLES))
            if euler is not None and all(element.ref is None for element in euler.angles.values):
                self.graph.add((node, RDF.type, URI_GEOM_TYPE_ANGLES_ABG))
            # `extrinsic` is an optional keyword, so its absence means intrinsic.
            extrinsic = euler.extrinsic if euler is not None else True
            self.graph.add(
                (node, RDF.type, URI_GEOM_TYPE_EXTRINSIC if extrinsic else URI_GEOM_TYPE_INTRINSIC)
            )
            self.graph.add((node, URI_GEOM_PRED_AXES_SEQ, Literal(euler.axes if euler else "xyz")))

        if self._is_euler_orientation(orientation):
            self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.PlaneAngle))
            self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(_angle_unit(euler))))
        else:
            self.graph.remove((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.PlaneAngle))
            for angle_unit in ANGLE_UNITS:
                self.graph.remove((node, QUDT_SCHEMA.unit, angle_unit))

    def _register_pose_orientation_view(
        self,
        scalar_uri: URIRef,
        quantity: "WorldQuantity",
        owner: Any,
    ) -> None:
        """Promote `<pose>.orientation` and register its coordinate view."""
        if scalar_uri in self._emitted_orientation_coords:
            return
        self._emitted_orientation_coords.add(scalar_uri)
        orientation_relation = URIRef(f"{quantity.uri}-orientation-rel")

        view_uri = self._owned_uri(f"view-{_scalar_id(quantity, 'orientation', None)}", owner)
        if view_uri not in self._emitted_views:
            self._emit_view(view_uri)
            self.graph.add((view_uri, RDF.type, MAP_EXT.PoseCoordinateView))
            self.graph.add((view_uri, MAP.superobject, URIRef(quantity.uri)))
            self.graph.add((view_uri, MAP.subobject, orientation_relation))
            self.graph.add((view_uri, MAP.subspace, MAP_EXT.orientation))

    def _register_world_component_view(
        self,
        scalar_uri: URIRef,
        quantity: WorldQuantity,
        mapped_subspace: str,
        axis: str | None,
        owner: Any,
    ) -> None:
        """Register a per-axis component view for a non-pose world-quantity subspace."""
        prop = WORLD_SPECS.get(quantity.type, (None, None, None, {}))[3].get(mapped_subspace)
        if prop is None or prop[4] is None:
            return
        view_subspace_uri, _, _, scalar_t, view_type = prop
        self._add_quantity(scalar_uri, scalar_t)
        view_uri = self._owned_uri(f"view-{_scalar_id(quantity, mapped_subspace, axis)}", owner)
        if view_uri in self._emitted_views:
            return
        self._emit_view(view_uri)
        self.graph.add((view_uri, RDF.type, view_type))
        self.graph.add((view_uri, MAP.superobject, URIRef(quantity.uri)))
        self.graph.add((view_uri, MAP.subobject, scalar_uri))
        self.graph.add((view_uri, MAP.subspace, MAP[view_subspace_uri]))
        if axis is not None:
            self.graph.add((view_uri, MAP.axis, MAP[axis]))

    def _emit_context_composite_metadata(
        self,
        node: URIRef,
        quantity: ContextQuantity,
        constraints: list[ConstraintSpecification],
        world_qtys: dict[str, WorldQuantity],
    ) -> None:
        """Emit composite RDF metadata (types, quantity-kinds, units and frame/reference-point
        relations) for a context quantity whose value is a twist/wrench/pose/acceleration.
        """
        world_type = CONTEXT_COMPOSITE_WORLD_TYPE.get(quantity.type)
        if world_type is None and quantity.type != QuantityType.AccelerationTwist:
            return

        if quantity.type == QuantityType.Pose:
            self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
            self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
            self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
            pose_relation = self._emit_declared_pose_frame_metadata(node, quantity)
            if pose_relation is None:
                raise ConstraintViolation("geometry", f"Pose '{node}' has no frame endpoints")
            self._emit_combined_pose_coordinate(node, pose_relation)
            return

        if quantity.type == QuantityType.Wrench:
            props = quantity.props if isinstance(quantity.props, GeometricProps) else None
            source = getattr(quantity.value, "source", None)
            source_qty = getattr(source, "quantity", None)
            if props is None and isinstance(source_qty, WorldQuantity):
                props = source_qty.props if isinstance(source_qty.props, GeometricProps) else None
                owner = source_qty
            else:
                owner = quantity
            reference_name = _geo_prop(props, "ref-point")
            reference_point = (
                self._owned_uri(reference_name, owner)
                if reference_name
                else self._owned_uri(f"point-{quantity.name}-origin", quantity)
            )
            self.graph.add((reference_point, RDF.type, GEOM_ENT.Point))
            seen_name = _geo_prop(props, "as-seen-by")
            if seen_name is None:
                raise ConstraintViolation(
                    "dynamics", f"Wrench '{quantity.name}' has no as-seen-by frame"
                )
            acts_on_name = _geo_prop(props, "of")
            self._emit_wrench_coordinate(
                node,
                reference_point,
                self._owned_uri(seen_name, owner),
                self._owned_uri(acts_on_name, owner) if acts_on_name else None,
            )
            return

        if world_type is not None:
            rdf_types, qkinds, units, _ = WORLD_SPECS[world_type]
            for rdf_type in rdf_types:
                self.graph.add((node, RDF.type, rdf_type))
            for qkind in qkinds:
                self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], qkind))
            for unit in units:
                self.graph.add((node, QUDT_SCHEMA.unit, unit))
        else:
            self.graph.add((node, RDF.type, GEOM_REL.AccelerationTwist))
            self.graph.add((node, RDF.type, GEOM_COORD.AccelerationTwistCoordinate))
            self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
            self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.AngularAcceleration))
            self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.LinearAcceleration))
            self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT["RAD-PER-SEC2"]))
            self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT["M-PER-SEC2"]))

        props = quantity.props if isinstance(quantity.props, GeometricProps) else None
        source = getattr(quantity.value, "source", None)
        source_qty = getattr(source, "quantity", None)
        if props is None and isinstance(source_qty, WorldQuantity):
            props = source_qty.props if isinstance(source_qty.props, GeometricProps) else None
            owner = source_qty
        else:
            owner = quantity

        if quantity.type in {QuantityType.VelocityTwist, QuantityType.AccelerationTwist}:
            of_v = _geo_prop(props, "of")
            wrt_v = _geo_prop(props, "wrt")
            rp_v = _geo_prop(props, "ref-point")
            asb_v = _geo_prop(props, "as-seen-by") or wrt_v
            if of_v:
                self.graph.add((node, GEOM_REL.of, self._owned_uri(of_v, owner)))
            if wrt_v:
                self.graph.add(
                    (
                        node,
                        GEOM_REL["with-respect-to"],
                        self._owned_uri(wrt_v, owner),
                    )
                )
            point_node = (
                self._owned_uri(rp_v, owner)
                if rp_v
                else self._owned_uri(f"point-{quantity.name}-origin", quantity)
            )
            self.graph.add((point_node, RDF.type, GEOM_ENT.Point))
            self.graph.add((node, GEOM_REL["reference-point"], point_node))
            if asb_v:
                self.graph.add((node, GEOM_COORD["as-seen-by"], self._owned_uri(asb_v, owner)))
            return

    def _context_ref_view_spec(
        self,
        quantity: ContextQuantity,
        subspace_raw: str,
        axis: str | None,
    ) -> tuple[Any, Any, Any] | None:
        """The (scalar type, view type, view subspace) for a context pose/path reference's
        subspace+axis, or None when that subspace exposes no view.
        """
        if quantity.type in {QuantityType.Pose, ReferenceGeneratorType.Path}:
            if subspace_raw == "position":
                return (
                    QuantityType.Distance if axis is not None else QuantityType.Position,
                    MAP_EXT.PoseCoordinateView,
                    MAP_EXT.position,
                )
            if subspace_raw == "orientation":
                return (
                    QuantityType.Angle if axis is not None else QuantityType.Orientation,
                    MAP_EXT.PoseCoordinateView,
                    MAP_EXT.orientation,
                )
        if quantity.type == QuantityType.VelocityTwist:
            return {
                "linvel": (
                    QuantityType.LinearVelocity,
                    MAP_EXT.VelocityTwistCoordinateView,
                    MAP["linear-velocity"],
                ),
                "angvel": (
                    QuantityType.AngularVelocity,
                    MAP_EXT.VelocityTwistCoordinateView,
                    MAP["angular-velocity"],
                ),
            }.get(subspace_raw)
        if quantity.type == QuantityType.AccelerationTwist:
            return {
                "linacc": (
                    QuantityType.LinearAcceleration,
                    MAP_EXT.AccelerationTwistCoordinateView,
                    MAP["linear-acceleration"],
                ),
                "angacc": (
                    QuantityType.AngularAcceleration,
                    MAP_EXT.AccelerationTwistCoordinateView,
                    MAP["angular-acceleration"],
                ),
            }.get(subspace_raw)
        if quantity.type == QuantityType.Wrench:
            return {
                "force": (QuantityType.Force, MAP_EXT.WrenchCoordinateView, MAP.force),
                "torque": (QuantityType.Torque, MAP_EXT.WrenchCoordinateView, MAP.torque),
            }.get(subspace_raw)
        return None

    @staticmethod
    def _reference_output_node(quantity: ContextQuantity) -> URIRef:
        """Return the quantity produced by a reference generator declaration."""
        return URIRef(f"{quantity.uri}/reference")

    def _emit_context_ref_view_node(
        self,
        quantity: ContextQuantity,
        subspace_raw: str,
        axis: str | None,
    ) -> URIRef:
        """Emit (once) the scalar/coordinate node and its map:View for a subspace of a context
        pose/path reference, and return the value node.
        """
        view_spec = self._context_ref_view_spec(quantity, subspace_raw, axis)
        if view_spec is None:
            return URIRef(quantity.uri)
        scalar_type, view_type, view_subspace = view_spec
        suffix = f"{subspace_raw}" + (f".{axis}" if axis is not None else "")
        node = URIRef(f"{quantity.uri}.{suffix}")
        super_node = (
            self._reference_output_node(quantity)
            if quantity.type == ReferenceGeneratorType.Path
            else URIRef(quantity.uri)
        )
        view_target = node
        if (
            quantity.type in {QuantityType.Pose, ReferenceGeneratorType.Path}
            and subspace_raw == "position"
        ):
            if axis is None:
                position_coord = (
                    URIRef(f"{quantity.uri}.position")
                    if isinstance(quantity.value, PoseCoordinate)
                    else super_node
                )
                view_target = URIRef(f"{position_coord}-position-rel")
        elif (
            quantity.type in {QuantityType.Pose, ReferenceGeneratorType.Path}
            and subspace_raw == "orientation"
            and axis is None
        ):
            orientation_coord = (
                URIRef(f"{quantity.uri}.orientation")
                if isinstance(quantity.value, PoseCoordinate)
                else super_node
            )
            view_target = URIRef(f"{orientation_coord}-orientation-rel")

        if view_target == node:
            self._add_quantity(node, scalar_type)

        view_node = URIRef(f"{quantity.uri}.view-{suffix}")
        if view_node not in self._emitted_views:
            self._emit_view(view_node)
            if axis is None or quantity.type not in {
                QuantityType.Pose,
                ReferenceGeneratorType.Path,
            }:
                self.graph.add((view_node, RDF.type, view_type))
            self.graph.add((view_node, MAP.superobject, super_node))
            self.graph.add((view_node, MAP.subobject, view_target))
            self.graph.add((view_node, MAP.subspace, view_subspace))
            if axis is not None:
                self.graph.add((view_node, MAP.axis, MAP[axis]))
        return view_target

    def _emit_context_quantities(
        self,
        context_quantities: dict[str, ContextQuantity],
        constraints: list[ConstraintSpecification],
        world_qtys: dict[str, WorldQuantity],
    ) -> None:
        """Emit every context quantity. Paths, directions, profiles, admittance and pose
        values dispatch to their own emitters; the rest get quantity-kind typing, composite
        metadata and their value (reference, snapshot with optional offset, measure or vector).
        """
        for quantity in context_quantities.values():
            node = URIRef(quantity.uri)
            if quantity.type == ReferenceGeneratorType.Path:
                self._emit_path_quantity(quantity, constraints, world_qtys)
                continue
            if quantity.type == QuantityType.Direction:
                self._emit_direction_quantity(node, quantity)
                continue
            if quantity.type == ReferenceGeneratorType.VelocityProfile:
                self._emit_velocity_profile_quantity(node, quantity)
                continue
            if quantity.type == ReferenceGeneratorType.Admittance:
                self._emit_admittance_quantity(node, quantity)
                continue
            if quantity.type == QuantityType.Duration and isinstance(quantity.value, Measure):
                self._emit_duration_measure(node, quantity.value)
                continue
            if isinstance(quantity.value, PoseCoordinate):
                self._emit_pose_value_quantity(node, quantity, constraints, world_qtys)
                continue
            qkind = QUDT_KIND_BY_QUANTITY_TYPE.get(quantity.type)
            if qkind is None:
                qkind = QUDT_QKIND[quantity.type]
            self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
            if quantity.type != QuantityType.Pose:
                self._emit_quantity_kind(node, qkind)
            if quantity.type == QuantityType.Orientation:
                self.graph.add((node, RDF.type, GEOM_REL.Orientation))
                self.graph.add((node, RDF.type, GEOM_COORD.OrientationCoordinate))
                self._emit_orientation_type(node, None)
            self._emit_context_composite_metadata(node, quantity, constraints, world_qtys)
            if quantity.value is None:
                if quantity.type == QuantityType.PathParameter:
                    self.graph.add(
                        (
                            node,
                            QUDT_SCHEMA.value,
                            Literal(0.0, datatype=XSD.double),
                        )
                    )
                continue
            if isinstance(quantity.value, ReferenceValue):
                if quantity.type in {
                    QuantityType.Pose,
                    QuantityType.Position,
                    QuantityType.Orientation,
                    QuantityType.VelocityTwist,
                    QuantityType.AccelerationTwist,
                    QuantityType.Wrench,
                    QuantityType.Direction,
                }:
                    raise ConstraintViolation(
                        "geometry",
                        f"Direct geometry alias '{quantity.name}' ({quantity.type}) is unsupported; "
                        "reference pose components through their map views instead",
                    )
                source_node = self._emit_context_ref_node(quantity.value.source, quantity, "source")
                self.graph.add(
                    (node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(quantity.type, QUDT_UNIT.UNITLESS))
                )
                if quantity.value.offset is not None:
                    offset_ref_node = self._emit_context_ref_node(
                        quantity.value.offset, quantity, "add-offset"
                    )
                    add_node = URIRef(f"{node}-add")
                    self.graph.add((add_node, RDF.type, ALGO_EXT.Addition))
                    self.graph.add((add_node, _ns_term(ALGO_EXT, "in"), source_node))
                    self.graph.add((add_node, _ns_term(ALGO_EXT, "in"), offset_ref_node))
                    self.graph.add((add_node, ALGO_EXT.out, node))
                else:
                    self.graph.add((node, CSTR["reference-value"], source_node))
                continue
            if isinstance(quantity.value, SnapshotValue):
                snapshot_node = URIRef(f"{node}-snapshot")
                self.graph.add((snapshot_node, RDF.type, ALGO_EXT.Snapshot))
                view_node = self._view_node(quantity.value.source, quantity)
                if quantity.value.offset is not None:
                    offset_ref_node = self._emit_context_ref_node(
                        quantity.value.offset, quantity, "add-offset"
                    )
                    # Own the op nodes by the quantity's motion-qualified URI (not the flat namespace) so two
                    # motions declaring a same-named quantity don't collapse into one op accumulating both inputs.
                    add_node = URIRef(f"{node}-add")
                    out_node = URIRef(f"{node}-add-out")
                    qkind = (
                        QUDT_KIND_BY_QUANTITY_TYPE.get(quantity.type) or QUDT_QKIND[quantity.type]
                    )
                    self.graph.add((add_node, RDF.type, ALGO_EXT.Addition))
                    self.graph.add((add_node, _ns_term(ALGO_EXT, "in"), view_node))
                    self.graph.add((add_node, _ns_term(ALGO_EXT, "in"), offset_ref_node))
                    self.graph.add((add_node, ALGO_EXT.out, out_node))
                    self.graph.add((out_node, RDF.type, QUDT_SCHEMA.Quantity))
                    self._emit_quantity_kind(out_node, qkind)
                    self.graph.add(
                        (
                            out_node,
                            QUDT_SCHEMA.unit,
                            SCALAR_UNIT.get(quantity.type, QUDT_UNIT.UNITLESS),
                        )
                    )
                    # A vector-valued (Position) offset result is a 3-vector: tag it with
                    # Position-coordinate metadata so the IR types it as a 3-vector, not a scalar. (Scalar offsets, e.g. LinearDistance, stay scalar.)
                    if quantity.type == QuantityType.Position:
                        self._emit_snapshot_position_metadata(out_node, quantity)
                    snap_source = out_node
                else:
                    snap_source = view_node
                self.graph.add((snapshot_node, _ns_term(ALGO_EXT, "in"), snap_source))
                self.graph.add((snapshot_node, ALGO_EXT.out, node))
                trigger = quantity.value.trigger
                self.graph.add(
                    (
                        snapshot_node,
                        _ns_term(ALGO_EXT, "sampling"),
                        _ns_term(
                            ALGO_EXT,
                            "event-triggered-sampling" if trigger else "initial-sampling",
                        ),
                    )
                )
                if trigger is not None:
                    self.graph.add(
                        (snapshot_node, _ns_term(ALGO_EXT, "trigger"), URIRef(trigger.uri))
                    )
                self.graph.add(
                    (node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(quantity.type, QUDT_UNIT.UNITLESS))
                )
                if qkind == GEOM_REL.Pose:
                    self._emit_declared_pose_frame_metadata(node, quantity)
                elif quantity.type == QuantityType.Position:
                    self._emit_snapshot_position_metadata(node, quantity)
                continue
            if isinstance(
                quantity.value,
                (VelocityTwistCoordinate, AccelerationTwistCoordinate, WrenchCoordinate),
            ):
                self._emit_two_subspace_coordinate(node, quantity)
                continue
            self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(quantity.value.unit)))
            if isinstance(quantity.value, Measure):
                self.graph.add(
                    (
                        node,
                        QUDT_SCHEMA.value,
                        Literal(
                            float(quantity.value.value),
                            datatype=XSD.double,
                        ),
                    )
                )
            elif isinstance(quantity.value, VectorXYZ):
                self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
                for label, element in zip(("x", "y", "z"), quantity.value.coords.values):
                    if element.ref is not None:
                        value_obj = self._emit_context_ref_node(element.ref, quantity, label)
                    else:
                        value_obj = Literal(
                            float(element.value), datatype=XSD.double
                        )
                    self.graph.add((node, GEOM_COORD[label], value_obj))

    def _emit_velocity_profile_quantity(self, node: URIRef, quantity: ContextQuantity) -> None:
        """Emit a well-typed placeholder for a velocity-profile reference; the per-controller
        profile op is emitted later at constraint-binding time (`_emit_velocity_profile_reference`),
        mirroring how an admittance reference's placeholder relates to its filter op.
        """
        if not isinstance(quantity.value, ProfileSpec):
            return
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.LinearVelocity))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT["M-PER-SEC"]))

    def _emit_admittance_quantity(self, node: URIRef, quantity: ContextQuantity) -> None:
        """Emit a well-typed placeholder for an admittance reference; the per-step filter op is
        emitted later at constraint-binding time (`_emit_admittance_reference`), mirroring how
        a velocity profile's op is emitted from the controller path.
        """
        if not isinstance(quantity.value, AdmittanceSpec):
            return
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.LinearVelocity))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT["M-PER-SEC"]))

    def _emit_coordinate_components(
        self,
        container_node: URIRef,
        superobject: URIRef,
        coords: Coordinates,
        labels: list[str],
        component_kind: Any,
        subspace: URIRef,
        unit: str | None,
        quantity: ContextQuantity,
        name_prefix: str,
    ) -> None:
        """Emit `coords` onto `container_node`.

        A fully literal coordinate carries its values directly as `geom-coord:<label>`
        doubles. When any axis references another quantity the coordinate carries no
        literals at all: each axis becomes a `map:View` binding that axis of
        `superobject` to the quantity it *is* -- the referenced node, or a small
        value-carrying quantity for the literal axes beside it.

        Either way the coordinate carries the unit it was written in, not a canonical
        one: the value is never rescaled, so the unit is the only thing saying what the
        number means.

        `unit=None` means dimensionless -- quaternion/direction-cosine components carry
        no unit at all.
        """

        def _literal(element):
            return Literal(float(element.value), datatype=XSD.double)

        # A wrench passes itself as its own container for both subspaces, so its unit pair is
        # stamped by the caller; anywhere else the container holds one subspace and one unit.
        if unit is not None and container_node != superobject:
            self.graph.remove((container_node, QUDT_SCHEMA.unit, None))
            self.graph.add((container_node, QUDT_SCHEMA.unit, _dsl_unit(unit)))

        elements = list(zip(labels, coords.values))
        if all(element.ref is None for _, element in elements):
            predicates = {
                "x": URI_GEOM_PRED_X,
                "y": URI_GEOM_PRED_Y,
                "z": URI_GEOM_PRED_Z,
                "w": URI_GEOM_PRED_W,
                "alpha": URI_GEOM_PRED_ALPHA,
                "beta": URI_GEOM_PRED_BETA,
                "gamma": URI_GEOM_PRED_GAMMA,
            }
            for label, element in elements:
                self.graph.add((container_node, predicates[label], _literal(element)))
            return

        for label, element in elements:
            if element.ref is not None:
                subobject = self._emit_context_ref_node(element.ref, quantity, label)
            else:
                subobject = URIRef(f"{name_prefix}.{label}")
                if unit is None:
                    self.graph.add((subobject, RDF.type, QUDT_SCHEMA.Quantity))
                    self._emit_quantity_kind(subobject, QUDT_KIND_BY_QUANTITY_TYPE[component_kind])
                else:
                    self._add_quantity(subobject, component_kind)
                    self.graph.remove((subobject, QUDT_SCHEMA.unit, None))
                    self.graph.add((subobject, QUDT_SCHEMA.unit, _dsl_unit(unit)))
                self.graph.add((subobject, QUDT_SCHEMA.value, _literal(element)))
            view_node = URIRef(f"{name_prefix}.{label}-view")
            self._emit_view(view_node)
            self.graph.add((view_node, MAP.superobject, superobject))
            self.graph.add((view_node, MAP.subobject, subobject))
            self.graph.add((view_node, MAP.subspace, subspace))
            self.graph.add((view_node, MAP.axis, _ns_term(MAP, label)))

    def _emit_pose_value_quantity(
        self,
        node: URIRef,
        quantity: ContextQuantity,
        constraints: list[ConstraintSpecification] | None = None,
        world_qtys: dict[str, WorldQuantity] | None = None,
    ) -> None:
        """Emit a literal pose value: its pose relation and coordinate, the position and
        orientation coordinates it binds by `map:View`, and each subspace's values.
        """
        assert isinstance(quantity.value, PoseCoordinate)
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((node, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.UNITLESS))
        self.graph.add(
            (node, QUDT_SCHEMA.unit, _dsl_unit(quantity.value.position.unit or "m"))
        )
        pose_relation = self._emit_declared_pose_frame_metadata(node, quantity)
        if pose_relation is None:
            path_quantity = next(
                (
                    sibling
                    for sibling in getattr(quantity.parent, "declaration", [])
                    if isinstance(getattr(sibling, "value", None), PathValue)
                    and quantity in self._path_pose_endpoints(sibling)
                ),
                None,
            )
            frame_nodes = (
                self._path_frame_nodes(path_quantity) if path_quantity is not None else None
            )
            relative = quantity.value.orientation.relative
            relative_base = _context_quantity(relative.base) if relative is not None else None
            if frame_nodes is None and relative_base is not None:
                frames = _pose_frame_names(relative_base)
                if frames is not None:
                    frame_nodes = tuple(self._owned_uri(name, relative_base) for name in frames)
            component_sources = {
                source
                for source in (
                    _context_quantity(quantity.value.position.ref),
                    _context_quantity(quantity.value.orientation.ref),
                )
                if source is not None
            }
            component_frames = {
                tuple(self._owned_uri(name, source) for name in frames)
                for source in component_sources
                if (frames := _pose_frame_names(source)) is not None
            }
            if len(component_frames) > 1:
                raise ConstraintViolation(
                    "geometry", f"Pose '{node}' component references disagree on frame endpoints"
                )
            if frame_nodes is None and component_frames:
                frame_nodes = next(iter(component_frames))
            source_pose = next(
                (
                    self._resolve_constraint_quantity(spec, world_qtys or {})
                    for spec in constraints or []
                    if getattr(
                        _context_quantity(getattr(spec.expr, "reference", None)), "name", None
                    )
                    == quantity.name
                    and self._resolve_constraint_quantity(spec, world_qtys or {}) is not None
                ),
                None,
            )
            if frame_nodes is None and source_pose is not None:
                frames = _pose_frame_names(source_pose)
                if frames is not None:
                    frame_nodes = tuple(self._owned_uri(name, source_pose) for name in frames)
            if frame_nodes is None:
                raise ConstraintViolation("geometry", f"Pose '{node}' has no frame endpoints")
            of_node, wrt_node, seen_by_node = frame_nodes
            self._frame_coords_index[node] = (of_node, wrt_node, seen_by_node)
            pose_relation = self._emit_geom_relation(
                node,
                "pose",
                of_node,
                wrt_node,
                seen_by_node,
                (QUDT_QKIND.PlaneAngle, URI_QUDT_QK_LENGTH),
            )

        position = quantity.value.position
        orientation = quantity.value.orientation
        position_node = URIRef(f"{quantity.uri}.position")
        orientation_node = URIRef(f"{quantity.uri}.orientation")
        self.graph.add((position_node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((position_node, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
        self.graph.add((position_node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        self.graph.add((orientation_node, RDF.type, QUDT_SCHEMA.Quantity))
        if orientation.relative is None:
            self._emit_orientation_type(orientation_node, orientation)

        authored_orientation_coords = (
            orientation.quat.xyzw
            if orientation.quat is not None
            else orientation.euler.angles
            if orientation.euler is not None
            else None
        )
        has_symbolic_orientation = authored_orientation_coords is not None and any(
            element.ref is not None for element in authored_orientation_coords.values
        )
        if has_symbolic_orientation and (
            orientation.quat is not None
            or orientation.euler is not None
            and len(set(orientation.euler.axes)) != 3
        ):
            raise ConstraintViolation(
                "geometry",
                f"Orientation coordinate '{orientation_node}' cannot use symbolic components",
            )
        if orientation.direction_cosine is not None and any(
            element.ref is not None
            for axis in (
                orientation.direction_cosine.x_axis,
                orientation.direction_cosine.y_axis,
                orientation.direction_cosine.z_axis,
            )
            for element in axis.values
        ):
            raise ConstraintViolation(
                "geometry",
                f"Orientation coordinate '{orientation_node}' cannot use symbolic components",
            )

        coords = self._frame_coords(node)
        pose_of, pose_wrt, pose_asb = coords if coords is not None else (None, None, None)
        position_relation = self._emit_geom_relation(
            position_node,
            "position",
            self._frame_origin(pose_of) if pose_of is not None else None,
            self._frame_origin(pose_wrt) if pose_wrt is not None else None,
            pose_asb or pose_wrt,
            (URI_QUDT_QK_LENGTH,),
        )
        orientation_relation = self._emit_geom_relation(
            orientation_node, "orientation", pose_of, pose_wrt, pose_asb or pose_wrt
        )
        if pose_relation is not None:
            self.graph.add((pose_relation, RDF.type, URI_GEOM_TYPE_POSITION_REF))
            self.graph.add((pose_relation, RDF.type, URI_GEOM_TYPE_ORIENT_REF))
            self.graph.add((pose_relation, URI_GEOM_PRED_OF_POSITION, position_relation))
            self.graph.add((pose_relation, URI_GEOM_PRED_OF_ORIENT, orientation_relation))
        for subobject, subspace in (
            (position_relation, MAP_EXT.position),
            (orientation_relation, MAP_EXT.orientation),
        ):
            view_node = URIRef(f"{subobject}-view")
            self._emit_view(view_node)
            self.graph.add((view_node, RDF.type, MAP_EXT.PoseCoordinateView))
            self.graph.add((view_node, MAP.superobject, node))
            self.graph.add((view_node, MAP.subobject, subobject))
            self.graph.add((view_node, MAP.subspace, subspace))

        if position.ref is not None:
            ref_node = self._emit_context_ref_node(position.ref, quantity, "position")
            view_node = URIRef(f"{position_node}-ref-view")
            self._emit_view(view_node)
            self.graph.add((view_node, RDF.type, MAP_EXT.PoseCoordinateView))
            self.graph.add((view_node, MAP.superobject, node))
            self.graph.add((view_node, MAP.subobject, ref_node))
            self.graph.add((view_node, MAP.subspace, MAP_EXT.position))
        else:
            self._emit_coordinate_components(
                position_node,
                node,
                position.coords,
                ["x", "y", "z"],
                QuantityType.Distance,
                MAP_EXT.position,
                position.unit,
                quantity,
                f"{quantity.uri}.position",
            )

        if orientation.relative is not None:
            self._emit_relative_orientation(orientation_node, orientation.relative, quantity)
        elif orientation.ref is not None:
            ref_node = self._emit_context_ref_node(orientation.ref, quantity, "orientation")
            view_node = URIRef(f"{orientation_node}-ref-view")
            self._emit_view(view_node)
            self.graph.add((view_node, RDF.type, MAP_EXT.PoseCoordinateView))
            self.graph.add((view_node, MAP.superobject, node))
            self.graph.add((view_node, MAP.subobject, ref_node))
            self.graph.add((view_node, MAP.subspace, MAP_EXT.orientation))
        elif orientation.quat is not None:
            self._emit_coordinate_components(
                orientation_node,
                node,
                orientation.quat.xyzw,
                ["x", "y", "z", "w"],
                QuantityType.Dimensionless,
                MAP_EXT.orientation,
                None,
                quantity,
                f"{quantity.uri}.orientation",
            )
        elif orientation.direction_cosine is not None:
            dc = orientation.direction_cosine
            for axis_coords, pred in (
                (dc.x_axis, URI_GEOM_PRED_DIRECTION_COSINE_X),
                (dc.y_axis, URI_GEOM_PRED_DIRECTION_COSINE_Y),
                (dc.z_axis, URI_GEOM_PRED_DIRECTION_COSINE_Z),
            ):
                add_literal_list_pred(
                    self.graph,
                    orientation_node,
                    pred,
                    tuple(float(element.value) for element in axis_coords.values),
                )
        else:
            euler = orientation.euler
            self._emit_coordinate_components(
                orientation_node,
                node,
                euler.angles,
                list(euler.axes) if has_symbolic_orientation else ["alpha", "beta", "gamma"],
                QuantityType.Angle,
                MAP_EXT.orientation,
                _angle_unit(euler),
                quantity,
                f"{quantity.uri}.orientation",
            )

    def _emit_relative_orientation(self, orientation_node, relative, quantity) -> None:
        """Emit an orientation composed from a base orientation and a delta rotation as a
        `geom-op-ext:ComposeOrientation` operator writing into `orientation_node`.

        The base is never decomposed, so no rotation parameterisation is round-tripped. The
        basis the delta turns in decides slot order: intrinsic Euler angles use the base's own
        body frame and compose post (base * delta); extrinsic Euler angles use the basis the
        base is expressed in and compose pre (delta * base). Quaternion and direction-cosine
        deltas must name their basis explicitly.
        """
        # Bind to the pose itself, not its orientation subobject: the backend reads the
        # composed rotation off the materialised frame.
        base_quantity = _context_quantity(relative.base)
        base_node = (
            URIRef(_resolved_context_quantity(base_quantity).uri)
            if isinstance(base_quantity, ContextQuantity)
            else self._emit_context_ref_node(relative.base, quantity, "orientation-base")
        )

        if not isinstance(base_quantity, ContextQuantity):
            raise ValueError(
                f"Relative orientation on '{quantity.uri}' needs a resolvable base pose to "
                "derive its composition frames."
            )
        frames = _pose_frame_names(base_quantity)
        if frames is None:
            raise ValueError(
                f"Relative orientation on '{quantity.uri}' cannot resolve its base pose's "
                "of/with-respect-to/as-seen-by frames."
            )
        of_frame, _wrt_frame, base_as_seen_by = frames
        of_frame_node = self._owned_uri(of_frame, base_quantity)
        base_as_seen_by_node = self._owned_uri(base_as_seen_by, base_quantity)
        self.graph.add((orientation_node, GEOM_REL.of, of_frame_node))
        self.graph.add((orientation_node, GEOM_COORD["as-seen-by"], base_as_seen_by_node))
        if relative.frame is not None:
            delta_basis = self._owned_uri(str(getattr(relative.frame, "uri", relative.frame)), quantity)
        elif relative.euler is not None and relative.euler.extrinsic:
            # Extrinsic Euler angles turn about the pose's fixed reference-frame axes.
            delta_basis = base_as_seen_by_node
        elif relative.euler is not None:
            # Intrinsic Euler angles turn about the moving body-frame axes.
            delta_basis = of_frame_node
        else:
            raise ValueError(
                f"Relative orientation on '{quantity.uri}' needs an explicit basis frame for "
                "a quaternion or direction-cosine delta."
            )

        delta_node = URIRef(f"{quantity.uri}.orientation-delta")
        self.graph.add((delta_node, RDF.type, QUDT_SCHEMA.Quantity))
        self._emit_orientation_type(delta_node, relative)

        if delta_basis == of_frame_node:
            in1, in2 = base_node, delta_node
        elif delta_basis == base_as_seen_by_node:
            in1, in2 = delta_node, base_node
        else:
            raise ValueError(
                f"Relative orientation on '{quantity.uri}' turns its delta in '{delta_basis}', "
                f"which is neither the base's body frame '{of_frame_node}' nor its coordinate "
                f"basis '{base_as_seen_by_node}'. Composing it needs a change of basis, which is "
                "not supported."
            )
        composition_node = URIRef(f"{quantity.uri}.orientation-composition")
        self.graph.add((composition_node, RDF.type, GEOM_OP_EXT.ComposeOrientation))
        self.graph.add((composition_node, GEOM_OP.in1, in1))
        self.graph.add((composition_node, GEOM_OP.in2, in2))
        self.graph.add((composition_node, GEOM_OP.composite, orientation_node))

        if relative.quat is not None:
            coords = relative.quat.xyzw
            predicates = (
                URI_GEOM_PRED_X,
                URI_GEOM_PRED_Y,
                URI_GEOM_PRED_Z,
                URI_GEOM_PRED_W,
            )
            unit = None
        elif relative.direction_cosine is not None:
            dc = relative.direction_cosine
            for axis_coords, predicate in (
                (dc.x_axis, URI_GEOM_PRED_DIRECTION_COSINE_X),
                (dc.y_axis, URI_GEOM_PRED_DIRECTION_COSINE_Y),
                (dc.z_axis, URI_GEOM_PRED_DIRECTION_COSINE_Z),
            ):
                if any(element.ref is not None for element in axis_coords.values):
                    raise ConstraintViolation(
                        "geometry",
                        f"Relative orientation '{quantity.uri}' has a symbolic direction cosine",
                    )
                add_literal_list_pred(
                    self.graph,
                    delta_node,
                    predicate,
                    tuple(float(element.value) for element in axis_coords.values),
                )
            return
        else:
            euler = relative.euler
            coords = euler.angles
            predicates = (URI_GEOM_PRED_ALPHA, URI_GEOM_PRED_BETA, URI_GEOM_PRED_GAMMA)
            unit = _angle_unit(euler)
        self._emit_delta_components(delta_node, coords, predicates, unit, quantity)

    def _emit_delta_components(self, node, coords, predicates, unit, quantity) -> None:
        """Emit one standalone literal rotation parameter consumed by RelativeOrientation."""
        if any(element.ref is not None for element in coords.values):
            raise ConstraintViolation(
                "geometry", f"Relative orientation '{quantity.uri}' has symbolic components"
            )
        if unit is not None:
            self.graph.remove((node, QUDT_SCHEMA.unit, None))
            self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(unit)))
        for predicate, element in zip(predicates, coords.values):
            self.graph.add((node, predicate, Literal(float(element.value), datatype=XSD.double)))

    def _emit_two_subspace_coordinate(self, node: URIRef, quantity: ContextQuantity) -> None:
        """Emit a literal velocity-twist/acceleration-twist/wrench value: its two named
        subspace vectors, each an authored Coordinates tuple in its own unit.
        """
        value = quantity.value
        if isinstance(value, VelocityTwistCoordinate):
            subspaces = (
                (
                    "angular-velocity",
                    value.angular,
                    value.angular_unit,
                    GEOM_COORD["angular-velocity"],
                    QuantityType.AngularVelocity,
                ),
                (
                    "linear-velocity",
                    value.linear,
                    value.linear_unit,
                    GEOM_COORD["linear-velocity"],
                    QuantityType.LinearVelocity,
                ),
            )
        elif isinstance(value, AccelerationTwistCoordinate):
            subspaces = (
                (
                    "angular-acceleration",
                    value.angular,
                    value.angular_unit,
                    GEOM_COORD["angular-acceleration"],
                    QuantityType.AngularAcceleration,
                ),
                (
                    "linear-acceleration",
                    value.linear,
                    value.linear_unit,
                    GEOM_COORD["linear-acceleration"],
                    QuantityType.LinearAcceleration,
                ),
            )
        else:
            assert isinstance(value, WrenchCoordinate)
            subspaces = (
                (
                    "torque",
                    value.torque,
                    value.torque_unit,
                    RBDYN_COORD.torque,
                    QuantityType.Torque,
                ),
                ("force", value.force, value.force_unit, RBDYN_COORD.force, QuantityType.Force),
            )

        # The container's unit pair is what says which scale its subspace numbers are on.
        self.graph.remove((node, QUDT_SCHEMA.unit, None))
        for _, _, subspace_unit, _, _ in subspaces:
            self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(subspace_unit)))

        if isinstance(value, WrenchCoordinate):
            for label, coords, unit, predicate, kind in subspaces:
                if all(element.ref is None for element in coords.values):
                    add_literal_list_pred(
                        self.graph,
                        node,
                        predicate,
                        tuple(float(element.value) for element in coords.values),
                    )
                else:
                    self._emit_coordinate_components(
                        node,
                        node,
                        coords,
                        ["x", "y", "z"],
                        kind,
                        MAP[label],
                        unit,
                        quantity,
                        f"{quantity.uri}.{label}",
                    )
            return
        for label, coords, unit, pred, kind in subspaces:
            subspace_node = URIRef(f"{quantity.uri}.{label}")
            self.graph.add((subspace_node, RDF.type, QUDT_SCHEMA.Quantity))
            self.graph.add((subspace_node, RDF.type, GEOM_COORD.VectorXYZ))
            self._emit_quantity_kind(
                subspace_node, QUDT_KIND_BY_QUANTITY_TYPE.get(kind) or QUDT_QKIND[kind]
            )
            self.graph.add(
                (subspace_node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(kind, QUDT_UNIT.UNITLESS))
            )
            self.graph.add((node, pred, subspace_node))
            self._emit_coordinate_components(
                subspace_node,
                node,
                coords,
                ["x", "y", "z"],
                kind,
                MAP[label],
                unit,
                quantity,
                f"{quantity.uri}.{label}",
            )

    @staticmethod
    def _lerp_value_kind(lerp) -> Any | None:
        """The shared QUDT kind of a lerp's start and goal quantities; raises if they differ,
        None if neither is typed.
        """
        start_qty = _context_quantity(lerp.start)
        goal_qty = _context_quantity(lerp.goal)
        start_kind = QUDT_KIND_BY_QUANTITY_TYPE.get(getattr(start_qty, "type", None))
        goal_kind = QUDT_KIND_BY_QUANTITY_TYPE.get(getattr(goal_qty, "type", None))
        if start_kind is None and goal_kind is None:
            return None
        if start_kind != goal_kind:
            raise ValueError(
                f"Lerp path start type '{getattr(start_qty, 'type', None)}' "
                f"does not match goal type '{getattr(goal_qty, 'type', None)}'"
            )
        return start_kind

    def _emit_direction_quantity(
        self,
        node: URIRef,
        quantity: ContextQuantity,
    ) -> None:
        """Emit a Direction quantity as a unit direction coordinate in its as-seen-by frame,
        optionally initialized from a literal vector value.
        """
        as_seen_by_name = _geo_prop(quantity.props, "as-seen-by") or _geo_prop(
            quantity.props, "wrt"
        )
        if as_seen_by_name is None:
            raise ValueError(
                f"Direction quantity '{quantity.name}' needs an 'as-seen-by: <frame>' prop."
            )
        as_seen_by_node = self._owned_uri(as_seen_by_name, quantity)
        vector: tuple[float, float, float] | None = None
        if isinstance(quantity.value, VectorXYZ):
            elements = quantity.value.coords.values
            if len(elements) != 3 or any(e.ref is not None for e in elements):
                raise ValueError(
                    f"Direction quantity '{quantity.name}' value must be 3 literal components."
                )
            vector = tuple(float(e.value) for e in elements)
        elif quantity.value is not None:
            raise ValueError(
                f"Direction quantity '{quantity.name}' value must be a Vector literal."
            )
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        self._emit_direction_coordinate(node, as_seen_by_node, vector)

    def _emit_path_quantity(
        self,
        quantity: ContextQuantity,
        constraints: list[ConstraintSpecification] | None = None,
        world_qtys: dict[str, WorldQuantity] | None = None,
    ) -> None:
        """Emit geometric path data; traversal is emitted by the handler's progress objective."""
        assert isinstance(quantity.value, PathValue)
        value = quantity.value
        if value.lerp is not None:
            self._emit_lerp_path(quantity, value.lerp, constraints, world_qtys)
        elif value.circle is not None:
            self._emit_geometric_path(
                quantity,
                GEOM_PATH.Circle,
                "circle",
                [
                    ("start", GEOM_PATH.start, value.circle.start),
                    ("center", GEOM_PATH.center, value.circle.center),
                    ("plane-normal", GEOM_PATH["plane-normal"], value.circle.plane_normal),
                ],
                constraints,
                world_qtys,
            )
        elif value.arc is not None:
            self._emit_geometric_path(
                quantity,
                GEOM_PATH.Arc,
                "arc",
                [
                    ("start", GEOM_PATH.start, value.arc.start),
                    ("end", GEOM_PATH.end, value.arc.end),
                    ("amplitude", GEOM_PATH.amplitude, value.arc.amplitude),
                    ("plane-normal", GEOM_PATH["plane-normal"], value.arc.plane_normal),
                ],
                constraints,
                world_qtys,
            )
        elif value.helix is not None:
            self._emit_geometric_path(
                quantity,
                GEOM_PATH.Helix,
                "helix",
                [
                    ("start", GEOM_PATH.start, value.helix.start),
                    ("center", GEOM_PATH.center, value.helix.center),
                    ("axis", GEOM_PATH.axis, value.helix.axis),
                    ("pitch", GEOM_PATH.pitch, value.helix.pitch),
                    ("revolutions", GEOM_PATH.revolutions, value.helix.revolutions),
                ],
                constraints,
                world_qtys,
            )
        elif value.figure8 is not None:
            self._emit_geometric_path(
                quantity,
                GEOM_PATH.Figure8,
                "figure8",
                [
                    ("anchor", GEOM_PATH.anchor, value.figure8.anchor),
                    ("radius", GEOM_PATH.radius, value.figure8.radius),
                    ("plane-normal", GEOM_PATH["plane-normal"], value.figure8.plane_normal),
                    ("direction", GEOM_PATH.direction, value.figure8.direction),
                ],
                constraints,
                world_qtys,
                path_terms=[(GEOM_PATH.form, _ns_term(GEOM_PATH, value.figure8.form or "gerono"))],
            )
        else:
            raise ValueError(f"PathValue on '{quantity.name}' has no populated spec")

    def _emit_path_pose_metadata(
        self,
        quantity: ContextQuantity,
        value_kind: Any | None,
        constraints: list[ConstraintSpecification] | None,
        world_qtys: dict[str, WorldQuantity] | None,
    ) -> None:
        """Emit the setpoint pose the path evaluator produces, with its frame metadata."""
        node = self._reference_output_node(quantity)
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        if value_kind is not None and value_kind != URI_GEOM_TYPE_POSE:
            self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], value_kind))
        if value_kind == URI_GEOM_TYPE_POSE:
            self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
            self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
            pose_relation = self._emit_declared_pose_frame_metadata(node, quantity)
            if pose_relation is None:
                of_node, wrt_node, seen_by_node = self._path_frame_nodes(quantity)
                self._frame_coords_index[node] = (of_node, wrt_node, seen_by_node)
                pose_relation = self._emit_geom_relation(
                    node,
                    "pose",
                    of_node,
                    wrt_node,
                    seen_by_node,
                    (QUDT_QKIND.PlaneAngle, URI_QUDT_QK_LENGTH),
                )
            self._emit_combined_pose_coordinate(node, pose_relation)

    def _emit_lerp_path(
        self,
        quantity: ContextQuantity,
        lerp: Any,
        constraints: list[ConstraintSpecification] | None,
        world_qtys: dict[str, WorldQuantity] | None,
    ) -> None:
        """Emit a geometric linear path and its eventual setpoint metadata."""
        lerp_node = self._owned_uri(f"lerp-{quantity.name}", quantity)
        value_kind = self._lerp_value_kind(lerp)
        self._emit_path_pose_metadata(quantity, value_kind, constraints, world_qtys)
        self.graph.add((lerp_node, RDF.type, GEOM_PATH.Path))
        self.graph.add((lerp_node, RDF.type, GEOM_PATH.LinearPath))
        self.graph.add(
            (lerp_node, GEOM_PATH.start, self._emit_context_ref_node(lerp.start, quantity, "start"))
        )
        self.graph.add(
            (lerp_node, GEOM_PATH.goal, self._emit_context_ref_node(lerp.goal, quantity, "goal"))
        )

    def _emit_geometric_path(
        self,
        quantity: ContextQuantity,
        path_type: URIRef,
        spec_prefix: str,
        inputs: list[tuple[str, URIRef, Any]],
        constraints: list[ConstraintSpecification] | None,
        world_qtys: dict[str, WorldQuantity] | None,
        path_terms: list[tuple[URIRef, Any]] = (),
    ) -> None:
        """Emit path geometry and its eventual setpoint metadata."""
        self._emit_path_pose_metadata(quantity, URI_GEOM_TYPE_POSE, constraints, world_qtys)
        path_node = self._owned_uri(f"{spec_prefix}-{quantity.name}", quantity)
        self.graph.add((path_node, RDF.type, GEOM_PATH.Path))
        self.graph.add((path_node, RDF.type, path_type))
        for suffix, predicate, ref in inputs:
            self.graph.add(
                (path_node, predicate, self._emit_context_ref_node(ref, quantity, suffix))
            )
        for predicate, term in path_terms:
            self.graph.add((path_node, predicate, term))

    @staticmethod
    def _path_shape(quantity: ContextQuantity) -> str:
        """Return the populated geometric shape name of a Path quantity."""
        value = quantity.value
        for name in ("lerp", "circle", "arc", "helix", "figure8"):
            if getattr(value, name, None) is not None:
                return name
        raise ValueError(f"PathValue on '{quantity.name}' has no populated spec")

    def _along_path_scalar(self, spec: ConstraintSpecification) -> tuple[str, Any] | None:
        """The scalar a driver or progress guard measures: the speed along its path."""
        operand = spec.view.moving or spec.view.progress
        if operand is None:
            return None
        path = _resolved_context_quantity(_context_quantity(operand.path))
        return f"{path.name}-along-speed", QuantityType.LinearVelocity

    def _path_geometry_node(self, path: ContextQuantity) -> URIRef:
        """The node carrying `path`'s geometry (its shape and that shape's inputs)."""
        return self._owned_uri(f"{self._path_shape(path)}-{path.name}", path)

    def _emit_along_path_constraint(
        self, node: URIRef, spec: ConstraintSpecification, motion: GuardedMotion
    ) -> None:
        """Emit a driver or a progress guard on the speed measured along a path.

        Both read the same measured quantity. The driver commands it, which is what moves the
        robot along the path; the guard only bounds it from below, so it decides whether the
        current control action may go on and never contributes a solver row.
        """
        operand = spec.view.moving or spec.view.progress
        path = _resolved_context_quantity(_context_quantity(operand.path))
        self.graph.add((node, RDF.type, CSTR.Constraint))
        self.graph.add((node, RDF.type, _constraint_type_iri(QuantityType.LinearVelocity)))
        self.graph.add((node, CSTR.quantity, self._path_along_speed_node(path)))
        self.graph.add((node, GEOM_OP_EXT.path, self._path_geometry_node(path)))
        if spec.view.moving is not None:
            self.graph.add((node, RDF.type, CSTR.EqualityConstraint))
            ref_node = self._emit_context_ref_node(operand.speed, motion, f"{spec.name}-ref")
            self.graph.add((node, CSTR["reference-value"], ref_node))
            self._reference_value_index[node] = ref_node
            return
        self.graph.add((node, RDF.type, CSTR.UnilateralConstraint))
        self.graph.add((node, RDF.type, CSTR.GreaterThanConstraint))
        self.graph.add(
            (
                node,
                CSTR.threshold,
                self._emit_context_ref_node(spec.expr.threshold, motion, f"{spec.name}-threshold"),
            )
        )

    def _path_along_speed_node(self, path: ContextQuantity) -> URIRef:
        """The measured speed of the followed frame along `path`, shared by driver and guard."""
        return self._owned_uri(f"{path.name}-along-speed", path)

    def _measured_twist_of(
        self, moved: WorldQuantity, world_qtys: dict[str, WorldQuantity]
    ) -> WorldQuantity:
        """The velocity twist of the same frame pair as `moved`.

        Speed along a path is a measured speed at the attachment point, so it reads the twist
        of exactly the frame being followed rather than differentiating its pose.
        """
        context = f"Path following of '{moved.name}'"
        of_frame, wrt_frame = self._pose_frames(moved, context)
        twist = next(
            (
                quantity
                for quantity in world_qtys.values()
                if quantity.type == WorldQuantityType.VelocityTwist
                and isinstance(quantity.props, GeometricProps)
                and _geo_prop(quantity.props, "of") == of_frame
                and _geo_prop(quantity.props, "wrt") == wrt_frame
            ),
            None,
        )
        if twist is None:
            raise ValueError(
                f"{context} needs a declared 'velocity-twist' of <{of_frame}> "
                f"wrt <{wrt_frame}> to measure the speed along the path."
            )
        return twist

    def _emit_path_projection(
        self,
        path: ContextQuantity,
        moved: WorldQuantity,
        world_qtys: dict[str, WorldQuantity],
    ) -> None:
        """Emit the operator that projects a measured pose onto a path.

        The projection replaces a commanded path parameter: it measures where on the path the
        frame already is, and reports the pose there together with the local frame that
        separates travelling along the path from leaving it. Nothing upstream can outrun the
        robot because nothing upstream writes the parameter.
        """
        projection_node = self._owned_uri(f"projection-{path.name}", path)
        if projection_node in self._path_projections:
            return
        self._path_projections.add(projection_node)

        path_node = self._path_geometry_node(path)
        as_seen_by_name = _geo_prop(moved.props, "as-seen-by") or _geo_prop(moved.props, "wrt")
        if as_seen_by_name is None:
            raise ValueError(
                f"Path following of '{moved.name}' needs an 'as-seen-by' or 'wrt' frame."
            )
        as_seen_by = self._owned_uri(as_seen_by_name, moved)

        parameter_node = self._owned_uri(f"{path.name}-s", path)
        self.graph.add((parameter_node, RDF.type, QUDT_SCHEMA.Quantity))
        self._emit_quantity_kind(parameter_node, NS_MM_QUDT_QTY["Dimensionless"])
        self.graph.add((parameter_node, QUDT_SCHEMA.unit, QUDT_UNIT.UNITLESS))

        speed_node = self._path_along_speed_node(path)
        self._add_quantity(speed_node, QuantityType.LinearVelocity)

        # Where the frame is on the path. Only this reads the robot; the three below are
        # functions of the parameter it produces.
        self.graph.add((projection_node, RDF.type, GEOM_OP_EXT.PathProjection))
        self.graph.add((projection_node, GEOM_OP_EXT.path, path_node))
        self.graph.add((projection_node, GEOM_OP.pose, URIRef(moved.uri)))
        self.graph.add((projection_node, _ns_term(GEOM_OP_EXT, "path-parameter"), parameter_node))

        # The local frame there: one axis along the path, two across it.
        directions = {}
        for term in ("tangent", "normal-a", "normal-b"):
            direction_node = self._owned_uri(f"{path.name}-{term}", path)
            self.graph.add((direction_node, RDF.type, QUDT_SCHEMA.Quantity))
            self._emit_direction_coordinate(direction_node, as_seen_by)
            directions[term] = direction_node
        frame_node = self._owned_uri(f"frame-{path.name}", path)
        self.graph.add((frame_node, RDF.type, GEOM_OP_EXT.PathTangentFrame))
        self.graph.add((frame_node, GEOM_OP_EXT.path, path_node))
        self.graph.add((frame_node, _ns_term(GEOM_OP_EXT, "path-parameter"), parameter_node))
        for term, direction_node in directions.items():
            self.graph.add((frame_node, _ns_term(GEOM_OP_EXT, term), direction_node))

        # How fast the frame travels along the path: the measured twist onto that tangent.
        along_node = self._owned_uri(f"along-{path.name}", path)
        self.graph.add((along_node, RDF.type, GEOM_OP_EXT.TwistToLinearVelocityAlong))
        self.graph.add(
            (along_node, GEOM_OP["in"], URIRef(self._measured_twist_of(moved, world_qtys).uri))
        )
        self.graph.add((along_node, GEOM_OP.direction, directions["tangent"]))
        self.graph.add((along_node, _ns_term(GEOM_OP_EXT, "along-speed"), speed_node))

        # The pose the path carries there, which is what the setpoint follows.
        evaluator_node = self._owned_uri(f"evaluator-{path.name}", path)
        self.graph.add((evaluator_node, RDF.type, GEOM_OP_EXT.PathEvaluator))
        self.graph.add((evaluator_node, RDF.type, CSTR_HDL_EXT.SetpointGenerator))
        self.graph.add((evaluator_node, GEOM_OP_EXT.path, path_node))
        self.graph.add((evaluator_node, _ns_term(GEOM_OP_EXT, "path-parameter"), parameter_node))
        self.graph.add((evaluator_node, GEOM_OP.out, self._reference_output_node(path)))

    def _emit_path_following(
        self,
        constraints: list[ConstraintSpecification],
        world_qtys: dict[str, WorldQuantity],
    ) -> None:
        """Emit one projection per path a motion's constraints follow."""
        for spec in constraints:
            operand = _path_operand(spec.view)
            if operand is None:
                continue
            path = _resolved_context_quantity(_context_quantity(operand.path))
            moved = self._resolve_qty(operand.moved, world_qtys)
            if moved is None:
                raise ValueError(f"Constraint '{spec.name}' follows a path with an unknown frame.")
            self._emit_path_projection(path, moved, world_qtys)

    def _emit_context_ref_node(self, ref: ContextRef, owner: Any, suffix: str) -> URIRef:
        """Resolve a context reference to its value node: a subspace view, a passthrough source,
        the referenced quantity, or a freshly emitted literal quantity.
        """
        quantity = _context_quantity(ref)
        if not isinstance(quantity, ContextQuantity):
            return self._owned_uri(_node_name(quantity), owner)

        quantity = _resolved_context_quantity(quantity)
        if ref.literal_value is None:
            subspace_raw = getattr(ref, "subspace", None)
            if subspace_raw is not None:
                subspace = str(getattr(subspace_raw, "value", subspace_raw))
                axis = semantic_axis_label(getattr(ref, "axis", None))
                return self._emit_context_ref_view_node(quantity, subspace, axis)
            if isinstance(quantity.value, ReferenceValue) and quantity.value.offset is None:
                return self._emit_context_ref_node(quantity.value.source, owner, suffix)
            if quantity.type == ReferenceGeneratorType.Path:
                return self._reference_output_node(quantity)
            return URIRef(quantity.uri)

        node = URIRef(f"{quantity.uri}-{suffix}")
        qkind = QUDT_KIND_BY_QUANTITY_TYPE.get(quantity.type)
        if qkind is None:
            qkind = QUDT_QKIND[quantity.type]

        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        if quantity.type == QuantityType.Distance:
            self.graph.add((node, RDF.type, GEOM_COORD.LinearDistanceCoordinate))
        self._emit_quantity_kind(node, qkind)
        self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(ref.literal_value.unit)))
        if isinstance(ref.literal_value, Measure):
            self.graph.add(
                (
                    node,
                    QUDT_SCHEMA.value,
                    Literal(
                        float(ref.literal_value.value),
                        datatype=XSD.double,
                    ),
                )
            )
        elif isinstance(ref.literal_value, VectorXYZ):
            self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
            for label, element in zip(("x", "y", "z"), ref.literal_value.coords.values):
                if element.ref is not None:
                    value_obj = self._emit_context_ref_node(element.ref, owner, f"{suffix}-{label}")
                else:
                    value_obj = Literal(
                        float(element.value), datatype=XSD.double
                    )
                self.graph.add((node, GEOM_COORD[label], value_obj))
        return node

    def _constraint_reference_node(
        self,
        ref: ContextRef,
        owner: Any,
        suffix: str,
        subspace: str,
        axis: str | None,
    ) -> URIRef:
        """The node a constraint compares against, promoted to a position/orientation coordinate
        for a whole-subspace (axis-less) pose/path reference.
        """
        ref_node = self._emit_context_ref_node(ref, owner, suffix)
        quantity = _context_quantity(ref)
        if not isinstance(quantity, ContextQuantity) or axis is not None:
            return ref_node
        quantity = _resolved_context_quantity(quantity)
        if quantity.type == QuantityType.Pose:
            component_prefix = (
                f"{quantity.uri}.{{component}}"
                if isinstance(quantity.value, PoseCoordinate)
                else quantity.uri
            )
            if subspace == "position":
                return URIRef(f"{component_prefix.format(component='position')}-position-rel")
            if subspace == "orientation":
                return URIRef(f"{component_prefix.format(component='orientation')}-orientation-rel")
        if quantity.type == ReferenceGeneratorType.Path:
            if subspace == "position":
                return URIRef(f"{ref_node}-position-rel")
            elif subspace == "orientation":
                return URIRef(f"{ref_node}-orientation-rel")
        return ref_node

    def _emit_constraints(
        self,
        motion: GuardedMotion,
        constraints: list[ConstraintSpecification],
        world_qtys: dict[str, WorldQuantity],
    ) -> None:
        """Emit each elapsed, distance, or quantity constraint in a motion."""
        seen_uris: set[str] = set()
        for spec in constraints:
            uri_str = str(spec.uri)
            if uri_str in seen_uris:
                continue
            seen_uris.add(uri_str)

            node = URIRef(spec.uri)

            if getattr(spec.view, "is_elapsed", False):
                self._emit_elapsed_constraint(node, spec, motion)
                continue

            if spec.view.moving is not None or spec.view.progress is not None:
                self._emit_along_path_constraint(node, spec, motion)
                continue

            qty = self._resolve_constraint_quantity(spec, world_qtys)
            if qty is None:
                raise ValueError(f"Constraint '{spec.name}' does not resolve to a world quantity.")
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
                    qty.type == WorldQuantityType.Pose
                    and not _is_distance_view(spec)
                    and axis is None
                    and subspace
                    in {
                        "position",
                        "distance",
                    }
                ):
                    qty_node = URIRef(f"{qty.uri}-position-rel")
                elif (
                    qty.type == WorldQuantityType.Pose
                    and axis is None
                    and subspace
                    in {
                        "orientation",
                        "rotation",
                    }
                ):
                    qty_node = URIRef(f"{qty.uri}-orientation-rel")
                elif axis is None and (
                    (subspace == "pose" and qty.type == WorldQuantityType.Pose)
                    or qty.type == WorldQuantityType.JointPosition
                ):
                    qty_node = URIRef(qty.uri)
                else:
                    sid = _scalar_id(qty, subspace, axis)
                    qty_node = self._owned_uri(sid, motion)
            else:
                qty_node = None

            scalar_t = _scalar_type(qty, subspace, axis) if qty else subspace
            self.graph.add((node, RDF.type, CSTR.Constraint))
            self.graph.add((node, RDF.type, _constraint_type_iri(scalar_t)))
            if qty_node is not None:
                self.graph.add((node, CSTR.quantity, qty_node))

            if spec.view.on is not None:
                # The path is the reference: it is evaluated where the frame already is, so
                # this states no target of its own and needs no authored comparison.
                path = _resolved_context_quantity(_context_quantity(spec.view.on.path))
                self.graph.add((node, RDF.type, CSTR.EqualityConstraint))
                ref_node = self._emit_context_ref_view_node(path, subspace, axis)
                self.graph.add((node, CSTR["reference-value"], ref_node))
                self._reference_value_index[node] = ref_node
                self.graph.add((node, GEOM_OP_EXT.path, self._path_geometry_node(path)))
                continue

            expr = spec.expr
            if isinstance(expr, EqualityConstraint):
                self.graph.add((node, RDF.type, CSTR.EqualityConstraint))
                ref_node = self._constraint_reference_node(
                    expr.reference, motion, f"{spec.name}-ref", subspace, axis
                )
                profiled_ctrl = self._profiled_controller_for_spec(spec)
                if profiled_ctrl is not None:
                    ref_node = self._emit_velocity_profile_reference(
                        profiled_ctrl,
                        spec,
                        motion,
                        ref_node,
                        qty_node,
                        scalar_t,
                    )
                admit_qty = _context_quantity(expr.reference)
                if isinstance(admit_qty, ContextQuantity):
                    admit_qty = _resolved_context_quantity(admit_qty)
                if (
                    isinstance(admit_qty, ContextQuantity)
                    and admit_qty.type == ReferenceGeneratorType.Admittance
                ):
                    admit_ctrl = self._controller_for_spec(spec)
                    if admit_ctrl is None:
                        raise ValueError(
                            f"Admittance constraint '{spec.name}' needs a tracking PID "
                            "to host the filter's per-step integrator state."
                        )
                    ref_node = self._emit_admittance_reference(
                        admit_ctrl, spec, motion, admit_qty, scalar_t
                    )
                self.graph.add((node, CSTR["reference-value"], ref_node))
                self._reference_value_index[node] = ref_node
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
            elif isinstance(expr, OutsideConstraint):
                self.graph.add((node, RDF.type, CSTR_EXT.OutsideConstraint))
                lo_node = self._emit_context_ref_node(expr.lower, motion, f"{spec.name}-lower")
                up_node = self._emit_context_ref_node(expr.upper, motion, f"{spec.name}-upper")
                self.graph.add((node, CSTR["lower-threshold"], lo_node))
                self.graph.add((node, CSTR["upper-threshold"], up_node))

    def _profiled_controller_for_spec(
        self, spec: ConstraintSpecification
    ) -> ControllerEntry | None:
        """The profiled controller bound to `spec`, or None."""
        return self._profiled_controller_by_spec.get(spec)

    def _controller_for_spec(self, spec: ConstraintSpecification) -> ControllerEntry | None:
        """The controller bound to `spec`, or None."""
        return self._controller_by_spec.get(spec)

    def _emit_admittance_reference(
        self,
        ctrl: ControllerEntry,
        spec: ConstraintSpecification,
        motion: GuardedMotion,
        admit_qty: ContextQuantity,
        scalar_t: Any,
    ) -> URIRef:
        """Emit a stateful admittance filter op (force -> velocity via mass/damping/stiffness,
        velocity-capped) whose integrator state lives on the constraint's controller. Returns
        the reference-value node.
        """
        spec_val = admit_qty.value
        if not isinstance(spec_val, AdmittanceSpec):
            raise ValueError(f"Admittance quantity '{admit_qty.name}' has no filter spec.")

        out_node = self._owned_uri(f"{spec.name}-{ctrl.name}-admit-ref", motion)
        self._add_quantity(out_node, scalar_t)

        op_node = self._owned_uri(f"admit-{spec.name}-{ctrl.name}", motion)
        self.graph.add((op_node, RDF.type, ALGO_EXT.Admittance))
        self.graph.add((op_node, RDF.type, CSTR_HDL_EXT.SetpointGenerator))
        self.graph.add(
            (
                op_node,
                _ns_term(ALGO_EXT, "in"),
                self._emit_profile_view_node(spec_val.force, admit_qty),
            )
        )
        mass_node = URIRef(f"{op_node}-mass")
        self._emit_scalar_quantity(mass_node, spec_val.mass, URI_QUDT_QK_MASS, QUDT_UNIT["KiloGM"])
        self.graph.add((op_node, ALGO_EXT.mass, mass_node))
        damping_node = URIRef(f"{op_node}-damping")
        self._emit_scalar_quantity(damping_node, spec_val.damping, None, QUDT_UNIT["N-SEC-PER-M"])
        self.graph.add((op_node, ALGO_EXT.damping, damping_node))
        stiffness_node = URIRef(f"{op_node}-stiffness")
        self._emit_scalar_quantity(stiffness_node, spec_val.stiffness, None, QUDT_UNIT["N-PER-M"])
        self.graph.add((op_node, ALGO_EXT.stiffness, stiffness_node))
        max_velocity_node = URIRef(f"{op_node}-max-velocity")
        self._emit_scalar_quantity(
            max_velocity_node,
            float(spec_val.max_velocity),
            QUDT_QKIND.LinearVelocity,
            _dsl_unit(spec_val.max_velocity_unit or "m/s"),
        )
        self.graph.add((op_node, CSTR_HDL["maximum-velocity"], max_velocity_node))
        self.graph.add((op_node, ALGO_EXT.out, out_node))
        return out_node

    def _emit_velocity_profile_reference(
        self,
        ctrl: ControllerEntry,
        spec: ConstraintSpecification,
        motion: GuardedMotion,
        goal_node: URIRef,
        measured_node: URIRef | None,
        scalar_t: Any,
    ) -> URIRef:
        """Emit a velocity-profile reference-generating op (goal + measured -> profiled velocity)
        for a profiled controller. Returns the reference-value node.
        """
        if measured_node is None:
            raise ValueError(f"Profiled controller '{ctrl.name}' needs a measured quantity.")
        profile_qty = _context_quantity(ctrl.params.profile)
        if not isinstance(profile_qty, ContextQuantity):
            raise ValueError(f"Controller '{ctrl.name}' has an unresolved velocity profile.")
        profile_qty = _resolved_context_quantity(profile_qty)
        if not isinstance(profile_qty.value, ProfileSpec):
            raise ValueError(
                f"Controller '{ctrl.name}' profile '{profile_qty.name}' is not a Profile."
            )

        # The profile emits a setpoint for the quantity it drives, so the output carries
        # that quantity's kind, not a velocity.
        out_node = self._owned_uri(f"{spec.name}-{ctrl.name}-profile-ref", motion)
        self._add_quantity(out_node, scalar_t)

        op_node = self._owned_uri(f"profile-{spec.name}-{ctrl.name}", motion)
        self.graph.add((op_node, RDF.type, ALGO_EXT.VelocityProfile))
        self.graph.add((op_node, RDF.type, CSTR_HDL_EXT.SetpointGenerator))
        # Where it is driving to. The value it starts from is the constraint's own
        # quantity, so the profile does not restate it.
        self.graph.add((op_node, _ns_term(ALGO_EXT, "target"), goal_node))
        self.graph.add(
            (
                op_node,
                _ns_term(ALGO_EXT, "maximum-velocity"),
                self._emit_context_ref_node(
                    profile_qty.value.max_velocity, profile_qty, "max-velocity"
                ),
            )
        )
        self.graph.add(
            (
                op_node,
                _ns_term(ALGO_EXT, "maximum-acceleration"),
                self._emit_context_ref_node(
                    profile_qty.value.max_acceleration, profile_qty, "max-acceleration"
                ),
            )
        )
        if profile_qty.value.measured_velocity is not None:
            self.graph.add(
                (
                    op_node,
                    _ns_term(ALGO_EXT, "in"),
                    self._emit_profile_view_node(profile_qty.value.measured_velocity, profile_qty),
                )
            )
        if profile_qty.value.max_jerk is not None:
            self.graph.add(
                (
                    op_node,
                    _ns_term(ALGO_EXT, "maximum-jerk"),
                    self._emit_context_ref_node(
                        profile_qty.value.max_jerk, profile_qty, "max-jerk"
                    ),
                )
            )
        self.graph.add(
            (op_node, ALGO_EXT.shape, _ns_term(ALGO_EXT, profile_qty.value.shape or "trapezoidal"))
        )
        self.graph.add((op_node, ALGO_EXT.out, out_node))
        return out_node

    def _emit_profile_view_node(self, view: Any, owner: Any) -> URIRef:
        """Resolve a profile/admittance measured-velocity view to its value node, registering the
        per-axis component view when the view selects a single axis.
        """
        node = self._view_node(view, owner)
        quantity = getattr(view, "quantity", None)
        if not isinstance(quantity, WorldQuantity):
            return node
        subspace = SUBSPACE_ALIAS.get(
            str(getattr(view, "subspace", "")), str(getattr(view, "subspace", ""))
        )
        axis = semantic_axis_label(getattr(view, "axis", None))
        prop = WORLD_SPECS.get(quantity.type, (None, None, None, {}))[3].get(subspace)
        if axis is None or prop is None or prop[4] is None:
            return node
        view_subspace_uri, _, _, scalar_t, view_type = prop
        self._add_quantity(node, scalar_t)
        view_node = self._owned_uri(f"view-{_scalar_id(quantity, subspace, axis)}", owner)
        if view_node in self._emitted_views:
            return node
        self._emit_view(view_node)
        if quantity.type != WorldQuantityType.Pose:
            self.graph.add((view_node, RDF.type, view_type))
        self.graph.add((view_node, MAP.superobject, URIRef(quantity.uri)))
        self.graph.add((view_node, MAP.subobject, node))
        subspace_value = (
            MAP_EXT.orientation
            if quantity.type == WorldQuantityType.Pose and subspace == "rotation"
            else MAP_EXT.position
            if quantity.type == WorldQuantityType.Pose
            else MAP[view_subspace_uri]
        )
        self.graph.add((view_node, MAP.subspace, subspace_value))
        self.graph.add((view_node, MAP.axis, MAP[axis]))
        return node

    def _elapsed_quantity_node(
        self, spec: ConstraintSpecification, motion: GuardedMotion
    ) -> URIRef:
        """Owned node holding a timing constraint's elapsed-time quantity."""
        return self._owned_uri(f"{spec.name}-elapsed", motion)

    def _motion_time_endpoints(self, motion: GuardedMotion) -> tuple[URIRef, URIRef]:
        """The (motion-entry, current-time) time:Instant pair shared by every elapsed
        constraint in `motion`, emitted once per motion."""
        key = str(motion.uri)
        cached = self._motion_time_endpoints_index.get(key)
        if cached is not None:
            return cached
        entry_node = self._owned_uri("motion-entry", motion)
        current_node = self._owned_uri("current-time", motion)
        self.graph.add((entry_node, RDF.type, TIME.Instant))
        self.graph.add((current_node, RDF.type, TIME.Instant))
        endpoints = (entry_node, current_node)
        self._motion_time_endpoints_index[key] = endpoints
        return endpoints

    def _emit_elapsed_constraint(
        self, node: URIRef, spec: ConstraintSpecification, motion: GuardedMotion
    ) -> None:
        """A timing constraint: a cstr-ext:TimeConstraint over the time:ProperInterval
        spanning motion entry to now, whose measured time:Duration (filled at runtime from
        the world clock) is compared against an authored Duration threshold, reference, or
        tolerance. No kinematics — codegen reads the clock directly."""
        expr = spec.expr
        self.graph.add((node, RDF.type, CSTR.Constraint))
        self.graph.add((node, RDF.type, CSTR_EXT.TimeConstraint))

        qty_node = self._elapsed_quantity_node(spec, motion)
        self.graph.add((qty_node, RDF.type, QUDT_SCHEMA.Quantity))
        self._emit_quantity_kind(qty_node, NS_MM_QUDT_QTY["Time"])
        # No authored value: the clock fills this one, and it ticks in seconds.
        self.graph.add((qty_node, QUDT_SCHEMA.unit, _dsl_unit("s")))
        self.graph.add((node, CSTR.quantity, qty_node))

        entry_node, current_node = self._motion_time_endpoints(motion)
        interval_node = self._owned_uri(f"{spec.name}-interval", motion)
        self.graph.add((interval_node, RDF.type, TIME.ProperInterval))
        self.graph.add((interval_node, TIME.hasBeginning, entry_node))
        self.graph.add((interval_node, TIME.hasEnd, current_node))

        if isinstance(expr, GreaterThanConstraint):
            self.graph.add((node, RDF.type, CSTR.UnilateralConstraint))
            self.graph.add((node, RDF.type, CSTR.GreaterThanConstraint))
            thr_node = self._emit_duration_threshold_node(
                expr.threshold, motion, f"{spec.name}-threshold"
            )
            self.graph.add((node, CSTR.threshold, thr_node))
        elif isinstance(expr, LessThanConstraint):
            self.graph.add((node, RDF.type, CSTR.UnilateralConstraint))
            self.graph.add((node, RDF.type, CSTR.LessThanConstraint))
            thr_node = self._emit_duration_threshold_node(
                expr.threshold, motion, f"{spec.name}-threshold"
            )
            self.graph.add((node, CSTR.threshold, thr_node))
        elif isinstance(expr, EqualityConstraint):
            self.graph.add((node, RDF.type, CSTR.EqualityConstraint))
            ref_node = self._emit_duration_threshold_node(
                expr.reference, motion, f"{spec.name}-reference"
            )
            self.graph.add((node, CSTR["reference-value"], ref_node))
            # Semantic validation guarantees an elapsed equality always carries a tolerance.
            tol_node = self._emit_duration_threshold_node(
                expr.tolerance, motion, f"{spec.name}-tolerance"
            )
            self.graph.add((node, CSTR_EXT.tolerance, tol_node))
        else:
            raise ValueError(
                f"Timing constraint '{spec.name}' must use 'greater than', 'less than', "
                "or 'equal to'."
            )

    def _emit_duration_threshold_node(self, ref: ContextRef, owner: Any, suffix: str) -> URIRef:
        """Resolve a timing threshold/reference/tolerance to a native OWL-Time Duration
        node: an inline literal, or a declared Duration quantity.
        """
        bare = getattr(ref, "bare", None)
        if bare is not None:
            node = self._owned_uri(suffix, owner)
            self._emit_duration_measure(node, bare)
            return node
        quantity = _context_quantity(ref)
        if isinstance(quantity, ContextQuantity):
            return URIRef(_resolved_context_quantity(quantity).uri)
        raise ValueError(
            "Timing threshold must be a declared Duration quantity or an inline literal like `5.0 s`."
        )

    def _emit_duration_measure(self, node: URIRef, value: Measure) -> None:
        """Emit a Measure (`5.0 s`, `10.0 ms`) as a time:Duration whose magnitude is carried
        by qudt, in the unit it was written in.

        owl-time's own magnitude properties cannot say `10 ms` -- `time:unitType` ranges over
        `time:TemporalUnit`, whose smallest member is `time:unitSecond` -- so writing one
        would mean rescaling the authored number. The class still describes what this is;
        only the magnitude moves to the vocabulary that can express it, the same qudt
        Time-kind scalar a monitor's debounce already uses.
        """
        self.graph.add((node, RDF.type, TIME.Duration))
        self._emit_scalar_quantity(
            node, value.value, NS_MM_QUDT_QTY["Time"], _dsl_unit(value.unit)
        )

    def _emit_motion_spec(self, motion: GuardedMotion) -> None:
        """Emit the guarded-motion node linking its when/while/until constraints (with disjunction
        nodes for `any` logic) and any path.
        """
        motion_node = self._owned_uri(f"motion-{motion.name}", motion)
        self.graph.add((motion_node, RDF.type, MOT.GuardedMotion))
        self.graph.add((motion_node, SDO.name, Literal(motion.name)))
        # textX leaves an unmatched optional STRING as '', not None.
        if motion.description:
            self.graph.add((motion_node, SDO.description, Literal(motion.description)))
        raw_when_logic = getattr(motion.when, "logic", None)
        when_constraints = [i for i in motion.when.constraints if not _resolved_spec(i).disabled]
        if raw_when_logic == "any" and when_constraints:
            when_disjunction_node = self._owned_uri(
                f"motion-{motion.name}-when-disjunction", motion
            )
            self.graph.add((when_disjunction_node, RDF.type, CSTR_EXT.ConstraintDisjunction))
            self.graph.add((motion_node, MOT.when, when_disjunction_node))
            for item in when_constraints:
                self.graph.add(
                    (
                        when_disjunction_node,
                        CSTR_EXT["has-constraint"],
                        URIRef(_resolved_spec(item).uri),
                    )
                )
        else:
            for item in when_constraints:
                self.graph.add((motion_node, MOT.when, URIRef(_resolved_spec(item).uri)))
        for item in motion.while_.constraints:
            spec = _resolved_spec(item)
            if not spec.disabled:
                self.graph.add((motion_node, MOT["while"], URIRef(spec.uri)))
        raw_logic = getattr(motion.until, "logic", None)
        # A named group is one transition condition of its own, so it becomes a single
        # conjunction/disjunction node the motion points at and a monitor can target.
        groups = [i for i in motion.until.constraints if isinstance(i, ConstraintGroup)]
        for group in groups:
            members = [i for i in group.constraints if not _resolved_spec(i).disabled]
            if not members:
                continue
            group_node = URIRef(group.uri)
            group_type = (
                CSTR_EXT.ConstraintDisjunction
                if group.logic == "any"
                else CSTR_EXT.ConstraintConjunction
            )
            self.graph.add((group_node, RDF.type, group_type))
            self.graph.add((motion_node, MOT.until, group_node))
            for item in members:
                self.graph.add(
                    (group_node, CSTR_EXT["has-constraint"], URIRef(_resolved_spec(item).uri))
                )
        until_constraints = [
            i
            for i in motion.until.constraints
            if not isinstance(i, ConstraintGroup) and not _resolved_spec(i).disabled
        ]
        if raw_logic == "any" and until_constraints:
            disjunction_node = self._owned_uri(f"motion-{motion.name}-until-disjunction", motion)
            self.graph.add((disjunction_node, RDF.type, CSTR_EXT.ConstraintDisjunction))
            self.graph.add((motion_node, MOT.until, disjunction_node))
            for item in until_constraints:
                self.graph.add(
                    (disjunction_node, CSTR_EXT["has-constraint"], URIRef(_resolved_spec(item).uri))
                )
        else:
            for item in until_constraints:
                self.graph.add((motion_node, MOT.until, URIRef(_resolved_spec(item).uri)))

    def _emit_scalar_views(
        self,
        motion: GuardedMotion,
        constraints: list[ConstraintSpecification],
        world_qtys: dict[str, WorldQuantity],
    ) -> None:
        """Emit the scalar/component nodes and map:Views for each constraint's viewed subspace
        (whole pose, position/orientation vectors, per-axis components), deduplicated per motion.
        """
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
                            self.graph.add(
                                (
                                    pose_scalar,
                                    GEOM_REL["with-respect-to"],
                                    self._owned_uri(wrt_v, qty),
                                )
                            )
                            self.graph.add(
                                (pose_scalar, GEOM_COORD["as-seen-by"], self._owned_uri(wrt_v, qty))
                            )
                continue

            if (
                qty.type == WorldQuantityType.Pose
                and subspace in {"position", "orientation"}
                and axis is None
            ):
                key = (qty.name, subspace, None)
                if key not in seen:
                    seen.add(key)
                    self._view_node(spec.view, motion)
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

            view_node = self._owned_uri(f"view-{sid}", motion)
            self._emit_view(view_node)
            if qty.type != WorldQuantityType.Pose:
                self.graph.add((view_node, RDF.type, view_type))
            self.graph.add((view_node, MAP.superobject, URIRef(qty.uri)))
            self.graph.add((view_node, MAP.subobject, scalar_node))
            subspace_value = (
                MAP_EXT.orientation
                if qty.type == WorldQuantityType.Pose and subspace == "rotation"
                else MAP_EXT.position
                if qty.type == WorldQuantityType.Pose
                else MAP[view_subspace_uri]
            )
            self.graph.add((view_node, MAP.subspace, subspace_value))
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
        motion: GuardedMotion,
        constraints: list[ConstraintSpecification],
        world_qtys: dict[str, WorldQuantity],
    ) -> None:
        """Emit the map/geometry operations feeding constraint views, notably the
        PoseToLinearDistance op producing a distance constraint's scalar from its relative pose.
        """
        rotation_pose = next(
            (
                _node_name(spec.view.quantity)
                for spec in constraints
                if not getattr(spec.view, "is_elapsed", False)
                and _view_subspace(spec) == "rotation"
                and spec.view.axis is None
            ),
            None,
        )
        if rotation_pose is not None:
            rotation_id = f"rotation-{motion.name}"
            op_node = self._owned_uri(f"compute-{rotation_id}", motion)
            self.graph.add((op_node, RDF.type, GEOM_OP_EXT.PoseToAngularDistance))
            self.graph.add((op_node, GEOM_OP.pose, self._owned_uri(rotation_pose, motion)))
            self.graph.add(
                (
                    op_node,
                    GEOM_OP_EXT["angular-distance"],
                    self._owned_uri(rotation_id, motion),
                )
            )

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

        seen_distance_ops = self._emitted_distance_ops
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

            plan = self._distance_plan(spec, world_qtys)
            distance_node = self._owned_uri(distance_id, motion)
            self._add_quantity(distance_node, QuantityType.Distance)
            self.graph.add((distance_node, RDF.type, GEOM_REL.LinearDistance))
            self.graph.add((distance_node, GEOM_REL["between-entities"], URIRef(plan.start.uri)))
            self.graph.add((distance_node, GEOM_REL["between-entities"], URIRef(plan.end.uri)))

    def _emit_controller_base(self, ctrl_node: URIRef, ctrl: ControllerEntry) -> None:
        """Emit a controller's type and gains: PID (kp/ki/kd, optional decay), Impedance
        (stiffness/damping, optional integral gain), or feed-forward.
        """
        self.graph.add((ctrl_node, RDF.type, CSTR_HDL.Controller))
        if ctrl.command_type is not None:
            self.graph.add((ctrl_node, APP["command-type"], Literal(ctrl.command_type.value)))
        if ctrl.type == ControllerType.PID:
            self.graph.add((ctrl_node, RDF.type, CSTR_HDL.ProportionalIntegralDerivative))
            if ctrl.params.kp is not None:
                self.graph.add(
                    (
                        ctrl_node,
                        CSTR_HDL["proportional-gain"],
                        Literal(float(ctrl.params.kp), datatype=XSD.double),
                    )
                )
            if ctrl.params.ki is not None:
                self.graph.add(
                    (
                        ctrl_node,
                        CSTR_HDL["integral-gain"],
                        Literal(float(ctrl.params.ki), datatype=XSD.double),
                    )
                )
            if ctrl.params.kd is not None:
                self.graph.add(
                    (
                        ctrl_node,
                        CSTR_HDL["derivative-gain"],
                        Literal(float(ctrl.params.kd), datatype=XSD.double),
                    )
                )
            if ctrl.params.decay is not None:
                self.graph.add((ctrl_node, RDF.type, CSTR_HDL.DecayingIntegralTerm))
                self.graph.add(
                    (
                        ctrl_node,
                        CSTR_HDL["decay-rate"],
                        Literal(float(ctrl.params.decay), datatype=XSD.double),
                    )
                )
        elif ctrl.type == ControllerType.Impedance:
            self.graph.add((ctrl_node, RDF.type, CSTR_HDL.ImpedanceController))
            if ctrl.params.stiffness is not None:
                k_node = URIRef(f"{ctrl_node}-stiffness")
                self.graph.add(
                    (
                        k_node,
                        QUDT_SCHEMA.value,
                        Literal(float(ctrl.params.stiffness), datatype=XSD.double),
                    )
                )
                self.graph.add((ctrl_node, CSTR_HDL["stiffness"], k_node))
            if ctrl.params.damping is not None:
                d_node = URIRef(f"{ctrl_node}-damping")
                self.graph.add(
                    (
                        d_node,
                        QUDT_SCHEMA.value,
                        Literal(float(ctrl.params.damping), datatype=XSD.double),
                    )
                )
                self.graph.add((ctrl_node, CSTR_HDL["damping"], d_node))
            # Optional integral gain (Ki): integral action to null steady-state error.
            if ctrl.params.ki is not None:
                i_node = URIRef(f"{ctrl_node}-integral")
                self.graph.add(
                    (i_node, QUDT_SCHEMA.value, Literal(float(ctrl.params.ki), datatype=XSD.double))
                )
                self.graph.add((ctrl_node, CSTR_HDL["integral-gain"], i_node))
        elif ctrl.type == ControllerType.FeedForward:
            self.graph.add((ctrl_node, RDF.type, CSTR_HDL_EXT.FeedForwardController))
        else:
            raise ValueError(
                f"Controller '{ctrl.name}' uses {ctrl.type.value}, "
                "which is not supported for graph emission."
            )

    def _emit_saturation(
        self,
        owner_node: URIRef,
        node: URIRef,
        saturation: SaturationSpec,
        input_signal: URIRef,
        output_signal: URIRef,
        owner: Any,
    ) -> URIRef:
        """Attach a generic saturation from `input_signal` to `output_signal`."""
        self.graph.add((node, RDF.type, ALGO_EXT.Saturation))
        self.graph.add((node, ALGO_EXT["in"], input_signal))
        self.graph.add((node, ALGO_EXT.out, output_signal))
        if saturation.maximum is not None:
            limit_node = self._emit_context_ref_node(saturation.maximum, owner, "max")
            self.graph.add((node, ALGO_EXT["maximum-absolute-value"], limit_node))
        else:
            assert saturation.lower is not None and saturation.upper is not None
            lower_node = self._emit_context_ref_node(saturation.lower, owner, "lower")
            upper_node = self._emit_context_ref_node(saturation.upper, owner, "upper")
            self.graph.add((node, ALGO_EXT["lower-bound"], lower_node))
            self.graph.add((node, ALGO_EXT["upper-bound"], upper_node))
        self.graph.add((owner_node, ALGO_EXT.limits, node))
        return node

    def _emit_controller_limits(
        self,
        controller_node: URIRef,
        controller: ControllerEntry,
        command,
        handler: ConstraintHandler,
    ) -> None:
        """Emit authored controller limits without choosing a solver representation."""
        if controller.params.output_saturation is not None:
            output = self._owned_uri(f"output-{controller.name}", handler)
            self._add_quantity(output, command.command_type)
            self._emit_saturation(
                controller_node,
                self._owned_uri(f"sat-output-{controller.name}", handler),
                controller.params.output_saturation,
                output,
                output,
                controller,
            )
        if controller.params.integral_saturation is not None:
            integral = self._owned_uri(f"integral-state-{controller.name}", handler)
            self.graph.add((integral, RDF.type, QUDT_SCHEMA.Quantity))
            self._emit_saturation(
                controller_node,
                self._owned_uri(f"sat-integral-{controller.name}", handler),
                controller.params.integral_saturation,
                integral,
                integral,
                controller,
            )

    def _emit_error_evaluator(
        self,
        handler_node: URIRef,
        spec: ConstraintSpecification,
        error_node: URIRef,
        seen_eval_ids: set[str],
    ) -> None:
        """Emit (once per evaluator id) a constraint error evaluator linking `spec` to
        `error_node`, and attach it to the handler.
        """
        eval_id = _evaluator_id(spec)
        eval_node = self._owned_uri(eval_id, spec.parent)
        if eval_id not in seen_eval_ids:
            seen_eval_ids.add(eval_id)
            self.graph.add((eval_node, RDF.type, CSTR_HDL.ConstraintEvaluator))
            self.graph.add((eval_node, RDF.type, CSTR_HDL.ErrorEvaluator))
            self.graph.add((eval_node, CSTR_HDL.constraint, URIRef(spec.uri)))
            self.graph.add((eval_node, CSTR_HDL.error, error_node))
        self.graph.add((handler_node, CSTR_HDL.evaluators, eval_node))

    def _emit_ros_topic(self, monitor: Any, monitor_node: URIRef) -> None:
        """Describe a monitor backed by a ROS topic."""
        topic = monitor.ros_topic
        if topic is None:
            return
        self.graph.add((monitor_node, RDF.type, ROS.Topic))
        self.graph.add((monitor_node, ROS["channel-name"], Literal(topic.channel_name)))
        self.graph.add(
            (
                monitor_node,
                ROS["type-name"],
                Literal(topic.type_name or "std_msgs/msg/Empty"),
            )
        )

    def _emit_constraint_handler(
        self,
        handler: ConstraintHandler,
        motion: GuardedMotion,
        world_qtys: dict[str, WorldQuantity],
        shared_spec_ids: frozenset[ConstraintSpecification],
        handler_order: int,
    ) -> None:
        """Emit a constraint handler: control mode, event loop, and per-controller controllers,
        evaluators, control signals and saturations.
        """
        handler_node = URIRef(handler.uri)
        self.graph.add((handler_node, RDF.type, CSTR_HDL.ConstraintHandler))
        self.graph.add((handler_node, APP.order, Literal(handler_order)))
        self.graph.add(
            (handler_node, CSTR_HDL.motion, self._owned_uri(f"motion-{motion.name}", motion))
        )
        event_loop_node = URIRef(f"{handler.uri}.event-loop")

        seen_error_ids: set[str] = set()
        seen_eval_ids: set[str] = set()

        for controller_order, ctrl_item in enumerate(getattr(handler, "controllers", [])):
            ctrl = ctrl_item.ref.controller if hasattr(ctrl_item, "ref") else ctrl_item
            cref = ctrl.params.constraint
            spec = cref.constraint if hasattr(cref, "constraint") else None
            if spec is None:
                continue
            if spec.disabled:
                continue

            along_path = self._along_path_scalar(spec)
            qty = self._resolve_constraint_quantity(spec, world_qtys)
            if qty is None and along_path is None:
                raise ValueError(
                    f"Controller '{ctrl.name}' constraint '{spec.name}' does not resolve to a world quantity."
                )
            subspace = _view_subspace(spec)
            axis_raw = spec.view.axis
            axis = semantic_axis_label(axis_raw)
            shared = spec in shared_spec_ids
            scalar_t = (
                along_path[1]
                if along_path
                else (_scalar_type(qty, subspace, axis) if qty else subspace)
            )
            command = controller_command_record(ctrl)

            authored_ctrl_node = URIRef(ctrl.uri)
            self._emit_controller_base(authored_ctrl_node, ctrl)
            self.graph.add((authored_ctrl_node, APP.order, Literal(controller_order)))
            self.graph.add((authored_ctrl_node, CSTR_HDL.constraint, URIRef(spec.uri)))
            measured_derivative = getattr(ctrl.params, "measured_derivative", None)
            derivative_quantity = getattr(measured_derivative, "quantity", None)
            if isinstance(derivative_quantity, WorldQuantity):
                # An authored axis names one scalar; a bare subspace leaves the choice to the
                # per-axis controller, which reads the component matching the axis it drives.
                derivative_node = (
                    self._emit_profile_view_node(measured_derivative, handler)
                    if getattr(measured_derivative, "axis", None) is not None
                    else URIRef(_resolved_world_quantity(derivative_quantity).uri)
                )
                self.graph.add((authored_ctrl_node, CSTR_HDL["measured-velocity"], derivative_node))
            solver = self._controller_solver(handler, ctrl)
            if solver is not None:
                self.graph.add(
                    (
                        authored_ctrl_node,
                        CSTR_HDL_EXT.solver,
                        self._solver_node(handler, motion, solver),
                    )
                )

            self._emit_controller_limits(authored_ctrl_node, ctrl, command, handler)

            controller_error_id: str | None = None
            evaluator_error_id: str | None = None
            if qty is not None or along_path is not None:
                sid = along_path[0] if along_path else _scalar_id(qty, subspace, axis)
                candidate_error_id = (sid + "-err") if shared else f"{sid}-err-{motion.name}"
                if ctrl.type != ControllerType.FeedForward:
                    controller_error_id = candidate_error_id
                    evaluator_error_id = candidate_error_id
                elif isinstance(spec.expr, EqualityConstraint):
                    evaluator_error_id = candidate_error_id

            # Pose equality can expand to any requested acceleration components:
            # full pose -> 6D, position-only -> 3D, or orientation-only -> 3D.
            if (
                qty is not None
                and qty.type == WorldQuantityType.Pose
                and subspace in {"pose", "position", "orientation", "distance", "rotation"}
                and axis is None
                and command.controlled_axes
                and (isinstance(spec.expr, EqualityConstraint) or spec.view.on is not None)
                and not _is_distance_view(spec)
            ):
                self.graph.add((handler_node, CSTR_HDL.controllers, authored_ctrl_node))
                continue

            ctrl_node = authored_ctrl_node
            if controller_error_id:
                self.graph.add(
                    (
                        ctrl_node,
                        CSTR_HDL["error-signal"],
                        self._owned_uri(controller_error_id, spec.parent),
                    )
                )

            if ctrl.type == ControllerType.FeedForward and isinstance(
                spec.expr, EqualityConstraint
            ):
                ref_qty = _context_quantity(spec.expr.reference)
                if ref_qty is not None:
                    self.graph.add(
                        (ctrl_node, CSTR_HDL_EXT["reference-signal"], URIRef(ref_qty.uri))
                    )

            self.graph.add((handler_node, CSTR_HDL.controllers, ctrl_node))

            if evaluator_error_id and evaluator_error_id not in seen_error_ids:
                seen_error_ids.add(evaluator_error_id)
                err_node = self._owned_uri(evaluator_error_id, spec.parent)
                self._add_quantity(err_node, scalar_t)
                if (
                    scalar_t == QuantityType.Pose
                    and qty is not None
                    and isinstance(qty.props, GeometricProps)
                ):
                    of_v = _geo_prop(qty.props, "of")
                    wrt_v = _geo_prop(qty.props, "wrt")
                    if of_v and wrt_v:
                        self.graph.add((err_node, GEOM_REL.of, self._owned_uri(of_v, qty)))
                        self.graph.add(
                            (err_node, GEOM_REL["with-respect-to"], self._owned_uri(wrt_v, qty))
                        )
                        self.graph.add(
                            (err_node, GEOM_COORD["as-seen-by"], self._owned_uri(wrt_v, qty))
                        )

            if evaluator_error_id:
                self._emit_error_evaluator(
                    handler_node,
                    spec,
                    self._owned_uri(evaluator_error_id, spec.parent),
                    seen_eval_ids,
                )

        for mon in getattr(handler, "monitors", []):
            cref = mon.constraint
            is_event = mon.event is not None
            signal_kind = "event" if is_event else "flag"
            signal_node = URIRef(mon.event.uri) if is_event else URIRef(f"{mon.uri}.{mon.flag}")
            mon_node = URIRef(mon.uri)

            # A group monitor aggregates exactly like a whole-section one, over the group's
            # members, and points at the group node so the logic travels with it.
            group_target = (
                cref.constraint
                if not isinstance(cref, (UntilMonitorRef, WhenMonitorRef))
                and isinstance(getattr(cref, "constraint", None), ConstraintGroup)
                else None
            )
            if isinstance(cref, (UntilMonitorRef, WhenMonitorRef)) or group_target is not None:
                is_when_ref = isinstance(cref, WhenMonitorRef)
                if group_target is not None:
                    section_constraints = list(group_target.constraints)
                else:
                    section_constraints = _flatten_constraint_items(
                        cref.motion.when.constraints
                        if is_when_ref
                        else cref.motion.until.constraints
                    )
                section_specs = [
                    _resolved_spec(item)
                    for item in section_constraints
                    if not _resolved_spec(item).disabled
                ]
                if group_target is not None:
                    guard_nodes = [URIRef(group_target.uri)]
                else:
                    section = cref.motion.when if is_when_ref else cref.motion.until
                    if section.logic == "any":
                        guard_nodes = [
                            self._owned_uri(
                                f"motion-{cref.motion.name}-{'when' if is_when_ref else 'until'}-disjunction",
                                cref.motion,
                            )
                        ]
                    else:
                        guard_nodes = [URIRef(spec.uri) for spec in section_specs]
                component_error_nodes: list[URIRef] = []
                for spec in section_specs:
                    if getattr(spec.view, "is_elapsed", False):
                        error_node = self._elapsed_quantity_node(spec, cref.motion)
                        component_error_nodes.append(error_node)
                        self._emit_error_evaluator(
                            handler_node,
                            spec,
                            error_node,
                            seen_eval_ids,
                        )
                        continue
                    qty = self._resolve_constraint_quantity(spec, world_qtys)
                    if qty is None:
                        raise ValueError(
                            f"Aggregate monitor '{mon.name}' constraint '{spec.name}' does not resolve to a world quantity."
                        )
                    subspace = _view_subspace(spec)
                    axis_raw = spec.view.axis
                    axis = semantic_axis_label(axis_raw)
                    scalar_t = _scalar_type(qty, subspace, axis) if qty else subspace
                    error_id = f"{_evaluator_id(spec)}-err"

                    if error_id not in seen_error_ids:
                        seen_error_ids.add(error_id)
                        err_node = self._owned_uri(error_id, spec.parent)
                        self._add_quantity(err_node, scalar_t)
                        if (
                            scalar_t == QuantityType.Pose
                            and qty is not None
                            and isinstance(qty.props, GeometricProps)
                        ):
                            of_v = _geo_prop(qty.props, "of")
                            wrt_v = _geo_prop(qty.props, "wrt")
                            if of_v and wrt_v:
                                self.graph.add((err_node, GEOM_REL.of, self._owned_uri(of_v, qty)))
                                self.graph.add(
                                    (
                                        err_node,
                                        GEOM_REL["with-respect-to"],
                                        self._owned_uri(wrt_v, qty),
                                    )
                                )
                                self.graph.add(
                                    (
                                        err_node,
                                        GEOM_COORD["as-seen-by"],
                                        self._owned_uri(wrt_v, qty),
                                    )
                                )

                    self._emit_error_evaluator(
                        handler_node,
                        spec,
                        self._owned_uri(error_id, spec.parent),
                        seen_eval_ids,
                    )
                    component_error_nodes.append(self._owned_uri(error_id, spec.parent))

                if len(component_error_nodes) == 1:
                    aggregate_error_node = component_error_nodes[0]
                else:
                    aggregate_error_node = URIRef(f"{mon.uri}.error")
                    self._add_quantity(aggregate_error_node, QuantityType.FreeVector)

                self.graph.add(
                    (signal_node, RDF.type, EL.Event if signal_kind == "event" else EL.Flag)
                )
                self.graph.add(
                    (
                        event_loop_node,
                        EL["has-event"] if signal_kind == "event" else EL["has-flag"],
                        signal_node,
                    )
                )
                self.graph.add((mon_node, RDF.type, CSTR_HDL.Monitor))
                self.graph.add((mon_node, CSTR_HDL.error, aggregate_error_node))
                for guard_node in guard_nodes:
                    self.graph.add((mon_node, CSTR_HDL.constraint, guard_node))
                if signal_kind == "event":
                    self.graph.add((event_loop_node, RDF.type, EL.EventLoop))
                    self.graph.add((mon_node, RDF.type, CSTR_HDL.EdgeTriggeredMonitor))
                    self.graph.add((mon_node, CSTR_HDL.event, signal_node))
                    self.graph.add((mon_node, CSTR_HDL["event-queue"], event_loop_node))
                    self._emit_ros_topic(mon, mon_node)
                    if mon.fallback is not None:
                        self.graph.add(
                            (
                                mon_node,
                                CSTR_HDL_EXT["fallback-motion"],
                                self._owned_uri(f"motion-{mon.fallback.name}", mon.fallback),
                            )
                        )
                    if mon.debounce_duration is not None:
                        debounce_node = URIRef(f"{mon.uri}.debounce")
                        self._emit_scalar_quantity(
                            debounce_node,
                            mon.debounce_duration,
                            NS_MM_QUDT_QTY["Time"],
                            _dsl_unit(mon.debounce_unit),
                        )
                        self.graph.add((mon_node, CSTR_HDL_EXT["debounce-duration"], debounce_node))
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

            if getattr(spec.view, "is_elapsed", False):
                error_node = self._elapsed_quantity_node(spec, cref.motion)
            else:
                along_path = self._along_path_scalar(spec)
                qty = self._resolve_constraint_quantity(spec, world_qtys)
                if qty is None and along_path is None:
                    raise ValueError(
                        f"Monitor '{mon.name}' constraint '{spec.name}' does not resolve to a world quantity."
                    )
                subspace = _view_subspace(spec)
                axis_raw = spec.view.axis
                axis = semantic_axis_label(axis_raw)
                scalar_t = along_path[1] if along_path else _scalar_type(qty, subspace, axis)
                error_id = f"{_evaluator_id(spec)}-err"
                error_node = self._owned_uri(error_id, spec.parent)
                if error_id not in seen_error_ids:
                    seen_error_ids.add(error_id)
                    self._add_quantity(error_node, scalar_t)
                    if (
                        scalar_t == QuantityType.Pose
                        and qty is not None
                        and isinstance(qty.props, GeometricProps)
                    ):
                        of_v = _geo_prop(qty.props, "of")
                        wrt_v = _geo_prop(qty.props, "wrt")
                        if of_v and wrt_v:
                            self.graph.add((error_node, GEOM_REL.of, self._owned_uri(of_v, qty)))
                            self.graph.add(
                                (
                                    error_node,
                                    GEOM_REL["with-respect-to"],
                                    self._owned_uri(wrt_v, qty),
                                )
                            )
                            self.graph.add(
                                (error_node, GEOM_COORD["as-seen-by"], self._owned_uri(wrt_v, qty))
                            )

            self._emit_error_evaluator(
                handler_node,
                spec,
                error_node,
                seen_eval_ids,
            )

            self.graph.add((signal_node, RDF.type, EL.Event if signal_kind == "event" else EL.Flag))
            self.graph.add(
                (
                    event_loop_node,
                    EL["has-event"] if signal_kind == "event" else EL["has-flag"],
                    signal_node,
                )
            )
            self.graph.add((mon_node, RDF.type, CSTR_HDL.Monitor))
            self.graph.add((mon_node, CSTR_HDL.constraint, URIRef(spec.uri)))
            self.graph.add((mon_node, CSTR_HDL.error, error_node))
            if signal_kind == "event":
                self.graph.add((event_loop_node, RDF.type, EL.EventLoop))
                self.graph.add((mon_node, RDF.type, CSTR_HDL.EdgeTriggeredMonitor))
                self.graph.add((mon_node, CSTR_HDL.event, signal_node))
                self.graph.add((mon_node, CSTR_HDL["event-queue"], event_loop_node))
                self._emit_ros_topic(mon, mon_node)
                if mon.fallback is not None:
                    self.graph.add(
                        (
                            mon_node,
                            CSTR_HDL_EXT["fallback-motion"],
                            self._owned_uri(f"motion-{mon.fallback.name}", mon.fallback),
                        )
                    )
                if mon.debounce_duration is not None:
                    debounce_node = URIRef(f"{mon.uri}.debounce")
                    self._emit_scalar_quantity(
                        debounce_node,
                        mon.debounce_duration,
                        NS_MM_QUDT_QTY["Time"],
                        _dsl_unit(mon.debounce_unit),
                    )
                    self.graph.add((mon_node, CSTR_HDL_EXT["debounce-duration"], debounce_node))
            else:
                self.graph.add((mon_node, RDF.type, CSTR_HDL.LevelTriggeredMonitor))
                self.graph.add((mon_node, CSTR_HDL.flag, signal_node))
            self.graph.add((handler_node, CSTR_HDL.monitors, mon_node))

    def _forwarded_command_signal(
        self,
        ctrl: ControllerEntry,
        qty: WorldQuantity | None,
        subspace: str,
        axis: str | None,
        handler: ConstraintHandler,
    ) -> URIRef:
        """Emit the authored signal forwarded directly to a device command."""
        signal = self._owned_uri(f"cmd-{ctrl.name}", handler)
        signal_type = _scalar_type(qty, subspace, axis) if qty else QuantityType.FreeVector
        self._add_quantity(signal, signal_type)
        return signal

    def _solver_node(self, handler, motion, solver) -> URIRef:
        return self._owned_uri(f"{solver.name}-{motion.name}", handler)

    def _emit_solvers(
        self,
        handler: ConstraintHandler,
        motion: GuardedMotion,
        world_qtys: dict[str, WorldQuantity],
    ) -> None:
        """Emit each of a handler's solvers, routed by mechanism: serial-chain dynamics,
        mobile-platform velocity/force, or command forwarding."""
        solvers = [_resolved_solver(s) for s in getattr(handler, "solvers", [])]
        multi = len(solvers) > 1

        for solver in solvers:
            solver_node = self._solver_node(handler, motion, solver)
            robot_uri = getattr(solver.agent, "uri", None)
            if robot_uri:
                self.graph.add((solver_node, AGN["of-agent"], URIRef(robot_uri)))

            if isinstance(solver, MobilePlatformSolver):
                quantity = _context_quantity(solver.quantity)
                self.graph.add((solver_node, SLV.configuration, Literal(solver.configuration)))
                rdf_class, predicate = _MOBILE_PLATFORM_ALGORITHM_RDF[solver.algorithm]
                self.graph.add((solver_node, RDF.type, rdf_class))
                if quantity is not None:
                    self.graph.add((solver_node, predicate, URIRef(quantity.uri)))
                continue

            driver_stem = (
                f"{solver.name}-{motion.name or handler.name}"
                if multi
                else (motion.name or handler.name)
            )

            driver_node = self._owned_uri(f"driver-{driver_stem}", handler)
            self.graph.add((driver_node, RDF.type, SLV.MotionDrivers))

            if isinstance(solver, CommandForwardingSolver):
                self.graph.add((solver_node, RDF.type, SLV_EXT.CommandForwardingSolver))
                self.graph.add((solver_node, SLV["motion-drivers"], driver_node))
                self._emit_solver_interfaces(
                    handler,
                    motion,
                    solver,
                    solver_node,
                    driver_node,
                    world_qtys,
                )
                continue

            # SerialChainSolver: ACHD/RNE dynamics over one ordered kinematic chain.
            self.graph.add((solver_node, RDF.type, SLV.SolverWithInputAndOutput))

            alg = solver.algorithm
            alg_node = (
                SLV.AccelerationConstrainedHybridDynamicsAlgorithm
                if alg == "ACHD"
                else SLV["RecursiveNewtonEulerAlgorithm"]
            )
            self.graph.add((solver_node, SLV.solver, alg_node))

            gravity_value = getattr(solver, "gravity_value", None)
            if gravity_value is not None:
                gravity_ref = _context_quantity(getattr(gravity_value, "ref", None))
                if gravity_ref is not None:
                    gravity_value_node = URIRef(gravity_ref.uri)
                else:
                    gravity_value_node = self._owned_uri(f"gravity-value-{solver.name}", handler)
                    self._add_quantity(gravity_value_node, QuantityType.FreeVector)
                    self.graph.add((gravity_value_node, RDF.type, GEOM_COORD.VectorXYZ))
                    for label, element in zip(("x", "y", "z"), gravity_value.coords.values):
                        if element.ref is not None:
                            value_obj = self._emit_context_ref_node(
                                element.ref, handler, f"gravity-{label}"
                            )
                        else:
                            value_obj = Literal(
                                float(element.value), datatype=XSD.double
                            )
                        self.graph.add((gravity_value_node, GEOM_COORD[label], value_obj))
                    self.graph.remove((gravity_value_node, QUDT_SCHEMA.unit, None))
                    self.graph.add(
                        (gravity_value_node, QUDT_SCHEMA.unit, _dsl_unit(gravity_value.unit))
                    )
                self.graph.add((solver_node, SLV.gravity, gravity_value_node))

            solver_limits_by_target = {
                str(entry.target): entry.saturation
                for entry in getattr(getattr(solver, "limits", None), "entries", [])
            }
            limit_quantity_types = {
                "torque": QuantityType.Torque,
            }
            for target, quantity_type in limit_quantity_types.items():
                saturation = solver_limits_by_target.get(target)
                if saturation is None:
                    continue
                signal_name = f"torque-output-{solver.name}"
                signal = self._owned_uri(signal_name, handler)
                self._add_quantity(signal, quantity_type)
                self._emit_saturation(
                    solver_node,
                    self._owned_uri(f"sat-{target}-{solver.name}", handler),
                    saturation,
                    signal,
                    signal,
                    solver,
                )

            self.graph.add((solver_node, SLV["motion-drivers"], driver_node))

            self._emit_solver_interfaces(
                handler,
                motion,
                solver,
                solver_node,
                driver_node,
                world_qtys,
            )

    def _emit_solver_interfaces(
        self,
        handler: ConstraintHandler,
        motion: GuardedMotion,
        solver: Any,
        solver_node: URIRef,
        driver_node: URIRef,
        world_qtys: dict[str, WorldQuantity],
    ) -> None:
        """Emit authored force, forwarding, and auxiliary solver interfaces."""

        for ctrl_item in getattr(handler, "controllers", []):
            ctrl = ctrl_item.ref.controller if hasattr(ctrl_item, "ref") else ctrl_item
            if self._controller_solver(handler, ctrl) is not solver:
                continue

            cref = ctrl.params.constraint
            spec = cref.constraint if hasattr(cref, "constraint") else None
            if spec is None:
                continue

            qty = self._resolve_constraint_quantity(spec, world_qtys)
            if qty is None and self._along_path_scalar(spec) is None:
                raise ValueError(
                    f"Controller '{ctrl.name}' constraint '{spec.name}' does not resolve to a world quantity."
                )

            subspace = _view_subspace(spec)
            axis_raw = spec.view.axis
            axis = semantic_axis_label(axis_raw)
            command = controller_command_record(ctrl)

            if solver.algorithm == "CommandForwarding" and ctrl.type == ControllerType.FeedForward:
                control_signal_node = self._forwarded_command_signal(
                    ctrl, qty, subspace, axis, handler
                )
                self.graph.add((solver_node, SLV.output, control_signal_node))
                continue

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
                    # The scene reference is already the rigid body.
                    body_node = URIRef(str(apply_at.uri))
                    self.graph.add((body_node, RDF.type, GEOM_ENT.SimplicialComplex))
                    self.graph.add(
                        (
                            spec_node,
                            SLV["attached-to"],
                            body_node,
                        )
                    )
                self.graph.add((driver_node, SLV["cartesian-force"], spec_node))

            if (
                solver.algorithm == "ACHD"
                and command.is_posture_torque_command
                and qty.type == WorldQuantityType.JointPosition
            ):
                torque_id = f"tau-{ctrl.name}"
                torque_node = self._owned_uri(torque_id, handler)
                self.graph.add((torque_node, RDF.type, QUDT_SCHEMA.Quantity))
                self.graph.add((torque_node, RDF.type, KC_STAT.JointReference))
                self.graph.add((torque_node, RDF.type, KC_STAT.JointForce))
                self.graph.add((torque_node, RDF.type, KC_STAT.JointForceCoordinate))
                self.graph.add((torque_node, QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND.Torque))
                self.graph.add((torque_node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M"]))

                joint_name = _geo_prop(
                    qty.props if isinstance(qty.props, GeometricProps) else None, "joint"
                )
                if joint_name is None:
                    raise ConstraintViolation(
                        "kinematic-chain",
                        f"Joint torque command '{torque_id}' has no target joint",
                    )
                joint_node = self._owned_uri(joint_name, qty)
                self.graph.add((torque_node, KC_STAT["of-joint"], joint_node))

                spec_node = self._owned_uri(f"jf-spec-{ctrl.name}", handler)
                self.graph.add((spec_node, RDF.type, SLV.JointForceSpecification))
                self.graph.add((spec_node, SLV.force, torque_node))
                self.graph.add((solver_node, SLV["output"], URIRef(qty.uri)))
                self.graph.add((spec_node, SLV["attached-to"], joint_node))
                self.graph.add((driver_node, SLV["joint-force"], spec_node))

    def _controller_solver(self, handler: ConstraintHandler, ctrl: ControllerEntry) -> Any:
        """Return the shared semantic solver resolution for `ctrl`."""
        return controller_solver(handler, ctrl)

    def _add_quantity(self, node: URIRef, scalar_type: Any) -> None:
        """Type `node` as a QUDT quantity of `scalar_type` (kind + unit), adding a
        LinearDistanceCoordinate for distances.
        """
        qkind = QUDT_KIND_BY_QUANTITY_TYPE.get(scalar_type)
        if qkind is None:
            qkind = QUDT_QKIND[scalar_type]
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        if scalar_type == QuantityType.Distance:
            self.graph.add((node, RDF.type, GEOM_COORD.LinearDistanceCoordinate))
        self._emit_quantity_kind(node, qkind)
        self.graph.add((node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(scalar_type, QUDT_UNIT.UNITLESS)))
