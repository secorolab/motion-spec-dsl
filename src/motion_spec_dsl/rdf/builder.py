# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Emit the motion-specification RDF/JSON-LD graph from a parsed DSL model.

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
from rdflib.namespace import Namespace, RDF, XSD
from rdflib.term import Literal, URIRef
from rdf_utils.models.vocab import URI_QUDT_QK_LENGTH, URI_QUDT_QK_MASS
from rdf_utils.namespace import (
    NS_MM_GEOM_REL,
    NS_MM_QUDT_QTY,
    NS_MM_QUDT_UNIT as QUDT_UNIT,
)

from textx.scoping import get_included_models

from motion_spec.namespace import (
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
)
from motion_spec_dsl.controller_semantics import (
    SUBSPACE_ALIAS,
    axis_label as semantic_axis_label,
    controller_command_record,
)
from motion_spec_dsl.classes import (
    BilateralConstraint,
    OutsideConstraint,
    ConstraintGroup,
    ConstraintHandler,
    ConstraintSpecification,
    ContextDeclReference,
    ControllerEntry,
    ControllerType,
    ContextRef,
    ContextSpec,
    EqualityConstraint,
    ExecutionContext,
    GeoPropPair,
    GeometricPropKey,
    GeometricProps,
    GreaterThanConstraint,
    LessThanConstraint,
    Model,
    GuardedMotion,
    PostContextDecl,
    PreContextDecl,
    ProfileSpec,
    AdmittanceSpec,
    QuantityType,
    ReferenceGeneratorType,
    PoseValue,
    ReferenceValue,
    SaturationSpec,
    Measure,
    SnapshotValue,
    SpecContextDecl,
    ContextQuantity,
    TrajectoryValue,
    UntilMonitorRef,
    WhenMonitorRef,
    VectorQuantity,
    WorldContextDecl,
    WorldQuantity,
    WorldQuantityType,
    _resolved_context_quantity,
    _flatten_constraint_items,
    _resolved_spec,
    _resolved_solver,
    _resolved_world_quantity,
)

from motion_spec_dsl.rdf._specs import (
    WORLD_SPECS,
    SCALAR_UNIT,
    CSTR_TYPE_NAME,
    CONSTRAINT_TYPE_OVERRIDE,
    QUDT_KIND_BY_QUANTITY_TYPE,
    _QKIND_PREFIX,
    CONTEXT_COMPOSITE_WORLD_TYPE,
    GRAPH_BINDINGS,
)
from motion_spec_dsl.rdf._helpers import (
    _ns_term,
    _node_name,
    _geo_prop,
    _is_distance_view,
    _view_subspace,
    _scalar_id,
    _axis_vector,
    _scalar_type,
    _evaluator_id,
    _dsl_unit,
    _time_unit,
    _linear_velocity_mps,
    _context_quantity,
    _resolved_constraint_items,
    _DistancePlan,
)


def _constraint_type_iri(scalar_t: Any) -> URIRef:
    """Grounded domain-constraint IRI for a scalar quantity type."""
    name = CSTR_TYPE_NAME.get(scalar_t, scalar_t)
    override = CONSTRAINT_TYPE_OVERRIDE.get(name)
    if override is not None:
        namespace, local = override
        return _ns_term(namespace, local)
    return _ns_term(CSTR, f"{name}Constraint")


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
            context[prefix] = str(ns._NS)

        for model in self.models:
            for spec in getattr(model, "specs", []):
                if isinstance(spec, ExecutionContext):
                    self._emit_execution_context(spec)
                    self.dataset.bind(spec.ns_prefix, spec.ns.uri)
                    context[spec.ns_prefix] = spec.ns.uri
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
            context.timestep,
            NS_MM_QUDT_QTY["Time"],
            _dsl_unit(context.timestep_unit),
        )
        self.graph.add((node, EXEC.timestep, timestep))
        if context.platform.name:
            self.graph.add((node, EXEC["platform-name"], Literal(context.platform.name)))
        if context.platform.version:
            self.graph.add(
                (node, EXEC["platform-version"], Literal(context.platform.version))
            )

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
        if not str(qkind).startswith(_QKIND_PREFIX):
            self.graph.add((node, RDF.type, qkind))
        elif qkind == QUDT_QKIND.PlaneAngle:
            # The angle shapes express quantity-kind membership with sh:class.
            self.graph.add((node, RDF.type, qkind))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], qkind))

    def _retag_as_position_kind(self, node: URIRef) -> None:
        """Replace `node`'s hasQuantityKind with Position.

        PoseCoordinateView position subobjects must report hasQuantityKind=Position for the
        SHACL shape and ir_gen's single-valued lookup, overriding an earlier Distance tag.
        """
        self.graph.remove((node, QUDT_SCHEMA["hasQuantityKind"], None))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Position))

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
        self.graph.add((node, RDF.type, GEOM_REL.Position))
        self.graph.add((node, RDF.type, GEOM_COORD.PositionCoordinate))
        self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
        self.graph.add((node, GEOM_REL.of, self._owned_uri(of_frame, source_qty)))
        self.graph.add(
            (
                node,
                GEOM_REL["with-respect-to"],
                self._owned_uri(wrt_frame, source_qty),
            )
        )
        self.graph.add((node, GEOM_COORD["as-seen-by"], self._owned_uri(as_seen_by, source_qty)))

    def _emit_declared_pose_frame_metadata(
        self,
        node: URIRef,
        quantity: ContextQuantity,
    ) -> None:
        """Attach explicitly authored of/wrt/as-seen-by frames to a context pose."""
        props = quantity.props if isinstance(quantity.props, GeometricProps) else None
        if props is not None:
            of_frame = _geo_prop(props, "of")
            wrt_frame = _geo_prop(props, "wrt")
            if of_frame is None or wrt_frame is None:
                return
            of_node = self._owned_uri(of_frame, quantity)
            wrt_node = self._owned_uri(wrt_frame, quantity)
            self.graph.add((node, GEOM_REL.of, of_node))
            self.graph.add((node, GEOM_REL["with-respect-to"], wrt_node))
            self.graph.add((node, GEOM_COORD["as-seen-by"], wrt_node))
            self._frame_coords_index[node] = (of_node, wrt_node, wrt_node)
            return

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
        self.graph.add(
            (node, QUDT_SCHEMA["hasQuantityKind"], NS_MM_QUDT_QTY["Dimensionless"])
        )
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
        self.graph.add((node, RDF.type, GEOM_REL.Position))
        self.graph.add((node, RDF.type, GEOM_COORD.PositionCoordinate))
        self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
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
        """Emit a Wrench coordinate (force+torque VectorXYZ) at `reference_point`, as seen by `as_seen_by`."""
        self.graph.add((node, RDF.type, RBDYN_ENT.Wrench))
        self.graph.add((node, RDF.type, RBDYN_COORD.WrenchCoordinate))
        self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Torque))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Force))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M"]))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.N))
        self.graph.add((node, RBDYN_ENT["reference-point"], reference_point))
        self.graph.add((node, RBDYN_COORD["as-seen-by"], as_seen_by))

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

            if qty.type == WorldQuantityType.Wrench:
                ft_ref = _geo_prop(props, "ft-sensor")
                if ft_ref:
                    self.graph.add((node, RDF.type, SOSA.Observation))
                    self.graph.add((node, SOSA.madeBySensor, URIRef(ft_ref)))

            if of_v:
                self.graph.add((node, GEOM_REL.of, self._owned_uri(of_v, qty)))
            if wrt_v:
                self.graph.add((node, GEOM_REL["with-respect-to"], self._owned_uri(wrt_v, qty)))
            if rp_v:
                ref_node = self._owned_uri(rp_v, qty)
                ref_predicate = (
                    RBDYN_ENT["reference-point"]
                    if qty.type == WorldQuantityType.Wrench
                    else GEOM_REL["reference-point"]
                )
                self.graph.add((node, ref_predicate, ref_node))
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
                and mapped_subspace == "position"
                and axis is None
            ):
                self._register_pose_position_view(scalar_uri, quantity, owner)
            elif (
                quantity.type == WorldQuantityType.Pose
                and mapped_subspace == "orientation"
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
        props = quantity.props if isinstance(quantity.props, GeometricProps) else None
        of_frame = _geo_prop(props, "of") if props is not None else None
        wrt_frame = _geo_prop(props, "wrt") if props is not None else None
        if of_frame is None or wrt_frame is None:
            return
        self._emitted_position_coords.add(scalar_uri)
        as_seen_by = _geo_prop(props, "as-seen-by") or wrt_frame
        self.graph.add((scalar_uri, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((scalar_uri, RDF.type, GEOM_REL.Position))
        self.graph.add((scalar_uri, RDF.type, GEOM_COORD.PositionCoordinate))
        self.graph.add((scalar_uri, RDF.type, GEOM_COORD.VectorXYZ))
        self.graph.add((scalar_uri, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Position))
        self.graph.add((scalar_uri, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        self.graph.add((scalar_uri, GEOM_REL.of, self._owned_uri(of_frame, quantity)))
        self.graph.add(
            (
                scalar_uri,
                GEOM_REL["with-respect-to"],
                self._owned_uri(wrt_frame, quantity),
            )
        )
        self.graph.add(
            (scalar_uri, GEOM_COORD["as-seen-by"], self._owned_uri(as_seen_by, quantity))
        )

        view_uri = self._owned_uri(f"view-{_scalar_id(quantity, 'position', None)}", owner)
        if view_uri not in self._emitted_views:
            self._emit_view(view_uri)
            self.graph.add((view_uri, RDF.type, MAP_EXT.PoseCoordinateView))
            self.graph.add((view_uri, MAP.superobject, URIRef(quantity.uri)))
            self.graph.add((view_uri, MAP.subobject, scalar_uri))
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
        if view_type == MAP_EXT.PoseCoordinateView:
            self._retag_as_position_kind(scalar_uri)
        self._emit_view(view_uri)
        self.graph.add((view_uri, MAP.superobject, URIRef(quantity.uri)))
        self.graph.add((view_uri, MAP.subobject, scalar_uri))
        subspace = MAP_EXT.orientation if view_subspace_uri == "rotation" else MAP_EXT.position
        self.graph.add((view_uri, MAP.subspace, subspace))
        self.graph.add((view_uri, MAP.axis, MAP[axis]))

    def _register_pose_orientation_view(
        self,
        scalar_uri: URIRef,
        quantity: "WorldQuantity",
        owner: Any,
    ) -> None:
        """Promote `<pose>.orientation` and register its coordinate view."""
        if scalar_uri in self._emitted_orientation_coords:
            return
        props = quantity.props if isinstance(quantity.props, GeometricProps) else None
        of_frame = _geo_prop(props, "of") if props is not None else None
        wrt_frame = _geo_prop(props, "wrt") if props is not None else None
        if of_frame is None or wrt_frame is None:
            return
        self._emitted_orientation_coords.add(scalar_uri)
        as_seen_by = _geo_prop(props, "as-seen-by") or wrt_frame
        self.graph.add((scalar_uri, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((scalar_uri, RDF.type, GEOM_REL.Orientation))
        self.graph.add((scalar_uri, RDF.type, GEOM_COORD.OrientationCoordinate))
        self.graph.add((scalar_uri, RDF.type, GEOM_COORD["EulerAngles"]))
        self.graph.add((scalar_uri, GEOM_COORD["axes-sequence"], Literal("xyz")))
        self.graph.add((scalar_uri, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.PlaneAngle))
        self.graph.add((scalar_uri, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
        self.graph.add((scalar_uri, GEOM_REL.of, self._owned_uri(of_frame, quantity)))
        self.graph.add(
            (scalar_uri, GEOM_REL["with-respect-to"], self._owned_uri(wrt_frame, quantity))
        )
        self.graph.add(
            (scalar_uri, GEOM_COORD["as-seen-by"], self._owned_uri(as_seen_by, quantity))
        )
        self.graph.add((URIRef(quantity.uri), GEOM_COORD["has-coordinate"], scalar_uri))

        view_uri = self._owned_uri(f"view-{_scalar_id(quantity, 'orientation', None)}", owner)
        if view_uri not in self._emitted_views:
            self._emit_view(view_uri)
            self.graph.add((view_uri, RDF.type, MAP_EXT.PoseCoordinateView))
            self.graph.add((view_uri, MAP.superobject, URIRef(quantity.uri)))
            self.graph.add((view_uri, MAP.subobject, scalar_uri))
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

        if quantity.type == QuantityType.Pose:
            self._emit_declared_pose_frame_metadata(node, quantity)
            return

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

        if quantity.type == QuantityType.Wrench:
            rp_v = _geo_prop(props, "ref-point")
            asb_v = _geo_prop(props, "as-seen-by")
            if rp_v:
                point_node = self._owned_uri(rp_v, owner)
                self.graph.add((point_node, RDF.type, GEOM_ENT.Point))
                self.graph.add((node, RBDYN_ENT["reference-point"], point_node))
            else:
                point_node = self._owned_uri(f"point-{quantity.name}-origin", quantity)
                self.graph.add((point_node, RDF.type, GEOM_ENT.Point))
                self.graph.add((node, RBDYN_ENT["reference-point"], point_node))
            if asb_v:
                self.graph.add((node, RBDYN_COORD["as-seen-by"], self._owned_uri(asb_v, owner)))

    def _context_ref_view_spec(
        self,
        quantity: ContextQuantity,
        subspace_raw: str,
        axis: str | None,
    ) -> tuple[Any, Any, Any] | None:
        """The (scalar type, view type, view subspace) for a context pose/trajectory reference's
        subspace+axis, or None when that subspace exposes no view.
        """
        if quantity.type in {QuantityType.Pose, ReferenceGeneratorType.Trajectory}:
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
        pose/trajectory reference, and return the value node.
        """
        view_spec = self._context_ref_view_spec(quantity, subspace_raw, axis)
        if view_spec is None:
            return URIRef(quantity.uri)
        scalar_type, view_type, view_subspace = view_spec
        suffix = f"{quantity.name}.{subspace_raw}" + (f".{axis}" if axis is not None else "")
        node = self._owned_uri(suffix, quantity)
        self._add_quantity(node, scalar_type)
        super_node = (
            self._reference_output_node(quantity)
            if quantity.type == ReferenceGeneratorType.Trajectory
            else URIRef(quantity.uri)
        )
        if (
            quantity.type in {QuantityType.Pose, ReferenceGeneratorType.Trajectory}
            and subspace_raw == "position"
        ):
            self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
            if axis is None:
                self.graph.add((node, RDF.type, GEOM_REL.Position))
                self.graph.add((node, RDF.type, GEOM_COORD.PositionCoordinate))
                self.graph.add((super_node, GEOM_COORD["has-coordinate"], node))
                coords = self._frame_coords(super_node)
                if coords is not None:
                    pose_of, pose_wrt, pose_asb = coords
                    self.graph.add((node, GEOM_REL.of, pose_of))
                    self.graph.add((node, GEOM_REL["with-respect-to"], pose_wrt))
                    self.graph.add((node, GEOM_COORD["as-seen-by"], pose_asb or pose_wrt))
            else:
                self._retag_as_position_kind(node)
        elif (
            quantity.type in {QuantityType.Pose, ReferenceGeneratorType.Trajectory}
            and subspace_raw == "orientation"
            and axis is None
        ):
            self.graph.add((node, RDF.type, GEOM_REL.Orientation))
            self.graph.add((node, RDF.type, GEOM_COORD.OrientationCoordinate))
            self.graph.add((node, RDF.type, GEOM_COORD["EulerAngles"]))
            self.graph.add((node, GEOM_COORD["axes-sequence"], Literal("xyz")))
            self.graph.add((super_node, GEOM_COORD["has-coordinate"], node))
            coords = self._frame_coords(super_node)
            if coords is not None:
                pose_of, pose_wrt, pose_asb = coords
                self.graph.add((node, GEOM_REL.of, pose_of))
                self.graph.add((node, GEOM_REL["with-respect-to"], pose_wrt))
                self.graph.add((node, GEOM_COORD["as-seen-by"], pose_asb or pose_wrt))

        view_node = self._owned_uri(f"view-{suffix}", quantity)
        if view_node not in self._emitted_views:
            self._emit_view(view_node)
            if axis is None or quantity.type not in {
                QuantityType.Pose,
                ReferenceGeneratorType.Trajectory,
            }:
                self.graph.add((view_node, RDF.type, view_type))
            self.graph.add((view_node, MAP.superobject, super_node))
            self.graph.add((view_node, MAP.subobject, node))
            self.graph.add((view_node, MAP.subspace, view_subspace))
            if axis is not None:
                self.graph.add((view_node, MAP.axis, MAP[axis]))
        return node

    def _emit_context_quantities(
        self,
        context_quantities: dict[str, ContextQuantity],
        constraints: list[ConstraintSpecification],
        world_qtys: dict[str, WorldQuantity],
    ) -> None:
        """Emit every context quantity. Trajectories, directions, profiles, admittance and pose
        values dispatch to their own emitters; the rest get quantity-kind typing, composite
        metadata and their value (reference, snapshot with optional offset, measure or vector).
        """
        for quantity in context_quantities.values():
            node = URIRef(quantity.uri)
            if quantity.type == ReferenceGeneratorType.Trajectory:
                self._emit_trajectory_quantity(quantity, constraints, world_qtys)
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
            if isinstance(quantity.value, PoseValue):
                self._emit_pose_value_quantity(node, quantity, constraints, world_qtys)
                continue
            qkind = QUDT_KIND_BY_QUANTITY_TYPE.get(quantity.type)
            if qkind is None:
                qkind = QUDT_QKIND[quantity.type]
            self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
            self._emit_quantity_kind(node, qkind)
            if quantity.type == QuantityType.Orientation:
                self.graph.add((node, RDF.type, GEOM_REL.Orientation))
                self.graph.add((node, RDF.type, GEOM_COORD.OrientationCoordinate))
                self.graph.add((node, RDF.type, GEOM_COORD["EulerAngles"]))
                self.graph.add((node, GEOM_COORD["axes-sequence"], Literal("xyz")))
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
                    self.graph.add((snapshot_node, _ns_term(ALGO_EXT, "trigger"), URIRef(trigger.uri)))
                self.graph.add(
                    (node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(quantity.type, QUDT_UNIT.UNITLESS))
                )
                if qkind == GEOM_REL.Pose:
                    self._emit_declared_pose_frame_metadata(node, quantity)
                elif quantity.type == QuantityType.Position:
                    self._emit_snapshot_position_metadata(node, quantity)
                continue
            self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(quantity.value.unit)))
            if isinstance(quantity.value, Measure):
                self.graph.add(
                    (
                        node,
                        QUDT_SCHEMA.value,
                        Literal(float(quantity.value.value), datatype=XSD.double),
                    )
                )
            elif isinstance(quantity.value, VectorQuantity):
                self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
                self.graph.add(
                    (node, GEOM_COORD.x, Literal(float(quantity.value.x), datatype=XSD.double))
                )
                self.graph.add(
                    (node, GEOM_COORD.y, Literal(float(quantity.value.y), datatype=XSD.double))
                )
                self.graph.add(
                    (node, GEOM_COORD.z, Literal(float(quantity.value.z), datatype=XSD.double))
                )

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

    def _emit_pose_value_quantity(
        self,
        node: URIRef,
        quantity: ContextQuantity,
        constraints: list[ConstraintSpecification] | None = None,
        world_qtys: dict[str, WorldQuantity] | None = None,
    ) -> None:
        """Emit a literal pose value: its pose coordinate, per-axis position/orientation
        component nodes with map:Views, and each component's reference or literal value.
        """
        assert isinstance(quantity.value, PoseValue)
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((node, RDF.type, GEOM_REL.Pose))
        self.graph.add((node, RDF.type, GEOM_COORD.PoseCoordinate))
        self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.PlaneAngle))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], URI_QUDT_QK_LENGTH))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.UNITLESS))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        self._emit_declared_pose_frame_metadata(node, quantity)

        position_node = self._owned_uri(f"{quantity.name}.position", quantity)
        orientation_node = self._owned_uri(f"{quantity.name}.orientation", quantity)
        self.graph.add((position_node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((position_node, RDF.type, GEOM_COORD.VectorXYZ))
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

        coords = self._frame_coords(node)
        if coords is not None:
            pose_of, pose_wrt, pose_asb = coords
            self.graph.add((orientation_node, GEOM_REL.of, pose_of))
            self.graph.add((orientation_node, GEOM_REL["with-respect-to"], pose_wrt))
            self.graph.add((orientation_node, GEOM_COORD["as-seen-by"], pose_asb or pose_wrt))

        for term in quantity.value.position.terms:
            component_node = self._owned_uri(f"{quantity.name}.position.{term.axis}", quantity)
            view_node = self._owned_uri(f"view-{quantity.name}.position.{term.axis}", quantity)
            self._add_quantity(component_node, QuantityType.Distance)
            self._retag_as_position_kind(component_node)
            self.graph.add((position_node, GEOM_COORD["has-coordinate"], component_node))
            self._emit_view(view_node)
            self.graph.add((view_node, MAP.superobject, node))
            self.graph.add((view_node, MAP.subobject, component_node))
            self.graph.add((view_node, MAP.subspace, MAP_EXT.position))
            self.graph.add((view_node, MAP.axis, MAP[term.axis]))
            if term.ref is not None:
                ref_node = self._emit_context_ref_node(term.ref, quantity, term.axis)
                self.graph.add((component_node, CSTR["reference-value"], ref_node))
            else:
                self.graph.add(
                    (
                        component_node,
                        QUDT_SCHEMA.value,
                        Literal(float(term.value), datatype=XSD.double),
                    )
                )
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
            self._emit_view(view_node)
            self.graph.add((view_node, MAP.superobject, node))
            self.graph.add((view_node, MAP.subobject, component_node))
            self.graph.add((view_node, MAP.subspace, MAP_EXT.orientation))
            self.graph.add((view_node, MAP.axis, MAP[axis]))
            if term.ref is not None:
                ref_node = self._emit_context_ref_node(term.ref, quantity, term.axis)
                self.graph.add((component_node, CSTR["reference-value"], ref_node))
            else:
                self.graph.add(
                    (
                        component_node,
                        QUDT_SCHEMA.value,
                        Literal(float(term.value), datatype=XSD.double),
                    )
                )
                self.graph.add((component_node, QUDT_SCHEMA.unit, _dsl_unit(term.unit)))

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
                f"Lerp trajectory start type '{getattr(start_qty, 'type', None)}' "
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
        quantity: ContextQuantity,
        constraints: list[ConstraintSpecification] | None = None,
        world_qtys: dict[str, WorldQuantity] | None = None,
    ) -> None:
        """Emit a geometric path and the evaluator that traverses it, dispatching by shape
        (lerp / circle / arc / helix / figure-8) to the matching emitter.
        """
        assert isinstance(quantity.value, TrajectoryValue)
        value = quantity.value
        if value.lerp is not None:
            self._emit_lerp_trajectory(quantity, value.lerp, constraints, world_qtys)
        elif value.circle is not None:
            self._emit_geometric_trajectory(
                quantity, GEOM_PATH.Circle, "circle", value.circle.alpha,
                [
                    ("start", GEOM_PATH.start, value.circle.start),
                    ("center", GEOM_PATH.center, value.circle.center),
                    ("plane-normal", GEOM_PATH["plane-normal"], value.circle.plane_normal),
                ],
                constraints,
                world_qtys,
            )
        elif value.arc is not None:
            self._emit_geometric_trajectory(
                quantity, GEOM_PATH.Arc, "arc", value.arc.alpha,
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
            self._emit_geometric_trajectory(
                quantity, GEOM_PATH.Helix, "helix", value.helix.alpha,
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
            self._emit_geometric_trajectory(
                quantity, GEOM_PATH.Figure8, "figure8", value.figure8.alpha,
                [
                    ("anchor", GEOM_PATH.anchor, value.figure8.anchor),
                    ("radius", GEOM_PATH.radius, value.figure8.radius),
                    ("plane-normal", GEOM_PATH["plane-normal"], value.figure8.plane_normal),
                ],
                constraints,
                world_qtys,
                path_terms=[
                    (GEOM_PATH.form, _ns_term(GEOM_PATH, value.figure8.form or "gerono"))
                ],
            )
        else:
            raise ValueError(f"TrajectoryValue on '{quantity.name}' has no populated spec")

    def _emit_trajectory_pose_metadata(
        self,
        quantity: ContextQuantity,
        value_kind: Any | None,
        constraints: list[ConstraintSpecification] | None,
        world_qtys: dict[str, WorldQuantity] | None,
    ) -> None:
        """Emit the setpoint pose the path evaluator produces, with its frame metadata."""
        node = self._reference_output_node(quantity)
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        if value_kind is not None:
            self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], value_kind))
        if value_kind == GEOM_REL.Pose:
            self.graph.add((node, RDF.type, GEOM_REL.Pose))
            self.graph.add((node, RDF.type, GEOM_COORD.PoseCoordinate))
            self.graph.add((node, RDF.type, GEOM_COORD.DirectionCosineXYZ))
            self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
            self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.PlaneAngle))
            self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], URI_QUDT_QK_LENGTH))
            self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.UNITLESS))
            self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        self._emit_declared_pose_frame_metadata(node, quantity)

    def _emit_lerp_trajectory(
        self,
        quantity: ContextQuantity,
        lerp: Any,
        constraints: list[ConstraintSpecification] | None,
        world_qtys: dict[str, WorldQuantity] | None,
    ) -> None:
        """Emit a linear path (start/goal) and the evaluator that eases along it."""
        lerp_node = self._owned_uri(f"lerp-{quantity.name}", quantity)
        value_kind = self._lerp_value_kind(lerp)
        self._emit_trajectory_pose_metadata(quantity, value_kind, constraints, world_qtys)
        self.graph.add((lerp_node, RDF.type, GEOM_PATH.Path))
        self.graph.add((lerp_node, RDF.type, GEOM_PATH.LinearPath))
        self.graph.add(
            (lerp_node, GEOM_PATH.start, self._emit_context_ref_node(lerp.start, quantity, "start"))
        )
        self.graph.add(
            (lerp_node, GEOM_PATH.goal, self._emit_context_ref_node(lerp.goal, quantity, "goal"))
        )
        self._emit_path_evaluator(
            quantity,
            lerp_node,
            lerp.alpha,
            "lerp",
            _ns_term(GEOM_OP_EXT, lerp.profile or "ease-in-out"),
        )

    def _emit_geometric_trajectory(
        self,
        quantity: ContextQuantity,
        path_type: URIRef,
        spec_prefix: str,
        alpha: Any,
        inputs: list[tuple[str, URIRef, Any]],
        constraints: list[ConstraintSpecification] | None,
        world_qtys: dict[str, WorldQuantity] | None,
        path_terms: list[tuple[URIRef, Any]] = (),
    ) -> None:
        """Emit the path as geometry and a PathEvaluator that traverses it. The path carries no
        parameter and no output; both belong to the evaluator, which produces the setpoint.
        """
        self._emit_trajectory_pose_metadata(quantity, GEOM_REL.Pose, constraints, world_qtys)
        path_node = self._owned_uri(f"{spec_prefix}-{quantity.name}", quantity)
        self.graph.add((path_node, RDF.type, GEOM_PATH.Path))
        self.graph.add((path_node, RDF.type, path_type))
        for suffix, predicate, ref in inputs:
            self.graph.add(
                (path_node, predicate, self._emit_context_ref_node(ref, quantity, suffix))
            )
        for predicate, term in path_terms:
            self.graph.add((path_node, predicate, term))
        self._emit_path_evaluator(quantity, path_node, alpha, spec_prefix)

    def _emit_path_evaluator(
        self,
        quantity: ContextQuantity,
        path_node: URIRef,
        alpha: Any,
        spec_prefix: str,
        easing: URIRef | None = None,
    ) -> None:
        """Emit the operator that turns a position along a path into the pose setpoint.

        The evaluator *is* the declaration's reference generator: it owns the path parameter
        and produces the setpoint pose, so the declared quantity needs no node of its own.
        """
        eval_node = self._owned_uri(f"{spec_prefix}-eval-{quantity.name}", quantity)
        self.graph.add((eval_node, RDF.type, GEOM_OP_EXT.PathEvaluator))
        self.graph.add((eval_node, RDF.type, CSTR_HDL_EXT.SetpointGenerator))
        self.graph.add((eval_node, GEOM_OP_EXT.path, path_node))
        self.graph.add(
            (
                eval_node,
                _ns_term(GEOM_OP_EXT, "path-parameter"),
                self._emit_context_ref_node(alpha, quantity, "path-parameter"),
            )
        )
        if easing is not None:
            self.graph.add((eval_node, GEOM_OP_EXT.easing, easing))
        self.graph.add((eval_node, GEOM_OP.out, self._reference_output_node(quantity)))

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
            if quantity.type == ReferenceGeneratorType.Trajectory:
                return self._reference_output_node(quantity)
            return URIRef(quantity.uri)

        node = self._owned_uri(f"{quantity.name}-{suffix}", owner)
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
                    Literal(float(ref.literal_value.value), datatype=XSD.double),
                )
            )
        elif isinstance(ref.literal_value, VectorQuantity):
            self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
            self.graph.add(
                (node, GEOM_COORD.x, Literal(float(ref.literal_value.x), datatype=XSD.double))
            )
            self.graph.add(
                (node, GEOM_COORD.y, Literal(float(ref.literal_value.y), datatype=XSD.double))
            )
            self.graph.add(
                (node, GEOM_COORD.z, Literal(float(ref.literal_value.z), datatype=XSD.double))
            )
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
        for a whole-subspace (axis-less) pose/trajectory reference.
        """
        ref_node = self._emit_context_ref_node(ref, owner, suffix)
        quantity = _context_quantity(ref)
        if not isinstance(quantity, ContextQuantity) or axis is not None:
            return ref_node
        quantity = _resolved_context_quantity(quantity)
        if quantity.type == QuantityType.Pose:
            if subspace == "position":
                position_node = self._owned_uri(f"{quantity.name}.position", quantity)
                self.graph.add((position_node, RDF.type, QUDT_SCHEMA.Quantity))
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
                self.graph.add(
                    (orientation_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.PlaneAngle)
                )
                self.graph.add((orientation_node, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
                pose_node = URIRef(quantity.uri)
                self.graph.add((pose_node, GEOM_COORD["has-coordinate"], orientation_node))
                coords = self._frame_coords(pose_node)
                if coords is not None:
                    pose_of, pose_wrt, pose_asb = coords
                    self.graph.add((orientation_node, GEOM_REL.of, pose_of))
                    self.graph.add((orientation_node, GEOM_REL["with-respect-to"], pose_wrt))
                    self.graph.add(
                        (orientation_node, GEOM_COORD["as-seen-by"], pose_asb or pose_wrt)
                    )
                return orientation_node
        if quantity.type == ReferenceGeneratorType.Trajectory:
            if subspace == "position":
                self.graph.add((ref_node, RDF.type, QUDT_SCHEMA.Quantity))
                self.graph.add((ref_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.Position))
                self.graph.add((ref_node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
            elif subspace == "orientation":
                self.graph.add((ref_node, RDF.type, GEOM_REL.Orientation))
                self.graph.add((ref_node, RDF.type, GEOM_COORD.OrientationCoordinate))
                self.graph.add((ref_node, RDF.type, GEOM_COORD["EulerAngles"]))
                self.graph.add((ref_node, GEOM_COORD["axes-sequence"], Literal("xyz")))
                self.graph.add((ref_node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.PlaneAngle))
                self.graph.add((ref_node, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
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
                if axis is None and (
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
            _linear_velocity_mps(spec_val.max_velocity, spec_val.max_velocity_unit),
            QUDT_QKIND.LinearVelocity,
            QUDT_UNIT["M-PER-SEC"],
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

    def _emit_elapsed_constraint(
        self, node: URIRef, spec: ConstraintSpecification, motion: GuardedMotion
    ) -> None:
        """A timing constraint: a normal cstr:Constraint whose measured quantity is the
        motion-state elapsed time (filled at runtime from the world clock), compared
        against a Duration threshold. No kinematics — codegen reads the clock directly."""
        expr = spec.expr
        self.graph.add((node, RDF.type, CSTR.Constraint))

        qty_node = self._elapsed_quantity_node(spec, motion)
        self.graph.add((qty_node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((qty_node, QUDT_SCHEMA["hasQuantityKind"], NS_MM_QUDT_QTY["Time"]))
        self.graph.add((qty_node, QUDT_SCHEMA.unit, QUDT_UNIT["SEC"]))
        self.graph.add((node, CSTR.quantity, qty_node))

        if isinstance(expr, GreaterThanConstraint):
            self.graph.add((node, RDF.type, CSTR.UnilateralConstraint))
            self.graph.add((node, RDF.type, CSTR.GreaterThanConstraint))
            threshold = expr.threshold
        elif isinstance(expr, LessThanConstraint):
            self.graph.add((node, RDF.type, CSTR.UnilateralConstraint))
            self.graph.add((node, RDF.type, CSTR.LessThanConstraint))
            threshold = expr.threshold
        else:
            raise ValueError(
                f"Timing constraint '{spec.name}' must use 'greater than' or 'less than'."
            )
        thr_node = self._emit_duration_threshold_node(threshold, motion, f"{spec.name}-threshold")
        self.graph.add((node, CSTR.threshold, thr_node))

    def _emit_duration_threshold_node(self, ref: ContextRef, owner: Any, suffix: str) -> URIRef:
        """Resolve a timing threshold to a node: an inline Duration literal, or a declared
        Duration quantity.
        """
        bare = getattr(ref, "bare", None)
        if bare is not None:
            node = self._owned_uri(suffix, owner)
            self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
            self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], NS_MM_QUDT_QTY["Time"]))
            self.graph.add((node, QUDT_SCHEMA.unit, _time_unit(bare.unit)))
            self.graph.add(
                (node, QUDT_SCHEMA.value, Literal(float(bare.value), datatype=XSD.double))
            )
            return node
        quantity = _context_quantity(ref)
        if isinstance(quantity, ContextQuantity):
            return URIRef(_resolved_context_quantity(quantity).uri)
        raise ValueError(
            "Timing threshold must be a declared Duration quantity or an inline literal like `5.0 s`."
        )

    def _emit_motion_spec(self, motion: GuardedMotion) -> None:
        """Emit the guarded-motion node linking its when/while/until constraints (with disjunction
        nodes for `any` logic) and any trajectory.
        """
        motion_node = self._owned_uri(f"motion-{motion.name}", motion)
        self.graph.add((motion_node, RDF.type, MOT.GuardedMotion))
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
            if view_type == MAP_EXT.PoseCoordinateView:
                self._retag_as_position_kind(scalar_node)

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
            self.graph.add(
                (distance_node, GEOM_REL["between-entities"], URIRef(plan.start.uri))
            )
            self.graph.add((distance_node, GEOM_REL["between-entities"], URIRef(plan.end.uri)))

    def _emit_controller_base(self, ctrl_node: URIRef, ctrl: ControllerEntry) -> None:
        """Emit a controller's type and gains: PID (kp/ki/kd, optional decay), Impedance
        (stiffness/damping, optional integral gain), or feed-forward.
        """
        self.graph.add((ctrl_node, RDF.type, CSTR_HDL.Controller))
        if ctrl.command_type is not None:
            self.graph.add(
                (ctrl_node, APP["command-type"], Literal(ctrl.command_type.value))
            )
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
        self.graph.add((event_loop_node, RDF.type, EL.EventLoop))

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

            qty = self._resolve_constraint_quantity(spec, world_qtys)
            if qty is None:
                raise ValueError(
                    f"Controller '{ctrl.name}' constraint '{spec.name}' does not resolve to a world quantity."
                )
            subspace = _view_subspace(spec)
            axis_raw = spec.view.axis
            axis = semantic_axis_label(axis_raw)
            shared = spec in shared_spec_ids
            scalar_t = _scalar_type(qty, subspace, axis) if qty else subspace
            command = controller_command_record(ctrl)

            authored_ctrl_node = URIRef(ctrl.uri)
            self._emit_controller_base(authored_ctrl_node, ctrl)
            self.graph.add((authored_ctrl_node, APP.order, Literal(controller_order)))
            self.graph.add((authored_ctrl_node, CSTR_HDL.constraint, URIRef(spec.uri)))
            measured_derivative = getattr(ctrl.params, "measured_derivative", None)
            derivative_quantity = getattr(measured_derivative, "quantity", None)
            if isinstance(derivative_quantity, WorldQuantity):
                self.graph.add(
                    (
                        authored_ctrl_node,
                        CSTR_HDL["measured-velocity"],
                        URIRef(_resolved_world_quantity(derivative_quantity).uri),
                    )
                )
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
            if qty is not None:
                sid = _scalar_id(qty, subspace, axis)
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
                and isinstance(spec.expr, EqualityConstraint)
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
                    self.graph.add((mon_node, RDF.type, CSTR_HDL.EdgeTriggeredMonitor))
                    self.graph.add((mon_node, CSTR_HDL.event, signal_node))
                    self.graph.add((mon_node, CSTR_HDL["event-queue"], event_loop_node))
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
                            QUDT_UNIT.SEC,
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
                qty = self._resolve_constraint_quantity(spec, world_qtys)
                if qty is None:
                    raise ValueError(
                        f"Monitor '{mon.name}' constraint '{spec.name}' does not resolve to a world quantity."
                    )
                subspace = _view_subspace(spec)
                axis_raw = spec.view.axis
                axis = semantic_axis_label(axis_raw)
                scalar_t = _scalar_type(qty, subspace, axis)
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
                self.graph.add((mon_node, RDF.type, CSTR_HDL.EdgeTriggeredMonitor))
                self.graph.add((mon_node, CSTR_HDL.event, signal_node))
                self.graph.add((mon_node, CSTR_HDL["event-queue"], event_loop_node))
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
                        QUDT_UNIT.SEC,
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
        """Emit each of a handler's motion drivers/solvers with its algorithm and interfaces."""
        solvers = [_resolved_solver(s) for s in getattr(handler, "solvers", [])]
        multi = len(solvers) > 1

        for solver in solvers:
            if not getattr(solver, "algorithm", ""):
                continue

            driver_stem = (
                f"{solver.name}-{motion.name or handler.name}"
                if multi
                else (motion.name or handler.name)
            )

            driver_node = self._owned_uri(f"driver-{driver_stem}", handler)
            self.graph.add((driver_node, RDF.type, SLV.MotionDrivers))

            solver_node = self._solver_node(handler, motion, solver)
            robot_uri = getattr(solver.agent, "uri", None)
            if robot_uri:
                self.graph.add((solver_node, AGN["of-agent"], URIRef(robot_uri)))

            alg = solver.algorithm
            if alg == "CommandForwarding":
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

            self.graph.add((solver_node, RDF.type, SLV.SolverWithInputAndOutput))

            alg_node = (
                SLV.AccelerationConstrainedHybridDynamicsAlgorithm
                if alg == "ACHD"
                else SLV["RecursiveNewtonEulerAlgorithm"]
                if alg == "RNE"
                else SLV[alg]
            )
            self.graph.add((solver_node, SLV.solver, alg_node))

            gravity_value = getattr(solver, "gravity_value", None)
            if gravity_value is not None:
                gravity_ref = _context_quantity(getattr(gravity_value, "ref", None))
                if gravity_ref is not None:
                    gravity_value_node = URIRef(gravity_ref.uri)
                else:
                    gravity_vector = gravity_value.literal
                    gravity_value_node = self._owned_uri(f"gravity-value-{solver.name}", handler)
                    self._add_quantity(gravity_value_node, QuantityType.FreeVector)
                    self.graph.add((gravity_value_node, RDF.type, GEOM_COORD.VectorXYZ))
                    self.graph.add(
                        (
                            gravity_value_node,
                            GEOM_COORD.x,
                            Literal(float(gravity_vector.x), datatype=XSD.double),
                        )
                    )
                    self.graph.add(
                        (
                            gravity_value_node,
                            GEOM_COORD.y,
                            Literal(float(gravity_vector.y), datatype=XSD.double),
                        )
                    )
                    self.graph.add(
                        (
                            gravity_value_node,
                            GEOM_COORD.z,
                            Literal(float(gravity_vector.z), datatype=XSD.double),
                        )
                    )
                    self.graph.add(
                        (gravity_value_node, QUDT_SCHEMA.unit, _dsl_unit(gravity_vector.unit))
                    )
                self.graph.add((solver_node, SLV.gravity, gravity_value_node))

            solver_limits_by_target = {
                str(entry.target): entry.saturation
                for entry in getattr(getattr(solver, "limits", None), "entries", [])
            }
            limit_quantity_types = {
                "torque": QuantityType.Torque,
                "linear-acceleration": QuantityType.LinearAcceleration,
                "angular-acceleration": QuantityType.AngularAcceleration,
            }
            for target, quantity_type in limit_quantity_types.items():
                saturation = solver_limits_by_target.get(target)
                if saturation is None:
                    continue
                signal_name = (
                    f"torque-output-{solver.name}"
                    if target == "torque"
                    else f"limit-target-{solver.name}-{target}"
                )
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
            if qty is None:
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

    def _controller_solver(self, handler: ConstraintHandler, ctrl: ControllerEntry) -> Any:
        """The solver that runs `ctrl`: its explicit solver, the sole solver, or the one matching
        its command-forwarding vs dynamics role.
        """
        explicit = getattr(getattr(ctrl, "solver", None), "solver", None)
        if explicit is not None:
            return explicit
        solvers = [_resolved_solver(s) for s in getattr(handler, "solvers", [])]
        if len(solvers) == 1:
            return solvers[0]
        if ctrl.type == ControllerType.FeedForward:
            candidates = [
                solver for solver in solvers if str(solver.algorithm) == "CommandForwarding"
            ]
        else:
            candidates = [
                solver for solver in solvers if str(solver.algorithm) != "CommandForwarding"
            ]
        return candidates[0] if len(candidates) == 1 else None

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
