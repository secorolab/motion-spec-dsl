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

import math
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from rdf_utils.collection import add_literal_list_pred
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.geom_rel import PoseModel
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
    URI_GEOM_PRED_OF_POSE,
    URI_GEOM_PRED_OF_POSITION,
    URI_GEOM_PRED_ORIGIN,
    URI_GEOM_PRED_SEEN_BY,
    URI_GEOM_PRED_W,
    URI_GEOM_PRED_WRT,
    URI_GEOM_PRED_X,
    URI_GEOM_PRED_Y,
    URI_GEOM_PRED_Z,
    URI_GEOM_TYPE_ANGLES_ABG,
    URI_GEOM_TYPE_DIRECTION_COSINE_XYZ,
    URI_GEOM_TYPE_EULER_ANGLES,
    URI_GEOM_TYPE_EXTRINSIC,
    URI_GEOM_TYPE_FRAME,
    URI_GEOM_TYPE_INTRINSIC,
    URI_GEOM_TYPE_ORIENT_REF,
    URI_GEOM_TYPE_POINT,
    URI_GEOM_TYPE_POSE,
    URI_GEOM_TYPE_POSITION_REF,
    URI_GEOM_TYPE_QUATERNION,
    URI_GEOM_TYPE_VECTOR_XYZ,
    URI_QUDT_QK_LENGTH,
    URI_QUDT_QK_MASS,
    URI_TIME_PRED_AFTER_EVT,
    URI_TIME_PRED_OF_CONSTRAINT,
    URI_TIME_TYPE_AFTER_EVT,
    URI_TIME_TYPE_TC,
)
from rdf_utils.namespace import (
    NS_MM_GEOM_REL,
    NS_MM_QUDT_QTY,
)
from rdf_utils.namespace import (
    NS_MM_QUDT_UNIT as QUDT_UNIT,
)
from rdflib.graph import Dataset
from rdflib.namespace import PROV, RDF, RDFS, SDO, XSD, Namespace
from rdflib.term import BNode, Literal, URIRef
from scene_dsl.classes.distrib import DistributionRef
from scene_dsl.rdf.distrib import add_sampled_quantity
from textx import get_model
from textx.scoping import get_included_models

from motion_spec_dsl.classes.constraint_handler import (
    CommandForwardingSolver,
    ConstraintHandler,
    ControllerEntry,
    ControllerType,
    MobilePlatformSolver,
    SaturationSpec,
    UntilMonitorRef,
    WhenMonitorRef,
    _resolved_solver,
)
from motion_spec_dsl.classes.constraints import (
    BilateralConstraint,
    ConstraintGroup,
    ConstraintSpecification,
    EqualityConstraint,
    GoalStatusConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    OutsideConstraint,
    _flatten_constraint_items,
    _resolved_spec,
)
from motion_spec_dsl.classes.context import (
    GEOMETRIC_DISTANCE_OPS,
    GEOMETRIC_DISTANCE_RELATION,
    GEOMETRIC_PROJECTION_OPS,
    ConfigValue,
    ContextQuantity,
    ContextRef,
    DirectionBetween,
    GeometricPropKey,
    GeometricProps,
    GeoPropPair,
    Measure,
    QOpNode,
    QuantityType,
    ReferenceGeneratorType,
    ReferenceValue,
    SampledValue,
    SnapshotValue,
    VectorXYZ,
    WorldQuantity,
    WorldQuantityType,
    _geometric_operand_kind,
    _resolved_context_quantity,
    _resolved_world_quantity,
)
from motion_spec_dsl.classes.controller_semantics import (
    ANGULAR_SUBSPACES,
    SUBSPACE_ALIAS,
    _alignment_is_pointwise,
    controller_command_record,
    controller_solver,
)
from motion_spec_dsl.classes.controller_semantics import (
    axis_label as semantic_axis_label,
)
from motion_spec_dsl.classes.coordinates import (
    AccelerationTwistCoordinate,
    Coordinates,
    OrientationCoordinate,
    PoseCoordinate,
    VelocityTwistCoordinate,
    WrenchCoordinate,
)
from motion_spec_dsl.classes.dimensions import (
    DIMENSION_VECTOR,
    VECTOR_COMPONENT_TYPE,
)
from motion_spec_dsl.classes.dimensions import (
    infer as _infer_expr_type,
)
from motion_spec_dsl.classes.dimensions import (
    resolve_leaf as _resolve_expr_leaf_type,
)
from motion_spec_dsl.classes.dimensions import (
    same_scalar_dimension as _same_scalar_dimension,
)
from motion_spec_dsl.classes.motion_spec import (
    ContextDeclReference,
    ContextSpec,
    ExecutionContext,
    GuardedMotion,
    Model,
    PostContextDecl,
    PreContextDecl,
    SpecContextDecl,
    ToleranceDefaults,
    WorldContextDecl,
)
from motion_spec_dsl.classes.path import (
    AdmittanceSpec,
    PathValue,
    ProfileSpec,
)
from motion_spec_dsl.classes.ros import (
    Ros,
    RosActionServerDecl,
    RosSubscriptionDecl,
)
from motion_spec_dsl.rdf.common import (
    ANGLE_UNITS,
    _alignment_id,
    _AlignmentPlan,
    _angle_bound,
    _angle_unit,
    _axis_vector,
    _context_quantity,
    _DistancePlan,
    _dsl_unit,
    _evaluator_id,
    _geo_prop,
    _geo_prop_events,
    _geo_prop_value,
    _GeometricDistancePlan,
    _gradient_scalar_id,
    _is_alignment_view,
    _is_distance_view,
    _is_geometric_distance_view,
    _is_incident_angle_view,
    _is_plane_angle_view,
    _is_projection_view,
    _node_name,
    _ns_term,
    _resolved_constraint_items,
    _scalar_id,
    _scalar_type,
    _view_subspace,
)
from motion_spec_dsl.rdf.model import (
    _QKIND_PREFIXES,
    CONSTRAINT_TYPE_OVERRIDE,
    CONTEXT_COMPOSITE_WORLD_TYPE,
    CSTR_TYPE_NAME,
    GEOM_DOMAIN_SPLIT,
    GRAPH_BINDINGS,
    QUDT_KIND_BY_QUANTITY_TYPE,
    ROS,
    SCALAR_UNIT,
    WORLD_SPECS,
)
from motion_spec_dsl.rdf_parser.vocab import (
    AGN,
    ALGO_EXT,
    APP,
    CSTR,
    CSTR_EXT,
    CSTR_HDL,
    CSTR_HDL_EXT,
    EL,
    EXEC,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_EXT,
    GEOM_OP,
    GEOM_OP_EXT,
    GEOM_PATH,
    GEOM_REL,
    GEOM_REL_EXT,
    KC_STAT,
    MAP,
    MAP_EXT,
    MOT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    RBDYN_COORD,
    RBDYN_ENT,
    RBDYN_OP,
    RBDYN_OP_EXT,
    SENSORS,
    SIM,
    SLV,
    SLV_EXT,
    SOSA,
    SSN,
    TIME,
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


def _owns_pose_subobjects(value) -> bool:
    """Whether a pose quantity carries its own `.position`/`.orientation` coordinate nodes.

    True for a pose stated coordinate-wise -- authored literally or read from the deployment
    config. A snapshot or reference borrows its source's, so its subspaces hang off itself.
    """
    return isinstance(value, (PoseCoordinate, ConfigValue))


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


def _orientation_angle_unit(orientation) -> str:
    """The angle unit a pose's rotation is authored in; radians when it states none (a quaternion
    or a direction-cosine matrix carries angles all the same, just not as an authored number)."""
    for attr in ("euler", "angle_axis", "rotation"):
        spec = getattr(orientation, attr, None)
        unit = getattr(spec, "unit", None) if spec is not None else None
        if unit:
            return unit
    return getattr(orientation, "unit", None) or "rad"


def _qudt_kind(quantity_type: Any) -> URIRef:
    """QUDT quantity kind for a DSL quantity type."""
    return QUDT_KIND_BY_QUANTITY_TYPE.get(quantity_type) or QUDT_QKIND[quantity_type]


# A band bounds a scalar error in one unit, so one default per unit family: an axis of a
# position and a distance between two poses are both metres, an axis of an orientation and a
# joint angle both radians. The author states `position` and `orientation`; the rest fold in.
_TOLERANCE_DEFAULT_KIND: dict[Any, Any] = {
    QuantityType.Distance: QuantityType.Position,
    QuantityType.Angle: QuantityType.Orientation,
    QuantityType.PlaneAngle: QuantityType.Orientation,
}


def _tolerance_defaults(models) -> dict[Any, ContextRef]:
    """Model-wide satisfaction bands by quantity kind, from every `tolerances` block.

    Uniqueness and units are settled in validation, so this only indexes.
    """
    return {
        entry.kind: entry.band
        for model in models
        for spec in getattr(model, "specs", [])
        if isinstance(spec, ToleranceDefaults)
        for entry in spec.defaults
    }


_EXPRESSION_SUFFIX = {
    CSTR_EXT.ConstraintDisjunction: "disjunction",
    CSTR_EXT.ConstraintConjunction: "conjunction",
}


def _section_expression_type(logic, member_count: int):
    """The expression node type a when/until section's logic mints, or None when it stays flat.

    No keyword means the implicit default, so nothing is minted and the authored graph shape is
    kept. An `all` of one member states nothing beyond that member, so it stays flat too.
    """
    if not member_count:
        return None
    if logic == "any":
        return CSTR_EXT.ConstraintDisjunction
    if logic == "all" and member_count > 1:
        return CSTR_EXT.ConstraintConjunction
    return None


def _perturbation_conditions(handler: ConstraintHandler) -> list[ConstraintSpecification]:
    """Enabled, alias-resolved constraints the handler's perturbation gates hold.

    They are authored in the handler rather than in the motion, so every pass that walks a
    motion's constraints has to be told about them or their views reach the graph unemitted.
    """
    out = []
    for perturbation in getattr(handler, "perturbations", []) or []:
        for item in _flatten_constraint_items(perturbation.conditions):
            spec = _resolved_spec(item)
            if not isinstance(spec, GoalStatusConstraint) and not spec.disabled:
                out.append(spec)
    return out


_DEVICE_TARGETS = {"Agent", "KinematicTreeInstance", "ForceTorqueSensorSpec"}


def _authored_fqn(target) -> str:
    """The dotted name the model refers to this element by, e.g. `agents.arm1`."""
    parts, node = [], target
    while node is not None and getattr(node, "name", None):
        parts.append(node.name)
        node = getattr(node, "parent", None)
    # A sensor's chain runs up through the scene instance; the agent that hosts it is enough.
    return ".".join(reversed(parts[:2]))


# Resolved view subspaces whose command is a force, not a moment.
_LINEAR_SUBSPACES = frozenset({"position", "distance", "linear", "linear-velocity"})


def _validate_command_subspace(ctrl: ControllerEntry, spec, command) -> None:
    """Reject an authored `as` that contradicts its constraint's subspace: an angular
    subspace commands a moment, a linear one a force."""
    authored = ctrl.command_type
    if authored is None:
        return
    subspace = command.view_subspace
    if subspace in ANGULAR_SUBSPACES and authored == QuantityType.Force:
        raise ValueError(
            f"Controller '{ctrl.name}' commands a force on the angular subspace of "
            f"'{spec.name}'; an angular constraint commands a moment ('as torque')."
        )
    if subspace in _LINEAR_SUBSPACES and authored == QuantityType.Torque:
        raise ValueError(
            f"Controller '{ctrl.name}' commands a moment on the linear subspace of "
            f"'{spec.name}'; a linear constraint commands a force ('as force')."
        )


_COERCIBLE_DATATYPES = (XSD.double, XSD.integer)


def _numeric_term_coercions(graph, prefixes: dict[str, str]) -> dict[str, dict[str, str]]:
    """Declare xsd:double/xsd:integer on the JSON-LD terms that carry them.

    rdflib forces JSON-LD native types whenever a context is active
    (``Converter.use_native_types = context.active or use_native_types``), so numeric
    literals always serialize as bare JSON numbers with no datatype of their own. Coercing
    the term declares that datatype once, keeping it visible in the document and pinning it
    on re-parse -- otherwise a whole-valued double reads back as xsd:integer.

    A predicate carrying more than one numeric datatype has no single coercion and is left
    alone; its values stay bare, as they were before any term was declared.
    """
    longest_first = sorted(prefixes.items(), key=lambda item: -len(item[1]))

    def to_curie(iri: str) -> str | None:
        for prefix, namespace in longest_first:
            if iri.startswith(namespace):
                return f"{prefix}:{iri[len(namespace) :]}"
        return None

    datatypes: dict[URIRef, set[URIRef]] = {}
    for _, predicate, obj in graph:
        if isinstance(obj, Literal) and obj.datatype in _COERCIBLE_DATATYPES:
            datatypes.setdefault(predicate, set()).add(obj.datatype)

    terms: dict[str, dict[str, str]] = {}
    for predicate, found in datatypes.items():
        if len(found) > 1:
            continue
        term = to_curie(str(predicate))
        datatype = to_curie(str(next(iter(found))))
        if term is not None and datatype is not None:
            terms[term] = {"@id": str(predicate), "@type": datatype}
    return terms


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
        self._tolerance_defaults = _tolerance_defaults(self.models)

        # Resolution indexes, populated once and read during emission (see module docstring).
        self._distance_plans: dict[ConstraintSpecification, _DistancePlan] = {}
        self._alignment_plans: dict[ConstraintSpecification, _AlignmentPlan] = {}
        self._geometric_distance_plans: dict[ConstraintSpecification, _GeometricDistancePlan] = {}
        self._frame_coords_index: dict[URIRef, tuple[URIRef, URIRef, URIRef]] = {}
        # coord_node, domain -> the relation `_emit_geom_relation` named for it. hasQuantityKind
        # lives on the relation, not the (possibly domain-combined) coordinate, and position/
        # orientation relations now pool by frame pair, so neither is derivable from coord_node's
        # own URI any more; every reader looks the relation up here instead of reconstructing
        # the pre-pooling `f"{coord_node}-{domain}-rel"` string.
        self._component_relation_index: dict[tuple[URIRef, str], URIRef] = {}
        # coord_node, domain -> the view that pins that component of that coordinate. A pooled
        # relation is shared by every coordinate over the frame pair, so only the view can say
        # which operand a constraint edge means.
        self._component_view_index: dict[tuple[URIRef, str], URIRef] = {}
        self._config_resource: URIRef | None = None
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
        self._emitted_bands: set[URIRef] = set()
        self._emitted_distance_ops: set[str] = set()
        self._emitted_alignment_ops: set[str] = set()
        self._emitted_geometric_distance_ops: set[str] = set()
        self._linear_distance_relations: dict[tuple[str, str], URIRef] = {}
        self._angular_distance_relations: dict[tuple[str, str], URIRef] = {}
        self._emitted_views: set[URIRef] = set()
        self._emitted_position_coords: set[URIRef] = set()
        self._emitted_orientation_coords: set[URIRef] = set()
        self._path_projections: set[URIRef] = set()
        self._motion_time_endpoints_index: dict[str, tuple[URIRef, URIRef]] = {}

    def build(self) -> tuple[Dataset, dict[str, Any]]:
        """Emit the full dataset and return it with its JSON-LD namespace context.

        Emits context specs once, then for each authored handler+motion runs
        the ordered emission phases (structural entities, world/context quantities,
        transforms, constraints, motion spec, scalar views, map ops, handler, solvers).
        The returned dict maps namespace prefixes to their URIs for JSON-LD serialization.
        """
        handlers = self.authored_handlers

        shared_spec_ids = self._compute_shared_specs(handlers)

        context: dict[str, Any] = {}
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
                elif isinstance(spec, Ros):
                    for server in spec.servers:
                        self._emit_ros_action_server(server)
                    for subscription in spec.subscriptions:
                        self._emit_ros_subscription(subscription)
                    for standing in spec.always:
                        self._emit_ros_standing_pub(standing)
                    if spec.servers or spec.subscriptions or spec.always:
                        self.dataset.bind(spec.ns.name, spec.ns.uri)
                        context[spec.ns.name] = spec.ns.uri

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
            constraints = _resolved_constraint_items(motion) + _perturbation_conditions(handler)

            self._emit_world_quantities(world_qtys)
            self._emit_context_quantities(context_quantities, constraints, world_qtys)
            self._emit_path_following(constraints, world_qtys)
            self._emit_constraints(motion, constraints, world_qtys)
            self._emit_detect_acts(motion)
            self._emit_motion_spec(motion)
            self._emit_scalar_views(motion, constraints, world_qtys)
            self._emit_map_operations(motion, constraints, world_qtys)
            self._emit_constraint_handler(
                handler, motion, world_qtys, shared_spec_ids, handler_order
            )
            self._emit_solvers(handler, motion, world_qtys)

        context.update(_numeric_term_coercions(self.graph, context))
        return self.dataset, context

    def _emit_ros_action_server(self, server: RosActionServerDecl) -> None:
        """The action the runtime answers goals on: its one valueless member is the event an
        accepted goal produces. What a scenario observes while the run plays out, and what the
        goal is finally answered with, are authored on the monitors (`publish: event to`,
        `result: succeeded`), not here.
        """
        node = URIRef(server.uri)
        self.graph.add((node, RDF.type, ROS.Action))
        self.graph.add((node, ROS["channel-name"], Literal(server.channel_name)))
        self.graph.add((node, ROS["type-name"], Literal(server.type_name)))
        self.graph.add((node, RDFS.member, URIRef(server.goal_event.uri)))

    def _emit_ros_subscription(self, subscription: RosSubscriptionDecl) -> None:
        """A subscribed topic: the channel it arrives on, the message it carries, and what it
        informs the model about -- world poses, or the camera whose images it carries. The
        features of interest are what make it a channel the model reads rather than one it
        writes; a field path says where in a detection a pose sits, so an image carries none.
        """
        node = URIRef(subscription.uri)
        self.graph.add((node, RDF.type, ROS.Topic))
        self.graph.add((node, ROS["channel-name"], Literal(subscription.channel_name)))
        self.graph.add((node, ROS["type-name"], Literal(subscription.type_name)))
        if subscription.pose_path is not None:
            self.graph.add((node, ROS["field-path"], Literal(subscription.pose_path)))
        for target in (*subscription.targets, *subscription.cameras):
            self.graph.add(
                (
                    node,
                    SOSA.hasFeatureOfInterest,
                    URIRef(str(target.ref.uri)),
                )
            )

    def _emit_ros_standing_pub(self, standing: Any) -> None:
        """A declared topic published for the whole run, at the rate the model states.

        Held by the execution context rather than by a monitor: what it publishes is true of
        the run, so it is not scoped to any one motion the way a verdict is. Each entry is one
        quantity stated whole -- the message reports it, and only a field the model wants mapped
        differently is written out -- and the entity it is about, for a message that carries
        several and so has to say which is which.
        """
        node = URIRef(standing.uri)
        self.graph.add((node, RDF.type, ROS.Topic))
        self.graph.add((node, ROS["channel-name"], Literal(standing.topic.channel_name)))
        self.graph.add((node, ROS["type-name"], Literal(standing.topic.type_name)))
        for index, entry in enumerate(standing.entries):
            row = URIRef(f"{standing.uri}.e{index}")
            self.graph.add((node, RDFS.member, row))
            self.graph.add((row, RDF.value, URIRef(entry.quantity.uri)))
            if entry.subject is not None:
                self.graph.add((row, SOSA.hasFeatureOfInterest, URIRef(entry.subject.uri)))

        rate = URIRef(f"{standing.uri}.rate")
        self._emit_scalar_quantity(
            rate,
            float(standing.rate.value),
            NS_MM_QUDT_QTY["Frequency"],
            _dsl_unit(standing.rate.unit),
        )
        self.graph.add((node, SENSORS["update-rate"], rate))

        for index, assignment in enumerate(standing.fields):
            row = URIRef(f"{standing.uri}.f{index}")
            self.graph.add((node, RDFS.member, row))
            self.graph.add((row, ROS["field-path"], Literal(assignment.field_path)))
            self.graph.add((row, RDF.value, self._view_node(assignment, standing)))
        for context in self.model.specs:
            if isinstance(context, ExecutionContext):
                self.graph.add((URIRef(context.uri), RDFS.member, node))

    def _emit_execution_context(self, context: ExecutionContext) -> None:
        """Emit the authored scene, platform, and control-period binding."""
        node = URIRef(context.uri)
        self.graph.add((node, RDF.type, EXEC.ExecutionContext))
        # Arranging equipment for a purpose is what SSN calls a deployment, and that is what
        # an execution context is: these systems, on this platform, for this specification.
        self.graph.add((node, RDF.type, SSN.Deployment))
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
        # A simulator is software: it has a name. Real-world hardware is named per device, on
        # the system that realizes each element.
        if getattr(context.platform, "name", None):
            self.graph.add((node, SDO.name, Literal(context.platform.name)))
        self._emit_deployment(context, node)

    def _emit_deployment(self, context: ExecutionContext, node: URIRef) -> None:
        """Emit the systems this execution context deploys, and where their addresses live.

        Real-world and simulation deploy different things, and the asymmetry is the fact
        rather than an accident: on hardware each element has its own device, so each gets a
        system this context owns; in simulation one simulator answers for all of them, so
        there is nothing per-element to own and the scene's agents are named directly.
        """
        devices = getattr(context.platform, "devices", None) or ()
        real_world = context.platform.kind == "real-world"
        if devices and not real_world:
            raise ValueError(
                f"Execution context '{context.name}' binds devices on a simulation platform."
            )
        if devices and not context.config:
            raise ValueError(
                f"Execution context '{context.name}' binds devices but declares no 'config'. "
                "Every bound device needs somewhere to read its address from."
            )
        seen: dict[str, str] = {}
        for binding in devices:
            target = binding.target
            uri = getattr(target, "uri", None)
            if uri is None:
                raise ValueError(
                    f"Execution context '{context.name}' binds '{binding.device}' to "
                    f"'{getattr(target, 'name', target)}', which is not an addressable element."
                )
            if type(target).__name__ not in _DEVICE_TARGETS:
                raise ValueError(
                    f"Execution context '{context.name}' binds '{binding.device}' to a "
                    f"{type(target).__name__}. A device realizes an agent or a sensor."
                )
            if uri in seen:
                raise ValueError(
                    f"Execution context '{context.name}' binds '{target.name}' twice: "
                    f"'{seen[uri]}' and '{binding.device}'. One element, one device."
                )
            seen[uri] = binding.device

        if context.config:
            config = URIRef(f"{context.uri}.config")
            self.graph.add((config, RDF.type, EXEC.ResourceWithPath))
            self.graph.add((config, RDF.type, EXEC.SystemResource))
            # Authored relative to the model that names it, and only the DSL knows that
            # directory -- so the resource states where the file is, not where it was typed.
            declared_in = Path(get_model(context)._tx_filename).resolve()
            resolved = (declared_in.parent / context.config).resolve()
            self.graph.add((config, EXEC.path, Literal(str(resolved))))
            self.graph.add((node, EXEC["has-resource"], config))
            self._config_resource = config
        # The device is a system this deployment owns, naming the hardware it is and the
        # modelled element it realizes. The element belongs to the scene, so it is only ever
        # referred to -- nothing is asserted onto it.
        for binding in devices:
            fqn = _authored_fqn(binding.target)
            device = URIRef(f"{context.uri}.{fqn.replace('.', '-')}")
            self.graph.add((device, RDF.type, SSN.System))
            self.graph.add((device, SDO.model, Literal(binding.device)))
            self.graph.add((device, EXEC.realizes, URIRef(binding.target.uri)))
            self.graph.add((node, SSN["deployedSystem"], device))

        # A simulation has no hardware to stand in for anything: the simulator answers for
        # every agent, so the deployment names the agents the scene already declares. They
        # are systems in their own right, which is what lets this refer to them directly
        # instead of owning a node per element the way the real-world branch must.
        if not real_world:
            for modelled_agent in context.scene.modelled_agns:
                self.graph.add((node, SSN["deployedSystem"], URIRef(modelled_agent.agn.uri)))

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
        """Create a URI in the nearest namespace owned by owner or its parents.

        For a node named after something the namespace names -- a world quantity, a solver, a
        robot -- which is why one view, one gravity value and one gripper position serve every
        motion that asks for them. A node named after a context quantity takes `_declared_uri`
        instead: those names are scoped to the block they are written in.
        """
        if urlsplit(str(name)).scheme:
            return URIRef(name)
        ns_uri = str(self._namespace_owner(owner).ns.uri)
        return Namespace(ns_uri)[name]

    def _emit_qexpr(self, tree: Any, owner: Any, node: URIRef) -> URIRef:
        """Emit `tree` (a `QExpr`/`SnapshotValue` `as_op_tree()`) as a chain of ALGO ops whose
        root result lands at `node`; returns `node`. Interior op/result nodes below the root
        are suffixed by their DFS position off `node` -- deterministic and stable across
        regeneration, mirroring the flat node this replaces.
        """
        self._emit_qexpr_at(tree, owner, node, node, "")
        return node

    def _emit_qexpr_at(
        self, tree: Any, owner: Any, stem: URIRef, out_node: URIRef, path: str
    ) -> None:
        """Emit the op chain for `tree` landing its result at `out_node`. `stem` is the fixed
        per-declaration naming root; `path` is this node's DFS position under it.
        """
        op_node = URIRef(f"{stem}-{tree.op}-{path}" if path else f"{stem}-{tree.op}")
        operand_nodes = []
        for index, operand in enumerate(tree.operands):
            child_path = f"{path}-{index}" if path else str(index)
            if isinstance(operand, QOpNode):
                operand_out = URIRef(f"{stem}-{operand.op}-{child_path}-out")
                self._emit_qexpr_at(operand, owner, stem, operand_out, child_path)
                self._stamp_expr_result(operand_out, operand)
                operand_nodes.append(operand_out)
            else:
                operand_nodes.append(self._emit_qexpr_leaf(operand, owner, f"expr-{child_path}"))
        if tree.op == "subtract":
            self.graph.add((op_node, RDF.type, ALGO_EXT.Subtraction))
            self.graph.add((op_node, ALGO_EXT.minuend, operand_nodes[0]))
            self.graph.add((op_node, ALGO_EXT.subtrahend, operand_nodes[1]))
        elif tree.op == "divide":
            self.graph.add((op_node, RDF.type, ALGO_EXT.Division))
            self.graph.add((op_node, ALGO_EXT.dividend, operand_nodes[0]))
            self.graph.add((op_node, ALGO_EXT.divisor, operand_nodes[1]))
        else:
            self.graph.add(
                (
                    op_node,
                    RDF.type,
                    ALGO_EXT.Multiplication if tree.op == "multiply" else ALGO_EXT.Addition,
                )
            )
            for operand_node in operand_nodes:
                self.graph.add((op_node, _ns_term(ALGO_EXT, "in"), operand_node))
        self.graph.add((op_node, ALGO_EXT.out, out_node))

    def _emit_qexpr_leaf(self, leaf: Any, owner: Any, suffix: str) -> URIRef:
        """A quantity-expression leaf's value node: a world-quantity view resolves through
        `_view_node` (subspace/axis-aware); everything else (context ref, bare measure) through
        `_emit_context_ref_node`, typed by what the leaf itself infers to -- `owner` need not
        be a typed quantity (a motion, for a constraint's expression).
        """
        if isinstance(getattr(leaf, "quantity", None), WorldQuantity):
            return self._view_node(leaf, owner)
        return self._emit_context_ref_node(leaf, owner, suffix, _resolve_expr_leaf_type(leaf))

    def _stamp_expr_result(self, node: URIRef, tree: Any) -> None:
        """Type a compiler-generated interior op result by what it dimensionally infers to."""
        qty_type = _infer_expr_type(tree)
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        self._emit_quantity_kind(node, _qudt_kind(qty_type))
        self.graph.add((node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(qty_type, QUDT_UNIT.UNITLESS)))

    def _declared_uri(self, name: str, declaration: Any) -> URIRef:
        """Create a URI for a node named after `declaration`, under the declaration itself.

        A name written inside a motion or a handler -- a context quantity, a controller -- is
        scoped to that block, so two motions may each call their path `trajectory` and two
        handlers each call a controller `look-tangent`. Hanging what is generated from one under
        its own IRI keeps them apart; minting in the namespace root collapses them onto one node,
        which then collects both motions' inputs.
        """
        if urlsplit(str(name)).scheme:
            return URIRef(name)
        return URIRef(f"{declaration.uri}/{name}")

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

        A relation is a fact about a frame pair, not a coordinate: every coordinate over the
        same (domain, of, wrt) shares one relation node, named from the pair rather than from
        whichever coordinate happened to emit it first. A quantity missing `of` or `wrt` cannot
        be pooled this way and keeps a node of its own.
        """
        ref_type, coord_type, of_pred, rel_type = GEOM_DOMAIN_SPLIT[domain]
        rel_node = (
            URIRef(f"{of_node}-{wrt_node}-{domain}-rel")
            if of_node is not None and wrt_node is not None
            else URIRef(f"{coord_node}-{domain}-rel")
        )
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
        self._component_relation_index[(coord_node, domain)] = rel_node
        return rel_node

    def _component_relation(self, coord_node: URIRef, domain: str) -> URIRef:
        """The relation `_emit_geom_relation` named for `coord_node`'s `domain` component --
        see `_component_relation_index`."""
        return self._component_relation_index[(coord_node, domain)]

    def _register_component_view(self, coord_node: URIRef, domain: str, view_node: URIRef) -> None:
        """Record the view a constraint operand names for this coordinate's component. First
        emitter wins: several motions may each view the same coordinate, all equivalently."""
        self._component_view_index.setdefault((coord_node, domain), view_node)

    def _component_view(self, coord_node: URIRef, domain: str) -> URIRef:
        """The view pinning `coord_node`'s `domain` component: `map:superobject` the coordinate,
        `map:subobject` the pooled relation. A constraint operand names this, never the relation
        -- that one is shared by every coordinate over the same frame pair."""
        view_node = self._component_view_index.get((coord_node, domain))
        if view_node is None:
            raise ConstraintViolation(
                "geometry",
                f"Coordinate '{coord_node}' has no {domain} view, so nothing can name it as a "
                "constraint operand.",
            )
        return view_node

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
        """Collect the WorldQuantities a view references (quantity, and the binary view's
        left/right operands, when the operand in question is a pose) into `qtys`. An
        angle-between or line/plane operand resolves to a ContextQuantity and is a no-op here.
        """
        if view is None:
            return
        self._add_world_quantity(qtys, getattr(view, "quantity", None))
        binary = getattr(view, "binary", None)
        if binary is not None:
            self._add_world_quantity(qtys, getattr(binary, "left", None))
            self._add_world_quantity(qtys, getattr(binary, "right", None))

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
        for constraint in _resolved_constraint_items(motion) + _perturbation_conditions(handler):
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
        for constraint in _resolved_constraint_items(motion) + _perturbation_conditions(handler):
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
        if _is_alignment_view(spec):
            return self._alignment_plan(spec, world_qtys).target
        if _is_geometric_distance_view(spec) or _is_projection_view(spec):
            return self._geometric_distance_plan(spec, world_qtys).target
        if _is_incident_angle_view(spec):
            return self._incident_angle_plan(spec, world_qtys).target
        if _is_plane_angle_view(spec):
            return self._plane_angle_plan(spec, world_qtys).target
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

    def _distance_operand(
        self, ref: Any, world_qtys: dict[str, WorldQuantity]
    ) -> WorldQuantity | ContextQuantity | None:
        """A distance endpoint: a world pose, or a context pose (a snapshot) compared by value."""
        qty = self._resolve_qty(ref, world_qtys)
        if qty is not None:
            return qty
        return ref if isinstance(ref, ContextQuantity) else None

    def _distance_endpoint_frame(self, qty: Any, context: str) -> str:
        """The `of` frame of a distance endpoint; raises when it is not a framed pose."""
        if isinstance(qty, WorldQuantity):
            return self._pose_frames(qty, context)[0]
        if qty.type != QuantityType.Pose:
            raise ValueError(f"{context} must reference Pose quantities.")
        frames = _pose_frame_names(qty)
        if frames is None:
            raise ValueError(f"{context} needs explicit 'of' and 'wrt' frames.")
        return frames[0]

    def _distance_plan(
        self,
        spec: ConstraintSpecification,
        world_qtys: dict[str, WorldQuantity],
    ) -> _DistancePlan:
        """Resolve an authored distance relation's pose endpoints and scalar-view carrier."""
        cached = self._distance_plans.get(spec)
        if cached is not None:
            return cached

        start = self._distance_operand(spec.view.binary.left, world_qtys)
        end = self._distance_operand(spec.view.binary.right, world_qtys)
        if start is None or end is None:
            raise ValueError(f"Distance constraint '{spec.name}' references an unknown pose.")
        context = f"Distance constraint '{spec.name}'"
        start_frame = self._distance_endpoint_frame(start, context)
        end_frame = self._distance_endpoint_frame(end, context)
        props = GeometricProps(
            [
                GeoPropPair(GeometricPropKey.Of, end_frame),
                GeoPropPair(GeometricPropKey.Wrt, start_frame),
                GeoPropPair(GeometricPropKey.AsSeenBy, start_frame),
            ]
        )
        # The motion qualifies the carrier because an endpoint may be motion-local: two motions
        # can name the same constraint over their own snapshots, and one carrier for both would
        # measure the first motion's snapshot from inside the second.
        motion = getattr(getattr(spec, "parent", None), "parent", None)
        target = WorldQuantity(
            parent=motion,
            name=f"distance-{getattr(motion, 'name', '')}-{spec.name}",
            type=WorldQuantityType.Pose,
            props=props,
        )
        relation_a = str(self._distance_endpoint_point(start))
        relation_b = str(self._distance_endpoint_point(end))
        plan = _DistancePlan(start, end, target, relation_a, relation_b)
        self._distance_plans[spec] = plan
        return plan

    def _distance_endpoint_point(self, operand: WorldQuantity | ContextQuantity) -> URIRef:
        """The Point a `distance between` operand's pose value measures: its `of` frame's
        origin (`_frame_origin` already materializes one per frame -- no minted stand-in).

        Frame-scoped, not operand-scoped: a live pose and a snapshot of that same pose share a
        frame, and now share this Point too. What still tells the two apart -- which pose
        coordinate each endpoint names -- is carried separately, as a coord_policy selection
        recorded as PROV (`_emit_distance_operand_selection`), not by tagging the Point.
        """
        of_frame = self._distance_endpoint_frame(operand, f"Distance operand '{operand.name}'")
        return self._frame_origin(self._owned_uri(of_frame, operand))

    def _record_pose_component(self, pose_node: URIRef, component: str, coordinate: URIRef) -> None:
        """Record which position/orientation coordinate a pose coordinate is built from.

        A pooled Position/Orientation relation is shared by every pose over the same frames, so
        the relation alone cannot say which coordinate belongs to this pose. The model does --
        the DSL emits both -- so it is recorded here rather than re-derived downstream by name.
        """
        relation = self.graph.value(coordinate, GEOM_COORD[f"of-{component}"])
        if relation is None:
            return
        candidates = sorted(self.graph.subjects(GEOM_COORD[f"of-{component}"], relation), key=str)
        self._record_coord_selection(
            URIRef(f"{pose_node}-{component}-selection"),
            relation,
            candidates,
            coordinate,
            self._owned_uri("coord-policy/pose-component", None),
            "authored pose component",
        )

    def _record_coord_selection(
        self,
        activity: URIRef,
        relation: URIRef,
        candidates,
        chosen: URIRef,
        policy: URIRef,
        policy_label: str,
    ) -> None:
        """Record a coordinate choice as PROV: ``activity`` used ``relation`` and every
        candidate, ran as the ``policy`` agent, and the candidate it settled on is its
        qualified usage. Nothing is ``prov:wasGeneratedBy`` -- a selection creates no
        coordinate. Idempotent: re-recording the same selection leaves the graph as it was.
        Lives here until rdf-utils grows a provenance home for it (see the tracking issue).
        """
        graph = self.graph
        graph.add((activity, RDF.type, PROV.Activity))
        graph.add((relation, RDF.type, PROV.Entity))
        graph.add((activity, PROV.used, relation))
        for candidate in candidates:
            graph.add((candidate, RDF.type, PROV.Entity))
            graph.add((activity, PROV.used, candidate))
        graph.add((activity, PROV.wasAssociatedWith, policy))
        graph.add((policy, RDF.type, PROV.SoftwareAgent))
        graph.add((policy, RDF.type, PROV.Agent))
        graph.add((policy, RDFS.label, Literal(policy_label)))
        usage = next(
            (
                node
                for node in graph.objects(activity, PROV.qualifiedUsage)
                if (node, PROV.entity, chosen) in graph
            ),
            None,
        )
        if usage is None:
            usage = BNode()
            graph.add((activity, PROV.qualifiedUsage, usage))
            graph.add((usage, RDF.type, PROV.Usage))
            graph.add((usage, PROV.entity, chosen))

    def _emit_distance_operand_selection(self, distance_node: URIRef, plan: _DistancePlan) -> None:
        """Record, as PROV, which pose coordinate each distance endpoint names.

        `between-entities` only carries the two frame-origin Points, and a live pose and its
        own snapshot collapse onto one when they share a frame (the `table-moved` drift check),
        so the Point alone cannot tell `operations.py` which coordinate to read. The DSL already
        knows the answer -- `plan.start`/`plan.end` -- so it runs a trivial coord_policy (the
        authored choice) and records the pick; `operations.py` reads it back by the same
        deterministic activity name instead of reversing an entity edge.
        """
        policy = self._owned_uri("coord-policy/distance-operand", None)
        for role, operand in (("start", plan.start), ("end", plan.end)):
            chosen = URIRef(operand.uri)
            relation = self.graph.value(chosen, URI_GEOM_PRED_OF_POSE)
            if relation is None:
                continue
            candidates = PoseModel(relation, self.graph).coordinate_ids
            activity = URIRef(f"{distance_node}-{role}-selection")
            self._record_coord_selection(
                activity,
                relation,
                candidates,
                chosen,
                policy,
                "distance operand: the coordinate the constraint names",
            )

    def _direction_pair_plan(
        self,
        moving: ContextQuantity,
        reference: ContextQuantity,
        context: str,
        world_qtys: dict[str, WorldQuantity],
        relation_a: str | None = None,
        relation_b: str | None = None,
    ) -> _AlignmentPlan:
        """Resolve two direction operands of an `angle between` view and the already computed
        pose (moving frame wrt reference frame) the rotated-direction op reads.

        Shared by all three Table IIb forms: versor-versor compares its two authored directions
        directly; versor-plane and plane-plane resolve their plane operand(s) to a normal
        direction first (see `_incident_angle_plan` / `_plane_angle_plan`) and pass it through
        here unchanged. `relation_a`/`relation_b` default to the direction operands themselves
        (versor-versor); the plane forms pass the plane entity instead of its normal.

        Unlike a distance's endpoints, this pose's *value* is read at runtime, so it must be an
        existing solver-computed `world` quantity -- there is no operator to derive a fresh
        relative pose from two bare frame names.
        """
        moving_frame = _geo_prop(moving.props, "as-seen-by") or _geo_prop(moving.props, "wrt")
        reference_frame = _geo_prop(reference.props, "as-seen-by") or _geo_prop(
            reference.props, "wrt"
        )
        if moving_frame is None or reference_frame is None:
            raise ValueError(f"{context} needs 'as-seen-by' frames on both directions.")
        target = next(
            (
                qty
                for qty in world_qtys.values()
                if qty.type == WorldQuantityType.Pose
                and isinstance(qty.props, GeometricProps)
                and _geo_prop(qty.props, "of") == moving_frame
                and _geo_prop(qty.props, "wrt") == reference_frame
            ),
            None,
        )
        if target is None:
            raise ValueError(
                f"{context} needs a declared 'world' pose of '{moving_frame}' wrt "
                f"'{reference_frame}': alignment reads an already-computed pose, it does "
                "not derive one."
            )
        # The rotation carries the moving direction into the frame the pose is seen by, and the
        # reference direction is read in `wrt`; a third frame would compare two unrelated vectors.
        if _geo_prop(target.props, "as-seen-by") not in (None, reference_frame):
            raise ValueError(
                f"{context} needs '{target.name}' seen by '{reference_frame}', the frame its "
                "reference direction is stated in."
            )
        return _AlignmentPlan(
            moving,
            reference,
            target,
            relation_a if relation_a is not None else str(moving.uri),
            relation_b if relation_b is not None else str(reference.uri),
        )

    def _alignment_plan(
        self,
        spec: ConstraintSpecification,
        world_qtys: dict[str, WorldQuantity],
    ) -> _AlignmentPlan:
        """Resolve a versor-versor `angle between` view's direction operands (plan cached by
        spec, shared with the incident-angle and plane-angle forms below)."""
        cached = self._alignment_plans.get(spec)
        if cached is not None:
            return cached
        plan = self._direction_pair_plan(
            spec.view.binary.left,
            spec.view.binary.right,
            f"Alignment constraint '{spec.name}'",
            world_qtys,
        )
        self._alignment_plans[spec] = plan
        return plan

    def _plane_normal(self, plane: ContextQuantity) -> ContextQuantity:
        """The resolved `direction` quantity a `plane`'s `normal:` role names. `_geo_prop`
        would give its URI string (the frame-lookup helper); this needs the object itself,
        the same way `validate_line_plane_primitives` reads it during validation.
        """
        return _resolved_context_quantity(_geo_prop_value(plane.props, GeometricPropKey.Normal))

    def _incident_angle_plan(
        self,
        spec: ConstraintSpecification,
        world_qtys: dict[str, WorldQuantity],
    ) -> _AlignmentPlan:
        """Resolve a versor-plane `angle between` view: the versor operand against the plane
        operand's normal direction."""
        cached = self._alignment_plans.get(spec)
        if cached is not None:
            return cached
        moving = _resolved_context_quantity(spec.view.binary.left)
        plane = _resolved_context_quantity(spec.view.binary.right)
        reference = self._plane_normal(plane)
        plan = self._direction_pair_plan(
            moving,
            reference,
            f"Incident-angle constraint '{spec.name}'",
            world_qtys,
            relation_a=str(moving.uri),
            relation_b=str(plane.uri),
        )
        self._alignment_plans[spec] = plan
        return plan

    def _plane_angle_plan(
        self,
        spec: ConstraintSpecification,
        world_qtys: dict[str, WorldQuantity],
    ) -> _AlignmentPlan:
        """Resolve a plane-plane `angle between` view: the first-named plane's normal (moving)
        against the second's (reference) -- see plan 09 Sec.1, only the first operand moves."""
        cached = self._alignment_plans.get(spec)
        if cached is not None:
            return cached
        plane_a = _resolved_context_quantity(spec.view.binary.left)
        plane_b = _resolved_context_quantity(spec.view.binary.right)
        moving = self._plane_normal(plane_a)
        reference = self._plane_normal(plane_b)
        plan = self._direction_pair_plan(
            moving,
            reference,
            f"Plane-angle constraint '{spec.name}'",
            world_qtys,
            relation_a=str(plane_a.uri),
            relation_b=str(plane_b.uri),
        )
        self._alignment_plans[spec] = plan
        return plan

    def _primitive_direction(self, primitive: ContextQuantity, context: str) -> ContextQuantity:
        """The direction context quantity a `line`/`plane` primitive composes (its `along` or
        `normal`); plan 06 validation guarantees it exists and resolves to a well-formed
        direction by the time RDF emission runs.
        """
        key = (
            GeometricPropKey.Normal
            if primitive.type == QuantityType.Plane
            else GeometricPropKey.Along
        )
        referent = next((pair.value for pair in primitive.props.pairs if pair.key == key), None)
        if referent is None:
            raise ValueError(f"{context} needs '{primitive.name}' to declare its {key.value}.")
        return _resolved_context_quantity(referent)

    def _existing_world_pose(
        self, world_qtys: dict[str, WorldQuantity], of_frame: str, wrt_frame: str
    ) -> WorldQuantity | None:
        """An already-declared `world` Pose(of, wrt) quantity, or None.

        Mirrors `_alignment_plan`: a Table IIa expression reads an already-computed pose the
        same way alignment does, it does not derive one -- minting a fresh of/wrt pose here
        would duplicate whatever fixed/articulated chain already relates the two frames, which
        is exactly the resources.py agent-placement and dataflow-scheduling machinery's job, not
        this emitter's (`feedback_no_generated_copies_of_runtime_data`).
        """
        return next(
            (
                qty
                for qty in world_qtys.values()
                if qty.type == WorldQuantityType.Pose
                and isinstance(qty.props, GeometricProps)
                and _geo_prop(qty.props, "of") == of_frame
                and _geo_prop(qty.props, "wrt") == wrt_frame
            ),
            None,
        )

    def _emit_pose_difference_coordinate(
        self, node: URIRef, as_seen_by_frame: str, motion: GuardedMotion, stem: str
    ) -> None:
        """Emit a `PoseDifferenceCoordinate` node: the shape `quantities.pose_difference` (the
        motion-spec IR reader `emit-call-PoseDiffEvaluator`'s closure writes into) expects.
        """
        point_node = self._declared_uri(f"point-{stem}-origin", motion)
        self.graph.add((point_node, RDF.type, GEOM_ENT.Point))
        self.graph.add((node, RDF.type, GEOM_COORD.PoseDifferenceCoordinate))
        self.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], QUDT_QKIND.PlaneAngle))
        self.graph.add((node, QUDT_SCHEMA["hasQuantityKind"], URI_QUDT_QK_LENGTH))
        self.graph.add((node, GEOM_REL["reference-point"], point_node))
        self.graph.add((node, GEOM_COORD["as-seen-by"], self._owned_uri(as_seen_by_frame, motion)))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))

    def _geometric_distance_plan(
        self,
        spec: ConstraintSpecification,
        world_qtys: dict[str, WorldQuantity],
    ) -> _GeometricDistancePlan:
        """Resolve an authored Table IIa `distance of`/`projection of` view (plan 08): which of
        the five operators it dispatches to, its operand poses/directions, and the scalar-view
        carrier `_resolve_constraint_quantity` returns for it.

        Every pose operand -- the point (ops 1-3) and both line origins (ops 4-5) -- has to
        already be a declared `world` pose (same requirement `_alignment_plan` has): this
        emitter reads already-computed poses, it does not derive new ones. Ops 4-5 additionally
        difference the two origins through a `PoseDiffEvaluator`.
        """
        cached = self._geometric_distance_plans.get(spec)
        if cached is not None:
            return cached

        binary = spec.view.binary
        a_ref, b_ref = binary.left, binary.right
        table = (
            GEOMETRIC_DISTANCE_OPS
            if _is_geometric_distance_view(spec)
            else GEOMETRIC_PROJECTION_OPS
        )
        op_type = table[(_geometric_operand_kind(a_ref), _geometric_operand_kind(b_ref))]
        context = f"Constraint '{spec.name}'"
        motion = getattr(getattr(spec, "parent", None), "parent", None)
        stem = f"geo-distance-{getattr(motion, 'name', '')}-{spec.name}"

        if op_type in ("LineLineToLinearDistance", "LineOnLineProjection"):
            line_a = _resolved_context_quantity(a_ref)
            line_b = _resolved_context_quantity(b_ref)
            dir_a = self._primitive_direction(line_a, context)
            dir_b = self._primitive_direction(line_b, context)
            frame = _geo_prop(dir_a.props, "as-seen-by")
            if frame is None or _geo_prop(dir_b.props, "as-seen-by") != frame:
                raise ValueError(
                    f"{context} needs both lines' directions stated 'as-seen-by' the same frame."
                )
            origin_a = self._existing_world_pose(world_qtys, _geo_prop(line_a.props, "of"), frame)
            origin_b = self._existing_world_pose(world_qtys, _geo_prop(line_b.props, "of"), frame)
            for line, origin in ((line_a, origin_a), (line_b, origin_b)):
                if origin is None:
                    raise ValueError(
                        f"{context} needs a declared 'world' pose of '{_geo_prop(line.props, 'of')}' "
                        f"wrt '{frame}': a Table IIa expression reads an already-computed pose, "
                        "it does not derive one."
                    )
            pose_diff = self._owned_uri(f"{stem}-pose-diff", motion)
            diff_op = self._owned_uri(f"compute-{stem}-pose-diff", motion)
            self.graph.add((diff_op, RDF.type, GEOM_OP_EXT.PoseDiffEvaluator))
            self.graph.add((diff_op, GEOM_OP.in1, URIRef(origin_a.uri)))
            self.graph.add((diff_op, GEOM_OP.in2, URIRef(origin_b.uri)))
            self.graph.add((diff_op, GEOM_OP.out, pose_diff))
            self._emit_pose_difference_coordinate(pose_diff, frame, motion, stem)
            target_props = GeometricProps(
                [
                    GeoPropPair(GeometricPropKey.Of, _geo_prop(line_b.props, "of")),
                    GeoPropPair(GeometricPropKey.Wrt, frame),
                    GeoPropPair(GeometricPropKey.AsSeenBy, frame),
                ]
            )
            target = WorldQuantity(
                parent=motion, name=stem, type=WorldQuantityType.Pose, props=target_props
            )
            plan = _GeometricDistancePlan(
                op_type=op_type,
                in1=str(dir_a.uri),
                in2=str(dir_b.uri),
                direction=None,
                pose=str(pose_diff),
                diff_in1=str(origin_a.uri),
                diff_in2=str(origin_b.uri),
                relation_a=str(line_a.uri),
                relation_b=str(line_b.uri),
                gradient_frame=frame,
                target=target,
            )
        else:
            point_qty = self._distance_operand(a_ref, world_qtys)
            primitive = _resolved_context_quantity(b_ref)
            direction_qty = self._primitive_direction(primitive, context)
            direction_frame = _geo_prop(direction_qty.props, "as-seen-by")
            point_frames = _pose_frame_names(point_qty)
            if point_frames is None:
                raise ValueError(f"{context} needs an explicit-frame pose operand.")
            point_of, point_wrt, _ = point_frames
            if direction_frame is None or direction_frame != point_wrt:
                role = "normal" if primitive.type == QuantityType.Plane else "direction"
                raise ValueError(
                    f"{context} needs '{primitive.name}' {role} stated 'as-seen-by' "
                    f"'{point_wrt}', the point operand's own reference frame."
                )
            primitive_frame = _geo_prop(primitive.props, "of")
            if op_type == "PointLineToLinearDistance" and primitive_frame == point_wrt:
                # The line rides the frame the point is measured against (Borghesan's
                # line-point distance from the grasping axis): its origin is that frame's own
                # origin, so no origin pose exists to read, and the operator carries an
                # angular gradient half besides the linear one.
                op_type = "PointBodyLineToLinearDistance"
                target = WorldQuantity(
                    parent=motion, name=stem, type=WorldQuantityType.Pose, props=point_qty.props
                )
                plan = _GeometricDistancePlan(
                    op_type=op_type,
                    in1=str(point_qty.uri),
                    in2=str(point_qty.uri),
                    direction=str(direction_qty.uri),
                    pose=None,
                    diff_in1=None,
                    diff_in2=None,
                    relation_a=str(self._frame_origin(self._owned_uri(point_of, motion))),
                    relation_b=str(primitive.uri),
                    gradient_frame=point_wrt,
                    target=target,
                )
                self._geometric_distance_plans[spec] = plan
                return plan
            origin = self._existing_world_pose(world_qtys, primitive_frame, point_wrt)
            if origin is None:
                raise ValueError(
                    f"{context} needs a declared 'world' pose of '{primitive_frame}' wrt "
                    f"'{point_wrt}': a Table IIa expression reads an already-computed pose, it "
                    "does not derive one."
                )
            target = WorldQuantity(
                parent=motion, name=stem, type=WorldQuantityType.Pose, props=origin.props
            )
            plan = _GeometricDistancePlan(
                op_type=op_type,
                in1=str(point_qty.uri),
                in2=str(origin.uri),
                direction=str(direction_qty.uri),
                pose=None,
                diff_in1=None,
                diff_in2=None,
                relation_a=str(self._frame_origin(self._owned_uri(point_of, motion))),
                relation_b=str(primitive.uri),
                gradient_frame=point_wrt,
                target=target,
            )

        self._geometric_distance_plans[spec] = plan
        return plan

    def _linear_distance_relation(
        self, start_uri: str, end_uri: str, relation_type: URIRef
    ) -> URIRef:
        """The linear-distance relation two entities stand in, minted once per entity pair.

        The relation is the geometric fact; a constraint's coordinate is one motion's sampling
        of it. Motions measuring the same two entities share the relation and differ only in
        the coordinate, so the entity pair -- not the constraint's name -- is its identity.
        `relation_type` is the specific `geom-rel:PointToPointDistance` /
        `geom-rel-ext:PointPlaneDistance` / etc. the operand kinds dispatch to.
        """
        key = (str(start_uri), str(end_uri))
        node = self._linear_distance_relations.get(key)
        if node is not None:
            return node
        name = "-".join(
            ["linear-distance", self._model_local_path(start_uri), self._model_local_path(end_uri)]
        )
        node = self._owned_uri(name, None)
        self.graph.add((node, RDF.type, relation_type))
        self.graph.add((node, GEOM_REL["between-entities"], URIRef(start_uri)))
        self.graph.add((node, GEOM_REL["between-entities"], URIRef(end_uri)))
        self._linear_distance_relations[key] = node
        return node

    def _angular_distance_relation(self, a_uri: str, b_uri: str, relation_type: URIRef) -> URIRef:
        """The angular-distance relation two entities stand in, minted once per entity pair.

        Mirrors `_linear_distance_relation`: the relation is the geometric fact, a constraint's
        angle coordinate is one motion's sampling of it, and two motions measuring the same
        entity pair share the relation.
        """
        key = (str(a_uri), str(b_uri))
        node = self._angular_distance_relations.get(key)
        if node is not None:
            return node
        name = "-".join(
            ["angular-distance", self._model_local_path(a_uri), self._model_local_path(b_uri)]
        )
        node = self._owned_uri(name, None)
        self.graph.add((node, RDF.type, relation_type))
        self.graph.add((node, GEOM_REL["between-entities"], URIRef(a_uri)))
        self.graph.add((node, GEOM_REL["between-entities"], URIRef(b_uri)))
        self._angular_distance_relations[key] = node
        return node

    def _model_local_path(self, uri: str) -> str:
        """A URI's path below the model namespace, flattened into one name segment.

        The whole path, not the last segment: two motions declare the same `start-pose`, and
        only the path above it tells them apart. A relation entity can come from outside the
        model too -- a scene-imported frame's origin Point, say -- so a URI the model namespace
        does not own instead flattens its own scheme-stripped path, keeping it readable and
        unique without assuming it is model-local.
        """
        namespace = str(self._namespace_owner(None).ns.uri)
        if str(uri).startswith(namespace):
            return str(uri).removeprefix(namespace).replace("/", "-")
        _, _, rest = str(uri).partition("://")
        return rest.replace("/", "-")

    def _frame_coords(self, node: URIRef) -> tuple[URIRef, URIRef, URIRef] | None:
        """Resolved (of, wrt, as-seen-by) frame nodes recorded when `node`'s pose
        coordinate was emitted, or None if none were emitted for it."""
        return self._frame_coords_index.get(node)

    def _emit_view(self, view_node: URIRef) -> None:
        """Type a node as a map:View and record it, so idempotency checks read the
        Python registry instead of querying the graph."""
        self.graph.add((view_node, RDF.type, MAP.View))
        self._emitted_views.add(view_node)

    def _emit_snapshot_geometry_metadata(
        self,
        node: URIRef,
        quantity: ContextQuantity,
    ) -> None:
        """Give a snapshot-with-offset result the frames its own quantity is stated in.

        The offset's output is a new quantity carrying none of them. A position without frames
        surfaces as a plain scalar; a pose without them is rejected outright, as a relation that
        links to no `of`. `_pose_frame_names` supplies the precedence: the frames the quantity
        declares, or the snapshotted source's when it declares none. That distinction matters for
        a derived goal — a pose built from the handle but authored `of` the gripper is a goal for
        the gripper, and a path may only join endpoints that are poses of the same body.
        """
        if quantity.type == QuantityType.Pose:
            # A pose coordinate carries both of its units, the way a world pose does.
            self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
            self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
            pose_relation = self._emit_declared_pose_frame_metadata(node, quantity)
            if pose_relation is not None:
                self._emit_combined_pose_coordinate(node, pose_relation)
            return
        frames = _pose_frame_names(quantity)
        if frames is None:
            return
        of_frame, wrt_frame, as_seen_by = frames
        self.graph.add((node, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
        self._emit_geom_relation(
            node,
            "position",
            self._owned_uri(of_frame, quantity),
            self._owned_uri(wrt_frame, quantity),
            self._owned_uri(as_seen_by, quantity),
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
        signal_node = self._declared_uri(f"force-{ctrl.name}", ctrl)
        self._add_quantity(signal_node, QuantityType.Force)
        return signal_node

    def _moment_control_signal_node(
        self, ctrl: ControllerEntry, handler: ConstraintHandler, axis: str | None = None
    ) -> URIRef:
        """Owned Torque-quantity node carrying one axis of a moment controller's control signal."""
        name = f"moment-{ctrl.name}" if axis is None else f"moment-{ctrl.name}-ang-{axis}"
        signal_node = self._owned_uri(name, handler)
        self._add_quantity(signal_node, QuantityType.Torque)
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

    def _commanded_wrench_node(self, qty: WorldQuantity) -> URIRef | None:
        """The declared wrench a command realizes, or None when `qty` is not one.

        A wrench carrying `ft-sensor` states what that sensor reads and is never commanded;
        validation rejects assigning to it.
        """
        if qty.type != WorldQuantityType.Wrench:
            return None
        props = qty.props if isinstance(qty.props, GeometricProps) else None
        if _geo_prop(props, "ft-sensor") is not None:
            return None
        return URIRef(qty.uri)

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

        direction_node = self._declared_uri(f"direction-{ctrl.name}", ctrl)
        if (
            qty.type == WorldQuantityType.Pose
            and _view_subspace(spec) == "distance"
            and axis is None
        ):
            self._emit_pose_to_direction(direction_node, qty, as_seen_by_node, motion, ctrl.name)
        elif axis is not None:
            self._emit_direction_coordinate(direction_node, as_seen_by_node, _axis_vector(axis))
        else:
            gradient_id = _gradient_scalar_id(qty, spec)
            if gradient_id is None:
                raise ValueError(
                    f"Force controller '{ctrl.name}' needs an axis or a distance pose."
                )
            direction_node = self._owned_uri(gradient_id, motion)

        point_node = self._declared_uri(f"point-force-{ctrl.name}", ctrl)
        position_node = self._declared_uri(f"position-force-{ctrl.name}", ctrl)
        self._emit_zero_position_coordinate(position_node, point_node, as_seen_by_node)
        # A commanded wrench -- one no sensor observes -- is what this op produces: the model
        # declared the quantity the command realizes, so the value belongs in it rather than in
        # a second wrench beside it. Its coordinate is already emitted with the world quantities.
        wrench_node = self._commanded_wrench_node(qty)
        if wrench_node is None:
            wrench_node = self._declared_uri(f"wrench-force-{ctrl.name}", ctrl)
            self._emit_wrench_coordinate(wrench_node, point_node, as_seen_by_node)

        op_node = self._declared_uri(f"compute-wrench-force-{ctrl.name}", ctrl)
        self.graph.add((op_node, RDF.type, RBDYN_OP.WrenchFromPositionDirectionAndMagnitude))
        self.graph.add((op_node, RBDYN_OP.magnitude, magnitude_node))
        self.graph.add((op_node, RBDYN_OP.direction, direction_node))
        self.graph.add((op_node, RBDYN_OP.position, position_node))
        self.graph.add((op_node, RBDYN_OP.wrench, wrench_node))

        return wrench_node

    def _emit_moment_command_wrench(
        self,
        ctrl: ControllerEntry,
        spec: ConstraintSpecification,
        qty: WorldQuantity,
        command: Any,
        handler: ConstraintHandler,
        motion: GuardedMotion,
    ) -> URIRef:
        """Emit the op chain building a moment controller's command wrench: one
        WrenchFromDirectionAndMoment per commanded angular axis, folded with AddWrench --
        or, when the view is a geometric expression with no named axis, a single
        WrenchFromDirectionAndMoment wired to that expression's runtime gradient direction.
        A couple is reference-point independent, so no position enters the ops.
        """
        apply_at = getattr(ctrl, "apply_at", None)
        if apply_at is None or not hasattr(apply_at, "uri"):
            raise ValueError(f"Moment controller '{ctrl.name}' must specify 'apply at <link>'.")

        props = qty.props if isinstance(qty.props, GeometricProps) else None
        as_seen_by_name = _geo_prop(props, "as-seen-by") or _geo_prop(props, "wrt")
        if as_seen_by_name is None:
            raise ValueError(
                f"Moment controller '{ctrl.name}' needs a frame from the constrained quantity."
            )
        as_seen_by_node = self._owned_uri(as_seen_by_name, qty)

        axes = [axis for _, axis in command.controlled_axes if axis is not None]

        # The wrench coordinate still needs a well-formed reference point; the op ignores it.
        point_node = self._declared_uri(f"point-moment-{ctrl.name}", ctrl)
        position_node = self._declared_uri(f"position-moment-{ctrl.name}", ctrl)
        self._emit_zero_position_coordinate(position_node, point_node, as_seen_by_node)

        if not axes:
            gradient_id = _gradient_scalar_id(qty, spec)
            if gradient_id is None:
                raise ValueError(f"Moment controller '{ctrl.name}' commands no angular axis.")
            direction_node = self._owned_uri(gradient_id, motion)
            magnitude_node = self._moment_control_signal_node(ctrl, handler, None)
            wrench_node = self._declared_uri(f"wrench-moment-{ctrl.name}", ctrl)
            self._emit_wrench_coordinate(wrench_node, point_node, as_seen_by_node)

            op_node = self._declared_uri(f"compute-wrench-moment-{ctrl.name}", ctrl)
            self.graph.add((op_node, RDF.type, RBDYN_OP_EXT.WrenchFromDirectionAndMoment))
            self.graph.add((op_node, RBDYN_OP_EXT.moment, magnitude_node))
            self.graph.add((op_node, RBDYN_OP.direction, direction_node))
            self.graph.add((op_node, RBDYN_OP.wrench, wrench_node))
            return wrench_node

        multi = len(axes) > 1
        wrench_nodes: list[URIRef] = []
        for axis in axes:
            direction_node = self._declared_uri(f"direction-moment-{ctrl.name}-ang-{axis}", ctrl)
            self._emit_direction_coordinate(direction_node, as_seen_by_node, _axis_vector(axis))
            magnitude_node = self._moment_control_signal_node(
                ctrl, handler, axis if multi else None
            )
            wrench_node = self._declared_uri(f"wrench-moment-{ctrl.name}-ang-{axis}", ctrl)
            self._emit_wrench_coordinate(wrench_node, point_node, as_seen_by_node)

            op_node = self._declared_uri(f"compute-wrench-moment-{ctrl.name}-ang-{axis}", ctrl)
            self.graph.add((op_node, RDF.type, RBDYN_OP_EXT.WrenchFromDirectionAndMoment))
            self.graph.add((op_node, RBDYN_OP_EXT.moment, magnitude_node))
            self.graph.add((op_node, RBDYN_OP.direction, direction_node))
            self.graph.add((op_node, RBDYN_OP.wrench, wrench_node))
            wrench_nodes.append(wrench_node)

        total = wrench_nodes[0]
        for index, addend in enumerate(wrench_nodes[1:], start=1):
            sum_node = self._declared_uri(f"wrench-moment-{ctrl.name}-sum-{index}", ctrl)
            self._emit_wrench_coordinate(sum_node, point_node, as_seen_by_node)
            add_node = self._declared_uri(f"add-wrench-{ctrl.name}-{index}", ctrl)
            self.graph.add((add_node, RDF.type, RBDYN_OP.AddWrench))
            self.graph.add((add_node, RBDYN_OP["in1"], total))
            self.graph.add((add_node, RBDYN_OP["in2"], addend))
            self.graph.add((add_node, RBDYN_OP.out, sum_node))
            total = sum_node
        return total

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
                    else self._declared_uri(f"point-{qty.name}-origin", qty)
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
                retare_events = _geo_prop_events(props, "re-tare-on")
                if retare_events and not ft_ref:
                    raise ConstraintViolation(
                        "dynamics",
                        f"Wrench '{qty.name}' names re-tare-on events but no ft-sensor to tare.",
                    )
                # The tare is a sampling of what the sensor reads unloaded: once at startup, and
                # again on each named occurrence. There is no implicit "once at startup" left --
                # an author who wants that names their model's own run-start event explicitly.
                if ft_ref and not retare_events:
                    raise ConstraintViolation(
                        "dynamics",
                        f"Wrench '{qty.name}' has an ft-sensor but names no re-tare-on events; "
                        "name the model's run-start event to tare once at startup.",
                    )
                for event in retare_events:
                    if event.event is None:
                        raise ConstraintViolation(
                            "dynamics",
                            f"Wrench '{qty.name}' re-tare-on '{event.name}' carries no namespace, "
                            "so it resolves to no declared event; write it as ns.EVENT.",
                        )
                    schedule_node = URIRef(f"{node}-retare-{event.name}-schedule")
                    self.graph.add((schedule_node, RDF.type, URI_TIME_TYPE_AFTER_EVT))
                    self.graph.add((schedule_node, RDF.type, URI_TIME_TYPE_TC))
                    self.graph.add((schedule_node, URI_TIME_PRED_OF_CONSTRAINT, node))
                    self.graph.add((schedule_node, URI_TIME_PRED_AFTER_EVT, URIRef(event.uri)))
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
                normalization = _geo_prop_value(props, "normalization")
                if normalization is not None:
                    self._emit_angle_normalization(node, normalization, qty, f"norm-{qty.name}")

            if of_v:
                self.graph.add((node, GEOM_REL.of, self._owned_uri(of_v, qty)))
            if wrt_v:
                self.graph.add((node, GEOM_REL["with-respect-to"], self._owned_uri(wrt_v, qty)))
            if rp_v:
                ref_node = self._owned_uri(rp_v, qty)
                self.graph.add((node, GEOM_REL["reference-point"], ref_node))
            elif qty.type == WorldQuantityType.VelocityTwist:
                point_node = self._declared_uri(f"point-{qty.name}-origin", qty)
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
        binary = getattr(view, "binary", None)
        if binary is not None and type(binary).__name__ == "DistanceBetweenView":
            return self._owned_uri(
                f"distance-{_node_name(binary.left)}-{_node_name(binary.right)}",
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
                    return self._component_view(URIRef(quantity.uri), "position")
                if mapped_subspace in {"orientation", "rotation"}:
                    return self._component_view(URIRef(quantity.uri), "orientation")
            return scalar_uri

        return self._owned_uri(_node_name(quantity), owner)

    def _register_wrench_vector_view(
        self,
        scalar_uri: URIRef,
        quantity: WorldQuantity,
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
        quantity: WorldQuantity,
        owner: Any,
    ) -> None:
        """Promote `<pose>.position` and register its whole-vector coordinate view."""
        if scalar_uri in self._emitted_position_coords:
            return
        self._emitted_position_coords.add(scalar_uri)
        position_relation = self._component_relation(URIRef(quantity.uri), "position")

        view_uri = self._owned_uri(f"view-{_scalar_id(quantity, 'position', None)}", owner)
        if view_uri not in self._emitted_views:
            self._emit_view(view_uri)
            self.graph.add((view_uri, RDF.type, MAP_EXT.PoseCoordinateView))
            self.graph.add((view_uri, MAP.superobject, URIRef(quantity.uri)))
            self.graph.add((view_uri, MAP.subobject, position_relation))
            self.graph.add((view_uri, MAP.subspace, MAP_EXT.position))
            # axis intentionally omitted: this view exposes the whole 3-vector.
        self._register_component_view(URIRef(quantity.uri), "position", view_uri)

    def _register_pose_component_view(
        self,
        scalar_uri: URIRef,
        quantity: WorldQuantity,
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
        quantity: WorldQuantity,
        owner: Any,
    ) -> None:
        """Promote `<pose>.orientation` and register its coordinate view."""
        if scalar_uri in self._emitted_orientation_coords:
            return
        self._emitted_orientation_coords.add(scalar_uri)
        orientation_relation = self._component_relation(URIRef(quantity.uri), "orientation")

        view_uri = self._owned_uri(f"view-{_scalar_id(quantity, 'orientation', None)}", owner)
        if view_uri not in self._emitted_views:
            self._emit_view(view_uri)
            self.graph.add((view_uri, RDF.type, MAP_EXT.PoseCoordinateView))
            self.graph.add((view_uri, MAP.superobject, URIRef(quantity.uri)))
            self.graph.add((view_uri, MAP.subobject, orientation_relation))
            self.graph.add((view_uri, MAP.subspace, MAP_EXT.orientation))
        self._register_component_view(URIRef(quantity.uri), "orientation", view_uri)

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
                else self._declared_uri(f"point-{quantity.name}-origin", quantity)
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
                else self._declared_uri(f"point-{quantity.name}-origin", quantity)
            )
            self.graph.add((point_node, RDF.type, GEOM_ENT.Point))
            self.graph.add((node, GEOM_REL["reference-point"], point_node))
            if asb_v:
                self.graph.add((node, GEOM_COORD["as-seen-by"], self._owned_uri(asb_v, owner)))
            return

    def _context_ref_view_spec(
        self,
        quantity: ContextQuantity,
        subspace_raw: str | None,
        axis: str | None,
    ) -> tuple[Any, Any, Any] | None:
        """The (scalar type, view type, view subspace) for a context pose/path reference's
        subspace+axis, or None when that subspace exposes no view.

        A bare axis names a component of a quantity that is itself a 3-vector, so it has no
        view type or subspace of its own -- only the axis link distinguishes it.
        """
        if subspace_raw is None:
            component_type = VECTOR_COMPONENT_TYPE.get(quantity.type)
            return None if component_type is None else (component_type, None, None)
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
        subspace_raw: str | None,
        axis: str | None,
    ) -> URIRef:
        """Emit (once) the scalar/coordinate node and its map:View for a subspace of a context
        pose/path reference. Returns the view for a whole position/orientation component -- it
        is what names that operand -- and the scalar node for every other subspace.
        """
        view_spec = self._context_ref_view_spec(quantity, subspace_raw, axis)
        if view_spec is None:
            return URIRef(quantity.uri)
        scalar_type, view_type, view_subspace = view_spec
        suffix = ".".join(part for part in (subspace_raw, axis) if part is not None)
        node = URIRef(f"{quantity.uri}.{suffix}")
        super_node = (
            self._reference_output_node(quantity)
            if quantity.type == ReferenceGeneratorType.Path
            else URIRef(quantity.uri)
        )
        view_target = node
        component_coord: URIRef | None = None
        if (
            quantity.type in {QuantityType.Pose, ReferenceGeneratorType.Path}
            and subspace_raw in {"position", "orientation"}
            and axis is None
        ):
            component_coord = (
                URIRef(f"{quantity.uri}.{subspace_raw}")
                if _owns_pose_subobjects(quantity.value)
                else super_node
            )
            emitted = self._component_view_index.get((component_coord, subspace_raw))
            if emitted is not None:
                # One view per (coordinate, component); a second one of the same shape would
                # only re-create the ambiguity the pooled relation had.
                return emitted
            view_target = self._component_relation(component_coord, subspace_raw)

        if view_target == node:
            self._add_quantity(node, scalar_type)

        view_node = URIRef(f"{quantity.uri}.view-{suffix}")
        if view_node not in self._emitted_views:
            self._emit_view(view_node)
            if view_type is not None and (
                axis is None
                or quantity.type
                not in {
                    QuantityType.Pose,
                    ReferenceGeneratorType.Path,
                }
            ):
                self.graph.add((view_node, RDF.type, view_type))
            self.graph.add((view_node, MAP.superobject, super_node))
            self.graph.add((view_node, MAP.subobject, view_target))
            if view_subspace is not None:
                self.graph.add((view_node, MAP.subspace, view_subspace))
            if axis is not None:
                self.graph.add((view_node, MAP.axis, MAP[axis]))
        if component_coord is not None:
            # The operand is this view -- coordinate plus component -- not the pooled relation.
            self._register_component_view(component_coord, subspace_raw, view_node)
            return view_node
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
                self._emit_direction_quantity(node, quantity, world_qtys)
                continue
            if quantity.type in (QuantityType.Line, QuantityType.Plane):
                self._emit_structural_primitive(node, quantity)
                continue
            if quantity.type == ReferenceGeneratorType.VelocityProfile:
                self._emit_velocity_profile_quantity(node, quantity)
                continue
            if quantity.type == ReferenceGeneratorType.Admittance:
                self._emit_admittance_quantity(node, quantity)
                continue
            if isinstance(quantity.value, SampledValue):
                self._emit_sampled_quantity(node, quantity)
                continue
            if quantity.type == QuantityType.Duration and isinstance(quantity.value, Measure):
                self._emit_duration_measure(node, quantity.value)
                continue
            if isinstance(quantity.value, ConfigValue):
                self._emit_config_pose_quantity(node, quantity)
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
                expr_tree = quantity.value.expr.as_op_tree()
                if isinstance(expr_tree, QOpNode):
                    self._emit_qexpr(expr_tree, quantity, node)
                    self.graph.add(
                        (node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(quantity.type, QUDT_UNIT.UNITLESS))
                    )
                else:
                    source_node = self._emit_context_ref_node(
                        quantity.value.source, quantity, "source"
                    )
                    self.graph.add(
                        (node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(quantity.type, QUDT_UNIT.UNITLESS))
                    )
                    self.graph.add((node, CSTR["reference-value"], source_node))
                continue
            if isinstance(quantity.value, SnapshotValue):
                # A snapshot derives from another quantity (prov:wasDerivedFrom); it is not a
                # sosa:Observation -- only a sensor act is, and the shape demands madeBySensor.
                view_node = self._view_node(quantity.value.source, quantity)
                snap_tree = quantity.value.as_op_tree()
                if isinstance(snap_tree, QOpNode):
                    # Own the op nodes by the quantity's motion-qualified URI (not the flat namespace) so two
                    # motions declaring a same-named quantity don't collapse into one op accumulating both inputs.
                    out_node = URIRef(f"{node}-{snap_tree.op}-out")
                    qkind = (
                        QUDT_KIND_BY_QUANTITY_TYPE.get(quantity.type) or QUDT_QKIND[quantity.type]
                    )
                    self._emit_qexpr(snap_tree, quantity, out_node)
                    self.graph.add((out_node, RDF.type, QUDT_SCHEMA.Quantity))
                    # A pose's kind is structural and its units are the pair a pose coordinate
                    # carries, so both are left to the geometry emission below; stamping the
                    # scalar forms here would type the coordinate as the relation itself and
                    # give it a unitless length.
                    if quantity.type is not QuantityType.Pose:
                        self._emit_quantity_kind(out_node, qkind)
                        self.graph.add(
                            (
                                out_node,
                                QUDT_SCHEMA.unit,
                                SCALAR_UNIT.get(quantity.type, QUDT_UNIT.UNITLESS),
                            )
                        )
                    # A geometry-valued offset result keeps the source's frames: Position so the IR
                    # types it as a 3-vector rather than a scalar, Pose so it is a well-formed
                    # relation. (Scalar offsets, e.g. LinearDistance, stay scalar.)
                    if quantity.type in {QuantityType.Position, QuantityType.Pose}:
                        self._emit_snapshot_geometry_metadata(out_node, quantity)
                    snap_source = out_node
                else:
                    snap_source = view_node
                self.graph.add((node, PROV.wasDerivedFrom, snap_source))
                schedule_node = URIRef(f"{node}-schedule")
                self.graph.add((schedule_node, RDF.type, URI_TIME_TYPE_AFTER_EVT))
                self.graph.add((schedule_node, RDF.type, URI_TIME_TYPE_TC))
                self.graph.add((schedule_node, URI_TIME_PRED_OF_CONSTRAINT, node))
                self.graph.add(
                    (schedule_node, URI_TIME_PRED_AFTER_EVT, URIRef(quantity.value.trigger.uri))
                )
                self.graph.add(
                    (node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(quantity.type, QUDT_UNIT.UNITLESS))
                )
                if quantity.type is QuantityType.Pose:
                    self._emit_declared_pose_frame_metadata(node, quantity)
                elif quantity.type == QuantityType.Position:
                    self._emit_snapshot_geometry_metadata(node, quantity)
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
                        value_obj = Literal(float(element.value), datatype=XSD.double)
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

    def _add_literal_list_once(self, node: URIRef, predicate: URIRef, values: tuple) -> None:
        """Shared context re-emits per motion; plain triples dedupe but an RDF list mints
        fresh blank nodes each time, so a second emission must be skipped."""
        if (node, predicate, None) not in self.graph:
            add_literal_list_pred(self.graph, node, predicate, values)

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

    def _emit_config_pose_quantity(self, node: URIRef, quantity: ContextQuantity) -> None:
        """Emit a pose the deployment states: the same relation, coordinates and views a literal
        pose gets, minus the coordinate values, plus the resource they are read from.

        The frames come from the quantity it is a value for, the way a snapshot's do, and the
        authored key rides on `schema:identifier` -- what the config file calls this pose. The
        declaration must be a shared one: the file is read once before the loop, so a per-motion
        one would promise a value that changes with the state and never does.
        """
        if not isinstance(getattr(quantity.parent, "parent", None), ContextSpec):
            raise ConstraintViolation(
                "geometry",
                f"Pose '{quantity.name}' reads the deployment config, so it must be declared in "
                "a shared context: it is read once for the run, not per motion.",
            )
        if quantity.type != QuantityType.Pose:
            raise ConstraintViolation(
                "geometry",
                f"'{quantity.name}' reads the deployment config, which states poses; "
                f"a {quantity.type} cannot come from one.",
            )
        if self._config_resource is None:
            raise ConstraintViolation(
                "platform",
                f"Pose '{quantity.name}' reads the deployment config, but the exec-context "
                'declares no `config: "<file>.toml"`.',
            )
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((node, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
        self.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.UNITLESS))
        self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit("m")))
        self.graph.add((node, EXEC["has-resource"], self._config_resource))
        self.graph.add((node, SDO.identifier, Literal(quantity.value.key)))

        pose_relation = self._emit_declared_pose_frame_metadata(node, quantity)
        if pose_relation is None:
            raise ConstraintViolation(
                "geometry",
                f"Pose '{quantity.name}' reads the deployment config, so the quantity it is "
                "stated `for` must declare of/with-respect-to/as-seen-by frames.",
            )
        position_node = URIRef(f"{quantity.uri}.position")
        orientation_node = URIRef(f"{quantity.uri}.orientation")
        self.graph.add((position_node, RDF.type, QUDT_SCHEMA.Quantity))
        self.graph.add((position_node, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
        self.graph.add((position_node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        self.graph.add((orientation_node, RDF.type, QUDT_SCHEMA.Quantity))
        self._emit_orientation_type(orientation_node, None)

        pose_of, pose_wrt, pose_asb = self._frame_coords_index[node]
        position_relation = self._emit_geom_relation(
            position_node,
            "position",
            self._frame_origin(pose_of),
            self._frame_origin(pose_wrt),
            pose_asb or pose_wrt,
            (URI_QUDT_QK_LENGTH,),
        )
        orientation_relation = self._emit_geom_relation(
            orientation_node, "orientation", pose_of, pose_wrt, pose_asb or pose_wrt
        )
        # The pooled Position/Orientation relation is shared by every pose over these frames, so
        # which coordinate is *this* pose's is recorded here rather than guessed downstream.
        self._record_pose_component(node, "position", position_node)
        self._record_pose_component(node, "orientation", orientation_node)
        self.graph.add((pose_relation, RDF.type, URI_GEOM_TYPE_POSITION_REF))
        self.graph.add((pose_relation, RDF.type, URI_GEOM_TYPE_ORIENT_REF))
        self.graph.add((pose_relation, URI_GEOM_PRED_OF_POSITION, position_relation))
        self.graph.add((pose_relation, URI_GEOM_PRED_OF_ORIENT, orientation_relation))
        for coord, subobject, subspace, label in (
            (position_node, position_relation, MAP_EXT.position, "position"),
            (orientation_node, orientation_relation, MAP_EXT.orientation, "orientation"),
        ):
            # Named from `node` (this quantity), not `subobject`: the relation pools by frame
            # pair since plan 11 phase B, so several quantities can share one, and each still
            # needs its own view -- one map:superobject per view (map-ext:PoseCoordinateView).
            view_node = URIRef(f"{node}-{label}-view")
            self._emit_view(view_node)
            self.graph.add((view_node, RDF.type, MAP_EXT.PoseCoordinateView))
            self.graph.add((view_node, MAP.superobject, node))
            self.graph.add((view_node, MAP.subobject, subobject))
            self.graph.add((view_node, MAP.subspace, subspace))
            self._register_component_view(coord, label, view_node)

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
        # A pose coordinate carries both of its units: the length its translation is in, and the
        # angle its rotation is in. `UNITLESS` said neither.
        self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(quantity.value.position.unit or "m")))
        self.graph.add(
            (node, QUDT_SCHEMA.unit, _dsl_unit(_orientation_angle_unit(quantity.value.orientation)))
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
        # The pooled Position/Orientation relation is shared by every pose over these frames, so
        # which coordinate is *this* pose's is recorded here rather than guessed downstream.
        self._record_pose_component(node, "position", position_node)
        self._record_pose_component(node, "orientation", orientation_node)
        if pose_relation is not None:
            self.graph.add((pose_relation, RDF.type, URI_GEOM_TYPE_POSITION_REF))
            self.graph.add((pose_relation, RDF.type, URI_GEOM_TYPE_ORIENT_REF))
            self.graph.add((pose_relation, URI_GEOM_PRED_OF_POSITION, position_relation))
            self.graph.add((pose_relation, URI_GEOM_PRED_OF_ORIENT, orientation_relation))
        for coord, subobject, subspace, label in (
            (position_node, position_relation, MAP_EXT.position, "position"),
            (orientation_node, orientation_relation, MAP_EXT.orientation, "orientation"),
        ):
            # See `_emit_config_pose_quantity`'s identical note: named from `node`, not the
            # (now possibly pooled) relation.
            view_node = URIRef(f"{node}-{label}-view")
            self._emit_view(view_node)
            self.graph.add((view_node, RDF.type, MAP_EXT.PoseCoordinateView))
            self.graph.add((view_node, MAP.superobject, node))
            self.graph.add((view_node, MAP.subobject, subobject))
            self.graph.add((view_node, MAP.subspace, subspace))
            self._register_component_view(coord, label, view_node)

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
                self._add_literal_list_once(
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
            delta_basis = self._owned_uri(
                str(getattr(relative.frame, "uri", relative.frame)), quantity
            )
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
                self._add_literal_list_once(
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
                    self._add_literal_list_once(
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
        world_qtys: dict[str, WorldQuantity] | None = None,
    ) -> None:
        """Emit a Direction quantity as a unit direction coordinate in its as-seen-by frame,
        optionally initialized from a literal vector value or recomputed every cycle from the
        pose relating two frames.
        """
        as_seen_by_name = _geo_prop(quantity.props, "as-seen-by") or _geo_prop(
            quantity.props, "wrt"
        )
        if as_seen_by_name is None:
            raise ValueError(
                f"Direction quantity '{quantity.name}' needs an 'as-seen-by: <frame>' prop."
            )
        as_seen_by_node = self._owned_uri(as_seen_by_name, quantity)
        if isinstance(quantity.value, DirectionBetween):
            self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
            self.graph.add((node, RDF.type, GEOM_ENT.UnitVector))
            self._emit_direction_coordinate(node, as_seen_by_node)
            self._emit_direction_between(node, quantity, as_seen_by_name, world_qtys or {})
            return
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
        # An authored direction is itself the structural unit-vector entity (same node doubling
        # as line/plane's own along/normal direction does in `_emit_structural_primitive`).
        self.graph.add((node, RDF.type, GEOM_ENT.UnitVector))
        self._emit_direction_coordinate(node, as_seen_by_node, vector)

    def _emit_direction_between(
        self,
        node: URIRef,
        quantity: ContextQuantity,
        as_seen_by_name: str,
        world_qtys: dict[str, WorldQuantity],
    ) -> None:
        """Wire a `from <a> to <b>` direction to the pose that already relates the two frames:
        normalizing that pose's translation is the direction, recomputed every cycle.

        The pose is not minted here -- whichever chain relates the two frames is the world
        model's to walk, so the model declares the pose and this reads it.
        """
        value = quantity.value
        of_name = str(value.to_frame.uri)
        wrt_name = str(value.from_frame.uri)
        pose = self._existing_world_pose(world_qtys, of_name, wrt_name)
        if pose is None:
            raise ValueError(
                f"Direction quantity '{quantity.name}' is computed from the pose of "
                f"'{_authored_fqn(value.to_frame)}' with respect to "
                f"'{_authored_fqn(value.from_frame)}', which no `world` block declares."
            )
        pose_seen_by = _geo_prop(pose.props, "as-seen-by") or _geo_prop(pose.props, "wrt")
        if pose_seen_by != as_seen_by_name:
            raise ValueError(
                f"Direction quantity '{quantity.name}' is seen by a different frame than the "
                f"pose '{pose.name}' it is computed from; normalizing that pose would answer "
                "in the pose's frame, not this one."
            )
        op_node = self._declared_uri("compute-direction", quantity)
        self.graph.add((op_node, RDF.type, GEOM_OP.PoseToDirection))
        self.graph.add((op_node, GEOM_OP.pose, URIRef(pose.uri)))
        self.graph.add((op_node, GEOM_OP.direction, node))

    def _emit_structural_primitive(
        self,
        node: URIRef,
        quantity: ContextQuantity,
    ) -> None:
        """Emit a `line`/`plane` as an origin point (the `of` frame's) plus the unit vector
        carried by the referenced `direction` quantity.
        """
        is_plane = quantity.type == QuantityType.Plane
        frame_name = _geo_prop(quantity.props, GeometricPropKey.Of)
        vector_name = _geo_prop(
            quantity.props, GeometricPropKey.Normal if is_plane else GeometricPropKey.Along
        )
        vector_node = self._owned_uri(vector_name, quantity)
        # The referenced direction is emitted as a direction coordinate; the entity role ranges
        # over the structural unit vector, so state that type on the same node.
        self.graph.add((vector_node, RDF.type, GEOM_ENT.UnitVector))
        self.graph.add((node, RDF.type, GEOM_EXT.Plane if is_plane else GEOM_EXT.Line))
        self.graph.add(
            (node, URI_GEOM_PRED_ORIGIN, self._frame_origin(self._owned_uri(frame_name, quantity)))
        )
        self.graph.add((node, GEOM_EXT.normal if is_plane else GEOM_EXT.direction, vector_node))

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
        lerp_node = self._declared_uri(f"lerp-{quantity.name}", quantity)
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
        path_node = self._declared_uri(f"{spec_prefix}-{quantity.name}", quantity)
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
        return self._declared_uri(f"{self._path_shape(path)}-{path.name}", path)

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
            ctrl = self._controller_for_spec(spec)
            if ctrl is None:
                raise ValueError(
                    f"Profiled path constraint '{spec.name}' needs a tracking controller."
                )
            profile_qty = _resolved_context_quantity(_context_quantity(operand.profile))
            ref_node = self._emit_velocity_profile_reference(
                ctrl,
                spec,
                motion,
                None,
                self._path_along_speed_node(path),
                QuantityType.LinearVelocity,
                profile_qty,
            )
            profile_node = self._declared_uri(f"profile-{spec.name}-{ctrl.name}", ctrl)
            self.graph.add((profile_node, GEOM_OP_EXT.path, self._path_geometry_node(path)))
            self.graph.add(
                (
                    profile_node,
                    _ns_term(GEOM_OP_EXT, "path-parameter"),
                    self._declared_uri(f"{path.name}-s", path),
                )
            )
            self.graph.add((node, CSTR["reference-value"], ref_node))
            self._reference_value_index[node] = ref_node
            # A commanded speed is an equality like any other: it is never met exactly, so it
            # needs a band. Without this it reached codegen with none and was judged by exact
            # float comparison.
            self._emit_constraint_tolerance(
                node, spec, motion, None, "", None, QuantityType.LinearVelocity
            )
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
        return self._declared_uri(f"{path.name}-along-speed", path)

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
        projection_node = self._declared_uri(f"projection-{path.name}", path)
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

        parameter_node = self._declared_uri(f"{path.name}-s", path)
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
            direction_node = self._declared_uri(f"{path.name}-{term}", path)
            self.graph.add((direction_node, RDF.type, QUDT_SCHEMA.Quantity))
            self._emit_direction_coordinate(direction_node, as_seen_by)
            directions[term] = direction_node
        frame_node = self._declared_uri(f"frame-{path.name}", path)
        self.graph.add((frame_node, RDF.type, GEOM_OP_EXT.PathTangentFrame))
        self.graph.add((frame_node, GEOM_OP_EXT.path, path_node))
        self.graph.add((frame_node, _ns_term(GEOM_OP_EXT, "path-parameter"), parameter_node))
        for term, direction_node in directions.items():
            self.graph.add((frame_node, _ns_term(GEOM_OP_EXT, term), direction_node))

        # How fast the frame travels along the path: the measured twist onto that tangent.
        along_node = self._declared_uri(f"along-{path.name}", path)
        self.graph.add((along_node, RDF.type, GEOM_OP_EXT.TwistToLinearVelocityAlong))
        self.graph.add(
            (along_node, GEOM_OP["in"], URIRef(self._measured_twist_of(moved, world_qtys).uri))
        )
        self.graph.add((along_node, GEOM_OP.direction, directions["tangent"]))
        self.graph.add((along_node, _ns_term(GEOM_OP_EXT, "along-speed"), speed_node))

        # The pose the path carries there, which is what the setpoint follows.
        evaluator_node = self._declared_uri(f"evaluator-{path.name}", path)
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

    def _emit_context_ref_node(
        self, ref: ContextRef, owner: Any, suffix: str, scalar_t: Any = None
    ) -> URIRef:
        """Resolve a context reference to its value node: a subspace view, a passthrough source,
        or the referenced quantity.

        `scalar_t` types a bare literal when `owner` is not itself a typed quantity (e.g. a
        motion, for a constraint's threshold/reference); it falls back to `owner.type`.
        """
        # A literal operand names nothing, so it carries its own node: the number and the unit
        # the author wrote, typed like any other quantity the op takes.
        bare = getattr(ref, "bare", None)
        if isinstance(bare, Measure):
            return self._emit_scalar_quantity(
                self._declared_uri(suffix, owner),
                bare.value,
                _qudt_kind(scalar_t if scalar_t is not None else owner.type),
                _dsl_unit(bare.unit),
            )
        expr = getattr(ref, "expr", None)
        if expr is not None:
            return self._emit_inline_qexpr(expr, owner, suffix, scalar_t)
        quantity = _context_quantity(ref)
        if not isinstance(quantity, ContextQuantity):
            return self._owned_uri(_node_name(quantity), owner)

        quantity = _resolved_context_quantity(quantity)
        subspace_raw = getattr(ref, "subspace", None)
        axis = semantic_axis_label(getattr(ref, "axis", None))
        if subspace_raw is not None:
            subspace = str(getattr(subspace_raw, "value", subspace_raw))
            return self._emit_context_ref_view_node(quantity, subspace, axis)
        if axis is not None:
            # A bare axis on a 3-vector quantity: the same component view, with no subspace to name.
            return self._emit_context_ref_view_node(quantity, None, axis)
        if isinstance(quantity.value, ReferenceValue) and not isinstance(
            quantity.value.expr.as_op_tree(), QOpNode
        ):
            return self._emit_context_ref_node(quantity.value.source, owner, suffix, scalar_t)
        if quantity.type == ReferenceGeneratorType.Path:
            return self._reference_output_node(quantity)
        return URIRef(quantity.uri)

    def _emit_inline_qexpr(self, expr: Any, owner: Any, suffix: str, scalar_t: Any) -> URIRef:
        """A parenthesized inline expression at a `ContextRef` slot: sugar for a
        compiler-named quantity owned by the consuming declaration (constraint RHS, reference,
        tolerance, saturation bound, profile limit, solver gravity, coordinate element).
        """
        tree = expr.as_op_tree()
        inferred = _infer_expr_type(tree)
        if scalar_t is not None and not _same_scalar_dimension(scalar_t, inferred):
            raise ValueError(
                f"'{suffix}' is a {scalar_t} slot, but its expression infers {inferred}."
            )
        if not isinstance(tree, QOpNode):
            return self._emit_qexpr_leaf(tree, owner, suffix)
        root = self._owned_uri(suffix, owner)
        self._emit_qexpr(tree, owner, root)
        self.graph.add((root, RDF.type, QUDT_SCHEMA.Quantity))
        self._emit_quantity_kind(root, _qudt_kind(inferred))
        self.graph.add((root, QUDT_SCHEMA.unit, SCALAR_UNIT.get(inferred, QUDT_UNIT.UNITLESS)))
        return root

    def _emit_constraint_tolerance(
        self,
        node: URIRef,
        spec: ConstraintSpecification,
        motion: Any,
        qty: WorldQuantity | None,
        subspace: str,
        axis: str | None,
        scalar_t: Any,
    ) -> None:
        """Link a constraint's satisfaction band: its own `within`, else the model-wide
        default for the unit its error is measured in -- `position` for a metre error,
        whether one axis or three, `orientation` for a radian one.

        An equality has to end up with one -- `equal to <x>` names a single point and `on
        <path>` a single curve, and neither is ever met exactly. A gate need not: its
        admissible region has an interior, so the band only says how close to the boundary
        counts as arrived, and a model that states none asks for the boundary itself.

        A band carries the kind and unit of the value it bounds, so a whole pose -- a position
        and an orientation in one error -- cannot state one: metres and radians would share a
        number. Those are toleranced per subspace, one constraint each.
        """
        band = spec.tolerance
        owner, suffix = motion, f"{spec.name}-tolerance"
        default_kind = _TOLERANCE_DEFAULT_KIND.get(scalar_t, scalar_t)
        if band is None:
            # One node per kind: the model states the default once, so the graph shows every
            # constraint that takes it pointing at the same band.
            band = self._tolerance_defaults.get(default_kind)
            owner, suffix = self._default_ns_owner, f"default-tolerance-{scalar_t}"
        whole_pose = qty is not None and qty.type == WorldQuantityType.Pose
        if band is not None and whole_pose and axis is None and subspace == "pose":
            raise ValueError(
                f"Constraint '{spec.name}' tolerances a whole pose, whose error mixes a "
                "position and an orientation. State the band on '.position' and on "
                "'.orientation' separately, each in its own unit."
            )
        if band is None:
            if (
                isinstance(spec.expr, EqualityConstraint)
                or spec.view.on is not None
                or spec.view.moving is not None
            ):
                raise ValueError(
                    f"Equality constraint '{spec.name}' states no band. An equality is only ever "
                    "satisfied within one, so it must say which: '... within <band>', or "
                    f"declare a model-wide default for '{default_kind}' in a 'tolerances' block."
                )
            return
        self.graph.add((node, CSTR_EXT.tolerance, self._band_node(band, owner, suffix, scalar_t)))

    def _band_node(self, band: ContextRef, owner: Any, suffix: str, scalar_t: Any) -> URIRef:
        """The quantity a satisfaction band resolves to.

        An inline measure takes its kind from the value it bounds, so a band that is used
        nowhere else -- most of them -- is written where it applies instead of being declared
        as a context quantity first.
        """
        measure = getattr(band, "bare", None)
        if measure is None:
            return self._emit_context_ref_node(band, owner, suffix)
        node = self._owned_uri(suffix, owner)
        if node not in self._emitted_bands:
            self._emitted_bands.add(node)
            self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
            if scalar_t == QuantityType.Distance:
                self.graph.add((node, RDF.type, GEOM_COORD.LinearDistanceCoordinate))
            self._emit_quantity_kind(node, _qudt_kind(scalar_t))
            self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(measure.unit)))
            self.graph.add(
                (node, QUDT_SCHEMA.value, Literal(float(measure.value), datatype=XSD.double))
            )
        return node

    def _constraint_reference_node(
        self,
        ref: ContextRef,
        owner: Any,
        suffix: str,
        subspace: str,
        axis: str | None,
        scalar_t: Any = None,
    ) -> URIRef:
        """The node a constraint compares against, promoted to the position/orientation
        component view for a whole-subspace (axis-less) pose/path reference.
        """
        ref_node = self._emit_context_ref_node(ref, owner, suffix, scalar_t)
        quantity = _context_quantity(ref)
        if not isinstance(quantity, ContextQuantity) or axis is not None:
            return ref_node
        quantity = _resolved_context_quantity(quantity)
        if quantity.type in {QuantityType.Pose, ReferenceGeneratorType.Path} and subspace in {
            "position",
            "orientation",
        }:
            # The reference operand is that coordinate's component view: the pooled relation is
            # the target's too, and naming it would state measured == reference.
            return self._emit_context_ref_view_node(quantity, subspace, None)
        return ref_node

    def _constraint_context_quantity(self, spec: ConstraintSpecification) -> ContextQuantity | None:
        """The scalar context quantity a constraint's view names directly, or None."""
        quantity = getattr(spec.view, "quantity", None)
        if isinstance(quantity, ContextQuantity):
            return _resolved_context_quantity(quantity)
        return None

    def _quantityless_scalar_type(self, spec: ConstraintSpecification) -> Any:
        """The kind a constraint acts on when its view names no world quantity -- a scalar
        context quantity's declared type, or what its inline expression infers to. None when the
        view names neither, which is every constraint the world-quantity path already answers.
        """
        if getattr(spec, "view", None) is None:
            return None
        context_qty = self._constraint_context_quantity(spec)
        if context_qty is not None:
            return context_qty.type
        expr = getattr(spec.view, "expr", None)
        return _infer_expr_type(expr.as_op_tree()) if expr is not None else None

    def _constraint_quantityless_view(
        self,
        spec: ConstraintSpecification,
        motion: GuardedMotion,
        context_qty: ContextQuantity | None,
    ) -> tuple[URIRef, Any]:
        """`qty_node`/`scalar_t` for a constraint view naming no world quantity: a scalar
        context quantity, or an inline parenthesized expression owned by the constraint.
        """
        tree = None
        if context_qty is not None:
            scalar_t = context_qty.type
        else:
            tree = spec.view.expr.as_op_tree()
            scalar_t = _infer_expr_type(tree)
        if scalar_t not in DIMENSION_VECTOR and scalar_t != QuantityType.Pose:
            raise ValueError(
                f"Constraint '{spec.name}' names no known kind; select a scalar quantity."
            )
        if scalar_t == QuantityType.Pose:
            raise ValueError(
                f"Constraint '{spec.name}' names a whole pose; select '.position' or "
                "'.orientation', each with its own tolerance."
            )
        if scalar_t == QuantityType.Mass:
            raise ValueError(f"Constraint '{spec.name}': no constraint type for kind Mass.")

        if context_qty is not None:
            return URIRef(context_qty.uri), scalar_t
        if isinstance(tree, QOpNode):
            # Motion-qualified last segment: the downstream id is the last segment alone, so a
            # bare "expr" would collapse every inline-expression constraint onto one field.
            qty_node = URIRef(f"{spec.uri}/expr-{motion.name}-{spec.name}")
            self._emit_qexpr(tree, motion, qty_node)
            self.graph.add((qty_node, RDF.type, QUDT_SCHEMA.Quantity))
            self._emit_quantity_kind(qty_node, _qudt_kind(scalar_t))
            self.graph.add(
                (qty_node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(scalar_t, QUDT_UNIT.UNITLESS))
            )
        else:
            qty_node = self._emit_qexpr_leaf(tree, motion, "expr")
        return qty_node, scalar_t

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
            context_qty = self._constraint_context_quantity(spec) if qty is None else None
            if qty is None and context_qty is None and spec.view.expr is None:
                raise ValueError(f"Constraint '{spec.name}' does not resolve to a world quantity.")

            if qty is None:
                subspace = axis = None
                qty_node, scalar_t = self._constraint_quantityless_view(spec, motion, context_qty)
            else:
                subspace = _view_subspace(spec)
                axis_raw = spec.view.axis
                axis = semantic_axis_label(axis_raw)
                if (
                    qty.type == WorldQuantityType.Pose
                    and subspace == "distance"
                    and axis is None
                    and not _is_distance_view(spec)
                ):
                    raise ValueError(
                        f"Constraint '{spec.name}' must use explicit "
                        "'distance between <pose-a> and <pose-b>' syntax."
                    )
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
                    # `_view_node` emits the pose's component view and returns it: the operand
                    # is coordinate + component, not the relation every pose over these frames
                    # shares.
                    qty_node = self._view_node(spec.view, motion)
                elif (
                    qty.type == WorldQuantityType.Pose
                    and axis is None
                    and subspace
                    in {
                        "orientation",
                        "rotation",
                    }
                ):
                    qty_node = self._view_node(spec.view, motion)
                elif axis is None and (
                    (subspace == "pose" and qty.type == WorldQuantityType.Pose)
                    or qty.type == WorldQuantityType.JointPosition
                ):
                    qty_node = URIRef(qty.uri)
                else:
                    sid = (
                        _alignment_id(qty, spec)
                        if (
                            _is_alignment_view(spec)
                            or _is_incident_angle_view(spec)
                            or _is_plane_angle_view(spec)
                        )
                        else _scalar_id(qty, subspace, axis)
                    )
                    qty_node = self._owned_uri(sid, motion)
                scalar_t = _scalar_type(qty, subspace, axis)

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
                self._emit_constraint_tolerance(node, spec, motion, qty, subspace, axis, scalar_t)
                continue

            expr = spec.expr
            if isinstance(expr, EqualityConstraint):
                self.graph.add((node, RDF.type, CSTR.EqualityConstraint))
                ref_node = self._constraint_reference_node(
                    expr.reference, motion, f"{spec.name}-ref", subspace, axis, scalar_t
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
                    expr.threshold, motion, f"{spec.name}-threshold", scalar_t
                )
                self.graph.add((node, CSTR.threshold, thr_node))
            elif isinstance(expr, LessThanConstraint):
                self.graph.add((node, RDF.type, CSTR.UnilateralConstraint))
                self.graph.add((node, RDF.type, CSTR.LessThanConstraint))
                thr_node = self._emit_context_ref_node(
                    expr.threshold, motion, f"{spec.name}-threshold", scalar_t
                )
                self.graph.add((node, CSTR.threshold, thr_node))
            elif isinstance(expr, BilateralConstraint):
                self.graph.add((node, RDF.type, CSTR.BilateralConstraint))
                lo_node = self._emit_context_ref_node(
                    expr.lower, motion, f"{spec.name}-lower", scalar_t
                )
                up_node = self._emit_context_ref_node(
                    expr.upper, motion, f"{spec.name}-upper", scalar_t
                )
                self.graph.add((node, CSTR["lower-threshold"], lo_node))
                self.graph.add((node, CSTR["upper-threshold"], up_node))
            elif isinstance(expr, OutsideConstraint):
                self.graph.add((node, RDF.type, CSTR_EXT.OutsideConstraint))
                lo_node = self._emit_context_ref_node(
                    expr.lower, motion, f"{spec.name}-lower", scalar_t
                )
                up_node = self._emit_context_ref_node(
                    expr.upper, motion, f"{spec.name}-upper", scalar_t
                )
                self.graph.add((node, CSTR["lower-threshold"], lo_node))
                self.graph.add((node, CSTR["upper-threshold"], up_node))

            self._emit_constraint_tolerance(node, spec, motion, qty, subspace, axis, scalar_t)

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

        out_node = self._declared_uri(f"{spec.name}-{ctrl.name}-admit-ref", ctrl)
        self._add_quantity(out_node, scalar_t)

        op_node = self._declared_uri(f"admit-{spec.name}-{ctrl.name}", ctrl)
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
        self.graph.add((op_node, ALGO_EXT["maximum-velocity"], max_velocity_node))
        # The excursion bound is a saturation on how far the yield travels.
        max_excursion_node = URIRef(f"{op_node}-max-excursion")
        self._emit_scalar_quantity(
            max_excursion_node,
            float(spec_val.max_excursion),
            URI_QUDT_QK_LENGTH,
            _dsl_unit(spec_val.max_excursion_unit or "m"),
        )
        self.graph.add((op_node, ALGO_EXT["maximum-absolute-value"], max_excursion_node))
        # The deadband is an outside-band on the input force: the same pair of thresholds the
        # detection constraint states, applied to the signal instead of monitored.
        deadband_unit = _dsl_unit(spec_val.deadband_unit or "N")
        for term, value in (
            ("lower-threshold", -float(spec_val.deadband)),
            ("upper-threshold", float(spec_val.deadband)),
        ):
            band_node = URIRef(f"{op_node}-{term}")
            self._emit_scalar_quantity(band_node, value, QUDT_QKIND.Force, deadband_unit)
            self.graph.add((op_node, CSTR[term], band_node))
        self.graph.add((op_node, ALGO_EXT.out, out_node))
        return out_node

    def _emit_velocity_profile_reference(
        self,
        ctrl: ControllerEntry,
        spec: ConstraintSpecification,
        motion: GuardedMotion,
        goal_node: URIRef | None,
        measured_node: URIRef | None,
        scalar_t: Any,
        profile_qty: ContextQuantity | None = None,
    ) -> URIRef:
        """Emit a velocity-profile reference-generating op (goal + measured -> profiled velocity)
        for a profiled controller. Returns the reference-value node.
        """
        if measured_node is None:
            raise ValueError(f"Profiled controller '{ctrl.name}' needs a measured quantity.")
        profile_qty = profile_qty or _context_quantity(ctrl.params.profile)
        if not isinstance(profile_qty, ContextQuantity):
            raise ValueError(f"Controller '{ctrl.name}' has an unresolved velocity profile.")
        profile_qty = _resolved_context_quantity(profile_qty)
        if not isinstance(profile_qty.value, ProfileSpec):
            raise ValueError(
                f"Controller '{ctrl.name}' profile '{profile_qty.name}' is not a Profile."
            )

        # The profile emits a setpoint for the quantity it drives, so the output carries
        # that quantity's kind, not a velocity.
        out_node = self._declared_uri(f"{spec.name}-{ctrl.name}-profile-ref", ctrl)
        self._add_quantity(out_node, scalar_t)

        op_node = self._declared_uri(f"profile-{spec.name}-{ctrl.name}", ctrl)
        self.graph.add((op_node, RDF.type, ALGO_EXT.VelocityProfile))
        self.graph.add((op_node, RDF.type, CSTR_HDL_EXT.SetpointGenerator))
        # Where it is driving to. The value it starts from is the constraint's own
        # quantity, so the profile does not restate it.
        max_velocity_node = self._emit_profile_limit(
            profile_qty.value.max_velocity,
            profile_qty,
            "max-velocity",
            QuantityType.LinearVelocity,
        )
        self.graph.add((op_node, _ns_term(ALGO_EXT, "target"), goal_node or max_velocity_node))
        self.graph.add(
            (
                op_node,
                _ns_term(ALGO_EXT, "maximum-velocity"),
                max_velocity_node,
            )
        )
        self.graph.add(
            (
                op_node,
                _ns_term(ALGO_EXT, "maximum-acceleration"),
                self._emit_profile_limit(
                    profile_qty.value.max_acceleration,
                    profile_qty,
                    "max-acceleration",
                    QuantityType.LinearAcceleration,
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
                    self._emit_profile_limit(
                        profile_qty.value.max_jerk,
                        profile_qty,
                        "max-jerk",
                        QuantityType.LinearJerk,
                    ),
                )
            )
        self.graph.add(
            (op_node, ALGO_EXT.shape, _ns_term(ALGO_EXT, profile_qty.value.shape or "trapezoidal"))
        )
        self.graph.add((op_node, ALGO_EXT.out, out_node))
        return out_node

    def _emit_profile_limit(
        self, ref: ContextRef, owner: ContextQuantity, suffix: str, quantity_type: QuantityType
    ) -> URIRef:
        """Emit an inline profile limit or resolve its named quantity."""
        measure = getattr(ref, "bare", None)
        if not isinstance(measure, Measure):
            return self._emit_context_ref_node(ref, owner, suffix)
        return self._emit_scalar_quantity(
            self._declared_uri(f"{owner.name}-{suffix}", owner),
            float(measure.value),
            QUDT_KIND_BY_QUANTITY_TYPE.get(quantity_type, QUDT_QKIND[quantity_type.value]),
            _dsl_unit(measure.unit),
        )

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
            band = spec.tolerance or self._tolerance_defaults.get(QuantityType.Duration)
            if band is None:
                raise ValueError(
                    f"Elapsed equality '{spec.name}' states no band. A sampled clock never "
                    "lands exactly on the reference, so it must say how close counts: "
                    "'... equal to <t> within <band>'."
                )
            tol_node = self._emit_duration_threshold_node(band, motion, f"{spec.name}-tolerance")
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
        self._emit_scalar_quantity(node, value.value, NS_MM_QUDT_QTY["Time"], _dsl_unit(value.unit))

    def _emit_sampled_quantity(self, node: URIRef, quantity: ContextQuantity) -> None:
        """Emit a `sample <distribution> <unit>` scalar: the typing, the kind, the unit and the
        from-distribution link. The number itself is drawn per generation downstream, so no
        `qudt:value` is written here.
        """
        if quantity.type == QuantityType.Duration:
            self.graph.add((node, RDF.type, TIME.Duration))
            qkind = NS_MM_QUDT_QTY["Time"]
        else:
            qkind = QUDT_KIND_BY_QUANTITY_TYPE.get(quantity.type) or QUDT_QKIND[quantity.type]
        self.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        self._emit_quantity_kind(node, qkind)
        self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(quantity.value.unit)))
        add_sampled_quantity(
            self.graph, node, DistributionRef(parent=None, distribution=quantity.value.distribution)
        )

    def _section_expression(self, motion: GuardedMotion, phase: str):
        """A when/until section's expression node and the members it holds.

        The node is None when the section's logic mints none and the members link flat. Both the
        motion's own links and any whole-section monitor's guard resolve through here, so a
        monitor can never point at a node the section did not mint.
        """
        section = motion.when if phase == "when" else motion.until
        members = [
            item
            for item in section.constraints
            if not isinstance(item, ConstraintGroup) and not _resolved_spec(item).disabled
        ]
        node_type = _section_expression_type(getattr(section, "logic", None), len(members))
        if node_type is None:
            return None, node_type, members
        name = f"motion-{motion.name}-{phase}-{_EXPRESSION_SUFFIX[node_type]}"

        return self._owned_uri(name, motion), node_type, members

    def _emit_section_constraints(
        self, motion: GuardedMotion, motion_node: URIRef, phase: str, predicate: URIRef
    ) -> None:
        """Link one section's constraints to the motion, under its expression node when it has one."""
        node, node_type, members = self._section_expression(motion, phase)
        if node is None:
            for item in members:
                self.graph.add((motion_node, predicate, URIRef(_resolved_spec(item).uri)))
            return
        self.graph.add((node, RDF.type, node_type))
        self.graph.add((motion_node, predicate, node))
        for item in members:
            self.graph.add((node, CSTR_EXT["has-constraint"], URIRef(_resolved_spec(item).uri)))

    def _emit_detect_acts(self, motion: GuardedMotion) -> None:
        """A detect act is its own ros:Action node -- the channel the goal is sent on, the action
        type it carries, and the scene objects it is asked to locate -- plus the status slot its
        outcome lands in, and any until item comparing that status.
        """
        for act in getattr(motion, "detects", ()):
            act_node = URIRef(act.uri)
            self.graph.add((act_node, RDF.type, ROS.Action))
            self.graph.add((act_node, ROS["channel-name"], Literal(act.action.channel_name)))
            self.graph.add((act_node, ROS["type-name"], Literal(act.action.type_name)))
            if act.action.pose_path:
                self.graph.add((act_node, ROS["field-path"], Literal(act.action.pose_path)))
            for target in act.targets:
                self.graph.add(
                    (
                        act_node,
                        SOSA.hasFeatureOfInterest,
                        URIRef(str(getattr(target.ref, "uri", target.ref))),
                    )
                )
            self.graph.add((URIRef(act.status_uri), PROV.wasDerivedFrom, act_node))

        for item in _flatten_constraint_items(motion.until.constraints):
            if isinstance(item, GoalStatusConstraint):
                self._emit_goal_status_constraint(item)

    def _emit_goal_status_constraint(self, spec: GoalStatusConstraint) -> None:
        """A goal-status until item: an equality whose quantity is the act's status slot and
        whose reference is the GoalStatus constant the model named.
        """
        node = URIRef(spec.uri)
        self.graph.add((node, RDF.type, CSTR.Constraint))
        self.graph.add((node, RDF.type, CSTR.EqualityConstraint))
        self.graph.add((node, CSTR.quantity, URIRef(spec.act.status_uri)))
        ref_node = URIRef(f"{spec.uri}-reference")
        self.graph.add((ref_node, RDF.value, Literal(spec.status_constant)))
        self.graph.add((node, CSTR["reference-value"], ref_node))

    def _emit_motion_spec(self, motion: GuardedMotion) -> None:
        """Emit the guarded-motion node linking its when/while/until constraints (with expression
        nodes for explicit `any`/`all` logic) and any path.
        """
        motion_node = self._owned_uri(f"motion-{motion.name}", motion)
        self.graph.add((motion_node, RDF.type, MOT.GuardedMotion))
        self.graph.add((motion_node, SDO.name, Literal(motion.name)))
        # textX leaves an unmatched optional STRING as '', not None.
        if motion.description:
            self.graph.add((motion_node, SDO.description, Literal(motion.description)))
        self._emit_section_constraints(motion, motion_node, "when", MOT.when)
        for item in motion.while_.constraints:
            spec = _resolved_spec(item)
            if not spec.disabled:
                self.graph.add((motion_node, MOT["while"], URIRef(spec.uri)))
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
        self._emit_section_constraints(motion, motion_node, "until", MOT.until)

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
                and self._resolve_constraint_quantity(spec, world_qtys) is not None
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
            self.graph.add((distance_node, RDF.type, GEOM_COORD.DistanceReference))
            self.graph.add(
                (
                    distance_node,
                    GEOM_COORD.of,
                    self._linear_distance_relation(
                        plan.relation_a, plan.relation_b, GEOM_REL.PointToPointDistance
                    ),
                )
            )
            self._emit_distance_operand_selection(distance_node, plan)

        seen_alignment_ops = self._emitted_alignment_ops
        for spec in constraints:
            if not _is_alignment_view(spec):
                continue
            qty = self._resolve_constraint_quantity(spec, world_qtys)
            alignment_id = _alignment_id(qty, spec)
            if alignment_id in seen_alignment_ops:
                continue
            seen_alignment_ops.add(alignment_id)

            plan = self._alignment_plan(spec, world_qtys)
            reference_frame = self._owned_uri(_geo_prop(plan.target.props, "wrt"), motion)

            rotated_node = self._owned_uri(f"{alignment_id}-rotated", motion)
            self._emit_direction_coordinate(rotated_node, reference_frame)
            rotate_op = self._owned_uri(f"compute-{alignment_id}-rotated", motion)
            self.graph.add((rotate_op, RDF.type, GEOM_OP.RotateDirectionDistalToProximalWithPose))
            self.graph.add((rotate_op, GEOM_OP.pose, URIRef(plan.target.uri)))
            self.graph.add((rotate_op, GEOM_OP["from"], URIRef(plan.moving.uri)))
            self.graph.add((rotate_op, GEOM_OP.to, rotated_node))

            theta_node = self._owned_uri(alignment_id, motion)
            self._add_quantity(theta_node, QuantityType.Angle)
            # Base-alignment (fixed-axis solver rows) is decided from this frame downstream.
            self.graph.add((theta_node, GEOM_COORD["as-seen-by"], reference_frame))
            self.graph.add(
                (
                    theta_node,
                    GEOM_COORD.of,
                    self._angular_distance_relation(
                        plan.relation_a,
                        plan.relation_b,
                        GEOM_REL_EXT.DirectionDirectionAngularDistance,
                    ),
                )
            )
            angle_op = self._owned_uri(f"compute-{alignment_id}", motion)
            self.graph.add((angle_op, RDF.type, GEOM_OP.PlanarAngleFromDirections))
            self.graph.add((angle_op, GEOM_OP["from-directions"], rotated_node))
            self.graph.add((angle_op, GEOM_OP["from-directions"], URIRef(plan.reference.uri)))
            self.graph.add((angle_op, GEOM_OP.angle, theta_node))

            if _alignment_is_pointwise(spec):
                # Point target (2 DOF): the exact cross-product error, unchanged from before
                # this plan.
                vector_node = self._owned_uri(f"{alignment_id}-error", motion)
                self._add_quantity(vector_node, QuantityType.FreeVector)
                self.graph.add((vector_node, RDF.type, GEOM_COORD.VectorXYZ))
                self.graph.remove((vector_node, QUDT_SCHEMA.unit, None))
                self.graph.add((vector_node, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
                self.graph.add((vector_node, GEOM_COORD["as-seen-by"], reference_frame))
                vector_op = self._owned_uri(f"compute-{alignment_id}-error", motion)
                self.graph.add((vector_op, RDF.type, GEOM_OP_EXT.RotationVectorFromDirections))
                self.graph.add((vector_op, GEOM_OP.in1, rotated_node))
                self.graph.add((vector_op, GEOM_OP.in2, URIRef(plan.reference.uri)))
                self.graph.add((vector_op, GEOM_OP.out, vector_node))
            else:
                # Cone target (1 DOF): the row is theta's runtime gradient, not a rotation
                # vector -- same shared theta_node/rotated_node, no second scalar minted.
                gradient_node = self._owned_uri(_gradient_scalar_id(qty, spec), motion)
                self._emit_direction_coordinate(gradient_node, reference_frame)
                grad_op = self._owned_uri(f"compute-{alignment_id}-gradient", motion)
                self.graph.add((grad_op, RDF.type, GEOM_OP_EXT.AngleGradientFromDirections))
                self.graph.add((grad_op, GEOM_OP.in1, rotated_node))
                self.graph.add((grad_op, GEOM_OP.in2, URIRef(plan.reference.uri)))
                self.graph.add((grad_op, GEOM_OP_EXT.gradient, gradient_node))

        # Table IIa (plan 08): point-plane/point-line/point-on-line distance, and line-line
        # distance/projection. One operator node per expression, wired to its already-resolved
        # in1/in2/direction (or pose-difference) operands.
        seen_geometric_ops = self._emitted_geometric_distance_ops
        for spec in constraints:
            if not (_is_geometric_distance_view(spec) or _is_projection_view(spec)):
                continue
            qty = self._resolve_constraint_quantity(spec, world_qtys)
            subspace = _view_subspace(spec)
            distance_id = _scalar_id(qty, subspace, None)
            if distance_id in seen_geometric_ops:
                continue
            seen_geometric_ops.add(distance_id)

            plan = self._geometric_distance_plan(spec, world_qtys)
            distance_node = self._owned_uri(distance_id, motion)
            self._add_quantity(distance_node, QuantityType.Distance)
            relation_name = GEOMETRIC_DISTANCE_RELATION[plan.op_type]
            # The body-line case is comp-rob2b's own PointLineCollinearity relation; the rest
            # are geom-rel-ext distance relations.
            relation_type = (
                NS_MM_GEOM_REL[relation_name]
                if plan.op_type == "PointBodyLineToLinearDistance"
                else GEOM_REL_EXT[relation_name]
            )
            self.graph.add(
                (
                    distance_node,
                    GEOM_COORD.of,
                    self._linear_distance_relation(plan.relation_a, plan.relation_b, relation_type),
                )
            )

            gradient_node = self._owned_uri(_gradient_scalar_id(qty, spec), motion)
            self._emit_direction_coordinate(
                gradient_node, self._owned_uri(plan.gradient_frame, motion)
            )

            op_node = self._owned_uri(f"compute-{distance_id}", motion)
            self.graph.add((op_node, RDF.type, GEOM_OP_EXT[plan.op_type]))
            self.graph.add((op_node, GEOM_OP.in1, URIRef(plan.in1)))
            self.graph.add((op_node, GEOM_OP.in2, URIRef(plan.in2)))
            if plan.direction is not None:
                self.graph.add((op_node, GEOM_OP.direction, URIRef(plan.direction)))
            if plan.pose is not None:
                self.graph.add((op_node, GEOM_OP.pose, URIRef(plan.pose)))
            self.graph.add((op_node, GEOM_OP.distance, distance_node))
            self.graph.add((op_node, GEOM_OP_EXT.gradient, gradient_node))
            if plan.op_type == "PointBodyLineToLinearDistance":
                # Tilting a body-fixed line sweeps it across the point: the distance rate has
                # an angular term besides the linear one, carried as its own direction.
                moment_node = self._owned_uri(f"{_gradient_scalar_id(qty, spec)}-moment", motion)
                self._emit_direction_coordinate(
                    moment_node, self._owned_uri(plan.gradient_frame, motion)
                )
                self.graph.add((op_node, GEOM_OP_EXT["gradient-moment"], moment_node))

        for spec in constraints:
            if not _is_incident_angle_view(spec):
                continue
            qty = self._resolve_constraint_quantity(spec, world_qtys)
            alignment_id = _alignment_id(qty, spec)
            if alignment_id in seen_alignment_ops:
                continue
            seen_alignment_ops.add(alignment_id)

            plan = self._incident_angle_plan(spec, world_qtys)
            reference_frame = self._owned_uri(_geo_prop(plan.target.props, "wrt"), motion)

            rotated_node = self._owned_uri(f"{alignment_id}-rotated", motion)
            self._emit_direction_coordinate(rotated_node, reference_frame)
            rotate_op = self._owned_uri(f"compute-{alignment_id}-rotated", motion)
            self.graph.add((rotate_op, RDF.type, GEOM_OP.RotateDirectionDistalToProximalWithPose))
            self.graph.add((rotate_op, GEOM_OP.pose, URIRef(plan.target.uri)))
            self.graph.add((rotate_op, GEOM_OP["from"], URIRef(plan.moving.uri)))
            self.graph.add((rotate_op, GEOM_OP.to, rotated_node))

            theta_node = self._owned_uri(alignment_id, motion)
            self._add_quantity(theta_node, QuantityType.Angle)
            self.graph.add((theta_node, GEOM_COORD["as-seen-by"], reference_frame))
            self.graph.add(
                (
                    theta_node,
                    GEOM_COORD.of,
                    self._angular_distance_relation(
                        plan.relation_a, plan.relation_b, GEOM_REL_EXT.DirectionPlaneAngularDistance
                    ),
                )
            )

            gradient_node = self._owned_uri(_gradient_scalar_id(qty, spec), motion)
            self._emit_direction_coordinate(gradient_node, reference_frame)

            angle_op = self._owned_uri(f"compute-{alignment_id}", motion)
            self.graph.add((angle_op, RDF.type, GEOM_OP_EXT.DirectionPlaneToAngularDistance))
            self.graph.add((angle_op, GEOM_OP.in1, rotated_node))
            self.graph.add((angle_op, GEOM_OP.in2, URIRef(plan.reference.uri)))
            self.graph.add((angle_op, GEOM_OP.angle, theta_node))
            self.graph.add((angle_op, GEOM_OP_EXT.gradient, gradient_node))

        for spec in constraints:
            if not _is_plane_angle_view(spec):
                continue
            qty = self._resolve_constraint_quantity(spec, world_qtys)
            alignment_id = _alignment_id(qty, spec)
            if alignment_id in seen_alignment_ops:
                continue
            seen_alignment_ops.add(alignment_id)

            plan = self._plane_angle_plan(spec, world_qtys)
            reference_frame = self._owned_uri(_geo_prop(plan.target.props, "wrt"), motion)

            rotated_node = self._owned_uri(f"{alignment_id}-rotated", motion)
            self._emit_direction_coordinate(rotated_node, reference_frame)
            rotate_op = self._owned_uri(f"compute-{alignment_id}-rotated", motion)
            self.graph.add((rotate_op, RDF.type, GEOM_OP.RotateDirectionDistalToProximalWithPose))
            self.graph.add((rotate_op, GEOM_OP.pose, URIRef(plan.target.uri)))
            self.graph.add((rotate_op, GEOM_OP["from"], URIRef(plan.moving.uri)))
            self.graph.add((rotate_op, GEOM_OP.to, rotated_node))

            theta_node = self._owned_uri(alignment_id, motion)
            self._add_quantity(theta_node, QuantityType.Angle)
            self.graph.add((theta_node, GEOM_COORD["as-seen-by"], reference_frame))
            self.graph.add(
                (
                    theta_node,
                    GEOM_COORD.of,
                    self._angular_distance_relation(
                        plan.relation_a, plan.relation_b, GEOM_REL_EXT.PlanePlaneAngularDistance
                    ),
                )
            )
            angle_op = self._owned_uri(f"compute-{alignment_id}", motion)
            self.graph.add((angle_op, RDF.type, GEOM_OP.PlanarAngleFromDirections))
            self.graph.add((angle_op, GEOM_OP["from-directions"], rotated_node))
            self.graph.add((angle_op, GEOM_OP["from-directions"], URIRef(plan.reference.uri)))
            self.graph.add((angle_op, GEOM_OP.angle, theta_node))

            # from-directions is multi-valued/unordered on angle_op; the scalar above is
            # symmetric so that is safe, but gradient normalize(n2 x n1) is not -- it needs
            # AngleGradientFromDirections' ordered in1/in2, never riding on angle_op.
            gradient_node = self._owned_uri(_gradient_scalar_id(qty, spec), motion)
            self._emit_direction_coordinate(gradient_node, reference_frame)
            grad_op = self._owned_uri(f"compute-{alignment_id}-gradient", motion)
            self.graph.add((grad_op, RDF.type, GEOM_OP_EXT.AngleGradientFromDirections))
            self.graph.add((grad_op, GEOM_OP.in1, rotated_node))
            self.graph.add((grad_op, GEOM_OP.in2, URIRef(plan.reference.uri)))
            self.graph.add((grad_op, GEOM_OP_EXT.gradient, gradient_node))

    def _emit_controller_base(
        self, ctrl_node: URIRef, ctrl: ControllerEntry, command: Any = None
    ) -> None:
        """Emit a controller's type and gains: PID (kp/ki/kd, optional decay), Impedance
        (stiffness/damping, optional integral gain), or feed-forward.
        """
        self.graph.add((ctrl_node, RDF.type, CSTR_HDL.Controller))
        # A derived moment must be visible downstream; everything else states only what was
        # authored, so ids and existing force commands stay as they are.
        command_type = ctrl.command_type
        if command_type is None and command is not None and command.is_moment_command:
            command_type = command.command_type
        if command_type is not None:
            self.graph.add((ctrl_node, APP["command-type"], Literal(command_type.value)))
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

    def _emit_angle_normalization(self, owner_node: URIRef, angle_range, owner, name: str) -> None:
        """The interval an angle is normalized into, as a turn's worth of authored bounds.

        Not a Saturation: a value outside the interval is moved onto it by whole turns, not
        clamped to its nearest edge.
        """
        node = self._owned_uri(name, owner)
        self.graph.add((node, RDF.type, ALGO_EXT.AngularNormalization))
        scale = math.pi / 180.0 if getattr(angle_range, "unit", "rad") == "deg" else 1.0
        for edge, value in (
            ("lower", _angle_bound(angle_range.lower) * scale),
            ("upper", _angle_bound(angle_range.upper) * scale),
        ):
            predicate = ALGO_EXT[f"{edge}-bound"]
            bound = self._owned_uri(f"{name}-{edge}", owner)
            self.graph.add((bound, RDF.type, QUDT_SCHEMA.Quantity))
            self.graph.add((bound, QUDT_SCHEMA.value, Literal(value, datatype=XSD.double)))
            self.graph.add((bound, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
            self.graph.add((bound, QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND.Angle))
            self.graph.add((node, predicate, bound))
        self.graph.add((owner_node, ALGO_EXT["normalization"], node))

    def _emit_controller_limits(
        self,
        controller_node: URIRef,
        controller: ControllerEntry,
        command,
        handler: ConstraintHandler,
    ) -> None:
        """Emit authored controller limits without choosing a solver representation."""
        if controller.params.output_saturation is not None:
            output = self._declared_uri(f"output-{controller.name}", controller)
            self._add_quantity(output, command.command_type)
            self._emit_saturation(
                controller_node,
                self._declared_uri(f"sat-output-{controller.name}", controller),
                controller.params.output_saturation,
                output,
                output,
                controller,
            )
        error_normalization = getattr(controller.params, "error_normalization", None)
        if error_normalization is not None:
            self._emit_angle_normalization(
                controller_node, error_normalization, controller, f"err-norm-{controller.name}"
            )
        if controller.params.integral_saturation is not None:
            integral = self._declared_uri(f"integral-state-{controller.name}", controller)
            self.graph.add((integral, RDF.type, QUDT_SCHEMA.Quantity))
            self._emit_saturation(
                controller_node,
                self._declared_uri(f"sat-integral-{controller.name}", controller),
                controller.params.integral_saturation,
                integral,
                integral,
                controller,
            )

    def _constraint_error_node(
        self,
        spec: ConstraintSpecification,
        motion: GuardedMotion,
        world_qtys: dict[str, WorldQuantity],
        seen_error_ids: set[str],
        error_id_by_constraint: dict[str, str],
        owner: str,
    ) -> URIRef:
        """The error signal one constraint writes, minting its quantity the first time.

        Raises:
            ValueError: the constraint's view resolves to nothing measurable.
        """
        if isinstance(spec, GoalStatusConstraint):
            return URIRef(spec.act.status_uri)
        if getattr(spec.view, "is_elapsed", False):
            return self._elapsed_quantity_node(spec, motion)
        along_path = self._along_path_scalar(spec)
        qty = self._resolve_constraint_quantity(spec, world_qtys)
        derived_t = (
            self._quantityless_scalar_type(spec) if qty is None and along_path is None else None
        )
        if qty is None and along_path is None and derived_t is None:
            raise ValueError(f"{owner} constraint '{spec.name}' does not resolve to a quantity.")
        scalar_t = (
            along_path[1]
            if along_path
            else derived_t
            if derived_t is not None
            else _scalar_type(qty, _view_subspace(spec), semantic_axis_label(spec.view.axis))
        )
        error_id = error_id_by_constraint.get(spec.uri, f"{_evaluator_id(spec)}-err")
        error_node = self._owned_uri(error_id, spec.parent)
        if error_id in seen_error_ids:
            return error_node
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
                    (error_node, GEOM_REL["with-respect-to"], self._owned_uri(wrt_v, qty))
                )
                self.graph.add((error_node, GEOM_COORD["as-seen-by"], self._owned_uri(wrt_v, qty)))

        return error_node

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

    def _emit_ros_publication(
        self, monitor: Any, monitor_node: URIRef, watched: list[URIRef]
    ) -> None:
        """Name the channel a monitor publishes on, the message it carries, and every field
        assignment it authored, each under the condition its state block holds.
        """
        answer = monitor.answer
        if answer is not None:
            self._emit_ros_answer(monitor, monitor_node, watched, *answer)
        occurrence = monitor.occurrence_topic
        if occurrence is not None:
            # An announced event is the whole payload: each member is one event the monitor
            # publishes, carrying no value of its own to distinguish it from a field row.
            self.graph.add((monitor_node, RDF.type, ROS.Topic))
            self.graph.add((monitor_node, ROS["channel-name"], Literal(occurrence.channel_name)))
            self.graph.add((monitor_node, ROS["type-name"], Literal(occurrence.type_name)))
            for event in monitor.announced_events:
                self.graph.add((monitor_node, RDFS.member, URIRef(event.uri)))
            return
        topic = monitor.topic
        if topic is None:
            return
        self.graph.add((monitor_node, RDF.type, ROS.Topic))
        self.graph.add((monitor_node, ROS["channel-name"], Literal(topic.channel_name)))
        self.graph.add((monitor_node, ROS["type-name"], Literal(topic.type_name)))
        row_index = 0
        for block, action in monitor.actions("publish"):
            if action.rate is not None:
                rate = URIRef(f"{monitor.uri}.rate")
                self._emit_scalar_quantity(
                    rate,
                    float(action.rate.value),
                    NS_MM_QUDT_QTY["Frequency"],
                    _dsl_unit(action.rate.unit),
                )
                self.graph.add((monitor_node, SENSORS["update-rate"], rate))
            # A satisfied row holds under the watched constraint; a violated row is the
            # otherwise, and states no condition at all.
            conditions = watched if block.state == "satisfied" else []
            for path, value in action.assignments:
                row = URIRef(f"{monitor.uri}.f{row_index}")
                row_index += 1
                self.graph.add((monitor_node, RDFS.member, row))
                self.graph.add((row, ROS["field-path"], Literal(path)))
                self.graph.add((row, RDF.value, Literal(value)))
                for condition in conditions:
                    self.graph.add((row, CSTR_EXT["has-constraint"], condition))

    def _emit_ros_answer(
        self,
        monitor: Any,
        monitor_node: URIRef,
        watched: list[URIRef],
        block: Any,
        action: Any,
    ) -> None:
        """The goal this monitor's state answers: the action it is served on, the status it
        answers with, and the result fields it states.

        The monitor is where the run finishes, so the answer is stated here rather than at the
        action, which knows only that goals arrive. It is a member of the monitor rather than the
        monitor itself, so the same monitor may also publish: one node states one message. Its
        own outcome member carries the answer's status and, when the answering state is the
        satisfied one, the constraint that state holds under; the remaining members are the
        result's field rows.
        """
        server = action.server
        answer_node = URIRef(f"{monitor.uri}.answer")
        self.graph.add((monitor_node, RDFS.member, answer_node))
        self.graph.add((answer_node, RDF.type, ROS.Action))
        self.graph.add((answer_node, ROS["channel-name"], Literal(server.channel_name)))
        self.graph.add((answer_node, ROS["type-name"], Literal(server.type_name)))
        outcome = URIRef(f"{answer_node}.outcome")
        self.graph.add((answer_node, RDFS.member, outcome))
        self.graph.add((outcome, RDF.value, Literal(f"STATUS_{action.outcome.upper()}")))
        for condition in watched if block.state == "satisfied" else []:
            self.graph.add((outcome, CSTR_EXT["has-constraint"], condition))
        for row_index, (path, value) in enumerate(action.assignments):
            row = URIRef(f"{answer_node}.f{row_index}")
            self.graph.add((answer_node, RDFS.member, row))
            self.graph.add((row, ROS["field-path"], Literal(path)))
            self.graph.add((row, RDF.value, Literal(value)))

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
        # A motion is otherwise bound to a state by the event that leaves it, which a motion
        # meant to keep running never fires.
        if handler.runs_in is not None:
            self.graph.add(
                (handler_node, CSTR_HDL_EXT["runs-in-state"], URIRef(handler.runs_in.uri))
            )
        event_loop_node = URIRef(f"{handler.uri}.event-loop")

        seen_error_ids: set[str] = set()
        seen_eval_ids: set[str] = set()
        # A constraint has one error signal. A monitor on a constraint a controller already drives
        # must read that signal: minting a second quantity leaves it with nothing writing it.
        error_id_by_constraint: dict[str, str] = {}
        # Constraints a controller drives or a monitor watches. The sweep at the end of this
        # method evaluates the ones neither claims: a pose command claims its constraint without
        # an evaluator of its own, since the pose-difference machinery produces its error.
        claimed_constraints: set[str] = set()

        for controller_order, ctrl_item in enumerate(getattr(handler, "controllers", [])):
            ctrl = ctrl_item.ref.controller if hasattr(ctrl_item, "ref") else ctrl_item
            cref = ctrl.params.constraint
            spec = cref.constraint if hasattr(cref, "constraint") else None
            if spec is None:
                continue
            if spec.disabled:
                continue
            claimed_constraints.add(spec.uri)

            along_path = self._along_path_scalar(spec)
            qty = self._resolve_constraint_quantity(spec, world_qtys)
            derived_t = (
                self._quantityless_scalar_type(spec) if qty is None and along_path is None else None
            )
            if qty is None and along_path is None and derived_t is None:
                raise ValueError(
                    f"Controller '{ctrl.name}' constraint '{spec.name}' does not resolve to a world quantity."
                )
            subspace = None if derived_t is not None else _view_subspace(spec)
            axis_raw = spec.view.axis
            axis = semantic_axis_label(axis_raw)
            shared = spec in shared_spec_ids
            scalar_t = (
                along_path[1]
                if along_path
                else derived_t
                if derived_t is not None
                else (_scalar_type(qty, subspace, axis) if qty else subspace)
            )
            command = controller_command_record(ctrl)
            _validate_command_subspace(ctrl, spec, command)

            authored_ctrl_node = URIRef(ctrl.uri)
            self._emit_controller_base(authored_ctrl_node, ctrl, command)
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
                        self._solver_node(handler, solver),
                    )
                )

            self._emit_controller_limits(authored_ctrl_node, ctrl, command, handler)

            controller_error_id: str | None = None
            evaluator_error_id: str | None = None
            if qty is not None or along_path is not None or derived_t is not None:
                # A constraint on a context quantity or an expression names no scalar view, so
                # its error is named after the evaluator, which is already motion-qualified.
                #
                # An alignment is named by `_alignment_id`, not by the scalar view: the view is
                # only the carrier pose and the word `alignment`, so two `angle between` rows on
                # one pose in one motion would answer to the same error and the second would
                # overwrite the first -- both controllers then driving one number. The operands
                # and the cone are what tell the two angles apart, and that is what that id
                # carries.
                sid = (
                    along_path[0]
                    if along_path
                    else _evaluator_id(spec)
                    if qty is None
                    else _alignment_id(qty, spec)
                    if subspace == "alignment"
                    else _scalar_id(qty, subspace, axis)
                )
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
                error_id_by_constraint[spec.uri] = evaluator_error_id
                self._emit_error_evaluator(
                    handler_node,
                    spec,
                    self._owned_uri(evaluator_error_id, spec.parent),
                    seen_eval_ids,
                )

        for mon in getattr(handler, "monitors", []):
            cref = mon.constraint
            trigger = mon.trigger
            if mon.fallback is not None and trigger is None:
                raise ValueError(
                    f"Monitor '{mon.name}' holds a fallback motion but triggers no event; "
                    "the fallback is only taken on the edge that fires."
                )
            is_event = trigger is not None
            signal_kind = "event" if is_event else "flag" if mon.flag else "publish-only"
            signal_node = (
                URIRef(mon.event.uri)
                if is_event
                else URIRef(f"{mon.uri}.{mon.flag}")
                if mon.flag
                else None
            )
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
                    section_node, _, _ = self._section_expression(
                        cref.motion, "when" if is_when_ref else "until"
                    )
                    guard_nodes = (
                        [section_node]
                        if section_node is not None
                        else [URIRef(spec.uri) for spec in section_specs]
                    )
                component_error_nodes: list[URIRef] = []
                claimed_constraints.update(spec.uri for spec in section_specs)
                for spec in section_specs:
                    if isinstance(spec, GoalStatusConstraint):
                        error_node = URIRef(spec.act.status_uri)
                        component_error_nodes.append(error_node)
                        self._emit_error_evaluator(handler_node, spec, error_node, seen_eval_ids)
                        continue
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
                    derived_t = self._quantityless_scalar_type(spec) if qty is None else None
                    if qty is None and derived_t is None:
                        raise ValueError(
                            f"Aggregate monitor '{mon.name}' constraint '{spec.name}' does not resolve to a world quantity."
                        )
                    axis_raw = spec.view.axis
                    axis = semantic_axis_label(axis_raw)
                    scalar_t = (
                        derived_t if qty is None else _scalar_type(qty, _view_subspace(spec), axis)
                    )
                    error_id = error_id_by_constraint.get(spec.uri, f"{_evaluator_id(spec)}-err")

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

                if signal_node is not None:
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
                self._emit_ros_publication(mon, mon_node, guard_nodes)
                if signal_kind == "event":
                    self.graph.add((event_loop_node, RDF.type, EL.EventLoop))
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
                            _dsl_unit(mon.debounce_unit),
                        )
                        self.graph.add((mon_node, CSTR_HDL_EXT["debounce-duration"], debounce_node))
                elif signal_kind == "flag":
                    self.graph.add((mon_node, RDF.type, CSTR_HDL.LevelTriggeredMonitor))
                    self.graph.add((mon_node, CSTR_HDL.flag, signal_node))
                self.graph.add((handler_node, CSTR_HDL.monitors, mon_node))
                continue

            spec = cref.constraint if hasattr(cref, "constraint") else None
            if spec is None:
                continue
            if spec.disabled:
                continue
            claimed_constraints.add(spec.uri)

            error_node = self._constraint_error_node(
                spec,
                cref.motion,
                world_qtys,
                seen_error_ids,
                error_id_by_constraint,
                f"Monitor '{mon.name}'",
            )

            self._emit_error_evaluator(
                handler_node,
                spec,
                error_node,
                seen_eval_ids,
            )

            if signal_node is not None:
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
            self.graph.add((mon_node, CSTR_HDL.constraint, URIRef(spec.uri)))
            self.graph.add((mon_node, CSTR_HDL.error, error_node))
            self._emit_ros_publication(mon, mon_node, [URIRef(spec.uri)])
            if signal_kind == "event":
                self.graph.add((event_loop_node, RDF.type, EL.EventLoop))
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
                        _dsl_unit(mon.debounce_unit),
                    )
                    self.graph.add((mon_node, CSTR_HDL_EXT["debounce-duration"], debounce_node))
            elif signal_kind == "flag":
                self.graph.add((mon_node, RDF.type, CSTR_HDL.LevelTriggeredMonitor))
                self.graph.add((mon_node, CSTR_HDL.flag, signal_node))
            self.graph.add((handler_node, CSTR_HDL.monitors, mon_node))

        # A `while` constraint no controller drives and no monitor watches still states a bound.
        # It gets the same evaluator every other constraint gets, so its threshold is compared and
        # logged instead of reaching the program as a constant nothing reads.
        for item in _flatten_constraint_items(motion.while_.constraints):
            spec = _resolved_spec(item)
            if spec.disabled or spec.uri in claimed_constraints:
                continue
            self._emit_error_evaluator(
                handler_node,
                spec,
                self._constraint_error_node(
                    spec,
                    motion,
                    world_qtys,
                    seen_error_ids,
                    error_id_by_constraint,
                    f"Motion '{motion.name}'",
                ),
                seen_eval_ids,
            )

        for perturbation in getattr(handler, "perturbations", []) or []:
            self._emit_perturbation(
                perturbation,
                handler,
                handler_node,
                motion,
                world_qtys,
                seen_error_ids,
                error_id_by_constraint,
                seen_eval_ids,
            )

    @staticmethod
    def _perturbation_quantity(ref: ContextRef, perturbation: Any) -> ContextQuantity:
        """The named quantity a perturbation's magnitude or direction slot points at.

        Only a name is admitted: a force pattern richer than a constant arrives as a new kind of
        quantity, and an inline number in the handler would leave nowhere for it to go.
        """
        quantity = _context_quantity(ref)
        if not isinstance(quantity, ContextQuantity):
            raise ValueError(
                f"Perturbation '{perturbation.name}' must name a declared quantity for its "
                "magnitude and its direction."
            )
        return _resolved_context_quantity(quantity)

    def _perturbation_frame(self, perturbation: Any) -> tuple[URIRef, str]:
        """The frame a perturbation's wrench is stated in: the one its direction is seen by."""
        direction = self._perturbation_quantity(
            perturbation.force_direction or perturbation.moment_direction, perturbation
        )
        frame_name = _geo_prop(direction.props, "as-seen-by") or _geo_prop(direction.props, "wrt")
        if frame_name is None:
            raise ValueError(
                f"Perturbation '{perturbation.name}' direction '{direction.name}' needs an "
                "'as-seen-by: <frame>' prop: it is the frame the applied wrench is stated in."
            )
        return self._owned_uri(frame_name, direction), frame_name

    def _emit_perturbation_wrench(self, perturbation: Any) -> URIRef:
        """The wrench a perturbation's named magnitudes and directions compose to, by the same
        ops a force or moment command is built from."""
        as_seen_by_node, _frame_name = self._perturbation_frame(perturbation)
        point_node = self._declared_uri(f"point-{perturbation.name}", perturbation)
        position_node = self._declared_uri(f"position-{perturbation.name}", perturbation)
        self._emit_zero_position_coordinate(position_node, point_node, as_seen_by_node)

        wrench_nodes: list[URIRef] = []
        if perturbation.force is not None:
            wrench_node = self._declared_uri(f"wrench-force-{perturbation.name}", perturbation)
            self._emit_wrench_coordinate(wrench_node, point_node, as_seen_by_node)
            op_node = self._declared_uri(f"compute-wrench-force-{perturbation.name}", perturbation)
            self.graph.add((op_node, RDF.type, RBDYN_OP.WrenchFromPositionDirectionAndMagnitude))
            self.graph.add(
                (
                    op_node,
                    RBDYN_OP.magnitude,
                    URIRef(self._perturbation_quantity(perturbation.force, perturbation).uri),
                )
            )
            self.graph.add(
                (
                    op_node,
                    RBDYN_OP.direction,
                    URIRef(
                        self._perturbation_quantity(perturbation.force_direction, perturbation).uri
                    ),
                )
            )
            self.graph.add((op_node, RBDYN_OP.position, position_node))
            self.graph.add((op_node, RBDYN_OP.wrench, wrench_node))
            wrench_nodes.append(wrench_node)

        if perturbation.moment is not None:
            wrench_node = self._declared_uri(f"wrench-moment-{perturbation.name}", perturbation)
            self._emit_wrench_coordinate(wrench_node, point_node, as_seen_by_node)
            op_node = self._declared_uri(f"compute-wrench-moment-{perturbation.name}", perturbation)
            self.graph.add((op_node, RDF.type, RBDYN_OP_EXT.WrenchFromDirectionAndMoment))
            self.graph.add(
                (
                    op_node,
                    RBDYN_OP_EXT.moment,
                    URIRef(self._perturbation_quantity(perturbation.moment, perturbation).uri),
                )
            )
            self.graph.add(
                (
                    op_node,
                    RBDYN_OP.direction,
                    URIRef(
                        self._perturbation_quantity(perturbation.moment_direction, perturbation).uri
                    ),
                )
            )
            self.graph.add((op_node, RBDYN_OP.wrench, wrench_node))
            wrench_nodes.append(wrench_node)

        if not wrench_nodes:
            raise ValueError(
                f"Perturbation '{perturbation.name}' applies neither a force nor a torque."
            )
        if len(wrench_nodes) == 1:
            return wrench_nodes[0]

        total_node = self._declared_uri(f"wrench-{perturbation.name}", perturbation)
        self._emit_wrench_coordinate(total_node, point_node, as_seen_by_node)
        add_node = self._declared_uri(f"add-wrench-{perturbation.name}", perturbation)
        self.graph.add((add_node, RDF.type, RBDYN_OP.AddWrench))
        self.graph.add((add_node, RBDYN_OP["in1"], wrench_nodes[0]))
        self.graph.add((add_node, RBDYN_OP["in2"], wrench_nodes[1]))
        self.graph.add((add_node, RBDYN_OP.out, total_node))
        return total_node

    def _emit_perturbation(
        self,
        perturbation: Any,
        handler: ConstraintHandler,
        handler_node: URIRef,
        motion: GuardedMotion,
        world_qtys: dict[str, WorldQuantity],
        seen_error_ids: set[str],
        error_id_by_constraint: dict[str, str],
        seen_eval_ids: set[str],
    ) -> None:
        """Emit one perturbation: the wrench it applies, the body it acts on, the state that
        arms it, the conditions that open its window and how long the window stays open.
        """
        node = URIRef(perturbation.uri)
        self.graph.add((node, RDF.type, SIM.Perturbation))
        self.graph.add((node, PROV.wasDerivedFrom, handler_node))
        body_node = URIRef(str(perturbation.body.uri))
        self.graph.add((body_node, RDF.type, GEOM_ENT.SimplicialComplex))
        self.graph.add((node, SLV["attached-to"], body_node))
        if handler.runs_in is not None:
            self.graph.add((node, CSTR_HDL_EXT["runs-in-state"], URIRef(handler.runs_in.uri)))
        self.graph.add((node, SLV.force, self._emit_perturbation_wrench(perturbation)))

        if perturbation.duration is not None:
            self.graph.add(
                (
                    node,
                    TIME.hasDuration,
                    self._emit_duration_threshold_node(
                        perturbation.duration, perturbation, f"duration-{perturbation.name}"
                    ),
                )
            )

        members = [
            spec
            for spec in (
                _resolved_spec(item) for item in _flatten_constraint_items(perturbation.conditions)
            )
            if not spec.disabled
        ]
        if not members:
            return
        for spec in members:
            self._emit_error_evaluator(
                handler_node,
                spec,
                self._constraint_error_node(
                    spec,
                    motion,
                    world_qtys,
                    seen_error_ids,
                    error_id_by_constraint,
                    f"Perturbation '{perturbation.name}'",
                ),
                seen_eval_ids,
            )
        node_type = _section_expression_type(
            getattr(perturbation.gate, "logic", None), len(members)
        )
        holder = node
        if node_type is not None:
            holder = self._declared_uri(f"when-{perturbation.name}", perturbation)
            self.graph.add((holder, RDF.type, node_type))
            self.graph.add((node, CSTR_EXT["has-constraint"], holder))
        for spec in members:
            self.graph.add((holder, CSTR_EXT["has-constraint"], URIRef(spec.uri)))

    def _forwarded_command_signal(
        self,
        ctrl: ControllerEntry,
        qty: WorldQuantity | None,
        subspace: str,
        axis: str | None,
        handler: ConstraintHandler,
    ) -> URIRef:
        """Emit the authored signal forwarded directly to a device command."""
        signal = self._declared_uri(f"cmd-{ctrl.name}", ctrl)
        signal_type = _scalar_type(qty, subspace, axis) if qty else QuantityType.FreeVector
        self._add_quantity(signal, signal_type)
        return signal

    def _solver_node(self, handler, solver) -> URIRef:
        """A solver instance belongs to the handler that runs it: two handlers realizing the
        same motion each drive their own, with their own controllers and drivers."""
        return self._owned_uri(f"{solver.name}-{handler.name}", handler)

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
            solver_node = self._solver_node(handler, solver)
            # The declared solvers are the handler's runtimes; controller plans alone cannot
            # recover one that no controller routes to (a monitor-only arm).
            self.graph.add((URIRef(handler.uri), CSTR_HDL_EXT["runs-solver"], solver_node))
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

            driver_stem = f"{solver.name}-{handler.name}" if multi else handler.name

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
            if alg is not None:
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
                            value_obj = Literal(float(element.value), datatype=XSD.double)
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

            if solver.algorithm is None:
                raise ValueError(
                    f"Serial-chain solver '{solver.name}' declares no algorithm, but "
                    f"controller '{ctrl.name}' routes through it; a driven solver needs one."
                )

            cref = ctrl.params.constraint
            spec = cref.constraint if hasattr(cref, "constraint") else None
            if spec is None:
                continue

            qty = self._resolve_constraint_quantity(spec, world_qtys)
            quantityless = qty is None and self._quantityless_scalar_type(spec) is not None
            if qty is None and not quantityless and self._along_path_scalar(spec) is None:
                raise ValueError(
                    f"Controller '{ctrl.name}' constraint '{spec.name}' does not resolve to a world quantity."
                )

            subspace = None if quantityless else _view_subspace(spec)
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
                self._emit_cartesian_force_spec(ctrl, wrench_node, handler, driver_node)
            elif command.is_moment_command:
                wrench_node = self._emit_moment_command_wrench(
                    ctrl, spec, qty, command, handler, motion
                )
                self._emit_cartesian_force_spec(ctrl, wrench_node, handler, driver_node)

            # Both serial-chain algorithms consume a joint force: ACHD takes it as its
            # feed-forward torque input, RNE adds it to the torque it computed.
            if (
                solver.algorithm in {"ACHD", "RNE"}
                and command.is_posture_torque_command
                and qty is not None
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

                spec_node = self._declared_uri(f"jf-spec-{ctrl.name}", ctrl)
                self.graph.add((spec_node, RDF.type, SLV.JointForceSpecification))
                self.graph.add((spec_node, PROV.wasDerivedFrom, URIRef(ctrl.uri)))
                self.graph.add((spec_node, SLV.force, torque_node))
                self.graph.add((solver_node, SLV["output"], URIRef(qty.uri)))
                self.graph.add((spec_node, SLV["attached-to"], joint_node))
                self.graph.add((driver_node, SLV["joint-force"], spec_node))

    def _emit_cartesian_force_spec(
        self,
        ctrl: ControllerEntry,
        wrench_node: URIRef,
        handler: ConstraintHandler,
        driver_node: URIRef,
    ) -> None:
        """Attach a commanded wrench (force or moment) to the `apply at` body as the driver's
        Cartesian force specification."""
        spec_node = self._declared_uri(f"spec-{ctrl.name}", ctrl)
        self.graph.add((spec_node, RDF.type, SLV.CartesianForceSpecification))
        self.graph.add((spec_node, PROV.wasDerivedFrom, URIRef(ctrl.uri)))
        self.graph.add((spec_node, SLV.force, wrench_node))
        apply_at = getattr(ctrl, "apply_at", None)
        if apply_at is not None and hasattr(apply_at, "uri"):
            # The scene reference is already the rigid body.
            body_node = URIRef(str(apply_at.uri))
            self.graph.add((body_node, RDF.type, GEOM_ENT.SimplicialComplex))
            self.graph.add((spec_node, SLV["attached-to"], body_node))
        self.graph.add((driver_node, SLV["cartesian-force"], spec_node))

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
