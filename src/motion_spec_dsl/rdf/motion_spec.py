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
from rdflib.term import Literal, URIRef
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
    ConfigValue,
    ContextQuantity,
    ContextRef,
    GeometricPropKey,
    GeometricProps,
    GeoPropPair,
    Measure,
    QuantityType,
    ReferenceGeneratorType,
    ReferenceValue,
    SnapshotValue,
    VectorXYZ,
    WorldQuantity,
    WorldQuantityType,
    _resolved_context_quantity,
    _resolved_world_quantity,
)
from motion_spec_dsl.classes.controller_semantics import (
    ANGULAR_SUBSPACES,
    SUBSPACE_ALIAS,
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
    _angle_unit,
    _axis_vector,
    _context_quantity,
    _DistancePlan,
    _dsl_unit,
    _evaluator_id,
    _angle_bound,
    _geo_prop,
    _geo_prop_value,
    _is_distance_view,
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
    RBDYN_OP_EXT,
    SENSORS,
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
        self._frame_coords_index: dict[URIRef, tuple[URIRef, URIRef, URIRef]] = {}
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
        self._linear_distance_relations: dict[tuple[str, str], URIRef] = {}
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
            constraints = _resolved_constraint_items(motion)

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
        accepted goal produces, and its field rows are what a completed run answers with. What a
        scenario observes while the run plays out is authored on the monitors
        (`publish: event to`), not here.
        """
        node = URIRef(server.uri)
        self.graph.add((node, RDF.type, ROS.Action))
        self.graph.add((node, ROS["channel-name"], Literal(server.channel_name)))
        self.graph.add((node, ROS["type-name"], Literal(server.type_name)))
        self.graph.add((node, RDFS.member, URIRef(server.goal_event.uri)))
        for row_index, (path, value) in enumerate(server.result):
            row = URIRef(f"{server.uri}.r{row_index}")
            self.graph.add((node, RDFS.member, row))
            self.graph.add((row, ROS["field-path"], Literal(path)))
            self.graph.add((row, RDF.value, Literal(value)))

    def _emit_ros_subscription(self, subscription: RosSubscriptionDecl) -> None:
        """A subscribed topic: the channel poses arrive on, the message it carries, and the
        world poses it informs the model about. The features of interest are what make it
        a channel the model reads rather than one it writes.
        """
        node = URIRef(subscription.uri)
        self.graph.add((node, RDF.type, ROS.Topic))
        self.graph.add((node, ROS["channel-name"], Literal(subscription.channel_name)))
        self.graph.add((node, ROS["type-name"], Literal(subscription.type_name)))
        self.graph.add((node, ROS["field-path"], Literal(subscription.pose_path)))
        for target in subscription.targets:
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
        the run, so it is not scoped to any one motion the way a verdict is. The quantity is
        stated whole -- the message reports it, and only a field the model wants mapped
        differently is written out.
        """
        node = URIRef(standing.uri)
        self.graph.add((node, RDF.type, ROS.Topic))
        self.graph.add((node, ROS["channel-name"], Literal(standing.topic.channel_name)))
        self.graph.add((node, ROS["type-name"], Literal(standing.topic.type_name)))
        self.graph.add((node, RDF.value, URIRef(standing.quantity.uri)))

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

    def _declared_uri(self, name: str, quantity: ContextQuantity) -> URIRef:
        """Create a URI for a node named after `quantity`, under the quantity itself.

        A context quantity's name is scoped to the block declaring it, so two motions may each
        call their path `trajectory`. Hanging the geometry, the projection and the parameters
        generated from one under its own IRI keeps them apart; minting them in the namespace
        root would collapse both motions onto one node.
        """
        if urlsplit(str(name)).scheme:
            return URIRef(name)
        return URIRef(f"{quantity.uri}/{name}")

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

        start = self._distance_operand(spec.view.distance_from, world_qtys)
        end = self._distance_operand(spec.view.distance_to, world_qtys)
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
        plan = _DistancePlan(start, end, target)
        self._distance_plans[spec] = plan
        return plan

    def _linear_distance_relation(self, start_uri: str, end_uri: str) -> URIRef:
        """The LinearDistance two pose endpoints stand in, minted once per endpoint pair.

        The relation is the geometric fact; a constraint's coordinate is one motion's sampling
        of it. Motions measuring the same two poses share the relation and differ only in the
        coordinate, so the endpoint pair -- not the constraint's name -- is its identity.
        """
        key = (str(start_uri), str(end_uri))
        node = self._linear_distance_relations.get(key)
        if node is not None:
            return node
        name = "-".join(
            ["linear-distance", self._model_local_path(start_uri), self._model_local_path(end_uri)]
        )
        node = self._owned_uri(name, None)
        self.graph.add((node, RDF.type, GEOM_REL.LinearDistance))
        self.graph.add((node, GEOM_REL["between-entities"], URIRef(start_uri)))
        self.graph.add((node, GEOM_REL["between-entities"], URIRef(end_uri)))
        self._linear_distance_relations[key] = node
        return node

    def _model_local_path(self, uri: str) -> str:
        """A URI's path below the model namespace, flattened into one name segment.

        The whole path, not the last segment: two motions declare the same `start-pose`, and
        only the path above it tells them apart.
        """
        namespace = str(self._namespace_owner(None).ns.uri)
        if not str(uri).startswith(namespace):
            raise ValueError(f"'{uri}' is not owned by the model namespace '{namespace}'.")
        return str(uri).removeprefix(namespace).replace("/", "-")

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
        self._emit_zero_position_coordinate(position_node, point_node, as_seen_by_node)
        # A commanded wrench -- one no sensor observes -- is what this op produces: the model
        # declared the quantity the command realizes, so the value belongs in it rather than in
        # a second wrench beside it. Its coordinate is already emitted with the world quantities.
        wrench_node = self._commanded_wrench_node(qty)
        if wrench_node is None:
            wrench_node = self._owned_uri(f"wrench-force-{ctrl.name}", motion)
            self._emit_wrench_coordinate(wrench_node, point_node, as_seen_by_node)

        op_node = self._owned_uri(f"compute-wrench-force-{ctrl.name}", motion)
        self.graph.add((op_node, RDF.type, RBDYN_OP.WrenchFromPositionDirectionAndMagnitude))
        self.graph.add((op_node, RBDYN_OP.magnitude, magnitude_node))
        self.graph.add((op_node, RBDYN_OP.direction, direction_node))
        self.graph.add((op_node, RBDYN_OP.position, position_node))
        self.graph.add((op_node, RBDYN_OP.wrench, wrench_node))

        return wrench_node

    def _emit_moment_command_wrench(
        self,
        ctrl: ControllerEntry,
        qty: WorldQuantity,
        command: Any,
        handler: ConstraintHandler,
        motion: GuardedMotion,
    ) -> URIRef:
        """Emit the op chain building a moment controller's command wrench: one
        WrenchFromDirectionAndMoment per commanded angular axis, folded with AddWrench.
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
        if not axes:
            raise ValueError(f"Moment controller '{ctrl.name}' commands no angular axis.")
        multi = len(axes) > 1

        # The wrench coordinate still needs a well-formed reference point; the op ignores it.
        point_node = self._owned_uri(f"point-moment-{ctrl.name}", motion)
        position_node = self._owned_uri(f"position-moment-{ctrl.name}", motion)
        self._emit_zero_position_coordinate(position_node, point_node, as_seen_by_node)

        wrench_nodes: list[URIRef] = []
        for axis in axes:
            direction_node = self._owned_uri(f"direction-moment-{ctrl.name}-ang-{axis}", motion)
            self._emit_direction_coordinate(direction_node, as_seen_by_node, _axis_vector(axis))
            magnitude_node = self._moment_control_signal_node(
                ctrl, handler, axis if multi else None
            )
            wrench_node = self._owned_uri(f"wrench-moment-{ctrl.name}-ang-{axis}", motion)
            self._emit_wrench_coordinate(wrench_node, point_node, as_seen_by_node)

            op_node = self._owned_uri(f"compute-wrench-moment-{ctrl.name}-ang-{axis}", motion)
            self.graph.add((op_node, RDF.type, RBDYN_OP_EXT.WrenchFromDirectionAndMoment))
            self.graph.add((op_node, RBDYN_OP_EXT.moment, magnitude_node))
            self.graph.add((op_node, RBDYN_OP.direction, direction_node))
            self.graph.add((op_node, RBDYN_OP.wrench, wrench_node))
            wrench_nodes.append(wrench_node)

        total = wrench_nodes[0]
        for index, addend in enumerate(wrench_nodes[1:], start=1):
            sum_node = self._owned_uri(f"wrench-moment-{ctrl.name}-sum-{index}", motion)
            self._emit_wrench_coordinate(sum_node, point_node, as_seen_by_node)
            add_node = self._owned_uri(f"add-wrench-{ctrl.name}-{index}", motion)
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
                    if _owns_pose_subobjects(quantity.value)
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
                if _owns_pose_subobjects(quantity.value)
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
        self.graph.add((node, QUDT_SCHEMA.unit, _dsl_unit(quantity.value.position.unit or "m")))
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
            profile_node = self._owned_uri(f"profile-{spec.name}-{ctrl.name}", motion)
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

    def _emit_context_ref_node(self, ref: ContextRef, owner: Any, suffix: str) -> URIRef:
        """Resolve a context reference to its value node: a subspace view, a passthrough source,
        or the referenced quantity.
        """
        quantity = _context_quantity(ref)
        if not isinstance(quantity, ContextQuantity):
            return self._owned_uri(_node_name(quantity), owner)

        quantity = _resolved_context_quantity(quantity)
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
                if _owns_pose_subobjects(quantity.value)
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
                self._emit_constraint_tolerance(node, spec, motion, qty, subspace, axis, scalar_t)
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
        out_node = self._owned_uri(f"{spec.name}-{ctrl.name}-profile-ref", motion)
        self._add_quantity(out_node, scalar_t)

        op_node = self._owned_uri(f"profile-{spec.name}-{ctrl.name}", motion)
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
                    self._linear_distance_relation(plan.start.uri, plan.end.uri),
                )
            )

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
        error_normalization = getattr(controller.params, "error_normalization", None)
        if error_normalization is not None:
            self._emit_angle_normalization(
                controller_node, error_normalization, controller, f"err-norm-{controller.name}"
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

    def _emit_ros_publication(
        self, monitor: Any, monitor_node: URIRef, watched: list[URIRef]
    ) -> None:
        """Name the channel a monitor publishes on, the message it carries, and every field
        assignment it authored, each under the condition its state block holds.
        """
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
                    if qty is None:
                        raise ValueError(
                            f"Aggregate monitor '{mon.name}' constraint '{spec.name}' does not resolve to a world quantity."
                        )
                    subspace = _view_subspace(spec)
                    axis_raw = spec.view.axis
                    axis = semantic_axis_label(axis_raw)
                    scalar_t = _scalar_type(qty, subspace, axis) if qty else subspace
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

            if isinstance(spec, GoalStatusConstraint):
                error_node = URIRef(spec.act.status_uri)
            elif getattr(spec.view, "is_elapsed", False):
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
                error_id = error_id_by_constraint.get(spec.uri, f"{_evaluator_id(spec)}-err")
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
                self._emit_cartesian_force_spec(ctrl, wrench_node, handler, driver_node)
            elif command.is_moment_command:
                wrench_node = self._emit_moment_command_wrench(ctrl, qty, command, handler, motion)
                self._emit_cartesian_force_spec(ctrl, wrench_node, handler, driver_node)

            # Both serial-chain algorithms consume a joint force: ACHD takes it as its
            # feed-forward torque input, RNE adds it to the torque it computed.
            if (
                solver.algorithm in {"ACHD", "RNE"}
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

    def _emit_cartesian_force_spec(
        self,
        ctrl: ControllerEntry,
        wrench_node: URIRef,
        handler: ConstraintHandler,
        driver_node: URIRef,
    ) -> None:
        """Attach a commanded wrench (force or moment) to the `apply at` body as the driver's
        Cartesian force specification."""
        spec_node = self._owned_uri(f"spec-{ctrl.name}", handler)
        self.graph.add((spec_node, RDF.type, SLV.CartesianForceSpecification))
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
