# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""The state one emission shares, and the node naming and typing every emitter uses.

Resolution is kept apart from writing: plans and indexes are filled once and read during
emission, which never queries the graph back for what it already decided.
"""

from __future__ import annotations

from typing import Any
from urllib.parse import urlsplit

from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT
from rdflib.graph import Dataset
from rdflib.namespace import RDF, XSD, Namespace
from rdflib.term import Literal, URIRef
from textx.scoping import get_included_models

from motion_spec_dsl.classes.common import IHasNamespaceDeclare
from motion_spec_dsl.classes.constraint_handler import ConstraintHandler, ControllerAlias
from motion_spec_dsl.classes.constraints import ConstraintSpecification
from motion_spec_dsl.classes.context import ContextQuantityAlias, QuantityType
from motion_spec_dsl.classes.motion_spec import Model, ToleranceDefaults
from motion_spec_dsl.classes.units import QUDT_KIND_BY_QUANTITY_TYPE
from motion_spec_dsl.rdf.model import GRAPH_BINDINGS, QKIND_PREFIXES, SCALAR_UNIT
from motion_spec_dsl.rdf_parser.vocab import GEOM_COORD, MAP, QUDT_QKIND, QUDT_SCHEMA


class Emission:
    """What one dataset's emission resolves once and remembers: plans, indexes, emitted nodes."""

    def __init__(self, model: Model) -> None:
        self.model = model
        self.models = get_included_models(model)
        self.dataset = Dataset()
        for prefix, namespace in GRAPH_BINDINGS:
            self.dataset.bind(prefix, namespace)
        self.graph = self.dataset.default_graph
        specs = [spec for included in self.models if isinstance(included, Model) for spec in included.specs]
        self.handlers = [spec for spec in specs if isinstance(spec, ConstraintHandler)]
        self.default_ns_owner = next(iter(self.handlers), None)
        self.tolerance_defaults = {
            entry.kind: entry.band
            for spec in specs
            if isinstance(spec, ToleranceDefaults)
            for entry in spec.defaults
        }
        # One plan per constraint, of whichever kind its view is.
        self.plans: dict[ConstraintSpecification, Any] = {}
        self.derived_scalars: dict[Any, Any] = {}
        # A derived scalar's view, held by a constraint so the plan builders take it as one.
        self.derived_specs: dict[Any, ConstraintSpecification] = {}
        self.derived_scalar_declarations: list[tuple[Any, URIRef]] = []
        # A pose coordinate's (of, wrt, as-seen-by) frame nodes.
        self.frame_coords: dict[URIRef, tuple[URIRef, URIRef, URIRef]] = {}
        # (coordinate, domain) -> its relation; relations pool by frame pair, so only this says
        # which coordinate's component a relation stands for.
        self.component_relations: dict[tuple[URIRef, str], URIRef] = {}
        # (coordinate, domain) -> the view a constraint operand names for that component.
        self.component_views: dict[tuple[URIRef, str], URIRef] = {}
        self.config_resource: URIRef | None = None
        self.controller_by_spec: dict[ConstraintSpecification, Any] = {}
        self.profiled_controller_by_spec: dict[ConstraintSpecification, Any] = {}
        # The context quantities a controller drives: an expression among them needs its gradient.
        self.driven_quantities: set[Any] = set()
        for handler in self.handlers:
            for item in handler.controllers:
                controller = item.ref.controller if isinstance(item, ControllerAlias) else item
                spec = controller.params.constraint.constraint
                self.controller_by_spec.setdefault(spec, controller)
                if controller.params.profile is not None:
                    self.profiled_controller_by_spec.setdefault(spec, controller)
                driven = spec.view.quantity
                self.driven_quantities.add(driven.ref if isinstance(driven, ContextQuantityAlias) else driven)
        # A controlled expression's gradient, by the node the expression computes into.
        self.expression_plans: dict[URIRef, Any] = {}
        self.emitted_bands: set[URIRef] = set()
        self.emitted_distance_ops: set[str] = set()
        self.emitted_alignment_ops: set[str] = set()
        self.emitted_geometric_distance_ops: set[str] = set()
        self.emitted_norm_ops: set[URIRef] = set()
        self.emitted_views: set[URIRef] = set()
        self.emitted_pose_parts: set[URIRef] = set()
        self.path_projections: set[URIRef] = set()
        self.linear_distance_relations: dict[tuple[str, str], URIRef] = {}
        self.angular_distance_relations: dict[tuple[str, str], URIRef] = {}
        self.motion_time_endpoints: dict[str, tuple[URIRef, URIRef]] = {}
        self.observation_instants: dict[str, URIRef] = {}


def owned_uri(em: Emission, name: str, owner: Any) -> URIRef:
    """A node named in its owner's namespace: a world quantity, a solver, a robot.

    One such node serves every motion that names it; a name scoped to a block takes
    `declared_uri` instead.
    """
    if urlsplit(str(name)).scheme:
        return URIRef(name)
    current = owner
    while current is not None and not isinstance(current, IHasNamespaceDeclare):
        current = None if isinstance(current, Model) else current.parent
    if current is None:
        current = em.default_ns_owner
    if current is None:
        raise ValueError(f"'{name}' has no namespace owner to mint its IRI in")
    return Namespace(str(current.ns.uri))[name]


def declared_uri(name: str, declaration: Any) -> URIRef:
    """A node named under DECLARATION's own IRI, for names written inside a block.

    Two motions may each call their path `trajectory`; minted with `owned_uri` they would
    collapse onto one node collecting both motions' inputs.
    """
    if urlsplit(str(name)).scheme:
        return URIRef(name)
    return URIRef(f"{declaration.uri}/{name}")


def emit_quantity_kind(em: Emission, node: URIRef, qkind: URIRef) -> None:
    """Link NODE's kind; a structural kind is a class too, a QUDT kind an individual."""
    # The angle shapes state kind membership with sh:class.
    if not str(qkind).startswith(QKIND_PREFIXES) or qkind == QUDT_QKIND.PlaneAngle:
        em.graph.add((node, RDF.type, qkind))
    em.graph.add((node, QUDT_SCHEMA.hasQuantityKind, qkind))


def emit_scalar_quantity(
    em: Emission, node: URIRef, value: float, qkind: URIRef | None, unit: URIRef
) -> URIRef:
    """A unit-bearing qudt:Quantity holding one number."""
    em.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    if qkind is not None:
        emit_quantity_kind(em, node, qkind)
    em.graph.add((node, QUDT_SCHEMA.unit, unit))
    em.graph.add((node, QUDT_SCHEMA.value, Literal(float(value), datatype=XSD.double)))
    return node


def add_quantity(em: Emission, node: URIRef, scalar_type: Any) -> None:
    """Type NODE as a quantity of SCALAR_TYPE, with its kind and canonical unit."""
    qkind = QUDT_KIND_BY_QUANTITY_TYPE.get(scalar_type) or QUDT_QKIND[scalar_type]
    em.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    if scalar_type == QuantityType.Distance:
        em.graph.add((node, RDF.type, GEOM_COORD.LinearDistanceCoordinate))
    emit_quantity_kind(em, node, qkind)
    em.graph.add((node, QUDT_SCHEMA.unit, SCALAR_UNIT.get(scalar_type, QUDT_UNIT.UNITLESS)))


def emit_view(em: Emission, view_node: URIRef) -> None:
    """Type VIEW_NODE a map:View and remember it, so no emitter queries the graph for it."""
    em.graph.add((view_node, RDF.type, MAP.View))
    em.emitted_views.add(view_node)


def model_local_path(em: Emission, uri: str) -> str:
    """An IRI's whole path below the model namespace as one name segment, else its own path.

    The whole path, since two motions may each declare a `start-pose`.
    """
    namespace = str(owned_uri(em, "", None))
    if str(uri).startswith(namespace):
        return str(uri).removeprefix(namespace).replace("/", "-")
    _, _, rest = str(uri).partition("://")
    return rest.replace("/", "-")
