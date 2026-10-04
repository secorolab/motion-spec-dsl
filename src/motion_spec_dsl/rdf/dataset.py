# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""The motion-specification dataset of a parsed model, and the JSON-LD context it serializes with."""

from __future__ import annotations

from typing import Any

from rdflib.graph import Dataset
from rdflib.namespace import RDF, XSD, Namespace
from rdflib.term import Literal, URIRef

from motion_spec_dsl.classes.constraint_handler import (
    motion_context_quantities,
    motion_world_quantities,
    perturbation_conditions,
    resolved_constraint_items,
)
from motion_spec_dsl.classes.context import QuantityType
from motion_spec_dsl.classes.controller_semantics import constraint_view_subspace
from motion_spec_dsl.classes.motion_spec import ContextSpec, ExecutionContext, GuardedMotion, Model
from motion_spec_dsl.classes.ros import Ros
from motion_spec_dsl.rdf.common import scalar_id
from motion_spec_dsl.rdf.constraints import emit_constraints, emit_detect_acts, emit_motion_spec
from motion_spec_dsl.rdf.emission import Emission, add_quantity, owned_uri
from motion_spec_dsl.rdf.execution import (
    emit_execution_context,
    emit_ros_action_server,
    emit_ros_standing_pub,
    emit_ros_subscription,
)
from motion_spec_dsl.rdf.handlers import emit_constraint_handler
from motion_spec_dsl.rdf.model import GRAPH_BINDINGS
from motion_spec_dsl.rdf.plans import derived_scalar_spec, resolve_constraint_quantity
from motion_spec_dsl.rdf.quantities import emit_context_members, emit_context_quantities, emit_world_quantities
from motion_spec_dsl.rdf.solvers import emit_solvers
from motion_spec_dsl.rdf.views import emit_map_operations, emit_path_following, emit_scalar_views
from motion_spec_dsl.rdf_parser.vocab import ALGO_EXT, QUDT_SCHEMA

_COERCIBLE_DATATYPES = (XSD.double, XSD.integer)


def build_dataset(model: Model) -> tuple[Dataset, dict[str, Any]]:
    """The dataset of MODEL's handlers, motions and deployment, with its JSON-LD namespace context."""
    em = Emission(model)
    context: dict[str, Any] = {
        prefix: str(namespace if isinstance(namespace, Namespace) else namespace._NS)
        for prefix, namespace in GRAPH_BINDINGS
    }
    for included in em.models:
        if not isinstance(included, Model):
            continue
        for spec in included.specs:
            if isinstance(spec, ExecutionContext):
                emit_execution_context(em, spec)
                bindings = [(spec.ns_prefix, spec.ns.uri), (spec.scene.ns_prefix, spec.scene.ns.uri)]
                for modelled_agent in spec.scene.modelled_agns:
                    agent_set = modelled_agent.agn.parent
                    bindings.append((agent_set.ns_prefix, agent_set.ns.uri))
            elif isinstance(spec, ContextSpec):
                emit_context_members(em, URIRef(spec.uri), spec)
                bindings = [(spec.ns_prefix, spec.ns.uri)]
            elif isinstance(spec, Ros):
                for server in spec.servers:
                    emit_ros_action_server(em, server)
                for subscription in spec.subscriptions:
                    emit_ros_subscription(em, subscription)
                for standing in spec.always:
                    emit_ros_standing_pub(em, standing)
                served = spec.servers or spec.subscriptions or spec.always
                bindings = [(spec.ns.name, spec.ns.uri)] if served else []
            else:
                bindings = []
            for prefix, uri in bindings:
                em.dataset.bind(prefix, uri)
                context[prefix] = uri
    # A constraint more than one motion names has its controllers and solvers emitted once.
    motions_by_spec: dict[Any, set[str]] = {}
    for handler in em.handlers:
        if isinstance(handler.motion, GuardedMotion):
            for spec in resolved_constraint_items(handler.motion):
                motions_by_spec.setdefault(spec, set()).add(str(handler.motion.uri))
    shared_specs = {spec for spec, motions in motions_by_spec.items() if len(motions) > 1}
    all_world_qtys: dict[str, Any] = {}
    for handler_order, handler in enumerate(em.handlers):
        motion = handler.motion
        if not isinstance(motion, GuardedMotion):
            continue
        for prefix, uri in ((handler.ns_prefix, handler.ns.uri), (motion.ns_prefix, motion.ns.uri)):
            em.dataset.bind(prefix, uri)
            context[prefix] = uri
        context_quantities = motion_context_quantities(em.models, motion, handler)
        world_qtys = motion_world_quantities(em.models, motion, handler, context_quantities)
        constraints = resolved_constraint_items(motion) + perturbation_conditions(handler)
        emit_world_quantities(em, world_qtys)
        emit_context_quantities(em, context_quantities, constraints, world_qtys)
        all_world_qtys.update(world_qtys)
        emit_path_following(em, constraints, world_qtys)
        emit_constraints(em, motion, constraints, world_qtys)
        emit_detect_acts(em, motion)
        emit_motion_spec(em, motion)
        emit_scalar_views(em, motion, constraints, world_qtys)
        emit_map_operations(em, motion, constraints, world_qtys)
        emit_constraint_handler(em, handler, motion, world_qtys, shared_specs, handler_order)
        emit_solvers(em, handler, motion, world_qtys)
    # A derived scalar gets the ops a constraint holding its view would, owned by its declaring context.
    for declaration, declared_node in em.derived_scalar_declarations:
        derived = derived_scalar_spec(em, declaration)
        owner = derived.parent.parent
        emit_map_operations(em, owner, [derived], all_world_qtys)
        # The declared name is what the model reads: tied to the view's scalar by a one-input sum.
        target = resolve_constraint_quantity(em, derived, all_world_qtys)
        scalar = owned_uri(em, scalar_id(target, constraint_view_subspace(derived), None), owner)
        zero = owned_uri(em, f"{declaration.name}-zero", owner)
        add_quantity(em, zero, QuantityType.Distance)
        em.graph.add((zero, QUDT_SCHEMA.value, Literal(0.0, datatype=XSD.double)))
        copy_op = owned_uri(em, f"compute-{declaration.name}", owner)
        em.graph.add((copy_op, RDF.type, ALGO_EXT.Addition))
        em.graph.add((copy_op, ALGO_EXT["in"], scalar))
        em.graph.add((copy_op, ALGO_EXT["in"], zero))
        em.graph.add((copy_op, ALGO_EXT.out, declared_node))
    # rdflib serializes numbers bare under an active context, so a whole double would read back
    # as an integer; a term with one numeric datatype declares it once.
    datatypes: dict[URIRef, set[URIRef]] = {}
    for _subject, predicate, obj in em.graph:
        if isinstance(obj, Literal) and obj.datatype in _COERCIBLE_DATATYPES:
            datatypes.setdefault(predicate, set()).add(obj.datatype)
    terms = {}
    for predicate, found in datatypes.items():
        if len(found) != 1:
            continue
        curies = []
        for iri in (str(predicate), str(next(iter(found)))):
            # The longest namespace the IRI falls in names it.
            longest = None
            for prefix, namespace in context.items():
                if iri.startswith(namespace) and (longest is None or len(namespace) > len(longest[1])):
                    longest = (prefix, namespace)
            if longest is not None:
                curies.append(f"{longest[0]}:{iri[len(longest[1]):]}")
        if len(curies) == 2:
            terms[curies[0]] = {"@id": str(predicate), "@type": curies[1]}
    context.update(terms)
    return em.dataset, context
