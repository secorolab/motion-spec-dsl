# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""The execution context a specification is deployed in, and the ROS interfaces it serves and reads."""

from __future__ import annotations

from pathlib import Path

from rdf_utils.namespace import NS_MM_GEOM_REL, NS_MM_QUDT_QTY
from rdflib.namespace import RDF, RDFS, SDO, SSN
from rdflib.term import Literal, URIRef
from textx import get_model

from motion_spec_dsl.classes.motion_spec import ExecutionContext
from motion_spec_dsl.classes.ros import RosActionServerDecl, RosStandingPub, RosSubscriptionDecl
from motion_spec_dsl.classes.units import DSL_UNITS
from motion_spec_dsl.rdf.emission import Emission, emit_scalar_quantity
from motion_spec_dsl.rdf.geometry import view_node
from motion_spec_dsl.rdf.model import ROS
from motion_spec_dsl.rdf_parser.vocab import EXEC, GEOM_REL, SENSORS, SOSA


def emit_ros_action_server(em: Emission, server: RosActionServerDecl) -> None:
    """The action goals arrive on; its one valueless member is the event an accepted goal produces."""
    node = URIRef(server.uri)
    em.graph.add((node, RDF.type, ROS.Action))
    em.graph.add((node, ROS["channel-name"], Literal(server.channel_name)))
    em.graph.add((node, ROS["type-name"], Literal(server.type_name)))
    em.graph.add((node, RDFS.member, URIRef(server.goal_event.uri)))


def emit_ros_subscription(em: Emission, subscription: RosSubscriptionDecl) -> None:
    """A topic the model reads: its message, the pose it observes, and the poses or cameras it is about."""
    node = URIRef(subscription.uri)
    em.graph.add((node, RDF.type, ROS.Topic))
    em.graph.add((node, ROS["channel-name"], Literal(subscription.channel_name)))
    em.graph.add((node, ROS["type-name"], Literal(subscription.type_name)))
    # Where in a detection the pose sits; an image carries none.
    if subscription.pose_path is not None:
        em.graph.add((node, ROS["field-path"], Literal(subscription.pose_path)))
    if subscription.observed is not None:
        observed = URIRef(f"{subscription.uri}-observed-pose")
        em.graph.add((observed, RDF.type, NS_MM_GEOM_REL["Pose"]))
        em.graph.add((observed, GEOM_REL.of, URIRef(str(subscription.observed.of.uri))))
        em.graph.add((observed, GEOM_REL["with-respect-to"], URIRef(str(subscription.observed.wrt.uri))))
        em.graph.add((node, SOSA.observedProperty, observed))
    for target in (*subscription.targets, *subscription.cameras):
        em.graph.add((node, SOSA.hasFeatureOfInterest, URIRef(str(target.ref.uri))))


def emit_ros_standing_pub(em: Emission, standing: RosStandingPub) -> None:
    """A topic published for the whole run at its rate, held by the execution context, not a motion."""
    topic_uri = standing.topic.uri
    node = URIRef(topic_uri)
    em.graph.add((node, RDF.type, ROS.Topic))
    em.graph.add((node, ROS["channel-name"], Literal(standing.topic.channel_name)))
    em.graph.add((node, ROS["type-name"], Literal(standing.topic.type_name)))
    # Each entry is a quantity stated whole, and the entity it is about when a message carries several.
    for index, entry in enumerate(standing.entries):
        row = URIRef(f"{topic_uri}.e{index}")
        em.graph.add((node, RDFS.member, row))
        em.graph.add((row, RDF.value, URIRef(entry.quantity.uri)))
        if entry.subject is not None:
            em.graph.add((row, SOSA.hasFeatureOfInterest, URIRef(entry.subject.uri)))
    rate = URIRef(f"{topic_uri}.rate")
    emit_scalar_quantity(em, rate, float(standing.rate.value), NS_MM_QUDT_QTY["Frequency"], DSL_UNITS[standing.rate.unit].iri)
    em.graph.add((node, SENSORS["update-rate"], rate))
    for index, assignment in enumerate(standing.fields):
        row = URIRef(f"{topic_uri}.f{index}")
        em.graph.add((node, RDFS.member, row))
        em.graph.add((row, ROS["field-path"], Literal(assignment.field_path)))
        em.graph.add((row, RDF.value, view_node(em, assignment, standing)))
    for context in em.model.specs:
        if isinstance(context, ExecutionContext):
            em.graph.add((URIRef(context.uri), RDFS.member, node))


def emit_execution_context(em: Emission, context: ExecutionContext) -> None:
    """The deployment: its scene, control period, config resource, and the systems it deploys."""
    node = URIRef(context.uri)
    em.graph.add((node, RDF.type, EXEC.ExecutionContext))
    # Arranging equipment for a purpose is what SSN calls a deployment.
    em.graph.add((node, RDF.type, SSN.Deployment))
    simulation = context.platform.kind == "simulation"
    em.graph.add((node, RDF.type, EXEC.Simulation if simulation else EXEC.RealWorld))
    em.graph.add((node, EXEC["runs-scene"], URIRef(context.scene.uri)))
    timestep = URIRef(f"{context.uri}.timestep")
    emit_scalar_quantity(em, timestep, float(context.timestep), NS_MM_QUDT_QTY["Time"], DSL_UNITS[context.timestep_unit].iri)
    em.graph.add((node, EXEC.timestep, timestep))
    if context.config:
        config = URIRef(f"{context.uri}.config")
        em.graph.add((config, RDF.type, EXEC.ResourceWithPath))
        em.graph.add((config, RDF.type, EXEC.SystemResource))
        # Only the DSL knows the directory the path was authored relative to.
        declared_in = Path(get_model(context)._tx_filename).resolve()
        em.graph.add((config, EXEC.path, Literal(str((declared_in.parent / context.config).resolve()))))
        em.graph.add((node, EXEC["has-resource"], config))
        em.config_resource = config
    if simulation:
        # The simulator is software with a name, and it answers for every agent the scene declares.
        em.graph.add((node, SDO.name, Literal(context.platform.name)))
        for modelled_agent in context.scene.modelled_agns:
            em.graph.add((node, SSN.deployedSystem, URIRef(modelled_agent.agn.uri)))
        return
    # On hardware each element has its own device: a system this context owns, realizing the element.
    for binding in context.platform.devices:
        # The dotted name the model refers to it by; a sensor's host agent is enough.
        parts, element = [], binding.target
        while element is not None and getattr(element, "name", None):
            parts.append(element.name)
            element = getattr(element, "parent", None)
        fqn = "-".join(reversed(parts[:2]))
        device = URIRef(f"{context.uri}.{fqn}")
        em.graph.add((device, RDF.type, SSN.System))
        em.graph.add((device, SDO.model, Literal(binding.device)))
        em.graph.add((device, EXEC.realizes, URIRef(binding.target.uri)))
        em.graph.add((node, SSN.deployedSystem, device))
