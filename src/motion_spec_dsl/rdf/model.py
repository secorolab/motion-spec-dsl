# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Mappings from parsed model concepts to their RDF representation."""

from __future__ import annotations

from typing import Any, NamedTuple

from rdf_utils.models.vocab import (
    URI_GEOM_PRED_OF_ORIENT,
    URI_GEOM_PRED_OF_POSE,
    URI_GEOM_PRED_OF_POSITION,
    URI_GEOM_TYPE_ORIENT,
    URI_GEOM_TYPE_ORIENT_COORD,
    URI_GEOM_TYPE_ORIENT_REF,
    URI_GEOM_TYPE_POSE,
    URI_GEOM_TYPE_POSE_COORD,
    URI_GEOM_TYPE_POSE_REF,
    URI_GEOM_TYPE_POSITION,
    URI_GEOM_TYPE_POSITION_COORD,
    URI_GEOM_TYPE_POSITION_REF,
    URI_QUDT_QK_LENGTH,
)
from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT
from rdflib import URIRef
from rdflib.namespace import SDO, SSN, XSD, DefinedNamespace, Namespace

from motion_spec_dsl.classes.context import QuantityType, WorldQuantityType
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
    GEOM_REL_EXT,
    KC_STAT,
    MAP,
    MAP_EXT,
    MOT,
    QKIND_EXT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    RBDYN_COORD,
    RBDYN_ENT,
    RBDYN_OP,
    RBDYN_OP_EXT,
    SIM,
    SLV,
    SLV_EXT,
    SOSA,
    TIME,
)


class ROS(DefinedNamespace):
    Action: URIRef
    Topic: URIRef
    _extras = ("channel-name", "type-name", "field-path")
    _NS = Namespace("https://index.ros.org/p/")


# comp-rob2b's relation/coordinate split: (<Domain>Reference, <Domain>Coordinate, of-<domain>, relation).
GEOM_DOMAIN_SPLIT: dict[str, tuple[URIRef, URIRef, URIRef, URIRef]] = {
    "position": (
        URI_GEOM_TYPE_POSITION_REF,
        URI_GEOM_TYPE_POSITION_COORD,
        URI_GEOM_PRED_OF_POSITION,
        URI_GEOM_TYPE_POSITION,
    ),
    "orientation": (
        URI_GEOM_TYPE_ORIENT_REF,
        URI_GEOM_TYPE_ORIENT_COORD,
        URI_GEOM_PRED_OF_ORIENT,
        URI_GEOM_TYPE_ORIENT,
    ),
    "pose": (
        URI_GEOM_TYPE_POSE_REF,
        URI_GEOM_TYPE_POSE_COORD,
        URI_GEOM_PRED_OF_POSE,
        URI_GEOM_TYPE_POSE,
    ),
}


class WorldView(NamedTuple):
    """One subspace of a world quantity, viewed as a scalar: its map subspace and view class."""

    subspace: str
    view_type: URIRef


class WorldSpec(NamedTuple):
    """A world quantity type's RDF types, quantity kinds and units, and its subspace views."""

    rdf_types: tuple
    kinds: tuple
    units: tuple
    views: dict[str, WorldView]


WORLD_SPECS: dict[WorldQuantityType, WorldSpec] = {
    WorldQuantityType.VelocityTwist: WorldSpec(
        (
            QUDT_SCHEMA.Quantity,
            GEOM_REL.VelocityTwist,
            GEOM_COORD.VelocityTwistCoordinate,
            GEOM_COORD.VectorXYZ,
        ),
        (QUDT_QKIND.AngularVelocity, QUDT_QKIND.LinearVelocity),
        (QUDT_UNIT["RAD-PER-SEC"], QUDT_UNIT["M-PER-SEC"]),
        {
            "angular": WorldView("angular-velocity", MAP_EXT.VelocityTwistCoordinateView),
            "linear": WorldView("linear-velocity", MAP_EXT.VelocityTwistCoordinateView),
        },
    ),
    WorldQuantityType.Wrench: WorldSpec(
        (
            QUDT_SCHEMA.Quantity,
            RBDYN_ENT.Wrench,
            RBDYN_COORD.WrenchCoordinate,
            GEOM_COORD.VectorXYZ,
        ),
        (QUDT_QKIND.Torque, QUDT_QKIND.Force),
        (QUDT_UNIT["N-M"], QUDT_UNIT.N),
        {
            "torque": WorldView("torque", MAP_EXT.WrenchCoordinateView),
            "force": WorldView("force", MAP_EXT.WrenchCoordinateView),
        },
    ),
    WorldQuantityType.Pose: WorldSpec(
        (
            QUDT_SCHEMA.Quantity,
            GEOM_REL.Pose,
            GEOM_COORD.PoseCoordinate,
            GEOM_COORD.VectorXYZ,
        ),
        (QUDT_QKIND.PlaneAngle, URI_QUDT_QK_LENGTH),
        (QUDT_UNIT["RAD"], QUDT_UNIT.M),
        {
            "rotation": WorldView("rotation", MAP_EXT.PoseCoordinateView),
            "distance": WorldView("position", MAP_EXT.PoseCoordinateView),
        },
    ),
    WorldQuantityType.JointPosition: WorldSpec(
        (QUDT_SCHEMA.Quantity, KC_STAT.JointReference, KC_STAT.JointPositionCoordinate),
        (QUDT_QKIND.PlaneAngle,),
        (QUDT_UNIT.RAD,),
        {},
    ),
    WorldQuantityType.JointVelocity: WorldSpec(
        (QUDT_SCHEMA.Quantity, KC_STAT.JointReference, KC_STAT.JointVelocityCoordinate),
        (QUDT_QKIND.AngularVelocity,),
        (QUDT_UNIT["RAD-PER-SEC"],),
        {},
    ),
    WorldQuantityType.JointForce: WorldSpec(
        (QUDT_SCHEMA.Quantity, KC_STAT.JointReference, KC_STAT.JointForceCoordinate),
        (QUDT_QKIND.Torque,),
        (QUDT_UNIT["N-M"],),
        {},
    ),
}

SCALAR_UNIT: dict[Any, Any] = {
    QuantityType.Pose: QUDT_UNIT.UNITLESS,
    QuantityType.Position: QUDT_UNIT.M,
    QuantityType.Orientation: QUDT_UNIT["RAD"],
    QuantityType.Length: QUDT_UNIT.M,
    QuantityType.Distance: QUDT_UNIT.M,
    QuantityType.Angle: QUDT_UNIT["RAD"],
    QuantityType.PlaneAngle: QUDT_UNIT["RAD"],
    QuantityType.LinearVelocity: QUDT_UNIT["M-PER-SEC"],
    QuantityType.AngularVelocity: QUDT_UNIT["RAD-PER-SEC"],
    QuantityType.LinearAcceleration: QUDT_UNIT["M-PER-SEC2"],
    QuantityType.AngularAcceleration: QUDT_UNIT["RAD-PER-SEC2"],
    QuantityType.LinearJerk: QUDT_UNIT["M-PER-SEC3"],
    QuantityType.Force: QUDT_UNIT.N,
    QuantityType.Torque: QUDT_UNIT["N-M"],
    QuantityType.Dimensionless: QUDT_UNIT.UNITLESS,
    QuantityType.PathParameter: QUDT_UNIT.UNITLESS,
    QuantityType.Duration: QUDT_UNIT["SEC"],
    QuantityType.Mass: QUDT_UNIT["KiloGM"],
}

CSTR_TYPE_NAME: dict[Any, str] = {
    QuantityType.PlaneAngle: QuantityType.Angle,
}

# Constraint types not named `cstr:{Type}Constraint`: a length or distance is comp-rob2b's
# LinearDistanceConstraint, an angle or orientation the secorolab extension's.
CONSTRAINT_TYPE_OVERRIDE: dict[Any, tuple[Any, str]] = {
    QuantityType.Length: (CSTR, "LinearDistanceConstraint"),
    QuantityType.Distance: (CSTR, "LinearDistanceConstraint"),
    QuantityType.Angle: (CSTR_EXT, "AngleConstraint"),
    QuantityType.Orientation: (CSTR_EXT, "OrientationConstraint"),
}

# Quantity kinds are individuals, not classes, unlike structural kinds such as geom-rel:Pose.
QKIND_PREFIXES = (str(QUDT_QKIND), str(QKIND_EXT))

CONTEXT_COMPOSITE_WORLD_TYPE: dict[QuantityType, WorldQuantityType] = {
    QuantityType.Pose: WorldQuantityType.Pose,
    QuantityType.VelocityTwist: WorldQuantityType.VelocityTwist,
    QuantityType.Wrench: WorldQuantityType.Wrench,
}

GRAPH_BINDINGS: tuple[tuple[str, Any], ...] = (
    ("algo-ext", ALGO_EXT),
    ("application", APP),
    ("agn", AGN),
    ("kc-stat", KC_STAT),
    ("geom-ent", GEOM_ENT),
    ("geom-rel", GEOM_REL),
    ("geom-coord", GEOM_COORD),
    ("geom-op", GEOM_OP),
    ("geom-op-ext", GEOM_OP_EXT),
    ("geom-rel-ext", GEOM_REL_EXT),
    ("el", EL),
    ("exec", EXEC),
    ("rbdyn-ent", RBDYN_ENT),
    ("rbdyn-coord", RBDYN_COORD),
    ("rbdyn-op", RBDYN_OP),
    ("rbdyn-op-ext", RBDYN_OP_EXT),
    ("qudt", QUDT_SCHEMA),
    ("qkind", QUDT_QKIND),
    ("unit", QUDT_UNIT),
    ("map", MAP),
    ("cstr", CSTR),
    ("cstr-ext", CSTR_EXT),
    ("map-ext", MAP_EXT),
    ("mot", MOT),
    ("cstr-hdl", CSTR_HDL),
    ("cstr-hdl-ext", CSTR_HDL_EXT),
    ("qkind-ext", QKIND_EXT),
    ("sim", SIM),
    ("slv", SLV),
    ("slv-ext", SLV_EXT),
    ("sosa", SOSA),
    ("ssn", SSN),
    ("geom-path", GEOM_PATH),
    ("ros", ROS),
    ("time", TIME),
    ("schema", SDO),
    ("xsd", XSD),
)
