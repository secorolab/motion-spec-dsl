# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Mappings from parsed model concepts to their RDF representation."""

from __future__ import annotations

from typing import Any

from rdflib import URIRef
from rdflib.namespace import DefinedNamespace, Namespace, SDO, XSD
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
from rdf_utils.namespace import NS_MM_QUDT_QTY, NS_MM_QUDT_UNIT as QUDT_UNIT

from motion_spec_dsl.rdf_parser.vocab import (
    ALGO_EXT,
    AGN,
    APP,
    EL,
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
    GEOM_REL_EXT,
    KC_STAT,
    MAP,
    MAP_EXT,
    MOT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    QKIND_EXT,
    RBDYN_COORD,
    RBDYN_ENT,
    RBDYN_OP,
    RBDYN_OP_EXT,
    SLV,
    SLV_EXT,
    SOSA,
    SSN,
    TIME,
)
from motion_spec_dsl.classes.context import QuantityType, WorldQuantityType


class ROS(DefinedNamespace):
    Action: URIRef
    Topic: URIRef
    Action: URIRef
    _extras = ["channel-name", "type-name", "field-path"]
    _NS = Namespace("https://index.ros.org/p/")


# comp-rob2b's relation/coordinate split, per geometry domain
# (coordinates.ttl:8-73): (<Domain>Reference, <Domain>Coordinate, of-<domain>, relation class)
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
                MAP_EXT.VelocityTwistCoordinateView,
            ),
            "linear": (
                "linear-velocity",
                "linear-acceleration",
                "lin",
                QuantityType.LinearVelocity,
                MAP_EXT.VelocityTwistCoordinateView,
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
            "torque": ("torque", None, None, QuantityType.Torque, MAP_EXT.WrenchCoordinateView),
            "force": ("force", None, None, QuantityType.Force, MAP_EXT.WrenchCoordinateView),
        },
    ),
    WorldQuantityType.Pose: (
        (
            QUDT_SCHEMA.Quantity,
            GEOM_REL.Pose,
            GEOM_COORD.PoseCoordinate,
            GEOM_COORD.VectorXYZ,
        ),
        (QUDT_QKIND.PlaneAngle, URI_QUDT_QK_LENGTH),
        (QUDT_UNIT["RAD"], QUDT_UNIT.M),
        {
            "rotation": (
                "rotation",
                "angular-acceleration",
                "ang",
                QuantityType.PlaneAngle,
                MAP_EXT.PoseCoordinateView,
            ),
            "distance": (
                "position",
                "linear-acceleration",
                "lin",
                QuantityType.Distance,
                MAP_EXT.PoseCoordinateView,
            ),
        },
    ),
    WorldQuantityType.JointPosition: (
        (QUDT_SCHEMA.Quantity, KC_STAT.JointReference, KC_STAT.JointPositionCoordinate),
        (QUDT_QKIND.PlaneAngle,),
        (QUDT_UNIT.RAD,),
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
}

CSTR_TYPE_NAME: dict[Any, str] = {
    QuantityType.PlaneAngle: QuantityType.Angle,
}

# Domain-constraint types whose grounded IRI is not `cstr:{Type}Constraint`:
# a distance is the comp-rob2b LinearDistanceConstraint; an angle has no
# comp-rob2b equivalent and lives in the secorolab extension. (namespace, local-name)
CONSTRAINT_TYPE_OVERRIDE: dict[Any, tuple[Any, str]] = {
    # A length constrained against a bound is comp-rob2b's linear-distance constraint, whether
    # the model calls the value a length or a distance.
    QuantityType.Length: (CSTR, "LinearDistanceConstraint"),
    QuantityType.Distance: (CSTR, "LinearDistanceConstraint"),
    QuantityType.Angle: (CSTR_EXT, "AngleConstraint"),
}

QUDT_KIND_BY_QUANTITY_TYPE: dict[Any, Any] = {
    QuantityType.Pose: GEOM_REL.Pose,
    QuantityType.Length: QUDT_QKIND.Length,
    QuantityType.Position: QUDT_QKIND.Position,
    QuantityType.Orientation: QUDT_QKIND.PlaneAngle,
    QuantityType.Angle: QUDT_QKIND.PlaneAngle,
    QuantityType.PlaneAngle: QUDT_QKIND.PlaneAngle,
    QuantityType.VelocityTwist: GEOM_REL.VelocityTwist,
    QuantityType.AccelerationTwist: GEOM_REL.AccelerationTwist,
    QuantityType.Wrench: RBDYN_ENT.Wrench,
    QuantityType.Direction: NS_MM_QUDT_QTY["Dimensionless"],
    QuantityType.FreeVector: NS_MM_QUDT_QTY["FreeVector"],
    QuantityType.Dimensionless: NS_MM_QUDT_QTY["Dimensionless"],
    QuantityType.Duration: NS_MM_QUDT_QTY["Time"],
    QuantityType.PathParameter: NS_MM_QUDT_QTY["Dimensionless"],
    QuantityType.LinearJerk: QKIND_EXT.LinearJerk,
}

# Quantity kinds are individuals, not classes; these namespaces distinguish them
# from structural kinds (geom-rel:Pose, rbdyn-ent:Wrench, …) during emission.
_QKIND_PREFIXES = (str(QUDT_QKIND), str(QKIND_EXT))

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
