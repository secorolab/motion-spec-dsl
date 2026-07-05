# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Constant tables and namespace bindings for RDF emission."""

from __future__ import annotations

from typing import Any



from motion_spec.namespace import (
    EL,
    CSTR,
    CSTR_EXT,
    CSTR_HDL,
    CSTR_HDL_EXT,
    ENV,
    EXEC,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_OP,
    GEOM_OP_EXT,
    GEOM_REL,
    KC,
    KC_STAT,
    MAP,
    MAP_EXT,
    MJ,
    MOT,
    POLY,
    TRAJ,
    QUDT_QKIND,
    QUDT_SCHEMA,
    QUDT_UNIT,
    RBDYN_COORD,
    RBDYN_ENT,
    RBDYN_OP,
    RBDYN_OP_EXT,
    RT,
    SNAP,
    SLV,
    SLV_EXT,
)
from motion_spec_dsl.domain import (
    QuantityType,
    WorldEntityType,
    WorldFieldType,
    WorldQuantityType,
)


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
                MAP.VelocityTwistCoordinateView,
            ),
            "linear": (
                "linear-velocity",
                "linear-acceleration",
                "lin",
                QuantityType.LinearVelocity,
                MAP.VelocityTwistCoordinateView,
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
            "torque": ("torque", None, None, QuantityType.Torque, MAP.WrenchCoordinateView),
            "force": ("force", None, None, QuantityType.Force, MAP.WrenchCoordinateView),
        },
    ),
    WorldQuantityType.Pose: (
        (
            QUDT_SCHEMA.Quantity,
            GEOM_REL.Pose,
            GEOM_COORD.PoseCoordinate,
            GEOM_COORD.DirectionCosineXYZ,
            GEOM_COORD.VectorXYZ,
        ),
        (QUDT_QKIND.PlaneAngle, QUDT_QKIND.Length),
        (QUDT_UNIT.UNITLESS, QUDT_UNIT.M),
        {
            "rotation": (
                "rotation",
                "angular-acceleration",
                "ang",
                QuantityType.PlaneAngle,
                MAP_EXT.PoseOrientationView,
            ),
            "distance": (
                "position",
                "linear-acceleration",
                "lin",
                QuantityType.Distance,
                MAP.PoseCoordinateView,
            ),
        },
    ),
    WorldQuantityType.JointPosition: (
        (QUDT_SCHEMA.Quantity, KC_STAT.JointPositionCoordinate),
        (QUDT_QKIND.PlaneAngle,),
        (QUDT_UNIT.RAD,),
        {},
    ),
}

WORLD_STRUCTURE_TYPES: dict[Any, Any] = {
    WorldEntityType.Frame: GEOM_ENT.Frame,
    WorldEntityType.Link: GEOM_ENT.SimplicialComplex,
    WorldEntityType.SceneObject: ENV.RigidObject,
    WorldEntityType.KinematicChain: GEOM_ENT.KinematicChain,
    WorldFieldType.Gravity: GEOM_ENT.UniformGravitationalField,
}

SCALAR_UNIT: dict[Any, Any] = {
    QuantityType.Pose: QUDT_UNIT.UNITLESS,
    QuantityType.Position: QUDT_UNIT.M,
    QuantityType.Orientation: QUDT_UNIT["RAD"],
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
    QuantityType.TrajectoryProgress: QUDT_UNIT.UNITLESS,
    QuantityType.Duration: QUDT_UNIT["SEC"],
}

DSL_UNIT: dict[str, Any] = {
    "rad/s": QUDT_UNIT["RAD-PER-SEC"],
    "rad": QUDT_UNIT["RAD"],
    "m/s": QUDT_UNIT["M-PER-SEC"],
    "m/s3": QUDT_UNIT["M-PER-SEC3"],
    "m": QUDT_UNIT.M,
    "Nm": QUDT_UNIT["N-M"],
    "N": QUDT_UNIT.N,
    "deg/s": QUDT_UNIT["DEG-PER-SEC"],
    "deg": QUDT_UNIT["DEG"],
    "cm/s": QUDT_UNIT["CentiM-PER-SEC"],
    "cm": QUDT_UNIT["CentiM"],
    "m/s2": QUDT_UNIT["M-PER-SEC2"],
    "rad/s2": QUDT_UNIT["RAD-PER-SEC2"],
    "s": QUDT_UNIT["SEC"],
    "ms": QUDT_UNIT["MilliSEC"],
    "1": QUDT_UNIT.UNITLESS,
}

CONSTRAINT_PATH_BY_PREFIX = {
    "geom-rel": "https://comp-rob2b.github.io/metamodels/geometry/spatial-relations.ttl",
    "geom-coord": "https://comp-rob2b.github.io/metamodels/geometry/coordinates.ttl",
    "geom-op": "https://secorolab.github.io/metamodels/geometry/spatial-operators.shacl.ttl",
    "rbdyn-op": "https://comp-rob2b.github.io/metamodels/newtonian-rigid-body-dynamics/operators.ttl",
    # de-punned overrides in the secorolab layer (comp-rob2b bases pun quantity
    # kinds as rdf:type via sh:class; these fork them to hasQuantityKind checks).
    "map": "https://secorolab.github.io/metamodels/task/map.shacl.ttl",
    "cstr": "https://secorolab.github.io/metamodels/task/constraint.shacl.ttl",
    "mot": "https://secorolab.github.io/metamodels/task/motion-specification.shacl.ttl",
    "cstr-hdl": "https://secorolab.github.io/metamodels/task/constraint-handler.shacl.ttl",
    "slv": "https://secorolab.github.io/metamodels/task/solver-specification.shacl.ttl",
}

CSTR_TYPE_NAME: dict[Any, str] = {
    QuantityType.PlaneAngle: QuantityType.Angle,
}

QUDT_KIND_BY_QUANTITY_TYPE: dict[Any, Any] = {
    QuantityType.Pose: GEOM_REL.Pose,
    QuantityType.Position: QUDT_QKIND.Position,
    QuantityType.Orientation: QUDT_QKIND.PlaneAngle,
    QuantityType.VelocityTwist: GEOM_REL.VelocityTwist,
    QuantityType.AccelerationTwist: GEOM_REL.AccelerationTwist,
    QuantityType.Wrench: RBDYN_ENT.Wrench,
    QuantityType.Direction: QUDT_QKIND.Dimensionless,
    QuantityType.FreeVector: QUDT_QKIND.FreeVector,
    QuantityType.Dimensionless: QUDT_QKIND.Dimensionless,
    QuantityType.Duration: QUDT_QKIND.Time,
    QuantityType.TrajectoryProgress: TRAJ.Progress,
    QuantityType.LinearJerk: CSTR_HDL_EXT.LinearJerk,
}

# QUDT quantity-kinds are individuals, not classes; used to tell them apart from
# structural kinds (geom-rel:Pose, rbdyn-ent:Wrench, …) when emitting typing.
_QKIND_PREFIX = str(QUDT_QKIND)

CONTEXT_COMPOSITE_WORLD_TYPE: dict[QuantityType, WorldQuantityType] = {
    QuantityType.Pose: WorldQuantityType.Pose,
    QuantityType.VelocityTwist: WorldQuantityType.VelocityTwist,
    QuantityType.Wrench: WorldQuantityType.Wrench,
}

GRAPH_BINDINGS: tuple[tuple[str, Any], ...] = (
    ("kc", KC),
    ("kc-stat", KC_STAT),
    ("geom-ent", GEOM_ENT),
    ("geom-rel", GEOM_REL),
    ("geom-coord", GEOM_COORD),
    ("geom-op", GEOM_OP),
    ("geom-op-ext", GEOM_OP_EXT),
    ("env", ENV),
    ("exec", EXEC),
    ("el", EL),
    ("rt", RT),
    ("mj", MJ),
    ("poly", POLY),
    ("snap", SNAP),
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
    ("slv", SLV),
    ("slv-ext", SLV_EXT),
)
