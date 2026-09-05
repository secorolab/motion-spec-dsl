# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A `joint-velocity` world quantity is a joint-referenced angular velocity, in rad/s.

Fingers that have stopped short of their commanded closure are a grasp; the stop is read off
the joint's rate, so the emitted quantity must carry the velocity coordinate type, the joint it
belongs to, and the rad/s unit -- a bare scalar with no joint would be unreadable by any backend.
"""

from __future__ import annotations

from rdflib import RDF

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import (
    KC_STAT,
    QUDT_QKIND,
    QUDT_SCHEMA,
)
from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT

WORLD_ANCHOR = """        velocity-twist twist-ee-base {
            of:         <gripper.g_base.g_pinch>,
            wrt:        <kinova.base_link.base_link_origin>,
            as-seen-by: <kinova.base_link.base_link_origin>
        }"""
WORLD_WITH_VELOCITY = (
    WORLD_ANCHOR
    + """,
        joint-velocity gripper-vel {
            joint: <gripper.g_left_driver_joint>
        }"""
)

SPEC_ANCHOR = "        linear-velocity zero-linvel = 0.0 m/s"
SPEC_WITH_VELOCITY = (
    SPEC_ANCHOR
    + """,
        angular-velocity zero-angvel = 0.0 rad/s,
        angular-velocity stall-band = 0.05 rad/s"""
)

UNTIL_ANCHOR = (
    "        settled-z: <shared.world.twist-ee-base>.linvel.z equal to "
    "<shared.spec.zero-linvel> within <shared.spec.satisfied-band-vel>"
)
UNTIL_WITH_VELOCITY = (
    UNTIL_ANCHOR
    + """,
        fingers-stopped: <shared.world.gripper-vel> equal to <shared.spec.zero-angvel> within <shared.spec.stall-band>"""
)


def _swap(source: str, anchor: str, replacement: str) -> str:
    assert anchor in source, anchor
    return source.replace(anchor, replacement, 1)


def test_a_joint_velocity_is_a_rad_per_sec_quantity_of_its_joint(parse_source, base_source) -> None:
    source = _swap(base_source, WORLD_ANCHOR, WORLD_WITH_VELOCITY)
    source = _swap(source, SPEC_ANCHOR, SPEC_WITH_VELOCITY)
    source = _swap(source, UNTIL_ANCHOR, UNTIL_WITH_VELOCITY)
    graph = MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph

    nodes = list(graph.subjects(RDF.type, KC_STAT.JointVelocityCoordinate))
    assert len(nodes) == 1, "the joint-velocity world quantity emitted no JointVelocityCoordinate"
    node = nodes[0]
    assert (node, RDF.type, QUDT_SCHEMA.Quantity) in graph
    assert (node, RDF.type, KC_STAT.JointReference) in graph
    assert (node, QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND.AngularVelocity) in graph
    assert (node, QUDT_SCHEMA.unit, QUDT_UNIT["RAD-PER-SEC"]) in graph
    joint = graph.value(node, KC_STAT["of-joint"])
    assert joint is not None and str(joint).endswith("g_left_driver_joint")
