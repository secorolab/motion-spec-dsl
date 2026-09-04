# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A `joint-current` world quantity is a joint-referenced electric current, in amperes.

A gripper motor's current is what tells a grasp from a free closure; it only says that if the
emitted quantity carries the actuation type, the joint it belongs to, and the ampere unit --
a bare scalar with no joint would be unreadable by any backend.
"""

from __future__ import annotations

from rdflib import RDF

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import (
    ACT,
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
WORLD_WITH_CURRENT = (
    WORLD_ANCHOR
    + """,
        joint-current gripper-cur {
            joint: <gripper.g_left_driver_joint>
        }"""
)

SPEC_ANCHOR = "        linear-velocity zero-linvel = 0.0 m/s"
SPEC_WITH_CURRENT = (
    SPEC_ANCHOR
    + """,
        current contact-idle = 0.02 A,
        current satisfied-band-cur = 0.001 A"""
)

UNTIL_ANCHOR = (
    "        settled-z: <shared.world.twist-ee-base>.linvel.z equal to "
    "<shared.spec.zero-linvel> within <shared.spec.satisfied-band-vel>"
)
UNTIL_WITH_CURRENT = (
    UNTIL_ANCHOR
    + """,
        fingers-stopped: <shared.world.gripper-cur> less than <shared.spec.contact-idle>"""
)


def _swap(source: str, anchor: str, replacement: str) -> str:
    assert anchor in source, anchor
    return source.replace(anchor, replacement, 1)


def test_a_joint_current_is_an_ampere_quantity_of_its_joint(parse_source, base_source) -> None:
    source = _swap(base_source, WORLD_ANCHOR, WORLD_WITH_CURRENT)
    source = _swap(source, SPEC_ANCHOR, SPEC_WITH_CURRENT)
    source = _swap(source, UNTIL_ANCHOR, UNTIL_WITH_CURRENT)
    graph = MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph

    nodes = list(graph.subjects(RDF.type, ACT.JointCurrent))
    assert len(nodes) == 1, "the joint-current world quantity emitted no act:JointCurrent node"
    node = nodes[0]
    assert (node, RDF.type, QUDT_SCHEMA.Quantity) in graph
    assert (node, RDF.type, KC_STAT.JointReference) in graph
    assert (node, QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND.ElectricCurrent) in graph
    assert (node, QUDT_SCHEMA.unit, QUDT_UNIT.A) in graph
    joint = graph.value(node, KC_STAT["of-joint"])
    assert joint is not None and str(joint).endswith("g_left_driver_joint")
