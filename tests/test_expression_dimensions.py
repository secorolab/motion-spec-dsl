# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Quantity expressions: their ALGO op chain, inferred dimension, and where a product may stand."""

from __future__ import annotations

from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT
from rdflib.namespace import RDF

from motion_spec_dsl.langs import motion_spec_metamodel
from motion_spec_dsl.rdf.dataset import build_dataset
from motion_spec_dsl.rdf_parser.vocab import ALGO_EXT, QUDT_QKIND, QUDT_SCHEMA
from support import BASE, BASE_TEXT, SNAPSHOT, SPEC, TWIST, UNTIL


def test_precedence_nests_a_product_inside_a_difference_with_an_inferred_force() -> None:
    source = BASE_TEXT.replace(
        SPEC,
        f"{SPEC},\n        force f = 10.0 N,\n        mass k = 1.2 kg,\n"
        "        linear-acceleration a = 9.81 m/s^2,\n"
        "        force residual = <spec.f> - <spec.k> * <spec.a>",
        1,
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph

    [subtraction] = graph.subjects(RDF.type, ALGO_EXT.Subtraction)
    [multiplication] = graph.subjects(RDF.type, ALGO_EXT.Multiplication)
    product = graph.value(multiplication, ALGO_EXT.out)
    assert graph.value(subtraction, ALGO_EXT.subtrahend) == product
    assert graph.value(product, QUDT_SCHEMA.hasQuantityKind) == QUDT_QKIND.Force
    assert graph.value(product, QUDT_SCHEMA.unit) == QUDT_UNIT.N


def test_a_minus_offset_names_the_operand_it_takes_away() -> None:
    """`-` does not commute, so the sampled value is the minuend and the offset the subtrahend."""
    source = BASE_TEXT.replace(
        SNAPSHOT,
        f"{SNAPSHOT},\n            length lift = 0.05 m,\n"
        "            length support-z = snapshot of <shared.world.pose-ee-base>.position.z "
        "- <spec.lift> on event <aas.E_HOME_ENTERED>",
        1,
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph

    [subtraction] = graph.subjects(RDF.type, ALGO_EXT.Subtraction)
    assert str(graph.value(subtraction, ALGO_EXT.subtrahend)).endswith("/lift")
    assert str(graph.value(subtraction, ALGO_EXT.minuend)).endswith("pose-ee-base.distance.z")


def test_a_product_of_measured_views_is_accepted_where_only_a_monitor_reads_it() -> None:
    """A controller needs the gradient such a product lacks; a monitor only compares it."""
    source = (
        BASE_TEXT.replace(
            TWIST,
            TWIST + ",\n        wrench press-wrench {\n"
            "            of:         <gripper.g_base.g_pinch>,\n"
            "            ref-point:  <gripper.g_base.g_pinch>,\n"
            "            as-seen-by: <kinova.base_link.base_link_origin>\n        }",
            1,
        )
        .replace(SPEC, f"{SPEC},\n        torque tq-lo = -1.0 Nm,\n        torque tq-hi = 1.0 Nm", 1)
        .replace(
            UNTIL,
            f"{UNTIL},\n        torque-load: (<shared.world.press-wrench>.force.z * "
            "<shared.world.pose-ee-base>.position.x) outside <spec.tq-lo> and <spec.tq-hi>",
            1,
        )
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    build_dataset(model)
