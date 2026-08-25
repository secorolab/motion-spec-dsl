# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A `while` constraint no controller drives and no monitor watches still states a bound.

It gets the same error evaluator every other constraint gets, so the threshold reaches the
generated program as a comparison rather than as a constant nothing reads. The exception is a
pose command: it claims its constraint without an evaluator, because the pose-difference
machinery produces its error per axis.
"""

from __future__ import annotations

from rdflib.namespace import RDF

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import CSTR, CSTR_HDL

SPEC_ANCHOR = "        linear-velocity zero-linvel = 0.0 m/s"
WHILE_ANCHOR = (
    "        hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
)
WORLD_ANCHOR = (
    "        velocity-twist twist-ee-base {\n"
    "            of:         <gripper.g_base.g_pinch>,\n"
    "            wrt:        <kinova.base_link.base_link_origin>,\n"
    "            as-seen-by: <kinova.base_link.base_link_origin>\n"
    "        }"
)
TABLE_POSE = (
    ",\n        pose pose-table-top {\n"
    "            of:         <table.table_top>,\n"
    "            wrt:        <kinova.base_link.base_link_origin>,\n"
    "            as-seen-by: <kinova.base_link.base_link_origin>\n"
    "        }"
)
TABLE_PLANE = (
    ",\n        direction table-normal { as-seen-by: <kinova.base_link.base_link_origin> } "
    "= (0, 0, 1)"
    ",\n        plane table { of: <table.table_top>, normal: <shared.spec.table-normal> }"
)
ABOVE_TABLE = (
    ",\n        above-table: keeping distance of <shared.world.pose-ee-base> from "
    "<shared.spec.table> greater than 0.05 m"
)


def _swap(source: str, anchor: str, replacement: str) -> str:
    assert anchor in source, anchor
    return source.replace(anchor, replacement, 1)


def _name(node) -> str:
    return str(node).rsplit("/", 1)[-1]


def _evaluated(graph) -> dict[str, str]:
    """The constraint each declared evaluator reads, by local name."""
    return {
        _name(node): _name(graph.value(node, CSTR_HDL.constraint))
        for node in graph.subjects(RDF.type, CSTR_HDL.ErrorEvaluator)
        if (None, CSTR_HDL.evaluators, node) in graph
    }


def _graph(parse_source, source: str):
    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


def test_an_undriven_while_constraint_still_gets_its_evaluator(parse_source, base_source) -> None:
    """`above-table` names no controller and no monitor. Without an evaluator its `0.05 m`
    threshold is compared nowhere in the generated program."""
    source = _swap(base_source, WORLD_ANCHOR, WORLD_ANCHOR + TABLE_POSE)
    source = _swap(source, SPEC_ANCHOR, SPEC_ANCHOR + TABLE_PLANE)
    graph = _graph(parse_source, _swap(source, WHILE_ANCHOR, WHILE_ANCHOR + ABOVE_TABLE))

    assert _evaluated(graph).get("eval-home-while-above-table") == "above-table"
    constraint = next(node for node in graph.subjects(CSTR.threshold, None))
    assert _name(constraint) == "above-table"


def test_a_pose_command_keeps_its_constraint_off_the_sweep(parse_source, base_source) -> None:
    """A per-axis pose command produces its error through the pose difference, so its constraint
    must not also collect an evaluator: two writers of one error is what the sweep must avoid."""
    graph = _graph(parse_source, base_source)

    assert "hold-position" not in _evaluated(graph).values()
