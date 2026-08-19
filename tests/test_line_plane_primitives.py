# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""`line` and `plane` compose a frame origin with a declared unit direction."""

from __future__ import annotations

import pytest
from rdflib.namespace import RDF
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import GEOM_ENT, GEOM_EXT

SPEC_ANCHOR = "        linear-velocity zero-linvel = 0.0 m/s"
BASE_DIRECTION = (
    ",\n        direction table-normal "
    "{ as-seen-by: <kinova.base_link.base_link_origin> } = (0, 0, 1)"
)


def _source(base_source: str, *declarations: str) -> str:
    return base_source.replace(SPEC_ANCHOR, SPEC_ANCHOR + "".join(declarations), 1)


def _graph(parse_source, source: str):
    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


def test_plane_emits_entity_with_origin_and_normal(parse_source, base_source) -> None:
    source = _source(
        base_source,
        BASE_DIRECTION,
        ",\n        plane table { of: <gripper.g_base.g_pinch>, "
        "normal: <shared.spec.table-normal> }",
    )
    graph = _graph(parse_source, source)

    planes = list(graph.subjects(RDF.type, GEOM_EXT.Plane))
    assert len(planes) == 1
    normal = graph.value(planes[0], GEOM_EXT.normal)
    assert normal is not None and str(normal).endswith("table-normal")
    assert graph.value(planes[0], GEOM_ENT.origin) is not None


def test_line_emits_entity_with_origin_and_direction(parse_source, base_source) -> None:
    source = _source(
        base_source,
        BASE_DIRECTION.replace("table-normal", "rail-axis").replace("(0, 0, 1)", "(1, 0, 0)"),
        ",\n        line rail { of: <gripper.g_base.g_pinch>, along: <shared.spec.rail-axis> }",
    )
    graph = _graph(parse_source, source)

    lines = list(graph.subjects(RDF.type, GEOM_EXT.Line))
    assert len(lines) == 1
    direction = graph.value(lines[0], GEOM_EXT.direction)
    assert direction is not None and str(direction).endswith("rail-axis")
    assert graph.value(lines[0], GEOM_ENT.origin) is not None


def test_plane_rejects_along(parse_source, base_source) -> None:
    source = _source(
        base_source,
        BASE_DIRECTION,
        ",\n        plane table { of: <gripper.g_base.g_pinch>, "
        "along: <shared.spec.table-normal> }",
    )
    with pytest.raises(TextXSemanticError, match="is not one of them"):
        parse_source(source)


def test_plane_normal_must_be_a_direction(parse_source, base_source) -> None:
    source = _source(
        base_source,
        ",\n        plane table { of: <gripper.g_base.g_pinch>, "
        "normal: <shared.spec.satisfied-band> }",
    )
    with pytest.raises(TextXSemanticError, match="must be a 'direction' context quantity"):
        parse_source(source)


def test_plane_requires_normal(parse_source, base_source) -> None:
    source = _source(base_source, ",\n        plane table { of: <gripper.g_base.g_pinch> }")
    with pytest.raises(TextXSemanticError, match="needs exactly one 'normal'"):
        parse_source(source)
