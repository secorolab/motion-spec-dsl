# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A context quantity's name is scoped to the block declaring it.

Nodes generated from one -- a path's geometry, its projection, a profile's limits -- hang under
that quantity's IRI. Minted in the namespace root instead, two motions that each call their path
`trajectory` would state their geometry on one node: one arc carrying both amplitudes.
"""

from __future__ import annotations

from rdflib.namespace import RDF

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import ALGO_EXT, GEOM_PATH

TRAJECTORY = """,
            path trajectory = lerp {
                start: <spec.home-pose>,
                goal: <spec.home-pose>
            }"""
FOLLOW = """,
        follow: keeping <shared.world.pose-ee-base>.position on <spec.trajectory> within <shared.spec.satisfied-band>"""


def _graph(parse_source, source: str):
    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


def _with_trajectory(base_source: str) -> str:
    """The base model, with `home` driving a path it declares."""
    source = base_source.replace(
        "pose home-pose = snapshot of <shared.world.pose-ee-base> on event <aas.E_HOME_ENTERED>",
        "pose home-pose = snapshot of <shared.world.pose-ee-base> on event <aas.E_HOME_ENTERED>"
        + TRAJECTORY,
    )
    return source.replace(
        "hold-position: keeping <shared.world.pose-ee-base>.position equal to "
        "<spec.home-pose>.position within <shared.spec.satisfied-band>",
        "hold-position: keeping <shared.world.pose-ee-base>.position equal to "
        "<spec.home-pose>.position within <shared.spec.satisfied-band>" + FOLLOW,
    )


def test_path_geometry_hangs_under_the_quantity_that_declares_it(parse_source, base_source) -> None:
    graph = _graph(parse_source, _with_trajectory(base_source))

    (path_node,) = list(graph.subjects(RDF.type, GEOM_PATH.LinearPath))
    quantity = "https://secorolab.github.io/models/base/home/spec/trajectory"
    assert str(path_node) == f"{quantity}/lerp-trajectory"
    # Everything the path implies travels with it, so a second `trajectory` states its own.
    assert all(
        str(node).startswith(f"{quantity}/")
        for node in graph.subjects(RDF.type, GEOM_PATH.Path)
        if "trajectory" in str(node)
    )


SATURATED = (
    "pid ctrl-hold-position { constraint: <home.hold-position>, "
    "output-saturation: saturation { max: <shared.spec.satisfied-band> }, "
    "Kp: 200, Ki: 100, Kd: 40, decay: 0 }"
)
PLAIN = (
    "pid ctrl-hold-position { constraint: <home.hold-position>, "
    "Kp: 200, Ki: 100, Kd: 40, decay: 0 }"
)


def test_what_a_controller_implies_hangs_under_the_controller(parse_source, base_source) -> None:
    """A controller's name is scoped to its handler, so two handlers may each have a
    `ctrl-hold-position`. What is generated from one -- its saturation, its limited output --
    has to hang under it, or both handlers stack their inputs on one node."""
    graph = _graph(parse_source, base_source.replace(PLAIN, SATURATED))

    controller = "https://secorolab.github.io/models/base/handler-home/ctrl-hold-position"
    saturation = _op(graph, ALGO_EXT.Saturation)
    assert str(saturation) == f"{controller}/sat-output-ctrl-hold-position"
    assert all(
        str(node).startswith(f"{controller}/") for node in graph.objects(saturation, ALGO_EXT["in"])
    )


def _op(graph, type_):
    (node,) = list(graph.subjects(RDF.type, type_))
    return node
