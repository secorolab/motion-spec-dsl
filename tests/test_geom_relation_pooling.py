# SPDX-License-Identifier: MPL-2.0
"""Plan 11 phase B: `_emit_geom_relation` dedupes on (domain, of, wrt) -- one relation per frame
pair, not one per coordinate."""

from __future__ import annotations

from pathlib import Path

from rdf_utils.models.geom_rel import URI_GEOM_TYPE_POSE
from rdf_utils.models.vocab import URI_GEOM_PRED_OF_POSE
from rdflib import RDF

from motion_spec_dsl.langs import motion_spec_metamodel
from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import CSTR, MAP

MODELS = Path(__file__).parents[1] / "models"


def _pick_place_single_graph():
    model = motion_spec_metamodel().model_from_file(
        MODELS / "pick_place_single" / "pick_place_single.robmot"
    )
    return MotionSpecDatasetBuilder(model).build()[0].default_graph


def test_one_relation_per_frame_pair():
    """16 Pose coordinates over 4 distinct (of, wrt) frame pairs now yield 4 Pose relations."""
    graph = _pick_place_single_graph()
    poses = set(graph.subjects(RDF.type, URI_GEOM_TYPE_POSE))
    assert len(poses) == 4


def test_coordinates_of_one_frame_pair_share_their_relation():
    """A measured pose and a config-authored goal pose stated `for` it are two coordinates of
    the same frame pair -- after phase B they point at one relation node, not two."""
    graph = _pick_place_single_graph()
    pose_ee_base = next(
        s for s in graph.subjects() if str(s).endswith("/shared/world/pose-ee-base")
    )
    home_pose = next(s for s in graph.subjects() if str(s).endswith("/shared/spec/home-pose"))

    ee_relation = graph.value(pose_ee_base, URI_GEOM_PRED_OF_POSE)
    home_relation = graph.value(home_pose, URI_GEOM_PRED_OF_POSE)

    assert ee_relation is not None
    assert ee_relation == home_relation


def test_a_constraint_names_a_view_per_operand():
    """Pooling made the relation 1-to-N, so a constraint's two operands over one frame pair
    would be the same node. Each edge names the view that pins its own coordinate instead."""
    graph = _pick_place_single_graph()
    constraints = {
        s for s in graph.subjects(RDF.type, CSTR.Constraint) if str(s).endswith("hold-position")
    }
    assert len(constraints) == 1
    constraint = constraints.pop()

    quantity_view = graph.value(constraint, CSTR.quantity)
    reference_view = graph.value(constraint, CSTR["reference-value"])
    assert quantity_view is not None
    assert quantity_view != reference_view

    pose_ee_base = next(
        s for s in graph.subjects() if str(s).endswith("/shared/world/pose-ee-base")
    )
    home_pose = next(s for s in graph.subjects() if str(s).endswith("/shared/spec/home-pose"))
    assert graph.value(quantity_view, MAP.superobject) == pose_ee_base
    assert graph.value(reference_view, MAP.superobject) == home_pose

    # Pooling stands: one relation for the frame pair, viewed twice.
    assert graph.value(quantity_view, MAP.subobject) == graph.value(reference_view, MAP.subobject)
    assert graph.value(quantity_view, MAP.subobject) is not None
