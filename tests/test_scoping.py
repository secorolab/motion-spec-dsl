# SPDX-License-Identifier: MPL-2.0
"""Scene reference resolution and name scoping."""

from __future__ import annotations

from pathlib import Path

import pytest
from rdflib.namespace import RDF
from textx import metamodel_from_file
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.classes.scoping import SceneRefProvider, finalize_imported_scenes
from motion_spec_dsl.langs import motion_spec_metamodel
from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import GEOM_PATH
from support import BASE, BASE_TEXT, HOLD, MODELS, SNAPSHOT

GRAMMAR = Path(__file__).parents[1] / "src/motion_spec_dsl/grammars/model.tx"


@pytest.mark.parametrize(
    ("source", "message"),
    [
        pytest.param(
            """import "01_pick_and_place/pick_and_place.scenex"
ns app = "https://example.org/app/"
context (ns=app) shared {
    world {
        pose p3 { of: <half_arm_2_link>, wrt: <kinova.base_link> }
    }
}
""",
            "Unknown object",
            id="template_internal_as_suffix",
        ),
        pytest.param(
            """ns app = "https://example.org/app/"
context (ns=app) c1 { spec { length support-z = 0.1 m } }
context (ns=app) c2 { spec { length support-z = 0.2 m } }
guarded-motion (ns=app) m1 {
    context { spec { duration d = 5.0 s } }
    when {}
    while { k1: elapsed up to <support-z> }
    until {}
}
""",
            "ambiguous.*c1.spec.support-z.*c2.spec.support-z",
            id="ambiguous_suffix",
        ),
    ],
)
def test_a_scene_reference_resolves_to_one_visible_element(source, message) -> None:
    """Only the scene reference provider is under test, so the grammar loads without the rest."""
    metamodel = metamodel_from_file(GRAMMAR, autokwd=True)
    metamodel.register_scope_providers({"*.*": SceneRefProvider()})
    metamodel.register_model_processor(finalize_imported_scenes)
    with pytest.raises(TextXSemanticError, match=message):
        metamodel.model_from_str(source, file_name=str(MODELS / "probe.robmot"))


def test_two_motions_each_declaring_a_trajectory_get_distinct_path_nodes() -> None:
    """A context quantity's name is scoped to its block, so neither motion's path overwrites
    the other's geometry."""
    source = BASE_TEXT.replace(
        SNAPSHOT,
        f"{SNAPSHOT},\n            path trajectory = lerp {{ start: <spec.home-pose>, goal: <spec.home-pose> }}",
        1,
    ).replace(
        HOLD,
        f"{HOLD},\n        follow: keeping <shared.world.pose-ee-base>.position on <spec.trajectory>"
        " within <shared.spec.satisfied-band>",
        1,
    )
    source += """
guarded-motion (ns=app) away {
    context {
        spec {
            pose away-pose = snapshot of <shared.world.pose-ee-base> on event <aas.E_HOME_ENTERED>,
            path trajectory = lerp { start: <spec.away-pose>, goal: <spec.away-pose> }
        }
    }
    when {}
    while {
        follow: keeping <shared.world.pose-ee-base>.position on <spec.trajectory> within <shared.spec.satisfied-band>
    }
    until {}
}

constraint-handler (ns=app) handler-away {
    handles: <away>
    controllers {
        pid ctrl-follow { constraint: <away.follow>, Kp: 200, Ki: 100, Kd: 40, decay: 0 }
    }
    solvers {
        <handler-home.arm-solver>
    }
}
"""
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph

    assert len(set(graph.subjects(RDF.type, GEOM_PATH.LinearPath))) == 2
