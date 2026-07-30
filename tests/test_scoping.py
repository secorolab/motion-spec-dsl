# SPDX-License-Identifier: MPL-2.0
"""Scene reference resolution: instanced-tree heads, unique-suffix short forms."""

from __future__ import annotations

from pathlib import Path

import pytest
from textx import metamodel_from_file
from textx.exceptions import TextXSemanticError
from textx.scoping import providers as scoping_providers

from motion_spec_dsl.registration import motion_spec_metamodel
from motion_spec_dsl.scoping import SceneRefProvider, _fqn, finalize_imported_scenes

GRAMMAR = Path(__file__).parents[1] / "src/motion_spec_dsl/grammars/model.tx"
MODELS = Path(__file__).parents[1] / "models"

HEAD = """
import "pick_place_single/pick_place_single.scenex"
ns app = "https://example.org/app/"
context (ns=app) shared {
    world {
"""
TAIL = """
    }
}
"""


def parse(world_items: str):
    # Grammar-level metamodel: no user classes, so registration's domain-coupled
    # providers are left off; only the providers under test are registered.
    mm = metamodel_from_file(GRAMMAR, autokwd=True)
    mm.register_scope_providers(
        {
            "*.*": SceneRefProvider(),
            "EventName.event": scoping_providers.PlainNameImportURI(),
        }
    )
    mm.register_model_processor(finalize_imported_scenes)
    return mm.model_from_str(HEAD + world_items + TAIL, file_name=str(MODELS / "probe.robmot"))


def test_instance_head_ref_lands_on_the_instance_copy():
    model = parse("pose p1 {\n    of: <kinova.half_arm_2_link>,\n    wrt: <kinova.base_link>\n}")
    declaration = model.specs[0].context[0].declaration[0]
    body = declaration.props.pairs[0].frame.parent
    assert _fqn(body) == "kinova.half_arm_2_link"
    assert body.parent.template is not None


def test_frame_refs_through_two_instances():
    model = parse(
        "pose p1 {\n"
        "    of: <gripper.g_base.g_pinch>,\n"
        "    wrt: <kinova.base_link.base_link_origin>,\n"
        "    as-seen-by: <kinova.base_link.base_link_origin>\n"
        "}"
    )
    declaration = model.specs[0].context[0].declaration[0]
    fqns = [_fqn(pair.frame) for pair in declaration.props.pairs]
    assert fqns == [
        "gripper.g_base.g_pinch",
        "kinova.base_link.base_link_origin",
        "kinova.base_link.base_link_origin",
    ]


def test_unique_suffix_reaches_into_the_kinematic_graph():
    model = parse(
        "pose p2 {\n    of: <pick_place_graph.cube.cube_origin>,\n    wrt: <cube.cube_origin>\n}"
    )
    declaration = model.specs[0].context[0].declaration[0]
    fqns = {_fqn(pair.frame) for pair in declaration.props.pairs}
    assert fqns == {"pick_place_scene_mjc.pick_place_graph.cube.cube_origin"}


def test_body_ref_coerces_to_its_default_frame():
    model = parse(
        "pose p3 {\n"
        "    of: <pick_place_graph.cube>,\n"
        "    wrt: <kinova.base_link>,\n"
        "    as-seen-by: <world_tree.world_body>\n"
        "}"
    )
    declaration = model.specs[0].context[0].declaration[0]
    fqns = [_fqn(pair.frame) for pair in declaration.props.pairs]
    assert fqns == [
        "pick_place_scene_mjc.pick_place_graph.cube.cube_origin",
        "kinova.base_link.base_link_origin",
        "world_tree.world_body.world",
    ]


def test_template_internals_are_not_suffix_targets():
    with pytest.raises(TextXSemanticError, match="Unknown object"):
        parse("pose p3 { of: <half_arm_2_link>, wrt: <kinova.base_link> }")


def test_ambiguous_suffix_names_all_candidates():
    mm = metamodel_from_file(GRAMMAR, autokwd=True)
    mm.register_scope_providers({"*.*": SceneRefProvider()})
    src = """
ns app = "https://example.org/app/"
context (ns=app) c1 { spec { linear-distance support-z = 0.1 m } }
context (ns=app) c2 { spec { linear-distance support-z = 0.2 m } }
guarded-motion (ns=app) m1 {
    context { spec { duration d = 5.0 s } }
    when {}
    while { k1: elapsed up to <support-z> }
    until {}
}
"""
    with pytest.raises(TextXSemanticError, match="ambiguous.*c1.spec.support-z.*c2.spec.support-z"):
        mm.model_from_str(src, file_name=str(MODELS / "probe.robmot"))


def test_progress_requires_a_path_parameter_and_tracking_equality():
    source = (MODELS / "pick_place_single" / "pick_place_single.robmot").read_text()
    metamodel = motion_spec_metamodel()
    model_path = str(MODELS / "pick_place_single" / "pick_place_single.robmot")
    model = metamodel.model_from_str(source, file_name=model_path)
    handler = next(spec for spec in model.specs if spec.name == "handler-pick-above")
    assert len(handler.progress) == 1
    assert not hasattr(handler.motion, "progress")

    with pytest.raises(TextXSemanticError, match="PathParameter"):
        metamodel.model_from_str(
            source.replace("path-parameter s,", "dimensionless alpha,").replace(
                "<pick-above.spec.s>", "<pick-above.spec.alpha>"
            ),
            file_name=model_path,
        )

    untracked = source.replace(
        "equal to <spec.approach-path>.position", "equal to <spec.goal-pose>.position"
    ).replace(
        "equal to <spec.approach-path>.orientation", "equal to <spec.goal-pose>.orientation"
    )
    with pytest.raises(TextXSemanticError, match="needs a WHILE equality"):
        metamodel.model_from_str(untracked, file_name=model_path)


def test_progress_policy_names_and_paths_are_unique():
    source = (MODELS / "pick_place_single" / "pick_place_single.robmot").read_text()
    model_path = str(MODELS / "pick_place_single" / "pick_place_single.robmot")
    progress_block = (
        "        approach: constraint {\n"
        "            advance <pick-above.spec.s> along <pick-above.spec.approach-path> at 1.0 Hz\n"
        "        }"
    )
    duplicate_name = source.replace(progress_block, f"{progress_block},\n{progress_block}")
    with pytest.raises(TextXSemanticError, match="duplicate progress policy name"):
        motion_spec_metamodel().model_from_str(duplicate_name, file_name=model_path)

    duplicate_path = source.replace(
        "along <pick-above.spec.approach-path>",
        "along { <pick-above.spec.approach-path>, <pick-above.spec.approach-path> }",
    )
    with pytest.raises(TextXSemanticError, match="selects path 'approach-path' more than once"):
        motion_spec_metamodel().model_from_str(duplicate_path, file_name=model_path)


def test_progress_objective_needs_a_capable_solver():
    source = (MODELS / "pick_place_single" / "pick_place_single.robmot").read_text()
    model_path = str(MODELS / "pick_place_single" / "pick_place_single.robmot")
    with_objective = source.replace(
        "        approach: constraint {\n"
        "            advance <pick-above.spec.s> along <pick-above.spec.approach-path> at 1.0 Hz\n"
        "        }",
        "        approach: constraint {\n"
        "            advance <pick-above.spec.s> along <pick-above.spec.approach-path> at 1.0 Hz\n"
        "        },\n"
        "        approach-objective: objective {\n"
        "            maximize <pick-above.spec.s> along <pick-above.spec.approach-path>\n"
        "        }",
    )
    with pytest.raises(TextXSemanticError, match="needs a solver.*that can consume"):
        motion_spec_metamodel().model_from_str(with_objective, file_name=model_path)


def test_progress_at_most_one_objective_per_handler(monkeypatch: pytest.MonkeyPatch):
    # Isolate the duplicate-objective check from the (currently always-failing)
    # solver-capability check by granting ACHD objective capability for this test.
    from motion_spec_dsl.validation import constraints as validation_constraints

    monkeypatch.setattr(
        validation_constraints, "_OBJECTIVE_CAPABLE_SOLVER_ALGORITHMS", frozenset({"ACHD"})
    )
    source = (MODELS / "pick_place_single" / "pick_place_single.robmot").read_text()
    model_path = str(MODELS / "pick_place_single" / "pick_place_single.robmot")
    two_objectives = source.replace(
        "        approach: constraint {\n"
        "            advance <pick-above.spec.s> along <pick-above.spec.approach-path> at 1.0 Hz\n"
        "        }",
        "        approach-one: objective {\n"
        "            maximize <pick-above.spec.s> along <pick-above.spec.approach-path>\n"
        "        },\n"
        "        approach-two: objective {\n"
        "            maximize <pick-above.spec.s> along <pick-above.spec.approach-path>\n"
        "        }",
    )
    with pytest.raises(TextXSemanticError, match="at most one progress objective"):
        motion_spec_metamodel().model_from_str(two_objectives, file_name=model_path)
