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


def test_full_example_parses():
    model = motion_spec_metamodel().model_from_file(str(MODELS / "pick_place_single" / "pick_place_single.robmot"))
    assert len(model.specs) == 22
    handler = next(spec for spec in model.specs if spec.name == "handler-pick-above")
    assert handler.motion.name == "pick-above"
    assert handler.monitors[0].fallback.name == "home"


def test_solver_gravity_accepts_literal_equals_and_spec_reference():
    source = (MODELS / "pick_place_single" / "pick_place_single.robmot").read_text()
    metamodel = motion_spec_metamodel()

    equal_literal = metamodel.model_from_str(
        source.replace("gravity:   {", "gravity:   = {"),
        file_name=str(MODELS / "pick_place_single" / "pick_place_single.robmot"),
    )
    equal_solver = next(
        spec for spec in equal_literal.specs if spec.name == "handler-home"
    ).solvers[0]
    assert equal_solver.gravity_value.literal.z == -9.81

    referenced = source.replace(
        "linear-distance lost-dist   = 0.12 m",
        "linear-distance lost-dist   = 0.12 m,\n"
        "        free-vector solver-gravity = { x: 0.0, y: 0.0, z: -9.81 m/s2 }",
    ).replace(
        "gravity:   { x: 0.0, y: 0.0, z: -9.81 m/s2 }",
        "gravity:   <shared.spec.solver-gravity>",
    )
    referenced_solver = next(
        spec
        for spec in metamodel.model_from_str(
            referenced, file_name=str(MODELS / "pick_place_single" / "pick_place_single.robmot")
        ).specs
        if spec.name == "handler-home"
    ).solvers[0]
    assert referenced_solver.gravity_value.ref.quantity.name == "solver-gravity"


def test_grammar_keyword_cannot_be_used_as_a_quantity_name():
    source = (MODELS / "pick_place_single" / "pick_place_single.robmot").read_text()
    reserved_name = source.replace("path-parameter progress", "path-parameter gravity").replace(
        "<spec.progress>", "<spec.gravity>"
    )

    with pytest.raises(TextXSemanticError, match="gravity.*keyword"):
        motion_spec_metamodel().model_from_str(
            reserved_name, file_name=str(MODELS / "pick_place_single" / "pick_place_single.robmot")
        )
