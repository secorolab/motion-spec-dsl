# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Context quantity forms no shipped example uses: path kinds, derived directions and scalars,
sampled values, `post` declarations, and elapsed time since an observation."""

from __future__ import annotations

from pathlib import Path

import pytest
from rdflib.namespace import RDF, SOSA
from rdf_utils.models.vocab import URI_DISTRIB_PRED_FROM_DISTRIB, URI_DISTRIB_TYPE_SAMPLED_QUANTITY
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.langs import motion_spec_metamodel
from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import ALGO_EXT, CSTR_EXT, GEOM_OP, GEOM_PATH, QUDT_SCHEMA, TIME
from support import BASE, BASE_TEXT, HOLD, MODELS, SNAPSHOT, SPEC, TWIST

PATH_INPUTS = (
    ",\n        direction up { as-seen-by: <kinova.base_link.base_link_origin> } = (0, 0, 1)"
    ",\n        position centre { of: <table.table_top>, wrt: <kinova.base_link.base_link_origin>,"
    " as-seen-by: <kinova.base_link.base_link_origin> } = (0.4, 0.0, 0.3) m"
    ",\n        length radius = 0.05 m"
    ",\n        length pitch = 0.01 m"
    ",\n        dimensionless turns = 2.0 1"
)


@pytest.mark.parametrize(
    ("path", "type_"),
    [
        pytest.param(
            "circle { start: <spec.home-pose>, center: <shared.spec.centre>,"
            " plane-normal: <shared.spec.up> }",
            GEOM_PATH.Circle,
            id="circle",
        ),
        pytest.param(
            "helix { start: <spec.home-pose>, center: <shared.spec.centre>, axis: <shared.spec.up>,"
            " pitch: <shared.spec.pitch>, revolutions: <shared.spec.turns> }",
            GEOM_PATH.Helix,
            id="helix",
        ),
        pytest.param(
            "figure8 { anchor: <spec.home-pose>, radius: <shared.spec.radius>,"
            " plane-normal: <shared.spec.up>, form: gerono }",
            GEOM_PATH.Figure8,
            id="figure8_gerono",
        ),
        pytest.param(
            "figure8 { anchor: <spec.home-pose>, radius: <shared.spec.radius>,"
            " plane-normal: <shared.spec.up>, form: bernoulli }",
            GEOM_PATH.Figure8,
            id="figure8_bernoulli",
        ),
    ],
)
def test_a_path_kind_is_typed_and_tracked(path, type_) -> None:
    source = (
        BASE_TEXT.replace(SPEC, SPEC + PATH_INPUTS, 1)
        .replace(SNAPSHOT, f"{SNAPSHOT},\n            path track = {path}", 1)
        .replace(
            HOLD,
            f"{HOLD},\n        follow: keeping <shared.world.pose-ee-base>.position on"
            " <spec.track> within <shared.spec.satisfied-band>",
            1,
        )
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph

    [node] = graph.subjects(RDF.type, type_)
    assert (node, RDF.type, GEOM_PATH.Path) in graph


def test_a_direction_between_two_frames_normalizes_the_pose_relating_them() -> None:
    source = BASE_TEXT.replace(
        SPEC,
        f"{SPEC},\n        direction to-ee {{ as-seen-by: <kinova.base_link.base_link_origin> }}"
        " = from <kinova.base_link.base_link_origin> to <gripper.g_base.g_pinch>",
        1,
    ).replace(
        HOLD,
        f"{HOLD},\n        speed: norm of <shared.world.twist-ee-base>.linvel across"
        " <shared.spec.to-ee> greater than 0.05 m/s",
        1,
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph

    [op] = graph.subjects(RDF.type, GEOM_OP.PoseToDirection)
    assert str(graph.value(op, GEOM_OP.pose)).endswith("/pose-ee-base")
    assert str(graph.value(op, GEOM_OP.direction)).endswith("/to-ee")


def test_a_difference_of_two_derived_scalars_subtracts_them() -> None:
    source = (
        BASE_TEXT.replace(
            TWIST,
            TWIST + ",\n        pose pose-top { of: <table.table_top>,"
            " wrt: <kinova.base_link.base_link_origin>, as-seen-by: <kinova.base_link.base_link_origin> }",
            1,
        )
        .replace(
            SPEC,
            f"{SPEC},\n        direction up {{ as-seen-by: <kinova.base_link.base_link_origin> }} = (0, 0, 1)"
            ",\n        plane top { of: <table.table_top>, normal: <shared.spec.up> }"
            ",\n        direction across { as-seen-by: <kinova.base_link.base_link_origin> } = (1, 0, 0)"
            ",\n        plane floor { of: <table.table_top>, normal: <shared.spec.across> }"
            ",\n        distance above-top = distance of <shared.world.pose-ee-base> from <shared.spec.top>"
            ",\n        distance above-floor = distance of <shared.world.pose-ee-base> from <shared.spec.floor>",
            1,
        )
        .replace(
            HOLD,
            f"{HOLD},\n        gap: keeping difference of <shared.spec.above-floor> and"
            " <shared.spec.above-top> greater than 0.05 m",
            1,
        )
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph

    [subtraction] = [
        node
        for node in graph.subjects(RDF.type, ALGO_EXT.Subtraction)
        if "compute-difference-gap" in str(node)
    ]
    assert graph.value(subtraction, ALGO_EXT.minuend) != graph.value(subtraction, ALGO_EXT.subtrahend)


def test_a_post_declaration_is_in_scope_like_spec() -> None:
    source = BASE_TEXT.replace(
        SNAPSHOT, f"{SNAPSHOT}\n        }},\n        post {{\n            length reached = 0.02 m", 1
    ).replace(
        HOLD, f"{HOLD},\n        lift: keeping <shared.world.pose-ee-base>.position.z greater than <post.reached>", 1
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph

    [reached] = [s for s in graph.subjects(RDF.type, QUDT_SCHEMA.Quantity) if str(s).endswith("/reached")]
    assert float(graph.value(reached, QUDT_SCHEMA.value)) == 0.02


def test_elapsed_since_observed_begins_at_the_quantitys_phenomenon_time() -> None:
    source = BASE_TEXT.replace(
        HOLD, f"{HOLD},\n        seen: elapsed since <shared.world.pose-ee-base> observed less than 1.0 s", 1
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph

    [constraint] = graph.subjects(RDF.type, CSTR_EXT.TimeConstraint)
    instant = graph.value(graph.value(constraint, TIME.hasTime), TIME.hasBeginning)
    assert (instant, RDF.type, TIME.Instant) in graph
    [pose] = graph.subjects(SOSA.phenomenonTime, instant)
    assert str(pose).endswith("/shared/world/pose-ee-base")


def test_a_sampled_scalar_draws_from_a_one_dimensional_distribution(tmp_path: Path) -> None:
    common = MODELS / "common"
    scene = (common / "table_arm_sim.scenex").read_text()
    for imported in ("table_arm_sim.scene", "wrist_ft.scenex", "kinova_ft_2f85.scenex", "table_arm.scenex"):
        scene = scene.replace(f'import "{imported}"', f'import "{common / imported}"', 1)
    ns = 'ns tas_mjc = "https://secorolab.github.io/models/scenes/table-arm-sim-mjc/"'
    distribution = (
        "distrib (ns=tas_mjc) lift-draw {\n    uniform {\n        dimension: 1\n"
        "        lower: (0.05)\n        upper: (0.15)\n    }\n}"
    )
    scenex = tmp_path / "sampled.scenex"
    scenex.write_text(scene.replace(ns, f"{ns}\n\n{distribution}", 1))
    source = BASE.read_text()
    for old, new in (
        ('"../../src/motion_spec_dsl/models/06_arc_tracing_with_admittance/', f'"{MODELS}/06_arc_tracing_with_admittance/'),
        ('"../../src/motion_spec_dsl/models/common/table_arm_sim.scenex"', '"sampled.scenex"'),
        (SPEC, f"{SPEC},\n        length lift = sample <lift-draw> m"),
        (HOLD, f"{HOLD},\n        lift-above: keeping <shared.world.pose-ee-base>.position.z greater than <shared.spec.lift>"),
    ):
        source = source.replace(old, new, 1)
    robmot = tmp_path / "sampled.robmot"
    robmot.write_text(source)

    graph = MotionSpecDatasetBuilder(motion_spec_metamodel().model_from_file(str(robmot))).build()[0].default_graph
    [lift] = graph.subjects(RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY)
    assert str(graph.value(lift, URI_DISTRIB_PRED_FROM_DISTRIB)).endswith("lift-draw")
    assert graph.value(lift, QUDT_SCHEMA.value) is None

    scenex.write_text(
        scenex.read_text().replace(
            "dimension: 1\n        lower: (0.05)\n        upper: (0.15)",
            "dimension: 3\n        lower: (0.0, 0.0, 0.0)\n        upper: (0.1, 0.1, 0.1)",
        )
    )
    with pytest.raises(TextXSemanticError, match="samples a scalar quantity"):
        motion_spec_metamodel().model_from_file(str(robmot))
