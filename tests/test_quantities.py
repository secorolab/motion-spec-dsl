# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Quantities keep the unit they were written in, orientations are typed as scene-dsl types them,
and joint quantities and tolerance defaults reach the graph."""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from rdf_utils.models.vocab import (
    URI_GEOM_PRED_ALPHA,
    URI_GEOM_PRED_BETA,
    URI_GEOM_PRED_GAMMA,
    URI_GEOM_TYPE_ANGLES_ABG,
    URI_GEOM_TYPE_EULER_ANGLES,
    URI_GEOM_TYPE_EXTRINSIC,
    URI_GEOM_TYPE_ORIENT_COORD,
    URI_GEOM_TYPE_QUATERNION,
)
from rdf_utils.namespace import NS_MM_QUDT_QTY as QKIND, NS_MM_QUDT_UNIT as QUDT_UNIT
from rdflib import Graph, URIRef
from rdflib.namespace import RDF
from scene_dsl.classes.common import FloatVector
from scene_dsl.classes.geom import (
    DirectionCosineOrientationSpec,
    EulerOrientationSpec,
    QuaternionOrientationSpec,
)
from scene_dsl.rdf.geom import add_orientation_coord

from motion_spec_dsl.langs import motion_spec_metamodel
from motion_spec_dsl.rdf.dataset import build_dataset
from motion_spec_dsl.rdf_parser.vocab import (
    ACT,
    CSTR,
    CSTR_EXT,
    GEOM_COORD,
    GEOM_OP,
    GEOM_OP_EXT,
    KC_STAT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    TIME,
)
from support import BASE, BASE_TEXT, HOLD, SNAPSHOT, SPEC, TWIST, UNTIL


@pytest.mark.parametrize(
    ("declaration", "name", "value", "unit"),
    [
        pytest.param("distance d-cm = 5.0 cm", "d-cm", 5.0, "CentiM", id="cm"),
        pytest.param("distance d-mm = 1.0 mm", "d-mm", 1.0, "MilliM", id="mm"),
        pytest.param("angle a-deg = 90.0 deg", "a-deg", 90.0, "DEG", id="deg"),
        pytest.param("angular-velocity av = 10.0 deg/s", "av", 10.0, "DEG-PER-SEC", id="deg_per_s"),
        pytest.param("linear-velocity lv = 5.0 cm/s", "lv", 5.0, "CentiM-PER-SEC", id="cm_per_s"),
        pytest.param(
            "angular-acceleration aa = 90.0 deg/s^2", "aa", 90.0, "DEG-PER-SEC2", id="deg_per_s2"
        ),
        pytest.param("path-parameter s = 0.5 1", "s", 0.5, "UNITLESS", id="path_parameter"),
    ],
)
def test_a_quantity_keeps_the_unit_it_was_written_in(declaration, name, value, unit) -> None:
    source = BASE_TEXT.replace(SPEC, f"{SPEC},\n        {declaration}", 1)
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph

    [node] = [s for s in graph.subjects(RDF.type, QUDT_SCHEMA.Quantity) if str(s).endswith(f"/{name}")]
    assert float(graph.value(node, QUDT_SCHEMA.value)) == pytest.approx(value)
    assert graph.value(node, QUDT_SCHEMA.unit) == QUDT_UNIT[unit]


@pytest.mark.parametrize(
    ("declaration", "name", "units"),
    [
        pytest.param(
            "pose lit-pose { of: <gripper.g_base.g_pinch>, wrt: <kinova.base_link.base_link_origin>,"
            " as-seen-by: <kinova.base_link.base_link_origin> } = { position: (<spec.zero-linvel>, 20.0, 30.0)"
            " cm, orientation: euler { axes: xyz extrinsic, angles: (90.0, 0.0, 0.0) deg } }",
            "lit-pose",
            {"position": "CentiM", "orientation": "DEG"},
            id="pose",
        ),
        pytest.param(
            "velocity-twist tw = { angular-velocity: (1.0, 0.0, 0.0) deg/s, "
            "linear-velocity: (5.0, 0.0, 0.0) cm/s }",
            "tw",
            {"angular-velocity": "DEG-PER-SEC", "linear-velocity": "CentiM-PER-SEC"},
            id="velocity_twist",
        ),
        pytest.param(
            "acceleration-twist acc = { angular-acceleration: (1.0, 0.0, 0.0) deg/s^2, "
            "linear-acceleration: (1.0, 0.0, 0.0) m/s^2 }",
            "acc",
            {"angular-acceleration": "DEG-PER-SEC2", "linear-acceleration": "M-PER-SEC2"},
            id="acceleration_twist",
        ),
    ],
)
def test_each_part_of_a_compound_keeps_its_own_unit(declaration, name, units) -> None:
    """10 cm must not be labelled 10 metres, even when one axis references another quantity."""
    source = BASE_TEXT.replace(SPEC, f"{SPEC},\n        {declaration}", 1)
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph

    for part, unit in units.items():
        node = next(s for s in graph.subjects() if str(s).endswith(f"{name}.{part}"))
        assert graph.value(node, QUDT_SCHEMA.unit) == QUDT_UNIT[unit]


def test_an_elapsed_threshold_keeps_the_milliseconds_it_was_written_in() -> None:
    """owl-time has no unit below the second, so the magnitude and unit are qudt's."""
    source = BASE_TEXT.replace(HOLD, f"{HOLD},\n        wait: elapsed less than 10.0 ms", 1)
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph

    node = graph.value(next(graph.subjects(RDF.type, CSTR_EXT.TimeConstraint)), CSTR.threshold)
    assert (node, RDF.type, TIME.Duration) in graph
    assert float(graph.value(node, QUDT_SCHEMA.value)) == pytest.approx(10.0)
    assert graph.value(node, QUDT_SCHEMA.unit) == QUDT_UNIT["MilliSEC"]


def test_a_free_vector_is_a_dimensionless_vector_of_its_own_kind() -> None:
    source = BASE_TEXT.replace(SPEC, f"{SPEC},\n        free-vector weights = (1.0, 0.5, 0.0) 1", 1)
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph

    [node] = [s for s in graph.subjects(RDF.type, QUDT_SCHEMA.Quantity) if str(s).endswith("/weights")]
    assert graph.value(node, QUDT_SCHEMA.hasQuantityKind) == QKIND.FreeVector
    assert graph.value(node, QUDT_SCHEMA.unit) == QUDT_UNIT["UNITLESS"]


@pytest.mark.parametrize(
    ("block", "scene_spec"),
    [
        pytest.param(
            "euler { axes: xyz extrinsic, angles: (0.1, 0.2, 0.3) rad }",
            EulerOrientationSpec(None, "xyz", True, FloatVector(None, [0.1, 0.2, 0.3]), "rad"),
            id="euler",
        ),
        pytest.param(
            "quat { xyzw: (0.0, 0.0, 0.0, 1.0) }",
            QuaternionOrientationSpec(None, FloatVector(None, [0.0, 0.0, 0.0, 1.0])),
            id="quat",
        ),
        pytest.param(
            "direction-cosine { x: (1.0, 0.0, 0.0), y: (0.0, 1.0, 0.0), z: (0.0, 0.0, 1.0) }",
            DirectionCosineOrientationSpec(
                None,
                FloatVector(None, [1.0, 0.0, 0.0]),
                FloatVector(None, [0.0, 1.0, 0.0]),
                FloatVector(None, [0.0, 0.0, 1.0]),
            ),
            id="direction-cosine",
        ),
    ],
)
def test_an_orientation_is_typed_as_scene_dsl_types_it(block, scene_spec) -> None:
    """Value encodings differ by design; the representation-discriminating types must not."""
    source = BASE_TEXT.replace(
        SPEC,
        f"{SPEC},\n        pose test-pose {{ of: <gripper.g_base.g_pinch>,"
        " wrt: <kinova.base_link.base_link_origin>, as-seen-by: <kinova.base_link.base_link_origin> }"
        f" = {{ position: (0.1, 0.2, 0.3) m, orientation: {block} }}",
        1,
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph
    coord = next(
        s
        for s in graph.subjects(RDF.type, GEOM_COORD.OrientationCoordinate)
        if str(s).endswith("test-pose.orientation")
    )
    scene_coord = URIRef("https://example.test/pose/orientation-coord")
    scene_graph = Graph()
    add_orientation_coord(
        scene_graph,
        SimpleNamespace(
            orientation_uri=URIRef("https://example.test/pose/orientation"),
            orientation_coord_uri=scene_coord,
            of_frame=SimpleNamespace(uri=URIRef("https://example.test/frame/of")),
            wrt=SimpleNamespace(uri=URIRef("https://example.test/frame/wrt")),
            orientation=SimpleNamespace(spec=scene_spec, coord_type=block.split(" ")[0]),
        ),
    )

    discriminating = {
        URI_GEOM_TYPE_EULER_ANGLES,
        URI_GEOM_TYPE_EXTRINSIC,
        URI_GEOM_TYPE_QUATERNION,
        GEOM_COORD.DirectionCosineXYZ,
    }
    assert set(graph.objects(coord, RDF.type)) & discriminating == (
        set(scene_graph.objects(scene_coord, RDF.type)) & discriminating
    )


def test_relative_orientation_composes_instead_of_decomposing() -> None:
    """A measured rotation's components are a rotation vector while `euler` rebuilds from Euler
    angles, so a turned base reaches the graph as base + delta, never as angles."""
    source = BASE_TEXT.replace(
        SNAPSHOT,
        f"{SNAPSHOT},\n            pose turned-pose = {{ position: (0.1, 0.2, 0.3) m,"
        " orientation: <spec.home-pose>.orientation rotated by"
        " euler { axes: xyz extrinsic, angles: (-0.75, 0.0, 0.0) rad } }",
        1,
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph

    [composition] = graph.subjects(RDF.type, GEOM_OP_EXT.ComposeOrientation)
    orientation = graph.value(composition, GEOM_OP["composite"])
    assert (orientation, RDF.type, URI_GEOM_TYPE_ORIENT_COORD) in graph
    assert (orientation, RDF.type, URI_GEOM_TYPE_EULER_ANGLES) not in graph
    # The declared base frame turns the delta in the pose's basis, so the delta composes first.
    delta = graph.value(composition, GEOM_OP["in1"])
    base = graph.value(composition, GEOM_OP["in2"])
    assert (delta, RDF.type, URI_GEOM_TYPE_ANGLES_ABG) in graph
    assert str(base).endswith("home-pose")
    assert [
        float(graph.value(delta, predicate))
        for predicate in (URI_GEOM_PRED_ALPHA, URI_GEOM_PRED_BETA, URI_GEOM_PRED_GAMMA)
    ] == [-0.75, 0.0, 0.0]


@pytest.mark.parametrize(
    ("keyword", "spec", "relation", "type_", "kind", "unit"),
    [
        pytest.param(
            "joint-current",
            "current idle = 0.02 A",
            "less than <shared.spec.idle>",
            ACT.JointCurrent,
            QUDT_QKIND.ElectricCurrent,
            QUDT_UNIT.A,
            id="current",
        ),
        pytest.param(
            "joint-velocity",
            "angular-velocity idle = 0.0 rad/s,\n        angular-velocity band = 0.05 rad/s",
            "equal to <shared.spec.idle> within <shared.spec.band>",
            KC_STAT.JointVelocityCoordinate,
            QUDT_QKIND.AngularVelocity,
            QUDT_UNIT["RAD-PER-SEC"],
            id="velocity",
        ),
    ],
)
def test_a_joint_quantity_is_typed_measured_and_of_its_joint(keyword, spec, relation, type_, kind, unit) -> None:
    source = (
        BASE_TEXT.replace(TWIST, f"{TWIST},\n        {keyword} finger {{ joint: <gripper.g_left_driver_joint> }}", 1)
        .replace(SPEC, f"{SPEC},\n        {spec}", 1)
        .replace(UNTIL, f"{UNTIL},\n        stopped: <shared.world.finger> {relation}", 1)
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph

    [node] = graph.subjects(RDF.type, type_)
    assert (node, RDF.type, KC_STAT.JointReference) in graph
    assert (node, QUDT_SCHEMA.hasQuantityKind, kind) in graph
    assert (node, QUDT_SCHEMA.unit, unit) in graph
    assert str(graph.value(node, KC_STAT["of-joint"])).endswith("g_left_driver_joint")


def test_an_authored_band_wins_over_the_tolerance_default() -> None:
    source = BASE_TEXT.replace(
        "guarded-motion", "tolerances { linear-velocity: 0.02 m/s }\n\nguarded-motion", 1
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph

    bands = {
        str(constraint).rsplit("/", 1)[-1]: float(graph.value(band, QUDT_SCHEMA.value))
        for constraint, band in graph.subject_objects(CSTR_EXT.tolerance)
    }
    assert bands["settled-z"] == 0.01
