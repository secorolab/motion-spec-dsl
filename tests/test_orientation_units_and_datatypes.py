# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Scene-aligned orientation/value/unit grammar and JSON-LD numeric datatypes."""

from __future__ import annotations

import json
import re
from importlib.resources import files
from pathlib import Path

import pytest
from rdf_utils.models.vocab import (
    URI_GEOM_PRED_ALPHA,
    URI_GEOM_PRED_BETA,
    URI_GEOM_PRED_GAMMA,
    URI_GEOM_PRED_W,
    URI_GEOM_TYPE_ANGLES_ABG,
    URI_GEOM_TYPE_EULER_ANGLES,
    URI_GEOM_TYPE_EXTRINSIC,
    URI_GEOM_TYPE_ORIENT_COORD,
    URI_GEOM_TYPE_QUATERNION,
)
from rdflib.namespace import RDF, XSD
from textx.exceptions import TextXSemanticError, TextXSyntaxError

from motion_spec_dsl.gens import _canonicalize_jsonld, _gen_graph
from motion_spec_dsl.langs import motion_spec_metamodel
from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import GEOM_COORD

MODELS = Path(__file__).parents[1] / "models"
METAMODELS = Path(__file__).resolve().parents[2] / "metamodels"


def test_kdl_rpy_is_extrinsic_xyz() -> None:
    """STOP-condition check: RDF must align Euler with the runtime's actual convention."""
    PyKDL = pytest.importorskip("PyKDL")
    for r, p, y in [(0.3, -0.2, 1.1), (1.5, 0.4, -0.9), (0.0, 0.0, 0.0), (-2.7, 0.05, 3.0)]:
        lhs = PyKDL.Rotation.RPY(r, p, y)
        rhs = PyKDL.Rotation.RotZ(y) * PyKDL.Rotation.RotY(p) * PyKDL.Rotation.RotX(r)
        for i in range(3):
            for j in range(3):
                assert abs(lhs[i, j] - rhs[i, j]) < 1e-9


OLD_SYNTAX_REJECTIONS = [
    pytest.param(
        "gravity: (0.0, 0.0, 9.81) m/s^2",
        "gravity: { x: 0.0, y: 0.0, z: 9.81 m/s^2 }",
        id="gravity_brace_form",
    ),
    pytest.param(
        "gravity: (0.0, 0.0, 9.81) m/s^2",
        "gravity: (0.0, 0.0, 9.81) m/s2",
        id="gravity_old_unit_spelling",
    ),
    pytest.param(
        "linear-velocity zero-linvel = 0.0 m/s",
        "linear-jerk zero-jerk = 0.0 m/s3,\n        linear-velocity zero-linvel = 0.0 m/s",
        id="linear_jerk_old_unit_spelling",
    ),
    pytest.param(
        "pose home-pose = snapshot of <shared.world.pose-ee-base>",
        "pose home-pose = { position: { x: 0.1 m, y: 0.2 m, z: 0.3 m },"
        " orientation: euler { axes: xyz extrinsic, roll: 0.0 rad, pitch: 0.0 rad, yaw: 0.0 rad } }",
        id="position_and_orientation_per_axis_form",
    ),
    pytest.param(
        'description: "Hold the current end-effector pose"',
        'move: "Hold the current end-effector pose"',
        id="motion_metadata_move_keyword",
    ),
]


@pytest.mark.parametrize(("old", "new"), OLD_SYNTAX_REJECTIONS)
def test_old_syntax_no_longer_parses(parse_mutated, old, new) -> None:
    with pytest.raises((TextXSyntaxError, TextXSemanticError)):
        parse_mutated(old, new)


def test_unit_of_wrong_kind_is_a_parse_error(parse_mutated) -> None:
    with pytest.raises(TextXSyntaxError):
        parse_mutated(
            "pose home-pose = snapshot of <shared.world.pose-ee-base>",
            "pose home-pose = { position: (0.1, 0.2, 0.3) rad,"
            " orientation: euler { axes: xyz extrinsic, angles: (0.0, 0.0, 0.0) rad } }",
        )


WRONG_KIND_UNITS = [
    pytest.param("force zero-force = (0.0, 0.0, 0.0) m", "Force", id="force_in_metres"),
    pytest.param("duration settle = 0.3 m", "Duration", id="duration_in_metres"),
    pytest.param("angle tilt = 0.5 m/s", "Angle", id="angle_in_speed"),
]


@pytest.mark.parametrize(("declaration", "message"), WRONG_KIND_UNITS)
def test_unit_of_wrong_kind_for_its_quantity_is_rejected(
    parse_mutated, declaration, message
) -> None:
    """The value rule is shared across quantity types, so the kind check is semantic."""
    with pytest.raises(TextXSemanticError, match=message):
        parse_mutated(ORIENTATION_ANCHOR, f"{ORIENTATION_ANCHOR},\n        {declaration}")


EULER_CONVENTIONS = [
    pytest.param("euler { axes: zyx extrinsic, angles: (0.1, 0.2, 0.3) rad }", id="zyx_extrinsic"),
    pytest.param("euler { axes: xyz, angles: (0.1, 0.2, 0.3) rad }", id="xyz_intrinsic"),
    pytest.param("euler { axes: xyz extrinsic, angles: (0.1, 0.2, 0.3) rad }", id="xyz_extrinsic"),
]


@pytest.mark.parametrize("orientation_block", EULER_CONVENTIONS)
def test_any_euler_convention_is_accepted(parse_mutated, orientation_block) -> None:
    """The backend composes per-axis quaternions in the authored order, so no sequence is
    privileged the way KDL::Rotation::RPY once privileged extrinsic-XYZ."""
    parse_mutated(ORIENTATION_ANCHOR, _pose_source(orientation_block))


ORIENTATION_ANCHOR = "linear-velocity zero-linvel = 0.0 m/s"


def _pose_source(orientation_block: str) -> str:
    return (
        f"{ORIENTATION_ANCHOR},\n        pose test-pose {{ "
        "of: <gripper.g_base.g_pinch>, "
        "wrt: <kinova.base_link.base_link_origin>, "
        "as-seen-by: <kinova.base_link.base_link_origin> } = "
        "{ position: (0.1, 0.2, 0.3) m, orientation: " + orientation_block + " }"
    )


def _build(parse_mutated, orientation_block: str):
    model = parse_mutated(ORIENTATION_ANCHOR, _pose_source(orientation_block))
    dataset, _ctx = MotionSpecDatasetBuilder(model).build()
    return dataset.default_graph


def test_euler_orientation_emits_euler_type(parse_mutated) -> None:
    graph = _build(
        parse_mutated, "euler { axes: xyz extrinsic, angles: (0.1, 0.2, 0.3) rad }"
    )
    euler_subjects = set(graph.subjects(RDF.type, URI_GEOM_TYPE_EULER_ANGLES))
    assert euler_subjects
    for s in euler_subjects:
        assert (s, RDF.type, URI_GEOM_TYPE_EXTRINSIC) in graph
        assert (s, RDF.type, URI_GEOM_TYPE_QUATERNION) not in graph
        assert (s, RDF.type, GEOM_COORD.DirectionCosineXYZ) not in graph


def test_quaternion_orientation_emits_quaternion_type_and_no_unit(parse_mutated) -> None:
    graph = _build(parse_mutated, "quat { xyzw: (0.0, 0.0, 0.0, 1.0) }")
    quat_subjects = set(graph.subjects(RDF.type, URI_GEOM_TYPE_QUATERNION))
    assert quat_subjects
    for s in quat_subjects:
        assert (s, RDF.type, URI_GEOM_TYPE_EULER_ANGLES) not in graph
        assert (s, RDF.type, GEOM_COORD.DirectionCosineXYZ) not in graph
        from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT

        from motion_spec_dsl.rdf_parser.vocab import QUDT_SCHEMA

        assert (s, QUDT_SCHEMA.unit, QUDT_UNIT.RAD) not in graph


def test_direction_cosine_orientation_emits_dcm_type(parse_mutated) -> None:
    graph = _build(
        parse_mutated,
        "direction-cosine { x: (1.0, 0.0, 0.0), y: (0.0, 1.0, 0.0), z: (0.0, 0.0, 1.0) }",
    )
    dc_subjects = set(graph.subjects(RDF.type, GEOM_COORD.DirectionCosineXYZ))
    assert dc_subjects
    for s in dc_subjects:
        assert (s, RDF.type, URI_GEOM_TYPE_EULER_ANGLES) not in graph
        assert (s, RDF.type, URI_GEOM_TYPE_QUATERNION) not in graph
        assert any(graph.objects(s, GEOM_COORD["direction-cosine-x"]))


def test_whole_value_reference_position_and_orientation(parse_mutated) -> None:
    anchor = "pose home-pose = snapshot of <shared.world.pose-ee-base>"
    model = parse_mutated(
        anchor,
        f"{anchor},\n            pose ref-pose = "
        "{ position: <spec.home-pose>.position,"
        " orientation: <spec.home-pose>.orientation }",
    )
    dataset, _ctx = MotionSpecDatasetBuilder(model).build()
    assert len(dataset.default_graph) > 0


def test_motion_metadata_emits_name_and_optional_description(parse_mutated) -> None:
    """A motion always carries schema:name; schema:description only when authored."""
    from rdflib.namespace import SDO

    described = parse_mutated(ORIENTATION_ANCHOR, ORIENTATION_ANCHOR)
    graph = MotionSpecDatasetBuilder(described).build()[0].default_graph
    named = {str(o) for o in graph.objects(None, SDO.name)}
    assert "home" in named
    assert "Hold the current end-effector pose" in {
        str(o) for o in graph.objects(None, SDO.description)
    }

    plain = parse_mutated('    description: "Hold the current end-effector pose"\n', "")
    plain_graph = MotionSpecDatasetBuilder(plain).build()[0].default_graph
    assert "home" in {str(o) for o in plain_graph.objects(None, SDO.name)}
    assert not set(plain_graph.objects(None, SDO.description))


def _scene_orientation_types(spec, coord_type: str) -> set:
    """Type triples scene-dsl's own emitter puts on an orientation coordinate for `spec`."""
    from types import SimpleNamespace

    from rdflib import Graph, URIRef
    from scene_dsl.rdf.geom import add_orientation_coord

    coord_uri = URIRef("https://example.test/pose/orientation-coord")
    pose = SimpleNamespace(
        orientation_uri=URIRef("https://example.test/pose/orientation"),
        orientation_coord_uri=coord_uri,
        of_frame=SimpleNamespace(uri=URIRef("https://example.test/frame/of")),
        wrt=SimpleNamespace(uri=URIRef("https://example.test/frame/wrt")),
        orientation=SimpleNamespace(spec=spec, coord_type=coord_type),
    )
    graph = Graph()
    add_orientation_coord(graph, pose)
    return set(graph.objects(coord_uri, RDF.type))


# The representation-discriminating types both packages must agree on. Value encodings
# differ by design: motion's components can be context references, scene's are literals.
DISCRIMINATING_TYPES = {
    URI_GEOM_TYPE_EULER_ANGLES,
    URI_GEOM_TYPE_EXTRINSIC,
    URI_GEOM_TYPE_QUATERNION,
    GEOM_COORD.DirectionCosineXYZ,
}


def _scene_spec(coord_type: str):
    """The scene-dsl orientation spec matching the authored block of the same coord type."""
    from scene_dsl.classes.common import FloatVector
    from scene_dsl.classes.geom import (
        DirectionCosineOrientationSpec,
        EulerOrientationSpec,
        QuaternionOrientationSpec,
    )

    def vector(*values):
        return FloatVector(None, list(values))

    if coord_type == "euler":
        return EulerOrientationSpec(None, "xyz", True, vector(0.1, 0.2, 0.3), "rad")
    if coord_type == "quat":
        return QuaternionOrientationSpec(None, vector(0.0, 0.0, 0.0, 1.0))
    return DirectionCosineOrientationSpec(
        None, vector(1.0, 0.0, 0.0), vector(0.0, 1.0, 0.0), vector(0.0, 0.0, 1.0)
    )


ORIENTATION_BLOCKS = {
    "euler": "euler { axes: xyz extrinsic, angles: (0.1, 0.2, 0.3) rad }",
    "quat": "quat { xyzw: (0.0, 0.0, 0.0, 1.0) }",
    "direction-cosine": (
        "direction-cosine { x: (1.0, 0.0, 0.0), y: (0.0, 1.0, 0.0), z: (0.0, 0.0, 1.0) }"
    ),
}


@pytest.mark.parametrize("coord_type", sorted(ORIENTATION_BLOCKS))
def test_orientation_types_match_scene_dsl(parse_mutated, coord_type) -> None:
    """Cross-package contract: the same authored orientation must be typed identically by
    motion's builder and by scene-dsl's own emitter."""
    pytest.importorskip("scene_dsl.rdf.geom")
    graph = _build(parse_mutated, ORIENTATION_BLOCKS[coord_type])
    coord = next(
        s
        for s in graph.subjects(RDF.type, GEOM_COORD.OrientationCoordinate)
        if str(s).endswith("test-pose.orientation")
    )
    motion_types = set(graph.objects(coord, RDF.type)) & DISCRIMINATING_TYPES
    scene_types = _scene_orientation_types(_scene_spec(coord_type), coord_type)
    assert motion_types == scene_types & DISCRIMINATING_TYPES


def _orientation_coord_graph(graph):
    """The test pose's orientation coordinate as a standalone graph, carrying only its
    coordinate types -- the relation shapes want frames this fragment does not describe."""
    from rdflib import Graph

    from motion_spec_dsl.rdf_parser.vocab import GEOM_REL

    coord = next(
        s
        for s in graph.subjects(RDF.type, GEOM_COORD.OrientationCoordinate)
        if str(s).endswith("test-pose.orientation")
    )
    isolated = Graph()
    for predicate, object_ in graph.predicate_objects(coord):
        if (predicate, object_) == (RDF.type, GEOM_REL.Orientation):
            continue
        isolated.add((coord, predicate, object_))
    return coord, isolated


@pytest.mark.parametrize("coord_type", sorted(ORIENTATION_BLOCKS))
def test_orientation_shapes_accept_the_emitted_triples(parse_mutated, coord_type) -> None:
    """Each representation's SHACL cardinality shape must match what the builder emits."""
    pyshacl = pytest.importorskip("pyshacl")
    from rdflib import Graph

    shapes = Graph().parse(METAMODELS / "geometry" / "geometry.shacl.ttl", format="turtle")
    _coord, data = _orientation_coord_graph(_build(parse_mutated, ORIENTATION_BLOCKS[coord_type]))
    conforms, _report, text = pyshacl.validate(
        data_graph=data, shacl_graph=shapes, inference="none"
    )
    assert conforms, text


ORIENTATION_SHAPE_VIOLATIONS = [
    pytest.param("quat", URI_GEOM_PRED_W, id="quaternion_missing_component"),
    pytest.param(
        "direction-cosine", GEOM_COORD["direction-cosine-z"], id="direction_cosine_missing_axis"
    ),
    pytest.param("euler", GEOM_COORD["axes-sequence"], id="euler_missing_axes_sequence"),
]


@pytest.mark.parametrize(("coord_type", "predicate"), ORIENTATION_SHAPE_VIOLATIONS)
def test_orientation_shapes_reject_a_missing_component(
    parse_mutated, coord_type, predicate
) -> None:
    pyshacl = pytest.importorskip("pyshacl")
    from rdflib import Graph

    shapes = Graph().parse(METAMODELS / "geometry" / "geometry.shacl.ttl", format="turtle")
    coord, data = _orientation_coord_graph(_build(parse_mutated, ORIENTATION_BLOCKS[coord_type]))
    data.remove((coord, predicate, next(data.objects(coord, predicate))))
    conforms, _report, _text = pyshacl.validate(
        data_graph=data, shacl_graph=shapes, inference="none"
    )
    assert not conforms


ARITY_REJECTIONS = [
    pytest.param(
        "euler { axes: xyz extrinsic, angles: (0.1, 0.2) rad }",
        "Euler",
        id="euler_wrong_arity",
    ),
    pytest.param(
        "quat { xyzw: (0.0, 0.0, 0.0) }",
        "Quaternion",
        id="quaternion_wrong_arity",
    ),
    pytest.param(
        "direction-cosine { x: (1.0, 0.0), y: (0.0, 1.0, 0.0), z: (0.0, 0.0, 1.0) }",
        "Direction-cosine",
        id="direction_cosine_wrong_arity",
    ),
]


@pytest.mark.parametrize(("orientation_block", "message"), ARITY_REJECTIONS)
def test_orientation_arity_is_validated(parse_mutated, orientation_block, message) -> None:
    with pytest.raises(TextXSemanticError, match=message):
        parse_mutated(ORIENTATION_ANCHOR, _pose_source(orientation_block))


def test_euler_axes_matches_scene_dsl() -> None:
    """Drift guard: motion's copied EulerAxes must match scene's geom.tx verbatim."""
    geom_tx = (files("scene_dsl") / "grammars" / "geom.tx").read_text()
    scene_match = re.search(r"EulerAxes:\s*(.+?);", geom_tx, re.DOTALL)
    assert scene_match is not None
    scene_sequences = re.findall(r"'(\w+)'", scene_match.group(1))

    context_tx = (
        files("motion_spec_dsl.grammars") / "context.tx"
    ).read_text()
    motion_match = re.search(r"EulerAxes:\s*(.+?);", context_tx, re.DOTALL)
    assert motion_match is not None
    motion_sequences = re.findall(r"'(\w+)'", motion_match.group(1))

    assert motion_sequences == scene_sequences == ["xyz", "xzy", "yxz", "yzx", "zxy", "zyx"]


TWO_SUBSPACE_REJECTIONS = [
    pytest.param(
        "velocity-twist platform-velocity {\n"
        "            of: <kinova.base_link.base_link_origin>,\n"
        "            wrt: <kinova.base_link.base_link_origin>\n"
        "        }",
        "velocity-twist platform-velocity {\n"
        "            of: <kinova.base_link.base_link_origin>,\n"
        "            wrt: <kinova.base_link.base_link_origin>\n"
        "        } = (0.0, 0.0, 0.0) rad/s",
        id="velocity_twist_bare_vector_rejected",
    ),
    pytest.param(
        "wrench platform-force {\n"
        "            of:         <kinova.base_link.base_link_origin>,\n"
        "            as-seen-by: <kinova.base_link.base_link_origin>\n"
        "        }",
        "wrench platform-force {\n"
        "            of:         <kinova.base_link.base_link_origin>,\n"
        "            as-seen-by: <kinova.base_link.base_link_origin>\n"
        "        } = (0.0, 0.0, 0.0) N",
        id="wrench_bare_vector_rejected",
    ),
]


@pytest.fixture
def parse_mixed_solvers_mutated():
    from motion_spec_dsl.langs import motion_spec_metamodel

    fixture_path = Path(__file__).parent / "fixtures" / "mixed_solvers.robmot"
    source = fixture_path.read_text()

    def _mutate(old: str, new: str):
        assert old in source, f"mutation anchor not found in mixed_solvers.robmot: {old!r}"
        return motion_spec_metamodel().model_from_str(
            source.replace(old, new, 1), file_name=str(fixture_path)
        )

    return _mutate


@pytest.mark.parametrize(("old", "new"), TWO_SUBSPACE_REJECTIONS)
def test_bare_vector_literal_rejected_for_two_subspace_types(
    parse_mixed_solvers_mutated, old, new
) -> None:
    with pytest.raises(TextXSemanticError, match="two-subspace"):
        parse_mixed_solvers_mutated(old, new)


def test_velocity_twist_and_wrench_two_subspace_literals(parse_mutated) -> None:
    model = parse_mutated(
        ORIENTATION_ANCHOR,
        f"{ORIENTATION_ANCHOR},\n"
        "        velocity-twist vt = { angular-velocity: (0.0, 0.0, 0.0) rad/s,"
        " linear-velocity: (0.0, 0.0, -0.05) m/s },\n"
        "        acceleration-twist at = { angular-acceleration: (0.0, 0.0, 0.0) rad/s^2,"
        " linear-acceleration: (0.0, 0.0, 0.0) m/s^2 },\n"
            "        wrench w { ref-point: <ft_tree.wrist_ft_body.wrist_ft_site>,"
            " as-seen-by: <kinova.base_link.base_link_origin>,"
            " of: <gripper.g_base.g_pinch> } ="
            " { torque: (0.0, 0.0, 0.0) Nm, force: (0.0, 0.0, 10.0) N }",
    )
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph
    assert set(graph.subjects(RDF.type, GEOM_COORD.VelocityTwistCoordinate))
    assert any(graph.objects(None, GEOM_COORD["angular-velocity"]))
    assert any(graph.objects(None, GEOM_COORD["linear-velocity"]))
    assert any(graph.objects(None, GEOM_COORD["angular-acceleration"]))
    assert any(graph.objects(None, GEOM_COORD["linear-acceleration"]))


FIXTURES = Path(__file__).parent / "fixtures"

@pytest.fixture(scope="module")
def generated_pick_place(tmp_path_factory: pytest.TempPathFactory) -> Path:
    tmp_path = tmp_path_factory.mktemp("pick_place_single_numerics")
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv("METAMODELS_PATH", str(METAMODELS))
        metamodel = motion_spec_metamodel()
        model = metamodel.model_from_file(
            MODELS / "pick_place_single" / "pick_place_single.robmot"
        )
        _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)
    return tmp_path / "pick_place_single.ld.json"


def _walk_value_objects(value, sink: list) -> None:
    if isinstance(value, dict):
        if "@value" in value and "@type" in value and len(value) == 2:
            sink.append(value)
        else:
            for v in value.values():
                _walk_value_objects(v, sink)
    elif isinstance(value, list):
        for item in value:
            _walk_value_objects(item, sink)


def test_generated_jsonld_has_explicit_numeric_datatypes(generated_pick_place: Path) -> None:
    document = json.loads(generated_pick_place.read_text())
    assert document["@context"]["xsd"] == str(XSD)

    objects: list = []
    _walk_value_objects(document.get("@graph", []), objects)
    doubles = [o for o in objects if o["@type"] == "xsd:double"]
    assert doubles
    assert all(isinstance(o["@value"], float) for o in doubles)

    def raw_numbers(value, sink: list) -> None:
        if isinstance(value, dict):
            if "@value" in value and "@type" in value:
                return
            for v in value.values():
                raw_numbers(v, sink)
        elif isinstance(value, list):
            for item in value:
                raw_numbers(item, sink)
        elif isinstance(value, (int, float)) and not isinstance(value, bool):
            sink.append(value)

    bare: list = []
    raw_numbers(document.get("@graph", []), bare)
    assert bare == []


def test_jsonld_numeric_datatype_canonicalization_unit() -> None:
    """`_canonicalize_jsonld` derives the datatype from the source graph, not Python shape:
    the canonicalized document must stay graph-isomorphic to the source, with every literal's
    datatype -- including xsd:boolean, which must never be classified as xsd:integer --
    unchanged. Visibility (the datatype showing as an explicit JSON-LD value object rather
    than a bare number) is covered on the real generated document by
    `test_generated_jsonld_has_explicit_numeric_datatypes`.
    """
    from rdflib import Graph, Literal, Namespace, URIRef
    from rdflib.compare import isomorphic

    EX = Namespace("https://example.test/")
    source = Graph()
    source.bind("ex", EX)
    source.bind("xsd", XSD)
    subject = URIRef(EX["thing"])
    source.add((subject, EX.wholeCount, Literal(0, datatype=XSD.integer)))
    source.add((subject, EX.measured, Literal(0.0, datatype=XSD.double)))
    source.add((subject, EX.negative, Literal(-3, datatype=XSD.integer)))
    source.add((subject, EX.tiny, Literal(1.5e-10, datatype=XSD.double)))
    source.add((subject, EX.flag, Literal(True)))
    other = URIRef(EX["other"])
    source.add((other, EX.wholeCount, Literal(7, datatype=XSD.integer)))
    context = {"ex": str(EX), "xsd": str(XSD)}
    serialized = source.serialize(format="json-ld", context=context, auto_compact=True, indent=2)
    canonical = _canonicalize_jsonld(serialized, source)

    roundtripped = Graph()
    roundtripped.parse(data=canonical, format="json-ld")
    assert isomorphic(source, roundtripped)
    for predicate, expected_datatype in (
        (EX.wholeCount, XSD.integer),
        (EX.measured, XSD.double),
        (EX.negative, XSD.integer),
        (EX.tiny, XSD.double),
    ):
        literal = roundtripped.value(subject, predicate)
        assert literal.datatype == expected_datatype
    assert roundtripped.value(subject, EX.flag).datatype == XSD.boolean


def test_canonical_jsonld_stable_across_two_generations(tmp_path_factory) -> None:
    tmp_a = tmp_path_factory.mktemp("gen_a")
    tmp_b = tmp_path_factory.mktemp("gen_b")
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv("METAMODELS_PATH", str(METAMODELS))
        model_a = motion_spec_metamodel().model_from_file(
            MODELS / "pick_place_single" / "pick_place_single.robmot"
        )
        _gen_graph(None, model_a, tmp_a, overwrite=True, debug=False)
        model_b = motion_spec_metamodel().model_from_file(
            MODELS / "pick_place_single" / "pick_place_single.robmot"
        )
        _gen_graph(None, model_b, tmp_b, overwrite=True, debug=False)
    a = (tmp_a / "pick_place_single.ld.json").read_text()
    b = (tmp_b / "pick_place_single.ld.json").read_text()
    assert a == b


def test_relative_orientation_composes_instead_of_decomposing(parse_mutated) -> None:
    """A base orientation turned by a delta must never be decomposed into angles.

    Reading a measured rotation's components gives a rotation vector while `euler` rebuilds
    from Euler angles; the two disagree by over a radian for a general pose, so the
    composition has to reach the graph as base + delta, not as three scalars.
    """
    anchor = "pose home-pose = snapshot of <shared.world.pose-ee-base>"
    model = parse_mutated(
        anchor,
        f"{anchor},\n            pose turned-pose = "
        "{ position: (0.1, 0.2, 0.3) m,"
        " orientation: <spec.home-pose>.orientation rotated by"
        " euler { axes: xyz extrinsic, angles: (-0.75, 0.0, 0.0) rad } }",
    )
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph

    from motion_spec.rdf_parser.ir import Parser

    from motion_spec_dsl.rdf_parser.vocab import GEOM_OP, GEOM_OP_EXT

    relative = set(graph.subjects(GEOM_OP.in1, None))
    assert len(relative) == 1
    orientation = relative.pop()

    in1 = graph.value(orientation, GEOM_OP["in1"])
    in2 = graph.value(orientation, GEOM_OP["in2"])
    assert in1 is not None and in2 is not None

    def is_delta(node):
        return (node, RDF.type, URI_GEOM_TYPE_ANGLES_ABG) in graph

    # No frame named: the delta turns in the base's own body frame, so it composes post
    # (base first).
    assert not is_delta(in1) and is_delta(in2)
    base, delta = in1, in2
    assert str(base).endswith("home-pose")
    assert (delta, RDF.type, URI_GEOM_TYPE_ORIENT_COORD) not in graph
    assert [
        float(graph.value(delta, predicate))
        for predicate in (URI_GEOM_PRED_ALPHA, URI_GEOM_PRED_BETA, URI_GEOM_PRED_GAMMA)
    ] == [-0.75, 0.0, 0.0]
    assert not list(graph.objects(delta, GEOM_COORD["as-seen-by"]))

    operands = Parser(graph)._relative_orientation(orientation)
    assert [set(operand) & {"pose", "delta"} for operand in operands] == [
        {"pose"},
        {"delta"},
    ]
    # The graph keeps the authored Euler triple; the reader hands codegen the quaternion.
    assert operands[1]["representation"] == "quaternion"
    assert [c["value"] for c in operands[1]["delta"]] == pytest.approx(
        [-0.36627253, 0.0, 0.0, 0.93050762]
    )


def test_relative_orientation_rejects_a_third_frame(parse_mutated) -> None:
    """A delta that turns in neither the base's body frame nor its own basis needs a change
    of basis to compose, which is not supported."""
    anchor = "pose home-pose = snapshot of <shared.world.pose-ee-base>"
    model = parse_mutated(
        anchor,
        f"{anchor},\n            pose turned-pose = "
        "{ position: (0.1, 0.2, 0.3) m,"
        " orientation: <spec.home-pose>.orientation rotated in <kinova.bracelet_link.pinch_site> by"
        " euler { axes: xyz extrinsic, angles: (-0.75, 0.0, 0.0) rad } }",
    )
    with pytest.raises(ValueError, match="neither the base's body frame"):
        MotionSpecDatasetBuilder(model).build()
