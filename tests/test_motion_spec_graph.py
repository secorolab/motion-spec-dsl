# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Integration checks for the reduced motion-spec RDF contract."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from rdflib import Graph, Namespace, URIRef
from rdflib.namespace import RDF

from motion_spec.rdf_parser.ir import _load_graph, generate_ir
from motion_spec.rdf_parser.vocab import (
    ALGO_EXT,
    CSTR,
    CSTR_HDL,
    GEOM_OP_EXT,
    MAP,
    MAP_EXT,
    QKIND_EXT,
    QUDT_SCHEMA,
    SLV,
    SLV_EXT,
)
from motion_spec_dsl.classes.constraint_handler import ROSTopic
from motion_spec_dsl.rdf.builder import MotionSpecDatasetBuilder
from motion_spec_dsl.registration import _gen_graph, motion_spec_metamodel
from motion_spec_dsl.rdf._specs import ROS


MODELS = Path(__file__).parents[1] / "models"
METAMODELS = Path(__file__).resolve().parents[2] / "metamodels"


@pytest.fixture(scope="module")
def generated_model(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Generate the representative model once per module; consumers load their own
    fresh graph from the immutable result and must not mutate the manifest on disk."""
    tmp_path = tmp_path_factory.mktemp("pick_place_single")
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv("METAMODELS_PATH", str(METAMODELS))
        metamodel = motion_spec_metamodel()
        model = metamodel.model_from_file(MODELS / "pick_place_single" / "pick_place_single.robmot")
        _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)
    return tmp_path / "pick_place_single-app.ld.json"


@pytest.fixture(scope="module")
def generated_dual_model(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Generate the dual-arm model once per module; consumers load their own fresh
    graph from the immutable result and must not mutate the manifest on disk."""
    tmp_path = tmp_path_factory.mktemp("pick_place_dual")
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv("METAMODELS_PATH", str(METAMODELS))
        metamodel = motion_spec_metamodel()
        model = metamodel.model_from_file(MODELS / "pick_place_dual" / "pick_place_dual.robmot")
        _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)
    return tmp_path / "pick_place_dual-app.ld.json"


def test_generated_jsonld_compacts_hierarchical_identifiers(generated_model: Path) -> None:
    document = json.loads(generated_model.with_name("pick_place_single.ld.json").read_text())
    identifiers: list[str] = []

    def collect(value) -> None:
        if isinstance(value, dict):
            if isinstance(value.get("@id"), str):
                identifiers.append(value["@id"])
            for item in value.values():
                collect(item)
        elif isinstance(value, list):
            for item in value:
                collect(item)

    collect(document.get("@graph", []))
    assert "app:pick-above/while/follow-lat" in identifiers
    assert not any(identifier.startswith(("http://", "https://", "/")) for identifier in identifiers)


def test_dual_arm_physical_profiles_and_path_progress_reach_ir(generated_dual_model: Path) -> None:
    graph = _load_graph(generated_dual_model)[1]
    assert len(set(graph.subjects(QUDT_SCHEMA.hasQuantityKind, QKIND_EXT.LinearJerk))) == 1
    # Each arm follows its own path, so each gets its own projection and its own local frame.
    projections = set(graph.subjects(RDF.type, GEOM_OP_EXT.PathProjection))
    assert len(projections) == 2
    assert len({graph.value(node, GEOM_OP_EXT.path) for node in projections}) == 2
    assert len({graph.value(node, GEOM_OP_EXT.tangent) for node in projections}) == 2

    ir = generate_ir(generated_dual_model)
    profiles = [value for value in ir["closures"].values() if value.get("type") == "VelocityProfile"]

    motion_profiles = profiles
    assert len(motion_profiles) == 2
    assert all(str(profile["shape"]) == "s_curve" and profile["in"] for profile in motion_profiles)
    assert all(profile["maximum_jerk"] == "max_lower_jerk" for profile in motion_profiles)
    pick_above = next(motion for motion in ir["motions"] if motion.id == "motion_pick_above")
    assert {entry["parameter"] for entry in pick_above.path_projections} == {
        "arm1_approach_path_s",
        "arm2_approach_path_s",
    }
    assert {entry["along_speed"] for entry in pick_above.path_projections} == {
        "arm1_approach_path_along_speed",
        "arm2_approach_path_along_speed",
    }


def test_generation_keeps_scene_fsm_and_provenance_separate(generated_model: Path) -> None:
    output = generated_model.parent
    assert (output / "pick_place_single.ld.json").exists()
    assert (output / "pick_place_single.scenex.ld.json").exists()
    assert (output / "pick_place_single_fsm.ld.json").exists()

    provenance = Graph().parse(output / "provenance" / "dsl.ld.json", format="json-ld")
    prov = Namespace("http://www.w3.org/ns/prov#")
    activity = URIRef(
        "https://secorolab.github.io/motion-spec-dsl/provenance/"
        "activity/jsonld_generation/pick_place_single"
    )
    assert (activity, RDF.type, prov.Activity) in provenance


def test_non_pose_component_views_keep_their_subspace(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Wrench and twist axes retain their authored non-pose subspaces."""
    monkeypatch.setenv("METAMODELS_PATH", str(METAMODELS))
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_file(
        MODELS / "admittance_arc_single" / "admittance_arc_single.robmot"
    )
    _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)
    graph = _load_graph(tmp_path / "admittance_arc_single-app.ld.json")[1]
    wrench_views = set(graph.subjects(RDF.type, MAP_EXT.WrenchCoordinateView))
    twist_views = set(graph.subjects(RDF.type, MAP_EXT.VelocityTwistCoordinateView))
    assert wrench_views and twist_views
    assert {graph.value(view, MAP.subspace) for view in wrench_views} == {MAP.force}
    assert {graph.value(view, MAP.subspace) for view in twist_views} == {
        MAP["linear-velocity"]
    }


def test_monitor_publishes_to_ros_topic(monkeypatch: pytest.MonkeyPatch) -> None:
    """`also publish to topic` marks the event monitor as a ROS topic, reusing the canonical
    ROS namespace and leaking none of the unused terms from its JSON-LD context."""
    monkeypatch.setenv("METAMODELS_PATH", str(METAMODELS))
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_file(
        MODELS / "admittance_arc_single" / "admittance_arc_single.robmot"
    )
    builder = MotionSpecDatasetBuilder(model)
    dataset, context = builder.build()
    graph = dataset.default_graph

    monitor = next(graph.subjects(RDF.type, ROS.Topic))
    assert graph.value(monitor, CSTR_HDL.event) is not None
    assert str(graph.value(monitor, ROS["channel-name"])) == "/motion/forward_done"
    assert str(graph.value(monitor, ROS["type-name"])) == "bdd_ros2_interfaces/msg/Trinary"

    document = graph.serialize(format="json-ld", context=context)
    document = document.decode() if isinstance(document, bytes) else document
    assert '"ros": "https://index.ros.org/p/"' in document

    # An omitted `as <type>` falls back to the empty message type.
    node = URIRef("urn:test:default-topic")
    builder._emit_ros_topic(SimpleNamespace(ros_topic=ROSTopic(channel_name="/x")), node)
    assert str(graph.value(node, ROS["type-name"])) == "std_msgs/msg/Empty"


def test_ir_derives_forwarded_commands_and_monitors(generated_model: Path) -> None:
    ir = generate_ir(generated_model)
    forwarded = [command for motion in ir["motions"] for command in motion.forwarded_commands]
    assert len(forwarded) == 6
    assert all(command.target for command in forwarded)
    graph = _load_graph(generated_model)[1]
    # The progress guard is a lower bound on the measured speed along the path, so it names
    # the same path as the projection and never produces the parameter itself.
    (guard,) = list(graph.subjects(RDF.type, ALGO_EXT.ProgressConstraint))
    (projection,) = list(graph.subjects(RDF.type, GEOM_OP_EXT.PathProjection))
    assert graph.value(guard, GEOM_OP_EXT.path) == graph.value(projection, GEOM_OP_EXT.path)
    assert graph.value(guard, CSTR.quantity) == graph.value(
        projection, GEOM_OP_EXT["along-speed"]
    )
    assert CSTR.GreaterThanConstraint in graph[guard : RDF.type]
    forwarding_solvers = set(graph.subjects(RDF.type, SLV_EXT.CommandForwardingSolver))
    assert forwarding_solvers
    assert all(graph.value(solver, SLV.output) is not None for solver in forwarding_solvers)
    assert all(
        graph.value(monitor, CSTR_HDL.constraint) is not None
        for monitor in graph.subjects(RDF.type, CSTR_HDL.Monitor)
    )

    pick_above = next(motion for motion in ir["motions"] if motion.id == "motion_pick_above")
    scheduled = [ir["closures"][step] for step in pick_above.while_schedule]
    projection_call = next(
        closure for closure in scheduled if closure["type"] == "PathProjection"
    )
    assert projection_call["shape"] == "LinearPath"
    assert (projection_call["setpoint"], projection_call["path_parameter"]) == (
        "reference",
        "approach_path_s",
    )
    assert projection_call["assign_goal"]
    assert any(closure["type"] == "PoseDiffEvaluator" for closure in scheduled)
    assert any(component["id"] == "goal_pose" for component in pick_above.declared_pose_components)
    # The progress guard is a monitored condition, never a solver row: the schedule holds
    # exactly the 1 tangent + 2 normal + 3 angular controllers, and the guard appears only as
    # a while monitor over its along-speed error.
    assert [closure["type"] for closure in scheduled].count("Controller") == 6
    assert not any("advance" in closure["id"] and closure["type"] == "Controller" for closure in scheduled)
    assert len(pick_above.while_monitors) == 1
    assert pick_above.while_monitors[0].monitor_type == "LevelTriggeredMonitor"
    assert all(closure["type"] != "VelocityProfile" for closure in scheduled)


def test_generated_manifest_is_portable(generated_model: Path) -> None:
    document = json.loads(generated_model.read_text())
    text = json.dumps(document)
    assert str(generated_model.parent) not in text
    assert "https://secorolab.github.io/" in text
