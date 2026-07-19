# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Integration checks for the reduced motion-spec RDF contract."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from pyshacl import validate
from rdflib import Graph, Namespace, URIRef
from rdflib.namespace import RDF

from motion_spec.ir_gen import _authored_controller_axes, _load_graph, generate_ir
from motion_spec.namespace import (
    ALGO_EXT,
    APP,
    CSTR_HDL,
    EXEC,
    GEOM_OP,
    GEOM_REL,
    MAP_EXT,
    SLV,
    SLV_EXT,
)
from motion_spec_dsl.registration import _gen_graph, motion_spec_metamodel


MODELS = Path(__file__).parents[1] / "models"
METAMODELS = Path(__file__).resolve().parents[2] / "metamodels"


@pytest.fixture
def generated_model(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """Generate the representative model and return its application manifest."""
    monkeypatch.setenv("METAMODELS_PATH", str(METAMODELS))
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_file(MODELS / "pick_place_single.robmot")
    _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)
    return tmp_path / "pick_place_single-app.jsonld"


def test_generation_keeps_scene_fsm_and_provenance_separate(generated_model: Path) -> None:
    output = generated_model.parent
    assert (output / "pick_place_single.jsonld").exists()
    assert (output / "pick_place_single.scenex.jsonld").exists()
    assert (output / "pick_place_single_fsm.jsonld").exists()

    provenance = Graph().parse(output / "provenance" / "dsl.jsonld", format="json-ld")
    prov = Namespace("http://www.w3.org/ns/prov#")
    activity = URIRef(
        "https://secorolab.github.io/motion-spec-dsl/provenance/"
        "activity/jsonld_generation/pick_place_single"
    )
    assert (activity, RDF.type, prov.Activity) in provenance


def test_graph_uses_only_the_reduced_extension_contract(generated_model: Path) -> None:
    graph = _load_graph(generated_model)[1]
    serialized = graph.serialize(format="nt")

    obsolete_terms = {
        "SignalLimiter",
        "IntegralSaturation",
        "TorqueSaturation",
        "AccelerationSaturation",
        "DirectionAligned",
        "JointCommand",
        "monitors-until",
        "monitors-when",
        "forwards-command",
        "command-signal",
        "regularization",
        "sampled-on",
        "ft-sensor-ref",
        "AddQuantity",
        "Norm",
        "PoseOrientationView",
        "PosePositionView",
        "WrenchVectorView",
        "ComputeRotationFromPose",
        "attached-body",
    }
    assert not any(term in serialized for term in obsolete_terms)
    assert ".origin>" not in serialized
    assert ".body>" not in serialized

    assert not set(graph.subjects(RDF.type, GEOM_OP.InvertPose))
    assert not set(graph.subjects(RDF.type, GEOM_OP.ComposePose))
    assert not set(graph.subjects(RDF.type, GEOM_OP.PoseToLinearDistance))
    distances = set(graph.subjects(RDF.type, GEOM_REL.LinearDistance))
    assert distances
    assert all(
        len(set(graph.objects(distance, GEOM_REL["between-entities"]))) == 2
        for distance in distances
    )
    assert set(graph.subjects(RDF.type, ALGO_EXT.Addition))

    execution_context = next(graph.subjects(RDF.type, URIRef(f"{EXEC._NS}ExecutionContext")))
    assert graph.value(execution_context, URIRef(f"{EXEC._NS}runs-scene")) is not None
    assert graph.value(execution_context, URIRef(f"{EXEC._NS}timestep")) is not None

    allowed_map_types = {
        MAP_EXT.PoseCoordinateView,
        MAP_EXT.VelocityTwistCoordinateView,
        MAP_EXT.AccelerationTwistCoordinateView,
        MAP_EXT.WrenchCoordinateView,
        MAP_EXT.PoseDifferenceView,
    }
    emitted_map_types = {
        type_
        for type_ in graph.objects(predicate=RDF.type)
        if str(type_).startswith(str(MAP_EXT._NS))
    }
    assert emitted_map_types <= allowed_map_types


def test_reduced_shacl_contract_conforms(generated_model: Path) -> None:
    data = _load_graph(generated_model)[1]
    constraint_locations = set(map(str, data.objects(predicate=APP.constraints)))
    assert constraint_locations
    shapes = Graph()
    for location in constraint_locations:
        shapes.parse(location)

    conforms, _, report = validate(data, shacl_graph=shapes, inference="rdfs")
    assert conforms, report

    assert (
        "https://secorolab.github.io/metamodels/algorithm-extension.shacl.ttl"
        in constraint_locations
    )
    assert (
        "https://secorolab.github.io/metamodels/geometry/spatial-relations-extension.shacl.ttl"
        in constraint_locations
    )
    assert (
        "https://secorolab.github.io/metamodels/geometry/geometry.shacl.ttl"
        not in constraint_locations
    )
    assert not any(
        location.startswith("https://comp-rob2b.github.io/metamodels/geometry/")
        for location in constraint_locations
    )


def test_ir_derives_forwarded_commands_and_monitors(generated_model: Path) -> None:
    ir = generate_ir(generated_model)
    forwarded = [command for motion in ir["motions"] for command in motion.forwarded_commands]
    assert len(forwarded) == 6
    assert all(command.target for command in forwarded)
    graph = _load_graph(generated_model)[1]
    authored_axes = _authored_controller_axes(graph)
    assert authored_axes
    assert sum(len(axes) for axes in authored_axes.values()) == 60
    forwarding_solvers = set(graph.subjects(RDF.type, SLV_EXT.CommandForwardingSolver))
    assert forwarding_solvers
    assert all(graph.value(solver, SLV.output) is not None for solver in forwarding_solvers)
    assert all(
        graph.value(monitor, CSTR_HDL.constraint) is not None
        for monitor in graph.subjects(RDF.type, CSTR_HDL.Monitor)
    )

    (robot,) = ir["scene"].robots
    assert (robot.id, robot.attach_kind, robot.attach_name) == (
        "kinova_2f85",
        "Site",
        "table_table_top",
    )
    (gripper,) = robot.attachments
    assert (gripper.id, gripper.attach_to, gripper.prefix) == (
        "gripper",
        "pinch_site",
        "g_",
    )
    objects = {obj.id: obj for obj in ir["scene"].objects}
    assert objects["table"].pos == [0.0, 0.0, 0.72]
    assert objects["cube"].pos == [0.5, 0.0, 0.76]
    assert not objects["cube"].fixed

    arm_solver = ir["slv_arm"][0]
    assert (
        arm_solver.chain_root,
        arm_solver.chain_tip,
        arm_solver.tool_body,
        arm_solver.tcp_site,
    ) == ("base_link", "bracelet_link", "g_base", "g_pinch")
    assert {
        "pose_cube_base",
        "pose_ee_base",
        "pose_elbow_base",
        "twist_ee_base",
        "gripper_pos",
    } <= {output.id for output in arm_solver.output}
    cube_pose = next(output for output in arm_solver.output if output.id == "pose_cube_base")
    assert (cube_pose.of.id, cube_pose.of.body, cube_pose.of.is_scene_object) == (
        "cube",
        "cube",
        True,
    )
    pick_above = next(motion for motion in ir["motions"] if motion.id == "motion_pick_above")
    scheduled = [ir["closures"][step] for step in pick_above.while_schedule]
    interpolation = next(
        closure for closure in scheduled if closure["type"] == "CartesianPoseInterpolation"
    )
    assert (interpolation["trajectory"], interpolation["path_parameter"]) == (
        "reference",
        "progress",
    )
    assert interpolation["assign_goal"]
    assert any(closure["type"] == "PoseDiffEvaluator" for closure in scheduled)
    assert any(component["id"] == "goal_pose" for component in pick_above.declared_pose_components)
    assert {snapshot.target_id for snapshot in pick_above.snapshots} >= {
        "start_pose",
        "start_cube_x",
        "start_cube_y",
    }
    assert pick_above.time_trajectory_progress_ids == ["progress"]
    pose_ee_base = next(item for item in ir["shared_data"] if item.id == "pose_ee_base")
    assert pose_ee_base.with_respect_to.id == "base_link"


def test_generated_manifest_is_portable(generated_model: Path) -> None:
    document = json.loads(generated_model.read_text())
    text = json.dumps(document)
    assert str(generated_model.parent) not in text
    assert "https://secorolab.github.io/" in text
