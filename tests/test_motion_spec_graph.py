# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Integration checks for the reduced motion-spec RDF contract."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from pyshacl import validate
from rdflib import Graph, Namespace, URIRef
from rdflib.namespace import RDF

from motion_spec.ir_gen import Parser, _authored_controller_axes, _load_graph, generate_ir
from motion_spec.namespace import (
    ALGO_EXT,
    APP,
    CSTR_HDL,
    CSTR_HDL_EXT,
    EXEC,
    GEOM_OP,
    GEOM_REL,
    MAP,
    MAP_EXT,
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


@pytest.fixture
def generated_model(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """Generate the representative model and return its application manifest."""
    monkeypatch.setenv("METAMODELS_PATH", str(METAMODELS))
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_file(MODELS / "pick_place_single" / "pick_place_single.robmot")
    _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)
    return tmp_path / "pick_place_single-app.ld.json"


@pytest.fixture
def generated_dual_model(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """Generate the dual-arm model used to exercise profiles and repeated robot assets."""
    monkeypatch.setenv("METAMODELS_PATH", str(METAMODELS))
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
    assert "app:pick-above/while/follow-pos" in identifiers
    assert not any(identifier.startswith(("http://", "https://", "/")) for identifier in identifiers)


def test_dual_arm_physical_profiles_and_path_progress_reach_ir(generated_dual_model: Path) -> None:
    graph = _load_graph(generated_dual_model)[1]
    assert len(set(graph.subjects(RDF.type, CSTR_HDL_EXT["LinearJerk"]))) == 1
    objectives = set(graph.subjects(RDF.type, ALGO_EXT.ProgressObjective))
    (handler_node,) = set(graph.subjects(ALGO_EXT.progress, None))
    assert set(graph.objects(handler_node, ALGO_EXT.progress)) == objectives
    assert len(Parser(graph).constraint_handler(handler_node).progress) == 2

    ir = generate_ir(generated_dual_model)
    profiles = [value for value in ir["closures"].values() if value.get("type") == "VelocityProfile"]

    motion_profiles = profiles
    assert len(motion_profiles) == 2
    assert all(str(profile["shape"]) == "s_curve" and profile["in"] for profile in motion_profiles)
    assert all(profile["maximum_jerk"] == "max_lower_jerk" for profile in motion_profiles)
    pick_above = next(motion for motion in ir["motions"] if motion.id == "motion_pick_above")
    assert {objective.parameter for objective in pick_above.progress_objectives} == {
        "alpha1",
        "alpha2",
    }
    assert {objective.id for objective in pick_above.progress_objectives} == {
        "arm1_approach",
        "arm2_approach",
    }
    assert all(len(objective.paths) == 1 for objective in pick_above.progress_objectives)
    assert all(objective.errors for objective in pick_above.progress_objectives)
    assert [robot.prefix for robot in ir["scene"].robots] == ["kinova1_", "kinova2_"]
    objects = {obj.id: obj for obj in ir["scene"].objects}
    assert set(objects) == {"table", "cube", "cube2"}
    robots = {robot.id: robot for robot in ir["scene"].robots}
    assert robots["arm1"].pos == [-0.7, 0.0, 0.0]
    assert robots["arm2"].pos == [0.7, 0.0, 0.0]
    assert robots["arm2"].euler == [0.0, 0.0, 180.0]
    assert robots["arm1"].attach_name == "table_table_top"
    assert robots["arm2"].attach_name == "table_table_top"
    pick_above_outputs = {
        solver.chain_root: {output.id for output in solver.output if output.id.startswith("pose_ee")}
        for solver in ir["slv_arm"]
        if solver.id.endswith("pick_above")
    }
    assert pick_above_outputs == {
        "kinova1_base_link": {"pose_ee1_base"},
        "kinova2_base_link": {"pose_ee2_base"},
    }
    grasp_outputs = {
        solver.chain_root: {
            output.id: output.joint_name
            for output in solver.output
            if output.id.startswith("gripper")
        }
        for solver in ir["slv_arm"]
        if solver.id.endswith("grasp_hold")
    }
    assert grasp_outputs == {
        "kinova1_base_link": {"gripper1_pos": "kinova1_g_left_driver_joint"},
        "kinova2_base_link": {"gripper2_pos": "kinova2_g_left_driver_joint"},
    }
    grasp_motion = next(motion for motion in ir["motions"] if motion.id == "motion_grasp_hold")
    assert {
        "ctrl_cg1_close_gripper",
        "ctrl_cg2_close_gripper",
    } <= set(grasp_motion.while_schedule)


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

    objective = next(data.subjects(RDF.type, ALGO_EXT.ProgressObjective))
    advancement = data.value(objective, ALGO_EXT.advancement)
    data.set((advancement, QUDT_SCHEMA.unit, URIRef("http://qudt.org/vocab/unit/KiloHZ")))
    conforms, _, _ = validate(data, shacl_graph=shapes, inference="rdfs")
    assert not conforms

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
    unused_terms = ("ROSPackage", "ROSAction", "ROSService", "HasFrameId")
    assert not any(term in document for term in unused_terms)

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
    objectives = list(graph.subjects(RDF.type, ALGO_EXT.ProgressObjective))
    assert len(objectives) == 1
    (handler_node,) = graph.subjects(ALGO_EXT.progress, objectives[0])
    motion_node = graph.value(handler_node, CSTR_HDL.motion)
    assert CSTR_HDL.ConstraintHandler in graph[handler_node : RDF.type]
    assert (motion_node, ALGO_EXT.progress, objectives[0]) not in graph
    parsed_handler = Parser(graph).constraint_handler(handler_node)
    assert len(parsed_handler.progress) == 1
    assert not hasattr(parsed_handler.motion, "progress")
    assert graph.value(objectives[0], ALGO_EXT.parameter) is not None
    assert graph.value(objectives[0], ALGO_EXT.path) is not None
    assert not any(
        graph.value(profile, ALGO_EXT.out) == graph.value(objectives[0], ALGO_EXT.parameter)
        for profile in graph.subjects(RDF.type, ALGO_EXT.VelocityProfile)
    )
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
    interpolation = next(closure for closure in scheduled if closure["type"] == "LinearPath")
    assert (interpolation["setpoint"], interpolation["path_parameter"]) == (
        "reference",
        "s",
    )
    assert interpolation["assign_goal"]
    assert any(closure["type"] == "PoseDiffEvaluator" for closure in scheduled)
    assert any(component["id"] == "goal_pose" for component in pick_above.declared_pose_components)
    assert {snapshot.target_id for snapshot in pick_above.snapshots} >= {
        "start_pose",
        "start_cube_x",
        "start_cube_y",
    }
    assert len(pick_above.progress_objectives) == 1
    objective = pick_above.progress_objectives[0]
    assert objective.parameter == "s"
    assert objective.id == "approach"
    assert objective.paths == ["lerp_approach_path"]
    assert len(objective.errors) == 6
    assert all(closure["type"] != "VelocityProfile" for closure in scheduled)
    pose_ee_base = next(item for item in ir["shared_data"] if item.id == "pose_ee_base")
    assert pose_ee_base.with_respect_to.id == "base_link"


def test_one_named_progress_policy_synchronizes_two_paths(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """One shared parameter is gated by the union of both paths' controller errors."""
    monkeypatch.setenv("METAMODELS_PATH", str(METAMODELS))
    model_path = MODELS / "pick_place_dual" / "pick_place_dual.robmot"
    source = model_path.read_text().replace(
        "            path-parameter alpha1,", "            path-parameter s,"
    ).replace("            path-parameter alpha2,\n", "").replace(
        """    progress {
        arm1-approach: maximizing <pick-above.spec.alpha1> along <pick-above.spec.arm1-approach-path> advancing at 1.0 Hz,
        arm2-approach: maximizing <pick-above.spec.alpha2> along <pick-above.spec.arm2-approach-path> advancing at 1.0 Hz
    }""",
        """    progress {
        dual-approach: maximizing <pick-above.spec.s> along {
            <pick-above.spec.arm1-approach-path>,
            <pick-above.spec.arm2-approach-path>
        } advancing at 1.0 Hz
    }""",
    )
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_str(source, file_name=str(model_path))
    _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)

    manifest = tmp_path / "pick_place_dual-app.ld.json"
    graph = _load_graph(manifest)[1]
    (objective_node,) = graph.subjects(RDF.type, ALGO_EXT.ProgressObjective)
    assert len(set(graph.objects(objective_node, ALGO_EXT.path))) == 2

    ir = generate_ir(manifest)
    pick_above = next(motion for motion in ir["motions"] if motion.id == "motion_pick_above")
    (objective,) = pick_above.progress_objectives
    assert objective.id == "dual_approach"
    assert objective.parameter == "s"
    assert set(objective.paths) == {
        "lerp_arm1_approach_path",
        "lerp_arm2_approach_path",
    }
    assert len(objective.constraints) == 4
    assert len(objective.errors) == 12


def test_generated_manifest_is_portable(generated_model: Path) -> None:
    document = json.loads(generated_model.read_text())
    text = json.dumps(document)
    assert str(generated_model.parent) not in text
    assert "https://secorolab.github.io/" in text
