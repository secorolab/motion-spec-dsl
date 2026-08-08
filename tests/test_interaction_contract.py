# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Invariants for the interaction path: force sensing, velocity control, per-motion ownership.

Each check here corresponds to a defect that a passing headless run did not reveal -- the FSM
completed start-to-end while the arm was uncommanded on an axis, blind to external force, or
recomputing another motion's state.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from motion_spec.rdf_parser.ir import generate_ir
from motion_spec_dsl.gens import _gen_graph
from motion_spec_dsl.langs import motion_spec_metamodel


MODELS = Path(__file__).parents[1] / "models"
METAMODELS = Path(__file__).resolve().parents[2] / "metamodels"


@pytest.fixture(scope="module")
def admittance_arc_manifest(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Generate the force-interaction model once per module (admittance, arc re-entry,
    until groups); consumers derive their own fresh IR from the immutable manifest."""
    tmp_path = tmp_path_factory.mktemp("admittance_arc_single")
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv("METAMODELS_PATH", str(METAMODELS))
        metamodel = motion_spec_metamodel()
        model = metamodel.model_from_file(
            MODELS / "admittance_arc_single" / "admittance_arc_single.robmot"
        )
        _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)
    return tmp_path / "admittance_arc_single-app.ld.json"


@pytest.fixture
def interaction_ir(admittance_arc_manifest: Path) -> dict:
    """Fresh IR per test: derived from the immutable manifest so no test can leak
    mutations of the IR's dataclasses/lists into another."""
    return generate_ir(admittance_arc_manifest)


def _motion(ir: dict, motion_id: str):
    return next(m for m in ir["coordination"]["motions"] if m.id == motion_id)


def _spatial_rows(motion) -> set[tuple[str, str]]:
    """The (subspace, axis) solver rows a motion actually commands."""
    return {
        (row.subspace, row.axis)
        for solver in motion.serial_chain_solvers
        for row in (
            *solver.motion_driver.acceleration_constraint,
            *solver.motion_driver.cartesian_acceleration,
        )
    }


def test_velocity_constraints_get_a_solver_row(interaction_ir: dict) -> None:
    """A velocity constraint must reach the solver.

    Axis derivation matched stale subspace tokens, so these produced no solver row: the
    controller computed a signal that codegen then dropped, leaving the axis uncommanded.
    """
    rows = _spatial_rows(_motion(interaction_ir, "motion_touchdown"))
    assert ("Linear", "Z") in rows, f"velocity constraint produced no Z row; rows={sorted(rows)}"

    comply_rows = _spatial_rows(_motion(interaction_ir, "motion_compliance"))
    assert {("Linear", "X"), ("Linear", "Y"), ("Linear", "Z")} <= comply_rows


def test_force_torque_sensor_reaches_the_runtime(interaction_ir: dict) -> None:
    """The scenex force-torque declaration must survive into the solver setup.

    It was hardcoded empty, so find_ft_sensor() returned null and the measured wrench stayed
    exactly zero for whole runs while the physics engine applied real forces.
    """
    sensors = [
        sensor
        for solver in interaction_ir["resources"]["by_kind"]["serial_chain"]
        for sensor in solver.sensors
        if sensor.type == "ForceTorque"
    ]
    assert sensors, "scenex declares force-torque wrist_ft but no solver carries it"
    assert {s.id for s in sensors} == {"wrist_ft"}
    assert all(s.frame for s in sensors)
    outputs = {
        (
            output.sensor_frame.id,
            output.reference_point.id,
            output.as_seen_by.id,
        )
        for solver in interaction_ir["resources"]["by_kind"]["serial_chain"]
        for output in solver.output
        if output.type == "Wrench" and output.sensor_name
    }
    assert outputs == {("wrist_ft_site", "wrist_ft_site", "base_link")}


def test_measured_wrench_defaults_to_its_physical_sensor_frame(tmp_path: Path) -> None:
    source_path = MODELS / "admittance_arc_single" / "admittance_arc_single.robmot"
    source = source_path.read_text().replace(
        "            ref-point:  <ft_tree.wrist_ft_body.wrist_ft_site>,\n"
        "            as-seen-by: <kinova.base_link.base_link_origin>,\n",
        "",
        1,
    )
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_str(source, file_name=str(source_path))
    _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)
    ir = generate_ir(tmp_path / "admittance_arc_single-app.ld.json")
    outputs = [
        output
        for solver in ir["resources"]["by_kind"]["serial_chain"]
        for output in solver.output
        if output.type == "Wrench" and output.sensor_name
    ]

    assert outputs
    assert {
        (output.sensor_frame.id, output.reference_point.id, output.as_seen_by.id)
        for output in outputs
    } == {("wrist_ft_site", "wrist_ft_site", "wrist_ft_site")}


def test_admittance_reference_is_produced_before_it_is_consumed(interaction_ir: dict) -> None:
    """A closure feeding a pose-axis error group must be scheduled, and scheduled first.

    Grouped evaluators were excluded from the schedule walk, so the admittance filter was
    generated but never called and its reference stayed 0.0 -- the compliant axes were in
    fact a stiff zero-velocity regulator.
    """
    compliance = _motion(interaction_ir, "motion_compliance")
    closures = interaction_ir["computation"]["closures"]
    admit = [c for c in closures if c.startswith("admit_")]
    assert admit, "model declares admittance references"
    assert set(admit) <= set(compliance.while_pre_schedule), (
        f"admittance closures unscheduled: {sorted(set(admit) - set(compliance.while_pre_schedule))}"
    )


def test_no_motion_captures_another_motions_snapshot(interaction_ir: dict) -> None:
    """Snapshots write shared slots, so a motion re-capturing another's retargets it.

    Entering the compliance motion used to re-derive the arc's end pose from the pose at that
    instant, moving the arc's goal every time the arm was pushed.
    """
    owners: dict[str, set[str]] = {}
    for motion in interaction_ir["coordination"]["motions"]:
        for snapshot in motion.snapshots:
            owners.setdefault(snapshot.target_id, set()).add(motion.id)
    shared = {t: m for t, m in owners.items() if len(m) > 1}
    assert not shared, f"snapshots captured by several motions: {shared}"


def test_no_motion_schedules_another_motions_closure(interaction_ir: dict) -> None:
    """The backward schedule walk must not pull in a closure another motion declares.

    The arc path was scheduled into every motion, so the compliance motion advanced the
    arc's path parameter and recomputed its setpoint while the arc was not running.
    """
    arc_only = {"arc_eval_arc_path"}
    for motion in interaction_ir["coordination"]["motions"]:
        if motion.id == "motion_arc_motion":
            continue
        scheduled = set(motion.while_schedule) | set(motion.while_pre_schedule)
        assert not (arc_only & scheduled), f"{motion.id} schedules the arc path"


def test_until_groups_are_monitored_independently(interaction_ir: dict) -> None:
    """Named until groups give one motion several transitions, each ANDed on its own."""
    compliance = _motion(interaction_ir, "motion_compliance")
    by_event = {m.event_name: m for m in compliance.until_monitors}
    assert {"E_FORCE_GONE", "E_TABLE_CONTACT"} <= set(by_event)

    released = by_event["E_FORCE_GONE"]
    assert len(released.group_constraint_ids) == 3, "release must cover every force axis"
    assert released.group_any is False
    assert len(released.active_terms) == 3

    table = by_event["E_TABLE_CONTACT"]
    assert len(table.group_constraint_ids) == 2
    assert set(released.group_constraint_ids) & set(table.group_constraint_ids) == set()


def test_arc_reentry_rebuilds_from_current_pose_to_the_fixed_target(
    interaction_ir: dict,
) -> None:
    """Arc re-entry re-samples only its start; the initial pose still anchors its target."""
    arc = _motion(interaction_ir, "motion_arc_motion")
    triggered = [s for s in arc.snapshots if s.trigger_event]
    assert {s.trigger_event for s in triggered} == {"E_ARC_ENTERED"}
    assert all(s.fsm_namespace for s in triggered)
    assert any(
        s.trigger_event is None and s.target_id.endswith("initial_pose")
        for s in arc.snapshots
    )
