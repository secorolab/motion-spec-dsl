# SPDX-License-Identifier: MPL-2.0
"""Paths and the base fixture's edit anchors shared by the tests."""

from __future__ import annotations

from pathlib import Path

MODELS = Path(__file__).parents[1] / "src" / "motion_spec_dsl" / "models"
BASE = Path(__file__).parent / "fixtures" / "base.robmot"
BASE_TEXT = BASE.read_text()

EXEC = "exec-context (ns=app) base-exec {"
SIM = 'platform:   simulation { name: "MuJoCo" }'
MOTION = "guarded-motion (ns=app) home {"
TWIST = """        velocity-twist twist-ee-base {
            of:         <gripper.g_base.g_pinch>,
            wrt:        <kinova.base_link.base_link_origin>,
            as-seen-by: <kinova.base_link.base_link_origin>
        }"""
SPEC = "linear-velocity zero-linvel = 0.0 m/s"
SNAPSHOT = "pose home-pose = snapshot of <shared.world.pose-ee-base> on event <aas.E_HOME_ENTERED>"
HOLD = (
    "hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
)
UNTIL = (
    "settled-z: <shared.world.twist-ee-base>.linvel.z equal to <shared.spec.zero-linvel> "
    "within <shared.spec.satisfied-band-vel>"
)
MONITOR = "satisfied for 0.3 s { trigger: event <aas.E_HOME_SETTLED> },"
CTRL = (
    "pid ctrl-hold-position { constraint: <home.hold-position>, "
    "Kp: 200, Ki: 100, Kd: 40, decay: 0 }"
)
SOLVERS_END = "            gravity: (0.0, 0.0, 9.81) m/s^2\n        }\n    }\n"

POSE_TABLE_TOP = """,
        pose pose-table-top {
            of:         <table.table_top>,
            wrt:        <kinova.base_link.base_link_origin>,
            as-seen-by: <kinova.base_link.base_link_origin>
        }"""
TABLE_PLANE = (
    ",\n        direction table-normal { as-seen-by: <kinova.base_link.base_link_origin> } = (0, 0, 1)"
    ",\n        plane table { of: <table.table_top>, normal: <shared.spec.table-normal> }"
)

SERVER = """ros (ns=app) {
    action-servers {
        arc-behaviour: action "run_arc" type "control_msgs/action/GripperCommand" {
            on-goal: produce event <aas.E_HOME_SETTLED>,
        },
    },
}

"""
TOPICS = """ros (ns=app) {
    publishers {
        settled: topic "/base/settled" message "std_msgs/msg/Float64",
        events: topic "/events" message "std_msgs/msg/String",
    },
}

"""
OCCURRENCE = (
    "satisfied for 0.3 s { trigger: event <aas.E_HOME_SETTLED>, "
    "publish: events { <aas.E_HOME_SETTLED> } to <ros.publishers.events> },"
)
ACTIONS = """ros (ns=app) {
    action-clients {
        locate: action "/perception/locate" type "aruco_perception/action/LocateObjects",
    },
}

"""
LOCATED = f"{UNTIL},\n        located: <find-ee>.status equal to succeeded"

PUSH = (
    ",\n        force push = 5.0 N"
    ",\n        direction push-dir { as-seen-by: <kinova.base_link.base_link_origin> } = (1, 0, 0)"
    ",\n        duration push-time = 0.5 s"
)
PERTURBATION = (
    f"{SOLVERS_END}    perturbations {{\n        nudge: on <gripper.g_base> apply force"
    " <shared.spec.push> along <shared.spec.push-dir> for <shared.spec.push-time>\n    }\n"
)
