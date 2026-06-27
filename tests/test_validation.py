# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

from pathlib import Path
import re

import pytest
from textx.exceptions import TextXSemanticError, TextXSyntaxError

from motion_spec_dsl.domain import (
    ConstraintAlias,
    ControllerAlias,
    HandlerControlMode,
    QuantityType,
    SolverAlias,
    ContextQuantityAlias,
    WorldQuantityAlias,
)
from motion_spec_dsl.registration import motion_spec_metamodel


FIXTURES = Path(__file__).parent / "fixtures"
VALID_FIXTURES = FIXTURES / "valid"
INVALID_FIXTURES = FIXTURES / "invalid"

AMBIGUOUS_CONTROLLER_COMMAND = "01_core_semantics/05_ambiguous_controller_command.robmot"
STANDALONE_MANIPULATOR = "01_core_semantics/01_standalone_manipulator.robmot"
POSTURE_CONTROLLER = "04_posture_control/01_posture_controller.robmot"
JOINT_LIMIT_POSTURE = "04_posture_control/02_joint_limit_posture.robmot"
IMPEDANCE_CONTROLLER = "01_core_semantics/08_impedance_controller.robmot"


@pytest.mark.parametrize(
    ("fixture", "message"),
    [
        (
            "01_constraints_and_handlers/01_duplicate_constraint.robmot",
            "duplicate constraint name(s): c1",
        ),
        (
            "01_constraints_and_handlers/02_handler_motion_mismatch.robmot",
            "primary motion 'm_a' does not assemble it",
        ),
        (
            "01_constraints_and_handlers/03_missing_controller.robmot",
            "WHILE constraints must have at least one controller",
        ),
        (
            "01_constraints_and_handlers/04_missing_monitor.robmot",
            "WHEN or UNTIL constraints must have at least one monitor",
        ),
        (
            "02_controllers_and_solvers/01_apply_at_non_link.robmot",
            "apply at target must be a Link",
        ),
        (
            "02_controllers_and_solvers/05_mixed_solver_same_domain.robmot",
            "uses RNE, but RNE is not modeled",
        ),
        (
            "02_controllers_and_solvers/06_unsupported_impedance_controller.robmot",
            "produces Force and must specify 'apply at <link>'",
        ),
        (
            "02_controllers_and_solvers/02_joint_position_missing_torque_command.robmot",
            "targets JointPosition and must use 'as Torque'",
        ),
        (
            "02_controllers_and_solvers/03_missing_controller_solver.robmot",
            "must specify solver because handler 'handler_move' assembles 2 solvers",
        ),
        (
            "02_controllers_and_solvers/04_duplicate_achd_axis.robmot",
            "Multiple constraints on the same Cartesian axis are not supported yet",
        ),
        (
            "02_controllers_and_solvers/09_unsupported_explicit_command_type.robmot",
            "Only 'as Force' is supported",
        ),
        (
            "02_controllers_and_solvers/07_unsupported_abag_controller.robmot",
            "uses ABAG, but ABAG is not implemented yet",
        ),
        (
            "02_controllers_and_solvers/08_joint_torque_mode_unsupported_solver.robmot",
            "control mode JointTorque is not supported by solver 'kinova_solver' "
            "with algorithm ForceDistribution",
        ),
    ],
)
def test_invalid_models_fail_validation(fixture: str, message: str) -> None:
    metamodel = motion_spec_metamodel()

    with pytest.raises(TextXSemanticError, match=re.escape(message)):
        metamodel.model_from_file(INVALID_FIXTURES / fixture)


def test_pose_position_controller_command_is_inferred() -> None:
    metamodel = motion_spec_metamodel()

    metamodel.model_from_file(VALID_FIXTURES / AMBIGUOUS_CONTROLLER_COMMAND)


def test_standalone_manipulator_solver_refs_use_robot_name() -> None:
    metamodel = motion_spec_metamodel()

    model = metamodel.model_from_file(VALID_FIXTURES / STANDALONE_MANIPULATOR)
    handler = model.specs[-1]
    solver = handler.solvers[0]

    assert str(solver.robot) == "world.kinova"
    assert str(solver.root) == "world.kinova.chain.root"
    assert str(solver.end) == "world.kinova.chain.end"


def test_crf_model_supports_context_and_solver_references() -> None:
    metamodel = motion_spec_metamodel()

    model = metamodel.model_from_file(Path(__file__).parents[1] / "models" / "crf.robmot")
    motion_loosen = next(spec for spec in model.specs if getattr(spec, "name", "") == "m-loosen")
    handler_loosen = next(spec for spec in model.specs if getattr(spec, "name", "") == "handler-loosen")

    loosen_world = motion_loosen.context[0].declaration
    loosen_spec = motion_loosen.context[2].declaration
    loosen_constraints = motion_loosen.while_.constraints
    loosen_controllers = handler_loosen.controllers
    handler_solver = handler_loosen.solvers[0]

    assert isinstance(loosen_world[0], WorldQuantityAlias)
    assert loosen_world[0].name == loosen_world[0].ref.name
    assert isinstance(loosen_spec[1], ContextQuantityAlias)
    assert loosen_spec[1].name == loosen_spec[1].ref.name
    assert isinstance(loosen_constraints[1], ConstraintAlias)
    assert loosen_constraints[1].name == loosen_constraints[1].ref.constraint.name
    assert isinstance(loosen_controllers[1], ControllerAlias)
    assert loosen_controllers[1].name == loosen_controllers[1].ref.controller.name
    assert isinstance(handler_solver, SolverAlias)
    assert handler_solver.name == handler_solver.ref.solver.name
    assert handler_solver.uri == handler_solver.ref.solver.uri


def test_posture_controller_accepts_unilateral_and_bilateral_joint_limits() -> None:
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_file(VALID_FIXTURES / JOINT_LIMIT_POSTURE)
    handler = model.specs[-1]
    controllers = {c.name: c for c in handler.controllers}

    assert handler.control_mode == HandlerControlMode.JointTorque
    assert controllers["ctrl-bilateral-j2"].command_type == QuantityType.Torque
    assert controllers["ctrl-less-than-j4"].command_type == QuantityType.Torque


def test_posture_controller_uses_single_handler_solver_implicitly() -> None:
    metamodel = motion_spec_metamodel()

    model = metamodel.model_from_file(VALID_FIXTURES / POSTURE_CONTROLLER)
    handler = model.specs[-1]
    controller = handler.controllers[0]

    assert handler.control_mode == HandlerControlMode.JointTorque
    assert controller.solver is None
    assert controller.command_type == QuantityType.Torque


def test_impedance_controller_passes_validation() -> None:
    metamodel = motion_spec_metamodel()

    model = metamodel.model_from_file(VALID_FIXTURES / IMPEDANCE_CONTROLLER)
    handler = model.specs[-1]
    controller = handler.controllers[0]

    assert controller.type.value == "Impedance"
    assert controller.params.stiffness == 1.0
    assert controller.params.damping == 0.1


def test_constraint_handler_requires_control_period() -> None:
    metamodel = motion_spec_metamodel()

    with pytest.raises(TextXSyntaxError, match="CONTROL_PERIOD"):
        metamodel.model_from_str(
            """ns app = "https://secorolab.github.io/models/tests/"

MOTION_SPEC (ns=app) move {
    CONTEXT {
        w: World { joint: JointPosition { of: j1 } },
        s: Spec { target: Angle = 0.0 rad }
    }
    WHEN {}
    WHILE { hold: keeping <w.joint> equal to <s.target> }
    UNTIL {}
}

CONSTRAINT_HANDLER (ns=app) handler_move {
    CONTEXT {}
    MOTION: <move>
    CONTROL_MODE: JointTorque
    CONTROLLERS {
        ctrl-hold: PID { constraint: <move.hold>, Kp = 1.0, Ki = 0.0, Kd = 0.1 }
    }
    SOLVERS {
        arm-solver: Solver {
            robot: <robot>,
            algorithm: ACHD,
            root: <root>,
            end: <end>
        }
    }
}
"""
        )


@pytest.mark.parametrize(
    ("constraint", "message"),
    [
        (
            "keeping <w.pose-ee-base>.position equal to <s.pose-start>",
            "Constraint 'c' compares Position with Pose.",
        ),
        (
            "keeping <w.pose-ee-base>.orientation equal to <s.pose-start>",
            "Constraint 'c' compares Orientation with Pose.",
        ),
        (
            "keeping <w.pose-ee-base>.position equal to <s.traj>",
            "Constraint 'c' compares Position with Trajectory.",
        ),
        (
            "keeping <w.pose-ee-base>.orientation equal to <s.traj>",
            "Constraint 'c' compares Orientation with Trajectory.",
        ),
    ],
)
def test_pose_subspace_constraints_require_matching_reference_subspace(
    constraint: str, message: str
) -> None:
    metamodel = motion_spec_metamodel()
    model = f"""ns app = \"https://secorolab.github.io/models/tests/\"

MOTION_SPEC (ns=app) bad_pose_subspace {{
    CONTEXT {{
        w: World {{
            pose-ee-base: Pose {{ of: ee, wrt: base, as-seen-by: base }}
        }},
        s: Spec {{
            alpha: TrajectoryProgress,
            pose-start: Pose = Snapshot of <w.pose-ee-base>,
            traj: Trajectory = Lerp {{
                start: <s.pose-start>,
                goal:  <s.pose-start>,
                alpha: <s.alpha>
            }}
        }}
    }}

    WHEN {{}}
    WHILE {{
        c: {constraint}
    }}
    UNTIL {{}}
}}
"""

    with pytest.raises(TextXSemanticError, match=re.escape(message)):
        metamodel.model_from_str(model)
