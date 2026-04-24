# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

from pathlib import Path
import re

import pytest
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.generators.classes import (
    ConstraintReference,
    ControllerReference,
    SolverReference,
    ValueVariableReference,
    WorldQuantityReference,
)
from motion_spec_dsl.generators.registration import motion_spec_metamodel


FIXTURES = Path(__file__).parent / "fixtures"


@pytest.mark.parametrize(
    ("fixture", "message"),
    [
        (
            "duplicate_constraint.robmot",
            "duplicate constraint name(s): c1",
        ),
        (
            "handler_motion_mismatch.robmot",
            "primary motion 'm_a' does not assemble it",
        ),
        (
            "missing_controller.robmot",
            "WHILE constraints must have at least one controller",
        ),
        (
            "missing_monitor.robmot",
            "WHEN or UNTIL constraints must have at least one monitor",
        ),
        (
            "apply_at_non_link.robmot",
            "apply at target must be a Link",
        ),
        (
            "ambiguous_controller_command.robmot",
            "requires explicit 'as'",
        ),
    ],
)
def test_invalid_models_fail_validation(fixture: str, message: str) -> None:
    metamodel = motion_spec_metamodel()

    with pytest.raises(TextXSemanticError, match=re.escape(message)):
        metamodel.model_from_file(FIXTURES / fixture)


def test_standalone_manipulator_solver_refs_use_robot_name() -> None:
    metamodel = motion_spec_metamodel()

    model = metamodel.model_from_file(FIXTURES / "standalone_manipulator.robmot")
    handler = model.specs[-1]
    solver = handler.solvers[0]

    assert str(solver.robot) == "kinova"
    assert str(solver.root) == "kinova.chain.root"
    assert str(solver.end) == "kinova.chain.end"


def test_crf_model_supports_context_and_solver_references() -> None:
    metamodel = motion_spec_metamodel()

    model = metamodel.model_from_file(Path(__file__).parents[1] / "models" / "crf.robmot")
    motion_loosen = next(spec for spec in model.specs if getattr(spec, "name", "") == "motion_loosen")
    handler_loosen = next(spec for spec in model.specs if getattr(spec, "name", "") == "handler_loosen")

    loosen_world = motion_loosen.context[0].declaration
    loosen_spec = motion_loosen.context[2].declaration
    loosen_constraints = motion_loosen.while_.constraints
    loosen_controllers = handler_loosen.controllers
    handler_solver = handler_loosen.solvers[0]

    assert isinstance(loosen_world[0], WorldQuantityReference)
    assert isinstance(loosen_spec[1], ValueVariableReference)
    assert isinstance(loosen_constraints[1], ConstraintReference)
    assert isinstance(loosen_controllers[1], ControllerReference)
    assert isinstance(handler_solver, SolverReference)
    assert handler_solver.uri == handler_solver.ref.solver.uri
