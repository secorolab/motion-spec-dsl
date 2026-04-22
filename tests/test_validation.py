# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

from pathlib import Path
import re

import pytest
from textx.exceptions import TextXSemanticError

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
            "references motion 'm_b', but handler 'handler_a' is bound to motion 'm_a'",
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
