# SPDX-License-Identifier: MPL-2.0
"""Valid fixtures parse; mutating one clause of the base fixture makes validate_model reject it."""

from __future__ import annotations

import pytest
from textx.exceptions import TextXSemanticError

REJECTIONS = [
    pytest.param(
        "linear-velocity zero-linvel = 0.0 m/s",
        "linear-velocity zero-linvel = 0.0 m/s,\n        linear-velocity gravity = 0.0 m/s",
        "gravity.*keyword",
        id="reject_keyword_names",
    ),
    pytest.param(
        "settled-z: <shared.world.twist-ee-base>.linvel.z equal to <shared.spec.zero-linvel>",
        "hold-position: <shared.world.twist-ee-base>.linvel.z equal to <shared.spec.zero-linvel>",
        "hold-position",
        id="validate_unique_constraint_names",
    ),
    # validate_supported_solver_algorithms has no case: the grammar's algorithm rule is a
    # closed enum, so an unsupported name is a syntax error and the validator never runs.
    pytest.param(
        "Kp: 200, Ki: 100, Kd: 40, decay: 0",
        "Kp: 200, Ki: 100, Kd: 40, Kp: 1, decay: 0",
        "repeats parameter",
        id="validate_controller_commands_duplicate_param",
    ),
    pytest.param(
        'ns app = "https://secorolab.github.io/models/base/"',
        'ns app = "https://secorolab.github.io/models/base"',
        "must end with",
        id="validate_namespace_uris_missing_separator",
    ),
    pytest.param(
        'ns app = "https://secorolab.github.io/models/base/"',
        'ns app = "https://secorolab.github.io/models//base/"',
        "empty path segment",
        id="validate_namespace_uris_empty_segment",
    ),
]


@pytest.mark.parametrize(("old", "new", "message"), REJECTIONS)
def test_invalid_model_is_rejected(parse_mutated, old, new, message):
    with pytest.raises(TextXSemanticError, match=message):
        parse_mutated(old, new)
