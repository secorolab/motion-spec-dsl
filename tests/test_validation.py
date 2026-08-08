# SPDX-License-Identifier: MPL-2.0
"""Valid fixtures parse; mutating one clause of the base fixture makes validate_model reject it."""

from __future__ import annotations

import pytest
from textx.exceptions import TextXSemanticError

REJECTIONS = [
    pytest.param(
        "settled-z: <shared.world.twist-ee-base>.linvel.z equal to <shared.spec.zero-linvel>",
        "hold-position: <shared.world.twist-ee-base>.linvel.z equal to <shared.spec.zero-linvel>",
        "hold-position",
        id="validate_unique_constraint_names",
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
    pytest.param(
        "satisfied for 0.3 s { trigger: event <aas.E_HOME_SETTLED> },",
        "inactive { trigger: event <aas.E_HOME_SETTLED> },",
        "belongs in satisfied or violated",
        id="validate_monitor_trigger_placement",
    ),
    pytest.param(
        "satisfied for 0.3 s { trigger: event <aas.E_HOME_SETTLED> },",
        "inactive for 0.3 s { trigger: event <aas.E_HOME_SETTLED> },",
        "sustains 'inactive'",
        id="validate_monitor_sustained_inactive",
    ),
    pytest.param(
        "satisfied for 0.3 s { trigger: event <aas.E_HOME_SETTLED> },",
        "satisfied { flag: settled }, violated { hold: <home> }, violated { hold: <home> },",
        "'violated' state twice",
        id="validate_monitor_duplicate_state",
    ),
    pytest.param(
        "satisfied for 0.3 s { trigger: event <aas.E_HOME_SETTLED> },",
        "satisfied { trigger: event <aas.E_HOME_SETTLED> },"
        " violated { trigger: event <aas.E_HOME_SETTLED> },",
        "more than one 'trigger'",
        id="validate_monitor_duplicate_trigger",
    ),
    pytest.param(
        "satisfied for 0.3 s { trigger: event <aas.E_HOME_SETTLED> },",
        "satisfied { flag: settled, hold: <home> },",
        "belongs in violated",
        id="validate_monitor_hold_placement",
    ),
]


@pytest.mark.parametrize(("old", "new", "message"), REJECTIONS)
def test_invalid_model_is_rejected(parse_mutated, old, new, message):
    with pytest.raises(TextXSemanticError, match=message):
        parse_mutated(old, new)
