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
        "violated { flag: settled },",
        "belongs in satisfied",
        id="validate_monitor_flag_placement",
    ),
    pytest.param(
        "satisfied for 0.3 s { trigger: event <aas.E_HOME_SETTLED> },",
        "satisfied { flag: settled }, violated { hold: <home> }, violated { hold: <home> },",
        "'violated' state twice",
        id="validate_monitor_duplicate_state",
    ),
    pytest.param(
        "satisfied for 0.3 s { trigger: event <aas.E_HOME_SETTLED> },",
        "satisfied { trigger: event <aas.E_HOME_SETTLED>, trigger: event <aas.E_HOME_SETTLED> },",
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


TWIST_ANCHOR = """        velocity-twist twist-ee-base {
            of:         <gripper.g_base.g_pinch>,
            wrt:        <kinova.base_link.base_link_origin>,
            as-seen-by: <kinova.base_link.base_link_origin>
        }"""
SPEC_ANCHOR = "        linear-velocity zero-linvel = 0.0 m/s"
WHILE_ANCHOR = (
    "        hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
)
CTRL_ANCHOR = (
    "        pid ctrl-hold-position { constraint: <home.hold-position>, "
    "Kp: 200, Ki: 100, Kd: 40, decay: 0 }"
)
MEASURED = """,
        wrench press-wrench {
            ref-point:  <ft_tree.wrist_ft_body.wrist_ft_site>,
            as-seen-by: <kinova.base_link.base_link_origin>,
            ft-sensor:  <wrist_ft>
        }"""
COMMANDED = """,
        wrench press-wrench {
            of:         <gripper.g_base.g_pinch>,
            ref-point:  <gripper.g_base.g_pinch>,
            as-seen-by: <kinova.base_link.base_link_origin>
        }"""


def _press_source(base_source: str, wrench: str) -> str:
    """The base model with `wrench` declared and a feed-forward force command assigning it."""
    for old, new in (
        (TWIST_ANCHOR, TWIST_ANCHOR + wrench),
        (
            SPEC_ANCHOR,
            SPEC_ANCHOR + ",\n        force press-force = -5.0 N"
            ",\n        force satisfied-band-force = 0.5 N",
        ),
        (
            WHILE_ANCHOR,
            WHILE_ANCHOR + ",\n        press-down: keeping <shared.world.press-wrench>.force.z"
            " equal to <shared.spec.press-force> within <shared.spec.satisfied-band-force>",
        ),
        (
            CTRL_ANCHOR,
            CTRL_ANCHOR + ",\n        feed-forward ctrl-press-down"
            " { constraint: <home.press-down> } as force apply at <gripper.g_base>",
        ),
    ):
        assert old in base_source, old
        base_source = base_source.replace(old, new, 1)
    return base_source


def test_a_sensor_observed_wrench_cannot_be_commanded(parse_source, base_source):
    with pytest.raises(TextXSemanticError, match="which 'wrist_ft' measures"):
        parse_source(_press_source(base_source, MEASURED))


def test_a_wrench_without_a_sensor_can_be_commanded(parse_source, base_source):
    parse_source(_press_source(base_source, COMMANDED))
