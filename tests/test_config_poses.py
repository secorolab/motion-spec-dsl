# SPDX-License-Identifier: MPL-2.0
"""A pose the deployment states: the graph names the resource it is read from, and carries the
coordinate subobjects a constraint compares against -- it has no values, but it is still a pose."""

from __future__ import annotations

import pytest
from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from rdf_utils.constraints import ConstraintViolation
from motion_spec_dsl.rdf_parser.vocab import EXEC

_POSE = """        linear-velocity zero-linvel = 0.0 m/s,
        pose look-at-table = [config.poses.table] for <shared.world.pose-ee-base>
    }"""
_ANCHOR = """        linear-velocity zero-linvel = 0.0 m/s
    }"""
_CONSTRAINT = (
    "hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<spec.home-pose>.position within <shared.spec.satisfied-band>"
)
_USES_CONFIG_POSE = (
    "hold-position: keeping <shared.world.pose-ee-base>.position equal to "
    "<shared.spec.look-at-table>.position within <shared.spec.satisfied-band>"
)


def _graph(base_source: str, parse_source, *, declare_config: bool = True):
    source = base_source.replace(_ANCHOR, _POSE, 1).replace(_CONSTRAINT, _USES_CONFIG_POSE, 1)
    if declare_config:
        source = source.replace(
            "    timestep:   1.0 ms", '    config:     "robot.toml"\n    timestep:   1.0 ms', 1
        )
    return MotionSpecDatasetBuilder(parse_source(source)).build()[0].default_graph


def test_the_pose_names_the_resource_its_numbers_come_from(base_source, parse_source) -> None:
    graph = _graph(base_source, parse_source)
    pose = next(s for s in graph.subjects() if str(s).endswith("/spec/look-at-table"))
    resource = graph.value(pose, EXEC["has-resource"])

    assert resource is not None
    assert (resource, EXEC.path, None) in graph


def test_no_coordinate_carries_a_value(base_source, parse_source) -> None:
    """The numbers live in the config, so the graph states none of them."""
    graph = _graph(base_source, parse_source)
    valued = {
        str(subject) for subject, predicate, _o in graph if str(predicate).endswith("qudt/value")
    }

    assert not [uri for uri in valued if "look-at-table" in uri]


def test_a_constraint_compares_against_the_poses_own_coordinate(base_source, parse_source) -> None:
    """Not against a node nothing emits: a config pose owns its position/orientation subobjects
    the way a literal pose does, and a constraint must land on them."""
    graph = _graph(base_source, parse_source)
    subjects = {str(s) for s in graph.subjects()}
    pose = next(uri for uri in subjects if uri.endswith("/spec/look-at-table"))
    references = [
        str(o)
        for _s, predicate, o in graph
        if str(predicate).endswith("constraint#reference-value") and "look-at-table" in str(o)
    ]

    assert references == [f"{pose}.position-position-rel"]


def test_the_frames_come_from_the_quantity_it_is_stated_for(base_source, parse_source) -> None:
    """It contributes no number, only the frames the target is expressed in -- so the config pose
    stands in exactly the geometry of the pose it is compared against, without restating it."""
    graph = _graph(base_source, parse_source)
    frames = {
        suffix: {
            (str(predicate), str(o))
            for predicate, o in graph.predicate_objects(subject)
            if str(predicate).endswith(("#of", "#with-respect-to", "#as-seen-by"))
        }
        for suffix in ("look-at-table-pose-rel", "pose-ee-base-pose-rel")
        for subject in graph.subjects()
        if str(subject).endswith(suffix)
    }

    assert frames["look-at-table-pose-rel"] == frames["pose-ee-base-pose-rel"]
    assert frames["pose-ee-base-pose-rel"]


def test_a_pose_read_from_a_config_no_exec_context_declares_is_rejected(
    base_source, parse_source
) -> None:
    with pytest.raises(ConstraintViolation, match="declares no"):
        _graph(base_source, parse_source, declare_config=False)
