# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A real-world execution context names the hardware behind each agent and sensor.

The scene stays deployment-neutral: the same `.scenex` runs simulated or on hardware, and only the
execution context differs. Addresses and credentials live in a config file the graph only points
at -- they change nothing the controller computes, so nothing puts them in the graph.
"""

from __future__ import annotations

import pytest
import rdflib

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import EXEC

SIM = 'platform:   simulation { name: "MuJoCo" }'

REAL = """platform:   real-world {
                    name: KinovaGen3-2F85 maps to <agents.kinova_ft_2f85>,
                    name: RobotiqFT300s   maps to <wrist_ft>
                }
    config:     "robot.toml\""""


def _graph(parse_mutated, platform: str):
    model = parse_mutated(SIM, platform)
    return MotionSpecDatasetBuilder(model).build()[0].default_graph


def test_a_real_world_context_names_the_device_behind_each_element(parse_mutated) -> None:
    g = _graph(parse_mutated, REAL)
    context = next(g.subjects(rdflib.RDF.type, EXEC.RealWorld))
    assert (context, rdflib.RDF.type, EXEC.Simulation) not in g

    devices = {
        str(name): str(subject).rsplit("/", 1)[-1]
        for subject, name in g.subject_objects(EXEC["platform-name"])
    }
    assert devices == {"KinovaGen3-2F85": "kinova_ft_2f85", "RobotiqFT300s": "wrist_ft"}


def test_the_config_is_referenced_by_path_and_never_inlined(parse_mutated) -> None:
    g = _graph(parse_mutated, REAL)
    context = next(g.subjects(rdflib.RDF.type, EXEC.RealWorld))
    config = next(g.objects(context, EXEC["has-config"]))
    # A resource with a path, like any other model file -- not an inline blob. Addresses and
    # credentials are deployment facts and must not end up in an archived run graph.
    assert (config, rdflib.RDF.type, EXEC.ResourceWithPath) in g
    assert str(next(g.objects(config, EXEC.path))) == "robot.toml"


def test_a_simulation_context_carries_no_deployment(parse_mutated) -> None:
    g = _graph(parse_mutated, SIM)
    assert not list(g.subject_objects(EXEC["has-config"]))
    assert not list(g.subjects(rdflib.RDF.type, EXEC.RealWorld))


def test_an_unknown_device_is_rejected_while_parsing(parse_mutated) -> None:
    # A closed grammar keyword, so a typo fails before anything is emitted rather than silently
    # generating no driver.
    with pytest.raises(Exception):
        _graph(
            parse_mutated,
            'platform:   real-world { name: KinovaGen4 maps to <agents.kinova_ft_2f85> }\n'
            '    config:     "robot.toml"',
        )


def test_config_on_a_simulation_platform_is_rejected(parse_mutated) -> None:
    with pytest.raises(ValueError, match="config.*simulation"):
        _graph(parse_mutated, f'{SIM}\n    config:     "robot.toml"')


def test_binding_a_device_without_a_config_is_rejected(parse_mutated) -> None:
    with pytest.raises(ValueError, match="no 'config'"):
        _graph(
            parse_mutated,
            "platform:   real-world { name: KinovaGen3-2F85 maps to <agents.kinova_ft_2f85> }",
        )


def test_binding_one_element_twice_is_rejected(parse_mutated) -> None:
    with pytest.raises(ValueError, match="twice"):
        _graph(
            parse_mutated,
            "platform:   real-world {\n"
            "                    name: KinovaGen3-2F85 maps to <agents.kinova_ft_2f85>,\n"
            "                    name: KinovaGen3      maps to <agents.kinova_ft_2f85>\n"
            "                }\n"
            '    config:     "robot.toml"',
        )
