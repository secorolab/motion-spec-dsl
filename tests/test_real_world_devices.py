# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A real-world execution context names the hardware behind each agent and sensor.

The scene stays deployment-neutral: the same `.scenex` runs simulated or on hardware, and only the
execution context differs. Addresses and credentials live in a config file the graph only points
at -- they change nothing the controller computes, so nothing puts them in the graph.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import rdflib

from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from rdflib.namespace import SDO

from motion_spec_dsl.rdf_parser.vocab import EXEC, SSN

SIM = 'platform:   simulation { name: "MuJoCo" }'

REAL = """platform:   real-world {
                    <agents.kinova_ft_2f85> realized by KinovaGen3-2F85,
                    <wrist_ft> realized by RobotiqFT300s
                }
    config:     "robot.toml\""""


def _graph(parse_mutated, platform: str):
    model = parse_mutated(SIM, platform)
    return MotionSpecDatasetBuilder(model).build()[0].default_graph


def test_a_real_world_context_deploys_one_system_per_element(parse_mutated) -> None:
    """Arranging equipment for a purpose is a deployment; each device is a system it owns."""
    g = _graph(parse_mutated, REAL)
    context = next(g.subjects(rdflib.RDF.type, EXEC.RealWorld))
    assert (context, rdflib.RDF.type, EXEC.Simulation) not in g
    assert (context, rdflib.RDF.type, SSN.Deployment) in g

    deployed = {
        str(g.value(device, SDO.model)): str(
            g.value(device, EXEC["realizes"])
        ).rsplit("/", 1)[-1]
        for device in g.objects(context, SSN.deployedSystem)
    }
    assert deployed == {"KinovaGen3-2F85": "kinova_ft_2f85", "RobotiqFT300s": "wrist_ft"}


def test_the_scene_element_carries_nothing_the_deployment_knows(parse_mutated) -> None:
    """The agent belongs to the scene, so the deployment refers to it and never asserts onto
    it -- otherwise two deployments of one scene would overwrite each other's hardware."""
    g = _graph(parse_mutated, REAL)
    element = next(g.objects(None, EXEC["realizes"]))

    assert not list(g.objects(element, SDO.model))
    assert not list(g.objects(element, SDO.name))
    assert not list(g.objects(element, SDO.identifier))


def test_the_config_is_referenced_by_path_and_never_inlined(parse_mutated) -> None:
    g = _graph(parse_mutated, REAL)
    context = next(g.subjects(rdflib.RDF.type, EXEC.RealWorld))
    config = next(g.objects(context, EXEC["has-resource"]))
    # A resource with a path, like any other model file -- not an inline blob. Addresses and
    # credentials are deployment facts and must not end up in an archived run graph.
    assert (config, rdflib.RDF.type, EXEC.ResourceWithPath) in g
    # Authored as "robot.toml", relative to the model -- the only place that knows the
    # directory -- so the path stated is the one a consumer can actually open.
    path = Path(str(next(g.objects(config, EXEC.path))))
    assert path == Path(__file__).parent / "fixtures" / "robot.toml"


def test_a_simulation_context_deploys_no_hardware(parse_mutated) -> None:
    """The simulator answers for every agent, so no device stands in for one."""
    g = _graph(parse_mutated, SIM)
    assert not list(g.subject_objects(EXEC["has-resource"]))
    assert not list(g.subjects(rdflib.RDF.type, EXEC.RealWorld))
    assert not list(g.subject_objects(EXEC["realizes"]))
    # Still a deployment, and it names what it runs: the scene's agents are the systems,
    # which is why nothing per-element has to be owned here.
    assert list(g.subjects(rdflib.RDF.type, SSN.Deployment))
    assert list(g.subject_objects(SSN.deployedSystem))


def test_an_unknown_device_is_rejected_while_parsing(parse_mutated) -> None:
    # A closed grammar keyword, so a typo fails before anything is emitted rather than silently
    # generating no driver.
    with pytest.raises(Exception):
        _graph(
            parse_mutated,
            'platform:   real-world { <agents.kinova_ft_2f85> realized by KinovaGen4 }\n'
            '    config:     "robot.toml"',
        )


def test_a_simulation_platform_may_carry_a_config(parse_mutated) -> None:
    """A config is a deployment fact, not a device-address file.

    Device addresses are the real-world subset of it; a simulated deployment still has facts
    of its own -- where the robot resets to, above all -- and they belong beside the model
    rather than baked into a template no run artifact can report.
    """
    _graph(parse_mutated, f'{SIM}\n    config:     "robot.toml"')


def test_binding_a_device_without_a_config_is_rejected(parse_mutated) -> None:
    with pytest.raises(ValueError, match="no 'config'"):
        _graph(
            parse_mutated,
            "platform:   real-world { <agents.kinova_ft_2f85> realized by KinovaGen3-2F85 }",
        )


def test_binding_one_element_twice_is_rejected(parse_mutated) -> None:
    with pytest.raises(ValueError, match="twice"):
        _graph(
            parse_mutated,
            "platform:   real-world {\n"
            "                    <agents.kinova_ft_2f85> realized by KinovaGen3-2F85,\n"
            "                    <agents.kinova_ft_2f85> realized by KinovaGen3\n"
            "                }\n"
            '    config:     "robot.toml"',
        )
