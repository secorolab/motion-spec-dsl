# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Contract for the mobile-platform solver quantity x operation 2x2 emitted by the DSL.

No maintained model authors a mobile-platform solver, so this fixture is the only
thing exercising the four algorithm-to-class mappings.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from rdflib import Dataset, Graph, URIRef
from rdflib.namespace import RDF

from motion_spec.rdf_parser.vocab import SLV, SLV_EXT
from motion_spec_dsl.registration import _gen_graph, motion_spec_metamodel

FIXTURE = Path(__file__).parent / "fixtures" / "mixed_solvers.robmot"

# Keyed by the authored configuration, which is unique per solver in the fixture.
EXPECTED = {
    "hddc2b_example_vel": (SLV.VelocityCompositionSolver, SLV.velocity),
    "hddc2b_example_vel_dist": (SLV_EXT.VelocityDistributionSolver, SLV.velocity),
    "hddc2b_example_frc_sc1": (SLV.ForceDistributionSolver, SLV.force),
    "hddc2b_example_frc": (SLV_EXT.ForceCompositionSolver, SLV.force),
}


@pytest.fixture(scope="module")
def solvers_by_configuration(tmp_path_factory: pytest.TempPathFactory) -> dict[str, URIRef]:
    tmp_path = tmp_path_factory.mktemp("mixed_solvers")
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv("METAMODELS_PATH", "/home/batsy/work/ms/src/metamodels")
        metamodel = motion_spec_metamodel()
        model = metamodel.model_from_file(FIXTURE)
        _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)

    dataset = Dataset()
    dataset.parse(str(tmp_path / "mixed_solvers.ld.json"), format="json-ld")
    graph = Graph()
    for quad in dataset.quads((None, None, None, None)):
        graph.add(quad[:3])
    return graph, {
        str(configuration): solver
        for solver, configuration in graph.subject_objects(SLV.configuration)
    }


@pytest.mark.parametrize("configuration", sorted(EXPECTED))
def test_algorithm_selects_its_class_and_predicate(solvers_by_configuration, configuration):
    """Direction picks the class; quantity picks the predicate. A velocity algorithm must
    never emit slv:force, and vice versa."""
    graph, by_configuration = solvers_by_configuration
    expected_class, expected_predicate = EXPECTED[configuration]
    other_predicate = SLV.force if expected_predicate == SLV.velocity else SLV.velocity

    solver = by_configuration[configuration]
    assert (solver, RDF.type, expected_class) in graph
    assert len(list(graph.objects(solver, expected_predicate))) == 1
    assert not list(graph.objects(solver, other_predicate))
