# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Baseline for audit finding B (plans 001-003): the ACHD/RNE solver structure the
DSL emits today. Plan 003 inverts these assertions to 0 once the derivation moves to
motion-spec. Loads the JSON-LD directly — never import motion_spec.ir_gen here."""

from __future__ import annotations

from pathlib import Path

import pytest
from rdflib import Dataset, Graph
from rdflib.namespace import RDF

from motion_spec.namespace import CSTR_HDL, GEOM_OP_EXT, SLV
from motion_spec_dsl.registration import _gen_graph, motion_spec_metamodel

MODELS = Path(__file__).parents[1] / "models"

# Recorded at motion-spec-dsl 4fa96df against models/pick_place_single.robmot.
BASELINE = {
    "AccelerationConstraint": 60,
    "AccelerationConstraintSpecification": 10,
    "control-signal": 74,
    "PoseDiffEvaluator": 12,
    "acceleration-energy": 60,
}


@pytest.fixture
def constraint_graph(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Graph:
    monkeypatch.setenv("METAMODELS_PATH", "/home/batsy/work/ms/src/metamodels")
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_file(MODELS / "pick_place_single.robmot")
    _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)
    dataset = Dataset()
    dataset.parse(str(tmp_path / "pick_place_single.jsonld"), format="json-ld")
    graph = Graph()
    for quad in dataset.quads((None, None, None, None)):
        graph.add(quad[:3])
    return graph


def _counts(graph: Graph) -> dict[str, int]:
    return {
        "AccelerationConstraint": len(set(graph.subjects(RDF.type, SLV.AccelerationConstraint))),
        "AccelerationConstraintSpecification": len(
            set(graph.subjects(RDF.type, SLV.AccelerationConstraintSpecification))
        ),
        "control-signal": len(list(graph.triples((None, CSTR_HDL["control-signal"], None)))),
        "PoseDiffEvaluator": len(set(graph.subjects(RDF.type, GEOM_OP_EXT["PoseDiffEvaluator"]))),
        "acceleration-energy": len(list(graph.triples((None, SLV["acceleration-energy"], None)))),
    }


def test_dsl_still_emits_the_achd_solver_structure(constraint_graph: Graph) -> None:
    assert _counts(constraint_graph) == BASELINE
