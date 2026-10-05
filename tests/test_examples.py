# SPDX-License-Identifier: MPL-2.0
"""Every shipped example parses, generates, and conforms to the metamodels' SHACL shapes."""

from __future__ import annotations

from pathlib import Path

import pytest
import rdflib
from coord_dsl.rdf.fsm import get_fsm_graph
from scene_dsl.rdf.scenex import create_scenex_model_graph

from motion_spec_dsl.gens import generate
from motion_spec_dsl.langs import motion_spec_metamodel
from motion_spec_dsl.rdf_parser.check import validate_dataset

MODELS = Path(__file__).parents[1] / "src" / "motion_spec_dsl" / "models"


@pytest.mark.parametrize(
    "robmot", list(MODELS.glob("*/*.robmot")), ids=lambda robmot: robmot.parent.name
)
def test_an_example_generates_a_conforming_graph(robmot: Path, tmp_path: Path) -> None:
    authored = motion_spec_metamodel().model_from_file(str(robmot))
    dataset, _provenance = generate(authored, tmp_path)
    # The shapes span documents: a frame or an event is typed by the scene or FSM graph.
    for imported in authored.imports:
        for document in imported._tx_loaded_models:
            source = Path(document._tx_filename).resolve()
            if source.suffix == ".scenex":
                graph = create_scenex_model_graph(document)
            elif source.suffix == ".fsm":
                graph = get_fsm_graph(document)[0]
            else:
                continue
            named = dataset.graph(rdflib.URIRef(source.as_uri()))
            named += graph

    conforms, report = validate_dataset(dataset)
    assert conforms, report
