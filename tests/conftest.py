# SPDX-License-Identifier: MPL-2.0
"""Shared fixtures: the minimal valid model and a mutate-then-parse harness."""

from __future__ import annotations

import os
from pathlib import Path

import pytest

METAMODELS = Path(__file__).resolve().parents[2] / "metamodels"
BASE = Path(__file__).parent / "fixtures" / "base.robmot"


@pytest.fixture(autouse=True)
def _metamodels_path(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("METAMODELS_PATH", os.environ.get("METAMODELS_PATH", str(METAMODELS)))


@pytest.fixture(scope="session")
def base_source() -> str:
    return BASE.read_text()


@pytest.fixture
def parse_source():
    """Parse DSL source as if it sat next to the base fixture, so imports resolve."""
    from motion_spec_dsl.langs import motion_spec_metamodel

    def _parse(source: str):
        return motion_spec_metamodel().model_from_str(source, file_name=str(BASE))

    return _parse


def load_authored(authored, out: Path):
    """A parsed robmot loaded as motion-spec's generation loads it: `(Model, framed FSM or None)`."""
    import rdflib
    from coord_dsl.generators.fsm import gen_json
    from coord_dsl.rdf.fsm import get_fsm_graph
    from motion_spec.rdf_parser.model import Model
    from scene_dsl.rdf.scenex import create_scenex_model_graph

    from motion_spec_dsl.gens import generate

    dataset, _provenance = generate(authored, out)
    fsm = None
    for imported in authored.imports:
        for document in imported._tx_loaded_models:
            source = Path(document._tx_filename).resolve()
            if source.suffix == ".scenex":
                graph = create_scenex_model_graph(document)
            elif source.suffix == ".fsm":
                graph, fsm_ref = get_fsm_graph(document)
                fsm = {**gen_json(graph, fsm_ref), "namespace_uri": document.fsm.ns.uri}
            else:
                continue
            named = dataset.graph(rdflib.URIRef(source.as_uri()))
            named += graph
    model = Model(
        graph=dataset,
        app_path=(out / f"{Path(authored._tx_filename).stem}-app.ld.json").resolve(),
        namespaces=tuple(namespace.uri for namespace in authored.namespaces),
    )
    return model, fsm


@pytest.fixture
def parse_mutated(base_source: str, parse_source):
    """Parse the base model with `old` replaced by `new`; asserts the mutation applied."""

    def _mutate(old: str, new: str):
        assert old in base_source, f"mutation anchor not found in base.robmot: {old!r}"
        return parse_source(base_source.replace(old, new, 1))

    return _mutate
