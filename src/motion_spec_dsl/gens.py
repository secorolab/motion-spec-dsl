# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Generate motion-spec RDF, manifests, and provenance artifacts."""

from __future__ import annotations

import json
import logging
import os
import pwd
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import pyshacl
from rdf_utils.models.prov import (
    add_agent,
    add_file_entity,
    get_pkg_info,
    load_pkg_prov,
    load_transformation_prov,
)
from rdf_utils.models.vocab import URI_PROV_EXT_TYPE_SPECIFICATION
from rdf_utils.namespace import (
    URL_MM_PROV_EXT_JSON,
    URL_MM_PROV_EXT_SHACL,
    URL_MM_PROV_JSON,
    URL_MM_PROV_SHACL,
)
from rdflib import Dataset, Graph, Literal, URIRef
from rdflib.namespace import PROV, RDF, Namespace
from textx import get_model
from textx.scoping import get_included_models

from motion_spec_dsl.classes.motion_spec import ExecutionContext, Model
from motion_spec_dsl.rdf.dataset import build_dataset
from motion_spec_dsl.rdf_parser.manifest import build_url_map, install_metamodel_resolver
from motion_spec_dsl.rdf_parser.vocab import SHAPES

log = logging.getLogger(__name__)

_ANSI = re.compile("\x1b\\[[0-9;]*m")
_UNSLUGGED = re.compile(r"[^A-Za-z0-9_.-]+")

# What the caller calls a level. Python's own "warning" is a word wider than the column.
_LEVEL_NAMES = {"WARNING": "warn", "CRITICAL": "error"}

DSLPROV = Namespace("https://secorolab.github.io/motion-spec-dsl/provenance/")
# Agents and files are one IRI each across coord-dsl's, motion-spec's and this document.
MSPROV = Namespace("https://secorolab.github.io/motion-spec/provenance/")

# agent.json redefines "Agent" as agn:Agent, so it precedes prov.json, whose "Agent" stays prov:Agent.
PROVENANCE_CONTEXT = [
    "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/agent.json",
    URL_MM_PROV_JSON,
    URL_MM_PROV_EXT_JSON,
    {"dslprov": str(DSLPROV), "msprov": str(MSPROV)},
]
PROVENANCE_SHAPES = (URL_MM_PROV_SHACL, URL_MM_PROV_EXT_SHACL)
# The manifest names each imported document under this base; its IRI also names the graph it is.
IMPORT_BASE = "https://secorolab.github.io/"


class LevelNameFilter(logging.Filter):
    """Gives each record the caller's word for its level, as the caller writes it."""

    def __init__(self, labels: dict[str, str]) -> None:
        super().__init__()
        self.labels = labels

    def filter(self, record: logging.LogRecord) -> bool:
        level = _LEVEL_NAMES.get(record.levelname, record.levelname.lower())
        record.levelname = self.labels.get(level, level)
        return True


def stamp_lines() -> None:
    """Log in the caller's format when it names one, so a toolchain reads as one stream."""
    # textx gives the root logger the bare message and runs the generator in its own process.
    pattern = os.environ.get("MOTION_SPEC_LOG_FORMAT")
    if not pattern or any(isinstance(h, logging.StreamHandler) for h in log.handlers):
        return
    handler = logging.StreamHandler()
    labels = json.loads(os.environ.get("MOTION_SPEC_LOG_LEVELS") or "{}")
    if not handler.stream.isatty():
        pattern = _ANSI.sub("", pattern)
        labels = {level: _ANSI.sub("", label) for level, label in labels.items()}
    handler.setFormatter(logging.Formatter(pattern, datefmt=os.environ.get("MOTION_SPEC_LOG_DATEFMT") or None))
    handler.addFilter(LevelNameFilter(labels))
    log.addHandler(handler)
    log.setLevel(logging.INFO)
    log.propagate = False


def file_mtime(path: Path) -> datetime | None:
    """When the file last changed, or None while it is still being written."""
    if not path.exists():
        return None
    return datetime.fromtimestamp(path.stat().st_mtime, timezone.utc)


def build_manifest(imported_files: list[str], graph: Graph) -> dict[str, Any]:
    """The app manifest: this model's imported graph files and the shapes they are checked against.

    The shapes are those of every vocabulary GRAPH uses, so a model is checked against what it
    states and nothing else.
    """
    terms = {term for triple in graph for term in triple if isinstance(term, URIRef)}
    shapes = {
        shape
        for namespace, files in SHAPES.items()
        if any(term in namespace for term in terms)
        for shape in files
    }
    # Metamodel prefixes resolve through rdf-utils, so the manifest carries no machine paths.
    return {
        "license": "https://github.com/aws/mit-0",
        "@context": {
            "@version": 1.1,
            "xsd": "http://www.w3.org/2001/XMLSchema#",
            "app": "https://comp-rob2b.github.io/metamodels/application/",
            "import": {
                "@id": "app:import",
                "@type": "@id",
                "@context": {"@base": IMPORT_BASE},
            },
            "constraints": {
                "@id": "app:constraints",
                "@type": "@id",
                "@container": "@set",
                "@context": {"@base": "https://comp-rob2b.github.io/metamodels/"},
            },
            "iri-map": {"@id": "app:iri-map", "@container": "@id"},
            "path": {"@id": "app:path", "@type": "xsd:string"},
        },
        "@id": "https://secorolab.github.io/models/generated/",
        "@graph": [
            {
                "import": imported_files,
                "constraints": list(shapes),
                "iri-map": {IMPORT_BASE: {"path": "."}},
            }
        ],
    }


def build_provenance_document(
    model: Model, artifact_names: list[str], output_dir: Path, span: tuple[datetime, datetime]
) -> Graph:
    """The DSL's own provenance: who authored each source and what motion-spec-dsl made of them."""
    paths = {Path(included._tx_filename).resolve(): None for included in get_included_models(model)}
    # A deployment config is an authored input, stated relative to the model naming it.
    for spec in model.specs:
        if isinstance(spec, ExecutionContext) and spec.config:
            paths[(Path(get_model(spec)._tx_filename).resolve().parent / spec.config).resolve()] = None
    graph = Graph()
    source_ids = []
    for source in paths:
        source_id = MSPROV[f"entity/source/{_UNSLUGGED.sub('_', source.name).strip('_') or 'item'}"]
        source_ids.append(source_id)
        activity = MSPROV[f"activity/specification/{_UNSLUGGED.sub('_', source.name).strip('_') or 'item'}"]
        uid = source.stat().st_uid
        try:
            owner = pwd.getpwuid(uid).pw_name
        except KeyError:
            owner = str(uid)
        # The file's OS account is all a source file can say of its author.
        person = MSPROV[f"agent/os-account/{_UNSLUGGED.sub('_', owner).strip('_') or 'item'}"]
        add_agent(graph, person, (PROV.Person,), name=owner)
        add_file_entity(graph, source_id, location=str(source), generated_by=activity)
        graph.add((activity, RDF.type, PROV.Activity))
        graph.add((activity, RDF.type, URI_PROV_EXT_TYPE_SPECIFICATION))
        graph.add((activity, PROV.wasAssociatedWith, person))
        graph.add((activity, PROV.endedAtTime, Literal(file_mtime(source))))
    agent = MSPROV["agent/motion_spec_dsl"]
    load_pkg_prov(graph, agent, *get_pkg_info("motion_spec_dsl"))
    stem = _UNSLUGGED.sub("_", Path(model._tx_filename).stem).strip("_") or "item"
    activity = DSLPROV[f"activity/jsonld_generation/{stem}"]
    target_ids = []
    for name in artifact_names:
        path = (output_dir / name).resolve()
        target_id = MSPROV[f"entity/generated/{_UNSLUGGED.sub('_', path.name).strip('_') or 'item'}"]
        add_file_entity(graph, target_id, location=str(path), generated_by=activity, generated_at=file_mtime(path))
        target_ids.append(target_id)
    load_transformation_prov(graph, activity, source_ids, target_ids, agent, *span)
    return graph


def validate_provenance(graph: Graph) -> None:
    """Check the provenance document against the PROV shapes, from METAMODELS_PATH when set."""
    override = os.environ.get("METAMODELS_PATH")
    install_metamodel_resolver()
    shapes = Graph()
    for url in PROVENANCE_SHAPES:
        source: str | Path = url
        if override:
            source = Path(override) / url.rsplit("/", 1)[-1]
            if not source.is_file():
                raise RuntimeError(f"METAMODELS_PATH={override} does not contain {source.name}")
        shapes.parse(str(source), format="turtle")
    conforms, _graph, report = pyshacl.validate(data_graph=graph, shacl_graph=shapes, inference="rdfs")
    if not conforms:
        raise RuntimeError(f"the DSL's provenance failed PROV SHACL validation\n{report}")


def gen_graph(metamodel, model, output_path, overwrite, debug, **kwargs) -> None:
    """textx generator: write the model's JSON-LD, app manifest and provenance document."""
    del metamodel, overwrite, debug
    stamp_lines()
    output_dir = Path(output_path) if output_path else Path(model._tx_filename).parent
    _dataset, provenance = generate(model, output_dir)
    path = output_dir / "provenance" / "dsl.ld.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    serialized = provenance.serialize(format="json-ld", context=PROVENANCE_CONTEXT, auto_compact=True, indent=2)
    document = json.loads(serialized.decode() if isinstance(serialized, bytes) else serialized)
    path.write_text(json.dumps(document, indent=2) + "\n")
    log.info("wrote %s", path)


def generate(model, output_dir: Path) -> tuple[Dataset, Graph]:
    """Write the model's JSON-LD and manifest, and return the dataset loading them yields and its provenance."""
    started = datetime.now(timezone.utc)
    dataset, context = build_dataset(model)
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(model._tx_filename).stem
    graph_path = output_dir / f"{stem}.ld.json"
    manifest_path = output_dir / f"{stem}-app.ld.json"
    serialized = dataset.default_graph.serialize(format="json-ld", indent=2, context=context)
    graph_path.write_text(serialized.decode() if isinstance(serialized, bytes) else serialized)
    log.info("wrote %s", graph_path)
    manifest = json.dumps(build_manifest([graph_path.name], dataset.default_graph), indent=2)
    manifest_path.write_text(manifest)
    log.info("wrote %s", manifest_path)
    provenance = build_provenance_document(
        model, [graph_path.name, manifest_path.name], output_dir, (started, datetime.now(timezone.utc))
    )
    validate_provenance(provenance)
    # The manifest in the default graph, the model's document as the graph its import IRI names.
    merged = Dataset(default_union=True)
    merged.parse(data=manifest, format="json-ld")
    named = merged.graph(URIRef(IMPORT_BASE + graph_path.name))
    named += dataset.default_graph
    # After the provenance check, which installs the resolver without this model's iri-map.
    install_metamodel_resolver(build_url_map(merged, manifest_path.resolve()))
    return merged, provenance
