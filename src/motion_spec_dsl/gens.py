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

from motion_spec_dsl.classes.motion_spec import Model
from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder

log = logging.getLogger(__name__)


_ANSI = re.compile("\x1b\\[[0-9;]*m")

# What the caller calls a level. Python's own "warning" is a word wider than the column.
_LEVEL_NAMES = {"WARNING": "warn", "CRITICAL": "error"}


def _level_namer(labels: dict[str, str]):
    """Give each record the caller's word for its level, as the caller writes it."""

    def name(record: logging.LogRecord) -> bool:
        level = _LEVEL_NAMES.get(record.levelname, record.levelname.lower())
        record.levelname = labels.get(level, level)
        return True

    return name


def _stamp_lines() -> None:
    """Log in the caller's format when it named one, so a toolchain reads as a single stream.

    textx configures the root logger with the bare message, and the generator runs in its own
    process, so the line has to be formatted here or not at all.
    """
    pattern = os.environ.get("MOTION_SPEC_LOG_FORMAT")
    if not pattern or any(isinstance(h, logging.StreamHandler) for h in log.handlers):
        return
    handler = logging.StreamHandler()
    labels = json.loads(os.environ.get("MOTION_SPEC_LOG_LEVELS") or "{}")
    if not handler.stream.isatty():
        pattern = _ANSI.sub("", pattern)
        labels = {level: _ANSI.sub("", label) for level, label in labels.items()}
    handler.setFormatter(
        logging.Formatter(pattern, datefmt=os.environ.get("MOTION_SPEC_LOG_DATEFMT") or None)
    )
    handler.addFilter(_level_namer(labels))
    log.addHandler(handler)
    log.setLevel(logging.INFO)
    log.propagate = False


DSLPROV = Namespace("https://secorolab.github.io/motion-spec-dsl/provenance/")
# Agents and file entities are shared concepts: one IRI each, in the space motion-spec's
# prov_uri already mints, so this document, coord-dsl's and motion-spec's describe one node per
# tool and per file rather than three parallel ones. Only this document's own activity
# instances stay under dslprov.
MSPROV = Namespace("https://secorolab.github.io/motion-spec/provenance/")

# agent.json redefines the term "Agent" as agn:Agent, so it goes before prov.json, whose
# "Agent" must stay prov:Agent for the shapes to see one.
PROVENANCE_CONTEXT = [
    "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/agent.json",
    URL_MM_PROV_JSON,
    URL_MM_PROV_EXT_JSON,
    {"dslprov": str(DSLPROV), "msprov": str(MSPROV)},
]
PROVENANCE_SHAPES = (URL_MM_PROV_SHACL, URL_MM_PROV_EXT_SHACL)
# The manifest names each imported document under this base; its IRI also names the graph it is.
IMPORT_BASE = "https://secorolab.github.io/"


def _build_manifest(imported_files: list[str]) -> dict[str, Any]:
    """Build the app manifest (namespace prefixes and imported graph files) for a dataset."""
    local_constraint_paths = {
        "https://secorolab.github.io/metamodels/algorithm-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/behaviour/event_loop.shacl.ttl",
        "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/environment.shacl.ttl",
        "https://secorolab.github.io/metamodels/robot/sensors.shacl.ttl",
        "https://secorolab.github.io/metamodels/geometry/spatial-operators-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/newtonian-rigid-body-dynamics/operators-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/geometry/spatial-relations-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/geometry/structural-entities-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/execution-context.shacl.ttl",
        "https://secorolab.github.io/metamodels/task/constraint-handler-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/task/constraint-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/task/map-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/task/solver-specification-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/geometry/path.shacl.ttl",
        "https://secorolab.github.io/metamodels/qudt.shacl.ttl",
        "https://secorolab.github.io/metamodels/time.shacl.ttl",
    }
    # The manifest's iri-map only declares where *this model's* imported graphs live
    # (relative to the manifest). Metamodel/ontology prefixes are deliberately NOT
    # baked in here: they resolve through rdf-utils' IriToFileResolver against the
    # local metamodels checkout, or the rdf-utils cache when there is none
    # (rdf_parser.manifest.metamodel_url_map), so the
    # generated manifest stays free of machine-specific absolute paths and is
    # portable into run archives.
    iri_map = {
        IMPORT_BASE: {"path": "."},
    }

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
                "constraints": list(local_constraint_paths),
                "iri-map": iri_map,
            }
        ],
    }


def _record_authoring(graph: Graph, source: Path) -> URIRef:
    """Who wrote this source file: a Specification generating it, ended when the file last changed."""
    entity = _source_entity(source)
    activity = MSPROV[f"activity/specification/{_slug(source.name)}"]
    owner = _file_owner(source)
    # The file's owner is an OS account, which is all a source file can say of its author.
    person = MSPROV[f"agent/os-account/{_slug(owner)}"]
    add_agent(graph, person, (PROV.Person,), name=owner)
    add_file_entity(graph, entity, location=str(source), generated_by=activity)
    graph.add((activity, RDF.type, PROV.Activity))
    graph.add((activity, RDF.type, URI_PROV_EXT_TYPE_SPECIFICATION))
    graph.add((activity, PROV.wasAssociatedWith, person))
    graph.add((activity, PROV.endedAtTime, Literal(_mtime(source))))
    return entity


def _transformation(
    graph: Graph,
    activity: URIRef,
    sources: list[Path],
    targets: list[str],
    output_dir: Path,
    agent: URIRef,
    span: tuple[datetime, datetime],
) -> None:
    """One generation step: the files it read, the files it wrote, and when it ran."""
    target_ids = []
    for name in targets:
        path = (output_dir / name).resolve()
        target_id = _generated_entity(path)
        add_file_entity(
            graph, target_id, location=str(path), generated_by=activity, generated_at=_mtime(path)
        )
        target_ids.append(target_id)
    load_transformation_prov(
        graph, activity, [_source_entity(s) for s in sources], target_ids, agent, *span
    )


def _build_provenance_document(
    model: Model,
    artifact_names: list[str],
    output_dir: Path,
    span: tuple[datetime, datetime],
) -> Graph:
    """The DSL's own provenance: who authored each source and what motion-spec-dsl made of them."""
    stem = _slug(Path(model._tx_filename).stem)
    sources = _source_paths(model)
    graph = Graph()
    for source in sources:
        _record_authoring(graph, source)

    # One node per generator package, in the IRI space every generation document shares.
    agent = MSPROV["agent/motion_spec_dsl"]
    load_pkg_prov(graph, agent, *get_pkg_info("motion_spec_dsl"))
    _transformation(
        graph,
        DSLPROV[f"activity/jsonld_generation/{stem}"],
        sources,
        artifact_names,
        output_dir,
        agent,
        span,
    )
    return graph


def _slug(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_") or "item"


def _mtime(path: Path) -> datetime | None:
    """When the file last changed, or None while it is still being written."""
    if not path.exists():
        return None
    return datetime.fromtimestamp(path.stat().st_mtime, timezone.utc)


def _file_owner(path: Path) -> str:
    uid = path.stat().st_uid
    try:
        return pwd.getpwuid(uid).pw_name
    except KeyError:
        return str(uid)


def _source_entity(path: Path) -> URIRef:
    """The authored file's node. One IRI per file across every generation-time document, so
    coord-dsl's view of the same .fsm and this one's are the same node."""
    return MSPROV[f"entity/source/{_slug(Path(path).name)}"]


def _generated_entity(path: Path) -> URIRef:
    return MSPROV[f"entity/generated/{_slug(Path(path).name)}"]


def _validate_provenance(graph: Graph) -> None:
    from motion_spec_dsl.rdf_parser.manifest import install_metamodel_resolver

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
    conforms, _graph, report = pyshacl.validate(
        data_graph=graph, shacl_graph=shapes, inference="rdfs"
    )
    if not conforms:
        raise RuntimeError(f"the DSL's provenance failed PROV SHACL validation\n{report}")


def _source_paths(model: Model) -> list[Path]:
    paths: dict[Path, None] = {}

    def visit(item: Any) -> None:
        filename = getattr(item, "_tx_filename", None)
        if filename:
            paths[Path(filename).resolve()] = None
        for imp in getattr(item, "imports", []):
            for loaded in getattr(imp, "_tx_loaded_models", []):
                visit(loaded)

    visit(model)

    # A declared deployment config is an authored input like any grammar file: it is stated
    # relative to the model that names it, so record it here rather than leave every consumer
    # to guess where it lives.
    for spec in getattr(model, "specs", []) or []:
        config = getattr(spec, "config", None)
        if config:
            declared_in = Path(get_model(spec)._tx_filename).resolve()
            paths[(declared_in.parent / config).resolve()] = None
    return list(paths)


def _gen_graph(metamodel, model, output_path, overwrite, debug, **kwargs) -> None:
    """textx generator: write `model`'s JSON-LD, app manifest and provenance document."""
    del metamodel, overwrite, debug
    _stamp_lines()
    output_dir = Path(output_path) if output_path else Path(model._tx_filename).parent
    _dataset, provenance = generate(model, output_dir)
    path = output_dir / "provenance" / "dsl.ld.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    serialized = provenance.serialize(
        format="json-ld", context=PROVENANCE_CONTEXT, auto_compact=True, indent=2
    )
    document = json.loads(serialized.decode() if isinstance(serialized, bytes) else serialized)
    path.write_text(json.dumps(document, indent=2) + "\n")
    log.info("wrote %s", path)


def generate(model, output_dir: Path) -> tuple[Dataset, Graph]:
    """Write `model`'s JSON-LD and app manifest into OUTPUT_DIR, and return them as the dataset
    loading that manifest yields -- the manifest in the default graph, and the model's document as
    the graph its import IRI names -- with the provenance of what this generation read and wrote.
    """
    from motion_spec_dsl.rdf_parser.manifest import build_url_map, install_metamodel_resolver

    started = datetime.now(timezone.utc)
    builder = MotionSpecDatasetBuilder(model)
    dataset, context = builder.build()

    output_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(model._tx_filename).stem

    graph_path = output_dir / f"{stem}.ld.json"
    manifest_path = output_dir / f"{stem}-app.ld.json"
    serialized = dataset.default_graph.serialize(format="json-ld", indent=2, context=context)
    serialized = serialized.decode() if isinstance(serialized, bytes) else serialized
    graph_path.write_text(serialized)
    log.info("wrote %s", graph_path)

    manifest = json.dumps(_build_manifest([graph_path.name]), indent=2)
    manifest_path.write_text(manifest)
    log.info("wrote %s", manifest_path)

    provenance = _build_provenance_document(
        model,
        [graph_path.name, manifest_path.name],
        output_dir,
        (started, datetime.now(timezone.utc)),
    )
    _validate_provenance(provenance)

    merged = Dataset(default_union=True)
    merged.parse(data=manifest, format="json-ld")
    named = merged.graph(URIRef(IMPORT_BASE + graph_path.name))
    named += dataset.default_graph
    # After the provenance check, which installs the resolver without this model's iri-map.
    install_metamodel_resolver(build_url_map(merged, manifest_path.resolve()))

    return merged, provenance
