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
from importlib.metadata import PackageNotFoundError, distribution, version
from pathlib import Path
from typing import Any

import pyshacl
from rdf_utils.models.prov import (
    add_agent,
    add_file_entity,
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
from rdf_utils.naming import get_valid_var_name
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
PACKAGE_REPOSITORIES = {
    "motion_spec_dsl": "https://github.com/secorolab/motion-spec-dsl",
    "coord_dsl": "https://github.com/secorolab/coord-dsl",
    "scene_dsl": "https://github.com/secorolab/scene-dsl",
}


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
        "https://secorolab.github.io/": {"path": "models/"},
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
                "@context": {"@base": "https://secorolab.github.io/"},
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
                "constraints": sorted(local_constraint_paths),
                "iri-map": iri_map,
            }
        ],
    }


def _write_provenance_artifact(
    model: Model,
    artifact_names: list[str],
    output_dir: Path,
    path: Path,
    fsm_tool_names: list[str],
    scene_tool_names: list[str],
    spans: dict[str, tuple[datetime, datetime]],
) -> None:
    from motion_spec_dsl.rdf_parser.manifest import install_metamodel_resolver

    path.parent.mkdir(parents=True, exist_ok=True)
    install_metamodel_resolver()
    graph = _build_provenance_document(
        model, artifact_names, output_dir, fsm_tool_names, scene_tool_names, spans
    )
    serialized = graph.serialize(
        format="json-ld", context=PROVENANCE_CONTEXT, auto_compact=True, indent=2
    )
    document = json.loads(serialized.decode() if isinstance(serialized, bytes) else serialized)
    path.write_text(json.dumps({"schema_version": 1, **document}, indent=2) + "\n")
    _validate_provenance_artifact(path)
    log.info("wrote %s", path)


def _package_agent(graph: Graph, package: str) -> URIRef:
    """One node per generator package, in the IRI space every generation document shares."""
    agent = MSPROV[f"agent/{package}"]
    load_pkg_prov(
        graph,
        agent,
        name=package.replace("_", "-"),
        version=_package_version(package),
        commit=_package_commit(package),
        repository=PACKAGE_REPOSITORIES.get(package),
    )
    return agent


def _package_version(package: str) -> str | None:
    try:
        return version(package)
    except PackageNotFoundError:
        return None


def _package_commit(package: str) -> str | None:
    """The revision an installer recorded (PEP 610); absent for a plain source install."""
    try:
        direct_url = distribution(package).read_text("direct_url.json")
    except PackageNotFoundError:
        return None
    if not direct_url:
        return None
    return (json.loads(direct_url).get("vcs_info") or {}).get("commit_id")


def _record_authoring(graph: Graph, source: Path) -> URIRef:
    """Who wrote this source file: a Specification generating it, ended when the file last changed."""
    entity = _source_entity(source)
    activity = MSPROV[f"activity/specification/{_slug(source.name)}"]
    owner = _file_owner(source)
    person = MSPROV[f"agent/person/{_slug(owner)}"]
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
    fsm_tool_names: list[str],
    scene_tool_names: list[str],
    spans: dict[str, tuple[datetime, datetime]],
) -> Graph:
    """The DSL's own provenance: who authored each source, what motion-spec-dsl made of them,
    and what it delegated to coord-dsl (FSM) and scene-dsl (scenex) -- each step its own
    Transformation under its own package agent. coord-dsl also writes its own, richer
    provenance.ld.json beside its artifacts; a pointer entity here links to it rather than
    duplicating its activity detail.
    """
    stem = _slug(Path(model._tx_filename).stem)
    sources = _source_paths(model)
    graph = Graph()
    for source in sources:
        _record_authoring(graph, source)

    delegated = {*fsm_tool_names, *scene_tool_names}
    _transformation(
        graph,
        DSLPROV[f"activity/jsonld_generation/{stem}"],
        sources,
        [name for name in artifact_names if name not in delegated],
        output_dir,
        _package_agent(graph, "motion_spec_dsl"),
        spans["jsonld"],
    )
    if fsm_tool_names:
        _transformation(
            graph,
            DSLPROV[f"activity/fsm_generation/{stem}"],
            [s for s in sources if s.suffix == ".fsm"],
            fsm_tool_names,
            output_dir,
            _package_agent(graph, "coord_dsl"),
            spans["fsm"],
        )
    if scene_tool_names:
        _transformation(
            graph,
            DSLPROV[f"activity/scenex_generation/{stem}"],
            [s for s in sources if s.suffix == ".scenex"],
            scene_tool_names,
            output_dir,
            _package_agent(graph, "scene_dsl"),
            spans["scenex"],
        )

    coord_document = (output_dir / "provenance.ld.json").resolve()
    if fsm_tool_names and coord_document.exists():
        entity = _generated_entity(coord_document)
        add_file_entity(
            graph, entity, location=str(coord_document), generated_at=_mtime(coord_document)
        )
        graph.add((entity, PROV.wasAttributedTo, MSPROV["agent/coord_dsl"]))
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


def _validate_provenance_artifact(path: Path) -> None:
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
        data_graph=str(path),
        data_graph_format="json-ld",
        shacl_graph=shapes,
        inference="rdfs",
    )
    if not conforms:
        raise RuntimeError(f"{path}: PROV SHACL validation failed\n{report}")


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


def _fsm_named_graph_jsonld(graph, context, fsm_ref) -> str:
    """Serialize the FSM rdflib graph as a JSON-LD *named graph* (@id = the FSM IRI) so
    it stays a distinct graph when ir_gen unions it into the model dataset."""
    dataset = Dataset()
    named_graph = dataset.graph(URIRef(fsm_ref))
    for triple in graph:
        named_graph.add(triple)
    serialized = dataset.serialize(format="json-ld", context=context, auto_compact=True, indent=2)
    serialized = serialized.decode() if isinstance(serialized, bytes) else serialized
    return serialized


def _gen_scenex(model, output_dir: Path) -> tuple[list[str], list[str]]:
    """Generate JSON-LD and KDL headers for directly imported executable scenes."""
    import scene_dsl
    from jinja2 import Environment, FileSystemLoader
    from scene_dsl.kdl_tree import build_kdl_trees
    from scene_dsl.rdf.scenex import create_scenex_model_graph

    jsonld_names: list[str] = []
    kdl_names: list[str] = []
    for imp in getattr(model, "imports", []):
        if not imp.importURI.endswith(".scenex"):
            continue
        loaded = getattr(imp, "_tx_loaded_models", [])
        if not loaded:
            continue

        jsonld_name = f"{Path(imp.importURI).stem}.scenex.ld.json"
        jsonld_path = output_dir / jsonld_name
        scene_graph = create_scenex_model_graph(loaded[0])
        serialized = scene_graph.serialize(format="json-ld", auto_compact=True, indent=2)
        serialized = serialized.decode() if isinstance(serialized, bytes) else serialized
        jsonld_path.write_text(serialized)
        log.info("wrote %s", jsonld_path)
        jsonld_names.append(jsonld_name)

        kdl_name = f"{Path(imp.importURI).stem}.kdl.hpp"
        kdl_path = output_dir / kdl_name
        env = Environment(
            loader=FileSystemLoader(Path(scene_dsl.__file__).parent / "templates"),
            keep_trailing_newline=True,
        )
        kdl_path.write_text(
            env.get_template("kdl.hpp.jinja2").render(
                {
                    "data": {
                        "name": get_valid_var_name(Path(imp.importURI).stem),
                        "source": Path(imp.importURI).name,
                        "trees": build_kdl_trees(scene_graph, Path(loaded[0]._tx_filename).parent),
                    }
                }
            )
        )
        log.info("wrote %s", kdl_path)
        kdl_names.append(kdl_name)
    return jsonld_names, kdl_names


def _gen_fsm(model, output_dir: Path) -> tuple[list[str], list[str]]:
    """Generate FSM C++ header, a graphviz drawing, IR JSON, and a named-graph JSON-LD for
    any .fsm files imported by the model. Returns (jsonld_names, tool_artifact_names):
    jsonld_names are added to the app manifest so ir_gen derives the FSM wiring from the
    combined graph (the .hpp stays standalone); tool_artifact_names are the filenames
    coord-dsl itself generated (and recorded into its own provenance.ld.json beside
    motion-spec-dsl's), so the caller can attribute them to coord-dsl rather than to
    motion-spec-dsl in the DSL's own provenance document.
    """
    import shutil

    from coord_dsl.generators.dot import fsm_dot, write_dot
    from coord_dsl.generators.fsm import gen_cpp_header, gen_json
    from coord_dsl.generators.provenance import record
    from coord_dsl.rdf.fsm import get_fsm_graph

    jsonld_names: list[str] = []
    tool_artifact_names: list[str] = []
    for imp in getattr(model, "imports", []):
        if not imp.importURI.endswith(".fsm"):
            continue
        loaded = getattr(imp, "_tx_loaded_models", [])
        if not loaded:
            continue
        fsm_model = loaded[0]
        started = datetime.now(timezone.utc)
        graph, fsm_ref = get_fsm_graph(fsm_model)
        ir = gen_json(graph, fsm_ref)
        # Add the namespace URI so it also travels with the framed IR / header.
        ir["namespace_uri"] = fsm_model.fsm.ns.uri

        hpp_path = output_dir / f"{ir['name']}.hpp"
        hpp_path.write_text(gen_cpp_header(ir))
        log.info("wrote %s", hpp_path)
        record(fsm_model, "cpp", hpp_path, started)
        tool_artifact_names.append(hpp_path.name)

        started = datetime.now(timezone.utc)
        dot_source = fsm_dot(graph, fsm_ref)
        dot_path = output_dir / f"{ir['name']}.dot"
        dot_path.write_text(dot_source)
        log.info("wrote %s", dot_path)
        record(fsm_model, "dot", dot_path, started)
        tool_artifact_names.append(dot_path.name)
        if shutil.which("dot") is not None:
            started = datetime.now(timezone.utc)
            svg_path = output_dir / f"{ir['name']}.svg"
            write_dot(dot_source, svg_path, "svg")
            log.info("wrote %s", svg_path)
            record(fsm_model, "dot", svg_path, started)
            tool_artifact_names.append(svg_path.name)

        ir_path = output_dir / "fsm_ir.json"
        ir_path.write_text(json.dumps(ir, indent=2))
        log.info("wrote %s", ir_path)

        jsonld_name = f"{ir['name']}.ld.json"
        jsonld_path = output_dir / jsonld_name
        jsonld_path.write_text(_fsm_named_graph_jsonld(graph, None, fsm_ref))
        log.info("wrote %s", jsonld_path)
        jsonld_names.append(jsonld_name)
    return jsonld_names, tool_artifact_names


def _gen_graph(metamodel, model, output_path, overwrite, debug, **kwargs) -> None:
    """textx generator: build the dataset for `model` and write its JSON-LD, app manifest
    and FSM outputs.
    """
    del metamodel, overwrite, debug
    _stamp_lines()
    started = datetime.now(timezone.utc)
    builder = MotionSpecDatasetBuilder(model)
    dataset, context = builder.build()

    output_dir = Path(output_path) if output_path else Path(model._tx_filename).parent
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(model._tx_filename).stem

    graph_path = output_dir / f"{stem}.ld.json"
    manifest_path = output_dir / f"{stem}-app.ld.json"
    provenance_name = "provenance/dsl.ld.json"
    provenance_path = output_dir / provenance_name
    serialized = dataset.default_graph.serialize(format="json-ld", indent=2, context=context)
    serialized = serialized.decode() if isinstance(serialized, bytes) else serialized
    graph_path.write_text(serialized)
    log.info("wrote %s", graph_path)

    scene_started = datetime.now(timezone.utc)
    scene_jsonld_names, scene_kdl_names = _gen_scenex(model, output_dir)
    scene_ended = datetime.now(timezone.utc)

    # FSM graphs are emitted as separate named-graph JSON-LD files and imported by the
    # manifest, so ir_gen loads them as named graphs and derives the FSM wiring from the
    # combined graph (no fsm_ir.json read at codegen time).
    fsm_started = datetime.now(timezone.utc)
    fsm_jsonld_names, fsm_tool_names = _gen_fsm(model, output_dir)
    fsm_ended = datetime.now(timezone.utc)
    artifact_names = [
        graph_path.name,
        manifest_path.name,
        provenance_name,
        *fsm_tool_names,
        *(["fsm_ir.json"] if fsm_tool_names else []),
        *scene_jsonld_names,
        *scene_kdl_names,
        *fsm_jsonld_names,
    ]
    manifest_imports = [
        graph_path.name,
        *scene_jsonld_names,
        *fsm_jsonld_names,
    ]
    manifest_path.write_text(json.dumps(_build_manifest(manifest_imports), indent=2))
    log.info("wrote %s", manifest_path)

    _write_provenance_artifact(
        model,
        artifact_names,
        output_dir,
        provenance_path,
        fsm_tool_names,
        [*scene_jsonld_names, *scene_kdl_names],
        {
            "jsonld": (started, datetime.now(timezone.utc)),
            "fsm": (fsm_started, fsm_ended),
            "scenex": (scene_started, scene_ended),
        },
    )
