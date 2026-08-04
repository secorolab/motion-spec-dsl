# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Generate motion-spec RDF, manifests, and provenance artifacts."""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import pyshacl
from pyld import jsonld
from rdf_utils.resolver import IriToFileResolver, install_resolver
from rdflib import Dataset, Graph, Literal, URIRef
from rdflib.namespace import Namespace

from motion_spec_dsl.classes.motion_spec import Model
from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder

DSLPROV = Namespace("https://secorolab.github.io/motion-spec-dsl/provenance/")


_JSONLD_NATIVE_XSD_LOCALS = {"integer", "double"}


def _numeric_datatype_index(graph: Graph) -> dict[tuple[str, str], str]:
    """Map (subject IRI, predicate IRI) -> 'integer'|'double' for every literal in `graph`
    typed with one of the two XSD datatypes JSON-LD compaction silently turns into bare,
    untyped JSON numbers. Read directly off the source RDF literals -- never inferred from
    the compacted JSON value's Python shape, so a genuine xsd:double stays xsd:double even
    when its value happens to be a whole number, and booleans (a different datatype
    entirely) are never touched.
    """
    index: dict[tuple[str, str], str] = {}
    for s, p, o in graph:
        if not isinstance(o, Literal) or o.datatype is None:
            continue
        local = str(o.datatype).rsplit("#", 1)[-1]
        if local in _JSONLD_NATIVE_XSD_LOCALS:
            index[(str(s), str(p))] = local
    return index


def _expand_curie(value: Any, context: dict[str, str]) -> Any:
    """Expand a compacted `prefix:local` term back to its full IRI using `context`, the
    same prefix map the compaction step used -- so it inverts that exact compaction rather
    than guessing.
    """
    if not isinstance(value, str) or "://" in value:
        return value
    prefix, sep, rest = value.partition(":")
    if not sep:
        return value
    ns = context.get(prefix)
    return f"{ns}{rest}" if ns is not None else value


def _wrap_numeric(value: Any, local: str) -> Any:
    if isinstance(value, list):
        return [_wrap_numeric(v, local) for v in value]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return value
    coerced = int(value) if local == "integer" else float(value)
    return {"@value": coerced, "@type": f"xsd:{local}"}


def _rewrap_numeric_literals(
    value: Any, index: dict[tuple[str, str], str], context: dict[str, str]
) -> Any:
    """Recreate the explicit xsd:integer/xsd:double value objects JSON-LD compaction
    dropped, using `index` (sourced from the pre-serialization graph) to tell which
    (subject, predicate) pairs need it back.
    """
    if isinstance(value, list):
        return [_rewrap_numeric_literals(v, index, context) for v in value]
    if not isinstance(value, dict):
        return value
    subject = _expand_curie(value.get("@id"), context)
    result = {}
    for key, val in value.items():
        recursed = _rewrap_numeric_literals(val, index, context)
        if subject is not None and key not in ("@id", "@type"):
            local = index.get((subject, _expand_curie(key, context)))
            if local is not None:
                recursed = _wrap_numeric(recursed, local)
        result[key] = recursed
    return result


def _canonicalize_jsonld(text: str, graph: Graph | None = None) -> str:
    """Compact graph identifiers, restore explicit numeric datatypes, and order nodes so
    emission is stable across runs.

    rdflib's JSON-LD serializer lays out ``@graph`` nodes in hash-seeded set order
    and does not compact hierarchical identifiers against a parent prefix. Compacting
    identifiers from the emitted context and sorting by ``@id`` yields a deterministic,
    semantically identical document. Compaction also applies JSON-LD's native-type
    rule, silently turning every xsd:integer/xsd:double literal into a bare JSON number;
    passing `graph` (the pre-serialization source) restores those as explicit value
    objects. Booleans and every other datatype pass through untouched.
    """
    doc = json.loads(text)
    context = doc.get("@context") if isinstance(doc, dict) else None
    if context:
        doc = jsonld.compact(doc, context)
    graph_nodes = doc.get("@graph") if isinstance(doc, dict) else None
    if isinstance(graph_nodes, list):
        if graph is not None and context:
            index = _numeric_datatype_index(graph)
            if index:
                graph_nodes = [_rewrap_numeric_literals(n, index, context) for n in graph_nodes]
        graph_nodes = [_sort_lists(node) for node in graph_nodes]
        doc["@graph"] = sorted(graph_nodes, key=lambda node: node.get("@id", ""))
        return json.dumps(doc, indent=2)
    return text


def _sort_lists(value: Any) -> Any:
    """Order JSON-LD set arrays while preserving semantic RDF-list order.

    JSON-LD arrays (``@type``, multi-valued properties) are unordered sets, so
    sorting them by canonical form removes the last hash-seed dependence rdflib's
    serializer leaves behind after the top-level ``@graph`` sort. An ``@list``
    array is ordered RDF data and must never be sorted.
    """
    if isinstance(value, dict):
        return {
            key: (
                [_sort_lists(item) for item in child]
                if key == "@list" and isinstance(child, list)
                else _sort_lists(child)
            )
            for key, child in value.items()
        }
    if isinstance(value, list):
        return sorted(
            (_sort_lists(v) for v in value),
            key=lambda v: json.dumps(v, sort_keys=True),
        )
    return value


def _build_manifest(imported_files: list[str]) -> dict[str, Any]:
    """Build the app manifest (namespace prefixes and imported graph files) for a dataset."""
    local_constraint_paths = {
        "https://secorolab.github.io/metamodels/algorithm-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/behaviour/event_loop.shacl.ttl",
        "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/environment.shacl.ttl",
        "https://secorolab.github.io/metamodels/prov.shacl.ttl",
        "https://secorolab.github.io/metamodels/robot/sensors.shacl.ttl",
        "https://secorolab.github.io/metamodels/geometry/spatial-operators-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/geometry/spatial-relations-extension.shacl.ttl",
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
    # local metamodels checkout (rdf_parser.manifest.metamodel_url_map), so the
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


def _artifact_role(name: str) -> str:
    if name.endswith("-app.ld.json"):
        return "app_manifest"
    if name == "provenance/dsl.ld.json":
        return "provenance"
    if name == "fsm_ir.json":
        return "fsm_ir"
    if name.endswith("_fsm.hpp"):
        return "fsm_header"
    if name.endswith(("_fsm.dot", "_fsm.svg")):
        return "fsm_graphviz"
    if name.endswith(".scenex.ld.json"):
        return "scene_graph"
    if name.endswith(".kdl.hpp"):
        return "scene_kdl_header"
    return "generated_graph"


def _write_provenance_artifact(
    model: Model,
    artifact_names: list[str],
    output_dir: Path,
    path: Path,
    fsm_tool_names: list[str],
    scene_tool_names: list[str],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    document = _build_provenance_document(
        model, artifact_names, output_dir, fsm_tool_names, scene_tool_names
    )
    path.write_text(json.dumps(document, indent=2) + "\n")
    _validate_provenance_artifact(path)
    print(f"  wrote {path}")


def _tool_activity(
    graph: list[dict[str, Any]],
    tool_names: list[str],
    activity_id: str,
    agent_id: str,
    sources: list[Path],
    generated_at: str,
) -> None:
    """Append an Activity (and its Agent) attributing `tool_names` to an external tool,
    only if that tool actually produced anything for this generation pass.
    """
    if not tool_names:
        return
    graph.append(
        {
            "@id": activity_id,
            "@type": ["prov:Activity"],
            "used": [f"dslprov:entity/source/{_slug(s.name)}" for s in sources],
            "wasAssociatedWith": agent_id,
            "startedAtTime": generated_at,
            "endedAtTime": generated_at,
        }
    )
    graph.append({"@id": agent_id, "@type": ["prov:SoftwareAgent", "prov:Agent"]})


def _build_provenance_document(
    model: Model,
    artifact_names: list[str],
    output_dir: Path,
    fsm_tool_names: list[str],
    scene_tool_names: list[str],
) -> dict[str, Any]:
    """The DSL's own provenance: what motion-spec-dsl generated directly, plus what it
    delegated to coord-dsl (FSM) and scene-dsl (scenex) -- each attributed to its own
    agent rather than folded into motion_spec_dsl. coord-dsl also writes its own, richer
    provenance.ld.json beside its artifacts; a pointer entity here links to it rather than
    duplicating its activity detail (this document is SHACL-validated standalone, so any
    node referenced by @id must be described here too -- see `_tool_activity`).
    """
    stem = Path(model._tx_filename).stem
    sources = _source_paths(model)
    activity = f"dslprov:activity/jsonld_generation/{_slug(stem)}"
    fsm_activity = f"dslprov:activity/fsm_generation/{_slug(stem)}"
    scene_activity = f"dslprov:activity/scenex_generation/{_slug(stem)}"
    generated_at = datetime.now(timezone.utc).isoformat()

    graph = [
        {"@id": "dslprov:bundle/dsl-provenance", "@type": "prov:Bundle"},
        {
            "@id": activity,
            "@type": ["prov:Activity"],
            "used": [f"dslprov:entity/source/{_slug(s.name)}" for s in sources],
            "wasAssociatedWith": "dslprov:agent/motion_spec_dsl",
            "startedAtTime": generated_at,
            "endedAtTime": generated_at,
        },
    ]
    _tool_activity(
        graph,
        fsm_tool_names,
        fsm_activity,
        "dslprov:agent/coord_dsl",
        [s for s in sources if s.suffix == ".fsm"],
        generated_at,
    )
    _tool_activity(
        graph,
        scene_tool_names,
        scene_activity,
        "dslprov:agent/scene_dsl",
        [s for s in sources if s.suffix == ".scenex"],
        generated_at,
    )

    for source in sources:
        graph.append(
            {
                "@id": f"dslprov:entity/source/{_slug(source.name)}",
                "@type": ["prov:Entity"],
                "atLocation": source.resolve().as_uri(),
            }
        )
    for name in artifact_names:
        if name in fsm_tool_names:
            generated_by = fsm_activity
        elif name in scene_tool_names:
            generated_by = scene_activity
        else:
            generated_by = activity
        graph.append(
            {
                "@id": f"dslprov:entity/{_artifact_role(name)}/{_slug(name)}",
                "@type": ["prov:Entity"],
                "atLocation": (output_dir / name).resolve().as_uri(),
                "wasGeneratedBy": generated_by,
                "generatedAtTime": generated_at,
            }
        )
    if fsm_tool_names and (output_dir / "provenance.ld.json").exists():
        graph.append(
            {
                "@id": "dslprov:entity/provenance/coord_dsl",
                "@type": ["prov:Entity"],
                "atLocation": (output_dir / "provenance.ld.json").resolve().as_uri(),
                "wasAttributedTo": "dslprov:agent/coord_dsl",
            }
        )

    graph.append(
        {"@id": "dslprov:agent/motion_spec_dsl", "@type": ["prov:SoftwareAgent", "prov:Agent"]}
    )
    return {
        "schema_version": 1,
        "@context": [
            "https://secorolab.github.io/metamodels/prov.json",
            {"dslprov": str(DSLPROV)},
        ],
        "@graph": graph,
    }


def _slug(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "_.-" else "_" for ch in value).strip("_") or "item"


def _validate_provenance_artifact(path: Path) -> None:
    base = "https://secorolab.github.io/metamodels/"
    override = os.environ.get("METAMODELS_PATH")
    shape: str | Path = f"{base}prov.shacl.ttl"
    if override:
        metamodels = Path(override)
        if not (metamodels / "prov.shacl.ttl").is_file():
            raise RuntimeError(f"METAMODELS_PATH={override} does not contain prov.shacl.ttl")
        install_resolver(IriToFileResolver({base: str(metamodels)}, download=False))
        shape = metamodels / "prov.shacl.ttl"
    conforms, _graph, report = pyshacl.validate(
        data_graph=str(path),
        data_graph_format="json-ld",
        shacl_graph=str(shape),
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
    return _canonicalize_jsonld(serialized, graph)


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
        jsonld_path.write_text(_canonicalize_jsonld(serialized, scene_graph))
        print(f"  wrote {jsonld_path}")
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
                        "name": Path(imp.importURI).stem,
                        "source": Path(imp.importURI).name,
                        "trees": build_kdl_trees(
                            scene_graph, Path(loaded[0]._tx_filename).parent
                        ),
                    }
                }
            )
        )
        print(f"  wrote {kdl_path}")
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
        graph, fsm_ref = get_fsm_graph(fsm_model)
        ir = gen_json(graph, fsm_ref)
        # Add the namespace URI so it also travels with the framed IR / header.
        ir["namespace_uri"] = fsm_model.fsm.ns.uri

        hpp_path = output_dir / f"{ir['name']}.hpp"
        hpp_path.write_text(gen_cpp_header(ir))
        print(f"  wrote {hpp_path}")
        record(fsm_model, "cpp", hpp_path)
        tool_artifact_names.append(hpp_path.name)

        dot_source = fsm_dot(graph, fsm_ref)
        dot_path = output_dir / f"{ir['name']}.dot"
        dot_path.write_text(dot_source)
        print(f"  wrote {dot_path}")
        record(fsm_model, "dot", dot_path)
        tool_artifact_names.append(dot_path.name)
        if shutil.which("dot") is not None:
            svg_path = output_dir / f"{ir['name']}.svg"
            write_dot(dot_source, svg_path, "svg")
            print(f"  wrote {svg_path}")
            record(fsm_model, "dot", svg_path)
            tool_artifact_names.append(svg_path.name)

        ir_path = output_dir / "fsm_ir.json"
        ir_path.write_text(json.dumps(ir, indent=2))
        print(f"  wrote {ir_path}")

        jsonld_name = f"{ir['name']}.ld.json"
        jsonld_path = output_dir / jsonld_name
        jsonld_path.write_text(_fsm_named_graph_jsonld(graph, None, fsm_ref))
        print(f"  wrote {jsonld_path}")
        jsonld_names.append(jsonld_name)
    return jsonld_names, tool_artifact_names


def _gen_graph(metamodel, model, output_path, overwrite, debug, **kwargs) -> None:
    """textx generator: build the dataset for `model` and write its JSON-LD, app manifest
    and FSM outputs.
    """
    del metamodel, overwrite, debug, kwargs

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
    serialized = _canonicalize_jsonld(serialized, dataset.default_graph)
    graph_path.write_text(serialized)
    print(f"  wrote {graph_path}")

    scene_jsonld_names, scene_kdl_names = _gen_scenex(model, output_dir)

    # FSM graphs are emitted as separate named-graph JSON-LD files and imported by the
    # manifest, so ir_gen loads them as named graphs and derives the FSM wiring from the
    # combined graph (no fsm_ir.json read at codegen time).
    fsm_jsonld_names, fsm_tool_names = _gen_fsm(model, output_dir)
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
        provenance_name,
        *scene_jsonld_names,
        *fsm_jsonld_names,
    ]
    manifest_path.write_text(json.dumps(_build_manifest(manifest_imports), indent=2))
    print(f"  wrote {manifest_path}")

    _write_provenance_artifact(
        model,
        artifact_names,
        output_dir,
        provenance_path,
        fsm_tool_names,
        [*scene_jsonld_names, *scene_kdl_names],
    )
