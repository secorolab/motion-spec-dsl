# SPDX-License-Identifier: MPL-2.0
"""textX registration and JSON-LD file generation for the motion-spec DSL."""

from __future__ import annotations

import json
from importlib.resources import files
from pathlib import Path
from typing import Any

from rdflib.graph import Graph
from rdflib.namespace import Namespace
from textx import GeneratorDesc, LanguageDesc, metamodel_from_file
from textx.scoping import providers as scoping_providers

from motion_spec_dsl.generators.classes import (
    BilateralConstraint,
    ConstraintHandler,
    ConstraintSpecification,
    ControllerEntry,
    ControllerParams,
    EqualityConstraint,
    ForceSolverEntry,
    GeoPropPair,
    GeometricProps,
    GreaterThanConstraint,
    Import,
    LessThanConstraint,
    Model,
    MonitorEntry,
    MotionSpec,
    ConstraintRef,
    NamespaceDeclare,
    PostContextDecl,
    PreContextDecl,
    View,
    ScalarQuantity,
    SolverSpec,
    SpecContextDecl,
    ValueVariable,
    VectorQuantity,
    VelocitySolverEntry,
    WorldContextDecl,
    WorldQuantity,
    WhenSection,
    WhileSection,
    UntilSection
)
from motion_spec_dsl.generators.motion_spec_graph import (
    CONSTRAINT_PATH_BY_PREFIX,
    get_motion_spec_graphs,
)

GRAMMAR_PATH = str(files("motion_spec_dsl.metamodels").joinpath("motion_spec.tx"))
SUPPORTED_FORMATS = {"json-ld": "json", "ttl": "ttl", "xml": "xml"}

LANGUAGE_CLASSES = [
    Model,
    NamespaceDeclare,
    Import,
    MotionSpec,
    ConstraintHandler,
    WorldContextDecl,
    PreContextDecl,
    SpecContextDecl,
    PostContextDecl,
    WorldQuantity,
    GeometricProps,
    GeoPropPair,
    ValueVariable,
    ScalarQuantity,
    VectorQuantity,
    ConstraintSpecification,
    ConstraintRef,
    View,
    EqualityConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    BilateralConstraint,
    MonitorEntry,
    ControllerEntry,
    ControllerParams,
    SolverSpec,
    VelocitySolverEntry,
    ForceSolverEntry,
    WhenSection,
    WhileSection,
    UntilSection,
]


def _motion_constraints(spec: MotionSpec) -> list[ConstraintSpecification]:
    return [
        constraint
        for section in spec.sections
        for constraint in section.constraints
    ]


def _validate_motion_constraint_ref(
    ref: ConstraintRef,
    handler: ConstraintHandler,
    motion_specs: dict[str, MotionSpec],
    owner_name: str,
) -> None:
    motion_name = ref.motion.name
    if handler.motion and motion_name != handler.motion.name:
        raise ValueError(
            f"{owner_name} references motion '{motion_name}', but handler "
            f"'{handler.name}' is bound to motion '{handler.motion.name}'."
        )

    motion = motion_specs.get(motion_name)
    if motion is None:
        raise ValueError(f"{owner_name} references unknown motion '{motion_name}'.")

    if not any(constraint.name == ref.name for constraint in _motion_constraints(motion)):
        raise ValueError(
            f"{owner_name} references constraint '{motion_name}.{ref.name}', "
            f"but it is not defined in motion '{motion_name}'."
        )


def _validate_motion_constraint_refs(model, metamodel) -> None:
    del metamodel
    motion_specs = {
        spec.name: spec
        for spec in model.specs
        if isinstance(spec, MotionSpec)
    }
    for motion in motion_specs.values():
        seen: set[str] = set()
        duplicates: set[str] = set()
        for constraint in _motion_constraints(motion):
            if constraint.name in seen:
                duplicates.add(constraint.name)
            seen.add(constraint.name)
        if duplicates:
            names = ", ".join(sorted(duplicates))
            raise ValueError(
                f"Motion '{motion.name}' has duplicate constraint name(s): {names}. "
                "Constraint names must be unique across WHEN, WHILE, and UNTIL."
            )

    for handler in (spec for spec in model.specs if isinstance(spec, ConstraintHandler)):
        for monitor in handler.monitors:
            _validate_motion_constraint_ref(
                monitor.constraint,
                handler,
                motion_specs,
                f"Monitor '{monitor.name}'",
            )
        for controller in handler.controllers:
            _validate_motion_constraint_ref(
                controller.params.constraint,
                handler,
                motion_specs,
                f"Controller '{controller.name}'",
            )


def motion_spec_metamodel():
    metamodel = metamodel_from_file(GRAMMAR_PATH, autokwd=True, classes=LANGUAGE_CLASSES)
    metamodel.register_scope_providers({"*.*": scoping_providers.FQNImportURI()})
    metamodel.register_model_processor(_validate_motion_constraint_refs)
    return metamodel


motion_spec_lang = LanguageDesc(
    name="motion_spec_dsl",
    pattern="*.robmot",
    description="Motion specification DSL for guarded motions",
    metamodel=motion_spec_metamodel,
)


def _iter_entity_objects(obj, seen: set[int] | None = None):
    if seen is None:
        seen = set()
    obj_id = id(obj)
    if obj_id in seen:
        return
    seen.add(obj_id)

    if isinstance(obj, (str, bytes, int, float, bool, type(None), Namespace)):
        return
    if hasattr(obj, "name") and hasattr(obj, "uri"):
        yield obj
    if isinstance(obj, dict):
        children = obj.values()
    elif isinstance(obj, (list, tuple, set)):
        children = obj
    elif hasattr(obj, "__dict__"):
        children = (
            value
            for key, value in obj.__dict__.items()
            if not key.startswith("_tx_")
        )
    else:
        return

    for child in children:
        yield from _iter_entity_objects(child, seen)


def _print_entity_uris(model) -> None:
    for entity in _iter_entity_objects(model):
        print(f"{entity.__class__.__name__} {entity.name}: {entity.uri}")


def _graph_format(output_format: str) -> str:
    if output_format not in SUPPORTED_FORMATS:
        raise ValueError(
            f"Unsupported format '{output_format}', supported: {list(SUPPORTED_FORMATS)}"
        )
    return SUPPORTED_FORMATS[output_format]


def _serialize_graph(graph: Graph, output_format: str, context: Any) -> str:
    serialized = graph.serialize(format=output_format, indent=2, context=context)
    return serialized.decode() if isinstance(serialized, bytes) else serialized


def _merged_context(graphs) -> list[str | dict[str, str]]:
    context_urls: set[str] = set()
    local_context: dict[str, str] = {}

    for _, _, context in graphs:
        if isinstance(context, list):
            for item in context:
                if isinstance(item, str):
                    context_urls.add(item)
                elif isinstance(item, dict):
                    local_context.update(item)
        elif isinstance(context, dict):
            local_context.update(context)

    return [*sorted(context_urls), local_context]


def _constraint_paths(graphs) -> list[str]:
    constraint_paths: set[str] = set()

    for _, graph, _ in graphs:
        for prefix, _ in graph.namespaces():
            path = CONSTRAINT_PATH_BY_PREFIX.get(prefix)
            if path:
                constraint_paths.add(path)

    for _, _, context in graphs:
        if not isinstance(context, list):
            continue
        for item in context:
            if isinstance(item, str) and "metamodels/" in item:
                constraint_paths.add(item.split("metamodels/")[1].replace(".json", ".ttl"))

    return sorted(constraint_paths)


def _build_manifest(graphs, imported_files: list[str]) -> dict[str, Any]:
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
                "constraints": _constraint_paths(graphs),
                "iri-map": {
                    "https://comp-rob2b.github.io/": {"path": "comp-rob2b/"},
                    "https://secorolab.github.io/": {"path": "models/"},
                },
            }
        ],
    }


def _write_text(path: Path, content: str) -> None:
    path.write_text(content)
    print(f"  wrote {path}")


def _write_json(path: Path, content: dict[str, Any]) -> None:
    path.write_text(json.dumps(content, indent=2))
    print(f"  wrote {path}")


def _write_single_output(graphs, model, output_dir: Path, output_format: str) -> None:
    merged = Graph()
    for _, graph, _ in graphs:
        for triple in graph:
            merged.add(triple)
        for prefix, namespace in graph.namespaces():
            merged.bind(prefix, namespace)

    file_extension = _graph_format(output_format)
    stem = Path(model._tx_filename).stem
    graph_path = output_dir / f"{stem}.{file_extension}"
    _write_text(graph_path, _serialize_graph(merged, output_format, _merged_context(graphs)))

    manifest_path = output_dir / f"{stem}-app.json"
    _write_json(manifest_path, _build_manifest(graphs, [graph_path.name]))


def _write_split_output(graphs, model, output_dir: Path, output_format: str) -> None:
    file_extension = _graph_format(output_format)
    stem = Path(model._tx_filename).stem
    model_dir = output_dir / "models" / stem
    model_dir.mkdir(parents=True, exist_ok=True)

    imported_files: list[str] = []
    for filename, graph, context in graphs:
        output_name = filename if output_format == "json-ld" else filename.replace(".json", f".{file_extension}")
        output_path = model_dir / output_name
        _write_text(output_path, _serialize_graph(graph, output_format, context))
        imported_files.append(f"{stem}/{output_name}")

    manifest_path = output_dir / f"{stem}-app.json"
    _write_json(manifest_path, _build_manifest(graphs, imported_files))


def _gen_jsonld(metamodel, model, output_path, overwrite, debug, **kwargs) -> None:
    del metamodel, debug

    output_format = kwargs.get("format", "json-ld")
    _graph_format(output_format)

    # print all entities with their URIs
    _print_entity_uris(model)

    # graphs = get_motion_spec_graphs(model)
    # output_dir = Path(output_path) if output_path else Path(model._tx_filename).parent
    # output_dir.mkdir(parents=True, exist_ok=True)
    #
    # if kwargs.get("single", False):
    #     _write_single_output(graphs, model, output_dir, output_format)
    # else:
    #     _write_split_output(graphs, model, output_dir, output_format)


motion_spec_gen = GeneratorDesc(
    language="motion_spec_dsl",
    target="jsonld",
    description="Generates JSON-LD files from a .rob_mot motion specification",
    generator=_gen_jsonld,
)
