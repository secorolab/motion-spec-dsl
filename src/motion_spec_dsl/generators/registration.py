# SPDX-License-Identifier: MPL-2.0
"""textX registration and JSON-LD file generation for the motion-spec DSL."""

from __future__ import annotations

import json
from importlib.resources import files
from pathlib import Path
from typing import Any

from rdflib.graph import Dataset
from textx import GeneratorDesc, LanguageDesc, metamodel_from_file
from textx.scoping import providers as scoping_providers

from motion_spec_dsl.generators.classes import (
    BilateralConstraint,
    ConstraintAlias,
    ConstraintHandler,
    ConstraintReference,
    ConstraintSpecification,
    ConstraintRef,
    ContextRef,
    ControllerAlias,
    ControllerEntry,
    ControllerReference,
    ControllerRef,
    ControllerParams,
    EqualityConstraint,
    GeoPropPair,
    GeometricProps,
    GreaterThanConstraint,
    Import,
    LessThanConstraint,
    Model,
    MonitorEntry,
    MotionSpec,
    NamespaceDeclare,
    PostContextDecl,
    PreContextDecl,
    RobotAnchorRef,
    RobotBaseComponent,
    RobotChainComponent,
    RobotComponentRef,
    RobotManipulatorComponent,
    RobotRef,
    RobotSpec,
    ScalarQuantity,
    SolverAlias,
    SolverEntry,
    SolverReference,
    SolverRef,
    SpecContextDecl,
    ValueVariableAlias,
    ValueVariableReference,
    ValueVariable,
    VectorQuantity,
    View,
    WorldContextDecl,
    WorldQuantityAlias,
    WorldQuantityReference,
    WorldQuantity,
    WhenSection,
    WhileSection,
    UntilSection,
    _resolved_controller,
    _resolved_spec,
    _resolved_solver,
)
from motion_spec_dsl.generators.motion_spec_graph import (
    CONSTRAINT_PATH_BY_PREFIX,
    MotionSpecDatasetBuilder,
)
from motion_spec_dsl.generators.validation import motion_constraint_items, validate_model

GRAMMAR_PATH = str(files("motion_spec_dsl.metamodels").joinpath("motion_spec.tx"))
SUPPORTED_FORMATS = {"json-ld": "json", "ttl": "ttl", "xml": "xml"}

LANGUAGE_CLASSES = [
    Model,
    NamespaceDeclare,
    Import,
    RobotSpec,
    RobotBaseComponent,
    RobotChainComponent,
    RobotManipulatorComponent,
    RobotRef,
    RobotComponentRef,
    RobotAnchorRef,
    MotionSpec,
    ConstraintHandler,
    WorldContextDecl,
    PreContextDecl,
    SpecContextDecl,
    PostContextDecl,
    WorldQuantity,
    WorldQuantityAlias,
    WorldQuantityReference,
    GeometricProps,
    GeoPropPair,
    ValueVariable,
    ValueVariableAlias,
    ValueVariableReference,
    ScalarQuantity,
    VectorQuantity,
    ConstraintAlias,
    ConstraintReference,
    ConstraintSpecification,
    ConstraintRef,
    ContextRef,
    View,
    EqualityConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    BilateralConstraint,
    MonitorEntry,
    ControllerAlias,
    ControllerReference,
    ControllerEntry,
    ControllerRef,
    ControllerParams,
    SolverAlias,
    SolverReference,
    SolverEntry,
    SolverRef,
    WhenSection,
    WhileSection,
    UntilSection,
]


class MotionConstraintScopeProvider:
    """Resolve the constraint part of refs authored as motion.constraint."""

    def __call__(self, obj: ConstraintRef, attr, obj_ref):
        del attr
        motion = obj.motion
        if motion is None or not isinstance(motion, MotionSpec):
            return None

        for item in motion_constraint_items(motion):
            item_name = getattr(item, "name", None) or getattr(_resolved_spec(item), "name", None)
            if item_name == obj_ref.obj_name:
                return _resolved_spec(item)
        return None


class HandlerControllerScopeProvider:
    """Resolve controller refs against controllers declared in the target handler."""

    def __call__(self, obj: ControllerRef, attr, obj_ref):
        del attr
        handler = obj.handler
        if handler is None or not isinstance(handler, ConstraintHandler):
            return None
        for controller in getattr(handler, "controllers", []):
            if controller.name == obj_ref.obj_name:
                return _resolved_controller(controller)
        return None


class CrossHandlerSolverScopeProvider:
    """Resolve solver refs: cross-handler when handler is set, else local handler via parent chain."""

    def __call__(self, obj: SolverRef, attr, obj_ref):
        del attr
        handler = obj.handler
        if not isinstance(handler, ConstraintHandler):
            current = getattr(obj, "parent", None)
            while current is not None and not isinstance(current, ConstraintHandler):
                current = getattr(current, "parent", None)
            handler = current
        if not isinstance(handler, ConstraintHandler):
            return None
        for solver in getattr(handler, "solvers", []):
            solver_name = getattr(solver, "name", None) or getattr(_resolved_solver(solver), "name", None)
            if solver_name == obj_ref.obj_name:
                return _resolved_solver(solver)
        return None


def motion_spec_metamodel():
    metamodel = metamodel_from_file(GRAMMAR_PATH, autokwd=True, classes=LANGUAGE_CLASSES)
    metamodel.register_scope_providers({
        "*.*": scoping_providers.FQNImportURI(),
        "ConstraintRef.constraint": MotionConstraintScopeProvider(),
        "ControllerRef.controller": HandlerControllerScopeProvider(),
        "SolverRef.solver": CrossHandlerSolverScopeProvider(),
    })
    metamodel.register_model_processor(validate_model)
    return metamodel


motion_spec_lang = LanguageDesc(
    name="motion_spec_dsl",
    pattern="*.robmot",
    description="Motion specification DSL for guarded motions",
    metamodel=motion_spec_metamodel,
)


def _merged_context(context: Any) -> list[str | dict[str, str]]:
    context_urls: set[str] = set()
    local_context: dict[str, str] = {}
    if isinstance(context, list):
        for item in context:
            if isinstance(item, str):
                context_urls.add(item)
            elif isinstance(item, dict):
                local_context.update(item)
    elif isinstance(context, dict):
        local_context.update(context)
    return [*sorted(context_urls), local_context]


def _build_manifest(dataset: Dataset, imported_files: list[str]) -> dict[str, Any]:
    constraint_paths: set[str] = set()
    for prefix, _ in dataset.namespaces():
        path = CONSTRAINT_PATH_BY_PREFIX.get(prefix)
        if path:
            constraint_paths.add(path)

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
                "constraints": sorted(constraint_paths),
                "iri-map": {
                    "https://comp-rob2b.github.io/": {"path": "comp-rob2b/"},
                    "https://secorolab.github.io/": {"path": "models/"},
                },
            }
        ],
    }


def _gen_graph(metamodel, model, output_path, overwrite, debug, **kwargs) -> None:
    del metamodel, overwrite, debug

    output_format = kwargs.get("format", "json-ld")
    if output_format not in SUPPORTED_FORMATS:
        raise ValueError(f"Unsupported format '{output_format}', supported: {list(SUPPORTED_FORMATS)}")

    builder = MotionSpecDatasetBuilder(model)
    dataset, context = builder.build()

    output_dir = Path(output_path) if output_path else Path(model._tx_filename).parent
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(model._tx_filename).stem

    graph_path = output_dir / f"{stem}.{SUPPORTED_FORMATS[output_format]}"
    serialized = dataset.default_graph.serialize(
        format=output_format, indent=2, context=_merged_context(context)
    )
    graph_path.write_text(serialized.decode() if isinstance(serialized, bytes) else serialized)
    print(f"  wrote {graph_path}")

    manifest_path = output_dir / f"{stem}-app.json"
    manifest_path.write_text(json.dumps(_build_manifest(dataset, [graph_path.name]), indent=2))
    print(f"  wrote {manifest_path}")


motion_spec_gen = GeneratorDesc(
    language="motion_spec_dsl",
    target="jsonld",
    description="Generates JSON-LD files from a .rob_mot motion specification",
    generator=_gen_graph,
)
