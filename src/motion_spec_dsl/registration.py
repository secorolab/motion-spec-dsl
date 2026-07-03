# SPDX-License-Identifier: MPL-2.0
"""textX registration and JSON-LD file generation for the motion-spec DSL."""

from __future__ import annotations

import json
import os
from importlib.resources import files
from pathlib import Path
from typing import Any

from rdflib.graph import Dataset
from textx import GeneratorDesc, LanguageDesc, metamodel_from_file
from textx.scoping import providers as scoping_providers

from motion_spec_dsl.domain import (
    BilateralConstraint,
    OutsideConstraint,
    ConstraintAlias,
    BareScalar,
    ConstraintHandler,
    ConstraintSpecification,
    ConstraintRef,
    ContextDeclReference,
    ContextRef,
    ContextSpec,
    ControllerAlias,
    ControllerEntry,
    ControllerParam,
    ControllerRef,
    ControllerParams,
    EnvironmentAssembly,
    EnvironmentAttachTargetEntry,
    EnvironmentAttachTargetRef,
    EnvironmentAsset,
    EnvironmentAttachmentActuatorEntry,
    EnvironmentAttachmentPrefixEntry,
    EnvironmentChainEntry,
    EnvironmentFrictionEntry,
    EnvironmentFreeEntry,
    EnvironmentMassEntry,
    EnvironmentOrientationEntry,
    EnvironmentPositionEntry,
    EnvironmentRobotRef,
    EnvironmentShapeEntry,
    EnvironmentSizeEntry,
    EnvironmentSpec,
    EnvironmentFtSensorEntry,
    EnvironmentTcpSiteEntry,
    EnvironmentToolBodyEntry,
    EnvironmentTrace,
    FrictionTerm,
    FrictionValue,
    EqualityConstraint,
    Figure8Spec,
    GeoPropPair,
    GeometricProps,
    GreaterThanConstraint,
    EventName,
    Import,
    LessThanConstraint,
    LerpSpec,
    Model,
    MonitorEntry,
    MotionSpec,
    NamespaceDeclare,
    OrientationTerm,
    OrientationValue,
    PostContextDecl,
    PoseValue,
    PreContextDecl,
    PositionTerm,
    PositionValue,
    ProfileSpec,
    ReferenceValue,
    AdmittanceSpec,
    RobotAnchorRef,
    RobotRef,
    ScalarQuantity,
    SolverAlias,
    SolverEntry,
    SolverRef,
    SnapshotValue,
    SpecContextDecl,
    ContextQuantityAlias,
    ContextQuantity,
    TrajectorySpec,
    TrajectoryValue,
    VectorQuantity,
    View,
    WorldContextDecl,
    WorldQuantityAlias,
    WorldQuantity,
    WhenSection,
    WhileSection,
    UntilSection,
    UntilMonitorRef,
    WhenMonitorRef,
    _resolved_controller,
    _resolved_spec,
    _resolved_solver,
)
from motion_spec_dsl.rdf import (
    CONSTRAINT_PATH_BY_PREFIX,
    MotionSpecDatasetBuilder,
)
from motion_spec_dsl.validation import motion_constraint_items, validate_model

GRAMMAR_PATH = str(files("motion_spec_dsl.metamodels").joinpath("motion_spec.tx"))

LANGUAGE_CLASSES = [
    Model,
    NamespaceDeclare,
    Import,
    EnvironmentSpec,
    ContextSpec,
    EnvironmentAsset,
    EnvironmentAssembly,
    EnvironmentAttachTargetRef,
    EnvironmentAttachTargetEntry,
    EnvironmentRobotRef,
    EnvironmentAttachmentPrefixEntry,
    EnvironmentAttachmentActuatorEntry,
    EnvironmentPositionEntry,
    EnvironmentOrientationEntry,
    EnvironmentFreeEntry,
    EnvironmentToolBodyEntry,
    EnvironmentTcpSiteEntry,
    EnvironmentFtSensorEntry,
    EnvironmentShapeEntry,
    EnvironmentSizeEntry,
    EnvironmentMassEntry,
    EnvironmentFrictionEntry,
    EnvironmentChainEntry,
    EnvironmentTrace,
    FrictionValue,
    FrictionTerm,
    PositionValue,
    PositionTerm,
    OrientationValue,
    OrientationTerm,
    PoseValue,
    RobotRef,
    RobotAnchorRef,
    MotionSpec,
    TrajectorySpec,
    TrajectoryValue,
    LerpSpec,
    ProfileSpec,
    AdmittanceSpec,
    Figure8Spec,
    ConstraintHandler,
    WorldContextDecl,
    PreContextDecl,
    SpecContextDecl,
    PostContextDecl,
    ContextDeclReference,
    WorldQuantity,
    WorldQuantityAlias,
    GeometricProps,
    GeoPropPair,
    ContextQuantity,
    ContextQuantityAlias,
    ScalarQuantity,
    VectorQuantity,
    ReferenceValue,
    SnapshotValue,
    ConstraintAlias,
    ConstraintSpecification,
    BareScalar,
    ConstraintRef,
    UntilMonitorRef,
    WhenMonitorRef,
    ContextRef,
    View,
    EqualityConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    BilateralConstraint,
    OutsideConstraint,
    MonitorEntry,
    EventName,
    ControllerAlias,
    ControllerEntry,
    ControllerRef,
    ControllerParams,
    ControllerParam,
    SolverAlias,
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
            solver_name = getattr(solver, "name", None) or getattr(
                _resolved_solver(solver), "name", None
            )
            if solver_name == obj_ref.obj_name:
                return _resolved_solver(solver)
        return None


def motion_spec_metamodel():
    metamodel = metamodel_from_file(GRAMMAR_PATH, autokwd=True, classes=LANGUAGE_CLASSES)
    metamodel.register_scope_providers(
        {
            "*.*": scoping_providers.FQNImportURI(),
            # FSM events are flat (not FQN-nested) — resolve by plain name across
            # all models loaded via importURI (including any imported .fsm).
            "EventName.event": scoping_providers.PlainNameImportURI(),
            "ConstraintRef.constraint": MotionConstraintScopeProvider(),
            "ControllerRef.controller": HandlerControllerScopeProvider(),
            "SolverRef.solver": CrossHandlerSolverScopeProvider(),
        }
    )
    metamodel.register_model_processor(validate_model)
    return metamodel


motion_spec_lang = LanguageDesc(
    name="motion_spec_dsl",
    pattern="*.robmot",
    description="Motion specification DSL for guarded motions",
    metamodel=motion_spec_metamodel,
)


def _canonicalize_jsonld(text: str) -> str:
    """Order the ``@graph`` array by ``@id`` so emission is stable across runs.

    rdflib's JSON-LD serializer lays out ``@graph`` nodes in hash-seeded set order,
    so the generated graph — and every artifact derived from it (IR, generated C++) —
    differs from run to run for byte-identical input. Every node here carries a unique
    IRI ``@id`` (no blank nodes), so sorting by ``@id`` yields a deterministic,
    semantically identical document.
    """
    doc = json.loads(text)
    graph = doc.get("@graph") if isinstance(doc, dict) else None
    if isinstance(graph, list):
        doc["@graph"] = sorted(graph, key=lambda node: node.get("@id", ""))
        return json.dumps(doc, indent=2)
    return text


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

    local_constraint_paths = {
        "https://secorolab.github.io/metamodels/behaviour/event_loop.shacl.ttl",
        "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/environment.shacl.ttl",
        "https://secorolab.github.io/metamodels/geometry/geometry.shacl.ttl",
        "https://secorolab.github.io/metamodels/geometry/polytope.shacl.ttl",
        "https://secorolab.github.io/metamodels/geometry/spatial-operators-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/runtime/runtime.shacl.ttl",
        "https://secorolab.github.io/metamodels/simulation/mujoco.shacl.ttl",
        "https://secorolab.github.io/metamodels/acceptance-criteria/bdd/execution-context.shacl.ttl",
        "https://secorolab.github.io/metamodels/snapshot.shacl.ttl",
        "https://secorolab.github.io/metamodels/task/constraint-handler.shacl.ttl",
        "https://secorolab.github.io/metamodels/task/constraint-handler-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/task/constraint-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/task/map-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/task/motion-specification-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/task/solver-specification-extension.shacl.ttl",
        "https://secorolab.github.io/metamodels/task/trajectory.shacl.ttl",
        "https://secorolab.github.io/metamodels/task/value-role.shacl.ttl",
    }
    try:
        metamodels_root = Path(os.environ["METAMODELS_PATH"])
    except KeyError:
        raise RuntimeError("METAMODELS_PATH environment variable is not set")
    comp_rob2b_metamodels = metamodels_root.parent / "comp-rob2b" / "metamodels"
    iri_map = {
        "https://secorolab.github.io/metamodels/": {"path": str(metamodels_root)},
        "https://secorolab.github.io/": {"path": "models/"},
    }
    if comp_rob2b_metamodels.exists():
        iri_map["https://comp-rob2b.github.io/metamodels/"] = {"path": str(comp_rob2b_metamodels)}

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
                "constraints": sorted(constraint_paths | local_constraint_paths),
                "iri-map": iri_map,
            }
        ],
    }


def _gen_fsm(model, output_dir: Path) -> None:
    """Generate FSM C++ header and IR JSON for any .fsm files imported by the model."""
    from coord_dsl.generators.fsm_graph import gen_cpp_header, gen_json, get_fsm_graph

    for imp in getattr(model, "imports", []):
        if not imp.importURI.endswith(".fsm"):
            continue
        loaded = getattr(imp, "_tx_loaded_models", [])
        if not loaded:
            continue
        fsm_model = loaded[0]
        graph, _, fsm_ref = get_fsm_graph(fsm_model)
        ir = gen_json(graph, fsm_ref)
        # Add the namespace URI so codegen can match event IRIs without re-parsing the .fsm.
        ir["namespace_uri"] = fsm_model.fsm.ns.uri

        hpp_path = output_dir / f"{ir['name']}.hpp"
        hpp_path.write_text(gen_cpp_header(ir))
        print(f"  wrote {hpp_path}")

        ir_path = output_dir / "fsm_ir.json"
        ir_path.write_text(json.dumps(ir, indent=2))
        print(f"  wrote {ir_path}")


def _gen_graph(metamodel, model, output_path, overwrite, debug, **kwargs) -> None:
    del metamodel, overwrite, debug, kwargs

    builder = MotionSpecDatasetBuilder(model)
    dataset, context = builder.build()

    output_dir = Path(output_path) if output_path else Path(model._tx_filename).parent
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(model._tx_filename).stem

    graph_path = output_dir / f"{stem}.json"
    serialized = dataset.default_graph.serialize(
        format="json-ld", indent=2, context=_merged_context(context)
    )
    serialized = serialized.decode() if isinstance(serialized, bytes) else serialized
    serialized = _canonicalize_jsonld(serialized)
    graph_path.write_text(serialized)
    print(f"  wrote {graph_path}")

    manifest_path = output_dir / f"{stem}-app.json"
    manifest_path.write_text(json.dumps(_build_manifest(dataset, [graph_path.name]), indent=2))
    print(f"  wrote {manifest_path}")

    _gen_fsm(model, output_dir)


motion_spec_gen = GeneratorDesc(
    language="motion_spec_dsl",
    target="jsonld",
    description="Generates JSON-LD files from a .rob_mot motion specification",
    generator=_gen_graph,
)
