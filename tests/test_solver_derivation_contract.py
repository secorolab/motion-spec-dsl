# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Contract for the ACHD/RNE solver structure and ownership emitted by the DSL."""

from __future__ import annotations

from pathlib import Path

import pytest
from rdflib import Dataset, Graph
from rdflib.namespace import PROV, RDF

from motion_spec.namespace import (
    ALGO_EXT,
    CSTR_HDL,
    CSTR_HDL_EXT,
    GEOM_OP_EXT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    SLV,
    SLV_EXT,
)
from motion_spec.ir_gen import generate_ir
from motion_spec_dsl.registration import _gen_graph, motion_spec_metamodel

MODELS = Path(__file__).parents[1] / "models"


@pytest.fixture(scope="module")
def pick_place_single_jsonld(tmp_path_factory: pytest.TempPathFactory) -> Path:
    tmp_path = tmp_path_factory.mktemp("pick_place_single")
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv("METAMODELS_PATH", "/home/batsy/work/ms/src/metamodels")
        metamodel = motion_spec_metamodel()
        model = metamodel.model_from_file(
            MODELS / "pick_place_single" / "pick_place_single.robmot"
        )
        _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)
    return tmp_path / "pick_place_single.ld.json"


@pytest.fixture
def constraint_graph(pick_place_single_jsonld: Path) -> Graph:
    """Fresh graph per test: parsed from the immutable JSON-LD so no test can leak
    triple mutations into another."""
    dataset = Dataset()
    dataset.parse(str(pick_place_single_jsonld), format="json-ld")
    graph = Graph()
    for quad in dataset.quads((None, None, None, None)):
        graph.add(quad[:3])
    return graph


def test_dsl_omits_derived_achd_solver_structure(constraint_graph: Graph) -> None:
    """The authored DSL graph never materializes IR-derived ACHD rows: those are added
    later, when the IR layer derives the solver structure from the authored constraints."""
    assert not set(constraint_graph.subjects(RDF.type, SLV.AccelerationConstraint))
    assert not set(constraint_graph.subjects(RDF.type, SLV.AccelerationConstraintSpecification))
    assert not set(constraint_graph.subjects(RDF.type, GEOM_OP_EXT["PoseDiffEvaluator"]))
    assert not list(constraint_graph.triples((None, CSTR_HDL["control-signal"], None)))
    assert not list(constraint_graph.triples((None, SLV["acceleration-energy"], None)))
    assert not set(
        constraint_graph.subjects(QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND.AccelerationEnergy)
    )


def test_controllers_reference_their_solver(constraint_graph: Graph) -> None:
    handlers = set(constraint_graph.subjects(RDF.type, CSTR_HDL.ConstraintHandler))
    solvers = set(constraint_graph.subjects(RDF.type, SLV.SolverWithInputAndOutput)) | set(
        constraint_graph.subjects(RDF.type, SLV_EXT.CommandForwardingSolver)
    )
    controllers = {
        controller
        for handler in handlers
        for controller in constraint_graph.objects(handler, CSTR_HDL.controllers)
    }
    assert controllers
    assert all(
        len(set(constraint_graph.objects(controller, CSTR_HDL_EXT.solver))) == 1
        for controller in controllers
    )

    components = set(constraint_graph.subjects(PROV.wasDerivedFrom, None))
    assert not components
    assert {
        constraint_graph.value(controller, CSTR_HDL_EXT.solver) for controller in controllers
    } <= solvers

    declared_limit_kinds = {
        kind
        for solver in solvers
        for limit in constraint_graph.objects(solver, ALGO_EXT.limits)
        if (signal := constraint_graph.value(limit, ALGO_EXT["in"])) is not None
        for kind in constraint_graph.objects(signal, QUDT_SCHEMA.hasQuantityKind)
    }
    assert declared_limit_kinds <= {
        QUDT_QKIND.Torque,
        QUDT_QKIND.LinearAcceleration,
        QUDT_QKIND.AngularAcceleration,
    }


def test_authored_output_limit_binds_to_derived_signal(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("METAMODELS_PATH", "/home/batsy/work/ms/src/metamodels")
    source = (MODELS / "pick_place_single" / "pick_place_single.robmot").read_text()
    source = source.replace(
        "linear-velocity zero-linvel = 0.0 m/s,",
        "linear-velocity zero-linvel = 0.0 m/s,\n"
        # The signal saturated is the solver's acceleration-energy row, not a velocity.
        "        linear-acceleration output-limit = 10.0 m/s2,",
    ).replace(
        "constraint: <pick.hold-x>,     Kp:",
        "constraint: <pick.hold-x>, output-saturation: saturation { "
        "max: <shared.spec.output-limit> }, Kp:",
    )
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_str(
        source, file_name=str(MODELS / "pick_place_single" / "pick_place_single.robmot")
    )
    _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)

    graph = Graph().parse(tmp_path / "pick_place_single.ld.json", format="json-ld")
    controller = next(
        node
        for node in graph.subjects(RDF.type, CSTR_HDL.Controller)
        if str(node).endswith("/ctrl-pk-hold-x")
    )
    saturation = next(graph.objects(controller, ALGO_EXT.limits))
    authored_output = graph.value(saturation, ALGO_EXT["in"])
    assert QUDT_QKIND.LinearVelocity in graph[
        authored_output : QUDT_SCHEMA.hasQuantityKind
    ]

    ir = generate_ir(tmp_path / "pick_place_single-app.ld.json")
    shared_ids = {getattr(item, "id", None) or item["id"] for item in ir["shared_data"]}
    controllers = [
        controller for handler in ir["cstr_hdl"] for controller in handler.controllers
    ]
    acceleration_constraints = [
        constraint
        for solver in ir["slv_arm"]
        for driver in solver.motion_drivers
        for constraint in driver.acceleration_constraint
    ]
    assert {controller.control_signal.id for controller in controllers} <= shared_ids
    assert {
        constraint.acceleration_energy.id for constraint in acceleration_constraints
    } <= shared_ids
    derived = next(
        controller
        for controller in controllers
        if controller.id == "ctrl_pk_hold_x"
    )
    assert derived.output_saturation.maximum.value == 10.0
    assert derived.output_saturation.input_signal is derived.control_signal
