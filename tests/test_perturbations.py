# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A handler's perturbations: a wrench the simulator applies to a scene body."""

from __future__ import annotations

from rdflib.namespace import RDF

from motion_spec_dsl.langs import motion_spec_metamodel
from motion_spec_dsl.rdf.motion_spec import MotionSpecDatasetBuilder
from motion_spec_dsl.rdf_parser.vocab import RBDYN_OP, SIM, SLV
from support import BASE, BASE_TEXT, PERTURBATION, PUSH, SOLVERS_END, SPEC


def test_a_perturbation_pushes_its_body_with_the_wrench_it_names() -> None:
    source = BASE_TEXT.replace(SPEC, SPEC + PUSH, 1).replace(SOLVERS_END, PERTURBATION, 1)
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = MotionSpecDatasetBuilder(model).build()[0].default_graph

    [perturbation] = graph.subjects(RDF.type, SIM.Perturbation)
    assert str(graph.value(perturbation, SLV["attached-to"])).endswith("/g_base")
    [push] = graph.subjects(RDF.type, RBDYN_OP.WrenchFromPositionDirectionAndMagnitude)
    assert str(graph.value(push, RBDYN_OP.magnitude)).endswith("/push")
    assert str(graph.value(push, RBDYN_OP.direction)).endswith("/push-dir")
