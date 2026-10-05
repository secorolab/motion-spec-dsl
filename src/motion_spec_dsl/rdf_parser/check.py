# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Validate generated motion-spec RDF against its declared SHACL constraints."""

from pathlib import Path

import pyshacl
import rdflib

from motion_spec_dsl.rdf_parser.manifest import build_url_map, install_metamodel_resolver
from motion_spec_dsl.rdf_parser.vocab import APP


def validate_manifest(app_model: str | Path, *, meta_shacl: bool = False) -> tuple[bool, str]:
    """Validate one application manifest and return conformance plus the SHACL report."""
    dataset = rdflib.Dataset()
    dataset.parse(app_model, format="json-ld")
    install_metamodel_resolver(build_url_map(dataset, Path(app_model).resolve()))
    for location in {o for _, _, o, _ in dataset.quads((None, APP["import"], None, None))}:
        dataset.parse(location=location, format="json-ld")
    return validate_dataset(dataset, meta_shacl=meta_shacl)


def validate_dataset(dataset: rdflib.Dataset, *, meta_shacl: bool = False) -> tuple[bool, str]:
    """Validate a loaded application dataset as one graph and return conformance plus the report.

    pyshacl validates each named graph of a dataset on its own, and shapes span documents.
    """
    data = rdflib.Graph()
    for s, p, o, _graph in dataset.quads():
        data.add((s, p, o))
    g_sh = rdflib.Dataset()
    metamodels = {str(o) for _, _, o, _ in dataset.quads((None, APP["constraints"], None, None))}
    if not metamodels:
        return (
            False,
            (
                "Validation Report\nConforms: False\n"
                "No SHACL constraint files were listed in the application manifest."
            ),
        )
    for location in metamodels:
        try:
            g_sh.parse(location=location, format="turtle")
        except (OSError, SyntaxError) as exc:
            return (
                False,
                (
                    "Validation Report\nConforms: False\n"
                    f"Failed to load SHACL constraint graph {location}: {exc}"
                ),
            )

    conforms, _v_graph, v_text = pyshacl.validate(
        data_graph=data, shacl_graph=g_sh, inference="none", meta_shacl=meta_shacl
    )
    return bool(conforms), v_text
