# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Resolve application-manifest IRIs to local model and metamodel files."""

import os
from pathlib import Path

from rdf_utils.namespace import URL_COMP_ROB2B, URL_SECORO
from rdf_utils.resolver import PKG_CACHE_ROOT, IriToFileResolver, install_resolver

from motion_spec_dsl.rdf_parser.vocab import APP

PACKAGE_ROOT = Path(__file__).resolve().parents[3]
METAMODELS_URL = "https://secorolab.github.io/metamodels/"
COMP_ROB2B_URL = "https://comp-rob2b.github.io/metamodels/"

_resolver = None


def metamodels_root() -> Path | None:
    """Locate the local secorolab metamodels checkout (honours METAMODELS_PATH)."""
    roots = []
    env_path = os.environ.get("METAMODELS_PATH")
    if env_path:
        roots.append(Path(env_path))
    for start in (PACKAGE_ROOT, Path.cwd()):
        roots.extend([start, *start.parents])
    for root in roots:
        for candidate in (root, root / "src" / "metamodels", root / "metamodels"):
            if (candidate / "prov.json").exists():
                return candidate
    return None


def metamodel_url_map() -> dict[str, str]:
    """Map the metamodel IRI prefixes to their local checkouts, longest prefix first.

    Without a checkout (an installed deployment) this is rdf-utils' own cache, read and filled
    exactly as plain rdf-utils does.
    """
    root = metamodels_root()
    if root is None:
        # The metamodel prefixes are listed before the hosts: a generated model's iri-map claims
        # the whole host, and would otherwise download every metamodel into its own directory.
        return {
            METAMODELS_URL: str(Path(PKG_CACHE_ROOT) / "secoro" / "metamodels"),
            COMP_ROB2B_URL: str(Path(PKG_CACHE_ROOT) / "comp-rob2b" / "metamodels"),
            URL_SECORO: str(Path(PKG_CACHE_ROOT) / "secoro"),
            URL_COMP_ROB2B: str(Path(PKG_CACHE_ROOT) / "comp-rob2b"),
        }
    url_map = {METAMODELS_URL: str(root)}
    comp_rob2b = root.parent / "comp-rob2b" / "metamodels"
    if comp_rob2b.exists():
        url_map[COMP_ROB2B_URL] = str(comp_rob2b)
    return url_map


def install_metamodel_resolver(extra_map: dict | None = None) -> None:
    """Point the process-wide resolver at the metamodels plus a model's own iri-map.

    rdf-utils takes the first matching prefix, so the metamodel prefixes precede the model's
    map, which claims only its host root. Dev checkouts never download; the cache downloads a
    file on its first miss. The one urllib opener lives here: callers swap only its map.
    """
    global _resolver
    url_map = {**metamodel_url_map(), **(extra_map or {})}
    if _resolver is None:
        _resolver = IriToFileResolver(url_map, download=metamodels_root() is None)
    else:
        _resolver.url_map = url_map
    install_resolver(_resolver)


def build_url_map(dataset, manifest_path) -> dict[str, str]:
    """The IRI-to-local-path map a manifest's iri-map entries declare, relative to the manifest.

    It reads quads, so it works on union and non-union rdflib Datasets alike.
    """
    manifest_dir = Path(manifest_path).resolve().parent
    url_map = {}
    for key in {o for _, _, o, _ in dataset.quads((None, APP["iri-map"], None, None))}:
        path_node = next((o for _, _, o, _ in dataset.quads((key, APP.path, None, None))), None)
        if path_node is None:
            continue
        value = str(path_node)
        if Path(value).is_absolute():
            url_map[str(key)] = value
            continue
        path = manifest_dir / value
        if not path.exists():
            source_path = PACKAGE_ROOT / value
            path = source_path if source_path.exists() else Path.cwd() / value
        url_map[str(key)] = str(path)
    return url_map
