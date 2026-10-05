# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Validate user-authored names."""

from __future__ import annotations

from urllib.parse import urlsplit

from textx import get_location
from textx.exceptions import TextXSemanticError


def validate_namespace_uris(model) -> None:
    """Reject namespace IRIs that mint malformed IRIs once a name is appended."""
    for declaration in model.namespaces:
        parsed = urlsplit(declaration.uri)
        if not parsed.scheme or not parsed.netloc:
            problem = "needs a scheme and an authority"
        elif parsed.query or parsed.fragment:
            problem = "cannot carry a query or a fragment"
        elif not declaration.uri.endswith(("/", "#")):
            problem = "must end with '/' or '#' to separate it from the names below it"
        elif "//" in parsed.path:
            problem = "has an empty path segment"
        else:
            continue
        raise TextXSemanticError(
            f"namespace '{declaration.name}' ('{declaration.uri}') {problem}",
            **get_location(declaration),
        )
