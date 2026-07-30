# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Validation of user-authored names."""

from __future__ import annotations

from urllib.parse import urlsplit

from motion_spec_dsl.validation.common import semantic_error


def validate_namespace_uris(model) -> None:
    """Reject namespace URIs that mint malformed IRIs once a name is appended."""
    for declaration in getattr(model, "namespaces", ()):
        parsed = urlsplit(declaration.uri)
        if not parsed.scheme or not parsed.netloc:
            problem = "needs a scheme and an authority"
        elif parsed.query or parsed.fragment:
            problem = "cannot carry a query or a fragment name"
        elif not declaration.uri.endswith(("/", "#")):
            problem = "must end with '/' or '#' to separate it from the names below it"
        elif "//" in parsed.path:
            problem = "has an empty path segment"
        else:
            continue
        raise semantic_error(
            f"namespace '{declaration.name}' {problem}: {declaration.uri}", declaration
        )
