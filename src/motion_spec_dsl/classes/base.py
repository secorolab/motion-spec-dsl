# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Classes bound to the shared motion-spec grammar rules."""

from __future__ import annotations


class NamespaceDeclare:
    """A namespace declaration: a prefix `name` and its `uri`."""

    def __init__(self, name: str = "", uri: str = "", **_):
        self.name = name
        self.uri = uri


class Import:
    """An `importURI` reference to another model or FSM file."""

    def __init__(self, importURI: str = "", **_):
        self.importURI = importURI
