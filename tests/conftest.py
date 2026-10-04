# SPDX-License-Identifier: MPL-2.0
"""Point the metamodel loader at the workspace's metamodels checkout."""

from __future__ import annotations

import os
from pathlib import Path

import pytest

METAMODELS = Path(__file__).resolve().parents[2] / "metamodels"


@pytest.fixture(autouse=True)
def _metamodels_path(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("METAMODELS_PATH", os.environ.get("METAMODELS_PATH", str(METAMODELS)))
