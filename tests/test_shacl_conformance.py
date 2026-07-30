# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""End-to-end SHACL validation through the public `motion-spec` CLI."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

WORKSPACE = Path(__file__).resolve().parents[3]
MODELS_DIR = WORKSPACE / "src" / "motion-spec-dsl" / "models"
METAMODELS = WORKSPACE / "src" / "metamodels"
MOTION_SPEC_CLI = Path(sys.executable).parent / "motion-spec"
MAINTAINED_MODELS = ("pick_place_single", "pick_place_dual", "admittance_arc_single")


@pytest.mark.parametrize("name", MAINTAINED_MODELS)
def test_maintained_model_conforms_to_shacl(name: str, tmp_path: Path) -> None:
    model = MODELS_DIR / name / f"{name}.robmot"
    env = {**os.environ, "METAMODELS_PATH": str(METAMODELS)}
    generation = tmp_path / "generation"

    gen = subprocess.run(
        [str(MOTION_SPEC_CLI), "gen", "ir", str(model), "-o", str(generation)],
        cwd=model.parent,
        env=env,
        capture_output=True,
        text=True,
    )
    assert gen.returncode == 0, gen.stdout + gen.stderr

    manifest = generation / "generated" / "model" / f"{name}-app.ld.json"
    check = subprocess.run(
        [str(MOTION_SPEC_CLI), "check", str(manifest)],
        cwd=model.parent,
        env=env,
        capture_output=True,
        text=True,
    )
    assert check.returncode == 0 and "Conforms: True" in check.stdout, check.stdout + check.stderr
