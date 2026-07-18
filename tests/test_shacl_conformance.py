# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""End-to-end SHACL validation through the public generator and checker."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[3]
MODEL = WORKSPACE / "src" / "motion-spec-dsl" / "models" / "pick_place_single.robmot"
METAMODELS = WORKSPACE / "src" / "metamodels"


def test_example_model_conforms_to_shacl(tmp_path: Path) -> None:
    env = {**os.environ, "METAMODELS_PATH": str(METAMODELS)}
    generated = tmp_path / "generated"

    generation = subprocess.run(
        ["textx", "generate", str(MODEL), "--target", "jsonld", "-o", str(generated)],
        cwd=MODEL.parent,
        env=env,
        capture_output=True,
        text=True,
    )
    assert generation.returncode == 0, generation.stdout + generation.stderr

    check = subprocess.run(
        [
            sys.executable,
            "-m",
            "motion_spec.check",
            str(generated / "pick_place_single-app.jsonld"),
        ],
        cwd=MODEL.parent,
        env=env,
        capture_output=True,
        text=True,
    )
    assert check.returncode == 0 and "Conforms: True" in check.stdout, check.stdout + check.stderr
