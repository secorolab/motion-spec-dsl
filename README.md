# motion-spec-dsl

The textX language for guarded robot motion. A `.robmot` model composes scene and FSM models
with typed context, constraints, monitors, controllers and solvers, and is emitted as JSON-LD.

The language reference and tutorials are part of the
[motion-spec documentation](https://secorolab.github.io/motion-spec/dsl/).

## Install

Python >= 3.10. rdf-utils, scene-dsl and coord-dsl are not on PyPI; `pyproject.toml` pulls
them from git.

```bash
git clone https://github.com/secorolab/motion-spec-dsl.git && cd motion-spec-dsl
pip install -e .
```

Check that textX sees the language and parses an example:

```bash
textx list-languages
textx check src/motion_spec_dsl/models/01_pick_and_place/pick_and_place.robmot
```

## Generate

Emit a model as JSON-LD: the model graph, its application manifest and a provenance document.

```bash
textx generate src/motion_spec_dsl/models/01_pick_and_place/pick_and_place.robmot \
  --target jsonld -o out
```

This writes `out/pick_and_place.ld.json`, `out/pick_and_place-app.ld.json` and
`out/provenance/dsl.ld.json`.
