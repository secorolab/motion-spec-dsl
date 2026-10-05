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

## Development

As CI does, with the patched rdflib:

```bash
pip install pytest
pip install --force-reinstall --no-deps \
  "git+https://github.com/secorolab/rdflib.git@7.6.0-fix-multi-type-scoped-context"
pytest -q
ruff check src
```
