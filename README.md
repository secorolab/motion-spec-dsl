# motion-spec-dsl

The textX authoring language for guarded robot motion. `.robmot` models compose
scene and FSM models with typed context, constraints, monitors, controllers, and
solvers.

User documentation—including setup, the complete language reference, and model
tutorials—is maintained in
[`motion-spec`](https://github.com/secorolab/motion-spec/tree/dev/docs/sphinx/source/dsl).

## Development

Use the workspace virtual environment and install this checkout without
re-resolving sibling packages:

```bash
cd /path/to/workspace
source .venv/bin/activate
python -m pip install --no-deps -e src/motion-spec-dsl
```

Verify language registration and parse the maintained model:

```bash
textx list-languages
textx check src/motion-spec-dsl/models/pick_place_single/pick_place_single.robmot
```

Run repository checks from `src/motion-spec-dsl`:

```bash
pytest
ruff check src tests
```
