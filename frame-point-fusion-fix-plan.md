# Plan: De-fuse Frame + Point + SimplicialComplex (todo #1)

## Problem
`rdf.py::_emit_structural_entities` types a **single** URI as the full trio
`geom-ent:Frame` + `geom-ent:Point` + `geom-ent:SimplicialComplex` (see the SceneObject block and
the `frame_trio` loop). This is a shortcut so that one authored name satisfies three different
geometry shapes at once (`spatial-relations.ttl`):

| Relation node        | `of` requires            | `with-respect-to` requires | `reference-point` |
|----------------------|--------------------------|----------------------------|-------------------|
| `geom-rel:Pose`      | `geom-ent:Frame`         | `geom-ent:Frame`           | —                 |
| `geom-rel:Velocity/AccelerationTwist` | `geom-ent:SimplicialComplex` | `geom-ent:SimplicialComplex` | `geom-ent:Point` |
| `geom-rel:Position` (position coord)  | `geom-ent:Point`         | —                          | —                 |
| `as-seen-by` (coordinates.ttl)        | `geom-ent:Frame`         | —                          | —                 |

Because pose, its derived position coordinate, and any twist all reference the **same authored
name**, fusing the three types onto that one node makes every shape pass. It is semantically wrong
(a Frame is not a Point is not a body) and blocks any downstream consumer that wants to tell a
frame from the body it is attached to.

## Target model (semantically correct)
Each authored kinematic frame `X` becomes **three linked entities**:
- `X` — `geom-ent:Frame` (used by `Pose.of/wrt`, `as-seen-by`).
- `X` origin **Point** — `geom-ent:Point` (used by `Position.of`, `Twist/Wrench reference-point`).
- `X` attached **SimplicialComplex** / body — `geom-ent:SimplicialComplex` (used by `Twist.of/wrt`).

The Frame is linked to its origin Point and attached body via the geometry metamodel's own
relations — CHECK `src/comp-rob2b/metamodels/geometry/structural-entities.*` for the canonical
predicates (e.g. a Frame's origin Point, a Point's `of` SimplicialComplex). Reuse existing terms;
only invent `mj:`/`secorolab:` terms if the upstream metamodel has none (grep comp-rob2b +
src/metamodels first — same rule as the rest of this codebase).

`of`/`wrt`/`reference-point` must then be resolved **context-aware** at emission:
- Pose `of`/`wrt` → the Frame node.
- Position `of` → the origin Point node.
- Twist `of`/`wrt` → the body (SimplicialComplex) node; `reference-point` → the origin Point.
- `as-seen-by` → the Frame node (already correct).

## The hard part — downstream identity (this is why it is load-bearing)
`ir_gen.py` resolves each relation to a C++ identifier via type-dispatch and `self.id(node)`:
- `position_reference()` (ir_gen ~1806) dispatches by `rdf:type` in order **RigidObject → Frame →
  SimplicialComplex → Point**. Today the fused node hits **Frame first**, so *everything* collapses
  to `frame(X)` → id `"X"`, giving codegen one consistent identity.
- `frame()`, `point()`, `simplicial_complex()` each return `Frame/Point/SimplicialComplex(self.id(node))`.
- `scene_object()` already **decouples** RDF-node-id from the physical body via `MJ:body-name`
  (`body = g.value(id_, MJ["body-name"]) or self.id(id_)`) — this is the pattern to follow.

If `X` is split into `X` / `X-point` / `X-body` as distinct URIs, ir_gen will emit three different
identifiers for one physical link and **codegen/mj_kdl_wrapper will break** unless the split
entities dereference back to the same physical frame. Two options:

- **(A) Naming + resolver dereference:** name the split nodes off the base frame and have
  `point()` / `simplicial_complex()` follow the link back to the owning Frame's id (like
  `scene_object` uses `MJ:body-name`). Codegen keeps emitting the physical frame/link name.
- **(B) Genuinely distinct entities in codegen:** teach the mj_kdl_wrapper/codegen layer that a
  body/point/frame triad maps to one MuJoCo body. Larger blast radius.

Recommend **(A)**: it is the minimal change that keeps codegen output identical while making the
graph semantically correct. Validate output identifiers are unchanged by diffing generated C++.

## Scope / files
1. `src/motion_spec_dsl/rdf.py`
   - `_emit_structural_entities`: stop fusing; emit the Frame + origin-Point + body triad with the
     canonical links; drop the `frame_trio` type-spraying.
   - Context-aware `of`/`wrt`/`reference-point` resolution wherever these are written — including
     the derived position/twist nodes that currently **copy** `pose_of`/`pose_wrt` verbatim
     (search `GEOM_REL.of` / `with-respect-to` adds; e.g. the pose→position coordinate block).
   - SceneObject: same de-fusion (it currently gets Frame+Point+SimplicialComplex too).
2. `src/motion-spec/src/motion_spec/ir_gen.py`
   - `point()` / `simplicial_complex()` resolve to the owning frame's physical id (option A), or
     confirm codegen tolerates distinct ids.
   - `position_reference()` dispatch order: revisit now that types are disjoint (no node carries
     two of Frame/Point/SimplicialComplex).
3. Possibly `motion_spec/codegen` templates / mj_kdl_wrapper — only if identifiers change (option A
   aims to avoid this).

## SHACL note
No SHACL change is needed (unlike #2): the fusion is what makes today's graph conform; a correct
split that routes each relation to the right entity type ALSO conforms. SHACL conformance is the
acceptance gate, unchanged. Watch the cross-relation SPARQL constraints in
`spatial-operators.ttl` (`ComposePose-in1-of-equals-in2-wrt`, `…composite-of-equals-in2-of`) and
`coordinates.ttl` — splitting must keep `of`/`wrt` identity chains consistent across composed poses.

## Validation loop (acceptance gate — heavier than #2)
1. `pytest tests/test_motion_spec_graph.py tests/test_validation.py` → 82 passed.
2. SHACL conformance for all conforming models (`test_shacl_conformance.py` → 4 passed), plus
   `pick_place_dual` should not regress further.
3. **Generated-C++ diff:** regenerate IR + C++ for `pick_place_single`, `pick_place_single_rnea`,
   `admittance_arc_single` before/after; the emitted identifiers/kinematic references must be
   equivalent (option A → byte-diff clean or only-intended changes).
4. **Build + run** (the real gate the user asked for):
   ```
   cd src/bdd_collab_bhv_cpp/models
   export METAMODELS_PATH=/home/batsy/work/ms/src/metamodels INSTALL=/home/batsy/work/ms/install
   make run MODEL=admittance_arc_single        # or run-headless STEPS=<n> for automated check
   ```
   Must build and run the sim without solver/reference errors. Repeat for pick_place_single.
5. Determinism byte-identical across `PYTHONHASHSEED` (already guarded).

## Recommendation on sequencing
High-risk, cross-repo (emitter + IR + possibly codegen). Do it **after** #2 lands and only with the
build+run gate wired in from the first change — a graph that "conforms" can still produce a sim that
references the wrong body. Land it incrementally: (1) emit the linked triad WITHOUT changing
relation targets and confirm no regression; (2) flip Position/Twist relation targets to Point/body
and fix ir_gen resolvers; (3) drop the fusion type-spray; validate build+run at each step.
```
```
