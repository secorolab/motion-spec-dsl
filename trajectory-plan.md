# Plan: New Trajectory Types (Circle, SemiCircle, Helix)

## Context

The motion-spec-dsl currently supports only `Lerp`. Three new trajectory types
are needed, each fully specified in Cartesian space:

- **Circle**: full 360° circle, defined by geometry (center, radius, plane-normal).
- **SemiCircle**: 180° arc defined by two endpoint positions and a radius;
  `plane-normal` defines the plane of the arc and disambiguates which of the two
  possible centers (both on the perpendicular bisector of the chord) to use.
- **Helix**: spiral — circular motion in a plane with a linear rise along a
  central axis (the one parameter you DO move along linearly, so `axis` is apt).

---

## Trajectory type specifications

### 1. `Circle` — full 360° circular path
**DSL surface:**
```
traj: Trajectory = Circle {
    center:       <spec.center-pos>,     // Position — center of circle
    radius:       <spec.radius>,         // LinearDistance
    plane-normal: <spec.circle-normal>,  // Direction — normal to the plane of the circle
    alpha:        <spec.alpha>           // TrajectoryProgress [0,1] → 0 to 2π
}
```
`plane-normal` defines which plane the circle lives in.
The starting point is determined by a fixed reference direction perpendicular to
`plane-normal` (implementation: stable perpendicular via Gram-Schmidt).

**C++ math:**
```cpp
const double _angle = 2.0 * M_PI * motion_spec::runtime::smoothstep(<alpha>);
const KDL::Vector _n   = <plane_normal direction>;
const KDL::Vector _ref = motion_spec::runtime::plane_reference(_n);
const KDL::Vector _pos = <center_pos> + KDL::Rotation::Rot(_n, _angle) * (_ref * <radius>);
const KDL::Rotation _rot = KDL::Rotation::Rot(_n, _angle);
shared.<trajectory> = KDL::Frame(_rot, _pos);
```
Runtime helper: `plane_reference(normal)` — stable unit vector ⊥ normal.

---

### 2. `SemiCircle` — 180° arc defined by endpoints
**DSL surface:**
```
traj: Trajectory = SemiCircle {
    start:        <spec.start-pos>,      // Position — entry point of the arc
    end:          <spec.end-pos>,        // Position — exit point of the arc
    radius:       <spec.radius>,         // LinearDistance — must be ≥ |start-end|/2
    plane-normal: <spec.arc-normal>,     // Direction — normal to arc plane; selects center side
    alpha:        <spec.alpha>           // TrajectoryProgress
}
```
`plane-normal` disambiguates: the center lies on the side of the chord in the
positive-normal half-space.

**C++ math:**
```cpp
const KDL::Vector _n      = <plane_normal direction>;
const KDL::Vector _chord  = <end_pos> - <start_pos>;
const KDL::Vector _mid    = 0.5 * (<start_pos> + <end_pos>);
const double _h_chord     = 0.5 * _chord.Norm();
const double _h           = std::sqrt(<radius>*<radius> - _h_chord*_h_chord);
const KDL::Vector _perp   = (_n * _chord).Normalize(); // in-plane, perp to chord
const KDL::Vector _center = _mid + _h * _perp;
const double _angle       = M_PI * motion_spec::runtime::smoothstep(<alpha>);
const KDL::Vector _r0     = <start_pos> - _center;
const KDL::Vector _pos    = _center + KDL::Rotation::Rot(_n, _angle) * _r0;
const KDL::Rotation _rot  = KDL::Rotation::Rot(_n, _angle);
shared.<trajectory> = KDL::Frame(_rot, _pos);
```

---

### 3. `Helix` — spiral along a central axis
**DSL surface:**
```
traj: Trajectory = Helix {
    center:      <spec.helix-center>,   // Position — point on the helix axis
    radius:      <spec.radius>,         // LinearDistance — helix radius
    axis:        <spec.helix-axis>,     // Direction — the axis you rise along (and rotate around)
    pitch:       <spec.pitch>,          // LinearDistance — rise per full revolution
    revolutions: <spec.revolutions>,    // Scalar — number of turns
    alpha:        <spec.alpha>          // TrajectoryProgress
}
```
`axis` is the direction you physically translate along (hence not `plane-normal`).
Each cross-section perpendicular to `axis` is a circle of the given radius.

**C++ math:**
```cpp
const double _s     = motion_spec::runtime::smoothstep(<alpha>);
const double _angle = <revolutions> * 2.0 * M_PI * _s;
const double _rise  = <pitch> * <revolutions> * _s;
const KDL::Vector _ax  = <axis direction>;
const KDL::Vector _ref = motion_spec::runtime::plane_reference(_ax);
const KDL::Vector _pos = <center_pos>
    + KDL::Rotation::Rot(_ax, _angle) * (_ref * <radius>)
    + _ax * _rise;
const KDL::Rotation _rot = KDL::Rotation::Rot(_ax, _angle);
shared.<trajectory> = KDL::Frame(_rot, _pos);
```

---

## Implementation layers

### A. Grammar — `motion-spec-dsl/src/motion_spec_dsl/metamodels/motion_spec.tx`
Extend `TrajectoryType`: add `"Circle" | "SemiCircle" | "Helix"`.
Add grammar rules `CircleSpec`, `SemiCircleSpec`, `HelixSpec` (each a `ContextRef`-field block):

| Spec | fields |
|---|---|
| `CircleSpec` | `center`, `radius`, `plane-normal`, `alpha` |
| `SemiCircleSpec` | `start`, `end`, `radius`, `plane-normal`, `alpha` |
| `HelixSpec` | `center`, `radius`, `axis`, `pitch`, `revolutions`, `alpha` |

Extend `TrajectoryValue` to dispatch to the correct spec.

### B. Domain — `motion-spec-dsl/src/motion_spec_dsl/domain.py`
Add `CircleSpec`, `SemiCircleSpec`, `HelixSpec` dataclasses.
Extend `TrajectoryValue` with three new optional spec fields (only one populated per instance).

### C. Namespace — `motion-spec/src/motion_spec/namespace.py` (TRAJ class)
```python
Circle: URIRef
SemiCircle: URIRef
Helix: URIRef
# new predicates
center: URIRef
radius: URIRef
plane_normal: URIRef   # IRI: traj:plane-normal
end: URIRef            # SemiCircle's end position
axis: URIRef           # Helix axis
pitch: URIRef
revolutions: URIRef
```
Existing `start`, `alpha`, `trajectory` reused.

### D. RDF emitter — `motion-spec-dsl/src/motion_spec_dsl/rdf.py`
Add `_emit_circle_quantity`, `_emit_semi_circle_quantity`, `_emit_helix_quantity`.
Pattern: same as `_emit_trajectory_quantity` — emit `traj:Trajectory` types on
output node, emit operator node (`traj:Circle`/`traj:SemiCircle`/`traj:Helix`)
with all input refs via `_emit_context_ref_node`, emit `traj:trajectory` back-link.
Route from `_emit_context_quantities` based on which spec field is populated.
All three output Pose (same value-kind logic as Lerp).

### E. IR generation — `motion-spec/src/motion_spec/ir_gen.py`
Add to `ops_generic`:
```python
Operator(type_=TRAJ["Circle"],
    input=[TRAJ["center"], TRAJ["radius"], TRAJ["plane-normal"], TRAJ["alpha"]],
    output=[TRAJ["trajectory"]]),
Operator(type_=TRAJ["SemiCircle"],
    input=[TRAJ["start"], TRAJ["end"], TRAJ["radius"], TRAJ["plane-normal"], TRAJ["alpha"]],
    output=[TRAJ["trajectory"]]),
Operator(type_=TRAJ["Helix"],
    input=[TRAJ["center"], TRAJ["radius"], TRAJ["axis"],
           TRAJ["pitch"], TRAJ["revolutions"], TRAJ["alpha"]],
    output=[TRAJ["trajectory"]]),
```

### F. Codegen — `motion-spec/src/motion_spec/codegen.py`
Extend `closure.get("type")` check in `add_motion_trajectory_progress`:
```python
{"Lerp", "Circle", "SemiCircle", "Helix"}
```

### G. Template — `motion-spec/code-generator/module.stg`
- `shared-member-Circle/SemiCircle/Helix` — identical to `shared-member-Lerp` (output `KDL::Frame`).
- `emit-call-Circle`, `emit-call-SemiCircle`, `emit-call-Helix` — new blocks with the math above.
- Add `plane_reference(normal)` to the runtime helper section.

### H. Metamodels — `src/metamodels/task/trajectory.{json,shacl.ttl}`
Add scoped JSON-LD context entries for `Circle` (with `center`, `radius`,
`plane-normal`, `alpha`, `trajectory`), `SemiCircle` (with `start`, `end`,
`radius`, `plane-normal`, `alpha`, `trajectory`), and `Helix` (with `center`,
`radius`, `axis`, `pitch`, `revolutions`, `alpha`, `trajectory`).
Add `sh:NodeShape` for each in SHACL with `sh:minCount 1 / sh:maxCount 1 /
sh:class qudt:Quantity` on every input.

---

## Files to modify / create

| File | Change |
|---|---|
| `motion-spec-dsl/.../metamodels/motion_spec.tx` | Extend grammar |
| `motion-spec-dsl/.../domain.py` | Add 3 spec dataclasses; extend TrajectoryValue |
| `motion-spec/src/motion_spec/namespace.py` | Add 3 type URIRefs + 7 predicate URIRefs |
| `motion-spec-dsl/.../rdf.py` | Add 3 emit methods + routing |
| `motion-spec/src/motion_spec/ir_gen.py` | Add 3 operators |
| `motion-spec/src/motion_spec/codegen.py` | Extend progress type check |
| `motion-spec/code-generator/module.stg` | 3 emit-call templates + plane_reference helper |
| `src/metamodels/task/trajectory.json` | Add 3 context entries |
| `src/metamodels/task/trajectory.shacl.ttl` | Add 3 NodeShape blocks |
| `motion-spec-dsl/tests/fixtures/valid/` | 3 new fixture .robmot files |
| `motion-spec-dsl/tests/test_motion_spec_graph.py` | 3 new graph tests |

---

## Verification

1. `make jsonld MODEL=pick_place` — Lerp unchanged, no regressions.
2. Three new fixture models parse and generate JSON-LD without errors.
3. `motion-spec check` passes on their manifests.
4. `motion-spec ir-gen` produces correct closures in `ir.json`.
5. `motion-spec codegen` expands all three templates; generated C++ contains the arc/helix math.
6. `pytest tests/ -q` — all pass including 3 new graph tests.
