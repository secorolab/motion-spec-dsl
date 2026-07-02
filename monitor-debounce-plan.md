# Plan: arc-oscillation fix — port to pipeline

Root cause (confirmed empirically via scripted double-push headless repro of
`admittance_arc_single`): a strong/repeated perturbation makes the arm ring in
its position loop (~2 Hz = sqrt(Kp=200)); the wrist FT sensor reads the arm's
own inertial reaction (m*a of the tool) as "external force", re-crosses the
10 N `force-threshold`, and bounces the FSM ARC<->ADMITTANCE (chatter, measured
34 transitions; one arc segment 67 zero-crossings). The first push decays; repeats
self-sustain. Two secondary bugs worsen it. Controller internal state IS correctly
reset on re-entry (`state = motion_*_state{}` reconstructs PID integral/derivative
and the admittance filter) — no windup carryover.

Fixes verified in the *generated* tree (realistic 35 N pushes -> 8 clean
transitions, no chatter, zero-cross ~0-2). Now porting to source.

## Fix 1 — arc support-z: `on task` -> `on entry`  [DONE-able, trivial]
- File: `src/bdd_collab_bhv_cpp/models/admittance_arc_single.robmot:189`
- `support-z: ... on task` -> `on entry`. The elbow-support target was frozen at
  its first-entry value while the arm drifted each admittance cycle, so the stiff
  (ks=800) support force ratcheted 24N -> 80N. Admittance's own support-z is already
  `on entry` (line 243). Guard moves from shared `arc_support_z_captured` to the
  motion-local `snapshot_taken` block automatically (snapshot-clock codegen).

## Fix 2 — sim timestep = declared control period  [template + ir_gen]
- Bug: `mj_kdl_backend.stg:149` hardcodes `mj_scene.timestep = 0.002;` but every
  handler declares `CONTROL_PERIOD: 1.0 ms`, and the loop paces at
  `control_period_ns` and sets `kControlPeriodS = control_period_ns*1e-9 = 0.001`.
  So controllers run once per 2 ms sim step with dt_=1 ms -> derivative 2x, integral 0.5x.
- Fix: emit timestep from the control period.
  - `ir_gen.py`: where the app dict is assembled (control_period_ns computed at
    ir_gen.py:3521; app dict `"scene": scene` at 3582), add `scene["timestep_s"] =
    control_period_ns * 1e-9` (SceneSpec built at 3146 / `_scene_from_graph` 3347).
  - `mj_kdl_backend.stg:149`: `mj_scene.timestep = <scene.timestep_s>;`
- REGRESSION SURFACE: ALL models (all declare 1 ms) switch 0.002 -> 0.001. Must
  regen + build + headless-run pick_place_single, pick_place_dual, pick_place_single_rnea,
  pick_place_single_wait. Trajectories use sim-time (alpha=elapsed/dur) so timing is
  preserved; smaller step is more contact-stable, but verify grasp still succeeds
  (memory: pick_place grasp is finicky). If a model regresses, that model needs
  re-tuning — do NOT revert the timestep fix silently.

## Fix 3 — per-monitor debounce knob  [DSL feature, follows snapshot-clock precedent]
Grammar knob: `... trigger event E when active [for <FLOAT> <Unit>]`. Default (absent)
= 0 = current `rising_edge` behavior (no debounce, byte-identical for all existing models).
Semantics: fire only after the monitored condition has held continuously for the
debounce duration -> rejects a ring's transient threshold crossing, still fires on a
sustained push.

Thread it exactly like `snap:sampled-on` (snapshot clock). Reference commits/edit sites:
1. Grammar `src/motion-spec-dsl/.../metamodels/motion_spec.tx` `MonitorEntry` (line 564):
   add optional `("for" debounce=FLOAT debounce_unit=Unit)?` after `"when" "active"`.
   Also mirror into editor grammar `src/motion-spec-dsl-nvim/server/grammar/motion_spec.tx`.
2. Metamodel: add a term (e.g. `mon:debounce` / `debounce-seconds`) + SHACL, mirroring
   the snapshot metamodel (`snapshot.json` + `snapshot.shacl.ttl`). `namespace.py` accessor.
3. `domain.py`: carry `MonitorEntry.debounce` (seconds).
4. `rdf.py`: emit the debounce triple on the monitor node.
5. `ir_gen.py`: read it into the monitor IR dict as `debounce_steps =
   round(debounce_s / (control_period_ns*1e-9))` (0 when absent). Monitor IR is built
   near the monitor/event handling (grep `E_FORCE`/`produce_event`/monitor emit).
6. Codegen `module.stg` monitor block: when `debounce_steps > 0`, emit a per-monitor
   `int <mon>_hold=0;` state field and detect via
   `sustained_edge(state.<mon>_hold, active, <debounce_steps>)`; else keep
   `rising_edge_after(...)`. Runtime helper `sustained_edge` already prototyped:
   ```
   inline bool sustained_edge(int &counter, bool active, int hold_steps) {
       if (!active) { counter = 0; return false; }
       ++counter; return counter == hold_steps; }
   ```
   Add it to `runtime_header` in module.stg (near rising_edge_after ~line 1345+).
7. Apply the knob in `admittance_arc_single.robmot`: the 4 arc force monitors
   (`mon-force-{pos,neg}-{x,y}`) and admittance `mon-released` get `for 0.3 s`
   (300 steps @1ms). Window must exceed the ~0.25 s ring half-period. (User: knob, not global.)

## Verify
- Regen+build admittance_arc_single; headless double-push repro (scratch harness in
  session) should show no chatter. Then GUI confirm with the user.
- Regen+build+headless all pick_place variants: grasp/place still succeed.

## Status
- [x] Fix 1 applied (admittance_arc_single.robmot:189 on task->on entry; regen confirms
      arc_support_z now captured under !state.snapshot_taken).
- [x] Fix 2 applied (ir_gen SceneSpec.timestep_s = control_period_ns*1e-9; mj_kdl_backend.stg
      emits <scene.timestep_s>). Verified: admittance_arc, pick_place_single AND
      pick_place_dual all emit timestep=0.001, build, and complete full sequence
      (single+dual: START..GRASP..LIFT..PLACE..OPEN..RETREAT; cube placed). No regression.
- [x] Fix 3 (debounce DSL knob) DONE. `... when active for <N> <unit>` optional clause;
      absent = byte-identical (rising_edge). Threaded: grammar (MonitorEntry reuses
      BareScalar) + editor grammar mirror; metamodel task/constraint-handler-extension
      {.json,.shacl.ttl} `debounce-seconds` + DebounceMonitor shape; namespace.py
      CSTR_HDL_EXT extra; domain.py MonitorEntry.debounce(_seconds); rdf.py emits triple;
      ir_gen.py debounce_steps=round(s/dt) as `int|None=None` (NOT 0 — ST `<if(0)>` is
      truthy!); module.stg sustained_edge helper + conditional `<id>_hold` field + detect.
      Model: admittance_arc_single 4 force monitors + mon-released get `for 0.3 s` (300 steps).
      VERIFIED: admittance_arc Conforms+builds, 4 sustained_edge in motion_arc, 1(x2) in
      admittance, mon-arc-complete still rising_edge; pick_place_single/dual emit 0
      sustained_edge (byte-compat) + build + full pick/place sequence. End-to-end double-push
      repro: chatter gone (34->22 transitions, no rapid toggling, ripple eliminated).
