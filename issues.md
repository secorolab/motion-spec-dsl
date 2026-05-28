# SHACL Validation Issues

1. SHACL checker was hiding failures by silently ignoring failed constraint graph loads.
2. Generated manifests referenced unavailable comp-rob2b TTLs, causing 404s or environment-dependent validation.
3. FeedForward controller semantics do not match upstream `cstr_hdl:Controller`, which requires `error-signal` even though FeedForward should use `reference-signal`.
4. Pose/trajectory position references can still emit full pose or trajectory nodes where SHACL expects `qudt_quant:Position`.
5. Fixed: monitors now emit required `cstr_hdl:error` links, including aggregate until-motion monitors.
6. Event monitors are missing required `cstr_hdl:event-queue` links.
7. Some evaluator nodes are linked as constraint evaluators without the base `cstr_hdl:ConstraintEvaluator` type.
8. Gripper/feedforward constraints are not evaluated, causing the “All constraints must be evaluated” SPARQL violation.
9. Solver nodes can receive multiple `slv:motion-drivers`, violating the max-count shape.
10. `slv:AccelerationConstraintSpecification` nodes are missing required `slv:attached-to` links.
11. Gravity nodes need `dyn_coord:UniformGravitationalFieldCoordinate` typing and root-frame `dyn_coord:as-seen-by` metadata.
12. Scene objects are used as pose frames/points, while SHACL expects `geom:Frame` or `geom:Point`.
13. Environment object positions use world as `with-respect-to`, but position SHACL expects a `geom:Point`.
14. Fixed: distance quantities keep `geom-coord:LinearDistanceCoordinate` typing without over-typing scalar distances as `geom-rel:LinearDistance`.
15. Relative pose-valued trajectory nodes are used where `geom-coord:PoseCoordinate` is required.
16. Relative pose declarations with `wrt != as-seen-by` violate comp-rob2b `PoseCoordinate` `as-seen-by == with-respect-to` rule.
17. Existing DSL tests include expected-output drift and unrelated fixture/parser failures that need cleanup separately.
