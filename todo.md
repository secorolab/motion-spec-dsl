# TODO

## Immediate

- Impedance is parsed and semantically validated, but intentionally blocked before RDF/codegen until ontology and IR support are implemented.
- ABAG is reserved and immediately rejected as not implemented.


## Later

- Carry `JointPosition` / posture-control semantics through `motion-spec` IR parsing and downstream codegen.
- Decide whether non-posture joint-torque controllers should be supported as a separate dispatch family from `for Posture`.
- Add prioritization support for mixed ACHD motion drivers, including `joint-force`.
