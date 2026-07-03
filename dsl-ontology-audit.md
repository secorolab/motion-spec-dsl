# motion-spec-dsl — ontological / taxonomical audit

Concepts modeled at the wrong categorical level, or the same concept modeled
twice. Scope: the quantity / value / constraint ontology. Excludes ENVIRONMENT
(separate refactor) and pure dead-code/YAGNI items.

Unifying defect: the **"quantity kind" axis is overloaded to do four jobs** —
it encodes operators, entities/fields, provenance, and synonyms — while
"reference a quantity" and "axis" are each modeled twice. A clean model keeps
four orthogonal axes: **entity** / **quantity (kind + dimension)** /
**reference-generator (operator)** / **value-provenance (role)**, with one
selector grammar and one kind-scoped axis vocabulary.

---

## 1. Operators/generators typed as "quantities"  (archetype: ExternalForce family)
`ContextQuantityType` (`motion_spec.tx:317`) and `domain.QuantityType` list
`Trajectory`, `VelocityProfile`, `Admittance` next to `Force`, `Pose`,
`Distance`. Those three are **reference generators / operators** — their value
forms are operator specs (`TrajectoryValue = Arc{…}`, `ProfileSpec = Profile{…}`,
`AdmittanceSpec = {mass, damping,…}`). Their *output* is a quantity; they are
not quantities. "Kind" is thereby overloaded to mean both a physical dimension
(Force, Pose) and an operator class (Admittance, Trajectory).
Tell: `Impedance` — admittance's dual — is correctly a `ControllerType`, not a
quantity. The asymmetry gives it away.
→ Separate `ReferenceGenerator`/operator category whose *result* carries a QuantityKind.

## 2. Entities and fields typed as "quantities"  — `WorldQuantityType`
`Frame`, `Link`, `SceneObject`, `KinematicChain`, `Gravity` share
`WorldQuantityType` (`motion_spec.tx:287`) with real quantities (`Pose`,
`VelocityTwist`, `Wrench`). `WORLD_STRUCTURE_TYPES` (`rdf.py:204`) then
re-classifies exactly those five as `geom-ent:Frame` / `SimplicialComplex` /
`env:RigidObject` / `KinematicChain` / `UniformGravitationalField` — spatial
entities and a physical field, not measurable quantities. The grammar merges
what the emitter re-splits (same shape as ExternalForce). Category error:
entity/field vs quantity.
→ Distinct `WorldEntity` vs `WorldQuantity` rules.

## 3. Quantity kind fragmented by *declaration site*, not by *what it is*
Same kind spread across three enums: `QuantityType` (scalars,
`motion_spec.tx:372`), `SnapshotQuantityType`
(`Pose|Position|Orientation|VelocityTwist|AccelerationTwist|Wrench`, `:321`),
and inline `ContextQuantityType` strings. `Pose`/`VelocityTwist`/`Wrench` appear
in **both** `WorldQuantityType` and `SnapshotQuantityType`. `domain.QuantityType`
is the **union** of all — proof it's one vocabulary sliced by site.
`SnapshotQuantityType` exists only because composites were omitted from the
grammar's `QuantityType`; "can be snapshotted" is not a kind distinction —
snapshot-ness is already correctly a *value provenance* (`SnapshotValue` +
`on task|entry` clock).
→ One `QuantityKind` vocabulary; drop `SnapshotQuantityType`.

## 4. Synonym kinds — one dimension spelled several ways  (archetype: BareScalar dup)
`Angle` / `PlaneAngle` / `AngularDistance` = one dimension (plane angle);
`Distance` / `LinearDistance` = one (length). The layer betrays it:
`LinearDistance`→`Distance` (`domain.py:744`), `PlaneAngle`→`Angle`
(`CSTR_TYPE_NAME`, `rdf.py:260`) collapsed at parse/emit. `AngularDistance` is
collapsed *nowhere* and is absent from `SCALAR_UNIT` → emits `UNITLESS` instead
of `RAD` (latent unit bug).
→ One canonical name per dimension; others are explicit aliases or removed.

## 5. Two grammars for one operation: "project a quantity onto subspace/axis"
`View` (`motion_spec.tx:473`, `<worldquantity>.subspace.axis`) and `ContextRef`
(`:507`, `<contextquantity>.subspace.axis`) are near-identical selector grammars,
split solely by World-vs-Context referent. Same duplication in the axis
taxonomy: type-scoped `LinearAxis`/`AngularAxis` (`:180`,`196`) coexist with a
merged flat `Axis` (`:501`); the flat `Axis`/`SubSpace` enums aren't kind-scoped,
so `.position.roll` / `.pose.torque` are well-formed nonsense the type layer must
reject after the fact.
→ One selector concept; one kind-scoped axis vocabulary.

## 6. `elapsed` — a Duration smuggled in as a boolean View
`is_elapsed?="elapsed"` is an alternative inside `View` (`motion_spec.tx:481`).
Elapsed-since-entry is a real `Duration` quantity; modeling it as a magic
boolean branch of the selector is a category workaround. (`distance between
<A> and <B>`, `:474`, is the same shape but reads as sugar for a computed scalar.)
→ Model elapsed time as a proper time quantity.

---

## 7. `ExternalForce` / `ExternalForceMagnitude` — a Wrench in disguise  (FIXING FIRST)
Separate `WorldQuantityType`s (`motion_spec.tx:290-291`) typed with invented
`mj:` coordinates (`rdf.py:182,194`). But an FT reading **is** a wrench; the
only real differences are **role/source** ("Measured from ft-sensor X", already
expressible — `Wrench` accepts `ft-sensor`/`deadband` GeoPropKeys) and a
**subspace restriction** (force only — `Wrench` already has a `.force` view).
`ExternalForce` also structurally discards torque (lossy). `ExternalForceMagnitude`
is `norm(.force)` — a derived scalar, not a third kind.
→ Model as a **Measured `Wrench` carrying an ft-sensor source**; force vector =
`.force` view; magnitude = derived scalar.

## 8. `BareScalar` vs `ScalarQuantity` — one concept, two dataclasses  (FIXING FIRST)
`BareScalar: value=FLOAT unit=Unit` (`motion_spec.tx:512`) and
`ScalarQuantity: "=" value=FLOAT unit=Unit` (`:350`) have byte-identical fields
(`value, unit, parent`). The only difference is grammatical position (named-decl
value vs anonymous inline literal). Two representations of "a dimensioned scalar
literal."
→ One literal value type reused everywhere; the leading `=` handled at the decl
level.

---

_Consolidation sequence (when tackled fully): unify the kind vocabulary (#3/#4) →
split entities (#2) and operators (#1) off the kind axis → dedup the selector/axis
grammars (#5) → fold elapsed into a quantity (#6). #7 and #8 are being done first
as self-contained instances of #1-class and #4-class defects._
