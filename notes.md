# Notes

## JSON-LD `@graph` canonicalization (`registration.py:_canonicalize_jsonld`)

**Required — do not remove.** rdflib's JSON-LD serializer emits `@graph` nodes (and
within-node arrays) in hash-seeded set order, so the raw output is non-deterministic:
serializing the *same* graph under `PYTHONHASHSEED=1` vs `7` gives completely different
byte content and node order. Without canonicalization the generated model graph — and
everything derived from it (IR, generated C++, provenance, all hashes) — would differ
run-to-run and machine-to-machine for identical input, breaking reproducible builds,
provenance integrity, caching, and git diffs. The Makefile's `PYTHONHASHSEED=0` is only a
weaker backup (stable within one Python version); the sort is the robust primary mechanism.

Two steps:
1. Sort `@graph` by `@id` — correct/safe (every node has a unique IRI, no blank nodes).
2. `_sort_lists`: recursively sort every array — needed because rdflib also emits
   multi-valued properties / `@type` in set order.

**Latent footgun in `_sort_lists`:** it sorts *every* array unconditionally, justified by
"JSON-LD arrays are unordered sets." That's only true for `@set` arrays. An **ordered**
array (`@list`) serializes as `{"@list": [a,b,c]}`, and `_sort_lists` would recurse in and
reorder the elements — silently corrupting an ordered collection (trajectory waypoints,
rdf:List, inline vector). Currently harmless: the generated graph has no `@list` and no
numeric arrays (coordinates are separate `x/y/z` nodes), so nothing is corrupted. It's
guarded only by the unenforced invariant "the model never emits ordered arrays."

**If we ever emit an ordered collection:** add a one-line guard in `_sort_lists` to skip
the value under an `@list` key. No behavior change today (no `@list`s exist).
