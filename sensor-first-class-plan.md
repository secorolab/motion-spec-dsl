# Plan — First-class Force-Torque Sensor (SOSA/SSN)

**Goal:** elevate the FT sensor from a MuJoCo-bound name (`MJ:ft-sensor` + `ft-sensor-ref`
string) to a first-class **`sosa:Sensor`** (kind ForceTorque) hosted by the robot, whose
**source-of-information is backend-specific** (sim = MuJoCo site sensor; real = Robotiq
FT300S driver, *declared-but-stubbed*, wired on the Kinova route later). The measured
`Wrench` links to it by `sosa:madeBySensor` instead of a string ref.

**Decisions (locked with user):** reuse **W3C SOSA/SSN** vocabulary; sensor is a **typed
entry on the Robot** (evolve the existing `ft-sensor:` entry in place). Real path stubbed.
No backward-compat (only `admittance_arc_single` uses a sensor).

**SOSA mapping:** robot = `sosa:Platform` `sosa:hosts` sensor · sensor = `sosa:Sensor`
`sosa:observes` a ForceTorque `sosa:ObservableProperty` · measured Wrench (the result)
`sosa:madeBySensor` sensor.

---

## Target syntax

```
Robot robot using <kinova-mjcf> {
    attach-to: <table>.site(table_top),
    sensor: wrist_ft: ForceTorque frame-site wrist_ft_site sim-source mujoco-sensor wrist_ft,
    chain: { root: base_link, end: g_pinch }
}
...
ext-force: Wrench { measured-by: wrist_ft, as-seen-by: base_link }
```
`sim-source` optional; when omitted the MuJoCo sensor name defaults to the entry name.
`real-source robotiq FT300S …` is the future slot (not in grammar yet — add on the Kinova route).

---

## Layer-by-layer

### 1. Grammar `motion_spec.tx`
- Replace `EnvironmentFtSensorEntry` →
  ```
  EnvironmentSensorEntry:
      "sensor" ":" name=IRI_TRUNK ":" type=SensorType
      "frame-site" frame_site=IRI_TRUNK
      ("sim-source" "mujoco-sensor" sim_sensor=IRI_TRUNK)?
  ;
  SensorType: "ForceTorque" ;
  ```
- Swap it into `EnvironmentAssemblyEntry`.
- `GeoPropKey`: rename `"ft-sensor"` → `"measured-by"`.

### 2. `domain.py`
- `SensorType(StrEnum){ ForceTorque }`.
- `EnvironmentSensorEntry(name, type, frame_site, sim_sensor="")` (replaces `EnvironmentFtSensorEntry`).
- `GeometricPropKey`: `FtSensor` → `MeasuredBy = "measured-by"`.
- `registration.py`: swap the class name in both LANGUAGE_CLASSES lists.

### 3. `rdf.py`
- Namespace: `SOSA = Namespace("http://www.w3.org/ns/sosa/")`; register prefix `"sosa"`.
  A secorolab `SENSOR_EXT` for the kind subclass/property individual if needed
  (`sensor-ext:ForceTorqueSensor`, `sensor-ext:ForceTorque a sosa:ObservableProperty`).
- Assembly emission (replace `EnvironmentFtSensorEntry` branch): emit
  `sensor_node a sosa:Sensor, sensor-ext:ForceTorqueSensor ; sosa:observes ft-prop`,
  `robot a sosa:Platform ; sosa:hosts sensor_node`, plus codegen metadata
  `MJ:frame-site` + `MJ:mujoco-sensor` (= sim_sensor or name). Record a name→sensor_node map.
- Wrench measured branch (~1738): read the `measured-by` prop → resolve to sensor_node →
  `wrench_node sosa:madeBySensor sensor_node` (drop the `MJ:ft-sensor-ref` literal).

### 4. `ir_gen.py` (+ `namespace.py`)
- Add `SOSA` namespace + terms; add `MJ:mujoco-sensor`; drop `MJ:ft-sensor`/`ft-sensor-ref`.
- `wrench()` parser: `sensor_name` = `g.value(g.value(wrench, SOSA.madeBySensor), MJ["mujoco-sensor"])`.
- Robot setup ft_sensors extraction (~3402): iterate `SOSA.hosts` sensors of the robot
  (was `MJ:ft-sensor` objects); read `MJ:frame-site` + `MJ:mujoco-sensor` (name for find_ft_sensor).

### 5. codegen / `module.stg`
- No change: `solver-output-Wrench-mj_kdl` already uses `<out.sensor_name>` (now the MuJoCo
  sensor name) for `find_ft_sensor`. `robif2b` stub stays the Robotiq-FT300S "later" slot.

### 6. SHACL `src/metamodels/task/sensor.shacl.ttl` (new)
- Declare `sosa:` prefix; a `sensor-ext:ForceTorqueSensor` node shape (a `sosa:Sensor`,
  has `MJ:frame-site`, `sosa:observes` an ObservableProperty). Optionally require a Measured
  Wrench to carry `sosa:madeBySensor`. Register its URL in
  `registration.py::local_constraint_paths` (same list that loads `map-extension.shacl.ttl`).

### 7. Model migration `admittance_arc_single.robmot`
- Robot: `ft-sensor: wrist_ft frame-site wrist_ft_site` →
  `sensor: wrist_ft: ForceTorque frame-site wrist_ft_site sim-source mujoco-sensor wrist_ft`.
- Wrench: `ft-sensor: wrist_ft` → `measured-by: wrist_ft`.

---

## Verification
- `pytest -q` green both repos (update any fixture using the old `EnvironmentFtSensorEntry`
  / `ft-sensor:` wrench prop).
- `make codegen MODEL=admittance_arc_single` → `Conforms: True`; generated C++ **unchanged**:
  one `find_ft_sensor(...,"wrist_ft")` per state, `get_site_frame(...,"wrist_ft_site")`.
- `pick_place_*` regression `Conforms: True` (no sensor → unaffected).
- `grep -rn "ft-sensor\|ft_sensor\|FtSensor" src/motion-spec src/motion-spec-dsl/src` →
  only SOSA/mujoco-sensor plumbing, no legacy `ft-sensor-ref`.

## Out of scope (Kinova route later)
Real `robotiq FT300S` `real-source` grammar + a hardware FT driver behind the `robif2b`
`solver-output-Wrench` stub. SOSA `sosa:Procedure`/calibration modeling if needed.
