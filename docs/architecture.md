# Architecture

## The loop

```
World -> sensor -> risk calculation -> controller -> vehicle physics -> supervisor truth -> evaluation
```

1. **World** (`worlds/obstacle_braking.wbt`): a straight paved road, one `LincolnMKZ` vehicle,
   one large static box obstacle 145 m ahead, and an independent `experiment_supervisor` robot.
2. **Sensor**: a front-facing `DistanceSensor` mounted on the car (`sensorsSlotFront`), range 0-40 m.
3. **Risk calculation**: `controllers/autonomous_car/autonomous_car.py` turns the raw sensor
   reading into a validated distance, combines it with the car's own current speed (the obstacle
   is static, so closing speed = the car's own speed), and computes time-to-collision (TTC) with
   the pure function `compute_ttc(distance, closing_speed)`.
4. **Controller decision**: `decide_state(...)` maps TTC (plus sensor validity) to one of
   `ACCELERATING`, `CRUISING`, `BRAKING_NORMAL`, `BRAKING_EMERGENCY`, `SENSOR_FAULT`, `STOPPED`.
   A preceding `INITIALIZING` hold commands zero cruise and full brake until three consecutive
   valid speed/range samples arrive within one second. Missing startup speed estimates are
   recorded, not treated as zero-speed evidence. Initialization timeout and faults after
   initialization latch `SENSOR_FAULT` and full braking until controller restart.
   Each state ramps a commanded speed target (`ramp_towards`) at the configured acceleration or
   braking rate and sets a proportional brake intensity.
5. **Vehicle physics**: the commanded speed and brake intensity are applied via the official
   `vehicle.Driver` API (`setCruisingSpeed`, `setBrakeIntensity`, `setSteeringAngle(0.0)` held
   centred throughout). Webots' ODE-based physics engine handles the actual wheel dynamics.
6. **Supervisor truth**: `controllers/experiment_supervisor/experiment_supervisor.py` is a
   separate Webots Supervisor process. It independently queries the car's and
   obstacle's true position/velocity from the physics engine, and detects physical contact
   via `Node.getContactPoints()` filtered to the obstacle's bounding box. The car's own per-step
   decision data (sensor reading, commanded speed, brake command, state), received over an
   Emitter/Receiver link, additionally checks telemetry validity and whether braking was
   commanded. Those checks do use reported controller data; the position, velocity and contact
   measurements do not. Unavailable startup samples are exempted only during a bounded,
   fully braked hold with less than 0.05 m independently measured X displacement.
7. **Evaluation**: the supervisor writes `results/run.csv` (per-step telemetry, both car-reported
   and ground-truth columns) and `results/summary.json` (aggregate pass/fail verdict with
   itemized failure reasons). `scripts/validate_results.py` then independently re-derives the
   same safety verdict from `summary.json`'s own fields, rather than only trusting the `pass`
   boolean the supervisor already computed -- a second, separate check on the same evidence.

## Process boundaries

Three independent OS processes run each simulation:

- **Webots** itself (the physics/rendering engine and world host).
- **`autonomous_car`** controller process (attached to the `LincolnMKZ` vehicle node, `supervisor`
  field `FALSE` -- it has no access to ground truth, only its own sensor and `Driver` state).
- **`experiment_supervisor`** controller process (attached to a separate `Robot` node with
  `supervisor` field `TRUE` -- it has full read access to the scene graph and physics state, but
  no access to the car's Python-side variables except what the car chooses to broadcast).

This separation keeps the physical stop and contact measurements independent of the car's
reported decisions. It is a simulation validation boundary, not a vehicle safety certification.

## Known simplifications (documented, not hidden)

- The supervisor's `true_distance_m` telemetry column treats the car as a point at its reported
  translation (ignoring the ~1-2 m front overhang of the physical vehicle body). This makes the
  telemetry distance *slightly larger* than the true clearance to the car's front bumper. The
  contact check uses contact-point geometry (`Node.getContactPoints`) against the obstacle's
  bounding box. The separate 3 m stopping-distance acceptance threshold does use this reference
  distance, so it must not be interpreted as 3 m of bumper clearance.
- `worlds/obstacle_braking.wbt` is a static, hand-authored file. `config/scenario.json`'s
  `car_start_position_m` and `obstacle_position_m` are not used to procedurally generate the
  world -- they must be kept in sync with the world file's baked-in translations by hand. This is
  documented in `scenario.json` itself and is an intentional scope limit for milestone 1.


## At-speed comparison harness

`scripts/robust_matrix.py` generates a separate family using `robust_car` and `robust_supervisor`. A 120 m no-hit-aware range frontend allows measured cruise before target introduction. Both policies then use only sensor range/speed; supervisor ground truth is reserved for measurement. The common frontend and a controlled hazard-appearance phase distinguish this comparator from the original hand-authored static world above.

The supervisor commands static, constant-speed or decelerating kinematic box targets. Translation updates reach physics on the next 32 ms step, which the raw audit reconstructs. Actual orientation and pinned vehicle bounding geometry provide signed collision-hull X clearance. Scenario noise/delay/dropouts are explicit software inputs. The new stopping envelope and stale guard are deterministic control logic; there is no trained braking model.

See [the full protocol and observed failures](../results/at-speed-v2/RESULTS.md). The original nine-case world and earlier kinematic stress model remain separate.
