# Autonomous Braking Simulation

**Feras Elghandour**

Connect obstacle sensing, time-to-collision decisions, vehicle control, and independent ground-truth evaluation.

## Implementation

A Webots vehicle controller validates distance and speed readings, enters a persistent braking state, and records telemetry. A separate supervisor checks movement, stopping, collision status, and stopping margin against scenario thresholds.

```mermaid
flowchart LR
 A[Distance and speed] --> B[Validate and compute TTC]
 B --> C[Persistent braking state]
 C --> D[Vehicle physics]
 D --> E[Independent supervisor and telemetry]
```

## Evidence and results

**76 unit tests pass.** Independent rechecking of the saved run confirms no collision, a minimum ground-truth obstacle distance of **15.3837 m**, and final speed **0.0265 m/s**, over a **17.056 s** simulation. The physics run is historical; this release freshly checks its telemetry and controller tests.

![Recorded braking telemetry](https://raw.githubusercontent.com/FerasElghandour/FerasElghandour.github.io/main/assets/braking-telemetry.png)

Replotted from [results/run.csv](results/run.csv), using ground-truth speed and obstacle distance. The dashed line marks entry into the braking state.

![Actual Webots frame from the recorded simulation](results/final_screenshot.png)

## Run locally

Python 3.11 or later; use an isolated environment.

```sh
python -m unittest discover -s tests -v
python scripts/validate_results.py
```

For a new physics run, open `worlds/obstacle_braking.wbt` in Webots R2025a. The world references official version-pinned assets. The controller bindings come from Webots and should not be installed from PyPI. [Architecture](docs/architecture.md) describes the control loop.

## New validation and fail-safe guard

The current controller prevents acceleration on an invalid speed or distance reading and retains fail-safe braking. **76 tests** include an actual control-loop regression and generated-world/config consistency checks.

A separate **1,836-run kinematic stress comparison** covers speed, clearance, moving obstacles, noise, bias, dropouts, delay and friction. It exposes collisions and inconsistent performance against a fixed-distance baseline. [All results and model limits](results/stress-v1/RESULTS.md).

Nine fresh Webots cases are prepared. The installed runtime currently fails before loading a world; their physics outcomes remain unmeasured. The historical telemetry above is preserved.

## Scope and limitations

One straight-road, static-obstacle scenario. No physical vehicle, moving obstacle, weather robustness, or broader safety claim is established. The simulator and official vehicle assets are external dependencies.

[Validation record](VALIDATION.md) · [Inputs and applicable notices](DATA.md)

## Reuse

The original contributions listed in [LICENSING.md](LICENSING.md) are available under [MIT](LICENSE). This scoped grant preserves upstream notices and separate dataset, simulator, notebook and checkpoint terms.
