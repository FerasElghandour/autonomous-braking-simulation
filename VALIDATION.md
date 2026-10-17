# Current validation extension

76 controller/validation unit tests passed on the M2 Max, Python 3.14.3. The new fail-safe loop and matrix geometry checks passed. The 1,836 software-in-the-loop stress runs are reported separately and include failures. Fresh Webots physics remains blocked at runtime startup. See [results and limits](results/stress-v1/RESULTS.md).

## Preserved earlier validation record

# Validation record

Release review: 17 October 2026.

**71 unit tests pass.** Independent rechecking of the saved run confirms no collision, a minimum ground-truth obstacle distance of **15.3837 m**, and final speed **0.0265 m/s**, over a **17.056 s** simulation. The physics run is historical; this release freshly checks its telemetry and controller tests.

The listed fresh checks exercise the included public edition. Historical full experiments, representative replays, and synthetic fixtures are identified separately. Test success does not certify commercial deployment.

```sh
python -m unittest discover -s tests -q
python scripts/validate_results.py
```

One straight-road, static-obstacle scenario. No physical vehicle, moving obstacle, weather robustness, or broader safety claim is established. The simulator and official vehicle assets are external dependencies.
