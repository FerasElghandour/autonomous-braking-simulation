# Current validation extension

86 controller/validation unit tests passed on the M2 Max, Python 3.14.3. Nine fresh Webots matrix cases passed, with no obstacle contact and 11.05–15.83 m minimum reference distance. The same nine configurations also passed a same-seed repeat check. These are deterministic runs starting from rest, not independent safety trials or braking tests initialized at the requested cruise speeds.

Stable R2025a still fails at startup on macOS 27. The official R2025b nightly_16_10_2026 resolves that upstream bug; it was mounted read-only without replacing the installed app. The first nine physics executions exposed a startup speed-estimator issue and all failed without driving. Those failures are retained beside the corrected runs.

The controller now holds zero cruise/full brake for at most one second while waiting for three consecutive valid samples. After initialization, invalid speed/range readings latch full braking until restart. Five new startup/fault tests and five numerical audit tests cover these behaviors and reject corrupted evidence. The independent telemetry replay checks 27 recorded runs and 15,514 rows with no discrepancies. [Exact protocol, source and results](results/webots-physics-v1/RESULTS.md).

The earlier 1,836 software-model stress runs remain separate, use their recorded earlier source version, and include failures. See [results and limits](results/stress-v1/RESULTS.md).

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
