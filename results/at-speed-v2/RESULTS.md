# At-speed braking: 66 Webots executions

**Both policies pass 27/33 runs and record six obstacle contacts.** The stopping-envelope policy increases stopping clearance in several cases; it does not improve the pass count or eliminate short-gap failures. These are 11 designed conditions × three physics seeds × two policies, not 66 independent traffic scenarios.

![Every condition and worst collision-hull gap](at-speed-results.png)

## What was actually tested

The ego vehicle first reaches measured cruise within 5% of 10 or 15 m/s for 20 consecutive 32 ms steps. A supervisor then introduces a scripted box target at a declared reference gap. Tests include stationary, moving and decelerating targets, short gaps, contact-friction parameters 0.9/0.35, a 256 ms range delay, a 200 ms dropout and Gaussian range noise with SD 0.5 m. Every case and failure is retained.

The physical ego vehicle is simulated by Webots. Targets are kinematic boxes repositioned by the supervisor, not validated lead-vehicle dynamics or a full lane-change model. All hazards appear after cruise; the sudden-appearance case does not represent a continuous cut-in trajectory. Friction changes are parameters, not calibrated wet-road conditions. Sensor faults are injected in software after the hazard event.

## Frozen comparison

The common frontend recognizes no-hit sensor range (120 m), uses the existing bounded startup hold and provides only measured range/speed to the policies. The adapted legacy TTC comparator uses the released sticky TTC decision and 0.4 normal brake. It is not byte-identical to the historical 40 m-range experiment. The new conservative envelope applies full brake at distance <= 2 m + speed × 0.35 s + speed²/(2 × 3 m/s²); it also latches full braking on invalid or older-than-160-ms range. The 3 m/s² value is a design assumption, not a friction measurement. Neither policy receives supervisor ground truth, object speed or future motion.

Four development cases on seed 97 checked the harness before this family was frozen. The evaluation settings and executed code hashes are in [protocol.json](protocol.json). The earlier development source and raw records are separately retained. No parameters were changed after inspecting the 66 evaluation outcomes.

## Results

Acceptance requires verified cruise, no obstacle contact, a stop below 0.1 m/s and at least 1 m signed collision-hull X clearance. A negative clearance is overlap. Contact is also read directly from simulator contact points.

| Condition | TTC pass | Envelope pass | TTC worst hull gap | Envelope worst hull gap |
| --- | ---: | ---: | ---: | ---: |
| Stationary · 10 m/s / 30 m | 3/3 | 3/3 | 9.2308 m | 10.8677 m |
| Stationary · 15 m/s / 45 m | 3/3 | 3/3 | 2.2960 m | 15.3918 m |
| Lower friction · 15 / 45 | 3/3 | 3/3 | 2.2174 m | 2.8437 m |
| Short gap · 15 / 18 | 0/3 | 0/3 | -0.1526 m | -0.1526 m |
| Severe stress · 15 / 9 | 0/3 | 0/3 | -0.4374 m | -0.4374 m |
| Moving lead target · 15 / 30 | 3/3 | 3/3 | 17.5327 m | 21.3112 m |
| Braking lead target · 15 / 30 | 3/3 | 3/3 | 18.3030 m | 24.8368 m |
| Sudden target · 10 / 22 | 3/3 | 3/3 | 2.3298 m | 8.9476 m |
| Range delayed by 256 ms | 3/3 | 3/3 | 1.5768 m | 20.6717 m |
| Range dropout · 200 ms | 3/3 | 3/3 | 18.3535 m | 17.3117 m |
| Noisy range · SD 0.5 m | 3/3 | 3/3 | 2.7075 m | 15.8718 m |

The 18 m reference-gap and 9 m severe-stress cases contact the obstacle for both policies in all three repeats. Excluding the declared severe-stress condition still leaves only 27/30 passes per policy. This small campaign does not establish a collision-free operating range. The envelope also leaves less clearance than TTC in the dropout condition, so a universal advantage is not supported.

## Measurement and replay

The separate supervisor records physical position, orientation, velocity and obstacle contact. Collision-hull extent is computed from the pinned R2025a LincolnMKZ bounding boxes and the actual car orientation; it is distinct from reference-point distance and rendered bodywork. Upstream Cyberbotics asset terms remain applicable. The older nine-run matrix reported reference distances and is kept separately.

Webots applies supervisor translation commands on the next physics step. The event timestamp is the command-issue time; the first active row still measures the hidden target at X=10000, and the target appears in the next row. Actual measured reference gaps are therefore slightly smaller than requested. The replay explicitly checks that 32 ms queue alignment, the subsequent target trajectory, all orientations/clearances, summary aggregates and every verdict. No commanded pose is substituted for a measured pose.

**All 66 runs / 22,416 telemetry rows pass the numerical audit.** This checks consistency of the saved evidence; it is not an independent physical reproduction or a simulator-accuracy guarantee. Three physics seeds produce identical outcomes in most deterministic cases; only the noise condition introduces the declared additional range randomness.

- `matrix.json`: every execution, configuration, summary, runtime hash and measured outcome.
- `case-artifacts-*.zip`: lossless per-case CSVs, summaries, generated worlds/configurations and runtime logs.
- `executed-source.zip`: exact code and original world used to generate the campaign.
- `audit.json`, `summary.json`, `artifacts.sha256.json`: replay result, figure values and artifact fingerprints.
- `development-*`: separate four-case development records and earlier harness.
- `hardware.json`: local Apple M2 Max, 12 logical cores, 96 GiB memory; no paid compute.

## Reproduce

Use the official R2025b nightly_16_10_2026 runtime and download/hash recorded in [the earlier runtime protocol](../webots-physics-v1/protocol.json). The installed application was preserved and the verified runtime was mounted read-only. This is an official prerelease. No simulator or asset binary is redistributed.

```sh
python -m unittest discover -s tests -q
python scripts/audit_robust_matrix.py --results results/at-speed-v2
WEBOTS_VERSION=R2025b-nightly_16_10_2026 python scripts/robust_matrix.py \
  --out /tmp/new-at-speed-matrix --webots /path/to/Webots.app/Contents/MacOS/webots
```

Regenerate the comparison figures with `python scripts/plot_robust_results.py` in a separate environment using `matplotlib==3.8.2` and `numpy==1.26.3`. Plotting does not rerun physics.

Use a new output directory. The runner refuses to overwrite existing trials. Fresh simulation may vary by runtime/platform. This evaluates controlled stop responses, not adaptive cruising, traffic behavior, perception, human comfort, real vehicles or certification.
