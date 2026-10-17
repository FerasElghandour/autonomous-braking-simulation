# Braking stress validation

The actual TTC state/command functions are exercised in a declared one-dimensional software-in-the-loop plant. This is separate from Webots and the retained historical physics run.

1,836 seeded runs cover 378 scenario/policy conditions: three initial speeds, three clearances, three obstacle velocities, seven input/friction conditions and two policies. Deterministic conditions use one seed; noise/dropout/combined conditions use ten.

| Condition | TTC collisions / runs | Fixed-distance collisions / runs |
|---|---:|---:|
| nominal | 3/27 | 3/27 |
| noise | 29/270 | 30/270 |
| positive_bias | 9/27 | 6/27 |
| dropout | 23/270 | 13/270 |
| latency | 6/27 | 5/27 |
| low_friction | 19/27 | 15/27 |
| combined | 173/270 | 150/270 |

The TTC policy is not robust across this stress envelope and does not consistently beat the fixed-distance baseline. Retain the collisions. The controller still assumes a static obstacle; incoming/moving obstacles, warm cruise starts, bias, latency and reduced braking are deliberate stress inputs. Results depend on the stated idealized point-mass plant and cannot be translated into real collision probabilities or Webots validation. Episode observation ends at first stop/collision or 20 seconds; no indefinite post-stop safety is inferred. A horizon timeout means the policy did not stop within the window, not necessarily a collision.

## Actual guard correction

Invalid speed readings previously produced a sanitized zero but did not enter the fail-safe decision. Invalid observations at rest could also permit acceleration, and the stopped-state override could release the fault brake. The live controller loop now treats invalid distance or speed as a fault, keeps zero commanded speed and full braking, and the supervisor flags invalid speed telemetry. A regression test executes the actual main loop with fake Webots device inputs.

## Fresh Webots status

Nine separate speed/clearance projects were generated, with world geometry checked against each config. The installed Webots runtime failed before loading the first world: it reported every attempted local controller port as already in use. A different port range, direct local networking and the Rosetta route did not resolve startup. One final matrix case was attempted; eight remain prepared and unexecuted. No new physics result is claimed. The historical `results/run.csv` and `results/summary.json` remain unchanged.

```sh
python -m unittest discover -s tests -q
python scripts/stress_validation.py --out new-stress
python scripts/webots_matrix.py --out new-webots-matrix --webots /path/to/webots
```

[Declared plant and cases](protocol.json) · [Every run](cases.csv) · [Summary](summary.json) · [Runtime attempt](../webots-matrix-v1/matrix.json)

The installed Webots executable and checked Qt network/core runtimes match the official R2025a disk image byte-for-byte. The installed app and its settings were not modified. [Integrity record](../webots-matrix-v1/runtime-integrity.json).
