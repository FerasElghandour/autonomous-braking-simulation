# Inputs and data handling

Public test fixtures and generated experimental artifacts are included where described. Private client records, account data, credentials, original submission archives, and unrelated business documents are excluded. Runtime data should remain outside version control.

Code imports external libraries through their normal installations; it does not vendor those libraries. Their licenses and notices remain with the installed distributions. No blanket software license is assigned to third-party material.

The controller and supervisor code are accompanied by a version-pinned Webots world. Official simulator and PROTO assets are acquired separately from Cyberbotics: https://github.com/cyberbotics/webots/tree/R2025a. Saved telemetry and the generated screenshot describe the single retained experiment.


## At-speed follow-up

`results/at-speed-v2/` retains all 66 new Webots executions. The six case archives contain lossless telemetry, summaries, generated worlds/configurations and logs. `executed-source.zip` preserves the exact harness. `development-*` preserves four earlier development cases and their earlier source separately. These records extend the historical single-world evidence; they are not additional road data or independent traffic scenarios. The ego is physical; lead targets are scripted kinematic boxes. Cyberbotics geometry and vehicle assets retain their original terms.
