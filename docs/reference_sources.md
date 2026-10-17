# Reference Sources

All syntax in this project was adapted from official files shipped inside the installed
`/Applications/Webots.app` (R2025a) and the matching official Cyberbotics GitHub repository
at tag `R2025a`. Nothing here was invented from memory. The bundled app was inspected, not modified.

## Files inspected

| Reference | Path | What it provided |
|---|---|---|
| `vehicle_driver.py` | `/Applications/Webots.app/Contents/projects/vehicles/controllers/vehicle_driver/vehicle_driver.py` | Minimal `vehicle.Driver` usage: `Driver()`, `setCruisingSpeed()`, `setSteeringAngle()`, `driver.step()` loop |
| `highway_overtake.py` | `/Applications/Webots.app/Contents/projects/vehicles/controllers/highway_overtake/highway_overtake.py` | Full Driver control pattern: `getDevice()`, `DistanceSensor.getValue()`/`getMaxValue()`, combining `setCruisingSpeed()` with `setBrakeIntensity()` to actually decelerate, `driver.getCurrentSpeed()` |
| `highway_overtake.wbt` | `/Applications/Webots.app/Contents/projects/vehicles/worlds/highway_overtake.wbt` | World-file syntax: `EXTERNPROTO` declarations pinned to `R2025a`, `StraightRoadSegment`/`RoadLine` fields in practice, `LincolnMKZ` vehicle node with `controller` field and `sensorsSlotFront`/`sensorsSlotCenter` device slots, `DistanceSensor` with `lookupTable`/`numberOfRays`/`aperture`, a nested `Robot { supervisor TRUE }` pattern for an independent measurement controller, `GPS` device, `Viewpoint` with `follow` |
| `StraightRoadSegment.proto` (source) | fetched from `https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/road/protos/StraightRoadSegment.proto` | Exact field names/defaults: `width`, `numberOfLanes`, `numberOfForwardLanes`, `lines` (`RoadLine`), `rightBarrier`/`leftBarrier`, `length`, default orientation (road extends along local +X from `translation` when `rotation` angle is 0) |
| `LincolnMKZ.proto` (source) | fetched from `https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/vehicles/protos/lincoln/LincolnMKZ.proto` | Exact field names: `translation` (default z=0.4), `rotation` (default `0 0 1 0`, facing +X), `controller`, `supervisor`, `sensorsSlotFront/Rear/Top/Center` |
| `driver.py` (Python API source) | `/Applications/Webots.app/Contents/lib/controller/python/vehicle/driver.py` | Confirmed `Driver` is a subclass of `Supervisor`; exact method set: `setCruisingSpeed`, `setSteeringAngle`, `setBrakeIntensity`, `setThrottle`, `getCurrentSpeed`, `getTargetCruisingSpeed`, `step()`. Confirmed the Python `controller`/`vehicle` packages are pure-Python + `ctypes` bindings (no per-Python-version compiled extension), so they are not tied to one specific CPython minor version. |
| Webots stable release page | `https://github.com/cyberbotics/webots/releases/tag/R2025a` | R2025a remains the stable release; the original world and asset references use this tag. |

## macOS 27 runtime follow-up, 17 October 2026

Stable R2025a's localhost connection probe falsely reports occupied ports on macOS 27.
This was reproduced both in Webots and in an isolated diagnostic using its bundled QtNetwork.
Cyberbotics [issue #6976](https://github.com/cyberbotics/webots/issues/6976) and merged
[fix #7032](https://github.com/cyberbotics/webots/pull/7032) document the same defect.
Fresh physics uses the official [nightly_16_10_2026 R2025b build](https://github.com/cyberbotics/webots/releases/tag/nightly_16_10_2026),
mounted read-only alongside the existing installed app. No binary was patched. This is a
prerelease runtime, explicitly recorded in the [experiment protocol](../results/webots-physics-v1/protocol.json).
The world still references the original version-pinned R2025a assets.

## Key decisions this drove

- **Vehicle PROTO**: `LincolnMKZ` — the same vehicle used in the official `highway_overtake.wbt`, already proven to work with `vehicle.Driver`, a front `DistanceSensor`, and a `controller` field set directly on the PROTO instance.
- **Road**: `StraightRoadSegment` + one `RoadLine` — identical pattern to the official world, simplified to a single straight segment instead of a full highway network.
- **EXTERNPROTO strategy**: the officially bundled worlds (including this app's own shipped examples) declare `EXTERNPROTO` via versioned `raw.githubusercontent.com/.../R2025a/...` URLs rather than a local `protos/` copy — the installed app does not ship a local `projects/vehicles/protos` tree. This is Cyberbotics' standard mechanism: on first load, Webots fetches and caches each PROTO file pinned to the exact release tag (`R2025a`), guaranteeing version compatibility; subsequent loads reuse the local cache and require no internet. This project's world file follows that same official pattern.
- **Front sensor**: a single `DistanceSensor` node (built-in Webots device type, not a PROTO) mounted in `sensorsSlotFront`, using the same `lookupTable [0 0 0, R R 0]` linear-mapping pattern seen in `highway_overtake.wbt`'s `distance sensor front`, so `getValue()` returns true metres up to `getMaxValue()`.
- **Independent supervisor**: modeled on the nested `Robot { controller "radar_target_tracker" supervisor TRUE }` pattern in `highway_overtake.wbt`, but implemented as its own top-level `Robot` node (not nested inside the car) so it has no access to the car's own sensor readings and must independently query ground-truth node positions via the Supervisor API.
