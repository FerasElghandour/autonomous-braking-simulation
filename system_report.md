# System Report

## Phase 1 — Mac Inspection (initial)

| Item | Value |
|---|---|
| macOS | 26.5.1 (Build 25F80) |
| CPU Architecture | arm64 (Apple Silicon — Apple M2 Max) |
| Disk free (/) | ~110 GiB free |
| Memory | 96 GB |
| Shell | /bin/zsh |
| Git | 2.53.0 |
| Homebrew | 6.0.9, installed at /opt/homebrew/bin/brew |
| Python 3 | 3.14.3 at /opt/homebrew/bin/python3 |
| pip3 | present at /opt/homebrew/bin/pip3 |
| Webots installed? | No — `/Applications/Webots.app` not found |
| Working directory | ~/autonomy-lab |

## Decisions

- Apple Silicon confirmed (arm64, M2 Max) — will download the Apple Silicon (ARM64) Webots DMG.
- Homebrew and Python 3 already present — no need to install Homebrew.
- Python 3.14.3 is very new; Webots' bundled `controller` Python API has historically pinned to specific CPython minor versions. This will be verified in Phase 3/4 against the official Webots R2025a release notes before relying on system Python for the controller. If incompatible, an isolated Python version will be installed via Homebrew for the project virtualenv only (not touching system Python).
- No existing Webots install to reuse — will proceed with a fresh official install to `/Applications/Webots.app`.

## Phase 2 — Python Environment

- System Python was 3.14.3 (very new, untested against Webots R2025a's controller bindings). Homebrew already had `python@3.11` installed, so the project venv was built on that instead, to avoid compatibility risk without touching system Python.
- Virtualenv created at `~/autonomy-lab/.venv` using `/opt/homebrew/opt/python@3.11/bin/python3.11` → Python 3.11.15.

## Phase 3 — Webots Installation

| Item | Value |
|---|---|
| Release used | R2025a (latest stable, non-prerelease as of 2026-07-14; only newer items on GitHub were nightly pre-releases, correctly rejected) |
| Source | https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a.dmg (official Cyberbotics GitHub releases) |
| Downloaded to | `~/autonomy-lab/.downloads/webots-R2025a.dmg` (102 MB / 107,119,977 bytes) |
| SHA-256 (computed, no official checksum published for this asset) | `484f6cc84ca794dd33e410b0bfc030132cc5b163a882c350c1d293d06d87584f` |
| Installed to | `/Applications/Webots.app` |
| Bundle version | R2025a (`CFBundleShortVersionString` / `CFBundleVersion`) |
| Executable | `/Applications/Webots.app/Contents/MacOS/webots` (universal binary: x86_64 + arm64) |
| CLI controller helper | `/Applications/Webots.app/Contents/MacOS/webots-controller` |
| Gatekeeper | No quarantine attribute was present (curl-downloaded, not browser-downloaded) — app launched cleanly with no first-launch security dialog. Quarantine attribute was never present, so none was removed. |
| Launch verification | `open -a /Applications/Webots.app` → process confirmed running via `ps`/System Events, then closed (empty new-document session, nothing to save) |
