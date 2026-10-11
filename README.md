# PMWeather-IV 0.12.0-rc1

Standalone Immersive Vehicles addon connecting ground vehicles and aircraft to PMWeather Aeronautics and persistent Sable/Rapier physics.

Ground vehicles share aircraft body collision, terrain recovery and integrated True Impact handling. IV retains drivetrain, controls and content-pack tire coefficients; road drive/braking acts through physical load-bounded tire contacts. Eligible road wheels and treads use a bounded series tire/suspension travel model unless the pack already authors vertical support motion. Compatibility follows authored features and geometry. Helicopters retain the assisted controller.

Auto trim is off by default. When you enter the pilot seat of a supported fixed-wing aircraft, a brief centered popup shows the key to enable or disable it. Press **Right Alt** (default; rebind it under Controls) to toggle auto trim for that aircraft. The corner badge shows when it is adjusting elevator trim. Your choice is stored per aircraft and persists through dismount and save/reload. There is no global auto-trim config toggle.

Please report content-pack incompatibilities or unexpected behavior across cars, tanks, APCs, planes and helicopters at [PMWeather-IV Issues](https://github.com/G9third/pmweather-iv/issues). Include the pack name and version, mod versions, and what happened. A development trace is optional.

The public config keeps player-facing switches for general physics, the wind-shear warning threshold and terrain block breaking. Auto trim is toggled per aircraft in game and has no global config toggle. Internal aerodynamic coefficients and model sampling budgets use the reviewed defaults in code. Older config files may retain retired entries, but PMIV no longer reads them. The existing `terrainDamage.enableBlockBreaking` setting still controls crash terrain removal and continues to require IV's own global block-breaking setting.

See the [0.12.0-rc1 release notes](docs/RELEASE-0.12.0-rc1.md), [previous release notes](docs/RELEASE-0.11.53.md), [architecture](docs/ARCHITECTURE.md) and [notices](THIRD_PARTY_NOTICES.md) for behavior, supported modes and limitations.

## Wind display and weather test

These public commands use the same `/pmaero` prefix with PMWeather Aeronautics alone or with PMWeather-IV installed. The compact wind HUD uses the vehicle's heading while riding and remains visible with chat and ordinary screens open. F1 hides it with the rest of the HUD.

| Command | Action |
| --- | --- |
| `/pmaero wind` | Print one server-sampled wind reading. |
| `/pmaero live on` | Show the live wind HUD. |
| `/pmaero live off` | Hide the live wind HUD. |
| `/pmaero test start [secondsPerPhase]` | Start the flight sequence; optionally set seconds per phase. |
| `/pmaero test stress [secondsPerPhase]` | Start the stress sequence; optionally set seconds per phase. |
| `/pmaero test status` | Show the active test's status. |
| `/pmaero test next` | Advance to the next phase. |
| `/pmaero test stop` | Stop the active test. |
| `/pmaero test list` | List the flight sequence's phases. |
| `/pmaero test list stress` | List the stress sequence's phases. |

Test commands require operator permission on the server. The default flight sequence runs 16 phases for 15 seconds each (4 minutes); the full test is capped at 5 minutes. PMAero supplies the authoritative test wind.

The private development artifact adds operator-only `/pmiv trace ...` and `/pmiv probe ...` commands. `/pmivtrace` and `/pmivtest` remain aliases. These commands are not included in the public artifact.

[![Watch the full weather test cycle on YouTube](https://img.youtube.com/vi/pCM3C3d09k0/hqdefault.jpg)](https://www.youtube.com/watch?v=pCM3C3d09k0)

[Watch the full weather test cycle on YouTube](https://www.youtube.com/watch?v=pCM3C3d09k0)

## Build

Use Java 21 and obtain external compile inputs described in `libs/README.txt` and `libs/DEPENDENCIES.json`. Run `./gradlew clean jar check` (or `gradlew.bat clean jar check`) for a clean public build. It produces `PMWeather-IV-<version>.jar`; `check` runs public numerical checks. Run `./gradlew jarDev checkDev` for the separate `PMWeather-IV-<version>-dev.jar` and its private diagnostics and artifact-separation checks. The development JAR shares the same gameplay classes as the public JAR and adds private trace, profiling, probe commands, and diagnostic networking. Neither compilation nor numerical checks establish Minecraft runtime integration or calibrated handling across every pack.

The public artifact contains gameplay physics, state synchronization, the wind HUD, and public wind/weather-test commands, plus a no-op observer boundary. Private trace writers, profiling, probe commands, and diagnostic networking remain in the development artifact. A separate private development source set binds the same core to an observer without changing physics authority.

## Runtime dependencies

Minecraft 1.21.1, NeoForge, Immersive Vehicles, PMWeather, Sable and PMWeather Aeronautics 1.0 or later. Install dependency mods separately. Source licensing does not grant redistribution rights for dependency mods or content-pack assets.

## Current integration

This build requires the matching PMWeather Aeronautics 1.0 **wind revision 3**.
Same display versions can identify different rebuilds; use the supplied matching artifacts.
PMWeather 0.17.14 through 0.17.16 is the inspected integration family.

Cars, planes and wheel-equipped helicopters share terrain contact normals and load-bounded
tire reactions. Authored tire coefficients stay intact. Wheel edge support is approximated
from the pack's radius; it does not fit an artificial slope across terrain blocks.
See [compatibility evidence](docs/COMPATIBILITY-COVERAGE.md) and
[current changes](docs/AUDIT-FIXES-20261009.md).

Current compilation and packaging succeeded. Live contact and flight behavior still
needs fresh trace evidence; earlier numerical assertion counts are historical.
