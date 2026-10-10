# PMWeather-IV 0.12.0-rc1

Standalone Immersive Vehicles addon connecting ground vehicles and aircraft to PMWeather Aeronautics and persistent Sable/Rapier physics.

Ground vehicles share aircraft body collision, terrain recovery and integrated True Impact handling. IV retains drivetrain, controls and content-pack tire coefficients; road drive/braking acts through physical load-bounded tire contacts. Eligible road wheels and treads use a bounded series tire/suspension travel model unless the pack already authors vertical support motion. Compatibility follows authored features and geometry. Helicopters retain the assisted controller.

Press **Right Alt** to toggle the auto-trim panel; its corner badge shows trim state and the current trim value.

See [previous release notes](docs/RELEASE-0.11.53.md), [architecture](docs/ARCHITECTURE.md) and [notices](THIRD_PARTY_NOTICES.md) for behavior, supported modes and limitations.

## Wind display and weather test

With PMWeather Aeronautics installed, `/pmaero` provides the wind and weather-test commands. When PMWeather-IV is installed, it also registers `/pmiv` aliases. The compact wind HUD uses an aircraft's heading while riding and remains visible with chat and ordinary screens open. F1 hides it with the rest of the HUD.

| Command | Action |
| --- | --- |
| `/pmaero wind` | Print one server-sampled wind reading. |
| `/pmaero live [on|off]` and `/pmaero wind live [on|off]` | Show the live wind HUD, show its status, or turn it off. |
| `/pmiv wind`, `/pmiv live [on|off]`, and `/pmiv wind live [on|off]` | PMIV client aliases for the same wind commands. |
| `/aerowind wind`, `/aerowind live [on|off]`, `/pmweatheriv wind`, `/pmweatheriv wind live [on|off]` | Legacy PMIV aliases. |
| `/pmaero test start [seconds]` | Start the repeatable weather sequence, optionally setting phase length in seconds. |
| `/pmaero test stress [seconds]` | Start the stress sequence, optionally setting its duration in seconds. |
| `/pmaero test stop`, `/pmaero test status`, `/pmaero test next` | Stop, inspect, or advance the active sequence. |
| `/pmaero test list [stress]` | List the regular phases or the stress sequence. |
| `/pmiv test ...` | PMIV alias for the server weather-test commands. |

The legacy `/pmiv weather test ...` and `/aerowind test ...` routes remain available where registered. Test commands require operator permission on the server. The default flight sequence runs 16 phases for 15 seconds each (4 minutes); the full test is capped at 5 minutes. PMAero supplies the authoritative test wind.

The private development artifact adds operator-only `/pmiv trace ...` and `/pmiv probe ...` commands. `/pmivtrace` and `/pmivtest` remain aliases. These commands are not included in the public artifact.

[Watch full weather test cycle](https://streamable.com/fomjry)

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
