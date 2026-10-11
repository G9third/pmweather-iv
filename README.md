# PMWeather-IV 0.12

Standalone Immersive Vehicles addon connecting ground vehicles and aircraft to PMWeather Aeronautics and persistent Sable/Rapier physics.

Ground vehicles share aircraft body collision, terrain recovery and integrated True Impact handling. IV retains drivetrain, controls and content-pack tire coefficients; road drive/braking acts through physical load-bounded tire contacts. Wheel positions and suspension animations come from the content pack. PMIV adds no suspension travel; wind-driven body roll and tipping follow physical loads at those authored contact points. Compatibility follows authored features and geometry. Helicopters retain the assisted controller.

Road tires retain ordinary pack grip. On grass and loose terrain, excessive weather loading gradually enables a lower sliding-grip limit using actual wheel support. Drive and brakes share that contact budget. Pavement coefficients and airborne aerodynamics are unchanged. This compatibility correction preserves normal handling; its generic soil limits are approximations rather than measurements for every pack tire.

Road wind breakaway limits differ for grass, dirt/clay, gravel, sand, snow and ice, including wet surfaces. These limits are capped by the pack's own tire coefficients and take effect only when tangential weather force exceeds the terrain's available support-based grip. The transition reaches full sliding correction at 10% overload and retains the existing time smoothing. IV material categories and block slipperiness determine the underlying contact properties; Sable friction datapacks do not currently configure PMIV tire grip.

IV particle wind preserves stationary surface effects such as tire marks, bullet decals and water ripples, plus stationary fixed blast effects and brief flashes. Authored movement and orientation determine eligibility; smoke remains eligible for wind even when a pack caps its native velocity to zero. Attached effects, streaks, trails and fluid-bound particles retain their existing exclusions. Classification is cached once per particle, and eligible particles share PMAero's bounded wind cache.

Auto trim is off by default. Press **Right Alt** to enable it for the current supported
fixed-wing aircraft; press it again to turn it off. The entire auto-trim HUD is hidden
while auto trim is off, except for the brief enable/disable key hint shown on
entering a supported pilot seat. When enabled, the compact instrument label shows
live trim angle and status. The choice is stored per
aircraft and persists through dismount and save/reload. See Controls to rebind the key.
There is no global auto-trim config toggle.

The displayed angle is live server trim, not a target. Auto trim uses IV's native 0.1-degree manual increment as its maximum and respects slower linear held-button trim controls identified in the pack. Authored surface animation timing remains unchanged; unsupported custom actuator definitions cannot be inferred automatically.

Please report content-pack incompatibilities or unexpected behavior across cars, tanks, APCs, planes and helicopters at [PMWeather-IV Issues](https://github.com/G9third/pmweather-iv/issues). Include the pack name and version, mod versions, and what happened. A development trace is optional.

The public config keeps player-facing switches for general physics, the wind-shear warning threshold and terrain block breaking. Auto trim is enabled per aircraft with the key and has no global config toggle. Internal aerodynamic coefficients and model sampling budgets now use the reviewed defaults in code. Older config files may retain those retired entries, but PMIV no longer reads them. The existing `terrainDamage.enableBlockBreaking` setting still controls crash terrain removal and continues to require IV's own global block-breaking setting.

See the [0.12 release notes](docs/RELEASE-0.12.md), [previous release notes](docs/RELEASE-0.11.53.md), [architecture](docs/ARCHITECTURE.md) and [notices](THIRD_PARTY_NOTICES.md) for behavior, supported modes and limitations.

## Wind display and weather test

These public commands use the same `/pmaero` prefix with PMWeather Aeronautics alone or with PMWeather-IV installed. The compact wind HUD uses the vehicle's heading while riding and remains visible with chat and ordinary screens open. F1 hides it with the rest of the HUD.

With PMIV installed, the shared live wind HUD appears automatically only while occupying an IV aircraft pilot seat, including helicopter pilot seats, and disappears when leaving it. Disable `display.autoLiveWindInPilotSeat` in `config/pmweather_iv-client.toml` to use manual commands only. `/pmaero live on` shows the HUD anywhere until manually disabled; `/pmaero live off` hides it, including for the current pilot seat. Entering a pilot seat again restores automatic display.

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

Cars, planes and wheel-equipped helicopters share terrain contact normals and load-bounded
tire reactions. Authored tire coefficients stay intact. Wheel edge support is approximated
from the pack's radius; it does not fit an artificial slope across terrain blocks.
See [compatibility evidence](docs/COMPATIBILITY-COVERAGE.md) and
[current changes](docs/AUDIT-FIXES-20261009.md).

Current compilation and packaging succeeded. Live contact and flight behavior still
needs fresh trace evidence; earlier numerical assertion counts are historical.
