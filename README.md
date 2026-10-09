# PMWeather-IV 0.12.0

Standalone Immersive Vehicles addon connecting ground vehicles and aircraft to PMWeather Aeronautics and persistent Sable/Rapier physics.

Ground vehicles share aircraft body collision, terrain recovery and integrated True Impact handling. IV retains drivetrain, controls and content-pack tire coefficients; road drive/braking acts through physical load-bounded tire contacts. Eligible road wheels and treads use a bounded series tire/suspension travel model unless the pack already authors vertical support motion. Compatibility follows authored features and geometry. Helicopters retain the assisted controller.

See [release notes](docs/RELEASE-0.12.0.md), [architecture](docs/ARCHITECTURE.md) and [notices](THIRD_PARTY_NOTICES.md) for behavior, supported modes and limitations.

## Build

Use Java 21 and obtain external compile inputs described in `libs/README.txt` and `libs/DEPENDENCIES.json`. Run `./gradlew jar check` (or `gradlew.bat jar check`) for the public build. Numerical checks exercise production helpers; they do not establish Minecraft integration or calibrated handling across every pack.

The public artifact contains gameplay physics, state synchronization and a no-op observer boundary. Development-only tracing, log writers, profiling, diagnostic networking and telemetry HUDs are excluded. The user-facing wind display and commands are documented below.

## Wind display and weather test

With PMWeather Aeronautics installed, use `/pmiv wind` for a reading and `/pmiv wind live on|off` to toggle PMIV's compact live wind display. While seated in an IV aircraft, the display uses the aircraft's heading. `/pmiv weather test start` starts the short flight sequence; `stress`, `status`, `next`, `stop` and `list` are available under `/pmiv weather test`.

The same features are available through `/aerowind wind`, `/aerowind live on|off` and `/aerowind test ...`. PMIV forwards its weather-test commands to PMAero, which runs the test and supplies the authoritative test wind. The test commands require operator command permission on the server. The default sequence takes about three minutes and the complete test is capped at five minutes.

## Runtime dependencies

Minecraft 1.21.1, NeoForge, Immersive Vehicles, PMWeather, Sable and PMWeather Aeronautics 1.0 or later. Install dependency mods separately. Source licensing does not grant redistribution rights for dependency mods or content-pack assets.
