# PMWeather-IV 0.12.0-rc1

Standalone Immersive Vehicles addon connecting ground vehicles and aircraft to PMWeather Aeronautics and persistent Sable/Rapier physics.

Ground vehicles share aircraft body collision, terrain recovery and integrated True Impact handling. IV retains drivetrain, controls and content-pack tire coefficients; road drive/braking acts through physical load-bounded tire contacts. Eligible road wheels and treads use a bounded series tire/suspension travel model unless the pack already authors vertical support motion. Compatibility follows authored features and geometry. Helicopters retain the assisted controller.

Press **Right Alt** to toggle the auto-trim panel; its corner badge shows trim state and the current trim value.

See [previous release notes](docs/RELEASE-0.11.53.md), [architecture](docs/ARCHITECTURE.md) and [notices](THIRD_PARTY_NOTICES.md) for behavior, supported modes and limitations.

## Wind display and weather test

With PMWeather Aeronautics installed, use `/pmiv wind` for a reading and `/pmiv wind live on|off` to toggle the compact live wind display. While riding an aircraft, the HUD uses its heading. The same commands are available under `/aerowind wind` and `/aerowind live on|off`.

Start the repeatable weather sequence with `/pmiv weather test start` or `/aerowind test start`. The `stress`, `status`, `next`, `stop` and `list` subcommands are available under `/pmiv weather test`. PMIV forwards these commands to PMAero, which supplies the authoritative test wind. Test commands require operator permission on the server. The default flight sequence runs 16 phases for 15 seconds each (4 minutes); the full test is capped at 5 minutes.

[Watch full weather test cycle](https://streamable.com/fomjry)

## Build

Use Java 21 and obtain external compile inputs described in `libs/README.txt` and `libs/DEPENDENCIES.json`. Run `./gradlew jar check` (or `gradlew.bat jar check`) for the public build. Numerical checks exercise production helpers; they do not establish Minecraft integration or calibrated handling across every pack.

The public artifact contains gameplay physics, state synchronization, the wind HUD, and wind/weather-test commands, plus a no-op observer boundary. Private trace writers, profiling, diagnostic commands and diagnostic networking remain in the development artifact. A separate private development source set binds the same core to an observer without changing physics authority.

## Runtime dependencies

Minecraft 1.21.1, NeoForge, Immersive Vehicles, PMWeather, Sable and PMWeather Aeronautics 1.0 or later. Install dependency mods separately. Source licensing does not grant redistribution rights for dependency mods or content-pack assets.
