# PMWeather-IV 0.12.0

Standalone Immersive Vehicles addon connecting ground vehicles and aircraft to PMWeather Aeronautics and persistent Sable/Rapier physics.

Ground vehicles share aircraft body collision, terrain recovery and integrated True Impact handling. IV retains drivetrain, controls and content-pack tire coefficients; road drive/braking acts through physical load-bounded tire contacts. Eligible road wheels and treads use a bounded series tire/suspension travel model unless the pack already authors vertical support motion. Compatibility follows authored features and geometry. Helicopters retain the assisted controller.

See [release notes](docs/RELEASE-0.12.0.md), [architecture](docs/ARCHITECTURE.md) and [notices](THIRD_PARTY_NOTICES.md) for behavior, supported modes and limitations.

## Build

Use Java 21 and obtain external compile inputs described in `libs/README.txt` and `libs/DEPENDENCIES.json`. Run `./gradlew jar check` (or `gradlew.bat jar check`) for the public build. Numerical checks exercise production helpers; they do not establish Minecraft integration or calibrated handling across every pack.

The public artifact contains gameplay physics, state synchronization and a no-op observer boundary. Development writers, tracing, commands, diagnostic networking and HUD implementations are excluded. A separate private development source set can bind the same core to an observer without changing physics authority.

## Runtime dependencies

Minecraft 1.21.1, NeoForge, Immersive Vehicles, PMWeather, Sable and PMWeather Aeronautics 1.0 or later. Install dependency mods separately. Source licensing does not grant redistribution rights for dependency mods or content-pack assets.
