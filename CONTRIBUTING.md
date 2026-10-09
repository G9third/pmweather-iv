# Contributing

Read `docs/ARCHITECTURE.md` before changing force generation, collision, or
terrain response. Use Java 21 and the included Gradle wrapper. Keep each change
focused on one behavior and explain the physical assumption or integration
problem it addresses.

## Physics rules

- Apply forces and impulses once. Sable integrates pose, momentum, gravity, and
  gyroscopic response. Material solvers return residual contact speed; they do
  not write body velocity.
- Use metres, seconds, kilograms, newtons, and radians at the physics boundary.
  Keep IV per-tick and `speedFactor` conversion in existing helpers.
- Use the actual Sable timestep. Wind queries belong to the owner-tick snapshot;
  substeps reuse that snapshot.
- Preserve contact lever arms, authored colliders, and damage ownership. Use
  centre-of-mass velocity for force evaluation and the physical lever arm for
  point velocity and torque.
- Document physical handling limits as approximations. Avoid per-aircraft IDs,
  grounded suppression, or arbitrary whole-body velocity damping as substitutes
  for a contact or authority fix.

## Build and validation

The public `main` source set contains gameplay code and the no-op observer
boundary. Private logs, trace tools, diagnostic networking, commands, and HUD
providers live in the separate `dev` source set and use ServiceLoader. Public
build and check tasks do not compile or package private sources.

Run the public checks with:

```sh
./gradlew check
```

This compiles the public sources and runs the public regressions, including
geometry, terrain, material, lifecycle, networking, and contact checks. The
geometry, terrain, and material output baselines are checked against their
reviewed SHA-256 files.

For private diagnostics and the artifact boundary, run:

```sh
./gradlew checkDev
```

This runs the private trace schema and feedback checks and builds both artifacts
to verify that private entries are absent from the public JAR, present in the
development JAR, and shared class files are byte-identical. `./gradlew jar`
builds the public artifact; `./gradlew jarDev` builds the private development
artifact.

On Windows, use `.\gradlew.bat` with the same task names. The private trace
regression also checks explicit current schema structures; its existing hashed
stdout remains a pinned historical serialization fixture. Review an intentional
output change before updating any regression digest.

These checks do not validate Minecraft world mutation, Mixin application,
Rapier/native behavior, or in-game flight handling. For changes that affect
runtime behavior, compare controlled flights/crashes and include the vehicle
or content pack, wind, inputs, Sable settings, and PMIVTrace files in the report.

## Runtime validation

- Check Bell 206 hover/control pulses and crosswind response.
- Check PZL/E500 taxi, braking, takeoff, landing, and wing/tail ground contact.
- Check glancing and head-on crashes, soft-ground penetration, indestructible
  terrain, and an already embedded/reloaded wreck.
- Review `SABLE_SUBSTEP_TERRAIN_BOUNDARY_RESOLVED` records. When
  `sweptGameplayImpactCaptured=false`, pose recovery preserves Rapier momentum;
  investigate repeated recovery, jitter, tunnelling, or repeated damage.
- Check animated/retracted gear, totaled wings, multiplayer synchronization,
  optional standalone True Impact, and non-default Sable substep counts.

## Where a change belongs

Model object filters and aerodynamic sampling belong in `ModelTriangleSampling`
and `ModelAnimationHints`; raw IV buffer decoding belongs in
`ParsedModelSnapshot`. Keep aerodynamic sampling thresholds separate from
collision shell rules and preserve transparent/damage filters.

`SurfaceClassifier` owns classification order. Add control, wing, tail, or body
rules in the corresponding classifier. Pressure patches and lifting frames
have separate construction stages.

`SableVehicleBody` owns registration, handles, loads, and pose export.
`SableTerrainResponse` owns contact recovery and clear-pose memory;
`SableTerrainSweep` queries snapshots of current body-child geometry, while
collider mounting stays in `SableCompoundCollider`.

Capture immutable values in `TraceSamples` and serialize them in `TraceJson`.
Keep queue/session/archive lifecycle in `PMIVTraceRecorder`; do not move world or
entity reads onto the writer thread. Copy applied-substep values before enqueueing.
