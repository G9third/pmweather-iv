# Third-party notices

Notice updated 2026-10-11.

## True Impact

PMWeather-IV incorporates and modifies portions of the True Impact terrain-impact
implementation.

- Project: **True Impact**
- Upstream author/project owner: **OMEGAU371 / True Impact contributors**
- Upstream repository: https://github.com/OMEGAU371/sable-true-impact
- **Upstream source baseline:** True Impact **v0.5.7-delta**, as published by OMEGAU371
  (https://github.com/OMEGAU371/sable-true-impact/tree/v0.5.7-delta).
- **Local PMIV-derived variant:** **0.5.8-delta**, created by modifying the 0.5.7-delta
  source for PMIV's external rigid-body world-impact integration. This is **not an
  upstream True Impact release or version attributed to OMEGAU371**.
- **License of the upstream baseline:** LGPL-3.0-only, retained for the local
  modifications and incorporated LGPL-covered portions.

The historical local change from 0.5.7-delta to 0.5.8-delta added an
`ExternalWorldImpactApi` for contacts from bodies that were not ordinary Sable
sublevels. The separately developed API and its local version identifier must
not be confused with an upstream feature or release. The repository's source
headers using "True Impact 0.5.8-delta" refer to this **local derivative
snapshot**; original True Impact authors retain credit for their upstream
code, not for PMIV's later adaptations.

The incorporated portions are primarily the world-material response algorithms and support
classes for hardness/blast-resistance thresholds, material classification, confinement,
overburden, penetration-energy loss, projected penetration footprints, cumulative block
damage, compaction, crack overlays, and force-dependent block-drop behavior.

They have been adapted into the namespace:

`com.g9third.pmweatheriv.terrain.trueimpact`

PMWeather-IV does not claim authorship of those upstream portions. Files substantially
derived from True Impact carry an attribution header. PMWeather-IV as distributed
here is licensed under LGPL-3.0-only; the corresponding LGPL and GPL texts are included as
`COPYING.LESSER` and `COPYING`.

The standalone True Impact mod is not bundled as a JAR and is not required at runtime by
PMWeather-IV. It may be installed separately. PMIV owns Sable 2.x's
`RapierPhysicsPipeline.processCollisionEffects -> Rapier3D.clearCollisions(long)` drain at
higher mixin priority for its multi-substep vehicle capture and reflectively forwards the
complete raw collision batch to standalone True Impact's existing `SableImpactCapture` for
ordinary Sable `ServerSubLevel` bodies. Before that forwarding call, PMIV removes every raw
collision record involving a PMIV managed vehicle body ID. This avoids True Impact treating an unknown
direct vehicle body as static world terrain and keeps vehicle/world material response solely
under PMIV's integrated path.

## ProtoManly's Weather (separate runtime dependency)

- Project: **ProtoManly's Weather**
- Creator: **ProtoManly**
- Official project page: https://modrinth.com/mod/protomanlys-weather
- Distribution license identified in PMWeather 0.17.16 metadata and on Modrinth:
  **All Rights Reserved**

PMWeather-IV does not bundle PMWeather's mod JAR, resources, or implementation classes.
The PMWeather runtime dependency must be obtained separately from an authorized
distribution channel. PMWeather-IV's LGPL license does not grant permission to copy,
relicense, or redistribute ProtoManly's Weather code, assets, or JAR.

PMWeather Aeronautics is a separate MIT-licensed integration mod developed by G9third
(https://github.com/G9third/pmweather-aeronautics). PMIV consumes its exposed APIs.
PMAero's operator-controlled wind test calculates temporary synthetic wind samples
for integration testing; it does not add weather-test code to PMWeather itself or
redistribute the PMWeather JAR. This describes the integration's design, not an
assertion of permission to incorporate third-party proprietary implementation code.

## Other runtime integrations

- **Sable 2.0.5**, RyanHCode and contributors: PolyForm Shield License 1.0.0.
  https://github.com/ryanhcode/sable/blob/main/LICENSE.md
  This includes a noncompete condition and is not an MIT license. PMIV requires
  the independently installed Sable mod and uses its native physics APIs. PMIV
  does not include Sable's classes, native libraries, assets or player-contact
  implementation, and is not a replacement distribution of Sable.
- **Sable Companion 1.6.0**, RyanHCode and contributors: MIT, as identified in
  its separately supplied JAR. Its classes and JAR are not included in PMIV.
- **Create: Aeronautics, Create: Simulated and Offroad**, The Simulated Team /
  The Creators of Aeronautics: MIT for code, All Rights Reserved for the
  designated assets directories. See
  https://github.com/Creators-of-Aeronautics/Simulated-Project/blob/main/LICENSE.md.
  PMIV does not distribute these mods' code or assets. Standing-player contact
  uses PMIV's own IV-oriented-box geometry with Minecraft's native movement;
  it does not incorporate Sable's sublevel player-collision implementation.
- **Immersive Vehicles**, don_bruce and contributors: Immersive Vehicles
  General Public License, version 1 (13 January 2024).
  https://github.com/DonBruce64/MinecraftTransportSimulator/blob/master/LICENCE.md
  Its terms restrict public derivative distributions and redistribution of IV.
  PMIV is a separate integration requiring the original IV JAR; it does not
  bundle IV classes, a modified IV binary, or content-pack models and textures.

PMIV's LGPL license applies to PMIV and the incorporated LGPL-covered True
Impact portions. It does not relicense any dependency or grant permission to
redistribute third-party mods. Integration references and API hooks are not a
claim of authorship or ownership of those dependencies.

## Release source distribution

For each public PMWeather-IV release, distribute the mod JAR and its matching public source
archive from the same download location. Recipients may modify and rebuild PMWeather-IV under
the LGPL-3.0-only terms. Keep the corresponding source, this notice, and the included
`COPYING.LESSER` and `COPYING` license texts with the release distribution.

The earlier RC1 source snapshot was published at
https://github.com/G9third/pmweather-iv/commit/e68701f037fa58537327f3ce08c1866cfd5f5dbb.
It identifies the earlier source state. The 2026-10-09 audit-fix candidate is supplied with
its own matching source archive and SHA-256 manifest. Preserve that matching archive with
any distribution of this rebuilt binary; the version label alone does not identify it.

## Build tooling

The public source archive includes the Gradle wrapper, a build tool from the
Gradle project under Apache License 2.0. The matching license text is retained
in `gradle/wrapper/LICENSE`. The wrapper is not bundled in the mod JAR.
https://www.apache.org/licenses/LICENSE-2.0
