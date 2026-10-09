# Third-party notices

Notice updated 2026-10-09.

## True Impact

PMWeather-IV incorporates and modifies portions of the True Impact terrain-impact
implementation.

- Project: **True Impact**
- Upstream author/project owner: **OMEGAU371 / True Impact contributors**
- Upstream repository: https://github.com/OMEGAU371/sable-true-impact
- Source baseline used for this integration: **True Impact 0.5.8-delta development source**
- Public upstream project's stated license: **LGPL-3.0-only**

The integration's exact 0.5.8-delta development-source snapshot should be checked
against the original upstream distribution or permission record before treating the
provenance and license verification as complete. The public upstream repository's
`v0.5.7-delta` tag does not, by itself, establish the license of a later development snapshot.

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

## Release source distribution

For each public PMWeather-IV release, distribute the mod JAR and its matching public source
archive from the same download location. Recipients may modify and rebuild PMWeather-IV under
the LGPL-3.0-only terms. Keep the corresponding source, this notice, and the included
`COPYING.LESSER` and `COPYING` license texts with the release distribution.

The corresponding 0.12.0-rc1 public source snapshot has been published in the PMWeather-IV
repository at commit
https://github.com/G9third/pmweather-iv/commit/e68701f037fa58537327f3ce08c1866cfd5f5dbb.
This identifies the source snapshot for the separately supplied RC1 JAR, not a claim
that a GitHub Release or its downloadable binary has already been published. Later
repository documentation changes do not retroactively alter that compiled JAR; a
new JAR build is required for updated embedded notices.
