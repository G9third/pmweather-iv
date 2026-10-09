# Third-party notices

Notice updated 2026-10-09.

## True Impact

PMWeather-IV incorporates and modifies portions of the True Impact terrain-impact
implementation.

- Project: **True Impact**
- Upstream author/project owner: **OMEGAU371 / True Impact contributors**
- Upstream repository: https://github.com/OMEGAU371/sable-true-impact
- Source baseline used for this integration: **True Impact 0.5.8-delta development source**
- Upstream license: **LGPL-3.0-only**

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

## Release source distribution

For each public PMWeather-IV release, distribute the mod JAR and its matching public source
archive from the same download location. Recipients may modify and rebuild PMWeather-IV under
the LGPL-3.0-only terms. Keep the corresponding source, this notice, and the included
`COPYING.LESSER` and `COPYING` license texts with the release distribution.

The 2026-10-09 notice update is local release preparation. The matching source has not yet
been published on GitHub.
