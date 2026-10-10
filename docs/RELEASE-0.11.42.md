# PMWeather-IV 0.11.42

This maintenance release removes obsolete implementation layers from the current Sable addon.

- Removed the unused standalone landing-gear tangential, rolling-resistance and directional impulse solvers, their private state/result records and exclusive constants. The live shared coupled contact solver remains.
- Consolidated ground-contact construction onto the current record contract with identical defaults for native fallback and fixture callers.
- Removed superseded fixture-only fixed-wing control calculations. Existing fixtures now use the same command-aware wing/tail helpers as gameplay.
- Reduced integrated terrain material preparation to the density actually used for overburden. Removed unused upstream plasticity, friction, restitution, yield and compaction-property calculations and inert class-threshold tuning. Deferred material application still refines thresholds from real block hardness and confinement.
- Scoped terrain queue deduplication and damage-feedback cooldowns to their dimension, so equal block coordinates in separate levels cannot suppress each other. Existing global budgets and timings remain.
- Simplified the existing deterministic force-to-tool drop policy, removed unused internal aliases/no-op diagnostics and cleaned obsolete tool-layout reflection.
- Replaced accumulated architecture descriptions and stale build instructions with the current Sable/public/private contracts. The embedded private trace README now identifies format 23.

Aircraft and road force laws, live tire grip/braking, rotor assistance, authored animation/trim handling, physical integration, terrain energy/loss calibration, seat recovery and gameplay networking retain their current behavior. Native towing/road-following/blimp compatibility and incomplete-model fallbacks remain. The pack-facing sound-variable API remains unchanged. Unused internal Java helpers and constructors were removed; code that directly called those internal helpers must use the current contracts.

Public/developer compilation, compilation of the existing fixtures and independent public-source compilation are recorded separately. No regression tests or gameplay tests were run for this release. Public/private gameplay class identity and source/archive boundaries are checked during packaging. Fresh runtime confirmation remains pending; this cleanup does not establish calibrated aircraft handling or compatibility with every content pack.
