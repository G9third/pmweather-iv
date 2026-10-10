# Compatibility evidence for 0.12.0-rc1

## Current candidate

The 2026-10-09 audit-fix candidate requires matching PMAero 1.0 wind revision 3.
It uses vector-only wind sampling and shares exact same-tick native queries.
Contact normals propagate through the common road/plane/helicopter tire solver.
Pack tire coefficients are retained. Rounded wheel support uses a sphere/voxel proxy
derived from the authored wheel radius; it is not an exact cylindrical tire mesh.

Wheel impact effective mass, terrain response and gravity compensation use that normal.
Body friction stops after convergence; tire setup evaluates symmetric matrix pairs once.
Private traces include separate terrain-probe and gear-constraint timing stages.

Java 21 offline compilation and public/dev packaging were performed. No new regression
suite, game session or content-pack test was run for this candidate.
Counts below are historical evidence and do not validate the new contact or wind behavior.
Public and private builds share identical gameplay classes.

## Public source corpus

| Repository | Pinned commit | Selected JSON definitions | Selected OBJ models |
| --- | --- | ---: | ---: |
| [Trin Civil](https://github.com/TheOddlySeagull/Trin-Civil-Pack-V3-V4/tree/27b24fe57b67be40d14b88afddaeab54433adc53) | `27b24fe57b67` | 5 | 2 |
| [Trin Parts](https://github.com/TheOddlySeagull/Trin-Part-Pack/tree/8703b840f4909c837dae39663901defa27d1049a) | `8703b840f490` | 13 | 0 |
| [Trin Military](https://github.com/TheOddlySeagull/Trin-Military-Pack-V2/tree/acb532f953d8b528ffb0d9fb08e271f7bf6e94ba) | `acb532f953d8` | 3 | 3 |
| [IV Vanity](https://github.com/mklave4d6v8/immersive_vehicles_vanity/tree/cea9b654978889c1f6961bfca0fefe8e10cb39b2) | `cea9b6549788` | 7 | 3 |
| [Official Pack](https://github.com/DonBruce64/MTSOfficialPack/tree/a157eef94d525b7411504264c40e2abab53c10cc) | `a157eef94d52` | 9 | 6 |
| [Turbo Random Packs](https://github.com/RishiMenon2004/turbos-random-packs/tree/b5a1290587149e6a5d002f13316c1376c0de748b) | `b5a129058714` | 2 | 2 |
| Total | | 39 | 16 |


The retained local corpus contains IV-style JSON definitions and OBJ models from these pinned sources. Earlier inventory checks parsed commented/trailing-comma JSON and verified 55 fixture hashes. The assets stay outside the public source archive. This corpus exposes feature diversity; it does not establish correct runtime behavior for every vehicle in these packs.

## Feature coverage retained

- Authored rotation/translation, applyAfter ancestry, separate input/trim ownership, non-unit axes, model scale and inverse-transpose normal conversion.
- Attached lifting parts, nested placement, owner/slot provenance, conserved global area and hidden/detached control shares.
- Static and eligible rigid animated exterior collision shells, topology invalidation and consecutive owner-tick surface rates.
- Pack tire coefficients, differential steering, liquid support and authored ballast/gravity ratios.
- Cumulative physical pose, finite packet state, entity/seat lifecycle and observer isolation.

Existing standalone fixtures exercise these production helpers. They use synthetic geometry, native field fixtures or selected real meshes; they do not replace game validation.

## Historical helper results

Earlier geometry/contact checks passed 29,275 numerical assertions. An optional real moving-hull OBJ/JSON check passed 67,598 assertions, covering seven moving objects and 108 merged boxes with 13,342 sampled source-triangle points. Sampled coverage does not prove complete volume containment or live collision response.

Earlier authored-pose, moving-hull, wing-moment, road, packet and observer fixtures passed 6,079, 54,248, 196, 186, 22 and 17 assertions respectively. The material and private trace golden comparisons originated in older source layouts. These historical counts were not rerun after this cleanup.

Use `-PaccuracyModel=/path/to/model.obj` for the existing coordinate fixture and both `-PmovingHullDefinition=/path/to/definition.json` and `-PmovingHullModel=/path/to/model.obj` for the existing moving-hull fixture. Obtain original assets from their pinned upstream sources.

## Runtime evidence and limits

Recent sampled fixed-wing captures supported successful placement clearance and mounted-player tracking. They also exposed near-stationary skid feedback and prolonged wreck actuator loads; 0.11.40 addressed those paths. The 0.11.41 control changes retain copied native input/trim ownership and relative parent/control chord deflection. Their installed audio, control/trim response and flight behavior still need fresh runtime confirmation.

Save/reload, multiplayer, unseen pack driving/flight and continuously swept independently animated panels remain validation gaps. Dynamic scaling, shear, mixed-source lifting geometry, complete hydrodynamics and same-resource-path OBJ hot reload remain unsupported or approximate. Derived mass/CG, neutral-frame inference, area-integrated parent circulation and attached geometry prepared at an already-deflected pose remain approximations. Helicopters retain the assisted controller.

No correction selects a pack ID, vehicle identity or fixture filename. No broad compatibility or calibrated flight-performance claim follows from compilation or helper coverage alone.

The rounded edge proxy is anchored to IV\'s existing lowest tire support point, which
already includes axle, tilt and width. Flat top-face support height is preserved.
