# Current architecture

Paths are relative to `src/main/java/com/g9third/pmweatheriv/`.

PMWeather-IV connects Immersive Vehicles to PMWeather Aeronautics and Sable. PMAero supplies aerodynamic coefficients and atmosphere samples. Sable/Rapier owns physical integration and compound collision for supported aircraft and ordinary untowed ground vehicles. IV continues to own authored controls, actuators, vehicle/part health, content-pack data, and gameplay networking. Towed vehicles, IV road-following vehicles, and blimps retain their native compatibility paths.

## Runtime flow

An IV owner tick captures controls, native actuator output, and the PMAero wind snapshot. Each Sable substep evaluates aerodynamic loads at the current physical pose, reusing that owner-tick wind; applies loads once; solves loaded wheel/gear contacts at their actual lever arms; and lets Rapier integrate the body. Terrain sweeps and post-step recovery handle contact evidence, while shared crash gameplay consumes contacts for IV damage and terrain effects. IV mirrors Sable's pose and receives physical velocity for animation and drivetrain state.

Ordinary road vehicles and aircraft share the Sable body lifecycle. Road drive and braking are represented as contact demands and bounded by live tire loads and friction. Native actuator updates still run once per owner tick. Gravity, aerodynamic/propulsion loads, normal support, and tire impulses each have one physical authority.

## Code map

| Responsibility | Start here | Boundary |
| --- | --- | --- |
| IV interception and lifecycle | `mixin/`, `sable/SableVehicleManager.java` | Select supported vehicles and bridge IV lifecycle to Sable |
| Shared model snapshot | `model/ParsedModelSnapshot.java` | Immutable, unscaled parsed geometry |
| Surface preparation | `physics/ModelSurfaceMap.java`, `AirframePreparation.java` | Prepared model and lifting patch records |
| Sampling and animation evidence | `physics/ModelTriangleSampling.java`, `ModelAnimationHints.java` | Aerodynamic filters, sampling budget, authored motion evidence |
| Surface classification | `physics/SurfaceClassifier.java`, `ControlSurfaceClassifier.java`, `WingSurfaceClassifier.java`, `TailSurfaceClassifier.java`, `BodySurfaceClassifier.java` | Classification priority: controls, explicit names, wing, tail, then body |
| Pressure and lifting geometry | `physics/BodyPressureGeometry.java`, `SableBodyPressureGeometry.java`, `LiftingPatchGeometry.java`, `LiftingSurfaceFrames.java` | Aircraft and road-body pressure patches; lifting sections, frames, and force points |
| Wind and force evaluation | `physics/AircraftPhysics.java`, `RoadVehiclePhysics.java`, `AircraftWind.java`, `AirframeLoads.java` | PMAero calls, owner-tick wind, physical substep loads |
| Fixed-wing controls | `physics/LiftingSurfaceAdapter.java`, `AuthoredLiftingSurfacePoses.java` | Trim, relative flow, copied IV poses, control coupling |
| Rotors and propulsion | `physics/RotorModel.java`, `PropulsionModel.java` | IV actuators, rotor loads, and assisted helicopter controller |
| Body lifecycle and integration | `sable/SableVehicleBody.java` | Registration, loads, gear, body state, pose export |
| Compound collision | `sable/SableCompoundCollider.java`, `SableModelCollisionHull.java` | Authored boxes, model shell, moving rigid children |
| Wheel/gear contact | `physics/LandingGearSolver.java` | Per-contact normal, steering, brake, and friction constraints |
| Terrain momentum response | `sable/SableTerrainResponse.java`, `SableTerrainSweep.java` | Terrain queries, material response, bounded impulses, clear-pose recovery |
| Crash gameplay | `sable/SableCollisionGameplay.java` | IV health/parts and collision episode handling |
| Integrated terrain damage | `terrain/AircraftTerrainImpact.java`, `terrain/trueimpact/` | Material/energy decisions and deferred world effects; no body velocity writes |
| Optional standalone True Impact | `compat/StandaloneTrueImpactCompat.java` | Forwards non-PMIV Sable collision records |
| Oriented entity interactions | `sable/OrientedHitboxRegistry.java`, `OrientedDamageQuery.java`, `OrientedEntityContact.java` | Rotated hitboxes, damage queries, and entity contact |
| Gameplay synchronization | `network/AircraftStateNetwork.java` | Authoritative physical pose, independent airspeed, and IV axial velocity |
| Optional observation | `devsupport/PMIVObserver.java` | No-op public boundary; private ServiceLoader providers implement logs, traces, commands, and HUDs |

## Physical conventions and safeguards

Use metres, seconds, kilograms, newtons, and radians at the physics boundary. Keep IV per-tick and `speedFactor` conversion in existing conversion helpers. Use Sable's actual substep duration for contact stabilization and force impulses. Wind queries belong to the owner-tick snapshot; substeps reuse that snapshot.

Use centre-of-mass velocity for force evaluation. Point velocity adds `omega × (point − COM)`, and pressure/contact torque uses that physical lever arm. Preserve authored collision boxes, attached-part ownership, and terrain-damage ownership. Do not write body velocity from material response or pose recovery.

Model parsing, visibility, damage, animation, and contact filters have separate consumers. Keep aerodynamic sampling thresholds separate from collision shell rules. `SurfaceClassifier` owns classification order; pressure patches and lifting frames have separate preparation stages.

Terrain sweep, finite-state quarantine, collider topology caching, and crash-episode gates address active integration needs. If changing them, preserve contact evidence and prevent tunnelling or duplicate impulses. When pose recovery has no contact normal, it preserves Rapier momentum rather than projecting velocity along an invented direction.

## Limits

The helicopter rate controller, thrust filtering, cyclic translation cue, and bounded rotor shear are the current assisted helicopter model. They are not a full rotor/control simulation. Derived centre of mass and inertia, finite-wing corrections, tire calibration, material response, road roof pressure, and liquid support are approximations. Point-based liquid support is not full hull buoyancy or hydrodynamics. Rigid moving model children do not provide continuous collision for every independently articulating surface.

The material package is PMIV's world-terrain subset; its local defaults are not standalone True Impact configuration. The optional standalone mod is not required for PMIV-managed terrain damage.

## Public and private builds

The public artifact contains shared gameplay physics, synchronization, and the no-op observer boundary. Private logs, trace writers and schema, diagnostic networking, commands, and HUD providers live in the separate `dev` source set and are installed through ServiceLoader. The private artifact must use byte-identical shared production classes. Public build/check tasks do not depend on private sources; `checkDev` verifies both artifact boundaries.
