# PMWeather-IV 0.12.0-rc1

## Auto trim

Automatic elevator trim is off by default for new aircraft and for older saves
without an explicit per-aircraft opt-in. Every time a player enters a supported
fixed-wing pilot seat, a brief centered HUD popup shows the current toggle key
and whether it enables or disables auto trim. Client eligibility uses live rotor
parts, and the popup and badge render in their own HUD layer. Right Alt is the default and can
be rebound in Controls. Press it to enable or disable auto trim for that
aircraft; the choice persists through dismount and save/reload. The small HUD
badge uses plain instrument labels, a trim angle and a small status light.
Complete status text wraps on a narrow viewport instead of being truncated.
Rotorcraft and unsupported
vehicles are not affected. Turning auto trim off for an aircraft stops further
PMIV adjustments immediately, including during a learning probe.

For fixed-wing packs that author `trim_elevator` modifiers, PMIV applies its
learned value as a separate offset around IV's normal modifier pass. The authored
pack baseline continues to update as before, and PMIV's offset follows IV's
0.1-degree-per-tick adjustment limit. Other eligible fixed-wing aircraft retain
the native absolute-trim path.

Auto trim pauses during manual input, ground contact, low airflow, stall-like
conditions, high angle of attack or pitch rate, and native autopilot control.
Please report content-pack incompatibilities across cars, tanks, APCs, planes
and helicopters at [PMWeather-IV Issues](https://github.com/G9third/pmweather-iv/issues).
Include the pack name and version, mod versions, and what happened. A development
trace is optional.

## Public configuration

The public config retains player-facing controls for general physics, the
wind-shear warning threshold in knots, and terrain block breaking. Auto trim is
controlled per aircraft with the toggle key. Internal aerodynamic coefficients
and model sampling budgets use code-owned defaults. Older config files can
retain retired entries, but PMIV no longer reads them. Terrain block breaking
still requires IV's `vehicleBlockBreaking` option.

## Body terrain contacts

Continuous low-speed support on a belly, side or roof no longer accumulates terrain
fracture or repeatedly restarts a crash episode. New impacts and severe reimpacts
retain terrain damage. This policy applies to the shared car and aircraft body path;
wind forces, authored tire coefficients and brake inputs are unchanged.

Terrain damage now uses the impulse assigned by the complete contact manifold,
so a station released by the coupled solver cannot destroy its supporting block.
Quiet geometric contacts keep their support history, and surviving hull stations
remain active when another station fractures terrain. Query-local air-column proofs
reduce repeated height scans without skipping solid blocks; fracture invalidates them.

Material evaluation starts at the damage-eligible station carrying the largest
coupled impact work, rather than always using the first swept contact. When terrain
fractures, only that station's paid energy changes momentum; the remaining compound
is swept again before integration. This prevents simultaneous contacts from
committing rigid stops before their terrain has been evaluated. Material defaults
and the existing bounded fracture work limits remain the same.

Accepted deferred damage on intact terrain now uses the full rigid contact manifold.
Swept impulses use the time-of-impact pose, and contact points are localized to the
actual overlap with the struck terrain shape. These changes correct false torque
arms that can exaggerate rotation and subsequent rebound. Real contact-induced
rotation and separation are preserved; this is not a blanket no-bounce rule.

## IV smoke wind

Free IV smoke, including generic billboard particles using smoke textures, keeps
ambient wind drift separate from authored particle motion. Pack movement-duration
and terminal-speed limits therefore do not erase that wind contribution each tick.
Authored plume motion and native particle collisions remain active. The existing
shared wind cache and application/query budgets are retained; vanilla and
Aeronautics particles continue through PMWeather's own particle handling.

For packs without custom crash-speed limits, IV's fallback now counts the actual
hardness of all immediately fractured blocks, rather than only the seed block.
Its formula and authored crash thresholds are unchanged.

The development trace attributes deferred block removal and compaction to the
originating vehicle and records rejected support contacts at most once per tick.

## Other changes and limits

Road contact tightens the bounded fallback sag and compression travel for
eligible wheels and treads when a pack does not already author vertical support
motion. Full-unload travel is retained. Authored suspension, tire coefficients,
and brake controls are unchanged. PMWeather's native vector contract and
particle path are unchanged by this PMIV patch.

Private development trace format 28 records auto trim's per-aircraft opt-in in
its per-tick state summary, including the native baseline and requested versus
applied offset when the modifier overlay is active. Detailed profiling and
trace recording remain optional development tools.

This is an initial PMIV GitHub release. It does not require a new PMWeather
Aeronautics release. Compilation and numerical regressions were completed;
fresh in-game verification is still needed. No live handling calibration or
frame-rate improvement is claimed here.
