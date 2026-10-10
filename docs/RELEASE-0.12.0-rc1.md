# PMWeather-IV 0.12.0-rc1

## Auto trim

Automatic elevator trim is enabled by default for healthy, supported fixed-wing
aircraft while a pilot occupies the controller seat. The HUD badge shows when
PMIV is adjusting trim. The first adjustment displays the current key and the
`autoTrim.enabled=false` option for turning the feature off globally. Right Alt
is the default key and can be rebound in Controls. Its on/off choice is saved
for each aircraft through dismount and save/reload. Rotorcraft and unsupported
vehicles are not affected. A global disable preserves each aircraft's saved
choice. Turning auto trim off for an aircraft stops further PMIV adjustments
immediately, including during a learning probe.

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

The public config retains player-facing controls for general physics, auto trim,
the wind-shear warning threshold in knots, and terrain block breaking. Internal
aerodynamic coefficients and model sampling budgets use code-owned defaults.
Older config files can retain retired entries, but PMIV no longer reads them.
Terrain block breaking still requires IV's `vehicleBlockBreaking` option.

## Other changes and limits

Road contact tightens the bounded fallback sag and compression travel for
eligible wheels and treads when a pack does not already author vertical support
motion. Full-unload travel is retained. Authored suspension, tire coefficients,
and brake controls are unchanged. PMWeather's native vector contract and
particle path are unchanged by this PMIV patch.

The private development trace adds a small per-tick auto-trim state summary,
including the native baseline and requested versus applied offset when the
modifier overlay is active. Detailed profiling and trace recording remain
optional development tools.

This is an initial PMIV GitHub release. It does not require a new PMWeather
Aeronautics release. Compilation and numerical regressions were completed;
fresh in-game verification is still needed. No live handling calibration or
frame-rate improvement is claimed here.
