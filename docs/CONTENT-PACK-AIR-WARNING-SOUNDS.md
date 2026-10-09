# PMWeather-IV content-pack air-warning sounds

PMWeather-IV 0.11.34 exposes atmospheric warning data as **normal Immersive Vehicles
computed variables**. Content-pack authors do not need a PMIV-specific sound format and do
not need Java code.

Use the same `rendering.sounds` entries, `.ogg` assets, `activeAnimations`,
`volumeAnimations`, `pitchAnimations`, interior/exterior flags, positions, distances and
sound variations that you already use for ordinary IV sounds. PMIV only supplies additional
variables that IV can read while evaluating those normal sound definitions. The variables
work on sounds defined directly on the aircraft and on normal attached IV parts; part-level
sounds resolve the owning aircraft automatically.

## Requirements

- Minecraft 1.21.1
- Immersive Vehicles / MTS 24+
- PMWeather-IV 0.11.34
- The normal PMIV runtime dependencies
- The content pack must be installed on the client, as with any other IV pack that contains
  custom sound assets.

## Quick example: one-shot "WINDSHEAR" callout

Put your audio file in your pack the same way you would any normal IV sound. For example,
if your pack uses the namespace `yourpack`, a typical asset is:

```text
assets/yourpack/sounds/wind_shear_warning.ogg
```

Then add a normal IV sound entry to the vehicle's `rendering.sounds` list:

```json
{
  "name": "yourpack:wind_shear_warning",
  "isInterior": true,
  "activeAnimations": [
    {
      "animationType": "visibility",
      "variable": "pmiv_warning_windshear",
      "clampMin": 1,
      "clampMax": 1
    }
  ]
}
```

`pmiv_warning_windshear` is `0` while the warning is inactive and `1` while it is active.
With an ordinary **non-looping** IV sound, IV plays the sound when the condition becomes
true and will not play it again until the condition first becomes false and then becomes
true again. This is useful for spoken alerts such as "WINDSHEAR".

Do not set `forceSound` for this ordinary one-shot pattern unless you specifically want
IV's force-sound behavior.

## Looping warning

Use IV's normal `looping` field when the alarm should remain active for the duration of the
warning:

```json
{
  "name": "yourpack:wind_shear_tone",
  "looping": true,
  "isInterior": true,
  "activeAnimations": [
    {
      "animationType": "visibility",
      "variable": "pmiv_warning_windshear",
      "clampMin": 1,
      "clampMax": 1
    }
  ]
}
```

When the warning variable falls back to `0`, IV stops the looping sound normally.

## Severity-driven volume or pitch

`pmiv_warning_windshear_level` is a normalized `0.0` to `1.0` value. At PMIV's configured
wind-shear threshold it is approximately `0.5`; at twice that wind-change magnitude it is
`1.0`.

IV sound switchboxes use ordinary translation animations for volume/pitch values. The
translation contribution comes from `axis.y`, so a looping tone whose volume follows PMIV
severity can be written like this:

```json
{
  "name": "yourpack:wind_shear_tone",
  "looping": true,
  "isInterior": true,
  "activeAnimations": [
    {
      "animationType": "visibility",
      "variable": "pmiv_warning_windshear",
      "clampMin": 1,
      "clampMax": 1
    }
  ],
  "volumeAnimations": [
    {
      "animationType": "translation",
      "variable": "pmiv_warning_windshear_level",
      "axis": { "x": 0, "y": 1, "z": 0 }
    }
  ]
}
```

You can use the same PMIV variable in normal IV `pitchAnimations` if desired.

## Available PMIV variables

All values are read-only. PMIV does not replace or change existing IV variables.

| Variable | Range / units | Meaning |
| --- | --- | --- |
| `pmiv_warning_windshear` | `0` or `1` | Built-in wind-shear warning state. |
| `pmiv_warning_windshear_level` | `0.0` to `1.0` | Normalized wind-shear severity. The configured alert threshold is approximately `0.5`. |
| `pmiv_windshear_delta_mps` | m/s | Magnitude of the change in the sampled mean wind vector over one Minecraft second. |
| `pmiv_windshear_delta_knots` | knots | Same one-second wind-vector change magnitude in knots. |
| `pmiv_windshear_vertical_delta_mps` | m/s | Signed one-second vertical wind change. Positive is an increase toward +Y/up; negative is an increase toward downward wind. |
| `pmiv_windshear_vertical_delta_knots` | knots | Same signed vertical wind change in knots. |
| `pmiv_wind_speed_mps` | m/s | Current magnitude of PMWeather's mean source wind sampled at the aircraft's real aerodynamic force points. |
| `pmiv_wind_speed_knots` | knots | Same current wind magnitude in knots. |
| `pmiv_vertical_wind_mps` | m/s | Current signed vertical source wind. Positive is +Y/up and negative is downward. |
| `pmiv_vertical_wind_knots` | knots | Same current signed vertical wind in knots. |

If PMIV is not currently managing the vehicle, or the synchronized warning state is stale,
these variables evaluate to zero rather than leaving a warning sound stuck on.

## Use your own warning threshold

You do not have to use PMIV's built-in wind-shear trigger. The raw variables are exposed so
content packs can use IV's normal clamps to define their own warning thresholds.

For example, a non-looping sound that activates only when the one-second wind-vector change
reaches 20 knots:

```json
{
  "name": "yourpack:severe_wind_shear",
  "isInterior": true,
  "activeAnimations": [
    {
      "animationType": "visibility",
      "variable": "pmiv_windshear_delta_knots",
      "clampMin": 20,
      "clampMax": 10000
    }
  ]
}
```

A simple downdraft warning can likewise use the signed current vertical wind. This example
activates at -10 knots or stronger downward wind:

```json
{
  "name": "yourpack:downdraft_warning",
  "isInterior": true,
  "activeAnimations": [
    {
      "animationType": "visibility",
      "variable": "pmiv_vertical_wind_knots",
      "clampMin": -10000,
      "clampMax": -10
    }
  ]
}
```

These are content-pack choices. PMIV does not hardcode a particular aircraft, content-pack
ID, warning voice, alarm tone, or custom threshold into the vehicle definition.

## Built-in wind-shear measurement

The built-in warning compares PMWeather's source-native mean wind sampled at the actual
PMIV aerodynamic force points with the corresponding mean wind one Minecraft second
(20 ticks) earlier.

The default server-config threshold is:

```toml
[airWarnings]
windShearWarningThresholdKnots = 15.0
```

The warning releases at 80% of the configured threshold to avoid rapidly toggling on and
off right at the boundary. The default 15-knot magnitude is intentionally comparable to
the familiar FAA LLWAS wind-shear alert magnitude, but PMIV is **not** implementing an
airport LLWAS or a certified predictive/reactive aircraft wind-shear system. It is a game
telemetry trigger based on the wind actually entering PMIV's aircraft physics.

Changing this server setting changes `pmiv_warning_windshear` and its normalized level. It
does **not** change the raw `pmiv_windshear_*` or `pmiv_wind_*` measurements, so content packs
that use their own clamps remain independent of the server's built-in alert threshold.

## Normal IV sound features still work

Because these are IV computed variables rather than a separate PMIV sound engine, a pack can
continue using normal IV features including:

- `isInterior` / `isExterior`
- `looping`
- `soundVariations`
- `pos` / `centerPoint`
- distance attenuation fields
- conical sound fields
- `activeAnimations`
- `volumeAnimations`
- `pitchAnimations`
- combinations with other ordinary IV variables

For example, you may combine a PMIV air-warning variable with an existing IV electrical,
engine, master-warning, door, or cockpit-state variable in the same animation list if your
pack already uses those conditions.

## Naming convention for future PMIV warning variables

PMIV reserves the `pmiv_` prefix for this interface. New built-in warnings should follow:

```text
pmiv_warning_<warning_name>
pmiv_warning_<warning_name>_level
```

Raw atmospheric measurements use descriptive `pmiv_wind_*` or
`pmiv_<measurement>_*` names with units in the variable name where practical.

This convention is intended to let packs add new sounds without receiving pack-specific Java
code. It does not mean an arbitrary JSON variable automatically creates new PMIV warning
logic; a new physical warning measurement still needs to exist in PMIV before a pack can
reference it.

## Troubleshooting

1. Confirm PMWeather-IV 0.11.34 is installed on both server and client along with its required
   dependencies.
2. Confirm the vehicle is a PMIV-managed aircraft. Blimps currently retain IV-native flight.
3. Confirm the `.ogg` and sound name work as an ordinary IV sound first.
4. Temporarily use a low clamp on a raw PMIV variable to confirm the sound condition itself.
5. For a one-shot callout, leave `looping` false/omitted and do not set `forceSound` unless
   that is deliberately required by the pack.

## Compatibility note

This interface is additive. Existing IV content packs need no new sound definitions. PMIV physics and normal IV sound evaluation remain separate responsibilities.
