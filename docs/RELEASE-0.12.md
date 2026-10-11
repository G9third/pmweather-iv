# PMWeather-IV 0.12

PMWeather Aeronautics (PMAero) 1.0 or later is required. You can download it from [CurseForge](https://www.curseforge.com/minecraft/mc-mods/pmweather-aeronautics-compat/files/9114809) or [Modrinth](https://modrinth.com/mod/pmweather-aeronautics-compat/versions).

## Auto-trim HUD

- Auto trim remains off by default and is toggled per supported fixed-wing aircraft using the rebindable Right Alt key.
- When auto trim is OFF, the persistent auto-trim badge and status panel are hidden. The brief enable/disable key hint still appears upon entering a supported pilot seat.
- While auto trim is enabled (including learning, paused and limited states), its live trim and status indicators remain available.
- This is a client-side display change only; it does not change trim control, physics, networking or saved opt-in behavior.

## Other changes

Includes the source changes and compatibility updates documented in [0.12.0-rc1](https://github.com/G9third/pmweather-iv/blob/v0.12/docs/RELEASE-0.12.0-rc1.md).
The public-source package excludes third-party dependency JARs and private development tooling.

## Live wind HUD scope

- Automatic live wind display now follows occupancy of an IV aircraft pilot seat: it appears while piloting and hides after dismount.
- `/pmaero live on` enables a manual display elsewhere; `/pmaero live off` disables the manual display and suppresses automatic display until the next pilot-seat entry.
- A manual-on choice persists after dismount but resets on world change or disconnect. Wind sampling stops when neither manual nor automatic visibility is active.
- The 0.12 stable release JAR and matching source ZIP include this wind HUD fix.
