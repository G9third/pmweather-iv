# PMWeather-IV 0.12

## Auto-trim HUD

- Auto trim remains off by default and is toggled per supported fixed-wing aircraft using the rebindable Right Alt key.
- When auto trim is OFF, the persistent auto-trim badge and status panel are hidden. The brief enable/disable key hint still appears upon entering a supported pilot seat.
- While auto trim is enabled (including learning, paused and limited states), its live trim and status indicators remain available.
- This is a client-side display change only; it does not change trim control, physics, networking or saved opt-in behavior.

## Other changes

Includes the source changes and compatibility updates documented in [0.12.0-rc1](RELEASE-0.12.0-rc1.md).
The public-source package excludes third-party dependency JARs and private development tooling.
