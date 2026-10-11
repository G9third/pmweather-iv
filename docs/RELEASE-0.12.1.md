# PMWeather-IV 0.12.1

## Compatibility

- Adds support for **PMWeather 0.16.4 through PMWeather Aeronautics 1.1 or later**. The native XYZ wind, including vertical tornado wind, comes from PMWeather's own API.
- Retains PMWeather **0.17.14 through 0.17.16** support with compatible PMAero wind revision 3 and packed API version 2. The newer weather integration is unchanged.
- Keeps the existing vehicle physics, content-pack suspension limits, particle exclusions, Right Alt auto-trim toggle and shared `/pmaero` wind/weather-test commands.

Install dependency mods separately. For PMWeather 0.16.4, use the [PMAero 1.1 source and build instructions](https://github.com/G9third/pmweather-aeronautics); older PMAero 1.0 downloads do not support that weather version.

## Downloads

- `PMWeather-IV-0.12.1.jar`: clean public build.
- `PMWeather-IV-0.12.1-public-source.zip`: matching public source and build files.
- `SHA256SUMS.txt`: release artifact checksums.

The downloads exclude third-party dependency JARs and private development recorders, profiling, and diagnostic commands. The source build defaults to the clean variant.

Clean and development compilation and static API/packaging review succeeded locally. No new Minecraft session or regression suite was run for this compatibility update; in-game behavior still needs confirmation with the older weather version.
