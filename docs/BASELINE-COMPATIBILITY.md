# Baseline compatibility

Runtime metadata allows Sable `[2.0.0,3.0.0)` and PMWeather `[0.16.4],[0.17.0,0.18.0)`.

The official NeoForge Sable 2.0.0 and PMWeather 0.17.0 JARs were downloaded and verified against their publisher-provided hashes. Both mods compiled their clean and development variants against those exact dependency bytes, and their existing public regression checks passed.

Static review found all 78 native Sable/PMWeather member references used by PMAero and all 38 used by PMIV. All 11 PMIV Rapier invoker descriptors also match the backend embedded in Sable 2.0.0. The native tornado-vector and Sable wind hooks are available in PMWeather 0.17.0.

Intermediate and later 2.x/0.17.x releases are allowed under the assumption that the integration APIs remain compatible. These checks do not establish runtime correctness for every release, and no new Minecraft session was run. PMWeather 0.18 and Sable 3.x are outside the declared ranges.

Exactly PMWeather 0.16.4 continues to use the separate native XYZ adapter without the newer vector observation or built-in Sable suppression hook.
