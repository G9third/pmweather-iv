Dependency binaries are not included in this public source archive.

Place the exact locally obtained compile inputs listed in DEPENDENCIES.json in
this directory before building. Obtain Immersive Vehicles from its official
release channel, and obtain PMWeather and Sable from their authorized channels.
Keep these external binaries out of public PMIV source/release uploads unless
their rights holders have authorized redistribution.

The Sable Companion compile input is the Companion JAR embedded inside the
Sable 2.0.5 mod archive. Extract the matching nested JAR and name the local copy
sable-companion-common-1.21.1-1.6.0.jar. This exposes its classes to Gradle;
PMIV does not embed the dependency in its output JAR.

Runtime additionally requires PMWeather Aeronautics 1.0 or later. Its public API
is loaded by PMIV at runtime; that mod is not a bundled compile input.
