package com.g9third.pmweatheriv.physics;

/** Brief world-block mutation grace for a new player placement; physical collisions stay active. */
public final class SpawnTerrainDamageGrace {
    public static final long SETTLING_TICKS = 60;
    private SpawnTerrainDamageGrace() {}
    public static boolean allowsTerrainDamage(boolean playerPlacement, long entityAgeTicks) {
        return !playerPlacement || entityAgeTicks < 0 || entityAgeTicks >= SETTLING_TICKS;
    }
}
