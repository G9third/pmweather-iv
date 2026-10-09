/*
 * Portions derived from True Impact 0.5.8-delta by OMEGAU371 / True Impact contributors.
 * Original project: https://github.com/OMEGAU371/sable-true-impact
 * Original license: LGPL-3.0-only.
 * PMWeather-IV modification notice: adapted for PMIV; notice updated 2026-10-09.
 * Adapted for PMWeather-IV's integrated aircraft/world material-response subsystem.
 */
package com.g9third.pmweatheriv.terrain.trueimpact.damage;

/** Material density used by the integrated terrain overburden calculation. */
public final class MaterialPropertiesProfile {
    private MaterialPropertiesProfile() {}

    public static double densityKgM3(MaterialThresholdProfile.MaterialClass mc, float blastResist) {
        double base = switch (mc) {
            case SOFT_SOIL     -> 1_600;
            case BRITTLE       -> 2_500; 
            case WOOD          ->   700;
            case STONE         -> 2_400;
            case METAL         -> 7_800; 
            case HIGH_STRENGTH -> 9_500;
            case GENERIC       -> 1_500;
        };
        // Small intra-class variation: denser/more resistant variants weigh slightly more
        double scale = 1.0 + Math.min(Math.max(0, blastResist), 100.0) / 1000.0;
        return base * scale;
    }
}
