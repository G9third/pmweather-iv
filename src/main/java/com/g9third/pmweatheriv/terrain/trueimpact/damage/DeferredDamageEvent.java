/*
 * Portions derived from True Impact 0.5.8-delta by OMEGAU371 / True Impact contributors.
 * Original project: https://github.com/OMEGAU371/sable-true-impact
 * Original license: LGPL-3.0-only.
 * PMWeather-IV modification notice: adapted for PMIV; notice updated 2026-10-09.
 * Adapted for PMWeather-IV's integrated aircraft/world material-response subsystem.
 */
package com.g9third.pmweatheriv.terrain.trueimpact.damage;

/** One aircraft/world contact queued for end-of-tick terrain processing.
 * Energy is 0.5 * normal impulse * pre-impact inward point speed, in joules.
 * Direction points into the terrain; NaN selects isotropic confinement.
 */
public record DeferredDamageEvent(
    long serverTick,
    String levelKey,
    String victimBlock,
    int posX, int posY, int posZ,
    MaterialThresholdProfile.MaterialClass materialClass,
    double kImpact,
    double threshold,
    double impactDirX, double impactDirY, double impactDirZ
) {
    public DeferredDamageEvent withThreshold(double value) {
        return new DeferredDamageEvent(serverTick, levelKey, victimBlock,
            posX, posY, posZ, materialClass, kImpact, value,
            impactDirX, impactDirY, impactDirZ);
    }
}
