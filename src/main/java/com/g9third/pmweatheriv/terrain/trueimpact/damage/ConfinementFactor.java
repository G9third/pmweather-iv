/*
 * Portions derived from True Impact 0.5.8-delta by OMEGAU371 / True Impact contributors.
 * Original project: https://github.com/OMEGAU371/sable-true-impact
 * Original license: LGPL-3.0-only.
 * PMWeather-IV modification notice: adapted for PMIV; notice updated 2026-10-09.
 * Adapted for PMWeather-IV's integrated aircraft/world material-response subsystem.
 */
package com.g9third.pmweatheriv.terrain.trueimpact.damage;

/**
 * D-3: Confinement Factor — situational strength scaling for world blocks.
 *
 * An embedded block is harder to destroy than an isolated one because surrounding
 * solid blocks provide structural confinement. The effective break threshold is:
 *
 *   effectiveBreakThreshold = baseBreakThreshold × (1 + confinementFactor)
 *
 * Two modes:
 *   Isotropic (no direction): all 6 face neighbors weighted equally. Used as fallback
 *     when the impact direction is unavailable.
 *   Directional (impactDir known): face neighbors are weighted by how much they resist
 *     the actual impact direction. The block being pushed INTO contributes most;
 *     the face the impactor came FROM contributes nothing.
 *
 * Weight formula per face:
 *   w = clamp(DIRECTION_BASE + DIRECTION_AMPLITUDE × dot(faceNormal, impactDir), 0, 1)
 *   impactDir = normalized velocity of the moving body before impact
 *   dot = +1 → face is in the impact/push direction → maximum support weight
 *   dot = -1 → face is in the impactor's origin direction → zero weight (no confinement)
 *
 * Both modes produce comparable magnitudes because directional normalization uses
 * sum(all face weights) rather than a fixed 6, keeping the scale consistent.
 */
public final class ConfinementFactor {
    private ConfinementFactor() {}

    /** Standard gravity (m/s²), used for the lithostatic/overburden pressure calculation. */
    static final double GRAVITY = 9.8;

    // PER_FACE_CAP, OVERBURDEN_EFFICIENCY, DIRECTION_BASE, DIRECTION_AMPLITUDE now live in
    // ImpactRuntimeConfig (CONFINEMENT_PER_FACE_CAP / OVERBURDEN_EFFICIENCY /
    // CONFINEMENT_DIRECTION_BASE / CONFINEMENT_DIRECTION_AMPLITUDE) so they're configurable
    // as PMIV-local defaults. See ImpactRuntimeConfig for calibration values
    // (including the retained OVERBURDEN_EFFICIENCY of 0.15).

    /**
     * Face outward normals in the order used by IntegratedImpactTerrain.sampleNeighborCrackThresholds:
     *   [0] above  (0, +1,  0)
     *   [1] below  (0, -1,  0)
     *   [2] north  (0,  0, -1)
     *   [3] south  (0,  0, +1)
     *   [4] east  (+1,  0,  0)
     *   [5] west  (-1,  0,  0)
     */
    static final double[][] FACE_NORMALS = {
        { 0,  1,  0},
        { 0, -1,  0},
        { 0,  0, -1},
        { 0,  0,  1},
        { 1,  0,  0},
        {-1,  0,  0},
    };

    // ---- public API -------------------------------------------------------------

    /**
     * Direction-aware confinement.
     *
     * @param neighborCrackJ crack thresholds of up to 6 face neighbors (order matches
     *                       FACE_NORMALS): 0 = air/missing, MAX_VALUE = indestructible
     * @param victimCrackJ   crack threshold of the victim block; must be finite and > 0
     * @param dirX dirY dirZ normalized impact direction vector (moving body pre-impact
     *             velocity direction). NaN → falls back to isotropic weighting.
     */
    public static double compute(double[] neighborCrackJ, double victimCrackJ,
                                 double dirX, double dirY, double dirZ) {
        if (victimCrackJ <= 0 || !Double.isFinite(victimCrackJ)) return 0.0;

        int n = Math.min(neighborCrackJ.length, FACE_NORMALS.length);
        boolean directional = Double.isFinite(dirX) && Double.isFinite(dirY) && Double.isFinite(dirZ);

        double sum       = 0.0;
        double weightSum = 0.0; // sum of weights for all faces (used as denominator)

        for (int i = 0; i < n; i++) {
            double weight;
            if (directional) {
                double[] fn = FACE_NORMALS[i];
                double dot = fn[0] * dirX + fn[1] * dirY + fn[2] * dirZ;
                weight = Math.max(0.0, Math.min(1.0,
                        ImpactRuntimeConfig.CONFINEMENT_DIRECTION_BASE
                                + ImpactRuntimeConfig.CONFINEMENT_DIRECTION_AMPLITUDE * dot));
            } else {
                weight = 1.0; // isotropic: equal weight per face
            }
            weightSum += weight;

            if (neighborCrackJ[i] > 0 && weight > 0) {
                double ratio = Double.isFinite(neighborCrackJ[i])
                        ? neighborCrackJ[i] / victimCrackJ
                        : ImpactRuntimeConfig.CONFINEMENT_PER_FACE_CAP;
                sum += weight * Math.min(ratio, ImpactRuntimeConfig.CONFINEMENT_PER_FACE_CAP);
            }
        }

        return weightSum > 0 ? sum / weightSum : 0.0;
    }

    /**
     * Adds lithostatic yield cost to the confined fracture threshold in joules.
     * pressure(Pa) = density(kg/m³) × gravity × depth(m); energy(J) = pressure × 1 m³.
     * The caller compares this per-voxel cost with the remaining impact energy.
     */
    public static double overburdenEnergyJ(double overburdenDepthBlocks, double densityKgM3) {
        if (overburdenDepthBlocks <= 0 || densityKgM3 <= 0 || !Double.isFinite(overburdenDepthBlocks)) {
            return 0.0;
        }
        return overburdenDepthBlocks * densityKgM3 * GRAVITY * ImpactRuntimeConfig.OVERBURDEN_EFFICIENCY;
    }

    /**
     * Energy-driven dynamic sampling radius (D-3.1).
     * R=1 (6 face neighbors): impactEnergyJ ≤ ~100J.
     * R=5 (~sphere): impactEnergyJ ≥ ~1600J.
     */
    public static int dynamicRadius(double impactEnergyJ) {
        if (impactEnergyJ <= 0) return 1;
        int r = (int) Math.ceil(Math.log(impactEnergyJ / 50.0) / Math.log(2.0));
        return Math.max(1, Math.min(5, r));
    }
}
