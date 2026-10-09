/*
 * Portions derived from True Impact 0.5.8-delta by OMEGAU371 / True Impact contributors.
 * Original project: https://github.com/OMEGAU371/sable-true-impact
 * Original license: LGPL-3.0-only.
 * PMWeather-IV modification notice: adapted for PMIV; notice updated 2026-10-09.
 * Adapted for PMWeather-IV's integrated aircraft/world material-response subsystem.
 */
package com.g9third.pmweatheriv.terrain.trueimpact.damage;

/**
 * Derives per-block impact thresholds from vanilla hardness and blast resistance.
 *
 * Replaces the hand-filled numeric table in MaterialThresholdProfile with a formula
 * driven by real block data:
 *
 *   crackThresholdJ  -- crack-overlay threshold; visual cracking starts above this ratio
 *   breakThresholdJ  -- accumulator denominator and single-hit cap; ratio >= 1.0 -> block breaks
 *
 * Blast resistance is preferred over hardness for impact calculations because:
 *   - hardness = mining time (tool efficiency, sustained force)
 *   - blast resistance = resistance to sudden impulse (closer to impact physics)
 *
 * Formulas (calibrated against in-game targets):
 *   crackThreshold = clamp(15 × blastResist^0.6,  3.0, 500.0)
 *   breakMult      = 5 + blastResist^0.4 × 3
 *   breakThreshold = crackThreshold × breakMult
 *
 * Special case: hardness < 0 (bedrock, barriers) → Double.MAX_VALUE (indestructible).
 *
 * Formula coefficients live in ImpactRuntimeConfig (CRACK_MIN/MAX/COEFF/EXPONENT,
 * BREAK_BASE/COEFF/EXPONENT) as PMIV-local defaults --
 * the values below are only the compiled-in defaults, mirrored there at class-init.
 *
 * No Minecraft imports -- safe to unit-test without the game runtime.
 * Callers (IntegratedImpactTerrain) extract hardness/blastResist from BlockState and pass raw floats.
 */
public final class BlockHardnessProfile {

    private BlockHardnessProfile() {}

    /**
     * Crack-overlay threshold. Also used as the per-hit crack progress denominator in
     * CrackOverlayTracker (ratio = accumulatedEffective / crackThreshold).
     * hardness < 0  → Double.MAX_VALUE (indestructible).
     * blastResist ≤ 0 → ImpactRuntimeConfig.CRACK_MIN (fragile; trivially damaged).
     */
    public static double crackThresholdJ(float hardness, float blastResist) {
        if (hardness < 0) return Double.MAX_VALUE;
        if (blastResist <= 0) return ImpactRuntimeConfig.CRACK_MIN;
        double raw = ImpactRuntimeConfig.CRACK_COEFF * Math.pow(blastResist, ImpactRuntimeConfig.CRACK_EXPONENT);
        return Math.min(ImpactRuntimeConfig.CRACK_MAX, Math.max(ImpactRuntimeConfig.CRACK_MIN, raw));
    }

    /**
     * Total accumulated effective damage required to reach ratio >= 1.0 (CRITICAL / break).
     * hardness < 0  → Double.MAX_VALUE (block can never reach CRITICAL).
     */
    public static double breakThresholdJ(float hardness, float blastResist) {
        double crack = crackThresholdJ(hardness, blastResist);
        if (crack == Double.MAX_VALUE) return Double.MAX_VALUE;
        double breakMult = ImpactRuntimeConfig.BREAK_BASE
                + Math.pow(Math.max(0, blastResist), ImpactRuntimeConfig.BREAK_EXPONENT) * ImpactRuntimeConfig.BREAK_COEFF;
        return crack * breakMult;
    }
}
