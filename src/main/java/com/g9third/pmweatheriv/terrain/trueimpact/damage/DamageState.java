/*
 * Portions derived from True Impact 0.5.8-delta by OMEGAU371 / True Impact contributors.
 * Original project: https://github.com/OMEGAU371/sable-true-impact
 * Original license: LGPL-3.0-only.
 * PMWeather-IV modification notice: adapted for PMIV; notice updated 2026-10-09.
 * Adapted for PMWeather-IV's integrated aircraft/world material-response subsystem.
 */
package com.g9third.pmweatheriv.terrain.trueimpact.damage;

/** Accumulated damage classification used for crack feedback and terrain fracture. */
public enum DamageState {
    /** ratio < 0.25: no meaningful damage accumulated. */
    INTACT,
    /** 0.25 <= ratio < 0.60: minor damage; structural integrity mostly intact. */
    BRUISED,
    /** 0.60 <= ratio < 1.00: significant damage; approaching break threshold. */
    CRACKED,
    /** ratio >= 1.00: threshold exceeded; block would break in a destructive phase. */
    CRITICAL;

    /**
     * Classifies a damage ratio into a DamageState.
     * Non-finite ratio (NaN/Infinity) returns INTACT as a safe fallback.
     */
    public static DamageState of(double ratio) {
        if (!Double.isFinite(ratio) || ratio < 0.25) return INTACT;
        if (ratio < 0.60) return BRUISED;
        if (ratio < 1.00) return CRACKED;
        return CRITICAL;
    }
}
