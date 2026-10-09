/*
 * Portions derived from True Impact 0.5.8-delta by OMEGAU371 / True Impact contributors.
 * Original project: https://github.com/OMEGAU371/sable-true-impact
 * Original license: LGPL-3.0-only.
 * PMWeather-IV modification notice: adapted for PMIV; notice updated 2026-10-09.
 * Adapted for PMWeather-IV's integrated aircraft/world material-response subsystem.
 */
package com.g9third.pmweatheriv.terrain.trueimpact.damage;

import java.util.HashSet;
import java.util.Set;

/** Deferred terrain break eligibility and deduplication, without upstream status UI state. */
public final class MaterialResponsePlanner {
    private MaterialResponsePlanner() {}
    private static final Set<BlockDamageAccumulator.AccKey> breakScheduledKeys = new HashSet<>();

    public static boolean canBreak(BlockDamageAccumulator.Snapshot snapshot) {
        return snapshot.damageState() == DamageState.CRITICAL;
    }

    public static boolean markBreakScheduled(BlockDamageAccumulator.AccKey key) {
        return breakScheduledKeys.add(key);
    }

    public static void forgetKey(BlockDamageAccumulator.AccKey key) {
        if (key != null) breakScheduledKeys.remove(key);
    }

    public static void clear() {
        breakScheduledKeys.clear();
    }
}
