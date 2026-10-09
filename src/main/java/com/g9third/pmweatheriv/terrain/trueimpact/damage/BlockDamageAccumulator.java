/*
 * Portions derived from True Impact 0.5.8-delta by OMEGAU371 / True Impact contributors.
 * Original project: https://github.com/OMEGAU371/sable-true-impact
 * Original license: LGPL-3.0-only.
 * PMWeather-IV modification notice: adapted for PMIV; notice updated 2026-10-09.
 * Adapted for PMWeather-IV's integrated aircraft/world material-response subsystem.
 */
package com.g9third.pmweatheriv.terrain.trueimpact.damage;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Per-block cumulative damage accumulator for deferred impact events.
 *
 * Unified formula (matches SublevelDamageAccumulator for Phase 3A):
 *   effectiveJ = min(rawKImpact, breakThreshold)
 *
 * breakThreshold comes from DeferredDamageEvent.threshold(), which is computed by
 * BlockHardnessProfile.breakThresholdJ(hardness, blastResist) × (1 + confinement)
 * in IntegratedImpactTerrain.onServerTickPost. A single hit at or above the threshold
 * immediately saturates (ratio ≥ 1.0 → CRITICAL); sub-threshold hits accumulate.
 *
 * Each entry is keyed on (levelKey, posX, posY, posZ, victimBlock) so damage
 * from different block types at the same position stays in separate entries.
 * Raw kImpact is preserved per-hit for diagnostic display.
 *
 * Threading: all operations called on the server thread. No synchronization needed.
 * No Minecraft imports -- safe to unit-test without the game runtime.
 */
public final class BlockDamageAccumulator {

    private BlockDamageAccumulator() {}

    /** Uniquely identifies a (dimension, block position, block type) triplet. */
    public record AccKey(String levelKey, int posX, int posY, int posZ, String victimBlock) {}

    /** Immutable snapshot of accumulated damage state for a single block. */
    public record Snapshot(
            AccKey     key,
            MaterialThresholdProfile.MaterialClass materialClass,
            double     accumulatedEffectiveDamageJ,
            double     thresholdJ,
            double     lastRawImpactJ,
            double     lastEffectiveDamageJ,
            long       lastUpdatedTick,
            int        hitCount,
            DamageState damageState
    ) {
        /** Ratio of accumulated effective damage to threshold. NaN when threshold is zero. */
        public double ratio() {
            return thresholdJ > 0.0 ? accumulatedEffectiveDamageJ / thresholdJ : Double.NaN;
        }
    }

    private static final class Entry {
        final MaterialThresholdProfile.MaterialClass materialClass;
        final double thresholdJ;
        double     accumulatedEffectiveDamageJ;
        double     lastRawImpactJ;
        double     lastEffectiveDamageJ;
        long       lastUpdatedTick;
        int        hitCount;
        DamageState damageState;

        Entry(MaterialThresholdProfile.MaterialClass mc, double threshold,
              double rawImpact, double effectiveDamage, long tick) {
            this.materialClass              = mc;
            this.thresholdJ                 = threshold;
            this.accumulatedEffectiveDamageJ = effectiveDamage;
            this.lastRawImpactJ             = rawImpact;
            this.lastEffectiveDamageJ       = effectiveDamage;
            this.lastUpdatedTick            = tick;
            this.hitCount                   = 1;
            this.damageState                = DamageState.of(effectiveDamage / threshold);
        }

        void addImpact(double rawImpact, double effectiveDamage, long tick) {
            accumulatedEffectiveDamageJ += effectiveDamage;
            lastRawImpactJ               = rawImpact;
            lastEffectiveDamageJ         = effectiveDamage;
            lastUpdatedTick              = tick;
            hitCount++;
            damageState = DamageState.of(accumulatedEffectiveDamageJ / thresholdJ);
        }

        /** Exponential stress relaxation between hits (same model as SublevelDamageAccumulator). */
        void decayTo(long nowTick, int halfLifeTicks) {
            if (halfLifeTicks <= 0) return;
            long dt = nowTick - lastUpdatedTick;
            if (dt <= 0) return;
            accumulatedEffectiveDamageJ *= Math.pow(0.5, (double) dt / halfLifeTicks);
            damageState = DamageState.of(accumulatedEffectiveDamageJ / thresholdJ);
        }

        Snapshot toSnapshot(AccKey key) {
            return new Snapshot(key, materialClass, accumulatedEffectiveDamageJ, thresholdJ,
                    lastRawImpactJ, lastEffectiveDamageJ, lastUpdatedTick, hitCount, damageState);
        }
    }

    private static final Map<AccKey, Entry> entries = new LinkedHashMap<>();

    /**
     * Accumulates effective damage from a drained DeferredDamageEvent, or — when
     * ImpactRuntimeConfig.ENABLE_DAMAGE_ACCUMULATION is false — judges this hit alone with
     * no persistent state (single-hit mode: the hit either clears the threshold by itself
     * or nothing happens; no crack ever shows, no memory of sub-threshold hits).
     *
     * effectiveJ = min(rawKImpact, event.threshold()) — unified with SublevelDamageAccumulator.
     * event.threshold() is the vanilla-data-driven break threshold including ConfinementFactor
     * scaling, computed by IntegratedImpactTerrain.onServerTickPost before this call.
     *
     * Creates a new entry if none exists for this (levelKey, pos, victimBlock) triplet;
     * otherwise adds effectiveDamageJ to the running total.
     *
     * @return the updated snapshot, or null if the hit was rejected by the elastic floor
     *         and no prior entry exists for this key.
     */
    public static Snapshot accumulate(DeferredDamageEvent event) {
        double rawImpact  = event.kImpact();
        double effectiveJ = Math.min(rawImpact, event.threshold());
        AccKey key = new AccKey(event.levelKey(),
                event.posX(), event.posY(), event.posZ(), event.victimBlock());

        if (!ImpactRuntimeConfig.ENABLE_DAMAGE_ACCUMULATION) {
            // Single-hit mode: no persistent state. Clear any stale entry left over from
            // a session where accumulation was previously enabled, then judge this hit
            // alone — hitCount=1, accumulated=effectiveJ, no carry-over.
            entries.remove(key);
            Snapshot oneShot = new Snapshot(key, event.materialClass(), effectiveJ, event.threshold(),
                    rawImpact, effectiveJ, event.serverTick(), 1,
                    DamageState.of(effectiveJ / event.threshold()));
            return oneShot;
        }

        // Elastic floor (fatigue limit), same model as the sublevel side: a single hit
        // below floor × threshold is purely elastic — no accumulation, no entry. Without
        // it, unbounded accumulation let arbitrarily small repeated contacts grind world
        // blocks to CRITICAL (observed: 2.74 J hits on a 130 J sand threshold, 12 hits
        // marching every resting-contact block toward destruction in lockstep).
        if (effectiveJ < ImpactRuntimeConfig.WORLD_ELASTIC_FLOOR * event.threshold()) {
            Entry existing = entries.get(key);
            return existing != null ? existing.toSnapshot(key) : null;
        }
        Entry entry = entries.get(key);
        if (entry == null) {
            entry = new Entry(event.materialClass(), event.threshold(),
                    rawImpact, effectiveJ, event.serverTick());
            entries.put(key, entry);
        } else {
            // Stress relaxation: accumulated damage halves every N ticks between hits.
            entry.decayTo(event.serverTick(), ImpactRuntimeConfig.WORLD_DAMAGE_HALF_LIFE_TICKS);
            entry.addImpact(rawImpact, effectiveJ, event.serverTick());
        }
        return entry.toSnapshot(key);
    }


    /**
     * Removes the entry for a single (levelKey, pos, victimBlock) key.
     * Called by IntegratedImpactTerrain after a block is destroyed by the damage pipeline,
     * so stale damage state for that position does not persist.
     */
    public static void removeEntry(AccKey key) {
        entries.remove(key);
    }

    /**
     * Clears all accumulated damage state.
     * Cleared by IntegratedImpactTerrain on server stop.
     */
    public static void clear() {
        entries.clear();
    }
}
