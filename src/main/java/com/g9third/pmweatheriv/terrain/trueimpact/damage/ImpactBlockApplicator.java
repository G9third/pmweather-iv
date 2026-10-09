/*
 * Portions derived from True Impact 0.5.8-delta by OMEGAU371 / True Impact contributors.
 * Original project: https://github.com/OMEGAU371/sable-true-impact
 * Original license: LGPL-3.0-only.
 * PMWeather-IV modification notice: adapted for PMIV; notice updated 2026-10-09.
 * Adapted for PMWeather-IV's integrated aircraft/world material-response subsystem.
 */
package com.g9third.pmweatheriv.terrain.trueimpact.damage;

import java.util.concurrent.ThreadLocalRandom;

/** Applies surface compaction through an MC-free BlockView.
 * Rules and thresholds come from PMIV's local ImpactRuntimeConfig. The caller
 * handles fatigue accumulation, fracture, and overlays separately.
 */
public final class ImpactBlockApplicator {

    private ImpactBlockApplicator() {}

    /** Returns the first configured rule for the given block ID, or null if none. */
    public static CompactionRule findRule(String blockId) {
        for (CompactionRule rule : ImpactRuntimeConfig.COMPACTION_RULES) {
            if (rule.fromBlockId().equals(blockId)) return rule;
        }
        return null;
    }


    /**
     * Fast pre-check: validates all gates without world access.
     *
     * Returns an ApplyOutcome if any gate fails, or null if all gates pass
     * (meaning tryApply may proceed to world access).
     *
     * Fully MC-free and testable without a game level.
     */
    public static ApplyOutcome checkGates(DeferredDamageEvent event) {
        if (!ImpactRuntimeConfig.ENABLE_COMPACTION) {
            return ApplyOutcome.SKIP_EFFECTS_DISABLED;
        }
        if (!Double.isFinite(event.kImpact())) {
            return ApplyOutcome.SKIP_INVALID_ENERGY;
        }
        if (event.materialClass() != MaterialThresholdProfile.MaterialClass.SOFT_SOIL) {
            return ApplyOutcome.SKIP_MATERIAL_CLASS;
        }
        // A block with no configured rule has no threshold to check here -- it falls
        // through to tryApply's target lookup, which returns APPLIED_NO_OP for it.
        CompactionRule rule = findRule(event.victimBlock());
        if (rule != null && event.kImpact() <= rule.thresholdJ()) {
            return ApplyOutcome.SKIP_BELOW_THRESHOLD;
        }
        return null; // all gates passed
    }

    /**
     * Attempts to apply a block effect for the given event.
     *
     * 1. Runs checkGates (returns SKIP_* if any gate fails).
     * 2. Checks hasChunkAt (returns SKIP_CHUNK_UNLOADED if not loaded).
     * 3. Reads current block; verifies it is still SOFT_SOIL class
     *    (stale events where another process already changed the block are skipped).
     * 4. Looks up the configured rule for the victim block:
     *    - If none: returns APPLIED_NO_OP (SOFT_SOIL but no compaction rule defined).
     *    - If found: rolls the rule's probability; on success calls view.setBlock()
     *      and returns APPLIED or SKIP_SET_FAILED, on a failed roll returns APPLIED_NO_OP
     *      (this hit didn't transform the block, but may on a later hit).
     */
    public static ApplyOutcome tryApply(BlockView view, DeferredDamageEvent event) {
        ApplyOutcome gate = checkGates(event);
        if (gate != null) return gate;

        int x = event.posX(), y = event.posY(), z = event.posZ();
        if (!view.hasChunkAt(x, y, z)) return ApplyOutcome.SKIP_CHUNK_UNLOADED;

        // Revalidate: current block must still be SOFT_SOIL class.
        // If a previous tick already compacted grass->dirt, the current block is dirt
        // (still SOFT_SOIL), so repeated impacts won't re-apply the same effect.
        // If a completely different block replaced it, skip to avoid mismatched mutations.
        String currentBlock = view.getBlockId(x, y, z);
        MaterialThresholdProfile.MaterialClass currentClass =
                MaterialThresholdProfile.classify(currentBlock);
        if (currentClass != MaterialThresholdProfile.MaterialClass.SOFT_SOIL) {
            return ApplyOutcome.SKIP_BLOCK_MISMATCH;
        }

        // Look up the rule by ORIGINAL victim block (not current block). The original
        // block is recorded at enqueue time; use it to determine the effect even if the
        // block has already been compacted to an intermediate state.
        CompactionRule rule = findRule(event.victimBlock());
        if (rule == null) {
            // SOFT_SOIL but no compaction rule defined (e.g., dirt -> stays dirt)
            return ApplyOutcome.APPLIED_NO_OP;
        }
        if (rule.probability() < 1.0 && ThreadLocalRandom.current().nextDouble() >= rule.probability()) {
            return ApplyOutcome.APPLIED_NO_OP; // threshold cleared, but this hit's roll failed
        }

        boolean ok = view.setBlock(x, y, z, rule.toBlockId());
        return ok ? ApplyOutcome.APPLIED : ApplyOutcome.SKIP_SET_FAILED;
    }
}
