/*
 * Portions derived from True Impact 0.5.8-delta by OMEGAU371 / True Impact contributors.
 * Original project: https://github.com/OMEGAU371/sable-true-impact
 * Original license: LGPL-3.0-only.
 * PMWeather-IV modification notice: adapted for PMIV; notice updated 2026-10-09.
 * Adapted for PMWeather-IV's integrated aircraft/world material-response subsystem.
 */
package com.g9third.pmweatheriv.terrain.trueimpact.damage;

/**
 * Result of attempting to apply a DeferredDamageEvent's block effect.
 *
 * No Minecraft imports -- safe to unit-test without the game runtime.
 */
public enum ApplyOutcome {

    /** Block state was mutated (e.g., grass_block -> dirt). */
    APPLIED,

    /** SOFT_SOIL block, but no mutation needed for this specific block in Phase 2A. */
    APPLIED_NO_OP,

    /** ImpactRuntimeConfig.APPLY_BLOCK_EFFECTS is false. */
    SKIP_EFFECTS_DISABLED,

    /** event.kImpact() is NaN or non-finite. */
    SKIP_INVALID_ENERGY,

    /** event.kImpact() <= event.threshold() (impact too weak). */
    SKIP_BELOW_THRESHOLD,

    /** event.materialClass() not handled in Phase 2A (only SOFT_SOIL is). */
    SKIP_MATERIAL_CLASS,

    /** Chunk at event position not loaded in the target level. */
    SKIP_CHUNK_UNLOADED,

    /** Block at position no longer matches expected material class (stale event). */
    SKIP_BLOCK_MISMATCH,

    /** Block registry lookup for the target block failed. */
    SKIP_SET_FAILED;

}
