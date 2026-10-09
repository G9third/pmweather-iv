/*
 * Portions derived from True Impact 0.5.8-delta by OMEGAU371 / True Impact contributors.
 * Original project: https://github.com/OMEGAU371/sable-true-impact
 * Original license: LGPL-3.0-only.
 * PMWeather-IV modification notice: adapted for PMIV; notice updated 2026-10-09.
 * Adapted for PMWeather-IV's integrated aircraft/world material-response subsystem.
 */
package com.g9third.pmweatheriv.terrain.trueimpact.damage;

/**
 * Abstraction over block read/write operations.
 *
 * Decouples ImpactBlockApplicator from Minecraft's ServerLevel so the
 * applicator logic can be unit-tested without the game runtime.
 *
 * Concrete implementation: IntegratedImpactTerrain.ServerLevelBlockView (MC-dependent).
 * Test implementation: a simple map-backed mock.
 *
 * Block IDs use the full registry format: "minecraft:grass_block", etc.
 *
 * No Minecraft imports -- safe to unit-test without the game runtime.
 */
public interface BlockView {

    /** True if the chunk containing (x, y, z) is loaded and accessible. */
    boolean hasChunkAt(int x, int y, int z);

    /**
     * Returns the registry block ID at (x, y, z), e.g. "minecraft:grass_block".
     * Returns "minecraft:air" if the block is air or the position is unloaded.
     * Callers must guard with hasChunkAt() first.
     */
    String getBlockId(int x, int y, int z);

    /**
     * Replaces the block at (x, y, z) with the block identified by targetBlockId.
     * Returns true if the block was successfully set.
     * Returns false if the registry lookup failed or the set was rejected.
     */
    boolean setBlock(int x, int y, int z, String targetBlockId);
}
