package com.g9third.pmweatheriv.terrain.trueimpact;

import net.minecraft.world.item.ItemStack;
import net.minecraft.world.item.Items;

/**
 * PMIV-owned defaults copied from True Impact 0.5.8-delta's world-impact configuration.
 *
 * <p>Portions derived from True Impact 0.5.8-delta by OMEGAU371 / True Impact contributors,
 * LGPL-3.0-only. PMIV keeps these defaults local so the published True Impact mod is not a
 * runtime dependency. The material formulas themselves live in the attributed derived classes
 * under {@code terrain.trueimpact.damage}.</p>
 * <p>PMWeather-IV modification notice updated 2026-10-09.</p>
 */
public final class IntegratedImpactSettings {
    private IntegratedImpactSettings() {}

    /** True Impact 0.5.8-delta default global enqueue/detection threshold. */
    public static final double GLOBAL_DETECTION_THRESHOLD_J = 40.0;

    /** Default True Impact preset × total multiplier × world multiplier = 1.0. */
    public static final double WORLD_BLOCK_DAMAGE_MULTIPLIER = 1.0;

    /** Force-to-mining-tier thresholds for deterministic terrain drops. */
    public static final double NETHERITE_PICKAXE_MAX_J = 100.0;
    public static final double DIAMOND_PICKAXE_MAX_J = 300.0;
    public static final double IRON_PICKAXE_MAX_J = 800.0;
    public static final double STONE_PICKAXE_MAX_J = 2_000.0;
    public static final double WOODEN_PICKAXE_MAX_J = 5_000.0;

    /**
     * True Impact's 0.5.8-era force-to-mining-tier drop policy, including its later
     * external-impact fix: resources are dropped with an explicit tool before the block
     * is removed, rather than destroyBlock(..., true) using an empty tool.
     */
    public static ItemStack resolveDropTool(double impactEnergyJ) {
        if (impactEnergyJ <= NETHERITE_PICKAXE_MAX_J) return new ItemStack(Items.NETHERITE_PICKAXE);
        if (impactEnergyJ <= DIAMOND_PICKAXE_MAX_J) return new ItemStack(Items.DIAMOND_PICKAXE);
        if (impactEnergyJ <= IRON_PICKAXE_MAX_J) return new ItemStack(Items.IRON_PICKAXE);
        if (impactEnergyJ <= STONE_PICKAXE_MAX_J) return new ItemStack(Items.STONE_PICKAXE);
        if (impactEnergyJ <= WOODEN_PICKAXE_MAX_J) return new ItemStack(Items.WOODEN_PICKAXE);
        return null;
    }
}
