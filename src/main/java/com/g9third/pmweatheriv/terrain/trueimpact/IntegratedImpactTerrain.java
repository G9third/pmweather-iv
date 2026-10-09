/*
 * World-block damage/application flow derived from True Impact 0.5.8-delta
 * by OMEGAU371 / True Impact contributors.
 * Original project: https://github.com/OMEGAU371/sable-true-impact
 * Original license: LGPL-3.0-only.
 * PMWeather-IV modification notice: adapted for PMIV; notice updated 2026-10-09.
 * Adapted for PMWeather-IV so aircraft terrain response is self-contained.
 */
package com.g9third.pmweatheriv.terrain.trueimpact;

import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.ApplyOutcome;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.BlockDamageAccumulator;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.BlockHardnessProfile;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.BlockView;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.ConfinementFactor;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.CrackOverlayTracker;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.DamageFeedbackTracker;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.DamageState;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.DeferredDamageEvent;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.DeferredDamageQueue;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.ImpactBlockApplicator;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.ImpactRuntimeConfig;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.MaterialResponsePlanner;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.MaterialThresholdProfile;
import java.util.List;
import net.minecraft.core.BlockPos;
import net.minecraft.core.particles.BlockParticleOption;
import net.minecraft.core.particles.ParticleTypes;
import net.minecraft.core.registries.BuiltInRegistries;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.server.MinecraftServer;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.item.ItemStack;
import net.minecraft.world.level.block.Block;
import net.minecraft.world.level.block.entity.BlockEntity;
import net.minecraft.world.level.block.state.BlockState;
import net.neoforged.neoforge.event.server.ServerStoppedEvent;
import net.neoforged.neoforge.event.tick.ServerTickEvent;

/**
 * PMIV-owned application side of the True Impact 0.5.8-derived world material model.
 *
 * <p>Physics-thread callers only enqueue ordinary already-solved contacts or use
 * {@link ExternalWorldImpactModel#resolveExternalPenetration} for destructive swept
 * contacts. Deferred accumulation, compaction, crack overlays and final block mutation
 * happen here during {@link ServerTickEvent.Post}, matching True Impact's safe world-
 * mutation timing.</p>
 */
public final class IntegratedImpactTerrain {
    private IntegratedImpactTerrain() {}

    public static void onServerTickPost(ServerTickEvent.Post event) {
        if (event == null || event.getServer() == null) return;
        applyDeferredWorldDamage(event.getServer());
    }

    public static void onServerStopped(ServerStoppedEvent event) {
        clearState();
    }

    public static void clearState() {
        DeferredDamageQueue.clear();
        BlockDamageAccumulator.clear();
        MaterialResponsePlanner.clear();
        CrackOverlayTracker.clear();
        DamageFeedbackTracker.clear();
    }

    private static void applyDeferredWorldDamage(MinecraftServer server) {
        List<DeferredDamageEvent> events = DeferredDamageQueue.drainAll();
        if (events.isEmpty()) return;
        if (!ImpactRuntimeConfig.APPLY_BLOCK_EFFECTS && !ImpactRuntimeConfig.ENABLE_COMPACTION) {
            return;
        }

        int broken = 0;
        int compacted = 0;
        int cracked = 0;
        for (DeferredDamageEvent raw : events) {
            DeferredDamageEvent event = raw;
            ServerLevel level = findLevel(server, event.levelKey());
            if (level == null) {

                continue;
            }

            BlockPos pos = new BlockPos(event.posX(), event.posY(), event.posZ());
            if (!level.hasChunkAt(pos)) {

                continue;
            }

            BlockState current = level.getBlockState(pos);
            ResourceLocation currentKey = BuiltInRegistries.BLOCK.getKey(current.getBlock());
            String currentId = currentKey == null ? "minecraft:air" : currentKey.toString();
            if (!currentId.equals(event.victimBlock())) {
                clearStaleState(level, event);

                continue;
            }

            // True Impact 0.5.8 Path-1 threshold refinement: actual vanilla hardness/
            // blast resistance plus directional confinement from contiguous neighbors.
            float hardness = current.getDestroySpeed(level, pos);
            float blastResistance = current.getBlock().getExplosionResistance();
            double baseBreakJ = BlockHardnessProfile.breakThresholdJ(hardness, blastResistance);
            double victimCrackJ = BlockHardnessProfile.crackThresholdJ(hardness, blastResistance);
            int confinementRadius = ConfinementFactor.dynamicRadius(event.kImpact());
            double[] neighborCracks = sampleNeighborCrackThresholds(level, pos, confinementRadius);
            double confinement = ConfinementFactor.compute(
                neighborCracks, victimCrackJ,
                event.impactDirX(), event.impactDirY(), event.impactDirZ()
            );
            event = event.withThreshold(baseBreakJ * (1.0 + confinement));

            BlockDamageAccumulator.Snapshot snap = ImpactRuntimeConfig.APPLY_BLOCK_EFFECTS
                ? BlockDamageAccumulator.accumulate(event)
                : null;

            boolean criticalSoil = snap != null
                && snap.materialClass() == MaterialThresholdProfile.MaterialClass.SOFT_SOIL
                && snap.damageState() == DamageState.CRITICAL;

            ApplyOutcome outcome;
            if (criticalSoil) {
                outcome = ApplyOutcome.APPLIED_NO_OP;
            } else {
                outcome = ImpactBlockApplicator.tryApply(new ServerLevelBlockView(level), event);
            }

            if (outcome == ApplyOutcome.APPLIED) compacted++;

            if (snap == null) continue;
            boolean blockBroken = false;

            if (ImpactRuntimeConfig.ENABLE_BLOCK_BREAKING
                    && MaterialResponsePlanner.canBreak(snap)
                    && outcome != ApplyOutcome.APPLIED
                    && MaterialResponsePlanner.markBreakScheduled(snap.key())) {
                BlockState stateToBreak = level.getBlockState(pos);
                if (!stateToBreak.isAir()) {
                    ItemStack dropTool = IntegratedImpactSettings.resolveDropTool(event.kImpact());
                    if (dropTool != null) {
                        BlockEntity blockEntity = stateToBreak.hasBlockEntity()
                            ? level.getBlockEntity(pos) : null;
                        Block.dropResources(stateToBreak, level, pos, blockEntity, null, dropTool);
                    }
                    // Match the immediate penetration path: drops are already
                    // handled above, so mutate the block with normal client and
                    // neighbor updates but without one vanilla break-effect burst
                    // per deferred voxel. Large wrecks can otherwise keep the
                    // render thread busy with effects long after the main impact.
                    if (level.setBlock(
                            pos, stateToBreak.getFluidState().createLegacyBlock(), Block.UPDATE_ALL)) {
                        blockBroken = true;
                        broken++;
                        int breakerId = CrackOverlayTracker.removeEntry(snap.key());
                        if (breakerId != Integer.MIN_VALUE) {
                            level.destroyBlockProgress(breakerId, pos, -1);
                        }
                        BlockDamageAccumulator.removeEntry(snap.key());
                        MaterialResponsePlanner.forgetKey(snap.key());
                    }
                }
            }

            if (blockBroken) continue;

            if (DamageFeedbackTracker.shouldEmit(
                    event.levelKey(), event.posX(), event.posY(), event.posZ(),
                    snap.damageState(), server.getTickCount())) {
                emitDamageFeedback(level, pos, snap.damageState());
            }

            int crackProgress = CrackOverlayTracker.tryUpdate(
                snap.key(), snap.damageState(), snap.ratio(), server.getTickCount()
            );
            if (crackProgress >= 0) {
                level.destroyBlockProgress(
                    CrackOverlayTracker.fakeBreakerIdFor(snap.key()), pos, crackProgress
                );
                cracked++;
            }
        }

        if ((broken > 0 || compacted > 0) && ImpactRuntimeConfig.LOG_ENERGY_SUMMARY) {
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "INTEGRATED_TRUE_IMPACT_APPLY events=" + events.size()
                    + " blocksBroken=" + broken
                    + " blocksCompacted=" + compacted
                    + " crackUpdates=" + cracked
            );
            }
        }
    }

    private static void clearStaleState(ServerLevel level, DeferredDamageEvent event) {
        BlockDamageAccumulator.AccKey key = new BlockDamageAccumulator.AccKey(
            event.levelKey(), event.posX(), event.posY(), event.posZ(), event.victimBlock()
        );
        BlockDamageAccumulator.removeEntry(key);
        MaterialResponsePlanner.forgetKey(key);
        int breakerId = CrackOverlayTracker.removeEntry(key);
        if (breakerId != Integer.MIN_VALUE) {
            level.destroyBlockProgress(
                breakerId, new BlockPos(event.posX(), event.posY(), event.posZ()), -1
            );
        }
    }

    private static double[] sampleNeighborCrackThresholds(
            ServerLevel level, BlockPos center, int radius) {
        int[][] dirs = {{0,1,0},{0,-1,0},{0,0,-1},{0,0,1},{1,0,0},{-1,0,0}};
        double[] cracks = new double[6];
        int safeRadius = Math.max(1, radius);
        for (int i = 0; i < 6; i++) {
            double sum = 0.0;
            for (int d = 1; d <= safeRadius; d++) {
                BlockPos n = center.offset(
                    dirs[i][0] * d, dirs[i][1] * d, dirs[i][2] * d
                );
                if (!level.hasChunkAt(n)) break;
                BlockState state = level.getBlockState(n);
                if (state.isAir()) break;
                float h = state.getDestroySpeed(level, n);
                float b = state.getBlock().getExplosionResistance();
                sum += BlockHardnessProfile.crackThresholdJ(h, b);
            }
            cracks[i] = sum;
        }
        return cracks;
    }

    private static ServerLevel findLevel(MinecraftServer server, String levelKey) {
        for (ServerLevel level : server.getAllLevels()) {
            if (level.dimension().location().toString().equals(levelKey)) return level;
        }
        return null;
    }

    private static void emitDamageFeedback(
            ServerLevel level, BlockPos pos, DamageState state) {
        if (!level.hasChunkAt(pos)) return;
        BlockState blockState = level.getBlockState(pos);
        if (blockState.isAir()) return;
        int count = state == DamageState.CRITICAL ? 5 : 2;
        level.sendParticles(
            new BlockParticleOption(ParticleTypes.BLOCK, blockState),
            pos.getX() + 0.5, pos.getY() + 0.5, pos.getZ() + 0.5,
            count, 0.25, 0.25, 0.25, 0.0
        );
    }

    /** Minecraft-backed adapter copied from True Impact's Path-1 applicator. */
    private static final class ServerLevelBlockView implements BlockView {
        private final ServerLevel level;

        private ServerLevelBlockView(ServerLevel level) {
            this.level = level;
        }

        @Override
        public boolean hasChunkAt(int x, int y, int z) {
            return level.hasChunkAt(new BlockPos(x, y, z));
        }

        @Override
        public String getBlockId(int x, int y, int z) {
            BlockPos pos = new BlockPos(x, y, z);
            ResourceLocation id = BuiltInRegistries.BLOCK.getKey(level.getBlockState(pos).getBlock());
            return id == null ? "minecraft:air" : id.toString();
        }

        @Override
        public boolean setBlock(int x, int y, int z, String targetBlockId) {
            ResourceLocation id = ResourceLocation.tryParse(targetBlockId);
            if (id == null) return false;
            var block = BuiltInRegistries.BLOCK.getOptional(id);
            if (block.isEmpty()) return false;
            return level.setBlockAndUpdate(new BlockPos(x, y, z), block.get().defaultBlockState());
        }
    }
}
