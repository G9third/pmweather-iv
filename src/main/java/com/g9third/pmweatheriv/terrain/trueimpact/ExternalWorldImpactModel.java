/*
 * Derived from True Impact 0.5.8-delta ExternalWorldImpactApi by OMEGAU371 / True Impact contributors.
 * Original project: https://github.com/OMEGAU371/sable-true-impact
 * Original license: LGPL-3.0-only.
 * PMWeather-IV modification notice: adapted for PMIV; voxel-displacement and
 * inelastic-impact response updated 2026-10-09.
 * Adapted for PMWeather-IV so no modified True Impact runtime JAR is required.
 */
package com.g9third.pmweatheriv.terrain.trueimpact;

import com.g9third.pmweatheriv.terrain.trueimpact.damage.BlockDamageAccumulator;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.BlockHardnessProfile;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.ConfinementFactor;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.CrackOverlayTracker;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.DeferredDamageEvent;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.DeferredDamageQueue;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.ImpactRuntimeConfig;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.MaterialPropertiesProfile;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.MaterialResponsePlanner;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.MaterialThresholdProfile;
import net.minecraft.core.BlockPos;
import net.minecraft.core.registries.BuiltInRegistries;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.block.Block;
import net.minecraft.world.level.block.entity.BlockEntity;
import net.minecraft.world.item.ItemStack;
import net.minecraft.world.level.levelgen.Heightmap;
import net.minecraft.world.phys.AABB;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * PMIV-integrated True Impact 0.5.8 material-response surface for aircraft/world contacts.
 *
 * <p>The integrated True Impact-derived subsystem owns the terrain side of the interaction: impact-energy accounting,
 * material thresholds, confinement/overburden, immediate fracture of terrain that
 * cannot sustain a swept contact, and residual normal-energy calculation. The caller
 * remains the sole rigid-body momentum authority and applies the returned residual
 * normal speed/impulse to its own body.</p>
 *
 * <p>The penetration method is intended to run immediately before a caller commits a
 * swept rigid-body response on the server physics thread. It mutates only world blocks
 * that the material model proves have fractured. Surviving/partially damaged blocks are still
 * routed through the ordinary deferred damage queue for accumulation, overlays and
 * normal True Impact effects.</p>
 */
public final class ExternalWorldImpactModel {
    private static final double EPS = 1.0E-6;
    private static final double INDESTRUCTIBLE_BREAK_J = Double.MAX_VALUE * 0.25;

    private ExternalWorldImpactModel() {}

    public record Resolution(
            boolean accepted,
            boolean penetrationAttempted,
            boolean terrainOpened,
            double impactEnergyJ,
            double absorbedEnergyJ,
            double residualEnergyJ,
            double residualInwardSpeedMps,
            int blocksBroken,
            int terminalX,
            int terminalY,
            int terminalZ,
            double contactAreaM2,
            String mode
    ) {}

    /** Canonical external-contact energy definition owned by True Impact. */
    public static double deriveImpactEnergyJ(double normalImpulseNs, double inwardPointSpeedMps) {
        if (!Double.isFinite(normalImpulseNs) || !Double.isFinite(inwardPointSpeedMps)
                || normalImpulseNs <= 0.0 || inwardPointSpeedMps <= 0.0) {
            return 0.0;
        }
        return 0.5 * normalImpulseNs * inwardPointSpeedMps;
    }

    /**
     * Ordinary external contact path for collisions that have already been solved by
     * the rigid-body engine. Energy is derived here rather than by the integration mod.
     * Returns the accepted physical impact energy, or 0 when rejected/gated.
     */
    public static double enqueueExternalWorldImpact(
            ServerLevel level,
            int blockX, int blockY, int blockZ,
            double normalX, double normalY, double normalZ,
            double normalImpulseNs,
            double inwardPointSpeedMps
    ) {
        double energyJ = deriveImpactEnergyJ(normalImpulseNs, inwardPointSpeedMps);
        if (level == null || !(energyJ > IntegratedImpactSettings.GLOBAL_DETECTION_THRESHOLD_J)
                || (!ImpactRuntimeConfig.APPLY_BLOCK_EFFECTS
                    && !ImpactRuntimeConfig.ENABLE_COMPACTION)) {
            return 0.0;
        }
        BlockPos pos = new BlockPos(blockX, blockY, blockZ);
        if (!level.hasChunkAt(pos)) return 0.0;
        BlockState state = level.getBlockState(pos);
        if (state.isAir()) return 0.0;

        double[] dir = normalized(-normalX, -normalY, -normalZ);
        queueDamage(level, pos, state, energyJ,
                dir[0], dir[1], dir[2]);
        return energyJ;
    }

    /**
     * Resolves a destructive swept contact before the caller applies a rigid stop.
     *
     * <p>Only the physical distance the contact point can traverse during the current
     * integration slice is inspected, so there is no arbitrary block-count penetration
     * cap. Each loaded voxel in the projected leading footprint pays True Impact 0.5.8-delta's
     * confined/overburden-adjusted fracture threshold and the existing penetration
     * loss factor. This is the same material-cost model used by Phase 3C.</p>
     */
    public static Resolution resolveExternalPenetration(
            ServerLevel level,
            int seedBlockX, int seedBlockY, int seedBlockZ,
            double contactX, double contactY, double contactZ,
            double normalX, double normalY, double normalZ,
            double projectedContactAreaM2,
            double normalImpulseNs,
            double inwardPointSpeedMps,
            double maxTravelMeters
    ) {
        double energy0 = deriveImpactEnergyJ(normalImpulseNs, inwardPointSpeedMps);
        double area = finitePositive(projectedContactAreaM2) ? projectedContactAreaM2 : 1.0;
        if (level == null || !(energy0 > IntegratedImpactSettings.GLOBAL_DETECTION_THRESHOLD_J)
                || (!ImpactRuntimeConfig.APPLY_BLOCK_EFFECTS
                    && !ImpactRuntimeConfig.ENABLE_COMPACTION)) {
            return rejected(energy0, area, "REJECTED_DAMAGE_GATE");
        }

        double[] normal = normalized(normalX, normalY, normalZ);
        if (!finiteVector(normal)) {
            return rejected(energy0, area, "REJECTED_INVALID_NORMAL");
        }
        double dx = -normal[0], dy = -normal[1], dz = -normal[2];

        double effectiveMass = normalImpulseNs / Math.max(EPS, inwardPointSpeedMps);
        if (!finitePositive(effectiveMass)) {
            effectiveMass = 2.0 * energy0
                    / Math.max(EPS, inwardPointSpeedMps * inwardPointSpeedMps);
        }

        boolean penetrationRegime = ImpactRuntimeConfig.APPLY_BLOCK_EFFECTS
                && ImpactRuntimeConfig.ENABLE_BLOCK_BREAKING
                && ImpactRuntimeConfig.ENABLE_PENETRATION
                && energy0 >= ImpactRuntimeConfig.PENETRATION_TRIGGER_J
                && inwardPointSpeedMps >= ImpactRuntimeConfig.PENETRATION_MIN_SPEED_MS
                && finitePositive(maxTravelMeters);

        // The Sable sweep already identified the exact terrain voxel that produced
        // this contact. Use that voxel as the material-ray seed instead of deriving
        // it again from a floating-point support point. SAT support points can sit
        // on a voxel edge/corner; re-flooring them was able to select the adjacent
        // air cell and falsely report a "rigid terminal" with zero absorbed energy.
        BlockPos firstPos = new BlockPos(seedBlockX, seedBlockY, seedBlockZ);
        if (!penetrationRegime) {
            if (level.hasChunkAt(firstPos)) {
                BlockState state = level.getBlockState(firstPos);
                if (!state.isAir()) {
                    queueDamage(level, firstPos, state, energy0,
                            dx, dy, dz);
                }
            }
            return new Resolution(true, false, false, energy0, energy0, 0.0, 0.0,
                    0, firstPos.getX(), firstPos.getY(), firstPos.getZ(), area,
                    "DEFERRED_NON_PENETRATING_CONTACT");
        }

        double worldMultiplier = worldDamageMultiplier();

        double lossFactor = Math.max(1.0, ImpactRuntimeConfig.PENETRATION_LOSS_FACTOR);
        double remainingEnergy = energy0;
        double travelLimit = Math.max(EPS, maxTravelMeters);
        double traveled = 0.0;
        int broken = 0;
        int terminalX = Integer.MIN_VALUE;
        int terminalY = Integer.MIN_VALUE;
        int terminalZ = Integer.MIN_VALUE;

        // Match the existing Phase-3C philosophy: penetration pays the confined
        // break threshold of every loaded cell in the leading footprint, multiplied
        // by the configured penetration loss factor.  Contact area determines the
        // number of world cells that must yield at a layer; depth is still bounded
        // only by the physical normal travel available during this integration slice.
        List<int[]> footprint = footprintOffsets(
                area, dx, dy, dz, contactX, contactY, contactZ,
                seedBlockX, seedBlockY, seedBlockZ);
        // Start inside the exact swept voxel. Clamp all three coordinates because
        // a cuboid support point may lie on a neighboring voxel face even though
        // SAT identified this seed block as the actual collision partner.
        double px = clampInsideVoxel(contactX + dx * 1.0E-4, seedBlockX);
        double py = clampInsideVoxel(contactY + dy * 1.0E-4, seedBlockY);
        double pz = clampInsideVoxel(contactZ + dz * 1.0E-4, seedBlockZ);
        Set<Long> processedBlocks = new HashSet<>();
        boolean terminalReached = false;

        while (traveled < travelLimit - EPS && remainingEnergy > EPS && !terminalReached) {
            int bx = floor(px), by = floor(py), bz = floor(pz);
            double toExit = distanceToVoxelExit(px, py, pz, bx, by, bz, dx, dy, dz);
            if (!Double.isFinite(toExit) || toExit <= EPS) toExit = 1.0E-4;
            double segment = Math.min(toExit, travelLimit - traveled);

            for (int[] offset : footprint) {
                BlockPos pos = new BlockPos(bx + offset[0], by + offset[1], bz + offset[2]);
                long packed = pos.asLong();
                if (!processedBlocks.add(packed)) continue;

                if (!level.hasChunkAt(pos)) {
                    terminalX = pos.getX(); terminalY = pos.getY(); terminalZ = pos.getZ();
                    remainingEnergy = 0.0;
                    terminalReached = true;
                    break;
                }

                BlockState state = level.getBlockState(pos);
                if (state.isAir()) continue;

                String blockId = BuiltInRegistries.BLOCK.getKey(state.getBlock()).toString();
                MaterialThresholdProfile.MaterialClass mc = MaterialThresholdProfile.classify(blockId);
                float hardness = state.getDestroySpeed(level, pos);
                float blast = state.getBlock().getExplosionResistance();
                double baseBreakJ = BlockHardnessProfile.breakThresholdJ(hardness, blast);
                if (hardness < 0.0F || !Double.isFinite(baseBreakJ)
                        || baseBreakJ >= INDESTRUCTIBLE_BREAK_J) {
                    terminalX = pos.getX(); terminalY = pos.getY(); terminalZ = pos.getZ();
                    queueDamage(level, pos, state, remainingEnergy,
                            dx, dy, dz);
                    remainingEnergy = 0.0;
                    terminalReached = true;
                    break;
                }

                double crackJ = BlockHardnessProfile.crackThresholdJ(hardness, blast);
                double[] neighbors = sampleFaceNeighborCracks(level, pos);
                double confinement = ConfinementFactor.compute(neighbors, crackJ, dx, dy, dz);
                int surfaceY = sampleUndisturbedSurfaceY(
                        level, pos.getX(), pos.getZ(), dx, dy, dz, area);
                double overburdenDepth = Math.max(0.0, surfaceY - pos.getY());
                double overburdenJ = ConfinementFactor.overburdenEnergyJ(
                        overburdenDepth, MaterialPropertiesProfile.densityKgM3(mc, blast));
                double confinedBreakJ = baseBreakJ * (1.0 + confinement) + overburdenJ;
                double physicalBreakJ = confinedBreakJ / worldMultiplier;
                double materialCostJ = physicalBreakJ * lossFactor;

                if (!(remainingEnergy >= materialCostJ)) {
                    terminalX = pos.getX(); terminalY = pos.getY(); terminalZ = pos.getZ();
                    queueDamage(level, pos, state, remainingEnergy,
                            dx, dy, dz);
                    remainingEnergy = 0.0;
                    terminalReached = true;
                    break;
                }

                String victimBlock = blockId;
                // Reuse stock True Impact 0.5.8-delta's configured drop-mode behavior rather than
                // letting the PMIV external path invent a second loot policy.
                ItemStack dropTool = IntegratedImpactSettings.resolveDropTool(remainingEnergy);
                if (dropTool != null) {
                    BlockEntity blockEntity = state.hasBlockEntity() ? level.getBlockEntity(pos) : null;
                    Block.dropResources(state, level, pos, blockEntity, null, dropTool);
                }
                // The broken solid's collision volume is removed in this mutation.
                // Capture its bounded voxel-local volume before changing the state;
                // empty collision shapes (including fluids) carry no displaced mass.
                double displacedMassKg = displacedSolidMassKg(level, pos, state, mc, blast);
                // Immediate penetration can remove hundreds of voxels in a
                // high-energy aircraft crash. Level.destroyBlock emits a vanilla
                // 2001 break effect for every voxel in addition to the block
                // update, which floods the client with redundant particle/sound
                // work and forces avoidable render-thread pressure. Drops were
                // already handled above, so perform the same world mutation and
                // full neighbor/client update without the per-block break effect.
                boolean destroyed = level.setBlock(
                    pos, state.getFluidState().createLegacyBlock(), Block.UPDATE_ALL
                );
                if (!destroyed) {
                    terminalX = pos.getX(); terminalY = pos.getY(); terminalZ = pos.getZ();
                    queueDamage(level, pos, state, remainingEnergy,
                            dx, dy, dz);
                    remainingEnergy = 0.0;
                    terminalReached = true;
                    break;
                }
                broken++;
                clearDamageState(level, pos, victimBlock);
                double afterFracture = Math.max(0.0, remainingEnergy - materialCostJ);
                remainingEnergy = inelasticContactEnergy(afterFracture, effectiveMass, displacedMassKg);
                if (remainingEnergy <= EPS) {
                    terminalX = pos.getX(); terminalY = pos.getY(); terminalZ = pos.getZ();
                    terminalReached = true;
                    break;
                }
            }

            if (terminalReached) break;
            double advance = segment + 1.0E-5;
            px += dx * advance;
            py += dy * advance;
            pz += dz * advance;
            traveled += segment;
        }

        double residualSpeed = remainingEnergy > 0.0
                ? Math.sqrt(2.0 * remainingEnergy / Math.max(EPS, effectiveMass))
                : 0.0;
        residualSpeed = Math.min(inwardPointSpeedMps, Math.max(0.0, residualSpeed));
        double absorbed = Math.max(0.0, energy0 - remainingEnergy);
        String mode;
        if (terminalReached) {
            mode = "RIGID_TERMINAL_MATERIAL";
        } else if (broken > 0) {
            mode = "IMMEDIATE_MATERIAL_PENETRATION";
        } else {
            mode = "SLICE_TRAVEL_EXHAUSTED_NO_MATERIAL";
        }
        return new Resolution(true, true, broken > 0,
                energy0, absorbed, remainingEnergy, residualSpeed, broken,
                terminalX, terminalY, terminalZ, area, mode);
    }

    private static Resolution rejected(double energyJ, double area, String mode) {
        return new Resolution(false, false, false, Math.max(0.0, energyJ),
                0.0, Math.max(0.0, energyJ), 0.0, 0,
                Integer.MIN_VALUE, Integer.MIN_VALUE, Integer.MIN_VALUE,
                area, mode);
    }

    private static void queueDamage(
            ServerLevel level, BlockPos pos, BlockState state, double physicalEnergyJ,
            double dirX, double dirY, double dirZ
    ) {
        if (level == null || pos == null || state == null || state.isAir()
                || !Double.isFinite(physicalEnergyJ) || physicalEnergyJ <= 0.0) return;
        String blockId = BuiltInRegistries.BLOCK.getKey(state.getBlock()).toString();
        MaterialThresholdProfile.MaterialClass mc = MaterialThresholdProfile.classify(blockId);
        double baseBreakJ = BlockHardnessProfile.breakThresholdJ(
                state.getDestroySpeed(level, pos), state.getBlock().getExplosionResistance());
        double worldMult = worldDamageMultiplier();
        DeferredDamageQueue.enqueue(new DeferredDamageEvent(
                level.getServer().getTickCount(),
                level.dimension().location().toString(),
                blockId, pos.getX(), pos.getY(), pos.getZ(),
                mc, physicalEnergyJ * worldMult, baseBreakJ,
                dirX, dirY, dirZ));
    }

    private static void clearDamageState(ServerLevel level, BlockPos pos, String victimBlock) {
        BlockDamageAccumulator.AccKey key = new BlockDamageAccumulator.AccKey(
                level.dimension().location().toString(),
                pos.getX(), pos.getY(), pos.getZ(), victimBlock);
        BlockDamageAccumulator.removeEntry(key);
        int breakerId = CrackOverlayTracker.removeEntry(key);
        if (breakerId != Integer.MIN_VALUE) {
            level.destroyBlockProgress(breakerId, pos, -1);
        }
        MaterialResponsePlanner.forgetKey(key);
    }

    /**
     * Estimates the solid material mass removed with one voxel. Minecraft's
     * collision-shape AABBs are block-local; summing their clipped volumes is
     * bounded to one voxel and excludes non-colliding shapes and fluid volume.
     * The inelastic energy factor treats ejected material momentum as unavailable
     * to the vehicle; PMIV does not create fragment entities or model deformation.
     */
    private static double displacedSolidMassKg(
            ServerLevel level, BlockPos pos, BlockState state,
            MaterialThresholdProfile.MaterialClass materialClass, float blastResistance) {
        if (level == null || pos == null || state == null || state.isAir()) return 0.0;
        var shape = state.getCollisionShape(level, pos);
        if (shape.isEmpty()) return 0.0;
        double volume = 0.0;
        for (AABB box : shape.toAabbs()) {
            if (box == null || !Double.isFinite(box.minX) || !Double.isFinite(box.minY)
                    || !Double.isFinite(box.minZ) || !Double.isFinite(box.maxX)
                    || !Double.isFinite(box.maxY) || !Double.isFinite(box.maxZ)) continue;
            double width = Math.max(0.0, Math.min(1.0, box.maxX) - Math.max(0.0, box.minX));
            double height = Math.max(0.0, Math.min(1.0, box.maxY) - Math.max(0.0, box.minY));
            double depth = Math.max(0.0, Math.min(1.0, box.maxZ) - Math.max(0.0, box.minZ));
            double boxVolume = width * height * depth;
            if (Double.isFinite(boxVolume)) volume += boxVolume;
        }
        volume = Math.max(0.0, Math.min(1.0, volume));
        if (!(volume > 0.0)) return 0.0;
        double density = MaterialPropertiesProfile.densityKgM3(materialClass, blastResistance);
        double mass = density * volume;
        return finitePositive(mass) ? mass : 0.0;
    }

    /** The normal residual is the sole body response; displacement removes energy inelastically. */
    private static double inelasticContactEnergy(double energyAfterFractureJ,
                                                 double effectiveMassKg,
                                                 double displacedMassKg) {
        if (!(energyAfterFractureJ > 0.0) || !Double.isFinite(energyAfterFractureJ)) return 0.0;
        if (!(displacedMassKg > 0.0) || !Double.isFinite(displacedMassKg)) return energyAfterFractureJ;
        if (!(effectiveMassKg > 0.0) || !Double.isFinite(effectiveMassKg)) return energyAfterFractureJ;
        double ratio = effectiveMassKg / (effectiveMassKg + displacedMassKg);
        if (!Double.isFinite(ratio)) return 0.0;
        ratio = Math.max(0.0, Math.min(1.0, ratio));
        return energyAfterFractureJ * ratio * ratio;
    }

    /** Invalid world multipliers fall back to neutral damage scaling. */
    private static double worldDamageMultiplier() {
        double configured = IntegratedImpactSettings.WORLD_BLOCK_DAMAGE_MULTIPLIER;
        return finitePositive(configured) ? configured : 1.0;
    }

    /**
     * Conservative grid footprint derived from the collider's projected contact area.
     * The footprint is oriented on the block-grid plane perpendicular to the dominant
     * contact normal and is capped by True Impact 0.5.8-delta's existing Phase-3C radius setting.
     */
    private static List<int[]> footprintOffsets(
            double areaM2, double dx, double dy, double dz,
            double contactX, double contactY, double contactZ,
            int seedX, int seedY, int seedZ) {
        int radius = Math.max(0, ImpactRuntimeConfig.PENETRATION_FOOTPRINT_RADIUS);
        List<int[]> out = new ArrayList<>();
        out.add(new int[] {0, 0, 0});
        if (radius == 0) return out;

        // True Impact historically converted projected area directly to a count of
        // whole voxel columns. For a 0.3-0.9 m^2 aircraft patch that rounded to one
        // column even when the physical patch straddled a Minecraft cell edge, so
        // Rapier could hit the adjacent solid column in the same substep. Preserve
        // the same projected area, but place an equivalent square continuously on
        // the block grid and include every voxel column it actually overlaps.
        double side = Math.sqrt(Math.max(1.0E-6, areaM2));
        double half = 0.5 * side;
        double ax = Math.abs(dx), ay = Math.abs(dy), az = Math.abs(dz);

        if (ay >= ax && ay >= az) {
            appendFootprintPlane(out, seedX, seedZ,
                    floor(contactX - half), floor(contactX + half - 1.0E-9),
                    floor(contactZ - half), floor(contactZ + half - 1.0E-9),
                    radius, 0);
        } else if (ax >= az) {
            appendFootprintPlane(out, seedY, seedZ,
                    floor(contactY - half), floor(contactY + half - 1.0E-9),
                    floor(contactZ - half), floor(contactZ + half - 1.0E-9),
                    radius, 1);
        } else {
            appendFootprintPlane(out, seedX, seedY,
                    floor(contactX - half), floor(contactX + half - 1.0E-9),
                    floor(contactY - half), floor(contactY + half - 1.0E-9),
                    radius, 2);
        }
        return out;
    }

    private static void appendFootprintPlane(
            List<int[]> out, int seedA, int seedB,
            int minA, int maxA, int minB, int maxB,
            int radius, int plane) {
        for (int a = minA; a <= maxA; ++a) {
            for (int b = minB; b <= maxB; ++b) {
                int da = a - seedA;
                int db = b - seedB;
                if (Math.abs(da) > radius || Math.abs(db) > radius) continue;
                int ox, oy, oz;
                if (plane == 0) { ox = da; oy = 0; oz = db; }
                else if (plane == 1) { ox = 0; oy = da; oz = db; }
                else { ox = da; oy = db; oz = 0; }
                boolean duplicate = false;
                for (int[] existing : out) {
                    if (existing[0] == ox && existing[1] == oy && existing[2] == oz) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate) out.add(new int[] {ox, oy, oz});
            }
        }
    }

    private static double[] sampleFaceNeighborCracks(ServerLevel level, BlockPos pos) {
        int[][] dirs = {{0,1,0},{0,-1,0},{0,0,-1},{0,0,1},{1,0,0},{-1,0,0}};
        double[] out = new double[dirs.length];
        for (int i = 0; i < dirs.length; ++i) {
            BlockPos n = pos.offset(dirs[i][0], dirs[i][1], dirs[i][2]);
            if (!level.hasChunkAt(n)) { out[i] = 0.0; continue; }
            BlockState s = level.getBlockState(n);
            if (s.isAir()) { out[i] = 0.0; continue; }
            float h = s.getDestroySpeed(level, n);
            float b = s.getBlock().getExplosionResistance();
            out[i] = BlockHardnessProfile.crackThresholdJ(h, b);
        }
        return out;
    }

    private static int sampleUndisturbedSurfaceY(
            ServerLevel level, int x, int z,
            double dx, double dy, double dz,
            double areaM2
    ) {
        boolean vertical = Math.abs(dy) >= Math.abs(dx) && Math.abs(dy) >= Math.abs(dz);
        int lateral = Math.max(2, (int) Math.ceil(Math.sqrt(Math.max(1.0, areaM2))) + 2);
        int sx = x + (vertical ? lateral : 0);
        int sz = z + (vertical ? 0 : lateral);
        return level.getHeight(Heightmap.Types.MOTION_BLOCKING_NO_LEAVES, sx, sz);
    }


    private static double distanceToVoxelExit(
            double x, double y, double z,
            int bx, int by, int bz,
            double dx, double dy, double dz
    ) {
        double tx = axisExit(x, bx, dx);
        double ty = axisExit(y, by, dy);
        double tz = axisExit(z, bz, dz);
        return Math.min(tx, Math.min(ty, tz));
    }

    private static double axisExit(double p, int cell, double d) {
        if (d > EPS) return Math.max(0.0, (cell + 1.0 - p) / d);
        if (d < -EPS) return Math.max(0.0, (cell - p) / d);
        return Double.POSITIVE_INFINITY;
    }

    private static double clampInsideVoxel(double value, int cell) {
        double min = cell + 1.0E-6;
        double max = cell + 1.0 - 1.0E-6;
        if (!Double.isFinite(value)) return 0.5 * (min + max);
        return Math.max(min, Math.min(max, value));
    }

    private static int floor(double v) {
        return (int) Math.floor(v);
    }

    private static boolean finitePositive(double value) {
        return Double.isFinite(value) && value > 0.0;
    }

    private static boolean finiteVector(double[] v) {
        return v != null && v.length >= 3
                && Double.isFinite(v[0]) && Double.isFinite(v[1]) && Double.isFinite(v[2])
                && v[0] * v[0] + v[1] * v[1] + v[2] * v[2] > 0.5;
    }

    private static double[] normalized(double x, double y, double z) {
        double len2 = x * x + y * y + z * z;
        if (!Double.isFinite(len2) || len2 <= EPS * EPS) {
            return new double[] {Double.NaN, Double.NaN, Double.NaN};
        }
        double inv = 1.0 / Math.sqrt(len2);
        return new double[] {x * inv, y * inv, z * inv};
    }
}
