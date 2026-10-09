package com.g9third.pmweatheriv.sable;

import java.util.ArrayList;
import java.util.List;
import it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import java.util.Arrays;
import net.minecraft.world.level.levelgen.Heightmap;
import minecrafttransportsimulator.baseclasses.BoundingBox;
import net.minecraft.core.BlockPos;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.chunk.LevelChunk;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.shapes.VoxelShape;
import org.joml.Quaterniond;
import org.joml.Vector3d;
import org.joml.Vector3dc;

import static com.g9third.pmweatheriv.sable.SableCompoundCollider.*;

/** Terrain parent-pose sweeps and SAT queries over the current mounted BODY geometry.
 * Native collider creation, topology and gameplay attribution remain in the owner.
 * Child-relative poses are held at their current owner-tick values: this does not
 * sweep a child's articulated motion between owner ticks.
 */
final class SableTerrainSweep {
    private final List<PreparedChild> children;
    private final double collisionBoundingRadius;

    SableTerrainSweep(List<TerrainChild> sourceChildren, double radius) {
        this.children = prepareChildren(sourceChildren);
        this.collisionBoundingRadius = radius;
    }

    record TerrainChild(RelativePose lastPose, List<LocalCuboid> localCuboids,
                        List<BoundingBox> sourceBoxes) {}

    /**
     * Child-relative transforms and inset half extents are constant for every pose
     * query inside one sweep/recovery operation. Prepare them once rather than
     * re-transforming all ~200 aircraft cuboids for every one of the dozens of
     * high-resolution CCD samples.
     */
    private record PreparedChild(Quaterniond childOrientation, List<PreparedCuboid> cuboids) {}
    private record PreparedCuboid(
        double cxParent, double cyParent, double czParent,
        double hx, double hy, double hz,
        double contactHx, double contactHy, double contactHz,
        BoundingBox sourceBox
    ) {}

    private static List<PreparedChild> prepareChildren(List<TerrainChild> sourceChildren) {
        if (sourceChildren == null || sourceChildren.isEmpty()) {
            return List.of();
        }
        List<PreparedChild> preparedChildren = new ArrayList<>(sourceChildren.size());
        for (TerrainChild child : sourceChildren) {
            if (child == null || child.lastPose() == null
                || child.localCuboids() == null || child.localCuboids().isEmpty()) {
                continue;
            }
            Quaterniond childOrientation =
                new Quaterniond(child.lastPose().orientation()).normalize();
            List<PreparedCuboid> cuboids = new ArrayList<>(child.localCuboids().size());
            for (int localIndex = 0; localIndex < child.localCuboids().size(); ++localIndex) {
                LocalCuboid local = child.localCuboids().get(localIndex);
                if (local == null) {
                    continue;
                }
                Vector3d centerParent = childOrientation.transform(
                    new Vector3d(local.cx(), local.cy(), local.cz())
                ).add(child.lastPose().position());
                BoundingBox sourceBox = child.sourceBoxes() != null
                    && localIndex < child.sourceBoxes().size()
                    ? child.sourceBoxes().get(localIndex) : null;
                cuboids.add(new PreparedCuboid(
                    centerParent.x, centerParent.y, centerParent.z,
                    insetHalfExtent(local.hx()),
                    insetHalfExtent(local.hy()),
                    insetHalfExtent(local.hz()),
                    local.hx(), local.hy(), local.hz(),
                    sourceBox
                ));
            }
            if (!cuboids.isEmpty()) {
                preparedChildren.add(new PreparedChild(
                    new Quaterniond(childOrientation), List.copyOf(cuboids)
                ));
            }
        }
        return List.copyOf(preparedChildren);
    }

    // Rapier/Sable 2.0.3 exposes no CCD switch for these mounted LevelColliders.
    // A small inward skin distinguishes genuine compound penetration from the
    // contact slop required by the native solver. The recovery scan is only a
    // last-clear-pose safety boundary; Rapier remains impact/velocity authority.
    private static final double DEEP_PENETRATION_SKIN_METERS = 0.02;
    private static final double DEEP_PENETRATION_SKIN_FRACTION = 0.25;
    private static final int DEEP_PENETRATION_BINARY_STEPS = 14;
    private static final double DEEP_PENETRATION_REFINEMENT_EPSILON_METERS = 0.00025;
    // The 20260821 trace captured a thin wing crossing a 0.125 m terrain shape
    // completely between configured Sable physics substeps. Endpoint overlap alone cannot
    // detect that case, so contact-triggered recovery samples the complete
    // prior-to-completed parent rigid pose path before refining its first boundary.
    private static final double TERRAIN_SWEEP_MAX_TRAVEL_PER_CHECK_METERS = 0.025;
    private static final int MAX_TERRAIN_SWEEP_STEPS = 96;
    private static final double TERRAIN_VERTICAL_ESCAPE_STEP_METERS = 0.25;
    private static final int MAX_TERRAIN_VERTICAL_ESCAPE_STEPS = 512;
    private static final int MAX_TERRAIN_BLOCK_POSITIONS_PER_POSE = 16_384;

    /**
     * True when the inset physical BODY compound overlaps solid terrain at a
     * candidate parent pose. Used only by one-time spawn seating so lowering an
     * aircraft onto its authored gear can never teleport belly/tail geometry
     * through the runway.
     */
    boolean bodyIntersectsTerrainAtPose(
        ServerLevel level, Vector3dc parentWorldPosition, Quaterniond parentWorldOrientation
    ) {
        if (level == null || parentWorldPosition == null || parentWorldOrientation == null
            || !finite(parentWorldPosition) || !finite(parentWorldOrientation)) {
            return true;
        }
        TerrainPoseCheck check = deeplyIntersectsTerrain(
            level, parentWorldPosition, parentWorldOrientation
        );
        return check.truncated() || check.deepIntersection();
    }

    /**
     * Finds the first terrain boundary crossed between the prior clear pose and
     * a completed Rapier pose.
     *
     * <p>Every mounted BODY cuboid is transformed exactly through its current child
     * pose and the candidate parent pose. Its slightly inset OBB is tested
     * against the actual voxel-shape AABBs. Ordinary touching/contact slop does
     * not qualify. The parent rigid-pose path is sampled within the fixed travel/step
     * budget before bisection, so
     * a thin rotating box cannot pass completely through a thin terrain shape
     * and appear clear at both endpoints. If no prior clear pose remains, a
     * bounded world-up search provides a final terrain escape for already
     * embedded wrecks. The method never invents an impulse and never changes a
     * velocity; callers decide whether a completed Rapier rebound is already
     * sufficient or whether a missed/tunnelled parent crossing still needs its inward
     * penetration component removed at the returned boundary. Child-relative motion
     * itself is not continuous collision detection.</p>
     */
    TerrainPoseRecovery recoverDeepTerrainPenetration(
        ServerLevel level,
        Vector3dc priorClearPosition,
        Quaterniond priorClearOrientation,
        Vector3dc completedPosition,
        Quaterniond completedOrientation
    ) {
        if (level == null || priorClearPosition == null || priorClearOrientation == null
            || completedPosition == null || completedOrientation == null
            || !finite(priorClearPosition) || !finite(priorClearOrientation)
            || !finite(completedPosition) || !finite(completedOrientation)) {
            return TerrainPoseRecovery.invalid();
        }

        return recoverDeepTerrainPenetration(
            level, priorClearPosition, priorClearOrientation,
            completedPosition, completedOrientation, new TerrainBlockCache(level)
        );
    }

    /**
     * A pre-integration material response may fracture several voxels and then
     * re-query the same compound along the same predicted path. Keep the prepared
     * child geometry and exact Minecraft collision AABBs for that whole operation;
     * the session drops cached world shapes after each synchronous terrain mutation
     * before another query can use them.
     */
    TerrainQuerySession newQuerySession(ServerLevel level) {
        return new TerrainQuerySession(level);
    }

    final class TerrainQuerySession {
        private final ServerLevel level;
        private final TerrainBlockCache terrainCache;

        private TerrainQuerySession(ServerLevel level) {
            this.level = level;
            this.terrainCache = new TerrainBlockCache(level);
        }

        List<TerrainBoundaryImpact> rigidContactsAtPose(Vector3dc position, Quaterniond orientation) {
            List<TerrainBoundaryImpact> contacts = new ArrayList<>();
            terrainBoundaryImpactAtPose(level, position, orientation, terrainCache, contacts);
            return contacts;
        }

        TerrainPoseRecovery recoverDeepTerrainPenetration(
            Vector3dc priorClearPosition,
            Quaterniond priorClearOrientation,
            Vector3dc completedPosition,
            Quaterniond completedOrientation
        ) {
            return SableTerrainSweep.this.recoverDeepTerrainPenetration(
                level, priorClearPosition, priorClearOrientation,
                completedPosition, completedOrientation, terrainCache
            );
        }

        TerrainBoundaryImpact findFirstTerrainImpactAlongPath(
            Vector3dc priorPosition,
            Quaterniond priorOrientation,
            Vector3dc completedPosition,
            Quaterniond completedOrientation
        ) {
            if (level == null || priorPosition == null || priorOrientation == null
                || completedPosition == null || completedOrientation == null
                || !finite(priorPosition) || !finite(priorOrientation)
                || !finite(completedPosition) || !finite(completedOrientation)) {
                return null;
            }
            return SableTerrainSweep.this.findFirstTerrainImpactAlongPath(
                level, priorPosition, priorOrientation, completedPosition,
                completedOrientation, terrainCache
            );
        }

        /** Drop all world shapes after a synchronous terrain mutation. */
        void invalidateAfterMaterialFracture() {
            terrainCache.clear();
        }

    }

    private TerrainPoseRecovery recoverDeepTerrainPenetration(
        ServerLevel level,
        Vector3dc priorClearPosition,
        Quaterniond priorClearOrientation,
        Vector3dc completedPosition,
        Quaterniond completedOrientation,
        TerrainBlockCache terrainCache
    ) {
        Vector3d clearPosition = new Vector3d(priorClearPosition);
        Quaterniond clearOrientation = new Quaterniond(priorClearOrientation).normalize();
        Vector3d currentPosition = new Vector3d(completedPosition);
        Quaterniond currentOrientation = new Quaterniond(completedOrientation).normalize();
        TerrainPoseCheck prior = deeplyIntersectsTerrain(
            level, clearPosition, clearOrientation, terrainCache
        );
        int poseChecks = 1;
        int blockPositions = prior.blockPositions();
        int collisionShapes = prior.collisionShapes();
        if (prior.truncated()) {
            return new TerrainPoseRecovery(
                true, false, false, true, currentPosition, currentOrientation,
                1.0, poseChecks, blockPositions, collisionShapes
            );
        }

        if (prior.deepIntersection()) {
            TerrainPoseCheck current = deeplyIntersectsTerrain(
                level, currentPosition, currentOrientation, terrainCache
            );
            ++poseChecks;
            blockPositions += current.blockPositions();
            collisionShapes += current.collisionShapes();
            if (current.truncated()) {
                return new TerrainPoseRecovery(
                    true, false, false, true, currentPosition, currentOrientation,
                    1.0, poseChecks, blockPositions, collisionShapes
                );
            }
            if (!current.deepIntersection()) {
                return new TerrainPoseRecovery(
                    false, false, false, false, currentPosition, currentOrientation,
                    1.0, poseChecks, blockPositions, collisionShapes
                );
            }

            // A persisted/previously missed wreck may no longer have a usable
            // last-clear endpoint. Search upward with its completed orientation
            // until the full inset compound is clear, then refine the minimum
            // translation. This is only reached after real terrain evidence.
            double requiredEscapeMeters = Math.max(
                2.0,
                2.0 * Math.max(0.0, collisionBoundingRadius) + 1.0
            );
            int escapeSteps = Math.min(
                MAX_TERRAIN_VERTICAL_ESCAPE_STEPS,
                Math.max(1, (int) Math.ceil(
                    requiredEscapeMeters / TERRAIN_VERTICAL_ESCAPE_STEP_METERS
                ))
            );
            double embeddedHeight = 0.0;
            double clearHeight = Double.NaN;
            for (int step = 1; step <= escapeSteps; ++step) {
                double candidateHeight = step * TERRAIN_VERTICAL_ESCAPE_STEP_METERS;
                Vector3d candidatePosition = new Vector3d(currentPosition)
                    .add(0.0, candidateHeight, 0.0);
                TerrainPoseCheck candidate = deeplyIntersectsTerrain(
                    level, candidatePosition, currentOrientation, terrainCache
                );
                ++poseChecks;
                blockPositions += candidate.blockPositions();
                collisionShapes += candidate.collisionShapes();
                if (candidate.truncated()) {
                    return new TerrainPoseRecovery(
                        true, false, false, true, currentPosition, currentOrientation,
                        1.0, poseChecks, blockPositions, collisionShapes
                    );
                }
                if (!candidate.deepIntersection()) {
                    clearHeight = candidateHeight;
                    break;
                }
                embeddedHeight = candidateHeight;
            }
            if (!Double.isFinite(clearHeight)) {
                return new TerrainPoseRecovery(
                    true, false, false, false, currentPosition, currentOrientation,
                    1.0, poseChecks, blockPositions, collisionShapes
                );
            }
            for (int i = 0; i < DEEP_PENETRATION_BINARY_STEPS; ++i) {
                if (clearHeight - embeddedHeight <= DEEP_PENETRATION_REFINEMENT_EPSILON_METERS) {
                    break;
                }
                double candidateHeight = 0.5 * (embeddedHeight + clearHeight);
                Vector3d candidatePosition = new Vector3d(currentPosition)
                    .add(0.0, candidateHeight, 0.0);
                TerrainPoseCheck candidate = deeplyIntersectsTerrain(
                    level, candidatePosition, currentOrientation, terrainCache
                );
                ++poseChecks;
                blockPositions += candidate.blockPositions();
                collisionShapes += candidate.collisionShapes();
                if (candidate.truncated()) {
                    return new TerrainPoseRecovery(
                        true, false, false, true, currentPosition, currentOrientation,
                        1.0, poseChecks, blockPositions, collisionShapes
                    );
                }
                if (candidate.deepIntersection()) {
                    embeddedHeight = candidateHeight;
                } else {
                    clearHeight = candidateHeight;
                }
            }
            Vector3d resolvedPosition = new Vector3d(currentPosition)
                .add(0.0, clearHeight + 0.01, 0.0);
            TerrainBoundaryImpact preciseImpact = findFirstTerrainImpactAlongPath(
                level, clearPosition, clearOrientation, currentPosition, currentOrientation, terrainCache
            );
            return new TerrainPoseRecovery(
                true, true, false, false, resolvedPosition, currentOrientation,
                0.0, poseChecks, blockPositions, collisionShapes,
                preciseImpact
            );
        }

        double orientationDot = Math.min(1.0, Math.abs(
            clearOrientation.dot(currentOrientation)
        ));
        double angularTravel = 2.0 * Math.acos(orientationDot)
            * Math.max(0.0, collisionBoundingRadius);
        double travelMeters = clearPosition.distance(currentPosition) + angularTravel;
        int sweepSteps = Math.min(
            MAX_TERRAIN_SWEEP_STEPS,
            Math.max(1, (int) Math.ceil(
                travelMeters / TERRAIN_SWEEP_MAX_TRAVEL_PER_CHECK_METERS
            ))
        );
        double clearFraction = 0.0;
        double embeddedFraction = Double.NaN;
        TerrainBoundaryImpact embeddedImpact = null;
        for (int step = 1; step <= sweepSteps; ++step) {
            double candidateFraction = (double) step / (double) sweepSteps;
            Vector3d candidatePosition = new Vector3d(clearPosition)
                .lerp(currentPosition, candidateFraction);
            Quaterniond candidateOrientation = new Quaterniond(clearOrientation)
                .slerp(currentOrientation, candidateFraction)
                .normalize();
            TerrainPoseCheck candidate = deeplyIntersectsTerrain(
                level, candidatePosition, candidateOrientation, terrainCache
            );
            ++poseChecks;
            blockPositions += candidate.blockPositions();
            collisionShapes += candidate.collisionShapes();
            if (candidate.truncated()) {
                return new TerrainPoseRecovery(
                    true, false, true, true, currentPosition, currentOrientation,
                    1.0, poseChecks, blockPositions, collisionShapes
                );
            }
            if (candidate.deepIntersection()) {
                embeddedFraction = candidateFraction;
                embeddedImpact = candidate.impact();
                break;
            }
            clearFraction = candidateFraction;
        }
        if (!Double.isFinite(embeddedFraction)) {
            return new TerrainPoseRecovery(
                false, false, true, false, currentPosition, currentOrientation,
                1.0, poseChecks, blockPositions, collisionShapes
            );
        }

        for (int i = 0; i < DEEP_PENETRATION_BINARY_STEPS; ++i) {
            if ((embeddedFraction - clearFraction) * Math.max(0.0, travelMeters)
                <= DEEP_PENETRATION_REFINEMENT_EPSILON_METERS) {
                break;
            }
            double candidateFraction = 0.5 * (clearFraction + embeddedFraction);
            Vector3d candidatePosition = new Vector3d(clearPosition)
                .lerp(currentPosition, candidateFraction);
            Quaterniond candidateOrientation = new Quaterniond(clearOrientation)
                .slerp(currentOrientation, candidateFraction)
                .normalize();
            TerrainPoseCheck candidate = deeplyIntersectsTerrain(
                level, candidatePosition, candidateOrientation, terrainCache
            );
            ++poseChecks;
            blockPositions += candidate.blockPositions();
            collisionShapes += candidate.collisionShapes();
            if (candidate.truncated()) {
                return new TerrainPoseRecovery(
                    true, false, true, true, currentPosition, currentOrientation,
                    1.0, poseChecks, blockPositions, collisionShapes
                );
            }
            if (candidate.deepIntersection()) {
                embeddedFraction = candidateFraction;
                embeddedImpact = candidate.impact();
            } else {
                clearFraction = candidateFraction;
            }
        }

        double retreatFraction = Math.min(
            clearFraction,
            0.01 / Math.max(0.01, travelMeters)
        );
        double resolvedFraction = Math.max(0.0, clearFraction - retreatFraction);
        Vector3d resolvedPosition = new Vector3d(clearPosition)
            .lerp(currentPosition, resolvedFraction);
        Quaterniond resolvedOrientation = new Quaterniond(clearOrientation)
            .slerp(currentOrientation, resolvedFraction)
            .normalize();
        // The first deep sweep sample and the embedded side of the binary
        // refinement already identify the physical cuboid/block at the first
        // compound boundary. Replaying the entire path at twice the sampling
        // density only to rediscover that same hit was a major hard-crash CPU
        // cost. Carry the refined boundary impact forward instead.
        TerrainBoundaryImpact preciseImpact = embeddedImpact;
        if (preciseImpact == null) {
            Vector3d embeddedPosition = new Vector3d(clearPosition)
                .lerp(currentPosition, embeddedFraction);
            Quaterniond embeddedOrientation = new Quaterniond(clearOrientation)
                .slerp(currentOrientation, embeddedFraction).normalize();
            preciseImpact = terrainBoundaryImpactAtPose(
                level, embeddedPosition, embeddedOrientation, terrainCache
            );
        }
        return new TerrainPoseRecovery(
            true, true, true, false, resolvedPosition, resolvedOrientation,
            resolvedFraction, poseChecks, blockPositions, collisionShapes,
            preciseImpact
        );
    }

    /**
     * Replays the same inset BODY/voxel test along a recovered substep path and
     * returns the first concrete IV source box/block that crossed terrain. The
     * recovery path calls the same high-resolution replay with its operation-local
     * terrain-shape cache, preserving contact precision without re-fetching the
     * same Minecraft block states/voxel shapes. This is called only after the safety sweep has already proven a boundary
     * violation, so it cannot create a new collision by itself.
     */
    TerrainBoundaryImpact findFirstTerrainImpactAlongPath(
        ServerLevel level,
        Vector3dc priorPosition,
        Quaterniond priorOrientation,
        Vector3dc completedPosition,
        Quaterniond completedOrientation
    ) {
        if (level == null || priorPosition == null || priorOrientation == null
            || completedPosition == null || completedOrientation == null
            || !finite(priorPosition) || !finite(priorOrientation)
            || !finite(completedPosition) || !finite(completedOrientation)) {
            return null;
        }
        return findFirstTerrainImpactAlongPath(
            level, priorPosition, priorOrientation, completedPosition, completedOrientation,
            new TerrainBlockCache(level)
        );
    }

    private TerrainBoundaryImpact findFirstTerrainImpactAlongPath(
        ServerLevel level,
        Vector3dc priorPosition,
        Quaterniond priorOrientation,
        Vector3dc completedPosition,
        Quaterniond completedOrientation,
        TerrainBlockCache terrainCache
    ) {
        Quaterniond startOrientation = new Quaterniond(priorOrientation).normalize();
        Quaterniond endOrientation = new Quaterniond(completedOrientation).normalize();
        double orientationDot = Math.min(1.0, Math.abs(startOrientation.dot(endOrientation)));
        double angularTravel = 2.0 * Math.acos(orientationDot)
            * Math.max(0.0, collisionBoundingRadius);
        double travelMeters = priorPosition.distance(completedPosition) + angularTravel;
        int steps = Math.min(
            MAX_TERRAIN_SWEEP_STEPS * 2,
            Math.max(1, (int) Math.ceil(
                travelMeters / Math.max(0.005, TERRAIN_SWEEP_MAX_TRAVEL_PER_CHECK_METERS * 0.5)
            ))
        );
        for (int step = 0; step <= steps; ++step) {
            double fraction = (double) step / (double) steps;
            Vector3d position = new Vector3d(priorPosition).lerp(completedPosition, fraction);
            Quaterniond orientation = new Quaterniond(startOrientation)
                .slerp(endOrientation, fraction).normalize();
            TerrainBoundaryImpact impact = terrainBoundaryImpactAtPose(
                level, position, orientation, terrainCache
            );
            if (impact != null) {
                return impact;
            }
        }
        return null;
    }

    private TerrainBoundaryImpact terrainBoundaryImpactAtPose(
        ServerLevel level,
        Vector3dc parentWorldPosition,
        Quaterniond parentWorldOrientation,
        TerrainBlockCache terrainCache
    ) {
        return terrainBoundaryImpactAtPose(level, parentWorldPosition, parentWorldOrientation,
            terrainCache, null);
    }

    private TerrainBoundaryImpact terrainBoundaryImpactAtPose(
        ServerLevel level, Vector3dc parentWorldPosition, Quaterniond parentWorldOrientation,
        TerrainBlockCache terrainCache, List<TerrainBoundaryImpact> contacts
    ) {
        int candidateBlockPositions = 0;
        Quaterniond parentOrientation = new Quaterniond(parentWorldOrientation).normalize();
        TerrainBoundaryImpact bestImpact = null;
        double bestPenetrationMeters = Double.POSITIVE_INFINITY;

        for (PreparedChild child : children) {
            Quaterniond worldOrientation = new Quaterniond(parentOrientation)
                .mul(child.childOrientation()).normalize();
            Vector3d axisX = worldOrientation.transform(new Vector3d(1.0, 0.0, 0.0));
            Vector3d axisY = worldOrientation.transform(new Vector3d(0.0, 1.0, 0.0));
            Vector3d axisZ = worldOrientation.transform(new Vector3d(0.0, 0.0, 1.0));

            for (PreparedCuboid local : child.cuboids()) {
                Vector3d centerWorld = parentOrientation.transform(
                    new Vector3d(local.cxParent(), local.cyParent(), local.czParent())
                ).add(parentWorldPosition);
                // Recovery uses the inset shell. Rigid support must cover the actual
                // native envelope, including faces already touching beside the hit.
                double hx = contacts == null ? local.hx() : local.contactHx() + 0.001;
                double hy = contacts == null ? local.hy() : local.contactHy() + 0.001;
                double hz = contacts == null ? local.hz() : local.contactHz() + 0.001;
                OrientedCuboid cuboid = new OrientedCuboid(
                    centerWorld, axisX, axisY, axisZ, hx, hy, hz
                );
                double extentX = Math.abs(axisX.x) * hx
                    + Math.abs(axisY.x) * hy + Math.abs(axisZ.x) * hz;
                double extentY = Math.abs(axisX.y) * hx
                    + Math.abs(axisY.y) * hy + Math.abs(axisZ.y) * hz;
                double extentZ = Math.abs(axisX.z) * hx
                    + Math.abs(axisY.z) * hy + Math.abs(axisZ.z) * hz;
                int minX = floorBlock(centerWorld.x - extentX);
                int minY = floorBlock(centerWorld.y - extentY);
                int minZ = floorBlock(centerWorld.z - extentZ);
                int maxX = floorBlock(centerWorld.x + extentX - EPSILON);
                int maxY = floorBlock(centerWorld.y + extentY - EPSILON);
                int maxZ = floorBlock(centerWorld.z + extentZ - EPSILON);
                long positionCount = (long) (maxX - minX + 1)
                    * (long) (maxY - minY + 1)
                    * (long) (maxZ - minZ + 1);
                if (positionCount < 0L
                    || positionCount > MAX_TERRAIN_BLOCK_POSITIONS_PER_POSE - candidateBlockPositions) {
                    return bestImpact;
                }
                if (terrainCache.allCandidateBlocksAboveTerrain(minX, minY, minZ, maxX, maxZ)) {
                    candidateBlockPositions += (int) positionCount;
                    continue;
                }
                for (int x = minX; x <= maxX; ++x) {
                    for (int y = minY; y <= maxY; ++y) {
                        for (int z = minZ; z <= maxZ; ++z) {
                            ++candidateBlockPositions;
                            for (AABB worldShape : terrainCache.worldCollisionShapes(x, y, z)) {
                                SatContact contact = satContact(cuboid, worldShape);
                                if (contact == null || (contacts == null
                                    && contact.penetrationMeters() >= bestPenetrationMeters)) {
                                    continue;
                                }
                                BoundingBox sourceBox = local.sourceBox();
                                Vector3d impactNormal = new Vector3d(contact.normalWorld());
                                Vector3d impactPoint = supportPoint(
                                    cuboid, new Vector3d(impactNormal).negate()
                                );
                                TerrainBoundaryImpact impact = new TerrainBoundaryImpact(
                                    new minecrafttransportsimulator.baseclasses.Point3D(x, y, z),
                                    sourceBox, impactPoint, impactNormal,
                                    projectedAreaSquareMeters(cuboid, impactNormal)
                                );
                                if (contacts != null) contacts.add(impact);
                                if (contact.penetrationMeters() < bestPenetrationMeters) {
                                    bestPenetrationMeters = contact.penetrationMeters();
                                    bestImpact = impact;
                                }
                            }
                        }
                    }
                }
            }
        }
        return bestImpact;
    }

    /**
     * Full OBB-vs-AABB separating-axis contact query. Detection alone is not
     * enough for a swept rigid-body response: the old centre-to-AABB normal could
     * point along an unrelated face for a thin rotated wing, making the measured
     * inward point speed zero even though the sweep proved terrain crossing.
     * The minimum-overlap SAT axis is the geometric contact/separating normal.
     */
    private static SatContact satContact(OrientedCuboid cuboid, AABB box) {
        double bx = 0.5 * (box.maxX - box.minX);
        double by = 0.5 * (box.maxY - box.minY);
        double bz = 0.5 * (box.maxZ - box.minZ);
        double dx = 0.5 * (box.minX + box.maxX) - cuboid.center().x;
        double dy = 0.5 * (box.minY + box.maxY) - cuboid.center().y;
        double dz = 0.5 * (box.minZ + box.maxZ) - cuboid.center().z;

        Vector3d x = cuboid.axisX();
        Vector3d y = cuboid.axisY();
        Vector3d z = cuboid.axisZ();
        double bestOverlap = Double.POSITIVE_INFINITY;
        double bestX = 0.0, bestY = 1.0, bestZ = 0.0;
        double overlap;

        overlap = axisOverlapMeters(cuboid, bx, by, bz, dx, dy, dz, x.x, x.y, x.z);
        if (overlap < -1.0E-9) return null;
        if (overlap < bestOverlap) { bestOverlap = overlap; bestX = x.x; bestY = x.y; bestZ = x.z; }
        overlap = axisOverlapMeters(cuboid, bx, by, bz, dx, dy, dz, y.x, y.y, y.z);
        if (overlap < -1.0E-9) return null;
        if (overlap < bestOverlap) { bestOverlap = overlap; bestX = y.x; bestY = y.y; bestZ = y.z; }
        overlap = axisOverlapMeters(cuboid, bx, by, bz, dx, dy, dz, z.x, z.y, z.z);
        if (overlap < -1.0E-9) return null;
        if (overlap < bestOverlap) { bestOverlap = overlap; bestX = z.x; bestY = z.y; bestZ = z.z; }

        overlap = axisOverlapMeters(cuboid, bx, by, bz, dx, dy, dz, 1.0, 0.0, 0.0);
        if (overlap < -1.0E-9) return null;
        if (overlap < bestOverlap) { bestOverlap = overlap; bestX = 1.0; bestY = 0.0; bestZ = 0.0; }
        overlap = axisOverlapMeters(cuboid, bx, by, bz, dx, dy, dz, 0.0, 1.0, 0.0);
        if (overlap < -1.0E-9) return null;
        if (overlap < bestOverlap) { bestOverlap = overlap; bestX = 0.0; bestY = 1.0; bestZ = 0.0; }
        overlap = axisOverlapMeters(cuboid, bx, by, bz, dx, dy, dz, 0.0, 0.0, 1.0);
        if (overlap < -1.0E-9) return null;
        if (overlap < bestOverlap) { bestOverlap = overlap; bestX = 0.0; bestY = 0.0; bestZ = 1.0; }

        // Nine OBB-axis x world-axis candidates. Keep these scalar to avoid
        // allocating temporary arrays in the terrain sweep's hottest predicate.
        overlap = axisOverlapMeters(cuboid, bx, by, bz, dx, dy, dz, 0.0, x.z, -x.y);
        if (overlap < -1.0E-9) return null;
        if (overlap < bestOverlap) { bestOverlap = overlap; bestX = 0.0; bestY = x.z; bestZ = -x.y; }
        overlap = axisOverlapMeters(cuboid, bx, by, bz, dx, dy, dz, -x.z, 0.0, x.x);
        if (overlap < -1.0E-9) return null;
        if (overlap < bestOverlap) { bestOverlap = overlap; bestX = -x.z; bestY = 0.0; bestZ = x.x; }
        overlap = axisOverlapMeters(cuboid, bx, by, bz, dx, dy, dz, x.y, -x.x, 0.0);
        if (overlap < -1.0E-9) return null;
        if (overlap < bestOverlap) { bestOverlap = overlap; bestX = x.y; bestY = -x.x; bestZ = 0.0; }

        overlap = axisOverlapMeters(cuboid, bx, by, bz, dx, dy, dz, 0.0, y.z, -y.y);
        if (overlap < -1.0E-9) return null;
        if (overlap < bestOverlap) { bestOverlap = overlap; bestX = 0.0; bestY = y.z; bestZ = -y.y; }
        overlap = axisOverlapMeters(cuboid, bx, by, bz, dx, dy, dz, -y.z, 0.0, y.x);
        if (overlap < -1.0E-9) return null;
        if (overlap < bestOverlap) { bestOverlap = overlap; bestX = -y.z; bestY = 0.0; bestZ = y.x; }
        overlap = axisOverlapMeters(cuboid, bx, by, bz, dx, dy, dz, y.y, -y.x, 0.0);
        if (overlap < -1.0E-9) return null;
        if (overlap < bestOverlap) { bestOverlap = overlap; bestX = y.y; bestY = -y.x; bestZ = 0.0; }

        overlap = axisOverlapMeters(cuboid, bx, by, bz, dx, dy, dz, 0.0, z.z, -z.y);
        if (overlap < -1.0E-9) return null;
        if (overlap < bestOverlap) { bestOverlap = overlap; bestX = 0.0; bestY = z.z; bestZ = -z.y; }
        overlap = axisOverlapMeters(cuboid, bx, by, bz, dx, dy, dz, -z.z, 0.0, z.x);
        if (overlap < -1.0E-9) return null;
        if (overlap < bestOverlap) { bestOverlap = overlap; bestX = -z.z; bestY = 0.0; bestZ = z.x; }
        overlap = axisOverlapMeters(cuboid, bx, by, bz, dx, dy, dz, z.y, -z.x, 0.0);
        if (overlap < -1.0E-9) return null;
        if (overlap < bestOverlap) { bestOverlap = overlap; bestX = z.y; bestY = -z.x; bestZ = 0.0; }

        if (!Double.isFinite(bestOverlap)) {
            return null;
        }
        double length = Math.sqrt(bestX * bestX + bestY * bestY + bestZ * bestZ);
        if (!(length > 1.0E-12) || !Double.isFinite(length)) {
            return null;
        }
        double nx = bestX / length;
        double ny = bestY / length;
        double nz = bestZ / length;
        double signedTerrainFromBody = dx * nx + dy * ny + dz * nz;
        // Normal points from terrain toward the aircraft.
        if (signedTerrainFromBody > 0.0) {
            nx = -nx; ny = -ny; nz = -nz;
        }
        return new SatContact(
            new Vector3d(nx, ny, nz), Math.max(0.0, bestOverlap)
        );
    }

    private static double axisOverlapMeters(
        OrientedCuboid cuboid,
        double boxHalfX, double boxHalfY, double boxHalfZ,
        double centerDeltaX, double centerDeltaY, double centerDeltaZ,
        double axisX, double axisY, double axisZ
    ) {
        double lengthSquared = axisX * axisX + axisY * axisY + axisZ * axisZ;
        if (lengthSquared <= 1.0E-18) {
            return Double.POSITIVE_INFINITY;
        }
        double inverseLength = 1.0 / Math.sqrt(lengthSquared);
        double nx = axisX * inverseLength;
        double ny = axisY * inverseLength;
        double nz = axisZ * inverseLength;
        double centerDistance = Math.abs(
            centerDeltaX * nx + centerDeltaY * ny + centerDeltaZ * nz
        );
        double cuboidRadius = cuboid.hx() * Math.abs(cuboid.axisX().dot(nx, ny, nz))
            + cuboid.hy() * Math.abs(cuboid.axisY().dot(nx, ny, nz))
            + cuboid.hz() * Math.abs(cuboid.axisZ().dot(nx, ny, nz));
        double boxRadius = boxHalfX * Math.abs(nx)
            + boxHalfY * Math.abs(ny) + boxHalfZ * Math.abs(nz);
        return cuboidRadius + boxRadius - centerDistance;
    }

    private static Vector3d supportPoint(OrientedCuboid cuboid, Vector3dc direction) {
        Vector3d point = new Vector3d(cuboid.center());
        addSupportComponent(point, cuboid.axisX(), cuboid.hx(), direction);
        addSupportComponent(point, cuboid.axisY(), cuboid.hy(), direction);
        addSupportComponent(point, cuboid.axisZ(), cuboid.hz(), direction);
        return point;
    }

    private static void addSupportComponent(
        Vector3d point, Vector3dc axis, double halfExtent, Vector3dc direction
    ) {
        double projection = direction.dot(axis);
        if (projection > 1.0E-12) {
            point.fma(halfExtent, axis);
        } else if (projection < -1.0E-12) {
            point.fma(-halfExtent, axis);
        }
    }

    private static boolean intersects(OrientedCuboid cuboid, AABB box) {
        return satContact(cuboid, box) != null;
    }

    private record SatContact(Vector3d normalWorld, double penetrationMeters) {
    }

    private static double projectedAreaSquareMeters(
        OrientedCuboid cuboid, Vector3dc surfaceNormalWorld
    ) {
        if (cuboid == null || surfaceNormalWorld == null
            || surfaceNormalWorld.lengthSquared() <= EPSILON * EPSILON) {
            return 1.0;
        }
        Vector3d n = new Vector3d(surfaceNormalWorld).normalize();
        // Orthographic projection area of an oriented rectangular cuboid onto the
        // plane perpendicular to n. This is the physical leading area True Impact
        // uses to size the swept material-failure footprint.
        double area = 4.0 * (
            cuboid.hy() * cuboid.hz() * Math.abs(cuboid.axisX().dot(n))
                + cuboid.hx() * cuboid.hz() * Math.abs(cuboid.axisY().dot(n))
                + cuboid.hx() * cuboid.hy() * Math.abs(cuboid.axisZ().dot(n))
        );
        return Double.isFinite(area) && area > EPSILON ? area : 1.0;
    }

    private TerrainPoseCheck deeplyIntersectsTerrain(
        ServerLevel level,
        Vector3dc parentWorldPosition,
        Quaterniond parentWorldOrientation
    ) {
        return deeplyIntersectsTerrain(
            level, parentWorldPosition, parentWorldOrientation, new TerrainBlockCache(level)
        );
    }

    private TerrainPoseCheck deeplyIntersectsTerrain(
        ServerLevel level,
        Vector3dc parentWorldPosition,
        Quaterniond parentWorldOrientation,
        TerrainBlockCache terrainCache
    ) {
        int candidateBlockPositions = 0;
        int collisionShapeTests = 0;
        Quaterniond parentOrientation = new Quaterniond(parentWorldOrientation).normalize();

        for (PreparedChild child : children) {
            Quaterniond worldOrientation = new Quaterniond(parentOrientation)
                .mul(child.childOrientation())
                .normalize();
            Vector3d axisX = worldOrientation.transform(new Vector3d(1.0, 0.0, 0.0));
            Vector3d axisY = worldOrientation.transform(new Vector3d(0.0, 1.0, 0.0));
            Vector3d axisZ = worldOrientation.transform(new Vector3d(0.0, 0.0, 1.0));

            for (PreparedCuboid local : child.cuboids()) {
                Vector3d centerWorld = parentOrientation.transform(
                    new Vector3d(local.cxParent(), local.cyParent(), local.czParent())
                ).add(parentWorldPosition);
                double hx = local.hx();
                double hy = local.hy();
                double hz = local.hz();
                OrientedCuboid cuboid = new OrientedCuboid(
                    centerWorld, axisX, axisY, axisZ, hx, hy, hz
                );

                double extentX = Math.abs(axisX.x) * hx
                    + Math.abs(axisY.x) * hy + Math.abs(axisZ.x) * hz;
                double extentY = Math.abs(axisX.y) * hx
                    + Math.abs(axisY.y) * hy + Math.abs(axisZ.y) * hz;
                double extentZ = Math.abs(axisX.z) * hx
                    + Math.abs(axisY.z) * hy + Math.abs(axisZ.z) * hz;
                int minX = floorBlock(centerWorld.x - extentX);
                int minY = floorBlock(centerWorld.y - extentY);
                int minZ = floorBlock(centerWorld.z - extentZ);
                int maxX = floorBlock(centerWorld.x + extentX - EPSILON);
                int maxY = floorBlock(centerWorld.y + extentY - EPSILON);
                int maxZ = floorBlock(centerWorld.z + extentZ - EPSILON);

                long positionCount = (long) (maxX - minX + 1)
                    * (long) (maxY - minY + 1)
                    * (long) (maxZ - minZ + 1);
                if (positionCount < 0L
                    || positionCount > MAX_TERRAIN_BLOCK_POSITIONS_PER_POSE - candidateBlockPositions) {
                    return new TerrainPoseCheck(
                        false, true,
                        candidateBlockPositions,
                        collisionShapeTests
                    );
                }

                if (terrainCache.allCandidateBlocksAboveTerrain(minX, minY, minZ, maxX, maxZ)) {
                    candidateBlockPositions += (int) positionCount;
                    continue;
                }
                for (int x = minX; x <= maxX; ++x) {
                    for (int y = minY; y <= maxY; ++y) {
                        for (int z = minZ; z <= maxZ; ++z) {
                            ++candidateBlockPositions;
                            for (AABB worldShape : terrainCache.worldCollisionShapes(x, y, z)) {
                                ++collisionShapeTests;
                                SatContact contact = satContact(cuboid, worldShape);
                                if (contact != null) {
                                    BoundingBox sourceBox = local.sourceBox();
                                    Vector3d impactNormal = new Vector3d(contact.normalWorld());
                                    Vector3d impactPoint = supportPoint(
                                        cuboid, new Vector3d(impactNormal).negate()
                                    );
                                    TerrainBoundaryImpact impact = new TerrainBoundaryImpact(
                                        new minecrafttransportsimulator.baseclasses.Point3D(x, y, z),
                                        sourceBox, impactPoint, impactNormal,
                                        projectedAreaSquareMeters(cuboid, impactNormal)
                                    );
                                    return new TerrainPoseCheck(
                                        true, false,
                                        candidateBlockPositions,
                                        collisionShapeTests,
                                        impact
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
        return new TerrainPoseCheck(
            false, false,
            candidateBlockPositions,
            collisionShapeTests
        );
    }

    private static double insetHalfExtent(double halfExtent) {
        double inset = Math.min(
            DEEP_PENETRATION_SKIN_METERS,
            Math.max(0.0, halfExtent) * DEEP_PENETRATION_SKIN_FRACTION
        );
        return Math.max(EPSILON, halfExtent - inset);
    }

    private static int floorBlock(double coordinate) {
        return (int) Math.floor(coordinate);
    }

    /**
     * Operation-local immutable view of Minecraft terrain collision shapes.
     * Recovery/sweep poses may revisit the same blocks hundreds of times as
     * mounted cuboids overlap and the rigid pose is bisected. Fetch each block
     * state/voxel shape once per recovery operation, then reuse the exact world
     * AABBs. This changes no collision geometry or sweep spacing.
     */
    private static final class TerrainBlockCache {
        private static final List<AABB> EMPTY_SHAPES = List.of();

        private final ServerLevel level;
        /*
         * CCD revisits the same small set of 16^3 chunk sections across dozens
         * of nearby poses. A global block-key hash lookup for every candidate
         * voxel showed up as one of the largest remaining crash hot spots in the
         * 0.11.15 JFR. Hash only when crossing a section boundary, then address
         * the block directly inside a 4096-entry section-local array.
         */
        private final Long2ObjectOpenHashMap<SectionShapeCache> sections =
            new Long2ObjectOpenHashMap<>(16);
        private final Long2ObjectOpenHashMap<ChunkColumns> chunks = new Long2ObjectOpenHashMap<>(4);
        private long lastChunkKey = Long.MIN_VALUE;
        private ChunkColumns lastChunk;
        private final BlockPos.MutableBlockPos mutable = new BlockPos.MutableBlockPos();
        private long lastSectionKey = Long.MIN_VALUE;
        private SectionShapeCache lastSection;
        private int worldBlockLookups;
        private int cacheHits;
        private int collisionShapesLoaded;

        private TerrainBlockCache(ServerLevel level) {
            this.level = level;
        }

        /**
         * WORLD_SURFACE is one above the highest non-air block in a column. If
         * every candidate cell starts at or above it, the original voxel loop
         * would only read air. This skips no non-air voxel or collision shape;
         * the original exact shape/SAT path still handles all other candidates.
         */
        private boolean allCandidateBlocksAboveTerrain(int minX, int minY, int minZ,
                int maxX, int maxZ) {
            for (int x = minX; x <= maxX; ++x) {
                for (int z = minZ; z <= maxZ; ++z) {
                    if (minY < columnHeight(x, z)) return false;
                }
            }
            return true;
        }

        private List<AABB> worldCollisionShapes(int x, int y, int z) {
            // The exact WORLD_SURFACE height proves these cells are air even
            // when another column kept the complete cuboid on the SAT path.
            if (y >= columnHeight(x, z)) return EMPTY_SHAPES;
            int sectionX = x >> 4;
            int sectionY = y >> 4;
            int sectionZ = z >> 4;
            // BlockPos packing is collision-free for section coordinates inside
            // Minecraft's world border and avoids another allocation/API layer.
            long sectionKey = BlockPos.asLong(sectionX, sectionY, sectionZ);
            SectionShapeCache section;
            if (lastSection != null && sectionKey == lastSectionKey) {
                section = lastSection;
            } else {
                section = sections.get(sectionKey);
                if (section == null) {
                    section = new SectionShapeCache();
                    sections.put(sectionKey, section);
                }
                lastSectionKey = sectionKey;
                lastSection = section;
            }

            int localIndex = ((y & 15) << 8) | ((z & 15) << 4) | (x & 15);
            List<AABB> cached = section.worldShapes[localIndex];
            if (cached != null) {
                ++cacheHits;
                return cached;
            }

            ++worldBlockLookups;
            mutable.set(x, y, z);
            LevelChunk chunk = chunkColumns(x, z).chunk;
            BlockState state = chunk == null ? level.getBlockState(mutable) : chunk.getBlockState(mutable);
            if (state.isAir()) {
                section.worldShapes[localIndex] = EMPTY_SHAPES;
                return EMPTY_SHAPES;
            }
            VoxelShape shape = state.getCollisionShape(level, mutable);
            if (shape.isEmpty()) {
                section.worldShapes[localIndex] = EMPTY_SHAPES;
                return EMPTY_SHAPES;
            }
            List<AABB> localShapes = shape.toAabbs();
            if (localShapes.isEmpty()) {
                section.worldShapes[localIndex] = EMPTY_SHAPES;
                return EMPTY_SHAPES;
            }
            List<AABB> worldShapes = new ArrayList<>(localShapes.size());
            for (AABB localShape : localShapes) {
                worldShapes.add(localShape.move(x, y, z));
            }
            collisionShapesLoaded += worldShapes.size();
            List<AABB> frozen = List.copyOf(worldShapes);
            section.worldShapes[localIndex] = frozen;
            return frozen;
        }

        private void clear() {
            sections.clear();
            chunks.clear();
            lastChunk = null;
            lastChunkKey = Long.MIN_VALUE;
            lastSection = null;
            lastSectionKey = Long.MIN_VALUE;
        }

        private ChunkColumns chunkColumns(int x, int z) {
            long key = ((long) (x >> 4) << 32) | ((z >> 4) & 0xffffffffL);
            if (lastChunk != null && key == lastChunkKey) return lastChunk;
            ChunkColumns chunk = chunks.get(key);
            if (chunk == null) {
                chunk = new ChunkColumns(level.getChunkSource().getChunkNow(x >> 4, z >> 4));
                chunks.put(key, chunk);
            }
            lastChunkKey = key;
            lastChunk = chunk;
            return chunk;
        }

        private int columnHeight(int x, int z) {
            if (x < -30000000 || z < -30000000 || x >= 30000000 || z >= 30000000)
                return level.getHeight(Heightmap.Types.WORLD_SURFACE, x, z);
            ChunkColumns chunk = chunkColumns(x, z);
            int index = (x & 15) | ((z & 15) << 4);
            int height = chunk.heights[index];
            if (height == Integer.MIN_VALUE) {
                height = chunk.chunk == null ? level.getHeight(Heightmap.Types.WORLD_SURFACE, x, z)
                    : chunk.chunk.getHeight(Heightmap.Types.WORLD_SURFACE, x & 15, z & 15) + 1;
                chunk.heights[index] = height;
            }
            return height;
        }

        private static final class ChunkColumns {
            private final LevelChunk chunk;
            private final int[] heights = new int[256];
            private ChunkColumns(LevelChunk chunk) {
                this.chunk = chunk;
                Arrays.fill(heights, Integer.MIN_VALUE);
            }
        }

        private static final class SectionShapeCache {
            @SuppressWarnings("unchecked")
            private final List<AABB>[] worldShapes =
                (List<AABB>[]) new List<?>[16 * 16 * 16];
        }
    }

    private record TerrainPoseCheck(
        boolean deepIntersection,
        boolean truncated,
        int blockPositions,
        int collisionShapes,
        TerrainBoundaryImpact impact
    ) {
        private TerrainPoseCheck(
            boolean deepIntersection, boolean truncated, int blockPositions, int collisionShapes
        ) {
            this(deepIntersection, truncated, blockPositions, collisionShapes, null);
        }
    }

    private record OrientedCuboid(
        Vector3d center,
        Vector3d axisX,
        Vector3d axisY,
        Vector3d axisZ,
        double hx,
        double hy,
        double hz
    ) {
    }
}
