package com.g9third.pmweatheriv.sable;

import java.util.Arrays;
import java.util.Set;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import minecrafttransportsimulator.jsondefs.JSONCollisionGroup.CollisionType;

/**
 * Stable, value-based collision-topology hashing for IV aircraft.
 *
 * <p>IV is free to recreate equivalent BoundingBox objects while updating its
 * collision state. Java object identity is therefore deliberately excluded from
 * every signature here. Part ownership is represented by the authored placement
 * path plus definition identity, and unordered box/device collections are sorted
 * before they are folded into the final signature.</p>
 */
public final class StableCollisionTopology {
    private static final double SIGNATURE_SCALE = 1_000_000.0;
    private static final long FNV_OFFSET = 0xcbf29ce484222325L;
    private static final long FNV_PRIME = 0x100000001b3L;

    private StableCollisionTopology() {
    }

    public static long boxSignature(
        double localX,
        double localY,
        double localZ,
        double widthRadius,
        double heightRadius,
        double depthRadius,
        Set<CollisionType> collisionTypes,
        long semanticOwnerSignature
    ) {
        long hash = FNV_OFFSET;
        hash = fnv(hash, semanticOwnerSignature);
        hash = fnv(hash, stableBits(localX));
        hash = fnv(hash, stableBits(localY));
        hash = fnv(hash, stableBits(localZ));
        hash = fnv(hash, stableBits(widthRadius));
        hash = fnv(hash, stableBits(heightRadius));
        hash = fnv(hash, stableBits(depthRadius));
        hash = fnv(hash, collisionTypeSignature(collisionTypes));
        return hash;
    }

    public static long aggregate(long seed, long[] unorderedSignatures, int count) {
        if (unorderedSignatures == null || count <= 0) {
            return fnv(seed, 0L);
        }
        int safeCount = Math.min(count, unorderedSignatures.length);
        long[] ordered = Arrays.copyOf(unorderedSignatures, safeCount);
        Arrays.sort(ordered);
        long hash = fnv(seed, safeCount);
        for (long signature : ordered) {
            hash = fnv(hash, signature);
        }
        return hash;
    }

    public static long vehicleFrameOwnerSignature() {
        return hashString("vehicle-frame");
    }

    /** Stable authored placement path. No runtime object identity is used. */
    public static long partSemanticSignature(APart part) {
        if (part == null) {
            return vehicleFrameOwnerSignature();
        }
        APart[] chain = new APart[32];
        int depth = 0;
        APart cursor = part;
        while (cursor != null && depth < chain.length) {
            chain[depth++] = cursor;
            cursor = cursor.partOn;
        }

        long hash = fnv(FNV_OFFSET, depth);
        for (int index = depth - 1; index >= 0; --index) {
            APart node = chain[index];
            hash = fnv(hash, node.placementSlot);
            if (node.placementDefinition != null && node.placementDefinition.pos != null) {
                hash = fnv(hash, stableBits(node.placementDefinition.pos.x));
                hash = fnv(hash, stableBits(node.placementDefinition.pos.y));
                hash = fnv(hash, stableBits(node.placementDefinition.pos.z));
            }
            if (node.definition != null) {
                hash = fnv(hash, hashString(node.definition.packID));
                hash = fnv(hash, hashString(node.definition.systemName));
            }
            if (node.subDefinition != null) {
                hash = fnv(hash, hashString(node.subDefinition.subName));
            }
        }
        return hash;
    }

    /**
     * Stable authored rolling-device membership only.
     *
     * <p>Rolling ground devices are live Sable point constraints, not mounted
     * collider children. Their suspension/wheelbase point, active animation,
     * width and height can vary at runtime without changing compound collision
     * geometry. Hashing those live values invalidates and rebuilds otherwise
     * identical compound colliders during ordinary animation.
     * Definition/placement identity plus the static wheel/tread role is enough
     * to detect a different authored device.</p>
     */
    public static long groundDeviceSignature(PartGroundDevice device) {
        if (device == null) {
            return fnv(FNV_OFFSET, 0L);
        }
        long hash = fnv(FNV_OFFSET, partSemanticSignature(device));
        hash = fnv(hash, device.isSpare ? 1L : 0L);
        if (device.definition != null && device.definition.ground != null) {
            hash = fnv(hash, device.definition.ground.isWheel ? 1L : 0L);
            hash = fnv(hash, device.definition.ground.isTread ? 1L : 0L);
        }
        return hash;
    }

    public static long collisionTypeSignature(Set<CollisionType> collisionTypes) {
        if (collisionTypes == null || collisionTypes.isEmpty()) {
            return 0L;
        }
        String[] names = new String[collisionTypes.size()];
        int index = 0;
        for (CollisionType type : collisionTypes) {
            names[index++] = type == null ? "<null>" : type.name();
        }
        Arrays.sort(names);
        long hash = fnv(FNV_OFFSET, names.length);
        for (String name : names) {
            hash = fnv(hash, hashString(name));
        }
        return hash;
    }

    private static long hashString(String value) {
        String safe = value == null ? "" : value;
        long hash = FNV_OFFSET;
        for (int i = 0; i < safe.length(); ++i) {
            hash = fnv(hash, safe.charAt(i));
        }
        return hash;
    }

    private static long stableBits(double value) {
        if (!Double.isFinite(value)) {
            return Double.doubleToLongBits(0.0);
        }
        double stable = Math.rint(value * SIGNATURE_SCALE) / SIGNATURE_SCALE;
        return Double.doubleToLongBits(stable);
    }

    private static long fnv(long hash, long value) {
        hash ^= value;
        return hash * FNV_PRIME;
    }
}
