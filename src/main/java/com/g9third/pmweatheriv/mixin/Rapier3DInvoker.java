package com.g9third.pmweatheriv.mixin;

import dev.ryanhcode.sable.api.physics.callback.BlockSubLevelCollisionCallback;
import net.minecraft.server.level.ServerLevel;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Invoker;

/**
 * Verified Sable 2.x runtime bridge used by PMWeather-IV's compound aircraft body.
 *
 * <p>PMWeather-IV deliberately targets Rapier3D by binary name rather than
 * importing Sable's internal Rapier implementation as a normal Java API.  The
 * exact invoker descriptors used here were compared against the supplied Sable
 * 2.0.3 and 2.0.5 sources and are unchanged.  Runtime metadata accepts the
 * compatible [2.0.5,3.0.0) line while the local development baseline is 2.0.5.</p>
 */
@Mixin(targets = "dev.ryanhcode.sable.physics.impl.rapier.Rapier3D", remap = false)
public interface Rapier3DInvoker {
    @Invoker("getSceneHandle")
    static long pmweatherIv$getSceneHandle(ServerLevel level) {
        throw new AssertionError();
    }

    @Invoker("nextBodyID")
    static int pmweatherIv$nextBodyId() {
        throw new AssertionError();
    }

    @Invoker("clearCollisions")
    static double[] pmweatherIv$clearCollisions(long sceneHandle) {
        throw new AssertionError();
    }

    @Invoker("setMassProperties")
    static void pmweatherIv$setMassProperties(
        long sceneHandle,
        int id,
        double mass,
        double[] centerOfMass,
        double[] inertiaTensor
    ) {
        throw new AssertionError();
    }

    @Invoker("setLocalBounds")
    static void pmweatherIv$setLocalBounds(
        long sceneHandle,
        int id,
        int minX,
        int minY,
        int minZ,
        int maxX,
        int maxY,
        int maxZ
    ) {
        throw new AssertionError();
    }

    @Invoker("newVoxelCollider")
    static int pmweatherIv$newVoxelCollider(
        double frictionMultiplier,
        double volume,
        double restitution,
        boolean isFluid,
        BlockSubLevelCollisionCallback contactEvents
    ) {
        throw new AssertionError();
    }

    @Invoker("addVoxelColliderBox")
    static void pmweatherIv$addVoxelColliderBox(int index, double[] bounds) {
        throw new AssertionError();
    }

    @Invoker("createKinematicContraption")
    static void pmweatherIv$createMountedLevelCollider(
        long sceneHandle,
        int mountId,
        int id,
        double[] pose
    ) {
        throw new AssertionError();
    }

    @Invoker("removeKinematicContraption")
    static void pmweatherIv$removeMountedLevelCollider(long sceneHandle, int id) {
        throw new AssertionError();
    }

    @Invoker("setKinematicContraptionTransform")
    static void pmweatherIv$setMountedLevelColliderTransform(
        long sceneHandle,
        int id,
        double[] centerOfMass,
        double[] pose,
        double[] velocities
    ) {
        throw new AssertionError();
    }

    @Invoker("addKinematicContraptionChunkSection")
    static void pmweatherIv$addMountedLevelColliderChunkSection(
        long sceneHandle,
        int id,
        int x,
        int y,
        int z,
        int[] data
    ) {
        throw new AssertionError();
    }
}
