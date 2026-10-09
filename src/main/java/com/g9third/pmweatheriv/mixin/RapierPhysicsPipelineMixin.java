package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.compat.StandaloneTrueImpactCompat;
import com.g9third.pmweatheriv.sable.SableCollisionCapture;
import net.minecraft.server.level.ServerLevel;
import org.spongepowered.asm.mixin.Final;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Redirect;

/**
 * Replay point for PMWeather-IV's substep-resolved Sable contact-force tap.
 *
 * <p>PMIV uses priority 1100 so it deterministically owns the Sable 2.x
 * {@code clearCollisions(long)} drain and can reconstruct the complete collision stream
 * across PMIV-managed substeps. A separately installed True Impact uses an optional
 * lower/default-priority redirect; PMIV forwards the complete raw collision batch into
 * True Impact's existing SableImpactCapture so ordinary Sable sublevels retain True
 * Impact behavior without competing for the native drain. Records involving PMIV aircraft
 * body IDs are removed from that third-party batch to prevent unknown-body/world
 * misclassification.</p>
 */
@Mixin(
    targets = "dev.ryanhcode.sable.physics.impl.rapier.RapierPhysicsPipeline",
    priority = 1100,
    remap = false
)
public abstract class RapierPhysicsPipelineMixin {
    @Shadow @Final private ServerLevel level;

    @Redirect(
        method = "processCollisionEffects",
        at = @At(
            value = "INVOKE",
            target = "Ldev/ryanhcode/sable/physics/impl/rapier/Rapier3D;clearCollisions(J)[D"
        )
    )
    private double[] pmweatherIv$captureCollisionForces(long sceneHandle) {
        double[] bufferedForSable = SableCollisionCapture.finishTickForSable(sceneHandle);
        if (bufferedForSable != null) {
            double[] completeTick = SableCollisionCapture.consumeCompletedExternalReplay(
                sceneHandle
            );
            double[] thirdPartyBatch = SableCollisionCapture.withoutPmivVehicleContacts(
                sceneHandle, completeTick != null ? completeTick : bufferedForSable
            );
            StandaloneTrueImpactCompat.processSableCollisionBatch(level, thirdPartyBatch);
            return bufferedForSable;
        }

        // No PMIV aircraft substep tap was active. Preserve Sable's ordinary
        // collision-effects input and, when standalone True Impact is installed,
        // forward the exact same raw records to its normal ServerSubLevel resolver.
        double[] collisions = Rapier3DInvoker.pmweatherIv$clearCollisions(sceneHandle);
        SableCollisionCapture.captureWholeTick(sceneHandle, collisions);
        StandaloneTrueImpactCompat.processSableCollisionBatch(
            level,
            SableCollisionCapture.withoutPmivVehicleContacts(sceneHandle, collisions)
        );
        return collisions;
    }
}
