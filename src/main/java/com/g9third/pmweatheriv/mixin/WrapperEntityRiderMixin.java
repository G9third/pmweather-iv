package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.physics.LandingGearSolver;
import mcinterface1211.BuilderEntityLinkedSeat;
import mcinterface1211.WrapperEntity;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.components.AEntityB_Existing;
import minecrafttransportsimulator.entities.instances.PartSeat;
import net.minecraft.world.entity.Entity;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.Final;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

/** Keeps IV's position-update cache consistent with the actual vanilla mount. */
@Mixin(value = WrapperEntity.class, remap = false)
public abstract class WrapperEntityRiderMixin {
    @Shadow @Final protected Entity entity;
    @Shadow private AEntityB_Existing cachedEntityRiding;

    @Inject(method = "getEntityRiding", at = @At("HEAD"), remap = false)
    private void pmweatherIv$refreshSeatQuery(CallbackInfoReturnable<AEntityB_Existing> callbackInfo) {
        pmweatherIv$refreshMountedSeatCache();
    }

    @Inject(method = "setPosition", at = @At("HEAD"), remap = false)
    private void pmweatherIv$restoreMountedPositionPath(Point3D position, boolean onGround,
        CallbackInfo callbackInfo) {
        // IV's fallback query can resolve a linked seat without caching it. Its
        // position writer checks the cache directly and otherwise uses teleportTo
        // beyond 0.25 blocks; that Minecraft method is a no-op on the client.
        pmweatherIv$refreshMountedSeatCache();
    }

    @Unique
    private void pmweatherIv$refreshMountedSeatCache() {
        Entity mount = entity.getVehicle();
        if (mount instanceof BuilderEntityLinkedSeat linkedSeat && !linkedSeat.isRemoved()
            && linkedSeat.hasPassenger(entity) && linkedSeat.entity instanceof PartSeat seat
            && seat.isValid && pmweatherIv$isManagedSeat(seat)
            && ((WrapperEntity) (Object) this).equals(seat.rider)) {
            // Recover after packet ordering or a detached-cache invalidation using
            // the exact current vanilla passenger and resolved IV rider identity.
            cachedEntityRiding = seat;
            return;
        }
        if (!(cachedEntityRiding instanceof PartSeat seat) || !pmweatherIv$isManagedSeat(seat)) return;
        if (!(mount instanceof BuilderEntityLinkedSeat linkedSeat) || linkedSeat.isRemoved()
            || !linkedSeat.hasPassenger(entity)
            || (linkedSeat.entity != null && linkedSeat.entity != cachedEntityRiding)) {
            cachedEntityRiding = null;
        }
    }

    @Unique
    private static boolean pmweatherIv$isManagedSeat(PartSeat seat) {
        return seat.vehicleOn != null
            && LandingGearSolver.shouldReplaceIvGroundOperations(seat.vehicleOn);
    }
}
