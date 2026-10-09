package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.sable.OrientedHitboxRegistry;
import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.baseclasses.BoundingBoxHitResult;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.components.AEntityD_Definable;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

/**
 * Gives PMWeather-IV aircraft true oriented IV semantic hitboxes without
 * changing IV's JSON collision-group/gameplay ownership.
 */
@Mixin(value = BoundingBox.class, remap = false)
public abstract class BoundingBoxMixin {
    @Inject(method = "updateToEntity", at = @At("HEAD"), remap = false)
    private void pmweatherIv$captureUnroundedLocalCenter(
        AEntityD_Definable<?> entity,
        Point3D optionalOffset,
        CallbackInfo callbackInfo
    ) {
        OrientedHitboxRegistry.captureLocalUpdate((BoundingBox) (Object) this, optionalOffset, entity);
    }

    @Inject(method = "updateToEntity", at = @At("RETURN"), remap = false)
    private void pmweatherIv$trackOrientedOwner(
        AEntityD_Definable<?> entity,
        Point3D optionalOffset,
        CallbackInfo callbackInfo
    ) {
        OrientedHitboxRegistry.updateFromEntity((BoundingBox) (Object) this, entity);
    }

    @Inject(method = "isPointInside", at = @At("HEAD"), cancellable = true, remap = false)
    private void pmweatherIv$orientedPointTest(
        Point3D point,
        Point3D growthOffset,
        CallbackInfoReturnable<Boolean> callbackInfo
    ) {
        Boolean result = OrientedHitboxRegistry.isPointInside(
            (BoundingBox) (Object) this, point, growthOffset
        );
        if (result != null) {
            callbackInfo.setReturnValue(result);
        }
    }

    @Inject(method = "isPointInsideAndBelow", at = @At("HEAD"), cancellable = true, remap = false)
    private void pmweatherIv$orientedPointBelowTest(
        Point3D point,
        CallbackInfoReturnable<Boolean> callbackInfo
    ) {
        Boolean result = OrientedHitboxRegistry.isPointInsideAndBelow(
            (BoundingBox) (Object) this, point
        );
        if (result != null) {
            callbackInfo.setReturnValue(result);
        }
    }

    @Inject(method = "intersects", at = @At("HEAD"), cancellable = true, remap = false)
    private void pmweatherIv$orientedIntersectionTest(
        BoundingBox other,
        CallbackInfoReturnable<Boolean> callbackInfo
    ) {
        Boolean result = OrientedHitboxRegistry.intersects((BoundingBox) (Object) this, other);
        if (result != null) {
            callbackInfo.setReturnValue(result);
        }
    }

    @Inject(method = "getIntersection", at = @At("HEAD"), cancellable = true, remap = false)
    private void pmweatherIv$orientedRayTest(
        Point3D start,
        Point3D end,
        CallbackInfoReturnable<BoundingBoxHitResult> callbackInfo
    ) {
        BoundingBox box = (BoundingBox) (Object) this;
        if (OrientedHitboxRegistry.contains(box)) {
            callbackInfo.setReturnValue(OrientedHitboxRegistry.getIntersection(box, start, end));
        }
    }
}
