package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.physics.LandingGearSolver;
import com.g9third.pmweatheriv.physics.GroundVehicleWind;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.VehicleGroundDeviceCollection;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

/**
 * Keeps IV's ground-device collision scan as a sensor/animation source while
 * removing its managed vehicle pose solver.
 *
 * <p>IV still updates the four ground boxes and the groundedGroundDevices set,
 * which keeps normal wheel/tread animations and content-pack variables alive.
 * It no longer contributes vertical groundMotion depenetration or direct
 * pitch/roll Euler corrections for Sable-managed vehicles. PMWeather-IV reads
 * the completed generic scan as sensor data and applies one normal/lateral/
 * commanded-brake point constraint to the parent Sable body.</p>
 */
@Mixin(value = VehicleGroundDeviceCollection.class, remap = false)
public abstract class VehicleGroundDeviceCollectionMixin {
    @Inject(method = "performPitchCorrection", at = @At("HEAD"), cancellable = true, remap = false)
    private void pmweatherIv$replacePitchGroundCorrection(
        Point3D groundMotion,
        CallbackInfo callbackInfo
    ) {
        var vehicle = ((VehicleGroundDeviceCollectionAccessor) (Object) this)
            .pmweatherIv$getVehicle();
        if (LandingGearSolver.shouldReplaceIvGroundAngularCorrection(vehicle)) {
            callbackInfo.cancel();
        }
    }

    @Inject(method = "performRollCorrection", at = @At("HEAD"), cancellable = true, remap = false)
    private void pmweatherIv$replaceRollGroundCorrection(
        Point3D groundMotion,
        CallbackInfo callbackInfo
    ) {
        var vehicle = ((VehicleGroundDeviceCollectionAccessor) (Object) this)
            .pmweatherIv$getVehicle();
        if (LandingGearSolver.shouldReplaceIvGroundAngularCorrection(vehicle)) {
            callbackInfo.cancel();
        }
    }

    @Inject(method = "getMaxCollisionDepth", at = @At("HEAD"), cancellable = true, remap = false)
    private void pmweatherIv$replaceGroundDepenetration(
        CallbackInfoReturnable<Double> callbackInfo
    ) {
        var vehicle = ((VehicleGroundDeviceCollectionAccessor) (Object) this)
            .pmweatherIv$getVehicle();
        if (LandingGearSolver.shouldReplaceIvGroundOperations(vehicle)
            || GroundVehicleWind.replacesNativeTires(vehicle)) {
            // CollisionDepth remains stored on each VehicleGroundDeviceBox for
            // IV sensor/animation/gameplay state and diagnostics only. Rapier
            // owns physical penetration/support; returning zero prevents IV's
            // moveVehicle() stage from translating the mirrored entity again.
            callbackInfo.setReturnValue(0.0);
        }
    }
}
