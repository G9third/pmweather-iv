package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.sable.SableVehicleManager;
import java.util.Set;
import minecrafttransportsimulator.baseclasses.VehicleGroundDeviceBox;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

/**
 * Keeps IV ground-device collision scans as wheel/contact/animation sensors.
 *
 * <p>When the Sable compound body is active, physical vehicle-to-vehicle
 * collision is already solved in Rapier. IV's ground-device scan may still add
 * another IV vehicle to {@code collidedEntities}; remove only that movement
 * bookkeeping so the later EntityMoveAlong stage cannot alter Sable's pose.
 * Grounded/isCollided/contactDepth/tread state remains untouched.</p>
 */
@Mixin(value = VehicleGroundDeviceBox.class, remap = false)
public abstract class VehicleGroundDeviceBoxMixin {
    @Inject(method = "updateCollisionStatuses", at = @At("RETURN"), remap = false)
    private void pmweatherIv$keepGroundDeviceScanSensorOnly(
        Set<PartGroundDevice> groundedGroundDevices,
        boolean updateGroundDeviceTreadPosition,
        CallbackInfo callbackInfo
    ) {
        EntityVehicleF_Physics vehicle = ((VehicleGroundDeviceBoxAccessor) (Object) this)
            .pmweatherIv$getVehicle();
        if (vehicle != null && SableVehicleManager.hasSableCollisionAuthority(vehicle)) {
            vehicle.collidedEntities.clear();
        }
    }
}
