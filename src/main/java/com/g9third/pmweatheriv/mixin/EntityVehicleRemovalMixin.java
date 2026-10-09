package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.sable.SableVehicleManager;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

/** Clears deferred PMIV bookkeeping when IV permanently removes a vehicle. */
@Mixin(targets = "minecrafttransportsimulator.entities.components.AEntityA_Base", remap = false)
public abstract class EntityVehicleRemovalMixin {
    @Inject(method = "remove", at = @At("TAIL"), remap = false)
    private void pmweatherIv$clearDeferredVehicleLifecycle(CallbackInfo callbackInfo) {
        if ((Object) this instanceof EntityVehicleF_Physics vehicle
            && vehicle.definition != null
            && vehicle.definition.motorized != null) {
            SableVehicleManager.onVehicleEntityRemoved(vehicle);
        }
    }
}
