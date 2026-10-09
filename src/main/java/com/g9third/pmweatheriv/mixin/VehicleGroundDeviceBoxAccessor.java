package com.g9third.pmweatheriv.mixin;

import minecrafttransportsimulator.baseclasses.VehicleGroundDeviceBox;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

@Mixin(value = VehicleGroundDeviceBox.class, remap = false)
public interface VehicleGroundDeviceBoxAccessor {
    @Accessor("vehicle")
    EntityVehicleF_Physics pmweatherIv$getVehicle();
}
