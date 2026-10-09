package com.g9third.pmweatheriv.mixin;

import minecrafttransportsimulator.baseclasses.VehicleGroundDeviceBox;
import minecrafttransportsimulator.baseclasses.VehicleGroundDeviceCollection;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

/**
 * Exposes IV's four consolidated ground-device boxes so runway constraints are
 * created only for groups that IV says are actually grounded. The public
 * getContactPoint method returns authored/contact geometry even while airborne,
 * so it cannot by itself determine whether a constraint exists.
 */
@Mixin(value = VehicleGroundDeviceCollection.class, remap = false)
public interface VehicleGroundDeviceCollectionAccessor {
    @Accessor("vehicle")
    EntityVehicleF_Physics pmweatherIv$getVehicle();

    @Accessor("frontLeftGDB")
    VehicleGroundDeviceBox pmweatherIv$getFrontLeftGDB();

    @Accessor("frontRightGDB")
    VehicleGroundDeviceBox pmweatherIv$getFrontRightGDB();

    @Accessor("rearLeftGDB")
    VehicleGroundDeviceBox pmweatherIv$getRearLeftGDB();

    @Accessor("rearRightGDB")
    VehicleGroundDeviceBox pmweatherIv$getRearRightGDB();
}
