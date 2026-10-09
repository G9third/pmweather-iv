package com.g9third.pmweatheriv.mixin;

import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

@Mixin(value = EntityVehicleF_Physics.class, remap = false)
public interface EntityVehiclePhysicsAccessor {
    @Accessor("dragForce")
    double pmweatherIv$getDragForce();

    @Accessor("normalizedVelocityVector")
    minecrafttransportsimulator.baseclasses.Point3D pmweatherIv$getNormalizedVelocityVector();

    @Accessor("hasRotors")
    void pmweatherIv$setHasRotors(boolean value);

    @Accessor("trackAngle")
    void pmweatherIv$setTrackAngle(double value);

    @Accessor("thrustForceValue")
    double pmweatherIv$getThrustForceValue();

    @Accessor("thrustForceValue")
    void pmweatherIv$setThrustForceValue(double value);
}
