package com.g9third.pmweatheriv.mixin;

import minecrafttransportsimulator.entities.instances.PartEngine;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

@Mixin(value = PartEngine.class, remap = false)
public interface PartEngineAccessor {
    @Accessor("jetPowerFactorVar")
    minecrafttransportsimulator.baseclasses.ComputedVariable pmweatherIv$getJetPowerFactorVar();
    /** Last native wheel/jet force; reading this never reruns engine calculations. */
    @Accessor("engineForceValue")
    double pmweatherIv$getEngineForceValue();

    @Accessor("engineAxialVelocity")
    double pmweatherIv$getEngineAxialVelocity();
}
