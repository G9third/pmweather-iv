package com.g9third.pmweatheriv.mixin;

import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import minecrafttransportsimulator.entities.instances.PartGroundDeviceFake;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

/** Resolves IV's distributed tread support sample back to its authored master part. */
@Mixin(value = PartGroundDeviceFake.class, remap = false)
public interface PartGroundDeviceFakeAccessor {
    @Accessor("masterPart")
    PartGroundDevice pmweatherIv$getMasterPart();
}
