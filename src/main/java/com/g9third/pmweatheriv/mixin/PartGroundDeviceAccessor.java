package com.g9third.pmweatheriv.mixin;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.blocks.components.ABlockBase.BlockMaterial;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

@Mixin(value = PartGroundDevice.class, remap = false)
public interface PartGroundDeviceAccessor {
    @Accessor("groundPosition") Point3D pmweatherIv$getGroundPosition();
    @Accessor("blockMaterialBelow") BlockMaterial pmweatherIv$getBlockMaterialBelow();
}
