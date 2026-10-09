package com.g9third.pmweatheriv.mixin;

import net.minecraft.world.level.Level;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

@Mixin(targets = "mcinterface1211.WrapperWorld", remap = false)
public interface WrapperWorldAccessor {
    @Accessor(value = "world", remap = false)
    Level pmweatherIv$getLevel();
}
