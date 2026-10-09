package com.g9third.pmweatheriv.mixin;

import minecrafttransportsimulator.jsondefs.JSONParticle;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

@Mixin(targets = "minecrafttransportsimulator.entities.instances.EntityParticle", remap = false)
public interface EntityParticleWindAccessor {
    @Accessor(value = "definition", remap = false)
    JSONParticle pmweatherIv$getDefinition();

    @Accessor(value = "touchingBlocks", remap = false)
    boolean pmweatherIv$isTouchingBlocks();
}
