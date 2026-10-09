package com.g9third.pmweatheriv.mixin;

import minecrafttransportsimulator.entities.components.AEntityD_Definable;
import minecrafttransportsimulator.entities.instances.PartEngine;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Redirect;

/**
 * Keeps IV's normal one-shot shutdown/crash audio, but stops continuous engine loops after a
 * PMIV aircraft has become an out-of-health persistent Sable wreck.
 *
 * <p>IV's {@code AEntityD_Definable.updateSounds} already gates only looping sounds on
 * {@code isValid}; non-looping sounds take a separate edge-triggered path. PMIV deliberately
 * leaves a catastrophically damaged IV vehicle entity valid so its Sable wreck can finish the
 * physical crash. Without this redirect, engine parts can therefore keep content-pack loops
 * (for example cooling, starter/cranking, or running loops) alive on a wreck that stock IV
 * would normally remove. We make only that looping-sound validity check see an out-of-health
 * engine's parent vehicle as invalid. The actual IV entity, engine state, temperature, and
 * Sable wreck remain untouched.
 */
@Mixin(value = AEntityD_Definable.class, remap = false)
public abstract class DefinablePersistentWreckSoundMixin {
    @Redirect(
        method = "updateSounds",
        at = @At(
            value = "FIELD",
            target = "Lminecrafttransportsimulator/entities/components/AEntityD_Definable;isValid:Z"
        ),
        remap = false
    )
    private boolean pmweatherIv$stopLoopingEngineSoundsOnPersistentWreck(
        AEntityD_Definable<?> definable
    ) {
        if (!definable.isValid) {
            return false;
        }
        if (definable instanceof PartEngine engine
            && engine.vehicleOn != null
            && engine.vehicleOn.isValid
            && engine.vehicleOn.outOfHealth) {
            return false;
        }
        return true;
    }
}
