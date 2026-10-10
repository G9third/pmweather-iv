package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.physics.AutoTrimOffset;
import minecrafttransportsimulator.entities.components.AEntityD_Definable;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

/** Applies PMIV's trim offset around, and outside of, IV's unchanged native modifier pass. */
@Mixin(value = AEntityD_Definable.class, remap = false)
public abstract class DefinableAutoTrimOffsetMixin {
    @Inject(method = "updateVariableModifiers", at = @At("HEAD"), remap = false)
    private void pmweatherIv$removePreviousAutoTrimOffset(CallbackInfo callbackInfo) {
        if ((Object) this instanceof EntityVehicleF_Physics vehicle) {
            AutoTrimOffset.beforeModifierPass(vehicle);
        }
    }

    @Inject(method = "updateVariableModifiers", at = @At("RETURN"), remap = false)
    private void pmweatherIv$applyAutoTrimOffset(CallbackInfo callbackInfo) {
        if ((Object) this instanceof EntityVehicleF_Physics vehicle) {
            AutoTrimOffset.afterModifierPass(vehicle);
        }
    }
}
