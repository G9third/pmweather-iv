package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.api.PMIVSoundVariableApi;
import minecrafttransportsimulator.baseclasses.ComputedVariable;
import minecrafttransportsimulator.entities.components.AEntityD_Definable;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

/** Makes PMIV air-warning telemetry available through IV's normal variable system. */
@Mixin(value = AEntityD_Definable.class, remap = false)
public abstract class DefinableAirWarningVariableMixin {
    @Inject(method = "createComputedVariable", at = @At("HEAD"), cancellable = true, remap = false)
    private void pmweatherIv$createAirWarningVariable(
        String variable,
        boolean createDefault,
        CallbackInfoReturnable<ComputedVariable> callbackInfo
    ) {
        ComputedVariable computed = PMIVSoundVariableApi.createComputedVariable(
            (AEntityD_Definable<?>) (Object) this, variable
        );
        if (computed != null) {
            callbackInfo.setReturnValue(computed);
        }
    }
}
