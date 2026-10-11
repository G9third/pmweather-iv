package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.physics.LandingGearSolver;
import com.g9third.pmweatheriv.physics.GroundDeviceFeedback;
import minecrafttransportsimulator.baseclasses.ComputedVariable;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

/** Keeps IV's transient wheel-strike feedback from surviving a lost ground contact. */
@Mixin(value = PartGroundDevice.class, remap = false)
public abstract class PartGroundDeviceContactMixin {
    @Inject(method = "update", at = @At("HEAD"), remap = false)
    private void pmweatherIv$resetWheelStrikePulse(CallbackInfo callbackInfo) {
        PartGroundDevice device = (PartGroundDevice) (Object) this;
        if (device.vehicleOn != null
            && LandingGearSolver.shouldReplaceIvGroundOperations(device.vehicleOn)) {
            // IV clears this only in its grounded branch. Its forceSound wheel
            // strike repeats every tick if a true pulse survives the airborne branch.
            device.contactThisTick = false;
        }
    }

    @Inject(method = "createComputedVariable", at = @At("RETURN"), cancellable = true, remap = false)
    private void pmweatherIv$groundRelativeFeedback(String variable, boolean createDefaultIfNotPresent,
        CallbackInfoReturnable<ComputedVariable> callbackInfo) {
        if (!variable.equals("speed") && !variable.equals("speed_scaled")
            && !variable.equals("slip") && !variable.equals("slip_degrees")) return;
        PartGroundDevice device = (PartGroundDevice) (Object) this;
        ComputedVariable nativeVariable = callbackInfo.getReturnValue();
        // Authored part-local variable modifiers retain their own meaning.
        if (device.vehicleOn == null || nativeVariable == null || nativeVariable.entity == device) return;
        callbackInfo.setReturnValue(new ComputedVariable(device, variable, partialTicks -> {
            if (!LandingGearSolver.shouldReplaceIvGroundOperations(device.vehicleOn))
                return nativeVariable.computeValue(partialTicks);
            GroundDeviceFeedback.Motion motion = GroundDeviceFeedback.motion(device);
            return switch (variable) {
                case "speed" -> motion.speedMps();
                case "speed_scaled" -> motion.speedMps() / Math.max(1e-6, Math.abs(device.vehicleOn.speedFactor));
                case "slip" -> motion.slip();
                default -> motion.slipDegrees();
            };
        }, false));
    }
}
