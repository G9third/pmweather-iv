package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.physics.LandingGearSolver;
import com.g9third.pmweatheriv.physics.GroundDeviceFeedback;
import com.g9third.pmweatheriv.physics.RoadSuspensionModel;
import com.g9third.pmweatheriv.physics.RoadSuspensionPartAccess;
import com.g9third.pmweatheriv.mixin.WrapperWorldAccessor;
import com.g9third.pmweatheriv.network.RoadSuspensionNetwork;
import com.g9third.pmweatheriv.sable.SableVehicleManager;
import com.g9third.pmweatheriv.physics.Vec3d;
import minecrafttransportsimulator.baseclasses.ComputedVariable;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import net.minecraft.world.level.Level;
import org.joml.Vector3d;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

/** Keeps IV's transient wheel-strike feedback from surviving a lost ground contact. */
@Mixin(value = PartGroundDevice.class, remap = false)
public abstract class PartGroundDeviceContactMixin implements RoadSuspensionPartAccess {
    @Unique private double pmweatherIv$appliedSuspensionOffset;
    @Unique private boolean pmweatherIv$authoredVerticalMotion = true;

    @Override
    public double pmweatherIv$appliedSuspensionOffset() {
        return pmweatherIv$appliedSuspensionOffset;
    }

    @Override
    public boolean pmweatherIv$hasAuthoredVerticalMotion() {
        return pmweatherIv$authoredVerticalMotion;
    }

    @Inject(method = "update", at = @At("RETURN"), remap = false)
    private void pmweatherIv$applyRoadSuspensionTravel(CallbackInfo callbackInfo) {
        PartGroundDevice device = (PartGroundDevice) (Object) this;
        if (device.vehicleOn == null || device.isSpare || !device.isValid
            || device.vehicleOn.uniqueUUID == null) {
            pmweatherIv$appliedSuspensionOffset = 0.0;
            pmweatherIv$authoredVerticalMotion = true;
            return;
        }
        if (!LandingGearSolver.shouldReplaceIvGroundOperations(device.vehicleOn)
            || device.vehicleOn.definition == null || device.vehicleOn.definition.motorized == null
            || device.vehicleOn.definition.motorized.isAircraft || device.definition == null
            || device.definition.ground == null
            || (!device.definition.ground.isWheel && !device.definition.ground.isTread)) {
            pmweatherIv$appliedSuspensionOffset = 0.0;
            pmweatherIv$authoredVerticalMotion = true;
            return;
        }

        Vec3d nativeModel = RoadSuspensionModel.nativeCenterLocal(device);
        if (nativeModel == null || !nativeModel.isFinite()) return;
        // APart.update rebuilds the native pose from authored placement data
        // before this injection. Do not infer authored motion from pose deltas:
        // float noise and our previous fallback travel can change Y.
        pmweatherIv$authoredVerticalMotion = RoadSuspensionModel.hasDeclaredVerticalStationMotion(device);

        double travel = 0.0;
        if (RoadSuspensionModel.supportsFallback(device.vehicleOn, device)) {
            if (device.world != null && device.world.isClient()) {
                if (device.world instanceof WrapperWorldAccessor accessor) {
                    Level level = accessor.pmweatherIv$getLevel();
                    PartGroundDevice master = RoadSuspensionModel.masterPart(device);
                    travel = RoadSuspensionNetwork.clientTravel(level, device.vehicleOn.uniqueUUID,
                        master == null ? null : master.uniqueUUID);
                }
            } else {
                travel = SableVehicleManager.roadSuspensionTravel(device);
            }
        }
        if (!Double.isFinite(travel)) travel = 0.0;

        Vec3d newWorldValue = RoadSuspensionModel.worldOffset(device, travel);
        Vector3d newWorld = new Vector3d(newWorldValue.x(), newWorldValue.y(), newWorldValue.z());
        Vec3d newParentValue = RoadSuspensionModel.parentOffsetFromWorld(device, newWorldValue);
        Vector3d newParent = new Vector3d(newParentValue.x(), newParentValue.y(), newParentValue.z());
        device.localOffset.add(new minecrafttransportsimulator.baseclasses.Point3D(
            newParent.x, newParent.y, newParent.z));
        device.position.add(new minecrafttransportsimulator.baseclasses.Point3D(
            newWorld.x, newWorld.y, newWorld.z));
        device.boundingBox.globalCenter.add(new minecrafttransportsimulator.baseclasses.Point3D(
            newWorld.x, newWorld.y, newWorld.z));
        if (device.vehicleOn.groundDeviceCollective != null) {
            device.vehicleOn.groundDeviceCollective.updateBounds();
        }
        pmweatherIv$appliedSuspensionOffset = travel;
    }

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
