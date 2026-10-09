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
    @Unique private double pmweatherIv$lastNativeModelY = Double.NaN;
    @Unique private double pmweatherIv$lastOutputModelX = Double.NaN;
    @Unique private double pmweatherIv$lastOutputModelY = Double.NaN;
    @Unique private double pmweatherIv$lastOutputModelZ = Double.NaN;
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
            pmweatherIv$lastOutputModelX = Double.NaN;
            pmweatherIv$lastOutputModelY = Double.NaN;
            pmweatherIv$lastOutputModelZ = Double.NaN;
            return;
        }

        Vec3d nativeModel = RoadSuspensionModel.nativeCenterLocal(device);
        if (nativeModel == null || !nativeModel.isFinite()) return;
        boolean previousOffsetStillPresent = Double.isFinite(pmweatherIv$lastOutputModelX)
            && nativeModel.subtract(new Vec3d(pmweatherIv$lastOutputModelX,
                pmweatherIv$lastOutputModelY, pmweatherIv$lastOutputModelZ)).lengthSquared() <= 4.0E-8;
        Vec3d nativeBase = previousOffsetStillPresent
            ? nativeModel.add(new Vec3d(0.0, -pmweatherIv$appliedSuspensionOffset, 0.0))
            : nativeModel;
        if (Double.isFinite(pmweatherIv$lastNativeModelY)
            && Math.abs(nativeBase.y() - pmweatherIv$lastNativeModelY) > 1.0E-4) {
            pmweatherIv$authoredVerticalMotion = true;
        }
        pmweatherIv$lastNativeModelY = nativeBase.y();
        if (RoadSuspensionModel.hasDeclaredVerticalStationMotion(device)) {
            pmweatherIv$authoredVerticalMotion = true;
        } else if (!Double.isFinite(pmweatherIv$lastOutputModelX)) {
            pmweatherIv$authoredVerticalMotion = false;
        }

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

        // Native update has just produced the fresh authored pose. If it did not
        // rewrite a station, remove only our last known displacement before
        // applying the new server-owned travel value.
        if (previousOffsetStillPresent && Math.abs(pmweatherIv$appliedSuspensionOffset) > 1.0E-9) {
            Vec3d oldWorldValue = RoadSuspensionModel.worldOffset(device, pmweatherIv$appliedSuspensionOffset);
            Vector3d oldWorld = new Vector3d(oldWorldValue.x(), oldWorldValue.y(), oldWorldValue.z());
            Vec3d oldParentValue = RoadSuspensionModel.parentOffsetFromWorld(device, oldWorldValue);
            Vector3d oldParent = new Vector3d(oldParentValue.x(), oldParentValue.y(), oldParentValue.z());
            device.localOffset.subtract(new minecrafttransportsimulator.baseclasses.Point3D(
                oldParent.x, oldParent.y, oldParent.z));
            device.position.subtract(new minecrafttransportsimulator.baseclasses.Point3D(
                oldWorld.x, oldWorld.y, oldWorld.z));
            device.boundingBox.globalCenter.subtract(new minecrafttransportsimulator.baseclasses.Point3D(
                oldWorld.x, oldWorld.y, oldWorld.z));
        }

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
        Vec3d output = nativeBase.add(RoadSuspensionModel.vehicleOffsetFromWorld(device, newWorldValue));
        pmweatherIv$lastOutputModelX = output.x();
        pmweatherIv$lastOutputModelY = output.y();
        pmweatherIv$lastOutputModelZ = output.z();
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
