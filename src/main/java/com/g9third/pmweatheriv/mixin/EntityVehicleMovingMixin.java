package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.physics.LandingGearSolver;
import com.g9third.pmweatheriv.sable.SableVehicleManager;
import com.g9third.pmweatheriv.sable.OrientedEntityContact;
import com.g9third.pmweatheriv.sable.OrientedEntityRideSupport;
import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import com.g9third.pmweatheriv.physics.GroundVehicleWind;
import com.g9third.pmweatheriv.network.AircraftStateNetwork;
import com.g9third.pmweatheriv.network.CumulativePoseAnchor;
import com.g9third.pmweatheriv.physics.Vec3d;
import com.g9third.pmweatheriv.physics.GroundVehicleWindStateAccess;
import java.util.Collections;
import java.util.List;
import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.mcinterface.AWrapperWorld;
import minecrafttransportsimulator.mcinterface.IWrapperEntity;
import minecrafttransportsimulator.entities.components.AEntityE_Interactable;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import net.minecraft.server.level.ServerLevel;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.Redirect;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

/**
 * Transfers physical movement/collision authority from IV to the world-space
 * Sable compound body while retaining IV's vehicle update/network/render path.
 */
@Mixin(targets = "minecrafttransportsimulator.entities.instances.AEntityVehicleD_Moving", remap = false)
public abstract class EntityVehicleMovingMixin {
    @Shadow private AEntityE_Interactable<?> lastCollidedEntity;

    /** Client pose at the beginning of the IV movement pass for visual interpolation. */
    @Unique private final Point3D pmweatherIv$preReconcilePosition = new Point3D();
    @Unique private final RotationMatrix pmweatherIv$preReconcileOrientation = new RotationMatrix();
    @Unique private boolean pmweatherIv$hasPreReconcilePose;
    @Unique private boolean pmweatherIv$placementClearanceHandled;

    /** Server tires and client prediction use the same bounded brake/skid rule. */
    @Inject(method = "performGroundOperations", at = @At("HEAD"), cancellable = true, remap = false)
    private void pmweatherIv$prepareGroundTires(CallbackInfo callbackInfo) {
        if ((Object) this instanceof EntityVehicleF_Physics vehicle) {
            if (LandingGearSolver.shouldReplaceIvGroundOperations(vehicle)) {
                LandingGearSolver.maintainIvGroundAnimationState(vehicle);
                callbackInfo.cancel();
                return;
            }
            try {
                GroundVehicleWind.prepareTires(vehicle);
                if (GroundVehicleWind.replacesNativeTires(vehicle)) {
                    LandingGearSolver.maintainIvGroundAnimationState(vehicle);
                    pmweatherIv$captureGroundVehicleAfterGroundOperations(callbackInfo);
                    callbackInfo.cancel();
                }
            } catch (RuntimeException exception) {
                if (PMIVObserver.loggingEnabled()) {
                    PMIVObserver.log("ERROR stage=groundTireImpulses uuid=" + vehicle.uniqueUUID
                    + " type=" + exception.getClass().getName()
                    + " message=" + String.valueOf(exception.getMessage()).replace('\n', ' ')
                    + " fallback=IV_NATIVE_GROUND_OPERATIONS");
                }
            }
        }
    }

    /** The shared contact solver already spent this tick's available brake grip. */
    @Inject(method = "getBrakingForce", at = @At("HEAD"), cancellable = true, remap = false)
    private void pmweatherIv$skipDuplicateNativeBraking(CallbackInfoReturnable<Float> callbackInfo) {
        if ((Object) this instanceof EntityVehicleF_Physics vehicle
            && GroundVehicleWind.replacesNativeTires(vehicle)) callbackInfo.setReturnValue(0.0F);
    }

    /** Avoid consuming tire grip a second time through native angular skid projection. */
    @Inject(method = "getSkiddingForce", at = @At("HEAD"), cancellable = true, remap = false)
    private void pmweatherIv$skipDuplicateNativeSkidding(CallbackInfoReturnable<Float> callbackInfo) {
        if ((Object) this instanceof EntityVehicleF_Physics vehicle
            && GroundVehicleWind.replacesNativeTires(vehicle)) callbackInfo.setReturnValue(0.0F);
    }

    @Inject(method = "performGroundOperations", at = @At("HEAD"), cancellable = true, remap = false)
    private void pmweatherIv$replaceAircraftGroundOperations(CallbackInfo callbackInfo) {
        if (!((Object) this instanceof EntityVehicleF_Physics vehicle)
            || !LandingGearSolver.shouldReplaceIvGroundOperations(vehicle)) {
            return;
        }

        // PartGroundDevice uses groundVelocity/goingInReverse to animate wheel
        // rotation. Keep presentation inputs live while Sable owns tire forces.
        LandingGearSolver.maintainIvGroundAnimationState(vehicle);
        callbackInfo.cancel();
    }

    /**
     * Rapier has already resolved terrain/other-Sable-body collision. Returning
     * false here prevents IV's axis-aligned body pass from rejecting the same
     * translation/rotation a second time.
     */
    @Inject(method = "isCollisionBoxCollided", at = @At("HEAD"), cancellable = true, remap = false)
    private void pmweatherIv$bypassDuplicateIvPhysicalCollision(
        CallbackInfoReturnable<Boolean> callbackInfo
    ) {
        if ((Object) this instanceof EntityVehicleF_Physics vehicle
            && (SableVehicleManager.hasSableCollisionAuthority(vehicle)
                || vehicle.world.isClient() && LandingGearSolver.shouldReplaceIvGroundOperations(vehicle))) {
            callbackInfo.setReturnValue(false);
        }
    }

    /**
     * Clear IV's previous entity move-along latch when Rapier owns collisions.
     * Otherwise IV may add back another vehicle's motion once even after the
     * physical authority has moved to Sable.
     */
    @Inject(method = "moveVehicle", at = @At("HEAD"), remap = false)
    private void pmweatherIv$clearLegacyEntityMoveAlongState(CallbackInfo callbackInfo) {
        if ((Object) this instanceof EntityVehicleF_Physics vehicle
            && SableVehicleManager.hasSableCollisionAuthority(vehicle)) {
            lastCollidedEntity = null;
            vehicle.collidedEntities.clear();
        }
    }
    /**
     * Suppress IV's legacy AABB-top entity carry pass for compound-authority
     * aircraft. A corrected OBB support pass runs at method tail instead.
     */
    @Redirect(
        method = "doPostUpdateLogic",
        at = @At(
            value = "INVOKE",
            target = "Lminecrafttransportsimulator/mcinterface/AWrapperWorld;getEntitiesWithin(Lminecrafttransportsimulator/baseclasses/BoundingBox;)Ljava/util/List;"
        ),
        remap = false
    )
    private List<IWrapperEntity> pmweatherIv$skipLegacyAabbMoveAlong(
        AWrapperWorld world,
        BoundingBox queryBox
    ) {
        if ((Object) this instanceof EntityVehicleF_Physics vehicle
            && SableVehicleManager.hasSableCollisionAuthority(vehicle)) {
            return Collections.emptyList();
        }
        return world.getEntitiesWithin(queryBox);
    }

    @Inject(method = "doPostUpdateLogic", at = @At("TAIL"), remap = false)
    private void pmweatherIv$moveAlongOrientedEntitySurfaces(CallbackInfo callbackInfo) {
        if ((Object) this instanceof EntityVehicleF_Physics vehicle
            && SableVehicleManager.hasSableCollisionAuthority(vehicle)) {
            if (!pmweatherIv$placementClearanceHandled && !vehicle.world.isClient()) {
                pmweatherIv$placementClearanceHandled = OrientedEntityContact.clearPlacementOverlap(
                    vehicle, ((EntityVehicleMovingAccessor) vehicle).pmweatherIv$getPlacingPlayer()
                );
            }
            OrientedEntityContact.resolve(vehicle);
            OrientedEntityRideSupport.moveAlongEntities(vehicle);
        }
    }


    @Inject(method = "moveVehicle", at = @At("HEAD"), remap = false)
    private void pmweatherIv$captureClientPoseBeforeAuthoritativeReconcile(CallbackInfo callbackInfo) {
        if ((Object) this instanceof EntityVehicleF_Physics vehicle
            && vehicle.world.isClient()
            && LandingGearSolver.shouldReplaceIvGroundOperations(vehicle)) {
            pmweatherIv$preReconcilePosition.set(vehicle.position);
            pmweatherIv$preReconcileOrientation.set(vehicle.orientation);
            pmweatherIv$hasPreReconcilePose = true;
        } else {
            pmweatherIv$hasPreReconcilePose = false;
        }
    }

    /**
     * IV normally lets the client run its own motion prediction, then converges
     * toward cumulative server deltas with a deliberately soft nonlinear filter.
     * That is a poor match for a Sable-authoritative aircraft: during a sharp
     * turn the local analytic predictor can follow a different curved trajectory
     * for several metres before IV's filter catches up.
     *
     * <p>For PMIV aircraft we keep IV's existing movement packets, but consume the
     * remaining cumulative server-delta error completely at the end of the client
     * movement pass.  A separate absolute origin anchors the same
     * PacketVehicleServerMovement stream after initial load or reconnect.
     * Rendering interpolates from the pre-reconcile client pose to the corrected
     * authoritative pose, but PMIV deliberately does NOT overwrite IV's per-tick
     * motion with a difference of cumulative server deltas. Those deltas may arrive
     * every two client ticks; treating them as one-tick velocity caused the observed
     * 2x-motion/0x-motion presentation stutter.</p>
     */
    @Inject(method = "moveVehicle", at = @At("TAIL"), remap = false)
    private void pmweatherIv$reconcileClientToAuthoritativeServerDelta(CallbackInfo callbackInfo) {
        if (!((Object) this instanceof EntityVehicleF_Physics vehicle)
            || !vehicle.world.isClient()
            || !LandingGearSolver.shouldReplaceIvGroundOperations(vehicle)) {
            pmweatherIv$hasPreReconcilePose = false;
            return;
        }

        EntityVehicleMovingAccessor access = (EntityVehicleMovingAccessor) vehicle;
        Point3D serverM = access.pmweatherIv$getServerDeltaM();
        Point3D serverR = access.pmweatherIv$getServerDeltaR();
        Point3D clientM = access.pmweatherIv$getClientDeltaM();
        Point3D clientR = access.pmweatherIv$getClientDeltaR();
        if (!pmweatherIv$finite(serverM) || !pmweatherIv$finite(serverR)
            || !pmweatherIv$finite(clientM) || !pmweatherIv$finite(clientR)) {
            pmweatherIv$hasPreReconcilePose = false;
            return;
        }

        double residualX = serverM.x - clientM.x;
        double residualY = serverM.y - clientM.y;
        double residualZ = serverM.z - clientM.z;
        vehicle.position.add(residualX, residualY, residualZ);
        clientM.set(serverM);

        double residualPitch = serverR.x - clientR.x;
        double residualYaw = serverR.y - clientR.y;
        double residualRoll = serverR.z - clientR.z;
        if (Math.abs(residualPitch) + Math.abs(residualYaw) + Math.abs(residualRoll) > 1.0E-12) {
            vehicle.orientation.angles.add(residualPitch, residualYaw, residualRoll).clamp180();
            vehicle.orientation.updateToAngles();
        }
        clientR.set(serverR);

        CumulativePoseAnchor anchor = AircraftStateNetwork.poseAnchor(vehicle);
        if (anchor != null) {
            Vec3d absolutePosition = anchor.position(new Vec3d(serverM.x, serverM.y, serverM.z));
            Vec3d absoluteAngles = anchor.angles(new Vec3d(serverR.x, serverR.y, serverR.z));
            if (absolutePosition.isFinite() && absoluteAngles.isFinite()) {
                vehicle.position.set(absolutePosition.x(), absolutePosition.y(), absolutePosition.z());
                vehicle.orientation.angles.set(absoluteAngles.x(), absoluteAngles.y(), absoluteAngles.z()).clamp180();
                vehicle.orientation.updateToAngles();
            }
        }

        if (pmweatherIv$hasPreReconcilePose) {
            // Keep the actual client entity at the latest authoritative server pose,
            // while rendering this tick between the pose IV predicted at method HEAD
            // and the corrected pose. Do not derive velocity from packet cadence.
            vehicle.prevPosition.set(pmweatherIv$preReconcilePosition);
            vehicle.prevOrientation.set(pmweatherIv$preReconcileOrientation);
        }
        pmweatherIv$hasPreReconcilePose = false;
    }

    @Inject(method = "moveVehicle", at = @At("RETURN"), remap = false)
    private void pmweatherIv$publishAbsolutePoseAnchor(CallbackInfo callbackInfo) {
        if ((Object) this instanceof EntityVehicleF_Physics vehicle
            && !vehicle.world.isClient()
            && SableVehicleManager.hasSableCollisionAuthority(vehicle)
            && vehicle.world instanceof WrapperWorldAccessor accessor
            && accessor.pmweatherIv$getLevel() instanceof ServerLevel level) {
            AircraftStateNetwork.publishPoseAnchor(vehicle, level);
        }
    }

    @Unique
    private static boolean pmweatherIv$finite(Point3D value) {
        return value != null && Double.isFinite(value.x)
            && Double.isFinite(value.y) && Double.isFinite(value.z);
    }


    /** Capture IV's native tire/skid/weight-transfer result for ordinary ground vehicles. */
    @Inject(method = "performGroundOperations", at = @At("TAIL"), remap = false)
    private void pmweatherIv$captureGroundVehicleAfterGroundOperations(CallbackInfo callbackInfo) {
        if ((Object) this instanceof EntityVehicleF_Physics vehicle
            && vehicle.definition != null
            && vehicle.definition.motorized != null
            && !vehicle.definition.motorized.isAircraft
            && !LandingGearSolver.shouldReplaceIvGroundOperations(vehicle)
            && !vehicle.lockedOnRoad
            && !vehicle.world.isClient()
            && vehicle.world instanceof WrapperWorldAccessor accessor
            && accessor.pmweatherIv$getLevel() instanceof ServerLevel serverLevel) {
            if ((Object) vehicle instanceof GroundVehicleWindStateAccess windState) {
                GroundVehicleWind.afterGroundOperations(vehicle, windState.pmweatherIv$getGroundWindState());
            }
            PMIVObserver.afterGroundOperations(vehicle, serverLevel);
        }
    }

    @Inject(method = "moveVehicle", at = @At("HEAD"), remap = false)
    private void pmweatherIv$stageGroundCenterOfMassPivot(CallbackInfo callbackInfo) {
        if ((Object) this instanceof EntityVehicleF_Physics vehicle) GroundVehicleWind.beforeMove(vehicle);
    }

    @Redirect(method = "moveVehicle", at = @At(value = "INVOKE",
        target = "Lminecrafttransportsimulator/baseclasses/VehicleGroundDeviceCollection;isBlockedVertically()Z"), remap = false)
    private boolean pmweatherIv$skipDuplicateGroundUprightCorrection(
        minecrafttransportsimulator.baseclasses.VehicleGroundDeviceCollection collective
    ) {
        return ((Object) this instanceof EntityVehicleF_Physics vehicle
            && (LandingGearSolver.shouldReplaceIvGroundOperations(vehicle) || GroundVehicleWind.replacesNativeTires(vehicle)))
            || collective.isBlockedVertically();
    }

    /** Capture the movement IV ultimately accepted after ground and collision resolution. */
    @Inject(method = "moveVehicle", at = @At("RETURN"), remap = false)
    private void pmweatherIv$captureGroundVehicleAfterMove(CallbackInfo callbackInfo) {
        if ((Object) this instanceof EntityVehicleF_Physics vehicle) GroundVehicleWind.finishMove(vehicle);
        if ((Object) this instanceof EntityVehicleF_Physics vehicle
            && vehicle.definition != null
            && vehicle.definition.motorized != null
            && !vehicle.definition.motorized.isAircraft
            && !LandingGearSolver.shouldReplaceIvGroundOperations(vehicle)
            && !vehicle.lockedOnRoad
            && !vehicle.world.isClient()
            && vehicle.world instanceof WrapperWorldAccessor accessor
            && accessor.pmweatherIv$getLevel() instanceof ServerLevel serverLevel) {
            if ((Object) vehicle instanceof GroundVehicleWindStateAccess windState) {
                GroundVehicleWind.afterMove(vehicle, windState.pmweatherIv$getGroundWindState());
            }
            PMIVObserver.afterMove(vehicle, serverLevel);
        }
    }

}
