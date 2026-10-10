package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import com.g9third.pmweatheriv.physics.AirframeLoads;
import com.g9third.pmweatheriv.physics.AirWarningSystem;
import com.g9third.pmweatheriv.physics.AircraftState;
import com.g9third.pmweatheriv.physics.AircraftStateAccess;
import com.g9third.pmweatheriv.physics.AutoTrimOffset;
import com.g9third.pmweatheriv.physics.AutoTrimPreferences;
import com.g9third.pmweatheriv.physics.GroundVehicleWind;
import com.g9third.pmweatheriv.physics.LandingGearSolver;
import com.g9third.pmweatheriv.physics.GroundVehicleWindStateAccess;
import com.g9third.pmweatheriv.sable.SableVehicleManager;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.items.instances.ItemVehicle;
import minecrafttransportsimulator.mcinterface.AWrapperWorld;
import minecrafttransportsimulator.mcinterface.IWrapperNBT;
import minecrafttransportsimulator.mcinterface.IWrapperPlayer;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.Level;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

/**
 * Sable owns eligible vehicle pose, momentum and collision. Aircraft and road
 * load adapters retain their respective actuators and aerodynamic models.
 * Clients consume the same authoritative IV movement stream for both kinds.
 */
@Mixin(value = EntityVehicleF_Physics.class, remap = false)
public abstract class EntityVehiclePhysicsMixin implements GroundVehicleWindStateAccess, AircraftStateAccess {
    @Unique private final AircraftState pmweatherIv$flightState = new AircraftState();
    @Unique private final GroundVehicleWind.State pmweatherIv$groundWindState = new GroundVehicleWind.State();

    @Override
    public AircraftState pmweatherIv$getAircraftState() {
        return pmweatherIv$flightState;
    }

    @Override
    public GroundVehicleWind.State pmweatherIv$getGroundWindState() {
        return pmweatherIv$groundWindState;
    }

    @Inject(method = "<init>", at = @At("RETURN"), remap = false)
    private void pmweatherIv$attachAutoTrimPreferences(AWrapperWorld world, IWrapperPlayer placingPlayer,
            ItemVehicle item, IWrapperNBT savedData, CallbackInfo callbackInfo) {
        AutoTrimPreferences.attach((EntityVehicleF_Physics) (Object) this, savedData);
        AutoTrimOffset.attach((EntityVehicleF_Physics) (Object) this, savedData);
    }

    @Inject(method = "getForcesAndMotions", at = @At("HEAD"), cancellable = true, remap = false)
    private void pmweatherIv$replaceManagedVehicleModel(CallbackInfo callbackInfo) {
        EntityVehicleF_Physics vehicle = (EntityVehicleF_Physics) (Object) this;
        PMWeatherIVConfig.Values config = PMWeatherIVConfig.get();

        if ((!vehicle.world.isClient() ? !config.enabled()
                : !com.g9third.pmweatheriv.network.AircraftStateNetwork.managed(vehicle))
            || !SableVehicleManager.supportsVehicle(vehicle)) {
            if (!vehicle.world.isClient()) {
                SableVehicleManager.removeBody(vehicle);
                if (vehicle.definition != null && vehicle.definition.motorized != null
                    && !vehicle.definition.motorized.isBlimp && vehicle.world instanceof WrapperWorldAccessor accessor
                    && accessor.pmweatherIv$getLevel() instanceof ServerLevel server) {
                    com.g9third.pmweatheriv.network.AircraftStateNetwork.publish(
                        vehicle, server, pmweatherIv$flightState, null, false);
                }
            }
            pmweatherIv$flightState.reset();
            return;
        }

        if (!(vehicle.world instanceof WrapperWorldAccessor worldAccessor)) {
            pmweatherIv$flightState.reset();
            return;
        }
        Level level = worldAccessor.pmweatherIv$getLevel();
        if (vehicle.world.isClient()) {
            com.g9third.pmweatheriv.network.AircraftStateNetwork.apply(vehicle);
            // The existing IV movement-delta reconciliation owns client presentation.
            // Do not run a second aerodynamic model or advance a local rigid body.
            callbackInfo.cancel();
            return;
        }
        if (!(level instanceof ServerLevel serverLevel)) {
            pmweatherIv$flightState.reset();
            return;
        }

        try {
            long solverStartedNanos = PMIVObserver.needsDetailedSamples(vehicle.uniqueUUID)
                ? System.nanoTime() : 0L;
            AirframeLoads.SolveResult result = SableVehicleManager.calculateAndQueueLoads(
                vehicle, serverLevel, config, pmweatherIv$flightState
            );
            if (vehicle.definition.motorized.isAircraft) AirWarningSystem.update(
                vehicle, serverLevel, config, pmweatherIv$flightState, result
            );
            long solverNanos = solverStartedNanos == 0L ? 0L
                : Math.max(0L, System.nanoTime() - solverStartedNanos);
            com.g9third.pmweatheriv.network.AircraftStateNetwork.publish(
                vehicle, serverLevel, pmweatherIv$flightState, result, true);
            PMIVObserver.captureFlightTick(
                vehicle, serverLevel, result, pmweatherIv$flightState, solverNanos
            );
            // Missing Sable residency holds motion; native IV flight remains
            // bypassed while the persistent body is sleeping or retrying.
            callbackInfo.cancel();
        } catch (RuntimeException exception) {
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "ERROR stage=persistentSableVehicleSolver uuid=" + vehicle.uniqueUUID
                    + " type=" + exception.getClass().getName()
                    + " message=" + String.valueOf(exception.getMessage()).replace('\n', ' ')
            );
            }
            boolean bodyPreserved = SableVehicleManager.hasSableCollisionAuthority(vehicle);
            if (!bodyPreserved) {
                // Preserve momentum and hold motion until the next owner tick
                // can restore Sable residency.
                SableVehicleManager.holdMotionRequest(vehicle, pmweatherIv$flightState);
            }
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "SABLE_SOLVER_FAILURE_HELD uuid=" + vehicle.uniqueUUID
                    + " bodyPreserved=" + bodyPreserved
                    + " fallback=NO_NATIVE_IV_PHYSICS_TICK"
            );
            }
            callbackInfo.cancel();
        }
    }


    @Inject(method = "update", at = @At("RETURN"), remap = false)
    private void pmweatherIv$restoreRoadPhysicalVelocity(CallbackInfo callbackInfo) {
        EntityVehicleF_Physics vehicle = (EntityVehicleF_Physics) (Object) this;
        if (!vehicle.world.isClient() && !vehicle.definition.motorized.isAircraft
            && LandingGearSolver.shouldReplaceIvGroundOperations(vehicle) && pmweatherIv$flightState.hasPhysicalVelocity) {
            var velocity = pmweatherIv$flightState.originVelocityWorld;
            double scale = Math.max(1e-6, Math.abs(vehicle.speedFactor)*20);
            vehicle.motion.set(velocity.x()/scale, velocity.y()/scale, velocity.z()/scale);
            vehicle.velocity = vehicle.motion.length();
            LandingGearSolver.maintainIvGroundAnimationState(vehicle);
        }
    }

    /** Wind-only compatibility path for IV modes outside persistent body management. */
    @Inject(method = "getForcesAndMotions", at = @At("RETURN"), remap = false)
    private void pmweatherIv$addGroundVehicleWind(CallbackInfo callbackInfo) {
        EntityVehicleF_Physics vehicle = (EntityVehicleF_Physics) (Object) this;
        PMWeatherIVConfig.Values config = PMWeatherIVConfig.get();
        if (LandingGearSolver.shouldReplaceIvGroundOperations(vehicle)) return;
        if (vehicle.definition != null && vehicle.definition.motorized != null) {
            GroundVehicleWind.correctNativeGroundGravity(vehicle);
        }
        if (!config.enabled()
            || vehicle.world.isClient()
            || vehicle.definition == null
            || vehicle.definition.motorized == null
            || vehicle.definition.motorized.isAircraft
            || vehicle.lockedOnRoad
            || !(vehicle.world instanceof WrapperWorldAccessor accessor)
            || !(accessor.pmweatherIv$getLevel() instanceof ServerLevel serverLevel)) {
            if (!vehicle.world.isClient()) {
                GroundVehicleWind.resetDynamics(pmweatherIv$groundWindState);
            }
            return;
        }
        try {
            PMIVObserver.beforeWind(vehicle, serverLevel);
            GroundVehicleWind.Result groundWind = GroundVehicleWind.apply(
                vehicle, serverLevel, config, pmweatherIv$groundWindState
            );
            PMIVObserver.afterWind(vehicle, serverLevel, groundWind);
        } catch (RuntimeException exception) {
            // Unsupported native modes retain their existing wind-only fallback.
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "ERROR stage=groundVehicleWind uuid=" + vehicle.uniqueUUID
                    + " type=" + exception.getClass().getName()
                    + " message=" + String.valueOf(exception.getMessage()).replace('\n', ' ')
                    + " fallback=IV_NATIVE_UNMODIFIED"
            );
            }
        }
    }
}
