package com.g9third.pmweatheriv.devsupport;

import com.g9third.pmweatheriv.physics.AircraftState;
import com.g9third.pmweatheriv.physics.AirframeLoads;
import com.g9third.pmweatheriv.physics.GroundVehicleWind;
import com.g9third.pmweatheriv.physics.LandingGearSolver;
import com.g9third.pmweatheriv.sable.SableCollisionCapture;
import com.g9third.pmweatheriv.sable.SableCompoundCollider;
import com.g9third.pmweatheriv.sable.SableVehicleBody;
import java.util.List;
import java.util.ServiceConfigurationError;
import java.util.ServiceLoader;
import java.util.UUID;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import net.minecraft.server.level.ServerLevel;
import net.neoforged.bus.api.IEventBus;
import net.neoforged.fml.ModContainer;
import net.neoforged.neoforge.event.server.ServerStartedEvent;
import net.neoforged.neoforge.event.server.ServerStoppingEvent;
import org.joml.Quaterniondc;
import org.joml.Vector3dc;

/**
 * Production-safe boundary for optional logging and trace observation.
 *
 * <p>The public artifact has no provider and uses a no-op implementation. The
 * private development artifact registers its provider with ServiceLoader. Core
 * physics APIs are the only types crossing this boundary; trace record classes,
 * writers, commands and diagnostic network types stay out of main.</p>
 */
public final class PMIVObserver {
    private static final Observer NO_OP = new Observer() {};
    private static volatile Observer observer = NO_OP;
    private static volatile ClientProvider clientProvider;
    private static volatile boolean commonInitialized;
    private static volatile boolean clientInitialized;

    private PMIVObserver() {}

    /** Installs the optional common observer and lets it register dev-only common events. */
    public static synchronized void initializeCommon(IEventBus modBus, ModContainer modContainer) {
        if (commonInitialized) return;
        commonInitialized = true;
        Observer loaded = null;
        try {
            Provider provider = ServiceLoader.load(Provider.class, PMIVObserver.class.getClassLoader())
                .findFirst().orElse(null);
            if (provider != null) {
                loaded = provider.create();
                if (loaded != null) {
                    observer = loaded;
                    loaded.initializeCommon(modBus, modContainer);
                }
            }
        } catch (RuntimeException | ServiceConfigurationError | LinkageError unavailable) {
            // Optional dev tooling must never prevent the production physics mod loading.
            observer = NO_OP;
            String failure = "[PMWeather-IV] Optional development observer initialization failed: "
                + unavailable.getClass().getName() + ": " + String.valueOf(unavailable.getMessage());
            if (loaded != null) {
                try { loaded.log("ERROR " + failure); } catch (RuntimeException ignored) {}
            }
            System.err.println(failure);
        }
    }

    /** Loads client-only observer services only from the client mod entry point. */
    public static synchronized void initializeClient(IEventBus modBus, ModContainer modContainer) {
        if (clientInitialized) return;
        clientInitialized = true;
        try {
            ClientProvider provider = ServiceLoader.load(
                ClientProvider.class, PMIVObserver.class.getClassLoader()
            ).findFirst().orElse(null);
            if (provider != null) {
                clientProvider = provider;
                provider.initialize(modBus, modContainer);
            }
        } catch (RuntimeException | ServiceConfigurationError | LinkageError unavailable) {
            // Dev-only client tools are optional and have no gameplay authority.
            System.err.println("[PMWeather-IV] Optional development client observer initialization failed: "
                + unavailable.getClass().getName() + ": " + String.valueOf(unavailable.getMessage()));
        }
    }

    /** Optional private diagnostics hook for changes to the promoted wind monitor. */
    public static void windMonitorLiveChanged(boolean enabled) {
        ClientProvider provider = clientProvider;
        if (provider != null) provider.onWindMonitorLiveChanged(enabled);
    }

    public static void capturePerformance(UUID vehicleUuid, String stage, long elapsedNanos) {
        observer.capturePerformance(vehicleUuid, stage, elapsedNanos);
    }

    public static boolean loggingEnabled() {
        return observer.loggingEnabled();
    }

    public static void log(String line) {
        if (line != null) observer.log(line);
    }

    public static boolean isCapturing(UUID vehicleUuid) {
        return observer.isCapturing(vehicleUuid);
    }

    /** True only when an installed observer needs full per-surface debug/trace samples. */
    public static boolean needsDetailedSamples(UUID vehicleUuid) {
        return observer.needsDetailedSamples(vehicleUuid);
    }

    public static void captureFlightTick(EntityVehicleF_Physics vehicle, ServerLevel level,
            AirframeLoads.SolveResult result, AircraftState state, long solverNanos) {
        observer.captureFlightTick(vehicle, level, result, state, solverNanos);
    }

    public static void captureAppliedAerodynamics(SableVehicleBody body, double dt, long windTick,
            Vector3dc externalForce, Vector3dc torqueBody, Vector3dc nativeGravity,
            AirframeLoads.SubstepAerodynamicLoads loads) {
        observer.captureAppliedAerodynamics(body, dt, windTick, externalForce, torqueBody, nativeGravity, loads);
    }

    public static void captureLandingGear(SableVehicleBody body, Vector3dc positionWorld,
            Quaterniondc orientationWorld, Vector3dc linearVelocityWorld, Vector3dc angularVelocityBody,
            LandingGearSolver.LandingGearConstraintResult gear,
            SableCollisionCapture.SceneSnapshot collisionSnapshot) {
        observer.captureLandingGear(body, positionWorld, orientationWorld, linearVelocityWorld,
            angularVelocityBody, gear, collisionSnapshot);
    }

    public static void captureCollisions(SableVehicleBody body,
            SableCollisionCapture.SceneSnapshot snapshot,
            List<SableCompoundCollider.GameplayContact> contacts) {
        observer.captureCollisions(body, snapshot, contacts);
    }

    public static void captureCollisionGameplay(UUID vehicleUuid, long gameTime, int terrainContacts,
            int sweptTerrainImpacts, int uniqueTerrainBlocks, double maxContactForceN,
            double maxImpactImpulseNs, double maxPreImpactNormalSpeedMps,
            double maxPreImpactVehicleSpeedMps, double maxNativeIvCrashSpeed,
            int trueImpactImpactsSubmitted, double maxTrueImpactEnergyJ,
            double maxTrueImpactAbsorbedEnergyJ, int trueImpactImmediateBlocksBroken,
            int blocksBroken, int collisionBoxesDamaged, double crashDamage, int partsRemoved,
            boolean destroyed, boolean catastrophicWrecked, boolean outOfHealthBefore,
            boolean outOfHealthAfter, String crashEpisodeState, int crashEpisodeSettledTicks,
            boolean terrainDamageAllowed) {
        observer.captureCollisionGameplay(vehicleUuid, gameTime, terrainContacts, sweptTerrainImpacts,
            uniqueTerrainBlocks, maxContactForceN, maxImpactImpulseNs, maxPreImpactNormalSpeedMps,
            maxPreImpactVehicleSpeedMps, maxNativeIvCrashSpeed, trueImpactImpactsSubmitted,
            maxTrueImpactEnergyJ, maxTrueImpactAbsorbedEnergyJ, trueImpactImmediateBlocksBroken,
            blocksBroken, collisionBoxesDamaged, crashDamage, partsRemoved, destroyed,
            catastrophicWrecked, outOfHealthBefore, outOfHealthAfter, crashEpisodeState,
            crashEpisodeSettledTicks, terrainDamageAllowed);
    }

    public static void captureTerrainCcd(SableVehicleBody body, double safePathFraction,
            double preImpactVehicleSpeedMps, SableCompoundCollider.TerrainBoundaryImpact impact,
            Vector3dc normalWorld, double inwardPointSpeedBeforeMps, double normalImpulseNs,
            double tangentialImpulseNs, double frictionCoefficient,
            double tangentialComSpeedBeforeMps, double tangentialComSpeedAfterMps,
            double projectedContactAreaM2, boolean trueImpactPreResolved, String trueImpactMode,
            double trueImpactEnergyJ, double trueImpactAbsorbedEnergyJ, double trueImpactResidualEnergyJ,
            double trueImpactResidualInwardSpeedMps, int trueImpactBlocksBroken) {
        observer.captureTerrainCcd(body, safePathFraction, preImpactVehicleSpeedMps, impact, normalWorld,
            inwardPointSpeedBeforeMps, normalImpulseNs, tangentialImpulseNs, frictionCoefficient,
            tangentialComSpeedBeforeMps,
            tangentialComSpeedAfterMps, projectedContactAreaM2, trueImpactPreResolved, trueImpactMode,
            trueImpactEnergyJ, trueImpactAbsorbedEnergyJ, trueImpactResidualEnergyJ,
            trueImpactResidualInwardSpeedMps, trueImpactBlocksBroken);
    }

    public static void vehicleRemoved(UUID uuid, String reason) {
        observer.vehicleRemoved(uuid, reason);
    }

    public static void onServerStarted(ServerStartedEvent event) {
        observer.onServerStarted(event);
    }

    public static void onServerStopping(ServerStoppingEvent event) {
        observer.onServerStopping(event);
    }

    public static void beforeWind(EntityVehicleF_Physics vehicle, ServerLevel level) {
        observer.beforeWind(vehicle, level);
    }

    public static void afterWind(EntityVehicleF_Physics vehicle, ServerLevel level,
            GroundVehicleWind.Result result) {
        observer.afterWind(vehicle, level, result);
    }

    public static void afterGroundOperations(EntityVehicleF_Physics vehicle, ServerLevel level) {
        observer.afterGroundOperations(vehicle, level);
    }

    public static void afterMove(EntityVehicleF_Physics vehicle, ServerLevel level) {
        observer.afterMove(vehicle, level);
    }

    /** Service provider implemented only in the private development source set. */
    public interface Provider {
        Observer create();
    }

    /** Client service has a separate provider so dedicated servers never link dev client classes. */
    public interface ClientProvider {
        void initialize(IEventBus modBus, ModContainer modContainer);
        default void onWindMonitorLiveChanged(boolean enabled) {}
    }

    /** Typed hooks receive physical values already computed by the production core. */
    public interface Observer {
        default void capturePerformance(UUID vehicleUuid, String stage, long elapsedNanos) {}

        default void initializeCommon(IEventBus modBus, ModContainer modContainer) {}
        default boolean loggingEnabled() { return false; }
        default void log(String line) {}
        default boolean isCapturing(UUID vehicleUuid) { return false; }
        default boolean needsDetailedSamples(UUID vehicleUuid) { return false; }
        default void captureFlightTick(EntityVehicleF_Physics vehicle, ServerLevel level,
                AirframeLoads.SolveResult result, AircraftState state, long solverNanos) {}
        default void captureAppliedAerodynamics(SableVehicleBody body, double dt, long windTick,
                Vector3dc externalForce, Vector3dc torqueBody, Vector3dc nativeGravity,
                AirframeLoads.SubstepAerodynamicLoads loads) {}
        default void captureLandingGear(SableVehicleBody body, Vector3dc positionWorld,
                Quaterniondc orientationWorld, Vector3dc linearVelocityWorld, Vector3dc angularVelocityBody,
                LandingGearSolver.LandingGearConstraintResult gear,
                SableCollisionCapture.SceneSnapshot collisionSnapshot) {}
        default void captureCollisions(SableVehicleBody body,
                SableCollisionCapture.SceneSnapshot snapshot,
                List<SableCompoundCollider.GameplayContact> contacts) {}
        default void captureCollisionGameplay(UUID vehicleUuid, long gameTime, int terrainContacts,
                int sweptTerrainImpacts, int uniqueTerrainBlocks, double maxContactForceN,
                double maxImpactImpulseNs, double maxPreImpactNormalSpeedMps,
                double maxPreImpactVehicleSpeedMps, double maxNativeIvCrashSpeed,
                int trueImpactImpactsSubmitted, double maxTrueImpactEnergyJ,
                double maxTrueImpactAbsorbedEnergyJ, int trueImpactImmediateBlocksBroken,
                int blocksBroken, int collisionBoxesDamaged, double crashDamage, int partsRemoved,
                boolean destroyed, boolean catastrophicWrecked, boolean outOfHealthBefore,
                boolean outOfHealthAfter, String crashEpisodeState, int crashEpisodeSettledTicks,
                boolean terrainDamageAllowed) {}
        default void captureTerrainCcd(SableVehicleBody body, double safePathFraction,
                double preImpactVehicleSpeedMps, SableCompoundCollider.TerrainBoundaryImpact impact,
                Vector3dc normalWorld, double inwardPointSpeedBeforeMps, double normalImpulseNs,
            double tangentialImpulseNs, double frictionCoefficient,
                double tangentialComSpeedBeforeMps, double tangentialComSpeedAfterMps,
                double projectedContactAreaM2, boolean trueImpactPreResolved, String trueImpactMode,
                double trueImpactEnergyJ, double trueImpactAbsorbedEnergyJ, double trueImpactResidualEnergyJ,
                double trueImpactResidualInwardSpeedMps, int trueImpactBlocksBroken) {}
        default void vehicleRemoved(UUID uuid, String reason) {}
        default void onServerStarted(ServerStartedEvent event) {}
        default void onServerStopping(ServerStoppingEvent event) {}
        default void beforeWind(EntityVehicleF_Physics vehicle, ServerLevel level) {}
        default void afterWind(EntityVehicleF_Physics vehicle, ServerLevel level,
                GroundVehicleWind.Result result) {}
        default void afterGroundOperations(EntityVehicleF_Physics vehicle, ServerLevel level) {}
        default void afterMove(EntityVehicleF_Physics vehicle, ServerLevel level) {}
    }
}
