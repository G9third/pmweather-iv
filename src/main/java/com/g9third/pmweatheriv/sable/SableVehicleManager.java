package com.g9third.pmweatheriv.sable;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import com.g9third.pmweatheriv.mixin.Rapier3DInvoker;
import com.g9third.pmweatheriv.physics.AircraftPhysics;
import com.g9third.pmweatheriv.physics.AirframeLoads;
import com.g9third.pmweatheriv.physics.AircraftState;
import com.g9third.pmweatheriv.physics.AutoTrimController;
import com.g9third.pmweatheriv.physics.AutoTrimRuntime;
import com.g9third.pmweatheriv.physics.Vec3d;
import com.g9third.pmweatheriv.network.RoadSuspensionNetwork;
import dev.ryanhcode.sable.neoforge.event.ForgeSablePostPhysicsTickEvent;
import dev.ryanhcode.sable.neoforge.event.ForgeSablePrePhysicsTickEvent;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import java.util.ArrayList;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.ConcurrentHashMap;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.server.level.ServerPlayer;
import net.neoforged.neoforge.event.level.LevelEvent;
import org.joml.Quaterniond;
import org.joml.Vector3d;

/**
 * Server bridge for vehicle loads and world-space Sable collision. Every eligible
 * road vehicle and aircraft receives one persistent Sable rigid body as soon as
 * the dimension physics system is available, including parked vehicles,
 * because physical collision authority must not disappear merely at zero speed.
 * Startup and retry windows preserve momentum and hold motion until a Sable body is available.
 */
public final class SableVehicleManager {
    private static final long INITIAL_RETRY_TICKS = 20L;
    private static final long MAX_RETRY_TICKS = 600L;
    private static final long HEALTHY_BODY_RESET_TICKS = 40L;
    private static final Map<UUID, SableVehicleBody> BODIES = new ConcurrentHashMap<>();
    /** Largest observed mounted-collider footprint and owning level per live IV entity. */
    private static final Map<UUID, RetainedLoadingFootprint> RETAINED_LOADING_RADII =
        new ConcurrentHashMap<>();
    private static final Map<UUID, Long> NEXT_BODY_RETRY_GAME_TIME = new ConcurrentHashMap<>();
    private static final Map<UUID, Long> BODY_RETRY_DELAY_TICKS = new ConcurrentHashMap<>();
    private static final Map<UUID, Long> BODY_CREATED_GAME_TIME = new ConcurrentHashMap<>();
    private static final Map<UUID, Long> LAST_GEOMETRY_WAIT_LOG_TIME = new ConcurrentHashMap<>();
    private static final Map<UUID, Long> LAST_UNLOADED_FOOTPRINT_LOG_TIME = new ConcurrentHashMap<>();
    private static final Map<UUID, Long> SABLE_RESIDENCY_HOLD_TICKS = new ConcurrentHashMap<>();
    private static final Map<UUID, Long> LAST_SABLE_RESIDENCY_HOLD_LOG_TIME = new ConcurrentHashMap<>();
    /** Rider reconnects may be detected before the persistent Sable body has been recreated. */
    private static final Map<UUID, UUID> PENDING_RIDER_RECONNECT_HOLDS = new ConcurrentHashMap<>();
    /** Tracks whether this live IV entity has ever completed Sable activation. */
    private static final java.util.Set<UUID> SABLE_ACTIVATED_ONCE = ConcurrentHashMap.newKeySet();
    private static final Map<SubLevelPhysicsSystem, Boolean> LOGGED_INHERITED_SUBSTEP_POLICY =
        java.util.Collections.synchronizedMap(new java.util.WeakHashMap<>());

    private SableVehicleManager() {
    }

    /** Special IV towing/road-following and blimp modes retain their existing authority. */
    public static boolean supportsVehicle(EntityVehicleF_Physics vehicle) {
        return vehicle != null && vehicle.definition != null && vehicle.definition.motorized != null
            && !vehicle.definition.motorized.isBlimp && vehicle.towedByConnection == null && !vehicle.lockedOnRoad;
    }

    private static AirframeLoads.SolveResult calculateVehicleLoads(EntityVehicleF_Physics vehicle,
            ServerLevel level, PMWeatherIVConfig.Values config, AircraftState state) {
        return vehicle.definition.motorized.isAircraft
            ? AircraftPhysics.calculateSableTickLoads(vehicle, level, config, state)
            : com.g9third.pmweatheriv.physics.RoadVehiclePhysics.calculateSableTickLoads(vehicle, level, config, state);
    }

    private static void advanceAutoTrim(
        EntityVehicleF_Physics vehicle, ServerLevel level, AircraftState state,
        SableVehicleBody body, AirframeLoads.SolveResult result
    ) {
        AutoTrimController controller = state.autoTrim;
        if (!controller.enabled()) return;
        UUID pilotId = controller.pilot();
        ServerPlayer pilot = pilotId == null ? null : level.getServer().getPlayerList().getPlayer(pilotId);
        if (pilot == null || pilot.level() != level
            || !com.g9third.pmweatheriv.network.AutoTrimNetwork.isCurrentController(pilot, vehicle)) {
            cancelAutoTrim(vehicle, level, state, "DISMOUNTED");
            return;
        }
        AutoTrimController.Output output = AutoTrimRuntime.advance(
            vehicle, state, result, body, level.getGameTime()
        );
        String fitDiagnostic = controller.consumeFitDiagnostic();
        if (fitDiagnostic != null && PMIVObserver.loggingEnabled()) {
            PMIVObserver.log("PMIV_AUTO_TRIM_FIT vehicleUuid=" + vehicle.uniqueUUID
                + " " + fitDiagnostic);
        }
        double currentTrim = vehicle.elevatorTrimVar.currentValue;
        if (output.enabled() && Double.isFinite(output.trim())
            && Math.abs(output.trim() - currentTrim) > 1.0E-6) {
            vehicle.elevatorTrimVar.setTo(output.trim(), true);
        }
        boolean changed = state.lastAutoTrimStatusState != output.state()
            || !state.lastAutoTrimStatusReason.equals(output.reason());
        if (changed && PMIVObserver.loggingEnabled()) {
            double flightPath = Double.NaN;
            if (state.kinematics != null && state.kinematics.centerVelocityWorld() != null) {
                Vec3d velocity = state.kinematics.centerVelocityWorld();
                flightPath = Math.toDegrees(Math.atan2(velocity.y(),
                    Math.max(1.0E-5, Math.hypot(velocity.x(), velocity.z()))));
            }
            PMIVObserver.log("PMIV_AUTO_TRIM_TRANSITION vehicleUuid=" + vehicle.uniqueUUID
                + " state=" + output.state() + " reason=" + output.reason()
                + " currentTrim=" + currentTrim + " requestedTrim=" + output.trim()
                + " flightPathDegrees=" + flightPath);
        }
        if (changed || state.lastAutoTrimStatusTick == Long.MIN_VALUE
            || level.getGameTime() - state.lastAutoTrimStatusTick >= 10L) {
            com.g9third.pmweatheriv.network.AutoTrimNetwork.sendStatus(pilot, vehicle, state, level);
            state.lastAutoTrimStatusState = output.state();
            state.lastAutoTrimStatusReason = output.reason();
            state.lastAutoTrimStatusTick = level.getGameTime();
        }
    }

    private static void cancelAutoTrim(EntityVehicleF_Physics vehicle, ServerLevel level,
                                       AircraftState state, String reason) {
        UUID pilotId = state.autoTrim.pilot();
        if (pilotId == null) return;
        state.autoTrim.disable(reason);
        state.nextAutoTrimDamageCheck = Long.MIN_VALUE;
        state.lastAutoTrimStatusState = AutoTrimController.State.OFF;
        state.lastAutoTrimStatusReason = reason;
        state.lastAutoTrimStatusTick = level.getGameTime();
        com.g9third.pmweatheriv.network.AutoTrimNetwork.sendStatusToPilot(pilotId, vehicle, state, level);
    }

    public static SableVehicleBody getBody(EntityVehicleF_Physics vehicle) {
        if (vehicle == null || vehicle.uniqueUUID == null) return null;
        SableVehicleBody body = BODIES.get(vehicle.uniqueUUID);
        return body != null && body.isUsable() ? body : null;
    }

    public static double roadSuspensionTravel(PartGroundDevice device) {
        if (device == null || device.vehicleOn == null) return 0.0;
        SableVehicleBody body = getBody(device.vehicleOn);
        return body == null ? 0.0 : body.roadSuspensionTravel(device);
    }

    /** True only when Rapier owns the vehicle's real physical collision shape. */
    public static boolean hasSableCollisionAuthority(EntityVehicleF_Physics vehicle) {
        SableVehicleBody body = getBody(vehicle);
        return body != null && body.hasCompoundCollisionAuthority();
    }

    /** Begins a short Sable pose hold while IV repairs a saved linked-seat rider. */
    public static boolean beginRiderReconnectHold(
        EntityVehicleF_Physics vehicle,
        UUID playerUuid
    ) {
        if (vehicle == null || vehicle.uniqueUUID == null || playerUuid == null) {
            return false;
        }
        UUID vehicleUuid = vehicle.uniqueUUID;
        SableVehicleBody body = BODIES.get(vehicleUuid);
        if (body != null && body.isUsable()) {
            return body.beginRiderReconnectHold(playerUuid);
        }
        // A stale/unusable body is about to be replaced by the normal lifecycle.
        // Keep the request pending so the replacement body starts held rather than
        // losing the reconnect ordering protection during that transition.
        PENDING_RIDER_RECONNECT_HOLDS.put(vehicleUuid, playerUuid);
        if (PMIVObserver.loggingEnabled()) {
            PMIVObserver.log(
            "PMIV_RIDER_RECONNECT_HOLD_PENDING uuid=" + vehicleUuid
                + " playerUuid=" + String.valueOf(playerUuid)
                + " reason=SABLE_BODY_NOT_YET_AVAILABLE"
        );
        }
        return true;
    }

    /** Releases a reconnect hold and restores the exact pre-hold Sable momentum. */
    public static void endRiderReconnectHold(
        EntityVehicleF_Physics vehicle,
        UUID playerUuid,
        String reason
    ) {
        if (vehicle == null || vehicle.uniqueUUID == null) {
            return;
        }
        UUID vehicleUuid = vehicle.uniqueUUID;
        PENDING_RIDER_RECONNECT_HOLDS.remove(vehicleUuid);
        SableVehicleBody body = BODIES.get(vehicleUuid);
        if (body != null) {
            body.endRiderReconnectHold(playerUuid, reason);
        }
    }

    public static void removeBody(EntityVehicleF_Physics vehicle) {
        UUID uuid = vehicle.uniqueUUID;
        SableVehicleBody body = BODIES.remove(uuid);
        RoadSuspensionNetwork.forget(body == null ? null : body.level(), uuid);
        if (body != null) {
            PMIVObserver.vehicleRemoved(uuid, "iv-vehicle-removed-or-physics-disabled");
        }
        safeRemove(body, "sableFlightBodyRemove");
        clearLifecycleState(uuid);
    }

    /**
     * Clears bookkeeping for an IV entity removed before Sable registered its body.
     * Active native bodies stay on their established physics-thread retirement path.
     */
    public static void onVehicleEntityRemoved(EntityVehicleF_Physics vehicle) {
        if (vehicle == null || vehicle.uniqueUUID == null) {
            return;
        }
        UUID uuid = vehicle.uniqueUUID;
        if (shouldClearLifecycleAfterEntityRemoval(BODIES.containsKey(uuid))) {
            clearLifecycleState(uuid);
        }
    }

    private static boolean shouldClearLifecycleAfterEntityRemoval(boolean bodyRemainsRegistered) {
        return !bodyRemainsRegistered;
    }

    private static boolean sameLevelOwner(Object recordedLevel, Object unloadingLevel) {
        return recordedLevel == unloadingLevel;
    }

    /**
     * Queues distributed loads for a usable Sable body. Missing body or terrain
     * residency holds motion while preserving the last physical momentum.
     */
    public static AirframeLoads.SolveResult calculateAndQueueLoads(
        EntityVehicleF_Physics vehicle,
        ServerLevel level,
        PMWeatherIVConfig.Values config,
        AircraftState state
    ) {
        UUID uuid = vehicle.uniqueUUID;
        SableVehicleBody body = BODIES.get(uuid);
        if (body != null && (!body.isRegisteredFor(vehicle, level) || !body.isUsable())) {
            retainLoadingRadius(uuid, body);
            BODIES.remove(uuid, body);
            RoadSuspensionNetwork.forget(level, uuid);
            safeRemove(body, "sableFlightBodyRetire");
            scheduleRetry(uuid, level.getGameTime());
            body = null;
        }

        if (body == null && state.autoTrim.enabled()) {
            cancelAutoTrim(vehicle, level, state, "PHYSICS_HOLD");
        }

        if (body != null) {
            logInheritedSubstepPolicy(body.physicsSystem(), uuid);
            resetRetryAfterHealthyLifetime(uuid, level.getGameTime());
            // Consume the prior completed Rapier tick before mirroring the new
            // absolute pose. Sable/Rapier (plus Sable swept CCD) is the only
            // terrain-contact detector; this bridge applies IV 24 gameplay
            // damage/destruction after that physical contact without feeding a
            // second COM impulse or angular correction back into the rigid body.
            // Breakable blocks are removed here, after the proven contact and
            // before the next Sable tick sees the world again. A reconnect hold
            // deliberately suppresses crash translation while its body is pinned.
            if (!body.riderReconnectHeld()) {
                body.advanceCrashEpisodeLifecycle();
                body.processCollisionGameplay();
            }
            if (!vehicle.isValid) {
                // IV-compatible native destroy() permanently removed the aircraft and already
                // emitted its explosion/fire/drops. Retire the matching Sable body
                // immediately so no later substep can keep integrating a wreck that
                // no longer exists in IV gameplay/network state.
                AirframeLoads.SolveResult previous = body.lastTickResult();
                BODIES.remove(uuid, body);
                RoadSuspensionNetwork.forget(level, uuid);
                safeRemove(body, "sableFlightNativeIvCrashDestroyed");
                clearLifecycleState(uuid);
                if (previous != null) {
                    return previous;
                }
                // The spawn-overlap collision-only cycle cannot pass IV's 500-tick
                // blockBreakDelay, so this is only a defensive unreachable path.
                return calculateVehicleLoads(
                    vehicle, level, config, state
                );
            }
            body.syncIvRequestFromSablePose(state);
            body.refreshCompoundCollider();
            AirframeLoads.SolveResult result = calculateVehicleLoads(
                vehicle, level, config, state
            );
            if (body.needsMassRefresh(result)) {
                try {
                    body.refreshMassProperties(result);
                } catch (RuntimeException | Error failure) {
                    // A mass refresh must never destroy/recreate the live collision
                    // body. Continue with the existing rolled-back mass properties
                    // and let Sable retain pose/contact/momentum authority.
                    if (PMIVObserver.loggingEnabled()) {
                        PMIVObserver.log(
                        "ERROR stage=sableMassPropertiesUpdate uuid=" + vehicle.uniqueUUID
                            + " type=" + failure.getClass().getSimpleName()
                            + " message=" + String.valueOf(failure.getMessage()).replace('\n', ' ')
                            + " bodyPreserved=true"
                            + " collisionAuthority=SABLE_RAPIER_COMPOUND"
                    );
                    }
                }
            }
            advanceAutoTrim(vehicle, level, state, body, result);
            body.setLoads(result, level.getGameTime(), config, state);
            return result;
        }

        long gameTime = level.getGameTime();
        SableCompoundCollider.GeometryReadiness readiness =
            SableCompoundCollider.geometryReadiness(vehicle);

        // IV populates allCollisionBoxes after the first force callback. Do not
        // create a doomed Rapier anchor and then enter exponential backoff merely
        // because that update-order boundary has not run yet. Hold motion for this tick and retry on the next tick.
        if (!readiness.ready()) {
            logGeometryWait(vehicle, readiness, gameTime);
            NEXT_BODY_RETRY_GAME_TIME.remove(uuid);
            BODY_RETRY_DELAY_TICKS.remove(uuid);
            return holdForSable(
                vehicle, level, config, state, "GEOMETRY_" + readiness.reason()
            );
        }

        SubLevelPhysicsSystem physicsSystem = SubLevelPhysicsSystem.get(level);
        if (physicsSystem != null && retryAllowed(uuid, gameTime)) {
            // Once collision geometry is ready, do not integrate analytically and
            // then seed Sable from that already-integrated velocity. Calculate the
            // same physical loads without advancing state, seed Rapier from the
            // current IV state, and let Sable become the sole integrator.
            AirframeLoads.SolveResult loadOnlyResult =
                calculateVehicleLoads(vehicle, level, config, state);
            BodyCreationAttempt creation = tryCreateBody(
                vehicle,
                level,
                loadOnlyResult,
                bodyStateFromCurrentIvState(vehicle, state)
            );
            SableVehicleBody created = creation.body();
            if (created != null) {
                // Mirror the just-created body's unchanged pose immediately so IV
                // cannot also apply a duplicate analytic movement request this
                // tick. Deliberately leave lastLoadsGameTime unset: the first
                // Sable substep is collision-only (gravity is cancelled by the
                // stale-load branch), allowing any spawn overlap to resolve for
                // the first physics cycle before loads resume next vehicle tick.
                created.syncIvRequestFromSablePose(state);
                if (PMIVObserver.loggingEnabled()) {
                    PMIVObserver.log(
                    "SABLE_BODY_ACTIVATED uuid=" + uuid
                        + " gameTime=" + gameTime
                        + " authoredBlockBoxes=" + readiness.authoredBlockBoxes()
                        + " ownerLocalReadyBlockBoxes=" + readiness.ownerLocalReadyBlockBoxes()
                        + " geometryReadinessSource=" + readiness.reason()
                        + " initialIntegration=RAPIER_PLUS_LIVE_GEAR_FIRST_PHYSICS_CYCLE"
                        + " sableSubsteps=" + physicsSystem.getConfig().substepsPerTick
                        + " duplicateAnalyticMovement=false"
                        + " nextTickLoads=NORMAL_PMIV_AERO_PROPULSION"
                );
                }
                LAST_GEOMETRY_WAIT_LOG_TIME.remove(uuid);
                LAST_UNLOADED_FOOTPRINT_LOG_TIME.remove(uuid);
                return loadOnlyResult;
            }
            if (creation.unloadedFootprint()) {
                // Chunk residency is not a Sable runtime failure.  Do not advance
                // an analytic gravity/aero integrator while there is no Rapier
                // terrain body to catch the aircraft.  Preserve the current IV
                // pose/momentum, return this load-only diagnostic snapshot, and
                // re-check Sable eligibility on the very next vehicle tick.
                NEXT_BODY_RETRY_GAME_TIME.remove(uuid);
                BODY_RETRY_DELAY_TICKS.remove(uuid);
                logUnloadedFootprintHold(vehicle, gameTime);
                holdMotionRequest(vehicle, state);
                return loadOnlyResult;
            }
            // Creation already evaluated this tick's native propulsion. Hold
            // using those loads instead of invoking the actuator a second time.
            holdMotionRequest(vehicle, state);
            return loadOnlyResult;
        }

        // Availability failures hold motion while the normal retry backoff runs.
        return holdForSable(
            vehicle, level, config, state,
            physicsSystem == null ? "SABLE_PHYSICS_SYSTEM_UNAVAILABLE" : "SABLE_BODY_RETRY_BACKOFF"
        );
    }

    private static AirframeLoads.SolveResult holdForSable(
        EntityVehicleF_Physics vehicle,
        ServerLevel level,
        PMWeatherIVConfig.Values config,
        AircraftState state,
        String reason
    ) {
        UUID uuid = vehicle.uniqueUUID;
        long count = SABLE_RESIDENCY_HOLD_TICKS.merge(uuid, 1L, Long::sum);
        long gameTime = level.getGameTime();
        long last = LAST_SABLE_RESIDENCY_HOLD_LOG_TIME.getOrDefault(uuid, Long.MIN_VALUE);
        if (last == Long.MIN_VALUE || gameTime < last || gameTime - last >= 20L) {
            LAST_SABLE_RESIDENCY_HOLD_LOG_TIME.put(uuid, gameTime);
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "SABLE_RESIDENCY_HOLD uuid=" + uuid
                    + " gameTime=" + gameTime
                    + " reason=" + reason
                    + " cumulativeTicks=" + count
                    + " normalServerAuthority=false"
                    + " nativeIvFlightFallback=false"
            );
            }
        }
        AirframeLoads.SolveResult loads = state.isPrepared()
            ? calculateVehicleLoads(vehicle, level, config, state) : null;
        holdMotionRequest(vehicle, state);
        return loads;
    }


    private static BodyCreationAttempt tryCreateBody(
        EntityVehicleF_Physics vehicle,
        ServerLevel level,
        AirframeLoads.SolveResult initialResult,
        SableVehicleBody.BodyState initialState
    ) {
        UUID uuid = vehicle.uniqueUUID;
        SubLevelPhysicsSystem physicsSystem = SubLevelPhysicsSystem.get(level);
        if (physicsSystem == null) {
            scheduleRetry(uuid, level.getGameTime());
            return BodyCreationAttempt.FAILED;
        }
        SableVehicleBody created = null;
        try {
            boolean firstSableActivation = !SABLE_ACTIVATED_ONCE.contains(uuid);
            created = new SableVehicleBody(
                vehicle, level, initialResult, initialState, firstSableActivation,
                retainedLoadingRadius(uuid)
            );
            retainLoadingRadius(uuid, created);
            // Sable's ticket manager removes an arbitrary object whenever any
            // chunk touched by its rotation-independent loading footprint is not
            // block-ticking.  Preflight the same predicate before addObject.
            // This is a residency condition, not a failed native creation: never
            // exponential-backoff it and never apply analytic gravity while the
            // aircraft has no Rapier terrain body.  The owner tick will retry this
            // predicate again immediately on the next vehicle tick.
            if (!physicsSystem.getTicketManager().wouldBeLoaded(level, created)) {
                NEXT_BODY_RETRY_GAME_TIME.remove(uuid);
                BODY_RETRY_DELAY_TICKS.remove(uuid);
                logUnloadedFootprintDeferred(
                    uuid, level.getGameTime(), created.loadingRadius(),
                    "PRE_ADD_MODEL_OR_RETAINED", false, false
                );
                return BodyCreationAttempt.UNLOADED_FOOTPRINT;
            }
            physicsSystem.addObject(created);
            retainLoadingRadius(uuid, created);
            // onAddition has initialized the complete compound, so this second
            // check sees its actual extent. Do not publish or mark first activation
            // until the Sable ticket footprint accepts that measured bound.
            if (!physicsSystem.getTicketManager().wouldBeLoaded(level, created)) {
                NEXT_BODY_RETRY_GAME_TIME.remove(uuid);
                BODY_RETRY_DELAY_TICKS.remove(uuid);
                double measuredRadius = created.loadingRadius();
                physicsSystem.removeObject(created);
                retainLoadingRadius(uuid, created);
                logUnloadedFootprintDeferred(
                    uuid, level.getGameTime(), measuredRadius,
                    "POST_ADD_MEASURED_COMPOUND", true, true
                );
                return BodyCreationAttempt.UNLOADED_FOOTPRINT;
            }
            if (!created.isUsable()) {
                throw new IllegalStateException("Sable body became unusable during footprint validation");
            }
            logInheritedSubstepPolicy(physicsSystem, uuid);
            BODIES.put(uuid, created);
            SABLE_ACTIVATED_ONCE.add(uuid);
            BODY_CREATED_GAME_TIME.put(uuid, level.getGameTime());
            UUID pendingReconnectPlayer = PENDING_RIDER_RECONNECT_HOLDS.get(uuid);
            if (pendingReconnectPlayer != null) {
                created.beginRiderReconnectHold(pendingReconnectPlayer);
            }
            NEXT_BODY_RETRY_GAME_TIME.remove(uuid);
            long compatibilityTicks = SABLE_RESIDENCY_HOLD_TICKS.getOrDefault(uuid, 0L);
            if (compatibilityTicks > 0L) {
                if (PMIVObserver.loggingEnabled()) {
                    PMIVObserver.log(
                    "SABLE_RESIDENCY_HOLD_ENDED uuid=" + uuid
                        + " totalTicks=" + compatibilityTicks
                        + " nextAuthority=SABLE_RAPIER"
                );
                }
            }
            SABLE_RESIDENCY_HOLD_TICKS.remove(uuid);
            LAST_SABLE_RESIDENCY_HOLD_LOG_TIME.remove(uuid);
            return new BodyCreationAttempt(created, false);
        } catch (RuntimeException exception) {
            // SubLevelPhysicsSystem adds the arbitrary object to its identity set
            // before invoking onAddition(). If compound creation throws from
            // onAddition, remove that Java object from the set as well as any
            // native handle cleanup SableVehicleBody already performed.
            if (created != null) {
                retainLoadingRadius(uuid, created);
                try {
                    physicsSystem.removeObject(created);
                } catch (RuntimeException ignored) {
                    // Preserve the original creation failure and retry normally.
                }
            }
            scheduleRetry(uuid, level.getGameTime());
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "ERROR stage=sableFlightBodyCreate uuid=" + uuid
                    + " retryBackoffTicks=" + BODY_RETRY_DELAY_TICKS.getOrDefault(
                        uuid, INITIAL_RETRY_TICKS
                    )
                    + " type=" + exception.getClass().getSimpleName()
                    + " message=" + String.valueOf(exception.getMessage()).replace('\n', ' ')
            );
            }
            return BodyCreationAttempt.FAILED;
        }
    }

    private static SableVehicleBody.BodyState bodyStateFromCurrentIvState(
        EntityVehicleF_Physics vehicle,
        AircraftState state
    ) {
        double ivVelocityScale = Math.max(1.0E-6, vehicle.speedFactor * 20.0);
        Vec3d velocity = state.hasPhysicalVelocity ? state.originVelocityWorld
            : new Vec3d(vehicle.motion.x * ivVelocityScale, vehicle.motion.y * ivVelocityScale,
                vehicle.motion.z * ivVelocityScale);
        Vector3d modelOriginLinear = new Vector3d(velocity.x(), velocity.y(), velocity.z());
        Vec3d angularBodyState = state.angularVelocityBody;
        Vector3d angularBody = new Vector3d(
            angularBodyState.x(), angularBodyState.y(), angularBodyState.z()
        );
        Quaterniond orientation = SablePoseConversions.toQuaternion(vehicle.orientation);
        Vector3d angularWorld = SablePoseConversions.localToWorld(
            orientation, angularBody, new Vector3d()
        );
        Vec3d centerLocal = state.plan.centerOfMassLocal() == null
            || !state.plan.centerOfMassLocal().isFinite()
                ? Vec3d.ZERO : state.plan.centerOfMassLocal();
        Vector3d centerOffsetWorld = new Vector3d(
            centerLocal.x(), centerLocal.y(), centerLocal.z()
        );
        orientation.transform(centerOffsetWorld);
        Vector3d rotationalCenterVelocity = new Vector3d();
        angularWorld.cross(centerOffsetWorld, rotationalCenterVelocity);
        Vector3d centerOfMassLinear = new Vector3d(modelOriginLinear)
            .add(rotationalCenterVelocity);
        Vector3d centerOfMassPosition = new Vector3d(
            vehicle.position.x, vehicle.position.y, vehicle.position.z
        ).add(centerOffsetWorld);
        return new SableVehicleBody.BodyState(
            centerOfMassLinear,
            angularWorld,
            new Vector3d(centerOfMassLinear),
            new Vector3d(angularBody),
            state.hasPhysicalVelocity,
            centerOfMassPosition,
            new Quaterniond(orientation)
        );
    }

    private static void logUnloadedFootprintDeferred(
        UUID uuid,
        long gameTime,
        double loadingRadius,
        String validationStage,
        boolean nativeBodyCreatedDuringAttempt,
        boolean nativeCompoundCreatedDuringAttempt
    ) {
        long last = LAST_UNLOADED_FOOTPRINT_LOG_TIME.getOrDefault(uuid, Long.MIN_VALUE);
        if (last != Long.MIN_VALUE && gameTime >= last && gameTime - last < 20L) {
            return;
        }
        LAST_UNLOADED_FOOTPRINT_LOG_TIME.put(uuid, gameTime);
        if (PMIVObserver.loggingEnabled()) {
            PMIVObserver.log(
            "SABLE_BODY_CREATION_DEFERRED_UNLOADED_FOOTPRINT uuid=" + uuid
                + " gameTime=" + gameTime
                + " loadingRadius=" + loadingRadius
                + " validationStage=" + validationStage
                + " retryPolicy=NEXT_VEHICLE_TICK_NO_BACKOFF"
                + " nativeBodyCreatedDuringAttempt=" + nativeBodyCreatedDuringAttempt
                + " nativeCompoundCreatedDuringAttempt=" + nativeCompoundCreatedDuringAttempt
                + " chunksForceLoaded=false"
                + " fallback=HOLD_CURRENT_IV_STATE_NO_ANALYTIC_GRAVITY"
        );
        }
    }

    private static void logUnloadedFootprintHold(
        EntityVehicleF_Physics vehicle,
        long gameTime
    ) {
        UUID uuid = vehicle.uniqueUUID;
        long last = LAST_UNLOADED_FOOTPRINT_LOG_TIME.getOrDefault(uuid, Long.MIN_VALUE);
        // The deferred log above normally owns this interval. Emit a hold row only
        // when a caller reaches this path without one in the last second.
        if (last != Long.MIN_VALUE && gameTime >= last && gameTime - last < 20L) {
            return;
        }
        LAST_UNLOADED_FOOTPRINT_LOG_TIME.put(uuid, gameTime);
        if (PMIVObserver.loggingEnabled()) {
            PMIVObserver.log(
            "SABLE_UNLOADED_FOOTPRINT_HOLD uuid=" + uuid
                + " gameTime=" + gameTime
                + " movementPolicy=PRESERVE_CURRENT_IV_STATE"
                + " gravityIntegration=false"
                + " aerodynamicIntegration=false"
                + " retryPolicy=NEXT_VEHICLE_TICK_NO_BACKOFF"
                + " chunksForceLoaded=false"
        );
        }
    }

    private static void logGeometryWait(
        EntityVehicleF_Physics vehicle,
        SableCompoundCollider.GeometryReadiness readiness,
        long gameTime
    ) {
        UUID uuid = vehicle.uniqueUUID;
        long last = LAST_GEOMETRY_WAIT_LOG_TIME.getOrDefault(uuid, Long.MIN_VALUE);
        if (last != Long.MIN_VALUE && gameTime - last < 20L) {
            return;
        }
        LAST_GEOMETRY_WAIT_LOG_TIME.put(uuid, gameTime);
        if (PMIVObserver.loggingEnabled()) {
            PMIVObserver.log(
            "SABLE_BODY_WAITING_FOR_IV_GEOMETRY uuid=" + uuid
                + " gameTime=" + gameTime
                + " reason=" + readiness.reason()
                + " authoredBlockBoxes=" + readiness.authoredBlockBoxes()
                + " ownerLocalReadyBlockBoxes=" + readiness.ownerLocalReadyBlockBoxes()
                + " retryPolicy=NEXT_VEHICLE_TICK_NO_BACKOFF"
        );
        }
    }

    /** Clears every per-vehicle lifecycle/backoff entry for a terminal retire. */
    private static void clearLifecycleState(UUID uuid) {
        NEXT_BODY_RETRY_GAME_TIME.remove(uuid);
        BODY_RETRY_DELAY_TICKS.remove(uuid);
        BODY_CREATED_GAME_TIME.remove(uuid);
        LAST_GEOMETRY_WAIT_LOG_TIME.remove(uuid);
        LAST_UNLOADED_FOOTPRINT_LOG_TIME.remove(uuid);
        SABLE_RESIDENCY_HOLD_TICKS.remove(uuid);
        LAST_SABLE_RESIDENCY_HOLD_LOG_TIME.remove(uuid);
        RETAINED_LOADING_RADII.remove(uuid);
        PENDING_RIDER_RECONNECT_HOLDS.remove(uuid);
        SABLE_ACTIVATED_ONCE.remove(uuid);
    }

    private record RetainedLoadingFootprint(ServerLevel level, double radius) {
    }

    private static void resetRetryAfterHealthyLifetime(UUID uuid, long gameTime) {
        long createdAt = BODY_CREATED_GAME_TIME.getOrDefault(uuid, gameTime);
        if (gameTime - createdAt < HEALTHY_BODY_RESET_TICKS) {
            return;
        }
        BODY_CREATED_GAME_TIME.remove(uuid);
        NEXT_BODY_RETRY_GAME_TIME.remove(uuid);
        BODY_RETRY_DELAY_TICKS.remove(uuid);
    }

    private static boolean retryAllowed(UUID uuid, long gameTime) {
        return gameTime >= NEXT_BODY_RETRY_GAME_TIME.getOrDefault(uuid, Long.MIN_VALUE);
    }

    private static void scheduleRetry(UUID uuid, long gameTime) {
        long delay = BODY_RETRY_DELAY_TICKS.getOrDefault(uuid, INITIAL_RETRY_TICKS);
        NEXT_BODY_RETRY_GAME_TIME.put(uuid, gameTime + delay);
        BODY_RETRY_DELAY_TICKS.put(uuid, Math.min(MAX_RETRY_TICKS, delay * 2L));
    }

    private static void safeRemove(SableVehicleBody body, String stage) {
        if (body == null) {
            return;
        }
        try {
            if (body.physicsSystem() != null) {
                body.physicsSystem().removeObject(body);
            }
        } catch (RuntimeException exception) {
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "ERROR stage=" + stage + " uuid=" + body.vehicle().uniqueUUID
                    + " type=" + exception.getClass().getSimpleName()
                    + " message=" + String.valueOf(exception.getMessage()).replace('\n', ' ')
            );
            }
        }
    }

    private static void retainLoadingRadius(UUID uuid, SableVehicleBody body) {
        retainLoadingRadius(uuid, body.level(), body.loadingRadius());
    }

    private static void retainLoadingRadius(UUID uuid, ServerLevel level, double radius) {
        if (Double.isFinite(radius) && radius > 0.0) {
            RETAINED_LOADING_RADII.compute(uuid, (ignored, previous) ->
                new RetainedLoadingFootprint(level,
                    Math.max(previous == null ? 0.0 : previous.radius(), radius)));
        }
    }

    private static double retainedLoadingRadius(UUID uuid) {
        RetainedLoadingFootprint footprint = RETAINED_LOADING_RADII.get(uuid);
        return footprint == null ? 0.0 : footprint.radius();
    }

    public static void onLevelUnload(LevelEvent.Unload event) {
        if (!(event.getLevel() instanceof ServerLevel level)) {
            return;
        }
        long sceneHandle = 0L;
        for (SableVehicleBody body : new ArrayList<>(BODIES.values())) {
            if (body.level() == level) {
                if (sceneHandle == 0L) {
                    sceneHandle = body.sceneHandle();
                }
                UUID uuid = body.vehicle().uniqueUUID;
                PMIVObserver.vehicleRemoved(uuid, "dimension-unloaded");
                RoadSuspensionNetwork.forget(level, uuid);
                BODIES.remove(uuid, body);
                safeRemove(body, "sableFlightLevelUnload");
                clearLifecycleState(uuid);
            }
        }
        for (Map.Entry<UUID, RetainedLoadingFootprint> entry : RETAINED_LOADING_RADII.entrySet()) {
            RetainedLoadingFootprint footprint = entry.getValue();
            if (sameLevelOwner(footprint.level(), level) && !BODIES.containsKey(entry.getKey())) {
                clearLifecycleState(entry.getKey());
            }
        }
        SableCollisionCapture.clearScene(sceneHandle);
    }

    public static void onPrePhysicsTick(ForgeSablePrePhysicsTickEvent event) {
        long sceneHandle = 0L;
        for (SableVehicleBody body : BODIES.values()) {
            if (body.physicsSystem() == event.getPhysicsSystem()
                && body.vehicle().isValid && body.isUsable()) {
                sceneHandle = body.sceneHandle();
                break;
            }
        }
        if (sceneHandle != 0L && isFirstPhysicsSubstep(event.getPhysicsSystem())) {
            // The pre-impact state must be captured before any aircraft applies
            // this substep's load impulse. Start the shared tick accumulator
            // before calling prepareSubstep on the first body.
            SableCollisionCapture.beginTick(sceneHandle);
        }
        // ConcurrentHashMap's values view is weakly consistent, so key removal
        // during iteration is supported. Avoid allocating a full body snapshot
        // on every configured Sable substep.
        for (SableVehicleBody body : BODIES.values()) {
            if (body.physicsSystem() != event.getPhysicsSystem()) {
                continue;
            }
            if (!body.vehicle().isValid || !body.isUsable()) {
                UUID uuid = body.vehicle().uniqueUUID;
                retainLoadingRadius(uuid, body);
                BODIES.remove(uuid, body);
                RoadSuspensionNetwork.forget(body.level(), uuid);
                safeRemove(body, "sableFlightPrePhysicsRetire");
                if (body.vehicle().isValid) {
                    // A transient native-body failure may retry, but the old
                    // creation timestamp must not survive into the replacement.
                    BODY_CREATED_GAME_TIME.remove(uuid);
                    scheduleRetry(uuid, body.level().getGameTime());
                } else {
                    PMIVObserver.vehicleRemoved(uuid, "iv-aircraft-invalid-prephysics");
                    clearLifecycleState(uuid);
                }
                continue;
            }
            try {
                body.prepareSubstep(event.getTimeStep());
            } catch (RuntimeException | LinkageError failure) {
                body.quarantineIntegrationFailure("preSubstep", failure);
            }
        }
    }

    public static void onPostPhysicsTick(ForgeSablePostPhysicsTickEvent event) {
        long sceneHandle = 0L;
        for (SableVehicleBody body : BODIES.values()) {
            if (body.physicsSystem() == event.getPhysicsSystem() && body.isUsable()) {
                try {
                    body.finishSubstep();
                } catch (RuntimeException | LinkageError failure) {
                    body.quarantineIntegrationFailure("postSubstep", failure);
                }
                long gameTime = body.level().getGameTime();
                if (body.markRoadSuspensionPublished(gameTime)) {
                    RoadSuspensionNetwork.publish(body.vehicle(), body.level(), gameTime,
                        body.roadSuspensionOffsets());
                }
                if (sceneHandle == 0L) {
                    sceneHandle = body.sceneHandle();
                }
            }
        }
        if (sceneHandle != 0L) {
            double[] collisions = Rapier3DInvoker.pmweatherIv$clearCollisions(sceneHandle);
            SableCollisionCapture.captureSubstep(
                sceneHandle, event.getTimeStep(), collisions
            );
        }
    }

    private static boolean isFirstPhysicsSubstep(SubLevelPhysicsSystem physicsSystem) {
        if (physicsSystem == null) {
            return false;
        }
        int substeps = Math.max(1, physicsSystem.getConfig().substepsPerTick);
        double firstFraction = 1.0 / substeps;
        return Math.abs(physicsSystem.getPartialPhysicsTick() - firstFraction) <= 1.0E-12;
    }

    /**
     * PMWeather-IV shares Sable's scene with Aeronautics and every other Sable
     * object. Never rewrite the scene-wide integration rate merely because an
     * aircraft exists. The user's Sable configuration remains authoritative;
     * Sable 2.x defaults to two substeps (40 Hz), but the user-configured 1-10 substep range remains authoritative.
     */
    private static void logInheritedSubstepPolicy(
        SubLevelPhysicsSystem physicsSystem,
        UUID triggeringVehicle
    ) {
        if (physicsSystem == null) {
            return;
        }
        synchronized (LOGGED_INHERITED_SUBSTEP_POLICY) {
            if (LOGGED_INHERITED_SUBSTEP_POLICY.putIfAbsent(physicsSystem, Boolean.TRUE) != null) {
                return;
            }
            int configured = Math.max(1, physicsSystem.getConfig().substepsPerTick);
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "SABLE_SUBSTEP_POLICY event=INHERITED_SCENE_CONFIGURATION"
                    + " uuid=" + triggeringVehicle
                    + " activeSubsteps=" + configured
                    + " contactRateHz=" + (20 * configured)
                    + " sceneConfigurationMutated=false"
                    + " reason=SHARED_SABLE_SCENE_RATE_REMAINS_USER_AND_AERONAUTICS_COMPATIBLE"
                    + " authority=SABLE_RAPIER"
            );
            }
        }
    }
    private record BodyCreationAttempt(SableVehicleBody body, boolean unloadedFootprint) {
        private static final BodyCreationAttempt FAILED = new BodyCreationAttempt(null, false);
        private static final BodyCreationAttempt UNLOADED_FOOTPRINT =
            new BodyCreationAttempt(null, true);
    }


    /** Freeze only the IV movement request while collision residency is unavailable. Retain momentum. */
    public static void holdMotionRequest(EntityVehicleF_Physics vehicle, AircraftState state) {
        if (!state.hasPhysicalVelocity) {
            double scale = Math.max(1E-6, vehicle.speedFactor * 20.0);
            state.originVelocityWorld = new Vec3d(vehicle.motion.x * scale, vehicle.motion.y * scale,
                vehicle.motion.z * scale);
            state.hasPhysicalVelocity = state.originVelocityWorld.isFinite();
        }
        vehicle.motion.set(0.0, 0.0, 0.0);
        vehicle.rotation.angles.set(0.0, 0.0, 0.0);
        state.heldForSable = true;
    }
}
