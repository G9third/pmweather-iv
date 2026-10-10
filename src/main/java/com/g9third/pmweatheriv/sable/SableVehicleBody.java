package com.g9third.pmweatheriv.sable;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import com.g9third.pmweatheriv.terrain.AircraftTerrainImpact;
import com.g9third.pmweatheriv.terrain.trueimpact.ExternalWorldImpactModel.Resolution;
import com.g9third.pmweatheriv.terrain.trueimpact.damage.ImpactRuntimeConfig;
import com.g9third.pmweatheriv.physics.FlightMath;
import com.g9third.pmweatheriv.physics.AircraftPhysics;
import com.g9third.pmweatheriv.physics.RotorModel;
import com.g9third.pmweatheriv.physics.PropulsionModel;
import com.g9third.pmweatheriv.physics.LandingGearSolver;
import com.g9third.pmweatheriv.physics.RoadSuspensionModel;
import com.g9third.pmweatheriv.physics.TireNormalCompliance;
import com.g9third.pmweatheriv.physics.AircraftWind;
import com.g9third.pmweatheriv.physics.AirframeLoads;
import com.g9third.pmweatheriv.physics.AirframeGeometry;
import com.g9third.pmweatheriv.physics.AircraftState;
import com.g9third.pmweatheriv.physics.NativeBallastModel;
import com.g9third.pmweatheriv.physics.LiquidSupportModel;
import com.g9third.pmweatheriv.physics.ModelSurfaceMap;
import com.g9third.pmweatheriv.physics.Vec3d;
import com.g9third.pmweatheriv.mixin.Rapier3DInvoker;
import com.g9third.pmweatheriv.mixin.EntityVehicleMovingAccessor;

import dev.ryanhcode.sable.api.physics.handle.RigidBodyHandle;
import dev.ryanhcode.sable.api.physics.mass.MassData;
import dev.ryanhcode.sable.api.physics.object.box.BoxPhysicsObject;
import dev.ryanhcode.sable.physics.config.dimension_physics.DimensionPhysicsData;
import dev.ryanhcode.sable.companion.math.BoundingBox3d;
import dev.ryanhcode.sable.companion.math.Pose3d;
import dev.ryanhcode.sable.sublevel.storage.holding.SubLevelHoldingChunkMap;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import minecrafttransportsimulator.jsondefs.JSONCollisionGroup.CollisionType;
import minecrafttransportsimulator.systems.ConfigSystem;
import net.minecraft.core.BlockPos;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.ChunkPos;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.shapes.VoxelShape;
import org.jetbrains.annotations.Nullable;
import org.joml.Matrix3d;
import org.joml.Matrix3dc;
import org.joml.Quaterniond;
import org.joml.Quaterniondc;
import org.joml.Vector3d;
import org.joml.Vector3dc;

/**
 * World-space persistent Sable/Rapier rigid body for managed IV vehicles.
 *
 * <p>Sable owns each managed vehicle's physical pose, linear/angular momentum
 * and body collision response. PMWeather-IV mounts active IV BLOCK cuboids as
 * compound LevelCollider children on the persistent body, so the complete
 * physical collision shape rotates with the vehicle and can collide directly
 * with terrain, other managed vehicles, and Sable/Aeronautics sublevels. IV
 * mirrors the resulting pose for networking/rendering/gameplay and does not
 * apply a second movement response.</p>
 *
 * <p>The rigid BODY shell supplies terrain-solid crash geometry. A shallow
 * low-envelope clearance at static rigid supports lets their exact point
 * constraints own the resting normal reaction without duplicate manifolds.
 * IV-authored stations provide predictive unilateral normal support plus a
 * per-contact friction-ellipse solve, authored steering/braking and
 * load/radius-derived rolling resistance. Authored float stations can carry
 * normal support at a fluid surface without solid-ground tire traction or impact
 * damage. If the body shell reaches terrain, Rapier still resolves that BODY
 * contact normally. There is no second pose solver or pack rule.</p>
 */
public final class SableVehicleBody extends BoxPhysicsObject {
    private final SableTerrainResponse terrainResponse = new SableTerrainResponse(this);
    private static final double GRAVITY = 9.80665;
    static final double MIN_INERTIA = 1.0E-3;
    private static final double MIN_HALF_EXTENT = 0.025;
    private static final double MAX_HALF_EXTENT = 64.0;
    /** The parent Box collider is only a rigid-body anchor; real geometry is mounted separately. */
    private static final double ANCHOR_HALF_EXTENT = 1.0E-4;
    /**
     * Narrow continuous-contact skin for rolling tires. This is deliberately
     * much smaller than IV's historical 0.05-block look-down shell, but large
     * enough that a tire does not lose all normal/lateral authority merely
     * because substep integration leaves it a few millimetres above a flat
     * runway. The constraint remains unilateral: an upward/separating wheel
     * receives no normal impulse and therefore no Coulomb lateral authority.
     */
    private static final double LIVE_GEAR_CONTACT_SKIN_METERS = 0.010;
    /** Matches IV's historical 0.05-block look-down only for diagnostics. */
    private static final double LIVE_GEAR_PROXIMITY_METERS = 0.05;
    /** Limits recovery to a local wheel/terrain support surface, not a remote block top. */
    private static final double LIVE_GEAR_MAX_PENETRATION_METERS = 0.75;
    private static final double LIVE_GEAR_HORIZONTAL_EPSILON = 1.0E-6;
    /** Preserve IV's 60-tick crash cadence for each gear part without blocking fuselage damage. */
    private static final long LANDING_GEAR_GAMEPLAY_DEBOUNCE_TICKS = 60L;
    /** Maximum one-time downward placement search for freshly spawned aircraft. */
    private static final double INITIAL_GROUND_SEATING_MAX_DROP_METERS = 3.0;
    private static final double INITIAL_GROUND_SEATING_TARGET_GAP_METERS = 0.005;
    /**
     * Numerical floor only. A wheel above the 10 mm live-contact skin must not be
     * called "already seated" merely because the remaining placement correction
     * is small. 0.8.3cu used 25 mm here, leaving ordinary 11-24 mm wheel gaps in
     * the first collision-only Sable cycle where BODY could touch before gear.
     */
    private static final double INITIAL_GROUND_SEATING_MIN_DROP_METERS = 0.001;
    private static final double INITIAL_GROUND_SEATING_MAX_LINEAR_SPEED_MPS = 1.0;
    private static final double INITIAL_GROUND_SEATING_MAX_ANGULAR_SPEED_RADPS = 0.35;
    /**
     * One-time fresh-placement attitude repair is bounded so it cannot turn a
     * deliberately airborne/sideways vehicle into a parked aircraft. The target
     * attitude is derived entirely from the vehicle's own active gear footprint
     * and the sampled terrain plane; this is only a sanity bound.
     */
    private static final double INITIAL_GROUND_POSE_MAX_ATTITUDE_REPAIR_DEGREES = 35.0;
    private static final double INITIAL_GROUND_POSE_MIN_SUPPORT_TRIANGLE_AREA2 = 1.0E-4;
    private static final double INITIAL_GROUND_POSE_MIN_TERRAIN_NORMAL_Y = 0.70;
    private static final long INITIAL_GROUND_POSE_MAX_PLACED_TICKS = 5L;
    // Last-resort creation-boundary guard. A fresh placement must never hand an
    // already-overlapping compound to dynamic Rapier: contact recovery can turn
    // a spawn overlap into a multi-metre launch before the normal tire solver has
    // any chance to establish support. Search only upward and only during the
    // one-time placement boundary; ordinary flight/crash recovery is unchanged.
    private static final double INITIAL_TERRAIN_CLEARANCE_STEP_METERS = 0.10;
    private static final int INITIAL_TERRAIN_CLEARANCE_BINARY_STEPS = 16;

    /**
     * Wreck terrain-damage continuation is physical-state driven.  A fatal impact
     * may keep fracturing terrain while the same wreck still carries meaningful
     * translational/rotational point speed, but once it remains settled below
     * this speed for the hysteresis window the gate latches closed permanently.
     * These values do not alter Sable motion; they only decide whether already
     * solved contact energy may still be submitted to True Impact.
     */
    private static final double CRASH_EPISODE_SETTLED_POINT_SPEED_MPS = 0.75;
    private static final int CRASH_EPISODE_SETTLED_TICKS = 10;

    final EntityVehicleF_Physics vehicle;
    final ServerLevel level;
    double mass;
    Vec3d desiredInertia;
    /** Frozen physical COM in IV/model-local coordinates; the Rapier parent origin lives here. */
    private final Vec3d centerOfMassLocal;
    private AircraftMassData massData;
    private final Vector3d modelHalfExtents;
    private final String modelLocation;
    private final @Nullable BodyState initialState;
    private final boolean initialGroundSeatingAllowed;
    /** Largest measured/cached full-body footprint; survives collider removal on chunk unload. */
    private double retainedLoadingRadius;

    private final Vector3d cachedExternalForceWorld = new Vector3d();
    private double cachedBallastForceWorldY;
    private final Vector3d cachedPropulsionForceBody = new Vector3d();
    private final Vector3d temporaryPropulsionForceWorld = new Vector3d();
    private final Vector3d cachedPropulsionTorqueBody = new Vector3d();
    final Vector3d sableGravityWorld = new Vector3d(0.0, -11.0, 0.0);
    private final Vector3d gravityCompensationForceWorld = new Vector3d();
    private final Vector3d cachedNetTorqueBody = new Vector3d();
    private final Vector3d temporaryNetTorqueBody = new Vector3d();
    final Quaterniond cachedOrientation = new Quaterniond();
    private PMWeatherIVConfig.Values cachedConfig;
    private AircraftState cachedAircraftState;
    private AircraftWind.WindFieldSnapshot cachedWindFieldSnapshot;
    private List<RotorModel.RotorActuatorSnapshot> cachedRotorActuators = List.of();
    private boolean airVehicleSubstepAerodynamics;
    private AirframeLoads.SolveResult lastTickResult;
    private LandingGearSolver.LandingGearConstraintResult lastSubstepLandingGear;
    /** Axial wheel travel keyed by authored real ground-device master parts. */
    private final Map<PartGroundDevice, Double> roadSuspensionTravel = new IdentityHashMap<>();
    private long lastRoadSuspensionPublishTick = Long.MIN_VALUE;

    final Vector3d temporaryLinearVelocityWorld = new Vector3d();
    private final Vector3d temporaryPredictedLinearVelocityWorld = new Vector3d();
    final Vector3d temporaryAngularVelocityWorld = new Vector3d();
    final Vector3d temporaryAngularVelocityBody = new Vector3d();
    final Vector3d temporaryLinearCorrectionWorld = new Vector3d();
    private final Vector3d temporaryAngularCorrectionBody = new Vector3d();
    final Vector3d temporaryAngularCorrectionWorld = new Vector3d();
    private final Vector3d temporaryAppliedForceWorld = new Vector3d();
    private final Vector3d temporaryLocalForce = new Vector3d();
    private final Vector3d temporaryImpulse = new Vector3d();
    private final Vector3d temporaryTorqueImpulse = new Vector3d();
    private final RotationMatrix temporaryPhysicalOrientation = new RotationMatrix();

    private final Vector3d lastExportedLinearVelocityWorld = new Vector3d();
    private final Vector3d lastExportedAngularVelocityBody = new Vector3d();
    private boolean hasExportedState;

    private SubLevelPhysicsSystem physicsSystem;
    RigidBodyHandle rigidBodyHandle;
    SableCompoundCollider compoundCollider;
    private long sceneHandle;
    private long lastLoadsGameTime = Long.MIN_VALUE;
    private long lastProcessedCollisionGeneration;
    private boolean substepArmed;
    private boolean loggedTransientFreshPlacementGear;
    /** A non-finite native body is quarantined immediately and retired on the next owner tick. */
    private boolean poisoned;
    private String poisonReason = "";

    // A reconnecting vanilla rider may remain attached to IV's invisible linked-seat
    // proxy while IV is still rebuilding the saved PartSeat/rider relationship. During
    // that short handshake PMIV pins the physical body in place, preserving the exact
    // pre-hold linear/angular momentum for restoration once both sides agree again.
    // This is lifecycle recovery only: it never trims, damps or alters normal flight.
    private boolean riderReconnectHeld;
    private UUID riderReconnectPlayerUuid;
    private long riderReconnectHoldStartGameTime = Long.MIN_VALUE;
    private final Vector3d riderReconnectSavedLinearVelocityWorld = new Vector3d();
    private final Vector3d riderReconnectSavedAngularVelocityWorld = new Vector3d();
    private final Vector3d riderReconnectHoldPositionWorld = new Vector3d();
    private final Quaterniond riderReconnectHoldOrientationWorld = new Quaterniond();

    // Crash-episode terrain-damage lifecycle.  Healthy aircraft may start and
    // naturally clear episodes repeatedly; once an out-of-health wreck settles,
    // terrain damage is one-way latched closed and later wind/player nudges cannot
    // reopen it.
    private boolean crashEpisodeActive;
    private boolean wreckTerrainDamageClosed;
    private int crashEpisodeSettledTicks;
    private long crashEpisodeStartGameTime = Long.MIN_VALUE;
    private long crashEpisodeLastImpactGameTime = Long.MIN_VALUE;
    private double crashEpisodePeakEnergyJ;
    private double crashEpisodePeakImpulseNs;
    private final Map<PartGroundDevice, Long> landingGearGameplayLastDamageTick =
        new IdentityHashMap<>();

    // A support impulse is not a new impact. Arm each device only after the live
    // probe observes separation beyond its existing proximity band.
    private final Map<PartGroundDevice, Boolean> landingGearImpactArmed = new IdentityHashMap<>();

    public SableVehicleBody(
        EntityVehicleF_Physics vehicle,
        ServerLevel level,
        AirframeLoads.SolveResult initialResult
    ) {
        this(vehicle, level, initialResult, null, true, 0.0);
    }

    public SableVehicleBody(
        EntityVehicleF_Physics vehicle,
        ServerLevel level,
        AirframeLoads.SolveResult initialResult,
        @Nullable BodyState initialState,
        boolean initialGroundSeatingAllowed
    ) {
        this(vehicle, level, initialResult, initialState, initialGroundSeatingAllowed, 0.0);
    }

    SableVehicleBody(
        EntityVehicleF_Physics vehicle,
        ServerLevel level,
        AirframeLoads.SolveResult initialResult,
        @Nullable BodyState initialState,
        boolean initialGroundSeatingAllowed,
        double retainedLoadingRadius
    ) {
        super(
            initialPose(vehicle, initialState, initialResult.centerOfMassLocal()),
            new Vector3d(ANCHOR_HALF_EXTENT, ANCHOR_HALF_EXTENT, ANCHOR_HALF_EXTENT),
            0.0
        );
        this.vehicle = vehicle;
        this.level = level;
        this.mass = Math.max(1.0, initialResult.mass());
        this.desiredInertia = sanitizeInertia(initialResult.inertia());
        this.centerOfMassLocal = initialResult.centerOfMassLocal() == null
            || !initialResult.centerOfMassLocal().isFinite()
                ? Vec3d.ZERO : initialResult.centerOfMassLocal();
        this.massData = new AircraftMassData(this.mass, this.desiredInertia);
        this.modelHalfExtents = modelHalfExtents(initialResult);
        this.modelLocation = initialResult.model() == null
            ? "unknown"
            : initialResult.model().modelLocation();
        this.initialState = initialState;
        this.initialGroundSeatingAllowed = initialGroundSeatingAllowed;
        this.retainedLoadingRadius = finitePositiveRadius(retainedLoadingRadius);
        this.cachedOrientation.set(SablePoseConversions.toQuaternion(vehicle.orientation));
        if (initialState != null) {
            this.lastExportedLinearVelocityWorld.set(initialState.lastExportedLinearVelocityWorld());
            this.lastExportedAngularVelocityBody.set(initialState.lastExportedAngularVelocityBody());
            this.hasExportedState = initialState.hasExportedState();
        }
    }

    @Override
    public MassData getMassTracker() {
        return massData;
    }

    @Override
    public void onAddition(SubLevelPhysicsSystem physicsSystem) {
        super.onAddition(physicsSystem);
        this.physicsSystem = physicsSystem;
        this.rigidBodyHandle = new RigidBodyHandle(this, physicsSystem);
        this.sceneHandle = Rapier3DInvoker.pmweatherIv$getSceneHandle(level);
        cachedOrientation.set(SablePoseConversions.toQuaternion(vehicle.orientation));
        try {
            applyExactMassProperties();
            this.compoundCollider = new SableCompoundCollider(
                vehicle, sceneHandle, getRuntimeId(), centerOfMassLocal
            );
            this.compoundCollider.initialize();
            retainCurrentLoadingRadius();
            if (PMIVObserver.loggingEnabled()) logGroundDeviceInventory();
            if (initialState != null
                && finite(initialState.linearVelocityWorld())
                && finite(initialState.angularVelocityWorld())) {
                Quaterniond placementOrientation = initialState.orientationWorld() != null
                    && finite(initialState.orientationWorld())
                        ? new Quaterniond(initialState.orientationWorld()).normalize()
                        : new Quaterniond(cachedOrientation);
                Vector3d placementModelOrigin = initialState.positionWorld() != null
                    && finite(initialState.positionWorld())
                        ? modelOriginFromCenterOfMassWorld(
                            initialState.positionWorld(), placementOrientation
                        )
                        : new Vector3d(vehicle.position.x, vehicle.position.y, vehicle.position.z);
                InitialPlacementPose placementPose = initialGroundSeatedPose(
                    placementModelOrigin, placementOrientation,
                    initialState.linearVelocityWorld(), initialState.angularVelocityWorld()
                );
                Vector3dc initialLinear = placementPose.terrainClearFallbackApplied()
                    ? new Vector3d() : initialState.linearVelocityWorld();
                Vector3dc initialAngular = placementPose.terrainClearFallbackApplied()
                    ? new Vector3d() : initialState.angularVelocityWorld();
                placeBodyAtModelOriginWithVelocity(
                    initialLinear, initialAngular,
                    placementPose.orientationWorld(),
                    placementPose.positionWorld()
                );
            } else {
                InitialPlacementPose placementPose = initialGroundSeatedPose(
                    new Vector3d(vehicle.position.x, vehicle.position.y, vehicle.position.z),
                    new Quaterniond(cachedOrientation), new Vector3d(), new Vector3d()
                );
                placeBodyAtModelOriginWithVelocity(
                    new Vector3d(), new Vector3d(),
                    placementPose.orientationWorld(), placementPose.positionWorld()
                );
            }

            // Establish the persistent terrain-safe anchor as soon as the full
            // compound exists. Every later Sable substep either advances this
            // anchor to another verified-clear pose or is restored against it.
            // Without this initialization, a first missed collision could make an
            // already-embedded substep pose the next recovery origin.
            updatePose();
            Vector3d initialTerrainPosition = new Vector3d(getPose().position());
            Quaterniond initialTerrainOrientation = new Quaterniond(
                getPose().orientation()
            ).normalize();
            if (!compoundCollider.bodyIntersectsTerrainAtPose(
                level, initialTerrainPosition, initialTerrainOrientation
            )) {
                terrainResponse.rememberClearTerrainPose(
                    initialTerrainPosition, initialTerrainOrientation
                );
            }
        } catch (RuntimeException | Error failure) {
            try {
                if (compoundCollider != null) {
                    compoundCollider.remove();
                }
            } finally {
                compoundCollider = null;
                try {
                    super.onRemoved();
                } catch (RuntimeException ignored) {
                    // Preserve the original compound-body creation failure.
                }
                rigidBodyHandle = null;
                this.physicsSystem = null;
            }
            throw failure;
        }
        if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
            "SABLE_PERSISTENT_FLIGHT_BODY_CREATED uuid=" + vehicle.uniqueUUID
                + " mass=" + mass
                + " inertia=" + desiredInertia
                + " model=" + safe(modelLocation)
                + " centerOfMassLocal=(" + centerOfMassLocal.x() + ","
                    + centerOfMassLocal.y() + "," + centerOfMassLocal.z() + ")"
                + " parentOrigin=PHYSICAL_CENTER_OF_MASS"
                + " modelHalfExtents=" + modelHalfExtents
                + " mode=world-space-single-rigid-body"
                + " colliderInteraction=mounted-level-collider-compound"
                + " compoundPhysicalBoxes=" + compoundCollider.physicalBoxCount()
                + " compoundWheelColliders=0"
                + " compoundChildren=" + compoundCollider.mountedChildCount()
                + " ivBodyCollisionAuthority=false"
                + " sableCollisionAuthority=true"
                + " sableFlightAndGearMomentumAuthority=true"
                + " landingGearAuthority=SABLE_LIVE_TERRAIN_POINT_CONSTRAINT"
                + " groundDevicePhysicalColliders=false"
                + " gyroMode=RAPIER_NATIVE_GYRO_PMIV_EXTERNAL_TORQUE_ONLY"
                + " collisionDeltaFeedback=false"
                + " absolutePoseAuthority=SABLE_RAPIER"
        );
    }

    private void logGroundDeviceInventory() {
        if (!PMIVObserver.loggingEnabled()) return;
        int total = 0;
        int valid = 0;
        int active = 0;
        int spare = 0;
        int fake = 0;
        int wheel = 0;
        int tread = 0;
        int floating = 0;
        int solidSupport = 0;
        List<PartGroundDevice> devices = new ArrayList<>();
        if (vehicle.allParts != null) {
            for (APart part : vehicle.allParts) {
                if (!(part instanceof PartGroundDevice device)) {
                    continue;
                }
                devices.add(device);
                ++total;
                if (device.isValid) ++valid;
                if (device.isActiveVar.isActive) ++active;
                if (device.isSpare) ++spare;
                if (device.isFake()) ++fake;
                boolean isWheel = device.definition != null
                    && device.definition.ground != null
                    && device.definition.ground.isWheel;
                boolean isTread = device.definition != null
                    && device.definition.ground != null
                    && device.definition.ground.isTread;
                boolean canFloat = device.definition != null
                    && device.definition.ground != null
                    && device.definition.ground.canFloat;
                if (isWheel) ++wheel;
                if (isTread) ++tread;
                if (canFloat) ++floating;
                if (!isWheel && !isTread) ++solidSupport;
            }
        }
        int staticPoints = compoundCollider == null
            ? 0 : compoundCollider.staticGroundSupportPoints().size();
        if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
            "SABLE_GROUND_DEVICE_INVENTORY uuid=" + vehicle.uniqueUUID
                + " total=" + total
                + " valid=" + valid
                + " active=" + active
                + " spare=" + spare
                + " fake=" + fake
                + " wheel=" + wheel
                + " tread=" + tread
                + " float=" + floating
                + " solidSupport=" + solidSupport
                + " staticModelSupportPointCount=" + staticPoints
        );
        int index = 0;
        for (PartGroundDevice device : devices) {
            boolean isWheel = device.definition != null
                && device.definition.ground != null
                && device.definition.ground.isWheel;
            boolean isTread = device.definition != null
                && device.definition.ground != null
                && device.definition.ground.isTread;
            boolean canFloat = device.definition != null
                && device.definition.ground != null
                && device.definition.ground.canFloat;
            double placementLongOffset = device.placementDefinition == null
                ? 0.0 : device.placementDefinition.extraCollisionBoxOffset;
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_GROUND_DEVICE uuid=" + vehicle.uniqueUUID
                    + " index=" + index++
                    + " fake=" + device.isFake()
                    + " valid=" + device.isValid
                    + " active=" + device.isActiveVar.isActive
                    + " spare=" + device.isSpare
                    + " wheel=" + isWheel
                    + " tread=" + isTread
                    + " float=" + canFloat
                    + " width=" + device.getWidth()
                    + " height=" + device.getHeight()
                    + " longPartOffset=" + device.getLongPartOffset()
                    + " placementLongOffset=" + placementLongOffset
                    + " localOffset=" + device.localOffset
                    + " wheelbasePoint=" + device.wheelbasePoint
            );
        }
    }

    @Override
    public void onRemoved() {
        if (compoundCollider != null) {
            compoundCollider.remove();
            compoundCollider = null;
        }
        // onAddition() may have already removed the anchor while unwinding a
        // compound-creation failure. Avoid calling BoxPhysicsObject.remove()
        // twice on its now-null handle when SubLevelPhysicsSystem then removes
        // the failed Java object from its arbitrary-object identity set.
        if (isActive()) {
            super.onRemoved();
        }
        this.rigidBodyHandle = null;
        this.physicsSystem = null;
        this.sceneHandle = 0L;
    }

    /**
     * Sable removes arbitrary objects as soon as any part of their loading
     * footprint reaches an unloaded chunk. Its BoxPhysicsObject callback only
     * removes the anchor body; PMWeather-IV must remove every separately mounted
     * compound child in the same callback or repeated retries leave native
     * LevelColliders behind until later manager cleanup.
     */
    @Override
    public void onUnloaded(SubLevelHoldingChunkMap holdingChunkMap, ChunkPos chunkPos) {
        int mountedChildren = compoundCollider == null ? 0 : compoundCollider.mountedChildCount();
        int physicalBoxes = compoundCollider == null ? 0 : compoundCollider.physicalBoxCount();
        if (compoundCollider != null) {
            compoundCollider.remove();
            compoundCollider = null;
        }
        super.onUnloaded(holdingChunkMap, chunkPos);
        rigidBodyHandle = null;
        if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
            "SABLE_FLIGHT_BODY_UNLOADED uuid=" + vehicle.uniqueUUID
                + " chunk=" + chunkPos
                + " mountedChildrenRemoved=" + mountedChildren
                + " physicalBoxesRemoved=" + physicalBoxes
                + " anchorRemoved=true"
                + " nativeCompoundLeak=false"
                + " recreationRequiresLoadedFootprint=true"
        );
    }

    /** Keep terrain synchronized around the real world-space Sable aircraft body. */
    @Override
    public void getBoundingBox(BoundingBox3d destination) {
        Vector3dc physicalPosition = getPose().position();
        double x = finite(physicalPosition) ? physicalPosition.x() : vehicle.position.x;
        double y = finite(physicalPosition) ? physicalPosition.y() : vehicle.position.y;
        double z = finite(physicalPosition) ? physicalPosition.z() : vehicle.position.z;
        destination.set(x, y, z, x, y, z);
        // Use the largest observed rotation-independent sphere. A measured compound
        // radius remains available if a chunk callback has already removed it.
        destination.expand(loadingRadius());
    }

    public EntityVehicleF_Physics vehicle() {
        return vehicle;
    }

    public double roadSuspensionTravel(PartGroundDevice device) {
        PartGroundDevice master = RoadSuspensionModel.masterPart(device);
        Double travel = master == null ? null : roadSuspensionTravel.get(master);
        return travel != null && Double.isFinite(travel) ? travel : 0.0;
    }

    public Map<UUID, Double> roadSuspensionOffsets() {
        Map<UUID, Double> result = new java.util.HashMap<>();
        roadSuspensionTravel.forEach((device, travel) -> {
            if (device != null && !(device instanceof minecrafttransportsimulator.entities.instances.PartGroundDeviceFake)
                && device.uniqueUUID != null && travel != null && Double.isFinite(travel)) {
                result.put(device.uniqueUUID, travel);
            }
        });
        return Map.copyOf(result);
    }

    public boolean markRoadSuspensionPublished(long tick) {
        if (tick == lastRoadSuspensionPublishTick) return false;
        lastRoadSuspensionPublishTick = tick;
        return true;
    }

    public ServerLevel level() {
        return level;
    }

    public SubLevelPhysicsSystem physicsSystem() {
        return physicsSystem;
    }

    /** Scene handle used only for PMWeather-IV read-only collision telemetry cleanup. */
    public long sceneHandle() {
        return sceneHandle;
    }

    public boolean isUsable() {
        // Rigid-body sleep is not removal. BoxPhysicsObject.isActive() may be
        // false for a sleeping proxy, while the registered physics system and
        // Rapier handle remain valid and can be woken by the next impulse.
        return !poisoned
            && vehicle.isValid
            && physicsSystem != null
            && rigidBodyHandle != null
            && rigidBodyHandle.isValid();
    }

    /** True while this Java object still belongs to the same live IV vehicle. */
    public boolean isRegisteredFor(EntityVehicleF_Physics candidate, ServerLevel candidateLevel) {
        return candidate == vehicle && candidateLevel == level && vehicle.isValid;
    }

    public double physicsMass() {
        return mass;
    }

    /** Rotation-independent radius used by Sable's chunk-loading preflight. */
    public double loadingRadius() {
        double compoundRadius = compoundCollider == null
            ? 0.0
            : compoundCollider.collisionBoundingRadius();
        double modelRadiusAboutCenterOfMass = modelHalfExtents.length()
            + Math.sqrt(centerOfMassLocal.lengthSquared());
        retainedLoadingRadius = conservativeLoadingRadius(
            modelRadiusAboutCenterOfMass, compoundRadius, retainedLoadingRadius
        );
        return retainedLoadingRadius;
    }

    /** Shared numeric policy for the live body, retained preflight cache, and lifecycle regression. */
    public static double conservativeLoadingRadius(
        double modelRadius,
        double liveCompoundRadius,
        double retainedRadius
    ) {
        return Math.max(2.0, Math.max(
            finitePositiveRadius(modelRadius),
            Math.max(finitePositiveRadius(liveCompoundRadius), finitePositiveRadius(retainedRadius))
        ));
    }

    private static double finitePositiveRadius(double radius) {
        return Double.isFinite(radius) && radius > 0.0 ? radius : 0.0;
    }

    private void retainCurrentLoadingRadius() {
        // loadingRadius() folds the current collider and model into the monotonic
        // value, which remains intact when onUnloaded() clears the collider.
        loadingRadius();
    }

    public boolean hasCompoundCollisionAuthority() {
        return isUsable() && compoundCollider != null && compoundCollider.isActive();
    }

    public void refreshCompoundCollider() {
        if (isUsable() && compoundCollider != null) {
            long start = PMIVObserver.isCapturing(vehicle.uniqueUUID) ? System.nanoTime() : 0;
            try {
                compoundCollider.refresh();
                retainCurrentLoadingRadius();
            } catch (RuntimeException | LinkageError failure) {
                quarantineIntegrationFailure("compoundRefresh", failure);
            } finally {
                if (start != 0) PMIVObserver.capturePerformance(vehicle.uniqueUUID,
                    "colliderRefresh", System.nanoTime() - start);
            }
        }
    }

    /** Translate each completed Sable terrain-contact report once into IV gameplay consequences. */
    public void processCollisionGameplay() {
        if (!isUsable() || compoundCollider == null || sceneHandle == 0L) {
            return;
        }
        SableCollisionCapture.SceneSnapshot snapshot = SableCollisionCapture.snapshot(sceneHandle);
        boolean newCollisionSnapshot = snapshot.generation() != 0L
            && snapshot.generation() != lastProcessedCollisionGeneration;
        // A swept fallback can be the *only* evidence of a terrain strike when
        // Rapier tunnels a thin BODY cuboid and therefore emits no contact row.
        // Drain it independently of collision-snapshot generation so gameplay
        // still sees that physical impact on the next IV vehicle tick.
        List<SweptTerrainImpact> sweptImpacts = terrainResponse.drainSweptTerrainImpacts();
        if (!newCollisionSnapshot && sweptImpacts.isEmpty()) {
            return;
        }
        if (newCollisionSnapshot) {
            lastProcessedCollisionGeneration = snapshot.generation();
        }
        updatePose();
        Vector3d position = new Vector3d(getPose().position());
        Quaterniond orientation = new Quaterniond(getPose().orientation()).normalize();
        if (!finite(position) || !finite(orientation)) {
            return;
        }
        if (newCollisionSnapshot && !snapshot.contacts().isEmpty()
            && PMIVObserver.isCapturing(vehicle.uniqueUUID)) {
            PMIVObserver.captureCollisions(
                this,
                snapshot,
                compoundCollider.gameplayContacts(snapshot, position, orientation)
            );
        }
        SableCollisionGameplay.process(
            this, snapshot, position, orientation, sweptImpacts, newCollisionSnapshot
        );
    }

    AirframeLoads.SolveResult lastTickResult() {
        return lastTickResult;
    }

    /** Per-aircraft contact evidence from Sable's completed tick, never scene-wide foreign contacts. */
    public boolean hasRecentTerrainContact() {
        if (lastSubstepLandingGear != null && lastSubstepLandingGear.contactCount() > 0) return true;
        return isUsable() && compoundCollider != null && sceneHandle != 0L
            && compoundCollider.hasExternalContact(SableCollisionCapture.snapshot(sceneHandle));
    }

    SableCompoundCollider compoundColliderForGameplay() {
        return compoundCollider;
    }

    boolean blockBreakGameplayReady() {
        return ((EntityVehicleMovingAccessor) vehicle).pmweatherIv$getBlockBreakDelay() == 0;
    }

    boolean crashGameplayReady() {
        return ((EntityVehicleMovingAccessor) vehicle).pmweatherIv$getCrashDebounce() == 0;
    }

    void markCrashGameplay() {
        ((EntityVehicleMovingAccessor) vehicle).pmweatherIv$setCrashDebounce(60);
    }

    boolean landingGearGameplayReady(@Nullable PartGroundDevice device) {
        if (device == null) {
            return false;
        }
        Long previous = landingGearGameplayLastDamageTick.get(device);
        return previous == null
            || level.getGameTime() - previous >= LANDING_GEAR_GAMEPLAY_DEBOUNCE_TICKS;
    }

    void markLandingGearGameplay(@Nullable PartGroundDevice device) {
        if (device != null) {
            landingGearGameplayLastDamageTick.put(device, level.getGameTime());
        }
    }

    /** Called once per owner tick before prior-Sable-tick collision gameplay is consumed. */
    void advanceCrashEpisodeLifecycle() {
        if (!crashEpisodeActive || wreckTerrainDamageClosed || !isUsable()) {
            return;
        }
        GameplayKinematics kinematics = gameplayKinematics();
        if (kinematics == null) {
            return;
        }
        double radius = compoundCollider == null ? 0.0 : Math.max(0.0, compoundCollider.collisionBoundingRadius());
        double pointSpeed = kinematics.linearVelocityWorld().length()
            + kinematics.angularVelocityWorld().length() * radius;
        long now = level.getGameTime();
        boolean impactThisTick = crashEpisodeLastImpactGameTime == now;
        if (!vehicle.outOfHealth) {
            // Healthy contact episodes are only contact-continuity bookkeeping.
            // Clear them after a short quiet interval even if the aircraft flies
            // away at high speed, so an unrelated later crash starts fresh.
            crashEpisodeSettledTicks = impactThisTick ? 0 : crashEpisodeSettledTicks + 1;
        } else if (!impactThisTick && Double.isFinite(pointSpeed)
            && pointSpeed < CRASH_EPISODE_SETTLED_POINT_SPEED_MPS) {
            ++crashEpisodeSettledTicks;
        } else {
            crashEpisodeSettledTicks = 0;
        }

        if (crashEpisodeSettledTicks < CRASH_EPISODE_SETTLED_TICKS) {
            return;
        }

        if (vehicle.outOfHealth) {
            wreckTerrainDamageClosed = true;
            crashEpisodeActive = false;
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_WRECK_TERRAIN_DAMAGE_CLOSED gameTime=" + now
                    + " uuid=" + vehicle.uniqueUUID
                    + " settledPointSpeedMps=" + pointSpeed
                    + " settledTicks=" + crashEpisodeSettledTicks
                    + " episodeStartGameTime=" + crashEpisodeStartGameTime
                    + " lastImpactGameTime=" + crashEpisodeLastImpactGameTime
                    + " peakImpactEnergyJ=" + crashEpisodePeakEnergyJ
                    + " peakImpulseNs=" + crashEpisodePeakImpulseNs
                    + " reopenPolicy=NEVER_FOR_THIS_WRECK"
                    + " motionAuthority=SABLE_UNCHANGED"
            );
        } else {
            // A healthy aircraft that merely scraped/touched terrain has no need
            // to carry episode state forever.  A later independent crash may start
            // a fresh episode normally.
            crashEpisodeActive = false;
            crashEpisodeSettledTicks = 0;
            crashEpisodeStartGameTime = Long.MIN_VALUE;
            crashEpisodeLastImpactGameTime = Long.MIN_VALUE;
            crashEpisodePeakEnergyJ = 0.0;
            crashEpisodePeakImpulseNs = 0.0;
        }
    }

    /**
     * Records physically solved contact work.  A wreck that was already closed
     * cannot start a new episode; the fatal impact starts while the aircraft is
     * still healthy, so immediate follow-on wreck contacts remain part of the same
     * physical crash.
     */
    void noteCrashEpisodeImpact(double energyJ, double impulseNs) {
        if (!Double.isFinite(energyJ) || energyJ <= 0.0
            || !Double.isFinite(impulseNs) || impulseNs <= 0.0) {
            return;
        }
        if (vehicle.outOfHealth && (!crashEpisodeActive || wreckTerrainDamageClosed)) {
            return;
        }
        long now = level.getGameTime();
        if (!crashEpisodeActive) {
            crashEpisodeActive = true;
            crashEpisodeStartGameTime = now;
            crashEpisodePeakEnergyJ = 0.0;
            crashEpisodePeakImpulseNs = 0.0;
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_CRASH_EPISODE_STARTED gameTime=" + now
                    + " uuid=" + vehicle.uniqueUUID
                    + " outOfHealthAtStart=" + vehicle.outOfHealth
                    + " policy=PHYSICAL_IMPACT_EPISODE"
            );
        }
        crashEpisodeLastImpactGameTime = now;
        crashEpisodeSettledTicks = 0;
        crashEpisodePeakEnergyJ = Math.max(crashEpisodePeakEnergyJ, energyJ);
        crashEpisodePeakImpulseNs = Math.max(crashEpisodePeakImpulseNs, impulseNs);
    }

    boolean terrainDamageAllowedForCurrentCrashEpisode() {
        if (!vehicle.outOfHealth) {
            return true;
        }
        return crashEpisodeActive && !wreckTerrainDamageClosed;
    }

    String crashEpisodeState() {
        if (wreckTerrainDamageClosed) {
            return "WRECK_TERRAIN_DAMAGE_CLOSED";
        }
        if (vehicle.outOfHealth && crashEpisodeActive) {
            return "WRECK_CONTINUATION";
        }
        if (crashEpisodeActive) {
            return "CRASH_EPISODE_ACTIVE";
        }
        return "LIVE";
    }

    int crashEpisodeSettledTicks() {
        return crashEpisodeSettledTicks;
    }


    public boolean needsMassRefresh(AirframeLoads.SolveResult result) {
        if (relativeDifference(mass, result.mass()) > 0.001) {
            return true;
        }
        Vec3d next = sanitizeInertia(result.inertia());
        return relativeDifference(desiredInertia.x(), next.x()) > 0.001
            || relativeDifference(desiredInertia.y(), next.y()) > 0.001
            || relativeDifference(desiredInertia.z(), next.z()) > 0.001;
    }

    /**
     * Updates Rapier mass/inertia on the existing aircraft body.
     *
     * <p>Passenger, fuel and cargo changes must not destroy/recreate the rigid
     * body merely to change mass properties. Sable 2.0.3's native
     * setMassProperties updates the existing Rapier RigidBody in place, so the
     * runtime body ID, mounted compound colliders, contact manifolds, pose, and
     * linear/angular velocities remain continuous.</p>
     */
    public void refreshMassProperties(AirframeLoads.SolveResult result) {
        if (!isUsable() || result == null) {
            return;
        }
        double nextMass = Math.max(1.0, result.mass());
        Vec3d nextInertia = sanitizeInertia(result.inertia());
        if (relativeDifference(mass, nextMass) <= 0.001
            && relativeDifference(desiredInertia.x(), nextInertia.x()) <= 0.001
            && relativeDifference(desiredInertia.y(), nextInertia.y()) <= 0.001
            && relativeDifference(desiredInertia.z(), nextInertia.z()) <= 0.001) {
            return;
        }

        double oldMass = mass;
        Vec3d oldInertia = desiredInertia;
        AircraftMassData oldMassData = massData;

        mass = nextMass;
        desiredInertia = nextInertia;
        massData = new AircraftMassData(nextMass, nextInertia);
        try {
            applyExactMassProperties();
        } catch (RuntimeException | Error failure) {
            // Keep Java/native mass descriptions coherent if the private Sable
            // bridge ever rejects an update. The existing body remains the
            // authority; never replace it as a fallback.
            mass = oldMass;
            desiredInertia = oldInertia;
            massData = oldMassData;
            try {
                applyExactMassProperties();
            } catch (RuntimeException | Error ignored) {
                // Preserve the original update failure for diagnostics.
            }
            throw failure;
        }

        if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
            "SABLE_MASS_PROPERTIES_UPDATED uuid=" + vehicle.uniqueUUID
                + " bodyId=" + getRuntimeId()
                + " oldMass=" + oldMass
                + " newMass=" + mass
                + " oldInertia=" + oldInertia
                + " newInertia=" + desiredInertia
                + " mountedChildren=" + (compoundCollider == null ? 0 : compoundCollider.mountedChildCount())
                + " physicalBoxes=" + (compoundCollider == null ? 0 : compoundCollider.physicalBoxCount())
                + " wheelColliders=0"
                + " landingGearAuthority=SABLE_LIVE_TERRAIN_POINT_CONSTRAINT"
                + " bodyRecreated=false"
                + " colliderIdsPreserved=true"
                + " posePreserved=true"
                + " linearAngularVelocityPreserved=true"
                + " collisionAuthority=SABLE_RAPIER_COMPOUND"
        );
    }

    /**
     * Mirrors Sable's absolute world pose into the IV movement request for this
     * tick. IV no longer feeds accepted-minus-requested collision corrections
     * back into Sable: Rapier has already resolved the physical collision.
     */
    public void syncIvRequestFromSablePose(AircraftState state) {
        if (!isUsable()) {
            return;
        }

        updatePose();
        Vector3d targetPosition = new Vector3d(getPose().position());
        Quaterniond targetOrientation = new Quaterniond(getPose().orientation()).normalize();
        rigidBodyHandle.getLinearVelocity(temporaryLinearVelocityWorld);
        rigidBodyHandle.getAngularVelocity(temporaryAngularVelocityWorld);
        if (!finite(targetPosition)
            || !finite(targetOrientation)
            || !finite(temporaryLinearVelocityWorld)
            || !finite(temporaryAngularVelocityWorld)) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "ERROR stage=sableAbsolutePoseRead reason=nonFinite uuid=" + vehicle.uniqueUUID
            );
            quarantineNonFiniteBody("sableAbsolutePoseRead");
            hasExportedState = false;
            return;
        }

        // Small mounted LevelCollider wheel shapes repeatedly lost terrain
        // contact during ordinary low-speed rolls. The authoritative substep
        // path therefore keeps IV's authored ground-device stations/properties
        // but probes terrain from the live Sable pose. IV's own 20 Hz ground
        // sensors remain useful for animation/metadata only.
        SablePoseConversions.worldToLocal(
            targetOrientation,
            temporaryAngularVelocityWorld,
            temporaryAngularVelocityBody
        );
        SablePoseConversions.writeRotationMatrix(
            targetOrientation, temporaryPhysicalOrientation
        );
        Vector3d targetModelOrigin = modelOriginFromCenterOfMassWorld(
            targetPosition, targetOrientation
        );
        Vector3d targetModelOriginVelocity = modelOriginVelocityFromCenterOfMassVelocity(
            temporaryLinearVelocityWorld, temporaryAngularVelocityWorld, targetOrientation
        );
        double liveConstraintMass = Math.max(1.0, vehicle.currentMass);
        double liveMassRatio = liveConstraintMass / Math.max(1.0, mass);
        Vec3d liveConstraintInertia = desiredInertia.scale(liveMassRatio);

        // Physical landing-gear impulses are applied in prepareSubstep() from
        // the live Sable terrain probe. Export/trace the last result that was
        // actually applied; do not perform a second hypothetical IV-sensor solve.
        LandingGearSolver.LandingGearConstraintResult landingGearConstraint =
            lastSubstepLandingGear != null
                ? lastSubstepLandingGear
                : LandingGearSolver.solveLandingGearConstraints(
                    vehicle,
                    new Vec3d(
                        temporaryLinearVelocityWorld.x,
                        temporaryLinearVelocityWorld.y,
                        temporaryLinearVelocityWorld.z
                    ),
                    new Vec3d(
                        temporaryAngularVelocityBody.x,
                        temporaryAngularVelocityBody.y,
                        temporaryAngularVelocityBody.z
                    ),
                    liveConstraintMass,
                    liveConstraintInertia,
                    temporaryPhysicalOrientation,
                    FlightMath.MC_TICK_SECONDS,
                    List.of(),
                    centerOfMassLocal
                );
        temporaryLinearCorrectionWorld.zero();
        temporaryAngularCorrectionBody.zero();

        // Terrain crossing is now checked after every completed Sable substep,
        // before another substep can integrate an already-embedded compound.
        // The IV export path only consumes the already-resolved Sable pose.
        SableCollisionCapture.SceneSnapshot collisionSnapshot =
            SableCollisionCapture.snapshot(sceneHandle);

        // IV applies position delta = motion * speedFactor once per vehicle tick.
        double speedFactor = Math.max(1.0E-6, vehicle.speedFactor);
        double deltaX = targetModelOrigin.x - vehicle.position.x;
        double deltaY = targetModelOrigin.y - vehicle.position.y;
        double deltaZ = targetModelOrigin.z - vehicle.position.z;
        vehicle.motion.set(deltaX / speedFactor, deltaY / speedFactor, deltaZ / speedFactor);

        // IV post-multiplies orientation by rotationApplied. Therefore the
        // required local delta is inverse(current) * target.
        Quaterniond currentOrientation = SablePoseConversions.toQuaternion(vehicle.orientation);
        Quaterniond deltaOrientation = new Quaterniond(currentOrientation)
            .conjugate()
            .mul(targetOrientation)
            .normalize();
        RotationMatrix deltaRotation = new RotationMatrix();
        SablePoseConversions.writeRotationMatrix(deltaOrientation, deltaRotation);
        vehicle.rotation.angles.set(deltaRotation.angles).clamp180();

        state.angularVelocityBody = new Vec3d(
            temporaryAngularVelocityBody.x,
            temporaryAngularVelocityBody.y,
            temporaryAngularVelocityBody.z
        );
        Vec3d physicalVelocity = new Vec3d(
            targetModelOriginVelocity.x,
            targetModelOriginVelocity.y,
            targetModelOriginVelocity.z
        );
        state.kinematics = new com.g9third.pmweatheriv.physics.AircraftKinematics(
            toVec3d(targetModelOrigin), temporaryPhysicalOrientation,
            toVec3d(temporaryLinearVelocityWorld), state.angularVelocityBody, level.getGameTime());
        state.originVelocityWorld = physicalVelocity;
        state.hasPhysicalVelocity = true;
        state.heldForSable = false;

        lastExportedLinearVelocityWorld.set(temporaryLinearVelocityWorld);
        lastExportedAngularVelocityBody.set(temporaryAngularVelocityBody);
        hasExportedState = true;
        cachedOrientation.set(targetOrientation);

        PMIVObserver.captureLandingGear(
            this,
            targetModelOrigin,
            targetOrientation,
            targetModelOriginVelocity,
            temporaryAngularVelocityBody,
            landingGearConstraint,
            collisionSnapshot
        );

        // Per-tick authoritative pose and complete landing-gear detail are retained
        // in PMIVTrace; routine text duplication is intentionally omitted.
    }

    /** Queues this game tick's external loads for persistent Sable integration. */
    public void setLoads(
        AirframeLoads.SolveResult result,
        long gameTime,
        PMWeatherIVConfig.Values config,
        AircraftState state
    ) {
        // Sable supplies gravity continuously. The custom solver result contains
        // explicit gravity for owner-tick load diagnostics, so remove that
        // term before applying external force to the persistent body.
        Vec3d externalForce = result.forceWorld().add(new Vec3d(0.0, result.mass() * GRAVITY, 0.0));
        DimensionPhysicsData.getGravity(level, getPose().position(), sableGravityWorld);
        // Sable's default dimension gravity is 11 m/s^2, while the aircraft
        // model and diagnostics use standard 9.80665 m/s^2 gravity. Add
        // the exact difference as an external force so persistent server
        // integration matches the model without assuming Sable's datapack value.
        gravityCompensationForceWorld.set(
            -mass * sableGravityWorld.x,
            mass * (-GRAVITY - sableGravityWorld.y),
            -mass * sableGravityWorld.z
        );
        cachedExternalForceWorld.set(externalForce.x(), externalForce.y(), externalForce.z())
            .add(gravityCompensationForceWorld);
        double speedFactor = Math.max(1.0e-9, Math.abs(vehicle.speedFactor));
        double nativeVerticalMotion = state != null && state.hasPhysicalVelocity
            && state.originVelocityWorld != null && state.originVelocityWorld.isFinite()
                ? state.originVelocityWorld.y() / (speedFactor * 20.0)
                : Double.isFinite(vehicle.motion.y) ? vehicle.motion.y : 0.0;
        NativeBallastModel.Loads ballast = NativeBallastModel.evaluate(
            vehicle.ballastVolumeVar.currentValue, vehicle.ballastVolumeVar.isActive, vehicle.outOfHealth,
            vehicle.ballastControlVar.currentValue, nativeVerticalMotion, vehicle.airDensity,
            vehicle.currentMass, mass, vehicle.gravityFactorVar.currentValue,
            vehicle.waterBallastFactorVar.isActive, vehicle.waterBallastFactorVar.currentValue,
            vehicle.world.isBlockLiquid(vehicle.position),
            NativeBallastModel.ivForceUnitNewtons(speedFactor)
        );
        cachedBallastForceWorldY = ballast.netWorldVerticalForceNewtons();
        cachedExternalForceWorld.add(0.0, ballast.netWorldVerticalForceNewtons(), 0.0);

        // Cache only externally generated aerodynamic/control torque. Sable's
        // Rapier dynamic body already enables gyroscopic forces natively, so PMIV
        // must not add/subtract Euler's w x (I*w) term on this authority path.
        cachedNetTorqueBody.set(
            result.torqueBody().x(),
            result.torqueBody().y(),
            result.torqueBody().z()
        );
        // IV propulsion magnitude/engine state is sampled once at 20 Hz, but
        // store its force in BODY space so the thrust axis follows the current
        // Sable orientation on every physics substep rather than remaining
        // frozen in last tick's world direction during a rapid rotation.
        updatePose();
        cachedOrientation.set(getPose().orientation()).normalize();
        temporaryPropulsionForceWorld.zero();
        cachedPropulsionForceBody.zero();
        cachedPropulsionTorqueBody.zero();
        for (PropulsionModel.PropulsionSnapshot load : result.propulsionLoads()) {
            if (load == null || load.forceWorldNewtons() == null
                || load.torqueBodyNewtonMeters() == null
                || !load.forceWorldNewtons().isFinite()
                || !load.torqueBodyNewtonMeters().isFinite()) {
                continue;
            }
            // Rotor actuator magnitude is frozen at the owner tick, but the
            // force itself is redistributed at live disc points every Sable
            // substep. Do not also cache that owner-tick rotor force here.
            if (load.type() != null && load.type().startsWith("ROTOR_")) {
                continue;
            }
            temporaryPropulsionForceWorld.add(
                load.forceWorldNewtons().x(),
                load.forceWorldNewtons().y(),
                load.forceWorldNewtons().z()
            );
            cachedPropulsionTorqueBody.add(
                load.torqueBodyNewtonMeters().x(),
                load.torqueBodyNewtonMeters().y(),
                load.torqueBodyNewtonMeters().z()
            );
        }
        SablePoseConversions.worldToLocal(
            cachedOrientation, temporaryPropulsionForceWorld, cachedPropulsionForceBody
        );
        lastTickResult = result;
        cachedConfig = config;
        cachedAircraftState = state;
        cachedWindFieldSnapshot = result.windFieldSnapshot();
        cachedRotorActuators = result.rotorActuators() == null
            ? List.of() : List.copyOf(result.rotorActuators());
        airVehicleSubstepAerodynamics = state != null
            && state.isPrepared()
            && !vehicle.definition.motorized.isBlimp
            && cachedWindFieldSnapshot != null;
        lastLoadsGameTime = gameTime;
    }

    /** Applies one force/torque impulse without resetting persistent momentum. */
    public void prepareSubstep(double timeStep) {
        if (!isUsable() || timeStep <= 0.0) {
            substepArmed = false;
            return;
        }

        if (riderReconnectHeld) {
            // Keep the aircraft at the exact pose captured when the stale IV rider
            // link was detected. Rapier still owns the registered body/colliders, but
            // normal aero/gear/terrain work is intentionally paused for this tiny
            // reconnect window. Resetting here also removes any prior-substep contact
            // velocity before the shared scene integrates again.
            placeBodyWithVelocity(
                new Vector3d(),
                new Vector3d(),
                new Quaterniond(riderReconnectHoldOrientationWorld),
                new Vector3d(riderReconnectHoldPositionWorld)
            );
            substepArmed = true;
            return;
        }

        // The body remains in world space between ticks. Never reseed or
        // re-anchor it from IV: that would discard Rapier collision response and
        // reintroduce the old ground/air authority discontinuity.

        updatePose();
        if (!finite(getPose().position()) || !finite(getPose().orientation())) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "ERROR stage=persistentFlightSubstepStart reason=nonFinite uuid=" + vehicle.uniqueUUID
            );
            quarantineNonFiniteBody("persistentFlightSubstepStart");
            substepArmed = false;
            return;
        }
        cachedOrientation.set(getPose().orientation()).normalize();
        terrainResponse.substepStartPositionWorld.set(getPose().position());
        terrainResponse.substepStartOrientationWorld.set(cachedOrientation).normalize();
        terrainResponse.hasSubstepStartPose = finite(terrainResponse.substepStartPositionWorld)
            && finite(terrainResponse.substepStartOrientationWorld);
        rigidBodyHandle.getLinearVelocity(temporaryLinearVelocityWorld);
        rigidBodyHandle.getAngularVelocity(temporaryAngularVelocityWorld);
        boolean recentLoads = lastLoadsGameTime != Long.MIN_VALUE
            && level.getGameTime() - lastLoadsGameTime <= 2L;

        // Landing-gear contact is solved after this substep's external
        // force/torque impulse. That makes support and the Coulomb friction cap
        // use the current substep load rather than a pre-load velocity snapshot.
        // Contact state itself is probed from the live Sable pose below; IV's
        // mirrored 20 Hz ground-sensor flags are animation/metadata only.

        if (!recentLoads) {
            // Freeze vertical acceleration while the IV entity is not producing
            // ticks. Existing linear/angular momentum is retained for a clean
            // resume, but Sable gravity is cancelled during the stale interval.
            DimensionPhysicsData.getGravity(level, getPose().position(), sableGravityWorld);
            temporaryAppliedForceWorld.set(sableGravityWorld).mul(-mass);
            SablePoseConversions.worldToLocal(
                cachedOrientation,
                temporaryAppliedForceWorld,
                temporaryLocalForce
            );
            temporaryImpulse.set(temporaryLocalForce).mul(timeStep);
            rigidBodyHandle.applyLinearImpulse(temporaryImpulse);
            rigidBodyHandle.getLinearVelocity(temporaryLinearVelocityWorld);
            rigidBodyHandle.getAngularVelocity(temporaryAngularVelocityWorld);

            // Landing gear is terrain physics, not an owner-tick aerodynamic
            // load. Keep the real unilateral wheel constraints alive even when
            // the latest IV force sample is stale. Most importantly, the very
            // first Sable cycle now gives already-deployed gear the same chance
            // to establish normal support as BODY/Rapier instead of making that
            // cycle collision-only. No attitude/ride-height target is injected.
            applyLiveLandingGearConstraint(timeStep);
            rigidBodyHandle.getLinearVelocity(temporaryLinearVelocityWorld);
            rigidBodyHandle.getAngularVelocity(temporaryAngularVelocityWorld);

            // Retained momentum can tunnel even while the IV entity is between
            // fresh vehicle ticks. Keep the same pre-integration BODY boundary
            // authoritative on this stale-load path as well.
            terrainResponse.enforcePreIntegrationTerrainCcd(timeStep);
            SableCollisionCapture.capturePreSubstepMotion(
                sceneHandle,
                getRuntimeId(),
                new Vector3d(getPose().position()),
                new Quaterniond(cachedOrientation),
                new Vector3d(temporaryLinearVelocityWorld),
                new Vector3d(temporaryAngularVelocityWorld)
            );
            // No new aero loads, but the persistent body can still cross terrain
            // on retained momentum. Arm post-substep boundary validation anyway.
            substepArmed = true;
            return;
        }

        AirframeLoads.SubstepAerodynamicLoads appliedAerodynamics = null;
        temporaryAppliedForceWorld.set(cachedExternalForceWorld);
        temporaryNetTorqueBody.set(cachedNetTorqueBody);
        if (airVehicleSubstepAerodynamics && cachedConfig != null
            && cachedAircraftState != null && cachedWindFieldSnapshot != null) {
            SablePoseConversions.worldToLocal(
                cachedOrientation, temporaryAngularVelocityWorld, temporaryAngularVelocityBody
            );
            SablePoseConversions.writeRotationMatrix(
                cachedOrientation, temporaryPhysicalOrientation
            );
            AirframeLoads.SubstepAerodynamicLoads aerodynamic =
                calculateVehicleSubstepAerodynamics(
                    vehicle,
                    level,
                    cachedConfig,
                    cachedAircraftState,
                    toVec3d(modelOriginFromCenterOfMassWorld(
                        getPose().position(), cachedOrientation
                    )),
                    temporaryPhysicalOrientation,
                    new Vec3d(
                        temporaryLinearVelocityWorld.x,
                        temporaryLinearVelocityWorld.y,
                        temporaryLinearVelocityWorld.z
                    ),
                    new Vec3d(
                        temporaryAngularVelocityBody.x,
                        temporaryAngularVelocityBody.y,
                        temporaryAngularVelocityBody.z
                    ),
                    cachedWindFieldSnapshot,
                    cachedRotorActuators,
                    timeStep
                );
            appliedAerodynamics = aerodynamic;
            DimensionPhysicsData.getGravity(level, getPose().position(), sableGravityWorld);
            gravityCompensationForceWorld.set(
                -mass * sableGravityWorld.x,
                mass * (-GRAVITY - sableGravityWorld.y),
                -mass * sableGravityWorld.z
            );
            temporaryAppliedForceWorld.set(
                aerodynamic.forceWorld().x(),
                aerodynamic.forceWorld().y(),
                aerodynamic.forceWorld().z()
            );
            SablePoseConversions.localToWorld(
                cachedOrientation, cachedPropulsionForceBody, temporaryPropulsionForceWorld
            );
            temporaryAppliedForceWorld.add(temporaryPropulsionForceWorld)
                .add(gravityCompensationForceWorld);
            temporaryAppliedForceWorld.y += cachedBallastForceWorldY;
            temporaryNetTorqueBody.set(
                aerodynamic.torqueBody().x(),
                aerodynamic.torqueBody().y(),
                aerodynamic.torqueBody().z()
            ).add(cachedPropulsionTorqueBody);
        }

        if (!finite(temporaryAppliedForceWorld) || !finite(temporaryNetTorqueBody)) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "ERROR stage=persistentFlightLoads reason=nonFinite uuid=" + vehicle.uniqueUUID
            );
            quarantineNonFiniteBody("persistentFlightLoads");
            substepArmed = false;
            return;
        }

        // Rapier receives the complete weight/aerodynamic load and resolves
        // mounted BLOCK-body contacts. The live-terrain landing-gear constraint has
        // already been applied above at this same Sable substep cadence.
        SablePoseConversions.worldToLocal(
            cachedOrientation,
            temporaryAppliedForceWorld,
            temporaryLocalForce
        );
        temporaryImpulse.set(temporaryLocalForce).mul(timeStep);

        // Sable's Rapier body enables native gyroscopic forces.  PMIV must pass
        // only the externally generated aerodynamic/propulsion torque here;
        // subtracting w x (I*w) again would apply Euler gyroscopic coupling twice.
        temporaryTorqueImpulse.set(temporaryNetTorqueBody).mul(timeStep);
        if (temporaryImpulse.lengthSquared() > 1.0E-18
            || temporaryTorqueImpulse.lengthSquared() > 1.0E-18) {
            rigidBodyHandle.applyLinearAndAngularImpulse(
                temporaryImpulse,
                temporaryTorqueImpulse,
                true
            );
        }

        if (appliedAerodynamics != null) PMIVObserver.captureAppliedAerodynamics(
            this, timeStep, lastLoadsGameTime, temporaryAppliedForceWorld,
            temporaryNetTorqueBody, sableGravityWorld, appliedAerodynamics);

        // The force impulse above changes the wheel point velocities and the
        // instantaneous normal load. Re-read that post-force state, probe the
        // terrain at the current Sable wheel stations, then solve the unilateral
        // rolling constraint before Rapier integrates this substep.
        rigidBodyHandle.getLinearVelocity(temporaryLinearVelocityWorld);
        rigidBodyHandle.getAngularVelocity(temporaryAngularVelocityWorld);
        applyLiveLandingGearConstraint(timeStep);

        // Mounted Sable LevelColliders do not provide a usable swept-CCD switch.
        // Before Rapier integrates, conservatively sweep the complete frozen IV
        // BODY compound through the exact pose this substep would reach. If the
        // first solid Minecraft boundary would be crossed, constrain this
        // substep to the last clear fraction and queue that first hit for native
        // IV block/crash gameplay. The existing post-step recovery remains only
        // a diagnostic failsafe and should never need to repair a high-speed
        // tunnel visible to IV/render state.
        terrainResponse.enforcePreIntegrationTerrainCcd(timeStep);

        // Collision capture must observe the velocity Rapier actually receives:
        // external loads plus the authoritative live gear constraint.
        SableCollisionCapture.capturePreSubstepMotion(
            sceneHandle,
            getRuntimeId(),
            new Vector3d(getPose().position()),
            new Quaterniond(cachedOrientation),
            new Vector3d(temporaryLinearVelocityWorld),
            new Vector3d(temporaryAngularVelocityWorld)
        );
        substepArmed = true;
    }

    /**
     * Applies the intact-wheel constraint from live Sable-space terrain contact.
     * IV still supplies the content data (wheel station, size, friction, steer
     * and brake properties), but its mirrored 20 Hz isGrounded/isCollided flags
     * are not physical authority on this path.
     */
    private AirframeLoads.SubstepAerodynamicLoads calculateVehicleSubstepAerodynamics(
        EntityVehicleF_Physics vehicle, net.minecraft.world.level.Level level, PMWeatherIVConfig.Values config,
        AircraftState state, Vec3d origin, RotationMatrix orientation, Vec3d velocity, Vec3d omega,
        AircraftWind.WindFieldSnapshot wind, List<RotorModel.RotorActuatorSnapshot> rotors, double seconds
    ) {
        return vehicle.definition.motorized.isAircraft
            ? AircraftPhysics.calculateAirVehicleSubstepAerodynamics(vehicle,level,config,state,origin,orientation,
                velocity,omega,wind,rotors,seconds)
            : com.g9third.pmweatheriv.physics.RoadVehiclePhysics.calculateSubstepAerodynamics(vehicle,level,config,state,
                origin,orientation,velocity,omega,wind,seconds);
    }

    private void applyLiveLandingGearConstraint(double timeStep) {
        SablePoseConversions.worldToLocal(
            cachedOrientation, temporaryAngularVelocityWorld, temporaryAngularVelocityBody
        );
        SablePoseConversions.writeRotationMatrix(
            cachedOrientation, temporaryPhysicalOrientation
        );
        double liveConstraintMass = Math.max(1.0, vehicle.currentMass);
        double liveMassRatio = liveConstraintMass / Math.max(1.0, mass);
        Vec3d liveConstraintInertia = desiredInertia.scale(liveMassRatio);

        // Rapier applies scene gravity inside the step after this callback. Solve
        // the wheel constraint against that predicted post-gravity velocity, then
        // apply only the constraint delta now. When Rapier subsequently adds the
        // same gravity impulse, the completed velocity lands on the solved state.
        DimensionPhysicsData.getGravity(level, getPose().position(), sableGravityWorld);
        temporaryPredictedLinearVelocityWorld.set(temporaryLinearVelocityWorld).add(
            sableGravityWorld.x * timeStep,
            sableGravityWorld.y * timeStep,
            sableGravityWorld.z * timeStep
        );
        boolean profileGear = PMIVObserver.isCapturing(vehicle.uniqueUUID);
        long probeStart = profileGear ? System.nanoTime() : 0L;
        List<LandingGearSolver.LandingGearPhysicalContact> physicalContacts =
            probeLiveLandingGearContacts(
                timeStep, temporaryPredictedLinearVelocityWorld
            );

        if (profileGear) PMIVObserver.capturePerformance(vehicle.uniqueUUID,
                "landing_gear_terrain_probe", System.nanoTime() - probeStart);
        Map<PartGroundDevice, Double> gearGameplayImpacts = new IdentityHashMap<>();
        landingGearImpactArmed.keySet().removeIf(device -> !device.isValid);
        for (APart part : vehicle.allParts) {
            if (!(part instanceof PartGroundDevice device) || device.isSpare || !device.isValid) continue;
            LandingGearSolver.LandingGearPhysicalContact solidContact = null;
            for (LandingGearSolver.LandingGearPhysicalContact contact : physicalContacts) {
                if (contact.device() == device && !isLiquidSupport(contact)) {
                    solidContact = contact;
                    break;
                }
            }
            if (solidContact == null) {
                landingGearImpactArmed.put(device, true);
            } else if (solidContact.exactCollision()) {
                if (Boolean.TRUE.equals(landingGearImpactArmed.get(device))
                    || solidContact.surfaceGapMeters() > LIVE_GEAR_PROXIMITY_METERS) {
                    // Remove the future Rapier gravity increment used only for support.
                    // Rotation at the wheel remains part of the real touchdown speed.
                    double actualClosing = Math.max(0.0,
                        solidContact.inwardNormalSpeedMetersPerSecond() + timeStep * (
                            sableGravityWorld.x * solidContact.normalWorld().x()
                            + sableGravityWorld.y * solidContact.normalWorld().y()
                            + sableGravityWorld.z * solidContact.normalWorld().z()));
                    gearGameplayImpacts.put(device, actualClosing);
                }
                landingGearImpactArmed.put(device, false);
            }
        }

        // A severe wheel strike is not an indestructible point constraint. Before
        // the tire solver chooses its stopping impulse, let the same integrated
        // True-Impact material model used by BODY CCD consume terrain energy. The
        // returned residual inward speed becomes the unilateral constraint target,
        // so Sable alone applies the corresponding momentum change.
        for (int contactIndex = 0; contactIndex < physicalContacts.size(); ++contactIndex) {
            LandingGearSolver.LandingGearPhysicalContact contact = physicalContacts.get(contactIndex);
            if (!isLiquidSupport(contact)) {
                physicalContacts.set(contactIndex, resolveSevereLandingGearTerrainImpact(
                    contact, timeStep, liveConstraintMass, liveConstraintInertia
                ));
            }
        }

        long solveStart = profileGear ? System.nanoTime() : 0L;
        LandingGearSolver.LandingGearConstraintResult result =
            LandingGearSolver.solveLandingGearConstraints(
                vehicle,
                new Vec3d(
                    temporaryPredictedLinearVelocityWorld.x,
                    temporaryPredictedLinearVelocityWorld.y,
                    temporaryPredictedLinearVelocityWorld.z
                ),
                new Vec3d(
                    temporaryAngularVelocityBody.x,
                    temporaryAngularVelocityBody.y,
                    temporaryAngularVelocityBody.z
                ),
                liveConstraintMass,
                liveConstraintInertia,
                temporaryPhysicalOrientation,
                timeStep,
                physicalContacts,
                centerOfMassLocal,
                !vehicle.definition.motorized.isAircraft && cachedAircraftState != null
                    && lastLoadsGameTime != Long.MIN_VALUE && level.getGameTime()-lastLoadsGameTime<=2
                        ? cachedAircraftState.roadDrive : null
            );
        if (profileGear) PMIVObserver.capturePerformance(vehicle.uniqueUUID,
                "landing_gear_constraint_solve", System.nanoTime() - solveStart);
        lastSubstepLandingGear = result;
        Vector3d bodyUpWorld = cachedOrientation.transform(new Vector3d(0.0, 1.0, 0.0));
        updateRoadSuspensionTravel(result, timeStep, liveConstraintMass,
            Math.max(0.0, bodyUpWorld.y()));
        if (cachedAircraftState != null)
            cachedAircraftState.rotorcraftGroundSupported = result.totalNormalImpulseNewtonSeconds()>1e-6;

        // The impact impulse that damages a wheel must still exist physically.
        // Apply IV part-health consequences only after this substep's Sable tire
        // response has been solved; if the part is destroyed it simply disappears
        // from the next probe. The per-part debounce is independent of body crash.
        for (Map.Entry<PartGroundDevice, Double> impact : gearGameplayImpacts.entrySet()) {
            SableCollisionGameplay.processLandingGearImpact(this, impact.getKey(), impact.getValue());
        }

        Vec3d gearLinearDelta = result.linearVelocityChangeWorld();
        Vec3d gearAngularDeltaBody = result.angularVelocityChangeBody();
        if (gearLinearDelta.lengthSquared() > 1.0E-18
            || gearAngularDeltaBody.lengthSquared() > 1.0E-18) {
            temporaryLinearCorrectionWorld.set(
                gearLinearDelta.x(), gearLinearDelta.y(), gearLinearDelta.z()
            );
            temporaryAngularCorrectionBody.set(
                gearAngularDeltaBody.x(), gearAngularDeltaBody.y(), gearAngularDeltaBody.z()
            );
            SablePoseConversions.localToWorld(
                cachedOrientation, temporaryAngularCorrectionBody,
                temporaryAngularCorrectionWorld
            );
            rigidBodyHandle.addLinearAndAngularVelocity(
                temporaryLinearCorrectionWorld, temporaryAngularCorrectionWorld
            );
        } else {
            temporaryLinearCorrectionWorld.zero();
            temporaryAngularCorrectionBody.zero();
        }

        // Re-read the actual pre-step body velocity after the constraint delta.
        // The result object intentionally describes the predicted post-gravity
        // constrained velocity used by the tire solve; Rapier will add gravity
        // during the step itself.
        rigidBodyHandle.getLinearVelocity(temporaryLinearVelocityWorld);
        rigidBodyHandle.getAngularVelocity(temporaryAngularVelocityWorld);
        SablePoseConversions.worldToLocal(
            cachedOrientation, temporaryAngularVelocityWorld, temporaryAngularVelocityBody
        );
    }

    /** Updates one shared suspension displacement per authored wheel/tread assembly. */
    private void updateRoadSuspensionTravel(
        LandingGearSolver.LandingGearConstraintResult result,
        double timeStep,
        double liveMass,
        double verticalProjection
    ) {
        if (vehicle.definition.motorized.isAircraft || verticalProjection <= 0.25
            || vehicle.allParts == null) {
            roadSuspensionTravel.clear();
            return;
        }
        Map<PartGroundDevice, Integer> supportCounts = new IdentityHashMap<>();
        for (APart part : vehicle.allParts) {
            if (!(part instanceof PartGroundDevice device) || device.isSpare || !device.isValid
                || !device.isActiveVar.isActive || !RoadSuspensionModel.supportsFallback(vehicle, device)) continue;
            PartGroundDevice master = RoadSuspensionModel.masterPart(device);
            if (master != null) supportCounts.merge(master, 1, Integer::sum);
        }
        if (supportCounts.isEmpty()) {
            roadSuspensionTravel.clear();
            return;
        }
        // Fallback groups keep their share of the vehicle's total support mass.
        // Native-motion supports still carry their authored share; dividing only
        // by fallback groups would over-stiffen the remaining fallback wheels.
        int totalSupports = TireNormalCompliance.installedSupportCount(vehicle);
        Map<PartGroundDevice, Double> totalNormalForce = new IdentityHashMap<>();
        if (result != null && timeStep > 0.0) {
            for (LandingGearSolver.LandingGearContactSnapshot contact : result.contacts()) {
                PartGroundDevice device = contact.device();
                if (!contact.roadSuspension() || device == null) continue;
                PartGroundDevice master = RoadSuspensionModel.masterPart(device);
                if (master == null || !supportCounts.containsKey(master)) continue;
                double force = contact.normalImpulseNewtonSeconds() / timeStep;
                if (Double.isFinite(force) && force > 0.0)
                    totalNormalForce.merge(master, force, Double::sum);
            }
        }
        roadSuspensionTravel.keySet().removeIf(master -> !supportCounts.containsKey(master));
        for (Map.Entry<PartGroundDevice, Integer> entry : supportCounts.entrySet()) {
            PartGroundDevice master = entry.getKey();
            int count = entry.getValue();
            double groupMass = Math.max(1.0, liveMass) * count / Math.max(1, totalSupports);
            double sag = RoadSuspensionModel.sagMeters(master);
            double previous = roadSuspensionTravel.getOrDefault(master, 0.0);
            double next = RoadSuspensionModel.nextTravelMeters(previous,
                totalNormalForce.getOrDefault(master, 0.0), groupMass, sag, timeStep, verticalProjection);
            roadSuspensionTravel.put(master, next);
        }
    }

    /**
     * Builds physical wheel contacts directly from the current rigid pose.
     * Contact becomes exact either when the tire-bottom station reaches a
     * collision-shape top surface or when its post-force point velocity would
     * cross that surface during this very Sable substep. The latter is a
     * continuous collision test, not a release grace/latch.
     */
    private List<LandingGearSolver.LandingGearPhysicalContact>
        probeLiveLandingGearContacts(
            double timeStep,
            Vector3dc predictedLinearVelocityWorld
        ) {
        List<LandingGearSolver.LandingGearPhysicalContact> contacts =
            new ArrayList<>();

        Vector3d bodyPosition = new Vector3d(getPose().position());
        Vector3d omegaWorld = new Vector3d(temporaryAngularVelocityWorld);
        int activeGroundDeviceCount = 0;
        if (vehicle.allParts != null && !vehicle.allParts.isEmpty()) {
            for (APart part : vehicle.allParts) {
                if (!(part instanceof PartGroundDevice device)
                    || device.isSpare
                    || !device.isValid
                    || (!device.isActiveVar.isActive
                        && !allowsTransientFreshPlacementGroundDevice(device))) {
                    continue;
                }
                ++activeGroundDeviceCount;
                double verticalProjection = Math.max(0.0,
                    cachedOrientation.transform(new Vector3d(0.0, 1.0, 0.0)).y());
                boolean roadSuspension = verticalProjection > 0.25
                    && RoadSuspensionModel.supportsFallback(vehicle, device);
                Vec3d pointLocal;
                RoadContact roadContact = null;
                if (roadSuspension) {
                    Vec3d centerLocal = RoadSuspensionModel.baseCenterLocal(device);
                    double travel = roadSuspensionTravel(device);
                    Vec3d basePoint = centerLocal == null ? null
                        : RoadSuspensionModel.supportPointLocal(device, centerLocal, temporaryPhysicalOrientation);
                    pointLocal = basePoint == null ? null : basePoint.add(new Vec3d(0.0, travel, 0.0));
                    if (pointLocal != null) roadContact = new RoadContact(
                        travel, RoadSuspensionModel.sagMeters(device), verticalProjection);
                } else {
                    pointLocal = LandingGearSolver.landingGearDeviceContactPointLocal(
                        device, temporaryPhysicalOrientation);
                }
                if (pointLocal == null || !pointLocal.isFinite()) {
                    continue;
                }
                addLiveTerrainSupportContact(
                    contacts, device, pointLocal, "GROUND_DEVICE",
                    timeStep, predictedLinearVelocityWorld, bodyPosition, omegaWorld,
                    device.definition != null && device.definition.ground != null
                        && device.definition.ground.canFloat, false, roadContact
                );
            }
        }

        // IV uses live BLOCK boxes marked collidesWithLiquids as hull/pontoon
        // support, independently of ground-device canFloat. Keep box eligibility
        // and the animated part transform sourced from allCollisionBoxes rather
        // than a frozen definition-only approximation.
        addAuthoredLiquidCollisionBoxContacts(
            contacts, timeStep, predictedLinearVelocityWorld, bodyPosition, omegaWorld
        );

        // Some skid/float aircraft do not instantiate an IV PartGroundDevice at
        // runtime even though their rigid support exists in the vehicle OBJ. A
        // coarse 0.20 m Sable voxel shell is appropriate for body/crash contact
        // but not for the precise ride height of a narrow skid rail. When no
        // active IV ground device exists, probe exact low-envelope vertices from
        // the retained static support mesh through the same unilateral point
        // constraint used by authored gear. These contacts deliberately have a
        // null device: they get normal support and bounded passive material friction, with no
        // wheel steering, rolling resistance or invented service-brake command.
        if (activeGroundDeviceCount == 0 && compoundCollider != null) {
            for (SableModelCollisionHull.StaticSupportPoint support
                : compoundCollider.staticGroundSupportPoints()) {
                Vec3d pointLocal = new Vec3d(support.x(), support.y(), support.z());
                addLiveTerrainSupportContact(
                    contacts, null, pointLocal, "STATIC_MODEL_SUPPORT",
                    timeStep, predictedLinearVelocityWorld, bodyPosition, omegaWorld
                );
            }
        }
        return contacts;
    }

    private void addAuthoredLiquidCollisionBoxContacts(
        List<LandingGearSolver.LandingGearPhysicalContact> contacts,
        double timeStep,
        Vector3dc predictedLinearVelocityWorld,
        Vector3d bodyPosition,
        Vector3d omegaWorld
    ) {
        if (vehicle.allCollisionBoxes == null || vehicle.allCollisionBoxes.isEmpty()) return;
        Quaterniond vehicleOrientation = SablePoseConversions.toQuaternion(vehicle.orientation);
        Quaterniond vehicleOrientationInverse = new Quaterniond(vehicleOrientation).conjugate().normalize();
        for (BoundingBox box : vehicle.allCollisionBoxes) {
            if (box == null || box.collisionTypes == null
                || !LiquidSupportModel.isAuthoredLiquidCollisionBox(
                    box.collisionTypes.contains(CollisionType.BLOCK), box.collidesWithLiquids,
                    box.widthRadius, box.heightRadius, box.depthRadius)) {
                continue;
            }
            APart partOwner = vehicle.getPartWithBox(box);
            APart geometricOwner = partOwner;
            Vec3d ownerOriginVehicleLocal = Vec3d.ZERO;
            Quaterniond ownerToVehicle = new Quaterniond();
            if (partOwner != null) {
                geometricOwner = partOwner;
                if (partOwner.localOffset == null || partOwner.orientation == null) continue;
                ownerOriginVehicleLocal = new Vec3d(
                    partOwner.localOffset.x, partOwner.localOffset.y, partOwner.localOffset.z
                );
                Quaterniond partOrientation = SablePoseConversions.toQuaternion(partOwner.orientation);
                ownerToVehicle = new Quaterniond(vehicleOrientationInverse)
                    .mul(partOrientation).normalize();
            }
            Vector3d rawCenterOwnerLocal = OrientedHitboxRegistry.unroundedLocalCenter(
                box, geometricOwner == null ? vehicle : geometricOwner
            );
            Vec3d centerOwnerLocal = rawCenterOwnerLocal == null ? null : new Vec3d(
                rawCenterOwnerLocal.x, rawCenterOwnerLocal.y, rawCenterOwnerLocal.z
            );
            Vec3d[] hullCorners = LiquidSupportModel.bottomCornersVehicleLocal(
                centerOwnerLocal, box.widthRadius, box.heightRadius, box.depthRadius,
                ownerOriginVehicleLocal, ownerToVehicle
            );
            for (Vec3d pointLocal : hullCorners) {
                boolean duplicateLiquidStation = false;
                for (LandingGearSolver.LandingGearPhysicalContact existing : contacts) {
                    if (isLiquidSupport(existing)
                        && existing.pointLocal().subtract(pointLocal).lengthSquared() <= 1.0e-8) {
                        duplicateLiquidStation = true;
                        break;
                    }
                }
                if (duplicateLiquidStation) continue;
                addLiveTerrainSupportContact(
                    contacts, null, pointLocal, "AUTHORED_LIQUID_BOX",
                    timeStep, predictedLinearVelocityWorld, bodyPosition, omegaWorld, true, true
                );
            }
        }
    }

    private void addLiveTerrainSupportContact(
        List<LandingGearSolver.LandingGearPhysicalContact> contacts,
        @Nullable PartGroundDevice device,
        Vec3d pointLocal,
        String sourceMode,
        double timeStep,
        Vector3dc predictedLinearVelocityWorld,
        Vector3d bodyPosition,
        Vector3d omegaWorld
    ) {
        boolean canFloat = device != null && device.definition != null
            && device.definition.ground != null && device.definition.ground.canFloat;
        addLiveTerrainSupportContact(contacts, device, pointLocal, sourceMode, timeStep,
            predictedLinearVelocityWorld, bodyPosition, omegaWorld, canFloat, false);
    }

    private void addLiveTerrainSupportContact(
        List<LandingGearSolver.LandingGearPhysicalContact> contacts,
        @Nullable PartGroundDevice device,
        Vec3d pointLocal,
        String sourceMode,
        double timeStep,
        Vector3dc predictedLinearVelocityWorld,
        Vector3d bodyPosition,
        Vector3d omegaWorld,
        boolean allowLiquid,
        boolean liquidOnly
    ) {
        addLiveTerrainSupportContact(contacts, device, pointLocal, sourceMode, timeStep,
            predictedLinearVelocityWorld, bodyPosition, omegaWorld, allowLiquid, liquidOnly, null);
    }

    private void addLiveTerrainSupportContact(
        List<LandingGearSolver.LandingGearPhysicalContact> contacts,
        @Nullable PartGroundDevice device,
        Vec3d pointLocal,
        String sourceMode,
        double timeStep,
        Vector3dc predictedLinearVelocityWorld,
        Vector3d bodyPosition,
        Vector3d omegaWorld,
        boolean allowLiquid,
        boolean liquidOnly,
        @Nullable RoadContact roadContact
    ) {
        Vector3d leverWorld = new Vector3d(
            pointLocal.x() - centerOfMassLocal.x(),
            pointLocal.y() - centerOfMassLocal.y(),
            pointLocal.z() - centerOfMassLocal.z()
        );
        cachedOrientation.transform(leverWorld);
        Vector3d pointWorld = new Vector3d(bodyPosition).add(leverWorld);

        Vector3d rotationalVelocity = new Vector3d();
        omegaWorld.cross(leverWorld, rotationalVelocity);
        double pointVerticalVelocity =
            (predictedLinearVelocityWorld == null
                ? temporaryLinearVelocityWorld.y
                : predictedLinearVelocityWorld.y())
            + rotationalVelocity.y;
        double crossingDistance = Math.max(
            0.0, -pointVerticalVelocity * Math.max(0.0, timeStep)
        );
        double lookDown = Math.max(
            LIVE_GEAR_PROXIMITY_METERS,
            crossingDistance + LIVE_GEAR_CONTACT_SKIN_METERS
        );
        Vector3d wheelUp = cachedOrientation.transform(new Vector3d(0, 1, 0));
        double wheelRadius = device != null && device.definition.ground.isWheel
                && Double.isFinite(device.getHeight()) ? Math.max(0.0, device.getHeight() * 0.5) : 0.0;
        TerrainSupportSample support = liveTerrainSupport(
            pointWorld.x, pointWorld.y, pointWorld.z, lookDown, allowLiquid, liquidOnly,
            wheelRadius
        );
        if (support == null || !Double.isFinite(support.topY())) {
            return;
        }

        Vec3d normal = support.normalWorld();
        double gap = (pointWorld.y - support.topY()) * normal.y();
        Vec3d pointVelocity = new Vec3d(
            predictedLinearVelocityWorld == null ? temporaryLinearVelocityWorld.x : predictedLinearVelocityWorld.x(),
            predictedLinearVelocityWorld == null ? temporaryLinearVelocityWorld.y : predictedLinearVelocityWorld.y(),
            predictedLinearVelocityWorld == null ? temporaryLinearVelocityWorld.z : predictedLinearVelocityWorld.z()
        ).add(new Vec3d(rotationalVelocity.x, rotationalVelocity.y, rotationalVelocity.z));
        double inwardVelocity = pointVelocity.dot(normal);
        crossingDistance = Math.max(0.0, -inwardVelocity * Math.max(0.0, timeStep));
        // A float-enabled IV ground device is a buoyant support station, not a
        // rigid tire that should be projected to the fluid surface. While
        // submerged it carries a unilateral vertical reaction at its current
        // depth; it receives neither tire friction nor wheel-strike damage.
        boolean liquidSupport = support.liquid();
        double penetration = LiquidSupportModel.penetrationMeters(gap, liquidSupport);
        boolean predictiveCrossing =
            gap > LIVE_GEAR_CONTACT_SKIN_METERS
                && inwardVelocity < 0.0
                && gap <= crossingDistance + LIVE_GEAR_CONTACT_SKIN_METERS;
        boolean exact = liquidSupport
            ? gap <= LIVE_GEAR_CONTACT_SKIN_METERS || predictiveCrossing
            : penetration > 0.0 || gap <= LIVE_GEAR_CONTACT_SKIN_METERS || predictiveCrossing;
        boolean proximity =
            !liquidSupport && !exact && gap >= 0.0 && gap <= LIVE_GEAR_PROXIMITY_METERS;
        if (!exact && !proximity) {
            return;
        }

        // For an impending crossing that begins outside the 10 mm continuous
        // contact skin, do not stop the wheel at its current positive gap. That
        // created the 0.8.3co 18-25 mm hover/chatter loop: gravity would re-arm
        // the speculative contact, receive a large zero-speed impulse, then lose
        // support again. Instead constrain only enough inward point speed for the
        // tire bottom to arrive at the skin at the end of this Sable substep. Once
        // there, the ordinary exact-contact solver carries gravity continuously.
        // The target remains unilateral and is never used for non-crossing 5 cm
        // proximity contacts, so it cannot recreate the old airborne proximity latch.
        double normalVelocityTarget;
        if (predictiveCrossing && timeStep > 1.0E-9) {
            normalVelocityTarget = -Math.max(
                0.0, gap - LIVE_GEAR_CONTACT_SKIN_METERS
            ) / timeStep;
        } else {
            normalVelocityTarget = exact ? 0.0 : Double.NaN;
        }
        String state = liquidSupport
            ? predictiveCrossing ? "PREDICTIVE_CROSSING" : "BUOYANT"
            : penetration > 0.0
                ? "PENETRATING"
                : predictiveCrossing
                    ? "PREDICTIVE_CROSSING"
                    : exact
                        ? "CONTINUOUS_SKIN"
                        : "PROXIMITY_ONLY";
        Vec3d applicationLocal=pointLocal;
        if (exact && !predictiveCrossing && !liquidSupport) {
            // Apply support at the measured shape point, including a rounded wheel's edge contact.
            Vec3d surfacePoint = support.worldPoint() == null
                    ? new Vec3d(pointWorld.x, support.topY(), pointWorld.z) : support.worldPoint();
            Vector3d correction = new Vector3d(surfacePoint.x() - pointWorld.x,
                    surfacePoint.y() - pointWorld.y, surfacePoint.z() - pointWorld.z);
            SablePoseConversions.worldToLocal(cachedOrientation, correction, correction);
            applicationLocal = pointLocal.add(new Vec3d(correction.x, correction.y, correction.z));
            pointWorld.set(surfacePoint.x(), surfacePoint.y(), surfacePoint.z());
        }
        contacts.add(new LandingGearSolver.LandingGearPhysicalContact(
            device, applicationLocal, gap, (liquidSupport ? "FLOAT_LIQUID" : sourceMode) + '_' + state,
            penetration, exact, normalVelocityTarget,
            Math.max(0.0, -inwardVelocity),
            new Vec3d(pointWorld.x, pointWorld.y, pointWorld.z),
            support.blockPos(), false,
            roadContact == null ? Double.NaN : RoadSuspensionModel.complianceGapMeters(
                gap, roadContact.travelMeters(), Math.max(0.0,
                    wheelUp.x * normal.x() + wheelUp.y * normal.y() + wheelUp.z * normal.z()),
                roadContact.sagMeters()),
            roadContact != null, normal
        ));
    }

    private record RoadContact(double travelMeters, double sagMeters, double verticalProjection) {}

    /**
     * Returns the highest eligible solid collision-shape or authored float
     * surface under/through one support point in its local search interval,
     * together with the exact terrain voxel that supplied the support.
     */
    private TerrainSupportSample liveTerrainSupport(
        double worldX,
        double worldY,
        double worldZ,
        double lookDownMeters
    ) {
        return liveTerrainSupport(worldX, worldY, worldZ, lookDownMeters, false);
    }

    private TerrainSupportSample liveTerrainSupport(
        double worldX,
        double worldY,
        double worldZ,
        double lookDownMeters,
        boolean allowLiquid
    ) {
        return liveTerrainSupport(worldX, worldY, worldZ, lookDownMeters, allowLiquid, false);
    }

    private TerrainSupportSample liveTerrainSupport(
        double worldX,
        double worldY,
        double worldZ,
        double lookDownMeters,
        boolean allowLiquid,
        boolean liquidOnly
    ) {
        return liveTerrainSupport(worldX, worldY, worldZ, lookDownMeters, allowLiquid, liquidOnly,
                0.0);
    }

    private TerrainSupportSample liveTerrainSupport(double worldX, double worldY, double worldZ,
            double lookDownMeters, boolean allowLiquid, boolean liquidOnly,
            double wheelRadius) {
        if (!Double.isFinite(worldX) || !Double.isFinite(worldY)
            || !Double.isFinite(worldZ)) {
            return null;
        }
        double minimumTop = worldY - Math.max(
            LIVE_GEAR_PROXIMITY_METERS,
            Math.max(0.0, lookDownMeters)
        );
        double maximumTop = worldY + LIVE_GEAR_MAX_PENETRATION_METERS;
        // IV already derives this lowest support point from tire axle, tilt and width.
        // Anchor the rounded edge proxy here so ordinary top-face ride height is unchanged.
        double centerX = worldX;
        double centerZ = worldZ;
        int reach = Math.max(1, (int) Math.ceil(wheelRadius));
        int baseX = (int) Math.floor(centerX);
        int baseY = (int) Math.floor(worldY);
        int baseZ = (int) Math.floor(centerZ);
        int minimumY = (int) Math.floor(minimumTop) - 1;
        int maximumY = (int) Math.floor(maximumTop + wheelRadius) + 1;
        double bestTop = Double.NEGATIVE_INFINITY;
        BlockPos bestBlock = null;
        boolean bestIsLiquid = false;
        Vec3d bestNormal = new Vec3d(0, 1, 0);
        Vec3d bestPoint = null;
        BlockPos.MutableBlockPos mutable = new BlockPos.MutableBlockPos();

        for (int blockX = baseX - reach; blockX <= baseX + reach; ++blockX) {
            for (int blockZ = baseZ - reach; blockZ <= baseZ + reach; ++blockZ) {
                if (allowLiquid) {
                    // A float station may already be submerged below the local
                    // search band. Find the top of the contiguous fluid column
                    // containing/under it, without scanning the whole dimension
                    // or treating the water surface as rigid penetration depth.
                    int fluidY = minimumY;
                    while (fluidY <= maximumY) {
                        mutable.set(blockX, fluidY, blockZ);
                        var fluid = level.getBlockState(mutable).getFluidState();
                        if (fluid.isEmpty()) {
                            ++fluidY;
                            continue;
                        }
                        var fluidType = fluid.getType();
                        int surfaceBlockY = fluidY;
                        var surfaceFluid = fluid;
                        while (surfaceBlockY + 1 < level.getMaxBuildHeight()) {
                            mutable.set(blockX, surfaceBlockY + 1, blockZ);
                            var above = level.getBlockState(mutable).getFluidState();
                            if (above.isEmpty() || !fluidType.isSame(above.getType())) break;
                            ++surfaceBlockY;
                            surfaceFluid = above;
                        }
                        mutable.set(blockX, surfaceBlockY, blockZ);
                        double fluidTop = LiquidSupportModel.surfaceY(
                            surfaceBlockY, surfaceFluid.getHeight(level, mutable), true
                        );
                        boolean pointInsideColumn = worldX >= blockX - LIVE_GEAR_HORIZONTAL_EPSILON
                            && worldX <= blockX + 1.0 + LIVE_GEAR_HORIZONTAL_EPSILON
                            && worldZ >= blockZ - LIVE_GEAR_HORIZONTAL_EPSILON
                            && worldZ <= blockZ + 1.0 + LIVE_GEAR_HORIZONTAL_EPSILON;
                        if (pointInsideColumn && Double.isFinite(fluidTop)
                            && fluidTop >= minimumTop - LIVE_GEAR_CONTACT_SKIN_METERS
                            && fluidTop > bestTop + LIVE_GEAR_HORIZONTAL_EPSILON) {
                            bestTop = fluidTop;
                            bestBlock = new BlockPos(blockX, surfaceBlockY, blockZ);
                            bestIsLiquid = true;
                            bestNormal = new Vec3d(0, 1, 0);
                            bestPoint = null;
                        }
                        fluidY = surfaceBlockY + 1;
                    }
                }
                if (liquidOnly) continue;
                for (int blockY = minimumY; blockY <= maximumY; ++blockY) {
                    mutable.set(blockX, blockY, blockZ);
                    BlockState state = level.getBlockState(mutable);
                    if (state.isAir()) {
                        continue;
                    }
                    VoxelShape shape = state.getCollisionShape(level, mutable);
                    if (shape.isEmpty()) {
                        continue;
                    }
                    for (AABB localBox : shape.toAabbs()) {
                        double minX = blockX + localBox.minX;
                        double maxX = blockX + localBox.maxX;
                        double minZ = blockZ + localBox.minZ;
                        double maxZ = blockZ + localBox.maxZ;
                        double contactX = Vec3d.clamp(centerX, minX, maxX);
                        double contactZ = Vec3d.clamp(centerZ, minZ, maxZ);
                        double dx = centerX - contactX, dz = centerZ - contactZ;
                        double radialSquared = dx * dx + dz * dz;
                        Vec3d normal = new Vec3d(0, 1, 0);
                        double shapeTop = blockY + localBox.maxY;
                        double top = shapeTop;
                        if (wheelRadius > 1.0e-4) {
                            // Sphere/voxel support gives a real edge normal rather than a fitted terrain slope.
                            // Ordinary top-face contact is unchanged; wheel radius comes from the content pack.
                            double riseSquared = wheelRadius * wheelRadius - radialSquared;
                            if (riseSquared <= 1.0e-12) continue;
                            double rise = Math.sqrt(riseSquared);
                            normal = new Vec3d(dx, rise, dz).normalized();
                            if (normal.y() < 0.1) continue;
                            top += rise - wheelRadius;
                        } else if (radialSquared > LIVE_GEAR_HORIZONTAL_EPSILON * LIVE_GEAR_HORIZONTAL_EPSILON) {
                            continue;
                        }
                        if (top < minimumTop - LIVE_GEAR_CONTACT_SKIN_METERS
                            || top > maximumTop + LIVE_GEAR_CONTACT_SKIN_METERS) {
                            continue;
                        }
                        if (top > bestTop + LIVE_GEAR_HORIZONTAL_EPSILON
                            || (Math.abs(top - bestTop) <= LIVE_GEAR_HORIZONTAL_EPSILON && bestIsLiquid)) {
                            bestTop = top;
                            bestBlock = new BlockPos(blockX, blockY, blockZ);
                            bestIsLiquid = false;
                            bestNormal = normal;
                            bestPoint = new Vec3d(contactX, shapeTop, contactZ);
                        }
                    }
                }
            }
        }
        return bestBlock == null ? null : new TerrainSupportSample(bestTop, bestBlock, bestIsLiquid,
                bestNormal, bestPoint);
    }

    /**
     * Compatibility helper for callers that only need the terrain support height.
     */
    private double liveTerrainSupportTop(
        double worldX,
        double worldY,
        double worldZ,
        double lookDownMeters
    ) {
        TerrainSupportSample sample = liveTerrainSupport(
            worldX, worldY, worldZ, lookDownMeters
        );
        return sample == null ? Double.NaN : sample.topY();
    }

    /**
     * Resolves a high-severity authored ground-device strike through the same
     * material model as swept BODY contact. Ordinary touchdowns remain on the
     * existing tire constraint path; destructive evaluation begins only once
     * the integrated material solver's physical penetration thresholds are met.
     */
    private LandingGearSolver.LandingGearPhysicalContact resolveSevereLandingGearTerrainImpact(
        LandingGearSolver.LandingGearPhysicalContact contact,
        double timeStep,
        double liveConstraintMass,
        Vec3d liveConstraintInertia
    ) {
        if (contact == null || contact.device() == null || !contact.exactCollision()
            || contact.supportBlock() == null || contact.worldPoint() == null
            || contact.inwardNormalSpeedMetersPerSecond() <= 0.0
            || !terrainDamageAllowedForCurrentCrashEpisode()
            || !PMWeatherIVConfig.get().enableBlockBreaking()
            || !ConfigSystem.settings.damage.vehicleBlockBreaking.value) {
            return contact;
        }

        // Do not gate world damage on optional IV crash-speed fields. Many otherwise
        // complete content packs leave crashSpeedMin/crashSpeedMax at zero, which made
        // severe wheel strikes silently bypass the terrain material solver. The
        // integrated True Impact model already owns the physical severity gate, so use
        // its penetration speed/energy thresholds for every pack. Normal runway support
        // remains far below this path.
        double inwardSpeed = contact.inwardNormalSpeedMetersPerSecond();
        if (inwardSpeed < ImpactRuntimeConfig.PENETRATION_MIN_SPEED_MS) {
            return contact;
        }

        double effectiveMass = landingGearNormalEffectiveMass(
            contact.pointLocal(), contact.normalWorld(), liveConstraintMass, liveConstraintInertia
        );
        if (!(effectiveMass > 0.0) || !Double.isFinite(effectiveMass)) {
            return contact;
        }
        double rigidStopImpulseNs = effectiveMass * inwardSpeed;
        double physicalImpactEnergyJ = AircraftTerrainImpact.deriveImpactEnergyJ(
            rigidStopImpulseNs, inwardSpeed
        );
        if (physicalImpactEnergyJ < ImpactRuntimeConfig.PENETRATION_TRIGGER_J) {
            return contact;
        }
        double width = contact.device().getWidth();
        double height = contact.device().getHeight();
        double projectedAreaM2 = Double.isFinite(width) && width > 0.0
            && Double.isFinite(height) && height > 0.0
                ? width * height : 0.0;
        if (!(projectedAreaM2 > 0.0)) {
            return contact;
        }
        double maxTravelMeters = Math.max(0.0,
            inwardSpeed * Math.max(0.0, timeStep)
                + Math.max(0.0, contact.penetrationDepthMeters())
        );
        if (!(maxTravelMeters > 0.0)) {
            return contact;
        }

        Resolution resolution = AircraftTerrainImpact.resolveExternalPenetration(
            level, contact.supportBlock(),
            new Vector3d(
                contact.worldPoint().x(), contact.worldPoint().y(), contact.worldPoint().z()
            ),
            new Vector3d(contact.normalWorld().x(), contact.normalWorld().y(), contact.normalWorld().z()),
            projectedAreaM2, rigidStopImpulseNs, inwardSpeed, maxTravelMeters
        );
        if (resolution == null || !resolution.accepted()) {
            return contact;
        }

        double residualInwardSpeed = Math.max(0.0, resolution.residualInwardSpeedMps());
        double targetNormalSpeed = -residualInwardSpeed;
        if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
            "SABLE_LANDING_GEAR_TRUE_IMPACT gameTime=" + level.getGameTime()
                + " uuid=" + vehicle.uniqueUUID
                + " inwardNormalSpeedMps=" + inwardSpeed
                + " severityGate=TRUE_IMPACT_PHYSICAL_THRESHOLDS"
                + " penetrationMinSpeedMps=" + ImpactRuntimeConfig.PENETRATION_MIN_SPEED_MS
                + " penetrationTriggerJ=" + ImpactRuntimeConfig.PENETRATION_TRIGGER_J
                + " effectiveNormalMassKg=" + effectiveMass
                + " rigidStopImpulseNs=" + rigidStopImpulseNs
                + " impactEnergyJ=" + resolution.impactEnergyJ()
                + " absorbedEnergyJ=" + resolution.absorbedEnergyJ()
                + " residualInwardSpeedMps=" + residualInwardSpeed
                + " blocksBroken=" + resolution.blocksBroken()
                + " mode=" + resolution.mode()
                + " contactAreaM2=" + projectedAreaM2
                + " maxTravelM=" + maxTravelMeters
                + " seedBlock=" + contact.supportBlock()
                + " momentumAuthority=SABLE_TARGET_RESIDUAL_POINT_SPEED"
        );
        return contact.withTrueImpactResidual(targetNormalSpeed);
    }

    private double landingGearNormalEffectiveMass(
        Vec3d pointLocal, Vec3d normalWorld, double liveConstraintMass, Vec3d liveConstraintInertia
    ) {
        if (pointLocal == null || !pointLocal.isFinite()
            || !(liveConstraintMass > 0.0) || liveConstraintInertia == null
            || !liveConstraintInertia.isFinite()) {
            return Double.NaN;
        }
        Vec3d leverLocal = pointLocal.subtract(centerOfMassLocal);
        Vector3d directionBodyVector = new Vector3d(normalWorld.x(), normalWorld.y(), normalWorld.z());
        new Quaterniond(cachedOrientation).conjugate().transform(directionBodyVector);
        Vec3d directionBody = new Vec3d(
            directionBodyVector.x, directionBodyVector.y, directionBodyVector.z
        ).normalized();
        Vec3d angularJacobian = leverLocal.cross(directionBody);
        double denominator = 1.0 / liveConstraintMass
            + angularJacobian.x() * angularJacobian.x() / Math.max(MIN_INERTIA, liveConstraintInertia.x())
            + angularJacobian.y() * angularJacobian.y() / Math.max(MIN_INERTIA, liveConstraintInertia.y())
            + angularJacobian.z() * angularJacobian.z() / Math.max(MIN_INERTIA, liveConstraintInertia.z());
        return Double.isFinite(denominator) && denominator > 1.0E-12
            ? 1.0 / denominator : Double.NaN;
    }

    private static boolean isLiquidSupport(LandingGearSolver.LandingGearPhysicalContact contact) {
        return contact != null && contact.contactMode() != null
            && contact.contactMode().startsWith("FLOAT_LIQUID_");
    }

    private record TerrainSupportSample(double topY, BlockPos blockPos, boolean liquid,
                                        Vec3d normalWorld, Vec3d worldPoint) {
    }

    /**
     * Validates the completed persistent substep and enforces the swept BODY
     * terrain boundary before another Sable substep can integrate an embedded
     * aircraft. Ordinary Rapier rebounds are retained; only velocity components
     * that are still travelling along the missed penetration sweep are removed.
     */
    public void finishSubstep() {
        if (!isUsable() || !substepArmed) {
            return;
        }
        if (riderReconnectHeld) {
            // Rapier applies scene gravity/contact during the shared step. Re-pin
            // after integration so the reconnect hold is truly pose-stationary.
            placeBodyWithVelocity(
                new Vector3d(),
                new Vector3d(),
                new Quaterniond(riderReconnectHoldOrientationWorld),
                new Vector3d(riderReconnectHoldPositionWorld)
            );
            substepArmed = false;
            return;
        }
        updatePose();
        rigidBodyHandle.getLinearVelocity(temporaryLinearVelocityWorld);
        rigidBodyHandle.getAngularVelocity(temporaryAngularVelocityWorld);
        if (!finite(getPose().position()) || !finite(getPose().orientation())
            || !finite(temporaryLinearVelocityWorld) || !finite(temporaryAngularVelocityWorld)) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "ERROR stage=persistentFlightSubstep reason=nonFinite uuid=" + vehicle.uniqueUUID
            );
            quarantineNonFiniteBody("persistentFlightSubstep");
            hasExportedState = false;
        } else {
            terrainResponse.resolveSweptTerrainBoundaryAfterSubstep();
            recoverSolidGearPenetration();
        }
        substepArmed = false;
    }

    /** Repairs solid tire pose overlap beyond its existing deflection range, preserving momentum. */
    private void recoverSolidGearPenetration() {
        if (vehicle.allParts == null || vehicle.outOfHealth || compoundCollider == null) return;
        updatePose();
        Quaterniond orientation = new Quaterniond(getPose().orientation()).normalize();
        double verticalProjection = Math.max(0.0,
            orientation.transform(new Vector3d(0.0, 1.0, 0.0)).y);
        RotationMatrix physicalOrientation = new RotationMatrix();
        SablePoseConversions.writeRotationMatrix(orientation, physicalOrientation);
        Vector3d origin = modelOriginFromCenterOfMassWorld(getPose().position(), orientation);
        double correction = 0.0;
        double worstGap = 0.0;
        for (APart part : vehicle.allParts) {
            if (!(part instanceof PartGroundDevice device)
                || !com.g9third.pmweatheriv.physics.TireContactMaterial.activeSupport(device)) continue;
            boolean roadSuspension = verticalProjection > 0.25
                && RoadSuspensionModel.supportsFallback(vehicle, device);
            double travel = roadSuspension ? roadSuspensionTravel(device) : 0.0;
            Vec3d local;
            if (roadSuspension) {
                Vec3d center = RoadSuspensionModel.baseCenterLocal(device);
                Vec3d basePoint = center == null ? null
                    : RoadSuspensionModel.supportPointLocal(device, center, physicalOrientation);
                local = basePoint == null ? null : basePoint.add(new Vec3d(0.0, travel, 0.0));
            } else {
                local = LandingGearSolver.landingGearDeviceContactPointLocal(device, physicalOrientation);
            }
            if (local == null || !local.isFinite()) continue;
            Vector3d point = orientation.transform(new Vector3d(local.x(), local.y(), local.z())).add(origin);
            TerrainSupportSample support = liveTerrainSupport(point.x, point.y, point.z,
                LIVE_GEAR_PROXIMITY_METERS, device.definition.ground.canFloat, false);
            if (support == null || support.liquid()) continue;
            double gap = point.y - support.topY();
            double baseGap = roadSuspension ? gap - travel * verticalProjection : gap;
            double required;
            if (roadSuspension) {
                double bodyTravelCorrection = RoadSuspensionModel.roadPoseCorrectionMeters(
                    baseGap, device, verticalProjection, LIVE_GEAR_CONTACT_SKIN_METERS);
                double tireOrHardwareCorrection =
                    com.g9third.pmweatheriv.physics.TireNormalCompliance.upwardRoadPoseCorrection(
                        gap, device.definition.ground.isWheel
                            ? com.g9third.pmweatheriv.physics.TireNormalCompliance.nominalDeflection(device)
                            : 0.0,
                        LIVE_GEAR_CONTACT_SKIN_METERS);
                required = Math.max(bodyTravelCorrection, tireOrHardwareCorrection);
            } else {
                required = com.g9third.pmweatheriv.physics.TireNormalCompliance.upwardPoseCorrection(
                    gap, com.g9third.pmweatheriv.physics.TireNormalCompliance.nominalDeflection(device),
                    LIVE_GEAR_CONTACT_SKIN_METERS);
            }
            if (required > correction) { correction = required; worstGap = gap; }
        }
        if (!(correction > 1.0e-5) || correction > LIVE_GEAR_MAX_PENETRATION_METERS) return;
        Vector3d correctedPosition = new Vector3d(getPose().position()).add(0.0, correction + 1.0e-6, 0.0);
        // A tire correction must not push the structural body through an overhang or ceiling.
        if (compoundCollider.bodyIntersectsTerrainAtPose(level, correctedPosition, orientation)) return;
        rigidBodyHandle.getLinearVelocity(temporaryLinearVelocityWorld);
        rigidBodyHandle.getAngularVelocity(temporaryAngularVelocityWorld);
        placeBodyWithVelocity(new Vector3d(temporaryLinearVelocityWorld),
            new Vector3d(temporaryAngularVelocityWorld), orientation, correctedPosition);
        terrainResponse.rememberClearTerrainPose(correctedPosition, orientation);
        if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
            "SABLE_GEAR_POSE_OVERLAP_RECOVERED gameTime=" + level.getGameTime()
                + " uuid=" + vehicle.uniqueUUID + " worstGapM=" + worstGap
                + " upwardCorrectionM=" + correction
                + " linearAngularMomentumPreserved=true extraFrictionImpulse=false attitudeTarget=false");
    }

    /**
     * Stops a non-finite Rapier body from contaminating later shared-scene
     * substeps. The body is first teleported to the strongest known finite pose
     * with zero velocity, then marked unusable so the manager removes/recreates
     * it on the next IV owner tick. No fallback aircraft physics is injected.
     */
    public void quarantineIntegrationFailure(String stage, Throwable failure) {
        if (PMIVObserver.loggingEnabled()) PMIVObserver.log("SABLE_INTEGRATION_FAILURE uuid=" + vehicle.uniqueUUID
            + " stage=" + stage + " type=" + failure.getClass().getName()
            + " message=" + String.valueOf(failure.getMessage()).replace('\n', ' '));
        if (cachedAircraftState != null) {
            cachedAircraftState.pendingSeparation.clear();
            cachedAircraftState.kinematics = null;
        }
        quarantineNonFiniteBody(stage);
    }

    private void quarantineNonFiniteBody(String stage) {
        if (poisoned) {
            return;
        }

        if (cachedAircraftState != null) {
            cachedAircraftState.originVelocityWorld = Vec3d.ZERO;
            cachedAircraftState.angularVelocityBody = Vec3d.ZERO;
            cachedAircraftState.hasPhysicalVelocity = true;
            cachedAircraftState.kinematics = null;
            cachedAircraftState.pendingSeparation.clear();
        }
        Vector3d safePosition;
        Quaterniond safeOrientation;
        String recoverySource;
        if (terrainResponse.hasClearTerrainPose
            && finite(terrainResponse.lastClearTerrainPositionWorld)
            && finite(terrainResponse.lastClearTerrainOrientationWorld)) {
            safePosition = new Vector3d(terrainResponse.lastClearTerrainPositionWorld);
            safeOrientation = new Quaterniond(terrainResponse.lastClearTerrainOrientationWorld).normalize();
            recoverySource = "LAST_VERIFIED_CLEAR_TERRAIN_POSE";
        } else if (terrainResponse.hasSubstepStartPose
            && finite(terrainResponse.substepStartPositionWorld)
            && finite(terrainResponse.substepStartOrientationWorld)) {
            safePosition = new Vector3d(terrainResponse.substepStartPositionWorld);
            safeOrientation = new Quaterniond(terrainResponse.substepStartOrientationWorld).normalize();
            recoverySource = "CURRENT_SUBSTEP_START_POSE";
        } else {
            safeOrientation = finite(cachedOrientation)
                ? new Quaterniond(cachedOrientation).normalize()
                : SablePoseConversions.toQuaternion(vehicle.orientation);
            if (!finite(safeOrientation)) {
                safeOrientation.identity();
            }
            Vector3d modelOrigin = new Vector3d(
                Double.isFinite(vehicle.position.x) ? vehicle.position.x : 0.0,
                Double.isFinite(vehicle.position.y) ? vehicle.position.y : 0.0,
                Double.isFinite(vehicle.position.z) ? vehicle.position.z : 0.0
            );
            safePosition = centerOfMassWorldFromModelOrigin(modelOrigin, safeOrientation);
            recoverySource = "FINITE_IV_MODEL_ORIGIN";
        }

        boolean nativeResetSucceeded = false;
        try {
            if (rigidBodyHandle != null && physicsSystem != null
                && rigidBodyHandle.isValid() && finite(safePosition) && finite(safeOrientation)) {
                placeBodyWithVelocity(
                    new Vector3d(), new Vector3d(), safeOrientation, safePosition
                );
                nativeResetSucceeded = true;
            }
        } catch (RuntimeException | Error ignored) {
            nativeResetSucceeded = false;
        }

        cachedExternalForceWorld.zero();
        cachedBallastForceWorldY = 0.0;
        cachedPropulsionForceBody.zero();
        cachedPropulsionTorqueBody.zero();
        cachedNetTorqueBody.zero();
        temporaryAppliedForceWorld.zero();
        temporaryNetTorqueBody.zero();
        terrainResponse.pendingSweptTerrainImpacts.clear();
        airVehicleSubstepAerodynamics = false;
        lastSubstepLandingGear = null;
        substepArmed = false;
        poisonReason = stage == null ? "unknown" : stage;
        poisoned = true;

        if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
            "SABLE_BODY_QUARANTINED uuid=" + vehicle.uniqueUUID
                + " stage=" + poisonReason
                + " recoverySource=" + recoverySource
                + " nativeFiniteReset=" + nativeResetSucceeded
                + " retireOnNextOwnerTick=true"
                + " sharedSceneProtected=true"
                + " fallbackNativeIvFlight=false"
        );
    }

    void placeBodyWithVelocity(
        Vector3dc linearVelocityWorld,
        Vector3dc angularVelocityWorld,
        Quaterniond orientation
    ) {
        placeBodyAtModelOriginWithVelocity(
            linearVelocityWorld,
            angularVelocityWorld,
            orientation,
            new Vector3d(vehicle.position.x, vehicle.position.y, vehicle.position.z)
        );
    }

    private void placeBodyAtModelOriginWithVelocity(
        Vector3dc linearVelocityWorld,
        Vector3dc angularVelocityWorld,
        Quaterniond orientation,
        Vector3dc modelOriginWorld
    ) {
        placeBodyWithVelocity(
            linearVelocityWorld, angularVelocityWorld, orientation,
            centerOfMassWorldFromModelOrigin(modelOriginWorld, orientation)
        );
    }

    private Vector3d centerOfMassWorldFromModelOrigin(
        Vector3dc modelOriginWorld, Quaterniond orientation
    ) {
        Vector3d offset = new Vector3d(
            centerOfMassLocal.x(), centerOfMassLocal.y(), centerOfMassLocal.z()
        );
        new Quaterniond(orientation).normalize().transform(offset);
        return new Vector3d(modelOriginWorld).add(offset);
    }

    private Vector3d modelOriginFromCenterOfMassWorld(
        Vector3dc centerOfMassWorld, Quaterniond orientation
    ) {
        Vector3d offset = new Vector3d(
            centerOfMassLocal.x(), centerOfMassLocal.y(), centerOfMassLocal.z()
        );
        new Quaterniond(orientation).normalize().transform(offset);
        return new Vector3d(centerOfMassWorld).sub(offset);
    }

    private static Vec3d toVec3d(Vector3dc value) {
        return value == null || !finite(value)
            ? Vec3d.ZERO
            : new Vec3d(value.x(), value.y(), value.z());
    }

    private Vector3d modelOriginVelocityFromCenterOfMassVelocity(
        Vector3dc centerOfMassVelocityWorld, Vector3dc angularVelocityWorld, Quaterniond orientation
    ) {
        Vector3d offset = new Vector3d(
            centerOfMassLocal.x(), centerOfMassLocal.y(), centerOfMassLocal.z()
        );
        new Quaterniond(orientation).normalize().transform(offset);
        Vector3d rotational = new Vector3d();
        new Vector3d(angularVelocityWorld).cross(offset, rotational);
        return new Vector3d(centerOfMassVelocityWorld).sub(rotational);
    }

    void placeBodyWithVelocity(
        Vector3dc linearVelocityWorld,
        Vector3dc angularVelocityWorld,
        Quaterniond orientation,
        Vector3dc worldPosition
    ) {
        if (rigidBodyHandle == null || physicsSystem == null) {
            return;
        }
        rigidBodyHandle.teleport(worldPosition, orientation);
        updatePose();
        physicsSystem.getPipeline().resetVelocity(this);
        rigidBodyHandle.addLinearAndAngularVelocity(
            linearVelocityWorld,
            angularVelocityWorld
        );
    }

    /**
     * Resolves the one-time placement pose before the persistent Sable body is
     * allowed to integrate. A freshly player-placed aircraft may arrive from IV
     * with pitch/roll already biased by its pre-Sable ground pass. If that pose
     * rotates the authored tire stations away from the surface they were placed
     * over, a translation-only seating pass cannot recover: the wheel rays miss
     * terrain and BODY becomes the first Rapier support.
     *
     * <p>For a genuinely fresh player placement only, reconstruct the natural
     * resting attitude from two pieces of content-independent geometry: the
     * aircraft's own active tire-bottom stations and the terrain plane sampled
     * beneath the same footprint with heading preserved. Taildraggers therefore
     * keep their natural nose-up gear attitude, tricycle aircraft keep whatever
     * pitch their wheel heights imply, and sloped terrain remains sloped. This is
     * a one-time spawn transform, not a level target or ongoing fake physics.</p>
     */
    private InitialPlacementPose initialGroundSeatedPose(
        Vector3dc requestedPosition,
        Quaterniond requestedOrientation,
        Vector3dc initialLinearVelocityWorld,
        Vector3dc initialAngularVelocityWorld
    ) {
        Vector3d position = new Vector3d(requestedPosition);
        Quaterniond orientation = new Quaterniond(requestedOrientation).normalize();
        String reconstructionSkipReason = initialGroundPoseReconstructionSkipReason(
            position, orientation, initialLinearVelocityWorld, initialAngularVelocityWorld
        );
        if (reconstructionSkipReason == null) {
            InitialPlacementPose reconstructed = initialFreshPlacementSupportPlanePose(
                position, orientation
            );
            if (reconstructed != null) {
                position.set(reconstructed.positionWorld());
                orientation.set(reconstructed.orientationWorld()).normalize();
            }
        } else {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_INITIAL_GROUND_POSE_RECONSTRUCTION uuid=" + vehicle.uniqueUUID
                    + " applied=false reason=" + reconstructionSkipReason
                    + " initialGroundSeatingAllowed=" + initialGroundSeatingAllowed
                    + " ticksExisted=" + vehicle.ticksExisted
                    + " placingPlayerPresent=" + placingPlayerPresent()
                    + " initialLinearSpeedMps=" + vectorLengthOrNaN(initialLinearVelocityWorld)
                    + " initialAngularSpeedRadps=" + vectorLengthOrNaN(initialAngularVelocityWorld)
                    + " firstSableActivation=true"
                    + " diagnosticOnly=true"
            );
        }
        position = initialGroundSeatedPosition(
            position, orientation, initialLinearVelocityWorld, initialAngularVelocityWorld
        );
        return ensureInitialPlacementTerrainClear(
            new InitialPlacementPose(position, orientation, false)
        );
    }

    /**
     * Defensive one-time spawn invariant: dynamic Rapier never receives a freshly
     * placed aircraft whose complete BODY compound is already inside terrain. The
     * normal support-plane reconstruction and seating passes run first; this path
     * is reached only when those could not produce a clear pose. It does not trim,
     * level, damp, or persistently hold the aircraft.
     */
    private InitialPlacementPose ensureInitialPlacementTerrainClear(InitialPlacementPose pose) {
        if (pose == null || !isFreshPlayerPlacement() || compoundCollider == null
            || pose.positionWorld() == null || pose.orientationWorld() == null
            || !finite(pose.positionWorld()) || !finite(pose.orientationWorld())) {
            return pose;
        }
        Quaterniond orientation = new Quaterniond(pose.orientationWorld()).normalize();
        Vector3d original = new Vector3d(pose.positionWorld());
        Vector3d originalCom = centerOfMassWorldFromModelOrigin(original, orientation);
        if (!compoundCollider.bodyIntersectsTerrainAtPose(level, originalCom, orientation)) {
            return pose;
        }

        double maximumRaise = Math.max(
            INITIAL_GROUND_SEATING_MAX_DROP_METERS,
            2.0 * Math.max(0.0, compoundCollider.collisionBoundingRadius()) + 1.0
        );
        double collidingRaise = 0.0;
        double clearRaise = Double.NaN;
        for (double raise = INITIAL_TERRAIN_CLEARANCE_STEP_METERS;
             raise <= maximumRaise + 1.0E-9;
             raise += INITIAL_TERRAIN_CLEARANCE_STEP_METERS) {
            Vector3d candidate = new Vector3d(original).add(0.0, raise, 0.0);
            Vector3d candidateCom = centerOfMassWorldFromModelOrigin(candidate, orientation);
            if (!compoundCollider.bodyIntersectsTerrainAtPose(level, candidateCom, orientation)) {
                clearRaise = raise;
                break;
            }
            collidingRaise = raise;
        }
        if (!Double.isFinite(clearRaise)) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_INITIAL_TERRAIN_CLEARANCE_FALLBACK uuid=" + vehicle.uniqueUUID
                    + " applied=false reason=NO_CLEAR_POSE_WITHIN_BOUND"
                    + " maxRaiseM=" + maximumRaise
                    + " firstSableActivation=true"
                    + " dynamicOverlapGuard=true"
            );
            return pose;
        }

        double low = collidingRaise;
        double high = clearRaise;
        for (int iteration = 0; iteration < INITIAL_TERRAIN_CLEARANCE_BINARY_STEPS; ++iteration) {
            double middle = 0.5 * (low + high);
            Vector3d candidate = new Vector3d(original).add(0.0, middle, 0.0);
            Vector3d candidateCom = centerOfMassWorldFromModelOrigin(candidate, orientation);
            if (compoundCollider.bodyIntersectsTerrainAtPose(level, candidateCom, orientation)) {
                low = middle;
            } else {
                high = middle;
            }
        }
        double appliedRaise = high + INITIAL_GROUND_SEATING_TARGET_GAP_METERS;
        Vector3d clearPosition = new Vector3d(original).add(0.0, appliedRaise, 0.0);
        if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
            "SABLE_INITIAL_TERRAIN_CLEARANCE_FALLBACK uuid=" + vehicle.uniqueUUID
                + " applied=true"
                + " raiseM=" + appliedRaise
                + " firstSableActivation=true"
                + " initialVelocityReset=true"
                + " dynamicOverlapGuard=true"
                + " persistentHold=false"
                + " packSpecific=false"
        );
        return new InitialPlacementPose(clearPosition, orientation, true);
    }

    /** Returns null only when one-time fresh-placement attitude reconstruction is eligible. */
    private @Nullable String initialGroundPoseReconstructionSkipReason(
        Vector3dc position,
        Quaterniond orientation,
        Vector3dc initialLinearVelocityWorld,
        Vector3dc initialAngularVelocityWorld
    ) {
        if (!initialGroundSeatingAllowed) {
            return "INITIAL_GROUND_SEATING_NOT_ALLOWED";
        }
        if (vehicle.ticksExisted < 0L
            || vehicle.ticksExisted > INITIAL_GROUND_POSE_MAX_PLACED_TICKS) {
            return "OUTSIDE_FRESH_PLACEMENT_TICK_WINDOW";
        }
        if (!placingPlayerPresent()) {
            return "NO_PLACING_PLAYER";
        }
        if (!finite(position) || !finite(orientation)
            || !finite(initialLinearVelocityWorld) || !finite(initialAngularVelocityWorld)) {
            return "NONFINITE_INITIAL_STATE";
        }
        if (initialLinearVelocityWorld.length() > INITIAL_GROUND_SEATING_MAX_LINEAR_SPEED_MPS) {
            return "INITIAL_LINEAR_SPEED_TOO_HIGH";
        }
        if (initialAngularVelocityWorld.length() > INITIAL_GROUND_SEATING_MAX_ANGULAR_SPEED_RADPS) {
            return "INITIAL_ANGULAR_SPEED_TOO_HIGH";
        }
        return null;
    }

    private boolean placingPlayerPresent() {
        try {
            return ((EntityVehicleMovingAccessor) (Object) vehicle)
                .pmweatherIv$getPlacingPlayer() != null;
        } catch (RuntimeException | LinkageError unavailable) {
            return false;
        }
    }

    private static double vectorLengthOrNaN(Vector3dc vector) {
        return vector != null && finite(vector) ? vector.length() : Double.NaN;
    }

    private @Nullable InitialPlacementPose initialFreshPlacementSupportPlanePose(
        Vector3dc requestedPosition,
        Quaterniond requestedOrientation
    ) {
        List<Vec3d> supports = initialPlacementSupportPoints();
        if (supports.size() < 3) {
            return null;
        }

        List<Vector3d> localSupportVectors = new ArrayList<>(supports.size());
        for (Vec3d support : supports) {
            localSupportVectors.add(new Vector3d(support.x(), support.y(), support.z()));
        }
        Vector3d localSupportNormal = widestTrianglePlaneNormal(localSupportVectors);
        if (localSupportNormal == null || localSupportNormal.y < 0.35) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_INITIAL_GROUND_POSE_RECONSTRUCTION uuid=" + vehicle.uniqueUUID
                    + " applied=false reason=DEGENERATE_SUPPORT_PLANE"
                    + " supportPoints=" + supports.size()
                    + " firstSableActivation=true"
            );
            return null;
        }

        // Preserve the heading that IV supplied, but intentionally discard its
        // pitch/roll only for the terrain-probe footprint. This lets a bad
        // pre-Sable lean no longer move all wheel probes off the runway.
        Vector3d headingWorld = new Vector3d(0.0, 0.0, 1.0);
        requestedOrientation.transform(headingWorld);
        headingWorld.y = 0.0;
        if (headingWorld.lengthSquared() <= 1.0E-10) {
            return null;
        }
        headingWorld.normalize();
        Vector3d worldUp = new Vector3d(0.0, 1.0, 0.0);
        Vector3d headingRight = new Vector3d(worldUp).cross(headingWorld).normalize();
        Quaterniond headingOnlyOrientation = orientationFromBasis(
            headingRight, worldUp, headingWorld
        );

        List<Vector3d> sampledTerrainPoints = new ArrayList<>(supports.size());
        Vector3d original = new Vector3d(requestedPosition);
        for (Vec3d support : supports) {
            Vector3d leverWorld = new Vector3d(support.x(), support.y(), support.z());
            headingOnlyOrientation.transform(leverWorld);
            Vector3d probeWorld = new Vector3d(original).add(leverWorld);
            double supportTop = liveTerrainSupportTop(
                probeWorld.x, probeWorld.y, probeWorld.z,
                INITIAL_GROUND_SEATING_MAX_DROP_METERS
            );
            if (Double.isFinite(supportTop)) {
                sampledTerrainPoints.add(new Vector3d(
                    probeWorld.x, supportTop, probeWorld.z
                ));
            }
        }
        if (sampledTerrainPoints.size() < 3) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_INITIAL_GROUND_POSE_RECONSTRUCTION uuid=" + vehicle.uniqueUUID
                    + " applied=false reason=INSUFFICIENT_TERRAIN_SUPPORT"
                    + " supportPoints=" + supports.size()
                    + " terrainBackedPoints=" + sampledTerrainPoints.size()
                    + " firstSableActivation=true"
            );
            return null;
        }

        Vector3d terrainNormal = widestTrianglePlaneNormal(sampledTerrainPoints);
        if (terrainNormal == null || terrainNormal.y < INITIAL_GROUND_POSE_MIN_TERRAIN_NORMAL_Y) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_INITIAL_GROUND_POSE_RECONSTRUCTION uuid=" + vehicle.uniqueUUID
                    + " applied=false reason=TERRAIN_TOO_STEEP_OR_DEGENERATE"
                    + " supportPoints=" + supports.size()
                    + " terrainBackedPoints=" + sampledTerrainPoints.size()
                    + " terrainNormal=" + vectorText(terrainNormal)
                    + " firstSableActivation=true"
            );
            return null;
        }

        Vector3d localForward = new Vector3d(0.0, 0.0, 1.0);
        localForward.sub(new Vector3d(localSupportNormal).mul(
            localForward.dot(localSupportNormal)
        ));
        if (localForward.lengthSquared() <= 1.0E-10) {
            return null;
        }
        localForward.normalize();
        Vector3d localRight = new Vector3d(localSupportNormal)
            .cross(localForward).normalize();
        localForward.set(new Vector3d(localRight).cross(localSupportNormal)).normalize();

        Vector3d desiredWorldForward = new Vector3d(headingWorld);
        desiredWorldForward.sub(new Vector3d(terrainNormal).mul(
            desiredWorldForward.dot(terrainNormal)
        ));
        if (desiredWorldForward.lengthSquared() <= 1.0E-10) {
            return null;
        }
        desiredWorldForward.normalize();
        Vector3d desiredWorldRight = new Vector3d(terrainNormal)
            .cross(desiredWorldForward).normalize();
        desiredWorldForward.set(
            new Vector3d(desiredWorldRight).cross(terrainNormal)
        ).normalize();

        Matrix3d localBasis = basisMatrix(localRight, localSupportNormal, localForward);
        Matrix3d worldBasis = basisMatrix(
            desiredWorldRight, terrainNormal, desiredWorldForward
        );
        Matrix3d desiredRotation = new Matrix3d(worldBasis)
            .mul(new Matrix3d(localBasis).transpose());
        Quaterniond desiredOrientation = new Quaterniond()
            .setFromNormalized(desiredRotation).normalize();
        if (!finite(desiredOrientation)) {
            return null;
        }

        double quaternionDot = Math.abs(new Quaterniond(requestedOrientation)
            .normalize().dot(desiredOrientation));
        quaternionDot = Math.max(-1.0, Math.min(1.0, quaternionDot));
        double repairAngleDegrees = Math.toDegrees(2.0 * Math.acos(quaternionDot));
        if (!Double.isFinite(repairAngleDegrees)
            || repairAngleDegrees > INITIAL_GROUND_POSE_MAX_ATTITUDE_REPAIR_DEGREES) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_INITIAL_GROUND_POSE_RECONSTRUCTION uuid=" + vehicle.uniqueUUID
                    + " applied=false reason=ATTITUDE_REPAIR_BOUND"
                    + " repairAngleDeg=" + repairAngleDegrees
                    + " maxRepairAngleDeg=" + INITIAL_GROUND_POSE_MAX_ATTITUDE_REPAIR_DEGREES
                    + " firstSableActivation=true"
            );
            return null;
        }

        // Re-sample under the corrected support locations and choose the minimum
        // vertical placement that keeps every backed wheel on/above its local
        // surface. At least three stations must still resolve actual collision
        // geometry after the attitude change.
        double requiredBodyY = Double.NEGATIVE_INFINITY;
        int correctedTerrainBackedPoints = 0;
        for (Vec3d support : supports) {
            Vector3d leverWorld = new Vector3d(support.x(), support.y(), support.z());
            desiredOrientation.transform(leverWorld);
            Vector3d probeWorld = new Vector3d(original).add(leverWorld);
            double supportTop = liveTerrainSupportTop(
                probeWorld.x, probeWorld.y, probeWorld.z,
                INITIAL_GROUND_SEATING_MAX_DROP_METERS
            );
            if (!Double.isFinite(supportTop)) {
                continue;
            }
            requiredBodyY = Math.max(
                requiredBodyY,
                supportTop + INITIAL_GROUND_SEATING_TARGET_GAP_METERS - leverWorld.y
            );
            ++correctedTerrainBackedPoints;
        }
        if (!Double.isFinite(requiredBodyY) || correctedTerrainBackedPoints < 3) {
            return null;
        }

        double verticalShift = requiredBodyY - original.y;
        if (Math.abs(verticalShift) > INITIAL_GROUND_SEATING_MAX_DROP_METERS) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_INITIAL_GROUND_POSE_RECONSTRUCTION uuid=" + vehicle.uniqueUUID
                    + " applied=false reason=VERTICAL_REPAIR_BOUND"
                    + " verticalShiftM=" + verticalShift
                    + " maxShiftM=" + INITIAL_GROUND_SEATING_MAX_DROP_METERS
                    + " firstSableActivation=true"
            );
            return null;
        }
        Vector3d candidatePosition = new Vector3d(original.x, requiredBodyY, original.z);
        if (compoundCollider.bodyIntersectsTerrainAtPose(
            level, centerOfMassWorldFromModelOrigin(candidatePosition, desiredOrientation),
            desiredOrientation
        )) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_INITIAL_GROUND_POSE_RECONSTRUCTION uuid=" + vehicle.uniqueUUID
                    + " applied=false reason=BODY_COLLISION_AT_RECONSTRUCTED_POSE"
                    + " repairAngleDeg=" + repairAngleDegrees
                    + " verticalShiftM=" + verticalShift
                    + " firstSableActivation=true"
            );
            return null;
        }

        if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
            "SABLE_INITIAL_GROUND_POSE_RECONSTRUCTION uuid=" + vehicle.uniqueUUID
                + " applied=true"
                + " supportPoints=" + supports.size()
                + " terrainBackedPoints=" + correctedTerrainBackedPoints
                + " repairAngleDeg=" + repairAngleDegrees
                + " verticalShiftM=" + verticalShift
                + " localSupportNormal=" + vectorText(localSupportNormal)
                + " terrainNormal=" + vectorText(terrainNormal)
                + " headingPreserved=true"
                + " firstSableActivation=true"
                + " persistentAttitudeTarget=false"
                + " packSpecific=false"
        );
        return new InitialPlacementPose(candidatePosition, desiredOrientation, false);
    }

    private static @Nullable Vector3d widestTrianglePlaneNormal(List<Vector3d> points) {
        if (points == null || points.size() < 3) {
            return null;
        }
        double bestAreaSquared = 0.0;
        Vector3d bestNormal = null;
        for (int first = 0; first < points.size() - 2; ++first) {
            Vector3d a = points.get(first);
            for (int second = first + 1; second < points.size() - 1; ++second) {
                Vector3d ab = new Vector3d(points.get(second)).sub(a);
                for (int third = second + 1; third < points.size(); ++third) {
                    Vector3d ac = new Vector3d(points.get(third)).sub(a);
                    Vector3d normal = new Vector3d(ab).cross(ac);
                    double areaSquared = normal.lengthSquared();
                    if (Double.isFinite(areaSquared) && areaSquared > bestAreaSquared) {
                        bestAreaSquared = areaSquared;
                        bestNormal = normal;
                    }
                }
            }
        }
        if (bestNormal == null
            || bestAreaSquared < INITIAL_GROUND_POSE_MIN_SUPPORT_TRIANGLE_AREA2) {
            return null;
        }
        bestNormal.normalize();
        if (bestNormal.y < 0.0) {
            bestNormal.negate();
        }
        return bestNormal;
    }

    private static Matrix3d basisMatrix(
        Vector3dc right,
        Vector3dc up,
        Vector3dc forward
    ) {
        Matrix3d basis = new Matrix3d();
        basis.setColumn(0, right);
        basis.setColumn(1, up);
        basis.setColumn(2, forward);
        return basis;
    }

    private static Quaterniond orientationFromBasis(
        Vector3dc right,
        Vector3dc up,
        Vector3dc forward
    ) {
        return new Quaterniond().setFromNormalized(
            basisMatrix(right, up, forward)
        ).normalize();
    }

    private static String vectorText(@Nullable Vector3dc vector) {
        return vector == null
            ? "null"
            : String.format("(%.5f,%.5f,%.5f)", vector.x(), vector.y(), vector.z());
    }

    /**
     * Seats only the first Sable activation of a newly spawned aircraft onto the
     * closest authored ground-support station. IV normally performs a placement
     * ground pass before ordinary motion; PMIV bypasses that physical solver, so
     * without this one-time placement step a vehicle may free-fall 1-2 blocks
     * before its wheels enter the 0.05 m live-contact search and can push the
     * spawning player into terrain. This is not an ongoing ride-height target:
     * after activation, all support remains the normal unilateral substep solver.
     */
    private Vector3d initialGroundSeatedPosition(
        Vector3dc requestedPosition,
        Quaterniond requestedOrientation,
        Vector3dc initialLinearVelocityWorld,
        Vector3dc initialAngularVelocityWorld
    ) {
        Vector3d original = new Vector3d(requestedPosition);
        if (!initialGroundSeatingAllowed || compoundCollider == null
            || requestedOrientation == null || !finite(requestedOrientation)
            || !finite(original)
            || !finite(initialLinearVelocityWorld) || !finite(initialAngularVelocityWorld)
            || initialLinearVelocityWorld.length() > INITIAL_GROUND_SEATING_MAX_LINEAR_SPEED_MPS
            || initialAngularVelocityWorld.length() > INITIAL_GROUND_SEATING_MAX_ANGULAR_SPEED_RADPS) {
            return original;
        }

        List<Vec3d> supports = initialPlacementSupportPoints();
        if (supports.isEmpty()) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_INITIAL_GROUND_SEATING uuid=" + vehicle.uniqueUUID
                    + " applied=false reason=NO_SUPPORT_POINTS"
                    + " firstSableActivation=true"
            );
            return original;
        }

        double minimumGap = Double.POSITIVE_INFINITY;
        int terrainBackedPoints = 0;
        for (Vec3d pointLocal : supports) {
            Vector3d leverWorld = new Vector3d(
                pointLocal.x(), pointLocal.y(), pointLocal.z()
            );
            requestedOrientation.transform(leverWorld);
            Vector3d pointWorld = new Vector3d(original).add(leverWorld);
            double supportTop = liveTerrainSupportTop(
                pointWorld.x, pointWorld.y, pointWorld.z,
                INITIAL_GROUND_SEATING_MAX_DROP_METERS
            );
            if (!Double.isFinite(supportTop)) {
                continue;
            }
            double gap = pointWorld.y - supportTop;
            // If any support is already at/below terrain, do not lower the body.
            if (gap < -LIVE_GEAR_CONTACT_SKIN_METERS) {
                if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                    "SABLE_INITIAL_GROUND_SEATING uuid=" + vehicle.uniqueUUID
                        + " applied=false reason=SUPPORT_ALREADY_PENETRATING"
                        + " supportGapM=" + gap
                        + " supportPoints=" + supports.size()
                );
                return original;
            }
            if (gap >= -LIVE_GEAR_CONTACT_SKIN_METERS
                && gap <= INITIAL_GROUND_SEATING_MAX_DROP_METERS) {
                minimumGap = Math.min(minimumGap, Math.max(0.0, gap));
                ++terrainBackedPoints;
            }
        }
        if (!Double.isFinite(minimumGap) || terrainBackedPoints == 0) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_INITIAL_GROUND_SEATING uuid=" + vehicle.uniqueUUID
                    + " applied=false reason=NO_TERRAIN_WITHIN_SEARCH"
                    + " maxDropM=" + INITIAL_GROUND_SEATING_MAX_DROP_METERS
                    + " supportPoints=" + supports.size()
            );
            return original;
        }

        // The first Sable physics cycle is deliberately collision-only. That means
        // gravity is not yet available to turn a small positive wheel gap into a
        // predictive crossing. "Already seated" must therefore mean the wheel is
        // actually inside the same 10 mm continuous-contact skin used by the live
        // substep gear solver. The former 25 mm minimum placement move created a
        // dead zone: e.g. an observed 13.25 mm gear gap was skipped here, then
        // BODY could become the first runway contact before gear ever carried load.
        if (minimumGap <= LIVE_GEAR_CONTACT_SKIN_METERS) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_INITIAL_GROUND_SEATING uuid=" + vehicle.uniqueUUID
                    + " applied=false reason=ALREADY_IN_LIVE_GEAR_SKIN"
                    + " nearestGapM=" + minimumGap
                    + " liveGearSkinM=" + LIVE_GEAR_CONTACT_SKIN_METERS
                    + " supportPoints=" + supports.size()
                    + " terrainBackedPoints=" + terrainBackedPoints
            );
            return original;
        }

        double requestedDrop = Math.max(
            0.0, minimumGap - INITIAL_GROUND_SEATING_TARGET_GAP_METERS
        );
        if (requestedDrop < INITIAL_GROUND_SEATING_MIN_DROP_METERS) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_INITIAL_GROUND_SEATING uuid=" + vehicle.uniqueUUID
                    + " applied=false reason=NUMERICAL_SEATING_EPSILON"
                    + " nearestGapM=" + minimumGap
                    + " requestedDropM=" + requestedDrop
                    + " minimumDropM=" + INITIAL_GROUND_SEATING_MIN_DROP_METERS
                    + " supportPoints=" + supports.size()
                    + " terrainBackedPoints=" + terrainBackedPoints
            );
            return original;
        }

        // Never teleport BODY collision through terrain just to make a wheel/skid
        // touch. If the requested seat is body-limited (for example on a slope),
        // binary-search the largest clear downward shift and let Rapier/gear finish
        // the remaining ordinary settlement physically.
        double clearDrop = requestedDrop;
        Vector3d candidate = new Vector3d(original).add(0.0, -clearDrop, 0.0);
        boolean bodyLimited = compoundCollider.bodyIntersectsTerrainAtPose(
            level, centerOfMassWorldFromModelOrigin(candidate, requestedOrientation),
            requestedOrientation
        );
        if (bodyLimited) {
            double low = 0.0;
            double high = requestedDrop;
            for (int iteration = 0; iteration < 16; ++iteration) {
                double middle = 0.5 * (low + high);
                candidate.set(original).add(0.0, -middle, 0.0);
                if (compoundCollider.bodyIntersectsTerrainAtPose(
                    level, centerOfMassWorldFromModelOrigin(candidate, requestedOrientation),
                    requestedOrientation
                )) {
                    high = middle;
                } else {
                    low = middle;
                }
            }
            clearDrop = low;
        }
        if (clearDrop < INITIAL_GROUND_SEATING_MIN_DROP_METERS) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_INITIAL_GROUND_SEATING uuid=" + vehicle.uniqueUUID
                    + " applied=false reason=BODY_COLLISION_LIMIT"
                    + " requestedDropM=" + requestedDrop
                    + " clearDropM=" + clearDrop
                    + " supportPoints=" + supports.size()
            );
            return original;
        }

        Vector3d seated = new Vector3d(original).add(0.0, -clearDrop, 0.0);
        if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
            "SABLE_INITIAL_GROUND_SEATING uuid=" + vehicle.uniqueUUID
                + " applied=true"
                + " requestedDropM=" + requestedDrop
                + " appliedDropM=" + clearDrop
                + " nearestInitialGapM=" + minimumGap
                + " targetGapM=" + INITIAL_GROUND_SEATING_TARGET_GAP_METERS
                + " bodyLimited=" + bodyLimited
                + " supportPoints=" + supports.size()
                + " terrainBackedPoints=" + terrainBackedPoints
                + " firstSableActivation=true"
                + " persistentRideHeightTarget=false"
        );
        return seated;
    }

    /**
     * IV part activity is computed during the part update. On the very first
     * server tick of a player placement, installed landing-gear parts can still
     * report part_active=false even though the vehicle-wide gear command is the
     * normal down state. Permit those valid installed devices only across this
     * bounded creation boundary. Saved vehicles, later gameplay, invalid parts,
     * spares, and gear-commanded-up aircraft never receive this exception.
     */
    private boolean isFreshPlayerPlacement() {
        return initialGroundSeatingAllowed
            && vehicle.ticksExisted >= 0L
            && vehicle.ticksExisted <= INITIAL_GROUND_POSE_MAX_PLACED_TICKS
            && placingPlayerPresent();
    }

    private boolean allowsTransientFreshPlacementGear() {
        return isFreshPlayerPlacement()
            && vehicle.retractGearVar != null
            && !vehicle.retractGearVar.isActive;
    }

    private boolean allowsTransientFreshPlacementGroundDevice(PartGroundDevice device) {
        if (!allowsTransientFreshPlacementGear() || device == null) {
            return false;
        }
        return device.placementDefinition == null
            || device.placementDefinition.activeAnimations == null
            || device.placementDefinition.activeAnimations.isEmpty();
    }

    /**
     * Placement may use installed active gear before IV's transient isValid flag
     * has completed its first update; the authored local station already exists.
     * Normal per-substep gear authority still requires isValid.
     */
    private List<Vec3d> initialPlacementSupportPoints() {
        List<Vec3d> points = new ArrayList<>();
        int transientWheelCount = 0;
        if (vehicle.allParts != null) {
            for (APart part : vehicle.allParts) {
                if (!(part instanceof PartGroundDevice device)
                    || device.isSpare
                    || (!device.isActiveVar.isActive
                        && !allowsTransientFreshPlacementGroundDevice(device))) {
                    continue;
                }
                if (!device.isActiveVar.isActive) {
                    ++transientWheelCount;
                }
                Vec3d point = LandingGearSolver.landingGearDeviceContactPointLocal(device, temporaryPhysicalOrientation);
                if (point != null && point.isFinite()) {
                    addUniqueInitialSupportPoint(points, point);
                }
            }
        }
        if (transientWheelCount > 0 && !loggedTransientFreshPlacementGear) {
            loggedTransientFreshPlacementGear = true;
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "SABLE_FRESH_PLACEMENT_TRANSIENT_GEAR uuid=" + vehicle.uniqueUUID
                    + " transientWheelCount=" + transientWheelCount
                    + " supportPointCount=" + points.size()
                    + " gearCommandedDown=true"
                    + " scope=FRESH_PLAYER_PLACEMENT_INITIALIZATION_ONLY"
                    + " packSpecific=false"
            );
        }
        if (points.isEmpty() && compoundCollider != null) {
            for (SableModelCollisionHull.StaticSupportPoint support
                : compoundCollider.staticGroundSupportPoints()) {
                addUniqueInitialSupportPoint(
                    points, new Vec3d(support.x(), support.y(), support.z())
                );
            }
        }
        return points;
    }

    private static void addUniqueInitialSupportPoint(List<Vec3d> points, Vec3d candidate) {
        for (Vec3d existing : points) {
            if (existing.subtract(candidate).lengthSquared() <= 1.0E-8) {
                return;
            }
        }
        points.add(candidate);
    }

    private void applyExactMassProperties() {
        double[] inertiaTensor = new double[]{
            desiredInertia.x(), 0.0, 0.0,
            0.0, desiredInertia.y(), 0.0,
            0.0, 0.0, desiredInertia.z()
        };
        Rapier3DInvoker.pmweatherIv$setMassProperties(
            sceneHandle,
            getRuntimeId(),
            mass,
            new double[]{0.0, 0.0, 0.0},
            inertiaTensor
        );
    }

    private static Pose3d initialPose(
        EntityVehicleF_Physics vehicle, @Nullable BodyState initialState, Vec3d centerOfMassLocal
    ) {
        Pose3d pose = new Pose3d();
        if (initialState != null
            && initialState.positionWorld() != null
            && finite(initialState.positionWorld())
            && initialState.orientationWorld() != null
            && finite(initialState.orientationWorld())) {
            // BodyState is already expressed at the physical COM.
            pose.position().set(initialState.positionWorld());
            pose.orientation().set(initialState.orientationWorld()).normalize();
        } else {
            Quaterniond orientation = SablePoseConversions.toQuaternion(vehicle.orientation);
            Vector3d offset = new Vector3d(
                centerOfMassLocal == null ? 0.0 : centerOfMassLocal.x(),
                centerOfMassLocal == null ? 0.0 : centerOfMassLocal.y(),
                centerOfMassLocal == null ? 0.0 : centerOfMassLocal.z()
            );
            orientation.transform(offset);
            pose.position().set(
                vehicle.position.x + offset.x,
                vehicle.position.y + offset.y,
                vehicle.position.z + offset.z
            );
            pose.orientation().set(orientation).normalize();
        }
        return pose;
    }

    private static Vector3d modelHalfExtents(AirframeLoads.SolveResult result) {
        ModelSurfaceMap.Bounds bounds = result.model() == null
            ? null
            : result.model().fullBounds();
        if (bounds == null || !bounds.valid()) {
            AirframeGeometry.Geometry geometry = result.geometry();
            return new Vector3d(
                clampHalf(geometry.bodyWidth() * 0.5),
                clampHalf(geometry.bodyHeight() * 0.5),
                clampHalf(geometry.bodyLength() * 0.5)
            );
        }
        return new Vector3d(
            clampHalf(bounds.spanX() * 0.5),
            clampHalf(bounds.spanY() * 0.5),
            clampHalf(bounds.spanZ() * 0.5)
        );
    }


    static boolean finite(Vector3dc value) {
        return value != null
            && Double.isFinite(value.x())
            && Double.isFinite(value.y())
            && Double.isFinite(value.z());
    }

    static boolean finite(Quaterniondc value) {
        return value != null
            && Double.isFinite(value.x())
            && Double.isFinite(value.y())
            && Double.isFinite(value.z())
            && Double.isFinite(value.w())
            && value.lengthSquared() > 1.0E-18;
    }

    private static double clampHalf(double value) {
        return Math.max(MIN_HALF_EXTENT, Math.min(MAX_HALF_EXTENT, Math.abs(value)));
    }

    private static Vec3d sanitizeInertia(Vec3d inertia) {
        if (inertia == null || !inertia.isFinite()) {
            return new Vec3d(1.0, 1.0, 1.0);
        }
        return new Vec3d(
            Math.max(MIN_INERTIA, inertia.x()),
            Math.max(MIN_INERTIA, inertia.y()),
            Math.max(MIN_INERTIA, inertia.z())
        );
    }

    private static double relativeDifference(double first, double second) {
        return Math.abs(first - second)
            / Math.max(1.0, Math.max(Math.abs(first), Math.abs(second)));
    }

    private static String safe(String value) {
        return value == null ? "null" : value.replace('\n', ' ');
    }

    /** Begins a short, momentum-preserving aircraft hold while IV repairs a saved rider link. */
    public boolean beginRiderReconnectHold(UUID playerUuid) {
        if (!isUsable()) {
            return false;
        }
        if (riderReconnectHeld) {
            return riderReconnectPlayerUuid == null || playerUuid == null
                || riderReconnectPlayerUuid.equals(playerUuid);
        }

        updatePose();
        rigidBodyHandle.getLinearVelocity(riderReconnectSavedLinearVelocityWorld);
        rigidBodyHandle.getAngularVelocity(riderReconnectSavedAngularVelocityWorld);
        riderReconnectHoldPositionWorld.set(getPose().position());
        riderReconnectHoldOrientationWorld.set(getPose().orientation()).normalize();
        if (!finite(riderReconnectSavedLinearVelocityWorld)
            || !finite(riderReconnectSavedAngularVelocityWorld)
            || !finite(riderReconnectHoldPositionWorld)
            || !finite(riderReconnectHoldOrientationWorld)) {
            return false;
        }

        riderReconnectHeld = true;
        riderReconnectPlayerUuid = playerUuid;
        riderReconnectHoldStartGameTime = level.getGameTime();
        placeBodyWithVelocity(
            new Vector3d(),
            new Vector3d(),
            new Quaterniond(riderReconnectHoldOrientationWorld),
            new Vector3d(riderReconnectHoldPositionWorld)
        );
        if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
            "PMIV_RIDER_RECONNECT_HOLD_BEGIN gameTime=" + riderReconnectHoldStartGameTime
                + " uuid=" + vehicle.uniqueUUID
                + " playerUuid=" + String.valueOf(playerUuid)
                + " savedLinearVelocityMps=" + riderReconnectSavedLinearVelocityWorld
                + " savedAngularVelocityRadps=" + riderReconnectSavedAngularVelocityWorld
                + " holdPose=" + riderReconnectHoldPositionWorld
                + " momentumPolicy=PRESERVE_AND_RESTORE_EXACT_SABLE_STATE"
        );
        return true;
    }

    /** Restores the exact pre-hold Sable momentum after a rider reconnect is verified. */
    public void endRiderReconnectHold(UUID playerUuid, String reason) {
        if (!riderReconnectHeld) {
            return;
        }
        if (riderReconnectPlayerUuid != null && playerUuid != null
            && !riderReconnectPlayerUuid.equals(playerUuid)) {
            return;
        }

        long now = level.getGameTime();
        Vector3d restoredLinear = new Vector3d(riderReconnectSavedLinearVelocityWorld);
        Vector3d restoredAngular = new Vector3d(riderReconnectSavedAngularVelocityWorld);
        Quaterniond restoredOrientation = new Quaterniond(riderReconnectHoldOrientationWorld);
        Vector3d restoredPosition = new Vector3d(riderReconnectHoldPositionWorld);
        if (isUsable()) {
            placeBodyWithVelocity(
                restoredLinear, restoredAngular, restoredOrientation, restoredPosition
            );
            // Contact rows produced while the body was intentionally pinned are
            // lifecycle artifacts, not a new crash episode. Consume them here so
            // they cannot become delayed IV gameplay damage after release.
            lastProcessedCollisionGeneration = SableCollisionCapture.snapshot(sceneHandle).generation();
            terrainResponse.drainSweptTerrainImpacts();
        }

        // Loads sampled while the body was held describe the temporary zero-speed
        // presentation state. Force the next substep through the existing stale-load
        // momentum-preserving path until the next owner tick supplies fresh aero.
        lastLoadsGameTime = Long.MIN_VALUE;
        airVehicleSubstepAerodynamics = false;
        lastSubstepLandingGear = null;

        if (cachedAircraftState != null) {
            Vector3d modelOriginVelocity = modelOriginVelocityFromCenterOfMassVelocity(
                restoredLinear, restoredAngular, restoredOrientation
            );
            Vector3d angularBody = SablePoseConversions.worldToLocal(
                restoredOrientation, restoredAngular, new Vector3d()
            );
            cachedAircraftState.originVelocityWorld = new Vec3d(
                modelOriginVelocity.x, modelOriginVelocity.y, modelOriginVelocity.z
            );
            cachedAircraftState.angularVelocityBody = new Vec3d(
                angularBody.x, angularBody.y, angularBody.z
            );
            cachedAircraftState.hasPhysicalVelocity = true;
            cachedAircraftState.kinematics = null;
        }

        if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
            "PMIV_RIDER_RECONNECT_HOLD_END gameTime=" + now
                + " uuid=" + vehicle.uniqueUUID
                + " playerUuid=" + String.valueOf(playerUuid)
                + " heldTicks=" + (riderReconnectHoldStartGameTime == Long.MIN_VALUE
                    ? 0L : Math.max(0L, now - riderReconnectHoldStartGameTime))
                + " reason=" + (reason == null ? "unspecified" : reason.replace(' ', '_'))
                + " restoredLinearVelocityMps=" + restoredLinear
                + " restoredAngularVelocityRadps=" + restoredAngular
                + " momentumPolicy=EXACT_PRE_HOLD_RESTORE"
        );

        riderReconnectHeld = false;
        riderReconnectPlayerUuid = null;
        riderReconnectHoldStartGameTime = Long.MIN_VALUE;
    }

    public boolean riderReconnectHeld() {
        return riderReconnectHeld;
    }

    /**
     * Live Sable state used by the IV-compatibility crash adapter before the
     * next physics tick.  This is read-only; gameplay code may scale velocity
     * from the pre-impact Sable contact evidence without injecting a second COM collision response.
     */
    public GameplayKinematics gameplayKinematics() {
        if (!isUsable()) {
            return null;
        }
        updatePose();
        rigidBodyHandle.getLinearVelocity(temporaryLinearVelocityWorld);
        rigidBodyHandle.getAngularVelocity(temporaryAngularVelocityWorld);
        Vector3d centerOfMassPosition = new Vector3d(getPose().position());
        Quaterniond orientation = new Quaterniond(getPose().orientation()).normalize();
        if (!finite(centerOfMassPosition) || !finite(orientation)
            || !finite(temporaryLinearVelocityWorld) || !finite(temporaryAngularVelocityWorld)) {
            return null;
        }
        Vector3d modelOriginPosition = modelOriginFromCenterOfMassWorld(
            centerOfMassPosition, orientation
        );
        Vector3d modelOriginVelocity = modelOriginVelocityFromCenterOfMassVelocity(
            temporaryLinearVelocityWorld, temporaryAngularVelocityWorld, orientation
        );
        return new GameplayKinematics(
            modelOriginPosition, orientation, modelOriginVelocity,
            new Vector3d(temporaryAngularVelocityWorld)
        );
    }

    private record InitialPlacementPose(
        Vector3d positionWorld,
        Quaterniond orientationWorld,
        boolean terrainClearFallbackApplied
    ) {
    }

    public record GameplayKinematics(
        Vector3d positionWorld,
        Quaterniond orientationWorld,
        Vector3d linearVelocityWorld,
        Vector3d angularVelocityWorld
    ) {
    }

    public record SweptTerrainImpact(
        long gameTime,
        minecrafttransportsimulator.baseclasses.Point3D block,
        minecrafttransportsimulator.baseclasses.BoundingBox sourceBox,
        Vector3d worldPoint,
        Vector3d worldNormal,
        double projectedAreaSquareMeters,
        double preImpactVehicleSpeedMetersPerSecond,
        double inwardPointSpeedMetersPerSecond,
        double normalImpulseNewtonSeconds,
        float preImpactBlockHardness,
        Resolution trueImpactResolution
    ) {
    }

    public record BodyState(
        Vector3d linearVelocityWorld,
        Vector3d angularVelocityWorld,
        Vector3d lastExportedLinearVelocityWorld,
        Vector3d lastExportedAngularVelocityBody,
        boolean hasExportedState,
        Vector3d positionWorld,
        Quaterniond orientationWorld
    ) {
    }

    private static final class AircraftMassData implements MassData {
        private final double mass;
        private final Matrix3dc inertia;
        private final Matrix3dc inverseInertia;
        private final Vector3dc centerOfMass = new Vector3d();

        private AircraftMassData(double mass, Vec3d desiredInertia) {
            this.mass = Math.max(1.0, mass);
            Matrix3d tensor = new Matrix3d().zero();
            tensor.m00(Math.max(MIN_INERTIA, desiredInertia.x()));
            tensor.m11(Math.max(MIN_INERTIA, desiredInertia.y()));
            tensor.m22(Math.max(MIN_INERTIA, desiredInertia.z()));
            this.inertia = tensor;
            this.inverseInertia = tensor.invert(new Matrix3d());
        }

        @Override
        public double getMass() {
            return mass;
        }

        @Override
        public double getInverseMass() {
            return 1.0 / mass;
        }

        @Override
        public Matrix3dc getInertiaTensor() {
            return inertia;
        }

        @Override
        public Matrix3dc getInverseInertiaTensor() {
            return inverseInertia;
        }

        @Override
        public Vector3dc getCenterOfMass() {
            return centerOfMass;
        }
    }
}
