package com.g9third.pmweatheriv.sable;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import com.g9third.pmweatheriv.mixin.Rapier3DInvoker;
import com.g9third.pmweatheriv.physics.AnimatedWingGeometry;
import com.g9third.pmweatheriv.physics.AuthoredLiftingSurfacePoses;
import com.g9third.pmweatheriv.physics.Vec3d;
import dev.ryanhcode.sable.physics.chunk.VoxelNeighborhoodState;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.entities.components.AEntityD_Definable;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import minecrafttransportsimulator.jsondefs.JSONCollisionGroup;
import minecrafttransportsimulator.jsondefs.JSONCollisionGroup.CollisionType;
import net.minecraft.server.level.ServerLevel;
import org.joml.Quaterniond;
import org.joml.Vector3d;
import org.joml.Vector3dc;

/**
 * PMWeather-IV-only compound collision bridge for Sable 2.x/Rapier.
 *
 * <p>One tiny massless Sable BoxPhysicsObject supplies the single dynamic rigid
 * body. IV-authored BLOCK boxes provide primary structural geometry, including
 * constructor-time static definitions. A cached OBJ surface shell supplements
 * uncovered geometry with native mounted LevelCollider cuboids. Non-ground-device
 * parts retain their authored BLOCK geometry. Vehicle-frame geometry uses an identity child while
 * moving parts use children whose relative transforms follow the authored parts.
 * The resulting compound shape
 * rotates physically with the aircraft and shares the same Rapier scene as
 * terrain, other PMWeather-IV aircraft, and Aeronautics/Sable sublevels.</p>
 *
 * <p>Ground devices are deliberately not uploaded as small mounted voxel
 * colliders. Live Sable terrain probes supply wheel contact metadata to the
 * point-constraint landing-gear solve on the parent body. Compact authored
 * BLOCK gear proxies remain excluded so they cannot create a second physical
 * wheel path.</p>
 *
 * <p>IV's VEHICLE collision flag is retained as semantic ride-surface metadata;
 * in IV 24.0.0 it means another vehicle's wheels may ride on that box and is not
 * a separate general rigid-body collision layer. CLICK/ATTACK/BULLET/ENTITY/etc.
 * likewise remain semantic/query definitions handled by oriented hitbox math.</p>
 */
public final class SableCompoundCollider {
    static final double EPSILON = 1.0E-7;
    private static final double SIGNATURE_SCALE = 1_000_000.0;
    // Tires own ground traction.  Aircraft skin/wing/tail boxes retain solid
    // normal collision but use a low sliding multiplier so a genuine scrape
    // does not behave like a rubber block catching the runway and pole-vaulting
    // the complete rigid body.
    private static final double BODY_VOXEL_FRICTION_MULTIPLIER = 0.15;
    static double bodyTerrainFriction(double surfaceCoefficient) {
        return Double.isFinite(surfaceCoefficient)
            ? BODY_VOXEL_FRICTION_MULTIPLIER * Math.max(0.0, surfaceCoefficient) : 0.0;
    }
    private static final double VOXEL_RESTITUTION = 0.0;
    // A mounted transform update invalidates Rapier's persistent manifold. IV
    // evaluates part animation in doubles, so sub-millimetre round-off must not
    // turn fixed landing gear into a collider that moves every game tick.
    private static final double CHILD_POSE_POSITION_EPSILON_SQUARED = 1.0E-8;
    private static final double CHILD_POSE_ORIENTATION_DOT_EPSILON = 1.0E-8;
    private static final double CHILD_SURFACE_VELOCITY_EPSILON_SQUARED = 1.0E-6;
    private static final int PACKED_CORNER_STATE = VoxelNeighborhoodState.CORNER.byteRepresentation() & 0xFFFF;

    /**
     * Sable 2.0.3 does not expose deletion of voxel-collider data entries. Reuse
     * identical cell shapes globally so animated/replaced IV boxes do not leak a
     * new native shape every time a compound child is refreshed.
     */
    private static final Map<CellShapeKey, Integer> VOXEL_SHAPE_CACHE = new HashMap<>();
    private static final Map<List<CellBox>, List<CellBox>> CELL_UNION_CACHE =
        Collections.synchronizedMap(new LinkedHashMap<>(128, .75f, true) {
            @Override protected boolean removeEldestEntry(Map.Entry<List<CellBox>, List<CellBox>> entry) {
                return size() > 2048;
            }
        });

    private final EntityVehicleF_Physics vehicle;
    private final long sceneHandle;
    private final int parentBodyId;
    /** Frozen IV/model-local center of mass. Mounted geometry is stored relative to this parent origin. */
    private final Vector3d centerOfMassLocal;
    private final IdentityHashMap<Object, MountedChild> children = new IdentityHashMap<>();
    private final Map<Integer, MountedChild> childrenById = new HashMap<>();
    private final Object vehicleFrameKey = new Object();
    private record MovingModelKey(String objectName) {}
    private final Map<String, MovingModelKey> movingModelKeys = new LinkedHashMap<>();
    private Map<MovingModelKey, AnimatedWingGeometry.RigidTransform> movingModelPoses = Map.of();
    private List<SableModelCollisionHull.PreparedMovingObjectHull> movingModelHulls = List.of();
    private int movingModelChildCount;
    private int movingModelBoxCount;
    private int movingModelUnavailableCount;
    private int physicalBoxCount;
    private int wheelColliderCount;
    private int vehicleOnlyBoxCount;
    private int rollingGroundDeviceProxyBoxCount;
    private boolean modelHullActive;
    private int modelHullBoxCount;
    private int authoredVehicleFrameBoxesRetained;
    private int modelHullBoxesSuppressedByAuthored;
    private int modelHullBoxesSupplemented;
    private int staticSupportEnvelopeAlignedCuboids;
    private int definitionStaticVehicleBlockBoxesAvailable;
    private int liveBlockBoxesObserved;
    private int modelHullSourceTriangles;
    private double modelHullResolution;
    private String modelHullReason = "not-prepared";
    private double collisionBoundingRadius;
    // The OBJ shell is immutable for the lifetime of this Sable body. Rebuilding
    // hundreds or thousands of SourceCuboid objects every Minecraft tick was
    // pure allocation churn. Only rebuild mounted geometry when IV's collision
    // topology/gear state actually changes; otherwise refresh child transforms.
    private long lastTopologySignature = Long.MIN_VALUE;
    private String lastModelSourceKey;
    private SableModelCollisionHull.PreparedHull preparedModelHull;
    private long nextModelHullRetryTick = Long.MIN_VALUE;
    private long topologyRebuilds;
    private long topologyFastPathTicks;
    private SableTerrainSweep preparedTerrainSweep;
    private long lastTopologyLogTime = Long.MIN_VALUE;
    private int lastLoggedPhysicalBoxCount = -1;
    private int lastLoggedChildCount = -1;

    public SableCompoundCollider(
        EntityVehicleF_Physics vehicle, long sceneHandle, int parentBodyId, Vec3d centerOfMassLocal
    ) {
        this.vehicle = Objects.requireNonNull(vehicle, "vehicle");
        this.sceneHandle = sceneHandle;
        this.parentBodyId = parentBodyId;
        if (centerOfMassLocal == null || !centerOfMassLocal.isFinite()) {
            throw new IllegalArgumentException("Invalid centerOfMassLocal for compound collider");
        }
        this.centerOfMassLocal = new Vector3d(
            centerOfMassLocal.x(), centerOfMassLocal.y(), centerOfMassLocal.z()
        );
    }

    /**
     * Preflight for the IV update-order boundary. allCollisionBoxes may be empty
     * for the first few vehicle ticks even though IV has already constructed the
     * immutable definitionCollisionBoxes from the content-pack JSON. Static
     * vehicle-frame BLOCK groups are valid immediately; animated groups continue
     * to wait for IV's live collision update.
     */
    public static GeometryReadiness geometryReadiness(EntityVehicleF_Physics vehicle) {
        if (vehicle == null) {
            return new GeometryReadiness(false, 0, 0, "NULL_VEHICLE");
        }

        // IV constructs definitionCollisionBoxes directly from the content-pack
        // JSON in AEntityE_Interactable's constructor, before the first update()
        // has populated collisionBoxes/allCollisionBoxes. Static vehicle-frame
        // BLOCK groups are therefore valid structural geometry immediately and
        // must participate in readiness instead of letting the OBJ shell race
        // ahead and become the only collider for the lifetime of the Sable body.
        Set<BoundingBox> candidates = Collections.newSetFromMap(new IdentityHashMap<>());
        if (vehicle.allCollisionBoxes != null) {
            candidates.addAll(vehicle.allCollisionBoxes);
        }
        candidates.addAll(staticDefinitionBlockBoxes(vehicle));

        int authoredBlockBoxes = 0;
        int ownerLocalReadyBlockBoxes = 0;
        for (BoundingBox box : candidates) {
            if (!isBlockBox(box)) {
                continue;
            }
            ++authoredBlockBoxes;
            APart partOwner = vehicle.getPartWithBox(box);
            AEntityD_Definable<?> owner = partOwner == null ? vehicle : partOwner;
            Vector3d local = OrientedHitboxRegistry.unroundedLocalCenter(box, owner);
            if (local != null && finite(local)
                && Double.isFinite(box.widthRadius) && box.widthRadius > EPSILON
                && Double.isFinite(box.heightRadius) && box.heightRadius > EPSILON
                && Double.isFinite(box.depthRadius) && box.depthRadius > EPSILON) {
                ++ownerLocalReadyBlockBoxes;
            }
        }
        if (authoredBlockBoxes == 0) {
            if (SableModelCollisionHull.canAttempt(vehicle)) {
                return new GeometryReadiness(true, 0, 0, "MODEL_HULL_ONLY_READY");
            }
            return new GeometryReadiness(false, 0, 0, "NO_AUTHORED_BLOCK_BOXES_OR_OBJ_MODEL");
        }
        if (ownerLocalReadyBlockBoxes != authoredBlockBoxes) {
            return new GeometryReadiness(
                false, authoredBlockBoxes, ownerLocalReadyBlockBoxes,
                "OWNER_LOCAL_BOX_CAPTURE_PENDING"
            );
        }
        return new GeometryReadiness(
            true, authoredBlockBoxes, ownerLocalReadyBlockBoxes,
            vehicle.allCollisionBoxes == null || vehicle.allCollisionBoxes.isEmpty()
                ? "READY_FROM_STATIC_DEFINITION_BOXES"
                : "READY"
        );
    }

    public void initialize() {
        refresh(true);
        if (children.isEmpty() || physicalBoxCount == 0) {
            throw new IllegalStateException(
                "IV aircraft has no usable BLOCK physical collision boxes for Sable compound collision"
            );
        }
    }

    /**
     * Refreshes active-box membership/shape only when its stable signature
     * changes. Part-relative transforms are updated every vehicle tick so moving
     * authored parts retain oriented collision without rebuilding their voxels.
     */
    public void refresh() {
        refresh(false);
    }

    private void refresh(boolean force) {
        // Child transforms and source attribution may change on this owner tick.
        preparedTerrainSweep = null;
        String animationDefinitions = SableModelCollisionHull.animationSignature(vehicle);
        boolean movingGeometryChanged = refreshMovingModelPoses(animationDefinitions);
        String modelSourceKey = SableModelCollisionHull.collisionCacheKey(vehicle, animationDefinitions);
        long topologySignature = collisionTopologySignature(modelSourceKey);
        boolean modelRetry=movingModelChildCount == 0 && preparedModelHull!=null && !preparedModelHull.usable()
            && preparedModelHull.modelLocation().toLowerCase(java.util.Locale.ROOT).endsWith(".obj")
            && vehicle.ticksExisted>=nextModelHullRetryTick;
        boolean topologyChanged = force || modelRetry || movingGeometryChanged
            || !Objects.equals(modelSourceKey, lastModelSourceKey)
            || topologySignature != lastTopologySignature;
        Quaterniond vehicleWorld = SablePoseConversions.toQuaternion(vehicle.orientation);

        if (topologyChanged) {
            Map<Object, GroupGeometry> desired = collectGroups(modelSourceKey);
            physicalBoxCount = desired.values().stream()
                .filter(group -> group.material() == ColliderMaterial.BODY)
                .mapToInt(group -> group.boxes().size())
                .sum();
            // Ground devices are IV sensor points, never mounted Sable colliders.
            wheelColliderCount = 0;

            List<Object> removed = new ArrayList<>();
            for (Object key : children.keySet()) {
                if (!desired.containsKey(key)) {
                    removed.add(key);
                }
            }
            for (Object key : removed) {
                MountedChild child = children.remove(key);
                if (child != null) {
                    child.remove();
                }
            }

            for (Map.Entry<Object, GroupGeometry> entry : desired.entrySet()) {
                Object key = entry.getKey();
                GroupGeometry group = entry.getValue();
                MountedChild child = children.get(key);
                if (child == null || force || child.geometrySignature != group.geometrySignature()) {
                    if (child != null) {
                        child.remove();
                    }
                    child = createChild(group);
                    children.put(key, child);
                } else {
                    // Damage attribution may change even when the physical local
                    // geometry does not. Keep gameplay metadata current without
                    // destroying/recreating the native Sable collider.
                    child.updateSourceBoxes(group.sourceBoxes());
                }
                child.updateTransform(group.relativePose());
            }
            lastTopologySignature = topologySignature;
            lastModelSourceKey = modelSourceKey;
            ++topologyRebuilds;
        } else {
            ++topologyFastPathTicks;
            // Geometry is frozen; only moving authored-part transforms can have
            // changed. The rigid vehicle-frame child is therefore a zero-cost
            // unchanged-transform skip on ordinary ticks.
            for (Map.Entry<Object, MountedChild> entry : children.entrySet()) {
                entry.getValue().updateTransform(relativePose(entry.getKey(), vehicleWorld));
            }
        }

        collisionBoundingRadius = 0.0;
        for (Map.Entry<Object, MountedChild> entry : children.entrySet()) {
            RelativePose pose = relativePose(entry.getKey(), vehicleWorld);
            collisionBoundingRadius = Math.max(
                collisionBoundingRadius, entry.getValue().boundingRadius(pose)
            );
        }

        if (PMIVObserver.loggingEnabled() && topologyChanged) {
            long gameTime = vehicle.world.getTime();
            boolean logNow = force || physicalBoxCount != lastLoggedPhysicalBoxCount
                || children.size() != lastLoggedChildCount || lastTopologyLogTime == Long.MIN_VALUE
                || gameTime < lastTopologyLogTime || gameTime - lastTopologyLogTime >= 20;
            if (!logNow) return;
            lastTopologyLogTime = gameTime;
            lastLoggedPhysicalBoxCount = physicalBoxCount;
            lastLoggedChildCount = children.size();
            long relativeTransformUploads = children.values().stream()
                    .mapToLong(child -> child.relativeTransformUploads)
                    .sum();
                long unchangedTransformSkips = children.values().stream()
                    .mapToLong(child -> child.unchangedTransformSkips)
                    .sum();
                if (PMIVObserver.loggingEnabled()) {
                    PMIVObserver.log(
                    "SABLE_COMPOUND_COLLIDER gameTime=" + gameTime
                        + " uuid=" + vehicle.uniqueUUID
                        + " parentBodyId=" + parentBodyId
                        + " mountedChildren=" + children.size()
                        + " movingModelChildren=" + movingModelChildCount
                        + " movingModelBoxes=" + movingModelBoxCount
                        + " movingModelUnavailable=" + movingModelUnavailableCount
                        + " physicalBoxes=" + physicalBoxCount
                        + " wheelColliders=" + wheelColliderCount
                        + " collisionBoundingRadius=" + collisionBoundingRadius
                        + " vehicleOnlySemanticBoxes=" + vehicleOnlyBoxCount
                        + " rollingGroundDeviceProxyBoxesExcluded=" + rollingGroundDeviceProxyBoxCount
                        + " modelHullActive=" + modelHullActive
                        + " modelHullBoxes=" + modelHullBoxCount
                        + " modelHullResolution=" + modelHullResolution
                        + " modelHullSourceTriangles=" + modelHullSourceTriangles
                        + " authoredVehicleFrameBoxesRetained=" + authoredVehicleFrameBoxesRetained
                        + " definitionStaticVehicleBlockBoxesAvailable=" + definitionStaticVehicleBlockBoxesAvailable
                        + " liveBlockBoxesObserved=" + liveBlockBoxesObserved
                        + " modelHullBoxesSuppressedByAuthored=" + modelHullBoxesSuppressedByAuthored
                        + " modelHullBoxesSupplemented=" + modelHullBoxesSupplemented
                        + " staticSupportEnvelopeAlignedCuboids=" + staticSupportEnvelopeAlignedCuboids
                        + " staticSupportClearanceM=0.03"
                        + " staticSupportHullPolicy=SHALLOW_ENVELOPE_CLEARANCE_POINT_NORMAL_AUTHORITY"
                        + " structuralColliderPolicy=IV_DEFINITION_AND_LIVE_AUTHORED_BLOCK_PRIMARY_MODEL_SHELL_SUPPLEMENT"
                        + " terrainContactNormalPolicy=OBB_AABB_SAT_MINIMUM_TRANSLATION_AXIS"
                        + " modelHullReason=" + modelHullReason
                        + " bodyFrictionMultiplier=" + BODY_VOXEL_FRICTION_MULTIPLIER
                        + " relativeTransformUploads=" + relativeTransformUploads
                        + " unchangedTransformSkips=" + unchangedTransformSkips
                        + " topologyRebuilds=" + topologyRebuilds
                        + " topologyFastPathTicks=" + topologyFastPathTicks
                        + " collisionAuthority=SABLE_RAPIER_COMPOUND"
                        + " childOrientationPolicy=BODY_LIVE_PART_POSE"
                        + " mountedTransformPolicy=LIVE_ONLY_WHEN_RELATIVE_POSE_CHANGES"
                        + " gearPolicy=SABLE_LIVE_TERRAIN_POINT_CONSTRAINT_ON_PARENT_BODY_PER_SUBSTEP"
                        + " groundDevicePhysicalColliders=false"
                    + " ivPhysicalCollisionCorrection=false"
            );
                }
        }
    }

    public void remove() {
        preparedTerrainSweep = null;
        for (MountedChild child : new ArrayList<>(children.values())) {
            child.remove();
        }
        children.clear();
        childrenById.clear();
        physicalBoxCount = 0;
        wheelColliderCount = 0;
        vehicleOnlyBoxCount = 0;
        rollingGroundDeviceProxyBoxCount = 0;
        modelHullActive = false;
        modelHullBoxCount = 0;
        authoredVehicleFrameBoxesRetained = 0;
        definitionStaticVehicleBlockBoxesAvailable = 0;
        liveBlockBoxesObserved = 0;
        modelHullBoxesSuppressedByAuthored = 0;
        modelHullBoxesSupplemented = 0;
        staticSupportEnvelopeAlignedCuboids = 0;
        modelHullSourceTriangles = 0;
        modelHullResolution = 0.0;
        modelHullReason = "removed";
        collisionBoundingRadius = 0.0;
        lastTopologySignature = Long.MIN_VALUE;
        lastModelSourceKey = null;
        preparedModelHull = null;
        movingModelKeys.clear(); movingModelPoses = Map.of(); movingModelHulls = List.of();
        movingModelChildCount = 0; movingModelBoxCount = 0;
    }

    public boolean isActive() {
        return !children.isEmpty() && physicalBoxCount > 0;
    }

    public int physicalBoxCount() {
        return physicalBoxCount;
    }

    /**
     * Exact low-envelope support stations from retained rigid skid/float/pontoon
     * OBJ geometry. The coarse mounted voxel shell remains physical body/crash
     * collision above a shallow clearance lip; these stations own the exact
     * normal reaction at their ride height without a duplicate solid manifold.
     */
    public List<SableModelCollisionHull.StaticSupportPoint> staticGroundSupportPoints() {
        return preparedModelHull == null || preparedModelHull.staticSupportPoints() == null
            ? List.of()
            : preparedModelHull.staticSupportPoints();
    }

    public int mountedChildCount() {
        return children.size();
    }

    /** Conservative radius around the parent COM covering every mounted child. */
    public double collisionBoundingRadius() {
        return collisionBoundingRadius;
    }

    public boolean bodyIntersectsTerrainAtPose(
        ServerLevel level, Vector3dc parentWorldPosition, Quaterniond parentWorldOrientation
    ) {
        return terrainSweep().bodyIntersectsTerrainAtPose(level, parentWorldPosition, parentWorldOrientation);
    }

    public TerrainPoseRecovery recoverDeepTerrainPenetration(
        ServerLevel level,
        Vector3dc priorClearPosition,
        Quaterniond priorClearOrientation,
        Vector3dc completedPosition,
        Quaterniond completedOrientation
    ) {
        return terrainSweep().recoverDeepTerrainPenetration(level, priorClearPosition, priorClearOrientation, completedPosition, completedOrientation);
    }

    public TerrainBoundaryImpact findFirstTerrainImpactAlongPath(
        ServerLevel level,
        Vector3dc priorPosition,
        Quaterniond priorOrientation,
        Vector3dc completedPosition,
        Quaterniond completedOrientation
    ) {
        return terrainSweep().findFirstTerrainImpactAlongPath(level, priorPosition, priorOrientation, completedPosition, completedOrientation);
    }

    public record TerrainBoundaryImpact(
        minecrafttransportsimulator.baseclasses.Point3D block,
        BoundingBox sourceBox,
        Vector3d worldPoint,
        Vector3d worldNormal,
        double projectedAreaSquareMeters
    ) {
    }

    public record TerrainPoseRecovery(
        boolean terrainBoundaryViolated,
        boolean recovered,
        boolean priorPoseClear,
        boolean truncated,
        Vector3d resolvedPosition,
        Quaterniond resolvedOrientation,
        double resolvedFraction,
        int poseChecks,
        int blockPositions,
        int collisionShapes,
        TerrainBoundaryImpact boundaryImpact
    ) {
        TerrainPoseRecovery(
            boolean terrainBoundaryViolated,
            boolean recovered,
            boolean priorPoseClear,
            boolean truncated,
            Vector3d resolvedPosition,
            Quaterniond resolvedOrientation,
            double resolvedFraction,
            int poseChecks,
            int blockPositions,
            int collisionShapes
        ) {
            this(
                terrainBoundaryViolated, recovered, priorPoseClear, truncated,
                resolvedPosition, resolvedOrientation, resolvedFraction, poseChecks,
                blockPositions, collisionShapes, null
            );
        }

        static TerrainPoseRecovery invalid() {
            return new TerrainPoseRecovery(
                false, false, false, true, new Vector3d(), new Quaterniond(),
                1.0, 0, 0, 0, null
            );
        }
    }

    SableTerrainSweep.TerrainQuerySession newTerrainQuerySession(ServerLevel level) {
        return terrainSweep().newQuerySession(level);
    }

    private SableTerrainSweep terrainSweep() {
        if (preparedTerrainSweep != null) return preparedTerrainSweep;
        List<SableTerrainSweep.TerrainChild> geometry = new ArrayList<>();
        for (MountedChild child : children.values()) {
            if (!child.removed && child.material == ColliderMaterial.BODY && child.lastPose != null) {
                geometry.add(new SableTerrainSweep.TerrainChild(
                    child.lastPose, child.localCuboids, child.sourceBoxes));
            }
        }
        preparedTerrainSweep = new SableTerrainSweep(geometry, collisionBoundingRadius);
        return preparedTerrainSweep;
    }

    /**
     * Cheap signature for the pieces of IV state that can alter physical
     * collision membership/local geometry. It deliberately excludes parent
     * world pose and ordinary moving-part transforms; those are uploaded by the
     * transform fast path without rebuilding voxel geometry.
     */
    private long collisionTopologySignature(String modelSourceKey) {
        long seed = vehicle.outOfHealth ? 0x5EEDBEEFL : 0x1F123BB5L;
        seed=seed*31+com.g9third.pmweatheriv.physics.ModelCoordinates.scale(vehicle).hashCode();
        seed = includeModelSourceSignature(seed, modelSourceKey);
        for (MovingModelKey key : movingModelKeys.values()) {
            seed = seed * 31 + key.objectName().hashCode();
            seed = seed * 31 + (movingModelPoses.containsKey(key) ? 1 : 0);
        }
        // Retraction/activation changes which support owns the low envelope.
        seed = seed*31 + (hasActiveGroundDeviceSupport() ? 1 : 0);
        Set<BoundingBox> boxes = Collections.newSetFromMap(new IdentityHashMap<>());
        if (vehicle.allCollisionBoxes != null) {
            boxes.addAll(vehicle.allCollisionBoxes);
        }
        boxes.addAll(staticDefinitionBlockBoxes(vehicle));
        if (boxes.isEmpty()) {
            seed = StableCollisionTopology.aggregate(seed, new long[0], 0);
        } else {
            long[] boxSignatures = new long[boxes.size()];
            int boxCount = 0;
            for (BoundingBox box : boxes) {
                // Only BLOCK boxes participate in the physical Sable compound.
                // ENTITY/VEHICLE-only gameplay boxes must not invalidate the
                // native collider cache when IV reconstructs or edits them.
                if (box == null || box.collisionTypes == null
                    || !box.collisionTypes.contains(CollisionType.BLOCK)) {
                    continue;
                }
                APart owner = vehicle.getPartWithBox(box);
                // Parent-frame authored BLOCK boxes are physical geometry in the
                // 0.11.3+ policy, so they MUST remain in the topology signature even
                // when a model hull is usable. 0.11.3 accidentally retained the old
                // "model shell replaces parent boxes" optimization here, which meant
                // late-arriving IV collision boxes never triggered a rebuild.
                AEntityD_Definable<?> geometricOwner = owner == null ? vehicle : owner;
                Vector3d local = OrientedHitboxRegistry.unroundedLocalCenter(box, geometricOwner);
                double localX = local != null && finite(local) ? local.x : 0.0;
                double localY = local != null && finite(local) ? local.y : 0.0;
                double localZ = local != null && finite(local) ? local.z : 0.0;
                boxSignatures[boxCount++] = StableCollisionTopology.boxSignature(
                    localX, localY, localZ,
                    box.widthRadius, box.heightRadius, box.depthRadius,
                    box.collisionTypes,
                    owner == null
                        ? StableCollisionTopology.vehicleFrameOwnerSignature()
                        : StableCollisionTopology.partSemanticSignature(owner)
                );
            }
            seed = StableCollisionTopology.aggregate(seed, boxSignatures, boxCount);
        }

        if (vehicle.allParts == null || vehicle.allParts.isEmpty()) {
            return StableCollisionTopology.aggregate(seed, new long[0], 0);
        }
        long[] groundDeviceSignatures = new long[vehicle.allParts.size()];
        int groundDeviceCount = 0;
        for (APart part : vehicle.allParts) {
            if (!(part instanceof PartGroundDevice device) || !isRollingGroundDevice(device)) {
                continue;
            }
            // Ground-device topology follows authored semantic membership only.
            // Wheel suspension/localOffset/active animation and live size state
            // are handled by the point constraint and must not invalidate or
            // recreate native mounted collider children.
            groundDeviceSignatures[groundDeviceCount++] =
                StableCollisionTopology.groundDeviceSignature(device);
        }
        return StableCollisionTopology.aggregate(seed, groundDeviceSignatures, groundDeviceCount);
    }

    static long includeModelSourceSignature(long seed, String modelSourceKey) {
        // Include resource, scale, animation-definition, static visibility and
        // rolling-envelope identity in topology itself. In particular, a
        // same-named moving object can point at a replacement model/source
        // while retaining its MovingModelKey.
        long signature = 0xCBF29CE484222325L;
        if (modelSourceKey != null) {
            for (int index = 0; index < modelSourceKey.length(); ++index) {
                signature ^= modelSourceKey.charAt(index);
                signature *= 0x100000001B3L;
            }
        }
        return seed * 31 + signature;
    }


    private Map<Object, GroupGeometry> collectGroups(String modelSourceKey) {
        Map<Object, List<SourceCuboid>> sourceCuboidsByOwner = new IdentityHashMap<>();
        List<SourceCuboid> vehicleFrameGameplaySources = new ArrayList<>();
        Set<BoundingBox> boxes = Collections.newSetFromMap(new IdentityHashMap<>());
        liveBlockBoxesObserved = 0;
        if (vehicle.allCollisionBoxes != null) {
            boxes.addAll(vehicle.allCollisionBoxes);
            for (BoundingBox liveBox : vehicle.allCollisionBoxes) {
                if (isBlockBox(liveBox)) {
                    ++liveBlockBoxesObserved;
                }
            }
        }
        // Static vehicle-frame collision groups exist in definitionCollisionBoxes
        // before IV's first update() has populated allCollisionBoxes. Add those
        // immutable authored boxes immediately. Identity de-duplication means the
        // same BoundingBox objects are not duplicated once IV later publishes them
        // into the live sets. Animated/applyAfter groups are deliberately excluded
        // here and continue to follow IV's live switchbox output.
        List<BoundingBox> definitionStaticBoxes = staticDefinitionBlockBoxes(vehicle);
        definitionStaticVehicleBlockBoxesAvailable = definitionStaticBoxes.size();
        boxes.addAll(definitionStaticBoxes);

        Quaterniond vehicleWorld = SablePoseConversions.toQuaternion(vehicle.orientation);
        vehicleOnlyBoxCount = 0;
        rollingGroundDeviceProxyBoxCount = 0;
        for (BoundingBox box : boxes) {
            if (box == null || box.collisionTypes == null || box.collisionTypes.isEmpty()) {
                continue;
            }
            boolean blockCollision = box.collisionTypes.contains(CollisionType.BLOCK);
            boolean vehicleCollision = box.collisionTypes.contains(CollisionType.VEHICLE);
            if (vehicleCollision && !blockCollision) {
                ++vehicleOnlyBoxCount;
            }
            if (!blockCollision) {
                continue;
            }
            APart partOwner = vehicle.getPartWithBox(box);
            if (isGroundDeviceProxy(box, partOwner)) {
                ++rollingGroundDeviceProxyBoxCount;
                continue;
            }
            AEntityD_Definable<?> geometricOwner = partOwner == null ? vehicle : partOwner;
            LocalCuboid cuboid = toOwnerLocalCuboid(box, geometricOwner);
            if (cuboid == null) {
                continue;
            }
            // Keep every usable parent-frame authored BLOCK box available for
            // IV damage/crash attribution.  With the full BODY shell policy these
            // boxes are no longer masked merely because they lie inside the
            // wheelbase; only true rolling ground-device hardware is excluded.
            // Once the OBJ shell is active these sources remain attribution
            // metadata rather than duplicate physical colliders.
            if (partOwner == null) {
                vehicleFrameGameplaySources.add(new SourceCuboid(cuboid, box));
            }
            Object key = partOwner == null ? vehicleFrameKey : partOwner;
            sourceCuboidsByOwner.computeIfAbsent(key, unused -> new ArrayList<>())
                .add(new SourceCuboid(cuboid, box));
        }

        // IV-authored BLOCK boxes are the primary structural collider for the rigid
        // parent airframe. Content packs already use these boxes to define wings,
        // tails and fuselage collision structure; replacing them with a reconstructed
        // render-model shell can silently remove valid wing geometry when an OBJ
        // object is animated or filtered. The OBJ shell therefore supplements, never
        // replaces, authored parent-frame structure. Generated shell cells whose
        // centers already lie inside an authored BLOCK box are omitted to avoid
        // duplicate manifolds; shell cells outside authored structure are retained to
        // add model-derived detail. Moving non-ground-device AParts remain separately
        // authored/mounted as before.
        // collectGroups() runs only on a collision-topology rebuild. Re-resolve
        // the cached prepared hull here because active ground-device membership
        // can change whether static skid/float support belongs in the Sable shell.
        // SableModelCollisionHull.prepare() is cache-backed, so ordinary rebuilds
        // do not reparse the OBJ.
        SableModelCollisionHull.PreparedHull modelHull = SableModelCollisionHull.prepareWithKey(vehicle, modelSourceKey);
        preparedModelHull = modelHull;
        nextModelHullRetryTick = vehicle.ticksExisted+40;
        modelHullActive = false;
        modelHullBoxCount = 0;
        authoredVehicleFrameBoxesRetained = 0;
        modelHullBoxesSuppressedByAuthored = 0;
        modelHullBoxesSupplemented = 0;
        staticSupportEnvelopeAlignedCuboids = 0;
        modelHullSourceTriangles = modelHull.sourceTriangles();
        modelHullResolution = modelHull.resolution();
        modelHullReason = modelHull.reason();
        if (modelHull.usable() && modelHull.boxes() != null && !modelHull.boxes().isEmpty()) {
            List<SourceCuboid> authoredVehicleFrame = sourceCuboidsByOwner.get(vehicleFrameKey);
            List<SourceCuboid> combined = new ArrayList<>(
                (authoredVehicleFrame == null ? 0 : authoredVehicleFrame.size())
                    + modelHull.boxes().size()
            );
            if (authoredVehicleFrame != null && !authoredVehicleFrame.isEmpty()) {
                combined.addAll(authoredVehicleFrame);
                authoredVehicleFrameBoxesRetained = authoredVehicleFrame.size();
            }
            for (SableModelCollisionHull.HullBox modelBox : modelHull.boxes()) {
                LocalCuboid cuboid = new LocalCuboid(
                    stable(modelBox.cx()), stable(modelBox.cy()), stable(modelBox.cz()),
                    stable(modelBox.hx()), stable(modelBox.hy()), stable(modelBox.hz())
                );
                if (cuboidCenterInsideAny(cuboid, authoredVehicleFrame)) {
                    ++modelHullBoxesSuppressedByAuthored;
                    continue;
                }
                BoundingBox gameplaySource = nearestSourceBox(
                    cuboid, vehicleFrameGameplaySources
                );
                if (gameplaySource == null) {
                    gameplaySource = vehicle.encompassingBox;
                }
                combined.add(new SourceCuboid(cuboid, gameplaySource));
                ++modelHullBoxesSupplemented;
            }
            if (!combined.isEmpty()) {
                sourceCuboidsByOwner.put(vehicleFrameKey, combined);
                modelHullActive = modelHullBoxesSupplemented > 0;
                modelHullBoxCount = modelHullBoxesSupplemented;
                modelHullReason = "authored-primary-model-supplement:" + modelHull.reason();
            } else {
                modelHullReason = "generated-shell-empty-after-authored-primary-filter";
            }
        }

        // Every moving exterior object owns immutable geometry and its actual native matrix.
        // No static-pose culling against authored boxes: that overlap can change with animation.
        movingModelChildCount = 0; movingModelBoxCount = 0;
        for (SableModelCollisionHull.PreparedMovingObjectHull hull : movingModelHulls) {
            MovingModelKey key = movingModelKeys.get(hull.objectName());
            if (key == null || !movingModelPoses.containsKey(key)) continue;
            List<SourceCuboid> generated = new ArrayList<>(hull.boxes().size());
            for (SableModelCollisionHull.HullBox box : hull.boxes()) generated.add(new SourceCuboid(
                new LocalCuboid(stable(box.cx()), stable(box.cy()), stable(box.cz()),
                    stable(box.hx()), stable(box.hy()), stable(box.hz())), vehicle.encompassingBox));
            sourceCuboidsByOwner.put(key, generated);
            ++movingModelChildCount; movingModelBoxCount += generated.size();
        }

        if (!hasActiveGroundDeviceSupport() && modelHull.usable()
            && modelHull.staticSupportPoints()!=null && !modelHull.staticSupportPoints().isEmpty()) {
            List<SourceCuboid> parent = sourceCuboidsByOwner.get(vehicleFrameKey);
            if (parent!=null) sourceCuboidsByOwner.put(vehicleFrameKey,
                alignStaticSupportEnvelope(parent,modelHull.staticSupportPoints(),modelHull.resolution()));
        }

        Map<Object, GroupGeometry> groups = new IdentityHashMap<>();
        for (Map.Entry<Object, List<SourceCuboid>> entry : sourceCuboidsByOwner.entrySet()) {
            List<SourceCuboid> sourced = entry.getValue();
            sourced.sort(Comparator.comparing(SourceCuboid::cuboid, LocalCuboid.COMPARATOR));
            if (sourced.isEmpty()) {
                continue;
            }
            List<LocalCuboid> cuboids = new ArrayList<>(sourced.size());
            List<BoundingBox> sourceBoxes = new ArrayList<>(sourced.size());
            for (SourceCuboid source : sourced) {
                cuboids.add(source.cuboid());
                sourceBoxes.add(source.sourceBox());
            }
            RelativePose relativePose = relativePose(entry.getKey(), vehicleWorld);
            groups.put(entry.getKey(), new GroupGeometry(
                List.copyOf(cuboids),
                Collections.unmodifiableList(new ArrayList<>(sourceBoxes)),
                geometrySignature(cuboids, ColliderMaterial.BODY),
                relativePose,
                ColliderMaterial.BODY
            ));
        }

        return groups;
    }

    /**
     * Converts the latest Sable collision report into world-space contacts that
     * retain the originating IV collision-box identity. This is gameplay
     * metadata only; Rapier has already solved the physical impulse.
     */
    public List<GameplayContact> gameplayContacts(
        SableCollisionCapture.SceneSnapshot snapshot,
        Vector3d parentWorldPosition,
        Quaterniond parentWorldOrientation
    ) {
        if (snapshot == null || snapshot.contacts().isEmpty()
            || parentWorldPosition == null || parentWorldOrientation == null) {
            return List.of();
        }
        Quaterniond parent = new Quaterniond(parentWorldOrientation).normalize();
        List<GameplayContact> result = new ArrayList<>();
        for (SableCollisionCapture.ContactForce contact : snapshot.contacts()) {
            MountedChild child = childrenById.get(contact.colliderA());
            if (child == null) {
                child = childrenById.get(contact.colliderB());
            }
            if (child == null || child.removed || child.lastPose == null) {
                continue;
            }
            int other = contact.otherCollider(child.id);
            if (other == Integer.MIN_VALUE || childrenById.containsKey(other)) {
                // Rapier normally suppresses same-rigid-body self contacts, but
                // keep the gameplay bridge defensive if such a report appears.
                continue;
            }
            Vector3d pointChild = contact.localPointFor(child.id);
            Vector3d normalChild = contact.localNormalFor(child.id);
            if (!finite(pointChild) || !finite(normalChild)) {
                continue;
            }
            Vector3d pointParent = new Quaterniond(child.lastPose.orientation())
                .transform(pointChild, new Vector3d())
                .add(child.lastPose.position());
            Vector3d normalParent = new Quaterniond(child.lastPose.orientation())
                .transform(normalChild, new Vector3d());
            Vector3d pointWorld = new Quaterniond(parent)
                .transform(pointParent, new Vector3d())
                .add(parentWorldPosition);
            Vector3d normalWorld = new Quaterniond(parent)
                .transform(normalParent, new Vector3d());
            if (!finite(pointWorld) || !finite(normalWorld)
                || normalWorld.lengthSquared() <= 1.0E-12) {
                continue;
            }
            normalWorld.normalize();
            result.add(new GameplayContact(
                child.id,
                other,
                contact.forceNewtons(),
                contact.impulseNewtonSeconds(),
                contact.substepIndex(),
                pointWorld,
                normalWorld,
                pointParent,
                normalParent.normalize(),
                sourceBoxAt(child, pointChild),
                false
            ));
        }
        return result.isEmpty() ? List.of() : List.copyOf(result);
    }

    /** Checks only contacts involving this compound's colliders in a scene-wide snapshot. */
    public boolean hasExternalContact(SableCollisionCapture.SceneSnapshot snapshot) {
        if (snapshot == null || snapshot.contacts().isEmpty()) return false;
        for (SableCollisionCapture.ContactForce contact : snapshot.contacts()) {
            MountedChild child = childrenById.get(contact.colliderA());
            if (child == null) child = childrenById.get(contact.colliderB());
            if (child == null || child.removed) continue;
            int other = contact.otherCollider(child.id);
            if (other != Integer.MIN_VALUE && !childrenById.containsKey(other)) return true;
        }
        return false;
    }

    private static BoundingBox sourceBoxAt(MountedChild child, Vector3d pointChild) {
        if (child.sourceBoxes.isEmpty() || child.localCuboids.isEmpty()) {
            return null;
        }
        BoundingBox best = null;
        double bestDistance = Double.POSITIVE_INFINITY;
        double bestVolume = Double.POSITIVE_INFINITY;
        int count = Math.min(child.sourceBoxes.size(), child.localCuboids.size());
        for (int i = 0; i < count; ++i) {
            LocalCuboid box = child.localCuboids.get(i);
            double dx = Math.max(0.0, Math.abs(pointChild.x - box.cx()) - box.hx());
            double dy = Math.max(0.0, Math.abs(pointChild.y - box.cy()) - box.hy());
            double dz = Math.max(0.0, Math.abs(pointChild.z - box.cz()) - box.hz());
            double distance = dx * dx + dy * dy + dz * dz;
            double volume = box.hx() * box.hy() * box.hz();
            if (distance < bestDistance - 1.0E-12
                || (Math.abs(distance - bestDistance) <= 1.0E-12 && volume < bestVolume)) {
                bestDistance = distance;
                bestVolume = volume;
                best = child.sourceBoxes.get(i);
            }
        }
        return best;
    }

    public record GameplayContact(
        int ownColliderId,
        int otherColliderId,
        double forceNewtons,
        double impulseNewtonSeconds,
        int substepIndex,
        Vector3d worldPoint,
        Vector3d worldNormal,
        Vector3d parentLocalPoint,
        Vector3d parentLocalNormal,
        BoundingBox sourceBox,
        boolean wheelContact
    ) {
        public boolean terrainContact() {
            return otherColliderId == -1;
        }
    }

    /**
     * Returns whether a small generated OBJ-shell cell is already represented by
     * primary authored IV structure. Generated detail inside authored structure is
     * suppressed so Rapier does not receive duplicate body manifolds there.
     */
    private static boolean cuboidCenterInsideAny(
        LocalCuboid candidate, List<SourceCuboid> authored
    ) {
        if (candidate == null || authored == null || authored.isEmpty()) {
            return false;
        }
        double x = candidate.cx();
        double y = candidate.cy();
        double z = candidate.cz();
        for (SourceCuboid source : authored) {
            if (source == null || source.cuboid() == null) {
                continue;
            }
            LocalCuboid box = source.cuboid();
            if (x >= box.cx() - box.hx() - EPSILON
                && x <= box.cx() + box.hx() + EPSILON
                && y >= box.cy() - box.hy() - EPSILON
                && y <= box.cy() + box.hy() + EPSILON
                && z >= box.cz() - box.hz() - EPSILON
                && z <= box.cz() + box.hz() + EPSILON) {
                return true;
            }
        }
        return false;
    }

    private static BoundingBox nearestSourceBox(
        LocalCuboid generated,
        List<SourceCuboid> authoredVehicleFrame
    ) {
        if (generated == null || authoredVehicleFrame == null || authoredVehicleFrame.isEmpty()) {
            return null;
        }
        BoundingBox best = null;
        double bestDistance = Double.POSITIVE_INFINITY;
        double bestVolume = Double.POSITIVE_INFINITY;
        for (SourceCuboid source : authoredVehicleFrame) {
            if (source == null || source.cuboid() == null || source.sourceBox() == null) {
                continue;
            }
            LocalCuboid box = source.cuboid();
            double dx = Math.max(0.0, Math.abs(generated.cx() - box.cx()) - box.hx());
            double dy = Math.max(0.0, Math.abs(generated.cy() - box.cy()) - box.hy());
            double dz = Math.max(0.0, Math.abs(generated.cz() - box.cz()) - box.hz());
            double distance = dx * dx + dy * dy + dz * dz;
            double volume = box.hx() * box.hy() * box.hz();
            if (distance < bestDistance - 1.0E-12
                || (Math.abs(distance - bestDistance) <= 1.0E-12 && volume < bestVolume)) {
                bestDistance = distance;
                bestVolume = volume;
                best = source.sourceBox();
            }
        }
        return best;
    }

    /** Only rolling/tread hardware is removed from the static BODY shell. */
    private static boolean isRollingGroundDevice(PartGroundDevice device) {
        return device != null
            && device.definition != null
            && device.definition.ground != null
            && (device.definition.ground.isWheel || device.definition.ground.isTread);
    }

    private boolean hasActiveGroundDeviceSupport() {
        if (vehicle.allParts==null) return false;
        for (APart part:vehicle.allParts)
            if (part instanceof PartGroundDevice device && device.isValid && !device.isSpare
                && device.isActiveVar.isActive) return true;
        return false;
    }

    /**
     * A voxel/authored low-envelope lip must not land before the exact rigid
     * support stations. Remove only shallow excess within each support object's
     * footprint; the point solve owns normal support there. Side/upper structure,
     * deeper authored body geometry, and geometry outside that footprint survive.
     */
    private List<SourceCuboid> alignStaticSupportEnvelope(List<SourceCuboid> sources,
            List<SableModelCollisionHull.StaticSupportPoint> supports, double resolution) {
        Map<String,double[]> footprints=new LinkedHashMap<>();
        for (SableModelCollisionHull.StaticSupportPoint point:supports) {
            double[] b=footprints.computeIfAbsent(point.sourceObject(),key->new double[] {
                Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY});
            b[0]=Math.min(b[0],point.x()); b[1]=Math.min(b[1],point.z());
            b[2]=Math.min(b[2],point.y()); b[3]=Math.max(b[3],point.x()); b[4]=Math.max(b[4],point.z());
        }
        List<SourceCuboid> result=new ArrayList<>(sources);
        double overshoot=Math.max(0.03,resolution)+1e-6;
        for (double[] b:footprints.values()) {
            // 0.2 m voxel cells can extend beyond the rail at its endpoints.
            double padding=Math.max(0.03,resolution);
            double x0=b[0]-padding,z0=b[1]-padding,x1=b[3]+padding,z1=b[4]+padding;
            double floor=b[2]+0.03;
            List<SourceCuboid> next=new ArrayList<>();
            for (SourceCuboid source:result) {
                LocalCuboid c=source.cuboid();
                double minX=c.cx()-c.hx(),maxX=c.cx()+c.hx();
                double minY=c.cy()-c.hy(),maxY=c.cy()+c.hy();
                double minZ=c.cz()-c.hz(),maxZ=c.cz()+c.hz();
                double ix0=Math.max(minX,x0),ix1=Math.min(maxX,x1);
                double iz0=Math.max(minZ,z0),iz1=Math.min(maxZ,z1);
                if (minY>=floor || minY<b[2]-overshoot || ix1-ix0<=EPSILON || iz1-iz0<=EPSILON) {
                    next.add(source); continue;
                }
                ++staticSupportEnvelopeAlignedCuboids;
                // Partition x/z before clipping y, preserving authored geometry
                // outside the exact support envelope without widening a carve.
                addSupportPiece(next,source,minX,minY,minZ,ix0,maxY,maxZ);
                addSupportPiece(next,source,ix1,minY,minZ,maxX,maxY,maxZ);
                addSupportPiece(next,source,ix0,minY,minZ,ix1,maxY,iz0);
                addSupportPiece(next,source,ix0,minY,iz1,ix1,maxY,maxZ);
                addSupportPiece(next,source,ix0,Math.max(minY,floor),iz0,ix1,maxY,iz1);
            }
            result=next;
        }
        return result;
    }
    private static void addSupportPiece(List<SourceCuboid> result, SourceCuboid source,
            double x0,double y0,double z0,double x1,double y1,double z1) {
        if (x1-x0<=EPSILON || y1-y0<=EPSILON || z1-z0<=EPSILON) return;
        result.add(new SourceCuboid(new LocalCuboid(stable((x0+x1)*0.5),stable((y0+y1)*0.5),
            stable((z0+z1)*0.5),stable((x1-x0)*0.5),stable((y1-y0)*0.5),stable((z1-z0)*0.5)),source.sourceBox()));
    }

    /**
     * Authored BLOCK boxes owned by installed rolling/tread ground devices are
     * sensor/animation hardware, not parent BODY structure. Keep them out of the
     * rigid compound for their entire installed lifetime. In particular, do not
     * key this decision to IV's transient {@code part_active}: on the first tick
     * of a player placement the real wheels can be valid and installed while
     * part_active is still false. Uploading those boxes as BODY for that one
     * topology build makes the tire solver and the BODY collider compete for the
     * runway and can start Rapier already embedded in terrain.
     *
     * <p>Live gear contact still decides whether a wheel can carry load; this only
     * prevents the wheel/tread part's authored BLOCK proxy from becoming a second
     * rigid-body collision path. Spare devices remain ordinary carried geometry.</p>
     */
    private boolean isGroundDeviceProxy(BoundingBox candidate, APart partOwner) {
        return candidate != null
            && partOwner instanceof PartGroundDevice device
            && !device.isSpare
            && device.isValid
            && isRollingGroundDevice(device);
    }

    /**
     * Returns always-active vehicle-frame BLOCK boxes directly from IV's parsed
     * content-pack definition. AEntityE_Interactable constructs these BoundingBox
     * instances before the first update tick, while collisionBoxes/allCollisionBoxes
     * are populated later. Groups with animations/applyAfter remain live-only so
     * PMIV never freezes a moving collision group at its authored rest position.
     * Group health is honored using IV's own collision_N_damage variable.
     */
    private static List<BoundingBox> staticDefinitionBlockBoxes(
        EntityVehicleF_Physics vehicle
    ) {
        if (vehicle == null || vehicle.definition == null
            || vehicle.definition.collisionGroups == null
            || vehicle.definitionCollisionBoxes == null
            || vehicle.definition.collisionGroups.isEmpty()
            || vehicle.definitionCollisionBoxes.isEmpty()) {
            return List.of();
        }
        int groups = Math.min(
            vehicle.definition.collisionGroups.size(),
            vehicle.definitionCollisionBoxes.size()
        );
        List<BoundingBox> result = new ArrayList<>();
        for (int index = 0; index < groups; ++index) {
            JSONCollisionGroup group = vehicle.definition.collisionGroups.get(index);
            if (group == null || group.collisionTypes == null
                || !group.collisionTypes.contains(CollisionType.BLOCK)) {
                continue;
            }
            if ((group.animations != null && !group.animations.isEmpty())
                || (group.applyAfter != null && !group.applyAfter.isBlank())) {
                continue;
            }
            if (group.health > 0) {
                double damage = vehicle.getOrCreateVariable(
                    "collision_" + (index + 1) + "_damage"
                ).currentValue;
                if (Double.isFinite(damage) && damage >= group.health) {
                    continue;
                }
            }
            List<BoundingBox> groupBoxes = vehicle.definitionCollisionBoxes.get(index);
            if (groupBoxes == null || groupBoxes.isEmpty()) {
                continue;
            }
            for (BoundingBox box : groupBoxes) {
                if (isBlockBox(box)) {
                    result.add(box);
                }
            }
        }
        return result.isEmpty() ? List.of() : List.copyOf(result);
    }

    private static boolean isBlockBox(BoundingBox box) {
        return box != null && box.collisionTypes != null
            && box.collisionTypes.contains(CollisionType.BLOCK);
    }

    private LocalCuboid toOwnerLocalCuboid(
        BoundingBox box,
        AEntityD_Definable<?> owner
    ) {
        double hx = Math.abs(box.widthRadius);
        double hy = Math.abs(box.heightRadius);
        double hz = Math.abs(box.depthRadius);
        if (!Double.isFinite(hx) || !Double.isFinite(hy) || !Double.isFinite(hz)
            || hx <= EPSILON || hy <= EPSILON || hz <= EPSILON) {
            return null;
        }

        // Do not reconstruct local geometry from box.globalCenter. IV rounds the
        // world center of ENTITY/VEHICLE boxes to a 1/64-block grid after its
        // local->world transform. Capturing the owner-local input at
        // BoundingBox.updateToEntity HEAD keeps Rapier geometry continuous.
        Vector3d localCenter = OrientedHitboxRegistry.unroundedLocalCenter(box, owner);
        if (localCenter == null || !finite(localCenter)) {
            return null;
        }
        return new LocalCuboid(
            stable(localCenter.x), stable(localCenter.y), stable(localCenter.z),
            stable(hx), stable(hy), stable(hz)
        );
    }

    private boolean refreshMovingModelPoses(String animationDefinitions) {
        List<SableModelCollisionHull.PreparedMovingObjectHull> latestHulls =
            SableModelCollisionHull.prepareMovingExteriorHulls(vehicle, animationDefinitions);
        boolean geometryChanged = latestHulls != movingModelHulls;
        movingModelHulls = latestHulls;
        if (geometryChanged) {
            Set<String> activeNames = new java.util.HashSet<>();
            for (SableModelCollisionHull.PreparedMovingObjectHull hull : latestHulls)
                activeNames.add(hull.objectName());
            movingModelKeys.keySet().retainAll(activeNames);
        }
        Map<MovingModelKey, AnimatedWingGeometry.RigidTransform> poses = new IdentityHashMap<>();
        movingModelUnavailableCount = 0;
        for (SableModelCollisionHull.PreparedMovingObjectHull hull : movingModelHulls) {
            MovingModelKey key = movingModelKeys.computeIfAbsent(hull.objectName(), MovingModelKey::new);
            var transform = AuthoredLiftingSurfacePoses.liveMatrix(vehicle, hull.objectName());
            if (transform == null || !transform.isRigid()) { ++movingModelUnavailableCount; continue; }
            poses.put(key, transform);
        }
        movingModelPoses = poses;
        return geometryChanged;
    }

    /** A cuboid child supports proper rigid affine transforms only; never flatten shear into a rotation. */
    static RelativePose movingModelRelativePose(AnimatedWingGeometry.RigidTransform transform, Vector3dc center) {
        if (transform == null || !transform.isRigid() || center == null || !finite(center)) return null;
        Vec3d offset = transform.translation();
        return new RelativePose(new Vector3d(offset.x(), offset.y(), offset.z()).sub(center),
            SablePoseConversions.toQuaternion(transform.rotationMatrix()));
    }

    private RelativePose relativePose(Object key, Quaterniond vehicleWorld) {
        if (key instanceof MovingModelKey moving) {
            RelativePose pose = movingModelRelativePose(movingModelPoses.get(moving), centerOfMassLocal);
            if (pose == null) throw new IllegalStateException("Unresolved moving model pose: " + moving.objectName());
            return pose;
        }
        if (key == vehicleFrameKey) {
            return new RelativePose(
                new Vector3d(centerOfMassLocal).negate(), new Quaterniond()
            );
        }
        APart part = (APart) key;
        // IV's APart.update() computes position from the master entity pose, then
        // rewrites localOffset through partOn.localOrientation and partOn.localOffset.
        // That final localOffset is parent-relative for nested parts. Keep the
        // numerically stable direct-root path, but resolve nested origins from their
        // actual world position back into the vehicle frame before subtracting COM.
        Vector3d localOffset = mountedPartRelativePosition(
            part.partOn != null,
            new Vector3d(part.localOffset.x, part.localOffset.y, part.localOffset.z),
            new Vector3d(part.position.x, part.position.y, part.position.z),
            new Vector3d(vehicle.position.x, vehicle.position.y, vehicle.position.z),
            vehicleWorld,
            centerOfMassLocal
        );
        Quaterniond partWorld = SablePoseConversions.toQuaternion(part.orientation);
        Quaterniond liveRelativeOrientation = new Quaterniond(vehicleWorld)
            .conjugate()
            .mul(partWorld)
            .normalize();
        Quaterniond relativeOrientation = liveRelativeOrientation;
        if (part instanceof PartGroundDevice device
            && device.definition != null
            && device.definition.ground != null
            && device.definition.ground.isWheel) {
            // IV applies ground_rotation as an internal visual rotation about
            // the tire's local X axle. Collision must not spin the contact pad
            // through the terrain every render-wheel revolution. Preserve live
            // placement/steering/retraction through the axle direction and use
            // the minimum-roll orientation that maps local +X onto that axle.
            Vector3d axleVehicleLocal = liveRelativeOrientation.transform(
                new Vector3d(1.0, 0.0, 0.0)
            );
            if (finite(axleVehicleLocal) && axleVehicleLocal.lengthSquared() > 1.0E-12) {
                axleVehicleLocal.normalize();
                relativeOrientation = new Quaterniond().rotationTo(
                    new Vector3d(1.0, 0.0, 0.0), axleVehicleLocal
                ).normalize();
            }
            if (device.placementDefinition != null
                && device.placementDefinition.turnsWithSteer
                && Double.isFinite(vehicle.rudderAngleVar.currentValue)) {
                // IV uses turnsWithSteer in its ground solver without rotating
                // the APart transform itself. Rotate the physical pad by the
                // same live steering angle so generic steerable gear remains
                // coherent with PMWeather-IV's tire-plane direction.
                relativeOrientation = new Quaterniond()
                    .rotationY(Math.toRadians(-vehicle.rudderAngleVar.currentValue))
                    .mul(relativeOrientation)
                    .normalize();
            }
        }
        return new RelativePose(localOffset, relativeOrientation);
    }

    /**
     * Resolves a mounted part origin in the Sable parent body's COM frame.
     * Direct-root AParts retain their stable master-frame localOffset; nested
     * AParts use their resolved world origin because IV rebases localOffset to
     * partOn after position has already been calculated.
     */
    static Vector3d mountedPartRelativePosition(
        boolean nested,
        Vector3dc localOffset,
        Vector3dc partWorldPosition,
        Vector3dc vehicleWorldPosition,
        Quaterniond vehicleWorldOrientation,
        Vector3dc centerOfMassLocal
    ) {
        if (!finite(centerOfMassLocal)) {
            throw new IllegalArgumentException("Invalid centerOfMassLocal for mounted collider pose");
        }
        Vector3d center = new Vector3d(centerOfMassLocal);
        Vector3d resolvedMasterOrigin = resolvedMasterFrameOrigin(
            partWorldPosition, vehicleWorldPosition, vehicleWorldOrientation
        );

        if (nested && resolvedMasterOrigin != null) {
            Vector3d relative = new Vector3d(resolvedMasterOrigin).sub(center);
            if (finite(relative)) return relative;
        }
        if (finite(localOffset)) {
            Vector3d relative = new Vector3d(localOffset).sub(center);
            if (finite(relative)) return relative;
        }
        if (resolvedMasterOrigin != null) {
            Vector3d relative = new Vector3d(resolvedMasterOrigin).sub(center);
            if (finite(relative)) return relative;
        }

        throw new IllegalArgumentException("No finite origin available for mounted collider pose");
    }

    private static Vector3d resolvedMasterFrameOrigin(
        Vector3dc partWorldPosition,
        Vector3dc vehicleWorldPosition,
        Quaterniond vehicleWorldOrientation
    ) {
        if (!finite(partWorldPosition) || !finite(vehicleWorldPosition)
            || !finite(vehicleWorldOrientation)
            || !Double.isFinite(vehicleWorldOrientation.lengthSquared())) {
            return null;
        }
        Vector3d worldOffset = new Vector3d(partWorldPosition).sub(vehicleWorldPosition);
        if (!finite(worldOffset)) return null;
        Quaterniond inverseVehicleOrientation = new Quaterniond(vehicleWorldOrientation)
            .normalize()
            .conjugate();
        Vector3d masterOffset = inverseVehicleOrientation.transform(worldOffset);
        return finite(masterOffset) ? masterOffset : null;
    }

    private MountedChild createChild(GroupGeometry group) {
        int id = Rapier3DInvoker.pmweatherIv$nextBodyId();
        Rapier3DInvoker.pmweatherIv$createMountedLevelCollider(
            sceneHandle,
            parentBodyId,
            id,
            new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}
        );
        try {
            // Sable 2.0.3 only caches a LevelCollider AABB once both its
            // center-of-mass metadata and local bounds exist.  The native create
            // entrypoint ignores its pose argument, so seed the mounted transform
            // first; setLocalBounds below then builds the valid broadphase AABB.
            setChildTransform(id, group.relativePose(), new Vector3d(), new Vector3d());
            UploadData upload = voxelize(group.boxes(), group.material());
            for (Map.Entry<SectionKey, int[]> section : upload.sections().entrySet()) {
                SectionKey pos = section.getKey();
                Rapier3DInvoker.pmweatherIv$addMountedLevelColliderChunkSection(
                    sceneHandle, id, pos.x(), pos.y(), pos.z(), section.getValue()
                );
            }
            Rapier3DInvoker.pmweatherIv$setLocalBounds(
                sceneHandle,
                id,
                upload.minX(), upload.minY(), upload.minZ(),
                upload.maxX(), upload.maxY(), upload.maxZ()
            );
            MountedChild child = new MountedChild(
                id,
                group.geometrySignature(),
                group.material(),
                List.copyOf(group.boxes()),
                Collections.unmodifiableList(new ArrayList<>(group.sourceBoxes()))
            );
            childrenById.put(id, child);
            return child;
        } catch (RuntimeException | Error failure) {
            try {
                Rapier3DInvoker.pmweatherIv$removeMountedLevelCollider(sceneHandle, id);
            } catch (RuntimeException ignored) {
                // Preserve the original creation failure.
            }
            throw failure;
        }
    }

    private UploadData voxelize(List<LocalCuboid> cuboids, ColliderMaterial material) {
        Map<CellKey, List<CellBox>> cells = new HashMap<>();
        int minX = Integer.MAX_VALUE;
        int minY = Integer.MAX_VALUE;
        int minZ = Integer.MAX_VALUE;
        int maxX = Integer.MIN_VALUE;
        int maxY = Integer.MIN_VALUE;
        int maxZ = Integer.MIN_VALUE;

        for (LocalCuboid cuboid : cuboids) {
            double boxMinX = cuboid.cx() - cuboid.hx();
            double boxMinY = cuboid.cy() - cuboid.hy();
            double boxMinZ = cuboid.cz() - cuboid.hz();
            double boxMaxX = cuboid.cx() + cuboid.hx();
            double boxMaxY = cuboid.cy() + cuboid.hy();
            double boxMaxZ = cuboid.cz() + cuboid.hz();

            int firstX = floorCell(boxMinX);
            int firstY = floorCell(boxMinY);
            int firstZ = floorCell(boxMinZ);
            int lastX = floorCell(Math.nextDown(boxMaxX));
            int lastY = floorCell(Math.nextDown(boxMaxY));
            int lastZ = floorCell(Math.nextDown(boxMaxZ));

            for (int x = firstX; x <= lastX; ++x) {
                for (int z = firstZ; z <= lastZ; ++z) {
                    for (int y = firstY; y <= lastY; ++y) {
                        double localMinX = Math.max(0.0, boxMinX - x);
                        double localMinY = Math.max(0.0, boxMinY - y);
                        double localMinZ = Math.max(0.0, boxMinZ - z);
                        double localMaxX = Math.min(1.0, boxMaxX - x);
                        double localMaxY = Math.min(1.0, boxMaxY - y);
                        double localMaxZ = Math.min(1.0, boxMaxZ - z);
                        if (localMaxX - localMinX <= EPSILON
                            || localMaxY - localMinY <= EPSILON
                            || localMaxZ - localMinZ <= EPSILON) {
                            continue;
                        }
                        CellKey cell = new CellKey(x, y, z);
                        cells.computeIfAbsent(cell, unused -> new ArrayList<>()).add(
                            new CellBox(
                                stable(localMinX), stable(localMinY), stable(localMinZ),
                                stable(localMaxX), stable(localMaxY), stable(localMaxZ)
                            )
                        );
                        minX = Math.min(minX, x);
                        minY = Math.min(minY, y);
                        minZ = Math.min(minZ, z);
                        maxX = Math.max(maxX, x);
                        maxY = Math.max(maxY, y);
                        maxZ = Math.max(maxZ, z);
                    }
                }
            }
        }

        if (cells.isEmpty()) {
            throw new IllegalStateException("No non-degenerate Sable compound cells were generated");
        }

        Map<SectionKey, int[]> sections = new LinkedHashMap<>();
        List<Map.Entry<CellKey, List<CellBox>>> orderedCells = new ArrayList<>(cells.entrySet());
        orderedCells.sort(Map.Entry.comparingByKey());
        for (Map.Entry<CellKey, List<CellBox>> entry : orderedCells) {
            CellKey cell = entry.getKey();
            List<CellBox> pieces = unionCellPieces(entry.getValue());
            int colliderHandle = voxelColliderFor(pieces, material);
            SectionKey section = new SectionKey(
                Math.floorDiv(cell.x(), 16),
                Math.floorDiv(cell.y(), 16),
                Math.floorDiv(cell.z(), 16)
            );
            int[] data = sections.computeIfAbsent(section, unused -> new int[4096]);
            int bx = Math.floorMod(cell.x(), 16);
            int by = Math.floorMod(cell.y(), 16);
            int bz = Math.floorMod(cell.z(), 16);
            int index = bx + (bz << 4) + (by << 8);
            data[index] = PACKED_CORNER_STATE | ((colliderHandle + 1) << 16);
        }
        return new UploadData(sections, minX, minY, minZ, maxX, maxY, maxZ);
    }


    /**
     * Converts potentially overlapping authored cuboids in one unit cell into a
     * non-overlapping cuboid union. Rapier solves every generated contact; this
     * prevents overlapping IV hitboxes from multiplying contact manifolds and
     * therefore impact/support response.
     */
    private static List<CellBox> unionCellPieces(List<CellBox> input) {
        if (input.size() <= 1) {
            return input.isEmpty() ? List.of() : List.of(input.get(0));
        }

        // Moving authored boxes can require a child upload while most unit cells
        // keep identical geometry. Cache the exact union, independent of scene,
        // vehicle pose and native handles. No quantization is introduced here.
        List<CellBox> key = List.copyOf(input);
        List<CellBox> cached = CELL_UNION_CACHE.get(key);
        if (cached != null) return cached;
        List<CellBox> result = List.copyOf(unionCellPiecesUncached(input));
        CELL_UNION_CACHE.put(key, result);
        return result;
    }

    private static List<CellBox> unionCellPiecesUncached(List<CellBox> input) {

        List<Double> xs = uniqueBoundaries(input, 0);
        List<Double> ys = uniqueBoundaries(input, 1);
        List<Double> zs = uniqueBoundaries(input, 2);
        int nx = xs.size() - 1;
        int ny = ys.size() - 1;
        int nz = zs.size() - 1;
        boolean[][][] occupied = new boolean[nx][ny][nz];
        for (int x = 0; x < nx; ++x) {
            double mx = (xs.get(x) + xs.get(x + 1)) * 0.5;
            for (int y = 0; y < ny; ++y) {
                double my = (ys.get(y) + ys.get(y + 1)) * 0.5;
                for (int z = 0; z < nz; ++z) {
                    double mz = (zs.get(z) + zs.get(z + 1)) * 0.5;
                    for (CellBox box : input) {
                        if (mx > box.minX() - EPSILON && mx < box.maxX() + EPSILON
                            && my > box.minY() - EPSILON && my < box.maxY() + EPSILON
                            && mz > box.minZ() - EPSILON && mz < box.maxZ() + EPSILON) {
                            occupied[x][y][z] = true;
                            break;
                        }
                    }
                }
            }
        }

        boolean[][][] used = new boolean[nx][ny][nz];
        List<CellBox> result = new ArrayList<>();
        for (int y = 0; y < ny; ++y) {
            for (int z = 0; z < nz; ++z) {
                for (int x = 0; x < nx; ++x) {
                    if (!occupied[x][y][z] || used[x][y][z]) {
                        continue;
                    }
                    int xEnd = x + 1;
                    while (xEnd < nx && occupied[xEnd][y][z] && !used[xEnd][y][z]) {
                        ++xEnd;
                    }
                    int zEnd = z + 1;
                    while (zEnd < nz && planeFree(occupied, used, x, xEnd, y, y + 1, zEnd)) {
                        ++zEnd;
                    }
                    int yEnd = y + 1;
                    while (yEnd < ny && volumeFree(occupied, used, x, xEnd, yEnd, z, zEnd)) {
                        ++yEnd;
                    }
                    for (int yy = y; yy < yEnd; ++yy) {
                        for (int zz = z; zz < zEnd; ++zz) {
                            for (int xx = x; xx < xEnd; ++xx) {
                                used[xx][yy][zz] = true;
                            }
                        }
                    }
                    result.add(new CellBox(
                        xs.get(x), ys.get(y), zs.get(z),
                        xs.get(xEnd), ys.get(yEnd), zs.get(zEnd)
                    ));
                }
            }
        }
        result.sort(CellBox.COMPARATOR);
        return result;
    }

    private static List<Double> uniqueBoundaries(List<CellBox> boxes, int axis) {
        List<Double> values = new ArrayList<>();
        for (CellBox box : boxes) {
            double min = axis == 0 ? box.minX() : axis == 1 ? box.minY() : box.minZ();
            double max = axis == 0 ? box.maxX() : axis == 1 ? box.maxY() : box.maxZ();
            values.add(stable(min));
            values.add(stable(max));
        }
        values.sort(Double::compare);
        List<Double> unique = new ArrayList<>();
        for (double value : values) {
            if (unique.isEmpty() || Math.abs(value - unique.get(unique.size() - 1)) > EPSILON) {
                unique.add(value);
            }
        }
        return unique;
    }

    private static boolean planeFree(
        boolean[][][] occupied, boolean[][][] used,
        int x0, int x1, int y0, int y1, int z
    ) {
        for (int y = y0; y < y1; ++y) {
            for (int x = x0; x < x1; ++x) {
                if (!occupied[x][y][z] || used[x][y][z]) {
                    return false;
                }
            }
        }
        return true;
    }

    private static boolean volumeFree(
        boolean[][][] occupied, boolean[][][] used,
        int x0, int x1, int y, int z0, int z1
    ) {
        for (int z = z0; z < z1; ++z) {
            for (int x = x0; x < x1; ++x) {
                if (!occupied[x][y][z] || used[x][y][z]) {
                    return false;
                }
            }
        }
        return true;
    }

    private static synchronized int voxelColliderFor(
        List<CellBox> pieces,
        ColliderMaterial material
    ) {
        CellShapeKey key = new CellShapeKey(List.copyOf(pieces), material);
        Integer existing = VOXEL_SHAPE_CACHE.get(key);
        if (existing != null) {
            return existing;
        }
        double volume = 0.0;
        for (CellBox box : pieces) {
            volume += (box.maxX() - box.minX())
                * (box.maxY() - box.minY())
                * (box.maxZ() - box.minZ());
        }
        int handle = Rapier3DInvoker.pmweatherIv$newVoxelCollider(
            material.frictionMultiplier(),
            Math.min(1.0, Math.max(0.0, volume)),
            VOXEL_RESTITUTION,
            false,
            null
        );
        for (CellBox box : pieces) {
            Rapier3DInvoker.pmweatherIv$addVoxelColliderBox(
                handle,
                new double[]{
                    box.minX(), box.minY(), box.minZ(),
                    box.maxX(), box.maxY(), box.maxZ()
                }
            );
        }
        VOXEL_SHAPE_CACHE.put(key, handle);
        return handle;
    }

    private static long geometrySignature(List<LocalCuboid> boxes, ColliderMaterial material) {
        long hash = fnv(0xcbf29ce484222325L, material.ordinal());
        for (LocalCuboid box : boxes) {
            hash = fnv(hash, stableBits(box.cx()));
            hash = fnv(hash, stableBits(box.cy()));
            hash = fnv(hash, stableBits(box.cz()));
            hash = fnv(hash, stableBits(box.hx()));
            hash = fnv(hash, stableBits(box.hy()));
            hash = fnv(hash, stableBits(box.hz()));
        }
        return hash;
    }

    private static long fnv(long hash, long value) {
        hash ^= value;
        return hash * 0x100000001b3L;
    }

    static boolean finite(Vector3dc value) {
        return value != null
            && Double.isFinite(value.x())
            && Double.isFinite(value.y())
            && Double.isFinite(value.z());
    }

    static boolean finite(Quaterniond value) {
        return value != null
            && Double.isFinite(value.x)
            && Double.isFinite(value.y)
            && Double.isFinite(value.z)
            && Double.isFinite(value.w)
            && value.lengthSquared() > 1.0E-18;
    }

    private static long stableBits(double value) {
        return Double.doubleToLongBits(stable(value));
    }

    private static double stable(double value) {
        return Math.rint(value * SIGNATURE_SCALE) / SIGNATURE_SCALE;
    }

    private static int floorCell(double value) {
        return (int) Math.floor(value + 1.0E-10);
    }

    private void setChildTransform(
        int id,
        RelativePose pose,
        Vector3d linearVelocityLocal,
        Vector3d angularVelocityLocal
    ) {
        Vector3d p = pose.position();
        Quaterniond q = new Quaterniond(pose.orientation()).normalize();
        Rapier3DInvoker.pmweatherIv$setMountedLevelColliderTransform(
            sceneHandle,
            id,
            new double[]{0.0, 0.0, 0.0},
            new double[]{p.x, p.y, p.z, q.x, q.y, q.z, q.w},
            new double[]{
                linearVelocityLocal.x, linearVelocityLocal.y, linearVelocityLocal.z,
                angularVelocityLocal.x, angularVelocityLocal.y, angularVelocityLocal.z
            }
        );
    }

    /** Native fake surface rates are child-local, relative to the child's zero origin. */
    static RelativeSurfaceVelocity relativeSurfaceVelocity(RelativePose previous, RelativePose current,
                                                            long elapsedOwnerTicks) {
        if (previous == null || current == null || elapsedOwnerTicks != 1
            || !finite(previous.position()) || !finite(current.position())
            || !finite(previous.orientation()) || !finite(current.orientation()))
            return new RelativeSurfaceVelocity(new Vector3d(), new Vector3d());
        Quaterniond orientation = new Quaterniond(current.orientation()).normalize();
        Vector3d linear = orientation.transformInverse(new Vector3d(current.position())
            .sub(previous.position()).mul(20.0));
        // current * inverse(previous) expresses the finite rotation axis in the
        // parent frame. Rapier's fake surface rate is child-local, so convert
        // that parent-frame angular rate to the current child frame exactly once.
        Quaterniond delta = new Quaterniond(orientation)
            .mul(new Quaterniond(previous.orientation()).normalize().conjugate()).normalize();
        if (delta.w < 0.0) delta.mul(-1.0);
        Vector3d angularParent = new Vector3d(delta.x, delta.y, delta.z);
        double sineHalfAngle = angularParent.length();
        angularParent.mul(sineHalfAngle > 1E-12
            ? 40.0 * Math.atan2(sineHalfAngle, delta.w) / sineHalfAngle : 40.0);
        Vector3d angular = orientation.transformInverse(angularParent);
        return new RelativeSurfaceVelocity(finite(linear) ? linear : new Vector3d(),
            finite(angular) ? angular : new Vector3d());
    }

    record RelativeSurfaceVelocity(Vector3d linear, Vector3d angular) {}

    private static final class RelativeMotionHistory {
        private RelativePose lastObservedPose;
        private long lastObservationGameTime = Long.MIN_VALUE;

        RelativeSurfaceVelocity observe(RelativePose pose, long gameTime) {
            RelativeSurfaceVelocity velocity = relativeSurfaceVelocity(lastObservedPose, pose,
                lastObservationGameTime == Long.MIN_VALUE ? 0 : gameTime - lastObservationGameTime);
            lastObservedPose = pose.copy();
            lastObservationGameTime = gameTime;
            return velocity;
        }
    }

    private final class MountedChild {
        private final int id;
        private final long geometrySignature;
        private final ColliderMaterial material;
        private final List<LocalCuboid> localCuboids;
        private List<BoundingBox> sourceBoxes;
        private final double localBoundingRadius;
        private RelativePose lastPose;
        private final RelativeMotionHistory relativeMotionHistory = new RelativeMotionHistory();
        private boolean surfaceVelocityUploaded;
        private long relativeTransformUploads;
        private long unchangedTransformSkips;
        private boolean removed;

        private MountedChild(
            int id,
            long geometrySignature,
            ColliderMaterial material,
            List<LocalCuboid> localCuboids,
            List<BoundingBox> sourceBoxes
        ) {
            this.id = id;
            this.geometrySignature = geometrySignature;
            this.material = material;
            this.localCuboids = localCuboids;
            this.sourceBoxes = sourceBoxes;
            double radius = 0.0;
            for (LocalCuboid box : localCuboids) {
                double centerRadius = Math.sqrt(
                    box.cx() * box.cx() + box.cy() * box.cy() + box.cz() * box.cz()
                );
                double extentRadius = Math.sqrt(
                    box.hx() * box.hx() + box.hy() * box.hy() + box.hz() * box.hz()
                );
                radius = Math.max(radius, centerRadius + extentRadius);
            }
            this.localBoundingRadius = radius;
        }

        private void updateSourceBoxes(List<BoundingBox> latestSourceBoxes) {
            this.sourceBoxes = latestSourceBoxes;
        }

        private double boundingRadius(RelativePose pose) {
            Vector3d offset = pose == null ? null : pose.position();
            double offsetRadius = offset == null || !finite(offset) ? 0.0 : offset.length();
            return offsetRadius + localBoundingRadius;
        }

        private void updateTransform(RelativePose pose) {
            if (removed) {
                return;
            }

            Vector3d linearLocal = new Vector3d();
            Vector3d angularLocal = new Vector3d();
            long gameTime = vehicle.world.getTime();
            double orientationDot = lastPose == null
                ? 0.0
                : Math.abs(pose.orientation().dot(lastPose.orientation()));
            boolean poseChanged = lastPose == null
                || pose.position().distanceSquared(lastPose.position())
                    > CHILD_POSE_POSITION_EPSILON_SQUARED
                || 1.0 - Math.min(1.0, orientationDot)
                    > CHILD_POSE_ORIENTATION_DOT_EPSILON;
            // Duplicate refreshes within one owner tick must not erase that tick's
            // uploaded rate. Observe every new tick, including long idle periods,
            // so the first motion after idle is not averaged over the idle duration.
            if (!poseChanged && gameTime == relativeMotionHistory.lastObservationGameTime) {
                ++unchangedTransformSkips;
                return;
            }
            RelativeSurfaceVelocity rate = relativeMotionHistory.observe(pose, gameTime);
            if (poseChanged) {
                linearLocal.set(rate.linear());
                angularLocal.set(rate.angular());
            }
            if (!finite(linearLocal)) {
                linearLocal.zero();
            }
            if (!finite(angularLocal)) {
                angularLocal.zero();
            }

            // Never upload a non-zero velocity that we simultaneously classify
            // as residue. Otherwise a very slow animated/noisy child can retain
            // that surface velocity forever because the following unchanged pose
            // would not qualify for the one-shot stop update.
            if (linearLocal.lengthSquared() <= CHILD_SURFACE_VELOCITY_EPSILON_SQUARED) {
                linearLocal.zero();
            }
            if (angularLocal.lengthSquared() <= CHILD_SURFACE_VELOCITY_EPSILON_SQUARED) {
                angularLocal.zero();
            }

            boolean hasSurfaceVelocity =
                linearLocal.lengthSquared() > CHILD_SURFACE_VELOCITY_EPSILON_SQUARED
                    || angularLocal.lengthSquared() > CHILD_SURFACE_VELOCITY_EPSILON_SQUARED;
            // lastPose is intentionally the last transform actually uploaded to
            // Rapier, not merely the last noisy IV observation. Contact telemetry
            // therefore uses the native pose; rate history separately records every
            // owner tick. One final same-pose zero-velocity upload is required
            // after genuine relative animation stops.
            if (poseChanged) {
                setChildTransform(id, pose, linearLocal, angularLocal);
                ++relativeTransformUploads;
                lastPose = pose.copy();
                surfaceVelocityUploaded = hasSurfaceVelocity;
            } else if (surfaceVelocityUploaded) {
                setChildTransform(id, lastPose, new Vector3d(), new Vector3d());
                ++relativeTransformUploads;
                surfaceVelocityUploaded = false;
            } else {
                ++unchangedTransformSkips;
            }
        }

        private void remove() {
            if (removed) {
                return;
            }
            removed = true;
            childrenById.remove(id, this);
            Rapier3DInvoker.pmweatherIv$removeMountedLevelCollider(sceneHandle, id);
        }
    }

    public record GeometryReadiness(
        boolean ready,
        int authoredBlockBoxes,
        int ownerLocalReadyBlockBoxes,
        String reason
    ) {
    }

    private record SourceCuboid(LocalCuboid cuboid, BoundingBox sourceBox) {
    }

    private record GroupGeometry(
        List<LocalCuboid> boxes,
        List<BoundingBox> sourceBoxes,
        long geometrySignature,
        RelativePose relativePose,
        ColliderMaterial material
    ) {
    }

    private enum ColliderMaterial {
        BODY(BODY_VOXEL_FRICTION_MULTIPLIER);

        private final double frictionMultiplier;

        ColliderMaterial(double frictionMultiplier) {
            this.frictionMultiplier = frictionMultiplier;
        }

        private double frictionMultiplier() {
            return frictionMultiplier;
        }
    }

    record RelativePose(Vector3d position, Quaterniond orientation) {
        private static final RelativePose IDENTITY = new RelativePose(
            new Vector3d(), new Quaterniond()
        );

        private RelativePose copy() {
            return new RelativePose(new Vector3d(position), new Quaterniond(orientation));
        }
    }

    record LocalCuboid(double cx, double cy, double cz, double hx, double hy, double hz) {
        private static final Comparator<LocalCuboid> COMPARATOR = Comparator
            .comparingDouble(LocalCuboid::cx)
            .thenComparingDouble(LocalCuboid::cy)
            .thenComparingDouble(LocalCuboid::cz)
            .thenComparingDouble(LocalCuboid::hx)
            .thenComparingDouble(LocalCuboid::hy)
            .thenComparingDouble(LocalCuboid::hz);
    }

    private record CellBox(double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
        private static final Comparator<CellBox> COMPARATOR = Comparator
            .comparingDouble(CellBox::minX)
            .thenComparingDouble(CellBox::minY)
            .thenComparingDouble(CellBox::minZ)
            .thenComparingDouble(CellBox::maxX)
            .thenComparingDouble(CellBox::maxY)
            .thenComparingDouble(CellBox::maxZ);
    }

    private record CellShapeKey(List<CellBox> boxes, ColliderMaterial material) {
    }

    private record CellKey(int x, int y, int z) implements Comparable<CellKey> {
        @Override
        public int compareTo(CellKey other) {
            int result = Integer.compare(x, other.x);
            if (result != 0) return result;
            result = Integer.compare(y, other.y);
            if (result != 0) return result;
            return Integer.compare(z, other.z);
        }
    }

    private record SectionKey(int x, int y, int z) {
    }

    private record UploadData(
        Map<SectionKey, int[]> sections,
        int minX,
        int minY,
        int minZ,
        int maxX,
        int maxY,
        int maxZ
    ) {
    }
}
