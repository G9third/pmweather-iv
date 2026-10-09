package com.g9third.pmweatheriv.sable;

import com.g9third.pmweatheriv.physics.LandingGearSolver;
import java.util.Collections;
import java.util.Map;
import java.util.WeakHashMap;
import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.baseclasses.BoundingBoxHitResult;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.blocks.components.ABlockBase.Axis;
import minecrafttransportsimulator.entities.components.AEntityD_Definable;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import org.joml.Quaterniond;
import org.joml.Vector3d;

/**
 * Oriented semantic hitbox view for IV collision boxes on managed PMWeather-IV vehicles.
 *
 * <p>IV stores a rotated world center but continues to test width/height/depth
 * against the world axes. Physical BLOCK collision is handled by the Sable
 * compound body and landing gear by the live-Sable-terrain point constraint; IV VEHICLE remains ride-surface
 * metadata. This registry makes the remaining IV semantic queries
 * (CLICK/ATTACK/BULLET/ENTITY/etc.) use the same live vehicle/part orientation.
 * It does not replace IV's collision-group metadata or gameplay callbacks.</p>
 *
 * <p>Entries are weak and identity-based in practice because BoundingBox keeps
 * Object identity semantics.  Boxes not belonging to an eligible managed PMWeather-IV
 * vehicle are never intercepted.</p>
 */
public final class OrientedHitboxRegistry {
    private static final double EPSILON = 1.0E-9;
    private static final double SAT_EPSILON = 1.0E-10;
    private static final Map<BoundingBox, OrientedBox> BOXES = Collections.synchronizedMap(
        new WeakHashMap<>()
    );
    /**
     * Unrounded owner-local center captured at BoundingBox.updateToEntity HEAD.
     * IV rounds world centers for ENTITY/VEHICLE boxes after transforming them;
     * Rapier geometry must not inherit that world-grid quantization.
     */
    private static final Map<BoundingBox, Vector3d> LOCAL_CENTERS = Collections.synchronizedMap(
        new WeakHashMap<>()
    );
    /** Boxes that are intentionally conservative world-AABB query proxies. */
    private static final Map<BoundingBox, Boolean> BROADPHASE_BOXES = Collections.synchronizedMap(
        new WeakHashMap<>()
    );

    private OrientedHitboxRegistry() {
    }

    /**
     * Called before IV mutates globalCenter. optionalOffset is still owner-local
     * here (and may alias box.globalCenter), so copy it immediately.
     */
    public static void captureLocalUpdate(
        BoundingBox box,
        Point3D optionalOffset,
        AEntityD_Definable<?> owner
    ) {
        if (box == null || owner == null) {
            return;
        }
        Point3D source = optionalOffset != null ? optionalOffset : box.localCenter;
        if (source == null || owner.scale == null
            || !Double.isFinite(source.x)
            || !Double.isFinite(source.y)
            || !Double.isFinite(source.z)
            || !Double.isFinite(owner.scale.x)
            || !Double.isFinite(owner.scale.y)
            || !Double.isFinite(owner.scale.z)) {
            LOCAL_CENTERS.remove(box);
            return;
        }
        // Mirror BoundingBox.updateToEntity's local scale step, but stop before
        // world rotation/translation and the ENTITY/VEHICLE 1/64-block clamp.
        LOCAL_CENTERS.put(box, new Vector3d(
            source.x * owner.scale.x,
            source.y * owner.scale.y,
            source.z * owner.scale.z
        ));
    }

    /**
     * Returns the current unrounded owner-local center for Sable physical
     * collider construction. Falls back to IV's immutable local center when a
     * box has not yet passed through updateToEntity in this lifecycle.
     */
    public static Vector3d unroundedLocalCenter(
        BoundingBox box,
        AEntityD_Definable<?> owner
    ) {
        if (box == null) {
            return null;
        }
        Vector3d captured = LOCAL_CENTERS.get(box);
        if (captured != null) {
            return new Vector3d(captured);
        }
        Point3D local = box.localCenter;
        if (local == null || owner == null || owner.scale == null) {
            return local == null ? null : new Vector3d(local.x, local.y, local.z);
        }
        return new Vector3d(
            local.x * owner.scale.x,
            local.y * owner.scale.y,
            local.z * owner.scale.z
        );
    }

    /** Called after IV updates a collision box's world center/dimensions. */
    public static void updateFromEntity(BoundingBox box, AEntityD_Definable<?> owner) {
        if (box == null || owner == null) {
            return;
        }

        EntityVehicleF_Physics vehicle = owner instanceof EntityVehicleF_Physics direct
            ? direct
            : owner instanceof APart part ? part.vehicleOn : null;
        if (!LandingGearSolver.shouldReplaceIvGroundOperations(vehicle)) {
            BOXES.remove(box);
            BROADPHASE_BOXES.remove(box);
            return;
        }

        // IV uses the vehicle encompassing box as a broadphase before it ever
        // checks CLICK/ATTACK/BULLET boxes. Keep that broadphase axis-aligned,
        // but make it conservatively contain the true oriented child boxes so a
        // rolled/pitched aircraft cannot be rejected before exact OBB testing.
        if (owner == vehicle && box == vehicle.encompassingBox) {
            updateVehicleBroadphase(vehicle, box);
            return;
        }
        BROADPHASE_BOXES.remove(box);

        if (box.collisionTypes == null || box.collisionTypes.isEmpty()) {
            BOXES.remove(box);
            return;
        }

        double hx = Math.abs(box.widthRadius);
        double hy = Math.abs(box.heightRadius);
        double hz = Math.abs(box.depthRadius);
        if (!finitePositive(hx) || !finitePositive(hy) || !finitePositive(hz)
            || box.globalCenter == null
            || !Double.isFinite(box.globalCenter.x)
            || !Double.isFinite(box.globalCenter.y)
            || !Double.isFinite(box.globalCenter.z)) {
            BOXES.remove(box);
            return;
        }

        Quaterniond orientation = SablePoseConversions.toQuaternion(owner.orientation);
        if (!finite(orientation)) {
            BOXES.remove(box);
            return;
        }
        Vector3d localCenter = unroundedLocalCenter(box, owner);
        Vector3d worldCenter;
        if (localCenter != null && finite(localCenter)) {
            worldCenter = new Quaterniond(orientation).transform(localCenter)
                .add(owner.position.x, owner.position.y, owner.position.z);
        } else {
            worldCenter = new Vector3d(box.globalCenter.x, box.globalCenter.y, box.globalCenter.z);
        }
        BOXES.put(box, new OrientedBox(
            worldCenter,
            orientation,
            new Vector3d(hx, hy, hz),
            owner,
            localCenter
        ));
    }

    private static void updateVehicleBroadphase(
        EntityVehicleF_Physics vehicle,
        BoundingBox encompassingBox
    ) {
        double minX = Double.POSITIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double minZ = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;
        double maxZ = Double.NEGATIVE_INFINITY;
        boolean found = false;

        for (BoundingBox child : vehicle.allCollisionBoxes) {
            OrientedBox oriented = BOXES.get(child);
            if (oriented == null) {
                oriented = axisAligned(child);
            }
            if (oriented == null) {
                continue;
            }
            Vector3d[] axes = oriented.axes();
            double worldHx = Math.abs(axes[0].x) * oriented.halfExtents.x
                + Math.abs(axes[1].x) * oriented.halfExtents.y
                + Math.abs(axes[2].x) * oriented.halfExtents.z;
            double worldHy = Math.abs(axes[0].y) * oriented.halfExtents.x
                + Math.abs(axes[1].y) * oriented.halfExtents.y
                + Math.abs(axes[2].y) * oriented.halfExtents.z;
            double worldHz = Math.abs(axes[0].z) * oriented.halfExtents.x
                + Math.abs(axes[1].z) * oriented.halfExtents.y
                + Math.abs(axes[2].z) * oriented.halfExtents.z;
            minX = Math.min(minX, oriented.center.x - worldHx);
            minY = Math.min(minY, oriented.center.y - worldHy);
            minZ = Math.min(minZ, oriented.center.z - worldHz);
            maxX = Math.max(maxX, oriented.center.x + worldHx);
            maxY = Math.max(maxY, oriented.center.y + worldHy);
            maxZ = Math.max(maxZ, oriented.center.z + worldHz);
            found = true;
        }

        if (!found) {
            BOXES.remove(encompassingBox);
            BROADPHASE_BOXES.remove(encompassingBox);
            return;
        }
        BROADPHASE_BOXES.put(encompassingBox, Boolean.TRUE);
        BOXES.put(encompassingBox, new OrientedBox(
            new Vector3d(
                (minX + maxX) * 0.5,
                (minY + maxY) * 0.5,
                (minZ + maxZ) * 0.5
            ),
            new Quaterniond(),
            new Vector3d(
                Math.max(EPSILON, (maxX - minX) * 0.5),
                Math.max(EPSILON, (maxY - minY) * 0.5),
                Math.max(EPSILON, (maxZ - minZ) * 0.5)
            )
        ));
    }

    public static boolean contains(BoundingBox box) {
        return box != null && BOXES.containsKey(box);
    }

    /**
     * Conservative world-axis bounds for the vehicle encompassing-box query
     * proxy. Exact semantic hit acceptance remains OBB-based.
     */
    public static WorldAabb conservativeWorldAabb(BoundingBox box) {
        if (box == null || !BROADPHASE_BOXES.containsKey(box)) {
            return null;
        }
        return worldAabb(BOXES.get(box));
    }

    /**
     * Conservative Minecraft broadphase for an individual oriented query box.
     * Unlike {@link #conservativeWorldAabb(BoundingBox)}, this is intentionally
     * available for exact-query call sites such as WrapperWorld.attackEntities:
     * callers MUST follow the broadphase with one of the exact methods below.
     *
     * <p>For simple auxiliary damage bounds (for example a propeller disc) IV
     * creates a box without collisionTypes and never calls updateToEntity on it.
     * When such a box is used by a definable aircraft part, allowAuxiliary may
     * derive the missing orientation from that source while retaining the box's
     * live world center and radii. This is query geometry only; it never creates
     * a Sable physical collider.</p>
     */
    public static WorldAabb conservativeWorldAabbForQuery(
        BoundingBox box,
        AEntityD_Definable<?> fallbackOwner,
        boolean allowAuxiliary
    ) {
        return worldAabb(resolveQueryBox(box, fallbackOwner, allowAuxiliary));
    }

    /** Exact registered/auxiliary OBB versus vanilla world-AABB test. */
    public static boolean intersectsWorldAabb(
        BoundingBox box,
        AEntityD_Definable<?> fallbackOwner,
        boolean allowAuxiliary,
        double minX, double minY, double minZ,
        double maxX, double maxY, double maxZ
    ) {
        OrientedBox oriented = resolveQueryBox(box, fallbackOwner, allowAuxiliary);
        OrientedBox target = axisAligned(minX, minY, minZ, maxX, maxY, maxZ);
        return oriented != null && target != null && intersectsSat(oriented, target);
    }

    /**
     * Exact continuous SAT for a linearly translating OBB against a static
     * vanilla AABB over normalized time [0,1]. Orientation is intentionally held
     * fixed over this one IV damage query, matching IV's motion-only argument.
     */
    public static boolean sweptIntersectsWorldAabb(
        BoundingBox box,
        AEntityD_Definable<?> fallbackOwner,
        boolean allowAuxiliary,
        Point3D motion,
        double minX, double minY, double minZ,
        double maxX, double maxY, double maxZ
    ) {
        OrientedBox moving = resolveQueryBox(box, fallbackOwner, allowAuxiliary);
        OrientedBox target = axisAligned(minX, minY, minZ, maxX, maxY, maxZ);
        if (moving == null || target == null || motion == null
            || !Double.isFinite(motion.x) || !Double.isFinite(motion.y) || !Double.isFinite(motion.z)) {
            return false;
        }
        Vector3d velocity = new Vector3d(motion.x, motion.y, motion.z);
        if (velocity.lengthSquared() <= EPSILON * EPSILON) {
            return intersectsSat(moving, target);
        }

        Vector3d[] aAxes = moving.axes();
        Vector3d[] worldAxes = {
            new Vector3d(1.0, 0.0, 0.0),
            new Vector3d(0.0, 1.0, 0.0),
            new Vector3d(0.0, 0.0, 1.0)
        };
        Vector3d centerDelta = new Vector3d(target.center).sub(moving.center);
        double enterTime = 0.0;
        double exitTime = 1.0;

        // 3 moving-box face normals + 3 world-AABB face normals.
        for (Vector3d axis : aAxes) {
            double[] interval = sweptAxisInterval(moving, target, centerDelta, velocity, axis);
            if (interval == null) return false;
            enterTime = Math.max(enterTime, interval[0]);
            exitTime = Math.min(exitTime, interval[1]);
            if (enterTime > exitTime + EPSILON) return false;
        }
        for (Vector3d axis : worldAxes) {
            double[] interval = sweptAxisInterval(moving, target, centerDelta, velocity, axis);
            if (interval == null) return false;
            enterTime = Math.max(enterTime, interval[0]);
            exitTime = Math.min(exitTime, interval[1]);
            if (enterTime > exitTime + EPSILON) return false;
        }

        // 9 edge cross-products. Degenerate parallel axes add no constraint.
        for (Vector3d aAxis : aAxes) {
            for (Vector3d worldAxis : worldAxes) {
                Vector3d axis = new Vector3d(aAxis).cross(worldAxis);
                if (axis.lengthSquared() <= EPSILON * EPSILON) {
                    continue;
                }
                axis.normalize();
                double[] interval = sweptAxisInterval(moving, target, centerDelta, velocity, axis);
                if (interval == null) return false;
                enterTime = Math.max(enterTime, interval[0]);
                exitTime = Math.min(exitTime, interval[1]);
                if (enterTime > exitTime + EPSILON) return false;
            }
        }
        return exitTime >= -EPSILON && enterTime <= 1.0 + EPSILON;
    }


    /**
     * Exact contact information for a vanilla world-AABB against one live
     * oriented IV hitbox. The normal always points from the aircraft hitbox
     * toward the vanilla entity, so translating the entity by
     * {@code normal * penetration} separates the pair.
     */
    public record EntityContact(
        double normalX,
        double normalY,
        double normalZ,
        double penetration,
        double time
    ) {
        public Vector3d normal() {
            return new Vector3d(normalX, normalY, normalZ);
        }
    }

    public static EntityContact contactWorldAabb(
        BoundingBox box,
        double minX, double minY, double minZ,
        double maxX, double maxY, double maxZ
    ) {
        OrientedBox moving = BOXES.get(box);
        OrientedBox target = axisAligned(minX, minY, minZ, maxX, maxY, maxZ);
        ContactAxis contact = minimumTranslation(moving, target);
        return contact == null ? null : new EntityContact(
            contact.normal.x, contact.normal.y, contact.normal.z,
            contact.penetration, 1.0
        );
    }

    /** Exact distance interval where a translated world AABB overlaps this OBB. */
    public record TranslationContactInterval(double enter, double exit) {
    }

    public static TranslationContactInterval translationContactInterval(
        BoundingBox box, BoundingBox entityBounds, Vector3d direction
    ) {
        OrientedBox aircraft = BOXES.get(box);
        OrientedBox entity = axisAligned(entityBounds);
        if (aircraft == null || entity == null || !finite(direction)
            || direction.lengthSquared() <= EPSILON * EPSILON) return null;
        Vector3d[] aAxes = aircraft.axes();
        Vector3d[] worldAxes = {
            new Vector3d(1, 0, 0), new Vector3d(0, 1, 0), new Vector3d(0, 0, 1)
        };
        Vector3d[] axes = new Vector3d[15];
        int index = 0;
        for (Vector3d axis : aAxes) axes[index++] = axis;
        for (Vector3d axis : worldAxes) axes[index++] = axis;
        for (Vector3d aAxis : aAxes)
            for (Vector3d worldAxis : worldAxes)
                axes[index++] = new Vector3d(aAxis).cross(worldAxis);
        Vector3d delta = new Vector3d(entity.center).sub(aircraft.center);
        // sweptAxisInterval models aircraft motion. Negating the entity's
        // translation gives the same relative configuration on every SAT axis.
        Vector3d relativeMotion = new Vector3d(direction).negate();
        double enter = Double.NEGATIVE_INFINITY;
        double exit = Double.POSITIVE_INFINITY;
        for (Vector3d axis : axes) {
            if (axis.lengthSquared() <= EPSILON * EPSILON) continue;
            Vector3d unitAxis = new Vector3d(axis).normalize();
            if (Math.abs(relativeMotion.dot(unitAxis)) <= EPSILON) {
                double aircraftRadius = aircraft.halfExtents.x * Math.abs(unitAxis.dot(aAxes[0]))
                    + aircraft.halfExtents.y * Math.abs(unitAxis.dot(aAxes[1]))
                    + aircraft.halfExtents.z * Math.abs(unitAxis.dot(aAxes[2]));
                double entityRadius = entity.halfExtents.x * Math.abs(unitAxis.x)
                    + entity.halfExtents.y * Math.abs(unitAxis.y)
                    + entity.halfExtents.z * Math.abs(unitAxis.z);
                // A floor tangent to the feet is not a horizontal cabin wall.
                if (Math.abs(delta.dot(unitAxis)) >= aircraftRadius + entityRadius - EPSILON)
                    return null;
            }
            double[] interval = sweptAxisInterval(aircraft, entity, delta, relativeMotion, axis);
            if (interval == null) return null;
            enter = Math.max(enter, interval[0]);
            exit = Math.min(exit, interval[1]);
            if (enter > exit + EPSILON) return null;
        }
        return !Double.isFinite(exit) || exit < 0.0 ? null
            : new TranslationContactInterval(enter, exit);
    }

    /**
     * Previous-to-current pose sweep for vanilla entity contact.
     *
     * <p>Sable owns the aircraft body, but vanilla entities are not Rapier
     * bodies. This sweep therefore samples the exact IV-authored OBB along its
     * owner pose from prevPosition/prevOrientation to the current pose. The
     * subdivision count is geometry-derived from translation plus rotational
     * arc travel and then refined by bisection at the first overlap. It is only
     * a Minecraft entity-contact bridge; it does not feed any impulse back into
     * the Sable aircraft.</p>
     */
    public static EntityContact sweptContactWorldAabb(
        BoundingBox box,
        double minX, double minY, double minZ,
        double maxX, double maxY, double maxZ
    ) {
        OrientedBox current = BOXES.get(box);
        OrientedBox target = axisAligned(minX, minY, minZ, maxX, maxY, maxZ);
        if (current == null || target == null) {
            return null;
        }

        ContactAxis currentContact = minimumTranslation(current, target);
        if (currentContact != null) {
            return new EntityContact(
                currentContact.normal.x, currentContact.normal.y, currentContact.normal.z,
                currentContact.penetration, 1.0
            );
        }

        if (current.owner == null || current.localCenter == null) {
            return null;
        }
        OrientedBox previous = atOwnerPose(current, 0.0);
        if (previous == null) {
            return null;
        }

        Vector3d ownerDelta = new Vector3d(
            current.owner.position.x - current.owner.prevPosition.x,
            current.owner.position.y - current.owner.prevPosition.y,
            current.owner.position.z - current.owner.prevPosition.z
        );
        Quaterniond previousQ = SablePoseConversions.toQuaternion(current.owner.prevOrientation);
        Quaterniond currentQ = SablePoseConversions.toQuaternion(current.owner.orientation);
        double dot = Math.min(1.0, Math.abs(previousQ.dot(currentQ)));
        double angle = 2.0 * Math.acos(dot);
        double radius = current.localCenter.length() + current.halfExtents.length();
        double travel = ownerDelta.length() + Math.abs(angle) * radius;
        double characteristic = Math.max(
            0.125,
            0.5 * Math.min(
                Math.min(current.halfExtents.x, current.halfExtents.y),
                Math.min(current.halfExtents.z,
                    Math.min(target.halfExtents.x, Math.min(target.halfExtents.y, target.halfExtents.z)))
            )
        );
        int steps = Math.max(1, Math.min(32, (int) Math.ceil(travel / characteristic)));

        double previousT = 0.0;
        ContactAxis previousContact = minimumTranslation(previous, target);
        if (previousContact != null) {
            return new EntityContact(
                previousContact.normal.x, previousContact.normal.y, previousContact.normal.z,
                previousContact.penetration, 0.0
            );
        }

        for (int i = 1; i <= steps; ++i) {
            double t = (double) i / steps;
            OrientedBox sample = atOwnerPose(current, t);
            ContactAxis contact = minimumTranslation(sample, target);
            if (contact == null) {
                previousT = t;
                continue;
            }

            // Refine the first overlap boundary. Eight bisections are enough to
            // make the remaining positional error far below Minecraft entity
            // collision tolerances without turning this into a physics solver.
            double low = previousT;
            double high = t;
            ContactAxis refined = contact;
            for (int iteration = 0; iteration < 8; ++iteration) {
                double mid = (low + high) * 0.5;
                ContactAxis midContact = minimumTranslation(atOwnerPose(current, mid), target);
                if (midContact == null) {
                    low = mid;
                } else {
                    high = mid;
                    refined = midContact;
                }
            }
            return new EntityContact(
                refined.normal.x, refined.normal.y, refined.normal.z,
                Math.max(0.0, refined.penetration), high
            );
        }
        return null;
    }

    private static OrientedBox atOwnerPose(OrientedBox template, double t) {
        if (template == null || template.owner == null || template.localCenter == null) {
            return null;
        }
        t = Math.max(0.0, Math.min(1.0, t));
        Quaterniond previous = SablePoseConversions.toQuaternion(template.owner.prevOrientation);
        Quaterniond current = SablePoseConversions.toQuaternion(template.owner.orientation);
        Quaterniond orientation = new Quaterniond(previous).slerp(current, t).normalize();
        Vector3d position = new Vector3d(
            template.owner.prevPosition.x + (template.owner.position.x - template.owner.prevPosition.x) * t,
            template.owner.prevPosition.y + (template.owner.position.y - template.owner.prevPosition.y) * t,
            template.owner.prevPosition.z + (template.owner.position.z - template.owner.prevPosition.z) * t
        );
        Vector3d center = new Quaterniond(orientation).transform(new Vector3d(template.localCenter)).add(position);
        return new OrientedBox(center, orientation, new Vector3d(template.halfExtents));
    }

    private static ContactAxis minimumTranslation(OrientedBox aircraft, OrientedBox entity) {
        if (aircraft == null || entity == null) {
            return null;
        }
        Vector3d[] aAxes = aircraft.axes();
        Vector3d[] worldAxes = {
            new Vector3d(1.0, 0.0, 0.0),
            new Vector3d(0.0, 1.0, 0.0),
            new Vector3d(0.0, 0.0, 1.0)
        };
        Vector3d centerDelta = new Vector3d(entity.center).sub(aircraft.center);
        double bestOverlap = Double.POSITIVE_INFINITY;
        Vector3d bestNormal = null;

        Vector3d[] candidateAxes = new Vector3d[15];
        int index = 0;
        for (Vector3d axis : aAxes) candidateAxes[index++] = new Vector3d(axis);
        for (Vector3d axis : worldAxes) candidateAxes[index++] = new Vector3d(axis);
        for (Vector3d aAxis : aAxes) {
            for (Vector3d worldAxis : worldAxes) {
                candidateAxes[index++] = new Vector3d(aAxis).cross(worldAxis);
            }
        }

        for (Vector3d axis : candidateAxes) {
            if (axis == null || axis.lengthSquared() <= SAT_EPSILON) {
                continue;
            }
            axis.normalize();
            double ra = aircraft.halfExtents.x * Math.abs(axis.dot(aAxes[0]))
                + aircraft.halfExtents.y * Math.abs(axis.dot(aAxes[1]))
                + aircraft.halfExtents.z * Math.abs(axis.dot(aAxes[2]));
            double rb = entity.halfExtents.x * Math.abs(axis.x)
                + entity.halfExtents.y * Math.abs(axis.y)
                + entity.halfExtents.z * Math.abs(axis.z);
            double signedDistance = centerDelta.dot(axis);
            double overlap = ra + rb - Math.abs(signedDistance);
            if (overlap <= EPSILON) {
                return null;
            }
            if (overlap < bestOverlap) {
                bestOverlap = overlap;
                bestNormal = signedDistance >= 0.0 ? new Vector3d(axis) : new Vector3d(axis).negate();
            }
        }
        return bestNormal == null ? null : new ContactAxis(bestNormal, bestOverlap);
    }

    /**
     * Highest world-Y intersection of a vertical line with a registered OBB.
     * Used only for IV's vanilla-entity ride/move-along presentation layer, so
     * standing entities follow the actual pitched/rolled surface rather than
     * the old globalCenter.y + heightRadius AABB top.
     */
    public static Double verticalTopAt(BoundingBox box, double worldX, double worldZ) {
        OrientedBox oriented = BOXES.get(box);
        if (oriented == null || !Double.isFinite(worldX) || !Double.isFinite(worldZ)) {
            return null;
        }
        Quaterniond inverse = new Quaterniond(oriented.orientation).conjugate();
        Vector3d origin = inverse.transform(new Vector3d(
            worldX - oriented.center.x,
            0.0,
            worldZ - oriented.center.z
        ));
        Vector3d direction = inverse.transform(new Vector3d(0.0, 1.0, 0.0));
        RayInterval interval = rayInterval(
            origin, direction, oriented.halfExtents,
            Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY
        );
        if (interval == null || !Double.isFinite(interval.exit())) {
            return null;
        }
        return oriented.center.y + interval.exit();
    }

    public record WorldAabb(
        double minX, double minY, double minZ,
        double maxX, double maxY, double maxZ
    ) {
    }

    public static Boolean isPointInside(BoundingBox box, Point3D point, Point3D growthOffset) {
        OrientedBox oriented = BOXES.get(box);
        if (oriented == null || point == null) {
            return null;
        }
        Vector3d local = oriented.worldToLocal(point);
        // IV treats growthOffset as signed per-axis growth. Preserve that
        // behavior exactly rather than turning a negative shrink into growth.
        double gx = growthOffset == null ? 0.0 : growthOffset.x;
        double gy = growthOffset == null ? 0.0 : growthOffset.y;
        double gz = growthOffset == null ? 0.0 : growthOffset.z;
        return Math.abs(local.x) <= oriented.halfExtents.x + gx
            && Math.abs(local.y) <= oriented.halfExtents.y + gy
            && Math.abs(local.z) <= oriented.halfExtents.z + gz;
    }

    /**
     * Exact world-vertical "inside XZ and below" test used by IV rain/height
     * helpers. A world-up ray from the point must intersect the oriented box.
     */
    public static Boolean isPointInsideAndBelow(BoundingBox box, Point3D point) {
        OrientedBox oriented = BOXES.get(box);
        if (oriented == null || point == null) {
            return null;
        }
        Vector3d origin = oriented.worldToLocal(point);
        Vector3d direction = oriented.worldDirectionToLocal(new Vector3d(0.0, 1.0, 0.0));
        RayInterval interval = rayInterval(origin, direction, oriented.halfExtents, 0.0, Double.POSITIVE_INFINITY);
        return interval != null && interval.exit() >= Math.max(0.0, interval.enter());
    }

    /**
     * Exact OBB/OBB SAT when at least one box is registered. Unregistered boxes
     * are interpreted exactly as IV normally does: world-axis-aligned cuboids.
     */
    public static Boolean intersects(BoundingBox first, BoundingBox second) {
        OrientedBox a = BOXES.get(first);
        OrientedBox b = BOXES.get(second);
        if (a == null && b == null) {
            return null;
        }
        if (a == null) {
            a = axisAligned(first);
        }
        if (b == null) {
            b = axisAligned(second);
        }
        if (a == null || b == null) {
            return false;
        }
        return intersectsSat(a, b);
    }

    /** Exact line-segment intersection against the live OBB. */
    public static BoundingBoxHitResult getIntersection(BoundingBox box, Point3D start, Point3D end) {
        OrientedBox oriented = BOXES.get(box);
        if (oriented == null || start == null || end == null) {
            return null;
        }

        Vector3d localStart = oriented.worldToLocal(start);
        Vector3d localEnd = oriented.worldToLocal(end);
        Vector3d direction = new Vector3d(localEnd).sub(localStart);
        if (direction.lengthSquared() <= EPSILON * EPSILON) {
            return null;
        }

        RayInterval interval = rayInterval(localStart, direction, oriented.halfExtents, 0.0, 1.0);
        if (interval == null) {
            return null;
        }

        boolean startsInside = Math.abs(localStart.x) <= oriented.halfExtents.x + EPSILON
            && Math.abs(localStart.y) <= oriented.halfExtents.y + EPSILON
            && Math.abs(localStart.z) <= oriented.halfExtents.z + EPSILON;
        double t = startsInside ? interval.exit() : interval.enter();
        Vector3d localNormal = startsInside ? interval.exitNormal() : interval.enterNormal();
        if (!Double.isFinite(t) || t < -EPSILON || t > 1.0 + EPSILON || localNormal == null) {
            return null;
        }
        t = Math.max(0.0, Math.min(1.0, t));

        Point3D hit = new Point3D(
            start.x + (end.x - start.x) * t,
            start.y + (end.y - start.y) * t,
            start.z + (end.z - start.z) * t
        );
        Vector3d worldNormal = new Quaterniond(oriented.orientation).transform(localNormal);
        return new BoundingBoxHitResult(box, hit, closestWorldAxis(worldNormal));
    }

    private static OrientedBox resolveQueryBox(
        BoundingBox box,
        AEntityD_Definable<?> fallbackOwner,
        boolean allowAuxiliary
    ) {
        OrientedBox registered = BOXES.get(box);
        if (registered != null) {
            return registered;
        }
        if (!allowAuxiliary || box == null || fallbackOwner == null || box.collisionTypes != null) {
            return null;
        }
        EntityVehicleF_Physics vehicle = fallbackOwner instanceof EntityVehicleF_Physics direct
            ? direct
            : fallbackOwner instanceof APart part ? part.vehicleOn : null;
        if (!LandingGearSolver.shouldReplaceIvGroundOperations(vehicle) || box.globalCenter == null) {
            return null;
        }
        double hx = Math.abs(box.widthRadius);
        double hy = Math.abs(box.heightRadius);
        double hz = Math.abs(box.depthRadius);
        if (!finitePositive(hx) || !finitePositive(hy) || !finitePositive(hz)
            || !Double.isFinite(box.globalCenter.x)
            || !Double.isFinite(box.globalCenter.y)
            || !Double.isFinite(box.globalCenter.z)) {
            return null;
        }
        Quaterniond orientation = SablePoseConversions.toQuaternion(fallbackOwner.orientation);
        if (!finite(orientation)) {
            return null;
        }
        return new OrientedBox(
            new Vector3d(box.globalCenter.x, box.globalCenter.y, box.globalCenter.z),
            orientation,
            new Vector3d(hx, hy, hz)
        );
    }

    private static WorldAabb worldAabb(OrientedBox oriented) {
        if (oriented == null) {
            return null;
        }
        Vector3d[] axes = oriented.axes();
        double worldHx = Math.abs(axes[0].x) * oriented.halfExtents.x
            + Math.abs(axes[1].x) * oriented.halfExtents.y
            + Math.abs(axes[2].x) * oriented.halfExtents.z;
        double worldHy = Math.abs(axes[0].y) * oriented.halfExtents.x
            + Math.abs(axes[1].y) * oriented.halfExtents.y
            + Math.abs(axes[2].y) * oriented.halfExtents.z;
        double worldHz = Math.abs(axes[0].z) * oriented.halfExtents.x
            + Math.abs(axes[1].z) * oriented.halfExtents.y
            + Math.abs(axes[2].z) * oriented.halfExtents.z;
        return new WorldAabb(
            oriented.center.x - worldHx, oriented.center.y - worldHy, oriented.center.z - worldHz,
            oriented.center.x + worldHx, oriented.center.y + worldHy, oriented.center.z + worldHz
        );
    }

    private static OrientedBox axisAligned(
        double minX, double minY, double minZ,
        double maxX, double maxY, double maxZ
    ) {
        if (!Double.isFinite(minX) || !Double.isFinite(minY) || !Double.isFinite(minZ)
            || !Double.isFinite(maxX) || !Double.isFinite(maxY) || !Double.isFinite(maxZ)
            || maxX < minX || maxY < minY || maxZ < minZ) {
            return null;
        }
        return new OrientedBox(
            new Vector3d((minX + maxX) * 0.5, (minY + maxY) * 0.5, (minZ + maxZ) * 0.5),
            new Quaterniond(),
            new Vector3d((maxX - minX) * 0.5, (maxY - minY) * 0.5, (maxZ - minZ) * 0.5)
        );
    }

    private static double[] sweptAxisInterval(
        OrientedBox moving,
        OrientedBox target,
        Vector3d centerDelta,
        Vector3d velocity,
        Vector3d rawAxis
    ) {
        Vector3d axis = new Vector3d(rawAxis);
        double axisLengthSquared = axis.lengthSquared();
        if (axisLengthSquared <= EPSILON * EPSILON) {
            return new double[]{Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY};
        }
        if (Math.abs(axisLengthSquared - 1.0) > 1.0E-8) {
            axis.normalize();
        }

        Vector3d[] movingAxes = moving.axes();
        double movingRadius = moving.halfExtents.x * Math.abs(axis.dot(movingAxes[0]))
            + moving.halfExtents.y * Math.abs(axis.dot(movingAxes[1]))
            + moving.halfExtents.z * Math.abs(axis.dot(movingAxes[2]));
        double targetRadius = target.halfExtents.x * Math.abs(axis.x)
            + target.halfExtents.y * Math.abs(axis.y)
            + target.halfExtents.z * Math.abs(axis.z);
        double limit = movingRadius + targetRadius;
        double centerProjection = centerDelta.dot(axis);
        double speedProjection = velocity.dot(axis);

        if (Math.abs(speedProjection) <= EPSILON) {
            return Math.abs(centerProjection) <= limit + EPSILON
                ? new double[]{Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY}
                : null;
        }
        double t0 = (centerProjection - limit) / speedProjection;
        double t1 = (centerProjection + limit) / speedProjection;
        return t0 <= t1 ? new double[]{t0, t1} : new double[]{t1, t0};
    }

    private static OrientedBox axisAligned(BoundingBox box) {
        if (box == null || box.globalCenter == null) {
            return null;
        }
        double hx = Math.abs(box.widthRadius);
        double hy = Math.abs(box.heightRadius);
        double hz = Math.abs(box.depthRadius);
        if (!finitePositive(hx) || !finitePositive(hy) || !finitePositive(hz)) {
            return null;
        }
        return new OrientedBox(
            new Vector3d(box.globalCenter.x, box.globalCenter.y, box.globalCenter.z),
            new Quaterniond(),
            new Vector3d(hx, hy, hz)
        );
    }

    private static boolean intersectsSat(OrientedBox a, OrientedBox b) {
        Vector3d[] aAxis = a.axes();
        Vector3d[] bAxis = b.axes();
        double[] ae = {a.halfExtents.x, a.halfExtents.y, a.halfExtents.z};
        double[] be = {b.halfExtents.x, b.halfExtents.y, b.halfExtents.z};
        double[][] r = new double[3][3];
        double[][] absR = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                r[i][j] = aAxis[i].dot(bAxis[j]);
                absR[i][j] = Math.abs(r[i][j]) + SAT_EPSILON;
            }
        }

        Vector3d centerDelta = new Vector3d(b.center).sub(a.center);
        double[] t = {
            centerDelta.dot(aAxis[0]),
            centerDelta.dot(aAxis[1]),
            centerDelta.dot(aAxis[2])
        };

        // A's three face normals.
        for (int i = 0; i < 3; ++i) {
            double rb = be[0] * absR[i][0] + be[1] * absR[i][1] + be[2] * absR[i][2];
            if (Math.abs(t[i]) >= ae[i] + rb) {
                return false;
            }
        }

        // B's three face normals.
        for (int j = 0; j < 3; ++j) {
            double projected = Math.abs(t[0] * r[0][j] + t[1] * r[1][j] + t[2] * r[2][j]);
            double ra = ae[0] * absR[0][j] + ae[1] * absR[1][j] + ae[2] * absR[2][j];
            if (projected >= ra + be[j]) {
                return false;
            }
        }

        // Nine edge cross-products Ai x Bj.
        for (int i = 0; i < 3; ++i) {
            int i1 = (i + 1) % 3;
            int i2 = (i + 2) % 3;
            for (int j = 0; j < 3; ++j) {
                int j1 = (j + 1) % 3;
                int j2 = (j + 2) % 3;
                double ra = ae[i1] * absR[i2][j] + ae[i2] * absR[i1][j];
                double rb = be[j1] * absR[i][j2] + be[j2] * absR[i][j1];
                double projected = Math.abs(t[i2] * r[i1][j] - t[i1] * r[i2][j]);
                if (projected >= ra + rb) {
                    return false;
                }
            }
        }
        return true;
    }

    private static RayInterval rayInterval(
        Vector3d origin,
        Vector3d direction,
        Vector3d halfExtents,
        double initialEnter,
        double initialExit
    ) {
        double tEnter = initialEnter;
        double tExit = initialExit;
        Vector3d enterNormal = null;
        Vector3d exitNormal = null;

        AxisInterval x = axisInterval(origin.x, direction.x, halfExtents.x, 0);
        if (x == null) return null;
        if (x.enter > tEnter) {
            tEnter = x.enter;
            enterNormal = x.enterNormal;
        }
        if (x.exit < tExit) {
            tExit = x.exit;
            exitNormal = x.exitNormal;
        }
        if (tEnter > tExit) return null;

        AxisInterval y = axisInterval(origin.y, direction.y, halfExtents.y, 1);
        if (y == null) return null;
        if (y.enter > tEnter) {
            tEnter = y.enter;
            enterNormal = y.enterNormal;
        }
        if (y.exit < tExit) {
            tExit = y.exit;
            exitNormal = y.exitNormal;
        }
        if (tEnter > tExit) return null;

        AxisInterval z = axisInterval(origin.z, direction.z, halfExtents.z, 2);
        if (z == null) return null;
        if (z.enter > tEnter) {
            tEnter = z.enter;
            enterNormal = z.enterNormal;
        }
        if (z.exit < tExit) {
            tExit = z.exit;
            exitNormal = z.exitNormal;
        }
        if (tEnter > tExit) return null;

        return new RayInterval(tEnter, tExit, enterNormal, exitNormal);
    }

    private static AxisInterval axisInterval(double origin, double direction, double halfExtent, int axis) {
        if (Math.abs(direction) <= EPSILON) {
            if (origin < -halfExtent || origin > halfExtent) {
                return null;
            }
            return new AxisInterval(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, null, null);
        }

        double tMin = (-halfExtent - origin) / direction;
        double tMax = (halfExtent - origin) / direction;
        Vector3d minNormal = axisVector(axis, -1.0);
        Vector3d maxNormal = axisVector(axis, 1.0);
        if (tMin <= tMax) {
            return new AxisInterval(tMin, tMax, minNormal, maxNormal);
        }
        return new AxisInterval(tMax, tMin, maxNormal, minNormal);
    }

    private static Vector3d axisVector(int axis, double sign) {
        return switch (axis) {
            case 0 -> new Vector3d(sign, 0.0, 0.0);
            case 1 -> new Vector3d(0.0, sign, 0.0);
            default -> new Vector3d(0.0, 0.0, sign);
        };
    }

    private static Axis closestWorldAxis(Vector3d normal) {
        double ax = Math.abs(normal.x);
        double ay = Math.abs(normal.y);
        double az = Math.abs(normal.z);
        if (ay >= ax && ay >= az) {
            return normal.y >= 0.0 ? Axis.UP : Axis.DOWN;
        }
        if (ax >= az) {
            return normal.x >= 0.0 ? Axis.EAST : Axis.WEST;
        }
        return normal.z >= 0.0 ? Axis.SOUTH : Axis.NORTH;
    }

    private static boolean finitePositive(double value) {
        return Double.isFinite(value) && value > EPSILON;
    }

    private static boolean finite(Vector3d value) {
        return value != null
            && Double.isFinite(value.x)
            && Double.isFinite(value.y)
            && Double.isFinite(value.z);
    }

    private static boolean finite(Quaterniond q) {
        return Double.isFinite(q.x) && Double.isFinite(q.y)
            && Double.isFinite(q.z) && Double.isFinite(q.w)
            && q.lengthSquared() > EPSILON;
    }

    private record ContactAxis(Vector3d normal, double penetration) {
    }

    private record AxisInterval(double enter, double exit, Vector3d enterNormal, Vector3d exitNormal) {
    }

    private record RayInterval(double enter, double exit, Vector3d enterNormal, Vector3d exitNormal) {
    }

    private static final class OrientedBox {
        private final Vector3d center;
        private final Quaterniond orientation;
        private final Vector3d[] axes;
        private final Vector3d halfExtents;
        private final AEntityD_Definable<?> owner;
        private final Vector3d localCenter;

        private OrientedBox(Vector3d center, Quaterniond orientation, Vector3d halfExtents) {
            this(center, orientation, halfExtents, null, null);
        }

        private OrientedBox(
            Vector3d center,
            Quaterniond orientation,
            Vector3d halfExtents,
            AEntityD_Definable<?> owner,
            Vector3d localCenter
        ) {
            this.center = center;
            this.orientation = new Quaterniond(orientation).normalize();
            this.axes = new Vector3d[]{
                this.orientation.transform(new Vector3d(1.0, 0.0, 0.0)),
                this.orientation.transform(new Vector3d(0.0, 1.0, 0.0)),
                this.orientation.transform(new Vector3d(0.0, 0.0, 1.0))
            };
            this.halfExtents = halfExtents;
            this.owner = owner;
            this.localCenter = localCenter == null ? null : new Vector3d(localCenter);
        }

        private Vector3d worldToLocal(Point3D point) {
            return new Quaterniond(orientation).conjugate().transform(
                new Vector3d(point.x - center.x, point.y - center.y, point.z - center.z)
            );
        }

        private Vector3d worldDirectionToLocal(Vector3d direction) {
            return new Quaterniond(orientation).conjugate().transform(direction);
        }

        private Vector3d[] axes() {
            return axes;
        }
    }
}
