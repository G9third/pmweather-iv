package com.g9third.pmweatheriv.sable;

import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import java.util.ArrayList;
import java.util.List;
import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.jsondefs.JSONCollisionGroup.CollisionType;
import minecrafttransportsimulator.mcinterface.IWrapperEntity;
import minecrafttransportsimulator.mcinterface.IWrapperPlayer;
import org.joml.Quaterniond;
import org.joml.Vector3d;

/**
 * Vanilla-entity contact bridge for Sable-owned IV aircraft.
 *
 * <p>Vanilla players/mobs are not Rapier rigid bodies, so Sable cannot resolve
 * these contacts itself. This layer treats IV ENTITY boxes as semantic copies
 * of the aircraft's live oriented geometry, performs exact OBB-vs-vanilla-AABB
 * contact, moves only the vanilla entity out of penetration, and removes inward
 * relative normal velocity. Aircraft pose/momentum are never modified here.</p>
 */
public final class OrientedEntityContact {
    private static final double CONTACT_SLOP = 1.0E-4;
    private static final double QUERY_MARGIN = 2.0;
    private static final int MAX_DEPENETRATION_PASSES = 4;
    // IV clamps ENTITY box centers to 1/64 blocks for native collision shapes.
    private static final double EXIT_CLEARANCE = 1.0 / 32.0;

    private OrientedEntityContact() {
    }

    public static void resolve(EntityVehicleF_Physics vehicle) {
        if (vehicle == null || vehicle.world == null || vehicle.encompassingBox == null) {
            return;
        }

        BoundingBox query = sweptEntityQuery(vehicle);
        List<IWrapperEntity> nearby = vehicle.world.getEntitiesWithin(query);
        for (IWrapperEntity entity : nearby) {
            if (!eligible(entity)) {
                continue;
            }

            BoundingBox bounds = entity.getBounds();
            if (!valid(bounds)) {
                continue;
            }

            // If the aircraft crossed this entity between ticks but no current
            // OBB overlaps, transfer only the remaining surface-normal motion.
            // Current overlap resolution below then handles any residual.
            OrientedHitboxRegistry.EntityContact sweep = earliestSweptContact(vehicle, bounds);
            if (sweep != null && sweep.time() < 1.0 - 1.0E-9
                && !hasCurrentContact(vehicle, bounds)) {
                Vector3d normal = sweep.normal();
                Vector3d center = center(bounds);
                Vector3d remainingSurfaceMotion = remainingVehiclePointMotion(
                    vehicle, center, sweep.time()
                );
                double push = Math.max(CONTACT_SLOP, remainingSurfaceMotion.dot(normal));
                if (push > 0.0) {
                    SafeTranslation safe = terrainSafeTranslation(
                        vehicle, entity, bounds, new Vector3d(normal).mul(push), normal
                    );
                    if (safe != null) {
                        translate(entity, safe.delta(), safe.normal().y > 0.5);
                        bounds = entity.getBounds();
                        if (!valid(bounds)) {
                            continue;
                        }
                        removeInwardNormalVelocity(
                            entity, safe.normal(), remainingSurfaceMotion
                        );
                    }
                }
            }

            // Compound ENTITY definitions may overlap. Resolve the smallest
            // exact separating translation and repeat a few times rather than
            // summing mutually contradictory box corrections in one pass.
            for (int pass = 0; pass < MAX_DEPENETRATION_PASSES; ++pass) {
                ContactChoice choice = smallestCurrentContact(vehicle, bounds);
                if (choice == null) {
                    break;
                }
                Vector3d normal = choice.contact.normal();
                double push = choice.contact.penetration() + CONTACT_SLOP;
                SafeTranslation safe = terrainSafeTranslation(
                    vehicle, entity, bounds, new Vector3d(normal).mul(push), normal
                );
                if (safe == null) {
                    // Remaining overlapped for one tick is safer than forcing a
                    // vanilla entity through solid terrain. A later vehicle/entity
                    // pose may expose a valid exit direction.
                    break;
                }
                translate(entity, safe.delta(), safe.normal().y > 0.5);

                Vector3d surfaceMotion = fullVehiclePointMotion(vehicle, center(bounds));
                removeInwardNormalVelocity(entity, safe.normal(), surfaceMotion);

                bounds = entity.getBounds();
                if (!valid(bounds)) {
                    break;
                }
            }
        }
    }

    private static boolean eligible(IWrapperEntity entity) {
        return entity != null
            && entity.isValid()
            && entity.getEntityRiding() == null
            && (!(entity instanceof IWrapperPlayer player) || !player.isSpectator());
    }

    private static BoundingBox sweptEntityQuery(EntityVehicleF_Physics vehicle) {
        BoundingBox current = vehicle.encompassingBox;
        double localReachX = Math.abs(current.globalCenter.x - vehicle.position.x) + current.widthRadius;
        double localReachY = Math.abs(current.globalCenter.y - vehicle.position.y) + current.heightRadius;
        double localReachZ = Math.abs(current.globalCenter.z - vehicle.position.z) + current.depthRadius;
        double radius = Math.sqrt(
            localReachX * localReachX + localReachY * localReachY + localReachZ * localReachZ
        ) + QUERY_MARGIN;

        double dx = vehicle.position.x - vehicle.prevPosition.x;
        double dy = vehicle.position.y - vehicle.prevPosition.y;
        double dz = vehicle.position.z - vehicle.prevPosition.z;
        return new BoundingBox(
            new Point3D(
                (vehicle.position.x + vehicle.prevPosition.x) * 0.5,
                (vehicle.position.y + vehicle.prevPosition.y) * 0.5,
                (vehicle.position.z + vehicle.prevPosition.z) * 0.5
            ),
            radius + Math.abs(dx) * 0.5,
            radius + Math.abs(dy) * 0.5,
            radius + Math.abs(dz) * 0.5
        );
    }

    private static OrientedHitboxRegistry.EntityContact earliestSweptContact(
        EntityVehicleF_Physics vehicle,
        BoundingBox entityBounds
    ) {
        OrientedHitboxRegistry.EntityContact best = null;
        for (BoundingBox box : vehicle.allCollisionBoxes) {
            if (!isEntityBox(box)) {
                continue;
            }
            OrientedHitboxRegistry.EntityContact contact =
                OrientedHitboxRegistry.sweptContactWorldAabb(
                    box,
                    entityBounds.globalCenter.x - entityBounds.widthRadius,
                    entityBounds.globalCenter.y - entityBounds.heightRadius,
                    entityBounds.globalCenter.z - entityBounds.depthRadius,
                    entityBounds.globalCenter.x + entityBounds.widthRadius,
                    entityBounds.globalCenter.y + entityBounds.heightRadius,
                    entityBounds.globalCenter.z + entityBounds.depthRadius
                );
            if (contact != null && (best == null || contact.time() < best.time())) {
                best = contact;
            }
        }
        return best;
    }

    private static boolean hasCurrentContact(
        EntityVehicleF_Physics vehicle,
        BoundingBox entityBounds
    ) {
        return smallestCurrentContact(vehicle, entityBounds) != null;
    }

    private static ContactChoice smallestCurrentContact(
        EntityVehicleF_Physics vehicle,
        BoundingBox entityBounds
    ) {
        ContactChoice best = null;
        for (BoundingBox box : vehicle.allCollisionBoxes) {
            if (!isEntityBox(box)) {
                continue;
            }
            OrientedHitboxRegistry.EntityContact contact =
                OrientedHitboxRegistry.contactWorldAabb(
                    box,
                    entityBounds.globalCenter.x - entityBounds.widthRadius,
                    entityBounds.globalCenter.y - entityBounds.heightRadius,
                    entityBounds.globalCenter.z - entityBounds.depthRadius,
                    entityBounds.globalCenter.x + entityBounds.widthRadius,
                    entityBounds.globalCenter.y + entityBounds.heightRadius,
                    entityBounds.globalCenter.z + entityBounds.depthRadius
                );
            if (contact != null
                && (best == null || contact.penetration() < best.contact.penetration())) {
                best = new ContactChoice(box, contact);
            }
        }
        return best;
    }

    private static boolean isEntityBox(BoundingBox box) {
        return box != null
            && box.collisionTypes != null
            && box.collisionTypes.contains(CollisionType.ENTITY)
            && OrientedHitboxRegistry.contains(box);
    }

    /**
     * One placement-only escape for an unmounted creator overlapped by the
     * live collision compound or enclosed in its cabin. Runs after seating and
     * live box updates, never on chunk reload or on an established rider.
     */
    public static boolean clearPlacementOverlap(EntityVehicleF_Physics vehicle, IWrapperPlayer player) {
        if (player == null || !eligible(player)) return true;
        if (vehicle == null || vehicle.world.isClient() || vehicle.encompassingBox == null) return true;
        boolean geometryReady = vehicle.allCollisionBoxes.stream().anyMatch(OrientedEntityContact::isEntityBox);
        if (!geometryReady) return false;
        BoundingBox bounds = player.getBounds();
        if (!valid(bounds) || !bounds.intersects(vehicle.encompassingBox)) return true;
        boolean overlap = hasCurrentContact(vehicle, bounds);
        boolean enclosed = lastRayExit(vehicle, bounds, new Vector3d(1, 0, 0)) != null
            && lastRayExit(vehicle, bounds, new Vector3d(-1, 0, 0)) != null
            && lastRayExit(vehicle, bounds, new Vector3d(0, 0, 1)) != null
            && lastRayExit(vehicle, bounds, new Vector3d(0, 0, -1)) != null;
        if (!overlap && !enclosed) return true;

        List<Vector3d> directions = escapeDirections(vehicle, bounds, null);
        SafeTranslation safe = bestCompoundExit(vehicle, player, bounds, directions, true);
        if (safe == null) safe = bestCompoundExit(vehicle, player, bounds, directions, false);
        if (safe == null) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                "PLACEMENT_PLAYER_CLEARANCE_BLOCKED uuid=" + vehicle.uniqueUUID
                    + " reason=NO_TERRAIN_CLEAR_COMPOUND_EXIT entityMoved=false");
            return true;
        }
        Point3D before = player.getPosition().copy();
        translate(player, safe.delta(), false);
        if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
            "PLACEMENT_PLAYER_CLEARANCE uuid=" + vehicle.uniqueUUID
                + " from=" + before + " to=" + player.getPosition()
                + " overlap=" + overlap + " enclosed=" + enclosed
                + " delta=" + safe.delta() + " vehiclePoseChanged=false");
        return true;
    }

    /** Reject partial corrections which separate one box but enter another. */
    private static SafeTranslation terrainSafeTranslation(
        EntityVehicleF_Physics vehicle, IWrapperEntity entity, BoundingBox bounds,
        Vector3d requestedDelta, Vector3d requestedNormal
    ) {
        if (requestedDelta == null || !finite(requestedDelta)
            || requestedDelta.lengthSquared() <= 1.0E-18) return null;
        Vector3d normal = new Vector3d(requestedNormal);
        if (!finite(normal) || normal.lengthSquared() <= 1.0E-18) normal.set(requestedDelta);
        normal.normalize();
        BoundingBox requestedBounds = translatedBounds(bounds, requestedDelta);
        boolean terrainBlocked = terrainCollisionAlongTranslation(entity, bounds, requestedDelta);
        boolean compoundBlocked = hasCurrentContact(vehicle, requestedBounds);
        if (!terrainBlocked && !compoundBlocked && !otherVehicleContact(vehicle, requestedBounds)) {
            return new SafeTranslation(new Vector3d(requestedDelta), normal);
        }
        SafeTranslation safe = compoundBlocked && !terrainBlocked
            ? projectedCompoundSeparation(vehicle, entity, bounds, requestedDelta) : null;
        if (safe == null) safe = bestCompoundExit(vehicle, entity, bounds,
            escapeDirections(vehicle, bounds, normal), false);
        if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
            (safe == null ? "ENTITY_DEPENETRATION_BLOCKED" : "ENTITY_DEPENETRATION_REROUTED")
                + " uuid=" + vehicle.uniqueUUID + " entity=" + entity.getID()
                + " requestedDelta=" + requestedDelta
                + " safeDelta=" + (safe == null ? "null" : safe.delta())
                + " terrainBlocked=" + terrainBlocked + " compoundBlocked=" + compoundBlocked
                + " reason=REQUIRE_TERRAIN_CLEAR_COMPOUND_EXIT aircraftMomentumChanged=false");
        return safe;
    }

    /** Try a local corner separation before taking a complete compound exit. */
    private static SafeTranslation projectedCompoundSeparation(
        EntityVehicleF_Physics vehicle, IWrapperEntity entity, BoundingBox original,
        Vector3d firstCorrection
    ) {
        Vector3d delta = new Vector3d(firstCorrection);
        for (int iteration = 0; iteration < MAX_DEPENETRATION_PASSES * 2; ++iteration) {
            BoundingBox candidate = translatedBounds(original, delta);
            ContactChoice next = smallestCurrentContact(vehicle, candidate);
            if (next == null) {
                if (!finite(delta) || delta.lengthSquared() <= 1.0E-18
                    || terrainCollisionAlongTranslation(entity, original, delta)
                    || otherVehicleContact(vehicle, candidate)) return null;
                return new SafeTranslation(delta, new Vector3d(delta).normalize());
            }
            delta.fma(next.contact.penetration() + CONTACT_SLOP, next.contact.normal());
        }
        return null;
    }

    private static List<Vector3d> escapeDirections(
        EntityVehicleF_Physics vehicle, BoundingBox bounds, Vector3d preferred
    ) {
        List<Vector3d> directions = new ArrayList<>();
        if (preferred != null) directions.add(new Vector3d(preferred).normalize());
        Vector3d radial = center(bounds).sub(vehicle.position.x, vehicle.position.y, vehicle.position.z);
        radial.y = 0;
        if (radial.lengthSquared() > 1.0E-8) directions.add(radial.normalize());
        directions.add(new Vector3d(1, 0, 0));
        directions.add(new Vector3d(-1, 0, 0));
        directions.add(new Vector3d(0, 0, 1));
        directions.add(new Vector3d(0, 0, -1));
        directions.add(new Vector3d(0, 1, 0));
        return directions;
    }

    private static SafeTranslation bestCompoundExit(
        EntityVehicleF_Physics vehicle, IWrapperEntity entity, BoundingBox bounds,
        List<Vector3d> directions, boolean horizontalOnly
    ) {
        SafeTranslation best = null;
        for (Vector3d direction : directions) {
            if (horizontalOnly && Math.abs(direction.y) > 1.0E-9) continue;
            Double exit = lastRayExit(vehicle, bounds, direction);
            if (exit == null) continue;
            Vector3d delta = new Vector3d(direction).mul(exit + EXIT_CLEARANCE);
            if (best != null && delta.lengthSquared() >= best.delta().lengthSquared()) continue;
            BoundingBox candidate = translatedBounds(bounds, delta);
            if (terrainCollisionAlongTranslation(entity, bounds, delta) || hasCurrentContact(vehicle, candidate)
                || otherVehicleContact(vehicle, candidate)) continue;
            best = new SafeTranslation(delta, new Vector3d(direction));
        }
        return best;
    }

    /** Exit the entire ray-intersected compound, including closed cabin walls. */
    private static Double lastRayExit(EntityVehicleF_Physics vehicle, BoundingBox bounds, Vector3d direction) {
        Double exit = null;
        for (BoundingBox box : vehicle.allCollisionBoxes) {
            if (!isEntityBox(box)) continue;
            var interval = OrientedHitboxRegistry.translationContactInterval(box, bounds, direction);
            if (interval != null) exit = exit == null ? interval.exit() : Math.max(exit, interval.exit());
        }
        return exit;
    }

    private static boolean otherVehicleContact(EntityVehicleF_Physics vehicle, BoundingBox bounds) {
        for (EntityVehicleF_Physics other : vehicle.world.getEntitiesOfType(EntityVehicleF_Physics.class)) {
            if (other == vehicle || !other.isValid || other.encompassingBox == null
                || !bounds.intersects(other.encompassingBox)) continue;
            for (BoundingBox box : other.allCollisionBoxes) {
                if (box.collisionTypes != null && box.collisionTypes.contains(CollisionType.ENTITY)
                    && bounds.intersects(box)) return true;
            }
        }
        return false;
    }

    /** A conservative swept entity volume prevents exits through solid terrain. */
    private static boolean terrainCollisionAlongTranslation(
        IWrapperEntity entity, BoundingBox bounds, Vector3d delta
    ) {
        BoundingBox swept = new BoundingBox(
            new Point3D(bounds.globalCenter.x + delta.x * 0.5,
                bounds.globalCenter.y + delta.y * 0.5, bounds.globalCenter.z + delta.z * 0.5),
            bounds.widthRadius + Math.abs(delta.x) * 0.5,
            bounds.heightRadius + Math.abs(delta.y) * 0.5,
            bounds.depthRadius + Math.abs(delta.z) * 0.5
        );
        return terrainCollisionAtBounds(entity, swept);
    }

    private static boolean terrainCollisionAtBounds(
        IWrapperEntity entity, BoundingBox bounds
    ) {
        return entity.getWorld().checkForCollisions(
            bounds, new Point3D(), true, false
        );
    }

    private static BoundingBox translatedBounds(BoundingBox bounds, Vector3d delta) {
        return new BoundingBox(
            new Point3D(
                bounds.globalCenter.x + delta.x,
                bounds.globalCenter.y + delta.y,
                bounds.globalCenter.z + delta.z
            ),
            bounds.widthRadius,
            bounds.heightRadius,
            bounds.depthRadius
        );
    }

    private static void translate(IWrapperEntity entity, Vector3d delta, boolean onGround) {
        if (delta == null || !finite(delta) || delta.lengthSquared() <= 1.0E-18) {
            return;
        }
        Point3D position = entity.getPosition();
        entity.setPosition(
            new Point3D(position.x + delta.x, position.y + delta.y, position.z + delta.z),
            onGround
        );
    }

    private static void removeInwardNormalVelocity(
        IWrapperEntity entity,
        Vector3d rawNormal,
        Vector3d surfaceMotion
    ) {
        Vector3d normal = new Vector3d(rawNormal);
        if (!finite(normal) || normal.lengthSquared() <= 1.0E-18) {
            return;
        }
        normal.normalize();
        Point3D velocityPoint = entity.getVelocity();
        Vector3d velocity = new Vector3d(velocityPoint.x, velocityPoint.y, velocityPoint.z);
        Vector3d surface = surfaceMotion == null || !finite(surfaceMotion)
            ? new Vector3d()
            : new Vector3d(surfaceMotion);

        double relativeNormal = new Vector3d(velocity).sub(surface).dot(normal);
        if (relativeNormal < 0.0) {
            velocity.fma(-relativeNormal, normal);
            entity.setVelocity(new Point3D(velocity.x, velocity.y, velocity.z));
        }
    }

    /**
     * Surface motion at the entity center over the full previous->current
     * vehicle pose. Units are Minecraft blocks/tick, matching wrapper velocity.
     */
    private static Vector3d fullVehiclePointMotion(
        EntityVehicleF_Physics vehicle,
        Vector3d worldPoint
    ) {
        Quaterniond previous = SablePoseConversions.toQuaternion(vehicle.prevOrientation);
        Quaterniond current = SablePoseConversions.toQuaternion(vehicle.orientation);
        Vector3d local = new Quaterniond(current).conjugate().transform(
            new Vector3d(
                worldPoint.x - vehicle.position.x,
                worldPoint.y - vehicle.position.y,
                worldPoint.z - vehicle.position.z
            )
        );
        Vector3d previousPoint = new Quaterniond(previous).transform(local).add(
            vehicle.prevPosition.x, vehicle.prevPosition.y, vehicle.prevPosition.z
        );
        return new Vector3d(worldPoint).sub(previousPoint);
    }

    /** Remaining rigid-body point motion after a swept contact at t. */
    private static Vector3d remainingVehiclePointMotion(
        EntityVehicleF_Physics vehicle,
        Vector3d worldPoint,
        double t
    ) {
        t = Math.max(0.0, Math.min(1.0, t));
        Quaterniond previous = SablePoseConversions.toQuaternion(vehicle.prevOrientation);
        Quaterniond current = SablePoseConversions.toQuaternion(vehicle.orientation);
        Quaterniond atContact = new Quaterniond(previous).slerp(current, t).normalize();
        Vector3d positionAtContact = new Vector3d(
            vehicle.prevPosition.x + (vehicle.position.x - vehicle.prevPosition.x) * t,
            vehicle.prevPosition.y + (vehicle.position.y - vehicle.prevPosition.y) * t,
            vehicle.prevPosition.z + (vehicle.position.z - vehicle.prevPosition.z) * t
        );
        Vector3d local = new Quaterniond(atContact).conjugate().transform(
            new Vector3d(worldPoint).sub(positionAtContact)
        );
        Vector3d finalPoint = new Quaterniond(current).transform(local).add(
            vehicle.position.x, vehicle.position.y, vehicle.position.z
        );
        return finalPoint.sub(worldPoint);
    }

    private static Vector3d center(BoundingBox box) {
        return new Vector3d(box.globalCenter.x, box.globalCenter.y, box.globalCenter.z);
    }

    private static boolean valid(BoundingBox box) {
        return box != null
            && box.globalCenter != null
            && Double.isFinite(box.globalCenter.x)
            && Double.isFinite(box.globalCenter.y)
            && Double.isFinite(box.globalCenter.z)
            && Double.isFinite(box.widthRadius)
            && Double.isFinite(box.heightRadius)
            && Double.isFinite(box.depthRadius)
            && box.widthRadius > 0.0
            && box.heightRadius > 0.0
            && box.depthRadius > 0.0;
    }

    private static boolean finite(Vector3d vector) {
        return Double.isFinite(vector.x)
            && Double.isFinite(vector.y)
            && Double.isFinite(vector.z);
    }

    private record SafeTranslation(Vector3d delta, Vector3d normal) {
    }

    private record ContactChoice(
        BoundingBox box,
        OrientedHitboxRegistry.EntityContact contact
    ) {
    }
}
