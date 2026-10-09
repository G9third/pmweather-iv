package com.g9third.pmweatheriv.sable;

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
 * IV gameplay-only entity carry for a vehicle whose physical body is already
 * owned by Sable/Rapier. This replaces IV's AABB-top assumption with the live
 * oriented ENTITY surface while retaining IV's normal wrapper/entity movement.
 */
public final class OrientedEntityRideSupport {
    private static final double SUPPORT_WINDOW = 0.5;
    private static final double ENTITY_VERTICAL_GROWTH = 0.25;
    private static final double MOVEMENT_EPSILON_SQ = 1.0E-12;

    private OrientedEntityRideSupport() {
    }

    public static void moveAlongEntities(EntityVehicleF_Physics vehicle) {
        if (vehicle == null || vehicle.world == null || vehicle.encompassingBox == null) {
            return;
        }

        Vector3d translation = new Vector3d(
            vehicle.position.x - vehicle.prevPosition.x,
            vehicle.position.y - vehicle.prevPosition.y,
            vehicle.position.z - vehicle.prevPosition.z
        );
        Quaterniond previousOrientation = SablePoseConversions.toQuaternion(vehicle.prevOrientation);
        Quaterniond currentOrientation = SablePoseConversions.toQuaternion(vehicle.orientation);
        double angularDelta = 1.0 - Math.abs(previousOrientation.dot(currentOrientation));
        if (translation.lengthSquared() <= MOVEMENT_EPSILON_SQ && angularDelta <= 1.0E-12) {
            return;
        }

        List<IWrapperEntity> nearbyEntities = vehicle.world.getEntitiesWithin(vehicle.encompassingBox);
        for (IWrapperEntity entity : nearbyEntities) {
            if (entity == null || entity.getEntityRiding() != null
                || (entity instanceof IWrapperPlayer player && player.isSpectator())) {
                continue;
            }

            BoundingBox rawBounds = entity.getBounds();
            if (rawBounds == null || rawBounds.globalCenter == null) {
                continue;
            }
            BoundingBox expandedBounds = new BoundingBox(
                rawBounds.globalCenter.copy(),
                rawBounds.widthRadius,
                rawBounds.heightRadius + ENTITY_VERTICAL_GROWTH,
                rawBounds.depthRadius
            );

            for (BoundingBox box : vehicle.allCollisionBoxes) {
                if (box == null || box.collisionTypes == null
                    || !box.collisionTypes.contains(CollisionType.ENTITY)
                    || !expandedBounds.intersects(box)) {
                    continue;
                }

                Double supportY = OrientedHitboxRegistry.verticalTopAt(
                    box, rawBounds.globalCenter.x, rawBounds.globalCenter.z
                );
                if (supportY == null) {
                    supportY = box.globalCenter.y + box.heightRadius;
                }
                double entityBottom = rawBounds.globalCenter.y - rawBounds.heightRadius;
                double supportDelta = supportY - entityBottom;
                if (supportDelta < -SUPPORT_WINDOW || supportDelta > SUPPORT_WINDOW) {
                    continue;
                }

                Point3D entityVelocity = entity.getVelocity();
                if (entityVelocity != null
                    && entityVelocity.y > 0.0
                    && entityVelocity.y >= supportDelta) {
                    continue;
                }

                carryWithVehiclePose(vehicle, entity, supportDelta, previousOrientation, currentOrientation);
                break;
            }
        }
    }

    private static void carryWithVehiclePose(
        EntityVehicleF_Physics vehicle,
        IWrapperEntity entity,
        double supportDelta,
        Quaterniond previousOrientation,
        Quaterniond currentOrientation
    ) {
        Point3D oldPosition = entity.getPosition();
        Vector3d previousRelative = new Vector3d(
            oldPosition.x - vehicle.prevPosition.x,
            oldPosition.y - vehicle.prevPosition.y,
            oldPosition.z - vehicle.prevPosition.z
        );
        Vector3d local = new Quaterniond(previousOrientation).conjugate().transform(previousRelative);
        Vector3d carriedRelative = new Quaterniond(currentOrientation).transform(local);
        Vector3d carried = new Vector3d(carriedRelative).add(
            vehicle.position.x, vehicle.position.y, vehicle.position.z
        );
        carried.y += supportDelta;

        entity.setPosition(new Point3D(carried.x, carried.y, carried.z), true);

        // Preserve IV's existing behavior of yawing a free-standing entity as
        // the vehicle turns beneath it. Use the orbital horizontal angle, not
        // aircraft pitch/roll, so standing players are not forcibly tilted.
        double oldYaw = Math.toDegrees(Math.atan2(previousRelative.x, previousRelative.z));
        double newYaw = Math.toDegrees(Math.atan2(carriedRelative.x, carriedRelative.z));
        double yawDelta = wrapDegrees(newYaw - oldYaw);
        if (Double.isFinite(yawDelta)) {
            entity.setYaw(entity.getYaw() + yawDelta);
            entity.setBodyYaw(entity.getBodyYaw() + yawDelta);
        }
    }

    private static double wrapDegrees(double degrees) {
        double wrapped = degrees % 360.0;
        if (wrapped > 180.0) wrapped -= 360.0;
        if (wrapped < -180.0) wrapped += 360.0;
        return wrapped;
    }
}
