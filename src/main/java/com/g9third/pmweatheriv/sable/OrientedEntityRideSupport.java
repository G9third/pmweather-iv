package com.g9third.pmweatheriv.sable;

import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.Set;
import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.jsondefs.JSONCollisionGroup.CollisionType;
import minecrafttransportsimulator.mcinterface.IWrapperEntity;
import minecrafttransportsimulator.mcinterface.IWrapperPlayer;
import net.minecraft.world.entity.MoverType;
import net.minecraft.world.entity.player.Player;
import net.minecraft.world.phys.Vec3;
import org.joml.Quaterniond;
import org.joml.Vector3d;

/** Carries a supported entity once from the previous surface pose, before contact repair. */
public final class OrientedEntityRideSupport {
    private static final double SUPPORT_GAP = 0.08;
    private static final double WALKABLE_NORMAL_Y = 0.65;
    private OrientedEntityRideSupport() {}

    public static Set<IWrapperEntity> moveAlongEntities(EntityVehicleF_Physics vehicle) {
        Set<IWrapperEntity> carried = Collections.newSetFromMap(new IdentityHashMap<>());
        if (vehicle == null || vehicle.world == null || vehicle.encompassingBox == null) return carried;
        Vector3d translation = new Vector3d(vehicle.position.x - vehicle.prevPosition.x,
            vehicle.position.y - vehicle.prevPosition.y, vehicle.position.z - vehicle.prevPosition.z);
        Quaterniond previous = SablePoseConversions.toQuaternion(vehicle.prevOrientation);
        Quaterniond current = SablePoseConversions.toQuaternion(vehicle.orientation);
        if (translation.lengthSquared() <= 1.0E-12 && 1 - Math.abs(previous.dot(current)) <= 1.0E-12)
            return carried;
        BoundingBox b = vehicle.encompassingBox;
        BoundingBox query = new BoundingBox(new Point3D(b.globalCenter.x - translation.x * .5,
            b.globalCenter.y - translation.y * .5, b.globalCenter.z - translation.z * .5),
            b.widthRadius + Math.abs(translation.x) * .5 + 1,
            b.heightRadius + Math.abs(translation.y) * .5 + 1,
            b.depthRadius + Math.abs(translation.z) * .5 + 1);
        for (IWrapperEntity entity : vehicle.world.getEntitiesWithin(query)) {
            if (entity == null || !entity.isValid() || entity.getEntityRiding() != null
                || entity instanceof IWrapperPlayer p && p.isSpectator()) continue;
            if (!(entity instanceof NativeEntityContactAccess access)) continue;
            var nativeEntity = access.pmweatherIv$contactEntity();
            if (nativeEntity.level().isClientSide
                && (!(nativeEntity instanceof Player p) || !p.isLocalPlayer())) continue;
            BoundingBox bounds = entity.getBounds();
            if (bounds == null || bounds.globalCenter == null || entity.getVelocity().y > .08) continue;
            OrientedHitboxRegistry.SupportSurface support = findSupport(vehicle, bounds, true);
            if (support == null) continue;
            Vector3d oldPoint = new Vector3d(support.x(), support.y(), support.z());
            Vector3d motion = pointMotion(oldPoint,
                new Vector3d(vehicle.prevPosition.x, vehicle.prevPosition.y, vehicle.prevPosition.z),
                new Vector3d(vehicle.position.x, vehicle.position.y, vehicle.position.z), previous, current);
            if (!motion.isFinite()) continue;
            // Native movement clips carry against terrain/other structures and updates
            // ordinary collision flags. No teleport, vertical snap or rigid attachment.
            nativeEntity.move(MoverType.SELF, new Vec3(motion.x, motion.y, motion.z));
            // An upward carry is not a jump. Native movement alone would clear
            // onGround because that displacement had no downward collision.
            // Retain support only if the feet still touch the current walkable face.
            if (findSupport(vehicle, entity.getBounds(), false) != null) {
                nativeEntity.setOnGround(true);
                nativeEntity.verticalCollisionBelow = true;
                nativeEntity.fallDistance = 0;
            }
            carried.add(entity);
            Vector3d heading = new Quaterniond(current).mul(new Quaterniond(previous).conjugate())
                .transform(new Vector3d(0, 0, 1));
            double yaw = Math.toDegrees(Math.atan2(heading.x, heading.z));
            if (Double.isFinite(yaw)) {
                entity.setYaw(entity.getYaw() + yaw);
                entity.setBodyYaw(entity.getBodyYaw() + yaw);
            }
        }
        return carried;
    }

    /** Rigid pose displacement of one support point, with no additional height correction. */
    public static Vector3d pointMotion(Vector3d point, Vector3d previousOrigin, Vector3d currentOrigin,
                                      Quaterniond previous, Quaterniond current) {
        Vector3d local = new Quaterniond(previous).conjugate().transform(new Vector3d(point).sub(previousOrigin));
        return new Quaterniond(current).transform(local).add(currentOrigin).sub(point);
    }

    private static OrientedHitboxRegistry.SupportSurface findSupport(
            EntityVehicleF_Physics vehicle, BoundingBox bounds, boolean previousPose) {
        if (bounds == null || bounds.globalCenter == null) return null;
        double feet = bounds.globalCenter.y - bounds.heightRadius;
        double x = bounds.globalCenter.x, z = bounds.globalCenter.z;
        double[][] samples = {{x,z}, {x-bounds.widthRadius,z-bounds.depthRadius},
            {x-bounds.widthRadius,z+bounds.depthRadius}, {x+bounds.widthRadius,z-bounds.depthRadius},
            {x+bounds.widthRadius,z+bounds.depthRadius}};
        OrientedHitboxRegistry.SupportSurface best = null;
        double error = SUPPORT_GAP;
        for (BoundingBox box : vehicle.allCollisionBoxes) {
            if (box == null || box.collisionTypes == null || !box.collisionTypes.contains(CollisionType.ENTITY)
                || !OrientedHitboxRegistry.contains(box)) continue;
            for (double[] sample : samples) {
                var support = OrientedHitboxRegistry.verticalSurfaceAt(box, sample[0], sample[1], previousPose);
                if (support == null || support.normal().y < WALKABLE_NORMAL_Y) continue;
                double gap = Math.abs(support.y() - feet);
                if (gap <= error) { best = support; error = gap; }
            }
        }
        return best;
    }
}
