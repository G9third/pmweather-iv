package com.g9third.pmweatheriv.physics;

import org.joml.Quaterniond;
import org.joml.Vector3d;

/** Generic bridge from IV's authored float and liquid-box flags to fluid support. */
public final class LiquidSupportModel {
    private LiquidSupportModel() {}

    /** Returns the world-space fluid top, or NaN when this support cannot float in it. */
    public static double surfaceY(double blockY, double fluidHeight, boolean canFloat) {
        if (!canFloat || !Double.isFinite(blockY) || !Double.isFinite(fluidHeight)
            || fluidHeight <= 0.0) return Double.NaN;
        return blockY + Vec3d.clamp(fluidHeight, 0.0, 1.0);
    }

    /** Submerged pontoon stations are supported, but are not rigid terrain penetrations. */
    public static double penetrationMeters(double signedGapMeters, boolean liquidSupport) {
        if (!Double.isFinite(signedGapMeters)) return 0.0;
        return liquidSupport ? 0.0 : Math.max(0.0, -signedGapMeters);
    }

    /** Fluid has no tire friction in this point-contact approximation. */
    public static boolean hasTireTraction(boolean liquidSupport) {
        return !liquidSupport;
    }

    /**
     * IV also gives authored BLOCK collision boxes an independent liquid-support
     * flag. Keep those hull stations eligible without interpreting unrelated
     * vehicle/entity query boxes or zero-volume boxes as buoyant support.
     */
    public static boolean isAuthoredLiquidCollisionBox(boolean blockCollision,
                                                        boolean collidesWithLiquids,
                                                        double halfWidth,
                                                        double halfHeight,
                                                        double halfDepth) {
        return blockCollision && collidesWithLiquids
            && Double.isFinite(halfWidth) && Math.abs(halfWidth) > 1.0e-9
            && Double.isFinite(halfHeight) && Math.abs(halfHeight) > 1.0e-9
            && Double.isFinite(halfDepth) && Math.abs(halfDepth) > 1.0e-9;
    }

    /**
     * Maps the four lower-face corners of an authored live box from its owner-
     * local frame into the vehicle frame used by Sable support contacts. IV has
     * already applied owner scale to the center and box radii before this call.
     */
    public static Vec3d[] bottomCornersVehicleLocal(Vec3d centerOwnerLocal,
                                                     double halfWidth,
                                                     double halfHeight,
                                                     double halfDepth,
                                                     Vec3d ownerOriginVehicleLocal,
                                                     Quaterniond ownerToVehicle) {
        if (centerOwnerLocal == null || !centerOwnerLocal.isFinite()
            || ownerOriginVehicleLocal == null || !ownerOriginVehicleLocal.isFinite()
            || !Double.isFinite(halfWidth) || !Double.isFinite(halfHeight)
            || !Double.isFinite(halfDepth) || ownerToVehicle == null
            || !Double.isFinite(ownerToVehicle.x) || !Double.isFinite(ownerToVehicle.y)
            || !Double.isFinite(ownerToVehicle.z) || !Double.isFinite(ownerToVehicle.w)) {
            return new Vec3d[0];
        }
        double hx = Math.abs(halfWidth);
        double hy = Math.abs(halfHeight);
        double hz = Math.abs(halfDepth);
        Vec3d[] corners = new Vec3d[4];
        int index = 0;
        for (int sx : new int[] {-1, 1}) {
            for (int sz : new int[] {-1, 1}) {
                Vector3d corner = new Vector3d(
                    centerOwnerLocal.x() + sx * hx,
                    centerOwnerLocal.y() - hy,
                    centerOwnerLocal.z() + sz * hz
                );
                ownerToVehicle.transform(corner);
                corner.add(ownerOriginVehicleLocal.x(), ownerOriginVehicleLocal.y(),
                    ownerOriginVehicleLocal.z());
                Vec3d station = new Vec3d(corner.x, corner.y, corner.z);
                if (!station.isFinite()) return new Vec3d[0];
                corners[index++] = station;
            }
        }
        return corners;
    }
}
