package com.g9third.pmweatheriv.sable;

import minecrafttransportsimulator.baseclasses.RotationMatrix;
import org.joml.Quaterniond;
import org.joml.Quaterniondc;
import org.joml.Vector3d;
import org.joml.Vector3dc;

/** Coordinate conversions between MTS rotation matrices and Sable/JOML poses. */
public final class SablePoseConversions {
    private static final double EPSILON = 1.0E-12;

    private SablePoseConversions() {
    }

    public static Quaterniond toQuaternion(RotationMatrix matrix) {
        double trace = matrix.m00 + matrix.m11 + matrix.m22;
        double x;
        double y;
        double z;
        double w;
        if (trace > 0.0) {
            double s = Math.sqrt(trace + 1.0) * 2.0;
            w = 0.25 * s;
            x = (matrix.m21 - matrix.m12) / s;
            y = (matrix.m02 - matrix.m20) / s;
            z = (matrix.m10 - matrix.m01) / s;
        } else if (matrix.m00 > matrix.m11 && matrix.m00 > matrix.m22) {
            double s = Math.sqrt(1.0 + matrix.m00 - matrix.m11 - matrix.m22) * 2.0;
            w = (matrix.m21 - matrix.m12) / s;
            x = 0.25 * s;
            y = (matrix.m01 + matrix.m10) / s;
            z = (matrix.m02 + matrix.m20) / s;
        } else if (matrix.m11 > matrix.m22) {
            double s = Math.sqrt(1.0 + matrix.m11 - matrix.m00 - matrix.m22) * 2.0;
            w = (matrix.m02 - matrix.m20) / s;
            x = (matrix.m01 + matrix.m10) / s;
            y = 0.25 * s;
            z = (matrix.m12 + matrix.m21) / s;
        } else {
            double s = Math.sqrt(1.0 + matrix.m22 - matrix.m00 - matrix.m11) * 2.0;
            w = (matrix.m10 - matrix.m01) / s;
            x = (matrix.m02 + matrix.m20) / s;
            y = (matrix.m12 + matrix.m21) / s;
            z = 0.25 * s;
        }
        Quaterniond result = new Quaterniond(x, y, z, w);
        return result.lengthSquared() > EPSILON ? result.normalize() : result.identity();
    }

    public static void writeRotationMatrix(Quaterniondc quaternion, RotationMatrix destination) {
        Quaterniond q = new Quaterniond(quaternion);
        if (q.lengthSquared() <= EPSILON) {
            q.identity();
        } else {
            q.normalize();
        }
        double x = q.x;
        double y = q.y;
        double z = q.z;
        double w = q.w;
        double xx = x * x;
        double yy = y * y;
        double zz = z * z;
        double xy = x * y;
        double xz = x * z;
        double yz = y * z;
        double wx = w * x;
        double wy = w * y;
        double wz = w * z;

        destination.m00 = 1.0 - 2.0 * (yy + zz);
        destination.m01 = 2.0 * (xy - wz);
        destination.m02 = 2.0 * (xz + wy);
        destination.m10 = 2.0 * (xy + wz);
        destination.m11 = 1.0 - 2.0 * (xx + zz);
        destination.m12 = 2.0 * (yz - wx);
        destination.m20 = 2.0 * (xz - wy);
        destination.m21 = 2.0 * (yz + wx);
        destination.m22 = 1.0 - 2.0 * (xx + yy);
        destination.convertToAngles();
    }


    /** Maximum disagreement between MTS and quaternion-transformed body axes. */
    public static Vector3d worldToLocal(Quaterniondc orientation, Vector3dc world, Vector3d destination) {
        return new Quaterniond(orientation).conjugate().transform(world, destination);
    }

    public static Vector3d localToWorld(Quaterniondc orientation, Vector3dc local, Vector3d destination) {
        return new Quaterniond(orientation).transform(local, destination);
    }
}
