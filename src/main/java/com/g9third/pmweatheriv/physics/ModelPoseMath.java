package com.g9third.pmweatheriv.physics;

import minecrafttransportsimulator.baseclasses.RotationMatrix;
import static com.g9third.pmweatheriv.physics.AnimatedWingGeometry.RigidTransform;

/** Immutable affine composition for raw OBJ, resolved part, and master-model frames. */
final class ModelPoseMath {
    private ModelPoseMath() {}

    /** Composition order matches IV: the right operand acts first. */
    static RigidTransform compose(RigidTransform left, RigidTransform right) {
        Vec3d x = left.direction(right.direction(new Vec3d(1, 0, 0)));
        Vec3d y = left.direction(right.direction(new Vec3d(0, 1, 0)));
        Vec3d z = left.direction(right.direction(new Vec3d(0, 0, 1)));
        return columns(x, y, z, left.point(right.translation()));
    }

    static RigidTransform inverse(RigidTransform matrix) {
        double a=matrix.m00(), b=matrix.m01(), c=matrix.m02(), d=matrix.m10(), e=matrix.m11(),
            f=matrix.m12(), g=matrix.m20(), h=matrix.m21(), i=matrix.m22();
        double det=a*(e*i-f*h)-b*(d*i-f*g)+c*(d*h-e*g);
        if (!Double.isFinite(det) || Math.abs(det)<1E-10) return null;
        RigidTransform inverse = new RigidTransform((e*i-f*h)/det, (c*h-b*i)/det, (b*f-c*e)/det,
            (f*g-d*i)/det, (a*i-c*g)/det, (c*d-a*f)/det,
            (d*h-e*g)/det, (b*g-a*h)/det, (a*e-b*d)/det, Vec3d.ZERO);
        return new RigidTransform(inverse.m00(), inverse.m01(), inverse.m02(), inverse.m10(),
            inverse.m11(), inverse.m12(), inverse.m20(), inverse.m21(), inverse.m22(),
            inverse.direction(matrix.translation()).negate());
    }

    /** Part orientation/offset/scale are already resolved by IV, including nested parents. */
    static RigidTransform resolvedPart(RotationMatrix master, RotationMatrix part, Vec3d offset, Vec3d scale) {
        if (master==null || part==null || offset==null || scale==null || !offset.isFinite() || !scale.isFinite()
            || Math.abs(scale.x()*scale.y()*scale.z())<1E-10) return null;
        Vec3d x=FlightMath.toLocal(master, FlightMath.toWorld(part, new Vec3d(scale.x(), 0, 0)));
        Vec3d y=FlightMath.toLocal(master, FlightMath.toWorld(part, new Vec3d(0, scale.y(), 0)));
        Vec3d z=FlightMath.toLocal(master, FlightMath.toWorld(part, new Vec3d(0, 0, scale.z())));
        return x.isFinite() && y.isFinite() && z.isFinite() ? columns(x, y, z, offset) : null;
    }

    private static RigidTransform columns(Vec3d x, Vec3d y, Vec3d z, Vec3d translation) {
        return new RigidTransform(x.x(), y.x(), z.x(), x.y(), y.y(), z.y(), x.z(), y.z(), z.z(), translation);
    }
}
