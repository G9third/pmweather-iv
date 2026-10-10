package com.g9third.pmweatheriv.physics;

import minecrafttransportsimulator.baseclasses.RotationMatrix;
import static com.g9third.pmweatheriv.physics.FlightMath.*;

/** Point friction bounded by a contact's already-applied normal impulse. */
public final class BodyContactFriction {
    private BodyContactFriction() {}
    public record Result(Vec3d velocityWorld, Vec3d angularVelocityWorld, Vec3d impulseWorld) {}
    public static Result solve(Vec3d velocity, Vec3d omegaWorld, RotationMatrix orientation,
                               Vec3d leverWorld, Vec3d normalWorld, Vec3d inertia,
                               double mass, double normalImpulse, double coefficient) {
        Result unchanged = new Result(velocity, omegaWorld, Vec3d.ZERO);
        if (velocity == null || omegaWorld == null || leverWorld == null || normalWorld == null
            || inertia == null || orientation == null || !velocity.isFinite() || !omegaWorld.isFinite()
            || !leverWorld.isFinite() || !normalWorld.isFinite() || !inertia.isFinite()
            || !(mass > 0.0) || !Double.isFinite(mass) || !(normalImpulse > 0.0)
            || !Double.isFinite(normalImpulse) || !(coefficient > 0.0) || !Double.isFinite(coefficient)
            || Math.min(inertia.x(), Math.min(inertia.y(), inertia.z())) <= 0.0) return unchanged;
        Vec3d n = normalWorld.normalized();
        if (n.lengthSquared() < 0.5) return unchanged;
        Vec3d pointVelocity = velocity.add(omegaWorld.cross(leverWorld));
        Vec3d tangent = pointVelocity.subtract(n.scale(pointVelocity.dot(n)));
        if (tangent.lengthSquared() < 1.0e-18) return unchanged;
        Vec3d t0 = tangent.normalized(), t1 = n.cross(t0).normalized();
        Vec3d lever = toLocal(orientation, leverWorld);
        Vec3d jn = lever.cross(toLocal(orientation, n));
        Vec3d a = lever.cross(toLocal(orientation, t0)), b = lever.cross(toLocal(orientation, t1));
        double k00 = 1.0 / mass + inertiaProduct(a, a, inertia);
        double k11 = 1.0 / mass + inertiaProduct(b, b, inertia);
        double k01 = inertiaProduct(a, b, inertia);
        double c0 = inertiaProduct(jn, a, inertia), c1 = inertiaProduct(jn, b, inertia);
        double cSquared = c0*c0 + c1*c1;
        double step = 0.8 / Math.max(k00 + Math.abs(k01), k11 + Math.abs(k01));
        double limit = normalImpulse * coefficient;
        if (!Double.isFinite(limit) || !Double.isFinite(step)) return unchanged;
        double v0 = pointVelocity.dot(t0), v1 = pointVelocity.dot(t1), x = 0.0, y = 0.0;
        for (int i = 0; i < 128; ++i) {
            double nx = x - step*(v0 + k00*x + k01*y);
            double ny = y - step*(v1 + k01*x + k11*y);
            // Tangent friction may not undo the normal/material response by creating inward motion.
            double normalChange = c0*nx + c1*ny;
            if (normalChange < 0.0 && cSquared > 1.0e-24) {
                nx -= normalChange*c0/cSquared; ny -= normalChange*c1/cSquared;
            }
            double length = Math.hypot(nx, ny);
            if (length > limit) { nx *= limit/length; ny *= limit/length; }
            double change = Math.max(Math.abs(nx - x), Math.abs(ny - y));
            x = nx; y = ny;
            if (change < 1.0e-7) break;
        }
        Vec3d impulse = t0.scale(x).add(t1.scale(y));
        Vec3d angularImpulse = a.scale(x).add(b.scale(y));
        Vec3d deltaOmegaBody = new Vec3d(angularImpulse.x()/inertia.x(),
            angularImpulse.y()/inertia.y(), angularImpulse.z()/inertia.z());
        return new Result(velocity.add(impulse.scale(1.0/mass)),
            omegaWorld.add(toWorld(orientation, deltaOmegaBody)), impulse);
    }
    private static double inertiaProduct(Vec3d a, Vec3d b, Vec3d inertia) {
        return a.x()*b.x()/inertia.x() + a.y()*b.y()/inertia.y() + a.z()*b.z()/inertia.z();
    }
}
