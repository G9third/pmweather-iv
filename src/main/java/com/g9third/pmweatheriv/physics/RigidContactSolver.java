package com.g9third.pmweatheriv.physics;

import java.util.List;
import org.joml.Quaterniond;
import org.joml.Vector3d;
import org.joml.Vector3dc;

/** Inelastic point constraints evaluated in the pose where their impulses are applied. */
public final class RigidContactSolver {
    private RigidContactSolver() {}
    public record Contact(Vector3d pointWorld, Vector3d normalWorld, double residualInwardMps) {}
    public record Result(Vector3d linearWorld, Vector3d angularWorld, double[] normalImpulses,
                         double[] inwardBefore, int iterations, double maximumResidualMps) {}

    public static Result solve(Vector3dc linear, Vector3dc angular, Vector3dc position,
                               Quaterniond orientation, Vec3d inertia, double mass,
                               List<Contact> contacts) {
        Vector3d v = new Vector3d(linear), w = new Vector3d(angular);
        double[] impulses = new double[contacts.size()], before = new double[contacts.size()];
        Vector3d[] normals = new Vector3d[contacts.size()], levers = new Vector3d[contacts.size()],
            deltaAngular = new Vector3d[contacts.size()];
        double[] denominators = new double[contacts.size()];
        Quaterniond q = new Quaterniond(orientation).normalize();
        if (!(mass > 0.0) || !Double.isFinite(mass) || !inertia.isFinite()
            || Math.min(inertia.x(), Math.min(inertia.y(), inertia.z())) <= 0.0)
            throw new IllegalArgumentException("Invalid rigid contact mass or inertia");
        for (int i = 0; i < contacts.size(); ++i) {
            Contact c = contacts.get(i);
            normals[i] = new Vector3d(c.normalWorld()).normalize();
            levers[i] = new Vector3d(c.pointWorld()).sub(position);
            Vector3d jacobian = q.transformInverse(levers[i], new Vector3d())
                .cross(q.transformInverse(normals[i], new Vector3d()));
            denominators[i] = 1.0 / mass + jacobian.x*jacobian.x/inertia.x()
                + jacobian.y*jacobian.y/inertia.y() + jacobian.z*jacobian.z/inertia.z();
            deltaAngular[i] = q.transform(new Vector3d(jacobian.x/inertia.x(),
                jacobian.y/inertia.y(), jacobian.z/inertia.z()));
            before[i] = Math.max(0.0, -pointSpeed(v, w, levers[i], normals[i]));
        }
        int iterations = 0;
        double residual = 0.0;
        // Accumulated unilateral impulses allow a later contact to release an
        // earlier constraint, avoiding order-dependent artificial rebounds.
        for (; iterations < 64; ++iterations) {
            double maximumChange = 0.0;
            for (int i = 0; i < contacts.size(); ++i) {
                double speed = pointSpeed(v, w, levers[i], normals[i]);
                double target = Math.max(0.0, contacts.get(i).residualInwardMps());
                double next = Math.max(0.0, impulses[i] - (speed + target)/denominators[i]);
                double change = next - impulses[i];
                impulses[i] = next;
                v.fma(change/mass, normals[i]);
                w.fma(change, deltaAngular[i]);
                maximumChange = Math.max(maximumChange, Math.abs(change)/mass);
            }
            residual = maximumResidual(v, w, contacts, levers, normals);
            if (maximumChange < 1.0E-6 && residual < 1.0E-5) { ++iterations; break; }
        }
        return new Result(v, w, impulses, before, iterations, residual);
    }

    private static double pointSpeed(Vector3dc v, Vector3dc w, Vector3dc r, Vector3dc n) {
        return (v.x() + w.y()*r.z() - w.z()*r.y())*n.x()
            + (v.y() + w.z()*r.x() - w.x()*r.z())*n.y()
            + (v.z() + w.x()*r.y() - w.y()*r.x())*n.z();
    }
    private static double maximumResidual(Vector3dc v, Vector3dc w, List<Contact> contacts,
                                         Vector3d[] levers, Vector3d[] normals) {
        double residual = 0.0;
        for (int i = 0; i < contacts.size(); ++i)
            residual = Math.max(residual, -pointSpeed(v, w, levers[i], normals[i])
                - Math.max(0.0, contacts.get(i).residualInwardMps()));
        return residual;
    }
}
