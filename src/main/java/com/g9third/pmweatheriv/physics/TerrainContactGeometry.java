package com.g9third.pmweatheriv.physics;

import org.joml.Vector3d;
import org.joml.Vector3dc;

/** Localizes a terrain contact to the intersection of the two collision shapes. */
public final class TerrainContactGeometry {
    private TerrainContactGeometry() {}
    private static final double TOLERANCE = 1.0e-8;

    /**
     * Averages the clipped edges of an oriented cuboid and a terrain AABB.
     * Every retained point belongs to both shapes, so their average cannot move
     * the contact onto an unrelated corner of a long wing or chassis box.
     * The work is bounded at 24 edges and is used only after a positive SAT hit.
     */
    public static Vector3d contactPoint(Vector3dc center, Vector3dc axisX,
            Vector3dc axisY, Vector3dc axisZ, double hx, double hy, double hz,
            Vector3dc minimum, Vector3dc maximum) {
        Vector3dc[] axes = {axisX, axisY, axisZ};
        double[] half = {hx, hy, hz};
        Vector3d[] cuboid = new Vector3d[8], terrain = new Vector3d[8];
        for (int i = 0; i < 8; ++i) {
            cuboid[i] = new Vector3d(center);
            for (int a = 0; a < 3; ++a)
                cuboid[i].fma((i & (1 << a)) == 0 ? -half[a] : half[a], axes[a]);
            terrain[i] = new Vector3d((i & 1) == 0 ? minimum.x() : maximum.x(),
                (i & 2) == 0 ? minimum.y() : maximum.y(),
                (i & 4) == 0 ? minimum.z() : maximum.z());
        }
        Vector3d sum = new Vector3d();
        int count = 0;
        for (int i = 0; i < 8; ++i) for (int a = 0; a < 3; ++a) {
            if ((i & (1 << a)) != 0) continue;
            int other = i | (1 << a);
            count += clip(cuboid[i], cuboid[other], minimum, maximum, null, null, null, sum);
            count += clip(terrain[i], terrain[other], null, null, center, axes, half, sum);
        }
        return count == 0 ? null : sum.div(count);
    }

    private static int clip(Vector3dc start, Vector3dc end, Vector3dc minimum,
            Vector3dc maximum, Vector3dc center, Vector3dc[] axes, double[] half,
            Vector3d sum) {
        double lower = 0.0, upper = 1.0;
        double dx = end.x()-start.x(), dy = end.y()-start.y(), dz = end.z()-start.z();
        for (int a = 0; a < 3; ++a) {
            double origin, direction, min, max;
            if (axes == null) {
                origin = start.get(a);
                direction = a == 0 ? dx : a == 1 ? dy : dz;
                min = minimum.get(a)-TOLERANCE;
                max = maximum.get(a)+TOLERANCE;
            } else {
                Vector3dc axis = axes[a];
                origin = (start.x()-center.x())*axis.x()
                    + (start.y()-center.y())*axis.y() + (start.z()-center.z())*axis.z();
                direction = dx*axis.x()+dy*axis.y()+dz*axis.z();
                min = -half[a]-TOLERANCE;
                max = half[a]+TOLERANCE;
            }
            if (Math.abs(direction) < 1.0e-12) {
                if (origin < min || origin > max) return 0;
            } else {
                double t0 = (min-origin)/direction, t1 = (max-origin)/direction;
                lower = Math.max(lower, Math.min(t0, t1));
                upper = Math.min(upper, Math.max(t0, t1));
                if (lower > upper) return 0;
            }
        }
        double midpoint = 0.5*(lower+upper);
        sum.add(start.x()+midpoint*dx, start.y()+midpoint*dy, start.z()+midpoint*dz);
        return 1;
    }
}
