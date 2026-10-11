package com.g9third.pmweatheriv.terrain.trueimpact;

import java.util.Arrays;

/** Volume intersected by the material resolver's square contact patch during one slice. */
public final class SweptContactVolume {
    private SweptContactVolume() {}

    /** Square area in the dominant-axis plane, preserving the normal swept cross-section. */
    public static double planeArea(double normalArea, double dx, double dy, double dz) {
        double length = Math.sqrt(dx * dx + dy * dy + dz * dz);
        double dominant = Math.max(Math.abs(dx), Math.max(Math.abs(dy), Math.abs(dz)));
        if (!Double.isFinite(normalArea) || normalArea <= 0.0
                || !Double.isFinite(length) || length <= 1.0E-12) return 0.0;
        return normalArea * length / dominant;
    }

    /**
     * Intersects an extruded square with a voxel-local collision box. The path and box
     * share a frame; direction is normalized here and travel is measured in meters.
     * Cross-sectional overlaps are piecewise linear, so Simpson integration on their
     * edge crossings gives exact volume without sampling a grid or creating fragments.
     */
    public static double intersection(double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ,
            double startX, double startY, double startZ,
            double dx, double dy, double dz, double travel, double normalArea) {
        double[] min = {minX, minY, minZ};
        double[] max = {maxX, maxY, maxZ};
        double[] start = {startX, startY, startZ};
        double[] dir = {dx, dy, dz};
        for (int i = 0; i < 3; ++i) {
            if (!Double.isFinite(min[i]) || !Double.isFinite(max[i])
                    || !Double.isFinite(start[i]) || !Double.isFinite(dir[i])
                    || max[i] <= min[i]) return 0.0;
        }
        if (!Double.isFinite(travel) || travel <= 0.0) return 0.0;
        double area = planeArea(normalArea, dx, dy, dz);
        if (!(area > 0.0) || !Double.isFinite(area)) return 0.0;
        double length = Math.sqrt(dx * dx + dy * dy + dz * dz);
        for (int i = 0; i < 3; ++i) dir[i] /= length;
        double ax = Math.abs(dir[0]), ay = Math.abs(dir[1]), az = Math.abs(dir[2]);
        int axis = ay >= ax && ay >= az ? 1 : ax >= az ? 0 : 2;
        int a = (axis + 1) % 3, b = (axis + 2) % 3;
        double t1 = (min[axis] - start[axis]) / dir[axis];
        double t2 = (max[axis] - start[axis]) / dir[axis];
        double lo = Math.max(0.0, Math.min(t1, t2));
        double hi = Math.min(travel, Math.max(t1, t2));
        if (!(hi > lo)) return 0.0;
        double half = 0.5 * Math.sqrt(area);
        double[] cuts = new double[10];
        cuts[0] = lo; cuts[1] = hi;
        int count = 2;
        for (int i : new int[] {a, b}) {
            if (Math.abs(dir[i]) < 1.0E-12) continue;
            for (double edge : new double[] {min[i], max[i]}) {
                for (double side : new double[] {-half, half}) {
                    double t = (edge - start[i] - side) / dir[i];
                    if (t > lo && t < hi) cuts[count++] = t;
                }
            }
        }
        Arrays.sort(cuts, 0, count);
        double volume = 0.0;
        for (int i = 1; i < count; ++i) {
            double left = cuts[i - 1], right = cuts[i];
            double middle = 0.5 * (left + right);
            double f0 = overlap(min[a], max[a], start[a] + dir[a] * left, half)
                * overlap(min[b], max[b], start[b] + dir[b] * left, half);
            double f1 = overlap(min[a], max[a], start[a] + dir[a] * middle, half)
                * overlap(min[b], max[b], start[b] + dir[b] * middle, half);
            double f2 = overlap(min[a], max[a], start[a] + dir[a] * right, half)
                * overlap(min[b], max[b], start[b] + dir[b] * right, half);
            volume += (right - left) * (f0 + 4.0 * f1 + f2) / 6.0;
        }
        volume *= Math.abs(dir[axis]);
        double boxVolume = (maxX - minX) * (maxY - minY) * (maxZ - minZ);
        return Math.max(0.0, Math.min(boxVolume, Math.min(normalArea * travel, volume)));
    }

    private static double overlap(double min, double max, double center, double half) {
        return Math.max(0.0, Math.min(max, center + half) - Math.max(min, center - half));
    }
}
