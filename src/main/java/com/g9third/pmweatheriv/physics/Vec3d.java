package com.g9third.pmweatheriv.physics;

/** Small immutable finite-safe vector used by the replacement flight solver. */
public record Vec3d(double x, double y, double z) {
    public static final Vec3d ZERO = new Vec3d(0.0, 0.0, 0.0);

    public Vec3d add(Vec3d other) {
        return new Vec3d(x + other.x, y + other.y, z + other.z);
    }

    public Vec3d subtract(Vec3d other) {
        return new Vec3d(x - other.x, y - other.y, z - other.z);
    }

    public Vec3d scale(double factor) {
        return new Vec3d(x * factor, y * factor, z * factor);
    }

    public Vec3d negate() {
        return new Vec3d(-x, -y, -z);
    }

    public double dot(Vec3d other) {
        return x * other.x + y * other.y + z * other.z;
    }

    public Vec3d cross(Vec3d other) {
        return new Vec3d(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    public double lengthSquared() {
        return x * x + y * y + z * z;
    }

    public double length() {
        return Math.sqrt(lengthSquared());
    }

    public Vec3d normalized() {
        double length = length();
        return length > 1.0E-9 && Double.isFinite(length) ? scale(1.0 / length) : ZERO;
    }

    public boolean isFinite() {
        return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z);
    }

    public static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}
