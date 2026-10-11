package com.g9third.pmweatheriv.physics;

/** Narrow tire strike geometry and preservation of material-resolved normal targets. */
public final class GearImpactContact {
    private GearImpactContact() {}

    /** Loaded tire cap, growing with actual compression/penetration rather than full diameter. */
    public static double area(double width, double diameter, double deflection, double penetration,
                              boolean wheel) {
        if (!Double.isFinite(width) || !Double.isFinite(diameter) || width <= 0 || diameter <= 0)
            return 0.0;
        if (!wheel) return width * diameter;
        if (!Double.isFinite(deflection) || !Double.isFinite(penetration)) return 0.0;
        double radius = diameter * 0.5;
        double depth = Math.min(radius, Math.max(0.0, deflection) + Math.max(0.0, penetration));
        return width * 2.0 * Math.sqrt(Math.max(0.0, depth * (2.0 * radius - depth)));
    }

    /** Material arrest (including exactly zero residual) must not become a recovery launch. */
    public static double normalTarget(double materialTarget, boolean materialResolved,
            double penetrationError, double rate, double dt) {
        if (materialResolved) return Double.isFinite(materialTarget) ? Math.min(0.0, materialTarget) : 0.0;
        if (Double.isFinite(materialTarget) && materialTarget < -1.0E-9) return materialTarget;
        return penetrationError > 0.0 && dt > 0.0
            ? penetrationError * (1.0 - Math.exp(-rate * dt)) / dt
            : Double.isFinite(materialTarget) ? Math.min(0.0, materialTarget) : 0.0;
    }
}
