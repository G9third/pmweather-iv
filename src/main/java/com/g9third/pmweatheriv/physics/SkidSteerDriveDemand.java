package com.g9third.pmweatheriv.physics;

/** Pure per-contact allocation for the IV-authored differential skid-steer request. */
public final class SkidSteerDriveDemand {
    private static final double YAW_RESPONSE_SECONDS = 0.25;
    private SkidSteerDriveDemand() {}

    /** Motorized skid contacts need a commanded drive interval, not passive static hold. */
    public static boolean usesMotorizedRolling(boolean authoredFreeRolling,
                                               boolean skidSteerDriven,
                                               boolean liquidSupport) {
        return !liquidSupport && (authoredFreeRolling || skidSteerDriven);
    }

    /** First-order rate response; the resulting demand is still tire-grip bounded. */
    public static double yawImpulse(double currentRateRadiansPerSecond,
                                    double targetRateRadiansPerSecond,
                                    double yawInertiaKgM2, double timeStepSeconds) {
        if (!Double.isFinite(currentRateRadiansPerSecond)
            || !Double.isFinite(targetRateRadiansPerSecond)
            || !(yawInertiaKgM2 > 0.0) || !Double.isFinite(yawInertiaKgM2)
            || !(timeStepSeconds > 0.0) || !Double.isFinite(timeStepSeconds)) return 0.0;
        double fraction = Vec3d.clamp(timeStepSeconds / YAW_RESPONSE_SECONDS, 0.0, 1.0);
        return yawInertiaKgM2 * (targetRateRadiansPerSecond - currentRateRadiansPerSecond) * fraction;
    }

    /**
     * Splits net engine impulse evenly, then adds a zero-net-longitudinal side
     * differential whose moment about body Y is the requested yaw impulse.
     * The tire solver remains responsible for clamping each result to that
     * contact's live normal load and friction ellipse.
     */
    public static double[] allocate(double totalDriveImpulseNs, double yawImpulseNmS,
                                    double[] leverXBodyMeters, boolean[] driven) {
        return allocate(totalDriveImpulseNs, yawImpulseNmS, leverXBodyMeters, leverXBodyMeters, driven);
    }

    /**
     * Variant that classifies left/right about the vehicle model centerline
     * while applying the differential moment at COM-relative lever arms.
     */
    public static double[] allocate(double totalDriveImpulseNs, double yawImpulseNmS,
                                    double[] leverXBodyMeters, double[] sideXBodyMeters,
                                    boolean[] driven) {
        if (leverXBodyMeters == null || sideXBodyMeters == null || driven == null
            || leverXBodyMeters.length != driven.length || sideXBodyMeters.length != driven.length)
            throw new IllegalArgumentException("Skid-steer contact arrays must have equal length");
        if (!Double.isFinite(totalDriveImpulseNs) || !Double.isFinite(yawImpulseNmS))
            throw new IllegalArgumentException("Skid-steer demand must be finite");

        int count = 0;
        boolean left = false, right = false;
        double meanX = 0.0;
        for (int i = 0; i < driven.length; ++i) {
            if (!driven[i] || !Double.isFinite(leverXBodyMeters[i])) continue;
            ++count;
            double leverX = leverXBodyMeters[i];
            double sideX = sideXBodyMeters[i];
            left |= sideX < -1.0e-6;
            right |= sideX > 1.0e-6;
            meanX += leverX;
        }
        double[] result = new double[driven.length];
        if (count == 0) return result;
        double common = totalDriveImpulseNs / count;
        meanX /= count;
        double momentDenominator = 0.0;
        if (left && right) {
            for (int i = 0; i < driven.length; ++i) {
                if (!driven[i] || !Double.isFinite(leverXBodyMeters[i])) continue;
                double centeredX = leverXBodyMeters[i] - meanX;
                momentDenominator += centeredX * centeredX;
            }
        }
        boolean canTurn = left && right && momentDenominator > 1.0e-12;
        for (int i = 0; i < driven.length; ++i) {
            if (!driven[i] || !Double.isFinite(leverXBodyMeters[i])) continue;
            result[i] = common;
            if (canTurn) {
                double centeredX = leverXBodyMeters[i] - meanX;
                result[i] -= yawImpulseNmS * centeredX / momentDenominator;
            }
        }
        return result;
    }
}
