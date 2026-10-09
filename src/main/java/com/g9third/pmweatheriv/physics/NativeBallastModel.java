package com.g9third.pmweatheriv.physics;

/** SI adapter for IV's authored ballast controls and liquid ballast gravity modifier. */
public final class NativeBallastModel {
    private static final double IV_AIR_DENSITY = 1.225;
    private static final double NEUTRAL_DAMPING_THRESHOLD_MOTION = 0.15;
    private static final double SI_GRAVITY = 9.80665;

    private NativeBallastModel() {}

    public record Loads(double verticalControlForceNewtons, double verticalGravityCorrectionNewtons) {
        public double netWorldVerticalForceNewtons() {
            return verticalControlForceNewtons + verticalGravityCorrectionNewtons;
        }
    }

    /**
     * Ports EntityVehicleF_Physics' ballast force equations. Inputs and outputs
     * are finite-checked, while IV-authored volume, control, gravity and water
     * factors retain their ordinary scale. Native motion.y is reconstructed
     * from the current physical m/s state without writing back to IV or Sable.
     */
    public static Loads evaluate(double ballastVolume, boolean ballastVolumeActive, boolean outOfHealth,
            double ballastControl, double nativeVerticalMotion, double airDensity,
            double currentMassKg, double bodyMassKg, double gravityFactor, boolean waterFactorActive,
            double waterBallastFactor, boolean vehicleCenterInLiquid, double ivForceUnitNewtons) {
        double forceScale = Double.isFinite(ivForceUnitNewtons) && ivForceUnitNewtons > 0.0
            ? ivForceUnitNewtons : 0.0;
        double controlForce = 0.0;
        if (forceScale > 0.0 && !outOfHealth && Double.isFinite(ballastVolume) && ballastVolume > 0.0
            && Double.isFinite(ballastControl) && Double.isFinite(nativeVerticalMotion)) {
            double density = Double.isFinite(airDensity) && airDensity > 0.0
                ? airDensity : IV_AIR_DENSITY;
            double nativeForce;
            if (ballastControl < 0.0) {
                nativeForce = density * ballastVolume * -ballastControl / 10.0;
            } else if (ballastControl > 0.0) {
                nativeForce = -IV_AIR_DENSITY * ballastVolume * ballastControl / 10.0;
            } else if (Math.abs(nativeVerticalMotion) > NEUTRAL_DAMPING_THRESHOLD_MOTION) {
                nativeForce = IV_AIR_DENSITY * ballastVolume * 10.0 * -nativeVerticalMotion;
            } else {
                nativeForce = 0.0;
            }
            if (nativeVerticalMotion * nativeForce != 0.0) {
                double damping = 1.0 + Math.abs(nativeVerticalMotion);
                nativeForce /= damping * damping;
            }
            if (Double.isFinite(nativeForce)) controlForce = nativeForce * forceScale;
        }

        // Keep ordinary managed bodies at standard SI gravity. IV's native
        // 0.0245 * mass * speedFactor value is a game-unit scaling artifact;
        // applying it to Sable would make ordinary road vehicles dramatically
        // lighter, and vary their gravity with speedFactor. Preserve IV's
        // authored multipliers as ratios over physical body weight instead.
        // A healthy active ballast variable replaces gravity; a wreck restores
        // it. WaterBallastFactor applies whenever active and the center is in
        // liquid, whether or not a ballast volume is authored.
        double gravityCorrection = 0.0;
        if (Double.isFinite(bodyMassKg) && bodyMassKg > 0.0
            && Double.isFinite(gravityFactor)) {
            double gravityMultiplier = gravityFactor;
            if (ballastVolumeActive && !outOfHealth) {
                gravityMultiplier = 0.0;
            }
            if (waterFactorActive && vehicleCenterInLiquid && Double.isFinite(waterBallastFactor)) {
                gravityMultiplier *= 1.0 - waterBallastFactor;
            }
            double targetDownwardForce = bodyMassKg * SI_GRAVITY * gravityMultiplier;
            gravityCorrection = bodyMassKg * SI_GRAVITY - targetDownwardForce;
        }
        if (!Double.isFinite(controlForce)) controlForce = 0.0;
        if (!Double.isFinite(gravityCorrection)) gravityCorrection = 0.0;
        return new Loads(controlForce, gravityCorrection);
    }

    public static double ivForceUnitNewtons(double speedFactor) {
        return Double.isFinite(speedFactor) && speedFactor > 1.0e-9
            ? 400.0 * speedFactor : 400.0e-9;
    }
}
