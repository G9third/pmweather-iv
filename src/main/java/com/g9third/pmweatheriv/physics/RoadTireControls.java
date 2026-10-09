package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.mixin.EntityVehicleMovingAccessor;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;

/** Native IV steering converted to wheel angles or a bounded differential yaw-rate request. */
public final class RoadTireControls {
    private RoadTireControls() {}

    public record Steering(double wheelAngleDegrees, boolean skidSteer,
                           double yawRateTargetRadiansPerSecond) {}

    /** Samples IV's native request once, preserving its authored skid-steer activation rules. */
    public static Steering capture(EntityVehicleF_Physics vehicle) {
        LandingGearSolver.maintainIvGroundAnimationState(vehicle);
        double request = ((EntityVehicleMovingAccessor) vehicle).pmweatherIv$getTurningForce();
        if (!Double.isFinite(request)) return new Steering(0.0, false, 0.0);

        // IV's getTurningForce returns steering angle / 20 for native skid steer.
        // Ground operations apply that request once per 20 Hz owner tick. Convert
        // it to the equivalent world yaw rate for the persistent Sable substeps.
        if (vehicle.skidSteerActive) {
            double rate = Math.toRadians(request * 20.0);
            return new Steering(0.0, true, Vec3d.clamp(rate, -Math.PI / 2.0, Math.PI / 2.0));
        }

        double wheelbase = vehicle.groundDeviceCollective.getTurningWheelbase();
        double speed = vehicle.groundVelocity * Math.abs(vehicle.speedFactor) * 20.0;
        if (!(wheelbase > 0.0) || !Double.isFinite(wheelbase)) return new Steering(0.0, false, 0.0);
        // The native request already includes the authored force factor and speed curve.
        // Its reverse sign is applied by IV after getTurningForce, so this rolling angle
        // keeps the forward sign; reversing motion reverses yaw through the contacts.
        double angle = speed > 1.0e-5
            ? Math.toDegrees(Math.atan(Math.toRadians(request) * 20.0 * wheelbase / speed))
            : Double.isFinite(vehicle.rudderInputVar.currentValue)
                ? -vehicle.rudderInputVar.currentValue : 0.0;
        return new Steering(Double.isFinite(angle) ? angle : 0.0, false, 0.0);
    }

    /** Compatibility accessor for existing callers and diagnostics. */
    public static double steeringDegrees(EntityVehicleF_Physics vehicle) {
        return capture(vehicle).wheelAngleDegrees();
    }
}
