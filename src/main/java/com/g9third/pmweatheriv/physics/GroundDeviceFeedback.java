package com.g9third.pmweatheriv.physics;

import minecrafttransportsimulator.entities.instances.PartGroundDevice;

/** Ground-relative presentation values at the installed device's support station. */
public final class GroundDeviceFeedback {
    // Presentation scale only: tire grip and the contact solver do not use this value.
    private static final double SLIP_PRESENTATION_SPEED_MPS = 1.0;

    private GroundDeviceFeedback() {}

    public record Motion(double speedMps, double slip, double slipDegrees) {}

    public static Motion motion(PartGroundDevice device) {
        if (device == null || device.vehicleOn == null) return new Motion(0, 0, 0);
        var vehicle = device.vehicleOn;
        double scale = Math.abs(vehicle.speedFactor) * 20.0;
        Vec3d velocity = new Vec3d(vehicle.motion.x, vehicle.motion.y, vehicle.motion.z).scale(scale);
        Vec3d omegaBody = new Vec3d(vehicle.rotation.angles.x, vehicle.rotation.angles.y,
            vehicle.rotation.angles.z).scale(Math.PI / 180.0 / FlightMath.MC_TICK_SECONDS);
        Vec3d station = LandingGearSolver.landingGearDeviceContactPointLocal(device, vehicle.orientation);
        if (station != null && station.isFinite()) {
            // IV motion is already model-origin velocity, so this lever is measured from that origin.
            velocity = velocity.add(FlightMath.toWorld(vehicle.orientation, omegaBody.cross(station)));
        }
        double speed = Math.hypot(velocity.x(), velocity.z());
        if (!Double.isFinite(speed)) return new Motion(0, 0, 0);
        // Direction-only slip is undefined at rest.
        if (speed == 0.0) return new Motion(speed, 0, 0);
        Vec3d forward = LandingGearSolver.landingGearWheelForwardWorld(vehicle.orientation,
            LandingGearSolver.landingGearSteeringDegrees(vehicle, device));
        Vec3d lateral = new Vec3d(forward.z(), 0, -forward.x());
        double fraction = Vec3d.clamp(velocity.dot(lateral) / speed, -1, 1);
        // Smoothly suppress the angle feedback from tiny support-station movements.
        // Actual speed remains available for wheel rotation and authored speed variables.
        double speedRatio = speed / Math.hypot(speed, SLIP_PRESENTATION_SPEED_MPS);
        double presentationWeight = speedRatio * speedRatio;
        return new Motion(speed, 75.0 * fraction * presentationWeight,
            -Math.toDegrees(Math.asin(fraction)) * presentationWeight);
    }
}
