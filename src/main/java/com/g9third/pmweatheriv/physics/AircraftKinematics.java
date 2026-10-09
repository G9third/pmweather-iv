package com.g9third.pmweatheriv.physics;

import minecrafttransportsimulator.baseclasses.RotationMatrix;

/** One physical state used consistently for owner-tick airflow and coordinate conversion. */
public record AircraftKinematics(Vec3d modelOriginWorld, RotationMatrix orientation,
                                 Vec3d centerVelocityWorld, Vec3d angularVelocityBody,
                                 long gameTime) {
    public AircraftKinematics {
        RotationMatrix copy = new RotationMatrix();
        copy.set(orientation);
        orientation = copy;
        if (!modelOriginWorld.isFinite() || !centerVelocityWorld.isFinite()
            || !angularVelocityBody.isFinite()) throw new IllegalArgumentException("Nonfinite aircraft state");
    }
}
