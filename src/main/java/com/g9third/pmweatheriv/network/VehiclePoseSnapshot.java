package com.g9third.pmweatheriv.network;

import com.g9third.pmweatheriv.physics.Vec3d;

/** Applied model-origin pose, independent of native cumulative movement counters. */
public record VehiclePoseSnapshot(Vec3d positionWorld, Vec3d anglesDegrees) {
    public boolean finite() {
        return positionWorld != null && anglesDegrees != null
            && positionWorld.isFinite() && anglesDegrees.isFinite();
    }
}
