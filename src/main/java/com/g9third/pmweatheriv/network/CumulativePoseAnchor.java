package com.g9third.pmweatheriv.network;

import com.g9third.pmweatheriv.physics.Vec3d;

/** Absolute origin of IV's cumulative world translation and Euler angle stream. */
public record CumulativePoseAnchor(Vec3d positionOrigin, Vec3d angleOrigin) {
    public static CumulativePoseAnchor capture(Vec3d position, Vec3d angles,
                                               Vec3d translationDelta, Vec3d angleDelta) {
        return new CumulativePoseAnchor(position.subtract(translationDelta), angles.subtract(angleDelta));
    }
    public boolean finite() {
        return positionOrigin != null && angleOrigin != null
            && positionOrigin.isFinite() && angleOrigin.isFinite();
    }
    public Vec3d position(Vec3d translationDelta) { return positionOrigin.add(translationDelta); }
    public Vec3d angles(Vec3d angleDelta) { return angleOrigin.add(angleDelta); }
}
