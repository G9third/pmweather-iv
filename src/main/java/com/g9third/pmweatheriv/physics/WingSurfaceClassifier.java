package com.g9third.pmweatheriv.physics;

import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.*;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.*;
import static com.g9third.pmweatheriv.physics.ModelAnimationHints.*;

/** One stage of the ordered surface-classification policy. */
final class WingSurfaceClassifier {
    private WingSurfaceClassifier() {}

    static SurfaceKind classify(Bounds bounds, boolean wide, boolean veryWide, boolean horizontalSurface) {
        // Wide aft triangles on delta wings are still main wing, not an invented
        // horizontal stabilizer. Named/animated stabilizers have already matched.
        if (wide && horizontalSurface
            && (veryWide || bounds.spanX() > bounds.spanZ() * 0.62)) {
            return SurfaceKind.WING;
        }
        return null;
    }
}
