package com.g9third.pmweatheriv.physics;

import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.*;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.*;
import static com.g9third.pmweatheriv.physics.ModelAnimationHints.*;

/** One stage of the ordered surface-classification policy. */
final class BodySurfaceClassifier {
    private BodySurfaceClassifier() {}

    static SurfaceKind classify(Bounds bounds, boolean farAft, boolean low, double longitudinal, double lateral, double vertical) {
        if (farAft && bounds.spanZ() > bounds.spanX() * 0.55
            && longitudinal >= Math.min(lateral, vertical) * 0.65) {
            return SurfaceKind.TAIL_BOOM;
        }
        if (low) {
            return SurfaceKind.UNDERBODY;
        }
        return SurfaceKind.BODY;
    }
}
