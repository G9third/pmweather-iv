package com.g9third.pmweatheriv.physics;

import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.*;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.*;
import static com.g9third.pmweatheriv.physics.ModelAnimationHints.*;
import static com.g9third.pmweatheriv.physics.ModelTriangleSampling.plausibleAnimatedSurface;

/** One stage of the ordered surface-classification policy. */
final class ControlSurfaceClassifier {
    private ControlSurfaceClassifier() {}

    static SurfaceKind classify(
        String normalizedName, SurfaceKind named, AnimationHint hint, Bounds bounds,
        Bounds objectBounds, boolean horizontalSurface, boolean verticalSurface,
        boolean wide, boolean farAft, boolean high
    ) {
        if (hint != null && !looksLikeCockpitOrGear(normalizedName)) {
            // A content pack's primary control animation is stronger evidence than
            // a generic mesh noun.  This matters for spoilerons/split surfaces
            // whose objects are legitimately named "spoiler" but are driven by
            // IV's aileron variable. Single-axis controls still only override
            // null/generic-WING naming. Mixed pitch+roll animation may also refine
            // an explicit canard/tail noun because the authored animation states
            // that the complete named object is a moving control surface.
            boolean genericWingName = named == null || named == SurfaceKind.WING;
            boolean objectHorizontal = plausibleAnimatedSurface(objectBounds, hint)
                && objectBounds != null && objectBounds.valid()
                && objectBounds.spanX() >= Math.max(0.45, objectBounds.spanY() * 1.35);
            Vec3d objectCenterNormalized = objectBounds != null && objectBounds.valid()
                ? bounds.normalized(objectBounds.minimum().add(objectBounds.maximum()).scale(0.5))
                : null;
            boolean objectFarAft = objectCenterNormalized != null
                && objectCenterNormalized.z() < -0.50;
            boolean objectForward = objectCenterNormalized != null
                && objectCenterNormalized.z() > 0.20;
            boolean mixedHorizontal = hint.aileron() && hint.elevator()
                && (horizontalSurface || objectHorizontal);

            if (mixedHorizontal) {
                // IV-authored mixed pitch+roll animation describes one moving
                // physical surface.  Object-level bounds are deliberately allowed
                // to override individual triangle normals: a thick/all-moving tail
                // or canard has vertical side faces, but those faces rotate with the
                // same control and must not be thrown back into BODY pressure.
                if (named == SurfaceKind.CANARD
                    || (objectForward && containsAny(normalizedName, "canard", "foreplane"))) {
                    return SurfaceKind.CANARDERON;
                }
                if ((farAft || objectFarAft)
                    && containsAny(normalizedName, "tail", "stab")) {
                    return SurfaceKind.TAILERON;
                }
                if (genericWingName) {
                    return SurfaceKind.ELEVON;
                }
            }
            if (genericWingName && hint.aileron() && horizontalSurface && wide) {
                return SurfaceKind.AILERON;
            }
            if (genericWingName && hint.elevator() && horizontalSurface && farAft) {
                return SurfaceKind.ELEVATOR;
            }
            if (genericWingName && hint.rudder() && verticalSurface && farAft && high) {
                return SurfaceKind.RUDDER;
            }
            if (genericWingName && (hint.flap() || hint.slat() || hint.spoiler())
                && horizontalSurface && wide) {
                return SurfaceKind.WING;
            }
        }
        return null;
    }
}
