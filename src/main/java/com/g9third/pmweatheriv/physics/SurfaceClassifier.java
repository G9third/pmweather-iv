package com.g9third.pmweatheriv.physics;

import java.util.Map;

import static com.g9third.pmweatheriv.physics.ModelGeometryData.AnimationHint;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.Bounds;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.RawTriangle;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.SurfaceKind;
import static com.g9third.pmweatheriv.physics.ModelAnimationHints.normalize;

/** Ordered classification policy: controls, explicit names, wing, tail, then body. */
final class SurfaceClassifier {
    private SurfaceClassifier() {}

    static SurfaceKind classify(
        RawTriangle triangle,
        Bounds bounds,
        Map<String, AnimationHint> animationHints,
        boolean rotorcraft
    ) {
        String normalizedName = normalize(triangle.objectName());
        SurfaceKind named = SurfaceKind.fromSpecificName(normalizedName);
        if (rotorcraft) {
            // Rotorcraft stabilizers, fins, and fuselage surfaces all participate in
            // direct body pressure. Their rotor controls are handled separately, so
            // removing visually wing-like triangles from the pressure shell would
            // make helicopters artificially insensitive to wind.
            return named != null && named.isBodyPressure() ? named : SurfaceKind.BODY;
        }

        Vec3d p = bounds.normalized(triangle.centroid());
        Vec3d n = triangle.normal();
        double lateral = Math.abs(n.x());
        double vertical = Math.abs(n.y());
        double longitudinal = Math.abs(n.z());
        boolean farAft = p.z() < -0.56;
        boolean veryFarAft = p.z() < -0.72;
        boolean wide = Math.abs(p.x()) > 0.34;
        boolean veryWide = Math.abs(p.x()) > 0.58;
        boolean high = p.y() > 0.05;
        boolean low = p.y() < -0.58;
        boolean horizontalSurface = vertical > 0.40 && vertical >= lateral * 0.72;
        boolean verticalSurface = lateral > 0.40 && lateral >= vertical * 0.72;

        SurfaceKind controlled = ControlSurfaceClassifier.classify(normalizedName, named,
            animationHints.get(normalizedName), bounds, triangle.objectBounds(),
            horizontalSurface, verticalSurface, wide, farAft, high);
        if (controlled != null) return controlled;
        if (named != null) return named;
        SurfaceKind wing = WingSurfaceClassifier.classify(bounds, wide, veryWide, horizontalSurface);
        if (wing != null) return wing;
        SurfaceKind tail = TailSurfaceClassifier.classify(bounds, veryFarAft, farAft,
            verticalSurface, horizontalSurface, high);
        if (tail != null) return tail;
        return BodySurfaceClassifier.classify(bounds, farAft, low, longitudinal, lateral, vertical);
    }
}
