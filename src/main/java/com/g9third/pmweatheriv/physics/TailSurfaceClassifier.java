package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.jsondefs.JSONAnimatedObject;
import minecrafttransportsimulator.jsondefs.JSONAnimationDefinition;

import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.Bounds;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.BoundsAccumulator;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.ClassifiedTriangle;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.EPSILON;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.SurfaceKind;
import static com.g9third.pmweatheriv.physics.ModelAnimationHints.normalize;

/** Repairs tail/control classifications using authored hinge coherence. */
final class TailSurfaceClassifier {
    private TailSurfaceClassifier() {}

    /**
     * Uses IV's authored control-animation rotation centres as independent geometric
     * evidence before a generic far-aft triangle cluster is allowed to become an
     * aerodynamic stabilizer.  Render meshes often contain horizontal fairings,
     * tail-boom caps, doors, and other aft panels that satisfy a purely positional
     * classifier.  If a real animated elevator/rudder is present and the generic
     * fixed-surface cluster is spatially incompatible with the moving control mesh
     * and with any independently trustworthy hinge evidence, the suspect triangles
     * return to BODY pressure instead of receiving lifting area.
     *
     * <p>The tolerance scales only with the frozen model length.  No model name,
     * pack name, aircraft mass, or tuned coordinate appears here.</p>
     */
    static void repairAllMovingHorizontalTailClassifications(
        EnumMap<SurfaceKind, List<ClassifiedTriangle>> byKind,
        Bounds bounds
    ) {
        List<ClassifiedTriangle> moving = byKind.get(SurfaceKind.TAILERON);
        List<ClassifiedTriangle> fixed = byKind.get(SurfaceKind.HORIZONTAL_TAIL);
        if (moving == null || moving.isEmpty() || fixed == null || fixed.isEmpty()
            || bounds == null || !bounds.valid()) {
            return;
        }

        Bounds movingBounds = triangleBounds(moving);
        Vec3d movingCenter = areaWeightedTriangleCentroid(moving);
        if (movingBounds == null || !movingBounds.valid() || movingCenter == null
            || !movingCenter.isFinite()) {
            return;
        }

        // TAILERON is only produced when an entire aft object is authored as a
        // mixed elevator+aileron moving surface (or is explicitly named as one).
        // Once that evidence exists, a second horizontal tail inferred solely
        // from anonymous far-aft render triangles is not independent evidence of
        // a fixed stabilizer.  Blended fighter fuselage/wing-root panels commonly
        // satisfy the generic positional HORIZONTAL_TAIL classifier and otherwise
        // duplicate the real all-moving stabilator area.
        //
        // Preserve any explicitly named fixed stabilizer/tailplane.  This keeps
        // the rule generic and evidence-based: authored fixed-tail semantics win,
        // while only geometry-guessed fixed-tail triangles are returned to BODY.
        Vec3d normalizedMovingCenter = bounds.normalized(movingCenter);
        if (normalizedMovingCenter.z() > -0.35) {
            return;
        }

        List<ClassifiedTriangle> body = byKind.get(SurfaceKind.BODY);
        List<ClassifiedTriangle> retainedFixed = new ArrayList<>(fixed.size());
        int movedTriangles = 0;
        double movedArea = 0.0;
        for (ClassifiedTriangle triangle : fixed) {
            SurfaceKind explicitKind = SurfaceKind.fromSpecificName(
                normalize(triangle.objectName())
            );
            if (explicitKind == SurfaceKind.HORIZONTAL_TAIL) {
                retainedFixed.add(triangle);
                continue;
            }
            body.add(new ClassifiedTriangle(
                triangle.first(), triangle.second(), triangle.third(),
                triangle.centroid(), triangle.normal(), triangle.area(),
                triangle.objectName(), SurfaceKind.BODY
            ));
            ++movedTriangles;
            movedArea += Math.max(0.0, triangle.area());
        }
        if (movedTriangles == 0) {
            return;
        }
        fixed.clear();
        fixed.addAll(retainedFixed);
        if (PMIVObserver.loggingEnabled()) {
            PMIVObserver.log(String.format(
            Locale.ROOT,
            "MODEL_ALL_MOVING_TAIL_REPAIR action=DEMOTE_GEOMETRY_GUESSED_FIXED_TAIL_TO_BODY movedTriangles=%d movedRawArea=%.5f retainedExplicitFixedTriangles=%d taileronTriangles=%d taileronCenter=(%.4f,%.4f,%.4f) taileronBounds=%s policy=AUTHORED_ALL_MOVING_TAIL_OVERRIDES_POSITION_ONLY_FIXED_TAIL",
            movedTriangles, movedArea, retainedFixed.size(), moving.size(),
            movingCenter.x(), movingCenter.y(), movingCenter.z(), movingBounds.compact()
        ));
        }
    }

    static void repairIncoherentTailClassifications(
        EntityVehicleF_Physics vehicle,
        EnumMap<SurfaceKind, List<ClassifiedTriangle>> byKind,
        Bounds bounds
    ) {
        repairIncoherentParentClassification(
            vehicle, byKind, bounds,
            SurfaceKind.HORIZONTAL_TAIL, SurfaceKind.ELEVATOR, "elevator", "HORIZONTAL_TAIL"
        );
        repairIncoherentParentClassification(
            vehicle, byKind, bounds,
            SurfaceKind.VERTICAL_TAIL, SurfaceKind.RUDDER, "rudder", "VERTICAL_TAIL"
        );
    }

    static void repairIncoherentParentClassification(
        EntityVehicleF_Physics vehicle,
        EnumMap<SurfaceKind, List<ClassifiedTriangle>> byKind,
        Bounds bounds,
        SurfaceKind parentKind,
        SurfaceKind controlKind,
        String variableToken,
        String diagnosticName
    ) {
        List<ClassifiedTriangle> parent = byKind.get(parentKind);
        List<ClassifiedTriangle> control = byKind.get(controlKind);
        if (parent == null || parent.isEmpty() || control == null || control.isEmpty()) {
            return;
        }
        Vec3d controlCenter = areaWeightedTriangleCentroid(control);
        Bounds controlBounds = triangleBounds(control);
        Vec3d authoredHinge = meanControlRotationCenter(vehicle, variableToken, control);
        boolean hingeTrusted = authoredHinge != null && authoredHinge.isFinite()
            && hingeCoherentWithControlMesh(authoredHinge, controlBounds, bounds);
        if (authoredHinge != null && authoredHinge.isFinite() && !hingeTrusted
            && controlCenter != null && controlCenter.isFinite()) {
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(String.format(
                Locale.ROOT,
                "MODEL_HINGE_EVIDENCE_REJECTED control=%s reason=HINGE_INCOHERENT_WITH_CONTROL_MESH hinge=(%.4f,%.4f,%.4f) controlCenter=(%.4f,%.4f,%.4f) controlBounds=%s",
                controlKind,
                authoredHinge.x(), authoredHinge.y(), authoredHinge.z(),
                controlCenter.x(), controlCenter.y(), controlCenter.z(),
                controlBounds != null && controlBounds.valid() ? controlBounds.compact() : "INVALID"
            ));
            }
        }
        Vec3d parentCenter = areaWeightedTriangleCentroid(parent);
        if (controlCenter == null || parentCenter == null
            || !controlCenter.isFinite() || !parentCenter.isFinite()) {
            return;
        }

        // A hinge is a pivot, not the aerodynamic centre of the complete parent
        // stabilizer.  Therefore it may corroborate a bad parent classification,
        // but it must never be the sole datum that deletes an otherwise coherent
        // fixed fin/tailplane.  Demote only when the generic parent is spatially
        // incompatible with the actual moving control mesh AND, when trustworthy
        // hinge evidence exists, incompatible with that hinge as well.
        double parentToControlDistance = Math.hypot(
            parentCenter.y() - controlCenter.y(), parentCenter.z() - controlCenter.z()
        );
        double controlTolerance = Math.max(0.80, bounds.spanZ() * 0.16);
        boolean controlIncoherent = Double.isFinite(parentToControlDistance)
            && parentToControlDistance > controlTolerance;

        double parentToHingeDistance = Double.NaN;
        double hingeTolerance = Math.max(0.60, bounds.spanZ() * 0.12);
        boolean hingeIncoherent = true;
        if (hingeTrusted) {
            parentToHingeDistance = Math.hypot(
                parentCenter.y() - authoredHinge.y(), parentCenter.z() - authoredHinge.z()
            );
            hingeIncoherent = Double.isFinite(parentToHingeDistance)
                && parentToHingeDistance > hingeTolerance;
        }
        if (!controlIncoherent || !hingeIncoherent) {
            return;
        }

        List<ClassifiedTriangle> body = byKind.get(SurfaceKind.BODY);
        int moved = parent.size();
        for (ClassifiedTriangle triangle : parent) {
            body.add(new ClassifiedTriangle(
                triangle.first(), triangle.second(), triangle.third(),
                triangle.centroid(), triangle.normal(), triangle.area(),
                triangle.objectName(), SurfaceKind.BODY
            ));
        }
        parent.clear();
        if (PMIVObserver.loggingEnabled()) {
            PMIVObserver.log(String.format(
            Locale.ROOT,
            "MODEL_CLASSIFICATION_REPAIR parent=%s action=DEMOTE_TO_BODY reason=CONTROL_AND_HINGE_INCOHERENT movedTriangles=%d parentCenter=(%.4f,%.4f,%.4f) controlCenter=(%.4f,%.4f,%.4f) parentToControlDistance=%.4f controlTolerance=%.4f parentToHingeDistance=%.4f hingeTolerance=%.4f authoredHinge=%s hingeTrusted=%s",
            diagnosticName, moved,
            parentCenter.x(), parentCenter.y(), parentCenter.z(),
            controlCenter.x(), controlCenter.y(), controlCenter.z(),
            parentToControlDistance, controlTolerance,
            parentToHingeDistance, hingeTolerance,
            authoredHinge != null, hingeTrusted
        ));
        }
    }

    /**
     * A pack-authored rotation centre is useful only when it is also plausible
     * for the moving mesh that supposedly owns it.  Some packs reuse control
     * variables for steering/linkage animations or carry legacy centres that
     * are nowhere near the rendered rudder/elevator.  Trusting such a centre
     * can incorrectly delete an otherwise coherent fixed stabilizer.  The
     * check is deliberately geometric and model-independent: the hinge must lie
     * inside the control's Y/Z envelope with a small envelope/chord-scaled
     * allowance for a leading-edge hinge just outside the rendered skin.
     */
    static boolean hingeCoherentWithControlMesh(
        Vec3d hinge,
        Bounds controlBounds,
        Bounds modelBounds
    ) {
        if (hinge == null || !hinge.isFinite() || controlBounds == null
            || !controlBounds.valid() || modelBounds == null || !modelBounds.valid()) {
            return false;
        }
        double localScale = Math.max(controlBounds.spanY(), controlBounds.spanZ());
        double margin = Math.max(
            0.12,
            Math.max(localScale * 0.18, modelBounds.spanZ() * 0.025)
        );
        return hinge.y() >= controlBounds.minimum().y() - margin
            && hinge.y() <= controlBounds.maximum().y() + margin
            && hinge.z() >= controlBounds.minimum().z() - margin
            && hinge.z() <= controlBounds.maximum().z() + margin;
    }

    static Bounds triangleBounds(List<ClassifiedTriangle> triangles) {
        if (triangles == null || triangles.isEmpty()) {
            return new Bounds(Vec3d.ZERO, Vec3d.ZERO, false);
        }
        BoundsAccumulator accumulator = new BoundsAccumulator();
        for (ClassifiedTriangle triangle : triangles) {
            if (triangle == null) {
                continue;
            }
            accumulator.include(triangle.first());
            accumulator.include(triangle.second());
            accumulator.include(triangle.third());
        }
        return accumulator.finishLoose();
    }

    static Vec3d meanControlRotationCenter(
        EntityVehicleF_Physics vehicle,
        String variableToken,
        List<ClassifiedTriangle> controlTriangles
    ) {
        if (vehicle == null || vehicle.definition == null || vehicle.definition.rendering == null
            || vehicle.definition.rendering.animatedObjects == null || variableToken == null
            || controlTriangles == null || controlTriangles.isEmpty()) {
            return null;
        }
        Map<String, Boolean> controlObjects = new HashMap<>();
        for (ClassifiedTriangle triangle : controlTriangles) {
            if (triangle.objectName() != null) {
                controlObjects.put(normalize(triangle.objectName()), Boolean.TRUE);
            }
        }
        Vec3d sum = Vec3d.ZERO;
        int count = 0;
        for (JSONAnimatedObject object : vehicle.definition.rendering.animatedObjects) {
            if (object == null || object.objectName == null || object.animations == null
                || !controlObjects.containsKey(normalize(object.objectName))) {
                continue;
            }
            for (JSONAnimationDefinition animation : object.animations) {
                if (animation == null || animation.variable == null || animation.centerPoint == null
                    || animation.animationType != JSONAnimationDefinition.AnimationComponentType.ROTATION
                    || !normalize(animation.variable).contains(variableToken)) {
                    continue;
                }
                Vec3d center = ModelCoordinates.scaledPoint(vehicle,new Vec3d(
                    animation.centerPoint.x, animation.centerPoint.y, animation.centerPoint.z
                ));
                if (center.isFinite()) {
                    sum = sum.add(center);
                    ++count;
                }
            }
        }
        return count > 0 ? sum.scale(1.0 / count) : null;
    }

    static Vec3d areaWeightedTriangleCentroid(List<ClassifiedTriangle> triangles) {
        if (triangles == null || triangles.isEmpty()) {
            return null;
        }
        Vec3d weighted = Vec3d.ZERO;
        double total = 0.0;
        for (ClassifiedTriangle triangle : triangles) {
            double weight = Math.max(EPSILON, triangle.area());
            weighted = weighted.add(triangle.centroid().scale(weight));
            total += weight;
        }
        return total > EPSILON ? weighted.scale(1.0 / total) : null;
    }

    static SurfaceKind classify(Bounds bounds, boolean veryFarAft, boolean farAft, boolean verticalSurface, boolean horizontalSurface, boolean high) {
        boolean conventionalLength = bounds.spanZ() > bounds.spanX() * 0.55;
        if (conventionalLength && veryFarAft && verticalSurface && high) {
            return SurfaceKind.VERTICAL_TAIL;
        }
        if (conventionalLength && farAft && horizontalSurface) {
            return SurfaceKind.HORIZONTAL_TAIL;
        }
        return null;
    }
}
