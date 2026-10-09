package com.g9third.pmweatheriv.physics;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.List;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;

import static com.g9third.pmweatheriv.physics.ModelGeometryData.Axis;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.Bounds;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.ClassifiedTriangle;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.EPSILON;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.LiftingPatch;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.LiftingSummary;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.SectionBand;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.SurfaceKind;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.SymmetryRole;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.TrianglePair;
import static com.g9third.pmweatheriv.physics.BodyPressureGeometry.coordinate;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceFrames.mirrorTolerance;
import static com.g9third.pmweatheriv.physics.BodyPressureGeometry.nearestCenterlineLiftingPoint;
import static com.g9third.pmweatheriv.physics.BodyPressureGeometry.nearestLiftingPoint;
import static com.g9third.pmweatheriv.physics.BodyPressureGeometry.nearestMirroredLiftingPair;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceFrames.symmetrySortOrder;
import static com.g9third.pmweatheriv.physics.BodyPressureGeometry.weightedAverage;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceFrames.weightedZ;

/** Builds adaptive wing and tail lifting patches from classified triangles. */
final class LiftingPatchGeometry {
    private LiftingPatchGeometry() {}

    static List<LiftingPatch> buildLiftingPatches(
        EntityVehicleF_Physics vehicle,
        EnumMap<SurfaceKind, List<ClassifiedTriangle>> byKind,
        Bounds bounds
    ) {
        List<LiftingPatch> output = new ArrayList<>();
        int pairId = 0;

        // The fixed main wing uses a geometry-scaled number of spanwise sections
        // per half-wing. Section width follows physical half-span and mean chord,
        // so small conventional aircraft generally receive two or three sections
        // while long glider/airliner wings can receive up to six. Every section
        // preserves its own local PMWeather query, represented area, separation
        // state, and torque lever arm. Controls remain separate components.
        boolean hasFixedWingGeometry = byKind.get(SurfaceKind.WING) != null
            && !byKind.get(SurfaceKind.WING).isEmpty();
        List<ClassifiedTriangle> mainLiftingGeometry = hasFixedWingGeometry
            ? byKind.get(SurfaceKind.WING)
            : byKind.get(SurfaceKind.ELEVON);
        int mainWingSections = adaptiveMainWingSectionCount(
            vehicle, mainLiftingGeometry, bounds
        );
        pairId = addAdaptivePairedLiftingComponent(output, byKind.get(SurfaceKind.WING),
            SurfaceKind.WING, bounds, Axis.VERTICAL, pairId, mainWingSections);
        pairId = addPairedLiftingComponent(output, byKind.get(SurfaceKind.AILERON),
            SurfaceKind.AILERON, bounds, Axis.VERTICAL, pairId);
        pairId = hasFixedWingGeometry
            ? addPairedLiftingComponent(output, byKind.get(SurfaceKind.ELEVON),
                SurfaceKind.ELEVON, bounds, Axis.VERTICAL, pairId)
            : addAdaptivePairedLiftingComponent(output, byKind.get(SurfaceKind.ELEVON),
                SurfaceKind.ELEVON, bounds, Axis.VERTICAL, pairId, mainWingSections);
        pairId = addPairedLiftingComponent(output, byKind.get(SurfaceKind.CANARD),
            SurfaceKind.CANARD, bounds, Axis.VERTICAL, pairId);
        pairId = addPairedLiftingComponent(output, byKind.get(SurfaceKind.CANARDERON),
            SurfaceKind.CANARDERON, bounds, Axis.VERTICAL, pairId);

        boolean taillessElevonLayout = !byKind.get(SurfaceKind.ELEVON).isEmpty()
            && byKind.get(SurfaceKind.ELEVATOR).isEmpty();
        if (!taillessElevonLayout) {
            pairId = addPairedLiftingComponent(output, byKind.get(SurfaceKind.HORIZONTAL_TAIL),
                SurfaceKind.HORIZONTAL_TAIL, bounds, Axis.VERTICAL, pairId);
        }
        pairId = addPairedLiftingComponent(output, byKind.get(SurfaceKind.ELEVATOR),
            SurfaceKind.ELEVATOR, bounds, Axis.VERTICAL, pairId);
        pairId = addPairedLiftingComponent(output, byKind.get(SurfaceKind.TAILERON),
            SurfaceKind.TAILERON, bounds, Axis.VERTICAL, pairId);
        pairId = addVerticalLiftingComponent(output, byKind.get(SurfaceKind.VERTICAL_TAIL),
            SurfaceKind.VERTICAL_TAIL, bounds, pairId);
        pairId = addVerticalLiftingComponent(output, byKind.get(SurfaceKind.RUDDER),
            SurfaceKind.RUDDER, bounds, pairId);

        double configuredSpan = Math.max(0.0, vehicle.wingSpanVar.currentValue);
        double wingSpan = Math.max(bounds.spanX(), configuredSpan);
        double wingX = Math.max(0.5, wingSpan * 0.35);
        double wingY = bounds.minimum().y() + bounds.spanY() * 0.56;
        double wingZ = weightedZ(output, SurfaceKind.WING, SurfaceKind.AILERON, SurfaceKind.ELEVON);
        if (!Double.isFinite(wingZ)) {
            wingZ = bounds.minimum().z() + bounds.spanZ() * 0.52;
        }

        boolean hasMainWing = output.stream().anyMatch(patch ->
            patch.kind() == SurfaceKind.WING
                || patch.kind() == SurfaceKind.AILERON
                || patch.kind() == SurfaceKind.ELEVON
                || patch.kind() == SurfaceKind.CANARD
                || patch.kind() == SurfaceKind.CANARDERON
        );
        if (!hasMainWing && (configuredSpan > 0.1 || vehicle.wingAreaVar.currentValue > 0.01)) {
            output.add(new LiftingPatch("FALLBACK_WING_L", SurfaceKind.WING,
                new Vec3d(-wingX, wingY, wingZ), 1.0, pairId, SymmetryRole.MIRRORED_LEFT));
            output.add(new LiftingPatch("FALLBACK_WING_R", SurfaceKind.WING,
                new Vec3d(wingX, wingY, wingZ), 1.0, pairId, SymmetryRole.MIRRORED_RIGHT));
            ++pairId;
        }

        double tailDistance = Math.max(0.0, Math.abs(vehicle.definition.motorized.tailDistance));
        double tailZ = bounds.minimum().z() + bounds.spanZ() * 0.08;
        if (tailDistance > 0.1) {
            tailZ = Math.min(tailZ, wingZ - tailDistance * 0.75);
        }
        tailZ = Vec3d.clamp(
            tailZ,
            bounds.minimum().z() + bounds.spanZ() * 0.02,
            bounds.maximum().z() - bounds.spanZ() * 0.02
        );
        double tailX = Math.max(0.25, Math.min(wingSpan * 0.16, bounds.spanX() * 0.28));
        double tailY = bounds.minimum().y() + bounds.spanY() * 0.60;

        boolean hasHorizontalTail = output.stream().anyMatch(patch ->
            patch.kind() == SurfaceKind.HORIZONTAL_TAIL
                || patch.kind() == SurfaceKind.ELEVATOR
                || patch.kind() == SurfaceKind.TAILERON
        );
        boolean hasElevon = output.stream().anyMatch(patch -> patch.kind() == SurfaceKind.ELEVON);
        if (!hasHorizontalTail && !hasElevon
            && (configuredSpan > 0.1 || vehicle.elevatorAreaVar.currentValue > 0.01)) {
            output.add(new LiftingPatch("FALLBACK_HTAIL_L", SurfaceKind.HORIZONTAL_TAIL,
                new Vec3d(-tailX, tailY, tailZ), 1.0, pairId, SymmetryRole.MIRRORED_LEFT));
            output.add(new LiftingPatch("FALLBACK_HTAIL_R", SurfaceKind.HORIZONTAL_TAIL,
                new Vec3d(tailX, tailY, tailZ), 1.0, pairId, SymmetryRole.MIRRORED_RIGHT));
            ++pairId;
            output.add(new LiftingPatch("FALLBACK_ELEVATOR_L", SurfaceKind.ELEVATOR,
                new Vec3d(-tailX, tailY, tailZ - bounds.spanZ() * 0.015), 1.0, pairId,
                SymmetryRole.MIRRORED_LEFT));
            output.add(new LiftingPatch("FALLBACK_ELEVATOR_R", SurfaceKind.ELEVATOR,
                new Vec3d(tailX, tailY, tailZ - bounds.spanZ() * 0.015), 1.0, pairId,
                SymmetryRole.MIRRORED_RIGHT));
            ++pairId;
        }

        boolean hasVerticalTail = output.stream().anyMatch(patch ->
            patch.kind() == SurfaceKind.VERTICAL_TAIL
        );
        boolean hasRudder = output.stream().anyMatch(patch ->
            patch.kind() == SurfaceKind.RUDDER
        );
        if (vehicle.rudderAreaVar.currentValue > 0.01) {
            double verticalY = bounds.maximum().y() - bounds.spanY() * 0.14;
            double centerX = 0.5 * (bounds.minimum().x() + bounds.maximum().x());
            if (!hasVerticalTail) {
                output.add(new LiftingPatch("FALLBACK_VTAIL", SurfaceKind.VERTICAL_TAIL,
                    new Vec3d(centerX, verticalY, tailZ), 1.0, -1, SymmetryRole.CENTERLINE));
            }
            if (!hasRudder) {
                output.add(new LiftingPatch("FALLBACK_RUDDER", SurfaceKind.RUDDER,
                    new Vec3d(centerX, verticalY, tailZ - bounds.spanZ() * 0.015), 1.0, -1,
                    SymmetryRole.CENTERLINE));
            }
        }

        output.sort(Comparator
            .comparingInt((LiftingPatch patch) -> patch.kind().ordinal())
            .thenComparingInt(patch -> symmetrySortOrder(patch.symmetryRole()))
            .thenComparingDouble(patch -> patch.pointLocal().x()));
        return List.copyOf(output);
    }

    static int adaptiveMainWingSectionCount(
        EntityVehicleF_Physics vehicle,
        List<ClassifiedTriangle> source,
        Bounds bounds
    ) {
        double configuredSpan = Math.max(0.0, vehicle.wingSpanVar.currentValue);
        double geometrySpan = liftingGeometrySpanX(source);
        double effectiveSpan = Math.max(configuredSpan, geometrySpan);
        if (effectiveSpan <= 0.5) {
            effectiveSpan = bounds.spanX();
        }
        if (effectiveSpan <= 1.0) {
            return 1;
        }

        double configuredArea = Math.max(0.0, vehicle.wingAreaVar.currentValue);
        double projectedGeometryArea = projectedLiftingArea(source, Axis.VERTICAL) * 0.5;
        double effectiveArea = configuredArea > 0.01
            ? configuredArea
            : projectedGeometryArea;
        double meanChord = effectiveArea > 0.01
            ? effectiveArea / Math.max(0.5, effectiveSpan)
            : Math.max(0.4, bounds.spanZ() * 0.12);

        // Wider-chord wings need fewer spatial samples for the same span, while
        // slender glider wings need more. This is a geometry rule, not an
        // aircraft/content-pack lookup. At least two sections are requested for
        // credible fixed-wing geometry; sparse meshes may reduce the final count.
        double targetSectionWidth = Vec3d.clamp(
            1.5 + 0.35 * meanChord,
            1.5,
            4.0
        );
        int requested = (int) Math.ceil((0.5 * effectiveSpan) / targetSectionWidth);
        return Math.max(2, Math.min(6, requested));
    }

    static double liftingGeometrySpanX(List<ClassifiedTriangle> source) {
        if (source == null || source.isEmpty()) {
            return 0.0;
        }
        double minimum = Double.POSITIVE_INFINITY;
        double maximum = Double.NEGATIVE_INFINITY;
        for (ClassifiedTriangle triangle : source) {
            if (Math.abs(triangle.normal().y()) < 0.08) {
                continue;
            }
            minimum = Math.min(minimum, triangle.centroid().x());
            maximum = Math.max(maximum, triangle.centroid().x());
        }
        return maximum > minimum ? maximum - minimum : 0.0;
    }

    static double projectedLiftingArea(
        List<ClassifiedTriangle> source,
        Axis projectedAxis
    ) {
        if (source == null || source.isEmpty()) {
            return 0.0;
        }
        double area = 0.0;
        for (ClassifiedTriangle triangle : source) {
            double projection = projectedAxis == Axis.VERTICAL
                ? Math.abs(triangle.normal().y())
                : Math.abs(triangle.normal().x());
            area += triangle.area() * Math.max(0.0, projection);
        }
        return area;
    }

    /**
     * Divides one left/right lifting component into one to six jointly mirrored
     * spanwise bands. Boundaries use equal physical span widths over the actual
     * retained component triangles, not fixed aircraft templates. If either side lacks
     * credible geometry in a requested band, the count is reduced until every
     * emitted section has a complete counterpart.
     */
    static int addAdaptivePairedLiftingComponent(
        List<LiftingPatch> output,
        List<ClassifiedTriangle> source,
        SurfaceKind kind,
        Bounds bounds,
        Axis projectedAxis,
        int pairId,
        int requestedSections
    ) {
        if (source == null || source.isEmpty() || requestedSections <= 1) {
            return addPairedLiftingComponent(
                output, source, kind, bounds, projectedAxis, pairId
            );
        }
        double centerX = 0.5 * (bounds.minimum().x() + bounds.maximum().x());
        double tolerance = mirrorTolerance(bounds);
        List<SectionBand> bands = adaptiveSectionBands(
            source, projectedAxis, centerX, tolerance, requestedSections, bounds
        );
        if (bands.size() <= 1) {
            return addPairedLiftingComponent(
                output, source, kind, bounds, projectedAxis, pairId
            );
        }

        int initialSize = output.size();
        int nextPairId = pairId;
        for (int index = 0; index < bands.size(); ++index) {
            SectionBand band = bands.get(index);
            List<ClassifiedTriangle> sectionSource = liftingSectionTriangles(
                source, centerX, tolerance, band, index == 0
            );
            int before = output.size();
            int afterPairId = addPairedLiftingComponent(
                output, sectionSource, kind, bounds, projectedAxis, nextPairId
            );
            if (afterPairId != nextPairId + 1 || output.size() != before + 2) {
                // The adaptive representation is only valid when every section is
                // a complete pair. Roll back and use the single-component result.
                while (output.size() > initialSize) {
                    output.remove(output.size() - 1);
                }
                return addPairedLiftingComponent(
                    output, source, kind, bounds, projectedAxis, pairId
                );
            }
            String sectionName = liftingSectionName(index, bands.size());
            for (int patchIndex = before; patchIndex < output.size(); ++patchIndex) {
                LiftingPatch patch = output.get(patchIndex);
                String side = patch.symmetryRole() == SymmetryRole.MIRRORED_LEFT ? "_L" : "_R";
                output.set(patchIndex, new LiftingPatch(
                    kind.name() + "_" + sectionName + side,
                    patch.kind(), patch.pointLocal(), patch.weight(), patch.symmetryPair(),
                    patch.symmetryRole(), index, bands.size(),
                    band.minimumRadius(), band.maximumRadius()
                ));
            }
            nextPairId = afterPairId;
        }
        return nextPairId;
    }

    static List<SectionBand> adaptiveSectionBands(
        List<ClassifiedTriangle> source,
        Axis projectedAxis,
        double centerX,
        double tolerance,
        int requestedSections,
        Bounds bounds
    ) {
        int maximum = Math.max(1, Math.min(6, requestedSections));
        for (int sections = maximum; sections >= 2; --sections) {
            List<Double> sampledRadii = new ArrayList<>();
            double maximumRadius = 0.0;
            for (ClassifiedTriangle triangle : source) {
                double radius = Math.abs(triangle.centroid().x() - centerX);
                if (radius <= tolerance) {
                    continue;
                }
                double projection = projectedAxis == Axis.VERTICAL
                    ? Math.abs(triangle.normal().y())
                    : Math.abs(triangle.normal().x());
                double weight = triangle.area() * Math.max(0.03, projection);
                if (weight <= EPSILON) {
                    continue;
                }
                sampledRadii.add(radius);
                maximumRadius = Math.max(maximumRadius, radius);
            }
            if (sampledRadii.size() < sections * 4
                || maximumRadius <= Math.max(0.20, bounds.spanX() * 0.12)) {
                continue;
            }
            // Use equal spanwise widths rather than equal-area quantiles. Wind
            // gradients are spatial, so root/mid/tip samples should cover similar
            // distances along the wing even when a tapered outer panel has less
            // area. Each section still carries its actual projected triangle
            // weight, so the outer section naturally contributes less force.
            List<Double> boundaries = new ArrayList<>();
            boundaries.add(0.0);
            for (int split = 1; split < sections; ++split) {
                boundaries.add(maximumRadius * split / sections);
            }
            boundaries.add(maximumRadius + Math.max(EPSILON, maximumRadius * 1.0E-9));

            double minimumBandWidth = Math.max(0.08, bounds.spanX() * 0.015);
            boolean separated = true;
            for (int i = 1; i < boundaries.size(); ++i) {
                if (boundaries.get(i) - boundaries.get(i - 1) < minimumBandWidth) {
                    separated = false;
                    break;
                }
            }
            if (!separated) {
                continue;
            }

            List<SectionBand> bands = new ArrayList<>();
            boolean complete = true;
            for (int index = 0; index < sections; ++index) {
                SectionBand band = new SectionBand(boundaries.get(index), boundaries.get(index + 1));
                List<ClassifiedTriangle> sectionSource = liftingSectionTriangles(
                    source, centerX, tolerance, band, index == 0
                );
                LiftingSummary left = summarizeLifting(
                    sectionSource, projectedAxis, centerX, -1, tolerance
                );
                LiftingSummary right = summarizeLifting(
                    sectionSource, projectedAxis, centerX, 1, tolerance
                );
                if (left.weight() <= EPSILON || right.weight() <= EPSILON) {
                    complete = false;
                    break;
                }
                bands.add(band);
            }
            if (complete) {
                return List.copyOf(bands);
            }
        }
        return List.of(new SectionBand(0.0, Double.MAX_VALUE));
    }

    static List<ClassifiedTriangle> liftingSectionTriangles(
        List<ClassifiedTriangle> source,
        double centerX,
        double tolerance,
        SectionBand band,
        boolean includeCenter
    ) {
        List<ClassifiedTriangle> output = new ArrayList<>();
        for (ClassifiedTriangle triangle : source) {
            double radius = Math.abs(triangle.centroid().x() - centerX);
            if (radius <= tolerance) {
                if (includeCenter) {
                    output.add(triangle);
                }
                continue;
            }
            if (radius + EPSILON >= band.minimumRadius()
                && radius < band.maximumRadius()) {
                output.add(triangle);
            }
        }
        return output;
    }

    static String liftingSectionName(int index, int count) {
        if (count <= 1) {
            return "WHOLE";
        }
        if (index <= 0) {
            return "INBOARD";
        }
        if (index >= count - 1) {
            return "OUTBOARD";
        }
        if ((count & 1) == 1 && index == count / 2) {
            return "MID";
        }
        int distanceFromRoot = index;
        int distanceFromTip = count - 1 - index;
        return distanceFromRoot <= distanceFromTip
            ? "INNER_" + distanceFromRoot
            : "OUTER_" + distanceFromTip;
    }

    static int addPairedLiftingComponent(
        List<LiftingPatch> output,
        List<ClassifiedTriangle> source,
        SurfaceKind kind,
        Bounds bounds,
        Axis projectedAxis,
        int pairId
    ) {
        if (source == null || source.isEmpty()) {
            return pairId;
        }
        double centerX = 0.5 * (bounds.minimum().x() + bounds.maximum().x());
        double tolerance = mirrorTolerance(bounds);
        LiftingSummary left = summarizeLifting(source, projectedAxis, centerX, -1, tolerance);
        LiftingSummary right = summarizeLifting(source, projectedAxis, centerX, 1, tolerance);
        LiftingSummary center = summarizeLifting(source, projectedAxis, centerX, 0, tolerance);
        double sideTotal = left.weight() + right.weight();
        double total = sideTotal + center.weight();
        if (total <= EPSILON) {
            return pairId;
        }

        boolean onlyLeft = left.weight() > EPSILON && right.weight() <= EPSILON
            && center.weight() <= left.weight() * 0.08;
        boolean onlyRight = right.weight() > EPSILON && left.weight() <= EPSILON
            && center.weight() <= right.weight() * 0.08;
        if (onlyLeft || onlyRight) {
            LiftingSummary dominant = onlyLeft ? left : right;
            int side = onlyLeft ? -1 : 1;
            if (credibleOneSidedLifting(dominant, bounds, centerX)) {
                Vec3d point = nearestLiftingPoint(
                    source, projectedAxis, centerX, side, tolerance, dominant.centroid()
                );
                output.add(new LiftingPatch(
                    kind.name() + (onlyLeft ? "_LEFT_ONLY_CONFIRMED" : "_RIGHT_ONLY_CONFIRMED"),
                    kind, point, dominant.weight(), -1,
                    onlyLeft ? SymmetryRole.LEFT_ONLY : SymmetryRole.RIGHT_ONLY
                ));
            } else {
                LiftingSummary summary = summarizeLifting(source, projectedAxis, centerX, 2, 0.0);
                Vec3d anchored = nearestCenterlineLiftingPoint(
                    source, projectedAxis, centerX, tolerance, summary.centroid(), bounds
                );
                if (anchored != null) {
                    output.add(new LiftingPatch(kind.name() + "_CENTER", kind,
                        anchored, summary.weight(), -1, SymmetryRole.CENTERLINE));
                }
            }
            return pairId;
        }

        // Never invent a mirrored lifting point when the mesh has no triangles on
        // that side.  A genuinely center-only surface remains a center patch.
        if (left.weight() <= EPSILON || right.weight() <= EPSILON) {
            LiftingSummary summary = summarizeLifting(source, projectedAxis, centerX, 2, 0.0);
            Vec3d point = nearestCenterlineLiftingPoint(
                source, projectedAxis, centerX, tolerance,
                new Vec3d(centerX, summary.centroid().y(), summary.centroid().z()), bounds
            );
            if (point != null) {
                output.add(new LiftingPatch(kind.name() + "_CENTER", kind,
                    point, summary.weight(), -1, SymmetryRole.CENTERLINE));
            }
            return pairId;
        }

        double radiusNumerator =
            Math.abs(left.centroid().x() - centerX) * left.weight()
                + Math.abs(right.centroid().x() - centerX) * right.weight();
        double radius = radiusNumerator / Math.max(EPSILON, sideTotal);
        double y = weightedCoordinate(left, right, center, 1,
            bounds.minimum().y() + bounds.spanY() * 0.55);
        double z = weightedCoordinate(left, right, center, 2,
            bounds.minimum().z() + bounds.spanZ() * 0.50);
        double halfWeight = total * 0.5;
        Vec3d leftTarget = new Vec3d(centerX - radius, y, z);
        Vec3d rightTarget = new Vec3d(centerX + radius, y, z);
        TrianglePair pair = nearestMirroredLiftingPair(
            source, projectedAxis, centerX, tolerance,
            leftTarget, rightTarget, bounds
        );
        Vec3d leftPoint = pair.left();
        Vec3d rightPoint = pair.right();
        output.add(new LiftingPatch(kind.name() + "_L", kind,
            leftPoint, halfWeight, pairId, SymmetryRole.MIRRORED_LEFT));
        output.add(new LiftingPatch(kind.name() + "_R", kind,
            rightPoint, halfWeight, pairId, SymmetryRole.MIRRORED_RIGHT));
        return pairId + 1;
    }

    /**
     * Handles one center fin, a mirrored twin-fin pair, or a triple-tail
     * arrangement using only generic X-distribution and projected area. The
     * center band scales with aircraft span so a thick single airliner fin is
     * not mistaken for two fins, while widely separated fins remain paired.
     */
    static int addVerticalLiftingComponent(
        List<LiftingPatch> output,
        List<ClassifiedTriangle> source,
        SurfaceKind kind,
        Bounds bounds,
        int pairId
    ) {
        if (source == null || source.isEmpty() || !bounds.valid()) {
            return pairId;
        }
        double centerX = 0.5 * (bounds.minimum().x() + bounds.maximum().x());
        double centerBand = Math.max(0.12, bounds.spanX() * 0.04);
        LiftingSummary left = summarizeLifting(source, Axis.LATERAL, centerX, -1, centerBand);
        LiftingSummary right = summarizeLifting(source, Axis.LATERAL, centerX, 1, centerBand);
        LiftingSummary center = summarizeLifting(source, Axis.LATERAL, centerX, 0, centerBand);
        double total = left.weight() + right.weight() + center.weight();
        if (total <= EPSILON) {
            return pairId;
        }

        double minimumBranchWeight = Math.max(0.01, total * 0.10);
        boolean hasLeft = left.weight() >= minimumBranchWeight;
        boolean hasRight = right.weight() >= minimumBranchWeight;
        boolean hasCenter = center.weight() >= minimumBranchWeight;

        if (hasLeft && hasRight) {
            double ratio = Math.max(left.weight(), right.weight())
                / Math.max(EPSILON, Math.min(left.weight(), right.weight()));
            double yError = Math.abs(left.centroid().y() - right.centroid().y())
                / Math.max(0.25, bounds.spanY());
            double zError = Math.abs(left.centroid().z() - right.centroid().z())
                / Math.max(0.25, bounds.spanZ());
            double minimumVerticalExtent = Math.max(0.25, bounds.spanY() * 0.08);
            double leftVerticalExtent = liftingBranchSpanY(source, centerX, -1, centerBand);
            double rightVerticalExtent = liftingBranchSpanY(source, centerX, 1, centerBand);
            boolean credibleFinPair = leftVerticalExtent >= minimumVerticalExtent
                && rightVerticalExtent >= minimumVerticalExtent;
            if (credibleFinPair && ratio <= 4.0 && yError <= 0.35 && zError <= 0.35) {
                double sideWeight = left.weight() + right.weight();
                double radius = (
                    Math.abs(left.centroid().x() - centerX) * left.weight()
                        + Math.abs(right.centroid().x() - centerX) * right.weight()
                ) / Math.max(EPSILON, sideWeight);
                double y = weightedAverage(
                    left.centroid().y(), left.weight(), right.centroid().y(), right.weight(),
                    0.5 * (bounds.minimum().y() + bounds.maximum().y())
                );
                double z = weightedAverage(
                    left.centroid().z(), left.weight(), right.centroid().z(), right.weight(),
                    bounds.minimum().z() + bounds.spanZ() * 0.10
                );
                TrianglePair pair = nearestMirroredLiftingPair(
                    source, Axis.LATERAL, centerX, centerBand,
                    new Vec3d(centerX - radius, y, z),
                    new Vec3d(centerX + radius, y, z), bounds
                );
                double halfWeight = sideWeight * 0.5;
                output.add(new LiftingPatch(kind.name() + "_L", kind,
                    pair.left(), halfWeight, pairId, SymmetryRole.MIRRORED_LEFT));
                output.add(new LiftingPatch(kind.name() + "_R", kind,
                    pair.right(), halfWeight, pairId, SymmetryRole.MIRRORED_RIGHT));
                ++pairId;
            }
        }

        if (hasCenter) {
            Vec3d point = nearestCenterlineLiftingPoint(
                source, Axis.LATERAL, centerX, centerBand, center.centroid(), bounds
            );
            if (point != null) {
                output.add(new LiftingPatch(kind.name() + "_CENTER", kind,
                    point, center.weight(), -1, SymmetryRole.CENTERLINE));
            }
        } else if (!(hasLeft && hasRight)) {
            LiftingSummary dominant = left.weight() >= right.weight() ? left : right;
            boolean leftDominant = left.weight() >= right.weight();
            if (credibleOneSidedLifting(dominant, bounds, centerX)) {
                Vec3d point = nearestLiftingPoint(
                    source, Axis.LATERAL, centerX, leftDominant ? -1 : 1,
                    centerBand, dominant.centroid()
                );
                output.add(new LiftingPatch(
                    kind.name() + (leftDominant ? "_LEFT_ONLY_CONFIRMED" : "_RIGHT_ONLY_CONFIRMED"),
                    kind, point, dominant.weight(), -1,
                    leftDominant ? SymmetryRole.LEFT_ONLY : SymmetryRole.RIGHT_ONLY
                ));
            } else {
                LiftingSummary summary = summarizeLifting(source, Axis.LATERAL, centerX, 2, 0.0);
                Vec3d point = nearestCenterlineLiftingPoint(
                    source, Axis.LATERAL, centerX, centerBand, summary.centroid(), bounds
                );
                if (point != null) {
                    output.add(new LiftingPatch(kind.name() + "_CENTER", kind,
                        point, summary.weight(), -1, SymmetryRole.CENTERLINE));
                }
            }
        }
        return pairId;
    }

    static double liftingBranchSpanY(
        List<ClassifiedTriangle> source,
        double centerX,
        int side,
        double tolerance
    ) {
        double minimum = Double.POSITIVE_INFINITY;
        double maximum = Double.NEGATIVE_INFINITY;
        if (source != null) {
            for (ClassifiedTriangle triangle : source) {
                double offset = triangle.centroid().x() - centerX;
                if ((side < 0 && offset >= -tolerance)
                    || (side > 0 && offset <= tolerance)) {
                    continue;
                }
                double projection = Math.abs(triangle.normal().x());
                if (projection < 0.08) {
                    continue;
                }
                minimum = Math.min(minimum, triangle.centroid().y());
                maximum = Math.max(maximum, triangle.centroid().y());
            }
        }
        return maximum > minimum ? maximum - minimum : 0.0;
    }

    static boolean credibleOneSidedLifting(
        LiftingSummary summary,
        Bounds bounds,
        double centerX
    ) {
        if (summary == null || summary.weight() <= EPSILON || !bounds.valid()) {
            return false;
        }
        double offset = Math.abs(summary.centroid().x() - centerX);
        double minimumOffset = Math.max(0.20, bounds.spanX() * 0.08);
        double minimumWeight = Math.max(0.02, bounds.spanX() * bounds.spanZ() * 0.0015);
        return offset >= minimumOffset && summary.weight() >= minimumWeight;
    }

    static LiftingSummary summarizeLifting(
        List<ClassifiedTriangle> source,
        Axis projectedAxis,
        double centerX,
        int side,
        double tolerance
    ) {
        Vec3d pointSum = Vec3d.ZERO;
        double weightSum = 0.0;
        for (ClassifiedTriangle triangle : source) {
            double offset = triangle.centroid().x() - centerX;
            boolean include = switch (side) {
                case -1 -> offset < -tolerance;
                case 1 -> offset > tolerance;
                case 0 -> Math.abs(offset) <= tolerance;
                default -> true;
            };
            if (!include) {
                continue;
            }
            double projection = projectedAxis == Axis.VERTICAL
                ? Math.abs(triangle.normal().y())
                : Math.abs(triangle.normal().x());
            double weight = triangle.area() * Math.max(0.03, projection);
            weightSum += weight;
            pointSum = pointSum.add(triangle.centroid().scale(weight));
        }
        return weightSum > EPSILON
            ? new LiftingSummary(pointSum.scale(1.0 / weightSum), weightSum)
            : new LiftingSummary(Vec3d.ZERO, 0.0);
    }

    static double weightedCoordinate(
        LiftingSummary left,
        LiftingSummary right,
        LiftingSummary center,
        int axis,
        double fallback
    ) {
        double total = left.weight() + right.weight() + center.weight();
        if (total <= EPSILON) {
            return fallback;
        }
        return (coordinate(left.centroid(), axis) * left.weight()
            + coordinate(right.centroid(), axis) * right.weight()
            + coordinate(center.centroid(), axis) * center.weight()) / total;
    }
}
