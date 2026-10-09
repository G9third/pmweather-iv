package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.Bounds;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.ClassifiedTriangle;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.EPSILON;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.IncidenceSample;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.IncidenceTrend;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.LiftingFrame;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.LiftingPatch;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.NormalFit;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.PressurePatch;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.SurfaceKind;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.SymmetryRole;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.WeightedScalar;
import static com.g9third.pmweatheriv.physics.BodyPressureGeometry.weightedQuantile;

/** Fits and regularizes lifting frames and aerodynamic centers. */
final class LiftingSurfaceFrames {
    private LiftingSurfaceFrames() {}

    /**
     * Attaches one frozen local aerodynamic frame to every model-derived lifting
     * patch.  The plane normal comes from area-weighted classified triangles on
     * the same exposed side/span band; buried main-wing geometry inside the robust
     * fuselage envelope is excluded from this orientation fit.  Chord is aircraft-
     * forward projected into that plane; span is the orthogonal in-plane axis.
     * This preserves incidence, washout/twist, dihedral/anhedral, canted fins,
     * and non-body-aligned control surfaces without any aircraft-name lookup.
     */
    static List<LiftingPatch> attachGeometryFrames(
        List<LiftingPatch> patches,
        EnumMap<SurfaceKind, List<ClassifiedTriangle>> byKind,
        Bounds bounds,
        Bounds bodyBounds
    ) {
        if (patches == null || patches.isEmpty()) {
            return patches == null ? List.of() : patches;
        }
        List<LiftingPatch> framed = new ArrayList<>(patches.size());
        for (LiftingPatch patch : patches) {
            List<ClassifiedTriangle> source = byKind.get(patch.kind());
            LiftingFrame frame = source == null || source.isEmpty()
                ? defaultLiftingFrame(patch.kind())
                : liftingFrameForPatch(source, patch, bounds, bodyBounds);
            framed.add(new LiftingPatch(
                patch.name(), patch.kind(), patch.pointLocal(), patch.weight(),
                patch.symmetryPair(), patch.symmetryRole(), patch.sectionIndex(),
                patch.sectionCount(), patch.sectionMinimumRadius(),
                patch.sectionMaximumRadius(), frame.spanLocal(), frame.chordLocal(),
                frame.normalLocal(), frame.confidence(), patch.pointLocal()
            ));
        }
        List<LiftingPatch> symmetricFrames = symmetrizeLiftingFrames(framed, bounds);
        List<LiftingPatch> coherentFrames = regularizeParentSurfaceFrames(symmetricFrames, bounds, bodyBounds);
        List<LiftingPatch> tailCoherentFrames =
            regularizeHorizontalTailParentFrames(coherentFrames);
        List<LiftingPatch> resymmetrizedFrames = symmetrizeLiftingFrames(tailCoherentFrames, bounds);
        List<LiftingPatch> centered = new ArrayList<>(resymmetrizedFrames.size());
        for (LiftingPatch patch : resymmetrizedFrames) {
            List<ClassifiedTriangle> source = byKind.get(patch.kind());
            boolean quarterChordSurface = patch.kind() == SurfaceKind.WING
                || patch.kind() == SurfaceKind.AILERON
                || patch.kind() == SurfaceKind.ELEVON
                || patch.kind() == SurfaceKind.HORIZONTAL_TAIL
                || patch.kind() == SurfaceKind.ELEVATOR
                || patch.kind() == SurfaceKind.TAILERON;
            Vec3d aerodynamicCenter = quarterChordSurface
                && source != null && !source.isEmpty()
                    ? liftingAerodynamicCenterForPatch(source, patch, bounds, bodyBounds)
                    : patch.pointLocal();
            centered.add(copyLiftingAerodynamicCenter(patch, aerodynamicCenter));
        }
        List<LiftingPatch> symmetricCenters = symmetrizeLiftingAerodynamicCenters(
            centered, bounds
        );
        List<LiftingPatch> coherentWingCenters = regularizeMainWingAerodynamicCenters(
            symmetricCenters, byKind, bounds, bodyBounds
        );
        return symmetrizeLiftingAerodynamicCenters(
            inheritParentAerodynamicCenters(coherentWingCenters, bounds), bounds
        );
    }

    /**
     * Removes isolated quarter-chord spikes without replacing the real wing
     * planform by a global straight-line fit.  A genuine swept/tapered wing
     * normally keeps a locally monotonic aerodynamic-centre line, including at a
     * planform kink.  Therefore only an interior local extremum that departs from
     * neighbor interpolation by a substantial fraction of the measured local
     * chord is repaired. Endpoints and monotonic sweep changes remain untouched.
     */
    static List<LiftingPatch> regularizeMainWingAerodynamicCenters(
        List<LiftingPatch> source,
        EnumMap<SurfaceKind, List<ClassifiedTriangle>> byKind,
        Bounds bounds,
        Bounds bodyBounds
    ) {
        if (source == null || source.isEmpty() || bounds == null || !bounds.valid()) {
            return source == null ? List.of() : List.copyOf(source);
        }
        double centerX = 0.5 * (bounds.minimum().x() + bounds.maximum().x());
        List<LiftingPatch> reference = new ArrayList<>();
        for (LiftingPatch patch : source) {
            if (patch.kind() != SurfaceKind.WING
                || patch.symmetryRole() == SymmetryRole.MIRRORED_LEFT
                || patch.symmetryRole() == SymmetryRole.LEFT_ONLY) {
                continue;
            }
            if (bodyBounds != null && bodyBounds.valid()
                && patch.pointLocal().x() > bodyBounds.minimum().x()
                && patch.pointLocal().x() < bodyBounds.maximum().x()) {
                continue;
            }
            reference.add(patch);
        }
        reference.sort(Comparator.comparingDouble(
            patch -> Math.abs(patch.pointLocal().x() - centerX)
        ));
        if (reference.size() < 3) {
            return List.copyOf(source);
        }

        Map<Integer, Double> correctedByPair = new HashMap<>();
        List<ClassifiedTriangle> wingTriangles = byKind == null
            ? List.of() : byKind.getOrDefault(SurfaceKind.WING, List.of());
        for (int index = 1; index + 1 < reference.size(); ++index) {
            LiftingPatch previous = reference.get(index - 1);
            LiftingPatch patch = reference.get(index);
            LiftingPatch next = reference.get(index + 1);
            double r0 = Math.abs(previous.pointLocal().x() - centerX);
            double r1 = Math.abs(patch.pointLocal().x() - centerX);
            double r2 = Math.abs(next.pointLocal().x() - centerX);
            if (!(r2 > r0 + EPSILON) || r1 <= r0 || r1 >= r2) {
                continue;
            }
            double z0 = previous.aerodynamicCenterLocal().z();
            double z1 = patch.aerodynamicCenterLocal().z();
            double z2 = next.aerodynamicCenterLocal().z();
            // Only an isolated reversal is suspicious. A genuine swept/kinked
            // planform whose quarter-chord continues monotonically is preserved.
            if ((z1 - z0) * (z2 - z1) >= 0.0) {
                continue;
            }
            double expected = z0 + (z2 - z0) * ((r1 - r0) / (r2 - r0));
            double deviation = Math.abs(z1 - expected);
            double localChord = measuredLiftingChordLength(
                wingTriangles, patch, bounds, bodyBounds
            );
            double tolerance = Math.max(0.20, 0.30 * Math.max(0.25, localChord));
            if (!Double.isFinite(expected) || !Double.isFinite(deviation)
                || deviation <= tolerance || patch.symmetryPair() < 0) {
                continue;
            }
            correctedByPair.put(patch.symmetryPair(), expected);
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(String.format(
                Locale.ROOT,
                "MODEL_AERODYNAMIC_CENTER_REPAIR surface=WING symmetryPair=%d rawZ=%.4f repairedZ=%.4f deviation=%.4f tolerance=%.4f localChord=%.4f policy=ISOLATED_LOCAL_EXTREMUM",
                patch.symmetryPair(), z1, expected, deviation, tolerance, localChord
            ));
            }
        }
        if (correctedByPair.isEmpty()) {
            return List.copyOf(source);
        }
        List<LiftingPatch> result = new ArrayList<>(source.size());
        for (LiftingPatch patch : source) {
            Double repairedZ = patch.kind() == SurfaceKind.WING && patch.symmetryPair() >= 0
                ? correctedByPair.get(patch.symmetryPair()) : null;
            if (repairedZ == null) {
                result.add(patch);
            } else {
                Vec3d old = patch.aerodynamicCenterLocal();
                result.add(copyLiftingAerodynamicCenter(
                    patch, new Vec3d(old.x(), old.y(), repairedZ)
                ));
            }
        }
        return List.copyOf(result);
    }

    static double measuredLiftingChordLength(
        List<ClassifiedTriangle> source,
        LiftingPatch patch,
        Bounds bounds,
        Bounds bodyBounds
    ) {
        List<ClassifiedTriangle> localTriangles = liftingFrameTriangles(
            source, patch, bounds, bodyBounds, true
        );
        if (localTriangles.isEmpty()) {
            return 0.0;
        }
        Vec3d chord = finiteUnitOr(patch.chordLocal(), new Vec3d(0.0, 0.0, 1.0));
        List<WeightedScalar> samples = new ArrayList<>(localTriangles.size() * 3);
        for (ClassifiedTriangle triangle : localTriangles) {
            Vec3d normal = triangle.normal().normalized();
            double projection = Math.abs(normal.y());
            double totalWeight = triangle.area() * Math.max(0.03, projection);
            if (!Double.isFinite(totalWeight) || totalWeight <= EPSILON) {
                continue;
            }
            double vertexWeight = totalWeight / 3.0;
            samples.add(new WeightedScalar(triangle.first().dot(chord), vertexWeight));
            samples.add(new WeightedScalar(triangle.second().dot(chord), vertexWeight));
            samples.add(new WeightedScalar(triangle.third().dot(chord), vertexWeight));
        }
        if (samples.size() < 6) {
            return 0.0;
        }
        samples.sort(Comparator.comparingDouble(WeightedScalar::value));
        double length = weightedScalarQuantile(samples, 0.99)
            - weightedScalarQuantile(samples, 0.01);
        return Double.isFinite(length) && length > 0.0 ? length : 0.0;
    }

    static double weightedScalarQuantile(
        List<WeightedScalar> sorted,
        double quantile
    ) {
        double totalWeight = 0.0;
        for (WeightedScalar sample : sorted) {
            totalWeight += Math.max(0.0, sample.weight());
        }
        if (totalWeight <= EPSILON) {
            return sorted.get(sorted.size() / 2).value();
        }
        double target = Vec3d.clamp(quantile, 0.0, 1.0) * totalWeight;
        double accumulated = 0.0;
        for (WeightedScalar sample : sorted) {
            accumulated += Math.max(0.0, sample.weight());
            if (accumulated + EPSILON >= target) {
                return sample.value();
            }
        }
        return sorted.get(sorted.size() - 1).value();
    }

    /**
     * Places attached control-surface circulation at the aerodynamic centre of
     * its real parent lifting surface.
     *
     * <p>A classified aileron or elevator mesh occupies only the trailing edge.
     * Its own quarter-chord is therefore still near the trailing edge of the
     * complete airfoil. Applying the attached circulation there gives the
     * control partition a false pitch lever and makes control response depend on
     * how a content pack happened to split its render mesh. The neutral frame is
     * already inherited from the parent surface; inherit the parent's chordwise
     * aerodynamic-centre projection for the same reason. Sampling remains at the
     * original control mesh point, and authored deflection, area, span station,
     * dihedral, dynamic stall, and separated-pressure placement are unchanged.</p>
     */
    static List<LiftingPatch> inheritParentAerodynamicCenters(
        List<LiftingPatch> source,
        Bounds bounds
    ) {
        if (source == null || source.isEmpty() || bounds == null || !bounds.valid()) {
            return source == null ? List.of() : List.copyOf(source);
        }
        double centerX = 0.5 * (bounds.minimum().x() + bounds.maximum().x());
        List<LiftingPatch> result = new ArrayList<>(source);
        for (int index = 0; index < result.size(); ++index) {
            LiftingPatch control = result.get(index);
            SurfaceKind parentKind = switch (control.kind()) {
                case AILERON, ELEVON -> SurfaceKind.WING;
                case ELEVATOR -> SurfaceKind.HORIZONTAL_TAIL;
                // A TAILERON is an all-moving/mixed tail surface.  Keep its own
                // aerodynamic centre rather than projecting it onto a possibly
                // absent fixed stabilizer parent.
                case TAILERON -> null;
                default -> null;
            };
            if (parentKind == null) {
                continue;
            }
            LiftingPatch parent = nearestParentPatch(
                source, control, parentKind, centerX
            );
            if (parent == null) {
                continue;
            }
            Vec3d chord = finiteUnitOr(
                control.chordLocal(), new Vec3d(0.0, 0.0, 1.0)
            );
            double targetProjection = parent.aerodynamicCenterLocal().dot(chord);
            double currentProjection = control.pointLocal().dot(chord);
            double shift = targetProjection - currentProjection;
            if (Double.isFinite(shift)) {
                result.set(index, copyLiftingAerodynamicCenter(
                    control, control.pointLocal().add(chord.scale(shift))
                ));
            }
        }
        return List.copyOf(result);
    }

    static LiftingPatch nearestParentPatch(
        List<LiftingPatch> patches,
        LiftingPatch control,
        SurfaceKind parentKind,
        double centerX
    ) {
        LiftingPatch best = null;
        double bestScore = Double.POSITIVE_INFINITY;
        double controlOffset = control.pointLocal().x() - centerX;
        for (LiftingPatch candidate : patches) {
            if (candidate.kind() != parentKind
                || !sameLiftingSide(control, candidate, centerX)) {
                continue;
            }
            double candidateOffset = candidate.pointLocal().x() - centerX;
            double score = Math.abs(Math.abs(controlOffset) - Math.abs(candidateOffset));
            if (candidate.adaptiveSection()) {
                double radius = Math.abs(controlOffset);
                if (radius < candidate.sectionMinimumRadius()) {
                    score = candidate.sectionMinimumRadius() - radius;
                } else if (radius > candidate.sectionMaximumRadius()) {
                    score = radius - candidate.sectionMaximumRadius();
                } else {
                    score = 0.0;
                }
            }
            if (score < bestScore) {
                bestScore = score;
                best = candidate;
            }
        }
        return best;
    }

    static boolean sameLiftingSide(
        LiftingPatch first,
        LiftingPatch second,
        double centerX
    ) {
        double firstOffset = first.pointLocal().x() - centerX;
        double secondOffset = second.pointLocal().x() - centerX;
        double tolerance = 1.0E-6;
        if (Math.abs(firstOffset) <= tolerance) {
            // A single full-span control partition may be centered even when
            // the fixed parent was split into left/right adaptive bands. Both
            // sides are symmetric here; nearestParentPatch chooses the root-
            // nearest parent and the final center is symmetrized.
            return true;
        }
        if (Math.abs(secondOffset) <= tolerance) {
            // Centerline parent geometry is a valid universal fallback when a
            // side-specific parent band is absent.
            return true;
        }
        return firstOffset * secondOffset > 0.0;
    }

    /**
     * Converts noisy per-patch skin-normal incidence into a coherent parent-
     * surface incidence field without adding any aircraft-specific tuning.
     *
     * <p>Airfoil shells are curved, and adaptive span bands do not necessarily
     * contain identical upper/lower tessellation.  Treating each band's average
     * skin normal as an independent neutral chord can therefore create alternating
     * incidence from one neighboring wing section to the next.  A real built wing
     * has a continuous spar/chord structure: spanwise twist may vary, but it varies
     * coherently.  We fit one area/confidence-weighted linear incidence trend over
     * the frozen main-wing sections and apply that trend while preserving each
     * section's own span axis, so dihedral/anhedral remains local geometry.</p>
     *
     * <p>Moving roll controls are then referenced to the parent aerodynamic
     * surface at zero authored deflection. Ailerons/elevons inherit their
     * main-wing neutral chord when a fixed parent wing exists. The solver still
     * applies the content pack's authored control angle dynamically, so this does
     * not change control travel or introduce a roll-rate target. Horizontal-tail
     * coherence is handled in the subsequent combined fixed-tail/elevator parent
     * pass; vertical-tail/rudder frames remain independently mesh-derived.</p>
     */
    static List<LiftingPatch> regularizeParentSurfaceFrames(
        List<LiftingPatch> source,
        Bounds bounds,
        Bounds bodyBounds
    ) {
        if (source == null || source.isEmpty() || !bounds.valid()) {
            return source == null ? List.of() : List.copyOf(source);
        }
        List<LiftingPatch> result = new ArrayList<>(source);
        double centerX = 0.5 * (bounds.minimum().x() + bounds.maximum().x());

        IncidenceTrend wingTrend = fitMainWingIncidenceTrend(result, centerX, bodyBounds);
        if (wingTrend.valid()) {
            for (int index = 0; index < result.size(); ++index) {
                LiftingPatch patch = result.get(index);
                if (patch.kind() != SurfaceKind.WING) {
                    continue;
                }
                double radius = Math.abs(patch.pointLocal().x() - centerX);
                result.set(index, withIncidenceDegrees(
                    patch, wingTrend.incidenceDegrees(radius)
                ));
            }
        }

        // Zero-deflection roll controls belong to the main-wing parent plane.
        // Keep tail controls unchanged in this build: some content packs do not
        // provide a separately trustworthy fixed-stabilizer mesh, so inferring
        // elevator/rudder neutral geometry from a generic tail classifier would
        // be less reliable than the control mesh itself.
        for (int index = 0; index < result.size(); ++index) {
            LiftingPatch patch = result.get(index);
            if ((patch.kind() == SurfaceKind.AILERON || patch.kind() == SurfaceKind.ELEVON)
                && wingTrend.valid()) {
                double radius = Math.abs(patch.pointLocal().x() - centerX);
                result.set(index, withIncidenceDegrees(
                    patch, wingTrend.incidenceDegrees(radius)
                ));
            }
        }
        return List.copyOf(result);
    }

    /**
     * Coheres attached elevators with their measured fixed stabilizer plane.
     * Each side keeps the stabilizer's area/confidence-weighted mesh incidence;
     * all-moving tailerons and controls without a measured parent keep their own
     * fitted frame. No universal tail offset or trim bias is added.
     */
    static List<LiftingPatch> regularizeHorizontalTailParentFrames(List<LiftingPatch> source) {
        if (source == null || source.isEmpty()) return source == null ? List.of() : List.copyOf(source);
        List<LiftingPatch> result = new ArrayList<>(source);
        for (SymmetryRole role : List.of(SymmetryRole.MIRRORED_LEFT, SymmetryRole.MIRRORED_RIGHT,
                SymmetryRole.CENTERLINE, SymmetryRole.UNPAIRED)) {
            List<Integer> indices = horizontalTailParentIndices(result, role);
            double weightedIncidence = 0.0, totalWeight = 0.0;
            for (int index : indices) {
                LiftingPatch patch = result.get(index);
                if (patch.kind() != SurfaceKind.HORIZONTAL_TAIL || patch.frameConfidence() <= EPSILON) continue;
                double incidence = patch.geometryIncidenceDegrees();
                double weight = Math.max(0, patch.weight()) * patch.frameConfidence();
                if (!Double.isFinite(incidence) || !Double.isFinite(weight)) continue;
                weightedIncidence += incidence * weight;
                totalWeight += weight;
            }
            if (totalWeight <= EPSILON) continue;
            double measuredIncidence = weightedIncidence / totalWeight;
            for (int index : indices) {
                LiftingPatch patch = result.get(index);
                if (patch.kind() != SurfaceKind.TAILERON)
                    result.set(index, withIncidenceDegrees(patch, measuredIncidence));
            }
        }
        return List.copyOf(result);
    }

    static List<Integer> horizontalTailParentIndices(
        List<LiftingPatch> patches,
        SymmetryRole role
    ) {
        List<Integer> indices = new ArrayList<>();
        for (int index = 0; index < patches.size(); ++index) {
            LiftingPatch patch = patches.get(index);
            if (!horizontalTailParentRoleMatches(patch.symmetryRole(), role)) {
                continue;
            }
            if (patch.kind() == SurfaceKind.HORIZONTAL_TAIL
                || patch.kind() == SurfaceKind.ELEVATOR
                || patch.kind() == SurfaceKind.TAILERON) {
                indices.add(index);
            }
        }
        return indices;
    }

    static boolean horizontalTailParentRoleMatches(
        SymmetryRole patchRole,
        SymmetryRole parentRole
    ) {
        if (parentRole == SymmetryRole.MIRRORED_LEFT) {
            return patchRole == SymmetryRole.MIRRORED_LEFT
                || patchRole == SymmetryRole.LEFT_ONLY;
        }
        if (parentRole == SymmetryRole.MIRRORED_RIGHT) {
            return patchRole == SymmetryRole.MIRRORED_RIGHT
                || patchRole == SymmetryRole.RIGHT_ONLY;
        }
        return patchRole == parentRole;
    }

    static IncidenceTrend fitMainWingIncidenceTrend(
        List<LiftingPatch> patches,
        double centerX,
        Bounds bodyBounds
    ) {
        List<IncidenceSample> samples = new ArrayList<>();
        for (LiftingPatch patch : patches) {
            if (patch.kind() != SurfaceKind.WING) {
                continue;
            }
            // After frame symmetrization both members of a mirrored pair have
            // the same neutral incidence. Keep one representative so a pair is
            // not counted twice. One-sided geometry remains eligible.
            if (patch.symmetryRole() == SymmetryRole.MIRRORED_LEFT) {
                continue;
            }
            double radius = Math.abs(patch.pointLocal().x() - centerX);
            // Root/fairing patches inside the robust fuselage envelope are useful
            // as force-area holders but are not independent evidence of wing
            // incidence. Their frame may already be a side-level fallback because
            // buried shell triangles were intentionally excluded above. Fit the
            // parent trend only from exposed span stations, then extrapolate it
            // smoothly through the root.
            if (bodyBounds != null && bodyBounds.valid()
                && patch.pointLocal().x() > bodyBounds.minimum().x()
                && patch.pointLocal().x() < bodyBounds.maximum().x()) {
                continue;
            }
            double incidence = patch.geometryIncidenceDegrees();
            double weight = Math.max(EPSILON, patch.weight())
                * Math.max(0.05, patch.frameConfidence());
            if (Double.isFinite(radius) && Double.isFinite(incidence)
                && Double.isFinite(weight) && weight > EPSILON) {
                samples.add(new IncidenceSample(radius, incidence, weight));
            }
        }
        if (samples.size() < 2) {
            return IncidenceTrend.INVALID;
        }

        double weightSum = 0.0;
        double radiusMean = 0.0;
        double incidenceMean = 0.0;
        for (IncidenceSample sample : samples) {
            weightSum += sample.weight();
            radiusMean += sample.weight() * sample.radius();
            incidenceMean += sample.weight() * sample.incidenceDegrees();
        }
        if (weightSum <= EPSILON) {
            return IncidenceTrend.INVALID;
        }
        radiusMean /= weightSum;
        incidenceMean /= weightSum;

        double radiusVariance = 0.0;
        double covariance = 0.0;
        for (IncidenceSample sample : samples) {
            double radiusOffset = sample.radius() - radiusMean;
            double incidenceOffset = sample.incidenceDegrees() - incidenceMean;
            radiusVariance += sample.weight() * radiusOffset * radiusOffset;
            covariance += sample.weight() * radiusOffset * incidenceOffset;
        }
        double slope = radiusVariance > EPSILON ? covariance / radiusVariance : 0.0;
        double intercept = incidenceMean - slope * radiusMean;
        if (!Double.isFinite(slope) || !Double.isFinite(intercept)) {
            return IncidenceTrend.INVALID;
        }
        return new IncidenceTrend(intercept, slope, true);
    }

    static LiftingPatch withIncidenceDegrees(
        LiftingPatch patch,
        double incidenceDegrees
    ) {
        if (!Double.isFinite(incidenceDegrees)) {
            return patch;
        }
        Vec3d span = finiteUnitOr(
            patch.spanLocal(),
            patch.vertical() ? new Vec3d(0.0, 1.0, 0.0) : new Vec3d(1.0, 0.0, 0.0)
        );
        double tangent = Math.tan(Math.toRadians(incidenceDegrees));
        Vec3d provisionalChord = patch.vertical()
            ? new Vec3d(tangent, 0.0, 1.0)
            : new Vec3d(0.0, tangent, 1.0);
        Vec3d chord = provisionalChord.subtract(
            span.scale(provisionalChord.dot(span))
        ).normalized();
        if (chord.lengthSquared() <= EPSILON) {
            return patch;
        }
        if (chord.z() < 0.0) {
            chord = chord.negate();
        }
        Vec3d normal = chord.cross(span).normalized();
        if (normal.lengthSquared() <= EPSILON) {
            return patch;
        }
        Vec3d preferredNormal = patch.vertical()
            ? new Vec3d(-1.0, 0.0, 0.0)
            : new Vec3d(0.0, 1.0, 0.0);
        if (normal.dot(preferredNormal) < 0.0) {
            span = span.negate();
            normal = chord.cross(span).normalized();
        }
        return copyLiftingFrame(
            patch, span, chord, normal, patch.frameConfidence()
        );
    }

    /**
     * Finds the actual local quarter-chord of a frozen model-backed lifting partition.
     *
     * <p>The sampling point stays on the retained mesh so wind and rotational
     * point-flow queries are unchanged.  Only the force application point moves
     * to the section's physical aerodynamic center.  Earlier builds shifted every
     * section forward by one quarter of the aircraft-wide mean chord.  On swept
     * or tapered wings that can leave large inboard panels far aft of their true
     * quarter chord while pushing short outer panels too far forward, creating a
     * false pitch moment.  This method instead projects the same frozen triangles
     * used by that section onto its frozen chord axis and takes the 25%-chord
     * point between robust trailing- and leading-edge limits.</p>
     */
    static Vec3d liftingAerodynamicCenterForPatch(
        List<ClassifiedTriangle> source,
        LiftingPatch patch,
        Bounds bounds,
        Bounds bodyBounds
    ) {
        List<ClassifiedTriangle> localTriangles = liftingFrameTriangles(
            source, patch, bounds, bodyBounds, true
        );
        // Do not borrow another span station when an adaptive section itself has
        // no credible exposed geometry.  A tiny/root fallback patch is safer at
        // its existing surface point than at an aerodynamic center from a
        // different part of the wing.
        if (localTriangles.isEmpty()) {
            return patch.pointLocal();
        }

        Vec3d chord = finiteUnitOr(patch.chordLocal(), new Vec3d(0.0, 0.0, 1.0));
        List<WeightedScalar> chordSamples = new ArrayList<>(localTriangles.size() * 3);
        for (ClassifiedTriangle triangle : localTriangles) {
            Vec3d normal = triangle.normal().normalized();
            double projection = patch.vertical() ? Math.abs(normal.x()) : Math.abs(normal.y());
            double totalWeight = triangle.area() * Math.max(0.03, projection);
            if (!Double.isFinite(totalWeight) || totalWeight <= EPSILON) {
                continue;
            }
            double vertexWeight = totalWeight / 3.0;
            chordSamples.add(new WeightedScalar(triangle.first().dot(chord), vertexWeight));
            chordSamples.add(new WeightedScalar(triangle.second().dot(chord), vertexWeight));
            chordSamples.add(new WeightedScalar(triangle.third().dot(chord), vertexWeight));
        }
        if (chordSamples.size() < 6) {
            return patch.pointLocal();
        }

        chordSamples.sort(Comparator.comparingDouble(WeightedScalar::value));
        double trailing = weightedQuantile(chordSamples, 0.01);
        double leading = weightedQuantile(chordSamples, 0.99);
        double chordLength = leading - trailing;
        if (!Double.isFinite(chordLength) || chordLength <= 0.05) {
            return patch.pointLocal();
        }

        // chord points aircraft-forward, so the leading edge is the larger
        // projection.  Quarter chord is 25% aft from that leading edge.
        double quarterChordProjection = leading - 0.25 * chordLength;
        double currentProjection = patch.pointLocal().dot(chord);
        double shift = quarterChordProjection - currentProjection;
        if (!Double.isFinite(shift)) {
            return patch.pointLocal();
        }
        return patch.pointLocal().add(chord.scale(shift));
    }

    static double weightedQuantile(List<WeightedScalar> sorted, double quantile) {
        double totalWeight = 0.0;
        for (WeightedScalar sample : sorted) {
            totalWeight += Math.max(0.0, sample.weight());
        }
        if (totalWeight <= EPSILON) {
            return sorted.get(sorted.size() / 2).value();
        }
        double target = Vec3d.clamp(quantile, 0.0, 1.0) * totalWeight;
        double cumulative = 0.0;
        for (WeightedScalar sample : sorted) {
            cumulative += Math.max(0.0, sample.weight());
            if (cumulative + EPSILON >= target) {
                return sample.value();
            }
        }
        return sorted.get(sorted.size() - 1).value();
    }

    static LiftingFrame liftingFrameForPatch(
        List<ClassifiedTriangle> source,
        LiftingPatch patch,
        Bounds bounds,
        Bounds bodyBounds
    ) {
        boolean vertical = patch.vertical();
        Vec3d preferredNormal = vertical
            ? new Vec3d(-1.0, 0.0, 0.0)
            : new Vec3d(0.0, 1.0, 0.0);
        Vec3d preferredSpan = vertical
            ? new Vec3d(0.0, 1.0, 0.0)
            : new Vec3d(1.0, 0.0, 0.0);
        Vec3d preferredChord = new Vec3d(0.0, 0.0, 1.0);

        List<ClassifiedTriangle> localTriangles = liftingFrameTriangles(
            source, patch, bounds, bodyBounds, true
        );
        List<ClassifiedTriangle> sideReferenceTriangles = liftingFrameTriangles(
            source, patch, bounds, bodyBounds, false
        );
        if (localTriangles.isEmpty()) {
            localTriangles = sideReferenceTriangles;
        }
        if (sideReferenceTriangles.isEmpty()) {
            sideReferenceTriangles = source;
        }

        NormalFit localFit = fitLiftingPlaneNormal(localTriangles, vertical, preferredNormal);
        NormalFit referenceFit = fitLiftingPlaneNormal(
            sideReferenceTriangles, vertical, preferredNormal
        );
        Vec3d localNormal = localFit.normalLocal();
        Vec3d referenceNormal = referenceFit.normalLocal();
        if (localNormal.dot(referenceNormal) < 0.0) {
            localNormal = localNormal.negate();
        }

        // Coherent local triangles are allowed to carry twist/dihedral/cant.
        // Mixed root/fuselage/control geometry fades continuously toward the
        // whole-side component plane rather than imposing an arbitrary angle cap.
        double localBlend = Math.pow(Vec3d.clamp(localFit.coherence(), 0.0, 1.0), 4.0);
        Vec3d planeNormal = referenceNormal.scale(1.0 - localBlend)
            .add(localNormal.scale(localBlend)).normalized();
        if (planeNormal.lengthSquared() <= EPSILON) {
            planeNormal = referenceNormal.lengthSquared() > EPSILON
                ? referenceNormal
                : preferredNormal;
        }

        // Chord is the aircraft-forward direction projected into the actual local
        // lifting plane. This preserves incidence/twist/cant while avoiding noisy
        // triangle-edge tessellation from being mistaken for aerodynamic sweep.
        // Planform sweep is already represented by each patch's frozen position
        // and spanwise layout.
        Vec3d chord = preferredChord.subtract(
            planeNormal.scale(preferredChord.dot(planeNormal))
        ).normalized();
        if (chord.lengthSquared() <= EPSILON) {
            chord = preferredSpan.cross(planeNormal).normalized();
        }
        if (chord.lengthSquared() <= EPSILON) {
            return defaultLiftingFrame(patch.kind());
        }
        if (chord.z() < 0.0) {
            chord = chord.negate();
        }

        Vec3d span = planeNormal.cross(chord).normalized();
        if (span.lengthSquared() <= EPSILON) {
            return defaultLiftingFrame(patch.kind());
        }
        if (span.dot(preferredSpan) < 0.0) {
            span = span.negate();
        }
        Vec3d normal = chord.cross(span).normalized();
        if (normal.lengthSquared() <= EPSILON) {
            return defaultLiftingFrame(patch.kind());
        }
        double confidence = Vec3d.clamp(
            0.5 * localFit.coherence() + 0.5 * referenceFit.coherence(), 0.0, 1.0
        );
        return new LiftingFrame(span, chord, normal, confidence);
    }

    static List<ClassifiedTriangle> liftingFrameTriangles(
        List<ClassifiedTriangle> source,
        LiftingPatch patch,
        Bounds bounds,
        Bounds bodyBounds,
        boolean respectAdaptiveSection
    ) {
        if (source == null || source.isEmpty()) {
            return List.of();
        }
        double centerX = bounds.valid()
            ? 0.5 * (bounds.minimum().x() + bounds.maximum().x())
            : 0.0;
        double tolerance = bounds.valid() ? mirrorTolerance(bounds) : 0.05;
        List<ClassifiedTriangle> selected = new ArrayList<>();
        for (ClassifiedTriangle triangle : source) {
            double offset = triangle.centroid().x() - centerX;
            // Main-wing geometry inside the robust fuselage envelope is not a
            // trustworthy aerodynamic reference plane: it commonly consists of
            // root fairings, blended body panels, or buried wing shell.  Exclude
            // that occluded center strip from frame fitting.  This is determined
            // only from the same frozen model geometry; no vehicle identity or
            // aircraft-specific span fraction is involved.
            if (patch.kind() == SurfaceKind.WING && bodyBounds != null && bodyBounds.valid()
                && triangle.centroid().x() > bodyBounds.minimum().x()
                && triangle.centroid().x() < bodyBounds.maximum().x()) {
                continue;
            }
            if ((patch.symmetryRole() == SymmetryRole.MIRRORED_LEFT
                    || patch.symmetryRole() == SymmetryRole.LEFT_ONLY)
                && offset >= -tolerance) {
                continue;
            }
            if ((patch.symmetryRole() == SymmetryRole.MIRRORED_RIGHT
                    || patch.symmetryRole() == SymmetryRole.RIGHT_ONLY)
                && offset <= tolerance) {
                continue;
            }
            if (respectAdaptiveSection && patch.adaptiveSection()) {
                double radius = Math.abs(offset);
                if (radius + EPSILON < patch.sectionMinimumRadius()
                    || radius >= patch.sectionMaximumRadius()) {
                    continue;
                }
            }
            selected.add(triangle);
        }
        return selected;
    }

    static NormalFit fitLiftingPlaneNormal(
        List<ClassifiedTriangle> triangles,
        boolean vertical,
        Vec3d preferredNormal
    ) {
        if (triangles == null || triangles.isEmpty()) {
            return new NormalFit(preferredNormal, 0.0);
        }
        Vec3d normalSum = Vec3d.ZERO;
        double totalWeight = 0.0;
        for (ClassifiedTriangle triangle : triangles) {
            Vec3d normal = triangle.normal().normalized();
            if (normal.lengthSquared() <= EPSILON) {
                continue;
            }
            double projection = vertical ? Math.abs(normal.x()) : Math.abs(normal.y());
            if (projection < 0.05) {
                continue;
            }
            if (normal.dot(preferredNormal) < 0.0) {
                normal = normal.negate();
            }
            double weight = triangle.area() * projection * projection;
            normalSum = normalSum.add(normal.scale(weight));
            totalWeight += weight;
        }
        if (totalWeight <= EPSILON || normalSum.lengthSquared() <= EPSILON) {
            return new NormalFit(preferredNormal, 0.0);
        }
        double coherence = Vec3d.clamp(normalSum.length() / totalWeight, 0.0, 1.0);
        return new NormalFit(normalSum.normalized(), coherence);
    }

    static List<LiftingPatch> symmetrizeLiftingFrames(
        List<LiftingPatch> source,
        Bounds bounds
    ) {
        if (source == null || source.isEmpty() || !bounds.valid()) {
            return source == null ? List.of() : List.copyOf(source);
        }
        List<LiftingPatch> result = new ArrayList<>(source);
        Map<Integer, Integer> leftByPair = new HashMap<>();
        Map<Integer, Integer> rightByPair = new HashMap<>();
        for (int index = 0; index < result.size(); ++index) {
            LiftingPatch patch = result.get(index);
            if (patch.symmetryPair() < 0) {
                continue;
            }
            if (patch.symmetryRole() == SymmetryRole.MIRRORED_LEFT) {
                leftByPair.put(patch.symmetryPair(), index);
            } else if (patch.symmetryRole() == SymmetryRole.MIRRORED_RIGHT) {
                rightByPair.put(patch.symmetryPair(), index);
            }
        }
        for (Map.Entry<Integer, Integer> entry : leftByPair.entrySet()) {
            Integer rightIndex = rightByPair.get(entry.getKey());
            if (rightIndex == null) {
                continue;
            }
            int leftIndex = entry.getValue();
            LiftingPatch left = result.get(leftIndex);
            LiftingPatch right = result.get(rightIndex);
            if (left.kind() != right.kind()) {
                continue;
            }
            Vec3d mirroredLeftNormal = mirrorXVector(left.normalLocal());
            Vec3d averagedNormal = mirroredLeftNormal.add(right.normalLocal()).normalized();
            if (averagedNormal.lengthSquared() <= EPSILON) {
                averagedNormal = right.normalLocal();
            }
            Vec3d preferredChord = new Vec3d(0.0, 0.0, 1.0);
            Vec3d rightChord = preferredChord.subtract(
                averagedNormal.scale(preferredChord.dot(averagedNormal))
            ).normalized();
            if (rightChord.lengthSquared() <= EPSILON) {
                rightChord = right.chordLocal();
            }
            if (rightChord.z() < 0.0) {
                rightChord = rightChord.negate();
            }
            Vec3d preferredSpan = right.vertical()
                ? new Vec3d(0.0, 1.0, 0.0)
                : new Vec3d(1.0, 0.0, 0.0);
            Vec3d rightSpan = averagedNormal.cross(rightChord).normalized();
            if (rightSpan.dot(preferredSpan) < 0.0) {
                rightSpan = rightSpan.negate();
            }
            Vec3d rightNormal = rightChord.cross(rightSpan).normalized();
            Vec3d leftChord = mirrorXVector(rightChord);
            Vec3d leftSpan = mirrorXVector(rightSpan).negate();
            Vec3d leftNormal = leftChord.cross(leftSpan).normalized();
            double confidence = 0.5 * (left.frameConfidence() + right.frameConfidence());
            result.set(leftIndex, copyLiftingFrame(left, leftSpan, leftChord, leftNormal, confidence));
            result.set(rightIndex, copyLiftingFrame(right, rightSpan, rightChord, rightNormal, confidence));
        }
        return List.copyOf(result);
    }

    static LiftingPatch copyLiftingFrame(
        LiftingPatch patch,
        Vec3d span,
        Vec3d chord,
        Vec3d normal,
        double confidence
    ) {
        return new LiftingPatch(
            patch.name(), patch.kind(), patch.pointLocal(), patch.weight(),
            patch.symmetryPair(), patch.symmetryRole(), patch.sectionIndex(),
            patch.sectionCount(), patch.sectionMinimumRadius(), patch.sectionMaximumRadius(),
            span, chord, normal, confidence, patch.aerodynamicCenterLocal()
        );
    }

    static LiftingPatch copyLiftingAerodynamicCenter(
        LiftingPatch patch,
        Vec3d aerodynamicCenterLocal
    ) {
        return new LiftingPatch(
            patch.name(), patch.kind(), patch.pointLocal(), patch.weight(),
            patch.symmetryPair(), patch.symmetryRole(), patch.sectionIndex(),
            patch.sectionCount(), patch.sectionMinimumRadius(), patch.sectionMaximumRadius(),
            patch.spanLocal(), patch.chordLocal(), patch.normalLocal(), patch.frameConfidence(),
            aerodynamicCenterLocal
        );
    }

    static List<LiftingPatch> symmetrizeLiftingAerodynamicCenters(
        List<LiftingPatch> source,
        Bounds bounds
    ) {
        if (source.isEmpty() || !bounds.valid()) {
            return source;
        }
        double mirrorPlaneX = 0.5 * (bounds.minimum().x() + bounds.maximum().x());
        List<LiftingPatch> result = new ArrayList<>(source);
        Map<Integer, Integer> leftByPair = new HashMap<>();
        Map<Integer, Integer> rightByPair = new HashMap<>();
        for (int index = 0; index < result.size(); ++index) {
            LiftingPatch patch = result.get(index);
            if (patch.symmetryPair() < 0) {
                if (patch.symmetryRole() == SymmetryRole.CENTERLINE) {
                    Vec3d ac = patch.aerodynamicCenterLocal();
                    result.set(index, copyLiftingAerodynamicCenter(
                        patch, new Vec3d(mirrorPlaneX, ac.y(), ac.z())
                    ));
                }
                continue;
            }
            if (patch.symmetryRole() == SymmetryRole.MIRRORED_LEFT) {
                leftByPair.put(patch.symmetryPair(), index);
            } else if (patch.symmetryRole() == SymmetryRole.MIRRORED_RIGHT) {
                rightByPair.put(patch.symmetryPair(), index);
            }
        }
        for (Map.Entry<Integer, Integer> entry : leftByPair.entrySet()) {
            Integer rightIndex = rightByPair.get(entry.getKey());
            if (rightIndex == null) {
                continue;
            }
            int leftIndex = entry.getValue();
            LiftingPatch left = result.get(leftIndex);
            LiftingPatch right = result.get(rightIndex);
            Vec3d leftAc = left.aerodynamicCenterLocal();
            Vec3d rightAc = right.aerodynamicCenterLocal();
            double radius = 0.5 * (
                Math.abs(leftAc.x() - mirrorPlaneX)
                    + Math.abs(rightAc.x() - mirrorPlaneX)
            );
            double y = 0.5 * (leftAc.y() + rightAc.y());
            double z = 0.5 * (leftAc.z() + rightAc.z());
            result.set(leftIndex, copyLiftingAerodynamicCenter(
                left, new Vec3d(mirrorPlaneX - radius, y, z)
            ));
            result.set(rightIndex, copyLiftingAerodynamicCenter(
                right, new Vec3d(mirrorPlaneX + radius, y, z)
            ));
        }
        return List.copyOf(result);
    }

    static Vec3d mirrorXVector(Vec3d vector) {
        return new Vec3d(-vector.x(), vector.y(), vector.z());
    }

    static LiftingFrame defaultLiftingFrame(SurfaceKind kind) {
        boolean vertical = kind == SurfaceKind.VERTICAL_TAIL || kind == SurfaceKind.RUDDER;
        if (vertical) {
            return new LiftingFrame(
                new Vec3d(0.0, 1.0, 0.0),
                new Vec3d(0.0, 0.0, 1.0),
                new Vec3d(-1.0, 0.0, 0.0),
                0.0
            );
        }
        return new LiftingFrame(
            new Vec3d(1.0, 0.0, 0.0),
            new Vec3d(0.0, 0.0, 1.0),
            new Vec3d(0.0, 1.0, 0.0),
            0.0
        );
    }

    static Vec3d finiteUnitOr(Vec3d value, Vec3d fallback) {
        if (value == null || !value.isFinite() || value.lengthSquared() <= EPSILON) {
            return fallback;
        }
        Vec3d normalized = value.normalized();
        return normalized.lengthSquared() > EPSILON ? normalized : fallback;
    }

    static double mirrorTolerance(Bounds bounds) {
        return Math.max(0.015, Math.min(0.05, bounds.spanX() * 0.005));
    }

    static Vec3d mirrorX(Vec3d vector) {
        return new Vec3d(-vector.x(), vector.y(), vector.z());
    }

    static Vec3d mirrorX(Vec3d point, double mirrorPlaneX) {
        return new Vec3d(2.0 * mirrorPlaneX - point.x(), point.y(), point.z());
    }

    static int symmetrySortOrder(SymmetryRole role) {
        return switch (role) {
            case MIRRORED_LEFT, MIRRORED_RIGHT -> 0;
            case CENTERLINE -> 1;
            case LEFT_ONLY, RIGHT_ONLY -> 2;
            case UNPAIRED -> 3;
        };
    }

    static String symmetryCountsPressure(List<PressurePatch> patches) {
        return symmetryCounts(patches.stream().map(PressurePatch::symmetryRole).toList());
    }

    static String symmetryCountsLifting(List<LiftingPatch> patches) {
        return symmetryCounts(patches.stream().map(LiftingPatch::symmetryRole).toList());
    }

    static String symmetryCounts(List<SymmetryRole> roles) {
        EnumMap<SymmetryRole, Integer> counts = new EnumMap<>(SymmetryRole.class);
        for (SymmetryRole role : roles) {
            counts.merge(role, 1, Integer::sum);
        }
        StringBuilder result = new StringBuilder();
        for (SymmetryRole role : SymmetryRole.values()) {
            int count = counts.getOrDefault(role, 0);
            if (count > 0) {
                if (!result.isEmpty()) {
                    result.append(',');
                }
                result.append(role.name()).append('=').append(count);
            }
        }
        return result.toString();
    }

    static double weightedZ(List<LiftingPatch> patches, SurfaceKind... kinds) {
        double weighted = 0.0;
        double totalWeight = 0.0;
        for (LiftingPatch patch : patches) {
            boolean match = false;
            for (SurfaceKind kind : kinds) {
                if (patch.kind() == kind) {
                    match = true;
                    break;
                }
            }
            if (match) {
                weighted += patch.pointLocal().z() * patch.weight();
                totalWeight += patch.weight();
            }
        }
        return totalWeight > EPSILON ? weighted / totalWeight : Double.NaN;
    }
}
