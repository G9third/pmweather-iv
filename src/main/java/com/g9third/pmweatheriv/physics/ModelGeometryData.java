package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.model.ParsedModelSnapshot.Mesh;

import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.Bounds;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.SurfaceKind;

/** Package-local geometry values shared by model preparation stages. */
final class ModelGeometryData {
    private ModelGeometryData() {}

    static final double EPSILON = 1.0E-9;

    enum Axis {
        LATERAL,
        VERTICAL
    }

    record BodyWettedAreaEstimate(
        double rawTriangleArea,
        double exposedWettedArea,
        int sourceTriangles,
        int projectionDirections,
        int projectionResolution
    ) {
    }

    record ObjectStats(int validTriangles, double totalArea, Bounds bounds) {
    }

    record ModelObject(
        Mesh vertices,
        String objectName,
        String normalizedName,
        ObjectStats stats,
        double priority
    ) {
    }

    record QuotaRemainder(ModelObject object, double fraction, int maximum) {
    }

    record AnimationHint(
        boolean aileron,
        boolean elevator,
        boolean rudder,
        boolean flap,
        boolean slat,
        boolean spoiler
    ) {
        static final AnimationHint NONE = new AnimationHint(false, false, false, false, false, false);

        static AnimationHint fromVariable(String variable) {
            if (variable == null || variable.isEmpty()) {
                return NONE;
            }
            return new AnimationHint(
                variable.contains("aileron"),
                variable.contains("elevator"),
                variable.contains("rudder"),
                variable.contains("flap"),
                variable.contains("slat"),
                variable.contains("spoiler") || variable.contains("speedbrake") || variable.contains("speed_brake")
            );
        }

        /**
         * Control-variable names are useful evidence only when the animation changes
         * the object's physical transform. State, visibility, color, and sound
         * animations can mention a control without moving an aerodynamic surface.
         */
        static AnimationHint fromAnimationType(String animationType, String variable) {
            if (animationType == null
                || (!"rotation".equalsIgnoreCase(animationType)
                    && !"translation".equalsIgnoreCase(animationType))) {
                return NONE;
            }
            return fromVariable(variable);
        }

        AnimationHint combine(AnimationHint other) {
            if (other == null) {
                return this;
            }
            return new AnimationHint(
                aileron || other.aileron,
                elevator || other.elevator,
                rudder || other.rudder,
                flap || other.flap,
                slat || other.slat,
                spoiler || other.spoiler
            );
        }

        boolean anySurfaceControl() {
            return aileron || elevator || rudder || flap || slat || spoiler;
        }

        String compact() {
            return (aileron ? "A" : "")
                + (elevator ? "E" : "")
                + (rudder ? "R" : "")
                + (flap ? "F" : "")
                + (slat ? "S" : "")
                + (spoiler ? "P" : "");
        }
    }

    record RawTriangle(
        Vec3d first,
        Vec3d second,
        Vec3d third,
        Vec3d centroid,
        Vec3d normal,
        double area,
        String objectName,
        Bounds objectBounds
    ) {
    }

    record PanelSummary(Vec3d centroid, double area) {
    }

    record TrianglePair(Vec3d left, Vec3d right) {
    }

    record SectionBand(double minimumRadius, double maximumRadius) {
    }

    record LiftingSummary(Vec3d centroid, double weight) {
    }

    record LiftingFrame(
        Vec3d spanLocal,
        Vec3d chordLocal,
        Vec3d normalLocal,
        double confidence
    ) {
    }

    record NormalFit(Vec3d normalLocal, double coherence) {
    }

    record IncidenceSample(double radius, double incidenceDegrees, double weight) {
    }

    record IncidenceTrend(double interceptDegrees, double slopeDegreesPerMeter, boolean valid) {
        static final IncidenceTrend INVALID = new IncidenceTrend(0.0, 0.0, false);

        double incidenceDegrees(double radius) {
            return interceptDegrees + slopeDegreesPerMeter * radius;
        }
    }

    record WeightedScalar(double value, double weight) {
    }

    record ClassifiedTriangle(
        Vec3d first,
        Vec3d second,
        Vec3d third,
        Vec3d centroid,
        Vec3d normal,
        double area,
        String objectName,
        SurfaceKind kind
    ) {
    }

    static final class BoundsAccumulator {
        double minX = Double.POSITIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double minZ = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;
        double maxZ = Double.NEGATIVE_INFINITY;

        void include(Vec3d point) {
            if (point == null || !point.isFinite()) {
                return;
            }
            minX = Math.min(minX, point.x());
            minY = Math.min(minY, point.y());
            minZ = Math.min(minZ, point.z());
            maxX = Math.max(maxX, point.x());
            maxY = Math.max(maxY, point.y());
            maxZ = Math.max(maxZ, point.z());
        }

        void include(Bounds bounds) {
            if (bounds == null || !bounds.valid()) {
                return;
            }
            include(bounds.minimum());
            include(bounds.maximum());
        }

        Bounds finishLoose() {
            boolean valid = Double.isFinite(minX) && Double.isFinite(maxX)
                && (maxX > minX || maxY > minY || maxZ > minZ);
            return new Bounds(
                valid ? new Vec3d(minX, minY, minZ) : Vec3d.ZERO,
                valid ? new Vec3d(maxX, maxY, maxZ) : Vec3d.ZERO,
                valid
            );
        }

        Bounds finish() {
            boolean valid = Double.isFinite(minX) && Double.isFinite(maxX)
                && maxX > minX && maxY > minY && maxZ > minZ;
            return new Bounds(
                valid ? new Vec3d(minX, minY, minZ) : Vec3d.ZERO,
                valid ? new Vec3d(maxX, maxY, maxZ) : Vec3d.ZERO,
                valid
            );
        }
    }
}
