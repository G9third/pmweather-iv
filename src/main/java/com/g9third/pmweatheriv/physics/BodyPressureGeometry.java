package com.g9third.pmweatheriv.physics;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.List;

import static com.g9third.pmweatheriv.physics.ModelGeometryData.Axis;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.BodyWettedAreaEstimate;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.Bounds;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.BoundsAccumulator;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.ClassifiedTriangle;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.EPSILON;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.LiftingSummary;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.PanelSummary;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.PressurePatch;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.SurfaceKind;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.SymmetryRole;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.TrianglePair;
import static com.g9third.pmweatheriv.physics.LiftingPatchGeometry.summarizeLifting;

/** Body bounds, exposed wetted area, and pressure-patch construction. */
final class BodyPressureGeometry {
    private BodyPressureGeometry() {}

    /**
     * Estimates the exposed non-lifting wetted area without summing every OBJ
     * triangle as independent skin. Detailed content packs frequently contain
     * overlapping shells, internal fairings, nested nacelle details, and other
     * geometry that is valid to render but is not additional exposed airflow
     * surface. Counting all of those triangles made clean skin-friction drag grow
     * with model-detail density rather than with the aircraft's exterior shape.
     *
     * <p>The model is rescanned only during frozen model preparation. For a
     * deterministic set of nearly uniform viewing directions, all body-pressure
     * triangles are projected into a union silhouette. Cauchy's surface-area
     * relation (surface area = four times mean projected area) then turns those
     * exterior silhouettes into one effective wetted area. Projection unions make
     * overlapping/internal triangles idempotent while detached nacelles, belly
     * shapes, and real exposed appendages still enlarge the silhouette where they
     * are externally visible. No per-tick geometry work is added.</p>
     */
    static BodyWettedAreaEstimate estimateBodyWettedArea(
        EnumMap<SurfaceKind, List<ClassifiedTriangle>> byKind
    ) {
        List<ClassifiedTriangle> pressureTriangles = byKind == null
            ? List.of()
            : byKind.values().stream()
                .flatMap(List::stream)
                .filter(triangle -> triangle.kind().isBodyPressure())
                .toList();
        if (pressureTriangles.isEmpty()) {
            return new BodyWettedAreaEstimate(0.0, 0.0, 0, 0, 0);
        }

        BoundsAccumulator pressureBoundsAccumulator = new BoundsAccumulator();
        double rawTriangleArea = 0.0;
        for (ClassifiedTriangle triangle : pressureTriangles) {
            pressureBoundsAccumulator.include(triangle.first());
            pressureBoundsAccumulator.include(triangle.second());
            pressureBoundsAccumulator.include(triangle.third());
            rawTriangleArea += triangle.area();
        }
        Bounds pressureBounds = pressureBoundsAccumulator.finishLoose();
        if (!pressureBounds.valid()) {
            return new BodyWettedAreaEstimate(0.0, 0.0, 0, 0, 0);
        }

        // Consume the repaired classification map.  A false tail/fairing cluster
        // demoted to BODY by the hinge-coherence pass therefore contributes to
        // body wetted area exactly once rather than disappearing from both lift
        // and pressure/profile drag.
        int pressureTriangleCount = pressureTriangles.size();
        int projectionResolution = Math.max(
            96,
            Math.min(192, (int) Math.ceil(Math.sqrt(pressureTriangleCount) * 0.85))
        );
        List<Vec3d> directions = wettedAreaProjectionDirections(18);
        List<ProjectionRaster> rasters = new ArrayList<>(directions.size());
        for (Vec3d direction : directions) {
            rasters.add(new ProjectionRaster(direction, pressureBounds, projectionResolution));
        }
        for (ClassifiedTriangle triangle : pressureTriangles) {
            for (ProjectionRaster raster : rasters) {
                raster.addTriangle(triangle.first(), triangle.second(), triangle.third());
            }
        }

        double projectedAreaSum = 0.0;
        int finiteProjectionCount = 0;
        for (ProjectionRaster raster : rasters) {
            double projectedArea = raster.projectedArea();
            if (Double.isFinite(projectedArea) && projectedArea > EPSILON) {
                projectedAreaSum += projectedArea;
                ++finiteProjectionCount;
            }
        }
        double exposedWettedArea = finiteProjectionCount > 0
            ? 4.0 * projectedAreaSum / finiteProjectionCount
            : rawTriangleArea;
        if (!Double.isFinite(exposedWettedArea) || exposedWettedArea <= EPSILON) {
            exposedWettedArea = rawTriangleArea;
        }
        return new BodyWettedAreaEstimate(
            rawTriangleArea, exposedWettedArea, pressureTriangleCount,
            finiteProjectionCount, projectionResolution
        );
    }

    static List<Vec3d> wettedAreaProjectionDirections(int count) {
        int samples = Math.max(6, count);
        List<Vec3d> directions = new ArrayList<>(samples);
        double goldenAngle = Math.PI * (3.0 - Math.sqrt(5.0));
        for (int index = 0; index < samples; ++index) {
            double y = 1.0 - 2.0 * (index + 0.5) / samples;
            double radius = Math.sqrt(Math.max(0.0, 1.0 - y * y));
            double azimuth = goldenAngle * index;
            directions.add(new Vec3d(
                Math.cos(azimuth) * radius,
                y,
                Math.sin(azimuth) * radius
            ).normalized());
        }
        return List.copyOf(directions);
    }

    static final class ProjectionRaster {
        final Vec3d basisU;
        final Vec3d basisV;
        final int resolution;
        final double minimumU;
        final double minimumV;
        final double cellU;
        final double cellV;
        final boolean[] occupied;

        ProjectionRaster(Vec3d direction, Bounds bounds, int resolution) {
            Vec3d view = direction.normalized();
            Vec3d reference = Math.abs(view.y()) < 0.88
                ? new Vec3d(0.0, 1.0, 0.0)
                : new Vec3d(1.0, 0.0, 0.0);
            this.basisU = view.cross(reference).normalized();
            this.basisV = view.cross(basisU).normalized();
            this.resolution = Math.max(8, resolution);

            double minU = Double.POSITIVE_INFINITY;
            double maxU = Double.NEGATIVE_INFINITY;
            double minV = Double.POSITIVE_INFINITY;
            double maxV = Double.NEGATIVE_INFINITY;
            double[] xs = {bounds.minimum().x(), bounds.maximum().x()};
            double[] ys = {bounds.minimum().y(), bounds.maximum().y()};
            double[] zs = {bounds.minimum().z(), bounds.maximum().z()};
            for (double x : xs) {
                for (double y : ys) {
                    for (double z : zs) {
                        Vec3d corner = new Vec3d(x, y, z);
                        double u = corner.dot(basisU);
                        double v = corner.dot(basisV);
                        minU = Math.min(minU, u);
                        maxU = Math.max(maxU, u);
                        minV = Math.min(minV, v);
                        maxV = Math.max(maxV, v);
                    }
                }
            }
            double spanU = Math.max(1.0E-6, maxU - minU);
            double spanV = Math.max(1.0E-6, maxV - minV);
            double padU = spanU / this.resolution;
            double padV = spanV / this.resolution;
            this.minimumU = minU - padU;
            this.minimumV = minV - padV;
            this.cellU = (spanU + 2.0 * padU) / this.resolution;
            this.cellV = (spanV + 2.0 * padV) / this.resolution;
            this.occupied = new boolean[this.resolution * this.resolution];
        }

        void addTriangle(Vec3d first, Vec3d second, Vec3d third) {
            double u1 = first.dot(basisU);
            double v1 = first.dot(basisV);
            double u2 = second.dot(basisU);
            double v2 = second.dot(basisV);
            double u3 = third.dot(basisU);
            double v3 = third.dot(basisV);
            double projectedTwiceArea = Math.abs(
                (u2 - u1) * (v3 - v1) - (v2 - v1) * (u3 - u1)
            );
            if (!(projectedTwiceArea > 1.0E-12) || !Double.isFinite(projectedTwiceArea)) {
                return;
            }

            int minX = cellIndex(Math.min(u1, Math.min(u2, u3)), minimumU, cellU);
            int maxX = cellIndex(Math.max(u1, Math.max(u2, u3)), minimumU, cellU);
            int minY = cellIndex(Math.min(v1, Math.min(v2, v3)), minimumV, cellV);
            int maxY = cellIndex(Math.max(v1, Math.max(v2, v3)), minimumV, cellV);
            minX = Math.max(0, Math.min(resolution - 1, minX));
            maxX = Math.max(0, Math.min(resolution - 1, maxX));
            minY = Math.max(0, Math.min(resolution - 1, minY));
            maxY = Math.max(0, Math.min(resolution - 1, maxY));

            boolean marked = false;
            for (int y = minY; y <= maxY; ++y) {
                double sampleV = minimumV + (y + 0.5) * cellV;
                for (int x = minX; x <= maxX; ++x) {
                    double sampleU = minimumU + (x + 0.5) * cellU;
                    if (pointInProjectedTriangle(
                        sampleU, sampleV, u1, v1, u2, v2, u3, v3
                    )) {
                        occupied[y * resolution + x] = true;
                        marked = true;
                    }
                }
            }

            // Sub-cell triangles can miss every cell center. Mark their projected
            // centroid so fine exterior detail contributes without requiring a
            // model-detail-dependent global resolution.
            if (!marked) {
                int x = cellIndex((u1 + u2 + u3) / 3.0, minimumU, cellU);
                int y = cellIndex((v1 + v2 + v3) / 3.0, minimumV, cellV);
                if (x >= 0 && x < resolution && y >= 0 && y < resolution) {
                    occupied[y * resolution + x] = true;
                }
            }
        }

        double projectedArea() {
            int occupiedCells = 0;
            for (boolean cell : occupied) {
                if (cell) {
                    ++occupiedCells;
                }
            }
            return occupiedCells * cellU * cellV;
        }

        static int cellIndex(double coordinate, double minimum, double cellSize) {
            return (int) Math.floor((coordinate - minimum) / Math.max(1.0E-12, cellSize));
        }

        static boolean pointInProjectedTriangle(
            double px, double py,
            double ax, double ay,
            double bx, double by,
            double cx, double cy
        ) {
            double first = projectedEdgeSign(px, py, ax, ay, bx, by);
            double second = projectedEdgeSign(px, py, bx, by, cx, cy);
            double third = projectedEdgeSign(px, py, cx, cy, ax, ay);
            boolean negative = first < -1.0E-12 || second < -1.0E-12 || third < -1.0E-12;
            boolean positive = first > 1.0E-12 || second > 1.0E-12 || third > 1.0E-12;
            return !(negative && positive);
        }

        static double projectedEdgeSign(
            double px, double py,
            double ax, double ay,
            double bx, double by
        ) {
            return (px - bx) * (ay - by) - (ax - bx) * (py - by);
        }
    }

    static Bounds robustBodyBounds(
        EnumMap<SurfaceKind, List<ClassifiedTriangle>> byKind,
        Bounds fullBounds
    ) {
        List<ClassifiedTriangle> body = new ArrayList<>();
        for (SurfaceKind kind : SurfaceKind.values()) {
            if (kind.isBodyPressure()) {
                body.addAll(byKind.get(kind));
            }
        }
        if (body.isEmpty()) {
            return fullBounds;
        }

        double rawMinX = weightedQuantile(body, 0, 0.10);
        double rawMaxX = weightedQuantile(body, 0, 0.90);
        double minY = weightedQuantile(body, 1, 0.03);
        double maxY = weightedQuantile(body, 1, 0.97);
        double minZ = weightedQuantile(body, 2, 0.02);
        double maxZ = weightedQuantile(body, 2, 0.98);

        if (!(rawMaxX > rawMinX && maxY > minY && maxZ > minZ)) {
            return fullBounds;
        }

        // Aircraft content packs are normally authored around a longitudinal
        // symmetry plane.  A quantile-only X center can drift toward a door,
        // engine, hoist, or other one-sided detail.  Use the complete model's
        // lateral midpoint as the candidate plane and expand the body bounds
        // equally around it.  This is geometric rather than pack/name specific.
        double symmetryPlaneX = 0.5 * (fullBounds.minimum().x() + fullBounds.maximum().x());
        double minimumWidth = Math.max(0.35, Math.min(fullBounds.spanX(), fullBounds.spanY() * 0.22));
        double minimumHeight = Math.max(0.35, fullBounds.spanY() * 0.45);
        double minimumLength = Math.max(0.75, fullBounds.spanZ() * 0.65);
        double halfX = Math.max(
            Math.max(symmetryPlaneX - rawMinX, rawMaxX - symmetryPlaneX),
            0.5 * minimumWidth
        );
        halfX = Math.min(halfX, Math.max(0.5 * minimumWidth, 0.5 * fullBounds.spanX()));
        double centerY = 0.5 * (minY + maxY);
        double centerZ = 0.5 * (minZ + maxZ);
        double halfY = Math.max(0.5 * (maxY - minY), 0.5 * minimumHeight);
        double halfZ = Math.max(0.5 * (maxZ - minZ), 0.5 * minimumLength);

        return new Bounds(
            new Vec3d(symmetryPlaneX - halfX, centerY - halfY, centerZ - halfZ),
            new Vec3d(symmetryPlaneX + halfX, centerY + halfY, centerZ + halfZ),
            true
        );
    }

    static double weightedQuantile(
        List<ClassifiedTriangle> triangles,
        int axis,
        double quantile
    ) {
        List<ClassifiedTriangle> sorted = new ArrayList<>(triangles);
        sorted.sort(Comparator.comparingDouble(triangle -> coordinate(triangle.centroid(), axis)));
        double total = 0.0;
        for (ClassifiedTriangle triangle : sorted) {
            total += Math.max(EPSILON, triangle.area());
        }
        double target = Vec3d.clamp(quantile, 0.0, 1.0) * total;
        double cumulative = 0.0;
        for (ClassifiedTriangle triangle : sorted) {
            cumulative += Math.max(EPSILON, triangle.area());
            if (cumulative >= target) {
                return coordinate(triangle.centroid(), axis);
            }
        }
        return coordinate(sorted.get(sorted.size() - 1).centroid(), axis);
    }

    static double coordinate(Vec3d point, int axis) {
        return switch (axis) {
            case 0 -> point.x();
            case 1 -> point.y();
            default -> point.z();
        };
    }

    static Vec3d outwardNormal(Vec3d normal, Vec3d centroid, Vec3d bodyCenter) {
        Vec3d candidate = normal.normalized();
        Vec3d outward = centroid.subtract(bodyCenter);
        if (candidate.lengthSquared() <= EPSILON) {
            return outward.normalized();
        }
        if (outward.lengthSquared() > EPSILON && candidate.dot(outward) < 0.0) {
            candidate = candidate.negate();
        }
        return candidate;
    }

    /**
     * Builds a compact, surface-anchored component map.  Every application
     * point is snapped to the centroid of an actual retained model triangle.
     * The model is divided only into a few longitudinal body stations, while
     * separately classified wings and tails remain in the lifting map.
     *
     * No content-pack-specific aircraft names are used here.  Optional object
     * names only contribute to the earlier generic surface classification.
     */
    static List<PressurePatch> buildPressurePatches(
        EnumMap<SurfaceKind, List<ClassifiedTriangle>> byKind,
        Bounds fullBounds,
        Bounds bodyBounds,
        int ceiling,
        boolean rotorcraft
    ) {
        if (!bodyBounds.valid()) {
            return List.of();
        }
        int limit = Math.max(8, Math.min(14, ceiling));
        double symmetryPlaneX = 0.5 * (fullBounds.minimum().x() + fullBounds.maximum().x());
        Vec3d bodyCenter = new Vec3d(
            symmetryPlaneX,
            0.5 * (bodyBounds.minimum().y() + bodyBounds.maximum().y()),
            0.5 * (bodyBounds.minimum().z() + bodyBounds.maximum().z())
        );

        // Include every non-lifting exterior kind.  This lets nacelles, fairings,
        // and other substantial exposed geometry contribute to wind pressure,
        // while the robust bounds still prevent a small appendage from moving the
        // aircraft's symmetry plane.
        List<ClassifiedTriangle> pressureShell = new ArrayList<>();
        for (SurfaceKind kind : SurfaceKind.values()) {
            if (kind.isBodyPressure()) {
                pressureShell.addAll(orientedTriangles(byKind.get(kind), bodyCenter));
            }
        }
        if (pressureShell.isEmpty()) {
            return List.of();
        }

        List<ClassifiedTriangle> rotorAppendages = rotorcraft
            ? rotorHorizontalAppendageTriangles(pressureShell, bodyBounds, symmetryPlaneX)
            : List.of();
        if (!rotorAppendages.isEmpty()) {
            pressureShell.removeAll(rotorAppendages);
        }

        List<PressurePatch> patches = new ArrayList<>(limit);
        int pairId = 0;

        // Three side stations preserve nose/mid/tail gradients without returning
        // to the noisy 32-point map.  Two upper/lower stations are enough to
        // produce pitch moments from vertical wind differences.
        pairId = addSideSections(
            patches, pressureShell, bodyBounds, symmetryPlaneX,
            "FUSELAGE", 3, pairId, limit
        );
        addTopBottomSections(
            patches, pressureShell, bodyBounds, symmetryPlaneX, "FUSELAGE", 2, limit
        );
        addEndCaps(
            patches, pressureShell, bodyBounds, symmetryPlaneX, "FUSELAGE", limit
        );

        // Rotorcraft do not run fixed-wing lifting surfaces.  Preserve an actual
        // far-aft horizontal stabilizer as one two-sided left/right pair when the
        // mesh genuinely contains it.  No synthetic points are emitted.
        if (rotorcraft && patches.size() + 2 <= limit) {
            addRotorHorizontalAppendages(
                patches, rotorAppendages, bodyBounds, symmetryPlaneX, limit, pairId
            );
        }

        List<PressurePatch> anchored=new ArrayList<>();
        List<ClassifiedTriangle> sampleMesh=new ArrayList<>(pressureShell);
        sampleMesh.addAll(rotorAppendages);
        for (PressurePatch patch:calibratePressureAreas(patches,bodyBounds)) {
            Vec3d nearest=patch.pointLocal();
            double distance=Double.POSITIVE_INFINITY;
            for (ClassifiedTriangle triangle:sampleMesh) {
                Vec3d candidate=ModelCoordinates.closestTrianglePoint(patch.pointLocal(),
                    triangle.first(),triangle.second(),triangle.third());
                double squared=candidate.subtract(patch.pointLocal()).lengthSquared();
                if (squared<distance) { distance=squared; nearest=candidate; }
            }
            anchored.add(new PressurePatch(patch.name(),patch.pointLocal(),patch.normalLocal(),patch.area(),
                patch.kind(),patch.twoSided(),patch.symmetryPair(),patch.symmetryRole(),nearest));
        }
        return List.copyOf(anchored);
    }

    static List<ClassifiedTriangle> orientedTriangles(
        List<ClassifiedTriangle> source,
        Vec3d bodyCenter
    ) {
        List<ClassifiedTriangle> result = new ArrayList<>();
        if (source == null) {
            return result;
        }
        for (ClassifiedTriangle triangle : source) {
            result.add(new ClassifiedTriangle(
                triangle.first(), triangle.second(), triangle.third(), triangle.centroid(),
                outwardNormal(triangle.normal(), triangle.centroid(), bodyCenter),
                triangle.area(), triangle.objectName(), triangle.kind()
            ));
        }
        return result;
    }

    static List<ClassifiedTriangle> rotorHorizontalAppendageTriangles(
        List<ClassifiedTriangle> source,
        Bounds bodyBounds,
        double symmetryPlaneX
    ) {
        if (source == null || source.isEmpty() || !bodyBounds.valid()) {
            return List.of();
        }
        double aftCutoff = bodyBounds.minimum().z() + bodyBounds.spanZ() * 0.46;
        double minimumY = bodyBounds.minimum().y() + bodyBounds.spanY() * 0.24;
        double outsideThreshold = Math.max(0.12, bodyBounds.spanX() * 0.30);
        List<ClassifiedTriangle> candidates = new ArrayList<>();
        for (ClassifiedTriangle triangle : source) {
            Vec3d normal = triangle.normal().normalized();
            if (triangle.centroid().z() > aftCutoff
                || triangle.centroid().y() < minimumY
                || Math.abs(triangle.centroid().x() - symmetryPlaneX) < outsideThreshold
                || Math.abs(normal.y()) < 0.58) {
                continue;
            }
            candidates.add(triangle);
        }
        double leftArea = projectedArea(candidates, new Vec3d(0.0, 1.0, 0.0), symmetryPlaneX, -1);
        double rightArea = projectedArea(candidates, new Vec3d(0.0, 1.0, 0.0), symmetryPlaneX, 1);
        double minimumCredibleArea = Math.max(0.015, bodyBounds.spanX() * bodyBounds.spanZ() * 0.0008);
        if (leftArea < minimumCredibleArea || rightArea < minimumCredibleArea) {
            return List.of();
        }
        double ratio = Math.max(leftArea, rightArea) / Math.max(EPSILON, Math.min(leftArea, rightArea));
        return ratio <= 3.5 ? candidates : List.of();
    }

    static double projectedArea(
        List<ClassifiedTriangle> source,
        Vec3d axis,
        double symmetryPlaneX,
        int side
    ) {
        double area = 0.0;
        if (source == null) {
            return area;
        }
        for (ClassifiedTriangle triangle : source) {
            double offset = triangle.centroid().x() - symmetryPlaneX;
            if ((side < 0 && offset >= 0.0) || (side > 0 && offset <= 0.0)) {
                continue;
            }
            area += triangle.area() * Math.abs(triangle.normal().normalized().dot(axis)) * 0.5;
        }
        return area;
    }

    static int addSideSections(
        List<PressurePatch> output,
        List<ClassifiedTriangle> source,
        Bounds bounds,
        double symmetryPlaneX,
        String component,
        int sectionCount,
        int pairId,
        int limit
    ) {
        if (!bounds.valid()) {
            return pairId;
        }
        double segmentLength = bounds.spanZ() / Math.max(1, sectionCount);
        for (int section = 0; section < sectionCount && output.size() < limit; ++section) {
            double minimumZ = bounds.minimum().z() + segmentLength * section;
            double maximumZ = section == sectionCount - 1
                ? bounds.maximum().z()
                : minimumZ + segmentLength;
            PanelSummary left = summarizePanel(source, new Vec3d(-1.0, 0.0, 0.0), minimumZ, maximumZ);
            PanelSummary right = summarizePanel(source, new Vec3d(1.0, 0.0, 0.0), minimumZ, maximumZ);
            String sectionName = component + "_" + switch (section) {
                case 0 -> "AFT";
                case 1 -> sectionCount == 2 ? "FRONT" : "MID";
                default -> "FRONT";
            };

            if (left.area() > EPSILON && right.area() > EPSILON && output.size() + 2 <= limit) {
                double area = 0.5 * (left.area() + right.area());
                double y = weightedAverage(
                    left.centroid().y(), left.area(), right.centroid().y(), right.area(),
                    0.5 * (bounds.minimum().y() + bounds.maximum().y())
                );
                double z = weightedAverage(
                    left.centroid().z(), left.area(), right.centroid().z(), right.area(),
                    0.5 * (minimumZ + maximumZ)
                );
                double radius = (
                    Math.abs(left.centroid().x() - symmetryPlaneX) * left.area()
                        + Math.abs(right.centroid().x() - symmetryPlaneX) * right.area()
                ) / Math.max(EPSILON, left.area() + right.area());
                Vec3d leftTarget = new Vec3d(symmetryPlaneX - radius, y, z);
                Vec3d rightTarget = new Vec3d(symmetryPlaneX + radius, y, z);
                TrianglePair pair = nearestMirroredPanelPair(
                    source, minimumZ, maximumZ, symmetryPlaneX,
                    leftTarget, rightTarget, bounds
                );
                Vec3d leftPoint = pair.left();
                Vec3d rightPoint = pair.right();
                output.add(new PressurePatch(
                    sectionName + "_SIDE_L", leftPoint,
                    new Vec3d(-1.0, 0.0, 0.0), area, SurfaceKind.BODY,
                    pairId, SymmetryRole.MIRRORED_LEFT
                ));
                output.add(new PressurePatch(
                    sectionName + "_SIDE_R", rightPoint,
                    new Vec3d(1.0, 0.0, 0.0), area, SurfaceKind.BODY,
                    pairId, SymmetryRole.MIRRORED_RIGHT
                ));
                ++pairId;
            }
            // A missing side in a coarse retained mesh is treated as insufficient
            // evidence, not as a permanent asymmetric fuselage. Genuine asymmetric
            // lifting components are handled separately with a much stronger test.
        }
        return pairId;
    }

    static void addTopBottomSections(
        List<PressurePatch> output,
        List<ClassifiedTriangle> source,
        Bounds bounds,
        double symmetryPlaneX,
        String component,
        int sectionCount,
        int limit
    ) {
        if (!bounds.valid()) {
            return;
        }
        double segmentLength = bounds.spanZ() / Math.max(1, sectionCount);
        for (int section = 0; section < sectionCount && output.size() < limit; ++section) {
            double minimumZ = bounds.minimum().z() + segmentLength * section;
            double maximumZ = section == sectionCount - 1
                ? bounds.maximum().z()
                : minimumZ + segmentLength;
            PanelSummary bottom = summarizePanel(source, new Vec3d(0.0, -1.0, 0.0), minimumZ, maximumZ);
            PanelSummary top = summarizePanel(source, new Vec3d(0.0, 1.0, 0.0), minimumZ, maximumZ);
            String sectionName = component + (section == 0 ? "_AFT" : "_FRONT");
            if (bottom.area() > EPSILON && output.size() < limit) {
                Vec3d point = centerlinePanelPoint(
                    source, new Vec3d(0.0, -1.0, 0.0), minimumZ, maximumZ,
                    new Vec3d(symmetryPlaneX, bottom.centroid().y(), bottom.centroid().z()),
                    symmetryPlaneX, bounds
                );
                if (point != null) {
                    output.add(new PressurePatch(
                        sectionName + "_BOTTOM", point,
                        new Vec3d(0.0, -1.0, 0.0), bottom.area(), SurfaceKind.BODY,
                        -1, SymmetryRole.CENTERLINE
                    ));
                }
            }
            if (top.area() > EPSILON && output.size() < limit) {
                Vec3d point = centerlinePanelPoint(
                    source, new Vec3d(0.0, 1.0, 0.0), minimumZ, maximumZ,
                    new Vec3d(symmetryPlaneX, top.centroid().y(), top.centroid().z()),
                    symmetryPlaneX, bounds
                );
                if (point != null) {
                    output.add(new PressurePatch(
                        sectionName + "_TOP", point,
                        new Vec3d(0.0, 1.0, 0.0), top.area(), SurfaceKind.BODY,
                        -1, SymmetryRole.CENTERLINE
                    ));
                }
            }
        }
    }

    static void addEndCaps(
        List<PressurePatch> output,
        List<ClassifiedTriangle> source,
        Bounds bounds,
        double symmetryPlaneX,
        String component,
        int limit
    ) {
        if (!bounds.valid() || output.size() >= limit) {
            return;
        }
        double depth = Math.max(0.05, bounds.spanZ() * 0.18);
        double rearMin = bounds.minimum().z();
        double rearMax = bounds.minimum().z() + depth;
        double frontMin = bounds.maximum().z() - depth;
        double frontMax = bounds.maximum().z();
        PanelSummary rear = summarizePanel(source, new Vec3d(0.0, 0.0, -1.0), rearMin, rearMax);
        PanelSummary front = summarizePanel(source, new Vec3d(0.0, 0.0, 1.0), frontMin, frontMax);
        double minimumCapArea = Math.max(0.02, bounds.spanX() * bounds.spanY() * 0.02);
        if (rear.area() >= minimumCapArea && output.size() < limit) {
            Vec3d point = centerlinePanelPoint(
                source, new Vec3d(0.0, 0.0, -1.0), rearMin, rearMax,
                new Vec3d(symmetryPlaneX, rear.centroid().y(), rear.centroid().z()),
                symmetryPlaneX, bounds
            );
            if (point != null) {
                output.add(new PressurePatch(
                    component + "_REAR", point,
                    new Vec3d(0.0, 0.0, -1.0), rear.area(), SurfaceKind.BODY,
                    -1, SymmetryRole.CENTERLINE
                ));
            }
        }
        if (front.area() >= minimumCapArea && output.size() < limit) {
            Vec3d point = centerlinePanelPoint(
                source, new Vec3d(0.0, 0.0, 1.0), frontMin, frontMax,
                new Vec3d(symmetryPlaneX, front.centroid().y(), front.centroid().z()),
                symmetryPlaneX, bounds
            );
            if (point != null) {
                output.add(new PressurePatch(
                    component + "_FRONT", point,
                    new Vec3d(0.0, 0.0, 1.0), front.area(), SurfaceKind.BODY,
                    -1, SymmetryRole.CENTERLINE
                ));
            }
        }
    }

    static void addRotorHorizontalAppendages(
        List<PressurePatch> output,
        List<ClassifiedTriangle> source,
        Bounds bodyBounds,
        double symmetryPlaneX,
        int limit,
        int pairId
    ) {
        if (source == null || source.isEmpty() || output.size() + 2 > limit) {
            return;
        }
        double tolerance = Math.max(0.04, bodyBounds.spanX() * 0.02);
        LiftingSummary left = summarizeLifting(source, Axis.VERTICAL, symmetryPlaneX, -1, tolerance);
        LiftingSummary right = summarizeLifting(source, Axis.VERTICAL, symmetryPlaneX, 1, tolerance);
        if (left.weight() <= EPSILON || right.weight() <= EPSILON) {
            return;
        }
        double area = 0.25 * (left.weight() + right.weight());
        double y = weightedAverage(left.centroid().y(), left.weight(), right.centroid().y(), right.weight(),
            0.5 * (bodyBounds.minimum().y() + bodyBounds.maximum().y()));
        double z = weightedAverage(left.centroid().z(), left.weight(), right.centroid().z(), right.weight(),
            bodyBounds.minimum().z() + bodyBounds.spanZ() * 0.18);
        double radius = (
            Math.abs(left.centroid().x() - symmetryPlaneX) * left.weight()
                + Math.abs(right.centroid().x() - symmetryPlaneX) * right.weight()
        ) / Math.max(EPSILON, left.weight() + right.weight());
        TrianglePair pair = nearestMirroredLiftingPair(
            source, Axis.VERTICAL, symmetryPlaneX, tolerance,
            new Vec3d(symmetryPlaneX - radius, y, z),
            new Vec3d(symmetryPlaneX + radius, y, z), bodyBounds
        );
        Vec3d leftPoint = pair.left();
        Vec3d rightPoint = pair.right();
        output.add(new PressurePatch(
            "ROTOR_STABILIZER_L", leftPoint,
            new Vec3d(0.0, 1.0, 0.0), area, SurfaceKind.HORIZONTAL_TAIL,
            true, pairId, SymmetryRole.MIRRORED_LEFT
        ));
        output.add(new PressurePatch(
            "ROTOR_STABILIZER_R", rightPoint,
            new Vec3d(0.0, 1.0, 0.0), area, SurfaceKind.HORIZONTAL_TAIL,
            true, pairId, SymmetryRole.MIRRORED_RIGHT
        ));
    }

    static PanelSummary summarizePanel(
        List<ClassifiedTriangle> source,
        Vec3d panelNormal,
        double minimumZ,
        double maximumZ
    ) {
        Vec3d weightedPoint = Vec3d.ZERO;
        double projectedArea = 0.0;
        if (source != null) {
            Vec3d axis = panelNormal.normalized();
            for (ClassifiedTriangle triangle : source) {
                if (triangle.centroid().z() < minimumZ || triangle.centroid().z() > maximumZ) {
                    continue;
                }
                double projection = Math.max(0.0, triangle.normal().normalized().dot(axis));
                if (projection < 0.18) {
                    continue;
                }
                double weight = triangle.area() * projection;
                projectedArea += weight;
                weightedPoint = weightedPoint.add(triangle.centroid().scale(weight));
            }
        }
        if (projectedArea <= EPSILON) {
            return new PanelSummary(Vec3d.ZERO, 0.0);
        }
        return new PanelSummary(weightedPoint.scale(1.0 / projectedArea), projectedArea);
    }

    static TrianglePair nearestMirroredPanelPair(
        List<ClassifiedTriangle> source,
        double minimumZ,
        double maximumZ,
        double symmetryPlaneX,
        Vec3d leftTarget,
        Vec3d rightTarget,
        Bounds scaleBounds
    ) {
        List<ClassifiedTriangle> left = panelCandidates(
            source, new Vec3d(-1.0, 0.0, 0.0), minimumZ, maximumZ, leftTarget, scaleBounds
        );
        List<ClassifiedTriangle> right = panelCandidates(
            source, new Vec3d(1.0, 0.0, 0.0), minimumZ, maximumZ, rightTarget, scaleBounds
        );
        return bestMirroredPair(left, right, symmetryPlaneX, leftTarget, rightTarget, scaleBounds);
    }

    static List<ClassifiedTriangle> panelCandidates(
        List<ClassifiedTriangle> source,
        Vec3d panelNormal,
        double minimumZ,
        double maximumZ,
        Vec3d target,
        Bounds scaleBounds
    ) {
        List<ClassifiedTriangle> candidates = new ArrayList<>();
        Vec3d axis = panelNormal.normalized();
        if (source != null) {
            for (ClassifiedTriangle triangle : source) {
                if (triangle.centroid().z() < minimumZ || triangle.centroid().z() > maximumZ
                    || triangle.normal().normalized().dot(axis) < 0.18) {
                    continue;
                }
                candidates.add(triangle);
            }
        }
        candidates.sort(Comparator.comparingDouble(triangle ->
            normalizedDistanceSquared(triangle.centroid(), target, scaleBounds)
        ));
        return candidates.size() <= 96
            ? candidates
            : new ArrayList<>(candidates.subList(0, 96));
    }

    static TrianglePair nearestMirroredLiftingPair(
        List<ClassifiedTriangle> source,
        Axis projectedAxis,
        double centerX,
        double tolerance,
        Vec3d leftTarget,
        Vec3d rightTarget,
        Bounds scaleBounds
    ) {
        List<ClassifiedTriangle> left = liftingCandidates(
            source, projectedAxis, centerX, -1, tolerance, leftTarget, scaleBounds
        );
        List<ClassifiedTriangle> right = liftingCandidates(
            source, projectedAxis, centerX, 1, tolerance, rightTarget, scaleBounds
        );
        return bestMirroredPair(left, right, centerX, leftTarget, rightTarget, scaleBounds);
    }

    static List<ClassifiedTriangle> liftingCandidates(
        List<ClassifiedTriangle> source,
        Axis projectedAxis,
        double centerX,
        int side,
        double tolerance,
        Vec3d target,
        Bounds scaleBounds
    ) {
        List<ClassifiedTriangle> candidates = new ArrayList<>();
        if (source != null) {
            for (ClassifiedTriangle triangle : source) {
                double offset = triangle.centroid().x() - centerX;
                if ((side < 0 && offset >= -tolerance) || (side > 0 && offset <= tolerance)) {
                    continue;
                }
                double projection = projectedAxis == Axis.VERTICAL
                    ? Math.abs(triangle.normal().y())
                    : Math.abs(triangle.normal().x());
                if (projection < 0.08) {
                    continue;
                }
                candidates.add(triangle);
            }
        }
        candidates.sort(Comparator.comparingDouble(triangle ->
            normalizedDistanceSquared(triangle.centroid(), target, scaleBounds)
        ));
        return candidates.size() <= 96
            ? candidates
            : new ArrayList<>(candidates.subList(0, 96));
    }

    static TrianglePair bestMirroredPair(
        List<ClassifiedTriangle> left,
        List<ClassifiedTriangle> right,
        double symmetryPlaneX,
        Vec3d leftTarget,
        Vec3d rightTarget,
        Bounds scaleBounds
    ) {
        if (left.isEmpty() || right.isEmpty()) {
            return new TrianglePair(
                left.isEmpty() ? leftTarget : left.get(0).centroid(),
                right.isEmpty() ? rightTarget : right.get(0).centroid()
            );
        }
        ClassifiedTriangle bestLeft = left.get(0);
        ClassifiedTriangle bestRight = right.get(0);
        double bestScore = Double.POSITIVE_INFINITY;
        double sx = Math.max(0.25, scaleBounds.spanX());
        double sy = Math.max(0.25, scaleBounds.spanY());
        double sz = Math.max(0.25, scaleBounds.spanZ());
        for (ClassifiedTriangle leftTriangle : left) {
            Vec3d lp = leftTriangle.centroid();
            for (ClassifiedTriangle rightTriangle : right) {
                Vec3d rp = rightTriangle.centroid();
                double targetScore = normalizedDistanceSquared(lp, leftTarget, scaleBounds)
                    + normalizedDistanceSquared(rp, rightTarget, scaleBounds);
                double planeError = (lp.x() + rp.x() - 2.0 * symmetryPlaneX) / sx;
                double radiusError = (
                    Math.abs(lp.x() - symmetryPlaneX) - Math.abs(rp.x() - symmetryPlaneX)
                ) / sx;
                double yError = (lp.y() - rp.y()) / sy;
                double zError = (lp.z() - rp.z()) / sz;
                double score = targetScore
                    + 18.0 * planeError * planeError
                    + 12.0 * radiusError * radiusError
                    + 15.0 * yError * yError
                    + 15.0 * zError * zError;
                if (score < bestScore) {
                    bestScore = score;
                    bestLeft = leftTriangle;
                    bestRight = rightTriangle;
                }
            }
        }
        Vec3d rawLeft = bestLeft.centroid();
        Vec3d rawRight = bestRight.centroid();
        double radius = 0.5 * (
            Math.abs(rawLeft.x() - symmetryPlaneX)
                + Math.abs(rawRight.x() - symmetryPlaneX)
        );
        double y = 0.5 * (rawLeft.y() + rawRight.y());
        double z = 0.5 * (rawLeft.z() + rawRight.z());
        // The two source anchors are real retained triangles. Their shared
        // aerodynamic application points are canonicalized between them so
        // authored triangulation density cannot create permanent roll/yaw bias.
        return new TrianglePair(
            new Vec3d(symmetryPlaneX - radius, y, z),
            new Vec3d(symmetryPlaneX + radius, y, z)
        );
    }

    static double normalizedDistanceSquared(Vec3d point, Vec3d target, Bounds bounds) {
        double dx = (point.x() - target.x()) / Math.max(0.25, bounds.spanX());
        double dy = (point.y() - target.y()) / Math.max(0.25, bounds.spanY());
        double dz = (point.z() - target.z()) / Math.max(0.25, bounds.spanZ());
        return dx * dx + dy * dy + dz * dz;
    }

    /**
     * Produces an exact centerline aerodynamic point from real panel evidence.
     * A near-center triangle is preferred. If the authored triangulation has no
     * centerline centroid, a jointly selected left/right surface pair supplies
     * the shared Y/Z coordinate. This avoids labeling a visibly one-sided
     * triangle as CENTERLINE while still deriving the point from the model.
     */
    static Vec3d centerlinePanelPoint(
        List<ClassifiedTriangle> source,
        Vec3d panelNormal,
        double minimumZ,
        double maximumZ,
        Vec3d target,
        double symmetryPlaneX,
        Bounds scaleBounds
    ) {
        Vec3d axis = panelNormal.normalized();
        Vec3d best = null;
        double bestScore = Double.POSITIVE_INFINITY;
        if (source != null) {
            for (ClassifiedTriangle triangle : source) {
                if (triangle.centroid().z() < minimumZ || triangle.centroid().z() > maximumZ
                    || triangle.normal().normalized().dot(axis) < 0.18) {
                    continue;
                }
                Vec3d point = closestTrianglePlaneIntersection(
                    triangle, symmetryPlaneX, target, scaleBounds
                );
                if (point == null) {
                    continue;
                }
                double score = normalizedDistanceSquared(point, target, scaleBounds);
                if (score < bestScore) {
                    bestScore = score;
                    best = point;
                }
            }
        }
        return best;
    }

    static Vec3d closestTrianglePlaneIntersection(
        ClassifiedTriangle triangle,
        double planeX,
        Vec3d target,
        Bounds scaleBounds
    ) {
        List<Vec3d> intersections = new ArrayList<>(4);
        addPlaneEdgeIntersections(intersections, triangle.first(), triangle.second(), planeX);
        addPlaneEdgeIntersections(intersections, triangle.second(), triangle.third(), planeX);
        addPlaneEdgeIntersections(intersections, triangle.third(), triangle.first(), planeX);
        if (intersections.isEmpty()) {
            return null;
        }
        Vec3d best = intersections.get(0);
        double bestScore = normalizedDistanceSquared(best, target, scaleBounds);
        for (int i = 1; i < intersections.size(); ++i) {
            Vec3d point = intersections.get(i);
            double score = normalizedDistanceSquared(point, target, scaleBounds);
            if (score < bestScore) {
                best = point;
                bestScore = score;
            }
        }
        if (intersections.size() >= 2) {
            // When the plane cuts an edge-to-edge segment, the closest point on
            // that segment is also on the original triangle and usually gives a
            // better aerodynamic application point than either endpoint.
            for (int i = 0; i < intersections.size(); ++i) {
                for (int j = i + 1; j < intersections.size(); ++j) {
                    Vec3d point = closestPointOnSegment(intersections.get(i), intersections.get(j), target);
                    double score = normalizedDistanceSquared(point, target, scaleBounds);
                    if (score < bestScore) {
                        best = point;
                        bestScore = score;
                    }
                }
            }
        }
        return new Vec3d(planeX, best.y(), best.z());
    }

    static void addPlaneEdgeIntersections(
        List<Vec3d> output,
        Vec3d first,
        Vec3d second,
        double planeX
    ) {
        double firstOffset = first.x() - planeX;
        double secondOffset = second.x() - planeX;
        double tolerance = 1.0E-8;
        if (Math.abs(firstOffset) <= tolerance) {
            addUniquePoint(output, new Vec3d(planeX, first.y(), first.z()));
        }
        if (Math.abs(secondOffset) <= tolerance) {
            addUniquePoint(output, new Vec3d(planeX, second.y(), second.z()));
        }
        if ((firstOffset < -tolerance && secondOffset > tolerance)
            || (firstOffset > tolerance && secondOffset < -tolerance)) {
            double t = -firstOffset / (secondOffset - firstOffset);
            Vec3d point = first.add(second.subtract(first).scale(t));
            addUniquePoint(output, new Vec3d(planeX, point.y(), point.z()));
        }
    }

    static void addUniquePoint(List<Vec3d> output, Vec3d candidate) {
        for (Vec3d existing : output) {
            if (existing.subtract(candidate).lengthSquared() <= 1.0E-14) {
                return;
            }
        }
        output.add(candidate);
    }

    static Vec3d closestPointOnSegment(Vec3d first, Vec3d second, Vec3d target) {
        Vec3d delta = second.subtract(first);
        double lengthSquared = delta.lengthSquared();
        if (lengthSquared <= EPSILON) {
            return first;
        }
        double t = Vec3d.clamp(target.subtract(first).dot(delta) / lengthSquared, 0.0, 1.0);
        return first.add(delta.scale(t));
    }

    static Vec3d nearestLiftingPoint(
        List<ClassifiedTriangle> source,
        Axis projectedAxis,
        double centerX,
        int side,
        double tolerance,
        Vec3d target
    ) {
        ClassifiedTriangle best = null;
        double bestScore = Double.POSITIVE_INFINITY;
        if (source != null) {
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
                double score = triangle.centroid().subtract(target).lengthSquared()
                    / Math.max(0.10, projection * projection);
                if (score < bestScore) {
                    bestScore = score;
                    best = triangle;
                }
            }
        }
        return best == null ? target : best.centroid();
    }

    static Vec3d nearestCenterlineLiftingPoint(
        List<ClassifiedTriangle> source,
        Axis projectedAxis,
        double centerX,
        double tolerance,
        Vec3d target,
        Bounds bounds
    ) {
        ClassifiedTriangle best = null;
        double bestScore = Double.POSITIVE_INFINITY;
        if (source != null) {
            for (ClassifiedTriangle triangle : source) {
                double projection = projectedAxis == Axis.VERTICAL
                    ? Math.abs(triangle.normal().y())
                    : Math.abs(triangle.normal().x());
                if (projection < 0.08) {
                    continue;
                }
                double score = normalizedDistanceSquared(triangle.centroid(), target, bounds)
                    / Math.max(0.10, projection * projection);
                if (score < bestScore) {
                    bestScore = score;
                    best = triangle;
                }
            }
        }
        // A thin center fin has two exterior skins and often no triangle that
        // literally crosses its internal symmetry plane. Project the selected
        // real surface anchor laterally onto that plane while preserving its
        // physically meaningful Y/Z centroid.
        return best == null
            ? null
            : new Vec3d(centerX, best.centroid().y(), best.centroid().z());
    }

    static double weightedAverage(
        double first, double firstWeight,
        double second, double secondWeight,
        double fallback
    ) {
        double weight = Math.max(0.0, firstWeight) + Math.max(0.0, secondWeight);
        if (weight <= EPSILON) {
            return fallback;
        }
        return (first * Math.max(0.0, firstWeight) + second * Math.max(0.0, secondWeight)) / weight;
    }

    static List<PressurePatch> calibratePressureAreas(List<PressurePatch> patches, Bounds bodyBounds) {
        if (patches.isEmpty()) {
            return patches;
        }
        double rawX = 0.0;
        double rawY = 0.0;
        double rawZ = 0.0;
        for (PressurePatch patch : patches) {
            if (patch.twoSided()) {
                continue;
            }
            rawX += patch.area() * patch.normalLocal().x() * patch.normalLocal().x();
            rawY += patch.area() * patch.normalLocal().y() * patch.normalLocal().y();
            rawZ += patch.area() * patch.normalLocal().z() * patch.normalLocal().z();
        }
        // Opposing faces each need the full projected silhouette for wind from
        // their respective side. The calibration sum contains both directions,
        // so its axis target is twice the one-sided projected area.
        double targetX = 2.0 * Math.max(0.25, bodyBounds.spanY() * bodyBounds.spanZ() * 0.82);
        double targetY = 2.0 * Math.max(0.25, bodyBounds.spanX() * bodyBounds.spanZ() * 0.58);
        double targetZ = 2.0 * Math.max(0.20, bodyBounds.spanX() * bodyBounds.spanY() * 0.72);
        double scaleX = Vec3d.clamp(targetX / Math.max(EPSILON, rawX), 0.50, 2.00);
        double scaleY = Vec3d.clamp(targetY / Math.max(EPSILON, rawY), 0.50, 2.00);
        double scaleZ = Vec3d.clamp(targetZ / Math.max(EPSILON, rawZ), 0.50, 2.00);
        List<PressurePatch> calibrated = new ArrayList<>(patches.size());
        for (PressurePatch patch : patches) {
            Vec3d n = patch.normalLocal();
            double weightedScale = n.x() * n.x() * scaleX
                + n.y() * n.y() * scaleY
                + n.z() * n.z() * scaleZ;
            double effectiveArea = patch.twoSided()
                ? Vec3d.clamp(patch.area(), 0.001, 256.0)
                : Vec3d.clamp(patch.area() * weightedScale, 0.001, 256.0);
            calibrated.add(new PressurePatch(
                patch.name(), patch.pointLocal(), n, effectiveArea, patch.kind(), patch.twoSided(),
                patch.symmetryPair(), patch.symmetryRole()
            ));
        }
        return calibrated;
    }
}
