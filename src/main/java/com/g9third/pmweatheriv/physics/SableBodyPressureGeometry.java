package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.sable.SableModelCollisionHull;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Converts a Sable-derived model shell into a compact set of PMAero body-pressure patches.
 *
 * <p>Ground vehicles use the same Sable OBJ parser/raster/merge machinery as aircraft, but the
 * supplied body-pressure shell deliberately differs from the collision shell where physics
 * semantics require it: exterior animated body panels remain represented and collision-only
 * rolling-gear support corridors are not carved from aerodynamic area. Road and aircraft
 * collision shells are attached to their persistent Sable bodies independently.</p>
 *
 * <p>Sable's raster is a surface shell rather than a filled solid, so simply counting every free
 * voxel face would incorrectly count inward faces too. Instead this adapter reconstructs the
 * shell cells and takes the two outermost occupied cells along every X/Y/Z raster ray. That gives
 * the six physical silhouette surfaces seen by axial/cross-flow pressure, then area-preserving
 * aggregation keeps the PMWeather sample count bounded.</p>
 */
public final class SableBodyPressureGeometry {
    private SableBodyPressureGeometry() {
    }

    public record PreparedBody(
        String modelLocation,
        boolean modelBased,
        String reason,
        List<ModelSurfaceMap.PressurePatch> patches,
        double width,
        double height,
        double length,
        double wettedArea,
        Vec3d inertiaPerKgAboutOrigin,
        Vec3d centerOfMassLocal,
        Vec3d inertiaPerKgAboutCenter,
        int shellCells,
        int exteriorSilhouetteFaces,
        int mergedCuboids,
        double resolution
    ) {
    }

    public static PreparedBody prepare(
        SableModelCollisionHull.PreparedHull hull,
        int maximumPatches
    ) {
        if (hull == null || !hull.usable() || hull.boxes() == null || hull.boxes().isEmpty()
            || !Double.isFinite(hull.resolution()) || hull.resolution() <= 0.0) {
            String location = hull == null ? "missing" : hull.modelLocation();
            String reason = hull == null ? "missing-hull" : hull.reason();
            return fallback(location, reason);
        }

        final double r = hull.resolution();
        final List<Cell> cells = new ArrayList<>(Math.max(16, hull.shellCells()));
        int minX = Integer.MAX_VALUE, minY = Integer.MAX_VALUE, minZ = Integer.MAX_VALUE;
        int maxX = Integer.MIN_VALUE, maxY = Integer.MIN_VALUE, maxZ = Integer.MIN_VALUE;
        for (SableModelCollisionHull.HullBox box : hull.boxes()) {
            int x0 = grid(box.cx() - box.hx(), r);
            int x1 = grid(box.cx() + box.hx(), r);
            int y0 = grid(box.cy() - box.hy(), r);
            int y1 = grid(box.cy() + box.hy(), r);
            int z0 = grid(box.cz() - box.hz(), r);
            int z1 = grid(box.cz() + box.hz(), r);
            for (int x = x0; x < x1; ++x) {
                for (int y = y0; y < y1; ++y) {
                    for (int z = z0; z < z1; ++z) {
                        cells.add(new Cell(x, y, z));
                        minX = Math.min(minX, x); maxX = Math.max(maxX, x);
                        minY = Math.min(minY, y); maxY = Math.max(maxY, y);
                        minZ = Math.min(minZ, z); maxZ = Math.max(maxZ, z);
                    }
                }
            }
        }
        if (cells.isEmpty()) {
            return fallback(hull.modelLocation(), "expanded-shell-empty");
        }

        // For each raster ray, record only the first and last occupied shell cell. This avoids
        // treating the inward face of a hollow Sable shell as another aerodynamic exterior face.
        Map<Column, Range> xRays = new HashMap<>(); // key=(y,z), range=x
        Map<Column, Range> yRays = new HashMap<>(); // key=(x,z), range=y
        Map<Column, Range> zRays = new HashMap<>(); // key=(x,y), range=z
        for (Cell cell : cells) {
            extend(xRays, new Column(cell.y(), cell.z()), cell.x());
            extend(yRays, new Column(cell.x(), cell.z()), cell.y());
            extend(zRays, new Column(cell.x(), cell.y()), cell.z());
        }

        final double faceArea = r * r;
        List<Face> faces = new ArrayList<>(2 * (xRays.size() + yRays.size() + zRays.size()));
        for (Map.Entry<Column, Range> entry : xRays.entrySet()) {
            Column column = entry.getKey(); Range range = entry.getValue();
            faces.add(new Face(
                new Vec3d(range.maxPlusOne() * r, (column.a() + 0.5) * r, (column.b() + 0.5) * r),
                new Vec3d(1, 0, 0), faceArea
            ));
            faces.add(new Face(
                new Vec3d(range.min() * r, (column.a() + 0.5) * r, (column.b() + 0.5) * r),
                new Vec3d(-1, 0, 0), faceArea
            ));
        }
        for (Map.Entry<Column, Range> entry : yRays.entrySet()) {
            Column column = entry.getKey(); Range range = entry.getValue();
            faces.add(new Face(
                new Vec3d((column.a() + 0.5) * r, range.maxPlusOne() * r, (column.b() + 0.5) * r),
                new Vec3d(0, 1, 0), faceArea
            ));
            faces.add(new Face(
                new Vec3d((column.a() + 0.5) * r, range.min() * r, (column.b() + 0.5) * r),
                new Vec3d(0, -1, 0), faceArea
            ));
        }
        for (Map.Entry<Column, Range> entry : zRays.entrySet()) {
            Column column = entry.getKey(); Range range = entry.getValue();
            faces.add(new Face(
                new Vec3d((column.a() + 0.5) * r, (column.b() + 0.5) * r, range.maxPlusOne() * r),
                new Vec3d(0, 0, 1), faceArea
            ));
            faces.add(new Face(
                new Vec3d((column.a() + 0.5) * r, (column.b() + 0.5) * r, range.min() * r),
                new Vec3d(0, 0, -1), faceArea
            ));
        }
        if (faces.isEmpty()) {
            return fallback(hull.modelLocation(), "no-silhouette-faces");
        }

        int limit = Math.max(6, maximumPatches);
        List<ModelSurfaceMap.PressurePatch> patches = aggregate(faces, limit);
        if (!hull.sampleSurfaceTriangles().isEmpty()) {
            List<ModelSurfaceMap.PressurePatch> anchored=new ArrayList<>(patches.size());
            for (var patch:patches) {
                Vec3d nearest=patch.windSamplePointLocal();
                double distance=Double.POSITIVE_INFINITY;
                for (var triangle:hull.sampleSurfaceTriangles()) {
                    Vec3d candidate=ModelCoordinates.closestTrianglePoint(patch.windSamplePointLocal(),
                        triangle.a(),triangle.b(),triangle.c());
                    double squared=candidate.subtract(patch.windSamplePointLocal()).lengthSquared();
                    if (squared<distance) { distance=squared; nearest=candidate; }
                }
                anchored.add(new ModelSurfaceMap.PressurePatch(patch.name(),patch.pointLocal(),patch.normalLocal(),
                    patch.area(),patch.kind(),patch.twoSided(),patch.symmetryPair(),patch.symmetryRole(),nearest));
            }
            patches=List.copyOf(anchored);
        }
        double wettedArea = faces.size() * faceArea;
        MassProperties massProperties = solidifiedMassProperties(xRays, r);
        Vec3d inertiaPerKg = massProperties.aboutOrigin();
        double width = (maxX - minX + 1) * r;
        double height = (maxY - minY + 1) * r;
        double length = (maxZ - minZ + 1) * r;
        return new PreparedBody(
            hull.modelLocation(), true, "sable-model-shell-silhouette",
            List.copyOf(patches), width, height, length, wettedArea, inertiaPerKg,
            massProperties.center(), massProperties.aboutCenter(),
            cells.size(), faces.size(), hull.mergedBoxes(), r
        );
    }


    /**
     * Derive a content-pack-generic rigid-body inertia shape from the same Sable shell.
     *
     * <p>The collision raster is a surface shell, not a filled mass volume. Treating every shell
     * voxel as equal vehicle mass would over-concentrate mass at the skin and make generic cars
     * artificially resistant to roll. For inertia only, each occupied X-ray is therefore filled
     * from its first to last shell cell. This gives a deterministic solidified Sable-body volume
     * without introducing a model/pack-specific mass distribution. The centroid and central inertia are used for ground dynamics; the origin
     * diagonal is retained only for compatibility and diagnostics. Uniform solidified volume
     * is a generic approximation, not an assertion about an authored engine or occupant CG.</p>
     */
    private static MassProperties solidifiedMassProperties(
        Map<Column, Range> xRays,
        double resolution
    ) {
        if (xRays == null || xRays.isEmpty() || !Double.isFinite(resolution) || resolution <= 0.0) {
            return new MassProperties(Vec3d.ZERO, Vec3d.ZERO, Vec3d.ZERO);
        }
        long cells = 0L;
        Vec3d centerSum = Vec3d.ZERO;
        double ix = 0.0;
        double iy = 0.0;
        double iz = 0.0;
        double cubePerAxis = resolution * resolution / 6.0;
        for (Map.Entry<Column, Range> entry : xRays.entrySet()) {
            Column column = entry.getKey();
            Range range = entry.getValue();
            double cy = (column.a() + 0.5) * resolution;
            double cz = (column.b() + 0.5) * resolution;
            for (int x = range.min(); x <= range.max(); ++x) {
                double cx = (x + 0.5) * resolution;
                ix += cubePerAxis + cy * cy + cz * cz;
                iy += cubePerAxis + cx * cx + cz * cz;
                iz += cubePerAxis + cx * cx + cy * cy;
                centerSum = centerSum.add(new Vec3d(cx, cy, cz));
                ++cells;
            }
        }
        if (cells <= 0L) {
            return new MassProperties(Vec3d.ZERO, Vec3d.ZERO, Vec3d.ZERO);
        }
        double inv = 1.0 / cells;
        Vec3d center = centerSum.scale(inv);
        Vec3d origin = new Vec3d(ix * inv, iy * inv, iz * inv);
        Vec3d central = origin.subtract(new Vec3d(
            center.y()*center.y()+center.z()*center.z(),
            center.x()*center.x()+center.z()*center.z(),
            center.x()*center.x()+center.y()*center.y()));
        return new MassProperties(center, origin, central);
    }

    private static void extend(Map<Column, Range> rays, Column key, int value) {
        Range current = rays.get(key);
        if (current == null) {
            rays.put(key, new Range(value, value));
        } else if (value < current.min() || value > current.max()) {
            rays.put(key, new Range(Math.min(value, current.min()), Math.max(value, current.max())));
        }
    }

    private static List<ModelSurfaceMap.PressurePatch> aggregate(List<Face> faces, int limit) {
        @SuppressWarnings("unchecked")
        List<Face>[] directions = new List[6];
        for (int i = 0; i < directions.length; ++i) directions[i] = new ArrayList<>();
        for (Face face : faces) directions[directionIndex(face.normal())].add(face);

        int nonEmpty = 0;
        for (List<Face> group : directions) if (!group.isEmpty()) ++nonEmpty;
        int[] counts = new int[6];
        int used = 0;
        for (int i = 0; i < 6; ++i) {
            if (!directions[i].isEmpty()) {
                counts[i] = 1;
                ++used;
            }
        }
        int target = Math.max(nonEmpty, limit);
        while (used < target) {
            int best = -1;
            double bestLoad = -1.0;
            for (int i = 0; i < 6; ++i) {
                if (directions[i].isEmpty()) continue;
                double load = (double) directions[i].size() / counts[i];
                if (load > bestLoad + 1.0E-9) {
                    bestLoad = load;
                    best = i;
                }
            }
            if (best < 0 || counts[best] >= directions[best].size()) break;
            ++counts[best];
            ++used;
        }

        List<ModelSurfaceMap.PressurePatch> out = new ArrayList<>(used);
        for (int direction = 0; direction < 6; ++direction) {
            List<Face> group = directions[direction];
            int bins = counts[direction];
            if (group.isEmpty() || bins == 0) continue;
            List<List<Face>> regions = new ArrayList<>();
            regions.add(new ArrayList<>(group));
            while (regions.size()<bins) {
                int largest=-1;
                double score=-1;
                for (int index=0; index<regions.size(); ++index) {
                    List<Face> region=regions.get(index);
                    if (region.size()>1 && region.size()>score) { largest=index; score=region.size(); }
                }
                if (largest<0) break;
                List<Face> region=regions.remove(largest);
                region.sort(splitComparator(region,direction));
                int middle=region.size()/2;
                regions.add(new ArrayList<>(region.subList(0,middle)));
                regions.add(new ArrayList<>(region.subList(middle,region.size())));
            }
            for (int bin=0; bin<regions.size(); ++bin) {
                List<Face> region=regions.get(bin);
                double area=0;
                Vec3d weighted=Vec3d.ZERO;
                for (Face face:region) { area+=face.area(); weighted=weighted.add(face.center().scale(face.area())); }
                Vec3d center=weighted.scale(1/area);
                Vec3d sample=region.get(0).center();
                double distance=Double.POSITIVE_INFINITY;
                for (Face face:region) {
                    double candidate=face.center().subtract(center).lengthSquared();
                    if (candidate<distance) { distance=candidate; sample=face.center(); }
                }
                out.add(new ModelSurfaceMap.PressurePatch(
                    "SABLE_BODY_"+directionName(direction)+"_"+bin,
                    center,region.get(0).normal(),area,ModelSurfaceMap.SurfaceKind.BODY,
                    false,-1,ModelSurfaceMap.SymmetryRole.UNPAIRED,sample));
            }
        }
        return out;
    }

    private static Comparator<Face> splitComparator(List<Face> group, int direction) {
        double minA = Double.POSITIVE_INFINITY, maxA = Double.NEGATIVE_INFINITY;
        double minB = Double.POSITIVE_INFINITY, maxB = Double.NEGATIVE_INFINITY;
        for (Face face : group) {
            double a;
            double b;
            if (direction <= 1) {
                a = face.center().y(); b = face.center().z();
            } else if (direction <= 3) {
                a = face.center().x(); b = face.center().z();
            } else {
                a = face.center().x(); b = face.center().y();
            }
            minA = Math.min(minA, a); maxA = Math.max(maxA, a);
            minB = Math.min(minB, b); maxB = Math.max(maxB, b);
        }
        boolean useA = (maxA - minA) >= (maxB - minB);
        return Comparator.comparingDouble(face -> {
            if (direction <= 1) return useA ? face.center().y() : face.center().z();
            if (direction <= 3) return useA ? face.center().x() : face.center().z();
            return useA ? face.center().x() : face.center().y();
        });
    }

    private static int directionIndex(Vec3d normal) {
        if (normal.x() > 0.5) return 0;
        if (normal.x() < -0.5) return 1;
        if (normal.y() > 0.5) return 2;
        if (normal.y() < -0.5) return 3;
        if (normal.z() > 0.5) return 4;
        return 5;
    }

    private static String directionName(int direction) {
        return switch (direction) {
            case 0 -> "RIGHT";
            case 1 -> "LEFT";
            case 2 -> "TOP";
            case 3 -> "BOTTOM";
            case 4 -> "FRONT";
            default -> "REAR";
        };
    }

    private static int grid(double coordinate, double resolution) {
        return (int) Math.round(coordinate / resolution);
    }

    private static PreparedBody fallback(String modelLocation, String reason) {
        return new PreparedBody(
            modelLocation == null ? "missing" : modelLocation,
            false,
            reason == null ? "unusable" : reason,
            List.of(), 0.0, 0.0, 0.0, 0.0, Vec3d.ZERO, Vec3d.ZERO, Vec3d.ZERO, 0, 0, 0, 0.0
        );
    }

    private record MassProperties(Vec3d center, Vec3d aboutOrigin, Vec3d aboutCenter) {}

    private record Cell(int x, int y, int z) {
    }

    private record Column(int a, int b) {
    }

    private record Range(int min, int max) {
        int maxPlusOne() {
            return max + 1;
        }
    }

    private record Face(Vec3d center, Vec3d normal, double area) {
    }
}
