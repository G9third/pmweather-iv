package com.g9third.pmweatheriv.physics;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import com.g9third.pmweatheriv.model.ParsedModelSnapshot.Mesh;

import static com.g9third.pmweatheriv.physics.ModelGeometryData.AnimationHint;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.Bounds;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.BoundsAccumulator;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.EPSILON;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.ModelObject;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.ObjectStats;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.QuotaRemainder;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.RawTriangle;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.SurfaceKind;
import static com.g9third.pmweatheriv.physics.ModelAnimationHints.containsAny;

/** Area-preserving, bounded sampling of immutable model triangles. */
final class ModelTriangleSampling {
    private ModelTriangleSampling() {}

    static final int FLOATS_PER_VERTEX = 3;

    static final int FLOATS_PER_TRIANGLE = FLOATS_PER_VERTEX * 3;

    static final double MIN_TRIANGLE_AREA = 0.0005;

    static final int MAX_RETAINED_TRIANGLES = 48000;

    static final int MIN_GENERIC_OBJECT_SAMPLES = 8;

    static final int MIN_SURFACE_OBJECT_SAMPLES = 64;

    static final int MAX_GENERIC_OBJECT_SAMPLES = 4096;

    static final int MAX_SURFACE_OBJECT_SAMPLES = 8192;

    static ObjectStats scanObject(Mesh object) {
        FloatBuffer buffer = object.positions();
        buffer.rewind();
        BoundsAccumulator bounds = new BoundsAccumulator();
        int validTriangles = 0;
        double totalArea = 0.0;
        while (buffer.remaining() >= FLOATS_PER_TRIANGLE) {
            Vec3d first = readPosition(buffer);
            Vec3d second = readPosition(buffer);
            Vec3d third = readPosition(buffer);
            Vec3d cross = second.subtract(first).cross(third.subtract(first));
            double twiceArea = cross.length();
            double area = 0.5 * twiceArea;
            if (!validTriangle(first, second, third, area, twiceArea)) {
                continue;
            }
            bounds.include(first);
            bounds.include(second);
            bounds.include(third);
            totalArea += area;
            ++validTriangles;
        }
        return new ObjectStats(validTriangles, totalArea, bounds.finishLoose());
    }

    static int readSampledTriangles(
        ModelObject object,
        int requested,
        List<RawTriangle> output
    ) {
        int quota = Math.max(0, Math.min(requested, object.stats().validTriangles()));
        if (quota <= 0) {
            return 0;
        }
        FloatBuffer buffer = object.vertices().positions();
        buffer.rewind();
        int validIndex = 0;
        int retained = 0;
        int nextTarget = sampleTarget(retained, quota, object.stats().validTriangles());
        List<RawTriangle> selected = new ArrayList<>(quota);
        double selectedArea = 0.0;
        while (buffer.remaining() >= FLOATS_PER_TRIANGLE && retained < quota) {
            Vec3d first = readPosition(buffer);
            Vec3d second = readPosition(buffer);
            Vec3d third = readPosition(buffer);
            Vec3d cross = second.subtract(first).cross(third.subtract(first));
            double twiceArea = cross.length();
            double area = 0.5 * twiceArea;
            if (!validTriangle(first, second, third, area, twiceArea)) {
                continue;
            }
            if (validIndex >= nextTarget) {
                Vec3d centroid = first.add(second).add(third).scale(1.0 / 3.0);
                Vec3d normal = cross.scale(1.0 / twiceArea);
                selected.add(new RawTriangle(
                    first, second, third, centroid, normal, area, object.objectName(),
                    object.stats().bounds()
                ));
                selectedArea += area;
                ++retained;
                nextTarget = sampleTarget(retained, quota, object.stats().validTriangles());
            }
            ++validIndex;
        }
        // Uniform triangle-count sampling must preserve the original object's total
        // surface area. Without this correction, detailed IFS models lose most of
        // their aerodynamic area simply because only a bounded subset is retained.
        double areaScale = selectedArea > EPSILON
            ? object.stats().totalArea() / selectedArea
            : 1.0;
        for (RawTriangle triangle : selected) {
            output.add(new RawTriangle(
                triangle.first(), triangle.second(), triangle.third(),
                triangle.centroid(), triangle.normal(), triangle.area() * areaScale, triangle.objectName(),
                triangle.objectBounds()
            ));
        }
        return retained;
    }

    static int sampleTarget(int retained, int quota, int total) {
        if (retained >= quota) {
            return Integer.MAX_VALUE;
        }
        return Math.min(total - 1, (int) Math.floor((retained + 0.5) * total / Math.max(1.0, quota)));
    }

    static boolean validTriangle(
        Vec3d first,
        Vec3d second,
        Vec3d third,
        double area,
        double twiceArea
    ) {
        return first.isFinite() && second.isFinite() && third.isFinite()
            && Double.isFinite(area) && area >= MIN_TRIANGLE_AREA && area <= 65536.0
            && twiceArea > EPSILON;
    }

    static Map<ModelObject, Integer> allocateObjectQuotas(List<ModelObject> objects) {
        int totalTriangles = objects.stream().mapToInt(object -> object.stats().validTriangles()).sum();
        int capacity = Math.min(MAX_RETAINED_TRIANGLES, totalTriangles);
        Map<ModelObject, Integer> quotas = new LinkedHashMap<>();
        if (capacity <= 0) {
            return quotas;
        }

        int minimumAssigned = 0;
        for (ModelObject object : objects) {
            int minimum = object.priority() >= 3.0
                ? MIN_SURFACE_OBJECT_SAMPLES
                : MIN_GENERIC_OBJECT_SAMPLES;
            minimum = Math.min(minimum, object.stats().validTriangles());
            quotas.put(object, minimum);
            minimumAssigned += minimum;
        }
        if (minimumAssigned > capacity) {
            quotas.clear();
            int remaining = capacity;
            List<ModelObject> ordered = new ArrayList<>(objects);
            ordered.sort(Comparator
                .comparingDouble(ModelObject::priority).reversed()
                .thenComparing(Comparator.comparingInt(
                    (ModelObject object) -> object.stats().validTriangles()
                ).reversed())
                .thenComparing(ModelObject::normalizedName));
            for (ModelObject object : ordered) {
                int quota = remaining > 0 ? 1 : 0;
                quotas.put(object, quota);
                remaining -= quota;
            }
            return quotas;
        }

        int remaining = capacity - minimumAssigned;
        double totalWeight = 0.0;
        Map<ModelObject, Double> weights = new LinkedHashMap<>();
        for (ModelObject object : objects) {
            double weight = Math.sqrt(object.stats().validTriangles()) * object.priority();
            weights.put(object, weight);
            totalWeight += weight;
        }

        List<QuotaRemainder> remainders = new ArrayList<>();
        int distributed = 0;
        for (ModelObject object : objects) {
            int maximum = object.priority() >= 3.0
                ? MAX_SURFACE_OBJECT_SAMPLES
                : MAX_GENERIC_OBJECT_SAMPLES;
            maximum = Math.min(maximum, object.stats().validTriangles());
            double exact = remaining * weights.get(object) / Math.max(EPSILON, totalWeight);
            int extra = Math.min(maximum - quotas.get(object), (int) Math.floor(exact));
            quotas.put(object, quotas.get(object) + Math.max(0, extra));
            distributed += Math.max(0, extra);
            remainders.add(new QuotaRemainder(object, exact - Math.floor(exact), maximum));
        }

        int leftover = remaining - distributed;
        remainders.sort(Comparator
            .comparingDouble(QuotaRemainder::fraction).reversed()
            .thenComparing(Comparator.comparingDouble(
                (QuotaRemainder remainder) -> remainder.object().priority()
            ).reversed())
            .thenComparing(remainder -> remainder.object().normalizedName()));
        while (leftover > 0) {
            boolean progressed = false;
            for (QuotaRemainder remainder : remainders) {
                ModelObject object = remainder.object();
                int current = quotas.get(object);
                if (current < remainder.maximum()) {
                    quotas.put(object, current + 1);
                    --leftover;
                    progressed = true;
                    if (leftover <= 0) {
                        break;
                    }
                }
            }
            if (!progressed) {
                break;
            }
        }
        return quotas;
    }

    static double objectPriority(String name, AnimationHint hint, ObjectStats stats) {
        SurfaceKind named = SurfaceKind.fromSpecificName(name);
        if (named != null && named != SurfaceKind.ENGINE_NACELLE) {
            return 5.0;
        }
        // Animation variables are only evidence. Large content packs frequently animate
        // cockpit sticks, pedals, landing-gear steering, trim indicators, and lights with
        // the same variables as real flight surfaces. Give an unnamed animated object
        // surface priority only when its own bounds are plausibly wing/tail shaped.
        if (hint.anySurfaceControl() && plausibleAnimatedSurface(stats.bounds(), hint)) {
            return 4.0;
        }
        if (containsAny(name, "body", "fuselage", "fuselage_fs", "hull", "shell", "airframe")) {
            return 3.0;
        }
        if (named == SurfaceKind.ENGINE_NACELLE
            || containsAny(name, "engine", "nacelle", "cowling", "cowl", "intake")) {
            return 2.0;
        }
        return 1.0;
    }

    static boolean plausibleAnimatedSurface(Bounds bounds, AnimationHint hint) {
        if (bounds == null || !bounds.valid()) {
            return false;
        }
        double x = bounds.spanX();
        double y = bounds.spanY();
        double z = bounds.spanZ();
        double longest = Math.max(x, Math.max(y, z));
        if (longest < 0.35) {
            return false;
        }
        boolean horizontal = x >= Math.max(0.45, y * 1.35)
            && (x >= z * 0.75 || z >= Math.max(0.45, y * 1.35));
        boolean vertical = y >= Math.max(0.45, x * 1.20)
            || (y >= 0.45 && z >= Math.max(0.45, x * 1.20));
        return ((hint.aileron() || hint.elevator() || hint.flap() || hint.slat() || hint.spoiler())
                && horizontal)
            || (hint.rudder() && vertical);
    }

    static Vec3d readPosition(FloatBuffer buffer) {
        return new Vec3d(buffer.get(), buffer.get(), buffer.get());
    }
}
