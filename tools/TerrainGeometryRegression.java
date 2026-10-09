import java.lang.reflect.*;
import java.util.*;
import org.joml.Quaterniond;
import org.joml.Vector3d;
import net.minecraft.world.phys.AABB;

/**
 * Cross-platform terrain geometry regression.
 *
 * This deliberately checks geometric invariants with numeric tolerances rather
 * than hashing thousands of raw floating-point toString() values. The old
 * golden-output form was architecture-sensitive (notably x86-64 vs ARM64)
 * despite equivalent geometry.
 */
public class TerrainGeometryRegression {
    private static final double EPS = 1.0E-9;

    static Method method(Class<?> c, String name, int count) throws Exception {
        for (var m : c.getDeclaredMethods()) {
            if (m.getName().equals(name) && m.getParameterCount() == count) {
                m.setAccessible(true);
                return m;
            }
        }
        throw new AssertionError(name);
    }

    static void require(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }

    static void near(double actual, double expected, double tolerance, String message) {
        if (!Double.isFinite(actual) || Math.abs(actual - expected) > tolerance) {
            throw new AssertionError(message + ": actual=" + actual + " expected=" + expected);
        }
    }

    static Object cuboid(Constructor<?> constructor, Vector3d center, Quaterniond q,
                         double hx, double hy, double hz) throws Exception {
        return constructor.newInstance(
            center,
            q.transform(new Vector3d(1, 0, 0)),
            q.transform(new Vector3d(0, 1, 0)),
            q.transform(new Vector3d(0, 0, 1)),
            hx, hy, hz
        );
    }

    static void validateContact(Object contact) throws Exception {
        if (contact == null) return;
        Class<?> type = contact.getClass();
        Method normalMethod = type.getDeclaredMethod("normalWorld");
        Method penetrationMethod = type.getDeclaredMethod("penetrationMeters");
        normalMethod.setAccessible(true);
        penetrationMethod.setAccessible(true);
        Vector3d normal = (Vector3d) normalMethod.invoke(contact);
        double penetration = ((Number) penetrationMethod.invoke(contact)).doubleValue();
        require(Double.isFinite(normal.x) && Double.isFinite(normal.y) && Double.isFinite(normal.z),
            "SAT normal must be finite");
        near(normal.length(), 1.0, 2.0E-9, "SAT normal must be unit length");
        require(Double.isFinite(penetration) && penetration >= -EPS,
            "SAT penetration must be finite and non-negative");
    }

    static void validateFootprint(List<int[]> points, int radius, int normalAxis) {
        require(points != null && !points.isEmpty(), "footprint must not be empty");
        int[] first = points.get(0);
        require(first.length == 3 && first[0] == 0 && first[1] == 0 && first[2] == 0,
            "footprint must begin at the seed voxel");
        Set<String> unique = new HashSet<>();
        for (int[] p : points) {
            require(p != null && p.length == 3, "footprint offset must be xyz");
            require(Math.abs(p[0]) <= radius && Math.abs(p[1]) <= radius && Math.abs(p[2]) <= radius,
                "footprint offset exceeds configured radius");
            require(p[normalAxis] == 0, "footprint must remain on the projected contact plane");
            require(unique.add(p[0] + "," + p[1] + "," + p[2]), "duplicate footprint voxel");
        }
    }

    @SuppressWarnings("unchecked")
    public static void main(String[] args) throws Exception {
        String owner = "com.g9third.pmweatheriv.sable.SableTerrainSweep";
        Class<?> c = Class.forName(owner);
        Class<?> shape = Class.forName(owner + "$OrientedCuboid");
        Constructor<?> constructor = shape.getDeclaredConstructors()[0];
        constructor.setAccessible(true);
        Method sat = method(c, "satContact", 2);
        Method area = method(c, "projectedAreaSquareMeters", 2);
        AABB block = new AABB(-.5, -.5, -.5, .5, .5, .5);

        // Deterministic reference geometry: overlap, separation and touching contact.
        Quaterniond identity = new Quaterniond();
        Object inside = cuboid(constructor, new Vector3d(0, 0, 0), identity, .25, .25, .25);
        Object insideContact = sat.invoke(null, inside, block);
        require(insideContact != null, "centered cuboid must overlap block");
        validateContact(insideContact);

        Object separated = cuboid(constructor, new Vector3d(2, 0, 0), identity, .25, .25, .25);
        require(sat.invoke(null, separated, block) == null, "separated cuboid must not contact block");

        Object touching = cuboid(constructor, new Vector3d(.75, 0, 0), identity, .25, .25, .25);
        Object touchingContact = sat.invoke(null, touching, block);
        require(touchingContact != null, "face-touching cuboid must count as contact");
        validateContact(touchingContact);

        // Exact orthographic projected areas for an axis-aligned 2x4x6 m cuboid.
        Object box246 = cuboid(constructor, new Vector3d(), identity, 1.0, 2.0, 3.0);
        near(((Number) area.invoke(null, box246, new Vector3d(1, 0, 0))).doubleValue(), 24.0, 1.0E-10,
            "projected area normal X");
        near(((Number) area.invoke(null, box246, new Vector3d(0, 1, 0))).doubleValue(), 12.0, 1.0E-10,
            "projected area normal Y");
        near(((Number) area.invoke(null, box246, new Vector3d(0, 0, 1))).doubleValue(), 8.0, 1.0E-10,
            "projected area normal Z");

        // Broad randomized coverage of SAT and projected-area invariants. No raw
        // floating-point text is compared, so equivalent ARM64/x86-64 results pass.
        Random random = new Random(1111);
        int contacts = 0;
        int separations = 0;
        for (int i = 0; i < 4000; ++i) {
            Quaterniond q = new Quaterniond().rotateXYZ(
                random.nextDouble() * 3.0,
                random.nextDouble() * 3.0,
                random.nextDouble() * 3.0
            );
            Vector3d center = new Vector3d(
                random.nextDouble() * 4.0 - 2.0,
                random.nextDouble() * 4.0 - 2.0,
                random.nextDouble() * 4.0 - 2.0
            );
            Object obb = cuboid(
                constructor, center, q,
                .02 + random.nextDouble(),
                .02 + random.nextDouble(),
                .02 + random.nextDouble()
            );
            Object contact = sat.invoke(null, obb, block);
            if (contact == null) {
                ++separations;
            } else {
                ++contacts;
                validateContact(contact);
            }
            double projected = ((Number) area.invoke(
                null, obb, new Vector3d(1, 2, 3).normalize()
            )).doubleValue();
            require(Double.isFinite(projected) && projected > 0.0,
                "projected area must be finite and positive");
        }
        require(contacts > 0 && separations > 0, "random SAT sweep must exercise both outcomes");

        Class<?> model = Class.forName("com.g9third.pmweatheriv.terrain.trueimpact.ExternalWorldImpactModel");
        Method footprint = method(model, "footprintOffsets", 10);
        Class<?> runtimeConfig = Class.forName(
            "com.g9third.pmweatheriv.terrain.trueimpact.damage.ImpactRuntimeConfig"
        );
        Field radiusField = runtimeConfig.getDeclaredField("PENETRATION_FOOTPRINT_RADIUS");
        radiusField.setAccessible(true);
        int radius = Math.max(0, radiusField.getInt(null));

        for (int i = 0; i < 300; ++i) {
            double dx = random.nextDouble() * 2.0 - 1.0;
            double dy = random.nextDouble() * 2.0 - 1.0;
            double dz = random.nextDouble() * 2.0 - 1.0;
            List<int[]> points = (List<int[]>) footprint.invoke(
                null,
                random.nextDouble() * 12.0,
                dx, dy, dz,
                random.nextDouble(), random.nextDouble(), random.nextDouble(),
                0, 0, 0
            );
            double ax = Math.abs(dx), ay = Math.abs(dy), az = Math.abs(dz);
            int normalAxis = ay >= ax && ay >= az ? 1 : (ax >= az ? 0 : 2);
            validateFootprint(points, radius, normalAxis);
        }

        System.out.println("TerrainGeometryRegression: invariants passed");
    }
}
