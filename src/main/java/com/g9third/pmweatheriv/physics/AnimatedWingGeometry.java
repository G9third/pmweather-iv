package com.g9third.pmweatheriv.physics;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import minecrafttransportsimulator.baseclasses.ComputedVariable;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.jsondefs.JSONAnimatedObject;
import minecrafttransportsimulator.jsondefs.JSONAnimationDefinition;

/**
 * Detects simple mirrored whole-wing rigid rotations authored by IV content
 * packs and evaluates their live model-frame transform without remeshing.
 *
 * <p>The detector is intentionally structural rather than pack-specific: a
 * candidate must be a primary wing object, rotate around a near-vertical root
 * axis, be driven by a flap/sweep-like variable, and be visible in the intact
 * damage state. Complex chained/eased/scaled animations are left on the frozen
 * geometry fallback rather than approximated.</p>
 */
public final class AnimatedWingGeometry {
    private static final double EPSILON = 1.0E-8;

    private final List<WingRoot> roots;
    private final boolean flapVariableIsWholeWingSweep;

    private AnimatedWingGeometry(List<WingRoot> roots, boolean flapVariableIsWholeWingSweep) {
        this.roots = roots == null ? List.of() : List.copyOf(roots);
        this.flapVariableIsWholeWingSweep = flapVariableIsWholeWingSweep;
    }

    public static AnimatedWingGeometry detect(EntityVehicleF_Physics vehicle) {
        if (vehicle == null || vehicle.definition == null || vehicle.definition.rendering == null
            || vehicle.definition.rendering.animatedObjects == null) {
            return empty();
        }
        List<WingRoot> candidates = new ArrayList<>();
        for (JSONAnimatedObject object : vehicle.definition.rendering.animatedObjects) {
            WingRoot candidate = candidate(object);
            if (candidate != null) {
                candidate = new WingRoot(candidate.objectName(),candidate.variable(),
                    ModelCoordinates.scaledPoint(vehicle,candidate.pivot()),candidate.axis(),
                    candidate.offset(),candidate.clampMin(),candidate.clampMax(),ModelCoordinates.scale(vehicle));
                candidates.add(candidate);
            }
        }
        if (candidates.size() < 2) {
            return empty();
        }

        // Require one coherent root on either side of the centreline. Prefer the
        // widest mirrored pair when definitions also contain damaged/replacement
        // objects or nested wing hardware.
        WingRoot negative = null;
        WingRoot positive = null;
        for (WingRoot candidate : candidates) {
            if (candidate.pivot().x() < -0.05
                && (negative == null || Math.abs(candidate.pivot().x()) > Math.abs(negative.pivot().x()))) {
                negative = candidate;
            } else if (candidate.pivot().x() > 0.05
                && (positive == null || Math.abs(candidate.pivot().x()) > Math.abs(positive.pivot().x()))) {
                positive = candidate;
            }
        }
        if (negative == null || positive == null || !mirrored(negative, positive)) {
            return empty();
        }
        boolean flapSweep = flapLike(negative.variable()) && flapLike(positive.variable())
            && normalize(negative.variable()).equals(normalize(positive.variable()));
        return new AnimatedWingGeometry(List.of(negative, positive), flapSweep);
    }

    public static AnimatedWingGeometry empty() {
        return new AnimatedWingGeometry(List.of(), false);
    }

    public boolean active() {
        return roots.size() == 2;
    }

    public boolean usesFlapVariableAsWholeWingSweep() {
        return active() && flapVariableIsWholeWingSweep;
    }

    public List<WingRoot> roots() {
        return roots;
    }

    public RuntimeState runtime(EntityVehicleF_Physics vehicle) {
        if (!active() || vehicle == null) {
            return RuntimeState.IDENTITY;
        }
        List<RootTransform> transforms = new ArrayList<>(roots.size());
        for (WingRoot root : roots) {
            double value = liveVariable(vehicle, root.variable());
            double angleDegrees = root.angleDegrees(value);
            RigidTransform matrix = AuthoredLiftingSurfacePoses.liveMatrix(vehicle, root.objectName());
            transforms.add(new RootTransform(root, matrix != null ? matrix : RigidTransform.scaledRotationAround(
                root.pivot(), root.axis(), root.modelScale(), Math.toRadians(angleDegrees)
            ), angleDegrees));
        }
        return new RuntimeState(List.copyOf(transforms));
    }

    private static WingRoot candidate(JSONAnimatedObject object) {
        if (object == null || object.objectName == null || object.animations == null) {
            return null;
        }
        String name = normalize(object.objectName);
        if (!primaryWingName(name) || damageReplacementName(name) || !visibleWhenIntact(object)) {
            return null;
        }
        JSONAnimationDefinition rotation = null;
        int physicalTransforms = 0;
        for (JSONAnimationDefinition animation : object.animations) {
            if (animation == null || animation.animationType == null) {
                continue;
            }
            String type = normalize(animation.animationType.name());
            if (type.contains("rotation") || type.contains("translation") || type.contains("scaling")) {
                ++physicalTransforms;
            }
            if (type.contains("rotation") && animation.variable != null
                && sweepLike(animation.variable) && verticalAxis(animation.axis)) {
                if (rotation != null) {
                    return null;
                }
                rotation = animation;
            }
        }
        // Whole-wing sweep must be one simple rigid transform plus any visibility
        // gates. Nested/compound animations are intentionally not approximated.
        if (rotation == null || physicalTransforms != 1 || rotation.centerPoint == null
            || rotation.axis == null || rotation.absolute || rotation.invert
            || rotation.duration != 0 || rotation.forwardsDelay != 0 || rotation.reverseDelay != 0) {
            return null;
        }
        Vec3d pivot = point(rotation.centerPoint);
        Vec3d axis = point(rotation.axis).normalized();
        if (!pivot.isFinite() || !axis.isFinite() || axis.lengthSquared() < 0.99) {
            return null;
        }
        return new WingRoot(
            object.objectName,
            rotation.variable,
            pivot,
            axis,
            finite(rotation.offset),
            finite(rotation.clampMin),
            finite(rotation.clampMax)
        );
    }

    private static boolean primaryWingName(String name) {
        if (name == null || name.isEmpty() || !name.contains("wing")) {
            return false;
        }
        // Exclude subordinate moving hardware; the whole-wing root must own the
        // rigid sweep transform. Damage/replacement forms are filtered separately.
        return !name.contains("wingtip") && !name.contains("wing_tip")
            && !name.contains("flap") && !name.contains("slat")
            && !name.contains("spoiler") && !name.contains("aileron")
            && !name.contains("light") && !name.contains("pylon");
    }

    private static boolean damageReplacementName(String name) {
        return name.contains("damage") || name.contains("destroy") || name.contains("wreck")
            || name.matches(".*wing[lr](d|dt|damaged)$") || name.matches(".*(d|dt)wing[lr]$");
    }

    private static boolean visibleWhenIntact(JSONAnimatedObject object) {
        for (JSONAnimationDefinition animation : object.animations) {
            if (animation == null || animation.variable == null || animation.animationType == null) {
                continue;
            }
            if (!normalize(animation.animationType.name()).contains("visibility")
                || !normalize(animation.variable).contains("damage")) {
                continue;
            }
            double min = finite(animation.clampMin);
            double max = finite(animation.clampMax);
            if (max > min + EPSILON && (0.0 < min - EPSILON || 0.0 > max + EPSILON)) {
                return false;
            }
        }
        return true;
    }

    private static boolean mirrored(WingRoot first, WingRoot second) {
        double scale = Math.max(1.0, Math.max(first.pivot().length(), second.pivot().length()));
        boolean pivot = Math.abs(first.pivot().x() + second.pivot().x()) <= scale * 0.08
            && Math.abs(first.pivot().y() - second.pivot().y()) <= scale * 0.08
            && Math.abs(first.pivot().z() - second.pivot().z()) <= scale * 0.08;
        boolean axis = Math.abs(Math.abs(first.axis().y()) - Math.abs(second.axis().y())) <= 0.12
            && Math.abs(first.axis().y()) >= 0.80 && Math.abs(second.axis().y()) >= 0.80;
        return pivot && axis;
    }

    private static boolean verticalAxis(Point3D axis) {
        if (axis == null) {
            return false;
        }
        double x = Math.abs(axis.x);
        double y = Math.abs(axis.y);
        double z = Math.abs(axis.z);
        return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z)
            && y >= 0.80 && y >= x * 3.0 && y >= z * 3.0;
    }

    private static boolean sweepLike(String variable) {
        String value = normalize(variable);
        return value.contains("sweep") || value.contains("wingfold") || value.contains("wing_fold")
            || flapLike(value);
    }

    private static boolean flapLike(String variable) {
        String value = normalize(variable);
        return value.equals("flaps_actual") || value.equals("flap_actual")
            || value.equals("flaps") || value.equals("flap");
    }

    private static double liveVariable(EntityVehicleF_Physics vehicle, String variable) {
        try {
            ComputedVariable computed = vehicle.getOrCreateVariable(variable);
            return computed != null && Double.isFinite(computed.currentValue)
                ? computed.currentValue : 0.0;
        } catch (RuntimeException ignored) {
            return 0.0;
        }
    }

    private static Vec3d point(Point3D point) {
        return point == null ? Vec3d.ZERO : new Vec3d(point.x, point.y, point.z);
    }

    private static double finite(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    private static String normalize(String value) {
        if (value == null) {
            return "";
        }
        return value.toLowerCase(Locale.ROOT).replace('$', ' ').trim().replaceAll("\\s+", "_");
    }

    public record WingRoot(
        String objectName,
        String variable,
        Vec3d pivot,
        Vec3d axis,
        double offset,
        double clampMin,
        double clampMax,
        Vec3d modelScale
    ) {
        public WingRoot(String objectName, String variable, Vec3d pivot, Vec3d axis,
                        double offset, double clampMin, double clampMax) {
            this(objectName,variable,pivot,axis,offset,clampMin,clampMax,new Vec3d(1,1,1));
        }
        double angleDegrees(double rawValue) {
            double value = rawValue + offset;
            if (clampMax > clampMin + EPSILON) {
                value = Vec3d.clamp(value, clampMin, clampMax);
            }
            return Double.isFinite(value) ? value : 0.0;
        }
    }

    public static final class RuntimeState {
        private static final RuntimeState IDENTITY = new RuntimeState(List.of());
        private final List<RootTransform> transforms;

        private RuntimeState(List<RootTransform> transforms) {
            this.transforms = transforms;
        }

        public boolean active() {
            return transforms.size() == 2;
        }

        public List<RootTransform> transforms() {
            return transforms;
        }

        public RigidTransform transformFor(Vec3d point) {
            if (!active() || point == null) {
                return RigidTransform.IDENTITY;
            }
            RootTransform best = null;
            for (RootTransform candidate : transforms) {
                if ((point.x() < 0.0 && candidate.root().pivot().x() < 0.0)
                    || (point.x() >= 0.0 && candidate.root().pivot().x() >= 0.0)) {
                    best = candidate;
                    break;
                }
            }
            return best == null ? RigidTransform.IDENTITY : best.transform();
        }

        /**
         * Returns the rigid-root transform appropriate for a prepared lifting
         * strip. A strip completely inboard of the authored pivot is the fixed
         * wing glove; a strip completely outboard follows the full root angle.
         * A legacy strip that straddles the pivot stays frozen: interpolating a
         * raw angle cannot represent an authored matrix/chain. Model-derived
         * patches use source-object provenance instead of this legacy heuristic.
         */
        public RigidTransform transformForPatch(ModelSurfaceMap.LiftingPatch patch) {
            if (!active() || patch == null) {
                return RigidTransform.IDENTITY;
            }
            RootTransform rootTransform = rootTransformForPoint(patch.pointLocal());
            if (rootTransform == null) {
                return RigidTransform.IDENTITY;
            }
            double rootRadius = Math.abs(rootTransform.root().pivot().x());
            double minimum = patch.sectionMinimumRadius();
            double maximum = patch.sectionMaximumRadius();
            double fraction;
            if (Double.isFinite(minimum) && Double.isFinite(maximum)
                && maximum > minimum + EPSILON) {
                if (maximum <= rootRadius) {
                    fraction = 0.0;
                } else if (minimum >= rootRadius) {
                    fraction = 1.0;
                } else {
                    fraction = Vec3d.clamp(
                        (maximum - rootRadius) / (maximum - minimum), 0.0, 1.0
                    );
                }
            } else {
                fraction = Math.abs(patch.pointLocal().x()) > rootRadius ? 1.0 : 0.0;
            }
            if (fraction <= EPSILON) {
                return RigidTransform.IDENTITY;
            }
            if (fraction >= 1.0 - EPSILON) {
                return rootTransform.transform();
            }
            return RigidTransform.IDENTITY;
        }

        private RootTransform rootTransformForPoint(Vec3d point) {
            if (!active() || point == null) {
                return null;
            }
            for (RootTransform candidate : transforms) {
                if ((point.x() < 0.0 && candidate.root().pivot().x() < 0.0)
                    || (point.x() >= 0.0 && candidate.root().pivot().x() >= 0.0)) {
                    return candidate;
                }
            }
            return null;
        }

        public RigidTransform transformFor(WingRoot root) {
            if (root == null) {
                return RigidTransform.IDENTITY;
            }
            for (RootTransform candidate : transforms) {
                if (candidate.root() == root || candidate.root().equals(root)) {
                    return candidate.transform();
                }
            }
            return RigidTransform.IDENTITY;
        }

        public double angleDegrees(WingRoot root) {
            for (RootTransform candidate : transforms) {
                if (candidate.root() == root || candidate.root().equals(root)) {
                    return candidate.angleDegrees();
                }
            }
            return 0.0;
        }
    }

    public record RootTransform(WingRoot root, RigidTransform transform, double angleDegrees) {
    }

    public record RigidTransform(
        double m00, double m01, double m02,
        double m10, double m11, double m12,
        double m20, double m21, double m22,
        Vec3d translation
    ) {
        public static final RigidTransform IDENTITY = new RigidTransform(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            Vec3d.ZERO
        );

        static RigidTransform rotationAround(Vec3d pivot, Vec3d axis, double angle) {
            Vec3d n = axis.normalized();
            if (!n.isFinite() || n.lengthSquared() < 0.99 || Math.abs(angle) < 1.0E-12) {
                return IDENTITY;
            }
            double c = Math.cos(angle);
            double s = Math.sin(angle);
            double t = 1.0 - c;
            double x = n.x(), y = n.y(), z = n.z();
            double m00 = t*x*x + c;
            double m01 = t*x*y - s*z;
            double m02 = t*x*z + s*y;
            double m10 = t*x*y + s*z;
            double m11 = t*y*y + c;
            double m12 = t*y*z - s*x;
            double m20 = t*x*z - s*y;
            double m21 = t*y*z + s*x;
            double m22 = t*z*z + c;
            Vec3d rotatedPivot = new Vec3d(
                m00*pivot.x() + m01*pivot.y() + m02*pivot.z(),
                m10*pivot.x() + m11*pivot.y() + m12*pivot.z(),
                m20*pivot.x() + m21*pivot.y() + m22*pivot.z()
            );
            return new RigidTransform(
                m00,m01,m02,m10,m11,m12,m20,m21,m22,
                pivot.subtract(rotatedPivot)
            );
        }

        public static RigidTransform scaledRotationAround(Vec3d pivot, Vec3d axis, Vec3d scale, double angle) {
            Vec3d authoredPivot=new Vec3d(pivot.x()/scale.x(),pivot.y()/scale.y(),pivot.z()/scale.z());
            RigidTransform r=rotationAround(authoredPivot,axis,angle);
            return new RigidTransform(r.m00(),r.m01()*scale.x()/scale.y(),r.m02()*scale.x()/scale.z(),
                r.m10()*scale.y()/scale.x(),r.m11(),r.m12()*scale.y()/scale.z(),
                r.m20()*scale.z()/scale.x(),r.m21()*scale.z()/scale.y(),r.m22(),
                new Vec3d(r.translation().x()*scale.x(),r.translation().y()*scale.y(),r.translation().z()*scale.z()));
        }

        /** Inverse-transpose mapping keeps a surface normal perpendicular after anisotropic scale. */
        public Vec3d normal(Vec3d value) {
            return new Vec3d(
                (m11*m22-m12*m21)*value.x()+(m12*m20-m10*m22)*value.y()+(m10*m21-m11*m20)*value.z(),
                (m02*m21-m01*m22)*value.x()+(m00*m22-m02*m20)*value.y()+(m01*m20-m00*m21)*value.z(),
                (m01*m12-m02*m11)*value.x()+(m02*m10-m00*m12)*value.y()+(m00*m11-m01*m10)*value.z()
            );
        }

        public Vec3d point(Vec3d value) {
            return direction(value).add(translation);
        }

        /** A quaternion collider may consume only a proper orthonormal rotation. */
        public boolean isRigid() {
            Vec3d x = direction(new Vec3d(1, 0, 0)), y = direction(new Vec3d(0, 1, 0)),
                z = direction(new Vec3d(0, 0, 1));
            return x.isFinite() && y.isFinite() && z.isFinite()
                && Math.abs(x.lengthSquared() - 1) < 1E-6 && Math.abs(y.lengthSquared() - 1) < 1E-6
                && Math.abs(z.lengthSquared() - 1) < 1E-6 && Math.abs(x.dot(y)) < 1E-6
                && Math.abs(x.dot(z)) < 1E-6 && Math.abs(y.dot(z)) < 1E-6
                && x.cross(y).dot(z) > 1 - 1E-6;
        }

        public minecrafttransportsimulator.baseclasses.RotationMatrix rotationMatrix() {
            var result = new minecrafttransportsimulator.baseclasses.RotationMatrix();
            result.m00 = m00; result.m01 = m01; result.m02 = m02;
            result.m10 = m10; result.m11 = m11; result.m12 = m12;
            result.m20 = m20; result.m21 = m21; result.m22 = m22;
            return result;
        }

        public Vec3d direction(Vec3d value) {
            return new Vec3d(
                m00*value.x() + m01*value.y() + m02*value.z(),
                m10*value.x() + m11*value.y() + m12*value.z(),
                m20*value.x() + m21*value.y() + m22*value.z()
            );
        }
    }
}
