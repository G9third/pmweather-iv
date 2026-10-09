package com.g9third.pmweatheriv.physics;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.UUID;
import minecrafttransportsimulator.baseclasses.AnimationSwitchbox;
import minecrafttransportsimulator.baseclasses.TransformationMatrix;
import minecrafttransportsimulator.entities.components.AEntityD_Definable;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.jsondefs.JSONAnimatedObject;
import minecrafttransportsimulator.jsondefs.JSONAnimationDefinition;

/** Immutable model provenance and copied IV poses; never advances a second animation clock. */
public final class AuthoredLiftingSurfacePoses {
    private AuthoredLiftingSurfacePoses() {}
    private static final Map<AEntityD_Definable<?>, TickMatrices> OWNER_MATRICES = new java.util.WeakHashMap<>();
    private record MatrixResult(AnimatedWingGeometry.RigidTransform matrix, boolean visible) {}
    private record TickMatrices(long tick, Vec3d scale, Map<String, MatrixResult> objects) {}

    public enum Status { STATIC, AUTHORED, FROZEN_MIXED_SOURCES, FROZEN_UNSUPPORTED_CHAIN,
        FROZEN_NO_SOURCE, FROZEN_UNAVAILABLE, HIDDEN }

    /** Raw part geometry is prepared once in the master frame; no mutable part/entity reference is retained. */
    public record PartReference(UUID ownerId, String modelLocation,
                                AnimatedWingGeometry.RigidTransform inverseReference, String slotPath) {
        public PartReference(UUID ownerId, String modelLocation,
                             AnimatedWingGeometry.RigidTransform inverseReference) {
            this(ownerId, modelLocation, inverseReference, ownerId.toString());
        }
    }

    public record Binding(String objectName, boolean elevator, boolean aileron, boolean rudder,
                          boolean flapSweep, boolean blended, Status status, PartReference partReference,
                          boolean elevatorTrim, boolean aileronTrim, boolean rudderTrim) {
        public Binding(String objectName, boolean elevator, boolean aileron, boolean rudder,
                       boolean flapSweep, boolean blended, Status status, PartReference partReference) {
            this(objectName, elevator, aileron, rudder, flapSweep, blended, status, partReference,
                false, false, false);
        }
        public Binding(String objectName, boolean elevator, boolean aileron, boolean rudder,
                       boolean flapSweep, boolean blended, Status status) {
            this(objectName, elevator, aileron, rudder, flapSweep, blended, status, null);
        }
        static Binding frozen(Status status) { return new Binding("", false, false, false, false, false, status); }
    }

    public record Pose(AnimatedWingGeometry.RigidTransform transform, Binding binding, Status status,
                       AnimatedWingGeometry.RigidTransform previousTransform) {
        public Pose(AnimatedWingGeometry.RigidTransform transform, Binding binding, Status status) {
            this(transform, binding, status, null);
        }
        public boolean live() { return status == Status.AUTHORED; }
        public boolean visible() { return status != Status.HIDDEN; }
        /** Relative actuation speed in master model coordinates; copied once per 20Hz owner tick. */
        public Vec3d relativePointVelocity(Vec3d preparedModelPoint) {
            if (!live() || previousTransform == null || preparedModelPoint == null || !preparedModelPoint.isFinite())
                return Vec3d.ZERO;
            Vec3d velocity = transform.point(preparedModelPoint).subtract(previousTransform.point(preparedModelPoint))
                .scale(1.0 / FlightMath.MC_TICK_SECONDS);
            return velocity.isFinite() ? velocity : Vec3d.ZERO;
        }
        Pose withPrevious(Pose previous) {
            return live() && previous != null && previous.live() && binding.equals(previous.binding())
                ? new Pose(transform, binding, status, previous.transform()) : this;
        }
        public double control(double elevator, double aileron, double rudder) {
            return (live() && binding.elevator() ? 0 : elevator)
                + (live() && binding.aileron() ? 0 : aileron)
                + (live() && binding.rudder() ? 0 : rudder);
        }
        public double control(double elevator, double aileron, double rudder,
                              double elevatorTrim, double aileronTrim, double rudderTrim) {
            return control(elevator, aileron, rudder)
                + (live() && binding.elevatorTrim() ? 0 : elevatorTrim)
                + (live() && binding.aileronTrim() ? 0 : aileronTrim)
                + (live() && binding.rudderTrim() ? 0 : rudderTrim);
        }
        public double areaFactor(ModelSurfaceMap.LiftingPatch patch) {
            Vec3d span = patch.spanLocal(), chord = patch.chordLocal();
            double baseline = span.cross(chord).length();
            double liveArea = transform.direction(span).cross(transform.direction(chord)).length();
            return baseline > 1E-8 && Double.isFinite(liveArea) ? liveArea / baseline : 1;
        }
    }

    public record Snapshot(Map<String, Pose> patches) {
        public static final Snapshot EMPTY = new Snapshot(Map.of());
        public Snapshot { patches = Map.copyOf(patches); }
        public Pose forPatch(ModelSurfaceMap.LiftingPatch patch,
                             AnimatedWingGeometry.RuntimeState legacyWing) {
            Pose pose = patches.get(patch.name());
            return pose != null ? pose : new Pose(legacyWing.transformForPatch(patch),
                Binding.frozen(Status.FROZEN_NO_SOURCE), Status.FROZEN_NO_SOURCE);
        }
        public boolean activeWing(ModelSurfaceMap.PreparedModel model) {
            if (model == null) return false;
            for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
                if (patch.kind() != ModelSurfaceMap.SurfaceKind.WING && patch.kind() != ModelSurfaceMap.SurfaceKind.AILERON
                    && patch.kind() != ModelSurfaceMap.SurfaceKind.ELEVON) continue;
                Pose pose = patches.get(patch.name());
                if (pose != null && pose.live()) return true;
            }
            return false;
        }
        public boolean allWingsUseFlapSweep(ModelSurfaceMap.PreparedModel model) {
            boolean found = false;
            for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
                if (patch.kind() != ModelSurfaceMap.SurfaceKind.WING) continue;
                Pose pose = patches.get(patch.name());
                if (pose == null || !pose.live() || !pose.binding().flapSweep()) return false;
                found = true;
            }
            return found;
        }
    }

    static Map<String, Binding> bind(EntityVehicleF_Physics vehicle, List<ModelSurfaceMap.LiftingPatch> patches,
                                    Map<ModelSurfaceMap.SurfaceKind, List<ModelGeometryData.ClassifiedTriangle>> source,
                                    ModelSurfaceMap.Bounds bounds) {
        return bind(definitions(vehicle), patches, source, bounds);
    }

    // Retained triangle identities are the evidence; synthetic patch names are never animation keys.
    static Map<String, Binding> bind(Map<String, JSONAnimatedObject> definitions,
                                    List<ModelSurfaceMap.LiftingPatch> patches,
                                    Map<ModelSurfaceMap.SurfaceKind, List<ModelGeometryData.ClassifiedTriangle>> source,
                                    ModelSurfaceMap.Bounds bounds) {
        Map<String, Binding> result = new LinkedHashMap<>();
        double center = (bounds.minimum().x() + bounds.maximum().x()) * .5;
        double tolerance = LiftingSurfaceFrames.mirrorTolerance(bounds);
        for (ModelSurfaceMap.LiftingPatch patch : patches) {
            Set<String> objects = new HashSet<>();
            for (ModelGeometryData.ClassifiedTriangle triangle : source.getOrDefault(patch.kind(), List.of())) {
                double offset = triangle.centroid().x() - center;
                double radius = Math.abs(offset);
                if (patch.sectionCount() > 1 && (radius > tolerance || patch.sectionIndex() != 0)
                    && (radius + 1E-8 < patch.sectionMinimumRadius() || radius >= patch.sectionMaximumRadius())) continue;
                boolean left = patch.symmetryRole() == ModelSurfaceMap.SymmetryRole.MIRRORED_LEFT
                    || patch.symmetryRole() == ModelSurfaceMap.SymmetryRole.LEFT_ONLY
                    || patch.symmetryRole() == ModelSurfaceMap.SymmetryRole.UNPAIRED && patch.pointLocal().x() < center - tolerance;
                boolean right = patch.symmetryRole() == ModelSurfaceMap.SymmetryRole.MIRRORED_RIGHT
                    || patch.symmetryRole() == ModelSurfaceMap.SymmetryRole.RIGHT_ONLY
                    || patch.symmetryRole() == ModelSurfaceMap.SymmetryRole.UNPAIRED && patch.pointLocal().x() > center + tolerance;
                // Center triangles contribute to both paired summaries, so retain their provenance too.
                if (left && offset > tolerance || right && offset < -tolerance) continue;
                if (triangle.objectName() != null) objects.add(triangle.objectName());
            }
            Binding binding = null;
            for (String object : objects) {
                Binding candidate = resolve(definitions, object, new HashSet<>());
                if (binding == null) binding = candidate;
                else if (!binding.equals(candidate)) {
                    binding = Binding.frozen(Status.FROZEN_MIXED_SOURCES);
                    break;
                }
            }
            result.put(patch.name(), binding == null ? Binding.frozen(Status.FROZEN_NO_SOURCE) : binding);
        }
        return Map.copyOf(result);
    }

    static Binding resolve(Map<String, JSONAnimatedObject> definitions, String name, Set<String> visiting) {
        JSONAnimatedObject object = definitions.get(name);
        if (object == null) return Binding.frozen(Status.STATIC);
        if (!visiting.add(name)) return Binding.frozen(Status.FROZEN_UNSUPPORTED_CHAIN);
        Binding parent = Binding.frozen(Status.STATIC);
        if (object.applyAfter != null && !object.applyAfter.isBlank()) {
            if (!definitions.containsKey(object.applyAfter)) return Binding.frozen(Status.FROZEN_UNSUPPORTED_CHAIN);
            parent = resolve(definitions, object.applyAfter, visiting);
            if (parent.status() != Status.STATIC && parent.status() != Status.AUTHORED) return parent;
        }
        boolean physical = false, elevator = parent.elevator(), aileron = parent.aileron(),
            rudder = parent.rudder(), sweep = parent.flapSweep();
        boolean elevatorTrim = parent.elevatorTrim(), aileronTrim = parent.aileronTrim(),
            rudderTrim = parent.rudderTrim();
        for (JSONAnimationDefinition animation : object.animations == null ? List.<JSONAnimationDefinition>of() : object.animations) {
            String type = animation == null || animation.animationType == null ? "" : animation.animationType.name();
            if (type.equals("SCALING")) return Binding.frozen(Status.FROZEN_UNSUPPORTED_CHAIN);
            if (!type.equals("ROTATION") && !type.equals("TRANSLATION")) continue;
            physical = true;
            String variable = animation.variable == null ? "" : animation.variable.toLowerCase(Locale.ROOT);
            boolean trimVariable = variable.contains("trim");
            elevator |= !trimVariable && variable.contains("elevator");
            aileron |= !trimVariable && variable.contains("aileron");
            rudder |= !trimVariable && variable.contains("rudder");
            elevatorTrim |= trimVariable && variable.contains("elevator");
            aileronTrim |= trimVariable && variable.contains("aileron");
            rudderTrim |= trimVariable && variable.contains("rudder");
            if (type.equals("ROTATION") && variable.contains("flap") && animation.axis != null) {
                double horizontal = Math.hypot(animation.axis.x, animation.axis.z);
                sweep |= Math.abs(animation.axis.y) > horizontal * 4;
            }
        }
        if (!physical && parent.status() == Status.STATIC)
            return new Binding(object.objectName, false, false, false, false,
                object.blendedAnimations, Status.STATIC);
        if (!physical && (object.animations == null || object.animations.isEmpty())) return parent;
        // A child's own visibility/inhibitor clocks must run even when only its parent moves.
        return new Binding(object.objectName, elevator, aileron, rudder, sweep,
            object.blendedAnimations, Status.AUTHORED, null, elevatorTrim, aileronTrim, rudderTrim);
    }

    static Map<String, JSONAnimatedObject> definitions(AEntityD_Definable<?> owner) {
        return definitions(owner == null || owner.definition == null || owner.definition.rendering == null
            ? null : owner.definition.rendering.animatedObjects,
            owner == null ? null : owner.animatedObjectDefinitions);
    }

    /** Runtime definitions include IV's resolved active content and override the base rendering list. */
    static Map<String, JSONAnimatedObject> definitions(List<JSONAnimatedObject> authored,
                                                      Map<String, JSONAnimatedObject> runtime) {
        Map<String, JSONAnimatedObject> definitions = new HashMap<>();
        if (authored != null) {
            for (JSONAnimatedObject object : authored)
                if (object != null && object.objectName != null) definitions.put(object.objectName, object);
        }
        if (runtime != null) runtime.forEach((name, object) -> {
            if (name != null && object != null) definitions.put(name, object);
        });
        return definitions;
    }

    static String signature(AEntityD_Definable<?> vehicle) {
        return signature(definitions(vehicle));
    }

    static String signature(Map<String, JSONAnimatedObject> definitions) {
        StringBuilder text = new StringBuilder();
        definitions.entrySet().stream().sorted(Map.Entry.comparingByKey()).forEach(entry -> {
            JSONAnimatedObject object = entry.getValue();
            text.append(entry.getKey()).append(':').append(object.objectName).append(':')
                .append(object.applyAfter).append(':').append(object.blendedAnimations);
            appendAnimations(text, object.animations);
            text.append(';');
        });
        return text.toString();
    }

    /** Every field that changes the native matrix, visibility, or clock must invalidate a binding. */
    static void appendAnimations(StringBuilder text, List<JSONAnimationDefinition> animations) {
        if (animations == null) { text.append("|null"); return; }
        for (JSONAnimationDefinition animation : animations) {
            if (animation == null) { text.append("|null-animation"); continue; }
            text.append('|').append(animation.animationType).append(':').append(animation.variable)
                .append(':').append(animation.axis).append(':').append(animation.centerPoint)
                .append(':').append(animation.offset).append(':').append(animation.clampMin)
                .append(':').append(animation.clampMax).append(':').append(animation.absolute)
                .append(':').append(animation.invert).append(':').append(animation.duration)
                .append(':').append(animation.forwardsEasing).append(':').append(animation.reverseEasing)
                .append(':').append(animation.forwardsDelay).append(':').append(animation.reverseDelay)
                .append(':').append(animation.skipForwardsMovement).append(':').append(animation.skipReverseMovement);
        }
    }

    static Snapshot snapshot(EntityVehicleF_Physics vehicle, ModelSurfaceMap.PreparedModel model, AircraftState state) {
        if (model == null || model.authoredPoseBindings().isEmpty() || vehicle == null) return Snapshot.EMPTY;
        if (state != null && state.liftingPoseModel == model && state.liftingPoseTick == vehicle.ticksExisted)
            return state.liftingPoses;
        Snapshot previous = state != null && state.liftingPoseModel == model
            && state.liftingPoseTick == vehicle.ticksExisted - 1 ? state.liftingPoses : Snapshot.EMPTY;
        Map<String, Pose> poses = new HashMap<>();
        Map<Binding, Pose> objects = new HashMap<>();
        Map<UUID, APart> parts = new HashMap<>();
        for (APart part : vehicle.allParts) if (part != null) parts.put(part.uniqueUUID, part);
        Map<UUID, AnimatedWingGeometry.RigidTransform> partFrames = new HashMap<>();
        Vec3d scale = ModelCoordinates.scale(vehicle);
        for (Map.Entry<String, Binding> entry : model.authoredPoseBindings().entrySet()) {
            Binding binding = entry.getValue();
            Pose pose;
            if (binding.partReference() != null) pose = objects.computeIfAbsent(binding,
                ignored -> readPartPose(vehicle, parts.get(binding.partReference().ownerId()), binding, partFrames));
            else if (binding.status() != Status.AUTHORED
                && (binding.status() != Status.STATIC || binding.objectName().isBlank()))
                pose = new Pose(AnimatedWingGeometry.RigidTransform.IDENTITY, binding, binding.status());
            else pose = objects.computeIfAbsent(binding, ignored -> readPose(vehicle, binding, scale));
            poses.put(entry.getKey(), pose.withPrevious(previous.patches().get(entry.getKey())));
        }
        Snapshot snapshot = new Snapshot(poses);
        if (state != null) {
            state.liftingPoseModel = model;
            state.liftingPoseTick = vehicle.ticksExisted;
            state.liftingPoses = snapshot;
        }
        return snapshot;
    }

    private static Pose readPose(EntityVehicleF_Physics vehicle, Binding binding, Vec3d scale) {
        MatrixResult result = ownerMatrix(vehicle, binding.objectName(), scale);
        if (result != null && !result.visible() && !binding.blended())
            return new Pose(AnimatedWingGeometry.RigidTransform.IDENTITY, binding, Status.HIDDEN);
        if (result != null && result.matrix() != null) return new Pose(result.matrix(), binding, binding.status());
        if (binding.status() == Status.STATIC && !vehicle.animatedObjectSwitchboxes.containsKey(binding.objectName()))
            return new Pose(AnimatedWingGeometry.RigidTransform.IDENTITY, binding, Status.STATIC);
        return new Pose(AnimatedWingGeometry.RigidTransform.IDENTITY, binding, Status.FROZEN_UNAVAILABLE);
    }

    private static Pose readPartPose(EntityVehicleF_Physics vehicle, APart part, Binding binding,
                                    Map<UUID, AnimatedWingGeometry.RigidTransform> frames) {
        if (binding.status() == Status.HIDDEN || !AttachedLiftingSurfaces.active(part))
            return new Pose(AnimatedWingGeometry.RigidTransform.IDENTITY, binding, Status.HIDDEN);
        if (!binding.partReference().modelLocation().equals(ModelAnimationHints.modelLocation(part)))
            return new Pose(AnimatedWingGeometry.RigidTransform.IDENTITY, binding, Status.HIDDEN);
        AnimatedWingGeometry.RigidTransform frame = frames.computeIfAbsent(part.uniqueUUID,
            ignored -> AttachedLiftingSurfaces.resolvedFrame(vehicle, part));
        if (frame == null) return new Pose(AnimatedWingGeometry.RigidTransform.IDENTITY, binding, Status.HIDDEN);
        if (binding.status()!=Status.STATIC && binding.status()!=Status.AUTHORED)
            return new Pose(AnimatedWingGeometry.RigidTransform.IDENTITY, binding, Status.HIDDEN);
        Vec3d partScale = part.scale == null ? new Vec3d(1, 1, 1)
            : new Vec3d(part.scale.x, part.scale.y, part.scale.z);
        MatrixResult object = ownerMatrix(part, binding.objectName(), partScale);
        if (object != null && !object.visible() && !binding.blended())
            return new Pose(AnimatedWingGeometry.RigidTransform.IDENTITY, binding, Status.HIDDEN);
        AnimatedWingGeometry.RigidTransform objectMatrix = object == null
            ? binding.status()==Status.STATIC ? AnimatedWingGeometry.RigidTransform.IDENTITY : null : object.matrix();
        if (objectMatrix == null)
            return new Pose(AnimatedWingGeometry.RigidTransform.IDENTITY, binding, Status.HIDDEN);
        return new Pose(ModelPoseMath.compose(ModelPoseMath.compose(frame, objectMatrix),
            binding.partReference().inverseReference()), binding, Status.AUTHORED);
    }

    public static AnimatedWingGeometry.RigidTransform liveMatrix(EntityVehicleF_Physics vehicle, String name) {
        MatrixResult result = ownerMatrix(vehicle, name, ModelCoordinates.scale(vehicle));
        return result != null && result.visible() ? result.matrix() : null;
    }

    /** Explicit owner and solver scale; part object keys belong to the part, never its master vehicle. */
    public static AnimatedWingGeometry.RigidTransform liveMatrix(AEntityD_Definable<?> owner, String name, Vec3d scale) {
        MatrixResult result = ownerMatrix(owner, name, scale);
        return result != null && result.visible() ? result.matrix() : null;
    }

    // Aero and the already-supported whole-wing colliders share these copied values.
    // Renderer writes to IV's mutable netMatrix cannot alter the substep geometry.
    private static synchronized MatrixResult ownerMatrix(AEntityD_Definable<?> vehicle, String name, Vec3d scale) {
        if (vehicle == null || name == null) return null;
        TickMatrices tick = OWNER_MATRICES.get(vehicle);
        if (tick == null || tick.tick() != vehicle.ticksExisted || !tick.scale().equals(scale)) {
            tick = new TickMatrices(vehicle.ticksExisted, scale, new HashMap<>());
            OWNER_MATRICES.put(vehicle, tick);
        }
        if (tick.objects().containsKey(name)) return tick.objects().get(name);
        MatrixResult result = null;
        try {
            AnimationSwitchbox box = vehicle.animatedObjectSwitchboxes.get(name);
            if (box != null) {
                boolean visible = box.runSwitchbox(0, false);
                result = new MatrixResult(copyMatrix(box.netMatrix, scale), visible);
            }
        } catch (RuntimeException ignored) {
            // Invalid/missing IV chain: freeze explicitly instead of guessing a pose.
        }
        tick.objects().put(name, result);
        return result;
    }

    /** Copies primitives, conjugating IV's unscaled OBJ transform into the scaled solver frame. */
    static AnimatedWingGeometry.RigidTransform copyMatrix(TransformationMatrix matrix, Vec3d scale) {
        if (matrix == null || scale == null || !scale.isFinite()
            || Math.abs(scale.x()) < 1E-9 || Math.abs(scale.y()) < 1E-9 || Math.abs(scale.z()) < 1E-9) return null;
        double[] values = {matrix.m00, matrix.m01, matrix.m02, matrix.m03, matrix.m10, matrix.m11, matrix.m12,
            matrix.m13, matrix.m20, matrix.m21, matrix.m22, matrix.m23, matrix.m30, matrix.m31, matrix.m32, matrix.m33};
        for (double value : values) if (!Double.isFinite(value)) return null;
        if (Math.abs(matrix.m30) + Math.abs(matrix.m31) + Math.abs(matrix.m32) > 1E-8 || Math.abs(matrix.m33 - 1) > 1E-8) return null;
        double determinant = matrix.m00 * (matrix.m11 * matrix.m22 - matrix.m12 * matrix.m21)
            - matrix.m01 * (matrix.m10 * matrix.m22 - matrix.m12 * matrix.m20)
            + matrix.m02 * (matrix.m10 * matrix.m21 - matrix.m11 * matrix.m20);
        if (!(determinant > 1E-8)) return null;
        AnimatedWingGeometry.RigidTransform result = new AnimatedWingGeometry.RigidTransform(matrix.m00, matrix.m01 * scale.x() / scale.y(), matrix.m02 * scale.x() / scale.z(),
            matrix.m10 * scale.y() / scale.x(), matrix.m11, matrix.m12 * scale.y() / scale.z(),
            matrix.m20 * scale.z() / scale.x(), matrix.m21 * scale.z() / scale.y(), matrix.m22,
            new Vec3d(matrix.m03 * scale.x(), matrix.m13 * scale.y(), matrix.m23 * scale.z()));
        double[] copied = {result.m00(), result.m01(), result.m02(), result.m10(), result.m11(), result.m12(),
            result.m20(), result.m21(), result.m22()};
        for (double value : copied) if (!Double.isFinite(value)) return null;
        return result.translation().isFinite() ? result : null;
    }
}
