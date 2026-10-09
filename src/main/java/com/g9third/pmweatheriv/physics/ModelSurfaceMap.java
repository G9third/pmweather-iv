package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import com.g9third.pmweatheriv.model.ParsedModelSnapshot;
import com.g9third.pmweatheriv.model.ParsedModelSnapshot.Mesh;

import static com.g9third.pmweatheriv.physics.ModelGeometryData.AnimationHint;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.BodyWettedAreaEstimate;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.BoundsAccumulator;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.ClassifiedTriangle;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.LiftingFrame;
import static com.g9third.pmweatheriv.physics.ModelTriangleSampling.MAX_RETAINED_TRIANGLES;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.ModelObject;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.ObjectStats;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.RawTriangle;
import static com.g9third.pmweatheriv.physics.ModelTriangleSampling.allocateObjectQuotas;
import static com.g9third.pmweatheriv.physics.ModelAnimationHints.animatedSurfaceHints;
import static com.g9third.pmweatheriv.physics.ModelAnimationHints.animationSignature;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceFrames.attachGeometryFrames;
import static com.g9third.pmweatheriv.physics.LiftingPatchGeometry.buildLiftingPatches;
import static com.g9third.pmweatheriv.physics.BodyPressureGeometry.buildPressurePatches;
import static com.g9third.pmweatheriv.physics.SurfaceClassifier.classify;
import static com.g9third.pmweatheriv.physics.TailSurfaceClassifier.classify;
import static com.g9third.pmweatheriv.physics.ModelAnimationHints.containsAny;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceFrames.defaultLiftingFrame;
import static com.g9third.pmweatheriv.physics.BodyPressureGeometry.estimateBodyWettedArea;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceFrames.finiteUnitOr;
import static com.g9third.pmweatheriv.physics.ModelAnimationHints.ignoredObject;
import static com.g9third.pmweatheriv.physics.ModelAnimationHints.isWaterRudderName;
import static com.g9third.pmweatheriv.physics.ModelAnimationHints.modelLocation;
import static com.g9third.pmweatheriv.physics.ModelAnimationHints.normalize;
import static com.g9third.pmweatheriv.physics.ModelTriangleSampling.objectPriority;
import static com.g9third.pmweatheriv.physics.ModelTriangleSampling.readSampledTriangles;
import static com.g9third.pmweatheriv.physics.TailSurfaceClassifier.repairAllMovingHorizontalTailClassifications;
import static com.g9third.pmweatheriv.physics.TailSurfaceClassifier.repairIncoherentTailClassifications;
import static com.g9third.pmweatheriv.physics.BodyPressureGeometry.robustBodyBounds;
import static com.g9third.pmweatheriv.physics.ModelTriangleSampling.scanObject;
import static com.g9third.pmweatheriv.physics.ModelAnimationHints.shellClosingTransparentObject;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceFrames.symmetryCountsLifting;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceFrames.symmetryCountsPressure;

/**
 * Prepares a deterministic coarse aerodynamic map from the actual IV OBJ model.
 *
 * <p>The visible model uses IV's own parsed-model cache and is interpreted only when a model/sub-definition is first seen.
 * Runtime physics uses a bounded list of direct pressure and lifting patches.
 * Every off-center aerodynamic component is represented by a credible left/right
 * pair unless strong geometric evidence confirms a genuinely one-sided surface.
 * Center components are placed exactly on the whole-model symmetry plane. No
 * directional response table, interpolation table, phased sampling, or in-flight
 * repartitioning exists.</p>
 */
public final class ModelSurfaceMap {
    // Live AircraftState instances retain their immutable PreparedModel directly.
    // Bound only the global loading cache so browsing many content packs/model
    // variants in one long server session cannot retain every parsed OBJ forever.
    private static final int MAX_PREPARED_MODEL_CACHE_ENTRIES = 128;
    private static final Map<String, PreparedModel> CACHE = Collections.synchronizedMap(
        new LinkedHashMap<>(32, 0.75f, true) {
            @Override
            protected boolean removeEldestEntry(Map.Entry<String, PreparedModel> eldest) {
                return size() > MAX_PREPARED_MODEL_CACHE_ENTRIES;
            }
        }
    );

    private ModelSurfaceMap() {
    }

    public static PreparedModel prepare(EntityVehicleF_Physics vehicle, int maximumPressurePatches) {
        return prepare(vehicle, maximumPressurePatches, false);
    }

    public static PreparedModel prepare(
        EntityVehicleF_Physics vehicle,
        int maximumPressurePatches,
        boolean rotorcraft
    ) {
        String location = modelLocation(vehicle);
        String key = location + "|scale=" + ModelCoordinates.scale(vehicle) + "|" + animationSignature(vehicle) + "|" + maximumPressurePatches
            + "|poses=" + AuthoredLiftingSurfacePoses.signature(vehicle) + "|rotor=" + rotorcraft + "|visible="
            + com.g9third.pmweatheriv.model.ModelPhysicalVisibility.signature(vehicle)
            + (rotorcraft ? "" : "|attached=" + AttachedLiftingSurfaces.signature(vehicle));
        PreparedModel cached=CACHE.get(key);
        if (cached!=null) return cached;
        PreparedModel built=build(vehicle,location,maximumPressurePatches,rotorcraft);
        if (!rotorcraft) built=AttachedLiftingSurfaces.append(vehicle, built);
        if (built.modelBased()) CACHE.put(key,built);
        return built;
    }

    private static PreparedModel build(
        EntityVehicleF_Physics vehicle,
        String modelLocation,
        int maximumPressurePatches,
        boolean rotorcraft
    ) {
        try {
            List<Mesh> parsedObjects = ParsedModelSnapshot.load(modelLocation).objects();
            if (parsedObjects == null || parsedObjects.isEmpty()) {
                return PreparedModel.fallback(modelLocation, "no-model-objects");
            }

            Map<String, AnimationHint> animationHints = animatedSurfaceHints(vehicle);
            List<ModelObject> objects = new ArrayList<>();
            BoundsAccumulator fullBoundsAccumulator = new BoundsAccumulator();
            int sourceTriangles = 0;
            Vec3d scale = ModelCoordinates.scale(vehicle);
            for (Mesh unscaled : parsedObjects) {
                Mesh object = unscaled == null ? null : unscaled.scaled(scale.x(),scale.y(),scale.z());
                if (object == null || object.isLines
                    || !com.g9third.pmweatheriv.model.ModelPhysicalVisibility.visible(vehicle, object.name)
                    || ignoredObject(object.name)
                    || (object.isTranslucent && !shellClosingTransparentObject(object.name))) {
                    continue;
                }
                ObjectStats stats = scanObject(object);
                if (stats.validTriangles() <= 0 || !stats.bounds().valid()) {
                    continue;
                }
                String normalizedName = normalize(object.name);
                AnimationHint hint = animationHints.getOrDefault(normalizedName, AnimationHint.NONE);
                ModelObject source = new ModelObject(
                    object,
                    object.name == null ? "model" : object.name,
                    normalizedName,
                    stats,
                    objectPriority(normalizedName, hint, stats)
                );
                objects.add(source);
                sourceTriangles += stats.validTriangles();
                fullBoundsAccumulator.include(stats.bounds());
            }

            Bounds bounds = fullBoundsAccumulator.finish();
            if (objects.isEmpty() || !bounds.valid()) {
                return PreparedModel.fallback(modelLocation, "empty-or-invalid-model");
            }

            Map<ModelObject, Integer> quotas = allocateObjectQuotas(objects);
            List<RawTriangle> triangles = new ArrayList<>(
                Math.min(MAX_RETAINED_TRIANGLES, Math.max(1024, sourceTriangles))
            );
            for (ModelObject object : objects) {
                readSampledTriangles(object, quotas.getOrDefault(object, 0), triangles);
            }
            if (triangles.isEmpty()) {
                return PreparedModel.fallback(modelLocation, "no-retained-triangles");
            }

            EnumMap<SurfaceKind, List<ClassifiedTriangle>> byKind = new EnumMap<>(SurfaceKind.class);
            for (SurfaceKind kind : SurfaceKind.values()) {
                byKind.put(kind, new ArrayList<>());
            }
            for (RawTriangle triangle : triangles) {
                SurfaceKind kind = classify(triangle, bounds, animationHints, rotorcraft);
                ClassifiedTriangle classified = new ClassifiedTriangle(
                    triangle.first(), triangle.second(), triangle.third(),
                    triangle.centroid(), triangle.normal(), triangle.area(), triangle.objectName(), kind
                );
                byKind.get(kind).add(classified);
            }
            if (!rotorcraft) {
                repairAllMovingHorizontalTailClassifications(byKind, bounds);
                repairIncoherentTailClassifications(vehicle, byKind, bounds);
            }
            Bounds bodyBounds = robustBodyBounds(byKind, bounds);
            BodyWettedAreaEstimate wettedAreaEstimate = estimateBodyWettedArea(byKind);
            double pressureWettedArea = wettedAreaEstimate.exposedWettedArea();

            List<PressurePatch> pressurePatches = buildPressurePatches(
                byKind, bounds, bodyBounds, maximumPressurePatches, rotorcraft
            );
            int pressureTarget = pressurePatches.size();
            List<LiftingPatch> liftingPatches = rotorcraft
                ? List.of()
                : attachGeometryFrames(buildLiftingPatches(vehicle, byKind, bounds), byKind, bounds, bodyBounds);

            PreparedModel prepared = new PreparedModel(
                modelLocation,
                true,
                "model-derived-parent-coherent-wing-and-horizontal-tail-frames",
                bounds,
                bodyBounds,
                List.copyOf(pressurePatches),
                List.copyOf(liftingPatches),
                pressureWettedArea,
                sourceTriangles,
                triangles.size(),
                pressureTarget,
                AuthoredLiftingSurfacePoses.bind(vehicle, liftingPatches, byKind, bounds)
            );
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "MODEL_PREPARED model=" + safe(modelLocation)
                    + " modelObjects=" + objects.size()
                    + " sourceTriangles=" + sourceTriangles
                    + " retainedTriangles=" + triangles.size()
                    + " pressureTarget=" + pressureTarget
                    + " pressurePatches=" + pressurePatches.size()
                    + " pressureRawTriangleArea=" + String.format(Locale.ROOT, "%.5f", wettedAreaEstimate.rawTriangleArea())
                    + " pressureWettedArea=" + String.format(Locale.ROOT, "%.5f", pressureWettedArea)
                    + " pressureWettedAreaPolicy=EXPOSED_SILHOUETTE_CAUCHY"
                    + " pressureProjectionDirections=" + wettedAreaEstimate.projectionDirections()
                    + " pressureProjectionResolution=" + wettedAreaEstimate.projectionResolution()
                    + " liftingPatches=" + liftingPatches.size()
                    + " bounds=" + bounds.compact()
                    + " bodyBounds=" + bodyBounds.compact()
                    + " pressureSymmetry=" + symmetryCountsPressure(pressurePatches)
                    + " liftingSymmetry=" + symmetryCountsLifting(liftingPatches)
                    + " patchPlacement=surface-pair-canonical-or-centerline"
                    + " horizontalTailNeutralPolicy=MEASURED_PARENT_FRAME_NO_OFFSET"
                    + " kinds=" + kindCounts(byKind)
            );
            }
            // Per-patch geometry is intentionally not duplicated in the text log.
            // Prepared geometry remains frozen in this object and is captured by PMIVTrace.
            return prepared;
        } catch (RuntimeException exception) {
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "ERROR stage=modelSurfaceMap model=" + safe(modelLocation)
                    + " type=" + exception.getClass().getSimpleName()
                    + " message=" + safe(exception.getMessage())
            );
            }
            return PreparedModel.fallback(modelLocation, exception.getClass().getSimpleName());
        }
    }

    static String safe(String value) {
        if (value == null) {
            return "null";
        }
        return value.replace('\n', ' ').replace('\r', ' ').replace(' ', '_');
    }

    private static String kindCounts(EnumMap<SurfaceKind, List<ClassifiedTriangle>> byKind) {
        StringBuilder result = new StringBuilder();
        for (SurfaceKind kind : SurfaceKind.values()) {
            if (!byKind.get(kind).isEmpty()) {
                if (!result.isEmpty()) {
                    result.append(',');
                }
                result.append(kind.name()).append('=').append(byKind.get(kind).size());
            }
        }
        return result.toString();
    }

    public enum SurfaceKind {
        BODY,
        WING,
        AILERON,
        ELEVON,
        TAILERON,
        CANARD,
        CANARDERON,
        HORIZONTAL_TAIL,
        ELEVATOR,
        VERTICAL_TAIL,
        RUDDER,
        TAIL_BOOM,
        UNDERBODY,
        ENGINE_NACELLE,
        EXTERIOR_PART;

        boolean isBodyPressure() {
            return this == BODY || this == TAIL_BOOM || this == UNDERBODY
                || this == ENGINE_NACELLE || this == EXTERIOR_PART;
        }

        static SurfaceKind fromSpecificName(String name) {
            if (name == null || name.isEmpty()) {
                return null;
            }
            if (isWaterRudderName(name)
                || containsAny(name, "rudderline", "rudder_line", "rudder_link", "rudderlink")) {
                return null;
            }
            if (containsAny(name, "engine", "nacelle", "intake", "cowling", "cowl", "turbine", "jet")
                && containsAny(name, "flap", "door", "reverser")) {
                return ENGINE_NACELLE;
            }
            if (containsAny(name, "taileron")) {
                return TAILERON;
            }
            if (containsAny(name, "elevon", "elevron", "ruddervator")) {
                return ELEVON;
            }
            if (containsAny(name, "flaperon", "aileron")) {
                return AILERON;
            }
            if (containsAny(name, "canard")) {
                return CANARD;
            }
            if (containsAny(name, "elevator", "stabilator")) {
                return ELEVATOR;
            }
            if (containsAny(name,
                "horizontal_stab", "horizontal_tail", "hstab", "tailplane",
                "horizontalstabilizer", "horizontal_stabilizer")) {
                return HORIZONTAL_TAIL;
            }
            if (containsAny(name, "rudder")) {
                return RUDDER;
            }
            if (containsAny(name,
                "vertical_stab", "vertical_tail", "vstab", "tailfin", "tail_fin",
                "verticalstabilizer", "vertical_stabilizer")) {
                return VERTICAL_TAIL;
            }
            if (containsAny(name,
                "wing", "wingtip", "wing_tip", "airfoil", "flap", "slat",
                "spoiler", "speedbrake", "speed_brake")) {
                return WING;
            }
            if (containsAny(name, "tailboom", "tail_boom", "boom")) {
                return TAIL_BOOM;
            }
            if (containsAny(name, "underbody", "belly")) {
                return UNDERBODY;
            }
            if (containsAny(name, "engine", "nacelle", "intake", "cowling", "cowl", "turbine", "jet")) {
                return ENGINE_NACELLE;
            }
            return null;
        }
    }

    public enum SymmetryRole {
        MIRRORED_LEFT,
        MIRRORED_RIGHT,
        CENTERLINE,
        LEFT_ONLY,
        RIGHT_ONLY,
        UNPAIRED
    }

    public record PressurePatch(
        String name,
        Vec3d pointLocal,
        Vec3d normalLocal,
        double area,
        SurfaceKind kind,
        boolean twoSided,
        int symmetryPair,
        SymmetryRole symmetryRole,
        Vec3d windSamplePointLocal
    ) {
        public PressurePatch(String name, Vec3d pointLocal, Vec3d normalLocal, double area,
                             SurfaceKind kind, boolean twoSided, int symmetryPair, SymmetryRole symmetryRole) {
            this(name,pointLocal,normalLocal,area,kind,twoSided,symmetryPair,symmetryRole,pointLocal);
        }
        public PressurePatch(
            String name,
            Vec3d pointLocal,
            Vec3d normalLocal,
            double area,
            SurfaceKind kind,
            int symmetryPair,
            SymmetryRole symmetryRole
        ) {
            this(name, pointLocal, normalLocal, area, kind, false, symmetryPair, symmetryRole);
        }

        public PressurePatch(
            String name,
            Vec3d pointLocal,
            Vec3d normalLocal,
            double area,
            SurfaceKind kind
        ) {
            this(name, pointLocal, normalLocal, area, kind, false, -1, SymmetryRole.UNPAIRED);
        }
    }

    public record LiftingPatch(
        String name,
        SurfaceKind kind,
        Vec3d pointLocal,
        double weight,
        int symmetryPair,
        SymmetryRole symmetryRole,
        int sectionIndex,
        int sectionCount,
        double sectionMinimumRadius,
        double sectionMaximumRadius,
        Vec3d spanLocal,
        Vec3d chordLocal,
        Vec3d normalLocal,
        double frameConfidence,
        Vec3d aerodynamicCenterLocal
    ) {
        public LiftingPatch {
            LiftingFrame fallback = defaultLiftingFrame(kind);
            spanLocal = finiteUnitOr(spanLocal, fallback.spanLocal());
            chordLocal = finiteUnitOr(chordLocal, fallback.chordLocal());
            normalLocal = finiteUnitOr(normalLocal, fallback.normalLocal());
            frameConfidence = Vec3d.clamp(
                Double.isFinite(frameConfidence) ? frameConfidence : 0.0, 0.0, 1.0
            );
            aerodynamicCenterLocal = aerodynamicCenterLocal != null
                && aerodynamicCenterLocal.isFinite()
                    ? aerodynamicCenterLocal
                    : pointLocal;
        }

        public LiftingPatch(
            String name,
            SurfaceKind kind,
            Vec3d pointLocal,
            double weight,
            int symmetryPair,
            SymmetryRole symmetryRole,
            int sectionIndex,
            int sectionCount,
            double sectionMinimumRadius,
            double sectionMaximumRadius
        ) {
            this(name, kind, pointLocal, weight, symmetryPair, symmetryRole,
                sectionIndex, sectionCount, sectionMinimumRadius, sectionMaximumRadius,
                defaultLiftingFrame(kind).spanLocal(),
                defaultLiftingFrame(kind).chordLocal(),
                defaultLiftingFrame(kind).normalLocal(),
                0.0,
                pointLocal);
        }

        public LiftingPatch(
            String name,
            SurfaceKind kind,
            Vec3d pointLocal,
            double weight,
            int symmetryPair,
            SymmetryRole symmetryRole
        ) {
            this(name, kind, pointLocal, weight, symmetryPair, symmetryRole, 0, 1, 0.0, 0.0);
        }

        public LiftingPatch(
            String name,
            SurfaceKind kind,
            Vec3d pointLocal,
            double weight
        ) {
            this(name, kind, pointLocal, weight, -1, SymmetryRole.UNPAIRED);
        }

        public boolean vertical() {
            return kind == SurfaceKind.VERTICAL_TAIL || kind == SurfaceKind.RUDDER;
        }

        public boolean adaptiveSection() {
            return sectionCount > 1;
        }

        public double geometryIncidenceDegrees() {
            if (vertical()) {
                return Math.toDegrees(Math.atan2(chordLocal.x(), chordLocal.z()));
            }
            return Math.toDegrees(Math.atan2(chordLocal.y(), chordLocal.z()));
        }

    }

    public record PreparedModel(
        String modelLocation,
        boolean modelBased,
        String reason,
        Bounds fullBounds,
        Bounds bodyBounds,
        List<PressurePatch> pressurePatches,
        List<LiftingPatch> liftingPatches,
        double pressureWettedArea,
        int sourceTriangles,
        int retainedTriangles,
        int pressureTarget,
        Map<String, AuthoredLiftingSurfacePoses.Binding> authoredPoseBindings
    ) {
        public PreparedModel {
            pressurePatches = List.copyOf(pressurePatches);
            liftingPatches = List.copyOf(liftingPatches);
            authoredPoseBindings = Map.copyOf(authoredPoseBindings);
        }

        public PreparedModel(String modelLocation, boolean modelBased, String reason, Bounds fullBounds,
                             Bounds bodyBounds, List<PressurePatch> pressurePatches, List<LiftingPatch> liftingPatches,
                             double pressureWettedArea, int sourceTriangles, int retainedTriangles, int pressureTarget) {
            this(modelLocation, modelBased, reason, fullBounds, bodyBounds, pressurePatches, liftingPatches,
                pressureWettedArea, sourceTriangles, retainedTriangles, pressureTarget, Map.of());
        }
        private static PreparedModel fallback(String modelLocation, String reason) {
            Bounds bounds = new Bounds(new Vec3d(-1.0, -0.75, -1.5), new Vec3d(1.0, 0.75, 1.5), true);
            return new PreparedModel(
                modelLocation, false, reason, bounds, bounds, List.of(), List.of(), 0.0, 0, 0, 8
            );
        }
    }

    public record Bounds(Vec3d minimum, Vec3d maximum, boolean valid) {
        public double spanX() {
            return valid ? Math.max(0.01, maximum.x() - minimum.x()) : 0.0;
        }

        public double spanY() {
            return valid ? Math.max(0.01, maximum.y() - minimum.y()) : 0.0;
        }

        public double spanZ() {
            return valid ? Math.max(0.01, maximum.z() - minimum.z()) : 0.0;
        }

        public Vec3d normalized(Vec3d point) {
            return new Vec3d(
                2.0 * (point.x() - minimum.x()) / spanX() - 1.0,
                2.0 * (point.y() - minimum.y()) / spanY() - 1.0,
                2.0 * (point.z() - minimum.z()) / spanZ() - 1.0
            );
        }

        public String compact() {
            return String.format(Locale.ROOT, "%.3fx%.3fx%.3f", spanX(), spanY(), spanZ());
        }
    }

}
