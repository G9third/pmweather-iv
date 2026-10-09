package com.g9third.pmweatheriv.sable;

import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.jsondefs.JSONAnimatedObject;
import minecrafttransportsimulator.jsondefs.JSONAnimationDefinition;
import com.g9third.pmweatheriv.model.ParsedModelSnapshot;
import com.g9third.pmweatheriv.model.ParsedModelSnapshot.Mesh;
import com.g9third.pmweatheriv.model.ModelPhysicalVisibility;

/**
 * Builds a content-pack-generic, coarse Sable collision shell from the rendered
 * IV vehicle OBJ.
 *
 * <p>This deliberately does not attempt to upload the raw triangle mesh to
 * Rapier. Sable 2.0.3's public/native contraption collision path is the
 * LevelCollider voxel/box path, and PMWeather-IV relies on that path for shared
 * terrain/Aeronautics contact telemetry. The OBJ is therefore rasterized once
 * into a deterministic shell of local cuboids, then adjacent occupied cells are
 * greedily merged. The result approximates curved fuselages and tapered wings
 * while remaining a native Sable LevelCollider once mounted by
 * {@link SableCompoundCollider}.</p>
 *
 * <p>Geometry with a real relative transform is mounted separately from the static shell. Damage-replacement geometry,
 * render-only geometry and wheeled landing-gear objects are excluded from the
 * static parent shell. Visibility/state-only animation remains physical, so an
 * intact wing cannot disappear from the Sable hull merely because IV toggles its
 * damaged replacement mesh. The parent airframe is never spatially cut around
 * wheel volumes or support polygons. Static skid/float/pontoon mesh is always
 * retained as rigid BODY collision and may additionally provide exact support
 * points; IV ground-device availability never removes that honest airframe
 * geometry.</p>
 */
public final class SableModelCollisionHull {
    private static final int FLOATS_PER_VERTEX = 3;
    private static final int FLOATS_PER_TRIANGLE = FLOATS_PER_VERTEX * 3;
    private static final double MIN_TRIANGLE_AREA = 1.0E-7;
    private static final double AXIS_EPSILON_SQUARED = 1.0E-18;
    private static final double MIN_RESOLUTION = 0.20;
    private static final double MAX_BASE_RESOLUTION = 0.50;
    private static final double MAX_RETRY_RESOLUTION = 0.85;
    private static final double TARGET_CELLS_ACROSS_LONGEST_SPAN = 160.0;
    private static final double CELL_OVERLAP_FACTOR = 1.04;
    private static final int MAX_SHELL_CELLS = 48_000;
    private static final long MAX_TRIANGLE_CELL_TESTS = 12_000_000L;
    private static final int MAX_RASTER_ATTEMPTS = 4;
    private static final int MAX_MERGED_BOXES = 24_000;
    private static final double STATIC_SUPPORT_LOW_BAND_METERS = 0.05;
    private static final int STATIC_SUPPORT_EXTREMA_DIRECTIONS = 16;
    private static final int MAX_STATIC_SUPPORT_POINTS = 16;
    /**
     * Small fixed padding around an authored rolling ground-device envelope when
     * removing baked wheel/tread pixels from the parent OBJ shell. The envelope
     * is derived only from immutable wheelbasePoint + device dimensions, never
     * live suspension/localOffset, so ordinary wheel motion cannot change BODY
     * topology or force a compound rebuild.
     */
    private static final double ROLLING_GEAR_HULL_EXCLUSION_WIDTH_MARGIN_METERS = 0.10;
    private static final double ROLLING_GEAR_HULL_SUPPORT_VERTICAL_MARGIN_METERS = 0.25;
    private static final double ROLLING_GEAR_HULL_SUPPORT_LONGITUDINAL_MARGIN_METERS = 0.35;
    // A live SableCompoundCollider retains its PreparedHull. This cache exists
    // only to avoid reparsing identical models during spawn/loading, so it can
    // be bounded without invalidating active aircraft.
    private static final int MAX_PREPARED_HULL_CACHE_ENTRIES = 64;
    private static final Map<String, PreparedHull> CACHE = Collections.synchronizedMap(
        new LinkedHashMap<>(16, 0.75f, true) {
            @Override
            protected boolean removeEldestEntry(Map.Entry<String, PreparedHull> eldest) {
                return size() > MAX_PREPARED_HULL_CACHE_ENTRIES;
            }
        }
    );
    /**
     * Geometry-only body-pressure shells for ordinary ground vehicles.  This cache is deliberately
     * separate from collision geometry: aerodynamic preparation retains exterior animated body
     * panels and does not carve wheel-support corridors out of the pressure silhouette.  Failed
     * preparations are never cached, allowing a freshly-created IV entity to be retried once its
     * model state is fully materialized.
     */
    private static final Map<String, PreparedHull> AERODYNAMIC_BODY_CACHE = Collections.synchronizedMap(
        new LinkedHashMap<>(16, 0.75f, true) {
            @Override
            protected boolean removeEldestEntry(Map.Entry<String, PreparedHull> eldest) {
                return size() > MAX_PREPARED_HULL_CACHE_ENTRIES;
            }
        }
    );

    private static final Map<String, List<PreparedMovingObjectHull>> MOVING_EXTERIOR_CACHE =
        Collections.synchronizedMap(new LinkedHashMap<>(16, .75f, true) {
            @Override protected boolean removeEldestEntry(Map.Entry<String, List<PreparedMovingObjectHull>> e) {
                return size() > MAX_PREPARED_HULL_CACHE_ENTRIES;
            }
        });

    /** Immutable per-object voxels; membership and poses are evaluated separately on the owner tick. */
    public static List<PreparedMovingObjectHull> prepareMovingExteriorHulls(EntityVehicleF_Physics vehicle) {
        return prepareMovingExteriorHulls(vehicle, null);
    }

    static List<PreparedMovingObjectHull> prepareMovingExteriorHulls(EntityVehicleF_Physics vehicle,
                                                                  String animationDefinitions) {
        if (vehicle == null || !canAttempt(vehicle)) return List.of();
        String location = modelLocation(vehicle);
        String key = movingExteriorCacheKey(location, vehicle.scale.x, vehicle.scale.y,
            vehicle.scale.z, animationDefinitions == null ? animationSignature(vehicle) : animationDefinitions);
        List<PreparedMovingObjectHull> cached = MOVING_EXTERIOR_CACHE.get(key);
        if (cached != null) return cached;
        List<PreparedMovingObjectHull> built = new ArrayList<>();
        try {
            Map<String, JSONAnimatedObject> definitions = animatedDefinitions(vehicle);
            AnimatedObjectClassification classification = classifyAnimatedObjects(definitions);
            List<Mesh> meshes = ParsedModelSnapshot.load(location).objects();
            // An empty early parser result must remain retriable. A completed model with
            // no eligible moving objects is safe to cache under its definition signature.
            if (meshes.stream().noneMatch(mesh -> mesh != null && !mesh.isLines
                && mesh.positions().remaining() >= FLOATS_PER_TRIANGLE)) return List.of();
            MovingRasterBudget budget = new MovingRasterBudget();
            int totalBoxes = 0;
            for (Mesh mesh : meshes) {
                if (totalBoxes >= MAX_MERGED_BOXES || budget.remaining <= 0) break;
                if (!eligibleMovingExterior(mesh, definitions, classification)) continue;
                PreparedMovingObjectHull hull = buildMovingObjectHull(mesh,
                    finiteScale(vehicle.scale.x), finiteScale(vehicle.scale.y), finiteScale(vehicle.scale.z), budget);
                if (hull == null || totalBoxes + hull.boxes().size() > MAX_MERGED_BOXES) continue;
                totalBoxes += hull.boxes().size(); built.add(hull);
            }
        } catch (RuntimeException failure) {
            if (PMIVObserver.loggingEnabled()) PMIVObserver.log("SABLE_MOVING_MODEL_HULL_UNAVAILABLE model="
                + safe(location) + " type=" + failure.getClass().getSimpleName());
            return List.of();
        }
        List<PreparedMovingObjectHull> result = List.copyOf(built);
        MOVING_EXTERIOR_CACHE.put(key, result);
        return result;
    }

    private static boolean eligibleMovingExterior(Mesh mesh, Map<String, JSONAnimatedObject> definitions,
                                                   AnimatedObjectClassification classification) {
        if (mesh == null || mesh.isLines) return false;
        String name = normalize(mesh.name);
        return classification.physicalTransformNames().contains(name)
            && !classification.damageReplacementNames().contains(name)
            && aerodynamicIgnoreReason(name, mesh.isTranslucent) == IgnoreReason.NONE
            && rigidAnimationChain(name, definitions, new HashSet<>());
    }

    static String movingExteriorCacheKey(String modelLocation, double sx, double sy, double sz,
                                         String animationDefinitions) {
        return modelLocation + "|scale=" + stable(sx) + ',' + stable(sy) + ',' + stable(sz)
            + "|anim=" + animationDefinitions;
    }

    private static boolean rigidAnimationChain(String name, Map<String, JSONAnimatedObject> definitions,
                                               Set<String> visiting) {
        JSONAnimatedObject object = definitions.get(name);
        if (object == null || !visiting.add(name)) return false;
        if (object.animations != null) for (JSONAnimationDefinition animation : object.animations)
            if (animation != null && animation.animationType != null
                && normalize(animation.animationType.name()).contains("scaling")) return false;
        return object.applyAfter == null || object.applyAfter.isBlank()
            || rigidAnimationChain(normalize(object.applyAfter), definitions, visiting);
    }

    private static final class MovingRasterBudget {
        private long remaining = MAX_TRIANGLE_CELL_TESTS * MAX_RASTER_ATTEMPTS;
    }

    private static PreparedMovingObjectHull buildMovingObjectHull(Mesh mesh, double sx, double sy, double sz) {
        return buildMovingObjectHull(mesh, sx, sy, sz, new MovingRasterBudget());
    }

    private static PreparedMovingObjectHull buildMovingObjectHull(Mesh mesh, double sx, double sy, double sz,
                                                                 MovingRasterBudget budget) {
        List<Triangle> triangles = new ArrayList<>(); Bounds bounds = new Bounds();
        readTriangles(mesh, sx, sy, sz, triangles, bounds);
        if (triangles.isEmpty() || !bounds.validSurface()) return null;
        double span = Math.max(bounds.spanX(), Math.max(bounds.spanY(), bounds.spanZ()));
        double resolution = clamp(span / TARGET_CELLS_ACROSS_LONGEST_SPAN, MIN_RESOLUTION, MAX_BASE_RESOLUTION);
        Rasterized raster = null;
        for (int attempt = 0; attempt < MAX_RASTER_ATTEMPTS; ++attempt) {
            raster = rasterize(triangles, resolution, Math.min(MAX_TRIANGLE_CELL_TESTS, budget.remaining));
            budget.remaining -= raster.cellTests();
            // A wheel corridor is defined in the parent's reference frame. Carving it
            // into a moving panel would move that hole with the panel. Retain its real
            // exterior and exclude rolling hardware through object eligibility instead.
            if (!raster.overBudget()) break;
            if (budget.remaining <= 0) break;
            resolution = Math.min(MAX_RETRY_RESOLUTION, resolution * 1.28);
        }
        if (raster == null || raster.overBudget() || raster.cells().isEmpty()) return null;
        List<HullBox> boxes = mergeCells(raster.cells(), resolution);
        if (boxes.isEmpty() || boxes.size() > MAX_MERGED_BOXES) return null;
        boxes.sort(HullBox.COMPARATOR);
        return new PreparedMovingObjectHull(mesh.name, resolution, triangles.size(), List.copyOf(boxes));
    }

    private SableModelCollisionHull() {
    }

    /** Cheap readiness check; actual parsing remains cached and is done only on activation. */
    public static boolean canAttempt(EntityVehicleF_Physics vehicle) {
        String location = modelLocation(vehicle);
        return location != null && location.toLowerCase(Locale.ROOT).endsWith(".obj");
    }

    public static PreparedHull prepare(EntityVehicleF_Physics vehicle) {
        return prepareWithKey(vehicle, null);
    }

    static PreparedHull prepareWithKey(EntityVehicleF_Physics vehicle, String resolvedKey) {
        if (vehicle == null) {
            return PreparedHull.fallback("missing", "null-vehicle");
        }
        String location = modelLocation(vehicle);
        if (location == null || !location.toLowerCase(Locale.ROOT).endsWith(".obj")) {
            return PreparedHull.fallback(location == null ? "missing" : location, "non-obj-model");
        }
        // Rigid skid/float/pontoon geometry is part of the airframe collision
        // shell regardless of whether IV also exposes PartGroundDevice support
        // stations.  Exact support points remain an additional contact solver,
        // not a reason to remove honest rigid-body collision geometry.
        boolean includeStaticGroundSupport = true;
        String key = resolvedKey == null ? collisionCacheKey(vehicle) : resolvedKey;
        PreparedHull cached=CACHE.get(key);
        if (cached!=null && cached.usable()) return cached;
        PreparedHull built=build(vehicle,location,includeStaticGroundSupport);
        if (built.usable()) CACHE.put(key,built);
        return built;
    }

    /** All static-shell preparation inputs; compound membership must follow these changes too. */
    static String collisionCacheKey(EntityVehicleF_Physics vehicle) {
        if (vehicle == null) return "missing";
        return collisionCacheKey(vehicle, animationSignature(vehicle));
    }

    static String collisionCacheKey(EntityVehicleF_Physics vehicle, String animationDefinitions) {
        if (vehicle == null) return "missing";
        return modelLocation(vehicle)
            + "|scale=" + stable(vehicle.scale.x) + ',' + stable(vehicle.scale.y) + ',' + stable(vehicle.scale.z)
            + "|anim=" + animationDefinitions
            + "|rollingGear=" + rollingGroundDeviceEnvelopeSignature(vehicle)
            + "|visible=" + ModelPhysicalVisibility.signature(vehicle);
    }

    /**
     * Prepare the Sable-derived exterior shell used only for PMAero body pressure on native-IV
     * ground vehicles.  This intentionally shares the OBJ parser/raster/merge machinery with the
     * aircraft Sable model, but not collision-only omissions: physically animated exterior panels
     * remain in their authored base pose and wheel support corridors are not cut from the shell.
     * Actual wheel/tire/prop/rotor meshes and render-only interior/effect objects are still omitted.
     *
     * <p>Only successful results are cached.  A transient early-entity failure is therefore
     * retriable and cannot permanently latch a vehicle onto PMIV's bounding-box fallback.</p>
     */
    public static PreparedHull prepareAerodynamicBody(EntityVehicleF_Physics vehicle) {
        if (vehicle == null) {
            return PreparedHull.fallback("missing", "null-vehicle");
        }
        String location = modelLocation(vehicle);
        if (location == null || !location.toLowerCase(Locale.ROOT).endsWith(".obj")) {
            return PreparedHull.fallback(location == null ? "missing" : location, "non-obj-model");
        }
        String key = location
            + "|scale=" + stable(vehicle.scale.x) + ',' + stable(vehicle.scale.y) + ',' + stable(vehicle.scale.z)
            + "|visible=" + ModelPhysicalVisibility.signature(vehicle);
        PreparedHull cached = AERODYNAMIC_BODY_CACHE.get(key);
        if (cached != null && cached.usable()) {
            return cached;
        }
        PreparedHull built = buildAerodynamicBody(vehicle, location);
        if (built.usable()) {
            AERODYNAMIC_BODY_CACHE.put(key, built);
        }
        return built;
    }

    private static PreparedHull buildAerodynamicBody(
        EntityVehicleF_Physics vehicle,
        String modelLocation
    ) {
        try {
            List<Mesh> parsed = ParsedModelSnapshot.load(modelLocation).objects();
            if (parsed == null || parsed.isEmpty()) {
                return PreparedHull.fallback(modelLocation, "missing-or-empty-model");
            }

            AnimatedObjectClassification animationObjects = classifyAnimatedObjects(vehicle);
            List<Triangle> triangles = new ArrayList<>();
            Bounds bounds = new Bounds();
            int retainedObjects = 0;
            int skippedAnimatedObjects = 0;
            int skippedPhysicalAnimatedObjects = 0;
            int skippedDamageReplacementObjects = 0;
            int retainedAnimatedExteriorObjects = 0;
            int skippedGearObjects = 0;
            int skippedRenderObjects = 0;

            double sx = finiteScale(vehicle.scale.x);
            double sy = finiteScale(vehicle.scale.y);
            double sz = finiteScale(vehicle.scale.z);

            for (Mesh object : parsed) {
                if (object == null || object.isLines) {
                    ++skippedRenderObjects;
                    continue;
                }
                String normalized = normalize(object.name);
                if (!ModelPhysicalVisibility.visible(vehicle, object.name)) {
                    ++skippedRenderObjects;
                    continue;
                }
                boolean damageReplacement = animationObjects.damageReplacementNames().contains(normalized);
                if (damageReplacement) {
                    ++skippedAnimatedObjects;
                    if (damageReplacement) ++skippedDamageReplacementObjects;
                    continue;
                }
                IgnoreReason ignore = bodyPressureIgnoreReason(object, animationObjects);
                if (ignore != IgnoreReason.NONE) {
                    if (ignore == IgnoreReason.GEAR_OR_MOVING_HARDWARE) ++skippedGearObjects;
                    else ++skippedRenderObjects;
                    continue;
                }
                if (animationObjects.physicalTransformNames().contains(normalized)
                    || animationObjects.knownAnimatedNames().contains(normalized)) {
                    // Body pressure wants the complete exposed vehicle envelope.  Unlike a rigid
                    // collision parent, a door/hood/body panel is still aerodynamic even though IV
                    // may animate it.  The authored OBJ pose is the generic geometry reference.
                    ++retainedAnimatedExteriorObjects;
                }

                int before = triangles.size();
                readTriangles(object, sx, sy, sz, triangles, bounds);
                if (triangles.size() > before) ++retainedObjects;
            }

            if (triangles.isEmpty() || !bounds.valid()) {
                return PreparedHull.fallback(modelLocation, "no-aerodynamic-exterior-triangles");
            }

            double longestSpan = Math.max(bounds.spanX(), Math.max(bounds.spanY(), bounds.spanZ()));
            double resolution = clamp(
                longestSpan / TARGET_CELLS_ACROSS_LONGEST_SPAN,
                MIN_RESOLUTION, MAX_BASE_RESOLUTION
            );
            Rasterized rasterized = null;
            for (int attempt = 0; attempt < MAX_RASTER_ATTEMPTS; ++attempt) {
                rasterized = rasterize(triangles, resolution);
                if (!rasterized.overBudget()) break;
                resolution = Math.min(MAX_RETRY_RESOLUTION, resolution * 1.28);
            }
            if (rasterized == null || rasterized.cells().isEmpty()) {
                return PreparedHull.fallback(modelLocation, "aerodynamic-rasterization-empty");
            }
            if (rasterized.overBudget()) {
                return PreparedHull.fallback(modelLocation, "aerodynamic-rasterization-budget-exceeded");
            }
            List<HullBox> boxes = mergeCells(rasterized.cells(), resolution);
            if (boxes.isEmpty() || boxes.size() > MAX_MERGED_BOXES) {
                return PreparedHull.fallback(modelLocation, boxes.isEmpty()
                    ? "aerodynamic-merged-shell-empty" : "aerodynamic-merged-shell-budget-exceeded");
            }

            PreparedHull result = new PreparedHull(
                modelLocation, true, "obj-ground-body-pressure-voxel-shell", resolution,
                retainedObjects, triangles.size(), rasterized.cells().size(), rasterized.cellTests(),
                boxes.size(), skippedAnimatedObjects, skippedPhysicalAnimatedObjects,
                skippedDamageReplacementObjects, retainedAnimatedExteriorObjects, skippedGearObjects,
                skippedRenderObjects, 0, List.copyOf(boxes), List.of(),
                triangles.stream().map(t -> new SurfaceTriangle(
                    new com.g9third.pmweatheriv.physics.Vec3d(t.a().x(),t.a().y(),t.a().z()),
                    new com.g9third.pmweatheriv.physics.Vec3d(t.b().x(),t.b().y(),t.b().z()),
                    new com.g9third.pmweatheriv.physics.Vec3d(t.c().x(),t.c().y(),t.c().z()))).toList()
            );
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "SABLE_MODEL_AERODYNAMIC_HULL model=" + safe(modelLocation)
                    + " usable=true"
                    + " mode=OBJ_EXTERIOR_TO_PMWEATHER_BODY_PRESSURE_SHELL"
                    + " resolution=" + result.resolution()
                    + " retainedObjects=" + result.retainedObjects()
                    + " sourceTriangles=" + result.sourceTriangles()
                    + " shellCells=" + result.shellCells()
                    + " mergedCuboids=" + result.mergedBoxes()
                    + " triangleCellTests=" + result.triangleCellTests()
                    + " retainedAnimatedExteriorObjects=" + retainedAnimatedExteriorObjects
                    + " skippedDamageReplacementObjects=" + result.skippedDamageReplacementObjects()
                    + " skippedGearObjects=" + result.skippedGearObjects()
                    + " skippedRenderObjects=" + result.skippedRenderObjects()
                    + " rollingGearCellsExcluded=0"
                    + " longestSpan=" + longestSpan
                    + " geometryOnly=true"
            );
            }
            return result;
        } catch (RuntimeException exception) {
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "ERROR stage=sableModelAerodynamicHull model=" + safe(modelLocation)
                    + " type=" + exception.getClass().getSimpleName()
                    + " message=" + safe(exception.getMessage())
            );
            }
            return PreparedHull.fallback(modelLocation, exception.getClass().getSimpleName());
        }
    }

    private static boolean animatedObjectIsDamageReplacement(JSONAnimatedObject object) {
        if (object == null) return false;
        String name = normalize(object.objectName);
        if (containsAny(name, "damage", "damaged", "destroy", "destroyed", "wreck")) {
            return true;
        }
        if (object.animations == null) return false;
        for (JSONAnimationDefinition animation : object.animations) {
            if (animation == null || animation.animationType == null || animation.variable == null) {
                continue;
            }
            if (!normalize(animation.animationType.name()).contains("visibility")
                || !normalize(animation.variable).contains("damage")) {
                continue;
            }
            double min = animation.clampMin;
            double max = animation.clampMax;
            // A damage-gated replacement is not visible at intact damage=0.
            if (Double.isFinite(min) && Double.isFinite(max)
                && max > min + 1.0E-8
                && (0.0 < min - 1.0E-8 || 0.0 > max + 1.0E-8)) {
                return true;
            }
        }
        return false;
    }

    private static PreparedHull build(
        EntityVehicleF_Physics vehicle,
        String modelLocation,
        boolean includeStaticGroundSupport
    ) {
        try {
            List<Mesh> parsed = ParsedModelSnapshot.load(modelLocation).objects();
            if (parsed == null || parsed.isEmpty()) {
                return PreparedHull.fallback(modelLocation, "missing-or-empty-model");
            }

            AnimatedObjectClassification animationObjects = classifyAnimatedObjects(vehicle);
            List<Triangle> triangles = new ArrayList<>();
            List<NamedSupportTriangles> staticSupportTriangles = new ArrayList<>();
            Bounds bounds = new Bounds();
            int retainedObjects = 0;
            int skippedAnimatedObjects = 0;
            int skippedPhysicalAnimatedObjects = 0;
            int skippedDamageReplacementObjects = 0;
            int retainedVisibilityOnlyAnimatedObjects = 0;
            int skippedGearObjects = 0;
            int skippedRenderObjects = 0;

            double sx = finiteScale(vehicle.scale.x);
            double sy = finiteScale(vehicle.scale.y);
            double sz = finiteScale(vehicle.scale.z);

            for (Mesh object : parsed) {
                if (object == null || object.isLines) {
                    ++skippedRenderObjects;
                    continue;
                }
                String normalized = normalize(object.name);
                if (!ModelPhysicalVisibility.visible(vehicle, object.name)) {
                    ++skippedRenderObjects;
                    continue;
                }
                boolean physicalAnimated = animationObjects.physicalTransformNames().contains(normalized);
                boolean damageReplacement = animationObjects.damageReplacementNames().contains(normalized);
                if (physicalAnimated || damageReplacement) {
                    ++skippedAnimatedObjects;
                    if (physicalAnimated) ++skippedPhysicalAnimatedObjects;
                    if (damageReplacement) ++skippedDamageReplacementObjects;
                    continue;
                }
                if (animationObjects.knownAnimatedNames().contains(normalized)) {
                    // Visibility/state-only animation does not move the mesh relative to
                    // the rigid body. Keep intact wings/tails in the physical shell.
                    ++retainedVisibilityOnlyAnimatedObjects;
                }
                IgnoreReason ignore = staticCollisionIgnoreReason(
                    object, animationObjects, includeStaticGroundSupport
                );
                if (ignore != IgnoreReason.NONE) {
                    if (ignore == IgnoreReason.GEAR_OR_MOVING_HARDWARE) {
                        ++skippedGearObjects;
                    } else {
                        ++skippedRenderObjects;
                    }
                    continue;
                }

                List<Triangle> objectTriangles = new ArrayList<>();
                readTriangles(object, sx, sy, sz, objectTriangles, bounds);
                if (!objectTriangles.isEmpty()) {
                    triangles.addAll(objectTriangles);
                    ++retainedObjects;
                    if (includeStaticGroundSupport && isStaticGroundSupport(normalized)) {
                        staticSupportTriangles.add(new NamedSupportTriangles(
                            normalized.isEmpty() ? "unnamed-static-support" : normalized,
                            List.copyOf(objectTriangles)
                        ));
                    }
                }
            }

            if (triangles.isEmpty() || !bounds.valid()) {
                return PreparedHull.fallback(modelLocation, "no-static-exterior-triangles");
            }

            double longestSpan = Math.max(bounds.spanX(), Math.max(bounds.spanY(), bounds.spanZ()));
            double resolution = clamp(
                longestSpan / TARGET_CELLS_ACROSS_LONGEST_SPAN,
                MIN_RESOLUTION,
                MAX_BASE_RESOLUTION
            );

            List<RollingGroundDeviceEnvelope> rollingGearEnvelopes =
                rollingGroundDeviceEnvelopes(vehicle);
            int rollingGearCellsExcluded = 0;

            Rasterized rasterized = null;
            for (int attempt = 0; attempt < MAX_RASTER_ATTEMPTS; ++attempt) {
                rasterized = rasterize(triangles, resolution);
                if (!rasterized.overBudget() && !rollingGearEnvelopes.isEmpty()) {
                    int cellsBeforeRollingGearExclusion = rasterized.cells().size();
                    rasterized = excludeRollingGroundDeviceCells(
                        rasterized, resolution, rollingGearEnvelopes
                    );
                    rollingGearCellsExcluded = Math.max(
                        0, cellsBeforeRollingGearExclusion - rasterized.cells().size()
                    );
                }
                if (!rasterized.overBudget()) {
                    break;
                }
                resolution = Math.min(MAX_RETRY_RESOLUTION, resolution * 1.28);
            }
            if (rasterized == null || rasterized.cells().isEmpty()) {
                return PreparedHull.fallback(modelLocation, "rasterization-empty");
            }
            if (rasterized.overBudget()) {
                return PreparedHull.fallback(modelLocation, "rasterization-budget-exceeded");
            }

            List<HullBox> boxes = mergeCells(rasterized.cells(), resolution);
            if (boxes.isEmpty() || boxes.size() > MAX_MERGED_BOXES) {
                return PreparedHull.fallback(modelLocation, boxes.isEmpty()
                    ? "merged-shell-empty" : "merged-shell-budget-exceeded");
            }

            List<StaticSupportPoint> staticSupportPoints =
                includeStaticGroundSupport
                    ? extractStaticSupportPoints(staticSupportTriangles)
                    : List.of();

            PreparedHull result = new PreparedHull(
                modelLocation,
                true,
                "obj-surface-voxel-shell",
                resolution,
                retainedObjects,
                triangles.size(),
                rasterized.cells().size(),
                rasterized.cellTests(),
                boxes.size(),
                skippedAnimatedObjects,
                skippedPhysicalAnimatedObjects,
                skippedDamageReplacementObjects,
                retainedVisibilityOnlyAnimatedObjects,
                skippedGearObjects,
                skippedRenderObjects,
                rollingGearCellsExcluded,
                List.copyOf(boxes),
                staticSupportPoints
            );
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "SABLE_MODEL_COLLISION_HULL model=" + safe(modelLocation)
                    + " usable=true"
                    + " mode=OBJ_SURFACE_TO_NATIVE_SABLE_LEVEL_COLLIDER"
                    + " resolution=" + result.resolution()
                    + " retainedObjects=" + result.retainedObjects()
                    + " sourceTriangles=" + result.sourceTriangles()
                    + " shellCells=" + result.shellCells()
                    + " mergedCuboids=" + result.mergedBoxes()
                    + " triangleCellTests=" + result.triangleCellTests()
                    + " skippedAnimatedObjects=" + result.skippedAnimatedObjects()
                    + " skippedPhysicalAnimatedObjects=" + result.skippedPhysicalAnimatedObjects()
                    + " skippedDamageReplacementObjects=" + result.skippedDamageReplacementObjects()
                    + " retainedVisibilityOnlyAnimatedObjects=" + result.retainedVisibilityOnlyAnimatedObjects()
                    + " skippedGearObjects=" + result.skippedGearObjects()
                    + " skippedRenderObjects=" + result.skippedRenderObjects()
                    + " rollingGearCellsExcluded=" + result.rollingGearCellsExcluded()
                    + " longestSpan=" + longestSpan
                    + " rawTriangleMeshCollider=false"
                    + " landingGearIncluded=false"
                    + " staticGroundSupportIncluded=" + includeStaticGroundSupport
                    + " staticGroundSupportPointCount=" + result.staticSupportPoints().size()
                    + " staticGroundSupportMinY=" + minimumStaticSupportY(result.staticSupportPoints())
                    + " staticGroundSupportObjects=" + staticSupportObjectNames(result.staticSupportPoints())
            );
            }
            return result;
        } catch (RuntimeException exception) {
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "ERROR stage=sableModelCollisionHull model=" + safe(modelLocation)
                    + " type=" + exception.getClass().getSimpleName()
                    + " message=" + safe(exception.getMessage())
            );
            }
            return PreparedHull.fallback(modelLocation, exception.getClass().getSimpleName());
        }
    }

    private static void readTriangles(
        Mesh object,
        double sx,
        double sy,
        double sz,
        List<Triangle> output,
        Bounds bounds
    ) {
        FloatBuffer buffer = object.positions();
        buffer.rewind();
        while (buffer.remaining() >= FLOATS_PER_TRIANGLE) {
            Vec3 first = readPosition(buffer, sx, sy, sz);
            Vec3 second = readPosition(buffer, sx, sy, sz);
            Vec3 third = readPosition(buffer, sx, sy, sz);
            Vec3 ab = second.subtract(first);
            Vec3 ac = third.subtract(first);
            double twiceAreaSquared = ab.cross(ac).lengthSquared();
            if (!first.finite() || !second.finite() || !third.finite()
                || !Double.isFinite(twiceAreaSquared)
                || twiceAreaSquared < 4.0 * MIN_TRIANGLE_AREA * MIN_TRIANGLE_AREA) {
                continue;
            }
            output.add(new Triangle(first, second, third));
            bounds.include(first);
            bounds.include(second);
            bounds.include(third);
        }
    }

    private static Vec3 readPosition(FloatBuffer buffer, double sx, double sy, double sz) {
        return new Vec3(buffer.get() * sx, buffer.get() * sy, buffer.get() * sz);
    }

    private static Rasterized rasterize(List<Triangle> triangles, double resolution) {
        return rasterize(triangles, resolution, MAX_TRIANGLE_CELL_TESTS);
    }

    private static Rasterized rasterize(List<Triangle> triangles, double resolution, long maxCellTests) {
        Set<GridCell> cells = new HashSet<>();
        long cellTests = 0L;
        double half = resolution * 0.5 * CELL_OVERLAP_FACTOR;

        for (Triangle triangle : triangles) {
            int minX = floorGrid(Math.min(triangle.a().x(), Math.min(triangle.b().x(), triangle.c().x())), resolution);
            int minY = floorGrid(Math.min(triangle.a().y(), Math.min(triangle.b().y(), triangle.c().y())), resolution);
            int minZ = floorGrid(Math.min(triangle.a().z(), Math.min(triangle.b().z(), triangle.c().z())), resolution);
            int maxX = floorGrid(Math.max(triangle.a().x(), Math.max(triangle.b().x(), triangle.c().x())), resolution);
            int maxY = floorGrid(Math.max(triangle.a().y(), Math.max(triangle.b().y(), triangle.c().y())), resolution);
            int maxZ = floorGrid(Math.max(triangle.a().z(), Math.max(triangle.b().z(), triangle.c().z())), resolution);

            long candidateCount = (long) (maxX - minX + 1)
                * (long) (maxY - minY + 1)
                * (long) (maxZ - minZ + 1);
            if (candidateCount <= 0L || cellTests + candidateCount > maxCellTests) {
                return new Rasterized(cells, cellTests, true);
            }

            for (int x = minX; x <= maxX; ++x) {
                double centerX = (x + 0.5) * resolution;
                for (int y = minY; y <= maxY; ++y) {
                    double centerY = (y + 0.5) * resolution;
                    for (int z = minZ; z <= maxZ; ++z) {
                        ++cellTests;
                        double centerZ = (z + 0.5) * resolution;
                        if (triangleIntersectsBox(triangle, centerX, centerY, centerZ, half)) {
                            cells.add(new GridCell(x, y, z));
                            if (cells.size() > MAX_SHELL_CELLS) {
                                return new Rasterized(cells, cellTests, true);
                            }
                        }
                    }
                }
            }
        }
        return new Rasterized(cells, cellTests, false);
    }


    /**
     * Removes raster cells that overlap a small frozen support corridor around a
     * real authored wheel/tread station. A centre-only test can retain a 0.20 m
     * shell cell that partially overlaps the corridor; that exact failure let the
     * MiG nose/body shell touch the runway while both nose-wheel probes remained
     * several centimetres above it. The corridor is still local to each authored
     * device and frozen from wheelbasePoint + device dimensions; it never follows
     * suspension localOffset, steering, or wheel rotation. BODY collision outside
     * these small gear corridors remains unchanged.
     */
    private static Rasterized excludeRollingGroundDeviceCells(
        Rasterized rasterized,
        double resolution,
        List<RollingGroundDeviceEnvelope> envelopes
    ) {
        if (rasterized == null || rasterized.cells().isEmpty() || envelopes.isEmpty()) {
            return rasterized;
        }
        Set<GridCell> filtered = new HashSet<>(rasterized.cells());
        filtered.removeIf(cell -> rollingGroundDeviceCell(
            cell, resolution, envelopes
        ));
        return new Rasterized(filtered, rasterized.cellTests(), rasterized.overBudget());
    }

    private static boolean rollingGroundDeviceCell(
        GridCell cell,
        double resolution,
        List<RollingGroundDeviceEnvelope> envelopes
    ) {
        double x = (cell.x() + 0.5) * resolution;
        double y = (cell.y() + 0.5) * resolution;
        double z = (cell.z() + 0.5) * resolution;
        double halfCell = 0.5 * resolution;
        for (RollingGroundDeviceEnvelope envelope : envelopes) {
            if (Math.abs(x - envelope.cx()) <= envelope.halfWidth() + halfCell
                && Math.abs(y - envelope.cy()) <= envelope.halfVerticalClearance() + halfCell
                && Math.abs(z - envelope.cz()) <= envelope.halfLongitudinalClearance() + halfCell) {
                return true;
            }
        }
        return false;
    }

    private static List<RollingGroundDeviceEnvelope> rollingGroundDeviceEnvelopes(
        EntityVehicleF_Physics vehicle
    ) {
        if (vehicle == null || vehicle.allParts == null || vehicle.allParts.isEmpty()) {
            return List.of();
        }
        List<RollingGroundDeviceEnvelope> envelopes = new ArrayList<>();
        for (APart part : vehicle.allParts) {
            if (!(part instanceof PartGroundDevice device)
                || device.isSpare
                || !device.isValid
                // A freshly placed IV aircraft can expose valid installed gear one
                // tick before part_active becomes true. Compound/model topology is
                // intentionally stable and does not rebuild on that live animation,
                // so filtering on isActiveVar here permanently misses the support
                // corridor for that aircraft. Use authored/valid rolling membership;
                // live gear authority still remains in LandingGearSolver.
                || device.definition == null
                || device.definition.ground == null
                || (!device.definition.ground.isWheel && !device.definition.ground.isTread)
                || device.wheelbasePoint == null
                || !Double.isFinite(device.wheelbasePoint.x)
                || !Double.isFinite(device.wheelbasePoint.y)
                || !Double.isFinite(device.wheelbasePoint.z)) {
                continue;
            }
            double width = device.getWidth();
            double height = device.getHeight();
            if (!Double.isFinite(width) || !Double.isFinite(height)
                || width <= 0.0 || height <= 0.0) {
                continue;
            }
            envelopes.add(new RollingGroundDeviceEnvelope(
                device.wheelbasePoint.x,
                device.wheelbasePoint.y,
                device.wheelbasePoint.z,
                0.5 * width + ROLLING_GEAR_HULL_EXCLUSION_WIDTH_MARGIN_METERS,
                0.5 * height + ROLLING_GEAR_HULL_SUPPORT_VERTICAL_MARGIN_METERS,
                0.5 * height + ROLLING_GEAR_HULL_SUPPORT_LONGITUDINAL_MARGIN_METERS
            ));
        }
        return envelopes.isEmpty() ? List.of() : List.copyOf(envelopes);
    }

    private static String rollingGroundDeviceEnvelopeSignature(EntityVehicleF_Physics vehicle) {
        List<RollingGroundDeviceEnvelope> envelopes = rollingGroundDeviceEnvelopes(vehicle);
        if (envelopes.isEmpty()) {
            return "none";
        }
        List<String> stable = new ArrayList<>(envelopes.size());
        for (RollingGroundDeviceEnvelope envelope : envelopes) {
            stable.add(
                stable(envelope.cx()) + "," + stable(envelope.cy()) + ","
                    + stable(envelope.cz()) + "," + stable(envelope.halfWidth()) + ","
                    + stable(envelope.halfVerticalClearance()) + ","
                    + stable(envelope.halfLongitudinalClearance())
            );
        }
        Collections.sort(stable);
        return Integer.toHexString(stable.toString().hashCode());
    }

    /** Generic 13-axis triangle/AABB SAT test. */
    private static boolean triangleIntersectsBox(
        Triangle triangle,
        double cx,
        double cy,
        double cz,
        double half
    ) {
        Vec3 a = triangle.a().subtract(cx, cy, cz);
        Vec3 b = triangle.b().subtract(cx, cy, cz);
        Vec3 c = triangle.c().subtract(cx, cy, cz);

        if (Math.min(a.x(), Math.min(b.x(), c.x())) > half
            || Math.max(a.x(), Math.max(b.x(), c.x())) < -half
            || Math.min(a.y(), Math.min(b.y(), c.y())) > half
            || Math.max(a.y(), Math.max(b.y(), c.y())) < -half
            || Math.min(a.z(), Math.min(b.z(), c.z())) > half
            || Math.max(a.z(), Math.max(b.z(), c.z())) < -half) {
            return false;
        }

        Vec3 ab = b.subtract(a);
        Vec3 bc = c.subtract(b);
        Vec3 ca = a.subtract(c);
        Vec3 normal = ab.cross(c.subtract(a));
        if (separated(normal, a, b, c, half)) {
            return false;
        }
        for (Vec3 edge : new Vec3[]{ab, bc, ca}) {
            if (separated(new Vec3(0.0, edge.z(), -edge.y()), a, b, c, half)
                || separated(new Vec3(-edge.z(), 0.0, edge.x()), a, b, c, half)
                || separated(new Vec3(edge.y(), -edge.x(), 0.0), a, b, c, half)) {
                return false;
            }
        }
        return true;
    }

    private static boolean separated(Vec3 axis, Vec3 a, Vec3 b, Vec3 c, double half) {
        if (axis.lengthSquared() <= AXIS_EPSILON_SQUARED) {
            return false;
        }
        double p0 = a.dot(axis);
        double p1 = b.dot(axis);
        double p2 = c.dot(axis);
        double min = Math.min(p0, Math.min(p1, p2));
        double max = Math.max(p0, Math.max(p1, p2));
        double radius = half * (Math.abs(axis.x()) + Math.abs(axis.y()) + Math.abs(axis.z()));
        return min > radius || max < -radius;
    }

    private static List<HullBox> mergeCells(Set<GridCell> occupied, double resolution) {
        if (occupied.isEmpty()) {
            return List.of();
        }
        Set<GridCell> remaining = new HashSet<>(occupied);
        List<GridCell> order = new ArrayList<>(occupied);
        order.sort(GridCell.COMPARATOR);
        List<HullBox> result = new ArrayList<>();

        for (GridCell origin : order) {
            if (!remaining.contains(origin)) {
                continue;
            }
            int x0 = origin.x();
            int y0 = origin.y();
            int z0 = origin.z();
            int x1 = x0 + 1;
            while (remaining.contains(new GridCell(x1, y0, z0))) {
                ++x1;
            }

            int z1 = z0 + 1;
            while (planePresent(remaining, x0, x1, y0, z1)) {
                ++z1;
            }

            int y1 = y0 + 1;
            while (volumePresent(remaining, x0, x1, y1, z0, z1)) {
                ++y1;
            }

            for (int x = x0; x < x1; ++x) {
                for (int y = y0; y < y1; ++y) {
                    for (int z = z0; z < z1; ++z) {
                        remaining.remove(new GridCell(x, y, z));
                    }
                }
            }

            double minX = x0 * resolution;
            double minY = y0 * resolution;
            double minZ = z0 * resolution;
            double maxX = x1 * resolution;
            double maxY = y1 * resolution;
            double maxZ = z1 * resolution;
            result.add(new HullBox(
                stable((minX + maxX) * 0.5),
                stable((minY + maxY) * 0.5),
                stable((minZ + maxZ) * 0.5),
                stable((maxX - minX) * 0.5),
                stable((maxY - minY) * 0.5),
                stable((maxZ - minZ) * 0.5)
            ));
            if (result.size() > MAX_MERGED_BOXES) {
                return result;
            }
        }
        result.sort(HullBox.COMPARATOR);
        return result;
    }

    private static boolean planePresent(
        Set<GridCell> remaining,
        int x0,
        int x1,
        int y,
        int z
    ) {
        for (int x = x0; x < x1; ++x) {
            if (!remaining.contains(new GridCell(x, y, z))) {
                return false;
            }
        }
        return true;
    }

    private static boolean volumePresent(
        Set<GridCell> remaining,
        int x0,
        int x1,
        int y,
        int z0,
        int z1
    ) {
        for (int x = x0; x < x1; ++x) {
            for (int z = z0; z < z1; ++z) {
                if (!remaining.contains(new GridCell(x, y, z))) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * Exact support stations from rigid skid/float/pontoon render geometry.
     *
     * <p>The 0.20 m voxel shell remains the collision/crash body. These points
     * preserve the actual low envelope so a static support does not have to
     * sink until a coarse body voxel reaches terrain. No vehicle/model-specific
     * offsets are used.</p>
     */
    private static List<StaticSupportPoint> extractStaticSupportPoints(
        List<NamedSupportTriangles> supportObjects
    ) {
        if (supportObjects == null || supportObjects.isEmpty()) {
            return List.of();
        }
        List<StaticSupportPoint> candidates = new ArrayList<>();
        for (NamedSupportTriangles supportObject : supportObjects) {
            if (supportObject == null || supportObject.triangles() == null
                || supportObject.triangles().isEmpty()) {
                continue;
            }
            double minimumY = Double.POSITIVE_INFINITY;
            for (Triangle triangle : supportObject.triangles()) {
                minimumY = Math.min(minimumY, triangle.a().y());
                minimumY = Math.min(minimumY, triangle.b().y());
                minimumY = Math.min(minimumY, triangle.c().y());
            }
            if (!Double.isFinite(minimumY)) {
                continue;
            }
            for (Triangle triangle : supportObject.triangles()) {
                addLowSupportCandidate(candidates, triangle.a(), minimumY, supportObject.name());
                addLowSupportCandidate(candidates, triangle.b(), minimumY, supportObject.name());
                addLowSupportCandidate(candidates, triangle.c(), minimumY, supportObject.name());
            }
        }
        candidates = dedupeStaticSupportPoints(candidates);
        if (candidates.size() <= MAX_STATIC_SUPPORT_POINTS) {
            candidates.sort(StaticSupportPoint.COMPARATOR);
            return List.copyOf(candidates);
        }

        // Preserve the support polygon rather than choosing arbitrary vertices.
        // Directional extrema retain long-rail endpoints and left/right footprint
        // edges for skid/float geometry while keeping the solver contact budget
        // bounded and independent of mesh tessellation density.
        List<StaticSupportPoint> extrema = new ArrayList<>();
        for (int index = 0; index < STATIC_SUPPORT_EXTREMA_DIRECTIONS; ++index) {
            double angle = Math.PI * 2.0 * index / STATIC_SUPPORT_EXTREMA_DIRECTIONS;
            double dx = Math.cos(angle);
            double dz = Math.sin(angle);
            StaticSupportPoint best = null;
            double bestProjection = Double.NEGATIVE_INFINITY;
            for (StaticSupportPoint candidate : candidates) {
                double projection = candidate.x() * dx + candidate.z() * dz;
                if (projection > bestProjection + 1.0E-9
                    || (Math.abs(projection - bestProjection) <= 1.0E-9
                        && (best == null || candidate.y() < best.y()))) {
                    best = candidate;
                    bestProjection = projection;
                }
            }
            if (best != null) {
                addUniqueStaticSupportPoint(extrema, best);
            }
        }
        if (extrema.isEmpty()) {
            candidates.sort(StaticSupportPoint.COMPARATOR);
            return List.copyOf(candidates.subList(0, MAX_STATIC_SUPPORT_POINTS));
        }
        extrema.sort(StaticSupportPoint.COMPARATOR);
        return List.copyOf(extrema);
    }

    private static void addLowSupportCandidate(
        List<StaticSupportPoint> candidates,
        Vec3 vertex,
        double minimumY,
        String sourceObject
    ) {
        if (vertex == null || !vertex.finite()
            || vertex.y() > minimumY + STATIC_SUPPORT_LOW_BAND_METERS) {
            return;
        }
        candidates.add(new StaticSupportPoint(
            stable(vertex.x()), stable(vertex.y()), stable(vertex.z()),
            sourceObject
        ));
    }

    private static List<StaticSupportPoint> dedupeStaticSupportPoints(
        List<StaticSupportPoint> candidates
    ) {
        if (candidates == null || candidates.isEmpty()) {
            return new ArrayList<>();
        }
        Map<String, StaticSupportPoint> unique = new LinkedHashMap<>();
        for (StaticSupportPoint point : candidates) {
            long qx = Math.round(point.x() * 100.0);
            long qz = Math.round(point.z() * 100.0);
            String key = qx + ":" + qz;
            StaticSupportPoint existing = unique.get(key);
            if (existing == null || point.y() < existing.y()) {
                unique.put(key, point);
            }
        }
        return new ArrayList<>(unique.values());
    }

    private static void addUniqueStaticSupportPoint(
        List<StaticSupportPoint> output,
        StaticSupportPoint candidate
    ) {
        for (StaticSupportPoint existing : output) {
            double dx = existing.x() - candidate.x();
            double dz = existing.z() - candidate.z();
            if (dx * dx + dz * dz <= 1.0E-6) {
                return;
            }
        }
        output.add(candidate);
    }

    private static double minimumStaticSupportY(List<StaticSupportPoint> points) {
        double minimum = Double.POSITIVE_INFINITY;
        if (points != null) {
            for (StaticSupportPoint point : points) {
                minimum = Math.min(minimum, point.y());
            }
        }
        return Double.isFinite(minimum) ? minimum : Double.NaN;
    }

    private static String staticSupportObjectNames(List<StaticSupportPoint> points) {
        if (points == null || points.isEmpty()) {
            return "[]";
        }
        Set<String> names = new java.util.TreeSet<>();
        for (StaticSupportPoint point : points) {
            names.add(point.sourceObject());
        }
        return names.toString().replace(' ', '_');
    }


    /**
     * Separates actual relative-pose animation from visibility/state animation.
     * IV lists both in animatedObjectDefinitions; treating every listed object as
     * physically moving was deleting intact wing meshes from the Sable hull.
     */
    private static AnimatedObjectClassification classifyAnimatedObjects(EntityVehicleF_Physics vehicle) {
        return classifyAnimatedObjects(animatedDefinitions(vehicle));
    }

    private static Map<String, JSONAnimatedObject> animatedDefinitions(
        EntityVehicleF_Physics vehicle
    ) {
        Map<String, JSONAnimatedObject> definitions = new LinkedHashMap<>();
        // Authored fallback is useful during early activation. The resolved runtime
        // definition is authoritative when IV has installed one for the same object.
        if (vehicle != null && vehicle.definition != null && vehicle.definition.rendering != null
            && vehicle.definition.rendering.animatedObjects != null) {
            for (JSONAnimatedObject object : vehicle.definition.rendering.animatedObjects) {
                if (object != null && object.objectName != null)
                    definitions.put(normalize(object.objectName), object);
            }
        }
        if (vehicle != null && vehicle.animatedObjectDefinitions != null) {
            vehicle.animatedObjectDefinitions.forEach((name, object) -> {
                if (!normalize(name).isEmpty()) definitions.put(normalize(name), object);
            });
        }
        return definitions;
    }

    private static AnimatedObjectClassification classifyAnimatedObjects(
        Map<String, JSONAnimatedObject> definitions
    ) {
        Set<String> physical = new HashSet<>();
        Set<String> damageReplacement = new HashSet<>();
        for (Map.Entry<String, JSONAnimatedObject> entry : definitions.entrySet()) {
            String name = entry.getKey();
            if (animationChainHasPhysicalTransform(name, definitions, new HashSet<>())) physical.add(name);
            if (animatedObjectIsDamageReplacement(entry.getValue())) damageReplacement.add(name);
        }
        return new AnimatedObjectClassification(
            Set.copyOf(definitions.keySet()), Set.copyOf(physical), Set.copyOf(damageReplacement)
        );
    }

    private static boolean animationChainHasPhysicalTransform(String name,
        Map<String, JSONAnimatedObject> definitions, Set<String> visiting
    ) {
        // Unknown definitions, unresolved parents and cycles cannot prove a static
        // pose. In particular a visibility-only skin can inherit a moving hinge.
        JSONAnimatedObject object = definitions.get(name);
        if (object == null || !visiting.add(name)) return true;
        if (animatedObjectHasPhysicalTransform(object)) return true;
        if (object.applyAfter == null || object.applyAfter.isBlank()) return false;
        return animationChainHasPhysicalTransform(normalize(object.applyAfter), definitions, visiting);
    }

    private static boolean animatedObjectHasPhysicalTransform(JSONAnimatedObject object) {
        if (object == null || object.animations == null) return false;
        for (JSONAnimationDefinition animation : object.animations) {
            if (animation == null || animation.animationType == null) continue;
            String type = normalize(animation.animationType.name());
            if (type.contains("rotation") || type.contains("translation")
                || type.contains("scaling")) {
                return true;
            }
        }
        return false;
    }

    static String animationSignature(EntityVehicleF_Physics vehicle) {
        return animationSignature(animatedDefinitions(vehicle));
    }

    private static String animationSignature(Map<String, JSONAnimatedObject> definitions) {
        AnimatedObjectClassification classification = classifyAnimatedObjects(definitions);
        if (classification.knownAnimatedNames().isEmpty()) return "none";
        List<String> tokens = new ArrayList<>();
        for (String name : classification.knownAnimatedNames()) {
            tokens.add(name
                + (classification.physicalTransformNames().contains(name) ? ":P" : ":V")
                + (rigidAnimationChain(name, definitions, new HashSet<>()) ? ":R" : ":U")
                + (classification.damageReplacementNames().contains(name) ? ":D" : "")
                + ':' + animationSourceSignature(definitions.get(name)));
        }
        Collections.sort(tokens);
        return tokens.toString();
    }

    /** Static IV animation-source inputs that can change a same-named mounted object's pose contract. */
    private static String animationSourceSignature(JSONAnimatedObject object) {
        if (object == null) return "missing";
        StringBuilder signature = new StringBuilder()
            .append("after=").append(normalize(object.applyAfter))
            .append(";blended=").append(object.blendedAnimations);
        if (object.animations == null) return signature.append(";animations=null").toString();
        for (JSONAnimationDefinition animation : object.animations) {
            if (animation == null) {
                signature.append(";null");
                continue;
            }
            signature.append(";type=").append(animation.animationType)
                .append(";variable=").append(animation.variable == null ? "" : animation.variable.trim())
                .append(";center=").append(pointSignature(animation.centerPoint))
                .append(";axis=").append(pointSignature(animation.axis))
                .append(";offset=").append(Double.toHexString(animation.offset))
                .append(";clamp=").append(Double.toHexString(animation.clampMin))
                .append(',').append(Double.toHexString(animation.clampMax))
                .append(";absolute=").append(animation.absolute)
                .append(";invert=").append(animation.invert)
                .append(";duration=").append(animation.duration)
                .append(";easing=").append(animation.forwardsEasing).append(',').append(animation.reverseEasing)
                .append(";delay=").append(animation.forwardsDelay).append(',').append(animation.reverseDelay)
                .append(";skip=").append(animation.skipForwardsMovement).append(',').append(animation.skipReverseMovement);
        }
        return signature.toString();
    }

    private static String pointSignature(Point3D point) {
        return point == null ? "null" : Double.toHexString(point.x) + ','
            + Double.toHexString(point.y) + ',' + Double.toHexString(point.z);
    }


    private static boolean isStaticGroundSupport(String name) {
        return containsAny(name, "skid", "float", "pontoon");
    }

    private static IgnoreReason bodyPressureIgnoreReason(
        Mesh mesh, AnimatedObjectClassification classification
    ) {
        if (mesh == null || mesh.isLines) return IgnoreReason.RENDER_ONLY;
        String name = normalize(mesh.name);
        if (classification.damageReplacementNames().contains(name)) return IgnoreReason.RENDER_ONLY;
        return aerodynamicIgnoreReason(name, mesh.isTranslucent);
    }

    private static IgnoreReason staticCollisionIgnoreReason(
        Mesh mesh, AnimatedObjectClassification classification, boolean includeStaticGroundSupport
    ) {
        if (mesh == null || mesh.isLines) return IgnoreReason.RENDER_ONLY;
        String name = normalize(mesh.name);
        if (classification.physicalTransformNames().contains(name)
            || classification.damageReplacementNames().contains(name)) return IgnoreReason.RENDER_ONLY;
        return includeStaticGroundSupport && isStaticGroundSupport(name)
            ? IgnoreReason.NONE
            : aerodynamicIgnoreReason(name, mesh.isTranslucent);
    }

    private static IgnoreReason aerodynamicIgnoreReason(String name, boolean translucent) {
        if (name.isEmpty()) return IgnoreReason.NONE;
        if (name.startsWith("#") || name.startsWith("&") || name.equals("key")) return IgnoreReason.RENDER_ONLY;
        if (containsAny(name,
            "wheel", "tire", "tyre", "tread", "landinggear", "landing_gear", "landing gear",
            "tailwheel", "tail_wheel", "nosewheel", "nose_wheel", "mainwheel", "main_wheel",
            "rotor", "blade", "propeller", "prop_", "axle", "bogie", "brakedisc", "brake_disc")) {
            return IgnoreReason.GEAR_OR_MOVING_HARDWARE;
        }
        if (containsAny(name,
            "instrument", "gauge", "needle", "text", "label", "seat", "cockpit_control",
            "stick", "yoke", "pedal", "lever", "handle", "dashboard", "interior",
            "mirror", "antenna", "lamp", "light", "flare", "exhaust", "particle",
            "missing_model", "boundingbox", "bounding_box", "glovebox", "glove_box", "ignition_key")) {
            return IgnoreReason.RENDER_ONLY;
        }
        if (translucent && !containsAny(name,
            "glass", "window", "windscreen", "windshield", "canopy", "translucent")) {
            return IgnoreReason.RENDER_ONLY;
        }
        return IgnoreReason.NONE;
    }

    private static boolean containsAny(String value, String... needles) {
        for (String needle : needles) {
            if (value.contains(needle)) {
                return true;
            }
        }
        return false;
    }

    private static String normalize(String value) {
        if (value == null) {
            return "";
        }
        return value.trim().toLowerCase(Locale.ROOT).replace('-', '_').replace(' ', '_');
    }

    private static String modelLocation(EntityVehicleF_Physics vehicle) {
        if (vehicle == null || vehicle.definition == null) {
            return null;
        }
        try {
            return vehicle.definition.getModelLocation(vehicle.subDefinition);
        } catch (RuntimeException exception) {
            return null;
        }
    }

    private static double finiteScale(double value) {
        return Double.isFinite(value) && Math.abs(value) > 1.0E-9 ? value : 1.0;
    }

    private static int floorGrid(double value, double resolution) {
        return (int) Math.floor(value / resolution + 1.0E-10);
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    private static double stable(double value) {
        return Math.rint(value * 1_000_000.0) / 1_000_000.0;
    }

    private static String safe(String value) {
        if (value == null) {
            return "null";
        }
        return value.replace('\n', '_').replace('\r', '_');
    }

    private record AnimatedObjectClassification(
        Set<String> knownAnimatedNames,
        Set<String> physicalTransformNames,
        Set<String> damageReplacementNames
    ) {
    }

    public record PreparedMovingObjectHull(String objectName, double resolution,
                                           int sourceTriangles, List<HullBox> boxes) {}

    public record SurfaceTriangle(com.g9third.pmweatheriv.physics.Vec3d a,
                                  com.g9third.pmweatheriv.physics.Vec3d b,
                                  com.g9third.pmweatheriv.physics.Vec3d c) {}

    public record PreparedHull(
        String modelLocation,
        boolean usable,
        String reason,
        double resolution,
        int retainedObjects,
        int sourceTriangles,
        int shellCells,
        long triangleCellTests,
        int mergedBoxes,
        int skippedAnimatedObjects,
        int skippedPhysicalAnimatedObjects,
        int skippedDamageReplacementObjects,
        int retainedVisibilityOnlyAnimatedObjects,
        int skippedGearObjects,
        int skippedRenderObjects,
        int rollingGearCellsExcluded,
        List<HullBox> boxes,
        List<StaticSupportPoint> staticSupportPoints,
        List<SurfaceTriangle> sampleSurfaceTriangles
    ) {
        public PreparedHull(String modelLocation, boolean usable, String reason, double resolution,
            int retainedObjects, int sourceTriangles, int shellCells, long triangleCellTests,
            int mergedBoxes, int skippedAnimatedObjects, int skippedPhysicalAnimatedObjects,
            int skippedDamageReplacementObjects, int retainedVisibilityOnlyAnimatedObjects,
            int skippedGearObjects, int skippedRenderObjects, int rollingGearCellsExcluded,
            List<HullBox> boxes, List<StaticSupportPoint> staticSupportPoints) {
            this(modelLocation,usable,reason,resolution,retainedObjects,sourceTriangles,shellCells,
                triangleCellTests,mergedBoxes,skippedAnimatedObjects,skippedPhysicalAnimatedObjects,
                skippedDamageReplacementObjects,retainedVisibilityOnlyAnimatedObjects,skippedGearObjects,
                skippedRenderObjects,rollingGearCellsExcluded,boxes,staticSupportPoints,List.of());
        }
        private static PreparedHull fallback(String modelLocation, String reason) {
            return new PreparedHull(
                modelLocation == null ? "missing" : modelLocation,
                false,
                reason,
                0.0,
                0,
                0,
                0,
                0L,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                List.of(),
                List.of()
            );
        }
    }

    public record StaticSupportPoint(double x, double y, double z, String sourceObject) {
        private static final Comparator<StaticSupportPoint> COMPARATOR = Comparator
            .comparingDouble(StaticSupportPoint::x)
            .thenComparingDouble(StaticSupportPoint::z)
            .thenComparingDouble(StaticSupportPoint::y)
            .thenComparing(StaticSupportPoint::sourceObject);
    }

    private record NamedSupportTriangles(String name, List<Triangle> triangles) {
    }

    public record HullBox(double cx, double cy, double cz, double hx, double hy, double hz) {
        private static final Comparator<HullBox> COMPARATOR = Comparator
            .comparingDouble(HullBox::cx)
            .thenComparingDouble(HullBox::cy)
            .thenComparingDouble(HullBox::cz)
            .thenComparingDouble(HullBox::hx)
            .thenComparingDouble(HullBox::hy)
            .thenComparingDouble(HullBox::hz);
    }

    private enum IgnoreReason {
        NONE,
        GEAR_OR_MOVING_HARDWARE,
        RENDER_ONLY
    }

    private record Rasterized(Set<GridCell> cells, long cellTests, boolean overBudget) {
    }

    private record RollingGroundDeviceEnvelope(
        double cx,
        double cy,
        double cz,
        double halfWidth,
        double halfVerticalClearance,
        double halfLongitudinalClearance
    ) {
    }

    private record GridCell(int x, int y, int z) {
        private static final Comparator<GridCell> COMPARATOR = Comparator
            .comparingInt(GridCell::y)
            .thenComparingInt(GridCell::z)
            .thenComparingInt(GridCell::x);
    }

    private record Triangle(Vec3 a, Vec3 b, Vec3 c) {
    }

    private record Vec3(double x, double y, double z) {
        private Vec3 subtract(Vec3 other) {
            return new Vec3(x - other.x, y - other.y, z - other.z);
        }

        private Vec3 subtract(double ox, double oy, double oz) {
            return new Vec3(x - ox, y - oy, z - oz);
        }

        private Vec3 cross(Vec3 other) {
            return new Vec3(
                y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x
            );
        }

        private double dot(Vec3 other) {
            return x * other.x + y * other.y + z * other.z;
        }

        private double lengthSquared() {
            return x * x + y * y + z * z;
        }

        private boolean finite() {
            return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z);
        }
    }

    private static final class Bounds {
        private double minX = Double.POSITIVE_INFINITY;
        private double minY = Double.POSITIVE_INFINITY;
        private double minZ = Double.POSITIVE_INFINITY;
        private double maxX = Double.NEGATIVE_INFINITY;
        private double maxY = Double.NEGATIVE_INFINITY;
        private double maxZ = Double.NEGATIVE_INFINITY;

        private void include(Vec3 point) {
            minX = Math.min(minX, point.x());
            minY = Math.min(minY, point.y());
            minZ = Math.min(minZ, point.z());
            maxX = Math.max(maxX, point.x());
            maxY = Math.max(maxY, point.y());
            maxZ = Math.max(maxZ, point.z());
        }

        private boolean validSurface() {
            return Double.isFinite(minX) && Double.isFinite(minY) && Double.isFinite(minZ)
                && Double.isFinite(maxX) && Double.isFinite(maxY) && Double.isFinite(maxZ)
                && Math.max(spanX(), Math.max(spanY(), spanZ())) > 1.0E-9;
        }

        private boolean valid() {
            return Double.isFinite(minX) && Double.isFinite(minY) && Double.isFinite(minZ)
                && Double.isFinite(maxX) && Double.isFinite(maxY) && Double.isFinite(maxZ)
                && maxX > minX && maxY > minY && maxZ > minZ;
        }

        private double spanX() {
            return maxX - minX;
        }

        private double spanY() {
            return maxY - minY;
        }

        private double spanZ() {
            return maxZ - minZ;
        }
    }
}
