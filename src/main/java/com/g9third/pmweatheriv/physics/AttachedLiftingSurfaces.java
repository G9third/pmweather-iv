package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.model.ParsedModelSnapshot;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.jsondefs.JSONAnimatedObject;
import static com.g9third.pmweatheriv.physics.AnimatedWingGeometry.RigidTransform;
import static com.g9third.pmweatheriv.physics.ModelGeometryData.*;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.*;

/** Explicit attached lifting geometry, with independently owned native poses and no extra area metadata. */
final class AttachedLiftingSurfaces {
    private AttachedLiftingSurfaces() {}
    private record Source(APart owner, String location, RigidTransform reference,
                          ModelObject object, Map<String, JSONAnimatedObject> definitions,
                          Map<String, AnimationHint> hints, AnimationHint placementHint) {}

    /** Placement motion belongs to the live pose, not the preparation/cache identity. */
    static String signature(EntityVehicleF_Physics vehicle) {
        if (vehicle==null || vehicle.allParts.isEmpty()) return "none";
        StringBuilder key=new StringBuilder();
        vehicle.allParts.stream().filter(part -> part!=null).sorted(Comparator.comparing(part -> part.uniqueUUID)).forEach(part -> {
            key.append(part.uniqueUUID).append(':').append(slotPath(part)).append(':')
                .append(ModelAnimationHints.modelLocation(part)).append(':').append(part.scale)
                .append(':').append(part.isSpare).append(':').append(part.isFake())
                .append(':').append(AuthoredLiftingSurfacePoses.signature(part));
            if (part.placementDefinition!=null) {
                var placement=part.placementDefinition;
                key.append(':').append(placement.pos).append(':').append(placement.partScale)
                    .append(':').append(placement.applyAfter);
                if (placement.rot!=null) key.append(':').append(placement.rot.angles)
                    .append(':').append(placement.rot.m00).append(':').append(placement.rot.m01).append(':').append(placement.rot.m02)
                    .append(':').append(placement.rot.m10).append(':').append(placement.rot.m11).append(':').append(placement.rot.m12)
                    .append(':').append(placement.rot.m20).append(':').append(placement.rot.m21).append(':').append(placement.rot.m22);
                AuthoredLiftingSurfacePoses.appendAnimations(key, placement.animations);
                AuthoredLiftingSurfacePoses.appendAnimations(key, placement.activeAnimations);
            }
            if (part.definition!=null && part.definition.generic!=null) {
                key.append(':').append(part.definition.generic.slotOffset);
                AuthoredLiftingSurfacePoses.appendAnimations(key, part.definition.generic.movementAnimations);
                AuthoredLiftingSurfacePoses.appendAnimations(key, part.definition.generic.activeAnimations);
            }
            key.append(';');
        });
        return key.toString();
    }

    static boolean active(APart part) {
        if (part==null) return false;
        java.util.Set<APart> visited=java.util.Collections.newSetFromMap(new java.util.IdentityHashMap<>());
        for (APart owner=part; owner!=null; owner=owner.partOn) {
            if (!visited.add(owner) || !owner.isValid || owner.isInvisible || owner.isSpare || owner.isFake()
                || owner.outOfHealth || (owner.isActiveVar!=null && !owner.isActiveVar.isActive)
                || (owner.isExteriorVar!=null && !owner.isExteriorVar.isActive)) return false;
        }
        return true;
    }

    static RigidTransform resolvedFrame(EntityVehicleF_Physics vehicle, APart part) {
        if (vehicle==null || part==null || vehicle.position==null || part.position==null) return null;
        Vec3d scale=part.scale==null ? new Vec3d(1, 1, 1) : new Vec3d(part.scale.x, part.scale.y, part.scale.z);
        // Native localOffset is rewritten for nested parts. The resolved rendered
        // world origin is the authoritative translation for every parent depth.
        Vec3d offset=FlightMath.toLocal(vehicle.orientation, new Vec3d(part.position.x-vehicle.position.x,
            part.position.y-vehicle.position.y, part.position.z-vehicle.position.z));
        return ModelPoseMath.resolvedPart(vehicle.orientation, part.orientation,
            offset, scale);
    }

    static String slotPath(APart part) {
        java.util.Set<APart> visited=java.util.Collections.newSetFromMap(new java.util.IdentityHashMap<>());
        String path="";
        for (APart owner=part; owner!=null; owner=owner.partOn) {
            if (!visited.add(owner)) return "cycle/"+part.uniqueUUID;
            path=owner.placementSlot+"/"+path;
        }
        return path;
    }

    static PreparedModel append(EntityVehicleF_Physics vehicle, PreparedModel base) {
        if (vehicle==null || vehicle.allParts.isEmpty()) return base;
        List<Source> sources=new ArrayList<>();
        BoundsAccumulator extent=new BoundsAccumulator();
        extent.include(base.fullBounds());
        int sourceCount=0;
        for (APart part : vehicle.allParts.stream().filter(p -> p!=null)
            .sorted(Comparator.comparing(p -> p.uniqueUUID)).toList()) {
            // Temporarily hidden/inactive parts retain their geometry, then lose force
            // through the owner snapshot. Removal changes the UUID signature and rebuilds.
            if (part.isSpare || part.isFake()) continue;
            RigidTransform reference=resolvedFrame(vehicle, part);
            if (reference==null || ModelPoseMath.inverse(reference)==null) continue;
            String location=ModelAnimationHints.modelLocation(part);
            Map<String, JSONAnimatedObject> definitions=AuthoredLiftingSurfacePoses.definitions(part);
            Map<String, AnimationHint> hints=new java.util.HashMap<>(ModelAnimationHints.animatedSurfaceHints(definitions));
            AnimationHint placementHint=placementHint(part);
            List<ParsedModelSnapshot.Mesh> meshes;
            try { meshes=ParsedModelSnapshot.load(location).objects(); }
            catch (RuntimeException unavailable) { continue; }
            for (ParsedModelSnapshot.Mesh mesh : meshes) {
                if (mesh==null || mesh.isLines || mesh.isTranslucent || ModelAnimationHints.ignoredObject(mesh.name)
                    || damageOnly(definitions, mesh.name, new HashSet<>())) continue;
                var candidate=AuthoredLiftingSurfacePoses.resolve(definitions, mesh.name, new HashSet<>());
                if (candidate.status()!=AuthoredLiftingSurfacePoses.Status.STATIC
                    && candidate.status()!=AuthoredLiftingSurfacePoses.Status.AUTHORED) continue;
                String name=ModelAnimationHints.normalize(mesh.name);
                SurfaceKind named=SurfaceKind.fromSpecificName(name);
                AnimationHint hint=hints.getOrDefault(name, AnimationHint.NONE).combine(placementHint);
                if (hint.anySurfaceControl()) hints.put(name, hint);
                if ((named==null || named.isBodyPressure()) && !hint.anySurfaceControl()) continue;
                ObjectStats stats=ModelTriangleSampling.scanObject(mesh);
                if (stats.validTriangles()<=0 || !stats.bounds().valid()) continue;
                // The quota map must distinguish two installed copies of one cached mesh.
                ModelObject object=new ModelObject(mesh, part.uniqueUUID+"/"+mesh.name, part.uniqueUUID+"/"+name, stats,
                    ModelTriangleSampling.objectPriority(name, hint, stats));
                sources.add(new Source(part, location, reference, object, definitions, hints, placementHint));
                sourceCount+=stats.validTriangles();
                extent.include(transformedBounds(stats.bounds(), reference));
            }
        }
        if (sources.isEmpty()) return base;
        Bounds bounds=extent.finish();
        List<ModelObject> objects=sources.stream().map(Source::object).toList();
        Map<ModelObject, Integer> quotas=ModelTriangleSampling.allocateObjectQuotas(objects);
        List<LiftingPatch> added=new ArrayList<>();
        Map<String, AuthoredLiftingSurfacePoses.Binding> addedBindings=new LinkedHashMap<>();
        int retained=0;
        int pairId=base.liftingPatches().stream().mapToInt(LiftingPatch::symmetryPair).max().orElse(-1)+1;
        for (Source source : sources) {
            List<RawTriangle> sampled=new ArrayList<>();
            ModelTriangleSampling.readSampledTriangles(source.object(), quotas.getOrDefault(source.object(), 0), sampled);
            EnumMap<SurfaceKind, List<ClassifiedTriangle>> byKind=new EnumMap<>(SurfaceKind.class);
            for (SurfaceKind kind : SurfaceKind.values()) byKind.put(kind, new ArrayList<>());
            Bounds objectBounds=transformedBounds(source.object().stats().bounds(), source.reference());
            for (RawTriangle raw : sampled) {
                RawTriangle ownerRaw=new RawTriangle(raw.first(), raw.second(), raw.third(), raw.centroid(), raw.normal(),
                    raw.area(), source.object().vertices().name, raw.objectBounds());
                RawTriangle mapped=transform(ownerRaw, source.reference(), objectBounds);
                SurfaceKind kind=explicitKind(mapped, bounds, source.hints());
                if (kind==null) continue;
                byKind.get(kind).add(new ClassifiedTriangle(mapped.first(), mapped.second(), mapped.third(),
                    mapped.centroid(), mapped.normal(), mapped.area(), mapped.objectName(), kind));
                ++retained;
            }
            List<LiftingPatch> patches=new ArrayList<>();
            for (SurfaceKind kind : SurfaceKind.values()) {
                List<ClassifiedTriangle> triangles=byKind.get(kind);
                if (kind.isBodyPressure() || triangles.isEmpty()) continue;
                int before=patches.size();
                if (kind==SurfaceKind.VERTICAL_TAIL || kind==SurfaceKind.RUDDER)
                    pairId=LiftingPatchGeometry.addVerticalLiftingComponent(patches, triangles, kind, bounds, pairId);
                else if (kind==SurfaceKind.WING || kind==SurfaceKind.ELEVON)
                    pairId=LiftingPatchGeometry.addAdaptivePairedLiftingComponent(patches, triangles, kind, bounds,
                        Axis.VERTICAL, pairId, LiftingPatchGeometry.adaptiveMainWingSectionCount(vehicle, triangles, bounds));
                else pairId=LiftingPatchGeometry.addPairedLiftingComponent(patches, triangles, kind, bounds, Axis.VERTICAL, pairId);
                // A small center/outboard part must not vanish because a global
                // symmetry heuristic found no credible opposite or center anchor.
                if (patches.size()==before) patches.add(observedPatch(triangles, kind, bounds));
            }
            patches=LiftingSurfaceFrames.attachGeometryFrames(patches, byKind, bounds, base.bodyBounds());
            Map<String, AuthoredLiftingSurfacePoses.Binding> bindings=AuthoredLiftingSurfacePoses.bind(
                source.definitions(), patches, byKind, bounds);
            AuthoredLiftingSurfacePoses.PartReference reference=new AuthoredLiftingSurfacePoses.PartReference(
                source.owner().uniqueUUID, source.location(), ModelPoseMath.inverse(source.reference()), slotPath(source.owner()));
            for (LiftingPatch patch : patches) {
                String name="PART_"+source.owner().uniqueUUID+"/"+source.object().vertices().name+"/"+patch.name();
                added.add(rename(patch, name));
                AuthoredLiftingSurfacePoses.Binding binding=bindings.get(patch.name());
                String object=binding.objectName().isBlank() && binding.status()==AuthoredLiftingSurfacePoses.Status.STATIC
                    ? source.object().vertices().name : binding.objectName();
                boolean[] placementControls = placementControlOwnership(source.owner());
                addedBindings.put(name, new AuthoredLiftingSurfacePoses.Binding(object,
                    binding.elevator() || placementControls[0],
                    binding.aileron() || placementControls[1],
                    binding.rudder() || placementControls[2], binding.flapSweep(), binding.blended(), binding.status(), reference,
                    binding.elevatorTrim() || placementControls[3],
                    binding.aileronTrim() || placementControls[4],
                    binding.rudderTrim() || placementControls[5]));
            }
        }
        if (added.isEmpty()) return base;
        List<LiftingPatch> patches=mergePatches(base.liftingPatches(), added);
        Map<String, AuthoredLiftingSurfacePoses.Binding> bindings=new LinkedHashMap<>();
        for (LiftingPatch patch : patches) {
            var binding=addedBindings.get(patch.name());
            if (binding==null) binding=base.authoredPoseBindings().get(patch.name());
            if (binding!=null) bindings.put(patch.name(), binding);
        }
        return new PreparedModel(base.modelLocation(), true, base.reason()+";attached-lifting-geometry",
            bounds, base.bodyBounds(), base.pressurePatches(), patches, base.pressureWettedArea(),
            base.sourceTriangles()+sourceCount, base.retainedTriangles()+retained, base.pressureTarget(), bindings);
    }

    /** Attached objects require explicit naming or real animation evidence, never position alone. */
    static SurfaceKind explicitKind(RawTriangle triangle, Bounds bounds, Map<String, AnimationHint> hints) {
        String name=ModelAnimationHints.normalize(triangle.objectName());
        SurfaceKind named=SurfaceKind.fromSpecificName(name);
        AnimationHint hint=hints.getOrDefault(name, AnimationHint.NONE);
        if ((named==null || named.isBodyPressure()) && !hint.anySurfaceControl()) return null;
        SurfaceKind kind=SurfaceClassifier.classify(triangle, bounds, hints, false);
        return kind.isBodyPressure() || named==null && !hint.anySurfaceControl() ? null : kind;
    }

    static List<LiftingPatch> mergePatches(List<LiftingPatch> main, List<LiftingPatch> attached) {
        HashSet<SurfaceKind> observed=new HashSet<>();
        for (LiftingPatch patch : attached) observed.add(patch.kind());
        boolean mainWing=observed.contains(SurfaceKind.WING) || observed.contains(SurfaceKind.AILERON)
            || observed.contains(SurfaceKind.ELEVON);
        boolean horizontal=observed.contains(SurfaceKind.HORIZONTAL_TAIL) || observed.contains(SurfaceKind.ELEVATOR)
            || observed.contains(SurfaceKind.TAILERON) || observed.contains(SurfaceKind.ELEVON);
        List<LiftingPatch> patches=new ArrayList<>();
        for (LiftingPatch patch : main) {
            boolean redundant=patch.name().startsWith("FALLBACK_WING") && mainWing
                || (patch.name().startsWith("FALLBACK_HTAIL") || patch.name().startsWith("FALLBACK_ELEVATOR")) && horizontal
                || patch.name().startsWith("FALLBACK_VTAIL") && (observed.contains(SurfaceKind.VERTICAL_TAIL) || observed.contains(SurfaceKind.RUDDER))
                || patch.name().startsWith("FALLBACK_RUDDER") && observed.contains(SurfaceKind.RUDDER);
            if (!redundant) patches.add(patch);
        }
        patches.addAll(attached);
        return List.copyOf(patches);
    }

    /** Missing owners keep their original area share while supplying zero force.
     * A lifting replacement in the same authored slot retires the old share. */
    static PreparedModel retainMissing(PreparedModel current, PreparedModel previous, java.util.Set<java.util.UUID> present) {
        if (previous==null || previous.authoredPoseBindings().isEmpty()) return current;
        java.util.Set<String> occupied=new HashSet<>();
        for (var binding : current.authoredPoseBindings().values())
            if (binding.partReference()!=null) occupied.add(binding.partReference().slotPath());
        List<LiftingPatch> missing=new ArrayList<>();
        Map<Integer, Integer> pairs=new LinkedHashMap<>();
        int firstPair=current.liftingPatches().stream().mapToInt(LiftingPatch::symmetryPair).max().orElse(-1)+1;
        Map<String, AuthoredLiftingSurfacePoses.Binding> bindings=new LinkedHashMap<>(current.authoredPoseBindings());
        for (LiftingPatch patch : previous.liftingPatches()) {
            var binding=previous.authoredPoseBindings().get(patch.name());
            if (binding==null || binding.partReference()==null || present.contains(binding.partReference().ownerId())
                || occupied.contains(binding.partReference().slotPath())) continue;
            int pair=patch.symmetryPair()<0 ? -1 : pairs.computeIfAbsent(patch.symmetryPair(), ignored -> firstPair+pairs.size());
            missing.add(new LiftingPatch(patch.name(), patch.kind(), patch.pointLocal(), patch.weight(), pair,
                patch.symmetryRole(), patch.sectionIndex(), patch.sectionCount(), patch.sectionMinimumRadius(),
                patch.sectionMaximumRadius(), patch.spanLocal(), patch.chordLocal(), patch.normalLocal(),
                patch.frameConfidence(), patch.aerodynamicCenterLocal()));
            bindings.put(patch.name(), new AuthoredLiftingSurfacePoses.Binding(binding.objectName(), binding.elevator(),
                binding.aileron(), binding.rudder(), binding.flapSweep(), binding.blended(),
                AuthoredLiftingSurfacePoses.Status.HIDDEN, binding.partReference(),
                binding.elevatorTrim(), binding.aileronTrim(), binding.rudderTrim()));
        }
        if (missing.isEmpty()) return current;
        List<LiftingPatch> patches=mergePatches(current.liftingPatches(), missing);
        java.util.Set<String> retainedNames=new HashSet<>();
        for (LiftingPatch patch : patches) retainedNames.add(patch.name());
        bindings.keySet().retainAll(retainedNames);
        return new PreparedModel(current.modelLocation(), true, current.reason()+";missing-attached-area-retained",
            current.fullBounds(), current.bodyBounds(), current.pressurePatches(), patches, current.pressureWettedArea(),
            current.sourceTriangles(), current.retainedTriangles(), current.pressureTarget(), bindings);
    }

    static RawTriangle transform(RawTriangle raw, RigidTransform matrix, Bounds objectBounds) {
        Vec3d a=matrix.point(raw.first()), b=matrix.point(raw.second()), c=matrix.point(raw.third());
        Vec3d cross=b.subtract(a).cross(c.subtract(a));
        double actualArea=raw.second().subtract(raw.first()).cross(raw.third().subtract(raw.first())).length()*.5;
        return new RawTriangle(a, b, c, a.add(b).add(c).scale(1.0/3.0), cross.normalized(),
            actualArea>EPSILON ? cross.length()*.5*raw.area()/actualArea : 0, raw.objectName(), objectBounds);
    }

    static Bounds transformedBounds(Bounds bounds, RigidTransform transform) {
        BoundsAccumulator accumulator=new BoundsAccumulator();
        for (double x : new double[]{bounds.minimum().x(), bounds.maximum().x()})
            for (double y : new double[]{bounds.minimum().y(), bounds.maximum().y()})
                for (double z : new double[]{bounds.minimum().z(), bounds.maximum().z()})
                    accumulator.include(transform.point(new Vec3d(x, y, z)));
        return accumulator.finishLoose();
    }

    /** Input and trim ownership in resolved placement chains, independent of classification hints. */
    private static boolean[] placementControlOwnership(APart part) {
        boolean[] axes = new boolean[6];
        java.util.Set<APart> visited = java.util.Collections.newSetFromMap(new java.util.IdentityHashMap<>());
        for (APart owner = part; owner != null && visited.add(owner); owner = owner.partOn) {
            if (owner.placementDefinition != null) {
                collectControlOwnership(axes, owner.placementDefinition.animations);
                String after = owner.placementDefinition.applyAfter;
                if (after != null && !after.isBlank() && owner.entityOn != null) {
                    var binding = AuthoredLiftingSurfacePoses.resolve(
                        AuthoredLiftingSurfacePoses.definitions(owner.entityOn), after, new HashSet<>());
                    axes[0] |= binding.elevator(); axes[1] |= binding.aileron(); axes[2] |= binding.rudder();
                    axes[3] |= binding.elevatorTrim(); axes[4] |= binding.aileronTrim(); axes[5] |= binding.rudderTrim();
                }
            }
            if (owner.definition != null && owner.definition.generic != null)
                collectControlOwnership(axes, owner.definition.generic.movementAnimations);
        }
        return axes;
    }

    private static void collectControlOwnership(boolean[] axes,
        List<minecrafttransportsimulator.jsondefs.JSONAnimationDefinition> animations) {
        if (animations == null) return;
        for (var animation : animations) {
            if (animation == null || animation.animationType == null || animation.variable == null) continue;
            String type = animation.animationType.name();
            if (!type.equals("ROTATION") && !type.equals("TRANSLATION")) continue;
            String variable = animation.variable.toLowerCase(java.util.Locale.ROOT);
            int trimOffset = variable.contains("trim") ? 3 : 0;
            axes[trimOffset] |= variable.contains("elevator");
            axes[trimOffset + 1] |= variable.contains("aileron");
            axes[trimOffset + 2] |= variable.contains("rudder");
        }
    }

    /** Evidence only: resolved native placement already contains these animations. */
    static AnimationHint placementHint(APart part) {
        AnimationHint hint=AnimationHint.NONE;
        java.util.Set<APart> visited=java.util.Collections.newSetFromMap(new java.util.IdentityHashMap<>());
        for (APart owner=part; owner!=null && visited.add(owner); owner=owner.partOn) {
            if (owner.placementDefinition!=null) {
                hint=hint.combine(animationHint(owner.placementDefinition.animations));
                if (owner.placementDefinition.applyAfter!=null && owner.entityOn!=null)
                    hint=hint.combine(ModelAnimationHints.animatedSurfaceHints(AuthoredLiftingSurfacePoses.definitions(owner.entityOn))
                        .getOrDefault(ModelAnimationHints.normalize(owner.placementDefinition.applyAfter), AnimationHint.NONE));
            }
            if (owner.definition!=null && owner.definition.generic!=null)
                hint=hint.combine(animationHint(owner.definition.generic.movementAnimations));
        }
        return hint;
    }

    private static AnimationHint animationHint(List<minecrafttransportsimulator.jsondefs.JSONAnimationDefinition> animations) {
        AnimationHint hint=AnimationHint.NONE;
        if (animations!=null) for (var animation : animations)
            if (animation!=null && animation.animationType!=null)
                hint=hint.combine(AnimationHint.fromAnimationType(animation.animationType.name(),
                    ModelAnimationHints.normalize(animation.variable)));
        return hint;
    }

    static boolean damageOnly(Map<String, JSONAnimatedObject> definitions, String name, java.util.Set<String> visited) {
        if (!visited.add(name)) return true;
        JSONAnimatedObject object=definitions.get(name);
        if (object==null) return false;
        return object.animations!=null && object.animations.stream().anyMatch(a -> a!=null && a.animationType!=null
            && a.animationType.name().equals("VISIBILITY") && a.variable!=null
            && a.variable.toLowerCase(java.util.Locale.ROOT).contains("damage")
            && Double.isFinite(a.clampMin) && Double.isFinite(a.clampMax) && a.clampMin<=a.clampMax
            && (a.clampMin>0 || a.clampMax<0))
            || object.applyAfter!=null && !object.applyAfter.isBlank()
                && damageOnly(definitions, object.applyAfter, visited);
    }

    private static LiftingPatch observedPatch(List<ClassifiedTriangle> triangles, SurfaceKind kind, Bounds bounds) {
        double weight=0;
        Vec3d centroid=Vec3d.ZERO;
        for (ClassifiedTriangle triangle : triangles) {
            double w=triangle.area()*Math.abs(kind==SurfaceKind.RUDDER || kind==SurfaceKind.VERTICAL_TAIL
                ? triangle.normal().x() : triangle.normal().y());
            weight+=w;
            centroid=centroid.add(triangle.centroid().scale(w));
        }
        Vec3d point=weight>EPSILON ? centroid.scale(1/weight) : triangles.get(0).centroid();
        Vec3d closest=point;
        double distance=Double.POSITIVE_INFINITY;
        for (ClassifiedTriangle triangle : triangles) {
            Vec3d candidate=ModelCoordinates.closestTrianglePoint(point, triangle.first(), triangle.second(), triangle.third());
            double d=candidate.subtract(point).lengthSquared();
            if (d<distance) { distance=d; closest=candidate; }
        }
        return new LiftingPatch("OBSERVED_"+kind, kind, closest, Math.max(EPSILON, weight));
    }

    private static LiftingPatch rename(LiftingPatch p, String name) {
        return new LiftingPatch(name, p.kind(), p.pointLocal(), p.weight(), p.symmetryPair(), p.symmetryRole(),
            p.sectionIndex(), p.sectionCount(), p.sectionMinimumRadius(), p.sectionMaximumRadius(),
            p.spanLocal(), p.chordLocal(), p.normalLocal(), p.frameConfidence(), p.aerodynamicCenterLocal());
    }
}
