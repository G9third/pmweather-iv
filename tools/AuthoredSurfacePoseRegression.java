package com.g9third.pmweatheriv.physics;

import java.util.List;
import java.util.Map;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Random;
import java.util.Set;
import java.util.UUID;
import java.nio.FloatBuffer;
import minecrafttransportsimulator.baseclasses.AnimationSwitchbox;
import minecrafttransportsimulator.baseclasses.ComputedVariable;
import minecrafttransportsimulator.entities.components.AEntityD_Definable;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartGeneric;
import minecrafttransportsimulator.jsondefs.JSONPart;
import minecrafttransportsimulator.jsondefs.JSONPartDefinition;
import minecrafttransportsimulator.jsondefs.JSONRendering;
import minecrafttransportsimulator.jsondefs.JSONSubDefinition;
import minecrafttransportsimulator.jsondefs.JSONVehicle;
import minecrafttransportsimulator.rendering.AModelParser;
import minecrafttransportsimulator.rendering.RenderableVertices;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.baseclasses.TransformationMatrix;
import minecrafttransportsimulator.jsondefs.JSONAnimatedObject;
import minecrafttransportsimulator.jsondefs.JSONAnimationDefinition;
import minecrafttransportsimulator.jsondefs.JSONAnimationDefinition.AnimationComponentType;
import minecrafttransportsimulator.rendering.DurationDelayClock;

/** Native matrix and provenance fixtures; no world, animation clock timing, or game simulation. */
public final class AuthoredSurfacePoseRegression {
    private static int assertions;
    private static final Vec3d ONE = new Vec3d(1, 1, 1);
    private static final ModelSurfaceMap.Bounds BOUNDS = new ModelSurfaceMap.Bounds(
        new Vec3d(-6, -1, -8), new Vec3d(6, 1, 8), true);

    public static void main(String[] args) throws Exception {
        provenanceAndChains();
        nativeMatricesAndScaling();
        neutralAndFrozenControls();
        legacyPivotAndColliderPose();
        attachedFramesAndArea();
        attachedSnapshotsAndRemoval();
        attachedObjectScaleAndPublicMatrices();
        completeAnimationSignature();
        System.out.println("AuthoredSurfacePoseRegression: PASS assertions=" + assertions
            + " (native matrix/provenance fixtures; no world or clock timing)");
    }

    private static void attachedFramesAndArea() {
        Random random=new Random(1134);
        for (int index=0; index<1000; ++index) {
            RotationMatrix master=new RotationMatrix().setToAngles(new Point3D(random.nextDouble()*160-80,
                random.nextDouble()*360-180, random.nextDouble()*160-80));
            RotationMatrix local=new RotationMatrix().setToAngles(new Point3D(random.nextDouble()*70-35,
                random.nextDouble()*70-35, random.nextDouble()*70-35));
            RotationMatrix world=new RotationMatrix().set(master).multiply(local);
            Vec3d scale=new Vec3d(.3+random.nextDouble()*2, .3+random.nextDouble()*2, .3+random.nextDouble()*2);
            Vec3d offset=new Vec3d(random.nextDouble()*10-5, random.nextDouble()*4-2, random.nextDouble()*10-5);
            var frame=ModelPoseMath.resolvedPart(master, world, offset, scale);
            Vec3d raw=new Vec3d(.4, .1, -.7);
            nearVec(frame.point(raw), offset.add(FlightMath.toLocal(master,
                FlightMath.toWorld(world, componentScale(raw, scale)))), "part frame matches native world orientation and cumulative scale");
            nearVec(ModelPoseMath.inverse(frame).point(frame.point(raw)), raw, "part reference inverse closes");
            var animation=AuthoredLiftingSurfacePoses.copyMatrix(pivotRotation(new Point3D(.2, 0, -.4),
                new Point3D(1, 0, 0), random.nextDouble()*40-20), ONE);
            var live=ModelPoseMath.compose(ModelPoseMath.compose(frame, animation), ModelPoseMath.inverse(frame));
            nearVec(live.point(frame.point(raw)), frame.point(animation.point(raw)), "part and object transforms compose once in renderer order");
            near(live.normal(frame.normal(new Vec3d(0,1,0))).dot(live.direction(frame.direction(new Vec3d(1,0,0)))),
                0, 1E-10, "part/object inverse transpose stays perpendicular under anisotropic scale");
            near(live.normal(frame.normal(new Vec3d(0,1,0))).dot(live.direction(frame.direction(new Vec3d(0,0,1)))),
                0, 1E-10, "part/object normal is perpendicular to both panel axes");
            var rawTriangle=new ModelGeometryData.RawTriangle(Vec3d.ZERO, new Vec3d(2,0,0), new Vec3d(0,0,1),
                new Vec3d(2.0/3,0,1.0/3), new Vec3d(0,-1,0), 7, "wing", BOUNDS);
            var transformed=AttachedLiftingSurfaces.transform(rawTriangle, frame, BOUNDS);
            double ratio=frame.direction(new Vec3d(2,0,0)).cross(frame.direction(new Vec3d(0,0,1))).length()/2;
            near(transformed.area(), 7*ratio, 1E-10, "sampled area weight follows native affine projected area");
        }
        var noEvidence=new ModelGeometryData.RawTriangle(new Vec3d(4,0,0), new Vec3d(5,0,0), new Vec3d(4,0,1),
            new Vec3d(4.3,0,.3), new Vec3d(0,1,0), .5, "primitive_008", BOUNDS);
        require(AttachedLiftingSurfaces.explicitKind(noEvidence, BOUNDS, Map.of())==null,
            "attached geometry cannot acquire lift from position alone");
        JSONAnimationDefinition damage=animation(AnimationComponentType.VISIBILITY, "damage", new Point3D());
        damage.clampMin=1; damage.clampMax=100;
        require(AttachedLiftingSurfaces.damageOnly(Map.of("damage_root",object("damage_root",null,damage),
            "wing",object("wing","damage_root")),"wing",new HashSet<>()), "damage replacement ancestry is excluded");
    }

    private static void attachedSnapshotsAndRemoval() throws Exception {
        // Real entity fields and native matrices are exercised without entity constructors,
        // a world, or animation timing. FixedSwitchbox only supplies the copied tick result.
        new AModelParser() {
            protected String getModelSuffix() { return "fixture34"; }
            protected List<RenderableVertices> parseModelInternal(String location) {
                FloatBuffer data=FloatBuffer.allocate(48);
                for (float[] vertex : new float[][]{{-1,0,0},{1,0,0},{-1,0,1},{1,0,0},{1,0,1},{-1,0,1}})
                    data.put(0).put(1).put(0).put(0).put(0).put(vertex[0]).put(vertex[1]).put(vertex[2]);
                data.flip();
                return List.of(new RenderableVertices("wing",data,false));
            }
        };
        EntityVehicleF_Physics vehicle=fixtureVehicle();
        APart left=fixturePart(vehicle, UUID.fromString("00000000-0000-0000-0000-000000000001"),0,-3);
        APart right=fixturePart(vehicle, UUID.fromString("00000000-0000-0000-0000-000000000002"),1,3);
        vehicle.allParts.add(left); vehicle.allParts.add(right);
        var base=model(List.of(patch("FALLBACK_WING_L",ModelSurfaceMap.SurfaceKind.WING,-2),
            patch("FALLBACK_WING_R",ModelSurfaceMap.SurfaceKind.WING,2),
            patch("FALLBACK_HTAIL",ModelSurfaceMap.SurfaceKind.HORIZONTAL_TAIL,0),
            patch("independent_tail",ModelSurfaceMap.SurfaceKind.HORIZONTAL_TAIL,-1)),Map.of());
        var prepared=AttachedLiftingSurfaces.append(vehicle,base);
        require(prepared.liftingPatches().stream().noneMatch(p -> p.name().startsWith("FALLBACK_WING")),
            "actual attached wing geometry removes synthetic wing fallback");
        require(prepared.liftingPatches().stream().anyMatch(p -> p.name().equals("FALLBACK_HTAIL")),
            "attached wing geometry retains unrelated authored horizontal-tail fallback");
        require(prepared.liftingPatches().stream().anyMatch(p -> p.name().equals("independent_tail")),
            "independent vehicle-authored surfaces survive attached merge");
        require(prepared.authoredPoseBindings().values().stream().map(b -> b.partReference().ownerId()).distinct().count()==2,
            "two copies of one cached OBJ retain distinct exact owner provenance");
        require(prepared.authoredPoseBindings().values().stream().allMatch(b -> b.objectName().equals("wing")),
            "native object keys do not contain generated patch names or UUID prefixes");
        require(prepared.liftingPatches().stream().map(ModelSurfaceMap.LiftingPatch::name).distinct().count()==prepared.liftingPatches().size(),
            "attached patch names are unique across installed copies");
        var geometry=new AirframeGeometry.Geometry(2,2,10,20,12,2,0,4,4,1,4,2,1,0,7.2);
        var area=AirframeGeometry.SurfaceAreaPlan.from(prepared,geometry);
        near(prepared.liftingPatches().stream().filter(p -> p.kind()==ModelSurfaceMap.SurfaceKind.WING).mapToDouble(area::areaFor).sum(),
            20,1E-12,"all installed wing copies share one authored vehicle area");
        var mainWing=patch("main_model_wing",ModelSurfaceMap.SurfaceKind.WING,0);
        var withMainWing=AttachedLiftingSurfaces.append(vehicle,model(List.of(mainWing),Map.of()));
        require(withMainWing.liftingPatches().stream().anyMatch(p -> p.name().equals("main_model_wing")),
            "real main-model lifting geometry remains beside installed surfaces");
        var combinedArea=AirframeGeometry.SurfaceAreaPlan.from(withMainWing,geometry);
        near(withMainWing.liftingPatches().stream().filter(p -> p.kind()==ModelSurfaceMap.SurfaceKind.WING)
            .mapToDouble(combinedArea::areaFor).sum(),20,1E-12,
            "main-model and installed surfaces divide one authored vehicle area");
        AircraftState state=new AircraftState();
        var snapshot=AuthoredLiftingSurfacePoses.snapshot(vehicle,prepared,state);
        for (var patch : prepared.liftingPatches()) {
            if (prepared.authoredPoseBindings().get(patch.name())==null) continue;
            var pose=snapshot.patches().get(patch.name());
            require(pose.live() && pose.visible(), "healthy exact attached owner has a live native pose");
            nearVec(pose.relativePointVelocity(patch.pointLocal()),Vec3d.ZERO,"first attached snapshot has no invented velocity");
        }
        Vec3d shift=new Vec3d(.1,.02,-.03);
        left.position.add(shift.x(),shift.y(),shift.z());
        vehicle.ticksExisted++; left.ticksExisted++; right.ticksExisted++;
        snapshot=AuthoredLiftingSurfacePoses.snapshot(vehicle,prepared,state);
        var leftPatch=prepared.liftingPatches().stream().filter(p -> {
            var b=prepared.authoredPoseBindings().get(p.name()); return b!=null && b.partReference().ownerId().equals(left.uniqueUUID);
        }).findFirst().orElseThrow();
        nearVec(snapshot.patches().get(leftPatch.name()).relativePointVelocity(leftPatch.pointLocal()),
            FlightMath.toLocal(vehicle.orientation,shift).scale(20),"resolved placement motion enters point velocity exactly once");
        left.isInvisible=true; vehicle.ticksExisted++; left.ticksExisted++; right.ticksExisted++;
        require(!AuthoredLiftingSurfacePoses.snapshot(vehicle,prepared,state).patches().get(leftPatch.name()).visible(),
            "invisible attached owner has no lifting force");
        left.isInvisible=false; vehicle.ticksExisted++; left.ticksExisted++; right.ticksExisted++;
        nearVec(AuthoredLiftingSurfacePoses.snapshot(vehicle,prepared,state).patches().get(leftPatch.name()).relativePointVelocity(leftPatch.pointLocal()),
            Vec3d.ZERO,"reactivated part restarts rate history");
        left.outOfHealth=true; vehicle.ticksExisted++; left.ticksExisted++; right.ticksExisted++;
        require(!AuthoredLiftingSurfacePoses.snapshot(vehicle,prepared,state).patches().get(leftPatch.name()).visible(),
            "destroyed attached owner has no lifting force");
        left.outOfHealth=false;
        left.isActiveVar.isActive=false; vehicle.ticksExisted++; left.ticksExisted++; right.ticksExisted++;
        require(!AuthoredLiftingSurfacePoses.snapshot(vehicle,prepared,state).patches().get(leftPatch.name()).visible(),
            "inactive attached owner has no lifting force");
        left.isActiveVar.isActive=true;
        left.isExteriorVar.isActive=false; vehicle.ticksExisted++; left.ticksExisted++; right.ticksExisted++;
        require(!AuthoredLiftingSurfacePoses.snapshot(vehicle,prepared,state).patches().get(leftPatch.name()).visible(),
            "non-exterior attached owner has no lifting force");
        left.isExteriorVar.isActive=true;
        left.animatedObjectSwitchboxes.put("wing",new FixedSwitchbox(left,new TransformationMatrix(),false));
        vehicle.ticksExisted++; left.ticksExisted++; right.ticksExisted++;
        require(!AuthoredLiftingSurfacePoses.snapshot(vehicle,prepared,state).patches().get(leftPatch.name()).visible(),
            "static part object native visibility is respected");
        left.animatedObjectSwitchboxes.clear();
        vehicle.allParts.remove(left);
        var rebuilt=AttachedLiftingSurfaces.append(vehicle,base);
        var removed=AttachedLiftingSurfaces.retainMissing(rebuilt,prepared,Set.of(right.uniqueUUID));
        var removedAreas=AirframeGeometry.SurfaceAreaPlan.from(removed,geometry);
        double missingShare=removed.liftingPatches().stream().filter(p -> {
            var b=removed.authoredPoseBindings().get(p.name()); return b!=null && b.status()==AuthoredLiftingSurfacePoses.Status.HIDDEN;
        }).mapToDouble(removedAreas::areaFor).sum();
        near(missingShare,10,1E-12,"missing wing retains its own area share instead of giving it to its survivor");
        require(removed.liftingPatches().stream().noneMatch(p -> p.name().startsWith("FALLBACK_WING")),
            "detach cannot recreate phantom wing fallback");
        require(AttachedLiftingSurfaces.retainMissing(rebuilt,removed,Set.of(right.uniqueUUID)).liftingPatches().size()==removed.liftingPatches().size(),
            "missing part state survives later structural refreshes without duplication");
        vehicle.ticksExisted++; right.ticksExisted++;
        require(!AuthoredLiftingSurfacePoses.snapshot(vehicle,removed,state).patches().get(leftPatch.name()).visible(),
            "missing exact owner remains hidden during solve");
        APart replacement=fixturePart(vehicle,UUID.fromString("00000000-0000-0000-0000-000000000003"),0,-3);
        vehicle.allParts.add(replacement);
        var replaced=AttachedLiftingSurfaces.retainMissing(AttachedLiftingSurfaces.append(vehicle,base),removed,
            Set.of(right.uniqueUUID,replacement.uniqueUUID));
        require(replaced.authoredPoseBindings().values().stream().noneMatch(b -> b.partReference().ownerId().equals(left.uniqueUUID)),
            "replacement in the same authored slot retires old UUID and area share");
        near(replaced.liftingPatches().stream().filter(p -> p.kind()==ModelSurfaceMap.SurfaceKind.WING)
            .mapToDouble(AirframeGeometry.SurfaceAreaPlan.from(replaced,geometry)::areaFor).sum(),20,1E-12,
            "replacement restores one conserved wing budget");
        String stable=AttachedLiftingSurfaces.signature(vehicle);
        replacement.position.add(10,0,0);
        require(AttachedLiftingSurfaces.signature(vehicle).equals(stable),"live placement translation does not invalidate immutable geometry");
        replacement.scale.x*=2;
        require(!AttachedLiftingSurfaces.signature(vehicle).equals(stable),"part model scale invalidates geometry");
        String scaleSignature=AttachedLiftingSurfaces.signature(vehicle);
        replacement.placementDefinition.pos=new Point3D(0.25,0,0);
        require(!AttachedLiftingSurfaces.signature(vehicle).equals(scaleSignature),"authored part placement position invalidates geometry");
        String positionSignature=AttachedLiftingSurfaces.signature(vehicle);
        replacement.placementDefinition.partScale=new Point3D(1.5,1,1);
        require(!AttachedLiftingSurfaces.signature(vehicle).equals(positionSignature),"authored part placement scale invalidates geometry");
        String placementScaleSignature=AttachedLiftingSurfaces.signature(vehicle);
        replacement.placementDefinition.rot=new RotationMatrix().setToAngles(new Point3D(10,20,30));
        require(!AttachedLiftingSurfaces.signature(vehicle).equals(placementScaleSignature),"authored part placement rotation invalidates geometry");
        String hierarchySignature=AttachedLiftingSurfaces.signature(vehicle);
        setField(replacement,"partOn",right);
        require(!AttachedLiftingSurfaces.signature(vehicle).equals(hierarchySignature),"part parent changes invalidate geometry provenance");
        require(AttachedLiftingSurfaces.slotPath(replacement).equals("1/0/"),"nested authored slots retain ordered placement provenance");
        right.outOfHealth=true;
        require(!AttachedLiftingSurfaces.active(replacement),"unhealthy parent gates nested lifting part");
        APart nested=fixturePart(vehicle,UUID.fromString("00000000-0000-0000-0000-000000000004"),2,7);
        setField(nested,"partOn",left);
        setField(nested,"localOffset",new Point3D(-99,3,42));
        var nestedFrame=AttachedLiftingSurfaces.resolvedFrame(vehicle,nested);
        nearVec(nestedFrame.translation(),FlightMath.toLocal(vehicle.orientation,
            new Vec3d(nested.position.x-vehicle.position.x,nested.position.y-vehicle.position.y,nested.position.z-vehicle.position.z)),
            "nested part geometry uses resolved native position rather than re-originated localOffset");
    }

    private static void attachedObjectScaleAndPublicMatrices() throws Exception {
        EntityVehicleF_Physics vehicle=fixtureVehicle();
        APart part=fixturePart(vehicle,UUID.fromString("00000000-0000-0000-0000-000000000005"),0,-2);
        part.scale.x=2; part.scale.y=1; part.scale.z=.5;
        vehicle.allParts.add(part);
        var matrix=pivotRotation(new Point3D(.2,0,.1),new Point3D(1,0,0),17);
        FixedSwitchbox switchbox=new FixedSwitchbox(part,matrix,true);
        part.animatedObjectSwitchboxes.put("wing",switchbox);
        var prepared=AttachedLiftingSurfaces.append(vehicle,model(List.of(),Map.of()));
        var patch=prepared.liftingPatches().stream().filter(p -> {
            var binding=prepared.authoredPoseBindings().get(p.name());
            return binding!=null && binding.partReference().ownerId().equals(part.uniqueUUID);
        }).findFirst().orElseThrow();
        var reference=prepared.authoredPoseBindings().get(patch.name()).partReference().inverseReference();
        var frame=ModelPoseMath.inverse(reference);
        Vec3d scale=new Vec3d(part.scale.x,part.scale.y,part.scale.z);
        var live=AuthoredLiftingSurfacePoses.liveMatrix(part,"wing",scale);
        var copied=AuthoredLiftingSurfacePoses.copyMatrix(matrix,scale);
        Vec3d rawPatchPoint=reference.point(patch.pointLocal());
        nearVec(live.point(rawPatchPoint),copied.point(rawPatchPoint),
            "public owner liveMatrix and part pose use one scaled native matrix");
        AircraftState state=new AircraftState();
        var pose=AuthoredLiftingSurfacePoses.snapshot(vehicle,prepared,state).patches().get(patch.name());
        var expected=ModelPoseMath.compose(ModelPoseMath.compose(frame,copied),reference);
        nearVec(pose.transform().point(patch.pointLocal()),expected.point(patch.pointLocal()),
            "attached surface pose conjugates native animation through anisotropic part scale");
        TransformationMatrix nextMatrix=pivotRotation(new Point3D(.2,0,.1),new Point3D(1,0,0),23);
        switchbox.netMatrix.set(nextMatrix);
        ++vehicle.ticksExisted; ++part.ticksExisted;
        var nextPose=AuthoredLiftingSurfacePoses.snapshot(vehicle,prepared,state).patches().get(patch.name());
        var nextCopied=AuthoredLiftingSurfacePoses.copyMatrix(nextMatrix,scale);
        var nextExpected=ModelPoseMath.compose(ModelPoseMath.compose(frame,nextCopied),reference);
        nearVec(nextPose.relativePointVelocity(patch.pointLocal()),
            nextExpected.point(patch.pointLocal()).subtract(expected.point(patch.pointLocal())).scale(20),
            "attached surface rate matches consecutive scaled native poses");

        var bodyDefinition=object("body",null,animation(AnimationComponentType.ROTATION,"rudder",new Point3D(0,1,0)));
        vehicle.definition.rendering=new JSONRendering();
        vehicle.definition.rendering.animatedObjects=List.of(bodyDefinition);
        vehicle.animatedObjectDefinitions.put("body",bodyDefinition);
        var bodyMatrix=pivotRotation(new Point3D(0,0,.3),new Point3D(0,1,0),-11);
        vehicle.animatedObjectSwitchboxes.put("body",new FixedSwitchbox(vehicle,bodyMatrix,true));
        setField(vehicle,"scale",new Point3D(1.5,.75,2));
        var publicVehicle=AuthoredLiftingSurfacePoses.liveMatrix(vehicle,"body");
        var explicitVehicle=AuthoredLiftingSurfacePoses.liveMatrix(vehicle,"body",ModelCoordinates.scale(vehicle));
        Vec3d sample=new Vec3d(.4,-.2,1.3);
        nearVec(publicVehicle.point(sample),explicitVehicle.point(sample),
            "public vehicle liveMatrix shares the explicit scaled-owner pose API");
    }

    private static void completeAnimationSignature() {
        var animated=object("wing",null,animation(AnimationComponentType.ROTATION,"elevator",new Point3D(1,0,0)));
        Map<String,JSONAnimatedObject> definitions=Map.of("wing",animated);
        String previous=AuthoredLiftingSurfacePoses.signature(definitions);
        animated.animations.get(0).centerPoint.x=.5;
        require(!previous.equals(AuthoredLiftingSurfacePoses.signature(definitions)),"authored pivot invalidates pose geometry");
        previous=AuthoredLiftingSurfacePoses.signature(definitions);
        animated.animations.get(0).clampMax=30;
        require(!previous.equals(AuthoredLiftingSurfacePoses.signature(definitions)),"authored clamp invalidates pose geometry");
        previous=AuthoredLiftingSurfacePoses.signature(definitions);
        animated.animations.get(0).duration=8;
        require(!previous.equals(AuthoredLiftingSurfacePoses.signature(definitions)),"authored clock timing invalidates pose geometry");
        var binding=new AuthoredLiftingSurfacePoses.Binding("wing",false,false,false,false,false,AuthoredLiftingSurfacePoses.Status.AUTHORED);
        var next=new AuthoredLiftingSurfacePoses.Pose(AnimatedWingGeometry.RigidTransform.IDENTITY,binding,AuthoredLiftingSurfacePoses.Status.AUTHORED);
        var rebound=new AuthoredLiftingSurfacePoses.Binding("other",false,false,false,false,false,AuthoredLiftingSurfacePoses.Status.AUTHORED);
        var previousPose=new AuthoredLiftingSurfacePoses.Pose(AuthoredLiftingSurfacePoses.copyMatrix(new TransformationMatrix().applyTranslation(1,0,0),ONE),
            rebound,AuthoredLiftingSurfacePoses.Status.AUTHORED);
        nearVec(next.withPrevious(previousPose).relativePointVelocity(Vec3d.ZERO),Vec3d.ZERO,"rebound provenance does not invent a rate");
    }

    private static ModelSurfaceMap.PreparedModel model(List<ModelSurfaceMap.LiftingPatch> patches,
        Map<String,AuthoredLiftingSurfacePoses.Binding> bindings) {
        return new ModelSurfaceMap.PreparedModel("fixture",true,"fixture",BOUNDS,BOUNDS,List.of(),patches,0,0,0,0,bindings);
    }
    private static class FixturePartDefinition extends JSONPart {
        public String getModelLocation(JSONSubDefinition sub) { return "attached.fixture34"; }
    }
    private static class FixedSwitchbox extends AnimationSwitchbox {
        private final boolean visible;
        FixedSwitchbox(AEntityD_Definable<?> owner, TransformationMatrix matrix, boolean visible) {
            super(owner,List.of(),null); this.netMatrix.set(matrix); this.visible=visible;
        }
        public boolean runSwitchbox(float partialTicks,boolean force) { return visible; }
    }
    private static EntityVehicleF_Physics fixtureVehicle() throws Exception {
        var vehicle=allocate(EntityVehicleF_Physics.class);
        setField(vehicle,"uniqueUUID",UUID.fromString("00000000-0000-0000-0000-000000000100"));
        setField(vehicle,"definition",new JSONVehicle());
        setField(vehicle,"allParts",new ArrayList<APart>());
        setField(vehicle,"position",new Point3D(100,40,-70));
        setField(vehicle,"orientation",new RotationMatrix());
        setField(vehicle,"scale",new Point3D(1,1,1));
        setField(vehicle,"animatedObjectDefinitions",new HashMap<String,JSONAnimatedObject>());
        setField(vehicle,"animatedObjectSwitchboxes",new HashMap<String,AnimationSwitchbox>());
        setField(vehicle,"wingSpanVar",new ComputedVariable(true)); vehicle.wingSpanVar.currentValue=12;
        setField(vehicle,"wingAreaVar",new ComputedVariable(true)); vehicle.wingAreaVar.currentValue=20;
        return vehicle;
    }
    private static APart fixturePart(EntityVehicleF_Physics vehicle, UUID id, int slot, double x) throws Exception {
        var part=allocate(PartGeneric.class);
        setField(part,"uniqueUUID",id); part.isValid=true;
        setField(part,"placementSlot",slot);
        setField(part,"placementDefinition",new JSONPartDefinition());
        setField(part,"entityOn",vehicle); setField(part,"masterEntity",vehicle); setField(part,"vehicleOn",vehicle);
        var definition=new FixturePartDefinition(); definition.generic=new JSONPart.JSONPartGeneric();
        definition.rendering=new JSONRendering(); definition.rendering.animatedObjects=List.of();
        setField(part,"definition",definition);
        setField(part,"position",new Point3D(vehicle.position.x+x,vehicle.position.y,vehicle.position.z));
        setField(part,"orientation",new RotationMatrix()); setField(part,"scale",new Point3D(1,1,1));
        setField(part,"animatedObjectDefinitions",new HashMap<String,JSONAnimatedObject>());
        setField(part,"animatedObjectSwitchboxes",new HashMap<String,AnimationSwitchbox>());
        setField(part,"isActiveVar",new ComputedVariable(true)); setField(part,"isExteriorVar",new ComputedVariable(true));
        return part;
    }
    @SuppressWarnings("unchecked")
    private static <T> T allocate(Class<T> type) throws Exception {
        var field=sun.misc.Unsafe.class.getDeclaredField("theUnsafe"); field.setAccessible(true);
        return (T)((sun.misc.Unsafe)field.get(null)).allocateInstance(type);
    }
    private static void setField(Object object,String name,Object value) throws Exception {
        for (Class<?> owner=object.getClass();owner!=null;owner=owner.getSuperclass()) try {
            var field=owner.getDeclaredField(name); field.setAccessible(true); field.set(object,value); return;
        } catch (NoSuchFieldException absent) { }
        throw new NoSuchFieldException(name);
    }

    private static void provenanceAndChains() {
        JSONAnimatedObject parent = object("parent_frame", null,
            animation(AnimationComponentType.ROTATION, "elevator", new Point3D(2, 0, 0)));
        JSONAnimatedObject child = object("primitive_004", "parent_frame",
            animation(AnimationComponentType.ROTATION, "aileron", new Point3D(-1, 0, 0)));
        var patch = patch("synthetic_left_tail", ModelSurfaceMap.SurfaceKind.TAILERON, -2);
        var triangle = triangle(child.objectName, patch.kind(), -2);
        var bindings = AuthoredLiftingSurfacePoses.bind(Map.of(parent.objectName, parent, child.objectName, child),
            List.of(patch), Map.of(patch.kind(), List.of(triangle)), BOUNDS);
        var binding = bindings.get(patch.name());
        require(binding.status() == AuthoredLiftingSurfacePoses.Status.AUTHORED,
            "applyAfter chain binds actual triangle object, independent of synthetic patch name");
        require(binding.elevator() && binding.aileron(), "parent and child physical controls retained");
        require(binding.objectName().equals(child.objectName), "matrix owner is the final chain object");

        JSONAnimatedObject inheritedSkin = object("primitive_005", "parent_frame",
            animation(AnimationComponentType.VISIBILITY, "show_aileron", new Point3D()));
        var inheritedHints = ModelAnimationHints.animatedSurfaceHints(Map.of("parent_frame", parent,
            inheritedSkin.objectName, inheritedSkin));
        require(inheritedHints.get(inheritedSkin.objectName).elevator()
            && !inheritedHints.get(inheritedSkin.objectName).aileron(),
            "name-free skin inherits parent physical evidence without its visibility token");

        JSONAnimatedObject base = object("active_control", null,
            animation(AnimationComponentType.ROTATION, "rudder", new Point3D(0, 1, 0)));
        JSONAnimatedObject active = object("active_control", null,
            animation(AnimationComponentType.ROTATION, "elevator", new Point3D(1, 0, 0)));
        var activeDefinitions = AuthoredLiftingSurfacePoses.definitions(List.of(base), Map.of("active_control", active));
        var activeHint = ModelAnimationHints.animatedSurfaceHints(activeDefinitions).get("active_control");
        require(activeDefinitions.get("active_control") == active && activeHint.elevator() && !activeHint.rudder(),
            "resolved runtime definition overrides stale base control evidence");
        var activeBinding = AuthoredLiftingSurfacePoses.bind(activeDefinitions, List.of(patch),
            Map.of(patch.kind(), List.of(triangle("active_control", patch.kind(), -2))), BOUNDS).get(patch.name());
        require(activeBinding.elevator() && !activeBinding.rudder(), "pose binding uses the same resolved control definition");
        require(AuthoredLiftingSurfacePoses.definitions(List.of(base), Map.of()).get("active_control") == base,
            "authored definition remains available before runtime activation");

        var mixed = AuthoredLiftingSurfacePoses.bind(Map.of(parent.objectName, parent, child.objectName, child),
            List.of(patch), Map.of(patch.kind(), List.of(triangle, triangle("static_skin", patch.kind(), -2))), BOUNDS);
        require(mixed.get(patch.name()).status() == AuthoredLiftingSurfacePoses.Status.FROZEN_MIXED_SOURCES,
            "static/moving patch cannot borrow an unrelated hinge");

        JSONAnimatedObject skin1 = object("skin1", "parent_frame");
        JSONAnimatedObject skin2 = object("skin2", "parent_frame");
        var coherent = AuthoredLiftingSurfacePoses.bind(Map.of(parent.objectName, parent, "skin1", skin1, "skin2", skin2),
            List.of(patch), Map.of(patch.kind(), List.of(triangle("skin1", patch.kind(), -2),
                triangle("skin2", patch.kind(), -2))), BOUNDS).get(patch.name());
        require(coherent.status() == AuthoredLiftingSurfacePoses.Status.AUTHORED
            && coherent.objectName().equals(parent.objectName), "two static children share their actual authored parent");

        JSONAnimatedObject cycle = object("cycle", "cycle",
            animation(AnimationComponentType.ROTATION, "elevator", new Point3D(1, 0, 0)));
        var invalid = bindOne(patch, cycle);
        require(invalid.status() == AuthoredLiftingSurfacePoses.Status.FROZEN_UNSUPPORTED_CHAIN, "cycle freezes");
        JSONAnimatedObject scaling = object("scaling", null,
            animation(AnimationComponentType.SCALING, "elevator", new Point3D(1, 0, 0)));
        require(bindOne(patch, scaling).status() == AuthoredLiftingSurfacePoses.Status.FROZEN_UNSUPPORTED_CHAIN,
            "dynamic authored scaling is explicit unsupported fallback");

        JSONAnimatedObject visible = object("visible", null,
            animation(AnimationComponentType.VISIBILITY, "elevator", new Point3D(1, 0, 0)));
        var staticBinding = bindOne(patch, visible);
        require(staticBinding.status() == AuthoredLiftingSurfacePoses.Status.STATIC && !staticBinding.elevator(),
            "visibility variable is not physical control motion");

        // Name-free WING geometry and an authored vertical flap rotation are sufficient sweep evidence.
        var wing = patch("prepared_section", ModelSurfaceMap.SurfaceKind.WING, -2);
        JSONAnimatedObject sweep = object("primitive_099", null,
            animation(AnimationComponentType.ROTATION, "flap_actual", new Point3D(0, 1, 0)));
        require(bindOne(wing, sweep).flapSweep(), "whole-wing sweep uses physical axis and provenance");
        JSONAnimatedObject flap = object("primitive_100", null,
            animation(AnimationComponentType.ROTATION, "flap_actual", new Point3D(1, 0, 0)));
        require(!bindOne(wing, flap).flapSweep(), "ordinary chord hinge is not flap-driven wing sweep");
    }

    private static void nativeMatricesAndScaling() {
        JSONAnimationDefinition definition = animation(AnimationComponentType.ROTATION, "elevator", new Point3D(2, 0, 0));
        DurationDelayClock clock = new DurationDelayClock(definition);
        near(clock.animationAxisMagnitude, 2, 0, "IV retains authored axis magnitude");
        TransformationMatrix parent = pivotRotation(new Point3D(-2, .2, -3),
            clock.animationAxisNormalized, 10 * clock.animationAxisMagnitude);
        TransformationMatrix child = pivotRotation(new Point3D(-2, 0, -4), new Point3D(0, 1, 0), -15);
        child.applyTranslation(.3, .2, -.1);
        // IV applyAfter copies the parent netMatrix then right-multiplies each local operation.
        TransformationMatrix chain = new TransformationMatrix(parent).multiply(child);
        Vec3d scale = new Vec3d(2, .5, 1.3);
        var copied = AuthoredLiftingSurfacePoses.copyMatrix(chain, scale);
        require(copied != null, "finite composed native matrix accepted");
        Vec3d authoredPoint = new Vec3d(-2.4, .1, -4.3);
        Vec3d modelPoint = componentScale(authoredPoint, scale);
        Point3D nativeResult = chain.transform(new Point3D(authoredPoint.x(), authoredPoint.y(), authoredPoint.z()));
        nearVec(copied.point(modelPoint), componentScale(new Vec3d(nativeResult.x, nativeResult.y, nativeResult.z), scale),
            "copied pose matches native applyAfter composition in anisotropic solver coordinates");
        Vec3d span = new Vec3d(1, 0, 0), chord = new Vec3d(0, 0, 1), normal = new Vec3d(0, 1, 0);
        Vec3d liveNormal = copied.normal(normal).normalized();
        near(liveNormal.dot(copied.direction(span)), 0, 1E-12, "normal inverse transpose remains perpendicular to span");
        near(liveNormal.dot(copied.direction(chord)), 0, 1E-12, "normal inverse transpose remains perpendicular to chord");
        Vec3d beforeRenderUpdate = copied.point(modelPoint);
        chain.resetTransforms().applyTranslation(100, 100, 100);
        nearVec(copied.point(modelPoint), beforeRenderUpdate, "copied solver matrix cannot mutate with renderer matrix");

        TransformationMatrix invalid = new TransformationMatrix();
        invalid.m03 = Double.NaN;
        require(AuthoredLiftingSurfacePoses.copyMatrix(invalid, ONE) == null, "nonfinite native matrix freezes");
        invalid.resetTransforms().applyScaling(1, 0, 1);
        require(AuthoredLiftingSurfacePoses.copyMatrix(invalid, ONE) == null, "singular matrix freezes");
        invalid.resetTransforms();
        invalid.m30 = .1;
        require(AuthoredLiftingSurfacePoses.copyMatrix(invalid, ONE) == null, "projective matrix freezes");
        require(AuthoredLiftingSurfacePoses.copyMatrix(new TransformationMatrix(), new Vec3d(0, 1, 1)) == null,
            "invalid model scale freezes");
    }

    private static void legacyPivotAndColliderPose() throws Exception {
        var left = new AnimatedWingGeometry.WingRoot("left", "flap_actual", new Vec3d(-2, 0, 0),
            new Vec3d(0, 1, 0), 0, 0, 60);
        var right = new AnimatedWingGeometry.WingRoot("right", "flap_actual", new Vec3d(2, 0, 0),
            new Vec3d(0, 1, 0), 0, 0, 60);
        var nativeMatrix = pivotRotation(new Point3D(2, 0, 0), new Point3D(0, 1, 0), 30)
            .applyTranslation(.1, .2, -.3);
        var nativePose = AuthoredLiftingSurfacePoses.copyMatrix(nativeMatrix, ONE);
        var constructor = AnimatedWingGeometry.RuntimeState.class.getDeclaredConstructor(List.class);
        constructor.setAccessible(true);
        var runtime = constructor.newInstance(List.of(
            new AnimatedWingGeometry.RootTransform(left, AnimatedWingGeometry.RigidTransform.IDENTITY, 10),
            new AnimatedWingGeometry.RootTransform(right, nativePose, 10)));
        var straddle = new ModelSurfaceMap.LiftingPatch("legacy_straddle", ModelSurfaceMap.SurfaceKind.WING,
            new Vec3d(2, 0, 0), 1, 0, ModelSurfaceMap.SymmetryRole.MIRRORED_RIGHT, 0, 2, 1, 3);
        require(runtime.transformForPatch(straddle).equals(AnimatedWingGeometry.RigidTransform.IDENTITY),
            "straddling legacy strip freezes rather than scaling raw angle or dropping native chain motion");
        var outboard = new ModelSurfaceMap.LiftingPatch("legacy_outboard", ModelSurfaceMap.SurfaceKind.WING,
            new Vec3d(4, 0, 0), 1, 0, ModelSurfaceMap.SymmetryRole.MIRRORED_RIGHT, 1, 2, 3, 5);
        nearVec(runtime.transformForPatch(outboard).point(outboard.pointLocal()), nativePose.point(outboard.pointLocal()),
            "coherent outboard strip consumes the full native matrix once");
        require(nativePose.isRigid(), "uniform-scale native chain is valid for existing rigid collider child");
        var quaternion = com.g9third.pmweatheriv.sable.SablePoseConversions.toQuaternion(nativePose.rotationMatrix());
        var local = new Vec3d(1, .3, -.4);
        var rotated = quaternion.transform(new org.joml.Vector3d(local.x(), local.y(), local.z()));
        nearVec(new Vec3d(rotated.x, rotated.y, rotated.z), nativePose.direction(local),
            "collider quaternion matches full native rotation, not raw angle/normalized axis");
        var affine = AuthoredLiftingSurfacePoses.copyMatrix(nativeMatrix, new Vec3d(2, 1, .5));
        require(!affine.isRigid(), "anisotropic solver transform cannot be passed as a quaternion collider");
    }

    private static void neutralAndFrozenControls() {
        var binding = new AuthoredLiftingSurfacePoses.Binding("physical_control", true, true, false,
            false, false, AuthoredLiftingSurfacePoses.Status.AUTHORED);
        var pose = new AuthoredLiftingSurfacePoses.Pose(AnimatedWingGeometry.RigidTransform.IDENTITY,
            binding, AuthoredLiftingSurfacePoses.Status.AUTHORED);
        near(pose.control(-.2, .1, 0), 0, 0, "mixed pitch/roll authored geometry receives no duplicate scalar control");
        near(pose.control(-.2, .1, .03), .03, 1E-12, "unrepresented rudder axis keeps its approximation");
        var frozen = new AuthoredLiftingSurfacePoses.Pose(AnimatedWingGeometry.RigidTransform.IDENTITY,
            binding, AuthoredLiftingSurfacePoses.Status.FROZEN_UNAVAILABLE);
        near(frozen.control(-.2, .1, 0), -.1, 1E-12, "frozen fallback keeps bounded scalar controls");
        near(pose.areaFactor(patch("panel", ModelSurfaceMap.SurfaceKind.TAILERON, -2)), 1, 1E-12,
            "neutral identity matrix preserves patch area");
        require(!new AuthoredLiftingSurfacePoses.Pose(AnimatedWingGeometry.RigidTransform.IDENTITY,
            binding, AuthoredLiftingSurfacePoses.Status.HIDDEN).visible(), "hidden authored panel supplies no force");
    }

    private static AuthoredLiftingSurfacePoses.Binding bindOne(ModelSurfaceMap.LiftingPatch patch, JSONAnimatedObject object) {
        return AuthoredLiftingSurfacePoses.bind(Map.of(object.objectName, object), List.of(patch),
            Map.of(patch.kind(), List.of(triangle(object.objectName, patch.kind(), patch.pointLocal().x()))), BOUNDS).get(patch.name());
    }
    private static ModelSurfaceMap.LiftingPatch patch(String name, ModelSurfaceMap.SurfaceKind kind, double x) {
        return new ModelSurfaceMap.LiftingPatch(name, kind, new Vec3d(x, 0, -4), 1, 0,
            x < 0 ? ModelSurfaceMap.SymmetryRole.MIRRORED_LEFT : ModelSurfaceMap.SymmetryRole.MIRRORED_RIGHT);
    }
    private static ModelGeometryData.ClassifiedTriangle triangle(String name, ModelSurfaceMap.SurfaceKind kind, double x) {
        Vec3d a = new Vec3d(x, 0, -4), b = new Vec3d(x + .1, 0, -4), c = new Vec3d(x, 0, -3.9);
        return new ModelGeometryData.ClassifiedTriangle(a, b, c, a.add(b).add(c).scale(1.0 / 3),
            new Vec3d(0, 1, 0), .005, name, kind);
    }
    private static JSONAnimatedObject object(String name, String parent, JSONAnimationDefinition... animations) {
        JSONAnimatedObject result = new JSONAnimatedObject();
        result.objectName = name; result.applyAfter = parent; result.animations = List.of(animations);
        return result;
    }
    private static JSONAnimationDefinition animation(AnimationComponentType type, String variable, Point3D axis) {
        JSONAnimationDefinition result = new JSONAnimationDefinition();
        result.animationType = type; result.variable = variable; result.axis = axis; result.centerPoint = new Point3D();
        return result;
    }
    private static TransformationMatrix pivotRotation(Point3D pivot, Point3D axis, double angle) {
        return new TransformationMatrix().applyTranslation(pivot)
            .applyRotation(new RotationMatrix().setToAxisAngle(axis, angle)).applyInvertedTranslation(pivot);
    }
    private static Vec3d componentScale(Vec3d point, Vec3d scale) {
        return new Vec3d(point.x() * scale.x(), point.y() * scale.y(), point.z() * scale.z());
    }
    private static void nearVec(Vec3d actual, Vec3d expected, String message) {
        near(actual.subtract(expected).length(), 0, 1E-11, message);
    }
    private static void near(double actual, double expected, double tolerance, String message) {
        require(Double.isFinite(actual) && Math.abs(actual - expected) <= tolerance,
            message + " actual=" + actual + " expected=" + expected);
    }
    private static void require(boolean condition, String message) {
        ++assertions;
        if (!condition) throw new AssertionError(message);
    }
}
