import java.nio.FloatBuffer;
import java.util.*;
import java.lang.reflect.*;
import com.g9third.pmweatheriv.sable.SableModelCollisionHull;
import com.g9third.pmweatheriv.physics.AnimatedWingGeometry.RigidTransform;
import com.g9third.pmweatheriv.physics.Vec3d;
import com.g9third.pmweatheriv.model.ParsedModelSnapshot.Mesh;
import minecrafttransportsimulator.rendering.RenderableVertices;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.jsondefs.*;
import minecrafttransportsimulator.jsondefs.JSONAnimationDefinition.AnimationComponentType;
import org.joml.Vector3d;
import org.joml.Quaterniond;
import java.nio.file.Files;
import java.nio.file.Path;
import minecrafttransportsimulator.packloading.JSONParser;

/** Production voxel/matrix helpers; no native scene or in-game assembly is simulated. */
public final class MovingModelHullRegression {
    private static int assertions;
    private static void require(boolean v,String msg) { ++assertions; if(!v) throw new AssertionError(msg); }
    private static void near(double a,double b,double eps,String msg) { require(Double.isFinite(a)&&Double.isFinite(b)&&Math.abs(a-b)<=eps,msg+": "+a+" vs "+b); }
    private static Method method(Class<?> c,String name,int count) {
        for(var m:c.getDeclaredMethods())if(m.getName().equals(name)&&m.getParameterCount()==count){m.setAccessible(true);return m;}
        throw new AssertionError(name);
    }
    private static JSONAnimatedObject object(String name,String parent,AnimationComponentType type) {
        var o=new JSONAnimatedObject();o.objectName=name;o.applyAfter=parent;
        var a=new JSONAnimationDefinition();a.animationType=type;a.variable="actuator";a.axis=new Point3D(1,0,0);o.animations=List.of(a);return o;
    }
    private static Mesh mesh(String name,Vec3d... points) {
        FloatBuffer b=FloatBuffer.allocate(points.length*8);
        for(var p:points){for(int i=0;i<5;++i)b.put(0);b.put((float)p.x()).put((float)p.y()).put((float)p.z());}
        b.flip();try {var constructor=Mesh.class.getDeclaredConstructor(RenderableVertices.class);constructor.setAccessible(true);return constructor.newInstance(new RenderableVertices(name,b,false));}catch(Exception e){throw new AssertionError(e);}
    }
    private static void eligibility()throws Exception {
        var parent=object("body_frame",null,AnimationComponentType.ROTATION);
        var door=object("door_skin","body_frame",AnimationComponentType.VISIBILITY);
        var cycle=object("cycle","cycle",AnimationComponentType.ROTATION);
        var bad=object("bad","missing",AnimationComponentType.ROTATION);
        var scaling=object("scale",null,AnimationComponentType.SCALING);
        Map<String,JSONAnimatedObject> definitions=Map.of("body_frame",parent,"door_skin",door,"cycle",cycle,"bad",bad,"scale",scaling,
            "wheel",object("wheel",null,AnimationComponentType.ROTATION),"headlight",object("headlight",null,AnimationComponentType.ROTATION));
        Method classify=SableModelCollisionHull.class.getDeclaredMethod("classifyAnimatedObjects",Map.class);classify.setAccessible(true);Object classified=classify.invoke(null,definitions);
        Method eligible=method(SableModelCollisionHull.class,"eligibleMovingExterior",3);
        for(String name:List.of("body_frame","door_skin"))require((boolean)eligible.invoke(null,mesh(name),definitions,classified),"moving exterior retained "+name);
        for(String name:List.of("cycle","bad","scale","wheel","headlight"))require(!(boolean)eligible.invoke(null,mesh(name),definitions,classified),"unsupported/hardware excluded "+name);

        var wingR=object("$wingr",null,AnimationComponentType.ROTATION);
        var wingRAnimation=wingR.animations.get(0);
        wingRAnimation.variable="flaps_actual";
        wingRAnimation.centerPoint=new Point3D(-2.5048,.92311,.55271);
        wingRAnimation.axis=new Point3D(0,1,0);
        var wingL=object("$wingl",null,AnimationComponentType.ROTATION);
        var wingLAnimation=wingL.animations.get(0);
        wingLAnimation.variable="flaps_actual";
        wingLAnimation.centerPoint=new Point3D(2.5048,.92311,.55271);
        wingLAnimation.axis=new Point3D(0,-1,0);
        var damagedWing=object("$winglDT",null,AnimationComponentType.ROTATION);
        var damageGate=new JSONAnimationDefinition();
        damageGate.animationType=AnimationComponentType.VISIBILITY;
        damageGate.variable="damage";damageGate.clampMin=180;damageGate.clampMax=200;
        damagedWing.animations=List.of(damagedWing.animations.get(0),damageGate);
        Map<String,JSONAnimatedObject> prefixedDefinitions=new LinkedHashMap<>();
        for(var item:List.of(wingR,wingL,damagedWing,
            object("$headlight",null,AnimationComponentType.ROTATION),
            object("$lamp",null,AnimationComponentType.ROTATION),
            object("$lightbar",null,AnimationComponentType.ROTATION),
            object("$interior_stick",null,AnimationComponentType.ROTATION),
            object("#light",null,AnimationComponentType.ROTATION),
            object("&light",null,AnimationComponentType.ROTATION)))
            prefixedDefinitions.put(item.objectName.toLowerCase(Locale.ROOT),item);
        var prefixedClassification=classify.invoke(null,prefixedDefinitions);
        for(String name:List.of("$wingr","$wingl"))
            require((boolean)eligible.invoke(null,mesh(name,new Vec3d(0,0,0),new Vec3d(4,0,0),new Vec3d(4,0,-3)),
                prefixedDefinitions,prefixedClassification),"physical prefixed wing enters moving collision eligibility: "+name);
        for(String name:List.of("$winglDT","$headlight","$lamp","$lightbar","$interior_stick","#light","&light"))
            require(!(boolean)eligible.invoke(null,mesh(name,new Vec3d(0,0,0),new Vec3d(1,0,0),new Vec3d(1,0,1)),
                prefixedDefinitions,prefixedClassification),"damage/decorative semantics remain excluded: "+name);

        Method pressureReason=method(SableModelCollisionHull.class,"bodyPressureIgnoreReason",2);
        Method staticReason=method(SableModelCollisionHull.class,"staticCollisionIgnoreReason",3);
        require("NONE".equals(String.valueOf(pressureReason.invoke(null,mesh("$wingr"),prefixedClassification))),
            "moving prefixed wing contributes to body-pressure exterior geometry");
        require("RENDER_ONLY".equals(String.valueOf(staticReason.invoke(null,mesh("$wingr"),prefixedClassification,false))),
            "moving wing remains out of rigid static shell by animation classification");
        require("NONE".equals(String.valueOf(pressureReason.invoke(null,mesh("$fuselage"),prefixedClassification))),
            "ordinary prefixed fuselage remains eligible for body pressure");
        require("NONE".equals(String.valueOf(staticReason.invoke(null,mesh("$fuselage"),prefixedClassification,false))),
            "ordinary prefixed fuselage remains eligible for static shell");
        require("RENDER_ONLY".equals(String.valueOf(pressureReason.invoke(null,mesh("$winglDT"),prefixedClassification))),
            "damage replacement stays out of body-pressure geometry");
        require("RENDER_ONLY".equals(String.valueOf(staticReason.invoke(null,mesh("$winglDT"),prefixedClassification,false))),
            "damage replacement stays out of static collision shell");

        Method buildMoving=method(SableModelCollisionHull.class,"buildMovingObjectHull",4);
        var syntheticWing=mesh("$wingr",new Vec3d(0,0,0),new Vec3d(4,0,0),new Vec3d(4,0,-3),
            new Vec3d(0,0,0),new Vec3d(4,0,-3),new Vec3d(0,0,-3));
        var syntheticWingHull=(SableModelCollisionHull.PreparedMovingObjectHull)buildMoving.invoke(null,syntheticWing,1.,1.,1.);
        require(syntheticWingHull!=null&&!syntheticWingHull.boxes().isEmpty(),
            "physical prefixed wing rasterizes into a moving collision hull");
        require(syntheticWingHull.objectName().equals("$wingr")&&syntheticWingHull.sourceTriangles()==2,
            "prefixed wing collision hull preserves source identity and triangles");
        assertRigidPoseTracksWingHinge();

        Method signature=SableModelCollisionHull.class.getDeclaredMethod("animationSignature",Map.class);signature.setAccessible(true);
        String rigid=(String)signature.invoke(null,Map.of("panel",object("panel",null,AnimationComponentType.ROTATION)));
        String scaled=(String)signature.invoke(null,Map.of("panel",object("panel",null,AnimationComponentType.SCALING)));
        String missing=(String)signature.invoke(null,Map.of("panel",object("panel","absent",AnimationComponentType.ROTATION)));
        var changedSource=object("panel",null,AnimationComponentType.ROTATION);changedSource.animations.get(0).variable="other_actuator";
        String alternateSource=(String)signature.invoke(null,Map.of("panel",changedSource));
        require(!rigid.equals(scaled),"rotation to scaling invalidates moving geometry cache");
        require(!rigid.equals(missing),"missing applyAfter invalidates moving geometry cache");
        require(!rigid.equals(alternateSource),"same-name animation source change invalidates moving hull cache");
        Method movingKey=SableModelCollisionHull.class.getDeclaredMethod("movingExteriorCacheKey",
            String.class,double.class,double.class,double.class,String.class);movingKey.setAccessible(true);
        String originalModelKey=(String)movingKey.invoke(null,"fixture.obj",1.0,1.0,1.0,rigid);
        String replacementModelKey=(String)movingKey.invoke(null,"fixture-replacement.obj",1.0,1.0,1.0,rigid);
        String replacementAnimationKey=(String)movingKey.invoke(null,"fixture.obj",1.0,1.0,1.0,alternateSource);
        require(!originalModelKey.equals(replacementModelKey),"same-name model resource replacement changes moving hull key");
        require(!originalModelKey.equals(replacementAnimationKey),"same-name animation source replacement changes moving hull key");
        Method includeSource=Class.forName("com.g9third.pmweatheriv.sable.SableCompoundCollider")
            .getDeclaredMethod("includeModelSourceSignature",long.class,String.class);includeSource.setAccessible(true);
        long sourceTopology=(long)includeSource.invoke(null,17L,originalModelKey);
        long replacementTopology=(long)includeSource.invoke(null,17L,replacementModelKey);
        require(sourceTopology!=replacementTopology,"same-named collider child rebuilds for changed model source");
    }
    private static void assertRigidPoseTracksWingHinge()throws Exception {
        Method relative=method(Class.forName("com.g9third.pmweatheriv.sable.SableCompoundCollider"),
            "movingModelRelativePose",2);
        Vec3d hinge=new Vec3d(-2.5048,.92311,.55271),tip=new Vec3d(-6.4,.8,.8);
        var authored=rotationAboutY(hinge,0.0);
        var deflected=rotationAboutY(hinge,Math.toRadians(27));
        Object authoredPose=relative.invoke(null,authored,new Vector3d());
        Object deflectedPose=relative.invoke(null,deflected,new Vector3d());
        Vector3d authoredPoint=posedPoint(authoredPose,tip);
        Vector3d deflectedPoint=posedPoint(deflectedPose,tip);
        vectorNear(new Vector3d(hinge.x(),hinge.y(),hinge.z()),posedPoint(deflectedPose,hinge),1e-9,
            "live wing rotation keeps its authored hinge fixed");
        require(authoredPoint.distance(deflectedPoint)>.5,
            "moving collision pose follows a changed live wing rotation around its hinge");
    }
    private static RigidTransform rotationAboutY(Vec3d pivot,double angle) {
        double c=Math.cos(angle),s=Math.sin(angle);
        Vec3d translatedPivot=new Vec3d(c*pivot.x()+s*pivot.z(),pivot.y(),-s*pivot.x()+c*pivot.z());
        Vec3d translation=pivot.subtract(translatedPivot);
        return new RigidTransform(c,0,s,0,1,0,-s,0,c,translation);
    }
    private static Vector3d posedPoint(Object pose,Vec3d point)throws Exception {
        Vector3d position=new Vector3d((Vector3d)method(pose.getClass(),"position",0).invoke(pose));
        Quaterniond orientation=new Quaterniond((Quaterniond)method(pose.getClass(),"orientation",0).invoke(pose));
        return orientation.transform(new Vector3d(point.x(),point.y(),point.z())).add(position);
    }
    private static void raster()throws Exception {
        Vec3d a=new Vec3d(-1,0,-2),b=new Vec3d(1,0,-2),c=new Vec3d(1,0,2),d=new Vec3d(-1,0,2);
        Mesh planar=mesh("door_skin",a,b,c,a,c,d);
        Method build=method(SableModelCollisionHull.class,"buildMovingObjectHull",4);
        var hull=(SableModelCollisionHull.PreparedMovingObjectHull)build.invoke(null,planar,2.0,.5,1.3);
        require(hull!=null&&!hull.boxes().isEmpty(),"zero-thickness panel voxelizes");
        require(hull.sourceTriangles()==2&&hull.objectName().equals("door_skin"),"object provenance retained");
        for(int i=0;i<=40;++i)for(int j=0;j<=40;++j){double x=-2+i*.1,z=-2.6+j*.13;boolean covered=false;
            for(var box:hull.boxes())if(Math.abs(x-box.cx())<=box.hx()+1e-6&&Math.abs(box.cy())<=box.hy()+1e-6&&Math.abs(z-box.cz())<=box.hz()+1e-6){covered=true;break;}
            require(covered,"scaled planar surface covered");}
        require(build.invoke(null,mesh("body"),1.,1.,1.)==null,"empty object has no hull");
        Class<?> budgetClass=Class.forName("com.g9third.pmweatheriv.sable.SableModelCollisionHull$MovingRasterBudget");
        Constructor<?> budgetConstructor=budgetClass.getDeclaredConstructor();budgetConstructor.setAccessible(true);
        Object budget=budgetConstructor.newInstance();Field remaining=budgetClass.getDeclaredField("remaining");remaining.setAccessible(true);
        remaining.setLong(budget,4);
        Method bounded=SableModelCollisionHull.class.getDeclaredMethod("buildMovingObjectHull",Mesh.class,double.class,double.class,double.class,budgetClass);bounded.setAccessible(true);
        require(bounded.invoke(null,planar,1.,1.,1.,budget)==null,"insufficient aggregate raster budget omits whole object");
        require(remaining.getLong(budget)>=0,"raster work never overruns aggregate budget");
    }
    private static void poses()throws Exception {
        Class<?> compound=Class.forName("com.g9third.pmweatheriv.sable.SableCompoundCollider");
        Method relative=method(compound,"movingModelRelativePose",2);Random random=new Random(34);
        for(int i=0;i<1500;++i){
            RotationMatrix matrix=new RotationMatrix().setToAngles(new Point3D(random.nextDouble()*360-180,random.nextDouble()*360-180,random.nextDouble()*360-180));
            Vec3d t=new Vec3d(random.nextDouble()*4-2,random.nextDouble()*2-1,random.nextDouble()*4-2);
            RigidTransform transform=new RigidTransform(matrix.m00,matrix.m01,matrix.m02,matrix.m10,matrix.m11,matrix.m12,matrix.m20,matrix.m21,matrix.m22,t);
            Vector3d com=new Vector3d(.3,-.2,1.1);Object pose=relative.invoke(null,transform,com);require(pose!=null,"rigid pose accepted");
            Vector3d position=(Vector3d)method(pose.getClass(),"position",0).invoke(pose);
            Quaterniond orientation=(Quaterniond)method(pose.getClass(),"orientation",0).invoke(pose);
            for(int k=0;k<8;++k){Vec3d p=new Vec3d((k&1)==0?-1:1,(k&2)==0?-.5:.5,(k&4)==0?-2:2);Vec3d expected=transform.point(p);
                Vector3d actual=new Quaterniond(orientation).transform(new Vector3d(p.x(),p.y(),p.z())).add(position).add(com);
                near(actual.x,expected.x(),1e-9,"moving voxel x");near(actual.y,expected.y(),1e-9,"moving voxel y");near(actual.z,expected.z(),1e-9,"moving voxel z");}
        }
        require(relative.invoke(null,new RigidTransform(1,.3,0,0,1,0,0,0,1,Vec3d.ZERO),new Vector3d())==null,"shear never masquerades as rigid rotation");
        require(relative.invoke(null,null,new Vector3d())==null,"missing pose has no collider");
    }
    private static void mountedPartOrigins()throws Exception {
        Class<?> compound=Class.forName("com.g9third.pmweatheriv.sable.SableCompoundCollider");
        Method resolve=method(compound,"mountedPartRelativePosition",6);
        Vector3d centerOfMass=new Vector3d(.35,-.22,1.15);
        Quaterniond vehicleOrientation=new Quaterniond().rotationXYZ(.24,-.63,.18);
        Vector3d vehicleWorldPosition=new Vector3d(3.0e9,-2.0e9,1.5e9);

        // IV leaves a direct-root localOffset in the master frame. Keep that
        // stable path rather than introducing large-coordinate cancellation.
        Vector3d rootLocalOffset=new Vector3d(1.2,-.7,2.4);
        Vector3d rootWorldPosition=new Vector3d(vehicleWorldPosition).add(
            new Quaterniond(vehicleOrientation).transform(new Vector3d(rootLocalOffset)));
        Vector3d rootResult=(Vector3d)resolve.invoke(null,false,rootLocalOffset,rootWorldPosition,
            vehicleWorldPosition,vehicleOrientation,centerOfMass);
        vectorNear(rootResult,new Vector3d(rootLocalOffset).sub(centerOfMass),0,
            "direct-root collider uses stable master-frame localOffset");

        // A nested child is attached under a rotated part. The resolved origin
        // combines the parent's translated/rotated frame with the child offset;
        // the post-update rebased localOffset is intentionally in a different frame.
        Quaterniond parentOrientation=new Quaterniond().rotationY(Math.PI*.5);
        Vector3d parentOriginInMaster=new Vector3d(3.0,-.8,1.6);
        Vector3d childOffsetInParent=new Vector3d(.35,.6,2.1);
        Vector3d resolvedMasterOrigin=new Vector3d(parentOriginInMaster).add(
            new Quaterniond(parentOrientation).transform(new Vector3d(childOffsetInParent)));
        Vector3d rewrittenLocalOffset=new Quaterniond(parentOrientation).conjugate()
            .transform(new Vector3d(resolvedMasterOrigin)).add(parentOriginInMaster);
        require(rewrittenLocalOffset.distance(resolvedMasterOrigin)>.5,
            "rotated nested parent makes native rebased localOffset differ from master origin");
        Vector3d partWorldPosition=new Vector3d(vehicleWorldPosition).add(
            new Quaterniond(vehicleOrientation).transform(new Vector3d(resolvedMasterOrigin)));
        Vector3d nestedResult=(Vector3d)resolve.invoke(null,true,rewrittenLocalOffset,partWorldPosition,
            vehicleWorldPosition,vehicleOrientation,centerOfMass);
        Vector3d expectedRelativeOrigin=new Vector3d(resolvedMasterOrigin).sub(centerOfMass);
        vectorNear(nestedResult,expectedRelativeOrigin,2e-6,
            "nested origin resolves world-to-master and subtracts COM once");

        // Reconstruct a point on the mounted collider in world space. This
        // catches both a misplaced origin and applying the COM correction twice.
        Quaterniond partWorldOrientation=new Quaterniond(vehicleOrientation).mul(parentOrientation).normalize();
        Quaterniond relativeOrientation=new Quaterniond(vehicleOrientation).conjugate()
            .mul(partWorldOrientation).normalize();
        Vector3d bodyWorldPosition=new Vector3d(vehicleWorldPosition).add(
            new Quaterniond(vehicleOrientation).transform(new Vector3d(centerOfMass)));
        Vector3d childPoint=new Vector3d(.7,-.25,1.3);
        Vector3d actualWorldPoint=new Vector3d(bodyWorldPosition).add(
            new Quaterniond(vehicleOrientation).transform(
                new Vector3d(nestedResult).add(relativeOrientation.transform(new Vector3d(childPoint)))));
        Vector3d expectedWorldPoint=new Vector3d(partWorldPosition).add(
            new Quaterniond(partWorldOrientation).transform(new Vector3d(childPoint)));
        vectorNear(actualWorldPoint,expectedWorldPoint,3e-6,
            "nested mounted collider point follows rotated parent at large world coordinates");

        Vector3d invalidWorldPosition=new Vector3d(Double.NaN,Double.POSITIVE_INFINITY,0);
        Vector3d fallback=(Vector3d)resolve.invoke(null,true,new Vector3d(2,3,4),invalidWorldPosition,
            vehicleWorldPosition,vehicleOrientation,centerOfMass);
        vectorNear(fallback,new Vector3d(2,3,4).sub(centerOfMass),0,
            "invalid resolved nested pose uses finite localOffset fallback");
        try {
            resolve.invoke(null,true,new Vector3d(Double.NaN,0,0),invalidWorldPosition,
                vehicleWorldPosition,vehicleOrientation,centerOfMass);
            throw new AssertionError("fully invalid mounted origin must be rejected");
        } catch (InvocationTargetException failure) {
            Throwable cause=failure.getCause();
            require(cause instanceof IllegalArgumentException,
                "fully invalid mounted origin rejects without guessing a pose");
            require(cause.getMessage().contains("origin"),
                "invalid mounted origin reports the unusable source");
        }
        try {
            resolve.invoke(null,true,new Vector3d(2,3,4),invalidWorldPosition,
                vehicleWorldPosition,vehicleOrientation,
                new Vector3d(Double.NaN,0,0));
            throw new AssertionError("invalid COM must be rejected");
        } catch (InvocationTargetException failure) {
            Throwable cause=failure.getCause();
            require(cause instanceof IllegalArgumentException,
                "invalid COM is not silently treated as zero");
            require(cause.getMessage().contains("centerOfMassLocal"),
                "invalid COM reports the source field");
        }
    }
    private static Object relativePose(Vector3d position,Quaterniond orientation)throws Exception {
        Class<?> type=Class.forName("com.g9third.pmweatheriv.sable.SableCompoundCollider$RelativePose");
        Constructor<?> constructor=type.getDeclaredConstructor(Vector3d.class,Quaterniond.class);constructor.setAccessible(true);
        return constructor.newInstance(position,orientation);
    }
    private static void vectorNear(Vector3d actual,Vector3d expected,double tolerance,String label) {
        near(actual.x,expected.x,tolerance,label+" x");near(actual.y,expected.y,tolerance,label+" y");near(actual.z,expected.z,tolerance,label+" z");
    }
    private static Vector3d rate(Object value,String name)throws Exception {
        // JOML transforms mutate Vector3d arguments. Return a copy so checks
        // cannot alter the production rate record between assertions.
        return new Vector3d((Vector3d)method(value.getClass(),name,0).invoke(value));
    }
    private static void surfaceRates()throws Exception {
        Class<?> compound=Class.forName("com.g9third.pmweatheriv.sable.SableCompoundCollider");
        Method velocity=method(compound,"relativeSurfaceVelocity",3);
        Random random=new Random(3401);
        for(int i=0;i<1000;++i) {
            Quaterniond previous=new Quaterniond().rotationXYZ(random.nextDouble()*6,random.nextDouble()*6,random.nextDouble()*6);
            Vector3d axis=new Vector3d(random.nextDouble()-.5,random.nextDouble()-.5,random.nextDouble()-.5).normalize();
            double angle=(random.nextDouble()-.5)*.6;
            Quaterniond current=new Quaterniond().rotationAxis(angle,axis.x,axis.y,axis.z).mul(previous);
            Vector3d start=new Vector3d(1,-2,3),delta=new Vector3d(.01,-.02,.03);
            Object from=relativePose(start,previous),to=relativePose(new Vector3d(start).add(delta),current);
            Object result=velocity.invoke(null,from,to,1L);
            vectorNear(new Quaterniond(current).transform(rate(result,"linear")),new Vector3d(delta).mul(20),1e-11,"child rate reconstructs parent translation");
            vectorNear(new Quaterniond(current).transform(rate(result,"angular")),new Vector3d(axis).mul(angle*20),1e-11,"child rate reconstructs parent rotation axis");
            Vector3d point=new Vector3d(.3,1.1,-.7);
            Vector3d nativeRate=rate(result,"angular").cross(point,new Vector3d()).add(rate(result,"linear"));
            current.transform(nativeRate);
            Vector3d expected=new Vector3d(axis).mul(angle*20).cross(new Quaterniond(current).transform(point),new Vector3d()).add(new Vector3d(delta).mul(20));
            vectorNear(nativeRate,expected,1e-10,"native hook point velocity");
            Object negative=velocity.invoke(null,from,relativePose(new Vector3d(start).add(delta),new Quaterniond(current).mul(-1)),1L);
            vectorNear(rate(negative,"angular"),rate(result,"angular"),1e-11,"quaternion sign does not alter rate");
        }
        Object fixed=relativePose(new Vector3d(),new Quaterniond().rotationXYZ(.2,.6,-.3));
        Object moved=relativePose(new Vector3d(.1,0,0),new Quaterniond().rotationXYZ(.2,.6,-.3));
        for(long ticks:new long[]{0,2,100}) {
            Object unavailable=velocity.invoke(null,fixed,moved,ticks);
            vectorNear(rate(unavailable,"linear"),new Vector3d(),0,"nonconsecutive history has zero linear rate");
            vectorNear(rate(unavailable,"angular"),new Vector3d(),0,"nonconsecutive history has zero angular rate");
        }
        vectorNear(rate(velocity.invoke(null,null,moved,1L),"linear"),new Vector3d(),0,"first pose has zero rate");
        Class<?> historyClass=Class.forName("com.g9third.pmweatheriv.sable.SableCompoundCollider$RelativeMotionHistory");
        Constructor<?> constructor=historyClass.getDeclaredConstructor();constructor.setAccessible(true);Object history=constructor.newInstance();
        Method observe=method(historyClass,"observe",2);
        for(long tick=0;tick<1000;++tick) vectorNear(rate(observe.invoke(history,fixed,tick),"linear"),new Vector3d(),0,"idle history");
        Object firstMotion=observe.invoke(history,moved,1000L);
        vectorNear(new Quaterniond().rotationXYZ(.2,.6,-.3).transform(rate(firstMotion,"linear")),new Vector3d(2,0,0),1e-11,"first movement after idle uses one tick");
        vectorNear(rate(observe.invoke(history,moved,1001L),"linear"),new Vector3d(),0,"stopped child has no residual rate");
        vectorNear(rate(observe.invoke(history,fixed,1004L),"linear"),new Vector3d(),0,"skipped observation does not invent a rate");
    }
    private static void external(String definitionPath,String modelPath)throws Exception {
        JSONVehicle definition;
        try(var input=Files.newInputStream(Path.of(definitionPath))) {
            definition=JSONParser.parseStream(input,JSONVehicle.class,"moving-hull-fixture","fixture");
        }
        Map<String,JSONAnimatedObject> definitions=new LinkedHashMap<>();
        if(definition.rendering!=null&&definition.rendering.animatedObjects!=null)
            for(var object:definition.rendering.animatedObjects)
                if(object!=null&&object.objectName!=null)definitions.put(object.objectName.toLowerCase(Locale.ROOT).trim(),object);
        Method classify=SableModelCollisionHull.class.getDeclaredMethod("classifyAnimatedObjects",Map.class);classify.setAccessible(true);
        Object classification=classify.invoke(null,definitions);
        Method eligible=method(SableModelCollisionHull.class,"eligibleMovingExterior",3);
        Method build=method(SableModelCollisionHull.class,"buildMovingObjectHull",4);
        Map<String,List<Vec3d>> objects=new LinkedHashMap<>();List<Vec3d> vertices=new ArrayList<>();String name="";
        for(String raw:Files.readAllLines(Path.of(modelPath))) {
            String line=raw.trim();String[] fields=line.split("\\s+");
            if(line.startsWith("v "))vertices.add(new Vec3d(Double.parseDouble(fields[1]),Double.parseDouble(fields[2]),Double.parseDouble(fields[3])));
            else if(line.startsWith("o ")) { name=line.substring(2).trim();objects.computeIfAbsent(name,n->new ArrayList<>()); }
            else if(line.startsWith("f ")) {
                int[] indices=new int[fields.length-1];
                for(int i=1;i<fields.length;i++){int index=Integer.parseInt(fields[i].split("/")[0]);indices[i-1]=index>0?index-1:vertices.size()+index;}
                List<Vec3d> points=objects.computeIfAbsent(name,n->new ArrayList<>());
                for(int i=1;i<indices.length-1;i++){points.add(vertices.get(indices[0]));points.add(vertices.get(indices[i]));points.add(vertices.get(indices[i+1]));}
            }
        }
        int retained=0,totalBoxes=0,totalSamples=0;Set<String> retainedNames=new HashSet<>();
        for(var entry:objects.entrySet()) {
            Mesh mesh=mesh(entry.getKey(),entry.getValue().toArray(Vec3d[]::new));
            if(!(boolean)eligible.invoke(null,mesh,definitions,classification))continue;
            var hull=(SableModelCollisionHull.PreparedMovingObjectHull)build.invoke(null,mesh,1.,1.,1.);
            require(hull!=null&&!hull.boxes().isEmpty(),"eligible real model has bounded hull "+entry.getKey());
            ++retained;retainedNames.add(entry.getKey().toLowerCase(Locale.ROOT));
            totalBoxes+=hull.boxes().size();int samples=0;
            List<Vec3d> points=entry.getValue();
            for(int i=0;i+2<points.size();i+=3) {
                Vec3d a=points.get(i),b=points.get(i+1),c=points.get(i+2);
                if(b.subtract(a).cross(c.subtract(a)).lengthSquared()<1e-12)continue;
                for(Vec3d p:List.of(a,b,c,a.add(b).scale(.5),b.add(c).scale(.5),c.add(a).scale(.5),a.add(b).add(c).scale(1./3))) {
                    boolean covered=false;
                    for(var box:hull.boxes())if(Math.abs(p.x()-box.cx())<=box.hx()+2e-6&&Math.abs(p.y()-box.cy())<=box.hy()+2e-6&&Math.abs(p.z()-box.cz())<=box.hz()+2e-6){covered=true;break;}
                    require(covered,"real source triangle sample covered "+entry.getKey());++samples;
                }
            }
            totalSamples+=samples;
            System.out.println("Moving exterior object "+entry.getKey()+": sourceTriangles="+hull.sourceTriangles()+", boxes="+hull.boxes().size()+", resolution="+hull.resolution()+", coveredSamples="+samples);
        }
        if(objects.containsKey("$wingl")&&objects.containsKey("$wingr")) {
            require(retainedNames.contains("$wingl")&&retainedNames.contains("$wingr"),
                "real model's intact prefixed left and right wings enter moving exterior hulls");
            for(String damagedName:List.of("$wingldt","$wingrd"))
                if(objects.keySet().stream().anyMatch(meshName->meshName.equalsIgnoreCase(damagedName)))
                    require(!retainedNames.contains(damagedName),
                        "real model damage replacement stays excluded: "+damagedName);
        }
        require(retained>0,"real fixture exercises moving exterior");
        System.out.println("External moving model: objects="+retained+", boxes="+totalBoxes+", samples="+totalSamples+" (reference scale1; no live native matrices)");
    }
    public static void main(String[] args)throws Exception {eligibility();raster();poses();mountedPartOrigins();surfaceRates();if(args.length==2)external(args[0],args[1]);System.out.println("MovingModelHullRegression: "+assertions+" assertions passed (voxel/matrix/rate helpers; no game simulation)");}
}
