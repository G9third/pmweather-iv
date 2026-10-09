import com.g9third.pmweatheriv.physics.*;
import com.g9third.pmweatheriv.sable.*;
import com.g9third.pmweatheriv.model.ParsedModelSnapshot;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.rendering.RenderableVertices;
import org.joml.Vector3d;
import java.lang.reflect.*;
import java.nio.*;
import java.util.*;

/** Numeric coordinate and contact invariants using the actual compiled physics classes. */
public class CoordinateContactRegression {
    static int checks;
    static void require(boolean value,String message) { ++checks; if (!value) throw new AssertionError(message); }
    static void near(double a,double b,double tolerance,String message) { require(Double.isFinite(a)&&Math.abs(a-b)<=tolerance,message+" actual="+a+" expected="+b); }
    static void vector(Vec3d a,Vec3d b,double tolerance,String message) { near(a.subtract(b).length(),0,tolerance,message); }
    static Vec3d world(RotationMatrix q,Vec3d p) { Point3D v=new Point3D(p.x(),p.y(),p.z()).rotate(q);return new Vec3d(v.x,v.y,v.z); }
    static Vec3d local(RotationMatrix q,Vec3d p) { Point3D v=new Point3D(p.x(),p.y(),p.z()).reOrigin(q);return new Vec3d(v.x,v.y,v.z); }
    public static void main(String[] args)throws Exception {
        Random random=new Random(1129);
        for (int i=0;i<6000;++i) {
            RotationMatrix orientation=new RotationMatrix().setToAngles(new Point3D(random.nextDouble()*360-180,random.nextDouble()*360-180,random.nextDouble()*360-180));
            Vec3d origin=new Vec3d(random.nextDouble()*2000-1000,random.nextDouble()*300-60,random.nextDouble()*2000-1000);
            Vec3d station=new Vec3d(random.nextDouble()*40-20,random.nextDouble()*8-4,random.nextDouble()*30-15);
            Vec3d center=new Vec3d(0,.542,1.413);
            Vec3d result=ModelCoordinates.worldPoint(origin,orientation,station);
            vector(result,origin.add(world(orientation,station)),1e-10,"IV model-origin world station");
            vector(local(orientation,result.subtract(origin)),station,1e-10,"world/local station roundtrip");
            vector(result,origin.add(world(orientation,center)).add(world(orientation,station.subtract(center))),1e-10,"COM/model-origin closure");
            Vector3d joml=new Vector3d(station.x(),station.y(),station.z());
            SablePoseConversions.toQuaternion(orientation).transform(joml);
            vector(new Vec3d(joml.x,joml.y,joml.z),world(orientation,station),1e-9,"Sable/MTS orientation parity");
        }
        System.out.println("World/local/COM/quaternion coordinate checks: PASS (6000 poses)");
        FloatBuffer raw=FloatBuffer.allocate(24*2);
        float[][] vertices={{10,2,-7},{12,2,-7},{10,4,-7},{10,2,-7},{10,4,-7},{10,2,-5}};
        for (float[] vertex:vertices) { raw.put(0).put(0).put(1).put(0).put(0); for(float value:vertex) raw.put(value); }
        raw.flip();raw.position(3);
        var constructor=ParsedModelSnapshot.Mesh.class.getDeclaredConstructor(RenderableVertices.class);constructor.setAccessible(true);
        var mesh=constructor.newInstance(new RenderableVertices("fuselage",raw,false));
        var empty=constructor.newInstance(new RenderableVertices("empty",null,false));
        require(empty.scaled(2,2,2).positions().remaining()==0,"empty model is safely retriable");
        final int[] parseAttempts={0};
        new minecrafttransportsimulator.rendering.AModelParser() {
            protected String getModelSuffix() { return "pmivcheck"; }
            protected List<RenderableVertices> parseModelInternal(String location) {
                return ++parseAttempts[0]==1 ? List.of() : List.of(new RenderableVertices("body",raw,false));
            }
        };
        require(ParsedModelSnapshot.load("retry-fixture.pmivcheck").objects().isEmpty(),"empty parser result observed");
        require(!ParsedModelSnapshot.load("retry-fixture.pmivcheck").objects().isEmpty(),"empty parser result is retried");
        require(!ParsedModelSnapshot.load("retry-fixture.pmivcheck").objects().isEmpty() && parseAttempts[0]==2,"successful parser result is cached");
        var scaled=mesh.scaled(2,.5,3);FloatBuffer positions=scaled.positions();
        for(float[] vertex:vertices) {near(positions.get(),vertex[0]*2,1e-6,"scaled x");near(positions.get(),vertex[1]*.5,1e-6,"scaled y");near(positions.get(),vertex[2]*3,1e-6,"scaled z");}
        near(raw.position(),3,0,"renderer cursor untouched");near(mesh.positions().get(),10,0,"renderer geometry untouched");
        require(scaled.positions().isReadOnly(),"immutable scaled geometry");
        for (double roll:new double[]{0,5,30,60,90,120,175,180}) {
            RotationMatrix q=new RotationMatrix().setToAngles(new Point3D(0,37,roll));
            Vec3d axle=new Vec3d(1,0,0), support=ModelCoordinates.tireSupportOffset(q,axle,.45,.13);
            Vec3d down=local(q,new Vec3d(0,-1,0));double axial=down.dot(axle);
            double expected=-Math.sqrt(.45*.45*(1-axial*axial)+.13*.13*axial*axial);
            near(world(q,support).y(),expected,1e-10,"tilted tire lowest point");
            near(support.x()*support.x()/(.13*.13)+(support.y()*support.y()+support.z()*support.z())/(.45*.45),1,1e-10,"station lies on rounded tire");
        }
        System.out.println("Mesh scale and tilted tire support checks: PASS");
        // Matches renderer S*R applied to authored vertices about the authored hinge.
        for (int i=0;i<1000;++i) {
            Vec3d scale=new Vec3d(.5+random.nextDouble()*3,.5+random.nextDouble()*3,.5+random.nextDouble()*3);
            Vec3d authoredPivot=new Vec3d(2,.3,-1), authoredPoint=new Vec3d(7,.2,-3);
            Vec3d pivot=new Vec3d(authoredPivot.x()*scale.x(),authoredPivot.y()*scale.y(),authoredPivot.z()*scale.z());
            Vec3d point=new Vec3d(authoredPoint.x()*scale.x(),authoredPoint.y()*scale.y(),authoredPoint.z()*scale.z());
            double angle=random.nextDouble()*1.5;
            var transform=AnimatedWingGeometry.RigidTransform.scaledRotationAround(pivot,new Vec3d(0,1,0),scale,angle);
            RotationMatrix rotation=new RotationMatrix().setToAngles(new Point3D(0,Math.toDegrees(angle),0));
            Vec3d authored=authoredPivot.add(world(rotation,authoredPoint.subtract(authoredPivot)));
            vector(transform.point(point),new Vec3d(authored.x()*scale.x(),authored.y()*scale.y(),authored.z()*scale.z()),1e-9,"scaled animated pivot");
            vector(transform.point(pivot),pivot,1e-9,"hinge remains fixed");
            near(transform.normal(new Vec3d(0,1,0)).dot(transform.direction(new Vec3d(1,0,0))),0,1e-9,"animated normal perpendicular");
        }
        System.out.println("Scaled animated wing coordinate checks: PASS (1000 poses)");
        checkPressureStations();
        for (String arg:args) checkActualModel(java.nio.file.Path.of(arg));
        checkContactDynamics();
        System.out.println("PASS: "+checks+" numeric assertions; no in-game simulation performed");
    }
    static void checkPressureStations() {
        List<SableModelCollisionHull.SurfaceTriangle> triangles=new ArrayList<>();
        Vec3d[] v={new Vec3d(9,2,-9),new Vec3d(11,2,-9),new Vec3d(9,4,-9),new Vec3d(11,4,-9),new Vec3d(9,2,-5),new Vec3d(11,2,-5),new Vec3d(9,4,-5),new Vec3d(11,4,-5)};
        int[][] quads={{0,1,3,2},{4,6,7,5},{0,4,5,1},{2,3,7,6},{0,2,6,4},{1,5,7,3}};
        for(int[] q:quads) {triangles.add(new SableModelCollisionHull.SurfaceTriangle(v[q[0]],v[q[1]],v[q[2]]));triangles.add(new SableModelCollisionHull.SurfaceTriangle(v[q[0]],v[q[2]],v[q[3]]));}
        var hull=new SableModelCollisionHull.PreparedHull("synthetic-offset-body",true,"fixture",.2,1,12,2000,0,1,0,0,0,0,0,0,0,List.of(new SableModelCollisionHull.HullBox(10,3,-7,1,1,2)),List.of(),triangles);
        var body=SableBodyPressureGeometry.prepare(hull,64);
        require(body.patches().size()==64,"road spatial station budget");near(body.wettedArea(),40,1e-9,"pressure area invariant");vector(body.centerOfMassLocal(),new Vec3d(10,3,-7),1e-9,"offset model COM");
        double area=0;
        for(var patch:body.patches()) {
            area+=patch.area();double distance=Double.POSITIVE_INFINITY;
            for(var t:triangles)distance=Math.min(distance,ModelCoordinates.closestTrianglePoint(patch.windSamplePointLocal(),t.a(),t.b(),t.c()).subtract(patch.windSamplePointLocal()).length());
            near(distance,0,1e-10,"wind probe lies on mesh");
            require(patch.area()>0,"nonzero pressure region");
        }
        near(area,40,1e-9,"aggregation conserves pressure area");
        // Face, edge, vertex and sloped triangle projection cases.
        Vec3d a=new Vec3d(2,1,3),b=new Vec3d(4,3,3),c=new Vec3d(2,1,5);
        Vec3d centroid=a.add(b).add(c).scale(1.0/3);Vec3d n=b.subtract(a).cross(c.subtract(a)).normalized();
        vector(ModelCoordinates.closestTrianglePoint(centroid.add(n.scale(2)),a,b,c),centroid,1e-10,"sloped mesh face projection");
        vector(ModelCoordinates.closestTrianglePoint(a.subtract(new Vec3d(2,2,2)),a,b,c),a,1e-10,"mesh vertex projection");
        vector(ModelCoordinates.closestTrianglePoint(new Vec3d(1,1,0),Vec3d.ZERO,Vec3d.ZERO,new Vec3d(2,0,0)),new Vec3d(1,0,0),1e-12,"degenerate mesh edge projection");
        System.out.println("Mesh-anchored pressure station and area checks: PASS (64 regions)");
    }
    static List<GroundContactImpulseSolver.Contact> contacts(double mass,double dt,double gap,boolean soft) {
        List<GroundContactImpulseSolver.Contact> contacts=new ArrayList<>();
        for(double x:new double[]{-1,1})for(double z:new double[]{-2,2}) {
            var response=soft?TireNormalCompliance.response(mass/4,.03,gap,0,.01,dt):new TireNormalCompliance.Response(0,0,0,0,0);
            contacts.add(new GroundContactImpulseSolver.Contact(new Vec3d(x,-1,z),new Vec3d(0,0,1),.25,.25,response.targetMps(),false,.01,true,response.softnessInverseKg(),Double.POSITIVE_INFINITY,Double.NaN));
        }
        return contacts;
    }
    static Method reflected(Class<?> type,String name,int count) {
        for(Method method:type.getDeclaredMethods()) if(method.getName().equals(name)&&method.getParameterCount()==count) {method.setAccessible(true);return method;}
        throw new AssertionError(name);
    }
    static Vec3d vec(Object value)throws Exception {
        return new Vec3d(((Number)reflected(value.getClass(),"x",0).invoke(value)).doubleValue(),
            ((Number)reflected(value.getClass(),"y",0).invoke(value)).doubleValue(),
            ((Number)reflected(value.getClass(),"z",0).invoke(value)).doubleValue());
    }
    static void checkActualModel(java.nio.file.Path path)throws Exception {
        List<Vec3d> vertices=new ArrayList<>();
        List<SableModelCollisionHull.SurfaceTriangle> mesh=new ArrayList<>();
        String name="body";int objects=0;
        Method filter=reflected(SableModelCollisionHull.class,"aerodynamicIgnoreReason",2);
        for(String line:java.nio.file.Files.readAllLines(path)) {
            String[] tokens=line.trim().split("\\s+");
            if(tokens[0].equals("o")||tokens[0].equals("g")) {name=tokens.length>1?tokens[1]:"body";++objects;}
            else if(tokens[0].equals("v")) vertices.add(new Vec3d(Double.parseDouble(tokens[1]),Double.parseDouble(tokens[2]),Double.parseDouble(tokens[3])));
            else if(tokens[0].equals("f")&&!name.startsWith("$")&&filter.invoke(null,name.toLowerCase(Locale.ROOT),false).toString().equals("NONE")) {
                int[] indices=new int[tokens.length-1];
                for(int i=1;i<tokens.length;++i) {int index=Integer.parseInt(tokens[i].split("/")[0]);indices[i-1]=index<0?vertices.size()+index:index-1;}
                for(int i=1;i<indices.length-1;++i)mesh.add(new SableModelCollisionHull.SurfaceTriangle(vertices.get(indices[0]),vertices.get(indices[i]),vertices.get(indices[i+1])));
            }
        }
        require(!mesh.isEmpty(),"actual retained OBJ triangles");
        Class<?> vectorType=Class.forName(SableModelCollisionHull.class.getName()+"$Vec3"),triangleType=Class.forName(SableModelCollisionHull.class.getName()+"$Triangle");
        var vectorConstructor=vectorType.getDeclaredConstructors()[0];vectorConstructor.setAccessible(true);
        var triangleConstructor=triangleType.getDeclaredConstructors()[0];triangleConstructor.setAccessible(true);
        List<Object> rasterTriangles=new ArrayList<>();Vec3d min=new Vec3d(Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY),max=new Vec3d(Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY);
        for(var t:mesh) {
            rasterTriangles.add(triangleConstructor.newInstance(vectorConstructor.newInstance(t.a().x(),t.a().y(),t.a().z()),vectorConstructor.newInstance(t.b().x(),t.b().y(),t.b().z()),vectorConstructor.newInstance(t.c().x(),t.c().y(),t.c().z())));
            for(Vec3d vertex:List.of(t.a(),t.b(),t.c())) {min=new Vec3d(Math.min(min.x(),vertex.x()),Math.min(min.y(),vertex.y()),Math.min(min.z(),vertex.z()));max=new Vec3d(Math.max(max.x(),vertex.x()),Math.max(max.y(),vertex.y()),Math.max(max.z(),vertex.z()));}
        }
        double resolution=.2;
        Object raster=reflected(SableModelCollisionHull.class,"rasterize",2).invoke(null,rasterTriangles,resolution);
        require(!(boolean)reflected(raster.getClass(),"overBudget",0).invoke(raster),"actual model raster budget");
        Set<?> cells=(Set<?>)reflected(raster.getClass(),"cells",0).invoke(raster);
        @SuppressWarnings("unchecked") List<SableModelCollisionHull.HullBox> boxes=(List<SableModelCollisionHull.HullBox>)reflected(SableModelCollisionHull.class,"mergeCells",2).invoke(null,cells,resolution);
        var hull=new SableModelCollisionHull.PreparedHull("actual-OBJ-check",true,"fixture",resolution,objects,mesh.size(),cells.size(),0,boxes.size(),0,0,0,0,0,0,0,boxes,List.of(),mesh);
        var body=SableBodyPressureGeometry.prepare(hull,64);
        for(var patch:body.patches()) {
            double nearest=Double.POSITIVE_INFINITY;
            for(var t:mesh)nearest=Math.min(nearest,ModelCoordinates.closestTrianglePoint(patch.windSamplePointLocal(),t.a(),t.b(),t.c()).subtract(patch.windSamplePointLocal()).length());
            near(nearest,0,1e-9,"actual model wind station on triangle");
            Vec3d probe=patch.windSamplePointLocal();
            require(probe.x()>=min.x()-1e-9&&probe.x()<=max.x()+1e-9&&probe.y()>=min.y()-1e-9&&probe.y()<=max.y()+1e-9&&probe.z()>=min.z()-1e-9&&probe.z()<=max.z()+1e-9,"actual station within authored model bounds");
        }
        Vec3d span=max.subtract(min);
        require(body.width()>=span.x()-1e-6&&body.width()-span.x()<=2*resolution+1e-6,"voxel width bounded by resolution");
        require(body.height()>=span.y()-1e-6&&body.height()-span.y()<=2*resolution+1e-6,"voxel height bounded by resolution");
        require(body.length()>=span.z()-1e-6&&body.length()-span.z()<=2*resolution+1e-6,"voxel length bounded by resolution");
        System.out.printf(Locale.ROOT,"Actual OBJ checks: PASS; triangles=%d stations=%d mesh spans=(%.6f,%.6f,%.6f) voxel spans=(%.6f,%.6f,%.6f), CG=(%.6f,%.6f,%.6f)%n",mesh.size(),body.patches().size(),span.x(),span.y(),span.z(),body.width(),body.height(),body.length(),body.centerOfMassLocal().x(),body.centerOfMassLocal().y(),body.centerOfMassLocal().z());
    }
    static void checkContactDynamics() {
        double mass=2100,dt=.05,g=9.80665;Vec3d inertia=new Vec3d(7000,7000,2300);RotationMatrix q=new RotationMatrix();
        // Static spring equilibrium is skin minus nominal deflection, at mg normal load.
        var equilibrium=GroundContactImpulseSolver.solve(new Vec3d(0,-g*dt,0),Vec3d.ZERO,inertia,q,Vec3d.ZERO,mass,1,0,dt,contacts(mass,dt,-.02,true));
        near(equilibrium.velocityWorld().y(),0,2e-6,"spring equilibrium velocity");near(equilibrium.normalImpulseNs()/dt,mass*g,.1,"spring equilibrium weight");
        var soft=GroundContactImpulseSolver.solve(new Vec3d(5000*dt/mass,-g*dt,0),Vec3d.ZERO,inertia,q,Vec3d.ZERO,mass,1,0,dt,contacts(mass,dt,-.02,true));
        var rigid=GroundContactImpulseSolver.solve(new Vec3d(5000*dt/mass,-g*dt,0),Vec3d.ZERO,inertia,q,Vec3d.ZERO,mass,1,0,dt,contacts(mass,dt,-.02,false));
        require(Math.abs(soft.angularVelocityBody().z())>Math.abs(rigid.angularVelocityBody().z())+.001,"finite support allows initial rocking");
        // Upward acceleration beyond weight requires no downward normal impulse.
        var lift=GroundContactImpulseSolver.solve(new Vec3d(0,(50000/mass-g)*dt,0),Vec3d.ZERO,inertia,q,Vec3d.ZERO,mass,1,0,dt,contacts(mass,dt,-.02,true));
        near(lift.normalImpulseNs(),0,1e-5,"contacts cannot oppose lift by pulling down");require(lift.velocityWorld().y()>0,"lift persists");
        // Complete loss of contact preserves COM velocity and all angular velocity.
        Vec3d velocity=new Vec3d(20,7,-3),omega=new Vec3d(.5,.8,-.4);
        var air=GroundContactImpulseSolver.solve(velocity,omega,inertia,q,Vec3d.ZERO,mass,1,0,dt,List.of());
        vector(air.velocityWorld(),velocity,1e-12,"uncapped airborne momentum");vector(air.angularVelocityBody(),omega,1e-12,"no upright correction");
        // High-wind impulses must overpower full brakes once current tire grip is exhausted.
        var wind=GroundContactImpulseSolver.solve(new Vec3d(18000*dt/mass,-g*dt,0),Vec3d.ZERO,inertia,q,Vec3d.ZERO,mass,1,0,dt,contacts(mass,dt,-.02,true));
        require(wind.velocityWorld().x()>.2,"wind-driven skid under full brake");
        for (int i=0;i<4;++i) {
            double cap=.25*wind.normalImpulses()[i];double f=wind.longitudinalImpulses()[i],s=wind.lateralImpulses()[i];
            require(wind.normalImpulses()[i]>=0,"unilateral tire load");require(cap<1e-8?Math.hypot(f,s)<1e-7:(f*f+s*s)/(cap*cap)<=1+1e-8,"shared brake/side grip bound");
        }
        var tip=GroundContactImpulseSolver.solve(new Vec3d(0,-g*dt,0),new Vec3d(0,0,2),inertia,q,Vec3d.ZERO,mass,1,0,dt,contacts(mass,dt,-.02,true));
        require(Arrays.stream(tip.normalImpulses()).anyMatch(n->n<1e-5),"lifting side unloads");require(tip.angularVelocityBody().z()>.05,"roll remains after the physical support impulse");
        var separated=TireNormalCompliance.response(mass/4,.03,.1,-1,.01,dt);near(separated.softnessInverseKg(),0,0,"airborne proximity has no spring");
        System.out.printf(Locale.ROOT,"Contact checks: PASS; first-step soft roll %.6f vs rigid %.6f rad/s; strong-wind brake slip %.6f m/s%n",soft.angularVelocityBody().z(),rigid.angularVelocityBody().z(),wind.velocityWorld().x());
        System.out.printf(Locale.ROOT,"Overturning checks: PASS; accepted roll %.6f rad/s, loaded contacts %d/4%n",tip.angularVelocityBody().z(),Arrays.stream(tip.normalImpulses()).filter(n->n>1e-5).count());
    }
}
