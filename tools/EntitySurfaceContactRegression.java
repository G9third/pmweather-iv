import com.g9third.pmweatheriv.sable.*;
import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.jsondefs.JSONCollisionGroup.CollisionType;
import net.minecraft.core.Direction.Axis;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.shapes.Shapes;
import org.joml.Quaterniond;
import org.joml.Vector3d;
import java.lang.reflect.*;
import java.util.*;

/** Actual collision shapes and pose-carry math, without a running game or player network. */
public final class EntitySurfaceContactRegression {
    private static int assertions;
    private static Map<BoundingBox,Object> registry;
    private static Constructor<?> constructor;
    private static void require(boolean value,String message) {
        ++assertions;if(!value)throw new AssertionError(message);
    }
    private static void near(double a,double b,String message) {
        require(Double.isFinite(a)&&Math.abs(a-b)<2e-6,message+" actual="+a+" expected="+b);
    }
    private static BoundingBox box(Vector3d center,Quaterniond q,Vector3d half)throws Exception {
        BoundingBox b=new BoundingBox(new Point3D(center.x,center.y,center.z),half.x,half.y,half.z);
        registry.put(b,constructor.newInstance(center,q,half));return b;
    }
    private static OrientedEntityVoxelShape shape(BoundingBox b) {
        return new OrientedEntityVoxelShape(new OrientedEntityAabb(b,
            OrientedHitboxRegistry.conservativeWorldAabbForQuery(b, null, false)));
    }

    private static void setField(Object target,String name,Object value)throws Exception {
        for(Class<?> c=target.getClass();c!=null;c=c.getSuperclass()) {
            try {Field f=c.getDeclaredField(name);f.setAccessible(true);f.set(target,value);return;}
            catch(NoSuchFieldException ignored) {}
        }
        throw new NoSuchFieldException(name);
    }

    private static void movingSupport()throws Exception {
        Field unsafe=sun.misc.Unsafe.class.getDeclaredField("theUnsafe");unsafe.setAccessible(true);
        var vehicle=(EntityVehicleF_Physics)((sun.misc.Unsafe)unsafe.get(null))
            .allocateInstance(EntityVehicleF_Physics.class);
        setField(vehicle,"prevPosition",new Point3D());setField(vehicle,"position",new Point3D(0,.3,0));
        setField(vehicle,"prevOrientation",new RotationMatrix());setField(vehicle,"orientation",new RotationMatrix());
        BoundingBox roof=box(new Vector3d(0,.3,0),new Quaterniond(),new Vector3d(2,.25,2));
        setField(roof,"collisionTypes",Set.of(CollisionType.ENTITY));
        setField(vehicle,"allCollisionBoxes",new HashSet<>(Set.of(roof)));
        Constructor<?> tracked=Class.forName(OrientedHitboxRegistry.class.getName()+"$OrientedBox")
            .getDeclaredConstructors()[0];
        for(Constructor<?> c:tracked.getDeclaringClass().getDeclaredConstructors())
            if(c.getParameterCount()==5)tracked=c;
        tracked.setAccessible(true);
        registry.put(roof,tracked.newInstance(new Vector3d(0,.3,0),new Quaterniond(),
            new Vector3d(2,.25,2),vehicle,new Vector3d()));
        near(OrientedHitboxRegistry.verticalSurfaceAt(roof,0,0,true).y(),.25,
            "support acquisition uses the prior pose before a rising platform penetrates the feet");
        near(OrientedHitboxRegistry.verticalSurfaceAt(roof,0,0,false).y(),.55,
            "current support is evaluated after the carry");
        Method find=OrientedEntityRideSupport.class.getDeclaredMethod("findSupport",
            EntityVehicleF_Physics.class,BoundingBox.class,boolean.class);find.setAccessible(true);
        BoundingBox oldFeet=new BoundingBox(new Point3D(0,1.15,0),.3,.9,.3);
        require(find.invoke(null,vehicle,oldFeet,true)!=null,"old grounded feet qualify for carry");
        require(find.invoke(null,vehicle,oldFeet,false)==null,"new roof cannot falsely attach feet 0.3 blocks away");
        BoundingBox carriedFeet=new BoundingBox(new Point3D(0,1.45,0),.3,.9,.3);
        require(find.invoke(null,vehicle,carriedFeet,false)!=null,"one carry preserves actual current support");
        BoundingBox offEdge=new BoundingBox(new Point3D(3,1.45,0),.3,.9,.3);
        require(find.invoke(null,vehicle,offEdge,false)==null,"walking clear of the footprint releases support");
    }
    @SuppressWarnings("unchecked")
    public static void main(String[] args)throws Exception {
        Field map=OrientedHitboxRegistry.class.getDeclaredField("BOXES");map.setAccessible(true);
        registry=(Map<BoundingBox,Object>)map.get(null);
        Class<?> type=Class.forName(OrientedHitboxRegistry.class.getName()+"$OrientedBox");
        constructor=type.getDeclaredConstructor(Vector3d.class,Quaterniond.class,Vector3d.class);
        constructor.setAccessible(true);
        movingSupport();
        BoundingBox flat=box(new Vector3d(),new Quaterniond(),new Vector3d(2,.25,2));
        var flatShape=shape(flat);
        AABB above=new AABB(-.3,1,-.3,.3,2.8,.3);
        near(flatShape.collide(Axis.Y,above,-2),-.75,"landing clips at actual roof");
        near(Shapes.collide(Axis.Y,above,List.of(flatShape),-2),-.75,
            "Minecraft's native shape collision dispatcher uses the exact override");
        AABB standing=above.move(0,-.75,0);
        near(flatShape.collide(Axis.X,standing,4),4,"feet tangent to roof can walk off its edge");
        near(flatShape.collide(Axis.Y,standing,-.08),0,"supported feet retain grounded contact");
        near(flatShape.collide(Axis.Y,standing,.42),.42,"jumping away remains free");
        near(flatShape.move(0,3,0).collide(Axis.Y,above.move(0,3,0),-2),-.75,
            "shape translations retain exact collision rather than returning a solid envelope");
        require(flatShape.optimize()==flatShape,"optimization preserves exact shape identity");
        var surface=OrientedHitboxRegistry.verticalSurfaceAt(flat,0,0,false);
        near(surface.y(),.25,"support height matches real face");near(surface.normal().y,1,"support normal is up");
        BoundingBox diagonal=box(new Vector3d(),new Quaterniond().rotateY(Math.PI/4),new Vector3d(1,.25,.1));
        var diagShape=shape(diagonal);
        AABB emptyCorner=new AABB(.65,1,.65,.75,2,.75);
        near(diagShape.collide(Axis.Y,emptyCorner,-3),-3,
            "empty corner of a yawed conservative envelope cannot block a player");
        require(OrientedHitboxRegistry.verticalSurfaceAt(diagonal,.7,.7,false)==null,
            "empty envelope corner cannot become ride support");
        BoundingBox slope=box(new Vector3d(),new Quaterniond().rotateZ(Math.toRadians(60)),new Vector3d(2,.25,2));
        var steep=OrientedHitboxRegistry.verticalSurfaceAt(slope,0,0,false);
        near(steep.normal().y,.5,"steep roof is distinguished from walkable support");
        Vector3d motion=OrientedEntityRideSupport.pointMotion(new Vector3d(2,3,4),new Vector3d(),
            new Vector3d(0,1,0),new Quaterniond(),new Quaterniond());
        near(motion.y,1,"rising platform moves supported feet once, not twice");
        near(motion.x,0,"vertical carry invents no sideways correction");
        Random random=new Random(411);
        for(int i=0;i<160;i++) {
            Quaterniond q=new Quaterniond().rotateXYZ(random.nextDouble()*1.5-.75,
                random.nextDouble()*6,random.nextDouble()*1.5-.75);
            BoundingBox b=box(new Vector3d(10,4,-7),q,new Vector3d(2,.2,1));var s=shape(b);
            AABB bounds=s.bounds();
            double x=(bounds.minX+bounds.maxX)*.5+(random.nextDouble()-.5)*bounds.getXsize();
            double z=(bounds.minZ+bounds.maxZ)*.5+(random.nextDouble()-.5)*bounds.getZsize();
            AABB actor=new AABB(x-.1,bounds.maxY+1,z-.1,x+.1,bounds.maxY+2.8,z+.1);
            double requested=-bounds.getYsize()-3;
            double allowed=s.collide(Axis.Y,actor,requested);
            require(allowed<=0&&allowed>=requested,"clipping never reverses or adds motion");
            var before=actor.move(0,allowed,0);
            require(OrientedHitboxRegistry.contactWorldAabb(b,before.minX,before.minY,before.minZ,
                before.maxX,before.maxY,before.maxZ)==null,"accepted movement ends outside exact OBB");
            if(allowed>requested+1e-5) {
                var after=before.move(0,-1e-4,0);
                require(OrientedHitboxRegistry.contactWorldAabb(b,after.minX,after.minY,after.minZ,
                    after.maxX,after.maxY,after.maxZ)!=null,"stopped movement corresponds to a real face");
            }
        }
        System.out.println("EntitySurfaceContactRegression: "+assertions+" assertions passed");
    }
}
