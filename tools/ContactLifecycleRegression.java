import com.g9third.pmweatheriv.network.CumulativePoseAnchor;
import com.g9third.pmweatheriv.network.AircraftStateNetwork.PoseAnchorPayload;
import com.g9third.pmweatheriv.physics.BodyContactFriction;
import com.g9third.pmweatheriv.physics.TireNormalCompliance;
import com.g9third.pmweatheriv.physics.Vec3d;
import com.g9third.pmweatheriv.sable.SableVehicleBody;
import com.g9third.pmweatheriv.sable.SableVehicleManager;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import io.netty.buffer.Unpooled;
import net.minecraft.core.RegistryAccess;
import net.minecraft.network.RegistryFriendlyByteBuf;
import net.minecraft.resources.ResourceLocation;
import java.util.Random;
import java.util.UUID;
import java.lang.reflect.Method;
import static com.g9third.pmweatheriv.physics.FlightMath.*;

/** Absolute cumulative-stream anchors, solid tire overlap and friction-bounded body contact. */
public final class ContactLifecycleRegression {
    private static int assertions;
    private static void require(boolean v, String message) {
        ++assertions; if (!v) throw new AssertionError(message);
    }
    private static void near(double a, double b, double tolerance, String message) {
        require(Double.isFinite(a) && Double.isFinite(b) && Math.abs(a-b)<=tolerance,
            message + ": " + a + " vs " + b);
    }
    private static void vector(Vec3d a, Vec3d b, double tolerance, String message) {
        near(a.x(),b.x(),tolerance,message); near(a.y(),b.y(),tolerance,message); near(a.z(),b.z(),tolerance,message);
    }
    private static double energy(Vec3d v, Vec3d omegaWorld, RotationMatrix r, Vec3d inertia, double mass) {
        Point3D omegaPoint=new Point3D(omegaWorld.x(),omegaWorld.y(),omegaWorld.z()).reOrigin(r);
        Vec3d omega=new Vec3d(omegaPoint.x,omegaPoint.y,omegaPoint.z);
        return .5*mass*v.lengthSquared()+.5*(inertia.x()*omega.x()*omega.x()
            +inertia.y()*omega.y()*omega.y()+inertia.z()*omega.z()*omega.z());
    }
    private static void anchors() {
        Vec3d origin=new Vec3d(-140.2,25.6,183.1), angles=new Vec3d(15,170,-20);
        Vec3d initialMismatch=new Vec3d(-.34973166325,.0094067377755,-1.35465869538);
        for(int i=0;i<1000;++i) {
            Vec3d sentDelta=new Vec3d(i*.17,Math.sin(i*.05),i*.31);
            Vec3d sentAngleDelta=new Vec3d(i*.23,i*1.7,i*2.1);
            CumulativePoseAnchor anchor=CumulativePoseAnchor.capture(origin.add(sentDelta),angles.add(sentAngleDelta),sentDelta,sentAngleDelta);
            require(anchor.finite(),"valid anchor");
            // Different channel arrival orders and native packet cadence preserve the same absolute baseline.
            for(int delay : new int[]{-2,0,2}) {
                Vec3d receivedDelta=sentDelta.add(new Vec3d(delay*.17,0,delay*.31));
                Vec3d expected=origin.add(receivedDelta);
                vector(anchor.position(receivedDelta),expected,1e-9,"packet-independent absolute position");
                vector(anchor.angles(sentAngleDelta),angles.add(sentAngleDelta),1e-9,"Euler cumulative winding");
            }
            Vec3d wrongClient=origin.add(sentDelta).add(initialMismatch);
            vector(anchor.position(sentDelta).subtract(wrongClient),initialMismatch.negate(),1e-9,"stopped/reconnect baseline repair");
        }
        require(!new CumulativePoseAnchor(new Vec3d(Double.NaN,0,0),Vec3d.ZERO).finite(),"invalid anchor rejected");
        PoseAnchorPayload p=new PoseAnchorPayload(new UUID(1,2),ResourceLocation.fromNamespaceAndPath("minecraft","overworld"),123,
            new CumulativePoseAnchor(origin,angles));
        RegistryFriendlyByteBuf buffer=new RegistryFriendlyByteBuf(Unpooled.buffer(),RegistryAccess.EMPTY);
        try {
            PoseAnchorPayload.CODEC.encode(buffer,p);
            require(PoseAnchorPayload.CODEC.decode(buffer).equals(p),"absolute pose codec roundtrip");
            require(buffer.readableBytes()==0,"absolute pose codec consumes complete record");
        } finally { buffer.release(); }
    }
    private static void gear() {
        near(TireNormalCompliance.upwardPoseCorrection(.20,.03,.01),0,0,"airborne gear has no seating target");
        near(TireNormalCompliance.upwardPoseCorrection(-.03,.03,.01),0,0,"normal suspension deflection retained");
        near(TireNormalCompliance.upwardPoseCorrection(-.08,.03,.01),0,1e-12,"bump boundary retained");
        near(TireNormalCompliance.upwardPoseCorrection(-.41365034974241155,.03,.01),.33365034974241155,1e-12,"observed deep wheel overlap correction");
        near(TireNormalCompliance.upwardPoseCorrection(-.02,0,.01),.015,1e-12,"rigid/flat support overlap");
        near(TireNormalCompliance.upwardPoseCorrection(Double.NaN,.03,.01),0,0,"invalid geometry correction ignored");
    }
    private static void friction() {
        RotationMatrix identity=new RotationMatrix().setToAngles(new Point3D());
        Vec3d inertia=new Vec3d(8000,20000,13000),up=new Vec3d(0,1,0),v=Vec3d.ZERO,w=new Vec3d(0,1,0);
        var airborne=BodyContactFriction.solve(v,w,identity,new Vec3d(2,-.5,0),up,inertia,1220,0,.15);
        vector(airborne.angularVelocityWorld(),w,0,"airborne angular momentum retained");
        var contact=BodyContactFriction.solve(v,w,identity,new Vec3d(2,-.5,0),up,inertia,1220,300,.15);
        require(contact.angularVelocityWorld().y()<w.y(),"body contact opposes observed wreck yaw");
        require(contact.impulseWorld().length()<=45+1e-10,"body tangent Coulomb impulse bound");
        Random random=new Random(33);
        for(int i=0;i<1000;++i) {
            RotationMatrix orientation=new RotationMatrix().setToAngles(new Point3D(random.nextDouble()*360-180,
                random.nextDouble()*360-180,random.nextDouble()*360-180));
            Vec3d normal=new Vec3d(random.nextDouble()-.5,random.nextDouble()-.5,random.nextDouble()-.5).normalized();
            Vec3d lever=new Vec3d(random.nextDouble()*6-3,random.nextDouble()*3-1.5,random.nextDouble()*6-3);
            Vec3d velocity=new Vec3d(random.nextDouble()*20-10,random.nextDouble()*20-10,random.nextDouble()*20-10);
            Vec3d omega=new Vec3d(random.nextDouble()*6-3,random.nextDouble()*6-3,random.nextDouble()*6-3);
            double jn=random.nextDouble()*5000,mu=random.nextDouble()*.5;
            var result=BodyContactFriction.solve(velocity,omega,orientation,lever,normal,inertia,1220,jn,mu);
            require(result.velocityWorld().isFinite()&&result.angularVelocityWorld().isFinite(),"finite body contact result");
            require(result.impulseWorld().length()<=jn*mu+1e-8,"shared normal impulse friction bound");
            near(result.impulseWorld().dot(normal),0,1e-8,"friction tangent to actual terrain normal");
            double beforeNormal=velocity.add(omega.cross(lever)).dot(normal);
            double afterNormal=result.velocityWorld().add(result.angularVelocityWorld().cross(lever)).dot(normal);
            require(afterNormal>=beforeNormal-1e-8,"friction preserves material normal response");
            require(energy(result.velocityWorld(),result.angularVelocityWorld(),orientation,inertia,1220)
                <=energy(velocity,omega,orientation,inertia,1220)+1e-7,"friction never creates kinetic energy");
            vector(result.velocityWorld().subtract(velocity),result.impulseWorld().scale(1.0/1220),1e-10,"COM impulse consistency");
        }
        // Persistent off-centre body contact slows a wreck without a speed or attitude cap.
        Vec3d velocity=Vec3d.ZERO,omega=new Vec3d(0,1.5,0);
        for(int i=0;i<1600;++i) {
            Vec3d lever=i%2==0 ? new Vec3d(2,-.5,0) : new Vec3d(-2,-.5,0);
            var result=BodyContactFriction.solve(velocity,omega,identity,lever,up,inertia,1220,1220*9.80665*.025,.15);
            velocity=result.velocityWorld();omega=result.angularVelocityWorld();
        }
        require(Math.abs(omega.y())<.01,"contact friction settles wreck yaw over repeated physical support steps");
    }
    private static void loadingFootprint() {
        double measuredCompoundRadius = 33.775790704339315;
        near(SableVehicleBody.conservativeLoadingRadius(27.0, measuredCompoundRadius, 0.0),
            measuredCompoundRadius, 1e-12,
            "initialized compound replaces smaller model-only preflight footprint");
        near(SableVehicleBody.conservativeLoadingRadius(27.0, 0.0, measuredCompoundRadius),
            measuredCompoundRadius, 1e-12,
            "measured compound footprint survives collider removal on chunk unload");
        near(SableVehicleBody.conservativeLoadingRadius(27.0, 31.0, measuredCompoundRadius),
            measuredCompoundRadius, 1e-12,
            "retained footprint never shrinks during temporary residency holds");
        near(SableVehicleBody.conservativeLoadingRadius(27.0, 35.0, measuredCompoundRadius),
            35.0, 1e-12,
            "a later larger mounted compound grows the retained footprint");
        near(SableVehicleBody.conservativeLoadingRadius(Double.NaN, Double.NaN, Double.NaN),
            2.0, 0.0,
            "invalid footprint inputs retain the minimum safe loading radius");
    }
    private static boolean managerPolicy(String methodName, Class<?>[] parameterTypes, Object... arguments) {
        try {
            Method method = SableVehicleManager.class.getDeclaredMethod(methodName, parameterTypes);
            if (!method.trySetAccessible()) {
                throw new AssertionError("manager lifecycle policy is accessible: " + methodName);
            }
            return (boolean) method.invoke(null, arguments);
        } catch (ReflectiveOperationException failure) {
            throw new AssertionError("manager lifecycle policy is testable: " + methodName, failure);
        }
    }
    private static void deferredEntityCleanup() {
        require(managerPolicy("shouldClearLifecycleAfterEntityRemoval", new Class<?>[] {boolean.class}, false),
            "removed deferred vehicle clears retained lifecycle state");
        require(!managerPolicy("shouldClearLifecycleAfterEntityRemoval", new Class<?>[] {boolean.class}, true),
            "active Rapier body keeps its native-thread retirement path");
        Object unloadingLevel = new Object();
        require(managerPolicy("sameLevelOwner", new Class<?>[] {Object.class, Object.class}, unloadingLevel,
            unloadingLevel),
            "matching level identity owns deferred footprint cleanup");
        require(!managerPolicy("sameLevelOwner", new Class<?>[] {Object.class, Object.class}, unloadingLevel,
            new Object()),
            "another level cannot clear deferred footprint ownership");
    }
    public static void main(String[] args) { anchors();gear();friction();loadingFootprint();deferredEntityCleanup();
        System.out.println("ContactLifecycleRegression: "+assertions+" assertions passed"); }
}
