import com.g9third.pmweatheriv.network.VehiclePoseSnapshot;
import com.g9third.pmweatheriv.network.AircraftStateNetwork.PoseAnchorPayload;
import com.g9third.pmweatheriv.physics.BodyContactFriction;
import com.g9third.pmweatheriv.physics.BodyTerrainImpactPolicy;
import com.g9third.pmweatheriv.physics.TireNormalCompliance;
import com.g9third.pmweatheriv.physics.Vec3d;
import com.g9third.pmweatheriv.sable.SableVehicleBody;
import com.g9third.pmweatheriv.sable.SableVehicleManager;
import com.g9third.pmweatheriv.terrain.trueimpact.ExternalWorldImpactModel.Resolution;
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

/** Absolute authoritative poses, solid tire overlap and friction-bounded body contact. */
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
        Vec3d origin=new Vec3d(-31836.65,-2.4385,-68602.75), angles=new Vec3d(15,170,-20);
        Vec3d lostDelta=new Vec3d(-3.4453312054,.3594903195,8.7186999187);
        for(int i=0;i<1000;++i) {
            Vec3d sentDelta=new Vec3d(i*.17,Math.sin(i*.05),i*.31);
            Vec3d sentAngleDelta=new Vec3d(i*.23,i*1.7,i*2.1);
            Vec3d expected=origin.add(sentDelta), expectedAngles=angles.add(sentAngleDelta);
            // A complete snapshot repairs even a permanently missing native movement packet.
            // The old origin + received cumulative delta stayed wrong by lostDelta indefinitely.
            Vec3d oldCounterPose=origin.add(sentDelta.subtract(lostDelta));
            VehiclePoseSnapshot pose=new VehiclePoseSnapshot(expected,expectedAngles);
            require(pose.finite(),"valid authoritative pose");
            vector(pose.positionWorld(),expected,0,"position independent of native counters");
            vector(pose.anglesDegrees(),expectedAngles,0,"angle winding independent of native counters");
            vector(pose.positionWorld().subtract(oldCounterPose),lostDelta,1e-10,
                "observed missing movement offset repaired at large coordinates");
            PoseAnchorPayload packet=new PoseAnchorPayload(new UUID(1,2),
                ResourceLocation.fromNamespaceAndPath("minecraft","overworld"),123+i,pose);
            RegistryFriendlyByteBuf buffer=new RegistryFriendlyByteBuf(Unpooled.buffer(),RegistryAccess.EMPTY);
            try {
                PoseAnchorPayload.CODEC.encode(buffer,packet);
                PoseAnchorPayload decoded=PoseAnchorPayload.CODEC.decode(buffer);
                require(decoded.equals(packet),"complete pose codec roundtrip");
                require(buffer.readableBytes()==0,"complete pose codec consumes every byte");
                vector(decoded.anchor().positionWorld(),expected,0,"wire position remains absolute");
            } finally { buffer.release(); }
        }
        require(!new VehiclePoseSnapshot(new Vec3d(Double.NaN,0,0),Vec3d.ZERO).finite(),"NaN pose rejected");
        require(!new VehiclePoseSnapshot(Vec3d.ZERO,new Vec3d(0,Double.POSITIVE_INFINITY,0)).finite(),
            "infinite angles rejected");
        require(!new VehiclePoseSnapshot(null,Vec3d.ZERO).finite(),"missing position rejected");
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
    private static void bodyTerrainImpactPolicy() {
        Vec3d up=new Vec3d(0,1,0), adjacentSoilFace=new Vec3d(.1,.995,0), wall=new Vec3d(1,0,0);
        BodyTerrainImpactPolicy policy=new BodyTerrainImpactPolicy();
        var first=policy.classify(100,up,1.0,25.0);
        require(first.freshContact()&&first.damageEligible()
            &&BodyTerrainImpactPolicy.REASON_NEW_IMPACT.equals(first.reason()),
            "fresh body terrain closure at 1 m/s is eligible");
        var sameTickVoxel=policy.classify(100,adjacentSoilFace,1.1,30.0);
        require(sameTickVoxel.freshContact()&&sameTickVoxel.damageEligible(),
            "all same-owner-tick voxels retain initial fresh eligibility on an unchanged face");
        var steadySupport=policy.classify(101,up,.1,1.156);
        require(!steadySupport.freshContact()&&!steadySupport.damageEligible()
            &&BodyTerrainImpactPolicy.REASON_SUPPORT_CONTACT.equals(steadySupport.reason()),
            "steady 0.1 m/s body support is blocked after its first tick");
        var adjacentSupport=policy.classify(102,adjacentSoilFace,1.2,1.156);
        require(!adjacentSupport.freshContact()&&!adjacentSupport.damageEligible(),
            "sliding across adjacent soil with the same face normal does not reset impact freshness");
        var persistentReimpact=policy.classify(102,up,2.5,100.0);
        require(!persistentReimpact.freshContact()&&persistentReimpact.damageEligible()
            &&BodyTerrainImpactPolicy.REASON_REIMPACT.equals(persistentReimpact.reason()),
            "a hard normal reimpact remains eligible during continuous contact");
        var wallImpact=policy.classify(102,wall,1.0,20.0);
        require(wallImpact.freshContact()&&wallImpact.damageEligible(),
            "a new wall normal can impact while the body remains supported by ground");
        var persistentWallSupport=policy.classify(103,wall,1.2,20.0);
        require(!persistentWallSupport.freshContact()&&!persistentWallSupport.damageEligible(),
            "a wall normal stays persistent on the following support tick");

        BodyTerrainImpactPolicy grace=new BodyTerrainImpactPolicy();
        require(grace.classify(10,up,1.0,10.0).freshContact(),"first face observation is fresh");
        require(!grace.classify(12,up,1.0,10.0).freshContact(),
            "a face observed within the two-tick grace remains persistent");
        require(grace.classify(15,up,1.0,10.0).freshContact(),
            "a face unseen for more than two ticks becomes fresh");

        BodyTerrainImpactPolicy soft=new BodyTerrainImpactPolicy();
        var softFirst=soft.classify(1,up,.999,1.156);
        require(softFirst.freshContact()&&!softFirst.damageEligible()
            &&BodyTerrainImpactPolicy.REASON_SOFT_TOUCH.equals(softFirst.reason()),
            "fresh closure below 1 m/s is a blocked soft touch");
        var softRepeat=soft.classify(1,up,.999,1.156);
        require(softRepeat.freshContact()&&!softRepeat.damageEligible(),
            "under-threshold repeated same-tick samples stay blocked");
        require(!soft.classify(2,up,.999,1.156).damageEligible(),
            "under-1 m/s persistent support remains blocked");

        BodyTerrainImpactPolicy invalid=new BodyTerrainImpactPolicy();
        require(BodyTerrainImpactPolicy.REASON_INVALID.equals(invalid.classify(1,null,2.0,1.0).reason()),
            "null normal is rejected");
        require(BodyTerrainImpactPolicy.REASON_INVALID.equals(invalid.classify(1,Vec3d.ZERO,2.0,1.0).reason()),
            "zero normal is rejected");
        require(BodyTerrainImpactPolicy.REASON_INVALID.equals(invalid.classify(1,
            new Vec3d(Double.NaN,1,0),2.0,1.0).reason()),"nonfinite normal is rejected");
        require(BodyTerrainImpactPolicy.REASON_INVALID.equals(invalid.classify(1,up,Double.NaN,1.0).reason()),
            "nonfinite inward speed is rejected");
        require(BodyTerrainImpactPolicy.REASON_INVALID.equals(invalid.classify(1,up,1.0,Double.POSITIVE_INFINITY).reason()),
            "nonfinite impulse is rejected");
        require(BodyTerrainImpactPolicy.REASON_INVALID.equals(invalid.classify(1,up,0.0,1.0).reason()),
            "non-inward sample is rejected");
        require(BodyTerrainImpactPolicy.REASON_INVALID.equals(invalid.classify(1,up,1.0,0.0).reason()),
            "zero impulse is rejected");
        require(!invalid.classify(2,up,1.0,1.0).freshContact(),
            "quiet valid geometric contacts record support history");
        BodyTerrainImpactPolicy quiet=new BodyTerrainImpactPolicy();
        quiet.classify(1,up,4.0,30.0);
        for(int tick=2;tick<=20;++tick) quiet.classify(tick,up,0.0,0.0);
        require(!quiet.classify(21,up,1.2,30.0).damageEligible(),
            "a long zero-impulse support interval must not become a new soil strike");
        require(quiet.classify(25,up,1.2,30.0).damageEligible(),
            "a genuinely released face can become a fresh impact again");

        BodyTerrainImpactPolicy rollback=new BodyTerrainImpactPolicy();
        require(rollback.classify(20,up,1.0,1.0).freshContact(),"rollback fixture starts fresh");
        require(rollback.classify(19,up,1.0,1.0).freshContact(),
            "clock rollback clears the per-body direction history");

        // Distinct deterministic sphere directions force the fixed-size table
        // past capacity without accidentally matching an earlier canonical face.
        Vec3d[] directions=new Vec3d[BodyTerrainImpactPolicy.MAX_DIRECTIONS+1];
        Random directionRandom=new Random(20261010L);
        int found=0;
        while(found<directions.length) {
            Vec3d candidate=new Vec3d(directionRandom.nextDouble()*2-1,
                directionRandom.nextDouble()*2-1,directionRandom.nextDouble()*2-1).normalized();
            if(candidate.lengthSquared()<.5) continue;
            boolean distinct=true;
            for(int i=0;i<found;++i) if(candidate.dot(directions[i])>=BodyTerrainImpactPolicy.DIRECTION_MATCH_DOT) {
                distinct=false; break;
            }
            if(distinct) directions[found++]=candidate;
        }
        BodyTerrainImpactPolicy bounded=new BodyTerrainImpactPolicy();
        for(int i=0;i<directions.length;++i) bounded.classify(i,directions[i],.5,1.0);
        var evictedOldest=bounded.classify(directions.length,directions[0],1.0,1.0);
        require(evictedOldest.freshContact()&&evictedOldest.damageEligible(),
            "direction history is bounded at 16 entries and evicts its least-recent face");

        require(BodyTerrainImpactPolicy.requiresRigidManifold(true,false,0.0),
            "accepted deferred closed manifold keeps rigid contact");
        require(BodyTerrainImpactPolicy.requiresRigidManifold(false,true,2.0),
            "rejected manifold keeps rigid contact");
        require(!BodyTerrainImpactPolicy.requiresRigidManifold(true,true,2.0),
            "physically opened terrain with positive residual may use compliance");
        require(!BodyTerrainImpactPolicy.requiresRigidManifold(true,false,2.0),
            "intact material may support a positive normal residual");
        require(BodyTerrainImpactPolicy.requiresRigidManifold(true,true,-.1),
            "negative residual conservatively keeps rigid contact");
        require(BodyTerrainImpactPolicy.requiresRigidManifold(true,true,Double.NaN),
            "nonfinite residual conservatively keeps rigid contact");
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
    private static void legacyCrashHardness() {
        try {
            Method hardness = Class.forName("com.g9third.pmweatheriv.sable.SableCollisionGameplay")
                .getDeclaredMethod("legacyImpactHardness", float.class, Resolution.class);
            hardness.setAccessible(true);
            Resolution fractured = new Resolution(true,true,true,10000,9000,1000,1,
                3,0,0,0,1,"IMMEDIATE_MATERIAL_PENETRATION",2.1);
            near((double)hardness.invoke(null,.6f,fractured),2.1,1e-10,
                "legacy IV fallback counts every immediately fractured block's actual hardness");
            Resolution single = new Resolution(true,true,true,10000,9000,1000,1,
                1,0,0,0,1,"IMMEDIATE_MATERIAL_PENETRATION",(double).6f);
            near((double)hardness.invoke(null,.6f,single),(double).6f,1e-10,
                "legacy IV fallback does not count the seed voxel twice");
            Resolution oldConstructor = new Resolution(true,true,true,10000,9000,1000,1,
                2,0,0,0,1,"IMMEDIATE_MATERIAL_PENETRATION");
            near((double)hardness.invoke(null,.6f,oldConstructor),(double).6f,1e-10,
                "existing resolution constructor retains seed-hardness fallback");
            Resolution malformed = new Resolution(true,true,true,10000,9000,1000,1,
                2,0,0,0,1,"IMMEDIATE_MATERIAL_PENETRATION",Double.NaN);
            near((double)hardness.invoke(null,.6f,malformed),(double).6f,1e-10,
                "nonfinite fracture hardness does not corrupt crash classification");
        } catch (ReflectiveOperationException failure) {
            throw new AssertionError("legacy terrain hardness accounting is testable", failure);
        }
    }
    public static void main(String[] args) { anchors();gear();friction();bodyTerrainImpactPolicy();loadingFootprint();deferredEntityCleanup();legacyCrashHardness();
        System.out.println("ContactLifecycleRegression: "+assertions+" assertions passed"); }
}
