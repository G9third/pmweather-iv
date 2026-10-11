import com.g9third.pmweatheriv.physics.RigidContactSolver;
import com.g9third.pmweatheriv.physics.TerrainContactGeometry;
import com.g9third.pmweatheriv.physics.Vec3d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.joml.Quaterniond;
import org.joml.Vector3d;

public class RigidTerrainContactRegression {
    private static void require(boolean value, String label) {
        if (!value) throw new AssertionError(label);
    }
    private static double energy(Vector3d v, Vector3d w, Quaterniond q, Vec3d inertia, double mass) {
        Vector3d local = q.transformInverse(w, new Vector3d());
        return 0.5*(mass*v.lengthSquared() + inertia.x()*local.x*local.x
            + inertia.y()*local.y*local.y + inertia.z()*local.z*local.z);
    }
    private static RigidContactSolver.Result check(Vector3d v, Vector3d w, Quaterniond q,
            List<RigidContactSolver.Contact> contacts, String label) {
        Vec3d inertia = new Vec3d(7871.8, 20391.7, 13283.2);
        var result = RigidContactSolver.solve(v, w, new Vector3d(), q, inertia, 1220, contacts);
        require(result.maximumResidualMps() < 0.0001, label + " unresolved inward motion");
        require(energy(result.linearWorld(), result.angularWorld(), q, inertia, 1220)
            <= energy(v, w, q, inertia, 1220) + 0.001, label + " created energy");
        for (double impulse : result.normalImpulses()) require(impulse >= 0, label + " adhesive contact");
        return result;
    }
    public static void main(String[] args) {
        contactPatch();
        releasedMaterialStation();
        List<RigidContactSolver.Contact> floor = new ArrayList<>();
        for (double x : new double[]{-4,4}) for (double z : new double[]{-3,3})
            floor.add(new RigidContactSolver.Contact(new Vector3d(x,-1,z), new Vector3d(0,1,0),0));
        var a = check(new Vector3d(0,-5,0), new Vector3d(), new Quaterniond(), floor,"symmetric floor");
        require(a.linearWorld().length() < 0.0001 && a.angularWorld().length() < 0.0001,
            "flat impact must settle without manufactured rotation or rebound");
        List<RigidContactSolver.Contact> reversed = new ArrayList<>(floor);
        Collections.reverse(reversed);
        var b = check(new Vector3d(0,-5,0),new Vector3d(),new Quaterniond(),reversed,"reversed floor");
        require(a.linearWorld().distance(b.linearWorld()) < 0.0001
            && a.angularWorld().distance(b.angularWorld()) < 0.0001,"contact order bias");
        List<RigidContactSolver.Contact> corner = new ArrayList<>(floor);
        for (double y : new double[]{-1,1}) for (double z : new double[]{-3,3})
            corner.add(new RigidContactSolver.Contact(new Vector3d(-4,y,z),new Vector3d(1,0,0),0));
        var wall = check(new Vector3d(-12,-4,6),new Vector3d(),
            new Quaterniond().rotationXYZ(0.4,0.7,-0.3),corner,"wall plus floor");
        require(Math.abs(wall.linearWorld().z-6) < 1e-8,"free tangent momentum must remain");
        check(new Vector3d(0,-0.245,0),new Vector3d(0.03,0.02,-0.04),
            new Quaterniond().rotationXYZ(0.6,-0.8,2.9),floor,"wreck support");
        var material = check(new Vector3d(0,-10,0),new Vector3d(),new Quaterniond(),
            List.of(new RigidContactSolver.Contact(new Vector3d(0,-1,0),
                new Vector3d(0,1,0),8)),"material residual");
        require(Math.abs(material.linearWorld().y+8) < 1e-8,"material momentum budget lost");
        var free = check(new Vector3d(10,4,3),new Vector3d(),new Quaterniond(),floor,"separating");
        require(free.linearWorld().distance(new Vector3d(10,4,3)) < 1e-8,"separating motion damped");
        System.out.println("RigidTerrainContactRegression: compound support, order, energy, tangent and material checks passed");
    }

    private static void releasedMaterialStation() {
        var station=new RigidContactSolver.Contact(new Vector3d(4,-1,0),new Vector3d(0,1,0),0);
        var center=new RigidContactSolver.Contact(new Vector3d(0,-1,0),new Vector3d(0,1,0),0);
        var solo=check(new Vector3d(0,-5,0),new Vector3d(),new Quaterniond(),
            List.of(station),"solo material candidate");
        var coupled=check(new Vector3d(0,-5,0),new Vector3d(),new Quaterniond(),
            List.of(station,center),"released material station");
        require(solo.normalImpulses()[0]>0,"isolated station fixture must estimate impact work");
        require(coupled.normalImpulses()[0]<.001,
            "released station has no material work even though its initial closing speed is positive");
        require(coupled.normalImpulses()[1]>0,"actual load-bearing station retains its impulse");
        var yielding=new RigidContactSolver.Contact(station.pointWorld(),station.normalWorld(),4);
        var surviving=check(new Vector3d(0,-5,0),new Vector3d(),new Quaterniond(),
            List.of(yielding,center),"yielding plus surviving terrain");
        require(surviving.linearWorld().y>=-1e-5,
            "opening one material station cannot bypass the surviving center support");
    }

    private static void contactPatch() {
        Vector3d x = new Vector3d(1,0,0), y = new Vector3d(0,1,0), z = new Vector3d(0,0,1);
        Vector3d p = TerrainContactGeometry.contactPoint(new Vector3d(0,.5,0), x,y,z,8,.5,2,
            new Vector3d(6,-1,-.5), new Vector3d(7,0,.5));
        require(p != null && p.x >= 6-1e-7 && p.x <= 7+1e-7
            && Math.abs(p.y) < 1e-7 && Math.abs(p.z) <= .5+1e-7,
            "long wing contact must lie in the actual struck terrain patch");
        require(p.distance(new Vector3d(6.5,0,0)) < 1e-6,
            "face contact must use the overlap center, not the cuboid center");
        require(TerrainContactGeometry.contactPoint(new Vector3d(0,5,0),x,y,z,1,1,1,
            new Vector3d(-1,-1,-1),new Vector3d(1,0,1)) == null,
            "separated shapes have no contact patch");
        Quaterniond q = new Quaterniond().rotationXYZ(.3,.7,-.2);
        Vector3d rx=q.transform(x,new Vector3d()), ry=q.transform(y,new Vector3d()),
            rz=q.transform(z,new Vector3d());
        p=TerrainContactGeometry.contactPoint(new Vector3d(),rx,ry,rz,2,.3,1,
            new Vector3d(.5,-.2,-.2),new Vector3d(1,.2,.2));
        require(p != null && p.x >= .5-1e-7 && p.x <= 1+1e-7
            && Math.abs(p.y) <= .2+1e-7 && Math.abs(p.z) <= .2+1e-7,
            "rotated cuboid patch stays inside the contacted terrain shape");
        require(Math.abs(p.dot(rx)) <= 2+1e-7 && Math.abs(p.dot(ry)) <= .3+1e-7
            && Math.abs(p.dot(rz)) <= 1+1e-7,
            "rotated patch also stays inside the vehicle collision shape");
        // The same physical strike expressed at a distant world origin must
        // preserve the torque arm and inelastic result.
        Vector3d origin = new Vector3d(10000,-58,-30000), point = new Vector3d(2,-1,4);
        var local = RigidContactSolver.solve(new Vector3d(0,-90,-110),new Vector3d(),
            new Vector3d(),q,new Vec3d(130000,237000,117000),9300,
            List.of(new RigidContactSolver.Contact(point,new Vector3d(0,1,0),0)));
        var world = RigidContactSolver.solve(new Vector3d(0,-90,-110),new Vector3d(),
            origin,q,new Vec3d(130000,237000,117000),9300,
            List.of(new RigidContactSolver.Contact(new Vector3d(point).add(origin),new Vector3d(0,1,0),0)));
        require(local.linearWorld().distance(world.linearWorld()) < 1e-8
            && local.angularWorld().distance(world.angularWorld()) < 1e-8,
            "time-of-impact contact lever is invariant to world coordinates");
    }
}
