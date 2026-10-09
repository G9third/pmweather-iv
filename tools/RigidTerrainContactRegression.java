import com.g9third.pmweatheriv.physics.RigidContactSolver;
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
}
