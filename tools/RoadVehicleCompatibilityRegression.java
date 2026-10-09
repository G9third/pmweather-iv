import com.g9third.pmweatheriv.physics.LiquidSupportModel;
import com.g9third.pmweatheriv.physics.NativeBallastModel;
import com.g9third.pmweatheriv.physics.SkidSteerDriveDemand;
import com.g9third.pmweatheriv.physics.GroundContactImpulseSolver;
import com.g9third.pmweatheriv.physics.Vec3d;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import org.joml.Quaterniond;
import java.util.List;

/** Pure regression cases for authored float/ballast behavior and skid-steer demand. */
public final class RoadVehicleCompatibilityRegression {
    private static int assertions;
    private RoadVehicleCompatibilityRegression() {}

    public static void main(String[] args) {
        liquidSupportSeparatesPontoonFromTireAndBallast();
        authoredLiquidBoxesProvideGenericHullStations();
        ballastUsesAuthoredControlAndWaterInputs();
        skidSteerProducesBoundablePivotAndReverseTurnDemand();
        groundSolverConstrainsFourCornerLiquidHullWithoutTireGrip();
        groundSolverAppliesFrictionBoundedTrackDifferentialAtRest();
        System.out.println("RoadVehicleCompatibilityRegression: " + assertions + " assertions passed (helper/contact fixtures; no game simulation)");
    }

    private static void authoredLiquidBoxesProvideGenericHullStations() {
        require(LiquidSupportModel.isAuthoredLiquidCollisionBox(true, true,
            0.25, 0.0625, 0.25),
            "authored IV BLOCK hull boxes marked collidesWithLiquids are float stations");
        require(!LiquidSupportModel.isAuthoredLiquidCollisionBox(false, true,
            0.25, 0.0625, 0.25),
            "vehicle/query-only boxes do not create fluid support");
        require(!LiquidSupportModel.isAuthoredLiquidCollisionBox(true, false,
            0.25, 0.0625, 0.25),
            "solid-only authored hull boxes do not create fluid support");
        require(!LiquidSupportModel.isAuthoredLiquidCollisionBox(true, true,
            0.25, 0.0, 0.25),
            "zero-height boxes do not create support stations");

        Quaterniond ownerToVehicle = new Quaterniond().rotationZ(Math.PI / 2.0);
        Vec3d[] rotatedCorners = LiquidSupportModel.bottomCornersVehicleLocal(
            new Vec3d(0.0, 1.0, 0.0), 0.25, 0.5, 0.25,
            new Vec3d(1.0, 2.0, 3.0), ownerToVehicle);
        double[][] expectedRotated = {
            {0.5, 1.75, 2.75}, {0.5, 1.75, 3.25},
            {0.5, 2.25, 2.75}, {0.5, 2.25, 3.25}
        };
        for (int i = 0; i < expectedRotated.length; ++i) {
            near(expectedRotated[i][0], rotatedCorners[i].x(), 1.0e-12,
                "part-owned lower corner rotates into vehicle x " + i);
            near(expectedRotated[i][1], rotatedCorners[i].y(), 1.0e-12,
                "part-owned box lower face remains at live bottom " + i);
            near(expectedRotated[i][2], rotatedCorners[i].z(), 1.0e-12,
                "part origin is retained in vehicle-frame z " + i);
        }

        // Public boat fixture: its eight collidesWithLiquids boxes sit at the
        // four bow/stern-side hull stations (JSONCollisionBox centers, already
        // scaled by BoundingBox.updateToEntity in the runtime path).
        Vec3d[] boatBoxCenters = {
            new Vec3d(0.25, -0.15, -0.25), new Vec3d(0.25, -0.15, -0.75),
            new Vec3d(0.25, -0.15, 0.25), new Vec3d(0.25, -0.15, 0.75),
            new Vec3d(-0.25, -0.15, -0.25), new Vec3d(-0.25, -0.15, -0.75),
            new Vec3d(-0.25, -0.15, 0.25), new Vec3d(-0.25, -0.15, 0.75)
        };
        Quaterniond noRotation = new Quaterniond();
        for (int i = 0; i < boatBoxCenters.length; ++i) {
            Vec3d[] corners = LiquidSupportModel.bottomCornersVehicleLocal(
                boatBoxCenters[i], 0.25, 0.0625, 0.25, Vec3d.ZERO, noRotation
            );
            for (int corner = 0; corner < corners.length; ++corner) {
                double expectedX = boatBoxCenters[i].x() + (corner < 2 ? -0.25 : 0.25);
                double expectedZ = boatBoxCenters[i].z() + (corner % 2 == 0 ? -0.25 : 0.25);
                near(expectedX, corners[corner].x(), 1.0e-12,
                    "authored hull corner x remains correctly spaced " + i + "/" + corner);
                near(-0.2125, corners[corner].y(), 1.0e-12,
                    "authored hull corner uses every box bottom " + i + "/" + corner);
                near(expectedZ, corners[corner].z(), 1.0e-12,
                    "authored hull corner z remains correctly spaced " + i + "/" + corner);
            }
        }

        // Simulate nonuniform owner scale already applied by IV: center
        // (1,.75,1) * (2,2,4) and half extents (.5,.25,.5) * (2,2,4).
        // The support transform consumes those live scaled values once.
        Vec3d[] scaledCorners = LiquidSupportModel.bottomCornersVehicleLocal(
            new Vec3d(2.0, 1.5, 4.0), 1.0, 0.5, 2.0, Vec3d.ZERO, noRotation
        );
        double[][] expectedScaled = {
            {1.0, 1.0, 2.0}, {1.0, 1.0, 6.0},
            {3.0, 1.0, 2.0}, {3.0, 1.0, 6.0}
        };
        for (int i = 0; i < expectedScaled.length; ++i) {
            near(expectedScaled[i][0], scaledCorners[i].x(), 1.0e-12,
                "pre-scaled x half extent is applied once " + i);
            near(expectedScaled[i][1], scaledCorners[i].y(), 1.0e-12,
                "pre-scaled box height reaches bottom once " + i);
            near(expectedScaled[i][2], scaledCorners[i].z(), 1.0e-12,
                "pre-scaled z half extent is applied once " + i);
        }
    }

    private static void liquidSupportSeparatesPontoonFromTireAndBallast() {
        near(65.0, LiquidSupportModel.surfaceY(64.0, 1.0, true), 1.0e-12,
            "source fluid surface");
        near(64.875, LiquidSupportModel.surfaceY(64.0, 0.875, true), 1.0e-12,
            "partial fluid surface");
        require(Double.isNaN(LiquidSupportModel.surfaceY(64.0, 1.0, false)),
            "ordinary gear must not receive liquid support");
        near(0.0, LiquidSupportModel.penetrationMeters(-0.4, true), 0.0,
            "submerged float station must not be projected out of the fluid");
        near(0.4, LiquidSupportModel.penetrationMeters(-0.4, false), 1.0e-12,
            "solid gear penetration remains physical");
        require(!LiquidSupportModel.hasTireTraction(true),
            "fluid support must not spend solid tire friction");
        require(LiquidSupportModel.hasTireTraction(false),
            "solid support keeps authored tire friction");
    }

    private static void ballastUsesAuthoredControlAndWaterInputs() {
        double scale = NativeBallastModel.ivForceUnitNewtons(1.0);
        NativeBallastModel.Loads rise = NativeBallastModel.evaluate(
            10.0, true, false, -1.0, 0.0, 1.0, 1000.0, 1000.0, 1.0,
            true, 0.5, true, scale);
        NativeBallastModel.Loads descend = NativeBallastModel.evaluate(
            10.0, true, false, 1.0, 0.0, 1.225, 1000.0, 1000.0, 1.0,
            true, 0.5, true, scale);
        near(400.0, rise.verticalControlForceNewtons(), 1.0e-9,
            "negative ballast control rises using current air density");
        near(-490.0, descend.verticalControlForceNewtons(), 1.0e-9,
            "positive ballast control descends using IV reference density");

        NativeBallastModel.Loads wetGravity = NativeBallastModel.evaluate(
            0.0, false, false, 0.0, 0.0, 1.225, 1000.0, 1000.0, 1.0,
            true, 0.5, true, scale);
        NativeBallastModel.Loads dryGravity = NativeBallastModel.evaluate(
            0.0, false, false, 0.0, 0.0, 1.225, 1000.0, 1000.0, 1.0,
            true, 0.5, false, scale);
        NativeBallastModel.Loads activeBallast = NativeBallastModel.evaluate(
            10.0, true, false, 0.0, 0.0, 1.225, 1000.0, 1000.0, 1.0,
            true, 0.5, true, scale);
        NativeBallastModel.Loads wreck = NativeBallastModel.evaluate(
            10.0, true, true, -1.0, 0.0, 1.225, 1000.0, 1000.0, 1.0,
            true, 0.5, true, scale);
        near(4903.325, wetGravity.verticalGravityCorrectionNewtons(), 1.0e-8,
            "waterBallastFactor reduces native gravity in liquid without ballast volume");
        near(0.0, dryGravity.verticalGravityCorrectionNewtons(), 1.0e-8,
            "waterBallastFactor has no effect outside liquid");
        near(9806.65, activeBallast.verticalGravityCorrectionNewtons(), 1.0e-8,
            "healthy active ballast replaces ordinary Sable gravity");
        near(0.0, -9806.65 + activeBallast.netWorldVerticalForceNewtons(), 1.0e-8,
            "healthy active ballast does not receive duplicate Sable weight");
        near(-9806.65, -9806.65 + dryGravity.netWorldVerticalForceNewtons(), 1.0e-8,
            "ordinary gravity remains standard SI gravity");
        near(4903.325, wreck.verticalGravityCorrectionNewtons(), 1.0e-8,
            "wreck restores water-adjusted native gravity and suppresses ballast control");
        near(0.0, wreck.verticalControlForceNewtons(), 0.0,
            "wrecked vehicle has no active ballast control force");

        NativeBallastModel.Loads neutralDamping = NativeBallastModel.evaluate(
            10.0, true, false, 0.0, 0.2, 1.225, 1000.0, 1000.0, 1.0,
            false, 0.0, false, scale);
        require(neutralDamping.verticalControlForceNewtons() < 0.0,
            "neutral ballast damps upward native vertical motion");

        for (double speedFactor : new double[] {0.25, 0.35, 1.0}) {
            NativeBallastModel.Loads ordinary = NativeBallastModel.evaluate(
                0.0, false, false, 0.0, 0.0, 1.225, 850.0, 1200.0, 1.0,
                false, 0.0, false, NativeBallastModel.ivForceUnitNewtons(speedFactor));
            near(0.0, ordinary.verticalGravityCorrectionNewtons(), 1.0e-12,
                "ordinary SI gravity must use Sable body mass and ignore speedFactor " + speedFactor);
        }

        NativeBallastModel.Loads gravityMultiplier = NativeBallastModel.evaluate(
            0.0, false, false, 0.0, 0.0, 1.225, 700.0, 1000.0, 0.35,
            false, 0.0, false, scale);
        near(6374.3225, gravityMultiplier.verticalGravityCorrectionNewtons(), 1.0e-8,
            "authored gravityFactor scales physical body weight rather than IV speedFactor gravity");

        NativeBallastModel.Loads waterFactorWithoutBallast = NativeBallastModel.evaluate(
            0.0, false, false, 0.0, 0.0, 1.225, 1000.0, 1000.0, 1.0,
            true, 0.5, true, scale);
        near(4903.325, waterFactorWithoutBallast.verticalGravityCorrectionNewtons(), 1.0e-8,
            "water factor multiplies physical body weight even with zero ballast volume");
    }

    private static void skidSteerProducesBoundablePivotAndReverseTurnDemand() {
        double[] symmetricX = {-1.0, 1.0};
        boolean[] bothSides = {true, true};
        double[] pivot = SkidSteerDriveDemand.allocate(0.0, 100.0, symmetricX, bothSides);
        near(0.0, pivot[0] + pivot[1], 1.0e-12,
            "stationary pivot has no net longitudinal impulse");
        near(100.0, -symmetricX[0] * pivot[0] - symmetricX[1] * pivot[1], 1.0e-12,
            "pivot delivers requested positive yaw impulse");

        double[] offsetComPivot = SkidSteerDriveDemand.allocate(
            0.0, 100.0, new double[] {-3.0, -1.0}, symmetricX, bothSides);
        near(0.0, offsetComPivot[0] + offsetComPivot[1], 1.0e-12,
            "model-center side detection survives an offset center of mass");
        near(100.0, 3.0 * offsetComPivot[0] + 1.0 * offsetComPivot[1], 1.0e-12,
            "offset-COM skid demand still applies its requested yaw moment");

        double[] asymmetricX = {-1.3, -0.7, 0.8, 1.4};
        boolean[] loaded = {true, true, true, true};
        double[] reverseTurn = SkidSteerDriveDemand.allocate(-60.0, 100.0, asymmetricX, loaded);
        double sum = 0.0, yaw = 0.0;
        for (int i = 0; i < loaded.length; ++i) {
            sum += reverseTurn[i];
            yaw -= asymmetricX[i] * reverseTurn[i];
        }
        near(-60.0, sum, 1.0e-12,
            "reverse engine demand remains net reverse after differential allocation");
        double commonDriveYaw = -(-60.0)
            * (asymmetricX[0] + asymmetricX[1] + asymmetricX[2] + asymmetricX[3]) / 4.0;
        near(100.0, yaw - commonDriveYaw, 1.0e-12,
            "asymmetric contacts add only the requested differential moment");

        double[] ordinaryCar = SkidSteerDriveDemand.allocate(-60.0, 0.0, asymmetricX, loaded);
        for (double demand : ordinaryCar) near(-15.0, demand, 1.0e-12,
            "ordinary car retains even native drive demand");

        double[] oneTrack = SkidSteerDriveDemand.allocate(
            20.0, 100.0, new double[] {-1.3, -0.7}, new boolean[] {true, true});
        near(10.0, oneTrack[0], 1.0e-12,
            "missing opposite-side contact suppresses differential yaw demand");
        near(10.0, oneTrack[1], 1.0e-12,
            "missing opposite-side contact preserves ordinary drive split");

        boolean[] oneStationUnloaded = {true, false, true, true};
        double[] reducedSet = SkidSteerDriveDemand.allocate(0.0, 80.0,
            asymmetricX, oneStationUnloaded);
        double reducedYaw = 0.0;
        for (int i = 0; i < reducedSet.length; ++i) reducedYaw -= asymmetricX[i] * reducedSet[i];
        near(80.0, reducedYaw, 1.0e-12,
            "differential demand redistributes across the remaining loaded contacts");
        near(0.0, reducedSet[1], 0.0,
            "an unloaded station receives no drive or differential demand");

        require(SkidSteerDriveDemand.yawImpulse(0.0, 1.0, 10.0, 0.05) > 0.0,
            "rate command requests a finite positive yaw impulse");
    }

    private static void groundSolverAppliesFrictionBoundedTrackDifferentialAtRest() {
        RotationMatrix level = new RotationMatrix().setToAngles(new Point3D(0.0, 0.0, 0.0));
        boolean motorizedTrackMode = SkidSteerDriveDemand.usesMotorizedRolling(
            false, true, false);
        require(motorizedTrackMode,
            "skid-driven rigid tracks must use commanded rolling demand instead of static hold");
        require(!SkidSteerDriveDemand.usesMotorizedRolling(false, true, true),
            "liquid support cannot be treated as powered track traction");
        List<GroundContactImpulseSolver.Contact> contacts = List.of(
            new GroundContactImpulseSolver.Contact(new Vec3d(-1.0, 0.0, 0.0),
                new Vec3d(0.0, 0.0, 1.0), 1.0, 1.0, 0.0, true, 0.0, motorizedTrackMode,
                0.0, Double.POSITIVE_INFINITY, 1.0),
            new GroundContactImpulseSolver.Contact(new Vec3d(1.0, 0.0, 0.0),
                new Vec3d(0.0, 0.0, 1.0), 1.0, 1.0, 0.0, true, 0.0, motorizedTrackMode,
                0.0, Double.POSITIVE_INFINITY, -1.0)
        );
        GroundContactImpulseSolver.Result solved = GroundContactImpulseSolver.solve(
            new Vec3d(0.0, -1.0, 0.0), Vec3d.ZERO, new Vec3d(10.0, 10.0, 10.0),
            level, Vec3d.ZERO, 100.0, 0.0, 0.0, 0.05, contacts);
        double[] normals = solved.normalImpulses();
        double[] longitudinal = solved.longitudinalImpulses();
        require(normals[0] > 0.0 && normals[1] > 0.0,
            "pivot receives positive live normal loads before drive traction");
        require(longitudinal[0] > 0.0 && longitudinal[1] < 0.0,
            "opposed track impulses create pivot traction from rest");
        near(0.0, longitudinal[0] + longitudinal[1], 1.0e-5,
            "stationary track pivot adds no longitudinal translation");
        require(solved.angularVelocityBody().y() > 0.0,
            "track impulses generate positive body yaw without a separate yaw injection");
        require(Math.abs(longitudinal[0]) <= normals[0] + 1.0e-6
            && Math.abs(longitudinal[1]) <= normals[1] + 1.0e-6,
            "each track impulse stays inside its live motive-friction cap");
    }

    private static void groundSolverConstrainsFourCornerLiquidHullWithoutTireGrip() {
        RotationMatrix level = new RotationMatrix().setToAngles(new Point3D(0.0, 0.0, 0.0));
        List<GroundContactImpulseSolver.Contact> liquidCorners = List.of(
            liquidCorner(-1.0, -2.0), liquidCorner(-1.0, 2.0),
            liquidCorner(1.0, -2.0), liquidCorner(1.0, 2.0)
        );
        GroundContactImpulseSolver.Result solved = GroundContactImpulseSolver.solve(
            new Vec3d(1.0, -1.0, 2.0), new Vec3d(0.25, 0.2, 0.15),
            new Vec3d(10.0, 12.0, 14.0), level, Vec3d.ZERO,
            100.0, 0.0, 0.0, 0.05, liquidCorners);
        double[] normals = solved.normalImpulses();
        double[] longitudinal = solved.longitudinalImpulses();
        double[] lateral = solved.lateralImpulses();
        double totalNormal = 0.0;
        for (int i = 0; i < normals.length; ++i) {
            require(normals[i] > 0.0,
                "each lower hull corner contributes live liquid normal support " + i);
            near(0.0, longitudinal[i], 1.0e-10,
                "liquid hull corner does not apply longitudinal tire force " + i);
            near(0.0, lateral[i], 1.0e-10,
                "liquid hull corner does not apply lateral tire force " + i);
            totalNormal += normals[i];
        }
        require(totalNormal > 0.0, "four-corner hull carries positive normal impulse");
        near(1.0, solved.velocityWorld().x(), 1.0e-10,
            "liquid support preserves horizontal x velocity without tire grip");
        near(2.0, solved.velocityWorld().z(), 1.0e-10,
            "liquid support preserves horizontal z velocity without tire grip");
        require(Math.abs(solved.angularVelocityBody().x()) < 0.25,
            "opposed hull-corner reactions resist BODY_X pitch rate");
        near(0.2, solved.angularVelocityBody().y(), 1.0e-10,
            "vertical liquid reactions do not inject yaw");
        require(Math.abs(solved.angularVelocityBody().z()) < 0.15,
            "opposed hull-corner reactions resist BODY_Z roll rate");
    }

    private static GroundContactImpulseSolver.Contact liquidCorner(double x, double z) {
        return new GroundContactImpulseSolver.Contact(
            new Vec3d(x, 0.0, z), new Vec3d(0.0, 0.0, 1.0),
            0.0, 0.0, 0.0, false, 0.0, false,
            0.0, Double.POSITIVE_INFINITY, 0.0);
    }

    private static void near(double expected, double actual, double tolerance, String label) {
        ++assertions;
        if (!Double.isFinite(actual) || Math.abs(expected - actual) > tolerance) {
            throw new AssertionError(label + ": expected " + expected + ", got " + actual);
        }
    }

    private static void require(boolean condition, String label) {
        ++assertions;
        if (!condition) throw new AssertionError(label);
    }
}
