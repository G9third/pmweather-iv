package com.g9third.pmweatheriv.physics;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartGeneric;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import minecrafttransportsimulator.jsondefs.JSONAnimationDefinition;
import minecrafttransportsimulator.jsondefs.JSONAnimationDefinition.AnimationComponentType;
import minecrafttransportsimulator.jsondefs.JSONPart;
import minecrafttransportsimulator.jsondefs.JSONPartDefinition;
import minecrafttransportsimulator.jsondefs.JSONVehicle;

/** Focused road-wheel support, friction, and native-animation regressions. */
public final class RoadSuspensionRegression {
    private static final double GRAVITY = 9.80665;
    private static int assertions;

    private RoadSuspensionRegression() {}

    public static void main(String[] args) throws Exception {
        calmBrakedHold();
        saturatedBrakingSlipsBeforeLift();
        torqueTransfersLoadAndUnloadsWheels();
        unloadedWheelHasNoGrip();
        serviceAndParkingBrakeCommandsRemainManual();
        grassRollingCoefficientRemainsAuthored();
        nativeAnimationEvidenceIsDeclaredAndFrameAware();
        nestedStationUsesVehicleFrameOnce();
        System.out.println("RoadSuspensionRegression: " + assertions
            + " assertions passed (solver and native-pose fixtures; no game simulation)");
    }

    private static void calmBrakedHold() {
        double mass = 2100.0;
        double dt = 0.05;
        GroundContactImpulseSolver.Result result = solve(mass,
            new Vec3d(0.0, -GRAVITY * dt, 0.0), Vec3d.ZERO, 1.0, dt, fourWheels(0.58, 0.78));
        near(0.0, result.velocityWorld().x(), 1.0e-8, "calm brake hold has no lateral drift");
        near(0.0, result.velocityWorld().y(), 1.0e-8, "calm brake hold balances weight");
        near(0.0, result.velocityWorld().z(), 1.0e-8, "calm brake hold has no forward drift");
        for (int i = 0; i < 4; ++i) {
            require(result.normalImpulses()[i] > 0.0, "calm hold retains wheel load " + i);
            near(0.0, result.longitudinalImpulses()[i], 1.0e-8,
                "calm hold does not invent brake impulse " + i);
        }
    }

    private static void saturatedBrakingSlipsBeforeLift() {
        double mass = 2100.0;
        double dt = 0.05;
        GroundContactImpulseSolver.Result result = solve(mass,
            new Vec3d(0.0, -GRAVITY * dt, 12.0), Vec3d.ZERO, 1.0, dt, fourWheels(0.58, 0.78));
        require(result.velocityWorld().z() > 1.0,
            "braking demand beyond tire capacity leaves forward slip");
        for (int i = 0; i < 4; ++i) {
            double normal = result.normalImpulses()[i];
            double longitudinal = Math.abs(result.longitudinalImpulses()[i]);
            require(normal > 0.0, "high brake demand does not pull a wheel off level ground " + i);
            require(longitudinal <= 0.58 * normal + 1.0e-5,
                "brake impulse remains within mu N before lift " + i);
            require(0.58 * normal - longitudinal < 1.0e-3,
                "excess braking reaches the tire limit before slip " + i);
        }
    }

    private static void torqueTransfersLoadAndUnloadsWheels() {
        double mass = 2100.0;
        double dt = 0.05;
        GroundContactImpulseSolver.Result result = solve(mass,
            new Vec3d(0.0, -GRAVITY * dt, 0.0), new Vec3d(0.0, 0.0, 3.0),
            0.0, dt, fourWheels(0.58, 0.78));
        double[] normals = result.normalImpulses();
        long unloaded = java.util.Arrays.stream(normals).filter(value -> value < 1.0e-5).count();
        long loaded = java.util.Arrays.stream(normals).filter(value -> value > 1.0e-4).count();
        require(unloaded > 0, "roll torque unloads the lifting side");
        require(loaded > 0 && loaded < normals.length, "remaining wheels carry the transferred load");
        require(result.angularVelocityBody().z() > 0.05,
            "physical support impulse leaves the accepted tip rate");
        for (double normal : normals) require(normal >= 0.0, "unilateral wheel load");
    }

    private static void unloadedWheelHasNoGrip() {
        List<GroundContactImpulseSolver.Contact> oneWheel = List.of(wheel(0.0, 0.0, 0.58, 0.78));
        GroundContactImpulseSolver.Result result = solve(1000.0,
            new Vec3d(1.0, 3.0, 8.0), Vec3d.ZERO, 1.0, 0.05, oneWheel);
        near(0.0, result.normalImpulses()[0], 1.0e-10, "separating wheel has zero normal load");
        near(0.0, result.longitudinalImpulses()[0], 1.0e-10, "zero-load wheel has no motive grip");
        near(0.0, result.lateralImpulses()[0], 1.0e-10, "zero-load wheel has no lateral grip");
        near(8.0, result.velocityWorld().z(), 1.0e-10, "zero-load wheel cannot brake airborne motion");
    }

    private static void serviceAndParkingBrakeCommandsRemainManual() {
        near(0.0, LandingGearSolver.landingGearBrakeCommand(0.0, false, 0.0), 0.0,
            "solver does not synthesize a brake command when both native controls are released");
        near(0.4, LandingGearSolver.landingGearBrakeCommand(0.4, false, 0.0), 0.0,
            "native service brake command passes through");
        near(1.0, LandingGearSolver.landingGearBrakeCommand(0.0, true, 0.0), 0.0,
            "native parking-brake toggle retains full command");
        near(0.6, LandingGearSolver.landingGearBrakeCommand(0.6, false, 0.2), 0.0,
            "service and partial parking commands retain the stronger manual input");
    }

    private static void grassRollingCoefficientRemainsAuthored() {
        TireContactMaterial.Grip grass = new TireContactMaterial.Grip(0.58, 0.78, "GRASS", false);
        near(0.08, TireContactMaterial.rollingResistance(0.03, grass), 0.0,
            "grass retains the existing rolling coefficient floor");
        near(0.12, TireContactMaterial.rollingResistance(0.12, grass), 0.0,
            "a larger authored tire rolling coefficient remains unchanged");
        near(0.58, grass.motive(), 0.0, "grass motive grip remains the evaluated pack value");
        near(0.78, grass.lateral(), 0.0, "grass lateral grip remains the evaluated pack value");
    }

    private static void nativeAnimationEvidenceIsDeclaredAndFrameAware() throws Exception {
        EntityVehicleF_Physics vehicle = vehicle();
        PartGroundDevice device = device(vehicle, new Vec3d(1.0, 0.0, 0.0));

        device.placementDefinition.animations = List.of(
            animation(AnimationComponentType.TRANSLATION, new Point3D(0.0, 1.0, 0.0), new Point3D()));
        require(RoadSuspensionModel.hasDeclaredVerticalStationMotion(device),
            "authored placement suspension translation owns vertical wheel travel");

        device.placementDefinition.animations = List.of(
            animation(AnimationComponentType.ROTATION, new Point3D(0.0, 1.0, 0.0), new Point3D(1.0, 0.0, 0.0)));
        require(!RoadSuspensionModel.hasDeclaredVerticalStationMotion(device),
            "planar steering around the wheel station remains fallback eligible");

        JSONPart owner = partDefinition();
        setField(device, "definition", owner);
        device.placementDefinition.animations = null;
        owner.generic.movementAnimations = List.of(
            animation(AnimationComponentType.TRANSLATION, new Point3D(1.0, 0.0, 0.0), new Point3D()));
        require(!RoadSuspensionModel.hasDeclaredVerticalStationMotion(device),
            "generic planar motion is not mistaken for vertical suspension");
        owner.generic.movementAnimations = List.of(
            animation(AnimationComponentType.TRANSLATION, new Point3D(0.0, 1.0, 0.0), new Point3D()));
        require(RoadSuspensionModel.hasDeclaredVerticalStationMotion(device),
            "generic vertical movement owns suspension travel");

        owner.generic.movementAnimations = List.of(
            animation(AnimationComponentType.ROTATION, new Point3D(1.0, 0.0, 0.0), new Point3D()));
        require(!RoadSuspensionModel.hasDeclaredVerticalStationMotion(device),
            "wheel spin around its own station is not vertical travel");

        owner.generic.movementAnimations = null;
        APart parent = genericPart(vehicle, new Vec3d(0.0, 0.0, 0.0));
        parent.placementDefinition.animations = List.of(
            animation(AnimationComponentType.TRANSLATION, new Point3D(0.0, 1.0, 0.0), new Point3D()));
        setField(device, "partOn", parent);
        require(RoadSuspensionModel.hasDeclaredVerticalStationMotion(device),
            "nested wheel inherits authored vertical motion from its parent");

        parent.placementDefinition.animations = null;
        setField(device, "partOn", null);
        setField(device, "definition", owner);
        require(!RoadSuspensionModel.hasDeclaredVerticalStationMotion(device),
            "stationary authored pose is eligible for fallback at rest");
        for (double y : new double[] {0.0, 0.00002, 0.00009, 0.05, -0.03}) {
            device.position.y = y;
            require(!RoadSuspensionModel.hasDeclaredVerticalStationMotion(device),
                "repeated fresh poses and fallback offsets do not latch authored motion");
        }
    }

    private static void nestedStationUsesVehicleFrameOnce() throws Exception {
        EntityVehicleF_Physics vehicle = vehicle();
        setField(vehicle, "position", new Point3D(120.0, 42.0, -80.0));
        setField(vehicle, "orientation", new RotationMatrix().setToAngles(new Point3D(17.0, 43.0, -11.0)));
        Vec3d station = new Vec3d(2.5, -0.4, 4.0);
        Vec3d world = vehiclePosition(vehicle).add(FlightMath.toWorld(vehicle.orientation, station));
        PartGroundDevice device = device(vehicle, world);
        setField(device, "partOn", genericPart(vehicle, Vec3d.ZERO));
        Vec3d resolved = RoadSuspensionModel.nativeCenterLocal(device);
        nearVec(station, resolved, 1.0e-9,
            "nested native world station resolves to root model coordinates exactly once");
    }

    private static GroundContactImpulseSolver.Result solve(double mass, Vec3d velocity, Vec3d omega,
                                                            double brake, double dt,
                                                            List<GroundContactImpulseSolver.Contact> contacts) {
        return GroundContactImpulseSolver.solve(velocity, omega,
            new Vec3d(8000.0, 20000.0, 12000.0), new RotationMatrix(), Vec3d.ZERO,
            mass, brake, 0.0, dt, contacts);
    }

    private static List<GroundContactImpulseSolver.Contact> fourWheels(double motive, double lateral) {
        List<GroundContactImpulseSolver.Contact> contacts = new ArrayList<>(4);
        for (double x : new double[] {-1.0, 1.0}) {
            for (double z : new double[] {-1.8, 1.8}) contacts.add(wheel(x, z, motive, lateral));
        }
        return contacts;
    }

    private static GroundContactImpulseSolver.Contact wheel(double x, double z, double motive, double lateral) {
        return new GroundContactImpulseSolver.Contact(new Vec3d(x, 0.0, z),
            new Vec3d(0.0, 0.0, 1.0), motive, lateral, 0.0, false,
            0.0, true, 0.0, Double.POSITIVE_INFINITY, Double.NaN);
    }

    private static EntityVehicleF_Physics vehicle() throws Exception {
        EntityVehicleF_Physics vehicle = allocate(EntityVehicleF_Physics.class);
        setField(vehicle, "uniqueUUID", UUID.fromString("00000000-0000-0000-0000-000000000101"));
        setField(vehicle, "definition", new JSONVehicle());
        setField(vehicle, "position", new Point3D());
        setField(vehicle, "orientation", new RotationMatrix());
        setField(vehicle, "allParts", new ArrayList<APart>());
        return vehicle;
    }

    private static PartGroundDevice device(EntityVehicleF_Physics vehicle, Vec3d worldPosition) throws Exception {
        PartGroundDevice device = allocate(PartGroundDevice.class);
        setField(device, "vehicleOn", vehicle);
        setField(device, "partOn", null);
        setField(device, "position", new Point3D(worldPosition.x(), worldPosition.y(), worldPosition.z()));
        setField(device, "orientation", new RotationMatrix());
        setField(device, "definition", partDefinition());
        setField(device, "placementDefinition", new JSONPartDefinition());
        return device;
    }

    private static APart genericPart(EntityVehicleF_Physics vehicle, Vec3d worldPosition) throws Exception {
        APart part = allocate(PartGeneric.class);
        setField(part, "vehicleOn", vehicle);
        setField(part, "partOn", null);
        setField(part, "position", new Point3D(worldPosition.x(), worldPosition.y(), worldPosition.z()));
        setField(part, "orientation", new RotationMatrix());
        setField(part, "definition", partDefinition());
        setField(part, "placementDefinition", new JSONPartDefinition());
        return part;
    }

    private static JSONPart partDefinition() {
        JSONPart part = new JSONPart();
        part.generic = new JSONPart.JSONPartGeneric();
        return part;
    }

    private static JSONAnimationDefinition animation(AnimationComponentType type, Point3D axis,
                                                       Point3D center) {
        JSONAnimationDefinition animation = new JSONAnimationDefinition();
        animation.animationType = type;
        animation.axis = axis;
        animation.centerPoint = center;
        return animation;
    }

    private static Vec3d vehiclePosition(EntityVehicleF_Physics vehicle) {
        return new Vec3d(vehicle.position.x, vehicle.position.y, vehicle.position.z);
    }

    @SuppressWarnings("unchecked")
    private static <T> T allocate(Class<T> type) throws Exception {
        var field = sun.misc.Unsafe.class.getDeclaredField("theUnsafe");
        field.setAccessible(true);
        return (T) ((sun.misc.Unsafe) field.get(null)).allocateInstance(type);
    }

    private static void setField(Object object, String name, Object value) throws Exception {
        for (Class<?> owner = object.getClass(); owner != null; owner = owner.getSuperclass()) {
            try {
                var field = owner.getDeclaredField(name);
                field.setAccessible(true);
                field.set(object, value);
                return;
            } catch (NoSuchFieldException absent) {
                // Keep searching the native base-class hierarchy.
            }
        }
        throw new NoSuchFieldException(name);
    }

    private static void nearVec(Vec3d expected, Vec3d actual, double tolerance, String label) {
        near(expected.subtract(actual).length(), 0.0, tolerance, label);
    }

    private static void near(double expected, double actual, double tolerance, String label) {
        require(Double.isFinite(expected) && Double.isFinite(actual)
            && Math.abs(expected - actual) <= tolerance,
            label + ": expected " + expected + ", got " + actual);
    }

    private static void require(boolean condition, String label) {
        ++assertions;
        if (!condition) throw new AssertionError(label);
    }
}
