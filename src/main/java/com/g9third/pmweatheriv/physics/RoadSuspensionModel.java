package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.mixin.PartGroundDeviceFakeAccessor;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import minecrafttransportsimulator.entities.instances.PartGroundDeviceFake;
import minecrafttransportsimulator.jsondefs.JSONAnimationDefinition;
import org.joml.Vector3d;

/** Generic series tire/suspension support for managed road vehicles. */
public final class RoadSuspensionModel {
    private static final double GRAVITY = 9.80665;
    private static final double WHEEL_SAG_FRACTION = 0.25;
    private static final double MIN_WHEEL_SAG_METERS = 0.04;
    private static final double MAX_WHEEL_SAG_METERS = 0.14;
    private static final double TREAD_SAG_FRACTION = 0.12;
    private static final double MIN_TREAD_SAG_METERS = 0.02;
    private static final double MAX_TREAD_SAG_METERS = 0.06;
    private static final double TIRE_DAMPING_RATIO = 0.75;
    private static final double SUSPENSION_DAMPING_RATIO = 0.65;
    private static final double MOTION_EPSILON = 1.0E-4;

    private RoadSuspensionModel() {}

    public record Parameters(double cornerMassKg, double sagMeters, double tireDeflectionMeters,
                             double totalDeflectionMeters, double stiffnessNpm,
                             double dampingNsPm, double maxTotalCompressionMeters,
                             double axialSpringStiffnessNpm, double axialSpringDampingNsPm,
                             double verticalProjection) {
        public boolean finite() {
            return Double.isFinite(cornerMassKg) && cornerMassKg > 0.0
                && Double.isFinite(sagMeters) && sagMeters > 0.0
                && Double.isFinite(totalDeflectionMeters) && totalDeflectionMeters > 0.0
                && Double.isFinite(stiffnessNpm) && stiffnessNpm > 0.0
                && Double.isFinite(dampingNsPm) && dampingNsPm >= 0.0
                && Double.isFinite(maxTotalCompressionMeters) && maxTotalCompressionMeters > 0.0
                && Double.isFinite(axialSpringStiffnessNpm) && axialSpringStiffnessNpm > 0.0
                && Double.isFinite(axialSpringDampingNsPm) && axialSpringDampingNsPm >= 0.0
                && Double.isFinite(verticalProjection) && verticalProjection >= 0.25;
        }
    }

    /** Resolve IV's hidden tread sample to the authored device that owns its assembly. */
    public static PartGroundDevice masterPart(PartGroundDevice device) {
        if (device instanceof PartGroundDeviceFake fake) {
            PartGroundDevice master = ((PartGroundDeviceFakeAccessor) (Object) fake)
                .pmweatherIv$getMasterPart();
            if (master != null) return master;
        }
        return device;
    }

    /**
     * Fallback applies only to intact, active road wheels/tracks on a managed
     * non-aircraft. A declared native vertical motion or detected changing
     * authored station owns that movement instead.
     */
    public static boolean supportsFallback(EntityVehicleF_Physics vehicle, PartGroundDevice device) {
        if (vehicle == null || device == null || vehicle.definition == null
            || vehicle.definition.motorized == null || vehicle.definition.motorized.isAircraft
            || !TireContactMaterial.activeSupport(device) || device.definition == null
            || device.definition.ground == null || device.definition.ground.canFloat || device.flatVar.isActive
            || (!device.definition.ground.isWheel && !device.definition.ground.isTread)) return false;
        PartGroundDevice master = masterPart(device);
        if (master == null || master.flatVar.isActive || master.definition == null
            || master.definition.ground == null || master.definition.ground.canFloat
            || (!master.definition.ground.isWheel && !master.definition.ground.isTread)) return false;
        // The per-part native update caches authored-motion eligibility once per
        // owner tick. The physics substep must not traverse animation trees or
        // allocate vectors for every support sample.
        return device instanceof RoadSuspensionPartAccess access
            && !access.pmweatherIv$hasAuthoredVerticalMotion()
            && (!(master instanceof RoadSuspensionPartAccess masterAccess)
                || !masterAccess.pmweatherIv$hasAuthoredVerticalMotion());
    }

    /** Called once from the native part update to cache authored support motion. */
    public static boolean hasDeclaredVerticalStationMotion(PartGroundDevice device) {
        if (device == null || device.vehicleOn == null) return true;
        Vec3d up = FlightMath.toWorld(device.vehicleOn.orientation, new Vec3d(0.0, 1.0, 0.0));
        return declaresVerticalStationMotion(device, new Vector3d(up.x(), up.y(), up.z()));
    }

    public static double sagMeters(PartGroundDevice device) {
        if (device == null || device.definition == null || device.definition.ground == null) return 0.0;
        double radius = Math.max(0.0, device.getHeight()) * 0.5;
        if (device.definition.ground.isWheel) {
            return Vec3d.clamp(radius * WHEEL_SAG_FRACTION,
                MIN_WHEEL_SAG_METERS, MAX_WHEEL_SAG_METERS);
        }
        if (device.definition.ground.isTread) {
            return Vec3d.clamp(radius * TREAD_SAG_FRACTION,
                MIN_TREAD_SAG_METERS, MAX_TREAD_SAG_METERS);
        }
        return 0.0;
    }

    public static Parameters parameters(PartGroundDevice device, double massKg, int installedSupports) {
        return parameters(device, massKg, installedSupports, 1.0);
    }

    public static Parameters parameters(PartGroundDevice device, double massKg, int installedSupports,
                                        double verticalProjection) {
        if (device == null || !(massKg > 0.0) || !Double.isFinite(massKg)) return null;
        double count = Math.max(1, installedSupports);
        double cornerMass = massKg / count;
        return parameters(device, cornerMass, verticalProjection);
    }

    /** Per-contact load share; fake tread samples retain their native distribution. */
    public static Parameters parameters(PartGroundDevice device, double cornerMassKg) {
        return parameters(device, cornerMassKg, 1.0);
    }

    public static Parameters parameters(PartGroundDevice device, double cornerMassKg,
                                        double verticalProjection) {
        if (device == null || !(cornerMassKg > 0.0) || !Double.isFinite(cornerMassKg)) return null;
        double projection = Double.isFinite(verticalProjection)
            ? Vec3d.clamp(verticalProjection, 0.25, 1.0) : 1.0;
        double sag = sagMeters(device);
        double tireDeflection = device.definition.ground.isWheel
            ? TireNormalCompliance.nominalDeflection(device) : 0.0;
        double totalDeflection = sag + tireDeflection;
        if (!(sag > 0.0) || !(totalDeflection > 0.0)) return null;

        double kSuspension = cornerMassKg * GRAVITY / sag;
        double cSuspension = 2.0 * SUSPENSION_DAMPING_RATIO
            * Math.sqrt(kSuspension * cornerMassKg);
        double kSuspensionVertical = kSuspension / (projection * projection);
        double cSuspensionVertical = cSuspension / (projection * projection);
        double stiffness;
        double damping;
        if (tireDeflection > 0.0) {
            double kTire = cornerMassKg * GRAVITY / tireDeflection;
            double cTire = 2.0 * TIRE_DAMPING_RATIO * Math.sqrt(kTire * cornerMassKg);
            double sum = kTire + kSuspensionVertical;
            stiffness = cornerMassKg * GRAVITY / (tireDeflection + sag * projection * projection);
            damping = (cTire * kSuspensionVertical * kSuspensionVertical
                + cSuspensionVertical * kTire * kTire) / (sum * sum);
        } else {
            stiffness = kSuspensionVertical;
            damping = cSuspensionVertical;
        }
        double maximumCompression = 3.0 * tireDeflection + 3.0 * sag * projection;
        Parameters parameters = new Parameters(cornerMassKg, sag, tireDeflection,
            tireDeflection + sag * projection * projection, stiffness, damping, maximumCompression,
            kSuspension, cSuspension, projection);
        return parameters.finite() ? parameters : null;
    }

    /** Implicit Kelvin-Voigt spring update from the accepted shared normal load. */
    public static double nextTravelMeters(double previousMeters, double totalNormalForceNewtons,
                                          double groupMassKg, double sagMeters, double dtSeconds) {
        return nextTravelMeters(previousMeters, totalNormalForceNewtons, groupMassKg,
            sagMeters, dtSeconds, 1.0);
    }

    public static double nextTravelMeters(double previousMeters, double totalNormalForceNewtons,
                                          double groupMassKg, double sagMeters, double dtSeconds,
                                          double verticalProjection) {
        if (!(groupMassKg > 0.0) || !(sagMeters > 0.0) || !(dtSeconds > 0.0)
            || !Double.isFinite(previousMeters) || !Double.isFinite(totalNormalForceNewtons)) {
            return 0.0;
        }
        double stiffness = groupMassKg * GRAVITY / sagMeters;
        double damping = 2.0 * SUSPENSION_DAMPING_RATIO * Math.sqrt(stiffness * groupMassKg);
        double projection = Double.isFinite(verticalProjection)
            ? Vec3d.clamp(verticalProjection, 0.0, 1.0) : 0.0;
        double denominator = damping + dtSeconds * stiffness;
        if (!(denominator > 0.0) || !Double.isFinite(denominator)) return 0.0;
        double next = (damping * previousMeters
            + dtSeconds * (Math.max(0.0, totalNormalForceNewtons) * projection
                - stiffness * sagMeters))
            / denominator;
        return Double.isFinite(next) ? Vec3d.clamp(next, -sagMeters, 2.0 * sagMeters) : 0.0;
    }

    /**
     * Restores a native wheel center in the current vehicle/model frame. Nested
     * parts use their resolved world position, since their localOffset frame is
     * parent-relative during native animation updates.
     */
    public static Vec3d baseCenterLocal(PartGroundDevice device) {
        Vec3d center = nativeCenterLocal(device);
        if (center == null) return null;
        double applied = device instanceof RoadSuspensionPartAccess access
            ? access.pmweatherIv$appliedSuspensionOffset() : 0.0;
        return new Vec3d(center.x(), center.y() - (Double.isFinite(applied) ? applied : 0.0), center.z());
    }

    /** Native, unshifted current station in vehicle model coordinates. */
    public static Vec3d nativeCenterLocal(PartGroundDevice device) {
        if (device == null || device.vehicleOn == null) return null;
        if (device.partOn != null && finite(device.position) && finite(device.vehicleOn.position)) {
            Point3D relativeWorld = new Point3D(
                device.position.x - device.vehicleOn.position.x,
                device.position.y - device.vehicleOn.position.y,
                device.position.z - device.vehicleOn.position.z
            );
            Vec3d center = FlightMath.toLocal(device.vehicleOn.orientation,
                new Vec3d(relativeWorld.x, relativeWorld.y, relativeWorld.z));
            return center.isFinite() ? center : null;
        }
        Point3D nativeOffset = device.localOffset != null && finite(device.localOffset)
            ? device.localOffset : device.wheelbasePoint;
        if (!finite(nativeOffset)) return null;
        return new Vec3d(nativeOffset.x, nativeOffset.y, nativeOffset.z);
    }

    /** Returns the wheel/tread's physical support point from a native base center. */
    public static Vec3d supportPointLocal(PartGroundDevice device, Vec3d center,
                                         minecrafttransportsimulator.baseclasses.RotationMatrix physicalOrientation) {
        if (device == null || center == null || !center.isFinite() || physicalOrientation == null) return null;
        double height = device.getHeight();
        double halfHeight = Double.isFinite(height) ? Math.max(0.0, height) * 0.5 : 0.0;
        if (!device.definition.ground.isWheel)
            return new Vec3d(center.x(), center.y() - halfHeight, center.z());
        Point3D axleDirection = new Point3D(1.0, 0.0, 0.0).rotate(device.orientation);
        Vec3d axle = FlightMath.toLocal(device.vehicleOn.orientation,
            new Vec3d(axleDirection.x, axleDirection.y, axleDirection.z));
        double width = device.getWidth();
        double halfWidth = Double.isFinite(width) ? Math.max(0.0, width) * 0.5 : 0.0;
        return center.add(ModelCoordinates.tireSupportOffset(physicalOrientation, axle, halfHeight, halfWidth));
    }

    public static double complianceGapMeters(double actualGapMeters, double travelMeters,
                                            double verticalProjection, double sagMeters) {
        if (!Double.isFinite(actualGapMeters) || !Double.isFinite(travelMeters)
            || !Double.isFinite(verticalProjection) || !Double.isFinite(sagMeters)) return actualGapMeters;
        return actualGapMeters - (travelMeters + sagMeters) * verticalProjection;
    }

    public static Vec3d worldOffset(PartGroundDevice device, double travelMeters) {
        if (device == null || device.vehicleOn == null || !Double.isFinite(travelMeters)) return Vec3d.ZERO;
        return FlightMath.toWorld(device.vehicleOn.orientation, new Vec3d(0.0, travelMeters, 0.0));
    }

    public static Vec3d parentOffsetFromWorld(PartGroundDevice device, Vec3d worldOffset) {
        if (device == null || device.vehicleOn == null || worldOffset == null || !worldOffset.isFinite())
            return Vec3d.ZERO;
        return device.partOn == null
            ? FlightMath.toLocal(device.vehicleOn.orientation, worldOffset)
            : FlightMath.toLocal(device.partOn.orientation, worldOffset);
    }

    public static Vec3d vehicleOffsetFromWorld(PartGroundDevice device, Vec3d worldOffset) {
        if (device == null || device.vehicleOn == null || worldOffset == null || !worldOffset.isFinite())
            return Vec3d.ZERO;
        return FlightMath.toLocal(device.vehicleOn.orientation, worldOffset);
    }

    public static double maxPenetrationMeters(PartGroundDevice device, double skinMeters) {
        double sag = sagMeters(device);
        double tire = device != null && device.definition.ground.isWheel
            ? TireNormalCompliance.nominalDeflection(device) : 0.0;
        return Math.max(0.005, 3.0 * tire + 2.0 * sag - Math.max(0.0, skinMeters));
    }

    public static double roadPoseCorrectionMeters(double baseGapMeters, PartGroundDevice device,
                                                   double verticalProjection, double skinMeters) {
        if (!Double.isFinite(baseGapMeters) || device == null) return 0.0;
        double projection = Double.isFinite(verticalProjection)
            ? Vec3d.clamp(verticalProjection, 0.25, 1.0) : 1.0;
        double tire = device.definition.ground.isWheel ? TireNormalCompliance.nominalDeflection(device) : 0.0;
        double allowedBasePenetration = Math.max(0.0,
            3.0 * tire + 2.0 * sagMeters(device) * projection - Math.max(0.0, skinMeters));
        return Math.max(0.0, -baseGapMeters - allowedBasePenetration);
    }

    private static boolean declaresVerticalStationMotion(PartGroundDevice device, Vector3d upWorld) {
        if (device == null || device.vehicleOn == null) return true;
        for (APart ancestor = device; ancestor != null; ancestor = ancestor.partOn) {
            var placement = ancestor.placementDefinition;
            if (placement == null) continue;
            if (verticalMotionIn(placement.animations, ancestor, device, upWorld)
                || verticalMotionIn(placement.activeAnimations, ancestor, device, upWorld)) return true;
        }
        return false;
    }

    private static boolean verticalMotionIn(java.util.List<JSONAnimationDefinition> animations,
                                            APart animatedPart, PartGroundDevice station,
                                            Vector3d upWorld) {
        if (animations == null) return false;
        for (JSONAnimationDefinition animation : animations) {
            if (animation == null || animation.animationType == null) continue;
            // Placement movements run in the animation owner's coordinate
            // frame: the vehicle for a top-level part, or partOn for a nested
            // part. The part's current orientation already includes its own
            // animation and is not the input frame for these axes/pivots.
            APart parent = animatedPart.partOn;
            var frameOrientation = parent == null
                ? animatedPart.vehicleOn.orientation : parent.orientation;
            Point3D framePosition = parent == null
                ? animatedPart.vehicleOn.position : parent.position;
            if (frameOrientation == null || !finite(framePosition)) return true;
            if (animation.animationType == JSONAnimationDefinition.AnimationComponentType.TRANSLATION) {
                if (animation.axis == null) return true;
                Vector3d axisWorld = rotate(frameOrientation, animation.axis);
                if (!finite(axisWorld) || Math.abs(axisWorld.dot(upWorld)) > MOTION_EPSILON) return true;
            } else if (animation.animationType == JSONAnimationDefinition.AnimationComponentType.ROTATION) {
                if (animation.axis == null || animation.centerPoint == null
                    || !finite(animatedPart.position) || !finite(station.position)) return true;
                // Do not transform through animatedPart.orientation: that
                // double-applies its own steering/spin rotation and can turn a
                // planar rotation into a false vertical-motion declaration.
                Vector3d axisWorld = rotate(frameOrientation, animation.axis);
                if (!finite(axisWorld) || axisWorld.lengthSquared() <= 1.0E-12) return true;
                axisWorld.normalize();
                Vector3d pivotWorld = rotate(frameOrientation, animation.centerPoint)
                    .add(framePosition.x, framePosition.y, framePosition.z);
                Vector3d stationWorld = new Vector3d(station.position.x, station.position.y, station.position.z);
                Vector3d velocityPerRadian = axisWorld.cross(stationWorld.sub(pivotWorld));
                if (Math.abs(velocityPerRadian.dot(upWorld)) > MOTION_EPSILON) return true;
            }
        }
        return false;
    }

    private static Vector3d rotate(minecrafttransportsimulator.baseclasses.RotationMatrix matrix,
                                   Point3D point) {
        if (matrix == null || !finite(point)) return new Vector3d(Double.NaN, Double.NaN, Double.NaN);
        Point3D rotated = point.copy().rotate(matrix);
        return new Vector3d(rotated.x, rotated.y, rotated.z);
    }

    private static boolean finite(Point3D point) {
        return point != null && Double.isFinite(point.x) && Double.isFinite(point.y) && Double.isFinite(point.z);
    }

    private static boolean finite(Vector3d value) {
        return value != null && Double.isFinite(value.x) && Double.isFinite(value.y) && Double.isFinite(value.z);
    }
}
