package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import com.g9third.pmweatheriv.mixin.VehicleGroundDeviceCollectionAccessor;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.baseclasses.VehicleGroundDeviceBox;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import net.minecraft.core.BlockPos;
import static com.g9third.pmweatheriv.physics.FlightMath.EPSILON;
import static com.g9third.pmweatheriv.physics.FlightMath.LANDING_GEAR_POSITION_SLOP_METERS;
import static com.g9third.pmweatheriv.physics.FlightMath.LANDING_GEAR_POSITION_STABILIZATION_RATE_PER_SECOND;
import static com.g9third.pmweatheriv.physics.FlightMath.LANDING_GEAR_ROLLING_RESISTANCE_DEFORMATION_METERS;
import static com.g9third.pmweatheriv.physics.FlightMath.LANDING_GEAR_MIN_ROLLING_RESISTANCE_COEFFICIENT;
import static com.g9third.pmweatheriv.physics.FlightMath.LANDING_GEAR_MAX_ROLLING_RESISTANCE_COEFFICIENT;
import static com.g9third.pmweatheriv.physics.FlightMath.MC_TICK_SECONDS;
import static com.g9third.pmweatheriv.physics.FlightMath.point;
import static com.g9third.pmweatheriv.physics.FlightMath.toLocal;
import static com.g9third.pmweatheriv.physics.FlightMath.toWorld;

/** Builds live support constraints and solves shared tire contact impulses. */
public final class LandingGearSolver {
    private LandingGearSolver() {}

    /**
     * True when aircraft landing-gear physics is owned by PMWeather-IV/Sable.
     * IV remains the ground-device/contact sensor and part/animation system.
     * On the server Sable owns physical collision and pose when its compound body
     * is active; IV tire-force, depenetration, and direct ground-angle solvers are
     * always bypassed for the custom aircraft solver.
     */
    public static boolean shouldReplaceIvGroundOperations(
        EntityVehicleF_Physics vehicle
    ) {
        return vehicle != null
            && (vehicle.world.isClient()
                ? com.g9third.pmweatheriv.network.AircraftStateNetwork.managed(vehicle)
                : PMWeatherIVConfig.get().enabled())
            && com.g9third.pmweatheriv.sable.SableVehicleManager.supportsVehicle(vehicle)
            && vehicle.groundDeviceCollective != null;
    }

    /** Retained name for the existing pitch/roll mixin and diagnostics. */
    public static boolean shouldReplaceIvGroundAngularCorrection(
        EntityVehicleF_Physics vehicle
    ) {
        return shouldReplaceIvGroundOperations(vehicle) || GroundVehicleWind.replacesNativeTires(vehicle);
    }

    /**
     * Keeps IV's wheel-animation inputs alive after performGroundOperations is
     * cancelled.  PartGroundDevice derives ground_rotation from groundVelocity
     * and goingInReverse; neither value is allowed to become a physics force.
     */
    public static void maintainIvGroundAnimationState(EntityVehicleF_Physics vehicle) {
        if (vehicle == null) {
            return;
        }
        double horizontalSpeed = Math.hypot(vehicle.motion.x, vehicle.motion.z);
        vehicle.groundVelocity = Double.isFinite(horizontalSpeed) ? horizontalSpeed : 0.0;
        if (vehicle.groundVelocity > 1.0E-7) {
            Vec3d forwardWorld = toWorld(vehicle, new Vec3d(0.0, 0.0, 1.0));
            Vec3d horizontalForward = new Vec3d(forwardWorld.x(), 0.0, forwardWorld.z());
            if (horizontalForward.lengthSquared() > 1.0E-12) {
                horizontalForward = horizontalForward.normalized();
                double signedForwardMotion = new Vec3d(
                    vehicle.motion.x, 0.0, vehicle.motion.z
                ).dot(horizontalForward);
                vehicle.goingInReverse = signedForwardMotion < 0.0;
            }
        }
        // IV's old ground-operation stage normally updates this; the Sable tire
        // solver owns actual yaw now, so do not leave a stale authored turn force.
        vehicle.turningForce = 0.0;
    }

    /**
     * Solves vehicle wheel/gear contacts in persistent Sable momentum.
     *
     * <p>IV contributes live authored wheel stations, friction coefficients,
     * steering and brake state. Sable supplies the current physical pose and
     * exact terrain contact. Each exact wheel receives unilateral normal support
     * at its real lever arm. Normal and tangential behavior are coupled at each wheel from
     * each contact-point velocity: lateral no-slip demand and commanded braking
     * share the wheel's Coulomb friction ellipse, while pneumatic rolling
     * resistance spends the remaining shared grip. Rigid support receives passive two-axis friction. All wheel
     * tangential unknowns are advanced simultaneously by a projected solver, so
     * there is no first-wheel/contact-order yaw injection.</p>
     *
     * <p>Every resulting impulse acts at the real wheel point. Translation and
     * pitch/roll/yaw moments therefore follow directly from rigid-body r x J.
     * There is no separate heading target, yaw-rate controller, grounded drag
     * multiplier, synthetic suspension spring/damper, release timer or ground/air
     * handoff. As aerodynamic lift unloads a tire its normal impulse, grip, brake
     * capacity and rolling resistance all fall continuously to zero.</p>
     */
    public static LandingGearConstraintResult solveLandingGearConstraints(
        EntityVehicleF_Physics vehicle,
        Vec3d linearVelocityWorld,
        Vec3d angularVelocityBody,
        double mass,
        Vec3d inertia
    ) {
        return solveLandingGearConstraints(
            vehicle,
            linearVelocityWorld,
            angularVelocityBody,
            mass,
            inertia,
            vehicle == null ? null : vehicle.orientation
        );
    }

    /**
     * Physical-orientation overload used by the authoritative Sable body. IV's
     * entity pose is a one-tick mirror while the current Rapier quaternion is
     * already available, so every point velocity/Jacobian uses that exact pose.
     */
    public static LandingGearConstraintResult solveLandingGearConstraints(
        EntityVehicleF_Physics vehicle,
        Vec3d linearVelocityWorld,
        Vec3d angularVelocityBody,
        double mass,
        Vec3d inertia,
        RotationMatrix physicalOrientation
    ) {
        return solveLandingGearConstraints(
            vehicle, linearVelocityWorld, angularVelocityBody, mass, inertia,
            physicalOrientation, MC_TICK_SECONDS
        );
    }

    public static LandingGearConstraintResult solveLandingGearConstraints(
        EntityVehicleF_Physics vehicle,
        Vec3d linearVelocityWorld,
        Vec3d angularVelocityBody,
        double mass,
        Vec3d inertia,
        Vec3d centerOfMassLocal
    ) {
        return solveLandingGearConstraints(
            vehicle, linearVelocityWorld, angularVelocityBody, mass, inertia,
            vehicle != null ? vehicle.orientation : new RotationMatrix(),
            MC_TICK_SECONDS, null, centerOfMassLocal
        );
    }

    /**
     * Sable-substep overload.  The velocity constraint itself has no artificial
     * cadence dependency; Baumgarte penetration stabilization is expressed as
     * a continuous per-second rate and evaluated with the actual substep dt.
     * This keeps seating response invariant when Sable uses any configured
     * number of substeps per Minecraft tick.
     */
    public static LandingGearConstraintResult solveLandingGearConstraints(
        EntityVehicleF_Physics vehicle,
        Vec3d linearVelocityWorld,
        Vec3d angularVelocityBody,
        double mass,
        Vec3d inertia,
        RotationMatrix physicalOrientation,
        double timeStepSeconds
    ) {
        return solveLandingGearConstraints(
            vehicle, linearVelocityWorld, angularVelocityBody, mass, inertia,
            physicalOrientation, timeStepSeconds, null, Vec3d.ZERO
        );
    }

    /**
     * Authoritative Sable-contact overload. When {@code physicalContacts} is
     * non-null, the supplied contacts are the physical contact authority and
     * IV's 20 Hz ground-device sensor flags are not consulted. IV still owns
     * the authored gear stations, friction, steering and brake properties.
     */
    public static LandingGearConstraintResult solveLandingGearConstraints(
        EntityVehicleF_Physics vehicle,
        Vec3d linearVelocityWorld,
        Vec3d angularVelocityBody,
        double mass,
        Vec3d inertia,
        RotationMatrix physicalOrientation,
        double timeStepSeconds,
        List<LandingGearPhysicalContact> physicalContacts
    ) {
        return solveLandingGearConstraints(
            vehicle, linearVelocityWorld, angularVelocityBody, mass, inertia,
            physicalOrientation, timeStepSeconds, physicalContacts, Vec3d.ZERO
        );
    }

    public static LandingGearConstraintResult solveLandingGearConstraints(
        EntityVehicleF_Physics vehicle,
        Vec3d linearVelocityWorld,
        Vec3d angularVelocityBody,
        double mass,
        Vec3d inertia,
        RotationMatrix physicalOrientation,
        double timeStepSeconds,
        List<LandingGearPhysicalContact> physicalContacts,
        Vec3d centerOfMassLocal
    ) {
        return solveLandingGearConstraints(vehicle, linearVelocityWorld, angularVelocityBody, mass, inertia,
            physicalOrientation, timeStepSeconds, physicalContacts, centerOfMassLocal, null);
    }

    public static LandingGearConstraintResult solveLandingGearConstraints(
        EntityVehicleF_Physics vehicle, Vec3d linearVelocityWorld, Vec3d angularVelocityBody,
        double mass, Vec3d inertia, RotationMatrix physicalOrientation, double timeStepSeconds,
        List<LandingGearPhysicalContact> physicalContacts, Vec3d centerOfMassLocal,
        RoadVehiclePhysics.DriveSnapshot drive
    ) {
        Vec3d safeLinear = linearVelocityWorld == null || !linearVelocityWorld.isFinite()
            ? Vec3d.ZERO
            : linearVelocityWorld;
        Vec3d safeAngular = angularVelocityBody == null || !angularVelocityBody.isFinite()
            ? Vec3d.ZERO
            : angularVelocityBody;
        double safeMass = Math.max(1.0, Double.isFinite(mass) ? mass : 1.0);
        Vec3d safeInertia = inertia == null || !inertia.isFinite()
            ? new Vec3d(1.0, 1.0, 1.0)
            : new Vec3d(
                Math.max(EPSILON, inertia.x()),
                Math.max(EPSILON, inertia.y()),
                Math.max(EPSILON, inertia.z())
            );
        RotationMatrix orientation = physicalOrientation != null
            ? physicalOrientation
            : vehicle != null ? vehicle.orientation : new RotationMatrix();
        double constraintStepSeconds = Double.isFinite(timeStepSeconds) && timeStepSeconds > 1.0E-6
            ? timeStepSeconds
            : MC_TICK_SECONDS;
        Vec3d safeCenterOfMassLocal = centerOfMassLocal == null || !centerOfMassLocal.isFinite()
            ? Vec3d.ZERO : centerOfMassLocal;

        // Totaling the chassis does not remove its surviving ground devices.
        // Those devices are excluded from the Rapier BODY compound, so their
        // live terrain constraints must continue until removal/retraction.
        // Disable stale motor demand while retaining passive rolling, friction,
        // compliance and the installed tire's flat/damage state.
        if (vehicle != null && vehicle.outOfHealth) {
            drive = null;
        }

        List<LandingGearContact> contacts = physicalContacts == null
            ? groundedWheelConstraintContacts(vehicle)
            : physicalLandingGearConstraintContacts(physicalContacts);
        if (contacts.isEmpty()) {
            if (vehicle != null) {
                vehicle.slipping = false;
            }
            return new LandingGearConstraintResult(
                safeLinear, safeAngular, Vec3d.ZERO, Vec3d.ZERO, 0, 0, 0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, List.of()
            );
        }

        Vec3d solvedLinear = safeLinear;
        Vec3d solvedAngular = safeAngular;
        double maxClosingBefore = maximumWheelClosingSpeed(
            orientation, contacts, solvedLinear, solvedAngular, safeCenterOfMassLocal
        );
        double maxLateralSlipBefore = maximumWheelLateralSlipSpeed(
            vehicle, orientation, contacts, solvedLinear, solvedAngular, safeCenterOfMassLocal
        );
        double maximumPenetration = 0.0;
        int exactCollisionCount = 0;
        for (LandingGearContact contact : contacts) {
            maximumPenetration = Math.max(maximumPenetration, contact.penetrationDepthMeters());
            if (contact.exactCollision()) {
                ++exactCollisionCount;
            }
        }
        int proximityContactCount = contacts.size() - exactCollisionCount;

        // Use the same coupled 3D kernel as road tires. Tangential forces and
        // normal load transfer are solved together; rolling loss spends the same
        // ellipse as commanded braking, never an additional post-solve budget.
        java.util.Map<Vec3d, BlockPos> blocks = new java.util.HashMap<>();
        if (physicalContacts != null) for (LandingGearPhysicalContact contact : physicalContacts) {
            if (contact != null && contact.pointLocal() != null && contact.supportBlock() != null)
                blocks.put(contact.pointLocal(), contact.supportBlock());
        }
        List<GroundContactImpulseSolver.Contact> constraints = new ArrayList<>();
        List<Integer> indices = new ArrayList<>();
        TireContactMaterial.Grip[] grips = new TireContactMaterial.Grip[contacts.size()];
        TireNormalCompliance.Response[] responses = new TireNormalCompliance.Response[contacts.size()];
        double[] rollingCoefficients = new double[contacts.size()];
        double[] driveLeverX = new double[contacts.size()];
        double[] driveSideX = new double[contacts.size()];
        boolean[] drivenContacts = new boolean[contacts.size()];
        double brake = vehicle != null && vehicle.outOfHealth ? 0.0 : landingGearBrakeCommand(vehicle);
        if (drive != null && vehicle != null) brake *= Double.isFinite(vehicle.brakingFactorVar.currentValue)
            ? Math.max(0,vehicle.brakingFactorVar.currentValue) : 0;
        int installedSupports=TireNormalCompliance.installedSupportCount(vehicle);
        double roadVerticalProjection = Math.max(0.0, toWorld(orientation, new Vec3d(0.0, 1.0, 0.0)).y());
        for (int index=0; index<contacts.size(); ++index) {
            LandingGearContact contact = contacts.get(index);
            if (!contact.exactCollision() || contact.device() == null
                || !LiquidSupportModel.hasTireTraction(isLiquidSupport(contact))) continue;
            driveLeverX[index] = contact.pointLocal().subtract(safeCenterOfMassLocal).x();
            driveSideX[index] = contact.pointLocal().x();
            drivenContacts[index] = drive != null && (drive.skidSteer() || contact.device().drivenLastTick);
        }
        double skidYawImpulse = drive != null && drive.skidSteer()
            ? SkidSteerDriveDemand.yawImpulse(safeAngular.y(), drive.yawRateTargetRadiansPerSecond(),
                safeInertia.y(), constraintStepSeconds)
            : 0.0;
        double[] requestedDriveImpulses = SkidSteerDriveDemand.allocate(
            drive == null ? 0.0 : drive.impulse(constraintStepSeconds), skidYawImpulse,
            driveLeverX, driveSideX, drivenContacts
        );
        for (int index=0; index<contacts.size(); ++index) {
            LandingGearContact contact = contacts.get(index);
            boolean liquidSupport = isLiquidSupport(contact);
            grips[index] = !LiquidSupportModel.hasTireTraction(liquidSupport)
                ? new TireContactMaterial.Grip(0.0, 0.0, "LIQUID", true)
                : contact.device() == null
                    ? TireContactMaterial.staticSupport(vehicle,blocks.get(contact.pointLocal()))
                    : TireContactMaterial.evaluate(contact.device(),blocks.get(contact.pointLocal()));
            rollingCoefficients[index] = liquidSupport || !freeRolling(contact.device()) ? 0.0
                : TireContactMaterial.rollingResistance(
                    landingGearRollingResistanceCoefficient(contact.device()), grips[index]);
            if (!contact.exactCollision()) continue;
            double penetrationError = Math.max(0, contact.penetrationDepthMeters()-LANDING_GEAR_POSITION_SLOP_METERS);
            double target = contact.normalVelocityTargetMetersPerSecond();
            if (!(Double.isFinite(target) && target < -1e-9)) {
                target = penetrationError>0 ? penetrationError*(1-Math.exp(
                    -LANDING_GEAR_POSITION_STABILIZATION_RATE_PER_SECOND*constraintStepSeconds))/constraintStepSeconds
                    : Double.isFinite(target) ? Math.min(0,target) : 0;
            }
            TireNormalCompliance.Response response = contact.roadSuspension()
                ? TireNormalCompliance.evaluateRoadSuspension(
                    RoadSuspensionModel.parameters(contact.device(), safeMass, installedSupports,
                        roadVerticalProjection),
                    contact.complianceGapMeters(), target, 0.01, constraintStepSeconds,
                    contact.trueImpactResidual())
                : TireNormalCompliance.evaluate(contact.device(),safeMass,
                    installedSupports,contact.surfaceGapMeters(),target,0.01,constraintStepSeconds,
                    !liquidSupport,contact.trueImpactResidual());
            responses[index]=response;
            constraints.add(new GroundContactImpulseSolver.Contact(
                contact.pointLocal().subtract(safeCenterOfMassLocal),
                landingGearWheelForwardWorld(orientation,physicalSteeringDegrees(vehicle,contact.device(),drive)),
                grips[index].motive(), grips[index].lateral(), response.targetMps(),
                drivenContacts[index],
                rollingCoefficients[index],
                SkidSteerDriveDemand.usesMotorizedRolling(
                    freeRolling(contact.device()), drive != null && drive.skidSteer() && drivenContacts[index],
                    liquidSupport),
                response.softnessInverseKg(),drive == null ? Double.POSITIVE_INFINITY : drive.coastingCoefficientLimit(),
                requestedDriveImpulses[index],response.hardNormalVelocityTargetMps()));
            indices.add(index);
        }
        GroundContactImpulseSolver.Result reaction = GroundContactImpulseSolver.solve(
            safeLinear, safeAngular, safeInertia, orientation, Vec3d.ZERO, safeMass,
            brake, drive == null ? 0 : drive.impulse(constraintStepSeconds), constraintStepSeconds, constraints);
        solvedLinear = reaction.velocityWorld();
        solvedAngular = reaction.angularVelocityBody();
        double[] normalImpulses = new double[contacts.size()];
        double[] lateralImpulses = new double[contacts.size()];
        double[] longitudinalImpulses = new double[contacts.size()];
        for (int i=0; i<indices.size(); ++i) {
            int index=indices.get(i);
            normalImpulses[index]=reaction.normalImpulses()[i];
            lateralImpulses[index]=reaction.lateralImpulses()[i];
            longitudinalImpulses[index]=reaction.longitudinalImpulses()[i];
        }
        double totalLateral=0, passiveLateral=0, steeringLateral=0, passiveYaw=0, naturalYaw=0;
        double totalBrake=0, totalRolling=0, passiveApplicationZ=0, passiveLever=0, passiveWeight=0;
        List<LandingGearContactSnapshot> snapshots = new ArrayList<>();
        for (int i=0; i<contacts.size(); ++i) {
            LandingGearContact contact = contacts.get(i);
            Vec3d lever = contact.pointLocal().subtract(safeCenterOfMassLocal);
            double steering = physicalSteeringDegrees(vehicle,contact.device(),drive);
            Vec3d forward = landingGearWheelForwardWorld(orientation,steering);
            Vec3d lateral = new Vec3d(forward.z(),0,-forward.x());
            double side = Math.abs(lateralImpulses[i]);
            boolean commanded = landingGearSteeringCommanded(contact.device(),steering);
            totalLateral += side;
            double yaw = toWorld(orientation,lever.cross(toLocal(orientation,lateral.scale(lateralImpulses[i])))).y();
            naturalYaw += yaw + toWorld(orientation,lever.cross(
                toLocal(orientation,forward.scale(longitudinalImpulses[i])))).y();
            if (commanded) steeringLateral+=side;
            else {
                passiveLateral+=side; passiveYaw+=yaw;
                double weight = grips[i].lateral()*normalImpulses[i];
                passiveApplicationZ+=weight*lever.z(); passiveLever+=weight*Math.abs(lever.z()); passiveWeight+=weight;
            }
            double rollingCoefficient=rollingCoefficients[i];
            // Longitudinal demand is shared. Attribute its accepted total to the
            // active command rather than inventing a second rolling impulse.
            double rollingShare=brake*grips[i].motive()+rollingCoefficient>EPSILON
                ? rollingCoefficient/(brake*grips[i].motive()+rollingCoefficient) : 0;
            boolean rollingDevice=freeRolling(contact.device());
            double rolling=rollingDevice ? longitudinalImpulses[i]*rollingShare : 0;
            double braking=rollingDevice ? longitudinalImpulses[i]-rolling : 0;
            double supportFriction=rollingDevice ? 0 : longitudinalImpulses[i];
            totalRolling+=Math.abs(rolling); totalBrake+=Math.abs(braking);
            TireNormalCompliance.Response response=responses[i];
            snapshots.add(new LandingGearContactSnapshot(contact.pointLocal(),contact.surfaceGapMeters(),
                contact.contactMode(),contact.penetrationDepthMeters(),contact.exactCollision(),
                response==null ? contact.normalVelocityTargetMetersPerSecond() : response.targetMps(),normalImpulses[i],lateralImpulses[i],
                grips[i].lateral()*normalImpulses[i],braking,rolling,rollingCoefficient,steering,
                grips[i].lateral(),grips[i].motive(),grips[i].material(),grips[i].wet(),
                rollingDevice ? "ROLLING_TIRE" : "RIGID_SUPPORT",supportFriction,
                response==null ? 0 : response.softnessInverseKg(),response==null ? 0 : response.stiffnessNpm(),
                response==null ? 0 : response.dampingNsPm(),response==null ? 0 : response.nominalDeflectionMeters(),
                response != null && Double.isFinite(response.hardNormalVelocityTargetMps()),
                response != null && Double.isFinite(response.hardNormalVelocityTargetMps())
                    ? response.hardNormalVelocityTargetMps() : 0.0,
                contact.device(), contact.roadSuspension()));
        }
        double maxResidualClosing = maximumWheelClosingSpeed(orientation,contacts,solvedLinear,solvedAngular,safeCenterOfMassLocal);
        double maxResidualSlip = maximumWheelLateralSlipSpeed(vehicle,orientation,contacts,solvedLinear,solvedAngular,safeCenterOfMassLocal);
        if (vehicle != null) vehicle.slipping = reaction.saturated();
        return new LandingGearConstraintResult(solvedLinear,solvedAngular,solvedLinear.subtract(safeLinear),
            solvedAngular.subtract(safeAngular),contacts.size(),exactCollisionCount,proximityContactCount,
            reaction.normalImpulseNs(),totalLateral,passiveLateral,steeringLateral,Math.abs(passiveYaw),Math.abs(naturalYaw),
            passiveWeight>EPSILON ? passiveApplicationZ/passiveWeight : 0,
            passiveWeight>EPSILON ? passiveLever/passiveWeight : 0,totalBrake,totalRolling,
            maxClosingBefore,maxResidualClosing,maxLateralSlipBefore,maxResidualSlip,maximumPenetration,List.copyOf(snapshots));
    }

    static double maximumWheelClosingSpeed(
        RotationMatrix orientation,
        List<LandingGearContact> contacts,
        Vec3d linearVelocityWorld,
        Vec3d angularVelocityBody,
        Vec3d centerOfMassLocal
    ) {
        double maximum = 0.0;
        Vec3d omegaWorld = toWorld(orientation, angularVelocityBody);
        for (LandingGearContact contact : contacts) {
            Vec3d pointVelocity = linearVelocityWorld.add(
                omegaWorld.cross(toWorld(orientation, contact.pointLocal().subtract(centerOfMassLocal)))
            );
            maximum = Math.max(maximum, Math.max(0.0, -pointVelocity.y()));
        }
        return maximum;
    }

    static double maximumWheelLateralSlipSpeed(
        EntityVehicleF_Physics vehicle,
        RotationMatrix orientation,
        List<LandingGearContact> contacts,
        Vec3d linearVelocityWorld,
        Vec3d angularVelocityBody,
        Vec3d centerOfMassLocal
    ) {
        double maximum = 0.0;
        Vec3d omegaWorld = toWorld(orientation, angularVelocityBody);
        Vec3d bodyForward = landingGearBodyForwardWorld(orientation);
        Vec3d bodyLateral = new Vec3d(
            bodyForward.z(), 0.0, -bodyForward.x()
        ).normalized();
        for (LandingGearContact contact : contacts) {
            // Static skid/float model support stations are normal-only rigid
            // support, not tires. They must not drive IV's wheel-slip state or
            // skid particles. Proximity is likewise diagnostic only.
            if (contact.device() == null || !contact.exactCollision()) {
                continue;
            }
            double steeringDegrees = landingGearSteeringDegrees(
                vehicle, contact.device()
            );
            if (landingGearSteeringCommanded(contact.device(), steeringDegrees)) {
                Vec3d forward = landingGearWheelForwardWorld(
                    orientation, steeringDegrees
                );
                Vec3d lateral = new Vec3d(
                    forward.z(), 0.0, -forward.x()
                ).normalized();
                Vec3d pointVelocity = linearVelocityWorld.add(
                    omegaWorld.cross(toWorld(orientation, contact.pointLocal().subtract(centerOfMassLocal)))
                );
                maximum = Math.max(maximum, Math.abs(pointVelocity.dot(lateral)));
            } else {
                Vec3d pointVelocity = linearVelocityWorld.add(
                    omegaWorld.cross(toWorld(orientation, contact.pointLocal().subtract(centerOfMassLocal)))
                );
                maximum = Math.max(
                    maximum,
                    Math.abs(pointVelocity.dot(bodyLateral))
                );
            }
        }
        return maximum;
    }

    static List<LandingGearContact> physicalLandingGearConstraintContacts(
        List<LandingGearPhysicalContact> physicalContacts
    ) {
        List<LandingGearContact> contacts = new ArrayList<>();
        if (physicalContacts == null) {
            return contacts;
        }
        for (LandingGearPhysicalContact contact : physicalContacts) {
            if (contact == null || contact.pointLocal() == null
                || !contact.pointLocal().isFinite()) {
                continue;
            }
            addUniqueConstraintContact(
                contacts,
                contact.device(),
                contact.pointLocal(),
                contact.surfaceGapMeters(),
                contact.contactMode(),
                Math.max(
                    0.0,
                    Double.isFinite(contact.penetrationDepthMeters())
                        ? contact.penetrationDepthMeters()
                        : 0.0
                ),
                contact.exactCollision(),
                contact.normalVelocityTargetMetersPerSecond(),
                contact.trueImpactResidual(),
                contact.complianceGapMeters(), contact.roadSuspension()
            );
        }
        contacts.sort(
            Comparator.comparingDouble((LandingGearContact contact) -> contact.pointLocal().z())
                .thenComparingDouble(contact -> contact.pointLocal().x())
                .thenComparingDouble(contact -> contact.pointLocal().y())
        );
        return contacts;
    }

    static List<LandingGearContact> groundedWheelConstraintContacts(
        EntityVehicleF_Physics vehicle
    ) {
        List<LandingGearContact> contacts = new ArrayList<>();
        if (vehicle == null || vehicle.groundDeviceCollective == null) {
            return contacts;
        }
        VehicleGroundDeviceCollectionAccessor accessor =
            (VehicleGroundDeviceCollectionAccessor) vehicle.groundDeviceCollective;
        addGroundedBoxConstraintContacts(contacts, accessor.pmweatherIv$getFrontLeftGDB());
        addGroundedBoxConstraintContacts(contacts, accessor.pmweatherIv$getFrontRightGDB());
        addGroundedBoxConstraintContacts(contacts, accessor.pmweatherIv$getRearLeftGDB());
        addGroundedBoxConstraintContacts(contacts, accessor.pmweatherIv$getRearRightGDB());
        contacts.sort(
            Comparator.comparingDouble((LandingGearContact contact) -> contact.pointLocal().z())
                .thenComparingDouble(contact -> contact.pointLocal().x())
                .thenComparingDouble(contact -> contact.pointLocal().y())
        );
        return contacts;
    }

    static void addGroundedBoxConstraintContacts(
        List<LandingGearContact> contacts,
        VehicleGroundDeviceBox box
    ) {
        // isGrounded is deliberately kept as the broad animation/contact sensor:
        // MTS sets it both for an exact solid-box collision and for its 0.05-block
        // look-down shell. LandingGearContact remembers which state this is for
        // diagnostics, but only exact collision receives physical authority.
        if (box == null || !box.isGrounded || box.isUsingLiquidBoxes) {
            return;
        }
        double penetrationDepth = Math.max(
            0.0,
            Double.isFinite(box.collisionDepth) ? box.collisionDepth : 0.0
        );
        boolean exactCollision = box.isCollided || penetrationDepth > EPSILON;
        List<PartGroundDevice> devices = box.getGroundDevices();
        boolean addedDevice = false;
        if (devices != null) {
            for (PartGroundDevice device : devices) {
                if (!TireContactMaterial.activeSupport(device)) continue;
                Vec3d contactPoint = landingGearDeviceContactPointLocal(device);
                if (contactPoint != null) {
                    addUniqueConstraintContact(
                        contacts, device, contactPoint, penetrationDepth, exactCollision
                    );
                    addedDevice = true;
                }
            }
        }
        if (!addedDevice && (devices == null || devices.isEmpty())
            && box.contactPoint != null && validGroundPoint(box.contactPoint)) {
            addUniqueConstraintContact(
                contacts, null, point(box.contactPoint), penetrationDepth, exactCollision
            );
        }
    }

    /**
     * MTS wheelbasePoint is the authored wheel station/centre.  Its own ground
     * contact starts at the live localOffset and scaled size. For wheels, the
     * support mapping follows the installed axle and current body tilt; other
     * devices retain their authored lower station. No additional vehicle scale
     * is applied to localOffset, which IV has already scaled and animated.
     */
    public static Vec3d landingGearDeviceContactPointLocal(PartGroundDevice device) {
        return device == null || device.vehicleOn == null ? null
            : landingGearDeviceContactPointLocal(device,device.vehicleOn.orientation);
    }

    public static Vec3d landingGearDeviceContactPointLocal(PartGroundDevice device, RotationMatrix orientation) {
        if (device==null || orientation==null) return null;
        Point3D center=validGroundPoint(device.localOffset) ? device.localOffset
            : validGroundPoint(device.wheelbasePoint) ? device.wheelbasePoint : null;
        if (center==null) return null;
        double height=device.getHeight();
        double halfHeight=Double.isFinite(height) ? Math.max(0,height)*0.5 : 0;
        if (!device.definition.ground.isWheel || device.vehicleOn==null) {
            return new Vec3d(center.x,center.y-halfHeight,center.z);
        }
        Vec3d axle=toLocal(device.vehicleOn.orientation,
            point(new Point3D(1,0,0).rotate(device.orientation)));
        double width=device.getWidth();
        double halfWidth=Double.isFinite(width) ? Math.max(0,width)*0.5 : 0;
        return point(center).add(ModelCoordinates.tireSupportOffset(orientation,axle,halfHeight,halfWidth));
    }

    static boolean validGroundPoint(Point3D value) {
        return value != null
            && Double.isFinite(value.x)
            && Double.isFinite(value.y)
            && Double.isFinite(value.z);
    }

    static void addUniqueConstraintContact(
        List<LandingGearContact> contacts,
        PartGroundDevice device,
        Vec3d candidate,
        double penetrationDepth,
        boolean exactCollision
    ) {
        addUniqueConstraintContact(
            contacts, device, candidate, Double.NaN, "IV_SENSOR",
            penetrationDepth, exactCollision, Double.NaN
        );
    }

    static void addUniqueConstraintContact(
        List<LandingGearContact> contacts,
        PartGroundDevice device,
        Vec3d candidate,
        double surfaceGapMeters,
        String contactMode,
        double penetrationDepth,
        boolean exactCollision,
        double normalVelocityTargetMetersPerSecond
    ) {
        addUniqueConstraintContact(contacts, device, candidate, surfaceGapMeters, contactMode,
            penetrationDepth, exactCollision, normalVelocityTargetMetersPerSecond, false);
    }

    static void addUniqueConstraintContact(
        List<LandingGearContact> contacts,
        PartGroundDevice device,
        Vec3d candidate,
        double surfaceGapMeters,
        String contactMode,
        double penetrationDepth,
        boolean exactCollision,
        double normalVelocityTargetMetersPerSecond,
        boolean trueImpactResidual
    ) {
        addUniqueConstraintContact(contacts, device, candidate, surfaceGapMeters, contactMode,
            penetrationDepth, exactCollision, normalVelocityTargetMetersPerSecond,
            trueImpactResidual, Double.NaN, false);
    }

    static void addUniqueConstraintContact(
        List<LandingGearContact> contacts,
        PartGroundDevice device,
        Vec3d candidate,
        double surfaceGapMeters,
        String contactMode,
        double penetrationDepth,
        boolean exactCollision,
        double normalVelocityTargetMetersPerSecond,
        boolean trueImpactResidual,
        double complianceGapMeters,
        boolean roadSuspension
    ) {
        for (int index = 0; index < contacts.size(); ++index) {
            LandingGearContact existing = contacts.get(index);
            if (existing.pointLocal().subtract(candidate).lengthSquared() <= 1.0E-10) {
                if (penetrationDepth > existing.penetrationDepthMeters()
                    || (exactCollision && !existing.exactCollision())
                    || (trueImpactResidual && !existing.trueImpactResidual())
                    || (roadSuspension && !existing.roadSuspension())) {
                    double existingTarget =
                        existing.normalVelocityTargetMetersPerSecond();
                    double mergedTarget;
                    if (existing.trueImpactResidual() && trueImpactResidual
                        && Double.isFinite(existingTarget)
                        && Double.isFinite(normalVelocityTargetMetersPerSecond)) {
                        mergedTarget = Math.min(existingTarget, normalVelocityTargetMetersPerSecond);
                    } else if (existing.trueImpactResidual()) {
                        mergedTarget = existingTarget;
                    } else if (trueImpactResidual) {
                        mergedTarget = normalVelocityTargetMetersPerSecond;
                    } else if (Double.isFinite(existingTarget)
                        && Double.isFinite(normalVelocityTargetMetersPerSecond)) {
                        mergedTarget = Math.max(
                            existingTarget, normalVelocityTargetMetersPerSecond
                        );
                    } else if (Double.isFinite(existingTarget)) {
                        mergedTarget = existingTarget;
                    } else {
                        mergedTarget = normalVelocityTargetMetersPerSecond;
                    }
                    contacts.set(index, new LandingGearContact(
                        existing.device() != null ? existing.device() : device,
                        existing.pointLocal(),
                        Double.isFinite(existing.surfaceGapMeters())
                            ? existing.surfaceGapMeters()
                            : surfaceGapMeters,
                        existing.exactCollision()
                            ? existing.contactMode()
                            : contactMode,
                        Math.max(existing.penetrationDepthMeters(), penetrationDepth),
                        existing.exactCollision() || exactCollision,
                        mergedTarget,
                        existing.trueImpactResidual() || trueImpactResidual,
                        existing.roadSuspension() && Double.isFinite(existing.complianceGapMeters())
                            ? existing.complianceGapMeters() : complianceGapMeters,
                        existing.roadSuspension() || roadSuspension
                    ));
                }
                return;
            }
        }
        contacts.add(new LandingGearContact(
            device, candidate, surfaceGapMeters, contactMode,
            penetrationDepth, exactCollision,
            normalVelocityTargetMetersPerSecond, trueImpactResidual,
            complianceGapMeters, roadSuspension
        ));
    }

    static Vec3d landingGearBodyForwardWorld(RotationMatrix orientation) {
        return landingGearWheelForwardWorld(orientation, 0.0);
    }

    private static double physicalSteeringDegrees(EntityVehicleF_Physics vehicle, PartGroundDevice device,
            RoadVehiclePhysics.DriveSnapshot drive) {
        if (vehicle != null && vehicle.outOfHealth) return 0.0;
        return drive == null ? landingGearSteeringDegrees(vehicle,device)
            : device != null && device.placementDefinition != null && device.placementDefinition.turnsWithSteer
                ? drive.steeringDegrees() : 0;
    }

    private static boolean isLiquidSupport(LandingGearContact contact) {
        return contact != null && contact.contactMode() != null
            && contact.contactMode().startsWith("FLOAT_LIQUID_");
    }

    static double landingGearSteeringDegrees(
        EntityVehicleF_Physics vehicle,
        PartGroundDevice device
    ) {
        if (vehicle == null || device == null) {
            return 0.0;
        }
        double steeringDegrees = device.placementDefinition != null
            && device.placementDefinition.turnsWithSteer
                ? -vehicle.rudderAngleVar.currentValue
                : 0.0;
        return Double.isFinite(steeringDegrees) ? steeringDegrees : 0.0;
    }

    static boolean landingGearSteeringCommanded(
        PartGroundDevice device,
        double steeringDegrees
    ) {
        return device != null
            && device.placementDefinition != null
            && device.placementDefinition.turnsWithSteer
            && Math.abs(steeringDegrees) > 1.0E-6;
    }

    static Vec3d landingGearWheelForwardWorld(
        RotationMatrix orientation,
        double steeringDegrees
    ) {
        double steeringRadians = Math.toRadians(steeringDegrees);
        Vec3d forwardBody = new Vec3d(
            Math.sin(steeringRadians), 0.0, Math.cos(steeringRadians)
        );
        Vec3d forwardWorld = toWorld(orientation, forwardBody);
        Vec3d horizontal = new Vec3d(forwardWorld.x(), 0.0, forwardWorld.z());
        if (horizontal.lengthSquared() <= 1.0E-12) {
            Vec3d fallback = toWorld(orientation, new Vec3d(0.0, 0.0, 1.0));
            horizontal = new Vec3d(fallback.x(), 0.0, fallback.z());
        }
        return horizontal.lengthSquared() <= 1.0E-12
            ? new Vec3d(0.0, 0.0, 1.0)
            : horizontal.normalized();
    }

    private static boolean freeRolling(PartGroundDevice device) {
        return device != null && device.definition.ground != null
            && (device.definition.ground.isWheel || device.definition.ground.isTread);
    }

    static double landingGearRollingResistanceCoefficient(PartGroundDevice device) {
        if (device == null || device.definition == null || device.definition.ground == null
            || !device.definition.ground.isWheel) {
            return 0.0;
        }
        double diameter = device.getHeight();
        if (!Double.isFinite(diameter) || diameter <= 1.0E-6) {
            return LANDING_GEAR_MIN_ROLLING_RESISTANCE_COEFFICIENT;
        }
        double radius = Math.max(1.0E-4, diameter * 0.5);
        return Vec3d.clamp(
            LANDING_GEAR_ROLLING_RESISTANCE_DEFORMATION_METERS / radius,
            LANDING_GEAR_MIN_ROLLING_RESISTANCE_COEFFICIENT,
            LANDING_GEAR_MAX_ROLLING_RESISTANCE_COEFFICIENT
        );
    }

    static double landingGearBrakeCommand(EntityVehicleF_Physics vehicle) {
        if (vehicle == null) {
            return 0.0;
        }
        return landingGearBrakeCommand(vehicle.brakeVar.currentValue,
            vehicle.parkingBrakeVar.isActive, vehicle.parkingBrakeVar.currentValue);
    }

    static double landingGearBrakeCommand(double serviceValue, boolean parkingActive, double parkingValue) {
        double service = Double.isFinite(serviceValue)
            ? Vec3d.clamp(Math.abs(serviceValue), 0.0, 1.0)
            : 0.0;
        double parking = parkingActive
            ? 1.0
            : Double.isFinite(parkingValue)
                ? Vec3d.clamp(Math.abs(parkingValue), 0.0, 1.0)
                : 0.0;
        return Math.max(service, parking);
    }

    public record LandingGearConstraintResult(
        Vec3d linearVelocityWorld,
        Vec3d angularVelocityBody,
        Vec3d linearVelocityChangeWorld,
        Vec3d angularVelocityChangeBody,
        int contactCount,
        int exactCollisionCount,
        int proximityContactCount,
        double totalNormalImpulseNewtonSeconds,
        double totalLateralImpulseNewtonSeconds,
        double aggregateLateralImpulseNewtonSeconds,
        double commandedSteeringImpulseNewtonSeconds,
        double passiveYawAngularImpulseNewtonMeterSeconds,
        double naturalTireYawAngularImpulseNewtonMeterSeconds,
        double aggregateLateralApplicationZMeters,
        double passiveYawLeverArmMeters,
        double totalBrakeImpulseNewtonSeconds,
        double totalRollingResistanceImpulseNewtonSeconds,
        double maximumClosingSpeedBeforeMetersPerSecond,
        double maximumResidualClosingSpeedMetersPerSecond,
        double maximumLateralSlipBeforeMetersPerSecond,
        double maximumResidualLateralSlipMetersPerSecond,
        double maximumPenetrationDepthMeters,
        List<LandingGearContactSnapshot> contacts
    ) {
    }

    /**
     * Live physical gear contact supplied by the Sable substep terrain probe.
     * The device remains IV-authored content data; only contact state comes from
     * the current physical pose rather than IV's mirrored 20 Hz sensor flags.
     */
    public record LandingGearPhysicalContact(
        PartGroundDevice device,
        Vec3d pointLocal,
        double surfaceGapMeters,
        String contactMode,
        double penetrationDepthMeters,
        boolean exactCollision,
        double normalVelocityTargetMetersPerSecond,
        double inwardNormalSpeedMetersPerSecond,
        Vec3d worldPoint,
        BlockPos supportBlock,
        boolean trueImpactResidual,
        double complianceGapMeters,
        boolean roadSuspension
    ) {
        public LandingGearPhysicalContact(PartGroundDevice device, Vec3d pointLocal,
                double surfaceGapMeters, String contactMode, double penetrationDepthMeters,
                boolean exactCollision, double normalVelocityTargetMetersPerSecond,
                double inwardNormalSpeedMetersPerSecond, Vec3d worldPoint, BlockPos supportBlock) {
            this(device, pointLocal, surfaceGapMeters, contactMode, penetrationDepthMeters,
                exactCollision, normalVelocityTargetMetersPerSecond, inwardNormalSpeedMetersPerSecond,
                worldPoint, supportBlock, false, Double.NaN, false);
        }

        public LandingGearPhysicalContact(PartGroundDevice device, Vec3d pointLocal,
                double surfaceGapMeters, String contactMode, double penetrationDepthMeters,
                boolean exactCollision, double normalVelocityTargetMetersPerSecond,
                double inwardNormalSpeedMetersPerSecond, Vec3d worldPoint, BlockPos supportBlock,
                boolean trueImpactResidual) {
            this(device, pointLocal, surfaceGapMeters, contactMode, penetrationDepthMeters,
                exactCollision, normalVelocityTargetMetersPerSecond, inwardNormalSpeedMetersPerSecond,
                worldPoint, supportBlock, trueImpactResidual, Double.NaN, false);
        }

        public LandingGearPhysicalContact withNormalVelocityTarget(double targetMetersPerSecond) {
            return withTarget(targetMetersPerSecond, trueImpactResidual);
        }

        public LandingGearPhysicalContact withTrueImpactResidual(double targetMetersPerSecond) {
            return withTarget(targetMetersPerSecond, true);
        }

        private LandingGearPhysicalContact withTarget(double targetMetersPerSecond, boolean impactResidual) {
            return new LandingGearPhysicalContact(
                device, pointLocal, surfaceGapMeters, contactMode, penetrationDepthMeters,
                exactCollision, targetMetersPerSecond, inwardNormalSpeedMetersPerSecond,
                worldPoint, supportBlock, impactResidual, complianceGapMeters, roadSuspension
            );
        }
    }

    public record LandingGearContactSnapshot(
        Vec3d pointLocal,
        double surfaceGapMeters,
        String contactMode,
        double penetrationDepthMeters,
        boolean exactCollision,
        double normalVelocityTargetMetersPerSecond,
        double normalImpulseNewtonSeconds,
        double lateralImpulseNewtonSeconds,
        double lateralImpulseLimitNewtonSeconds,
        double brakeImpulseNewtonSeconds,
        double rollingResistanceImpulseNewtonSeconds,
        double rollingResistanceCoefficient,
        double steeringDegrees,
        double lateralFriction,
        double motiveFriction,
        String material,
        boolean wet,
        String frictionMode,
        double passiveSupportImpulseNewtonSeconds,
        double normalSoftnessInverseKg, double supportStiffnessNpm,
        double supportDampingNsPm, double nominalDeflectionMeters,
        boolean hardBumpstopEnabled, double hardBumpstopVelocityTargetMetersPerSecond,
        PartGroundDevice device, boolean roadSuspension
    ) {
        public LandingGearContactSnapshot(Vec3d pointLocal, double surfaceGapMeters,
                String contactMode, double penetrationDepthMeters, boolean exactCollision,
                double normalVelocityTargetMetersPerSecond, double normalImpulseNewtonSeconds,
                double lateralImpulseNewtonSeconds, double lateralImpulseLimitNewtonSeconds,
                double brakeImpulseNewtonSeconds, double rollingResistanceImpulseNewtonSeconds,
                double rollingResistanceCoefficient, double steeringDegrees, double lateralFriction,
                double motiveFriction, String material, boolean wet, String frictionMode,
                double passiveSupportImpulseNewtonSeconds, double normalSoftnessInverseKg,
                double supportStiffnessNpm, double supportDampingNsPm, double nominalDeflectionMeters,
                boolean hardBumpstopEnabled, double hardBumpstopVelocityTargetMetersPerSecond) {
            this(pointLocal, surfaceGapMeters, contactMode, penetrationDepthMeters, exactCollision,
                normalVelocityTargetMetersPerSecond, normalImpulseNewtonSeconds, lateralImpulseNewtonSeconds,
                lateralImpulseLimitNewtonSeconds, brakeImpulseNewtonSeconds, rollingResistanceImpulseNewtonSeconds,
                rollingResistanceCoefficient, steeringDegrees, lateralFriction, motiveFriction, material,
                wet, frictionMode, passiveSupportImpulseNewtonSeconds, normalSoftnessInverseKg,
                supportStiffnessNpm, supportDampingNsPm, nominalDeflectionMeters, hardBumpstopEnabled,
                hardBumpstopVelocityTargetMetersPerSecond, null, false);
        }
    }

    public record LandingGearContact(
        PartGroundDevice device,
        Vec3d pointLocal,
        double surfaceGapMeters,
        String contactMode,
        double penetrationDepthMeters,
        boolean exactCollision,
        double normalVelocityTargetMetersPerSecond,
        boolean trueImpactResidual,
        double complianceGapMeters,
        boolean roadSuspension
    ) {
        public LandingGearContact(PartGroundDevice device, Vec3d pointLocal,
                double surfaceGapMeters, String contactMode, double penetrationDepthMeters,
                boolean exactCollision, double normalVelocityTargetMetersPerSecond) {
            this(device, pointLocal, surfaceGapMeters, contactMode, penetrationDepthMeters,
                exactCollision, normalVelocityTargetMetersPerSecond, false, Double.NaN, false);
        }

        public LandingGearContact(PartGroundDevice device, Vec3d pointLocal,
                double surfaceGapMeters, String contactMode, double penetrationDepthMeters,
                boolean exactCollision, double normalVelocityTargetMetersPerSecond,
                boolean trueImpactResidual) {
            this(device, pointLocal, surfaceGapMeters, contactMode, penetrationDepthMeters,
                exactCollision, normalVelocityTargetMetersPerSecond, trueImpactResidual,
                Double.NaN, false);
        }
    }

}
