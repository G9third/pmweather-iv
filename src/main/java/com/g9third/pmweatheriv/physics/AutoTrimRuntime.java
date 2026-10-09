package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.jsondefs.JSONCollisionGroup;
import com.g9third.pmweatheriv.sable.SableVehicleBody;

/** Builds pitch-only controller observations from the already-computed owner-tick solve. */
public final class AutoTrimRuntime {
    private AutoTrimRuntime() {}

    public static AutoTrimController.Output advance(
        EntityVehicleF_Physics vehicle,
        AircraftState state,
        AirframeLoads.SolveResult result,
        SableVehicleBody body,
        long tick
    ) {
        AutoTrimController controller = state.autoTrim;
        double trim = vehicle.elevatorTrimVar.currentValue;
        if (!controller.enabled()) return new AutoTrimController.Output(trim,
            AutoTrimController.State.OFF, "OFF", false);
        if (tick >= state.nextAutoTrimDamageCheck) {
            state.autoTrimDamageSignature = damageSignature(vehicle);
            state.nextAutoTrimDamageCheck = tick + 20;
        }

        AircraftKinematics physical = state.kinematics;
        double wingArea = state.plan != null && state.plan.geometry() != null
            ? state.plan.geometry().wingArea() : 0.0;
        double maximumLiftCoefficient = maximumLiftCoefficient(vehicle, state);
        double maxMainWingSeparation = maximumMainWingSeparation(state);
        boolean fixedWing = vehicle.definition != null && vehicle.definition.motorized != null
            && vehicle.definition.motorized.isAircraft && !vehicle.definition.motorized.isBlimp
            && state.plan != null && !state.plan.rotorcraft();
        boolean finite = result != null && physical != null && physical.orientation() != null
            && physical.centerVelocityWorld() != null && physical.centerVelocityWorld().isFinite()
            && state.angularVelocityBody != null && state.angularVelocityBody.isFinite()
            && result.inertia() != null && result.inertia().isFinite()
            && result.torqueBody() != null && result.torqueBody().isFinite()
            && Double.isFinite(trim) && Double.isFinite(result.trueAirspeed())
            && Double.isFinite(result.forwardAirspeed()) && Double.isFinite(result.airDensity())
            && Double.isFinite(wingArea) && wingArea > 0.01
            && Double.isFinite(maximumLiftCoefficient) && maximumLiftCoefficient > 0.01
            && Double.isFinite(maxMainWingSeparation)
            && result.centerWind() != null && result.centerWind().windMetersPerSecond().isFinite();
        if (!finite) return controller.update(new AutoTrimController.Input(
            tick, trim, EntityVehicleF_Physics.MAX_ELEVATOR_TRIM, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, vehicle.elevatorInputVar.currentValue,
            vehicle.flapActualAngleVar.currentValue, vehicle.currentMass,
            0, state.autoTrimDamageSignature, fixedWing, !vehicle.outOfHealth,
            true, false, vehicle.autopilotValueVar.isActive, false
        ));

        RotationMatrix orientation = physical.orientation();
        Vec3d forwardWorld = FlightMath.toWorld(orientation, new Vec3d(0.0, 0.0, 1.0)).normalized();
        Vec3d omegaBody = state.angularVelocityBody;
        Vec3d omegaWorld = FlightMath.toWorld(orientation, omegaBody);
        Vec3d forwardDerivative = omegaWorld.cross(forwardWorld);
        double horizontalForward = Math.hypot(forwardWorld.x(), forwardWorld.z());
        double horizontalForwardDerivative = horizontalForward > 1.0E-8
            ? (forwardWorld.x() * forwardDerivative.x() + forwardWorld.z() * forwardDerivative.z()) / horizontalForward
            : 0.0;
        double elevationDenominator = horizontalForward * horizontalForward
            + forwardWorld.y() * forwardWorld.y();
        double pitchRate = horizontalForward > 1.0E-8 && elevationDenominator > 1.0E-8
            ? (horizontalForward * forwardDerivative.y() - forwardWorld.y() * horizontalForwardDerivative)
                / elevationDenominator
            : forwardDerivative.y();

        Vec3d worldVelocity = physical.centerVelocityWorld();
        double horizontalWorldSpeed = Math.hypot(worldVelocity.x(), worldVelocity.z());
        double flightPath = Math.toDegrees(Math.atan2(worldVelocity.y(), Math.max(1.0E-5, horizontalWorldSpeed)));
        double dynamicPressure = 0.5 * result.airDensity() * result.trueAirspeed() * result.trueAirspeed();
        double bank = orientation.angles == null ? 0.0 : wrapDegrees(orientation.angles.z);
        Vec3d inertia = result.inertia();
        Vec3d gyroscopicMoment = new Vec3d(
            omegaBody.y() * omegaBody.z() * (inertia.z() - inertia.y()),
            omegaBody.z() * omegaBody.x() * (inertia.x() - inertia.z()),
            omegaBody.x() * omegaBody.y() * (inertia.y() - inertia.x())
        );
        Vec3d angularAccelerationBody = new Vec3d(
            (result.torqueBody().x() - gyroscopicMoment.x()) / Math.max(1.0E-6, inertia.x()),
            (result.torqueBody().y() - gyroscopicMoment.y()) / Math.max(1.0E-6, inertia.y()),
            (result.torqueBody().z() - gyroscopicMoment.z()) / Math.max(1.0E-6, inertia.z())
        );
        Vec3d angularAccelerationWorld = FlightMath.toWorld(orientation, angularAccelerationBody);
        Vec3d forwardSecondDerivative = angularAccelerationWorld.cross(forwardWorld)
            .add(omegaWorld.cross(forwardDerivative));
        double horizontalForwardSecondDerivative = horizontalForward > 1.0E-8
            ? (forwardDerivative.x() * forwardDerivative.x()
                + forwardWorld.x() * forwardSecondDerivative.x()
                + forwardDerivative.z() * forwardDerivative.z()
                + forwardWorld.z() * forwardSecondDerivative.z()) / horizontalForward
                - horizontalForwardDerivative * horizontalForwardDerivative / horizontalForward
            : 0.0;
        double elevationNumerator = horizontalForward * forwardDerivative.y()
            - forwardWorld.y() * horizontalForwardDerivative;
        double denominatorDerivative = 2.0 * (horizontalForward * horizontalForwardDerivative
            + forwardWorld.y() * forwardDerivative.y());
        double pitchAcceleration = elevationDenominator > 1.0E-8
            ? (horizontalForward * forwardSecondDerivative.y()
                - forwardWorld.y() * horizontalForwardSecondDerivative) / elevationDenominator
                - elevationNumerator * denominatorDerivative
                    / (elevationDenominator * elevationDenominator)
            : Double.NaN;
        boolean held = state.heldForSable || body == null || !body.isUsable()
            || body.riderReconnectHeld();
        boolean contact = body != null && body.hasRecentTerrainContact();
        boolean valuesFinite = Double.isFinite(forwardWorld.x()) && Double.isFinite(forwardWorld.y())
            && Double.isFinite(forwardWorld.z()) && Double.isFinite(pitchRate)
            && Double.isFinite(flightPath) && Double.isFinite(dynamicPressure)
            && Double.isFinite(pitchAcceleration) && Double.isFinite(bank)
            && Double.isFinite(wingArea) && wingArea > 0.01
            && Double.isFinite(maximumLiftCoefficient) && maximumLiftCoefficient > 0.01
            && Double.isFinite(maxMainWingSeparation);
        AutoTrimController.Input input = new AutoTrimController.Input(
            tick, trim, EntityVehicleF_Physics.MAX_ELEVATOR_TRIM,
            result.trueAirspeed(), result.forwardAirspeed(), dynamicPressure,
            wingArea, maximumLiftCoefficient, maxMainWingSeparation,
            result.trackAngleDegrees(), bank, pitchRate, flightPath, pitchAcceleration,
            vehicle.elevatorInputVar.currentValue, vehicle.flapActualAngleVar.currentValue,
            vehicle.currentMass, inertia.x(), state.autoTrimDamageSignature,
            fixedWing, !vehicle.outOfHealth, held, contact,
            vehicle.autopilotValueVar.isActive, valuesFinite
        );
        return controller.update(input);
    }

    private static double maximumMainWingSeparation(AircraftState state) {
        double maximum = 0.0;
        for (var entry : state.surfaceSeparationFractions.entrySet()) {
            if (!entry.getKey().startsWith("SABLE_WING_")) continue;
            Double value = entry.getValue();
            if (value != null && Double.isFinite(value)) maximum = Math.max(maximum, value);
        }
        return Vec3d.clamp(maximum, 0.0, 1.0);
    }

    private static double maximumLiftCoefficient(EntityVehicleF_Physics vehicle, AircraftState state) {
        if (vehicle == null || vehicle.definition == null || vehicle.definition.motorized == null
            || state.plan == null || state.plan.model() == null) return 0.0;
        ModelSurfaceMap.PreparedModel model = state.plan.model();
        LiftingSurfaceAdapter.PreparedFixedWingPlan wingPlan = state.plan.fixedWingPlan();
        boolean allWingsUseFlapSweep = state.liftingPoses.allWingsUseFlapSweep(model)
            || model.authoredPoseBindings().isEmpty() && wingPlan != null
                && wingPlan.animatedWingGeometry().usesFlapVariableAsWholeWingSweep();
        double flapFraction = allWingsUseFlapSweep ? 0.0
            : LiftingSurfaceAdapter.flapDeploymentFraction(vehicle.flapActualAngleVar.currentValue,
                vehicle.definition.motorized.flapNotches);
        return PMWeatherIVConfig.get().maxLiftCoefficient() * (1.0 + Math.abs(flapFraction) * 0.45);
    }

    private static int damageSignature(EntityVehicleF_Physics vehicle) {
        if (vehicle == null || vehicle.definition == null || vehicle.definition.collisionGroups == null) return 0;
        int hash = 1;
        int count = vehicle.definition.collisionGroups.size();
        for (int i = 0; i < count; ++i) {
            JSONCollisionGroup group = vehicle.definition.collisionGroups.get(i);
            if (group == null || group.health <= 0) continue;
            var damage = vehicle.getOrCreateVariable("collision_" + (i + 1) + "_damage");
            var totaled = vehicle.getOrCreateVariable("collision_" + (i + 1) + "_totaled");
            long value = Double.doubleToLongBits(damage.currentValue);
            hash = 31 * hash + (int) (value ^ (value >>> 32));
            hash = 31 * hash + (totaled.isActive ? 1 : 0);
        }
        return hash;
    }

    private static double wrapDegrees(double angle) {
        if (!Double.isFinite(angle)) return 0.0;
        angle %= 360.0;
        if (angle > 180.0) angle -= 360.0;
        if (angle < -180.0) angle += 360.0;
        return angle;
    }
}
