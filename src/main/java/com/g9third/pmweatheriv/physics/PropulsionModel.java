package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.mixin.PartEngineAccessor;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartEngine;
import minecrafttransportsimulator.entities.instances.PartPropeller;
import static com.g9third.pmweatheriv.physics.FlightMath.EPSILON;
import static com.g9third.pmweatheriv.physics.FlightMath.IV_TICKS_SQUARED_TO_SECONDS_SQUARED;
import static com.g9third.pmweatheriv.physics.FlightMath.PointFlow;
import static com.g9third.pmweatheriv.physics.FlightMath.point;
import static com.g9third.pmweatheriv.physics.FlightMath.pointFlow;
import static com.g9third.pmweatheriv.physics.FlightMath.toLocal;
import static com.g9third.pmweatheriv.physics.FlightMath.toWorld;

import static com.g9third.pmweatheriv.physics.AircraftWind.WindField;
import static com.g9third.pmweatheriv.physics.AirframeLoads.Accumulator;

/** Maps native IV actuators into physical force and torque at the owner tick. */
public final class PropulsionModel {
    private PropulsionModel() {}

    static PropulsionLoad addPropeller(
        EntityVehicleF_Physics vehicle,
        PartPropeller propeller,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        Accumulator accumulator
    ) {
        Vec3d pointLocal = point(propeller.localOffset);
        PointFlow flow = pointFlow(
            vehicle, pointLocal, accumulator.flightState.plan.centerOfMassLocal(),
            linearVelocityWorld, omegaWorld, windField, "PROP_" + propeller.uniqueUUID
        );
        Vec3d axisWorld = toWorld(vehicle, toLocal(vehicle.orientation, point(new Point3D(0.0, 0.0, 1.0).rotate(propeller.orientation)))).normalized();
        double physicalPointInflowMetersPerSecond = flow.relativeAirWorld().dot(axisWorld);
        // Fixed-wing propulsion remains an IV-authored actuator.  IV's
        // airstreamLinearVelocity is part of its opaque content-tuned propeller
        // law, not a general aerodynamic state variable.  Replacing it with
        // PMWeather-relative inflow caused wind to alter thrust a second time,
        // after the airframe had already responded physically to V + omega x r
        // - wind.  Preserve IV's native fixed-wing inflow and keep the physical
        // PMWeather-relative value as diagnostic evidence only.  Rotorcraft are
        // intentionally different: their distributed rotor-disc path below
        // evaluates IV's actuator magnitude against sampled disc inflow.
        double weatherEquivalentIvInflow = toIvPropellerInflow(vehicle, physicalPointInflowMetersPerSecond);
        double nativeInflow = propeller.airstreamLinearVelocity;
        Point3D forceMts = new Point3D();
        Point3D torqueMts = new Point3D();
        double value = propeller.addToForceOutput(forceMts, torqueMts);
        // Keep native actuator state/rundown current, but a totaled persistent chassis
        // cannot deliver actuator loads. Rotor and drivetrain paths use the same policy.
        if (vehicle.outOfHealth) {
            value = 0.0;
            forceMts.set(0.0, 0.0, 0.0);
            torqueMts.set(0.0, 0.0, 0.0);
        }
        double forceScale = ivForceNewtonsPerUnit(vehicle);
        Vec3d forceWorld = toWorld(vehicle, toLocal(vehicle.orientation, point(forceMts))).scale(forceScale);
        Vec3d torqueAboutModelOrigin = point(torqueMts).scale(forceScale);
        Vec3d forceBody = toLocal(vehicle, forceWorld);
        Vec3d torqueBody = torqueAboutModelOrigin.subtract(
            accumulator.flightState.plan.centerOfMassLocal().cross(forceBody)
        );
        accumulator.forceWorld = accumulator.forceWorld.add(forceWorld);
        accumulator.torqueBody = accumulator.torqueBody.add(torqueBody);
        String type = propeller.definition.propeller != null && propeller.definition.propeller.isRotor
            ? "ROTOR" : "PROPELLER";
        accumulator.recordPropulsionLoad(type, value, forceWorld, torqueBody);
        if (accumulator.capturePropulsion && accumulator.captureSurfaces) accumulator.propulsionSamples.add(new PropulsionSample(
            type,
            pointLocal,
            flow.windWorld(),
            axisWorld,
            nativeInflow,
            weatherEquivalentIvInflow,
            physicalPointInflowMetersPerSecond,
            vehicle.speedFactor,
            vehicle.throttleVar.currentValue,
            propeller.getOrCreateVariable("propeller_rpm").getValue(),
            propeller.getOrCreateVariable("propeller_pitch_deg").getValue(),
            propeller.desiredLinearVelocity,
            value,
            forceWorld,
            torqueBody
        ));
        return new PropulsionLoad(value, forceWorld, torqueBody);
    }

    static PropulsionLoad addJetEngine(
        EntityVehicleF_Physics vehicle,
        PartEngine engine,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        Accumulator accumulator
    ) {
        Vec3d pointLocal = point(engine.localOffset);
        PointFlow flow = pointFlow(
            vehicle, pointLocal, accumulator.flightState.plan.centerOfMassLocal(),
            linearVelocityWorld, omegaWorld, windField, "JET_" + engine.uniqueUUID
        );
        Vec3d axisWorld = toWorld(vehicle, toLocal(vehicle.orientation, point(new Point3D(0.0, 0.0, 1.0).rotate(engine.orientation)))).normalized();
        double physicalPointInflowMetersPerSecond = flow.relativeAirWorld().dot(axisWorld);
        // As with fixed-wing propellers, preserve IV's native internal jet
        // inflow.  PMWeather still changes the aircraft's actual relative airflow
        // (and therefore body/wing/tail forces), but it does not overwrite IV's
        // content-tuned engine state.  weatherEquivalentIvInflowPerTick remains
        // diagnostic-only so a trace can compare the two frames directly.
        double weatherEquivalentIvInflowPerTick = toIvJetInflow(vehicle, physicalPointInflowMetersPerSecond);
        PartEngineAccessor accessor = (PartEngineAccessor) engine;
        double nativeInflow = accessor.pmweatherIv$getEngineAxialVelocity();
        Point3D forceMts = new Point3D();
        Point3D torqueMts = new Point3D();
        double value = engine.addToForceOutput(forceMts, torqueMts);
        // Keep native actuator state/rundown current, but a totaled persistent chassis
        // cannot deliver actuator loads. Rotor and drivetrain paths use the same policy.
        if (vehicle.outOfHealth) {
            value = 0.0;
            forceMts.set(0.0, 0.0, 0.0);
            torqueMts.set(0.0, 0.0, 0.0);
        }
        double forceScale = ivForceNewtonsPerUnit(vehicle);
        Vec3d forceWorld = toWorld(vehicle, toLocal(vehicle.orientation, point(forceMts))).scale(forceScale);
        Vec3d torqueAboutModelOrigin = point(torqueMts).scale(forceScale);
        Vec3d forceBody = toLocal(vehicle, forceWorld);
        Vec3d torqueBody = torqueAboutModelOrigin.subtract(
            accumulator.flightState.plan.centerOfMassLocal().cross(forceBody)
        );
        accumulator.forceWorld = accumulator.forceWorld.add(forceWorld);
        accumulator.torqueBody = accumulator.torqueBody.add(torqueBody);
        accumulator.recordPropulsionLoad("JET", value, forceWorld, torqueBody);
        if (accumulator.capturePropulsion && accumulator.captureSurfaces) accumulator.propulsionSamples.add(new PropulsionSample(
            "JET",
            pointLocal,
            flow.windWorld(),
            axisWorld,
            nativeInflow,
            weatherEquivalentIvInflowPerTick,
            physicalPointInflowMetersPerSecond,
            vehicle.speedFactor,
            vehicle.throttleVar.currentValue,
            engine.rpm,
            0.0,
            0.0,
            value,
            forceWorld,
            torqueBody
        ));
        return new PropulsionLoad(value, forceWorld, torqueBody);
    }

    /**
     * Converts actual point-relative airspeed in m/s to IV PartPropeller's
     * authored internal inflow frame. IV computes this value as
     * {@code 20 * vehicle.motion.dot(axis)} before applying speedFactor to the
     * actual movement. Dividing by speedFactor preserves native no-weather
     * propulsion while still allowing local wind and rotational inflow.
     */
    static double toIvPropellerInflow(
        EntityVehicleF_Physics vehicle,
        double physicalMetersPerSecond
    ) {
        return physicalMetersPerSecond / Math.max(EPSILON, vehicle.speedFactor);
    }

    /**
     * Physical newtons represented by one IV force-accumulator unit for this
     * vehicle. One block is one meter and IV applies speedFactor only when the
     * integrated motion is translated into world displacement.
     */
    static double ivForceNewtonsPerUnit(EntityVehicleF_Physics vehicle) {
        return IV_TICKS_SQUARED_TO_SECONDS_SQUARED
            * Math.max(EPSILON, vehicle.speedFactor);
    }

    /** Converts actual point-relative airspeed to PartEngine's blocks/tick frame. */
    static double toIvJetInflow(
        EntityVehicleF_Physics vehicle,
        double physicalMetersPerSecond
    ) {
        return physicalMetersPerSecond / Math.max(EPSILON, vehicle.speedFactor * 20.0);
    }

    public record PropulsionLoad(double mtsForceValue, Vec3d forceWorldNewtons, Vec3d torqueBodyNewtonMeters) {
    }

    /** Owner-tick actuator loads retained for physical substeps and state replication. */
    public record PropulsionSnapshot(String type, double mtsForceValue,
        Vec3d forceWorldNewtons, Vec3d torqueBodyNewtonMeters) {
    }


    public record PropulsionSample(
        String type,
        Vec3d pointLocal,
        Vec3d windWorld,
        Vec3d axisWorld,
        double nativeInflow,
        double weatherEquivalentInflow,
        double physicalPointInflowMetersPerSecond,
        double ivSpeedFactor,
        double throttle,
        double actuatorRpm,
        double actuatorPitchDegrees,
        double desiredVelocity,
        double mtsForceValue,
        Vec3d forceWorldNewtons,
        Vec3d torqueBodyNewtonMeters
    ) {
    }
}
