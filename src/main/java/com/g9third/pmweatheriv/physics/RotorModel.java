package com.g9third.pmweatheriv.physics;

import java.util.ArrayList;
import java.util.List;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartPropeller;
import static com.g9third.pmweatheriv.physics.FlightMath.EPSILON;
import static com.g9third.pmweatheriv.physics.FlightMath.PointFlow;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTOR_DISC_SAMPLE_RADIUS_FRACTION;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTORCRAFT_TRANSLATION_CYCLIC_TILT_RADIANS;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTORCRAFT_CONTROL_CENTER_SLOPE;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTORCRAFT_THRUST_LINEAR_WEIGHT_MULTIPLIER;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTORCRAFT_THRUST_SOFT_MAX_WEIGHT_MULTIPLIER;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTORCRAFT_THRUST_RESPONSE_SECONDS;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTORCRAFT_FULL_CONTROL_WEIGHT_FRACTION;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTORCRAFT_MAX_PITCH_RATE_RADPS;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTORCRAFT_MAX_YAW_RATE_RADPS;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTORCRAFT_MAX_ROLL_RATE_RADPS;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTORCRAFT_RATE_RESPONSE_SECONDS;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTORCRAFT_MAX_PITCH_ACCEL_RADPS2;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTORCRAFT_MAX_YAW_ACCEL_RADPS2;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTORCRAFT_MAX_ROLL_ACCEL_RADPS2;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTORCRAFT_MAX_WIND_SHEAR_PITCH_ACCEL_RADPS2;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTORCRAFT_MAX_WIND_SHEAR_YAW_ACCEL_RADPS2;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTORCRAFT_MAX_WIND_SHEAR_ROLL_ACCEL_RADPS2;
import static com.g9third.pmweatheriv.physics.FlightMath.GRAVITY;
import static com.g9third.pmweatheriv.physics.FlightMath.activeAngularVelocityBody;
import static com.g9third.pmweatheriv.physics.FlightMath.activeTimeStepSeconds;
import static com.g9third.pmweatheriv.physics.FlightMath.advanceUnsteadyState;
import static com.g9third.pmweatheriv.physics.FlightMath.normalizedControl;
import static com.g9third.pmweatheriv.physics.FlightMath.point;
import static com.g9third.pmweatheriv.physics.FlightMath.pointFlow;
import static com.g9third.pmweatheriv.physics.FlightMath.toLocal;
import static com.g9third.pmweatheriv.physics.FlightMath.toWorld;

import static com.g9third.pmweatheriv.physics.PropulsionModel.PropulsionSample;
import static com.g9third.pmweatheriv.physics.PropulsionModel.ivForceNewtonsPerUnit;
import static com.g9third.pmweatheriv.physics.PropulsionModel.toIvPropellerInflow;

import static com.g9third.pmweatheriv.physics.AircraftWind.WindField;
import static com.g9third.pmweatheriv.physics.AirframeLoads.Accumulator;

/** Authored rotor actuators and the assisted rotorcraft controller. */
public final class RotorModel {
    private RotorModel() {}

    static List<PreparedRotorDisc> prepareRotorDiscs(EntityVehicleF_Physics vehicle) {
        List<PreparedRotorDisc> discs = new ArrayList<>();
        for (APart part : vehicle.allParts) {
            if (!(part instanceof PartPropeller propeller)
                || propeller.definition.propeller == null
                || !propeller.definition.propeller.isRotor) {
                continue;
            }
            Vec3d axisWorld = toWorld(vehicle, toLocal(vehicle.orientation, point(new Point3D(0.0, 0.0, 1.0).rotate(propeller.orientation)))).normalized();
            Vec3d axisBody = toLocal(vehicle, axisWorld).normalized();
            Vec3d basisU=toLocal(vehicle.orientation,point(new Point3D(1,0,0).rotate(propeller.orientation))).normalized();
            Vec3d basisV=toLocal(vehicle.orientation,point(new Point3D(0,1,0).rotate(propeller.orientation))).normalized();
            double authoredRadius=Math.max(0.01,propeller.definition.propeller.diameter*0.0254*0.5);
            double radiusU=authoredRadius*Math.abs(ModelCoordinates.scaleComponent(propeller.scale.x));
            double radiusV=authoredRadius*Math.abs(ModelCoordinates.scaleComponent(propeller.scale.y));
            double radiusMeters=Math.sqrt(radiusU*radiusV);
            boolean mainRotor=Math.abs(axisBody.y())>=0.55;
            Vec3d center=point(propeller.localOffset);
            discs.add(new PreparedRotorDisc(propeller,center,axisBody,basisU,basisV,radiusMeters,mainRotor,
                new Vec3d[] {center,
                    center.add(basisU.scale(radiusU*ROTOR_DISC_SAMPLE_RADIUS_FRACTION)),
                    center.subtract(basisU.scale(radiusU*ROTOR_DISC_SAMPLE_RADIUS_FRACTION)),
                    center.add(basisV.scale(radiusV*ROTOR_DISC_SAMPLE_RADIUS_FRACTION)),
                    center.subtract(basisV.scale(radiusV*ROTOR_DISC_SAMPLE_RADIUS_FRACTION))}));
        }
        return List.copyOf(discs);
    }

    static RotorLoadCandidate sampleRotorDisc(
        EntityVehicleF_Physics vehicle,
        PreparedRotorDisc disc,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField
    ) {
        List<DiscPointFlow> discFlows = sampleRotorDiscFlows(
            vehicle, disc, linearVelocityWorld, omegaWorld, windField
        );
        Vec3d axisWorld = toWorld(vehicle, disc.axisBody()).normalized();
        double inflowSum = 0.0;
        for (DiscPointFlow flow : discFlows) {
            inflowSum += flow.axialInflowMetersPerSecond();
        }
        double meanInflowMetersPerSecond = discFlows.isEmpty()
            ? 0.0 : inflowSum / discFlows.size();
        // IV remains the rotor actuator authority. 0.8.3di exposed an
        // authority-crossing bug here: substituting PMWeather's ambient axial
        // disc flow into IV's opaque propeller law can make a healthy powered
        // rotor return negative thrust in a strong downdraft. The later
        // positive-main-rotor gate then turns that into exactly zero lift.
        //
        // Keep the authored actuator scalar on IV's native inflow, exactly as
        // fixed-wing propulsion already does. PMWeather still enters the
        // rotorcraft physically through the five sampled disc flows below.
        // 0.11.0 uses those live flows only for a bounded disc-shear disturbance;
        // cyclic/pedals are handled by the helicopter body-rate controller.
        // The same PMWeather flow also reaches the body-pressure model. This is
        // separation of weather/actuator authority: PMWeather does not replace
        // IV's native inflow.
        double ivInflow = toIvPropellerInflow(vehicle, meanInflowMetersPerSecond);
        PartPropeller propeller = disc.propeller();
        double nativeInflow = propeller.airstreamLinearVelocity;
        Point3D forceMts = new Point3D();
        Point3D torqueMts = new Point3D();
        double value = propeller.addToForceOutput(forceMts, torqueMts);
        double propellerRpm = propeller.getOrCreateVariable("propeller_rpm").getValue();
        double propellerPitchDegrees = propeller.getOrCreateVariable("propeller_pitch_deg").getValue();
        double signedAxialThrustNewtons = value * ivForceNewtonsPerUnit(vehicle);
        Vec3d ownerTickWindWorld = discFlows.isEmpty()
            ? Vec3d.ZERO : discFlows.get(0).flow().windWorld();
        RotorActuatorSnapshot actuator = new RotorActuatorSnapshot(
            disc,
            nativeInflow,
            ivInflow,
            meanInflowMetersPerSecond,
            vehicle.speedFactor,
            vehicle.throttleVar.currentValue,
            propellerRpm,
            propellerPitchDegrees,
            propeller.desiredLinearVelocity,
            value,
            signedAxialThrustNewtons,
            ownerTickWindWorld,
            axisWorld
        );
        return new RotorLoadCandidate(actuator, List.copyOf(discFlows));
    }

    static List<DiscPointFlow> sampleRotorDiscFlows(
        EntityVehicleF_Physics vehicle,
        PreparedRotorDisc disc,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField
    ) {
        Vec3d axisWorld = toWorld(vehicle, disc.axisBody()).normalized();
        Vec3d[] points = disc.samplePoints();
        List<DiscPointFlow> discFlows = new ArrayList<>(points.length);
        for (int index = 0; index < points.length; ++index) {
            PointFlow flow = pointFlow(
                vehicle, points[index], linearVelocityWorld, omegaWorld,
                windField, "ROTOR_DISC_" + disc.propeller().uniqueUUID + "_" + index
            );
            discFlows.add(new DiscPointFlow(
                points[index], flow, flow.relativeAirWorld().dot(axisWorld)
            ));
        }
        return discFlows;
    }

    static RotorLoadCandidate resampleRotorDisc(
        EntityVehicleF_Physics vehicle,
        RotorActuatorSnapshot actuator,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField
    ) {
        return new RotorLoadCandidate(
            actuator,
            List.copyOf(sampleRotorDiscFlows(
                vehicle, actuator.disc(), linearVelocityWorld, omegaWorld, windField
            ))
        );
    }

    /**
     * 0.11.0 stabilized rotorcraft path.
     *
     * <p>IV remains the actuator source for rotor RPM, collective/pitch, and raw
     * available thrust. PMIV no longer treats the rotor tip-path-plane as a
     * free world-space orientation state. Instead, aggregate main-rotor thrust
     * is smoothed relative to vehicle weight, applied through the physical mast
     * direction at the vehicle CG, and cyclic/pedals command body angular rates
     * through an inertia-scaled controller with bounded disturbance trim. Releasing the controls commands zero
     * angular rate, not level attitude: there is no horizon, attitude, position,
     * or altitude hold.</p>
     *
     * <p>PMWeather still samples the hub plus four physical disc edge stations.
     * Their local flow differences are retained as a deliberately limited
     * disturbance moment so gusts and shear remain real without overpowering
     * pilot authority.</p>
     */
    static RotorApplication applyDistributedRotorLoads(
        EntityVehicleF_Physics vehicle,
        AircraftState state,
        double airDensity,
        List<RotorActuatorSnapshot> actuators,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        Accumulator accumulator
    ) {
        if (actuators == null || actuators.isEmpty()) {
            return RotorApplication.ZERO;
        }
        List<RotorLoadCandidate> candidates = new ArrayList<>(actuators.size());
        for (RotorActuatorSnapshot actuator : actuators) {
            if (actuator == null || actuator.disc() == null) {
                continue;
            }
            candidates.add(resampleRotorDisc(
                vehicle, actuator, linearVelocityWorld, omegaWorld, windField
            ));
        }
        return applyDistributedRotorCandidates(
            vehicle, state, airDensity, candidates, accumulator
        );
    }

    static RotorApplication applyDistributedRotorCandidates(
        EntityVehicleF_Physics vehicle,
        AircraftState state,
        double airDensity,
        List<RotorLoadCandidate> candidates,
        Accumulator accumulator
    ) {
        if (vehicle == null || state == null || state.plan == null
            || candidates == null || candidates.isEmpty() || accumulator == null) {
            return RotorApplication.ZERO;
        }

        List<RotorLoadCandidate> mainRotors = new ArrayList<>();
        for (RotorLoadCandidate candidate : candidates) {
            if (candidate != null && candidate.actuator() != null
                && candidate.actuator().disc() != null
                && candidate.actuator().disc().mainRotor()) {
                mainRotors.add(candidate);
            }
        }
        if (mainRotors.isEmpty()) {
            return RotorApplication.ZERO;
        }

        double rawTotalMainThrust = 0.0;
        for (RotorLoadCandidate candidate : mainRotors) {
            rawTotalMainThrust += positiveMainRotorThrust(candidate.actuator());
        }

        double targetMainThrust = stabilizedRotorThrustTarget(
            vehicle, rawTotalMainThrust
        );
        double totalMainThrust = filteredRotorThrust(
            vehicle, state, targetMainThrust
        );
        double rawScale = rawTotalMainThrust > EPSILON
            ? totalMainThrust / rawTotalMainThrust : 0.0;
        double residualPerRotor = rawTotalMainThrust > EPSILON || mainRotors.isEmpty()
            ? 0.0 : totalMainThrust / mainRotors.size();

        AirframeLoads.Controls controls = FlightMath.pointControls(vehicle);
        double aileron = normalizedControl(
            controls.aileron(), EntityVehicleF_Physics.MAX_AILERON_ANGLE
        );
        double elevator = normalizedControl(
            controls.elevator(), EntityVehicleF_Physics.MAX_ELEVATOR_ANGLE
        );
        double rudder = normalizedControl(
            controls.rudder(), EntityVehicleF_Physics.MAX_RUDDER_ANGLE
        );

        double appliedMagnitude = 0.0;
        for (RotorLoadCandidate candidate : mainRotors) {
            double raw = positiveMainRotorThrust(candidate.actuator());
            // If IV output drops to zero abruptly, retain the filtered lift state
            // and distribute its short physical decay evenly across the authored
            // main rotors. Otherwise a 0.30 s thrust state would control only the
            // rate authority while the actual lift vanished instantly.
            double thrust = rawTotalMainThrust > EPSILON
                ? raw * rawScale : residualPerRotor;
            if (!(thrust > EPSILON)) {
                addRotorPropulsionDiagnostic(
                    accumulator, candidate, "ROTOR_MAIN_STABILIZED",
                    Vec3d.ZERO, Vec3d.ZERO
                );
                continue;
            }

            PreparedRotorDisc disc = candidate.actuator().disc();
            Vec3d rawAxisBody = disc.axisBody().normalized();
            Vec3d liftAxisBody = rawAxisBody.scale(rawAxisBody.y() >= 0.0 ? 1.0 : -1.0);
            Vec3d thrustAxisBody = translationThrustAxisBody(
                liftAxisBody, elevator, aileron
            );
            Vec3d thrustAxisWorld = toWorld(vehicle, thrustAxisBody).normalized();
            Vec3d forceWorld = thrustAxisWorld.scale(thrust);

            // Deliberately apply the aggregate lifting force at the physical CG.
            // Body attitude still redirects the lift vector, while cyclic body
            // rotation is owned by the rate controller below. This removes the
            // unstable duplicate "hub lever + cyclic torque" control path.
            Vec3d cg = state.plan.centerOfMassLocal() == null
                ? Vec3d.ZERO : state.plan.centerOfMassLocal();
            PointFlow hubFlow = candidate.discFlows().isEmpty()
                ? new PointFlow(Vec3d.ZERO, Vec3d.ZERO, Vec3d.ZERO)
                : candidate.discFlows().get(0).flow();
            Vec3d forceTorque = accumulator.addRotorSurface(
                "ROTOR_MAIN_STABILIZED", cg, hubFlow, forceWorld,
                Math.PI * disc.radiusMeters() * disc.radiusMeters(),
                0.0, 0.0, vehicle
            );

            Vec3d disturbance = rotorWindShearDisturbanceTorqueBody(
                vehicle, candidate, thrust, airDensity, liftAxisBody
            );
            disturbance = limitRotorWindShearTorque(
                vehicle, state, disturbance
            );
            accumulator.addRotorStabilityTorque(disturbance);

            appliedMagnitude += forceWorld.length();

            addRotorPropulsionDiagnostic(
                accumulator, candidate, "ROTOR_MAIN_STABILIZED",
                forceWorld, forceTorque.add(disturbance)
            );
        }

        applyRotorcraftRateCommandController(
            vehicle, state, totalMainThrust, aileron, elevator, rudder, accumulator
        );

        return new RotorApplication(totalMainThrust, appliedMagnitude);
    }

    static double positiveMainRotorThrust(RotorActuatorSnapshot actuator) {
        if (actuator == null || actuator.disc() == null) {
            return 0.0;
        }
        Vec3d axisBody = actuator.disc().axisBody().normalized();
        double upwardSign = axisBody.y() >= 0.0 ? 1.0 : -1.0;
        return Math.max(0.0, actuator.signedAxialThrustNewtons() * upwardSign);
    }

    /**
     * Converts IV's native rotor output to the 0.11.0 helicopter lift envelope.
     * Hover and ordinary climb authority remain linear through 1.30x weight;
     * larger opaque IV spikes are smoothly compressed toward 2.00x weight.
     */
    static double stabilizedRotorThrustTarget(
        EntityVehicleF_Physics vehicle,
        double rawTotalThrustNewtons
    ) {
        double raw = Math.max(0.0, Double.isFinite(rawTotalThrustNewtons)
            ? rawTotalThrustNewtons : 0.0);
        double mass = vehicle != null && Double.isFinite(vehicle.currentMass)
            ? Math.max(50.0, vehicle.currentMass) : 50.0;
        double weight = mass * GRAVITY;
        double onset = weight * ROTORCRAFT_THRUST_LINEAR_WEIGHT_MULTIPLIER;
        double maximum = weight * ROTORCRAFT_THRUST_SOFT_MAX_WEIGHT_MULTIPLIER;
        if (raw <= onset || !(maximum > onset + EPSILON)) {
            return raw;
        }
        double excess = raw - onset;
        double headroom = maximum - onset;
        return onset + excess / (1.0 + excess / headroom);
    }

    /**
     * Low-pass rotor thrust state. Only real Sable substeps mutate it; owner
     * preview/diagnostic solves reuse the same state and cannot double-advance it.
     */
    static double filteredRotorThrust(
        EntityVehicleF_Physics vehicle,
        AircraftState state,
        double targetThrustNewtons
    ) {
        double target = Math.max(0.0, Double.isFinite(targetThrustNewtons)
            ? targetThrustNewtons : 0.0);
        if (state == null) {
            return target;
        }
        double previous = state.rotorcraftMainThrustNewtons;
        if (!Double.isFinite(previous) || previous < 0.0) {
            previous = target;
        }
        if (!advanceUnsteadyState()) {
            return previous;
        }
        double dt = activeTimeStepSeconds();
        if (!(dt > 0.0) || !Double.isFinite(dt)) {
            state.rotorcraftMainThrustNewtons = previous;
            return previous;
        }
        double alpha = 1.0 - Math.exp(
            -dt / Math.max(EPSILON, ROTORCRAFT_THRUST_RESPONSE_SECONDS)
        );
        double updated = previous + (target - previous) * alpha;
        if (target <= EPSILON && updated < 1.0) {
            updated = 0.0;
        }
        state.rotorcraftMainThrustNewtons = Math.max(0.0, updated);
        return state.rotorcraftMainThrustNewtons;
    }

    static double shapeRotorControl(double control) {
        double x = Vec3d.clamp(Double.isFinite(control) ? control : 0.0, -1.0, 1.0);
        return x * (ROTORCRAFT_CONTROL_CENTER_SLOPE
            + (1.0 - ROTORCRAFT_CONTROL_CENTER_SLOPE) * x * x);
    }

    /**
     * Small direct thrust-vector translation cue. Most horizontal acceleration
     * comes from the body attitude produced by the rate controller; this small
     * component keeps initial cyclic response from feeling delayed.
     */
    static Vec3d translationThrustAxisBody(
        Vec3d liftAxisBody,
        double elevatorControl,
        double aileronControl
    ) {
        Vec3d lift = liftAxisBody == null || !liftAxisBody.isFinite()
            ? new Vec3d(0.0, 1.0, 0.0) : liftAxisBody.normalized();
        if (lift.lengthSquared() <= EPSILON) {
            lift = new Vec3d(0.0, 1.0, 0.0);
        }
        double elevator = shapeRotorControl(elevatorControl);
        double aileron = shapeRotorControl(aileronControl);
        double magnitude = Math.min(1.0, Math.hypot(elevator, aileron));
        if (magnitude <= 1.0E-6) {
            return lift;
        }
        Vec3d requested = new Vec3d(-aileron, 0.0, -elevator);
        requested = requested.subtract(lift.scale(requested.dot(lift)));
        Vec3d tangent = requested.normalized();
        if (tangent.lengthSquared() <= EPSILON) {
            return lift;
        }
        double tilt = ROTORCRAFT_TRANSLATION_CYCLIC_TILT_RADIANS * magnitude;
        return lift.scale(Math.cos(tilt)).add(tangent.scale(Math.sin(tilt))).normalized();
    }

    static double commandedTranslationTiltDegrees(
        EntityVehicleF_Physics vehicle,
        PreparedRotorDisc disc
    ) {
        if (vehicle == null || disc == null) {
            return 0.0;
        }
        AirframeLoads.Controls controls = FlightMath.pointControls(vehicle);
        double aileron = normalizedControl(
            controls.aileron(), EntityVehicleF_Physics.MAX_AILERON_ANGLE
        );
        double elevator = normalizedControl(
            controls.elevator(), EntityVehicleF_Physics.MAX_ELEVATOR_ANGLE
        );
        Vec3d rawAxis = disc.axisBody().normalized();
        Vec3d lift = rawAxis.scale(rawAxis.y() >= 0.0 ? 1.0 : -1.0);
        Vec3d commanded = translationThrustAxisBody(lift, elevator, aileron);
        double dot = Vec3d.clamp(lift.normalized().dot(commanded.normalized()), -1.0, 1.0);
        return Math.toDegrees(Math.acos(dot));
    }

    static Vec3d commandedBodyRateRadps(
        double aileronControl,
        double elevatorControl,
        double rudderControl
    ) {
        double rollInput = shapeRotorControl(aileronControl);
        double pitchInput = shapeRotorControl(elevatorControl);
        double yawInput = shapeRotorControl(rudderControl);
        return new Vec3d(
            -pitchInput * ROTORCRAFT_MAX_PITCH_RATE_RADPS,
            -yawInput * ROTORCRAFT_MAX_YAW_RATE_RADPS,
             rollInput * ROTORCRAFT_MAX_ROLL_RATE_RADPS
        );
    }

    static Vec3d commandedBodyRateRadps(EntityVehicleF_Physics vehicle) {
        if (vehicle == null) {
            return Vec3d.ZERO;
        }
        AirframeLoads.Controls controls = FlightMath.pointControls(vehicle);
        return commandedBodyRateRadps(
            normalizedControl(controls.aileron(), EntityVehicleF_Physics.MAX_AILERON_ANGLE),
            normalizedControl(controls.elevator(), EntityVehicleF_Physics.MAX_ELEVATOR_ANGLE),
            normalizedControl(controls.rudder(), EntityVehicleF_Physics.MAX_RUDDER_ANGLE)
        );
    }

    /**
     * Rate-command flight-control system used only for rotorcraft.
     *
     * <p>Full cyclic commands finite roll/pitch rates rather than an unbounded
     * force couple. Zero stick commands zero angular rate, so the aircraft stops
     * rotating but remains at whatever attitude it has reached. Torque is
     * inertia-scaled, giving comparable handling across content packs without
     * per-aircraft tuning.</p>
     */
    static void applyRotorcraftRateCommandController(
        EntityVehicleF_Physics vehicle,
        AircraftState state,
        double mainThrustNewtons,
        double aileronControl,
        double elevatorControl,
        double rudderControl,
        Accumulator accumulator
    ) {
        if (vehicle == null || state == null || state.plan == null
            || accumulator == null) {
            return;
        }
        if (!(mainThrustNewtons > EPSILON)) {
            if (advanceUnsteadyState()) state.rotorcraftRateTrimAlphaBody = Vec3d.ZERO;
            return;
        }

        double mass = Double.isFinite(vehicle.currentMass)
            ? Math.max(50.0, vehicle.currentMass) : 50.0;
        double weight = mass * GRAVITY;
        double authority = Vec3d.clamp(
            mainThrustNewtons / Math.max(EPSILON, weight * ROTORCRAFT_FULL_CONTROL_WEIGHT_FRACTION),
            0.0, 1.0
        );
        if (authority <= EPSILON) {
            return;
        }

        Vec3d targetOmega = commandedBodyRateRadps(
            aileronControl, elevatorControl, rudderControl
        );
        Vec3d omega = activeAngularVelocityBody(state.angularVelocityBody);
        Vec3d error = targetOmega.subtract(omega);

        Vec3d trim = state.rotorcraftRateTrimAlphaBody;
        if (trim == null || !trim.isFinite()) trim = Vec3d.ZERO;
        if (advanceUnsteadyState()) {
            if (state.rotorcraftGroundSupported || authority<0.5) {
                trim = Vec3d.ZERO; // No ground/low-RPM wind-up.
            } else {
                double dt = activeTimeStepSeconds();
                trim = new Vec3d(
                    rateTrimAxis(trim.x(),error.x(),elevatorControl,dt,
                        Math.toRadians(6),ROTORCRAFT_MAX_PITCH_ACCEL_RADPS2),
                    rateTrimAxis(trim.y(),error.y(),rudderControl,dt,
                        Math.toRadians(4),ROTORCRAFT_MAX_YAW_ACCEL_RADPS2),
                    rateTrimAxis(trim.z(),error.z(),aileronControl,dt,
                        Math.toRadians(8),ROTORCRAFT_MAX_ROLL_ACCEL_RADPS2));
            }
            state.rotorcraftRateTrimAlphaBody = trim;
        }
        Vec3d desiredAlpha = new Vec3d(
            Vec3d.clamp(
                error.x() / ROTORCRAFT_RATE_RESPONSE_SECONDS + trim.x(),
                -ROTORCRAFT_MAX_PITCH_ACCEL_RADPS2,
                 ROTORCRAFT_MAX_PITCH_ACCEL_RADPS2
            ),
            Vec3d.clamp(
                error.y() / ROTORCRAFT_RATE_RESPONSE_SECONDS + trim.y(),
                -ROTORCRAFT_MAX_YAW_ACCEL_RADPS2,
                 ROTORCRAFT_MAX_YAW_ACCEL_RADPS2
            ),
            Vec3d.clamp(
                error.z() / ROTORCRAFT_RATE_RESPONSE_SECONDS + trim.z(),
                -ROTORCRAFT_MAX_ROLL_ACCEL_RADPS2,
                 ROTORCRAFT_MAX_ROLL_ACCEL_RADPS2
            )
        ).scale(authority);

        Vec3d inertia = state.plan.geometry().inertiaForMass(mass);
        Vec3d torque = new Vec3d(
            inertia.x() * desiredAlpha.x(),
            inertia.y() * desiredAlpha.y(),
            inertia.z() * desiredAlpha.z()
        );
        accumulator.addRotorStabilityTorque(torque);
    }

    /** Bounded slow disturbance trim for centered rate commands; no attitude target. */
    private static double rateTrimAxis(double previous, double error, double input,
            double dt, double trimLimit, double accelerationLimit) {
        if (!(dt>0) || !Double.isFinite(dt) || Math.abs(input)>0.02) return previous;
        double requested=error/ROTORCRAFT_RATE_RESPONSE_SECONDS+previous;
        if (Math.abs(requested)>=accelerationLimit && Math.signum(error)==Math.signum(requested))
            return previous; // Conditional integration prevents saturation wind-up.
        double delta=error*dt/(ROTORCRAFT_RATE_RESPONSE_SECONDS*2.0);
        return Vec3d.clamp(previous+delta,-trimLimit,trimLimit);
    }

    /**
     * Retains real PMWeather/relative-flow differences across the physical rotor
     * disc as a limited disturbance torque. Net lift is not changed here.
     */
    static Vec3d rotorWindShearDisturbanceTorqueBody(
        EntityVehicleF_Physics vehicle,
        RotorLoadCandidate candidate,
        double totalThrustNewtons,
        double airDensity,
        Vec3d liftAxisBody
    ) {
        if (candidate == null || candidate.actuator() == null
            || candidate.actuator().disc() == null
            || candidate.discFlows() == null || candidate.discFlows().size() < 5
            || !(totalThrustNewtons > EPSILON)) {
            return Vec3d.ZERO;
        }
        PreparedRotorDisc disc = candidate.actuator().disc();
        List<RotorcraftAerodynamics.EdgeKinematics> edges = new ArrayList<>(4);
        for (int index = 1; index <= 4; ++index) {
            DiscPointFlow edge = candidate.discFlows().get(index);
            edges.add(new RotorcraftAerodynamics.EdgeKinematics(
                edge.pointLocal().subtract(disc.centerLocal()),
                toLocal(vehicle, edge.flow().relativeAirWorld())
            ));
        }

        double[] edgeForces = RotorcraftAerodynamics.distributeMainRotorForces(
            totalThrustNewtons,
            airDensity,
            candidate.actuator().actuatorRpm(),
            candidate.actuator().actuatorPitchDegrees(),
            disc.axisBody(),
            liftAxisBody,
            edges
        );
        // Compare against the same rotor in uniform hub-relative flow. The
        // blade-speed proxy has an advancing/retreating imbalance even in a
        // uniform breeze or steady forward flight; that is not spatial shear.
        // Keep actual edge-minus-hub flow (including omega x radial) so gusts
        // and rotational aerodynamic response still produce physical moments.
        Vec3d uniformFlow = toLocal(vehicle,
            candidate.discFlows().get(0).flow().relativeAirWorld());
        List<RotorcraftAerodynamics.EdgeKinematics> baseline = new ArrayList<>(4);
        for (RotorcraftAerodynamics.EdgeKinematics edge : edges)
            baseline.add(new RotorcraftAerodynamics.EdgeKinematics(edge.radialBody(), uniformFlow));
        double[] baselineForces = RotorcraftAerodynamics.distributeMainRotorForces(
            totalThrustNewtons, airDensity, candidate.actuator().actuatorRpm(),
            candidate.actuator().actuatorPitchDegrees(), disc.axisBody(), liftAxisBody, baseline);
        Vec3d torque = Vec3d.ZERO;
        for (int index = 0; index < 4; ++index) {
            Vec3d radial = edges.get(index).radialBody();
            double delta = edgeForces[index] - baselineForces[index];
            torque = torque.add(radial.cross(liftAxisBody.scale(delta)));
        }
        return torque;
    }

    /**
     * Converts rotor-disc differential loading into a bounded angular disturbance.
     * The bound is inertia-relative rather than a fixed torque, so large and small
     * content-pack helicopters experience a comparable gust-induced angular kick.
     * Generic body pressure remains untouched and continues to respond to PMWeather.
     */
    static Vec3d limitRotorWindShearTorque(
        EntityVehicleF_Physics vehicle,
        AircraftState state,
        Vec3d torqueBody
    ) {
        if (torqueBody == null || !torqueBody.isFinite() || state == null
            || state.plan == null) {
            return Vec3d.ZERO;
        }
        double mass = vehicle != null && Double.isFinite(vehicle.currentMass)
            ? Math.max(50.0, vehicle.currentMass) : 50.0;
        Vec3d inertia = state.plan.geometry().inertiaForMass(mass);
        return new Vec3d(
            Vec3d.clamp(
                torqueBody.x(),
                -inertia.x() * ROTORCRAFT_MAX_WIND_SHEAR_PITCH_ACCEL_RADPS2,
                 inertia.x() * ROTORCRAFT_MAX_WIND_SHEAR_PITCH_ACCEL_RADPS2
            ),
            Vec3d.clamp(
                torqueBody.y(),
                -inertia.y() * ROTORCRAFT_MAX_WIND_SHEAR_YAW_ACCEL_RADPS2,
                 inertia.y() * ROTORCRAFT_MAX_WIND_SHEAR_YAW_ACCEL_RADPS2
            ),
            Vec3d.clamp(
                torqueBody.z(),
                -inertia.z() * ROTORCRAFT_MAX_WIND_SHEAR_ROLL_ACCEL_RADPS2,
                 inertia.z() * ROTORCRAFT_MAX_WIND_SHEAR_ROLL_ACCEL_RADPS2
            )
        );
    }

    static void addRotorPropulsionDiagnostic(
        Accumulator accumulator,
        RotorLoadCandidate candidate,
        String type,
        Vec3d forceWorld,
        Vec3d torqueBody
    ) {
        RotorActuatorSnapshot actuator = candidate.actuator();
        // For rotor records, axisWorld is the actual stabilized resultant-thrust
        // direction whenever force exists. At zero thrust retain the owner-tick
        // mast axis; no independent world-space disc orientation exists in 0.11.0.
        Vec3d axisWorld = forceWorld != null && forceWorld.isFinite()
            && forceWorld.lengthSquared() > EPSILON
                ? forceWorld.normalized() : actuator.ownerTickAxisWorld();
        accumulator.recordPropulsionLoad(type, actuator.rawMtsForceValue(), forceWorld, torqueBody);
        if (accumulator.capturePropulsion && accumulator.captureSurfaces) accumulator.propulsionSamples.add(new PropulsionSample(
            type,
            actuator.disc().centerLocal(),
            actuator.ownerTickWindWorld(),
            axisWorld,
            actuator.nativeInflow(),
            actuator.temporaryWeatherInflow(),
            actuator.physicalPointInflowMetersPerSecond(),
            actuator.ivSpeedFactor(),
            actuator.throttle(),
            actuator.actuatorRpm(),
            actuator.actuatorPitchDegrees(),
            actuator.desiredVelocity(),
            actuator.rawMtsForceValue(),
            forceWorld,
            torqueBody
        ));
    }

    public record PreparedRotorDisc(
        PartPropeller propeller,
        Vec3d centerLocal,
        Vec3d axisBody,
        Vec3d basisULocal,
        Vec3d basisVLocal,
        double radiusMeters,
        boolean mainRotor,
        Vec3d[] samplePoints
    ) {
        public PreparedRotorDisc(PartPropeller propeller, Vec3d centerLocal, Vec3d axisBody,
                Vec3d basisULocal, Vec3d basisVLocal, double radiusMeters, boolean mainRotor) {
            this(propeller, centerLocal, axisBody, basisULocal, basisVLocal, radiusMeters, mainRotor,
                new Vec3d[] {centerLocal,
                    centerLocal.add(basisULocal.scale(radiusMeters * ROTOR_DISC_SAMPLE_RADIUS_FRACTION)),
                    centerLocal.subtract(basisULocal.scale(radiusMeters * ROTOR_DISC_SAMPLE_RADIUS_FRACTION)),
                    centerLocal.add(basisVLocal.scale(radiusMeters * ROTOR_DISC_SAMPLE_RADIUS_FRACTION)),
                    centerLocal.subtract(basisVLocal.scale(radiusMeters * ROTOR_DISC_SAMPLE_RADIUS_FRACTION))});
        }
    }

    public record DiscPointFlow(
        Vec3d pointLocal,
        PointFlow flow,
        double axialInflowMetersPerSecond
    ) {
    }

    public record RotorActuatorSnapshot(
        PreparedRotorDisc disc,
        double nativeInflow,
        double temporaryWeatherInflow,
        double physicalPointInflowMetersPerSecond,
        double ivSpeedFactor,
        double throttle,
        double actuatorRpm,
        double actuatorPitchDegrees,
        double desiredVelocity,
        double rawMtsForceValue,
        double signedAxialThrustNewtons,
        Vec3d ownerTickWindWorld,
        Vec3d ownerTickAxisWorld
    ) {
    }

    public record RotorLoadCandidate(
        RotorActuatorSnapshot actuator,
        List<DiscPointFlow> discFlows
    ) {
    }

    public record RotorApplication(
        double mainRotorThrustNewtons,
        double appliedForceMagnitudeNewtons
    ) {
        static final RotorApplication ZERO = new RotorApplication(0.0, 0.0);
    }
}
