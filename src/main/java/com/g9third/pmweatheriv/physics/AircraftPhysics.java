package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import com.g9third.pmweatheriv.mixin.EntityVehiclePhysicsAccessor;
import java.util.ArrayList;
import java.util.List;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartEngine;
import minecrafttransportsimulator.entities.instances.PartPropeller;
import net.minecraft.world.level.Level;
import static com.g9third.pmweatheriv.physics.FlightMath.EPSILON;
import static com.g9third.pmweatheriv.physics.FlightMath.GRAVITY;
import static com.g9third.pmweatheriv.physics.FlightMath.MC_TICK_SECONDS;
import static com.g9third.pmweatheriv.physics.FlightMath.SUBSTEP_EVALUATION;
import static com.g9third.pmweatheriv.physics.FlightMath.SubstepEvaluationContext;
import static com.g9third.pmweatheriv.physics.FlightMath.activePositionWorld;
import static com.g9third.pmweatheriv.physics.FlightMath.finiteClamp;
import static com.g9third.pmweatheriv.physics.FlightMath.ivCompatibleAirDensity;
import static com.g9third.pmweatheriv.physics.FlightMath.point;
import static com.g9third.pmweatheriv.physics.FlightMath.pointControls;
import static com.g9third.pmweatheriv.physics.FlightMath.signedForwardDenominator;
import static com.g9third.pmweatheriv.physics.FlightMath.toLocal;
import static com.g9third.pmweatheriv.physics.FlightMath.toWorld;
import static com.g9third.pmweatheriv.physics.AirframePreparation.prepareAircraftOnce;
import static com.g9third.pmweatheriv.physics.BodySurfaceAdapter.addBodyPressure;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceAdapter.WingRuntimeGeometry;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceAdapter.addFixedWingAerodynamics;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceAdapter.wingRuntimeGeometry;
import static com.g9third.pmweatheriv.physics.RotorModel.PreparedRotorDisc;
import static com.g9third.pmweatheriv.physics.RotorModel.RotorActuatorSnapshot;
import static com.g9third.pmweatheriv.physics.RotorModel.RotorApplication;
import static com.g9third.pmweatheriv.physics.RotorModel.RotorLoadCandidate;
import static com.g9third.pmweatheriv.physics.RotorModel.applyDistributedRotorCandidates;
import static com.g9third.pmweatheriv.physics.RotorModel.applyDistributedRotorLoads;
import static com.g9third.pmweatheriv.physics.RotorModel.sampleRotorDisc;
import static com.g9third.pmweatheriv.physics.RotorModel.sampleRotorDiscFlows;
import static com.g9third.pmweatheriv.physics.PropulsionModel.PropulsionLoad;
import static com.g9third.pmweatheriv.physics.PropulsionModel.addJetEngine;
import static com.g9third.pmweatheriv.physics.PropulsionModel.addPropeller;
import static com.g9third.pmweatheriv.physics.PropulsionModel.ivForceNewtonsPerUnit;

import static com.g9third.pmweatheriv.physics.AircraftWind.WindField;
import static com.g9third.pmweatheriv.physics.AircraftWind.WindFieldSnapshot;
import static com.g9third.pmweatheriv.physics.AircraftWind.WindSample;
import static com.g9third.pmweatheriv.physics.AirframeLoads.Accumulator;
import static com.g9third.pmweatheriv.physics.AirframeLoads.SolveResult;
import static com.g9third.pmweatheriv.physics.AirframeLoads.SubstepAerodynamicLoads;
import static com.g9third.pmweatheriv.physics.AirframeLoads.SurfaceSample;
import static com.g9third.pmweatheriv.physics.AirframeGeometry.Geometry;
import static com.g9third.pmweatheriv.physics.AirframeGeometry.SurfaceAreaPlan;

/** Evaluates owner-tick and substep aerodynamic loads for managed aircraft. */
public final class AircraftPhysics {
    private AircraftPhysics() {}

    /**
     * Minecraft-tick actuator/wind snapshot for a persistent Sable body. Both
     * fixed-wing and rotorcraft aerodynamics are evaluated again from the live
     * rigid pose on every configured Sable substep. This owner-tick pass is
     * still the only place that queries PMWeather or advances IV engine/rotor
     * actuator state.
     */
    public static SolveResult calculateSableTickLoads(
        EntityVehicleF_Physics vehicle,
        Level level,
        PMWeatherIVConfig.Values config,
        AircraftState state
    ) {
        prepareAircraftOnce(vehicle, level, config, state);
        if (state.plan.rotorcraft()) {
            AirframePlan plan=state.plan;
            state.plan=new AirframePlan(plan.model(),plan.geometry(),plan.areaPlan(),plan.fixedWingPlan(),
                plan.inertia(),plan.centerOfMassLocal(),true,RotorModel.prepareRotorDiscs(vehicle),plan.pressurePatches());
        }
        SubstepEvaluationContext previous = SUBSTEP_EVALUATION.get();
        AircraftKinematics physical = state.kinematics;
        SUBSTEP_EVALUATION.set(physical == null || physical.gameTime() != level.getGameTime()
            ? SubstepEvaluationContext.previewOnly()
            : new SubstepEvaluationContext(physical.modelOriginWorld(), physical.orientation(),
                physical.centerVelocityWorld(), physical.angularVelocityBody(), MC_TICK_SECONDS, false));
        try {
            return solve(vehicle, level, config, state);
        } finally {
            restoreSubstepContext(previous);
        }
    }

    /**
     * Re-evaluates body and lifting/rotor aerodynamic loads from the current
     * Sable rigid pose and momentum. PMWeather is never queried here: the
     * immutable owner-tick field is reused. IV engine/propeller state remains
     * 20 Hz authority, while the resulting actuator magnitude is distributed
     * across the live rotor disc at the configured Sable substep rate.
     */
    public static SubstepAerodynamicLoads calculateAirVehicleSubstepAerodynamics(
        EntityVehicleF_Physics vehicle,
        Level level,
        PMWeatherIVConfig.Values config,
        AircraftState state,
        Vec3d positionWorld,
        RotationMatrix orientation,
        Vec3d linearVelocityWorld,
        Vec3d angularVelocityBody,
        WindFieldSnapshot windSnapshot,
        List<RotorActuatorSnapshot> rotorActuators,
        double timeStepSeconds
    ) {
        if (vehicle == null || level == null || config == null || state == null
            || !state.isPrepared() || vehicle.definition.motorized.isBlimp
            || positionWorld == null || !positionWorld.isFinite()
            || orientation == null
            || linearVelocityWorld == null || !linearVelocityWorld.isFinite()
            || angularVelocityBody == null || !angularVelocityBody.isFinite()
            || windSnapshot == null
            || !Double.isFinite(timeStepSeconds) || timeStepSeconds <= 0.0) {
            return SubstepAerodynamicLoads.ZERO;
        }

        state.pendingSeparation.clear();
        SubstepEvaluationContext previous = SUBSTEP_EVALUATION.get();
        SUBSTEP_EVALUATION.set(new SubstepEvaluationContext(
            positionWorld, orientation, linearVelocityWorld, angularVelocityBody,
            timeStepSeconds, true
        ));
        try {
            Geometry geometry = state.plan.geometry();
            ModelSurfaceMap.PreparedModel model = state.plan.model();
            SurfaceAreaPlan areaPlan = state.plan.areaPlan();
            double density;
            if (state.plan.rotorcraft()) {
                double altitude = Math.max(-1000.0, positionWorld.y() - vehicle.seaLevel);
                density = Vec3d.clamp(1.225 * Math.exp(-altitude / 8500.0), 0.08, 1.45);
            } else {
                density = ivCompatibleAirDensity(level, vehicle, positionWorld.y());
            }
            Vec3d omegaWorld = toWorld(vehicle, angularVelocityBody);
            WindField windField = new WindField(vehicle, windSnapshot);
            Accumulator accumulator = new Accumulator(state, com.g9third.pmweatheriv.devsupport.PMIVObserver.isCapturing(vehicle.uniqueUUID), false);
            addBodyPressure(
                vehicle, geometry, model, density, linearVelocityWorld, omegaWorld,
                windField, accumulator, config
            );
            if (!vehicle.outOfHealth) {
                if (state.plan.rotorcraft()) {
                    applyDistributedRotorLoads(
                        vehicle, state, density,
                        rotorActuators == null ? List.of() : rotorActuators,
                        linearVelocityWorld, omegaWorld, windField, accumulator
                    );
                } else {
                    addFixedWingAerodynamics(
                        vehicle, geometry, model, areaPlan, density, linearVelocityWorld,
                        omegaWorld, state, windField, accumulator, config
                    );
                }
            }
            if (!accumulator.forceWorld.isFinite() || !accumulator.torqueBody.isFinite()) {
                throw new IllegalStateException("Nonfinite aerodynamic load");
            }
            state.surfaceSeparationFractions.putAll(state.pendingSeparation);
            WindSample centerWind = windField.initializeCenter();
            return new SubstepAerodynamicLoads(
                accumulator.forceWorld,
                accumulator.torqueBody,
                accumulator.componentPressureTorqueBody,
                accumulator.meanMainWingLiftCoefficient,
                accumulator.wingDownwashRadians,
                centerWind,
                List.copyOf(accumulator.surfaceSamples)
            );
        } finally {
            state.pendingSeparation.clear();
            restoreSubstepContext(previous);
        }
    }

    static void restoreSubstepContext(SubstepEvaluationContext previous) {
        if (previous == null) {
            SUBSTEP_EVALUATION.remove();
        } else {
            SUBSTEP_EVALUATION.set(previous);
        }
    }

    static SolveResult solve(
        EntityVehicleF_Physics vehicle,
        Level level,
        PMWeatherIVConfig.Values config,
        AircraftState state
    ) {
        prepareAircraftOnce(vehicle, level, config, state);
        boolean rotorcraft = state.plan.rotorcraft();
        ModelSurfaceMap.PreparedModel model = state.plan.model();
        Geometry geometry = state.plan.geometry();
        SurfaceAreaPlan areaPlan = state.plan.areaPlan();
        double mass = finiteClamp(vehicle.currentMass, 50.0, 1.0E8, Math.max(50.0, vehicle.definition.motorized.emptyMass));
        // Mesh topology and mass distribution policy are frozen at preparation.
        // A detected rigid swing-wing animation is the one intentional geometry
        // exception: its existing wing patches are transformed by the live IV
        // root rotation, and the same projected span updates Rapier inertia in
        // place. Nothing is remeshed/reclassified in flight.
        WingRuntimeGeometry runtimeWing = wingRuntimeGeometry(
            vehicle, model, areaPlan, geometry, state.plan.fixedWingPlan()
        );
        Vec3d inertia = geometry.inertiaForMassAndWingSpan(mass, runtimeWing.wingSpan());

        double altitude = Math.max(-1000.0, activePositionWorld(vehicle).y() - vehicle.seaLevel);
        boolean ivCompatibleFixedWing = !rotorcraft && !vehicle.definition.motorized.isBlimp;
        double airDensity = ivCompatibleFixedWing
            ? ivCompatibleAirDensity(level, vehicle, activePositionWorld(vehicle).y())
            : Vec3d.clamp(1.225 * Math.exp(-altitude / 8500.0), 0.08, 1.45);
        vehicle.airDensity = airDensity;

        double ivVelocityScale = Math.max(EPSILON, vehicle.speedFactor * 20.0);
        Vec3d linearVelocityWorld = state.hasPhysicalVelocity ? state.originVelocityWorld
            : point(vehicle.motion).scale(ivVelocityScale);
        SubstepEvaluationContext activeEvaluation = SUBSTEP_EVALUATION.get();
        Vec3d omegaBodyBefore = activeEvaluation != null
            && activeEvaluation.angularVelocityBody() != null
                ? activeEvaluation.angularVelocityBody()
                : state.angularVelocityBody;
        Vec3d omegaWorld = toWorld(vehicle, omegaBodyBefore);
        // IV's public motion is the velocity of the rendered/model origin, while
        // the persistent Rapier body now translates at the physical COM. Convert
        // the owner-tick/compatibility path to COM velocity before evaluating
        // V + omega x (point-COM). A true Sable substep already supplies COM
        // velocity through SubstepEvaluationContext and must not be converted twice.
        boolean suppliedCenterOfMassVelocity = activeEvaluation != null
            && activeEvaluation.linearVelocityWorld() != null;
        Vec3d aerodynamicLinearVelocityWorld = suppliedCenterOfMassVelocity
            ? activeEvaluation.linearVelocityWorld() : linearVelocityWorld;
        if (!suppliedCenterOfMassVelocity
            && state.plan.centerOfMassLocal() != null
            && state.plan.centerOfMassLocal().isFinite()
            && state.plan.centerOfMassLocal().lengthSquared() > 1.0E-18) {
            aerodynamicLinearVelocityWorld = linearVelocityWorld.add(
                omegaWorld.cross(toWorld(vehicle, state.plan.centerOfMassLocal()))
            );
        }

        WindField windField;
        if (state.windSnapshot != null && state.windGameTime == level.getGameTime()) {
            windField = new WindField(vehicle, state.windSnapshot);
        } else {
            windField = new WindField(vehicle);
            Accumulator collecting = new Accumulator(state, false, false);
            addBodyPressure(vehicle, geometry, model, airDensity, aerodynamicLinearVelocityWorld,
                omegaWorld, windField, collecting, config);
            if (!rotorcraft && !vehicle.outOfHealth) {
                addFixedWingAerodynamics(vehicle, geometry, model, areaPlan, airDensity,
                    aerodynamicLinearVelocityWorld, omegaWorld, state, windField, collecting, config);
            }
            for (PreparedRotorDisc disc : state.plan.rotorDiscs()) {
                if (vehicle.allParts.contains(disc.propeller())) {
                    sampleRotorDiscFlows(vehicle, disc, aerodynamicLinearVelocityWorld, omegaWorld, windField);
                }
            }
            for (APart part : vehicle.allParts) {
                if (part instanceof PartPropeller propeller && !propeller.definition.propeller.isRotor) {
                    windField.register("PROP_" + propeller.uniqueUUID, point(propeller.localOffset));
                } else if (part instanceof PartEngine engine && engine.definition.engine != null
                    && engine.definition.engine.jetPowerFactor > 0.0F) {
                    windField.register("JET_" + engine.uniqueUUID, point(engine.localOffset));
                }
            }
            windField.resolve((net.minecraft.server.level.ServerLevel) level, state);
        }
        WindSample centerWind = windField.initializeCenter();
        Vec3d centerRelativeAirWorld = aerodynamicLinearVelocityWorld.subtract(centerWind.windMetersPerSecond());
        Vec3d centerRelativeAirBody = toLocal(vehicle, centerRelativeAirWorld);
        double forwardAirspeed = centerRelativeAirBody.z();
        double trueAirspeed = centerRelativeAirWorld.length();
        // MTS calls its vertical flight-path/AoA quantity trackAngle; preserve that
        // semantic because IV variables/animations may read it even though the
        // native aircraft force law is cancelled. Record horizontal sideslip
        // separately from the body-X component of the same air-relative vector.
        double trackAngleDegrees = Math.toDegrees(Math.atan2(
            -centerRelativeAirBody.y(), signedForwardDenominator(centerRelativeAirBody.z())
        ));
        double sideslipDegrees = trueAirspeed > EPSILON
            ? Math.toDegrees(Math.asin(Math.max(
                -1.0, Math.min(1.0, centerRelativeAirBody.x() / trueAirspeed)
            )))
            : 0.0;
        vehicle.axialVelocity = Math.abs(forwardAirspeed) / ivVelocityScale;
        vehicle.indicatedSpeed = Math.abs(forwardAirspeed);

        EntityVehiclePhysicsAccessor vehicleAccessor = (EntityVehiclePhysicsAccessor) vehicle;
        vehicleAccessor.pmweatherIv$setTrackAngle(trackAngleDegrees);

        // All distributed forces, rotational point velocities, Sable inertia,
        // and landing-gear/tire constraints use the same
        // vehicle-local rigid-body origin. A separate wing-centroid datum made
        // static and dynamic moments disagree and could understate or reverse
        // aerodynamic rate damping on arbitrary content-pack aircraft.
        Accumulator accumulator = new Accumulator(state,
            com.g9third.pmweatheriv.devsupport.PMIVObserver.needsDetailedSamples(vehicle.uniqueUUID), true);
        boolean hasRotor = rotorcraft;
        double rotorThrustNewtons = 0.0;
        double thrustForceValue = 0.0;
        List<RotorLoadCandidate> rotorCandidates = rotorcraft
            ? new ArrayList<>(Math.max(1, state.plan.rotorDiscs().size()))
            : List.of();
        List<RotorActuatorSnapshot> rotorActuators = rotorcraft
            ? new ArrayList<>(Math.max(1, state.plan.rotorDiscs().size()))
            : List.of();

        addBodyPressure(vehicle, geometry, model, airDensity, aerodynamicLinearVelocityWorld, omegaWorld, windField, accumulator, config);

        for (PreparedRotorDisc disc : state.plan.rotorDiscs()) {
            if (vehicle.allParts.contains(disc.propeller())) {
                hasRotor = true;
                RotorLoadCandidate rotorCandidate = sampleRotorDisc(
                    vehicle,
                    disc,
                    aerodynamicLinearVelocityWorld,
                    omegaWorld,
                    windField
                );
                rotorCandidates.add(rotorCandidate);
                rotorActuators.add(rotorCandidate.actuator());
            }
        }

        for (APart part : vehicle.allParts) {
            if (part instanceof PartPropeller propeller) {
                if (propeller.definition.propeller != null && propeller.definition.propeller.isRotor) {
                    continue; // Frozen rotor discs above own all rotor sampling.
                }
                PropulsionLoad load = addPropeller(
                    vehicle,
                    propeller,
                    aerodynamicLinearVelocityWorld,
                    omegaWorld,
                    windField,
                    accumulator
                );
                thrustForceValue += load.mtsForceValue();
            } else if (part instanceof PartEngine engine
                && engine.definition.engine != null
                && engine.definition.engine.jetPowerFactor > 0.0F) {
                PropulsionLoad load = addJetEngine(
                    vehicle,
                    engine,
                    aerodynamicLinearVelocityWorld,
                    omegaWorld,
                    windField,
                    accumulator
                );
                thrustForceValue += load.mtsForceValue();
            }
        }

        if (!rotorCandidates.isEmpty() && !vehicle.outOfHealth) {
            RotorApplication rotorApplication = applyDistributedRotorCandidates(
                vehicle, state, airDensity, rotorCandidates, accumulator
            );
            rotorThrustNewtons = rotorApplication.mainRotorThrustNewtons();
            thrustForceValue += rotorApplication.appliedForceMagnitudeNewtons()
                / ivForceNewtonsPerUnit(vehicle);
        }

        vehicleAccessor.pmweatherIv$setHasRotors(hasRotor);
        vehicleAccessor.pmweatherIv$setThrustForceValue(thrustForceValue);

        if (hasRotor) {
            // Rotorcraft loads were already applied by the 0.11.0 stabilized
            // rotor path: smoothed main lift plus bounded rate-command torque.
        } else if (!vehicle.outOfHealth) {
            addFixedWingAerodynamics(
                vehicle,
                geometry,
                model,
                areaPlan,
                airDensity,
                aerodynamicLinearVelocityWorld,
                omegaWorld,
                state,
                windField,
                accumulator,
                config
            );
        }

        // No server physical contact force is generated in the aerodynamic pass. Rapier resolves BLOCK and wheel-normal collision on the persistent body; PMWeather-IV applies only anisotropic tire grip/braking afterward.

        accumulator.forceWorld = accumulator.forceWorld.add(new Vec3d(0.0, -mass * GRAVITY, 0.0));
        if (accumulator.captureSurfaces) accumulator.surfaceSamples.add(new SurfaceSample(
            "GRAVITY",
            Vec3d.ZERO,
            Vec3d.ZERO,
            Vec3d.ZERO,
            Vec3d.ZERO,
            new Vec3d(0.0, -mass * GRAVITY, 0.0),
            Vec3d.ZERO,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ));

        return new SolveResult(
            geometry, model, mass, inertia, state.plan.centerOfMassLocal(), airDensity,
            linearVelocityWorld, trueAirspeed, forwardAirspeed,
            trackAngleDegrees, sideslipDegrees, pointControls(vehicle), hasRotor,
            rotorThrustNewtons, List.copyOf(rotorActuators), centerWind,
            accumulator.forceWorld, accumulator.torqueBody, accumulator.componentPressureTorqueBody,
            accumulator.rotorDistributedTorqueBody, omegaBodyBefore,
            accumulator.meanMainWingLiftCoefficient, accumulator.wingDownwashRadians,
            List.copyOf(accumulator.surfaceSamples), List.copyOf(accumulator.propulsionSamples),
            List.copyOf(accumulator.propulsionLoads),
            windField.snapshot()
        );
    }

}
