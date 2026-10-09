package com.g9third.pmweatheriv.physics;

import java.util.ArrayList;
import java.util.List;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import static com.g9third.pmweatheriv.physics.FlightMath.PointFlow;
import static com.g9third.pmweatheriv.physics.FlightMath.toLocal;

import static com.g9third.pmweatheriv.physics.LiftingSurfaceAdapter.wingDownwashRadians;
import static com.g9third.pmweatheriv.physics.RotorModel.RotorActuatorSnapshot;
import static com.g9third.pmweatheriv.physics.PropulsionModel.PropulsionSample;
import static com.g9third.pmweatheriv.physics.PropulsionModel.PropulsionSnapshot;

import static com.g9third.pmweatheriv.physics.AircraftWind.WindFieldSnapshot;
import static com.g9third.pmweatheriv.physics.AircraftWind.WindSample;
import static com.g9third.pmweatheriv.physics.AirframeGeometry.Geometry;

/** Physical load accumulation and immutable owner/substep results. */
public final class AirframeLoads {
    private AirframeLoads() {}

    public static final class Accumulator {
        final AircraftState flightState;
        Vec3d forceWorld = Vec3d.ZERO;
        Vec3d torqueBody = Vec3d.ZERO;
        Vec3d componentPressureTorqueBody = Vec3d.ZERO;
        Vec3d rotorDistributedTorqueBody = Vec3d.ZERO;
        double meanMainWingLiftCoefficient;
        double wingDownwashRadians;
        final boolean captureSurfaces;
        final boolean capturePropulsion;
        final List<SurfaceSample> surfaceSamples;
        final List<PropulsionSample> propulsionSamples;
        final List<PropulsionSnapshot> propulsionLoads;

        Accumulator(AircraftState flightState, boolean captureSurfaces, boolean capturePropulsion) {
            this.flightState = flightState;
            this.captureSurfaces = captureSurfaces;
            this.capturePropulsion = capturePropulsion;
            this.surfaceSamples = captureSurfaces ? new ArrayList<>() : List.of();
            this.propulsionSamples = capturePropulsion && captureSurfaces ? new ArrayList<>() : List.of();
            this.propulsionLoads = capturePropulsion ? new ArrayList<>() : List.of();
        }

        Vec3d momentArm(Vec3d pointLocal) {
            Vec3d center = flightState == null || flightState.plan.centerOfMassLocal() == null
                || !flightState.plan.centerOfMassLocal().isFinite()
                    ? Vec3d.ZERO : flightState.plan.centerOfMassLocal();
            return pointLocal.subtract(center);
        }

        void recordPropulsionLoad(String type, double nativeThrust,
                Vec3d forceWorld, Vec3d torqueBody) {
            if (capturePropulsion) propulsionLoads.add(
                new PropulsionSnapshot(type, nativeThrust, forceWorld, torqueBody));
        }

        void addSurface(
            String name,
            Vec3d pointLocal,
            PointFlow flow,
            Vec3d forceWorld,
            double area,
            double liftCoefficient,
            double dragCoefficient,
            double incidenceRadians,
            double separationFraction,
            EntityVehicleF_Physics vehicle
        ) {
            addSurface(
                name, pointLocal, pointLocal, flow, forceWorld, area, liftCoefficient,
                dragCoefficient, incidenceRadians, separationFraction, vehicle,
                0.0, 0.0, 0.0
            );
        }

        void addSurface(
            String name,
            Vec3d pointLocal,
            Vec3d forceApplicationPointLocal,
            PointFlow flow,
            Vec3d forceWorld,
            double area,
            double liftCoefficient,
            double dragCoefficient,
            double incidenceRadians,
            double separationFraction,
            EntityVehicleF_Physics vehicle
        ) {
            addSurface(
                name, pointLocal, forceApplicationPointLocal, flow, forceWorld, area,
                liftCoefficient, dragCoefficient, incidenceRadians, separationFraction,
                vehicle, 0.0, 0.0, 0.0
            );
        }

        void addSurface(
            String name,
            Vec3d pointLocal,
            Vec3d forceApplicationPointLocal,
            PointFlow flow,
            Vec3d forceWorld,
            double area,
            double liftCoefficient,
            double dragCoefficient,
            double incidenceRadians,
            double separationFraction,
            EntityVehicleF_Physics vehicle,
            double relativeAirChordMps,
            double relativeAirNormalMps,
            double relativeAirSpanMps
        ) {
            Vec3d forceLocal = toLocal(vehicle, forceWorld);
            Vec3d torque = momentArm(forceApplicationPointLocal).cross(forceLocal);
            this.forceWorld = this.forceWorld.add(forceWorld);
            this.torqueBody = this.torqueBody.add(torque);
            if (captureSurfaces) surfaceSamples.add(new SurfaceSample(
                name,
                pointLocal,
                forceApplicationPointLocal,
                flow.windWorld(),
                flow.relativeAirWorld(),
                forceWorld,
                torque,
                area,
                liftCoefficient,
                dragCoefficient,
                Math.toDegrees(incidenceRadians),
                Vec3d.clamp(separationFraction, 0.0, 1.0),
                Vec3d.clamp(
                    flightState.surfaceSeparationTargets.getOrDefault(name, separationFraction),
                    0.0, 1.0
                ),
                relativeAirChordMps, relativeAirNormalMps, relativeAirSpanMps
            ));
        }
 
        void addPressureSurface(
            String name,
            Vec3d pointLocal,
            PointFlow flow,
            Vec3d forceWorld,
            double area,
            double dragCoefficient,
            EntityVehicleF_Physics vehicle
        ) {
            Vec3d forceBody = toLocal(vehicle, forceWorld);
            Vec3d torqueBody = momentArm(pointLocal).cross(forceBody);
            this.forceWorld = this.forceWorld.add(forceWorld);
            this.torqueBody = this.torqueBody.add(torqueBody);
            this.componentPressureTorqueBody = this.componentPressureTorqueBody.add(torqueBody);
            if (captureSurfaces) surfaceSamples.add(new SurfaceSample(
                name,
                pointLocal,
                pointLocal,
                flow.windWorld(),
                flow.relativeAirWorld(),
                forceWorld,
                torqueBody,
                area,
                0.0,
                dragCoefficient,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            ));
        }

        void addRotorStabilityTorque(Vec3d stabilityTorqueBody) {
            if (stabilityTorqueBody == null || !stabilityTorqueBody.isFinite()) {
                return;
            }
            this.torqueBody = this.torqueBody.add(stabilityTorqueBody);
            this.rotorDistributedTorqueBody = this.rotorDistributedTorqueBody.add(stabilityTorqueBody);
        }

        Vec3d addRotorSurface(
            String name,
            Vec3d pointLocal,
            PointFlow flow,
            Vec3d forceWorld,
            double area,
            double liftCoefficient,
            double dragCoefficient,
            EntityVehicleF_Physics vehicle
        ) {
            Vec3d forceBody = toLocal(vehicle, forceWorld);
            Vec3d torqueBody = momentArm(pointLocal).cross(forceBody);
            this.forceWorld = this.forceWorld.add(forceWorld);
            this.torqueBody = this.torqueBody.add(torqueBody);
            this.rotorDistributedTorqueBody = this.rotorDistributedTorqueBody.add(torqueBody);
            if (captureSurfaces) surfaceSamples.add(new SurfaceSample(
                name,
                pointLocal,
                pointLocal,
                flow.windWorld(),
                flow.relativeAirWorld(),
                forceWorld,
                torqueBody,
                area,
                liftCoefficient,
                dragCoefficient,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            ));
            return torqueBody;
        }

   }

    public record Controls(double aileron, double elevator, double rudder, double flaps, double throttle) {
    }

    public record SurfaceSample(
        String name,
        Vec3d pointLocal,
        Vec3d forceApplicationPointLocal,
        Vec3d windWorld,
        Vec3d relativeAirWorld,
        Vec3d forceWorld,
        Vec3d torqueBody,
        double area,
        double liftCoefficient,
        double dragCoefficient,
        double incidenceDegrees,
        double separationFraction,
        double separationTargetFraction,
        double relativeAirChordMps,
        double relativeAirNormalMps,
        double relativeAirSpanMps,
        Vec3d physicalRelativeAirWorld,
        Vec3d adjustedRelativeAirWorld,
        double attachedIncidenceDegrees,
        double separationBefore
    ) {
        public SurfaceSample(String name, Vec3d pointLocal, Vec3d forceApplicationPointLocal,
                Vec3d windWorld, Vec3d relativeAirWorld,
                Vec3d forceWorld, Vec3d torqueBody, double area, double liftCoefficient,
                double dragCoefficient, double incidenceDegrees, double separationFraction,
                double separationTargetFraction, double relativeAirChordMps,
                double relativeAirNormalMps, double relativeAirSpanMps) {
            this(name, pointLocal, forceApplicationPointLocal, windWorld, relativeAirWorld,
                forceWorld, torqueBody, area,
                liftCoefficient, dragCoefficient, incidenceDegrees, separationFraction,
                separationTargetFraction, relativeAirChordMps, relativeAirNormalMps,
                relativeAirSpanMps, relativeAirWorld, relativeAirWorld, incidenceDegrees, separationFraction);
        }
        SurfaceSample withLiftFlow(Vec3d physical, Vec3d adjusted, double attachedAlpha, double prior) {
            return new SurfaceSample(name, pointLocal, forceApplicationPointLocal,
                windWorld, relativeAirWorld, forceWorld,
                torqueBody, area, liftCoefficient, dragCoefficient, incidenceDegrees,
                separationFraction, separationTargetFraction, relativeAirChordMps,
                relativeAirNormalMps, relativeAirSpanMps, physical, adjusted,
                Math.toDegrees(attachedAlpha), prior);
        }
    }

    public record SubstepAerodynamicLoads(
        Vec3d forceWorld,
        Vec3d torqueBody,
        Vec3d componentPressureTorqueBody,
        double meanMainWingLiftCoefficient,
        double wingDownwashRadians,
        WindSample centerWind,
        List<SurfaceSample> surfaceSamples
    ) {
        static final SubstepAerodynamicLoads ZERO = new SubstepAerodynamicLoads(
            Vec3d.ZERO, Vec3d.ZERO, Vec3d.ZERO, 0.0, 0.0,
            new WindSample(Vec3d.ZERO, Vec3d.ZERO, Vec3d.ZERO), List.of()
        );
    }

    public record SolveResult(
        Geometry geometry, ModelSurfaceMap.PreparedModel model, double mass, Vec3d inertia,
        Vec3d centerOfMassLocal, double airDensity, Vec3d linearVelocityBefore,
        double trueAirspeed, double forwardAirspeed,
        double trackAngleDegrees, double sideslipDegrees, Controls controls, boolean hasRotor,
        double rotorThrustNewtons, List<RotorActuatorSnapshot> rotorActuators, WindSample centerWind,
        Vec3d forceWorld, Vec3d torqueBody, Vec3d componentPressureTorqueBody,
        Vec3d rotorDistributedTorqueBody, Vec3d omegaBefore, double meanMainWingLiftCoefficient,
        double wingDownwashRadians, List<SurfaceSample> surfaceSamples,
        List<PropulsionSample> propulsionSamples, List<PropulsionSnapshot> propulsionLoads,
        WindFieldSnapshot windFieldSnapshot
    ) {
        public Vec3d appliedAcceleration() { return forceWorld.scale(1.0 / mass); }
        public int windRequests() { return windFieldSnapshot.stations().size(); }
        public int windQueries() { return windFieldSnapshot.points().size(); }
    }
}
