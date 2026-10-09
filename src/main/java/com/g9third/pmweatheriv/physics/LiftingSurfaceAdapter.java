package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import com.g9third.pmweatheriv.compat.PMAeroBridge;
import java.util.Map;
import java.util.List;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import static com.g9third.pmweatheriv.physics.FlightMath.EPSILON;
import static com.g9third.pmweatheriv.physics.FlightMath.MAX_CONTROL_SURFACE_EFFECTIVENESS;
import static com.g9third.pmweatheriv.physics.FlightMath.MAX_FINITE_WING_ROLL_FLOW_FACTOR;
import static com.g9third.pmweatheriv.physics.FlightMath.MIN_CONTROL_SURFACE_EFFECTIVENESS;
import static com.g9third.pmweatheriv.physics.FlightMath.MIN_FINITE_WING_ROLL_FLOW_FACTOR;
import static com.g9third.pmweatheriv.physics.FlightMath.PointFlow;
import static com.g9third.pmweatheriv.physics.FlightMath.WING_SPAN_EFFICIENCY;
import static com.g9third.pmweatheriv.physics.FlightMath.activeTimeStepSeconds;
import static com.g9third.pmweatheriv.physics.FlightMath.advanceUnsteadyState;
import static com.g9third.pmweatheriv.physics.FlightMath.point;
import static com.g9third.pmweatheriv.physics.FlightMath.pointFlow;
import static com.g9third.pmweatheriv.physics.FlightMath.toLocal;
import static com.g9third.pmweatheriv.physics.FlightMath.toWorld;

import static com.g9third.pmweatheriv.physics.AirframePreparation.prepareFixedWingPlan;

import static com.g9third.pmweatheriv.physics.AircraftWind.WindField;
import static com.g9third.pmweatheriv.physics.AirframeLoads.Accumulator;
import static com.g9third.pmweatheriv.physics.AirframeGeometry.Geometry;
import static com.g9third.pmweatheriv.physics.AirframeGeometry.SurfaceAreaPlan;

/** Evaluates distributed fixed-wing loads in copied authored surface poses about the physical center of mass. */
public final class LiftingSurfaceAdapter {
    private LiftingSurfaceAdapter() {}

    static WingRuntimeGeometry wingRuntimeGeometry(
        EntityVehicleF_Physics vehicle,
        ModelSurfaceMap.PreparedModel model,
        SurfaceAreaPlan areaPlan,
        Geometry geometry,
        PreparedFixedWingPlan fixedWingPlan
    ) {
        return wingRuntimeGeometry(vehicle, model, areaPlan, geometry, fixedWingPlan, null);
    }

    static WingRuntimeGeometry wingRuntimeGeometry(
        EntityVehicleF_Physics vehicle,
        ModelSurfaceMap.PreparedModel model,
        SurfaceAreaPlan areaPlan,
        Geometry geometry,
        PreparedFixedWingPlan fixedWingPlan,
        AircraftState state
    ) {
        AnimatedWingGeometry animated = fixedWingPlan == null
            ? AnimatedWingGeometry.empty() : fixedWingPlan.animatedWingGeometry();
        AnimatedWingGeometry.RuntimeState animation = animated.runtime(vehicle);
        AuthoredLiftingSurfacePoses.Snapshot poses = AuthoredLiftingSurfacePoses.snapshot(vehicle, model, state);
        if ((!poses.activeWing(model) && (!animation.active() || model != null && !model.authoredPoseBindings().isEmpty()))
            || model == null || !model.modelBased()) {
            return new WingRuntimeGeometry(
                animation, poses, geometry.wingSpan(), geometry.aspectRatio(),
                fixedWingPlan == null ? Math.max(0.05, geometry.wingArea() / Math.max(0.5, geometry.wingSpan()))
                    : fixedWingPlan.wingChord()
            );
        }

        double minimumX = Double.POSITIVE_INFINITY;
        double maximumX = Double.NEGATIVE_INFINITY;
        double baselineMinimumX = Double.POSITIVE_INFINITY;
        double baselineMaximumX = Double.NEGATIVE_INFINITY;
        double centerX = .5 * (model.fullBounds().minimum().x() + model.fullBounds().maximum().x());
        for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
            ModelSurfaceMap.SurfaceKind kind = patch.kind();
            if (kind != ModelSurfaceMap.SurfaceKind.WING
                && kind != ModelSurfaceMap.SurfaceKind.AILERON
                && kind != ModelSurfaceMap.SurfaceKind.ELEVON) {
                continue;
            }
            AuthoredLiftingSurfacePoses.Pose pose = poses.forPatch(patch, animation);
            if (!pose.visible()) continue;
            AnimatedWingGeometry.RigidTransform transform = pose.transform();
            Vec3d baselineSpan = patch.spanLocal().normalized();
            double side = patch.pointLocal().x() >= centerX ? 1.0 : -1.0;
            double targetRadius = Math.max(
                Math.abs(patch.pointLocal().x() - centerX), patch.sectionMaximumRadius()
            );
            Vec3d radialEdge = patch.pointLocal();
            if (Math.abs(baselineSpan.x()) > 1.0E-5) {
                double distanceAlongSpan =
                    (centerX + side * targetRadius - patch.pointLocal().x()) / baselineSpan.x();
                radialEdge = patch.pointLocal().add(baselineSpan.scale(distanceAlongSpan));
            }
            Vec3d liveEdge = transform.point(radialEdge);
            if (liveEdge.isFinite()) {
                minimumX = Math.min(minimumX, liveEdge.x());
                maximumX = Math.max(maximumX, liveEdge.x());
                baselineMinimumX = Math.min(baselineMinimumX, radialEdge.x());
                baselineMaximumX = Math.max(baselineMaximumX, radialEdge.x());
            }
        }
        double liveSpan = projectedWingSpan(geometry.wingSpan(), baselineMaximumX - baselineMinimumX,
            maximumX - minimumX);
        double aspectRatio = liveSpan * liveSpan / Math.max(0.01, geometry.wingArea());
        double meanChord = Math.max(0.05, geometry.wingArea() / Math.max(0.5, liveSpan));
        return new WingRuntimeGeometry(animation, poses, liveSpan, aspectRatio, meanChord);
    }

    /** Sparse one-section geometry must not change authored span at an identity pose. */
    static double projectedWingSpan(double preparedSpan, double baselineSampleSpan, double liveSampleSpan) {
        double ratio = Double.isFinite(baselineSampleSpan) && baselineSampleSpan > .1
            && Double.isFinite(liveSampleSpan) && liveSampleSpan > .1 ? liveSampleSpan / baselineSampleSpan : 1;
        return Math.max(.1, preparedSpan * ratio);
    }

    static void addFixedWingAerodynamics(
        EntityVehicleF_Physics vehicle,
        Geometry geometry,
        ModelSurfaceMap.PreparedModel model,
        SurfaceAreaPlan areaPlan,
        double density,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        AircraftState state,
        WindField windField,
        Accumulator accumulator,
        PMWeatherIVConfig.Values config
    ) {
        // 0.8.3bx restores fixed-wing aerodynamic ownership to the persistent
        // Sable rigid body. IV remains the content/control/propulsion/gameplay
        // layer, but its per-tick pitch/yaw/roll response law is deliberately not
        // converted into a continuous Sable torque. Every aerodynamic force below
        // is instead evaluated from the live rigid-body point velocity
        // V + omega x r, the immutable same-tick PMAero force-point wind snapshot, and
        // the copied same-tick authored model pose. Torque therefore exists only as the physical
        // r x F moment of distributed forces (plus the rigid-body gyroscopic term
        // handled by the integrator).
        if (geometry.wingArea() <= 0.01 || geometry.wingSpan() <= 0.10) {
            return;
        }

        PreparedFixedWingPlan fixedWingPlan = state != null
            && state.plan.fixedWingPlan() != null
            ? state.plan.fixedWingPlan()
            : prepareFixedWingPlan(vehicle, model, areaPlan, geometry);
        WingRuntimeGeometry runtimeWing = wingRuntimeGeometry(
            vehicle, model, areaPlan, geometry, fixedWingPlan, state
        );
        double flapActual = Double.isFinite(vehicle.flapActualAngleVar.currentValue)
            ? vehicle.flapActualAngleVar.currentValue : 0.0;
        double flapFraction = (runtimeWing.poses().allWingsUseFlapSweep(model)
            || model.authoredPoseBindings().isEmpty() && fixedWingPlan.animatedWingGeometry().usesFlapVariableAsWholeWingSweep())
            ? 0.0
            : flapDeploymentFraction(flapActual, vehicle.definition.motorized.flapNotches);
        double aileronRadians = boundedAuthoredControlRadians(
            vehicle.aileronInputVar.currentValue,
            0.0,
            EntityVehicleF_Physics.MAX_AILERON_ANGLE,
            EntityVehicleF_Physics.MAX_AILERON_TRIM
        );
        double elevatorRadians = boundedAuthoredControlRadians(
            vehicle.elevatorInputVar.currentValue,
            0.0,
            EntityVehicleF_Physics.MAX_ELEVATOR_ANGLE,
            EntityVehicleF_Physics.MAX_ELEVATOR_TRIM
        );
        double rudderRadians = boundedAuthoredControlRadians(
            vehicle.rudderInputVar.currentValue,
            0.0,
            EntityVehicleF_Physics.MAX_RUDDER_ANGLE,
            EntityVehicleF_Physics.MAX_RUDDER_TRIM
        );

        double aileronTrimRadians = boundedAuthoredControlRadians(0.0,
            vehicle.aileronTrimVar.currentValue, EntityVehicleF_Physics.MAX_AILERON_ANGLE,
            EntityVehicleF_Physics.MAX_AILERON_TRIM);
        double elevatorTrimRadians = boundedAuthoredControlRadians(0.0,
            vehicle.elevatorTrimVar.currentValue, EntityVehicleF_Physics.MAX_ELEVATOR_ANGLE,
            EntityVehicleF_Physics.MAX_ELEVATOR_TRIM);
        double rudderTrimRadians = boundedAuthoredControlRadians(0.0,
            vehicle.rudderTrimVar.currentValue, EntityVehicleF_Physics.MAX_RUDDER_ANGLE,
            EntityVehicleF_Physics.MAX_RUDDER_TRIM);
        double wingChord = runtimeWing.wingChord();
        double horizontalTailChord = fixedWingPlan.horizontalTailChord();
        double verticalTailChord = fixedWingPlan.verticalTailChord();
        double wingMaxCl = config.maxLiftCoefficient() * (1.0 + Math.abs(flapFraction) * 0.45);
        double wingZeroLiftCl = wingZeroLiftCoefficient(flapFraction);
        double flapFormDrag = flapFraction * flapFraction * 0.08;
        StructuralDamageMask.Snapshot structuralDamage = StructuralDamageMask.snapshot(vehicle);

        if (model.modelBased() && !model.liftingPatches().isEmpty()) {
            // A classified hinged control is folded back into its parent section.
            // The parent owns the section flow, separation state, and force; control
            // deflection changes its circulation without adding a second surface force.
            AileronWingCoupling aileronCoupling = fixedWingPlan.aileronCoupling();
            AileronWingCoupling elevonCoupling = fixedWingPlan.elevonCoupling();
            double elevatorEffectiveness = fixedWingPlan.elevatorEffectiveness();
            double rudderEffectiveness = fixedWingPlan.rudderEffectiveness();

            double wingLiftAreaSum = 0.0;
            double wingAreaSum = 0.0;
            boolean modelWingSolved = false;

            // Main wing is solved first so the same instantaneous loading can
            // generate physical downwash for the tail on this Sable substep.
            for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
                if (!structuralDamage.active(patch)) {
                    continue;
                }
                if (patch.kind() != ModelSurfaceMap.SurfaceKind.WING) {
                    continue;
                }
                double baseArea = areaPlan.areaFor(patch);
                double movingAileronArea = wingControlAreaForWingPatch(
                    model, patch, aileronCoupling, areaPlan, ModelSurfaceMap.SurfaceKind.AILERON,
                    runtimeWing.poses(), structuralDamage
                );
                double movingElevonArea = wingControlAreaForWingPatch(
                    model, patch, elevonCoupling, areaPlan, ModelSurfaceMap.SurfaceKind.ELEVON,
                    runtimeWing.poses(), structuralDamage
                );
                double movingArea = movingAileronArea + movingElevonArea;
                double coherentArea = baseArea + movingArea;
                if (coherentArea <= EPSILON) {
                    continue;
                }
                AuthoredLiftingSurfacePoses.Pose pose = runtimeWing.poses().forPatch(patch, runtimeWing.animation());
                if (!pose.visible()) continue;
                double side = patch.pointLocal().x() >= 0.0 ? 1.0 : -1.0;
                double control = wingControlStripControlRadians(
                    model, areaPlan, runtimeWing.poses(), structuralDamage, patch,
                    elevonCoupling, ModelSurfaceMap.SurfaceKind.ELEVON,
                    new ControlDeflection(-elevatorRadians, side * aileronRadians, 0,
                        -elevatorTrimRadians, side * aileronTrimRadians, 0).remaining(pose)
                ) + wingControlStripControlRadians(
                    model, areaPlan, runtimeWing.poses(), structuralDamage, patch,
                    aileronCoupling, ModelSurfaceMap.SurfaceKind.AILERON,
                    new ControlDeflection(0, side * aileronRadians, 0,
                        0, side * aileronTrimRadians, 0).remaining(pose)
                );
                AnimatedWingGeometry.RigidTransform wingTransform = pose.transform();
                coherentArea *= pose.areaFactor(patch);
                Vec3d livePoint = wingTransform.point(patch.pointLocal());
                // CG and aerodynamic centers share the same model-local frame.
                // Preserve the real lever, including motion caused by wing sweep.
                Vec3d liveAerodynamicCenter = wingTransform.point(patch.aerodynamicCenterLocal());
                Vec3d relativeControlVelocity = relativeWingControlVelocityForStrip(
                    model, areaPlan, runtimeWing.poses(), patch, pose,
                    aileronCoupling, ModelSurfaceMap.SurfaceKind.AILERON, structuralDamage
                ).scale(movingAileronArea).add(relativeWingControlVelocityForStrip(
                    model, areaPlan, runtimeWing.poses(), patch, pose,
                    elevonCoupling, ModelSurfaceMap.SurfaceKind.ELEVON, structuralDamage
                ).scale(movingElevonArea));
                if (movingArea > EPSILON) relativeControlVelocity = relativeControlVelocity.scale(1.0 / movingArea);
                Vec3d surfaceVelocity = mixedSurfaceKinematicVelocity(
                    pose.relativePointVelocity(patch.pointLocal()), relativeControlVelocity,
                    movingArea, baseArea + movingArea
                );
                SurfaceLoad load = addHorizontalSurface(
                    vehicle,
                    "SABLE_WING_" + patch.name(),
                    livePoint,
                    liveAerodynamicCenter,
                    coherentArea,
                    control,
                    wingZeroLiftCl,
                    wingMaxCl,
                    flapFormDrag,
                    runtimeWing.aspectRatio(),
                    1.0,
                    wingTransform.direction(patch.spanLocal()).normalized(),
                    wingTransform.direction(patch.chordLocal()).normalized(),
                    wingTransform.normal(patch.normalLocal()).normalized(),
                    liftingPatchWettedAreaRatio(patch, Math.max(baseArea, EPSILON)),
                    wingChord,
                    density,
                    linearVelocityWorld,
                    omegaWorld,
                    windField,
                    accumulator,
                    config, 0.0, surfaceVelocity
        );
                wingLiftAreaSum += load.liftCoefficient() * coherentArea;
                wingAreaSum += coherentArea;
                modelWingSolved = true;
            }

            // A few packs expose only moving/elevon wing geometry. Keep those
            // model-backed rather than falling all the way to body-axis points.
            if (!modelWingSolved) {
                for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
                    if (!structuralDamage.active(patch)) {
                        continue;
                    }
                    ModelSurfaceMap.SurfaceKind kind = patch.kind();
                    if (kind != ModelSurfaceMap.SurfaceKind.AILERON
                        && kind != ModelSurfaceMap.SurfaceKind.ELEVON) {
                        continue;
                    }
                    double area = areaPlan.areaFor(patch);
                    if (area <= EPSILON) {
                        continue;
                    }
                    AuthoredLiftingSurfacePoses.Pose pose = runtimeWing.poses().forPatch(patch, runtimeWing.animation());
                    if (!pose.visible()) continue;
                    double side = patch.pointLocal().x() >= 0.0 ? 1.0 : -1.0;
                    double control = pose.control(kind == ModelSurfaceMap.SurfaceKind.ELEVON ? -elevatorRadians : 0,
                        side * aileronRadians, 0,
                        kind == ModelSurfaceMap.SurfaceKind.ELEVON ? -elevatorTrimRadians : 0,
                        side * aileronTrimRadians, 0);
                    AnimatedWingGeometry.RigidTransform wingTransform = pose.transform();
                    area *= pose.areaFactor(patch);
                    Vec3d livePoint = wingTransform.point(patch.pointLocal());
                    Vec3d liveAerodynamicCenter = wingTransform.point(patch.aerodynamicCenterLocal());
                    SurfaceLoad load = addHorizontalSurface(
                        vehicle,
                        "SABLE_" + kind.name() + "_" + patch.name(),
                        livePoint,
                        liveAerodynamicCenter,
                        area,
                        control,
                        wingZeroLiftCl,
                        wingMaxCl,
                        flapFormDrag,
                        runtimeWing.aspectRatio(),
                        1.0,
                        wingTransform.direction(patch.spanLocal()).normalized(),
                        wingTransform.direction(patch.chordLocal()).normalized(),
                        wingTransform.normal(patch.normalLocal()).normalized(),
                        liftingPatchWettedAreaRatio(patch, area),
                        wingChord,
                        density,
                        linearVelocityWorld,
                        omegaWorld,
                        windField,
                        accumulator,
                        config, 0.0, pose.relativePointVelocity(patch.pointLocal())
        );
                    wingLiftAreaSum += load.liftCoefficient() * area;
                    wingAreaSum += area;
                }
            }

            double meanWingCl = wingAreaSum > EPSILON
                ? wingLiftAreaSum / wingAreaSum : 0.0;
            double downwashRadians = wingDownwashRadians(
                meanWingCl, runtimeWing.aspectRatio()
            );
            accumulator.meanMainWingLiftCoefficient = meanWingCl;
            accumulator.wingDownwashRadians = downwashRadians;

            boolean horizontalParentSolved = false;
            boolean verticalParentSolved = false;
            for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
                if (!structuralDamage.active(patch)) {
                    continue;
                }
                ModelSurfaceMap.SurfaceKind kind = patch.kind();
                AuthoredLiftingSurfacePoses.Pose pose = runtimeWing.poses().forPatch(patch, runtimeWing.animation());
                if (!pose.visible()) continue;
                AnimatedWingGeometry.RigidTransform transform = pose.transform();
                Vec3d livePoint = transform.point(patch.pointLocal());
                Vec3d liveCenter = transform.point(patch.aerodynamicCenterLocal());
                Vec3d liveSpan = transform.direction(patch.spanLocal()).normalized();
                Vec3d liveChord = transform.direction(patch.chordLocal()).normalized();
                Vec3d liveNormal = transform.normal(patch.normalLocal()).normalized();
                double areaFactor = pose.areaFactor(patch);
                if (kind == ModelSurfaceMap.SurfaceKind.HORIZONTAL_TAIL) {
                    double baseArea = areaPlan.areaFor(patch);
                    double movingArea = movingControlAreaForParentPatch(
                        model, areaPlan, patch, kind, ModelSurfaceMap.SurfaceKind.ELEVATOR,
                        runtimeWing.poses(), structuralDamage
                    );
                    double coherentArea = (baseArea + movingArea) * areaFactor;
                    if (coherentArea <= EPSILON) {
                        continue;
                    }
                    Vec3d elevatorVelocity = relativeMovingControlVelocityForParent(
                        model, areaPlan, runtimeWing.poses(), patch, pose,
                        ModelSurfaceMap.SurfaceKind.HORIZONTAL_TAIL,
                        ModelSurfaceMap.SurfaceKind.ELEVATOR, structuralDamage
                    );
                    double coupledElevator = coupledParentControlRadians(
                        model, areaPlan, runtimeWing.poses(), structuralDamage, patch, pose,
                        ModelSurfaceMap.SurfaceKind.ELEVATOR,
                        new ControlDeflection(-elevatorRadians, 0, 0,
                            -elevatorTrimRadians, 0, 0).remaining(pose), elevatorEffectiveness
                    );
                    Vec3d surfaceVelocity = mixedSurfaceKinematicVelocity(
                        pose.relativePointVelocity(patch.pointLocal()), elevatorVelocity,
                        movingArea, baseArea + movingArea
                    );
                    addHorizontalSurface(
                        vehicle,
                        "SABLE_HTAIL_" + patch.name(),
                        livePoint,
                        liveCenter,
                        coherentArea,
                        coupledElevator,
                        0.0,
                        Math.min(1.25, config.maxLiftCoefficient()),
                        0.0,
                        4.0,
                        1.0,
                        liveSpan,
                        liveChord,
                        liveNormal,
                        liftingPatchWettedAreaRatio(patch, Math.max(baseArea, EPSILON)),
                        horizontalTailChord,
                        density,
                        linearVelocityWorld,
                        omegaWorld,
                        windField,
                        accumulator,
                        config, downwashRadians, surfaceVelocity
        );
                    horizontalParentSolved = true;
                } else if (kind == ModelSurfaceMap.SurfaceKind.TAILERON) {
                    double area = areaPlan.areaFor(patch) * areaFactor;
                    if (area <= EPSILON) {
                        continue;
                    }
                    double side = patch.pointLocal().x() >= 0.0 ? 1.0 : -1.0;
                    addHorizontalSurface(
                        vehicle,
                        "SABLE_TAILERON_" + patch.name(),
                        livePoint,
                        liveCenter,
                        area,
                        pose.control(-elevatorRadians, side * aileronRadians, 0,
                            -elevatorTrimRadians, side * aileronTrimRadians, 0),
                        0.0,
                        Math.min(1.25, config.maxLiftCoefficient()),
                        0.0,
                        4.0,
                        1.0,
                        liveSpan,
                        liveChord,
                        liveNormal,
                        liftingPatchWettedAreaRatio(patch, area),
                        horizontalTailChord,
                        density,
                        linearVelocityWorld,
                        omegaWorld,
                        windField,
                        accumulator,
                        config, downwashRadians,
                        pose.relativePointVelocity(patch.pointLocal())
                    );
                    // A taileron is itself the physical parent surface; it does
                    // not imply that a separate ELEVATOR mesh has a fixed parent.
                } else if (kind == ModelSurfaceMap.SurfaceKind.VERTICAL_TAIL) {
                    double baseArea = areaPlan.areaFor(patch);
                    double movingArea = movingControlAreaForParentPatch(
                        model, areaPlan, patch, kind, ModelSurfaceMap.SurfaceKind.RUDDER,
                        runtimeWing.poses(), structuralDamage
                    );
                    double coherentArea = (baseArea + movingArea) * areaFactor;
                    if (coherentArea <= EPSILON) {
                        continue;
                    }
                    Vec3d rudderVelocity = relativeMovingControlVelocityForParent(
                        model, areaPlan, runtimeWing.poses(), patch, pose,
                        ModelSurfaceMap.SurfaceKind.VERTICAL_TAIL,
                        ModelSurfaceMap.SurfaceKind.RUDDER, structuralDamage
                    );
                    double coupledRudder = coupledParentControlRadians(
                        model, areaPlan, runtimeWing.poses(), structuralDamage, patch, pose,
                        ModelSurfaceMap.SurfaceKind.RUDDER,
                        new ControlDeflection(0, 0, -rudderRadians,
                            0, 0, -rudderTrimRadians).remaining(pose), rudderEffectiveness
                    );
                    Vec3d surfaceVelocity = mixedSurfaceKinematicVelocity(
                        pose.relativePointVelocity(patch.pointLocal()), rudderVelocity,
                        movingArea, baseArea + movingArea
                    );
                    addVerticalSurface(
                        vehicle,
                        "SABLE_VTAIL_" + patch.name(),
                        livePoint,
                        coherentArea,
                        coupledRudder,
                        liveSpan,
                        liveChord,
                        liveNormal,
                        liftingPatchWettedAreaRatio(patch, Math.max(baseArea, EPSILON)),
                        verticalTailChord,
                        density,
                        linearVelocityWorld,
                        omegaWorld,
                        windField,
                        accumulator,
                        config, surfaceVelocity
                    );
                    verticalParentSolved = true;
                } else if (kind == ModelSurfaceMap.SurfaceKind.CANARD
                    || kind == ModelSurfaceMap.SurfaceKind.CANARDERON) {
                    double area = areaPlan.areaFor(patch) * areaFactor;
                    if (area <= EPSILON) {
                        continue;
                    }
                    double side = patch.pointLocal().x() >= 0.0 ? 1.0 : -1.0;
                    double canardDeflection = pose.control(-elevatorRadians,
                        kind == ModelSurfaceMap.SurfaceKind.CANARDERON ? side * aileronRadians : 0, 0,
                        -elevatorTrimRadians,
                        kind == ModelSurfaceMap.SurfaceKind.CANARDERON ? side * aileronTrimRadians : 0, 0);
                    addHorizontalSurface(
                        vehicle,
                        kind == ModelSurfaceMap.SurfaceKind.CANARDERON
                            ? "SABLE_CANARDERON_" + patch.name()
                            : "SABLE_CANARD_" + patch.name(),
                        livePoint,
                        liveCenter,
                        area,
                        canardDeflection,
                        neutralCanardLiftCoefficient(),
                        Math.min(1.45, config.maxLiftCoefficient()),
                        0.0,
                        3.5,
                        1.0,
                        liveSpan,
                        liveChord,
                        liveNormal,
                        liftingPatchWettedAreaRatio(patch, area),
                        Math.max(0.05, wingChord * 0.75),
                        density,
                        linearVelocityWorld,
                        omegaWorld,
                        windField,
                        accumulator,
                        config, 0.0, pose.relativePointVelocity(patch.pointLocal())
        );
                }
            }

            // If classification found a moving control but no usable fixed parent,
            // keep a physical mesh-local control surface as a universal fallback.
            // This still creates only r x F torque; it never reinstates IV's
            // angular-response bridge.
            if (!horizontalParentSolved) {
                for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
                    if (!structuralDamage.active(patch)) {
                        continue;
                    }
                    if (patch.kind() != ModelSurfaceMap.SurfaceKind.ELEVATOR) {
                        continue;
                    }
                    double area = areaPlan.areaFor(patch);
                    if (area <= EPSILON) {
                        continue;
                    }
                    AuthoredLiftingSurfacePoses.Pose pose = runtimeWing.poses().forPatch(patch, runtimeWing.animation());
                    if (!pose.visible()) continue;
                    AnimatedWingGeometry.RigidTransform transform = pose.transform();
                    area *= pose.areaFactor(patch);
                    addHorizontalSurface(
                        vehicle,
                        "SABLE_ELEVATOR_FALLBACK_" + patch.name(),
                        transform.point(patch.aerodynamicCenterLocal()), transform.point(patch.aerodynamicCenterLocal()), area,
                        pose.control(-elevatorRadians * elevatorEffectiveness, 0, 0,
                            -elevatorTrimRadians * elevatorEffectiveness, 0, 0),
                        0.0, Math.min(1.25, config.maxLiftCoefficient()), 0.0,
                        4.0, 1.0,
                        transform.direction(patch.spanLocal()).normalized(), transform.direction(patch.chordLocal()).normalized(),
                        transform.normal(patch.normalLocal()).normalized(),
                        liftingPatchWettedAreaRatio(patch, area), horizontalTailChord,
                        density, linearVelocityWorld, omegaWorld, windField,
                        accumulator, config, downwashRadians,
                        pose.relativePointVelocity(patch.pointLocal())
        );
                }
            }
            if (!verticalParentSolved) {
                for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
                    if (!structuralDamage.active(patch)) {
                        continue;
                    }
                    if (patch.kind() != ModelSurfaceMap.SurfaceKind.RUDDER) {
                        continue;
                    }
                    double area = areaPlan.areaFor(patch);
                    if (area <= EPSILON) {
                        continue;
                    }
                    AuthoredLiftingSurfacePoses.Pose pose = runtimeWing.poses().forPatch(patch, runtimeWing.animation());
                    if (!pose.visible()) continue;
                    AnimatedWingGeometry.RigidTransform transform = pose.transform();
                    area *= pose.areaFactor(patch);
                    addVerticalSurface(
                        vehicle,
                        "SABLE_RUDDER_FALLBACK_" + patch.name(),
                        transform.point(patch.aerodynamicCenterLocal()), area,
                        pose.control(0, 0, -rudderRadians * rudderEffectiveness,
                            0, 0, -rudderTrimRadians * rudderEffectiveness),
                        transform.direction(patch.spanLocal()).normalized(), transform.direction(patch.chordLocal()).normalized(),
                        transform.normal(patch.normalLocal()).normalized(),
                        liftingPatchWettedAreaRatio(patch, area), verticalTailChord,
                        density, linearVelocityWorld, omegaWorld, windField,
                        accumulator, config,
                        pose.relativePointVelocity(patch.pointLocal())
                    );
                }
            }
            return;
        }

        addGeometrylessFixedWingFallback(
            vehicle, geometry, density, linearVelocityWorld, omegaWorld,
            windField, accumulator, config, fixedWingPlan,
            aileronRadians + aileronTrimRadians, elevatorRadians + elevatorTrimRadians,
            rudderRadians + rudderTrimRadians,
            wingZeroLiftCl, wingMaxCl, flapFormDrag
        );
    }

    /**
     * Compatibility-only force model for content packs without usable model
     * lifting geometry. It remains fully Sable force-at-point based, but is kept
     * outside the normal model-backed path so future removal or replacement does
     * not disturb the active mesh-derived solver.
     */
    static void addGeometrylessFixedWingFallback(
        EntityVehicleF_Physics vehicle,
        Geometry geometry,
        double density,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        Accumulator accumulator,
        PMWeatherIVConfig.Values config,
        PreparedFixedWingPlan fixedWingPlan,
        double aileronRadians,
        double elevatorRadians,
        double rudderRadians,
        double wingZeroLiftCl,
        double wingMaxCl,
        double flapFormDrag
    ) {
        double wingChord = fixedWingPlan.wingChord();
        double horizontalTailChord = fixedWingPlan.horizontalTailChord();
        double verticalTailChord = fixedWingPlan.verticalTailChord();
        double fallbackRollFlowFactor = finiteWingRollRateFlowFactor(
            geometry.aspectRatio(), config.liftSlopePerRadian()
        );
        double wingX = geometry.wingSpan() * 0.35;
        double wingHalfArea = geometry.wingArea() * 0.5;
        double aileronEffectiveness = fixedWingPlan.aileronEffectiveness();
        double fallbackAileron = aileronRadians * aileronEffectiveness;
        SurfaceLoad rightWing = addHorizontalSurface(
            vehicle, "SABLE_WING_FALLBACK_R", new Vec3d(wingX, 0.0, 0.0),
            wingHalfArea, fallbackAileron, wingZeroLiftCl, wingMaxCl,
            config.baseWingDragCoefficient() + flapFormDrag,
            geometry.aspectRatio(), fallbackRollFlowFactor, wingChord,
            density, linearVelocityWorld, omegaWorld, windField, accumulator, config, 0.0
        );
        SurfaceLoad leftWing = addHorizontalSurface(
            vehicle, "SABLE_WING_FALLBACK_L", new Vec3d(-wingX, 0.0, 0.0),
            wingHalfArea, -fallbackAileron, wingZeroLiftCl, wingMaxCl,
            config.baseWingDragCoefficient() + flapFormDrag,
            geometry.aspectRatio(), fallbackRollFlowFactor, wingChord,
            density, linearVelocityWorld, omegaWorld, windField, accumulator, config, 0.0
        );
        double meanWingCl = 0.5 * (
            rightWing.liftCoefficient() + leftWing.liftCoefficient()
        );
        double downwashRadians = wingDownwashRadians(
            meanWingCl, geometry.aspectRatio()
        );
        accumulator.meanMainWingLiftCoefficient = meanWingCl;
        accumulator.wingDownwashRadians = downwashRadians;

        double tailX = Math.max(0.25, geometry.horizontalTailSpan() * 0.28);
        double tailHalfArea = geometry.horizontalTailArea() * 0.5;
        double elevatorEffectiveness = fixedWingPlan.elevatorEffectiveness();
        double tailControl = -elevatorRadians * elevatorEffectiveness;
        addHorizontalSurface(
            vehicle, "SABLE_HTAIL_FALLBACK_R",
            new Vec3d(tailX, 0.0, -geometry.tailArm()), tailHalfArea,
            tailControl, 0.0, Math.min(1.25, config.maxLiftCoefficient()),
            config.baseWingDragCoefficient() * 1.15, 4.0, 1.0,
            horizontalTailChord, density, linearVelocityWorld, omegaWorld,
            windField, accumulator, config, downwashRadians
        );
        addHorizontalSurface(
            vehicle, "SABLE_HTAIL_FALLBACK_L",
            new Vec3d(-tailX, 0.0, -geometry.tailArm()), tailHalfArea,
            tailControl, 0.0, Math.min(1.25, config.maxLiftCoefficient()),
            config.baseWingDragCoefficient() * 1.15, 4.0, 1.0,
            horizontalTailChord, density, linearVelocityWorld, omegaWorld,
            windField, accumulator, config, downwashRadians
        );
        double rudderEffectiveness = fixedWingPlan.rudderEffectiveness();
        addVerticalSurface(
            vehicle, "SABLE_VTAIL_FALLBACK",
            new Vec3d(0.0, geometry.bodyHeight() * 0.25, -geometry.tailArm()),
            geometry.verticalTailArea(), -rudderRadians * rudderEffectiveness,
            verticalTailChord, density, linearVelocityWorld, omegaWorld,
            windField, accumulator, config
        );
    }

    static double liftingPatchWettedAreaRatio(
        ModelSurfaceMap.LiftingPatch patch,
        double referenceArea
    ) {
        if (patch == null || !Double.isFinite(referenceArea) || referenceArea <= EPSILON
            || !Double.isFinite(patch.weight()) || patch.weight() <= EPSILON) {
            return 2.0;
        }
        return patch.weight() / referenceArea;
    }

    static SurfaceLoad addHorizontalSurface(
        EntityVehicleF_Physics vehicle,
        String name,
        Vec3d pointLocal,
        double area,
        double controlAlphaRadians,
        double zeroLiftCoefficient,
        double maxLiftCoefficient,
        double baseDragCoefficient,
        double aspectRatio,
        double rollRateFlowFactor,
        double characteristicChordMeters,
        double density,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        Accumulator accumulator,
        PMWeatherIVConfig.Values config,
        double inducedDownwashRadians
        ) {
        return addHorizontalSurface(
            vehicle, name, pointLocal, pointLocal, area, controlAlphaRadians,
            zeroLiftCoefficient, maxLiftCoefficient, baseDragCoefficient, aspectRatio,
            rollRateFlowFactor,
            new Vec3d(1.0, 0.0, 0.0), new Vec3d(0.0, 0.0, 1.0), new Vec3d(0.0, 1.0, 0.0),
            -1.0, characteristicChordMeters, density, linearVelocityWorld, omegaWorld, windField,
            accumulator, config, inducedDownwashRadians
        );
    }

    static SurfaceLoad addHorizontalSurface(
        EntityVehicleF_Physics vehicle,
        String name,
        Vec3d pointLocal,
        double area,
        double controlAlphaRadians,
        double zeroLiftCoefficient,
        double maxLiftCoefficient,
        double baseDragCoefficient,
        double aspectRatio,
        double rollRateFlowFactor,
        Vec3d spanLocal,
        Vec3d chordLocal,
        Vec3d normalLocal,
        double wettedToReferenceAreaRatio,
        double characteristicChordMeters,
        double density,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        Accumulator accumulator,
        PMWeatherIVConfig.Values config,
        double inducedDownwashRadians
        ) {
        return addHorizontalSurface(
            vehicle, name, pointLocal, pointLocal, area, controlAlphaRadians,
            zeroLiftCoefficient, maxLiftCoefficient, baseDragCoefficient, aspectRatio,
            rollRateFlowFactor, spanLocal, chordLocal, normalLocal,
            wettedToReferenceAreaRatio, characteristicChordMeters, density,
            linearVelocityWorld, omegaWorld, windField, accumulator, config, inducedDownwashRadians
        );
    }

    static SurfaceLoad addHorizontalSurface(
        EntityVehicleF_Physics vehicle,
        String name,
        Vec3d pointLocal,
        Vec3d forceApplicationPointLocal,
        double area,
        double controlAlphaRadians,
        double zeroLiftCoefficient,
        double maxLiftCoefficient,
        double baseDragCoefficient,
        double aspectRatio,
        double rollRateFlowFactor,
        Vec3d spanLocal,
        Vec3d chordLocal,
        Vec3d normalLocal,
        double wettedToReferenceAreaRatio,
        double characteristicChordMeters,
        double density,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        Accumulator accumulator,
        PMWeatherIVConfig.Values config,
        double inducedDownwashRadians
        ) {
        return addHorizontalSurface(
            vehicle, name, pointLocal, forceApplicationPointLocal, area,
            controlAlphaRadians, zeroLiftCoefficient, maxLiftCoefficient,
            baseDragCoefficient, aspectRatio, rollRateFlowFactor,
            spanLocal, chordLocal, normalLocal, wettedToReferenceAreaRatio,
            characteristicChordMeters, density, linearVelocityWorld, omegaWorld,
            windField, accumulator, config, inducedDownwashRadians, Vec3d.ZERO
        );
    }

    static SurfaceLoad addHorizontalSurface(
        EntityVehicleF_Physics vehicle,
        String name,
        Vec3d pointLocal,
        Vec3d forceApplicationPointLocal,
        double area,
        double controlAlphaRadians,
        double zeroLiftCoefficient,
        double maxLiftCoefficient,
        double baseDragCoefficient,
        double aspectRatio,
        double rollRateFlowFactor,
        Vec3d spanLocal,
        Vec3d chordLocal,
        Vec3d normalLocal,
        double wettedToReferenceAreaRatio,
        double characteristicChordMeters,
        double density,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        Accumulator accumulator,
        PMWeatherIVConfig.Values config,
        double inducedDownwashRadians,
        Vec3d articulatedVelocityModel
        ) {
        if (area <= 1.0E-4) {
            return new SurfaceLoad(Vec3d.ZERO, 0.0, baseDragCoefficient, 0.0, 0.0);
        }
        // Wind is always sampled at the real mesh station.  The aerodynamic
        // center/center-of-pressure may differ for moment correctness and dynamic
        // separation, but moving that force point must not erase a gust gradient
        // across the aircraft.
        double priorSeparation = accumulator.flightState.surfaceSeparationFractions.getOrDefault(name, 0.0);
        Vec3d samplePoint = pointLocal.add(forceApplicationPointLocal.subtract(pointLocal)
            .scale(1.0 - Vec3d.clamp(priorSeparation, 0.0, 1.0)));
        if (windField.collecting()) {
            windField.register(name, pointLocal);
            return new SurfaceLoad(Vec3d.ZERO, 0.0, 0.0, 0.0, priorSeparation);
        }
        PointFlow flow = pointFlow(
            vehicle, samplePoint, pointLocal, accumulator.flightState.plan.centerOfMassLocal(),
            articulatedVelocityModel, linearVelocityWorld, omegaWorld, windField, name,
            rollRateFlowFactor
        );
        Vec3d normalWorld = toWorld(vehicle, normalLocal.normalized());
        double chordFlow = Math.max(0.0, flow.physicalRelativeAirWorld().dot(
            toWorld(vehicle, chordLocal.normalized())));
        Vec3d adjustedFlow = flow.relativeAirWorld().add(normalWorld.scale(
            chordFlow * Math.tan(inducedDownwashRadians)));
        SurfaceLoad windOn = evaluateVirtualSurface(
            vehicle, samplePoint, flow.physicalRelativeAirWorld(), adjustedFlow, area,
            controlAlphaRadians, zeroLiftCoefficient, maxLiftCoefficient, baseDragCoefficient,
            aspectRatio, spanLocal, chordLocal, normalLocal, wettedToReferenceAreaRatio,
            density, config.liftSlopePerRadian(), Double.NaN, accumulator.flightState,
            name, characteristicChordMeters
        );
        // Apply force at the station whose rigid-body velocity was evaluated. The new
        // separation moves the centre of pressure on the next substep, without mismatched r x F.
        Vec3d effectiveForceApplicationPointLocal = samplePoint;
        Vec3d relativeAirBody = toLocal(vehicle, flow.physicalRelativeAirWorld());
        Vec3d chordAxis = chordLocal.normalized();
        Vec3d normalAxis = normalLocal.normalized();
        Vec3d spanAxis = spanLocal.normalized();
        accumulator.addSurface(
            name, pointLocal, effectiveForceApplicationPointLocal, flow, windOn.forceWorld(), area,
            windOn.liftCoefficient(), windOn.dragCoefficient(), windOn.incidenceRadians(),
            windOn.separationFraction(), vehicle,
            relativeAirBody.dot(chordAxis), relativeAirBody.dot(normalAxis),
            relativeAirBody.dot(spanAxis)
        );
        annotateLiftFlow(accumulator, flow.physicalRelativeAirWorld(), adjustedFlow);
        return windOn;
    }

    static double finiteWingRollRateFlowFactor(double aspectRatio, double sectionLiftSlopePerRadian) {
        double effectiveAspectRatio = Vec3d.clamp(
            Double.isFinite(aspectRatio) ? aspectRatio : 1.0, 1.0, 40.0
        );
        double effectiveLiftSlope = Vec3d.clamp(
            Double.isFinite(sectionLiftSlopePerRadian)
                ? Math.abs(sectionLiftSlopePerRadian)
                : 2.0 * Math.PI,
            1.0,
            2.0 * Math.PI
        );
        double liftingLineRatio = 1.0 / (
            1.0 + effectiveLiftSlope
                / (Math.PI * WING_SPAN_EFFICIENCY * effectiveAspectRatio)
        );
        return Vec3d.clamp(
            liftingLineRatio,
            MIN_FINITE_WING_ROLL_FLOW_FACTOR,
            MAX_FINITE_WING_ROLL_FLOW_FACTOR
        );
    }

    /** No camber/incidence is invented when lifting geometry is unavailable. */
    static double wingZeroLiftCoefficient(double flapFraction) {
        return flapFraction * 0.55;
    }

    /** Signed deployment relative to authored travel; IV's 350 reference is a legacy lift-law constant. */
    static double flapDeploymentFraction(double actualDegrees, List<Float> authoredNotches) {
        if (!Double.isFinite(actualDegrees)) return 0.0;
        double travel = 0.0;
        if (authoredNotches != null) for (Float notch : authoredNotches)
            if (notch != null && Float.isFinite(notch)) travel = Math.max(travel, Math.abs(notch));
        // No authored travel means no evidence for a different normalization.
        if (travel <= EPSILON) travel = Math.max(1.0, EntityVehicleF_Physics.MAX_FLAP_ANGLE_REFERENCE);
        return Vec3d.clamp(actualDegrees / travel, -1.0, 1.0);
    }

    static double neutralCanardLiftCoefficient() { return 0.0; }

    static double mainWingAerodynamicCenterReferenceZ(
        ModelSurfaceMap.PreparedModel model,
        SurfaceAreaPlan areaPlan
    ) {
        if (model == null || areaPlan == null || !model.modelBased()) {
            return 0.0;
        }
        double weightedZ = 0.0;
        double totalArea = 0.0;
        boolean hasFixedWing = false;
        for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
            if (patch.kind() == ModelSurfaceMap.SurfaceKind.WING
                && areaPlan.areaFor(patch) > EPSILON) {
                hasFixedWing = true;
                break;
            }
        }
        for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
            ModelSurfaceMap.SurfaceKind kind = patch.kind();
            boolean mainWing = hasFixedWing
                ? kind == ModelSurfaceMap.SurfaceKind.WING
                : kind == ModelSurfaceMap.SurfaceKind.AILERON
                    || kind == ModelSurfaceMap.SurfaceKind.ELEVON;
            if (!mainWing) {
                continue;
            }
            double area = areaPlan.areaFor(patch);
            if (area <= EPSILON) {
                continue;
            }
            weightedZ += patch.aerodynamicCenterLocal().z() * area;
            totalArea += area;
        }
        return totalArea > EPSILON ? weightedZ / totalArea : 0.0;
    }

    /**
     * Ailerons use IV's actual authored geometric deflection as the hinge input.
     * Model-backed force uses the geometry-derived plain-flap section derivative
     * rather than treating the moving panel as a second isolated full-angle
     * airfoil. Detected area and spanwise position still scale force and torque.
     */
    static double boundedAuthoredControlRadians(
        double angleDegrees,
        double trimDegrees,
        double maximumAngleDegrees,
        double maximumTrimDegrees
    ) {
        double angle = Double.isFinite(angleDegrees) ? angleDegrees : 0.0;
        double trim = Double.isFinite(trimDegrees) ? trimDegrees : 0.0;
        angle = Vec3d.clamp(angle, -Math.abs(maximumAngleDegrees), Math.abs(maximumAngleDegrees));
        trim = Vec3d.clamp(trim, -Math.abs(maximumTrimDegrees), Math.abs(maximumTrimDegrees));
        return Math.toRadians(angle + trim);
    }

    /**
     * Infers the spanwise strip controlled by a model-derived aileron pair.
     *
     * <p>The OBJ patch gives a frozen spanwise centroid, while authored/model
     * area gives the total moving planform. A bounded trailing-edge chord ratio
     * converts that area into a per-side span. That same chord ratio derives a
     * thin-airfoil plain-flap effectiveness for the complete local section, so
     * the moving AILERON mesh and the fixed WING chord ahead of it remain one
     * coupled airfoil rather than two independently-stalling wings.</p>
     */
    static AileronWingCoupling wingControlCoupling(
        ModelSurfaceMap.PreparedModel model,
        SurfaceAreaPlan areaPlan,
        Geometry geometry,
        ModelSurfaceMap.SurfaceKind controlKind
    ) {
        if (controlKind != ModelSurfaceMap.SurfaceKind.AILERON
            && controlKind != ModelSurfaceMap.SurfaceKind.ELEVON) {
            return AileronWingCoupling.DISABLED;
        }
        double aileronArea = areaPlan.totalArea(controlKind);
        double wingArea = geometry.wingArea();
        double wingSpan = geometry.wingSpan();
        if (!model.modelBased() || aileronArea <= EPSILON
            || wingArea <= EPSILON || wingSpan <= 0.1) {
            return AileronWingCoupling.DISABLED;
        }

        double weightedRadius = 0.0;
        double representedArea = 0.0;
        for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
            if (patch.kind() != controlKind) {
                continue;
            }
            double patchArea = areaPlan.areaFor(patch);
            double radius = Math.abs(patch.pointLocal().x());
            if (patchArea <= EPSILON || radius <= 0.02) {
                continue;
            }
            weightedRadius += radius * patchArea;
            representedArea += patchArea;
        }
        if (representedArea <= EPSILON) {
            return AileronWingCoupling.DISABLED;
        }

        double halfSpan = Math.max(0.05, wingSpan * 0.5);
        double centerRadius = Vec3d.clamp(
            weightedRadius / representedArea, 0.0, halfSpan
        );
        double meanChord = wingArea / Math.max(0.5, wingSpan);
        double maximumAreaFraction = controlKind == ModelSurfaceMap.SurfaceKind.ELEVON ? 0.55 : 0.38;
        double areaFraction = Vec3d.clamp(aileronArea / wingArea, 0.0, maximumAreaFraction);

        // The chord ratio is inferred rather than fixed per aircraft. Typical
        // trailing-edge controls occupy roughly 18-34% of local chord; larger
        // authored aileron fractions imply a somewhat deeper control surface.
        double chordRatio = Vec3d.clamp(
            0.18 + 0.75 * areaFraction, 0.18, 0.34
        );
        double sideSpan = (aileronArea * 0.5)
            / Math.max(0.05, meanChord * chordRatio);
        double minimumSideSpan = Math.min(
            halfSpan, Math.max(0.10, halfSpan * 0.08)
        );
        double maximumSideSpan = Math.min(
            halfSpan, Math.max(minimumSideSpan, halfSpan * 0.70)
        );
        sideSpan = Vec3d.clamp(sideSpan, minimumSideSpan, maximumSideSpan);

        double minimumRadius = centerRadius - sideSpan * 0.5;
        double maximumRadius = centerRadius + sideSpan * 0.5;
        if (minimumRadius < 0.0) {
            maximumRadius -= minimumRadius;
            minimumRadius = 0.0;
        }
        if (maximumRadius > halfSpan) {
            minimumRadius -= maximumRadius - halfSpan;
            maximumRadius = halfSpan;
        }
        minimumRadius = Vec3d.clamp(minimumRadius, 0.0, halfSpan);
        maximumRadius = Vec3d.clamp(maximumRadius, minimumRadius, halfSpan);

        // A plain aileron is the hinged aft portion of one local airfoil
        // section, not a second independent wing. Convert the inferred local
        // chord fraction into the thin-airfoil plain-flap section derivative.
        // Both the moving mesh and the fixed chord ahead of the hinge use this
        // same effective section-incidence change. This prevents the moving
        // panel from being driven to the full authored deflection as an isolated
        // airfoil (and stalling on its own) while still preserving the physical
        // authored angle through a geometry-derived flap effectiveness.
        double circulationFactor = controlSurfaceEffectiveness(chordRatio, 1.0);
        return new AileronWingCoupling(
            true, minimumRadius, maximumRadius, chordRatio, circulationFactor,
            halfSpan
        );
    }

    static double wingControlStripControlRadians(
        ModelSurfaceMap.LiftingPatch wingPatch,
        AileronWingCoupling coupling,
        double localAuthoredControlRadians
    ) {
        if (!coupling.enabled()
            || wingPatch.kind() != ModelSurfaceMap.SurfaceKind.WING
            || Math.abs(localAuthoredControlRadians) <= EPSILON) {
            return 0.0;
        }

        double sectionMinimum = wingPatch.sectionMinimumRadius();
        double sectionMaximum = wingPatch.sectionMaximumRadius();
        if (!(sectionMaximum > sectionMinimum + EPSILON)) {
            double sectionCount = Math.max(1.0, wingPatch.sectionCount());
            double width = coupling.halfSpan() / sectionCount;
            sectionMinimum = Vec3d.clamp(
                Math.abs(wingPatch.pointLocal().x()) - width * 0.5,
                0.0, coupling.halfSpan()
            );
            sectionMaximum = Vec3d.clamp(
                sectionMinimum + width, sectionMinimum, coupling.halfSpan()
            );
        }

        double overlap = Math.max(
            0.0,
            Math.min(sectionMaximum, coupling.maximumRadius())
                - Math.max(sectionMinimum, coupling.minimumRadius())
        );
        double overlapFraction = Vec3d.clamp(
            overlap / Math.max(EPSILON, sectionMaximum - sectionMinimum),
            0.0, 1.0
        );
        return localAuthoredControlRadians
            * coupling.circulationFactor() * overlapFraction;
    }

    static double wingControlStripControlRadians(
        ModelSurfaceMap.PreparedModel model,
        SurfaceAreaPlan areaPlan,
        AuthoredLiftingSurfacePoses.Snapshot poses,
        StructuralDamageMask.Snapshot structuralDamage,
        ModelSurfaceMap.LiftingPatch wingPatch,
        AileronWingCoupling coupling,
        ModelSurfaceMap.SurfaceKind controlKind,
        ControlDeflection command
    ) {
        if (model == null || areaPlan == null || wingPatch == null || coupling == null
            || !coupling.enabled()) return 0.0;
        AuthoredLiftingSurfacePoses.Pose parentPose = poses == null
            ? null : poses.patches().get(wingPatch.name());
        int side = sideOf(wingPatch.pointLocal().x());
        double totalArea = 0.0;
        double weightedControl = 0.0;
        for (ModelSurfaceMap.LiftingPatch controlPatch : model.liftingPatches()) {
            if (controlPatch.kind() != controlKind || sideOf(controlPatch.pointLocal().x()) != side) continue;
            double area = areaPlan.areaFor(controlPatch) * controlSpanFraction(controlPatch, coupling);
            if (!Double.isFinite(area) || area <= EPSILON) continue;
            totalArea += area;
            if (!eligibleControlPose(poses, structuralDamage, controlPatch)) continue;
            double actual = authoredRelativeControlRadians(wingPatch, parentPose, controlPatch,
                poses == null ? null : poses.patches().get(controlPatch.name()));
            var controlPose = poses == null ? null : poses.patches().get(controlPatch.name());
            weightedControl += area * (Double.isFinite(actual)
                ? actual + command.remaining(controlPose).angle() : command.angle());
        }
        double equivalent = totalArea > EPSILON ? weightedControl / totalArea : 0.0;
        return wingControlStripControlRadians(wingPatch, coupling, equivalent);
    }

    static double wingControlAreaForWingPatch(
        ModelSurfaceMap.LiftingPatch wingPatch,
        AileronWingCoupling coupling,
        SurfaceAreaPlan areaPlan,
        ModelSurfaceMap.SurfaceKind controlKind
    ) {
        if (!coupling.enabled()
            || wingPatch.kind() != ModelSurfaceMap.SurfaceKind.WING
            || areaPlan == null) {
            return 0.0;
        }
        double totalAileronArea = areaPlan.totalArea(controlKind);
        if (totalAileronArea <= EPSILON) {
            return 0.0;
        }
        double sectionMinimum = wingPatch.sectionMinimumRadius();
        double sectionMaximum = wingPatch.sectionMaximumRadius();
        if (!(sectionMaximum > sectionMinimum + EPSILON)) {
            double sectionCount = Math.max(1.0, wingPatch.sectionCount());
            double width = coupling.halfSpan() / sectionCount;
            sectionMinimum = Vec3d.clamp(
                Math.abs(wingPatch.pointLocal().x()) - width * 0.5,
                0.0, coupling.halfSpan()
            );
            sectionMaximum = Vec3d.clamp(
                sectionMinimum + width, sectionMinimum, coupling.halfSpan()
            );
        }
        double overlap = Math.max(
            0.0,
            Math.min(sectionMaximum, coupling.maximumRadius())
                - Math.max(sectionMinimum, coupling.minimumRadius())
        );
        double controlledSpan = Math.max(
            EPSILON, coupling.maximumRadius() - coupling.minimumRadius()
        );
        return totalAileronArea * 0.5 * Vec3d.clamp(overlap / controlledSpan, 0.0, 1.0);
    }

    /** Live parent share. Missing area stays unassigned; each side owns its own controls. */
    static double wingControlAreaForWingPatch(
        ModelSurfaceMap.PreparedModel model,
        ModelSurfaceMap.LiftingPatch wingPatch,
        AileronWingCoupling coupling,
        SurfaceAreaPlan areaPlan,
        ModelSurfaceMap.SurfaceKind controlKind,
        AuthoredLiftingSurfacePoses.Snapshot poses,
        StructuralDamageMask.Snapshot structuralDamage
    ) {
        if (model == null || !coupling.enabled() || wingPatch == null
            || wingPatch.kind() != ModelSurfaceMap.SurfaceKind.WING
            || areaPlan == null) {
            return 0.0;
        }
        double overlapFraction = wingControlOverlapFraction(wingPatch, coupling);
        if (overlapFraction <= EPSILON) {
            return 0.0;
        }
        int side = sideOf(wingPatch.pointLocal().x());
        double representedControlArea = 0.0;
        for (ModelSurfaceMap.LiftingPatch controlPatch : model.liftingPatches()) {
            if (controlPatch.kind() != controlKind
                || sideOf(controlPatch.pointLocal().x()) != side
                || !eligibleControlPose(poses, structuralDamage, controlPatch)) {
                continue;
            }
            representedControlArea += areaPlan.areaFor(controlPatch)
                * controlSpanFraction(controlPatch, coupling);
        }
        return representedControlArea * overlapFraction;
    }

    private static double wingControlOverlapFraction(
        ModelSurfaceMap.LiftingPatch wingPatch,
        AileronWingCoupling coupling
    ) {
        if (wingPatch == null || coupling == null || !coupling.enabled()) {
            return 0.0;
        }
        double sectionMinimum = wingPatch.sectionMinimumRadius();
        double sectionMaximum = wingPatch.sectionMaximumRadius();
        if (!(sectionMaximum > sectionMinimum + EPSILON)) {
            double sectionCount = Math.max(1.0, wingPatch.sectionCount());
            double width = coupling.halfSpan() / sectionCount;
            sectionMinimum = Vec3d.clamp(
                Math.abs(wingPatch.pointLocal().x()) - width * 0.5,
                0.0, coupling.halfSpan()
            );
            sectionMaximum = Vec3d.clamp(
                sectionMinimum + width, sectionMinimum, coupling.halfSpan()
            );
        }
        double overlap = Math.max(
            0.0,
            Math.min(sectionMaximum, coupling.maximumRadius())
                - Math.max(sectionMinimum, coupling.minimumRadius())
        );
        return Vec3d.clamp(
            overlap / Math.max(EPSILON, coupling.maximumRadius() - coupling.minimumRadius()),
            0.0, 1.0
        );
    }

    private static double controlSpanFraction(
        ModelSurfaceMap.LiftingPatch controlPatch,
        AileronWingCoupling coupling
    ) {
        double minimum = controlPatch.sectionMinimumRadius();
        double maximum = controlPatch.sectionMaximumRadius();
        if (maximum > minimum + EPSILON) {
            double overlap = Math.max(0.0,
                Math.min(maximum, coupling.maximumRadius())
                    - Math.max(minimum, coupling.minimumRadius()));
            return Vec3d.clamp(overlap / (maximum - minimum), 0.0, 1.0);
        }
        double radius = Math.abs(controlPatch.pointLocal().x());
        return radius >= coupling.minimumRadius() - EPSILON
            && radius <= coupling.maximumRadius() + EPSILON ? 1.0 : 0.0;
    }

    private static boolean eligibleControlPose(
        AuthoredLiftingSurfacePoses.Snapshot poses,
        StructuralDamageMask.Snapshot structuralDamage,
        ModelSurfaceMap.LiftingPatch patch
    ) {
        AuthoredLiftingSurfacePoses.Pose pose = poses == null
            ? null : poses.patches().get(patch.name());
        return (pose == null || pose.visible())
            && (structuralDamage == null || structuralDamage.active(patch));
    }

    /**
     * Distributes a moving tail/fin planform onto the real fixed parent patches
     * on the same side. The resulting section preserves total area without
     * introducing a second aerodynamic state for the hinged surface.
     */
    static double movingControlAreaForParentPatch(
        ModelSurfaceMap.PreparedModel model,
        SurfaceAreaPlan areaPlan,
        ModelSurfaceMap.LiftingPatch parentPatch,
        ModelSurfaceMap.SurfaceKind parentKind,
        ModelSurfaceMap.SurfaceKind movingKind
    ) {
        if (model == null || areaPlan == null || parentPatch == null
            || parentPatch.kind() != parentKind) {
            return 0.0;
        }
        int side = sideOf(parentPatch.pointLocal().x());
        double parentAreaOnSide = 0.0;
        double movingAreaOnSide = 0.0;
        for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
            if (sideOf(patch.pointLocal().x()) != side) {
                continue;
            }
            if (patch.kind() == parentKind) {
                parentAreaOnSide += areaPlan.areaFor(patch);
            } else if (patch.kind() == movingKind) {
                movingAreaOnSide += areaPlan.areaFor(patch);
            }
        }
        double parentArea = areaPlan.areaFor(parentPatch);
        if (parentAreaOnSide <= EPSILON || movingAreaOnSide <= EPSILON
            || parentArea <= EPSILON) {
            return 0.0;
        }
        return movingAreaOnSide * Vec3d.clamp(parentArea / parentAreaOnSide, 0.0, 1.0);
    }

    /** Live parent share preserves the prepared parent denominator and drops hidden control area. */
    static double movingControlAreaForParentPatch(
        ModelSurfaceMap.PreparedModel model,
        SurfaceAreaPlan areaPlan,
        ModelSurfaceMap.LiftingPatch parentPatch,
        ModelSurfaceMap.SurfaceKind parentKind,
        ModelSurfaceMap.SurfaceKind movingKind,
        AuthoredLiftingSurfacePoses.Snapshot poses,
        StructuralDamageMask.Snapshot structuralDamage
    ) {
        if (model == null || areaPlan == null || parentPatch == null
            || parentPatch.kind() != parentKind) {
            return 0.0;
        }
        int side = sideOf(parentPatch.pointLocal().x());
        double parentAreaOnSide = 0.0;
        double visibleMovingAreaOnSide = 0.0;
        for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
            if (sideOf(patch.pointLocal().x()) != side) {
                continue;
            }
            if (patch.kind() == parentKind) {
                parentAreaOnSide += areaPlan.areaFor(patch);
            } else if (patch.kind() == movingKind
                && eligibleControlPose(poses, structuralDamage, patch)) {
                visibleMovingAreaOnSide += areaPlan.areaFor(patch);
            }
        }
        double parentArea = areaPlan.areaFor(parentPatch);
        if (parentAreaOnSide <= EPSILON || visibleMovingAreaOnSide <= EPSILON
            || parentArea <= EPSILON) {
            return 0.0;
        }
        return visibleMovingAreaOnSide
            * Vec3d.clamp(parentArea / parentAreaOnSide, 0.0, 1.0);
    }

    /** Coupled tail/fin circulation uses the live hinge angle, with scalar fallback for unresolved poses. */
    static double coupledParentControlRadians(
        ModelSurfaceMap.PreparedModel model, SurfaceAreaPlan areaPlan,
        AuthoredLiftingSurfacePoses.Snapshot poses, StructuralDamageMask.Snapshot structuralDamage,
        ModelSurfaceMap.LiftingPatch parentPatch, AuthoredLiftingSurfacePoses.Pose parentPose,
        ModelSurfaceMap.SurfaceKind controlKind, ControlDeflection command, double effectiveness
    ) {
        if (model == null || areaPlan == null || parentPatch == null) return command.angle() * effectiveness;
        int side = sideOf(parentPatch.pointLocal().x());
        double totalArea = 0.0;
        double weightedControl = 0.0;
        for (ModelSurfaceMap.LiftingPatch controlPatch : model.liftingPatches()) {
            if (controlPatch.kind() != controlKind || sideOf(controlPatch.pointLocal().x()) != side) continue;
            double area = areaPlan.areaFor(controlPatch);
            if (!Double.isFinite(area) || area <= EPSILON) continue;
            totalArea += area;
            if (!eligibleControlPose(poses, structuralDamage, controlPatch)) continue;
            double actual = authoredRelativeControlRadians(parentPatch, parentPose, controlPatch,
                poses == null ? null : poses.patches().get(controlPatch.name()));
            var controlPose = poses == null ? null : poses.patches().get(controlPatch.name());
            weightedControl += area * (Double.isFinite(actual)
                ? actual + command.remaining(controlPose).angle() : command.angle());
        }
        return effectiveness * (totalArea > EPSILON ? weightedControl / totalArea : command.angle());
    }

    /**
     * Signed chord rotation in the parent lifting plane, after removing common parent motion
     * and the prepared control's neutral incidence. Native IV matrices retain hinge direction,
     * axis magnitude, trim, offsets, clamps, easing and applyAfter composition.
     * NaN selects the existing scalar approximation when the control pose is unresolved.
     */
    static double authoredRelativeControlRadians(
        ModelSurfaceMap.LiftingPatch parentPatch, AuthoredLiftingSurfacePoses.Pose parentPose,
        ModelSurfaceMap.LiftingPatch controlPatch, AuthoredLiftingSurfacePoses.Pose controlPose
    ) {
        if (parentPatch == null || parentPose == null || controlPatch == null
            || controlPose == null || !controlPose.live()
            || parentPose.status() != AuthoredLiftingSurfacePoses.Status.STATIC && !parentPose.live())
            return Double.NaN;
        var binding = controlPose.binding();
        boolean authoredControl = switch (controlPatch.kind()) {
            case AILERON -> binding.aileron() || binding.aileronTrim();
            case ELEVON -> binding.elevator() || binding.aileron()
                || binding.elevatorTrim() || binding.aileronTrim();
            case ELEVATOR -> binding.elevator() || binding.elevatorTrim();
            case RUDDER -> binding.rudder() || binding.rudderTrim();
            default -> false;
        };
        if (!authoredControl) return Double.NaN;
        var parentTransform = parentPose.transform();
        Vec3d chord = parentTransform.direction(parentPatch.chordLocal()).normalized();
        Vec3d normal = parentTransform.normal(parentPatch.normalLocal()).normalized();
        // Keep an orthonormal section basis even after copied affine transforms.
        normal = normal.subtract(chord.scale(normal.dot(chord))).normalized();
        if (!chord.isFinite() || !normal.isFinite() || chord.lengthSquared() <= EPSILON
            || normal.lengthSquared() <= EPSILON) return Double.NaN;
        Vec3d neutral = parentTransform.direction(controlPatch.chordLocal());
        Vec3d live = controlPose.transform().direction(controlPatch.chordLocal());
        double neutralChord = neutral.dot(chord), neutralNormal = neutral.dot(normal);
        double liveChord = live.dot(chord), liveNormal = live.dot(normal);
        if (!Double.isFinite(neutralChord) || !Double.isFinite(neutralNormal)
            || !Double.isFinite(liveChord) || !Double.isFinite(liveNormal)
            || Math.hypot(neutralChord, neutralNormal) <= EPSILON
            || Math.hypot(liveChord, liveNormal) <= EPSILON) return Double.NaN;
        double delta = Math.atan2(liveNormal, liveChord) - Math.atan2(neutralNormal, neutralChord);
        return Math.atan2(Math.sin(delta), Math.cos(delta));
    }

    /**
     * Area-averaged motion of articulated controls relative to a coherent parent
     * surface. Parent motion is removed at each moving patch station so a common
     * flap-sweep transform is not counted once for the parent and again for its
     * hinged control.
     */
    static Vec3d relativeMovingControlVelocityForParent(
        ModelSurfaceMap.PreparedModel model,
        SurfaceAreaPlan areaPlan,
        AuthoredLiftingSurfacePoses.Snapshot poses,
        ModelSurfaceMap.LiftingPatch parentPatch,
        AuthoredLiftingSurfacePoses.Pose parentPose,
        ModelSurfaceMap.SurfaceKind parentKind,
        ModelSurfaceMap.SurfaceKind movingKind
    ) {
        return relativeMovingControlVelocityForParent(
            model, areaPlan, poses, parentPatch, parentPose, parentKind, movingKind,
            StructuralDamageMask.Snapshot.EMPTY
        );
    }

    static Vec3d relativeMovingControlVelocityForParent(
        ModelSurfaceMap.PreparedModel model,
        SurfaceAreaPlan areaPlan,
        AuthoredLiftingSurfacePoses.Snapshot poses,
        ModelSurfaceMap.LiftingPatch parentPatch,
        AuthoredLiftingSurfacePoses.Pose parentPose,
        ModelSurfaceMap.SurfaceKind parentKind,
        ModelSurfaceMap.SurfaceKind movingKind,
        StructuralDamageMask.Snapshot structuralDamage
    ) {
        if (model == null || areaPlan == null || poses == null || parentPatch == null
            || parentPose == null || parentPatch.kind() != parentKind) {
            return Vec3d.ZERO;
        }
        int side = sideOf(parentPatch.pointLocal().x());
        Vec3d weighted = Vec3d.ZERO;
        double totalArea = 0.0;
        for (ModelSurfaceMap.LiftingPatch movingPatch : model.liftingPatches()) {
            if (movingPatch.kind() != movingKind || sideOf(movingPatch.pointLocal().x()) != side) continue;
            double area = areaPlan.areaFor(movingPatch);
            if (!Double.isFinite(area) || area <= EPSILON) continue;
            AuthoredLiftingSurfacePoses.Pose movingPose = poses.patches().get(movingPatch.name());
            if (movingPose != null && !movingPose.visible()
                || structuralDamage != null && !structuralDamage.active(movingPatch)) continue;
            Vec3d point = movingPatch.pointLocal();
            Vec3d movingVelocity = movingPose == null
                ? Vec3d.ZERO : movingPose.relativePointVelocity(point);
            Vec3d relativeVelocity = movingVelocity.subtract(parentPose.relativePointVelocity(point));
            if (!relativeVelocity.isFinite()) continue;
            weighted = weighted.add(relativeVelocity.scale(area));
            totalArea += area;
        }
        return totalArea > EPSILON ? weighted.scale(1.0 / totalArea) : Vec3d.ZERO;
    }

    /**
     * Ailerons/elevons are distributed uniformly across the inferred span band
     * by the coupled-section area plan. The area-weighted control rate is taken
     * over that same band, then each affected parent strip receives the matching
     * fraction of it.
     */
    static Vec3d relativeWingControlVelocityForStrip(
        ModelSurfaceMap.PreparedModel model,
        SurfaceAreaPlan areaPlan,
        AuthoredLiftingSurfacePoses.Snapshot poses,
        ModelSurfaceMap.LiftingPatch wingPatch,
        AuthoredLiftingSurfacePoses.Pose wingPose,
        AileronWingCoupling coupling,
        ModelSurfaceMap.SurfaceKind controlKind
    ) {
        return relativeWingControlVelocityForStrip(
            model, areaPlan, poses, wingPatch, wingPose, coupling, controlKind,
            StructuralDamageMask.Snapshot.EMPTY
        );
    }

    static Vec3d relativeWingControlVelocityForStrip(
        ModelSurfaceMap.PreparedModel model,
        SurfaceAreaPlan areaPlan,
        AuthoredLiftingSurfacePoses.Snapshot poses,
        ModelSurfaceMap.LiftingPatch wingPatch,
        AuthoredLiftingSurfacePoses.Pose wingPose,
        AileronWingCoupling coupling,
        ModelSurfaceMap.SurfaceKind controlKind,
        StructuralDamageMask.Snapshot structuralDamage
    ) {
        if (model == null || areaPlan == null || poses == null || wingPatch == null
            || wingPose == null || !coupling.enabled()) {
            return Vec3d.ZERO;
        }
        if (wingControlOverlapFraction(wingPatch, coupling) <= EPSILON)
            return Vec3d.ZERO;

        int side = sideOf(wingPatch.pointLocal().x());
        Vec3d weighted = Vec3d.ZERO;
        double totalArea = 0.0;
        for (ModelSurfaceMap.LiftingPatch controlPatch : model.liftingPatches()) {
            if (controlPatch.kind() != controlKind || sideOf(controlPatch.pointLocal().x()) != side) continue;
            double spanFraction = controlSpanFraction(controlPatch, coupling);
            if (spanFraction <= EPSILON) continue;
            double area = areaPlan.areaFor(controlPatch);
            if (!Double.isFinite(area) || area <= EPSILON) continue;
            AuthoredLiftingSurfacePoses.Pose controlPose = poses.patches().get(controlPatch.name());
            double representedArea = area * spanFraction;
            if (!Double.isFinite(representedArea) || representedArea <= EPSILON
                || controlPose != null && !controlPose.visible()
                || structuralDamage != null && !structuralDamage.active(controlPatch)) continue;
            Vec3d point = controlPatch.pointLocal();
            Vec3d controlVelocity = controlPose == null
                ? Vec3d.ZERO : controlPose.relativePointVelocity(point);
            Vec3d relativeVelocity = controlVelocity.subtract(wingPose.relativePointVelocity(point));
            if (!relativeVelocity.isFinite()) continue;
            weighted = weighted.add(relativeVelocity.scale(representedArea));
            totalArea += representedArea;
        }
        return totalArea > EPSILON ? weighted.scale(1.0 / totalArea) : Vec3d.ZERO;
    }

    /** Mixes parent motion and control-only motion by the physical area share. */
    static Vec3d mixedSurfaceKinematicVelocity(
        Vec3d parentVelocity,
        Vec3d relativeControlVelocity,
        double movingArea,
        double totalArea
    ) {
        Vec3d safeParent = parentVelocity != null && parentVelocity.isFinite()
            ? parentVelocity : Vec3d.ZERO;
        Vec3d safeControl = relativeControlVelocity != null && relativeControlVelocity.isFinite()
            ? relativeControlVelocity : Vec3d.ZERO;
        double fraction = Double.isFinite(movingArea) && Double.isFinite(totalArea) && totalArea > EPSILON
            ? Vec3d.clamp(movingArea / totalArea, 0.0, 1.0) : 0.0;
        return safeParent.add(safeControl.scale(fraction));
    }

    static int sideOf(double x) {
        return x > 0.02 ? 1 : x < -0.02 ? -1 : 0;
    }

    /**
     * Thin-airfoil plain-flap effectiveness inferred from the movable fraction of
     * the parent lifting surface. This is the area-integrated whole-parent
     * derivative, not an isolated-moving-patch multiplier. Model-backed geometry
     * applies the equivalent derivative to one coherent parent section; the
     * partition record is retained for fallback behavior and diagnostics. It contains
     * no model-name control multiplier or target rate.
     */
    static double controlSurfaceEffectiveness(double movableArea, double totalParentArea) {
        if (!Double.isFinite(movableArea) || !Double.isFinite(totalParentArea)
            || movableArea <= EPSILON || totalParentArea <= EPSILON) {
            return MIN_CONTROL_SURFACE_EFFECTIVENESS;
        }
        double movableFraction = Vec3d.clamp(movableArea / totalParentArea, 0.02, 0.80);
        double hingeXOverChord = 1.0 - movableFraction;
        double hingeTheta = Math.acos(Vec3d.clamp(1.0 - 2.0 * hingeXOverChord, -1.0, 1.0));
        double effectiveness = 1.0 - (hingeTheta - Math.sin(hingeTheta)) / Math.PI;
        return Vec3d.clamp(
            effectiveness,
            MIN_CONTROL_SURFACE_EFFECTIVENESS,
            MAX_CONTROL_SURFACE_EFFECTIVENESS
        );
    }

    static double wingDownwashRadians(double meanWingLiftCoefficient, double aspectRatio) {
        if (!Double.isFinite(meanWingLiftCoefficient) || Math.abs(meanWingLiftCoefficient) <= 1.0E-9) {
            return 0.0;
        }
        // The tail sees the finite wing's induced-flow angle, not an unconditionally
        // doubled far-wake angle.  The prior 2*CL/(pi*e*AR) law is the asymptotic
        // far-wake small-angle result; applying all of it directly to every tail
        // made low-aspect-ratio aircraft feed increasing wing lift back into a
        // stronger tail downforce and could overwhelm their natural omega x r
        // pitch damping.  atan keeps the exact velocity-angle interpretation and
        // naturally bounds the result without a synthetic degree cap.
        double effectiveAspectRatio = Math.max(1.0, aspectRatio);
        double inducedVelocityRatio = meanWingLiftCoefficient
            / (Math.PI * effectiveAspectRatio * WING_SPAN_EFFICIENCY);
        return Math.atan(inducedVelocityRatio);
    }

    static void addVerticalSurface(
        EntityVehicleF_Physics vehicle,
        String name,
        Vec3d pointLocal,
        double area,
        double controlBetaRadians,
        double characteristicChordMeters,
        double density,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        Accumulator accumulator,
        PMWeatherIVConfig.Values config
    ) {
        addVerticalSurface(
            vehicle, name, pointLocal, area, controlBetaRadians,
            new Vec3d(0.0, 1.0, 0.0), new Vec3d(0.0, 0.0, 1.0), new Vec3d(-1.0, 0.0, 0.0),
            -1.0, characteristicChordMeters, density, linearVelocityWorld, omegaWorld,
            windField, accumulator, config
        );
    }

    static void addVerticalSurface(
        EntityVehicleF_Physics vehicle,
        String name,
        Vec3d pointLocal,
        double area,
        double controlBetaRadians,
        Vec3d spanLocal,
        Vec3d chordLocal,
        Vec3d normalLocal,
        double wettedToReferenceAreaRatio,
        double characteristicChordMeters,
        double density,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        Accumulator accumulator,
        PMWeatherIVConfig.Values config
    ) {
        addVerticalSurface(
            vehicle, name, pointLocal, area, controlBetaRadians, spanLocal,
            chordLocal, normalLocal, wettedToReferenceAreaRatio,
            characteristicChordMeters, density, linearVelocityWorld, omegaWorld,
            windField, accumulator, config, Vec3d.ZERO
        );
    }

    static void addVerticalSurface(
        EntityVehicleF_Physics vehicle,
        String name,
        Vec3d pointLocal,
        double area,
        double controlBetaRadians,
        Vec3d spanLocal,
        Vec3d chordLocal,
        Vec3d normalLocal,
        double wettedToReferenceAreaRatio,
        double characteristicChordMeters,
        double density,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        Accumulator accumulator,
        PMWeatherIVConfig.Values config,
        Vec3d articulatedVelocityModel
    ) {
        if (area <= 1.0E-4) {
            return;
        }
        if (windField.collecting()) {
            windField.register(name, pointLocal);
            return;
        }
        PointFlow flow = pointFlow(
            vehicle, pointLocal, pointLocal, accumulator.flightState.plan.centerOfMassLocal(),
            articulatedVelocityModel, linearVelocityWorld, omegaWorld, windField, name, 1.0
        );
        SurfaceLoad windOn = evaluateVirtualSurface(
            vehicle, pointLocal, flow.physicalRelativeAirWorld(), flow.physicalRelativeAirWorld(),
            area, controlBetaRadians, 0.0, 1.25,
            wettedToReferenceAreaRatio > 0.0 ? 0.0 : config.baseWingDragCoefficient() * 1.2,
            4.0, spanLocal, chordLocal, normalLocal, wettedToReferenceAreaRatio,
            density, config.liftSlopePerRadian() * 0.82, 0.08, accumulator.flightState,
            name, characteristicChordMeters
        );
        accumulator.addSurface(
            name, pointLocal, flow, windOn.forceWorld(), area, windOn.liftCoefficient(),
            windOn.dragCoefficient(), windOn.incidenceRadians(),
            windOn.separationFraction(), vehicle
        );
        annotateLiftFlow(accumulator, flow.physicalRelativeAirWorld(), flow.physicalRelativeAirWorld());
    }

    /** Signed equivalent-angle inputs that have not already been represented by a parent pose. */
    record ControlDeflection(double elevator, double aileron, double rudder,
                             double elevatorTrim, double aileronTrim, double rudderTrim) {
        double angle() { return elevator + aileron + rudder + elevatorTrim + aileronTrim + rudderTrim; }
        ControlDeflection remaining(AuthoredLiftingSurfacePoses.Pose pose) {
            if (pose == null || !pose.live()) return this;
            var binding = pose.binding();
            return new ControlDeflection(binding.elevator() ? 0 : elevator,
                binding.aileron() ? 0 : aileron, binding.rudder() ? 0 : rudder,
                binding.elevatorTrim() ? 0 : elevatorTrim,
                binding.aileronTrim() ? 0 : aileronTrim, binding.rudderTrim() ? 0 : rudderTrim);
        }
    }

    public record AileronWingCoupling(
        boolean enabled,
        double minimumRadius,
        double maximumRadius,
        double chordRatio,
        double circulationFactor,
        double halfSpan
    ) {
        static final AileronWingCoupling DISABLED =
            new AileronWingCoupling(false, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    public record WingRuntimeGeometry(
        AnimatedWingGeometry.RuntimeState animation,
        AuthoredLiftingSurfacePoses.Snapshot poses,
        double wingSpan,
        double aspectRatio,
        double wingChord
    ) {
    }

    public record PreparedFixedWingPlan(
        AileronWingCoupling aileronCoupling,
        AileronWingCoupling elevonCoupling,
        /** Prepared geometric maximum; each solve filters it by the live visible subset. */
        Map<ModelSurfaceMap.LiftingPatch, Double> movingAreaByParent,
        AnimatedWingGeometry animatedWingGeometry,
        double mainWingAerodynamicCenterReferenceZ,
        double wingChord,
        double horizontalTailChord,
        double verticalTailChord,
        double aileronEffectiveness,
        double elevatorEffectiveness,
        double rudderEffectiveness
    ) {
        static final PreparedFixedWingPlan EMPTY = new PreparedFixedWingPlan(
            AileronWingCoupling.DISABLED, AileronWingCoupling.DISABLED,
            Map.of(), AnimatedWingGeometry.empty(),
            0.0, 0.05, 0.05, 0.05,
            MIN_CONTROL_SURFACE_EFFECTIVENESS, MIN_CONTROL_SURFACE_EFFECTIVENESS,
            MIN_CONTROL_SURFACE_EFFECTIVENESS
        );

        /** Immutable prepared maximum. Runtime parent loads derive their live share from poses. */
        double movingAreaFor(ModelSurfaceMap.LiftingPatch patch) {
            return movingAreaByParent.getOrDefault(patch, 0.0);
        }
    }

    public record SurfaceLoad(
        Vec3d forceWorld,
        double liftCoefficient,
        double dragCoefficient,
        double incidenceRadians,
        double separationFraction
    ) {
    }

    static SurfaceLoad evaluateVirtualSurface(EntityVehicleF_Physics vehicle, Vec3d point,
            Vec3d physicalWorld, Vec3d adjustedWorld, double area, double control,
            double zeroCl, double maxCl, double profileCd, double aspectRatio,
            Vec3d span, Vec3d chord, Vec3d normal, double wettedRatio, double density,
            double slope, double inducedFactor, AircraftState state, String name, double chordLength) {
        double[] input = state.pmaeroLiftInputBuffer;
        double[] output = state.pmaeroLiftOutputBuffer;
        Vec3d physical = toLocal(vehicle, physicalWorld);
        Vec3d adjusted = toLocal(vehicle, adjustedWorld);
        packVector(input, 0, point);
        packVector(input, 3, chord.normalized());
        packVector(input, 6, span.normalized());
        packVector(input, 9, normal.normalized());
        input[12] = area;
        input[13] = Math.sqrt(Math.max(1.0, aspectRatio) * area);
        input[14] = Math.max(0.05, chordLength);
        input[15] = Math.max(1.0, aspectRatio);
        input[16] = control;
        packVector(input, 17, physical);
        packVector(input, 20, adjusted);
        input[23] = 1.0;
        input[24] = state.surfaceSeparationFractions.getOrDefault(name, Double.NaN);
        input[25] = activeTimeStepSeconds();
        input[26] = advanceUnsteadyState() ? 1.0 : 0.0;
        input[27] = slope; input[28] = maxCl; input[29] = Math.max(0.0, profileCd);
        input[30] = 0.0; input[31] = Math.max(0.0, wettedRatio);
        input[32] = density; input[33] = zeroCl; input[34] = inducedFactor;
        PMAeroBridge.evaluateLiftInto(input, output);
        state.surfaceSeparationTargets.put(name, output[10]);
        if (advanceUnsteadyState()) state.pendingSeparation.put(name, output[9]);
        return new SurfaceLoad(toWorld(vehicle, new Vec3d(output[0], output[1], output[2])),
            output[6], output[7], output[8], output[9]);
    }

    static void packVector(double[] data, int offset, Vec3d value) {
        data[offset] = value.x(); data[offset + 1] = value.y(); data[offset + 2] = value.z();
    }
    private static void annotateLiftFlow(Accumulator accumulator, Vec3d physical, Vec3d adjusted) {
        if (!accumulator.captureSurfaces || accumulator.surfaceSamples.isEmpty()) return;
        double[] input = accumulator.flightState.pmaeroLiftInputBuffer;
        double[] output = accumulator.flightState.pmaeroLiftOutputBuffer;
        double prior = Double.isNaN(input[24]) ? output[10] : Vec3d.clamp(input[24], 0.0, 1.0);
        int index = accumulator.surfaceSamples.size() - 1;
        accumulator.surfaceSamples.set(index, accumulator.surfaceSamples.get(index)
            .withLiftFlow(physical, adjusted, output[14], prior));
    }

}
