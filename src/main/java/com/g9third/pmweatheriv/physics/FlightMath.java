package com.g9third.pmweatheriv.physics;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import net.minecraft.world.level.Level;

import static com.g9third.pmweatheriv.physics.AircraftWind.WindField;
import static com.g9third.pmweatheriv.physics.AircraftWind.WindSample;
import static com.g9third.pmweatheriv.physics.AirframeLoads.Controls;

/** Converts IV motion and rotations into physical force and velocity frames. */
public final class FlightMath {
    private FlightMath() {}

    public static final double MC_TICK_SECONDS = 1.0 / 20.0;
    /**
     * Converts IV's per-tick force accumulator into SI acceleration before the
     * vehicle-specific movement scale is applied. IV integrates
     * {@code motion += force / mass} and then moves by
     * {@code motion * speedFactor}; converting that world acceleration to SI
     * therefore requires {@code 20^2 * speedFactor}, not a universal 400.
     */
    static final double IV_TICKS_SQUARED_TO_SECONDS_SQUARED = 400.0;
    static final double MPH_TO_METERS_PER_SECOND = 0.44704;
    static final double GRAVITY = 9.80665;
    static final double RAD_TO_DEG = 180.0 / Math.PI;
    static final double EPSILON = 1.0E-9;
    static final double WING_SPAN_EFFICIENCY = 0.82;
    static final double MIN_FINITE_WING_ROLL_FLOW_FACTOR = 0.78;
    static final double MAX_FINITE_WING_ROLL_FLOW_FACTOR = 0.92;
    static final double MIN_CONTROL_SURFACE_EFFECTIVENESS = 0.30;
    static final double MAX_CONTROL_SURFACE_EFFECTIVENESS = 0.85;
    static final double MIN_AUTHORED_TO_MODEL_CONTROL_AREA_RATIO = 0.20;
    static final double MAX_AUTHORED_TO_MODEL_CONTROL_AREA_RATIO = 5.00;
    static final double LANDING_GEAR_POSITION_SLOP_METERS = 0.00025;
    /** Continuous Baumgarte seating rate; equivalent to the historical 25% correction per 20 Hz tick. */
    static final double LANDING_GEAR_POSITION_STABILIZATION_RATE_PER_SECOND = 5.753641449035618;
    /**
     * Generic pneumatic-tire rolling-loss moment arm. Free-rolling resistance
     * is Crr*N with Crr=b/r, bounded below/above for unusually large/small
     * content-pack wheels. This is load-derived, not a grounded multiplier.
     */
    static final double LANDING_GEAR_ROLLING_RESISTANCE_DEFORMATION_METERS = 0.006;
    static final double LANDING_GEAR_MIN_ROLLING_RESISTANCE_COEFFICIENT = 0.008;
    static final double LANDING_GEAR_MAX_ROLLING_RESISTANCE_COEFFICIENT = 0.030;
    /**
     * Generic static end-load floor when IV provides no authored CG.  Eight percent is deliberately
     * the crossover value supported by fixed-wing landing-gear design guidance: tricycle nose gear
     * is commonly kept at roughly 8-15% static load, while tailwheel designs commonly target about
     * 5-10% on the tail.  It is a safety constraint, not the aerodynamic CG target.
     */
    static final double DERIVED_CG_MIN_END_SUPPORT_FRACTION = 0.08;
    /** Tricycle aircraft normally carry materially more than the absolute 8% floor on the nose gear. */
    static final double DERIVED_CG_TRICYCLE_MIN_FRONT_SUPPORT_FRACTION = 0.12;
    /**
     * Real-aircraft calibration for conventional horizontal-tail airplanes.  PMIV retains the
     * area-weighted main-wing aerodynamic centre in the unchanged model frame. Across the real
     * aircraft represented in the available content packs, the robust conventional-aircraft center is
     * approximately 25% MAC, matching FAA weight-and-balance guidance.  Therefore the aerodynamic prior
     * is the model-frame quarter-chord/wing-AC itself; landing-gear support may move the final CG forward.
     */
    static final double DERIVED_CONVENTIONAL_CG_TARGET_MAC_FRACTION = 0.25;
    /**
     * Safe fallback for tailless/elevon and model-unresolved airplanes.  Their reference-chord CG
     * conventions can differ radically from conventional aircraft (e.g. slender deltas), so PMIV does
     * not force the conventional %MAC calibration onto them; it keeps 5% chord positive static margin
     * forward of the model-frame wing aerodynamic centre instead.
     */
    static final double DERIVED_SAFE_FORWARD_STATIC_MARGIN_MEAN_CHORD_FRACTION = 0.05;

    // 0.11.0 rotorcraft handling model. IV remains the authored source for rotor
    // RPM, collective/pitch and raw available thrust, but PMIV owns a stabilized
    // helicopter control envelope on top of the common Sable rigid body.
    static final double ROTOR_DISC_SAMPLE_RADIUS_FRACTION = 0.72;

    /** Small immediate translation cue; primary cyclic motion comes from body attitude. */
    static final double ROTORCRAFT_TRANSLATION_CYCLIC_TILT_RADIANS = Math.toRadians(2.5);
    /** Centre-stick shaping for keyboard/joystick cyclic and pedals. */
    static final double ROTORCRAFT_CONTROL_CENTER_SLOPE = 0.45;

    /** Preserve normal IV rotor thrust linearly through ordinary hover/climb authority. */
    static final double ROTORCRAFT_THRUST_LINEAR_WEIGHT_MULTIPLIER = 1.30;
    /** Smooth asymptotic ceiling for opaque IV overspeed/descent thrust spikes. */
    static final double ROTORCRAFT_THRUST_SOFT_MAX_WEIGHT_MULTIPLIER = 2.00;
    /** First-order main-rotor thrust response time. */
    static final double ROTORCRAFT_THRUST_RESPONSE_SECONDS = 0.30;

    /** PMWeather still redistributes local disc loading, but only as a modest disturbance. */
    static final double ROTOR_DISC_AIRFLOW_ASYMMETRY_BLEND = 0.12;

    /** Rotor authority reaches full flight-control authority by this fraction of vehicle weight. */
    static final double ROTORCRAFT_FULL_CONTROL_WEIGHT_FRACTION = 0.70;

    /** Full-stick body-rate commands. Zero stick commands zero rate, never level attitude. */
    static final double ROTORCRAFT_MAX_PITCH_RATE_RADPS = Math.toRadians(40.0);
    static final double ROTORCRAFT_MAX_YAW_RATE_RADPS = Math.toRadians(35.0);
    static final double ROTORCRAFT_MAX_ROLL_RATE_RADPS = Math.toRadians(50.0);

    /** Rate-controller response time and angular-acceleration envelopes. */
    static final double ROTORCRAFT_RATE_RESPONSE_SECONDS = 0.35;
    static final double ROTORCRAFT_MAX_PITCH_ACCEL_RADPS2 = Math.toRadians(75.0);
    static final double ROTORCRAFT_MAX_YAW_ACCEL_RADPS2 = Math.toRadians(70.0);
    static final double ROTORCRAFT_MAX_ROLL_ACCEL_RADPS2 = Math.toRadians(90.0);

    /** Rotor-disc weather shear remains physical but cannot overwhelm the pilot-control envelope. */
    static final double ROTORCRAFT_MAX_WIND_SHEAR_PITCH_ACCEL_RADPS2 = Math.toRadians(12.0);
    static final double ROTORCRAFT_MAX_WIND_SHEAR_YAW_ACCEL_RADPS2 = Math.toRadians(8.0);
    static final double ROTORCRAFT_MAX_WIND_SHEAR_ROLL_ACCEL_RADPS2 = Math.toRadians(15.0);

    /**
     * Per-thread rigid-pose override used while Sable evaluates air-vehicle
     * aerodynamics at its configured physics-substep rate. It changes no IV
     * entity state and performs no PMWeather query; the wind field is the same
     * immutable force-point snapshot captured on the owning Minecraft tick.
     */
    static final ThreadLocal<SubstepEvaluationContext> SUBSTEP_EVALUATION =
        new ThreadLocal<>();

    static PointFlow pointFlow(
        EntityVehicleF_Physics vehicle,
        Vec3d pointLocal,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        String label
    ) {
        return pointFlow(
            vehicle, pointLocal, Vec3d.ZERO, linearVelocityWorld, omegaWorld,
            windField, label, 1.0
        );
    }

    static PointFlow pointFlow(
        EntityVehicleF_Physics vehicle,
        Vec3d pointLocal,
        Vec3d centerOfMassLocal,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        String label
    ) {
        return pointFlow(
            vehicle, pointLocal, centerOfMassLocal, linearVelocityWorld, omegaWorld,
            windField, label, 1.0
        );
    }

    static PointFlow pointFlow(
        EntityVehicleF_Physics vehicle,
        Vec3d pointLocal,
        Vec3d centerOfMassLocal,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        String label,
        double rollRateFlowFactor
    ) {
        return pointFlow(
            vehicle, pointLocal, pointLocal, centerOfMassLocal, linearVelocityWorld,
            omegaWorld, windField, label, rollRateFlowFactor
        );
    }

    /**
     * Evaluates rigid-body point velocity at {@code kinematicPointLocal} while
     * sampling the frozen PMWeather field at the real physical surface station
     * {@code windSamplePointLocal}.  Keeping these distinct lets a center-of-
     * pressure/aerodynamic-reference correction change r x F without moving the
     * turbulence probe away from the actual wing or tail mesh.
     */
    static PointFlow pointFlow(
        EntityVehicleF_Physics vehicle,
        Vec3d kinematicPointLocal,
        Vec3d windSamplePointLocal,
        Vec3d centerOfMassLocal,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        String label,
        double rollRateFlowFactor
    ) {
        return pointFlow(
            vehicle, kinematicPointLocal, windSamplePointLocal, centerOfMassLocal,
            Vec3d.ZERO, linearVelocityWorld, omegaWorld, windField, label, rollRateFlowFactor
        );
    }

    /**
     * Evaluates the air-relative flow at a surface that also moves relative to
     * the vehicle body. {@code articulatedVelocityModel} is the finite-difference
     * surface velocity in the same prepared model frame as its station; it is
     * added once to the rigid-body point velocity before comparing with wind.
     */
    static PointFlow pointFlow(
        EntityVehicleF_Physics vehicle,
        Vec3d kinematicPointLocal,
        Vec3d windSamplePointLocal,
        Vec3d centerOfMassLocal,
        Vec3d articulatedVelocityModel,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        String label,
        double rollRateFlowFactor
    ) {
        // linearVelocityWorld is the physical COM velocity. Surface stations are
        // still authored in IV/model-local coordinates, so every omega x r term
        // uses r = point - COM. This same frozen lever arm is also used for the
        // force moment and landing-gear Jacobian.
        Vec3d safeCenterOfMass = centerOfMassLocal == null || !centerOfMassLocal.isFinite()
            ? Vec3d.ZERO : centerOfMassLocal;
        Vec3d safeKinematicPoint = kinematicPointLocal == null || !kinematicPointLocal.isFinite()
            ? safeCenterOfMass : kinematicPointLocal;
        Vec3d safeWindPoint = windSamplePointLocal == null || !windSamplePointLocal.isFinite()
            ? safeKinematicPoint : windSamplePointLocal;
        Vec3d leverLocal = safeKinematicPoint.subtract(safeCenterOfMass);
        Vec3d omegaBody = toLocal(vehicle, omegaWorld);
        Vec3d fullAngularVelocityBody = omegaBody.cross(leverLocal);
        Vec3d rollAngularVelocityBody = new Vec3d(0.0, 0.0, omegaBody.z()).cross(leverLocal);
        Vec3d rollNormalVelocityBody = new Vec3d(0.0, rollAngularVelocityBody.y(), 0.0);
        double boundedRollFactor = Vec3d.clamp(
            Double.isFinite(rollRateFlowFactor) ? rollRateFlowFactor : 1.0,
            MIN_FINITE_WING_ROLL_FLOW_FACTOR,
            1.0
        );
        Vec3d adjustedAngularVelocityBody = fullAngularVelocityBody.add(
            rollNormalVelocityBody.scale(boundedRollFactor - 1.0)
        );
        Vec3d physicalPointVelocity = linearVelocityWorld.add(
            toWorld(vehicle, fullAngularVelocityBody)
        );
        Vec3d adjustedPointVelocity = linearVelocityWorld.add(
            toWorld(vehicle, adjustedAngularVelocityBody)
        );
        Vec3d safeArticulatedVelocity = articulatedVelocityModel != null && articulatedVelocityModel.isFinite()
            ? toWorld(vehicle, articulatedVelocityModel) : Vec3d.ZERO;
        physicalPointVelocity = physicalPointVelocity.add(safeArticulatedVelocity);
        adjustedPointVelocity = adjustedPointVelocity.add(safeArticulatedVelocity);
        WindSample wind = windField.sample(safeWindPoint, label);
        return new PointFlow(
            wind.windMetersPerSecond(),
            adjustedPointVelocity.subtract(wind.windMetersPerSecond()),
            physicalPointVelocity.subtract(wind.windMetersPerSecond())
        );
    }

    static double lerp(double start, double end, double amount) {
        return start + (end - start) * amount;
    }

    /**
     * Converts Sable's body-frame angular velocity in radians per second to
     * the relative Euler increment IV expects for one 20 Hz vehicle tick.
     */
    public static Vec3d degreesPerTickFromOmega(Vec3d omegaBody) {
        if (omegaBody == null || !omegaBody.isFinite()) {
            return Vec3d.ZERO;
        }
        return omegaBody.scale(MC_TICK_SECONDS * RAD_TO_DEG);
    }

    static double normalizedControl(double value, double maximum) {
        return Vec3d.clamp(value / Math.max(1.0, maximum), -1.0, 1.0);
    }

    static Controls pointControls(EntityVehicleF_Physics vehicle) {
        return new Controls(
            vehicle.aileronInputVar.currentValue + vehicle.aileronTrimVar.currentValue,
            vehicle.elevatorInputVar.currentValue + vehicle.elevatorTrimVar.currentValue,
            vehicle.rudderInputVar.currentValue + vehicle.rudderTrimVar.currentValue,
            vehicle.flapActualAngleVar.currentValue,
            vehicle.throttleVar.currentValue
        );
    }

    static double signedForwardDenominator(double forward) {
        if (Math.abs(forward) >= 0.05) {
            return forward;
        }
        return Math.copySign(0.05, forward == 0.0 ? 1.0 : forward);
    }

    /**
     * IV 24.0.0's authored aircraft density law. Keeping this for fixed-wing
     * flight also keeps jet power and pack-authored lift/drag on the same
     * atmospheric baseline that the content pack was tuned against.
     */
    static double ivCompatibleAirDensity(
        Level level,
        EntityVehicleF_Physics vehicle,
        double worldY
    ) {
        double worldHeight = level == null ? 384.0 : Math.max(1.0, level.getHeight());
        double halfDensityHeight = 500.0 * worldHeight / 256.0;
        double exponent = -(worldY - vehicle.seaLevel) / Math.max(1.0, halfDensityHeight);
        double density = 1.225 * Math.pow(2.0, exponent);
        return Double.isFinite(density) ? Vec3d.clamp(density, 0.05, 4.0) : 1.225;
    }

    static double finiteNonNegative(double value) {
        return Double.isFinite(value) ? Math.max(0.0, value) : 0.0;
    }

    static double finiteClamp(double value, double minimum, double maximum, double fallback) {
        return Double.isFinite(value) ? Vec3d.clamp(value, minimum, maximum) : fallback;
    }

    static Vec3d point(Point3D point) {
        return point == null ? Vec3d.ZERO : new Vec3d(point.x, point.y, point.z);
    }

    static Vec3d toWorld(EntityVehicleF_Physics vehicle, Vec3d local) {
        return toWorld(activeOrientation(vehicle), local);
    }

    static Vec3d toWorld(RotationMatrix orientation, Vec3d local) {
        Point3D point = new Point3D(local.x(), local.y(), local.z()).rotate(orientation);
        return new Vec3d(point.x, point.y, point.z);
    }

    static Vec3d toLocal(EntityVehicleF_Physics vehicle, Vec3d world) {
        return toLocal(activeOrientation(vehicle), world);
    }

    static Vec3d toLocal(RotationMatrix orientation, Vec3d world) {
        Point3D point = new Point3D(world.x(), world.y(), world.z()).reOrigin(orientation);
        return new Vec3d(point.x, point.y, point.z);
    }

    static RotationMatrix activeOrientation(EntityVehicleF_Physics vehicle) {
        SubstepEvaluationContext evaluation = SUBSTEP_EVALUATION.get();
        return evaluation != null && evaluation.orientation() != null
            ? evaluation.orientation()
            : vehicle.orientation;
    }

    static Vec3d activePositionWorld(EntityVehicleF_Physics vehicle) {
        SubstepEvaluationContext evaluation = SUBSTEP_EVALUATION.get();
        return evaluation != null && evaluation.positionWorld() != null
            ? evaluation.positionWorld()
            : point(vehicle.position);
    }

    static double activeTimeStepSeconds() {
        SubstepEvaluationContext evaluation = SUBSTEP_EVALUATION.get();
        return evaluation != null && Double.isFinite(evaluation.timeStepSeconds())
            && evaluation.timeStepSeconds() > 0.0
                ? evaluation.timeStepSeconds()
                : MC_TICK_SECONDS;
    }

    static boolean advanceUnsteadyState() {
        SubstepEvaluationContext evaluation = SUBSTEP_EVALUATION.get();
        return evaluation == null || evaluation.advanceUnsteadyState();
    }

    static Vec3d activeAngularVelocityBody(Vec3d fallback) {
        SubstepEvaluationContext evaluation = SUBSTEP_EVALUATION.get();
        Vec3d value = evaluation != null ? evaluation.angularVelocityBody() : null;
        if (value != null && value.isFinite()) {
            return value;
        }
        return fallback != null && fallback.isFinite() ? fallback : Vec3d.ZERO;
    }

    public record PointFlow(
        Vec3d windWorld,
        Vec3d relativeAirWorld,
        Vec3d physicalRelativeAirWorld
    ) {
    }

    public record SubstepEvaluationContext(
        Vec3d positionWorld,
        RotationMatrix orientation,
        Vec3d linearVelocityWorld,
        Vec3d angularVelocityBody,
        double timeStepSeconds,
        boolean advanceUnsteadyState
    ) {
        static SubstepEvaluationContext previewOnly() {
            return new SubstepEvaluationContext(
                null, null, null, null, MC_TICK_SECONDS, false
            );
        }
    }
}
