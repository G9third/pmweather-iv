package com.g9third.pmweatheriv.physics;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import com.g9third.pmweatheriv.compat.PMAeroBridge;

/** Only mutable aircraft state: Sable-exported velocity, separation, and reusable API buffers. */
public final class AircraftState {
    public AirframePlan plan;
    public RoadVehiclePhysics.BodyPlan roadBody;
    public RoadVehiclePhysics.DriveSnapshot roadDrive = RoadVehiclePhysics.DriveSnapshot.ZERO;
    public List<PropulsionModel.PropulsionSnapshot> roadPropulsion = List.of();
    public List<PropulsionModel.PropulsionSample> roadPropulsionSamples = List.of();
    public long roadActuatorGameTime = Long.MIN_VALUE;
    public String preparedGeometrySignature = "";
    public long nextModelRetryTick = Long.MIN_VALUE;
    public AircraftKinematics kinematics;
    public final Map<String, Double> pendingSeparation = new HashMap<>();
    public java.util.Set<java.util.UUID> preparedParts = java.util.Set.of();
    public Vec3d angularVelocityBody = Vec3d.ZERO;
    public Vec3d originVelocityWorld = Vec3d.ZERO;
    public boolean hasPhysicalVelocity;
    public boolean heldForSable;
    public long lastDebugGameTime = Long.MIN_VALUE;
    public final Map<String, Double> surfaceSeparationFractions = new HashMap<>();
    public final Map<String, Double> surfaceSeparationTargets = new HashMap<>();
    /**
     * 0.11.0 aggregate main-rotor lift state. The value is advanced only by
     * real Sable substeps so owner-preview solves cannot double-filter thrust.
     */
    public double rotorcraftMainThrustNewtons = Double.NaN;
    /** Slow body-rate disturbance trim, advanced only by real Sable substeps. */
    public Vec3d rotorcraftRateTrimAlphaBody = Vec3d.ZERO;
    /** Last live support solve; freezes trim while parked instead of winding up. */
    public boolean rotorcraftGroundSupported;
    public double[] pmaeroBodyInputBuffer = new double[0];
    public double[] pmaeroBodyOutputBuffer = new double[0];
    public FlightMath.PointFlow[] bodyFlowBuffer = new FlightMath.PointFlow[0];
    public final double[] pmaeroLiftInputBuffer = new double[PMAeroBridge.LIFT_INPUT_STRIDE];
    public final double[] pmaeroLiftOutputBuffer = new double[PMAeroBridge.LIFT_OUTPUT_STRIDE];
    public double[] windInputBuffer = new double[0];
    public double[] windOutputBuffer = new double[0];
    public long windGameTime = Long.MIN_VALUE;
    public AircraftWind.WindFieldSnapshot windSnapshot;
    /** One immutable IV animation pose snapshot for all solves in the owner tick. */
    public AuthoredLiftingSurfacePoses.Snapshot liftingPoses = AuthoredLiftingSurfacePoses.Snapshot.EMPTY;
    public ModelSurfaceMap.PreparedModel liftingPoseModel;
    public long liftingPoseTick = Long.MIN_VALUE;
    public final AirWarningSystem.State airWarnings = new AirWarningSystem.State();
    /** Opt-in, owner-tick pitch trim controller; rotorcraft never enter it. */
    public final AutoTrimController autoTrim = new AutoTrimController();
    public long nextAutoTrimDamageCheck = Long.MIN_VALUE;
    public int autoTrimDamageSignature;
    public AutoTrimController.State lastAutoTrimStatusState = AutoTrimController.State.OFF;
    public String lastAutoTrimStatusReason = "OFF";
    public long lastAutoTrimStatusTick = Long.MIN_VALUE;

    public boolean isPrepared() { return plan != null; }
    public List<ModelSurfaceMap.PressurePatch> bodyPressurePatches() { return plan.pressurePatches(); }

    public void reset() {
        plan = null;
        roadBody = null;
        roadDrive = RoadVehiclePhysics.DriveSnapshot.ZERO;
        roadPropulsion = List.of();
        roadPropulsionSamples = List.of();
        roadActuatorGameTime = Long.MIN_VALUE;
        preparedGeometrySignature = "";
        nextModelRetryTick = Long.MIN_VALUE;
        kinematics = null;
        pendingSeparation.clear();
        preparedParts = java.util.Set.of();
        angularVelocityBody = Vec3d.ZERO;
        originVelocityWorld = Vec3d.ZERO;
        hasPhysicalVelocity = false;
        heldForSable = false;
        lastDebugGameTime = Long.MIN_VALUE;
        surfaceSeparationFractions.clear();
        surfaceSeparationTargets.clear();
        rotorcraftMainThrustNewtons = Double.NaN;
        rotorcraftRateTrimAlphaBody = Vec3d.ZERO;
        rotorcraftGroundSupported = false;
        pmaeroBodyInputBuffer = new double[0];
        pmaeroBodyOutputBuffer = new double[0];
        bodyFlowBuffer = new FlightMath.PointFlow[0];
        windInputBuffer = new double[0];
        windOutputBuffer = new double[0];
        windSnapshot = null;
        windGameTime = Long.MIN_VALUE;
        liftingPoses = AuthoredLiftingSurfacePoses.Snapshot.EMPTY;
        liftingPoseModel = null;
        liftingPoseTick = Long.MIN_VALUE;
        airWarnings.reset();
        autoTrim.disable("RESET");
        nextAutoTrimDamageCheck = Long.MIN_VALUE;
        autoTrimDamageSignature = 0;
        lastAutoTrimStatusState = AutoTrimController.State.OFF;
        lastAutoTrimStatusReason = "OFF";
        lastAutoTrimStatusTick = Long.MIN_VALUE;
    }
}
