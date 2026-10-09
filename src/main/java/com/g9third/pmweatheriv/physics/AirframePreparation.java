package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import minecrafttransportsimulator.entities.instances.PartPropeller;
import net.minecraft.world.level.Level;
import static com.g9third.pmweatheriv.physics.FlightMath.DERIVED_CG_MIN_END_SUPPORT_FRACTION;
import static com.g9third.pmweatheriv.physics.FlightMath.DERIVED_CG_TRICYCLE_MIN_FRONT_SUPPORT_FRACTION;
import static com.g9third.pmweatheriv.physics.FlightMath.DERIVED_CONVENTIONAL_CG_TARGET_MAC_FRACTION;
import static com.g9third.pmweatheriv.physics.FlightMath.DERIVED_SAFE_FORWARD_STATIC_MARGIN_MEAN_CHORD_FRACTION;
import static com.g9third.pmweatheriv.physics.FlightMath.EPSILON;
import static com.g9third.pmweatheriv.physics.FlightMath.finiteClamp;
import static com.g9third.pmweatheriv.physics.FlightMath.point;

import static com.g9third.pmweatheriv.physics.BodySurfaceAdapter.BodyFace;
import static com.g9third.pmweatheriv.physics.BodySurfaceAdapter.bodyFaces;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceAdapter.AileronWingCoupling;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceAdapter.PreparedFixedWingPlan;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceAdapter.wingControlAreaForWingPatch;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceAdapter.wingControlCoupling;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceAdapter.controlSurfaceEffectiveness;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceAdapter.mainWingAerodynamicCenterReferenceZ;
import static com.g9third.pmweatheriv.physics.LiftingSurfaceAdapter.movingControlAreaForParentPatch;
import com.g9third.pmweatheriv.physics.RotorModel.PreparedRotorDisc;
import static com.g9third.pmweatheriv.physics.RotorModel.prepareRotorDiscs;

import static com.g9third.pmweatheriv.physics.AirframeGeometry.Geometry;
import static com.g9third.pmweatheriv.physics.AirframeGeometry.SurfaceAreaPlan;

/** Prepares immutable airframe geometry, mass references, rotor stations and control relationships. */
public final class AirframePreparation {
    private AirframePreparation() {}

    static DerivedCenterOfMass deriveCenterOfMass(
        EntityVehicleF_Physics vehicle,
        ModelSurfaceMap.PreparedModel model,
        Geometry geometry,
        PreparedFixedWingPlan fixedWingPlan,
        boolean rotorcraft
    ) {
        // All geometry and COM remain in the same IV model-local frame.
        // Choose the generic static margin relative to the measured wing AC.
        // This changes the physical moment reference, never the wing force points:
        // all aerodynamic moments use their actual application point minus this CG.
        if (rotorcraft) {
            return new DerivedCenterOfMass(
                Vec3d.ZERO,
                DerivedCenterOfMassConfiguration.ROTORCRAFT_MODEL_REFERENCE,
                0.0,
                false,
                Double.NaN,
                "IV_MODEL_ROTATION_REFERENCE_ROTORCRAFT"
            );
        }

        double meanChord = fixedWingPlan != null
            && Double.isFinite(fixedWingPlan.wingChord())
            && fixedWingPlan.wingChord() > 0.05
                ? fixedWingPlan.wingChord()
                : geometry.wingArea() / Math.max(0.5, geometry.wingSpan());
        meanChord = Math.max(0.05, meanChord);

        DerivedCenterOfMassConfiguration configuration = centerOfMassConfiguration(model);
        double candidateZ;
        double targetMacPercent;
        String targetReference;
        if (configuration == DerivedCenterOfMassConfiguration.CONVENTIONAL_HORIZONTAL_TAIL) {
            // The conventional 25% MAC target coincides with the measured wing AC.
            candidateZ = fixedWingPlan.mainWingAerodynamicCenterReferenceZ()
                + (0.25 - DERIVED_CONVENTIONAL_CG_TARGET_MAC_FRACTION) * meanChord;
            targetMacPercent = 100.0 * DERIVED_CONVENTIONAL_CG_TARGET_MAC_FRACTION;
            targetReference = "REAL_AIRCRAFT_CALIBRATED_25_PERCENT_MAC_APPROX";
        } else {
            candidateZ = fixedWingPlan.mainWingAerodynamicCenterReferenceZ()
                + DERIVED_SAFE_FORWARD_STATIC_MARGIN_MEAN_CHORD_FRACTION * meanChord;
            targetMacPercent = 20.0;
            targetReference = configuration == DerivedCenterOfMassConfiguration.TAILLESS_ELEVON_MODEL
                ? "TAILLESS_MODEL_WING_AC_PLUS_5_PERCENT_CHORD_FORWARD_MARGIN"
                : "UNRESOLVED_MODEL_WING_AC_PLUS_5_PERCENT_CHORD_FORWARD_MARGIN";
        }

        double aerodynamicCandidateZ = candidateZ;
        boolean gearClamped = false;
        CenterOfMassDiagnostic gear = centerOfMassDiagnostic(vehicle);
        if (gear.hasLongitudinalFootprint()) {
            double span = gear.maximumGearZ() - gear.minimumGearZ();
            double minimumFrontSupport = gear.tricycleLayout()
                ? DERIVED_CG_TRICYCLE_MIN_FRONT_SUPPORT_FRACTION
                : DERIVED_CG_MIN_END_SUPPORT_FRACTION;
            double minimumRearSupport = DERIVED_CG_MIN_END_SUPPORT_FRACTION;
            double minimumZ = gear.minimumGearZ() + minimumFrontSupport * span;
            double maximumZ = gear.maximumGearZ() - minimumRearSupport * span;
            double clamped = Vec3d.clamp(candidateZ, minimumZ, maximumZ);
            gearClamped = Math.abs(clamped - candidateZ) > 1.0E-9;
            candidateZ = clamped;
        } else if (model != null && model.fullBounds() != null && model.fullBounds().valid()) {
            double minZ = model.fullBounds().minimum().z();
            double maxZ = model.fullBounds().maximum().z();
            double margin = 0.10 * Math.max(0.01, maxZ - minZ);
            double clamped = Vec3d.clamp(candidateZ, minZ + margin, maxZ - margin);
            gearClamped = Math.abs(clamped - candidateZ) > 1.0E-9;
            candidateZ = clamped;
        }
        if (!Double.isFinite(candidateZ)) {
            candidateZ = 0.0;
        }
        return new DerivedCenterOfMass(
            new Vec3d(0.0, 0.0, candidateZ),
            configuration,
            aerodynamicCandidateZ,
            gearClamped,
            targetMacPercent,
            targetReference
        );
    }

    static DerivedCenterOfMassConfiguration centerOfMassConfiguration(
        ModelSurfaceMap.PreparedModel model
    ) {
        if (model == null || !model.modelBased() || model.liftingPatches() == null
            || model.liftingPatches().isEmpty()) {
            return DerivedCenterOfMassConfiguration.UNRESOLVED_SAFE_FORWARD_MARGIN;
        }
        boolean horizontalTail = false;
        boolean elevon = false;
        for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
            if (patch == null || patch.kind() == null) {
                continue;
            }
            ModelSurfaceMap.SurfaceKind kind = patch.kind();
            if (kind == ModelSurfaceMap.SurfaceKind.HORIZONTAL_TAIL
                || kind == ModelSurfaceMap.SurfaceKind.ELEVATOR
                || kind == ModelSurfaceMap.SurfaceKind.TAILERON) {
                horizontalTail = true;
            }
            if (kind == ModelSurfaceMap.SurfaceKind.ELEVON) {
                elevon = true;
            }
        }
        if (horizontalTail) {
            return DerivedCenterOfMassConfiguration.CONVENTIONAL_HORIZONTAL_TAIL;
        }
        if (elevon) {
            return DerivedCenterOfMassConfiguration.TAILLESS_ELEVON_MODEL;
        }
        return DerivedCenterOfMassConfiguration.UNRESOLVED_SAFE_FORWARD_MARGIN;
    }

    static CenterOfMassDiagnostic centerOfMassDiagnostic(EntityVehicleF_Physics vehicle) {
        double minimumZ = Double.POSITIVE_INFINITY;
        double maximumZ = Double.NEGATIVE_INFINITY;
        List<Vec3d> wheelPoints = new ArrayList<>();
        if (vehicle != null && vehicle.allParts != null) {
            for (APart part : vehicle.allParts) {
                if (!(part instanceof PartGroundDevice device)
                    || device.isSpare
                    || device.definition == null
                    || device.definition.ground == null
                    || !device.definition.ground.isWheel
                    || device.wheelbasePoint == null) {
                    continue;
                }
                double z = device.wheelbasePoint.z;
                if (Double.isFinite(z)) {
                    minimumZ = Math.min(minimumZ, z);
                    maximumZ = Math.max(maximumZ, z);
                    if (Double.isFinite(device.wheelbasePoint.x)) {
                        wheelPoints.add(new Vec3d(device.wheelbasePoint.x, device.wheelbasePoint.y, z));
                    }
                }
            }
        }
        boolean footprint = Double.isFinite(minimumZ) && Double.isFinite(maximumZ)
            && maximumZ - minimumZ > 0.25;
        if (!footprint) {
            return new CenterOfMassDiagnostic(Double.NaN, Double.NaN, false, false);
        }
        double span = maximumZ - minimumZ;
        double stationTolerance = Math.max(0.05, span * 0.05);
        double rearMinX = Double.POSITIVE_INFINITY, rearMaxX = Double.NEGATIVE_INFINITY;
        double frontMinX = Double.POSITIVE_INFINITY, frontMaxX = Double.NEGATIVE_INFINITY;
        for (Vec3d point : wheelPoints) {
            if (Math.abs(point.z() - minimumZ) <= stationTolerance) {
                rearMinX = Math.min(rearMinX, point.x()); rearMaxX = Math.max(rearMaxX, point.x());
            }
            if (Math.abs(point.z() - maximumZ) <= stationTolerance) {
                frontMinX = Math.min(frontMinX, point.x()); frontMaxX = Math.max(frontMaxX, point.x());
            }
        }
        double rearTrack = Double.isFinite(rearMinX) && Double.isFinite(rearMaxX) ? rearMaxX - rearMinX : 0.0;
        double frontTrack = Double.isFinite(frontMinX) && Double.isFinite(frontMaxX) ? frontMaxX - frontMinX : 0.0;
        boolean tricycleLayout = rearTrack > Math.max(0.25, frontTrack * 2.0);
        return new CenterOfMassDiagnostic(minimumZ, maximumZ, true, tricycleLayout);
    }

    public enum DerivedCenterOfMassConfiguration {
        ROTORCRAFT_MODEL_REFERENCE,
        CONVENTIONAL_HORIZONTAL_TAIL,
        TAILLESS_ELEVON_MODEL,
        UNRESOLVED_SAFE_FORWARD_MARGIN
    }

    public record DerivedCenterOfMass(
        Vec3d position,
        DerivedCenterOfMassConfiguration configuration,
        double aerodynamicCandidateZ,
        boolean gearClamped,
        double targetMacPercent,
        String targetReference
    ) {
    }

    public record CenterOfMassDiagnostic(
        double minimumGearZ,
        double maximumGearZ,
        boolean hasLongitudinalFootprint,
        boolean tricycleLayout
    ) {
    }

    static boolean vehicleHasRotorPart(EntityVehicleF_Physics vehicle) {
        for (APart part : vehicle.allParts) {
            if (part instanceof PartPropeller propeller
                && propeller.definition.propeller != null
                && propeller.definition.propeller.isRotor) {
                return true;
            }
        }
        return false;
    }

    static void prepareAircraftOnce(
        EntityVehicleF_Physics vehicle,
        Level level,
        PMWeatherIVConfig.Values config,
        AircraftState state
    ) {
        boolean sameParts = state.preparedParts.size() == vehicle.allParts.size();
        if (sameParts) for (APart part : vehicle.allParts) {
            if (!state.preparedParts.contains(part.uniqueUUID)) { sameParts = false; break; }
        }
        String geometrySignature = ModelAnimationHints.modelLocation(vehicle) + "|"
            + ModelCoordinates.scale(vehicle) + "|"
            + AuthoredLiftingSurfacePoses.signature(vehicle) + "|"
            + com.g9third.pmweatheriv.model.ModelPhysicalVisibility.signature(vehicle)
            + "|attached=" + AttachedLiftingSurfaces.signature(vehicle)
            + "|stations=" + config.maximumModelPressurePatches();
        if (state.isPrepared() && sameParts && geometrySignature.equals(state.preparedGeometrySignature)
            && (state.plan.model().modelBased() || vehicle.ticksExisted<state.nextModelRetryTick)) return;
        AirframePlan previousPlan = state.plan;
        java.util.Set<java.util.UUID> parts = new java.util.HashSet<>();
        for (APart part : vehicle.allParts) parts.add(part.uniqueUUID);
        boolean rotorcraft = vehicleHasRotorPart(vehicle);
        ModelSurfaceMap.PreparedModel model = ModelSurfaceMap.prepare(vehicle,
            config.maximumModelPressurePatches(), rotorcraft);
        if (!rotorcraft) model = AttachedLiftingSurfaces.retainMissing(model,
            previousPlan == null ? null : previousPlan.model(), parts);
        if (rotorcraft && !model.liftingPatches().isEmpty()) {
            model = new ModelSurfaceMap.PreparedModel(model.modelLocation(), model.modelBased(),
                model.reason(), model.fullBounds(), model.bodyBounds(), model.pressurePatches(),
                List.of(), model.pressureWettedArea(), model.sourceTriangles(), model.retainedTriangles(),
                model.pressureTarget());
        }
        Geometry geometry = Geometry.from(vehicle, config, model);
        SurfaceAreaPlan areaPlan = SurfaceAreaPlan.from(model, geometry);
        PreparedFixedWingPlan fixedWingPlan = rotorcraft ? PreparedFixedWingPlan.EMPTY
            : prepareFixedWingPlan(vehicle, model, areaPlan, geometry);
        double mass = finiteClamp(vehicle.currentMass, 50.0, 1.0E8,
            Math.max(50.0, vehicle.definition.motorized.emptyMass));
        DerivedCenterOfMass center = deriveCenterOfMass(vehicle, model, geometry, fixedWingPlan, rotorcraft);
        List<ModelSurfaceMap.PressurePatch> pressure = model.pressurePatches();
        if (!model.modelBased() || pressure.isEmpty()) {
            List<ModelSurfaceMap.PressurePatch> inferred = new ArrayList<>();
            for (BodyFace face : bodyFaces(geometry)) {
                inferred.add(new ModelSurfaceMap.PressurePatch(face.name(), face.pointLocal(),
                    face.normalLocal(), face.area(), ModelSurfaceMap.SurfaceKind.BODY));
            }
            pressure = List.copyOf(inferred);
        }
        List<PreparedRotorDisc> preparedRotors = prepareRotorDiscs(vehicle);
        state.plan = new AirframePlan(model, geometry, areaPlan, fixedWingPlan,
            geometry.inertiaForMass(mass),
            previousPlan == null ? center.position() : previousPlan.centerOfMassLocal(),
            rotorcraft, preparedRotors, pressure);
        state.preparedParts = java.util.Set.copyOf(parts);
        state.preparedGeometrySignature = geometrySignature;
        state.nextModelRetryTick = vehicle.ticksExisted+40;
        state.windSnapshot = null;
        state.windGameTime = Long.MIN_VALUE;
        state.liftingPoses = AuthoredLiftingSurfacePoses.Snapshot.EMPTY;
        state.liftingPoseModel = null;
        state.liftingPoseTick = Long.MIN_VALUE;
        if (previousPlan == null || previousPlan.rotorcraft() != rotorcraft) {
            state.surfaceSeparationFractions.clear();
            state.surfaceSeparationTargets.clear();
        }
        // A structural rotorcraft refresh may mean a rotor was removed/replaced.
        // Never carry the previous actuator's filtered lift into the new plan.
        if (rotorcraft) {
            state.rotorcraftMainThrustNewtons = Double.NaN;
        }
        state.pendingSeparation.clear();
        if (PMIVObserver.loggingEnabled()) PMIVObserver.log("AIRFRAME_PREPARED uuid=" + vehicle.uniqueUUID
            + " model=" + model.modelLocation() + " pressurePoints=" + pressure.size()
            + " liftingPoints=" + model.liftingPatches().size() + " rotor=" + rotorcraft
            + " wingAreaM2=" + areaPlan.totalArea(ModelSurfaceMap.SurfaceKind.WING)
            + " taileronAreaM2=" + areaPlan.totalArea(ModelSurfaceMap.SurfaceKind.TAILERON)
            + " horizontalTailAreaM2=" + areaPlan.totalArea(ModelSurfaceMap.SurfaceKind.HORIZONTAL_TAIL)
            + " elevatorAreaM2=" + areaPlan.totalArea(ModelSurfaceMap.SurfaceKind.ELEVATOR)
            + " centerOfMass=" + center.position() + " cgPolicy=" + center.configuration());
    }

    /**
     * Freezes relationships that depend only on prepared aircraft geometry.
     * Live substeps must evaluate only pose/velocity/wind/control/stall state,
     * not repeatedly repartition the same immutable mesh.
     */
    static PreparedFixedWingPlan prepareFixedWingPlan(
        EntityVehicleF_Physics vehicle,
        ModelSurfaceMap.PreparedModel model,
        SurfaceAreaPlan areaPlan,
        Geometry geometry
    ) {
        if (model == null || areaPlan == null || geometry == null) {
            return PreparedFixedWingPlan.EMPTY;
        }
        AileronWingCoupling aileronCoupling = wingControlCoupling(
            model, areaPlan, geometry, ModelSurfaceMap.SurfaceKind.AILERON
        );
        AileronWingCoupling elevonCoupling = wingControlCoupling(
            model, areaPlan, geometry, ModelSurfaceMap.SurfaceKind.ELEVON
        );
        Map<ModelSurfaceMap.LiftingPatch, Double> movingAreaByParent = new HashMap<>();
        if (model.modelBased()) {
            for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
                double movingArea = switch (patch.kind()) {
                    case WING -> wingControlAreaForWingPatch(
                        patch, aileronCoupling, areaPlan, ModelSurfaceMap.SurfaceKind.AILERON
                    ) + wingControlAreaForWingPatch(
                        patch, elevonCoupling, areaPlan, ModelSurfaceMap.SurfaceKind.ELEVON
                    );
                    case HORIZONTAL_TAIL -> movingControlAreaForParentPatch(
                        model, areaPlan, patch,
                        ModelSurfaceMap.SurfaceKind.HORIZONTAL_TAIL,
                        ModelSurfaceMap.SurfaceKind.ELEVATOR
                    );
                    case VERTICAL_TAIL -> movingControlAreaForParentPatch(
                        model, areaPlan, patch,
                        ModelSurfaceMap.SurfaceKind.VERTICAL_TAIL,
                        ModelSurfaceMap.SurfaceKind.RUDDER
                    );
                    default -> 0.0;
                };
                if (movingArea > EPSILON) {
                    movingAreaByParent.put(patch, movingArea);
                }
            }
        }

        double elevatorEffectiveness;
        double rudderEffectiveness;
        if (model.modelBased()) {
            double horizontalParentArea =
                areaPlan.totalArea(ModelSurfaceMap.SurfaceKind.HORIZONTAL_TAIL)
                    + areaPlan.totalArea(ModelSurfaceMap.SurfaceKind.ELEVATOR)
                    + areaPlan.totalArea(ModelSurfaceMap.SurfaceKind.TAILERON);
            double verticalParentArea =
                areaPlan.totalArea(ModelSurfaceMap.SurfaceKind.VERTICAL_TAIL)
                    + areaPlan.totalArea(ModelSurfaceMap.SurfaceKind.RUDDER);
            elevatorEffectiveness = controlSurfaceEffectiveness(
                geometry.elevatorArea(), horizontalParentArea
            );
            rudderEffectiveness = controlSurfaceEffectiveness(
                geometry.rudderArea(), verticalParentArea
            );
        } else {
            elevatorEffectiveness = controlSurfaceEffectiveness(
                geometry.elevatorArea(), geometry.horizontalTailArea()
            );
            rudderEffectiveness = controlSurfaceEffectiveness(
                geometry.rudderArea(), geometry.verticalTailArea()
            );
        }

        AnimatedWingGeometry animatedWingGeometry = model.modelBased()
            ? AnimatedWingGeometry.detect(vehicle)
            : AnimatedWingGeometry.empty();
        return new PreparedFixedWingPlan(
            aileronCoupling,
            elevonCoupling,
            Map.copyOf(movingAreaByParent),
            animatedWingGeometry,
            mainWingAerodynamicCenterReferenceZ(model, areaPlan),
            Math.max(0.05, geometry.wingArea() / Math.max(0.5, geometry.wingSpan())),
            Math.max(0.05, geometry.horizontalTailArea() / Math.max(0.5, geometry.horizontalTailSpan())),
            Math.max(0.05, Math.sqrt(Math.max(0.01, geometry.verticalTailArea()))),
            controlSurfaceEffectiveness(geometry.aileronArea(), geometry.wingArea()),
            elevatorEffectiveness,
            rudderEffectiveness
        );
    }
}
