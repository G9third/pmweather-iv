package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import com.g9third.pmweatheriv.compat.PMAeroBridge;
import com.g9third.pmweatheriv.mixin.PartEngineAccessor;
import com.g9third.pmweatheriv.mixin.EntityVehiclePhysicsAccessor;
import com.g9third.pmweatheriv.sable.SableModelCollisionHull;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import minecrafttransportsimulator.entities.instances.PartEngine;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import com.g9third.pmweatheriv.model.ModelPhysicalVisibility;
import net.minecraft.server.level.ServerLevel;

import static com.g9third.pmweatheriv.physics.FlightMath.IV_TICKS_SQUARED_TO_SECONDS_SQUARED;
import static com.g9third.pmweatheriv.physics.FlightMath.MC_TICK_SECONDS;
import static com.g9third.pmweatheriv.physics.FlightMath.MPH_TO_METERS_PER_SECOND;
import static com.g9third.pmweatheriv.physics.FlightMath.RAD_TO_DEG;
import static com.g9third.pmweatheriv.physics.FlightMath.activePositionWorld;
import static com.g9third.pmweatheriv.physics.FlightMath.ivCompatibleAirDensity;
import static com.g9third.pmweatheriv.physics.FlightMath.point;
import static com.g9third.pmweatheriv.physics.FlightMath.toLocal;
import static com.g9third.pmweatheriv.physics.FlightMath.toWorld;

/**
 * Model-surface aerodynamics and COM momentum for native-IV road vehicles.
 * IV retains drivetrain and solid collision; PMIV supplies bounded tire contacts.
 * Wheeled bodies replace native air drag with complete PMAero relative-air loads.
 * Other native ground bodies retain their additive ambient-wind integration.
 */
public final class GroundVehicleWind {
    private GroundVehicleWind() {
    }

    public static final class State {
        private SableBodyPressureGeometry.PreparedBody bodyGeometry;
        private List<ModelSurfaceMap.PressurePatch> patches = List.of();
        private double bodyWidth;
        private double bodyHeight;
        private double bodyLength;
        private double wettedArea;
        private Vec3d inertiaPerKgAboutCenter = Vec3d.ZERO;
        private Vec3d centerOfMassLocal = Vec3d.ZERO;
        private String geometryVisibility = "";
        private Vec3d stagedMoveOffset = Vec3d.ZERO;
        private Vec3d stagedOriginMotion = Vec3d.ZERO;
        private Vec3d stagedCenterWorld = Vec3d.ZERO;
        private Vec3d stagedOmegaWorld = Vec3d.ZERO;
        private long stagedMoveTick = Long.MIN_VALUE;
        private List<TireContactSnapshot> tireContacts = List.of();
        private List<WindStationSnapshot> windStations = List.of();
        private List<RoadRoofPressure.Panel> roofPanels = List.of();
        private int geometryPreparationAttempts;
        private long nextGeometryRetryTick = Long.MIN_VALUE;

        // PMWeather-created angular momentum is intentionally separate from IV's steering
        // request. It is stored in world coordinates so an airborne vehicle conserves
        // angular momentum as its body axes rotate. IV ground/collision corrections may
        // remove this momentum through the post-ground/post-move feedback hooks below,
        // while tire impulses exchange linear and pitch/roll/yaw momentum.
        private Vec3d angularMomentumWorld = Vec3d.ZERO;
        private Vec3d lastInertiaKgM2 = Vec3d.ZERO;
        private Vec3d lastExternalAngularVelocityBodyRadps = Vec3d.ZERO;
        private long pendingAngularTick = Long.MIN_VALUE;
        private Vec3d pendingExternalRequestDegrees = Vec3d.ZERO;
        private Vec3d pendingRetainedRequestDegrees = Vec3d.ZERO;
        private Vec3d pendingAfterGroundRotationDegrees = Vec3d.ZERO;
        private Vec3d pendingBodyAngularMomentum = Vec3d.ZERO;
        private Vec3d pendingBodyAxisXWorld = new Vec3d(1, 0, 0);
        private Vec3d pendingBodyAxisYWorld = new Vec3d(0, 1, 0);
        private Vec3d pendingBodyAxisZWorld = new Vec3d(0, 0, 1);

        private double[] windInput = new double[0];
        private double[] windOutput = new double[0];
        private double[] windBodyInput = new double[0];
        private double[] stillBodyInput = new double[0];
        private double[] windBodyOutput = new double[0];
        private double[] stillBodyOutput = new double[0];
        private long lastDebugTick = Long.MIN_VALUE;
        private long tireSolverTick = Long.MIN_VALUE;
        private Result currentResult;
    }

    public record TireContactSnapshot(Vec3d pointLocal, String material, boolean wet,
        double motiveMu, double lateralMu, double normalLoadN, double gapMeters,
        double longitudinalImpulseNs, double lateralImpulseNs,
        double normalVelocityTargetMps, double normalSoftnessInverseKg,
        double supportStiffnessNpm, double supportDampingNsPm, double nominalDeflectionMeters,
        double steeringDegrees, double nativeDriveDemandImpulseNs, boolean closedThrottleEngineBraking,
        double engineBrakeCoefficientLimit, double rollingResistanceCoefficient) {}

    public record WindStationSnapshot(String name, Vec3d forcePointLocal, Vec3d samplePointLocal,
                                      Vec3d samplePositionWorld, Vec3d rawMph, double areaM2,
                                      Vec3d roofPointLocal, Vec3d roofNormalLocal, double roofAreaM2,
                                      Vec3d roofSuctionForceBodyN) {
        public WindStationSnapshot(String name, Vec3d forcePointLocal, Vec3d samplePointLocal,
                                   Vec3d samplePositionWorld, Vec3d rawMph, double areaM2) {
            this(name,forcePointLocal,samplePointLocal,samplePositionWorld,rawMph,areaM2,
                Vec3d.ZERO,Vec3d.ZERO,0,Vec3d.ZERO);
        }
    }
    public static List<WindStationSnapshot> windStations(EntityVehicleF_Physics vehicle) {
        return vehicle instanceof GroundVehicleWindStateAccess access
            ? access.pmweatherIv$getGroundWindState().windStations : List.of();
    }

    public static Vec3d centerOfMassLocal(EntityVehicleF_Physics vehicle) {
        return vehicle instanceof GroundVehicleWindStateAccess access
            ? access.pmweatherIv$getGroundWindState().centerOfMassLocal : Vec3d.ZERO;
    }
    public static List<TireContactSnapshot> tireContacts(EntityVehicleF_Physics vehicle) {
        return vehicle instanceof GroundVehicleWindStateAccess access
            ? access.pmweatherIv$getGroundWindState().tireContacts : List.of();
    }

    public record TireYawReaction(
        double effectiveNormalLoadNewtons,
        double yawTorqueCapacityNewtonMeters,
        double angularImpulseRemovedNewtonMeterSeconds,
        int groundedContactBoxes,
        boolean saturated
    ) {
        static TireYawReaction none() {
            return new TireYawReaction(0.0, 0.0, 0.0, 0, false);
        }
    }

    public record BodyAerodynamics(boolean fullRelativeAir, double axialPressureCoefficient,
        double crossflowPressureCoefficient, Vec3d restoredNativeDragImpulseWorldNs,
        Vec3d centerOfMassVelocityBeforeForceWorldMps, Vec3d centerOfMassVelocityAfterForceWorldMps,
        double roofSuctionPressureCoefficient, Vec3d roofSuctionForceBodyN, Vec3d roofSuctionTorqueBodyNm) {
        public BodyAerodynamics(boolean fullRelativeAir, double axialPressureCoefficient,
                double crossflowPressureCoefficient, Vec3d restoredNativeDragImpulseWorldNs,
                Vec3d centerOfMassVelocityBeforeForceWorldMps, Vec3d centerOfMassVelocityAfterForceWorldMps) {
            this(fullRelativeAir,axialPressureCoefficient,crossflowPressureCoefficient,
                restoredNativeDragImpulseWorldNs,centerOfMassVelocityBeforeForceWorldMps,
                centerOfMassVelocityAfterForceWorldMps,0,Vec3d.ZERO,Vec3d.ZERO);
        }
    }

    public record Result(
        Vec3d forceWorldNewtons,
        Vec3d torqueBodyNewtonMeters,
        Vec3d meanWindMph,
        int patchCount,
        boolean modelBased,
        Vec3d inertiaKgM2,
        Vec3d externalAngularVelocityBeforeTorqueBodyRadps,
        Vec3d externalAngularVelocityAfterTorqueBodyRadps,
        Vec3d externalAngularMomentumAfterTorqueWorldNms,
        TireYawReaction tireYawReaction,
        Vec3d externalAngularVelocityAfterTireReactionBodyRadps,
        Vec3d externalAngularMomentumAfterTireReactionWorldNms,
        Vec3d externalRotationRequestDegreesPerTick,
        BodyAerodynamics bodyAerodynamics
    ) {
    }

    public record Telemetry(
        Vec3d inertiaKgM2,
        Vec3d externalAngularMomentumWorldNms,
        Vec3d externalAngularVelocityBodyRadps
    ) {
    }

    public static Result apply(
        EntityVehicleF_Physics vehicle,
        ServerLevel level,
        PMWeatherIVConfig.Values config,
        State state
    ) {
        if (vehicle == null || level == null || config == null || state == null
            || vehicle.definition == null || vehicle.definition.motorized == null
            || vehicle.definition.motorized.isAircraft || vehicle.lockedOnRoad) {
            return null;
        }

        state.tireContacts = List.of();
        state.windStations = List.of();
        ensurePrepared(vehicle, config, state);
        if (state.patches.isEmpty()) {
            return null;
        }

        final int patchCount = state.patches.size();
        final int windLength = patchCount * 3;
        final int windOutputLength = patchCount * PMAeroBridge.WIND_STRIDE;
        final int bodyInputLength = patchCount * PMAeroBridge.bodyInputStride();
        final int bodyOutputLength = PMAeroBridge.bodyOutputHeaderStride()
            + patchCount * PMAeroBridge.bodyOutputPatchStride();
        state.windInput = resize(state.windInput, windLength);
        state.windOutput = resize(state.windOutput, windOutputLength);
        state.windBodyInput = resize(state.windBodyInput, bodyInputLength);
        state.stillBodyInput = resize(state.stillBodyInput, bodyInputLength);
        state.windBodyOutput = resize(state.windBodyOutput, bodyOutputLength);
        state.stillBodyOutput = resize(state.stillBodyOutput, bodyOutputLength);

        Vec3d origin = activePositionWorld(vehicle);
        for (int i = 0; i < patchCount; ++i) {
            ModelSurfaceMap.PressurePatch patch = state.patches.get(i);
            Vec3d world = ModelCoordinates.worldPoint(origin,vehicle.orientation,patch.windSamplePointLocal());
            int wind = i * 3;
            state.windInput[wind] = world.x();
            state.windInput[wind + 1] = world.y();
            state.windInput[wind + 2] = world.z();
        }
        PMAeroBridge.sampleAircraftAtmosphereInto(level, state.windInput, state.windOutput);
        List<WindStationSnapshot> stations=new ArrayList<>(patchCount);
        for (int i=0; i<patchCount; ++i) {
            var patch=state.patches.get(i); int xyz=i*3, out=i*PMAeroBridge.WIND_STRIDE;
            stations.add(new WindStationSnapshot(patch.name(),patch.pointLocal(),patch.windSamplePointLocal(),
                new Vec3d(state.windInput[xyz],state.windInput[xyz+1],state.windInput[xyz+2]),
                new Vec3d(state.windOutput[out],state.windOutput[out+1],state.windOutput[out+2]),patch.area()));
        }
        state.windStations=List.copyOf(stations);

        // IV motion is an internal blocks/tick value whose actual displacement is
        // motion*speedFactor.  Convert the rendered/model-origin velocity to m/s.
        double speedFactor = Math.max(1.0E-6, Math.abs(vehicle.speedFactor));
        Vec3d linearVelocityWorld = point(vehicle.motion).scale(speedFactor / MC_TICK_SECONDS);
        double mass = Double.isFinite(vehicle.currentMass) && vehicle.currentMass > 1.0E-6
            ? vehicle.currentMass
            : Math.max(1.0, vehicle.definition.motorized.emptyMass);
        Vec3d inertia = inertiaForMass(state.inertiaPerKgAboutCenter, mass, state.bodyWidth, state.bodyHeight, state.bodyLength);
        state.lastInertiaKgM2 = inertia;
        boolean fullRelativeAir = hasInstalledTires(vehicle) && vehicle.towedByConnection == null
            && !vehicle.outOfHealth;
        Vec3d restoredNativeDragImpulse = Vec3d.ZERO;
        if (fullRelativeAir) {
            // IV has already integrated -normalizedVelocity*dragForce/mass in
            // internal units. Undo only that air-drag increment in the working
            // velocity; commit after the replacement aero result is finite.
            EntityVehiclePhysicsAccessor access = (EntityVehiclePhysicsAccessor) vehicle;
            double drag = access.pmweatherIv$getDragForce();
            Vec3d direction = point(access.pmweatherIv$getNormalizedVelocityVector());
            if (!Double.isFinite(drag) || !direction.isFinite())
                throw new IllegalStateException("Non-finite native road air drag");
            restoredNativeDragImpulse = direction.scale(drag*speedFactor/MC_TICK_SECONDS);
            linearVelocityWorld = linearVelocityWorld.add(restoredNativeDragImpulse.scale(1/mass));
        }
        double axialCd = fullRelativeAir ? roadAxialCoefficient(vehicle, config) : config.bodyAxialDragCoefficient();
        double crossflowCd = fullRelativeAir ? config.groundCrossflowDragCoefficient() : config.bodyCrossflowDragCoefficient();
        // The required API attenuates its axial input by frozen fineness. Cancel
        // that attenuation for a road body's authored frontal coefficient while
        // retaining the real dimensions for skin-friction Reynolds evaluation.
        double axialApiCd = fullRelativeAir
            ? axialCd/axialPressureScale(state.bodyWidth, state.bodyHeight, state.bodyLength) : axialCd;

        // IV's native rotation request (steering/skid/etc.) remains native IV. PMWeather's
        // separate external angular momentum is added only for the local point velocity seen
        // by PMAero and later as an additive rotation request. Storing the external momentum
        // in world space prevents it from being reset simply because IV rebuilds rotation.angles
        // every tick.
        Vec3d externalOmegaBeforeTorqueBody = externalAngularVelocityBody(vehicle, state, inertia);
        Vec3d nativeOmegaBody = new Vec3d(
            Math.toRadians(vehicle.rotation.angles.x) / MC_TICK_SECONDS,
            Math.toRadians(vehicle.rotation.angles.y) / MC_TICK_SECONDS,
            Math.toRadians(vehicle.rotation.angles.z) / MC_TICK_SECONDS
        );
        Vec3d omegaBody = nativeOmegaBody.add(externalOmegaBeforeTorqueBody);
        Vec3d omegaWorld = toWorld(vehicle, omegaBody);
        linearVelocityWorld = linearVelocityWorld.add(omegaWorld.cross(toWorld(vehicle, state.centerOfMassLocal)));
        Vec3d centerVelocityBeforeForce = linearVelocityWorld;
        Vec3d meanWindMph = Vec3d.ZERO;

        final int inputStride = PMAeroBridge.bodyInputStride();
        for (int i = 0; i < patchCount; ++i) {
            ModelSurfaceMap.PressurePatch patch = state.patches.get(i);
            int wind = i * PMAeroBridge.WIND_STRIDE;
            Vec3d rawMph = new Vec3d(
                state.windOutput[wind],
                state.windOutput[wind + 1],
                state.windOutput[wind + 2]
            );
            meanWindMph = meanWindMph.add(rawMph);
            Vec3d windMetersPerSecond = rawMph.scale(MPH_TO_METERS_PER_SECOND);
            Vec3d leverWorld = toWorld(vehicle, patch.pointLocal().subtract(state.centerOfMassLocal));
            Vec3d pointVelocityWorld = linearVelocityWorld.add(omegaWorld.cross(leverWorld));
            Vec3d windRelativeLocal = toLocal(
                vehicle, pointVelocityWorld.subtract(windMetersPerSecond)
            );
            Vec3d stillRelativeLocal = toLocal(vehicle, pointVelocityWorld);
            Vec3d normal = patch.normalLocal().normalized();
            int base = i * inputStride;
            packPatch(state.windBodyInput, base, patch, normal, windRelativeLocal);
            packPatch(state.stillBodyInput, base, patch, normal, stillRelativeLocal);
        }
        meanWindMph = meanWindMph.scale(1.0 / patchCount);

        double density = ivCompatibleAirDensity(level, vehicle, origin.y());
        PMAeroBridge.evaluateExternalBodyInto(
            state.windBodyInput, state.windBodyOutput, density,
            axialApiCd, crossflowCd,
            state.bodyWidth, state.bodyHeight, state.bodyLength, state.wettedArea,
            state.centerOfMassLocal
        );
        if (!fullRelativeAir) {
            PMAeroBridge.evaluateExternalBodyInto(
                state.stillBodyInput, state.stillBodyOutput, density,
                axialApiCd, crossflowCd, state.bodyWidth, state.bodyHeight, state.bodyLength,
                state.wettedArea, state.centerOfMassLocal
            );
        }

        Vec3d forceBody = fullRelativeAir ? header(state.windBodyOutput, 0)
            : headerDelta(state.windBodyOutput, state.stillBodyOutput, 0);
        Vec3d torqueBody = fullRelativeAir ? header(state.windBodyOutput, 3)
            : headerDelta(state.windBodyOutput, state.stillBodyOutput, 3);
        Vec3d roofForce=Vec3d.ZERO, roofTorque=Vec3d.ZERO;
        List<WindStationSnapshot> loadStations=new ArrayList<>(state.windStations.size());
        for (int i=0;i<patchCount;i++) {
            var station=state.windStations.get(i);
            var panel=state.roofPanels.size()==patchCount ? state.roofPanels.get(i)
                : new RoadRoofPressure.Panel(station.forcePointLocal(),Vec3d.ZERO,0);
            Vec3d partForce=Vec3d.ZERO;
            if (fullRelativeAir && panel.areaM2()>0) {
                int offset=i*inputStride+8;
                Vec3d flow=new Vec3d(state.windBodyInput[offset],state.windBodyInput[offset+1],state.windBodyInput[offset+2]);
                flow=flow.add(omegaBody.cross(panel.pointLocal().subtract(state.patches.get(i).pointLocal())));
                partForce=RoadRoofPressure.force(panel,flow,density,config.roadRoofSuctionPressureCoefficient());
                roofForce=roofForce.add(partForce);
                roofTorque=roofTorque.add(panel.pointLocal().subtract(state.centerOfMassLocal).cross(partForce));
            }
            loadStations.add(new WindStationSnapshot(station.name(),station.forcePointLocal(),station.samplePointLocal(),
                station.samplePositionWorld(),station.rawMph(),station.areaM2(),panel.pointLocal(),panel.normalLocal(),
                panel.areaM2(),partForce));
        }
        state.windStations=List.copyOf(loadStations);
        forceBody=forceBody.add(roofForce);
        torqueBody=torqueBody.add(roofTorque);
        if (!forceBody.isFinite() || !torqueBody.isFinite()) {
            throw new IllegalStateException("Non-finite ground-vehicle PMAero wind result");
        }
        Vec3d forceWorld = toWorld(vehicle, forceBody);

        // Integrate force at the physical COM, then torque. A change of omega
        // must not manufacture translation through IV's offset model origin.
        Vec3d centerVelocityAfterForce = centerVelocityBeforeForce
            .add(forceWorld.scale(MC_TICK_SECONDS/mass));
        Vec3d torqueWorld = toWorld(vehicle, torqueBody);
        Vec3d angularMomentumAfterTorqueWorld = state.angularMomentumWorld.add(torqueWorld.scale(MC_TICK_SECONDS));
        if (!angularMomentumAfterTorqueWorld.isFinite() || !centerVelocityAfterForce.isFinite())
            throw new IllegalStateException("Non-finite road COM momentum");
        Vec3d momentumBody = toLocal(vehicle, angularMomentumAfterTorqueWorld);
        Vec3d externalOmegaAfterTorqueBody = new Vec3d(momentumBody.x()/inertia.x(),
            momentumBody.y()/inertia.y(), momentumBody.z()/inertia.z());
        Vec3d newOmegaWorld = toWorld(vehicle, nativeOmegaBody.add(externalOmegaAfterTorqueBody));
        Vec3d originVelocityAfterForce = centerVelocityAfterForce.subtract(
            newOmegaWorld.cross(toWorld(vehicle, state.centerOfMassLocal)));
        if (!originVelocityAfterForce.isFinite())
            throw new IllegalStateException("Non-finite road origin velocity");
        state.angularMomentumWorld = angularMomentumAfterTorqueWorld;
        vehicle.motion.set(originVelocityAfterForce.x()*MC_TICK_SECONDS/speedFactor,
            originVelocityAfterForce.y()*MC_TICK_SECONDS/speedFactor,
            originVelocityAfterForce.z()*MC_TICK_SECONDS/speedFactor);
        BodyAerodynamics bodyAerodynamics = new BodyAerodynamics(fullRelativeAir, axialCd, crossflowCd,
            restoredNativeDragImpulse, centerVelocityBeforeForce, centerVelocityAfterForce,
            fullRelativeAir ? config.roadRoofSuctionPressureCoefficient() : 0,roofForce,roofTorque);

        // Stage the aerodynamic request. Tires run at performGroundOperations HEAD so
        // traces retain a genuine after-wind/before-tires snapshot.
        TireYawReaction tireYawReaction = TireYawReaction.none();
        Vec3d externalOmegaAfterTireBody = externalOmegaAfterTorqueBody;
        Vec3d externalRequestDegrees = externalOmegaAfterTorqueBody.scale(RAD_TO_DEG * MC_TICK_SECONDS);
        beginAngularFeedback(vehicle, state, inertia, externalRequestDegrees);
        vehicle.rotation.angles.add(
            externalRequestDegrees.x(),
            externalRequestDegrees.y(),
            externalRequestDegrees.z()
        );
        state.lastExternalAngularVelocityBodyRadps = externalOmegaAfterTireBody;

        Result result = new Result(
            forceWorld, torqueBody, meanWindMph, patchCount,
            state.bodyGeometry != null && state.bodyGeometry.modelBased(),
            inertia, externalOmegaBeforeTorqueBody, externalOmegaAfterTorqueBody,
            angularMomentumAfterTorqueWorld, tireYawReaction,
            externalOmegaAfterTireBody, state.angularMomentumWorld, externalRequestDegrees, bodyAerodynamics
        );
        state.currentResult = result;
        return result;
    }

    private static void logTires(EntityVehicleF_Physics vehicle, State state, Result result) {
        PMWeatherIVConfig.Values config = PMWeatherIVConfig.get();
        double mass = vehicle.currentMass;
        double speedFactor = vehicle.speedFactor;
        int patchCount = result.patchCount();
        Vec3d meanWindMph = result.meanWindMph();
        Vec3d forceWorld = result.forceWorldNewtons();
        Vec3d torqueBody = result.torqueBodyNewtonMeters();
        Vec3d inertia = result.inertiaKgM2();
        TireYawReaction tireYawReaction = result.tireYawReaction();
        Vec3d externalOmegaAfterTireBody = result.externalAngularVelocityAfterTireReactionBodyRadps();
        Vec3d externalRequestDegrees = result.externalRotationRequestDegreesPerTick();
        if (PMIVObserver.loggingEnabled()
            && (state.lastDebugTick == Long.MIN_VALUE || vehicle.ticksExisted - state.lastDebugTick >= 20)) {
            state.lastDebugTick = vehicle.ticksExisted;
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(String.format(
                Locale.ROOT,
                "GROUND_VEHICLE_WIND uuid=%s model=%s patches=%d modelBased=%s massKg=%.4f speedFactor=%.6f meanWindMph=(%.4f,%.4f,%.4f) forceN=(%.4f,%.4f,%.4f) torqueNmBody=(%.4f,%.4f,%.4f) inertiaKgM2=(%.4f,%.4f,%.4f) tireYawNormalLoadN=%.4f tireYawCapacityNm=%.4f tireYawImpulseRemovedNms=%.4f tireGroundedBoxes=%d tireYawSaturated=%s externalOmegaBodyRadps=(%.5f,%.5f,%.5f) externalRotationDegPerTick=(%.5f,%.5f,%.5f) authority=IV_DRIVETRAIN_SOLID_COLLISION_PMIV_COUPLED_3D_TIRES sable=false bodyFullRelativeAir=%s bodyAxialCd=%.5f bodyCrossflowCd=%.5f",
                vehicle.uniqueUUID,
                state.bodyGeometry == null ? "fallback" : ModelSurfaceMap.safe(state.bodyGeometry.modelLocation()),
                patchCount,
                result.modelBased(),
                mass,
                speedFactor,
                meanWindMph.x(), meanWindMph.y(), meanWindMph.z(),
                forceWorld.x(), forceWorld.y(), forceWorld.z(),
                torqueBody.x(), torqueBody.y(), torqueBody.z(),
                inertia.x(), inertia.y(), inertia.z(),
                tireYawReaction.effectiveNormalLoadNewtons(),
                tireYawReaction.yawTorqueCapacityNewtonMeters(),
                tireYawReaction.angularImpulseRemovedNewtonMeterSeconds(),
                tireYawReaction.groundedContactBoxes(),
                tireYawReaction.saturated(),
                externalOmegaAfterTireBody.x(), externalOmegaAfterTireBody.y(), externalOmegaAfterTireBody.z(),
                externalRequestDegrees.x(), externalRequestDegrees.y(), externalRequestDegrees.z(),
                result.bodyAerodynamics().fullRelativeAir(), result.bodyAerodynamics().axialPressureCoefficient(),
                result.bodyAerodynamics().crossflowPressureCoefficient()
            ));
            }
        }
    }

    private static void ensurePrepared(
        EntityVehicleF_Physics vehicle,
        PMWeatherIVConfig.Values config,
        State state
    ) {
        String visibility = ModelPhysicalVisibility.signature(vehicle) + "|" + vehicle.scale
            + "|" + vehicle.definition.getModelLocation(vehicle.subDefinition)
            + "|stations=" + config.maximumGroundPressurePatches();
        boolean geometryChanged = !visibility.equals(state.geometryVisibility);
        if (!geometryChanged && state.bodyGeometry != null
            && state.bodyGeometry.modelBased()
            && !state.patches.isEmpty()) {
            return;
        }
        if (!geometryChanged && state.bodyGeometry != null && vehicle.ticksExisted < state.nextGeometryRetryTick) {
            return;
        }

        // Use Sable's model parser/rasterization pipeline, but a body-pressure-specific shell:
        // animated exterior body panels remain represented and collision-only rolling-gear
        // corridors are not carved out of the aerodynamic envelope.  Failed preparation is
        // deliberately retried; it is never cached/latching for the life of a fresh entity.
        SableModelCollisionHull.PreparedHull hull = SableModelCollisionHull.prepareAerodynamicBody(vehicle);
        SableBodyPressureGeometry.PreparedBody candidate = SableBodyPressureGeometry.prepare(
            hull, config.maximumGroundPressurePatches()
        );
        state.roofPanels=RoadRoofPressure.prepare(candidate.modelBased() ? candidate.patches() : List.of(),
            hull==null ? List.of() : hull.sampleSurfaceTriangles());
        Vec3d oldCenter = state.centerOfMassLocal;
        Vec3d oldOmegaWorld = toWorld(vehicle,externalAngularVelocityBody(vehicle,state,state.lastInertiaKgM2));
        boolean hadDynamics = state.lastInertiaKgM2.x()>0 && state.lastInertiaKgM2.y()>0 && state.lastInertiaKgM2.z()>0;
        state.bodyGeometry = candidate;
        state.geometryVisibility = visibility;
        if (candidate.modelBased() && !candidate.patches().isEmpty()) {
            state.patches = candidate.patches();
            state.bodyWidth = Math.max(0.05, candidate.width());
            state.bodyHeight = Math.max(0.05, candidate.height());
            state.bodyLength = Math.max(0.05, candidate.length());
            state.wettedArea = Math.max(0.01, candidate.wettedArea());
            state.inertiaPerKgAboutCenter = sanitizeInertiaPerKg(candidate.inertiaPerKgAboutCenter());
            state.centerOfMassLocal = candidate.centerOfMassLocal();
            state.geometryPreparationAttempts = 0;
            state.nextGeometryRetryTick = Long.MAX_VALUE;
        } else {
            FallbackBody fallback = fallbackBody(vehicle);
            state.patches = fallback.patches();
            state.bodyWidth = fallback.width();
            state.bodyHeight = fallback.height();
            state.bodyLength = fallback.length();
            state.wettedArea = fallback.wettedArea();
            state.inertiaPerKgAboutCenter = fallback.inertiaPerKgAboutCenter();
            state.centerOfMassLocal = vehicle.encompassingBox == null ? Vec3d.ZERO : point(vehicle.encompassingBox.localCenter);
            state.geometryPreparationAttempts = Math.min(30, state.geometryPreparationAttempts + 1);
            long delay = geometryRetryDelayTicks(state.geometryPreparationAttempts);
            state.nextGeometryRetryTick = vehicle.ticksExisted + delay;
        }
        if (hadDynamics) {
            double factor = Math.max(1e-6,Math.abs(vehicle.speedFactor));
            Vec3d oldCOMVelocity = point(vehicle.motion).scale(factor/MC_TICK_SECONDS)
                .add(oldOmegaWorld.cross(toWorld(vehicle,oldCenter)));
            Vec3d nextInertia = inertiaForMass(state.inertiaPerKgAboutCenter,vehicle.currentMass,
                state.bodyWidth,state.bodyHeight,state.bodyLength);
            Vec3d newOriginVelocity = oldCOMVelocity.subtract(toWorld(vehicle,
                externalAngularVelocityBody(vehicle,state,nextInertia)).cross(toWorld(vehicle,state.centerOfMassLocal)));
            vehicle.motion.set(newOriginVelocity.x()*MC_TICK_SECONDS/factor,
                newOriginVelocity.y()*MC_TICK_SECONDS/factor,newOriginVelocity.z()*MC_TICK_SECONDS/factor);
        }
        if (PMIVObserver.loggingEnabled()) {
            PMIVObserver.log(
            "GROUND_VEHICLE_WIND_PREPARED uuid=" + vehicle.uniqueUUID
                + " model=" + (state.bodyGeometry == null ? "fallback" : ModelSurfaceMap.safe(state.bodyGeometry.modelLocation()))
                + " modelBased=" + (state.bodyGeometry != null && state.bodyGeometry.modelBased())
                + " geometryAuthority=" + (state.bodyGeometry != null && state.bodyGeometry.modelBased()
                    ? "SABLE_MODEL_BODY_PRESSURE_HULL_EXPOSED_FACES" : "IV_ENCOMPASSING_BOX_FALLBACK_RETRIABLE")
                + " pressurePatches=" + state.patches.size()
                + " sableShellCells=" + (state.bodyGeometry == null ? 0 : state.bodyGeometry.shellCells())
                + " sableExteriorSilhouetteFaces=" + (state.bodyGeometry == null ? 0 : state.bodyGeometry.exteriorSilhouetteFaces())
                + " sableMergedCuboids=" + (state.bodyGeometry == null ? 0 : state.bodyGeometry.mergedCuboids())
                + " sableResolution=" + (state.bodyGeometry == null ? 0.0 : state.bodyGeometry.resolution())
                + " wettedAreaM2=" + String.format(Locale.ROOT, "%.5f", state.wettedArea)
                + " centerOfMassLocal=" + state.centerOfMassLocal
                + " massPolicy=UNIFORM_SOLIDIFIED_VISIBLE_BODY_VOLUME"
                + " inertiaPerKgAboutCOM=" + state.inertiaPerKgAboutCenter
                + " bounds=" + String.format(Locale.ROOT, "%.3fx%.3fx%.3f", state.bodyWidth, state.bodyHeight, state.bodyLength)
                + " prepareAttempt=" + state.geometryPreparationAttempts
                + " nextRetryTick=" + state.nextGeometryRetryTick
                + " integration=IV_NATIVE_EXTERNAL_PMWEATHER_FORCE_PERSISTENT_ANGULAR_MOMENTUM"
                + " sableRigidBody=false"
        );
        }
    }

    private static long geometryRetryDelayTicks(int attempts) {
        int shift = Math.max(0, Math.min(3, attempts - 1));
        return Math.min(200L, 20L << shift);
    }

    private static FallbackBody fallbackBody(EntityVehicleF_Physics vehicle) {
        BoundingBox box = vehicle.encompassingBox;
        Vec3d center = box != null && box.localCenter != null
            ? point(box.localCenter) : Vec3d.ZERO;
        double width = box != null && Double.isFinite(box.widthRadius)
            ? Math.max(0.2, box.widthRadius * 2.0) : 2.0;
        double height = box != null && Double.isFinite(box.heightRadius)
            ? Math.max(0.2, box.heightRadius * 2.0) : 1.5;
        double length = box != null && Double.isFinite(box.depthRadius)
            ? Math.max(0.2, box.depthRadius * 2.0) : 3.0;
        double hx = width * 0.5;
        double hy = height * 0.5;
        double hz = length * 0.5;
        double side = height * length;
        double top = width * length;
        double end = width * height;
        List<ModelSurfaceMap.PressurePatch> patches = List.of(
            new ModelSurfaceMap.PressurePatch("GROUND_LEFT", center.add(new Vec3d(-hx, 0, 0)), new Vec3d(-1, 0, 0), side, ModelSurfaceMap.SurfaceKind.BODY),
            new ModelSurfaceMap.PressurePatch("GROUND_RIGHT", center.add(new Vec3d(hx, 0, 0)), new Vec3d(1, 0, 0), side, ModelSurfaceMap.SurfaceKind.BODY),
            new ModelSurfaceMap.PressurePatch("GROUND_BOTTOM", center.add(new Vec3d(0, -hy, 0)), new Vec3d(0, -1, 0), top, ModelSurfaceMap.SurfaceKind.BODY),
            new ModelSurfaceMap.PressurePatch("GROUND_TOP", center.add(new Vec3d(0, hy, 0)), new Vec3d(0, 1, 0), top, ModelSurfaceMap.SurfaceKind.BODY),
            new ModelSurfaceMap.PressurePatch("GROUND_REAR", center.add(new Vec3d(0, 0, -hz)), new Vec3d(0, 0, -1), end, ModelSurfaceMap.SurfaceKind.BODY),
            new ModelSurfaceMap.PressurePatch("GROUND_FRONT", center.add(new Vec3d(0, 0, hz)), new Vec3d(0, 0, 1), end, ModelSurfaceMap.SurfaceKind.BODY)
        );
        Vec3d inertiaPerKg = boxInertiaPerKgAboutOrigin(Vec3d.ZERO, width, height, length);
        return new FallbackBody(
            patches, width, height, length,
            2.0 * (width * height + width * length + height * length),
            inertiaPerKg
        );
    }

    /** True only after this tick's bounded solver has actually run. */
    public static boolean replacesNativeTires(EntityVehicleF_Physics vehicle) {
        return vehicle != null && !vehicle.lockedOnRoad
            && vehicle instanceof GroundVehicleWindStateAccess access
            && access.pmweatherIv$getGroundWindState().tireSolverTick == vehicle.ticksExisted;
    }

    /** Native gravity is 0.0245 internal blocks/tick²; native pose multiplies by speedFactor. */
    public static void correctNativeGroundGravity(EntityVehicleF_Physics vehicle) {
        if (!PMWeatherIVConfig.get().enabled() || vehicle.definition.motorized.isAircraft
            || vehicle.towedByConnection != null || vehicle.lockedOnRoad
            || vehicle.ballastVolumeVar.isActive || !hasInstalledTires(vehicle)) return;
        double speedFactor = Math.max(1e-6, Math.abs(vehicle.speedFactor));
        double gravityFactor = vehicle.gravityFactorVar.currentValue;
        if (vehicle.waterBallastFactorVar.isActive && vehicle.world.isBlockLiquid(vehicle.position))
            gravityFactor *= 1-vehicle.waterBallastFactorVar.currentValue;
        if (!Double.isFinite(gravityFactor)) return;
        vehicle.motion.y -= (9.80665 - 9.8*speedFactor)*gravityFactor
            * MC_TICK_SECONDS*MC_TICK_SECONDS/speedFactor;
    }

    /** Solve tires before native ground operations, on the server and in client prediction. */
    public static void prepareTires(EntityVehicleF_Physics vehicle) {
        if (!PMWeatherIVConfig.get().enabled()
            || vehicle.definition == null || vehicle.definition.motorized == null
            || vehicle.definition.motorized.isAircraft || vehicle.lockedOnRoad
            || vehicle.towedByConnection != null || vehicle.outOfHealth
            || !(vehicle instanceof GroundVehicleWindStateAccess access)
            || !hasInstalledTires(vehicle)) return;
        State state = access.pmweatherIv$getGroundWindState();
        if (vehicle.world.isClient()) {
            ensurePrepared(vehicle, PMWeatherIVConfig.get(), state);
            state.lastInertiaKgM2 = inertiaForMass(state.inertiaPerKgAboutCenter,
                vehicle.currentMass, state.bodyWidth, state.bodyHeight, state.bodyLength);
            applyTireYawReaction(vehicle, state, Vec3d.ZERO, vehicle.currentMass);
            Vec3d request = externalAngularVelocityBody(vehicle, state, state.lastInertiaKgM2)
                .scale(RAD_TO_DEG*MC_TICK_SECONDS);
            vehicle.rotation.angles.set(request.x(), request.y(), request.z());
            beginAngularFeedback(vehicle, state, state.lastInertiaKgM2, request);
            state.pendingAfterGroundRotationDegrees = request;
            return;
        }
        if (!pending(vehicle, state) || state.currentResult == null) return;
        Result before = state.currentResult;
        TireYawReaction tires = applyTireYawReaction(vehicle, state, before.forceWorldNewtons(), vehicle.currentMass);
        Vec3d omega = externalAngularVelocityBody(vehicle, state, before.inertiaKgM2());
        Vec3d request = omega.scale(RAD_TO_DEG*MC_TICK_SECONDS);
        // Replace just PMIV's staged request with the reacted request; preserve native inputs.
        vehicle.rotation.angles.set(0, 0, 0);
        beginAngularFeedback(vehicle, state, before.inertiaKgM2(), request);
        vehicle.rotation.angles.add(request.x(), request.y(), request.z());
        state.lastExternalAngularVelocityBodyRadps = omega;
        state.currentResult = new Result(before.forceWorldNewtons(), before.torqueBodyNewtonMeters(),
            before.meanWindMph(), before.patchCount(), before.modelBased(), before.inertiaKgM2(),
            before.externalAngularVelocityBeforeTorqueBodyRadps(), before.externalAngularVelocityAfterTorqueBodyRadps(),
            before.externalAngularMomentumAfterTorqueWorldNms(), tires, omega, state.angularMomentumWorld, request,
            before.bodyAerodynamics());
        state.pendingAfterGroundRotationDegrees = request;
        logTires(vehicle, state, state.currentResult);
    }

    /** Already evaluated solver values only: tracing never reruns any physics. */
    public static Result latestResult(EntityVehicleF_Physics vehicle, State state) {
        return pending(vehicle, state) ? state.currentResult : null;
    }

    /**
     * Shared full 3D contacts replace native braking, angular skid projection and upright
     * pose correction. IV supplies drivetrain forces and retains solid body collision.
     * Unilateral normal loads arise from actual support geometry and contact velocities.
     */
    private static boolean hasInstalledTires(EntityVehicleF_Physics vehicle) {
        for (APart part : vehicle.allParts) {
            if (part instanceof PartGroundDevice device && TireContactMaterial.activeSupport(device)
                && (device.definition.ground.isWheel || device.definition.ground.isTread)) return true;
        }
        return false;
    }

    private static TireYawReaction applyTireYawReaction(
        EntityVehicleF_Physics vehicle, State state, Vec3d forceWorldNewtons, double massKg
    ) {
        if (vehicle.groundDeviceCollective == null || !(massKg > 0)) return TireYawReaction.none();
        vehicle.groundDeviceCollective.updateCollisions(true); // animation/gameplay sensors only
        double speedFactor = Math.max(1e-6, Math.abs(vehicle.speedFactor));
        Vec3d inertia = state.lastInertiaKgM2;
        Vec3d omega = externalAngularVelocityBody(vehicle, state, inertia);
        if (!vehicle.world.isClient()) {
            Vec3d nativeRequest = point(vehicle.rotation.angles).subtract(state.pendingExternalRequestDegrees);
            omega = omega.add(nativeRequest.scale(Math.PI/180/MC_TICK_SECONDS));
        }
        Vec3d velocity = point(vehicle.motion).scale(speedFactor/MC_TICK_SECONDS)
            .add(toWorld(vehicle, omega).cross(toWorld(vehicle, state.centerOfMassLocal)));
        List<GroundWheelTerrain.Contact> exact = GroundWheelTerrain.contacts(vehicle,
            state.centerOfMassLocal, velocity, omega, MC_TICK_SECONDS);
        List<GroundContactImpulseSolver.Contact> contacts = new ArrayList<>();
        List<TireContactMaterial.Grip> grips = new ArrayList<>();
        List<TireNormalCompliance.Response> responses = new ArrayList<>();
        double nativeDriveImpulse = 0;
        for (PartEngine engine : vehicle.engines) {
            if (((PartEngineAccessor) engine).pmweatherIv$getJetPowerFactorVar().isActive) continue;
            nativeDriveImpulse += ((PartEngineAccessor) engine).pmweatherIv$getEngineForceValue()
                * IV_TICKS_SQUARED_TO_SECONDS_SQUARED*speedFactor*MC_TICK_SECONDS;
        }
        double forwardSpeed=velocity.dot(toWorld(vehicle,new Vec3d(0,0,1)));
        boolean coastingBrake=vehicle.throttleVar.currentValue<=1e-6 && nativeDriveImpulse*forwardSpeed<0;
        double engineBrakeLimit=coastingBrake ? PMWeatherIVConfig.get().roadMaximumCoastingBrakeCoefficient()
            : Double.POSITIVE_INFINITY;
        double steering=RoadTireControls.steeringDegrees(vehicle);
        int installedSupports=TireNormalCompliance.installedSupportCount(vehicle);
        for (GroundWheelTerrain.Contact contact : exact) {
            TireContactMaterial.Grip grip = TireContactMaterial.evaluate(contact.device(), contact.supportBlock());
            grips.add(grip);
            TireNormalCompliance.Response response=TireNormalCompliance.evaluate(contact.device(),massKg,
                installedSupports,contact.gapMeters(),contact.targetNormalMps(),0.01,MC_TICK_SECONDS,true);
            responses.add(response);
            contacts.add(new GroundContactImpulseSolver.Contact(
                contact.pointLocal().subtract(state.centerOfMassLocal),
                LandingGearSolver.landingGearWheelForwardWorld(vehicle.orientation,
                    contact.device().placementDefinition.turnsWithSteer ? steering : 0),
                grip.motive(), grip.lateral(), response.targetMps(), contact.device().drivenLastTick,
                TireContactMaterial.rollingResistance(
                    LandingGearSolver.landingGearRollingResistanceCoefficient(contact.device()), grip), true,
                response.softnessInverseKg(),engineBrakeLimit,Double.NaN,
                response.hardNormalVelocityTargetMps()));
        }
        double brake = LandingGearSolver.landingGearBrakeCommand(vehicle)
            * (Double.isFinite(vehicle.brakingFactorVar.currentValue)
                ? Math.max(0,vehicle.brakingFactorVar.currentValue) : 0);
        double previousYawMomentum = Math.abs(state.angularMomentumWorld.y());
        GroundContactImpulseSolver.Result solved = GroundContactImpulseSolver.solve(
            velocity, omega, inertia, vehicle.orientation, toWorld(vehicle, new Vec3d(0,0,1)),
            massKg, brake, nativeDriveImpulse, MC_TICK_SECONDS, contacts);
        Vec3d solvedOmega = solved.angularVelocityBody();
        Vec3d originVelocity = solved.velocityWorld().subtract(
            toWorld(vehicle, solvedOmega).cross(toWorld(vehicle, state.centerOfMassLocal)));
        vehicle.motion.set(originVelocity.x()*MC_TICK_SECONDS/speedFactor,
            originVelocity.y()*MC_TICK_SECONDS/speedFactor, originVelocity.z()*MC_TICK_SECONDS/speedFactor);
        state.angularMomentumWorld = toWorld(vehicle, new Vec3d(
            inertia.x()*solvedOmega.x(), inertia.y()*solvedOmega.y(), inertia.z()*solvedOmega.z()));
        List<TireContactSnapshot> snapshots = new ArrayList<>();
        int loadedContacts = 0;
        for (int i=0; i<exact.size(); ++i) {
            GroundWheelTerrain.Contact contact = exact.get(i);
            TireContactMaterial.Grip grip = grips.get(i);
            double load = solved.normalImpulses()[i]/MC_TICK_SECONDS;
            if (load > 1e-5) ++loadedContacts;
            snapshots.add(new TireContactSnapshot(contact.pointLocal(), grip.material(), grip.wet(),
                grip.motive(), grip.lateral(), load, contact.gapMeters(),
                solved.longitudinalImpulses()[i], solved.lateralImpulses()[i],
                responses.get(i).targetMps(),responses.get(i).softnessInverseKg(),
                responses.get(i).stiffnessNpm(),responses.get(i).dampingNsPm(),
                responses.get(i).nominalDeflectionMeters(),
                contact.device().placementDefinition.turnsWithSteer ? steering : 0,nativeDriveImpulse,
                coastingBrake,coastingBrake ? engineBrakeLimit : 0,
                contacts.get(i).rollingCoefficient()));
        }
        state.tireContacts = List.copyOf(snapshots);
        state.tireSolverTick = vehicle.ticksExisted;
        vehicle.slipping = solved.saturated();
        if (PMIVObserver.loggingEnabled() && vehicle.ticksExisted % 20 == 0 && !vehicle.world.isClient()) {
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log("GROUND_CONTACTS uuid=" + vehicle.uniqueUUID
                + " centerOfMassLocal=" + state.centerOfMassLocal + " contacts=" + state.tireContacts
                + " authority=COUPLED_NORMAL_BRAKE_DRIVE_LATERAL_FULL_3D" );
            }
        }
        return new TireYawReaction(solved.normalImpulseNs()/MC_TICK_SECONDS, solved.yawCapacityNm(),
            Math.max(0, previousYawMomentum - Math.abs(state.angularMomentumWorld.y())), loadedContacts, solved.saturated());
    }

    /** Finite COM pivot displacement supplements IV's origin-based Euler pose step. */
    public static void beforeMove(EntityVehicleF_Physics vehicle) {
        if (!replacesNativeTires(vehicle) || !(vehicle instanceof GroundVehicleWindStateAccess access)) return;
        State state = access.pmweatherIv$getGroundWindState();
        RotationMatrix increment = new RotationMatrix().setToAngles(vehicle.rotation.angles);
        RotationMatrix predicted = new RotationMatrix().set(vehicle.orientation).multiply(increment);
        Vec3d centerWorld = toWorld(vehicle,state.centerOfMassLocal);
        Vec3d nextCenterWorld = toWorld(predicted,state.centerOfMassLocal);
        Vec3d omegaWorld = toWorld(vehicle,externalAngularVelocityBody(vehicle,state,state.lastInertiaKgM2));
        double factor = Math.max(1e-6,Math.abs(vehicle.speedFactor));
        state.stagedMoveOffset = centerWorld.subtract(nextCenterWorld)
            .add(omegaWorld.cross(centerWorld).scale(MC_TICK_SECONDS)).scale(1/factor);
        state.stagedOriginMotion = point(vehicle.motion);
        state.stagedCenterWorld = centerWorld;
        state.stagedOmegaWorld = omegaWorld;
        state.stagedMoveTick = vehicle.ticksExisted;
        vehicle.motion.add(state.stagedMoveOffset.x(),state.stagedMoveOffset.y(),state.stagedMoveOffset.z());
    }

    /** Remove positional pivot correction from the persisted velocity, including early returns. */
    public static void finishMove(EntityVehicleF_Physics vehicle) {
        if (!(vehicle instanceof GroundVehicleWindStateAccess access)) return;
        State state = access.pmweatherIv$getGroundWindState();
        if (state.stagedMoveTick != vehicle.ticksExisted) return;
        Vec3d staged = state.stagedOriginMotion.add(state.stagedMoveOffset);
        Vec3d motion = point(vehicle.motion);
        // A native solid collision may zero or scale a component. Do not reintroduce
        // motion into a stopped axis by subtracting a correction that was rejected too.
        vehicle.motion.set(unpivot(motion.x(), staged.x(), state.stagedOriginMotion.x()),
            unpivot(motion.y(), staged.y(), state.stagedOriginMotion.y()),
            unpivot(motion.z(), staged.z(), state.stagedOriginMotion.z()));
        double factor = Math.max(1e-6,Math.abs(vehicle.speedFactor));
        Vec3d velocityCOM = point(vehicle.motion).scale(factor/MC_TICK_SECONDS)
            .add(state.stagedOmegaWorld.cross(state.stagedCenterWorld));
        afterMove(vehicle,state); // accepted solid-collision rotation, in the local request frame
        Vec3d nextOriginVelocity = velocityCOM.subtract(toWorld(vehicle,
            externalAngularVelocityBody(vehicle,state,state.lastInertiaKgM2))
            .cross(toWorld(vehicle,state.centerOfMassLocal)));
        vehicle.motion.set(nextOriginVelocity.x()*MC_TICK_SECONDS/factor,
            nextOriginVelocity.y()*MC_TICK_SECONDS/factor,nextOriginVelocity.z()*MC_TICK_SECONDS/factor);
        state.stagedMoveTick = Long.MIN_VALUE;
        state.stagedMoveOffset = Vec3d.ZERO;
    }
    private static double unpivot(double accepted, double staged, double original) {
        if (Math.abs(accepted-staged)<1e-10) return original;
        return Math.abs(staged)>1e-10 ? original*(accepted/staged) : accepted;
    }

    private static void packPatch(
        double[] target,
        int base,
        ModelSurfaceMap.PressurePatch patch,
        Vec3d normal,
        Vec3d relativeAirLocal
    ) {
        target[base] = patch.pointLocal().x();
        target[base + 1] = patch.pointLocal().y();
        target[base + 2] = patch.pointLocal().z();
        target[base + 3] = normal.x();
        target[base + 4] = normal.y();
        target[base + 5] = normal.z();
        target[base + 6] = Math.max(0.0, patch.area());
        target[base + 7] = patch.twoSided() ? 1.0 : 0.0;
        target[base + 8] = relativeAirLocal.x();
        target[base + 9] = relativeAirLocal.y();
        target[base + 10] = relativeAirLocal.z();
    }

    private static double roadAxialCoefficient(EntityVehicleF_Physics vehicle, PMWeatherIVConfig.Values config) {
        double live = vehicle.dragCoefficientVar.currentValue;
        return Double.isFinite(live) && live >= 0 ? live : config.groundFallbackAxialDragCoefficient();
    }

    /** Mirrors PMAero 1.0's documented axial scaling contract. */
    private static double axialPressureScale(double width, double height, double length) {
        return 1/Math.max(1, length/Math.sqrt(Math.max(1e-9, width*height)));
    }

    private static Vec3d header(double[] output, int offset) {
        return new Vec3d(output[offset], output[offset+1], output[offset+2]);
    }

    private static Vec3d headerDelta(double[] wind, double[] still, int offset) {
        return new Vec3d(
            wind[offset] - still[offset],
            wind[offset + 1] - still[offset + 1],
            wind[offset + 2] - still[offset + 2]
        );
    }

    private static double[] resize(double[] values, int size) {
        return values.length == size ? values : new double[size];
    }

    /** Called after IV's performGroundOperations() has modified the rotation request. */
    public static void afterGroundOperations(EntityVehicleF_Physics vehicle, State state) {
        if (!pending(vehicle, state)) return;
        Vec3d current = point(vehicle.rotation.angles);
        // Native steering is an input, not a contact rejecting PMWeather momentum.
        // Tire reactions have already changed momentum in the shared impulse solver.
        state.pendingAfterGroundRotationDegrees = current;
    }

    /**
     * Called after IV move/collision resolution. Any native correction that opposes the
     * PMWeather request is interpreted as a real contact reaction and removes the same
     * principal-axis fraction of external angular momentum. Same-direction steering,
     * terrain or collision corrections are deliberately ignored so IV cannot inject
     * artificial PMWeather momentum.
     */
    public static void afterMove(EntityVehicleF_Physics vehicle, State state) {
        if (!pending(vehicle, state)) return;
        if (state.pendingAfterGroundRotationDegrees == null) {
            state.pendingAfterGroundRotationDegrees = point(vehicle.rotation.angles);
        }
        // IV corrects rotation.angles in place for ground/block collisions. It leaves
        // that accepted local request intact, while rotationApplied.angles is repurposed
        // for WORLD Euler networking deltas. Never compare those different frames.
        Vec3d applied = point(vehicle.rotation.angles);
        Vec3d moveCorrection = applied.subtract(state.pendingAfterGroundRotationDegrees);
        Vec3d retained = retainAgainstOpposingCorrection(
            state.pendingRetainedRequestDegrees, moveCorrection
        );

        Vec3d scale = retentionScale(state.pendingExternalRequestDegrees, retained);
        Vec3d bodyL = state.pendingBodyAngularMomentum;
        Vec3d reactedWorld = state.pendingBodyAxisXWorld.scale(bodyL.x() * scale.x())
            .add(state.pendingBodyAxisYWorld.scale(bodyL.y() * scale.y()))
            .add(state.pendingBodyAxisZWorld.scale(bodyL.z() * scale.z()));
        state.angularMomentumWorld = reactedWorld.isFinite() ? reactedWorld : Vec3d.ZERO;
        state.lastExternalAngularVelocityBodyRadps = externalAngularVelocityBody(
            vehicle, state, state.lastInertiaKgM2
        );
        clearPending(state);
    }

    public static Telemetry telemetry(EntityVehicleF_Physics vehicle, State state) {
        if (state == null) {
            return new Telemetry(Vec3d.ZERO, Vec3d.ZERO, Vec3d.ZERO);
        }
        Vec3d omega = vehicle == null
            ? state.lastExternalAngularVelocityBodyRadps
            : externalAngularVelocityBody(vehicle, state, state.lastInertiaKgM2);
        return new Telemetry(state.lastInertiaKgM2, state.angularMomentumWorld, omega);
    }

    /** Disable/re-road-lock transition: PMWeather must not resume stale momentum later. */
    public static void resetDynamics(State state) {
        if (state == null) return;
        state.angularMomentumWorld = Vec3d.ZERO;
        state.lastExternalAngularVelocityBodyRadps = Vec3d.ZERO;
        state.lastInertiaKgM2 = Vec3d.ZERO;
        state.tireSolverTick = Long.MIN_VALUE;
        state.tireContacts = List.of();
        clearPending(state);
    }

    private static void beginAngularFeedback(
        EntityVehicleF_Physics vehicle,
        State state,
        Vec3d inertia,
        Vec3d externalRequestDegrees
    ) {
        state.pendingAngularTick = vehicle.ticksExisted;
        state.pendingExternalRequestDegrees = externalRequestDegrees;
        state.pendingRetainedRequestDegrees = externalRequestDegrees;
        state.pendingAfterGroundRotationDegrees = null;
        state.pendingBodyAngularMomentum = toLocal(vehicle, state.angularMomentumWorld);
        state.pendingBodyAxisXWorld = toWorld(vehicle, new Vec3d(1, 0, 0));
        state.pendingBodyAxisYWorld = toWorld(vehicle, new Vec3d(0, 1, 0));
        state.pendingBodyAxisZWorld = toWorld(vehicle, new Vec3d(0, 0, 1));
        state.lastInertiaKgM2 = inertia;
    }

    private static boolean pending(EntityVehicleF_Physics vehicle, State state) {
        return vehicle != null && state != null && state.pendingAngularTick == vehicle.ticksExisted;
    }

    private static void clearPending(State state) {
        state.pendingAngularTick = Long.MIN_VALUE;
        state.pendingExternalRequestDegrees = Vec3d.ZERO;
        state.pendingRetainedRequestDegrees = Vec3d.ZERO;
        state.pendingAfterGroundRotationDegrees = Vec3d.ZERO;
        state.pendingBodyAngularMomentum = Vec3d.ZERO;
        state.currentResult = null;
    }

    private static Vec3d retainAgainstOpposingCorrection(Vec3d external, Vec3d correction) {
        return new Vec3d(
            retainAxis(external.x(), correction.x()),
            retainAxis(external.y(), correction.y()),
            retainAxis(external.z(), correction.z())
        );
    }

    private static double retainAxis(double external, double correction) {
        if (!Double.isFinite(external) || Math.abs(external) <= 1.0E-12) return 0.0;
        if (!Double.isFinite(correction) || external * correction >= 0.0) return external;
        double remaining = Math.max(0.0, Math.abs(external) - Math.abs(correction));
        return Math.copySign(remaining, external);
    }

    private static Vec3d retentionScale(Vec3d original, Vec3d retained) {
        return new Vec3d(
            retentionAxis(original.x(), retained.x()),
            retentionAxis(original.y(), retained.y()),
            retentionAxis(original.z(), retained.z())
        );
    }

    private static double retentionAxis(double original, double retained) {
        if (!Double.isFinite(original) || Math.abs(original) <= 1.0E-12) return 1.0;
        if (!Double.isFinite(retained) || retained * original <= 0.0) return 0.0;
        return Vec3d.clamp(Math.abs(retained / original), 0.0, 1.0);
    }

    private static Vec3d externalAngularVelocityBody(
        EntityVehicleF_Physics vehicle, State state, Vec3d inertia
    ) {
        if (vehicle == null || state == null || inertia == null || !inertia.isFinite()) return Vec3d.ZERO;
        Vec3d bodyL = toLocal(vehicle, state.angularMomentumWorld);
        return new Vec3d(
            bodyL.x() / Math.max(1.0E-6, inertia.x()),
            bodyL.y() / Math.max(1.0E-6, inertia.y()),
            bodyL.z() / Math.max(1.0E-6, inertia.z())
        );
    }

    private static Vec3d inertiaForMass(
        Vec3d inertiaPerKg, double mass, double width, double height, double length
    ) {
        Vec3d shape = sanitizeInertiaPerKg(inertiaPerKg);
        if (shape.lengthSquared() <= 1.0E-12) {
            shape = boxInertiaPerKgAboutOrigin(Vec3d.ZERO, width, height, length);
        }
        return new Vec3d(
            Math.max(1.0E-6, shape.x() * mass),
            Math.max(1.0E-6, shape.y() * mass),
            Math.max(1.0E-6, shape.z() * mass)
        );
    }

    private static Vec3d sanitizeInertiaPerKg(Vec3d value) {
        if (value == null || !value.isFinite()) return Vec3d.ZERO;
        return new Vec3d(
            Math.max(0.0, value.x()),
            Math.max(0.0, value.y()),
            Math.max(0.0, value.z())
        );
    }

    private static Vec3d boxInertiaPerKgAboutOrigin(
        Vec3d center, double width, double height, double length
    ) {
        double w = Math.max(0.05, Math.abs(width));
        double h = Math.max(0.05, Math.abs(height));
        double l = Math.max(0.05, Math.abs(length));
        Vec3d c = center == null || !center.isFinite() ? Vec3d.ZERO : center;
        return new Vec3d(
            (h * h + l * l) / 12.0 + c.y() * c.y() + c.z() * c.z(),
            (w * w + l * l) / 12.0 + c.x() * c.x() + c.z() * c.z(),
            (w * w + h * h) / 12.0 + c.x() * c.x() + c.y() * c.y()
        );
    }

    private record FallbackBody(
        List<ModelSurfaceMap.PressurePatch> patches,
        double width,
        double height,
        double length,
        double wettedArea,
        Vec3d inertiaPerKgAboutCenter
    ) {
    }
}
