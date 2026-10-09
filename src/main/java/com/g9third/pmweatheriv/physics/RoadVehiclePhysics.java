package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import com.g9third.pmweatheriv.compat.PMAeroBridge;
import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import com.g9third.pmweatheriv.model.ModelPhysicalVisibility;
import com.g9third.pmweatheriv.mixin.EntityVehiclePhysicsAccessor;
import com.g9third.pmweatheriv.mixin.PartEngineAccessor;
import com.g9third.pmweatheriv.sable.SableModelCollisionHull;
import java.util.ArrayList;
import java.util.List;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartEngine;
import minecrafttransportsimulator.entities.instances.PartPropeller;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.Level;
import static com.g9third.pmweatheriv.physics.FlightMath.*;
import static com.g9third.pmweatheriv.physics.AircraftWind.*;
import static com.g9third.pmweatheriv.physics.AirframeLoads.*;

/** Road loads for the same persistent body, terrain and impact solver used by aircraft. */
public final class RoadVehiclePhysics {
    private RoadVehiclePhysics() {}

    public record DriveSnapshot(double forceNewtons, double steeringDegrees,
                                double coastingCoefficientLimit, long gameTime,
                                boolean skidSteer, double yawRateTargetRadiansPerSecond) {
        public DriveSnapshot(double forceNewtons, double steeringDegrees,
                             double coastingCoefficientLimit, long gameTime) {
            this(forceNewtons, steeringDegrees, coastingCoefficientLimit, gameTime, false, 0.0);
        }
        public static final DriveSnapshot ZERO = new DriveSnapshot(
            0, 0, Double.POSITIVE_INFINITY, Long.MIN_VALUE, false, 0.0);
        public double impulse(double seconds) { return forceNewtons * seconds; }
    }

    public record BodyPlan(SableBodyPressureGeometry.PreparedBody body,
                           List<RoadRoofPressure.Panel> roofPanels) {
        public BodyPlan { roofPanels = List.copyOf(roofPanels); }
    }

    public static SolveResult calculateSableTickLoads(EntityVehicleF_Physics vehicle, Level level,
            PMWeatherIVConfig.Values config, AircraftState state) {
        prepare(vehicle, config, state);
        AircraftKinematics physical = state.kinematics;
        RotationMatrix orientation = physical == null ? vehicle.orientation : physical.orientation();
        Vec3d origin = physical == null ? point(vehicle.position) : physical.modelOriginWorld();
        Vec3d omega = physical == null ? state.angularVelocityBody : physical.angularVelocityBody();
        Vec3d originVelocity = state.hasPhysicalVelocity ? state.originVelocityWorld
            : point(vehicle.motion).scale(Math.abs(vehicle.speedFactor) / MC_TICK_SECONDS);
        Vec3d velocity = physical == null ? originVelocity.add(toWorld(orientation, omega)
            .cross(toWorld(orientation, state.plan.centerOfMassLocal()))) : physical.centerVelocityWorld();
        double mass = finiteClamp(vehicle.currentMass, 1, 1e8, Math.max(1, vehicle.definition.motorized.emptyMass));
        Vec3d inertia = inertiaForMass(state.roadBody.body(), mass, state.plan.centerOfMassLocal());
        double density = ivCompatibleAirDensity(level, vehicle, origin.y());
        vehicle.airDensity = density;
        SubstepEvaluationContext previous = SUBSTEP_EVALUATION.get();
        SUBSTEP_EVALUATION.set(new SubstepEvaluationContext(origin, orientation, velocity, omega, MC_TICK_SECONDS, false));
        try {
            if (state.windSnapshot == null || state.windGameTime != level.getGameTime()) {
                WindField field = new WindField(vehicle);
                for (var patch : state.plan.pressurePatches()) field.register(patch.name(), patch.windSamplePointLocal());
                for (APart part : vehicle.allParts) {
                    if (part instanceof PartPropeller propeller) field.register("PROP_" + part.uniqueUUID, point(part.localOffset));
                    else if (part instanceof PartEngine engine && ((PartEngineAccessor) engine).pmweatherIv$getJetPowerFactorVar().isActive)
                        field.register("JET_" + part.uniqueUUID, point(part.localOffset));
                }
                field.resolve((ServerLevel) level, state);
            }
            // Actuators mutate IV wheel/RPM state once per owner tick, never per Sable substep.
            if (state.roadActuatorGameTime != level.getGameTime()) {
                Point3D savedMotion = vehicle.motion.copy();
                double savedVelocity = vehicle.velocity;
                vehicle.motion.set(originVelocity.x(), originVelocity.y(), originVelocity.z())
                    .scale(MC_TICK_SECONDS / Math.max(1e-6, Math.abs(vehicle.speedFactor)));
                vehicle.velocity = vehicle.motion.length();
                LandingGearSolver.maintainIvGroundAnimationState(vehicle);
                Accumulator propulsion = new Accumulator(
                    state, PMIVObserver.needsDetailedSamples(vehicle.uniqueUUID), true);
                WindField field = new WindField(vehicle, state.windSnapshot);
                double drive = 0, nativeThrust = 0;
                try {
                    for (APart part : vehicle.allParts) {
                        if (part instanceof PartEngine engine) {
                            if (((PartEngineAccessor) engine).pmweatherIv$getJetPowerFactorVar().isActive) {
                                nativeThrust += PropulsionModel.addJetEngine(vehicle, engine, velocity,
                                    toWorld(orientation, omega), field, propulsion).mtsForceValue();
                            } else {
                                double demand = engine.addToForceOutput(new Point3D(), new Point3D());
                                drive += demand * PropulsionModel.ivForceNewtonsPerUnit(vehicle);
                                nativeThrust += demand;
                            }
                        } else if (part instanceof PartPropeller propeller) {
                            nativeThrust += PropulsionModel.addPropeller(vehicle, propeller, velocity,
                                toWorld(orientation, omega), field, propulsion).mtsForceValue();
                        }
                    }
                    double forwardSpeed = velocity.dot(toWorld(orientation, new Vec3d(0, 0, 1)));
                    double limit = vehicle.throttleVar.currentValue <= 1e-6 && drive * forwardSpeed < 0
                        ? config.roadMaximumCoastingBrakeCoefficient() : Double.POSITIVE_INFINITY;
                    RoadTireControls.Steering steering = RoadTireControls.capture(vehicle);
                    state.roadDrive = new DriveSnapshot(drive, steering.wheelAngleDegrees(), limit,
                        level.getGameTime(), steering.skidSteer(), steering.yawRateTargetRadiansPerSecond());
                    state.roadPropulsion = List.copyOf(propulsion.propulsionLoads);
                    state.roadPropulsionSamples = List.copyOf(propulsion.propulsionSamples);
                    state.roadActuatorGameTime = level.getGameTime();
                    ((EntityVehiclePhysicsAccessor) vehicle).pmweatherIv$setThrustForceValue(nativeThrust);
                } finally {
                    vehicle.motion.set(savedMotion);
                    vehicle.velocity = savedVelocity;
                }
            }
            SubstepAerodynamicLoads aero = evaluate(vehicle, config, state, velocity, omega, state.windSnapshot, density);
            Vec3d force = aero.forceWorld().add(new Vec3d(0, -mass * GRAVITY, 0));
            Vec3d torque = aero.torqueBody();
            for (var propulsion : state.roadPropulsion) {
                force = force.add(propulsion.forceWorldNewtons());
                torque = torque.add(propulsion.torqueBodyNewtonMeters());
            }
            WindSample center = aero.centerWind();
            Vec3d relative = toLocal(orientation, velocity.subtract(center.windMetersPerSecond()));
            vehicle.indicatedSpeed = Math.abs(relative.z());
            vehicle.axialVelocity = Math.abs(originVelocity.dot(toWorld(orientation, new Vec3d(0, 0, 1))))
                * MC_TICK_SECONDS / Math.max(1e-6, Math.abs(vehicle.speedFactor));
            return new SolveResult(state.plan.geometry(), state.plan.model(), mass, inertia,
                state.plan.centerOfMassLocal(), density, originVelocity, relative.length(), relative.z(),
                0, relative.length() > 1e-6 ? Math.toDegrees(Math.asin(Vec3d.clamp(relative.x()/relative.length(), -1, 1))) : 0,
                pointControls(vehicle), false, 0, List.of(), center, force, torque, aero.componentPressureTorqueBody(),
                Vec3d.ZERO, omega, 0, 0, aero.surfaceSamples(), state.roadPropulsionSamples,
                state.roadPropulsion, state.windSnapshot);
        } finally { AircraftPhysics.restoreSubstepContext(previous); }
    }

    public static SubstepAerodynamicLoads calculateSubstepAerodynamics(EntityVehicleF_Physics vehicle,
            Level level, PMWeatherIVConfig.Values config, AircraftState state, Vec3d origin,
            RotationMatrix orientation, Vec3d velocity, Vec3d omega, WindFieldSnapshot wind, double seconds) {
        if (state == null || state.roadBody == null || wind == null) return SubstepAerodynamicLoads.ZERO;
        SubstepEvaluationContext previous = SUBSTEP_EVALUATION.get();
        SUBSTEP_EVALUATION.set(new SubstepEvaluationContext(origin, orientation, velocity, omega, seconds, true));
        try { return evaluate(vehicle, config, state, velocity, omega, wind,
            ivCompatibleAirDensity(level, vehicle, origin.y())); }
        finally { AircraftPhysics.restoreSubstepContext(previous); }
    }

    private static SubstepAerodynamicLoads evaluate(EntityVehicleF_Physics vehicle, PMWeatherIVConfig.Values config,
            AircraftState state, Vec3d velocity, Vec3d omega, WindFieldSnapshot wind, double density) {
        var body = state.roadBody.body();
        var patches = state.plan.pressurePatches();
        int inputStride = PMAeroBridge.bodyInputStride(), outputStride = PMAeroBridge.bodyOutputPatchStride();
        int inputLength = patches.size()*inputStride;
        int outputLength = PMAeroBridge.bodyOutputHeaderStride()+patches.size()*outputStride;
        if (state.pmaeroBodyInputBuffer.length != inputLength) state.pmaeroBodyInputBuffer = new double[inputLength];
        if (state.pmaeroBodyOutputBuffer.length != outputLength) state.pmaeroBodyOutputBuffer = new double[outputLength];
        RotationMatrix orientation = activeOrientation(vehicle);
        Vec3d omegaWorld = toWorld(orientation, omega);
        WindField field = new WindField(vehicle, wind);
        boolean captureSurfaceSamples = PMIVObserver.needsDetailedSamples(vehicle.uniqueUUID);
        List<SurfaceSample> surfaces = captureSurfaceSamples ? new ArrayList<>() : List.of();
        for (int i=0; i<patches.size(); i++) {
            var p = patches.get(i);
            var sample = field.sample(p.windSamplePointLocal(), p.name());
            Vec3d flow = toLocal(orientation, velocity.add(omegaWorld.cross(toWorld(orientation,
                p.pointLocal().subtract(state.plan.centerOfMassLocal())))).subtract(sample.windMetersPerSecond()));
            int b = i*inputStride;
            state.pmaeroBodyInputBuffer[b]=p.pointLocal().x(); state.pmaeroBodyInputBuffer[b+1]=p.pointLocal().y();
            state.pmaeroBodyInputBuffer[b+2]=p.pointLocal().z(); state.pmaeroBodyInputBuffer[b+3]=p.normalLocal().x();
            state.pmaeroBodyInputBuffer[b+4]=p.normalLocal().y(); state.pmaeroBodyInputBuffer[b+5]=p.normalLocal().z();
            state.pmaeroBodyInputBuffer[b+6]=p.area(); state.pmaeroBodyInputBuffer[b+7]=p.twoSided()?1:0;
            state.pmaeroBodyInputBuffer[b+8]=flow.x(); state.pmaeroBodyInputBuffer[b+9]=flow.y(); state.pmaeroBodyInputBuffer[b+10]=flow.z();
        }
        double cd = vehicle.dragCoefficientVar.currentValue;
        if (!Double.isFinite(cd) || cd < 0) cd=config.groundFallbackAxialDragCoefficient();
        double attenuation = 1/Math.max(1,body.length()/Math.sqrt(Math.max(1e-9,body.width()*body.height())));
        PMAeroBridge.evaluateExternalBodyInto(state.pmaeroBodyInputBuffer, state.pmaeroBodyOutputBuffer,
            density, cd/attenuation, config.groundCrossflowDragCoefficient(), body.width(), body.height(),
            body.length(), body.wettedArea(), state.plan.centerOfMassLocal());
        double[] out = state.pmaeroBodyOutputBuffer;
        Vec3d force = new Vec3d(out[0],out[1],out[2]), torque = new Vec3d(out[3],out[4],out[5]);
        for (int i=0; i<patches.size(); i++) {
            var p=patches.get(i); var panel=state.roadBody.roofPanels().get(i);
            int b=i*inputStride;
            Vec3d flow=new Vec3d(state.pmaeroBodyInputBuffer[b+8],state.pmaeroBodyInputBuffer[b+9],state.pmaeroBodyInputBuffer[b+10]);
            Vec3d roofFlow=flow.add(omega.cross(panel.pointLocal().subtract(p.pointLocal())));
            Vec3d roofForce=RoadRoofPressure.force(panel,roofFlow,density,config.roadRoofSuctionPressureCoefficient());
            force=force.add(roofForce);
            Vec3d roofTorque=panel.pointLocal().subtract(state.plan.centerOfMassLocal()).cross(roofForce);
            torque=torque.add(roofTorque);
            // Record the already evaluated pressure forces at their physical application points.
            if (captureSurfaceSamples) {
                int o=PMAeroBridge.bodyOutputHeaderStride()+i*outputStride;
                Vec3d partForce=new Vec3d(out[o],out[o+1],out[o+2]);
                Vec3d partTorque=p.pointLocal().subtract(state.plan.centerOfMassLocal()).cross(partForce);
                var sample=field.sample(p.windSamplePointLocal(),p.name());
                surfaces.add(new SurfaceSample(p.name(),p.windSamplePointLocal(),p.pointLocal(),sample.windMetersPerSecond(),
                    toWorld(orientation,flow),toWorld(orientation,partForce),partTorque,p.area(),0,cd,0,0,0,0,0,0));
                if (panel.areaM2()>0) surfaces.add(new SurfaceSample("ROAD_ROOF_"+i,p.windSamplePointLocal(),panel.pointLocal(),
                    sample.windMetersPerSecond(),toWorld(orientation,roofFlow),toWorld(orientation,roofForce),roofTorque,
                    panel.areaM2(),0,config.roadRoofSuctionPressureCoefficient(),0,0,0,0,0,0));
            }
        }
        if (!force.isFinite() || !torque.isFinite()) throw new IllegalStateException("Non-finite Sable road load");
        return new SubstepAerodynamicLoads(toWorld(orientation,force),torque,torque,0,0,
            field.initializeCenter(),captureSurfaceSamples ? List.copyOf(surfaces) : List.of());
    }

    private static void prepare(EntityVehicleF_Physics vehicle, PMWeatherIVConfig.Values config, AircraftState state) {
        String signature=ModelPhysicalVisibility.signature(vehicle)+"|"+vehicle.scale+"|"+vehicle.definition.getModelLocation(vehicle.subDefinition)
            +"|"+config.maximumGroundPressurePatches();
        if (state.roadBody!=null && signature.equals(state.preparedGeometrySignature)
            && (state.roadBody.body().modelBased() || vehicle.ticksExisted<state.nextModelRetryTick)) return;
        var hull=SableModelCollisionHull.prepareAerodynamicBody(vehicle);
        var body=SableBodyPressureGeometry.prepare(hull,config.maximumGroundPressurePatches());
        ModelSurfaceMap.PreparedModel model=ModelSurfaceMap.prepare(vehicle,config.maximumGroundPressurePatches());
        if (!body.modelBased() || body.patches().isEmpty()) body=fallback(model);
        List<RoadRoofPressure.Panel> roof=RoadRoofPressure.prepare(body.modelBased()?body.patches():List.of(),
            hull==null?List.of():hull.sampleSurfaceTriangles());
        if (roof.size()!=body.patches().size()) roof=body.patches().stream().map(p->new RoadRoofPressure.Panel(p.pointLocal(),Vec3d.ZERO,0)).toList();
        var geometry=new AirframeGeometry.Geometry(body.width(),body.height(),body.length(),0,0,0,0,0,0,0,0,0,0,0,0);
        var bounds=model.fullBounds();
        model=new ModelSurfaceMap.PreparedModel(body.modelLocation(),body.modelBased(),body.reason(),bounds,model.bodyBounds(),
            body.patches(),List.of(),body.wettedArea(),model.sourceTriangles(),model.retainedTriangles(),body.patches().size());
        state.roadBody=new BodyPlan(body,roof);
        Vec3d center=state.hasPhysicalVelocity && state.plan!=null ? state.plan.centerOfMassLocal() : body.centerOfMassLocal();
        state.plan=new AirframePlan(model,geometry,null,null,inertiaForMass(body,1,center),center,false,List.of(),body.patches());
        state.preparedGeometrySignature=signature;
        state.nextModelRetryTick=body.modelBased()?Long.MAX_VALUE:vehicle.ticksExisted+40;
        state.windSnapshot=null; state.windGameTime=Long.MIN_VALUE;
    }

    private static Vec3d inertiaForMass(SableBodyPressureGeometry.PreparedBody body,double mass,Vec3d center) {
        Vec3d d=body.centerOfMassLocal().subtract(center);
        Vec3d i=body.inertiaPerKgAboutCenter().add(new Vec3d(d.y()*d.y()+d.z()*d.z(),
            d.x()*d.x()+d.z()*d.z(),d.x()*d.x()+d.y()*d.y()));
        return new Vec3d(Math.max(1e-3,i.x()*mass),Math.max(1e-3,i.y()*mass),Math.max(1e-3,i.z()*mass));
    }

    private static SableBodyPressureGeometry.PreparedBody fallback(ModelSurfaceMap.PreparedModel model) {
        var b=model.bodyBounds();
        double w=b.spanX(),h=b.spanY(),l=b.spanZ(); Vec3d center=b.minimum().add(b.maximum()).scale(0.5);
        List<ModelSurfaceMap.PressurePatch> p=new ArrayList<>();
        for (int axis=0;axis<3;axis++) for (int sign:new int[]{-1,1}) {
            Vec3d n=axis==0?new Vec3d(sign,0,0):axis==1?new Vec3d(0,sign,0):new Vec3d(0,0,sign);
            double half=(axis==0?w:axis==1?h:l)*0.5;
            p.add(new ModelSurfaceMap.PressurePatch("ROAD_FALLBACK_"+axis+'_'+sign,center.add(n.scale(half)),n,
                axis==0?h*l:axis==1?w*l:w*h,ModelSurfaceMap.SurfaceKind.BODY));
        }
        Vec3d i=new Vec3d((h*h+l*l)/12,(w*w+l*l)/12,(w*w+h*h)/12);
        return new SableBodyPressureGeometry.PreparedBody(model.modelLocation(),false,"RETRIABLE_BODY_BOUNDS",p,w,h,l,
            2*(w*h+w*l+h*l),i,center,i,0,0,0,0);
    }
}
