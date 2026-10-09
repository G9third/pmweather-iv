package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import com.g9third.pmweatheriv.compat.PMAeroBridge;
import java.util.List;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import static com.g9third.pmweatheriv.physics.FlightMath.PointFlow;
import static com.g9third.pmweatheriv.physics.FlightMath.pointFlow;
import static com.g9third.pmweatheriv.physics.FlightMath.toLocal;
import static com.g9third.pmweatheriv.physics.FlightMath.toWorld;

import static com.g9third.pmweatheriv.physics.AircraftWind.WindField;
import static com.g9third.pmweatheriv.physics.AirframeLoads.Accumulator;
import static com.g9third.pmweatheriv.physics.AirframeGeometry.Geometry;

/** Evaluates body pressure patches at the current physical pose. */
public final class BodySurfaceAdapter {
    private BodySurfaceAdapter() {}

    static void addBodyPressure(
        EntityVehicleF_Physics vehicle,
        Geometry geometry,
        ModelSurfaceMap.PreparedModel model,
        double density,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        Accumulator accumulator,
        PMWeatherIVConfig.Values config
    ) {
        if (windField.collecting()) {
            for (ModelSurfaceMap.PressurePatch patch : StructuralDamageMask.activePressurePatches(
                vehicle, accumulator.flightState.bodyPressurePatches()
            )) {
                windField.register(patch.name(), patch.windSamplePointLocal());
            }
            return;
        }
        addPMAeroBodyPressure(vehicle, geometry, model, density, linearVelocityWorld, omegaWorld,
            windField, accumulator, config);
    }

    /**
     * Required PMAero 1.0 owns the generic body pressure/skin-friction law. PMIV still
     * owns IV model interpretation, same-substep rigid-body point flow and semantic pressure-patch
     * selection. This deliberately exports ModelSurfaceMap pressure patches rather than raw Sable
     * collision cuboids: merged collision hulls no longer reliably distinguish fuselage from
     * wing/tail geometry and would double-count lifting surfaces.
     */
    static void addPMAeroBodyPressure(
        EntityVehicleF_Physics vehicle,
        Geometry geometry,
        ModelSurfaceMap.PreparedModel model,
        double density,
        Vec3d linearVelocityWorld,
        Vec3d omegaWorld,
        WindField windField,
        Accumulator accumulator,
        PMWeatherIVConfig.Values config
    ) {
        final List<ModelSurfaceMap.PressurePatch> patches = StructuralDamageMask.activePressurePatches(
            vehicle, accumulator.flightState.bodyPressurePatches()
        );
        final int inputStride = PMAeroBridge.bodyInputStride();
        final int inputLength = patches.size() * inputStride;
        final int outputLength = PMAeroBridge.bodyOutputHeaderStride()
            + patches.size() * PMAeroBridge.bodyOutputPatchStride();
        if (accumulator.flightState.pmaeroBodyInputBuffer.length != inputLength) {
            accumulator.flightState.pmaeroBodyInputBuffer = new double[inputLength];
        }
        if (accumulator.flightState.pmaeroBodyOutputBuffer.length != outputLength) {
            accumulator.flightState.pmaeroBodyOutputBuffer = new double[outputLength];
        }
        final double[] packed = accumulator.flightState.pmaeroBodyInputBuffer;
        final double[] result = accumulator.flightState.pmaeroBodyOutputBuffer;
        if (accumulator.flightState.bodyFlowBuffer.length != patches.size()) {
            accumulator.flightState.bodyFlowBuffer = new PointFlow[patches.size()];
        }
        final PointFlow[] flows = accumulator.flightState.bodyFlowBuffer;

        for (int i = 0; i < patches.size(); i++) {
            final ModelSurfaceMap.PressurePatch patch = patches.get(i);
            final PointFlow flow = pointFlow(
                vehicle, patch.pointLocal(), patch.windSamplePointLocal(), accumulator.flightState.plan.centerOfMassLocal(),
                linearVelocityWorld, omegaWorld, windField, patch.name(), 1.0
            );
            flows[i] = flow;
            final Vec3d relativeLocal = toLocal(vehicle, flow.relativeAirWorld());
            final Vec3d normal = patch.normalLocal().normalized();
            final int base = i * inputStride;
            packed[base] = patch.pointLocal().x();
            packed[base + 1] = patch.pointLocal().y();
            packed[base + 2] = patch.pointLocal().z();
            packed[base + 3] = normal.x();
            packed[base + 4] = normal.y();
            packed[base + 5] = normal.z();
            packed[base + 6] = patch.area();
            packed[base + 7] = patch.twoSided() ? 1.0 : 0.0;
            packed[base + 8] = relativeLocal.x();
            packed[base + 9] = relativeLocal.y();
            packed[base + 10] = relativeLocal.z();
        }

        double originalPressureArea = 0.0;
        for (ModelSurfaceMap.PressurePatch patch : accumulator.flightState.bodyPressurePatches()) {
            originalPressureArea += Math.max(0.0, patch.area());
        }
        double activePressureArea = 0.0;
        for (ModelSurfaceMap.PressurePatch patch : patches) {
            activePressureArea += Math.max(0.0, patch.area());
        }
        double survivingWettedFraction = originalPressureArea > 1.0E-9
            ? Vec3d.clamp(activePressureArea / originalPressureArea, 0.0, 1.0) : 0.0;

        PMAeroBridge.evaluateExternalBodyInto(
            packed, result, density,
            config.bodyAxialDragCoefficient(), config.bodyCrossflowDragCoefficient(),
            geometry.bodyWidth(), geometry.bodyHeight(), geometry.bodyLength(),
            Math.max(0.0, model.pressureWettedArea()) * survivingWettedFraction,
            accumulator.flightState.plan.centerOfMassLocal()
        );
        final int header = PMAeroBridge.bodyOutputHeaderStride();
        final int outputStride = PMAeroBridge.bodyOutputPatchStride();

        // Apply the returned per-patch force at the exact same semantic point PMIV used before.
        // Accumulator reconstructs r x F for Sable and retains detailed trace rows. The aggregate
        // force/torque in result[0..5] is intentionally redundant and makes the PMAero API usable
        // by arbitrary Sable owners that prefer one net ForceTotal.
        for (int i = 0; i < patches.size(); i++) {
            final ModelSurfaceMap.PressurePatch patch = patches.get(i);
            final int out = header + i * outputStride;
            final Vec3d forceLocal = new Vec3d(result[out], result[out + 1], result[out + 2]);
            final double pressureCoefficient = result[out + 3];
            final double skinCoefficient = result[out + 4];
            accumulator.addPressureSurface(
                "PMAERO_COMPONENT_PRESSURE_" + patch.name(),
                patch.pointLocal(),
                flows[i],
                toWorld(vehicle, forceLocal),
                patch.area(),
                pressureCoefficient + skinCoefficient,
                vehicle
            );
        }
    }

    static List<BodyFace> bodyFaces(Geometry geometry) {
        double halfX = geometry.bodyWidth() * 0.5;
        double halfY = geometry.bodyHeight() * 0.5;
        double halfZ = geometry.bodyLength() * 0.5;
        double sideArea = Math.max(0.05, geometry.bodyHeight() * geometry.bodyLength() * 0.62);
        double topArea = Math.max(0.05, geometry.bodyWidth() * geometry.bodyLength() * 0.62);
        double frontalArea = Math.max(0.05, geometry.bodyWidth() * geometry.bodyHeight() * 0.72);
        return List.of(
            new BodyFace("BODY_LEFT", new Vec3d(-halfX, 0.0, 0.0), new Vec3d(-1.0, 0.0, 0.0), sideArea),
            new BodyFace("BODY_RIGHT", new Vec3d(halfX, 0.0, 0.0), new Vec3d(1.0, 0.0, 0.0), sideArea),
            new BodyFace("BODY_BOTTOM", new Vec3d(0.0, -halfY, 0.0), new Vec3d(0.0, -1.0, 0.0), topArea),
            new BodyFace("BODY_TOP", new Vec3d(0.0, halfY, 0.0), new Vec3d(0.0, 1.0, 0.0), topArea),
            new BodyFace("BODY_REAR", new Vec3d(0.0, 0.0, -halfZ), new Vec3d(0.0, 0.0, -1.0), frontalArea),
            new BodyFace("BODY_FRONT", new Vec3d(0.0, 0.0, halfZ), new Vec3d(0.0, 0.0, 1.0), frontalArea)
        );
    }

    public record BodyFace(
        String name,
        Vec3d pointLocal,
        Vec3d normalLocal,
        double area
    ) {
    }
}
