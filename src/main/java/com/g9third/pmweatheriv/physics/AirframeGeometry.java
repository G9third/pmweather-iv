package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import java.util.EnumMap;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import static com.g9third.pmweatheriv.physics.FlightMath.EPSILON;
import static com.g9third.pmweatheriv.physics.FlightMath.MAX_AUTHORED_TO_MODEL_CONTROL_AREA_RATIO;
import static com.g9third.pmweatheriv.physics.FlightMath.MIN_AUTHORED_TO_MODEL_CONTROL_AREA_RATIO;
import static com.g9third.pmweatheriv.physics.FlightMath.finiteClamp;
import static com.g9third.pmweatheriv.physics.FlightMath.finiteNonNegative;

/** Derives generic aircraft areas and dimensions from authored IV geometry. */
public final class AirframeGeometry {
    private AirframeGeometry() {}

    public record Geometry(
        double bodyWidth,
        double bodyHeight,
        double bodyLength,
        double wingArea,
        double wingSpan,
        double aileronArea,
        double elevonArea,
        double tailArm,
        double horizontalTailArea,
        double elevatorArea,
        double horizontalTailSpan,
        double verticalTailArea,
        double rudderArea,
        double canardArea,
        double aspectRatio
    ) {
        static Geometry from(
            EntityVehicleF_Physics vehicle,
            PMWeatherIVConfig.Values config,
            ModelSurfaceMap.PreparedModel model
        ) {
            ModelSurfaceMap.Bounds body = model.bodyBounds();
            ModelSurfaceMap.Bounds full = model.fullBounds();
            double bodyWidth = finiteClamp(body.spanX(), 0.8, 80.0, 2.0);
            double bodyHeight = finiteClamp(body.spanY(), 0.5, 40.0, 1.5);
            double bodyLength = finiteClamp(body.spanZ(), 0.8, 120.0, 3.0);

            double rawWing = totalWeight(model, ModelSurfaceMap.SurfaceKind.WING)
                + totalWeight(model, ModelSurfaceMap.SurfaceKind.AILERON)
                + totalWeight(model, ModelSurfaceMap.SurfaceKind.ELEVON);
            double rawAileron = totalWeight(model, ModelSurfaceMap.SurfaceKind.AILERON);
            double rawElevon = totalWeight(model, ModelSurfaceMap.SurfaceKind.ELEVON);
            double rawTaileron = totalWeight(model, ModelSurfaceMap.SurfaceKind.TAILERON);
            double rawHorizontalTail = totalWeight(model, ModelSurfaceMap.SurfaceKind.HORIZONTAL_TAIL)
                + totalWeight(model, ModelSurfaceMap.SurfaceKind.ELEVATOR)
                + rawTaileron;
            double rawElevator = totalWeight(model, ModelSurfaceMap.SurfaceKind.ELEVATOR)
                + rawTaileron;
            double rawVerticalTail = totalWeight(model, ModelSurfaceMap.SurfaceKind.VERTICAL_TAIL)
                + totalWeight(model, ModelSurfaceMap.SurfaceKind.RUDDER);
            double rawRudder = totalWeight(model, ModelSurfaceMap.SurfaceKind.RUDDER);
            double rawCanard = totalWeight(model, ModelSurfaceMap.SurfaceKind.CANARD)
                + totalWeight(model, ModelSurfaceMap.SurfaceKind.CANARDERON);

            double modelWingArea = Math.max(0.0, rawWing * 0.5);
            // Lifting-patch weight is OBJ area projected onto the panel normal.
            // The mesh commonly contains both faces of a thin wing, so half the
            // sum is a useful geometric planform estimate.  A sane authored MTS
            // area remains authoritative because content packs may intentionally
            // define the physical wing beyond what a render mesh exposes; model
            // area distributes that total across the detected components and is
            // the fallback when authored area is missing.
            double authoredWingArea = finiteNonNegative(vehicle.definition.motorized.wingArea);
            double runtimeWingArea = finiteNonNegative(vehicle.wingAreaVar.currentValue);
            double wingArea = authoredWingArea > 0.05
                ? authoredWingArea
                : modelWingArea > 0.05 ? modelWingArea : Math.max(0.5, runtimeWingArea);
            double areaScale = modelWingArea > 0.05
                ? Vec3d.clamp(wingArea / modelWingArea, 0.20, 10.0)
                : 1.0;

            double authoredAileronArea = finiteNonNegative(vehicle.definition.motorized.aileronArea);
            double runtimeAileronArea = finiteNonNegative(vehicle.aileronAreaVar.currentValue);
            double modelAileronArea = rawAileron * 0.5 * areaScale;
            double aileronArea = controlAreaFromAuthoredOrModel(
                authoredAileronArea, runtimeAileronArea, modelAileronArea
            );
            aileronArea = Vec3d.clamp(aileronArea, 0.0, wingArea * 0.38);

            double authoredElevatorArea = finiteNonNegative(vehicle.definition.motorized.elevatorArea);
            double runtimeElevatorArea = finiteNonNegative(vehicle.elevatorAreaVar.currentValue);
            double modelElevatorArea = rawElevator * 0.5 * areaScale;
            double elevatorArea = controlAreaFromAuthoredOrModel(
                authoredElevatorArea, runtimeElevatorArea, modelElevatorArea
            );

            double authoredRudderArea = finiteNonNegative(vehicle.definition.motorized.rudderArea);
            double runtimeRudderArea = finiteNonNegative(vehicle.rudderAreaVar.currentValue);
            double modelRudderArea = rawRudder * 0.5 * areaScale;
            double rudderArea = controlAreaFromAuthoredOrModel(
                authoredRudderArea, runtimeRudderArea, modelRudderArea
            );

            double elevonArea = rawElevon > EPSILON
                ? Math.max(rawElevon * 0.5 * areaScale, Math.max(aileronArea, elevatorArea))
                : 0.0;
            elevonArea = Vec3d.clamp(elevonArea, 0.0, wingArea * 0.55);

            double modelHorizontalTailArea = rawHorizontalTail * 0.5 * areaScale;
            double horizontalTailArea = (
                modelHorizontalTailArea > 0.02
                    ? modelHorizontalTailArea
                    : Math.max(elevatorArea * 1.25, wingArea * 0.08)
            );
            horizontalTailArea = Vec3d.clamp(horizontalTailArea, elevatorArea, wingArea * 0.45);
            elevatorArea = Vec3d.clamp(elevatorArea, 0.0, horizontalTailArea);

            double modelVerticalTailArea = rawVerticalTail * 0.5 * areaScale;
            double verticalTailArea = (
                modelVerticalTailArea > 0.02
                    ? modelVerticalTailArea
                    : Math.max(rudderArea * 1.25, wingArea * 0.04)
            );
            verticalTailArea = Vec3d.clamp(verticalTailArea, rudderArea, wingArea * 0.28);
            rudderArea = Vec3d.clamp(rudderArea, 0.0, verticalTailArea);

            double canardArea = Vec3d.clamp(
                rawCanard * 0.5 * areaScale,
                0.0,
                wingArea * 0.35
            );

            double authoredSpan = finiteNonNegative(vehicle.definition.motorized.wingSpan);
            double runtimeSpan = finiteNonNegative(vehicle.wingSpanVar.currentValue);
            double configuredSpan = authoredSpan > 0.1 ? authoredSpan : runtimeSpan;
            double minWingX = Double.POSITIVE_INFINITY;
            double maxWingX = Double.NEGATIVE_INFINITY;
            // Section centroids systematically under-report span for wide sections.
            // Preserve the actual frozen strip edge so aspect ratio, induced drag,
            // wing-to-tail downwash, convective stall scale and roll-flow geometry
            // all use the same physical planform that the sectional solver uses.
            double maximumWingSectionRadius = 0.0;
            double maximumControlSectionRadius = 0.0;
            double minTailX = Double.POSITIVE_INFINITY;
            double maxTailX = Double.NEGATIVE_INFINITY;
            double wingZ = 0.0;
            double wingWeight = 0.0;
            double tailZ = 0.0;
            double tailWeight = 0.0;
            for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
                switch (patch.kind()) {
                    case WING -> {
                        minWingX = Math.min(minWingX, patch.pointLocal().x());
                        maxWingX = Math.max(maxWingX, patch.pointLocal().x());
                        if (Double.isFinite(patch.sectionMaximumRadius())) {
                            maximumWingSectionRadius = Math.max(
                                maximumWingSectionRadius, patch.sectionMaximumRadius()
                            );
                        }
                        wingZ += patch.pointLocal().z() * patch.weight();
                        wingWeight += patch.weight();
                    }
                    case AILERON, ELEVON -> {
                        minWingX = Math.min(minWingX, patch.pointLocal().x());
                        maxWingX = Math.max(maxWingX, patch.pointLocal().x());
                        if (Double.isFinite(patch.sectionMaximumRadius())) {
                            maximumControlSectionRadius = Math.max(
                                maximumControlSectionRadius, patch.sectionMaximumRadius()
                            );
                        }
                        wingZ += patch.pointLocal().z() * patch.weight();
                        wingWeight += patch.weight();
                    }
                    case HORIZONTAL_TAIL, ELEVATOR, TAILERON, VERTICAL_TAIL, RUDDER -> {
                        minTailX = Math.min(minTailX, patch.pointLocal().x());
                        maxTailX = Math.max(maxTailX, patch.pointLocal().x());
                        tailZ += patch.pointLocal().z() * patch.weight();
                        tailWeight += patch.weight();
                    }
                    default -> {
                    }
                }
            }

            double centroidWingSpan = Double.isFinite(minWingX) && Double.isFinite(maxWingX)
                ? maxWingX - minWingX
                : full.spanX();
            double sectionRadius = maximumWingSectionRadius > 0.05
                ? maximumWingSectionRadius
                : maximumControlSectionRadius;
            double sectionBoundWingSpan = sectionRadius > 0.05
                ? 2.0 * sectionRadius
                : 0.0;
            double modelWingSpan = Math.max(centroidWingSpan, sectionBoundWingSpan);
            double wingSpan = Math.max(0.5, Math.max(configuredSpan, modelWingSpan));
            double fallbackTailArm = Math.max(bodyLength * 0.42, Math.abs(vehicle.definition.motorized.tailDistance));
            double tailArm = wingWeight > 0.0 && tailWeight > 0.0
                ? Math.abs(tailZ / tailWeight - wingZ / wingWeight)
                : fallbackTailArm;
            tailArm = Math.max(bodyLength * 0.25, tailArm);

            double horizontalTailSpan = Double.isFinite(minTailX) && Double.isFinite(maxTailX)
                ? Math.max(0.5, maxTailX - minTailX)
                : Math.max(bodyWidth * 0.75, wingSpan * 0.30);
            double aspectRatio = wingArea > 0.01 ? wingSpan * wingSpan / wingArea : 1.0;

            return new Geometry(
                bodyWidth,
                bodyHeight,
                bodyLength,
                wingArea,
                wingSpan,
                aileronArea,
                elevonArea,
                tailArm,
                horizontalTailArea,
                elevatorArea,
                horizontalTailSpan,
                verticalTailArea,
                rudderArea,
                canardArea,
                Math.max(1.0, aspectRatio)
            );
        }

        static double totalWeight(
            ModelSurfaceMap.PreparedModel model,
            ModelSurfaceMap.SurfaceKind kind
        ) {
            double total = 0.0;
            for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
                if (patch.kind() == kind && Double.isFinite(patch.weight()) && patch.weight() > 0.0) {
                    total += patch.weight();
                }
            }
            return total;
        }

        static double finiteNonNegative(double value) {
            return Double.isFinite(value) ? Math.max(0.0, value) : 0.0;
        }

        /**
         * IV content packs do not all use control-area fields as square metres.
         * Some use them as native-flight tuning coefficients, while others provide
         * physically scaled areas. When a classified OBJ control surface exists,
         * retain a sane authored value but reject values that differ from the
         * frozen mesh planform by more than a universal 5:1 ratio. This keeps
         * well-authored MTS packs unchanged and prevents tiny tuning coefficients
         * from shrinking large rendered elevators, rudders, or ailerons.
         */
        static double controlAreaFromAuthoredOrModel(
            double authoredArea,
            double runtimeArea,
            double modelArea
        ) {
            double safeAuthored = finiteNonNegative(authoredArea);
            double safeRuntime = finiteNonNegative(runtimeArea);
            double safeModel = finiteNonNegative(modelArea);
            double configured = safeAuthored > 0.01
                ? safeAuthored
                : safeRuntime > 0.01 ? safeRuntime : safeModel;
            if (safeModel <= 0.01 || configured <= 0.01) {
                return Math.max(configured, safeModel);
            }
            double ratio = configured / safeModel;
            return ratio < MIN_AUTHORED_TO_MODEL_CONTROL_AREA_RATIO
                || ratio > MAX_AUTHORED_TO_MODEL_CONTROL_AREA_RATIO
                    ? safeModel
                    : configured;
        }

        Vec3d inertiaForMass(double mass) {
            return inertiaForMassAndWingSpan(mass, wingSpan);
        }

        Vec3d inertiaForMassAndWingSpan(double mass, double liveWingSpan) {
            double safeWingSpan = Double.isFinite(liveWingSpan) && liveWingSpan > 0.10
                ? liveWingSpan : wingSpan;
            double effectiveWidth = Math.max(bodyWidth, safeWingSpan * 0.72);
            double effectiveLength = Math.max(bodyLength, tailArm * 1.35);
            double ix = mass * (bodyHeight * bodyHeight + effectiveLength * effectiveLength) / 12.0;
            double iy = mass * (effectiveWidth * effectiveWidth + effectiveLength * effectiveLength) / 12.0;
            double iz = mass * (effectiveWidth * effectiveWidth + bodyHeight * bodyHeight) / 12.0;
            return new Vec3d(Math.max(1.0, ix), Math.max(1.0, iy), Math.max(1.0, iz));
        }
    }

    public static final class SurfaceAreaPlan {
        final EnumMap<ModelSurfaceMap.SurfaceKind, Double> totalWeights =
            new EnumMap<>(ModelSurfaceMap.SurfaceKind.class);
        final EnumMap<ModelSurfaceMap.SurfaceKind, Double> totalAreas =
            new EnumMap<>(ModelSurfaceMap.SurfaceKind.class);

        static SurfaceAreaPlan from(ModelSurfaceMap.PreparedModel model, Geometry geometry) {
            SurfaceAreaPlan plan = new SurfaceAreaPlan();
            for (ModelSurfaceMap.SurfaceKind kind : ModelSurfaceMap.SurfaceKind.values()) {
                plan.totalWeights.put(kind, 0.0);
                plan.totalAreas.put(kind, 0.0);
            }
            for (ModelSurfaceMap.LiftingPatch patch : model.liftingPatches()) {
                plan.totalWeights.merge(patch.kind(), Math.max(0.0, patch.weight()), Double::sum);
            }

            double fixedWingWeight = plan.weight(ModelSurfaceMap.SurfaceKind.WING);
            double aileronWeight = plan.weight(ModelSurfaceMap.SurfaceKind.AILERON);
            double elevonWeight = plan.weight(ModelSurfaceMap.SurfaceKind.ELEVON);
            if (fixedWingWeight <= EPSILON && elevonWeight > EPSILON) {
                plan.totalAreas.put(ModelSurfaceMap.SurfaceKind.ELEVON, geometry.wingArea());
            } else if (fixedWingWeight <= EPSILON && aileronWeight > EPSILON) {
                plan.totalAreas.put(ModelSurfaceMap.SurfaceKind.AILERON, geometry.wingArea());
            } else {
                double aileronArea = aileronWeight > EPSILON ? geometry.aileronArea() : 0.0;
                double elevonArea = elevonWeight > EPSILON ? geometry.elevonArea() : 0.0;
                double controlArea = Math.min(geometry.wingArea() * 0.55, aileronArea + elevonArea);
                double fixedArea = Math.max(0.0, geometry.wingArea() - controlArea);
                plan.totalAreas.put(ModelSurfaceMap.SurfaceKind.WING, fixedArea);
                if (controlArea > 0.0) {
                    double requested = Math.max(EPSILON, aileronArea + elevonArea);
                    plan.totalAreas.put(ModelSurfaceMap.SurfaceKind.AILERON,
                        controlArea * aileronArea / requested);
                    plan.totalAreas.put(ModelSurfaceMap.SurfaceKind.ELEVON,
                        controlArea * elevonArea / requested);
                }
            }

            double fixedCanardWeight = plan.weight(ModelSurfaceMap.SurfaceKind.CANARD);
            double canarderonWeight = plan.weight(ModelSurfaceMap.SurfaceKind.CANARDERON);
            double totalCanardWeight = fixedCanardWeight + canarderonWeight;
            if (totalCanardWeight > EPSILON) {
                plan.totalAreas.put(ModelSurfaceMap.SurfaceKind.CANARD,
                    geometry.canardArea() * fixedCanardWeight / totalCanardWeight);
                plan.totalAreas.put(ModelSurfaceMap.SurfaceKind.CANARDERON,
                    geometry.canardArea() * canarderonWeight / totalCanardWeight);
            }

            double fixedHorizontalWeight = plan.weight(ModelSurfaceMap.SurfaceKind.HORIZONTAL_TAIL);
            double elevatorWeight = plan.weight(ModelSurfaceMap.SurfaceKind.ELEVATOR);
            double taileronWeight = plan.weight(ModelSurfaceMap.SurfaceKind.TAILERON);
            double movingHorizontalWeight = elevatorWeight + taileronWeight;
            if (fixedHorizontalWeight <= EPSILON && movingHorizontalWeight > EPSILON) {
                // TAILERON is strong authored evidence for an all-moving pitch/roll
                // control surface, but it is not permission to replace IV's authored
                // elevator-area semantics with every horizontal triangle in the OBJ.
                // 0.11.16 correctly removed geometry-guessed duplicate fixed-tail lift,
                // but this branch then expanded the remaining MiG tailerons from the
                // IV-authored 7.2 m^2 control area to the full inferred tail planform.
                // Keep ordinary elevator-only fallback behavior for incomplete models;
                // when an authored all-moving taileron exists, distribute the existing
                // elevator/control area across the moving surfaces instead.
                double total = Math.max(EPSILON, movingHorizontalWeight);
                double movingPlanformArea = taileronWeight > EPSILON
                    ? Math.min(geometry.elevatorArea(), geometry.horizontalTailArea())
                    : geometry.horizontalTailArea();
                plan.totalAreas.put(ModelSurfaceMap.SurfaceKind.ELEVATOR,
                    movingPlanformArea * elevatorWeight / total);
                plan.totalAreas.put(ModelSurfaceMap.SurfaceKind.TAILERON,
                    movingPlanformArea * taileronWeight / total);
            } else {
                double movingArea = movingHorizontalWeight > EPSILON ? geometry.elevatorArea() : 0.0;
                movingArea = Math.min(movingArea, geometry.horizontalTailArea());
                double total = Math.max(EPSILON, movingHorizontalWeight);
                plan.totalAreas.put(ModelSurfaceMap.SurfaceKind.ELEVATOR,
                    movingArea * elevatorWeight / total);
                plan.totalAreas.put(ModelSurfaceMap.SurfaceKind.TAILERON,
                    movingArea * taileronWeight / total);
                plan.totalAreas.put(ModelSurfaceMap.SurfaceKind.HORIZONTAL_TAIL,
                    Math.max(0.0, geometry.horizontalTailArea() - movingArea));
            }

            double fixedVerticalWeight = plan.weight(ModelSurfaceMap.SurfaceKind.VERTICAL_TAIL);
            double rudderWeight = plan.weight(ModelSurfaceMap.SurfaceKind.RUDDER);
            if (fixedVerticalWeight <= EPSILON && rudderWeight > EPSILON) {
                plan.totalAreas.put(ModelSurfaceMap.SurfaceKind.RUDDER, geometry.verticalTailArea());
            } else {
                double rudderArea = rudderWeight > EPSILON ? geometry.rudderArea() : 0.0;
                rudderArea = Math.min(rudderArea, geometry.verticalTailArea());
                plan.totalAreas.put(ModelSurfaceMap.SurfaceKind.RUDDER, rudderArea);
                plan.totalAreas.put(ModelSurfaceMap.SurfaceKind.VERTICAL_TAIL,
                    Math.max(0.0, geometry.verticalTailArea() - rudderArea));
            }
            return plan;
        }

        double weight(ModelSurfaceMap.SurfaceKind kind) {
            return totalWeights.getOrDefault(kind, 0.0);
        }

        double totalArea(ModelSurfaceMap.SurfaceKind kind) {
            return totalAreas.getOrDefault(kind, 0.0);
        }

        double areaFor(ModelSurfaceMap.LiftingPatch patch) {
            double totalWeight = weight(patch.kind());
            double totalArea = totalAreas.getOrDefault(patch.kind(), 0.0);
            if (totalWeight <= EPSILON || totalArea <= 0.0) {
                return 0.0;
            }
            return totalArea * Math.max(0.0, patch.weight()) / totalWeight;
        }
    }
}
