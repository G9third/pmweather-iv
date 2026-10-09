package com.g9third.pmweatheriv;

import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import net.neoforged.fml.config.ModConfig;
import net.neoforged.fml.event.config.ModConfigEvent;
import net.neoforged.neoforge.common.ModConfigSpec;

/** Gameplay configuration for PMAero loads and the shared Sable vehicle bridge. */
public final class PMWeatherIVConfig {
    private static final ModConfigSpec.Builder BUILDER = new ModConfigSpec.Builder();

    private static final ModConfigSpec.BooleanValue ENABLED;

    private static final ModConfigSpec.DoubleValue BODY_AXIAL_CD;
    private static final ModConfigSpec.DoubleValue BODY_CROSSFLOW_CD;
    private static final ModConfigSpec.DoubleValue GROUND_FALLBACK_AXIAL_CD;
    private static final ModConfigSpec.DoubleValue GROUND_CROSSFLOW_CD;
    private static final ModConfigSpec.DoubleValue ROAD_ROOF_SUCTION_CP;
    private static final ModConfigSpec.DoubleValue ROAD_COAST_BRAKE_MU;
    private static final ModConfigSpec.DoubleValue BASE_WING_CD;
    private static final ModConfigSpec.DoubleValue LIFT_SLOPE;
    private static final ModConfigSpec.DoubleValue MAX_LIFT_COEFFICIENT;
    private static final ModConfigSpec.IntValue MAXIMUM_MODEL_PRESSURE_PATCHES;
    private static final ModConfigSpec.IntValue MAXIMUM_GROUND_PRESSURE_PATCHES;

    private static final ModConfigSpec.DoubleValue WIND_SHEAR_WARNING_THRESHOLD_KNOTS;

    private static final ModConfigSpec.BooleanValue ENABLE_TERRAIN_BLOCK_BREAKING;

    public static final ModConfigSpec SPEC;
    private static volatile Values values = Values.defaults();

    static {
        BUILDER.push("general");
        ENABLED = bool("enabled", true,
            "Enable PMWeather-IV persistent Sable vehicle physics. IV supplies authored controls and drivetrain; PMIV applies aircraft/road aerodynamic loads and bounded tire contacts.");
        BUILDER.pop();

        BUILDER.push("aerodynamics");
        BODY_AXIAL_CD = number("fuselageAxialDragCoefficient", 0.18, 0.0, 3.0,
            "Blunt-body axial pressure coefficient ceiling. Model-backed bodies reduce it from frozen fineness and add Reynolds skin friction over represented surface area; this coefficient applies to aircraft fuselages.");
        BODY_CROSSFLOW_CD = number("fuselageCrossflowDragCoefficient", 0.95, 0.0, 3.0,
            "Crossflow coefficient for frozen aircraft body side, top, and bottom projected areas.");
        BASE_WING_CD = number("baseWingDragCoefficient", 0.028, 0.0, 0.5,
            "Profile-drag coefficient for non-model fallback lifting geometry. Model-backed surfaces derive clean profile drag from Reynolds-number skin friction and their frozen mesh wetted/reference-area ratio.");
        LIFT_SLOPE = number("liftSlopePerRadian", 4.8, 0.5, 8.0,
            "Attached-flow lift slope used before separation blends the surface toward flat-plate pressure.");
        MAX_LIFT_COEFFICIENT = number("maxLiftCoefficient", 1.65, 0.2, 4.0,
            "Maximum attached-flow clean-wing lift coefficient before flap contribution and stall separation.");
        MAXIMUM_MODEL_PRESSURE_PATCHES = integer("maximumModelPressurePatches", 14, 8, 14,
            "Hard ceiling for surface-anchored body-pressure sections. The model normally uses 10-14, all sampled in one tick.");
        BUILDER.pop();

        BUILDER.push("groundAerodynamics");
        MAXIMUM_GROUND_PRESSURE_PATCHES = integer("maximumPressurePatches", 64, 6, 256,
            "Road pressure station ceiling. Exterior silhouette regions are split in both surface directions; each region samples an actual represented exterior face while applying force at its area centroid.");
        GROUND_FALLBACK_AXIAL_CD = number("fallbackAxialDragCoefficient", 0.6, 0.0, 3.0,
            "Road-body frontal pressure coefficient used only when the live IV dragCoefficient is invalid. Valid pack coefficients, including zero, are preserved without aircraft fineness scaling.");
        GROUND_CROSSFLOW_CD = number("crossflowDragCoefficient", 0.95, 0.0, 3.0,
            "Road-body side, top and bottom pressure coefficient. Road tires use complete relative-air pressure and replace the native calm-air drag term.");
        ROAD_ROOF_SUCTION_CP = number("roofSuctionPressureCoefficient", 0.15, 0.0, 1.0,
            "Generic road upper-surface pressure-drop coefficient applied to local tangential dynamic pressure and represented roof area. Acts along the represented upper-surface normal, including when rolled or airborne. This is an approximate uncalibrated body-pressure extension, not a wind-speed rollover threshold. Zero disables it. Aircraft body aerodynamics are unaffected.");
        BUILDER.pop();

        BUILDER.push("groundContacts");
        ROAD_COAST_BRAKE_MU = number("maximumCoastingBrakeCoefficient", 0.08, 0.0, 1.0,
            "Maximum closed-throttle native engine-braking demand as a fraction of actual tire normal load. Service/parking brake commands retain the full authored tire grip. Drive traction and wind slip share the joint friction ellipse; unloaded tires provide no force.");
        BUILDER.pop();

        BUILDER.push("terrainDamage");
        ENABLE_TERRAIN_BLOCK_BREAKING = bool("enableBlockBreaking", true,
            "Allow managed vehicle crashes to fracture and remove Minecraft terrain through the integrated True-Impact-derived material solver. IV's global vehicleBlockBreaking setting must also be enabled. Disable this to keep terrain fully rigid while preserving Sable collision and crash response.");
        BUILDER.pop();

        BUILDER.push("airWarnings");
        WIND_SHEAR_WARNING_THRESHOLD_KNOTS = number("windShearWarningThresholdKnots", 15.0, 1.0, 100.0,
            "One-second source-wind vector change that activates pmiv_warning_windshear. The 15-knot default matches the familiar FAA LLWAS wind-shear alert magnitude, but PMIV is an onboard game measurement rather than an LLWAS implementation. Content packs may use the raw PMIV air-warning variables with their own IV animation clamps.");
        BUILDER.pop();

        SPEC = BUILDER.build();
    }

    private PMWeatherIVConfig() {
    }

    public static Values get() {
        return values;
    }

    public static void onLoading(ModConfigEvent.Loading event) {
        refreshIfOurs(event.getConfig());
    }

    public static void onReloading(ModConfigEvent.Reloading event) {
        refreshIfOurs(event.getConfig());
    }

    private static void refreshIfOurs(ModConfig config) {
        if (config != null && config.getSpec() == SPEC) {
            values = new Values(
                ENABLED.get(),
                BODY_AXIAL_CD.get(), BODY_CROSSFLOW_CD.get(), GROUND_FALLBACK_AXIAL_CD.get(),
                GROUND_CROSSFLOW_CD.get(), ROAD_ROOF_SUCTION_CP.get(), ROAD_COAST_BRAKE_MU.get(), BASE_WING_CD.get(), LIFT_SLOPE.get(),
                MAX_LIFT_COEFFICIENT.get(), MAXIMUM_MODEL_PRESSURE_PATCHES.get(), MAXIMUM_GROUND_PRESSURE_PATCHES.get(),
                WIND_SHEAR_WARNING_THRESHOLD_KNOTS.get(), ENABLE_TERRAIN_BLOCK_BREAKING.get()
            );
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "CONFIG aero=PMAERO integration=SABLE windBatch=FORCE_POINTS_ONCE_PER_TICK values=" + values
            );
            }
        }
    }

    private static ModConfigSpec.BooleanValue bool(String key, boolean defaultValue, String comment) {
        return BUILDER.comment(comment).define(key, defaultValue);
    }

    private static ModConfigSpec.IntValue integer(String key, int defaultValue, int minimum, int maximum, String comment) {
        return BUILDER.comment(comment).defineInRange(key, defaultValue, minimum, maximum);
    }

    private static ModConfigSpec.DoubleValue number(
        String key,
        double defaultValue,
        double minimum,
        double maximum,
        String comment
    ) {
        return BUILDER.comment(comment).defineInRange(key, defaultValue, minimum, maximum);
    }

    public record Values(
        boolean enabled,
        double bodyAxialDragCoefficient,
        double bodyCrossflowDragCoefficient,
        double groundFallbackAxialDragCoefficient,
        double groundCrossflowDragCoefficient,
        double roadRoofSuctionPressureCoefficient,
        double roadMaximumCoastingBrakeCoefficient,
        double baseWingDragCoefficient,
        double liftSlopePerRadian,
        double maxLiftCoefficient,
        int maximumModelPressurePatches,
        int maximumGroundPressurePatches,
        double windShearWarningThresholdKnots,
        boolean enableBlockBreaking
    ) {
        public static Values defaults() {
            return new Values(
                true, 0.18, 0.95, 0.6, 0.95, 0.15, 0.08, 0.028, 4.8, 1.65, 14, 64, 15.0, true
            );
        }
    }
}
