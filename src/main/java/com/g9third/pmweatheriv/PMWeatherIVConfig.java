package com.g9third.pmweatheriv;

import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import net.neoforged.fml.config.ModConfig;
import net.neoforged.fml.event.config.ModConfigEvent;
import net.neoforged.neoforge.common.ModConfigSpec;

/** Player-facing settings for PMWeather-IV vehicle physics and flight assist. */
public final class PMWeatherIVConfig {
    private static final ModConfigSpec.Builder BUILDER = new ModConfigSpec.Builder();

    private static final ModConfigSpec.BooleanValue ENABLED;
    private static final ModConfigSpec.BooleanValue AUTO_TRIM_ENABLED;
    private static final ModConfigSpec.DoubleValue WIND_SHEAR_WARNING_THRESHOLD_KNOTS;
    private static final ModConfigSpec.BooleanValue ENABLE_TERRAIN_BLOCK_BREAKING;

    public static final ModConfigSpec SPEC;
    private static volatile Values values = Values.defaults();
    private static volatile boolean autoTrimEnabled = true;

    static {
        BUILDER.comment("General settings").translation("config.pmweather_iv.category.general").push("general");
        ENABLED = bool("enabled", true,
            "config.pmweather_iv.general.enabled",
            "Enable PMWeather-IV physics for supported vehicles.");
        BUILDER.pop();

        BUILDER.comment("Auto trim settings").translation("config.pmweather_iv.category.auto_trim").push("autoTrim");
        AUTO_TRIM_ENABLED = bool("enabled", true,
            "config.pmweather_iv.auto_trim.enabled",
            "Enable automatic elevator trim for supported fixed-wing aircraft. Use the auto-trim key to switch it off or on for one aircraft.");
        BUILDER.pop();

        BUILDER.comment("Air warning settings").translation("config.pmweather_iv.category.air_warnings").push("airWarnings");
        WIND_SHEAR_WARNING_THRESHOLD_KNOTS = number("windShearWarningThresholdKnots", 15.0, 1.0, 100.0,
            "config.pmweather_iv.air_warnings.wind_shear_threshold",
            "Trigger the wind-shear warning when the wind changes by this many knots in one second.");
        BUILDER.pop();

        BUILDER.comment("Terrain damage settings").translation("config.pmweather_iv.category.terrain_damage").push("terrainDamage");
        ENABLE_TERRAIN_BLOCK_BREAKING = bool("enableBlockBreaking", true,
            "config.pmweather_iv.terrain_damage.enable_block_breaking",
            "Let vehicle crashes break terrain. Requires IV's vehicleBlockBreaking setting.");
        BUILDER.pop();

        SPEC = BUILDER.build();
    }

    private PMWeatherIVConfig() {}

    public static Values get() {
        return values;
    }

    /** Separate from Values to preserve the established physics config contract. */
    public static boolean autoTrimEnabled() {
        return autoTrimEnabled;
    }

    public static void onLoading(ModConfigEvent.Loading event) {
        refreshIfOurs(event.getConfig());
    }

    public static void onReloading(ModConfigEvent.Reloading event) {
        refreshIfOurs(event.getConfig());
    }

    private static void refreshIfOurs(ModConfig config) {
        if (config != null && config.getSpec() == SPEC) {
            Values physicsDefaults = Values.defaults();
            autoTrimEnabled = AUTO_TRIM_ENABLED.get();
            values = new Values(
                ENABLED.get(),
                physicsDefaults.bodyAxialDragCoefficient(), physicsDefaults.bodyCrossflowDragCoefficient(),
                physicsDefaults.groundFallbackAxialDragCoefficient(), physicsDefaults.groundCrossflowDragCoefficient(),
                physicsDefaults.roadRoofSuctionPressureCoefficient(), physicsDefaults.roadMaximumCoastingBrakeCoefficient(),
                physicsDefaults.baseWingDragCoefficient(), physicsDefaults.liftSlopePerRadian(),
                physicsDefaults.maxLiftCoefficient(), physicsDefaults.maximumModelPressurePatches(),
                physicsDefaults.maximumGroundPressurePatches(), WIND_SHEAR_WARNING_THRESHOLD_KNOTS.get(),
                ENABLE_TERRAIN_BLOCK_BREAKING.get()
            );
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log("CONFIG aero=PMAERO integration=SABLE windBatch=FORCE_POINTS_ONCE_PER_TICK values="
                    + values + " autoTrimEnabled=" + autoTrimEnabled);
            }
        }
    }

    private static ModConfigSpec.BooleanValue bool(String key, boolean defaultValue,
                                                    String translationKey, String comment) {
        return BUILDER.comment(comment).translation(translationKey).define(key, defaultValue);
    }

    private static ModConfigSpec.DoubleValue number(
        String key,
        double defaultValue,
        double minimum,
        double maximum,
        String translationKey,
        String comment
    ) {
        return BUILDER.comment(comment).translation(translationKey)
            .defineInRange(key, defaultValue, minimum, maximum);
    }

    /** Stable physics accessor contract; internal coefficients remain code-owned defaults. */
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
