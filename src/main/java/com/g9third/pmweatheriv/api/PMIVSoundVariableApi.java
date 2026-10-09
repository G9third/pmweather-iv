package com.g9third.pmweatheriv.api;

import com.g9third.pmweatheriv.network.AircraftStateNetwork;
import com.g9third.pmweatheriv.physics.AirWarningSystem;
import java.util.Set;
import minecrafttransportsimulator.baseclasses.ComputedVariable;
import minecrafttransportsimulator.entities.components.AEntityD_Definable;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;

/**
 * Content-pack-facing air-warning sound API.
 *
 * <p>PMIV exposes warning telemetry as ordinary IV computed variables. Content
 * packs therefore use the normal IV {@code rendering.sounds} JSON format and do
 * not need PMIV-specific sound files, packets, or Java code.</p>
 */
public final class PMIVSoundVariableApi {
    public static final String WIND_SHEAR_WARNING = "pmiv_warning_windshear";
    public static final String WIND_SHEAR_LEVEL = "pmiv_warning_windshear_level";
    public static final String WIND_SHEAR_DELTA_MPS = "pmiv_windshear_delta_mps";
    public static final String WIND_SHEAR_DELTA_KNOTS = "pmiv_windshear_delta_knots";
    public static final String WIND_SHEAR_VERTICAL_DELTA_MPS = "pmiv_windshear_vertical_delta_mps";
    public static final String WIND_SHEAR_VERTICAL_DELTA_KNOTS = "pmiv_windshear_vertical_delta_knots";
    public static final String WIND_SPEED_MPS = "pmiv_wind_speed_mps";
    public static final String WIND_SPEED_KNOTS = "pmiv_wind_speed_knots";
    public static final String VERTICAL_WIND_MPS = "pmiv_vertical_wind_mps";
    public static final String VERTICAL_WIND_KNOTS = "pmiv_vertical_wind_knots";

    public static final Set<String> CONTENT_PACK_VARIABLES = Set.of(
        WIND_SHEAR_WARNING,
        WIND_SHEAR_LEVEL,
        WIND_SHEAR_DELTA_MPS,
        WIND_SHEAR_DELTA_KNOTS,
        WIND_SHEAR_VERTICAL_DELTA_MPS,
        WIND_SHEAR_VERTICAL_DELTA_KNOTS,
        WIND_SPEED_MPS,
        WIND_SPEED_KNOTS,
        VERTICAL_WIND_MPS,
        VERTICAL_WIND_KNOTS
    );

    private PMIVSoundVariableApi() {
    }

    /** Returns true when PMIV owns the named IV computed variable. */
    public static boolean supports(String variable) {
        return CONTENT_PACK_VARIABLES.contains(variable);
    }

    /**
     * Creates a normal IV computed variable backed by PMIV's synchronized warning state.
     * Returns {@code null} for non-PMIV variables so IV can create them normally.
     */
    public static ComputedVariable createComputedVariable(
        AEntityD_Definable<?> definable,
        String variable
    ) {
        if (definable == null || !supports(variable)) {
            return null;
        }
        EntityVehicleF_Physics vehicle = owningVehicle(definable);
        if (vehicle == null) {
            return null;
        }
        return new ComputedVariable(
            definable,
            variable,
            partialTicks -> value(vehicle, variable),
            false
        );
    }

    /** Resolves both vehicle-level and normal attached-part sound definitions. */
    public static EntityVehicleF_Physics owningVehicle(AEntityD_Definable<?> definable) {
        if (definable instanceof EntityVehicleF_Physics vehicle) {
            return vehicle;
        }
        if (definable instanceof APart part && part.masterEntity instanceof EntityVehicleF_Physics vehicle) {
            return vehicle;
        }
        return null;
    }

    /** Current value for an IV animation/sound variable. */
    public static double value(EntityVehicleF_Physics vehicle, String variable) {
        AircraftStateNetwork.WarningView warning = AircraftStateNetwork.warningView(vehicle);
        if (warning == null) {
            return 0.0;
        }
        return switch (variable) {
            case WIND_SHEAR_WARNING -> warning.windShearWarning() ? 1.0 : 0.0;
            case WIND_SHEAR_LEVEL -> warning.windShearLevel();
            case WIND_SHEAR_DELTA_MPS -> warning.windShearDeltaMps();
            case WIND_SHEAR_DELTA_KNOTS -> warning.windShearDeltaMps()
                / AirWarningSystem.KNOT_TO_METERS_PER_SECOND;
            case WIND_SHEAR_VERTICAL_DELTA_MPS -> warning.windShearVerticalDeltaMps();
            case WIND_SHEAR_VERTICAL_DELTA_KNOTS -> warning.windShearVerticalDeltaMps()
                / AirWarningSystem.KNOT_TO_METERS_PER_SECOND;
            case WIND_SPEED_MPS -> warning.windSpeedMps();
            case WIND_SPEED_KNOTS -> warning.windSpeedMps()
                / AirWarningSystem.KNOT_TO_METERS_PER_SECOND;
            case VERTICAL_WIND_MPS -> warning.verticalWindMps();
            case VERTICAL_WIND_KNOTS -> warning.verticalWindMps()
                / AirWarningSystem.KNOT_TO_METERS_PER_SECOND;
            default -> 0.0;
        };
    }
}
