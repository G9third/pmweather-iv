package com.g9third.pmweatheriv.physics;

import minecrafttransportsimulator.baseclasses.ComputedVariable;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.mcinterface.IWrapperNBT;

/** Per-aircraft auto-trim opt-in registered through IV's saved computed-variable list. */
public final class AutoTrimPreferences {
    public static final String ENABLED_VARIABLE = "pmiv_auto_trim_enabled";

    private AutoTrimPreferences() {}

    /** Called from the IV vehicle constructor after its saved variables have loaded. */
    public static void attach(EntityVehicleF_Physics vehicle, IWrapperNBT savedData) {
        if (vehicle == null) return;
        attachVariable(vehicle, ENABLED_VARIABLE, savedData);
    }

    /** A missing variable in a new vehicle or an older save is intentionally off. */
    public static boolean enabled(EntityVehicleF_Physics vehicle) {
        return enabledValue(value(vehicle, ENABLED_VARIABLE));
    }

    /** Missing, malformed, and default-zero saved values all remain off. */
    public static boolean enabledValue(double savedValue) {
        return Double.isFinite(savedValue) && savedValue >= 0.5;
    }

    public static void setEnabled(EntityVehicleF_Physics vehicle, boolean enabled) {
        setValue(vehicle, ENABLED_VARIABLE, enabled ? 1.0 : 0.0);
    }

    /** Auto-start requires an explicit per-aircraft opt-in and its current pilot. */
    public static boolean shouldAutoStart(boolean optedIn, boolean supportedFixedWing,
                                          boolean hasControllerPilot) {
        return optedIn && supportedFixedWing && hasControllerPilot;
    }

    private static void attachVariable(EntityVehicleF_Physics vehicle, String key, IWrapperNBT savedData) {
        if (!vehicle.containsVariable(key)) {
            // The IWrapperNBT constructor marks this variable for IV's normal
            // "variables" NBT list and restores its value on the next load.
            vehicle.addVariable(new ComputedVariable(vehicle, key, savedData));
        }
    }

    private static double value(EntityVehicleF_Physics vehicle, String key) {
        if (vehicle == null) return 0.0;
        ComputedVariable variable = vehicle.getOrCreateVariable(key);
        return key.equals(variable.variableKey) && variable.entity == vehicle
            ? variable.currentValue : 0.0;
    }

    private static void setValue(EntityVehicleF_Physics vehicle, String key, double value) {
        if (vehicle == null) return;
        ComputedVariable variable = vehicle.getOrCreateVariable(key);
        if (!key.equals(variable.variableKey) || variable.entity != vehicle) {
            throw new IllegalStateException("IV auto-trim preference variable was not attached: " + key);
        }
        // PMIV status packets update the pilot HUD. These private values are only
        // persisted by IV and are not exposed as pack animation variables.
        variable.setTo(value, false);
    }
}
