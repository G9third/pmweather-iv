package com.g9third.pmweatheriv.physics;

import minecrafttransportsimulator.baseclasses.ComputedVariable;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.mcinterface.IWrapperNBT;

/** Per-aircraft preferences registered through IV's saved computed-variable list. */
public final class AutoTrimPreferences {
    public static final String DISABLED_VARIABLE = "pmiv_auto_trim_disabled";
    public static final String NOTICE_SHOWN_VARIABLE = "pmiv_auto_trim_notice_shown";

    private AutoTrimPreferences() {}

    /** Called from the IV vehicle constructor after its saved variables have loaded. */
    public static void attach(EntityVehicleF_Physics vehicle, IWrapperNBT savedData) {
        if (vehicle == null) return;
        attachVariable(vehicle, DISABLED_VARIABLE, savedData);
        attachVariable(vehicle, NOTICE_SHOWN_VARIABLE, savedData);
    }

    public static boolean disabled(EntityVehicleF_Physics vehicle) {
        return value(vehicle, DISABLED_VARIABLE) >= 0.5;
    }

    public static void setDisabled(EntityVehicleF_Physics vehicle, boolean disabled) {
        setValue(vehicle, DISABLED_VARIABLE, disabled ? 1.0 : 0.0);
    }

    public static boolean noticeShown(EntityVehicleF_Physics vehicle) {
        return value(vehicle, NOTICE_SHOWN_VARIABLE) >= 0.5;
    }

    /** Auto-start is allowed only for the default-on, pilot-owned fixed-wing path. */
    public static boolean shouldAutoStart(boolean globallyEnabled, boolean supportedFixedWing,
                                          boolean optedOut, boolean hasControllerPilot) {
        return globallyEnabled && supportedFixedWing && !optedOut && hasControllerPilot;
    }

    public static void setNoticeShown(EntityVehicleF_Physics vehicle) {
        setValue(vehicle, NOTICE_SHOWN_VARIABLE, 1.0);
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
