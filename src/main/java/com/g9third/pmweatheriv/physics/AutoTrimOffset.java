package com.g9third.pmweatheriv.physics;

import java.util.List;
import minecrafttransportsimulator.baseclasses.ComputedVariable;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartPropeller;
import minecrafttransportsimulator.jsondefs.AJSONMultiModelProvider;
import minecrafttransportsimulator.jsondefs.JSONVariableModifier;
import minecrafttransportsimulator.mcinterface.IWrapperNBT;

/** Reversible PMIV trim offset for fixed-wing packs that author trim modifiers. */
public final class AutoTrimOffset {
    public static final String REQUESTED_VARIABLE = "pmiv_auto_trim_requested_offset";
    public static final String APPLIED_VARIABLE = "pmiv_auto_trim_applied_offset";

    private AutoTrimOffset() {}

    public static void attach(EntityVehicleF_Physics vehicle, IWrapperNBT savedData) {
        if (vehicle == null || vehicle.definition == null || vehicle.definition.motorized == null
            || !vehicle.definition.motorized.isAircraft || vehicle.definition.motorized.isBlimp
            || !hasElevatorTrimModifier(vehicle)) return;
        attachVariable(vehicle, REQUESTED_VARIABLE, savedData);
        attachVariable(vehicle, APPLIED_VARIABLE, savedData);
    }

    /** Only fixed-wing definitions that author elevator-trim modifiers need the offset layer. */
    public static boolean usesAuthoredModifierOffset(EntityVehicleF_Physics vehicle) {
        if (vehicle == null || vehicle.definition == null || vehicle.definition.motorized == null
            || !vehicle.definition.motorized.isAircraft || vehicle.definition.motorized.isBlimp) return false;
        AircraftState state = ((AircraftStateAccess) vehicle).pmweatherIv$getAircraftState();
        AJSONMultiModelProvider definition = vehicle.definition;
        if (state.autoTrimOverlayDefinition != definition) {
            state.autoTrimOverlayDefinition = definition;
            state.autoTrimOverlayDefinitionHasModifier = hasElevatorTrimModifier(vehicle);
            state.autoTrimOverlayPartCount = -1;
        }
        if (!state.autoTrimOverlayDefinitionHasModifier) return false;
        int partCount = vehicle.allParts == null ? 0 : vehicle.allParts.size();
        if (state.autoTrimOverlayPartCount != partCount) {
            state.autoTrimOverlayPartCount = partCount;
            state.autoTrimOverlayPartsHaveRotor = hasRotorPart(vehicle);
        }
        return !state.autoTrimOverlayPartsHaveRotor;
    }

    public static double requested(EntityVehicleF_Physics vehicle) {
        return read(vehicle, REQUESTED_VARIABLE);
    }

    public static double applied(EntityVehicleF_Physics vehicle) {
        return read(vehicle, APPLIED_VARIABLE);
    }

    public static void setRequested(EntityVehicleF_Physics vehicle, double offset, boolean sync) {
        write(vehicle, REQUESTED_VARIABLE, offset, sync);
    }

    /** A stopped or paused controller holds the offset already achieved by the modifier pass. */
    public static void freezeAtAppliedOffset(EntityVehicleF_Physics vehicle) {
        if (!usesAuthoredModifierOffset(vehicle)) return;
        double requested = requested(vehicle);
        double applied = applied(vehicle);
        double held = heldOffset(requested, applied);
        if (Double.isFinite(held)) setRequested(vehicle, held, !vehicle.world.isClient());
    }

    public static double heldOffset(double requestedOffset, double appliedOffset) {
        return Double.isFinite(appliedOffset) ? appliedOffset : requestedOffset;
    }

    public static double minimumOffset(double baseline, double limit) {
        return Math.min(0.0, -Math.abs(limit) - baseline);
    }

    public static double maximumOffset(double baseline, double limit) {
        return Math.max(0.0, Math.abs(limit) - baseline);
    }

    public static Composition compose(double baseline, double requestedOffset, double limit) {
        double trimLimit = Math.abs(limit);
        if (!Double.isFinite(baseline) || !Double.isFinite(requestedOffset) || !Double.isFinite(trimLimit)) {
            return new Composition(baseline, requestedOffset, Double.NaN, Double.NaN);
        }
        double applied = clamp(requestedOffset, minimumOffset(baseline, trimLimit),
            maximumOffset(baseline, trimLimit));
        return new Composition(baseline, requestedOffset, applied, baseline + applied);
    }

    public static boolean atOffsetLimit(double offset, double baseline, double trimLimit) {
        double minimum = minimumOffset(baseline, trimLimit);
        double maximum = maximumOffset(baseline, trimLimit);
        return Double.isFinite(offset) && (Math.abs(offset - minimum) <= 1.0E-5
            || Math.abs(offset - maximum) <= 1.0E-5);
    }

    /** Called immediately before IV evaluates authored variable modifiers. */
    public static boolean beforeModifierPass(EntityVehicleF_Physics vehicle) {
        AircraftState state = ((AircraftStateAccess) vehicle).pmweatherIv$getAircraftState();
        if (!usesAuthoredModifierOffset(vehicle)) {
            leaveOverlayIfPresent(vehicle, state);
            return false;
        }
        double effective = vehicle.elevatorTrimVar.currentValue;
        double previousApplied = applied(vehicle);
        double requested = requested(vehicle);
        state.autoTrimOverlayPassActive = false;
        if (!Double.isFinite(effective) || !Double.isFinite(previousApplied) || !Double.isFinite(requested)) {
            state.autoTrimOverlayBaseline = Double.NaN;
            return false;
        }
        if (!state.autoTrimOverlayInitialized) {
            // IV already restored this effective trim from the vehicle's variables list.
            state.autoTrimOverlayExpectedEffective = effective;
            state.autoTrimOverlayInitialized = true;
        }
        double externalDelta = effective - state.autoTrimOverlayExpectedEffective;
        if (Double.isFinite(externalDelta) && Math.abs(externalDelta) > 0.025) {
            state.autoTrimOverlayManualTrimChanged = true;
            state.autoTrimOverlayManualTrimDelta = externalDelta;
        }
        double baseline = effective - previousApplied;
        if (!Double.isFinite(baseline)) {
            state.autoTrimOverlayBaseline = Double.NaN;
            return false;
        }
        state.autoTrimOverlayBaseline = baseline;
        state.autoTrimOverlayRequestedOffset = requested;
        state.autoTrimOverlayAppliedOffset = previousApplied;
        vehicle.elevatorTrimVar.setTo(baseline, false);
        state.autoTrimOverlayPassActive = true;
        return true;
    }

    /** Called after IV has completed its unchanged native modifier pass. */
    public static void afterModifierPass(EntityVehicleF_Physics vehicle) {
        AircraftState state = ((AircraftStateAccess) vehicle).pmweatherIv$getAircraftState();
        if (!state.autoTrimOverlayPassActive) return;
        state.autoTrimOverlayPassActive = false;
        double baseline = vehicle.elevatorTrimVar.currentValue;
        double requested = requested(vehicle);
        if (!Double.isFinite(baseline) || !Double.isFinite(requested)) {
            state.autoTrimOverlayBaseline = Double.NaN;
            return;
        }
        Composition composition = compose(baseline, requested, EntityVehicleF_Physics.MAX_ELEVATOR_TRIM);
        if (!Double.isFinite(composition.effectiveTrim()) || !Double.isFinite(composition.appliedOffset())) {
            state.autoTrimOverlayBaseline = Double.NaN;
            return;
        }
        vehicle.elevatorTrimVar.setTo(composition.effectiveTrim(), false);
        write(vehicle, APPLIED_VARIABLE, composition.appliedOffset(), false);
        if (Math.abs(composition.appliedOffset() - requested) > 1.0E-6) {
            // Saturation changes the persistent target to the offset that was
            // actually achieved, preventing a stale request from snapping back
            // if the authored baseline later moves inside the trim range.
            setRequested(vehicle, composition.appliedOffset(), !vehicle.world.isClient());
            requested = composition.appliedOffset();
        }
        state.autoTrimOverlayBaseline = baseline;
        state.autoTrimOverlayRequestedOffset = requested;
        state.autoTrimOverlayAppliedOffset = composition.appliedOffset();
        state.autoTrimOverlayEffectiveTrim = composition.effectiveTrim();
        state.autoTrimOverlayExpectedEffective = composition.effectiveTrim();
        state.autoTrimOverlayInitialized = true;
    }

    public record Composition(double baseline, double requestedOffset,
                              double appliedOffset, double effectiveTrim) {}

    private static boolean hasElevatorTrimModifier(EntityVehicleF_Physics vehicle) {
        if (vehicle == null || vehicle.definition == null) return false;
        List<JSONVariableModifier> modifiers = vehicle.definition.variableModifiers;
        if (modifiers == null) return false;
        for (JSONVariableModifier modifier : modifiers) {
            if (modifier != null && "trim_elevator".equals(modifier.variable)) return true;
        }
        return false;
    }

    private static boolean hasRotorPart(EntityVehicleF_Physics vehicle) {
        if (vehicle.allParts == null) return false;
        for (APart part : vehicle.allParts) {
            if (part instanceof PartPropeller propeller && propeller.definition.propeller != null
                && propeller.definition.propeller.isRotor) return true;
        }
        return false;
    }

    private static void leaveOverlayIfPresent(EntityVehicleF_Physics vehicle, AircraftState state) {
        state.autoTrimOverlayPassActive = false;
        if (vehicle == null || !vehicle.containsVariable(APPLIED_VARIABLE)) return;
        double applied = applied(vehicle);
        double effective = vehicle.elevatorTrimVar.currentValue;
        if (!Double.isFinite(applied) || !Double.isFinite(effective)) return;
        if (Math.abs(applied) > 1.0E-6) vehicle.elevatorTrimVar.setTo(effective - applied, false);
        if (Math.abs(applied) > 1.0E-6) write(vehicle, APPLIED_VARIABLE, 0.0, false);
        if (vehicle.containsVariable(REQUESTED_VARIABLE)) {
            double requested = requested(vehicle);
            if (Math.abs(requested) > 1.0E-6) setRequested(vehicle, 0.0, !vehicle.world.isClient());
        }
        state.autoTrimOverlayExpectedEffective = vehicle.elevatorTrimVar.currentValue;
        state.autoTrimOverlayEffectiveTrim = vehicle.elevatorTrimVar.currentValue;
        state.autoTrimOverlayBaseline = vehicle.elevatorTrimVar.currentValue;
        state.autoTrimOverlayRequestedOffset = 0.0;
        state.autoTrimOverlayAppliedOffset = 0.0;
        state.autoTrimOverlayManualTrimChanged = false;
        state.autoTrimOverlayManualTrimDelta = 0.0;
    }

    private static void attachVariable(EntityVehicleF_Physics vehicle, String key, IWrapperNBT savedData) {
        if (!vehicle.containsVariable(key)) vehicle.addVariable(new ComputedVariable(vehicle, key, savedData));
    }

    private static double read(EntityVehicleF_Physics vehicle, String key) {
        if (vehicle == null) return 0.0;
        ComputedVariable variable = vehicle.getOrCreateVariable(key);
        return key.equals(variable.variableKey) && variable.entity == vehicle
            ? variable.currentValue : 0.0;
    }

    private static void write(EntityVehicleF_Physics vehicle, String key, double value, boolean sync) {
        if (vehicle == null || !Double.isFinite(value)) return;
        ComputedVariable variable = vehicle.getOrCreateVariable(key);
        if (!key.equals(variable.variableKey) || variable.entity != vehicle) {
            throw new IllegalStateException("IV auto-trim offset variable was not attached: " + key);
        }
        variable.setTo(value, sync);
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}
