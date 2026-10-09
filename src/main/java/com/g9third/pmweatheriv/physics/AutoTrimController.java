package com.g9third.pmweatheriv.physics;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

/**
 * Small owner-tick-only pitch trim controller. Its trim response is measured
 * from the live aerodynamic solve before the controller writes the next native
 * trim value; it never predicts or mutates a physics solve.
 */
public final class AutoTrimController {
    public static final double MAX_STEP_PER_TICK = 0.1;
    private static final double PROBE_DISTANCE = 0.4;
    private static final int BASELINE_TICKS = 12;
    private static final int PROBE_HOLD_TICKS = 30;
    private static final int MAX_ATTEMPTS_PER_SESSION = 4;
    private static final int RETRY_COOLDOWN_TICKS = 200;
    private static final int RETRY_STABLE_TICKS = 40;
    private static final double TARGET_FLIGHT_PATH_DEGREES = 0.5;

    public enum State { OFF, LEARNING, ON, PAUSED, LIMITED }

    public record Output(double trim, State state, String reason, boolean enabled) {}

    /** All values are physical owner-tick observations; no IV or Sable object is retained. */
    public record Input(
        long tick,
        double trim,
        double trimLimit,
        double trueAirspeed,
        double forwardAirspeed,
        double dynamicPressure,
        double wingArea,
        double maximumLiftCoefficient,
        double maxMainWingSeparation,
        double angleOfAttackDegrees,
        double bankDegrees,
        double pitchRateRadiansPerSecond,
        double flightPathDegrees,
        double pitchAngularAcceleration,
        double elevatorInput,
        double flapAngle,
        double mass,
        double pitchInertia,
        int damageSignature,
        boolean fixedWing,
        boolean healthy,
        boolean heldForPhysics,
        boolean terrainContact,
        boolean existingAutopilot,
        boolean finite
    ) {}

    private enum LearningPhase { BASE_BEFORE, RAMP_TO_PROBE, PROBE_HOLD, RAMP_TO_BASE, BASE_AFTER }

    private boolean enabled;
    private UUID pilot;
    private State state = State.OFF;
    private String reason = "OFF";
    private double expectedTrim = Double.NaN;
    private double trimSlopePerPressure;
    private boolean hasTrimSlope;
    private double learnedSpeed;
    private double learnedAoA;
    private double learnedFlaps;
    private double learnedMass;
    private double learnedInertia;
    private double learnedMaximumLiftCoefficient;
    private int learnedDamageSignature;
    private long nextDamageSignatureCheck;
    private int configurationDriftTicks;
    private int stableTicks;
    private int manualNeutralTicks;
    private int learningAttempts;
    private int consistentMeasurements;
    private double candidateSlope;
    private int candidateLagTicks;
    private LearningPhase learningPhase;
    private List<TrimResponseRegression.Sample> attemptSamples;
    private long attemptStartTick;
    private double learningBaseTrim;
    private double probeTargetTrim;
    private int baselineTicks;
    private int probeHoldTicks;
    private boolean attemptConfigurationSet;
    private boolean attemptConfigurationStable;
    private double attemptFlaps;
    private double attemptMass;
    private double attemptInertia;
    private double attemptMaximumLiftCoefficient;
    private int attemptDamageSignature;
    private double minimumAttemptSpeed = Double.POSITIVE_INFINITY;
    private double maximumAttemptSpeed = Double.NEGATIVE_INFINITY;
    private double nextProbeDirection = 1.0;
    private boolean probeTrimOwned;
    private boolean restorePending;
    private boolean disableAfterRestore;
    private boolean learningInterrupted;
    private double restoreTargetTrim = Double.NaN;
    private boolean retryBudgetExhausted;
    private long retryAfterTick = Long.MIN_VALUE;
    private int retryStableTicks;
    private String lastFitDiagnostic;

    public boolean enabled() { return enabled; }
    public UUID pilot() { return pilot; }
    public State state() { return state; }
    public String reason() { return reason; }
    public double expectedTrim() { return expectedTrim; }

    /** A one-shot compact fit record, consumed only by the optional developer logger. */
    public String consumeFitDiagnostic() {
        String diagnostic = lastFitDiagnostic;
        lastFitDiagnostic = null;
        return diagnostic;
    }

    public void toggle(UUID controllingPilot, double currentTrim) {
        if (enabled) {
            if (finite(expectedTrim) && finite(currentTrim)
                && Math.abs(currentTrim - expectedTrim) > 0.025) {
                disable("MANUAL_TRIM");
            } else if (probeTrimOwned && finite(learningBaseTrim)
                && Math.abs(currentTrim - learningBaseTrim) > 0.001) {
                restoreTargetTrim = learningBaseTrim;
                restorePending = true;
                disableAfterRestore = true;
                learningInterrupted = false;
                clearAttempt();
                state = State.PAUSED;
                reason = "RESTORING_TRIM";
                stableTicks = 0;
                manualNeutralTicks = 0;
            } else {
                disable("OFF");
            }
            return;
        }
        enabled = true;
        pilot = controllingPilot;
        expectedTrim = currentTrim;
        hasTrimSlope = false;
        disableAfterRestore = false;
        restorePending = false;
        probeTrimOwned = false;
        retryBudgetExhausted = false;
        retryAfterTick = Long.MIN_VALUE;
        startFreshLearning(currentTrim, Long.MIN_VALUE, "LEARNING");
    }

    public void disable(String why) {
        enabled = false;
        pilot = null;
        hasTrimSlope = false;
        expectedTrim = Double.NaN;
        state = State.OFF;
        reason = why == null ? "OFF" : why;
        learningPhase = null;
        clearAttempt();
        probeTrimOwned = false;
        restorePending = false;
        disableAfterRestore = false;
        learningInterrupted = false;
        restoreTargetTrim = Double.NaN;
        retryBudgetExhausted = false;
        retryAfterTick = Long.MIN_VALUE;
        retryStableTicks = 0;
        learningAttempts = 0;
        consistentMeasurements = 0;
        candidateSlope = 0.0;
        candidateLagTicks = 0;
        stableTicks = 0;
        manualNeutralTicks = 0;
    }

    /**
     * Advances once after the ordinary solve. Returned trim is applied by the
     * caller through native ComputedVariable.setTo(value, true).
     */
    public Output update(Input in) {
        if (in == null || !enabled) return output(in == null ? 0.0 : in.trim());
        double currentTrim = finite(in.trim()) ? clamp(in.trim(), -in.trimLimit(), in.trimLimit()) : 0.0;

        if (finite(expectedTrim) && Math.abs(currentTrim - expectedTrim) > 0.025) {
            // A player or another native system moved elevator trim. Never chase it.
            disable("MANUAL_TRIM");
            return output(currentTrim);
        }
        if (disableAfterRestore) {
            if (!safeForToggleOffRestore(in)) {
                // OFF is final when the same pilot or a safe Sable owner tick is lost.
                // Never carry the old pilot's cleanup into another rider's session.
                disable("OFF");
                return output(currentTrim);
            }
            return advanceRestore(currentTrim, true, in);
        }
        if (!in.finite() || !in.fixedWing() || !in.healthy() || in.heldForPhysics()
            || in.existingAutopilot()) {
            retryStableTicks = 0;
            pause(!in.finite() ? "UNAVAILABLE" : !in.fixedWing() ? "UNSUPPORTED"
                : !in.healthy() ? "AIRCRAFT" : in.heldForPhysics() ? "PHYSICS_HOLD" : "AUTOPILOT");
            return output(currentTrim);
        }
        if (Math.abs(in.elevatorInput()) > 0.025) {
            manualNeutralTicks = 0;
            retryStableTicks = 0;
            pause("PILOT_INPUT");
            return output(currentTrim);
        }
        if (state == State.PAUSED && "PILOT_INPUT".equals(reason)) {
            if (++manualNeutralTicks < 12) return output(currentTrim);
        }

        String ineligible = in.terrainContact() ? "CONTACT"
            : in.trueAirspeed() < 1.5 || in.forwardAirspeed() < 1.5 || in.dynamicPressure() < 0.75 ? "LOW_SPEED"
            // Retain 25% headroom against the solver's live flap-adjusted lift ceiling.
            : in.maxMainWingSeparation() > 0.55
                || in.mass() * 9.81 / Math.max(in.dynamicPressure() * in.wingArea(), 1.0E-6)
                    > 0.75 * in.maximumLiftCoefficient()
                    ? "STALL"
            : Math.abs(in.angleOfAttackDegrees()) > 18.0 ? "HIGH_AOA"
            : Math.abs(in.bankDegrees()) > 25.0 ? "BANK"
            : Math.abs(in.pitchRateRadiansPerSecond()) > Math.toRadians(25.0) ? "HIGH_RATE"
            : null;
        if (ineligible != null) {
            retryStableTicks = 0;
            pause(ineligible);
            return output(currentTrim);
        }

        if (learningPhase != null && !attemptConfigurationMatches(in)) {
            pause("UNAVAILABLE");
            return output(currentTrim);
        }

        if (retryBudgetExhausted) {
            if (state == State.PAUSED) {
                if (++stableTicks < 8) return output(currentTrim);
                stableTicks = 0;
            }
            ++retryStableTicks;
            state = State.LIMITED;
            reason = "RESPONSE_UNAVAILABLE";
            if (in.tick() < retryAfterTick || retryStableTicks < RETRY_STABLE_TICKS) {
                return output(currentTrim);
            }
            retryBudgetExhausted = false;
            retryAfterTick = Long.MIN_VALUE;
            retryStableTicks = 0;
            startFreshLearning(currentTrim, in.tick(), "LEARNING");
            return output(currentTrim);
        }

        if (state == State.PAUSED) {
            if (stableTicks < 8 && ++stableTicks < 8) return output(currentTrim);
            if (restorePending) return advanceRestore(currentTrim, false, in);
            stableTicks = 0;
            manualNeutralTicks = 0;
            if (learningInterrupted) {
                startFreshLearning(currentTrim, in.tick(), "LEARNING");
            } else if (hasTrimSlope) {
                state = State.ON;
                reason = "ON";
            } else {
                startFreshLearning(currentTrim, in.tick(), "LEARNING");
            }
            return output(currentTrim);
        }

        if (configurationChanged(in)) {
            hasTrimSlope = false;
            startFreshLearning(currentTrim, in.tick(), "LEARNING");
            return output(currentTrim);
        }
        if (!hasTrimSlope || learningPhase != null) return advanceLearning(in, currentTrim);

        state = State.ON;
        reason = "ON";
        double gammaError = Math.toRadians(TARGET_FLIGHT_PATH_DEGREES - in.flightPathDegrees());
        double desiredPitchRate = clamp(gammaError * 0.35, -Math.toRadians(2.0), Math.toRadians(2.0));
        double desiredAngularAcceleration = clamp(
            (desiredPitchRate - in.pitchRateRadiansPerSecond()) * 0.8,
            -Math.toRadians(2.5), Math.toRadians(2.5)
        );
        double denominator = trimSlopePerPressure * in.dynamicPressure();
        if (!finite(denominator) || Math.abs(denominator) < 1.0E-7) {
            hasTrimSlope = false;
            state = State.LIMITED;
            reason = "NO_AUTHORITY";
            scheduleLearningRetry(in.tick());
            return output(currentTrim);
        }
        double trimDemand = (desiredAngularAcceleration - in.pitchAngularAcceleration()) / denominator;
        if (!finite(trimDemand)) {
            hasTrimSlope = false;
            state = State.LIMITED;
            reason = "NO_AUTHORITY";
            scheduleLearningRetry(in.tick());
            return output(currentTrim);
        }
        double desiredTrim = clamp(currentTrim + trimDemand, -in.trimLimit(), in.trimLimit());
        if (Math.abs(desiredTrim - currentTrim) < 1.0E-5
            && Math.abs(trimDemand) > 0.025
            && Math.signum(trimDemand) == Math.signum(currentTrim)) {
            state = State.LIMITED;
            reason = "TRIM_LIMIT";
            return output(currentTrim);
        }
        state = State.ON;
        reason = "ON";
        double delta = clamp(desiredTrim - currentTrim, -MAX_STEP_PER_TICK, MAX_STEP_PER_TICK);
        return output(clamp(currentTrim + delta, -in.trimLimit(), in.trimLimit()));
    }

    private Output advanceLearning(Input in, double currentTrim) {
        if (learningPhase == null) startFreshLearning(currentTrim, in.tick(), "LEARNING");
        state = State.LEARNING;
        reason = "LEARNING";
        switch (learningPhase) {
            case BASE_BEFORE -> {
                recordAttemptSample(in);
                if (++baselineTicks >= BASELINE_TICKS) {
                    learningBaseTrim = currentTrim;
                    double positiveRoom = in.trimLimit() - currentTrim;
                    double negativeRoom = currentTrim + in.trimLimit();
                    double direction = nextProbeDirection;
                    if (direction > 0.0 && positiveRoom < PROBE_DISTANCE) direction = -1.0;
                    else if (direction < 0.0 && negativeRoom < PROBE_DISTANCE) direction = 1.0;
                    if (Math.max(positiveRoom, negativeRoom) < PROBE_DISTANCE) {
                        learningAttempts = MAX_ATTEMPTS_PER_SESSION;
                        scheduleLearningRetry(in.tick());
                        return output(currentTrim);
                    }
                    nextProbeDirection = -direction;
                    probeTargetTrim = clamp(currentTrim + direction * PROBE_DISTANCE,
                        -in.trimLimit(), in.trimLimit());
                    learningPhase = LearningPhase.RAMP_TO_PROBE;
                }
                return output(currentTrim);
            }
            case RAMP_TO_PROBE -> {
                recordAttemptSample(in);
                double next = stepToward(currentTrim, probeTargetTrim);
                if (Math.abs(next - currentTrim) > 1.0E-6) {
                    probeTrimOwned = true;
                    restoreTargetTrim = learningBaseTrim;
                }
                if (Math.abs(probeTargetTrim - next) < 0.001) {
                    next = probeTargetTrim;
                    learningPhase = LearningPhase.PROBE_HOLD;
                    probeHoldTicks = 0;
                }
                return output(next);
            }
            case PROBE_HOLD -> {
                recordAttemptSample(in);
                if (++probeHoldTicks >= PROBE_HOLD_TICKS) learningPhase = LearningPhase.RAMP_TO_BASE;
                return output(currentTrim);
            }
            case RAMP_TO_BASE -> {
                recordAttemptSample(in);
                double next = stepToward(currentTrim, learningBaseTrim);
                if (Math.abs(learningBaseTrim - next) < 0.001) {
                    next = learningBaseTrim;
                    learningPhase = LearningPhase.BASE_AFTER;
                    baselineTicks = 0;
                    probeTrimOwned = false;
                    restorePending = false;
                    restoreTargetTrim = Double.NaN;
                }
                return output(next);
            }
            case BASE_AFTER -> {
                recordAttemptSample(in);
                if (++baselineTicks >= BASELINE_TICKS) finishLearning(in, currentTrim);
                return output(currentTrim);
            }
        }
        return output(currentTrim);
    }

    private void finishLearning(Input in, double currentTrim) {
        TrimResponseRegression.Fit fit = TrimResponseRegression.fit(attemptSamples);
        if (!attemptConfigurationStable) {
            fit = new TrimResponseRegression.Fit(false, "CONFIGURATION_CHANGED", fit.samples(),
                fit.rank(), fit.conditionRatio(), fit.trimExcitation(), fit.slope(),
                fit.slopeStandardError(), fit.rSquared(), fit.lagTicks());
        }
        ++learningAttempts;
        lastFitDiagnostic = fit.diagnostic() + " attempt=" + learningAttempts;
        if (fit.accepted()) {
            double slope = fit.slope();
            boolean consistent = consistentMeasurements == 0
                || Math.signum(slope) == Math.signum(candidateSlope)
                    && Math.abs(slope) >= Math.abs(candidateSlope) * 0.4
                    && Math.abs(slope) <= Math.abs(candidateSlope) * 2.5
                    && Math.abs(fit.lagTicks() - candidateLagTicks) <= 4;
            if (consistent) {
                candidateSlope = consistentMeasurements == 0 ? slope
                    : (candidateSlope * consistentMeasurements + slope) / (consistentMeasurements + 1);
                candidateLagTicks = consistentMeasurements == 0 ? fit.lagTicks()
                    : (candidateLagTicks * consistentMeasurements + fit.lagTicks()) / (consistentMeasurements + 1);
                ++consistentMeasurements;
            } else {
                candidateSlope = slope;
                candidateLagTicks = fit.lagTicks();
                consistentMeasurements = 1;
            }
            if (consistentMeasurements >= 2) {
                trimSlopePerPressure = candidateSlope;
                hasTrimSlope = true;
                learnedSpeed = in.trueAirspeed();
                learnedAoA = in.angleOfAttackDegrees();
                learnedFlaps = in.flapAngle();
                learnedMass = in.mass();
                learnedInertia = in.pitchInertia();
                learnedMaximumLiftCoefficient = in.maximumLiftCoefficient();
                learnedDamageSignature = in.damageSignature();
                nextDamageSignatureCheck = in.tick() + 20;
                configurationDriftTicks = 0;
                retryBudgetExhausted = false;
                learningInterrupted = false;
                attemptSamples = null;
                learningPhase = null;
                state = State.ON;
                reason = "ON";
                return;
            }
        }
        if (learningAttempts < MAX_ATTEMPTS_PER_SESSION) {
            beginAttempt(currentTrim, in.tick());
        } else {
            scheduleLearningRetry(in.tick());
        }
    }

    private boolean configurationChanged(Input in) {
        if (hasTrimSlope && in.tick() >= nextDamageSignatureCheck) {
            nextDamageSignatureCheck = in.tick() + 20;
            if (in.damageSignature() != learnedDamageSignature) return true;
        }
        if (!hasTrimSlope) return false;
        boolean drift = Math.abs(in.flapAngle() - learnedFlaps) > 2.0
            || relativeChange(in.mass(), learnedMass) > 0.10
            || relativeChange(in.pitchInertia(), learnedInertia) > 0.10
            || relativeChange(in.maximumLiftCoefficient(), learnedMaximumLiftCoefficient) > 0.05
            || relativeChange(in.trueAirspeed(), learnedSpeed) > 0.45
            || Math.abs(in.angleOfAttackDegrees() - learnedAoA) > 7.0;
        configurationDriftTicks = drift ? configurationDriftTicks + 1 : 0;
        if (configurationDriftTicks >= 8) return true;
        return false;
    }

    private void startFreshLearning(double trim, long tick, String why) {
        hasTrimSlope = false;
        learningAttempts = 0;
        consistentMeasurements = 0;
        candidateSlope = 0.0;
        candidateLagTicks = 0;
        retryBudgetExhausted = false;
        retryAfterTick = Long.MIN_VALUE;
        retryStableTicks = 0;
        learningInterrupted = false;
        disableAfterRestore = false;
        restorePending = false;
        probeTrimOwned = false;
        restoreTargetTrim = Double.NaN;
        stableTicks = 0;
        manualNeutralTicks = 0;
        beginAttempt(trim, tick);
        state = State.LEARNING;
        reason = why;
    }

    private void beginAttempt(double trim, long tick) {
        learningPhase = LearningPhase.BASE_BEFORE;
        attemptSamples = new ArrayList<>(64);
        attemptStartTick = tick;
        learningBaseTrim = trim;
        baselineTicks = 0;
        probeHoldTicks = 0;
        attemptConfigurationSet = false;
        attemptConfigurationStable = true;
        attemptFlaps = Double.NaN;
        attemptMass = Double.NaN;
        attemptInertia = Double.NaN;
        attemptMaximumLiftCoefficient = Double.NaN;
        attemptDamageSignature = 0;
        minimumAttemptSpeed = Double.POSITIVE_INFINITY;
        maximumAttemptSpeed = Double.NEGATIVE_INFINITY;
    }

    private void clearAttempt() {
        learningPhase = null;
        attemptSamples = null;
        attemptStartTick = Long.MIN_VALUE;
        baselineTicks = 0;
        probeHoldTicks = 0;
        attemptConfigurationSet = false;
        attemptConfigurationStable = true;
        minimumAttemptSpeed = Double.POSITIVE_INFINITY;
        maximumAttemptSpeed = Double.NEGATIVE_INFINITY;
    }

    private void recordAttemptSample(Input in) {
        if (attemptSamples == null) beginAttempt(in.trim(), in.tick());
        if (attemptStartTick == Long.MIN_VALUE) attemptStartTick = in.tick();
        if (!attemptConfigurationSet) {
            attemptFlaps = in.flapAngle();
            attemptMass = in.mass();
            attemptInertia = in.pitchInertia();
            attemptMaximumLiftCoefficient = in.maximumLiftCoefficient();
            attemptDamageSignature = in.damageSignature();
            attemptConfigurationSet = true;
        } else if (!configurationMatches(in)) {
            attemptConfigurationStable = false;
        }
        minimumAttemptSpeed = Math.min(minimumAttemptSpeed, in.trueAirspeed());
        maximumAttemptSpeed = Math.max(maximumAttemptSpeed, in.trueAirspeed());
        if (maximumAttemptSpeed / Math.max(1.0, minimumAttemptSpeed) > 1.30) {
            attemptConfigurationStable = false;
        }
        attemptSamples.add(new TrimResponseRegression.Sample(
            in.pitchAngularAcceleration() / in.dynamicPressure(),
            in.trim(), in.angleOfAttackDegrees(),
            in.pitchRateRadiansPerSecond() / in.trueAirspeed(),
            in.tick() - attemptStartTick
        ));
    }

    private boolean attemptConfigurationMatches(Input in) {
        return !attemptConfigurationSet || configurationMatches(in);
    }

    private boolean configurationMatches(Input in) {
        return attemptDamageSignature == in.damageSignature()
            && Math.abs(attemptFlaps - in.flapAngle()) <= 0.5
            && relativeChange(in.mass(), attemptMass) <= 0.05
            && relativeChange(in.pitchInertia(), attemptInertia) <= 0.05
            && relativeChange(in.maximumLiftCoefficient(), attemptMaximumLiftCoefficient) <= 0.05;
    }

    private void scheduleLearningRetry(long tick) {
        retryBudgetExhausted = true;
        retryAfterTick = tick + RETRY_COOLDOWN_TICKS;
        retryStableTicks = 0;
        learningInterrupted = false;
        restorePending = false;
        probeTrimOwned = false;
        restoreTargetTrim = Double.NaN;
        disableAfterRestore = false;
        clearAttempt();
        state = State.LIMITED;
        reason = "RESPONSE_UNAVAILABLE";
    }

    private void pause(String why) {
        if (learningPhase != null) {
            learningInterrupted = true;
            if (probeTrimOwned && finite(learningBaseTrim)) {
                restoreTargetTrim = learningBaseTrim;
                restorePending = true;
            }
            clearAttempt();
        }
        state = State.PAUSED;
        reason = why;
        stableTicks = 0;
    }

    private boolean safeForToggleOffRestore(Input in) {
        return in != null && in.finite() && in.fixedWing() && in.healthy()
            && !in.heldForPhysics() && !in.existingAutopilot()
            && Math.abs(in.elevatorInput()) <= 0.025;
    }

    private Output advanceRestore(double currentTrim, boolean turnOff, Input in) {
        if (!restorePending || !finite(restoreTargetTrim)) {
            if (turnOff) disable("OFF");
            else if (learningInterrupted) startFreshLearning(currentTrim, in.tick(), "LEARNING");
            return output(currentTrim);
        }
        double target = clamp(restoreTargetTrim, -in.trimLimit(), in.trimLimit());
        if (Math.abs(currentTrim - target) <= 0.001) {
            restorePending = false;
            probeTrimOwned = false;
            restoreTargetTrim = Double.NaN;
            if (turnOff) {
                disable("OFF");
            } else {
                startFreshLearning(currentTrim, in.tick(), "LEARNING");
            }
            return output(currentTrim);
        }
        state = State.PAUSED;
        reason = "RESTORING_TRIM";
        return output(stepToward(currentTrim, target));
    }

    private Output output(double trim) {
        expectedTrim = enabled ? trim : Double.NaN;
        return new Output(trim, state, reason, enabled);
    }

    private static double stepToward(double from, double to) {
        return from + clamp(to - from, -MAX_STEP_PER_TICK, MAX_STEP_PER_TICK);
    }

    private static double relativeChange(double value, double reference) {
        return Math.abs(value - reference) / Math.max(1.0, Math.abs(reference));
    }

    private static boolean finite(double value) { return Double.isFinite(value); }
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

}
