package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.network.AutoTrimNetwork;
import io.netty.buffer.Unpooled;
import net.minecraft.core.RegistryAccess;
import net.minecraft.network.RegistryFriendlyByteBuf;
import net.minecraft.resources.ResourceLocation;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

/** Covers explicit per-aircraft opt-in, pause safety and trim controller lifecycle. */
public final class AutoTrimRegression {
    private static int assertions;

    private static void require(boolean condition, String message) {
        ++assertions;
        if (!condition) throw new AssertionError(message);
    }

    private static AutoTrimController.Input input(long tick, double trim, boolean terrainContact) {
        return new AutoTrimController.Input(
            tick, trim, 10.0, 50.0, 50.0, 900.0, 30.0, 1.6,
            0.0, 3.0, 0.0, 0.0, 0.5, 0.0,
            0.0, 0.0, 1200.0, 15000.0, 0,
            true, true, false, terrainContact, false, true
        );
    }

    private static AutoTrimController.Input overlayInput(long tick, double offset,
            double minimumOffset, double maximumOffset, boolean manualTrimChanged) {
        return new AutoTrimController.Input(
            tick, offset, 10.0, 50.0, 50.0, 900.0, 30.0, 1.6,
            0.0, 3.0, 0.0, 0.0, 0.5, 0.0,
            0.0, 0.0, 1200.0, 15000.0, 0,
            true, true, false, false, false, true,
            minimumOffset, maximumOffset, true, manualTrimChanged
        );
    }

    private static void perAircraftOptInPolicy() {
        require(!AutoTrimPreferences.enabledValue(0.0),
            "a new plane or legacy save with no opt-in defaults off");
        require(!AutoTrimPreferences.enabledValue(Double.NaN),
            "a malformed saved opt-in remains off");
        require(AutoTrimPreferences.enabledValue(1.0),
            "an explicitly saved per-plane opt-in is retained");
        require(!AutoTrimPreferences.shouldAutoStart(false, true, true),
            "a new plane cannot auto-start before an explicit toggle");
        require(AutoTrimPreferences.shouldAutoStart(true, true, true),
            "an explicitly opted-in fixed-wing aircraft starts with its pilot");
        require(!AutoTrimPreferences.shouldAutoStart(true, false, true),
            "unsupported aircraft cannot auto-start");
        require(!AutoTrimPreferences.shouldAutoStart(true, true, false),
            "auto-start requires the current controller pilot");
    }

    private static void groundPauseAndReenable() {
        AutoTrimController controller = new AutoTrimController();
        UUID pilot = new UUID(0x504D4956L, 1L);
        controller.enable(pilot, 0.0);

        AutoTrimController.Output ground = controller.update(input(1L, 0.0, true));
        require(ground.enabled(), "ground contact pauses rather than disabling auto trim");
        require(ground.state() == AutoTrimController.State.PAUSED
            && "CONTACT".equals(ground.reason()), "ground contact reports a paused state");
        require(ground.trim() == 0.0 && !ground.adjusting(), "controller writes no trim while grounded");

        AutoTrimController.Output clear = null;
        for (long tick = 2L; tick < 20L; ++tick) {
            clear = controller.update(input(tick, 0.0, false));
            require(clear.trim() == 0.0 || Math.abs(clear.trim()) <= AutoTrimController.MAX_STEP_PER_TICK,
                "recovery from contact stays within one native trim increment");
        }
        require(clear != null && controller.enabled(), "clearing contact resumes the same pilot session");

        // Turning off the per-aircraft preference stops further writes immediately.
        controller.toggle(pilot, clear.trim());
        AutoTrimController.Output off = controller.update(input(20L, clear.trim(), false));
        require(!off.enabled() && off.state() == AutoTrimController.State.OFF,
            "explicit toggle fully stops the controller");
        require(off.trim() == clear.trim(), "off state does not write a probe cleanup trim");
        require(AutoTrimOffset.heldOffset(0.3, off.trim()) == off.trim(),
            "turning the preference off discards a queued request without restoring an earlier probe baseline");
        require(!AutoTrimPreferences.shouldAutoStart(false, true, true),
            "the saved off choice prevents automatic restart for the same pilot");

        // Re-entry is possible only through an explicit toggle that opts in again.
        controller.toggle(pilot, off.trim());
        AutoTrimController.Output on = controller.update(input(21L, off.trim(), false));
        require(on.enabled() && on.state() == AutoTrimController.State.LEARNING,
            "explicit re-enable starts a fresh learning session");
        require(AutoTrimPreferences.shouldAutoStart(true, true, true),
            "explicit re-enable permits a later auto-start");
    }

    private static void statusCodec() {
        AutoTrimNetwork.StatusPayload original = new AutoTrimNetwork.StatusPayload(
            new UUID(0x504D4956L, 2L), ResourceLocation.fromNamespaceAndPath("minecraft", "overworld"),
            456L, -1.25, (byte) AutoTrimController.State.LEARNING.ordinal(),
            (byte) AutoTrimNetwork.StatusReason.LEARNING.ordinal(), (byte) 3
        );
        RegistryFriendlyByteBuf buffer = new RegistryFriendlyByteBuf(Unpooled.buffer(), RegistryAccess.EMPTY);
        try {
            AutoTrimNetwork.StatusPayload.CODEC.encode(buffer, original);
            AutoTrimNetwork.StatusPayload decoded = AutoTrimNetwork.StatusPayload.CODEC.decode(buffer);
            require(decoded.equals(original), "status codec preserves trim state and appended flags");
            require(buffer.readableBytes() == 0, "status codec consumes the complete packet");
            require(decoded.finite() && decoded.adjusting() && decoded.firstEngagementNotice(),
                "valid status flags decode both indicators");
            AutoTrimNetwork.StatusPayload invalidFlags = new AutoTrimNetwork.StatusPayload(
                original.aircraft(), original.dimension(), original.tick(), original.trim(),
                original.state(), original.reason(), (byte) 4
            );
            require(!invalidFlags.finite(), "unknown status flag bits are rejected");
        } finally {
            buffer.release();
        }
    }

    private static void trimResponseFit() {
        List<TrimResponseRegression.Sample> samples = new ArrayList<>();
        for (int tick = 0; tick < 160; ++tick) {
            double trim = 0.20 * Math.sin(tick * 0.08);
            double response = 1.7 * trim + 0.005 * Math.sin(tick * 0.31);
            samples.add(new TrimResponseRegression.Sample(response, trim, 3.0, 0.0, tick));
        }
        TrimResponseRegression.Fit fit = TrimResponseRegression.fit(samples);
        require(fit.accepted(), "known trim response fits a stable synthetic probe");
        require(Math.abs(fit.slope() - 1.7) < 0.05,
            "fit recovers the known trim response slope");

        List<TrimResponseRegression.Sample> discontinuous = new ArrayList<>(samples);
        TrimResponseRegression.Sample sample = discontinuous.get(40);
        discontinuous.set(40, new TrimResponseRegression.Sample(sample.alphaPerPressure(), sample.trim(),
            sample.angleOfAttack(), sample.pitchRateOverSpeed(), sample.elapsedTicks() + 0.25));
        require("NONCONTIGUOUS_SAMPLES".equals(TrimResponseRegression.fit(discontinuous).rejection()),
            "fit rejects interrupted owner-tick sample sequences");
    }

    private static void authoredModifierOffsetComposition() {
        AutoTrimOffset.Composition first = AutoTrimOffset.compose(2.0, 0.3, 10.0);
        require(Math.abs(first.effectiveTrim() - 2.3) < 1.0E-9
            && Math.abs(first.appliedOffset() - 0.3) < 1.0E-9,
            "requested offset composes additively with the authored native baseline");

        // A native current-value modifier runs again from the prior native output.
        // Removing only PMIV's actual offset preserves that authored compounding.
        double strippedBaseline = first.effectiveTrim() - first.appliedOffset();
        double nextNativeBaseline = strippedBaseline + 0.04;
        AutoTrimOffset.Composition second = AutoTrimOffset.compose(nextNativeBaseline, 0.3, 10.0);
        require(Math.abs(second.effectiveTrim() - 2.34) < 1.0E-9,
            "repeated passes do not compound PMIV's offset into the native modifier input");

        AutoTrimOffset.Composition saturated = AutoTrimOffset.compose(9.95, 0.3, 10.0);
        require(Math.abs(saturated.effectiveTrim() - 10.0) < 1.0E-9
            && Math.abs(saturated.appliedOffset() - 0.05) < 1.0E-9,
            "saturation records the actually applied offset rather than the requested offset");
        double reloadedBaseline = saturated.effectiveTrim() - saturated.appliedOffset();
        AutoTrimOffset.Composition reloaded = AutoTrimOffset.compose(reloadedBaseline,
            saturated.appliedOffset(), 10.0);
        require(Math.abs(reloaded.effectiveTrim() - saturated.effectiveTrim()) < 1.0E-9,
            "saved effective trim and applied offset recover the baseline after saturation");

        AutoTrimOffset.Composition nativeOutside = AutoTrimOffset.compose(10.1, 0.0, 10.0);
        require(nativeOutside.effectiveTrim() == 10.1 && nativeOutside.appliedOffset() == 0.0,
            "a zero PMIV offset preserves an authored native trim outside IV's manual range");
        AutoTrimOffset.Composition clipped = AutoTrimOffset.compose(10.1, 0.2, 10.0);
        require(clipped.appliedOffset() == 0.0 && clipped.effectiveTrim() == 10.1
            && AutoTrimOffset.atOffsetLimit(clipped.appliedOffset(), 10.1, 10.0),
            "an outward offset is clamped at zero when the authored baseline is already out of range");
        AutoTrimOffset.Composition noSnapBack = AutoTrimOffset.compose(9.8,
            clipped.appliedOffset(), 10.0);
        require(Math.abs(noSnapBack.effectiveTrim() - 9.8) < 1.0E-9,
            "clamped actual offset remains the target when the native baseline moves back");

        require(Math.abs(AutoTrimOffset.minimumOffset(9.95, 10.0) - (-19.95)) < 1.0E-9
            && Math.abs(AutoTrimOffset.maximumOffset(9.95, 10.0) - 0.05) < 1.0E-9,
            "controller bounds are expressed in offset space around the live baseline");
        require(AutoTrimOffset.minimumOffset(10.1, 10.0) <= 0.0
            && AutoTrimOffset.maximumOffset(10.1, 10.0) == 0.0,
            "offset bounds always include zero and prevent outward trim beyond an authored baseline");
        require(Double.isNaN(AutoTrimOffset.compose(Double.NaN, 0.0, 10.0).effectiveTrim()),
            "a malformed native baseline is rejected without producing a PMIV trim value");
        require(AutoTrimOffset.heldOffset(0.3, 0.2) == 0.2,
            "stopping with a queued probe request holds the offset already applied");

        AutoTrimController controller = new AutoTrimController();
        UUID pilot = new UUID(0x504D4956L, 3L);
        controller.enable(pilot, 0.0);
        AutoTrimController.Output manual = controller.update(
            overlayInput(1L, 0.0, -10.0, 0.05, true));
        require(manual.state() == AutoTrimController.State.PAUSED
            && "MANUAL_TRIM".equals(manual.reason()) && manual.trim() == 0.0,
            "an external IV trim change pauses overlay learning without an offset write");

        AutoTrimController bounded = new AutoTrimController();
        bounded.enable(pilot, 0.0);
        for (long tick = 1L; tick <= 12L; ++tick) {
            bounded.update(overlayInput(tick, 0.0, -19.95, 0.05, false));
        }
        AutoTrimController.Output probe = bounded.update(
            overlayInput(13L, 0.0, -19.95, 0.05, false));
        require(Math.abs(probe.trim()) <= AutoTrimController.MAX_STEP_PER_TICK
            && probe.trim() >= -19.95 && probe.trim() <= 0.05,
            "overlay probes obey live asymmetric offset bounds and the native per-tick rate");

        AutoTrimController malformedBounds = new AutoTrimController();
        malformedBounds.enable(pilot, 0.0);
        AutoTrimController.Output safeFallback = malformedBounds.update(new AutoTrimController.Input(
            1L, 0.0, 10.0, 50.0, 50.0, 900.0, 30.0, 1.6,
            0.0, 3.0, 0.0, 0.0, 0.5, 0.0,
            0.0, 0.0, 1200.0, 15000.0, 0,
            true, true, false, false, false, true,
            Double.NaN, 0.05, true, false
        ));
        require(Double.isFinite(safeFallback.trim()) && safeFallback.trim() >= -10.0
            && safeFallback.trim() <= 10.0,
            "malformed asymmetric bounds fall back to finite symmetric native trim limits");
    }

    private static AutoTrimController.Input responseInput(long tick, double trim,
            double slope, double elevator, int damage, double flightPath) {
        return new AutoTrimController.Input(tick, trim, 10.0, 50.0, 50.0,
            900.0, 30.0, 1.6, 0.0, 3.0, 0.0, 0.0, flightPath,
            flightPath == -10.0 ? 0.0 : 900.0 * (slope * trim + 0.0001 * Math.sin(tick * 0.74)), elevator, 0.0, 1200.0, 15000.0, damage,
            true, true, false, false, false, true);
    }

    private static void interruptedLearningAndCorrection() {
        for (double slope : new double[] {0.002, -0.002}) {
            for (boolean changedConfiguration : new boolean[] {false, true}) {
                AutoTrimController controller = new AutoTrimController();
                controller.enable(new UUID(1L, 2L), 0.0);
                double trim = 0.0;
                long tick = 0;
                String fit = null;
                while (++tick < 100 && fit == null) {
                    trim = controller.update(responseInput(tick, trim, slope, 0.0, 0, 0.5)).trim();
                    fit = controller.consumeFitDiagnostic();
                }
                require(fit != null && fit.contains("ACCEPTED"), "first probe measures a confident response");
                require(controller.state() == AutoTrimController.State.LEARNING,
                    "one noisy fit alone cannot authorize correction");
                // Interrupt a partly moved second probe and retain its actual setting.
                for (int i = 0; i < 15; ++i) {
                    trim = controller.update(responseInput(++tick, trim, slope, 0.0, 0, 0.5)).trim();
                }
                for (int i = 0; i < 20; ++i) {
                    AutoTrimController.Output paused = controller.update(
                        responseInput(++tick, trim, slope, 1.0, 0, 0.5));
                    require(paused.trim() == trim && !paused.adjusting(),
                        "pilot input immediately freezes all probe and correction writes");
                }
                int damage = changedConfiguration ? 1 : 0;
                int resumedTicks = 0;
                while (++resumedTicks < 200 && controller.state() != AutoTrimController.State.ON) {
                    double previous = trim;
                    AutoTrimController.Output resumed = controller.update(
                        responseInput(++tick, trim, slope, 0.0, damage, 0.5));
                    require(!"RESTORING_TRIM".equals(resumed.reason()),
                        "pilot interruption never rewinds an incomplete probe");
                    trim = resumed.trim();
                    require(Math.abs(trim - previous) <= AutoTrimController.MAX_STEP_PER_TICK + 1.0E-9,
                        "resumed learning respects native trim slew");
                }
                require(controller.state() == AutoTrimController.State.ON, "compatible response learning completes");
                require(changedConfiguration ? resumedTicks > 120 : resumedTicks < 110,
                    "compatible completed fit survives input; damage requires two new fits");
                AutoTrimController.Output correction = controller.update(
                    responseInput(++tick, trim, slope, 0.0, damage, -10.0));
                double delta = correction.trim() - trim;
                require(Math.signum(delta) == Math.signum(slope),
                    "descending hands-off flight requests nose-up trim for either response sign");
                require(Math.abs(delta) > 0.03 && Math.abs(delta) <= AutoTrimController.MAX_STEP_PER_TICK,
                    "larger descent requests a stronger correction within native slew");
                trim = correction.trim();
                AutoTrimController.Output manual = controller.update(
                    responseInput(++tick, trim, slope, 1.0, damage, -10.0));
                require(manual.trim() == trim && "PILOT_INPUT".equals(manual.reason()),
                    "active correction yields immediately to pilot input");
                controller.disable("OFF");
                require(controller.update(responseInput(++tick, trim, slope, 0.0, damage, -10.0)).trim() == trim,
                    "explicit off freezes trim");
            }
        }
    }

    private static void confidentResponseSurvivesPilotManeuver() {
        AutoTrimController controller = new AutoTrimController();
        controller.enable(new UUID(4L, 9L), 1.0);
        double trim = 1.0;
        long tick = 0;
        while (++tick < 100 && controller.state() != AutoTrimController.State.ON) {
            trim = controller.update(maneuverInput(tick, trim, 50, 3, 0)).trim();
        }
        require(controller.state() == AutoTrimController.State.ON && tick < 70,
            "a decisive response fit starts correction after one complete probe");
        double held = trim;
        for (int i=0;i<40;++i) {
            AutoTrimController.Output out=controller.update(maneuverInput(++tick,trim,80,12,1));
            require(out.trim()==held && "PILOT_INPUT".equals(out.reason()),
                "pilot maneuver holds trim exactly even as speed and AoA change");
        }
        for (int i=0;i<20 && controller.state()!=AutoTrimController.State.ON;++i) {
            AutoTrimController.Output out=controller.update(maneuverInput(++tick,trim,80,12,0));
            require(out.trim()==held && out.state()!=AutoTrimController.State.LEARNING,
                "returning from a compatible maneuver resumes rather than recalibrating or rewinding");
            trim=out.trim();
        }
        require(controller.state()==AutoTrimController.State.ON,"learned authority resumes promptly");
    }

    private static void pilotCancelsPreviouslyQueuedProbeRestore() {
        AutoTrimController controller = new AutoTrimController();
        controller.enable(new UUID(5L, 9L), 1.0);
        double trim = 1.0;
        long tick = 0;
        while (++tick <= 25) {
            trim = controller.update(maneuverInput(tick, trim, 50, 3, 0)).trim();
        }
        require(Math.abs(trim - 1.0) > 0.1, "fixture interrupts a displaced, incomplete probe");
        double held = trim;
        AutoTrimController.Output unsafe = controller.update(maneuverInput(++tick, trim, 50, 20, 0));
        require("HIGH_AOA".equals(unsafe.reason()) && unsafe.trim() == held,
            "unsafe flight holds trim while scheduling cleanup");
        controller.update(maneuverInput(++tick, trim, 50, 3, 1));
        for (int i = 0; i < 12; ++i) {
            AutoTrimController.Output resumed = controller.update(maneuverInput(++tick, trim, 50, 3, 0));
            require(!"RESTORING_TRIM".equals(resumed.reason()) && resumed.trim() == held,
                "pilot override discards an earlier cleanup and learns from the held setting");
            trim = resumed.trim();
        }
    }

    private static AutoTrimController.Input maneuverInput(long tick,double trim,double speed,double aoa,double elevator) {
        double pressure=.36*speed*speed;
        return new AutoTrimController.Input(tick,trim,10,speed,speed,pressure,30,1.6,
            0,aoa,0,0,.5,pressure*.002*trim,elevator,0,1200,15000,0,
            true,true,false,false,false,true);
    }

    public static void main(String[] args) {
        perAircraftOptInPolicy();
        groundPauseAndReenable();
        statusCodec();
        trimResponseFit();
        authoredModifierOffsetComposition();
        interruptedLearningAndCorrection();
        pilotCancelsPreviouslyQueuedProbeRestore();
        confidentResponseSurvivesPilotManeuver();
        System.out.println("AutoTrimRegression: " + assertions + " assertions passed");
    }
}
