package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import java.util.ArrayDeque;
import java.util.Deque;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import net.minecraft.server.level.ServerLevel;

/**
 * Server-side atmospheric warning measurements exported to IV computed variables.
 *
 * <p>The first public warning is wind shear. PMIV compares the source-native mean
 * PMWeather wind currently sampled at the real aerodynamic force points with the
 * wind experienced one second earlier. This is intentionally an environmental
 * measurement, not a synthetic force, attitude aid, or flight-state override.</p>
 */
public final class AirWarningSystem {
    /** Exact international knot in metres per second. */
    public static final double KNOT_TO_METERS_PER_SECOND = 0.5144444444444445;
    /** Warning measurement window. One Minecraft second at 20 Hz. */
    public static final long WIND_SHEAR_WINDOW_TICKS = 20L;
    private static final long HISTORY_RETENTION_TICKS = 50L;
    private static final double WARNING_RELEASE_FRACTION = 0.80;

    private AirWarningSystem() {
    }

    /** Mutable per-aircraft history. Stored inside {@link AircraftState}. */
    public static final class State {
        private final Deque<WindFrame> windHistory = new ArrayDeque<>(32);
        private Snapshot snapshot = Snapshot.ZERO;
        private long lastGameTime = Long.MIN_VALUE;

        public Snapshot snapshot() {
            return snapshot;
        }

        public void reset() {
            windHistory.clear();
            snapshot = Snapshot.ZERO;
            lastGameTime = Long.MIN_VALUE;
        }
    }

    /** Values sent to clients and exposed to normal IV sound animations. */
    public record Snapshot(
        boolean windShearWarning,
        double windShearLevel,
        double windShearDeltaMps,
        double windShearVerticalDeltaMps,
        double windSpeedMps,
        double verticalWindMps
    ) {
        public static final Snapshot ZERO = new Snapshot(false, 0.0, 0.0, 0.0, 0.0, 0.0);

        public boolean finite() {
            return Double.isFinite(windShearLevel)
                && Double.isFinite(windShearDeltaMps)
                && Double.isFinite(windShearVerticalDeltaMps)
                && Double.isFinite(windSpeedMps)
                && Double.isFinite(verticalWindMps);
        }
    }

    private record WindFrame(long gameTime, Vec3d windMetersPerSecond) {
    }

    /**
     * Advances warning measurements once per owning Minecraft tick.
     * Substep aerodynamics reuse the frozen wind snapshot and never advance this history.
     */
    public static Snapshot update(
        EntityVehicleF_Physics vehicle,
        ServerLevel level,
        PMWeatherIVConfig.Values config,
        AircraftState aircraftState,
        AirframeLoads.SolveResult result
    ) {
        State state = aircraftState.airWarnings;
        if (vehicle == null || level == null || result == null
            || result.windFieldSnapshot() == null || result.windFieldSnapshot().points().isEmpty()) {
            state.snapshot = Snapshot.ZERO;
            return state.snapshot;
        }

        long gameTime = level.getGameTime();
        if (state.lastGameTime != Long.MIN_VALUE
            && (gameTime <= state.lastGameTime || gameTime - state.lastGameTime > 2L)) {
            // Chunk unload/reload, pause, dimension migration, or duplicate owner evaluation:
            // never interpret a discontinuity as atmospheric wind shear.
            state.windHistory.clear();
            state.snapshot = Snapshot.ZERO;
        }
        state.lastGameTime = gameTime;

        Vec3d meanWindMps = result.windFieldSnapshot().meanRawMph()
            .scale(FlightMath.MPH_TO_METERS_PER_SECOND);
        if (!meanWindMps.isFinite()) {
            state.snapshot = Snapshot.ZERO;
            state.windHistory.clear();
            return state.snapshot;
        }

        if (state.windHistory.isEmpty() || state.windHistory.peekLast().gameTime() != gameTime) {
            state.windHistory.addLast(new WindFrame(gameTime, meanWindMps));
        } else {
            state.windHistory.removeLast();
            state.windHistory.addLast(new WindFrame(gameTime, meanWindMps));
        }
        while (!state.windHistory.isEmpty()
            && state.windHistory.peekFirst().gameTime() < gameTime - HISTORY_RETENTION_TICKS) {
            state.windHistory.removeFirst();
        }

        long referenceTime = gameTime - WIND_SHEAR_WINDOW_TICKS;
        WindFrame reference = null;
        for (WindFrame frame : state.windHistory) {
            if (frame.gameTime() <= referenceTime) {
                reference = frame;
            } else {
                break;
            }
        }
        if (reference == null) {
            state.snapshot = Snapshot.ZERO;
            return state.snapshot;
        }

        Vec3d delta = meanWindMps.subtract(reference.windMetersPerSecond());
        double deltaMps = delta.length();
        // Signed +Y is increasing/upward source wind; negative is a downward change.
        double verticalDeltaMps = delta.y();
        double thresholdMps = Math.max(
            KNOT_TO_METERS_PER_SECOND,
            config.windShearWarningThresholdKnots() * KNOT_TO_METERS_PER_SECOND
        );
        double releaseMps = thresholdMps * WARNING_RELEASE_FRACTION;
        boolean wasActive = state.snapshot.windShearWarning();
        boolean active = wasActive ? deltaMps >= releaseMps : deltaMps >= thresholdMps;

        // 0.5 corresponds to the configured warning threshold; 1.0 is twice it.
        // This makes the value useful as a native IV volume/pitch animation input
        // without hiding the raw SI/knots measurements from content packs.
        double levelValue = Vec3d.clamp(deltaMps / (2.0 * thresholdMps), 0.0, 1.0);
        state.snapshot = new Snapshot(
            active, levelValue, deltaMps, verticalDeltaMps,
            meanWindMps.length(), meanWindMps.y()
        );
        if (PMIVObserver.loggingEnabled() && active != wasActive) {
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "AIR_WARNING_WINDSHEAR uuid=" + vehicle.uniqueUUID
                    + " active=" + active
                    + " deltaMps=" + deltaMps
                    + " verticalDeltaMps=" + verticalDeltaMps
                    + " thresholdMps=" + thresholdMps
                    + " windSpeedMps=" + meanWindMps.length()
                    + " verticalWindMps=" + meanWindMps.y()
                    + " windowTicks=" + WIND_SHEAR_WINDOW_TICKS
                    + " source=PMWEATHER_FORCE_POINT_MEAN"
            );
            }
        }
        return state.snapshot;
    }
}
