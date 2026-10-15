package com.g9third.pmweatheriv.physics;

import java.util.List;
import com.g9third.pmweatheriv.PMWeatherIVConfig;

/** Force-triggered tire breakaway on every terrain; ordinary pack grip remains the default. */
public final class RoadWindTraction {
    private static final double WIND_BREAKAWAY_CAPACITY_FRACTION = 0.25;
    private static final double WIND_SLIDING_GRIP_FRACTION = 0.25;
    private static final double BREAKAWAY_TRANSITION_OVERLOAD = 0.10;
    private static final double SKID_RELEASE_SPEED_MPS = 0.25;
    private RoadWindTraction() {}

    public static final class State {
        public double fraction;
        public double weatherDemandN;
        public double looseSurfaceCapacityN;
        public double breakawayThresholdN;
        public double residualSlipMps;
        public boolean skidTriggered;

        public void reset() {
            fraction = weatherDemandN = looseSurfaceCapacityN = breakawayThresholdN = residualSlipMps = 0.0;
            skidTriggered = false;
        }
    }

    /** Conservative slide limits, not measured coefficients for every tire/soil combination. */
    public static TireContactMaterial.Grip slidingGrip(TireContactMaterial.Grip authored) {
        double motive, lateral;
        switch (authored.material()) {
            case "GRASS" -> { motive = authored.wet() ? 0.20 : 0.30; lateral = authored.wet() ? 0.22 : 0.35; }
            case "DIRT", "DIRT_WET", "CLAY" -> { motive = authored.wet() ? 0.28 : 0.40; lateral = authored.wet() ? 0.32 : 0.45; }
            case "GRAVEL" -> { motive = authored.wet() ? 0.32 : 0.45; lateral = authored.wet() ? 0.38 : 0.50; }
            case "SAND", "SAND_WET" -> { motive = authored.wet() ? 0.22 : 0.30; lateral = authored.wet() ? 0.28 : 0.35; }
            case "SNOW" -> { motive = authored.wet() ? 0.12 : 0.15; lateral = authored.wet() ? 0.15 : 0.20; }
            case "ICE" -> { motive = 0.08; lateral = 0.10; }
            default -> { motive = authored.motive(); lateral = authored.lateral(); }
        }
        return new TireContactMaterial.Grip(Math.min(authored.motive(), motive) * WIND_SLIDING_GRIP_FRACTION,
            Math.min(authored.lateral(), lateral) * WIND_SLIDING_GRIP_FRACTION, authored.material(), authored.wet());
    }

    /** Advances only in the live contact solve. No wind query or additional force is applied. */
    public static double advance(State state, Vec3d weatherForceWorld,
            List<GroundContactImpulseSolver.Contact> contacts, List<TireContactMaterial.Grip> sliding,
            double[] normalImpulses, double seconds) {
        if (state == null) return 0.0;
        if (contacts.isEmpty() || weatherForceWorld == null || !weatherForceWorld.isFinite()
            || !(seconds > 0.0)) { state.reset(); return 0.0; }
        double load = 0.0, demand = 0.0, capacity = 0.0, activationCapacity = 0.0;
        boolean canReduce = false;
        for (int i = 0; i < contacts.size(); ++i) {
            var c = contacts.get(i);
            var grip = sliding.get(i);
            double normalN = normalImpulses[i] / seconds;
            if (!(normalN > 1.0e-6)) continue;
            Vec3d n = c.normalWorld().normalized();
            Vec3d forward = GroundContactImpulseSolver.tangentForward(c.forwardWorld(), n);
            Vec3d side = n.cross(forward).normalized();
            Vec3d tangent = weatherForceWorld.subtract(n.scale(weatherForceWorld.dot(n)));
            double tangentLength = tangent.length();
            double cap = 0.0;
            if (tangentLength > 1.0e-9) {
                double f = tangent.dot(forward) / tangentLength;
                double s = tangent.dot(side) / tangentLength;
                double ellipse = (Math.abs(f) > 1.0e-9 && grip.motive() <= 0.0)
                    || (Math.abs(s) > 1.0e-9 && grip.lateral() <= 0.0) ? Double.POSITIVE_INFINITY
                    : f*f / Math.max(1.0e-18, grip.motive()*grip.motive())
                        + s*s / Math.max(1.0e-18, grip.lateral()*grip.lateral());
                cap = normalN / Math.max(1.0e-9, Math.sqrt(ellipse));
            }
            load += normalN;
            demand += normalN * tangentLength;
            capacity += cap;
            boolean reduced = grip.motive() < c.motiveMu() || grip.lateral() < c.lateralMu();
            // Keep activation tuning independent from the lower sliding-grip limit.
            activationCapacity += reduced ? cap / WIND_SLIDING_GRIP_FRACTION : cap;
            canReduce |= reduced;
        }
        if (!(load > 1.0e-6)) { state.reset(); return 0.0; }
        state.weatherDemandN = demand / load;
        state.looseSurfaceCapacityN = capacity;
        double threshold = activationCapacity * WIND_BREAKAWAY_CAPACITY_FRACTION
            * PMWeatherIVConfig.get().breakawayForceMultiplier();
        state.breakawayThresholdN = threshold;
        double overload = canReduce && capacity > 1.0e-6 && state.weatherDemandN > 0.0
            ? state.weatherDemandN / Math.max(1.0e-9, threshold) : 0.0;
        // Once weather exceeds the breakaway threshold, reach sliding grip across
        // a narrow overload band rather than retaining most pack grip there.
        double x = Vec3d.clamp((overload - 1.0) / BREAKAWAY_TRANSITION_OVERLOAD, 0.0, 1.0);
        double target = x*x*(3.0 - 2.0*x);
        if (target > 0.0) state.skidTriggered = true;
        // Kinetic grip lasts while a weather-triggered skid is still moving.
        // Free wheel rotation is excluded from the measured slip below.
        if (canReduce && state.skidTriggered && state.residualSlipMps > SKID_RELEASE_SPEED_MPS)
            target = 1.0;
        double responseSeconds = target > state.fraction ? 0.10 : 0.25;
        state.fraction += (target - state.fraction) * -Math.expm1(-seconds / responseSeconds);
        if (state.fraction < 1.0e-5) {
            state.fraction = 0.0;
            state.skidTriggered = false;
        }
        return state.fraction;
    }
    /** Observe the final contact solve, after any wind-induced grip change. */
    public static void recordResidualSlip(State state, GroundContactImpulseSolver.Result result,
            List<GroundContactImpulseSolver.Contact> contacts,
            minecrafttransportsimulator.baseclasses.RotationMatrix orientation, double brake) {
        if (state == null) return;
        double maximum = 0.0;
        Vec3d omegaWorld = FlightMath.toWorld(orientation, result.angularVelocityBody());
        for (int i=0; i<contacts.size(); ++i) {
            if (!(result.normalImpulses()[i] > 1.0e-8)) continue;
            var contact = contacts.get(i);
            Vec3d normal = contact.normalWorld().normalized();
            Vec3d forward = GroundContactImpulseSolver.tangentForward(contact.forwardWorld(), normal);
            Vec3d side = normal.cross(forward).normalized();
            Vec3d velocity = result.velocityWorld().add(omegaWorld.cross(
                FlightMath.toWorld(orientation, contact.leverBody())));
            maximum = Math.max(maximum, Math.abs(velocity.dot(side)));
            if (brake > 0.0 || !contact.freeRolling())
                maximum = Math.max(maximum, Math.abs(velocity.dot(forward)));
        }
        state.residualSlipMps = maximum;
    }
}
