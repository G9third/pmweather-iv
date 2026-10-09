package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import java.util.UUID;

/** Checks no-op observation and the production actuator snapshot contract. */
public final class ObserverIsolationRegression {
    private static int checks;
    private static void require(boolean condition, String message) {
        ++checks;
        if (!condition) throw new AssertionError(message);
    }

    public static void main(String[] args) {
        UUID uuid = new UUID(3, 4);
        require(!PMIVObserver.loggingEnabled(), "public observer unexpectedly logs");
        require(!PMIVObserver.isCapturing(uuid), "public observer unexpectedly captures");
        require(!PMIVObserver.needsDetailedSamples(uuid), "public observer requests detail");
        AirframeLoads.Accumulator publicOwner = new AirframeLoads.Accumulator(null, false, true);
        AirframeLoads.Accumulator observedOwner = new AirframeLoads.Accumulator(null, true, true);
        AirframeLoads.Accumulator substep = new AirframeLoads.Accumulator(null, false, false);
        require(publicOwner.surfaceSamples.isEmpty() && publicOwner.propulsionSamples.isEmpty(),
            "public accumulator created optional diagnostic records");
        for (String type : new String[] {"PROPELLER", "JET", "ROTOR", "MAIN_ROTOR", "TAIL_ROTOR"}) {
            Vec3d force = new Vec3d(10, 20, 30);
            Vec3d torque = new Vec3d(2, -3, 4);
            publicOwner.recordPropulsionLoad(type, 5, force, torque);
            observedOwner.recordPropulsionLoad(type, 5, force, torque);
            substep.recordPropulsionLoad(type, 5, force, torque);
            require(publicOwner.propulsionLoads.equals(observedOwner.propulsionLoads),
                "observation changed physical actuator snapshots");
            var load = publicOwner.propulsionLoads.get(publicOwner.propulsionLoads.size() - 1);
            require(load.type().equals(type) && load.mtsForceValue() == 5
                    && load.forceWorldNewtons().equals(force) && load.torqueBodyNewtonMeters().equals(torque),
                "compact actuator snapshot lost a physical field");
        }
        require(substep.propulsionLoads.isEmpty(), "substep recaptured owner-tick actuator loads");
        require(publicOwner.propulsionSamples.isEmpty(), "physical snapshot populated public debug samples");
        System.out.println("ObserverIsolationRegression: " + checks + " assertions passed");
    }
}
