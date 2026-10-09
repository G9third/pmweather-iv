package com.g9third.pmweatheriv.sable;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import org.joml.Quaterniond;
import org.joml.Vector3d;

/**
 * Substep-resolved tap of Sable 2.0.3's LevelCollider contact-force buffer.
 *
 * <p>Sable normally accumulates contact reports from every physics substep and
 * exposes one unlabelled, 100-record array after the complete 20 Hz tick. A
 * force in that array is the force from one substep, not a force that acted for
 * the complete 0.05-second game tick. Treating the strongest reported force as
 * a full-tick force exaggerates a one-substep wheel/body impact by the Sable
 * substep count.</p>
 *
 * <p>While a PMWeather-IV vehicle is active, the manager drains the native
 * report after each completed substep and records that substep's exact time
 * interval. At the end of the tick the pipeline mixin returns the first 100
 * records to Sable in their original order, preserving Sable's existing public
 * collision-effect limit. PMWeather-IV retains the substep labels internally
 * so tire Coulomb limits and IV-compatible terrain impact severity integrate
 * force * dt exactly instead of assuming every contact lasted a full tick.</p>
 */
public final class SableCollisionCapture {
    private static final int RECORD_WIDTH = 15;
    private static final int SABLE_RECORD_LIMIT = 100;
    private static final double FULL_TICK_SECONDS = 1.0 / 20.0;
    private static final Map<Long, SceneSnapshot> SNAPSHOTS = new ConcurrentHashMap<>();
    private static final Map<Long, TickAccumulator> ACTIVE_TICKS = new ConcurrentHashMap<>();
    private static final Map<Long, double[]> COMPLETED_EXTERNAL_REPLAY = new ConcurrentHashMap<>();

    private SableCollisionCapture() {
    }

    /** Starts a new Sable 20 Hz tick before its first physics substep. */
    public static void beginTick(long sceneHandle) {
        if (sceneHandle != 0L) {
            ACTIVE_TICKS.put(sceneHandle, new TickAccumulator());
        }
    }

    /** Records one body's exact motion immediately before the current substep. */
    public static void capturePreSubstepMotion(
        long sceneHandle,
        int bodyId,
        Vector3d positionWorld,
        Quaterniond orientationWorld,
        Vector3d linearVelocityWorld,
        Vector3d angularVelocityWorld
    ) {
        if (sceneHandle == 0L || bodyId < 0
            || !finite(positionWorld) || !finite(orientationWorld)
            || !finite(linearVelocityWorld) || !finite(angularVelocityWorld)) {
            return;
        }
        TickAccumulator accumulator = ACTIVE_TICKS.get(sceneHandle);
        if (accumulator == null) {
            return;
        }
        synchronized (accumulator) {
            List<BodyMotionSnapshot> motions = accumulator.bodyMotions.computeIfAbsent(
                bodyId, unused -> new ArrayList<>()
            );
            int substepIndex = accumulator.substepsCaptured;
            while (motions.size() <= substepIndex) {
                motions.add(null);
            }
            motions.set(substepIndex, new BodyMotionSnapshot(
                new Vector3d(positionWorld),
                new Quaterniond(orientationWorld).normalize(),
                new Vector3d(linearVelocityWorld),
                new Vector3d(angularVelocityWorld)
            ));
        }
    }

    /** Captures and labels one completed Sable physics substep. */
    public static void captureSubstep(long sceneHandle, double timeStepSeconds, double[] raw) {
        if (sceneHandle == 0L) {
            return;
        }
        double safeTimeStep = Double.isFinite(timeStepSeconds) && timeStepSeconds > 0.0
            ? timeStepSeconds
            : FULL_TICK_SECONDS;
        TickAccumulator accumulator = ACTIVE_TICKS.computeIfAbsent(
            sceneHandle, unused -> new TickAccumulator()
        );
        synchronized (accumulator) {
            int substepIndex = accumulator.substepsCaptured++;
            accumulator.appendForSable(raw);
            accumulator.contacts.addAll(parse(raw, safeTimeStep, substepIndex));
        }
    }

    /**
     * Publishes the completed substep-resolved snapshot and returns the exact
     * first 100 native records for Sable's ordinary post-tick processing.
     * Returns {@code null} when no per-substep capture was active for the scene.
     */
    public static double[] finishTickForSable(long sceneHandle) {
        if (sceneHandle == 0L) {
            return null;
        }
        TickAccumulator accumulator = ACTIVE_TICKS.remove(sceneHandle);
        if (accumulator == null) {
            return null;
        }
        synchronized (accumulator) {
            publish(
                sceneHandle,
                accumulator.substepsCaptured,
                accumulator.contacts,
                accumulator.bodyMotions
            );
            COMPLETED_EXTERNAL_REPLAY.put(sceneHandle, accumulator.allNativeRecords());
            return accumulator.sableRecords();
        }
    }


    /**
     * Returns the complete native collision stream captured across all Sable
     * substeps for third-party damage integrations, then clears that replay.
     * Sable itself still receives its historical first-100-record view.
     */
    public static double[] consumeCompletedExternalReplay(long sceneHandle) {
        if (sceneHandle == 0L) return null;
        return COMPLETED_EXTERNAL_REPLAY.remove(sceneHandle);
    }

    /**
     * Removes raw collision records involving PMIV's direct persistent vehicle bodies
     * before forwarding a batch to third-party Sable damage integrations.
     *
     * <p>Standalone True Impact correlates active bodies through SableEventBridge's
     * ServerSubLevel snapshots. PMIV vehicles are not ServerSubLevels; forwarding a
     * vehicle-vs-sublevel record unchanged would make the unknown vehicle side look
     * like static world terrain to that resolver. The per-substep PMIV motion map is an
     * exact scene-local set of PMIV vehicle body IDs for the completed tick, so exclude
     * any 15-double record touching one of those IDs. Sable itself still receives its
     * unfiltered historical collision stream.</p>
     */
    public static double[] withoutPmivVehicleContacts(long sceneHandle, double[] raw) {
        if (raw == null || raw.length < RECORD_WIDTH || sceneHandle == 0L) {
            return raw;
        }
        SceneSnapshot snapshot = SNAPSHOTS.get(sceneHandle);
        if (snapshot == null || snapshot.bodyMotions().isEmpty()) {
            return raw;
        }
        Map<Integer, List<BodyMotionSnapshot>> pmivBodies = snapshot.bodyMotions();
        int completeRecords = raw.length / RECORD_WIDTH;
        int keptRecords = 0;
        for (int index = 0; index < completeRecords; ++index) {
            int start = index * RECORD_WIDTH;
            int bodyA = (int) raw[start];
            int bodyB = (int) raw[start + 1];
            if (!pmivBodies.containsKey(bodyA) && !pmivBodies.containsKey(bodyB)) {
                ++keptRecords;
            }
        }
        if (keptRecords == completeRecords) {
            return raw;
        }
        if (keptRecords == 0) {
            return new double[0];
        }
        double[] filtered = new double[keptRecords * RECORD_WIDTH];
        int write = 0;
        for (int index = 0; index < completeRecords; ++index) {
            int start = index * RECORD_WIDTH;
            int bodyA = (int) raw[start];
            int bodyB = (int) raw[start + 1];
            if (pmivBodies.containsKey(bodyA) || pmivBodies.containsKey(bodyB)) {
                continue;
            }
            System.arraycopy(raw, start, filtered, write, RECORD_WIDTH);
            write += RECORD_WIDTH;
        }
        return filtered;
    }

    /** Fallback for a scene that had no active PMWeather-IV substep tap. */
    public static void captureWholeTick(long sceneHandle, double[] raw) {
        publish(sceneHandle, 1, parse(raw, FULL_TICK_SECONDS, 0), Map.of());
    }

    private static List<ContactForce> parse(
        double[] raw,
        double timeStepSeconds,
        int substepIndex
    ) {
        if (raw == null || raw.length < RECORD_WIDTH) {
            return List.of();
        }
        int count = raw.length / RECORD_WIDTH;
        List<ContactForce> contacts = new ArrayList<>(count);
        for (int index = 0; index < count; ++index) {
            int start = index * RECORD_WIDTH;
            double force = raw[start + 2];
            if (!Double.isFinite(force) || force <= 0.0) {
                continue;
            }
            Vector3d normalA = vector(raw, start + 3);
            Vector3d normalB = vector(raw, start + 6);
            Vector3d pointA = vector(raw, start + 9);
            Vector3d pointB = vector(raw, start + 12);
            if (!finite(normalA) || !finite(normalB) || !finite(pointA) || !finite(pointB)) {
                continue;
            }
            contacts.add(new ContactForce(
                (int) raw[start],
                (int) raw[start + 1],
                force,
                timeStepSeconds,
                force * timeStepSeconds,
                substepIndex,
                normalA,
                normalB,
                pointA,
                pointB
            ));
        }
        return contacts;
    }

    private static void publish(
        long sceneHandle,
        int substepsCaptured,
        List<ContactForce> captured,
        Map<Integer, List<BodyMotionSnapshot>> capturedMotions
    ) {
        if (sceneHandle == 0L) {
            return;
        }
        SceneSnapshot previous = SNAPSHOTS.get(sceneHandle);
        long generation = previous == null ? 1L : previous.generation() + 1L;
        List<ContactForce> contacts = captured == null || captured.isEmpty()
            ? List.of()
            : Collections.unmodifiableList(new ArrayList<>(captured));
        Map<Integer, List<BodyMotionSnapshot>> bodyMotions = immutableMotionMap(
            capturedMotions
        );
        SNAPSHOTS.put(
            sceneHandle,
            new SceneSnapshot(
                generation,
                Math.max(0, substepsCaptured),
                contacts.size(),
                contacts,
                bodyMotions
            )
        );
    }

    private static Map<Integer, List<BodyMotionSnapshot>> immutableMotionMap(
        Map<Integer, List<BodyMotionSnapshot>> source
    ) {
        if (source == null || source.isEmpty()) {
            return Map.of();
        }
        Map<Integer, List<BodyMotionSnapshot>> copy = new LinkedHashMap<>();
        for (Map.Entry<Integer, List<BodyMotionSnapshot>> entry : source.entrySet()) {
            List<BodyMotionSnapshot> motions = entry.getValue();
            if (motions != null && !motions.isEmpty()) {
                // Null marks a substep that completed before this body joined
                // the scene. Preserve the exact native substep index.
                copy.put(
                    entry.getKey(),
                    Collections.unmodifiableList(new ArrayList<>(motions))
                );
            }
        }
        return copy.isEmpty() ? Map.of() : Collections.unmodifiableMap(copy);
    }

    /** Latest complete post-physics collision snapshot for one Sable scene. */
    public static SceneSnapshot snapshot(long sceneHandle) {
        SceneSnapshot snapshot = SNAPSHOTS.get(sceneHandle);
        return snapshot == null ? SceneSnapshot.EMPTY : snapshot;
    }

    public static void clearScene(long sceneHandle) {
        if (sceneHandle != 0L) {
            SNAPSHOTS.remove(sceneHandle);
            ACTIVE_TICKS.remove(sceneHandle);
            COMPLETED_EXTERNAL_REPLAY.remove(sceneHandle);
        }
    }

    private static Vector3d vector(double[] raw, int start) {
        return new Vector3d(raw[start], raw[start + 1], raw[start + 2]);
    }

    private static boolean finite(Vector3d value) {
        return value != null
            && Double.isFinite(value.x)
            && Double.isFinite(value.y)
            && Double.isFinite(value.z);
    }

    private static boolean finite(Quaterniond value) {
        return value != null
            && Double.isFinite(value.x)
            && Double.isFinite(value.y)
            && Double.isFinite(value.z)
            && Double.isFinite(value.w)
            && value.lengthSquared() > 1.0E-18;
    }

    public record SceneSnapshot(
        long generation,
        int substepsCaptured,
        int contactRecordsCaptured,
        List<ContactForce> contacts,
        Map<Integer, List<BodyMotionSnapshot>> bodyMotions
    ) {
        private static final SceneSnapshot EMPTY = new SceneSnapshot(
            0L, 0, 0, List.of(), Map.of()
        );

        public BodyMotionSnapshot bodyMotion(int bodyId, int substepIndex) {
            List<BodyMotionSnapshot> motions = bodyMotions.get(bodyId);
            return motions == null || substepIndex < 0 || substepIndex >= motions.size()
                ? null
                : motions.get(substepIndex);
        }
    }

    public record BodyMotionSnapshot(
        Vector3d positionWorld,
        Quaterniond orientationWorld,
        Vector3d linearVelocityWorld,
        Vector3d angularVelocityWorld
    ) {
    }

    public record ContactForce(
        int colliderA,
        int colliderB,
        double forceNewtons,
        double timeStepSeconds,
        double impulseNewtonSeconds,
        int substepIndex,
        Vector3d localNormalA,
        Vector3d localNormalB,
        Vector3d localPointA,
        Vector3d localPointB
    ) {
        public int otherCollider(int colliderId) {
            if (colliderA == colliderId) {
                return colliderB;
            }
            if (colliderB == colliderId) {
                return colliderA;
            }
            return Integer.MIN_VALUE;
        }

        public Vector3d localNormalFor(int colliderId) {
            if (colliderA == colliderId) {
                return new Vector3d(localNormalA);
            }
            if (colliderB == colliderId) {
                return new Vector3d(localNormalB);
            }
            return new Vector3d();
        }

        public Vector3d localPointFor(int colliderId) {
            if (colliderA == colliderId) {
                return new Vector3d(localPointA);
            }
            if (colliderB == colliderId) {
                return new Vector3d(localPointB);
            }
            return new Vector3d();
        }
    }

    private static final class TickAccumulator {
        private final List<ContactForce> contacts = new ArrayList<>();
        private final Map<Integer, List<BodyMotionSnapshot>> bodyMotions =
            new LinkedHashMap<>();
        private final double[] rawForSable = new double[SABLE_RECORD_LIMIT * RECORD_WIDTH];
        private final List<double[]> rawSubsteps = new ArrayList<>();
        private int rawValueCount;
        private int substepsCaptured;

        private void appendForSable(double[] raw) {
            if (raw == null || raw.length < RECORD_WIDTH) {
                return;
            }
            int completeValues = (raw.length / RECORD_WIDTH) * RECORD_WIDTH;
            if (completeValues > 0) {
                double[] complete = new double[completeValues];
                System.arraycopy(raw, 0, complete, 0, completeValues);
                rawSubsteps.add(complete);
            }
            if (rawValueCount >= rawForSable.length) {
                return;
            }
            int copyValues = Math.min(completeValues, rawForSable.length - rawValueCount);
            System.arraycopy(raw, 0, rawForSable, rawValueCount, copyValues);
            rawValueCount += copyValues;
        }

        private double[] sableRecords() {
            double[] result = new double[rawValueCount];
            System.arraycopy(rawForSable, 0, result, 0, rawValueCount);
            return result;
        }

        private double[] allNativeRecords() {
            int total = 0;
            for (double[] raw : rawSubsteps) total += raw.length;
            double[] result = new double[total];
            int offset = 0;
            for (double[] raw : rawSubsteps) {
                System.arraycopy(raw, 0, result, offset, raw.length);
                offset += raw.length;
            }
            return result;
        }
    }
}
