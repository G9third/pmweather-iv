package com.g9third.pmweatheriv.physics;

/**
 * Bounded, per-body terrain-contact history for deciding when a body collision
 * is a new impact rather than ordinary support or sliding contact.
 *
 * <p>Normals are stored as canonical outward directions. Adjacent terrain
 * blocks with the same face normal therefore continue the existing contact.
 * This class is independent of Minecraft and does not modify body motion.</p>
 */
public final class BodyTerrainImpactPolicy {
    public static final int MAX_DIRECTIONS = 16;
    public static final int RECENT_CONTACT_GRACE_TICKS = 2;
    public static final double DIRECTION_MATCH_DOT = 0.8;
    public static final double FRESH_IMPACT_MIN_INWARD_MPS = 1.0;
    public static final double PERSISTENT_IMPACT_MIN_INWARD_MPS = 2.5;

    public static final String REASON_INVALID = "INVALID";
    public static final String REASON_NEW_IMPACT = "NEW_IMPACT";
    public static final String REASON_REIMPACT = "REIMPACT";
    public static final String REASON_SUPPORT_CONTACT = "SUPPORT_CONTACT";
    public static final String REASON_SOFT_TOUCH = "SOFT_TOUCH";

    /** A classification result; instances are shared to avoid per-contact allocations. */
    public record Decision(boolean freshContact, boolean damageEligible, String reason) {}

    private static final Decision INVALID = new Decision(false, false, REASON_INVALID);
    private static final Decision NEW_IMPACT = new Decision(true, true, REASON_NEW_IMPACT);
    private static final Decision SOFT_TOUCH = new Decision(true, false, REASON_SOFT_TOUCH);
    private static final Decision REIMPACT = new Decision(false, true, REASON_REIMPACT);
    private static final Decision SUPPORT_CONTACT = new Decision(false, false, REASON_SUPPORT_CONTACT);

    private final double[] normalX = new double[MAX_DIRECTIONS];
    private final double[] normalY = new double[MAX_DIRECTIONS];
    private final double[] normalZ = new double[MAX_DIRECTIONS];
    private final long[] lastSeenTick = new long[MAX_DIRECTIONS];
    private final long[] classifiedTick = new long[MAX_DIRECTIONS];
    private final boolean[] occupied = new boolean[MAX_DIRECTIONS];
    private final boolean[] classifiedFresh = new boolean[MAX_DIRECTIONS];
    private boolean hasLastTick;
    private long lastTick;

    /**
     * Classifies one collision sample for this body.
     *
     * <p>Positive finite inward speed and impulse are required for damage. Quiet
     * geometric stations still refresh contact history. A valid soft
     * touch still records its face, so later substeps cannot turn sustained
     * support into a fresh impact. Repeated queries for the same face on one
     * game tick retain that tick's initial fresh/persistent classification;
     * each query independently evaluates its severity, so every collision
     * voxel on a fresh impact can be eligible.</p>
     */
    public Decision classify(long tick, Vec3d outwardNormal, double inwardMps, double impulseNs) {
        if (hasLastTick && tick < lastTick) clearHistory();
        hasLastTick = true;
        lastTick = tick;

        if (outwardNormal == null || !outwardNormal.isFinite()
            || inwardMps < 0.0 || !Double.isFinite(inwardMps)
            || impulseNs < 0.0 || !Double.isFinite(impulseNs)) {
            return INVALID;
        }

        // Scale first, so every finite nonzero vector (including extreme
        // magnitudes) can be normalized without overflow or underflow.
        double scale = Math.max(Math.abs(outwardNormal.x()),
            Math.max(Math.abs(outwardNormal.y()), Math.abs(outwardNormal.z())));
        if (!(scale > 0.0) || !Double.isFinite(scale)) return INVALID;
        double x = outwardNormal.x() / scale;
        double y = outwardNormal.y() / scale;
        double z = outwardNormal.z() / scale;
        double length = Math.sqrt(x * x + y * y + z * z);
        if (!(length > 0.0) || !Double.isFinite(length)) return INVALID;
        x /= length;
        y /= length;
        z /= length;

        int match = -1;
        double bestDot = DIRECTION_MATCH_DOT;
        for (int i = 0; i < MAX_DIRECTIONS; ++i) {
            if (!occupied[i]) continue;
            double dot = x * normalX[i] + y * normalY[i] + z * normalZ[i];
            if (dot >= bestDot) {
                bestDot = dot;
                match = i;
            }
        }

        boolean fresh;
        if (match >= 0) {
            if (classifiedTick[match] == tick) {
                // Keep the initial classification for same-owner-tick
                // collision voxels, even after updating lastSeenTick.
                fresh = classifiedFresh[match];
            } else {
                fresh = beyondGrace(tick, lastSeenTick[match]);
                classifiedTick[match] = tick;
                classifiedFresh[match] = fresh;
            }
            lastSeenTick[match] = tick;
        } else {
            int slot = findAvailableSlot();
            // Keep the original face direction as the canonical key. A
            // neighboring soil block with this same normal cannot reset it.
            normalX[slot] = x;
            normalY[slot] = y;
            normalZ[slot] = z;
            lastSeenTick[slot] = tick;
            classifiedTick[slot] = tick;
            fresh = inwardMps > 0.0 && impulseNs > 0.0;
            classifiedFresh[slot] = fresh;
            occupied[slot] = true;
        }

        // Quiet and released constraints still observe the geometric face.
        // They carry no impact work, but must not age support out of history.
        if (inwardMps == 0.0 || impulseNs == 0.0) return INVALID;
        if (fresh) {
            return inwardMps >= FRESH_IMPACT_MIN_INWARD_MPS ? NEW_IMPACT : SOFT_TOUCH;
        }
        return inwardMps >= PERSISTENT_IMPACT_MIN_INWARD_MPS ? REIMPACT : SUPPORT_CONTACT;
    }

    /**
     * Keeps deferred, closed, or invalid terrain manifolds on the rigid
     * response path. An accepted manifold with an opened terrain residual can
     * use material compliance, including a positive inward residual.
     */
    public static boolean requiresRigidManifold(boolean accepted, boolean terrainOpened,
                                                double residualInwardSpeedMps) {
        if (!accepted || !Double.isFinite(residualInwardSpeedMps) || residualInwardSpeedMps < 0.0) {
            return true;
        }
        return !terrainOpened && residualInwardSpeedMps <= 1.0e-9;
    }

    private static boolean beyondGrace(long tick, long seenTick) {
        // This addition cannot overflow for the ordinary game tick range; the
        // guard also keeps behavior defined for arbitrary long inputs.
        return seenTick <= Long.MAX_VALUE - RECENT_CONTACT_GRACE_TICKS
            && tick > seenTick + RECENT_CONTACT_GRACE_TICKS;
    }

    private int findAvailableSlot() {
        int oldest = -1;
        for (int i = 0; i < MAX_DIRECTIONS; ++i) {
            if (!occupied[i]) return i;
            if (oldest < 0 || lastSeenTick[i] < lastSeenTick[oldest]) oldest = i;
        }
        return oldest;
    }

    private void clearHistory() {
        for (int i = 0; i < MAX_DIRECTIONS; ++i) occupied[i] = false;
    }
}
