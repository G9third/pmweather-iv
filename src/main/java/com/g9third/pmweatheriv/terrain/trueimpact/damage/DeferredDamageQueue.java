/*
 * Portions derived from True Impact 0.5.8-delta by OMEGAU371 / True Impact contributors.
 * Original project: https://github.com/OMEGAU371/sable-true-impact
 * Original license: LGPL-3.0-only.
 * PMWeather-IV modification notice: adapted for PMIV; notice updated 2026-10-09.
 * Adapted for PMWeather-IV's integrated aircraft/world material-response subsystem.
 */
package com.g9third.pmweatheriv.terrain.trueimpact.damage;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

/** Server-thread queue drained by IntegratedImpactTerrain at ServerTickEvent.Post. */
public final class DeferredDamageQueue {
    private DeferredDamageQueue() {}
    static final int MAX_PENDING = 64;
    private static final ArrayDeque<DeferredDamageEvent> pending = new ArrayDeque<>();
    private static final HashSet<String> seenThisTick = new HashSet<>();
    private static long lastEnqueueTick = -1L;

    public static boolean enqueue(DeferredDamageEvent event) {
        if (!Double.isFinite(event.kImpact())) return false;
        if (event.serverTick() != lastEnqueueTick) {
            seenThisTick.clear();
            lastEnqueueTick = event.serverTick();
        }
        String key = event.levelKey() + "," + event.posX() + "," + event.posY() + "," + event.posZ()
            + "," + event.victimBlock();
        if (!seenThisTick.add(key) || pending.size() >= MAX_PENDING) return false;
        pending.add(event);
        return true;
    }

    public static List<DeferredDamageEvent> drainAll() {
        if (pending.isEmpty()) return List.of();
        List<DeferredDamageEvent> result = new ArrayList<>(pending);
        pending.clear();
        return result;
    }

    public static void clear() {
        pending.clear();
        seenThisTick.clear();
        lastEnqueueTick = -1L;
    }
}
