package com.g9third.pmweatheriv.compat;

import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import java.lang.reflect.Method;
import java.util.Map;
import net.minecraft.server.level.ServerLevel;

/**
 * Optional reflection-only compatibility with a separately installed True Impact mod.
 *
 * <p>PMIV owns the Sable 2.x clearCollisions redirect at priority 1100 so it can preserve
 * complete collision records across PMIV's explicit Sable substeps. Stock True Impact's
 * competing redirect is optional ({@code require=0}) and normally has lower/default mixin
 * priority, so PMIV forwards the complete collision batch into True Impact's existing
 * {@code SableImpactCapture.process(...)} entry point instead.</p>
 *
 * <p>This path is for ordinary Sable {@code ServerSubLevel} bodies. The pipeline
 * removes every raw collision record involving one of PMIV's direct persistent aircraft
 * body IDs before calling this bridge. That prevents True Impact from mistaking an
 * unknown PMIV aircraft body for static world terrain and keeps PMIV's integrated
 * True-Impact-derived aircraft/world material model as the sole terrain-damage authority
 * for PMIV aircraft.</p>
 */
public final class StandaloneTrueImpactCompat {
    private static final String EVENT_BRIDGE =
        "io.github.omegau371.trueimpact.sable.SableEventBridge";
    private static final String IMPACT_CAPTURE =
        "io.github.omegau371.trueimpact.sable.SableImpactCapture";

    private static volatile boolean resolutionAttempted;
    private static volatile boolean availabilityLogged;
    private static volatile Method getLastPostSnapshots;
    private static volatile Method getTickStartVels;
    private static volatile Method processImpacts;

    private StandaloneTrueImpactCompat() {}

    @SuppressWarnings("unchecked")
    public static void processSableCollisionBatch(ServerLevel level, double[] collisions) {
        if (level == null || collisions == null || collisions.length == 0) return;
        resolve();
        if (processImpacts == null || getLastPostSnapshots == null || getTickStartVels == null) {
            return;
        }
        try {
            Map<Integer, ?> postSnapshots = (Map<Integer, ?>) getLastPostSnapshots.invoke(null);
            Map<Integer, double[]> tickStart =
                (Map<Integer, double[]>) getTickStartVels.invoke(null);
            SubLevelPhysicsSystem system = SubLevelPhysicsSystem.get(level);
            int substeps = system != null ? system.getConfig().substepsPerTick : 1;
            String levelKey = system != null
                ? system.getLevel().dimension().location().toString()
                : level.dimension().location().toString();
            processImpacts.invoke(
                null,
                collisions,
                level.getGameTime(),
                substeps,
                postSnapshots,
                tickStart,
                levelKey
            );
        } catch (Throwable t) {
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "ERROR stage=standaloneTrueImpactSableBatchBridge type="
                    + t.getClass().getSimpleName()
                    + " message=" + safe(t.getMessage())
            );
            }
        }
    }

    private static synchronized void resolve() {
        if (resolutionAttempted) return;
        resolutionAttempted = true;
        try {
            Class<?> eventBridge = Class.forName(EVENT_BRIDGE);
            Class<?> impactCapture = Class.forName(IMPACT_CAPTURE);
            getLastPostSnapshots = eventBridge.getMethod("getLastPostSnapshots");
            getTickStartVels = eventBridge.getMethod("getTickStartVels");
            processImpacts = impactCapture.getMethod(
                "process",
                double[].class,
                long.class,
                int.class,
                Map.class,
                Map.class,
                String.class
            );
            availabilityLogged = true;
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "STANDALONE_TRUE_IMPACT_COMPAT active=true mode=PMIV_OWNS_SABLE_COLLISION_DRAIN_"
                    + "AND_FORWARDS_SERVER_SUBLEVEL_CONTACTS aircraftTerrainAuthority="
                    + "PMIV_INTEGRATED_TRUE_IMPACT_DERIVED_MODEL"
            );
            }
        } catch (ClassNotFoundException missing) {
            // Standalone True Impact is optional. The integrated PMIV aircraft/world
            // material subsystem remains fully functional without it.
        } catch (Throwable t) {
            if (!availabilityLogged) {
                availabilityLogged = true;
                if (PMIVObserver.loggingEnabled()) {
                    PMIVObserver.log(
                    "ERROR stage=standaloneTrueImpactCompatResolve type="
                        + t.getClass().getSimpleName()
                        + " message=" + safe(t.getMessage())
                );
                }
            }
            getLastPostSnapshots = null;
            getTickStartVels = null;
            processImpacts = null;
        }
    }

    private static String safe(String value) {
        return value == null ? "" : value.replace('\n', ' ').replace('\r', ' ');
    }
}
