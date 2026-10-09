package com.g9third.pmweatheriv.network;

import java.lang.reflect.Method;
import mcinterface1211.BuilderEntityLinkedSeat;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartSeat;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.server.level.ServerPlayer;
import net.minecraft.world.phys.Vec3;
import net.neoforged.fml.ModList;

/** Thin PMIV bridge: PMAero owns world/plot sampling; IV supplies the optional seat heading. */
public final class WindSamplePosition {
    private static volatile Method pmaeroWorldPoint;
    private static volatile Method pmaeroExposedPoint;
    private static volatile boolean pmaeroMethodsResolved;

    private WindSamplePosition() {}

    /** Rider location and fixed world-space vehicle nose direction. */
    public record RiderSample(Vec3 worldPoint, Vec3 forward) {}

    public static RiderSample resolve(ServerPlayer player) {
        return new RiderSample(worldPoint(player), vehicleForward(player));
    }

    public static Vec3 worldPoint(ServerPlayer player) {
        resolvePMAeroMethods();
        Vec3 shared = invokePoint(pmaeroWorldPoint, player);
        return shared != null ? shared : player.getEyePosition();
    }

    /** Called only by the PMIV HUD server sampler; actual atmosphere sampling remains PMAero-owned. */
    public static Vec3 exposedWorldPoint(ServerLevel level, ServerPlayer player) {
        resolvePMAeroMethods();
        Vec3 shared = invokePoint(pmaeroExposedPoint, level, player);
        return shared != null ? shared : worldPoint(player);
    }

    /** Optional world-space IV heading; PMAero resolves generic sub-level headings otherwise. */
    public static Vec3 vehicleForward(ServerPlayer player) {
        try {
            if (player.getVehicle() instanceof BuilderEntityLinkedSeat linkedSeat
                && linkedSeat.entity instanceof PartSeat seat) {
                EntityVehicleF_Physics vehicle = seat.vehicleOn;
                if (vehicle != null && vehicle.orientation != null
                    && Double.isFinite(vehicle.orientation.angles.y)) {
                    double yaw = Math.toRadians(-vehicle.orientation.angles.y);
                    // SableVehicleBody exposes the physics orientation in world space.
                    // Applying the sub-level pose here would rotate it a second time.
                    return new Vec3(-Math.sin(yaw), 0.0, Math.cos(yaw));
                }
            }
        } catch (RuntimeException | LinkageError ignored) {
            // A seat can be replaced while the rider relationship is resolving.
        }
        return null;
    }

    private static void resolvePMAeroMethods() {
        if (pmaeroMethodsResolved) return;
        synchronized (WindSamplePosition.class) {
            if (pmaeroMethodsResolved) return;
            pmaeroMethodsResolved = true;
            try {
                if (!ModList.get().isLoaded("pmweather_aeronautics")) return;
                Class<?> helper = Class.forName("com.axes.pmweather_aeronautics.WindSamplePosition", false,
                    WindSamplePosition.class.getClassLoader());
                pmaeroWorldPoint = helper.getMethod("worldPoint", ServerPlayer.class);
                pmaeroExposedPoint = helper.getMethod("exposedWorldPoint", ServerLevel.class, ServerPlayer.class);
            } catch (ReflectiveOperationException | RuntimeException | LinkageError ignored) {
                pmaeroWorldPoint = null;
                pmaeroExposedPoint = null;
            }
        }
    }

    private static Vec3 invokePoint(Method method, Object... arguments) {
        if (method == null) return null;
        try {
            Object result = method.invoke(null, arguments);
            return result instanceof Vec3 point && finite(point) ? point : null;
        } catch (ReflectiveOperationException | RuntimeException | LinkageError ignored) {
            return null;
        }
    }

    private static boolean finite(Vec3 point) {
        return point != null && Double.isFinite(point.x) && Double.isFinite(point.y) && Double.isFinite(point.z);
    }
}
