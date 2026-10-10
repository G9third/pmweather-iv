package com.g9third.pmweatheriv.network;

import com.g9third.pmweatheriv.PMWeatherIV;
import com.g9third.pmweatheriv.physics.Vec3d;
import com.g9third.pmweatheriv.compat.PMAeroBridge;
import com.mojang.logging.LogUtils;
import net.minecraft.network.RegistryFriendlyByteBuf;
import net.minecraft.network.codec.StreamCodec;
import net.minecraft.network.protocol.common.custom.CustomPacketPayload;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.server.level.ServerPlayer;
import net.minecraft.server.level.ServerLevel;
import net.neoforged.neoforge.network.event.RegisterPayloadHandlersEvent;
import net.neoforged.neoforge.network.handling.IPayloadContext;
import org.slf4j.Logger;

/**
 * Tiny optional request/reply channel for the live wind HUD.
 *
 * The HUD intentionally does not calculate its own weather when the server supports this
 * channel. The logical server samples the required PMAero wind-vector API,
 * then returns only the resulting 3-D vector. This keeps the graphic consistent with the
 * PMWeather field used by aircraft physics and also works when PMWeather itself is server-only.
 */
public final class WindMonitorNetwork {
    private static final Logger LOGGER = LogUtils.getLogger();
    private static final java.util.Map<ServerPlayer, CachedReading> SERVER = new java.util.WeakHashMap<>();
    private static final java.util.Map<ServerPlayer, SamplingFailure> FAILURES = new java.util.WeakHashMap<>();
    private record CachedReading(ServerLevel level, long tick, Vec3d wind, boolean valid) {}
    private record SamplingFailure(ServerLevel level) {}
    public static void clearServer() {
        SERVER.clear();
        FAILURES.clear();
    }
    private static volatile Reading latestClientReading = Reading.UNAVAILABLE;

    private WindMonitorNetwork() {
    }

    public static void registerPayloads(RegisterPayloadHandlersEvent event) {
        var registrar = event.registrar("1").optional();
        registrar.playToServer(
            RequestPayload.TYPE,
            RequestPayload.STREAM_CODEC,
            WindMonitorNetwork::handleRequest
        );
        registrar.playToClient(
            ReadingPayload.TYPE,
            ReadingPayload.STREAM_CODEC,
            WindMonitorNetwork::handleReading
        );
    }

    private static void handleRequest(RequestPayload request, IPayloadContext context) {
        if (!(context.player() instanceof ServerPlayer player)) return;
        long tick = player.level().getGameTime();
        CachedReading cached = SERVER.get(player);
        ServerLevel level = player.serverLevel();
        if (cached == null || cached.level() != level || tick - cached.tick() >= 2L
            || tick < cached.tick()) {
            double[] output = new double[PMAeroBridge.VECTOR_STRIDE];
            boolean valid = true;
            Vec3d wind = Vec3d.ZERO;
            try {
                net.minecraft.world.phys.Vec3 samplePoint = WindSamplePosition.exposedWorldPoint(level, player);
                if (samplePoint == null || !Double.isFinite(samplePoint.x)
                    || !Double.isFinite(samplePoint.y) || !Double.isFinite(samplePoint.z)) {
                    throw new IllegalStateException("PMAero wind sample position is unavailable");
                }
                double[] xyz = {samplePoint.x, samplePoint.y, samplePoint.z};
                PMAeroBridge.sampleAircraftWindInto(level, xyz, output);
                wind = new Vec3d(output[0], output[1], output[2]);
                FAILURES.remove(player);
            } catch (Throwable failure) {
                if (failure instanceof VirtualMachineError error) throw error;
                if (failure instanceof ThreadDeath death) throw death;
                valid = false;
                logSamplingFailure(player, level, failure);
            }
            cached = new CachedReading(level, tick,
                wind, valid);
            SERVER.put(player, cached);
        }
        context.reply(new ReadingPayload(request.requestId(), tick,
            cached.wind().x(), cached.wind().y(), cached.wind().z(), cached.valid()));
    }

    public static void onServerStopping(net.neoforged.neoforge.event.server.ServerStoppingEvent event) {
        clearServer();
    }

    private static void logSamplingFailure(ServerPlayer player, ServerLevel level, Throwable failure) {
        SamplingFailure previous = FAILURES.get(player);
        if (previous != null && previous.level() == level) return;
        FAILURES.put(player, new SamplingFailure(level));
        LOGGER.warn("PMIV wind HUD sampling failed for player {} ({}) in {}; suppressing repeats until recovery.",
            player.getGameProfile().getName(), player.getUUID(), level.dimension().location(), failure);
    }

    private static void handleReading(ReadingPayload payload, IPayloadContext context) {
        Vec3d wind = new Vec3d(payload.windX(), payload.windY(), payload.windZ());
        latestClientReading = new Reading(
            payload.pmweatherPresent() && wind.isFinite(),
            payload.requestId(),
            payload.serverGameTime(),
            wind.isFinite() ? wind : Vec3d.ZERO,
            payload.pmweatherPresent()
        );
    }

    public static Reading latestClientReading() {
        return latestClientReading;
    }

    public static void clearClientReading() {
        latestClientReading = Reading.UNAVAILABLE;
    }

    public record Reading(
        boolean available,
        long requestId,
        long serverGameTime,
        Vec3d windMph,
        boolean pmweatherPresent
    ) {
        public static final Reading UNAVAILABLE = new Reading(false, Long.MIN_VALUE, Long.MIN_VALUE, Vec3d.ZERO, false);
    }

    public record RequestPayload(long requestId) implements CustomPacketPayload {
        public static final Type<RequestPayload> TYPE = new Type<>(
            ResourceLocation.fromNamespaceAndPath(PMWeatherIV.MOD_ID, "wind_monitor_request")
        );
        public static final StreamCodec<RegistryFriendlyByteBuf, RequestPayload> STREAM_CODEC = new StreamCodec<>() {
            @Override
            public RequestPayload decode(RegistryFriendlyByteBuf buffer) {
                return new RequestPayload(buffer.readVarLong());
            }

            @Override
            public void encode(RegistryFriendlyByteBuf buffer, RequestPayload payload) {
                buffer.writeVarLong(payload.requestId());
            }
        };

        @Override
        public Type<? extends CustomPacketPayload> type() {
            return TYPE;
        }
    }

    public record ReadingPayload(
        long requestId,
        long serverGameTime,
        double windX,
        double windY,
        double windZ,
        boolean pmweatherPresent
    ) implements CustomPacketPayload {
        public static final Type<ReadingPayload> TYPE = new Type<>(
            ResourceLocation.fromNamespaceAndPath(PMWeatherIV.MOD_ID, "wind_monitor_reading")
        );
        public static final StreamCodec<RegistryFriendlyByteBuf, ReadingPayload> STREAM_CODEC = new StreamCodec<>() {
            @Override
            public ReadingPayload decode(RegistryFriendlyByteBuf buffer) {
                return new ReadingPayload(
                    buffer.readVarLong(),
                    buffer.readVarLong(),
                    buffer.readDouble(),
                    buffer.readDouble(),
                    buffer.readDouble(),
                    buffer.readBoolean()
                );
            }

            @Override
            public void encode(RegistryFriendlyByteBuf buffer, ReadingPayload payload) {
                buffer.writeVarLong(payload.requestId());
                buffer.writeVarLong(payload.serverGameTime());
                buffer.writeDouble(payload.windX());
                buffer.writeDouble(payload.windY());
                buffer.writeDouble(payload.windZ());
                buffer.writeBoolean(payload.pmweatherPresent());
            }
        };

        @Override
        public Type<? extends CustomPacketPayload> type() {
            return TYPE;
        }
    }
}
