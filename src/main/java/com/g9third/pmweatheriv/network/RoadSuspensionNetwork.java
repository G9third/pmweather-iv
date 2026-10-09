package com.g9third.pmweatheriv.network;

import com.g9third.pmweatheriv.PMWeatherIV;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import net.minecraft.network.RegistryFriendlyByteBuf;
import net.minecraft.network.codec.StreamCodec;
import net.minecraft.network.protocol.common.custom.CustomPacketPayload;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.Level;
import net.neoforged.neoforge.network.PacketDistributor;
import net.neoforged.neoforge.network.event.RegisterPayloadHandlersEvent;
import net.neoforged.neoforge.event.server.ServerStoppingEvent;

/** Server-owned road-wheel travel, synchronized by authored part identity. */
public final class RoadSuspensionNetwork {
    private static final int MAX_PARTS_PER_PACKET = 512;
    private static final double SEND_RANGE_SQUARED = 128.0 * 128.0;
    private static final Map<UUID, ServerSnapshot> LAST_SENT = new HashMap<>();
    private static final Map<VehicleKey, SuspensionPayload> CLIENT = new HashMap<>();
    private static Level clientLevel;
    private static ResourceLocation clientDimension;
    private static long lastPruneTick = Long.MIN_VALUE;

    private RoadSuspensionNetwork() {}

    public static void registerPayloads(RegisterPayloadHandlersEvent event) {
        event.registrar("1").playToClient(SuspensionPayload.TYPE, SuspensionPayload.CODEC,
            (packet, context) -> context.enqueueWork(() -> accept(context.player().level(), packet)));
    }

    /** Publish only active/nonzero travel, and publish a single clearing edge when it returns to zero. */
    public static void publish(EntityVehicleF_Physics vehicle, ServerLevel level, long tick,
                               Map<UUID, Double> offsets) {
        if (vehicle == null || level == null || vehicle.uniqueUUID == null || offsets == null) return;
        Map<UUID, Double> clean = new HashMap<>();
        for (Map.Entry<UUID, Double> entry : offsets.entrySet()) {
            UUID part = entry.getKey();
            Double value = entry.getValue();
            if (part != null && value != null && Double.isFinite(value)
                && Math.abs(value) <= 0.5) clean.put(part, value);
        }
        boolean hasTravel = clean.values().stream().anyMatch(value -> Math.abs(value) > 1.0E-5);
        ServerSnapshot previous = LAST_SENT.get(vehicle.uniqueUUID);
        boolean hadTravel = previous != null
            && previous.offsets().values().stream().anyMatch(value -> Math.abs(value) > 1.0E-5);
        boolean changed = previous == null
            || !previous.dimension().equals(level.dimension().location())
            || !sameOffsets(previous.offsets(), clean)
            || tick - previous.tick() >= 5L;
        if (!hasTravel && !hadTravel) return;
        if (!hasTravel && !changed) return;

        List<PartOffset> parts = new ArrayList<>(clean.size());
        clean.forEach((uuid, travel) -> parts.add(new PartOffset(uuid, travel)));
        parts.sort(java.util.Comparator.comparing(value -> value.part().toString()));
        SuspensionPayload payload = new SuspensionPayload(vehicle.uniqueUUID,
            level.dimension().location(), tick, parts);
        if (!payload.finite()) return;
        for (var player : level.players()) {
            double dx = player.getX() - vehicle.position.x;
            double dy = player.getY() - vehicle.position.y;
            double dz = player.getZ() - vehicle.position.z;
            if (dx * dx + dy * dy + dz * dz <= SEND_RANGE_SQUARED) {
                PacketDistributor.sendToPlayer(player, payload);
            }
        }
        if (hasTravel) {
            LAST_SENT.put(vehicle.uniqueUUID,
                new ServerSnapshot(level.dimension().location(), tick, Map.copyOf(clean)));
        } else {
            LAST_SENT.remove(vehicle.uniqueUUID);
        }
    }

    /** Remove stale server bookkeeping when the managed body leaves Sable authority. */
    public static void forget(ServerLevel level, UUID vehicle) {
        if (vehicle == null) return;
        ServerSnapshot previous = LAST_SENT.remove(vehicle);
        if (level == null || previous == null) return;
        if (previous.offsets().values().stream().noneMatch(value -> Math.abs(value) > 1.0E-5)) return;
        SuspensionPayload clear = new SuspensionPayload(vehicle,
            level.dimension().location(), level.getGameTime(), List.of());
        if (clear.finite()) for (var player : level.players()) PacketDistributor.sendToPlayer(player, clear);
    }

    public static void onServerStopping(ServerStoppingEvent event) {
        LAST_SENT.clear();
    }

    /** Latest server travel for one real master part; absent or stale state means native position. */
    public static double clientTravel(Level level, UUID vehicle, UUID part) {
        if (level == null || vehicle == null || part == null) return 0.0;
        resetClient(level);
        SuspensionPayload payload = CLIENT.get(new VehicleKey(level.dimension().location(), vehicle));
        if (payload == null || level.getGameTime() - payload.tick() > 10L) return 0.0;
        for (PartOffset offset : payload.parts()) if (part.equals(offset.part())) return offset.travelMeters();
        return 0.0;
    }

    public static void resetClient(Level level) {
        ResourceLocation dimension = level == null ? null : level.dimension().location();
        if (clientLevel != level || !java.util.Objects.equals(clientDimension, dimension)) {
            CLIENT.clear();
            clientLevel = level;
            clientDimension = dimension;
            lastPruneTick = Long.MIN_VALUE;
        }
        if (level != null && (lastPruneTick == Long.MIN_VALUE
            || level.getGameTime() - lastPruneTick >= 20L)) {
            long now = level.getGameTime();
            CLIENT.values().removeIf(payload -> !payload.dimension().equals(dimension)
                || now - payload.tick() > 10L);
            lastPruneTick = now;
        }
    }

    private static void accept(Level level, SuspensionPayload payload) {
        if (level == null || payload == null) return;
        resetClient(level);
        if (!level.dimension().location().equals(payload.dimension()) || !payload.finite()) return;
        VehicleKey key = new VehicleKey(payload.dimension(), payload.vehicle());
        SuspensionPayload previous = CLIENT.get(key);
        if (previous == null || payload.tick() >= previous.tick()) CLIENT.put(key, payload);
    }

    private static boolean sameOffsets(Map<UUID, Double> left, Map<UUID, Double> right) {
        if (!left.keySet().equals(right.keySet())) return false;
        for (UUID uuid : left.keySet()) {
            if (Math.abs(left.get(uuid) - right.get(uuid)) > 1.0E-5) return false;
        }
        return true;
    }

    private record VehicleKey(ResourceLocation dimension, UUID vehicle) {}
    private record ServerSnapshot(ResourceLocation dimension, long tick, Map<UUID, Double> offsets) {}

    public record PartOffset(UUID part, double travelMeters) {}

    public record SuspensionPayload(UUID vehicle, ResourceLocation dimension, long tick,
                                    List<PartOffset> parts) implements CustomPacketPayload {
        public static final Type<SuspensionPayload> TYPE = new Type<>(ResourceLocation.fromNamespaceAndPath(
            PMWeatherIV.MOD_ID, "road_suspension"));
        public static final StreamCodec<RegistryFriendlyByteBuf, SuspensionPayload> CODEC = new StreamCodec<>() {
            @Override public SuspensionPayload decode(RegistryFriendlyByteBuf buffer) {
                UUID vehicle = buffer.readUUID();
                ResourceLocation dimension = buffer.readResourceLocation();
                long tick = buffer.readVarLong();
                int count = buffer.readVarInt();
                if (count < 0 || count > MAX_PARTS_PER_PACKET) throw new IllegalArgumentException("Invalid suspension part count");
                List<PartOffset> parts = new ArrayList<>(count);
                for (int index = 0; index < count; ++index) {
                    parts.add(new PartOffset(buffer.readUUID(), buffer.readDouble()));
                }
                return new SuspensionPayload(vehicle, dimension, tick, List.copyOf(parts));
            }
            @Override public void encode(RegistryFriendlyByteBuf buffer, SuspensionPayload payload) {
                buffer.writeUUID(payload.vehicle());
                buffer.writeResourceLocation(payload.dimension());
                buffer.writeVarLong(payload.tick());
                buffer.writeVarInt(payload.parts().size());
                for (PartOffset part : payload.parts()) {
                    buffer.writeUUID(part.part());
                    buffer.writeDouble(part.travelMeters());
                }
            }
        };
        public boolean finite() {
            if (vehicle == null || dimension == null || tick < 0L || parts == null
                || parts.size() > MAX_PARTS_PER_PACKET) return false;
            for (PartOffset part : parts) {
                if (part == null || part.part() == null || !Double.isFinite(part.travelMeters())
                    || Math.abs(part.travelMeters()) > 0.5) return false;
            }
            return true;
        }
        @Override public Type<? extends CustomPacketPayload> type() { return TYPE; }
    }
}
