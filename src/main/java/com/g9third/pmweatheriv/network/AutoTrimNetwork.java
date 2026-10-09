package com.g9third.pmweatheriv.network;

import com.g9third.pmweatheriv.PMWeatherIV;
import com.g9third.pmweatheriv.physics.AircraftStateAccess;
import com.g9third.pmweatheriv.physics.AircraftState;
import com.g9third.pmweatheriv.physics.AutoTrimController;
import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.UUID;
import java.util.WeakHashMap;
import mcinterface1211.BuilderEntityLinkedSeat;
import mcinterface1211.WrapperPlayer;
import mcinterface1211.WrapperWorld;
import minecrafttransportsimulator.entities.components.AEntityB_Existing;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartSeat;
import net.minecraft.network.RegistryFriendlyByteBuf;
import net.minecraft.network.codec.StreamCodec;
import net.minecraft.network.protocol.common.custom.CustomPacketPayload;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.server.level.ServerPlayer;
import net.minecraft.world.level.Level;
import net.minecraft.world.entity.player.Player;
import net.neoforged.neoforge.network.PacketDistributor;
import net.neoforged.neoforge.network.event.RegisterPayloadHandlersEvent;
import net.neoforged.neoforge.event.entity.player.PlayerEvent;

/** Authenticated current-seat toggle and pilot-only auto-trim status. */
public final class AutoTrimNetwork {
    private static final Map<ServerPlayer, Long> LAST_TOGGLE_REQUEST = new WeakHashMap<>();
    private static final Map<StatusKey, StatusPayload> CLIENT_STATUS = new HashMap<>();
    private static ResourceLocation clientDimension;
    private static Level clientLevel;
    private static long lastClientPruneTick = Long.MIN_VALUE;

    private AutoTrimNetwork() {}

    public static void registerPayloads(RegisterPayloadHandlersEvent event) {
        var registrar = event.registrar("1");
        registrar.playToServer(ToggleRequest.TYPE, ToggleRequest.CODEC, (packet, context) ->
            context.enqueueWork(() -> {
                if (context.player() instanceof ServerPlayer player) handleToggle(player);
            }));
        registrar.playToClient(StatusPayload.TYPE, StatusPayload.CODEC, (packet, context) ->
            context.enqueueWork(() -> acceptStatus(context.player().level(), packet)));
    }

    private static void handleToggle(ServerPlayer player) {
        if (!(player.level() instanceof ServerLevel level)) return;
        long tick = level.getGameTime();
        Long previous = LAST_TOGGLE_REQUEST.get(player);
        if (previous != null && tick >= previous && tick - previous < 4L) return;
        LAST_TOGGLE_REQUEST.put(player, tick);
        PartSeat seat = currentControllerSeat(player);
        if (seat == null) return;
        EntityVehicleF_Physics vehicle = seat.vehicleOn;
        AircraftState state = ((AircraftStateAccess) vehicle).pmweatherIv$getAircraftState();
        if (!state.autoTrim.enabled()
            && (state.plan == null || state.plan.rotorcraft())) {
            state.autoTrim.disable(state.plan == null ? "UNAVAILABLE" : "UNSUPPORTED");
            state.lastAutoTrimStatusState = state.autoTrim.state();
            state.lastAutoTrimStatusReason = state.autoTrim.reason();
            state.lastAutoTrimStatusTick = tick;
            sendStatus(player, vehicle, state, level);
            return;
        }
        state.autoTrim.toggle(player.getUUID(), vehicle.elevatorTrimVar.currentValue);
        if (!state.autoTrim.enabled()) state.nextAutoTrimDamageCheck = Long.MIN_VALUE;
        state.lastAutoTrimStatusState = state.autoTrim.state();
        state.lastAutoTrimStatusReason = state.autoTrim.reason();
        state.lastAutoTrimStatusTick = tick;
        if (PMIVObserver.loggingEnabled()) {
            PMIVObserver.log("PMIV_AUTO_TRIM_TRANSITION vehicleUuid=" + vehicle.uniqueUUID
                + " source=TOGGLE state=" + state.autoTrim.state()
                + " reason=" + state.autoTrim.reason()
                + " currentTrim=" + vehicle.elevatorTrimVar.currentValue
                + " requestedTrim=" + vehicle.elevatorTrimVar.currentValue);
        }
        sendStatus(player, vehicle, state, level);
    }

    /** Revalidates the real linked seat and its current controller on every owner tick. */
    public static boolean isCurrentController(ServerPlayer player, EntityVehicleF_Physics vehicle) {
        PartSeat seat = currentControllerSeat(player);
        if (seat == null || seat.vehicleOn != vehicle) return false;
        AircraftState state = ((AircraftStateAccess) vehicle).pmweatherIv$getAircraftState();
        return state.plan != null && !state.plan.rotorcraft();
    }

    private static PartSeat currentControllerSeat(Player player) {
        if (player == null || !(player.getVehicle() instanceof BuilderEntityLinkedSeat linkedSeat)
            || linkedSeat.level() != player.level() || !linkedSeat.hasPassenger(player)) return null;
        WrapperPlayer wrapper = WrapperPlayer.getWrapperFor(player);
        AEntityB_Existing riding = wrapper.getEntityRiding();
        if (!(riding instanceof PartSeat seat) || !seat.isValid || seat.placementDefinition == null
            || !seat.placementDefinition.isController || seat.vehicleOn == null
            || linkedSeat.entity != seat || seat.world != WrapperWorld.getWrapperFor(player.level())
            || !seat.vehicleOn.isValid || seat.vehicleOn.outOfHealth
            || seat.vehicleOn.definition == null || seat.vehicleOn.definition.motorized == null
            || !seat.vehicleOn.definition.motorized.isAircraft || seat.vehicleOn.definition.motorized.isBlimp
            || !com.g9third.pmweatheriv.sable.SableVehicleManager.supportsVehicle(seat.vehicleOn)
            || seat.rider == null || !seat.rider.equals(wrapper)) return null;
        return seat;
    }

    public static void sendStatus(ServerPlayer player, EntityVehicleF_Physics vehicle,
                                  AircraftState state, ServerLevel level) {
        if (player == null || vehicle == null || state == null || level == null
            || !player.isAlive() || player.level() != level) return;
        AutoTrimController controller = state.autoTrim;
        StatusPayload payload = new StatusPayload(vehicle.uniqueUUID,
            level.dimension().location(), level.getGameTime(),
            vehicle.elevatorTrimVar.currentValue, (byte) controller.state().ordinal(),
            (byte) StatusReason.from(controller.reason()).ordinal());
        if (payload.finite()) PacketDistributor.sendToPlayer(player, payload);
    }

    /** Used when controller authority is revoked by dismount or Sable residency loss. */
    public static void sendStatusToPilot(UUID pilotId, EntityVehicleF_Physics vehicle,
                                         AircraftState state, ServerLevel level) {
        if (pilotId == null || vehicle == null || state == null || level == null) return;
        ServerPlayer player = level.getServer().getPlayerList().getPlayer(pilotId);
        if (player != null) sendStatus(player, vehicle, state, level);
    }

    public static void sendToggleRequest() {
        PacketDistributor.sendToServer(ToggleRequest.INSTANCE);
    }

    public static void onPlayerLoggedOut(PlayerEvent.PlayerLoggedOutEvent event) {
        if (event.getEntity() instanceof ServerPlayer player) LAST_TOGGLE_REQUEST.remove(player);
    }

    public static StatusPayload status(Level level, UUID aircraft) {
        if (level == null || aircraft == null) return null;
        resetClient(level, level.getGameTime());
        ResourceLocation dimension = level.dimension().location();
        StatusPayload payload = CLIENT_STATUS.get(new StatusKey(dimension, aircraft));
        long clientTick = level.getGameTime();
        return payload != null && clientTick + 20L >= payload.tick()
            && payload.tick() + 100L >= clientTick
            ? payload : null;
    }

    public static void resetClient(Level level, long clientTick) {
        ResourceLocation dimension = level == null ? null : level.dimension().location();
        if (clientLevel != level || !java.util.Objects.equals(clientDimension, dimension)) {
            CLIENT_STATUS.clear();
            clientDimension = dimension;
            clientLevel = level;
            lastClientPruneTick = Long.MIN_VALUE;
        }
        if (lastClientPruneTick == Long.MIN_VALUE || clientTick - lastClientPruneTick >= 20L) {
            long now = clientTick;
            Iterator<Map.Entry<StatusKey, StatusPayload>> iterator = CLIENT_STATUS.entrySet().iterator();
            while (iterator.hasNext()) {
                StatusPayload payload = iterator.next().getValue();
                if (!payload.dimension().equals(dimension) || now - payload.tick() > 100L) iterator.remove();
            }
            lastClientPruneTick = clientTick;
        }
    }

    private static void acceptStatus(net.minecraft.world.level.Level level, StatusPayload payload) {
        resetClient(level, level.getGameTime());
        if (!level.dimension().location().equals(payload.dimension()) || !payload.finite()) return;
        StatusKey key = new StatusKey(payload.dimension(), payload.aircraft());
        StatusPayload previous = CLIENT_STATUS.get(key);
        if (previous == null || payload.tick() >= previous.tick()) CLIENT_STATUS.put(key, payload);
    }

    private record StatusKey(ResourceLocation dimension, UUID aircraft) {}

    public enum StatusReason {
        OFF, LEARNING, ON, CONTACT, LOW_SPEED, STALL, HIGH_AOA, BANK, HIGH_RATE,
        PILOT_INPUT, MANUAL_TRIM, TRIM_LIMIT, NO_AUTHORITY, RESPONSE_UNAVAILABLE,
        PHYSICS_HOLD, AUTOPILOT, AIRCRAFT, UNSUPPORTED, UNAVAILABLE, RESTORING_TRIM;

        static StatusReason from(String reason) {
            try {
                return valueOf(reason);
            } catch (IllegalArgumentException | NullPointerException ignored) {
                return UNAVAILABLE;
            }
        }
    }

    public record ToggleRequest() implements CustomPacketPayload {
        public static final ToggleRequest INSTANCE = new ToggleRequest();
        public static final Type<ToggleRequest> TYPE = new Type<>(ResourceLocation.fromNamespaceAndPath(
            PMWeatherIV.MOD_ID, "auto_trim_toggle"));
        public static final StreamCodec<RegistryFriendlyByteBuf, ToggleRequest> CODEC = new StreamCodec<>() {
            @Override public ToggleRequest decode(RegistryFriendlyByteBuf buffer) { return INSTANCE; }
            @Override public void encode(RegistryFriendlyByteBuf buffer, ToggleRequest packet) {}
        };
        @Override public Type<? extends CustomPacketPayload> type() { return TYPE; }
    }

    public record StatusPayload(UUID aircraft, ResourceLocation dimension, long tick,
                               double trim, byte state, byte reason) implements CustomPacketPayload {
        public static final Type<StatusPayload> TYPE = new Type<>(ResourceLocation.fromNamespaceAndPath(
            PMWeatherIV.MOD_ID, "auto_trim_status"));
        public static final StreamCodec<RegistryFriendlyByteBuf, StatusPayload> CODEC = new StreamCodec<>() {
            @Override public StatusPayload decode(RegistryFriendlyByteBuf buffer) {
                return new StatusPayload(buffer.readUUID(), buffer.readResourceLocation(),
                    buffer.readVarLong(), buffer.readDouble(), buffer.readByte(), buffer.readByte());
            }
            @Override public void encode(RegistryFriendlyByteBuf buffer, StatusPayload packet) {
                buffer.writeUUID(packet.aircraft());
                buffer.writeResourceLocation(packet.dimension());
                buffer.writeVarLong(packet.tick());
                buffer.writeDouble(packet.trim());
                buffer.writeByte(packet.state());
                buffer.writeByte(packet.reason());
            }
        };
        public boolean finite() {
            return aircraft != null && dimension != null && tick >= 0L && Double.isFinite(trim)
                && state >= 0 && state < AutoTrimController.State.values().length
                && reason >= 0 && reason < StatusReason.values().length;
        }
        @Override public Type<? extends CustomPacketPayload> type() { return TYPE; }
    }
}
