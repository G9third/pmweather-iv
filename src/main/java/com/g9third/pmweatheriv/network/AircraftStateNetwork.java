package com.g9third.pmweatheriv.network;

import com.g9third.pmweatheriv.PMWeatherIV;
import com.g9third.pmweatheriv.mixin.EntityVehiclePhysicsAccessor;
import com.g9third.pmweatheriv.mixin.EntityVehicleMovingAccessor;
import com.g9third.pmweatheriv.physics.AircraftState;
import com.g9third.pmweatheriv.physics.AirframeLoads;
import com.g9third.pmweatheriv.physics.AirWarningSystem;
import com.g9third.pmweatheriv.physics.FlightMath;
import com.g9third.pmweatheriv.physics.Vec3d;
import java.util.HashMap;
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

/** Physical state and absolute origins for IV's cumulative client movement stream. */
public final class AircraftStateNetwork {
    private static final Map<UUID, StatePayload> CLIENT = new HashMap<>();
    private static final Map<UUID, PoseAnchorPayload> CLIENT_POSE_ANCHORS = new HashMap<>();
    private static Level clientLevel;
    private static long lastPruneTick = Long.MIN_VALUE;
    private AircraftStateNetwork() {}

    public static void registerPayloads(RegisterPayloadHandlersEvent event) {
        var registrar = event.registrar("5");
        registrar.playToClient(StatePayload.TYPE, StatePayload.CODEC, (packet, context) -> {
            Level level = context.player().level();
            resetClientLevel(level);
            if (!level.dimension().location().equals(packet.dimension()) || !packet.finite()) return;
            StatePayload previous = CLIENT.get(packet.uuid());
            if (previous == null || packet.tick() >= previous.tick()) CLIENT.put(packet.uuid(), packet);
        });
        registrar.playToClient(PoseAnchorPayload.TYPE, PoseAnchorPayload.CODEC, (packet, context) -> {
            Level level = context.player().level();
            resetClientLevel(level);
            if (!level.dimension().location().equals(packet.dimension()) || !packet.anchor().finite()) return;
            PoseAnchorPayload previous = CLIENT_POSE_ANCHORS.get(packet.uuid());
            if (previous == null || packet.tick() >= previous.tick()) CLIENT_POSE_ANCHORS.put(packet.uuid(), packet);
        });
    }

    public static void resetClientLevel(Level level) {
        if (clientLevel != level) { CLIENT.clear(); CLIENT_POSE_ANCHORS.clear(); clientLevel = level; lastPruneTick = Long.MIN_VALUE; }
        if (level != null && (lastPruneTick == Long.MIN_VALUE || level.getGameTime() - lastPruneTick >= 20L)) {
            CLIENT.values().removeIf(p -> level.getGameTime() - p.tick() > 100L);
            CLIENT_POSE_ANCHORS.values().removeIf(p -> level.getGameTime() - p.tick() > 100L);
            lastPruneTick = level.getGameTime();
        }
    }

    public static boolean managed(EntityVehicleF_Physics vehicle) {
        StatePayload p = CLIENT.get(vehicle.uniqueUUID);
        // During initial synchronization wait for the server, without running a local integrator.
        return p == null || p.managed();
    }

    public static void apply(EntityVehicleF_Physics vehicle) {
        StatePayload p = CLIENT.get(vehicle.uniqueUUID);
        if (p == null || !p.managed()) return;
        double scale = Math.max(1E-6, vehicle.speedFactor * 20.0);
        vehicle.motion.set(p.vx() / scale, p.vy() / scale, p.vz() / scale);
        Vec3d degrees = FlightMath.degreesPerTickFromOmega(new Vec3d(p.wx(), p.wy(), p.wz()));
        vehicle.rotation.angles.set(degrees.x(), degrees.y(), degrees.z());
        vehicle.velocity = vehicle.motion.length();
        vehicle.axialVelocity = p.axialVelocity();
        vehicle.indicatedSpeed = p.airspeed();
        vehicle.airDensity = p.density();
        EntityVehiclePhysicsAccessor access = (EntityVehiclePhysicsAccessor) vehicle;
        access.pmweatherIv$setTrackAngle(p.trackAngle());
        access.pmweatherIv$setHasRotors(p.rotor());
        access.pmweatherIv$setThrustForceValue(p.thrust());
    }

    /** Uses the same delta stream as IV; the anchor repairs an initial/reconnect pose mismatch. */
    public static CumulativePoseAnchor poseAnchor(EntityVehicleF_Physics vehicle) {
        PoseAnchorPayload p = CLIENT_POSE_ANCHORS.get(vehicle.uniqueUUID);
        StatePayload state = CLIENT.get(vehicle.uniqueUUID);
        if (p == null || state == null || !state.managed()) return null;
        if (clientLevel == null || clientLevel.getGameTime() - p.tick() > 100L) return null;
        return p.anchor();
    }

    /** Publish after IV updates its cumulative deltas, so pose and counters share one boundary. */
    public static void publishPoseAnchor(EntityVehicleF_Physics vehicle, ServerLevel level) {
        EntityVehicleMovingAccessor access = (EntityVehicleMovingAccessor) vehicle;
        var movement = access.pmweatherIv$getServerDeltaM();
        var rotation = access.pmweatherIv$getServerDeltaR();
        var angles = vehicle.orientation.angles;
        CumulativePoseAnchor anchor = CumulativePoseAnchor.capture(
            new Vec3d(vehicle.position.x, vehicle.position.y, vehicle.position.z),
            new Vec3d(angles.x, angles.y, angles.z),
            new Vec3d(movement.x, movement.y, movement.z),
            new Vec3d(rotation.x, rotation.y, rotation.z));
        if (!anchor.finite()) return;
        PoseAnchorPayload packet = new PoseAnchorPayload(vehicle.uniqueUUID,
            level.dimension().location(), level.getGameTime(), anchor);
        for (var player : level.players()) PacketDistributor.sendToPlayer(player, packet);
    }

    public record PoseAnchorPayload(UUID uuid, ResourceLocation dimension, long tick,
                                   CumulativePoseAnchor anchor) implements CustomPacketPayload {
        public static final Type<PoseAnchorPayload> TYPE = new Type<>(ResourceLocation.fromNamespaceAndPath(
            PMWeatherIV.MOD_ID, "vehicle_pose_anchor"));
        public static final StreamCodec<RegistryFriendlyByteBuf, PoseAnchorPayload> CODEC = new StreamCodec<>() {
            public PoseAnchorPayload decode(RegistryFriendlyByteBuf b) {
                return new PoseAnchorPayload(b.readUUID(), b.readResourceLocation(), b.readVarLong(),
                    new CumulativePoseAnchor(new Vec3d(b.readDouble(), b.readDouble(), b.readDouble()),
                        new Vec3d(b.readDouble(), b.readDouble(), b.readDouble())));
            }
            public void encode(RegistryFriendlyByteBuf b, PoseAnchorPayload p) {
                b.writeUUID(p.uuid()); b.writeResourceLocation(p.dimension()); b.writeVarLong(p.tick());
                b.writeDouble(p.anchor().positionOrigin().x()); b.writeDouble(p.anchor().positionOrigin().y());
                b.writeDouble(p.anchor().positionOrigin().z()); b.writeDouble(p.anchor().angleOrigin().x());
                b.writeDouble(p.anchor().angleOrigin().y()); b.writeDouble(p.anchor().angleOrigin().z());
            }
        };
        public Type<? extends CustomPacketPayload> type() { return TYPE; }
    }

    /** Air-warning values synchronized to the IV client sound/animation system. */
    public static WarningView warningView(EntityVehicleF_Physics vehicle) {
        if (vehicle == null) return null;
        StatePayload p = CLIENT.get(vehicle.uniqueUUID);
        Level level = clientLevel;
        if (p == null || !p.managed()) return null;
        if (level != null && level.getGameTime() - p.tick() > 5L) return null;
        return new WarningView(
            p.windShearWarning(), p.windShearLevel(),
            p.windShearDeltaMps(), p.windShearVerticalDeltaMps(),
            p.windSpeedMps(), p.verticalWindMps()
        );
    }

    public record WarningView(
        boolean windShearWarning,
        double windShearLevel,
        double windShearDeltaMps,
        double windShearVerticalDeltaMps,
        double windSpeedMps,
        double verticalWindMps
    ) {
    }

    public static void publish(EntityVehicleF_Physics vehicle, ServerLevel level,
                               AircraftState state, AirframeLoads.SolveResult result, boolean managed) {
        Vec3d v = state.hasPhysicalVelocity ? state.originVelocityWorld : Vec3d.ZERO;
        Vec3d w = state.angularVelocityBody;
        double thrust = 0.0;
        if (result != null) for (var p : result.propulsionLoads()) thrust += p.mtsForceValue();
        if (managed && !vehicle.definition.motorized.isAircraft)
            thrust = ((EntityVehiclePhysicsAccessor) vehicle).pmweatherIv$getThrustForceValue();
        AirWarningSystem.Snapshot warning = managed
            ? state.airWarnings.snapshot() : AirWarningSystem.Snapshot.ZERO;
        StatePayload packet = new StatePayload(vehicle.uniqueUUID, level.dimension().location(),
            level.getGameTime(), managed, v.x(), v.y(), v.z(), w.x(), w.y(), w.z(),
            result == null ? 0.0 : Math.abs(result.forwardAirspeed()),
            vehicle.axialVelocity,
            result == null ? 1.225 : result.airDensity(), result == null ? 0.0 : result.trackAngleDegrees(),
            result != null && result.hasRotor(), thrust,
            warning.windShearWarning(), warning.windShearLevel(),
            warning.windShearDeltaMps(), warning.windShearVerticalDeltaMps(),
            warning.windSpeedMps(), warning.verticalWindMps());
        if (packet.finite()) for (var player : level.players()) PacketDistributor.sendToPlayer(player, packet);
    }

    /** Airspeed is m/s; axialVelocity retains the authoritative IV variable's internal units. */
    public record StatePayload(UUID uuid, ResourceLocation dimension, long tick, boolean managed,
                               double vx, double vy, double vz, double wx, double wy, double wz,
                               double airspeed, double axialVelocity, double density, double trackAngle, boolean rotor,
                               double thrust, boolean windShearWarning, double windShearLevel,
                               double windShearDeltaMps, double windShearVerticalDeltaMps,
                               double windSpeedMps, double verticalWindMps)
                               implements CustomPacketPayload {
        public static final Type<StatePayload> TYPE = new Type<>(ResourceLocation.fromNamespaceAndPath(
            PMWeatherIV.MOD_ID, "aircraft_physical_state"));
        public static final StreamCodec<RegistryFriendlyByteBuf, StatePayload> CODEC = new StreamCodec<>() {
            public StatePayload decode(RegistryFriendlyByteBuf b) {
                return new StatePayload(b.readUUID(), b.readResourceLocation(), b.readVarLong(), b.readBoolean(),
                    b.readDouble(), b.readDouble(), b.readDouble(), b.readDouble(), b.readDouble(), b.readDouble(),
                    b.readDouble(), b.readDouble(), b.readDouble(), b.readDouble(), b.readBoolean(), b.readDouble(),
                    b.readBoolean(), b.readDouble(), b.readDouble(), b.readDouble(),
                    b.readDouble(), b.readDouble());
            }
            public void encode(RegistryFriendlyByteBuf b, StatePayload p) {
                b.writeUUID(p.uuid()); b.writeResourceLocation(p.dimension()); b.writeVarLong(p.tick());
                b.writeBoolean(p.managed()); b.writeDouble(p.vx()); b.writeDouble(p.vy()); b.writeDouble(p.vz());
                b.writeDouble(p.wx()); b.writeDouble(p.wy()); b.writeDouble(p.wz()); b.writeDouble(p.airspeed());
                b.writeDouble(p.axialVelocity());
                b.writeDouble(p.density()); b.writeDouble(p.trackAngle()); b.writeBoolean(p.rotor());
                b.writeDouble(p.thrust()); b.writeBoolean(p.windShearWarning());
                b.writeDouble(p.windShearLevel()); b.writeDouble(p.windShearDeltaMps());
                b.writeDouble(p.windShearVerticalDeltaMps()); b.writeDouble(p.windSpeedMps());
                b.writeDouble(p.verticalWindMps());
            }
        };
        boolean finite() {
            return Double.isFinite(vx) && Double.isFinite(vy) && Double.isFinite(vz)
                && Double.isFinite(wx) && Double.isFinite(wy) && Double.isFinite(wz)
                && Double.isFinite(airspeed) && Double.isFinite(axialVelocity) && Double.isFinite(density)
                && Double.isFinite(trackAngle) && Double.isFinite(thrust)
                && Double.isFinite(windShearLevel) && Double.isFinite(windShearDeltaMps)
                && Double.isFinite(windShearVerticalDeltaMps) && Double.isFinite(windSpeedMps)
                && Double.isFinite(verticalWindMps);
        }
        public Type<? extends CustomPacketPayload> type() { return TYPE; }
    }
}
