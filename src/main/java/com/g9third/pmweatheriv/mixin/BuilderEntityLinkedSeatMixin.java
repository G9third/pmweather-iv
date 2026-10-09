package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import com.g9third.pmweatheriv.network.AircraftStateNetwork;
import com.g9third.pmweatheriv.sable.SableVehicleManager;
import java.util.List;
import java.util.UUID;
import mcinterface1211.BuilderEntityLinkedSeat;
import mcinterface1211.WrapperEntity;
import mcinterface1211.WrapperWorld;
import com.g9third.pmweatheriv.network.LinkedSeatMountAccess;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.entities.components.AEntityB_Existing;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartSeat;
import minecrafttransportsimulator.mcinterface.IWrapperEntity;
import minecrafttransportsimulator.mcinterface.InterfaceManager;
import minecrafttransportsimulator.packets.instances.PacketEntityRiderChange;
import net.minecraft.world.entity.Entity;
import net.minecraft.nbt.CompoundTag;
import net.minecraft.server.level.ServerPlayer;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

/**
 * Repairs IV's saved linked-seat rider relationship after a world/server reconnect.
 *
 * <p>Vanilla keeps the real player mounted to {@link BuilderEntityLinkedSeat}, while
 * IV separately keeps a wrapper on the PartSeat.  IV 24's reload path may temporarily
 * trust an old wrapper instead of the currently mounted vanilla passenger.  The result
 * is a perfectly valid Minecraft mount attached to an invisible seat proxy whose IV
 * rider transform is updating a stale player object.  At aircraft speeds that leaves
 * the real player floating behind the aircraft while the HUD still says to dismount.</p>
 *
 * <p>This mixin only reconciles those two identities for PMIV-managed vehicles. It
 * does not modify normal mounting, seat controls, aircraft physics, or pack data.</p>
 */
@Mixin(value = BuilderEntityLinkedSeat.class, remap = false)
public abstract class BuilderEntityLinkedSeatMixin implements LinkedSeatMountAccess {
    @Unique private static final int PMWEATHER_IV_RECONNECT_STABLE_TICKS = 3;
    @Unique private static final int PMWEATHER_IV_RECONNECT_MAX_HOLD_TICKS = 20;

    @Shadow public AEntityB_Existing entity;
    @Shadow protected WrapperEntity rider;
    @Shadow private boolean dismountedRider;
    @Shadow private int ticksWithoutRider;
    @Shadow private UUID entityUUID;

    @Unique private boolean pmweatherIv$reconnectRepairActive;
    @Unique private int pmweatherIv$reconnectStableTicks;
    @Unique private int pmweatherIv$reconnectSessionTicks;
    @Unique private UUID pmweatherIv$reconnectPlayerUuid;
    @Unique private EntityVehicleF_Physics pmweatherIv$reconnectVehicle;
    @Unique private boolean pmweatherIv$sableHoldActive;
    @Unique private boolean pmweatherIv$mountRecoveryLogged;

    @Override
    public void pmweatherIv$prepareMountedPlayer(ServerPlayer player) {
        BuilderEntityLinkedSeat linkedSeat = (BuilderEntityLinkedSeat) (Object) this;
        if (!PMWeatherIVConfig.get().enabled() || linkedSeat.level().isClientSide
            || linkedSeat.isRemoved() || player == null
            || player.level() != linkedSeat.level() || entity != null) {
            return;
        }

        List<Entity> passengers = linkedSeat.getPassengers();
        if (passengers.size() != 1 || passengers.get(0) != player
            || !pmweatherIv$finitePosition(player.getX(), player.getY(), player.getZ())) {
            return;
        }
        pmweatherIv$consumeUnparsedSeatLink();
        WrapperEntity actualRider = WrapperEntity.getWrapperFor(player);
        if (actualRider == null || !pmweatherIv$valid(actualRider)) {
            pmweatherIv$anchorProxyToPlayer(linkedSeat, player);
            return;
        }

        if (entityUUID != null) {
            minecrafttransportsimulator.entities.components.AEntityA_Base resolved =
                WrapperWorld.getWrapperFor(linkedSeat.level()).getEntity(entityUUID);
            if (resolved instanceof PartSeat seat && pmweatherIv$validSeatForMountedPlayer(
                    seat, actualRider, linkedSeat)) {
                double gap = pmweatherIv$distance(linkedSeat.getX(), linkedSeat.getY(), linkedSeat.getZ(),
                    seat.position.x, seat.position.y, seat.position.z);
                entity = seat;
                entityUUID = seat.uniqueUUID;
                linkedSeat.setPos(seat.position.x, seat.position.y, seat.position.z);
                pmweatherIv$logMountRecovery(linkedSeat, "UUID_RESOLVED", gap);
                return;
            }
        }

        // The seat may load later through ordinary player/vehicle chunk activity.
        // Keep only its proxy close to the actual current passenger so vanilla
        // player save/root-vehicle handling cannot persist the stale NBT position.
        pmweatherIv$anchorProxyToPlayer(linkedSeat, player);
    }

    @Unique
    private void pmweatherIv$anchorProxyToPlayer(BuilderEntityLinkedSeat linkedSeat, ServerPlayer player) {
        double gap = pmweatherIv$distance(linkedSeat.getX(), linkedSeat.getY(), linkedSeat.getZ(),
            player.getX(), player.getY(), player.getZ());
        linkedSeat.setPos(player.getX(), player.getY(), player.getZ());
        pmweatherIv$logMountRecovery(linkedSeat, "PROXY_ANCHORED", gap);
    }

    @Unique
    private void pmweatherIv$logMountRecovery(BuilderEntityLinkedSeat linkedSeat, String action, double gap) {
        if (!pmweatherIv$mountRecoveryLogged && gap > 4.0 && PMIVObserver.loggingEnabled()) {
            pmweatherIv$mountRecoveryLogged = true;
            PMIVObserver.log("PMIV_SEAT_MOUNT_RECOVERY action=" + action
                + " proxyUuid=" + linkedSeat.getUUID()
                + " linkedUuid=" + entityUUID
                + " gapMeters=" + gap);
        }
    }

    @Unique
    private boolean pmweatherIv$validSeatForMountedPlayer(
        PartSeat seat, WrapperEntity actualRider, BuilderEntityLinkedSeat linkedSeat
    ) {
        if (seat == null || !seat.isValid || seat.world != WrapperWorld.getWrapperFor(linkedSeat.level())
            || seat.uniqueUUID == null || !seat.uniqueUUID.equals(entityUUID)
            || seat.vehicleOn == null || !seat.vehicleOn.isValid
            || !com.g9third.pmweatheriv.sable.SableVehicleManager.supportsVehicle(seat.vehicleOn)
            || !pmweatherIv$safeToReplace(seat.rider, actualRider)) {
            return false;
        }
        return pmweatherIv$finitePosition(seat.position.x, seat.position.y, seat.position.z);
    }

    @Unique
    private void pmweatherIv$consumeUnparsedSeatLink() {
        BuilderEntityLinkedSeat linkedSeat = (BuilderEntityLinkedSeat) (Object) this;
        CompoundTag pending = linkedSeat.lastLoadedNBT;
        if (pending == null) {
            return;
        }
        if (entityUUID == null && pending.hasUUID("entityUUID")) {
            try {
                entityUUID = pending.getUUID("entityUUID");
            } catch (RuntimeException ignored) {
                return;
            }
        }
        if (entityUUID != null) {
            // Match IV's successful UUID parse branch before dropping the saved
            // snapshot that otherwise overwrites the live vanilla proxy position.
            linkedSeat.loadedFromSavedNBT = true;
            linkedSeat.lastLoadedNBT = null;
        }
    }

    @Inject(method = "baseTick", at = @At("HEAD"), remap = false)
    private void pmweatherIv$repairSavedRiderBeforeIvTick(CallbackInfo callbackInfo) {
        pmweatherIv$repairIfNeeded("HEAD");
    }

    @Inject(method = "baseTick", at = @At("TAIL"), remap = false)
    private void pmweatherIv$verifySavedRiderAfterIvTick(CallbackInfo callbackInfo) {
        pmweatherIv$consumeParsedSeatData();
        BuilderEntityLinkedSeat linkedSeat = (BuilderEntityLinkedSeat) (Object) this;
        if (entity == null && entityUUID != null && linkedSeat.tickCount == 20
            && PMIVObserver.loggingEnabled()) {
            PMIVObserver.log("PMIV_SEAT_LINK_PENDING side=" + pmweatherIv$side(linkedSeat)
                + " proxyUuid=" + linkedSeat.getUUID() + " linkedUuid=" + entityUUID
                + " proxyPosition=" + pmweatherIv$position(linkedSeat)
                + " passengers=" + linkedSeat.getPassengers().size());
        }
        pmweatherIv$repairIfNeeded("TAIL");
        pmweatherIv$advanceReconnectVerification();
    }

    @Inject(method = "removePassenger", at = @At("RETURN"), remap = false)
    private void pmweatherIv$observeSeatDeparture(Entity passenger, CallbackInfo callbackInfo) {
        if (!PMIVObserver.loggingEnabled()) return;
        BuilderEntityLinkedSeat linkedSeat = (BuilderEntityLinkedSeat) (Object) this;
        PMIVObserver.log("PMIV_SEAT_PASSENGER_REMOVED side=" + pmweatherIv$side(linkedSeat)
            + " proxyUuid=" + linkedSeat.getUUID() + " passengerUuid=" + passenger.getUUID()
            + " linkedUuid=" + (entity == null ? entityUUID : entity.uniqueUUID)
            + " passengerPosition=" + pmweatherIv$position(passenger)
            + " proxyPosition=" + pmweatherIv$position(linkedSeat)
            + " dismountedRider=" + dismountedRider + " proxyTicks=" + linkedSeat.tickCount);
    }

    @Unique
    private void pmweatherIv$consumeParsedSeatData() {
        BuilderEntityLinkedSeat linkedSeat = (BuilderEntityLinkedSeat) (Object) this;
        if (linkedSeat.loadedFromSavedNBT && (entityUUID != null || entity != null)
            && linkedSeat.lastLoadedNBT != null) {
            // IV's base builder merges pending load data over live entity data on
            // save. Once the seat UUID is parsed, that snapshot is no longer pending.
            linkedSeat.lastLoadedNBT = null;
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log("PMIV_SEAT_LOAD_DATA_CONSUMED side=" + pmweatherIv$side(linkedSeat)
                    + " proxyUuid=" + linkedSeat.getUUID()
                    + " linkedUuid=" + (entity == null ? entityUUID : entity.uniqueUUID));
            }
        }
    }

    @Inject(method = "saveWithoutId", at = @At("HEAD"), remap = false)
    private void pmweatherIv$saveLiveSeatState(CompoundTag tag,
        CallbackInfoReturnable<CompoundTag> callbackInfo) {
        BuilderEntityLinkedSeat linkedSeat = (BuilderEntityLinkedSeat) (Object) this;
        if (!linkedSeat.level().isClientSide && linkedSeat.getPassengers().size() == 1
            && linkedSeat.getPassengers().get(0) instanceof ServerPlayer player) {
            pmweatherIv$prepareMountedPlayer(player);
        }
        pmweatherIv$consumeParsedSeatData();
    }

    @Inject(method = "saveWithoutId", at = @At("RETURN"), remap = false)
    private void pmweatherIv$preserveUnresolvedSeatLink(CompoundTag tag,
        CallbackInfoReturnable<CompoundTag> callbackInfo) {
        // Clearing consumed load data must not lose a parsed link while its IV
        // entity is still loading. Preserve that UUID alongside the current pose.
        if (entity == null && entityUUID != null)
            callbackInfo.getReturnValue().putUUID("entityUUID", entityUUID);
    }

    @Inject(method = "onRemovedFromLevel", at = @At("HEAD"), remap = false)
    private void pmweatherIv$releaseReconnectHoldOnSeatRemoval(CallbackInfo callbackInfo) {
        pmweatherIv$releaseHold("LINKED_SEAT_REMOVED");
    }

    @Unique
    private void pmweatherIv$repairIfNeeded(String phase) {
        BuilderEntityLinkedSeat linkedSeat = (BuilderEntityLinkedSeat) (Object) this;
        if (dismountedRider || linkedSeat.isRemoved()) {
            pmweatherIv$releaseHold("RIDER_DISMOUNTED");
            pmweatherIv$clearSession();
            return;
        }
        EntityVehicleF_Physics vehicle = pmweatherIv$managedAircraft(entity);
        if (vehicle == null) {
            return;
        }

        List<Entity> passengers = linkedSeat.getPassengers();
        if (passengers.size() != 1) {
            return;
        }
        Entity passenger = passengers.get(0);
        WrapperEntity actualRider = WrapperEntity.getWrapperFor(passenger);
        if (actualRider == null) {
            return;
        }

        boolean ivMatches = pmweatherIv$sameRider(entity.rider, actualRider);
        boolean proxyMatches = pmweatherIv$sameRider(rider, actualRider);
        if (ivMatches) {
            if (!proxyMatches) {
                // Fresh mounts normally reach the first linked-seat tick with the
                // authoritative IV rider already installed while this proxy-local
                // cache is still null.  That is ordinary initialization, not a
                // reconnect race, so synchronize the cache without pausing Sable.
                rider = actualRider;
                dismountedRider = false;
                ticksWithoutRider = 0;
            }
            if (pmweatherIv$reconnectRepairActive) {
                // Make the current vanilla passenger authoritative one more time.
                // This updates the real player, not a stale wrapper, before the
                // verification counter is allowed to release Sable.
                entity.updateRider();
            }
            return;
        }

        IWrapperEntity previousIvRider = entity.rider;
        if (previousIvRider != null && !pmweatherIv$safeToReplace(previousIvRider, actualRider)) {
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "PMIV_RIDER_RECONNECT_CONFLICT side=" + pmweatherIv$side(linkedSeat)
                    + " phase=" + phase
                    + " vehicleUuid=" + vehicle.uniqueUUID
                    + " passengerUuid=" + passenger.getUUID()
                    + " ivRiderUuid=" + pmweatherIv$id(previousIvRider)
                    + " ivRiderValid=" + pmweatherIv$valid(previousIvRider)
                    + " action=NO_SEAT_STEAL"
            );
            }
            return;
        }

        pmweatherIv$beginSession(vehicle, passenger, linkedSeat, phase, previousIvRider);

        if (previousIvRider == null) {
            // If only the linked-seat wrapper is stale, clear it and let IV's own
            // baseTick perform its normal null->mounted-rider setRider path. At TAIL
            // (including the first tick where entityUUID resolves) we may need to do
            // that normal setRider immediately because IV already passed the branch.
            rider = null;
            dismountedRider = false;
            ticksWithoutRider = 0;
            if ("TAIL".equals(phase)) {
                boolean mounted = entity.setRider(actualRider, true);
                if (mounted || pmweatherIv$sameRider(entity.rider, actualRider)) {
                    rider = actualRider;
                }
            }
        } else {
            // Preserve PartSeat's already-established controller/HUD side effects.
            // Calling PartSeat.removeRider()+setRider() here would decrement then
            // re-increment controller state and can toggle authored seat behavior.
            // Instead swap only the stale identity and reconstruct the base rider
            // transform/cache that AEntityB_Existing.setRider normally owns.
            pmweatherIv$replaceStaleRiderIdentity(linkedSeat, actualRider);
        }

        if (pmweatherIv$sameRider(entity.rider, actualRider)) {
            rider = actualRider;
            dismountedRider = false;
            ticksWithoutRider = 0;
            linkedSeat.setPos(entity.position.x, entity.position.y, entity.position.z);
            entity.updateRider();

            if (!linkedSeat.level().isClientSide && previousIvRider != null) {
                // Direct stale-wrapper replacement deliberately bypasses PartSeat's
                // mount side effects, so mirror the corrected IV rider identity to
                // clients using IV's own packet format.
                InterfaceManager.packetInterface.sendToAllClients(
                    new PacketEntityRiderChange(entity, actualRider, true)
                );
            }

            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "PMIV_RIDER_RECONNECT_REBOUND side=" + pmweatherIv$side(linkedSeat)
                    + " phase=" + phase
                    + " vehicleUuid=" + vehicle.uniqueUUID
                    + " playerUuid=" + passenger.getUUID()
                    + " previousIvRiderUuid=" + pmweatherIv$id(previousIvRider)
                    + " previousIvRiderValid=" + pmweatherIv$valid(previousIvRider)
                    + " proxyPosition=" + pmweatherIv$position(linkedSeat)
                    + " seatPosition=" + entity.position
                    + " policy=ACTUAL_VANILLA_PASSENGER_IS_AUTHORITATIVE"
            );
            }
        }
    }

    @Unique
    private void pmweatherIv$replaceStaleRiderIdentity(
        BuilderEntityLinkedSeat linkedSeat,
        WrapperEntity actualRider
    ) {
        entity.rider = actualRider;
        entity.riderIsClient = linkedSeat.level().isClientSide
            && InterfaceManager.clientInterface != null
            && actualRider.equals(InterfaceManager.clientInterface.getClientPlayer());

        if (entity.riderRelativeOrientation == null) {
            entity.riderRelativeOrientation = new RotationMatrix();
        }
        if (entity.prevRiderRelativeOrientation == null) {
            entity.prevRiderRelativeOrientation = new RotationMatrix();
        }
        entity.riderRelativeOrientation.setToZero();
        entity.prevRiderRelativeOrientation.set(entity.riderRelativeOrientation);

        actualRider.setOrientation(entity.orientation);
        actualRider.setPosition(entity.position, false);
        actualRider.getYawDelta();
        actualRider.getPitchDelta();

        // entity.rider already points at actualRider, so WrapperEntity.setRiding()
        // recognizes the existing vanilla BuilderEntityLinkedSeat and reuses it;
        // it does not spawn a duplicate hidden seat entity.
        actualRider.setRiding(entity);
        rider = actualRider;
        dismountedRider = false;
        ticksWithoutRider = 0;
    }

    @Unique
    private void pmweatherIv$beginSession(
        EntityVehicleF_Physics vehicle,
        Entity passenger,
        BuilderEntityLinkedSeat linkedSeat,
        String phase,
        IWrapperEntity previousIvRider
    ) {
        if (!pmweatherIv$reconnectRepairActive) {
            pmweatherIv$reconnectRepairActive = true;
            pmweatherIv$reconnectStableTicks = 0;
            pmweatherIv$reconnectSessionTicks = 0;
            pmweatherIv$reconnectPlayerUuid = passenger.getUUID();
            pmweatherIv$reconnectVehicle = vehicle;
            if (!linkedSeat.level().isClientSide) {
                pmweatherIv$sableHoldActive = SableVehicleManager.beginRiderReconnectHold(
                    vehicle, passenger.getUUID()
                );
            }
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "PMIV_RIDER_RECONNECT_BEGIN side=" + pmweatherIv$side(linkedSeat)
                    + " phase=" + phase
                    + " vehicleUuid=" + vehicle.uniqueUUID
                    + " playerUuid=" + passenger.getUUID()
                    + " previousIvRiderUuid=" + pmweatherIv$id(previousIvRider)
                    + " previousIvRiderValid=" + pmweatherIv$valid(previousIvRider)
                    + " vanillaMount=" + linkedSeat.getUUID()
                    + " sableHold=" + pmweatherIv$sableHoldActive
            );
            }
        }
    }

    @Unique
    private void pmweatherIv$advanceReconnectVerification() {
        if (!pmweatherIv$reconnectRepairActive) {
            return;
        }
        ++pmweatherIv$reconnectSessionTicks;

        BuilderEntityLinkedSeat linkedSeat = (BuilderEntityLinkedSeat) (Object) this;
        EntityVehicleF_Physics vehicle = pmweatherIv$managedAircraft(entity);
        List<Entity> passengers = linkedSeat.getPassengers();
        boolean stable = vehicle != null
            && vehicle == pmweatherIv$reconnectVehicle
            && passengers.size() == 1
            && passengers.get(0).getUUID().equals(pmweatherIv$reconnectPlayerUuid);
        if (stable) {
            WrapperEntity actual = WrapperEntity.getWrapperFor(passengers.get(0));
            stable = actual != null
                && pmweatherIv$sameRider(entity.rider, actual)
                && pmweatherIv$sameRider(rider, actual)
                && actual.getEntityRiding() == entity;
            if (stable) {
                entity.updateRider();
            }
        }

        if (stable) {
            ++pmweatherIv$reconnectStableTicks;
            if (pmweatherIv$reconnectStableTicks >= PMWEATHER_IV_RECONNECT_STABLE_TICKS) {
                if (PMIVObserver.loggingEnabled()) {
                    PMIVObserver.log(
                    "PMIV_RIDER_RECONNECT_VERIFIED side=" + pmweatherIv$side(linkedSeat)
                        + " vehicleUuid=" + vehicle.uniqueUUID
                        + " playerUuid=" + pmweatherIv$reconnectPlayerUuid
                        + " stableTicks=" + pmweatherIv$reconnectStableTicks
                        + " sessionTicks=" + pmweatherIv$reconnectSessionTicks
                        + " proxyPosition=" + pmweatherIv$position(linkedSeat)
                        + " seatPosition=" + entity.position
                );
                }
                pmweatherIv$releaseHold("RIDER_LINK_VERIFIED");
                pmweatherIv$clearSession();
            }
        } else {
            pmweatherIv$reconnectStableTicks = 0;
            if (pmweatherIv$reconnectSessionTicks >= PMWEATHER_IV_RECONNECT_MAX_HOLD_TICKS) {
                if (PMIVObserver.loggingEnabled()) {
                    PMIVObserver.log(
                    "PMIV_RIDER_RECONNECT_TIMEOUT side=" + pmweatherIv$side(linkedSeat)
                        + " vehicleUuid=" + (pmweatherIv$reconnectVehicle == null
                            ? "null" : pmweatherIv$reconnectVehicle.uniqueUUID)
                        + " playerUuid=" + pmweatherIv$reconnectPlayerUuid
                        + " sessionTicks=" + pmweatherIv$reconnectSessionTicks
                        + " action=RELEASE_SABLE_HOLD_KEEP_IV_VANILLA_STATE"
                );
                }
                pmweatherIv$releaseHold("RIDER_LINK_TIMEOUT");
                pmweatherIv$clearSession();
            }
        }
    }

    @Unique
    private void pmweatherIv$releaseHold(String reason) {
        if (pmweatherIv$sableHoldActive && pmweatherIv$reconnectVehicle != null) {
            SableVehicleManager.endRiderReconnectHold(
                pmweatherIv$reconnectVehicle,
                pmweatherIv$reconnectPlayerUuid,
                reason
            );
        }
        pmweatherIv$sableHoldActive = false;
    }

    @Unique
    private void pmweatherIv$clearSession() {
        pmweatherIv$reconnectRepairActive = false;
        pmweatherIv$reconnectStableTicks = 0;
        pmweatherIv$reconnectSessionTicks = 0;
        pmweatherIv$reconnectPlayerUuid = null;
        pmweatherIv$reconnectVehicle = null;
    }

    @Unique
    private static EntityVehicleF_Physics pmweatherIv$managedAircraft(AEntityB_Existing linkedEntity) {
        if (!(linkedEntity instanceof PartSeat seat)) {
            return null;
        }
        EntityVehicleF_Physics vehicle = seat.vehicleOn;
        if (vehicle == null || !vehicle.isValid || vehicle.definition == null
            || vehicle.definition.motorized == null
            || !com.g9third.pmweatheriv.sable.SableVehicleManager.supportsVehicle(vehicle)) {
            return null;
        }
        if (vehicle.world.isClient()) {
            return AircraftStateNetwork.managed(vehicle) ? vehicle : null;
        }
        return PMWeatherIVConfig.get().enabled() ? vehicle : null;
    }

    @Unique
    private static boolean pmweatherIv$sameRider(IWrapperEntity first, IWrapperEntity second) {
        return first != null && second != null && first.equals(second);
    }

    @Unique
    private static boolean pmweatherIv$safeToReplace(
        IWrapperEntity previous,
        IWrapperEntity actual
    ) {
        if (previous == null || actual == null || previous.equals(actual)) {
            return true;
        }
        try {
            if (!previous.isValid()) {
                return true;
            }
            UUID previousId = previous.getID();
            UUID actualId = actual.getID();
            return previousId != null && previousId.equals(actualId);
        } catch (RuntimeException ignored) {
            return false;
        }
    }

    @Unique
    private static boolean pmweatherIv$valid(IWrapperEntity wrapper) {
        try {
            return wrapper != null && wrapper.isValid();
        } catch (RuntimeException ignored) {
            return false;
        }
    }

    @Unique
    private static boolean pmweatherIv$finitePosition(double x, double y, double z) {
        return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z);
    }

    @Unique
    private static double pmweatherIv$distance(double ax, double ay, double az, double bx, double by, double bz) {
        return Math.sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by) + (az - bz) * (az - bz));
    }

    @Unique
    private static String pmweatherIv$id(IWrapperEntity wrapper) {
        if (wrapper == null) {
            return "null";
        }
        try {
            return String.valueOf(wrapper.getID());
        } catch (RuntimeException ignored) {
            return "unavailable";
        }
    }

    @Unique
    private static String pmweatherIv$side(BuilderEntityLinkedSeat linkedSeat) {
        return linkedSeat.level().isClientSide ? "CLIENT" : "SERVER";
    }

    @Unique
    private static String pmweatherIv$position(Entity entity) {
        return "(" + entity.getX() + "," + entity.getY() + "," + entity.getZ() + ")";
    }
}
