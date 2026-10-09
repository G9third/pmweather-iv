package com.g9third.pmweatheriv.network;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import mcinterface1211.BuilderEntityLinkedSeat;
import net.minecraft.server.level.ServerPlayer;
import net.neoforged.neoforge.event.entity.player.PlayerEvent;
import net.neoforged.neoforge.event.tick.PlayerTickEvent;

/** Keeps an unresolved IV seat proxy near its mounted player until its UUID resolves. */
public final class LinkedSeatMountMaintenance {
    private LinkedSeatMountMaintenance() {
    }

    public static void onPlayerLoggedIn(PlayerEvent.PlayerLoggedInEvent event) {
        prepare(event.getEntity());
    }

    public static void onPlayerTickPost(PlayerTickEvent.Post event) {
        prepare(event.getEntity());
    }

    private static void prepare(net.minecraft.world.entity.player.Player player) {
        if (!PMWeatherIVConfig.get().enabled() || !(player instanceof ServerPlayer serverPlayer)
            || !(serverPlayer.getVehicle() instanceof BuilderEntityLinkedSeat linkedSeat)
            || linkedSeat.level() != serverPlayer.level() || linkedSeat.isRemoved()
            || !linkedSeat.hasPassenger(serverPlayer)
            || !(linkedSeat instanceof LinkedSeatMountAccess access)) {
            return;
        }
        access.pmweatherIv$prepareMountedPlayer(serverPlayer);
    }
}
