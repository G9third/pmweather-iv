package com.g9third.pmweatheriv.network;

import net.minecraft.server.level.ServerPlayer;

/** Narrow bridge for recovering a mounted player's unresolved IV seat link. */
public interface LinkedSeatMountAccess {
    void pmweatherIv$prepareMountedPlayer(ServerPlayer player);
}
