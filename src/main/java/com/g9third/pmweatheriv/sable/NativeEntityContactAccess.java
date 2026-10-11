package com.g9third.pmweatheriv.sable;

import net.minecraft.world.entity.Entity;

/** Native movement access for an unmounted IV wrapper, without its teleport fallback. */
public interface NativeEntityContactAccess {
    Entity pmweatherIv$contactEntity();
}
