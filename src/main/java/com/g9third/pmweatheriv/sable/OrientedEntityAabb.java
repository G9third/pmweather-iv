package com.g9third.pmweatheriv.sable;

import minecrafttransportsimulator.baseclasses.BoundingBox;
import net.minecraft.world.phys.AABB;

/** Conservative query bounds carrying an exact collision shape into IV's shape conversion. */
public final class OrientedEntityAabb extends AABB {
    public final BoundingBox source;
    public OrientedEntityAabb(BoundingBox source, OrientedHitboxRegistry.WorldAabb b) {
        super(b.minX(), b.minY(), b.minZ(), b.maxX(), b.maxY(), b.maxZ());
        this.source = source;
    }
}
