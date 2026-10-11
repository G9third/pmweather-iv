package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.sable.OrientedHitboxRegistry;
import mcinterface1211.WrapperAABBCollective;
import java.util.Set;
import minecrafttransportsimulator.baseclasses.BoundingBox;
import net.minecraft.world.phys.AABB;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.Final;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

/** The native entity-shape collector must query the same oriented boxes it will collide with. */
@Mixin(value = WrapperAABBCollective.class, remap = false)
public abstract class WrapperEntityBoundsMixin {
    @Shadow @Final private boolean collision;
    @Shadow public abstract Set<BoundingBox> getBoxes();

    @Inject(method = "intersects(DDDDDD)Z", at = @At("HEAD"), cancellable = true, remap = false)
    private void pmweatherIv$orientedEntityBounds(double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ, CallbackInfoReturnable<Boolean> ci) {
        if (!collision) return;
        boolean managed = false, intersects = false;
        for (BoundingBox box : getBoxes()) {
            if (OrientedHitboxRegistry.contains(box)) {
                managed = true;
                intersects |= OrientedHitboxRegistry.intersectsWorldAabb(box, null, false,
                    minX, minY, minZ, maxX, maxY, maxZ);
            } else {
                intersects |= new AABB(box.globalCenter.x - box.widthRadius,
                    box.globalCenter.y - box.heightRadius, box.globalCenter.z - box.depthRadius,
                    box.globalCenter.x + box.widthRadius, box.globalCenter.y + box.heightRadius,
                    box.globalCenter.z + box.depthRadius).intersects(minX, minY, minZ, maxX, maxY, maxZ);
            }
        }
        if (managed) ci.setReturnValue(intersects);
    }
}
