package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.sable.OrientedDamageQuery;
import com.g9third.pmweatheriv.sable.OrientedHitboxRegistry;
import mcinterface1211.WrapperWorld;
import java.util.List;
import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.baseclasses.Damage;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.mcinterface.IWrapperEntity;
import net.minecraft.world.level.Level;
import net.minecraft.world.phys.AABB;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

/**
 * Makes Minecraft-side broadphase queries conservatively contain PMWeather-IV
 * oriented semantic hitboxes. Exact hit acceptance remains in BoundingBoxMixin.
 */
@Mixin(value = WrapperWorld.class, remap = false)
public abstract class WrapperWorldMixin {

    @Inject(method = "attackEntities", at = @At("HEAD"), cancellable = true, remap = false)
    private void pmweatherIv$attackEntitiesOrientedExact(
        Damage damage,
        Point3D motion,
        boolean generateList,
        CallbackInfoReturnable<List<IWrapperEntity>> callbackInfo
    ) {
        if (!OrientedDamageQuery.canHandle(damage, motion)) {
            return;
        }
        Level level = ((WrapperWorldAccessor) (Object) this).pmweatherIv$getLevel();
        callbackInfo.setReturnValue(
            OrientedDamageQuery.attackEntities(level, damage, motion, generateList)
        );
    }

    @Inject(method = "convert", at = @At("HEAD"), cancellable = true, remap = false)
    private static void pmweatherIv$convertOrientedBroadphase(
        BoundingBox box,
        CallbackInfoReturnable<AABB> callbackInfo
    ) {
        OrientedHitboxRegistry.WorldAabb bounds = OrientedHitboxRegistry.conservativeWorldAabb(box);
        if (bounds != null) {
            callbackInfo.setReturnValue(new AABB(
                bounds.minX(), bounds.minY(), bounds.minZ(),
                bounds.maxX(), bounds.maxY(), bounds.maxZ()
            ));
        }
    }

    @Inject(method = "convertWithOffset", at = @At("HEAD"), cancellable = true, remap = false)
    private static void pmweatherIv$convertOrientedBroadphaseWithOffset(
        BoundingBox box,
        double x,
        double y,
        double z,
        CallbackInfoReturnable<AABB> callbackInfo
    ) {
        OrientedHitboxRegistry.WorldAabb bounds = OrientedHitboxRegistry.conservativeWorldAabb(box);
        if (bounds != null) {
            callbackInfo.setReturnValue(new AABB(
                bounds.minX() + x, bounds.minY() + y, bounds.minZ() + z,
                bounds.maxX() + x, bounds.maxY() + y, bounds.maxZ() + z
            ));
        }
    }
}
