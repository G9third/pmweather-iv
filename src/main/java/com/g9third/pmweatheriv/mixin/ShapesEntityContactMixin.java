package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.sable.OrientedEntityAabb;
import com.g9third.pmweatheriv.sable.OrientedEntityVoxelShape;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.shapes.Shapes;
import net.minecraft.world.phys.shapes.VoxelShape;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

/** Retains exact vehicle geometry at IV's native vanilla collision-shape call site. */
@Mixin(Shapes.class)
public abstract class ShapesEntityContactMixin {
    @Inject(method = "create(Lnet/minecraft/world/phys/AABB;)Lnet/minecraft/world/phys/shapes/VoxelShape;",
        at = @At("HEAD"), cancellable = true)
    private static void pmweatherIv$exactEntityShape(AABB bounds, CallbackInfoReturnable<VoxelShape> ci) {
        if (bounds instanceof OrientedEntityAabb oriented)
            ci.setReturnValue(new OrientedEntityVoxelShape(oriented));
    }
}
