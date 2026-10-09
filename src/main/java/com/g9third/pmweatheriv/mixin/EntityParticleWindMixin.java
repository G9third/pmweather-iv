package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.compat.PMAeroBridge;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.EntityParticle;
import minecrafttransportsimulator.jsondefs.JSONParticle;
import net.minecraft.world.level.Level;
import org.joml.Vector3d;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;
import org.objectweb.asm.Opcodes;

@Mixin(targets = "minecrafttransportsimulator.entities.instances.EntityParticle", remap = false)
public abstract class EntityParticleWindMixin {
    private static final Vector3d pmweatherIv$velocityScratch = new Vector3d();

    @Inject(method = "update", at = @At(value = "FIELD",
        target = "Lminecrafttransportsimulator/jsondefs/JSONParticle;ignoreCollision:Z", opcode = Opcodes.GETFIELD,
        shift = At.Shift.BEFORE), remap = false)
    private void pmweatherIv$applyWindBeforeParticleCollision(CallbackInfo callback) {
        EntityParticle particle = (EntityParticle) (Object) this;
        EntityParticleWindAccessor accessor = (EntityParticleWindAccessor) particle;
        JSONParticle definition = accessor.pmweatherIv$getDefinition();
        if (definition == null || accessor.pmweatherIv$isTouchingBlocks() && definition.stopsOnGround
                || excluded(definition)) return;

        if (!(particle.world instanceof WrapperWorldAccessor worldAccessor)) return;
        Level level = worldAccessor.pmweatherIv$getLevel();
        if (level == null || !level.isClientSide) return;

        Point3D position = particle.position;
        Point3D motion = particle.motion;
        pmweatherIv$velocityScratch.set(motion.x, motion.y, motion.z);
        if (PMAeroBridge.applyClientParticleWind(level, particle, position.x, position.y, position.z,
                pmweatherIv$velocityScratch, response(definition.type))) {
            motion.set(pmweatherIv$velocityScratch.x, pmweatherIv$velocityScratch.y, pmweatherIv$velocityScratch.z);
        }
    }

    private static boolean excluded(JSONParticle definition) {
        if (definition.type == JSONParticle.ParticleType.BUBBLE || definition.type == JSONParticle.ParticleType.DRIP) return true;
        if (definition.spawningOrientation == null) return true;
        String orientation = definition.spawningOrientation.name();
        return orientation.equals("ATTACHED") || orientation.equals("ATTACHED_Z")
                || orientation.equals("WORLD_ATTACHED") || orientation.equals("STREAK")
                || orientation.equals("TRAIL");
    }

    private static double response(JSONParticle.ParticleType type) {
        if (type == JSONParticle.ParticleType.SMOKE) return 0.12D;
        if (type == JSONParticle.ParticleType.BREAK || type == JSONParticle.ParticleType.CASING) return 0.015D;
        return 0.04D;
    }
}
