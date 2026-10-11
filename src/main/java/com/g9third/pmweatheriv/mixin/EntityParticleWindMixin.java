package com.g9third.pmweatheriv.mixin;

import com.g9third.pmweatheriv.compat.PMAeroBridge;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.EntityParticle;
import minecrafttransportsimulator.jsondefs.JSONParticle;
import net.minecraft.world.level.Level;
import org.joml.Vector3d;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;
import org.objectweb.asm.Opcodes;

@Mixin(targets = "minecrafttransportsimulator.entities.instances.EntityParticle", remap = false)
public abstract class EntityParticleWindMixin {
    @Unique private static final Vector3d pmweatherIv$velocityScratch = new Vector3d();
    @Unique private boolean pmweatherIv$classified;
    @Unique private boolean pmweatherIv$excluded;
    @Unique private boolean pmweatherIv$smoke;
    @Unique private boolean pmweatherIv$advectionApplied;
    @Unique private double pmweatherIv$windX;
    @Unique private double pmweatherIv$windY;
    @Unique private double pmweatherIv$windZ;
    @Unique private double pmweatherIv$appliedWindX;
    @Unique private double pmweatherIv$appliedWindY;
    @Unique private double pmweatherIv$appliedWindZ;

    @Inject(method = "update", at = @At("HEAD"), remap = false)
    private void pmweatherIv$separateSmokeAdvection(CallbackInfo callback) {
        if (!pmweatherIv$advectionApplied) return;
        // Let the pack update and cap its own motion, not yesterday's air flow.
        // Reapply the independent ambient component before native collision.
        Point3D motion = ((EntityParticle) (Object) this).motion;
        motion.add(-pmweatherIv$appliedWindX, -pmweatherIv$appliedWindY, -pmweatherIv$appliedWindZ);
        pmweatherIv$advectionApplied = false;
    }

    @Inject(method = "update", at = @At(value = "FIELD",
        target = "Lminecrafttransportsimulator/jsondefs/JSONParticle;ignoreCollision:Z", opcode = Opcodes.GETFIELD,
        shift = At.Shift.BEFORE), remap = false)
    private void pmweatherIv$applyWindBeforeParticleCollision(CallbackInfo callback) {
        EntityParticle particle = (EntityParticle) (Object) this;
        EntityParticleWindAccessor accessor = (EntityParticleWindAccessor) particle;
        JSONParticle definition = accessor.pmweatherIv$getDefinition();
        if (definition == null) return;
        if (!pmweatherIv$classified) {
            pmweatherIv$smoke = isSmoke(definition);
            pmweatherIv$excluded = excluded(definition)
                || pmweatherIv$isStationaryEffect(definition, pmweatherIv$smoke);
            pmweatherIv$classified = true;
        }
        if (pmweatherIv$excluded || accessor.pmweatherIv$isTouchingBlocks() && definition.stopsOnGround) return;

        if (!(particle.world instanceof WrapperWorldAccessor worldAccessor)) return;
        Level level = worldAccessor.pmweatherIv$getLevel();
        if (level == null || !level.isClientSide) return;

        Point3D position = particle.position;
        Point3D motion = particle.motion;
        pmweatherIv$velocityScratch.set(pmweatherIv$smoke ? pmweatherIv$windX : motion.x,
            pmweatherIv$smoke ? pmweatherIv$windY : motion.y,
            pmweatherIv$smoke ? pmweatherIv$windZ : motion.z);
        if (PMAeroBridge.applyClientParticleWind(level, particle, position.x, position.y, position.z,
                pmweatherIv$velocityScratch, pmweatherIv$smoke ? 0.12D : response(definition.type))) {
            if (pmweatherIv$smoke) {
                pmweatherIv$windX = pmweatherIv$velocityScratch.x;
                pmweatherIv$windY = pmweatherIv$velocityScratch.y;
                pmweatherIv$windZ = pmweatherIv$velocityScratch.z;
                motion.add(pmweatherIv$windX, pmweatherIv$windY, pmweatherIv$windZ);
                pmweatherIv$appliedWindX = pmweatherIv$windX;
                pmweatherIv$appliedWindY = pmweatherIv$windY;
                pmweatherIv$appliedWindZ = pmweatherIv$windZ;
                pmweatherIv$advectionApplied = true;
            } else {
                motion.set(pmweatherIv$velocityScratch.x, pmweatherIv$velocityScratch.y, pmweatherIv$velocityScratch.z);
            }
        }
    }

    @Inject(method = "update", at = @At("TAIL"), remap = false)
    private void pmweatherIv$respectSmokeCollision(CallbackInfo callback) {
        var accessor = (EntityParticleWindAccessor) this;
        JSONParticle definition = accessor.pmweatherIv$getDefinition();
        if (pmweatherIv$advectionApplied && definition != null && !definition.ignoreCollision
            && accessor.pmweatherIv$isTouchingBlocks()) {
            // Only remove the portion that survived native axis clipping next
            // tick. Retain the air-flow state and the unchanged tangent portions.
            EntityParticle particle = (EntityParticle) (Object) this;
            Point3D depth = particle.boundingBox.currentCollisionDepth;
            pmweatherIv$appliedWindX *= survivingFraction(particle.motion.x, depth.x);
            pmweatherIv$appliedWindY *= survivingFraction(particle.motion.y, depth.y);
            pmweatherIv$appliedWindZ *= survivingFraction(particle.motion.z, depth.z);
        }
    }

    @Unique
    private static double survivingFraction(double clippedMotion, double collisionDepth) {
        if (collisionDepth == 0) return 1.0;
        double original = clippedMotion + collisionDepth;
        return original == 0 ? 0 : Math.max(0.0, Math.min(1.0, Math.abs(clippedMotion / original)));
    }

    @Unique
    private static boolean isSmoke(JSONParticle definition) {
        if (definition.type == JSONParticle.ParticleType.SMOKE) return true;
        if (definition.type != JSONParticle.ParticleType.GENERIC || definition.model != null) return false;
        if (smokeTexture(definition.texture)) return true;
        if (definition.textureList != null)
            for (String texture : definition.textureList) if (smokeTexture(texture)) return true;
        return false;
    }

    @Unique
    private static boolean smokeTexture(String texture) {
        return texture != null && texture.toLowerCase(java.util.Locale.ROOT).contains("smoke");
    }

    private static boolean excluded(JSONParticle definition) {
        if (definition.type == JSONParticle.ParticleType.BUBBLE || definition.type == JSONParticle.ParticleType.DRIP) return true;
        if (definition.spawningOrientation == null) return true;
        String orientation = definition.spawningOrientation.name();
        return orientation.equals("ATTACHED") || orientation.equals("ATTACHED_Z")
                || orientation.equals("WORLD_ATTACHED") || orientation.equals("STREAK")
                || orientation.equals("TRAIL");
    }

    @Unique
    private static boolean pmweatherIv$isStationaryEffect(JSONParticle definition, boolean smoke) {
        if (definition.type != JSONParticle.ParticleType.GENERIC || definition.model != null
            || smoke) return false;
        boolean fixed = definition.renderingOrientation == JSONParticle.ParticleRenderingOrientation.FIXED;
        // Native terminal caps run before our wind hook. A locked decal must
        // remain locked rather than gaining a new velocity after that cap.
        if (fixed && definition.terminalVelocity != null
            && pmweatherIv$zeroOrAbsent(definition.terminalVelocity)) return true;
        boolean flash = definition.isBright && definition.duration > 0 && definition.duration <= 8;
        return (fixed || flash)
            && pmweatherIv$zeroOrAbsent(definition.initialVelocity)
            && pmweatherIv$zeroOrAbsent(definition.movementVelocity)
            && pmweatherIv$zeroOrAbsent(definition.relativeMovementVelocity)
            && pmweatherIv$zeroOrAbsent(definition.relativeInheritedVelocityFactor)
            && pmweatherIv$zeroOrAbsent(definition.spreadRandomness);
    }

    @Unique
    private static boolean pmweatherIv$zeroOrAbsent(Point3D value) {
        return value == null || value.x == 0.0 && value.y == 0.0 && value.z == 0.0;
    }

    private static double response(JSONParticle.ParticleType type) {
        if (type == JSONParticle.ParticleType.SMOKE) return 0.12D;
        if (type == JSONParticle.ParticleType.BREAK || type == JSONParticle.ParticleType.CASING) return 0.015D;
        return 0.04D;
    }
}
