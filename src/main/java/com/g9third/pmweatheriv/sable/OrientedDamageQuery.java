package com.g9third.pmweatheriv.sable;

import java.util.ArrayList;
import java.util.List;
import mcinterface1211.ABuilderEntityBase;
import mcinterface1211.BuilderEntityLinkedSeat;
import mcinterface1211.WrapperEntity;
import minecrafttransportsimulator.baseclasses.Damage;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.components.AEntityB_Existing;
import minecrafttransportsimulator.entities.components.AEntityD_Definable;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.mcinterface.IWrapperEntity;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.entity.LivingEntity;
import net.minecraft.world.level.Level;
import net.minecraft.world.phys.AABB;

/**
 * Exact oriented replacement for IV's Minecraft-side attackEntities query when
 * the damage volume belongs to a PMWeather-IV Sable-authority aircraft.
 *
 * <p>Minecraft still performs the cheap candidate lookup using a conservative
 * world AABB. Every candidate is then post-filtered against the true OBB (or a
 * continuously swept OBB when IV supplied a motion vector), so the empty
 * corners introduced by a pitched/rolled broadphase can never cause damage.</p>
 *
 * <p>This class preserves IV's rider/source exclusions and attack/list behavior.
 * It changes query geometry only; Sable remains the physical collision owner.</p>
 */
public final class OrientedDamageQuery {
    private OrientedDamageQuery() {
    }

    /** True only when the complete query can be handled without approximating. */
    public static boolean canHandle(Damage damage, Point3D motion) {
        if (damage == null || damage.box == null) {
            return false;
        }
        if (OrientedHitboxRegistry.contains(damage.box)) {
            return true;
        }
        // IV's auxiliary part-damage volumes (notably propeller discs) are
        // simple boxes with collisionTypes == null and are queried in-place.
        // A moving unregistered box is deliberately NOT inferred from the
        // damage source: bullets use their own world-space moving bounds.
        return motion == null
            && damage.box.collisionTypes == null
            && damage.damgeSource instanceof AEntityD_Definable<?> owner
            && OrientedHitboxRegistry.conservativeWorldAabbForQuery(
                damage.box, owner, true
            ) != null;
    }

    public static List<IWrapperEntity> attackEntities(
        Level world,
        Damage damage,
        Point3D motion,
        boolean generateList
    ) {
        AEntityD_Definable<?> fallbackOwner = damage.damgeSource instanceof AEntityD_Definable<?> owner
            ? owner : null;
        boolean allowAuxiliary = motion == null && damage.box.collisionTypes == null;
        OrientedHitboxRegistry.WorldAabb bounds = OrientedHitboxRegistry.conservativeWorldAabbForQuery(
            damage.box, fallbackOwner, allowAuxiliary
        );
        if (bounds == null) {
            // canHandle() prevents this path, but keep a safe empty result if a
            // box disappears between query setup and execution.
            return generateList ? new ArrayList<>() : null;
        }

        double minX = bounds.minX();
        double minY = bounds.minY();
        double minZ = bounds.minZ();
        double maxX = bounds.maxX();
        double maxY = bounds.maxY();
        double maxZ = bounds.maxZ();
        if (motion != null) {
            minX += Math.min(0.0, motion.x);
            minY += Math.min(0.0, motion.y);
            minZ += Math.min(0.0, motion.z);
            maxX += Math.max(0.0, motion.x);
            maxY += Math.max(0.0, motion.y);
            maxZ += Math.max(0.0, motion.z);
        }

        List<LivingEntity> candidates = world.getEntitiesOfClass(
            LivingEntity.class,
            new AABB(minX, minY, minZ, maxX, maxY, maxZ)
        );
        List<IWrapperEntity> hitEntities = new ArrayList<>();

        for (Entity mcEntityCollided : candidates) {
            // Preserve IV's internal-builder exclusion.
            if (mcEntityCollided instanceof ABuilderEntityBase) {
                continue;
            }

            // Preserve IV's exact source/rider self-damage exclusions.
            if (damage.damgeSource != null) {
                Entity mcRidingEntity = mcEntityCollided.getVehicle();
                if (mcRidingEntity instanceof BuilderEntityLinkedSeat linkedSeat) {
                    AEntityB_Existing internalRidingEntity = linkedSeat.entity;
                    if (damage.damgeSource == internalRidingEntity) {
                        continue;
                    }
                    if (internalRidingEntity instanceof APart ridingPart
                        && ridingPart.masterEntity.allParts.contains(damage.damgeSource)) {
                        continue;
                    }
                }
            }

            AABB target = mcEntityCollided.getBoundingBox();
            boolean exactHit = motion == null
                ? OrientedHitboxRegistry.intersectsWorldAabb(
                    damage.box, fallbackOwner, allowAuxiliary,
                    target.minX, target.minY, target.minZ,
                    target.maxX, target.maxY, target.maxZ
                )
                : OrientedHitboxRegistry.sweptIntersectsWorldAabb(
                    damage.box, fallbackOwner, allowAuxiliary, motion,
                    target.minX, target.minY, target.minZ,
                    target.maxX, target.maxY, target.maxZ
                );
            if (exactHit) {
                hitEntities.add(WrapperEntity.getWrapperFor(mcEntityCollided));
            }
        }

        if (generateList) {
            return hitEntities;
        }
        for (IWrapperEntity entity : hitEntities) {
            entity.attack(damage);
        }
        return null;
    }
}
