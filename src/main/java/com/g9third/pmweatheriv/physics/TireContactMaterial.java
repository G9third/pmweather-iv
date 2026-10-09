package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.mixin.PartGroundDeviceAccessor;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.blocks.components.ABlockBase.BlockMaterial;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import net.minecraft.core.BlockPos;

/** Live pack tire coefficients at the actual support block; reactions remain load bounded. */
public final class TireContactMaterial {
    private TireContactMaterial() {}
    public record Grip(double motive, double lateral, String material, boolean wet) {}

    /** Rolling loss is distinct from the larger brake/sliding grip coefficients.
     * Grass/ice anchors: NASA TP-2015-218751, table 4-3. Other loose surfaces
     * use generic approximations; packs do not supply soil or inflation data.
     * The shared solver multiplies this by actual normal impulse and bounds it
     * with the same tire grip ellipse as braking and lateral friction.
     */
    public static double rollingResistance(double tireCoefficient, Grip grip) {
        double surface = switch (grip.material()) {
            case "GRASS" -> 0.08;
            case "DIRT", "DIRT_WET", "CLAY", "GRAVEL" -> 0.06;
            case "SAND", "SAND_WET" -> 0.10;
            case "SNOW" -> 0.06;
            case "ICE" -> 0.02;
            default -> 0.0;
        };
        return Math.max(Math.max(0.0, tireCoefficient), surface);
    }

    public static boolean activeSupport(PartGroundDevice device) {
        return device != null && device.isValid && !device.isSpare && device.isActiveVar.isActive;
    }

    public static Grip evaluate(PartGroundDevice device, BlockPos supportBlock) {
        if (device == null || device.definition.ground == null) return new Grip(0, 0, "NONE", false);
        PartGroundDeviceAccessor access = (PartGroundDeviceAccessor) device;
        Point3D oldPoint = access.pmweatherIv$getGroundPosition().copy();
        Point3D actual = supportBlock == null ? oldPoint.copy()
            : new Point3D(supportBlock.getX(), supportBlock.getY(), supportBlock.getZ());
        BlockMaterial material = device.world.getBlockMaterial(actual);
        boolean wet = device.world.getRainStrength(actual.copy().add(0, 1, 0)) > 0
            || material == BlockMaterial.DIRT_WET || material == BlockMaterial.SAND_WET
            || material == BlockMaterial.NORMAL_WET;
        double oldLoss = loss(device, oldPoint, access.pmweatherIv$getBlockMaterialBelow(), false);
        double newLoss = loss(device, actual, material, false);
        double flat = device.flatVar.isActive ? 0.1 : 1.0;
        // Preserve live variable modifiers and IV's authored wet/terrain semantics at the actual contact.
        double motive = finite(device.motiveFrictionVar.currentValue
            - Math.max(0,device.definition.ground.motiveFriction-oldLoss)*flat
            + Math.max(0,device.definition.ground.motiveFriction-newLoss)*flat);
        double lateral = finite(device.lateralFrictionVar.currentValue
            - Math.max(0,device.definition.ground.lateralFriction-oldLoss)*flat
            + Math.max(0,device.definition.ground.lateralFriction-newLoss)*flat);
        return new Grip(motive, lateral, material == null ? "NORMAL" : material.name(), wet);
    }

    /** Generic rigid skid/float support friction; no wheel steering or service brake is synthesized. */
    public static Grip staticSupport(minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics vehicle,
                                     BlockPos block) {
        if (vehicle == null || block == null) return new Grip(0,0,"NONE",false);
        Point3D point=new Point3D(block.getX(),block.getY(),block.getZ());
        BlockMaterial material=vehicle.world.getBlockMaterial(point);
        boolean wet=vehicle.world.getRainStrength(point.copy().add(0,1,0))>0 || wetMaterial(material);
        double slipperiness=Math.max(0,vehicle.world.getBlockSlipperiness(point)-0.6);
        double coefficient=Math.min(Math.max(0,(wet ? 0.25 : 0.35)-slipperiness),surfaceCap(material,wet));
        return new Grip(coefficient,coefficient,material==null ? "NORMAL" : material.name(),wet);
    }
    private static boolean wetMaterial(BlockMaterial material) {
        return material==BlockMaterial.DIRT_WET || material==BlockMaterial.SAND_WET || material==BlockMaterial.NORMAL_WET;
    }
    private static double surfaceCap(BlockMaterial material,boolean wet) {
        return switch (material == null ? BlockMaterial.NORMAL : material) {
            case GRASS -> wet ? 0.24 : 0.38;
            case DIRT, DIRT_WET, CLAY -> wet ? 0.35 : 0.50;
            case GRAVEL -> wet ? 0.40 : 0.55;
            case SAND, SAND_WET -> wet ? 0.30 : 0.45;
            case SNOW -> wet ? 0.18 : 0.25;
            case ICE -> 0.10;
            default -> Double.POSITIVE_INFINITY;
        };
    }

    private static double loss(PartGroundDevice device, Point3D point,
                               BlockMaterial material, boolean physicalWetSign) {
        if (device.world.isAir(point)) return 0;
        double loss = device.world.getBlockSlipperiness(point) - 0.6;
        Float modifier = device.definition.ground.frictionModifiers == null ? null
            : device.definition.ground.frictionModifiers.get(material);
        if (modifier != null) loss -= modifier;
        if (device.world.getRainStrength(point.copy().add(0, 1, 0)) > 0
            || (physicalWetSign && wetMaterial(material))) {
            double wet = device.definition.ground.wetFrictionPenalty;
            loss += physicalWetSign ? Math.abs(wet) : wet;
        }
        return Double.isFinite(loss) ? loss : 0;
    }
    private static double finite(double value) { return Double.isFinite(value) ? Math.max(0, value) : 0; }
}
