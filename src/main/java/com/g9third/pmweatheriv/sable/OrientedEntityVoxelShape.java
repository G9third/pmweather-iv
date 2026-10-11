package com.g9third.pmweatheriv.sable;

import it.unimi.dsi.fastutil.doubles.DoubleArrayList;
import it.unimi.dsi.fastutil.doubles.DoubleList;
import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.baseclasses.Point3D;
import net.minecraft.core.Direction.Axis;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.shapes.BitSetDiscreteVoxelShape;
import net.minecraft.world.phys.shapes.VoxelShape;
import org.joml.Vector3d;

/**
 * Vanilla movement/step/ground bookkeeping with exact IV OBB collision distances.
 * The enclosing box is for queries only; its empty corners never stop movement.
 */
public final class OrientedEntityVoxelShape extends VoxelShape {
    private final BoundingBox source;
    private final AABB bounds;
    private final double offsetX, offsetY, offsetZ;

    public OrientedEntityVoxelShape(OrientedEntityAabb bounds) {
        this(bounds.source, bounds, 0, 0, 0);
    }

    private OrientedEntityVoxelShape(BoundingBox source, AABB bounds, double x, double y, double z) {
        super(BitSetDiscreteVoxelShape.withFilledBounds(1, 1, 1, 0, 0, 0, 1, 1, 1));
        this.source = source;
        this.bounds = bounds;
        this.offsetX = x;
        this.offsetY = y;
        this.offsetZ = z;
    }

    @Override public DoubleList getCoords(Axis axis) {
        return DoubleArrayList.wrap(new double[] {bounds.min(axis), bounds.max(axis)});
    }

    @Override public VoxelShape optimize() { return this; }

    @Override public VoxelShape move(double x, double y, double z) {
        return new OrientedEntityVoxelShape(source, bounds.move(x, y, z),
            offsetX + x, offsetY + y, offsetZ + z);
    }

    @Override public double collide(Axis axis, AABB entity, double requested) {
        if (!Double.isFinite(requested) || Math.abs(requested) < 1.0E-9) return requested;
        BoundingBox target = new BoundingBox(new Point3D(
            (entity.minX + entity.maxX) * .5 - offsetX,
            (entity.minY + entity.maxY) * .5 - offsetY,
            (entity.minZ + entity.maxZ) * .5 - offsetZ),
            entity.getXsize() * .5, entity.getYsize() * .5, entity.getZsize() * .5);
        double sign = Math.signum(requested);
        Vector3d direction = switch (axis) {
            case X -> new Vector3d(sign, 0, 0);
            case Y -> new Vector3d(0, sign, 0);
            case Z -> new Vector3d(0, 0, sign);
        };
        var interval = OrientedHitboxRegistry.translationContactInterval(source, target, direction);
        // Starting penetration is repaired by the vehicle contact pass. Never
        // trap an entity that is already inside a surface and trying to leave.
        if (interval == null || interval.exit() <= 1.0E-9 || interval.enter() < -1.0E-7)
            return requested;
        double allowed = Math.max(0, interval.enter() - 1.0E-7);
        return sign * Math.min(Math.abs(requested), allowed);
    }
}
