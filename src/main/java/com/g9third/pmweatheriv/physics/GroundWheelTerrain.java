package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.mixin.WrapperWorldAccessor;
import java.util.ArrayList;
import java.util.List;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.Level;
import net.minecraft.world.phys.AABB;
import static com.g9third.pmweatheriv.physics.FlightMath.*;

/** Exact live tire-bottom probes; consolidated IV animation boxes do not grant physical grip. */
final class GroundWheelTerrain {
    private GroundWheelTerrain() {}
    record Contact(PartGroundDevice device, Vec3d pointLocal, double targetNormalMps,
                   double gapMeters, BlockPos supportBlock) {}
    static List<Contact> contacts(EntityVehicleF_Physics vehicle, Vec3d center,
                                  Vec3d velocityCOM, Vec3d omegaBody, double dt) {
        List<Contact> contacts = new ArrayList<>();
        if (!(vehicle.world instanceof WrapperWorldAccessor access)) return contacts;
        Level level=access.pmweatherIv$getLevel();
        Vec3d omegaWorld=toWorld(vehicle,omegaBody);
        for (APart part:vehicle.allParts) {
            if (!(part instanceof PartGroundDevice device) || !TireContactMaterial.activeSupport(device)
                || (!device.definition.ground.isWheel && !device.definition.ground.isTread)) continue;
            Vec3d local=LandingGearSolver.landingGearDeviceContactPointLocal(device);
            if (local==null) continue;
            Vec3d world=activePositionWorld(vehicle).add(toWorld(vehicle,local));
            Vec3d speed=velocityCOM.add(omegaWorld.cross(toWorld(vehicle,local.subtract(center))));
            double down=Math.max(0,-speed.y()*dt);
            double minTop=world.y()-Math.max(0.05,down+0.01), maxTop=world.y()+0.25;
            int x=(int)Math.floor(world.x()), z=(int)Math.floor(world.z());
            double top=Double.NEGATIVE_INFINITY;
            BlockPos support=null;
            int y0=Math.max(level.getMinBuildHeight(),(int)Math.floor(minTop)-1);
            int y1=Math.min(level.getMaxBuildHeight()-1,(int)Math.floor(maxTop)+1);
            for (int y=y0; y<=y1; ++y) {
                BlockPos pos=new BlockPos(x,y,z);
                for (AABB box:level.getBlockState(pos).getCollisionShape(level,pos).toAabbs()) {
                    if (world.x()<x+box.minX-1e-6 || world.x()>x+box.maxX+1e-6
                        || world.z()<z+box.minZ-1e-6 || world.z()>z+box.maxZ+1e-6) continue;
                    double t=y+box.maxY;
                    if (t>=minTop && t<=maxTop && t>top) {top=t; support=pos;}
                }
            }
            if (support==null) continue;
            double gap=world.y()-top;
            boolean crossing=gap>0.01 && speed.y()<0 && gap<=down+0.01;
            if (gap>0.01 && !crossing) continue; // 5 cm proximity is animation only.
            double target=crossing ? -Math.max(0,gap-0.01)/dt
                : Math.max(0,-gap-0.005)*(1-Math.exp(-4*dt))/dt;
            Vec3d contactLocal=crossing ? local
                : local.add(toLocal(vehicle,new Vec3d(0,-gap,0)));
            contacts.add(new Contact(device,contactLocal,target,gap,support));
        }
        contacts.sort(java.util.Comparator.comparingDouble((Contact c)->c.pointLocal().z())
            .thenComparingDouble(c->c.pointLocal().x()).thenComparingDouble(c->c.pointLocal().y()));
        return contacts;
    }
}
