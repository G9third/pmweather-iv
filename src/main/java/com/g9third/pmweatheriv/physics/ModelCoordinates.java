package com.g9third.pmweatheriv.physics;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import static com.g9third.pmweatheriv.physics.FlightMath.*;

/** Authored mesh scale and body/model-origin transforms; no world-height offset. */
public final class ModelCoordinates {
    private ModelCoordinates() {}
    public static double scaleComponent(double value) {
        return Double.isFinite(value) && Math.abs(value)>1e-9 ? value : 1;
    }
    public static Vec3d scale(EntityVehicleF_Physics vehicle) {
        Point3D s = vehicle.scale;
        return s == null ? new Vec3d(1,1,1) : new Vec3d(
            scaleComponent(s.x),scaleComponent(s.y),scaleComponent(s.z));
    }
    public static Vec3d scaledPoint(EntityVehicleF_Physics vehicle, Vec3d point) {
        Vec3d s=scale(vehicle);
        return new Vec3d(point.x()*s.x(),point.y()*s.y(),point.z()*s.z());
    }
    public static Vec3d worldPoint(Vec3d origin, RotationMatrix orientation, Vec3d local) {
        return origin.add(toWorld(orientation,local));
    }
    /** Closest point lies on the actual triangle, including its edges and vertices. */
    public static Vec3d closestTrianglePoint(Vec3d point, Vec3d a, Vec3d b, Vec3d c) {
        Vec3d ab=b.subtract(a), ac=c.subtract(a), ap=point.subtract(a);
        if (ab.cross(ac).lengthSquared()<1e-20) {
            Vec3d first=closestSegmentPoint(point,a,b),second=closestSegmentPoint(point,a,c),
                third=closestSegmentPoint(point,b,c);
            Vec3d best=first.subtract(point).lengthSquared()<=second.subtract(point).lengthSquared() ? first : second;
            return best.subtract(point).lengthSquared()<=third.subtract(point).lengthSquared() ? best : third;
        }
        double d1=ab.dot(ap), d2=ac.dot(ap);
        if (d1<=0 && d2<=0) return a;
        Vec3d bp=point.subtract(b);
        double d3=ab.dot(bp), d4=ac.dot(bp);
        if (d3>=0 && d4<=d3) return b;
        double vc=d1*d4-d3*d2;
        if (vc<=0 && d1>=0 && d3<=0) return a.add(ab.scale(d1/(d1-d3)));
        Vec3d cp=point.subtract(c);
        double d5=ab.dot(cp), d6=ac.dot(cp);
        if (d6>=0 && d5<=d6) return c;
        double vb=d5*d2-d1*d6;
        if (vb<=0 && d2>=0 && d6<=0) return a.add(ac.scale(d2/(d2-d6)));
        double va=d3*d6-d5*d4;
        if (va<=0 && d4-d3>=0 && d5-d6>=0)
            return b.add(c.subtract(b).scale((d4-d3)/((d4-d3)+(d5-d6))));
        double denominator=va+vb+vc;
        if (Math.abs(denominator)<1e-20) return a;
        return a.add(ab.scale(vb/denominator)).add(ac.scale(vc/denominator));
    }

    private static Vec3d closestSegmentPoint(Vec3d point, Vec3d a, Vec3d b) {
        Vec3d edge=b.subtract(a);
        double lengthSquared=edge.lengthSquared();
        return lengthSquared>1e-20 ? a.add(edge.scale(Vec3d.clamp(point.subtract(a).dot(edge)/lengthSquared,0,1))) : a;
    }

    /** Support mapping of a rounded tire, with its axle in body coordinates. */
    public static Vec3d tireSupportOffset(RotationMatrix orientation, Vec3d axleBody,
                                         double radius, double halfWidth) {
        Vec3d axle=axleBody.normalized();
        Vec3d down=toLocal(orientation,new Vec3d(0,-1,0));
        double axial=down.dot(axle);
        Vec3d radial=down.subtract(axle.scale(axial));
        double r=Math.max(0,radius), w=Math.max(0,halfWidth);
        double denominator=Math.sqrt(r*r*radial.lengthSquared()+w*w*axial*axial);
        return denominator>1e-12 ? radial.scale(r*r/denominator)
            .add(axle.scale(w*w*axial/denominator)) : Vec3d.ZERO;
    }
}
