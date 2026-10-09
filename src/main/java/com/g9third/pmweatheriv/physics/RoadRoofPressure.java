package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.sable.SableModelCollisionHull;
import java.util.List;
import java.util.ArrayList;

/** Coefficient-based upper-surface pressure drop, separate from windward body pressure. */
public final class RoadRoofPressure {
    private RoadRoofPressure() {}
    public record Panel(Vec3d pointLocal, Vec3d normalLocal, double areaM2) {}
    public static List<Panel> prepare(List<ModelSurfaceMap.PressurePatch> patches,
                                     List<SableModelCollisionHull.SurfaceTriangle> triangles) {
        List<Panel> result=new ArrayList<>(patches.size());
        for (var patch:patches) {
            Panel panel=null;
            if (patch.normalLocal().y()>0.99 && !triangles.isEmpty()) {
                double best=Double.POSITIVE_INFINITY;
                for (var t:triangles) {
                    Vec3d cross=t.b().subtract(t.a()).cross(t.c().subtract(t.a()));
                    if (cross.lengthSquared()<1e-18) continue;
                    Vec3d normal=cross.normalized();
                    if (normal.y()<0) normal=normal.scale(-1);
                    if (normal.y()<0.5) continue;
                    Vec3d point=ModelCoordinates.closestTrianglePoint(patch.windSamplePointLocal(),t.a(),t.b(),t.c());
                    double distance=point.subtract(patch.windSamplePointLocal()).lengthSquared();
                    if (distance<best) {
                        best=distance;
                        panel=new Panel(point,normal,patch.area()/normal.y());
                    }
                }
            }
            // A zero-area entry preserves the weather station index without another query.
            result.add(panel==null ? new Panel(patch.pointLocal(),patch.normalLocal(),0) : panel);
        }
        return List.copyOf(result);
    }
    public static Vec3d force(Panel panel, Vec3d airRelativeVelocityLocal, double density, double cp) {
        if (!(panel.areaM2()>0) || !(density>0) || !(cp>0)) return Vec3d.ZERO;
        Vec3d normal=panel.normalLocal();
        Vec3d tangential=airRelativeVelocityLocal.subtract(normal.scale(airRelativeVelocityLocal.dot(normal)));
        // Delta p = -Cp * rho * Ut^2 / 2. Pressure-drop force is outward.
        return normal.scale(0.5*density*cp*panel.areaM2()*tangential.lengthSquared());
    }
}
