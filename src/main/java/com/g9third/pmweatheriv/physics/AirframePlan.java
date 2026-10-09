package com.g9third.pmweatheriv.physics;

import java.util.List;

/** Immutable model interpretation. Live poses transform these stations without remeshing them. */
public record AirframePlan(
    ModelSurfaceMap.PreparedModel model,
    AirframeGeometry.Geometry geometry,
    AirframeGeometry.SurfaceAreaPlan areaPlan,
    LiftingSurfaceAdapter.PreparedFixedWingPlan fixedWingPlan,
    Vec3d inertia,
    Vec3d centerOfMassLocal,
    boolean rotorcraft,
    List<RotorModel.PreparedRotorDisc> rotorDiscs,
    List<ModelSurfaceMap.PressurePatch> pressurePatches
) {
    public AirframePlan {
        rotorDiscs = List.copyOf(rotorDiscs);
        pressurePatches = List.copyOf(pressurePatches);
    }
}
