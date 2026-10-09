package com.g9third.pmweatheriv.physics;

import java.util.ArrayList;
import java.util.List;
import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.jsondefs.JSONCollisionGroup;
import minecrafttransportsimulator.jsondefs.JSONCollisionGroup.CollisionType;

/**
 * Read-only bridge from IV-authored structural collision-group health to PMAero's
 * semantic mesh patches.  Collision boxes never become aerodynamic surfaces;
 * they only mask model-derived pressure/lifting patches after the corresponding
 * authored BLOCK structure is actually totaled by IV.
 */
final class StructuralDamageMask {
    private static final double REGION_MARGIN_METERS = 0.20;

    private StructuralDamageMask() {}

    static Snapshot snapshot(EntityVehicleF_Physics vehicle) {
        if (vehicle == null || vehicle.definition == null
            || vehicle.definition.collisionGroups == null
            || vehicle.definitionCollisionBoxes == null) {
            return Snapshot.EMPTY;
        }
        int count = Math.min(
            vehicle.definition.collisionGroups.size(),
            vehicle.definitionCollisionBoxes.size()
        );
        List<Region> totaled = new ArrayList<>();
        for (int index = 0; index < count; ++index) {
            JSONCollisionGroup group = vehicle.definition.collisionGroups.get(index);
            if (group == null || group.health <= 0 || group.collisionTypes == null
                || !group.collisionTypes.contains(CollisionType.BLOCK)) {
                continue;
            }
            double damage = vehicle.getOrCreateVariable(
                "collision_" + (index + 1) + "_damage"
            ).currentValue;
            boolean totaledState = vehicle.getOrCreateVariable(
                "collision_" + (index + 1) + "_totaled"
            ).isActive;
            if (!(totaledState || (Double.isFinite(damage) && damage >= group.health))) {
                continue;
            }
            List<BoundingBox> boxes = vehicle.definitionCollisionBoxes.get(index);
            if (boxes == null) {
                continue;
            }
            for (BoundingBox box : boxes) {
                if (box == null || box.localCenter == null || box.collisionTypes == null
                    || !box.collisionTypes.contains(CollisionType.BLOCK)) {
                    continue;
                }
                totaled.add(new Region(
                    box.localCenter.x, box.localCenter.y, box.localCenter.z,
                    Math.abs(box.widthRadius) + REGION_MARGIN_METERS,
                    Math.abs(box.heightRadius) + REGION_MARGIN_METERS,
                    Math.abs(box.depthRadius) + REGION_MARGIN_METERS
                ));
            }
        }
        return totaled.isEmpty() ? Snapshot.EMPTY : new Snapshot(List.copyOf(totaled));
    }

    static List<ModelSurfaceMap.PressurePatch> activePressurePatches(
        EntityVehicleF_Physics vehicle, List<ModelSurfaceMap.PressurePatch> patches
    ) {
        if (patches == null || patches.isEmpty()) {
            return List.of();
        }
        Snapshot snapshot = snapshot(vehicle);
        if (snapshot.empty()) {
            return patches;
        }
        List<ModelSurfaceMap.PressurePatch> active = new ArrayList<>(patches.size());
        for (ModelSurfaceMap.PressurePatch patch : patches) {
            if (patch != null && snapshot.active(patch.pointLocal())) {
                active.add(patch);
            }
        }
        return active.isEmpty() ? List.of() : List.copyOf(active);
    }

    static final class Snapshot {
        static final Snapshot EMPTY = new Snapshot(List.of());
        private final List<Region> totaledRegions;

        Snapshot(List<Region> totaledRegions) {
            this.totaledRegions = totaledRegions;
        }

        boolean empty() {
            return totaledRegions.isEmpty();
        }

        boolean active(Vec3d point) {
            if (point == null || !point.isFinite() || totaledRegions.isEmpty()) {
                return true;
            }
            for (Region region : totaledRegions) {
                if (region.contains(point)) {
                    return false;
                }
            }
            return true;
        }

        boolean active(ModelSurfaceMap.LiftingPatch patch) {
            if (patch == null) {
                return false;
            }
            // Use both the mesh station and aerodynamic center.  A patch is
            // disabled if either lies in a totaled structural region.
            return active(patch.pointLocal()) && active(patch.aerodynamicCenterLocal());
        }
    }

    private record Region(
        double cx, double cy, double cz, double hx, double hy, double hz
    ) {
        boolean contains(Vec3d point) {
            return Math.abs(point.x() - cx) <= hx
                && Math.abs(point.y() - cy) <= hy
                && Math.abs(point.z() - cz) <= hz;
        }
    }
}
