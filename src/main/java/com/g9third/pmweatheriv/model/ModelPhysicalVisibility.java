package com.g9third.pmweatheriv.model;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import minecrafttransportsimulator.baseclasses.AnimationSwitchbox;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.jsondefs.JSONAnimatedObject;
import minecrafttransportsimulator.jsondefs.JSONAnimationDefinition;

/** Stable structural gates, excluding missing-part helpers and duplicate damaged shells. */
public final class ModelPhysicalVisibility {
    private ModelPhysicalVisibility() {}
    public static boolean visible(EntityVehicleF_Physics vehicle, String name) {
        if (vehicle.definition.rendering == null || vehicle.definition.rendering.animatedObjects == null) return true;
        for (JSONAnimatedObject object : vehicle.definition.rendering.animatedObjects) {
            if (object != null && object.objectName != null && object.objectName.equalsIgnoreCase(name)
                && !visible(vehicle,object)) return false;
        }
        return true;
    }
    private static boolean visible(EntityVehicleF_Physics vehicle, JSONAnimatedObject object) {
        if (object.animations == null) return true;
        List<JSONAnimationDefinition> gates = new ArrayList<>();
        boolean hasPartGate=false;
        for (JSONAnimationDefinition animation : object.animations) {
            if (animation == null || animation.variable == null || animation.animationType == null) continue;
            String type=animation.animationType.name();
            String variable=animation.variable.toLowerCase(Locale.ROOT);
            // Flight damage is already handled by the structural mask. Do not double
            // authored mass/aero area by also incorporating damage-only replacement meshes.
            if (type.equals("VISIBILITY") && variable.contains("damage")
                && Double.isFinite(animation.clampMin) && Double.isFinite(animation.clampMax)
                && animation.clampMin<=animation.clampMax
                && (animation.clampMin>0 || animation.clampMax<0)) return false;
            if (type.equals("INHIBITOR") || type.equals("ACTIVATOR")) gates.add(animation);
            else if (type.equals("VISIBILITY") && variable.contains("part_present_")) {
                gates.add(animation); hasPartGate=true;
            }
        }
        return !hasPartGate || new AnimationSwitchbox(vehicle,gates,null).runSwitchbox(0,true);
    }
    public static String signature(EntityVehicleF_Physics vehicle) {
        if (vehicle.definition.rendering == null || vehicle.definition.rendering.animatedObjects == null) return "none";
        StringBuilder key=new StringBuilder();
        for (JSONAnimatedObject object : vehicle.definition.rendering.animatedObjects) {
            if (object != null && object.objectName != null) key.append(object.objectName).append('=')
                .append(visible(vehicle,object)).append(';');
        }
        return key.toString();
    }
}
