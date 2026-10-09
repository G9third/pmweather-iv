package com.g9third.pmweatheriv.physics;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import minecrafttransportsimulator.entities.components.AEntityD_Definable;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.jsondefs.JSONAnimatedObject;
import minecrafttransportsimulator.jsondefs.JSONAnimationDefinition;

import static com.g9third.pmweatheriv.physics.ModelGeometryData.AnimationHint;
import static com.g9third.pmweatheriv.physics.ModelSurfaceMap.SurfaceKind;

/** Model names and authored animation evidence used during preparation. */
final class ModelAnimationHints {
    private ModelAnimationHints() {}

    static Map<String, AnimationHint> animatedSurfaceHints(EntityVehicleF_Physics vehicle) {
        return animatedSurfaceHints(AuthoredLiftingSurfacePoses.definitions(vehicle));
    }

    static Map<String, AnimationHint> animatedSurfaceHints(Map<String, JSONAnimatedObject> definitions) {
        Map<String, AnimationHint> result = new HashMap<>();
        for (String name : definitions.keySet()) {
            AnimationHint hint = chainHint(name, definitions, new java.util.HashSet<>());
            if (hint.anySurfaceControl()) result.merge(normalize(name), hint, AnimationHint::combine);
        }
        return result;
    }

    private static AnimationHint chainHint(String name, Map<String, JSONAnimatedObject> definitions,
                                           java.util.Set<String> visiting) {
        JSONAnimatedObject object = definitions.get(name);
        if (object == null || !visiting.add(name)) return AnimationHint.NONE;
        AnimationHint hint = AnimationHint.NONE;
        if (object.animations != null) for (JSONAnimationDefinition animation : object.animations) {
            if (animation != null && animation.variable != null && animation.animationType != null)
                hint = hint.combine(AnimationHint.fromAnimationType(
                    animation.animationType.name(), normalize(animation.variable)));
        }
        // A mesh can be only a skin on a name-free moving parent. Its inherited
        // rotation/translation is physical evidence; visibility tokens still are not.
        if (object.applyAfter != null && !object.applyAfter.isBlank())
            hint = hint.combine(chainHint(object.applyAfter, definitions, visiting));
        return hint;
    }

    static String animationSignature(EntityVehicleF_Physics vehicle) {
        Map<String, JSONAnimatedObject> definitions = AuthoredLiftingSurfacePoses.definitions(vehicle);
        Map<String, AnimationHint> hints = animatedSurfaceHints(definitions);
        StringBuilder signature = new StringBuilder();
        definitions.keySet().stream().sorted().forEach(objectName -> {
            String name = normalize(objectName);
            AnimationHint hint = hints.getOrDefault(name, AnimationHint.NONE);
            SurfaceKind named = SurfaceKind.fromSpecificName(name);
            if (hint.anySurfaceControl() || named != null) signature.append(name).append(':')
                .append(hint.compact()).append(':').append(named == null ? "none" : named.name()).append(';');
        });
        return Integer.toHexString(signature.toString().hashCode());
    }

    static String modelLocation(AEntityD_Definable<?> vehicle) {
        try {
            String location = vehicle.definition.getModelLocation(vehicle.subDefinition);
            return location == null || location.isBlank() ? "missing" : location;
        } catch (RuntimeException exception) {
            return "unavailable:" + exception.getClass().getSimpleName();
        }
    }

    static boolean shellClosingTransparentObject(String objectName) {
        String name = normalize(objectName);
        return containsAny(name, "glass", "window", "windscreen", "windshield", "canopy", "cockpit_glass");
    }

    static boolean ignoredObject(String objectName) {
        String name = normalize(objectName);
        if (name.isEmpty()) {
            return false;
        }
        // IFS and several other packs use # for interior/helper objects and &
        // for emissive/instrument geometry. Neither belongs in the exterior shell.
        if (name.startsWith("#") || name.startsWith("&")) {
            return true;
        }
        if (looksLikeCockpitOrGear(name)) {
            return true;
        }
        return containsAny(name,
            "instrument", "gauge", "needle", "text", "label", "seat",
            "wheel", "tire", "tyre", "propeller", "prop_", "rotor", "blade",
            "lamp", "light", "flare", "exhaust", "particle", "interior", "dashboard",
            "mirror", "antenna", "missing_model", "boundingbox", "bounding_box",
            "reverser", "thrust_reverser");
    }

    static boolean looksLikeCockpitOrGear(String name) {
        if (containsAny(name,
            "con_", "cockpit_control", "stick", "yoke", "pedal", "trim_wheel",
            "trim_handle", "trim_indicator", "trim_ele", "trim_roll", "trim_yaw",
            "lever", "handle", "steering", "steer", "gearstrut",
            "gear_strut", "strut", "bogie", "axle", "tailwheel", "tail_wheel",
            "nosewheel", "nose_wheel", "door", "linkage", "control_line",
            "rudderline", "rudder_line", "cable")) {
            return true;
        }
        return name.matches("^(ng|mg|mlg|nlg)(_|[0-9]).*")
            || name.matches("^(st|sys|lts|eng)_.+")
            || name.matches("^con(rud|stick|yoke|flap|spoiler).*")
            || name.matches("^c_(rud|stick|yoke|flap|spoiler).*");
    }

    static boolean isWaterRudderName(String name) {
        return containsAny(name,
            "float_rudder", "floatrudder", "pontoonrudder", "pontoon_rudder",
            "water_rudder", "waterrudder");
    }

    static boolean containsAny(String value, String... terms) {
        for (String term : terms) {
            if (value.contains(term)) {
                return true;
            }
        }
        return false;
    }

    static String normalize(String value) {
        return value == null ? "" : value.toLowerCase(Locale.ROOT).replace('-', '_').replace(' ', '_');
    }
}
