import java.lang.reflect.Method;
import java.util.List;
import java.util.Map;
import java.util.Set;
import minecrafttransportsimulator.jsondefs.JSONAnimatedObject;
import minecrafttransportsimulator.jsondefs.JSONAnimationDefinition;
import minecrafttransportsimulator.jsondefs.JSONAnimationDefinition.AnimationComponentType;

/** Focused checks for which IV animation types may supply aerodynamic-control evidence. */
public final class AnimationEvidenceRegression {
    private static final String PHYSICS = "com.g9third.pmweatheriv.physics.";

    private AnimationEvidenceRegression() {
    }

    private static Method method(Class<?> owner, String name, Class<?>... parameterTypes) throws Exception {
        Method method = owner.getDeclaredMethod(name, parameterTypes);
        method.setAccessible(true);
        return method;
    }

    private static void require(boolean condition, String message) {
        if (!condition) {
            throw new AssertionError(message);
        }
    }

    public static void main(String[] args) throws Exception {
        Class<?> hintType = Class.forName(PHYSICS + "ModelGeometryData$AnimationHint");
        Method fromAnimationType = method(hintType, "fromAnimationType", String.class, String.class);
        Method combine = method(hintType, "combine", hintType);
        Method anySurfaceControl = method(hintType, "anySurfaceControl");
        Method elevator = method(hintType, "elevator");
        Method aileron = method(hintType, "aileron");
        Method flap = method(hintType, "flap");

        // A state-gated fairing name may mention a control without being a surface.
        Object visibilityFairing = fromAnimationType.invoke(null, "VISIBILITY", "show_elevator");
        require(!(boolean) anySurfaceControl.invoke(visibilityFairing),
            "visibility-only fairing promoted from control token");
        require(!(boolean) elevator.invoke(visibilityFairing),
            "visibility-only elevator token retained as animation evidence");

        // Physical rotations and translations can both carry authored control evidence.
        Object physicalElevator = fromAnimationType.invoke(null, "ROTATION", "elevator");
        require((boolean) elevator.invoke(physicalElevator), "rotation elevator evidence missing");
        Object physicalFlap = fromAnimationType.invoke(null, "TRANSLATION", "flap_position");
        require((boolean) flap.invoke(physicalFlap), "translation flap evidence missing");

        // Mixed physical controls remain meaningful, while a nonphysical token cannot
        // add another control to an otherwise valid physical animation set.
        Object physicalAileron = fromAnimationType.invoke(null, "rotation", "aileron");
        Object mixedPhysical = combine.invoke(physicalElevator, physicalAileron);
        require((boolean) elevator.invoke(mixedPhysical) && (boolean) aileron.invoke(mixedPhysical),
            "mixed physical pitch+roll evidence was lost");
        Object mixedWithVisibility = combine.invoke(physicalElevator, visibilityFairing);
        require((boolean) elevator.invoke(mixedWithVisibility)
                && !(boolean) aileron.invoke(mixedWithVisibility),
            "visibility token contaminated physical mixed-control evidence");

        Object colorAileron = fromAnimationType.invoke(null, "COLOR", "aileron_color");
        Object soundRudder = fromAnimationType.invoke(null, "SOUND", "rudder_sound");
        require(!(boolean) anySurfaceControl.invoke(colorAileron), "color animation supplied control evidence");
        require(!(boolean) anySurfaceControl.invoke(soundRudder), "sound animation supplied control evidence");

        // Explicit semantic object names remain an independent classification fallback.
        Class<?> surfaceKind = Class.forName(PHYSICS + "ModelSurfaceMap$SurfaceKind");
        Object namedElevator = method(surfaceKind, "fromSpecificName", String.class).invoke(null, "elevator");
        require(namedElevator != null && "ELEVATOR".equals(namedElevator.toString()),
            "explicit elevator name fallback was removed");

        inheritedCollisionMotion();

        System.out.println("AnimationEvidenceRegression: PASS");
    }

    private static void inheritedCollisionMotion() throws Exception {
        JSONAnimatedObject front = object("front", null, AnimationComponentType.ROTATION);
        JSONAnimatedObject skin = object("front.001", "front", AnimationComponentType.VISIBILITY);
        JSONAnimatedObject staticParent = object("fixed", null, AnimationComponentType.VISIBILITY);
        JSONAnimatedObject staticSkin = object("fixed_skin", "fixed", AnimationComponentType.VISIBILITY);
        JSONAnimatedObject unresolved = object("unresolved", "missing", AnimationComponentType.VISIBILITY);
        JSONAnimatedObject cycle = object("cycle", "cycle", AnimationComponentType.VISIBILITY);
        JSONAnimatedObject grandchild = object("grandchild", "front.001", AnimationComponentType.VISIBILITY);
        Class<?> hull = Class.forName("com.g9third.pmweatheriv.sable.SableModelCollisionHull");
        Object result = method(hull, "classifyAnimatedObjects", Map.class).invoke(null, Map.of(
            "front", front, "front.001", skin, "fixed", staticParent, "fixed_skin", staticSkin,
            "unresolved", unresolved, "cycle", cycle, "grandchild", grandchild));
        @SuppressWarnings("unchecked")
        Set<String> moving = (Set<String>) method(result.getClass(), "physicalTransformNames").invoke(result);
        require(moving.contains("front.001"), "visibility-only bike skin froze its inherited steering hinge");
        require(moving.contains("grandchild"), "deep applyAfter physical motion was lost");
        require(moving.contains("front"), "own physical motion was lost");
        require(!moving.contains("fixed") && !moving.contains("fixed_skin"), "state-only chain lost rigid shell eligibility");
        require(moving.contains("unresolved"), "missing parent incorrectly proved a static pose");
        require(moving.contains("cycle"), "cyclic chain incorrectly proved a static pose");
    }

    private static JSONAnimatedObject object(String name, String parent, AnimationComponentType type) {
        JSONAnimationDefinition animation = new JSONAnimationDefinition();
        animation.animationType = type;
        JSONAnimatedObject object = new JSONAnimatedObject();
        object.objectName = name; object.applyAfter = parent; object.animations = List.of(animation);
        return object;
    }
}
