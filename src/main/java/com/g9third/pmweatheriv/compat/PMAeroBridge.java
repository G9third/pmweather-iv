package com.g9third.pmweatheriv.compat;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.Level;
import org.joml.Vector3d;
import com.g9third.pmweatheriv.physics.Vec3d;

/** Required PMAero 1.0 packed API. Failures stop the load update; no second physics law exists. */
public final class PMAeroBridge {
    public static final int WIND_STRIDE = 13;
    public static final int VECTOR_STRIDE = 3;
    public static final int LIFT_INPUT_STRIDE = 35;
    public static final int LIFT_OUTPUT_STRIDE = 15;
    private static final MethodHandle WIND;
    private static final MethodHandle WIND_VECTORS;
    private static final MethodHandle BODY;
    private static final MethodHandle LIFT;
    private static volatile MethodHandle PARTICLE_WIND;
    private static volatile boolean particleWindResolved;

    static {
        try {
            ClassLoader loader = PMAeroBridge.class.getClassLoader();
            Class<?> wind = Class.forName("com.axes.pmweather_aeronautics.PMWeatherWindApi", false, loader);
            Class<?> body = Class.forName("com.axes.pmweather_aeronautics.ExternalAirframeBodyApi", false, loader);
            Class<?> lift = Class.forName("com.axes.pmweather_aeronautics.ExternalLiftingSurfaceApi", false, loader);
            if (wind.getField("WIND_IMPLEMENTATION_REVISION").getInt(null) != 3
                || wind.getField("VECTOR_RESULT_STRIDE").getInt(null) != VECTOR_STRIDE
                || lift.getField("API_VERSION").getInt(null) != 2
                || body.getField("API_VERSION").getInt(null) != 2
                || wind.getField("API_VERSION").getInt(null) != 2
                || wind.getField("PACKED_RESULT_STRIDE").getInt(null) != WIND_STRIDE
                || body.getField("INPUT_STRIDE").getInt(null) != bodyInputStride()
                || body.getField("OUTPUT_HEADER_STRIDE").getInt(null) != bodyOutputHeaderStride()
                || body.getField("OUTPUT_PATCH_STRIDE").getInt(null) != bodyOutputPatchStride()
                || lift.getField("INPUT_STRIDE").getInt(null) != LIFT_INPUT_STRIDE
                || lift.getField("OUTPUT_STRIDE").getInt(null) != LIFT_OUTPUT_STRIDE) {
                throw new IllegalStateException("Unsupported PMAero packed API version or layout");
            }
            MethodHandles.Lookup lookup = MethodHandles.publicLookup();
            WIND = lookup.unreflect(wind.getMethod("sampleAircraftAtmosphereInto", ServerLevel.class, double[].class, double[].class));
            WIND_VECTORS = lookup.unreflect(wind.getMethod("sampleAircraftWindInto",
                ServerLevel.class, double[].class, double[].class));
            BODY = lookup.unreflect(body.getMethod("evaluatePackedInto", double[].class, double[].class,
                double.class, double.class, double.class, double.class, double.class,
                double.class, double.class, double.class, double.class, double.class));
            LIFT = lookup.unreflect(lift.getMethod("evaluatePackedInto", double[].class, double[].class));
        } catch (ReflectiveOperationException e) {
            throw new ExceptionInInitializerError(new IllegalStateException(
                "PMWeather-IV requires PMWeather Aeronautics 1.0 wind revision 3 and compatible packed APIs", e));
        }
    }

    private PMAeroBridge() {}
    public static void sampleAircraftWindInto(ServerLevel level, double[] xyz, double[] output) {
        try {
            if (!(boolean) WIND_VECTORS.invokeExact(level, xyz, output)) {
                throw new IllegalStateException("PMAero rejected the aircraft wind batch");
            }
        } catch (Throwable error) {
            throw apiFailure("vector wind", error);
        }
        for (double value : output) if (!Double.isFinite(value))
            throw new IllegalStateException("PMAero returned non-finite wind");
    }

    public static void requireApis() { /* Class initialization validates all three APIs. */ }

    /** Optional client-only particle bridge; unavailable PMAero client classes leave motion untouched. */
    public static boolean applyClientParticleWind(Level level, Object identity,
            double x, double y, double z, Vector3d velocity, double response) {
        if (level == null || !level.isClientSide || identity == null || velocity == null) return false;
        MethodHandle bridge = particleWindHandle();
        if (bridge == null) return false;
        try {
            return (boolean) bridge.invokeExact(level, identity, x, y, z, velocity, response);
        } catch (Throwable ignored) {
            return false;
        }
    }

    private static MethodHandle particleWindHandle() {
        if (particleWindResolved) return PARTICLE_WIND;
        synchronized (PMAeroBridge.class) {
            if (particleWindResolved) return PARTICLE_WIND;
            try {
                ClassLoader loader = PMAeroBridge.class.getClassLoader();
                Class<?> client = Class.forName("com.axes.pmweather_aeronautics.ParticleWindClient", false, loader);
                PARTICLE_WIND = MethodHandles.publicLookup().unreflect(client.getMethod("applyParticleWind",
                    Level.class, Object.class, double.class, double.class, double.class, Vector3d.class, double.class));
            } catch (ReflectiveOperationException | RuntimeException | LinkageError ignored) {
                PARTICLE_WIND = null;
            }
            particleWindResolved = true;
            return PARTICLE_WIND;
        }
    }

    public static void sampleAircraftAtmosphereInto(ServerLevel level, double[] xyz, double[] output) {
        try {
            if (!(boolean) WIND.invokeExact(level, xyz, output)) throw new IllegalStateException("Invalid PMAero wind batch");
        } catch (Throwable failure) { throw apiFailure("wind", failure); }
        for (int i = 0; i < output.length; i += WIND_STRIDE) {
            if (!Double.isFinite(output[i]) || !Double.isFinite(output[i + 1]) || !Double.isFinite(output[i + 2])) {
                throw new IllegalStateException("PMAero returned non-finite wind");
            }
        }
    }

    public static void evaluateLiftInto(double[] input, double[] output) {
        try {
            if (!(boolean) LIFT.invokeExact(input, output)) throw new IllegalStateException("Invalid PMAero lifting surface");
        } catch (Throwable failure) { throw apiFailure("lift", failure); }
    }

    public static boolean evaluateExternalBodyInto(double[] input, double[] output, double density,
            double axialCd, double crossflowCd, double width, double height, double length,
            double wettedArea, Vec3d centerOfMass) {
        try {
            if (!(boolean) BODY.invokeExact(input, output, density, axialCd, crossflowCd, width, height, length,
                    wettedArea, centerOfMass.x(), centerOfMass.y(), centerOfMass.z())) {
                throw new IllegalStateException("Invalid PMAero body pressure");
            }
        } catch (Throwable failure) { throw apiFailure("body", failure); }
        for (double value : output) if (!Double.isFinite(value)) {
            throw new IllegalStateException("PMAero returned non-finite body pressure");
        }
        return true;
    }

    public static int bodyInputStride() { return 11; }
    public static int bodyOutputHeaderStride() { return 6; }
    public static int bodyOutputPatchStride() { return 5; }

    private static IllegalStateException apiFailure(String operation, Throwable failure) {
        if (failure instanceof Error error) throw error;
        return new IllegalStateException("PMAero " + operation + " API failed", failure);
    }
}
