package com.g9third.pmweatheriv.physics;

import java.util.List;
import static com.g9third.pmweatheriv.physics.FlightMath.ROTOR_DISC_AIRFLOW_ASYMMETRY_BLEND;

/**
 * Pure rotor-disc aerodynamic load distribution used by the common Sable
 * air-vehicle path.
 *
 * <p>This class intentionally knows nothing about Minecraft, IV entities,
 * PMWeather queries, Sable handles, or pilot control mapping. The owner-tick
 * actuator supplies total authored thrust/RPM/pitch; {@link AircraftPhysics}
 * supplies the current per-substep local air velocity at the four disc
 * stations. In 0.11.0 this component is used only to estimate the relative
 * edge loading. RotorModel subtracts a matched uniform-hub-flow baseline before
 * converting it into a bounded physical wind/shear disturbance moment.
 * Pilot cyclic is handled separately by {@link RotorModel}'s body-rate command
 * controller.</p>
 */
final class RotorcraftAerodynamics {
    private static final double EPSILON = 1.0E-9;
    private static final double MIN_EDGE_FLOW_MPS = 0.5;

    private RotorcraftAerodynamics() {
    }

    /** Local body-frame kinematics for one rotor edge station. */
    record EdgeKinematics(Vec3d radialBody, Vec3d relativeAirBody) {
    }

    /**
     * Returns four non-negative axial forces ordered U+, U-, V+, V-. Their sum
     * is exactly {@code totalThrustNewtons} (within floating-point roundoff).
     *
     * <p>Only local aerodynamic flow redistributes the reference load around
     * the disc. Pilot cyclic is deliberately absent from this calculation.</p>
     */
    static double[] distributeMainRotorForces(
        double totalThrustNewtons,
        double airDensity,
        double rotorRpm,
        double collectivePitchDegrees,
        Vec3d rotorAxisBody,
        Vec3d liftAxisBody,
        List<EdgeKinematics> edges
    ) {
        double target = Math.max(0.0, finiteOrZero(totalThrustNewtons));
        double[] forces = new double[4];
        if (target <= EPSILON) {
            return forces;
        }

        double weightSum = 0.0;
        for (int index = 0; index < forces.length; ++index) {
            EdgeKinematics edge = edges != null && index < edges.size()
                ? edges.get(index) : null;
            double weight = edgeLoadProxy(
                edge,
                rotorAxisBody,
                liftAxisBody,
                airDensity,
                rotorRpm,
                collectivePitchDegrees
            );
            forces[index] = weight;
            weightSum += weight;
        }
        if (!(weightSum > EPSILON) || !Double.isFinite(weightSum)) {
            double equal = target / forces.length;
            for (int index = 0; index < forces.length; ++index) {
                forces[index] = equal;
            }
        } else {
            for (int index = 0; index < forces.length; ++index) {
                forces[index] = target * forces[index] / weightSum;
            }
        }

        normalizeNonNegative(forces, target);

        // Retain only a modest fraction of the physical edge asymmetry. The
        // resulting deviations from equal loading are converted to a disturbance
        // torque by RotorModel; net main lift is applied separately at the CG.
        double blend = Vec3d.clamp(ROTOR_DISC_AIRFLOW_ASYMMETRY_BLEND, 0.0, 1.0);
        double equal = target / forces.length;
        for (int index = 0; index < forces.length; ++index) {
            forces[index] = equal + blend * (forces[index] - equal);
        }
        normalizeNonNegative(forces, target);
        return forces;
    }

    private static double edgeLoadProxy(
        EdgeKinematics edge,
        Vec3d rotorAxisBody,
        Vec3d liftAxisBody,
        double density,
        double rotorRpm,
        double collectivePitchDegrees
    ) {
        if (edge == null || edge.radialBody() == null
            || edge.relativeAirBody() == null) {
            return 1.0;
        }
        Vec3d radialBody = edge.radialBody();
        double radius = radialBody.length();
        if (radius <= EPSILON || !Double.isFinite(radius)) {
            return 1.0;
        }
        Vec3d axis = safeNormalized(rotorAxisBody, new Vec3d(0.0, 1.0, 0.0));
        Vec3d liftAxis = safeNormalized(liftAxisBody, new Vec3d(0.0, 1.0, 0.0));
        Vec3d tangentBody = safeNormalized(
            axis.cross(radialBody.scale(1.0 / radius)),
            new Vec3d(1.0, 0.0, 0.0)
        );
        double rotorOmega = finiteOrZero(rotorRpm) * (2.0 * Math.PI / 60.0);
        Vec3d bladeRelativeAirBody = edge.relativeAirBody().add(
            tangentBody.scale(rotorOmega * radius)
        );
        double tangential = bladeRelativeAirBody.dot(tangentBody);
        double axial = bladeRelativeAirBody.dot(liftAxis);
        double inflowAngle = Math.atan2(
            axial, Math.max(MIN_EDGE_FLOW_MPS, Math.abs(tangential))
        );
        double collectivePitch = Math.toRadians(finiteOrZero(collectivePitchDegrees));
        double incidenceFactor = Vec3d.clamp(
            1.0 + 0.75 * Math.sin(collectivePitch - inflowAngle),
            0.25, 1.75
        );
        double speedSquared = Math.max(
            MIN_EDGE_FLOW_MPS * MIN_EDGE_FLOW_MPS,
            bladeRelativeAirBody.lengthSquared()
        );
        return Math.max(1.0E-6,
            0.5 * Math.max(0.08, finiteOrZero(density)) * speedSquared * incidenceFactor
        );
    }

    private static void normalizeNonNegative(double[] forces, double targetTotal) {
        double sum = 0.0;
        for (int index = 0; index < forces.length; ++index) {
            if (!Double.isFinite(forces[index]) || forces[index] < 0.0) {
                forces[index] = 0.0;
            }
            sum += forces[index];
        }
        if (!(sum > EPSILON)) {
            double equal = targetTotal / Math.max(1, forces.length);
            for (int index = 0; index < forces.length; ++index) {
                forces[index] = equal;
            }
            return;
        }
        double scale = targetTotal / sum;
        for (int index = 0; index < forces.length; ++index) {
            forces[index] *= scale;
        }
    }

    private static Vec3d safeNormalized(Vec3d value, Vec3d fallback) {
        if (value == null || !value.isFinite() || value.lengthSquared() <= EPSILON) {
            return fallback;
        }
        return value.normalized();
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
