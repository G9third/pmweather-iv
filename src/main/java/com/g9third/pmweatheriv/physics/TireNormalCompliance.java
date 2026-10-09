package com.g9third.pmweatheriv.physics;

import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartGroundDevice;
import minecrafttransportsimulator.entities.instances.APart;

/** Implicit damped tire/support deflection; no tensile normal force or attitude target. */
public final class TireNormalCompliance {
    private TireNormalCompliance() {}
    public record Response(double targetMps, double softnessInverseKg, double stiffnessNpm,
                           double dampingNsPm, double nominalDeflectionMeters,
                           double hardNormalVelocityTargetMps) {
        public Response(double targetMps, double softnessInverseKg, double stiffnessNpm,
                        double dampingNsPm, double nominalDeflectionMeters) {
            this(targetMps, softnessInverseKg, stiffnessNpm, dampingNsPm,
                nominalDeflectionMeters, Double.NEGATIVE_INFINITY);
        }
    }
    public static int installedSupportCount(EntityVehicleF_Physics vehicle) {
        int count=0;
        for (APart part:vehicle.allParts) if (part instanceof PartGroundDevice d
            && TireContactMaterial.activeSupport(d)
            && (d.definition.ground.isWheel || d.definition.ground.isTread)) ++count;
        return Math.max(1,count);
    }
    public static Response evaluate(PartGroundDevice device, double mass, int installed,
                                    double gap, double rigidTarget, double skin, double dt) {
        // Compatibility path retains the previous rigid bump-depth transition.
        return evaluate(device, mass, installed, gap, rigidTarget, skin, dt, false);
    }
    public static Response evaluate(PartGroundDevice device, double mass, int installed,
                                    double gap, double rigidTarget, double skin, double dt,
                                    boolean continuousSolidBumpstop) {
        return evaluate(device, mass, installed, gap, rigidTarget, skin, dt,
            continuousSolidBumpstop, false);
    }
    public static Response evaluate(PartGroundDevice device, double mass, int installed,
                                    double gap, double rigidTarget, double skin, double dt,
                                    boolean continuousSolidBumpstop, boolean trueImpactResidual) {
        if (device==null || !device.definition.ground.isWheel || device.flatVar.isActive)
            return rigid(rigidTarget);
        double deflection=nominalDeflection(device);
        return response(mass/Math.max(1,installed),deflection,gap,rigidTarget,skin,dt,
            continuousSolidBumpstop, trueImpactResidual);
    }
    public static double nominalDeflection(PartGroundDevice device) {
        if (device == null || device.definition == null || device.definition.ground == null
            || !device.definition.ground.isWheel || device.flatVar.isActive) return 0.0;
        return Vec3d.clamp(Math.max(0, device.getHeight()) * 0.04, 0.008, 0.040);
    }

    /** Series tire/suspension response; still uses the shared unilateral normal impulse. */
    public static Response evaluateRoadSuspension(RoadSuspensionModel.Parameters parameters,
            double complianceGap, double rigidTarget, double skin, double dt,
            boolean trueImpactResidual) {
        if (parameters == null || !parameters.finite() || !(dt > 0.0)
            || !Double.isFinite(complianceGap) || !Double.isFinite(skin)) return rigid(rigidTarget);
        // A material impact's explicit negative residual remains the exact hard
        // contact target and cannot be absorbed by the fallback suspension.
        if (trueImpactResidual) return rigid(rigidTarget);
        double stiffness = parameters.stiffnessNpm();
        double damping = parameters.dampingNsPm();
        double denominator = damping + dt * stiffness;
        if (!(denominator > 0.0) || !Double.isFinite(denominator)) return rigid(rigidTarget);
        double maximumCompression = parameters.maxTotalCompressionMeters();
        double compression = Math.min(skin - complianceGap, maximumCompression);
        double hardPenetration = Math.max(0.0, maximumCompression - skin);
        double hardTarget = -Math.max(0.0, complianceGap + hardPenetration) / dt;
        double springTarget = stiffness * compression / denominator;
        return new Response(springTarget, 1.0 / (dt * denominator), stiffness, damping,
            parameters.totalDeflectionMeters(), hardTarget);
    }
    /** Deep penetration is a pose error, not an additional source of tire grip. */
    public static double maximumSolidPenetration(double nominalDeflection, double skin) {
        return Math.max(0.005, 3.0 * Math.max(0.0, nominalDeflection) - Math.max(0.0, skin));
    }
    public static double upwardPoseCorrection(double gap, double nominalDeflection, double skin) {
        if (!Double.isFinite(gap) || !Double.isFinite(nominalDeflection) || !Double.isFinite(skin)) return 0.0;
        return Math.max(0.0, -gap - maximumSolidPenetration(nominalDeflection, skin));
    }

    /** The installed count stays fixed during unloading; a lifted wheel cannot stiffen the others. */
    public static Response response(double cornerMass, double deflection, double gap,
                                    double rigidTarget, double skin, double dt) {
        // Compatibility overload preserves the old transition for callers that
        // do not explicitly opt into solid-tire bumpstop support.
        return response(cornerMass, deflection, gap, rigidTarget, skin, dt, false);
    }
    public static Response response(double cornerMass, double deflection, double gap,
                                    double rigidTarget, double skin, double dt,
                                    boolean continuousSolidBumpstop) {
        return response(cornerMass, deflection, gap, rigidTarget, skin, dt,
            continuousSolidBumpstop, false);
    }
    public static Response response(double cornerMass, double deflection, double gap,
                                    double rigidTarget, double skin, double dt,
                                    boolean continuousSolidBumpstop, boolean trueImpactResidual) {
        if (!(cornerMass>0) || !(deflection>0) || !(dt>0) || !Double.isFinite(gap)
            || (!continuousSolidBumpstop && (gap>skin || gap<skin-3*deflection))) {
            return rigid(rigidTarget);
        }
        // Explicit material True-Impact residuals retain their exact negative
        // velocity target. Ordinary predictive-crossing targets remain elastic.
        if (continuousSolidBumpstop && trueImpactResidual) return rigid(rigidTarget);
        double k=cornerMass*9.80665/deflection;
        double c=2*0.75*Math.sqrt(k*cornerMass);
        double denominator=c+dt*k;
        // J=dt*[k*(skin-gap-dt*v)-c*v], solved implicitly with rigid-body velocity.
        double compression=continuousSolidBumpstop ? skin-gap : Math.max(0.0,skin-gap);
        if (continuousSolidBumpstop) compression=Math.min(compression,3.0*deflection);
        double hardTarget=continuousSolidBumpstop
            ? -Math.max(0.0,gap+maximumSolidPenetration(deflection,skin))/dt
            : Double.NEGATIVE_INFINITY;
        return new Response(k*compression/denominator,1/(dt*denominator),k,c,deflection,hardTarget);
    }
    private static Response rigid(double target) {
        return new Response(target,0,0,0,0,Double.NEGATIVE_INFINITY);
    }
}
