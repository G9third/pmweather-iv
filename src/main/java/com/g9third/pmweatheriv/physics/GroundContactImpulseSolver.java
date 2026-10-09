package com.g9third.pmweatheriv.physics;

import java.util.List;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import static com.g9third.pmweatheriv.physics.FlightMath.toLocal;
import static com.g9third.pmweatheriv.physics.FlightMath.toWorld;

/** Coupled unilateral normal support and bounded tires at their full 3D lever arms. */
public final class GroundContactImpulseSolver {
    private GroundContactImpulseSolver() {}
    public record Contact(Vec3d leverBody, Vec3d forwardWorld, double motiveMu, double lateralMu,
                          double normalVelocityTarget, boolean driven, double rollingCoefficient, boolean freeRolling,
                          double normalSoftnessInverseKg, double driveCoefficientLimit,
                          double demandedDriveImpulseNs, double hardNormalVelocityTarget) {
        /** Existing rigid/compliant contacts keep their original single-target behavior. */
        public Contact(Vec3d leverBody, Vec3d forwardWorld, double motiveMu, double lateralMu,
                       double normalVelocityTarget, boolean driven, double rollingCoefficient, boolean freeRolling,
                       double normalSoftnessInverseKg, double driveCoefficientLimit,
                       double demandedDriveImpulseNs) {
            this(leverBody, forwardWorld, motiveMu, lateralMu, normalVelocityTarget, driven,
                rollingCoefficient, freeRolling, normalSoftnessInverseKg, driveCoefficientLimit,
                demandedDriveImpulseNs, Double.NEGATIVE_INFINITY);
        }
    }
    public record Result(Vec3d velocityWorld, Vec3d angularVelocityBody,
                         double normalImpulseNs, double yawCapacityNm, boolean saturated,
                         double[] normalImpulses, double[] longitudinalImpulses, double[] lateralImpulses) {}

    public static Result solve(Vec3d velocity, Vec3d omega, Vec3d inertia,
                               RotationMatrix orientation, Vec3d nativeDriveDirection,
                               double mass, double brake, double driveImpulseNs, double dt,
                               List<Contact> contacts) {
        if (!(mass > 0) || !(dt > 0) || !Double.isFinite(driveImpulseNs)
            || !velocity.isFinite() || !omega.isFinite() || !inertia.isFinite())
            throw new IllegalArgumentException("Invalid coupled ground solver input");
        // Replace only IV's wheel traction increment, preserving weather and native drag.
        Vec3d baseVelocity = velocity.subtract(nativeDriveDirection.scale(driveImpulseNs/mass));
        int count = contacts.size(), variables = count*3, driven = 0;
        for (Contact contact : contacts) if (contact.driven()) ++driven;
        Vec3d[] direction = new Vec3d[variables], jacobian = new Vec3d[variables];
        double[] residual = new double[variables], impulse = new double[variables];

        Vec3d omegaWorld = toWorld(orientation, omega);
        for (int i=0; i<count; ++i) {
            Contact contact = contacts.get(i);
            Vec3d f = contact.forwardWorld().normalized();
            direction[i*3] = new Vec3d(0,1,0);
            direction[i*3+1] = f;
            direction[i*3+2] = new Vec3d(f.z(),0,-f.x()).normalized();
            Vec3d v = baseVelocity.add(omegaWorld.cross(toWorld(orientation, contact.leverBody())));
            for (int axis=0; axis<3; ++axis) {
                int index=i*3+axis;
                jacobian[index]=contact.leverBody().cross(toLocal(orientation,direction[index]));
                residual[index]=v.dot(direction[index])-(axis==0 ? contact.normalVelocityTarget() : 0);
            }
        }
        double maximumRow=0;
        for (int row=0; row<variables; ++row) {
            double rowSum=0;
            for (int column=0; column<variables; ++column) {
                Vec3d a=jacobian[row], b=jacobian[column];
                double entry=direction[row].dot(direction[column])/mass
                    + a.x()*b.x()/inertia.x()+a.y()*b.y()/inertia.y()+a.z()*b.z()/inertia.z();
                rowSum+=Math.abs(entry);
            }
            if (row%3==0) rowSum+=Math.max(0,contacts.get(row/3).normalSoftnessInverseKg());
            maximumRow=Math.max(maximumRow,rowSum);
        }
        double step=maximumRow>1e-12 ? 0.8/maximumRow : 0;
        brake=Vec3d.clamp(brake,0,1);
        // Simultaneous updates avoid first-wheel yaw bias. Reproject accumulated
        // impulses as normal load transfers: iterations never grant another dt.
        double[] next=new double[variables];
        for (int iteration=0; iteration<192; ++iteration) {
            // K has rank at most six. Evaluate K*J through the net rigid-body
            // impulse, avoiding a dense matrix multiplication for multi-point skids.
            double vx=0,vy=0,vz=0,ax=0,ay=0,az=0;
            for (int column=0; column<variables; ++column) {
                double j=impulse[column]; Vec3d d=direction[column], a=jacobian[column];
                vx+=d.x()*j; vy+=d.y()*j; vz+=d.z()*j;
                ax+=a.x()*j; ay+=a.y()*j; az+=a.z()*j;
            }
            vx/=mass; vy/=mass; vz/=mass;
            ax/=inertia.x(); ay/=inertia.y(); az/=inertia.z();
            for (int row=0; row<variables; ++row) {
                Vec3d d=direction[row], a=jacobian[row];
                double speed=residual[row]+d.x()*vx+d.y()*vy+d.z()*vz+a.x()*ax+a.y()*ay+a.z()*az;
                double candidate=impulse[row]-step*speed;
                if (row%3==0) {
                    Contact contact=contacts.get(row/3);
                    double gamma=Math.max(0,contact.normalSoftnessInverseKg());
                    speed+=gamma*impulse[row];
                    candidate=impulse[row]-step*speed;
                    double hardTarget=contact.hardNormalVelocityTarget();
                    if (Double.isFinite(hardTarget)) {
                        hardTarget=Math.min(0.0,hardTarget);
                        // The soft spring and bumpstop share one unilateral normal
                        // impulse. Apply whichever constraint requires more support.
                        double hardSpeed=speed-gamma*impulse[row]
                            +contact.normalVelocityTarget()-hardTarget;
                        candidate=Math.max(candidate,impulse[row]-step*hardSpeed);
                    }
                }
                next[row]=candidate;
            }
            double change=0;
            for (int i=0; i<count; ++i) {
                Contact c=contacts.get(i);
                int n=i*3, f=n+1, s=n+2;
                next[n]=Math.max(0,next[n]); // Tires push upward; they cannot pull the road.
                double capF=Math.max(0,c.motiveMu()*next[n]);
                double capS=Math.max(0,c.lateralMu()*next[n]);
                double driveCap=Double.isFinite(c.driveCoefficientLimit())
                    ? Math.min(capF,Math.max(0,c.driveCoefficientLimit())*next[n]) : capF;
                double requestedDrive = Double.isFinite(c.demandedDriveImpulseNs())
                    ? c.demandedDriveImpulseNs() : driven > 0 ? driveImpulseNs / driven : 0.0;
                double drive=c.driven() && driven>0
                    ? Vec3d.clamp(requestedDrive,-driveCap,driveCap) : 0;
                double command=c.freeRolling() ? brake : 1;
                double passive=Math.min(capF, command*capF+c.rollingCoefficient()*next[n]);
                projectTangentialImpulse(next, f, s, capF, capS,
                    Math.max(-capF,drive-passive), Math.min(capF,drive+passive));
            }
            for (int j=0; j<variables; ++j) change=Math.max(change,Math.abs(next[j]-impulse[j]));
            double[] previous=impulse; impulse=next; next=previous;
            if (change<1e-7) break;
        }
        Vec3d total=Vec3d.ZERO, angular=Vec3d.ZERO;
        double normal=0, capacity=0;
        double[] normals=new double[count], longitudinal=new double[count], lateral=new double[count];
        for (int i=0; i<count; ++i) {
            Contact c=contacts.get(i);
            normals[i]=impulse[i*3]; longitudinal[i]=impulse[i*3+1]; lateral[i]=impulse[i*3+2];
            normal+=normals[i];
            for (int axis=0; axis<3; ++axis) {
                int index=i*3+axis;
                total=total.add(direction[index].scale(impulse[index]));
                angular=angular.add(jacobian[index].scale(impulse[index]));
            }
            double capF=c.motiveMu()*normals[i];
            double used=capF>1e-12 ? longitudinal[i]/capF : 0;
            capacity+=Math.abs(toWorld(orientation,jacobian[i*3+2]).y())
                *c.lateralMu()*normals[i]*Math.sqrt(Math.max(0,1-used*used))/dt;
        }
        Vec3d solved=baseVelocity.add(total.scale(1/mass));
        Vec3d solvedOmega=omega.add(inverseInertia(angular,inertia));
        boolean saturated=false;
        Vec3d solvedOmegaWorld=toWorld(orientation,solvedOmega);
        for (int i=0; i<count; ++i) {
            if (normals[i]<=1e-8) continue;
            Vec3d v=solved.add(solvedOmegaWorld.cross(toWorld(orientation,contacts.get(i).leverBody())));
            if (Math.abs(v.dot(direction[i*3+2]))>0.02
                || ((brake>0 || !contacts.get(i).freeRolling()) && Math.abs(v.dot(direction[i*3+1]))>0.02)) saturated=true;
        }
        if (!solved.isFinite() || !solvedOmega.isFinite()) throw new IllegalStateException("Non-finite ground contact result");
        return new Result(solved,solvedOmega,normal,capacity,saturated,normals,longitudinal,lateral);
    }
    /**
     * Euclidean projection of one gradient step onto the ellipse intersected
     * with the drive/brake longitudinal interval. Both slip directions spend
     * grip together: clamping longitudinal first would erase lateral braking.
     */
    private static void projectTangentialImpulse(double[] values, int f, int s,
            double capF, double capS, double lowerF, double upperF) {
        double wantedF=values[f], wantedS=values[s];
        if (!(capF>1e-12)) {
            values[f]=0;
            values[s]=Vec3d.clamp(wantedS,-capS,capS);
            return;
        }
        if (!(capS>1e-12)) {
            values[f]=Vec3d.clamp(wantedF,lowerF,upperF);
            values[s]=0;
            return;
        }
        double clippedF=Vec3d.clamp(wantedF,lowerF,upperF);
        if (ellipseNormSquared(clippedF,wantedS,capF,capS)<=1) {
            values[f]=clippedF; values[s]=wantedS;
            return;
        }
        double projectedF, projectedS;
        if (Math.abs(capF-capS)<=1e-9*Math.max(capF,capS)) {
            double scale=capF/Math.max(capF,Math.hypot(wantedF,wantedS));
            projectedF=wantedF*scale; projectedS=wantedS*scale;
        } else if (ellipseNormSquared(wantedF,wantedS,capF,capS)<=1) {
            projectedF=wantedF; projectedS=wantedS;
        } else {
            // Closest ellipse point satisfies J_i=Jwanted_i/(1+lambda/a_i²).
            // A bounded scalar root avoids an axial ordering bias or allocations.
            double a2=capF*capF, b2=capS*capS;
            double lo=0, hi=2*Math.max(capF*Math.abs(wantedF),capS*Math.abs(wantedS));
            for (int iteration=0; iteration<32; ++iteration) {
                double lambda=(lo+hi)*0.5;
                double jf=wantedF/(1+lambda/a2), js=wantedS/(1+lambda/b2);
                if (ellipseNormSquared(jf,js,capF,capS)>1) lo=lambda;
                else hi=lambda;
            }
            projectedF=wantedF/(1+hi/a2); projectedS=wantedS/(1+hi/b2);
        }
        // If the unconstrained ellipse projection violates the interval, the
        // nearest feasible point lies on that interval boundary. Only then is
        // the other component clipped to its remaining ellipse cross-section.
        double acceptedF=Vec3d.clamp(projectedF,lowerF,upperF);
        double remaining=capS*Math.sqrt(Math.max(0,1-(acceptedF/capF)*(acceptedF/capF)));
        values[f]=acceptedF;
        values[s]=Vec3d.clamp(projectedS,-remaining,remaining);
        if (acceptedF!=projectedF) values[s]=Vec3d.clamp(wantedS,-remaining,remaining);
    }
    private static double ellipseNormSquared(double f, double s, double capF, double capS) {
        double x=f/capF, y=s/capS;
        return x*x+y*y;
    }
    private static Vec3d inverseInertia(Vec3d v, Vec3d inertia) {
        return new Vec3d(v.x()/Math.max(1e-6,inertia.x()),v.y()/Math.max(1e-6,inertia.y()),v.z()/Math.max(1e-6,inertia.z()));
    }
}
