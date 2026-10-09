package com.g9third.pmweatheriv.sable;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import com.g9third.pmweatheriv.terrain.AircraftTerrainImpact;
import com.g9third.pmweatheriv.terrain.trueimpact.ExternalWorldImpactModel.Resolution;
import com.g9third.pmweatheriv.physics.Vec3d;
import com.g9third.pmweatheriv.physics.RigidContactSolver;
import dev.ryanhcode.sable.physics.config.dimension_physics.DimensionPhysicsData;
import java.util.ArrayList;
import it.unimi.dsi.fastutil.longs.LongOpenHashSet;
import java.util.List;
import minecrafttransportsimulator.systems.ConfigSystem;
import net.minecraft.core.BlockPos;
import org.joml.Quaterniond;
import org.joml.Vector3d;
import org.joml.Vector3dc;

/** Terrain CCD and material response for one Sable body.
 * Owns swept-contact evidence and clear-pose recovery. The body calls this at
 * the same pre/post integration boundaries and remains the lifecycle owner.
 */
final class SableTerrainResponse {
    private final SableVehicleBody body;
    SableTerrainResponse(SableVehicleBody body) { this.body = body; }

    final Vector3d substepStartPositionWorld = new Vector3d();
    final Quaterniond substepStartOrientationWorld = new Quaterniond();
    boolean hasSubstepStartPose;
    final List<SableVehicleBody.SweptTerrainImpact> pendingSweptTerrainImpacts = new ArrayList<>();
    final Vector3d temporaryCcdPredictedPositionWorld = new Vector3d();
    final Vector3d temporaryCcdPostGravityVelocityWorld = new Vector3d();
    final Quaterniond temporaryCcdPredictedOrientationWorld = new Quaterniond();
    final Quaterniond temporaryCcdDeltaOrientationWorld = new Quaterniond();
    final Vector3d lastClearTerrainPositionWorld = new Vector3d();
    final Quaterniond lastClearTerrainOrientationWorld = new Quaterniond();
    boolean hasClearTerrainPose;

    void queueSweptTerrainImpact(
        SableCompoundCollider.TerrainBoundaryImpact impact,
        double preImpactVehicleSpeedMetersPerSecond,
        double inwardPointSpeedMetersPerSecond,
        double normalImpulseNewtonSeconds,
        float preImpactBlockHardness,
        Resolution trueImpactResolution
    ) {
        if (impact == null || impact.block() == null
            || !Double.isFinite(preImpactVehicleSpeedMetersPerSecond)
            || !Double.isFinite(inwardPointSpeedMetersPerSecond)
            || !Double.isFinite(normalImpulseNewtonSeconds)) {
            return;
        }
        long gameTime = body.level.getGameTime();
        double preSpeed = Math.max(0.0, preImpactVehicleSpeedMetersPerSecond);
        double inward = Math.max(0.0, inwardPointSpeedMetersPerSecond);
        double impulse = Math.max(0.0, normalImpulseNewtonSeconds);
        for (int i = 0; i < pendingSweptTerrainImpacts.size(); ++i) {
            SableVehicleBody.SweptTerrainImpact existing = pendingSweptTerrainImpacts.get(i);
            if (existing.gameTime() == gameTime
                && sameBlock(existing.block(), impact.block())) {
                if (preSpeed > existing.preImpactVehicleSpeedMetersPerSecond() + 1.0E-9
                    || (Math.abs(preSpeed - existing.preImpactVehicleSpeedMetersPerSecond()) <= 1.0E-9
                        && (inward > existing.inwardPointSpeedMetersPerSecond() + 1.0E-9
                            || (Math.abs(inward - existing.inwardPointSpeedMetersPerSecond()) <= 1.0E-9
                                && impulse > existing.normalImpulseNewtonSeconds() + 1.0E-9)))) {
                    pendingSweptTerrainImpacts.set(i, new SableVehicleBody.SweptTerrainImpact(
                        gameTime, impact.block(), impact.sourceBox(),
                        impact.worldPoint() == null ? null : new Vector3d(impact.worldPoint()),
                        impact.worldNormal() == null ? null : new Vector3d(impact.worldNormal()),
                        impact.projectedAreaSquareMeters(),
                        preSpeed, inward, impulse, preImpactBlockHardness, trueImpactResolution
                    ));
                }
                return;
            }
        }
        if (pendingSweptTerrainImpacts.size() >= 16) {
            pendingSweptTerrainImpacts.remove(0);
        }
        pendingSweptTerrainImpacts.add(new SableVehicleBody.SweptTerrainImpact(
            gameTime, impact.block(), impact.sourceBox(),
            impact.worldPoint() == null ? null : new Vector3d(impact.worldPoint()),
            impact.worldNormal() == null ? null : new Vector3d(impact.worldNormal()),
            impact.projectedAreaSquareMeters(),
            preSpeed, inward, impulse, preImpactBlockHardness, trueImpactResolution
        ));
    }

    List<SableVehicleBody.SweptTerrainImpact> drainSweptTerrainImpacts() {
        if (pendingSweptTerrainImpacts.isEmpty()) {
            return List.of();
        }
        List<SableVehicleBody.SweptTerrainImpact> drained = List.copyOf(pendingSweptTerrainImpacts);
        pendingSweptTerrainImpacts.clear();
        return drained;
    }

    static boolean sameBlock(
        minecrafttransportsimulator.baseclasses.Point3D first,
        minecrafttransportsimulator.baseclasses.Point3D second
    ) {
        return first != null && second != null
            && (int) first.x == (int) second.x
            && (int) first.y == (int) second.y
            && (int) first.z == (int) second.z;
    }

    void rememberClearTerrainPose(
        Vector3dc positionWorld,
        Quaterniond orientationWorld
    ) {
        if (!SableVehicleBody.finite(positionWorld) || !SableVehicleBody.finite(orientationWorld)) {
            return;
        }
        lastClearTerrainPositionWorld.set(positionWorld);
        lastClearTerrainOrientationWorld.set(orientationWorld).normalize();
        hasClearTerrainPose = true;
    }

    /**
     * Conservative BODY CCD for mounted LevelColliders.
     *
     * <p>Sable 2.0.3 enables CCD on the parent dynamic body, but PMWeather-IV's
     * real IV cuboids are custom mounted LevelCollider shapes. Those shapes can
     * cross a thin Minecraft collision shape between Rapier substeps without a
     * contact manifold. Predict the completed semi-implicit pose (including the
     * scene gravity Rapier will add), sweep the exact mounted BODY cuboids to
     * that pose, and use the last clear fraction to remove only the inward
     * collision-producing velocity before Rapier integrates. Because mounted
     * LevelCollider nonlinear shape casts are not available, the post-substep
     * persistent-clear-pose invariant remains the hard boundary guarantee.
     * This prepass is collision mitigation, not a free-flight damper or speed
     * cap.</p>
     */
    void enforcePreIntegrationTerrainCcd(double timeStep) {
        long start = PMIVObserver.isCapturing(body.vehicle.uniqueUUID) ? System.nanoTime() : 0;
        try {
            enforcePreIntegrationTerrainCcdImpl(timeStep);
        } finally {
            if (start != 0) PMIVObserver.capturePerformance(body.vehicle.uniqueUUID,
                "terrainPreIntegration", System.nanoTime() - start);
        }
    }

    private void enforcePreIntegrationTerrainCcdImpl(double timeStep) {
        if (body.compoundCollider == null || !body.compoundCollider.isActive()
            || timeStep <= 0.0 || !hasSubstepStartPose) {
            return;
        }

        // Current handle velocity is pre-Rapier-gravity. Rapier integrates its
        // configured scene gravity during the step, so all material preflight is
        // evaluated against the same post-gravity velocity Rapier would receive.
        DimensionPhysicsData.getGravity(body.level, body.getPose().position(), body.sableGravityWorld);
        Vector3d originalPreGravityLinear = new Vector3d(body.temporaryLinearVelocityWorld);
        Vector3d originalAngular = new Vector3d(body.temporaryAngularVelocityWorld);
        Vector3d workingPostGravityLinear = new Vector3d(body.temporaryLinearVelocityWorld).add(
            body.sableGravityWorld.x * timeStep,
            body.sableGravityWorld.y * timeStep,
            body.sableGravityWorld.z * timeStep
        );
        Vector3d workingAngular = new Vector3d(body.temporaryAngularVelocityWorld);
        Vector3d startPosition = new Vector3d(body.getPose().position());
        Quaterniond startOrientation = new Quaterniond(body.cachedOrientation).normalize();
        LongOpenHashSet openedSeedBlocks = new LongOpenHashSet();
        SableTerrainSweep.TerrainQuerySession terrainQuery =
            body.compoundCollider.newTerrainQuerySession(body.level);
        int materialPasses = 0;
        int materialBlocksOpened = 0;
        int rigidPasses = 0;
        boolean repairedEmbeddedStart = false;
        boolean changedMomentum = false;

        // A compound aircraft can cross several independent terrain voxels/source
        // boxes in one 25 ms Sable substep. 0.11.8 resolved only the first swept
        // boundary, then let Rapier encounter the remaining nose/wing/belly contacts
        // as ordinary rigid manifolds. Re-sweep the complete compound after every
        // immediate fracture until the actual trajectory is clear or material really
        // survives. There is no arbitrary pass count: every continuation must have
        // removed at least one concrete world voxel, and the duplicate-seed guard
        // below terminates any no-progress world mutation.
        while (true) {
            temporaryCcdPostGravityVelocityWorld.set(workingPostGravityLinear);
            temporaryCcdPredictedPositionWorld.set(startPosition).fma(
                timeStep, workingPostGravityLinear
            );

            temporaryCcdPredictedOrientationWorld.set(startOrientation);
            double angularSpeed = workingAngular.length();
            if (angularSpeed > 1.0E-10 && Double.isFinite(angularSpeed)) {
                double angle = angularSpeed * timeStep;
                temporaryCcdDeltaOrientationWorld.rotationAxis(
                    angle,
                    workingAngular.x / angularSpeed,
                    workingAngular.y / angularSpeed,
                    workingAngular.z / angularSpeed
                );
                temporaryCcdPredictedOrientationWorld.set(
                    temporaryCcdDeltaOrientationWorld
                ).mul(startOrientation).normalize();
            }

            SableCompoundCollider.TerrainPoseRecovery recovery =
                terrainQuery.recoverDeepTerrainPenetration(
                    startPosition,
                    startOrientation,
                    temporaryCcdPredictedPositionWorld,
                    temporaryCcdPredictedOrientationWorld
                );
            if (!recovery.priorPoseClear() && recovery.terrainBoundaryViolated()
                && !repairedEmbeddedStart) {
                // Articulated geometry can invalidate yesterday's clear pose.
                // Repair the current pose before native penetration bias runs,
                // preserving momentum and the current orientation.
                var currentRecovery = terrainQuery.recoverDeepTerrainPenetration(
                    startPosition, startOrientation, startPosition, startOrientation);
                if (currentRecovery.recovered() && !currentRecovery.priorPoseClear()) {
                    startPosition.set(currentRecovery.resolvedPosition());
                    body.placeBodyWithVelocity(originalPreGravityLinear, originalAngular,
                        startOrientation, startPosition);
                    substepStartPositionWorld.set(startPosition);
                    rememberClearTerrainPose(startPosition, startOrientation);
                    repairedEmbeddedStart = true;
                    if (PMIVObserver.loggingEnabled()) PMIVObserver.log(
                        "SABLE_PRESTEP_EMBEDDED_POSE_RECOVERED gameTime=" + body.level.getGameTime()
                            + " uuid=" + body.vehicle.uniqueUUID
                            + " momentumPreserved=true nativePenetrationBiasAvoided=true");
                    continue;
                }
            }
            if (!recovery.terrainBoundaryViolated() || !recovery.recovered()
                || !recovery.priorPoseClear()) {
                break;
            }

            double safeFraction = Math.max(0.0, Math.min(1.0, recovery.resolvedFraction()));
            if (!(safeFraction < 1.0 - 1.0E-8)) {
                break;
            }
            double preImpactVehicleSpeedMps = workingPostGravityLinear.length();
            SableCompoundCollider.TerrainBoundaryImpact impact = recovery.boundaryImpact();
            if (impact == null) {
                impact = terrainQuery.findFirstTerrainImpactAlongPath(
                    startPosition,
                    startOrientation,
                    temporaryCcdPredictedPositionWorld,
                    temporaryCcdPredictedOrientationWorld
                );
            }
            if (impact == null || impact.block() == null || impact.worldPoint() == null
                || impact.worldNormal() == null) {
                break;
            }

            TerrainVelocityResolution rigidImpactResolution = resolveTerrainImpactVelocity(
                workingPostGravityLinear,
                workingAngular,
                startPosition,
                startOrientation,
                impact
            );

            float preImpactBlockHardness = body.vehicle.world.getBlockHardness(impact.block());
            Resolution trueImpactResolution = null;
            double targetResidualInwardSpeedMps = 0.0;
            double maxMaterialTravelMeters = 0.0;
            if (rigidImpactResolution.inwardPointSpeedBeforeMps() > 0.0
                && rigidImpactResolution.normalImpulseNewtonSeconds() > 0.0
                && body.terrainDamageAllowedForCurrentCrashEpisode()
                && PMWeatherIVConfig.get().enableBlockBreaking()
                && ConfigSystem.settings.damage.vehicleBlockBreaking.value) {
                // Only the portion of this substep after the geometric time of impact
                // is physically available for penetration along this contact normal.
                double remainingSubstepSeconds = timeStep * Math.max(
                    0.0, 1.0 - safeFraction
                );
                maxMaterialTravelMeters = Math.max(0.0,
                    rigidImpactResolution.inwardPointSpeedBeforeMps()
                        * remainingSubstepSeconds);
                trueImpactResolution = AircraftTerrainImpact.resolveExternalPenetration(
                    body.level,
                    new BlockPos((int) impact.block().x, (int) impact.block().y, (int) impact.block().z),
                    impact.worldPoint(), impact.worldNormal(),
                    impact.projectedAreaSquareMeters(),
                    rigidImpactResolution.normalImpulseNewtonSeconds(),
                    rigidImpactResolution.inwardPointSpeedBeforeMps(),
                    maxMaterialTravelMeters
                );
                if (trueImpactResolution != null && trueImpactResolution.accepted()) {
                    targetResidualInwardSpeedMps = Math.max(0.0,
                        trueImpactResolution.residualInwardSpeedMps());
                }
            }

            TerrainVelocityResolution impactResolution =
                trueImpactResolution != null && trueImpactResolution.accepted()
                    ? resolveTerrainImpactVelocity(
                        workingPostGravityLinear,
                        workingAngular,
                        startPosition,
                        startOrientation,
                        impact, targetResidualInwardSpeedMps
                    )
                    : rigidImpactResolution;
            impactResolution = applyTerrainContactFriction(impactResolution,
                startPosition, startOrientation, impact);

            boolean materialAccepted = trueImpactResolution != null && trueImpactResolution.accepted();
            if (!materialAccepted) {
                List<SableCompoundCollider.TerrainBoundaryImpact> contacts =
                    terrainQuery.rigidContactsAtPose(startPosition, startOrientation);
                contacts.addAll(terrainQuery.rigidContactsAtPose(
                    recovery.resolvedPosition(), recovery.resolvedOrientation()));
                impactResolution = resolveRigidContactManifold(workingPostGravityLinear,
                    workingAngular, startPosition, startOrientation, impact, contacts,
                    preImpactVehicleSpeedMps, safeFraction);
            }

            queueSweptTerrainImpact(
                impact, preImpactVehicleSpeedMps,
                impactResolution.inwardPointSpeedBeforeMps(),
                impactResolution.normalImpulseNewtonSeconds(),
                preImpactBlockHardness, trueImpactResolution
            );

            boolean momentumChangedThisPass = workingPostGravityLinear.distanceSquared(
                impactResolution.linearVelocityWorld()) > 1.0E-12
                || workingAngular.distanceSquared(impactResolution.angularVelocityWorld()) > 1.0E-12;
            workingPostGravityLinear.set(impactResolution.linearVelocityWorld());
            workingAngular.set(impactResolution.angularVelocityWorld());
            if (impactResolution.normalImpulseNewtonSeconds() > 1.0E-9) {
                changedMomentum = true;
            }

            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "SABLE_PRESTEP_TERRAIN_CCD_CONSTRAINED gameTime=" + body.level.getGameTime()
                    + " uuid=" + body.vehicle.uniqueUUID
                    + " predictedPosition=" + temporaryCcdPredictedPositionWorld
                    + " safePathFraction=" + Double.toString(safeFraction)
                    + " preImpactVehicleSpeedMps=" + Double.toString(preImpactVehicleSpeedMps)
                    + " constrainedPostGravitySpeedMps=" + Double.toString(workingPostGravityLinear.length())
                    + " impactNormal=" + impactResolution.normalWorld()
                    + " inwardPointSpeedBeforeMps=" + Double.toString(impactResolution.inwardPointSpeedBeforeMps())
                    + " normalImpulseNs=" + Double.toString(impactResolution.normalImpulseNewtonSeconds())
                    + " tangentialImpulseNs=" + impactResolution.tangentialImpulseNewtonSeconds()
                    + " frictionCoefficient=" + impactResolution.frictionCoefficient()
                    + " tangentialComSpeedBeforeMps=" + Double.toString(impactResolution.tangentialComSpeedBeforeMps())
                    + " tangentialComSpeedAfterMps=" + Double.toString(impactResolution.tangentialComSpeedAfterMps())
                    + " fullVelocityScaleApplied=false"
                    + " poseChecks=" + recovery.poseChecks()
                    + " blockPositions=" + recovery.blockPositions()
                    + " collisionShapes=" + recovery.collisionShapes()
                    + " impactCaptured=true"
                    + " contactAreaM2=" + Double.toString(impact.projectedAreaSquareMeters())
                    + " trueImpactPreResolved=" + (trueImpactResolution != null
                        && trueImpactResolution.accepted())
                    + " trueImpactMode=" + (trueImpactResolution == null
                        ? "NONE" : trueImpactResolution.mode())
                    + " trueImpactEnergyJ=" + Double.toString(trueImpactResolution == null
                        ? 0.0 : trueImpactResolution.impactEnergyJ())
                    + " trueImpactAbsorbedEnergyJ=" + Double.toString(trueImpactResolution == null
                        ? 0.0 : trueImpactResolution.absorbedEnergyJ())
                    + " trueImpactResidualEnergyJ=" + Double.toString(trueImpactResolution == null
                        ? 0.0 : trueImpactResolution.residualEnergyJ())
                    + " trueImpactBlocksBroken=" + (trueImpactResolution == null
                        ? 0 : trueImpactResolution.blocksBroken())
                    + " targetResidualInwardSpeedMps="
                        + Double.toString(targetResidualInwardSpeedMps)
                    + " maxMaterialTravelMeters=" + Double.toString(maxMaterialTravelMeters)
                    + " compoundMaterialPass=" + materialPasses
                    + " compoundMaterialBlocksOpened="
                        + (materialBlocksOpened + (trueImpactResolution != null
                            && trueImpactResolution.terrainOpened()
                            ? trueImpactResolution.blocksBroken() : 0))
                    + " policy=PREINTEGRATION_COMPOUND_ITERATIVE_TRUE_IMPACT_MATERIAL_RESPONSE_THEN_SABLE_IMPULSE"
                    + " restitution=0.0"
                    + " postStepRecoveryRole=FAILSAFE_ONLY"
                    + " packSpecific=false"
            );
            }
            PMIVObserver.captureTerrainCcd(
                body, safeFraction, preImpactVehicleSpeedMps, impact,
                impactResolution.normalWorld(),
                impactResolution.inwardPointSpeedBeforeMps(),
                impactResolution.normalImpulseNewtonSeconds(),
                impactResolution.tangentialImpulseNewtonSeconds(), impactResolution.frictionCoefficient(),
                impactResolution.tangentialComSpeedBeforeMps(),
                impactResolution.tangentialComSpeedAfterMps(),
                impact.projectedAreaSquareMeters(),
                trueImpactResolution != null && trueImpactResolution.accepted(),
                trueImpactResolution == null ? "NONE" : trueImpactResolution.mode(),
                trueImpactResolution == null ? 0.0 : trueImpactResolution.impactEnergyJ(),
                trueImpactResolution == null ? 0.0 : trueImpactResolution.absorbedEnergyJ(),
                trueImpactResolution == null ? 0.0 : trueImpactResolution.residualEnergyJ(),
                trueImpactResolution == null ? 0.0 : trueImpactResolution.residualInwardSpeedMps(),
                trueImpactResolution == null ? 0 : trueImpactResolution.blocksBroken()
            );

            boolean terrainOpened = trueImpactResolution != null
                && trueImpactResolution.accepted()
                && trueImpactResolution.terrainOpened()
                && trueImpactResolution.blocksBroken() > 0;
            if (!terrainOpened) {
                // A rigid hit changes the trajectory too. Check the resulting
                // compound motion instead of handing its secondary intersections
                // to native penetration correction. Bound geometric work per step.
                if (!momentumChangedThisPass || ++rigidPasses >= 8) break;
                continue;
            }

            long seedKey = new BlockPos(
                (int) impact.block().x, (int) impact.block().y, (int) impact.block().z
            ).asLong();
            if (!openedSeedBlocks.add(seedKey)) {
                if (PMIVObserver.loggingEnabled()) {
                    PMIVObserver.log(
                    "ERROR stage=preIntegrationTerrainMaterial reason=repeatedOpenedSeed"
                        + " uuid=" + body.vehicle.uniqueUUID
                        + " gameTime=" + body.level.getGameTime()
                        + " block=" + impact.block()
                        + " materialPasses=" + materialPasses
                );
                }
                break;
            }
            terrainQuery.invalidateAfterMaterialFracture();
            materialBlocksOpened += trueImpactResolution.blocksBroken();
            ++materialPasses;
            // World terrain changed synchronously. Re-run the complete compound
            // sweep before Rapier integrates so another aircraft box cannot hit a
            // neighboring/deeper voxel as an unbudgeted native rigid contact.
        }

        Vector3d desiredPreGravityLinear = new Vector3d(workingPostGravityLinear).sub(
            body.sableGravityWorld.x * timeStep,
            body.sableGravityWorld.y * timeStep,
            body.sableGravityWorld.z * timeStep
        );
        body.temporaryLinearCorrectionWorld.set(desiredPreGravityLinear)
            .sub(originalPreGravityLinear);
        body.temporaryAngularCorrectionWorld.set(workingAngular).sub(originalAngular);
        if (changedMomentum
            || body.temporaryLinearCorrectionWorld.lengthSquared() > 1.0E-18
            || body.temporaryAngularCorrectionWorld.lengthSquared() > 1.0E-18) {
            body.rigidBodyHandle.addLinearAndAngularVelocity(
                body.temporaryLinearCorrectionWorld, body.temporaryAngularCorrectionWorld
            );
        }
        body.rigidBodyHandle.getLinearVelocity(body.temporaryLinearVelocityWorld);
        body.rigidBodyHandle.getAngularVelocity(body.temporaryAngularVelocityWorld);
        SablePoseConversions.worldToLocal(
            body.cachedOrientation, body.temporaryAngularVelocityWorld, body.temporaryAngularVelocityBody
        );
    }

    /**
     * Resolves one swept BODY terrain contact without damping any tangent motion.
     * The contact normal comes from the actual Minecraft collision-shape boundary.
     * Linear and angular changes use the same rigid-body effective-mass equation as
     * the wheel solver: J = -v_n / (1/m + (r x n) dot I^-1(r x n)).
     */
    TerrainVelocityResolution resolveTerrainImpactVelocity(
        Vector3dc linearVelocityWorld,
        Vector3dc angularVelocityWorld,
        Vector3dc bodyPositionWorld,
        Quaterniond bodyOrientationWorld,
        SableCompoundCollider.TerrainBoundaryImpact impact
    ) {
        return resolveTerrainImpactVelocity(
            linearVelocityWorld, angularVelocityWorld, bodyPositionWorld,
            bodyOrientationWorld, impact, 0.0
        );
    }

    TerrainVelocityResolution resolveTerrainImpactVelocity(
        Vector3dc linearVelocityWorld,
        Vector3dc angularVelocityWorld,
        Vector3dc bodyPositionWorld,
        Quaterniond bodyOrientationWorld,
        SableCompoundCollider.TerrainBoundaryImpact impact,
        double targetResidualInwardSpeedMps
    ) {
        Vector3d safeLinear = linearVelocityWorld == null
            ? new Vector3d() : new Vector3d(linearVelocityWorld);
        Vector3d safeAngular = angularVelocityWorld == null
            ? new Vector3d() : new Vector3d(angularVelocityWorld);
        Vector3d zeroNormal = new Vector3d();
        if (impact == null || impact.worldPoint() == null || impact.worldNormal() == null
            || bodyPositionWorld == null || bodyOrientationWorld == null
            || !SableVehicleBody.finite(safeLinear) || !SableVehicleBody.finite(safeAngular)
            || !SableVehicleBody.finite(bodyPositionWorld) || !SableVehicleBody.finite(bodyOrientationWorld)
            || !SableVehicleBody.finite(impact.worldPoint()) || !SableVehicleBody.finite(impact.worldNormal())) {
            return new TerrainVelocityResolution(
                safeLinear, safeAngular, zeroNormal, 0.0, 0.0,
                safeLinear.length(), safeLinear.length()
            );
        }

        Vector3d normalWorld = new Vector3d(impact.worldNormal());
        if (normalWorld.lengthSquared() <= 1.0E-12) {
            return new TerrainVelocityResolution(
                safeLinear, safeAngular, zeroNormal, 0.0, 0.0,
                safeLinear.length(), safeLinear.length()
            );
        }
        normalWorld.normalize();
        Vector3d leverWorld = new Vector3d(impact.worldPoint()).sub(bodyPositionWorld);
        Vector3d pointVelocityWorld = safeAngular.cross(leverWorld, new Vector3d())
            .add(safeLinear);
        double normalPointSpeed = pointVelocityWorld.dot(normalWorld);
        double tangentBefore = tangentialSpeed(safeLinear, normalWorld);
        if (!Double.isFinite(normalPointSpeed) || normalPointSpeed >= -1.0E-9) {
            return new TerrainVelocityResolution(
                safeLinear, safeAngular, normalWorld,
                Math.max(0.0, -normalPointSpeed), 0.0,
                tangentBefore, tangentBefore
            );
        }

        double liveMass = Math.max(1.0, body.vehicle.currentMass);
        double liveMassRatio = liveMass / Math.max(1.0, body.mass);
        Vec3d liveInertia = body.desiredInertia.scale(liveMassRatio);
        double inwardPointSpeed = -normalPointSpeed;
        double residualInward = Double.isFinite(targetResidualInwardSpeedMps)
            ? Math.max(0.0, Math.min(inwardPointSpeed, targetResidualInwardSpeedMps))
            : 0.0;
        var projected = RigidContactSolver.solve(safeLinear, safeAngular, bodyPositionWorld,
            bodyOrientationWorld, liveInertia, liveMass, List.of(new RigidContactSolver.Contact(
                new Vector3d(impact.worldPoint()), normalWorld, residualInward)));
        double impulse = projected.normalImpulses()[0];
        Vector3d correctedLinear = projected.linearWorld();
        Vector3d correctedAngular = projected.angularWorld();
        double tangentAfter = tangentialSpeed(correctedLinear, normalWorld);
        return new TerrainVelocityResolution(
            correctedLinear, correctedAngular, normalWorld, -normalPointSpeed, impulse,
            tangentBefore, tangentAfter
        );
    }

    private TerrainVelocityResolution resolveRigidContactManifold(
        Vector3dc linear, Vector3dc angular, Vector3dc position, Quaterniond orientation,
        SableCompoundCollider.TerrainBoundaryImpact primary,
        List<SableCompoundCollider.TerrainBoundaryImpact> candidates,
        double preSpeed, double safeFraction
    ) {
        List<SableCompoundCollider.TerrainBoundaryImpact> contacts = new ArrayList<>();
        contacts.add(primary);
        for (var candidate : candidates) {
            if (candidate == null || candidate.worldPoint() == null || candidate.worldNormal() == null)
                continue;
            boolean duplicate = false;
            for (var existing : contacts) {
                if (existing.worldPoint().distanceSquared(candidate.worldPoint()) < 0.0001
                    && existing.worldNormal().dot(candidate.worldNormal()) > 0.999) {
                    duplicate = true; break;
                }
            }
            if (!duplicate) contacts.add(candidate);
        }
        double mass = Math.max(1.0, body.vehicle.currentMass);
        Vec3d inertia = body.desiredInertia.scale(mass / Math.max(1.0, body.mass));
        List<RigidContactSolver.Contact> constraints = new ArrayList<>(contacts.size());
        for (var contact : contacts) constraints.add(new RigidContactSolver.Contact(
            contact.worldPoint(), contact.worldNormal(), 0.0));
        var normal = RigidContactSolver.solve(linear, angular, position, orientation,
            inertia, mass, constraints);
        Vector3d v = new Vector3d(normal.linearWorld()), w = new Vector3d(normal.angularWorld());
        double primaryTangentImpulse = 0.0;
        double primaryCoefficient = 0.0;
        for (int i = 0; i < contacts.size(); ++i) {
            var contact = contacts.get(i);
            double impulse = normal.normalImpulses()[i];
            var response = applyTerrainContactFriction(new TerrainVelocityResolution(v, w,
                contact.worldNormal(), normal.inwardBefore()[i], impulse,
                tangentialSpeed(v, contact.worldNormal()), tangentialSpeed(v, contact.worldNormal())),
                position, orientation, contact);
            v = response.linearVelocityWorld(); w = response.angularVelocityWorld();
            if (i == 0) {
                primaryTangentImpulse = response.tangentialImpulseNewtonSeconds();
                primaryCoefficient = response.frictionCoefficient();
            }
        }
        // Tangent response at one station can close another station's normal.
        // Re-project the coupled normals without applying friction twice.
        var finalNormal = RigidContactSolver.solve(v, w, position, orientation,
            inertia, mass, constraints);
        for (int i = 1; i < contacts.size(); ++i) {
            var contact = contacts.get(i);
            double impulse = normal.normalImpulses()[i] + finalNormal.normalImpulses()[i];
            if (impulse <= 1.0E-8) continue;
            queueSweptTerrainImpact(contact, preSpeed, normal.inwardBefore()[i], impulse,
                body.vehicle.world.getBlockHardness(contact.block()), null);
            PMIVObserver.captureTerrainCcd(body, safeFraction, preSpeed, contact,
                contact.worldNormal(), normal.inwardBefore()[i], impulse, 0.0, 0.0,
                tangentialSpeed(linear, contact.worldNormal()),
                tangentialSpeed(finalNormal.linearWorld(), contact.worldNormal()),
                contact.projectedAreaSquareMeters(), false, "RIGID_COMPOUND_SUPPORT",
                0.0, 0.0, 0.0, 0.0, 0);
        }
        return new TerrainVelocityResolution(finalNormal.linearWorld(), finalNormal.angularWorld(),
            primary.worldNormal(), normal.inwardBefore()[0],
            normal.normalImpulses()[0] + finalNormal.normalImpulses()[0],
            tangentialSpeed(linear, primary.worldNormal()),
            tangentialSpeed(finalNormal.linearWorld(), primary.worldNormal()),
            primaryTangentImpulse, primaryCoefficient);
    }

    private TerrainVelocityResolution applyTerrainContactFriction(
        TerrainVelocityResolution normalResponse, Vector3dc bodyPosition,
        Quaterniond orientation, SableCompoundCollider.TerrainBoundaryImpact impact
    ) {
        if (impact == null || impact.block() == null || normalResponse.normalImpulseNewtonSeconds() <= 0.0)
            return normalResponse;
        BlockPos block = new BlockPos((int) impact.block().x, (int) impact.block().y, (int) impact.block().z);
        double surfaceCoefficient = dev.ryanhcode.sable.physics.config.block_properties.PhysicsBlockPropertyHelper
            .getFriction(body.level.getBlockState(block));
        double coefficient = SableCompoundCollider.bodyTerrainFriction(surfaceCoefficient);
        double mass = Math.max(1.0, body.vehicle.currentMass);
        Vec3d inertia = body.desiredInertia.scale(mass / Math.max(1.0, body.mass));
        var matrix = new minecrafttransportsimulator.baseclasses.RotationMatrix();
        SablePoseConversions.writeRotationMatrix(orientation, matrix);
        var result = com.g9third.pmweatheriv.physics.BodyContactFriction.solve(
            new Vec3d(normalResponse.linearVelocityWorld().x, normalResponse.linearVelocityWorld().y,
                normalResponse.linearVelocityWorld().z),
            new Vec3d(normalResponse.angularVelocityWorld().x, normalResponse.angularVelocityWorld().y,
                normalResponse.angularVelocityWorld().z), matrix,
            new Vec3d(impact.worldPoint().x-bodyPosition.x(), impact.worldPoint().y-bodyPosition.y(),
                impact.worldPoint().z-bodyPosition.z()),
            new Vec3d(normalResponse.normalWorld().x, normalResponse.normalWorld().y, normalResponse.normalWorld().z),
            inertia, mass, normalResponse.normalImpulseNewtonSeconds(), coefficient);
        Vector3d linear = new Vector3d(result.velocityWorld().x(), result.velocityWorld().y(), result.velocityWorld().z());
        Vector3d angular = new Vector3d(result.angularVelocityWorld().x(), result.angularVelocityWorld().y(),
            result.angularVelocityWorld().z());
        return new TerrainVelocityResolution(linear, angular, normalResponse.normalWorld(),
            normalResponse.inwardPointSpeedBeforeMps(), normalResponse.normalImpulseNewtonSeconds(),
            normalResponse.tangentialComSpeedBeforeMps(), tangentialSpeed(linear, normalResponse.normalWorld()),
            result.impulseWorld().length(), coefficient);
    }

    static double tangentialSpeed(Vector3dc velocityWorld, Vector3dc normalWorld) {
        if (velocityWorld == null || normalWorld == null) {
            return 0.0;
        }
        double normalSpeed = velocityWorld.dot(normalWorld);
        return Math.sqrt(Math.max(
            0.0, velocityWorld.lengthSquared() - normalSpeed * normalSpeed
        ));
    }

    void resolveSweptTerrainBoundaryAfterSubstep() {
        long start = PMIVObserver.isCapturing(body.vehicle.uniqueUUID) ? System.nanoTime() : 0;
        try {
            resolveSweptTerrainBoundaryAfterSubstepImpl();
        } finally {
            if (start != 0) PMIVObserver.capturePerformance(body.vehicle.uniqueUUID,
                "terrainPostIntegration", System.nanoTime() - start);
        }
    }

    private void resolveSweptTerrainBoundaryAfterSubstepImpl() {
        if (!hasSubstepStartPose || body.compoundCollider == null || !body.compoundCollider.isActive()) {
            return;
        }
        Vector3d completedPosition = new Vector3d(body.getPose().position());
        Quaterniond completedOrientation = new Quaterniond(body.getPose().orientation()).normalize();
        if (!SableVehicleBody.finite(completedPosition) || !SableVehicleBody.finite(completedOrientation)) {
            return;
        }

        // A previous implementation always swept from substepStart. Once one
        // missed collision had already left that pose embedded, the next substep
        // therefore treated an invalid pose as its clear endpoint and could walk a
        // wreck farther through the terrain. Prefer the persistent last-known-clear
        // transform. It is refreshed after every verified clear substep and never
        // overwritten by an unresolved penetration.
        boolean persistentAnchor = hasClearTerrainPose
            && SableVehicleBody.finite(lastClearTerrainPositionWorld)
            && SableVehicleBody.finite(lastClearTerrainOrientationWorld);
        Vector3d recoveryStartPosition = persistentAnchor
            ? new Vector3d(lastClearTerrainPositionWorld)
            : new Vector3d(substepStartPositionWorld);
        Quaterniond recoveryStartOrientation = persistentAnchor
            ? new Quaterniond(lastClearTerrainOrientationWorld).normalize()
            : new Quaterniond(substepStartOrientationWorld).normalize();

        SableCompoundCollider.TerrainPoseRecovery recovery =
            body.compoundCollider.recoverDeepTerrainPenetration(
                body.level,
                recoveryStartPosition,
                recoveryStartOrientation,
                completedPosition,
                completedOrientation
            );
        if (!recovery.terrainBoundaryViolated()) {
            rememberClearTerrainPose(completedPosition, completedOrientation);
            return;
        }

        Vector3d resolvedPosition = null;
        Quaterniond resolvedOrientation = null;
        double resolvedPathFraction = recovery.resolvedFraction();
        boolean fallbackToPersistentClearPose = false;
        boolean fallbackToSubstepStartPose = false;
        boolean verticalEscapeRecovery = false;

        if (recovery.recovered()) {
            resolvedPosition = new Vector3d(recovery.resolvedPosition());
            resolvedOrientation = new Quaterniond(recovery.resolvedOrientation()).normalize();
            verticalEscapeRecovery = !recovery.priorPoseClear();
        } else if (persistentAnchor && !body.compoundCollider.bodyIntersectsTerrainAtPose(
            body.level, recoveryStartPosition, recoveryStartOrientation
        )) {
            // Recovery can fail because a sweep budget is exhausted or because a
            // highly degenerate completed pose cannot be refined. A previously
            // verified clear pose is still a stronger invariant than accepting the
            // embedded Rapier result as the next substep origin.
            resolvedPosition = new Vector3d(recoveryStartPosition);
            resolvedOrientation = new Quaterniond(recoveryStartOrientation).normalize();
            resolvedPathFraction = 0.0;
            fallbackToPersistentClearPose = true;
        } else if (!body.compoundCollider.bodyIntersectsTerrainAtPose(
            body.level, substepStartPositionWorld, substepStartOrientationWorld
        )) {
            resolvedPosition = new Vector3d(substepStartPositionWorld);
            resolvedOrientation = new Quaterniond(substepStartOrientationWorld).normalize();
            resolvedPathFraction = 0.0;
            fallbackToSubstepStartPose = true;
        } else {
            // Defensive recovery for a body that entered this code without any
            // surviving clear endpoint (for example a legacy/persisted wreck).
            // Passing the completed pose as both endpoints invokes the compound's
            // bounded upward escape search. This path is not used during ordinary
            // flight or collision response.
            SableCompoundCollider.TerrainPoseRecovery escape =
                body.compoundCollider.recoverDeepTerrainPenetration(
                    body.level,
                    completedPosition,
                    completedOrientation,
                    completedPosition,
                    completedOrientation
                );
            if (escape.recovered()) {
                recovery = escape;
                resolvedPosition = new Vector3d(escape.resolvedPosition());
                resolvedOrientation = new Quaterniond(escape.resolvedOrientation()).normalize();
                resolvedPathFraction = escape.resolvedFraction();
                verticalEscapeRecovery = true;
            } else {
                if (PMIVObserver.loggingEnabled()) {
                    PMIVObserver.log(
                    "ERROR stage=sableTerrainInvariant uuid=" + body.vehicle.uniqueUUID
                        + " gameTime=" + body.level.getGameTime()
                        + " reason=noVerifiedClearPose"
                        + " completedPosition=" + completedPosition
                        + " recoveryTruncated=" + recovery.truncated()
                        + " hasPersistentAnchor=" + persistentAnchor
                );
                }
                return;
            }
        }

        // Refuse to commit a recovery result that still leaves the inset BODY
        // shell inside Minecraft terrain. If the refined result is unexpectedly
        // bad, fall back to the persistent verified-clear pose rather than allowing
        // the next Sable substep to begin embedded.
        if (body.compoundCollider.bodyIntersectsTerrainAtPose(
            body.level, resolvedPosition, resolvedOrientation
        )) {
            if (persistentAnchor && !body.compoundCollider.bodyIntersectsTerrainAtPose(
                body.level, recoveryStartPosition, recoveryStartOrientation
            )) {
                resolvedPosition.set(recoveryStartPosition);
                resolvedOrientation.set(recoveryStartOrientation).normalize();
                resolvedPathFraction = 0.0;
                fallbackToPersistentClearPose = true;
            } else if (!body.compoundCollider.bodyIntersectsTerrainAtPose(
                body.level, substepStartPositionWorld, substepStartOrientationWorld
            )) {
                resolvedPosition.set(substepStartPositionWorld);
                resolvedOrientation.set(substepStartOrientationWorld).normalize();
                resolvedPathFraction = 0.0;
                fallbackToSubstepStartPose = true;
            } else {
                if (PMIVObserver.loggingEnabled()) {
                    PMIVObserver.log(
                    "ERROR stage=sableTerrainInvariant uuid=" + body.vehicle.uniqueUUID
                        + " gameTime=" + body.level.getGameTime()
                        + " reason=recoveryPoseStillEmbedded"
                        + " completedPosition=" + completedPosition
                        + " candidatePosition=" + resolvedPosition
                );
                }
                return;
            }
        }

        double preCorrectionVehicleSpeedMps = body.temporaryLinearVelocityWorld.length();
        SableCompoundCollider.TerrainBoundaryImpact sweptImpact = recovery.boundaryImpact();
        if (sweptImpact == null) {
            sweptImpact = body.compoundCollider.findFirstTerrainImpactAlongPath(
                body.level,
                recoveryStartPosition,
                recoveryStartOrientation,
                completedPosition,
                completedOrientation
            );
        }
        Vector3d correctedLinearVelocity = new Vector3d(body.temporaryLinearVelocityWorld);
        Vector3d correctedAngularVelocityWorld = new Vector3d(body.temporaryAngularVelocityWorld);
        double removedLinearMps = 0.0;
        double removedAngularRadps = 0.0;
        double sweptNormalImpulseNs = 0.0;
        double sweptInwardPointSpeedBeforeMps = 0.0;
        Vector3d sweptNormalWorld = new Vector3d();

        // Use the same point-normal impulse as the pre-step CCD whenever the
        // concrete swept BODY/block hit is known. This preserves runway/wall
        // tangential momentum and couples the impact to rotation through r x J.
        Resolution postStepTrueImpactResolution = null;
        double postStepTargetResidualInwardSpeedMps = 0.0;
        double postStepMaterialTravelMeters = 0.0;
        boolean completedPoseClearAfterMaterial = false;
        boolean postMaterialRecoveryAdvanced = false;
        float sweptPreImpactHardness = sweptImpact != null && sweptImpact.block() != null
            ? body.vehicle.world.getBlockHardness(sweptImpact.block()) : -1.0F;

        if (sweptImpact != null && sweptImpact.worldNormal() != null) {
            TerrainVelocityResolution rigidImpactResolution = resolveTerrainImpactVelocity(
                correctedLinearVelocity,
                correctedAngularVelocityWorld,
                resolvedPosition,
                resolvedOrientation,
                sweptImpact
            );

            // The post-step terrain-boundary pass is a fail-safe for a contact Rapier
            // integrated before PMIV's pre-step CCD could commit it. 0.11.7 applied the
            // full rigid stop here even when the preceding material pass had already
            // shown that the terrain should yield. That produced a second, unbudgeted
            // impulse (30-65 kN*s in the September 16 crash traces). Run the exact same
            // integrated material authority here before applying any recovery impulse.
            if (rigidImpactResolution.inwardPointSpeedBeforeMps() > 0.0
                && rigidImpactResolution.normalImpulseNewtonSeconds() > 0.0
                && body.terrainDamageAllowedForCurrentCrashEpisode()
                && PMWeatherIVConfig.get().enableBlockBreaking()
                && ConfigSystem.settings.damage.vehicleBlockBreaking.value) {
                double linearTravelMeters =
                    new Vector3d(completedPosition).sub(substepStartPositionWorld).length();
                double orientationDot = Math.abs(
                    new Quaterniond(substepStartOrientationWorld).normalize()
                        .dot(new Quaterniond(completedOrientation).normalize())
                );
                orientationDot = Math.max(0.0, Math.min(1.0, orientationDot));
                double rotationAngleRad = 2.0 * Math.acos(orientationDot);
                double rotationalPointTravelMeters = rotationAngleRad
                    * Math.max(0.0, body.compoundCollider.collisionBoundingRadius());
                postStepMaterialTravelMeters = Math.max(
                    0.0, linearTravelMeters + rotationalPointTravelMeters
                );

                postStepTrueImpactResolution = AircraftTerrainImpact.resolveExternalPenetration(
                    body.level,
                    new BlockPos((int) sweptImpact.block().x, (int) sweptImpact.block().y, (int) sweptImpact.block().z),
                    sweptImpact.worldPoint(),
                    sweptImpact.worldNormal(),
                    sweptImpact.projectedAreaSquareMeters(),
                    rigidImpactResolution.normalImpulseNewtonSeconds(),
                    rigidImpactResolution.inwardPointSpeedBeforeMps(),
                    postStepMaterialTravelMeters
                );
                if (postStepTrueImpactResolution != null
                    && postStepTrueImpactResolution.accepted()) {
                    postStepTargetResidualInwardSpeedMps = Math.max(
                        0.0, postStepTrueImpactResolution.residualInwardSpeedMps()
                    );
                }
            }

            TerrainVelocityResolution impactResolution =
                postStepTrueImpactResolution != null && postStepTrueImpactResolution.accepted()
                    ? resolveTerrainImpactVelocity(
                        correctedLinearVelocity,
                        correctedAngularVelocityWorld,
                        resolvedPosition,
                        resolvedOrientation,
                        sweptImpact,
                        postStepTargetResidualInwardSpeedMps
                    )
                    : rigidImpactResolution;
            impactResolution = applyTerrainContactFriction(impactResolution,
                resolvedPosition, resolvedOrientation, sweptImpact);

            if (postStepTrueImpactResolution == null || !postStepTrueImpactResolution.accepted()) {
                var query = body.compoundCollider.newTerrainQuerySession(body.level);
                impactResolution = resolveRigidContactManifold(correctedLinearVelocity,
                    correctedAngularVelocityWorld, resolvedPosition, resolvedOrientation,
                    sweptImpact, query.rigidContactsAtPose(resolvedPosition, resolvedOrientation),
                    preCorrectionVehicleSpeedMps, resolvedPathFraction);
            }


            removedLinearMps = Math.max(
                0.0, correctedLinearVelocity.length()
                    - impactResolution.linearVelocityWorld().length()
            );
            removedAngularRadps = Math.max(
                0.0, correctedAngularVelocityWorld.length()
                    - impactResolution.angularVelocityWorld().length()
            );
            correctedLinearVelocity.set(impactResolution.linearVelocityWorld());
            correctedAngularVelocityWorld.set(impactResolution.angularVelocityWorld());
            sweptNormalImpulseNs = impactResolution.normalImpulseNewtonSeconds();
            sweptInwardPointSpeedBeforeMps = impactResolution.inwardPointSpeedBeforeMps();
            sweptNormalWorld.set(impactResolution.normalWorld());

            // Immediate material failure may have made the already-integrated endpoint
            // physically valid. In that case do not rewind to the old solid-terrain
            // boundary; retain Rapier's completed pose and only apply the material-
            // budgeted momentum correction above. If other terrain still intersects the
            // body, retain the verified-clear recovery pose and let the next substep
            // continue penetration from there.
            if (postStepTrueImpactResolution != null
                && postStepTrueImpactResolution.accepted()
                && postStepTrueImpactResolution.terrainOpened()) {
                if (!body.compoundCollider.bodyIntersectsTerrainAtPose(
                    body.level, completedPosition, completedOrientation
                )) {
                    resolvedPosition.set(completedPosition);
                    resolvedOrientation.set(completedOrientation).normalize();
                    resolvedPathFraction = 1.0;
                    fallbackToPersistentClearPose = false;
                    fallbackToSubstepStartPose = false;
                    verticalEscapeRecovery = false;
                    completedPoseClearAfterMaterial = true;
                } else {
                    // Terrain changed under the recovery. Re-sweep against the new
                    // material state so position recovery advances to the next actual
                    // surviving boundary rather than the boundary of a block that has
                    // just fractured.
                    SableCompoundCollider.TerrainPoseRecovery postMaterialRecovery =
                        body.compoundCollider.recoverDeepTerrainPenetration(
                            body.level,
                            recoveryStartPosition,
                            recoveryStartOrientation,
                            completedPosition,
                            completedOrientation
                        );
                    if (postMaterialRecovery.recovered()
                        && !body.compoundCollider.bodyIntersectsTerrainAtPose(
                            body.level,
                            postMaterialRecovery.resolvedPosition(),
                            postMaterialRecovery.resolvedOrientation()
                        )) {
                        double previousFraction = resolvedPathFraction;
                        resolvedPosition.set(postMaterialRecovery.resolvedPosition());
                        resolvedOrientation.set(
                            postMaterialRecovery.resolvedOrientation()
                        ).normalize();
                        resolvedPathFraction = postMaterialRecovery.resolvedFraction();
                        postMaterialRecoveryAdvanced =
                            resolvedPathFraction > previousFraction + 1.0E-8;
                        fallbackToPersistentClearPose = false;
                        fallbackToSubstepStartPose = false;
                        verticalEscapeRecovery = false;
                    }
                }
            }
        }
        // Without a concrete contact normal, restore the verified clear pose only.
        // The recovery path is not a surface normal and cannot justify a physical
        // impulse or independent angular damping. Preserve Rapier momentum until
        // the next substep identifies an actual terrain contact.

        queueSweptTerrainImpact(
            sweptImpact, preCorrectionVehicleSpeedMps, sweptInwardPointSpeedBeforeMps,
            sweptNormalImpulseNs, sweptPreImpactHardness, postStepTrueImpactResolution
        );

        body.placeBodyWithVelocity(
            correctedLinearVelocity,
            correctedAngularVelocityWorld,
            resolvedOrientation,
            resolvedPosition
        );
        body.temporaryLinearVelocityWorld.set(correctedLinearVelocity);
        body.temporaryAngularVelocityWorld.set(correctedAngularVelocityWorld);
        rememberClearTerrainPose(resolvedPosition, resolvedOrientation);
        if (PMIVObserver.loggingEnabled()) {
            PMIVObserver.log(
            "SABLE_SUBSTEP_TERRAIN_BOUNDARY_RESOLVED gameTime=" + body.level.getGameTime()
                + " uuid=" + body.vehicle.uniqueUUID
                + " embeddedPosition=" + completedPosition
                + " recoveryAnchorPosition=" + recoveryStartPosition
                + " persistentClearAnchor=" + persistentAnchor
                + " resolvedPosition=" + resolvedPosition
                + " translationMeters=" + completedPosition.distance(resolvedPosition)
                + " orientationDot=" + Math.abs(
                    completedOrientation.dot(resolvedOrientation)
                )
                + " resolvedPathFraction=" + resolvedPathFraction
                + " poseChecks=" + recovery.poseChecks()
                + " blockPositions=" + recovery.blockPositions()
                + " collisionShapes=" + recovery.collisionShapes()
                + " removedInwardLinearMps=" + removedLinearMps
                + " removedInwardAngularRadps=" + removedAngularRadps
                + " sweptImpactNormal=" + sweptNormalWorld
                + " sweptNormalImpulseNs=" + sweptNormalImpulseNs
                + " contactAreaM2=" + (
                    sweptImpact == null ? 0.0 : sweptImpact.projectedAreaSquareMeters()
                )
                + " trueImpactPreResolved=" + (
                    postStepTrueImpactResolution != null
                        && postStepTrueImpactResolution.accepted()
                )
                + " trueImpactMode=" + (
                    postStepTrueImpactResolution == null
                        ? "NONE" : postStepTrueImpactResolution.mode()
                )
                + " trueImpactEnergyJ=" + (
                    postStepTrueImpactResolution == null
                        ? 0.0 : postStepTrueImpactResolution.impactEnergyJ()
                )
                + " trueImpactAbsorbedEnergyJ=" + (
                    postStepTrueImpactResolution == null
                        ? 0.0 : postStepTrueImpactResolution.absorbedEnergyJ()
                )
                + " trueImpactResidualEnergyJ=" + (
                    postStepTrueImpactResolution == null
                        ? 0.0 : postStepTrueImpactResolution.residualEnergyJ()
                )
                + " trueImpactBlocksBroken=" + (
                    postStepTrueImpactResolution == null
                        ? 0 : postStepTrueImpactResolution.blocksBroken()
                )
                + " targetResidualInwardSpeedMps=" + postStepTargetResidualInwardSpeedMps
                + " maxMaterialTravelMeters=" + postStepMaterialTravelMeters
                + " completedPoseClearAfterMaterial=" + completedPoseClearAfterMaterial
                + " postMaterialRecoveryAdvanced=" + postMaterialRecoveryAdvanced
                + " fullVelocityScaleApplied=false"
                + " sweptGameplayImpactCaptured=" + (sweptImpact != null)
                + " sweptPreImpactVehicleSpeedMps=" + preCorrectionVehicleSpeedMps
                + " fallbackToPersistentClearPose=" + fallbackToPersistentClearPose
                + " fallbackToSubstepStartPose=" + fallbackToSubstepStartPose
                + " verticalEscapeRecovery=" + verticalEscapeRecovery
                + " tangentialMomentumPolicy=CONTACT_FRICTION_BOUNDED_BY_NORMAL_IMPULSE"
                + " restitution=0.0"
                + " terrainRecovery=SABLE_PERSISTENT_CLEAR_POSE_BOUNDARY"
                + " nextSubstepStartsTerrainClear=true"
                + " packSpecific=false"
        );
        }
    }

    record TerrainVelocityResolution(
        Vector3d linearVelocityWorld,
        Vector3d angularVelocityWorld,
        Vector3d normalWorld,
        double inwardPointSpeedBeforeMps,
        double normalImpulseNewtonSeconds,
        double tangentialComSpeedBeforeMps,
        double tangentialComSpeedAfterMps,
        double tangentialImpulseNewtonSeconds,
        double frictionCoefficient
    ) {
        TerrainVelocityResolution(Vector3d linear, Vector3d angular, Vector3d normal,
            double inward, double impulse, double tangentBefore, double tangentAfter) {
            this(linear, angular, normal, inward, impulse, tangentBefore, tangentAfter, 0.0, 0.0);
        }
    }
}
