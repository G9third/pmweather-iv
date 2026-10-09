package com.g9third.pmweatheriv.sable;

import com.g9third.pmweatheriv.PMWeatherIVConfig;
import com.g9third.pmweatheriv.mixin.WrapperWorldAccessor;

import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import com.g9third.pmweatheriv.terrain.AircraftTerrainImpact;
import com.g9third.pmweatheriv.terrain.trueimpact.ExternalWorldImpactModel.Resolution;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import minecrafttransportsimulator.baseclasses.BoundingBox;
import minecrafttransportsimulator.baseclasses.Damage;
import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.entities.instances.APart;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.systems.ConfigSystem;
import net.minecraft.core.BlockPos;
import org.joml.Quaterniond;
import org.joml.Vector3d;

/**
 * IV 24.0.0 crash/damage gameplay translated from real Sable/Rapier terrain contacts.
 *
 * <p>0.8.3df has one terrain-contact authority. A crash exists only when the
 * Sable compound (or its Sable swept-CCD boundary) actually touches Minecraft
 * terrain. IV-authored BLOCK boxes are never swept or queried to decide whether
 * or where that contact happened. A box retained on a Sable collider is only an
 * attribution handle so the already-proven physical contact can be routed into
 * IV's native collision-group/part/destruction APIs.</p>
 *
 * <p>Ordinary Rapier contacts arrive after the physical impulse; destructive swept
 * contacts may first ask True Impact whether the terrain yields, then Sable applies
 * the residual material-aware impulse at the real contact point. The resulting r x J
 * angular response is left intact. This gameplay bridge does not
 * project, replace, damp, or re-apply that collision at the COM. In particular,
 * stock IV's post-block-hit linear motion scaling is intentionally not injected
 * into the Sable body. True Impact owns terrain fracture/damage while IV retains
 * authored aircraft crash-health/part gameplay and Sable retains momentum.</p>
 */
public final class SableCollisionGameplay {
    private static final double BLOCK_SAMPLE_OFFSET = 0.10;
    private static final double SECOND_BLOCK_SAMPLE_OFFSET = 0.30;
    /** Ignore suspension settling and ordinary support contacts; damage requires a hard gear strike. */
    private static final double MIN_LANDING_GEAR_DAMAGE_SPEED_MPS = 2.5;
    private static final int MAX_CONTACT_DIAGNOSTICS = 24;

    private SableCollisionGameplay() {
    }

    /**
     * Routes a severe landing-gear point impact into IV's existing part damage system.
     * The trigger uses the same authored crash-speed scale as IV, but the severity input
     * is the wheel station's real inward terrain speed because COM speed cannot represent
     * a one-wheel slam during a rotating touchdown. Normal runway support remains purely
     * physical and never reaches this path.
     *
     * @return true while the IV ground-device part remains valid/active after gameplay.
     */
    public static boolean processLandingGearImpact(
        SableVehicleBody body, minecrafttransportsimulator.entities.instances.PartGroundDevice device,
        double inwardNormalSpeedMps
    ) {
        if (body == null || device == null || !device.isValid
            || !Double.isFinite(inwardNormalSpeedMps)
            || inwardNormalSpeedMps <= MIN_LANDING_GEAR_DAMAGE_SPEED_MPS) {
            return device != null && device.isValid;
        }
        EntityVehicleF_Physics vehicle = body.vehicle();
        if (vehicle == null || !vehicle.isValid || vehicle.world.isClient()
            || !ConfigSystem.settings.damage.vehicleDestruction.value) {
            return device.isValid;
        }
        double speedFactor = nativeIvSpeedFactor(vehicle);
        double localIvCrashSpeed = inwardNormalSpeedMps / speedFactor;
        double crashMin = Math.max(vehicle.definition.motorized.crashSpeedMin,
            MIN_LANDING_GEAR_DAMAGE_SPEED_MPS / speedFactor);
        double crashMax = vehicle.definition.motorized.crashSpeedMax;
        if (!(crashMax > crashMin) || localIvCrashSpeed <= crashMin
            || !body.landingGearGameplayReady(device)) {
            return device.isValid;
        }
        double partHealth = device.definition != null && device.definition.general != null
            ? device.definition.general.health : 0.0;
        if (!(partHealth > 0.0)) {
            partHealth = Math.max(1.0, vehicle.definition.general.health);
        }
        double damage = partHealth * (localIvCrashSpeed - crashMin) / (crashMax - crashMin);
        damage = Math.max(0.0, Math.min(partHealth, damage));
        if (localIvCrashSpeed > vehicle.definition.motorized.crashSpeedDestroyed) {
            damage = partHealth;
        }
        if (damage > 0.0) {
            device.attack(new Damage(damage, null, null, null, null));
            body.markLandingGearGameplay(device);
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "SABLE_LANDING_GEAR_DAMAGE gameTime=" + simulationGameTime(vehicle)
                    + " uuid=" + vehicle.uniqueUUID
                    + " inwardNormalSpeedMps=" + inwardNormalSpeedMps
                    + " minimumDamageSpeedMps=" + MIN_LANDING_GEAR_DAMAGE_SPEED_MPS
                    + " localIvCrashSpeed=" + localIvCrashSpeed
                    + " crashMin=" + crashMin + " crashMax=" + crashMax
                    + " damage=" + damage
                    + " partValidAfter=" + device.isValid
                    + " policy=IV_PART_HEALTH_FROM_PHYSICAL_GEAR_POINT_IMPACT"
                    + " packSpecific=false"
            );
            }
        }
        return device.isValid && device.isActiveVar.isActive;
    }

    /**
     * Consumes one completed Sable tick (plus any Sable swept-CCD impacts) and
     * applies only IV gameplay consequences. Physical collision has already been
     * solved before this method runs.
     */
    public static void process(
        SableVehicleBody body,
        SableCollisionCapture.SceneSnapshot snapshot,
        Vector3d parentWorldPosition,
        Quaterniond parentWorldOrientation,
        List<SableVehicleBody.SweptTerrainImpact> sweptTerrainImpacts,
        boolean consumeReportedCollisionSnapshot
    ) {
        if (body == null || snapshot == null) {
            return;
        }
        SableCompoundCollider compound = body.compoundColliderForGameplay();
        EntityVehicleF_Physics vehicle = body.vehicle();
        if (compound == null || vehicle == null || !vehicle.isValid || vehicle.world.isClient()) {
            return;
        }

        List<SableCompoundCollider.GameplayContact> reported =
            consumeReportedCollisionSnapshot && !snapshot.contacts().isEmpty()
                ? compound.gameplayContacts(snapshot, parentWorldPosition, parentWorldOrientation)
                : List.of();
        List<SableVehicleBody.SweptTerrainImpact> swept = sweptTerrainImpacts == null
            ? List.of() : sweptTerrainImpacts;
        if (reported.isEmpty() && swept.isEmpty()) {
            return;
        }

        double mass = Math.max(1.0, vehicle.currentMass);
        double speedFactor = nativeIvSpeedFactor(vehicle);
        Map<BlockKey, ImpactEvidence> impactsByBlock = new LinkedHashMap<>();

        int terrainContacts = 0;
        double maxForce = 0.0;
        double maxImpulse = 0.0;
        double maxImpulseDeltaV = 0.0;
        double maxClosing = 0.0;
        double maxVehicleSpeed = 0.0;
        BoundingBox strongestSource = null;
        double strongestSourceImpulseNs = -1.0;

        for (SableCompoundCollider.GameplayContact contact : reported) {
            if (contact == null || !contact.terrainContact() || contact.wheelContact()) {
                continue;
            }
            ++terrainContacts;
            double preImpactSpeed = preImpactVehicleSpeed(body, snapshot, contact);
            double closingSpeed = preImpactNormalSpeed(body, snapshot, contact);
            maxImpulse = Math.max(maxImpulse, contact.impulseNewtonSeconds());
            maxImpulseDeltaV = Math.max(maxImpulseDeltaV, contact.impulseNewtonSeconds() / mass);
            maxClosing = Math.max(maxClosing, closingSpeed);
            maxVehicleSpeed = Math.max(maxVehicleSpeed, preImpactSpeed);
            if (contact.forceNewtons() > maxForce) {
                maxForce = contact.forceNewtons();
            }
            if (contact.sourceBox() != null
                && contact.impulseNewtonSeconds() > strongestSourceImpulseNs) {
                strongestSourceImpulseNs = contact.impulseNewtonSeconds();
                strongestSource = contact.sourceBox();
            }

            Point3D block = findTerrainBlock(vehicle, contact.worldPoint(), contact.worldNormal());
            if (block != null) {
                mergeImpact(impactsByBlock, new ImpactEvidence(
                    block.copy(), contact.sourceBox(),
                    copy(contact.worldPoint()), copy(contact.worldNormal()),
                    preImpactSpeed, closingSpeed,
                    Math.max(0.0, contact.impulseNewtonSeconds()),
                    Math.max(0.0, contact.forceNewtons()),
                    Float.NaN, null,
                    "RAPIER_CONTACT"
                ));
            }
        }

        for (SableVehicleBody.SweptTerrainImpact impact : swept) {
            if (impact == null || impact.block() == null) {
                continue;
            }
            double preImpactSpeed = Math.max(0.0, impact.preImpactVehicleSpeedMetersPerSecond());
            double closingSpeed = Math.max(0.0, impact.inwardPointSpeedMetersPerSecond());
            double impulse = Math.max(0.0, impact.normalImpulseNewtonSeconds());
            maxVehicleSpeed = Math.max(maxVehicleSpeed, preImpactSpeed);
            maxClosing = Math.max(maxClosing, closingSpeed);
            maxImpulse = Math.max(maxImpulse, impulse);
            maxImpulseDeltaV = Math.max(maxImpulseDeltaV, impulse / mass);
            if (impact.sourceBox() != null && impulse > strongestSourceImpulseNs) {
                strongestSourceImpulseNs = impulse;
                strongestSource = impact.sourceBox();
            }
            mergeImpact(impactsByBlock, new ImpactEvidence(
                impact.block().copy(), impact.sourceBox(),
                copy(impact.worldPoint()), copy(impact.worldNormal()),
                preImpactSpeed, closingSpeed, impulse, 0.0,
                impact.preImpactBlockHardness(), impact.trueImpactResolution(),
                "SABLE_SWEPT_CCD"
            ));
        }

        boolean blockBreakDelayReady = body.blockBreakGameplayReady();
        boolean outOfHealthBeforeGameplay = vehicle.outOfHealth;
        double totalLegacyCrashHardness = 0.0;
        int trueImpactImpactsSubmitted = 0;
        double maxTrueImpactEnergyJ = 0.0;
        double maxTrueImpactAbsorbedEnergyJ = 0.0;
        int trueImpactImmediateBlocksBroken = 0;
        int blocksBroken = 0;
        int collisionBoxesDamaged = 0;
        int partsRemoved = 0;
        double crashDamage = 0.0;
        boolean destroyed = false;
        boolean catastrophicWrecked = false;
        boolean anyHitBlock = false;
        boolean anySolidBlock = false;
        double maxNativeIvVelocity = 0.0;
        double maxNativeIvCrashSpeed = 0.0;
        List<String> contactDiagnostics = PMIVObserver.loggingEnabled() ? new ArrayList<>() : List.of();

        // A block is processed once per completed Sable tick. Multiple Rapier
        // manifold rows for the same voxel are collapsed to the strongest
        // pre-impact evidence so one physical block cannot multiply gameplay
        // damage merely because the model hull is finely voxelized.
        for (ImpactEvidence evidence : impactsByBlock.values()) {
            if (!vehicle.isValid) {
                break;
            }
            Point3D block = evidence.block();
            if (block == null || vehicle.world.isBlockLiquid(block)) {
                continue;
            }

            double physicalPreImpactSpeedMps = Math.max(0.0, evidence.preImpactVehicleSpeedMps());
            double physicalNormalImpactSpeedMps = Math.max(0.0, evidence.inwardPointSpeedMps());
            // IV aircraft crash severity remains authored in IV speed units, but its
            // input is the velocity actually driven into the contacted surface. The
            // terrain-damage path below no longer uses nativeIvVelocity at all.
            double nativeIvVelocity = physicalPreImpactSpeedMps / (speedFactor * 20.0);
            double nativeIvCrashSpeed = physicalNormalImpactSpeedMps / speedFactor;
            maxNativeIvVelocity = Math.max(maxNativeIvVelocity, nativeIvVelocity);
            maxNativeIvCrashSpeed = Math.max(maxNativeIvCrashSpeed, nativeIvCrashSpeed);

            BoundingBox sourceBox = attributionBox(vehicle, evidence.sourceBox());
            float hardness = Float.isFinite(evidence.preImpactBlockHardness())
                ? evidence.preImpactBlockHardness()
                : vehicle.world.getBlockHardness(block);
            // A swept material response may already have removed the contacted voxel
            // before this gameplay pass. Preserve the pre-impact hardness captured by
            // Sable so IV health/crash classification still describes the real hit.
            boolean hitBlock = hardness >= 0.0F;
            anyHitBlock |= hitBlock;
            anySolidBlock |= hitBlock;

            // A fatal crash must be allowed to finish the *same* physical impact
            // episode after IV health reaches zero.  The gate is therefore owned by
            // SableVehicleBody's momentum-based crash episode rather than the raw
            // outOfHealth bit.  Once the wreck has remained settled below the
            // structural-point-speed threshold for the hysteresis window, the gate
            // latches closed permanently for that wreck.
            boolean trueImpactSubmissionAllowed = body.terrainDamageAllowedForCurrentCrashEpisode();
            Resolution preResolved =
                evidence.trueImpactResolution();

            // True Impact owns the canonical contact-energy definition. For a swept
            // penetration it already derived this value before Sable's material-aware
            // response; ordinary Rapier contacts ask the same True Impact API here.
            double trueImpactEnergyJ = preResolved != null && preResolved.accepted()
                ? preResolved.impactEnergyJ()
                : AircraftTerrainImpact.deriveImpactEnergyJ(
                    Math.max(0.0, evidence.impulseNewtonSeconds()),
                    physicalNormalImpactSpeedMps
                );

            // A live aircraft starts a crash episode when the contact exceeds its own
            // authored IV crash-speed threshold. The episode begins before any fatal
            // IV health mutation exactly as in 0.11.5.
            double episodeCrashMin = Math.max(0.0, vehicle.definition.motorized.crashSpeedMin);
            if (!vehicle.outOfHealth && hitBlock && trueImpactEnergyJ > 0.0
                && nativeIvCrashSpeed > episodeCrashMin) {
                body.noteCrashEpisodeImpact(
                    trueImpactEnergyJ, Math.max(0.0, evidence.impulseNewtonSeconds())
                );
                trueImpactSubmissionAllowed = body.terrainDamageAllowedForCurrentCrashEpisode();
            }

            boolean trueImpactSubmitted = false;
            if (hitBlock && trueImpactSubmissionAllowed
                && PMWeatherIVConfig.get().enableBlockBreaking()
                && ConfigSystem.settings.damage.vehicleBlockBreaking.value
                && Double.isFinite(trueImpactEnergyJ) && trueImpactEnergyJ > 0.0) {
                if (preResolved != null && preResolved.accepted()) {
                    // The swept path has already been accepted/resolved by True Impact.
                    // Do not enqueue the same physical event again.
                    trueImpactSubmitted = true;
                    maxTrueImpactAbsorbedEnergyJ = Math.max(
                        maxTrueImpactAbsorbedEnergyJ, preResolved.absorbedEnergyJ());
                    trueImpactImmediateBlocksBroken += Math.max(0, preResolved.blocksBroken());
                } else {
                    BlockPos blockPos = new BlockPos(
                        (int) block.x, (int) block.y, (int) block.z
                    );
                    double acceptedEnergy = AircraftTerrainImpact.submitExternalWorldImpact(
                        body.level(), blockPos, evidence.worldNormal(),
                        Math.max(0.0, evidence.impulseNewtonSeconds()),
                        physicalNormalImpactSpeedMps
                    );
                    trueImpactSubmitted = acceptedEnergy > 0.0;
                    if (trueImpactSubmitted) {
                        trueImpactEnergyJ = acceptedEnergy;
                    }
                }
                if (trueImpactSubmitted) {
                    body.noteCrashEpisodeImpact(
                        trueImpactEnergyJ, Math.max(0.0, evidence.impulseNewtonSeconds())
                    );
                    ++trueImpactImpactsSubmitted;
                    maxTrueImpactEnergyJ = Math.max(maxTrueImpactEnergyJ, trueImpactEnergyJ);
                    // IV's legacy no-crashMax vehicle-destruction fallback still needs
                    // an authored terrain-hardness accumulator. Terrain fracture itself
                    // is entirely owned by True Impact.
                    totalLegacyCrashHardness += Math.max(0.0F, hardness);
                }
            }

            boolean destroyThisImpact = false;
            if (ConfigSystem.settings.damage.vehicleDestruction.value && vehicle.isValid
                && !vehicle.outOfHealth) {
                double crashMax = vehicle.definition.motorized.crashSpeedMax;
                if (crashMax > 0.0) {
                    double crashMin = vehicle.definition.motorized.crashSpeedMin;
                    if (hardness >= 0.0F && body.crashGameplayReady() && nativeIvCrashSpeed > crashMin) {
                        // Retain IV 24.0.0's authored crash-speed thresholds and
                        // linear damage curve, but feed them the Sable contact-point
                        // normal impact speed rather than total COM speed.  This keeps
                        // head-on/vertical crashes severe while a glancing tailstrike
                        // is graded by the velocity actually entering the terrain.
                        double damage = vehicle.definition.general.health
                            * (nativeIvCrashSpeed - crashMin) / (crashMax - crashMin);
                        if (damage >= vehicle.definition.general.health) {
                            if (nativeIvCrashSpeed > vehicle.definition.motorized.crashSpeedDestroyed) {
                                destroyThisImpact = true;
                                damage = 0.0;
                            } else {
                                damage = vehicle.definition.general.health;
                            }
                        }
                        if (damage > 0.0 && vehicle.isValid) {
                            double localized = damageAuthoredCollisionGroup(
                                vehicle, sourceBox, damage
                            );
                            if (localized > 0.0) {
                                ++collisionBoxesDamaged;
                            }
                            vehicle.attack(new Damage(damage, null, null, null, null));
                            crashDamage += damage;
                        }
                        body.markCrashGameplay();
                    }
                } else if (totalLegacyCrashHardness
                    > mass / (0.75 + nativeIvVelocity) / 250.0) {
                    // Retain IV 24.0.0's legacy vehicle-destruction fallback for
                    // packs without crashSpeedMax. Terrain fracture itself is not
                    // decided here; only True-Impact-accepted contact hardness feeds it.
                    destroyThisImpact = true;
                }
            }

            if (destroyThisImpact && vehicle.isValid) {
                APart partHit = sourceBox == null ? null : vehicle.getPartWithBox(sourceBox);
                if (partHit != null) {
                    totalLegacyCrashHardness = Math.max(0.0,
                        totalLegacyCrashHardness - Math.max(0.0F, hardness));
                    partHit.remove();
                    ++partsRemoved;
                } else if (!vehicle.outOfHealth) {
                    // IV's native terrain-crash path removes the entire vehicle at
                    // crashSpeedDestroyed. That would terminate the Sable body in
                    // the middle of the same physical crash. Saturate health instead
                    // so this becomes a persistent physical wreck; the crash-episode
                    // gate later closes terrain destruction after the wreck settles.
                    double fullHealthDamage = Math.max(1.0, vehicle.definition.general.health);
                    double localized = damageAuthoredCollisionGroup(
                        vehicle, sourceBox, fullHealthDamage
                    );
                    if (localized > 0.0) {
                        ++collisionBoxesDamaged;
                    }
                    vehicle.attack(new Damage(fullHealthDamage, null, null, null, null));
                    crashDamage += fullHealthDamage;
                    catastrophicWrecked = true;
                }
            }

            if (PMIVObserver.loggingEnabled() && contactDiagnostics.size() < MAX_CONTACT_DIAGNOSTICS) {
                contactDiagnostics.add(contactDiagnostic(
                    vehicle, evidence, sourceBox, hardness, hitBlock, trueImpactSubmitted,
                    trueImpactEnergyJ, physicalPreImpactSpeedMps, nativeIvVelocity, nativeIvCrashSpeed
                ));
            }
        }

        boolean outOfHealthAfterGameplay = vehicle.outOfHealth;
        boolean materialEvent = trueImpactImpactsSubmitted > 0
            || crashDamage > 0.0 || partsRemoved > 0 || destroyed;
        // The trace already contains every Sable contact/CCD sample.  Emit verbose
        // crash-contact text only when it actually changes IV gameplay, rather
        // than repeating resting BODY contacts every diagnostic interval.
        boolean diagnosticReady = materialEvent;
        if (diagnosticReady) {
            for (String diagnostic : contactDiagnostics) {
                if (PMIVObserver.loggingEnabled()) {
                    PMIVObserver.log(diagnostic);
                }
            }
            if (PMIVObserver.loggingEnabled()) {
                PMIVObserver.log(
                "SABLE_COLLISION_GAMEPLAY gameTime=" + simulationGameTime(vehicle)
                    + " uuid=" + vehicle.uniqueUUID
                    + " terrainContacts=" + terrainContacts
                    + " sweptTerrainImpacts=" + swept.size()
                    + " uniqueTerrainBlocks=" + impactsByBlock.size()
                    + " maxContactForceN=" + maxForce
                    + " maxIntegratedImpactImpulseNs=" + maxImpulse
                    + " maxImpulseEquivalentImpactDeltaVMps=" + maxImpulseDeltaV
                    + " maxPreImpactNormalSpeedMps=" + maxClosing
                    + " maxPreImpactVehicleSpeedMps=" + maxVehicleSpeed
                    + " maxNativeIvVelocity=" + maxNativeIvVelocity
                    + " maxNativeIvCrashSpeed=" + maxNativeIvCrashSpeed
                    + " hitBlock=" + anyHitBlock
                    + " solidBlock=" + anySolidBlock
                    + " legacyIvBlockBreakDelayReady=" + blockBreakDelayReady
                    + " trueImpactSubmissionAllowedAtEnd=" + body.terrainDamageAllowedForCurrentCrashEpisode()
                    + " crashEpisodeState=" + body.crashEpisodeState()
                    + " crashEpisodeSettledTicks=" + body.crashEpisodeSettledTicks()
                    + " outOfHealthBefore=" + outOfHealthBeforeGameplay
                    + " outOfHealthAfter=" + outOfHealthAfterGameplay
                    + " totalLegacyCrashHardness=" + totalLegacyCrashHardness
                    + " trueImpactImpactsSubmitted=" + trueImpactImpactsSubmitted
                    + " maxTrueImpactEnergyJ=" + maxTrueImpactEnergyJ
                    + " maxTrueImpactAbsorbedEnergyJ=" + maxTrueImpactAbsorbedEnergyJ
                    + " trueImpactImmediateBlocksBroken=" + trueImpactImmediateBlocksBroken
                    + " blocksBrokenByPmiv=" + blocksBroken
                    + " collisionBoxesDamagedByPmiv=" + collisionBoxesDamaged
                    + " crashDamage=" + crashDamage
                    + " partsRemoved=" + partsRemoved
                    + " destroyed=" + destroyed
                    + " catastrophicWrecked=" + catastrophicWrecked
                    + " nativeVehicleDestroySuppressedForCrashEpisode=" + catastrophicWrecked
                    + " terrainDamageFormula=INTEGRATED_TRUE_IMPACT_0_5_8_EXTERNAL_WORLD_MATERIAL_RESPONSE"
                    + " terrainImpactEnergy=INTEGRATED_TRUE_IMPACT_0_5_8_DERIVED_HALF_NORMAL_IMPULSE_TIMES_PREIMPACT_NORMAL_POINT_SPEED"
                    + " crashFormula=IV24_THRESHOLDS_LINEAR_DAMAGE_FROM_CONTACT_NORMAL_POINT_SPEED"
                    + " crashSpeedSource=SABLE_CONTACT_POINT_INWARD_NORMAL_SPEED"
                    + " crashDamageFactorApplied=false"
                    + " ivBlockVelocityLossApplied=false"
                    + " ivBlockVelocityLossReason=SABLE_SOLE_MOMENTUM_AUTHORITY"
                    + " supportImpulseGameplay=false"
                    + " contactDetectionAuthority=SABLE_RAPIER_PLUS_SABLE_CCD"
                    + " physicalImpulseAuthority=SABLE_RAPIER_PLUS_SABLE_CCD"
                    + " terrainDamageAuthority=INTEGRATED_TRUE_IMPACT_0_5_8"
                    + " vehicleCrashGameplayAuthority=IV24"
                    + " authoredBlockBoxContactDetection=PHYSICAL_PRIMARY_WITH_MODEL_SUPPLEMENT"
                    + " sourceBoxRole=PHYSICAL_AUTHORED_OR_MODEL_ATTRIBUTION"
                    + " contactPointLeverArmPreserved=true"
                    + " postContactComImpulseApplied=false"
                    + " postContactAngularVelocityMutationApplied=false"
                    + " terrainBlockRemovalTiming=INTEGRATED_TRUE_IMPACT_PRE_SABLE_FOR_SWEPT_PENETRATION_OR_DEFERRED_POST_TICK"
                    + " diagnosticRate=ON_MATERIAL_GAMEPLAY_EVENT"
                    + sourceBoxDiagnostic(strongestSource)
            );
            }
            PMIVObserver.captureCollisionGameplay(
                vehicle.uniqueUUID, simulationGameTime(vehicle), terrainContacts, swept.size(),
                impactsByBlock.size(), maxForce, maxImpulse, maxClosing, maxVehicleSpeed,
                maxNativeIvCrashSpeed, trueImpactImpactsSubmitted, maxTrueImpactEnergyJ,
                maxTrueImpactAbsorbedEnergyJ, trueImpactImmediateBlocksBroken,
                blocksBroken, collisionBoxesDamaged, crashDamage, partsRemoved, destroyed,
                catastrophicWrecked, outOfHealthBeforeGameplay, outOfHealthAfterGameplay,
                body.crashEpisodeState(), body.crashEpisodeSettledTicks(),
                body.terrainDamageAllowedForCurrentCrashEpisode()
            );
        }
    }

    /**
     * Mirrors a crash into IV's authored collision-group health without replacing
     * overall vehicle-health gameplay.  The source BoundingBox carries IV's real
     * groupDef, and damageCollisionBox() owns collision_N_damage/totaled sync.
     * Scaling preserves the same fractional severity as the whole-aircraft crash
     * while honoring an authored positive damageMultiplier.
     */
    private static double damageAuthoredCollisionGroup(
        EntityVehicleF_Physics vehicle, BoundingBox sourceBox, double vehicleDamage
    ) {
        if (vehicle == null || !(vehicleDamage > 0.0)) {
            return 0.0;
        }
        if (sourceBox == null) {
            logLocalizedDamageSkip(vehicle, sourceBox, "NO_SOURCE_BOX");
            return 0.0;
        }
        if (sourceBox.groupDef == null) {
            logLocalizedDamageSkip(vehicle, sourceBox, "SOURCE_BOX_HAS_NO_GROUP_DEF");
            return 0.0;
        }
        if (sourceBox.groupDef.health <= 0) {
            logLocalizedDamageSkip(vehicle, sourceBox, "GROUP_HAS_NO_POSITIVE_HEALTH");
            return 0.0;
        }
        if (vehicle.definition == null || vehicle.definition.general == null
            || vehicle.definition.collisionGroups == null) {
            logLocalizedDamageSkip(vehicle, sourceBox, "VEHICLE_COLLISION_GROUP_DEFINITION_UNAVAILABLE");
            return 0.0;
        }
        int groupIndex = vehicle.definition.collisionGroups.indexOf(sourceBox.groupDef);
        if (groupIndex < 0) {
            logLocalizedDamageSkip(vehicle, sourceBox, "GROUP_DEF_NOT_IN_VEHICLE_COLLISION_GROUPS");
            return 0.0;
        }
        double before = vehicle.getOrCreateVariable(
            "collision_" + (groupIndex + 1) + "_damage"
        ).currentValue;
        double remaining = sourceBox.groupDef.health - Math.max(0.0, before);
        if (!(remaining > 0.0)) {
            logLocalizedDamageSkip(vehicle, sourceBox, "GROUP_ALREADY_TOTALED");
            return 0.0;
        }
        double vehicleHealth = Math.max(1.0, vehicle.definition.general.health);
        double multiplier = sourceBox.groupDef.damageMultiplier > 0.0F
            ? sourceBox.groupDef.damageMultiplier : 1.0;
        double localized = vehicleDamage / vehicleHealth
            * sourceBox.groupDef.health * multiplier;
        localized = Math.min(remaining, Math.max(0.0, localized));
        if (!(localized > 0.0) || !Double.isFinite(localized)) {
            return 0.0;
        }
        vehicle.damageCollisionBox(sourceBox, localized);
        if (PMIVObserver.loggingEnabled()) {
            PMIVObserver.log(
            "SABLE_LOCALIZED_COLLISION_GROUP_DAMAGE gameTime=" + simulationGameTime(vehicle)
                + " uuid=" + vehicle.uniqueUUID
                + " groupIndex=" + (groupIndex + 1)
                + " vehicleDamage=" + vehicleDamage
                + " localizedDamage=" + localized
                + " groupHealth=" + sourceBox.groupDef.health
                + " groupDamageBefore=" + before
                + " groupDamageAfter=" + Math.min(sourceBox.groupDef.health, before + localized)
                + " damageMultiplier=" + multiplier
                + " authority=IV_DAMAGE_COLLISION_BOX"
        );
        }
        return localized;
    }

    private static void logLocalizedDamageSkip(
        EntityVehicleF_Physics vehicle, BoundingBox sourceBox, String reason
    ) {
        if (vehicle == null) return;
        if (PMIVObserver.loggingEnabled()) {
            PMIVObserver.log(
            "SABLE_LOCALIZED_COLLISION_GROUP_DAMAGE_SKIPPED gameTime=" + simulationGameTime(vehicle)
                + " uuid=" + vehicle.uniqueUUID
                + " reason=" + reason
                + " sourceBoxPresent=" + (sourceBox != null)
                + " groupDefPresent=" + (sourceBox != null && sourceBox.groupDef != null)
                + " groupHealth=" + (
                    sourceBox != null && sourceBox.groupDef != null
                        ? sourceBox.groupDef.health : -1.0
                )
                + " damageMultiplier=" + (
                    sourceBox != null && sourceBox.groupDef != null
                        ? sourceBox.groupDef.damageMultiplier : -1.0
                )
        );
        }
    }

    private static BoundingBox attributionBox(
        EntityVehicleF_Physics vehicle,
        BoundingBox sourceBox
    ) {
        if (sourceBox != null) {
            return sourceBox;
        }
        return vehicle == null ? null : vehicle.encompassingBox;
    }

    private static void mergeImpact(
        Map<BlockKey, ImpactEvidence> impacts,
        ImpactEvidence candidate
    ) {
        if (candidate == null || candidate.block() == null) {
            return;
        }
        BlockKey key = BlockKey.of(candidate.block());
        ImpactEvidence current = impacts.get(key);
        if (current == null || stronger(candidate, current)) {
            impacts.put(key, candidate);
        }
    }

    private static boolean stronger(ImpactEvidence candidate, ImpactEvidence current) {
        boolean candidatePreResolved = candidate.trueImpactResolution() != null
            && candidate.trueImpactResolution().accepted();
        boolean currentPreResolved = current.trueImpactResolution() != null
            && current.trueImpactResolution().accepted();
        if (candidatePreResolved != currentPreResolved) {
            return candidatePreResolved;
        }
        if (candidate.preImpactVehicleSpeedMps() > current.preImpactVehicleSpeedMps() + 1.0E-9) {
            return true;
        }
        if (current.preImpactVehicleSpeedMps() > candidate.preImpactVehicleSpeedMps() + 1.0E-9) {
            return false;
        }
        if (candidate.inwardPointSpeedMps() > current.inwardPointSpeedMps() + 1.0E-9) {
            return true;
        }
        if (current.inwardPointSpeedMps() > candidate.inwardPointSpeedMps() + 1.0E-9) {
            return false;
        }
        if (candidate.impulseNewtonSeconds() > current.impulseNewtonSeconds() + 1.0E-9) {
            return true;
        }
        return current.sourceBox() == null && candidate.sourceBox() != null;
    }

    /** Local point-closing speed is retained as physical evidence, not damage scale. */
    private static double preImpactNormalSpeed(
        SableVehicleBody body,
        SableCollisionCapture.SceneSnapshot snapshot,
        SableCompoundCollider.GameplayContact contact
    ) {
        SableCollisionCapture.BodyMotionSnapshot motion = snapshot.bodyMotion(
            body.getRuntimeId(), contact.substepIndex()
        );
        if (motion == null || contact.parentLocalPoint() == null
            || contact.parentLocalNormal() == null) {
            return 0.0;
        }
        Quaterniond orientation = new Quaterniond(motion.orientationWorld()).normalize();
        Vector3d radiusWorld = orientation.transform(new Vector3d(contact.parentLocalPoint()));
        Vector3d normalWorld = orientation.transform(new Vector3d(contact.parentLocalNormal()));
        if (normalWorld.lengthSquared() <= 1.0E-12) {
            return 0.0;
        }
        normalWorld.normalize();
        Vector3d pointVelocityWorld = new Vector3d(motion.angularVelocityWorld())
            .cross(radiusWorld)
            .add(motion.linearVelocityWorld());
        double speed = Math.max(0.0, pointVelocityWorld.dot(normalWorld));
        return Double.isFinite(speed) ? speed : 0.0;
    }

    /** Exact pre-substep Sable COM speed used to feed IV's native crash formula. */
    private static double preImpactVehicleSpeed(
        SableVehicleBody body,
        SableCollisionCapture.SceneSnapshot snapshot,
        SableCompoundCollider.GameplayContact contact
    ) {
        SableCollisionCapture.BodyMotionSnapshot motion = snapshot.bodyMotion(
            body.getRuntimeId(), contact.substepIndex()
        );
        if (motion == null || motion.linearVelocityWorld() == null) {
            return 0.0;
        }
        double speed = motion.linearVelocityWorld().length();
        return Double.isFinite(speed) ? Math.max(0.0, speed) : 0.0;
    }

    static double nativeIvSpeedFactor(EntityVehicleF_Physics vehicle) {
        double factor = vehicle == null ? 1.0 : Math.abs(vehicle.speedFactor);
        return Double.isFinite(factor) && factor > 1.0E-9 ? factor : 1.0;
    }

    /**
     * Maps a real Sable contact point to the concrete Minecraft terrain voxel.
     * This samples only around the Sable point/normal; no IV collision volume is
     * tested here and the result cannot invent a contact Sable did not report.
     */
    private static Point3D findTerrainBlock(
        EntityVehicleF_Physics vehicle,
        Vector3d worldPoint,
        Vector3d worldNormal
    ) {
        if (vehicle == null || worldPoint == null || worldNormal == null) {
            return null;
        }
        double[] offsets = new double[]{
            -SECOND_BLOCK_SAMPLE_OFFSET,
            -BLOCK_SAMPLE_OFFSET,
            0.0,
            BLOCK_SAMPLE_OFFSET,
            SECOND_BLOCK_SAMPLE_OFFSET
        };
        for (double offset : offsets) {
            Point3D candidate = new Point3D(
                Math.floor(worldPoint.x + worldNormal.x * offset),
                Math.floor(worldPoint.y + worldNormal.y * offset),
                Math.floor(worldPoint.z + worldNormal.z * offset)
            );
            if (!vehicle.world.isAir(candidate) && !vehicle.world.isBlockLiquid(candidate)) {
                return candidate;
            }
        }
        return null;
    }

    private static long simulationGameTime(EntityVehicleF_Physics vehicle) {
        if (vehicle.world instanceof WrapperWorldAccessor accessor) {
            return accessor.pmweatherIv$getLevel().getGameTime();
        }
        return vehicle.world.getTime();
    }

    private static String contactDiagnostic(
        EntityVehicleF_Physics vehicle,
        ImpactEvidence evidence,
        BoundingBox sourceBox,
        float hardness,
        boolean hitBlock,
        boolean trueImpactSubmitted,
        double trueImpactEnergyJ,
        double physicalPreImpactSpeedMps,
        double nativeIvVelocity,
        double nativeIvCrashSpeed
    ) {
        return "SABLE_CRASH_CONTACT gameTime=" + simulationGameTime(vehicle)
            + " uuid=" + vehicle.uniqueUUID
            + " evidence=" + evidence.evidenceSource()
            + " block=(" + (int) evidence.block().x + "," + (int) evidence.block().y
            + "," + (int) evidence.block().z + ")"
            + " worldPoint=" + vectorDiagnostic(evidence.worldPoint())
            + " worldNormal=" + vectorDiagnostic(evidence.worldNormal())
            + " preImpactComSpeedMps=" + physicalPreImpactSpeedMps
            + " inwardPointSpeedMps=" + evidence.inwardPointSpeedMps()
            + " impulseNs=" + evidence.impulseNewtonSeconds()
            + " forceN=" + evidence.forceNewtons()
            + " ivSpeedFactor=" + nativeIvSpeedFactor(vehicle)
            + " nativeIvVelocity=" + nativeIvVelocity
            + " nativeIvCrashSpeed=" + nativeIvCrashSpeed
            + " hardness=" + hardness
            + " hitBlock=" + hitBlock
            + " trueImpactEnergyJ=" + trueImpactEnergyJ
            + " trueImpactSubmitted=" + trueImpactSubmitted
            + " trueImpactPreResolved=" + (evidence.trueImpactResolution() != null
                && evidence.trueImpactResolution().accepted())
            + " trueImpactMode=" + (evidence.trueImpactResolution() == null
                ? "NONE" : evidence.trueImpactResolution().mode())
            + " trueImpactAbsorbedEnergyJ=" + (evidence.trueImpactResolution() == null
                ? 0.0 : evidence.trueImpactResolution().absorbedEnergyJ())
            + " trueImpactResidualEnergyJ=" + (evidence.trueImpactResolution() == null
                ? 0.0 : evidence.trueImpactResolution().residualEnergyJ())
            + " trueImpactBlocksBroken=" + (evidence.trueImpactResolution() == null
                ? 0 : evidence.trueImpactResolution().blocksBroken())
            + " contactDetection=SABLE_ONLY"
            + " sourceBoxUsedForDetection=false"
            + " sourceBoxUsedForDamageAttribution=" + (sourceBox != null)
            + sourceBoxDiagnostic(sourceBox);
    }

    private static Vector3d copy(Vector3d value) {
        return value == null ? null : new Vector3d(value);
    }

    private static String vectorDiagnostic(Vector3d value) {
        return value == null
            ? "NONE"
            : "(" + value.x + "," + value.y + "," + value.z + ")";
    }

    private static String sourceBoxDiagnostic(BoundingBox box) {
        if (box == null) {
            return " strongestSourceBox=NONE";
        }
        return " strongestSourceLocalCenter=("
            + box.localCenter.x + "," + box.localCenter.y + "," + box.localCenter.z + ")"
            + " strongestSourceHalfExtents=("
            + box.widthRadius + "," + box.heightRadius + "," + box.depthRadius + ")"
            + " strongestSourceCollisionTypes=" + String.valueOf(box.collisionTypes);
    }

    private record ImpactEvidence(
        Point3D block,
        BoundingBox sourceBox,
        Vector3d worldPoint,
        Vector3d worldNormal,
        double preImpactVehicleSpeedMps,
        double inwardPointSpeedMps,
        double impulseNewtonSeconds,
        double forceNewtons,
        float preImpactBlockHardness,
        Resolution trueImpactResolution,
        String evidenceSource
    ) {
    }

    private record BlockKey(int x, int y, int z) {
        private static BlockKey of(Point3D point) {
            return new BlockKey((int) point.x, (int) point.y, (int) point.z);
        }
    }
}
