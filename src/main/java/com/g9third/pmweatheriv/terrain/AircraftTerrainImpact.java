package com.g9third.pmweatheriv.terrain;

import com.g9third.pmweatheriv.terrain.trueimpact.ExternalWorldImpactModel;
import net.minecraft.core.BlockPos;
import net.minecraft.server.level.ServerLevel;
import org.joml.Vector3d;
import java.util.UUID;

/**
 * PMIV-facing adapter for the integrated True Impact 0.5.8-derived terrain model.
 *
 * <p>This is no longer a runtime bridge to a separate True Impact mod. The relevant
 * LGPL-3.0-only world-material code is incorporated into PMIV under
 * {@code com.g9third.pmweatheriv.terrain.trueimpact}. Sable remains the sole aircraft
 * rigid-body/momentum authority.</p>
 */
public final class AircraftTerrainImpact {
    private AircraftTerrainImpact() {}

    /** True Impact 0.5.8's canonical external-contact energy definition. */
    public static double deriveImpactEnergyJ(
        double normalImpulseNewtonSeconds,
        double inwardPointSpeedMetersPerSecond
    ) {
        return ExternalWorldImpactModel.deriveImpactEnergyJ(
            normalImpulseNewtonSeconds, inwardPointSpeedMetersPerSecond
        );
    }

    /**
     * Sends an already-solved aircraft/world contact into the integrated deferred
     * material path. Returns accepted physical impact energy, or zero when gated.
     */
    public static double submitExternalWorldImpact(
        ServerLevel level,
        BlockPos blockPos,
        Vector3d worldNormal,
        double normalImpulseNewtonSeconds,
        double inwardPointSpeedMetersPerSecond
    ) {
        return submitExternalWorldImpact(level, blockPos, worldNormal,
            normalImpulseNewtonSeconds, inwardPointSpeedMetersPerSecond, null);
    }

    /** Origin-aware overload for associating deferred terrain effects with a vehicle. */
    public static double submitExternalWorldImpact(
        ServerLevel level,
        BlockPos blockPos,
        Vector3d worldNormal,
        double normalImpulseNewtonSeconds,
        double inwardPointSpeedMetersPerSecond,
        UUID originVehicleUuid
    ) {
        if (level == null || blockPos == null) return 0.0;
        double nx = worldNormal == null ? Double.NaN : worldNormal.x;
        double ny = worldNormal == null ? Double.NaN : worldNormal.y;
        double nz = worldNormal == null ? Double.NaN : worldNormal.z;
        return ExternalWorldImpactModel.enqueueExternalWorldImpact(
            level,
            blockPos.getX(), blockPos.getY(), blockPos.getZ(),
            nx, ny, nz,
            normalImpulseNewtonSeconds, inwardPointSpeedMetersPerSecond, originVehicleUuid
        );
    }

    /**
     * Resolves terrain failure before Sable commits a swept rigid stop. The integrated
     * material model may fracture world blocks and returns the residual inward speed;
     * PMIV/Sable alone apply the resulting aircraft momentum response.
     */
    public static ExternalWorldImpactModel.Resolution resolveExternalPenetration(
        ServerLevel level,
        BlockPos seedBlock,
        Vector3d worldPoint,
        Vector3d worldNormal,
        double projectedContactAreaSquareMeters,
        double normalImpulseNewtonSeconds,
        double inwardPointSpeedMetersPerSecond,
        double maxTravelMeters
    ) {
        return resolveExternalPenetration(level, seedBlock, worldPoint, worldNormal,
            projectedContactAreaSquareMeters, normalImpulseNewtonSeconds,
            inwardPointSpeedMetersPerSecond, maxTravelMeters, null);
    }

    /** Origin-aware overload for associating deferred terrain effects with a vehicle. */
    public static ExternalWorldImpactModel.Resolution resolveExternalPenetration(
        ServerLevel level,
        BlockPos seedBlock,
        Vector3d worldPoint,
        Vector3d worldNormal,
        double projectedContactAreaSquareMeters,
        double normalImpulseNewtonSeconds,
        double inwardPointSpeedMetersPerSecond,
        double maxTravelMeters,
        UUID originVehicleUuid
    ) {
        if (level == null || seedBlock == null || worldPoint == null || worldNormal == null) return null;
        return ExternalWorldImpactModel.resolveExternalPenetration(
                level,
                seedBlock.getX(), seedBlock.getY(), seedBlock.getZ(),
                worldPoint.x, worldPoint.y, worldPoint.z,
                worldNormal.x, worldNormal.y, worldNormal.z,
                projectedContactAreaSquareMeters,
                normalImpulseNewtonSeconds,
                inwardPointSpeedMetersPerSecond,
                maxTravelMeters,
                originVehicleUuid
            );
    }
}
