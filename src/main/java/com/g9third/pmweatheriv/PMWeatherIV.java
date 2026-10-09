package com.g9third.pmweatheriv;

import com.g9third.pmweatheriv.sable.SableVehicleManager;
import com.g9third.pmweatheriv.compat.PMAeroBridge;
import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import com.g9third.pmweatheriv.network.LinkedSeatMountMaintenance;
import net.neoforged.bus.api.IEventBus;
import net.neoforged.fml.ModContainer;
import net.neoforged.fml.common.Mod;
import net.neoforged.fml.config.ModConfig;
import net.neoforged.neoforge.common.NeoForge;
import com.g9third.pmweatheriv.terrain.trueimpact.IntegratedImpactTerrain;

/** Shared aircraft and ground-vehicle physics with IV networking/gameplay mirroring. */
@Mod(PMWeatherIV.MOD_ID)
public final class PMWeatherIV {
    public static final String MOD_ID = "pmweather_iv";
    public static final String VERSION = "0.12.0-rc1";

    public PMWeatherIV(IEventBus modBus, ModContainer modContainer) {
        PMAeroBridge.requireApis();
        PMIVObserver.initializeCommon(modBus, modContainer);
        modBus.addListener(PMWeatherIVConfig::onLoading);
        modBus.addListener(PMWeatherIVConfig::onReloading);
        modBus.addListener(com.g9third.pmweatheriv.network.AircraftStateNetwork::registerPayloads);
        modBus.addListener(com.g9third.pmweatheriv.network.AutoTrimNetwork::registerPayloads);
        modBus.addListener(com.g9third.pmweatheriv.network.RoadSuspensionNetwork::registerPayloads);
        modBus.addListener(com.g9third.pmweatheriv.network.WindMonitorNetwork::registerPayloads);
        modContainer.registerConfig(ModConfig.Type.COMMON, PMWeatherIVConfig.SPEC);
        NeoForge.EVENT_BUS.addListener(SableVehicleManager::onPrePhysicsTick);
        NeoForge.EVENT_BUS.addListener(SableVehicleManager::onPostPhysicsTick);
        NeoForge.EVENT_BUS.addListener(SableVehicleManager::onLevelUnload);
        NeoForge.EVENT_BUS.addListener(LinkedSeatMountMaintenance::onPlayerLoggedIn);
        NeoForge.EVENT_BUS.addListener(LinkedSeatMountMaintenance::onPlayerTickPost);
        NeoForge.EVENT_BUS.addListener(com.g9third.pmweatheriv.network.AutoTrimNetwork::onPlayerLoggedOut);
        NeoForge.EVENT_BUS.addListener(com.g9third.pmweatheriv.network.RoadSuspensionNetwork::onServerStopping);
        NeoForge.EVENT_BUS.addListener(com.g9third.pmweatheriv.network.WindMonitorNetwork::onServerStopping);
        NeoForge.EVENT_BUS.addListener(IntegratedImpactTerrain::onServerTickPost);
        NeoForge.EVENT_BUS.addListener(IntegratedImpactTerrain::onServerStopped);
        NeoForge.EVENT_BUS.addListener(PMIVObserver::onServerStarted);
        NeoForge.EVENT_BUS.addListener(PMIVObserver::onServerStopping);
        if (PMIVObserver.loggingEnabled()) {
            PMIVObserver.log("EVENT modLoaded version=" + VERSION
                + " aerodynamicAuthority=PMAERO requiredVersion=1.0"
                + " integrationAndCollision=SABLE_RAPIER terrainDamage=INTEGRATED_TRUE_IMPACT_0_5_8 networkAndGameplay=IV"
                + " standaloneTrueImpact=OPTIONAL_SABLE_SUBLEVEL_FORWARDING"
                + " windSampling=FORCE_POINTS_ONCE_PER_OWNER_TICK"
                + " clientPose=IV_SERVER_DELTA_SYNC privateDevAutoTrace=CLOSEST_PMIV_VEHICLE");
        }
    }
}
