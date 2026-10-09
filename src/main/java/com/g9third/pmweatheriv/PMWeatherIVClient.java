package com.g9third.pmweatheriv;

import com.g9third.pmweatheriv.devsupport.PMIVObserver;
import com.g9third.pmweatheriv.client.AutoTrimClient;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.IEventBus;
import net.neoforged.fml.ModContainer;
import net.neoforged.fml.common.Mod;

/** Loads the optional client observer; IV owns normal client vehicle sync. */
@Mod(value = PMWeatherIV.MOD_ID, dist = Dist.CLIENT)
public final class PMWeatherIVClient {
    public PMWeatherIVClient(IEventBus modBus, ModContainer modContainer) {
        PMIVObserver.initializeClient(modBus, modContainer);
        modBus.addListener(AutoTrimClient::registerKeyMappings);
        modBus.addListener(com.g9third.pmweatheriv.client.ClientWindMonitor::registerCommands);
        modBus.addListener(com.g9third.pmweatheriv.client.ClientWindMonitor::registerGuiLayer);
        net.neoforged.neoforge.common.NeoForge.EVENT_BUS.addListener(com.g9third.pmweatheriv.client.ClientWindMonitor::onClientTick);
        net.neoforged.neoforge.common.NeoForge.EVENT_BUS.addListener(AutoTrimClient::onClientTick);
        net.neoforged.neoforge.common.NeoForge.EVENT_BUS.addListener(AutoTrimClient::renderBadge);
    }
}
