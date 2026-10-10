package com.g9third.pmweatheriv.client;

import com.g9third.pmweatheriv.network.AutoTrimNetwork;
import com.g9third.pmweatheriv.physics.AircraftState;
import com.g9third.pmweatheriv.physics.AircraftStateAccess;
import com.g9third.pmweatheriv.physics.AutoTrimController;
import mcinterface1211.BuilderEntityLinkedSeat;
import mcinterface1211.WrapperPlayer;
import mcinterface1211.WrapperWorld;
import minecrafttransportsimulator.entities.components.AEntityB_Existing;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartSeat;
import net.minecraft.client.Minecraft;
import net.minecraft.client.gui.GuiGraphics;
import net.minecraft.client.KeyMapping;
import net.minecraft.network.chat.Component;
import net.minecraft.world.entity.player.Player;
import net.neoforged.neoforge.client.event.RegisterKeyMappingsEvent;
import net.neoforged.neoforge.client.event.RenderGuiEvent;
import net.neoforged.neoforge.client.event.ClientTickEvent;
import org.lwjgl.glfw.GLFW;

/** Small in-flight auto-trim badge and its single rebindable toggle key. */
public final class AutoTrimClient {
    private static final KeyMapping TOGGLE = new KeyMapping(
        "key.pmweather_iv.auto_trim_panel", GLFW.GLFW_KEY_RIGHT_ALT, "key.categories.pmweather_iv");

    private AutoTrimClient() {}

    public static void registerKeyMappings(RegisterKeyMappingsEvent event) {
        event.register(TOGGLE);
    }

    public static void onClientTick(ClientTickEvent.Post event) {
        Minecraft minecraft = Minecraft.getInstance();
        Player player = minecraft.player;
        while (TOGGLE.consumeClick()) {
            if (player != null && minecraft.level != null && minecraft.screen == null
                && currentControllerSeat(player) != null) {
                AutoTrimNetwork.sendToggleRequest();
            }
        }
        if (player == null || minecraft.level == null) {
            AutoTrimNetwork.resetClient(null, 0L);
            return;
        }
        AutoTrimNetwork.resetClient(minecraft.level, minecraft.level.getGameTime());
        PartSeat seat = currentControllerSeat(player);
        if (seat != null && AutoTrimNetwork.consumeFirstEngagementNotice(
            minecraft.level, seat.vehicleOn.uniqueUUID)) {
            Component key = TOGGLE.getKey().getDisplayName();
            player.displayClientMessage(Component.translatable(
                "hud.pmweather_iv.auto_trim.first_notice", key), false);
        }
    }

    public static void renderBadge(RenderGuiEvent.Post event) {
        Minecraft minecraft = Minecraft.getInstance();
        Player player = minecraft.player;
        if (minecraft.level == null || player == null || minecraft.options.hideGui
            || minecraft.screen != null) return;
        PartSeat seat = currentControllerSeat(player);
        if (seat == null) return;
        EntityVehicleF_Physics vehicle = seat.vehicleOn;
        AutoTrimNetwork.StatusPayload status = AutoTrimNetwork.status(minecraft.level, vehicle.uniqueUUID);
        if (status == null) return;
        AutoTrimController.State state = AutoTrimController.State.values()[status.state()];
        AutoTrimNetwork.StatusReason reason = AutoTrimNetwork.StatusReason.values()[status.reason()];
        double trim = status.trim();
        GuiGraphics graphics = event.getGuiGraphics();
        int width = minecraft.getWindow().getGuiScaledWidth();
        int height = minecraft.getWindow().getGuiScaledHeight();
        int panelWidth = 130;
        int panelHeight = 26;
        int x = width - panelWidth - 8;
        int y = height - panelHeight - 58;
        int accent = switch (state) {
            case ON -> 0xFF72D4B6;
            case LEARNING -> 0xFFFFC56A;
            case PAUSED -> 0xFFFFA36B;
            case LIMITED -> 0xFFFF7777;
            case OFF -> 0xFF7E91A8;
        };
        graphics.fill(x, y, x + panelWidth, y + panelHeight, 0xD9141D28);
        graphics.fill(x, y, x + panelWidth, y + 1, accent);
        Component badge = Component.translatable("hud.pmweather_iv.auto_trim.badge", stateText(state));
        String trimValue = String.format(java.util.Locale.ROOT, "%+.1f", trim);
        Component trimText = Component.translatable("hud.pmweather_iv.auto_trim.trim",
            trimValue);
        Component detail = status.adjusting()
            ? Component.translatable("hud.pmweather_iv.auto_trim.adjusting", trimValue)
            : state == AutoTrimController.State.PAUSED || state == AutoTrimController.State.LIMITED
                ? reasonAndTrim(minecraft.font, reasonText(reason), trimText, panelWidth - 14)
                : state == AutoTrimController.State.OFF ? reasonText(reason) : trimText;
        graphics.drawString(minecraft.font, fit(minecraft.font, badge, panelWidth - 14),
            x + 7, y + 4, 0xFFE7EEF7, false);
        graphics.drawString(minecraft.font, fit(minecraft.font, detail, panelWidth - 14),
            x + 7, y + 15, state == AutoTrimController.State.PAUSED
                || state == AutoTrimController.State.LIMITED ? 0xFFFFC56A : accent, false);
    }

    private static Component stateText(AutoTrimController.State state) {
        return Component.translatable("hud.pmweather_iv.auto_trim.state." + state.name().toLowerCase(java.util.Locale.ROOT));
    }

    private static Component reasonText(AutoTrimNetwork.StatusReason reason) {
        return Component.translatable("hud.pmweather_iv.auto_trim.reason."
            + reason.name().toLowerCase(java.util.Locale.ROOT));
    }

    private static Component reasonAndTrim(net.minecraft.client.gui.Font font, Component reason,
                                           Component trim, int maxWidth) {
        String suffix = " · " + trim.getString();
        Component fittedReason = fit(font, reason, Math.max(0, maxWidth - font.width(suffix)));
        return Component.literal(fittedReason.getString() + suffix);
    }

    private static Component fit(net.minecraft.client.gui.Font font, Component component, int maxWidth) {
        String text = component.getString();
        if (font.width(text) <= maxWidth) return component;
        int end = 0;
        while (end < text.length()) {
            int next = end + Character.charCount(text.codePointAt(end));
            if (font.width(text.substring(0, next) + "…") > maxWidth) break;
            end = next;
        }
        return Component.literal(text.substring(0, end).stripTrailing() + "…");
    }

    private static PartSeat currentControllerSeat(Player player) {
        if (player == null || !(player.getVehicle() instanceof BuilderEntityLinkedSeat linkedSeat)
            || linkedSeat.level() != player.level() || !linkedSeat.hasPassenger(player)) return null;
        WrapperPlayer wrapper = WrapperPlayer.getWrapperFor(player);
        AEntityB_Existing riding = wrapper.getEntityRiding();
        if (!(riding instanceof PartSeat seat) || !seat.isValid || seat.placementDefinition == null
            || !seat.placementDefinition.isController || seat.vehicleOn == null
            || linkedSeat.entity != seat || seat.world != WrapperWorld.getWrapperFor(player.level())
            || !seat.vehicleOn.isValid || seat.vehicleOn.outOfHealth
            || seat.vehicleOn.definition == null || seat.vehicleOn.definition.motorized == null
            || !seat.vehicleOn.definition.motorized.isAircraft || seat.vehicleOn.definition.motorized.isBlimp
            || !com.g9third.pmweatheriv.sable.SableVehicleManager.supportsVehicle(seat.vehicleOn)
            || seat.rider == null || !seat.rider.equals(wrapper)) return null;
        AircraftState state = ((AircraftStateAccess) seat.vehicleOn).pmweatherIv$getAircraftState();
        if (state.plan != null && state.plan.rotorcraft()) return null;
        return seat;
    }

}
