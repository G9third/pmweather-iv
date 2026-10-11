package com.g9third.pmweatheriv.client;

import com.g9third.pmweatheriv.network.AutoTrimNetwork;
import com.g9third.pmweatheriv.physics.AirframePreparation;
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
import net.minecraft.world.level.Level;
import net.minecraft.world.entity.player.Player;
import net.neoforged.neoforge.client.event.RegisterKeyMappingsEvent;
import net.neoforged.neoforge.client.event.RegisterGuiLayersEvent;
import net.minecraft.resources.ResourceLocation;
import net.neoforged.neoforge.client.event.ClientTickEvent;
import org.lwjgl.glfw.GLFW;

/** Small in-flight auto-trim badge and its single rebindable toggle key. */
public final class AutoTrimClient {
    private static final long ENABLE_PROMPT_DURATION_TICKS = 80L;
    private static final KeyMapping TOGGLE = new KeyMapping(
        "key.pmweather_iv.auto_trim_panel", GLFW.GLFW_KEY_RIGHT_ALT, "key.categories.pmweather_iv");
    private static Level promptLevel;
    private static java.util.UUID promptAircraft;
    private static java.util.UUID promptSeat;
    private static boolean promptDecisionMade;
    private static long promptUntilTick = Long.MIN_VALUE;

    private AutoTrimClient() {}

    public static void registerKeyMappings(RegisterKeyMappingsEvent event) {
        event.register(TOGGLE);
    }

    public static void onClientTick(ClientTickEvent.Post event) {
        Minecraft minecraft = Minecraft.getInstance();
        Player player = minecraft.player;
        while (TOGGLE.consumeClick()) {
            PartSeat toggleSeat = player == null ? null : currentControllerSeat(player);
            if (toggleSeat != null && minecraft.level != null && minecraft.screen == null) {
                AutoTrimNetwork.sendToggleRequest();
                resetEnablePrompt(minecraft.level, toggleSeat.vehicleOn.uniqueUUID);
                promptSeat = toggleSeat.uniqueUUID;
                promptDecisionMade = true;
                promptUntilTick = Long.MIN_VALUE;
            }
        }
        if (player == null || minecraft.level == null) {
            AutoTrimNetwork.resetClient(null, 0L);
            resetEnablePrompt(null, null);
            return;
        }
        AutoTrimNetwork.resetClient(minecraft.level, minecraft.level.getGameTime());
        PartSeat seat = currentControllerSeat(player);
        if (seat == null) resetEnablePrompt(null, null);
        else updateEnablePrompt(minecraft.level, seat);
    }

    public static void registerGuiLayer(RegisterGuiLayersEvent event) {
        event.registerAboveAll(ResourceLocation.fromNamespaceAndPath(
            "pmweather_iv", "auto_trim"), (graphics, deltaTracker) -> renderBadge(graphics));
    }

    private static void renderBadge(GuiGraphics graphics) {
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
        int width = minecraft.getWindow().getGuiScaledWidth();
        int height = minecraft.getWindow().getGuiScaledHeight();
        // The temporary seat-entry toggle hint is separate from the persistent instrument panel.
        // When auto trim is off, keep that hint visible for its normal short duration,
        // but never draw the OFF/status panel.
        if (state == AutoTrimController.State.OFF) {
            if (shouldShowEnablePrompt(minecraft.level, vehicle, status)) {
                renderEnablePrompt(graphics, minecraft, width, height, false);
            }
            return;
        }
        AutoTrimNetwork.StatusReason reason = AutoTrimNetwork.StatusReason.values()[status.reason()];
        double trim = status.trim();
        boolean enabled = state != AutoTrimController.State.OFF;
        Component header = enabled
            ? Component.translatable("hud.pmweather_iv.auto_trim.value",
                String.format(java.util.Locale.ROOT, "%+.1f", trim))
            : Component.translatable("hud.pmweather_iv.auto_trim.off");
        Component detail = switch (state) {
            case ON -> Component.translatable(status.adjusting()
                ? "hud.pmweather_iv.auto_trim.adjusting" : "hud.pmweather_iv.auto_trim.holding");
            case LEARNING -> reasonText(AutoTrimNetwork.StatusReason.LEARNING);
            case PAUSED -> Component.translatable("hud.pmweather_iv.auto_trim.hold", reasonText(reason));
            case LIMITED -> Component.translatable("hud.pmweather_iv.auto_trim.limit", reasonText(reason));
            case OFF -> Component.empty();
        };
        // IV uses compact, plain instrument labels. Measure complete text and
        // wrap only at a narrow viewport; never replace the reason with an ellipsis.
        int maximumWidth = Math.max(1, width - 16);
        int panelWidth = Math.min(maximumWidth, Math.max(94,
            Math.max(minecraft.font.width(header), minecraft.font.width(detail)) + 16));
        int textWidth = Math.max(1, panelWidth - 16);
        var headerLines = minecraft.font.split(header, textWidth);
        var detailLines = enabled ? minecraft.font.split(detail, textWidth) : java.util.List.<net.minecraft.util.FormattedCharSequence>of();
        int panelHeight = 8 + (headerLines.size() + detailLines.size()) * minecraft.font.lineHeight;
        int x = Math.max(0, width - panelWidth - 8);
        int y = Math.max(8, height - panelHeight - 58);
        int accent = switch (state) {
            case ON -> status.adjusting() ? 0xFF91B879 : 0xFFCCCCCC;
            case LEARNING, PAUSED -> 0xFFD4B86A;
            case LIMITED -> 0xFFC87868;
            case OFF -> 0xFF777777;
        };
        graphics.fill(x, y, x + panelWidth, y + panelHeight, 0xAA181818);
        graphics.fill(x, y, x + panelWidth, y + 1, 0xAA777777);
        graphics.fill(x + 4, y + 6, x + 6, y + 8, accent);
        int textY = y + 4;
        for (var line : headerLines) {
            graphics.drawString(minecraft.font, line, x + 10, textY, 0xFFE8E8E8, false);
            textY += minecraft.font.lineHeight;
        }
        for (var line : detailLines) {
            graphics.drawString(minecraft.font, line, x + 10, textY, 0xFFBBBBBB, false);
            textY += minecraft.font.lineHeight;
        }
        if (shouldShowEnablePrompt(minecraft.level, vehicle, status)) {
            renderEnablePrompt(graphics, minecraft, width, height, state != AutoTrimController.State.OFF);
        }
    }

    private static void updateEnablePrompt(Level level, PartSeat seat) {
        java.util.UUID aircraft = seat.vehicleOn.uniqueUUID;
        if (promptLevel != level || !java.util.Objects.equals(promptAircraft, aircraft)
            || !java.util.Objects.equals(promptSeat, seat.uniqueUUID)) {
            resetEnablePrompt(level, aircraft);
            promptSeat = seat.uniqueUUID;
        }
        if (promptDecisionMade) return;
        AutoTrimNetwork.StatusPayload status = AutoTrimNetwork.status(level, aircraft);
        if (status == null) return;
        // A cached dismount status may precede the new pilot's first heartbeat.
        // Wait for an active or explicitly off status before choosing the hint.
        if (status.state() == (byte) AutoTrimController.State.OFF.ordinal()
            && status.reason() != (byte) AutoTrimNetwork.StatusReason.USER_DISABLED.ordinal()) return;
        promptDecisionMade = true;
        promptUntilTick = level.getGameTime() + ENABLE_PROMPT_DURATION_TICKS;
    }

    private static boolean shouldShowEnablePrompt(Level level, EntityVehicleF_Physics vehicle,
                                                   AutoTrimNetwork.StatusPayload status) {
        return level != null && vehicle != null && status != null
            && promptLevel == level && java.util.Objects.equals(promptAircraft, vehicle.uniqueUUID)
            && promptDecisionMade && promptUntilTick != Long.MIN_VALUE
            && level.getGameTime() < promptUntilTick;
    }

    private static void renderEnablePrompt(GuiGraphics graphics, Minecraft minecraft, int width, int height,
                                           boolean enabled) {
        Component prompt = Component.translatable(enabled
                ? "hud.pmweather_iv.auto_trim.disable_prompt" : "hud.pmweather_iv.auto_trim.enable_prompt",
            TOGGLE.getKey().getDisplayName());
        var lines = minecraft.font.split(prompt, Math.max(1, width - 16));
        int y = Math.max(8, height - 100 - (lines.size() - 1) * minecraft.font.lineHeight);
        for (var line : lines) {
            int x = width / 2 - minecraft.font.width(line) / 2;
            graphics.drawString(minecraft.font, line, x, y, 0xFFFFFFFF, true);
            y += minecraft.font.lineHeight;
        }
    }

    private static void resetEnablePrompt(Level level, java.util.UUID aircraft) {
        promptLevel = level;
        promptAircraft = aircraft;
        promptSeat = null;
        promptDecisionMade = false;
        promptUntilTick = Long.MIN_VALUE;
    }

    private static Component reasonText(AutoTrimNetwork.StatusReason reason) {
        return Component.translatable("hud.pmweather_iv.auto_trim.reason."
            + reason.name().toLowerCase(java.util.Locale.ROOT));
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
        // The aerodynamic plan exists only on the server. Client eligibility
        // uses the same live rotor parts without preparing a second model.
        if (AirframePreparation.vehicleHasRotorPart(seat.vehicleOn)) return null;
        return seat;
    }

}
