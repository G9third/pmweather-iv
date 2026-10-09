package com.g9third.pmweatheriv.client;

import com.g9third.pmweatheriv.PMWeatherIV;
import com.g9third.pmweatheriv.network.WindMonitorNetwork;
import com.g9third.pmweatheriv.physics.Vec3d;
import com.mojang.brigadier.builder.LiteralArgumentBuilder;
import com.mojang.brigadier.arguments.StringArgumentType;
import java.util.Locale;
import minecrafttransportsimulator.entities.components.AEntityB_Existing;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import minecrafttransportsimulator.entities.instances.PartSeat;
import minecrafttransportsimulator.mcinterface.InterfaceManager;
import net.minecraft.ChatFormatting;
import net.minecraft.client.Minecraft;
import net.minecraft.client.gui.GuiGraphics;
import net.minecraft.commands.CommandSourceStack;
import net.minecraft.commands.Commands;
import net.minecraft.network.chat.Component;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.world.entity.player.Player;
import net.minecraft.world.level.Level;
import net.neoforged.neoforge.client.event.ClientTickEvent;
import net.neoforged.neoforge.client.event.RegisterClientCommandsEvent;
import net.neoforged.neoforge.client.event.RegisterGuiLayersEvent;
import net.neoforged.neoforge.network.PacketDistributor;
import net.neoforged.neoforge.network.registration.NetworkRegistry;

/** Client HUD for the authoritative server wind field supplied by PMAero. */
public final class ClientWindMonitor {
    private static final ResourceLocation HUD_LAYER =
        ResourceLocation.fromNamespaceAndPath(PMWeatherIV.MOD_ID, "wind_monitor");

    private static final int BACKGROUND = 0x99000000;
    private static final int PLANE_DARK = 0x806D8296;
    private static final int PLANE_LIGHT = 0xB0AFC7D8;
    private static final int ARROW_SHADOW = 0xD0001820;
    private static final int ARROW_SIDE = 0xFF087C92;
    private static final int ARROW_HIGHLIGHT = 0xFF53E2FF;
    private static final int TEXT_PRIMARY = 0xFFF2FBFF;
    private static final int TEXT_SECONDARY = 0xFFB9D6E2;
    private static final double MIN_DIRECTIONAL_SPEED_MPH = 0.05;
    private static final String[] CARDINALS = {
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
    };

    private static Level sampledLevel;
    private static WindReading reading = WindReading.UNAVAILABLE;
    private static boolean liveMonitoring;
    private static long lastRequestGameTick = Long.MIN_VALUE;
    private static long nextRequestId;
    private static long pendingReportId = Long.MIN_VALUE;
    private static long pendingReportTick;
    private static long lastReceiptTick = Long.MIN_VALUE;
    private static long lastAcceptedRequestId = Long.MIN_VALUE;
    private static String testPhase = "", testProgress = "";
    private static long testReceiptNanos;

    /** The PMIV weather test can display its current phase beside the wind reading. */
    public static void setTestStatus(String phase, String progress) {
        testPhase = phase;
        testProgress = progress;
        testReceiptNanos = System.nanoTime();
    }

    private ClientWindMonitor() {
    }

    public static void registerCommands(RegisterClientCommandsEvent event) {
        event.getDispatcher().register(Commands.literal("aerowind")
            .then(Commands.literal("live")
                .executes(context -> reportLiveStatus())
                .then(Commands.literal("on").executes(context -> setLive(true)))
                .then(Commands.literal("off").executes(context -> setLive(false))))
            .then(Commands.literal("wind").executes(context -> reportWind()))
            .then(Commands.literal("test")
                .executes(context -> forwardWeather(""))
                .then(Commands.argument("arguments", StringArgumentType.greedyString())
                    .executes(context -> forwardWeather(StringArgumentType.getString(context, "arguments"))))));
        event.getDispatcher().register(commandRoot("pmiv"));
        event.getDispatcher().register(commandRoot("pmweatheriv"));
    }

    private static LiteralArgumentBuilder<CommandSourceStack> commandRoot(String root) {
        return Commands.literal(root)
            .then(Commands.literal("wind")
                .executes(context -> reportWind())
                .then(Commands.literal("live")
                    .executes(context -> reportLiveStatus())
                    .then(Commands.literal("on").executes(context -> setLive(true)))
                    .then(Commands.literal("off").executes(context -> setLive(false)))))
            .then(Commands.literal("weather")
                .then(Commands.literal("test")
                    .executes(context -> forwardWeather(""))
                    .then(Commands.argument("arguments", StringArgumentType.greedyString())
                        .executes(context -> forwardWeather(StringArgumentType.getString(context, "arguments"))))));
    }

    private static int forwardWeather(String arguments) {
        Minecraft minecraft = Minecraft.getInstance();
        if (minecraft.getConnection() == null || minecraft.player == null) {
            sendMessage("Join a world before starting the Aerowind test.", ChatFormatting.RED);
            return 0;
        }
        minecraft.getConnection().sendCommand("aerowind test" + (arguments.isBlank() ? "" : " " + arguments));
        return 1;
    }

    public static void registerGuiLayer(RegisterGuiLayersEvent event) {
        event.registerAboveAll(HUD_LAYER, (guiGraphics, deltaTracker) -> renderHud(guiGraphics));
    }

    public static void onClientTick(ClientTickEvent.Post event) {
        Level currentLevel = Minecraft.getInstance().level;
        com.g9third.pmweatheriv.network.AircraftStateNetwork.resetClientLevel(currentLevel);
        if (sampledLevel != currentLevel) {
            resetSamplingState();
            sampledLevel = currentLevel;
        }
        applyLatestServerReading();
        if (currentLevel != null) {
            long tick = currentLevel.getGameTime();
            if (lastReceiptTick != Long.MIN_VALUE && tick - lastReceiptTick > 40L)
                reading = WindReading.UNAVAILABLE;
            if (pendingReportId != Long.MIN_VALUE && tick - pendingReportTick > 40L) {
                sendMessage("The server did not return wind data. Try again.", ChatFormatting.RED);
                pendingReportId = Long.MIN_VALUE;
            }
        }
        if (liveMonitoring) requestAuthoritativeWind(false);
    }

    private static int reportWind() {
        if (!requestAuthoritativeWind(true)) {
            sendMessage("Server wind data is unavailable.", ChatFormatting.RED);
            return 0;
        }
        pendingReportId = nextRequestId;
        pendingReportTick = Minecraft.getInstance().level.getGameTime();
        return 1;
    }

    private static int reportLiveStatus() {
        sendMessage(
            "Live wind display is " + (liveMonitoring ? "enabled." : "disabled.")
                + " Use /live wind on|off.",
            liveMonitoring ? ChatFormatting.GREEN : ChatFormatting.YELLOW
        );
        return 1;
    }

    private static int setLive(boolean enabled) {
        liveMonitoring = enabled;
        if (enabled) {
            resetSamplingState();
            boolean serverChannel = requestAuthoritativeWind(true);
            sendMessage(
                serverChannel
                    ? "Live wind display enabled (authoritative server wind). Use /live wind off to disable it."
                    : "Waiting for server wind data. Use /pmiv wind live off to disable the display.",
                serverChannel ? ChatFormatting.GREEN : ChatFormatting.YELLOW
            );
        } else {
            sendMessage("Live wind display disabled.", ChatFormatting.YELLOW);
        }
        return 1;
    }

    private static void resetSamplingState() {
        sampledLevel = null;
        reading = WindReading.UNAVAILABLE;
        lastRequestGameTick = Long.MIN_VALUE;
        lastAcceptedRequestId = Long.MIN_VALUE;
        pendingReportId = Long.MIN_VALUE;
        lastReceiptTick = Long.MIN_VALUE;
        WindMonitorNetwork.clearClientReading();
        testPhase = testProgress = "";
    }

    /**
     * Ask the logical server to sample PMAero atmosphere at this player's eye position.
     * This is the same source-native PMWeather path and common config used by aircraft physics.
     */
    private static boolean requestAuthoritativeWind(boolean force) {
        Minecraft minecraft = Minecraft.getInstance();
        if (minecraft.player == null || minecraft.level == null || minecraft.getConnection() == null) {
            return false;
        }
        if (!NetworkRegistry.hasChannel(
            minecraft.getConnection(),
            WindMonitorNetwork.RequestPayload.TYPE.id()
        )) {
            return false;
        }
        if (sampledLevel != minecraft.level) {
            resetSamplingState();
            sampledLevel = minecraft.level;
        }
        long gameTime = minecraft.level.getGameTime();
        if (!force && lastRequestGameTick != Long.MIN_VALUE && gameTime - lastRequestGameTick < 2L) {
            return true;
        }
        lastRequestGameTick = gameTime;
        PacketDistributor.sendToServer(new WindMonitorNetwork.RequestPayload(++nextRequestId));
        return true;
    }

    /** Apply the newest server response, dropping out-of-order replies. */
    private static void applyLatestServerReading() {
        WindMonitorNetwork.Reading server = WindMonitorNetwork.latestClientReading();
        if (server.requestId() <= lastAcceptedRequestId) return;
        lastAcceptedRequestId = server.requestId();
        Level level = Minecraft.getInstance().level;
        lastReceiptTick = level == null ? Long.MIN_VALUE : level.getGameTime();
        reading = server.available() && server.windMph() != null && server.windMph().isFinite()
            ? describe(server.windMph(), true, true)
            : WindReading.UNAVAILABLE;
        if (pendingReportId != Long.MIN_VALUE && server.requestId() >= pendingReportId) {
            sendMessage(reading.available() ? commandReading(reading) : "Server wind data is unavailable.",
                reading.available() ? ChatFormatting.AQUA : ChatFormatting.RED);
            pendingReportId = Long.MIN_VALUE;
        }
    }



    private static WindReading describe(
        Vec3d effectiveMph,
        boolean authoritativeServer,
        boolean pmweatherPresentAtSource
    ) {
        double horizontal = Math.hypot(effectiveMph.x(), effectiveMph.z());
        double total = effectiveMph.length();
        if (horizontal < MIN_DIRECTIONAL_SPEED_MPH) {
            boolean calm = Math.abs(effectiveMph.y()) < MIN_DIRECTIONAL_SPEED_MPH;
            return new WindReading(
                true, effectiveMph, total, horizontal,
                Double.NaN, calm,
                calm ? "calm" : "vertical", calm ? "calm" : "vertical",
                authoritativeServer, pmweatherPresentAtSource
            );
        }
        double towardDegrees = normalizeDegrees(Math.toDegrees(Math.atan2(effectiveMph.x(), -effectiveMph.z())));
        double fromDegrees = normalizeDegrees(towardDegrees + 180.0);
        return new WindReading(
            true, effectiveMph, total, horizontal,
            fromDegrees, false,
            cardinal16(towardDegrees), cardinal16(fromDegrees),
            authoritativeServer, pmweatherPresentAtSource
        );
    }

    private static String commandReading(WindReading value) {
        String source = value.authoritativeServer() ? "authoritative server" : "client fallback";
        if (value.calm()) {
            return value.pmweatherPresentAtSource()
                ? "PMWeather wind: calm (0.0 mph), " + source + "."
                : "Ambient wind: still air (PMWeather is not installed on the "
                    + (value.authoritativeServer() ? "server" : "client") + ").";
        }
        if (!Double.isFinite(value.fromDegrees())) {
            return String.format(
                Locale.ROOT,
                "PMWeather wind: %.1f mph, vertical only (%+.1f mph), %s.",
                value.totalSpeedMph(), value.effectiveMph().y(), source
            );
        }
        return String.format(
            Locale.ROOT,
            "PMWeather wind: %.1f mph from %s (%.0f°), flow toward %s; horizontal %.1f, vertical %+.1f mph, %s.",
            value.totalSpeedMph(), value.fromCardinal(), value.fromDegrees(),
            value.towardCardinal(), value.horizontalSpeedMph(), value.effectiveMph().y(), source
        );
    }

    private static void renderHud(GuiGraphics guiGraphics) {
        Minecraft minecraft = Minecraft.getInstance();
        if (!liveMonitoring
            || !reading.available()
            || minecraft.player == null
            || minecraft.level == null
            || minecraft.options.hideGui
            || minecraft.screen != null) {
            return;
        }

        HorizontalReference reference = horizontalReference(minecraft.player);
        RelativeDirection relative = relativeToPlayer(reading.effectiveMph(), reference.yawDegrees());

        // Compact bottom HUD: intentionally a narrow two-line strip above the
        // vanilla hotbar instead of the former large diagram + vertical gauge.
        // Only horizontal flow gets an arrow. Vertical wind is numeric so UP/DN
        // can never be mistaken for forward/back airflow.
        int centerX = guiGraphics.guiWidth() / 2;
        int panelBottom = guiGraphics.guiHeight() - 27;
        boolean showTest = !testPhase.isEmpty()
            && System.nanoTime() - testReceiptNanos < 3_000_000_000L;
        int panelTop = panelBottom - (showTest ? 63 : 39);
        int panelWidth = showTest ? Math.max(222,
            Math.max(minecraft.font.width(testPhase), minecraft.font.width(testProgress)) + 16) : 222;
        int panelLeft = centerX - panelWidth / 2;
        int panelRight = panelLeft + panelWidth;

        guiGraphics.fill(panelLeft, panelTop, panelRight, panelBottom, BACKGROUND);
        guiGraphics.fill(panelLeft, panelTop, panelRight, panelTop + 1, PLANE_LIGHT);

        int arrowX = panelLeft + 29;
        int arrowY = panelTop + 20;
        drawCompactHorizontalReference(guiGraphics, arrowX, arrowY);
        drawCompactHorizontalFlowArrow(guiGraphics, arrowX, arrowY, relative);

        String primary = reading.calm()
            ? "WIND  calm"
            : String.format(Locale.ROOT, "WIND  %.1f mph  FROM %s", reading.totalSpeedMph(), reading.fromCardinal());
        String vertical = compactVerticalLabel(reading.effectiveMph().y());
        String referenceLabel = reference.aircraftRelative() ? "AIR" : "VIEW";
        String sourceSuffix = reading.authoritativeServer() ? "" : "  LOCAL";
        String secondary = reading.calm()
            ? referenceLabel + sourceSuffix
            : String.format(
                Locale.ROOT,
                "H %.1f  V %s  %s%s",
                reading.horizontalSpeedMph(), vertical, referenceLabel, sourceSuffix
            );

        int textX = panelLeft + 60;
        guiGraphics.drawString(minecraft.font, primary, textX, panelTop + 10, TEXT_PRIMARY, false);
        guiGraphics.drawString(minecraft.font, secondary, textX, panelTop + 22, TEXT_SECONDARY, false);
        if (showTest) {
            guiGraphics.drawString(minecraft.font, testPhase, panelLeft + 8, panelTop + 36, TEXT_PRIMARY, false);
            guiGraphics.drawString(minecraft.font, testProgress, panelLeft + 8, panelTop + 48, TEXT_SECONDARY, false);
        }
    }

    /**
     * IV intentionally lets the rider's head/camera yaw differ from the vehicle
     * heading. Using vanilla player yaw while seated can therefore rotate the
     * live wind diagram by tens of degrees (or even roughly 90 degrees) relative
     * to an Aeronautics aircraft structure. Prefer the actual IV aircraft yaw
     * when riding one of its seats; other vehicles retain player/view-relative HUD.
     */
    private static HorizontalReference horizontalReference(Player minecraftPlayer) {
        try {
            if (InterfaceManager.clientInterface != null) {
                var wrapperPlayer = InterfaceManager.clientInterface.getClientPlayer();
                AEntityB_Existing riding = wrapperPlayer == null
                    ? null
                    : wrapperPlayer.getEntityRiding();
                if (riding instanceof PartSeat seat) {
                    EntityVehicleF_Physics vehicle = seat.vehicleOn;
                    if (vehicle != null && vehicle.definition != null
                        && vehicle.definition.motorized != null && vehicle.definition.motorized.isAircraft
                        && vehicle.orientation != null
                        && Double.isFinite(vehicle.orientation.angles.y)) {
                        // IV positive body yaw is opposite vanilla Minecraft's
                        // getYRot convention used by relativeToPlayer().
                        return new HorizontalReference(-vehicle.orientation.angles.y, true);
                    }
                }
            }
        } catch (RuntimeException ignored) {
            // The wind HUD is diagnostic-only. A transient IV client-wrapper
            // lifecycle race must never make the HUD or client tick fail.
        }
        return new HorizontalReference(minecraftPlayer.getYRot(), false);
    }


    private static RelativeDirection relativeToPlayer(Vec3d windMph, double playerYawDegrees) {
        double totalLength = windMph.length();
        if (totalLength < MIN_DIRECTIONAL_SPEED_MPH) {
            return RelativeDirection.ZERO;
        }
        double yaw = Math.toRadians(playerYawDegrees);
        double forwardX = -Math.sin(yaw);
        double forwardZ = Math.cos(yaw);
        double rightX = -Math.cos(yaw);
        double rightZ = -Math.sin(yaw);
        return new RelativeDirection(
            (windMph.x() * rightX + windMph.z() * rightZ) / totalLength,
            (windMph.x() * forwardX + windMph.z() * forwardZ) / totalLength
        );
    }

    /** Tiny top-down cross: screen up is aircraft/view forward, screen right is right. */
    private static void drawCompactHorizontalReference(GuiGraphics guiGraphics, int centerX, int centerY) {
        drawLine(guiGraphics, centerX - 16, centerY, centerX + 16, centerY, PLANE_DARK);
        drawLine(guiGraphics, centerX, centerY + 10, centerX, centerY - 10, PLANE_LIGHT);
        guiGraphics.fill(centerX - 1, centerY - 1, centerX + 2, centerY + 2, PLANE_LIGHT);
    }

    /**
     * The HUD has exactly one direction arrow. Cyan points where horizontal air
     * flows relative to the aircraft/view. World wind-from remains explicit text.
     */
    private static void drawCompactHorizontalFlowArrow(
        GuiGraphics guiGraphics,
        int originX,
        int originY,
        RelativeDirection direction
    ) {
        double projectedX = direction.right() * 22.0;
        double projectedY = -direction.forward() * 15.0;
        double projectedLength = Math.hypot(projectedX, projectedY);
        if (projectedLength < 1.5) {
            guiGraphics.fill(originX - 1, originY - 1, originX + 2, originY + 2, ARROW_HIGHLIGHT);
            return;
        }
        drawArrow(guiGraphics, originX, originY, projectedX, projectedY, ARROW_SIDE, ARROW_HIGHLIGHT);
    }

    private static String compactVerticalLabel(double verticalMph) {
        if (Math.abs(verticalMph) < MIN_DIRECTIONAL_SPEED_MPH) {
            return "0.0";
        }
        return String.format(Locale.ROOT, "%s%.1f", verticalMph > 0.0 ? "↑" : "↓", Math.abs(verticalMph));
    }

    private static void drawArrow(
        GuiGraphics guiGraphics,
        int originX,
        int originY,
        double projectedX,
        double projectedY,
        int sideColor,
        int highlightColor
    ) {
        double projectedLength = Math.hypot(projectedX, projectedY);
        if (projectedLength < 1.0e-9) {
            return;
        }
        int tipX = originX + (int) Math.round(projectedX);
        int tipY = originY + (int) Math.round(projectedY);
        double unitX = projectedX / projectedLength;
        double unitY = projectedY / projectedLength;
        double perpendicularX = -unitY;
        double perpendicularY = unitX;
        int headLeftX = tipX - (int) Math.round(unitX * 8.5 - perpendicularX * 5.0);
        int headLeftY = tipY - (int) Math.round(unitY * 8.5 - perpendicularY * 5.0);
        int headRightX = tipX - (int) Math.round(unitX * 8.5 + perpendicularX * 5.0);
        int headRightY = tipY - (int) Math.round(unitY * 8.5 + perpendicularY * 5.0);

        drawThickLine(guiGraphics, originX + 2, originY + 2, tipX + 2, tipY + 2, ARROW_SHADOW);
        drawLine(guiGraphics, tipX + 2, tipY + 2, headLeftX + 2, headLeftY + 2, ARROW_SHADOW);
        drawLine(guiGraphics, tipX + 2, tipY + 2, headRightX + 2, headRightY + 2, ARROW_SHADOW);
        drawThickLine(guiGraphics, originX, originY, tipX, tipY, sideColor);
        drawLine(guiGraphics, originX, originY - 1, tipX, tipY - 1, highlightColor);
        drawThickLine(guiGraphics, tipX, tipY, headLeftX, headLeftY, sideColor);
        drawThickLine(guiGraphics, tipX, tipY, headRightX, headRightY, sideColor);
        drawLine(guiGraphics, tipX, tipY - 1, headLeftX, headLeftY - 1, highlightColor);
    }

    private static void drawThickLine(GuiGraphics guiGraphics, int x0, int y0, int x1, int y1, int color) {
        drawLine(guiGraphics, x0, y0, x1, y1, color);
        drawLine(guiGraphics, x0 + 1, y0, x1 + 1, y1, color);
    }

    private static void drawLine(GuiGraphics guiGraphics, int x0, int y0, int x1, int y1, int color) {
        int deltaX = x1 - x0;
        int deltaY = y1 - y0;
        int steps = Math.max(Math.abs(deltaX), Math.abs(deltaY));
        if (steps == 0) {
            guiGraphics.fill(x0, y0, x0 + 1, y0 + 1, color);
            return;
        }
        for (int step = 0; step <= steps; ++step) {
            int x = x0 + (int) Math.round((double) deltaX * step / steps);
            int y = y0 + (int) Math.round((double) deltaY * step / steps);
            guiGraphics.fill(x, y, x + 1, y + 1, color);
        }
    }


    private static String cardinal16(double degrees) {
        int index = (int) Math.floor((normalizeDegrees(degrees) + 11.25) / 22.5);
        return CARDINALS[index & 15];
    }

    private static double normalizeDegrees(double degrees) {
        double normalized = degrees % 360.0;
        return normalized < 0.0 ? normalized + 360.0 : normalized;
    }

    private static void sendMessage(String text, ChatFormatting color) {
        Player player = Minecraft.getInstance().player;
        if (player != null) {
            player.displayClientMessage(Component.literal(text).withStyle(color), false);
        }
    }

    private record RelativeDirection(double right, double forward) {
        private static final RelativeDirection ZERO = new RelativeDirection(0.0, 0.0);
    }

    private record HorizontalReference(double yawDegrees, boolean aircraftRelative) {
    }

    private record WindReading(
        boolean available,
        Vec3d effectiveMph,
        double totalSpeedMph,
        double horizontalSpeedMph,
        double fromDegrees,
        boolean calm,
        String towardCardinal,
        String fromCardinal,
        boolean authoritativeServer,
        boolean pmweatherPresentAtSource
    ) {
        private static final WindReading UNAVAILABLE = new WindReading(
            false, Vec3d.ZERO, 0.0, 0.0,
            Double.NaN, true, "calm", "calm",
            false, false
        );
    }
}
