package com.g9third.pmweatheriv;

import net.neoforged.neoforge.common.ModConfigSpec;

/** Local display preferences; never changes server vehicle physics. */
public final class PMIVClientConfig {
    public static final ModConfigSpec SPEC;
    public static final ModConfigSpec.BooleanValue AUTO_LIVE_WIND_IN_PILOT_SEAT;

    static {
        ModConfigSpec.Builder builder = new ModConfigSpec.Builder();
        builder.comment("Display settings").translation("config.pmweather_iv.category.display").push("display");
        AUTO_LIVE_WIND_IN_PILOT_SEAT = builder
            .comment("Automatically enable the shared live wind HUD when entering an IV aircraft pilot seat."
                + " /pmaero live off keeps it off until you enter a pilot seat again.")
            .translation("config.pmweather_iv.display.auto_live_wind_in_pilot_seat")
            .define("autoLiveWindInPilotSeat", true);
        builder.pop();
        SPEC = builder.build();
    }

    private PMIVClientConfig() {}
}
