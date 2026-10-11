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
            .comment("Automatically show the shared live wind HUD only while in an IV aircraft pilot seat."
                + " /pmaero live on allows manual display elsewhere; /pmaero live off hides it"
                + " until the next pilot-seat entry.")
            .translation("config.pmweather_iv.display.auto_live_wind_in_pilot_seat")
            .define("autoLiveWindInPilotSeat", true);
        builder.pop();
        SPEC = builder.build();
    }

    private PMIVClientConfig() {}
}
