package com.g9third.pmweatheriv.client;

import java.util.Objects;
import java.util.UUID;

/** Client-only live wind HUD policy: automatic pilot-seat display and explicit manual display. */
final class WindMonitorVisibility {
    private Object level;
    private UUID pilotSeat;
    private boolean autoPilotEnabled;
    private boolean manualEnabled;
    private boolean suppressedForSeat;

    /** The automatic display ends on dismount, and a fresh pilot-seat entry restores it. */
    boolean updatePilotSeat(Object currentLevel, UUID currentSeat, boolean autoEnabled) {
        if (level != currentLevel) {
            // Do not carry a manual HUD override or seat suppression into a different world.
            manualEnabled = false;
            suppressedForSeat = false;
            pilotSeat = null;
            level = currentLevel;
        }
        if (!Objects.equals(pilotSeat, currentSeat)) {
            suppressedForSeat = false;
        }
        pilotSeat = currentSeat;
        autoPilotEnabled = autoEnabled;
        return visible();
    }

    /** Manual /pmaero live on works anywhere; off also suppresses auto-display in this seat. */
    boolean setManual(boolean enabled) {
        manualEnabled = enabled;
        if (enabled) {
            suppressedForSeat = false;
        } else if (pilotSeat != null) {
            suppressedForSeat = true;
        }
        return visible();
    }

    boolean visible() {
        return manualEnabled || (autoPilotEnabled && pilotSeat != null && !suppressedForSeat);
    }

    void clear() {
        level = null;
        pilotSeat = null;
        autoPilotEnabled = false;
        manualEnabled = false;
        suppressedForSeat = false;
    }
}
