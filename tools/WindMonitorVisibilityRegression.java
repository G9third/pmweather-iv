package com.g9third.pmweatheriv.client;

import java.util.UUID;

/** Pure-Java behavior checks for automatic pilot HUD and manual command overrides. */
public final class WindMonitorVisibilityRegression {
    private static int assertions;

    private static void check(boolean pass, String message) {
        assertions++;
        if (!pass) throw new AssertionError(message);
    }

    public static void main(String[] args) {
        WindMonitorVisibility state = new WindMonitorVisibility();
        Object overworld = new Object();
        Object nether = new Object();
        UUID pilotA = new UUID(123L, 1L);
        UUID pilotB = new UUID(123L, 2L);

        check(!state.visible(), "hidden by default");
        check(!state.updatePilotSeat(overworld, null, true), "walking never automatically enables the HUD");
        check(state.updatePilotSeat(overworld, pilotA, true), "pilot-seat entry automatically displays wind");
        check(state.updatePilotSeat(overworld, pilotA, true), "remaining in the same pilot seat keeps it visible");
        check(!state.updatePilotSeat(overworld, null, true), "dismount immediately hides automatic HUD");
        check(!state.updatePilotSeat(overworld, null, true), "staying outside the cockpit stays hidden");

        check(state.setManual(true), "manual on works while walking");
        check(state.updatePilotSeat(overworld, null, true), "manual on persists without a pilot seat");
        check(!state.setManual(false), "manual off hides the HUD while walking");
        check(state.updatePilotSeat(overworld, pilotA, true), "pilot-seat entry restores automatic display");
        check(!state.setManual(false), "manual off hides automatic HUD in the current pilot seat");
        check(!state.updatePilotSeat(overworld, pilotA, true), "same seat does not undo manual off");
        check(!state.updatePilotSeat(overworld, null, true), "dismount stays hidden after manual off");
        check(state.updatePilotSeat(overworld, pilotA, true), "re-entering the seat re-enables automatic display");
        check(!state.setManual(false), "manual off suppresses auto for pilot A");
        check(state.updatePilotSeat(overworld, pilotB, true), "switching to another pilot seat restores auto display");

        check(!state.updatePilotSeat(overworld, pilotA, false), "client config can disable automatic display");
        check(state.setManual(true), "manual command still works with auto display disabled");
        check(state.updatePilotSeat(overworld, null, false), "manual display persists after dismount");
        check(!state.setManual(false), "manual off remains effective with auto disabled");

        check(state.updatePilotSeat(overworld, pilotA, true), "automatic display resumes with auto config enabled");
        check(state.setManual(true), "manual on while seated is explicit");
        check(state.updatePilotSeat(overworld, null, true), "explicit manual on survives leaving pilot seat");
        check(!state.updatePilotSeat(nether, null, true), "manual display does not leak into another world");
        check(state.updatePilotSeat(nether, pilotA, true), "new-world pilot seat still enables automatic display");
        state.clear();
        check(!state.visible(), "disconnect clears automatic and manual display");
        check(!state.updatePilotSeat(nether, null, true), "reconnection while walking remains hidden");

        System.out.println("WindMonitorVisibilityRegression PASS (" + assertions + " checks)");
    }
}
