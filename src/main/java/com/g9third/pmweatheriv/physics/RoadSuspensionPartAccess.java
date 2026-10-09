package com.g9third.pmweatheriv.physics;

/** Runtime view of the native part pose before/after PMIV's visual wheel travel. */
public interface RoadSuspensionPartAccess {
    double pmweatherIv$appliedSuspensionOffset();

    boolean pmweatherIv$hasAuthoredVerticalMotion();
}
