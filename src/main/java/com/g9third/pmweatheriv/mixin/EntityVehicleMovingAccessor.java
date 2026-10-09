package com.g9third.pmweatheriv.mixin;

import minecrafttransportsimulator.baseclasses.Point3D;
import minecrafttransportsimulator.baseclasses.RotationMatrix;
import minecrafttransportsimulator.mcinterface.IWrapperPlayer;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;
import org.spongepowered.asm.mixin.gen.Invoker;

/**
 * Reads the movement IV actually applied after wheel, ground, road, block, and
 * entity collision correction. These fields are the authoritative result of
 * IV's movement pass; {@code motion} and {@code rotation.angles} are only the
 * request that entered that pass.
 */
@Mixin(targets = "minecrafttransportsimulator.entities.instances.AEntityVehicleD_Moving", remap = false)
public interface EntityVehicleMovingAccessor {
    /** Native speed-dependent steering request; no pose or momentum is applied. */
    @Invoker("getTurningForce")
    double pmweatherIv$getTurningForce();

    @Accessor("motionApplied")
    Point3D pmweatherIv$getMotionApplied();

    /**
     * Positional wheel/ground depenetration added by IV after physical motion
     * is integrated. This term moves the pose but is not velocity or impulse.
     */
    @Accessor("groundMotion")
    Point3D pmweatherIv$getGroundMotion();

    @Accessor("rotationApplied")
    RotationMatrix pmweatherIv$getRotationApplied();

    @Accessor("blockBreakDelay")
    int pmweatherIv$getBlockBreakDelay();

    @Accessor("crashDebounce")
    int pmweatherIv$getCrashDebounce();

    @Accessor("crashDebounce")
    void pmweatherIv$setCrashDebounce(int value);

    @Accessor("serverDeltaM")
    Point3D pmweatherIv$getServerDeltaM();

    @Accessor("serverDeltaR")
    Point3D pmweatherIv$getServerDeltaR();

    @Accessor("clientDeltaM")
    Point3D pmweatherIv$getClientDeltaM();

    @Accessor("clientDeltaR")
    Point3D pmweatherIv$getClientDeltaR();

    /** Non-null only for a vehicle created from an active placement action. */
    @Accessor("placingPlayer")
    IWrapperPlayer pmweatherIv$getPlacingPlayer();
}

