/*
 * Portions derived from True Impact 0.5.8-delta by OMEGAU371 / True Impact contributors.
 * Original project: https://github.com/OMEGAU371/sable-true-impact
 * Original license: LGPL-3.0-only.
 * PMWeather-IV modification notice: adapted for PMIV; notice updated 2026-10-09.
 * Adapted for PMWeather-IV's integrated aircraft/world material-response subsystem.
 */
package com.g9third.pmweatheriv.terrain.trueimpact.damage;

/** Defaults for PMIV's integrated aircraft/world material solver.
 * These fields are local to PMIV, not synced from standalone True Impact's config.
 * This subsystem handles world terrain; structure and Create systems are separate.
 */
public final class ImpactRuntimeConfig {
    private ImpactRuntimeConfig() {}

    // Mutation/compaction switches are independent. Compaction rules use joules.
    public static volatile boolean APPLY_BLOCK_EFFECTS = true;
    public static volatile boolean ENABLE_COMPACTION = true;
    public static volatile java.util.List<CompactionRule> COMPACTION_RULES = java.util.List.of(
            new CompactionRule("minecraft:grass_block",       "minecraft:dirt",   5.0, 1.0),
            new CompactionRule("minecraft:farmland",          "minecraft:dirt",   5.0, 1.0),
            new CompactionRule("minecraft:podzol",             "minecraft:dirt",   5.0, 1.0),
            new CompactionRule("minecraft:mycelium",           "minecraft:dirt",   5.0, 1.0),
            new CompactionRule("minecraft:suspicious_sand",    "minecraft:sand",   5.0, 1.0),
            new CompactionRule("minecraft:suspicious_gravel",  "minecraft:gravel", 5.0, 1.0)
    );

    // Damage feedback, fatigue accumulation, fracture, and summary logging.
    public static volatile boolean ENABLE_DAMAGE_FEEDBACK = true;
    public static volatile boolean ENABLE_DAMAGE_ACCUMULATION = true;
    public static volatile boolean ENABLE_BLOCK_BREAKING = true;
    public static volatile boolean LOG_ENERGY_SUMMARY   = true;

    // Fatigue: ignore hits below this fraction; decay accumulated damage per half-life.
    public static volatile double WORLD_ELASTIC_FLOOR = 0.2;
    public static volatile int WORLD_DAMAGE_HALF_LIFE_TICKS = 60;

    // Penetration thresholds (J, m/s), energy loss factor, and footprint radius (blocks).
    public static volatile boolean ENABLE_PENETRATION = true;
    public static volatile double PENETRATION_TRIGGER_J = 1200.0;
    public static volatile double PENETRATION_MIN_SPEED_MS = 8.0;
    public static volatile double PENETRATION_LOSS_FACTOR = 2.0;
    public static volatile int PENETRATION_FOOTPRINT_RADIUS = 8;

    // Material hardness/confinement calibration and cosmetic feedback limits.
    public static volatile double CRACK_MIN      =   3.0;
    public static volatile double CRACK_MAX      = 500.0;
    public static volatile double CRACK_COEFF    =  15.0;
    public static volatile double CRACK_EXPONENT =   0.6;
    public static volatile double BREAK_BASE     =   5.0;
    public static volatile double BREAK_COEFF    =   3.0;
    public static volatile double BREAK_EXPONENT =   0.4;
    public static volatile double CONFINEMENT_PER_FACE_CAP = 3.0;
    public static volatile double CONFINEMENT_DIRECTION_BASE = 0.5;
    public static volatile double CONFINEMENT_DIRECTION_AMPLITUDE = 0.5;
    public static volatile double OVERBURDEN_EFFICIENCY = 0.15;
    public static volatile int FEEDBACK_COOLDOWN_TICKS = 10;
    public static volatile int FEEDBACK_BUDGET_PER_TICK = 16;
}
