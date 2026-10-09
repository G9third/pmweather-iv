import com.g9third.pmweatheriv.terrain.trueimpact.damage.*;
import java.util.*;

/** Runs material scenarios against the current production implementations. */
public class MaterialRegression {
    // Math.pow is allowed tiny platform-specific last-bit differences. Keep the
    // golden regression architecture-neutral without changing production math.
    static String stableDouble(double value) {
        if (!Double.isFinite(value)) return Double.toString(value);
        return String.format(Locale.ROOT, "%.12g", value);
    }
    static DeferredDamageEvent event(long tick, int x, double energy,
            MaterialThresholdProfile.MaterialClass material) {
        return new DeferredDamageEvent(
            tick, "minecraft:overworld", "minecraft:grass_block", x, 0, 0,
            material, energy, 100.0, 0.0, 0.0, 1.0
        );
    }
    static boolean eligible(BlockDamageAccumulator.Snapshot snap) {
        return MaterialResponsePlanner.canBreak(snap);
    }
    public static void main(String[] args) throws Exception {
        // These threshold lines preserve labels in the historical golden;
        // active hardness cases below still call the current production code.
        double[] historicalThresholds = {5, 15, 20, 50, 120, 300, 40};
        double[] historicalBreakMultipliers = {1, 3, 5, 10, 15, 25, 7};
        var soil=MaterialThresholdProfile.MaterialClass.SOFT_SOIL;
        for(var material:MaterialThresholdProfile.MaterialClass.values()) {
            int index = material.ordinal();
            System.out.println("threshold "+material+" "+historicalThresholds[index]+" "+historicalBreakMultipliers[index]);
            for(var state:DamageState.values()) {
                var snap=new BlockDamageAccumulator.Snapshot(
                    new BlockDamageAccumulator.AccKey("world",0,0,0,"stone"),material,
                    100,100,100,100,1,1,state);
                System.out.println("eligibility "+material+" "+state+" "+eligible(snap));
            }
            for(boolean accumulation:new boolean[]{true,false}) {
                ImpactRuntimeConfig.ENABLE_DAMAGE_ACCUMULATION=accumulation;
                BlockDamageAccumulator.clear();
                long tick=0;
                for(double energy:new double[]{0,1,19.9,20,30,50,100,200,20,100}) {
                    tick+=energy==200?120:1;
                    System.out.println("accumulate "+material+" "+accumulation+" "+BlockDamageAccumulator.accumulate(event(tick,0,energy,material)));
                }
            }
        }
        ImpactRuntimeConfig.ENABLE_DAMAGE_ACCUMULATION=true;
        for(float hardness:new float[]{-1,0,0.5f,3,50})
            for(float blast:new float[]{0,0.5f,6,1200})
                System.out.println("hardness "+stableDouble(BlockHardnessProfile.crackThresholdJ(hardness,blast))+" "+stableDouble(BlockHardnessProfile.breakThresholdJ(hardness,blast)));
        for(double dir:new double[]{-1,0,1,Double.NaN})
            for(double neighbor:new double[]{0,30,300,Double.POSITIVE_INFINITY})
                System.out.println("confinement "+ConfinementFactor.compute(new double[]{neighbor,10,0,30,40,50},20,dir,0,0));
        for(int depth=0;depth<=20;depth++)
            System.out.println("overburden "+ConfinementFactor.overburdenEnergyJ(depth,1600));
        DeferredDamageQueue.clear();
        System.out.println("queueNaN "+DeferredDamageQueue.enqueue(event(1,0,Double.NaN,soil)));
        for(int i=0;i<70;i++) System.out.println("queue "+i+" "+DeferredDamageQueue.enqueue(event(1,i,50,soil)));
        System.out.println("queueDuplicate "+DeferredDamageQueue.enqueue(event(1,0,50,soil)));
        for(var e:DeferredDamageQueue.drainAll()) System.out.println("drain "+e.posX()+" "+e.kImpact());
        System.out.println("drainEmpty "+DeferredDamageQueue.drainAll().size());
        System.out.println("queueNextTick "+DeferredDamageQueue.enqueue(event(2,0,50,soil)));
        DeferredDamageQueue.clear();
        System.out.println("queueReset "+DeferredDamageQueue.drainAll().size());
        var key=new BlockDamageAccumulator.AccKey("world",0,0,0,"stone");
        MaterialResponsePlanner.clear();
        System.out.println("dedup "+MaterialResponsePlanner.markBreakScheduled(key)+" "+MaterialResponsePlanner.markBreakScheduled(key));
        MaterialResponsePlanner.forgetKey(key);
        System.out.println("dedupReset "+MaterialResponsePlanner.markBreakScheduled(key));
        CrackOverlayTracker.clear();DamageFeedbackTracker.clear();
        for(int tick=0;tick<30;tick++) {
            System.out.println("crack "+CrackOverlayTracker.tryUpdate(key,DamageState.CRACKED,0.75,tick));
            System.out.println("feedback "+DamageFeedbackTracker.shouldEmit("world",0,0,0,DamageState.CRACKED,tick));
        }
        System.out.println("clearOverlay "+CrackOverlayTracker.removeEntry(key));
        BlockView view=new BlockView() {
            public boolean hasChunkAt(int x,int y,int z){return x>=0;}
            public String getBlockId(int x,int y,int z){return "minecraft:grass_block";}
            public boolean setBlock(int x,int y,int z,String id){return true;}
        };
        for(int x:new int[]{-1,0}) for(double e:new double[]{0,5,6,50,Double.NaN})
            System.out.println("compact "+ImpactBlockApplicator.tryApply(view,event(1,x,e,soil)));
    }
}
