import com.g9third.pmweatheriv.terrain.trueimpact.SweptContactVolume;
import java.util.Random;

/** Production swept-volume checks against analytic volumes and an independent midpoint integral. */
public final class SweptContactVolumeRegression {
    private static int assertions;
    private static void near(double actual, double expected, double tolerance, String message) {
        ++assertions;
        if (!Double.isFinite(actual) || Math.abs(actual - expected) > tolerance)
            throw new AssertionError(message + ": " + actual + " vs " + expected);
    }
    private static double volume(double x, double y, double z, double dx, double dy, double dz,
            double travel, double area) {
        return SweptContactVolume.intersection(0, 0, 0, 1, 1, 1,
            x, y, z, dx, dy, dz, travel, area);
    }
    private static double overlap(double center, double half) {
        return Math.max(0.0, Math.min(1.0, center + half) - Math.max(0.0, center - half));
    }
    private static double numerical(double[] start, double[] dir, double travel, double area) {
        double norm = Math.sqrt(dir[0]*dir[0]+dir[1]*dir[1]+dir[2]*dir[2]);
        double[] d = {dir[0]/norm, dir[1]/norm, dir[2]/norm};
        int axis = Math.abs(d[1]) >= Math.abs(d[0]) && Math.abs(d[1]) >= Math.abs(d[2])
            ? 1 : Math.abs(d[0]) >= Math.abs(d[2]) ? 0 : 2;
        int a=(axis+1)%3, b=(axis+2)%3;
        double half = 0.5*Math.sqrt(area/Math.abs(d[axis]));
        double t0=Math.max(0.0, Math.min(-start[axis]/d[axis],(1-start[axis])/d[axis]));
        double t1=Math.min(travel, Math.max(-start[axis]/d[axis],(1-start[axis])/d[axis]));
        if (t1<=t0) return 0.0;
        double dt=(t1-t0)/10000.0, sum=0.0;
        for (int i=0;i<10000;++i) {
            double t=t0+(i+0.5)*dt;
            sum+=overlap(start[a]+d[a]*t,half)*overlap(start[b]+d[b]*t,half);
        }
        return sum*dt*Math.abs(d[axis]);
    }
    public static void main(String[] args) {
        near(volume(.5,1,.5,0,-1,0,.77,.125), .125*.77, 1e-12,
            "small wheel displaces only its swept soil column");
        near(volume(.5,1,.5,0,-1,0,.03,.23), .23*.03, 1e-12,
            "short body sweep does not charge a full voxel");
        near(volume(.5,1,.5,0,-1,0,2,1),1,1e-12,"full-voxel sweep retains full solid mass");
        near(volume(.5,1,.5,0,-1,0,.77,.125)*1600,154,1e-9,
            "recorded wheel geometry engages 154 kg rather than 1600 kg");
        near(volume(0,1,.5,0,-1,0,.5,.25),.0625,1e-12,"cell edge clips contact area");
        double first=volume(0,1,.5,0,-1,0,.5,.25);
        double second=volume(1,1,.5,0,-1,0,.5,.25);
        near(first+second,.125,1e-12,"crossing cell edge neither doubles nor loses mass");
        near(SweptContactVolume.intersection(0,.5,0,1,1,1,.5,1,.5,0,-1,0,1,.25),
            .125,1e-12,"thin collision slab contributes only its solid depth");
        near(volume(.5,1,.5,0,-1,0,0,.25),0,0,"zero travel engages no material");
        near(volume(.5,1,.5,0,0,0,1,.25),0,0,"zero direction is rejected");
        near(volume(.5,1,.5,0,-1,0,Double.NaN,.25),0,0,"invalid travel is rejected");
        near(volume(.5,1,.5,0,-1,0,1,Double.NaN),0,0,"invalid area is rejected");
        near(volume(3,1,.5,0,-1,0,1,.25),0,0,"unstruck shape contributes no mass");
        Random random=new Random(714122);
        for (int i=0;i<300;++i) {
            double[] start={random.nextDouble()*2-.5,random.nextDouble()*2-.5,random.nextDouble()*2-.5};
            double[] dir={random.nextDouble()*2-1,random.nextDouble()*2-1,random.nextDouble()*2-1};
            double travel=random.nextDouble()*2+.01,area=random.nextDouble()*2+.01;
            double actual=volume(start[0],start[1],start[2],dir[0],dir[1],dir[2],travel,area);
            near(actual,numerical(start,dir,travel,area),1e-7,"oblique intersection matches independent quadrature");
            near(SweptContactVolume.intersection(-5,-5,-5,5,5,5,0,0,0,
                dir[0],dir[1],dir[2],travel,area),area*travel,1e-12,
                "normal area times travel is preserved in any direction");
            double partition=0;
            for(int x=0;x<2;++x) for(int y=0;y<2;++y) for(int z=0;z<2;++z) {
                partition+=SweptContactVolume.intersection(x*.5,y*.5,z*.5,(x+1)*.5,(y+1)*.5,(z+1)*.5,
                    start[0],start[1],start[2],dir[0],dir[1],dir[2],travel,area);
            }
            near(partition,actual,1e-12,"disjoint shape boxes conserve swept volume");
        }
        System.out.println("SweptContactVolumeRegression: "+assertions+" assertions passed");
    }
}
