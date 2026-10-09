import java.lang.reflect.*;
import java.nio.*;
import java.util.*;
import com.g9third.pmweatheriv.physics.Vec3d;
import com.g9third.pmweatheriv.physics.ModelSurfaceMap.Bounds;
import minecrafttransportsimulator.rendering.RenderableVertices;

/** Runs against the current PMWeather-IV geometry classes using the real compiled classes. */
public class GeometryRegression {
    static final String P="com.g9third.pmweatheriv.physics.";
    static Class<?> type(String name)throws Exception{return Class.forName(P+"ModelGeometryData$"+name);}
    static Object make(Class<?> c,Object... args)throws Exception{
        for(var x:c.getDeclaredConstructors()) if(x.getParameterCount()==args.length){x.setAccessible(true);return x.newInstance(args);}
        throw new AssertionError(c);
    }
    static Method method(String owner,String name,int count)throws Exception {
        Class<?> c=Class.forName(P+owner);
        for(var m:c.getDeclaredMethods())if(m.getName().equals(name)&&m.getParameterCount()==count){m.setAccessible(true);return m;}
        throw new AssertionError(name);
    }
    static Object mesh(RenderableVertices v)throws Exception{
        return make(Class.forName("com.g9third.pmweatheriv.model.ParsedModelSnapshot$Mesh"),v);
    }
    public static void main(String[] args)throws Exception {
        Random random=new Random(1111);
        Method classify=method("SurfaceClassifier","classify",4);
        String[] names={"mesh","wing","tail","stab","rudder","elevator","aileron","elevon","spoiler","taileron","cockpit","gear","canard","float","fuselage","glass"};
        double[] edges={-.721,-.72,-.719,-.581,-.58,-.579,-.561,-.56,-.559,.049,.05,.051,.339,.34,.341,.579,.58,.581};
        for(int i=0;i<16000;i++) {
            var bounds=new Bounds(new Vec3d(-6,-3,-8),new Vec3d(6,3,8),true);
            String name=names[i%names.length];
            Vec3d point=new Vec3d(edges[i%edges.length]*6,edges[(i/3)%edges.length]*3,edges[(i/7)%edges.length]*8);
            Vec3d normal=new Vec3d(random.nextDouble()*2-1,random.nextDouble()*2-1,random.nextDouble()*2-1).normalized();
            Object triangle=make(type("RawTriangle"),point,point,point,point,normal,1.0,name,bounds);
            int bits=i%64;
            Object hint=make(type("AnimationHint"),(bits&1)!=0,(bits&2)!=0,(bits&4)!=0,(bits&8)!=0,(bits&16)!=0,(bits&32)!=0);
            System.out.println("classify "+classify.invoke(null,triangle,bounds,Map.of(name,hint),i%5==0));
        }
        Method scan=method("ModelTriangleSampling","scanObject",1), sample=method("ModelTriangleSampling","readSampledTriangles",3);
        for(int run=0;run<12;run++) {
            FloatBuffer buffer=FloatBuffer.allocate(24*200+7);
            for(int triangle=0;triangle<200;triangle++)for(int v=0;v<3;v++){
                for(int j=0;j<5;j++)buffer.put(999);
                buffer.put((float)(random.nextDouble()*10)).put((float)(random.nextDouble()*2)).put((float)(random.nextDouble()*20));
            }
            buffer.position(7); // readers must rewind duplicates without touching this cursor.
            RenderableVertices raw=new RenderableVertices("wing",buffer,true);Object geometry=mesh(raw);
            Object stats=scan.invoke(null,geometry);
            System.out.println("scan "+stats);
            Object object=make(type("ModelObject"),geometry,"wing","wing",stats,2.0);
            List<Object> triangles=new ArrayList<>();sample.invoke(null,object,17+run,triangles);
            System.out.println("sample "+triangles);
            if(buffer.position()!=7)throw new AssertionError("source cursor changed");
            Class<?> hull=Class.forName("com.g9third.pmweatheriv.sable.SableModelCollisionHull");
            Method read=null;
            for(var candidate:hull.getDeclaredMethods())if(candidate.getName().equals("readTriangles")){read=candidate;read.setAccessible(true);}
            Class<?> hullBounds=Class.forName(hull.getName()+"$Bounds");
            for(double scale:new double[]{1,-1,.001}){
                List<Object> hullTriangles=new ArrayList<>();
                read.invoke(null,geometry,scale,scale,scale,hullTriangles,make(hullBounds));
                System.out.println("hull "+hullTriangles);
            }
            var positions=geometry.getClass().getMethod("positions");
            FloatBuffer first=(FloatBuffer)positions.invoke(geometry),second=(FloatBuffer)positions.invoke(geometry);
            float original=first.get(0);buffer.put(5,12345);first.get();
            if(second.position()!=0 || second.get(0)!=original)throw new AssertionError("shared mutable snapshot");
            try{second.put(0,1);throw new AssertionError("writable snapshot");}catch(ReadOnlyBufferException expected){}
        }
    }
}
