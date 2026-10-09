import java.lang.reflect.*;
import java.util.*;
import com.sun.management.ThreadMXBean;
import java.lang.management.ManagementFactory;

/** Exact cache equivalence, mutation isolation, bounded retention and warmed union costs. */
public final class CrashGeometryCacheRegression {
    static int checks;
    static void require(boolean value, String message) {
        checks++;
        if (!value) throw new AssertionError(message);
    }
    public static void main(String[] args) throws Exception {
        Class<?> owner = Class.forName("com.g9third.pmweatheriv.sable.SableCompoundCollider");
        Class<?> cell = Class.forName(owner.getName()+"$CellBox");
        Constructor<?> constructor = cell.getDeclaredConstructors()[0]; constructor.setAccessible(true);
        Method cached = owner.getDeclaredMethod("unionCellPieces", List.class); cached.setAccessible(true);
        Method direct = owner.getDeclaredMethod("unionCellPiecesUncached", List.class); direct.setAccessible(true);
        Field cacheField = owner.getDeclaredField("CELL_UNION_CACHE"); cacheField.setAccessible(true);
        Map<?,?> cache = (Map<?,?>) cacheField.get(null); cache.clear();
        Random random = new Random(48);
        List<List<Object>> fixtures = new ArrayList<>();
        for (int i=0; i<80; i++) {
            List<Object> boxes = new ArrayList<>();
            for (int j=0; j<2+i%4; j++) {
                double x=random.nextInt(8)/16.0, y=random.nextInt(8)/16.0, z=random.nextInt(8)/16.0;
                boxes.add(constructor.newInstance(x,y,z,x+.5,y+.5,z+.5));
            }
            fixtures.add(List.copyOf(boxes));
            Object expected = direct.invoke(null,boxes);
            Object first = cached.invoke(null,boxes);
            require(expected.equals(first),"cached overlapping union must match direct geometry");
            require(first == cached.invoke(null,new ArrayList<>(boxes)),"equal values reuse immutable union");
            Collections.reverse(boxes);
            require(direct.invoke(null,boxes).equals(cached.invoke(null,boxes)),"input order preserves direct result");
            boxes.set(0,constructor.newInstance(0.0,0.0,0.0,1.0,1.0,1.0));
            require(direct.invoke(null,boxes).equals(cached.invoke(null,boxes)),"changed geometry cannot reuse stale union");
            require(expected.equals(cached.invoke(null,fixtures.get(i))),"caller mutation cannot alter prior key");
        }
        for (int i=0; i<2100; i++) {
            double x=i/10000.0;
            List<Object> boxes=List.of(constructor.newInstance(x,0.0,0.0,.9,.9,.9),
                constructor.newInstance(.2,.2,.2,1.0,1.0,1.0));
            cached.invoke(null,boxes);
        }
        require(cache.size() <= 2048,"union retention must remain bounded");
        for (var boxes:fixtures)
            require(direct.invoke(null,boxes).equals(cached.invoke(null,boxes)),"evicted union rebuild remains exact");
        ThreadMXBean bean=(ThreadMXBean)ManagementFactory.getThreadMXBean();
        if (bean.isThreadAllocatedMemorySupported() && !bean.isThreadAllocatedMemoryEnabled())
            bean.setThreadAllocatedMemoryEnabled(true);
        for (int i=0;i<5;i++) for(var boxes:fixtures) { direct.invoke(null,boxes);cached.invoke(null,boxes); }
        long tid=Thread.currentThread().threadId();
        long bytes=bean.getThreadAllocatedBytes(tid), start=System.nanoTime();
        for (int i=0;i<10;i++) for(var boxes:fixtures) direct.invoke(null,boxes);
        long directTime=System.nanoTime()-start, directBytes=bean.getThreadAllocatedBytes(tid)-bytes;
        bytes=bean.getThreadAllocatedBytes(tid);start=System.nanoTime();
        for (int i=0;i<10;i++) for(var boxes:fixtures) cached.invoke(null,boxes);
        long cachedTime=System.nanoTime()-start, cachedBytes=bean.getThreadAllocatedBytes(tid)-bytes;
        System.out.printf(Locale.ROOT,"Crash geometry cache: PASS (%d assertions); 800 warmed unions direct=%.3fms/%d bytes cached=%.3fms/%d bytes%n",
            checks,directTime/1e6,directBytes,cachedTime/1e6,cachedBytes);
    }
}
