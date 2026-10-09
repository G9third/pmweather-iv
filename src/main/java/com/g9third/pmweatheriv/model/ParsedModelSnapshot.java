package com.g9third.pmweatheriv.model;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import minecrafttransportsimulator.rendering.AModelParser;
import minecrafttransportsimulator.rendering.RenderableVertices;

/** Immutable, unscaled triangle positions shared by aero and collision preparation.
 * Object order, null entries, names and transparency are preserved. Each consumer
 * owns its filters and area thresholds. No mutable render buffers escape here.
 */
public final class ParsedModelSnapshot {
    private static final int MAX_CACHE_ENTRIES = 128;
    private static final Map<String, ParsedModelSnapshot> CACHE = new LinkedHashMap<>(32, 0.75f, true) {
        @Override protected boolean removeEldestEntry(Map.Entry<String, ParsedModelSnapshot> eldest) {
            return size() > MAX_CACHE_ENTRIES;
        }
    };
    private final List<Mesh> objects;

    private ParsedModelSnapshot(List<RenderableVertices> source) {
        List<Mesh> copied = new ArrayList<>();
        if (source != null) {
            for (RenderableVertices object : source) {
                copied.add(object == null ? null : new Mesh(object));
            }
        }
        objects = Collections.unmodifiableList(copied);
    }

    public static synchronized ParsedModelSnapshot load(String location) {
        ParsedModelSnapshot cached=CACHE.get(location);
        if (cached!=null) return cached;
        // Use IV's parser without its mutable render cache. PMIV caches only
        // useful immutable snapshots, so an early empty model can recover.
        ParsedModelSnapshot built=new ParsedModelSnapshot(AModelParser.parseModel(location,false));
        if (built.objects.stream().anyMatch(m -> m!=null && !m.isLines && m.positions.remaining()>=9))
            CACHE.put(location,built);
        return built;
    }

    public List<Mesh> objects() { return objects; }

    public static final class Mesh {
        public final String name;
        public final boolean isLines;
        public final boolean isTranslucent;
        private final FloatBuffer positions;

        private Mesh(RenderableVertices source) {
            name = source.name;
            isLines = source.isLines;
            isTranslucent = source.isTranslucent;
            positions = source.vertices == null ? FloatBuffer.allocate(0).asReadOnlyBuffer()
                : decodePositions(source.vertices, isLines);
        }

        private Mesh(Mesh source, double sx, double sy, double sz) {
            name = source.name;
            isLines = source.isLines;
            isTranslucent = source.isTranslucent;
            FloatBuffer input = source.positions();
            FloatBuffer output = FloatBuffer.allocate(input.remaining());
            while (input.remaining() >= 3) {
                output.put((float)(input.get()*sx));
                output.put((float)(input.get()*sy));
                output.put((float)(input.get()*sz));
            }
            output.flip();
            positions = output.asReadOnlyBuffer();
        }

        public Mesh scaled(double sx, double sy, double sz) {
            if (!Double.isFinite(sx) || !Double.isFinite(sy) || !Double.isFinite(sz)
                || sx == 0 || sy == 0 || sz == 0) throw new IllegalArgumentException("Invalid model scale");
            return sx == 1 && sy == 1 && sz == 1 ? this : new Mesh(this,sx,sy,sz);
        }

        /** Independent read-only cursor; x/y/z triples, three vertices per triangle. */
        public FloatBuffer positions() { return positions.asReadOnlyBuffer(); }
    }

    static FloatBuffer decodePositions(FloatBuffer source, boolean lines) {
        FloatBuffer input = source.duplicate();
        input.rewind();
        int triangles = lines ? 0 : input.remaining() / 24;
        FloatBuffer output = FloatBuffer.allocate(triangles * 9);
        for (int vertex = 0; vertex < triangles * 3; ++vertex) {
            input.position(input.position() + 5); // IV normal xyz, UV, position xyz.
            output.put(input.get()).put(input.get()).put(input.get());
        }
        output.flip();
        return output.asReadOnlyBuffer();
    }
}
