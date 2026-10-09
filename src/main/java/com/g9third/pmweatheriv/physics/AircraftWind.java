package com.g9third.pmweatheriv.physics;

import com.g9third.pmweatheriv.compat.PMAeroBridge;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import minecrafttransportsimulator.entities.instances.EntityVehicleF_Physics;
import net.minecraft.server.level.ServerLevel;
import static com.g9third.pmweatheriv.physics.FlightMath.MPH_TO_METERS_PER_SECOND;
import static com.g9third.pmweatheriv.physics.FlightMath.activePositionWorld;
import static com.g9third.pmweatheriv.physics.FlightMath.point;
import static com.g9third.pmweatheriv.physics.FlightMath.toWorld;

/** Owner-tick sampling at semantic force stations. Substeps only read the immutable result. */
public final class AircraftWind {
    private AircraftWind() {}

    public record WindSample(Vec3d worldPosition, Vec3d rawMph, Vec3d windMetersPerSecond) {}

    public record WindFieldSnapshot(Map<String, WindSample> stations, List<WindSample> points,
                                    Vec3d meanRawMph) {
        public WindFieldSnapshot {
            stations = Map.copyOf(stations);
            points = List.copyOf(points);
        }
    }

    static final class WindField {
        private final EntityVehicleF_Physics vehicle;
        private final Map<String, Vec3d> requested;
        private WindFieldSnapshot snapshot;

        WindField(EntityVehicleF_Physics vehicle) { this.vehicle = vehicle; this.requested = new LinkedHashMap<>(); }
        WindField(EntityVehicleF_Physics vehicle, WindFieldSnapshot snapshot) {
            this.vehicle = vehicle;
            this.snapshot = java.util.Objects.requireNonNull(snapshot);
            this.requested = null;
        }
        boolean collecting() { return snapshot == null; }

        void register(String id, Vec3d localPoint) {
            if (!collecting()) throw new IllegalStateException("Wind batch already resolved");
            if (id == null || localPoint == null || !localPoint.isFinite()) {
                throw new IllegalArgumentException("Invalid aerodynamic wind station");
            }
            Vec3d previous = requested.putIfAbsent(id, localPoint);
            if (previous != null && !previous.equals(localPoint)) {
                throw new IllegalStateException("Duplicate aerodynamic station: " + id);
            }
        }

        void resolve(ServerLevel level, AircraftState state) {
            if (!collecting()) throw new IllegalStateException("Wind sampled twice");
            Map<Vec3d, Integer> unique = new LinkedHashMap<>();
            for (Vec3d point : requested.values()) unique.computeIfAbsent(point, ignored -> unique.size());
            int size = unique.size();
            if (state.windInputBuffer.length != size * 3) state.windInputBuffer = new double[size * 3];
            if (state.windOutputBuffer.length != size * PMAeroBridge.WIND_STRIDE) {
                state.windOutputBuffer = new double[size * PMAeroBridge.WIND_STRIDE];
            }
            List<Vec3d> worldPoints = new ArrayList<>(size);
            for (Vec3d local : unique.keySet()) {
                Vec3d world = ModelCoordinates.worldPoint(activePositionWorld(vehicle),
                    FlightMath.activeOrientation(vehicle),local);
                int i = worldPoints.size() * 3;
                state.windInputBuffer[i] = world.x();
                state.windInputBuffer[i + 1] = world.y();
                state.windInputBuffer[i + 2] = world.z();
                worldPoints.add(world);
            }
            PMAeroBridge.sampleAircraftAtmosphereInto(level, state.windInputBuffer, state.windOutputBuffer);
            List<WindSample> points = new ArrayList<>(size);
            Vec3d mean = Vec3d.ZERO;
            for (int i = 0; i < size; i++) {
                int offset = i * PMAeroBridge.WIND_STRIDE;
                Vec3d raw = new Vec3d(state.windOutputBuffer[offset], state.windOutputBuffer[offset + 1],
                    state.windOutputBuffer[offset + 2]);
                points.add(new WindSample(worldPoints.get(i), raw, raw.scale(MPH_TO_METERS_PER_SECOND)));
                mean = mean.add(raw);
            }
            Map<String, WindSample> stations = new LinkedHashMap<>();
            for (Map.Entry<String, Vec3d> request : requested.entrySet()) {
                stations.put(request.getKey(), points.get(unique.get(request.getValue())));
            }
            snapshot = new WindFieldSnapshot(stations, points, size == 0 ? Vec3d.ZERO : mean.scale(1.0 / size));
            state.windSnapshot = snapshot;
            state.windGameTime = level.getGameTime();
        }

        WindSample sample(Vec3d localPoint, String id) {
            if (collecting()) {
                register(id, localPoint);
                return new WindSample(Vec3d.ZERO, Vec3d.ZERO, Vec3d.ZERO);
            }
            WindSample sample = snapshot.stations().get(id);
            if (sample == null) throw new IllegalStateException("Unplanned aerodynamic station: " + id);
            return sample;
        }

        /** Diagnostic/IV airspeed reference: mean of force-point samples, with no centre query. */
        WindSample initializeCenter() {
            Vec3d mean = snapshot.meanRawMph();
            return new WindSample(activePositionWorld(vehicle), mean, mean.scale(MPH_TO_METERS_PER_SECOND));
        }
        WindFieldSnapshot snapshot() { return snapshot; }
    }
}
