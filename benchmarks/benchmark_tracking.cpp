#include "aegistrack/association/hungarian_associator.hpp"
#include "aegistrack/association/nearest_neighbor_associator.hpp"
#include "aegistrack/config/scenario_config.hpp"
#include "aegistrack/core/random.hpp"
#include "aegistrack/filtering/extended_kalman_filter.hpp"
#include "aegistrack/simulation/simulation_engine.hpp"
#include "aegistrack/tracking/track_manager.hpp"

#include <benchmark/benchmark.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace aegistrack::benchmarks {

static void BM_KalmanPredict(benchmark::State& state) {
    Vector4d x0{100.0, 200.0, 15.0, -10.0};
    Matrix4d P0 = Matrix4d::Identity() * 25.0;
    ExtendedKalmanFilter ekf(x0, P0, 1.5);

    for (auto _ : state) {
        ekf.predict(0.1);
        Vector4d st = ekf.state();
        benchmark::DoNotOptimize(st);
    }
    state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_KalmanPredict);

static void BM_EkfRadarUpdate(benchmark::State& state) {
    Vector4d x0{400.0, 300.0, 15.0, -10.0};
    Matrix4d P0 = Matrix4d::Identity() * 25.0;
    ExtendedKalmanFilter ekf(x0, P0, 1.5);

    Measurement m;
    m.measurement_id = 1;
    m.sensor_id = "radar_1";
    m.sensor_type = SensorType::Radar;
    m.capture_timestamp = 0.1;
    m.arrival_timestamp = 0.1;
    m.z = Vector2d{500.0, 0.6435};
    m.R = Matrix2d::Identity();
    m.R(0, 0) = 36.0;
    m.R(1, 1) = 0.0001;
    m.sensor_position = Vector2d::Zero();
    m.is_valid = true;

    for (auto _ : state) {
        ekf.predict(0.01);
        auto diag = ekf.updateMeasurement(m);
        benchmark::DoNotOptimize(diag);
    }
    state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_EkfRadarUpdate);

static void BM_HungarianAssociation(benchmark::State& state) {
    const auto n_tracks = static_cast<std::size_t>(state.range(0));
    DeterministicRng rng(42);

    std::vector<Track> tracks;
    std::vector<Measurement> measurements;
    tracks.reserve(n_tracks);
    measurements.reserve(n_tracks + 5);

    for (std::size_t i = 0; i < n_tracks; ++i) {
        const double px = static_cast<double>(i % 10) * 60.0 + rng.normal(0.0, 5.0);
        const double py = static_cast<double>(i / 10) * 60.0 + rng.normal(0.0, 5.0);
        Vector4d x0{px, py, 10.0, 5.0};
        Matrix4d P0 = Matrix4d::Identity() * 25.0;
        tracks.emplace_back(
            i + 1, 0.0, std::make_unique<ExtendedKalmanFilter>(x0, P0, 1.5), "gnss");

        Measurement m;
        m.measurement_id = i + 1;
        m.sensor_id = "gnss";
        m.sensor_type = SensorType::Gnss;
        m.capture_timestamp = 0.1;
        m.arrival_timestamp = 0.1;
        m.z = Vector2d{px + rng.normal(0.0, 3.0), py + rng.normal(0.0, 3.0)};
        m.R = Matrix2d::Identity() * 9.0;
        m.is_valid = true;
        measurements.push_back(std::move(m));
    }

    HungarianAssociator associator;
    for (auto _ : state) {
        auto res = associator.associate(tracks, measurements, 9.21034);
        benchmark::DoNotOptimize(res);
    }
    state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(n_tracks));
}
BENCHMARK(BM_HungarianAssociation)->Arg(1)->Arg(10)->Arg(50)->Arg(100);

static void BM_TrackManagerUpdate(benchmark::State& state) {
    const auto n_tracks = static_cast<std::size_t>(state.range(0));
    DeterministicRng rng(101);

    TrackManager tm;
    std::vector<Measurement> base_meas;
    base_meas.reserve(n_tracks + 4);

    for (std::size_t i = 0; i < n_tracks; ++i) {
        Measurement m;
        m.measurement_id = i + 1;
        m.sensor_id = "gnss";
        m.sensor_type = SensorType::Gnss;
        m.capture_timestamp = 0.0;
        m.arrival_timestamp = 0.0;
        m.z = Vector2d{static_cast<double>(i % 10) * 120.0,
                       static_cast<double>(i / 10) * 120.0};
        m.R = Matrix2d::Identity() * 16.0;
        m.is_valid = true;
        base_meas.push_back(m);
    }
    tm.processMeasurements(base_meas, 0.0);

    TimestampSec t = 0.0;
    for (auto _ : state) {
        t += 0.1;
        for (std::size_t i = 0; i < n_tracks; ++i) {
            base_meas[i].capture_timestamp = t;
            base_meas[i].arrival_timestamp = t;
            base_meas[i].z(0) += 2.0;
            base_meas[i].z(1) += 1.0;
        }
        auto summary = tm.processMeasurements(base_meas, t);
        benchmark::DoNotOptimize(summary);
    }
    state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(n_tracks));
}
BENCHMARK(BM_TrackManagerUpdate)->Arg(1)->Arg(10)->Arg(50)->Arg(100);

static void BM_EndToEndSimulationStep(benchmark::State& state) {
    const auto n_objects = static_cast<std::size_t>(state.range(0));
    ScenarioConfig cfg;
    cfg.name = "Benchmark Scenario";
    cfg.duration_sec = 100000.0;
    cfg.dt_sec = 0.1;
    cfg.seed = 42;

    for (std::size_t i = 0; i < n_objects; ++i) {
        SimulatedObjectConfig obj;
        obj.id = i + 1;
        obj.label = "Bench-" + std::to_string(obj.id);
        obj.initial_state.id = obj.id;
        obj.initial_state.px = static_cast<double>(i % 10) * 150.0 - 600.0;
        obj.initial_state.py = static_cast<double>(i / 10) * 150.0 - 600.0;
        obj.initial_state.vx = 18.0;
        obj.initial_state.vy = 12.0;
        obj.has_gnss_telemetry = true;
        cfg.objects.push_back(obj);
    }

    RadarSensorConfig radar;
    radar.id = "radar_bench";
    radar.sampling_frequency_hz = 5.0;
    radar.mean_false_alarms_per_scan = 0.5;
    cfg.radars.push_back(radar);

    GnssSensorConfig gnss;
    gnss.id = "gnss_bench";
    gnss.sampling_frequency_hz = 2.0;
    cfg.gnss_sensors.push_back(gnss);

    SimulationEngine engine(cfg);
    engine.resume();

    for (auto _ : state) {
        if (!engine.step()) {
            engine.reset();
            engine.resume();
        }
    }
    state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(n_objects));
}
BENCHMARK(BM_EndToEndSimulationStep)->Arg(1)->Arg(10)->Arg(50)->Arg(100);

}  // namespace aegistrack::benchmarks

BENCHMARK_MAIN();
