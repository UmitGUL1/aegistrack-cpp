#include "aegistrack/simulation/simulation_engine.hpp"

#include "aegistrack/config/configuration_manager.hpp"
#include "aegistrack/sensors/sensor_factory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace aegistrack {

SimulationEngine::SimulationEngine(ScenarioConfig config)
    : config_(std::move(config)),
      clock_(0.0, config_.dt_sec),
      rng_(config_.seed),
      event_bus_(5000),
      sensor_scheduler_(config_.scheduler_config, &event_bus_),
      fusion_engine_(config_.tracker_config, &event_bus_, &profiler_) {
    ConfigurationManager::validate(config_);
    rebuildFromConfig();
}

void SimulationEngine::loadScenario(ScenarioConfig config) {
    ConfigurationManager::validate(config);
    config_ = std::move(config);
    rebuildFromConfig();
}

void SimulationEngine::reset() {
    rebuildFromConfig();
}

void SimulationEngine::resetWithSeed(std::uint64_t new_seed) {
    config_.seed = new_seed;
    rebuildFromConfig();
}

void SimulationEngine::rebuildFromConfig() {
    clock_ = SimulationClock(0.0, config_.dt_sec);
    rng_.reseed(config_.seed);
    event_bus_.clearHistory();
    profiler_.reset();
    metrics_collector_.reset();
    all_recorded_measurements_.clear();

    objects_.clear();
    objects_.reserve(config_.objects.size());
    for (const auto& obj_cfg : config_.objects) {
        objects_.emplace_back(obj_cfg);
    }

    sensor_scheduler_ = SensorScheduler(config_.scheduler_config, &event_bus_);
    for (const auto& r_cfg : config_.radars) {
        sensor_scheduler_.addSensor(SensorFactory::createRadar(r_cfg));
    }
    for (const auto& g_cfg : config_.gnss_sensors) {
        sensor_scheduler_.addSensor(SensorFactory::createGnss(g_cfg));
    }

    fusion_engine_ = FusionEngine(config_.tracker_config, &event_bus_, &profiler_);
    paused_ = true;
    completed_ = false;

    event_bus_.publish(SystemEvent{0,
                                   0.0,
                                   SystemEventType::ScenarioStarted,
                                   0,
                                   "",
                                   "Scenario '" + config_.name + "' initialized (seed=" +
                                       std::to_string(config_.seed) + ")"});

    // Sample initial measurements at t = 0.0
    const auto initial_meas = sensor_scheduler_.sampleAndReleaseReady(0.0, objects_, rng_);
    all_recorded_measurements_.insert(
        all_recorded_measurements_.end(), initial_meas.begin(), initial_meas.end());
    fusion_engine_.step(initial_meas, 0.0);

    std::uint64_t total_failures = 0;
    std::uint64_t total_false_alarms = 0;
    for (std::size_t i = 0; i < sensor_scheduler_.sensorCount(); ++i) {
        const auto s = sensor_scheduler_.sensorAt(i)->runtimeStats();
        total_failures += s.detection_failures;
        total_false_alarms += s.false_alarms_generated;
    }

    metrics_collector_.evaluateStep(0.0,
                                    objects_,
                                    fusion_engine_.trackManager(),
                                    initial_meas,
                                    total_failures,
                                    total_false_alarms,
                                    fusion_engine_.lastUpdateLatencyUs());
    updateSnapshot();
}

bool SimulationEngine::step() {
    if (completed_) {
        return false;
    }

    ScopedTimer total_step_timer(&profiler_, "simulation.total_step_us");
    const double dt = clock_.nominalDt();
    const TimestampSec t_now = clock_.tick();

    for (auto& obj : objects_) {
        obj.step(dt, t_now);
    }

    const auto ready_meas = sensor_scheduler_.sampleAndReleaseReady(t_now, objects_, rng_);
    all_recorded_measurements_.insert(
        all_recorded_measurements_.end(), ready_meas.begin(), ready_meas.end());

    fusion_engine_.step(ready_meas, t_now);

    std::uint64_t total_failures = 0;
    std::uint64_t total_false_alarms = 0;
    for (std::size_t i = 0; i < sensor_scheduler_.sensorCount(); ++i) {
        const auto s = sensor_scheduler_.sensorAt(i)->runtimeStats();
        total_failures += s.detection_failures;
        total_false_alarms += s.false_alarms_generated;
    }

    metrics_collector_.evaluateStep(t_now,
                                    objects_,
                                    fusion_engine_.trackManager(),
                                    ready_meas,
                                    total_failures,
                                    total_false_alarms,
                                    fusion_engine_.lastUpdateLatencyUs());

    if (t_now + 1e-9 >= config_.duration_sec) {
        completed_ = true;
        paused_ = true;
        event_bus_.publish(SystemEvent{0,
                                       t_now,
                                       SystemEventType::ScenarioCompleted,
                                       0,
                                       "",
                                       "Scenario '" + config_.name + "' completed at t=" +
                                           std::to_string(t_now) + "s"});
    }

    updateSnapshot();
    return !completed_;
}

void SimulationEngine::runToCompletion() {
    resume();
    while (!completed_) {
        step();
    }
}

FusionComparisonReport SimulationEngine::runFusionComparison() const {
    // Run a reference simulation to generate the exact ground-truth states and sensor measurements
    // at every step from the same random seed, then feed filtered measurement subsets into
    // three independent FusionEngine + MetricsCollector pipelines.
    SimulationClock sim_clock(0.0, config_.dt_sec);
    DeterministicRng sim_rng(config_.seed);

    std::vector<SimulatedObject> gt_objects;
    gt_objects.reserve(config_.objects.size());
    for (const auto& obj_cfg : config_.objects) {
        gt_objects.emplace_back(obj_cfg);
    }

    SensorScheduler sched(config_.scheduler_config, nullptr);
    for (const auto& r_cfg : config_.radars) {
        sched.addSensor(SensorFactory::createRadar(r_cfg));
    }
    for (const auto& g_cfg : config_.gnss_sensors) {
        sched.addSensor(SensorFactory::createGnss(g_cfg));
    }

    FusionEngine radar_engine(config_.tracker_config);
    FusionEngine gnss_engine(config_.tracker_config);
    FusionEngine fused_engine(config_.tracker_config);

    MetricsCollector radar_metrics;
    MetricsCollector gnss_metrics;
    MetricsCollector fused_metrics;

    auto processStep = [&](TimestampSec t) {
        const auto all_meas = sched.sampleAndReleaseReady(t, gt_objects, sim_rng);
        std::vector<Measurement> radar_meas;
        std::vector<Measurement> gnss_meas;
        for (const auto& m : all_meas) {
            if (m.sensor_type == SensorType::Radar) {
                radar_meas.push_back(m);
            } else if (m.sensor_type == SensorType::Gnss) {
                gnss_meas.push_back(m);
            }
        }

        radar_engine.step(radar_meas, t);
        gnss_engine.step(gnss_meas, t);
        fused_engine.step(all_meas, t);

        radar_metrics.evaluateStep(t,
                                   gt_objects,
                                   radar_engine.trackManager(),
                                   radar_meas,
                                   0,
                                   0,
                                   radar_engine.lastUpdateLatencyUs());
        gnss_metrics.evaluateStep(t,
                                  gt_objects,
                                  gnss_engine.trackManager(),
                                  gnss_meas,
                                  0,
                                  0,
                                  gnss_engine.lastUpdateLatencyUs());
        fused_metrics.evaluateStep(t,
                                   gt_objects,
                                   fused_engine.trackManager(),
                                   all_meas,
                                   0,
                                   0,
                                   fused_engine.lastUpdateLatencyUs());
    };

    processStep(0.0);
    while (sim_clock.currentTime() + 1e-9 < config_.duration_sec) {
        const double dt = sim_clock.nominalDt();
        const TimestampSec t_now = sim_clock.tick();
        for (auto& obj : gt_objects) {
            obj.step(dt, t_now);
        }
        processStep(t_now);
    }

    FusionComparisonReport report;
    report.scenario_name = config_.name;
    report.seed = config_.seed;

    report.radar_only.modality_name = "Radar-Only (EKF)";
    report.radar_only.cumulative_position_rmse_m = radar_metrics.cumulativePositionRmseM();
    report.radar_only.cumulative_velocity_rmse_m = radar_metrics.cumulativeVelocityRmseM();
    report.radar_only.cumulative_raw_measurement_rmse_m =
        radar_metrics.cumulativeRawMeasurementRmseM();
    report.radar_only.total_measurements_processed = radar_engine.totalMeasurementsProcessed();
    report.radar_only.total_tracks_created =
        radar_engine.trackManager().totalTracksCreated();
    report.radar_only.metrics_history = radar_metrics.history();

    report.gnss_only.modality_name = "GNSS-Only (KF)";
    report.gnss_only.cumulative_position_rmse_m = gnss_metrics.cumulativePositionRmseM();
    report.gnss_only.cumulative_velocity_rmse_m = gnss_metrics.cumulativeVelocityRmseM();
    report.gnss_only.cumulative_raw_measurement_rmse_m =
        gnss_metrics.cumulativeRawMeasurementRmseM();
    report.gnss_only.total_measurements_processed = gnss_engine.totalMeasurementsProcessed();
    report.gnss_only.total_tracks_created = gnss_engine.trackManager().totalTracksCreated();
    report.gnss_only.metrics_history = gnss_metrics.history();

    report.radar_gnss_fusion.modality_name = "Radar + GNSS Fusion (EKF)";
    report.radar_gnss_fusion.cumulative_position_rmse_m = fused_metrics.cumulativePositionRmseM();
    report.radar_gnss_fusion.cumulative_velocity_rmse_m = fused_metrics.cumulativeVelocityRmseM();
    report.radar_gnss_fusion.cumulative_raw_measurement_rmse_m =
        fused_metrics.cumulativeRawMeasurementRmseM();
    report.radar_gnss_fusion.total_measurements_processed =
        fused_engine.totalMeasurementsProcessed();
    report.radar_gnss_fusion.total_tracks_created =
        fused_engine.trackManager().totalTracksCreated();
    report.radar_gnss_fusion.metrics_history = fused_metrics.history();

    return report;
}

void SimulationEngine::setPlaybackSpeed(double speed) {
    if (!std::isfinite(speed) || speed <= 0.0) {
        throw std::invalid_argument("SimulationEngine::setPlaybackSpeed requires speed > 0");
    }
    playback_speed_ = std::clamp(speed, 0.1, 20.0);
    updateSnapshot();
}

void SimulationEngine::setSensorOperational(const SensorId& sensor_id, bool operational) {
    ISensor* sensor = sensor_scheduler_.findSensor(sensor_id);
    if (sensor != nullptr) {
        sensor->setManualOperational(operational);
        updateSnapshot();
    }
}

void SimulationEngine::updateSnapshot() {
    auto snap = std::make_shared<SimulationSnapshot>();
    snap->scenario_name = config_.name;
    snap->timestamp = clock_.currentTime();
    snap->duration_sec = config_.duration_sec;
    snap->step_count = clock_.stepCount();
    snap->paused = paused_;
    snap->completed = completed_;
    snap->playback_speed = playback_speed_;

    snap->objects = objects_;
    snap->tracks = fusion_engine_.trackManager().tracks();
    snap->recent_measurements = fusion_engine_.recentMeasurements();

    for (const auto& r : config_.radars) {
        const ISensor* s = sensor_scheduler_.findSensor(r.id);
        SensorStatusSnapshot s_snap;
        s_snap.id = r.id;
        s_snap.type = SensorType::Radar;
        s_snap.position = r.position;
        s_snap.max_range_m = r.max_range_m;
        s_snap.fov_rad = r.fov_rad;
        s_snap.boresight_rad = r.boresight_rad;
        s_snap.sampling_frequency_hz = r.sampling_frequency_hz;
        s_snap.operational = (s != nullptr) ? s->isOperationalAt(snap->timestamp) : r.operational;
        if (s != nullptr) {
            s_snap.stats = s->runtimeStats();
        }
        snap->sensors.push_back(std::move(s_snap));
        for (const auto& z : r.occlusion_zones) {
            snap->occlusion_zones.push_back(z);
        }
    }
    for (const auto& g : config_.gnss_sensors) {
        const ISensor* s = sensor_scheduler_.findSensor(g.id);
        SensorStatusSnapshot s_snap;
        s_snap.id = g.id;
        s_snap.type = SensorType::Gnss;
        s_snap.position = Vector2d::Zero();
        s_snap.sampling_frequency_hz = g.sampling_frequency_hz;
        s_snap.operational = (s != nullptr) ? s->isOperationalAt(snap->timestamp) : g.operational;
        if (s != nullptr) {
            s_snap.stats = s->runtimeStats();
        }
        snap->sensors.push_back(std::move(s_snap));
        for (const auto& z : g.occlusion_zones) {
            snap->occlusion_zones.push_back(z);
        }
    }

    snap->latest_metrics = metrics_collector_.latestSnapshot();
    snap->metrics_history = metrics_collector_.history();
    snap->error_histogram = metrics_collector_.positionErrorHistogram();
    snap->cumulative_position_rmse_m = metrics_collector_.cumulativePositionRmseM();
    snap->cumulative_velocity_rmse_m = metrics_collector_.cumulativeVelocityRmseM();
    snap->cumulative_raw_measurement_rmse_m = metrics_collector_.cumulativeRawMeasurementRmseM();
    snap->events = event_bus_.history();

    std::scoped_lock lock(snapshot_mutex_);
    latest_snapshot_ = std::move(snap);
}

std::shared_ptr<const SimulationSnapshot> SimulationEngine::latestSnapshot() const {
    std::scoped_lock lock(snapshot_mutex_);
    return latest_snapshot_;
}

}  // namespace aegistrack
