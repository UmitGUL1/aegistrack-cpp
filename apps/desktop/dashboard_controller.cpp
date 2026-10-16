#include "dashboard_controller.hpp"

#include "aegistrack/config/configuration_manager.hpp"
#include "aegistrack/io/session_recorder.hpp"
#include "aegistrack/math/angles.hpp"

#include <QStringList>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>

namespace aegistrack::gui {

DashboardController::DashboardController(const std::string& initial_scenario_path,
                                         QObject* parent)
    : QObject(parent) {
    populateScenariosCatalog();

    std::filesystem::path default_path = initial_scenario_path;
    if (default_path.empty() || !std::filesystem::exists(default_path)) {
        default_path =
            std::filesystem::path(AEGISTRACK_SOURCE_DIR) / "scenarios" / "multiple_objects.json";
    }

    ScenarioConfig cfg = ConfigurationManager::loadFromFile(default_path);
    engine_ = std::make_unique<SimulationEngine>(std::move(cfg));
    gui_snapshot_ = engine_->latestSnapshot();

    connect(&gui_timer_, &QTimer::timeout, this, &DashboardController::onGuiRefreshTick);
    gui_timer_.start(16);  // 60 Hz decoupled UI refresh loop

    worker_thread_ = std::jthread([this](std::stop_token st) { workerLoop(std::move(st)); });
}

DashboardController::~DashboardController() {
    gui_timer_.stop();
    worker_thread_.request_stop();
    worker_cv_.notify_all();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

void DashboardController::workerLoop(std::stop_token stop_token) {
    while (!stop_token.stop_requested()) {
        double sleep_ms = 50.0;
        {
            std::unique_lock<std::mutex> lock(engine_mutex_);
            worker_cv_.wait(lock, stop_token, [this] {
                return engine_ && !engine_->isPaused() && !engine_->isCompleted();
            });
            if (stop_token.stop_requested()) {
                break;
            }
            if (engine_ && !engine_->isPaused() && !engine_->isCompleted()) {
                engine_->step();
                const double speed = std::max(0.1, engine_->playbackSpeed());
                sleep_ms = (engine_->config().dt_sec * 1000.0) / speed;
            }
        }
        std::unique_lock<std::mutex> wait_lock(engine_mutex_);
        worker_cv_.wait_for(
            wait_lock,
            stop_token,
            std::chrono::microseconds(static_cast<int>( std::max(1000.0, sleep_ms * 1000.0))),
            [] { return false; });
    }
}

void DashboardController::onGuiRefreshTick() {
    std::shared_ptr<const SimulationSnapshot> latest;
    {
        std::scoped_lock lock(engine_mutex_);
        if (engine_) {
            latest = engine_->latestSnapshot();
        }
    }
    if (!latest) {
        return;
    }

    if (force_gui_refresh_ || latest->step_count != last_rendered_step_ ||
        latest->paused != gui_snapshot_->paused) {
        gui_snapshot_ = latest;
        last_rendered_step_ = latest->step_count;
        force_gui_refresh_ = false;

        // Auto-select first confirmed track if none selected
        if (selected_track_id_ < 0 && !gui_snapshot_->tracks.empty()) {
            selected_track_id_ = static_cast<int>(gui_snapshot_->tracks.front().id());
        }

        for (auto* c : canvases_) {
            if (c != nullptr) {
                c->setSnapshot(gui_snapshot_);
            }
        }
        for (auto* ch : charts_) {
            if (ch != nullptr) {
                ch->setSnapshot(gui_snapshot_);
            }
        }

        emit telemetryUpdated();
        emit selectedTrackUpdated();
    }
}

void DashboardController::populateScenariosCatalog() {
    struct Entry {
        const char* file;
        const char* code;
        const char* title;
        const char* summary;
    };
    const std::vector<Entry> catalog = {
        {"basic.json", "SCENARIO A", "Basic Tracking", "Single aircraft constant-velocity baseline"},
        {"multiple_objects.json", "SCENARIO B", "Multiple Objects (20)", "20 independent targets with staggered spawns"},
        {"crossing.json", "SCENARIO C", "Crossing Trajectories", "4 intersecting paths testing velocity gating"},
        {"noisy_measurements.json", "SCENARIO D", "Noisy Measurements", "High-variance radar & GNSS variance reduction"},
        {"sensor_failure.json", "SCENARIO E", "Sensor Failure", "Scheduled primary radar and GNSS outages"},
        {"occlusion.json", "SCENARIO F", "Occlusion & Recovery", "Circular shadow testing coasting & re-acquisition"},
        {"dense_environment.json", "SCENARIO G", "Dense Environment (25)", "Closely spaced fleet with elevated radar clutter"},
        {"stress.json", "SCENARIO H", "Stress Test (80)", "80 simultaneous targets with dual radars"},
        {"fusion-comparison.json", "SCENARIO I", "Fusion Comparison", "Matched Radar-only vs GNSS-only vs Fused evaluation"},
        {"dynamic_maneuvers.json", "SCENARIO J", "Dynamic Maneuvers", "Coordinated turns & constant acceleration phases"}
    };

    scenarios_list_.clear();
    for (const auto& item : catalog) {
        QVariantMap m;
        m["filename"] = QString::fromUtf8(item.file);
        m["code"] = QString::fromUtf8(item.code);
        m["title"] = QString::fromUtf8(item.title);
        m["summary"] = QString::fromUtf8(item.summary);
        scenarios_list_.append(m);
    }
}

QString DashboardController::scenarioName() const {
    return gui_snapshot_ ? QString::fromStdString(gui_snapshot_->scenario_name) : QString{};
}

QString DashboardController::simulationStatusText() const {
    if (!gui_snapshot_) return "IDLE";
    if (gui_snapshot_->completed) return "COMPLETED";
    return gui_snapshot_->paused ? "PAUSED" : "RUNNING";
}

bool DashboardController::isPaused() const {
    return gui_snapshot_ ? gui_snapshot_->paused : true;
}

double DashboardController::simulationTime() const {
    return gui_snapshot_ ? gui_snapshot_->timestamp : 0.0;
}

double DashboardController::scenarioDuration() const {
    return gui_snapshot_ ? gui_snapshot_->duration_sec : 30.0;
}

double DashboardController::playbackSpeed() const {
    return gui_snapshot_ ? gui_snapshot_->playback_speed : 1.0;
}

int DashboardController::activeTracksCount() const {
    if (!gui_snapshot_) return 0;
    return static_cast<int>(gui_snapshot_->latest_metrics.confirmed_tracks +
                            gui_snapshot_->latest_metrics.coasting_tracks);
}

int DashboardController::confirmedTracksCount() const {
    return gui_snapshot_ ? static_cast<int>(gui_snapshot_->latest_metrics.confirmed_tracks) : 0;
}

int DashboardController::tentativeTracksCount() const {
    return gui_snapshot_ ? static_cast<int>(gui_snapshot_->latest_metrics.tentative_tracks) : 0;
}

int DashboardController::coastingTracksCount() const {
    return gui_snapshot_ ? static_cast<int>(gui_snapshot_->latest_metrics.coasting_tracks) : 0;
}

int DashboardController::groundTruthCount() const {
    return gui_snapshot_
               ? static_cast<int>(gui_snapshot_->latest_metrics.active_ground_truth_count)
               : 0;
}

double DashboardController::measurementsPerSec() const {
    return gui_snapshot_ ? gui_snapshot_->latest_metrics.measurements_per_sec : 0.0;
}

double DashboardController::currentPositionRmse() const {
    return gui_snapshot_ ? gui_snapshot_->latest_metrics.position_rmse_m : 0.0;
}

double DashboardController::cumulativePositionRmse() const {
    return gui_snapshot_ ? gui_snapshot_->cumulative_position_rmse_m : 0.0;
}

double DashboardController::cumulativeVelocityRmse() const {
    return gui_snapshot_ ? gui_snapshot_->cumulative_velocity_rmse_m : 0.0;
}

double DashboardController::rawMeasurementRmse() const {
    return gui_snapshot_ ? gui_snapshot_->cumulative_raw_measurement_rmse_m : 0.0;
}

double DashboardController::processingLatencyUs() const {
    return gui_snapshot_ ? gui_snapshot_->latest_metrics.step_latency_us : 0.0;
}

int DashboardController::totalMeasurementsProcessed() const {
    return gui_snapshot_ ? static_cast<int>(gui_snapshot_->latest_metrics.cumulative_measurements)
                         : 0;
}

int DashboardController::totalDetectionFailures() const {
    return gui_snapshot_
               ? static_cast<int>(gui_snapshot_->latest_metrics.cumulative_detection_failures)
               : 0;
}

int DashboardController::totalFalseAlarms() const {
    return gui_snapshot_ ? static_cast<int>(gui_snapshot_->latest_metrics.cumulative_false_alarms)
                         : 0;
}

void DashboardController::setSelectedTrackId(int id) {
    if (selected_track_id_ != id) {
        selected_track_id_ = id;
        for (auto* c : canvases_) {
            if (c != nullptr) {
                c->setSelectedTrackId(id);
            }
        }
        emit selectedTrackUpdated();
    }
}

const Track* DashboardController::findSelectedTrack() const {
    if (!gui_snapshot_ || selected_track_id_ < 0) {
        return nullptr;
    }
    for (const auto& t : gui_snapshot_->tracks) {
        if (static_cast<int>(t.id()) == selected_track_id_) {
            return &t;
        }
    }
    return nullptr;
}

bool DashboardController::hasSelectedTrack() const {
    return findSelectedTrack() != nullptr;
}

QString DashboardController::selectedTrackStatus() const {
    const Track* t = findSelectedTrack();
    if (!t) return "NONE";
    const auto sv = toString(t->status());
    return QString::fromUtf8(sv.data(), static_cast<qsizetype>(sv.size())).toUpper();
}

QString DashboardController::selectedTrackPosition() const {
    const Track* t = findSelectedTrack();
    if (!t) return "—";
    return QString("(%1 m, %2 m)")
        .arg(t->position().x(), 0, 'f', 1)
        .arg(t->position().y(), 0, 'f', 1);
}

QString DashboardController::selectedTrackVelocity() const {
    const Track* t = findSelectedTrack();
    if (!t) return "—";
    return QString("(%1 m/s, %2 m/s)")
        .arg(t->velocity().x(), 0, 'f', 1)
        .arg(t->velocity().y(), 0, 'f', 1);
}

QString DashboardController::selectedTrackSpeed() const {
    const Track* t = findSelectedTrack();
    if (!t) return "—";
    return QString("%1 m/s (%2 deg)")
        .arg(t->speed(), 0, 'f', 1)
        .arg(math::radToDeg(t->headingRad()), 0, 'f', 1);
}

QString DashboardController::selectedTrackCovariance() const {
    const Track* t = findSelectedTrack();
    if (!t) return "—";
    const Matrix2d Pxy = t->positionCovariance();
    const double sx = std::sqrt(std::max(0.0, Pxy(0, 0)));
    const double sy = std::sqrt(std::max(0.0, Pxy(1, 1)));
    const auto ell = t->uncertaintyEllipse95();
    return QString("σx=%1m, σy=%2m | 95% Ell: %3x%4m")
        .arg(sx, 0, 'f', 2)
        .arg(sy, 0, 'f', 2)
        .arg(ell.semi_major_m, 0, 'f', 1)
        .arg(ell.semi_minor_m, 0, 'f', 1);
}

QString DashboardController::selectedTrackNis() const {
    const Track* t = findSelectedTrack();
    if (!t) return "—";
    return QString("d² = %1").arg(t->lastNis(), 0, 'f', 3);
}

QString DashboardController::selectedTrackExistenceProb() const {
    const Track* t = findSelectedTrack();
    if (!t) return "—";
    return QString("P_exist = %1 (LLR = %2)")
        .arg(t->existenceProbability(), 0, 'f', 3)
        .arg(t->logLikelihoodRatio(), 0, 'f', 2);
}

QString DashboardController::selectedTrackLastUpdate() const {
    const Track* t = findSelectedTrack();
    if (!t) return "—";
    return QString("t = %1 s (Δt = %2 s)")
        .arg(t->lastUpdateTimestamp(), 0, 'f', 2)
        .arg(t->timeSinceLastUpdateSec(), 0, 'f', 2);
}

QString DashboardController::selectedTrackAge() const {
    const Track* t = findSelectedTrack();
    if (!t) return "—";
    return QString("%1 s").arg(t->ageSec(), 0, 'f', 2);
}

int DashboardController::selectedTrackHits() const {
    const Track* t = findSelectedTrack();
    return t ? static_cast<int>(t->hitCount()) : 0;
}

int DashboardController::selectedTrackMisses() const {
    const Track* t = findSelectedTrack();
    return t ? static_cast<int>(t->missCount()) : 0;
}

QString DashboardController::selectedTrackRecentSensors() const {
    const Track* t = findSelectedTrack();
    if (!t || t->recentSensors().empty()) return "None";
    QStringList list;
    for (const auto& s : t->recentSensors()) {
        list.append(QString::fromStdString(s));
    }
    return list.join(", ");
}

QVariantList DashboardController::sensorsList() const {
    QVariantList list;
    if (!gui_snapshot_) return list;
    for (const auto& s : gui_snapshot_->sensors) {
        QVariantMap m;
        m["id"] = QString::fromStdString(s.id);
        const auto tv = toString(s.type);
        m["type"] = QString::fromUtf8(tv.data(), static_cast<qsizetype>(tv.size()));
        m["position"] = QString("(%1m, %2m)").arg(s.position.x(), 0, 'f', 0).arg(s.position.y(), 0, 'f', 0);
        m["rateHz"] = s.sampling_frequency_hz;
        m["maxRangeM"] = s.max_range_m;
        m["operational"] = s.operational;
        m["scans"] = static_cast<qulonglong>(s.stats.total_scans);
        m["generated"] = static_cast<qulonglong>(s.stats.measurements_generated);
        m["failures"] = static_cast<qulonglong>(s.stats.detection_failures);
        m["falseAlarms"] = static_cast<qulonglong>(s.stats.false_alarms_generated);
        list.append(m);
    }
    return list;
}

QVariantList DashboardController::logsList() const {
    QVariantList list;
    if (!gui_snapshot_) return list;
    const auto& evs = gui_snapshot_->events;
    const std::size_t count = evs.size();
    const std::size_t start = (count > 80) ? (count - 80) : 0;
    for (std::size_t i = count; i > start; --i) {
        const auto& e = evs[i - 1];
        QVariantMap m;
        m["timestamp"] = QString("%1s").arg(e.timestamp, 0, 'f', 2);
        const auto tv = toString(e.type);
        m["type"] = QString::fromUtf8(tv.data(), static_cast<qsizetype>(tv.size()));
        m["message"] = QString::fromStdString(e.message);
        list.append(m);
    }
    return list;
}

QVariantList DashboardController::tracksList() const {
    QVariantList list;
    if (!gui_snapshot_) return list;
    for (const auto& t : gui_snapshot_->tracks) {
        QVariantMap m;
        m["id"] = static_cast<int>(t.id());
        const auto sv = toString(t.status());
        m["status"] = QString::fromUtf8(sv.data(), static_cast<qsizetype>(sv.size()));
        m["pos"] = QString("(%1, %2)").arg(t.position().x(), 0, 'f', 0).arg(t.position().y(), 0, 'f', 0);
        m["speed"] = QString("%1 m/s").arg(t.speed(), 0, 'f', 1);
        m["hits"] = static_cast<int>(t.hitCount());
        m["misses"] = static_cast<int>(t.missCount());
        list.append(m);
    }
    return list;
}

QVariantList DashboardController::timelineMarkers() const {
    QVariantList list;
    if (!gui_snapshot_ || gui_snapshot_->duration_sec <= 0.0) return list;
    const double dur = gui_snapshot_->duration_sec;
    for (const auto& e : gui_snapshot_->events) {
        if (e.type == SystemEventType::TrackConfirmed ||
            e.type == SystemEventType::SensorOutageStarted ||
            e.type == SystemEventType::TrackDeleted) {
            QVariantMap m;
            m["ratio"] = std::clamp(e.timestamp / dur, 0.0, 1.0);
            const auto tv = toString(e.type);
            m["type"] = QString::fromUtf8(tv.data(), static_cast<qsizetype>(tv.size()));
            list.append(m);
        }
    }
    return list;
}

double DashboardController::radarOnlyPosRmse() const noexcept {
    return comparison_report_ ? comparison_report_->radar_only.cumulative_position_rmse_m : 0.0;
}
double DashboardController::gnssOnlyPosRmse() const noexcept {
    return comparison_report_ ? comparison_report_->gnss_only.cumulative_position_rmse_m : 0.0;
}
double DashboardController::fusedPosRmse() const noexcept {
    return comparison_report_ ? comparison_report_->radar_gnss_fusion.cumulative_position_rmse_m
                              : 0.0;
}
double DashboardController::radarOnlyVelRmse() const noexcept {
    return comparison_report_ ? comparison_report_->radar_only.cumulative_velocity_rmse_m : 0.0;
}
double DashboardController::gnssOnlyVelRmse() const noexcept {
    return comparison_report_ ? comparison_report_->gnss_only.cumulative_velocity_rmse_m : 0.0;
}
double DashboardController::fusedVelRmse() const noexcept {
    return comparison_report_ ? comparison_report_->radar_gnss_fusion.cumulative_velocity_rmse_m
                              : 0.0;
}

void DashboardController::play() {
    {
        std::scoped_lock lock(engine_mutex_);
        if (engine_) {
            if (engine_->isCompleted()) {
                engine_->reset();
            }
            engine_->resume();
        }
    }
    force_gui_refresh_ = true;
    worker_cv_.notify_all();
}

void DashboardController::pause() {
    {
        std::scoped_lock lock(engine_mutex_);
        if (engine_) {
            engine_->pause();
        }
    }
    force_gui_refresh_ = true;
    worker_cv_.notify_all();
}

void DashboardController::togglePlayPause() {
    if (isPaused()) {
        play();
    } else {
        pause();
    }
}

void DashboardController::restart() {
    {
        std::scoped_lock lock(engine_mutex_);
        if (engine_) {
            engine_->reset();
        }
    }
    force_gui_refresh_ = true;
    worker_cv_.notify_all();
}

void DashboardController::stepForward() {
    {
        std::scoped_lock lock(engine_mutex_);
        if (engine_) {
            engine_->pause();
            engine_->step();
        }
    }
    force_gui_refresh_ = true;
}

void DashboardController::setPlaybackSpeed(double speed) {
    {
        std::scoped_lock lock(engine_mutex_);
        if (engine_) {
            engine_->setPlaybackSpeed(speed);
        }
    }
    force_gui_refresh_ = true;
    worker_cv_.notify_all();
}

void DashboardController::loadScenarioFile(const QString& filename, int seedOverride) {
    try {
        std::filesystem::path p = filename.toStdString();
        if (!std::filesystem::exists(p)) {
            p = std::filesystem::path(AEGISTRACK_SOURCE_DIR) / "scenarios" / filename.toStdString();
        }
        ScenarioConfig cfg = ConfigurationManager::loadFromFile(p);
        if (seedOverride >= 0) {
            cfg.seed = static_cast<std::uint64_t>(seedOverride);
        }

        {
            std::scoped_lock lock(engine_mutex_);
            engine_->loadScenario(std::move(cfg));
        }
        selected_track_id_ = -1;
        force_gui_refresh_ = true;
        onGuiRefreshTick();
        for (auto* c : canvases_) {
            if (c != nullptr) {
                c->fitToWorld();
            }
        }
        status_banner_message_ =
            QString("Loaded scenario '%1' (seed=%2)")
                .arg(scenarioName())
                .arg(seedOverride >= 0 ? QString::number(seedOverride) : "default");
        emit statusBannerChanged();
    } catch (const std::exception& e) {
        status_banner_message_ = QString("Scenario load error: %1").arg(e.what());
        emit statusBannerChanged();
    }
}

void DashboardController::toggleSensorOperational(const QString& sensorId, bool operational) {
    {
        std::scoped_lock lock(engine_mutex_);
        if (engine_) {
            engine_->setSensorOperational(sensorId.toStdString(), operational);
        }
    }
    force_gui_refresh_ = true;
    onGuiRefreshTick();
}

void DashboardController::runMatchedFusionComparison() {
    try {
        FusionComparisonReport report;
        {
            std::scoped_lock lock(engine_mutex_);
            if (!engine_) return;
            report = engine_->runFusionComparison();
        }
        comparison_report_ = std::make_shared<FusionComparisonReport>(std::move(report));
        for (auto* ch : charts_) {
            if (ch != nullptr) {
                ch->setComparisonReport(comparison_report_);
            }
        }
        status_banner_message_ =
            QString("Matched Fusion Comparison Complete — Radar: %1m | GNSS: %2m | Fused: %3m")
                .arg(comparison_report_->radar_only.cumulative_position_rmse_m, 0, 'f', 2)
                .arg(comparison_report_->gnss_only.cumulative_position_rmse_m, 0, 'f', 2)
                .arg(comparison_report_->radar_gnss_fusion.cumulative_position_rmse_m, 0, 'f', 2);
        emit comparisonUpdated();
        emit statusBannerChanged();
    } catch (const std::exception& e) {
        status_banner_message_ = QString("Comparison error: %1").arg(e.what());
        emit statusBannerChanged();
    }
}

void DashboardController::exportSessionBundle(const QString& outputDir) {
    try {
        const std::filesystem::path dir_path(outputDir.toStdString());
        {
            std::scoped_lock lock(engine_mutex_);
            if (!engine_) return;
            SessionRecorder::exportBundle(*engine_, dir_path);
        }
        status_banner_message_ =
            QString("Exported session.json + CSV tables to '%1'").arg(outputDir);
        emit statusBannerChanged();
    } catch (const std::exception& e) {
        status_banner_message_ = QString("Export failed: %1").arg(e.what());
        emit statusBannerChanged();
    }
}

void DashboardController::replayExportedSession(const QString& sessionFilePath,
                                                const QString& associatorName) {
    try {
        const std::filesystem::path p(sessionFilePath.toStdString());
        const RecordedSession session = SessionRecorder::loadSessionJson(p);
        TrackManagerConfig t_cfg = session.scenario_config.tracker_config;
        if (associatorName == "NearestNeighbor" || associatorName == "NN") {
            t_cfg.association_algorithm = AssociationAlgorithmType::NearestNeighbor;
        } else {
            t_cfg.association_algorithm = AssociationAlgorithmType::Hungarian;
        }
        const auto res = SessionRecorder::replayMeasurements(session, t_cfg);
        status_banner_message_ =
            QString("Replayed '%1' [%2]: Pos RMSE = %3 m, Vel RMSE = %4 m/s (%5 measurements)")
                .arg(QString::fromStdString(session.scenario_config.name))
                .arg(associatorName)
                .arg(res.cumulative_position_rmse_m, 0, 'f', 2)
                .arg(res.cumulative_velocity_rmse_m, 0, 'f', 2)
                .arg(res.total_measurements_processed);
        emit statusBannerChanged();
    } catch (const std::exception& e) {
        status_banner_message_ = QString("Session replay error: %1").arg(e.what());
        emit statusBannerChanged();
    }
}

void DashboardController::applyTrackerSettings(int confirmationHits,
                                               double gateThreshold2d,
                                               double processNoiseAccelStd,
                                               const QString& associatorName) {
    try {
        std::scoped_lock lock(engine_mutex_);
        if (!engine_) return;
        ScenarioConfig cfg = engine_->config();
        cfg.tracker_config.confirmation_hits =
            static_cast<std::uint32_t>(std::max(1, confirmationHits));
        cfg.tracker_config.gate_threshold_2d = std::max(1.0, gateThreshold2d);
        cfg.tracker_config.process_noise_accel_std = std::max(0.1, processNoiseAccelStd);
        cfg.tracker_config.association_algorithm =
            (associatorName == "NearestNeighbor" || associatorName == "NN")
                ? AssociationAlgorithmType::NearestNeighbor
                : AssociationAlgorithmType::Hungarian;
        engine_->loadScenario(std::move(cfg));
        force_gui_refresh_ = true;
        status_banner_message_ = "Applied new tracker parameters and restarted scenario.";
        emit statusBannerChanged();
    } catch (const std::exception& e) {
        status_banner_message_ = QString("Settings error: %1").arg(e.what());
        emit statusBannerChanged();
    }
}

void DashboardController::registerCanvas(TrackingCanvasItem* canvas) {
    if (canvas == nullptr) return;
    if (std::find(canvases_.begin(), canvases_.end(), canvas) == canvases_.end()) {
        canvases_.push_back(canvas);
        canvas->setSnapshot(gui_snapshot_);
        canvas->fitToWorld();
        connect(canvas,
                &TrackingCanvasItem::selectedTrackChanged,
                this,
                &DashboardController::setSelectedTrackId);
    }
}

void DashboardController::registerChart(AnalyticsChartItem* chart) {
    if (chart == nullptr) return;
    if (std::find(charts_.begin(), charts_.end(), chart) == charts_.end()) {
        charts_.push_back(chart);
        chart->setSnapshot(gui_snapshot_);
        if (comparison_report_) {
            chart->setComparisonReport(comparison_report_);
        }
    }
}

void DashboardController::fitAllCanvases() {
    onGuiRefreshTick();
    for (auto* c : canvases_) {
        if (c != nullptr) {
            c->fitToWorld();
        }
    }
}

}  // namespace aegistrack::gui
