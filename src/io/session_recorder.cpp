#include "aegistrack/io/session_recorder.hpp"

#include "aegistrack/config/configuration_manager.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace aegistrack {

namespace {

SensorType parseSensorType(const std::string& s) {
    if (s == "Radar") return SensorType::Radar;
    if (s == "GNSS") return SensorType::Gnss;
    if (s == "BearingOnly") return SensorType::BearingOnly;
    throw std::invalid_argument("Unknown SensorType in session JSON: " + s);
}

SystemEventType parseEventType(const std::string& s) {
    if (s == "TrackCreated") return SystemEventType::TrackCreated;
    if (s == "TrackConfirmed") return SystemEventType::TrackConfirmed;
    if (s == "TrackCoasting") return SystemEventType::TrackCoasting;
    if (s == "TrackRecovered") return SystemEventType::TrackRecovered;
    if (s == "TrackDeleted") return SystemEventType::TrackDeleted;
    if (s == "SensorOutageStarted") return SystemEventType::SensorOutageStarted;
    if (s == "SensorOutageEnded") return SystemEventType::SensorOutageEnded;
    if (s == "StaleMeasurementDropped") return SystemEventType::StaleMeasurementDropped;
    if (s == "ScenarioStarted") return SystemEventType::ScenarioStarted;
    if (s == "ScenarioCompleted") return SystemEventType::ScenarioCompleted;
    return SystemEventType::ScenarioStarted;
}

}  // namespace

RecordedSession SessionRecorder::capture(const SimulationEngine& engine) {
    RecordedSession session;
    session.format_version = kCurrentFormatVersion;
    session.scenario_config = engine.config();
    session.recorded_measurements = engine.allRecordedMeasurements();
    session.recorded_events = engine.eventBus().history();
    session.recorded_metrics = engine.metricsCollector().history();
    session.cumulative_position_rmse_m = engine.metricsCollector().cumulativePositionRmseM();
    session.cumulative_velocity_rmse_m = engine.metricsCollector().cumulativeVelocityRmseM();
    session.cumulative_raw_measurement_rmse_m =
        engine.metricsCollector().cumulativeRawMeasurementRmseM();
    return session;
}

void SessionRecorder::saveSessionJson(const RecordedSession& session,
                                      const std::filesystem::path& filepath) {
    if (filepath.has_parent_path()) {
        std::filesystem::create_directories(filepath.parent_path());
    }
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        throw std::runtime_error("SessionRecorder::saveSessionJson: cannot open '" +
                                 filepath.string() + "' for writing");
    }

    nlohmann::json j;
    j["format_version"] = session.format_version;
    j["scenario"] = ConfigurationManager::toJson(session.scenario_config);
    j["summary"] = {
        {"cumulative_position_rmse_m", session.cumulative_position_rmse_m},
        {"cumulative_velocity_rmse_m", session.cumulative_velocity_rmse_m},
        {"cumulative_raw_measurement_rmse_m", session.cumulative_raw_measurement_rmse_m},
        {"measurement_count", session.recorded_measurements.size()},
        {"event_count", session.recorded_events.size()}
    };

    j["measurements"] = nlohmann::json::array();
    for (const auto& m : session.recorded_measurements) {
        nlohmann::json mj;
        mj["measurement_id"] = m.measurement_id;
        mj["sensor_id"] = m.sensor_id;
        mj["sensor_type"] = std::string(toString(m.sensor_type));
        mj["capture_timestamp"] = m.capture_timestamp;
        mj["arrival_timestamp"] = m.arrival_timestamp;
        mj["z"] = std::vector<double>(m.z.data(), m.z.data() + m.z.size());
        mj["R_flat"] = std::vector<double>(m.R.data(), m.R.data() + m.R.size());
        mj["sensor_x"] = m.sensor_position.x();
        mj["sensor_y"] = m.sensor_position.y();
        mj["is_clutter"] = m.is_clutter;
        if (m.evaluation_ground_truth_id.has_value()) {
            mj["gt_id"] = *m.evaluation_ground_truth_id;
        }
        j["measurements"].push_back(std::move(mj));
    }

    j["events"] = nlohmann::json::array();
    for (const auto& ev : session.recorded_events) {
        j["events"].push_back({
            {"event_id", ev.event_id},
            {"timestamp", ev.timestamp},
            {"type", std::string(toString(ev.type))},
            {"related_track_id", ev.related_track_id},
            {"related_sensor_id", ev.related_sensor_id},
            {"message", ev.message}
        });
    }

    ofs << j.dump(2) << '\n';
    if (!ofs.good()) {
        throw std::runtime_error("SessionRecorder::saveSessionJson: write failure on '" +
                                 filepath.string() + "'");
    }
}

RecordedSession SessionRecorder::loadSessionJson(const std::filesystem::path& filepath) {
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        throw std::runtime_error("SessionRecorder::loadSessionJson: cannot open '" +
                                 filepath.string() + "'");
    }

    nlohmann::json j;
    try {
        ifs >> j;
    } catch (const nlohmann::json::parse_error& e) {
        throw std::invalid_argument(std::string("Invalid session JSON: ") + e.what());
    }

    RecordedSession session;
    session.format_version = j.at("format_version").get<std::string>();
    if (session.format_version != kCurrentFormatVersion) {
        throw std::invalid_argument("Unsupported session format_version: " +
                                    session.format_version);
    }
    session.scenario_config = ConfigurationManager::fromJson(j.at("scenario"));

    if (j.contains("summary")) {
        const auto& s = j.at("summary");
        session.cumulative_position_rmse_m = s.value("cumulative_position_rmse_m", 0.0);
        session.cumulative_velocity_rmse_m = s.value("cumulative_velocity_rmse_m", 0.0);
        session.cumulative_raw_measurement_rmse_m =
            s.value("cumulative_raw_measurement_rmse_m", 0.0);
    }

    if (j.contains("measurements")) {
        for (const auto& mj : j.at("measurements")) {
            Measurement m;
            m.measurement_id = mj.at("measurement_id").get<std::uint64_t>();
            m.sensor_id = mj.at("sensor_id").get<std::string>();
            m.sensor_type = parseSensorType(mj.at("sensor_type").get<std::string>());
            m.capture_timestamp = mj.at("capture_timestamp").get<double>();
            m.arrival_timestamp = mj.at("arrival_timestamp").get<double>();

            const auto z_vec = mj.at("z").get<std::vector<double>>();
            const auto dim = static_cast<Eigen::Index>(z_vec.size());
            m.z = Eigen::Map<const VectorXd>(z_vec.data(), dim);

            const auto r_vec = mj.at("R_flat").get<std::vector<double>>();
            if (static_cast<Eigen::Index>(r_vec.size()) != dim * dim) {
                throw std::invalid_argument("Corrupted R_flat dimension in session JSON");
            }
            m.R = Eigen::Map<const MatrixXd>(r_vec.data(), dim, dim);
            m.sensor_position.x() = mj.value("sensor_x", 0.0);
            m.sensor_position.y() = mj.value("sensor_y", 0.0);
            m.is_clutter = mj.value("is_clutter", false);
            if (mj.contains("gt_id")) {
                m.evaluation_ground_truth_id = mj.at("gt_id").get<ObjectId>();
            }
            m.is_valid = true;
            session.recorded_measurements.push_back(std::move(m));
        }
    }

    if (j.contains("events")) {
        for (const auto& ej : j.at("events")) {
            SystemEvent ev;
            ev.event_id = ej.value("event_id", 0ULL);
            ev.timestamp = ej.at("timestamp").get<double>();
            ev.type = parseEventType(ej.at("type").get<std::string>());
            ev.related_track_id = ej.value("related_track_id", 0ULL);
            ev.related_sensor_id = ej.value("related_sensor_id", std::string{});
            ev.message = ej.value("message", std::string{});
            session.recorded_events.push_back(std::move(ev));
        }
    }

    return session;
}

void SessionRecorder::exportBundle(const SimulationEngine& engine,
                                   const std::filesystem::path& output_dir) {
    std::filesystem::create_directories(output_dir);
    const RecordedSession session = capture(engine);
    saveSessionJson(session, output_dir / "session.json");
    exportTracksCsv(engine, output_dir / "tracks.csv");
    exportMeasurementsCsv(session.recorded_measurements, output_dir / "measurements.csv");
    exportMetricsCsv(session.recorded_metrics, output_dir / "metrics.csv");
    exportGroundTruthCsv(engine.objects(), output_dir / "ground_truth.csv");
}

void SessionRecorder::exportTracksCsv(const SimulationEngine& engine,
                                      const std::filesystem::path& filepath) {
    if (filepath.has_parent_path()) {
        std::filesystem::create_directories(filepath.parent_path());
    }
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        throw std::runtime_error("SessionRecorder::exportTracksCsv: cannot open '" +
                                 filepath.string() + "'");
    }
    ofs << "track_id,status,timestamp_s,px_m,py_m,vx_m_s,vy_m_s,hit_count,miss_count\n";
    ofs << std::fixed << std::setprecision(4);

    auto writeTrack = [&](const Track& t) {
        const auto status_str = toString(t.status());
        for (const auto& pt : t.history()) {
            ofs << t.id() << ',' << status_str << ',' << pt.timestamp << ',' << pt.position.x()
                << ',' << pt.position.y() << ',' << pt.velocity.x() << ',' << pt.velocity.y()
                << ',' << t.hitCount() << ',' << t.missCount() << '\n';
        }
    };

    for (const auto& t : engine.fusionEngine().trackManager().archivedDeletedTracks()) {
        writeTrack(t);
    }
    for (const auto& t : engine.fusionEngine().trackManager().tracks()) {
        writeTrack(t);
    }
}

void SessionRecorder::exportMeasurementsCsv(std::span<const Measurement> measurements,
                                            const std::filesystem::path& filepath) {
    if (filepath.has_parent_path()) {
        std::filesystem::create_directories(filepath.parent_path());
    }
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        throw std::runtime_error("SessionRecorder::exportMeasurementsCsv: cannot open '" +
                                 filepath.string() + "'");
    }
    ofs << "measurement_id,sensor_id,sensor_type,capture_time_s,arrival_time_s,z0,z1,cart_x_m,cart_y_m,is_clutter\n";
    ofs << std::fixed << std::setprecision(4);

    for (const auto& m : measurements) {
        Vector2d cart = Vector2d::Zero();
        if (m.sensor_type != SensorType::BearingOnly && m.isConsistent()) {
            cart = m.toCartesianPosition();
        }
        const double z0 = (m.z.size() >= 1) ? m.z(0) : 0.0;
        const double z1 = (m.z.size() >= 2) ? m.z(1) : 0.0;
        ofs << m.measurement_id << ',' << m.sensor_id << ',' << toString(m.sensor_type) << ','
            << m.capture_timestamp << ',' << m.arrival_timestamp << ',' << z0 << ',' << z1 << ','
            << cart.x() << ',' << cart.y() << ',' << (m.is_clutter ? 1 : 0) << '\n';
    }
}

void SessionRecorder::exportMetricsCsv(std::span<const MetricsSnapshot> metrics,
                                       const std::filesystem::path& filepath) {
    if (filepath.has_parent_path()) {
        std::filesystem::create_directories(filepath.parent_path());
    }
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        throw std::runtime_error("SessionRecorder::exportMetricsCsv: cannot open '" +
                                 filepath.string() + "'");
    }
    ofs << "timestamp_s,pos_rmse_m,vel_rmse_m_s,raw_meas_rmse_m,confirmed_tracks,tentative_tracks,coasting_tracks,gt_count,latency_us\n";
    ofs << std::fixed << std::setprecision(4);

    for (const auto& s : metrics) {
        ofs << s.timestamp << ',' << s.position_rmse_m << ',' << s.velocity_rmse_m << ','
            << s.raw_measurement_rmse_m << ',' << s.confirmed_tracks << ',' << s.tentative_tracks
            << ',' << s.coasting_tracks << ',' << s.active_ground_truth_count << ','
            << s.step_latency_us << '\n';
    }
}

void SessionRecorder::exportGroundTruthCsv(std::span<const SimulatedObject> objects,
                                           const std::filesystem::path& filepath) {
    if (filepath.has_parent_path()) {
        std::filesystem::create_directories(filepath.parent_path());
    }
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        throw std::runtime_error("SessionRecorder::exportGroundTruthCsv: cannot open '" +
                                 filepath.string() + "'");
    }
    ofs << "object_id,label,timestamp_s,px_m,py_m,vx_m_s,vy_m_s\n";
    ofs << std::fixed << std::setprecision(4);

    for (const auto& obj : objects) {
        for (const auto& st : obj.trajectoryHistory()) {
            ofs << obj.id() << ',' << obj.label() << ',' << st.timestamp << ',' << st.px << ','
                << st.py << ',' << st.vx << ',' << st.vy << '\n';
        }
    }
}

MeasurementReplayResult SessionRecorder::replayMeasurements(
    const RecordedSession& session,
    const TrackManagerConfig& override_tracker_config) {
    SimulationClock sim_clock(0.0, session.scenario_config.dt_sec);
    std::vector<SimulatedObject> gt_objects;
    gt_objects.reserve(session.scenario_config.objects.size());
    for (const auto& obj_cfg : session.scenario_config.objects) {
        gt_objects.emplace_back(obj_cfg);
    }

    FusionEngine engine(override_tracker_config);
    MetricsCollector metrics;

    std::size_t meas_cursor = 0;
    const auto& all_meas = session.recorded_measurements;

    auto stepAtTime = [&](TimestampSec t) {
        std::vector<Measurement> step_batch;
        while (meas_cursor < all_meas.size() &&
               all_meas[meas_cursor].arrival_timestamp <= t + 1e-9) {
            step_batch.push_back(all_meas[meas_cursor]);
            ++meas_cursor;
        }
        engine.step(step_batch, t);
        metrics.evaluateStep(t,
                             gt_objects,
                             engine.trackManager(),
                             step_batch,
                             0,
                             0,
                             engine.lastUpdateLatencyUs());
    };

    stepAtTime(0.0);
    while (sim_clock.currentTime() + 1e-9 < session.scenario_config.duration_sec) {
        const double dt = sim_clock.nominalDt();
        const TimestampSec t_now = sim_clock.tick();
        for (auto& obj : gt_objects) {
            obj.step(dt, t_now);
        }
        stepAtTime(t_now);
    }

    MeasurementReplayResult res;
    res.tracker_config = override_tracker_config;
    res.cumulative_position_rmse_m = metrics.cumulativePositionRmseM();
    res.cumulative_velocity_rmse_m = metrics.cumulativeVelocityRmseM();
    res.total_measurements_processed = engine.totalMeasurementsProcessed();
    res.total_tracks_created = engine.trackManager().totalTracksCreated();
    res.total_tracks_deleted = engine.trackManager().totalTracksDeleted();
    res.metrics_history = metrics.history();
    return res;
}

}  // namespace aegistrack
