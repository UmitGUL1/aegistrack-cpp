#include "aegistrack/config/configuration_manager.hpp"

#include "aegistrack/core/random.hpp"
#include "aegistrack/math/angles.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace aegistrack {

namespace {

MotionModelType parseMotionModel(const std::string& str) {
    if (str == "ConstantVelocity" || str == "CV") {
        return MotionModelType::ConstantVelocity;
    }
    if (str == "ConstantAcceleration" || str == "CA") {
        return MotionModelType::ConstantAcceleration;
    }
    if (str == "CoordinatedTurn" || str == "CT") {
        return MotionModelType::CoordinatedTurn;
    }
    throw std::invalid_argument("Unknown motion_model type: '" + str + "'");
}

AssociationAlgorithmType parseAssociationAlgorithm(const std::string& str) {
    if (str == "NearestNeighbor" || str == "NN") {
        return AssociationAlgorithmType::NearestNeighbor;
    }
    if (str == "Hungarian" || str == "GNN") {
        return AssociationAlgorithmType::Hungarian;
    }
    throw std::invalid_argument("Unknown association_algorithm: '" + str + "'");
}

OutOfSequencePolicy parseOosmPolicy(const std::string& str) {
    if (str == "DropStale") {
        return OutOfSequencePolicy::DropStale;
    }
    if (str == "BoundedReorderBuffer") {
        return OutOfSequencePolicy::BoundedReorderBuffer;
    }
    throw std::invalid_argument("Unknown oosm_policy: '" + str + "'");
}

std::vector<OutageWindow> parseOutageWindows(const nlohmann::json& arr) {
    std::vector<OutageWindow> out;
    for (const auto& item : arr) {
        OutageWindow w;
        w.start_time = item.at("start_time").get<double>();
        w.end_time = item.at("end_time").get<double>();
        if (!std::isfinite(w.start_time) || !std::isfinite(w.end_time) ||
            w.end_time < w.start_time) {
            throw std::invalid_argument("Invalid outage window [start_time, end_time]");
        }
        out.push_back(w);
    }
    return out;
}

std::vector<OcclusionZone> parseOcclusionZones(const nlohmann::json& arr) {
    std::vector<OcclusionZone> out;
    for (const auto& item : arr) {
        OcclusionZone z;
        z.center_x = item.at("center_x").get<double>();
        z.center_y = item.at("center_y").get<double>();
        z.radius_m = item.at("radius_m").get<double>();
        if (!std::isfinite(z.center_x) || !std::isfinite(z.center_y) ||
            !std::isfinite(z.radius_m) || z.radius_m <= 0.0) {
            throw std::invalid_argument("Invalid occlusion zone parameters");
        }
        out.push_back(z);
    }
    return out;
}

}  // namespace

ScenarioConfig ConfigurationManager::loadFromFile(const std::filesystem::path& filepath) {
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        throw std::runtime_error("ConfigurationManager::loadFromFile: cannot open file '" +
                                 filepath.string() + "'");
    }
    std::ostringstream oss;
    oss << ifs.rdbuf();
    return loadFromString(oss.str());
}

ScenarioConfig ConfigurationManager::loadFromString(std::string_view json_text) {
    if (json_text.empty()) {
        throw std::invalid_argument("ConfigurationManager::loadFromString: empty JSON input");
    }
    nlohmann::json j;
    try {
        j = nlohmann::json::parse(json_text);
    } catch (const nlohmann::json::parse_error& e) {
        throw std::invalid_argument(std::string("Invalid JSON syntax: ") + e.what());
    }
    return fromJson(j);
}

ScenarioConfig ConfigurationManager::fromJson(const nlohmann::json& j) {
    if (!j.is_object()) {
        throw std::invalid_argument("ConfigurationManager::fromJson: root must be a JSON object");
    }

    ScenarioConfig cfg;
    cfg.name = j.value("name", "Scenario");
    cfg.description = j.value("description", "");
    cfg.duration_sec = j.at("duration_sec").get<double>();
    cfg.dt_sec = j.value("dt_sec", 0.1);
    cfg.seed = j.value("seed", 42ULL);
    cfg.enable_fusion_comparison = j.value("enable_fusion_comparison", false);

    if (j.contains("objects")) {
        for (const auto& obj_j : j.at("objects")) {
            SimulatedObjectConfig obj;
            obj.id = obj_j.at("id").get<ObjectId>();
            obj.label = obj_j.value("label", "Obj-" + std::to_string(obj.id));
            obj.initial_state.id = obj.id;
            obj.initial_state.px = obj_j.at("px").get<double>();
            obj.initial_state.py = obj_j.at("py").get<double>();
            obj.initial_state.vx = obj_j.at("vx").get<double>();
            obj.initial_state.vy = obj_j.at("vy").get<double>();
            if (obj_j.contains("ax")) {
                obj.initial_state.ax = obj_j.at("ax").get<double>();
            }
            if (obj_j.contains("ay")) {
                obj.initial_state.ay = obj_j.at("ay").get<double>();
            }
            obj.initial_motion_model =
                parseMotionModel(obj_j.value("motion_model", "ConstantVelocity"));
            obj.turn_rate_rad_s = obj_j.value("turn_rate_rad_s", 0.0);
            obj.spawn_time = obj_j.value("spawn_time", 0.0);
            obj.initial_state.timestamp = obj.spawn_time;
            if (obj_j.contains("despawn_time") && !obj_j.at("despawn_time").is_null()) {
                obj.despawn_time = obj_j.at("despawn_time").get<double>();
            }
            obj.has_gnss_telemetry = obj_j.value("has_gnss_telemetry", true);

            if (obj_j.contains("maneuvers")) {
                for (const auto& m_j : obj_j.at("maneuvers")) {
                    ManeuverCommand cmd;
                    cmd.start_time = m_j.at("start_time").get<double>();
                    cmd.end_time = m_j.at("end_time").get<double>();
                    cmd.model_type =
                        parseMotionModel(m_j.value("motion_model", "ConstantVelocity"));
                    cmd.ax = m_j.value("ax", 0.0);
                    cmd.ay = m_j.value("ay", 0.0);
                    cmd.turn_rate_rad_s = m_j.value("turn_rate_rad_s", 0.0);
                    obj.maneuvers.push_back(cmd);
                }
            }
            cfg.objects.push_back(std::move(obj));
        }
    }

    if (j.contains("procedural_objects")) {
        const auto& proc_j = j.at("procedural_objects");
        const std::size_t count = proc_j.at("count").get<std::size_t>();
        const std::uint64_t proc_seed = proc_j.value("seed", cfg.seed + 1000ULL);
        const double area_radius_m = proc_j.value("area_radius_m", 2000.0);
        const double min_speed = proc_j.value("min_speed_m_s", 12.0);
        const double max_speed = proc_j.value("max_speed_m_s", 35.0);
        const double gnss_ratio = proc_j.value("gnss_equip_ratio", 0.75);
        const double max_spawn_time = proc_j.value("max_spawn_time_sec", 0.0);

        DeterministicRng rng(proc_seed);
        ObjectId next_id = cfg.objects.empty() ? 1 : (cfg.objects.back().id + 1);
        for (std::size_t k = 0; k < count; ++k) {
            SimulatedObjectConfig obj;
            obj.id = next_id++;
            obj.label = "SimTarget-" + std::to_string(obj.id);
            const double r = rng.uniform(150.0, area_radius_m);
            const double angle = rng.uniform(-math::kPi, math::kPi);
            const double speed = rng.uniform(min_speed, max_speed);
            const double heading = rng.uniform(-math::kPi, math::kPi);

            obj.initial_state.id = obj.id;
            obj.initial_state.px = r * std::cos(angle);
            obj.initial_state.py = r * std::sin(angle);
            obj.initial_state.vx = speed * std::cos(heading);
            obj.initial_state.vy = speed * std::sin(heading);
            obj.spawn_time =
                (max_spawn_time > 0.0 && k >= count / 2) ? rng.uniform(0.0, max_spawn_time) : 0.0;
            obj.initial_state.timestamp = obj.spawn_time;
            obj.has_gnss_telemetry = rng.bernoulli(gnss_ratio);
            if (k % 5 == 0) {
                obj.initial_motion_model = MotionModelType::CoordinatedTurn;
                obj.turn_rate_rad_s = rng.uniform(-0.04, 0.04);
            } else {
                obj.initial_motion_model = MotionModelType::ConstantVelocity;
            }
            cfg.objects.push_back(std::move(obj));
        }
    }

    if (j.contains("radars")) {
        for (const auto& r_j : j.at("radars")) {
            RadarSensorConfig r;
            r.id = r_j.at("id").get<std::string>();
            r.position.x() = r_j.value("x", 0.0);
            r.position.y() = r_j.value("y", 0.0);
            r.min_range_m = r_j.value("min_range_m", 5.0);
            r.max_range_m = r_j.value("max_range_m", 5000.0);
            r.boresight_rad = r_j.value("boresight_rad", 0.0);
            r.fov_rad = r_j.value("fov_rad", math::kTwoPi);
            r.sampling_frequency_hz = r_j.value("sampling_frequency_hz", 2.0);
            r.range_noise_std_m = r_j.value("range_noise_std_m", 8.0);
            r.bearing_noise_std_rad = r_j.value("bearing_noise_std_rad", 0.015);
            r.detection_probability = r_j.value("detection_probability", 0.95);
            r.mean_false_alarms_per_scan = r_j.value("mean_false_alarms_per_scan", 0.2);
            r.latency_sec = r_j.value("latency_sec", 0.0);
            r.range_bias_m = r_j.value("range_bias_m", 0.0);
            r.bearing_bias_rad = r_j.value("bearing_bias_rad", 0.0);
            r.operational = r_j.value("operational", true);
            if (r_j.contains("outage_windows")) {
                r.outage_windows = parseOutageWindows(r_j.at("outage_windows"));
            }
            if (r_j.contains("occlusion_zones")) {
                r.occlusion_zones = parseOcclusionZones(r_j.at("occlusion_zones"));
            }
            cfg.radars.push_back(std::move(r));
        }
    }

    if (j.contains("gnss_sensors")) {
        for (const auto& g_j : j.at("gnss_sensors")) {
            GnssSensorConfig g;
            g.id = g_j.at("id").get<std::string>();
            g.sampling_frequency_hz = g_j.value("sampling_frequency_hz", 1.0);
            g.position_noise_std_x_m = g_j.value("position_noise_std_x_m", 5.0);
            g.position_noise_std_y_m = g_j.value("position_noise_std_y_m", 5.0);
            g.packet_loss_probability = g_j.value("packet_loss_probability", 0.05);
            g.latency_sec = g_j.value("latency_sec", 0.0);
            g.bias_x_m = g_j.value("bias_x_m", 0.0);
            g.bias_y_m = g_j.value("bias_y_m", 0.0);
            g.drift_rate_x_m_s = g_j.value("drift_rate_x_m_s", 0.0);
            g.drift_rate_y_m_s = g_j.value("drift_rate_y_m_s", 0.0);
            g.include_telemetry_identity = g_j.value("include_telemetry_identity", false);
            g.operational = g_j.value("operational", true);
            if (g_j.contains("outage_windows")) {
                g.outage_windows = parseOutageWindows(g_j.at("outage_windows"));
            }
            if (g_j.contains("occlusion_zones")) {
                g.occlusion_zones = parseOcclusionZones(g_j.at("occlusion_zones"));
            }
            cfg.gnss_sensors.push_back(std::move(g));
        }
    }

    if (j.contains("scheduler")) {
        const auto& s_j = j.at("scheduler");
        cfg.scheduler_config.oosm_policy =
            parseOosmPolicy(s_j.value("oosm_policy", "DropStale"));
        cfg.scheduler_config.reorder_buffer_window_sec =
            s_j.value("reorder_buffer_window_sec", 0.2);
        cfg.scheduler_config.max_in_flight_queue_size =
            s_j.value("max_in_flight_queue_size", 10000ULL);
    }

    if (j.contains("tracker")) {
        const auto& t_j = j.at("tracker");
        cfg.tracker_config.confirmation_hits = t_j.value("confirmation_hits", 3U);
        cfg.tracker_config.max_tentative_misses = t_j.value("max_tentative_misses", 2U);
        cfg.tracker_config.max_missed_updates = t_j.value("max_missed_updates", 8U);
        cfg.tracker_config.coasting_threshold_sec = t_j.value("coasting_threshold_sec", 1.5);
        cfg.tracker_config.tentative_max_coast_sec = t_j.value("tentative_max_coast_sec", 2.2);
        cfg.tracker_config.max_time_without_observation_sec =
            t_j.value("max_time_without_observation_sec", 6.0);
        cfg.tracker_config.nominal_revisit_interval_sec =
            t_j.value("nominal_revisit_interval_sec", 0.6);
        cfg.tracker_config.gate_threshold_2d = t_j.value("gate_threshold_2d", 9.21034037);
        cfg.tracker_config.initial_velocity_std_m_s =
            t_j.value("initial_velocity_std_m_s", 45.0);
        cfg.tracker_config.process_noise_accel_std =
            t_j.value("process_noise_accel_std", 2.0);
        cfg.tracker_config.duplicate_initiation_radius_m =
            t_j.value("duplicate_initiation_radius_m", 25.0);
        cfg.tracker_config.min_visualization_confidence =
            t_j.value("min_visualization_confidence", 0.2);
        cfg.tracker_config.association_algorithm = parseAssociationAlgorithm(
            t_j.value("association_algorithm", "Hungarian"));
    }

    validate(cfg);
    return cfg;
}

nlohmann::json ConfigurationManager::toJson(const ScenarioConfig& config) {
    nlohmann::json j;
    j["name"] = config.name;
    j["description"] = config.description;
    j["duration_sec"] = config.duration_sec;
    j["dt_sec"] = config.dt_sec;
    j["seed"] = config.seed;
    j["enable_fusion_comparison"] = config.enable_fusion_comparison;

    j["objects"] = nlohmann::json::array();
    for (const auto& obj : config.objects) {
        nlohmann::json obj_j;
        obj_j["id"] = obj.id;
        obj_j["label"] = obj.label;
        obj_j["px"] = obj.initial_state.px;
        obj_j["py"] = obj.initial_state.py;
        obj_j["vx"] = obj.initial_state.vx;
        obj_j["vy"] = obj.initial_state.vy;
        if (obj.initial_state.ax.has_value()) {
            obj_j["ax"] = *obj.initial_state.ax;
        }
        if (obj.initial_state.ay.has_value()) {
            obj_j["ay"] = *obj.initial_state.ay;
        }
        obj_j["motion_model"] = std::string(toString(obj.initial_motion_model));
        obj_j["turn_rate_rad_s"] = obj.turn_rate_rad_s;
        obj_j["spawn_time"] = obj.spawn_time;
        if (obj.despawn_time.has_value()) {
            obj_j["despawn_time"] = *obj.despawn_time;
        }
        obj_j["has_gnss_telemetry"] = obj.has_gnss_telemetry;
        j["objects"].push_back(std::move(obj_j));
    }

    j["radars"] = nlohmann::json::array();
    for (const auto& r : config.radars) {
        nlohmann::json r_j;
        r_j["id"] = r.id;
        r_j["x"] = r.position.x();
        r_j["y"] = r.position.y();
        r_j["min_range_m"] = r.min_range_m;
        r_j["max_range_m"] = r.max_range_m;
        r_j["boresight_rad"] = r.boresight_rad;
        r_j["fov_rad"] = r.fov_rad;
        r_j["sampling_frequency_hz"] = r.sampling_frequency_hz;
        r_j["range_noise_std_m"] = r.range_noise_std_m;
        r_j["bearing_noise_std_rad"] = r.bearing_noise_std_rad;
        r_j["detection_probability"] = r.detection_probability;
        r_j["mean_false_alarms_per_scan"] = r.mean_false_alarms_per_scan;
        r_j["latency_sec"] = r.latency_sec;
        r_j["operational"] = r.operational;
        j["radars"].push_back(std::move(r_j));
    }

    j["gnss_sensors"] = nlohmann::json::array();
    for (const auto& g : config.gnss_sensors) {
        nlohmann::json g_j;
        g_j["id"] = g.id;
        g_j["sampling_frequency_hz"] = g.sampling_frequency_hz;
        g_j["position_noise_std_x_m"] = g.position_noise_std_x_m;
        g_j["position_noise_std_y_m"] = g.position_noise_std_y_m;
        g_j["packet_loss_probability"] = g.packet_loss_probability;
        g_j["latency_sec"] = g.latency_sec;
        g_j["bias_x_m"] = g.bias_x_m;
        g_j["bias_y_m"] = g.bias_y_m;
        g_j["operational"] = g.operational;
        j["gnss_sensors"].push_back(std::move(g_j));
    }

    j["tracker"] = {
        {"confirmation_hits", config.tracker_config.confirmation_hits},
        {"max_tentative_misses", config.tracker_config.max_tentative_misses},
        {"max_missed_updates", config.tracker_config.max_missed_updates},
        {"coasting_threshold_sec", config.tracker_config.coasting_threshold_sec},
        {"tentative_max_coast_sec", config.tracker_config.tentative_max_coast_sec},
        {"max_time_without_observation_sec",
         config.tracker_config.max_time_without_observation_sec},
        {"gate_threshold_2d", config.tracker_config.gate_threshold_2d},
        {"process_noise_accel_std", config.tracker_config.process_noise_accel_std},
        {"association_algorithm",
         std::string(toString(config.tracker_config.association_algorithm))}
    };

    return j;
}

void ConfigurationManager::saveToFile(const ScenarioConfig& config,
                                      const std::filesystem::path& filepath) {
    if (filepath.has_parent_path()) {
        std::filesystem::create_directories(filepath.parent_path());
    }
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        throw std::runtime_error("ConfigurationManager::saveToFile: failed to open '" +
                                 filepath.string() + "' for writing");
    }
    ofs << toJson(config).dump(2) << '\n';
}

void ConfigurationManager::validate(const ScenarioConfig& config) {
    if (config.name.empty()) {
        throw std::invalid_argument("ScenarioConfig: name must not be empty");
    }
    if (!std::isfinite(config.duration_sec) || config.duration_sec <= 0.0) {
        throw std::invalid_argument("ScenarioConfig: duration_sec must be finite and > 0");
    }
    if (!std::isfinite(config.dt_sec) || config.dt_sec <= 0.0 ||
        config.dt_sec > config.duration_sec) {
        throw std::invalid_argument(
            "ScenarioConfig: dt_sec must be finite and in (0, duration_sec]");
    }
    if (config.objects.empty()) {
        throw std::invalid_argument("ScenarioConfig: scenario must contain at least one object");
    }
    if (config.radars.empty() && config.gnss_sensors.empty()) {
        throw std::invalid_argument("ScenarioConfig: scenario must contain at least one sensor");
    }

    std::unordered_set<ObjectId> obj_ids;
    for (const auto& obj : config.objects) {
        if (obj.max_history_length > 0 && obj.max_history_length < 2) {
            throw std::invalid_argument("ScenarioConfig: max_history_length must be 0 or >= 2");
        }
        if (!obj.initial_state.isFinite()) {
            throw std::invalid_argument("ScenarioConfig: object has non-finite initial state");
        }
        if (!obj_ids.insert(obj.id).second) {
            throw std::invalid_argument("ScenarioConfig: duplicate object ID " +
                                        std::to_string(obj.id));
        }
        if (!std::isfinite(obj.spawn_time) || obj.spawn_time < 0.0) {
            throw std::invalid_argument("ScenarioConfig: invalid object spawn_time");
        }
        if (obj.despawn_time.has_value() && *obj.despawn_time <= obj.spawn_time) {
            throw std::invalid_argument("ScenarioConfig: despawn_time must be > spawn_time");
        }
    }

    std::unordered_set<SensorId> sensor_ids;
    for (const auto& r : config.radars) {
        if (!sensor_ids.insert(r.id).second) {
            throw std::invalid_argument("ScenarioConfig: duplicate sensor ID '" + r.id + "'");
        }
        // Construct temporary RadarSensor to trigger full parameter validation
        const RadarSensor validator(r);
        (void)validator;
    }
    for (const auto& g : config.gnss_sensors) {
        if (!sensor_ids.insert(g.id).second) {
            throw std::invalid_argument("ScenarioConfig: duplicate sensor ID '" + g.id + "'");
        }
        const GnssSensor validator(g);
        (void)validator;
    }
}

}  // namespace aegistrack
