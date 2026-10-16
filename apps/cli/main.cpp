#include "aegistrack/config/configuration_manager.hpp"
#include "aegistrack/io/session_recorder.hpp"
#include "aegistrack/simulation/simulation_engine.hpp"

#include <spdlog/spdlog.h>

#include <cstdint>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {

struct CliOptions {
    std::string scenario_path;
    std::string replay_session_path;
    std::string export_dir;
    std::optional<std::uint64_t> seed_override;
    std::optional<aegistrack::AssociationAlgorithmType> associator_override;
    bool headless{true};
    bool run_fusion_comparison{false};
    bool show_help{false};
};

void printUsage(const char* prog_name) {
    std::cout << "AegisTrack CLI — Multi-Sensor Fusion & Real-Time Object Tracking Engine\n\n"
              << "Usage:\n"
              << "  " << prog_name << " --scenario <path.json> [options]\n"
              << "  " << prog_name << " --replay-session <session.json> [options]\n\n"
              << "Options:\n"
              << "  --scenario <path>         Path to scenario configuration JSON file\n"
              << "  --seed <uint64>           Override deterministic simulation random seed\n"
              << "  --export <dir>            Export session.json, tracks.csv, measurements.csv, metrics.csv\n"
              << "  --associator <NN|Hungarian> Override data association algorithm\n"
              << "  --fusion-comparison       Force Radar-only vs GNSS-only vs Fused evaluation\n"
              << "  --replay-session <path>   Replay recorded measurements from a saved session.json\n"
              << "  --headless                Run in headless batch mode (default for CLI)\n"
              << "  -h, --help                Display this help message and exit\n";
}

bool parseArgs(int argc, char** argv, CliOptions& opts, std::string& error_msg) {
    if (argc <= 1) {
        opts.show_help = true;
        return true;
    }

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            opts.show_help = true;
            return true;
        }
        if (arg == "--headless") {
            opts.headless = true;
            continue;
        }
        if (arg == "--fusion-comparison") {
            opts.run_fusion_comparison = true;
            continue;
        }
        if (arg == "--scenario") {
            if (i + 1 >= argc) {
                error_msg = "Missing value for --scenario";
                return false;
            }
            opts.scenario_path = argv[++i];
            continue;
        }
        if (arg == "--replay-session") {
            if (i + 1 >= argc) {
                error_msg = "Missing value for --replay-session";
                return false;
            }
            opts.replay_session_path = argv[++i];
            continue;
        }
        if (arg == "--export") {
            if (i + 1 >= argc) {
                error_msg = "Missing value for --export";
                return false;
            }
            opts.export_dir = argv[++i];
            continue;
        }
        if (arg == "--seed") {
            if (i + 1 >= argc) {
                error_msg = "Missing value for --seed";
                return false;
            }
            try {
                const std::string val = argv[++i];
                if (!val.empty() && val.front() == '-') {
                    error_msg = "--seed must be a non-negative integer";
                    return false;
                }
                std::size_t pos = 0;
                opts.seed_override = std::stoull(val, &pos);
                if (pos != val.size()) {
                    error_msg = "Invalid integer for --seed: " + val;
                    return false;
                }
            } catch (const std::exception&) {
                error_msg = "Invalid integer for --seed";
                return false;
            }
            continue;
        }
        if (arg == "--associator") {
            if (i + 1 >= argc) {
                error_msg = "Missing value for --associator";
                return false;
            }
            const std::string val = argv[++i];
            if (val == "NN" || val == "NearestNeighbor") {
                opts.associator_override = aegistrack::AssociationAlgorithmType::NearestNeighbor;
            } else if (val == "Hungarian" || val == "GNN") {
                opts.associator_override = aegistrack::AssociationAlgorithmType::Hungarian;
            } else {
                error_msg = "Unknown --associator value '" + val + "' (expected NN or Hungarian)";
                return false;
            }
            continue;
        }

        error_msg = "Unknown command-line argument: '" + arg + "'";
        return false;
    }

    if (opts.scenario_path.empty() && opts.replay_session_path.empty()) {
        error_msg = "Either --scenario <path> or --replay-session <path> is required";
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    CliOptions opts;
    std::string parse_error;
    if (!parseArgs(argc, argv, opts, parse_error)) {
        spdlog::error("Argument error: {}", parse_error);
        printUsage(argv[0]);
        return 1;
    }

    if (opts.show_help) {
        printUsage(argv[0]);
        return 0;
    }

    try {
        if (!opts.replay_session_path.empty()) {
            spdlog::info("Loading recorded session from '{}'...", opts.replay_session_path);
            const auto session = aegistrack::SessionRecorder::loadSessionJson(
                std::filesystem::path(opts.replay_session_path));

            aegistrack::TrackManagerConfig tracker_cfg = session.scenario_config.tracker_config;
            if (opts.associator_override.has_value()) {
                tracker_cfg.association_algorithm = *opts.associator_override;
            }

            const auto replay_res =
                aegistrack::SessionRecorder::replayMeasurements(session, tracker_cfg);

            std::cout << "\n============================================================\n"
                      << " AEGISTRACK MEASUREMENT REPLAY REPORT\n"
                      << "============================================================\n"
                      << std::fixed << std::setprecision(3)
                      << " Scenario             : " << session.scenario_config.name << "\n"
                      << " Associator           : "
                      << aegistrack::toString(tracker_cfg.association_algorithm) << "\n"
                      << " Measurements Replayed: " << replay_res.total_measurements_processed
                      << "\n"
                      << " Tracks Created       : " << replay_res.total_tracks_created << "\n"
                      << " Cumulative Pos RMSE  : " << replay_res.cumulative_position_rmse_m
                      << " m\n"
                      << " Cumulative Vel RMSE  : " << replay_res.cumulative_velocity_rmse_m
                      << " m/s\n"
                      << "============================================================\n";
            return 0;
        }

        spdlog::info("Loading scenario '{}'...", opts.scenario_path);
        aegistrack::ScenarioConfig cfg = aegistrack::ConfigurationManager::loadFromFile(
            std::filesystem::path(opts.scenario_path));

        if (opts.seed_override.has_value()) {
            cfg.seed = *opts.seed_override;
        }
        if (opts.associator_override.has_value()) {
            cfg.tracker_config.association_algorithm = *opts.associator_override;
        }

        aegistrack::SimulationEngine engine(cfg);
        spdlog::info("Running scenario '{}' (duration={:.1f}s, dt={:.2f}s, seed={}, objects={})...",
                     cfg.name,
                     cfg.duration_sec,
                     cfg.dt_sec,
                     cfg.seed,
                     cfg.objects.size());

        engine.runToCompletion();

        const auto& metrics = engine.metricsCollector();
        const auto& tm = engine.fusionEngine().trackManager();
        const auto step_stats = engine.profiler().statsFor("simulation.total_step_us");
        const auto fusion_stats = engine.profiler().statsFor("fusion.step_us");

        std::cout << "\n============================================================\n"
                  << " AEGISTRACK SIMULATION SUMMARY\n"
                  << "============================================================\n"
                  << std::fixed << std::setprecision(3)
                  << " Scenario Name        : " << cfg.name << "\n"
                  << " Random Seed          : " << cfg.seed << "\n"
                  << " Duration Simulated   : " << engine.currentTime() << " s\n"
                  << " Simulated Objects    : " << cfg.objects.size() << "\n"
                  << " Measurements Ingested: " << engine.fusionEngine().totalMeasurementsProcessed()
                  << "\n"
                  << " Confirmed Tracks     : " << tm.confirmedTrackCount()
                  << " (Total Created: " << tm.totalTracksCreated()
                  << ", Deleted: " << tm.totalTracksDeleted() << ")\n"
                  << " Raw Sensor Pos RMSE  : " << metrics.cumulativeRawMeasurementRmseM() << " m\n"
                  << " Fused Track Pos RMSE : " << metrics.cumulativePositionRmseM() << " m\n"
                  << " Fused Track Vel RMSE : " << metrics.cumulativeVelocityRmseM() << " m/s\n"
                  << " Mean Fusion Latency  : " << fusion_stats.mean_us << " us (Max: "
                  << fusion_stats.max_us << " us)\n"
                  << " Mean Step Latency    : " << step_stats.mean_us << " us\n"
                  << "============================================================\n";

        if (cfg.enable_fusion_comparison || opts.run_fusion_comparison) {
            spdlog::info("Executing matched Radar-Only vs GNSS-Only vs Fusion comparison...");
            const auto comp = engine.runFusionComparison();
            std::cout << "\n------------------------------------------------------------\n"
                      << " MULTI-SENSOR FUSION COMPARISON (MATCHED GROUND TRUTH)\n"
                      << "------------------------------------------------------------\n"
                      << " Modality                    | Pos RMSE (m) | Vel RMSE (m/s) | Meas Count\n"
                      << "-----------------------------+--------------+----------------+-----------\n"
                      << " " << std::left << std::setw(28) << comp.radar_only.modality_name << "| "
                      << std::right << std::setw(12) << comp.radar_only.cumulative_position_rmse_m
                      << " | " << std::setw(14) << comp.radar_only.cumulative_velocity_rmse_m
                      << " | " << std::setw(9) << comp.radar_only.total_measurements_processed
                      << "\n"
                      << " " << std::left << std::setw(28) << comp.gnss_only.modality_name << "| "
                      << std::right << std::setw(12) << comp.gnss_only.cumulative_position_rmse_m
                      << " | " << std::setw(14) << comp.gnss_only.cumulative_velocity_rmse_m
                      << " | " << std::setw(9) << comp.gnss_only.total_measurements_processed
                      << "\n"
                      << " " << std::left << std::setw(28) << comp.radar_gnss_fusion.modality_name
                      << "| " << std::right << std::setw(12)
                      << comp.radar_gnss_fusion.cumulative_position_rmse_m << " | "
                      << std::setw(14) << comp.radar_gnss_fusion.cumulative_velocity_rmse_m
                      << " | " << std::setw(9)
                      << comp.radar_gnss_fusion.total_measurements_processed << "\n"
                      << "------------------------------------------------------------\n";
        }

        if (!opts.export_dir.empty()) {
            const std::filesystem::path out_path(opts.export_dir);
            spdlog::info("Exporting session bundle to '{}'...", out_path.string());
            aegistrack::SessionRecorder::exportBundle(engine, out_path);
            spdlog::info("Export completed successfully.");
        }

        return 0;
    } catch (const std::invalid_argument& e) {
        spdlog::error("Configuration / validation error: {}", e.what());
        return 2;
    } catch (const std::exception& e) {
        spdlog::error("Runtime error: {}", e.what());
        return 3;
    }
}
