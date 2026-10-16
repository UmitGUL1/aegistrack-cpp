#pragma once

#include "aegistrack/config/scenario_config.hpp"

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>

namespace aegistrack {

/**
 * @brief Loads, validates, and serializes JSON scenario configurations.
 */
class ConfigurationManager {
public:
    /// Parses and validates a ScenarioConfig from a JSON file path.
    [[nodiscard]] static ScenarioConfig loadFromFile(const std::filesystem::path& filepath);

    /// Parses and validates a ScenarioConfig from a raw JSON string.
    [[nodiscard]] static ScenarioConfig loadFromString(std::string_view json_text);

    /// Parses and validates a ScenarioConfig from a nlohmann::json object.
    [[nodiscard]] static ScenarioConfig fromJson(const nlohmann::json& j);

    /// Serializes a ScenarioConfig to a nlohmann::json object.
    [[nodiscard]] static nlohmann::json toJson(const ScenarioConfig& config);

    /// Saves a ScenarioConfig to a formatted JSON file.
    static void saveToFile(const ScenarioConfig& config, const std::filesystem::path& filepath);

    /// Validates all semantic invariants of a ScenarioConfig; throws std::invalid_argument on error.
    static void validate(const ScenarioConfig& config);
};

}  // namespace aegistrack
