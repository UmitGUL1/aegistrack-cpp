cmake_minimum_required(VERSION 3.24)

project(
    AegisTrack
    VERSION 1.0.0
    DESCRIPTION "Multi-Sensor Fusion & Real-Time Object Tracking System"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(AEGISTRACK_BUILD_CLI "Build the headless command-line application" ON)
option(AEGISTRACK_BUILD_GUI "Build the Qt6 Quick / QML desktop workstation" ON)
option(AEGISTRACK_BUILD_TESTS "Build GoogleTest unit and integration tests" ON)
option(AEGISTRACK_BUILD_BENCHMARKS "Build Google Benchmark performance suite" ON)
option(AEGISTRACK_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(AEGISTRACK_FETCH_MISSING_DEPENDENCIES "Allow FetchContent fallback for missing packages" ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CompilerWarnings)
include(Sanitizers)
include(Dependencies)

add_library(aegistrack_options INTERFACE)
aegistrack_set_project_warnings(aegistrack_options)
aegistrack_enable_sanitizers(aegistrack_options)

# ==============================================================================
# Core Engine Library (Strictly Qt-Independent)
# ==============================================================================
set(AEGISTRACK_CORE_SOURCES
    src/math/statistics.cpp
    src/math/hungarian.cpp
    src/events/event_bus.cpp
    src/simulation/motion_model.cpp
    src/simulation/simulated_object.cpp
    src/simulation/simulation_engine.cpp
    src/sensors/radar_sensor.cpp
    src/sensors/gnss_sensor.cpp
    src/sensors/sensor_factory.cpp
    src/sensors/sensor_scheduler.cpp
    src/filtering/kalman_filter.cpp
    src/filtering/extended_kalman_filter.cpp
    src/tracking/track.cpp
    src/tracking/track_manager.cpp
    src/association/nearest_neighbor_associator.cpp
    src/association/hungarian_associator.cpp
    src/fusion/fusion_engine.cpp
    src/metrics/metrics_collector.cpp
    src/config/configuration_manager.cpp
    src/io/session_recorder.cpp
)

add_library(aegistrack_core STATIC ${AEGISTRACK_CORE_SOURCES})
target_include_directories(
    aegistrack_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
    aegistrack_core
    PUBLIC
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
        spdlog::spdlog
    PRIVATE
        aegistrack_options
)
target_compile_definitions(
    aegistrack_core
    PUBLIC
        AEGISTRACK_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

# ==============================================================================
# Headless Command-Line Application (aegistrack-cli)
# ==============================================================================
if(AEGISTRACK_BUILD_CLI)
    add_executable(aegistrack-cli apps/cli/main.cpp)
    target_link_libraries(
        aegistrack-cli
        PRIVATE
            aegistrack_core
            aegistrack_options
    )
endif()

# ==============================================================================
# Qt 6 Quick / QML Desktop Application (aegistrack-desktop)
# ==============================================================================
if(AEGISTRACK_BUILD_GUI AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/apps/desktop/main.cpp")
    find_package(Qt6 COMPONENTS Core Gui Qml Quick QuickControls2 QUIET)
    if(Qt6_FOUND)
        message(STATUS "Qt6 found (${Qt6_VERSION}); configuring aegistrack-desktop GUI...")
        set(CMAKE_AUTOMOC ON)
        set(CMAKE_AUTORCC ON)

        add_executable(
            aegistrack-desktop
            apps/desktop/main.cpp
            apps/desktop/dashboard_controller.hpp
            apps/desktop/dashboard_controller.cpp
            apps/desktop/tracking_canvas_item.hpp
            apps/desktop/tracking_canvas_item.cpp
            apps/desktop/analytics_chart_item.hpp
            apps/desktop/analytics_chart_item.cpp
            ui/qml.qrc
        )
        target_include_directories(
            aegistrack-desktop
            PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/apps/desktop
        )
        target_link_libraries(
            aegistrack-desktop
            PRIVATE
                aegistrack_core
                Qt6::Core
                Qt6::Gui
                Qt6::Qml
                Qt6::Quick
                Qt6::QuickControls2
                aegistrack_options
        )
    else()
        message(WARNING "AEGISTRACK_BUILD_GUI is ON, but Qt6 was not found. Skipping aegistrack-desktop.")
    endif()
endif()

# ==============================================================================
# Unit & Integration Tests (GoogleTest + CTest)
# ==============================================================================
if(AEGISTRACK_BUILD_TESTS)
    enable_testing()
    include(GoogleTest)

    add_executable(
        aegistrack_tests
        tests/unit/test_mathematics.cpp
        tests/unit/test_sensors.cpp
        tests/unit/test_association.cpp
        tests/unit/test_track_management.cpp
        tests/unit/test_scenarios_and_architecture.cpp
        tests/integration/test_integration_pipeline.cpp
    )
    target_link_libraries(
        aegistrack_tests
        PRIVATE
            aegistrack_core
            GTest::gtest
            GTest::gtest_main
            aegistrack_options
    )
    gtest_discover_tests(aegistrack_tests)
endif()

# ==============================================================================
# Performance Benchmarks (Google Benchmark)
# ==============================================================================
if(AEGISTRACK_BUILD_BENCHMARKS)
    add_executable(aegistrack_benchmarks benchmarks/benchmark_tracking.cpp)
    target_link_libraries(
        aegistrack_benchmarks
        PRIVATE
            aegistrack_core
            benchmark::benchmark
            aegistrack_options
    )
endif()
