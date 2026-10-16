#pragma once

#include "aegistrack/simulation/simulation_engine.hpp"
#include "analytics_chart_item.hpp"
#include "tracking_canvas_item.hpp"

#include <QObject>
#include <QString>
#include <QTimer>
#include <QVariantList>
#include <QVariantMap>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stop_token>
#include <thread>
#include <vector>

namespace aegistrack::gui {

/**
 * @brief Thread-safe bridge between the background C++20 SimulationEngine worker thread
 * and the Qt 6 Quick / QML workstation interface.
 */
class DashboardController : public QObject {
    Q_OBJECT
    Q_PROPERTY(QString scenarioName READ scenarioName NOTIFY telemetryUpdated)
    Q_PROPERTY(QString simulationStatusText READ simulationStatusText NOTIFY telemetryUpdated)
    Q_PROPERTY(bool isPaused READ isPaused NOTIFY telemetryUpdated)
    Q_PROPERTY(double simulationTime READ simulationTime NOTIFY telemetryUpdated)
    Q_PROPERTY(double scenarioDuration READ scenarioDuration NOTIFY telemetryUpdated)
    Q_PROPERTY(double playbackSpeed READ playbackSpeed WRITE setPlaybackSpeed NOTIFY telemetryUpdated)

    // Top Status Bar KPIs
    Q_PROPERTY(int activeTracksCount READ activeTracksCount NOTIFY telemetryUpdated)
    Q_PROPERTY(int confirmedTracksCount READ confirmedTracksCount NOTIFY telemetryUpdated)
    Q_PROPERTY(int tentativeTracksCount READ tentativeTracksCount NOTIFY telemetryUpdated)
    Q_PROPERTY(int coastingTracksCount READ coastingTracksCount NOTIFY telemetryUpdated)
    Q_PROPERTY(int groundTruthCount READ groundTruthCount NOTIFY telemetryUpdated)
    Q_PROPERTY(double measurementsPerSec READ measurementsPerSec NOTIFY telemetryUpdated)
    Q_PROPERTY(double currentPositionRmse READ currentPositionRmse NOTIFY telemetryUpdated)
    Q_PROPERTY(double cumulativePositionRmse READ cumulativePositionRmse NOTIFY telemetryUpdated)
    Q_PROPERTY(double cumulativeVelocityRmse READ cumulativeVelocityRmse NOTIFY telemetryUpdated)
    Q_PROPERTY(double rawMeasurementRmse READ rawMeasurementRmse NOTIFY telemetryUpdated)
    Q_PROPERTY(double processingLatencyUs READ processingLatencyUs NOTIFY telemetryUpdated)
    Q_PROPERTY(int totalMeasurementsProcessed READ totalMeasurementsProcessed NOTIFY telemetryUpdated)
    Q_PROPERTY(int totalDetectionFailures READ totalDetectionFailures NOTIFY telemetryUpdated)
    Q_PROPERTY(int totalFalseAlarms READ totalFalseAlarms NOTIFY telemetryUpdated)

    // Selected Track Inspector
    Q_PROPERTY(int selectedTrackId READ selectedTrackId WRITE setSelectedTrackId NOTIFY selectedTrackUpdated)
    Q_PROPERTY(bool hasSelectedTrack READ hasSelectedTrack NOTIFY selectedTrackUpdated)
    Q_PROPERTY(QString selectedTrackStatus READ selectedTrackStatus NOTIFY selectedTrackUpdated)
    Q_PROPERTY(QString selectedTrackPosition READ selectedTrackPosition NOTIFY selectedTrackUpdated)
    Q_PROPERTY(QString selectedTrackVelocity READ selectedTrackVelocity NOTIFY selectedTrackUpdated)
    Q_PROPERTY(QString selectedTrackSpeed READ selectedTrackSpeed NOTIFY selectedTrackUpdated)
    Q_PROPERTY(QString selectedTrackCovariance READ selectedTrackCovariance NOTIFY selectedTrackUpdated)
    Q_PROPERTY(QString selectedTrackNis READ selectedTrackNis NOTIFY selectedTrackUpdated)
    Q_PROPERTY(QString selectedTrackExistenceProb READ selectedTrackExistenceProb NOTIFY selectedTrackUpdated)
    Q_PROPERTY(QString selectedTrackLastUpdate READ selectedTrackLastUpdate NOTIFY selectedTrackUpdated)
    Q_PROPERTY(QString selectedTrackAge READ selectedTrackAge NOTIFY selectedTrackUpdated)
    Q_PROPERTY(int selectedTrackHits READ selectedTrackHits NOTIFY selectedTrackUpdated)
    Q_PROPERTY(int selectedTrackMisses READ selectedTrackMisses NOTIFY selectedTrackUpdated)
    Q_PROPERTY(QString selectedTrackRecentSensors READ selectedTrackRecentSensors NOTIFY selectedTrackUpdated)

    // Dynamic Models for Views
    Q_PROPERTY(QVariantList scenariosList READ scenariosList CONSTANT)
    Q_PROPERTY(QVariantList sensorsList READ sensorsList NOTIFY telemetryUpdated)
    Q_PROPERTY(QVariantList logsList READ logsList NOTIFY telemetryUpdated)
    Q_PROPERTY(QVariantList tracksList READ tracksList NOTIFY telemetryUpdated)
    Q_PROPERTY(QVariantList timelineMarkers READ timelineMarkers NOTIFY telemetryUpdated)

    // Fusion Comparison Report
    Q_PROPERTY(bool hasComparisonReport READ hasComparisonReport NOTIFY comparisonUpdated)
    Q_PROPERTY(double radarOnlyPosRmse READ radarOnlyPosRmse NOTIFY comparisonUpdated)
    Q_PROPERTY(double gnssOnlyPosRmse READ gnssOnlyPosRmse NOTIFY comparisonUpdated)
    Q_PROPERTY(double fusedPosRmse READ fusedPosRmse NOTIFY comparisonUpdated)
    Q_PROPERTY(double radarOnlyVelRmse READ radarOnlyVelRmse NOTIFY comparisonUpdated)
    Q_PROPERTY(double gnssOnlyVelRmse READ gnssOnlyVelRmse NOTIFY comparisonUpdated)
    Q_PROPERTY(double fusedVelRmse READ fusedVelRmse NOTIFY comparisonUpdated)
    Q_PROPERTY(QString statusBannerMessage READ statusBannerMessage NOTIFY statusBannerChanged)

public:
    explicit DashboardController(const std::string& initial_scenario_path = "",
                                 QObject* parent = nullptr);
    ~DashboardController() override;

    DashboardController(const DashboardController&) = delete;
    DashboardController& operator=(const DashboardController&) = delete;

    // Property getters
    [[nodiscard]] QString scenarioName() const;
    [[nodiscard]] QString simulationStatusText() const;
    [[nodiscard]] bool isPaused() const;
    [[nodiscard]] double simulationTime() const;
    [[nodiscard]] double scenarioDuration() const;
    [[nodiscard]] double playbackSpeed() const;

    [[nodiscard]] int activeTracksCount() const;
    [[nodiscard]] int confirmedTracksCount() const;
    [[nodiscard]] int tentativeTracksCount() const;
    [[nodiscard]] int coastingTracksCount() const;
    [[nodiscard]] int groundTruthCount() const;
    [[nodiscard]] double measurementsPerSec() const;
    [[nodiscard]] double currentPositionRmse() const;
    [[nodiscard]] double cumulativePositionRmse() const;
    [[nodiscard]] double cumulativeVelocityRmse() const;
    [[nodiscard]] double rawMeasurementRmse() const;
    [[nodiscard]] double processingLatencyUs() const;
    [[nodiscard]] int totalMeasurementsProcessed() const;
    [[nodiscard]] int totalDetectionFailures() const;
    [[nodiscard]] int totalFalseAlarms() const;

    [[nodiscard]] int selectedTrackId() const noexcept { return selected_track_id_; }
    void setSelectedTrackId(int id);
    [[nodiscard]] bool hasSelectedTrack() const;
    [[nodiscard]] QString selectedTrackStatus() const;
    [[nodiscard]] QString selectedTrackPosition() const;
    [[nodiscard]] QString selectedTrackVelocity() const;
    [[nodiscard]] QString selectedTrackSpeed() const;
    [[nodiscard]] QString selectedTrackCovariance() const;
    [[nodiscard]] QString selectedTrackNis() const;
    [[nodiscard]] QString selectedTrackExistenceProb() const;
    [[nodiscard]] QString selectedTrackLastUpdate() const;
    [[nodiscard]] QString selectedTrackAge() const;
    [[nodiscard]] int selectedTrackHits() const;
    [[nodiscard]] int selectedTrackMisses() const;
    [[nodiscard]] QString selectedTrackRecentSensors() const;

    [[nodiscard]] QVariantList scenariosList() const { return scenarios_list_; }
    [[nodiscard]] QVariantList sensorsList() const;
    [[nodiscard]] QVariantList logsList() const;
    [[nodiscard]] QVariantList tracksList() const;
    [[nodiscard]] QVariantList timelineMarkers() const;

    [[nodiscard]] bool hasComparisonReport() const noexcept {
        return comparison_report_ != nullptr;
    }
    [[nodiscard]] double radarOnlyPosRmse() const noexcept;
    [[nodiscard]] double gnssOnlyPosRmse() const noexcept;
    [[nodiscard]] double fusedPosRmse() const noexcept;
    [[nodiscard]] double radarOnlyVelRmse() const noexcept;
    [[nodiscard]] double gnssOnlyVelRmse() const noexcept;
    [[nodiscard]] double fusedVelRmse() const noexcept;
    [[nodiscard]] QString statusBannerMessage() const { return status_banner_message_; }

    // QML Invokable Commands
    Q_INVOKABLE void play();
    Q_INVOKABLE void pause();
    Q_INVOKABLE void togglePlayPause();
    Q_INVOKABLE void restart();
    Q_INVOKABLE void stepForward();
    Q_INVOKABLE void setPlaybackSpeed(double speed);

    Q_INVOKABLE void loadScenarioFile(const QString& filename, int seedOverride = -1);
    Q_INVOKABLE void toggleSensorOperational(const QString& sensorId, bool operational);
    Q_INVOKABLE void runMatchedFusionComparison();
    Q_INVOKABLE void exportSessionBundle(const QString& outputDir = "exports/latest_session");
    Q_INVOKABLE void replayExportedSession(const QString& sessionFilePath,
                                           const QString& associatorName);
    Q_INVOKABLE void applyTrackerSettings(int confirmationHits,
                                          double gateThreshold2d,
                                          double processNoiseAccelStd,
                                          const QString& associatorName);

    Q_INVOKABLE void registerCanvas(TrackingCanvasItem* canvas);
    Q_INVOKABLE void registerChart(AnalyticsChartItem* chart);
    Q_INVOKABLE void fitAllCanvases();

signals:
    void telemetryUpdated();
    void selectedTrackUpdated();
    void comparisonUpdated();
    void statusBannerChanged();

private slots:
    void onGuiRefreshTick();

private:
    void workerLoop(std::stop_token stop_token);
    void populateScenariosCatalog();
    const Track* findSelectedTrack() const;

    mutable std::mutex engine_mutex_;
    std::condition_variable_any worker_cv_;
    std::unique_ptr<SimulationEngine> engine_;
    std::jthread worker_thread_;

    QTimer gui_timer_;
    std::shared_ptr<const SimulationSnapshot> gui_snapshot_;
    std::uint64_t last_rendered_step_{UINT64_MAX};
    bool force_gui_refresh_{true};

    int selected_track_id_{-1};
    QVariantList scenarios_list_;
    std::shared_ptr<const FusionComparisonReport> comparison_report_;
    QString status_banner_message_{"Ready — Select a scenario or press Play to start telemetry."};

    std::vector<TrackingCanvasItem*> canvases_;
    std::vector<AnalyticsChartItem*> charts_;
};

}  // namespace aegistrack::gui
