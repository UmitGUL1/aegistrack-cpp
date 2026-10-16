#pragma once

#include "aegistrack/simulation/simulation_engine.hpp"

#include <QPointF>
#include <QQuickPaintedItem>
#include <memory>

namespace aegistrack::gui {

/**
 * @brief Interactive 2D engineering tracking canvas rendered with QPainter.
 *
 * Consumes immutable SimulationSnapshot instances produced by the simulation worker thread.
 * Supports pan, zoom, fit-to-world, follow-selected-track, layer visibility toggles,
 * uncertainty ellipses, velocity vectors, predicted paths, and click track selection.
 */
class TrackingCanvasItem : public QQuickPaintedItem {
    Q_OBJECT
    Q_PROPERTY(bool showGrid READ showGrid WRITE setShowGrid NOTIFY layerChanged)
    Q_PROPERTY(bool showRadarCoverage READ showRadarCoverage WRITE setShowRadarCoverage NOTIFY layerChanged)
    Q_PROPERTY(bool showGroundTruth READ showGroundTruth WRITE setShowGroundTruth NOTIFY layerChanged)
    Q_PROPERTY(bool showRawRadar READ showRawRadar WRITE setShowRawRadar NOTIFY layerChanged)
    Q_PROPERTY(bool showRawGnss READ showRawGnss WRITE setShowRawGnss NOTIFY layerChanged)
    Q_PROPERTY(bool showConfirmedTracks READ showConfirmedTracks WRITE setShowConfirmedTracks NOTIFY layerChanged)
    Q_PROPERTY(bool showTentativeTracks READ showTentativeTracks WRITE setShowTentativeTracks NOTIFY layerChanged)
    Q_PROPERTY(bool showTrajectories READ showTrajectories WRITE setShowTrajectories NOTIFY layerChanged)
    Q_PROPERTY(bool showPredictions READ showPredictions WRITE setShowPredictions NOTIFY layerChanged)
    Q_PROPERTY(bool showUncertaintyEllipses READ showUncertaintyEllipses WRITE setShowUncertaintyEllipses NOTIFY layerChanged)
    Q_PROPERTY(bool showLabels READ showLabels WRITE setShowLabels NOTIFY layerChanged)
    Q_PROPERTY(bool showDecorativeSweep READ showDecorativeSweep WRITE setShowDecorativeSweep NOTIFY layerChanged)
    Q_PROPERTY(bool followSelectedTrack READ followSelectedTrack WRITE setFollowSelectedTrack NOTIFY cameraChanged)
    Q_PROPERTY(double zoomMetersPerPixel READ zoomMetersPerPixel NOTIFY cameraChanged)
    Q_PROPERTY(int selectedTrackId READ selectedTrackId WRITE setSelectedTrackId NOTIFY selectedTrackChanged)

public:
    explicit TrackingCanvasItem(QQuickItem* parent = nullptr);

    void paint(QPainter* painter) override;

    void setSnapshot(std::shared_ptr<const SimulationSnapshot> snapshot);

    [[nodiscard]] bool showGrid() const noexcept { return show_grid_; }
    void setShowGrid(bool v);

    [[nodiscard]] bool showRadarCoverage() const noexcept { return show_radar_coverage_; }
    void setShowRadarCoverage(bool v);

    [[nodiscard]] bool showGroundTruth() const noexcept { return show_ground_truth_; }
    void setShowGroundTruth(bool v);

    [[nodiscard]] bool showRawRadar() const noexcept { return show_raw_radar_; }
    void setShowRawRadar(bool v);

    [[nodiscard]] bool showRawGnss() const noexcept { return show_raw_gnss_; }
    void setShowRawGnss(bool v);

    [[nodiscard]] bool showConfirmedTracks() const noexcept { return show_confirmed_tracks_; }
    void setShowConfirmedTracks(bool v);

    [[nodiscard]] bool showTentativeTracks() const noexcept { return show_tentative_tracks_; }
    void setShowTentativeTracks(bool v);

    [[nodiscard]] bool showTrajectories() const noexcept { return show_trajectories_; }
    void setShowTrajectories(bool v);

    [[nodiscard]] bool showPredictions() const noexcept { return show_predictions_; }
    void setShowPredictions(bool v);

    [[nodiscard]] bool showUncertaintyEllipses() const noexcept {
        return show_uncertainty_ellipses_;
    }
    void setShowUncertaintyEllipses(bool v);

    [[nodiscard]] bool showLabels() const noexcept { return show_labels_; }
    void setShowLabels(bool v);

    [[nodiscard]] bool showDecorativeSweep() const noexcept { return show_decorative_sweep_; }
    void setShowDecorativeSweep(bool v);

    [[nodiscard]] bool followSelectedTrack() const noexcept { return follow_selected_track_; }
    void setFollowSelectedTrack(bool v);

    [[nodiscard]] double zoomMetersPerPixel() const noexcept { return meters_per_pixel_; }

    [[nodiscard]] int selectedTrackId() const noexcept { return selected_track_id_; }
    void setSelectedTrackId(int track_id);

    Q_INVOKABLE void zoomIn();
    Q_INVOKABLE void zoomOut();
    Q_INVOKABLE void resetCamera();
    Q_INVOKABLE void fitToWorld();
    Q_INVOKABLE void panByPixels(double dx, double dy);
    Q_INVOKABLE void selectTrackAtScreenPos(double sx, double sy);

signals:
    void layerChanged();
    void cameraChanged();
    void selectedTrackChanged(int trackId);

private:
    [[nodiscard]] QPointF worldToScreen(const Vector2d& world_pos) const noexcept;
    [[nodiscard]] Vector2d screenToWorld(const QPointF& screen_pos) const noexcept;

    std::shared_ptr<const SimulationSnapshot> snapshot_;
    Vector2d camera_center_world_{0.0, 0.0};
    double meters_per_pixel_{3.5};
    int selected_track_id_{-1};
    bool follow_selected_track_{false};

    bool show_grid_{true};
    bool show_radar_coverage_{true};
    bool show_ground_truth_{true};
    bool show_raw_radar_{true};
    bool show_raw_gnss_{true};
    bool show_confirmed_tracks_{true};
    bool show_tentative_tracks_{true};
    bool show_trajectories_{true};
    bool show_predictions_{true};
    bool show_uncertainty_ellipses_{true};
    bool show_labels_{true};
    bool show_decorative_sweep_{false};
};

}  // namespace aegistrack::gui
