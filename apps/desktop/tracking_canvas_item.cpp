#include "tracking_canvas_item.hpp"

#include "aegistrack/math/angles.hpp"

#include <QColor>
#include <QFont>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <algorithm>
#include <cmath>

namespace aegistrack::gui {

TrackingCanvasItem::TrackingCanvasItem(QQuickItem* parent)
    : QQuickPaintedItem(parent) {
    setAntialiasing(true);
    setRenderTarget(QQuickPaintedItem::FramebufferObject);
}

void TrackingCanvasItem::setSnapshot(std::shared_ptr<const SimulationSnapshot> snapshot) {
    snapshot_ = std::move(snapshot);
    if (follow_selected_track_ && selected_track_id_ >= 0 && snapshot_) {
        for (const auto& t : snapshot_->tracks) {
            if (static_cast<int>(t.id()) == selected_track_id_) {
                camera_center_world_ = t.position();
                break;
            }
        }
    }
    update();
}

void TrackingCanvasItem::setShowGrid(bool v) {
    if (show_grid_ != v) { show_grid_ = v; emit layerChanged(); update(); }
}
void TrackingCanvasItem::setShowRadarCoverage(bool v) {
    if (show_radar_coverage_ != v) { show_radar_coverage_ = v; emit layerChanged(); update(); }
}
void TrackingCanvasItem::setShowGroundTruth(bool v) {
    if (show_ground_truth_ != v) { show_ground_truth_ = v; emit layerChanged(); update(); }
}
void TrackingCanvasItem::setShowRawRadar(bool v) {
    if (show_raw_radar_ != v) { show_raw_radar_ = v; emit layerChanged(); update(); }
}
void TrackingCanvasItem::setShowRawGnss(bool v) {
    if (show_raw_gnss_ != v) { show_raw_gnss_ = v; emit layerChanged(); update(); }
}
void TrackingCanvasItem::setShowConfirmedTracks(bool v) {
    if (show_confirmed_tracks_ != v) { show_confirmed_tracks_ = v; emit layerChanged(); update(); }
}
void TrackingCanvasItem::setShowTentativeTracks(bool v) {
    if (show_tentative_tracks_ != v) { show_tentative_tracks_ = v; emit layerChanged(); update(); }
}
void TrackingCanvasItem::setShowTrajectories(bool v) {
    if (show_trajectories_ != v) { show_trajectories_ = v; emit layerChanged(); update(); }
}
void TrackingCanvasItem::setShowPredictions(bool v) {
    if (show_predictions_ != v) { show_predictions_ = v; emit layerChanged(); update(); }
}
void TrackingCanvasItem::setShowUncertaintyEllipses(bool v) {
    if (show_uncertainty_ellipses_ != v) {
        show_uncertainty_ellipses_ = v;
        emit layerChanged();
        update();
    }
}
void TrackingCanvasItem::setShowLabels(bool v) {
    if (show_labels_ != v) { show_labels_ = v; emit layerChanged(); update(); }
}
void TrackingCanvasItem::setShowDecorativeSweep(bool v) {
    if (show_decorative_sweep_ != v) {
        show_decorative_sweep_ = v;
        emit layerChanged();
        update();
    }
}
void TrackingCanvasItem::setFollowSelectedTrack(bool v) {
    if (follow_selected_track_ != v) {
        follow_selected_track_ = v;
        emit cameraChanged();
        update();
    }
}
void TrackingCanvasItem::setSelectedTrackId(int track_id) {
    if (selected_track_id_ != track_id) {
        selected_track_id_ = track_id;
        emit selectedTrackChanged(selected_track_id_);
        update();
    }
}

void TrackingCanvasItem::zoomIn() {
    meters_per_pixel_ = std::max(0.25, meters_per_pixel_ * 0.8);
    emit cameraChanged();
    update();
}

void TrackingCanvasItem::zoomOut() {
    meters_per_pixel_ = std::min(25.0, meters_per_pixel_ * 1.25);
    emit cameraChanged();
    update();
}

void TrackingCanvasItem::resetCamera() {
    camera_center_world_ = Vector2d{0.0, 0.0};
    meters_per_pixel_ = 3.5;
    follow_selected_track_ = false;
    emit cameraChanged();
    update();
}

void TrackingCanvasItem::fitToWorld() {
    if (!snapshot_) {
        resetCamera();
        return;
    }
    double min_x = -800.0;
    double max_x = 800.0;
    double min_y = -600.0;
    double max_y = 600.0;
    bool any = false;

    auto expand = [&](const Vector2d& p) {
        if (!any) {
            min_x = max_x = p.x();
            min_y = max_y = p.y();
            any = true;
        } else {
            min_x = std::min(min_x, p.x());
            max_x = std::max(max_x, p.x());
            min_y = std::min(min_y, p.y());
            max_y = std::max(max_y, p.y());
        }
    };

    for (const auto& obj : snapshot_->objects) {
        if (obj.isActiveAt(snapshot_->timestamp)) {
            expand(obj.state().position());
        }
    }
    for (const auto& t : snapshot_->tracks) {
        expand(t.position());
    }
    for (const auto& s : snapshot_->sensors) {
        if (s.type == SensorType::Radar) {
            expand(s.position);
        }
    }

    camera_center_world_ = Vector2d{0.5 * (min_x + max_x), 0.5 * (min_y + max_y)};
    const double span_x = std::max(600.0, (max_x - min_x) * 1.35);
    const double span_y = std::max(600.0, (max_y - min_y) * 1.35);
    const double w = width() > 100.0 ? width() : 780.0;
    const double h = height() > 100.0 ? height() : 660.0;
    meters_per_pixel_ = std::clamp(std::max(span_x / w, span_y / h), 0.3, 12.0);
    follow_selected_track_ = false;
    emit cameraChanged();
    update();
}

void TrackingCanvasItem::panByPixels(double dx, double dy) {
    follow_selected_track_ = false;
    camera_center_world_.x() -= dx * meters_per_pixel_;
    camera_center_world_.y() += dy * meters_per_pixel_;
    emit cameraChanged();
    update();
}

void TrackingCanvasItem::selectTrackAtScreenPos(double sx, double sy) {
    if (!snapshot_) {
        return;
    }
    const QPointF click_pt(sx, sy);
    int best_id = -1;
    double best_dist_px = 22.0;

    for (const auto& t : snapshot_->tracks) {
        const QPointF sp = worldToScreen(t.position());
        const double dist_px = std::hypot(sp.x() - click_pt.x(), sp.y() - click_pt.y());
        if (dist_px < best_dist_px) {
            best_dist_px = dist_px;
            best_id = static_cast<int>(t.id());
        }
    }
    setSelectedTrackId(best_id);
}

QPointF TrackingCanvasItem::worldToScreen(const Vector2d& world_pos) const noexcept {
    const double cx = width() * 0.5;
    const double cy = height() * 0.5;
    const double sx = cx + (world_pos.x() - camera_center_world_.x()) / meters_per_pixel_;
    const double sy = cy - (world_pos.y() - camera_center_world_.y()) / meters_per_pixel_;
    return QPointF(sx, sy);
}

Vector2d TrackingCanvasItem::screenToWorld(const QPointF& screen_pos) const noexcept {
    const double cx = width() * 0.5;
    const double cy = height() * 0.5;
    const double wx = camera_center_world_.x() + (screen_pos.x() - cx) * meters_per_pixel_;
    const double wy = camera_center_world_.y() - (screen_pos.y() - cy) * meters_per_pixel_;
    return Vector2d{wx, wy};
}

void TrackingCanvasItem::paint(QPainter* painter) {
    const double w = width();
    const double h = height();
    if (w <= 2.0 || h <= 2.0) {
        return;
    }

    painter->fillRect(QRectF(0.0, 0.0, w, h), QColor("#060B16"));

    QFont label_font("Consolas", 8);
    label_font.setStyleHint(QFont::Monospace);
    painter->setFont(label_font);

    // 1. Coordinate Grid
    if (show_grid_) {
        double grid_step_m = 200.0;
        if (meters_per_pixel_ > 8.0) {
            grid_step_m = 1000.0;
        } else if (meters_per_pixel_ > 3.0) {
            grid_step_m = 500.0;
        } else if (meters_per_pixel_ < 1.0) {
            grid_step_m = 100.0;
        }

        const Vector2d top_left_w = screenToWorld(QPointF(0.0, 0.0));
        const Vector2d bot_right_w = screenToWorld(QPointF(w, h));

        const double start_x = std::floor(top_left_w.x() / grid_step_m) * grid_step_m;
        const double end_x = std::ceil(bot_right_w.x() / grid_step_m) * grid_step_m;
        const double start_y = std::floor(bot_right_w.y() / grid_step_m) * grid_step_m;
        const double end_y = std::ceil(top_left_w.y() / grid_step_m) * grid_step_m;

        const QPen minor_pen(QColor(24, 38, 66, 180), 1.0);
        const QPen axis_pen(QColor(45, 72, 120, 220), 1.4);
        const QPen text_pen(QColor("#526D96"));

        for (double gx = start_x; gx <= end_x; gx += grid_step_m) {
            const double sx = worldToScreen(Vector2d{gx, 0.0}).x();
            painter->setPen(std::abs(gx) < 1e-3 ? axis_pen : minor_pen);
            painter->drawLine(QPointF(sx, 0.0), QPointF(sx, h));
            painter->setPen(text_pen);
            painter->drawText(QPointF(sx + 4.0, h - 8.0),
                              QString("%1m").arg(static_cast<int>(std::round(gx))));
        }

        for (double gy = start_y; gy <= end_y; gy += grid_step_m) {
            const double sy = worldToScreen(Vector2d{0.0, gy}).y();
            painter->setPen(std::abs(gy) < 1e-3 ? axis_pen : minor_pen);
            painter->drawLine(QPointF(0.0, sy), QPointF(w, sy));
            painter->setPen(text_pen);
            painter->drawText(QPointF(6.0, sy - 4.0),
                              QString("%1m").arg(static_cast<int>(std::round(gy))));
        }
    }

    if (!snapshot_) {
        return;
    }

    // 2. Occlusion Zones
    for (const auto& zone : snapshot_->occlusion_zones) {
        const QPointF center_sp = worldToScreen(Vector2d{zone.center_x, zone.center_y});
        const double rad_px = zone.radius_m / meters_per_pixel_;
        painter->setPen(QPen(QColor(239, 68, 68, 170), 1.4, Qt::DashLine));
        painter->setBrush(QColor(239, 68, 68, 28));
        painter->drawEllipse(center_sp, rad_px, rad_px);
        if (show_labels_) {
            painter->setPen(QColor("#F87171"));
            painter->drawText(center_sp + QPointF(-38.0, -rad_px - 6.0), "OCCLUSION ZONE");
        }
    }

    // 3. Radar Station and Coverage Rings
    if (show_radar_coverage_) {
        for (const auto& sensor : snapshot_->sensors) {
            if (sensor.type != SensorType::Radar) {
                continue;
            }
            const QPointF s_pt = worldToScreen(sensor.position);
            const double max_r_px = sensor.max_range_m / meters_per_pixel_;

            const QColor ring_color =
                sensor.operational ? QColor(0, 229, 255, 50) : QColor(239, 68, 68, 55);
            QPen ring_pen(ring_color, 1.0, Qt::DotLine);
            painter->setPen(ring_pen);
            painter->setBrush(Qt::NoBrush);

            for (double frac : {0.25, 0.5, 0.75, 1.0}) {
                painter->drawEllipse(s_pt, max_r_px * frac, max_r_px * frac);
            }

            if (show_decorative_sweep_ && sensor.operational) {
                const double sweep_angle =
                    std::fmod(snapshot_->timestamp * 1.4, math::kTwoPi);
                const QPointF end_pt(s_pt.x() + std::cos(sweep_angle) * max_r_px,
                                     s_pt.y() - std::sin(sweep_angle) * max_r_px);
                painter->setPen(QPen(QColor(0, 229, 255, 65), 1.5));
                painter->drawLine(s_pt, end_pt);
            }

            // Radar station icon
            painter->setPen(QPen(sensor.operational ? QColor("#00E5FF") : QColor("#EF4444"), 1.8));
            painter->setBrush(QColor("#0F172A"));
            painter->drawRect(QRectF(s_pt.x() - 5.0, s_pt.y() - 5.0, 10.0, 10.0));
            painter->drawLine(QPointF(s_pt.x() - 8.0, s_pt.y()),
                              QPointF(s_pt.x() + 8.0, s_pt.y()));
            painter->drawLine(QPointF(s_pt.x(), s_pt.y() - 8.0),
                              QPointF(s_pt.x(), s_pt.y() + 8.0));
            if (show_labels_) {
                painter->setPen(QColor("#38BDF8"));
                painter->drawText(s_pt + QPointF(9.0, 4.0),
                                  QString::fromStdString(sensor.id).toUpper());
            }
        }
    }

    // 4. Raw Sensor Observations (recent window)
    const double current_t = snapshot_->timestamp;
    for (const auto& m : snapshot_->recent_measurements) {
        const double age = current_t - m.capture_timestamp;
        if (age < 0.0 || age > 2.5 || m.sensor_type == SensorType::BearingOnly ||
            !m.isConsistent()) {
            continue;
        }
        if (m.sensor_type == SensorType::Radar && !show_raw_radar_) {
            continue;
        }
        if (m.sensor_type == SensorType::Gnss && !show_raw_gnss_) {
            continue;
        }

        const int alpha = static_cast<int>(std::clamp((1.0 - age / 2.5) * 195.0, 25.0, 195.0));
        const QPointF mp = worldToScreen(m.toCartesianPosition());

        if (m.sensor_type == SensorType::Radar) {
            const QColor c = m.is_clutter ? QColor(244, 63, 94, alpha)
                                          : QColor(0, 229, 255, alpha);
            painter->setPen(QPen(c, 1.3));
            painter->drawLine(mp + QPointF(-3.5, 0.0), mp + QPointF(3.5, 0.0));
            painter->drawLine(mp + QPointF(0.0, -3.5), mp + QPointF(0.0, 3.5));
        } else {
            const QColor c(255, 179, 0, alpha);
            painter->setPen(QPen(c, 1.3));
            painter->setBrush(Qt::NoBrush);
            painter->drawEllipse(mp, 3.0, 3.0);
        }
    }

    // 5. Ground Truth Trajectories & Positions
    if (show_ground_truth_) {
        for (const auto& obj : snapshot_->objects) {
            if (!obj.isActiveAt(current_t)) {
                continue;
            }
            if (show_trajectories_ && obj.trajectoryHistory().size() >= 2) {
                QPainterPath path;
                const auto& hist = obj.trajectoryHistory();
                const std::size_t start_idx =
                    (hist.size() > 120) ? (hist.size() - 120) : 0;
                path.moveTo(worldToScreen(hist[start_idx].position()));
                for (std::size_t i = start_idx + 1; i < hist.size(); ++i) {
                    path.lineTo(worldToScreen(hist[i].position()));
                }
                painter->setPen(QPen(QColor(167, 139, 250, 95), 1.1, Qt::DotLine));
                painter->setBrush(Qt::NoBrush);
                painter->drawPath(path);
            }

            const QPointF gp = worldToScreen(obj.state().position());
            painter->setPen(QPen(QColor("#A78BFA"), 1.4));
            painter->setBrush(Qt::NoBrush);
            painter->drawEllipse(gp, 4.5, 4.5);
        }
    }

    // 6. Estimated Tracks, Uncertainty Ellipses, Trajectories, Velocity Vectors, Predictions
    for (const auto& track : snapshot_->tracks) {
        const auto status = track.status();
        if (status == TrackLifecycleStatus::Deleted) {
            continue;
        }
        if (status == TrackLifecycleStatus::Confirmed && !show_confirmed_tracks_) {
            continue;
        }
        if (status == TrackLifecycleStatus::Tentative && !show_tentative_tracks_) {
            continue;
        }

        const bool is_selected = (static_cast<int>(track.id()) == selected_track_id_);
        QColor track_color("#10B981");  // Confirmed: Emerald green
        if (status == TrackLifecycleStatus::Tentative) {
            track_color = QColor("#FACC15");  // Tentative: Muted yellow
        } else if (status == TrackLifecycleStatus::Coasting) {
            track_color = QColor("#FB923C");  // Coasting: Amber-orange
        }

        const QPointF tp = worldToScreen(track.position());

        // 6a. 95% Uncertainty Ellipse
        if (show_uncertainty_ellipses_) {
            const auto ell = track.uncertaintyEllipse95();
            const double rx = std::clamp(ell.semi_major_m / meters_per_pixel_, 2.0, 220.0);
            const double ry = std::clamp(ell.semi_minor_m / meters_per_pixel_, 2.0, 220.0);
            painter->save();
            painter->translate(tp);
            painter->rotate(-math::radToDeg(ell.orientation_rad));
            QColor fill_c = track_color;
            fill_c.setAlpha(is_selected ? 50 : 26);
            QColor stroke_c = track_color;
            stroke_c.setAlpha(is_selected ? 180 : 100);
            painter->setPen(QPen(stroke_c, 1.0));
            painter->setBrush(fill_c);
            painter->drawEllipse(QPointF(0.0, 0.0), rx, ry);
            painter->restore();
        }

        // 6b. Historical Trajectory
        if (show_trajectories_ && track.history().size() >= 2) {
            QPainterPath hist_path;
            const auto& hist = track.history();
            const std::size_t start_idx =
                (hist.size() > 120) ? (hist.size() - 120) : 0;
            hist_path.moveTo(worldToScreen(hist[start_idx].position));
            for (std::size_t i = start_idx + 1; i < hist.size(); ++i) {
                hist_path.lineTo(worldToScreen(hist[i].position));
            }
            QColor trail_c = track_color;
            trail_c.setAlpha(170);
            painter->setPen(QPen(trail_c, is_selected ? 2.2 : 1.5));
            painter->setBrush(Qt::NoBrush);
            painter->drawPath(hist_path);
        }

        // 6c. Future Path Prediction (dashed line)
        if (show_predictions_) {
            const auto future_pts = track.predictFuturePath(4.0, 4);
            if (!future_pts.empty()) {
                QPainterPath pred_path;
                pred_path.moveTo(tp);
                for (const auto& fp : future_pts) {
                    pred_path.lineTo(worldToScreen(fp));
                }
                QColor pred_c = track_color;
                pred_c.setAlpha(140);
                painter->setPen(QPen(pred_c, 1.2, Qt::DashLine));
                painter->setBrush(Qt::NoBrush);
                painter->drawPath(pred_path);
            }
        }

        // 6d. Velocity Vector
        const Vector2d vel_end_world = track.position() + track.velocity() * 1.5;
        const QPointF vel_end_sp = worldToScreen(vel_end_world);
        painter->setPen(QPen(track_color, 1.6));
        painter->drawLine(tp, vel_end_sp);

        // 6e. Track Marker & Selection Highlight
        if (is_selected) {
            painter->setPen(QPen(QColor("#38BDF8"), 2.0));
            painter->setBrush(Qt::NoBrush);
            painter->drawEllipse(tp, 11.0, 11.0);
        }

        painter->setPen(QPen(QColor("#080D1A"), 1.2));
        painter->setBrush(track_color);
        painter->drawEllipse(tp, 5.0, 5.0);

        // 6f. Technical Label
        if (show_labels_) {
            painter->setPen(is_selected ? QColor("#F8FAFC") : track_color);
            const QString tag = QString("TRK-%1 [%2m/s]")
                                    .arg(track.id(), 2, 10, QChar('0'))
                                    .arg(track.speed(), 0, 'f', 1);
            painter->drawText(tp + QPointF(8.0, -7.0), tag);
        }
    }
}

}  // namespace aegistrack::gui
