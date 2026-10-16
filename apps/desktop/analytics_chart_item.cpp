#include "analytics_chart_item.hpp"

#include <QColor>
#include <QFont>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <algorithm>
#include <cmath>

namespace aegistrack::gui {

AnalyticsChartItem::AnalyticsChartItem(QQuickItem* parent)
    : QQuickPaintedItem(parent) {
    setAntialiasing(true);
    setRenderTarget(QQuickPaintedItem::FramebufferObject);
}

void AnalyticsChartItem::setSnapshot(std::shared_ptr<const SimulationSnapshot> snapshot) {
    snapshot_ = std::move(snapshot);
    update();
}

void AnalyticsChartItem::setComparisonReport(
    std::shared_ptr<const FusionComparisonReport> report) {
    comparison_report_ = std::move(report);
    update();
}

void AnalyticsChartItem::setChartType(const QString& type) {
    if (chart_type_ != type) {
        chart_type_ = type;
        emit chartTypeChanged();
        update();
    }
}

void AnalyticsChartItem::setTitle(const QString& t) {
    if (title_ != t) {
        title_ = t;
        emit titleChanged();
        update();
    }
}

void AnalyticsChartItem::paint(QPainter* painter) {
    const double w = width();
    const double h = height();
    if (w <= 20.0 || h <= 20.0) {
        return;
    }

    painter->fillRect(QRectF(0.0, 0.0, w, h), QColor("#0B1222"));
    painter->setPen(QPen(QColor("#1E2E4F"), 1.0));
    painter->drawRect(QRectF(0.5, 0.5, w - 1.0, h - 1.0));

    QFont title_font("Segoe UI", 9, QFont::Bold);
    painter->setFont(title_font);
    painter->setPen(QColor("#E2E8F0"));
    painter->drawText(QPointF(12.0, 18.0), title_);

    const QRectF plot_rect(48.0, 28.0, std::max(10.0, w - 62.0), std::max(10.0, h - 52.0));
    painter->fillRect(plot_rect, QColor("#070C18"));
    painter->setPen(QPen(QColor("#182642"), 1.0));
    painter->drawRect(plot_rect);

    for (int i = 1; i <= 3; ++i) {
        const double gy = plot_rect.top() + plot_rect.height() * (static_cast<double>(i) / 4.0);
        painter->drawLine(QPointF(plot_rect.left(), gy), QPointF(plot_rect.right(), gy));
    }

    QFont axis_font("Consolas", 8);
    painter->setFont(axis_font);

    if (chart_type_ == "rmse") {
        paintRmseChart(painter, plot_rect);
    } else if (chart_type_ == "tracks") {
        paintTracksChart(painter, plot_rect);
    } else if (chart_type_ == "latency") {
        paintLatencyChart(painter, plot_rect);
    } else if (chart_type_ == "histogram") {
        paintHistogramChart(painter, plot_rect);
    } else if (chart_type_ == "fusion_comparison") {
        paintFusionComparisonChart(painter, plot_rect);
    }
}

void AnalyticsChartItem::paintRmseChart(QPainter* painter, const QRectF& plot_rect) {
    if (!snapshot_ || snapshot_->metrics_history.empty()) {
        return;
    }
    const auto& hist = snapshot_->metrics_history;
    double max_y = 20.0;
    for (const auto& s : hist) {
        max_y = std::max({max_y, s.position_rmse_m * 1.15, s.raw_measurement_rmse_m * 1.15});
    }
    const double min_t = hist.front().timestamp;
    const double max_t = std::max(min_t + 1.0, hist.back().timestamp);

    auto mapPt = [&](double t, double val) {
        const double nx = (t - min_t) / (max_t - min_t);
        const double ny = std::clamp(val / max_y, 0.0, 1.0);
        return QPointF(plot_rect.left() + nx * plot_rect.width(),
                       plot_rect.bottom() - ny * plot_rect.height());
    };

    // Raw sensor RMSE curve (Cyan dashed)
    QPainterPath raw_path;
    QPainterPath fused_path;
    for (std::size_t i = 0; i < hist.size(); ++i) {
        const QPointF rp = mapPt(hist[i].timestamp, hist[i].raw_measurement_rmse_m);
        const QPointF fp = mapPt(hist[i].timestamp, hist[i].position_rmse_m);
        if (i == 0) {
            raw_path.moveTo(rp);
            fused_path.moveTo(fp);
        } else {
            raw_path.lineTo(rp);
            fused_path.lineTo(fp);
        }
    }

    painter->setPen(QPen(QColor("#00E5FF"), 1.3, Qt::DashLine));
    painter->drawPath(raw_path);
    painter->setPen(QPen(QColor("#10B981"), 2.0));
    painter->drawPath(fused_path);

    painter->setPen(QColor("#94A3B8"));
    painter->drawText(QPointF(4.0, plot_rect.top() + 8.0),
                      QString("%1m").arg(max_y, 0, 'f', 0));
    painter->drawText(QPointF(4.0, plot_rect.bottom()), "0m");
    painter->drawText(QPointF(plot_rect.left(), plot_rect.bottom() + 14.0),
                      QString("t=%1s").arg(min_t, 0, 'f', 1));
    painter->drawText(QPointF(plot_rect.right() - 48.0, plot_rect.bottom() + 14.0),
                      QString("t=%1s").arg(max_t, 0, 'f', 1));

    // Legend (compact right-aligned)
    QFont legend_font("Consolas", 8, QFont::Bold);
    painter->setFont(legend_font);
    painter->setPen(QColor("#10B981"));
    painter->drawText(QPointF(plot_rect.right() - 140.0, 18.0), "— Fused");
    painter->setPen(QColor("#00E5FF"));
    painter->drawText(QPointF(plot_rect.right() - 78.0, 18.0), "-- Raw Obs");
    painter->setFont(QFont("Consolas", 8));
}

void AnalyticsChartItem::paintTracksChart(QPainter* painter, const QRectF& plot_rect) {
    if (!snapshot_ || snapshot_->metrics_history.empty()) {
        return;
    }
    const auto& hist = snapshot_->metrics_history;
    double max_y = 5.0;
    for (const auto& s : hist) {
        const double total =
            static_cast<double>(s.confirmed_tracks + s.tentative_tracks + s.coasting_tracks);
        max_y = std::max({max_y,
                          total + 1.0,
                          static_cast<double>(s.active_ground_truth_count) + 1.0});
    }
    const double min_t = hist.front().timestamp;
    const double max_t = std::max(min_t + 1.0, hist.back().timestamp);

    auto mapPt = [&](double t, double val) {
        const double nx = (t - min_t) / (max_t - min_t);
        const double ny = std::clamp(val / max_y, 0.0, 1.0);
        return QPointF(plot_rect.left() + nx * plot_rect.width(),
                       plot_rect.bottom() - ny * plot_rect.height());
    };

    QPainterPath gt_path;
    QPainterPath conf_path;
    QPainterPath tent_path;
    for (std::size_t i = 0; i < hist.size(); ++i) {
        const QPointF gp =
            mapPt(hist[i].timestamp, static_cast<double>(hist[i].active_ground_truth_count));
        const QPointF cp =
            mapPt(hist[i].timestamp, static_cast<double>(hist[i].confirmed_tracks));
        const QPointF tp =
            mapPt(hist[i].timestamp, static_cast<double>(hist[i].tentative_tracks));
        if (i == 0) {
            gt_path.moveTo(gp);
            conf_path.moveTo(cp);
            tent_path.moveTo(tp);
        } else {
            gt_path.lineTo(gp);
            conf_path.lineTo(cp);
            tent_path.lineTo(tp);
        }
    }

    painter->setPen(QPen(QColor("#A78BFA"), 1.3, Qt::DotLine));
    painter->drawPath(gt_path);
    painter->setPen(QPen(QColor("#FACC15"), 1.4));
    painter->drawPath(tent_path);
    painter->setPen(QPen(QColor("#10B981"), 2.0));
    painter->drawPath(conf_path);

    painter->setPen(QColor("#94A3B8"));
    painter->drawText(QPointF(6.0, plot_rect.top() + 8.0),
                      QString::number(static_cast<int>(max_y)));
    painter->drawText(QPointF(6.0, plot_rect.bottom()), "0");

    QFont legend_font("Consolas", 8, QFont::Bold);
    painter->setFont(legend_font);
    painter->setPen(QColor("#10B981"));
    painter->drawText(QPointF(plot_rect.right() - 162.0, 18.0), "— Conf");
    painter->setPen(QColor("#FACC15"));
    painter->drawText(QPointF(plot_rect.right() - 108.0, 18.0), "— Tent");
    painter->setPen(QColor("#A78BFA"));
    painter->drawText(QPointF(plot_rect.right() - 54.0, 18.0), ".. Truth");
    painter->setFont(QFont("Consolas", 8));
}

void AnalyticsChartItem::paintLatencyChart(QPainter* painter, const QRectF& plot_rect) {
    if (!snapshot_ || snapshot_->metrics_history.empty()) {
        return;
    }
    const auto& hist = snapshot_->metrics_history;
    double max_y = 50.0;
    for (const auto& s : hist) {
        max_y = std::max(max_y, s.step_latency_us * 1.15);
    }
    const double min_t = hist.front().timestamp;
    const double max_t = std::max(min_t + 1.0, hist.back().timestamp);

    QPainterPath path;
    for (std::size_t i = 0; i < hist.size(); ++i) {
        const double nx = (hist[i].timestamp - min_t) / (max_t - min_t);
        const double ny = std::clamp(hist[i].step_latency_us / max_y, 0.0, 1.0);
        const QPointF pt(plot_rect.left() + nx * plot_rect.width(),
                         plot_rect.bottom() - ny * plot_rect.height());
        if (i == 0) {
            path.moveTo(pt);
        } else {
            path.lineTo(pt);
        }
    }

    painter->setPen(QPen(QColor("#38BDF8"), 1.8));
    painter->drawPath(path);

    painter->setPen(QColor("#94A3B8"));
    painter->drawText(QPointF(2.0, plot_rect.top() + 8.0),
                      QString("%1us").arg(max_y, 0, 'f', 0));
    painter->drawText(QPointF(6.0, plot_rect.bottom()), "0us");
}

void AnalyticsChartItem::paintHistogramChart(QPainter* painter, const QRectF& plot_rect) {
    if (!snapshot_) {
        return;
    }
    const auto& bins = snapshot_->error_histogram;
    std::uint64_t max_count = 5;
    for (const auto c : bins) {
        max_count = std::max(max_count, c);
    }

    const QStringList labels = {"0-2m", "2-5m", "5-10m", "10-20m", "20-40m", ">40m"};
    const double bar_slot_w = plot_rect.width() / static_cast<double>(bins.size());

    for (std::size_t i = 0; i < bins.size(); ++i) {
        const double ratio =
            static_cast<double>(bins[i]) / static_cast<double>(max_count);
        const double bh = ratio * (plot_rect.height() - 10.0);
        const double bx = plot_rect.left() + static_cast<double>(i) * bar_slot_w + 8.0;
        const double bw = std::max(8.0, bar_slot_w - 16.0);
        const double by = plot_rect.bottom() - bh;

        QColor bar_c("#10B981");
        if (i >= 4) {
            bar_c = QColor("#EF4444");
        } else if (i >= 2) {
            bar_c = QColor("#00E5FF");
        }
        painter->fillRect(QRectF(bx, by, bw, bh), bar_c);
        painter->setPen(QColor("#94A3B8"));
        painter->drawText(QPointF(bx, plot_rect.bottom() + 14.0), labels[static_cast<int>(i)]);
        painter->setPen(QColor("#E2E8F0"));
        painter->drawText(QPointF(bx + 2.0, by - 3.0), QString::number(bins[i]));
    }
}

void AnalyticsChartItem::paintFusionComparisonChart(QPainter* painter, const QRectF& plot_rect) {
    if (!comparison_report_ || comparison_report_->radar_gnss_fusion.metrics_history.empty()) {
        painter->setPen(QColor("#94A3B8"));
        painter->drawText(plot_rect.center() - QPointF(110.0, 0.0),
                          "Click 'Run Matched Fusion Comparison' to compute curves");
        return;
    }

    const auto& r_hist = comparison_report_->radar_only.metrics_history;
    const auto& g_hist = comparison_report_->gnss_only.metrics_history;
    const auto& f_hist = comparison_report_->radar_gnss_fusion.metrics_history;

    double max_y = 20.0;
    for (const auto& s : r_hist) max_y = std::max(max_y, s.position_rmse_m * 1.15);
    for (const auto& s : g_hist) max_y = std::max(max_y, s.position_rmse_m * 1.15);
    for (const auto& s : f_hist) max_y = std::max(max_y, s.position_rmse_m * 1.15);

    const double max_t = std::max(1.0, f_hist.back().timestamp);
    auto drawCurve = [&](const std::vector<MetricsSnapshot>& series,
                         const QColor& color,
                         double width_px,
                         Qt::PenStyle style) {
        if (series.empty()) return;
        QPainterPath p;
        for (std::size_t i = 0; i < series.size(); ++i) {
            const double nx = series[i].timestamp / max_t;
            const double ny = std::clamp(series[i].position_rmse_m / max_y, 0.0, 1.0);
            const QPointF pt(plot_rect.left() + nx * plot_rect.width(),
                             plot_rect.bottom() - ny * plot_rect.height());
            if (i == 0) p.moveTo(pt);
            else p.lineTo(pt);
        }
        painter->setPen(QPen(color, width_px, style));
        painter->drawPath(p);
    };

    drawCurve(r_hist, QColor("#00E5FF"), 1.6, Qt::DashLine);
    drawCurve(g_hist, QColor("#FFB300"), 1.6, Qt::DotLine);
    drawCurve(f_hist, QColor("#10B981"), 2.3, Qt::SolidLine);

    painter->setPen(QColor("#94A3B8"));
    painter->drawText(QPointF(4.0, plot_rect.top() + 8.0),
                      QString("%1m").arg(max_y, 0, 'f', 0));
    painter->drawText(QPointF(4.0, plot_rect.bottom()), "0m");

    QFont legend_font("Consolas", 8, QFont::Bold);
    painter->setFont(legend_font);
    painter->setPen(QColor("#10B981"));
    painter->drawText(QPointF(plot_rect.right() - 185.0, 18.0), "— Fused");
    painter->setPen(QColor("#00E5FF"));
    painter->drawText(QPointF(plot_rect.right() - 128.0, 18.0), "-- Radar");
    painter->setPen(QColor("#FFB300"));
    painter->drawText(QPointF(plot_rect.right() - 62.0, 18.0), ".. GNSS");
    painter->setFont(QFont("Consolas", 8));
}

}  // namespace aegistrack::gui
