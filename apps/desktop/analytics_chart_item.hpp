#pragma once

#include "aegistrack/simulation/simulation_engine.hpp"

#include <QQuickPaintedItem>
#include <QString>
#include <memory>

namespace aegistrack::gui {

/**
 * @brief Reusable technical time-series and histogram chart component rendered with QPainter.
 *
 * Supported chartType values:
 * - "rmse": Fused Position RMSE (m) vs Raw Sensor Measurement RMSE (m) over simulation time.
 * - "tracks": Confirmed, Tentative, Coasting tracks vs Ground Truth count over time.
 * - "latency": Fusion step latency (us) over simulation time.
 * - "histogram": Position estimation error distribution across meter bins.
 * - "fusion_comparison": Matched Radar-only vs GNSS-only vs Radar+GNSS Fusion RMSE curves.
 */
class AnalyticsChartItem : public QQuickPaintedItem {
    Q_OBJECT
    Q_PROPERTY(QString chartType READ chartType WRITE setChartType NOTIFY chartTypeChanged)
    Q_PROPERTY(QString title READ title WRITE setTitle NOTIFY titleChanged)

public:
    explicit AnalyticsChartItem(QQuickItem* parent = nullptr);

    void paint(QPainter* painter) override;

    void setSnapshot(std::shared_ptr<const SimulationSnapshot> snapshot);
    void setComparisonReport(std::shared_ptr<const FusionComparisonReport> report);

    [[nodiscard]] const QString& chartType() const noexcept { return chart_type_; }
    void setChartType(const QString& type);

    [[nodiscard]] const QString& title() const noexcept { return title_; }
    void setTitle(const QString& t);

signals:
    void chartTypeChanged();
    void titleChanged();

private:
    void paintRmseChart(QPainter* painter, const QRectF& plot_rect);
    void paintTracksChart(QPainter* painter, const QRectF& plot_rect);
    void paintLatencyChart(QPainter* painter, const QRectF& plot_rect);
    void paintHistogramChart(QPainter* painter, const QRectF& plot_rect);
    void paintFusionComparisonChart(QPainter* painter, const QRectF& plot_rect);

    QString chart_type_{"rmse"};
    QString title_{"Position RMSE Over Time (m)"};
    std::shared_ptr<const SimulationSnapshot> snapshot_;
    std::shared_ptr<const FusionComparisonReport> comparison_report_;
};

}  // namespace aegistrack::gui
