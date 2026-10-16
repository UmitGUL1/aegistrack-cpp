#include "analytics_chart_item.hpp"
#include "dashboard_controller.hpp"
#include "tracking_canvas_item.hpp"

#include <QDir>
#include <QGuiApplication>
#include <QImage>
#include <QMetaObject>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>
#include <QQuickWindow>
#include <QTimer>
#include <filesystem>
#include <string>

int main(int argc, char* argv[]) {
    QGuiApplication app(argc, argv);
    app.setOrganizationName("AegisTrack");
    app.setApplicationName("AegisTrack — Multi-Sensor Fusion Workstation");
    QQuickStyle::setStyle("Basic");

    std::string scenario_arg;
    std::string screenshot_dir;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--scenario" && i + 1 < argc) {
            scenario_arg = argv[++i];
        } else if (a == "--capture-screenshots" && i + 1 < argc) {
            screenshot_dir = argv[++i];
        }
    }

    qmlRegisterType<aegistrack::gui::TrackingCanvasItem>(
        "AegisTrack.UI", 1, 0, "TrackingCanvas");
    qmlRegisterType<aegistrack::gui::AnalyticsChartItem>(
        "AegisTrack.UI", 1, 0, "AnalyticsChart");

    aegistrack::gui::DashboardController controller(scenario_arg);

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("dashboard", &controller);
    engine.load(QUrl(QStringLiteral("qrc:/ui/qml/Main.qml")));
    if (engine.rootObjects().isEmpty()) {
        return -1;
    }

    if (!screenshot_dir.empty()) {
        auto* root_win = qobject_cast<QQuickWindow*>(engine.rootObjects().first());
        if (root_win != nullptr) {
            std::filesystem::create_directories(screenshot_dir);
            // Advance simulation 140 steps so trajectories, tracks, and metrics are rich
            for (int k = 0; k < 140; ++k) {
                controller.stepForward();
            }
            controller.runMatchedFusionComparison();

            QTimer::singleShot(450, &app, [root_win, screenshot_dir, &app, &controller]() {
                controller.fitAllCanvases();
                root_win->setProperty("currentTabIndex", 1);  // Live Tracking
                QTimer::singleShot(250, &app, [root_win, screenshot_dir, &app, &controller]() {
                    controller.fitAllCanvases();
                    const QImage img1 = root_win->grabWindow();
                    img1.save(QString::fromStdString(screenshot_dir + "/live_tracking.png"));

                    root_win->setProperty("currentTabIndex", 0);  // Dashboard
                    QTimer::singleShot(250, &app, [root_win, screenshot_dir, &app, &controller]() {
                        controller.fitAllCanvases();
                        const QImage img0 = root_win->grabWindow();
                        img0.save(
                            QString::fromStdString(screenshot_dir + "/dashboard_overview.png"));

                        root_win->setProperty("currentTabIndex", 4);  // Analytics
                        QTimer::singleShot(250, &app, [root_win, screenshot_dir, &app]() {
                            const QImage img4 = root_win->grabWindow();
                            img4.save(QString::fromStdString(
                                screenshot_dir + "/analytics_comparison.png"));

                            root_win->setProperty("currentTabIndex", 2);  // Sensors
                            QTimer::singleShot(250, &app, [root_win, screenshot_dir, &app]() {
                                const QImage img2 = root_win->grabWindow();
                                img2.save(QString::fromStdString(
                                    screenshot_dir + "/sensors_view.png"));
                                app.quit();
                            });
                        });
                    });
                });
            });
        }
    }

    return app.exec();
}
