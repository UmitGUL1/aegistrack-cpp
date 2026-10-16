import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window
import AegisTrack.UI 1.0

Window {
    id: rootWindow
    width: 1480
    height: 900
    minimumWidth: 1120
    minimumHeight: 720
    visible: true
    color: "#080D1A"
    title: "AegisTrack — Multi-Sensor Fusion & Object Tracking Workstation"

    property int currentTabIndex: 0

    // Color palette constants
    readonly property color bgDeep: "#080D1A"
    readonly property color bgPanel: "#0F172A"
    readonly property color bgCard: "#15203B"
    readonly property color borderMuted: "#1E2E4F"
    readonly property color accentCyan: "#00E5FF"
    readonly property color accentGreen: "#10B981"
    readonly property color accentAmber: "#FFB300"
    readonly property color accentRed: "#EF4444"
    readonly property color textPrimary: "#F8FAFC"
    readonly property color textSecondary: "#94A3B8"

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // =====================================================================
        // TOP STATUS BAR
        // =====================================================================
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 54
            color: rootWindow.bgPanel
            border.color: rootWindow.borderMuted
            border.width: 1

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 16
                anchors.rightMargin: 16
                spacing: 18

                // Brand Identity
                ColumnLayout {
                    spacing: 1
                    Text {
                        text: "AEGISTRACK"
                        color: rootWindow.accentCyan
                        font.pixelSize: 15
                        font.bold: true
                        font.letterSpacing: 1.5
                    }
                    Text {
                        text: "MULTI-SENSOR FUSION & TRACKING"
                        color: rootWindow.textSecondary
                        font.pixelSize: 9
                        font.letterSpacing: 0.8
                    }
                }

                Rectangle { width: 1; Layout.fillHeight: true; Layout.topMargin: 10; Layout.bottomMargin: 10; color: rootWindow.borderMuted }

                // Scenario Badge
                ColumnLayout {
                    spacing: 1
                    Text { text: "ACTIVE SCENARIO"; color: rootWindow.textSecondary; font.pixelSize: 9 }
                    Text {
                        text: dashboard.scenarioName
                        color: rootWindow.textPrimary
                        font.pixelSize: 12
                        font.bold: true
                    }
                }

                Item { Layout.fillWidth: true }

                // Top Status Bar Telemetry Pills
                Repeater {
                    model: [
                        { label: "CONFIRMED TRKS", val: dashboard.confirmedTracksCount.toString(), col: "#10B981" },
                        { label: "TENTATIVE / COAST", val: dashboard.tentativeTracksCount + " / " + dashboard.coastingTracksCount, col: "#FACC15" },
                        { label: "MEAS / SEC", val: dashboard.measurementsPerSec.toFixed(1) + " Hz", col: "#00E5FF" },
                        { label: "FUSED POS RMSE", val: dashboard.cumulativePositionRmse.toFixed(2) + " m", col: "#10B981" },
                        { label: "RAW SENSOR RMSE", val: dashboard.rawMeasurementRmse.toFixed(2) + " m", col: "#FFB300" },
                        { label: "FUSION LATENCY", val: dashboard.processingLatencyUs.toFixed(1) + " μs", col: "#38BDF8" },
                        { label: "SIM CLOCK", val: "T+ " + dashboard.simulationTime.toFixed(2) + " s", col: "#E2E8F0" }
                    ]
                    delegate: Rectangle {
                        Layout.preferredHeight: 38
                        Layout.preferredWidth: 118
                        radius: 4
                        color: rootWindow.bgCard
                        border.color: rootWindow.borderMuted
                        ColumnLayout {
                            anchors.centerIn: parent
                            spacing: 1
                            Text {
                                Layout.alignment: Qt.AlignHCenter
                                text: modelData.label
                                color: rootWindow.textSecondary
                                font.pixelSize: 8
                                font.bold: true
                            }
                            Text {
                                Layout.alignment: Qt.AlignHCenter
                                text: modelData.val
                                color: modelData.col
                                font.family: "Consolas"
                                font.pixelSize: 12
                                font.bold: true
                            }
                        }
                    }
                }

                // Engine Status Pill
                Rectangle {
                    Layout.preferredHeight: 34
                    Layout.preferredWidth: 96
                    radius: 4
                    color: dashboard.simulationStatusText === "RUNNING" ? "#064E3B" : "#1E293B"
                    border.color: dashboard.simulationStatusText === "RUNNING" ? "#10B981" : "#FFB300"
                    Text {
                        anchors.centerIn: parent
                        text: dashboard.simulationStatusText
                        color: dashboard.simulationStatusText === "RUNNING" ? "#10B981" : "#FFB300"
                        font.pixelSize: 11
                        font.bold: true
                    }
                }
            }
        }

        // =====================================================================
        // MAIN WORKSPACE: LEFT SIDEBAR + CENTER VIEW + RIGHT TRACK INSPECTOR
        // =====================================================================
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 0

            // -----------------------------------------------------------------
            // LEFT SIDEBAR
            // -----------------------------------------------------------------
            Rectangle {
                Layout.preferredWidth: 190
                Layout.fillHeight: true
                color: rootWindow.bgPanel
                border.color: rootWindow.borderMuted
                border.width: 1

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 6

                    Text {
                        text: "WORKSTATION VIEWS"
                        color: rootWindow.textSecondary
                        font.pixelSize: 10
                        font.bold: true
                        Layout.topMargin: 4
                        Layout.bottomMargin: 4
                    }

                    Repeater {
                        model: [
                            { idx: 0, title: "01  Dashboard" },
                            { idx: 1, title: "02  Live Tracking" },
                            { idx: 2, title: "03  Sensors" },
                            { idx: 3, title: "04  Scenarios & I/O" },
                            { idx: 4, title: "05  Live Analytics" },
                            { idx: 5, title: "06  System Logs" },
                            { idx: 6, title: "07  Filter Settings" }
                        ]
                        delegate: Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 38
                            radius: 4
                            color: rootWindow.currentTabIndex === modelData.idx ? "#172B4D" : "transparent"
                            border.color: rootWindow.currentTabIndex === modelData.idx ? rootWindow.accentCyan : "transparent"

                            Text {
                                anchors.verticalCenter: parent.verticalCenter
                                anchors.left: parent.left
                                anchors.leftMargin: 12
                                text: modelData.title
                                color: rootWindow.currentTabIndex === modelData.idx ? rootWindow.accentCyan : rootWindow.textPrimary
                                font.pixelSize: 12
                                font.bold: rootWindow.currentTabIndex === modelData.idx
                            }

                            MouseArea {
                                anchors.fill: parent
                                cursorShape: Qt.PointingHandCursor
                                onClicked: rootWindow.currentTabIndex = modelData.idx
                            }
                        }
                    }

                    Item { Layout.fillHeight: true }

                    // Color Legend Box
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 165
                        radius: 4
                        color: rootWindow.bgCard
                        border.color: rootWindow.borderMuted

                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: 10
                            spacing: 5
                            Text { text: "TACTICAL LEGEND"; color: rootWindow.textSecondary; font.pixelSize: 9; font.bold: true }

                            Repeater {
                                model: [
                                    { name: "Confirmed Track", col: "#10B981" },
                                    { name: "Tentative Track", col: "#FACC15" },
                                    { name: "Coasting Track", col: "#FB923C" },
                                    { name: "Raw Radar Obs", col: "#00E5FF" },
                                    { name: "Raw GNSS Obs", col: "#FFB300" },
                                    { name: "Ground Truth", col: "#A78BFA" }
                                ]
                                delegate: RowLayout {
                                    spacing: 8
                                    Rectangle { width: 10; height: 10; radius: 5; color: modelData.col }
                                    Text { text: modelData.name; color: rootWindow.textPrimary; font.pixelSize: 10 }
                                }
                            }
                        }
                    }
                }
            }

            // -----------------------------------------------------------------
            // CENTER STACKED VIEWS
            // -----------------------------------------------------------------
            StackLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                currentIndex: rootWindow.currentTabIndex

                // =============================================================
                // TAB 0: DASHBOARD OVERVIEW
                // =============================================================
                Item {
                    RowLayout {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 10

                        // Left 62%: Interactive 2D Tracking Canvas
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            Layout.preferredWidth: 620
                            color: "#060B16"
                            border.color: rootWindow.borderMuted

                            TrackingCanvas {
                                id: dashboardCanvas
                                anchors.fill: parent
                                anchors.margins: 1
                                Component.onCompleted: dashboard.registerCanvas(dashboardCanvas)

                                MouseArea {
                                    anchors.fill: parent
                                    acceptedButtons: Qt.LeftButton | Qt.RightButton
                                    property real lastX: 0
                                    property real lastY: 0
                                    onPressed: function(mouse) {
                                        lastX = mouse.x
                                        lastY = mouse.y
                                        if (mouse.button === Qt.LeftButton) {
                                            dashboardCanvas.selectTrackAtScreenPos(mouse.x, mouse.y)
                                        }
                                    }
                                    onPositionChanged: function(mouse) {
                                        if (mouse.buttons & Qt.RightButton || mouse.buttons & Qt.LeftButton) {
                                            var dx = mouse.x - lastX
                                            var dy = mouse.y - lastY
                                            if (Math.abs(dx) > 2 || Math.abs(dy) > 2) {
                                                dashboardCanvas.panByPixels(dx, dy)
                                                lastX = mouse.x
                                                lastY = mouse.y
                                            }
                                        }
                                    }
                                    onWheel: function(wheel) {
                                        if (wheel.angleDelta.y > 0) dashboardCanvas.zoomIn()
                                        else dashboardCanvas.zoomOut()
                                    }
                                }
                            }

                            // Canvas Overlay Toolbar
                            RowLayout {
                                anchors.top: parent.top
                                anchors.left: parent.left
                                anchors.margins: 10
                                spacing: 6

                                Repeater {
                                    model: [
                                        { label: "Zoom +", act: function() { dashboardCanvas.zoomIn() } },
                                        { label: "Zoom -", act: function() { dashboardCanvas.zoomOut() } },
                                        { label: "Fit World", act: function() { dashboardCanvas.fitToWorld() } },
                                        { label: "Reset Cam", act: function() { dashboardCanvas.resetCamera() } }
                                    ]
                                    delegate: Button {
                                        text: modelData.label
                                        implicitHeight: 26
                                        onClicked: modelData.act()
                                        background: Rectangle {
                                            color: "#CC0F172A"
                                            border.color: rootWindow.borderMuted
                                            radius: 3
                                        }
                                        contentItem: Text {
                                            text: parent.text
                                            color: rootWindow.textPrimary
                                            font.pixelSize: 10
                                            horizontalAlignment: Text.AlignHCenter
                                            verticalAlignment: Text.AlignVCenter
                                        }
                                    }
                                }
                            }
                        }

                        // Right 38%: Live Engineering Charts & Event Feed
                        ColumnLayout {
                            Layout.fillHeight: true
                            Layout.preferredWidth: 400
                            spacing: 10

                            AnalyticsChart {
                                id: dashRmseChart
                                Layout.fillWidth: true
                                Layout.preferredHeight: 210
                                chartType: "rmse"
                                title: "Position RMSE (m)"
                                Component.onCompleted: dashboard.registerChart(dashRmseChart)
                            }

                            AnalyticsChart {
                                id: dashTracksChart
                                Layout.fillWidth: true
                                Layout.preferredHeight: 200
                                chartType: "tracks"
                                title: "Active Tracks vs Truth"
                                Component.onCompleted: dashboard.registerChart(dashTracksChart)
                            }

                            Rectangle {
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                color: rootWindow.bgPanel
                                border.color: rootWindow.borderMuted

                                ColumnLayout {
                                    anchors.fill: parent
                                    anchors.margins: 10
                                    spacing: 6
                                    Text {
                                        text: "RECENT TELEMETRY & LIFECYCLE EVENTS"
                                        color: rootWindow.accentCyan
                                        font.pixelSize: 11
                                        font.bold: true
                                    }
                                    ListView {
                                        Layout.fillWidth: true
                                        Layout.fillHeight: true
                                        clip: true
                                        model: dashboard.logsList
                                        spacing: 4
                                        delegate: Rectangle {
                                            width: ListView.view.width
                                            height: 24
                                            color: rootWindow.bgCard
                                            radius: 3
                                            RowLayout {
                                                anchors.fill: parent
                                                anchors.leftMargin: 6
                                                anchors.rightMargin: 6
                                                spacing: 8
                                                Text {
                                                    text: modelData.timestamp
                                                    color: rootWindow.accentCyan
                                                    font.family: "Consolas"
                                                    font.pixelSize: 10
                                                }
                                                Text {
                                                    text: modelData.type
                                                    color: "#10B981"
                                                    font.pixelSize: 10
                                                    font.bold: true
                                                }
                                                Text {
                                                    Layout.fillWidth: true
                                                    text: modelData.message
                                                    color: rootWindow.textPrimary
                                                    font.pixelSize: 10
                                                    elide: Text.ElideRight
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // =============================================================
                // TAB 1: LIVE TRACKING (FULL INTERACTIVE WORKSPACE)
                // =============================================================
                Item {
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 8

                        // Layer Visibility & Camera Control Bar
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 42
                            color: rootWindow.bgPanel
                            border.color: rootWindow.borderMuted

                            RowLayout {
                                anchors.fill: parent
                                anchors.leftMargin: 10
                                anchors.rightMargin: 10
                                spacing: 6

                                Text { text: "LAYERS:"; color: rootWindow.accentCyan; font.pixelSize: 10; font.bold: true }

                                Repeater {
                                    model: [
                                        { label: "Grid", prop: "showGrid" },
                                        { label: "Coverage", prop: "showRadarCoverage" },
                                        { label: "Truth", prop: "showGroundTruth" },
                                        { label: "Radar Obs", prop: "showRawRadar" },
                                        { label: "GNSS Obs", prop: "showRawGnss" },
                                        { label: "95% Ellipses", prop: "showUncertaintyEllipses" },
                                        { label: "Predict", prop: "showPredictions" },
                                        { label: "Sweep", prop: "showDecorativeSweep" }
                                    ]
                                    delegate: Rectangle {
                                        property bool isOn: mainTrackingCanvas[modelData.prop]
                                        implicitWidth: pillRow.implicitWidth + 14
                                        implicitHeight: 26
                                        radius: 4
                                        color: isOn ? "#122B4A" : rootWindow.bgCard
                                        border.color: isOn ? rootWindow.accentCyan : rootWindow.borderMuted

                                        RowLayout {
                                            id: pillRow
                                            anchors.centerIn: parent
                                            spacing: 5
                                            Rectangle {
                                                width: 7
                                                height: 7
                                                radius: 3.5
                                                color: parent.parent.isOn ? rootWindow.accentCyan : "#475569"
                                            }
                                            Text {
                                                text: modelData.label
                                                color: parent.parent.isOn ? rootWindow.textPrimary : rootWindow.textSecondary
                                                font.pixelSize: 10
                                                font.bold: parent.parent.isOn
                                            }
                                        }
                                        MouseArea {
                                            anchors.fill: parent
                                            cursorShape: Qt.PointingHandCursor
                                            onClicked: mainTrackingCanvas[modelData.prop] = !mainTrackingCanvas[modelData.prop]
                                        }
                                    }
                                }

                                Item { Layout.fillWidth: true }

                                Button {
                                    text: mainTrackingCanvas.followSelectedTrack ? "Unfollow" : "Follow"
                                    implicitHeight: 26
                                    onClicked: mainTrackingCanvas.followSelectedTrack = !mainTrackingCanvas.followSelectedTrack
                                    background: Rectangle {
                                        color: mainTrackingCanvas.followSelectedTrack ? "#064E3B" : rootWindow.bgCard
                                        border.color: mainTrackingCanvas.followSelectedTrack ? rootWindow.accentGreen : rootWindow.borderMuted
                                        radius: 4
                                    }
                                    contentItem: Text {
                                        text: parent.text
                                        color: rootWindow.textPrimary
                                        font.pixelSize: 10
                                        leftPadding: 8
                                        rightPadding: 8
                                        horizontalAlignment: Text.AlignHCenter
                                        verticalAlignment: Text.AlignVCenter
                                    }
                                }
                                Button {
                                    text: "Fit World"
                                    implicitHeight: 26
                                    onClicked: mainTrackingCanvas.fitToWorld()
                                    background: Rectangle { color: rootWindow.bgCard; border.color: rootWindow.borderMuted; radius: 4 }
                                    contentItem: Text { text: parent.text; color: rootWindow.textPrimary; font.pixelSize: 10; leftPadding: 8; rightPadding: 8; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                                }
                                Button {
                                    text: "Reset Cam"
                                    implicitHeight: 26
                                    onClicked: mainTrackingCanvas.resetCamera()
                                    background: Rectangle { color: rootWindow.bgCard; border.color: rootWindow.borderMuted; radius: 4 }
                                    contentItem: Text { text: parent.text; color: rootWindow.textPrimary; font.pixelSize: 10; leftPadding: 8; rightPadding: 8; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                                }
                            }
                        }

                        // Full 2D Canvas
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            color: "#060B16"
                            border.color: rootWindow.borderMuted

                            TrackingCanvas {
                                id: mainTrackingCanvas
                                anchors.fill: parent
                                anchors.margins: 1
                                Component.onCompleted: dashboard.registerCanvas(mainTrackingCanvas)

                                MouseArea {
                                    anchors.fill: parent
                                    acceptedButtons: Qt.LeftButton | Qt.RightButton
                                    property real lastX: 0
                                    property real lastY: 0
                                    onPressed: function(mouse) {
                                        lastX = mouse.x
                                        lastY = mouse.y
                                        if (mouse.button === Qt.LeftButton) {
                                            mainTrackingCanvas.selectTrackAtScreenPos(mouse.x, mouse.y)
                                        }
                                    }
                                    onPositionChanged: function(mouse) {
                                        if (mouse.buttons & Qt.RightButton || mouse.buttons & Qt.LeftButton) {
                                            var dx = mouse.x - lastX
                                            var dy = mouse.y - lastY
                                            if (Math.abs(dx) > 2 || Math.abs(dy) > 2) {
                                                mainTrackingCanvas.panByPixels(dx, dy)
                                                lastX = mouse.x
                                                lastY = mouse.y
                                            }
                                        }
                                    }
                                    onWheel: function(wheel) {
                                        if (wheel.angleDelta.y > 0) mainTrackingCanvas.zoomIn()
                                        else mainTrackingCanvas.zoomOut()
                                    }
                                }
                            }
                        }
                    }
                }

                // =============================================================
                // TAB 2: SENSORS HEALTH & CONTROL
                // =============================================================
                Item {
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 16
                        spacing: 12

                        Text {
                            text: "CONFIGURED SENSOR SUITE & LIVE DIAGNOSTICS"
                            color: rootWindow.accentCyan
                            font.pixelSize: 14
                            font.bold: true
                        }
                        Text {
                            text: "Toggle individual Radar or GNSS telemetry channels online/offline during simulation to test filter coasting and failover."
                            color: rootWindow.textSecondary
                            font.pixelSize: 12
                        }

                        ListView {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            spacing: 10
                            model: dashboard.sensorsList
                            delegate: Rectangle {
                                width: ListView.view.width
                                height: 108
                                radius: 6
                                color: rootWindow.bgPanel
                                border.color: modelData.operational ? rootWindow.borderMuted : rootWindow.accentRed

                                RowLayout {
                                    anchors.fill: parent
                                    anchors.margins: 14
                                    spacing: 12

                                    ColumnLayout {
                                        Layout.preferredWidth: 200
                                        spacing: 4
                                        Text {
                                            text: modelData.id.toUpperCase() + "  [" + modelData.type + "]"
                                            color: modelData.type === "Radar" ? rootWindow.accentCyan : rootWindow.accentAmber
                                            font.pixelSize: 13
                                            font.bold: true
                                        }
                                        Text {
                                            text: "Station Position: " + modelData.position
                                            color: rootWindow.textSecondary
                                            font.pixelSize: 11
                                        }
                                        Text {
                                            text: "Sampling Rate: " + modelData.rateHz.toFixed(1) + " Hz"
                                            color: rootWindow.textSecondary
                                            font.pixelSize: 11
                                        }
                                    }

                                    Repeater {
                                        model: [
                                            { lbl: "SCANS EXECUTED", val: modelData.scans },
                                            { lbl: "DETECTIONS SENT", val: modelData.generated },
                                            { lbl: "MISSED / DROPOUTS", val: modelData.failures },
                                            { lbl: "FALSE ALARMS", val: modelData.falseAlarms }
                                        ]
                                        delegate: Rectangle {
                                            Layout.preferredWidth: 120
                                            Layout.preferredHeight: 62
                                            radius: 4
                                            color: rootWindow.bgCard
                                            border.color: rootWindow.borderMuted
                                            ColumnLayout {
                                                anchors.centerIn: parent
                                                spacing: 2
                                                Text { text: modelData.lbl; color: rootWindow.textSecondary; font.pixelSize: 9; font.bold: true }
                                                Text { text: modelData.val.toString(); color: rootWindow.textPrimary; font.family: "Consolas"; font.pixelSize: 14; font.bold: true }
                                            }
                                        }
                                    }

                                    Item { Layout.fillWidth: true }

                                    Button {
                                        text: modelData.operational ? "ONLINE (Click to Disable)" : "OFFLINE (Click to Enable)"
                                        implicitWidth: 175
                                        implicitHeight: 38
                                        onClicked: dashboard.toggleSensorOperational(modelData.id, !modelData.operational)
                                        background: Rectangle {
                                            radius: 4
                                            color: modelData.operational ? "#064E3B" : "#450A0A"
                                            border.color: modelData.operational ? "#10B981" : "#EF4444"
                                        }
                                        contentItem: Text {
                                            text: parent.text
                                            color: modelData.operational ? "#10B981" : "#FCA5A5"
                                            font.pixelSize: 11
                                            font.bold: true
                                            horizontalAlignment: Text.AlignHCenter
                                            verticalAlignment: Text.AlignVCenter
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // =============================================================
                // TAB 3: SCENARIOS & SESSION I/O
                // =============================================================
                Item {
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 16
                        spacing: 12

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 12
                            Text {
                                text: "SCENARIO ENGINE & DETERMINISTIC SESSION RECORDER"
                                color: rootWindow.accentCyan
                                font.pixelSize: 14
                                font.bold: true
                            }
                            Item { Layout.fillWidth: true }
                            Button {
                                text: "Export Session Bundle (JSON + CSV)"
                                onClicked: dashboard.exportSessionBundle("exports/latest_session")
                            }
                            Button {
                                text: "Replay Session (Hungarian)"
                                onClicked: dashboard.replayExportedSession("exports/latest_session/session.json", "Hungarian")
                            }
                            Button {
                                text: "Replay Session (NearestNeighbor)"
                                onClicked: dashboard.replayExportedSession("exports/latest_session/session.json", "NearestNeighbor")
                            }
                        }

                        GridView {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            cellWidth: 420
                            cellHeight: 112
                            clip: true
                            model: dashboard.scenariosList
                            delegate: Rectangle {
                                width: 408
                                height: 100
                                radius: 6
                                color: rootWindow.bgPanel
                                border.color: rootWindow.borderMuted

                                RowLayout {
                                    anchors.fill: parent
                                    anchors.margins: 12
                                    spacing: 12

                                    ColumnLayout {
                                        Layout.fillWidth: true
                                        spacing: 4
                                        Text {
                                            text: modelData.code + " — " + modelData.title
                                            color: rootWindow.accentCyan
                                            font.pixelSize: 13
                                            font.bold: true
                                        }
                                        Text {
                                            Layout.fillWidth: true
                                            text: modelData.summary
                                            color: rootWindow.textSecondary
                                            font.pixelSize: 11
                                            wrapMode: Text.WordWrap
                                        }
                                        Text {
                                            text: "File: scenarios/" + modelData.filename
                                            color: "#64748B"
                                            font.family: "Consolas"
                                            font.pixelSize: 10
                                        }
                                    }

                                    Button {
                                        text: "Load"
                                        implicitWidth: 74
                                        implicitHeight: 34
                                        onClicked: dashboard.loadScenarioFile(modelData.filename, -1)
                                    }
                                }
                            }
                        }
                    }
                }

                // =============================================================
                // TAB 4: LIVE ANALYTICS & FUSION COMPARISON
                // =============================================================
                Item {
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 12
                        spacing: 10

                        // Top Comparison Control & Summary Bar
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 62
                            color: rootWindow.bgPanel
                            border.color: rootWindow.borderMuted
                            radius: 4

                            RowLayout {
                                anchors.fill: parent
                                anchors.margins: 12
                                spacing: 18

                                Button {
                                    text: "Run Matched Fusion Comparison"
                                    implicitHeight: 36
                                    onClicked: dashboard.runMatchedFusionComparison()
                                    background: Rectangle {
                                        color: "#0E7490"
                                        border.color: rootWindow.accentCyan
                                        radius: 4
                                    }
                                    contentItem: Text {
                                        text: parent.text
                                        color: "#FFFFFF"
                                        font.pixelSize: 12
                                        font.bold: true
                                        horizontalAlignment: Text.AlignHCenter
                                        verticalAlignment: Text.AlignVCenter
                                    }
                                }

                                Repeater {
                                    model: [
                                        { lbl: "RADAR-ONLY (EKF) RMSE", val: dashboard.radarOnlyPosRmse.toFixed(2) + " m  |  " + dashboard.radarOnlyVelRmse.toFixed(2) + " m/s", col: "#00E5FF" },
                                        { lbl: "GNSS-ONLY (KF) RMSE", val: dashboard.gnssOnlyPosRmse.toFixed(2) + " m  |  " + dashboard.gnssOnlyVelRmse.toFixed(2) + " m/s", col: "#FFB300" },
                                        { lbl: "RADAR + GNSS FUSION RMSE", val: dashboard.fusedPosRmse.toFixed(2) + " m  |  " + dashboard.fusedVelRmse.toFixed(2) + " m/s", col: "#10B981" }
                                    ]
                                    delegate: Rectangle {
                                        Layout.preferredWidth: 235
                                        Layout.preferredHeight: 42
                                        color: rootWindow.bgCard
                                        border.color: rootWindow.borderMuted
                                        radius: 4
                                        ColumnLayout {
                                            anchors.centerIn: parent
                                            spacing: 1
                                            Text { text: modelData.lbl; color: rootWindow.textSecondary; font.pixelSize: 9; font.bold: true }
                                            Text { text: modelData.val; color: modelData.col; font.family: "Consolas"; font.pixelSize: 12; font.bold: true }
                                        }
                                    }
                                }

                                Item { Layout.fillWidth: true }
                            }
                        }

                        // 2x2 Chart Grid
                        GridLayout {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            columns: 2
                            rowSpacing: 10
                            columnSpacing: 10

                            AnalyticsChart {
                                id: chartFusionComp
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                chartType: "fusion_comparison"
                                title: "Matched Comparison RMSE (m)"
                                Component.onCompleted: dashboard.registerChart(chartFusionComp)
                            }

                            AnalyticsChart {
                                id: chartHistogram
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                chartType: "histogram"
                                title: "Position Estimation Error Distribution (Counts per Bin)"
                                Component.onCompleted: dashboard.registerChart(chartHistogram)
                            }

                            AnalyticsChart {
                                id: chartRmseFull
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                chartType: "rmse"
                                title: "Live Session Position RMSE (m)"
                                Component.onCompleted: dashboard.registerChart(chartRmseFull)
                            }

                            AnalyticsChart {
                                id: chartLatencyFull
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                chartType: "latency"
                                title: "Real-Time Fusion Engine Step Latency (μs)"
                                Component.onCompleted: dashboard.registerChart(chartLatencyFull)
                            }
                        }
                    }
                }

                // =============================================================
                // TAB 5: SYSTEM LOGS
                // =============================================================
                Item {
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 16
                        spacing: 10

                        Text {
                            text: "CHRONOLOGICAL TRACKER & SENSOR EVENT LOG"
                            color: rootWindow.accentCyan
                            font.pixelSize: 14
                            font.bold: true
                        }

                        Rectangle {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            color: rootWindow.bgPanel
                            border.color: rootWindow.borderMuted

                            ListView {
                                anchors.fill: parent
                                anchors.margins: 8
                                clip: true
                                spacing: 4
                                model: dashboard.logsList
                                delegate: Rectangle {
                                    width: ListView.view.width
                                    height: 28
                                    color: rootWindow.bgCard
                                    radius: 3
                                    RowLayout {
                                        anchors.fill: parent
                                        anchors.leftMargin: 10
                                        anchors.rightMargin: 10
                                        spacing: 14
                                        Text {
                                            Layout.preferredWidth: 70
                                            text: modelData.timestamp
                                            color: rootWindow.accentCyan
                                            font.family: "Consolas"
                                            font.pixelSize: 11
                                        }
                                        Text {
                                            Layout.preferredWidth: 160
                                            text: modelData.type
                                            color: "#10B981"
                                            font.family: "Consolas"
                                            font.pixelSize: 11
                                            font.bold: true
                                        }
                                        Text {
                                            Layout.fillWidth: true
                                            text: modelData.message
                                            color: rootWindow.textPrimary
                                            font.pixelSize: 11
                                            elide: Text.ElideRight
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // =============================================================
                // TAB 6: FILTER & TRACKER SETTINGS
                // =============================================================
                Item {
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 24
                        spacing: 16

                        Text {
                            text: "ESTIMATION, GATING & DATA ASSOCIATION PARAMETERS"
                            color: rootWindow.accentCyan
                            font.pixelSize: 15
                            font.bold: true
                        }

                        Rectangle {
                            Layout.preferredWidth: 560
                            Layout.preferredHeight: 280
                            color: rootWindow.bgPanel
                            border.color: rootWindow.borderMuted
                            radius: 6

                            GridLayout {
                                anchors.fill: parent
                                anchors.margins: 20
                                columns: 2
                                rowSpacing: 14
                                columnSpacing: 20

                                Text { text: "Confirmation Hits (Tentative → Confirmed):"; color: rootWindow.textPrimary; font.pixelSize: 12 }
                                SpinBox { id: spinConfirmHits; from: 1; to: 10; value: 3 }

                                Text { text: "2D Chi-Square Gate Threshold (γ):"; color: rootWindow.textPrimary; font.pixelSize: 12 }
                                ComboBox {
                                    id: comboGate
                                    Layout.preferredWidth: 220
                                    model: ["5.991 (95% Confidence)", "9.210 (99% Confidence)", "11.345 (99.7% Confidence)", "13.816 (99.9% Confidence)"]
                                    currentIndex: 1
                                }

                                Text { text: "Process Noise Accel Std σ_a (m/s²):"; color: rootWindow.textPrimary; font.pixelSize: 12 }
                                ComboBox {
                                    id: comboProcessNoise
                                    Layout.preferredWidth: 220
                                    model: ["0.5 m/s² (Low Maneuver)", "1.5 m/s² (Nominal)", "2.5 m/s² (Agile)", "4.0 m/s² (High Maneuver)"]
                                    currentIndex: 1
                                }

                                Text { text: "Data Association Algorithm:"; color: rootWindow.textPrimary; font.pixelSize: 12 }
                                ComboBox {
                                    id: comboAssociator
                                    Layout.preferredWidth: 220
                                    model: ["Hungarian", "NearestNeighbor"]
                                    currentIndex: 0
                                }

                                Item {}
                                Button {
                                    text: "Apply Parameters & Restart Scenario"
                                    onClicked: {
                                        var gates = [5.99146, 9.21034, 11.34487, 13.81551]
                                        var noises = [0.5, 1.5, 2.5, 4.0]
                                        dashboard.applyTrackerSettings(
                                            spinConfirmHits.value,
                                            gates[comboGate.currentIndex],
                                            noises[comboProcessNoise.currentIndex],
                                            comboAssociator.currentText
                                        )
                                    }
                                }
                            }
                        }

                        Item { Layout.fillHeight: true }
                    }
                }
            }

            // -----------------------------------------------------------------
            // RIGHT INFORMATION PANEL (SELECTED TRACK INSPECTOR)
            // -----------------------------------------------------------------
            Rectangle {
                Layout.preferredWidth: 300
                Layout.fillHeight: true
                color: rootWindow.bgPanel
                border.color: rootWindow.borderMuted
                border.width: 1

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 8

                    Text {
                        text: "SELECTED TRACK INSPECTOR"
                        color: rootWindow.accentCyan
                        font.pixelSize: 11
                        font.bold: true
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 335
                        radius: 4
                        color: rootWindow.bgCard
                        border.color: rootWindow.borderMuted

                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: 10
                            spacing: 6

                            RowLayout {
                                Layout.fillWidth: true
                                Text {
                                    text: dashboard.hasSelectedTrack ? ("TRACK #" + dashboard.selectedTrackId) : "NO TRACK SELECTED"
                                    color: rootWindow.textPrimary
                                    font.family: "Consolas"
                                    font.pixelSize: 14
                                    font.bold: true
                                }
                                Item { Layout.fillWidth: true }
                                Rectangle {
                                    visible: dashboard.hasSelectedTrack
                                    width: 82
                                    height: 20
                                    radius: 3
                                    color: dashboard.selectedTrackStatus === "CONFIRMED" ? "#064E3B" : "#422006"
                                    border.color: dashboard.selectedTrackStatus === "CONFIRMED" ? "#10B981" : "#FACC15"
                                    Text {
                                        anchors.centerIn: parent
                                        text: dashboard.selectedTrackStatus
                                        color: dashboard.selectedTrackStatus === "CONFIRMED" ? "#10B981" : "#FACC15"
                                        font.pixelSize: 9
                                        font.bold: true
                                    }
                                }
                            }

                            Rectangle { Layout.fillWidth: true; height: 1; color: rootWindow.borderMuted }

                            Repeater {
                                model: [
                                    { k: "Position (East, North)", v: dashboard.selectedTrackPosition },
                                    { k: "Velocity (Vx, Vy)", v: dashboard.selectedTrackVelocity },
                                    { k: "Speed & Heading", v: dashboard.selectedTrackSpeed },
                                    { k: "Position Covariance", v: dashboard.selectedTrackCovariance },
                                    { k: "Last Innovation (NIS)", v: dashboard.selectedTrackNis },
                                    { k: "SPRT Existence Score", v: dashboard.selectedTrackExistenceProb },
                                    { k: "Last Update Time", v: dashboard.selectedTrackLastUpdate },
                                    { k: "Track Age", v: dashboard.selectedTrackAge },
                                    { k: "Updates (Hits / Misses)", v: dashboard.selectedTrackHits + " hits / " + dashboard.selectedTrackMisses + " misses" },
                                    { k: "Contributing Sensors", v: dashboard.selectedTrackRecentSensors }
                                ]
                                delegate: ColumnLayout {
                                    Layout.fillWidth: true
                                    spacing: 1
                                    Text { text: modelData.k; color: rootWindow.textSecondary; font.pixelSize: 9 }
                                    Text {
                                        Layout.fillWidth: true
                                        text: modelData.v
                                        color: rootWindow.textPrimary
                                        font.family: "Consolas"
                                        font.pixelSize: 10
                                        elide: Text.ElideRight
                                    }
                                }
                            }
                        }
                    }

                    Text {
                        text: "ACTIVE TRACK TABLE (CLICK TO SELECT)"
                        color: rootWindow.textSecondary
                        font.pixelSize: 10
                        font.bold: true
                        Layout.topMargin: 4
                    }

                    ListView {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        clip: true
                        spacing: 4
                        model: dashboard.tracksList
                        delegate: Rectangle {
                            width: ListView.view.width
                            height: 30
                            radius: 3
                            color: dashboard.selectedTrackId === modelData.id ? "#172B4D" : rootWindow.bgCard
                            border.color: dashboard.selectedTrackId === modelData.id ? rootWindow.accentCyan : rootWindow.borderMuted

                            RowLayout {
                                anchors.fill: parent
                                anchors.leftMargin: 8
                                anchors.rightMargin: 8
                                spacing: 6
                                Text {
                                    text: "TRK-" + modelData.id
                                    color: rootWindow.accentCyan
                                    font.family: "Consolas"
                                    font.pixelSize: 11
                                    font.bold: true
                                }
                                Text {
                                    text: modelData.status
                                    color: modelData.status === "Confirmed" ? "#10B981" : "#FACC15"
                                    font.pixelSize: 10
                                }
                                Item { Layout.fillWidth: true }
                                Text {
                                    text: modelData.speed
                                    color: rootWindow.textPrimary
                                    font.family: "Consolas"
                                    font.pixelSize: 10
                                }
                            }

                            MouseArea {
                                anchors.fill: parent
                                cursorShape: Qt.PointingHandCursor
                                onClicked: dashboard.selectedTrackId = modelData.id
                            }
                        }
                    }
                }
            }
        }

        // =====================================================================
        // BOTTOM TIMELINE & PLAYBACK CONTROLS
        // =====================================================================
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 62
            color: rootWindow.bgPanel
            border.color: rootWindow.borderMuted
            border.width: 1

            ColumnLayout {
                anchors.fill: parent
                anchors.leftMargin: 16
                anchors.rightMargin: 16
                anchors.topMargin: 6
                anchors.bottomMargin: 6
                spacing: 4

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 10

                    Button {
                        text: dashboard.isPaused ? "▶ Play" : "⏸ Pause"
                        implicitWidth: 86
                        implicitHeight: 28
                        onClicked: dashboard.togglePlayPause()
                    }
                    Button {
                        text: "⏭ Step +dt"
                        implicitWidth: 86
                        implicitHeight: 28
                        onClicked: dashboard.stepForward()
                    }
                    Button {
                        text: "↺ Restart"
                        implicitWidth: 82
                        implicitHeight: 28
                        onClicked: dashboard.restart()
                    }

                    Text { text: "Speed:"; color: rootWindow.textSecondary; font.pixelSize: 11 }
                    Repeater {
                        model: [0.5, 1.0, 2.0, 5.0, 10.0]
                        delegate: Button {
                            text: modelData + "x"
                            implicitWidth: 44
                            implicitHeight: 26
                            onClicked: dashboard.playbackSpeed = modelData
                        }
                    }

                    // Timeline Progress Bar with Event Markers
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 16
                        radius: 4
                        color: rootWindow.bgCard
                        border.color: rootWindow.borderMuted

                        Rectangle {
                            width: parent.width * Math.min(1.0, dashboard.simulationTime / Math.max(0.1, dashboard.scenarioDuration))
                            height: parent.height
                            radius: 4
                            color: "#0284C7"
                        }

                        Repeater {
                            model: dashboard.timelineMarkers
                            delegate: Rectangle {
                                x: parent.width * modelData.ratio
                                y: 1
                                width: 2
                                height: parent.height - 2
                                color: modelData.type === "SensorOutageStarted" ? "#EF4444" : "#10B981"
                            }
                        }
                    }

                    Text {
                        text: dashboard.simulationTime.toFixed(1) + "s / " + dashboard.scenarioDuration.toFixed(1) + "s"
                        color: rootWindow.textPrimary
                        font.family: "Consolas"
                        font.pixelSize: 11
                    }
                }

                Text {
                    Layout.fillWidth: true
                    text: dashboard.statusBannerMessage
                    color: rootWindow.textSecondary
                    font.pixelSize: 10
                    elide: Text.ElideRight
                }
            }
        }
    }
}
