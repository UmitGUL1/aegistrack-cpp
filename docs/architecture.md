# AegisTrack Architecture Specification

## 1. Architectural Overview

**AegisTrack** is a modular, real-time 2D multi-sensor fusion and multi-object tracking system implemented in Modern C++20. The architecture enforces a strict separation between:

1. **Physical World Simulation (`aegistrack::simulation`)** — Ground-truth object kinematics, trajectory history, and scenario scheduling.
2. **Synthetic Sensor Models (`aegistrack::sensors`)** — Radar (nonlinear polar range-bearing with false alarms) and GNSS (linear Cartesian telemetry with packet dropouts and latency), scheduled asynchronously.
3. **Estimation, Association & Fusion Core (`aegistrack::filtering`, `aegistrack::association`, `aegistrack::tracking`, `aegistrack::fusion`)** — Linear Kalman Filter (KF), Extended Kalman Filter (EKF), Mahalanobis-gated Nearest Neighbor and Hungarian assignment, and time-aware track lifecycle management.
4. **Telemetry, Recording & Evaluation (`aegistrack::metrics`, `aegistrack::io`, `aegistrack::events`)** — Ground-truth-to-track evaluation (RMSE, error distributions, latency profiling) and deterministic session recording/replay.
5. **Presentation Layers (`apps/cli`, `apps/desktop`)** — Headless command-line runner and Qt 6 Quick / QML engineering dashboard communicating via immutable thread-safe state snapshots.

> [!IMPORTANT]
> The core library (`aegistrack_core`) has **zero dependency on Qt**. Ground truth (`ObjectId`, `SimulatedObject`) is never accessible to `IDataAssociator`, `TrackManager`, or `IStateEstimator`; it is strictly isolated to `SimulationEngine`, `ISensor::generateMeasurements`, and `MetricsCollector`.

---

## 2. High-Level Component Diagram

```mermaid
flowchart TD
    subgraph ConfigIO["Configuration & Session I/O"]
        CFG["ConfigurationManager (JSON)"]
        REC["SessionRecorder / SessionReplayer"]
        CSV["CsvExporter"]
    end

    subgraph SimLayer["Simulation Subsystem (Ground Truth)"]
        CLK["SimulationClock"]
        RNG["DeterministicRng (mt19937_64)"]
        OBJ["SimulatedObject + IMotionModel"]
        SIM["SimulationEngine"]
    end

    subgraph SensorLayer["Asynchronous Sensor Subsystem"]
        RAD["RadarSensor (Range, Bearing, Clutter)"]
        GNS["GnssSensor (Position, Latency, Outage)"]
        SCH["SensorScheduler (OOSM Buffer/Drop)"]
    end

    subgraph FusionCore["Tracking & Sensor Fusion Engine"]
        FUS["FusionEngine"]
        TRK["TrackManager (Lifecycle State Machine)"]
        ASC["IDataAssociator (NN / Hungarian + Chi2 Gate)"]
        EST["IStateEstimator (KF / EKF)"]
    end

    subgraph EvalLayer["Observability & Metrics"]
        BUS["EventBus"]
        MET["MetricsCollector"]
        PRF["InternalProfiler"]
    end

    subgraph Apps["Application Frontends"]
        CLI["aegistrack-cli (Headless)"]
        GUI["aegistrack-desktop (Qt 6 Quick / QML)"]
    end

    CFG --> SIM
    CLK --> SIM
    RNG --> SIM
    SIM --> OBJ
    OBJ -->|Ground Truth States| RAD
    OBJ -->|Ground Truth States| GNS
    RAD --> SCH
    GNS --> SCH
    SCH -->|Timestamped Measurements| FUS
    FUS --> TRK
    TRK --> ASC
    TRK --> EST
    FUS --> BUS
    SIM -->|Ground Truth Reference| MET
    FUS -->|Estimated Tracks| MET
    FUS --> PRF
    MET --> REC
    MET --> CSV
    SIM --> CLI
    SIM -->|Immutable Snapshot| GUI
```

---

## 3. UML Class Diagram (Core Domain)

```mermaid
classDiagram
    class ObjectState {
        +ObjectId id
        +double px
        +double py
        +double vx
        +double vy
        +optional~double~ ax
        +optional~double~ ay
        +TimestampSec timestamp
        +position() Vector2d
        +velocity() Vector2d
        +speed() double
    }

    class IMotionModel {
        <<interface>>
        +propagate(ObjectState state, double dt)* ObjectState
        +type()* MotionModelType
    }

    class ConstantVelocityMotionModel {
        +propagate(ObjectState state, double dt) ObjectState
    }

    class ConstantAccelerationMotionModel {
        +propagate(ObjectState state, double dt) ObjectState
    }

    class CoordinatedTurnMotionModel {
        -double turn_rate_rad_s_
        +propagate(ObjectState state, double dt) ObjectState
    }

    class SimulatedObject {
        -ObjectState state_
        -unique_ptr~IMotionModel~ motion_model_
        -vector~ObjectState~ history_
        +step(double dt)
        +state() ObjectState
    }

    class ISensor {
        <<interface>>
        +id()* SensorId
        +type()* SensorType
        +samplingInterval()* double
        +generateMeasurements(TimestampSec t, span~SimulatedObject~ objs, DeterministicRng rng)* vector~Measurement~
    }

    class RadarSensor {
        -Vector2d position_
        -double max_range_m_
        -double sigma_range_m_
        -double sigma_bearing_rad_
        +generateMeasurements(...) vector~Measurement~
    }

    class GnssSensor {
        -double sigma_pos_m_
        -double packet_loss_prob_
        +generateMeasurements(...) vector~Measurement~
    }

    class IStateEstimator {
        <<interface>>
        +predict(double dt)*
        +updateMeasurement(Measurement m)* FilterDiagnostics
        +state()* Vector4d
        +covariance()* Matrix4d
    }

    class KalmanFilter {
        #Vector4d x_
        #Matrix4d P_
        #double accel_noise_std_
        +predict(double dt)
        +updateLinear(VectorXd z, MatrixXd H, MatrixXd R) FilterDiagnostics
    }

    class ExtendedKalmanFilter {
        +updateMeasurement(Measurement m) FilterDiagnostics
        +computeRadarJacobian(Vector4d x, Vector2d radar_pos)$ Matrix2x4d
    }

    class IDataAssociator {
        <<interface>>
        +associate(span~Track~ tracks, span~Measurement~ measurements, double gate_threshold)* AssociationResult
    }

    class NearestNeighborAssociator {
        +associate(...) AssociationResult
    }

    class HungarianAssociator {
        +associate(...) AssociationResult
    }

    class Track {
        -TrackId id_
        -unique_ptr~IStateEstimator~ estimator_
        -TrackLifecycleStatus status_
        -uint32_t hit_count_
        -uint32_t miss_count_
        +predict(double dt)
        +update(Measurement m) FilterDiagnostics
    }

    class TrackManager {
        -vector~Track~ tracks_
        -unique_ptr~IDataAssociator~ associator_
        +predictAll(TimestampSec target_time)
        +processMeasurementBatch(span~Measurement~ batch, TimestampSec time)
    }

    IMotionModel <|-- ConstantVelocityMotionModel
    IMotionModel <|-- ConstantAccelerationMotionModel
    IMotionModel <|-- CoordinatedTurnMotionModel
    SimulatedObject *-- IMotionModel
    ISensor <|-- RadarSensor
    ISensor <|-- GnssSensor
    IStateEstimator <|-- KalmanFilter
    KalmanFilter <|-- ExtendedKalmanFilter
    IDataAssociator <|-- NearestNeighborAssociator
    IDataAssociator <|-- HungarianAssociator
    Track *-- IStateEstimator
    TrackManager *-- Track
    TrackManager *-- IDataAssociator
```

---

## 4. Coordinate System and Units

All physical quantities inside `aegistrack_core` use SI units in a local 2D East-North Cartesian coordinate frame:

| Quantity | Symbol | Unit | Convention |
| :--- | :--- | :--- | :--- |
| Position | $(p_x, p_y)$ | meters ($\text{m}$) | $+x$ East, $+y$ North |
| Velocity | $(v_x, v_y)$ | meters/second ($\text{m/s}$) | First derivative of $(p_x, p_y)$ |
| Acceleration | $(a_x, a_y)$ | meters/second$^2$ ($\text{m/s}^2$) | Second derivative of $(p_x, p_y)$ |
| Radar Range | $r$ | meters ($\text{m}$) | Euclidean distance from station $(x_s, y_s)$ |
| Radar Bearing | $\theta$ | radians ($\text{rad}$) | $\text{atan2}(p_y - y_s, p_x - x_s) \in (-\pi, \pi]$ (counter-clockwise from $+x$) |
| Time | $t, \Delta t$ | seconds ($\text{s}$) | Monotonic simulation clock relative to $t_0 = 0.0\text{ s}$ |

---

## 5. Sensor-to-Tracker Data Flow

```mermaid
sequenceDiagram
    participant Sim as SimulationEngine
    participant Obj as SimulatedObjects
    participant Sen as Radar / GNSS Sensors
    participant Sch as SensorScheduler
    participant Fus as FusionEngine
    participant TM as TrackManager
    participant Asc as IDataAssociator
    participant KF as ExtendedKalmanFilter

    Sim->>Obj: step(dt) -> update ground truth
    Sim->>Sen: sample due sensors at t_sim
    Sen-->>Sch: enqueue raw Measurements (capture_t, arrival_t)
    Sch->>Fus: release arrived Measurements sorted by capture_t
    Fus->>TM: processMeasurements(batch)
    TM->>KF: predict(t_meas - t_last)
    TM->>Asc: associate(active_tracks, batch, chi2_gate)
    Asc->>KF: computeInnovation(measurement)
    Asc-->>TM: matched_pairs, unmatched_tracks, unmatched_meas
    TM->>KF: updateMeasurement(matched_meas) (Joseph form)
    TM->>TM: updateLifecycle(hits, misses, elapsed_time)
    TM->>TM: initiateTentativeTracks(unmatched_meas)
```

---

## 6. Track Lifecycle State Machine

```mermaid
stateDiagram-v2
    [*] --> Tentative : Unmatched detection outside existing track gates
    Tentative --> Confirmed : consecutive_hits >= N_confirm
    Tentative --> Deleted : miss_count > M_tentative OR dt_unseen > T_tentative_max
    Confirmed --> Confirmed : Matched detection (reset miss_count, time_since_update = 0)
    Confirmed --> Coasting : dt_unseen > T_coast_threshold
    Coasting --> Confirmed : Matched detection within expanded covariance gate
    Coasting --> Deleted : dt_unseen > T_delete_max OR miss_count > M_max
    Deleted --> [*] : Pruned from active track list (archived to history)
```

---

## 7. Concurrency and Threading Model

AegisTrack supports two execution modes:

1. **Deterministic Single-Threaded Mode (CLI, Unit/Integration Tests, Batch Replay)**:
   - Executes `SimulationEngine::step()` synchronously on the calling thread.
   - Guarantees bit-identical numerical results for a fixed random seed and scenario configuration.
2. **Asynchronous Real-Time Workstation Mode (Desktop GUI)**:
   - **Simulation & Fusion Worker (`std::jthread`)**: Runs the deterministic step loop paced by a high-resolution timer scaled by `playback_speed`, respecting pause/step-forward commands via a condition variable and `std::stop_token`.
   - **Immutable Snapshot Bridge**: After each simulation tick, the worker publishes an immutable `SimulationSnapshot` (`std::shared_ptr<const SimulationSnapshot>`) protected by a lightweight mutex/atomic exchange.
   - **Qt GUI / Render Thread**: Reads the latest `SimulationSnapshot` at display refresh rate (60 Hz) without blocking the fusion worker and without ever mutating engine state directly.
