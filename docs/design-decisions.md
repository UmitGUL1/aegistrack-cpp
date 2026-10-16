# Architectural & Design Decisions — AegisTrack

## 1. Software Design Patterns Employed

AegisTrack applies design patterns strictly where behavioral variation, decoupling, or thread safety justifies them, favoring composition over deep inheritance hierarchies.

### 1.1 Strategy Pattern (`IMotionModel`, `IDataAssociator`, `IStateEstimator`)

- **Components**:
  - [`IMotionModel`](../include/aegistrack/simulation/motion_model.hpp): `ConstantVelocityMotionModel`, `ConstantAccelerationMotionModel`, `CoordinatedTurnMotionModel`.
  - [`IDataAssociator`](../include/aegistrack/association/idata_associator.hpp): `NearestNeighborAssociator`, `HungarianAssociator`.
  - [`IStateEstimator`](../include/aegistrack/filtering/istate_estimator.hpp): `KalmanFilter`, `ExtendedKalmanFilter`.
- **Problem Solved**:
  Different simulated objects follow distinct kinematics (straight flight, linear boost, circular turns), and the tracking engine must support swapping between greedy Nearest Neighbor and global Hungarian assignment at runtime or during recorded session replay without modifying `TrackManager`.
- **Why Better Than Alternatives**:
  Avoids brittle `switch (algorithm_type)` branches scattered across `SimulatedObject` and `TrackManager`, adheres to the Open-Closed Principle, and enables unit testing each algorithm in isolation.
- **Tradeoffs**:
  Introduces one virtual dispatch per object/scan step (`~2 ns`), which is negligible compared to matrix factorizations (`~200–800 ns`).

### 1.2 Factory Pattern (`SensorFactory`)

- **Component**: [`SensorFactory`](../include/aegistrack/sensors/sensor_factory.hpp)
- **Problem Solved**:
  Encapsulates the construction and validation of polymorphic [`ISensor`](../include/aegistrack/sensors/isensor.hpp) implementations (`RadarSensor`, `GnssSensor`) from deserialized JSON configuration structs.
- **Why Better Than Alternatives**:
  Decouples `SimulationEngine` and `ConfigurationManager` from concrete sensor construction details and centralizes validation before a sensor is registered with `SensorScheduler`.
- **Tradeoffs**:
  Adds a thin creation layer that must be updated when a new sensor family (e.g., passive acoustic or camera bearing sensor) is added.

### 1.3 Observer / Event Bus Pattern (`EventBus`)

- **Component**: [`EventBus`](../include/aegistrack/events/event_bus.hpp)
- **Problem Solved**:
  `TrackManager` and `SensorScheduler` generate lifecycle transitions (`TrackCreated`, `TrackConfirmed`, `TrackCoasting`, `TrackRecovered`, `TrackDeleted`, `SensorOutageStarted`, `StaleMeasurementDropped`) that must be consumed by the GUI timeline, the system log view, and the `SessionRecorder` without coupling core tracking classes to UI or file I/O subsystems.
- **Why Better Than Alternatives**:
  Eliminates upward dependencies from `aegistrack_core` to presentation or persistence layers while providing a thread-safe bounded chronological event log.
- **Tradeoffs**:
  Synchronous callback invocation requires subscribers to remain fast; heavy consumers read from the bounded history snapshot instead.

### 1.4 Dependency Injection (Constructor & Setter Injection)

- **Components**: [`Track`](../include/aegistrack/tracking/track.hpp), [`TrackManager`](../include/aegistrack/tracking/track_manager.hpp), [`FusionEngine`](../include/aegistrack/fusion/fusion_engine.hpp)
- **Problem Solved**:
  Supplies `IStateEstimator`, `IDataAssociator`, `EventBus*`, and `InternalProfiler*` explicitly via constructors rather than global singletons.
- **Why Better Than Alternatives**:
  No Singleton pattern is used anywhere in AegisTrack. Explicit injection allows `SimulationEngine::runFusionComparison()` to instantiate three completely independent `FusionEngine` pipelines (`Radar-Only`, `GNSS-Only`, and `Fused`) concurrently in memory without any shared global state interference.
- **Tradeoffs**:
  Slightly more verbose constructor signatures.

### 1.5 Immutable Snapshot Concurrency Pattern (`SimulationSnapshot`)

- **Components**: [`SimulationSnapshot`](../include/aegistrack/simulation/simulation_engine.hpp), [`DashboardController`](../apps/desktop/dashboard_controller.hpp)
- **Problem Solved**:
  The Qt 6 Quick / QML rendering thread runs at $60\text{ Hz}$ while the simulation & fusion worker (`std::jthread`) steps at configurable playback speeds. Sharing mutable `Track` or `SimulatedObject` vectors across threads would cause data races or lock contention during GUI painting.
- **Why Better Than Alternatives**:
  After each deterministic step, `SimulationEngine` publishes a `std::shared_ptr<const SimulationSnapshot>`. The GUI thread atomically acquires a shared pointer to the latest immutable snapshot and renders lock-free.
- **Tradeoffs**:
  Copying active tracks and recent measurements into a snapshot once per step incurs a small memory copy ($\sim 10\text{–}40\text{ KB}$ for 20–80 tracks), which takes $< 15\ \mu\text{s}$ and completely eliminates UI/worker lock contention.

---

## 2. Key Engineering Tradeoffs

| Decision | Chosen Approach | Rationale |
| :--- | :--- | :--- |
| **Radar Nonlinearity** | Analytical-Jacobian EKF with Joseph covariance update | Preserves real-time sub-microsecond per-measurement update cost while accurately propagating polar range/bearing covariance and wrapping azimuth innovations across $\pm\pi$. |
| **Covariance Linear Solve** | `Eigen::LLT` with `Eigen::LDLT` + diagonal Tikhonov fallback | Avoids explicit `.inverse()`, improving numerical stability and handling near-singular innovation covariances gracefully. |
| **Multi-Sensor Association** | Sequential single-sensor scan partitioning + duplicate initiation gate | Allows asynchronous Radar ($2.5\text{ Hz}$) and GNSS ($1.0\text{ Hz}$) scans to update tracks as measurements arrive without artificial synchronization delays. |
| **Out-of-Sequence Measurements** | Configurable `DropStale` and `BoundedReorderBuffer` policies | Prevents delayed telemetry packets from corrupting filter state with negative time steps $\Delta t < 0$, without requiring full retrodiction state-history storage. |
| **Ground-Truth Isolation** | Strict visibility boundary (`ObjectId` hidden from `TrackManager` / `IDataAssociator`) | Ensures data association is 100% genuine and relies solely on kinematic predictions and Mahalanobis gating. |
