# AegisTrack Product & Engineering Roadmap

## Completed Milestones (v1.0.0 Release)

- [x] **Phase 1: Core Mathematical & Domain Foundation**
  - Local 2D ENU Cartesian coordinate system (`[px, py, vx, vy]^T`).
  - Wrap-safe angle arithmetic (`wrapAnglePi`, `angleDifference`, `polarToCartesian`, `cartesianToPolar`).
  - Chi-square gating thresholds ($\chi^2_{df=1..4}$ at 90%, 95%, 99%, 99.9%) and 2D covariance error ellipse eigendecomposition.
  - Exact $O(N^3)$ Jonker-Volgenant / Kuhn-Munkres Hungarian assignment solver with per-track gated dummy variables.
- [x] **Phase 2: Multi-Model Physical Simulation & Asynchronous Sensor Suite**
  - Composable `IMotionModel` hierarchy (`ConstantVelocityModel`, `ConstantAccelerationModel`, `CoordinatedTurnModel`) with scheduled time-window maneuvers.
  - `RadarSensor` with polar $[r, \theta]^T$ measurement generation, FOV sector & range limits, circular occlusion shadow zones, scheduled outages, and Poisson false-alarm clutter.
  - `GnssSensor` with Cartesian $[p_x, p_y]^T$ telemetry, configurable bias + random-walk drift, packet loss, and latency injection.
  - `SensorScheduler` supporting both `DropStale` and `BoundedReorderBuffer` out-of-sequence measurement (OOSM) policies.
- [x] **Phase 3: State Estimation, Data Association & Track Management**
  - Linear `KalmanFilter` and nonlinear `ExtendedKalmanFilter` using `Eigen::LLT`/`LDLT` Cholesky linear solves and Joseph stabilized covariance updates.
  - `NearestNeighborAssociator` and `HungarianAssociator` with Mahalanobis $\chi^2$ gating.
  - Time-aware `TrackManager` (`Tentative` $\rightarrow$ `Confirmed` $\rightarrow$ `Coasting` $\rightarrow$ `Deleted`) and Sequential Probability Ratio Test (SPRT) log-likelihood existence scoring.
- [x] **Phase 4: Evaluation, Serialization & Deterministic Replay**
  - `MetricsCollector` computing real-time Position/Velocity RMSE, raw observation RMSE, error histograms, and step latency.
  - Matched ground-truth `SimulationEngine::runFusionComparison` comparing Radar-Only (EKF) vs GNSS-Only (KF) vs Multi-Sensor Radar+GNSS Fusion.
  - `SessionRecorder` exporting/importing versioned `session.json` and CSV tables (`ground_truth.csv`, `measurements.csv`, `tracks.csv`, `metrics.csv`, `events.csv`) plus deterministic measurement replay across different associators.
- [x] **Phase 5: Headless CLI & Qt 6 Quick Desktop Workstation**
  - `aegistrack-cli` headless runner and comparison benchmark tool.
  - `aegistrack-desktop` 7-view dark engineering workstation (`Dashboard`, `Live Tracking`, `Sensors`, `Scenarios & I/O`, `Live Analytics`, `System Logs`, `Filter Settings`).

---

## Future Engineering Roadmap (v1.1 – v2.0)

### v1.1 — Interacting Multiple Model (IMM) & Unscented Kalman Filter (UKF)
- Add `UnscentedKalmanFilter` (sigma-point propagation) to reduce linearization bias on high-range/high-bearing-uncertainty radar scans.
- Implement a 3-mode **Interacting Multiple Model (IMM)** estimator blending `ConstantVelocity`, `ConstantAcceleration`, and `CoordinatedTurn` filters with a Markov transition probability matrix for rapid maneuver adaptation.

### v1.2 — Joint Probabilistic Data Association (JPDA) & Multi-Hypothesis Tracking (MHT)
- Implement `JpdaAssociator` for coalescing-resistant state updates in extreme clutter and crossing-target scenarios.
- Introduce track-oriented `MhtAssociator` with $N$-scan pruning for delayed association decisions.

### v1.3 — Retrodiction OOSM & 3D State Extension
- Upgrade OOSM handling from bounded reorder buffers to full Bar-Shalom single-step retrodiction for delayed sensor packets exceeding the reorder window.
- Extend state vector to 3D ENU (`[px, py, pz, vx, vy, vz]^T`) with 3D spherical radar measurements (`[range, azimuth, elevation, range_rate]^T`).

### v2.0 — Live Hardware & Network Telemetry Adapters
- Implement UDP/Protobuf and Serial NMEA-0183 / ASTERIX CAT-048 `ISensor` adapters to ingest live field radar and GNSS receiver streams without modifying `FusionEngine` or `TrackManager`.
