# Testing & Verification Strategy — AegisTrack

## 1. Test Suite Architecture

AegisTrack provides an automated GoogleTest suite (`aegistrack_tests`) integrated with CTest and AddressSanitizer + UndefinedBehaviorSanitizer (`sanitizers` CMake preset). Run the suite against the exact commit being evaluated; historical test logs do not verify later changes.

| Test Suite | Source File | Verification Scope |
| :--- | :--- | :--- |
| **`MathematicsTest`** | [`tests/unit/test_mathematics.cpp`](../tests/unit/test_mathematics.cpp) | State prediction, CWNA process noise covariance $\mathbf{Q}$, Joseph stabilized update, central finite-difference verification of EKF radar Jacobian ($< 10^{-7}$ error), zero-range singularity guard, $\pm\pi$ bearing normalization, PSD checks, near-singular innovation covariance regularization, and variance reduction vs raw observations. |
| **`SensorTest`** | [`tests/unit/test_sensors.cpp`](../tests/unit/test_sensors.cpp) | Polar radar geometry, polar-to-Cartesian covariance projection, GNSS telemetry filtering (`has_gnss_telemetry`), deterministic seed reproducibility, $P_D \in \{0, 1\}$ boundaries, scheduled outages, stale measurement rejection, capture-time catch-up sampling, and delayed-measurement RMSE. |
| **`AssociationTest`** | [`tests/unit/test_association.cpp`](../tests/unit/test_association.cpp) | $1\times 1$ and $2\times 2$ matching, $\chi^2$ Mahalanobis gating rejection, missing detections, unmatched detections, covariance magnitude sensitivity, and global Hungarian optimality vs greedy Nearest Neighbor under measurement contention. |
| **`TrackManagementTest`** | [`tests/unit/test_track_management.cpp`](../tests/unit/test_track_management.cpp) | Full track lifecycle state machine (`Tentative -> Confirmed -> Coasting -> Confirmed (Recovery) -> Deleted`), clutter suppression via tentative timeout, and duplicate track initiation suppression across simultaneous multi-sensor scans. |
| **`ScenarioAndArchitectureTest`** | [`tests/unit/test_scenarios_and_architecture.cpp`](../tests/unit/test_scenarios_and_architecture.cpp) | JSON syntax & semantic validation, negative variance rejection, bit-identical deterministic seed execution, seed sensitivity, polymorphic `ISensor` and `IDataAssociator` substitution, `EventBus` pub/sub lifecycle, and loading of all 10 bundled scenarios (`A` through `J`). |
| **`IntegrationPipelineTest`** | [`tests/integration/test_integration_pipeline.cpp`](../tests/integration/test_integration_pipeline.cpp) | Matched ground-truth comparison proving `Radar + GNSS Fusion` reduces RMSE over single sensors under unbiased complementary noise, demonstration of unmodeled GNSS bias degrading fusion compared to unbiased Radar-only, sensor failure & occlusion recovery, versioned `session.json` + CSV bundle export/reload, deterministic measurement replay across associators, and pause/step/resume/restart controls. |

---

## 2. Running the Tests

### Standard Release / Headless Execution

```bash
cmake --preset headless
cmake --build --preset headless -j
ctest --preset headless --output-on-failure
```

### AddressSanitizer + UndefinedBehaviorSanitizer Execution

```bash
cmake --preset sanitizers
cmake --build --preset sanitizers -j
ctest --preset sanitizers --output-on-failure
```
