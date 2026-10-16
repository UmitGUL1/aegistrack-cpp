# Tracking & Fusion Algorithms — AegisTrack

## 1. Multi-Target Data Association

In multi-object tracking, the tracker receives a scan of $M$ sensor observations $\{\mathbf{z}_1, \dots, \mathbf{z}_M\}$ at timestamp $t_k$ without privileged target identities and must associate them with $N$ active predicted tracks $\{\mathcal{T}_1, \dots, \mathcal{T}_N\}$.

### 1.1 Statistical Ellipsoidal Gating

Before solving assignment, each track-measurement pair $(i, j)$ is evaluated using the track's `IStateEstimator::computeInnovation`:

$$
d_{ij}^2 = \boldsymbol{\nu}_{ij}^T \mathbf{S}_{ij}^{-1} \boldsymbol{\nu}_{ij}
$$

where $\boldsymbol{\nu}_{ij}$ is the measurement residual (with angular wrapping $\nu_\theta \in (-\pi, \pi]$ for polar radar bearings) and $\mathbf{S}_{ij} = \mathbf{H}_i \mathbf{P}_{i, k|k-1} \mathbf{H}_i^T + \mathbf{R}_j$ is the innovation covariance.

Pairs exceeding the $\chi^2$ threshold $\gamma_m(\alpha)$ ($9.2103$ for $99\%$ confidence in 2D) are marked inadmissible (`kForbiddenCost`).

### 1.2 Greedy Nearest Neighbor (`NearestNeighborAssociator`)

1. Collect all admissible pairs $(i, j)$ satisfying $d_{ij}^2 \le \gamma_m$.
2. Sort admissible pairs in ascending order of $d_{ij}^2$.
3. Iterate through the sorted pairs, greedily assigning $(i, j)$ if neither track $i$ nor measurement $j$ has been assigned yet.

**Complexity**: $\mathcal{O}(NM \log(NM))$. Fast, but susceptible to suboptimal local choices when two closely spaced or crossing tracks contend for a shared observation.

### 1.3 Global Gated Hungarian Assignment (`HungarianAssociator`)

To resolve contention globally while respecting statistical gates, AegisTrack constructs an augmented $N \times (M + N)$ cost matrix $\tilde{\mathbf{C}}$:

- **Real measurement columns ($0 \le j < M$)**:
  $$
  \tilde{C}_{i, j} = \begin{cases}
  d_{ij}^2 & \text{if } d_{ij}^2 \le \gamma_{\text{gate}} \\
  C_{\text{forbidden}} & \text{otherwise}
  \end{cases}
  $$
- **Per-track dummy unassigned columns ($j = M + i$)**:
  $$
  \tilde{C}_{i, M + i} = (N + 1)(\gamma_{\text{gate}} + 1), \quad \tilde{C}_{k, M + i} = C_{\text{forbidden}} \;\; (k \ne i)
  $$

where $C_{\text{forbidden}} \gg N \cdot \tilde{C}_{i, M+i}$.

Solving this rectangular assignment problem via the Jonker-Volgenant / Kuhn-Munkres shortest-augmenting-path algorithm ($\mathcal{O}(N^2(M+N))$) guarantees:
1. **No out-of-gate association is ever selected**, because every row $i$ can always choose its dedicated dummy column $M+i$ at cost $\ll C_{\text{forbidden}}$.
2. **Maximum-cardinality admissible matching**, because replacing any dummy assignment with an augmenting path of gated edges ($d_{ij}^2 \le \gamma_{\text{gate}}$) strictly lowers total cost.
3. **Minimum sum of squared Mahalanobis distances** $\sum d_{ij}^2$ among all maximum-cardinality gated matchings.

---

## 2. Sequential Multi-Sensor Fusion and Duplicate Initiation Suppression

When multiple sensors (e.g., a $2.5\text{ Hz}$ ground radar and $1.0\text{ Hz}$ onboard GNSS telemetry) produce observations at or near the same timestamp:

1. **Single-Sensor Scan Partitioning**: Incoming ready measurements are sorted chronologically by `capture_timestamp` and partitioned by `(capture_timestamp, sensor_id)` into single-sensor scans.
2. **Sequential Filter Updates**: Each single-sensor scan updates matched tracks sequentially using `ExtendedKalmanFilter::updateMeasurement`. Consequently, if both Radar and GNSS observe the same object at $t = 1.0\text{ s}$, both measurements sequentially refine the track's state and covariance rather than competing against each other in a single 1-to-1 assignment matrix.
3. **Duplicate Track Initiation Suppression**: Before an unmatched measurement $\mathbf{z}_u$ initiates a new `Tentative` track, `TrackManager::shouldSuppressDuplicateInitiation` checks whether $\mathbf{z}_u$ lies within `duplicate_initiation_radius_m` or within an expanded $2.25\gamma$ Mahalanobis gate of any existing track. This prevents separate sensors from spawning duplicate tentative tracks on a newly appearing target.

---

## 3. Time-Aware Track Lifecycle Management

Because sensors operate at heterogeneous sampling frequencies and not all objects carry GNSS transponders, lifecycle transitions are governed by **elapsed simulation time without observation** ($\Delta t_{\text{unseen}} = t_{\text{now}} - t_{\text{last\_update}}$) alongside a time-bucketed missed-revisit counter:

- **Tentative $\to$ Confirmed**: Requires `consecutive_hits >= confirmation_hits` (default `3`).
- **Tentative $\to$ Deleted**: Triggered when $\Delta t_{\text{unseen}} > T_{\text{tentative\_max}}$ (default `2.2 s`) or `consecutive_misses > max_tentative_misses`. Rapidly purges false radar alarms (clutter).
- **Confirmed $\to$ Coasting**: Triggered when $\Delta t_{\text{unseen}} > T_{\text{coast}}$ (default `1.5 s`). During coasting, the track continues predicting its state and growing its covariance ellipse $\mathbf{P}_{k|k-1}$.
- **Coasting $\to$ Confirmed**: Triggered immediately upon associating a gated measurement (enabling recovery after sensor outages or terrain/building occlusions).
- **Coasting $\to$ Deleted**: Triggered when $\Delta t_{\text{unseen}} > T_{\text{delete\_max}}$ (default `6.0 s`) or `consecutive_misses > max_missed_updates`.

---

## 4. Out-of-Sequence Measurement (OOSM) Handling

Each measurement carries both a physical `capture_timestamp` and a network `arrival_timestamp = capture_timestamp + latency_sec`. `SensorScheduler` supports two explicit policies:

1. **`DropStale` (Default)**: Measurements arriving at $t_{\text{now}}$ are sorted by `capture_timestamp`. Any measurement whose `capture_timestamp < last_released_capture_time` is rejected as stale and logged via `SystemEventType::StaleMeasurementDropped`, preventing silent corruption of the Kalman filter with out-of-order observations.
2. **`BoundedReorderBuffer`**: Arrived measurements are held in a sliding reorder window of duration `reorder_buffer_window_sec` so delayed packets within the window are sorted into strict chronological `capture_timestamp` order before release to the `FusionEngine`.
