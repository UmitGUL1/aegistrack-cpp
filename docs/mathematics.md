# Mathematical Formulation — AegisTrack

## 1. Coordinate System and State Vector

All state estimation and physical simulation occur in a local 2D East-North Cartesian frame measured in SI units (meters and seconds).

The 4D kinematic state vector of a tracked target at time $t_k$ is:

$$
\mathbf{x}_k = \begin{bmatrix} p_x \\ p_y \\ v_x \\ v_y \end{bmatrix} \in \mathbb{R}^4
$$

where $(p_x, p_y)$ denotes position in meters ($\text{m}$) and $(v_x, v_y)$ denotes velocity in meters per second ($\text{m/s}$).

---

## 2. State Prediction and Continuous White-Noise Acceleration (CWNA) Model

For a time step $\Delta t = t_k - t_{k-1} \ge 0$, the discrete-time Constant Velocity (CV) state-transition matrix is:

$$
\mathbf{F}(\Delta t) = \begin{bmatrix}
1 & 0 & \Delta t & 0 \\
0 & 1 & 0 & \Delta t \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

State and covariance prediction follow:

$$
\hat{\mathbf{x}}_{k|k-1} = \mathbf{F}(\Delta t) \hat{\mathbf{x}}_{k-1|k-1}
$$

$$
\mathbf{P}_{k|k-1} = \mathbf{F}(\Delta t) \mathbf{P}_{k-1|k-1} \mathbf{F}(\Delta t)^T + \mathbf{Q}(\Delta t)
$$

### Physical Process Noise Derivation

Target maneuvers and unmodeled accelerations are modeled as zero-mean continuous-time white Gaussian acceleration noise $\mathbf{a}(t) = [a_x(t), a_y(t)]^T$ with power spectral density $q = \sigma_a^2$ ($\text{m}^2/\text{s}^4$). Integrating the continuous system over $\Delta t$ yields the exact piecewise-constant / continuous white-noise acceleration covariance matrix:

$$
\mathbf{Q}(\Delta t) = \sigma_a^2 \begin{bmatrix}
\frac{\Delta t^4}{4} & 0 & \frac{\Delta t^3}{2} & 0 \\
0 & \frac{\Delta t^4}{4} & 0 & \frac{\Delta t^3}{2} \\
\frac{\Delta t^3}{2} & 0 & \Delta t^2 & 0 \\
0 & \frac{\Delta t^3}{2} & 0 & \Delta t^2
\end{bmatrix}
$$

---

## 3. Linear GNSS Position Measurement Update

Onboard GNSS telemetry provides direct Cartesian position observations:

$$
\mathbf{z}_k^{\text{GNSS}} = \begin{bmatrix} z_x \\ z_y \end{bmatrix} = \mathbf{H} \mathbf{x}_k + \mathbf{w}_k, \quad \mathbf{w}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{R}_k^{\text{GNSS}})
$$

with observation matrix:

$$
\mathbf{H} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0
\end{bmatrix}, \quad
\mathbf{R}_k^{\text{GNSS}} = \begin{bmatrix}
\sigma_x^2 & 0 \\
0 & \sigma_y^2
\end{bmatrix}
$$

### Innovation and Joseph Stabilized Update

1. **Innovation (Measurement Residual)**:
   $$
   \boldsymbol{\nu}_k = \mathbf{z}_k - \mathbf{H} \hat{\mathbf{x}}_{k|k-1}
   $$
2. **Innovation Covariance**:
   $$
   \mathbf{S}_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + \mathbf{R}_k
   $$
3. **Kalman Gain via Cholesky Decomposition**:
   Instead of computing $\mathbf{S}_k^{-1}$ explicitly, AegisTrack solves the symmetric positive-definite linear system:
   $$
   \mathbf{S}_k \mathbf{K}_k^T = \mathbf{H} \mathbf{P}_{k|k-1}^T
   $$
   using `Eigen::LLT` (with automatic `Eigen::LDLT` + diagonal Tikhonov regularization fallback if $\mathbf{S}_k$ is ill-conditioned).
4. **Posterior State Estimate**:
   $$
   \hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \boldsymbol{\nu}_k
   $$
5. **Joseph Stabilized Covariance Update**:
   To preserve symmetry and positive semi-definiteness in finite-precision floating-point arithmetic, the posterior covariance is computed via the Joseph form:
   $$
   \mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1} (\mathbf{I} - \mathbf{K}_k \mathbf{H})^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T
   $$
   followed by explicit symmetrization $\mathbf{P}_{k|k} \leftarrow \frac{1}{2}(\mathbf{P}_{k|k} + \mathbf{P}_{k|k}^T)$.

---

## 4. Nonlinear Radar Range-Bearing Model (Extended Kalman Filter)

A fixed radar station located at $\mathbf{p}_s = [x_s, y_s]^T$ measures target range $r$ and azimuth bearing $\theta$:

$$
\mathbf{z}_k^{\text{Radar}} = \begin{bmatrix} r \\ \theta \end{bmatrix} = \mathbf{h}(\mathbf{x}_k) + \mathbf{v}_k, \quad \mathbf{v}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{R}_k^{\text{Radar}})
$$

where $\Delta x = p_x - x_s$, $\Delta y = p_y - y_s$, and:

$$
\mathbf{h}(\mathbf{x}) = \begin{bmatrix}
\sqrt{\Delta x^2 + \Delta y^2} \\
\operatorname{atan2}(\Delta y, \Delta x)
\end{bmatrix}, \quad
\mathbf{R}_k^{\text{Radar}} = \begin{bmatrix}
\sigma_r^2 & 0 \\
0 & \sigma_\theta^2
\end{bmatrix}
$$

### Analytical Measurement Jacobian

Linearizing $\mathbf{h}(\mathbf{x})$ around the predicted state $\hat{\mathbf{x}}_{k|k-1}$ yields the $2 \times 4$ Jacobian matrix:

$$
\mathbf{H}_J = \left.\frac{\partial \mathbf{h}}{\partial \mathbf{x}}\right|_{\hat{\mathbf{x}}_{k|k-1}} =
\begin{bmatrix}
\frac{\Delta x}{r} & \frac{\Delta y}{r} & 0 & 0 \\
-\frac{\Delta y}{r^2} & \frac{\Delta x}{r^2} & 0 & 0
\end{bmatrix}
$$

### Singularity Guard and Bearing Normalization

- **Origin Singularity Guard**: When $r = \sqrt{\Delta x^2 + \Delta y^2} < r_{\min}$ ($r_{\min} = 10^{-4}\text{ m}$), $\Delta x$ and $r$ are clamped to $r_{\min}$ to prevent division by zero in $\mathbf{H}_J$.
- **Angle Wrapping**: Because bearing lies on the manifold $S^1$, the angular innovation is wrapped onto $(-\pi, \pi]$:
  $$
  \nu_\theta = \operatorname{wrap}_{(-\pi, \pi]}\left(z_\theta - \operatorname{atan2}(\Delta y, \Delta x)\right)
  $$

### Track Initiation Covariance Conversion

When an unmatched polar radar detection $(r, \theta)$ initiates a new track, its initial Cartesian position is $\mathbf{p}_0 = \mathbf{p}_s + [r\cos\theta, r\sin\theta]^T$, and its $2\times 2$ Cartesian position covariance is propagated via the polar-to-Cartesian Jacobian $\mathbf{J}_{p\to c}$:

$$
\mathbf{J}_{p\to c} = \begin{bmatrix}
\cos\theta & -r\sin\theta \\
\sin\theta & r\cos\theta
\end{bmatrix}, \quad
\mathbf{P}_{xy, 0} = \mathbf{J}_{p\to c} \mathbf{R}^{\text{Radar}} \mathbf{J}_{p\to c}^T
$$

---

## 5. Statistical Gating and 2D Uncertainty Ellipses

### Mahalanobis Distance Gating

For track $i$ and candidate measurement $j$, the Normalized Innovation Squared (squared Mahalanobis distance) is:

$$
d_{ij}^2 = \boldsymbol{\nu}_{ij}^T \mathbf{S}_{ij}^{-1} \boldsymbol{\nu}_{ij}
$$

Under Gaussian error assumptions, $d_{ij}^2 \sim \chi^2_m$ where $m = \dim(\mathbf{z}_j)$. A candidate association is admissible only if $d_{ij}^2 \le \gamma_m(\alpha)$. For $m = 2$, $\gamma_2(0.95) = 5.9915$ and $\gamma_2(0.99) = 9.2103$.

### 2D Covariance Uncertainty Ellipse

Given the $2 \times 2$ marginal position covariance $\mathbf{P}_{xy}$ with eigenvalues $\lambda_{\max} \ge \lambda_{\min} \ge 0$ and unit principal eigenvector $\mathbf{v}_{\max} = [v_x, v_y]^T$, the $95\%$ confidence ellipse ($\gamma = 5.9915$) has semi-axes and orientation:

$$
a = \sqrt{\gamma \lambda_{\max}}, \quad b = \sqrt{\gamma \lambda_{\min}}, \quad \phi = \operatorname{atan2}(v_y, v_x)
$$
