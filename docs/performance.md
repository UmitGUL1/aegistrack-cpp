# Performance and validation

This repository includes Google Benchmark cases for Kalman prediction, EKF radar updates, Hungarian association, track updates, and complete simulation steps at 1, 10, 50, and 100 objects. The benchmark cases mutate state across iterations, so they measure sustained execution rather than independent cold starts.

Run from a clean checkout with CMake 3.24+, Ninja, and the dependencies in `cmake/Dependencies.cmake`:

```bash
cmake --preset headless
cmake --build --preset headless
ctest --preset headless --output-on-failure
./build/headless/aegistrack_benchmarks --benchmark_min_time=0.05s
./build/headless/aegistrack-cli --scenario scenarios/fusion-comparison.json --compare-fusion
```

Capture the commit SHA, CPU, compiler, build type, complete benchmark output, and scenario seed with any published measurement. Historical values from before the capture-time sampling and RMSE corrections do not describe this revision. The core tracking engine and Qt rendering have separate scheduling and locking behavior; benchmark timings for the headless engine do not establish GUI responsiveness or a real-time deadline guarantee.

The fusion comparison runs radar-only, GNSS-only, and combined pipelines against the same generated measurements. Position RMSE is calculated over matched tracks within the evaluation cutoff, so report match counts and missed/false tracks alongside RMSE. Raw measurement error is evaluated against ground truth at each measurement's capture time. Ground truth is used for evaluation only.