# Implementation and verification status

AegisTrack contains a C++20 headless simulation/tracking library, a CLI, an optional Qt 6 desktop UI, GoogleTest tests, Google Benchmark cases, JSON scenarios, and GitHub Actions workflows. The core includes radar/GNSS models, KF/EKF estimation, nearest-neighbor/Hungarian association, track lifecycle management, recording, and replay.

The source has received fixes for capture-time sensor sampling, capture-time raw measurement error, revisit interval validation, and strict chronological sorting. Regression tests were added for those behaviors. GitHub Actions verified commit `86c6b09c777fd391c446fd5eb2d3bec17aece1f3` on Ubuntu 24.04 with GCC 13. The headless job built the core and CLI, passed all 34 CTest cases, ran a fusion comparison scenario and Google Benchmark smoke check. The sanitizer job passed the same tests under ASan and UBSan. The Qt 6 job built the desktop application and generated all four offscreen screenshots.

This verification is on Linux CI; the local Windows toolchain remains unavailable. The benchmark job is a smoke check, not a new published performance study. Do not reuse historical benchmark or accuracy figures for this revision.

## Repository hardening verified on 17 October 2026

Commit `6bc86dae7daa7d6a42260e95e97998d4e0a25723` passed all four jobs in
[run 39044689724](https://github.com/UmitGUL1/aegistrack-cpp/actions/runs/39044689724):
the pinned vcpkg Release route, system-package headless Release, ASan/UBSan,
and Qt desktop configuration. Each passed 34 CTest cases. Both headless routes
also ran the CLI fusion comparison and benchmark smoke; the Qt job generated
all four expected offscreen screenshots.

The four supplied RepoDoctor findings are resolved: contribution guidelines,
explicit least-privilege CI permissions, a vcpkg registry baseline, and direct
dependency version constraints. Additional hardening pins the checkout Action,
disables stored checkout credentials, bounds job/build resources and prevents
silent FetchContent fallback in manifest presets. Preset schema 5 matches the
documented CMake 3.24 minimum.

See [repository health verification](docs/repository-health.md) for evidence,
the repeat scan and verification limits.
