# Repository health follow-up — 17 October 2026

The supplied RepoDoctor report scored the original source 95.3/100 and listed
four actionable findings. The working source was compared against all 103
tracked files of upstream commit `0c8a84e9c372f3709978f96f047776ef88c2c648`
before changes; file contents matched (allowing checkout line endings).

| Finding | Resolution |
| --- | --- |
| RD-DOC-005: contribution guidelines missing | CONTRIBUTING.md and README contribution link |
| RD-BLD-004: CI permissions unspecified | Explicit `contents: read`, no persisted checkout credentials |
| RD-DEP-001: missing reproducible baseline | vcpkg baseline and CI tool revision both pinned to the same commit |
| RD-DEP-002: five unconstrained dependencies | Minimum constraints plus exact version/port overrides |

Further improvements include a commit-pinned checkout Action, CI timeouts and
bounded build parallelism, complete vcpkg configure/build/test presets, explicit
GUI feature selection, a corrected manifest homepage, and a dependency update
guide. Manifest builds reject missing packages instead of silently downloading
another dependency set through FetchContent.

## Executed CI verification

Source: `6bc86dae7daa7d6a42260e95e97998d4e0a25723`.
[Successful workflow 39044689724](https://github.com/UmitGUL1/aegistrack-cpp/actions/runs/39044689724).

| Ubuntu 24.04 configuration | CTest result | Recorded CTest duration | Additional checks |
| --- | --- | --- | --- |
| Pinned vcpkg Release | 34 passed, 0 failed | 0.17 s | CLI fusion comparison; benchmark smoke |
| System-package headless Release | 34 passed, 0 failed | 0.19 s | CLI fusion comparison; benchmark smoke |
| ASan + UBSan | 34 passed, 0 failed | 16.53 s | Leak detection and sanitizer halt-on-error enabled |
| System-package Qt desktop | 34 passed, 0 failed | 0.18 s | Four expected offscreen screenshots generated |

These durations describe test execution, not simulation performance. Running
the same cases in four environments does not create 136 distinct test cases.
The manifest log confirms installation of the versions listed in
[dependencies.md](dependencies.md), including their pinned port revisions.
No local C++ build or new manual GUI acceptance is claimed.

## RepoDoctor repeat scan

RepoDoctor CLI 1.0.0 scanned the modified local source with Git metadata on
17 October 2026 at 15:21:47 UTC, without baseline suppression or disabled rules:

```sh
repodoctor-cli scan . --format json --output ../work/aegistrack-after.json --fail-on NONE
```

Result: **100/100, grade A; 25 passed checks, no actionable findings, two
NOT_VERIFIED checks**. The original four rules now pass. The two unverified
checks concern POSIX permission bits on Windows and the absence of a local
dependency vulnerability advisory cache. They are not counted as passes.

This heuristic score is not proof of correctness or a vulnerability audit.
Windows runtime builds, the optional vcpkg Qt build, exhaustive GUI interaction,
and dependency advisory verification remain outside this validation.
