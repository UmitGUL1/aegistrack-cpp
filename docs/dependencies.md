# Dependency selection and repeatable builds

## Pinned manifest route

`vcpkg.json` uses the verified vcpkg 2024.03.25 commit
`a34c873a9717a888f58dc05268dea15592c2f0ff` as `builtin-baseline`.
This selects the registry versions of transitive packages. Direct dependencies
also have explicit minimum constraints and exact overrides, including port
revisions. The same commit is checked out for the vcpkg tool in CI.

| Direct package | Selected version | Port revision |
| --- | --- | --- |
| eigen3 | 3.4.0 | 4 |
| nlohmann-json | 3.11.3 | 0 |
| spdlog | 1.13.0 | 0 |
| gtest | 1.14.0 | 0 |
| benchmark | 1.8.3 | 2 |

This is a deliberate known-version baseline, not a claim that these are the
newest packages or that dependencies have undergone a vulnerability audit.
Pinning selects source versions; it does not make compiler/OS outputs bitwise
identical. The ordinary Ubuntu system-package jobs are compatibility checks,
not locked environment builds.

From a Linux shell with Git, CMake 3.24+, Ninja and a C++20 compiler:

```sh
git clone https://github.com/microsoft/vcpkg.git .tools/vcpkg
git -C .tools/vcpkg checkout a34c873a9717a888f58dc05268dea15592c2f0ff
./.tools/vcpkg/bootstrap-vcpkg.sh -disableMetrics
export VCPKG_ROOT="$PWD/.tools/vcpkg"
cmake --preset vcpkg-release
cmake --build --preset vcpkg-release --parallel 2
ctest --preset vcpkg-release --output-on-failure
./build/vcpkg-release/aegistrack-cli --scenario scenarios/fusion-comparison.json --fusion-comparison
```

On Windows use Developer PowerShell, `bootstrap-vcpkg.bat -disableMetrics`,
and `$env:VCPKG_ROOT = "$PWD/.tools/vcpkg"`; use the same CMake commands.
Windows is not covered by the new manifest CI job.

The manifest Release preset is explicitly headless. To compile Qt from the
manifest, use `vcpkg-desktop`, which selects the `gui` feature (Qt 6.6.1 at this
baseline). This optional vcpkg Qt build is not covered by CI; the existing
desktop CI tests Ubuntu Qt packages. All vcpkg presets disable FetchContent
fallback, so a missing toolchain package fails instead of silently using a
different dependency source.

## System packages / FetchContent

`default`, `headless` and `sanitizers` retain the system-package workflow.
`cmake/Dependencies.cmake` checks installed package minimums and can download
missing core packages from version tags. This convenient fallback is not an
immutable lock; pass `-DAEGISTRACK_FETCH_MISSING_DEPENDENCIES=OFF` to forbid it.

## Updating dependencies

1. Choose and inspect a vcpkg commit; update both `builtin-baseline` and the
   CI vcpkg checkout `ref` together.
2. Read that commit's port manifests; update direct constraints and overrides
   (including port revisions), this version table and the setup commands.
3. Configure into a fresh build directory, run the manifest test/CLI/benchmark
   job and existing system-package/sanitizer/desktop jobs.
4. Review numerical or scenario behavior changes before merging. Do not move
   pins simply to remove a diagnostic without building the selected versions.
