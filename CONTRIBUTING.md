# Contributing to AegisTrack

Open an issue for a substantial algorithm or architecture change before starting
implementation. For a small fix, branch from current `main` and open a pull
request describing the trigger, changed behavior, and verification performed.
Keep unrelated formatting and generated build/export files out of the PR.

## Build and test

Use the pinned dependency route in [docs/dependencies.md](docs/dependencies.md)
for a reproducible package selection:

```sh
cmake --preset vcpkg-release
cmake --build --preset vcpkg-release --parallel 2
ctest --preset vcpkg-release --output-on-failure
```

The `headless` preset also supports system packages. For engine changes, run
the `sanitizers` preset on Linux. For desktop changes, build the `default` preset
with Qt installed and run `aegistrack-desktop --capture-screenshots <directory>`;
inspect the images and describe any manual interaction checks separately.

## Code and verification expectations

- Use C++20, four spaces, and the repository `.clang-format` / `.clang-tidy`.
- Keep the core independent of Qt. Do not expose ground truth to filtering,
  association or fusion; ground truth belongs to simulation and evaluation.
- Add a regression test for behavior fixes. Use deterministic seeds for
  simulation tests and tolerances justified by the numerical model.
- Do not present CI smoke, benchmark samples or sanitizers as universal
  correctness, real-time guarantees or exhaustive GUI testing.
- Update affected docs and scenario examples. Dependency updates must include
  the registry baseline/tool revision, constraints/overrides and a passing
  vcpkg build; see the update procedure in docs/dependencies.md.

CI must pass before merging. Never commit credentials, local absolute paths,
personal data, build outputs or unrelated screenshots. Contributions are
distributed under the repository MIT license.
