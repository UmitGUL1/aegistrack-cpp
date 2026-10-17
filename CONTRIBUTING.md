# Contributing to AegisTrack

Thanks for helping improve AegisTrack. Please open an issue for substantial behavior or interface changes before preparing a pull request.

## Build and test

The core and CLI can be built without Qt. Use CMake 3.24+, Ninja, and a C++20 compiler:

```sh
cmake --preset headless
cmake --build --preset headless -j
ctest --preset headless --output-on-failure
./build/headless/aegistrack-cli --scenario scenarios/fusion-comparison.json --fusion-comparison
```

For GUI changes, also use the `default` preset with Qt 6 installed. See [README.md](README.md) for dependency and platform setup. CI additionally runs sanitizer tests and an offscreen Qt screenshot check.

## Pull requests

- Keep changes focused and include a test for behavioral fixes or new tracking behavior.
- Follow the existing C++20 style and `.clang-format` / `.clang-tidy` configuration.
- Keep simulation truth out of the fusion and association APIs; use it only for sensor generation and post-step evaluation.
- Describe the change, commands run, and any tests that could not be run locally.
- Do not commit build directories, generated sessions, credentials, or personal sensor recordings.

## Dependencies

The optional vcpkg manifest uses a `builtin-baseline` to select a repeatable dependency graph. Update the baseline deliberately, check the minimum versions in `vcpkg.json`, and run the headless and GUI CI paths before merging a dependency refresh.
