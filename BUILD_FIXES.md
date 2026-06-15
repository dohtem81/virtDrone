# Build and Runtime Fixes Summary

## Scope

This file summarizes major build/runtime corrections that brought the repository to a consistent split-runtime state.

## Key Fix Areas

1. API and integration alignment
- Updated control/runtime and test call sites to current method names and signatures.
- Restored compatibility between `RealDrone`, controller tests, and mixer tests.

2. Simulation gateway contract completeness
- Ensured simulator-side gateway implementation includes all required contract methods.
- Removed abstract-instantiation build breaks in simulator factory paths.

3. Dependency/toolchain stabilization
- Docker dev image now installs protobuf/gRPC development packages.
- CMake now resolves protobuf/gRPC with `find_package(...)`.
- This removed flaky source-fetch failures during configure.

4. Docker validation workflow
- Validation script runs configure/build/test in container.
- Summary generation is optional when `python3` is not available.

## Current Build/Test Status

- Docker configure/build: passing
- Docker full test run: `95/95` tests passed

## Current Validation Command

```bash
docker compose run --rm dev bash -lc '/workspace/tools/scripts/refactor-closeout-docker.sh'
```

## Runtime Entry Points

- Simulation process:

```bash
docker compose run --rm dev bash -lc '/workspace/build/simulator_app config/weather.yaml docs/tutorials 0.0.0.0:50051'
```

- Control process:

```bash
docker compose run --rm dev bash -lc '/workspace/build/drone_app grpc localhost:50051 10000 0.01 config/altitude_controller.yaml config/attitude_controller.yaml config/missions/hover_and_move.yaml docs/tutorials'
```