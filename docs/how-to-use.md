# How to Use

## Build and Run

### Docker (recommended)

```bash
docker compose run --rm dev cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
docker compose run --rm dev cmake --build build --target simulator_app
docker compose run --rm dev ./build/simulator_app 10000 0.01
```

### Host

```bash
cmake -S . -B build
cmake --build build --target simulator_app
./build/simulator_app 1000 0.01
```

## Configuration

Default controller configuration:

- `config/altitude_controller.yaml`

Aggressive alternative:

- `config/altitude_controller_fast.yaml`

Run with explicit config file:

```bash
./build/simulator_app 1000 0.01 config/altitude_controller_fast.yaml
```

Run with explicit altitude + weather config files:

```bash
./build/simulator_app 1000 0.01 config/altitude_controller.yaml config/weather.yaml
```

## Tests

Run all tests:

```bash
ctest --test-dir build --output-on-failure
```

Run altitude-controller-related tests:

```bash
ctest --test-dir build -R alt_ctrl --output-on-failure
```

## Chart Generation

Recommended workflow (ensures UTF-8 simulator log for chart parser):

```bash
docker compose run --rm dev sh -lc './build/simulator_app 1000 0.01 > test.txt'
docker compose run --rm chart
```

Generate default full dashboard chart:

```bash
docker compose run --rm chart
```

Default output file:

- `docs/tutorials/charts/flight_dashboard.png`

Generate custom full dashboard:

```bash
docker compose run --rm chart sh -c "pip install --no-cache-dir pandas matplotlib && python tools/scripts/generate_chart.py --input test.txt --mode full --output docs/tutorials/charts/my_run_full.png"
```

Generate minimal chart (sensors + altitude + controller state):

```bash
docker compose run --rm chart sh -c "pip install --no-cache-dir pandas matplotlib && python tools/scripts/generate_chart.py --input test.txt --mode minimal --output docs/tutorials/charts/my_run_minimal.png"
```

Run chart parser tests:

```bash
docker compose run --rm chart-test
```

Telemetry now also appends mixer and ENU/attitude fields (`ComRPM`, `MixYPR`, `MRef`, `PosENU`, `VelENU`, `YPR`) while preserving existing chart-compatible core fields.

Telemetry now also includes:

- sensed/perfect GPS position + velocity pairs
- weather terms (`WTotAcc`, `WSteady`, `WGust`, `WTurb`)

Full dashboard currently includes a dual-axis energy/temperature panel and weather plots.
