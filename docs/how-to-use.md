# How to Use

## Build and Run

### Docker (recommended)

```bash
docker compose run --rm dev cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
docker compose run --rm dev cmake --build build --target simulator_app
docker compose run --rm dev ./build/simulator_app 1000 0.01
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
