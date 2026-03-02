# virtDrone  
**Vehicle Subsystem Simulation & Control Framework (Work in Progress)**

## Overview
`virtDrone` is a modular simulation framework focused on **vehicle subsystem modeling, power/thrust closure, and incremental introduction of control loops**.

The goal of this project is **not** high-fidelity CFD or photorealistic simulation.  
Instead, it demonstrates a **correct engineering workflow**:

> component modeling → subsystem validation → open-loop feasibility → closed-loop control → system-level behavior under constraints

While the initial target vehicle is a multirotor drone, the architecture is intentionally generic and can be extended to other vehicles (e.g., rockets, wheeled platforms).

---

## Design Philosophy
This project follows principles used in real aerospace and industrial control programs:

- Physics before control
- Open-loop feasibility before closed-loop tuning
- Explicit assumptions over hidden magic
- Known limitations documented
- Component validation before system integration

The simulator is built to answer **engineering questions**, not to “look realistic”.

---

## Current Project Status
Development is intentionally structured into **phases**.

### Phase 1 — Component Modeling & Validation *(current)*
Focus: validating individual subsystems in isolation before system integration.

Implemented / in progress:
- **Motor model**
  - RPM ↔ torque ↔ current relationship
  - Efficiency and losses
  - Saturation limits
- **Battery model**
  - Voltage sag under load
  - Internal resistance
  - Capacity tracking
- **Thermal model**
  - Power loss → temperature rise
  - Thermal time constants
  - Temperature limits

Each component is tested independently before being coupled into a vehicle model.

---

### Phase 2 — Thrust & Power Closure *(next)*
Focus: answering the most basic flight question.

> *Can this vehicle hover at all?*

Planned:
- Thrust = f(RPM) mapping
- Vehicle mass and gravity balance
- Hover operating point calculation
- Power margin and headroom analysis
- Assertions such as:
  - Max thrust > 1.3 × vehicle weight
  - Battery voltage under hover load remains above cutoff
  - Thermal limits are not exceeded during steady hover

This phase remains **open-loop by design**.

---

### Phase 3 — Attitude Dynamics (Single Axis)
Focus: introducing rotational dynamics before position control.

Planned:
- Rigid-body rotational equations (moment → angular acceleration)
- Inertia modeling
- Single-axis (pitch or roll) response
- Incremental control:
  - P control
  - PD control
  - PID only if justified
- Step response plots and stability analysis

---

### Phase 4 — Full Closed-Loop Control
Focus: realistic control behavior under non-ideal conditions.

Planned:
- Multi-axis control
- Sensor noise and bias
- Control loop timing vs sensor update rates
- Actuator saturation and rate limits
- Latency effects

---

### Phase 5 — System Robustness & Uncertainty
Focus: understanding margins and failure modes.

Planned:
- Monte Carlo simulations
- Parameter uncertainty (mass, thrust, drag, sensor noise)
- Trajectory and attitude dispersion analysis
- Identification of dominant contributors to failure

---

## Architecture Overview
The system is split into clearly defined layers:

- **Core simulation**
  - Time stepping
  - Numerical integration
- **Subsystem models**
  - Motors
  - Battery
  - Thermal
  - Sensors (planned)
- **Vehicle model**
  - Aggregates subsystems
  - Applies forces and moments
- **Control logic**
  - Open-loop commands
  - Closed-loop controllers (incremental)
- **Interface**
  - Logging and visualization
  - External control inputs (planned)

This separation allows subsystems to be refined or replaced without rewriting the full simulator.

---

## Assumptions & Limitations
- Aerodynamics are simplified (no CFD)
- No structural flexibility
- No environment modeling beyond basic gravity
- Models favor clarity and traceability over fidelity

These limitations are **intentional** and explicitly documented.

---

## Why This Project Exists
This project is meant to demonstrate:
- Incremental simulation development
- Respect for physical constraints
- How control authority emerges from hardware limits
- How systems fail when assumptions are violated

It is a learning and exploration platform, not a finished product.

---

## Future Extensions
- Alternate vehicle configurations (rocket, fixed-wing)
- Hardware-in-the-loop integration
- Real sensor data playback
- Higher-order dynamics where justified

---

## Simulation
You can now run a quadcopter simulation with altitude control via the `simulator_app` executable.

### Key Features
- **Physics-based simulation**: Motor dynamics, thrust calculation, battery discharge, altitude integration
- **Real-drone control loop**: Altitude controller runs in `drone/runtime` and uses only sensor readings + actuator outputs
- **YAML configuration**: Controller parameters loaded from config files
- **Architecture separation**:
  - **Real drone (`drone/`)**: Reads sensors and computes actuator commands
  - **Simulation (`simulator/`)**: Simulates physical response (RPM, thrust, battery, temperature, altitude)
  - **Bridge contract**: Runtime interfaces connect real-drone logic to simulation

### Runtime Loop (Separated)

`simulator_app` now executes a two-side loop each step:

1. **Real side**: `drone::runtime::RealDrone::update()` reads a sensor frame and writes a motor RPM command.
2. **Simulation side**: `drone::simulator::QuaroSimulation::step()` applies that command and advances physics.

This keeps control decisions out of the simulation physics engine.

### Usage

**Basic usage:**
```bash
./build/simulator_app [steps] [dt_s] [config_file]
```

**Parameters:**
- `steps`: Number of simulation steps (default: 10)
- `dt_s`: Time step in seconds (default: 0.01)
- `config_file`: Path to YAML config file (default: config/altitude_controller.yaml)

**Examples:**
```bash
# Run with default config (50m target altitude)
./build/simulator_app 1000 0.01

# Run with custom config (100m target, aggressive control)
./build/simulator_app 1000 0.01 config/altitude_controller_fast.yaml

# Long simulation with output to file
./build/simulator_app 10000 0.01 > flight_data.txt
```

### Configuration Files

Controller parameters are defined in YAML files under `config/`:

**config/altitude_controller.yaml** (default):
```yaml
altitude_controller:
  target_altitude_m: 50.0        # Target altitude
  altitude_param_p: 2.0          # Altitude tracking P gain
  max_altitude_delta_mps: 5.0    # Max altitude change rate
  control_param_p: 100.0         # RPM control P gain
  control_param_i: 10.0          # RPM control I gain
  control_param_d: 0.0           # RPM control D gain
  enable_i_component: true       # Enable I term
  enable_d_component: false      # Enable D term
  neutral_rpm: 11400.0           # Hover RPM estimate
```

**config/altitude_controller_fast.yaml** (more aggressive):
```yaml
altitude_controller:
  target_altitude_m: 100.0
  altitude_param_p: 3.0
  max_altitude_delta_mps: 10.0
  control_param_p: 150.0
  control_param_i: 20.0
  control_param_d: 0.0
  enable_i_component: true
  enable_d_component: false
  neutral_rpm: 11400.0
```

You can create custom config files to tune controller behavior.

### Output Format

Each simulation step prints telemetry data:
```
T:    5.00s | Alt:    26.14m | RefRPM: 11407.50 | RPM: 11407.50 | Curr:  16.90A | Batt: 1409.60mAh | SOC:  93.97% | V: 16.32V | T:  28.82C
```

- **T**: Simulation time (seconds)
- **Alt**: Current altitude from GPS sensor (meters)
- **RefRPM**: RPM reference from controller
- **RPM**: Actual motor RPM
- **Curr**: Motor current draw (Amperes)
- **Batt**: Battery remaining capacity (mAh)
- **SOC**: State of charge (%)
- **V**: Battery voltage (Volts)
- **T**: Motor temperature (Celsius)

### Build and Run

**Using Docker (Recommended):**
```bash
docker compose run --rm dev cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
docker compose run --rm dev cmake --build build --target simulator_app
docker compose run --rm dev ./build/simulator_app 1000 0.01
```

**On Host:**
```bash
cmake -S . -B build
cmake --build build --target simulator_app
./build/simulator_app 1000 0.01
```

---

## Testing (Runnable Part)
Testing is a **first-class feature currently available**.

### Using Docker (Recommended)
```bash
docker compose run --rm dev
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

### Directly on Host
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

## Running Tests

```bash
# Run all tests
ctest --test-dir build --output-on-failure

# Run specific test
./build/tests/ionet_tests
```


Tests demonstrate modular behavior alongside the simulator app.

### Future Directions

* Fault injection and degraded-mode scenarios

* Optional higher-fidelity models

* Minimal interactive CLI or UI

* Hardware-in-the-loop abstractions

These are planned, but the core design reasoning is already visible.

### Final Note

virtDrone is not about building the best drone simulator. It is about demonstrating how complex engineering systems can be approached, decomposed, and reasoned about — the mindset is the real deliverable.

## License

See [LICENSE](LICENSE) for details.
