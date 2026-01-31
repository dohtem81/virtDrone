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

## Testing (Runnable Part)
Testing is the **first-class feature currently available**.

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


Tests are the only currently runnable component and demonstrate modular behavior.

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
