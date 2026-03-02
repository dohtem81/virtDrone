# Changelog

This changelog tracks major architecture, runtime, and documentation updates.

## Entry policy
- Use the `YYYY-MM-DD` date format for each release/update block.
- Group changes under short headings (for example: runtime, controller, energy, tooling, docs).
- Keep entries concise and user-visible (avoid internal-only refactor noise unless it changes behavior).
- Add newest entries at the top.

## 2026-03-02

### Runtime and architecture
- Separated real-drone control logic from simulation plant physics.
- Moved sensor noise modeling to the sim-to-real connection path.
- Added connection adapter for noisy sensor transport.
- Kept simulation physics updates on perfect internal plant state.

### Controller and telemetry
- Added PID-related telemetry in logs (target error, P, I, D).
- Added target altitude and sensed/perfect paired telemetry fields.
- Added configurable PID activation behavior with error-band gating.

### Energy and motor behavior
- Enforced battery depletion cutoff: depleted battery drives RPM to zero.
- Coupled max achievable RPM to available battery voltage.

### Tooling and charts
- Added Docker compose chart workflow.
- Updated chart generator to parse current telemetry format.
- Added full and minimal chart modes.
- Set default chart output path under docs/tutorials/charts.

### Documentation
- Reorganized root documentation to docs/ pages.
- Added dedicated pages for architecture, usage, current state, and roadmap.
