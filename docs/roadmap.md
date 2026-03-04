# Roadmap

## In progress

- Improve thrust/power closure consistency across full operating envelope
- Expand closed-loop behavior tests under saturation/noise/weather combinations
- Add optional strict mission schema validation mode (error on unknown enum/string values)
- Refine chart/report views for faster run-to-run diagnosis

## Mid-term

- Introduce richer rigid-body dynamics (beyond current translational + commanded-attitude model)
- Expand controller stack for multi-axis control paths
- Expand mission scenario libraries and mission-level integration test suites
- Add explicit scenario definitions for repeatable tuning experiments
- Add fault/degradation scenarios for robustness testing

## Longer-term

- Hardware-in-the-loop friendly interfaces
- Better parameter identification workflow for real hardware datasets
- Optional higher-fidelity model modules where justified by use case

## Direction principle

Model complexity should increase only when it improves engineering decisions.

The project prioritizes clarity, traceability, and testability over realism-for-its-own-sake.
