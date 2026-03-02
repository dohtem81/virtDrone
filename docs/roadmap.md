# Roadmap

## Near-term

- Improve thrust/power closure consistency across full operating envelope
- Add more explicit sensor noise configuration (per sensor, per profile)
- Add minimal scenario definitions for repeatable tuning experiments
- Add stronger test coverage around closed-loop behavior under saturation/noise

## Mid-term

- Introduce richer rigid-body dynamics (beyond current vertical-dynamics focus)
- Expand controller stack for multi-axis control paths
- Add fault/degradation scenarios for robustness testing
- Improve charting/reporting for run-to-run comparison

## Longer-term

- Hardware-in-the-loop friendly interfaces
- Better parameter identification workflow for real hardware datasets
- Optional higher-fidelity model modules where justified by use case

## Direction principle

Model complexity should increase only when it improves engineering decisions.

The project prioritizes clarity, traceability, and testability over realism-for-its-own-sake.
