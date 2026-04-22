# Fractional Teleoperation Analysis

This folder contains executable analyses for stability, controllability, bounded-input behavior, and hybrid snap behavior of the standalone node model.

## Scope

The implementation mirrors the update ordering used by the standalone node:

1. Dynamic alpha update and coefficient refresh.
2. Adaptive gain update (optional).
3. Fractional offset update (GL recursion).
4. Ramp scaling update.
5. Snap-on-release transition.
6. Reference drift update (first-order or fractional mode).
7. Velocity output from scaled offset difference.

The baseline parameters follow the current YAML runtime values.

## Files

- fractional_node_model.py: model-faithful simulator and analysis primitives.
- run_analysis.py: CLI runner that generates CSV tables and figures.
- results/: generated artifacts.

## Run

From this folder:

/bin/python3 run_analysis.py

Optional arguments:

- --output-dir PATH
- --alpha-min 0.1 --alpha-max 1.0 --alpha-count 10
- --iss-steps 1500
- --snap-steps 400
- --hybrid-steps 1200
- --input-bound 1.0
- --memory-min 20 --memory-max 100 --memory-count 5
- --threshold-min 0.005 --threshold-max 0.05 --threshold-count 8
- --seed 7

## Outputs

- stability_table.csv
  - alpha, augmented state dimension, spectral radius, Schur stability flag.
- controllability_table.csv
  - controllability rank, rank ratio, finite-horizon Gramian minimum eigenvalue.
- iss_table.csv
  - bounded-input metrics (max and mean velocity norm, reference excursion).
- hybrid_threshold_table.csv
  - transition count, snap count, mean dwell time, reset contraction ratios vs joystick active threshold.
- alpha_memory_sweep_table.csv
  - max velocity and reference excursion across alpha-memory grid.
- spectral_radius_vs_alpha.png
- iss_metrics_vs_alpha.png
- snap_transition_trace.png
- hybrid_threshold_sensitivity.png
- alpha_memory_heatmap.png

## Notes

- The linear stability and controllability checks are based on a fixed-alpha augmented linear model.
- The hybrid and ISS traces are generated with the nonlinear/hybrid simulator.
- This is an implementation-faithful engineering analysis, not a formal proof for variable-order fractional systems.
