# Baseline Analysis Results

This report summarizes the first execution of the analysis pipeline.

## Reproduction

From this folder:

/bin/python3 run_analysis.py

Generated outputs in results/:

- stability_table.csv
- controllability_table.csv
- iss_table.csv
- hybrid_threshold_table.csv
- alpha_memory_sweep_table.csv
- spectral_radius_vs_alpha.png
- iss_metrics_vs_alpha.png
- snap_transition_trace.png
- hybrid_threshold_sensitivity.png
- alpha_memory_heatmap.png

## Key observations

1. Fixed-alpha augmented linear model is Schur stable for alpha in [0.1, 0.9] under the current baseline setup.
2. At alpha = 1.0, the spectral radius reaches 1.0 (marginal case in this linearized check).
3. Controllability rank equals state dimension (49) for all tested alpha values in this model.
4. Gramian minimum eigenvalue decreases strongly as alpha increases, indicating weaker numerical conditioning at high alpha.
5. In bounded random-input tests, peak velocity norm decreases as alpha increases, while reference excursion increases.
6. In hybrid threshold sweeps, increasing joystick active threshold increases mode-switching and snap-event counts while reducing mean dwell time.
7. In alpha-memory sweeps, max velocity is mostly alpha-dominated (weak sensitivity to memory length), while reference excursion increases strongly with memory length.

## Baseline table snapshots

### Stability (excerpt)

- alpha 0.1: spectral_radius 0.9379, stable = 1
- alpha 0.5: spectral_radius 0.9828, stable = 1
- alpha 0.9: spectral_radius 0.9977, stable = 1
- alpha 1.0: spectral_radius 1.0000, stable = 0

### Controllability (excerpt)

- rank_ratio = 1.0 for all tested alphas.
- gramian_min_eig decreases from about 3.466e-01 at alpha 0.1 to 2.503e-05 at alpha 1.0.

### ISS-style bounded-input metrics (excerpt)

- alpha 0.1: max velocity norm about 357.01
- alpha 0.5: max velocity norm about 45.43
- alpha 1.0: max velocity norm about 3.29

### Hybrid threshold sweep (excerpt)

- threshold 0.005: transitions 99, snap events 49, mean dwell 12.00 steps
- threshold 0.0307: transitions 178, snap events 89, mean dwell 6.70 steps
- threshold 0.0500: transitions 274, snap events 137, mean dwell 4.36 steps

### Alpha-memory sweep (excerpt)

- max velocity changes little with memory length at fixed alpha (example alpha 0.5: about 45.37 to 45.43 from memory 20 to 100)
- max reference norm grows with memory length at fixed alpha (example alpha 0.5: about 0.26 to 1.03)

## Interpretation caveats

- The linear checks use a fixed-alpha, augmented GL model and do not represent full hybrid behavior.
- Hybrid effects (drift thresholding and snap reset) are analyzed in simulation traces, not formal switched-system proofs.
- Variable-order behavior uses coefficient refresh without history re-identification, matching implementation but not exact variable-order theory.
