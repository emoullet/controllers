#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np

from fractional_node_model import (
    NodeAnalysisParams,
    augmented_linear_model,
    controllability_rank,
    finite_horizon_gramian,
    make_baseline_params,
    make_input_sequence,
    simulate_node,
    spectral_radius,
    summarize_trace,
)


def _write_csv(path: Path, rows: List[Dict[str, float]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def run_linear_stability_and_controllability(
    params: NodeAnalysisParams,
    alpha_grid: np.ndarray,
) -> tuple[List[Dict[str, float]], List[Dict[str, float]]]:
    stability_rows: List[Dict[str, float]] = []
    controllability_rows: List[Dict[str, float]] = []

    for alpha in alpha_grid:
        a, b = augmented_linear_model(
            alpha=float(alpha),
            memory_length=params.memory_length,
            dt=params.dt,
            gain_k=params.fractional_offset_gain,
        )
        rho = spectral_radius(a)
        rank = controllability_rank(a, b)
        n = a.shape[0]
        w = finite_horizon_gramian(a, b, horizon=max(10, n))
        min_eig = float(np.min(np.real(np.linalg.eigvals(w))))

        stability_rows.append(
            {
                "alpha": float(alpha),
                "state_dim": float(n),
                "spectral_radius": float(rho),
                "is_schur_stable": float(rho < 1.0),
            }
        )
        controllability_rows.append(
            {
                "alpha": float(alpha),
                "controllability_rank": float(rank),
                "state_dim": float(n),
                "rank_ratio": float(rank / n),
                "gramian_min_eig": float(min_eig),
            }
        )

    return stability_rows, controllability_rows


def run_iss_sweep(
    baseline: NodeAnalysisParams,
    alpha_grid: np.ndarray,
    n_steps: int,
    amplitude: float,
    seed: int,
) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    u = make_input_sequence(n_steps=n_steps, pattern="random", amplitude=amplitude, seed=seed)

    for alpha in alpha_grid:
        p = NodeAnalysisParams(**baseline.__dict__)
        p.use_dynamic_alpha = False
        p.alpha = float(alpha)
        result = simulate_node(p, joystick_linear=u)
        summary = summarize_trace(result)
        rows.append(
            {
                "alpha": float(alpha),
                "input_bound": float(amplitude),
                "max_velocity_norm": summary["max_velocity_norm"],
                "mean_velocity_norm": summary["mean_velocity_norm"],
                "max_reference_norm": summary["max_reference_norm"],
                "max_desired_norm": summary["max_desired_norm"],
                "snap_events": summary["snap_events"],
            }
        )

    return rows


def run_snap_transition_test(params: NodeAnalysisParams, n_steps: int) -> Dict[str, np.ndarray]:
    p = NodeAnalysisParams(**params.__dict__)
    p.use_dynamic_alpha = False
    u = make_input_sequence(n_steps=n_steps, pattern="pulse", amplitude=0.9)
    result = simulate_node(p, joystick_linear=u)
    vel_norm = np.linalg.norm(result["velocity_linear"], axis=1)
    des_norm = np.linalg.norm(result["desired_linear"], axis=1)

    return {
        "u_norm": np.linalg.norm(u, axis=1),
        "vel_norm": vel_norm,
        "des_norm": des_norm,
        "snap_event": result["snap_event"].astype(float),
        "alpha": result["alpha"],
    }


def _mean_dwell_time(active: np.ndarray) -> float:
    if active.size == 0:
        return 0.0
    runs: List[int] = []
    run_len = 1
    for idx in range(1, active.size):
        if active[idx] == active[idx - 1]:
            run_len += 1
        else:
            runs.append(run_len)
            run_len = 1
    runs.append(run_len)
    return float(np.mean(runs))


def _make_noisy_sine_input(
    n_steps: int,
    amplitude: float,
    noise_std: float,
    seed: int,
) -> np.ndarray:
    rng = np.random.default_rng(seed)
    t = np.arange(n_steps)
    u = np.zeros((n_steps, 3), dtype=float)
    base = amplitude * np.sin(2.0 * np.pi * t / max(30, n_steps // 8))
    noise = rng.normal(0.0, noise_std, size=n_steps)
    u[:, 0] = base + noise
    return u


def run_hybrid_threshold_sweep(
    baseline: NodeAnalysisParams,
    threshold_grid: np.ndarray,
    n_steps: int,
    seed: int,
) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    u = _make_noisy_sine_input(n_steps=n_steps, amplitude=0.06, noise_std=0.01, seed=seed)
    u_norm = np.linalg.norm(u, axis=1)

    for threshold in threshold_grid:
        p = NodeAnalysisParams(**baseline.__dict__)
        p.joystick_active_threshold = float(threshold)
        p.use_dynamic_alpha = True
        result = simulate_node(p, joystick_linear=u)

        active = u_norm > float(threshold)
        transitions = int(np.count_nonzero(active[1:] != active[:-1]))
        mean_dwell_steps = _mean_dwell_time(active)
        snap_idx = np.where(result["snap_event"])[0]
        des_norm = np.linalg.norm(result["desired_linear"], axis=1)

        contraction = []
        for idx in snap_idx:
            if idx <= 0:
                continue
            pre = des_norm[idx - 1]
            post = des_norm[idx]
            contraction.append(float(post / (pre + 1e-12)))

        rows.append(
            {
                "joystick_active_threshold": float(threshold),
                "transitions": float(transitions),
                "mean_dwell_steps": float(mean_dwell_steps),
                "snap_events": float(snap_idx.size),
                "mean_snap_contraction_ratio": float(np.mean(contraction)) if contraction else 0.0,
                "max_snap_contraction_ratio": float(np.max(contraction)) if contraction else 0.0,
            }
        )

    return rows


def run_alpha_memory_sweep(
    baseline: NodeAnalysisParams,
    alpha_grid: np.ndarray,
    memory_grid: np.ndarray,
    n_steps: int,
    amplitude: float,
    seed: int,
) -> tuple[List[Dict[str, float]], np.ndarray]:
    rows: List[Dict[str, float]] = []
    heatmap = np.zeros((alpha_grid.size, memory_grid.size), dtype=float)
    u = make_input_sequence(n_steps=n_steps, pattern="random", amplitude=amplitude, seed=seed)

    for i, alpha in enumerate(alpha_grid):
        for j, memory_len in enumerate(memory_grid):
            p = NodeAnalysisParams(**baseline.__dict__)
            p.use_dynamic_alpha = False
            p.alpha = float(alpha)
            p.memory_length = int(memory_len)
            result = simulate_node(p, joystick_linear=u)
            summary = summarize_trace(result)
            max_vel = summary["max_velocity_norm"]
            heatmap[i, j] = max_vel
            rows.append(
                {
                    "alpha": float(alpha),
                    "memory_length": float(memory_len),
                    "max_velocity_norm": float(max_vel),
                    "max_reference_norm": float(summary["max_reference_norm"]),
                }
            )

    return rows, heatmap


def plot_stability(stability_rows: List[Dict[str, float]], output_path: Path) -> None:
    alphas = np.array([r["alpha"] for r in stability_rows], dtype=float)
    rho = np.array([r["spectral_radius"] for r in stability_rows], dtype=float)

    fig, ax = plt.subplots(figsize=(7.5, 4.0), constrained_layout=True)
    ax.plot(alphas, rho, marker="o", label="Spectral radius")
    ax.axhline(1.0, color="red", linestyle="--", linewidth=1.2, label="Schur limit")
    ax.set_xlabel("alpha")
    ax.set_ylabel("spectral radius")
    ax.set_title("Augmented linear model stability vs alpha")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_iss(iss_rows: List[Dict[str, float]], output_path: Path) -> None:
    alphas = np.array([r["alpha"] for r in iss_rows], dtype=float)
    vmax = np.array([r["max_velocity_norm"] for r in iss_rows], dtype=float)
    rmax = np.array([r["max_reference_norm"] for r in iss_rows], dtype=float)

    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.0), constrained_layout=True)

    axes[0].plot(alphas, vmax, marker="o")
    axes[0].set_xlabel("alpha")
    axes[0].set_ylabel("max ||v||")
    axes[0].set_title("Bounded-input max velocity")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(alphas, rmax, marker="o", color="tab:orange")
    axes[1].set_xlabel("alpha")
    axes[1].set_ylabel("max ||x_ref||")
    axes[1].set_title("Bounded-input reference excursion")
    axes[1].grid(True, alpha=0.3)

    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_snap_trace(trace: Dict[str, np.ndarray], output_path: Path) -> None:
    t = np.arange(trace["u_norm"].size)
    fig, axes = plt.subplots(3, 1, figsize=(8.5, 7.0), constrained_layout=True, sharex=True)

    axes[0].plot(t, trace["u_norm"], color="black", label="||u||")
    axes[0].plot(t, trace["snap_event"], color="tab:red", linestyle="--", label="snap event")
    axes[0].set_ylabel("input")
    axes[0].set_title("Snap-on-release transition trace")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, trace["des_norm"], color="tab:blue")
    axes[1].set_ylabel("||x_des||")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, trace["vel_norm"], color="tab:green")
    axes[2].set_xlabel("step")
    axes[2].set_ylabel("||v||")
    axes[2].grid(True, alpha=0.3)

    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_hybrid_threshold(rows: List[Dict[str, float]], output_path: Path) -> None:
    thresholds = np.array([r["joystick_active_threshold"] for r in rows], dtype=float)
    transitions = np.array([r["transitions"] for r in rows], dtype=float)
    snaps = np.array([r["snap_events"] for r in rows], dtype=float)
    contraction = np.array([r["mean_snap_contraction_ratio"] for r in rows], dtype=float)

    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.2), constrained_layout=True)

    axes[0].plot(thresholds, transitions, marker="o", label="mode transitions")
    axes[0].plot(thresholds, snaps, marker="s", label="snap events")
    axes[0].set_xlabel("joystick_active_threshold")
    axes[0].set_ylabel("event count")
    axes[0].set_title("Hybrid event sensitivity")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].plot(thresholds, contraction, marker="o", color="tab:purple")
    axes[1].set_xlabel("joystick_active_threshold")
    axes[1].set_ylabel("mean snap contraction ratio")
    axes[1].set_title("Reset contraction trend")
    axes[1].grid(True, alpha=0.3)

    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_alpha_memory_heatmap(
    heatmap: np.ndarray,
    alpha_grid: np.ndarray,
    memory_grid: np.ndarray,
    output_path: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(8.0, 4.4), constrained_layout=True)
    img = ax.imshow(heatmap, aspect="auto", origin="lower", cmap="viridis")
    ax.set_xticks(np.arange(memory_grid.size))
    ax.set_xticklabels([str(int(m)) for m in memory_grid])
    ax.set_xlabel("memory_length")
    ax.set_yticks(np.arange(alpha_grid.size))
    ax.set_yticklabels([f"{a:.2f}" for a in alpha_grid])
    ax.set_ylabel("alpha")
    ax.set_title("Max velocity norm: alpha-memory sweep")
    cbar = fig.colorbar(img, ax=ax)
    cbar.set_label("max ||v||")
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run stability, controllability, boundedness and hybrid snap analysis for fractional teleoperation node model."
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "results",
        help="Output directory for CSV tables and figures.",
    )
    parser.add_argument("--alpha-min", type=float, default=0.1)
    parser.add_argument("--alpha-max", type=float, default=1.0)
    parser.add_argument("--alpha-count", type=int, default=10)
    parser.add_argument("--iss-steps", type=int, default=1500)
    parser.add_argument("--snap-steps", type=int, default=400)
    parser.add_argument("--hybrid-steps", type=int, default=1200)
    parser.add_argument("--input-bound", type=float, default=1.0)
    parser.add_argument("--memory-min", type=int, default=20)
    parser.add_argument("--memory-max", type=int, default=100)
    parser.add_argument("--memory-count", type=int, default=5)
    parser.add_argument("--threshold-min", type=float, default=0.005)
    parser.add_argument("--threshold-max", type=float, default=0.05)
    parser.add_argument("--threshold-count", type=int, default=8)
    parser.add_argument("--seed", type=int, default=7)
    return parser


def main() -> None:
    args = make_parser().parse_args()
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)

    baseline = make_baseline_params()
    alpha_grid = np.linspace(args.alpha_min, args.alpha_max, args.alpha_count)
    memory_grid = np.linspace(args.memory_min, args.memory_max, args.memory_count).astype(int)
    threshold_grid = np.linspace(args.threshold_min, args.threshold_max, args.threshold_count)

    stability_rows, controllability_rows = run_linear_stability_and_controllability(baseline, alpha_grid)
    iss_rows = run_iss_sweep(
        baseline=baseline,
        alpha_grid=alpha_grid,
        n_steps=args.iss_steps,
        amplitude=args.input_bound,
        seed=args.seed,
    )
    snap_trace = run_snap_transition_test(baseline, n_steps=args.snap_steps)
    hybrid_rows = run_hybrid_threshold_sweep(
        baseline=baseline,
        threshold_grid=threshold_grid,
        n_steps=args.hybrid_steps,
        seed=args.seed,
    )
    alpha_memory_rows, alpha_memory_heatmap = run_alpha_memory_sweep(
        baseline=baseline,
        alpha_grid=alpha_grid,
        memory_grid=memory_grid,
        n_steps=args.iss_steps,
        amplitude=args.input_bound,
        seed=args.seed,
    )

    _write_csv(out / "stability_table.csv", stability_rows)
    _write_csv(out / "controllability_table.csv", controllability_rows)
    _write_csv(out / "iss_table.csv", iss_rows)
    _write_csv(out / "hybrid_threshold_table.csv", hybrid_rows)
    _write_csv(out / "alpha_memory_sweep_table.csv", alpha_memory_rows)

    plot_stability(stability_rows, out / "spectral_radius_vs_alpha.png")
    plot_iss(iss_rows, out / "iss_metrics_vs_alpha.png")
    plot_snap_trace(snap_trace, out / "snap_transition_trace.png")
    plot_hybrid_threshold(hybrid_rows, out / "hybrid_threshold_sensitivity.png")
    plot_alpha_memory_heatmap(
        alpha_memory_heatmap,
        alpha_grid=alpha_grid,
        memory_grid=memory_grid,
        output_path=out / "alpha_memory_heatmap.png",
    )

    print("Generated analysis outputs:")
    print(f"- {out / 'stability_table.csv'}")
    print(f"- {out / 'controllability_table.csv'}")
    print(f"- {out / 'iss_table.csv'}")
    print(f"- {out / 'hybrid_threshold_table.csv'}")
    print(f"- {out / 'alpha_memory_sweep_table.csv'}")
    print(f"- {out / 'spectral_radius_vs_alpha.png'}")
    print(f"- {out / 'iss_metrics_vs_alpha.png'}")
    print(f"- {out / 'snap_transition_trace.png'}")
    print(f"- {out / 'hybrid_threshold_sensitivity.png'}")
    print(f"- {out / 'alpha_memory_heatmap.png'}")


if __name__ == "__main__":
    main()
