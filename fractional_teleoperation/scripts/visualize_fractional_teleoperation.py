#!/usr/bin/env python3

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


@dataclass
class SimulationConfig:
    dt: float
    memory_length: int
    gain_k: float
    velocity_scale: float


def compute_grunwald_coefficients(memory_length: int, alpha: float) -> np.ndarray:
    coefficients = np.zeros(memory_length, dtype=float)
    coefficients[0] = 1.0
    for k in range(1, memory_length):
        coefficients[k] = coefficients[k - 1] * (k - 1.0 - alpha) / k
    return coefficients


def apply_fractional_integration(
    input_velocity: np.ndarray,
    history: deque[np.ndarray],
    gl_coefficients: np.ndarray,
    cfg: SimulationConfig,
    alpha: float,
) -> np.ndarray:
    weighted_sum = np.zeros(2, dtype=float)
    num_samples = min(len(history), len(gl_coefficients))
    for k in range(1, num_samples):
        weighted_sum += gl_coefficients[k] * history[k - 1]

    integrated = (cfg.dt**alpha) * cfg.gain_k * input_velocity - weighted_sum

    history.appendleft(integrated)
    if len(history) > cfg.memory_length:
        history.pop()

    return integrated


def simulate_response(
    joystick: np.ndarray,
    alpha_values: list[float],
    cfg: SimulationConfig,
) -> dict[float, dict[str, np.ndarray]]:
    responses: dict[float, dict[str, np.ndarray]] = {}

    for alpha in alpha_values:
        coeffs = compute_grunwald_coefficients(cfg.memory_length, alpha)
        history: deque[np.ndarray] = deque(maxlen=cfg.memory_length)
        desired = np.zeros_like(joystick)
        output_velocity = np.zeros_like(joystick)
        previous_desired = np.zeros(2, dtype=float)

        for idx, u in enumerate(joystick):
            desired_k = apply_fractional_integration(
                input_velocity=u,
                history=history,
                gl_coefficients=coeffs,
                cfg=cfg,
                alpha=alpha,
            )
            velocity_k = (desired_k - previous_desired) / cfg.dt
            previous_desired = desired_k

            desired[idx] = desired_k
            output_velocity[idx] = cfg.velocity_scale * velocity_k

        responses[alpha] = {"desired": desired, "velocity": output_velocity}

    return responses


def make_circle_input(time: np.ndarray, radius: float, linear_velocity: float) -> np.ndarray:
    if radius == 0.0:
        return np.zeros((time.size, 2), dtype=float)

    omega = linear_velocity / radius
    joystick = np.column_stack((radius * np.cos(omega * time), radius * np.sin(omega * time)))
    return joystick


def make_step_input(time: np.ndarray, step_time: float, step_size: float) -> np.ndarray:
    joystick = np.zeros((time.size, 2), dtype=float)
    joystick[time >= step_time, 0] = step_size
    return joystick


def _validate_args(args: argparse.Namespace) -> None:
    if not 0.0 <= args.circle_radius <= 1.0:
        raise ValueError("--circle-radius must be in [0, 1].")
    if args.circle_velocity < 0.0:
        raise ValueError("--circle-velocity must be >= 0.")
    if not 0.0 <= args.step_size <= 1.0:
        raise ValueError("--step-size must be in [0, 1].")
    if args.dt <= 0.0:
        raise ValueError("--dt must be > 0.")
    if args.memory_length < 2:
        raise ValueError("--memory-length must be >= 2.")
    if args.duration_circle <= 0.0 or args.duration_step <= 0.0:
        raise ValueError("Scenario durations must be > 0.")
    if args.step_time < 0.0 or args.step_time >= args.duration_step:
        raise ValueError("--step-time must be in [0, --duration-step).")

    for alpha in args.alphas:
        if not 0.0 <= alpha < 2.0:
            raise ValueError(f"Alpha value {alpha} is out of bounds. Expected 0 <= alpha < 2.")


def _plot_circle(
    joystick: np.ndarray,
    responses: dict[float, dict[str, np.ndarray]],
    output_path: Path,
) -> None:
    num_plots = 1 + len(responses)
    num_cols = min(3, num_plots)
    num_rows = int(np.ceil(num_plots / num_cols))
    fig, axes = plt.subplots(
        num_rows,
        num_cols,
        figsize=(4.8 * num_cols, 4.0 * num_rows),
        constrained_layout=True,
    )
    axes_flat = np.atleast_1d(axes).ravel()

    joystick_ax = axes_flat[0]
    joystick_ax.plot(joystick[:, 0], joystick[:, 1], color="black", linewidth=2)
    joystick_ax.set_title("Joystick trajectory (circle)")
    joystick_ax.set_xlabel("u_x")
    joystick_ax.set_ylabel("u_y")
    joystick_ax.axis("equal")
    joystick_ax.grid(True, alpha=0.3)

    for idx, (alpha, data) in enumerate(responses.items(), start=1):
        ax = axes_flat[idx]
        desired_with_origin = np.vstack((np.zeros((1, 2), dtype=float), data["desired"]))
        line, = ax.plot(desired_with_origin[:, 0], desired_with_origin[:, 1])
        ax.plot(
            desired_with_origin[0, 0],
            desired_with_origin[0, 1],
            marker="o",
            markersize=6,
            color=line.get_color(),
        )
        ax.set_title(f"Desired trajectory (alpha={alpha:g})")
        ax.set_xlabel("x_d")
        ax.set_ylabel("y_d")
        ax.axis("equal")
        ax.grid(True, alpha=0.3)

    for idx in range(num_plots, len(axes_flat)):
        axes_flat[idx].set_visible(False)

    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def _plot_step(
    time: np.ndarray,
    joystick: np.ndarray,
    responses: dict[float, dict[str, np.ndarray]],
    output_path: Path,
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)

    axes[0, 0].plot(time, joystick[:, 0], color="black", linewidth=2)
    axes[0, 0].set_title("Joystick step input (x-axis)")
    axes[0, 0].set_xlabel("time [s]")
    axes[0, 0].set_ylabel("u_x")
    axes[0, 0].grid(True, alpha=0.3)

    for alpha, data in responses.items():
        axes[0, 1].plot(time, data["desired"][:, 0], label=f"alpha={alpha:g}")
    axes[0, 1].set_title("Desired position x_d")
    axes[0, 1].set_xlabel("time [s]")
    axes[0, 1].set_ylabel("x_d")
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend(loc="best")

    for alpha, data in responses.items():
        axes[1, 0].plot(time, data["velocity"][:, 0], label=f"alpha={alpha:g}")
    axes[1, 0].set_title("Output velocity v_x")
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 0].set_ylabel("v_x")
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend(loc="best")

    for alpha, data in responses.items():
        axes[1, 1].plot(time, np.abs(data["velocity"][:, 0]), label=f"alpha={alpha:g}")
    axes[1, 1].set_title("|v_x| (response magnitude)")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].set_ylabel("|v_x|")
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend(loc="best")

    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Visualize fractional teleoperation response for multiple alpha values "
            "using joystick circle and step scenarios."
        )
    )
    parser.add_argument("--alphas", nargs="+", type=float, default=[0.0, 0.2, 0.5, 0.8, 1.0])

    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--memory-length", type=int, default=50)
    parser.add_argument("--gain-k", type=float, default=1.0)
    parser.add_argument("--velocity-scale", type=float, default=1.0)

    parser.add_argument("--duration-circle", type=float, default=8.0)
    parser.add_argument("--circle-radius", type=float, default=0.6)
    parser.add_argument(
        "--circle-velocity",
        type=float,
        default=0.8,
        help="Linear speed on the joystick circle (units/s).",
    )

    parser.add_argument("--duration-step", type=float, default=3.0)
    parser.add_argument("--step-time", type=float, default=0.5)
    parser.add_argument("--step-size", type=float, default=0.8)

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "fig",
        help="Directory where figures are saved.",
    )

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    _validate_args(args)

    cfg = SimulationConfig(
        dt=args.dt,
        memory_length=args.memory_length,
        gain_k=args.gain_k,
        velocity_scale=args.velocity_scale,
    )

    args.output_dir.mkdir(parents=True, exist_ok=True)

    time_circle = np.arange(0.0, args.duration_circle, args.dt)
    joystick_circle = make_circle_input(
        time=time_circle,
        radius=args.circle_radius,
        linear_velocity=args.circle_velocity,
    )
    responses_circle = simulate_response(joystick_circle, args.alphas, cfg)
    circle_path = args.output_dir / "fractional_circle_response.png"
    _plot_circle(joystick_circle, responses_circle, circle_path)

    time_step = np.arange(0.0, args.duration_step, args.dt)
    joystick_step = make_step_input(time_step, args.step_time, args.step_size)
    responses_step = simulate_response(joystick_step, args.alphas, cfg)
    step_path = args.output_dir / "fractional_step_response.png"
    _plot_step(time_step, joystick_step, responses_step, step_path)

    print(f"Saved figures:\n- {circle_path}\n- {step_path}")


if __name__ == "__main__":
    main()
