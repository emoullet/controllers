#!/usr/bin/env python3

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Dict, List, Tuple

import numpy as np


@dataclass
class NodeAnalysisParams:
    alpha: float = 0.5
    fractional_offset_gain: float = 1.0
    memory_length: int = 50
    dt: float = 0.01
    output_velocity_scale: float = 1.0
    normalize_gain_for_alpha: bool = False
    alpha_gain_normalization_mode: str = "perceptual"
    v_max: float = 1.0
    t_ref: float = 0.2
    k_0: float = 1.0
    k_1: float = 1.0
    use_dynamic_alpha: bool = True
    alpha_min: float = 0.1
    alpha_max: float = 1.0
    l_0: float = 0.5
    l_max: float = 1.0
    linear_offset_scale_max: float = 2.0
    angular_offset_scale_max: float = 1.0
    offset_ramp_time: float = 2.5
    offset_ramp_profile: str = "exponential"
    joystick_active_threshold: float = 0.01
    snap_reference_on_release: bool = True
    use_reference_drift: bool = True
    reference_drift_joystick_threshold: float = 0.5
    reference_first_order_rate: float = 0.5
    reference_update_mode: str = "fractional"
    reference_fractional_alpha: float = 0.5
    reference_fractional_gain: float = 0.5


RAMP_PROFILES = {
    "linear",
    "quadratic",
    "smoothstep",
    "sigmoid",
    "exponential",
    "inverse_exponential",
    "tanh",
    "cubic_hermite",
    "step",
}


def compute_grunwald_coefficients(memory_length: int, alpha: float) -> np.ndarray:
    coeffs = np.zeros(memory_length, dtype=float)
    coeffs[0] = 1.0
    for k in range(1, memory_length):
        coeffs[k] = coeffs[k - 1] * (k - 1.0 - alpha) / k
    return coeffs


def compute_dynamic_alpha(
    joystick_norm: float,
    l_0: float,
    l_max: float,
    alpha_min: float,
    alpha_max: float,
) -> float:
    if joystick_norm < l_0:
        return alpha_min
    if joystick_norm > l_max:
        return alpha_max
    if l_max <= l_0:
        return alpha_max
    ratio = (joystick_norm - l_0) / (l_max - l_0)
    return alpha_min + ratio * (alpha_max - alpha_min)


def compute_adaptive_gain(alpha: float, params: NodeAnalysisParams) -> float:
    if params.alpha_gain_normalization_mode == "geometric_transition":
        return params.k_0 ** (1.0 - alpha) * params.k_1 ** alpha
    if params.alpha_gain_normalization_mode == "perceptual":
        alpha_gamma = max(alpha, 1e-3)
        return params.v_max * math.gamma(alpha_gamma) / (params.t_ref ** (alpha_gamma - 1.0))
    return params.v_max * (params.dt ** (1.0 - alpha))


def compute_ramp_factor(t_ratio: float, profile: str) -> float:
    t = max(0.0, min(1.0, t_ratio))
    if profile == "linear":
        return t
    if profile == "quadratic":
        return t * t
    if profile == "smoothstep":
        return t * t * (3.0 - 2.0 * t)
    if profile == "sigmoid":
        return 1.0 / (1.0 + math.exp(-10.0 * (t - 0.5)))
    if profile == "exponential":
        return 1.0 - math.exp(-5.0 * t)
    if profile == "inverse_exponential":
        one_minus_t = 1.0 - t
        return 1.0 - (one_minus_t * one_minus_t * one_minus_t)
    if profile == "tanh":
        return (math.tanh(6.0 * (t - 0.5)) + 1.0) / 2.0
    if profile == "cubic_hermite":
        return 3.0 * t * t - 2.0 * t * t * t
    if profile == "step":
        return 1.0 if t >= 1.0 else 0.0
    raise ValueError(f"Unknown ramp profile: {profile}")


def apply_fractional_integration(
    joystick_input: np.ndarray,
    history: deque[np.ndarray],
    coeffs: np.ndarray,
    memory_length: int,
    dt: float,
    alpha: float,
    gain_k: float,
) -> np.ndarray:
    weighted = np.zeros(3, dtype=float)
    num_samples = min(len(history), len(coeffs))
    for k in range(1, num_samples):
        weighted += coeffs[k] * history[k - 1]

    integrated = (dt ** alpha) * gain_k * joystick_input - weighted

    history.appendleft(integrated)
    if len(history) > memory_length:
        history.pop()

    return integrated


def update_reference_first_order(
    current_reference: np.ndarray,
    target: np.ndarray,
    shift_rate: float,
    dt: float,
) -> np.ndarray:
    blend = max(0.0, min(1.0, max(0.0, shift_rate) * dt))
    return current_reference + blend * (target - current_reference)


def update_reference_fractional(
    current_reference: np.ndarray,
    target: np.ndarray,
    history: deque[np.ndarray],
    coeffs: np.ndarray,
    params: NodeAnalysisParams,
) -> np.ndarray:
    error = target - current_reference
    step = apply_fractional_integration(
        joystick_input=error,
        history=history,
        coeffs=coeffs,
        memory_length=params.memory_length,
        dt=params.dt,
        alpha=params.reference_fractional_alpha,
        gain_k=params.reference_fractional_gain,
    )
    return current_reference + params.dt * step


def sanitize_params(params: NodeAnalysisParams) -> NodeAnalysisParams:
    p = NodeAnalysisParams(**params.__dict__)
    p.alpha = min(1.9, max(0.1, p.alpha))
    p.reference_fractional_alpha = min(1.9, max(0.1, p.reference_fractional_alpha))
    p.alpha_min = max(0.0, p.alpha_min)
    p.alpha_max = min(1.9, p.alpha_max)
    if p.alpha_min > p.alpha_max:
        p.alpha_min = p.alpha_max
    p.memory_length = max(2, p.memory_length)
    p.dt = max(1e-6, p.dt)
    p.v_max = max(1e-6, p.v_max)
    p.t_ref = max(1e-6, p.t_ref)
    p.k_0 = max(1e-6, p.k_0)
    p.k_1 = max(1e-6, p.k_1)
    p.reference_first_order_rate = max(0.0, p.reference_first_order_rate)
    p.reference_drift_joystick_threshold = max(0.0, p.reference_drift_joystick_threshold)
    p.joystick_active_threshold = max(0.0, p.joystick_active_threshold)
    p.linear_offset_scale_max = max(0.0, p.linear_offset_scale_max)
    p.angular_offset_scale_max = max(0.0, p.angular_offset_scale_max)
    p.offset_ramp_time = max(0.0, p.offset_ramp_time)
    p.alpha_gain_normalization_mode = p.alpha_gain_normalization_mode.lower()
    if p.alpha_gain_normalization_mode not in {"dt", "perceptual", "geometric_transition"}:
        p.alpha_gain_normalization_mode = "dt"
    p.reference_update_mode = p.reference_update_mode.lower()
    if p.reference_update_mode not in {"first_order", "fractional"}:
        p.reference_update_mode = "first_order"
    p.offset_ramp_profile = p.offset_ramp_profile.lower()
    if p.offset_ramp_profile not in RAMP_PROFILES:
        p.offset_ramp_profile = "sigmoid"
    return p


def make_input_sequence(
    n_steps: int,
    pattern: str,
    amplitude: float,
    seed: int = 0,
) -> np.ndarray:
    u = np.zeros((n_steps, 3), dtype=float)
    if pattern == "step":
        u[n_steps // 4 :, 0] = amplitude
    elif pattern == "pulse":
        start = n_steps // 5
        stop = start + max(2, n_steps // 5)
        u[start:stop, 0] = amplitude
    elif pattern == "random":
        rng = np.random.default_rng(seed)
        u = rng.uniform(-amplitude, amplitude, size=(n_steps, 3))
    elif pattern == "sine":
        t = np.arange(n_steps)
        u[:, 0] = amplitude * np.sin(2.0 * np.pi * t / max(10, n_steps // 5))
    else:
        raise ValueError(f"Unsupported pattern: {pattern}")
    return u


def simulate_node(
    params: NodeAnalysisParams,
    joystick_linear: np.ndarray,
    joystick_angular: np.ndarray | None = None,
) -> Dict[str, np.ndarray]:
    p = sanitize_params(params)
    n_steps = joystick_linear.shape[0]
    if joystick_angular is None:
        joystick_angular = np.zeros_like(joystick_linear)

    current_alpha = 0.0 if p.use_dynamic_alpha else p.alpha
    last_alpha = current_alpha
    gain_k = p.fractional_offset_gain
    if p.normalize_gain_for_alpha:
        gain_k = compute_adaptive_gain(current_alpha, p)

    coeffs = compute_grunwald_coefficients(p.memory_length, current_alpha)
    ref_coeffs = compute_grunwald_coefficients(p.memory_length, p.reference_fractional_alpha)

    lin_hist: deque[np.ndarray] = deque(maxlen=p.memory_length)
    ang_hist: deque[np.ndarray] = deque(maxlen=p.memory_length)
    ref_lin_hist: deque[np.ndarray] = deque(maxlen=p.memory_length)
    ref_ang_hist: deque[np.ndarray] = deque(maxlen=p.memory_length)

    frac_lin = np.zeros(3, dtype=float)
    frac_ang = np.zeros(3, dtype=float)
    ref_lin = np.zeros(3, dtype=float)
    ref_ang = np.zeros(3, dtype=float)

    joystick_was_active = False
    joystick_active_duration = 0.0
    last_linear_scale = 0.0
    last_angular_scale = 0.0

    alpha_trace = np.zeros(n_steps, dtype=float)
    gain_trace = np.zeros(n_steps, dtype=float)
    linear_scale_trace = np.zeros(n_steps, dtype=float)
    angular_scale_trace = np.zeros(n_steps, dtype=float)
    desired_linear = np.zeros((n_steps, 3), dtype=float)
    desired_angular = np.zeros((n_steps, 3), dtype=float)
    reference_linear = np.zeros((n_steps, 3), dtype=float)
    reference_angular = np.zeros((n_steps, 3), dtype=float)
    velocity_linear = np.zeros((n_steps, 3), dtype=float)
    velocity_angular = np.zeros((n_steps, 3), dtype=float)
    snap_event = np.zeros(n_steps, dtype=bool)

    for k in range(n_steps):
        jl = joystick_linear[k]
        ja = joystick_angular[k]

        if p.use_dynamic_alpha:
            new_alpha = compute_dynamic_alpha(np.linalg.norm(jl), p.l_0, p.l_max, p.alpha_min, p.alpha_max)
            current_alpha = new_alpha
            if abs(new_alpha - last_alpha) > 1e-3:
                last_alpha = new_alpha
                coeffs = compute_grunwald_coefficients(p.memory_length, current_alpha)

        if p.normalize_gain_for_alpha:
            gain_k = compute_adaptive_gain(current_alpha, p)

        prev_scaled_lin = frac_lin * last_linear_scale
        prev_scaled_ang = frac_ang * last_angular_scale
        pre_update_lin = frac_lin.copy()
        pre_update_ang = frac_ang.copy()

        frac_lin = apply_fractional_integration(
            joystick_input=jl,
            history=lin_hist,
            coeffs=coeffs,
            memory_length=p.memory_length,
            dt=p.dt,
            alpha=current_alpha,
            gain_k=gain_k,
        )
        frac_ang = apply_fractional_integration(
            joystick_input=ja,
            history=ang_hist,
            coeffs=coeffs,
            memory_length=p.memory_length,
            dt=p.dt,
            alpha=current_alpha,
            gain_k=gain_k,
        )

        joystick_active = np.linalg.norm(jl) > p.joystick_active_threshold
        if joystick_active:
            joystick_active_duration += p.dt
        else:
            joystick_active_duration = 0.0

        t_ratio = 1.0
        if p.offset_ramp_time > 0.0:
            t_ratio = min(1.0, joystick_active_duration / p.offset_ramp_time)
        ramp = compute_ramp_factor(t_ratio, p.offset_ramp_profile)
        current_linear_scale = p.linear_offset_scale_max * ramp
        current_angular_scale = p.angular_offset_scale_max * ramp

        scaled_lin = frac_lin * current_linear_scale
        scaled_ang = frac_ang * current_angular_scale

        desired_lin = ref_lin + scaled_lin
        desired_ang = ref_ang + scaled_ang

        if p.snap_reference_on_release and joystick_was_active and (not joystick_active):
            ref_lin = ref_lin + pre_update_lin * last_linear_scale
            ref_ang = ref_ang + pre_update_ang * last_angular_scale
            frac_lin = np.zeros(3, dtype=float)
            frac_ang = np.zeros(3, dtype=float)
            lin_hist.clear()
            ang_hist.clear()
            prev_scaled_lin = np.zeros(3, dtype=float)
            prev_scaled_ang = np.zeros(3, dtype=float)
            scaled_lin = np.zeros(3, dtype=float)
            scaled_ang = np.zeros(3, dtype=float)
            desired_lin = ref_lin.copy()
            desired_ang = ref_ang.copy()
            snap_event[k] = True

        joystick_was_active = joystick_active
        last_linear_scale = current_linear_scale
        last_angular_scale = current_angular_scale

        if p.use_reference_drift and np.linalg.norm(jl) >= p.reference_drift_joystick_threshold:
            if p.reference_update_mode == "fractional":
                ref_lin = update_reference_fractional(ref_lin, desired_lin, ref_lin_hist, ref_coeffs, p)
                ref_ang = update_reference_fractional(ref_ang, desired_ang, ref_ang_hist, ref_coeffs, p)
            else:
                ref_lin = update_reference_first_order(ref_lin, desired_lin, p.reference_first_order_rate, p.dt)
                ref_ang = update_reference_first_order(ref_ang, desired_ang, p.reference_first_order_rate, p.dt)

        vel_lin = (scaled_lin - prev_scaled_lin) / p.dt
        vel_ang = (scaled_ang - prev_scaled_ang) / p.dt

        alpha_trace[k] = current_alpha
        gain_trace[k] = gain_k
        linear_scale_trace[k] = current_linear_scale
        angular_scale_trace[k] = current_angular_scale
        desired_linear[k] = desired_lin
        desired_angular[k] = desired_ang
        reference_linear[k] = ref_lin
        reference_angular[k] = ref_ang
        velocity_linear[k] = p.output_velocity_scale * vel_lin
        velocity_angular[k] = p.output_velocity_scale * vel_ang

    return {
        "alpha": alpha_trace,
        "gain": gain_trace,
        "linear_scale": linear_scale_trace,
        "angular_scale": angular_scale_trace,
        "desired_linear": desired_linear,
        "desired_angular": desired_angular,
        "reference_linear": reference_linear,
        "reference_angular": reference_angular,
        "velocity_linear": velocity_linear,
        "velocity_angular": velocity_angular,
        "snap_event": snap_event,
    }


def augmented_linear_model(alpha: float, memory_length: int, dt: float, gain_k: float) -> Tuple[np.ndarray, np.ndarray]:
    n = max(1, memory_length - 1)
    coeffs = compute_grunwald_coefficients(memory_length, alpha)

    a = np.zeros((n, n), dtype=float)
    b = np.zeros((n, 1), dtype=float)

    for idx in range(n):
        a[0, idx] = -coeffs[idx + 1]
    if n > 1:
        a[1:, :-1] = np.eye(n - 1)

    b[0, 0] = (dt ** alpha) * gain_k
    return a, b


def controllability_rank(a: np.ndarray, b: np.ndarray) -> int:
    n = a.shape[0]
    ctrb = b
    power = np.eye(n)
    for _ in range(1, n):
        power = power @ a
        ctrb = np.hstack((ctrb, power @ b))
    return int(np.linalg.matrix_rank(ctrb))


def spectral_radius(a: np.ndarray) -> float:
    eigvals = np.linalg.eigvals(a)
    return float(np.max(np.abs(eigvals)))


def finite_horizon_gramian(a: np.ndarray, b: np.ndarray, horizon: int) -> np.ndarray:
    n = a.shape[0]
    w = np.zeros((n, n), dtype=float)
    power = np.eye(n)
    for _ in range(horizon):
        w += power @ b @ b.T @ power.T
        power = a @ power
    return w


def summarize_trace(result: Dict[str, np.ndarray]) -> Dict[str, float]:
    vel_norm = np.linalg.norm(result["velocity_linear"], axis=1)
    ref_norm = np.linalg.norm(result["reference_linear"], axis=1)
    des_norm = np.linalg.norm(result["desired_linear"], axis=1)
    alpha = result["alpha"]
    snap_count = int(np.count_nonzero(result["snap_event"]))

    return {
        "max_velocity_norm": float(np.max(vel_norm)),
        "mean_velocity_norm": float(np.mean(vel_norm)),
        "max_reference_norm": float(np.max(ref_norm)),
        "max_desired_norm": float(np.max(des_norm)),
        "alpha_min": float(np.min(alpha)),
        "alpha_max": float(np.max(alpha)),
        "snap_events": float(snap_count),
    }


def make_baseline_params() -> NodeAnalysisParams:
    return NodeAnalysisParams()
