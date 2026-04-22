#include "fractional_teleoperation/fractional_teleoperation_core.hpp"

#include <algorithm>
#include <cmath>

namespace fractional_teleoperation::core
{
std::vector<double> computeGrunwaldCoefficients(int memory_length, double alpha)
{
  std::vector<double> gl_coefficients;
  gl_coefficients.reserve(memory_length);

  gl_coefficients.push_back(1.0);
  for (int k = 1; k < memory_length; ++k)
  {
    const double c_k = gl_coefficients[k - 1] * (k - 1.0 - alpha) / k;
    gl_coefficients.push_back(c_k);
  }

  return gl_coefficients;
}

double computeDynamicAlpha(double lambda, double l_0, double l_max, double alpha_min, double alpha_max)
{
  if (lambda < l_0)
  {
    return alpha_min;
  }

  if (lambda > l_max)
  {
    return alpha_max;
  }

  const double ratio = (lambda - l_0) / (l_max - l_0);
  return alpha_min + ratio * (alpha_max - alpha_min);
}

Eigen::Vector3d applyFractionalIntegration(
    const Eigen::Vector3d & joystick_input,
    std::deque<Eigen::Vector3d> & history,
    const std::vector<double> & gl_coefficients,
    int memory_length,
    double dt,
    double alpha,
    double gain_K)
{
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();

  const size_t num_samples = std::min(history.size(), gl_coefficients.size());
  for (size_t k = 1; k < num_samples; ++k)
  {
    sum += gl_coefficients[k] * history[k - 1];
  }

  const double dt_alpha = std::pow(dt, alpha);
  const Eigen::Vector3d integrated_value = dt_alpha * gain_K * joystick_input - sum;

  history.push_front(integrated_value);
  if (history.size() > static_cast<size_t>(memory_length))
  {
    history.pop_back();
  }

  return integrated_value;
}

Eigen::Vector3d computeVelocityFromDesiredPosition(
    const Eigen::Vector3d & desired_position,
    const Eigen::Vector3d & previous_desired_position,
    double dt)
{
  return (desired_position - previous_desired_position) / dt;
}

Eigen::Vector3d updateReferencePosition(
    const Eigen::Vector3d & current_reference,
    const Eigen::Vector3d & target_position,
    double shift_rate,
    double dt)
{
  const double clamped_shift_rate = std::max(0.0, shift_rate);
  const double blend = std::clamp(clamped_shift_rate * dt, 0.0, 1.0);
  return current_reference + blend * (target_position - current_reference);
}

Eigen::Vector3d updateReferencePositionFractional(
    const Eigen::Vector3d & current_reference,
    const Eigen::Vector3d & target_position,
    std::deque<Eigen::Vector3d> & history,
    const std::vector<double> & gl_coefficients,
    int memory_length,
    double dt,
    double alpha,
    double gain_K)
{
  const Eigen::Vector3d reference_error = target_position - current_reference;
  const Eigen::Vector3d fractional_step = applyFractionalIntegration(
      reference_error,
      history,
      gl_coefficients,
      memory_length,
      dt,
      alpha,
      gain_K);

  return current_reference + dt * fractional_step;
}

double computeAdaptiveGainK(
    double current_alpha,
    double v_max,
    double t_ref,
    double dt,
    const std::string & adaptive_gain_mode)
{
  if (adaptive_gain_mode == "perceptual")
  {
    const double alpha_for_gamma = std::max(current_alpha, 1e-3);
    return v_max * std::tgamma(alpha_for_gamma) / std::pow(t_ref, alpha_for_gamma - 1.0);
  }

  return v_max * std::pow(dt, 1.0 - current_alpha);
}

void updateDynamicAlphaAndCoefficients(
    bool use_dynamic_alpha,
    const Eigen::Vector3d & joystick_linear,
    double l_0,
    double l_max,
  double alpha_min,
    double alpha_max,
    double alpha_threshold,
    double & current_alpha,
    double & last_alpha,
    int memory_length,
    std::vector<double> & gl_coefficients)
{
  if (!use_dynamic_alpha)
  {
    return;
  }

  const double lambda = joystick_linear.norm();
  const double new_alpha = computeDynamicAlpha(lambda, l_0, l_max, alpha_min, alpha_max);

  current_alpha = new_alpha;
  if (std::abs(new_alpha - last_alpha) > alpha_threshold)
  {
    last_alpha = new_alpha;
    gl_coefficients = computeGrunwaldCoefficients(memory_length, current_alpha);
  }
}

void transformApplyModeAndScale(
    Eigen::Vector3d & cartesian_linear_velocity,
    Eigen::Vector3d & cartesian_angular_velocity,
    const std::string & input_frame,
    const Eigen::Quaterniond & current_orientation,
    uint8_t mode,
    uint8_t translation_mode,
    uint8_t rotation_mode,
    double velocity_scale)
{
  if (input_frame == "ee")
  {
    const Eigen::Matrix3d R_BE = current_orientation.toRotationMatrix();
    cartesian_linear_velocity = R_BE * cartesian_linear_velocity;
    cartesian_angular_velocity = R_BE * cartesian_angular_velocity;
  }

  if (mode == translation_mode)
  {
    cartesian_angular_velocity.setZero();
  }
  else if (mode == rotation_mode)
  {
    cartesian_linear_velocity.setZero();
  }

  cartesian_linear_velocity *= velocity_scale;
  cartesian_angular_velocity *= velocity_scale;
}

}  // namespace fractional_teleoperation::core
