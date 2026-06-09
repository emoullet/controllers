#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace fractional_teleoperation::core
{
std::vector<double> computeGrunwaldCoefficients(int memory_length, double alpha);

double computeDynamicAlpha(double lambda, double l_0, double l_max, double alpha_min, double alpha_max);

Eigen::Vector3d applyFractionalIntegration(
    const Eigen::Vector3d & input_velocity,
    std::deque<Eigen::Vector3d> & history,
    const std::vector<double> & gl_coefficients,
    int memory_length,
    double dt,
    double alpha,
    double gain_K);

Eigen::Vector3d computeVelocityFromDesiredPosition(
    const Eigen::Vector3d & desired_position,
    const Eigen::Vector3d & previous_desired_position,
    double dt);

Eigen::Vector3d updateReferencePosition(
    const Eigen::Vector3d & current_reference,
    const Eigen::Vector3d & target_position,
    double shift_rate,
    double dt);

Eigen::Vector3d updateReferencePositionFractional(
    const Eigen::Vector3d & current_reference,
    const Eigen::Vector3d & target_position,
    std::deque<Eigen::Vector3d> & history,
    const std::vector<double> & gl_coefficients,
    int memory_length,
    double dt,
    double alpha,
    double gain_K);

double computeAdaptiveGainK(
    double current_alpha,
    double v_max,
    double t_ref,
    double dt,
    double k_0,
    double k_1,
    const std::string & adaptive_gain_mode);

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
    std::vector<double> & gl_coefficients);

void transformApplyModeAndScale(
    Eigen::Vector3d & cartesian_linear_velocity,
    Eigen::Vector3d & cartesian_angular_velocity,
    const std::string & input_frame,
    const Eigen::Quaterniond & current_orientation,
    uint8_t mode,
    uint8_t translation_mode,
    uint8_t rotation_mode,
    double velocity_scale);

}  // namespace fractional_teleoperation::core
