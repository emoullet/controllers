#ifndef FRACTIONAL_TELEOPERATION_RAMP_PROFILE_HPP
#define FRACTIONAL_TELEOPERATION_RAMP_PROFILE_HPP

#include <string>
#include <cmath>
#include <stdexcept>

namespace fractional_teleoperation
{
namespace ramp
{

/**
 * Compute ramp factor using specified profile.
 * 
 * @param t_ratio Time ratio [0, 1] (t / ramp_time)
 * @param profile Name of ramp profile: "linear", "quadratic", "smoothstep", "sigmoid", 
 *                "exponential", "inverse_exponential", "tanh", "cubic_hermite"
 * @return Ramp factor in [0, 1]
 * @throws std::invalid_argument if profile is not recognized
 */
double computeRampFactor(double t_ratio, const std::string& profile);

/**
 * Linear ramp: constant velocity rise
 * scale(t) = t_ratio
 */
double rampLinear(double t_ratio);

/**
 * Quadratic ramp: smooth start
 * scale(t) = t_ratio^2
 */
double rampQuadratic(double t_ratio);

/**
 * Smoothstep (cubic Hermite): smooth position, velocity, and acceleration
 * scale(t) = t_ratio^2 * (3 - 2*t_ratio)
 */
double rampSmoothstep(double t_ratio);

/**
 * Sigmoid (S-curve): smooth symmetric profile
 * scale(t) = 1 / (1 + exp(-10*(t_ratio - 0.5)))
 */
double rampSigmoid(double t_ratio);

/**
 * Exponential: fast onset with asymptotic approach
 * scale(t) = 1 - exp(-5*t_ratio)
 */
double rampExponential(double t_ratio);

/**
 * Inverse exponential: gentle slow start
 * scale(t) = 1 - (1 - t_ratio)^3
 */
double rampInverseExponential(double t_ratio);

/**
 * Tanh ramp: sigmoid alternative, cheaper variant
 * scale(t) = (tanh(6*(t_ratio - 0.5)) + 1) / 2
 */
double rampTanh(double t_ratio);

/**
 * Cubic Hermite: zero velocity at boundaries
 * scale(t) = 3*t_ratio^2 - 2*t_ratio^3
 */
double rampCubicHermite(double t_ratio);

/**
 * Step function: instantaneous rise
 * scale(t) = t_ratio >= 1 ? 1 : 0
 * (Effectively just clamps to 1 when t_ratio reaches 1)
 */
double rampStep(double t_ratio);

/**
 * Validate and normalize profile name
 * 
 * @param profile Profile name (case-insensitive)
 * @return Lowercase normalized profile name
 * @throws std::invalid_argument if profile is not recognized
 */
std::string validateProfileName(const std::string& profile);

/**
 * Get list of supported profile names
 * 
 * @return String with comma-separated profile names
 */
std::string getSupportedProfiles();

}  // namespace ramp
}  // namespace fractional_teleoperation

#endif  // FRACTIONAL_TELEOPERATION_RAMP_PROFILE_HPP
