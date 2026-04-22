#include "fractional_teleoperation/ramp_profile.hpp"
#include <algorithm>
#include <sstream>

namespace fractional_teleoperation
{
namespace ramp
{

double rampLinear(double t_ratio)
{
  return std::min(1.0, std::max(0.0, t_ratio));
}

double rampQuadratic(double t_ratio)
{
  t_ratio = std::min(1.0, std::max(0.0, t_ratio));
  return t_ratio * t_ratio;
}

double rampSmoothstep(double t_ratio)
{
  t_ratio = std::min(1.0, std::max(0.0, t_ratio));
  return t_ratio * t_ratio * (3.0 - 2.0 * t_ratio);
}

double rampSigmoid(double t_ratio)
{
  const double sigmoid_input = 10.0 * (t_ratio - 0.5);
  return 1.0 / (1.0 + std::exp(-sigmoid_input));
}

double rampExponential(double t_ratio)
{
  t_ratio = std::min(1.0, std::max(0.0, t_ratio));
  return 1.0 - std::exp(-5.0 * t_ratio);
}

double rampInverseExponential(double t_ratio)
{
  t_ratio = std::min(1.0, std::max(0.0, t_ratio));
  const double one_minus_t = 1.0 - t_ratio;
  return 1.0 - (one_minus_t * one_minus_t * one_minus_t);
}

double rampTanh(double t_ratio)
{
  const double tanh_input = 6.0 * (t_ratio - 0.5);
  return (std::tanh(tanh_input) + 1.0) / 2.0;
}

double rampCubicHermite(double t_ratio)
{
  t_ratio = std::min(1.0, std::max(0.0, t_ratio));
  return 3.0 * t_ratio * t_ratio - 2.0 * t_ratio * t_ratio * t_ratio;
}

double rampStep(double t_ratio)
{
  return (t_ratio >= 1.0) ? 1.0 : 0.0;
}

std::string validateProfileName(const std::string& profile)
{
  std::string normalized = profile;
  // Convert to lowercase
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  
  // Check if valid
  if (normalized != "linear" && normalized != "quadratic" && normalized != "smoothstep" &&
      normalized != "sigmoid" && normalized != "exponential" && 
      normalized != "inverse_exponential" && normalized != "tanh" && 
      normalized != "cubic_hermite" && normalized != "step")
  {
    throw std::invalid_argument(
        "Unknown ramp profile: '" + profile + "'. Supported: " + getSupportedProfiles());
  }
  
  return normalized;
}

std::string getSupportedProfiles()
{
  return "linear, quadratic, smoothstep, sigmoid, exponential, inverse_exponential, tanh, cubic_hermite, step";
}

double computeRampFactor(double t_ratio, const std::string& profile)
{
  std::string normalized_profile = validateProfileName(profile);
  
  if (normalized_profile == "linear") {
    return rampLinear(t_ratio);
  } else if (normalized_profile == "quadratic") {
    return rampQuadratic(t_ratio);
  } else if (normalized_profile == "smoothstep") {
    return rampSmoothstep(t_ratio);
  } else if (normalized_profile == "sigmoid") {
    return rampSigmoid(t_ratio);
  } else if (normalized_profile == "exponential") {
    return rampExponential(t_ratio);
  } else if (normalized_profile == "inverse_exponential") {
    return rampInverseExponential(t_ratio);
  } else if (normalized_profile == "tanh") {
    return rampTanh(t_ratio);
  } else if (normalized_profile == "cubic_hermite") {
    return rampCubicHermite(t_ratio);
  } else if (normalized_profile == "step") {
    return rampStep(t_ratio);
  }
  
  // Should not reach here due to validateProfileName check
  throw std::invalid_argument("Unhandled ramp profile: " + profile);
}

}  // namespace ramp
}  // namespace fractional_teleoperation
